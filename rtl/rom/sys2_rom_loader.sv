// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Atari System 2 ROM download router. The MiSTer MRA emits the six MAME logical
// regions as one ordered index-0 byte stream (see rtl/rom/sys2_rom_layout.vh); index 1
// carries the four-byte game descriptor. This module decodes the fixed stream
// boundaries, applies the only load transform (sprite-region XOR 0xff), and
// emits one registered write per accepted byte to the selected target memory.
// The even/odd T-11 interleave is already baked into the stream, so no
// post-transform is needed there.
//
// Backpressure model: a small write FIFO decouples the HPS byte rate from the
// SDRAM write latency. The HPS download loop blocks ENTIRELY while ioctl_wait
// is high (it polls HPS_BUS[37]); driving ioctl_wait from the per-write SDRAM
// busy therefore charged a full HPS poll round-trip to every single byte and,
// worse, any genuinely stuck write could only be "escaped" by dropping bytes
// (the old anti-wedge watchdog). Instead, the HPS pushes one decoded
// {region,offset,data} entry per accepted byte at full speed into the FIFO, and
// a drain process pops them into the target memory paced by mem_ready.
// ioctl_wait asserts ONLY when the FIFO is almost full -- a margin below
// capacity so bytes already in HPS flight cannot overrun it -- which on a load
// the SDRAM keeps up with never happens. The FIFO cannot wedge: every SDRAM
// write completes on its own timer, so the drain always makes forward progress
// and almost_full always clears. This matches the reference core's behaviour
// (stream at full speed, wait only on a real backlog) without a watchdog.
//
// Physical storage (BRAM vs SDRAM, width, compaction of the sparse main/audio
// regions) is the integrator's choice; this router only produces the
// region/offset/data/strobe and the load status, keeping the placement
// observationally identical to the frozen logical image.
module sys2_rom_loader (
	input  logic        clk,
	input  logic        reset,

	// MiSTer hps_io download interface (8-bit, non-WIDE).
	input  logic        ioctl_download,
	input  logic [15:0] ioctl_index,
	input  logic        ioctl_wr,
	input  logic [26:0] ioctl_addr,
	input  logic [7:0]  ioctl_dout,
	output logic        ioctl_wait,

	// Downstream memory backpressure: while low the target memory is busy and
	// the FIFO drain stalls (the host is NOT stalled until the FIFO fills).
	input  logic        mem_ready,

	// Target write strobe (one registered pulse per drained byte).
	output logic        rom_we,
	output logic [2:0]  rom_region,    // REGION_* below
	output logic [19:0] rom_addr,      // byte offset within the selected region
	output logic [7:0]  rom_data,      // sprite region pre-inverted

	// (The dedicated per-region BRAM write strobes were removed -- all ROMs load via
	// top-level raw-ioctl decodes now.)

	// Index-1 game descriptor (4 bytes; see rtl/rom/sys2_rom_layout.vh).
	output logic [7:0]  slapstic_type,   // byte 0
	output logic [7:0]  game_id,         // byte 1
	output logic [7:0]  game_flags,      // byte 2
	output logic        descriptor_ok,   // all 4 bytes present and self-consistent
	output logic        descriptor_bad,  // descriptor complete but rejected

	output logic        rom_loaded     // full index-0 stream accepted AND drained
);

// Region codes.
localparam logic [2:0] REGION_NONE     = 3'd0;
localparam logic [2:0] REGION_MAINCPU  = 3'd1;
localparam logic [2:0] REGION_AUDIOCPU = 3'd2;
localparam logic [2:0] REGION_TILES    = 3'd3;
localparam logic [2:0] REGION_SPRITES  = 3'd4;
localparam logic [2:0] REGION_CHARS    = 3'd5;
localparam logic [2:0] REGION_EEPROM   = 3'd6;

// Index-0 stream boundaries (byte offsets). Single source of truth:
`include "rtl/rom/sys2_rom_layout.vh"
localparam logic [26:0] BASE_AUDIO  = SYS2_BASE_AUDIO;
localparam logic [26:0] BASE_TILES  = SYS2_BASE_TILES;
localparam logic [26:0] BASE_SPRITE = SYS2_BASE_SPRITE;
localparam logic [26:0] BASE_CHARS  = SYS2_BASE_CHARS;
localparam logic [26:0] BASE_EEPROM = SYS2_BASE_EEPROM;
localparam logic [26:0] END_STREAM  = SYS2_END_STREAM;

logic index0;
assign index0 = ioctl_download && (ioctl_index == 16'd0);

// Combinational region decode + transform for the current ioctl byte.
logic [2:0]  region_c;
logic [19:0] off_c;          // offset within the selected region (max 0x8ffff)
logic [7:0]  data_c;

always @* begin
	region_c = REGION_NONE;
	off_c    = 20'd0;
	data_c   = ioctl_dout;
	if (ioctl_addr < BASE_AUDIO) begin
		region_c = REGION_MAINCPU;
		off_c    = 20'(ioctl_addr);
	end else if (ioctl_addr < BASE_TILES) begin
		region_c = REGION_AUDIOCPU;
		off_c    = 20'(ioctl_addr - BASE_AUDIO);
	end else if (ioctl_addr < BASE_SPRITE) begin
		region_c = REGION_TILES;
		off_c    = 20'(ioctl_addr - BASE_TILES);
	end else if (ioctl_addr < BASE_CHARS) begin
		region_c = REGION_SPRITES;
		off_c    = 20'(ioctl_addr - BASE_SPRITE);
		data_c   = ~ioctl_dout;            // the only load transform
	end else if (ioctl_addr < BASE_EEPROM) begin
		region_c = REGION_CHARS;
		off_c    = 20'(ioctl_addr - BASE_CHARS);
	end else if (ioctl_addr < END_STREAM) begin
		region_c = REGION_EEPROM;
		off_c    = 20'(ioctl_addr - BASE_EEPROM);
	end
end

// ============================ write FIFO ============================
// Entry = {region[2:0], offset[19:0], data[7:0]} (31 bits). The pointers carry
// one extra MSB so a full FIFO (wr - rd == DEPTH) is distinguishable from empty
// (wr == rd). AF_MARGIN free entries are held in reserve before ioctl_wait
// asserts, covering bytes already in flight on HPS_BUS when the host samples it.
localparam int FIFO_AW    = 7;                 // 128 entries
localparam int FIFO_DEPTH = 1 << FIFO_AW;
localparam int FIFO_W     = 31;                // {region[2:0], off[19:0], data[7:0]}
localparam int AF_MARGIN  = 32;                // free entries held back before ioctl_wait

localparam int CW = FIFO_AW + 1;               // FIFO count/pointer width (bits)

logic [FIFO_W-1:0] fifo_mem [0:FIFO_DEPTH-1];
logic [CW-1:0]     wr_ptr;
logic [CW-1:0]     rd_ptr;
wire  [CW-1:0]     fifo_count  = wr_ptr - rd_ptr;
wire               fifo_empty  = (wr_ptr == rd_ptr);
wire               fifo_full   = (fifo_count == CW'(FIFO_DEPTH));
wire               almost_full = (fifo_count >= CW'(FIFO_DEPTH - AF_MARGIN));
wire [FIFO_W-1:0]  fifo_dout   = fifo_mem[rd_ptr[FIFO_AW-1:0]];

// Push every accepted index-0 byte; drain when the target memory can take one.
// pop is gated only by mem_ready, so an even (low-byte) drain and its following
// odd (word-completing) drain issue back-to-back, then the drain pauses for the
// SDRAM write -- mem_ready falls the same cycle the odd rom_we is presented.
wire push = index0 && ioctl_wr && (region_c != REGION_NONE) && !fifo_full;
wire pop  = !fifo_empty && mem_ready;

// The host only ever waits on a real backlog; the FIFO always drains, so this
// can never stick high (no watchdog needed).
assign ioctl_wait = almost_full;

// ---- push side (reset clears the FIFO; reset is inactive during a download) ----
always_ff @(posedge clk) begin
	if (reset) begin
		wr_ptr <= '0;
	end else if (push) begin
		fifo_mem[wr_ptr[FIFO_AW-1:0]] <= {region_c, off_c, data_c};
		wr_ptr <= wr_ptr + 1'b1;
	end
end

// ---- drain side: one registered write per popped byte ----
// The dedicated per-region strobes decode fifo_dout's region HERE (at the source, registered)
// so far BRAM consumers get a clean single-bit enable with no remote rom_region decode.
wire [2:0] drain_region = fifo_dout[30:28];
always_ff @(posedge clk) begin
	if (reset) begin
		rd_ptr <= '0;
		rom_we <= 1'b0;
	end else begin
		rom_we <= 1'b0;
		if (pop) begin
			rom_we       <= 1'b1;
			rom_region   <= drain_region;
			rom_addr     <= fifo_dout[27:8];
			rom_data     <= fifo_dout[7:0];
			rd_ptr       <= rd_ptr + 1'b1;
		end
	end
end

// Load-complete status, derived from the index-0 download envelope (NOT a fixed
// final address): cleared when the stream starts, set when the stream has ended
// AND the write FIFO has fully drained into memory (fifo_empty, no write pulse
// in flight, target idle). Latching on drain-complete -- not merely on the
// download's falling edge -- guarantees the integrator's CPU is released only
// after the very last byte has actually landed, for ANY stream length.
//
// This trio is deliberately OUTSIDE the `reset` domain. The downloaded image
// lives in BRAM/SDRAM and survives a game/framework reset, so "loaded" must
// too. Tying it to `reset` let a reset pulsing at/after the download (the
// MiSTer post-download clean-start reset) wipe rom_loaded with no re-load to
// follow, stranding the CPU in reset on a permanent black screen. It is cleared
// only by a NEW index-0 download starting; the power-on value 0 holds the CPU
// in reset until the first download finishes.
// Quartus honors the initializer as the configured FF state on Intel FPGAs.
// PROCASSINIT fires because there is both an initializer and a clocked write --
// exactly the power-on-then-update behavior we want here.
/* verilator lint_off PROCASSINIT */
logic index0_d  = 1'b0;
logic loaded    = 1'b0;
logic dl_active = 1'b0;   // a download has begun and not yet been declared loaded
/* verilator lint_on PROCASSINIT */

always_ff @(posedge clk) begin
	index0_d <= index0;
	if (index0 && !index0_d) begin
		loaded    <= 1'b0;   // index-0 stream started
		dl_active <= 1'b1;
	end else if (dl_active && !index0 && fifo_empty && !rom_we && mem_ready) begin
		loaded    <= 1'b1;   // stream ended AND the FIFO has fully drained
		dl_active <= 1'b0;
	end
end
assign rom_loaded = loaded;

// ---------------------------------------------------------------------------
// Index-1 game descriptor capture. Layout and validation: rtl/rom/sys2_rom_layout.vh.
//
// Outside the `reset` DOMAIN, for exactly the reason rom_loaded is (see the trio above).
// Leaving it inside is a long-running source of trouble: the descriptor arrives while
// rom_loaded has already been moved out, so the reset pulse wipes it.
//
// The board's sequence is: index-0 stream, THEN the index-1 descriptor, THEN **MiSTer's
// post-download clean-start reset pulse**. With the capture gated on `reset`, that pulse wiped
// `desc_seen` and `descriptor_ok` after a perfectly good descriptor had been received, and
// nothing ever re-sent it. `rom_loaded` survived (it is outside the domain), so the core sat
// at rom_loaded=1 / descriptor_ok=0 / descriptor_bad=0 -- `rom_ready` low FOREVER, the T-11
// held in reset, a black screen on EVERY game, and a medium-blink LED saying "loaded but not
// validated". That is precisely what hardware reported: maxaddr 0x2241 (the whole image
// arrived), DLCOUNT 1, CPU-status row 1 red.
//
// Simulation cannot see this: no bench models the post-download reset pulse. The same trap
// is documented one screen up for rom_loaded, in this same file -- "stranding the CPU in reset
// on a permanent black screen".
//
// Cleared only by a NEW index-1 download starting, mirroring `loaded`. Power-on value 0 keeps
// the CPU held until a descriptor genuinely arrives.
// ---------------------------------------------------------------------------
/* verilator lint_off PROCASSINIT */
logic [3:0] desc_seen     = 4'd0;            // one bit per descriptor byte
logic [7:0] desc_reserved = 8'd0;
logic       index1_d      = 1'b0;
logic [7:0] desc_slap     = 8'd0;
logic [7:0] desc_game     = 8'd0;
logic [7:0] desc_flags    = 8'd0;
logic       desc_ok_r     = 1'b0;
logic       desc_bad_r    = 1'b0;
/* verilator lint_on PROCASSINIT */

wire index1 = ioctl_download && (ioctl_index == 16'd1);

assign slapstic_type  = desc_slap;
assign game_id        = desc_game;
assign game_flags     = desc_flags;
assign descriptor_ok  = desc_ok_r;
assign descriptor_bad = desc_bad_r;

// Expected slapstic type for the captured game id; 0 marks an unknown id.
//
// `always_comb`, NOT `always @*`. `game_id` now powers up at 0 from an initializer and, for
// Paperboy, never changes -- so an `always @*` has no edge to trigger on and leaves this X for
// the whole simulation, which silently poisons desc_valid. `always_comb` is evaluated once at
// time zero by the LRM. (Synthesis was always fine; this is a simulation-visibility trap that
// only appeared once the descriptor registers left the reset domain and stopped transitioning.)
logic [7:0] expected_slapstic;
always_comb begin
	case (game_id)
		SYS2_GAME_PAPERBOY: expected_slapstic = SYS2_SLAPSTIC_PAPERBOY;
		SYS2_GAME_720:      expected_slapstic = SYS2_SLAPSTIC_720;
		SYS2_GAME_SSPRINT:  expected_slapstic = SYS2_SLAPSTIC_SSPRINT;
		SYS2_GAME_CSPRINT:  expected_slapstic = SYS2_SLAPSTIC_CSPRINT;
		SYS2_GAME_APB:      expected_slapstic = SYS2_SLAPSTIC_APB;
		default:            expected_slapstic = 8'd0;
	endcase
end

// Valid only once all four bytes have arrived and every field agrees. Anything
// else -- a short v1 descriptor, an unknown game, a reserved bit set, or a
// slapstic type that does not match the game id -- is a hard reject.
wire desc_complete = (desc_seen == 4'b1111);
wire desc_valid    = desc_complete
                  && (game_id <= SYS2_GAME_MAX)
                  && (slapstic_type == expected_slapstic)
                  && (game_flags[7:1] == 7'd0)
                  && (desc_reserved == 8'd0);

always_ff @(posedge clk) begin
	index1_d <= index1;
	if (index1) begin
		if (!index1_d) begin
			desc_seen     <= 4'd0;   // a NEW index-1 download begins: start a fresh capture
			desc_reserved <= 8'd0;
		end
		if (ioctl_wr && (ioctl_addr < 27'(SYS2_DESC_BYTES))) begin
			// Capture must WIN over the fresh-start clear in the same cycle. hps_io raises
			// ioctl_download a cycle or more before the first ioctl_wr, so they do not normally
			// coincide -- but an `else if` here would silently drop byte 0 if they ever did,
			// and a missing byte 0 reads as "descriptor rejected" with nothing to point at.
			desc_seen <= (index1_d ? desc_seen : 4'd0) | (4'd1 << ioctl_addr[1:0]);
			case (ioctl_addr[1:0])
				2'd0: desc_slap     <= ioctl_dout;
				2'd1: desc_game     <= ioctl_dout;
				2'd2: desc_flags    <= ioctl_dout;
				2'd3: desc_reserved <= ioctl_dout;
			endcase
		end
	end
end

// Registered so the CPU-hold / LED-refusal path sees a stable level, and so a
// partially-received descriptor never momentarily reads as accepted.
//
// `bad` deliberately covers the INCOMPLETE case (any descriptor byte seen but not
// a whole valid one), not just the complete-but-wrong case. An MRA still carrying
// the v1 single slapstic byte would otherwise leave both flags low, which reads
// on the status LED as "still loading" -- a silent refusal nobody can diagnose.
// Also outside `reset`: gating these would re-open the same hole one level down -- the
// capture would survive the post-download reset pulse but the flag derived from it would not.
always_ff @(posedge clk) begin
	desc_ok_r  <=  desc_valid;
	desc_bad_r <= (desc_seen != 4'd0) && !desc_valid;
end

endmodule
