// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Sound 6502 program ROM (0x4000-0xffff, 48 KiB) in on-chip BRAM -- the proven MiSTer/
// reference pattern (Arcade-Atari-system1 ap_srom0/1/2). Replaces the earlier SDRAM-backed
// audio ROM and its clk_t11<->clk_sys read CDC: the 48 KiB program fits in
// ~39 M10K, so it needs no SDRAM port, no read CDC, and no load FIFO. The low 0x4000 RAM/IO hole is not
// stored; index 0 = 6502 address 0x4000.
//
// LOAD (wr_clk = clk_sys, the HPS/loader domain): the top level decodes the audiocpu span of
// the index-0 ioctl stream (0x094000-0x09ffff) straight into ld_we, presenting the 6502-style
// address (ioctl_addr[15:0], i.e. 0x4000-0xffff) on ld_addr and the byte on ld_data -- exactly
// like the reference's `sl_wr_SROM* -> dpram` strobe, bypassing the loader FIFO entirely. One
// registered write per byte. This does NOT depend on the loader's region-2 drain bus.
//
// READ (clk = clk_t11, the 6502's clock): same contract the old SDRAM adapter presented to the
// 6502 (which samples its bus when its clock-enable fires). The 6502's
// snd_cpu_en is gated by data_ready, so it advances only when data_ready is high, at which
// point cpu_data is the byte at cpu_addr. A new cpu_addr drops data_ready for ONE clk while the
// registered M10K read settles, then raises it (1 clk << the /8 snd_cpu_en period, so the stall
// is invisible). cpu_hold (= snd_reset) holds the fetch invalid so a pre-load byte is never
// served while the 6502 is in reset during the download.
//
// The array is a dual-clock simple-dual-port M10K (write wr_clk, read clk) -- the same
// dual-clock dpram shape the video copy uses (sys2_dpram), which is proven on silicon.
module sys2_audio_rom_bram #(
	parameter int PROG_BYTES = 'hC000        // 48 KiB: 6502 0x4000..0xffff
) (
	// Load port (clk_sys).
	input  logic        wr_clk,
	input  logic        ld_we,
	input  logic [15:0] ld_addr,             // 6502-style address 0x4000..0xffff
	input  logic [7:0]  ld_data,

	// 6502 read port (clk_t11). Same data_ready/cpu_hold contract the 6502's read expects.
	input  logic        clk,
	input  logic        cpu_hold,            // = snd_reset: invalidate + suppress fetches
	input  logic [15:0] cpu_addr,            // 6502 address 0x4000..0xffff
	output logic [7:0]  cpu_data,            // held byte for cpu_addr
	output logic        data_ready           // high when cpu_data is valid for cpu_addr
);

localparam int AW = $clog2(PROG_BYTES);      // 16 bits for 0xC000

(* ramstyle = "no_rw_check, M10K" *) logic [7:0] rom [0:PROG_BYTES-1];

// ---- port A (clk_sys): the loader's write port ----
// One write port + the 6502 read port below = a simple dual-port M10K (no duplication).
wire [AW-1:0] wr_off = ld_addr[AW-1:0] - AW'(16'h4000);   // drop the 0x4000 base
always_ff @(posedge wr_clk) begin
	if (ld_we) rom[wr_off] <= ld_data;
end

// ---- 6502 (read) port: clk_t11, 1-clk registered M10K read with a hold-until-ready FSM ----
wire [AW-1:0] off = cpu_addr[AW-1:0] - AW'(16'h4000);

/* verilator lint_off PROCASSINIT */
logic         state = 1'b0;   // 0 = FETCH (read in flight), 1 = READY (cpu_data valid for cur_off)
logic [AW-1:0] cur_off = '0;
logic [7:0]   held = 8'd0;
/* verilator lint_on PROCASSINIT */

localparam logic S_FETCH = 1'b0, S_READY = 1'b1;

always_ff @(posedge clk) begin
	if (cpu_hold) begin
		state <= S_FETCH;                 // invalid while the 6502 is held in reset
	end else begin
		case (state)
			S_FETCH: begin
				held    <= rom[off];      // registered M10K read of the current address
				cur_off <= off;
				state   <= S_READY;
			end
			S_READY: if (off != cur_off) state <= S_FETCH;   // address changed -> refetch
			default: state <= S_FETCH;
		endcase
	end
end

assign data_ready = (state == S_READY) && (off == cur_off) && !cpu_hold;
assign cpu_data   = held;

endmodule
