// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Single direct-to-controller SDRAM writer for all ROM regions during the download.
//
// Replaces the per-adapter write paths that went through the old multi-port SDRAM arbiter. On silicon
// Quartus pruned every NON-port-0 arbiter write client ("Lost fanout"): maincpu/port-0 wrote,
// but sprite/port-1 and audio/port-2 writes were optimized away, so those regions never loaded
// (black-screen audio + junk graphics, on hardware). A focused
// sim proved the per-adapter write LOGIC is correct, so it is purely a synthesis prune that
// simulation cannot model. The one SDRAM write path Quartus never prunes is the DIRECT-to-
// controller path (green every build). Since the CPUs are held
// during the download there is no read contention, so one writer suffices. The arbiter has since
// been removed entirely: this writer is the single write path into SDRAM, and the top level
// drives ld_region from the stream decode with each SDRAM-bound region in turn (maincpu, tiles,
// sprite).
//
// Live regions and their write-side address mappings -- all three flat, no compaction:
//   maincpu (1): base BASE_MAINCPU, phys addr[19:1]   (720/APB read it, all games write it)
//   tiles   (3): base BASE_TILES,   phys addr[18:1]
//   sprite  (4): base BASE_SPRITE,  phys addr[19:1]   (1 MiB)
//   chars (5) and audio (2) are on-chip BRAMs loaded through the raw-ioctl decodes and never
//   reach this writer.
//
// maincpu is written for every game, not just the two that read it from SDRAM. It has to be:
// the MRA's index-1 game descriptor arrives AFTER the whole index-0 ROM stream, so `game_id` is
// 0 for the entire load and nothing in the load path may consult it (that law was learned twice
// on hardware). The per-game choice is made on
// the READ side, where the descriptor has long since landed: paperboy/ssprint/csprint read the
// BRAM window compaction, 720/APB read this copy through sys2_cpu_icache. Both get written; one
// gets read. The cost is download time and 576 KiB of a 32 MiB device.
//
// THE maincpu AND audio branches were deleted, and the reason matters.
// They carried their OWN copy of `win_pop`/`win_idx` -- and it was still the FIVE-window
// Paperboy map {1,2,6,10,14} long after `sys2_maincpu_rom` had moved to EIGHT
// {1,2,3,6,10,11,14,15}. Dead code, so nothing caught it and no gate covered it. Taking the
// T-11 program from SDRAM is precisely the change that re-activates this path, and it
// would have placed T-11 bytes with a three-windows-stale map: the ssprint dot-grid failure
// shape exactly (a wrong ADDRESS decode, invisible to every byte-level check).
//
// So the branches were deleted rather than updated, and maincpu came back as a
// FLAT mapping with no window map at all. That is the whole point: the compaction exists solely
// to save M10K, which SDRAM is not short of, and storing flat also disposes of the open
// problem that `win_pop`/`win_idx` are per-game data and so cannot be a hardcoded table here.
// One mapping, one place, and never a second copy of it. A region this writer does not know
// still writes NOTHING, which fails loudly (an all-fill region -- the game cannot boot) rather
// than quietly serving the wrong banks.
//
// All in clk_sys (the loader + controller domain); no CDC. Drop-in backpressure: wr_busy is
// combinational (falls mem_ready the same cycle a word-completing byte arrives) like the
// adapters, so the loader's drain never outruns the writes.
module sys2_sdram_loader_writer #(
	parameter int SDRAM_AW     = 24,
	// No defaults: a base is the address map's business, not this module's. A default here is a
	// SECOND place the number lives, which is how the write and read views of a region drift
	// apart. sys2_rom_layout.vh owns all three.
	parameter int BASE_MAINCPU,
	parameter int BASE_SPRITE,
	// No default here either, for the same reason -- see sys2_rom_layout.vh, which lays the
	// regions out around this one (tiles keeps the base that is proven on silicon).
	parameter int BASE_TILES
) (
	input  logic        clk,          // clk_sys
	input  logic        reset,
	// Loader write stream (rom_we/region/addr/data), one byte per strobe, even offset = low byte.
	input  logic        ld_we,
	input  logic [2:0]  ld_region,
	input  logic [19:0] ld_addr,
	input  logic [7:0]  ld_data,
	output logic        wr_busy,
	// SDRAM controller host port (write-only; muxed onto the controller during the download).
	output logic [SDRAM_AW-1:0] sdram_addr,
	output logic [15:0]         sdram_wdata,
	output logic                sdram_we,
	output logic                sdram_req,
	input  logic                sdram_ready
);

localparam logic [2:0] R_MAINCPU = 3'd1, R_SPRITE = 3'd4, R_TILES = 3'd3;

// ---- combinational decode of the current loader byte ----
wire is_main   = (ld_region == R_MAINCPU);
wire is_sprite = (ld_region == R_SPRITE);
wire is_tiles  = (ld_region == R_TILES);
wire is_sdram  = is_main | is_sprite | is_tiles;   // chars and audio are BRAM (raw-ioctl)

// A word-completing (odd) byte that lands in an SDRAM region. All three live regions are
// stored FLAT, so there is no populated/hole test to get wrong.
wire word_pop  = ld_we && is_sdram && ld_addr[0];

// Physical word address (region base + region phys) for the current byte.
// 1 MiB region -> 0x80000 words, 19 bits. (Was 17 for the 256 KiB allocation.)
wire [18:0] sprite_phys = ld_addr[19:1];
// Tiles: 0x80000 bytes -> 0x40000 words, so 18 bits of word address.
wire [17:0] tiles_phys  = ld_addr[18:1];
// maincpu: 0x90000 bytes -> 0x48000 words, so 19 bits. FLAT -- ld_addr IS the region offset.
wire [18:0] main_phys   = ld_addr[19:1];
wire [SDRAM_AW-1:0] word_addr =
       is_main  ? (SDRAM_AW'(BASE_MAINCPU) + {{(SDRAM_AW-19){1'b0}}, main_phys})   :
       is_tiles ? (SDRAM_AW'(BASE_TILES)   + {{(SDRAM_AW-18){1'b0}}, tiles_phys})  :
                  (SDRAM_AW'(BASE_SPRITE)  + {{(SDRAM_AW-19){1'b0}}, sprite_phys});

logic [7:0]          lo_buf;
logic [SDRAM_AW-1:0] w_addr;

/* verilator lint_off PROCASSINIT */
typedef enum logic [1:0] {M_IDLE, M_WREQ, M_WBUSY} mst_t;
mst_t mst = M_IDLE;
/* verilator lint_on PROCASSINIT */

assign wr_busy = (mst == M_WREQ) || (mst == M_WBUSY) || (mst == M_IDLE && word_pop);

always_ff @(posedge clk) begin
	sdram_req <= 1'b0;                       // default: no request
	if (reset) begin
		mst <= M_IDLE;
	end else begin
		unique case (mst)
		M_IDLE: begin
			if (ld_we && is_sdram) begin
				if (!ld_addr[0]) begin
					lo_buf <= ld_data;          // even byte -> low half
				end else if (word_pop) begin
					w_addr      <= word_addr;
					sdram_wdata <= {ld_data, lo_buf};
					mst         <= M_WREQ;
				end
				// odd byte in a hole / unpopulated window: drop.
			end
		end
		M_WREQ: begin
			sdram_addr <= w_addr;
			sdram_we   <= 1'b1;
			sdram_req  <= 1'b1;
			if (sdram_req && sdram_ready) begin sdram_req <= 1'b0; mst <= M_WBUSY; end
		end
		M_WBUSY: if (sdram_ready) mst <= M_IDLE;
		default: mst <= M_IDLE;
		endcase
	end
end

endmodule
