// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// T-11 program cache for the games whose maincpu region cannot live in M10K -- 720 and
// APB each need 13 of the 0x8000 windows, against the 8-window compaction
// `sys2_maincpu_rom` provides). Drop-in for that module's CPU-side contract, backed by
// SDRAM instead of BRAM.
//
// ---------------------------------------------------------------------------
// Why a cache, and why this shape -- all of it measured, none of it estimated
//
// The design chose between "bank-aware arbitration" and "64 MHz" on a worst-case CPU grant
// latency of 469 ns. That number was stale (the arbiter measures 40 clk = 1250 ns as built --
// two-deep requests deliberately traded CPU latency for playfield throughput), and both
// options attack the same thing: the cost of an access. The measurement says attack the
// COUNT instead. Measured over a real fetch stream (csprint, 741,001 fetches):
//
//   9,750 T-11 ROM fetches per frame, 75.9 % of them sequential, 9.4 KiB footprint
//   no cache                        34.7 % of a frame typical, 85.9 % worst
//   2 KiB / 8-word lines / direct   99.5 % hit -> 3.9 % / 4.2 %   (BRAM: 3.7 %)
//
// The CPU is the one client on this controller whose accesses are sequential, which is
// exactly the shape the cost model rewards -- short scattered accesses are why only 13.3 %
// of controller cycles carry data.
//
// The real risk is CONTENTION, not CPU speed. 9,750 fetches / 416 lines is ~23 CPU reads
// per scanline at the arbiter's highest priority, preempting the playfield fill, against a
// measured break point of 65-70 extra reads/line that motion objects already share. And the
// T-11 spends ~83 % of its time in a frame-tick spin loop (csprint 0xb854-0xb85c), so most
// of that traffic is a CPU waiting for vblank while starving the line fill -- which shows up
// as the stale-buffer scanlines already seen on ssprint's level-select screen. At 99.5 % hit
// this becomes ~0.12 line fills per scanline. The spin loop is the most cacheable code in
// the game.
//
// Those hit rates are a PROXY: 720 and APB cannot boot in the simulation that produced them
// (they read 0xffff from the very windows this fixes), so the stream is a Sprint's. The
// conclusion does not depend on them being right -- the model's sensitivity sweep shows the
// design only starts crowding the video clients below ~75 % hit. Two orders of margin.
//
// ---------------------------------------------------------------------------
// Burst fills through `c0`, and why the arbiter's rules are not touched
//
// A line fill is issued as ONE LINE_WORDS burst through the arbiter's highest-priority
// CPU client `c0` (see S_FILL below), which carries a burst length for exactly this use.
// The arbiter's ordering rules stay exactly as proven on silicon -- no re-ranking. Either
// fill shape is cheap at this hit rate:
//
//   misses/frame = 9,750 x 0.5 %  = ~49
//   8 single-word reads each      = ~2,730 clk/frame = 0.5 % of a frame
//   8-word burst each             = ~1,170 clk/frame = 0.22 %
//
// The fill was originally the single-word variant on exactly this frame-level costing; it
// moved to the burst when line-level measurement showed the frame total was the wrong
// metric -- a burst amortises its round trip over the whole line, so long lines stop
// blowing the per-scanline budget (see the S_FILL note below). The 0.3 % frame-level
// difference was never a reason to touch a silicon-proven arbiter whose ordering rules
// have their own recorded failure history (the pre-grant attempt that delivered one
// client's two bursts out of order).
//
// Per-line contention also lands where it should: 49 misses x 8 words = ~390 fill
// words per FRAME, i.e. ~0.94 per scanline, against the 65-70 break point.
//
// ---------------------------------------------------------------------------
// Tagged on the region address -- the load-bearing detail
//
// `rom_addr` here is the 20-bit maincpu REGION byte address: the output of
// `sys2_main_bus.decode_rom_bank`, window index already applied. Tagging on that means a
// bankselect write changes the TAG rather than invalidating anything, so there is no flush
// and no coherence problem. Tagging the CPU-side address instead would need a flush on every
// bank switch -- which works in simulation and fails on a game that switches banks inside an
// ISR, i.e. the failure that only appears on hardware.
//
// The region is ROM. Nothing writes it after the download, so the only invalidation that
// exists is reset.
//
// Held until `mem_ready`. Every SDRAM client in this core must be, and two have been
// caught not being (the mob engine in June, the tile prefetcher in August): plain `reset`
// drops BEFORE the download finishes, so a client gated only on it contends with the loader
// and wedges the HPS download -- a black screen on every game, and latent until whatever
// else was broken got fixed. `mem_ready`
// also covers correctness here: the SDRAM contents are not the ROM until the download ends,
// so a fill before that would cache garbage permanently.
// ---------------------------------------------------------------------------
module sys2_cpu_icache #(
	parameter int AW          = 24,     // SDRAM word-address width
	parameter int REGION_BASE = 0,      // word base of the maincpu region (sys2_rom_layout.vh)
	parameter int LINE_WORDS  = 8,      // words per line   (measured sweet spot; see header)
	parameter int LINES       = 128     // lines -> 128 x 8 x 16b = 2 KiB = ~2 M10K
) (
	input  logic        clk,
	input  logic        reset,
	// Held low until the ROM download has finished. Gates every SDRAM access AND every ack.
	input  logic        mem_ready,

	// ---- CPU side: byte-for-byte sys2_maincpu_rom's contract ----
	// rom_req is held with a stable rom_addr until rom_ack; rom_ack is a LEVEL, not a pulse.
	input  logic        rom_req,
	input  logic [19:0] rom_addr,       // maincpu region BYTE address, post-bank-decode
	output logic [15:0] rom_rdata,
	output logic        rom_ack,

	// ---- SDRAM side: the arbiter's burst client 0 ----
	output logic          sd_req,
	output logic [AW-1:0] sd_addr,
	// Burst length. sd_addr is the line base and stays CONSTANT for the whole fill -- the
	// arbiter identifies a head by {addr,len} and a moving address looks like a new request.
	output logic [8:0]    sd_len,
	input  logic          sd_valid,
	input  logic [15:0]   sd_rdata
);

localparam int OFF_BITS = $clog2(LINE_WORDS);   // 3
localparam int IDX_BITS = $clog2(LINES);        // 7
localparam int WORDS    = LINES * LINE_WORDS;   // 1024
localparam int TAG_HI   = 18;                   // word address is rom_addr[19:1]
localparam int TAG_LO   = OFF_BITS + IDX_BITS;  // 10
localparam int TAG_BITS = TAG_HI - TAG_LO + 1;  // 9
// fill_n counts 0..LINE_WORDS so it is OFF_BITS+1 wide; comparing it against an
// OFF_BITS-wide cast is a width mismatch that iverilog accepts silently and Verilator does
// not. Name the constant at the counter's width instead of casting at the comparison.
localparam logic [OFF_BITS:0] FILL_LAST = (OFF_BITS+1)'(LINE_WORDS - 1);

// Word address of the request. rom_addr[0] is always 0 (the bus reads words).
wire [18:0]          w_addr = rom_addr[19:1];
wire [OFF_BITS-1:0]  req_off = w_addr[OFF_BITS-1:0];
wire [IDX_BITS-1:0]  req_idx = w_addr[OFF_BITS+IDX_BITS-1:OFF_BITS];
wire [TAG_BITS-1:0]  req_tag = w_addr[TAG_HI:TAG_LO];

// ---- tag store in registers ----
// 128 x 10 bits is small enough to keep out of M10K, and a combinational tag read means the
// hit decision lands on the same cycle the data read returns: 2 clocks to ack, matching what
// sys2_maincpu_rom already costs the CPU. A BRAM tag would add a cycle to EVERY fetch to
// save nothing that is in short supply.
logic [TAG_BITS-1:0] tag_q   [0:LINES-1];
logic                valid_q [0:LINES-1];

// ---- data store: inferred simple dual-port M10K ----
// One 16-bit array, written whole words only. (sys2_maincpu_rom has to split into byte
// lanes because the LOADER writes it a byte at a time and Quartus will not infer that as
// BRAM -- it becomes a 1.3 Mbit logic decoder. Fills here are always whole words, so the
// split is unnecessary.)
logic [15:0] data_q [0:WORDS-1];

typedef enum logic [1:0] { S_IDLE, S_EVAL, S_FILL, S_SERVE } state_t;
state_t state;

logic [IDX_BITS-1:0] idx_r;
logic [TAG_BITS-1:0] tag_r;
logic [OFF_BITS-1:0] off_r;
logic [OFF_BITS:0]   fill_n;         // 0..LINE_WORDS
logic [15:0]         rd_q;           // registered data-array read
// A completed fill returns through S_EVAL to re-read the requested word. That pass must not
// be counted as a hit -- it would add exactly one phantom hit per miss, which is a small
// error at 99.5 % but a systematically flattering one, and an instrument that flatters the
// thing it measures is how this project has been fooled before.
logic                refill_r;

wire [IDX_BITS+OFF_BITS-1:0] rd_index = {idx_r, off_r};
wire hit_now = valid_q[idx_r] && (tag_q[idx_r] == tag_r);

// Fill address: the line's BASE word. Constant for the whole burst (was `+ 19'(fill_n)`
// while the fill was LINE_WORDS separate single-word reads). fill_n still selects the
// data-array write slot as words land.
wire [18:0] fill_base = {tag_r, idx_r, {OFF_BITS{1'b0}}};

integer i;
always_ff @(posedge clk) begin
	if (reset) begin
		state  <= S_IDLE;
		sd_req   <= 1'b0;
		fill_n   <= '0;
		refill_r <= 1'b0;
		for (i = 0; i < LINES; i = i + 1) valid_q[i] <= 1'b0;
	end else begin
		// Registered read of the data array, one port, address chosen by the state below.
		rd_q <= data_q[rd_index];

		unique case (state)
		S_IDLE: begin
			sd_req <= 1'b0;
			// mem_ready gates the whole engine: before the download ends the SDRAM does not
			// hold the ROM yet, and the CPU is held in reset anyway.
			if (rom_req && mem_ready) begin
				idx_r    <= req_idx;
				tag_r    <= req_tag;
				off_r    <= req_off;
				refill_r <= 1'b0;
				state    <= S_EVAL;
			end
		end

		S_EVAL: begin
			// rd_q now holds data_q[{idx_r, off_r}] and hit_now is valid (combinational tags).
			if (hit_now) begin
				state <= S_SERVE;
			end else begin
				// A refill that still misses is impossible: the tag was just written from
				// tag_r. If it ever happens the line would refill forever, so say so.
				// synthesis translate_off
				if (refill_r) $fatal(1, "sys2_cpu_icache: line missed immediately after its own fill");
				// synthesis translate_on
				fill_n   <= '0;
				refill_r <= 1'b1;
				sd_req   <= 1'b1;
				state    <= S_FILL;
			end
		end

		S_FILL: begin
			// One burst of LINE_WORDS, sd_req held with a stable addr/len until the last word
			// lands -- exactly the arbiter's head contract. fill_n counts DELIVERED words.
			//
			// WAS LINE_WORDS single-word reads at first, and the header's original
			// costing of burst-vs-single at "0.22 % vs 0.5 % of a FRAME" was the defect: a frame average
			// measured against the playfield fill's per-line deadline. With single words the
			// fill fences MO/PF (`can_hand && !c0_req`) for one round trip per word; the fill
			// overruns past ~24 CPU words on a scanline. As one burst it is one round trip.
			//
			// (historical, from the single-word era -- the modelling lesson below still stands)
			//
			// This relies on the memory NOT accepting a new request in the same cycle it
			// delivers a word -- because `sd_addr` is combinational from `fill_n`, which only
			// advances at the end of that cycle. That separation is structural on both sides
			// of this interface, not a coincidence to be lucky about:
			//   * sys2_sdram leaves S_CL to S_RECOV (tRP) before S_IDLE, and
			//     `ready = (state == S_IDLE) && ...`, so it is several cycles busy after valid
			//   * sys2_sdram_arb grants only when `owner == OWN_NONE`, and `owner` still reads
			//     OWN_CPU on the valid cycle -- it must, since `rdata` is shared and routed by
			//     owner. An arbiter that granted there would misroute the word it is delivering.
			//
			// An early simulation modelled a memory that re-grants on the delivery cycle
			// and "found" a duplicate read of the line's first word. The fix attempted --
			// dropping req for a cycle between words -- did not even prevent it (the duplicate
			// is at the delivery boundary, which a registered signal cannot cover) and cost a
			// cycle per word. The simulation was wrong, not the design. What survives from it is
			// an address-sequence assertion, which is what turned "9 reads" into "grant #1
			// repeated word 128" and made that diagnosable at all.
			if (sd_valid) begin
				data_q[{idx_r, fill_n[OFF_BITS-1:0]}] <= sd_rdata;
				if (fill_n == FILL_LAST) begin
					// Line complete; publish the tag now rather than at the start of the fill.
					//
					// This ordering is DEFENSIVE, not load-bearing, and the earlier comment
					// here claiming "an interrupt taken mid-fill would read an unfetched word"
					// was wrong. The CPU contract allows exactly ONE outstanding request --
					// rom_req is held until rom_ack -- so nothing can look at this line while
					// it fills, and an interrupt cannot start a fetch until the current one
					// retires. Simulation proves the point by mutation: publishing the tag at
					// the start of the fill survives the whole gate. Kept anyway, because it
					// costs nothing and stops being safe the moment anything gains a second
					// requester or a non-blocking fetch.
					sd_req         <= 1'b0;
					valid_q[idx_r] <= 1'b1;
					tag_q[idx_r]   <= tag_r;
					fill_n         <= '0;
					state          <= S_EVAL;   // re-read: rd_q is refreshed on the way round
				end else begin
					fill_n <= fill_n + 1'b1;
				end
			end
		end

		S_SERVE: begin
			// rom_ack is a level held while the request stands (the T-11 runs on a clock
			// enable and can miss a pulse between its active edges). Drop back only when the
			// CPU releases the request, so one req can never be served twice.
			if (!rom_req) state <= S_IDLE;
		end

		default: state <= S_IDLE;
		endcase
	end
end

// The data array has ONE read address: the requested word. A completed fill returns through
// S_EVAL, whose pass round the loop re-issues data_q[{idx_r, off_r}] with the fill's writes
// already landed -- so no second read port and no read/write bypass is needed.
assign rom_rdata = rd_q;
assign rom_ack   = (state == S_SERVE) && rom_req;
assign sd_addr   = AW'(REGION_BASE) + AW'(fill_base);
assign sd_len    = 9'(LINE_WORDS);

endmodule
