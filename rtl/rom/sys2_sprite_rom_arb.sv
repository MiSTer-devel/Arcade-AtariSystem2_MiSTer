// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// ---------------------------------------------------------------------------
// Per-game sprite geometry (derived by measurement).
//
// `spr_big` selects between the two sprite-ROM sizes in this family. It is NOT cosmetic:
// both the code mask and the plane stride depend on it, and hard-coding either at the
// family maximum silently corrupts the three 256 KiB games.
//
//   paperboy / ssprint / csprint   0x40000 B   2048 tiles   code[10:0]   plane +0x10000 words
//   720 / apb                     0x100000 B   8192 tiles   code[12:0]   plane +0x40000 words
//
// The MO parameter field is 14 bits ((word0&7)<<11 | (word1&0x7ff)) but NO region here can
// be indexed by 14 -- masking to 11 or 13 is MAME's `tile_data::set()` wrap
// (rawcode % gfx->elements()), the same rule the playfield already follows. Feeding all 14
// bits would address ROM that does not exist.
//
// The plane stride is content_size/2: it is a property of how much ROM the game has, not of
// the 1 MiB slot the stream gives every game.
// ---------------------------------------------------------------------------

// SDRAM sprite ROM fetcher: the sp_req/sp_valid/sp_data engine contract, with reads going
// through sys2_sdram_arb's c1 (motion-object) client as 2-word BURSTS rather than the four
// single-word controller reads a direct port would take.
//
// WHY (the ssprint level-select glitch): the
// playfield line fill shares the controller with sprite fetches. Four separate single-word
// reads per (tile,row) -- each a full request/act/CL/drain round trip on the single-word
// port -- cost the measured contention profile: the fill's 1279-clk line deadline
// breaks past ~65-70 sprite reads/line, and `pf_swap` swaps unconditionally at hblank, so a
// missed fill renders a buffer from two lines ago. The choose a track screen (8 thumbnails
// = near-max distinct-tile density + live MOs) is the first real workload that crosses the
// line: 33 late lines measured in 2 frames, visible glitching on hardware.
// The same load issued as bursts through c1 measured 1190 clk/line -- inside the deadline
// under simulated motion-object load.
//
// LAYOUT -> burst shape. A row's 8 bytes are two contiguous byte-PAIRS one plane stride
// apart (the pen-plane halves; the stride is content_size/2 words -- `plane_b` below), so
// one 4-word burst is impossible:
//   pair A: BASE +           entry*2   -> {first1,first0},  {first3,first2}
//   pair B: BASE + plane_b + entry*2   -> {second1,second0},{second3,second2}
//   entry = {tile bits, row[3:0]} (13+4 bits on the 1 MiB games, 11+4 on 256 KiB)
// The fetch is therefore the arbiter's two-deep shape: head = pair A (len 2), second =
// pair B (len 2). The controller chains the queued second as the head retires (S_BTRP ->
// accept), so the pair gap costs no full handshake. On c1_done for the head, the second is
// PROMOTED: this client shifts pair B's address into the head slot and drops req2, exactly
// the shift-up the arbiter's head-consumed interlock expects. The two requests always
// differ in address, so the identical-consecutive-requests restriction cannot trigger. (An
// earlier revision also leaned on req dropping between engine fetches; that assumption fails
// at a scanline boundary, which is why sp_valid carries request identity -- see below.)
//
// Words are counted on arb_valid (authoritative even across preemption -- the arbiter
// resumes a preempted head transparently); arb_done is used only for the shift and the
// final completion. sp_data bit layout is identical to the single-word module's.
//
// Duty-bounding note: c1 outranks the playfield's c2
// by fixed priority, so sustained c1 traffic starves line fills by design. This client is
// naturally duty-bounded -- the mob walker issues at most 40 slots (160 words) per line,
// which is the exact load the 1190-clk measurement covers.
//
// No write path: sprite loads go through sys2_sdram_loader_writer (the module this
// replaces had its ld_we tied off in integration since the loader unification).
module sys2_sprite_rom_arb #(
	parameter int SDRAM_AW   = 24,
	parameter int SDRAM_BASE = 0     // word base of the sprite image in SDRAM
) (
	input  logic        clk,
	input  logic        reset,

	// 1 = this game's sprite ROM is 1 MiB (720/apb): 8192 tiles, plane stride 0x40000 words.
	// 0 = 256 KiB (paperboy/Sprints): 2048 tiles, plane stride 0x10000 words. See the header.
	input  logic        spr_big,

	// Abstract sprite-ROM row read port (latency-tolerant request/valid).
	input  logic        sp_req,
	/* verilator lint_off UNUSEDSIGNAL */
	input  logic [13:0] sp_tile,    // [12:0] on a 1 MiB game, [10:0] otherwise; [13] never
	/* verilator lint_on UNUSEDSIGNAL */
	input  logic [3:0]  sp_row,
	output logic        sp_valid,
	output logic [63:0] sp_data,    // {row_second[31:0], row_first[31:0]}

	// sys2_sdram_arb c1 burst-client port.
	output logic                arb_req,
	output logic [SDRAM_AW-1:0] arb_addr,
	output logic [8:0]          arb_len,
	output logic                arb_req2,
	output logic [SDRAM_AW-1:0] arb_addr2,
	output logic [8:0]          arb_len2,
	input  logic                arb_valid,
	input  logic [15:0]         arb_rdata,
	input  logic                arb_done
);

typedef enum logic [1:0] { S_IDLE, S_FETCH, S_DONE } st_t;
st_t st = S_IDLE;

logic [16:0] r_entry;       // {tile[12:0], row[3:0]}, top two bits zero on a 256 KiB game
// Plane B offset in WORDS -- content_size/2, per game.
wire [SDRAM_AW-1:0] plane_b = spr_big ? SDRAM_AW'(24'h04_0000) : SDRAM_AW'(24'h01_0000);
logic [1:0]  wcnt;          // words captured, 0..3
logic        head_done;     // first burst (pair A) has completed
logic [15:0] rd_word0, rd_word1, rd_word2, rd_word3;
logic        sp_req_prev;

// word = base + (plane ? stride : 0) + entry*2. The old form folded the plane into a fixed
// bit-17 position, which only works while the stride happens to be 0x10000.
wire [SDRAM_AW-1:0] pairA = SDRAM_BASE[SDRAM_AW-1:0] +
                            {{(SDRAM_AW-18){1'b0}}, r_entry, 1'b0};
wire [SDRAM_AW-1:0] pairB = SDRAM_BASE[SDRAM_AW-1:0] + plane_b +
                            {{(SDRAM_AW-18){1'b0}}, r_entry, 1'b0};

assign sp_data  = {rd_word3, rd_word2, rd_word1, rd_word0};

// The entry the client is asking for right now, in the same encoding as the latched r_entry.
wire [16:0] cur_entry = spr_big ? {sp_tile[12:0], sp_row}
                                : {2'b00, sp_tile[10:0], sp_row};

// sp_valid must carry request identity. Completing a fetch is not the same as answering the
// question being asked. The header above assumed "req drops between engine fetches"; it does not
// hold at a scanline boundary, where sys2_mob_walker resets its link address to 0 and
// abandons whatever request was outstanding. This module cannot see that, so it used to finish
// the old row and assert sp_valid while the engine was already presenting the first object of
// the new line, which consumed the previous line's pixels as its own.
// Measured on 720's motion-object height test with the shipping read path: stale
// fetches at the first fetch of a new scanline, each carrying the previous line's row. On
// hardware the same screen drew 44 extra pixels.
// Generalise, as the SDRAM arbiter's head interlock already teaches: A handshake that samples a
// client at retire samples whatever the client moved on to. Match completion to the request.
assign sp_valid = (st == S_DONE) && (r_entry == cur_entry);

assign arb_len   = 9'd2;
assign arb_len2  = 9'd2;
// Head slot: pair A until its done pulses, then pair B (the shift-up). The second slot
// offers pair B only while the head is still pair A.
assign arb_addr  = head_done ? pairB : pairA;
assign arb_addr2 = pairB;
assign arb_req   = (st == S_FETCH);
assign arb_req2  = (st == S_FETCH) && !head_done;

always_ff @(posedge clk) begin
	sp_req_prev <= sp_req;

	if (reset) begin
		st          <= S_IDLE;
		r_entry     <= 17'd0;
		wcnt        <= 2'd0;
		head_done   <= 1'b0;
		rd_word0    <= 16'd0; rd_word1 <= 16'd0;
		rd_word2    <= 16'd0; rd_word3 <= 16'd0;
		sp_req_prev <= 1'b0;
	end else begin
		unique case (st)
		S_IDLE: begin
			if (sp_req && !sp_req_prev) begin
				// MAME's code % elements, expressed as the mask it actually is.
				r_entry   <= spr_big ? {sp_tile[12:0], sp_row}
				                     : {2'b00, sp_tile[10:0], sp_row};
				wcnt      <= 2'd0;
				head_done <= 1'b0;
				st        <= S_FETCH;
			end
		end

		S_FETCH: begin
			if (arb_valid) begin
				unique case (wcnt)
					2'd0: rd_word0 <= arb_rdata;
					2'd1: rd_word1 <= arb_rdata;
					2'd2: rd_word2 <= arb_rdata;
					2'd3: rd_word3 <= arb_rdata;
				endcase
				wcnt <= wcnt + 2'd1;
			end
			if (arb_done) begin
				if (!head_done) head_done <= 1'b1;   // pair A retired: shift pair B up
				else            st        <= S_DONE; // pair B retired: row complete
			end
		end

		S_DONE: begin
			// Gating sp_valid alone would DEADLOCK: with the client still asserting sp_req for a
			// request this fetch does not answer, sp_valid stays low and `!sp_req` never fires.
			// So a mismatched completion is discarded and the fetch re-issued for what is being
			// asked now. Costs one wasted 4-word row per abandoned request -- scanline boundaries
			// only -- and cannot loop, because the re-issue latches the entry it then checks
			// against. head_done/wcnt must restart with it or pair B would be re-requested as if
			// pair A had already retired.
			if (sp_req && (r_entry != cur_entry)) begin
				r_entry   <= cur_entry;
				wcnt      <= 2'd0;
				head_done <= 1'b0;
				st        <= S_FETCH;
			end else if (!sp_req) st <= S_IDLE;
		end

		default: st <= S_IDLE;
		endcase
	end
end

endmodule
