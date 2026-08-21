// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Playfield tile-row line buffer + prefetcher.
//
// Fetches every tile-row word a scanline will need, one line ahead, into a small
// double-buffered BRAM. The renderer then reads the buffer at zero latency cost instead of
// a 128 KiB tile ROM, which is what lets the tile data live in SDRAM -- mandatory for the
// other four System 2 games (up to 544 KiB of tiles against ~71 free M10K blocks).
//
// ---------------------------------------------------------------------------
// Why the buffer is indexed by column, not by address
//
// The renderer addresses the tile ROM by tile CODE
// (`pf_rom_addr = {pf_code2, pf_py1, pf_px1[2]}`), and codes span the whole region -- so a
// buffer indexed that way would have to be the entire ROM. But a scanline only ever
// touches one tile per playfield column, so indexing by {column, half} needs just
// 128 x 2 x 16 bit.
//
// The column is the playfield-map column `pfx[9:3]`, not the screen column
// `h_count[9:3]`. It was the screen column at first, and that is a real corruption
// whenever the horizontal scroll is not a multiple of 8:
//
//   pfx = h_count + sx, so within ONE screen column the renderer's pfx[9:3] takes TWO values
//   when sx[2:0] != 0 -- the first (8 - sx[2:0]) pixels come from map column S+c and the rest
//   from S+c+1. That is just how fine scrolling works, and the renderer does it correctly:
//   it re-reads the map every pixel, so the colour group and priority follow the boundary.
//   The buffer, however, holds ONE tile per entry. Indexed by screen column, the tail of every
//   column returned the PREVIOUS tile's bitmap while the renderer coloured it with the NEXT
//   tile's map word -- wrong bitmap, right colour, structure and palette perfectly intact.
//
// Measured before the fix, with a static non-tile-aligned scroll (sx[2:0] = 4):
// 97,536 of 195,840 fetches wrong, exactly 4 pixels in every 8, every column, every line --
// and the failing word at column N was precisely the word column N-1 should have shown.
//
// EVERY internal instrument read ZERO through that run: Column/map pairing, line alignment,
// burst data, dropped WORDS, WORDS per burst, P_ADVANCE. The prefetcher was doing exactly what
// it was told; the defect was the indexing contract between the buffer and the renderer, which
// no instrument inside either one can see. The equivalence bench missed it for a different
// reason -- see the FILL_COLS note below.
//
// Indexing in map space also means the fill must cover 65 columns, not 64: with a fine scroll
// the last visible pixel belongs to map column S+64.
//
// ---------------------------------------------------------------------------
// Repacked tile format
//
// The on-chip tile ROM this replaced kept two byte arrays and returned {hi, lo} from
// the SAME offset in each. In SDRAM that pair is stored as ONE 16-bit word at word address
//
//     code*16 + row*2 + half        (i.e. exactly today's pf_addr)
//
// so a single SDRAM word read yields precisely what `pf_data` returned. Without the
// repack every tile-row would cost two reads and double the bandwidth demand.
//
// The two halves of a column are adjacent (half 0 then half 1), so each column is a
// 2-word burst -- 64 bursts, 128 words per line. That is the short-burst access
// pattern the controller was tuned for, not a page stream.
//
// ---------------------------------------------------------------------------
// Playfield-RAM port
//
// The prefetcher needs the tile map for the line it is fetching. It does NOT need a third
// RAM port: the renderer's video port is idle 7 cycles out of 8, because `pfx[9:3]` only
// advances once per 8 pixels and the render pipeline re-reads the same address in between.
// The integrator hands those spare slots here via `ram_grant`.
module sys2_pf_linebuf #(
	parameter int AW   = 24,   // SDRAM word-address width
	parameter int COLS      = 128, // buffer depth in columns = the playfield's full column
	                               // count (pfx[9:3] is 7 bits). Indexing in map space means
	                               // a blanking-time read lands on a stale entry it will never
	                               // display, instead of aliasing onto a visible column.
	parameter int FILL_COLS = 65   // columns actually fetched: the 64 that cover the 512
	                               // visible pixels, plus one. With a fine scroll the last
	                               // visible pixel belongs to map column S+64, so 64 leaves
	                               // the right-hand edge reading a stale entry.
) (
	input  logic        clk,
	input  logic        reset,

	// ---- line control ----
	input  logic        start,        // pulse: begin prefetching the line described below
	input  logic        swap,         // pulse at the line boundary: the buffer just
	                                  // filled becomes the one the renderer reads
	input  logic [8:0]  pfy,          // playfield Y of the line being prefetched
	input  logic [9:0]  pfx0,         // playfield X of screen column 0 (scroll)
	input  logic [3:0]  bank0,        // code = (bank[(data>>10)&1] << 10) | (data & 0x3ff)
	input  logic [3:0]  bank1,
	input  logic [AW-1:0] tile_base,  // SDRAM word address of the tile region

	// Tile-code wrap, and it is NOT optional -- see the note at the tile_code assignment.
	// (elements - 1), where elements = the game's decoded tile count = HALF/16:
	// 4096 paperboy (0x0fff), 8192 720 (0x1fff), 16384 the rest (0x3fff).
	input  logic [13:0] code_mask,

	// ---- playfield tile-map read port (spare render slots) ----
	output logic        ram_req,
	input  logic        ram_grant,    // this cycle's slot is ours
	output logic [11:0] ram_addr,     // {row[4:0], col[6:0]}
	output logic        ram_bot,      // 0 = top RAM, 1 = bottom (tile rows 32-63)
	input  logic [15:0] ram_data,     // one-cycle registered latency, like the render port

	// ---- SDRAM burst client (to sys2_sdram_arb) ----
	output logic        sd_req,
	output logic [AW-1:0] sd_addr,
	output logic [8:0]  sd_len,
	// Second request, presented while the head is still draining. This is what lets the
	// arbiter hand column N+1 to the controller before column N retires; without it the
	// controller walks out to S_IDLE and waits to be asked again, which measured 33.6 % of
	// all cycles. The map word for N+1 is already fetched ahead of
	// time, so the address costs nothing extra.
	output logic        sd_req2,
	output logic [AW-1:0] sd_addr2,
	output logic [8:0]  sd_len2,
	input  logic        sd_valid,
	input  logic [15:0] sd_rdata,
	input  logic        sd_done,

	// ---- render read port: zero-latency-equivalent replacement for the tile ROM ----
	input  logic [6:0]  rd_col,
	input  logic        rd_half,
	output logic [15:0] rd_data,

	// ---- The look-ahead race fix ----
	//
	// The 5 attribute bits of the map word this column's bitmap was built from:
	//     rd_attr = map_word[15:11] = { priority category (raw), colour group }
	// so the renderer can take colour and priority from the SAME word as the bitmap.
	//
	// The race it closes. The prefetcher samples the map for line L+2 two LINES early; the
	// renderer used to take the colour from its OWN read at render time. A CPU write to that
	// entry in between left a stale bitmap under a current colour -- wrong bitmap, right colour.
	// A design without the look-ahead is immune on identical hardware: the bitmap and the
	// colour then come from one read. Measured in the real `emu` over 1200
	// frames: 1,176 bad fetches (~1/frame = 4 px), 1,166 of them on a map entry written within
	// 4 lines. Known-benign, but structurally impossible on the real board, so it is now closed.
	//
	// It is only five bits because only five were ever incoherent. The tile code (map[10:0])
	// already comes from the prefetched word -- see `map_used` below -- so bits 10:0 would be a
	// second copy of data the address is already built from. 5 x 128 x 2 buffers = 1280 bit, and
	// the docs' "~1 M10K (128 x 16 bit x 2)" estimate was costing the 11 bits that never moved.
	// The FITTED cost is still 2 M10K (500 -> 502/553): Quartus 17.0 infers these as altsyncram
	// Simple Dual Port and gives each a whole block. `ramstyle = "MLAB"` would return both if M10K
	// ever binds -- deliberately not applied.
	//
	// The colour is now coherent but two LINES stale, and that is the deliberate trade. The
	// real board has no prefetch, so neither answer reproduces it exactly; a frame that is a
	// self-consistent picture of the map two lines ago is far closer to hardware than one that
	// draws a tile's shape under another tile's colour, which the board cannot do at all.
	//
	// Verified with an animated tilemap -- the FIRST playfield check with a
	// map that moves while the frame renders. Every other one loads the map once, which is
	// exactly why this survived from the day the prefetcher was written. With this port: 0 bad
	// of 196,608 pixels. Without it: 257,624 findings over those same pixels -- more than one
	// each, because most fail the category AND the group check.
	output logic [4:0]  rd_attr,

	output logic        busy          // still filling; a line must not start reading early
);

// Double buffer: fill one while the renderer reads the other.
localparam int ENTRIES = COLS * 2;
logic [15:0] buf0 [0:ENTRIES-1];
logic [15:0] buf1 [0:ENTRIES-1];
// Attribute buffers, one entry per COLUMN (not per column-half: both halves of a column are
// the same tile, so they share one map word). Written from the same enable and the same
// `map_col` as the tile words above, so an attribute can never pair with a bitmap the fill
// did not fetch at that instant -- whatever alignment the tile data has, this has too.
logic [4:0]  abuf0 [0:COLS-1];
logic [4:0]  abuf1 [0:COLS-1];
logic        fill_sel;              // which buffer the prefetcher is filling
wire  [15:0] rd_q;         // driven by the post-register buffer select below

typedef enum logic [2:0] { P_IDLE, P_PRIME, P_RUN, P_ADVANCE, P_DONE } pstate_t;
pstate_t pstate;

logic [6:0]  col;          // fill counter, 0 .. FILL_COLS-1
// Playfield-map column of the column being fetched -- the buffer INDEX. Tracked separately
// from `col` because the buffer is addressed in map space while the fill is counted in
// columns-since-start; they differ by pfx0[9:3] and wrap independently (map columns wrap at
// 128, the fill just stops at FILL_COLS).
logic [6:0]  map_col;
logic        word_idx;     // which of the column's two words is arriving
logic [15:0] map_word;     // map word for the column being fetched

// ---- map-fetch sub-pipeline -------------------------------------------------------
// The serial version (slot -> RAM -> burst -> next column) cost ~20 clk per column,
// which at 80 columns overran the ~1280 clk line time and published half-filled
// buffers. The map word for column N+1 is therefore fetched WHILE column N's burst is
// in flight, so a column costs max(slot wait, burst) rather than their sum.
typedef enum logic [1:0] { M_IDLE, M_REQ, M_WAIT, M_READY } mstate_t;
mstate_t mstate;
logic [15:0] map_next;     // map word for col+1, ready ahead of time
logic [9:0]  nxt_pfx;

// The second request's map word, promoted to `map_word` when the head completes.
logic [15:0] nxt_map;
logic        nxt_valid;

// Both sides address the SAME space: rd_col is the renderer's pfx[9:3], map_col is the
// playfield column this fill is fetching. See the header note -- these used to be screen
// columns on the read side and fill-order columns on the write side, which agreed only while
// the scroll was tile-aligned.
wire [7:0] rd_index   = 8'({rd_col, rd_half});
wire [7:0] fill_index = 8'({map_col, word_idx});

// Registered read, matching the tile ROM's one-cycle latency exactly so the render
// pipeline's timing is untouched. While the prefetcher fills one buffer the renderer
// reads the other.
//
// The buffer select is applied after the output register, and that is deliberate.
// This was written as the obvious one-liner:
//
//     always_ff @(posedge clk) rd_q <= fill_sel ? buf0[rd_index] : buf1[rd_index];
//
// which is correct RTL, and correct in every simulator, because the select and the array
// read are sampled at the SAME edge. But it asks Quartus to infer two M10Ks whose outputs
// are muxed by a signal that must be delay-matched to the RAM's own output register. Get
// that alignment wrong by one cycle and the renderer reads the buffer currently being
// FILLED -- the NEXT line's tiles, half-written -- which is wrong bitmap, right colour,
// structure intact, at ANY scroll, and invisible to every simulation. That is exactly the
// hardware symptom, so the ambiguity is removed rather than argued about.
//
// Written this way there is nothing to align: each RAM has a plain registered read, and the
// select is explicitly delayed by the same single cycle. Semantically identical -- rd_data
// at T+1 is (fill_sel at T) ? buf0[rd_index at T] : buf1[rd_index at T] either way.
logic [15:0] q0, q1;
logic [4:0]  a0, a1;
logic        fill_sel_d;
always_ff @(posedge clk) begin
	q0         <= buf0[rd_index];
	q1         <= buf1[rd_index];
	// Same structure, same single cycle of latency, and deliberately the same post-register
	// select -- so rd_attr and rd_data can never come from different buffers, which would be
	// the look-ahead race back again with a one-line offset instead of two.
	a0         <= abuf0[rd_col];
	a1         <= abuf1[rd_col];
	fill_sel_d <= fill_sel;
end
assign rd_q    = fill_sel_d ? q0 : q1;
assign rd_data = rd_q;
assign rd_attr = fill_sel_d ? a0 : a1;

// Bits 15:11 are the colour group and priority category; they leave through rd_attr. Bits
// 10:0 are the bank select and tile code, which build the fetch address below. Every bit of
// the map word is now used, which is the point: before rd_attr existed, 15:11 were read by
// the RENDERER from a separate, later read of the same map entry.
wire [15:0] map_used = map_word;
// The code must wrap at the game's tile count. The real board's tile ROM only decodes as
// many address lines as it has, so an over-range code wraps; MAME reproduces that as
// `rawcode % gfx->elements()`. An on-chip tile ROM gets this for free -- wire it
// `pf_rom_addr[15:0]` and the two bank bits are silently dropped, which IS `code % 4096`
// (sys2_video_render.sv). This path builds its own address, so it must do the masking
// explicitly or it walks off the end of the image into 0xff fill = pen 15 = WHITE.
//
// That is exactly what shipped: Paperboy's playfield came up as white distortion behind the
// alpha layer on the first build that actually rendered. Paperboy sets bank bits
// it expects to be dropped, so every tile with bank >= 4 fetched fill.
//
// The equivalence bench was structurally blind to it: it drives `.bank0({2'b00, XSCROLL[1:0]})`,
// masking the bank to 0..3, so tile_code never exceeded 4095 and the wrap was never reached --
// while its own comment claimed "bank0 = 4". A gate that cannot reach the failing input proves
// nothing about it.
wire [3:0]  sel_bank = map_used[10] ? bank1 : bank0;
wire [13:0] tile_code = {sel_bank, map_used[9:0]} & code_mask;
wire [AW-1:0] col_addr = tile_base + AW'({tile_code, pfy[2:0], 1'b0});

// Same arithmetic for the queued column.
wire [3:0]  sel_bank2  = nxt_map[10] ? bank1 : bank0;
wire [13:0] tile_code2 = {sel_bank2, nxt_map[9:0]} & code_mask;
wire [AW-1:0] col_addr2 = tile_base + AW'({tile_code2, pfy[2:0], 1'b0});

assign ram_addr = {pfy[7:3], nxt_pfx[9:3]};
assign ram_bot  = pfy[8];
assign busy     = (pstate != P_IDLE);

always_ff @(posedge clk) begin
	ram_req <= 1'b0;
	sd_req  <= 1'b0;
	sd_req2 <= 1'b0;

	if (swap) fill_sel <= ~fill_sel;

	if (reset) begin
		pstate    <= P_IDLE;
		mstate    <= M_IDLE;
		fill_sel  <= 1'b0;
		col       <= '0;
		map_col   <= '0;
		word_idx  <= 1'b0;
		nxt_valid <= 1'b0;
	end else begin
		// ONLY accept burst data while actually FILLING.
		//
		// This was unconditional, so a burst still in flight when the fill ended kept writing --
		// measured 324 words per frame landing at P_IDLE, into `col` 63 of the buffer being
		// prepared for the NEXT line, with word_idx toggling as they went. `start` resets col and
		// word_idx, but it cannot un-write what already landed, and a word arriving after start
		// would shift every column of the new line by one.
		//
		// P_RUN and P_ADVANCE are the states where a burst can legitimately be outstanding:
		// P_ADVANCE is entered on sd_done, so its own burst has retired, but the arbiter can
		// still be draining. P_IDLE / P_PRIME / P_DONE must never write.
		if (sd_valid && ((pstate == P_RUN) || (pstate == P_ADVANCE))) begin
			// The attribute is written from the same register that built this word's
			// address, on the same cycle. `map_word` only changes on sd_done (P_RUN promotes
			// nxt_map) or on leaving P_ADVANCE, and both are non-blocking -- so a word arriving
			// on the done cycle still sees the map word its own burst was issued from. Writing
			// it HERE, under the same enable and the same `map_col` as the tile word, is what
			// makes the pairing structural rather than a timing argument: any fault that
			// misplaces a tile word misplaces its attribute identically.
			if (fill_sel) begin
				buf1[fill_index] <= sd_rdata;
				abuf1[map_col]   <= map_word[15:11];
			end else begin
				buf0[fill_index] <= sd_rdata;
				abuf0[map_col]   <= map_word[15:11];
			end
			word_idx <= ~word_idx;
		end

		// ---- map fetch: runs concurrently with the burst ----
		case (mstate)
			M_REQ: begin
				ram_req <= 1'b1;
				if (ram_grant) mstate <= M_WAIT;
			end
			M_WAIT: begin                    // one-cycle RAM latency
				map_next <= ram_data;
				mstate   <= M_READY;
			end
			default: ;                       // M_IDLE / M_READY: nothing to do
		endcase

		// ---- keep the second request slot loaded ----
		// Runs alongside the burst FSM. `map_next` holds the map word for the column after
		// the head, so capturing it here is what gives the arbiter something to queue.
		//
		// `!sd_done` is load-bearing. Without it, a cycle where sd_done coincides with
		// (!nxt_valid && mstate == M_READY) advances the MAP pipeline twice while `col`
		// advances once:
		//
		//   * this block consumes map_next (column N+1's word) into nxt_map and starts the
		//     fetch for N+2 -- but nxt_valid is non-blocking, so the case below still reads
		//     the OLD value;
		//   * the case therefore takes the !nxt_valid branch to P_ADVANCE;
		//   * P_ADVANCE latches `map_next`, which is now column N+2's word, into map_word
		//     while incrementing col by one.
		//
		// Column N+1 then fetches column N+2's tile: wrong bitmap, RIGHT colour (the renderer
		// reads the map itself). Skipping this block on the sd_done cycle leaves map_next
		// holding N+1's word for P_ADVANCE, which is what it is written to expect.
		//
		// P_ADVANCE had NEVER been simulated -- it never fired in the equivalence bench,
		// because with the map slot always granted the map fetch is always ready in
		// time. That bench now throttles the grant to
		// starve the slot and reach it; at grant_throttle=12 this bug is 103,174 wrong fetches
		// of 195,840, with the fill still comfortably inside the line (782 of 1279 clk), so it
		// is a correctness fault and not a bandwidth one.
		if ((pstate == P_RUN) && !nxt_valid && (mstate == M_READY) && !sd_done) begin
			nxt_map   <= map_next;
			nxt_valid <= 1'b1;
			nxt_pfx   <= nxt_pfx + 10'd8;
			mstate    <= M_REQ;
		end

		// ---- burst issue ----
		case (pstate)
			P_IDLE: if (start) begin
				col      <= '0;
				map_col  <= pfx0[9:3];       // the buffer index the first column lands at
				nxt_pfx  <= pfx0;
				word_idx <= 1'b0;
				mstate   <= M_REQ;           // prime the map word for column 0
				pstate   <= P_PRIME;
			end

			P_PRIME: if (mstate == M_READY) begin
				map_word  <= map_next;
				nxt_pfx   <= nxt_pfx + 10'd8; // look ahead to column 1
				mstate    <= M_REQ;
				nxt_valid <= 1'b0;
				pstate    <= P_RUN;
			end

			// One 2-word burst per column: {half 0, half 1} are adjacent by construction.
			//
			// The head is column `col`; the second, once its map word has arrived, is
			// column col+1. On done the second is PROMOTED to head, exactly the shift the
			// arbiter's two-deep protocol expects, and the map pipeline refills the slot.
			P_RUN: begin
				// DROP `sd_req` for the `sd_done` CYCLE. This is not a nicety -- without it
				// the playfield stalls the arbiter dead whenever two adjacent columns show the
				// same tile, which real game graphics do constantly (sky, grass, road are one
				// tile repeated across many columns).
				//
				// sys2_sdram_arb identifies "a new head" by {addr,len} CHANGING while `req`
				// stays up, and its own header states the contract: "a client must not issue
				// two IDENTICAL consecutive requests (same addr and len) without dropping `req`
				// between them, or the second is swallowed as already done. No real client does
				// -- the playfield walks distinct COLUMNS". Distinct columns, yes; distinct
				// ADDRESSES, no. col_addr is tile_base + tile_code*16 + row*2, so a repeated
				// tile repeats the address exactly and the arbiter's head-consumed interlock
				// never clears.
				//
				// Measured in simulation: with NO two adjacent columns
				// sharing a tile, 27,040 bursts/frame and a 674 clk fill. Let ANY adjacent pair
				// share one -- runlen=2 -- and it collapses to 845 bursts and a 39,710 clk fill
				// against a 1279 clk line, so every line overruns and publishes a half-filled
				// buffer: wrong tile bitmaps under correct colours, which is the hardware
				// symptom. EVERY earlier gate used a scattered synthetic map (`i*37+11`) whose
				// adjacent entries can never collide, so none of them could reach this.
				//
				// Dropping req on the done cycle is also strictly more correct than it was:
				// sd_addr is registered from col_addr, and `col` only advances ON done, so the
				// cycle after done had `req` high against a STALE address anyway.
				if (!sd_done) sd_req <= 1'b1;
				sd_addr <= col_addr;
				sd_len  <= 9'd2;
				if (nxt_valid && (col != 7'(FILL_COLS-1))) begin
					sd_req2  <= 1'b1;
					sd_addr2 <= col_addr2;
					sd_len2  <= 9'd2;
				end

				if (sd_done) begin
					word_idx <= 1'b0;
					if (col == 7'(FILL_COLS-1)) pstate <= P_DONE;
					else if (nxt_valid) begin
						// The queued column becomes the head, and the slot behind it is
						// refilled in the SAME cycle when the map pipeline already has the
						// next word. Leaving the slot empty for even one cycle lets the
						// arbiter run dry: it has nothing to queue, the controller drains
						// out to S_IDLE, and the chain breaks once per column -- which is
						// exactly what the integrated measurement showed (controller idle
						// 32.6 % of busy cycles while the client was offering a second
						// 92 % of the time).
						col       <= col + 1'b1;
						map_col   <= map_col + 1'b1;
						map_word  <= nxt_map;
						if (mstate == M_READY) begin
							nxt_map   <= map_next;
							nxt_valid <= 1'b1;
							nxt_pfx   <= nxt_pfx + 10'd8;
							mstate    <= M_REQ;
						end else begin
							nxt_valid <= 1'b0;
						end
					end else begin
						// Map word for the next column has not landed yet; fall back to
						// the serial path for this one column.
						pstate <= P_ADVANCE;
					end
				end
			end

			// Only reached when the map pipeline fell behind. sd_req is DE-asserted here on
			// purpose: holding it while waiting for the next map word would offer the
			// arbiter the column we just finished.
			P_ADVANCE: if (mstate == M_READY) begin
				col      <= col + 1'b1;
				map_col  <= map_col + 1'b1;
				map_word <= map_next;
				nxt_pfx  <= nxt_pfx + 10'd8;
				mstate   <= M_REQ;
				pstate   <= P_RUN;
			end

			// Filling is finished; the actual buffer swap waits for the line boundary
			// (see `swap`). Swapping mid-line would change what the renderer is reading
			// part-way down a line -- invisible in a unit test, catastrophic on screen.
			P_DONE: pstate <= P_IDLE;

			default: pstate <= P_IDLE;
		endcase
	end
end

endmodule
