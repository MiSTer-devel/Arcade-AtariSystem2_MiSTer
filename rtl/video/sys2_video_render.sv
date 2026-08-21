// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Paperboy video renderer: playfield (background), motion objects and the alpha
// (text) layer composited into one RGB pixel per ce_pix. Per-pixel raster fetch
// extended from a single-layer alpha bring-up renderer with the 4bpp playfield
// path. One palette read
// per output pixel (the winning layer's index), so the single colour-RAM video
// port is shared cleanly.
//
// Layers:
//   - Playfield: 128x64 map of 8x8, 4bpp tiles. pf_top RAM = tile rows 0-31,
//     pf_bot = rows 32-63, word index = row[4:0]*128 + col. Tile word: code[9:0],
//     bank-select[10], colour group[13:11], category[15:14] inverted (MAME
//     (~data>>14)&3, fed to the priority mixer). Full code = (bank[sel]<<10)|
//     code[9:0]; banks from the X/Y-scroll low nibbles. 4bpp pen via the MAME
// pflayout planar packing: the first ROM half
//     (rom_lo) holds pen bits 3,2 and the second half (rom_hi) pen bits 1,0,
//     MSB-first within each nibble. PF palette base 128: idx = {1, group, pen}.
// - Alpha: 64x48 map of 8x8, 2bpp chars. MSB-first nibble
//     order. Alpha palette base 64: idx = {010, group, pen}. Pen 0 is transparent
//     (shows the playfield beneath).
//
// Scroll value fields per SP-275 sheet 15A:
//   X = xscroll[15:6] (pixels, 0-1023) + bank0 = xscroll[3:0], effective on
//       the following scanline after the CPU write. The 4096-tile ROM consumes
//       the low two bank bits.
//   Y = yscroll[14:6] (0-511) + bank1 = yscroll[3:0]. The 4096-tile ROM consumes
//       the low two bank bits. When yscroll[4]=0, the
//       write clocks scroll immediately relative to the current scanline:
//       scrolly = value - v_count. When yscroll[4]=1, the value is loaded at
//       scanline 0.
//       BIT 4 defers the scroll position only. bank1 takes effect on the write in
//       BOTH modes -- see the note at bank1's declaration below.
// Motion objects arrive pre-rendered per scanline (the mob_lb_* line-buffer port
// below) and join the per-pixel priority resolve in sys2_priority.
module sys2_video_render
(
	input  logic        clk,        // 32 MHz video clock (clk_sys)
	input  logic        reset,
	input  logic        ce_pix,     // 16 MHz pixel enable

	// Raster from sys2_video_timing.
	/* verilator lint_off UNUSEDSIGNAL */
	input  logic [9:0]  h_count,
	/* verilator lint_on UNUSEDSIGNAL */
	input  logic [8:0]  v_count,
	input  logic        hblank,
	input  logic        vblank,
	input  logic        hsync,
	input  logic        vsync,

	// Scroll/bank registers (013400 / 013600); see the per-field latching rules above.
	/* verilator lint_off UNUSEDSIGNAL */
	input  logic [15:0] xscroll,
	input  logic [15:0] yscroll,
	/* verilator lint_on UNUSEDSIGNAL */
	input  logic        xscroll_update,  // clk-domain pulse after xscroll crosses CDC
	input  logic        yscroll_update,  // clk-domain pulse after yscroll crosses CDC

	// Alpha tile map (main-bus video read port; one-cycle latency).
	output logic [11:0] alpha_addr,
	/* verilator lint_off UNUSEDSIGNAL */
	input  logic [15:0] alpha_data,
	/* verilator lint_on UNUSEDSIGNAL */

	// Character ROM (sys2_char_rom; one-cycle latency).
	output logic [13:0] char_addr,
	input  logic [7:0]  char_data,

	// Playfield tile map (main-bus video ports for VMMU banks 2/3).
	output logic [11:0] pf_top_addr,
	input  logic [15:0] pf_top_data,
	output logic [11:0] pf_bot_addr,
	input  logic [15:0] pf_bot_data,

	// Playfield tile ROM (one-cycle latency).
	output logic [17:0] pf_rom_addr,
	input  logic [15:0] pf_rom_data,

	// Colour group and priority category for the tile word arriving on pf_rom_data, supplied by
	// the SDRAM line buffer (sys2_pf_linebuf.rd_attr) and aligned with it.
	//   pf_attr = the prefetched map word's [15:11] = { category (raw), colour group }
	//   pf_attr_en = 1 on the SDRAM tile path; 0 keeps the BRAM behaviour bit for bit.
	//
	// Why this port exists. On the BRAM path the bitmap and the colour come from ONE map
	// read -- the renderer's own -- exactly as the real board works. The SDRAM path has to
	// prefetch, so it samples the map for line L+2 two lines early, and taking the colour from
	// the render-time read instead left a stale bitmap under a current colour whenever the CPU
	// rewrote that entry in between (~1 fetch/frame, 4 px, measured over 1200 frames in `emu`).
	// A tile drawn in another tile's colour is something the hardware cannot do at all, so the
	// two are now taken from the same word. See sys2_pf_linebuf's rd_attr comment.
	input  logic [4:0]  pf_attr,
	input  logic        pf_attr_en,

	// Screen tile column matching pf_rom_addr, for the SDRAM line-buffer path
	// (sys2_pf_linebuf indexes by column, because the tile CODE spans the whole region
	// while a scanline only ever touches one tile per column). Registered in the same
	// stage as pf_rom_addr so the two stay aligned. Additive: the BRAM path ignores it
	// and nothing about pf_rom_addr or the pipeline timing changes.
	output logic [6:0]  pf_rom_col,

	// EFFECTIVE scroll actually used by this cycle's fetch, i.e. after the pending /
	// immediate-update rules have been applied. Exported so the SDRAM tile prefetcher can
	// be handed the same values the renderer is using rather than re-deriving them from
	// raw xscroll/yscroll -- reimplementing those rules in the caller would test a copy of
	// them, not the rules. Purely additive: nothing internal reads these back.
	output logic [9:0]  sx_fetch_o,
	output logic [8:0]  sy_fetch_o,
	output logic [3:0]  bank0_fetch_o,
	output logic [3:0]  bank1_fetch_o,

	// Colour RAM / palette (main-bus palette video port; one-cycle latency).
	output logic [7:0]  pal_addr,
	input  logic [15:0] pal_data,

	// Motion-object line-buffer read port (sys2_mob_engine). Read-and-clear,
	// one ce_pix registered latency like the other video read ports. The entry is
	// {priority[1:0], color[1:0], pen[3:0]}; pen 15 means no object (transparent).
	output logic [8:0]  mob_lb_col,
	output logic        mob_lb_rd_en,
	input  logic [7:0]  mob_lb_data,

	// Pipeline-aligned video output.
	output logic [7:0]  red,
	output logic [7:0]  green,
	output logic [7:0]  blue,
	output logic        hsync_o,
	output logic        vsync_o,
	output logic        de_o
);

// ---- scroll application timing ----
logic [9:0] sx;             // active X scroll, pixels
logic [9:0] sx_pending;
logic [3:0] bank0;          // active playfield bank for word[10]=0
logic [3:0] bank0_pending;
logic       x_pending;

logic [8:0] sy;             // active Y scroll offset, pixels
logic [8:0] sy_frame;
logic [3:0] bank1;          // active playfield bank for word[10]=1
logic       y_frame_pending;

// The bank is not deferred by yscroll[4]. Only the scroll position is.
// bank1 used to be written inside the `yscroll[4]` branch, so a write that
// deferred the scroll to line 0 ALSO held the new tile bank until frame_start. MAME's
// yscroll_w updates m_playfield_tile_bank[1] OUTSIDE its `if (!(newscroll & 0x10))`
// conditional (src/mame/atari/atarisy2_v.cpp) -- bit 4 schedules the scroll load, the bank
// takes effect on the write in both modes, followed by mark_all_dirty().
//
// The cost of the divergence was a whole frame of stale bank: every playfield map word with
// bit 10 set draws `(bank1 << 10) | word[9:0]`, so the affected tiles came from the previous
// 1024-tile page while bank-0 tiles beside them stayed correct. On hardware, in APB's attract
// demo, that renders a building as coherent artwork from the wrong page: measured drawn
// bank 1 where bank 2 was intended,
// tile DATA byte-perfect -- 145/145 blocks in the screenshot matched intact ROM tiles, which
// is what ruled the SDRAM read path out and pointed here.
//
// bank0/xscroll is NOT the same case and is deliberately left alone: it is applied at the
// next line_start, and MAME's segment granularity (update_partial on scroll writes) makes that
// the same scanline boundary. The bug here was a FRAME of deferral, not a line.
// A dedicated bench covers this latch timing, separately from the one covering the bank's
// width. A width bench cannot see a latch-timing bug, and vice versa.

wire line_start  = ce_pix && (h_count == 10'd0);
wire frame_start = line_start && (v_count == 9'd0);
wire x_apply     = line_start && x_pending;
wire y_frame_apply = frame_start && y_frame_pending;
wire y_immediate_update = yscroll_update && !yscroll[4];

wire [8:0] y_immediate_value = yscroll[14:6] - v_count;

// Values used by the current pixel fetch. The line/frame-start selectors let the
// first pixel of the line see newly loaded scroll values even though the active
// registers are updated on the same clock edge.
wire [9:0] sx_fetch    = x_apply ? sx_pending : sx;
wire [3:0] bank0_fetch = x_apply ? bank0_pending : bank0;
wire [8:0] sy_fetch    = y_immediate_update ? y_immediate_value
                         : (y_frame_apply ? sy_frame : sy);
assign sx_fetch_o    = sx_fetch;
assign sy_fetch_o    = sy_fetch;
assign bank0_fetch_o = bank0_fetch;
assign bank1_fetch_o = bank1_fetch;

// Immediate in BOTH yscroll[4] modes -- see the note at bank1's declaration. The selector
// mirrors sy_fetch's: it lets the pixel being fetched on this very edge see the new bank,
// because the `bank1` register only takes it at the end of the same clock.
wire [3:0] bank1_fetch = yscroll_update ? yscroll[3:0] : bank1;

always_ff @(posedge clk) begin
	if (reset) begin
		sx <= '0; sx_pending <= '0; bank0 <= '0; bank0_pending <= '0;
		x_pending <= 1'b0;
		sy <= '0; sy_frame <= '0; bank1 <= '0;
		y_frame_pending <= 1'b0;
	end else begin
		if (x_apply) begin
			sx <= sx_pending;
			bank0 <= bank0_pending;
			x_pending <= 1'b0;
		end
		if (xscroll_update) begin
			sx_pending <= xscroll[15:6];
			bank0_pending <= xscroll[3:0];
			x_pending <= 1'b1;
		end

		if (y_frame_apply) begin
			sy <= sy_frame;
			y_frame_pending <= 1'b0;
		end
		if (yscroll_update) begin
			// Unconditional: the bank is never deferred, only the scroll position is.
			bank1 <= yscroll[3:0];
			if (yscroll[4]) begin
				sy_frame <= yscroll[14:6];
				y_frame_pending <= 1'b1;
			end else begin
				sy <= y_immediate_value;
				y_frame_pending <= 1'b0;
			end
		end
	end
end

// ---- per-stage carried context ----
// stage 1 -> 2
logic [2:0] a_px1, a_py1;          // alpha pixel within tile
logic [2:0] pf_px1, pf_py1;        // playfield pixel within tile
logic       pf_half1;              // tile row >= 32 (select pf_bot)
logic [6:0] pf_col1;               // screen tile column, staged like pf_px1
logic [3:0] bank0_1, bank1_1;      // banks aligned with the fetched tile word
logic [8:0] col1;                  // screen column carried to the MO line-buffer read
logic       hs1, vs1, hb1, vb1;
// stage 2 -> 3
logic [1:0] a_pxlo2;
logic [2:0] a_group2;
logic [1:0] pf_pxlo2;
logic [2:0] pf_group2;
logic [1:0] pf_category2;     // PF priority category (tile word 15:14) for the MO mixer
logic       hs2, vs2, hb2, vb2;
// stage 3 -> 4
logic       hs3, vs3, hb3, vb3;

// playfield pixel-x/y after scroll (10-bit / 9-bit adds wrap mod 1024 / mod 512,
// matching the 128x8 / 64x8 playfield extent).
logic [9:0] pfx;
logic [8:0] pfy;
assign pfx = h_count + sx_fetch;
assign pfy = v_count + sy_fetch;

// full 14-bit playfield tile code from the fetched word
/* verilator lint_off UNUSEDSIGNAL */
logic [15:0] pf_word2;
/* verilator lint_on UNUSEDSIGNAL */
logic [3:0]  pf_bankval2;
logic [13:0] pf_code2;
assign pf_word2    = pf_half1 ? pf_bot_data : pf_top_data;
assign pf_bankval2 = pf_word2[10] ? bank1_1 : bank0_1;
// MAME: code = (bank[(data>>10)&1] << 10) | (data & 0x3ff), with a FOUR-bit bank
// (xscroll&0x0f / yscroll&0x0f) -- 14 bits, up to 16384 tiles for APB.
//
// Paperboy has only 4096 tiles, and its bank never exceeds 3: all three golden captures
// have xscroll&0x0f = 0 and yscroll&0x0f in {3,1}, so bits 3:2 are zero in every frame we
// have hardware-comparable evidence for.
//
// If they were ever set it is still correct. pf_rom_addr is {pf_code2, py, px[2]}, so the
// two new bank bits pf_code2[13:12] land in pf_rom_addr[17:16] -- and an on-chip tile ROM
// wired to pf_rom_addr[15:0] only would simply drop them. Dropping them is exactly
// `code % 4096`, and 4096 is Paperboy's decoded tile-element count, so this reproduces
// MAME's own wrap: tile_data::set() does `rawcode % gfx->elements()` before the fetch.
//
// The widening is therefore inert on that path -- which also means a frame-level CRC
// cannot observe it, and a dedicated bank-4 bench is what proves the wide
// bits actually reach pf_rom_addr for the SDRAM path to use.
assign pf_code2    = {pf_bankval2, pf_word2[9:0]};

// 2bpp alpha pen (MSB-first within the nibble).
logic [1:0] a_pen;
always_comb begin
	case (a_pxlo2)
		2'd0: a_pen = {char_data[7], char_data[3]};
		2'd1: a_pen = {char_data[6], char_data[2]};
		2'd2: a_pen = {char_data[5], char_data[1]};
		default: a_pen = {char_data[4], char_data[0]};
	endcase
end

// 4bpp playfield pen via the shared sys2_pen4 kernel (MAME `pflayout` planar
// packing: FIRST region half rom_lo = pf_rom_data[7:0] -> pen bits 3,2; SECOND
// half rom_hi = pf_rom_data[15:8] -> pen bits 1,0). Using the same kernel as the
// motion-object layer keeps the plane order in one place so the colour bug
// cannot reappear. Select the current pixel's pen by col&3 (pf_pxlo2).
logic [3:0] pf_pen0, pf_pen1, pf_pen2, pf_pen3;
sys2_pen4 u_pf_pen (
	.first_byte (pf_rom_data[7:0]),
	.second_byte(pf_rom_data[15:8]),
	.pen0(pf_pen0), .pen1(pf_pen1), .pen2(pf_pen2), .pen3(pf_pen3)
);
logic [3:0] pf_pen;
always_comb begin
	case (pf_pxlo2)
		2'd0:    pf_pen = pf_pen0;
		2'd1:    pf_pen = pf_pen1;
		2'd2:    pf_pen = pf_pen2;
		default: pf_pen = pf_pen3;
	endcase
end

// ---- effective playfield colour / priority -------------------------------------------
// Selected HERE, at the point of use, not at the stage-2 register: pf_group2/pf_category2 are
// latched at the same edge as pf_rom_addr, and pf_attr arrives with pf_rom_data one cycle
// later, so the two are already aligned at this stage. Latching pf_attr into the pipeline
// instead would shift it by a pixel.
//
// pf_category2 is `~pf_word2[15:14]`; pf_attr carries the RAW map bits, so the inversion has
// to be applied on this side too. Dropping it would flip every priority category on the SDRAM
// path -- a fault that leaves colour and structure perfect and only rearranges which layer
// wins, which is exactly the kind that survives a screenshot.
logic [2:0] pf_group_eff;
logic [1:0] pf_category_eff;
assign pf_group_eff    = pf_attr_en ?  pf_attr[2:0] : pf_group2;
assign pf_category_eff = pf_attr_en ? ~pf_attr[4:3] : pf_category2;

// palette indices for each layer
logic [7:0] a_idx, pf_idx;
assign a_idx  = {3'b010, a_group2, a_pen};        // alpha base 64
assign pf_idx = {1'b1, pf_group_eff, pf_pen};     // playfield base 128

// Motion-object pixel from the line-buffer read (valid this stage, aligned with
// pf_pen/a_pen). Entry {priority, color, pen}; mo_idx is palette base 0
// (mo_idx = {2'b00, color, pen}). Pen 15 -> transparent, handled by the mixer.
logic [3:0] mo_pen;
logic [1:0] mo_color, mo_priority;
logic [7:0] mo_idx;
assign mo_pen      = mob_lb_data[3:0];
assign mo_color    = mob_lb_data[5:4];
assign mo_priority = mob_lb_data[7:6];
assign mo_idx      = {2'b00, mo_color, mo_pen};

// Final per-pixel priority (SP-275 sheet 15B): the
// motion object competes with the playfield using the carried category, then the
// alpha layer is placed on top. When the line buffer is empty (pen 15) the mixer
// reduces to the playfield beneath the alpha layer (no motion-object contribution there).
logic [7:0] win_idx;
// layer is observability-only (used by a focused bench); unused in the renderer.
/* verilator lint_off PINCONNECTEMPTY */
sys2_priority priority_mux (
	.mo_pen      (mo_pen),
	.mo_priority (mo_priority),
	.mo_idx      (mo_idx),
	.pf_pen      (pf_pen),
	.pf_category (pf_category_eff),
	.pf_idx      (pf_idx),
	.alpha_pen   (a_pen),
	.alpha_idx   (a_idx),
	.pal_index   (win_idx),
	.layer       ()
);
/* verilator lint_on PINCONNECTEMPTY */

// colour conversion of the winning palette word.
logic [7:0] pr, pg, pb;
sys2_palette palette (.color_word(pal_data), .r(pr), .g(pg), .b(pb));

always_ff @(posedge clk) begin
	if (reset) begin
		alpha_addr <= '0; char_addr <= '0; pal_addr <= '0;
		pf_top_addr <= '0; pf_bot_addr <= '0; pf_rom_addr <= '0;
		a_px1 <= '0; a_py1 <= '0; pf_px1 <= '0; pf_py1 <= '0; pf_half1 <= 1'b0;
		pf_col1 <= '0; pf_rom_col <= '0;
		bank0_1 <= '0; bank1_1 <= '0; col1 <= '0;
		mob_lb_col <= '0; mob_lb_rd_en <= 1'b0;
		hs1 <= 1'b0; vs1 <= 1'b0; hb1 <= 1'b1; vb1 <= 1'b1;
		a_pxlo2 <= '0; a_group2 <= '0; pf_pxlo2 <= '0; pf_group2 <= '0;
		pf_category2 <= '0;
		hs2 <= 1'b0; vs2 <= 1'b0; hb2 <= 1'b1; vb2 <= 1'b1;
		hs3 <= 1'b0; vs3 <= 1'b0; hb3 <= 1'b1; vb3 <= 1'b1;
		red <= '0; green <= '0; blue <= '0;
		hsync_o <= 1'b0; vsync_o <= 1'b0; de_o <= 1'b0;
	end else if (ce_pix) begin
		// Stage 1: tile-map addresses.
		alpha_addr  <= {v_count[8:3], h_count[8:3]};
		pf_top_addr <= {pfy[7:3], pfx[9:3]};   // row[4:0]*128 + col[6:0]
		pf_bot_addr <= {pfy[7:3], pfx[9:3]};
		a_px1   <= h_count[2:0]; a_py1 <= v_count[2:0];
		pf_px1  <= pfx[2:0];     pf_py1 <= pfy[2:0];
		// PLAYFIELD column, not the screen column. This is the SAME pfx[9:3] that
		// picks the map word above, so a tile's bitmap and its colour/priority always
		// come from the same map entry.
		//
		// It was h_count[9:3] at first, on the reasoning that the prefetcher
		// already carries the scroll so indexing by pfx would double-apply it. That holds
		// only while sx[2:0] == 0. With a fine scroll the renderer crosses a tile boundary
		// in the MIDDLE of a screen column -- pfx[9:3] increments where h_count[9:3] does
		// not -- so the tail of every column drew the PREVIOUS tile's pixels under the NEXT
		// tile's colour: wrong bitmap, right colour, structure intact. sys2_pf_linebuf now
		// fills in this same map-column space, so there is nothing to double-apply.
		pf_col1 <= pfx[9:3];
		pf_half1<= pfy[8];                      // tile row 32-63 -> bottom RAM
		bank0_1 <= bank0_fetch; bank1_1 <= bank1_fetch;
		col1    <= h_count[8:0];                 // screen column for the MO line buffer
		hs1 <= hsync; vs1 <= vsync; hb1 <= hblank; vb1 <= vblank;

		// Stage 2: char/tile ROM addresses.
		// 10 code bits, matching MAME's `code = data & 0x3ff` for every System 2 game.
		// Paperboy never sets bit 9 (checked across all three golden captures: max
		// code 0x1d1), so this is a no-op for it and correct for the others.
		char_addr   <= {alpha_data[9:0], a_py1, a_px1[2]};
		pf_rom_addr <= {pf_code2, pf_py1, pf_px1[2]};   // code*16 + row*2 + (col>=4)
		pf_rom_col  <= pf_col1;                         // the column those bits came from
		a_pxlo2  <= a_px1[1:0]; a_group2  <= alpha_data[15:13];
		pf_pxlo2 <= pf_px1[1:0]; pf_group2 <= pf_word2[13:11];
		pf_category2 <= ~pf_word2[15:14];   // MAME (~data >> 14) & 3
		mob_lb_col   <= col1;               // read this column from the MO line buffer
		mob_lb_rd_en <= ~hb1;               // read-and-clear during active display
		hs2 <= hs1; vs2 <= vs1; hb2 <= hb1; vb2 <= vb1;

		// Stage 3: pens -> composite -> palette address.
		pal_addr <= win_idx;
		hs3 <= hs2; vs3 <= vs2; hb3 <= hb2; vb3 <= vb2;

		// Stage 4: palette word -> RGB (blanked region black).
		if (hb3 || vb3) begin
			red <= '0; green <= '0; blue <= '0;
		end else begin
			red <= pr; green <= pg; blue <= pb;
		end
		hsync_o <= hs3; vsync_o <= vs3; de_o <= ~(hb3 | vb3);
	end
end

endmodule
