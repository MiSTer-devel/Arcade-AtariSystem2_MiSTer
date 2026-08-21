// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Paperboy per-pixel layer priority (SP-275 sheet 15B; confirmed against MAME's
// atarisy2_v.cpp). Given the three rendered layers at one screen pixel -- the
// background playfield, an optional motion-object pixel, and the topmost alpha
// (text) pixel -- it selects the winning colour-RAM index.
//
// The hardware rule (SP-275 sheet 15B):
//   1. The playfield is the opaque background; every pixel carries a 2-bit
//      priority category (tile word bits 15:14).
//   2. A motion-object pixel is transparent when its 4bpp pen is 15.
//   3/4. A non-transparent motion-object pixel wins over the playfield iff
//        ((mo_priority + pf_category) & 2) == 0  OR  the playfield pen is 0-7
//        (i.e. pen bit 3 clear). Otherwise the playfield wins. This is exactly
//        MAME's `!((mopri + pfpri) & 2) || !(pf & 8)`.
//   5. The alpha layer is drawn last: where its 2bpp pen is non-zero it covers
//      whatever won between the motion object and the playfield.
//
// Pure combinational. The caller precomputes each layer's palette index (alpha
// base 64, playfield base 128, motion objects base 0) and pen/attribute fields;
// this module only decides the winner, so the single colour-RAM video port is
// addressed once per output pixel with the winning index.
module sys2_priority (
	// Motion object (16x16, 4bpp; pen 15 transparent).
	input  logic [3:0] mo_pen,
	input  logic [1:0] mo_priority,    // MO word 3 bits 15:14
	input  logic [7:0] mo_idx,         // palette index, base 0

	// Playfield (8x8, 4bpp; opaque background). Only pen bit 3 (pen >= 8) takes
	// part in the priority decision; the full pen reached pf_idx in the caller.
	/* verilator lint_off UNUSEDSIGNAL */
	input  logic [3:0] pf_pen,
	/* verilator lint_on UNUSEDSIGNAL */
	input  logic [1:0] pf_category,    // PF tile word bits 15:14
	input  logic [7:0] pf_idx,         // palette index, base 128

	// Alpha (8x8, 2bpp; pen 0 transparent; topmost).
	input  logic [1:0] alpha_pen,
	input  logic [7:0] alpha_idx,      // palette index, base 64

	output logic [7:0] pal_index,      // winning colour-RAM index
	output logic [1:0] layer           // 0 = playfield, 1 = motion object, 2 = alpha
);

// Motion object vs playfield (the SP15B priority circuit). Only bit 1 of the
// 0..6 priority sum is tested (the `& 2` term); the other sum bits are unused.
wire       mo_opaque = (mo_pen != 4'hf);
/* verilator lint_off UNUSEDSIGNAL */
wire [2:0] prio_sum  = {1'b0, mo_priority} + {1'b0, pf_category};
/* verilator lint_on UNUSEDSIGNAL */
wire       mo_wins_pf = mo_opaque & (~prio_sum[1] | ~pf_pen[3]);

wire [7:0] bg_idx = mo_wins_pf ? mo_idx : pf_idx;

// Alpha overlays the winner where its pen is non-zero.
wire alpha_opaque = (alpha_pen != 2'd0);

assign pal_index = alpha_opaque ? alpha_idx : bg_idx;
assign layer     = alpha_opaque ? 2'd2 : (mo_wins_pf ? 2'd1 : 2'd0);

endmodule
