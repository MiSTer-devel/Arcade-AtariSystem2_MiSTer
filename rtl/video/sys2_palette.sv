// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Paperboy palette colour conversion. A color RAM word is RRRRGGGGBBBBIIII: a
// 4-bit colour nibble per channel and a shared 4-bit intensity. Each channel is
// (colormap[nibble] * intensity[I]) >> 4, using the resistor/intensity ladders
// from schematic sheets SP16A-SP17A (integer reference values from MAME's atarisy2_v.cpp).
// Pure combinational; the caller registers it in the pixel pipeline as needed.
//
// Output range is 0..240 (brightest = 15 * 256 >> 4); this is the documented
// reference, not full-scale 255.
module sys2_palette (
	input  logic [15:0] color_word,   // RRRRGGGGBBBBIIII from color RAM
	output logic [7:0]  r,
	output logic [7:0]  g,
	output logic [7:0]  b
);

// Intensity ladder: ZB=115, Z3=78, Z2=37, Z1=17, Z0=9.
function automatic logic [8:0] intensity(input logic [3:0] i);
	case (i)
		4'h0: intensity = 9'd0;
		4'h1: intensity = 9'd124;   // ZB+Z0
		4'h2: intensity = 9'd132;   // ZB+Z1
		4'h3: intensity = 9'd141;   // ZB+Z1+Z0
		4'h4: intensity = 9'd152;   // ZB+Z2
		4'h5: intensity = 9'd161;   // ZB+Z2+Z0
		4'h6: intensity = 9'd169;   // ZB+Z2+Z1
		4'h7: intensity = 9'd178;   // ZB+Z2+Z1+Z0
		4'h8: intensity = 9'd193;   // ZB+Z3
		4'h9: intensity = 9'd202;   // ZB+Z3+Z0
		4'ha: intensity = 9'd210;   // ZB+Z3+Z1
		4'hb: intensity = 9'd219;   // ZB+Z3+Z1+Z0
		4'hc: intensity = 9'd230;   // ZB+Z3+Z2
		4'hd: intensity = 9'd239;   // ZB+Z3+Z2+Z0
		4'he: intensity = 9'd247;   // ZB+Z3+Z2+Z1
		4'hf: intensity = 9'd256;   // ZB+Z3+Z2+Z1+Z0
	endcase
endfunction

// Colour ladder: 0,3,4,5,6,7,8,9,10,11,12,13,14,14,15,15.
// Colour steps -- the one palette value still resting on MAME rather than the schematic.
//
// MAME's `color_table` is a bare hand-written array with no derivation comment, unlike its
// `intensity_table` which is built from named conductances (round 4 proved those ARE the branch
// conductances normalised to 256). Our own ngspice DC solve of the transcribed circuit
// disagrees with it: a much larger
// pedestal at code 1 and near-uniform ~0.65 steps, with the irregularity at the MSB transition
// (7->8) instead of at the top.
//
// We ship MAME's table anyway, deliberately. MAME is the only oracle we can actually validate
// against -- the frame comparison is pixel-exact against it on three scenes -- so swapping in
// a table validated by nothing would break that comparison and trade a known-good approximation
// for an unproven one. Freshly derived component values have also turned out wrong here three
// times (the intensity resistors twice, R28 once).
//
// `PALETTE_SPICE_COLORMAP` exists so the question can be settled on hardware instead of argued:
// build with it defined, run the two RBFs on a real cabinet next to each other, and see which
// matches. Expect the MAME frame comparison to fail when it is defined -- that is correct and expected,
// because that check measures agreement with MAME, which is exactly what this option changes.
//   Default (undefined) = MAME:  0,3,4,5,6,7,8,9,10,11,12,13,14,14,15,15
//   PALETTE_SPICE_COLORMAP       0,6,7,7,8,9,9,10,10,11,12,12,13,14,14,15
function automatic logic [4:0] colormap(input logic [3:0] c);
`ifdef PALETTE_SPICE_COLORMAP
	// Schematic-derived (ngspice DC solve, values rounded from 0.00/5.79/6.52/7.17/7.87/8.52/
	// 9.25/9.89/10.31/10.96/11.68/12.32/13.01/13.65/14.36/15.00). Monotonic, verified.
	case (c)
		4'h0: colormap = 5'd0;
		4'h1: colormap = 5'd6;
		4'h2: colormap = 5'd7;
		4'h3: colormap = 5'd7;
		4'h4: colormap = 5'd8;
		4'h5: colormap = 5'd9;
		4'h6: colormap = 5'd9;
		4'h7: colormap = 5'd10;
		4'h8: colormap = 5'd10;
		4'h9: colormap = 5'd11;
		4'ha: colormap = 5'd12;
		4'hb: colormap = 5'd12;
		4'hc: colormap = 5'd13;
		4'hd: colormap = 5'd14;
		4'he: colormap = 5'd14;
		4'hf: colormap = 5'd15;
	endcase
`else
	case (c)
		4'h0: colormap = 5'd0;
		4'h1: colormap = 5'd3;
		4'h2: colormap = 5'd4;
		4'h3: colormap = 5'd5;
		4'h4: colormap = 5'd6;
		4'h5: colormap = 5'd7;
		4'h6: colormap = 5'd8;
		4'h7: colormap = 5'd9;
		4'h8: colormap = 5'd10;
		4'h9: colormap = 5'd11;
		4'ha: colormap = 5'd12;
		4'hb: colormap = 5'd13;
		4'hc: colormap = 5'd14;
		4'hd: colormap = 5'd14;
		4'he: colormap = 5'd15;
		4'hf: colormap = 5'd15;
	endcase
`endif
endfunction

logic [3:0] i_nib;
assign i_nib = color_word[3:0];

// channel = (colormap[nibble] * intensity[I]) >> 4. The colour operand is widened
// to 12 bits so the product (max 15 * 256 = 3840) is computed at 12 bits rather
// than the multiply's self-determined 9 bits (which would wrap); (>> 4) is then at
// most 240 and the explicit 8'() states the intended channel width.
assign r = 8'((12'(colormap(color_word[15:12])) * intensity(i_nib)) >> 4);
assign g = 8'((12'(colormap(color_word[11:8]))  * intensity(i_nib)) >> 4);
assign b = 8'((12'(colormap(color_word[7:4]))   * intensity(i_nib)) >> 4);

endmodule
