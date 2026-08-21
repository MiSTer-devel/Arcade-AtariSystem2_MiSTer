// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Shared 4-pixel 4bpp pen kernel for the Atari System II planar packing
// (MAME RGN_FRAC(1,2): planeoffset { 0, 4, RGN_FRAC(1,2)+0, RGN_FRAC(1,2)+4 },
// MSB-first within each byte). A 4-pixel group is two bytes: the FIRST region
// half supplies pen bits 3,2 and the SECOND half pen bits 1,0. For group pixel
// c (0 = leftmost), with b = 3-c:
//   pen = { first[b+4], first[b], second[b+4], second[b] }.
//
// This is the single source of truth for the playfield (8x8 tiles) and the
// motion-object (16x16 tiles) pens, so
// the plane-order colour bug cannot reappear independently in two
// layers. The four pens are spelled out with constant bit-selects (the verified
// form from the MAME reference) -- pure combinational.
module sys2_pen4 (
	input  logic [7:0] first_byte,   // FIRST region half (pen bits 3,2)
	input  logic [7:0] second_byte,  // SECOND region half (pen bits 1,0)
	output logic [3:0] pen0,         // group pixel 0 (leftmost), c=0 -> b=3
	output logic [3:0] pen1,         // c=1 -> b=2
	output logic [3:0] pen2,         // c=2 -> b=1
	output logic [3:0] pen3          // c=3 -> b=0
);

assign pen0 = {first_byte[7], first_byte[3], second_byte[7], second_byte[3]};
assign pen1 = {first_byte[6], first_byte[2], second_byte[6], second_byte[2]};
assign pen2 = {first_byte[5], first_byte[1], second_byte[5], second_byte[1]};
assign pen3 = {first_byte[4], first_byte[0], second_byte[4], second_byte[0]};

endmodule
