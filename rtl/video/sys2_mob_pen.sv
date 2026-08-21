// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Paperboy motion-object 16x16 `molayout` row pen kernel.
// One 16-pixel sprite row is 4 bytes from each region half (one byte per 4-pixel
// group). molayout xoffset = { 0,1,2,3, 8,9,10,11, 16,17,18,19, 24,25,26,27 } is
// four groups of the same 4-pixel structure as the playfield, so the nibble
// packing is identical (shared sys2_pen4) -- group g uses first[g]/second[g].
//
//   sprite pixel p (0=leftmost): g = p>>2, q = p&3  ->  pen = pen4(first[g],second[g])[q]
//   H flip mirrors the 16 columns: screen offset k shows sprite pixel hflip?15-k:k
//
// The two half-rows arrive as 4 packed bytes each (group g = bits 8*g +: 8), the
// natural form of a 4-byte-per-half sprite-ROM burst read (the abstract,
// latency-tolerant port the engine drives). Output is the 16 pens already in
// left-to-right screen order so the compositing walker just indexes by column.
// Pen 15 is transparent. Pure combinational.
module sys2_mob_pen (
	input  logic [31:0] row_first,   // FIRST half: first[g] = row_first[8*g +: 8]
	input  logic [31:0] row_second,  // SECOND half: second[g] = row_second[8*g +: 8]
	input  logic        hflip,
	output logic [63:0] pens_screen  // pens_screen[4*k +: 4] = pen at screen offset k (0..15)
);

// Per-group pens -> the 16 sprite-space pens (p = 4*g + q, left to right).
logic [3:0] sp_pen [0:15];

genvar g;
generate
	for (g = 0; g < 4; g = g + 1) begin : grp
		logic [3:0] q0, q1, q2, q3;
		sys2_pen4 u_pen4 (
			.first_byte (row_first [8*g +: 8]),
			.second_byte(row_second[8*g +: 8]),
			.pen0(q0), .pen1(q1), .pen2(q2), .pen3(q3)
		);
		assign sp_pen[4*g + 0] = q0;
		assign sp_pen[4*g + 1] = q1;
		assign sp_pen[4*g + 2] = q2;
		assign sp_pen[4*g + 3] = q3;
	end
endgenerate

// Map sprite-space pens to screen order, applying H flip.
genvar k;
generate
	for (k = 0; k < 16; k = k + 1) begin : col
		assign pens_screen[4*k +: 4] = hflip ? sp_pen[15 - k] : sp_pen[k];
	end
endgenerate

endmodule
