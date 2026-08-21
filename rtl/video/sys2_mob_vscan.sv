// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Paperboy motion-object vertical scan test: for one decoded object and one
// screen scanline, does the object cover that line, and if so what is the
// sprite-internal row? This is the subtle, MAME-derived coordinate math, locked
// into a tested module so the sprite layer cannot repeat the playfield colour
// bug's "shapes fine, placement wrong" class of error.
//
// From MAME atarimo.cpp render_object (atarisy2: m_xoffset=m_xscroll=m_yscroll=0,
// no vflip, no width; motion objects are screen-absolute):
//   ypos  = -Y_field;
//   ypos -= height << 4;        // height in 16px tiles
//   ypos &= 0x1ff;              // bitmapheight = round_pow2(0x7fc0>>6 = 0x1ff) = 512
//   if (ypos >= 384) ypos -= 512;   // bitmap.height() = 384 visible; m_bitmapheight = 512
//   ... object drawn at rows [ypos, ypos + height*16)
// So ypos_top = ((-(Y_field + height*16)) mod 512) signed-folded into [-128,383].
// v_in_sprite (0..height*16-1) is the row within the stacked tiles; the engine
// takes tile-row = v_in_sprite>>4 (added to the base code) and sub-row =
// v_in_sprite[3:0].
module sys2_mob_vscan (
	input  logic [8:0] y_field,      // (word0 & 0x7fc0) >> 6
	input  logic [3:0] height,       // height in 16px tiles, 1..8 (decoded field + 1)
	input  logic [8:0] scanline,     // current screen scanline, 0..383
	output logic       on_line,      // object covers this scanline
	output logic [6:0] v_in_sprite   // scanline - ypos_top, 0..127 (valid when on_line)
);

logic [7:0]         h16;       // height * 16 (16..128)
logic [8:0]         sum9;      // (Y_field + h16) mod 512
logic [8:0]         yneg;      // (-(Y_field + h16)) mod 512
logic signed [10:0] ypos_top;  // top row, [-128, 383]
logic signed [10:0] delta;     // scanline - ypos_top

always_comb begin
	h16      = {height, 4'd0};                       // height*16
	sum9     = y_field + {1'b0, h16};                // truncation = mod 512
	yneg     = 9'd0 - sum9;                           // mod-512 negate
	ypos_top = (yneg >= 9'd384) ? ($signed({2'b0, yneg}) - 11'sd512)
	                            : $signed({2'b0, yneg});
	delta    = $signed({2'b0, scanline}) - ypos_top;
	on_line  = (delta >= 0) && (delta < $signed({3'b0, h16}));
	v_in_sprite = delta[6:0];
end

endmodule
