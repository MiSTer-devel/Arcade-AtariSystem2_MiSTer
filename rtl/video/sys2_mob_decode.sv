// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Paperboy motion-object (sprite) entry field decoder. Each motion object is
// four 16-bit words in motion-object RAM (VMMU view 0, 0x3800-0x3fff = 256
// entries x 4 words). The field masks are the MAME `atari_motion_objects_config`
// s_mob_config for atarisy2 (atarisy2_v.cpp @ d066f16), cross-checked against
// the schematics. Attribute word layout:
//
//   link      word3 & 0x07f8   next entry index (>>3 -> 0..255)
//   code      (word0 & 0x0007)<<11 | (word1 & 0x07ff)   14-bit tile code
//   color     word3 & 0x3000   2-bit colour group (palette base 0)
//   xpos      word2 & 0xffc0   >>6 -> 10-bit X
//   ypos      word0 & 0x7fc0   >>6 -> 9-bit Y
//   height    word1 & 0x3800   >>11, +1 -> 1..8 tiles of 16 px
//   hflip     word1 & 0x4000
//   neighbor  word1 & 0x8000   (config: does NOT affect the next object)
//   priority  word3 & 0xc000   2-bit MO priority (for sys2_priority)
//
// There is no vertical-flip or width field in this hardware (both masks are 0 in
// s_mob_config). Pure combinational; the list walker registers what it needs.
module sys2_mob_decode (
	// Some word bits are not part of any field in the atarisy2 s_mob_config
	// (e.g. word2[5:0], word3[2:0]); they are intentionally unused here.
	/* verilator lint_off UNUSEDSIGNAL */
	input  logic [15:0] word0,
	input  logic [15:0] word1,
	input  logic [15:0] word2,
	input  logic [15:0] word3,
	/* verilator lint_on UNUSEDSIGNAL */

	output logic [7:0]  link,        // next entry index (0..255)
	output logic [13:0] code,        // tile code (storage masks to its tile count)
	output logic [1:0]  color,       // colour group
	output logic [9:0]  xpos,        // X position field
	output logic [8:0]  ypos,        // Y position field
	output logic [3:0]  height,      // height in 16px tiles, 1..8 (field + 1)
	output logic        hflip,
	output logic        neighbor,
	output logic [1:0]  priority_lvl // MO priority for the SP15B mixer
);

assign link         = word3[10:3];
assign code         = {word0[2:0], word1[10:0]};
assign color        = word3[13:12];
assign xpos         = word2[15:6];
assign ypos         = word0[14:6];
assign height       = {1'b0, word1[13:11]} + 4'd1;
assign hflip        = word1[14];
assign neighbor     = word1[15];
assign priority_lvl = word3[15:14];

endmodule
