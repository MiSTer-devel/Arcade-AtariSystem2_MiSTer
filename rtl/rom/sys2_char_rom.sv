// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Alpha (character) ROM. 16 KiB of 8-bit BRAM written by a top-level decode of
// the chars byte stream (region 5) and read
// by the alpha renderer. The region is a single contiguous 0x4000-byte window
// (no holes, no transform), so storage is a flat byte array.
//
// Format (Atari System 1 `anlayout`): 8x8, 2 bpp, 16 bytes per character, up to
// 1024 characters. Each character row is two bytes (byte 0 = pixels 0-3, byte 1 =
// pixels 4-7); within a byte the four pixels are MSB-first, so pixel i (0-3) has
// 2-bpp pen {byte[7-i], byte[3-i]} (leftmost pixel = bit 3 plane 0 / bit 7 plane 1).
// The renderer addresses bytes as char_code*16 + row*2 + (pixel>=4), so an 8-bit
// byte port is the natural read width. (An earlier note here said pixel i uses
// {byte[i+4],byte[i]} -- ascending -- which scrambled all on-screen text.)
//
// Single clock: the loader write port and the renderer read port both run in the
// video/download domain (clk_sys). Written one whole byte per strobe (no byte
// lanes) and read with one-cycle latency -> a clean simple-dual-port M10K.
module sys2_char_rom (
	input  logic        clk,

	// Loader write port (region 5 = chars; one byte per strobe).
	input  logic        ld_we,
	input  logic [13:0] ld_addr,   // 0x0000-0x3fff
	input  logic [7:0]  ld_data,

	// Renderer read port: registered one-cycle.
	input  logic [13:0] char_addr,
	output logic [7:0]  char_data
);

// 16 KiB: Paperboy populates only the first 8 KiB (512 chars) and the rest is 0xff
// fill, but 720/ssprint/csprint/apb all ship 0x4000 and MAME masks the alpha code to
// 10 bits (`code = data & 0x3ff`) for every game in the driver.
localparam int BYTES = 'h4000;     // 16 KiB

logic [7:0] rom [0:BYTES-1];

// ld_we/ld_addr/ld_data are predecoded per region and registered at the source
// (atarisys2.sv, clk_sys) before the long route to this physically-distant BRAM, so this
// is a clean register-to-BRAM write hop -- the registered-address load-path rule proven on silicon.
always_ff @(posedge clk) begin
	if (ld_we) rom[ld_addr] <= ld_data;
	char_data <= rom[char_addr];
end

endmodule
