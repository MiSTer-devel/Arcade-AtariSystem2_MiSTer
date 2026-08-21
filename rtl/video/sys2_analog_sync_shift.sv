// SPDX-License-Identifier: GPL-3.0-or-later
`timescale 1ns/1ps

//============================================================================
//  sys2_analog_sync_shift.sv -- "Analog VGA H-Shift / V-Shift" OSD controls.
//
//  Ported from Arcade-Raiden_MiSTer by Umberto Parisi (rmonic79), GPL-3.0.
//  Raiden delays the sync signals through a shift register as deep as the whole
//  raster (H_TOTAL flops plus an H_TOTAL:1 mux, and the same again per line for
//  V), because a negative shift is a delay of nearly a full period. HSync and
//  VSync are pure functions of the raster counters, though, so delaying the
//  waveform by N is identical to evaluating the sync equation N positions
//  earlier -- two subtractors and two range compares instead of ~1000 flops and
//  two wide muxes. Paperboy's raster is 640x416 where Raiden's is 320x263, so
//  the shift-register form would have cost roughly four times as much here.
//
//  Moving sync moves the picture on a CRT: the tube's sweep is triggered by
//  sync, so sliding the pulse slides where the active video lands in the sweep.
//  The image itself is untouched -- nothing is resampled and no pixel changes
//  value. The limit is the blanking budget: push sync far enough and it lands
//  inside active video, where the monitor will blank part of the picture.
//
//  PIPE accounts for the core's sync pipeline: sys2_video_render carries
//  hsync/vsync alongside the pixels it is composing, so the sync leaving the
//  renderer is already PIPE ce_pix ticks behind the raster counters. This module
//  reads the counters combinationally and registers its result, which is one of
//  those ticks, so it walks back PIPE-1 more. At hshift == vshift == 0 the
//  outputs are therefore bit-identical to the renderer's own hsync_o/vsync_o --
//  simulation asserts exactly that against the real sys2_video_timing +
//  sys2_video_render pair.
//============================================================================

module sys2_analog_sync_shift #(
	parameter int H_TOTAL    = 640,
	parameter int H_SYNC_BEG = 544,
	parameter int H_SYNC_END = 608,
	parameter int V_TOTAL    = 416,
	parameter int V_SYNC_BEG = 392,
	parameter int V_SYNC_END = 400,
	parameter int PIPE       = 4
) (
	input  logic              clk,
	input  logic              reset,
	input  logic              ce_pix,

	input  logic [9:0]        h_count,
	input  logic [8:0]        v_count,

	// Signed shifts. >0 delays the pulse (picture moves one way on the tube),
	// <0 advances it. Range is bounded by the caller's menu, not here.
	input  logic signed [6:0] hshift,   // pixels
	input  logic signed [6:0] vshift,   // lines

	output logic              hs_out,
	output logic              vs_out
);

// Sized forms of the int parameters, so every comparison below is width-exact.
localparam signed [11:0] PIPE_ADJ = 12'(PIPE - 1);
localparam signed [11:0] HT_S     = 12'(H_TOTAL);
localparam signed [11:0] VT_S     = 12'(V_TOTAL);
localparam        [9:0]  HSB      = 10'(H_SYNC_BEG);
localparam        [9:0]  HSE      = 10'(H_SYNC_END);
localparam        [8:0]  VSB      = 9'(V_SYNC_BEG);
localparam        [8:0]  VSE      = 9'(V_SYNC_END);

wire signed [11:0] hsh = $signed({{5{hshift[6]}}, hshift});
wire signed [11:0] vsh = $signed({{5{vshift[6]}}, vshift});

// Raster column whose sync state belongs on the output this tick.
wire signed [11:0] h_back = $signed({2'd0, h_count}) - PIPE_ADJ - hsh;
wire               h_uf   = (h_back < 0);                       // walked past the start of the line
wire        [9:0]  h_ref  = h_uf                ? 10'(h_back + HT_S) :
                            (h_back >= HT_S)    ? 10'(h_back - HT_S) :
                                                  10'(h_back);

// A horizontal underflow means the sample belongs to the previous line, so the row borrows -- the
// same thing the renderer's own sync pipeline does when it carries sync across a line edge.
wire signed [11:0] v_back = $signed({3'd0, v_count}) - $signed({11'd0, h_uf}) - vsh;
wire        [8:0]  v_ref  = (v_back < 0)        ? 9'(v_back + VT_S) :
                            (v_back >= VT_S)    ? 9'(v_back - VT_S) :
                                                  9'(v_back);

always_ff @(posedge clk) begin
	if (reset) begin
		hs_out <= 1'b0;
		vs_out <= 1'b0;
	end else if (ce_pix) begin
		hs_out <= (h_ref >= HSB) && (h_ref < HSE);
		vs_out <= (v_ref >= VSB) && (v_ref < VSE);
	end
end

endmodule
