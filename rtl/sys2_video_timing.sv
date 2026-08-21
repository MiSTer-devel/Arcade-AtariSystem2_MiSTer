// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

// Atari Paperboy video timing.
// Sources: SP-275 sheet 11A and MAME atarisy2.cpp commit d066f16.
module sys2_video_timing
(
	input  logic       clk_32,
	input  logic       reset,
	output logic       ce_pix,
	output logic [9:0] h_count,
	output logic [8:0] v_count,
	output logic       hblank,
	output logic       hsync,
	output logic       vblank,
	output logic       vsync,
	output logic       scanline_irq,
	output logic       vblank_irq
);

localparam logic [9:0] H_ACTIVE   = 10'd512;
localparam logic [9:0] H_TOTAL    = 10'd640;
localparam logic [9:0] H_LAST     = 10'd639;
localparam logic [9:0] H_SYNC_BEG = 10'd544;
localparam logic [9:0] H_SYNC_END = 10'd608;

localparam logic [8:0] V_ACTIVE   = 9'd384;
localparam logic [8:0] V_TOTAL    = 9'd416;
localparam logic [8:0] V_LAST     = 9'd415;
localparam logic [8:0] V_SYNC_BEG = 9'd392;
localparam logic [8:0] V_SYNC_END = 9'd400;

logic [8:0] next_v;

always_comb begin
	hblank = h_count >= H_ACTIVE;
	hsync = (h_count >= H_SYNC_BEG) && (h_count < H_SYNC_END);
	vblank = v_count >= V_ACTIVE;
	vsync = (v_count >= V_SYNC_BEG) && (v_count < V_SYNC_END);

	if (v_count == V_LAST) next_v = 0;
	else                        next_v = v_count + 1'd1;
end

always_ff @(posedge clk_32) begin
	if (reset) begin
		ce_pix <= 0;
		h_count <= H_LAST;
		v_count <= V_LAST;
		scanline_irq <= 0;
		vblank_irq <= 0;
	end else begin
		ce_pix <= ~ce_pix;
		scanline_irq <= 0;
		vblank_irq <= 0;

		// Advance on the edge that asserts CE_PIXEL. This makes all raster
		// outputs stable for the complete 16 MHz pixel-enable interval.
		if (!ce_pix) begin
			if (h_count == H_LAST) begin
				h_count <= 0;
				v_count <= next_v;
				scanline_irq <= (next_v[5:0] == 0);
				vblank_irq <= (next_v == V_ACTIVE);
			end else begin
				h_count <= h_count + 1'd1;
			end
		end
	end
end

endmodule
