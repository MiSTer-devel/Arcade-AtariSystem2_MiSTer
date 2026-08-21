// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Paperboy motion-object double-buffered scanline line buffer
// Two 512-entry buffers ping-pong
// by the display-line parity: while the renderer reads the DISPLAY buffer for the
// current line, the walker composites the NEXT line into the BUILD buffer. At each
// line the roles swap (driven by `buf_sel` = v_count[0] from the engine).
//
// Each entry is {priority[1:0], color[1:0], pen[3:0]} (8 bits); pen 15 is the
// transparent/empty value (CLEAR = 0x0f). The display side is read-and-CLEAR: a
// column is returned to the renderer (one ce_pix registered latency, like the
// other video read ports) and one pixel later written back to transparent, so the
// buffer the walker inherits next line is already empty -- no separate clear pass
// is needed (the hardware "line buffer erased as it is displayed"). A reset
// init-clear sweeps both buffers so the first frame is clean.
//
// Each buffer is one 8-bit array with a single write port (clear when displaying,
// walker composite when building, init-clear on reset) and a registered read port
// -> a clean simple-dual-port M10K inference.
module sys2_mob_linebuf (
	input  logic       clk,
	input  logic       reset,
	input  logic       ce_pix,

	input  logic       buf_sel,    // 0: buf0 displays / buf1 builds; 1: swapped

	// Display side (renderer): read-and-clear, one ce_pix-latency registered read.
	input  logic [8:0] rd_col,
	input  logic       rd_en,      // active-display read strobe (level, qualified by ce_pix)
	output logic [7:0] rd_data,

	// Build side (walker compositing into the non-display buffer), full clk rate.
	input  logic       wr_en,
	input  logic [8:0] wr_col,
	input  logic [7:0] wr_data,

	output logic       ready       // reset init-clear complete
);

localparam logic [7:0] CLEAR = 8'h0f;   // priority 0, color 0, pen 15 (transparent)

// ramstyle pins both buffers to M10K so the over-capacity RAM balancer (Quartus
// caps inferred block RAM at the device's 553 M10K and spills the rest to flip-
// flops) never picks these per-line MO buffers as a spill victim. Read-and-clear
// never reads and writes the same column in one cycle (clear lags by one pixel),
// so no_rw_check is safe. (With the big ROMs in SDRAM, total M10K demand sits below
// 553 anyway; the ramstyle pin is belt and braces.)
(* ramstyle = "no_rw_check, M10K" *) logic [7:0] buf0 [0:511];
(* ramstyle = "no_rw_check, M10K" *) logic [7:0] buf1 [0:511];

// ---- reset init-clear sweep of both buffers (512 cycles) ----
logic [9:0] init_cnt;
logic       init_active;
always_ff @(posedge clk) begin
	if (reset) begin
		init_cnt    <= '0;
		init_active <= 1'b1;
	end else if (init_active) begin
		if (init_cnt[8:0] == 9'd511) init_active <= 1'b0;
		init_cnt <= init_cnt + 10'd1;
	end
end
assign ready = ~init_active;

// ---- read-and-clear bookkeeping (display side advances on ce_pix) ----
// Clear the column read on the PREVIOUS pixel, so the clear never collides with
// the column currently being read. prev_valid is flushed when reads stop (HBLANK)
// so the last active column is still cleared and no stale clear leaks across lines.
logic [8:0] prev_col;
logic       prev_valid;
always_ff @(posedge clk) begin
	if (reset) begin
		prev_col   <= '0;
		prev_valid <= 1'b0;
	end else if (ce_pix) begin
		if (rd_en) begin
			prev_col   <= rd_col;
			prev_valid <= 1'b1;
		end else begin
			prev_valid <= 1'b0;
		end
	end
end
wire clr_fire = ce_pix && prev_valid;   // clear prev_col on the display buffer

// ---- per-buffer write source mux: init / clear (display) / composite (build) ----
wire disp0 = (buf_sel == 1'b0);

wire        b0_we   = init_active ? 1'b1 : (disp0 ? clr_fire : wr_en);
wire [8:0]  b0_addr = init_active ? init_cnt[8:0] : (disp0 ? prev_col : wr_col);
wire [7:0]  b0_data = init_active ? CLEAR : (disp0 ? CLEAR : wr_data);

wire        b1_we   = init_active ? 1'b1 : (disp0 ? wr_en : clr_fire);
wire [8:0]  b1_addr = init_active ? init_cnt[8:0] : (disp0 ? wr_col : prev_col);
wire [7:0]  b1_data = init_active ? CLEAR : (disp0 ? wr_data : CLEAR);

// Registered reads (every clk, 1-clk latency); rd_col is held across the pixel so
// the data is stable by the next ce_pix, matching the alpha/playfield read ports.
logic [7:0] b0_q, b1_q;
logic       buf_sel_r;
always_ff @(posedge clk) begin
	if (b0_we) buf0[b0_addr] <= b0_data;
	if (b1_we) buf1[b1_addr] <= b1_data;
	b0_q <= buf0[rd_col];
	b1_q <= buf1[rd_col];
	if (ce_pix) buf_sel_r <= buf_sel;
end

assign rd_data = buf_sel_r ? b1_q : b0_q;

endmodule
