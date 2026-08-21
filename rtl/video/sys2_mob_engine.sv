// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Paperboy motion-object line engine. Hardware-accurate scanline line-buffer
// engine (not a frame
// renderer): each scanline it walks the active list and composites every object
// covering the NEXT line into the build half of a double-buffered line buffer;
// the renderer reads the display half per pixel into sys2_priority.
//
// During display of line N (buf_sel = v_count[0]) the walker builds line N+1 into
// the other buffer, so it has the whole line period as its budget; at the next
// line the buffers swap. build_line wraps 415->0 so line 0 is built during the
// last VBLANK line; VBLANK targets (>=384) are skipped. The sprite-ROM port is the
// abstract, latency-tolerant request/valid interface the SDRAM graphics path plugs into.
module sys2_mob_engine #(
	parameter int VTOTAL  = 416,    // lines per frame (last line = VTOTAL-1)
	parameter int VACTIVE = 384     // active display lines 0..VACTIVE-1
) (
	input  logic        clk,        // 32 MHz video clock (clk_sys)
	input  logic        reset,
	input  logic        ce_pix,     // 16 MHz pixel enable
	/* verilator lint_off UNUSEDSIGNAL */
	input  logic [9:0]  h_count,    // only h_count==0 (line start) is used
	/* verilator lint_on UNUSEDSIGNAL */
	input  logic [8:0]  v_count,

	input  logic [7:0]  start_link, // list head (object-zero/link latch; tie 0)

	// Motion-object RAM read port (sys2_main_bus mob_video_*; 1-cyc latency).
	output logic [9:0]  mob_video_addr,
	input  logic [15:0] mob_video_data,

	// Abstract, latency-tolerant sprite-ROM row read port.
	output logic        sp_req,
	output logic [13:0] sp_tile,
	output logic [3:0]  sp_row,
	input  logic        sp_valid,
	input  logic [63:0] sp_data,    // {row_second[31:0], row_first[31:0]}

	// Renderer read port: read-and-clear, one ce_pix registered latency. Returns
	// {priority[1:0], color[1:0], pen[3:0]}; pen 15 = no object (transparent).
	input  logic [8:0]  rd_col,
	input  logic        rd_en,
	output logic [7:0]  rd_data
);

// ---- per-line build control ----
wire        line_start = ce_pix && (h_count == 10'd0);
wire [8:0]  build_line = (v_count == 9'(VTOTAL-1)) ? 9'd0 : (v_count + 9'd1);
wire        build_active = (build_line < 9'(VACTIVE));
wire        lb_ready;
wire        start = line_start && build_active && lb_ready;

// Display buffer parity: during line N the renderer reads buf[N[0]] while the
// walker fills buf[~N[0]] for line N+1.
wire        buf_sel = v_count[0];

// ---- walker -> line-buffer build write ----
wire        lb_we;
wire [8:0]  lb_col;
wire [7:0]  lb_data;

sys2_mob_walker u_walker (
	.clk(clk), .reset(reset),
	.start(start), .build_line(build_line), .start_link(start_link),
	.mob_addr(mob_video_addr), .mob_data(mob_video_data),
	.sp_req(sp_req), .sp_tile(sp_tile), .sp_row(sp_row),
	.sp_valid(sp_valid), .sp_data(sp_data),
	.lb_we(lb_we), .lb_col(lb_col), .lb_data(lb_data),
	/* verilator lint_off PINCONNECTEMPTY */
	.busy()
	/* verilator lint_on PINCONNECTEMPTY */
);

sys2_mob_linebuf u_linebuf (
	.clk(clk), .reset(reset), .ce_pix(ce_pix), .buf_sel(buf_sel),
	.rd_col(rd_col), .rd_en(rd_en), .rd_data(rd_data),
	.wr_en(lb_we), .wr_col(lb_col), .wr_data(lb_data),
	.ready(lb_ready)
);

endmodule
