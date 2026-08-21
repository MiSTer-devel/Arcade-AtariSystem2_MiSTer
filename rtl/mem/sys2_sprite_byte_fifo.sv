// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Robust pacing FIFO for the raw-ioctl sprite SDRAM write. On hardware, feeding the loader_writer
// from the raw ioctl stream with a per-byte COMBINATIONAL ioctl_wait (spw_busy asserted in the SAME
// cycle as the word-completing byte) dropped every odd byte on silicon -- bytes arrived but no
// 16-bit word was ever formed, so nothing was ever written. Reactive
// per-byte backpressure races hps_io. The loader's proven pattern is a FIFO + MARGIN: assert ioctl_wait
// from the registered occupancy BEFORE overflow, never reactively. This tiny register FIFO (not an
// inferred RAM) buffers the incoming sprite bytes and drains them to the loader_writer paced by the
// writer's (registered) busy, so the writer always sees a clean even/odd byte pair and ioctl_wait
// never races the current byte.
module sys2_sprite_byte_fifo #(
	// Width of the region offset carried alongside each byte. The top level instantiates
	// OFF_W=20, wide enough for the largest region it routes through this FIFO (the 1 MiB
	// sprite image; maincpu and tiles are smaller). The default 18 is the original
	// sprite-only sizing. The datapath itself was fixed after a silicon bug
	// (the dropped word-completing byte) and is not something to perturb casually.
	parameter int OFF_W = 18
) (
	input  logic        clk,
	// raw-ioctl sprite byte stream in (sprite region decoded + XOR-0xff applied upstream)
	input  logic        in_we,
	input  logic [OFF_W-1:0] in_off,
	input  logic [7:0]  in_data,
	output logic        wait_req,      // -> ioctl_wait (margin-based, from registered occupancy)
	// paced byte stream out to the loader_writer; gated by its (registered) busy
	input  logic        writer_busy,   // loader_writer wr_busy
	output logic        out_we,
	output logic [OFF_W-1:0] out_off,
	output logic [7:0]  out_data,
	// STICKY: a byte arrived while the FIFO was FULL and was DROPPED on the floor.
	//
	// `push = in_we && !full` discards silently, and nothing has ever observed it. The margin
	// below is what is supposed to make it impossible -- but the margin was sized and
	// silicon-validated when this FIFO carried the 256 KiB SPRITE region alone. The family
	// build routes the maincpu, tiles and sprite regions through it -- roughly eight times
	// the traffic through the same 16 entries. Observation only: this output changes no behaviour,
	// it just makes the drop visible instead of silent.
	output logic        overflow
);

// DEPTH=16 / margin 4 is silicon-validated: on hardware the full inverted sum of the original
// 256 KiB sprite image matched, so the FIFO delivered every byte to the writer with no overflow.
// (The grey-with-colour sprite artifact was NOT a write/drop problem here -- it was the sprite
// fetcher's SDRAM burst-read addressing, since fixed.)
localparam int DEPTH  = 16;
localparam int PW     = 4;            // log2(DEPTH)
localparam int MARGIN = 4;            // free slots held in reserve before asserting wait
localparam int EW     = OFF_W + 8;    // {off[OFF_W-1:0], data[7:0]}

/* verilator lint_off PROCASSINIT */
logic [EW-1:0] fifo [0:DEPTH-1];
logic [PW:0]   wptr = '0, rptr = '0;
logic          writer_busy_q = 1'b0;
logic          overflow_r = 1'b0;
/* verilator lint_on PROCASSINIT */

wire [PW:0] count = wptr - rptr;
wire        empty = (wptr == rptr);
wire        full  = (count == DEPTH[PW:0]);
wire        push  = in_we && !full;
// Drain when the writer is not (registered-)busy: registering writer_busy breaks the combinational
// loop (writer_busy depends combinationally on out_we via the writer's word_pop).
wire        pop   = !empty && !writer_busy_q;

// Assert wait with MARGIN free slots so in-flight HPS bytes (post-ioctl_wait latency) never overflow.
assign wait_req = (count >= (DEPTH[PW:0] - MARGIN[PW:0]));

assign overflow = overflow_r;

always_ff @(posedge clk) begin
	writer_busy_q <= writer_busy;
	if (in_we && full) overflow_r <= 1'b1;   // sticky from power-on; never cleared
	if (push) begin
		fifo[wptr[PW-1:0]] <= {in_off, in_data};
		wptr <= wptr + 1'b1;
	end
	if (pop) rptr <= rptr + 1'b1;
end

assign out_we               = pop;
assign {out_off, out_data}  = fifo[rptr[PW-1:0]];

endmodule
