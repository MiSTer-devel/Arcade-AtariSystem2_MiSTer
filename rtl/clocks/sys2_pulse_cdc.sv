// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Single-event pulse clock-domain crossing. A one-cycle pulse in the source
// domain produces exactly one one-cycle pulse in the destination domain,
// regardless of the relative clock rates, by crossing a level toggle through a
// two-flop synchronizer and edge-detecting it in the destination domain.
//
// This began as the explicit CDC for events crossing between what were separate
// video, CPU and sound clocks. Those domains have since been collapsed onto one
// 32 MHz clock (see sys2_clocks), so every remaining instance crosses between
// two aliases of the same clock: the handshake degenerates to a fixed few-cycle
// delay, and it is kept for its explicit contract rather than rewired.
// `sys2_video_timing` still emits `scanline_irq`/`vblank_irq` as one-cycle
// pulses through it.
//
// Correctness relies on source events being spaced farther apart than the
// destination synchronizer latency (a few dst clocks). Paperboy's video and
// sound events (tens of microseconds apart at the closest) satisfy this with
// large margin; do not use this primitive for back-to-back single-cycle bursts.
module sys2_pulse_cdc (
	input  logic src_clk,
	input  logic src_reset,
	input  logic src_pulse,

	input  logic dst_clk,
	input  logic dst_reset,
	output logic dst_pulse
);

// Source: flip a level on each event.
logic src_toggle;
always_ff @(posedge src_clk) begin
	if (src_reset)      src_toggle <= 1'b0;
	else if (src_pulse) src_toggle <= ~src_toggle;
end

// Destination: two synchroniser flops ([0],[1]) plus one edge-detect delay ([2]).
logic [2:0] dst_sync;
always_ff @(posedge dst_clk) begin
	if (dst_reset) dst_sync <= 3'b000;
	else           dst_sync <= {dst_sync[1:0], src_toggle};
end

assign dst_pulse = dst_sync[2] ^ dst_sync[1];

endmodule
