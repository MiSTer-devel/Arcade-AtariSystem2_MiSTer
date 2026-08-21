// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Quadrature waveform generator: MiSTer spinner/dial -> the X/Y edges a real encoder
// disc would produce, for `sys2_leta` to count.
//
// Why this exists rather than writing the counter directly
//
// It would be shorter to add the spinner delta straight into the LETA's count register.
// That is what MAME does -- it reads the host analog port and never simulates the
// encoder at all -- and it is wrong for us in a specific way: it bypasses everything
// about the chip that a game can observe. The 156 kHz sample rate, the two-edge decode,
// the resolution bit, the direction logic and the count-loss at high speed would all be
// untested and unreachable. 720's service-mode calibration screen counts real edges.
//
// So the model stays a model: this block makes edges, `sys2_leta` counts them, and if the
// player spins faster than the chip can sample, counts are lost here exactly as on the
// board.
//
// ---------------------------------------------------------------------------
// Phase arithmetic
//
// The Gray sequence is 00 -> 01 -> 11 -> 10 -> 00 (X = bit0, Y = bit1) for "forward".
// X transitions twice per full cycle -- rising at phase 1->2 and falling at 3->0 -- so in
// the LETA's 2x mode one count is two phase steps. A requested movement of N counts is
// therefore 2N phase steps, which is why `phase_target` is the spinner accumulator
// shifted left by one.
//
// In 1x mode the chip counts only X's rising edge, so the same 2N phase steps yield
// N/2 counts -- half as many. That asymmetry is the chip's, not this generator's: the encoder
// disc does not change when the resolution bit does. Do NOT "compensate" for it here.
// ---------------------------------------------------------------------------
module sys2_quad_gen #(
	// Phase steps emitted per `ce_step` tick. 1 is the safe default: the caller drives
	// ce_step at the LETA's own 156.25 kHz phi2 rate, so every emitted edge is guaranteed
	// to be sampled. Raising this trades fidelity for catch-up speed and WILL lose counts.
	parameter int STEP_PER_TICK = 1
) (
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_step,     // emit at most STEP_PER_TICK phase steps

	// MiSTer spinner: [7:0] signed delta, [8] toggles on every update.
	input  logic [8:0]  spinner,

	// Optional absolute source (analog stick / digital steer), already converted to a
	// signed count delta by the caller. Added to the same accumulator as the spinner so
	// the two can be used interchangeably or together.
	input  logic signed [8:0] delta,
	input  logic              delta_valid,

	output logic        quad_x,
	output logic        quad_y,

	// Signed backlog still to be emitted, in phase steps. Nonzero means the player is
	// moving faster than the encoder can represent.
	output logic signed [15:0] backlog
);

logic        spin_tog_d;
logic signed [15:0] phase_target;   // in phase steps
logic signed [15:0] phase_now;

// Gray encode. The pairs below are written (X,Y) in the SAME order as sys2_leta's
// header and its bench: 00 -> 01 -> 11 -> 10 is forward, i.e. Y leads on the first
// step. Writing them (Y,X) instead swaps X and Y and inverts the counted direction --
// which is exactly the failure a round-trip check catches.
always_comb begin
	case (phase_now[1:0])
		2'd0: begin quad_x = 1'b0; quad_y = 1'b0; end
		2'd1: begin quad_x = 1'b0; quad_y = 1'b1; end
		2'd2: begin quad_x = 1'b1; quad_y = 1'b1; end
		default: begin quad_x = 1'b1; quad_y = 1'b0; end
	endcase
end

wire        spin_new   = (spinner[8] != spin_tog_d);
wire signed [8:0] spin_delta = {spinner[7], spinner[7:0]};   // sign-extend to 9 bits

// Named constants rather than inline casts. A negated inline cast on STEP_PER_TICK lints
// clean but is a syntax error in Quartus 17.0 ("near text: ' '; expecting )"), which no
// amount of linting catches -- only a real compile does. Quartus is the last gate for a
// reason; run it before believing a module is done.
//
// Do NOT let a comment line START with the word that names the linter: it reads the rest
// of the line as a lint pragma and stops with BADVLTPRAGMA. That aborted the whole lint of
// this tree, which is part of why the SDRAM controller's unconnected burst port went
// unnoticed until it showed up as a black screen on hardware.
localparam logic signed [15:0] STEP_POS =  16'sd0 + 16'(STEP_PER_TICK);
localparam logic signed [15:0] STEP_NEG =  16'sd0 - 16'(STEP_PER_TICK);

// One count = two phase steps (see the header), so every request is doubled here.
wire signed [15:0] req_phase = (spin_new    ? 16'(spin_delta) * 2 : 16'sd0)
                             + (delta_valid ? 16'(delta)      * 2 : 16'sd0);

wire signed [15:0] remaining = phase_target + req_phase - phase_now;

// Declared above, used here -- this assign used to sit ~35 lines UP, ahead of
// `remaining`. Verilator and Quartus both accepted the forward reference (it lints clean and
// synthesises), but ModelSim rejected it with TWO errors: vlog-2730 'Undefined variable' at
// the use, then vlog-2388 'already declared in this scope' at the real declaration -- an
// implicit 1-bit net created at the use site, which would have silently truncated this
// 16-bit value rather than failing loudly. Declare before use even where the tools let
// you get away with it.
assign backlog = remaining;

always_ff @(posedge clk) begin
	if (reset) begin
		spin_tog_d   <= spinner[8];
		phase_target <= '0;
		phase_now    <= '0;
	end else begin
		// Accumulate both request sources in ONE expression. Writing them as two separate
		// `if`s in the same always block would silently drop the spinner on any cycle where
		// both fired, because the later assignment wins.
		spin_tog_d   <= spinner[8];
		phase_target <= phase_target + req_phase;

		// Walk toward the target, never past it: with STEP_PER_TICK > 1 an unclamped step
		// oscillates around the target forever and emits edges that were never requested.
		if (ce_step) begin
			if (remaining > 0) phase_now <= phase_now +
				((remaining >= STEP_POS) ? STEP_POS : remaining);
			else if (remaining < 0) phase_now <= phase_now +
				((remaining <= STEP_NEG) ? STEP_NEG : remaining);
		end
	end
end

endmodule
