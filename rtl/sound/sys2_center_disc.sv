// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// 720's CENTRE disc: derives LETA channel 0 from the rotate disc on channel 1.
//
// ---------------------------------------------------------------------------
// Why this exists
//
// 720's controller is one shaft carrying TWO optical discs:
//
//   * the ROTATE disc, 72 teeth, read at 2x -> 144 counts per revolution, on LETA1;
//   * the CENTRE disc, 2 teeth, on LETA0, which tells the game where "top" is.
//
// A MiSTer player has one spinner, not a physical shaft, so the centre channel has to be
// synthesized from the same motion that drives rotate -- they are mechanically the same
// shaft and cannot drift apart.
//
// MAME's real-controller mode exposes LETA0 and LETA1 as two INDEPENDENT host dials
// ("Center" and "Rotate", both `// not direct mapped`). Only its Fake Spinner mode derives
// one from the other, the way this module must. The geometry comes from MAME's comment and
// its ASCII figure:
//
//     _____2  1________1  2_____
//          |__|        |__|          Center disc - 2 teeth
//        __    __    __    __
//     __|  |__|  |__|  |__|  |__     Rotate disc - 72 teeth (144 positions)
//       4  3  2  1  1  2  3  4
//
// Two teeth against seventy-two, both read at 2x, gives **4 centre counts per revolution
// against 144 rotate counts** -- a ratio of exactly 1:36. That ratio is the one solid fact
// here and is what this module implements.
//
// ---------------------------------------------------------------------------
// A position model -- the uniform divider is gone.
//
// This module used to be a rate divider: one centre count per 36 rotate counts,
// wherever the shaft happened to be, so the four counts of a revolution sat at four evenly
// spaced angles with none of them privileged. Every reference puts them at ONE place:
//
//   * MAME's Fake Spinner mode (atarisy2.cpp leta_r, `case 2`) keeps `m_spin_pos` over
//     0..143 and moves the centre count only at positions 2, 3, 141 and 142;
//   * the disc figure above shows both notches either side of the top;
//   * a photo of a new-old-stock centre disc (043807-02 Rev B) shows
//     its cut-out at a single angle;
//   * TM-294's Control Test: ZEROED "should occur when the control is at the top".
//
// What forced the change rather than leaving it open was 720's stick ANGLE mode
// (sys2_stick_angle): once the stick sets an absolute position, the game's idea of "top"
// has to land where the stick points up. With four candidate tops the skater could face
// 90 degrees off the stick.
//
// The model: `pos` is the shaft position, 0..143, counting up clockwise; count p is centred
// on p * 2.5 degrees, so 0 = the top. Every rotate count moves it. The centre count is a
// function of the UNWRAPPED position:
//
//     centre = 4 * revolutions + #{ e in {2, 3, 142, 143} : e <= pos }
//
// i.e. four edges per turn, entering and leaving two one-count notches at positions 142
// and 2 -- each 2.5 degrees wide, centred at -5 and +5 degrees, 10 degrees apart and
// symmetric about the top. That is MAME's own description ("two gaps 10 degrees apart,
// each 2.5 degrees wide") and its Fake Joystick window (+/-5 degrees). Crossing an edge
// upward counts +1 and back down -1.
//
// MAME's Fake Spinner uses {2, 3, 141, 142} on the way up -- the same two notches one
// count further round, because its position has no absolute phase and its pair sits
// symmetric about the 143|0 BOUNDARY. Ours is symmetric about count 0, which is where
// sys2_stick_angle puts "stick straight up". It also deliberately fixes MAME for the other
// direction: MAME decrements when the position ARRIVES at 2/3/141/142 going down, a
// different set of boundaries, so easing back and forth across a notch walks its centre
// count away for good. Here the count is a pure function of position, so any path back to
// the same place returns the same count.
//
// Still a model, not silicon: the sensor pitch, and so whether a pass reads as two
// separate blips or one signed pair, is unmeasured. Four counts per turn, all at the top,
// is the part every source agrees on, and simulation checks both the ratio and the place.
//
// Power-up phase: `pos` resets to 0. With a real spinner that is as arbitrary as MAME's
// own `m_spin_pos` start, and the game finds the top by the notches anyway. With the stick
// in angle mode it is exact: stick up = position 0.
// ---------------------------------------------------------------------------
module sys2_center_disc (
	input  logic       clk,
	input  logic       reset,

	// Rotate-disc movement, in LETA counts, as it is handed to channel 1's generator. Any
	// size up to +/-255: a spinner update and a stick move can arrive in the same cycle, so
	// the caller SUMS them and this module folds up to two whole turns.
	input  logic signed [8:0] rot_delta,
	input  logic              rot_valid,

	// Centre-disc movement, in LETA counts, for channel 0's generator.
	output logic signed [8:0] ctr_delta,
	output logic              ctr_valid,

	// Shaft position, 0..143, 0 = top. sys2_stick_angle steers against it.
	output logic [7:0]        pos
);

// Notch edges: count g(p) of edges at or below position p.
function automatic logic [2:0] edges_below(input logic [7:0] p);
	edges_below = {2'b00, (p >= 8'd2)}   + {2'b00, (p >= 8'd3)}
	            + {2'b00, (p >= 8'd142)} + {2'b00, (p >= 8'd143)};
endfunction

// pos + delta lies in [-255, 398]; fold it back into 0..143 and count the whole turns.
// Compares and adds only -- see the warning below on dividing by a non-power of two.
logic signed [10:0] sum;
logic signed [10:0] fold;
logic signed [3:0]  turns;     // -2..+2
logic signed [5:0]  cdelta;

always_comb begin
	sum = $signed({3'b000, pos}) + 11'(rot_delta);
	if (sum >= 11'sd288)      begin fold = sum - 11'sd288; turns =  4'sd2; end
	else if (sum >= 11'sd144) begin fold = sum - 11'sd144; turns =  4'sd1; end
	else if (sum >= 11'sd0)   begin fold = sum;            turns =  4'sd0; end
	else if (sum >= -11'sd144) begin fold = sum + 11'sd144; turns = -4'sd1; end
	else                      begin fold = sum + 11'sd288; turns = -4'sd2; end

	cdelta = $signed({turns, 2'b00})
	       + $signed({3'b000, edges_below(fold[7:0])})
	       - $signed({3'b000, edges_below(pos)});
end

// Never divide or take a modulo by 144 (or 36) HERE. The old divider's `acc / RATIO`
// with RATIO = 36 synthesised to a full signed divider and was the ENTIRE core-clock setup
// violation, -0.795 ns, until it became a compare-and-subtract. The fold above is the same
// idea: a handful of comparators, no divider.
always_ff @(posedge clk) begin
	ctr_valid <= 1'b0;
	ctr_delta <= '0;

	if (reset) begin
		pos <= 8'd0;
	end else if (rot_valid) begin
		pos <= fold[7:0];
		if (cdelta != 6'sd0) begin
			ctr_delta <= 9'(cdelta);
			ctr_valid <= 1'b1;
		end
	end
end

endmodule
