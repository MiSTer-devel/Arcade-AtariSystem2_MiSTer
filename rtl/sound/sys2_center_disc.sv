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
// MAME is no help here and it is worth saying why: it exposes LETA0 and LETA1 as two
// INDEPENDENT host dials ("Center" and "Rotate", both `// not direct mapped`) and never
// derives one from the other. So there is no reference implementation to copy; the ratio
// below comes from the disc geometry in MAME's own comment and its ASCII figure:
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
// OPEN -- and it is the DISTRIBUTION, not the phase. Read this before "fixing" either.
//
// This block used to say the open question was the absolute PHASE (an offset); a closer
// reading reduced it to "verify our position-0 alignment -- a code read, not research".
// The code read was done and the premise was wrong.
//
// There is no position-0 alignment to verify, because this module has no position. It is a
// pure rate divider: a signed remainder accumulator that emits one centre count per 36
// rotate counts, wherever the shaft happens to be. An offset cannot be checked against a
// thing that is not represented.
//
// And the comparison that matters is not the rate. MAME's Fake Spinner mode -- the one
// mode that derives centre from rotate, exactly as this module does -- keeps `m_spin_pos`
// over 0..143 and bumps the centre count at four positions only:
//
//     switch (m_spin_pos) { case 2: case 3: case 141: case 142: m_spin_center_count++; }
//
// (verified against MAME master upstream.) Those are two adjacent pairs straddling position 0.
//
// So both models give 4 centre counts per revolution -- which is why the 1:36 RATIO agrees
// and why a check on the ratio alone passes. But the emission pattern differs
// completely:
//
//     MAME    all four counts within +/-3 of the top; ~138 counts of silence elsewhere
//     ours    one count every 36 -- four evenly spaced angles, none of them privileged
//
// MAME's clustering is not an arbitrary choice: it is what the geometry figure QUOTED
// above in this very header shows. The two notches sit either side of the top ("2 1 | 1 2"),
// so every edge of a 2-tooth disc read at 2x lands near the top. This header transcribed
// that figure correctly and then implemented a uniform divider anyway.
//
// Why it may matter: the centre channel is how 720 learns where "top" is. TM-294's Control
// Test says the zeroing function "gives the control a point of reference for determining
// direction... Rotate the control clockwise slowly until the ZEROED message is displayed...
// This should occur when the control is at the top, or closest to the video display screen."
// A uniform divider offers four candidate angles per revolution instead of one.
//
// Be fair about the authority. MAME's own comment calls this "the easiest way to
// accurately FAKE the center count", so it is a model, not silicon -- but it is a model
// that agrees with the disc figure, and we have neither. 720 is hardware-confirmed
// PLAYABLE, so whatever the firmware does with this, it is not fatal.
//
// The discriminating test is cheap and externally anchored, and it is the manual's own:
// run 720's Control Test on the board, rotate slowly through a full revolution and count
// how many angles produce ZEROED. One (at the top) = this module is fine as built. Four =
// the distribution is wrong and this becomes a position model rather than a divider.
// Until then the check deliberately asserts the RATIO only -- locking today's emission
// pattern into a hard test would turn the correction into a spurious failure.
// ---------------------------------------------------------------------------
module sys2_center_disc #(
	// Rotate counts per revolution (72 teeth read at 2x).
	parameter int ROTATE_COUNTS = 144,
	// Centre counts per revolution (2 teeth read at 2x).
	parameter int CENTER_COUNTS = 4
) (
	input  logic       clk,
	input  logic       reset,

	// Rotate-disc movement, in LETA counts, as it is handed to channel 1's generator.
	input  logic signed [8:0] rot_delta,
	input  logic              rot_valid,

	// Centre-disc movement, in LETA counts, for channel 0's generator.
	output logic signed [8:0] ctr_delta,
	output logic              ctr_valid
);

// One centre count per this many rotate counts. 144/4 = 36.
localparam int RATIO = ROTATE_COUNTS / CENTER_COUNTS;

// The remainder accumulator is the whole point. Dropping the remainder loses 35 of every
// 36 counts of slow movement, so a player easing the spinner round would never advance the
// centre disc at all and 720's calibration would never converge. Signed, so backwards
// motion returns the borrow.
//
// Subtract, do not divide. The obvious `acc / RATIO` with RATIO = 36 (not a power of
// two) synthesises to a full 16-bit signed divider, and it landed straight on the critical
// path: it was the ENTIRE core-clock setup violation, -0.795 ns with TNS -2.695 against a
// previous best of +0.372. Quartus named `resid[9] -> resid[14]` as the failing path. Simulation
// and lint see nothing wrong with a divider; only a real compile does.
//
// One compare-and-subtract per clock is all that is needed. The chip samples at 156 kHz
// and this runs at 32 MHz, so a backlog of even the full +/-127 spinner delta drains in a
// handful of clocks -- thousands of times faster than the next sample.
localparam logic signed [15:0] RATIO_S = 16'sd0 + 16'(RATIO);

logic signed [15:0] resid;
logic signed [15:0] acc;

always_comb acc = resid + (rot_valid ? 16'(rot_delta) : 16'sd0);

always_ff @(posedge clk) begin
	ctr_valid <= 1'b0;
	ctr_delta <= '0;

	if (reset) begin
		resid <= '0;
	end else if (acc >= RATIO_S) begin
		resid     <= acc - RATIO_S;
		ctr_delta <= 9'sd1;
		ctr_valid <= 1'b1;
	end else if (acc <= -RATIO_S) begin
		resid     <= acc + RATIO_S;
		ctr_delta <= -9'sd1;
		ctr_valid <= 1'b1;
	end else begin
		resid <= acc;
	end
end

endmodule
