// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Analog stick -> encoder-count deltas, for the four LETA games' steering.
//
// Why this exists
// ---------------
// ssprint, csprint, apb and 720 steer through `sys2_leta`, which counts quadrature edges
// made by `sys2_quad_gen`. Originally the ONLY thing driving those generators was
// MiSTer's `spinner_*`, which is fed by a real spinner/dial/mouse. A gamepad thumbstick
// populates `joystick_l_analog_*` and leaves `spinner_*` static, so on a pad those four
// games had NO steering input at all -- reported from hardware, all four affected, and
// Paperboy unaffected because its handlebar is an ADC axis rather than an encoder.
//
// `sys2_quad_gen` already carries the injection port for this ("Optional absolute source
// (analog stick / digital steer), already converted to a signed count delta by the
// caller"); it was simply never fed by anything but 720's centre-disc divider. This module
// is that caller.
//
// ---------------------------------------------------------------------------
// What it models, and what it deliberately does not
//
// A real Sprint wheel and 720's rotate disc are continuous-rotation encoders: they report
// movement, not position, and holding the wheel at an angle produces nothing. A thumbstick
// reports position and springs back to centre, so the two cannot be mapped one-to-one.
// This converts deflection to rate -- push further, turn faster, hold to keep turning --
// which is the standard substitute and is what makes the games playable on a pad.
//
// It emits count deltas into the normal quadrature path. It does NOT write the LETA
// counter, so the 156 kHz sample rate, the 2x decode, the resolution bit and the count
// loss at high speed all still apply exactly as they do to a real encoder. That is the
// whole reason `sys2_quad_gen` exists (see its header) and this module must not shortcut
// it -- 720's service-mode calibration screen counts real edges.
//
// Divergence from MAME, deliberate and narrow. MAME gives 720 (and only 720) a
// `SELECT: Controller Type` choice between `FAKE_SPINNER: Dial` and `FAKE_JOY_X/Y: AD
// Stick X/Y`, and for the stick it converts the stick's ANGLE into disc movement --
// `m_joy_last_angle` in atarisy2.h, which MAME initialises to 90.0 as of commit 6effbd5.
// Angle mode is nicer for 720 specifically, because the skater then faces
// where the stick points, but it needs an atan2 this core has no reason to carry yet.
// Rate mode is used for all four games here; angle mode for 720 is a future refinement,
// and MAME's 90-degree initial value is the reference if it is ever built.
// MAME offers NO stick mapping at all for the three wheel games (they are pure `Dial`), so
// there is no reference to diverge from there.
//
// ---------------------------------------------------------------------------
// Rate arithmetic
//
// `axis` is MiSTer's signed analog axis: 0 is centre, +/-127 full scale. An absent stick
// reads 0, which is centre, so this block is INERT with no analog controller connected --
// it needs no enable, and adding one would be a constant on a port.
//
// Deflection past DEADZONE accumulates at `tick` rate; each time the accumulator crosses
// 2**ACC_BITS it emits one count and keeps the remainder, so sub-count rates are preserved
// rather than truncated (the same reason sys2_center_disc keeps its signed remainder --
// dropping it there lost 35 of every 36 counts of slow movement).
//
//   counts/sec = f_tick * (|axis| - deadzone) / 2**acc_bits
//
// At the default f_tick = 156.25 kHz (the LETA's own phi2), acc_bits = 15 and the
// caller's default deadzone of 12, full deflection (|axis| = 127) is ~548 counts/sec.
// For scale: 720's rotate disc is 72 teeth read at 2x = 144 counts per revolution, so
// full stick is a brisk ~3.8 rev/sec.
//
// ---------------------------------------------------------------------------
// Both terms are runtime inputs, not parameters (OSD steering options).
//
// `acc_bits` and `deadzone` were elaboration-time parameters. They are ports now so the OSD
// can offer the two things a pad player actually needs: how fast full deflection turns, and
// how much slop to ignore on a worn stick. The caller owns the menu-to-value table -- this
// module stays a rate converter and knows nothing about status bits. Precedent for the option
// itself: Arcade-Arkanoid's "Spinner Resolution", which is the same shift, chosen the same way.
//
// The digital arm benefits most, and that is not incidental. A D-pad pins |axis| at full
// scale, so `acc_bits` IS the turn rate there -- with one fixed value a pad player had 3.8
// rev/sec on 720 and no way to slow it down.
//
// `acc_bits` is CLAMPED to ACC_BITS_MAX. That is not dead code: the port is
// 5 bits, so 31 is expressible at this module's boundary even though the shipping caller drives
// a 4-entry table. Unclamped, an over-range shift makes the threshold 0 and every tick emits a
// count -- a runaway, not a graceful nothing. Contrast the -128 wrap guard removed above, which
// was unreachable at ITS port width and therefore genuinely dead.
// ---------------------------------------------------------------------------
module sys2_analog_spin #(
	// Sizes the accumulator and sets the slowest selectable rate. Raising it costs one flop
	// per step and widens the threshold compare; nothing else here depends on it.
	parameter int ACC_BITS_MAX = 17
) (
	input  logic              clk,
	input  logic              reset,
	input  logic              tick,        // rate reference; drive from leta_ce_phi2

	input  logic signed [7:0] axis,        // MiSTer signed analog axis, 0 = centre
	input  logic              invert,      // per-game steering polarity

	// Rate controls. Higher acc_bits = slower; deadzone is in axis units (0 disables it).
	input  logic        [4:0] acc_bits,
	input  logic        [8:0] deadzone,

	output logic signed [8:0] delta,       // counts, for sys2_quad_gen.delta
	output logic              delta_valid  // one-cycle pulse
);

// Magnitude past the deadzone. The axis is sign-extended to 9 bits FIRST, which is what
// makes -128 safe: it has no positive counterpart in 8 bits, but -(-128) = +128 is
// representable in 9, so the negate cannot wrap and full-left cannot silently become zero
// or full-right. Do not narrow this to 8 bits to save a flop.
// An earlier version carried an extra `mag[8] ? 0 :` guard against that wrap. It was
// UNREACHABLE at 8-bit input width -- proven by mutation: the mutant survived, which
// indicted the claim rather than the test. Removed rather than left as reassuring
// dead code. Simulation still pins -128 behaviourally, with a width-narrowing mutant that
// fails ONLY that case.
wire signed [8:0] ax9  = 9'(axis);
wire        [8:0] mag  = ax9[8] ? 9'(-ax9) : 9'(ax9);
wire        [8:0] over = (mag > deadzone) ? (mag - deadzone) : 9'd0;

// Direction, after the per-game polarity flip.
wire neg = invert ? (ax9 > 0) : (ax9 < 0);

// See the clamp note in the header: 5 bits can express more than the accumulator holds, and an
// over-range shift would zero the threshold and emit a count EVERY tick.
wire [4:0] acc_bits_c = (acc_bits > 5'(ACC_BITS_MAX)) ? 5'(ACC_BITS_MAX) : acc_bits;

localparam int ACC_W = ACC_BITS_MAX + 1;
wire [ACC_W-1:0] thresh = {{(ACC_W-1){1'b0}}, 1'b1} << acc_bits_c;

logic [ACC_W-1:0] acc;
wire  [ACC_W-1:0] acc_next = acc + {{(ACC_W-9){1'b0}}, over};

always_ff @(posedge clk) begin
	if (reset) begin
		acc         <= '0;
		delta       <= '0;
		delta_valid <= 1'b0;
	end else begin
		delta_valid <= 1'b0;
		delta       <= '0;
		if (tick) begin
			if (over == 0) begin
				// Recentre the accumulator when the stick returns to rest, so a long slow
				// push does not fire one stale count after the player lets go.
				acc <= '0;
			end else begin
				acc <= acc_next;
				if (acc_next >= thresh) begin
					acc         <= acc_next - thresh;
					delta       <= neg ? -9'sd1 : 9'sd1;
					delta_valid <= 1'b1;
				end
			end
		end
	end
end

endmodule
