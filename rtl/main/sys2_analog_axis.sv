// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// One ADC axis fed by either an analog stick or a D-pad.
//
// Why this is a module and not two inline ternaries in the top level -- the same reason
// sys2_analog_map is one: an axis that disagrees with itself fails QUIETLY. Nothing crashes,
// nothing lints, the game just drives wrong, and only a person playing it finds out.
//
// It already happened twice, both found by playing the games:
//
//   * Paperboy's D-pad up/down were INVERTED against the stick. The two axes were written as
//     separate inline expressions in atarisys2.sv, X assigned 0xff to `m_right` and Y assigned
//     0xff to `m_up` -- but the ANALOG flip makes 0xff the POSITIVE end of the axis, and on a
//     pad positive X is right while positive Y is DOWN. So X agreed with its stick and Y did
//     not. Left/right played fine; up was brake and down was gas.
//   * On the Sprints the same stick's Y was also wired to the gas pedal, so pressing DOWN
//     accelerated. That one is fixed by deleting the mapping outright -- see sys2_analog_map's
//     pedal(); the wheel owns that stick, so its Y cannot be the throttle at any polarity.
//
// The invariant this module exists to enforce: the digital extreme and the analog extreme in
// the SAME physical direction must produce the SAME value. Expressed as one implementation used
// by every axis, so the only per-axis decision left is naming which D-pad bit is the axis's
// POSITIVE end -- `dig_pos`. Two axes can no longer drift apart, because there is only one
// expression.
//
// What this module cannot check, stated so nobody reads more into it than is there:
// whether the CALLER named the right bit as `dig_pos`. That is a fact about the pad's sign
// convention, established by playing the game, and it is recorded at the instantiation in
// atarisys2.sv rather than derived here.
//
// Encoding: the ADC0809 idles at 0x80, and `axis ^ 8'h80` converts the stick's signed byte to
// that offset-binary scale -- centre 0x00 -> 0x80, full positive 0x7f -> 0xff, full negative
// 0x80 -> 0x00. The D-pad has no proportional information, so it slams to the same endpoints.
module sys2_analog_axis (
	// High when the pad is presenting a live analog stick, so the D-pad arms are ignored.
	// Derived once by the caller from the WHOLE stick word, not per axis: a player holding a
	// pure-X deflection still has a live stick, and its Y really is centred.
	input  logic       stick_active,

	// The stick's signed byte for this axis, 0 at rest.
	input  logic [7:0] axis,

	// The D-pad bits, named by which end of this axis they represent -- not by screen
	// direction. dig_pos is the end the analog axis reaches at 0x7f.
	input  logic       dig_pos,
	input  logic       dig_neg,

	output logic [7:0] value
);

localparam logic [7:0] AXIS_IDLE = 8'h80;   // the ADC0809's own idle reading
localparam logic [7:0] AXIS_POS  = 8'hff;
localparam logic [7:0] AXIS_NEG  = 8'h00;

// dig_pos is tested first, so both held reads as positive. Which one wins does not matter
// physically (a D-pad cannot press both) but leaving it undefined would be a second quiet
// difference between axes.
assign value = stick_active ? (axis ^ 8'h80)
             : dig_pos      ? AXIS_POS
             : dig_neg      ? AXIS_NEG
             :                AXIS_IDLE;

endmodule
