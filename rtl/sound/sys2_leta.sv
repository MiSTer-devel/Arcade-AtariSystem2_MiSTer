// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Atari LETA -- 4-channel quadrature counter, read by the sound 6502.
//
// Used by 720 (rotate disc + centre disc), both Sprints (steering wheels) and APB
// (steering wheel). Paperboy has no LETA; it steers through the ADC0809.
//
// ---------------------------------------------------------------------------
// EVIDENCE
//
// Schematic: SP-290 (Super Sprint schematic package), sheet 9B ("Steering Wheel
// Inputs"), chip at 7F. Confirmed there, and NOT guessed:
//
//   * Eight quadrature inputs in X/Y pairs -- X1/Y1 .. X4/Y4 on pins 8..1, from connector
//     J103, each with a 5.3K pull-up (R67, 8PL) and a 0.01 uF cap (C72-C79, 8PL). That RC
//     is ~53 us and is the chip's input debounce; it is modelled here as a sampling
//     filter rather than an analog pole.
//   * `A0`/`A1` (pins 11/10) select which of the four counters is read; `D0-D7` (pins
//     15-22) return it; `CE` (pin 23) is the 0x1810-0x1813 decode; `RESET` (pin 14) is
//     TBRES from the LS175 at 5F.
//   * phi2 (pin 13) is 156 kHz, NOT the 6502 clock. That is the rate at which the
//     chip samples its inputs, and therefore the maximum edge rate it can count. 156.25 kHz
//     is the 20 MHz master divided by 128.
//
// Sound-bus placement (`0x1810-0x1813`, read-only, mirrored 0x278c) is confirmed by both
// the schematic's Sheet 4B memory map and MAME's `atarisy2.cpp` sound map. It is NOT at
// 0x1850 -- that is the YM2151.
//
// ---------------------------------------------------------------------------
// Decoding rate -- why 2x and not 4x
//
// 720's rotate disc has 72 teeth and yields 144 counts per revolution (MAME's own
// comment, and the service-mode calibration depends on it). 72 teeth x 2 = 144, so the
// chip counts two edges per quadrature cycle, not four. This module therefore counts
// on every transition of X and uses Y only for direction, which is exactly 2x.
//
// `VERIFY` -- `0x187C` bit 4 is documented as "LETA Resolution" on Sheet 4B, but the
// sheet does not show what it changes, and MAME does not model it at all (it reads the
// host analog port directly and never simulates the counter). The most plausible reading
// is 2x vs 1x decoding, which is what `resolution` selects here. Confirm against hardware
// or a service-mode count before trusting 1x mode; 2x is the mode 720 needs and the one
// the 144-count figure pins down.
module sys2_leta (
	input  logic       clk,        // clk_sys
	input  logic       ce_phi2,    // 156.25 kHz enable (schematic pin 13)
	input  logic       reset,      // TBRES

	// Quadrature inputs, already synchronised to clk by the caller.
	input  logic [3:0] quad_x,     // X1..X4
	input  logic [3:0] quad_y,     // Y1..Y4

	// 0x187C bit 4. 1 = 2x (two counts per quadrature cycle, the 720 mode);
	// 0 = 1x. See the VERIFY note above.
	input  logic       resolution,

	// Read port: A1:A0 selects the counter, data is combinational (the 6502 read
	// strobe is the chip's CE).
	input  logic [1:0] sel,
	output logic [7:0] dout
);

logic [7:0] count [0:3];
// Only X needs a history: direction is decided from the CURRENT X and Y at the
// sampling edge, so a delayed Y would be dead weight.
logic [3:0] x_d;

assign dout = count[sel];

// One channel's decode. Counting on every X transition is 2x; counting only on the
// rising transition is 1x. Direction is the classic quadrature test: which of X/Y led.
function automatic logic tick(input logic x_now, input logic x_prev, input logic res);
	tick = res ? (x_now != x_prev)          // 2x: both edges of X
	           : (x_now && !x_prev);        // 1x: rising edge only
endfunction

// Direction from the Gray sequence 00->01->11->10->00 (forward):
//   X rises while Y=1  -> forward        X rises while Y=0  -> reverse
//   X falls while Y=0  -> forward        X falls while Y=1  -> reverse
// which collapses to "X and Y agree at the sampling edge".
function automatic logic up(input logic x_now, input logic y_now);
	up = (x_now == y_now);
endfunction

integer i;
always_ff @(posedge clk) begin
	if (reset) begin
		for (i = 0; i < 4; i = i + 1) count[i] <= 8'd0;
		// Prime the X history from the CURRENT inputs, not from zero. Zeroing it makes
		// every idle-HIGH channel look like a rising edge on the first phi2 tick after
		// reset, so an UNCONNECTED channel -- all eight inputs are pulled up through R67,
		// so unconnected reads 1 -- fabricates a count out of nothing. Paperboy has no
		// LETA at all and the Sprints leave one (ssprint) or two (csprint) wheel
		// channels empty, so that phantom would
		// have been live on real games. Found in simulation, which saw
		// its three idle channels sitting at 1 instead of 0.
		x_d <= quad_x;
	end else if (ce_phi2) begin
		// Sample at phi2 -- this is the whole input filter the chip has, and it is why
		// an encoder spun faster than 156 kHz of edges simply loses counts on real
		// hardware rather than reading wrong.
		x_d <= quad_x;
		for (i = 0; i < 4; i = i + 1) begin
			if (tick(quad_x[i], x_d[i], resolution))
				count[i] <= up(quad_x[i], quad_y[i]) ? count[i] + 8'd1 : count[i] - 8'd1;
		end
	end
end

endmodule
