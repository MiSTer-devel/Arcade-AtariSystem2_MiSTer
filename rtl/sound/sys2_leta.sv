// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Atari LETA (137304-2002) -- 4-channel quadrature counter, read by the sound 6502.
//
// Used by 720 (rotate disc + centre disc), both Sprints (steering wheels) and APB
// (steering wheel). Paperboy has no LETA; it steers through the ADC0809.
//
// ---------------------------------------------------------------------------
// EVIDENCE -- two independent sources, and they agree pin for pin
//
// 1. Schematic: SP-290 (Super Sprint schematic package), sheet 9B ("Steering Wheel
//    Inputs", PDF p18), chip at 7F:
//   * Eight quadrature inputs in X/Y pairs -- X1/Y1 .. X4/Y4 on pins 8..1, from connector
//     J103, each with a 3.3K pull-up (R67, 8PL) and a 0.01 uF cap (C72-C79, 8PL).
//   * `A0`/`A1` (pins 11/10) select which of the four counters is read; `D0-D7` (pins
//     15-22) return it; `CE` (pin 23) is the 0x1810-0x1813 decode.
//   * pin 9 `TEST` is tied to GND. pin 13 is phi2 = 156 kHz (20 MHz / 128), NOT the 6502
//     clock: it is the chip's only sampling clock.
//   * pin 14 is drawn "RESET" and driven by net `TBRES` = LS175 5F Q, whose D is DB4 of
//     the `0x187C` write and which P2RESET clears. Sheet 4B's memory map calls that bit
//     "LETA Resolution" -- TBRES is TrackBall RESolution, not a reset.
//
// 2. JROK, "LETA on a CPLD" (jrok.com/project/leta, 2007): a replacement built by bench-
//    testing a real 137304-2002, then run on Rampart, Off the Wall and Blasteroids boards.
//    Its pinout names pin 14 RESOLN, pin 13 CK, pin 9 TEST, and pins 1-8
//    DIR3 CLK3 DIR2 CLK2 DIR1 CLK1 DIR0 CLK0 -- so Atari X = CLK and Atari Y = DIR.
//    The behaviour below is its `LETA_REP.vhd`, transcribed.
//
// MAME does not model the counter at all (it adds the host analog delta straight into the
// count), so it is no reference for anything in this module except the 144-count figure.
//
// ---------------------------------------------------------------------------
// BEHAVIOUR (per channel, on every phi2 sample)
//
//   * Input filter: an input is accepted only after 3 consecutive identical samples; a
//     1- or 2-sample glitch is ignored. A state must therefore last >= 3 phi2 periods
//     (19.2 us) to be seen at all -- sys2_quad_gen holds each state long enough.
//   * DIR (= Y) edge, both edges, ALWAYS counted:   up iff new Y == X.
//   * CLK (= X) edge, both edges, only if RESOLN=1: up iff new X != Y.
//   * Both inputs changing between two stable states: no count (the chip cannot tell).
//   * So RESOLN = 0 is 2x (two counts per quadrature cycle) and RESOLN = 1 is 4x.
//     There is NO 1x mode.
//   * Counters are 8-bit and wrap.
//
// Every game writes bit 4 = 0 (static scan of all five sound ROMs for `STA $187C`, plus a
// MAME write tap through boot and attract). So the boards run at 2x, and 720's 72-tooth
// rotate disc gives 144 counts per revolution -- MAME's figure, now reached through the
// hardware rather than assumed. 4x is modelled because it is the chip's, but nothing is
// known to select it.
//
// Sound-bus placement (`0x1810-0x1813`, read-only, mirrored 0x278c) is confirmed by both
// the schematic's Sheet 4B memory map and MAME's `atarisy2.cpp` sound map. It is NOT at
// 0x1850 -- that is the YM2151.
module sys2_leta (
	input  logic       clk,        // clk_sys
	input  logic       ce_phi2,    // 156.25 kHz enable (schematic pin 13)
	input  logic       reset,      // model reset; the chip itself has none (TEST is grounded)

	// Quadrature inputs, already synchronised to clk by the caller.
	input  logic [3:0] quad_x,     // X1..X4 = the chip's CLK inputs
	input  logic [3:0] quad_y,     // Y1..Y4 = the chip's DIR inputs

	// RESOLN, pin 14 = 0x187C bit 4 (net TBRES). 0 = 2x (DIR edges), 1 = 4x (DIR + CLK).
	input  logic       resolution,

	// Read port: A1:A0 selects the counter, data is combinational (the 6502 read
	// strobe is the chip's CE).
	input  logic [1:0] sel,
	output logic [7:0] dout
);

logic [7:0] count [0:3];
logic [2:0] hx [0:3];      // last three samples of X (CLK), newest in bit 0
logic [2:0] hy [0:3];      // last three samples of Y (DIR)
logic [3:0] lx, ly;        // last STABLE level of each input

assign dout = count[sel];

integer i;
always_ff @(posedge clk) begin
	if (reset) begin
		for (i = 0; i < 4; i = i + 1) begin
			count[i] <= 8'd0;
			// Prime the filter from the CURRENT inputs, not from zero. All eight inputs are
			// pulled up through R67, so an UNCONNECTED channel reads 1 1; zeroed history
			// would see that as a fresh 0->1 and fabricate counts out of nothing. Paperboy
			// has no LETA and the Sprints leave channels empty, so that phantom would be live.
			hx[i] <= {3{quad_x[i]}};
			hy[i] <= {3{quad_y[i]}};
		end
		lx <= quad_x;
		ly <= quad_y;
	end else if (ce_phi2) begin
		for (i = 0; i < 4; i = i + 1) begin
			hx[i] <= {hx[i][1:0], quad_x[i]};
			hy[i] <= {hy[i][1:0], quad_y[i]};

			// Everything below reads the history as it stood BEFORE this sample, exactly as
			// the VHDL's two processes do on a shared clock edge.
			if (hx[i] == 3'b000) lx[i] <= 1'b0;
			if (hx[i] == 3'b111) lx[i] <= 1'b1;
			if (hy[i] == 3'b000) ly[i] <= 1'b0;
			if (hy[i] == 3'b111) ly[i] <= 1'b1;

			if ((hx[i] == 3'b000 || hx[i] == 3'b111) && (hy[i] == 3'b000 || hy[i] == 3'b111)) begin
				// DIR edge: counted at either resolution.
				if (hx[i][0] == lx[i] && hy[i][0] != ly[i])
					count[i] <= (hy[i][0] == hx[i][0]) ? count[i] + 8'd1 : count[i] - 8'd1;
				// CLK edge: counted only at RESOLN = 1.
				else if (resolution && hx[i][0] != lx[i] && hy[i][0] == ly[i])
					count[i] <= (hx[i][0] != hy[i][0]) ? count[i] + 8'd1 : count[i] - 8'd1;
			end
		end
	end
end

endmodule
