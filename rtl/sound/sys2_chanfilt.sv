// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

//============================================================================
//
// sys2_chanfilt -- one chip's analog channel strip, exactly as SP-275
// sheets 9A/9B build it:
//
//     chip -> [preamp: inverting LM324, RC low-pass in feedback]
//          -> [series .22uF]                       (interstage DC block)
//          -> [0x187a T-attenuator]                (programmable gain)
//          -> [output stage: inverting LM324, RC low-pass in feedback]
//          -> summing resistor
//
// This program is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation; either version 2 of the License, or (at your option)
// any later version.
//
//----------------------------------------------------------------------------
//
// Why per chip and not on the mix. The board does not low-pass the sum -- it
// low-passes each chip separately, with DIFFERENT corners, before the summing
// node (SP-275 sheets 9A/9B):
//
//     POKEY   7.234 kHz then 3.386 kHz     (R128 2.2K||C120 .01uF, R138 100K||C125 470pF)
//     YM2151  7.234 kHz then 7.234 kHz     (R106 100K||C93 220pF, R123 100K||C116 220pF)
//     TMS5220 4.019 kHz then 4.019 kHz     (R71 220K||C61 180pF, R119 220K||C114 180pF)
//
// A single pole on the sum can only equal the per-chip poles when those poles
// are equal, and here they are not: the POKEY is band-limited roughly two
// octaves harder than the YM. The previous single 12 kHz pole on the summed mix
// therefore made effects and speech audibly brighter than the PCB.
//
// Why the gain sits between the two poles. On the board the 0x187a attenuator
// is at the mixer node -- after the preamp, before the output stage. Order
// matters because the gain is time-varying: the game ducks music under speech by
// rewriting 0x187a. Applying the gain after both poles would let a gain step
// pass through as a click; here the second pole smooths it exactly as the real
// output stage does.
//
// The interstage DC block is what removes each chip's own bias -- notably the
// POKEY's -32 idle offset -- BEFORE summing, instead of leaning on one DC
// blocker downstream of the mix.
//
//============================================================================

module sys2_chanfilt #(
	parameter int    W     = 18,          // signed sample width
	parameter int    CW    = 18,          // pole coefficient fraction bits
	parameter [CW:0] A_PRE = 19'd6574,    // preamp feedback pole
	parameter [CW:0] A_HP  = 19'd7,       // interstage DC block (7.23 Hz: .22uF into 100K)
	parameter [CW:0] A_OUT = 19'd6574     // output-stage feedback pole
) (
	input  wire                 clk,
	input  wire                 ce,       // audio sample-rate enable (snd_pokey_en)
	input  wire                 rst,
	input  wire signed [W-1:0]  x,        // chip output, pre-scaled to mix units
	input  wire        [8:0]    gain,     // 0x187a Q0.8 gain for this chip (256 = unity)
	output wire signed [W-1:0]  y
);
	// Stage 1: preamp feedback low-pass.
	wire signed [W-1:0] pre;
	sys2_pole #(.W(W), .CW(CW)) u_pre (
		.clk(clk), .ce(ce), .rst(rst), .a(A_PRE), .x(x), .y(pre));

	// Interstage DC block: subtract the signal's own slow average (the series
	// coupling cap). One guard bit -- pre and its average can differ by more
	// than full scale during a step.
	wire signed [W-1:0] dcavg;
	sys2_pole #(.W(W), .CW(CW)) u_dc (
		.clk(clk), .ce(ce), .rst(rst), .a(A_HP), .x(pre), .y(dcavg));
	wire signed [W:0] hp = $signed({pre[W-1], pre}) - $signed({dcavg[W-1], dcavg});

	// 0x187a attenuator. gain <= 256, so the product cannot exceed hp's range
	// and the >>>8 lands back inside W+1 bits; W bits then hold it because the
	// summed inputs are scaled to leave headroom (see the top-level mixer).
	wire signed [W+9:0] scaled = hp * $signed({1'b0, gain});
	wire signed [W-1:0] atten  = W'(scaled >>> 8);

	// Stage 2: output-stage feedback low-pass.
	sys2_pole #(.W(W), .CW(CW)) u_out (
		.clk(clk), .ce(ce), .rst(rst), .a(A_OUT), .x(atten), .y(y));
endmodule
