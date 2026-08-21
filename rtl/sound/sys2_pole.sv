// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

//============================================================================
//
// sys2_pole -- single-pole IIR low-pass (the FPGA equivalent of one analog
// RC section). This is the JTFRAME `jtframe_pole` primitive re-implemented in
// clean SystemVerilog for this project (no jtframe dependency). It is the
// building block of the D4 analog output mixer (sys2_rcmix).
//
// This program is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation; either version 2 of the License, or (at your option)
// any later version.
//
//----------------------------------------------------------------------------
//
// Difference equation (exponential moving average, the discrete RC low-pass):
//
//     y[n] = y[n-1] + a*(x[n] - y[n-1])          0 < a < 1,  DC gain = 1
//
// with `a` supplied as an unsigned Q0.CW fraction. The -3 dB cutoff is
//
//     a  = 1 - exp(-2*pi*fc/fs)   (fs = the `ce` sample-enable rate)
//     fc = -fs*ln(1-a)/(2*pi)  ~=  fs*a/(2*pi)   for small a
//
// The state `yq` is kept at Q(W).CW (i.e. y scaled by 2^CW) so the CW extra
// fraction bits below the sample LSB are retained. That is what avoids the
// classic fixed-point IIR "dead zone": with a tiny coefficient, a[int]*(x-y)
// on a 1-LSB error is still a[int] (>= 1), so the filter always converges to
// DC instead of stalling short of it. Multiplying by 2^CW throughout:
//
//     yq[n] = yq[n-1] + a[int]*(x[n] - (yq[n-1] >>> CW))
//     y     = yq >>> CW
//
// No dependence on the absolute audio-clock rate: it advances one step per `ce`.
//============================================================================

module sys2_pole #(
	parameter int W  = 18,   // signed sample width (input and output)
	parameter int CW = 18    // coefficient / state fraction bits ( a is Q0.CW )
) (
	input  wire                 clk,
	input  wire                 ce,    // sample-rate clock enable (one cycle per new sample)
	input  wire                 rst,   // synchronous clear
	input  wire [CW:0]          a,     // Q0.CW pole coefficient (1 .. 2^CW); bigger a = higher fc
	input  wire signed [W-1:0]  x,     // input sample
	output wire signed [W-1:0]  y      // low-passed output (DC gain 1)
);
	// State width = integer part (W) + fraction (CW) + a few guard bits so the
	// a*err increment and the accumulator can never overflow for any a <= 2^CW.
	localparam int ACW = W + CW + 4;

	reg  signed [ACW-1:0] yq;
	wire signed [W-1:0]   yo   = yq[CW+W-1:CW];                   // y = yq >>> CW (integer part)
	wire signed [W:0]     err  = $signed({x [W-1], x })          // x - y, one guard bit
	                           - $signed({yo[W-1], yo});
	wire signed [ACW-1:0] step = $signed({1'b0, a}) * err;       // a[int] * err  (Q0.CW * int)

	always @(posedge clk) begin
		if (rst)     yq <= '0;
		else if (ce) yq <= yq + step;
	end

	assign y = yo;
endmodule
