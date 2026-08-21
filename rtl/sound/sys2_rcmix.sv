// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

//============================================================================
//
// sys2_rcmix -- analog output-network mixer (JTFRAME `jtframe_rcmix`
// analog, re-implemented here with no jtframe dependency). It applies the RC
// low-pass + AC-coupling ("DC removal") character of the Paperboy PCB's audio
// output stage to the already-summed, already-gain-scaled stereo mix, then
// saturates to 16 bits.
//
// This program is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation; either version 2 of the License, or (at your option)
// any later version.
//
//----------------------------------------------------------------------------
//
// What this models (SP-275 sheets 9A "Speech/Music", 9B "Audio Output Drivers"):
//   * Stage-1 per-chip programmable GAIN (the 0x187a resistor network) is
//     already handled upstream -- sys2_sound_bus decodes 0x187a into the
//     mix_gain_ym/pk/tms Q0.8 gains, and the per-chip strips (sys2_chanfilt,
//     inside sys2_audio_mix) apply them between their two poles. This module
//     is the piece those stages do NOT do.
//   * Each summing op-amp (LM324) has a feedback capacitor -> a single-pole
//     low-pass that rolls off the DAC/chip stair-stepping and HF hiss.
//   * The line out is AC-coupled (series cap) -> a very-low-frequency high-pass
//     that removes DC/bias (e.g. the POKEY's ~-32 idle offset) faithfully in
//     the core instead of leaning on the MiSTer framework's own DC filter.
//
// Fidelity note: the board's per-chip filtering ahead of the summing node is
// modelled upstream (sys2_chanfilt carries the sheet 9A/9B component values,
// one strip per chip); this module is only the shared OUTPUT stage after the
// sum -- one low-pass + one DC blocker per channel. Coefficients are
// schematic-plausible defaults (see below), overridable via parameters, and
// the whole stage is bypassable at runtime (bit-identical to the raw mix), so
// it is polish that can be A/B'd on hardware -- never a functional dependency.
//
// COEFFICIENTS (defaults, at fs = ce rate = snd_pokey_en = 1.789772 MHz):
//   A_LP: fc ~= 12.0 kHz  (gentle output low-pass; a = 1-exp(-2*pi*fc/fs))
//   A_DC: fc ~=  9.8 Hz   (AC-coupling DC blocker)
//   a[int] = round(a * 2^CW), CW = 18  ->  A_LP = 10815, A_DC = 9.
//
//============================================================================

module sys2_rcmix #(
	parameter int    W    = 18,           // signed mix width in
	parameter int    CW   = 18,           // pole coefficient fraction bits
	parameter [CW:0] A_LP = 19'd10815,    // low-pass pole  (~12.0 kHz @ 1.789772 MHz)
	parameter [CW:0] A_DC = 19'd9         // DC-block pole  (~9.8  Hz  @ 1.789772 MHz)
) (
	input  wire                 clk,
	input  wire                 ce,       // audio sample-rate enable (snd_pokey_en)
	input  wire                 rst,
	input  wire                 bypass,   // 1 = pass the raw mix straight through (no filtering)
	input  wire signed [W-1:0]  mix_l,
	input  wire signed [W-1:0]  mix_r,
	output wire signed [15:0]   audio_l,
	output wire signed [15:0]   audio_r
);
	// Per channel: output-driver low-pass, then AC-coupling DC removal.
	wire signed [W-1:0] lp_l, lp_r;       // low-passed mix
	wire signed [W-1:0] dc_l, dc_r;       // slow (DC/bias) average of the low-passed mix

	sys2_pole #(.W(W), .CW(CW)) u_lp_l (.clk(clk), .ce(ce), .rst(rst), .a(A_LP), .x(mix_l), .y(lp_l));
	sys2_pole #(.W(W), .CW(CW)) u_lp_r (.clk(clk), .ce(ce), .rst(rst), .a(A_LP), .x(mix_r), .y(lp_r));
	sys2_pole #(.W(W), .CW(CW)) u_dc_l (.clk(clk), .ce(ce), .rst(rst), .a(A_DC), .x(lp_l),  .y(dc_l));
	sys2_pole #(.W(W), .CW(CW)) u_dc_r (.clk(clk), .ce(ce), .rst(rst), .a(A_DC), .x(lp_r),  .y(dc_r));

	// High-pass (DC removal) = the low-passed signal minus its own slow average.
	// One guard bit: lp and dc can be opposite full-scale in the worst case.
	wire signed [W:0] hp_l = $signed({lp_l[W-1], lp_l}) - $signed({dc_l[W-1], dc_l});
	wire signed [W:0] hp_r = $signed({lp_r[W-1], lp_r}) - $signed({dc_r[W-1], dc_r});

	// bypass -> the sign-extended raw mix, so the low 16 bits and the clamp are
	// bit-identical to a plain top-level saturate of the raw mix.
	wire signed [W:0] sel_l = bypass ? $signed({mix_l[W-1], mix_l}) : hp_l;
	wire signed [W:0] sel_r = bypass ? $signed({mix_r[W-1], mix_r}) : hp_r;

	assign audio_l = (sel_l >  19'sd32767) ? 16'sd32767 :
	                 (sel_l < -19'sd32768) ? 16'sh8000  : sel_l[15:0];
	assign audio_r = (sel_r >  19'sd32767) ? 16'sd32767 :
	                 (sel_r < -19'sd32768) ? 16'sh8000  : sel_r[15:0];
endmodule
