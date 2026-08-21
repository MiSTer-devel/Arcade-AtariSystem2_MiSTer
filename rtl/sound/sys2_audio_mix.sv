// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Paperboy analog audio chain, SP-275 sheets 9A/9B, as ONE module.
//
// Why this module exists. This chain used to be written inline in
// atarisys2.sv, and the same constants were then re-typed in THREE other places:
// the top level and every verification bench. That is not a tidiness
// complaint -- it is the direct cause of a 9.2 dB speech regression that reached hardware. The
// "board-exact" speech gain was applied in the top level while the model it was validated against
// kept the old value, and the audio gate's balance metric multiplied by only part of the chain, so
// nothing anywhere disagreed loudly enough to notice. Extracting it means the top level, the
// verification bench and the golden gate all instantiate the SAME logic, and a constant can no
// longer drift between them.
//
// Signal chain, in board order:
//   chip output -> digital full-scale scaling -> preamp pole -> interstage DC block ->
//   0x187a programmable attenuator -> output-stage pole -> summing weights -> output network
//
// The 0x187a gains arrive already decoded (sys2_sound_bus owns that register); everything from
// the chip pins to the connector lives here.
module sys2_audio_mix (
	input  logic        clk,
	input  logic        ce,             // snd_pokey_en (1.789772 MHz) -- the filter sample rate
	input  logic        rst,

	// Raw chip outputs, exactly as the chips present them.
	input  logic signed [17:0] ym_l18,      // jt51 left, already sign-extended to 18 bits
	input  logic signed [17:0] ym_r18,
	input  logic signed [5:0]  pokey1_snd,  // POKEY 1 (left path); idles at -32
	input  logic signed [5:0]  pokey2_snd,  // POKEY 2 (right path)
	input  logic signed [13:0] tms_snd,     // TMS5220 speech (mono)

	// Decoded 0x187a attenuator gains, Q0.8 (256 = unity). From sys2_sound_bus.
	input  logic [8:0]  gain_ym,
	input  logic [8:0]  gain_pk,
	input  logic [8:0]  gain_tms,

	output logic signed [15:0] audio_l,
	output logic signed [15:0] audio_r
);

// ---------------------------------------------------------------------------
// Chip input scaling.
//
// YM `>>> 1` and POKEY `<<< 9` are OUR digital full-scale choices -- the schematic cannot set them,
// it never sees our sample widths -- and they are unchanged, so the YM:POKEY balance that was tuned
// on hardware is preserved.
//
// Speech level: two factors, deliberately kept separate because only one is a board value.
//   * TMS_PREAMP_Q8 = 1239 = x4.84 -- the REAL speech preamp, R71/R70 * R119/R120 = 2.2 * 2.2,
//     SP-275 sheet 9A. Board fact.
//   * TMS_FS_Q8     = 740  = x2.892 -- a digital full-SCALE calibration, the same kind of
//     hand-chosen constant as YM's `>>> 1` and POKEY's `<<< 9`. NOT a board value.
// The second exists because x4.84 is a VOLTAGE gain on the real chip's output SWING, and our TMS
// core emits a raw 14-bit number that is not calibrated to represent that swing (a real captured
// phrase peaks at only 12.8 % of 14-bit full scale). Applying a board gain to an uncalibrated
// quantity is meaningless alone, and doing it for ONE chip while the other two keep hand-tuned
// scales is what put speech 8 dB below the music when it had been 1 dB above.
// 740/256 = 2.892 restores the net the hardware-confirmed mix had (0.6916 x 2.892 = 2.0), so the
// board's INTERNAL ratios (preamp : attenuator : summing) stay exact and the fudge sits where the
// other two chips' fudges already sit.
// TODO (accuracy; needs hardware measurement): calibrate all three chips' digital full-scales to their
// real output swings, after which TMS_FS_Q8 becomes 1.0 by construction.
// Headroom: 8191 x 13.988 x 0.691 = 79,219 + YM's 16,383 max = 95,602 < 131,071 (18-bit signed).
// ---------------------------------------------------------------------------
localparam signed [27:0] TMS_PREAMP_Q8 = 28'sd1239;   // x4.84  board preamp (sheet 9A)
localparam signed [27:0] TMS_FS_Q8     = 28'sd740;    // x2.892 digital full-scale calibration
localparam signed [27:0] TMS_GAIN_Q8   = (TMS_PREAMP_Q8 * TMS_FS_Q8) >>> 8;   // x13.988

wire signed [17:0] ym_in_l = ym_l18 >>> 1;                          // ym * 1/2
wire signed [17:0] ym_in_r = ym_r18 >>> 1;
// POKEY idle-pedestal removal. `pokey*_snd` is offset binary: -32 means "no output",
// not "-32 of signal". Feeding it raw put a constant -16384 DC into the channel strip, and because
// the 0x187a attenuator is applied INSIDE that strip, the gain multiplied the pedestal -- so a gain
// change stepped the summing node by up to 16384 (6.3 % of full scale) out of nothing. On the board
// POKEY sinks current into a virtual ground, so silence is ZERO current and there is no pedestal for
// the attenuator to scale; this was purely an artefact of our encoding meeting the gain stage.
//
// The artefact was measured once already: a constant -32 is a DC step that the 7.23 Hz
// interstage blocker rings out over ~100 ms, and that transient (peak 15336 vs the speech's
// 1560) completely swamps the result. 15336 against speech at 1560 is a pop over the speech.
//
// +32 makes silence contribute exactly 0. The AC content is bit-identical (this is a pure offset);
// the DC it adds is what the ~42 Hz output coupling cap already removes, so the steady-state balance
// is unchanged -- verified in simulation. Headroom: POKEY becomes unipolar 0..30720, so the
// worst-case sum goes 95,602 -> 126,322, still inside 18-bit signed (131,071).
wire signed [17:0] pk_in_l = (18'($signed(pokey1_snd)) + 18'sd32) <<< 9;   // pokey * 512, pedestal removed
wire signed [17:0] pk_in_r = (18'($signed(pokey2_snd)) + 18'sd32) <<< 9;
wire signed [27:0] tms_pre = 18'($signed(tms_snd)) * TMS_GAIN_Q8;
wire signed [17:0] tms_in  = 18'(tms_pre >>> 8);

// ---------------------------------------------------------------------------
// Per-chip channel strips. Corners are the measured RC products from the schematic, expressed as
// sys2_pole coefficients at the ce sample rate (a = 1-exp(-2*pi*fc/1.789772 MHz), Q0.18):
// 7.234 kHz -> 6574, 4.019 kHz -> 3673, 3.386 kHz -> 3098, 7.23 Hz -> 7.
// The 0x187a gain sits BETWEEN the two poles, exactly as on the board -- which matters because it is
// time-varying (the game ducks music under speech) and the second pole smooths the step instead of
// letting it click.
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// 0x187a gain slew -- fixes the intermittent pop over the level-select speech.
//
// The gains arrive as an INSTANTANEOUS step: when the game ducks the music under speech it moves
// gain_ym 256 -> 227 (1.0 -> 0.885) in one cycle. The channel output is filtered(x) * gain, so the
// output jumps by 0.115 * ym_instantaneous -- up to ~1,900 counts with ym_in at +-16,383. That is a
// broadband click, and its size depends on where the music waveform happens to be when the switch
// lands: near a peak it is loud, near a zero crossing it is inaudible. Hence a pop that only
// sometimes happens, only during speech.
//
// Confirmed on hardware: at the worst discontinuity the dominant mover was the YM path, with
// a real magnitude. Note that a raw single-sample delta measured at audio_l UNDERSTATES the
// step by 1/a = 18.4x, because of the 15.92 kHz output pole.
//
// Slewing one Q0.8 LSB per sample spreads the full 29-LSB duck over 29 ticks = ~16 us, pushing the
// transient's energy above ~60 kHz where the output pole removes it, while being far too fast to
// soften the duck itself (the game's ducking envelope is orders of magnitude slower). It is also
// MORE physical than a one-cycle switch: the board's LF13201 analog attenuator has a finite
// transition time feeding an RC network, not an ideal step. Steady-state gains are untouched, so the
// hardware-tuned balance is unchanged -- verified in simulation.
logic [8:0] g_ym_q  = 9'd256;
logic [8:0] g_pk_q  = 9'd256;
logic [8:0] g_tms_q = 9'd256;
always_ff @(posedge clk) begin
	if (rst) begin
		g_ym_q  <= gain_ym;          // load directly on reset -- no startup ramp
		g_pk_q  <= gain_pk;
		g_tms_q <= gain_tms;
	end else if (ce) begin
		if      (g_ym_q  < gain_ym)  g_ym_q  <= g_ym_q  + 9'd1;
		else if (g_ym_q  > gain_ym)  g_ym_q  <= g_ym_q  - 9'd1;
		if      (g_pk_q  < gain_pk)  g_pk_q  <= g_pk_q  + 9'd1;
		else if (g_pk_q  > gain_pk)  g_pk_q  <= g_pk_q  - 9'd1;
		if      (g_tms_q < gain_tms) g_tms_q <= g_tms_q + 9'd1;
		else if (g_tms_q > gain_tms) g_tms_q <= g_tms_q - 9'd1;
	end
end
// ---------------------------------------------------------------------------

wire signed [17:0] ym_ch_l, ym_ch_r, pk_ch_l, pk_ch_r, tms_ch;

sys2_chanfilt #(.A_PRE(19'd6574), .A_OUT(19'd6574)) u_cf_ym_l (   // YM 7.234k / 7.234k
	.clk(clk), .ce(ce), .rst(rst), .x(ym_in_l), .gain(g_ym_q),    .y(ym_ch_l));
sys2_chanfilt #(.A_PRE(19'd6574), .A_OUT(19'd6574)) u_cf_ym_r (
	.clk(clk), .ce(ce), .rst(rst), .x(ym_in_r), .gain(g_ym_q),    .y(ym_ch_r));
sys2_chanfilt #(.A_PRE(19'd6574), .A_OUT(19'd3098)) u_cf_pk_l (   // POKEY 7.234k / 3.386k
	.clk(clk), .ce(ce), .rst(rst), .x(pk_in_l), .gain(g_pk_q),    .y(pk_ch_l));
sys2_chanfilt #(.A_PRE(19'd6574), .A_OUT(19'd3098)) u_cf_pk_r (
	.clk(clk), .ce(ce), .rst(rst), .x(pk_in_r), .gain(g_pk_q),    .y(pk_ch_r));
sys2_chanfilt #(.A_PRE(19'd3673), .A_OUT(19'd3673)) u_cf_tms (    // TMS 4.019k / 4.019k (mono)
	.clk(clk), .ce(ce), .rst(rst), .x(tms_in),  .gain(g_tms_q), .y(tms_ch));

// ---------------------------------------------------------------------------
// Summing node: R139/R142 = 47K (POKEY), R141/R144 = 47K (YM), R140/R143 = 68K (TMS), all into the
// common 4.7K load. Only the ratios matter -- 1 : 1 : 47/68 = 0.691 (Q8: 177).
// ---------------------------------------------------------------------------
wire signed [27:0] tms_w   = tms_ch * 28'sd177;
wire signed [17:0] tms_sum = 18'(tms_w >>> 8);
wire signed [17:0] mix_l   = ym_ch_l + pk_ch_l + tms_sum;
wire signed [17:0] mix_r   = ym_ch_r + pk_ch_r + tms_sum;

// ---------------------------------------------------------------------------
// Shared OUTPUT network (both channels identical on the board). The 15.92 kHz pole is R153/R154 1K
// against C111/C110 .01uF at the connector; the ~42 Hz high-pass is the C131/C130 470uF output
// coupling cap into an 8 ohm speaker (Paperboy drives ~10 W into 8 ohm per side).
// NOTE the board also has a ~33.9 Hz TDA2002 feedback rolloff (C136 470uF / R150 10 ohm, a = 31)
// which is derived but deliberately NOT instantiated -- the only remaining analog delta besides the
// cabinet speaker response, which is out of scope (the core stops at the amplifier output).
// ---------------------------------------------------------------------------
sys2_rcmix #(.W(18), .A_LP(19'd14245), .A_DC(19'd39)) u_rcmix (
	.clk     (clk),
	.ce      (ce),
	.rst     (rst),
	.bypass  (1'b0),                 // filter always engaged
	.mix_l   (mix_l),
	.mix_r   (mix_r),
	.audio_l (audio_l),
	.audio_r (audio_r)
);

endmodule
