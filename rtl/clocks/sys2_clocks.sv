// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Paperboy synchronous clock-enable generator.
//
// Clock collapse: the core runs in a SINGLE clock domain. clk_t11 is aliased
// to clk_sys (32 MHz) at the top level, so every former clk_t11<->clk_sys crossing (vblank/
// scanline IRQ, maincpu ROM read, scroll) is now same-clock and deterministic -- eliminating the
// metastable-CDC bug class that black-screened the board. Both the
// `clk_t11` and `clk_snd` inputs are therefore driven by clk_sys (32 MHz). The clk_t11 logic
// Fmax is 57.91 MHz, comfortably above 32 MHz, so running it here meets timing.
//
// This module divides the 32 MHz master into the per-subsystem clock enables.
//
// EXACT board clock rates via fractional clock-enables (JTFRAME `jtframe_frac_cen`
// style). The prior mod-3/9/18/51 integer dividers landed every rate slightly off the PCB (T-11
// +6.7 %, the sound chain -0.66 %). A phase accumulator that adds NUM per 32 MHz clock modulo DEN
// and emits an enable on each overflow produces an EXACT average rate of 32 MHz * NUM/DEN (with at
// most 1-clk / 31 ns jitter, invisible to these bus-/vblank-paced consumers). The chosen ratios hit
// the crystal frequencies exactly:
//   cpu_clk_en  : 5/16    -> 10.000000 MHz  (T-11 execution; PCB 10 MHz exactly)
//   adc_clk_en  : 5/256   ->  0.625000 MHz  (ADC0809;        PCB 625 kHz exactly)
//   snd_ym_en   : 315/2816-> 3.5795455 MHz  (YM2151 cen;     PCB 3.579545 MHz exactly)
//   snd_cpu_en  : YM/2    -> 1.7897727 MHz  (6502;           PCB 1.789772 MHz exactly)
//   snd_pokey_en: YM/2    -> 1.7897727 MHz  (both POKEYs;    PCB 1.789772 MHz)
//   snd_ym_cen_p1: YM/2   -> 1.7897727 MHz  (jt51 cen_p1; a coincident subset of snd_ym_en)
// The 6502/POKEY/cen_p1 enables are every-other YM tick, so they stay a same-cycle subset of
// snd_ym_en, keeping the 2:1 YM:6502 chain and jt51's "cen_p1 = cen at half speed" contract.
//
// T-11 headroom caveat: pinning the T-11 to EXACTLY 10 MHz drops it below the
// prior +6.7 % (10.67 MHz) rate, which had "masked a new-objects-arrive-
// late symptom." That symptom (graphic pop-in of houses/objects) was later root-caused to the sound
// stale-response desync -- the T-11 was reset-looping the sound 6502 and burning cycles in the
// error-recovery handshake -- and confirmed fixed on hardware. The
// real PCB runs the T-11 at exactly 10 MHz and renders fine, so matching that rate cannot itself
// reintroduce a throughput deficit; the "headroom" was never the fix.
//
// The periodic 6502 sound IRQ stays in sys2_sound_bus and counts raw 32 MHz
// clocks: IRQ_DIV = 131072 -> exactly 244.140625 Hz, the PCB divider-chain rate (SP7B-8A:
// 20 MHz / 2 / 16 / 16 / 16 / 10). The TMS5220's two selectable rates are likewise exact
// there via sys2_frac_cen (5/256 = 625.000 kHz, 5/192 = 833.333 kHz; 0x187c bit5 selects).
module sys2_clocks (
	input  logic clk_t11,        // 32 MHz master (= clk_sys after the clock collapse)
	input  logic clk_snd,        // 32 MHz master (tied to clk_t11 at the top)
	input  logic reset,          // synchronous reset (sync-released per domain)

	// clk_t11 (32 MHz) domain enables
	output logic cpu_clk_en,     // 10.000 MHz : T-11 execution clock (5/16)
	output logic adc_clk_en,     // 625 kHz    : ADC0809 (5/256)

	// clk_snd (32 MHz) domain enables
	output logic snd_cpu_en,     // 1.789772 MHz : 6502 (YM cen / 2)
	output logic snd_ym_en,      // 3.579545 MHz : YM2151 cen (315/2816)
	output logic snd_ym_cen_p1,  // 1.789772 MHz : YM2151 cen_p1 (subset of snd_ym_en)
	output logic snd_pokey_en    // 1.789772 MHz : both POKEYs (YM cen / 2)
);

// T-11 / ADC enables in the clk_t11 (32 MHz) domain.
sys2_frac_cen #(.NUM(5), .DEN(16))  u_cpu_cen (.clk(clk_t11), .reset(reset), .cen(cpu_clk_en));
sys2_frac_cen #(.NUM(5), .DEN(256)) u_adc_cen (.clk(clk_t11), .reset(reset), .cen(adc_clk_en));

// YM2151 cen in the clk_snd (32 MHz) domain: 315/2816 = exactly 3.579545 MHz.
logic ym_cen;
sys2_frac_cen #(.NUM(315), .DEN(2816)) u_ym_cen (.clk(clk_snd), .reset(reset), .cen(ym_cen));

// The 6502/POKEY tick and the YM cen_p1 are every other YM cen -- a same-cycle subset of ym_cen so
// the 2:1 YM:6502 ratio and jt51's cen_p1 contract hold. ym_half toggles on each ym_cen; gating on
// its pre-toggle value fires the /2 enables on alternating YM ticks (the first ym_cen included, as
// jt51 expects for cen_p1).
logic ym_half;
always_ff @(posedge clk_snd) begin
	if (reset)        ym_half <= 1'b0;
	else if (ym_cen)  ym_half <= ~ym_half;
end

assign snd_ym_en     = ym_cen;
assign snd_ym_cen_p1 = ym_cen & ~ym_half;
assign snd_cpu_en    = ym_cen & ~ym_half;
assign snd_pokey_en  = ym_cen & ~ym_half;

endmodule

// -----------------------------------------------------------------------------
// Fractional clock-enable generator (JTFRAME `jtframe_frac_cen` style).
//
// A phase accumulator adds NUM every clk, modulo DEN; each time the sum reaches
// DEN it wraps (subtract DEN) and asserts `cen` for one clk. Over any window of
// k*DEN clocks it emits exactly k*NUM enables, so the average rate is exactly
// clk * NUM/DEN, with at most 1-clk jitter between pulses. `reset` clears the
// accumulator for a deterministic start phase. Requires 0 < NUM < DEN.
//
// `cen` is a combinational decode of the registered accumulator (like the prior
// mod-N `assign cpu_clk_en = cnt==0` enables, which were HW-proven): it is a
// function of registered state, glitch-free between edges, and lands the enable
// on the exact posedge that advances the gated chip.
//
// Kept in this file (not its own) so every consumer/bench that already lists
// sys2_clocks.sv picks it up unchanged; that pairing trips DECLFILENAME only.
// -----------------------------------------------------------------------------
/* verilator lint_off DECLFILENAME */
module sys2_frac_cen #(
	parameter int NUM = 5,
	parameter int DEN = 16
) (
	input  logic clk,
	input  logic reset,
	output logic cen
);

localparam int W = $clog2(DEN + NUM);   // wide enough to hold cnt + NUM before the wrap

logic [W-1:0] cnt;
logic [W-1:0] nxt;

assign nxt = cnt + W'(NUM);
assign cen = (nxt >= W'(DEN));

always_ff @(posedge clk) begin
	if (reset)     cnt <= '0;
	else if (cen)  cnt <= nxt - W'(DEN);
	else           cnt <= nxt;
end

endmodule
/* verilator lint_on DECLFILENAME */
