// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// 720's stick ANGLE -> rotate-disc movement.
//
// ---------------------------------------------------------------------------
// Why this exists
//
// 720's controller is a joystick that turns on its shaft, and the game reads only that
// rotation (the rotate disc on LETA1, plus the centre disc on LETA0 for "top"). The other
// three LETA games are steering wheels, and sys2_analog_spin turns a thumbstick's deflection
// into a wheel's turn RATE. That is the wrong model for 720: there the skater should face
// where the stick POINTS. With the rate mapping, a thumbstick on 720's Control Test kept
// the control spinning instead of following the stick.
//
// This module is MAME's "Fake Joystick" mode for 720 (atarisy2.cpp leta_r, `case 1`):
//
//     angle = atan2(x, y)            y = up-positive (FAKE_JOY_Y carries PORT_REVERSE)
//     only updated when |x| > 32 or |y| > 32, otherwise the last angle is held
//     LETA1 = rotations*144 + angle*144/360
//
// so 0 is the top, the angle increases CLOCKWISE, and 144 rotate counts are one turn.
//
// Divergence from MAME, deliberate: MAME writes that angle straight into the LETA
// register. This core never writes the LETA counter (see sys2_quad_gen's header -- 720's
// calibration screen counts real edges), so the angle is turned into a count delta: the
// shortest signed move from where the shaft is now (`pos`, owned by sys2_center_disc) to
// where the stick points. That delta goes down the same quadrature path a real encoder
// drives, so the 2x decode, the input filter and the centre disc all still apply.
//
// `pos` MUST come from the centre disc, not from a private copy here. It is the one
// position that every source of shaft movement (this module AND a real spinner) adds into,
// and it is what the centre notches are placed against. So "stick straight up" is position 0,
// which is exactly where the notches are, and the game's ZEROED lands at the top.
//
// ---------------------------------------------------------------------------
// How the angle is computed
//
// A 12-iteration vectoring CORDIC, one iteration per clock, started on each `tick` (the
// LETA's 156 kHz phi2). That leaves ~190 idle clocks per tick at 32 MHz, so a sequential
// unit costs a handful of adders where an atan2 table or a divider would cost a lot more.
// Angle unit: one turn = 36864 = 144 * 256, so a rotate count is exactly 256 units and the
// position is a shift. Residual error is a few units (well under 0.1 degree) against a
// 2.5-degree count.
//
// HYSTERESIS: the shaft only moves when the stick angle leaves the current count's
// 2.5-degree window by more than HYST units. Without it, a stick resting on a count
// boundary dithers one count back and forth forever. MAME has no hysteresis because it
// recomputes a register value per read; here every dither is a real pair of encoder edges.
// ---------------------------------------------------------------------------
module sys2_stick_angle #(
	// MAME's centre window: |x| or |y| beyond this (of +/-127) counts as "pointing".
	parameter int THRESH = 32,
	// Hysteresis beyond the count's own window, in angle units (256 = one count).
	parameter int HYST   = 64
) (
	input  logic              clk,
	input  logic              reset,
	input  logic              tick,      // start one conversion
	input  logic              enable,    // 720 only

	input  logic signed [7:0] ax,        // MiSTer signed: right is positive
	input  logic signed [7:0] ay,        // MiSTer signed: DOWN is positive (hardware)

	input  logic [7:0]        pos,       // current shaft position, 0..143, 0 = top

	output logic signed [8:0] delta,     // rotate counts to move, (-72, +72]
	output logic              delta_valid
);

// Named, fully sized constants rather than inline casts of an int: Quartus 17.0 rejects
// some inline size casts (a negated one above all) that every simulator and lint accepts.
localparam logic signed [17:0] TURN_S  = 18'sd36864;   // 144 * 256
localparam logic signed [17:0] HALF_S  = 18'sd18432;
localparam logic signed [17:0] QUART_S = 18'sd9216;
localparam logic signed [17:0] NQUART_S = -18'sd9216;
localparam logic signed [17:0] NHALF_S = -18'sd18432;

localparam logic signed [8:0]  THRESH_S = 9'sd0 + 9'(THRESH);
localparam logic signed [17:0] WIN_S    = 18'sd128 + 18'(HYST);   // half a count + HYST
localparam logic signed [17:0] NWIN_S   = 18'sd0 - WIN_S;

// atan(2^-i) in angle units, rounded.
function automatic logic signed [17:0] atan_tab(input logic [3:0] i);
	case (i)
		4'd0:    atan_tab = 18'sd4608;
		4'd1:    atan_tab = 18'sd2720;
		4'd2:    atan_tab = 18'sd1437;
		4'd3:    atan_tab = 18'sd730;
		4'd4:    atan_tab = 18'sd366;
		4'd5:    atan_tab = 18'sd183;
		4'd6:    atan_tab = 18'sd92;
		4'd7:    atan_tab = 18'sd46;
		4'd8:    atan_tab = 18'sd23;
		4'd9:    atan_tab = 18'sd11;
		4'd10:   atan_tab = 18'sd6;
		default: atan_tab = 18'sd3;
	endcase
endfunction

// Sign-extend, flip Y to up-positive. 9 bits so that -(-128) = +128 is representable.
wire signed [8:0] sx = {ax[7], ax};
wire signed [8:0] sy = {ay[7], ay};
wire signed [8:0] up = 9'sd0 - sy;
wire signed [8:0] mx = sx[8] ? (9'sd0 - sx) : sx;
wire signed [8:0] my = sy[8] ? (9'sd0 - sy) : sy;
wire pointing = (mx > THRESH_S) || (my > THRESH_S);

// CORDIC state. Vector (X, Y) = (up, right) scaled by 32: |input| <= 128 -> 4096, and the
// CORDIC gain (1.647) times the sqrt(2) of a diagonal stays under 9600, inside 16 bits.
logic signed [15:0] cx, cy;
logic signed [17:0] cz;
logic [3:0]         it;
logic               busy;

wire signed [15:0] x0 = 16'(up) <<< 5;
wire signed [15:0] y0 = 16'(sx) <<< 5;

wire signed [15:0] xs = cx >>> it;
wire signed [15:0] ys = cy >>> it;

// Final angle folded to 0..TURN-1, then the NEAREST count (rounded, so count p is centred
// on p * 2.5 degrees and all eight D-pad directions land mid-count: up 0, right 36, down 72,
// left 108 -- never on a boundary, where they would dither) and the distance from the
// current position's centre.
logic signed [17:0] zu, zr;
logic [7:0]         tgt;
logic signed [17:0] dfine;
logic signed [8:0]  dcount;

always_comb begin
	zu = (cz < 0) ? (cz + TURN_S) : cz;
	if (zu >= TURN_S) zu = zu - TURN_S;
	zr = zu + 18'sd128;
	if (zr >= TURN_S) zr = zr - TURN_S;
	tgt = zr[15:8];   // zr < 36864, so bits 17:16 are zero and this is 0..143

	// Fine distance from the current count's centre, wrapped to (-half, half].
	dfine = zu - $signed({2'b00, pos, 8'd0});
	if (dfine >  HALF_S)  dfine = dfine - TURN_S;
	if (dfine <= NHALF_S) dfine = dfine + TURN_S;

	// Shortest count move, wrapped to (-72, 72].
	dcount = $signed({1'b0, tgt}) - $signed({1'b0, pos});
	if (dcount >  9'sd72)  dcount = dcount - 9'sd144;
	if (dcount <= -9'sd72) dcount = dcount + 9'sd144;
end

wire outside = (dfine > WIN_S) || (dfine < NWIN_S);

always_ff @(posedge clk) begin
	delta_valid <= 1'b0;
	delta       <= '0;

	if (reset) begin
		busy <= 1'b0;
		it   <= 4'd0;
		cx   <= '0;
		cy   <= '0;
		cz   <= '0;
	end else if (!busy) begin
		if (tick && enable && pointing) begin
			// Pre-rotate into the right half-plane so the iterations only cover +/-90.
			if (x0 < 0) begin
				if (y0 >= 0) begin cx <= y0;          cy <= 16'sd0 - x0; cz <= QUART_S;  end
				else         begin cx <= 16'sd0 - y0; cy <= x0;          cz <= NQUART_S; end
			end else begin
				cx <= x0; cy <= y0; cz <= '0;
			end
			it   <= 4'd0;
			busy <= 1'b1;
		end
	end else if (it != 4'd12) begin
		if (cy >= 0) begin
			cx <= cx + ys; cy <= cy - xs; cz <= cz + atan_tab(it);
		end else begin
			cx <= cx - ys; cy <= cy + xs; cz <= cz - atan_tab(it);
		end
		it <= it + 4'd1;
	end else begin
		busy <= 1'b0;
		if (outside && dcount != 9'sd0) begin
			delta       <= dcount;
			delta_valid <= 1'b1;
		end
	end
end

endmodule
