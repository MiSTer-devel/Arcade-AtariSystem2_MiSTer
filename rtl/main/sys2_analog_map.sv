// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Per-game ADC0809 channel map.
//
// This is a module rather than an `always_comb` in the top level for one reason: the
// per-game input maps are exactly the kind of table that fails quietly -- a pedal on the
// wrong channel makes the car crawl, it does not crash -- so it has to be testable
// without elaborating `emu`. The T-11 IN0 map lives inside sys2_main_bus and is
// covered the same way for the same reason.
//
// Channel assignment, from each game's MAME PORT_MODIFY("ADCn"):
//
//   channel   paperboy      720   ssprint      csprint      apb
//   ADC0      handlebar X   --    P1 pedal     P1 pedal     --
//   ADC1      handlebar Y   --    P2 pedal     P2 pedal     pedal
//   ADC2      --            --    P3 pedal     --           --
//   ADC3-7    --            --    --           --           --
//
// 720 uses no ADC channel at all: it steers entirely through the LETA.
// APB's pedal is on ADC1, NOT ADC0 -- its PORT_MODIFY clears ADC0 to IPT_UNUSED.
//
// ---------------------------------------------------------------------------
// The pedal ENCODING is CLOSED. This block used to be a two-part VERIFY; both parts
// were answered by research and the whole path was then confirmed on
// hardware. Kept (rather than deleted) because the endpoints below are only
// readable if you know where they came from.
//
// MAME declares the Sprint/APB pedals `PORT_MINMAX(0x00,0x3f) ... PORT_INVERT`, so the
// converter reads only the bottom quarter of full scale and rests HIGH. The two things
// that were inferred are now sourced:
//
//   1. PORT_INVERT IS a bitwise `~` of the accumulated value -- `src/emu/ioport.h` comments
//      it "positional control bits are active low" and `ioport.cpp` applies a plain
//      `value = ~value`. NOT a reflection within MINMAX. So released = 0xff and floored =
//      0xc0, which is the reading PEDAL_RELEASED/PEDAL_FLOORED already carried.
//   2. The 0x00-0x3f span is mechanical travel, not MAME clamping and not electronics.
//      SP-290 sheet 8B shows the pedal pot going into the ADC0809 through a 100 R series
//      resistor with NO divider, i.e. the converter sees the full rail.
//
// Confirmed on hardware via APB's Pedal Test, which prints lo/hi/range/raw/
// normalized rather than the Sprints' bare lo/value/hi: raw moves 00 -> 3F and normalized
// 00 -> FF as the pedal travels. That is a live ADC reading rather than a stored
// calibration record, which is the discriminating evidence the question needed.
//
// The self-test's apparent COMPLEMENT of our value is the GAME's display, not our ADC.
// All four hops were traced: this module emits `PEDAL_RELEASED - travel` (0xFF idle /
// 0xC0 floored, matching MAME) -> `adc_in` -> `adc0809` straight capture -> {8'hff, data}.
// There is no inversion anywhere in the path; the game inverts for a sensible display,
// which is why its `raw` column reads 00..3F while the ADC carries FF..C0.
//
// One honest residual, and it is calibration rather than encoding. TM-290 Fig 3-9
// prints a REAL Super Sprint cabinet's Pedal Test: rest ~0x67, pressed ~0x96 -- mid-scale,
// ~0x2F of travel, and INCREASING with press, i.e. opposite in direction and offset to the
// MAME-derived 0xff -> 0xc0 here. Both are playable because the pedals self-calibrate and
// the self-test grades range SIZE (ours 0x3F exceeds the cabinet's ~0x2E), so this is not
// known to affect play and is not a reason to move the constants. If a Sprint's own Pedal
// Test is ever run on the board, that is the comparison to make.
//
// The two endpoints stay named constants carrying their derivation rather than magic
// numbers. The channel map above comes straight from the per-game PORT_MODIFY list and is
// what this module guarantees.
// ---------------------------------------------------------------------------
module sys2_analog_map (
	input  logic [7:0] game_id,

	// Paperboy's handlebar, already converted to the chip's 0x00-0xff scale by the caller.
	input  logic [7:0] steer_x,
	input  logic [7:0] speed_y,

	// Gas. The only source -- see the pedal() note below for why the stick's Y axis is not
	// one, and must not be added back.
	input  logic       press_0,
	input  logic       press_1,
	input  logic       press_2,

	output logic [7:0] adc_out [0:7]
);

`include "rtl/rom/sys2_rom_layout.vh"

localparam logic [7:0] PEDAL_RELEASED = 8'hff;   // ~0x00
localparam logic [7:0] PEDAL_FLOORED  = 8'hc0;   // ~0x3f
localparam logic [5:0] PEDAL_TRAVEL   = 6'h3f;   // full-scale travel, 0..0x3f

// Gas is the button and only the button. The stick's Y axis is not a pedal.
//
// This used to read the left stick's Y as an analog pedal -- `travel = stick[14:9]` whenever
// Y was positive -- with the button as a digital fallback. On hardware, Super Sprint:
// pressing DOWN on the stick accelerated the car. Positive Y is DOWN on the pad, so the
// axis was live and pointing the wrong way, and there is no polarity that makes it right:
//
//   On these three games the left stick is the steering wheel. Its X drives
//   sys2_analog_spin -> sys2_quad_gen -> the LETA. A player mid-corner cannot hold Y at
//   exactly zero, so ANY mapping of that stick's Y to the pedal makes steering modulate the
//   throttle. Flipping the sign would only move the problem from "down accelerates" to "up
//   accelerates" while leaving the coupling untouched.
//
// The code already knew this and applied it only half-way. atarisys2.sv's gas comment
// rejects the D-pad up bit in these exact terms -- "the same stick is also the STEERING WHEEL,
// so up fights the axis the player is already using" -- and then left the ANALOG Y of that
// same stick wired to the pedal. The stated principle was right; it was applied to one of the
// two inputs that violate it.
//
// So the pedal is now digital: released, or full travel. The self-test grades range SIZE
// (TM-290 Fig 3-9) and full scale is still reachable, so it still passes. What is lost is
// analog throttle modulation; if that is wanted back it belongs on an axis the wheel does not
// own -- the RIGHT stick -- not on this one.
function automatic logic [7:0] pedal(input logic pressed);
	logic [5:0] travel;
	begin
		travel = pressed ? PEDAL_TRAVEL : 6'd0;
		pedal  = PEDAL_RELEASED - {2'b00, travel};
	end
endfunction

wire [7:0] pedal_p1 = pedal(press_0);
wire [7:0] pedal_p2 = pedal(press_1);
wire [7:0] pedal_p3 = pedal(press_2);

integer i;
always_comb begin
	// Unused channels read 0x00, which is what this core has always presented for
	// every idle channel.
	for (i = 0; i < 8; i = i + 1) adc_out[i] = 8'h00;

	case (game_id)
		SYS2_GAME_720: ;                       // no ADC channels at all
		SYS2_GAME_SSPRINT: begin
			adc_out[0] = pedal_p1;
			adc_out[1] = pedal_p2;
			adc_out[2] = pedal_p3;
		end
		SYS2_GAME_CSPRINT: begin               // two players, no ADC2
			adc_out[0] = pedal_p1;
			adc_out[1] = pedal_p2;
		end
		SYS2_GAME_APB: begin
			adc_out[1] = pedal_p1;             // APB's pedal is on ADC1, not ADC0
		end
		default: begin                         // Paperboy
			adc_out[0] = steer_x;              // ch0 = handlebar X (steer)
			adc_out[1] = speed_y;              // ch1 = handlebar Y (speed)
		end
	endcase
end

endmodule
