// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Atari Slapstic for the whole System 2 family: types 137412-105/107/108/109/110.
//
// All five are "basic banking" variants -- identical FSM, different constants -- so this
// module is the earlier type-105-only implementation's state machine verbatim with its baked-in constants lifted
// into a runtime-selected table. One bitstream serves every game; the type arrives in the
// MRA index-1 game descriptor.
//
// On System 2 the bank output IS the VMMU/vram view select, not merely a ROM bank. A
// wrong table does not just mis-bank code -- it scrambles which VRAM plane the CPU writes.
// That is why the type is validated against the game id before the CPU is released
// (rtl/rom/sys2_rom_loader.sv) and why `type_supported` exists here as a second net.
//
// CONSTANTS. Transcribed from the tables in the Arcade-Atari-system1 core's
// `SLAPSTIC.vhd`, which carries MAME's `slapstic_data` structs verbatim as comments.
// Reference cores are weighed rather than trusted, and that is a second copy of MAME
// rather than independent evidence: the type-105 column is the one with hardware behind
// it, and simulation proves this module reproduces the proven 105 implementation cycle
// for cycle. The other four columns rest on MAME alone.
//
// What actually varies across the five, i.e. what could NOT be derived:
//   * bank-select values differ in SPACING -- 105 strides by 4, 107/108/109 by 2,
//     110 by 0x10 -- so they are four explicit values, not base + n*k.
//   * mask_alt4 is 0x3ff3 (105), 0x3ff9 (107/108/109) or 0x3fcf (110).
// Uniform across all five, and therefore not parameterised: starting bank 3, altshift 0,
// no additive mode, mask_alt1 0x007f, mask_alt2 0x3fff, mask_alt3 0x3ffc, mask_bit1
// 0x3ff0, mask_bit2* 0x3ff3, mask_bit3 0x3ff8.
//
// Address constants are chip WORD addresses; T-11 byte address bit 0 is ignored.
module sys2_slapstic (
	input  logic        clk,
	input  logic        reset,
	input  logic [7:0]  slapstic_type,
	input  logic        access,
	/* verilator lint_off UNUSEDSIGNAL */
	input  logic [15:0] address,
	/* verilator lint_on UNUSEDSIGNAL */
	output logic [1:0]  bank,
	output logic        type_supported
);

typedef enum logic [2:0] {
	ST_IDLE,
	ST_ACTIVE,
	ST_ALT_VALID,
	ST_ALT_SELECT,
	ST_ALT_COMMIT,
	ST_BIT_LOAD,
	ST_BIT_ODD,
	ST_BIT_EVEN
} state_t;

state_t state;
logic [1:0]  loaded_bank;
logic [13:0] chip_address;

assign chip_address = address[14:1];

// ---------------------------------------------------------------------------
// Per-type constant table (runtime mux; the type is fixed for a given MRA).
// ---------------------------------------------------------------------------
logic [13:0] v_bank0, v_bank1, v_bank2, v_bank3;
logic [13:0] v_alt1, v_alt2, v_alt3;
logic [13:0] m_alt4, v_alt4;
logic [13:0] v_bit1, v_bit3;

always_comb begin
	// Default to the 105 column so an unsupported type is inert rather than random;
	// type_supported is what actually flags it.
	v_bank0 = 14'h0010; v_bank1 = 14'h0014; v_bank2 = 14'h0018; v_bank3 = 14'h001c;
	v_alt1  = 14'h003d; v_alt2  = 14'h0092; v_alt3  = 14'h00a4;
	m_alt4  = 14'h3ff3; v_alt4  = 14'h0010;
	v_bit1  = 14'h35b0; v_bit3  = 14'h35c0;
	type_supported = 1'b0;

	case (slapstic_type)
		8'd105: begin // Indiana Jones / Paperboy (confirmed)
			v_bank0 = 14'h0010; v_bank1 = 14'h0014; v_bank2 = 14'h0018; v_bank3 = 14'h001c;
			v_alt1  = 14'h003d; v_alt2  = 14'h0092; v_alt3  = 14'h00a4;
			m_alt4  = 14'h3ff3; v_alt4  = 14'h0010;
			v_bit1  = 14'h35b0; v_bit3  = 14'h35c0;
			type_supported = 1'b1;
		end
		8'd107: begin // Peter Packrat / Xybots / 2p Gauntlet / 720 (confirmed)
			v_bank0 = 14'h0018; v_bank1 = 14'h001a; v_bank2 = 14'h001c; v_bank3 = 14'h001e;
			v_alt1  = 14'h006b; v_alt2  = 14'h3d52; v_alt3  = 14'h3d64;
			m_alt4  = 14'h3ff9; v_alt4  = 14'h0018;
			v_bit1  = 14'h00a0; v_bit3  = 14'h00b0;
			type_supported = 1'b1;
		end
		8'd108: begin // Road Runner / Super Sprint (confirmed)
			v_bank0 = 14'h0028; v_bank1 = 14'h002a; v_bank2 = 14'h002c; v_bank3 = 14'h002e;
			v_alt1  = 14'h001f; v_alt2  = 14'h3772; v_alt3  = 14'h3764;
			m_alt4  = 14'h3ff9; v_alt4  = 14'h0028;
			v_bit1  = 14'h0060; v_bit3  = 14'h0070;
			type_supported = 1'b1;
		end
		8'd109: begin // Championship Sprint
			v_bank0 = 14'h0008; v_bank1 = 14'h000a; v_bank2 = 14'h000c; v_bank3 = 14'h000e;
			v_alt1  = 14'h002b; v_alt2  = 14'h0052; v_alt3  = 14'h0064;
			m_alt4  = 14'h3ff9; v_alt4  = 14'h0008;
			v_bit1  = 14'h3da0; v_bit3  = 14'h3db0;
			type_supported = 1'b1;
		end
		8'd110: begin // APB
			v_bank0 = 14'h0040; v_bank1 = 14'h0050; v_bank2 = 14'h0060; v_bank3 = 14'h0070;
			v_alt1  = 14'h002d; v_alt2  = 14'h3d14; v_alt3  = 14'h3d24;
			m_alt4  = 14'h3fcf; v_alt4  = 14'h0040;
			v_bit1  = 14'h34c0; v_bit3  = 14'h34d0;
			type_supported = 1'b1;
		end
		default: ;   // keep the inert defaults above, type_supported stays 0
	endcase
end

// bit2c0/s0/c1/s1 are bit1's value with the low two bits carrying the
// clear/set-bit-0/1 selector, in every one of the five tables.
wire [13:0] v_bit2c0 = {v_bit1[13:2], 2'b00};
wire [13:0] v_bit2s0 = {v_bit1[13:2], 2'b01};
wire [13:0] v_bit2c1 = {v_bit1[13:2], 2'b10};
wire [13:0] v_bit2s1 = {v_bit1[13:2], 2'b11};

// Two matchers, because MAME has two, and the difference is not an oversight.
//
// On System 2 the slapstic is configured at 0x8000-0x81FF -- MAME gives all five games
// `set_range(m_maincpu, AS_PROGRAM, 0100000, 0100777, 0)`, octal. But it taps the WHOLE address
// space and decides per matcher whether the window applies (MAME slapstic.cpp):
//
//   test_in (mv)  = test(m_range_mask | (mv.mask << 1), m_range_value | (mv.value << 1))
//   test_any(mv)  = test(               mv.mask << 1,                  mv.value << 1 )
//
// `test_in` folds the window in; `test_any` does not, so it fires anywhere in the 64K space.
//
// For chips 103-110 -- which is all five of ours -- `active_103_110` builds alt1 with
// test_any, and MAME says why in its own section header: "Active state, 103-110, has direct,
// alt and bitwise, and alt can be done ANYWHERE". The 101/102 variant beside it uses test_in and
// is captioned "alt must be done in-range". Every other matcher we implement -- alt2, alt3,
// alt4, bit1, bit3 -- uses test_in. So alt1 takes `match_any` and the rest take `match_in`.
//
// Verified exhaustively over all 65536 addresses for all five types: this pairing reproduces
// MAME's matcher set exactly, matcher for matcher.
//
// A warning to future readers: alt1's use of `match_any` looks like a missing window check
// (4 legal addresses vs 512) and has been "fixed" once already -- wrongly. Reading MAME's
// `checker::test_in` and generalising it to every matcher is the natural mistake, but the
// call sites are four lines of C++ that say `test_any` in plain sight, under a comment that
// states the rule in English. Enumerate the call sites, not just the helper: a reference
// implementation's behaviour is where it is invoked, not where it is defined.

// MAME's `checker::test_any`: the matcher's own mask and value ONLY, with no window folded
// in, so it fires anywhere in the 64K space. Used by exactly one matcher -- alt1 -- and only
// because MAME uses it there for chips 103-110. See the note above `match_in`.
function automatic logic match_any(
	input logic [13:0] mask,
	input logic [13:0] expected
);
	match_any = ((chip_address & mask) == expected);
endfunction

function automatic logic match_in(
	input logic [13:0] mask,
	input logic [13:0] expected
);
	match_in = address[15]
	        && (chip_address[13:8] == expected[13:8])
	        && ((chip_address[7:0] & mask[7:0]) == expected[7:0]);
endfunction

function automatic logic reset_access;
	reset_access = address[15] && (chip_address == 14'h0000);
endfunction

always_ff @(posedge clk) begin
	if (reset) begin
		state <= ST_IDLE;
		bank <= 2'd3;
		loaded_bank <= 2'd3;
	end else if (access) begin
		case (state)
			ST_IDLE: begin
				if (reset_access()) state <= ST_ACTIVE;
			end

			ST_ACTIVE: begin
				if (address[15] && chip_address == v_bank0) begin
					bank <= 2'd0;
					state <= ST_IDLE;
				end else if (address[15] && chip_address == v_bank1) begin
					bank <= 2'd1;
					state <= ST_IDLE;
				end else if (address[15] && chip_address == v_bank2) begin
					bank <= 2'd2;
					state <= ST_IDLE;
				end else if (address[15] && chip_address == v_bank3) begin
					bank <= 2'd3;
					state <= ST_IDLE;
				end else if (match_any(14'h007f, v_alt1)) begin
					state <= ST_ALT_VALID;
				end else if (match_in(14'h3ff0, v_bit1)) begin
					state <= ST_BIT_LOAD;
				end
			end

			ST_ALT_VALID: begin
				if (reset_access()) begin
					state <= ST_ACTIVE;
				end else if (match_in(14'h3fff, v_alt2)) begin
					state <= ST_ALT_SELECT;
				end else begin
					state <= ST_ACTIVE;
				end
			end

			ST_ALT_SELECT: begin
				if (reset_access()) begin
					state <= ST_ACTIVE;
				end else if (match_in(14'h3ffc, v_alt3)) begin
					loaded_bank <= chip_address[1:0];   // altshift = 0 in all five tables
					state <= ST_ALT_COMMIT;
				end else begin
					state <= ST_ACTIVE;
				end
			end

			ST_ALT_COMMIT: begin
				if (reset_access()) begin
					state <= ST_ACTIVE;
				end else if (match_in(m_alt4, v_alt4)) begin
					bank <= loaded_bank;
					state <= ST_IDLE;
				end
			end

			ST_BIT_LOAD: begin
				if (reset_access()) begin
					state <= ST_ACTIVE;
				end else if (match_in(m_alt4, v_alt4)) begin
					loaded_bank <= bank;
					state <= ST_BIT_ODD;
				end
			end

			ST_BIT_ODD: begin
				if (reset_access()) begin
					state <= ST_ACTIVE;
				end else if (match_in(14'h3ff3, v_bit2c0)) begin
					loaded_bank[0] <= 1'b0;
					state <= ST_BIT_EVEN;
				end else if (match_in(14'h3ff3, v_bit2s0)) begin
					loaded_bank[0] <= 1'b1;
					state <= ST_BIT_EVEN;
				end else if (match_in(14'h3ff3, v_bit2c1)) begin
					loaded_bank[1] <= 1'b0;
					state <= ST_BIT_EVEN;
				end else if (match_in(14'h3ff3, v_bit2s1)) begin
					loaded_bank[1] <= 1'b1;
					state <= ST_BIT_EVEN;
				end else if (match_in(14'h3ff8, v_bit3)) begin
					bank <= loaded_bank;
					state <= ST_IDLE;
				end
			end

			// The even half of the bitwise sequence reverses the selector sense: the
			// same four addresses mean the opposite bit and the opposite value. This
			// alternation is inherited verbatim from the hardware-proven type-105
			// implementation -- do not "simplify" it to match a reference model
			// without hardware evidence.
			ST_BIT_EVEN: begin
				if (reset_access()) begin
					state <= ST_ACTIVE;
				end else if (match_in(14'h3ff3, v_bit2s1)) begin
					loaded_bank[0] <= 1'b0;
					state <= ST_BIT_ODD;
				end else if (match_in(14'h3ff3, v_bit2c1)) begin
					loaded_bank[0] <= 1'b1;
					state <= ST_BIT_ODD;
				end else if (match_in(14'h3ff3, v_bit2s0)) begin
					loaded_bank[1] <= 1'b0;
					state <= ST_BIT_ODD;
				end else if (match_in(14'h3ff3, v_bit2c0)) begin
					loaded_bank[1] <= 1'b1;
					state <= ST_BIT_ODD;
				end else if (match_in(14'h3ff8, v_bit3)) begin
					bank <= loaded_bank;
					state <= ST_IDLE;
				end
			end

			default: begin
				state <= ST_IDLE;
				bank <= 2'd3;
				loaded_bank <= 2'd3;
			end
		endcase
	end
end

endmodule
