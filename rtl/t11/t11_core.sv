// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

// Focused DEC T-11 compatible CPU for the Atari System 2 core.
// Behavioral sources: DEC DCT11-AA User's Guide and MAME t11 at d066f16.
module t11_core #(
	parameter logic [15:0] INITIAL_MODE = 16'h36ff
)(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce,

	output logic        bus_req,
	output logic        bus_write,
	output logic        bus_ifetch,
	output logic [15:0] bus_addr,
	output logic [15:0] bus_wdata,
	output logic [1:0]  bus_byte_en,
	input  logic [15:0] bus_rdata,
	input  logic        bus_ack,
	input  logic        bus_error,

	input  logic [3:0]  cp,
	input  logic        halt,
	input  logic        power_fail,
	output logic        reset_out,
	output logic        halted,
	output logic        waiting,
	output logic        retire
);

typedef enum logic [6:0] {
	ST_BOUNDARY,
	ST_FETCH_REQ, ST_FETCH_DONE, ST_DECODE,
	ST_BUS,
	ST_RESOLVE_START, ST_RESOLVE_EXT_DONE, ST_RESOLVE_PTR_DONE,
	ST_RESOLVE_VALUE_DONE, ST_RESOLVE_RETURN,
	ST_AFTER_SRC, ST_AFTER_DST, ST_EXECUTE, ST_WRITE_DONE,
	ST_JMP_DONE, ST_JSR_DONE, ST_JSR_PUSH_DONE,
	ST_RTS_POP_DONE, ST_RTI_PC_DONE, ST_RTI_PSW_DONE,
	ST_TRAP_VECTOR_PC_REQ, ST_TRAP_VECTOR_PC_DONE,
	ST_TRAP_VECTOR_PSW_REQ, ST_TRAP_VECTOR_PSW_DONE,
	ST_TRAP_PUSH_PSW_REQ, ST_TRAP_PUSH_PSW_DONE,
	ST_TRAP_PUSH_PC_REQ, ST_TRAP_PUSH_PC_DONE,
	ST_FINISH
} state_t;

typedef enum logic [5:0] {
	OP_NONE,
	OP_MOV, OP_CMP, OP_BIT, OP_BIC, OP_BIS, OP_ADD, OP_SUB,
	OP_CLR, OP_COM, OP_INC, OP_DEC, OP_NEG, OP_ADC, OP_SBC, OP_TST,
	OP_ROR, OP_ROL, OP_ASR, OP_ASL, OP_SWAB, OP_SXT, OP_XOR, OP_MFPS
} op_t;

logic [15:0] regs [0:7];
logic [7:0] psw;
logic [15:0] instruction;
state_t state;
state_t bus_return;
state_t resolve_return;

logic [15:0] bus_data;
logic [5:0] resolve_spec;
logic resolve_byte;
logic resolve_need_value;
logic resolve_pc_byte_word;
logic [15:0] resolve_ea;
logic [15:0] resolve_base;
logic resolve_is_reg;
logic [2:0] resolve_reg;
logic [15:0] resolve_value;

logic [15:0] src_value;
logic [15:0] dst_value;
logic dst_is_reg;
logic [2:0] dst_reg;
logic [15:0] dst_ea;
logic operation_byte;
op_t operation;

logic [15:0] exec_result;
logic [7:0] exec_psw;
logic exec_writes;

logic [7:0] trap_vector;
logic trap_hardware;
logic trap_halt;
logic trap_active;
logic [15:0] trap_new_pc;
logic [7:0] trap_new_psw;
logic halt_previous;
logic power_fail_previous;
logic halt_pending;
logic power_fail_pending;
logic trace_after_instruction;
logic trace_pending;
logic traced_instruction_trap;
logic rti_is_rtt;
logic timing_active;
logic [11:0] instruction_cycles;
logic [11:0] instruction_wait_cycles;
logic [11:0] nominal_cycles;

integer index;

function automatic logic [15:0] mode_initial_pc(input logic [2:0] mode);
	case (mode)
		3'd0: mode_initial_pc = 16'hc000;
		3'd1: mode_initial_pc = 16'h8000;
		3'd2: mode_initial_pc = 16'h4000;
		3'd3: mode_initial_pc = 16'h2000;
		3'd4: mode_initial_pc = 16'h1000;
		3'd5: mode_initial_pc = 16'h0000;
		3'd6: mode_initial_pc = 16'hf600;
		default: mode_initial_pc = 16'hf400;
	endcase
endfunction

function automatic logic [2:0] cp_priority(input logic [3:0] value);
	case (value)
		4'h0: cp_priority = 3'd0;
		4'h1, 4'h2, 4'h3: cp_priority = 3'd4;
		4'h4, 4'h5, 4'h6, 4'h7: cp_priority = 3'd5;
		4'h8, 4'h9, 4'ha, 4'hb: cp_priority = 3'd6;
		default: cp_priority = 3'd7;
	endcase
endfunction

function automatic logic [7:0] cp_vector(input logic [3:0] value);
	case (value)
		4'h0: cp_vector = 8'h00;
		4'h1: cp_vector = 8'h38; // 070
		4'h2: cp_vector = 8'h34; // 064
		4'h3: cp_vector = 8'h30; // 060
		4'h4: cp_vector = 8'h5c; // 134
		4'h5: cp_vector = 8'h58; // 130
		4'h6: cp_vector = 8'h54; // 124
		4'h7: cp_vector = 8'h50; // 120
		4'h8: cp_vector = 8'h4c; // 114
		4'h9: cp_vector = 8'h48; // 110
		4'ha: cp_vector = 8'h44; // 104
		4'hb: cp_vector = 8'h40; // 100
		4'hc: cp_vector = 8'h6c; // 154
		4'hd: cp_vector = 8'h68; // 150
		4'he: cp_vector = 8'h64; // 144
		default: cp_vector = 8'h60; // 140
	endcase
endfunction

function automatic logic branch_taken(input logic [7:0] opcode, input logic [3:0] flags);
	logic n, z, v, c;
	begin
		n = flags[3]; z = flags[2]; v = flags[1]; c = flags[0];
		case (opcode)
			8'h01: branch_taken = 1'b1;
			8'h02: branch_taken = !z;
			8'h03: branch_taken = z;
			8'h04: branch_taken = !(n ^ v);
			8'h05: branch_taken = n ^ v;
			8'h06: branch_taken = !(z | (n ^ v));
			8'h07: branch_taken = z | (n ^ v);
			8'h80: branch_taken = !n;
			8'h81: branch_taken = n;
			8'h82: branch_taken = !(c | z);
			8'h83: branch_taken = c | z;
			8'h84: branch_taken = !v;
			8'h85: branch_taken = v;
			8'h86: branch_taken = !c;
			8'h87: branch_taken = c;
			default: branch_taken = 1'b0;
		endcase
	end
endfunction

function automatic logic is_double_opcode(input logic [3:0] opcode);
	case (opcode)
		4'h1,4'h2,4'h3,4'h4,4'h5,4'h6,4'h9,4'ha,4'hb,4'hc,4'hd,4'he:
			is_double_opcode = 1'b1;
		default: is_double_opcode = 1'b0;
	endcase
endfunction

function automatic logic is_single_opcode(input logic [15:0] opcode);
	case (opcode)
		16'h0a00,16'h0a40,16'h0a80,16'h0ac0,16'h0b00,16'h0b40,16'h0b80,16'h0bc0,
		16'h0c00,16'h0c40,16'h0c80,16'h0cc0,16'h0dc0,
		16'h8a00,16'h8a40,16'h8a80,16'h8ac0,16'h8b00,16'h8b40,16'h8b80,16'h8bc0,
		16'h8c00,16'h8c40,16'h8c80,16'h8cc0,16'h8d00,16'h8dc0:
			is_single_opcode = 1'b1;
		default: is_single_opcode = 1'b0;
	endcase
endfunction

function automatic logic [11:0] single_rmw_cycles(input logic [2:0] mode);
	case (mode)
		3'd0: single_rmw_cycles = 12;
		3'd1, 3'd2: single_rmw_cycles = 21;
		3'd3: single_rmw_cycles = 27;
		3'd4: single_rmw_cycles = 24;
		3'd5, 3'd6: single_rmw_cycles = 30;
		default: single_rmw_cycles = 36;
	endcase
endfunction

function automatic logic [11:0] single_read_cycles(input logic [2:0] mode);
	case (mode)
		3'd0: single_read_cycles = 12;
		3'd1, 3'd2: single_read_cycles = 18;
		3'd3: single_read_cycles = 24;
		3'd4: single_read_cycles = 21;
		3'd5, 3'd6: single_read_cycles = 27;
		default: single_read_cycles = 33;
	endcase
endfunction

function automatic logic [11:0] mtps_cycles(input logic [2:0] mode);
	case (mode)
		3'd0: mtps_cycles = 24;
		3'd1, 3'd2: mtps_cycles = 30;
		3'd3: mtps_cycles = 36;
		3'd4: mtps_cycles = 33;
		3'd5, 3'd6: mtps_cycles = 39;
		default: mtps_cycles = 45;
	endcase
endfunction

function automatic logic [11:0] double_source_cycles(input logic [2:0] mode);
	case (mode)
		3'd0: double_source_cycles = 9;
		3'd1, 3'd2: double_source_cycles = 15;
		3'd3: double_source_cycles = 21;
		3'd4: double_source_cycles = 18;
		3'd5, 3'd6: double_source_cycles = 24;
		default: double_source_cycles = 30;
	endcase
endfunction

function automatic logic [11:0] double_rmw_dest_cycles(input logic [2:0] mode);
	case (mode)
		3'd0: double_rmw_dest_cycles = 3;
		3'd1, 3'd2: double_rmw_dest_cycles = 12;
		3'd3: double_rmw_dest_cycles = 18;
		3'd4: double_rmw_dest_cycles = 15;
		3'd5, 3'd6: double_rmw_dest_cycles = 21;
		default: double_rmw_dest_cycles = 27;
	endcase
endfunction

function automatic logic [11:0] double_read_dest_cycles(input logic [2:0] mode);
	case (mode)
		3'd0: double_read_dest_cycles = 3;
		3'd1, 3'd2: double_read_dest_cycles = 9;
		3'd3: double_read_dest_cycles = 15;
		3'd4: double_read_dest_cycles = 12;
		3'd5, 3'd6: double_read_dest_cycles = 18;
		default: double_read_dest_cycles = 24;
	endcase
endfunction

function automatic logic [11:0] jump_cycles(input logic [2:0] mode);
	case (mode)
		3'd1: jump_cycles = 15;
		3'd2, 3'd3, 3'd4: jump_cycles = 18;
		3'd5, 3'd6: jump_cycles = 21;
		3'd7: jump_cycles = 27;
		default: jump_cycles = 48;
	endcase
endfunction

function automatic logic [11:0] jsr_cycles(input logic [2:0] mode);
	case (mode)
		3'd1: jsr_cycles = 27;
		3'd2, 3'd3, 3'd4: jsr_cycles = 30;
		3'd5, 3'd6: jsr_cycles = 33;
		3'd7: jsr_cycles = 39;
		default: jsr_cycles = 48;
	endcase
endfunction

function automatic logic [11:0] instruction_nominal_cycles(input logic [15:0] opcode);
	logic [7:0] branch_opcode;
	logic [11:0] source_time;
	begin
		branch_opcode = opcode[15:8];
		instruction_nominal_cycles = 48;
		if ((branch_opcode >= 8'h01 && branch_opcode <= 8'h07) ||
		    (branch_opcode >= 8'h80 && branch_opcode <= 8'h87)) begin
			instruction_nominal_cycles = 12;
		end else if (is_double_opcode(opcode[15:12])) begin
			source_time = double_source_cycles(opcode[11:9]);
			if (opcode[15:12] == 4'h2 || opcode[15:12] == 4'h3 ||
			    opcode[15:12] == 4'ha || opcode[15:12] == 4'hb)
				instruction_nominal_cycles = source_time + double_read_dest_cycles(opcode[5:3]);
			else
				instruction_nominal_cycles = source_time + double_rmw_dest_cycles(opcode[5:3]);
		end else if ((opcode & 16'hfe00) == 16'h7800) begin
			instruction_nominal_cycles = single_rmw_cycles(opcode[5:3]);
		end else if ((opcode & 16'hfe00) == 16'h7e00) begin
			instruction_nominal_cycles = 18;
		end else if ((opcode & 16'hfe00) == 16'h0800) begin
			instruction_nominal_cycles = jsr_cycles(opcode[5:3]);
		end else if ((opcode & 16'hffc0) == 16'h0040) begin
			instruction_nominal_cycles = jump_cycles(opcode[5:3]);
		end else if ((opcode & 16'hfff8) == 16'h0080) begin
			instruction_nominal_cycles = 21;
		end else if ((opcode & 16'hfff0) == 16'h00a0 ||
		             (opcode & 16'hfff0) == 16'h00b0) begin
			instruction_nominal_cycles = 18;
		end else if ((opcode & 16'hffc0) == 16'h00c0) begin
			instruction_nominal_cycles = single_rmw_cycles(opcode[5:3]);
		end else if (is_single_opcode(opcode & 16'hffc0)) begin
			if ((opcode & 16'hffc0) == 16'h8d00)
				instruction_nominal_cycles = mtps_cycles(opcode[5:3]);
			else if ((opcode & 16'h7fc0) == 16'h0bc0)
				instruction_nominal_cycles = single_read_cycles(opcode[5:3]);
			else
				instruction_nominal_cycles = single_rmw_cycles(opcode[5:3]);
		end else if ((opcode & 16'hff00) == 16'h8800 ||
		             (opcode & 16'hff00) == 16'h8900) begin
			instruction_nominal_cycles = 48;
		end else begin
			case (opcode)
				16'h0000: instruction_nominal_cycles = 42;
				16'h0001: instruction_nominal_cycles = 12;
				16'h0002: instruction_nominal_cycles = 24;
				16'h0003, 16'h0004: instruction_nominal_cycles = 48;
				16'h0005: instruction_nominal_cycles = 110;
				16'h0006: instruction_nominal_cycles = 33;
				16'h0007: instruction_nominal_cycles = 15;
				default: instruction_nominal_cycles = 48;
			endcase
		end
	end
endfunction


always @* begin : execute_alu
	logic [16:0] wide;
	logic [15:0] source;
	logic [15:0] dest;
	logic [15:0] result;
	logic [15:0] mask;
	logic old_c;
	logic carry;
	logic overflow;

	source = operation_byte ? {8'h00, src_value[7:0]} : src_value;
	dest = operation_byte ? {8'h00, dst_value[7:0]} : dst_value;
	mask = operation_byte ? 16'h00ff : 16'hffff;
	old_c = psw[0];
	wide = 17'h00000;
	result = dest;
	carry = psw[0];
	overflow = psw[1];
	exec_psw = psw;
	exec_writes = 1'b1;

	case (operation)
		OP_MOV: begin result = source; overflow = 1'b0; end
		OP_CMP: begin
			wide = {1'b0, source} - {1'b0, dest}; result = wide[15:0];
			carry = operation_byte ? wide[8] : wide[16];
			overflow = operation_byte ? ((source[7] ^ dest[7]) & (source[7] ^ result[7]))
			                          : ((source[15] ^ dest[15]) & (source[15] ^ result[15]));
			exec_writes = 1'b0;
		end
		OP_BIT: begin result = dest & source; overflow = 1'b0; exec_writes = 1'b0; end
		OP_BIC: begin result = dest & ~source; overflow = 1'b0; end
		OP_BIS: begin result = dest | source; overflow = 1'b0; end
		OP_ADD: begin
			wide = {1'b0, dest} + {1'b0, source}; result = wide[15:0]; carry = wide[16];
			overflow = (~(dest[15] ^ source[15])) & (dest[15] ^ result[15]);
		end
		OP_SUB: begin
			wide = {1'b0, dest} - {1'b0, source}; result = wide[15:0]; carry = wide[16];
			overflow = (dest[15] ^ source[15]) & (dest[15] ^ result[15]);
		end
		OP_CLR: begin result = 16'h0000; carry = 1'b0; overflow = 1'b0; end
		OP_COM: begin result = ~dest; carry = 1'b1; overflow = 1'b0; end
		OP_INC: begin
			result = (dest + 1'b1) & mask; overflow = operation_byte ? (dest[7:0] == 8'h7f) : (dest == 16'h7fff);
		end
		OP_DEC: begin
			result = (dest - 1'b1) & mask; overflow = operation_byte ? (dest[7:0] == 8'h80) : (dest == 16'h8000);
		end
		OP_NEG: begin
			result = (-dest) & mask; carry = ((result & mask) != 0);
			overflow = operation_byte ? (dest[7:0] == 8'h80) : (dest == 16'h8000);
		end
		OP_ADC: begin
			wide = {1'b0, dest} + old_c; result = wide[15:0] & mask;
			carry = operation_byte ? wide[8] : wide[16];
			overflow = operation_byte ? (old_c && dest[7:0] == 8'h7f) : (old_c && dest == 16'h7fff);
		end
		OP_SBC: begin
			wide = {1'b0, dest} - old_c; result = wide[15:0] & mask;
			carry = operation_byte ? wide[8] : wide[16];
			overflow = operation_byte ? (old_c && dest[7:0] == 8'h80) : (old_c && dest == 16'h8000);
		end
		OP_TST: begin result = dest; carry = 1'b0; overflow = 1'b0; exec_writes = 1'b0; end
		OP_ROR: begin
			if (operation_byte) result = {8'h00, old_c, dest[7:1]};
			else                result = {old_c, dest[15:1]};
			carry = dest[0]; overflow = (operation_byte ? result[7] : result[15]) ^ carry;
		end
		OP_ROL: begin
			result = ((dest << 1) | {{15{1'b0}},old_c}) & mask;
			carry = operation_byte ? dest[7] : dest[15];
			overflow = (operation_byte ? result[7] : result[15]) ^ carry;
		end
		OP_ASR: begin
			if (operation_byte) result = {8'h00, dest[7], dest[7:1]};
			else                result = {dest[15], dest[15:1]};
			carry = dest[0]; overflow = (operation_byte ? result[7] : result[15]) ^ carry;
		end
		OP_ASL: begin
			result = (dest << 1) & mask; carry = operation_byte ? dest[7] : dest[15];
			overflow = (operation_byte ? result[7] : result[15]) ^ carry;
		end
		OP_SWAB: begin result = {dest[7:0], dest[15:8]}; carry = 1'b0; overflow = 1'b0; end
		OP_SXT: begin result = psw[3] ? 16'hffff : 16'h0000; overflow = 1'b0; end
		OP_XOR: begin result = dest ^ source; overflow = 1'b0; end
		OP_MFPS: begin result = {{8{psw[7]}}, psw}; overflow = 1'b0; end
		default: begin result = dest; exec_writes = 1'b0; end
	endcase

	if (operation == OP_CLR) begin
		exec_psw[3:0] = 4'b0100;
	end else if (operation == OP_SXT) begin
		exec_psw[2] = !psw[3];
		exec_psw[1] = 1'b0;
	end else if (operation == OP_SWAB) begin
		exec_psw[3] = result[7];
		exec_psw[2] = (result[7:0] == 0);
		exec_psw[1] = 1'b0;
		exec_psw[0] = 1'b0;
	end else begin
		exec_psw[3] = operation_byte ? result[7] : result[15];
		exec_psw[2] = operation_byte ? (result[7:0] == 0) : (result == 0);
		exec_psw[1] = overflow;
		if (operation == OP_CMP || operation == OP_ADD || operation == OP_SUB ||
		    operation == OP_COM || operation == OP_NEG || operation == OP_ADC ||
		    operation == OP_SBC || operation == OP_TST || operation == OP_ROR ||
		    operation == OP_ROL || operation == OP_ASR || operation == OP_ASL ||
		    operation == OP_SWAB)
			exec_psw[0] = carry;
	end
	exec_result = result;
end

always_ff @(posedge clk) begin : cpu_sequencer
	logic [2:0] mode;
	logic [2:0] regnum;
	logic [15:0] step;
	logic [15:0] address;
	logic actual_byte;
	logic [7:0] branch_opcode;

	if (reset) begin
		for (index = 0; index < 8; index = index + 1) regs[index] <= 16'h0000;
		regs[6] <= 16'h00fe;
		regs[7] <= mode_initial_pc(INITIAL_MODE[15:13]);
		psw <= 8'he0;
		state <= ST_BOUNDARY;
		bus_req <= 1'b0;
		bus_write <= 1'b0;
		bus_ifetch <= 1'b0;
		bus_addr <= 16'h0000;
		bus_wdata <= 16'h0000;
		bus_byte_en <= 2'b00;
		reset_out <= 1'b0;
		halted <= 1'b0;
		waiting <= 1'b0;
		retire <= 1'b0;
		trap_active <= 1'b0;
		halt_previous <= 1'b0;
		power_fail_previous <= 1'b0;
		halt_pending <= 1'b0;
		power_fail_pending <= 1'b0;
		trace_after_instruction <= 1'b0;
		trace_pending <= 1'b0;
		traced_instruction_trap <= 1'b0;
		rti_is_rtt <= 1'b0;
		timing_active <= 1'b0;
		instruction_cycles <= 12'd0;
		instruction_wait_cycles <= 12'd0;
		nominal_cycles <= 12'd0;
	end else begin
		halt_previous <= halt;
		power_fail_previous <= power_fail;
		if (halt && !halt_previous) halt_pending <= 1'b1;
		if (power_fail && !power_fail_previous) power_fail_pending <= 1'b1;

		if (ce) begin
			retire <= 1'b0;
			reset_out <= 1'b0;
			if (timing_active) instruction_cycles <= instruction_cycles + 12'd1;

			case (state)
				ST_BOUNDARY: begin
					if (halt_pending) begin
						halt_pending <= 1'b0; trap_halt <= 1'b1; trap_hardware <= 1'b0;
						trap_active <= 1'b1; state <= ST_TRAP_PUSH_PSW_REQ; waiting <= 1'b0;
						trace_after_instruction <= 1'b0;
					end else if (trace_pending) begin
						trace_pending <= 1'b0; trace_after_instruction <= 1'b0;
						trap_vector <= 8'h0c; trap_halt <= 1'b0; trap_hardware <= 1'b0;
						trap_active <= 1'b1; state <= ST_TRAP_PUSH_PSW_REQ; waiting <= 1'b0;
					end else if (psw[4] && !waiting && !halted) begin
						// A restored T bit blocks lower-priority interrupts until one instruction retires.
						trace_after_instruction <= psw[4];
						bus_req <= 1'b1; bus_write <= 1'b0; bus_ifetch <= 1'b1;
						bus_addr <= {regs[7][15:1], 1'b0}; bus_byte_en <= 2'b11;
						bus_return <= ST_FETCH_DONE; state <= ST_BUS;
					end else if (power_fail_pending) begin
						power_fail_pending <= 1'b0; trap_vector <= 8'h14; trap_halt <= 1'b0;
						trap_hardware <= 1'b1; trap_active <= 1'b1; state <= ST_TRAP_VECTOR_PC_REQ;
						waiting <= 1'b0;
					end else if ((cp_priority(cp) > psw[7:5]) && (cp != 0)) begin
						trap_vector <= cp_vector(cp); trap_halt <= 1'b0; trap_hardware <= 1'b1;
						trap_active <= 1'b1; state <= ST_TRAP_VECTOR_PC_REQ; waiting <= 1'b0;
					end else if (!waiting && !halted) begin
						trace_after_instruction <= psw[4];
						bus_req <= 1'b1; bus_write <= 1'b0; bus_ifetch <= 1'b1;
						bus_addr <= {regs[7][15:1], 1'b0}; bus_byte_en <= 2'b11;
						bus_return <= ST_FETCH_DONE; state <= ST_BUS;
					end
				end

				ST_FETCH_REQ: begin
					trace_after_instruction <= psw[4];
					bus_req <= 1'b1; bus_write <= 1'b0; bus_ifetch <= 1'b1;
					bus_addr <= {regs[7][15:1], 1'b0}; bus_byte_en <= 2'b11;
					bus_return <= ST_FETCH_DONE; state <= ST_BUS;
				end

				ST_BUS: begin
					if (bus_ack) begin
						bus_req <= 1'b0; bus_data <= bus_rdata;
						if (bus_ifetch) begin
							timing_active <= 1'b1;
							instruction_cycles <= 12'd0;
							instruction_wait_cycles <= 12'd0;
						end
						if (bus_error) begin
							if (trap_active) begin halted <= 1'b1; state <= ST_BOUNDARY; end
							else begin
								trap_vector <= 8'h04; trap_halt <= 1'b0; trap_hardware <= 1'b0;
								trap_active <= 1'b1; state <= ST_TRAP_PUSH_PSW_REQ;
							end
						end else begin
							state <= bus_return;
						end
					end else if (timing_active) begin
						instruction_wait_cycles <= instruction_wait_cycles + 12'd1;
					end
				end

				ST_FETCH_DONE: begin
					instruction <= bus_data; regs[7] <= regs[7] + 16'd2;
					nominal_cycles <= instruction_nominal_cycles(bus_data);
					state <= ST_DECODE;
				end

				ST_DECODE: begin
					branch_opcode = instruction[15:8];
					if ((branch_opcode >= 8'h01 && branch_opcode <= 8'h07) ||
					    (branch_opcode >= 8'h80 && branch_opcode <= 8'h87)) begin
						if (branch_taken(branch_opcode, psw[3:0]))
							regs[7] <= regs[7] + {{7{instruction[7]}}, instruction[7:0], 1'b0};
						state <= ST_FINISH;
					end else if (is_double_opcode(instruction[15:12])) begin
						operation_byte <= instruction[15] && (instruction[15:12] != 4'he);
						case (instruction[15:12])
							4'h1,4'h9: operation <= OP_MOV;
							4'h2,4'ha: operation <= OP_CMP;
							4'h3,4'hb: operation <= OP_BIT;
							4'h4,4'hc: operation <= OP_BIC;
							4'h5,4'hd: operation <= OP_BIS;
							4'h6: operation <= OP_ADD;
							default: operation <= OP_SUB;
						endcase
						resolve_spec <= instruction[11:6];
						resolve_byte <= instruction[15] && (instruction[15:12] != 4'he);
						resolve_need_value <= 1'b1; resolve_pc_byte_word <= 1'b1;
						resolve_return <= ST_AFTER_SRC; state <= ST_RESOLVE_START;
					end else if ((instruction & 16'hfe00) == 16'h7800) begin
						operation <= OP_XOR; operation_byte <= 1'b0; src_value <= regs[instruction[8:6]];
						resolve_spec <= instruction[5:0]; resolve_byte <= 1'b0;
						resolve_need_value <= 1'b1; resolve_pc_byte_word <= 1'b0;
						resolve_return <= ST_AFTER_DST; state <= ST_RESOLVE_START;
					end else if ((instruction & 16'hfe00) == 16'h7e00) begin
						regs[instruction[8:6]] <= regs[instruction[8:6]] - 16'd1;
						if (regs[instruction[8:6]] != 16'd1)
							regs[7] <= regs[7] - {9'd0, instruction[5:0], 1'b0};
						state <= ST_FINISH;
					end else if ((instruction & 16'hfe00) == 16'h0800) begin
						if (instruction[5:3] == 0) begin
							if (trace_after_instruction) traced_instruction_trap <= 1'b1;
							trace_after_instruction <= 1'b0;
							trap_vector <= 8'h08; trap_halt <= 1'b0; trap_hardware <= 1'b0;
							trap_active <= 1'b1; state <= ST_TRAP_PUSH_PSW_REQ;
						end else begin
							resolve_spec <= instruction[5:0]; resolve_byte <= 1'b0;
							resolve_need_value <= 1'b0; resolve_pc_byte_word <= 1'b0;
							resolve_return <= ST_JSR_DONE; state <= ST_RESOLVE_START;
						end
					end else if ((instruction & 16'hffc0) == 16'h0040) begin
						if (instruction[5:3] == 0) begin
							if (trace_after_instruction) traced_instruction_trap <= 1'b1;
							trace_after_instruction <= 1'b0;
							trap_vector <= 8'h08; trap_halt <= 1'b0; trap_hardware <= 1'b0;
							trap_active <= 1'b1; state <= ST_TRAP_PUSH_PSW_REQ;
						end else begin
							resolve_spec <= instruction[5:0]; resolve_byte <= 1'b0;
							resolve_need_value <= 1'b0; resolve_pc_byte_word <= 1'b0;
							resolve_return <= ST_JMP_DONE; state <= ST_RESOLVE_START;
						end
					end else if ((instruction & 16'hfff8) == 16'h0080) begin
						regs[7] <= regs[instruction[2:0]];
						bus_req <= 1'b1; bus_write <= 1'b0; bus_ifetch <= 1'b0;
						bus_addr <= {regs[6][15:1],1'b0}; bus_byte_en <= 2'b11;
						bus_return <= ST_RTS_POP_DONE; state <= ST_BUS;
					end else if ((instruction & 16'hfff0) == 16'h00a0) begin
						psw[3:0] <= psw[3:0] & ~instruction[3:0]; state <= ST_FINISH;
					end else if ((instruction & 16'hfff0) == 16'h00b0) begin
						psw[3:0] <= psw[3:0] | instruction[3:0]; state <= ST_FINISH;
					end else if ((instruction & 16'hffc0) == 16'h00c0) begin
						operation <= OP_SWAB; operation_byte <= 1'b0;
						resolve_spec <= instruction[5:0]; resolve_byte <= 1'b0;
						resolve_need_value <= 1'b1; resolve_pc_byte_word <= 1'b0;
						resolve_return <= ST_AFTER_DST; state <= ST_RESOLVE_START;
					end else if (is_single_opcode(instruction & 16'hffc0)) begin
						operation_byte <= instruction[15];
						case (instruction & 16'h7fc0)
							16'h0a00: operation <= OP_CLR;
							16'h0a40: operation <= OP_COM;
							16'h0a80: operation <= OP_INC;
							16'h0ac0: operation <= OP_DEC;
							16'h0b00: operation <= OP_NEG;
							16'h0b40: operation <= OP_ADC;
							16'h0b80: operation <= OP_SBC;
							16'h0bc0: operation <= OP_TST;
							16'h0c00: operation <= OP_ROR;
							16'h0c40: operation <= OP_ROL;
							16'h0c80: operation <= OP_ASR;
							16'h0cc0: operation <= OP_ASL;
							16'h0dc0: operation <= instruction[15] ? OP_MFPS : OP_SXT;
							default: operation <= OP_NONE;
						endcase
						if ((instruction & 16'h7fc0) == 16'h0d00) begin
							// MTPS is the only implemented 1064xx instruction on T-11.
							resolve_spec <= instruction[5:0]; resolve_byte <= 1'b1;
							resolve_need_value <= 1'b1; resolve_pc_byte_word <= 1'b0;
							resolve_return <= ST_AFTER_DST; operation <= OP_NONE;
							state <= ST_RESOLVE_START;
						end else begin
							resolve_spec <= instruction[5:0]; resolve_byte <= instruction[15];
							resolve_need_value <= ((instruction & 16'h7fc0) != 16'h0dc0) || !instruction[15];
							resolve_pc_byte_word <= 1'b0; resolve_return <= ST_AFTER_DST;
							state <= ST_RESOLVE_START;
						end
					end else if ((instruction & 16'hff00) == 16'h8800) begin
						if (trace_after_instruction) traced_instruction_trap <= 1'b1;
						trace_after_instruction <= 1'b0;
						trap_vector <= 8'h18; trap_halt <= 1'b0; trap_hardware <= 1'b0;
						trap_active <= 1'b1; state <= ST_TRAP_PUSH_PSW_REQ;
					end else if ((instruction & 16'hff00) == 16'h8900) begin
						if (trace_after_instruction) traced_instruction_trap <= 1'b1;
						trace_after_instruction <= 1'b0;
						trap_vector <= 8'h1c; trap_halt <= 1'b0; trap_hardware <= 1'b0;
						trap_active <= 1'b1; state <= ST_TRAP_PUSH_PSW_REQ;
					end else if (instruction == 16'h0000) begin
						trace_after_instruction <= 1'b0;
						trap_halt <= 1'b1; trap_hardware <= 1'b0; trap_active <= 1'b1;
						state <= ST_TRAP_PUSH_PSW_REQ;
					end else if (instruction == 16'h0001) begin
						waiting <= 1'b1; state <= ST_FINISH;
					end else if (instruction == 16'h0002 || instruction == 16'h0006) begin
						rti_is_rtt <= (instruction == 16'h0006);
						bus_req <= 1'b1; bus_write <= 1'b0; bus_ifetch <= 1'b0;
						bus_addr <= {regs[6][15:1],1'b0}; bus_byte_en <= 2'b11;
						bus_return <= ST_RTI_PC_DONE; state <= ST_BUS;
					end else if (instruction == 16'h0003) begin
						if (trace_after_instruction) traced_instruction_trap <= 1'b1;
						trace_after_instruction <= 1'b0;
						trap_vector <= 8'h0c; trap_halt <= 1'b0; trap_hardware <= 1'b0;
						trap_active <= 1'b1; state <= ST_TRAP_PUSH_PSW_REQ;
					end else if (instruction == 16'h0004) begin
						if (trace_after_instruction) traced_instruction_trap <= 1'b1;
						trace_after_instruction <= 1'b0;
						trap_vector <= 8'h10; trap_halt <= 1'b0; trap_hardware <= 1'b0;
						trap_active <= 1'b1; state <= ST_TRAP_PUSH_PSW_REQ;
					end else if (instruction == 16'h0005) begin
						reset_out <= 1'b1; state <= ST_FINISH;
					end else if (instruction == 16'h0007) begin
						regs[0] <= 16'd4; state <= ST_FINISH;
					end else begin
						if (trace_after_instruction) traced_instruction_trap <= 1'b1;
						trace_after_instruction <= 1'b0;
						trap_vector <= 8'h08; trap_halt <= 1'b0; trap_hardware <= 1'b0;
						trap_active <= 1'b1; state <= ST_TRAP_PUSH_PSW_REQ;
					end
				end

				ST_RESOLVE_START: begin
					mode = resolve_spec[5:3]; regnum = resolve_spec[2:0];
					resolve_reg <= regnum; resolve_is_reg <= (mode == 0);
					case (mode)
						3'd0: begin resolve_ea <= 0; resolve_value <= regs[regnum]; state <= ST_RESOLVE_RETURN; end
						3'd1: begin resolve_ea <= regs[regnum]; state <= resolve_need_value ? ST_RESOLVE_VALUE_DONE : ST_RESOLVE_RETURN;
							if (resolve_need_value) begin
								actual_byte = resolve_byte; bus_req <= 1'b1; bus_write <= 1'b0; bus_ifetch <= 1'b0;
								bus_addr <= actual_byte ? regs[regnum] : {regs[regnum][15:1],1'b0};
								bus_byte_en <= actual_byte ? (regs[regnum][0] ? 2'b10 : 2'b01) : 2'b11;
								bus_return <= ST_RESOLVE_VALUE_DONE; state <= ST_BUS;
							end
						end
						3'd2: begin
							resolve_ea <= regs[regnum]; step = (resolve_byte && regnum < 6) ? 16'd1 : 16'd2;
							regs[regnum] <= regs[regnum] + step;
							if (resolve_need_value) begin
								actual_byte = resolve_byte && !(resolve_pc_byte_word && regnum == 7);
								bus_req <= 1'b1; bus_write <= 1'b0; bus_ifetch <= 1'b0;
								bus_addr <= actual_byte ? regs[regnum] : {regs[regnum][15:1],1'b0};
								bus_byte_en <= actual_byte ? (regs[regnum][0] ? 2'b10 : 2'b01) : 2'b11;
								bus_return <= ST_RESOLVE_VALUE_DONE; state <= ST_BUS;
							end else state <= ST_RESOLVE_RETURN;
						end
						3'd3: begin
							resolve_base <= regs[regnum]; regs[regnum] <= regs[regnum] + 16'd2;
							bus_req <= 1'b1; bus_write <= 1'b0; bus_ifetch <= 1'b0;
							bus_addr <= {regs[regnum][15:1],1'b0}; bus_byte_en <= 2'b11;
							bus_return <= ST_RESOLVE_PTR_DONE; state <= ST_BUS;
						end
						3'd4: begin
							step = (resolve_byte && regnum < 6) ? 16'd1 : 16'd2; address = regs[regnum] - step;
							regs[regnum] <= address; resolve_ea <= address;
							if (resolve_need_value) begin
								bus_req <= 1'b1; bus_write <= 1'b0; bus_ifetch <= 1'b0;
								bus_addr <= resolve_byte ? address : {address[15:1],1'b0};
								bus_byte_en <= resolve_byte ? (address[0] ? 2'b10 : 2'b01) : 2'b11;
								bus_return <= ST_RESOLVE_VALUE_DONE; state <= ST_BUS;
							end else state <= ST_RESOLVE_RETURN;
						end
						3'd5: begin
							address = regs[regnum] - 16'd2; regs[regnum] <= address; resolve_base <= address;
							bus_req <= 1'b1; bus_write <= 1'b0; bus_ifetch <= 1'b0;
							bus_addr <= {address[15:1],1'b0}; bus_byte_en <= 2'b11;
							bus_return <= ST_RESOLVE_PTR_DONE; state <= ST_BUS;
						end
						default: begin
							resolve_base <= (regnum == 7) ? regs[7] + 16'd2 : regs[regnum];
							bus_req <= 1'b1; bus_write <= 1'b0; bus_ifetch <= 1'b0;
							bus_addr <= {regs[7][15:1],1'b0}; bus_byte_en <= 2'b11;
							bus_return <= ST_RESOLVE_EXT_DONE; state <= ST_BUS;
						end
					endcase
				end

				ST_RESOLVE_EXT_DONE: begin
					regs[7] <= regs[7] + 16'd2; address = resolve_base + bus_data;
					if (resolve_spec[5:3] == 3'd7) begin
						resolve_base <= address; bus_req <= 1'b1; bus_write <= 1'b0; bus_ifetch <= 1'b0;
						bus_addr <= {address[15:1],1'b0}; bus_byte_en <= 2'b11;
						bus_return <= ST_RESOLVE_PTR_DONE; state <= ST_BUS;
					end else begin
						resolve_ea <= address;
						if (resolve_need_value) begin
							bus_req <= 1'b1; bus_write <= 1'b0; bus_ifetch <= 1'b0;
							bus_addr <= resolve_byte ? address : {address[15:1],1'b0};
							bus_byte_en <= resolve_byte ? (address[0] ? 2'b10 : 2'b01) : 2'b11;
							bus_return <= ST_RESOLVE_VALUE_DONE; state <= ST_BUS;
						end else state <= ST_RESOLVE_RETURN;
					end
				end

				ST_RESOLVE_PTR_DONE: begin
					resolve_ea <= bus_data; resolve_is_reg <= 1'b0;
					if (resolve_need_value) begin
						bus_req <= 1'b1; bus_write <= 1'b0; bus_ifetch <= 1'b0;
						bus_addr <= resolve_byte ? bus_data : {bus_data[15:1],1'b0};
						bus_byte_en <= resolve_byte ? (bus_data[0] ? 2'b10 : 2'b01) : 2'b11;
						bus_return <= ST_RESOLVE_VALUE_DONE; state <= ST_BUS;
					end else state <= ST_RESOLVE_RETURN;
				end

				ST_RESOLVE_VALUE_DONE: begin
					if (resolve_byte)
						resolve_value <= resolve_ea[0] ? {8'h00,bus_data[15:8]} : {8'h00,bus_data[7:0]};
					else resolve_value <= bus_data;
					state <= ST_RESOLVE_RETURN;
				end

				ST_RESOLVE_RETURN: state <= resolve_return;

				ST_AFTER_SRC: begin
					src_value <= resolve_value;
					resolve_spec <= instruction[5:0]; resolve_byte <= operation_byte;
					resolve_need_value <= 1'b1; resolve_pc_byte_word <= 1'b0;
					resolve_return <= ST_AFTER_DST; state <= ST_RESOLVE_START;
				end

				ST_AFTER_DST: begin
					dst_value <= resolve_value; dst_is_reg <= resolve_is_reg;
					dst_reg <= resolve_reg; dst_ea <= resolve_ea;
					if (operation == OP_NONE && (instruction & 16'hffc0) == 16'h8d00) begin
						psw <= (psw & 8'h10) | (resolve_value[7:0] & 8'hef); state <= ST_FINISH;
					end else state <= ST_EXECUTE;
				end

				ST_EXECUTE: begin
					psw <= exec_psw;
					if (!exec_writes) state <= ST_FINISH;
					else if (dst_is_reg) begin
						if (operation_byte) begin
							if (operation == OP_MOV || operation == OP_MFPS)
								regs[dst_reg] <= {{8{exec_result[7]}},exec_result[7:0]};
							else regs[dst_reg] <= {regs[dst_reg][15:8],exec_result[7:0]};
						end else regs[dst_reg] <= exec_result;
						state <= ST_FINISH;
					end else begin
						bus_req <= 1'b1; bus_write <= 1'b1; bus_ifetch <= 1'b0;
						bus_addr <= operation_byte ? dst_ea : {dst_ea[15:1],1'b0};
						bus_byte_en <= operation_byte ? (dst_ea[0] ? 2'b10 : 2'b01) : 2'b11;
						bus_wdata <= operation_byte ? (dst_ea[0] ? {exec_result[7:0],8'h00} : {8'h00,exec_result[7:0]}) : exec_result;
						bus_return <= ST_WRITE_DONE; state <= ST_BUS;
					end
				end

				ST_WRITE_DONE: state <= ST_FINISH;
				ST_JMP_DONE: begin regs[7] <= resolve_ea; state <= ST_FINISH; end

				ST_JSR_DONE: begin
					bus_req <= 1'b1; bus_write <= 1'b1; bus_ifetch <= 1'b0;
					bus_addr <= (regs[6] - 16'd2) & 16'hfffe; bus_byte_en <= 2'b11;
					bus_wdata <= regs[instruction[8:6]]; regs[6] <= regs[6] - 16'd2;
					bus_return <= ST_JSR_PUSH_DONE; state <= ST_BUS;
				end
				ST_JSR_PUSH_DONE: begin
					regs[instruction[8:6]] <= regs[7]; regs[7] <= resolve_ea; state <= ST_FINISH;
				end

				ST_RTS_POP_DONE: begin
					regs[instruction[2:0]] <= bus_data; regs[6] <= regs[6] + 16'd2; state <= ST_FINISH;
				end

				ST_RTI_PC_DONE: begin
					regs[7] <= bus_data; regs[6] <= regs[6] + 16'd2;
					bus_req <= 1'b1; bus_write <= 1'b0; bus_ifetch <= 1'b0;
					bus_addr <= (regs[6] + 16'd2) & 16'hfffe; bus_byte_en <= 2'b11;
					bus_return <= ST_RTI_PSW_DONE; state <= ST_BUS;
				end
				ST_RTI_PSW_DONE: begin
					psw <= bus_data[7:0]; regs[6] <= regs[6] + 16'd2;
					if (rti_is_rtt || traced_instruction_trap) begin
						trace_pending <= 1'b0;
						traced_instruction_trap <= 1'b0;
					end else if (bus_data[4]) begin
						trace_pending <= 1'b1;
					end
					state <= ST_FINISH;
				end

				ST_TRAP_VECTOR_PC_REQ: begin
					bus_req <= 1'b1; bus_write <= 1'b0; bus_ifetch <= 1'b0;
					bus_addr <= {8'h00,trap_vector}; bus_byte_en <= 2'b11;
					bus_return <= ST_TRAP_VECTOR_PC_DONE; state <= ST_BUS;
				end
				ST_TRAP_VECTOR_PC_DONE: begin trap_new_pc <= bus_data; state <= ST_TRAP_VECTOR_PSW_REQ; end
				ST_TRAP_VECTOR_PSW_REQ: begin
					bus_req <= 1'b1; bus_write <= 1'b0; bus_ifetch <= 1'b0;
					bus_addr <= {8'h00,trap_vector} + 16'd2; bus_byte_en <= 2'b11;
					bus_return <= ST_TRAP_VECTOR_PSW_DONE; state <= ST_BUS;
				end
				ST_TRAP_VECTOR_PSW_DONE: begin
					trap_new_psw <= bus_data[7:0];
					if (trap_hardware) begin
						state <= ST_TRAP_PUSH_PSW_REQ;
					end else begin
						regs[7] <= trap_new_pc; psw <= bus_data[7:0]; waiting <= 1'b0;
						trap_active <= 1'b0; state <= ST_FINISH;
					end
				end

				ST_TRAP_PUSH_PSW_REQ: begin
					bus_req <= 1'b1; bus_write <= 1'b1; bus_ifetch <= 1'b0;
					bus_addr <= (regs[6] - 16'd2) & 16'hfffe; bus_byte_en <= 2'b11;
					bus_wdata <= {8'h00,psw}; regs[6] <= regs[6] - 16'd2;
					bus_return <= ST_TRAP_PUSH_PSW_DONE; state <= ST_BUS;
				end
				ST_TRAP_PUSH_PSW_DONE: state <= ST_TRAP_PUSH_PC_REQ;
				ST_TRAP_PUSH_PC_REQ: begin
					bus_req <= 1'b1; bus_write <= 1'b1; bus_ifetch <= 1'b0;
					bus_addr <= (regs[6] - 16'd2) & 16'hfffe; bus_byte_en <= 2'b11;
					bus_wdata <= regs[7]; regs[6] <= regs[6] - 16'd2;
					bus_return <= ST_TRAP_PUSH_PC_DONE; state <= ST_BUS;
				end
				ST_TRAP_PUSH_PC_DONE: begin
					if (trap_halt) begin
						regs[7] <= mode_initial_pc(INITIAL_MODE[15:13]) + 16'd4; psw <= 8'he0;
						trap_active <= 1'b0; waiting <= 1'b0; state <= ST_FINISH;
					end else if (trap_hardware) begin
						regs[7] <= trap_new_pc; psw <= trap_new_psw; trap_active <= 1'b0;
						waiting <= 1'b0; state <= ST_BOUNDARY;
					end else begin
						state <= ST_TRAP_VECTOR_PC_REQ;
					end
				end

				ST_FINISH: begin
					if (!timing_active ||
					    (instruction_cycles + 12'd3 >= nominal_cycles + instruction_wait_cycles)) begin
						retire <= 1'b1; trap_active <= 1'b0; timing_active <= 1'b0;
						if (trace_after_instruction) trace_pending <= 1'b1;
						trace_after_instruction <= 1'b0;
						state <= ST_BOUNDARY;
					end
				end
				default: state <= ST_BOUNDARY;
			endcase
		end
	end
end

endmodule
