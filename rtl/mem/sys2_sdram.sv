// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Single-port SDR-SDRAM controller for the MiSTer 32MB module (MT48LC16M16 class:
// 4 banks x 13-bit row x 9-bit col x16). One 16-bit word per access, read/write with
// auto-precharge, CL2. Blocking host contract: when `ready` is high, pulse `req` with
// `addr`/`we`(/`wdata`); `ready` drops during the access and returns high when done,
// read data on `rdata` with a one-cycle `valid` pulse. AUTO_REFRESH interleaves while
// idle.
//
// SystemVerilog re-implementation of the proven reference controller
// (Arcade-Atari-system1 rtl/lib/mem/sdram.vhd) -- same command/mode encoding -- so the
// iverilog/verilator flow (which can't run the VHDL original) exercises the identical
// protocol against a behavioral model. Structured as single-cycle command states +
// explicit wait states so each step is obviously correct. All SDRAM outputs go through
// one registered pin stage (see the note at the stage) -- combinational
// command/address pins were the root cause of the family build's tile corruption: the
// burst engine changes the address every cycle, and the chip latched the wrong column.
// The SDRAM_CLK pin is not driven from this module on the board: the top level leaves the
// SDRAM_CLK output unconnected and wires the pin to PLL outclk_2, a phase-controlled 32 MHz
// clock (rise at 15.625 ns of the 31.250 ns period, -180 deg; it was the inverted controller
// clock before that). The module's own `assign SDRAM_CLK = ~clk` keeps that same -180 deg
// relationship for benches that clock their SDRAM model from this output.
// Do not retune that phase to fix read timing. It was moved to 26.875 ns to
// close the dq_in_r capture slot and it corrupted SDRAM on hardware, because a later SDRAM_CLK
// spends the chip's own command/address hold (12.4 -> 1.8 ns) against a `set_output_delay` that
// models no board skew. Reverted the same day. The capture slot is closed from the OTHER end
// instead -- see the negedge note at dq_in_r below -- which leaves this phase, and therefore the
// whole command/address side, untouched. See rtl/pll/pll_0002.v and atarisys2.sdc.
module sys2_sdram #(
	parameter int CLK_MHZ  = 32,    // controller clock in MHz (integer)
	parameter int ROW_BITS = 13,
	parameter int COL_BITS = 9
) (
	input  logic        clk,
	input  logic        reset,

	// Host word port. addr is a 16-bit-word address: {BA[1:0], row, col}.
	input  logic [ROW_BITS+COL_BITS+1:0] addr,
	input  logic [15:0] wdata,
	input  logic        we,
	input  logic        req,
	output logic [15:0] rdata,
	// `valid` is the SHARED return strobe: it pulses for single-word reads AND for every
	// burst word. A consumer that watches only this cannot tell whose data just arrived.
	// Use brst_valid below if you only want burst words -- see the warning there.
	output logic        valid,
	output logic        ready,

	// ---- Burst read port -------------------------------------------------
	// Pulse brst_req while `ready` with a start address and a word count. Words come
	// back on the same rdata/valid pair, one valid pulse per word, strictly in order.
	// The row is held OPEN across the burst and READ commands are issued back to back,
	// so after the ACTIVE+tRCD overhead the pipe delivers one word per clock.
	//
	// brst_abort is a LEVEL, sampled every issue cycle: raising it stops further READs at a
	// word boundary. Reads already in flight (up to CL) still return their data, so the
	// consumer must accept up to CL more valids after asserting abort. That is the
	// preemption granularity the arbiter needs -- a burst never has to be waited out.
	//
	// Burst-only return strobe. `valid` pulses for BOTH paths -- this burst return pipeline and
	// the single-word S_CL wait -- because they share one rdata bus, so `valid` alone cannot tell
	// a burst consumer whose data just arrived.
	//
	// Hardening, not a bug fix. It is tempting to conclude that a foreign single-word client
	// (the top level's sprite ROM, wired straight to this port past sys2_sdram_arb) can therefore
	// have its word miscounted as a burst word. MEASURED, and it does not happen: the single-word
	// path is entered only from S_IDLE, which a burst has to have retired to reach, and the
	// arbiter releases `owner` on that retirement. Simulation instruments the exact coincidence
	// -- 50,287 single-word returns, `owner` was OWN_NONE at every one, and the pre-split RTL
	// passes the contention run unchanged.
	//
	// The split is kept because a burst counter has no business counting non-burst strobes, and
	// the invariant it relies on (S_IDLE ordering) is not local to either module. It changes no
	// behaviour in any build.
	output logic        brst_valid,   // one pulse per BURST word only
	//
	// Row/bank changes mid-burst (a burst straddling a row boundary) are handled by
	// precharging and re-activating; callers do not have to align or split requests.
	input  logic        brst_req,
	input  logic [ROW_BITS+COL_BITS+1:0] brst_addr,
	input  logic [8:0]  brst_len,     // words, 1..256 (0 is treated as 1)
	input  logic        brst_abort,
	output logic        brst_busy,
	output logic        brst_done,    // one-cycle pulse when the last word has been returned
	// High when a burst request can be latched right now, including while another burst is
	// still running. The arbiter uses it to hand over the next burst before the current one
	// retires, which is what removes the dead cycles between bursts -- see the
	// back-to-back chaining in S_BTRP below.
	output logic        brst_accept,
	// One-cycle pulse on the edge a burst actually BEGINS. The arbiter hands ownership of
	// returning data over on this, not on brst_done: with chaining, done for the old burst
	// and start for the new one land on the same edge, and without a separate start the
	// arbiter cannot tell a chained handover from a plain retirement -- which would
	// misroute a CPU read that slipped in between.
	output logic        brst_start,

	inout  logic [15:0] SDRAM_DQ,
	output logic [12:0] SDRAM_A,
	output logic [1:0]  SDRAM_BA,
	output logic        SDRAM_DQML,
	output logic        SDRAM_DQMH,
	output logic        SDRAM_CLK,
	output logic        SDRAM_CKE,
	output logic        SDRAM_nCS,
	output logic        SDRAM_nRAS,
	output logic        SDRAM_nCAS,
	output logic        SDRAM_nWE
);

// integer ceil-division: ceil(ns*MHz/1000) = (ns*MHz + 999)/1000 (synthesis-safe).
localparam int tINIT = (200000*CLK_MHZ + 999) / 1000;
localparam int tRC   = (    60*CLK_MHZ + 999) / 1000;
// 18 ns tRCD/tRP. The distortion this core chased for weeks turned out to be the DQ capture
// EDGE, not the command/address path -- see the capture note in atarisys2.sdc -- so these are
// the datasheet numbers with no padding.
localparam int tRCD_B = (    18*CLK_MHZ + 999) / 1000;
localparam int tRP_B  = (    18*CLK_MHZ + 999) / 1000;
wire [15:0] tRCD = 16'(tRCD_B);
wire [15:0] tRP  = 16'(tRP_B);
localparam int tMRD  = (    12*CLK_MHZ + 999) / 1000;
localparam int tREF  = (  7800*CLK_MHZ + 999) / 1000;
localparam int CL    = 2;

localparam logic [3:0] CMD_LMR=4'b0000, CMD_REFRESH=4'b0001, CMD_PRECHARGE=4'b0010,
                       CMD_ACTIVE=4'b0011, CMD_WRITE=4'b0100, CMD_READ=4'b0101,
                       CMD_NOP=4'b0111;
localparam logic [12:0] MODE_REG = {3'b000, 1'b1, 2'b00, 3'b010, 1'b0, 3'b000};

localparam int AW     = ROW_BITS + COL_BITS + 2;
localparam int BA_HI  = AW-1,  BA_LO  = AW-2;
localparam int ROW_HI = COL_BITS + ROW_BITS - 1, ROW_LO = COL_BITS;

typedef enum logic [4:0] {
	S_INIT, S_PRE, S_TRP_I, S_REFI, S_TRC_I, S_MRD, S_TMRD,
	S_IDLE, S_ACT, S_TRCD, S_RD, S_CL, S_WR, S_RECOV, S_REF, S_TRC,
	// Burst read: activate, wait tRCD, stream READs, drain the CL pipe, precharge.
	S_BACT, S_BTRCD, S_BRD, S_BTAIL, S_BPRE, S_BTRP
} state_t;
state_t state = S_INIT;

logic [15:0] dly      = '0;
logic [3:0]  ref_init = '0;
logic [15:0] rfsh_ctr = '0;
logic        rfsh_req = 1'b0;

logic           we_l;
logic [15:0]    wdata_l;
logic [AW-1:0]  addr_l;
logic           pending = 1'b0;   // a captured request awaiting service

// ---- combinational command / address / DQ-oe from the single-cycle states -------
logic [3:0]  cmd;
logic [12:0] a_comb;
logic [1:0]  ba_comb;
logic        dq_oe;

// Column address on A with A10 set = read/write with auto-precharge. COL_BITS<=9 so
// the column sits in A[8:0] and A10 is free for the auto-precharge flag. Built as one
// concatenation (iverilog mishandles partial bit-selects inside always_comb).
wire [12:0] col_a = {2'b00, 1'b1, {(10-COL_BITS){1'b0}}, addr_l[COL_BITS-1:0]};
localparam logic [12:0] PRE_ALL = 13'h0400; // A10=1 = precharge all banks

// Pre-extract the address fields as continuous-assign wires; iverilog mishandles
// constant part-selects inside always_* blocks (drives "all bits").
wire [1:0]           ba_w  = addr_l[BA_HI:BA_LO];
wire [ROW_BITS-1:0]  row_w = addr_l[ROW_HI:ROW_LO];
wire [12:0]          row_a = {{(13-ROW_BITS){1'b0}}, row_w};

// ---- burst state ------------------------------------------------------------------
logic [AW-1:0] b_addr;        // next word address to issue
logic [9:0]    b_left;        // words still to issue
logic [9:0]    b_inflight;    // READs issued whose data has not come back yet
// Controller-visible read latency in clk cycles: the cycle that drives the READ command
// to the posedge at which its data can be registered. This is exactly CL, the same
// sample point the proven single-word path reaches via S_RD -> S_CL -> sample.
//
// It is verified against a simulation chip model clocked on clk, which is how every
// simulation here wires it. Clock that model on ~clk instead and reads appear to arrive a
// cycle earlier -- an artefact of the bench, not of the design.
// CL chip cycles + 1 for the registered output stage (the READ reaches the chip one
// cycle after the issuing state drives it). See the pin-stage note below.
localparam int RD_LAT = CL + 1;
// How many playfield columns may chain on one open row before the single-word client is let in.
// Bounds the sprite ROM's extra wait to ~6 clk per chained column; see the note in S_BTAIL.
localparam logic [2:0] CHAIN_MAX = 3'd4;   // sized: an int here is a 32-bit compare (WIDTHEXPAND)
logic [2:0] b_chain_cnt = '0;

logic [RD_LAT-1:0] b_pipe;    // one bit per cycle of read latency; bit 0 = data due now
logic          b_row_open;
logic          b_aborted;
// One-deep burst request buffer. Originally this existed only so a burst request
// arriving in the same cycle as a refresh could not be dropped. It now does double duty
// as the PIPELINE stage: a request is accepted into it from ANY state, so the next burst
// is already latched when the current one retires and S_BTRP can chain straight into it.
logic          b_pending;
logic [AW-1:0] b_req_addr;
logic [8:0]    b_req_len;
logic [1:0]    b_open_ba;
logic [ROW_BITS-1:0] b_open_row;

wire [1:0]          b_ba_w  = b_addr[BA_HI:BA_LO];
wire [ROW_BITS-1:0] b_row_w = b_addr[ROW_HI:ROW_LO];
wire [12:0]         b_row_a = {{(13-ROW_BITS){1'b0}}, b_row_w};
// A10 = 0: NO auto-precharge, the row stays open for the next word of the burst.
wire [12:0]         b_col_a = {2'b00, 1'b0, {(10-COL_BITS){1'b0}}, b_addr[COL_BITS-1:0]};

// The next word lives in a different row/bank than the one currently open.
wire b_row_miss = !b_row_open || (b_ba_w != b_open_ba) || (b_row_w != b_open_row);
// Issue a READ this cycle?
wire b_issue = (state == S_BRD) && (b_left != 0) && !brst_abort && !b_row_miss;

always_comb begin
	cmd = CMD_NOP; a_comb = '0; ba_comb = '0; dq_oe = 1'b0;
	unique case (state)
		S_PRE:  begin cmd = CMD_PRECHARGE; a_comb = PRE_ALL; end
		S_REFI: cmd = CMD_REFRESH;
		S_MRD:  begin cmd = CMD_LMR; a_comb = MODE_REG; end
		S_ACT:  begin cmd = CMD_ACTIVE; ba_comb = ba_w; a_comb = row_a; end
		S_RD:   begin cmd = CMD_READ;  ba_comb = ba_w; a_comb = col_a; end
		S_WR:   begin cmd = CMD_WRITE; ba_comb = ba_w; a_comb = col_a; dq_oe = 1'b1; end
		S_REF:  cmd = CMD_REFRESH;
		S_BACT: begin cmd = CMD_ACTIVE; ba_comb = b_ba_w; a_comb = b_row_a; end
		S_BRD:  if (b_issue) begin cmd = CMD_READ; ba_comb = b_ba_w; a_comb = b_col_a; end
		S_BPRE: begin cmd = CMD_PRECHARGE; a_comb = PRE_ALL; end
		default: ;
	endcase
end

// ---------------------------------------------------------------------------
// Registered output stage. The command/address pins were COMBINATIONAL
// (`assign SDRAM_A = a_comb`), so the framework's `Fast Output Register=ON -to SDRAM_*`
// had nothing to pack -- the fit report shows `output register: no` on every SDRAM_A pin,
// and Quartus said so in its own way: Info 176252 "some destinations are not valid
// targets". The single-word engine survived that because its address is REGISTERED at
// request time and sits stable for many cycles before the READ edge. The burst engine is
// the ONE client that must change the address pins every cycle, racing SDRAM_CLK's
// mid-cycle sampling edge through a mux+adder cone.
//
// Hardware convicted it with a data-level fingerprint: the tile image in
// SDRAM is byte-perfect through the single-word path (checksum 0x28C2) while EVERY 2-word
// burst returns its SECOND word in BOTH slots (0xED4B == V(a)=img(a|1), exact, boot- and
// build-stable) -- the chip is latching the second read's column for both reads. No
// simulation can see it: the sim model samples ideal pins on ideal edges.
//
// One flop stage on every SDRAM output: IOE-packable (verify `output register: yes` in
// the fit after any change here), pins stable a full cycle around each chip edge. Every
// command reaches the chip exactly one cycle later, uniformly, so inter-command spacings
// (tRCD/tRP/tRAS/tRC) are untouched -- but read data also returns one cycle later
// relative to the ISSUING state, hence RD_LAT = CL+1 and the single path's S_CL wait
// gained a cycle. The sim model sees the same shifted stream, so the gates verify the
// alignment rather than assuming it.
// ---------------------------------------------------------------------------
logic [3:0]  cmd_r   = CMD_NOP;
logic [12:0] a_r     = '0;
logic [1:0]  ba_r    = '0;
logic        dq_oe_r = 1'b0;
logic [15:0] dq_out_r = '0;
// Unconditional DQ input capture (the ACTUAL fix for img(a|1)). An SDR chip
// LAUNCHES read data off the edge BEFORE the capture edge, so with SDRAM_CLK phase-shifted
// inside the fabric period (-180 deg -- a later move to 309.6 deg was REVERTED after it
// corrupted SDRAM on hardware; see rtl/pll/pll_0002.v) the data sits on the bus one full
// fabric cycle EARLIER than the consume points assumed.
// Consuming SDRAM_DQ directly therefore read each burst word one cycle late: the single-
// word path still worked -- a floating bus HOLDS its last value, and with one word in
// flight "late" reads the held (correct) word -- but in a burst the NEXT word has already
// overwritten the bus, so every 2-word burst returned its SECOND word twice
// (hardware fingerprint V(a)=img(a|1) = 0xED4B; sprites/pinned modes clean; registered
// command pins changed nothing, which is what killed the pin-race theory).
// dq_in_r samples the pins EVERY cycle (no enable -> IOE-packable, and the framework's
// FAST_INPUT_REGISTER ON -to SDRAM_DQ finally has its register); both consume points read
// dq_in_r one cycle later than the pin window, which is exactly where their existing
// RD_LAT = CL+1 / S_CL = CL waits already sample. Verified against a simulation chip model
// that reproduces the exact hardware fingerprint when this register is bypassed.
logic [15:0] dq_in_r = '0;
always_ff @(posedge clk) begin
	cmd_r    <= cmd;
	a_r      <= a_comb;
	ba_r     <= ba_comb;
	dq_oe_r  <= dq_oe;
	dq_out_r <= wdata_l;
end

// ---------------------------------------------------------------------------
// DQ is captured on the negedge. This is the whole capture-slot fix, and
// the reason it is here and not in the PLL.
//
// The requirement is a SLOT, not a delay: the word launched by the SDRAM_CLK edge at 15.625 ns
// must be captured at the clk_sys edge at 62.500 and must MISS the one at 31.250. A one-cycle
// slip is exactly V(a)=img(a|1), the tile corruption that cost weeks.
// Measured on the fitted netlist, that slot was open at BOTH ends of the PVT range: the data
// reaches this flop at 35.003..42.192 ns while the 31.250 edge reaches it at 39.217..40.679 --
// the unwanted edge sits INSIDE the arrival window, so which slot wins was decided by PVT, not
// by design. The board landed in 62.500 and all five games ran, but nothing HELD it there.
//
// There are only two ways to open the gap, and they are NOT symmetric:
//   (a) move the DATA later  -> delay SDRAM_CLK. Tried, shipped, corrupted the board. It
//       spends the SDRAM's own command/address hold one-for-one (12.461 -> 1.846 ns) against a
//       `set_output_delay` that carries tIS/tIH and NOTHING for board skew, so addresses latch
//       marginally and reads land on the wrong row. Reverted.
//   (b) move the capture edge earlier -> shift only this flop. The late side had ~28 ns of
//       unused slack, so this is pure conversion of setup margin into hold margin and it costs
//       the command/address side NOTHING -- SDRAM_CLK, the output register stage above and every
//       `set_output_delay` path are bit-for-bit untouched.
// (b) is free here because the negedge of clk_sys already exists: same net, same global network,
// no new PLL output, no VCO recompute that could perturb outclk_2's phase.
//
// Shifting the latch 15.625 ns earlier moves both sides by exactly that much (slack is linear in
// latch time; the data path is unchanged). Measured across all four operating conditions:
//     corner        late(setup)      early(hold)
//     slow  100c    28.093 ->  12.468    -5.707 -> +9.918
//     slow  -40c    28.882 ->  13.257    -6.198 -> +9.427
//     fast  100c    33.363 ->  17.738    -8.889 -> +6.736
//     fast  -40c    34.365 ->  18.740    -9.851 -> +5.774
// Any shift in 9.851..28.093 ns closes both sides; the negedge sits inside that window with
// >= 5.774 ns to spare, ~3x what the reverted phase fix left on the address side, on a path
// whose unmodelled board skew is far smaller (DQ and SDRAM_CLK share the module).
// The long-quoted -5.676 ns deficit was the slow corner only. The true worst is -9.851 at
// fast/-40c: the DQ path runs SDRAM_CLK out through a pad and back while the capture clock
// crosses only the internal network. Score every corner or do not score at all.
//
// The fabric alignment is unchanged on silicon, which is why RD_LAT does not move. The word
// from SDRAM_CLK pin edge N is valid at this flop for a full period; the old posedge M and the
// new negedge M+0.5 both fall inside the SAME word's window (M-N = 1 either way), and the
// consumer still reads it at posedge M+1. So RD_LAT = CL+1 and the single path's S_CL wait stay
// exactly as they are -- do NOT "compensate" for this in the controller.
// A simulation chip model DOES need a matching change, and it is not a fudge: the model drives
// SDRAM_DQ combinationally off a posedge pipeline, so in zero-delay simulation a negedge capture
// sees the driven word one delta-position EARLIER than a posedge one -- one whole fabric cycle at
// the consumer. The model puts that cycle back (its read pipe runs CL-1 -> CL). Mutating either
// half alone reproduces the 0xED4B img(a|1) fingerprint.
// ---------------------------------------------------------------------------
always_ff @(negedge clk) begin
	dq_in_r <= SDRAM_DQ;
end

assign SDRAM_CLK = ~clk;
assign SDRAM_CKE = ~reset;
assign {SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} = cmd_r;
assign SDRAM_A   = a_r;
assign SDRAM_BA  = ba_r;
assign SDRAM_DQ  = dq_oe_r ? dq_out_r : 16'hzzzz;
assign {SDRAM_DQMH, SDRAM_DQML} = 2'b00;
assign ready     = (state == S_IDLE) && !rfsh_req && !pending;
// The buffer is free, and we are past power-up init.
assign brst_accept = !b_pending && (state != S_INIT) && (state != S_PRE) &&
                     (state != S_TRP_I) && (state != S_REFI) && (state != S_TRC_I) &&
                     (state != S_MRD) && (state != S_TMRD);
assign brst_busy = (state == S_BACT) || (state == S_BTRCD) || (state == S_BRD) ||
                   (state == S_BTAIL) || (state == S_BPRE) || (state == S_BTRP);

// helper: wait state that decrements dly, jumps to NEXT when it hits 0.
`define WAIT(NEXT) begin if (dly != 0) dly <= dly - 1'b1; else state <= NEXT; end

always_ff @(posedge clk) begin
	valid      <= 1'b0;
	brst_valid <= 1'b0;
	brst_done  <= 1'b0;
	brst_start <= 1'b0;

	// Read-return pipeline, shared by every burst READ in flight. A READ issued this
	// cycle has its data on DQ CL cycles later; b_pipe[0] marks the cycle it is due.
	// The single-word path keeps its own explicit S_CL wait and does not use this.
	// Width-safe shift, sized to exactly RD_LAT bits on both sides. A part-select of
	// b_pipe[RD_LAT-1:1] is out of order when RD_LAT==1, and a ternary does not save
	// you -- both arms still elaborate.
	b_pipe <= {b_issue, {(RD_LAT-1){1'b0}}} | (b_pipe >> 1);
	// b_issue is `state == S_BRD` only, so b_pipe[0] marks a BURST word and nothing else.
	// That is the whole distinction brst_valid exposes; the single-word path sets `valid`
	// from its own S_CL wait below and never touches this pipe.
	if (b_pipe[0]) begin
		rdata      <= dq_in_r;
		valid      <= 1'b1;
		brst_valid <= 1'b1;
	end
	// One place only: issuing adds an in-flight read, returning one removes it. Doing
	// this in two branches would let the later assignment silently win.
	case ({b_issue, b_pipe[0]})
		2'b10:   b_inflight <= b_inflight + 1'b1;
		2'b01:   b_inflight <= b_inflight - 1'b1;
		default: ;  // 00 = idle, 11 = one in one out
	endcase

	if (state != S_INIT && state != S_PRE && state != S_TRP_I &&
	    state != S_REFI && state != S_TRC_I && state != S_MRD && state != S_TMRD) begin
		if (rfsh_ctr >= tREF[15:0]) begin rfsh_ctr <= '0; rfsh_req <= 1'b1; end
		else rfsh_ctr <= rfsh_ctr + 1'b1;
	end

	if (reset) begin
		state <= S_INIT; dly <= tINIT[15:0]; ref_init <= '0;
		rfsh_ctr <= '0; rfsh_req <= 1'b0; pending <= 1'b0;
		b_left <= '0; b_inflight <= '0; b_pipe <= '0; b_row_open <= 1'b0;
		b_aborted <= 1'b0; b_pending <= 1'b0;
	end else begin
		// Latch a burst request from ANY state while the buffer is free. This is what lets
		// the arbiter hand over the next burst during the current one's drain. Placed
		// before the case so a state that also writes b_pending (the launch paths below)
		// overrides it in the same cycle -- last assignment wins, and the launch is what
		// consumed the buffer.
		if (brst_req && brst_accept) begin
			b_req_addr <= brst_addr; b_req_len <= brst_len; b_pending <= 1'b1;
		end

		unique case (state)
		// ---- power-up init ----
		S_INIT:  begin if (dly != 0) dly <= dly - 1'b1; else begin state <= S_PRE; end end
		S_PRE:   begin state <= S_TRP_I; dly <= tRP-16'd1; end
		S_TRP_I: begin if (dly != 0) dly <= dly-1'b1; else begin state <= S_REFI; ref_init <= '0; end end
		S_REFI:  begin state <= S_TRC_I; dly <= tRC[15:0]-1'b1; end
		S_TRC_I: begin if (dly != 0) dly <= dly-1'b1;
		               else if (ref_init == 4'd7) state <= S_MRD;
		               else begin ref_init <= ref_init + 1'b1; state <= S_REFI; end end
		S_MRD:   begin state <= S_TMRD; dly <= tMRD[15:0]-1'b1; end
		S_TMRD:  `WAIT(S_IDLE)

		// ---- normal operation ----
		S_IDLE: begin
			// Capture an incoming request so a coincident refresh can't drop it.
			if (req && !pending) begin
				we_l <= we; wdata_l <= wdata; addr_l <= addr; pending <= 1'b1;
			end
			if (rfsh_req) begin state <= S_REF; rfsh_req <= 1'b0; end
			else if (pending || req) begin pending <= 1'b0; state <= S_ACT; end
			else if (b_pending || brst_req) begin
				// Single-word requests win over a burst: the CPU is the latency-critical
				// client and a burst can always be preempted, never the other way round.
				b_addr     <= b_pending ? b_req_addr : brst_addr;
				b_left     <= b_pending ? ((b_req_len == 9'd0) ? 10'd1 : {1'b0, b_req_len})
				                        : ((brst_len  == 9'd0) ? 10'd1 : {1'b0, brst_len});
				b_pending  <= 1'b0;
				b_row_open <= 1'b0;          // force the first ACTIVE
				b_inflight <= '0;
				b_aborted  <= 1'b0;
				brst_start <= 1'b1;
				state      <= S_BRD;
			end
		end

		// ---- burst read ----
		// S_BRD is the issue loop. It either fires a READ (one per cycle, row held
		// open), or handles a row change, or falls through to the drain.
		S_BRD: begin
			if (brst_abort) b_aborted <= 1'b1;
			if (b_issue) begin
				b_addr <= b_addr + 1'b1;
				b_left <= b_left - 1'b1;
			end else if ((b_left != 0) && !brst_abort && !b_aborted) begin
				// Row miss. If nothing is open yet this is just the first ACTIVE.
				// Otherwise go through the drain path: reads already in flight must be
				// returned BEFORE the row is precharged, or their data is lost.
				state <= b_row_open ? S_BTAIL : S_BACT;
			end else begin
				// Done issuing -- the count ran out, or abort stopped us at a word
				// boundary. Drain the reads still in flight before closing the row.
				state <= S_BTAIL;
			end
		end
		S_BACT:  begin
			b_open_ba <= b_ba_w; b_open_row <= b_row_w; b_row_open <= 1'b1;
			state <= S_BTRCD; dly <= tRCD-16'd1;
		end
		S_BTRCD: `WAIT(S_BRD)
		S_BTAIL: begin
			// b_inflight counts reads whose data has not yet been returned. The valid
			// pulses keep flowing from the shared pipeline while we sit here.
			if (b_inflight == 0 && !b_pipe[0]) begin
				// Same-row chaining: if the buffered next burst lives in the row that is
				// already open, retire this one and start it WITHOUT precharging.
				//
				// A 2-word playfield column otherwise costs ACT + tRCD + 2 READ + CL drain +
				// PRECHARGE + tRP ~= 12.7 clk for 2 data words -- only ~16 % of the fill is
				// data. Adjacent tile codes frequently share a row, and skipping the
				// PRE/tRP/ACT/tRCD round trip on those saves ~4 of those 12.7 clk.
				//
				// This matters because a dense sprite load pushes the fill past the line:
				// measured 1455 clk against a 1279 clk line at 24 sprite reads/line, which
				// publishes a half-filled buffer = wrong tile bitmaps under right colours.
				//
				// Refresh and the single-word client still win, exactly as the S_BTRP chain
				// checks them, so this cannot starve either.
				// `req` (the single-word client) may be pending and we STILL chain, up to
				// CHAIN_MAX columns. That is deliberate and it is what makes the playfield fit
				// its deadline under a sprite load:
				//
				//   The playfield fill has a HARD per-line deadline -- miss it and a half-filled
				//   buffer is published (scattered wrong tiles). The single-word client here is
				//   the sprite ROM, which has a whole line to do its 40 slots x 4 words, so it
				//   has slack the playfield does not. Yielding the row to it on every request
				//   costs the playfield a full PRE/tRP/ACT/tRCD round trip each time.
				//
				//   CHAIN_MAX bounds the sprite's extra wait to a few columns (~6 clk each), far
				//   inside its slack, while letting the fill make real progress. Refresh still
				//   wins unconditionally -- it must.
				if (b_pending && !rfsh_req && b_row_open
				    && (!pending && !req || b_chain_cnt < CHAIN_MAX)
				    && (b_req_addr[ROW_HI:ROW_LO] == b_open_row)
				    && (b_req_addr[BA_HI:BA_LO]   == b_open_ba)) begin
					b_chain_cnt <= b_chain_cnt + 1'b1;
					brst_done  <= 1'b1;                 // retire the burst that just drained
					b_addr     <= b_req_addr;
					b_left     <= (b_req_len == 9'd0) ? 10'd1 : {1'b0, b_req_len};
					b_pending  <= 1'b0;
					b_inflight <= '0;
					b_aborted  <= 1'b0;
					brst_start <= 1'b1;
					state      <= S_BRD;                // row stays open: straight to READs
				end else begin
					b_chain_cnt <= '0;                  // yielding: the run ends here
					state <= S_BPRE;
				end
			end
		end
		S_BPRE:  begin state <= S_BTRP; dly <= tRP-16'd1; b_row_open <= 1'b0; end
		S_BTRP:  begin
			if (dly != 0) dly <= dly-1'b1;
			// Resume only for a genuine row change. b_aborted is LATCHED: if the client
			// drops brst_abort during the drain, the burst must still stay cancelled
			// rather than quietly picking up where it left off.
			else if (b_left != 0 && !b_aborted) state <= S_BACT;
			else begin
				brst_done <= 1'b1; b_left <= '0;
				// Back-to-back chaining. If the next burst is already buffered and nothing
				// with higher standing wants the controller, start it now instead of
				// walking out to S_IDLE and waiting to be asked again. That round trip --
				// S_IDLE plus the arbiter's registered request handshake -- was a measured
				// 33.6 % of all cycles for the playfield's one-burst-per-column pattern.
				//
				// Refresh and the single-word (CPU) path still win: both are checked here
				// exactly as S_IDLE checks them, so chaining can never starve a refresh or
				// push the latency-critical client behind a burst.
				if (b_pending && !rfsh_req && !pending && !req) begin
					b_addr     <= b_req_addr;
					b_left     <= (b_req_len == 9'd0) ? 10'd1 : {1'b0, b_req_len};
					b_pending  <= 1'b0;
					b_row_open <= 1'b0;      // force the first ACTIVE
					b_inflight <= '0;
					b_aborted  <= 1'b0;
					brst_start <= 1'b1;
					state      <= S_BRD;
				end else begin
					state <= S_IDLE;
				end
			end
		end
		S_ACT:   begin state <= S_TRCD; dly <= tRCD-16'd1; end
		S_TRCD:  begin if (dly != 0) dly <= dly-1'b1; else state <= we_l ? S_WR : S_RD; end
		S_RD:    begin state <= S_CL; dly <= CL[15:0]; end   // CL + 1 for the registered pin stage
		S_CL:    begin if (dly != 0) dly <= dly-1'b1;
		               else begin rdata <= dq_in_r; valid <= 1'b1; state <= S_RECOV; dly <= tRP-16'd1; end end
		S_WR:    begin state <= S_RECOV; dly <= tRP-16'd1; end
		S_RECOV: `WAIT(S_IDLE)
		S_REF:   begin state <= S_TRC; dly <= tRC[15:0]-1'b1; end
		S_TRC:   `WAIT(S_IDLE)
		default: state <= S_IDLE;
		endcase
	end
end

endmodule
