// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Paperboy T-11 memory subsystem: program RAM, color RAM, banked/fixed ROM,
// VMMU RAM, the four CP0-CP3 interrupt latches with their enable register, the
// 014000 switch word (talk flags plus buttons/service), and the watchdog.
//
// Memory reads are registered (one-cycle), so every writable memory is a true
// dual-port BRAM (a CPU read/write port plus a video read port) that infers
// M10K. Local reads therefore complete one clock after the request via a
// level-held bus_ack, exactly like the ROM port; the T-11 absorbs the wait. This
// supersedes the original zero-wait combinational-read assumption, which could
// not synthesize (asynchronous reads of the large playfield/alpha RAMs exploded
// Quartus). Writes stay zero-wait. completed_access is a single pulse per bus
// transaction so the Slapstic and the register strobes fire exactly once even
// when the CPU holds the request across idle clock-enable cycles.
module sys2_main_bus #(
	// Watchdog timeout in 10 MHz T-11 execution-clock ticks. SP6B divides the CPU
	// clock by 2^20: 1/(20 MHz / 2 / 16 / 16 / 16 / 256) = 104.8576 ms. The
	// focused simulation overrides this with a small value.
	parameter int unsigned WATCHDOG_TICKS = 1048576
) (
	input  logic        clk,
	input  logic        video_clk,
	input  logic        reset,
	input  logic        cpu_clk_en,         // 10 MHz T-11 execution-clock enable

	// Slapstic type from the MRA index-1 game descriptor (105/107/108/109/110).
	// On System 2 this selects the VMMU/vram view, not just a ROM bank.
	input  logic [7:0]  slapstic_type,

	// Interrupt set events. Each is a single-cycle strobe in the clk domain;
	// the integrator must synchronize the originating video/sound edges to clk.
	input  logic        irq_snd_cmd_read,   // CP0: sound CPU read the command
	input  logic        irq_snd_resp_write, // CP1: sound CPU wrote a response
	input  logic        irq_scanline_32v,   // CP2: 32V scanline event
	input  logic        irq_vblank,         // CP3: VBLANK assertion edge

	// Sound response latch data, written on the sound side and read at 016000.
	input  logic [7:0]  snd_resp,

	input  logic        bus_req,
	input  logic        bus_write,
	input  logic [15:0] bus_addr,
	input  logic [15:0] bus_wdata,
	input  logic [1:0]  bus_byte_en,
	output logic [15:0] bus_rdata,
	output logic        bus_ack,
	output logic        bus_error,

	output logic        rom_req,
	output logic [19:0] rom_addr,
	input  logic [15:0] rom_rdata,
	input  logic        rom_ack,

	input  logic [11:0] alpha_video_addr,
	output logic [15:0] alpha_video_data,
	input  logic [9:0]  mob_video_addr,
	output logic [15:0] mob_video_data,
	input  logic [11:0] playfield_top_video_addr,
	output logic [15:0] playfield_top_video_data,
	input  logic [11:0] playfield_bottom_video_addr,
	output logic [15:0] playfield_bottom_video_data,
	// Third read port on both playfield map RAMs, for the SDRAM tile PREFETCHER. It used to
	// steal the renderer's port on "spare" slots, which corrupted the renderer's palette group
	// and priority category on every stolen cycle. See sys2_dpram's PORT_C note.
	input  logic [11:0] playfield_top_pf_addr,
	output logic [15:0] playfield_top_pf_data,
	input  logic [11:0] playfield_bottom_pf_addr,
	output logic [15:0] playfield_bottom_pf_data,
	input  logic [7:0]  palette_video_addr,
	output logic [15:0] palette_video_data,

	output logic [5:0]  rom_bank0,
	output logic [5:0]  rom_bank1,
	output logic [1:0]  vmmu_bank,

	output logic [3:0]  cpu_cp,      // coded interrupt request to the T-11
	output logic [3:0]  irq_enable,  // latched 013000 enable bits

	output logic [7:0]  snd_cmd,         // command latch to the sound 6502
	output logic        snd_cmd_pending, // command pending; drives the 6502 NMI
	output logic        resp_pending,    // response pending; visible to sound status
	output logic        snd_reset,       // 6502 reset level from 012640 bit 0

	// Player controls. Inputs are active-high (1 = pressed/asserted); the module
	// presents them at their active-low levels inside the 014000 switch word.
	// btn1/btn2 keep their Paperboy meaning; the other games re-map them below.
	input  logic        btn1,            // button 1       -> IN0 bit 7 (Paperboy/720)
	input  logic        btn2,            // button 2       -> IN0 bit 6 (Paperboy/720)
	input  logic        service,         // self-test/service -> IN0 bit 15 (all games)

	// Extra IN0 sources used only by the other System 2 games. All active-high.
	// Paperboy leaves them at 0 and the game_id case below never reads them.
	input  logic        start1,          // Sprints: 1P start   -> IN0 bit 7
	input  logic        start2,          // Sprints: 2P start   -> IN0 bit 6
	input  logic        start3,          // Super Sprint only   -> IN0 bit 3
	input  logic        btn3,            // APB: third button   -> IN0 bit 3

	// Which game the MRA descriptor selected (SYS2_GAME_*). IN0's bit assignment differs
	// per game, not just its wiring, so the map below is a case on this.
	//
	// IN0 is not the only such port -- MAME's ssprint also does PORT_MODIFY("IN1"), which
	// ROTATES the coin bits (paperboy 0x20/0x40/0x80 = COIN3/COIN1/COIN2; ssprint =
	// COIN1/COIN2/COIN3). IN1 is on the 6502 side, so that map lives in
	// sys2_sound_bus.sv (the 0x1840 status word) and is handled there. Do not read this
	// comment as "IN0 is the only per-game input word".
	input  logic [7:0]  game_id,

	// Handlebar ADC. Eight pre-digitized analog channels (Paperboy wires 0 = X,
	// 1 = Y; the rest read inactive). adc_clk_en is the ~625 kHz conversion clock.
	// The 012200 strobe starts a conversion; 012000 reads the latched result.
	input  logic        adc_clk_en,
	input  logic [7:0]  adc_in [0:7],
	output logic        adc_eoc,         // ADC end-of-conversion (1 = idle/done)

	// Playfield scroll/bank registers, the raw 16-bit values latched on the CPU
	// write (matching MAME's shared m_xscroll/m_yscroll). The video stage applies
	// the per-scanline/frame timing and slices the fields:
	//   xscroll = {hscroll[15:6], unused[5:4], pf_bank0[3:0]}
	//   yscroll = {unused[15], vscroll[14:6], reserved[5], mode[4], pf_bank1[3:0]}
	output logic [15:0] xscroll,         // 013400
	output logic [15:0] yscroll,         // 013600
	output logic        xscroll_strobe,  // one clk pulse when xscroll is written
	output logic        yscroll_strobe,  // one clk pulse when yscroll is written

	output logic        watchdog_reset   // one-clk pulse when the watchdog expires
);

// SYS2_GAME_* for the per-game IN0 map below. One include per module -- see the
// no-include-guard note at the top of the header.
`include "rtl/rom/sys2_rom_layout.vh"

// Writable memories are clean dual-port BRAMs (sys2_dpram, instantiated
// below): port A is the CPU (per-byte write + registered read), port B is the
// video read port clocked by video_clk. The alpha RAM is depth 4096 (only the
// lower 3072 alpha tiles are used) so the bus_addr[12:1] index is always in
// range across the VMMU window. The registered CPU read data feeds the bus_rdata
// mux; the video read data drives the *_video_data outputs.
logic [15:0] program_cpu_q, color_cpu_q, alpha_cpu_q, mob_cpu_q;
logic [15:0] playfield_top_cpu_q, playfield_bottom_cpu_q;
logic [15:0] program_b_unused;   // program RAM has no video port (port B unused)

// Single per-transaction completion pulse (driven in the handshake section).
logic completed_access;

// Page-data (write bits [15:10]) -> physical 8 KiB bank. MAME 0.288 bankselect_w,
// exactly: bank = bitswap(page ^ 3, 5,4,1,0,3,2) -- the XOR comes FIRST, then the
// low bit-pairs swap.
//
// History of this `^ 6'o03` (removed once, then restored -- read before
// touching): the XOR is REAL hardware behaviour, but with an early window map it sent
// Paperboy's attract bank (0xF000 -> bank 63 -> 32K unit 17) outside the stored window
// set -> 0xffff holes -> the 0xaa00 builder stall.
// Removing the XOR "fixed" that ONLY because Paperboy's banked region is 4-way mirrored
// (units 2..5, 6..9, 10..13, 14..17 hold identical bytes -- ROM_RELOAD), so ANY decode
// error within a quad serves identical data. The real defect was the missing mirror
// keys in sys2_maincpu_rom's window map. ssprint's region mirrors only pairwise
// (2=4, 3=5, ...), and bank^0xC crosses the pair: without the XOR its map-source read
// (bankselect 0x1000, MAME bank 13, region 0x2A000 = track data) served region 0x12000
// = a sea of 0x0F0F -- which the draw loop faithfully painted as the uniform pink dot
// grid observed on hardware. Verified against MAME bank writes: 0x1000->13,
// 0xE000->62, 0xF000->63 (bankoffset[] table in atarisy2.cpp agrees).
function automatic logic [5:0] decode_rom_bank(input logic [5:0] page_data);
	logic [5:0] pd;
	begin
		pd = page_data ^ 6'o03;
		decode_rom_bank = {pd[5],pd[4],pd[1],pd[0],pd[3],pd[2]};
	end
endfunction

function automatic logic [19:0] paged_rom_address(
	input logic [5:0] bank,
	input logic [11:0] word_offset
);
	paged_rom_address = 20'h10000 + {1'b0,bank,13'b0}
	                  + {7'b0000000,word_offset,1'b0};
endfunction

// Slapstic type arrives at runtime from the MRA index-1 game descriptor, so one
// bitstream serves every System 2 title. The loader has already refused to release the
// CPU unless the type is one of the five and agrees with the game id, so an unsupported
// value cannot reach here in a running machine; `type_supported` is left unconnected
// deliberately rather than duplicating that gate.
/* verilator lint_off PINCONNECTEMPTY */
sys2_slapstic slapstic (
	.clk,
	.reset,
	.slapstic_type,
	.access(completed_access),
	.address(bus_addr),
	.bank(vmmu_bank),
	.type_supported()
);
/* verilator lint_on PINCONNECTEMPTY */

// ADC0809. A low-byte write in 012200-012217 (mirror 0x70) starts a conversion;
// the channel is the word offset within the window (addr[3:1]). 012000 reads the
// 8-bit result.
logic [2:0] adc_channel;
logic       adc_start;
logic [7:0] adc_data;

assign adc_channel = bus_addr[3:1];

adc0809 adc (
	.clk,
	.reset,
	.clk_en(adc_clk_en),
	.channel_in(adc_in),
	.start(adc_start),
	.channel(adc_channel),
	.data(adc_data),
	.eoc(adc_eoc)
);

// ---------------------------------------------------------------------------
// Access decode
// ---------------------------------------------------------------------------
logic rom_access, local_read;
logic vmmu_range, sel_alpha, sel_mob, sel_pf_top, sel_pf_bot;

assign rom_access = bus_req && !bus_write && (bus_addr >= 16'h4000);
assign local_read = bus_req && !bus_write && (bus_addr <  16'h4000);

assign vmmu_range = (bus_addr >= 16'h2000) && (bus_addr < 16'h4000);
assign sel_alpha  = vmmu_range && (vmmu_bank == 2'd0) && (bus_addr <  16'h3800);
assign sel_mob    = vmmu_range && (vmmu_bank == 2'd0) && (bus_addr >= 16'h3800);
assign sel_pf_top = vmmu_range && (vmmu_bank == 2'd2);
assign sel_pf_bot = vmmu_range && (vmmu_bank == 2'd3);

// Per-RAM write enables. completed_access is a single per-transaction pulse, so
// each write commits exactly once.
logic we_program, we_color, we_alpha, we_mob, we_pf_top, we_pf_bot;
assign we_program = completed_access && bus_write && (bus_addr < 16'h1000);
assign we_color   = completed_access && bus_write && ((bus_addr & 16'hfc00) == 16'h1000);
assign we_alpha   = completed_access && bus_write && sel_alpha;
assign we_mob     = completed_access && bus_write && sel_mob;
assign we_pf_top  = completed_access && bus_write && sel_pf_top;
assign we_pf_bot  = completed_access && bus_write && sel_pf_bot;

// ---------------------------------------------------------------------------
// Writable memories: sys2_dpram dual-port BRAMs. Port A = CPU (per-byte
// write + registered read into *_cpu_q); port B = video read. The video reads
// drive the *_video_data outputs directly (one-cycle latency).
// ---------------------------------------------------------------------------
sys2_dpram #(.AW(11)) u_program_ram (
	.clk,
	.video_clk,
	.a_addr  (bus_addr[11:1]),
	.a_wdata (bus_wdata),
	.a_we    ({we_program & bus_byte_en[1], we_program & bus_byte_en[0]}),
	.a_rdata (program_cpu_q),
	.b_addr  (11'd0),
	.b_rdata (program_b_unused),
	.c_addr  (11'd0),
	.c_rdata ()          // PORT_C off: no third copy is inferred
);

sys2_dpram #(.AW(8)) u_color_ram (
	.clk,
	.video_clk,
	.a_addr  (bus_addr[8:1]),
	.a_wdata (bus_wdata),
	.a_we    ({we_color & bus_byte_en[1], we_color & bus_byte_en[0]}),
	.a_rdata (color_cpu_q),
	.b_addr  (palette_video_addr),
	.b_rdata (palette_video_data),
	.c_addr  (8'd0),
	.c_rdata ()          // PORT_C off: no third copy is inferred
);

sys2_dpram #(.AW(12)) u_alpha_ram (
	.clk,
	.video_clk,
	.a_addr  (bus_addr[12:1]),
	.a_wdata (bus_wdata),
	.a_we    ({we_alpha & bus_byte_en[1], we_alpha & bus_byte_en[0]}),
	.a_rdata (alpha_cpu_q),
	.b_addr  (alpha_video_addr),
	.b_rdata (alpha_video_data),
	.c_addr  (12'd0),
	.c_rdata ()          // PORT_C off: no third copy is inferred
);

sys2_dpram #(.AW(10)) u_mob_ram (
	.clk,
	.video_clk,
	.a_addr  (bus_addr[10:1]),
	.a_wdata (bus_wdata),
	.a_we    ({we_mob & bus_byte_en[1], we_mob & bus_byte_en[0]}),
	.a_rdata (mob_cpu_q),
	.b_addr  (mob_video_addr),
	.b_rdata (mob_video_data),
	.c_addr  (10'd0),
	.c_rdata ()          // PORT_C off: no third copy is inferred
);


sys2_dpram #(.AW(12), .PORT_C(1'b1)) u_pf_top_ram (
	.clk,
	.video_clk,
	.a_addr  (bus_addr[12:1]),
	.a_wdata (bus_wdata),
	.a_we    ({we_pf_top & bus_byte_en[1], we_pf_top & bus_byte_en[0]}),
	.a_rdata (playfield_top_cpu_q),
	.b_addr  (playfield_top_video_addr),
	.b_rdata (playfield_top_video_data),
	.c_addr  (playfield_top_pf_addr),
	.c_rdata (playfield_top_pf_data)
);

sys2_dpram #(.AW(12), .PORT_C(1'b1)) u_pf_bot_ram (
	.clk,
	.video_clk,
	.a_addr  (bus_addr[12:1]),
	.a_wdata (bus_wdata),
	.a_we    ({we_pf_bot & bus_byte_en[1], we_pf_bot & bus_byte_en[0]}),
	.a_rdata (playfield_bottom_cpu_q),
	.b_addr  (playfield_bottom_video_addr),
	.b_rdata (playfield_bottom_video_data),
	.c_addr  (playfield_bottom_pf_addr),
	.c_rdata (playfield_bottom_pf_data)
);

// ---------------------------------------------------------------------------
// Handshake
// ---------------------------------------------------------------------------
// Local reads: register the read DATA and assert ack one clock later, so the CPU
// captures a registered value via a short reg-to-reg hop rather than latching the
// deep combinational read mux across the fabric. On this board the CPU cannot
// reliably capture a live combinational read (proven by the ROM-data fix,
// rom_rdata_q): the 0x1800 switch-word read mis-branched the boot into the
// self-test path (the boot demonstrably entered self-test even though `service` is a clean,
// constant 0) until this last combinational CPU read was registered too. Local
// reads now take two clocks; both ack and data are held while the request stands,
// so a CPU on a divided clock enable cannot miss them. Writes are zero-wait; ROM
// uses rom_ack (rom_rdata is already registered in the ROM module).
logic [15:0] local_rdata;     // combinational read mux (assigned below)
logic [15:0] local_rdata_q;   // registered after the read sources have settled
logic        local_ack_r, local_ack;
always_ff @(posedge clk) begin
	if (reset) begin
		local_ack_r   <= 1'b0;
		local_ack     <= 1'b0;
		local_rdata_q <= 16'hffff;
	end else begin
		local_ack_r   <= local_read;    // cycle 1: CPU-port RAM q is valid
		local_rdata_q <= local_rdata;    // capture the settled read value
		local_ack     <= local_ack_r;    // cycle 2: ack aligned with local_rdata_q
	end
end

assign bus_error = 1'b0;

always @* begin
	if (!bus_req)        bus_ack = 1'b0;
	else if (bus_write)  bus_ack = 1'b1;     // writes are zero-wait
	else if (rom_access) bus_ack = rom_ack;  // ROM read latency
	else                 bus_ack = local_ack;// local read: registered data, held
end

// completed_access is a single pulse at the first clock of each transaction (the
// rising edge of bus_req). The Slapstic, RAM writes, and register strobes fire
// exactly once regardless of read latency or how long the CPU holds the request;
// the T-11 deasserts bus_req between accesses. Read side effects (e.g. the 016000
// CP1 clear) and the address-based Slapstic update are correct at the start edge.
logic bus_req_d;
always_ff @(posedge clk) begin
	if (reset) bus_req_d <= 1'b0;
	else       bus_req_d <= bus_req;
end
assign completed_access = bus_req && !bus_req_d;

// ROM address/select (combinational; the data and ack come from the ROM module).
always @* begin
	rom_addr = 20'h00000;
	if (bus_addr >= 16'h4000 && bus_addr < 16'h6000)
		rom_addr = paged_rom_address(rom_bank0, bus_addr[12:1]);
	else if (bus_addr >= 16'h6000 && bus_addr < 16'h8000)
		rom_addr = paged_rom_address(rom_bank1, bus_addr[12:1]);
	else if (bus_addr[15])
		rom_addr = {4'b0000, bus_addr[15:1], 1'b0};
end
assign rom_req = rom_access;

// ---------------------------------------------------------------------------
// 014000 switch/flags word (combinational). Talk flags active-high; everything
// else active-low, so the default of all-ones is "nothing pressed".
//
// Bits 15 (self-test), 5 (P1TALK) and 4 (P2TALK) are the SAME on all five games.
// Only the player-input bits move, and they move by game -- so this is a case on
// game_id, not a wiring change in the integrator. Derived from MAME 0.288
// atarisy2.cpp INPUT_PORTS_START(paperboy) plus each game's PORT_MODIFY("IN0"):
//
//   game      bit7      bit6      bit3      bit1
//   paperboy  BUTTON1   BUTTON2   --        --
//   720       BUTTON1   BUTTON2   --        --      (inherits paperboy unmodified)
//   ssprint   START1    START2    START3    --
//   csprint   START1    START2    --        --      (2-player: no START3)
//   apb       --        --        BUTTON3   BUTTON2
//
// APB really has no BUTTON1 in IN0: its PORT_MODIFY clears paperboy's 0x80 and
// 0x40 to IPT_UNUSED and adds only BUTTON2 (0x02) and BUTTON3 (0x08). That is
// MAME's mapping verbatim, not an omission here -- APB's remaining controls are
// the LETA wheel and the ADC pedal, which do not pass through this word.
// ---------------------------------------------------------------------------
logic [15:0] switch_word;
always @* begin
	switch_word = 16'hffff;
	switch_word[15] = ~service;        // self-test switch  (all games)
	switch_word[5]  = snd_cmd_pending; // P1TALK            (all games)
	switch_word[4]  = resp_pending;    // P2TALK            (all games)

	case (game_id)
		SYS2_GAME_SSPRINT: begin
			switch_word[7] = ~start1;
			switch_word[6] = ~start2;
			switch_word[3] = ~start3;
		end
		SYS2_GAME_CSPRINT: begin
			switch_word[7] = ~start1;
			switch_word[6] = ~start2;
		end
		SYS2_GAME_APB: begin
			switch_word[3] = ~btn3;
			switch_word[1] = ~btn2;
		end
		default: begin                 // Paperboy and 720 share this map
			switch_word[7] = ~btn1;
			switch_word[6] = ~btn2;
		end
	endcase
end

// Local read source mux (combinational). Registered into local_rdata_q (in the
// handshake above) before the CPU samples it. RAM sources are the registered
// CPU-port reads; register sources are combinational. Unmapped reads inactive-high.
always @* begin
	if (bus_addr < 16'h1000) begin
		local_rdata = program_cpu_q;
	end else if ((bus_addr & 16'hfc00) == 16'h1000) begin
		local_rdata = color_cpu_q;                     // color RAM (+ 0x200 mirror)
	end else if ((bus_addr & 16'hff81) == 16'h1400) begin
		local_rdata = {8'hff, adc_data};               // 012000 ADC result
	end else if ((bus_addr & 16'hfc01) == 16'h1800) begin
		local_rdata = switch_word;                     // 014000 switches/flags
	end else if ((bus_addr & 16'hfc01) == 16'h1c00) begin
		local_rdata = {8'hff, snd_resp};               // 016000 sound response
	end else if (sel_alpha) begin
		local_rdata = alpha_cpu_q;
	end else if (sel_mob) begin
		local_rdata = mob_cpu_q;
	end else if (sel_pf_top) begin
		local_rdata = playfield_top_cpu_q;
	end else if (sel_pf_bot) begin
		local_rdata = playfield_bottom_cpu_q;
	end else begin
		local_rdata = 16'hffff;
	end
end

// Final CPU read mux: ROM data is registered in the ROM module, local data is
// registered in local_rdata_q -- a clean 2:1 select of two registered sources.
assign bus_rdata = rom_access ? rom_rdata : local_rdata_q;

// ---------------------------------------------------------------------------
// ROM bank and scroll registers (written on the completed-access pulse)
// ---------------------------------------------------------------------------
always_ff @(posedge clk) begin
	if (reset) begin
		rom_bank0 <= 6'd0;
		rom_bank1 <= 6'd0;
		xscroll   <= 16'd0;
		yscroll   <= 16'd0;
		xscroll_strobe <= 1'b0;
		yscroll_strobe <= 1'b0;
	end else if (completed_access && bus_write) begin
		xscroll_strobe <= 1'b0;
		yscroll_strobe <= 1'b0;
		if ((bus_addr & 16'hff80) == 16'h1400) begin
			if (bus_addr[1]) rom_bank1 <= decode_rom_bank(bus_wdata[15:10]);
			else             rom_bank0 <= decode_rom_bank(bus_wdata[15:10]);
		end else if ((bus_addr & 16'hff80) == 16'h1700) begin
			if (bus_byte_en[0]) xscroll[7:0]  <= bus_wdata[7:0];
			if (bus_byte_en[1]) xscroll[15:8] <= bus_wdata[15:8];
			xscroll_strobe <= |bus_byte_en;
		end else if ((bus_addr & 16'hff80) == 16'h1780) begin
			if (bus_byte_en[0]) yscroll[7:0]  <= bus_wdata[7:0];
			if (bus_byte_en[1]) yscroll[15:8] <= bus_wdata[15:8];
			yscroll_strobe <= |bus_byte_en;
		end
	end else begin
		xscroll_strobe <= 1'b0;
		yscroll_strobe <= 1'b0;
	end
end

// ---------------------------------------------------------------------------
// Register strobes
// ---------------------------------------------------------------------------
logic irq_write, irq_read;
logic clr_cp0, clr_cp2, clr_cp3, wr_enable;
logic wr_command, wr_sound_reset, wr_watchdog;

assign irq_write = completed_access && bus_write;
assign irq_read  = completed_access && !bus_write;
assign clr_cp0   = irq_write && ((bus_addr & 16'hffe1) == 16'h1580); // W 012600
assign clr_cp2   = irq_write && ((bus_addr & 16'hffe1) == 16'h15c0); // W 012700
assign clr_cp3   = irq_write && ((bus_addr & 16'hffe1) == 16'h15e0); // W 012740
// The PCB LS74 P2TALK/CP1 flags clear on the read strobe (data accepted), not the request start.
// completed_access pulses at bus_req's rising edge -- two registered-read stages BEFORE local_ack
// delivers local_rdata_q -- so clearing on it can drop a response before the T-11 takes it. Clear on
// the ACCEPT instead (resp_read_sel held while the 016000 read stands, qualified by local_ack), so
// the byte is delivered before the flag is cleared. Mailbox stays atomic with the response latch.
wire resp_read_sel  = bus_req && !bus_write && ((bus_addr & 16'hfc01) == 16'h1c00);
wire clr_cp1_accept = resp_read_sel && local_ack;
assign wr_enable = irq_write && ((bus_addr & 16'hff81) == 16'h1600); // W 013000

// 012200-012217 ADC strobe (mirror 0x70, low byte only): start a conversion on
// the channel selected by the word offset (adc_channel = bus_addr[3:1]).
assign adc_start = irq_write && ((bus_addr & 16'hff80) == 16'h1480)
                 && bus_byte_en[0];

assign wr_command     = irq_write && ((bus_addr & 16'hff81) == 16'h1680); // W 013200
assign wr_sound_reset = irq_write && ((bus_addr & 16'hffe1) == 16'h15a0); // W 012640
assign wr_watchdog    = irq_write && ((bus_addr & 16'hfc01) == 16'h1800); // W 014000

// ---------------------------------------------------------------------------
// Interrupt enable register and the four CP0-CP3 event latches.
// Per MAME d066f16 and the SP6A transcription, each latch samples its own enable
// bit at the set event; delivery to the T-11 is ungated, so clearing an enable
// bit never clears an already-pending latch. A set event wins a coincident clear.
// ---------------------------------------------------------------------------
logic cp0_state, cp1_state, cp2_state, cp3_state;

always_ff @(posedge clk) begin
	if (reset) begin
		irq_enable <= 4'd0;
		cp0_state  <= 1'b0;
		cp1_state  <= 1'b0;
		cp2_state  <= 1'b0;
		cp3_state  <= 1'b0;
	end else begin
		if (wr_enable && bus_byte_en[0]) irq_enable <= bus_wdata[3:0];

		if (irq_snd_cmd_read)   cp0_state <= irq_enable[0];
		else if (clr_cp0)       cp0_state <= 1'b0;

		if (clr_cp1_accept)          cp1_state <= 1'b0;   // read-clear priority (LS74 async clear)
		else if (irq_snd_resp_write) cp1_state <= irq_enable[1];

		if (irq_scanline_32v)   cp2_state <= irq_enable[2];
		else if (clr_cp2)       cp2_state <= 1'b0;

		if (irq_vblank)         cp3_state <= irq_enable[3];
		else if (clr_cp3)       cp3_state <= 1'b0;
	end
end

assign cpu_cp = {cp3_state, cp2_state, cp1_state, cp0_state};

// ---------------------------------------------------------------------------
// Inter-CPU communication latches (main side). Command latch written by the
// T-11 and consumed by the sound CPU; response latch written on the sound side
// (snd_resp) and read here at 016000. Both pending flags appear at 014000.
// ---------------------------------------------------------------------------
always_ff @(posedge clk) begin
	if (reset) begin
		snd_cmd         <= 8'd0;
		snd_cmd_pending <= 1'b0;
		resp_pending    <= 1'b0;
		// RELEASED at power-on. Was 1'b1 -- the 6502 held until the T-11
		// explicitly released it, costing it the head start real hardware gives (both CPUs
		// come off one reset; the 6502 posts its 0xFF startup byte ~26 ms in, BEFORE the
		// T-11's first check). ssprint's boot checks fast: it found nothing, touched
		// sound_reset (0x15A0 -- the failure branch MAME's IO-read trace never takes), and
		// never started attract. Paperboy's patient boot masked this. The
		// DOWNLOAD hold no longer rides on this init: the top level gates the 6502 on its
		// own region being streamed (audio_loaded), which also grants it the tile/sprite
		// download time as a head start over the rom_loaded-gated T-11.
		snd_reset       <= 1'b0;
	end else begin
		// The command is the low byte, so only a low-byte write strobes the latch
		// and its pending flag (matching MAME's even-byte soundlatch handler).
		// BYTE: latched on the T-11's 0x1680 write, persists otherwise (PCB LS374; reset-only clear).
		if (wr_command && bus_byte_en[0]) snd_cmd <= bus_wdata[7:0];
		// P1TALK flag: set on the write, cleared on the 6502's 0x1860 read -- read-clear has PRIORITY
		// over a coincident write (LS74 async clear), so a stale flag never outlives its consumption.
		if (irq_snd_cmd_read)                  snd_cmd_pending <= 1'b0;
		else if (wr_command && bus_byte_en[0]) snd_cmd_pending <= 1'b1;

		// Response P2TALK flag: read-clear (016000 ACCEPTED) has priority over the 6502's set (LS74).
		// FIX (sim-proven): also DRAIN a stale, unconsumed response when the T-11 issues a
		// NEW command. The 6502 posts UNSOLICITED bytes via its $4117 background response-drain (e.g. an
		// 0x55 that lingers after the fire-and-forget cmd 0x19 gain-set at boot). Left in the mailbox, a
		// stale byte becomes data-byte-1 of the next readout (cmd 0x0B high-score table) and shifts the
		// whole 180-byte transfer by one -> the T-11's terminator check reads a data byte ('E'=0x45)
		// instead of 0x55 -> $01C0=04 -> $953e CALL $f5e8 resets the 6502 forever (no in-game sound, no
		// coin credit; confirmed on hardware by the written-vs-read command signature). A new command means the
		// prior transaction is over, so any un-read response is stale: clearing it here resyncs the
		// readout. Verified in sim: the reset loop disappears (err_writes 2->0 over 105M cyc, the game
		// reaches the banked game ROM) with no handshake regression.
		if (clr_cp1_accept || (wr_command && bus_byte_en[0])) resp_pending <= 1'b0;
		else if (irq_snd_resp_write) resp_pending <= 1'b1;

		if (wr_sound_reset && bus_byte_en[0]) snd_reset <= bus_wdata[0];
	end
end

// ---------------------------------------------------------------------------
// Watchdog. A write to 014000 services it; an un-serviced timeout of
// WATCHDOG_TICKS cpu_clk_en ticks pulses watchdog_reset for one clk and re-arms.
// A service write wins a coincident timeout tick. See SP-275 sheet 6B.
// ---------------------------------------------------------------------------
localparam int WATCHDOG_CW = (WATCHDOG_TICKS < 2) ? 1 : $clog2(WATCHDOG_TICKS);
localparam logic [WATCHDOG_CW-1:0] WATCHDOG_LAST =
	WATCHDOG_CW'(WATCHDOG_TICKS - 1);

logic [WATCHDOG_CW-1:0] watchdog_count;

always_ff @(posedge clk) begin
	if (reset) begin
		watchdog_count <= '0;
		watchdog_reset <= 1'b0;
	end else begin
		watchdog_reset <= 1'b0;
		if (wr_watchdog) begin
			watchdog_count <= '0;
		end else if (cpu_clk_en) begin
			if (watchdog_count == WATCHDOG_LAST) begin
				watchdog_count <= '0;
				watchdog_reset <= 1'b1;
			end else begin
				watchdog_count <= watchdog_count + 1'b1;
			end
		end
	end
end

endmodule
