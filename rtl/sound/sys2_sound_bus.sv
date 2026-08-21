// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps
// altera message_off 10268

// Paperboy 6502 sound subsystem.
//
// The real sound 6502 (the T65 core in rtl/lib/T65) runs the actual sound ROM,
// so the T-11's attract/game
// sequencing receives genuine command/response interaction. It implements the
// control flow -- CPU, RAM, ROM, 2804 EEPROM, the inter-CPU command/response
// latches and 0x1840 status, and the periodic 6502 IRQ -- and the real audio
// chips: the YM2151 (jotego's jt51), the two POKEYs (left/right path), and the
// TMS5220C speech chip synthesize sound. The 6502 reads the YM busy/timer flags
// (0x1851) and POKEY registers directly from the chips. Chip audio leaves on
// ym_left/ym_right (16-bit), pokey1_snd/pokey2_snd (6-bit), and tms_snd (14-bit,
// mono) for the top-level mixer.
//
// TMS5220 register semantics (0x1870/0x1872/0x1873) are cross-checked against both MAME's
// atarisy2.cpp (tms5220_w/tms5220_strobe_w, pinned commit d066f16) and the actual Paperboy CPU PCB
// schematic (SP-275 sheet 9A: the 4C/D LS74 flip-flop that generates the WS strobe, D=A0, Q-bar=
// TIWR into the TMS5220's WS pin) -- both independently agree the polarity documented in the
// project's own earlier notes had it backwards: writing 0x1872 sets WS high (idle),
// writing 0x1873 sets WS low (active, latches the byte
// in the 0x1870 data latch). Likewise 0x1840 bit2 is the RAW O_RDYn pin level (TIRDY on the
// schematic, buffered straight through by a non-inverting LS244) -- active HIGH means BUSY/not
// ready, not "ready" as an earlier note (incorrectly) stated; confirmed against MAME's
// tms5220_device::readyq_r() (== !ready_read()), which the same schematic sheet's IN1 bit reads
// directly. RSn is tied high permanently on this board (MAME atarisy2_state::init_paperboy() pins
// rsq_w(1); the schematic shows RS wired straight to +5V) -- Paperboy never reads the TMS5220 back.
// The 0x187a mixer-gain network (all three chips) is decoded here into per-chip Q0.8 gains
// (mix_gain_ym/pk/tms) that the top-level mixer applies -- Stage 1 of the SP-275 two-stage analog
// mixer (per-chip LF13201 resistor networks), so the game's music-ducking-under-speech envelope works.
//
// CPU writes are single-cycle on the 6502 bus (cpu_we/ab/cpu_do), so each chip
// write is captured into a one-deep hold latch and presented to the chip until
// its own clock enable consumes it: POKEY samples on snd_pokey_en, jt51 captures
// `write = !cs_n & !wr_n` on snd_ym_cen_p1 (its mmr runs on cen_p1). The 6502
// (1.778 MHz) writes slower than either chip enable, so a write is always
// consumed before the next arrives.
//
// Sound CPU map (SP-290 sheet 4B):
//   0x0000-0x0fff  4 KiB RAM (mirror 0x2000-0x2fff)
//   0x1000-0x11ff  512-byte 2804 EEPROM (partial-decode mirrors)
//   0x1800-0x180f  POKEY 1 (left path)
//   0x1830-0x183f  POKEY 2 (right path)
//   0x1840         sound switch/status  (P1TALK/P2TALK/coins/self-test/TMS)
//   0x1850-0x1851  YM2151 (jt51) address/data; read = status (busy + timers)
//   0x1860 (R)     main command; read pulses CP0 and clears P1TALK
//   0x1870 (W)     TMS5220C data latch
//   0x1872 (W)     TMS5220C WS strobe high (idle)
//   0x1873 (W)     TMS5220C WS strobe low (active; latches 0x1870 into the chip)
//   0x1874 (W)     response latch; write pulses CP1 and sets P2TALK
//   0x187a (W)     mixer gain (YM/POKEY/TMS) -- decoded into per-chip Q0.8 gains for the mixer
//   0x1876 (W)     coin counters, bits 1:0 (bit1 = CNTR L, bit0 = CNTR R)
//   0x187c (W)     cabinet lamps, bits 3:2 (bit3 = LED2, bit2 = LED1) + TMS clock select, bit5
//   0x187e (W)     sound-chip reset control, bit0 (TMS5220C soft reset)
//   0x1878 (W)     clear periodic 6502 IRQ
//   0x4000-0xffff  48 KiB program ROM (on-chip BRAM via sys2_audio_rom_bram)
//
// snd_cmd_pending drives the 6502 NMI; the periodic IRQ drives the
// regular sound service loop. The 6502 core (T65) advances one bus microcycle
// per snd_cpu_en tick. The program ROM lives in on-chip BRAM
// (sys2_audio_rom_bram at the top level); a fetch of a new address drops
// audio_data_ready for one clk while the registered M10K read settles, which
// gates snd_cpu_en so the 6502 holds its address until the byte is ready.
module sys2_sound_bus #(
	// Periodic 6502 IRQ divider, in clk (32 MHz) ticks. Real hardware asserts at exactly
	// 244.140625 Hz (SP7B-8A divider chain: 20 MHz / 2 / 16 / 16 / 16 / 10); 32 MHz / 131072
	// hits that rate exactly (the old 7332-snd_cpu_en-tick count was 244.108 Hz, -0.013%).
	// Overridable for accelerated sim -- NOTE the value is 32 MHz clocks, not 6502 ticks.
	parameter int unsigned IRQ_DIV = 131072,

	// X2804A byte-write cycle, in clk (32 MHz) ticks. The data sheet gives ~5 ms typical /
	// 10 ms maximum, during which the chip's data outputs are high impedance; 5 ms x 32 MHz =
	// 160000. Software cannot read the cell back until this elapses, which is exactly the
	// "write-completion timing visible to software" the schematics call for. Overridable so
	// a bench can shrink the interval instead of burning 160k cycles per byte -- but note that
	// shortening it also weakens the test, so leave it alone unless the run time demands it.
	parameter int unsigned EEPROM_WRITE_CLKS = 160000
) (
	input  logic        clk,             // sound clock domain (clk_snd)
	input  logic        reset,           // global reset (active high)
	input  logic        snd_cpu_en,      // 6502 clock enable (1.778 MHz)
	input  logic        snd_ym_en,       // YM2151 cen     (3.556 MHz)
	input  logic        snd_ym_cen_p1,   // YM2151 cen_p1  (1.778 MHz; jt51 register-write capture)
	input  logic        snd_pokey_en,    // POKEY enable   (1.778 MHz)

	// Inter-CPU interface to sys2_main_bus (command/response mailbox plus
	// resp_pending, which the 6502 reads as P2TALK in the 0x1840 status byte).
	input  logic        snd_reset,       // 6502 reset level (main 012640 bit0)
	input  logic [7:0]  snd_cmd,         // command byte latched by the T-11
	input  logic        snd_cmd_pending, // P1TALK: command pending (drives NMI)
	input  logic        resp_pending,    // P2TALK: response not yet read by T-11
	output logic [7:0]  snd_resp,        // response byte for the T-11
	output logic        snd_cmd_read,    // pulse: 6502 read the command (CP0)
	output logic        snd_resp_write,  // pulse: 6502 wrote a response (CP1)

	// LETA quadrature inputs (720 / Super Sprint / Championship Sprint / APB).
	// Paperboy has no LETA -- it steers through the ADC0809 on the main bus -- so with
	// leta_present low the 0x1810-0x1813 window reads back as open bus, exactly as an
	// unpopulated socket does.
	input  logic        leta_present,

	// Game id from the MRA descriptor (SYS2_GAME_* in rtl/rom/sys2_rom_layout.vh).
	// Selects the per-game 0x1840 bit map below -- the coin/service bits genuinely move
	// between games, they are not a single wiring with different labels.
	input  logic [7:0]  game_id,
	// TMS5220 fitted? ssprint/csprint have an empty socket. See the status byte.
	input  logic        tms_present,
	input  logic [3:0]  leta_x,
	input  logic [3:0]  leta_y,

	// The LETA's own 156.25 kHz phi2 enable, exported so the quadrature GENERATORS that
	// feed leta_x/leta_y step on exactly the same tick the chip samples on. Two separate
	// dividers of the same nominal rate would be phase-locked today and one refactor away
	// from drifting, and a generator that steps between samples loses counts silently.
	output logic        leta_ce_phi2,

	// Active-low cabinet inputs surfaced in the 0x1840 status byte.
	input  logic        coin1_n,
	input  logic        coin2_n,
	input  logic        coin3_n,
	input  logic        self_test_n,     // main service switch, active low

	// Physical DIP option switches (SP-275 sheet 9B), read by the 6502 through each POKEY's pot
	// lines (ALLPOT). dip_coin -> POKEY1 (coin options, location 6/7A), dip_game -> POKEY2 (game
	// options, 5/6A). Bit order matches the schematic + POKEY P-pins: bit7=switch1 .. bit0=switch8,
	// active-low (1 = switch OFF / line pulled to +5V, 0 = switch ON / grounded).
	input  logic [7:0]  dip_coin,
	input  logic [7:0]  dip_game,

	// On-chip BRAM program ROM read interface (sys2_audio_rom_bram, instantiated at the top
	// level). The 6502 program ROM (0x4000-0xffff, 48 KiB) fits in ~39 M10K and is loaded via a
	// top-level raw-ioctl decode straight into the BRAM -- the same proven pattern the maincpu/
	// char/tile images use -- so it no longer needs the old SDRAM adapter or its clk_t11<->clk_sys
	// read CDC. The BRAM watches audio_rom_addr (= the live 6502 address) and returns the byte on
	// audio_rom_data once audio_data_ready is high; a fetch of a NEW address pulls data_ready low
	// for one clk while the registered M10K read settles, which gates snd_cpu_en (below) so the
	// 6502 holds its address for that clk (<< the snd_cpu_en period, so the stall is invisible).
	output logic [15:0] audio_rom_addr,
	input  logic [7:0]  audio_rom_data,
	input  logic        audio_data_ready,

	// Factory EEPROM load (top-level raw-ioctl decode of ROM-stream 0x102000-0x1021ff). Without
	// this the 2804 EEPROM powers up blank -> the game reads garbage coinage and "INSERT COIN"
	// never credits. Loaded during download while the 6502 is held in reset (no write contention).
	// The SAME port also carries the NVRAM restore (top level ORs in the <nvram index=2> stream),
	// so a prior save always overwrites the factory fill when one exists.
	input  logic        eeprom_ld_we,
	input  logic [8:0]  eeprom_ld_addr,
	input  logic [7:0]  eeprom_ld_data,

	// NVRAM save-back (top-level <nvram index="2" size="512"/>): eeprom_wr pulses whenever the
	// 6502 actually writes a settings/high-score byte, marking the save dirty; eeprom_dump_data
	// is a combinational readback of the live EEPROM contents at eeprom_dump_addr, sampled by the
	// top level while HPS streams the upload (MiSTer paces the address, no read strobe needed).
	output logic        eeprom_wr,
	input  logic [8:0]  eeprom_dump_addr,
	output logic [7:0]  eeprom_dump_data,

	// Electromechanical coin counters (0x1876) and the two cabinet lamps (0x187c bits 3:2).
	// SP-275 sheet 8B: the COUNTERS strobe clocks two LS74s at 7H, bit1 -> CNTR L (J15 pin 5)
	// and bit0 -> CNTR R (J15 pin 9), each through a 1K base resistor into a 2N6044 Darlington;
	// the LEDS strobe clocks an LS175 at 5F, bit2 -> LED1 (J106 pin 4) and bit3 -> LED2 (J106
	// pin 3) through a 2N3904 + 220 ohm. All are active high and cleared by P2RESET (= our
	// reset|snd_reset), matching the board's "0 on main sound reset". MiSTer has
	// no counter/lamp pins, so the top level leaves these open -- they exist so the register
	// state is modelled rather than the write being silently dropped.
	output logic [1:0]  coin_counter,   // [1] = CNTR L, [0] = CNTR R
	output logic [1:0]  lamp,           // [1] = LED2,   [0] = LED1

	// Audio chip outputs, summed into AUDIO_L/R by the top-level mixer (like the reference
	// ATARISYS1 p_volmux): YM2151 full-resolution stereo, and each POKEY's 6-bit waveform
	// (POKEY 1 -> left path, POKEY 2 -> right path).
	output logic signed [15:0] ym_left,
	output logic signed [15:0] ym_right,
	output logic signed [5:0]  pokey1_snd,
	output logic signed [5:0]  pokey2_snd,
	output logic signed [13:0] tms_snd,

	// 0x187a programmable analog-mixer gains (Stage 1: the per-chip LF13201 resistor networks on
	// SP-275 sheets 9A/9B). Each is a Q0.8 unsigned fraction (256 = unity, no attenuation), applied
	// to the corresponding chip term by the top-level mixer. Decoded live from the register the 6502
	// writes, so the game's own volume/ducking envelope (e.g. music ducking under speech) takes
	// effect. See the 0x187a decode block below for the resistor->gain model (MAME atarisy2 mixer_w).
	output logic [8:0]         mix_gain_ym,   // bits 0-2 (100k/47k/22k)
	output logic [8:0]         mix_gain_pk,   // bits 3-4 (100k/47k, R129/R130), shared by both POKEYs
	output logic [8:0]         mix_gain_tms   // bits 5-7 (100k/47k/22k)
);


// SYS2_GAME_* ids for the per-game 0x1840 map below.
`include "rtl/rom/sys2_rom_layout.vh"

// ---------------------------------------------------------------------------
// Sound 6502 (T65) + bus. This section wires the T65 core (instantiated below)
// to the sound RAM, EEPROM, and program ROM, and to the audio chips
// (YM2151/POKEY/TMS5220), which are real cores -- not stubs -- decoded below.
// ---------------------------------------------------------------------------
logic [15:0] ab;
logic [7:0]  cpu_do;
logic        cpu_we;
logic        cpu_read;
logic [7:0]  di;
logic        cpu_irq;
logic        cpu_nmi;

// ROM-fetch stall: when the 6502 reads program ROM and the BRAM has not yet returned the byte
// (audio_data_ready low), suppress the clock-enable so the 6502 holds its address and waits for
// the read. Non-ROM accesses (RAM/IO/status/cmd) keep audio_data_ready high in the BRAM ROM, so
// they never stall. The periodic-IRQ divider counts raw 32 MHz clocks (below),
// so wall-clock IRQ timing is unaffected by fetch stalls.
wire snd_cpu_en_g = snd_cpu_en & audio_data_ready;

// snd_cmd_pending (P1TALK) is the 6502 NMI source (rising edge -> NMI), captured by the core.
assign cpu_nmi = snd_cmd_pending;

// --- Sound CPU: T65, a complete cycle-accurate NMOS 6502 (rtl/lib/T65), the same core the reference
//     Atari System 1 core uses for its sound. ALL toolchains now run T65: Quartus/ModelSim compile the
// VHDL directly; Verilator uses a GHDL-generated netlist of the same core -- so sim == silicon
//     Its address bus is REGISTERED, so there is no combinational AB->DI->AB loop with the BRAM bus
//     (the fatal flaw of Arlet's core). Enable = snd_cpu_en_g: the CPU advances only when the ROM byte
//     is ready, and A holds while Enable=0 so the registered BRAM read settles. BCD_en=1 -> decimal. ---
wire [23:0] t65_a;
wire        t65_rwn;
assign ab       = t65_a[15:0];
assign cpu_we   = ~t65_rwn & snd_cpu_en_g;   // 1-clk write strobe at the advance
assign cpu_read =  t65_rwn & snd_cpu_en_g;   // 1-clk read  strobe at the advance
T65 u_cpu (
	.Mode    (2'b00),
	.BCD_en  (1'b1),
	.Res_n   (~(reset | snd_reset)),
	.Clk     (clk),
	.Enable  (snd_cpu_en_g),
	.Rdy     (1'b1),
	.Abort_n (1'b1),
	.IRQ_n   (~cpu_irq),
	.NMI_n   (~cpu_nmi),
	.SO_n    (1'b1),
	.A       (t65_a),
	.DI      (di),
	.DO      (cpu_do),
	.R_W_n   (t65_rwn),
	.Sync    (), .EF (), .MF (), .XF (), .ML_n (), .VP_n (),
	// DEBUG (a VHDL record) is omitted, not connected open: GHDL --synth expands it into escaped
	// scalar ports (\DEBUG[I] ...), which have no single `DEBUG` to bind. Omitting
	// leaves it unconnected for BOTH the VHDL T65 (open output) and the generated Verilog T65.
	.VDA     (), .VPA (), .NMI_ack ()
);

// ---------------------------------------------------------------------------
// Address decode (canonical addresses + the RAM/EEPROM data mirrors that the
// ROM actually relies on; audio-chip partial-decode mirrors are not needed for
// control flow).
// ---------------------------------------------------------------------------
wire sel_rom    = (ab >= 16'h4000);
wire sel_ram    = (ab < 16'h1000) || (ab >= 16'h2000 && ab < 16'h3000);
wire sel_eeprom = (ab >= 16'h1000 && ab < 16'h1200) ||
                  (ab >= 16'h3000 && ab < 16'h3200);
wire sel_pokey1 = (ab >= 16'h1800 && ab < 16'h1810);   // POKEY 1, left path
wire sel_pokey2 = (ab >= 16'h1830 && ab < 16'h1840);   // POKEY 2, right path
// LETA counters. Schematic SP-290 Sheet 4B: 0x1810-0x1813, read only. Note this sits
// between the two POKEYs, and is NOT 0x1850 -- that is the YM2151.
wire sel_leta   = (ab >= 16'h1810 && ab < 16'h1814);
wire sel_status = (ab == 16'h1840);
wire sel_cmd    = (ab == 16'h1860);
wire sel_resp   = (ab == 16'h1874);
wire sel_irqclr = (ab == 16'h1878);
wire sel_ym     = (ab == 16'h1850 || ab == 16'h1851);
wire sel_tms_d  = (ab == 16'h1870);   // TMS5220C data latch
wire sel_tms_ws0= (ab == 16'h1872);   // WS -> high (idle)
wire sel_tms_ws1= (ab == 16'h1873);   // WS -> low  (active, latches sel_tms_d's byte)
wire sel_coin   = (ab == 16'h1876);   // coin counters, bits 1:0 (SP-275 sheet 8B)
wire sel_gain   = (ab == 16'h187a);   // audio mixer gain network (YM/POKEY/TMS)
wire sel_switch = (ab == 16'h187c);   // lamps (3:2) + LETA resolution (4) + TMS clk (5)
wire sel_sndrst = (ab == 16'h187e);   // sound-chip reset control bit0

// ---------------------------------------------------------------------------
// Sound RAM (4 KiB) and 2804 EEPROM (512 B). Both are loaded by the download
// path. Reads are SYNCHRONOUS (registered output) so Quartus infers M10K block
// RAM instead of spilling to ~37k flip-flops + a ~17k-ALUT async read mux (which
// was a latent on-silicon reliability risk for the song-sequencer state that lives
// here). Safe because the T65 holds its registered
// address for the whole snd_cpu_en window (~18 clk at /18) and RAM never stalls
// (audio_data_ready high), so the 1-cycle registered read is valid many cycles
// before the CPU samples DI -- exactly like the program-ROM read.
// ---------------------------------------------------------------------------
logic [7:0] ram    [0:4095];
logic [7:0] eeprom [0:511];
logic [7:0] ram_q;      // registered RAM read (M10K output reg)
logic [7:0] eeprom_q;   // registered EEPROM read

wire [11:0] ram_a = ab[11:0];
wire [8:0]  eep_a = ab[8:0];
wire        eeprom_cpu_we = cpu_we && sel_eeprom;

always_ff @(posedge clk) begin
	if (cpu_we && sel_ram)         ram[ram_a]    <= cpu_do;
	// Factory EEPROM load wins during download; otherwise the 6502 owns the EEPROM (settings,
	// high scores). The two never collide -- the 6502 is held in reset while ioctl is loading.
	if (eeprom_ld_we)              eeprom[eeprom_ld_addr] <= eeprom_ld_data;
	else if (eeprom_cpu_we)        eeprom[eep_a]          <= cpu_do;
	// Registered (synchronous) reads -> M10K inference. The address is held across the
	// snd_cpu_en window, so ram_q/eeprom_q are valid long before snd_cpu_en_g samples DI.
	ram_q    <= ram[ram_a];
	eeprom_q <= eeprom[eep_a];
end

// eeprom_wr marks the NVRAM save dirty (top level). eeprom_dump_data is a second,
// independent combinational read port for the upload -- separate from eep_a/eeprom_q so
// reading back for a save never disturbs the CPU's live registered read. It is deliberately
// NOT gated by the write-busy window below: that models the chip's data pins, whereas this
// port is the HPS reading our array, which has no physical equivalent on the PCB.
assign eeprom_wr         = eeprom_cpu_we;
assign eeprom_dump_data  = eeprom[eeprom_dump_addr];

// ---------------------------------------------------------------------------
// X2804A write cycle. The board requires that we "preserve byte writes, readback,
// write-completion timing visible to software"; until now a write committed instantly and the
// data pins never went away, which is the one part of the EEPROM contract we did not model.
//
// Real part: a byte write takes ~5 ms typical / 10 ms max, and during that internal programming
// interval the chip's data outputs are high impedance. The X2804A has NO data-polling bit and no
// toggle bit (those came with later EEPROMs), so the documented way for software to wait is to
// re-read a location whose programmed value contains zero bits until the comparison succeeds --
// which works precisely because the floating bus does not read back as the stored value. There is
// also NO unlock/write-enable latch on this board: the sound-board schematic ties the EEPROM write
// enable straight to the decoded processor write, so any write in the window lands.
//
// We keep committing the byte immediately and instead gate the READ during the busy window. That
// is behaviourally identical from the CPU's side -- while the outputs are high-Z software cannot
// observe either the old or the new value -- and it avoids carrying a pending address/data pair.
// The bus floats high on this board, so a busy read returns 0xff, the same open-bus convention the
// default read mux already uses.
//
// MAME's generic parallel-EEPROM device does not reproduce this, so this is a PCB-fidelity detail
// that emulation comparison would not have surfaced.
//
// Why this is safe for paperboy (verified in the sound ROM, not assumed): the game never writes the
// EEPROM directly. `$6947` pushes {addr_lo, addr_hi = #$10 + $5e, value} onto a 5-entry circular
// queue at `$0d88`, and `$6470` drains ONE byte per call -- `sta ($54),y` at `$6486` -- with NO
// read-back or verify afterwards. So nothing ever polls a cell during its write cycle, and the
// software already paces writes far more slowly than 5 ms. (This also confirms the decode window:
// the pointer high byte is `#$10 + $5e`, and for a 512-byte device $5e is 0 or 1, so accesses stay
// inside $1000-$11ff.)
//
// Known limitation: a real X2804A IGNORES a write issued while it is still programming, whereas we
// restart the interval and commit anyway. Modelling the write-abort would risk silently dropping
// high-score bytes in a way the real board would too -- but only if the game ever wrote that fast,
// and the queue above shows it does not. Left generous deliberately.
localparam int EE_BUSY_CW = (EEPROM_WRITE_CLKS < 2) ? 1 : $clog2(EEPROM_WRITE_CLKS);
logic [EE_BUSY_CW-1:0] ee_busy_cnt;
logic                  ee_busy;

always_ff @(posedge clk) begin
	if (reset || snd_reset) begin
		ee_busy     <= 1'b0;
		ee_busy_cnt <= '0;
	end else if (eeprom_cpu_we) begin
		// A write during a write restarts the interval, like re-triggering the internal timer.
		ee_busy     <= 1'b1;
		ee_busy_cnt <= EE_BUSY_CW'(EEPROM_WRITE_CLKS - 1);
	end else if (ee_busy) begin
		if (ee_busy_cnt == '0) ee_busy <= 1'b0;
		else                   ee_busy_cnt <= ee_busy_cnt - 1'b1;
	end
end

// ---------------------------------------------------------------------------
// Program ROM (0x4000-0xffff): served by the on-chip BRAM (sys2_audio_rom_bram) at the top
// level. Present the live 6502 address while it addresses ROM; park at the ROM base otherwise,
// so the BRAM never indexes out of range (its `addr - 0x4000` offset underflows for RAM/IO
// addresses) and non-ROM accesses never dip data_ready (the presented address holds steady
// across non-ROM streaks, restoring the "non-ROM never stalls" contract structurally). The BRAM
// returns the byte on audio_rom_data and gates the CPU via audio_data_ready (see snd_cpu_en_g
// above). data_ready guarantees audio_rom_data holds the byte at audio_rom_addr on every gated
// tick, so the di mux below can use it directly.
// ---------------------------------------------------------------------------
assign audio_rom_addr = sel_rom ? ab : 16'h4000;

// ---------------------------------------------------------------------------
// Audio chips: YM2151 (jt51) + two POKEYs. The 6502 (T65) presents a write
// for one clk only (cpu_we/ab/cpu_do valid together), so each chip write is held
// in a one-deep latch until the chip's own enable consumes it. The 6502 writes
// at 1.778 MHz; jt51 captures on snd_ym_cen_p1 (1.778 MHz) and each POKEY on
// snd_pokey_en (1.778 MHz), so the latch is empty again before the next write.
// ---------------------------------------------------------------------------
logic       pk1_wr_pend, pk2_wr_pend, ym_wr_pend;
logic [3:0] pk1_wr_a, pk2_wr_a;
logic [7:0] pk1_wr_d, pk2_wr_d;
logic       ym_wr_a0;
logic [7:0] ym_wr_d;

always_ff @(posedge clk) begin
	if (reset || snd_reset) begin
		pk1_wr_pend <= 1'b0;
		pk2_wr_pend <= 1'b0;
		ym_wr_pend  <= 1'b0;
	end else begin
		if (cpu_we && sel_pokey1) begin
			pk1_wr_pend <= 1'b1; pk1_wr_a <= ab[3:0]; pk1_wr_d <= cpu_do;
		end else if (snd_pokey_en) begin
			pk1_wr_pend <= 1'b0;
		end

		if (cpu_we && sel_pokey2) begin
			pk2_wr_pend <= 1'b1; pk2_wr_a <= ab[3:0]; pk2_wr_d <= cpu_do;
		end else if (snd_pokey_en) begin
			pk2_wr_pend <= 1'b0;
		end

		if (cpu_we && sel_ym) begin
			ym_wr_pend <= 1'b1; ym_wr_a0 <= ab[0]; ym_wr_d <= cpu_do;
		end else if (snd_ym_cen_p1) begin
			ym_wr_pend <= 1'b0;
		end
	end
end

// --- LETA: four 8-bit quadrature counters (sys2_leta). phi2 is 156.25 kHz per SP-290
// Sheet 9B (20 MHz / 128), NOT the 6502 clock.
//
// EXACT fractional enable, not an integer divide: 156.25 kHz is 5/1024 of 32 MHz exactly,
// where 32_000_000/156_250 truncates to 204 and runs 0.4 % fast. This is the same
// mechanism (and the same class of mistake) as the TMS enables just below, where integer
// /51 and /38 were replaced for being +0.4 % and +1.1 % off.
logic ce_leta;
assign leta_ce_phi2 = ce_leta;
sys2_frac_cen #(.NUM(5), .DEN(1024)) u_leta_cen (
	.clk(clk), .reset(reset || snd_reset), .cen(ce_leta)
);

// Declared HERE, before its use in the instance below: sys2_quad_gen documents the trap --
// ModelSim rejects the forward references Verilator and Quartus tolerate.
logic      leta_resolution;   // 0x187C bit 4
wire [7:0] leta_dout;
sys2_leta leta_inst (
	.clk        (clk),
	.ce_phi2    (ce_leta),
	.reset      (reset),
	.quad_x     (leta_x),
	.quad_y     (leta_y),
	.resolution (leta_resolution),
	.sel        (ab[1:0]),
	.dout       (leta_dout)
);

// --- YM2151 (jotego jt51). write = !cs_n & !wr_n captured on cen_p1. dout = status
//     (bit7 busy, bit1/0 timer flags) which the 6502 polls at 0x1851. ---
wire [7:0] ym_dout;

jt51 u_jt51 (
	.rst    (reset | snd_reset),
	.clk    (clk),
	.cen    (snd_ym_en),
	.cen_p1 (snd_ym_cen_p1),
	.cs_n   (~ym_wr_pend),
	.wr_n   (~ym_wr_pend),
	.a0     (ym_wr_pend ? ym_wr_a0 : ab[0]),
	.din    (ym_wr_d),
	.dout   (ym_dout),
	.ct1    (),
	.ct2    (),
	.irq_n  (),                 // Paperboy's 6502 IRQ is the 244 Hz divider, not the YM timer
	.sample (),
	.left   (),
	.right  (),
	.xleft  (ym_left),
	.xright (ym_right)
);

// --- POKEY 1 (left) and POKEY 2 (right). Held in write mode while a write is
//     pending; otherwise in read mode when selected so DOUT reflects ab[3:0]. ---
wire [7:0] pk1_dout, pk2_dout;

POKEY u_pokey1 (
	.ADDR      (pk1_wr_pend ? pk1_wr_a : ab[3:0]),
	.DIN       (pk1_wr_d),
	.DOUT      (pk1_dout),
	.DOUT_OE_L (),
	.RW_L      (~pk1_wr_pend),                 // 0 = write (pending), 1 = read
	.CS        (1'b1),
	.CS_L      (~(pk1_wr_pend | sel_pokey1)),  // assert for pending write or live read
	.AUDIO_OUT (pokey1_snd),
	.PIN       (dip_coin),              // coin-option DIP (6/7A) read via POKEY1 ALLPOT
	.ENA       (snd_pokey_en),
	.CLK       (clk)
);

POKEY u_pokey2 (
	.ADDR      (pk2_wr_pend ? pk2_wr_a : ab[3:0]),
	.DIN       (pk2_wr_d),
	.DOUT      (pk2_dout),
	.DOUT_OE_L (),
	.RW_L      (~pk2_wr_pend),
	.CS        (1'b1),
	.CS_L      (~(pk2_wr_pend | sel_pokey2)),
	.AUDIO_OUT (pokey2_snd),
	.PIN       (dip_game),              // game-option DIP (5/6A) read via POKEY2 ALLPOT
	.ENA       (snd_pokey_en),
	.CLK       (clk)
);

// ---------------------------------------------------------------------------
// TMS5220C speech chip. Data path: 0x1870 latches a byte into tms_dbus (the schematic's LS273
// 8D, cleared by SNDRST); the WS register (the schematic's LS74 4C/D, D=A0) is set high (idle) by
// a write to 0x1872 and set low (active) by a write to 0x1873 -- the falling edge latches tms_dbus
// into the chip's Speak-External FIFO or (in command mode) decodes it as a command. RSn is tied
// high permanently (this board never reads the TMS5220 back). Clock: the vendored core's I_ENA is
// a clock-enable gate at the TMS5220's real ~625/833 kHz rate (0x187c bit5 selects, matching the
// PCB's 12E divider) -- I_OSC is tied to the full-rate system clock like the other chip ENA ports.
// Reset: the model has no dedicated reset pin (matching the real chip, which is reset only by a
// command byte or by WSn+RSn both low); 0x187e bit0 (the sound-chip reset control, written by the
// 6502 itself) is leveled into that documented WSn+RSn-both-low hardware reset path, alongside the
// global reset/snd_reset, so it works regardless of whether the chip is mid-speech (DDIS=1, where a
// RESET *command* byte would otherwise just be swallowed as more FIFO data).
//
// POLARITY (fixed; confirmed on hardware by "music+SFX play but no speech"): 0x187e bit0 is
// active-low -- bit0=1 RUNS the chip, bit0=0 asserts reset. The boot ROM pulses this line
// 0xff -> 0x00 -> 0xff and then LEAVES it at 0xff for the rest of the run (verified against the real
// romset in MAME 0.288: the 6502 writes 0x187e only at $4007/$400c/$4011, final value 0xff, and MAME
// renders speech in that state). The original wiring here treated bit0=1 as reset-asserted, which
// held the TMS in a permanent WSn+RSn-both-low hardware reset for the entire game -- silencing ONLY
// speech (the YM2151 and POKEYs are not gated by this line), exactly the observed symptom. An
// earlier note flagged this active-high guess as unconfirmed ("revisit if HW testing shows the
// reset behavior disagrees" / "the PCB's TMS reset path may feed a stream of 0xff") -- it did, so we
// invert it. Default is reset-asserted so the chip is held until the 6502 releases it at boot.
logic [7:0] tms_dbus;
logic       tms_wsn;
logic       tms_clk_hi;    // 0x187c bit5: 0 -> ~625 kHz, 1 -> ~833 kHz
logic       tms_ctl_runn;  // 0x187e bit0, held: 1 = run, 0 = reset asserted (active-low)

always_ff @(posedge clk) begin
	if (reset || snd_reset) begin
		tms_dbus     <= 8'h00;
		tms_wsn      <= 1'b1;
		tms_clk_hi   <= 1'b0;
		leta_resolution <= 1'b1;   // 2x is the mode 720 needs; see sys2_leta
		tms_ctl_runn <= 1'b0;   // reset-asserted at power-on; the 6502 releases it (0x187e bit0=1) at boot
		coin_counter <= 2'b00;  // LS74 7H cleared by P2RESET
		lamp         <= 2'b00;  // LS175 5F cleared by P2RESET
	end else begin
		if (cpu_we && sel_tms_d)   tms_dbus     <= cpu_do;
		if (cpu_we && sel_tms_ws0) tms_wsn      <= 1'b1;
		if (cpu_we && sel_tms_ws1) tms_wsn      <= 1'b0;
		if (cpu_we && sel_switch)  tms_clk_hi   <= cpu_do[5];
		// 0x187C bit 4 = "LETA Resolution" (SP-290 Sheet 4B names it; the sheet does
		// not show what it changes and MAME does not model it -- see sys2_leta).
		if (cpu_we && sel_switch)  leta_resolution <= cpu_do[4];
		if (cpu_we && sel_sndrst)  tms_ctl_runn <= cpu_do[0];
		if (cpu_we && sel_coin)    coin_counter <= cpu_do[1:0];
		if (cpu_we && sel_switch)  lamp         <= cpu_do[3:2];
	end
end

// Clock enables for the TMS5220's I_ENA, selectable between the PCB's two EXACT rates
// (normal 20 MHz/4/4/2 = 625.000 kHz; alternate 20 MHz/4/3/2 = 833.333 kHz).
// Each is a fractional enable off the 32 MHz system clock (sys2_frac_cen, the same
// mechanism as the CPU/ADC/YM enables): 5/256 = 625.000 kHz and 5/192 = 833.333 kHz -- exact
// average rates with <=1-clk jitter, replacing the former integer /51 (+0.4%) and /38 (+1.1%)
// approximations. Both streams run continuously; 0x187c bit5 selects which one gates the chip
// (modelling the PCB's 12E divider-tap mux).
logic tms_cen_lo, tms_cen_hi;
sys2_frac_cen #(.NUM(5), .DEN(256)) u_tms_cen_lo (.clk(clk), .reset(reset || snd_reset), .cen(tms_cen_lo));
sys2_frac_cen #(.NUM(5), .DEN(192)) u_tms_cen_hi (.clk(clk), .reset(reset || snd_reset), .cen(tms_cen_hi));
wire tms_clk_en = tms_clk_hi ? tms_cen_hi : tms_cen_lo;

// Synthetic chip reset: hold both WSn and RSn low, the TMS5220's documented external hardware-reset
// method (the vendored model implements it unconditionally, unlike the DDIS-gated RESET command).
wire tms_force_rst = reset || snd_reset || ~tms_ctl_runn;
wire tms_wsn_pin    = tms_force_rst ? 1'b0 : tms_wsn;
wire tms_rsn_pin    = tms_force_rst ? 1'b0 : 1'b1;

wire [7:0]         tms_dbus_out;  // unused: this board never reads the TMS5220 back
wire               tms_rdyn;      // O_RDYn, raw (active-high = busy/not-ready) -> status[2]
wire               tms_intn;      // unused: TMS5220 IRQ is not wired to the 6502 on this board
wire signed [13:0] tms_spkr;

TMS5220 u_tms5220 (
	.I_OSC   (clk),
	.I_ENA   (tms_clk_en),
	.I_WSn   (tms_wsn_pin),
	.I_RSn   (tms_rsn_pin),
	.I_DATA  (1'b1),
	.I_TEST  (1'b1),
	.I_DBUS  (tms_dbus),
	.O_DBUS  (tms_dbus_out),
	.O_RDYn  (tms_rdyn),
	.O_INTn  (tms_intn),
	.O_M0    (), .O_M1  (), .O_ADD8 (), .O_ADD4 (), .O_ADD2 (), .O_ADD1 (), .O_ROMCLK (),
	.O_T11   (), .O_IO  (), .O_PRMOUT (),
	.O_SPKR  (tms_spkr)
);
assign tms_snd = tms_spkr;

// ---------------------------------------------------------------------------
// 0x187a audio mixer gain network (SP-275 sheets 9A/9B). Stage 1 of the two-stage analog mixer:
// each chip's LM324 op-amp has a switched feedback network of LF13201 analog switches ("MIXER0-7")
// selecting 100k/47k/22k resistors. The controls are active-low -- a register bit = 0 ENGAGES that
// resistor (pulls the gain down); all bits = 1 -> no resistor engaged -> unity. Reset value is 0
// (all resistors engaged = minimum gain), matching the sound-reset default on hardware; the 6502
// programs it during boot ($40fb writes 0x1f) and re-writes it per speech phrase (the $56xx gain
// routine) to duck the music under speech and set the speech level -- so modelling the register
// faithfully makes that envelope work, instead of running every chip at a fixed full scale.
//
// The resistor->gain mapping replicates MAME atarisy2.cpp mixer_w (the de-facto reference, which the
// SP-275 sheets corroborate): rtop = parallel of that chip's resistor set; the engaged (bit=0)
// resistors form rbottom; gain = Rengaged / (rtop + Rengaged), i.e. 0.5 (all engaged) .. 1.0 (none).
// Values are precomputed to Q0.8 (x256, 256 = unity) LUTs; the ground-truth register values
// (0x1f/0x1e) were captured from the real romset running in MAME 0.288.
//   YM  bits 0-2 : bit0=100k, bit1=47k, bit2=22k
//   POKEY bits 3-4: bit3=100k(R129), bit4=47k(R130)  (both POKEYs share this gain)
//   TMS bits 5-7 : bit5=100k, bit6=47k, bit7=22k
//   (POKEY resistors verified on SP-275 sheet 9B. The game only ever leaves these bits
//    open (0x1f/0x1e) = unity anyway, so the POKEY attenuator is never engaged in play.)
logic [7:0] reg_187a;
always_ff @(posedge clk) begin
	if (reset || snd_reset) reg_187a <= 8'h00;   // "0 on sound reset"
	else if (cpu_we && sel_gain) reg_187a <= cpu_do;
end

// 3-bit (YM/TMS) and 2-bit (POKEY) resistor-network gain LUTs, Q0.8.
// CORRECTION. The tables below were derived from the
// wrong network model and have been recomputed from the SP-275 schematic. The mixer node is a
// T-attenuator: R132/R74/R72 = 100K IN from the preamp, R137/R124/R120 = 100K OUT to the next
// stage's virtual ground (the second LM324 is an inverting amp with its + pin grounded), and the
// switched resistors shunt that node to ground. Node analysis with Rs = Rout = 100K gives
//
//     relative gain (normalised to no shunt engaged) = 0.02 / (0.02 + 1/Rshunt[kohm])
//
// The OLD tables implemented gain = Rengaged/(rtop + Rengaged) with rtop = the parallel of all that
// path's shunts (13.03K / 31.97K) -- i.e. they used the shunt resistors themselves as the series
// element. That construction forces Rengaged == rtop when everything is engaged, so every path
// landed on exactly 0.500; that round number was the tell. The real network reaches 0.2068.
function automatic logic [8:0] gain3 (input logic [2:0] code);
	case (code)                     // code = {22k_bit, 47k_bit, 100k_bit}; bit=0 engages
		3'b000: gain3 = 9'd53;      // all engaged        -> 0.2068
		3'b001: gain3 = 9'd59;      // 47k+22k            -> 0.2306
		3'b010: gain3 = 9'd68;      // 100k+22k           -> 0.2651
		3'b011: gain3 = 9'd78;      // 22k                -> 0.3056
		3'b100: gain3 = 9'd100;     // 100k+47k           -> 0.3900
		3'b101: gain3 = 9'd124;     // 47k                -> 0.4845
		3'b110: gain3 = 9'd171;     // 100k               -> 0.6667
		3'b111: gain3 = 9'd256;     // none engaged       -> 1.0000 (unity)
	endcase
endfunction
function automatic logic [8:0] gain2 (input logic [1:0] code);
	case (code)                     // code = {47k_bit(bit4), 100k_bit(bit3)}; bit=0 engages
		2'b00: gain2 = 9'd100;      // both engaged       -> 0.3900
		2'b01: gain2 = 9'd124;      // 47k (bit4)         -> 0.4845
		2'b10: gain2 = 9'd171;      // 100k (bit3)        -> 0.6667
		2'b11: gain2 = 9'd256;      // none engaged       -> 1.0000 (unity)
	endcase
endfunction

assign mix_gain_ym  = gain3(reg_187a[2:0]);
assign mix_gain_pk  = gain2(reg_187a[4:3]);
assign mix_gain_tms = gain3(reg_187a[7:5]);

// ---------------------------------------------------------------------------
// 0x1840 status byte. Active-high P1TALK/P2TALK/TMS-busy; active-low
// self-test/coins (idle = high). bit3 unused -> 0.
//
// The top three bits and bit 2 are per-game. MAME's `PORT_MODIFY("IN1")` per game
// (atarisy2.cpp) is the authority:
//
//              bit7    bit6    bit5       bit2
//   paperboy   COIN2   COIN1   SERVICE1   TMS ready
//   720        COIN2   COIN1   SERVICE1   TMS ready
//   ssprint    COIN3   COIN2   COIN1      unused
//   csprint    COIN2   COIN1   unused     unused
//   apb        COIN2   COIN1   SERVICE1   TMS ready
//
// The authority moved, and this comment did not follow it at first. MAME commit
// `76f3936922` ("paperboy: coin3 is service coin") reclassified the base IN1 bit
// 0x20 from `IPT_COIN3` to `IPT_SERVICE1` and deleted APB's redundant override, so the
// paperboy and apb rows above said COIN3 under a claim of verbatim transcription. The LOGIC
// was already right on all five rows -- `coin3_n` carries the coin-door service-credit button
// for the games that have one -- so only the comment misled. A vendored snapshot of MAME's
// source predates that commit; never cite one as current without re-fetching (its mtime is when it
// was COPIED, not the source date).
// csprint's `st_b5 = 1'b1` tie-off is confirmed correct and is NOT the same gap: csprint
// `PORT_INCLUDE`s ssprint, not paperboy, then restores `0x20` to `IPT_UNUSED` explicitly.
//
// bit 2 with no TMS fitted reads 1, NOT 0. ssprint/csprint redefine it as
// `IP_ACTIVE_LOW, IPT_UNUSED`, and an active-low unused bit sits at its INACTIVE state,
// which is high. That matches the board: Sheet 4B still lists ti ready on ssprint because
// the decode exists, and the LS244 buffering the absent TMS's O_RDYn floats its input
// high. Gating this to 0 -- the obvious reading of "no TMS" -- would present the chip as
// permanently ready rather than permanently not-ready, the opposite of the hardware.
//
// An unused active-low bit is 1 for the same reason.
// ---------------------------------------------------------------------------
wire st_tms = tms_present ? tms_rdyn   // raw O_RDYn, active high = busy
                          : 1'b1;      // empty socket: LS244 input floats high

logic st_b7, st_b6, st_b5;
always_comb begin
	// Default = the Paperboy/APB map, which is also MAME's unmodified IN1.
	st_b7 = coin2_n; st_b6 = coin1_n; st_b5 = coin3_n;
	case (game_id)
		SYS2_GAME_SSPRINT: begin st_b7 = coin3_n; st_b6 = coin2_n; st_b5 = coin1_n; end
		SYS2_GAME_CSPRINT: begin st_b7 = coin2_n; st_b6 = coin1_n; st_b5 = 1'b1;    end
		// 720 puts SERVICE1 on bit5; the top-level feeds it on coin3_n.
		default: ;
	endcase
end

wire [7:0] status = { st_b7, st_b6, st_b5, self_test_n,
                      1'b0, st_tms, resp_pending, snd_cmd_pending };

// ---------------------------------------------------------------------------
// CPU read data mux. The YM (0x1851) returns the live jt51 status (bit7 busy,
// bits1:0 timer A/B flags); each POKEY returns its selected register (RANDOM,
// etc.). Remaining stubbed I/O returns 0xff.
// ---------------------------------------------------------------------------
always_comb begin
	if      (sel_rom)    di = audio_rom_data;
	else if (sel_ram)    di = ram_q;       // registered (M10K) RAM read
	else if (sel_eeprom) di = ee_busy ? 8'hff : eeprom_q;  // high-Z (open bus) during the write cycle
	else if (sel_pokey1) di = pk1_dout;
	else if (sel_pokey2) di = pk2_dout;
	else if (sel_status) di = status;
	else if (sel_cmd)    di = snd_cmd;
	else if (sel_ym)     di = ym_dout;     // live YM status (busy + timer flags)
	else if (sel_leta && leta_present) di = leta_dout;
	else                 di = 8'hff;
end

// Periodic-IRQ state, driven by the IRQ block below.
localparam int IRQW = (IRQ_DIV < 2) ? 1 : $clog2(IRQ_DIV);
logic [IRQW-1:0] irq_count;
logic            irq_pending;


// ---------------------------------------------------------------------------
// Inter-CPU comm strobes. A read of 0x1860 acknowledges the command (CP0); a
// write of 0x1874 latches the response (CP1).
// ---------------------------------------------------------------------------
always_ff @(posedge clk) begin
	// The comm STROBES are momentary; they stay quiet while the 6502 is held in reset.
	if (reset || snd_reset) begin
		snd_cmd_read   <= 1'b0;
		snd_resp_write <= 1'b0;
	end else begin
		snd_cmd_read   <= cpu_read && sel_cmd;
		snd_resp_write <= cpu_we && sel_resp;
	end
	// The response BYTE is the PCB's LS374 latch: it is NOT erased by /SNDRST (snd_reset) -- only a
	// full game reset clears it. The 6502 latches it via the 0x1874 write; it holds otherwise. (The
	// 6502 is held in reset during snd_reset, so cpu_we is quiet -- snd_resp just holds.) Keeping the
	// byte across snd_reset matches the hardware mailbox so the T-11 never reads a flag/byte that
	// disagree (flag=pending, byte=0) after a sound-CPU reset.
	if (reset)                   snd_resp <= 8'h00;
	else if (cpu_we && sel_resp) snd_resp <= cpu_do;
end

// ---------------------------------------------------------------------------
// Periodic 6502 IRQ: asserted every IRQ_DIV clk (32 MHz) ticks -- at the default
// 131072 that is exactly the PCB's 244.140625 Hz divider-chain rate -- cleared by
// a write to 0x1878. Level-sensitive into the core's IRQ input. A coincident
// set beats the clear, as before. (irq_count/irq_pending are declared above.)
// ---------------------------------------------------------------------------
always_ff @(posedge clk) begin
	if (reset || snd_reset) begin
		irq_count   <= '0;
		irq_pending <= 1'b0;
	end else begin
		if (cpu_we && sel_irqclr) irq_pending <= 1'b0;
		if (irq_count == IRQW'(IRQ_DIV - 1)) begin
			irq_count   <= '0;
			irq_pending <= 1'b1;
		end else begin
			irq_count <= irq_count + 1'b1;
		end
	end
end

assign cpu_irq = irq_pending;

endmodule
