// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

//============================================================================
//
// Atari System 2 for MiSTer FPGA
//   Paperboy - 720 Degrees - Super Sprint - Championship Sprint - APB
//
// This program is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation; either version 2 of the License, or (at your option)
// any later version. The core as a whole links components that require v3, so
// the combined work is GPL-3.0-or-later -- see LICENSE and README.md.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
//============================================================================

module emu
(
	`include "sys/emu_ports.vh"
);

// Download stream region bases/sizes, shared with sys2_rom_loader.sv so the
// two cannot drift apart.
`include "rtl/rom/sys2_rom_layout.vh"

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
// SDRAM pins are driven by sys2_sdram below.
// DDRAM is driven by screen_rotate (APB's vertical cabinet) -- see the rotation block near the
// video output. It stays completely idle on the four horizontal games, and on APB in the plain
// "No Rotate" orientation.

assign VGA_SL          = 0;
assign VGA_F1          = 0;
assign VGA_SCALER      = 0;
assign VGA_DISABLE     = 0;
assign HDMI_FREEZE     = 0;
assign HDMI_BLACKOUT   = 0;
assign HDMI_BOB_DEINT  = 0;

assign AUDIO_S   = 1;          // signed samples (YM2151 + POKEY are signed)
assign AUDIO_MIX = 0;
// AUDIO_L / AUDIO_R are driven by the sound mixer below (after sound_bus, where
// the chip outputs ym_left/ym_right/pokey1_snd/pokey2_snd are in scope).

assign LED_DISK  = 0;
assign LED_POWER = 0;
assign BUTTONS   = 0;

// Aspect ratio (O[122:121]). VIDEO_ARX/ARY are driven by the video_freak instance in the video
// output stage below, which overrides them with an exact pixel size in the integer Scale modes.
//
// "Original" has to follow the orientation. Rotated, APB presents a 384x512 portrait frame out
// of the DDRAM framebuffer, so its original aspect is 3:4 -- the cabinet is a 4:3 tube turned on
// its side. Every other state (all four horizontal games, and APB's two passthrough states)
// presents the native 512x384 landscape raster, which is 4:3. Advertising 4:3 for the rotated
// frame stretches it horizontally into a landscape box; advertising 3:4 for a landscape raster
// squashes it. `ar_rotated` is declared with the rotation block further down -- search
// "SCREEN ROTATION" -- so the orientation decode stays in one place.
//
// `ar == 0` is "Original" and is spelled out rather than written as MiSTer's boilerplate `!ar`:
// a logical NOT of a 2-bit field is a Verilator WIDTHTRUNC, and the lint used to carry a
// line-numbered exclusion for it -- which any edit above here silently invalidates (this change
// did exactly that) and which would just as silently swallow an unrelated future warning that
// landed on the same line. Same logic, no exclusion.
wire [1:0]  ar  = status[122:121];
wire [11:0] arx = (ar == 2'd0) ? (ar_rotated ? 12'd3 : 12'd4) : 12'(ar - 1'd1);
wire [11:0] ary = (ar == 2'd0) ? (ar_rotated ? 12'd4 : 12'd3) : 12'd0;

`include "build_id.v"

// CRT / analog-video fine-alignment page. Every field is 6-bit two's complement -- menu index 0..31 is
// 0..+31, index 32..63 is -32..-1 -- which is what the RTL below reads back with $signed().
// +-32 comes from the horizontal blanking budget rather than being an arbitrary cap: the line is
// 640 pixels with a 32-pixel front porch, a 64-pixel HSync and a 32-pixel back porch
// so both horizontal controls are usable across their whole range.
// Vertically the budget is asymmetric -- 8 blank lines before VSync and 16 after -- so the useful
// V-Shift range is about -8..+16; the field keeps the full +-32 to match the upstream core and to
// leave room for badly adjusted monitors, but past that VSync lands inside active video.
`define PB_ALIGN32 "0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,+16,+17,+18,+19,+20,+21,+22,+23,+24,+25,+26,+27,+28,+29,+30,+31,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1"

// ---------------------------------------------------------------------------
// Physical DIP switches -- per-game.
//
// One bitstream serves five games, and the two option switches (6/7A coin, 5/6A game) mean
// different things on each. DSW0 is genuinely common where we expose it: Coins-Per-Credit is
// sw8,7 and Free Play is sw3,2,1 = ON ON ON on every game, and the sw6,5,4 field (Left/Right
// coin multiplier, or ssprint/csprint's single "All Coin Mechanisms" multiplier) is held at
// the factory 1:1 = 000 either way. So those two stay unconditional.
//
// DSW1 is NOT common -- only the field WIDTHS happen to line up, which is what made this
// look fine for so long. Paperboy's Lives/Bonus Life sit exactly where Super Sprint's
// Wrenches/Obstacles do, so the OSD was setting the right bits under the wrong names.
// Each game therefore gets its own group, gated by status_menumask via the `h<n>` prefix
// (shown only when the mask bit is SET). Groups use SEPARATE status bits rather than reusing
// Paperboy's: all five games share one .rbf, so MiSTer keeps ONE config file for them, and
// overlapping the bits would let a setting made in one game reappear as a different setting
// in the next. Paperboy's own bits are untouched.
//
// Option lists are in DIP-value order (index 0 = switch value 0), because the status
// field is fed straight into the switch bits. Reordering a list silently changes what the
// game is told. sw7 is the HIGH bit of each pair and sw8 the low one (verified: MAME's
// ssprint values reproduce TM-292's switch table exactly under that mapping).
//
// csprint is sourced from its manual, not MAME. MAME only fills in csprint's 0x80 bit and
// leaves bits 5:0 inheriting Paperboy's Lives/Bonus Life -- impossible for a racing game.
// TM-292 Table 1-3 gives Drone Difficulty (sw8,7), Track Hazard Difficulty (sw6,5), Wrenches
// to Customize Car (sw4,3) and Auto High Score Reset (sw1) -- i.e. Super Sprint's layout.
// Hardware doc outranks the reference.
//
// All five games are now manual-sourced. Every field below was checked against
// its operator manual; every encoding MAME had was correct, and what the manuals changed
// was the NAMES. MAME uses generic `DEF_STR` stand-ins where a game has no matching enum --
// 720's "Bonus Life" is really first bonus ticket at and its "Difficulty" is timer for
// STREET; APB's "Max Continues" is Add-A-Coin Control.
//
//   paperboy  TM-275 Table 1-3 (unchanged, and its labels predate this work)
//   720       TM-294 Table 1-3 + Figure 3-22, the game's own game options screen
//   ssprint   TM-290 Table 3-2 + Figure 3-20, the game's own switch settings screen
//             TM-290 keeps its table in chapter 3 (self-test), not chapter 1
//   csprint   TM-292 Table 1-3
//   apb       TM-308 Table 1-3
//
// Where a manual photographs the game's self-test screen, that screen wins: it is the
// machine's own wording. "DIFFICULTY / OBSTACLES / WRENCHES" (ssprint) and "FIRST BONUS
// TICKET AT / TIMER FOR STREET / MAXIMUM ADD. A. COINS" (720) are read straight off it.
//
// ssprint has NO switch-1 option; csprint's Auto High Score Reset is the one place the two
// Sprints genuinely differ, and MAME has that right.
// APB sw8 (bit 0): TM-308 says "as press time, this switch is not used and has no effect.
// However, refer to the Report Options screen in self-test to be sure" -- the manual hedges,
// so MAME's "Attract Lights" is kept rather than dropping the option.
// APB's Add-A-Coin maximum reads "Unlimited" on the manual's top setting; MAME calls it
// 199, which is presumably the internal counter cap. The manual's word is used.
`define PB_DIP_MENU \
	"P1,Game Options;", \
	"P1-,Physical DIP switches (6/7A coin, 5/6A game);", \
	"P1O[26],Free Play,Off,On;", \
	"h0P1O[97:96],Coins/Credit,2 Coins/1 Credit,1 Coin/1 Credit,4 Coins/1 Credit,3 Coins/1 Credit;", \
	"H0P1O[28:27],Coins/Credit,1 Coin/1 Credit,2 Coins/1 Credit,3 Coins/1 Credit,4 Coins/1 Credit;", \
	"h0P1O[30:29],Difficulty,Medium-Hard,Easy,Medium,Hard;", \
	"h0P1O[32:31],Bonus Life,15000,None,10000,20000;", \
	"h0P1O[34:33],Lives,4,Demo,3,5;", \
	"h1P1O[65:64],First Bonus Ticket,5000,3000,8000,12000;", \
	"h1P1O[67:66],Timer For Street,Medium,Easy,Hard,Very Hard;", \
	"h1P1O[69:68],Maximum Add. A. Coins,2,0,1,3;", \
	"h1P1O[71:70],Credits Required,2 Start/1 Cont,1 Start/1 Cont,3 Start/2 Cont,3 Start/1 Cont;", \
	"h2P1O[73:72],Difficulty,Medium,Easy,Medium-Hard,Hard;", \
	"h2P1O[75:74],Obstacles,Medium,Easy,Medium-Hard,Hard;", \
	"h2P1O[77:76],Wrenches,3,2,4,5;", \
	"h3P1O[79:78],Drone Difficulty,Medium,Easy,Medium-Hard,Hard;", \
	"h3P1O[81:80],Track Hazard Difficulty,Medium,Easy,Medium-Hard,Hard;", \
	"h3P1O[83:82],Wrenches,3,2,4,5;", \
	"h3P1O[84],Auto High Score Reset,Yes,No;", \
	"h4P1O[85],Attract Lights,On,Off;", \
	"h4P1O[87:86],Add-A-Coin Control,25,3,10,Unlimited;", \
	"h4P1O[90:88],Game Difficulty,Medium-Easy,Very Hard,Hard,Hardest,Medium-Hard,Easy,Very Easy,Easiest;", \
	"h4P1O[92:91],Credits Required,2 Start/1 Cont,1 Start/1 Cont,3 Start/2 Cont,3 Start/1 Cont;",

// Steering feel for the four LETA games (from a hardware report that the D-pad
// steered nothing but Paperboy: once it steers, ONE fixed rate is not enough -- a D-pad pins
// the axis at full scale, so the rate below IS the turn rate).
//
// H0, not h5. Capital H means "hide while mask bit 0 is SET", and bit 0 is Paperboy -- so
// this shows on exactly the four LETA games with no new menumask bit. Paperboy is excluded
// because its handlebar is a position ADC: there is no rate to scale and no deadzone to widen,
// so the option would be visible and inert, which is worse than absent.
//
// One macro used by both CONF_STRs, like the DIP and CRT groups -- Aspect ratio and Orientation
// are copy-pasted into both and are exactly the drift this avoids.
// Four choices in each 2-bit field, so check_conf_str.py's 2**width rule is met outright and
// neither needs a SHORT_LISTS exemption. Index 0 is the shipped behaviour in both.
`define PB_STEER_MENU \
	"H0O[99:98],Steering sensitivity,Normal,High,Low,Lowest;", \
	"H0O[101:100],Steering deadzone,Normal,Small,Large,Largest;",

`define PB_CRT_ALIGN_MENU \
	"-;", \
	"P2,CRT Alignment;", \
	"P2-,Analog output only - HDMI uses Scale;", \
	"P2O[51:46],CRT H-Position,", `PB_ALIGN32, ";", \
	"P2O[57:52],Analog VGA H-Shift,", `PB_ALIGN32, ";", \
	"P2O[63:58],Analog VGA V-Shift,", `PB_ALIGN32, ";",

// The Service switch is an original operator control (TM-275 self-test / control
// calibration), not a debug aid, so it is a normal menu entry.
localparam CONF_STR = {
	"Atari System 2;;",
	"-;",
	"O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"O[44:43],Scale,Normal,V-Integer,HV-Integer,Narrower HV-Integer;",
	// h4 = APB only: it is the family's single VERTICAL cabinet and the only game this does
	// anything for. Vert rotates it CCW for a normal landscape monitor; the two passthrough
	// states suit a physically rotated (TATE) one, turned either way. THREE choices in a 2-bit
	// field is deliberate -- screen_rotate has exactly three usable states -- see the rotation
	// block below for the decode.
	"h4O[94:93],Orientation,Vert,No Rotate,No Rotate 180;",
	`PB_STEER_MENU
	"-;",
	`PB_DIP_MENU
	`PB_CRT_ALIGN_MENU
	"-;",
	"O[4],Service switch,Off,On;",
	"-;",
	"T[0],Reset;",
	"J1,Throw R,Throw L,Coin,Service Credit,Start,Button 3;",
	"jn,A,B,Select,L,X,Y;",
	"V,v",`BUILD_DATE
};

wire         forced_scandoubler;
wire  [1:0]  buttons;
wire [127:0] status;
// HPS "Direct Video" (hps_io cfg[10]): the raw core video goes straight out with the scaler
// bypassed, so the DDRAM framebuffer cannot be presented at all. Read only by the orientation
// decode, which forces passthrough while it is set.
wire         direct_video;

// Player controls from the HPS (USB pad / keyboard, mapped by the framework via
// the CONF_STR "J1"/"jn" lines below). joystick_0 bit order: [0]=right [1]=left
// [2]=down [3]=up, then the J1 buttons start at [4]. joystick_l_analog_0 holds
// the analog stick: [7:0]=X, [15:8]=Y (signed, center 0).
wire [31:0]  joystick_0;
wire [15:0]  joystick_l_analog_0;
// P2/P3 analog for the Sprints' pedals; Paperboy reads neither.
wire [15:0]  joystick_l_analog_1;
wire [15:0]  joystick_l_analog_2;
// Players 2 and 3 exist only for the other System 2 games: Championship Sprint is
// two-player and Super Sprint is three. Paperboy reads neither, so these are inert
// on the shipping game. hps_io drives them all from the same HPS pad list.
wire [31:0]  joystick_1;
wire [31:0]  joystick_2;
// Spinners drive the LETA quadrature generators. [7:0] is a signed delta and [8] toggles
// on every update; see sys2_quad_gen. Paperboy uses none of them.
wire  [8:0]  spinner_0, spinner_1, spinner_2;

// OSD menu mask: one-hot over SYS2_GAME_*, driving the CONF_STR `h<n>` DIP groups. Declared
// here because hps_io is instantiated long before game_id exists; the assign lives next to
// game_id's declaration so the two cannot drift apart.
wire [15:0]  menumask;

// ROM download interface (index 0 = MRA ROM stream, index 1 = Slapstic byte, index 2 = the
// <nvram> EEPROM save-back channel -- see the sl_wr_nvram / nvram_dirty logic below).
wire         ioctl_download;
wire [15:0]  ioctl_index;
wire         ioctl_wr;
wire [26:0]  ioctl_addr;
wire  [7:0]  ioctl_dout;
wire         ioctl_wait;
wire         ioctl_upload;
wire         ioctl_upload_req;
wire  [7:0]  ioctl_upload_index;
wire  [7:0]  ioctl_din;
// ioctl_wait = the loader's own fifo backpressure OR the raw-ioctl sprite SDRAM writer's per-word
// backpressure (spw_busy, defined near the loader_writer below). The sprite ROM loads straight to
// SDRAM from ioctl (the proven raw-ioctl pattern, bypassing the unreliable loader FIFO/drain that
// left the BRAMs empty); the HPS must stall while each 16-bit SDRAM write drains.
wire         loader_ioctl_wait;
wire         spw_busy;
assign       ioctl_wait = loader_ioctl_wait | spw_busy;

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(),
	.forced_scandoubler(forced_scandoubler),
	.direct_video(direct_video),
	.buttons(buttons),
	.status(status),
	// One-hot game select, so the CONF_STR `h<n>` groups show only the running game's DIP
	// options (see PB_DIP_MENU). Before the game descriptor lands game_id is 0, which shows
	// Paperboy's -- the same thing that was shown unconditionally before, and the menu
	// re-resolves as soon as the MRA's index-1 bytes arrive.
	.status_menumask(menumask),

	.joystick_0(joystick_0),
	.joystick_1(joystick_1),
	.joystick_2(joystick_2),
	.spinner_0(spinner_0),
	.spinner_1(spinner_1),
	.spinner_2(spinner_2),
	.joystick_l_analog_0(joystick_l_analog_0),
	.joystick_l_analog_1(joystick_l_analog_1),
	.joystick_l_analog_2(joystick_l_analog_2),

	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wait(ioctl_wait),

	.ioctl_upload(ioctl_upload),
	.ioctl_upload_req(ioctl_upload_req),
	.ioctl_upload_index(ioctl_upload_index),
	.ioctl_din(ioctl_din)
);

wire clk_sys;   // 32 MHz video / download / loader / main-bus / sound domain
// clock collapse: clk_t11 is now the SAME net as clk_sys. The T-11 main bus + 6502
// sound run in the single 32 MHz domain via clock-enables (sys2_clocks), so every former
// clk_t11<->clk_sys crossing (vblank/scanline IRQ, maincpu ROM read, scroll, rom_loaded) is now
// same-clock and deterministic -- removing the metastable-CDC bug class that
// black-screened the board. The clk_t11 logic Fmax is 57.91 MHz, comfortably above 32 MHz. The old 20 MHz outclk_1
// is left generated-but-unused.
wire clk_t11 = clk_sys;
wire clk_pll1_unused;
wire pll_locked;

pll pll
(
	.refclk(CLK_50M),
	.rst(1'b0),
	.outclk_0(clk_sys),
	.outclk_1(clk_pll1_unused),  // was clk_t11 (20 MHz); collapsed into clk_sys
	.outclk_2(SDRAM_CLK),   // clean phase-controlled SDRAM clock (was fabric ~clk in the controller)
	.locked(pll_locked)
);

wire reset = RESET | status[0] | buttons[1] | ~pll_locked;

// PLL-lock-only reset for the SDRAM subsystem (controller + ROM loader-writer). The game `reset`
// above is HIGH during the ROM download (RESET asserted while loading) -- and the controller drops
// SDRAM_CKE and re-enters its 200us init whenever its reset is high, while the loader-writer's FSM is
// forced to M_IDLE. So with `reset`, no SDRAM write can land during the download (proven on
// hardware: every byte arrived yet the writer FSM never ran). This is the root cause the maincpu/audio
// "empty SDRAM" saga worked around by moving to BRAM. The reference Atari System 1 core (and standard
// MiSTer practice) keep the SDRAM controller alive across the download, gating only on pll_locked. Do
// the same: the controller inits ONCE at PLL lock and stays up through download + gameplay; the
// loader-writer runs during the download so its writes actually commit.
wire sdram_init_reset = ~pll_locked;

wire       ce_pix;
wire       hblank;
wire       hsync;
wire       vblank;
wire       vsync;
wire       scanline_irq;
wire       vblank_irq;
wire [9:0] h_count;
wire [8:0] v_count;
wire [7:0] video_red;
wire [7:0] video_green;
wire [7:0] video_blue;
wire       video_hsync;
wire       video_vsync;
wire       video_de;

sys2_video_timing video_timing
(
	.clk_32(clk_sys),
	.reset(reset),
	.ce_pix(ce_pix),
	.h_count(h_count),
	.v_count(v_count),
	.hblank(hblank),
	.hsync(hsync),
	.vblank(vblank),
	.vsync(vsync),
	.scanline_irq(scanline_irq),
	.vblank_irq(vblank_irq)
);

assign CLK_VIDEO = clk_sys;
assign CE_PIXEL  = ce_pix;
// VGA_DE / VGA_HS / VGA_VS / VGA_R / VGA_G / VGA_B are driven by the video output stage further
// down (search "Video output stage").


//============================================================================
// Video output stage: OSD scaling + analog CRT fine alignment.
//   O[44:43] Scale               -- HDMI/scaler integer modes (sys/video_freak.sv)  (O[45] spare)
//   O[51:46] CRT H-Position      -- moves the picture inside an untouched HSync
//   O[57:52] Analog VGA H-Shift  -- moves HSync itself, +-32 pixels
//   O[63:58] Analog VGA V-Shift  -- moves VSync itself, +-32 lines
//   (Everything from 46 to 94 is spoken for: 46-63 here, 64-92 the per-game DIP groups, and
//    O[94:93] APB's Orientation. O[95] is now SPARE -- it was the old 1-bit Rotation option,
//    replaced by the 2-bit field below it. O[97:96] is Paperboy's Coins/Credit, and
//    O[99:98] + O[101:100] are the two steering-feel fields (PB_STEER_MENU);
//    O[102] and up are free. Note O[51:46] is CRT H-Position -- a debug control was briefly
//    mis-assigned to O[47:46] and collided with it.)
// Ported from Arcade-Raiden_MiSTer by rmonic79 (GPL-3.0). Raiden's companion CRT H-Size control
// is deliberately NOT here: it stretches pixels in whole CLK_VIDEO periods, and Paperboy runs a
// 16 MHz pixel clock on a 32 MHz CLK_VIDEO, so its smallest step would be half a pixel (Raiden
// has 16 clocks per pixel, where the same step is invisible). See sys2_analog_hpos.sv.
//
// Every field defaults to 0, and at 0 this whole stage is a wire: each shift generator muxes back
// to the renderer's own sync output and the H-Position line buffer is bypassed, so a default build
// is bit-identical at VGA_* to having none of this.
//============================================================================

// ---- Analog VGA H-Shift / V-Shift -----------------------------------------------------------
// Raster geometry and the renderer's sync pipeline depth, from sys2_video_timing and
// sys2_video_timing.sv. sys2_analog_sync_shift regenerates sync from the counters at these
// numbers, so they have to agree with the timing module -- simulation checks that
// by comparing against the real modules rather than against a copy of the constants.
wire hs_shifted, vs_shifted;

sys2_analog_sync_shift #(
	.H_TOTAL(640), .H_SYNC_BEG(544), .H_SYNC_END(608),
	.V_TOTAL(416), .V_SYNC_BEG(392), .V_SYNC_END(400),
	.PIPE(4)
) analog_sync_shift (
	.clk    (clk_sys),
	.reset  (reset),
	.ce_pix (ce_pix),
	.h_count(h_count),
	.v_count(v_count),
	.hshift ($signed({status[57], status[57:52]})),
	.vshift ($signed({status[63], status[63:58]})),
	.hs_out (hs_shifted),
	.vs_out (vs_shifted)
);

// At shift 0 the generator already reproduces the renderer's own sync exactly; muxing back to it
// anyway keeps a default build provably untouched even if the renderer's pipeline depth changes.
assign VGA_HS = (|status[57:52]) ? hs_shifted : video_hsync;
assign VGA_VS = (|status[63:58]) ? vs_shifted : video_vsync;

// ---- CRT H-Position -------------------------------------------------------------------------
// video_de is the only raster input it needs: it derives its own vertical gate from whether a
// buffered line carried any active video (see the module header for why a vblank input would be
// misaligned by the back porch).
wire       hpos_active = |status[51:46];
wire [7:0] hpos_r, hpos_g, hpos_b;
wire       hpos_de;

sys2_analog_hpos #(.AW(10)) analog_hpos     // 1024 per bank >= the 640-pixel horizontal total
(
	.clk     (clk_sys),
	.reset   (reset),
	.ce_pix  (ce_pix),
	.hoffset ($signed({status[51], status[51:46]})),
	.r_in    (video_red),
	.g_in    (video_green),
	.b_in    (video_blue),
	.hs_in   (video_hsync),                     // native HSync, never the shifted one
	.de_in   (video_de),
	.r_out   (hpos_r),
	.g_out   (hpos_g),
	.b_out   (hpos_b),
	.de_out  (hpos_de)
);

// sys_top gates the analog DAC with VGA_DE (it feeds the scanline/OSD chain 0 outside it), so the
// repositioned picture and its DE window have to travel together.
wire vga_de_in = hpos_active ? hpos_de : video_de;

assign VGA_R = hpos_active ? hpos_r : video_red;
assign VGA_G = hpos_active ? hpos_g : video_green;
assign VGA_B = hpos_active ? hpos_b : video_blue;

// ---- Scale ------------------------------------------------------------------------------------
// video_freak SCALE codes: 0 normal, 1 V-integer, 2 HV-integer narrower, 3 HV-integer wider,
// 4 HV-integer nearest. The menu exposes the four Raiden offers, in Raiden's order.
wire [2:0] scale_sel = (status[44:43] == 2'd0) ? 3'd0 :   // Normal
                       (status[44:43] == 2'd1) ? 3'd1 :   // V-Integer
                       (status[44:43] == 2'd2) ? 3'd4 :   // HV-Integer
                                                 3'd2;    // Narrower HV-Integer

video_freak video_freak
(
	.CLK_VIDEO  (clk_sys),
	.CE_PIXEL   (ce_pix),
	.VGA_VS     (video_vsync),                  // native VSync: frame measurement, not presentation
	.HDMI_WIDTH (HDMI_WIDTH),
	.HDMI_HEIGHT(HDMI_HEIGHT),
	.VGA_DE     (VGA_DE),
	.VIDEO_ARX  (VIDEO_ARX),
	.VIDEO_ARY  (VIDEO_ARY),
	.VGA_DE_IN  (vga_de_in),
	.ARX        (arx),
	.ARY        (ary),
	.CROP_SIZE  (12'd0),                        // vertical crop is not exposed
	.CROP_OFF   (5'd0),
	.SCALE      (scale_sel)
);

// ---------------------------------------------------------------------------
// Screen rotation -- APB only
//
// APB is the only VERTICAL cabinet in the family. The raster is the same 512x384 as every
// other game; it is the MONITOR that is turned, which is why MAME renders APB 384x512 and why
// its self-test screen only matches ours after a rotation. **The direction is measured, not
// assumed: rotated CCW our frame is 0 px against MAME's, rotated CW it is 2432 px.**
//
// `screen_rotate` lives in sys/arcade_video.v and works by writing the video out to a DDRAM
// framebuffer and letting the scaler read it back transposed -- so it needs MISTER_FB, which
// was commented out in the .qsf until now, and it is the ONLY user of DDRAM in this core.
//
// Not simulatable here. The top-level bench stubs DDRAM (BUSY=0, DOUT_READY=0) and models no
// framebuffer, so no simulation can exercise this path -- lint covers the wiring
// and Quartus covers the fit and timing, and that is the whole of the offline evidence. It is
// a hardware-verified change by construction; treat the first APB flash as its real test.
//
// ORIENTATION (O[94:93], APB only -- `h4`). screen_rotate exposes exactly THREE usable states,
// not four: it derives `do_flip = no_rotate && flip` and `fb_en = ~no_rotate | flip`
// (sys/arcade_video.v:233,288), so rotating and flipping are mutually exclusive -- `flip` means
// "180 degrees INSTEAD of rotating" and is dead while the core rotates.
//   0 Vert          : rotate CCW through the DDRAM framebuffer -- a normal landscape monitor
//   1 No Rotate     : straight passthrough, framebuffer off  -- a physically rotated monitor
//   2 No Rotate 180 : passthrough flipped 180                -- monitor rotated the other way
//   3 (unreachable from the OSD; a stale .cfg decodes it as plain passthrough)
//
// `vertical_game` gates all three states, and that is not redundant with the `h4` menu mask.
// `h4` only HIDES the option on the four horizontal games -- the status bits still exist and
// still hold whatever the shared .cfg last wrote. Without the gate, a cfg left at "No Rotate
// 180" by an APB session would bring Paperboy up upside down with no visible control to undo
// it. Off-APB behaviour is therefore bit-identical to having none of this.
//
// direct_video bypasses the scaler and the framebuffer entirely, so it forces the passthrough
// state; this is the standard MiSTer arcade idiom (`no_rotate = <opt> | direct_video`).
//
// The default (0) is unchanged from the previous "Auto (CCW)": rotate iff the running game is
// vertical. The old option was a single bit at O[95] with just Auto/Off; a stale .cfg that still
// has bit 95 set is harmless -- nothing reads it now, and the new field's bits power up at 0.
wire vertical_game = (game_id == SYS2_GAME_APB);
wire [1:0] o_orient = status[94:93];        // OSD: "Orientation: Vert / No Rotate / No Rotate 180"
wire fb_no_rotate  = ~vertical_game | (o_orient != 2'd0) | direct_video;
wire fb_flip       =  vertical_game & (o_orient == 2'd2) & ~direct_video;
// Consumed by the aspect-ratio block at the top of the file: only state 0 presents a portrait
// frame. 180 runs THROUGH the framebuffer but comes back out landscape (FB_WIDTH <= hsz).
wire ar_rotated    =  vertical_game & (o_orient == 2'd0) & ~direct_video;

screen_rotate screen_rotate
(
	.CLK_VIDEO(CLK_VIDEO),
	.CE_PIXEL (CE_PIXEL),
	.VGA_R    (VGA_R),
	.VGA_G    (VGA_G),
	.VGA_B    (VGA_B),
	.VGA_HS   (VGA_HS),
	.VGA_VS   (VGA_VS),
	.VGA_DE   (VGA_DE),
	// CCW, measured against MAME rather than guessed (see above).
	.rotate_ccw(1'b1),
	.no_rotate (fb_no_rotate),
	.flip      (fb_flip),
	.video_rotated(),                       // not a port of this framework vintage's `emu`
	.FB_EN    (FB_EN),
	.FB_FORMAT(FB_FORMAT),
	.FB_WIDTH (FB_WIDTH),
	.FB_HEIGHT(FB_HEIGHT),
	.FB_BASE  (FB_BASE),
	.FB_STRIDE(FB_STRIDE),
	.FB_VBL   (FB_VBL),
	.FB_LL    (FB_LL),
	.DDRAM_CLK     (DDRAM_CLK),
	.DDRAM_BUSY    (DDRAM_BUSY),
	.DDRAM_BURSTCNT(DDRAM_BURSTCNT),
	.DDRAM_ADDR    (DDRAM_ADDR),
	.DDRAM_DIN     (DDRAM_DIN),
	.DDRAM_BE      (DDRAM_BE),
	.DDRAM_WE      (DDRAM_WE),
	.DDRAM_RD      (DDRAM_RD)
);
assign FB_FORCE_BLANK = 1'b0;

//============================================================================
// CPU, ROM loader, main bus, and video renderer integration.
//============================================================================

wire cpu_clk_en, adc_clk_en;
wire snd_cpu_en, snd_ym_en, snd_ym_cen_p1, snd_pokey_en;

sys2_clocks clocks
(
	.clk_t11        (clk_t11),
	.clk_snd        (clk_t11),
	.reset          (reset),
	.cpu_clk_en     (cpu_clk_en),
	.adc_clk_en     (adc_clk_en),
	.snd_cpu_en     (snd_cpu_en),
	.snd_ym_en      (snd_ym_en),
	.snd_ym_cen_p1  (snd_ym_cen_p1),
	.snd_pokey_en   (snd_pokey_en)
);

wire        ld_we;
wire  [2:0] ld_region;
wire [19:0] ld_addr;
wire  [7:0] ld_data;
wire        rom_loaded;
wire  [7:0] slapstic_type;
// Descriptor fields for the per-game config consumers that land in later phases
// (slapstic table select, TMS/LETA gating, input matrix, video widths).
// Captured and validated now so a wrong MRA is rejected from this build onward.
wire  [7:0] game_id, game_flags;
wire        descriptor_ok, descriptor_bad;

// OSD menu mask (declared up by hps_io): one bit per game, so each CONF_STR `h<n>` DIP group
// is shown only for its own game. An unknown id shows nothing rather than the wrong labels.
assign menumask = { 11'd0,
                    (game_id == SYS2_GAME_APB),
                    (game_id == SYS2_GAME_CSPRINT),
                    (game_id == SYS2_GAME_SSPRINT),
                    (game_id == SYS2_GAME_720),
                    (game_id == SYS2_GAME_PAPERBOY) };

// The loader decodes the ROM stream into the generic rom_we/rom_region/rom_addr/rom_data bus.
// All ROMs load via top-level raw-ioctl decodes -- BRAMs via sl_wr_*, the sprite SDRAM via
// spw_* -> the FIFO + loader-writer -- so the loader's dedicated per-region BRAM write
// strobes are removed (they were never the load path on silicon).
sys2_rom_loader rom_loader
(
	.clk            (clk_sys),
	.reset          (reset),
	.ioctl_download (ioctl_download),
	.ioctl_index    (ioctl_index),
	.ioctl_wr       (ioctl_wr),
	.ioctl_addr     (ioctl_addr),
	.ioctl_dout     (ioctl_dout),
	.ioctl_wait     (loader_ioctl_wait),
	.mem_ready      (~ldwr_wr_busy),   // single loader-writer paces the drain (all ROM writes)
	.rom_we         (ld_we),
	.rom_region     (ld_region),
	.rom_addr       (ld_addr),
	.rom_data       (ld_data),
	.slapstic_type  (slapstic_type),
	.game_id        (game_id),
	.game_flags     (game_flags),
	.descriptor_ok  (descriptor_ok),
	.descriptor_bad (descriptor_bad),
	.rom_loaded     (rom_loaded)
);

wire        rom_ack;
wire        rom_req;                  // maincpu ROM request  (registered CPU req)
wire [19:0] rom_addr_cpu;            // maincpu ROM address   (registered CPU addr)
wire [15:0] rom_rdata;
// ---------------------------------------------------------------------------
// Where the T-11's program comes from, chosen per game on the READ side.
//
// 720 and APB populate 13 and 17 of the 0x8000 windows against an 8-window BRAM compaction, so
// their program cannot be resident and comes from SDRAM through sys2_cpu_icache. Everything
// else reads the BRAM compaction exactly as before.
//
// The choice is read-side only. The MRA's index-1 game descriptor arrives after the whole
// index-0 stream, so the LOAD path cannot know the game -- both copies are written for every
// game and only one is read. (sys2_maincpu_rom's own game_in_bram already relies on the
// same "a read happens long after the descriptor lands" property.)
wire        use_prog_sdram = (game_id == SYS2_GAME_720) || (game_id == SYS2_GAME_APB);
wire [15:0] bram_rom_rdata;
wire        bram_rom_ack;
wire [15:0] ic_rom_rdata;
wire        ic_rom_ack;
// The cache must not see requests from the games that read BRAM: it would fill lines nobody
// reads and add ~23 single-word SDRAM reads per scanline of contention to three games that
// currently have none.
wire        ic_rom_req = rom_req & use_prog_sdram;
assign      rom_rdata  = use_prog_sdram ? ic_rom_rdata : bram_rom_rdata;
assign      rom_ack    = use_prog_sdram ? ic_rom_ack   : bram_rom_ack;
// The arbiter's single-word output, driven below.
wire [23:0] arb_sd_addr;
wire        arb_sd_req;

wire        cpu_rom_req;
wire [19:0] cpu_rom_addr;

// Register the CPU's ROM req+address TOGETHER before the ROM, so the maincpu BRAM receives a STABLE,
// aligned, REGISTERED address -- like the audio 6502 (registered address), which reads its BRAM
// correctly. The CPU otherwise drove the BRAM with main_bus's COMBINATIONAL rom_addr: on silicon that
// read 0x0000 at 0x8004 even though the captured address was correct (0x08004) and the content/timing
// were fine (clk_sys met) -- the lone structural difference from the working paths.
// The extra cycle is absorbed by the held rom_ack (the T-11 waits on it).
reg        cpu_rom_req_q  = 1'b0;
reg [19:0] cpu_rom_addr_q = 20'd0;
always @(posedge clk_t11) begin
	cpu_rom_req_q  <= cpu_rom_req;
	cpu_rom_addr_q <= cpu_rom_addr;
end
assign rom_req      = cpu_rom_req_q;
assign rom_addr_cpu = cpu_rom_addr_q;

// Sound 6502 program-ROM read interface (sound_bus <-> sys2_audio_rom_bram).
wire [15:0] audio_rom_addr;
wire [7:0]  audio_rom_data;
wire        audio_data_ready;

// Controller host-port wires. addr/wdata/we/req are muxed between the loader-writer
// (during the ROM download) and the gameplay read client.
wire [23:0] sdram_addr;
wire [15:0] sdram_wdata, sdram_rdata;
wire        sdram_we, sdram_req, sdram_valid, sdram_ready;
wire [23:0] ldwr_addr;
wire [15:0] ldwr_wdata;
wire        ldwr_we, ldwr_req, ldwr_wr_busy;

// All ROM writes go through ONE direct-to-controller path (sys2_sdram_loader_writer): on
// silicon Quartus pruned every non-maincpu arbiter WRITE client ("Lost fanout" -> sprite + audio
// never loaded), while the direct controller path (used
// by the direct write path) is never pruned. The loader-writer owns the controller while the
// download is in progress (and until any in-flight write drains). The arbiter has since been
// removed entirely -- maincpu and audio moved to on-chip BRAM, so the sprite ROM (the sole SDRAM
// client) keeps only its READ port, wired straight to the controller. The CPUs are held during
// the download, so there is no read contention.
wire        ldwr_sel = ~rom_loaded | ldwr_wr_busy;


// Gameplay single-word source: sprites ride the burst port, so this slot carries the
// ARBITER's single-word output -- client 0, the T-11 program cache.
wire [23:0] gp_sw_addr = arb_sd_addr;
wire        gp_sw_req  = arb_sd_req;
// The slot the arbiter actually owns. Everything above it in this priority chain drives the
// controller PAST the arbiter, so if the arbiter granted client 0 while one of them held the
// mux, the read would never be issued -- and the next `valid` from whoever DID own the bus would
// be routed to the CPU, because c0_valid is just `sd_valid && owner == OWN_CPU`. Masking c0_req
// with this makes the arbiter unable to grant unless the bus is really its own.
// An in-flight read is safe if the mux switches away mid-transaction: the controller has already
// latched it and still returns its `valid`, and the other requesters wait on sdram_ready.
assign sdram_addr  = ldwr_sel ? ldwr_addr  : gp_sw_addr;
assign sdram_wdata = ldwr_sel ? ldwr_wdata : 16'd0;
assign sdram_we    = ldwr_sel ? ldwr_we    : 1'b0;
assign sdram_req   = ldwr_sel ? ldwr_req   : gp_sw_req;

// Sprite graphics ROM -> SDRAM via a top-level raw-ioctl decode (NOT the loader FIFO/drain). The
// drain is the silicon hazard that left the maincpu/chars/tiles BRAMs EMPTY; the
// sprite was the last ROM still on it, so its SDRAM image came up zeros -> every sprite pixel
// decoded to pen 0 (pen 15 = transparent, so pen 0 is OPAQUE) -> solid grey boxes with correct
// geometry but no detail. Loading straight from ioctl to SDRAM, throttled by
// ioctl_wait (spw_busy) while each 16-bit write drains, is ALSO the standard MiSTer ROM-load
// pattern. The loader's only sprite transform -- XOR 0xff (sys2_rom_loader.sv:119) -- is
// reproduced here. The loader_writer below now serves the sprite region ONLY (maincpu and audio
// are in BRAM and no longer need SDRAM writes), so it is driven by these spw_* wires.
// Only the first SYS2_SPRITE_SDRAM_BYTES of the (now 1 MiB) v2 sprite region are forwarded:
// the SDRAM allocation and this FIFO's 18-bit offset are still Paperboy-sized. See the note in
// rtl/rom/sys2_rom_layout.vh -- widening lands with the SDRAM remap.
wire        spw_region = ioctl_download && (ioctl_index == 16'd0) &&
                         (ioctl_addr >= SYS2_BASE_SPRITE) &&
                         (ioctl_addr < SYS2_BASE_SPRITE + SYS2_SPRITE_SDRAM_BYTES);

// Tiles ride the same FIFO and writer as the sprite region. That is safe
// because the download stream is strictly address-ordered and the regions do not overlap,
// so exactly one of these is active at any moment -- no arbitration needed.
wire        tlw_region = ioctl_download && (ioctl_index == 16'd0) &&
                         (ioctl_addr >= SYS2_BASE_TILES) &&
                         (ioctl_addr < SYS2_BASE_TILES + 27'h08_0000);

// The T-11 program region rides the same FIFO and writer, for EVERY game (the load path
// cannot know which game -- see use_prog_sdram above). maincpu is at stream offset 0, so it is
// now the FIRST region through the FIFO; until now the FIFO was idle during it and spw_busy
// never asserted this early. That is safe because sl_wr_maincpu -- the BRAM copy -- decodes the
// SAME ioctl_wr strobe this FIFO samples, so the two see byte-for-byte the same stream whatever
// ioctl_wait does to its pacing.
wire        mcw_region = ioctl_download && (ioctl_index == 16'd0) &&
                         (ioctl_addr < SYS2_BASE_AUDIO);

wire        spw_we     = (mcw_region || spw_region || tlw_region) && ioctl_wr;
// Region offset: maincpu needs 20 bits (0x90000), tiles 19 (0x80000), sprite 18. The FIFO
// carries 20 now. Narrowing this silently aliases the top of a region onto its bottom --
// Simulation covers the high-offset case for exactly that reason.
wire [26:0] spw_off    = mcw_region ? (ioctl_addr - SYS2_BASE_MAINCPU)
                       : tlw_region ? (ioctl_addr - SYS2_BASE_TILES)
                                    : (ioctl_addr - SYS2_BASE_SPRITE);
// The inversion is the SPRITE region's transform only. Applying it to tiles would
// silently invert every tile pixel -- the kind of fault that renders a complete-looking
// picture in the wrong colours.
wire [7:0]  spw_data   = (mcw_region || tlw_region) ? ioctl_dout : ~ioctl_dout;
// Pace the raw-ioctl sprite bytes through a margin-buffered FIFO before the loader_writer.
// the previous per-byte combinational ioctl_wait (spw_busy = spw_region & ldwr_wr_busy) dropped the
// odd, word-completing byte on silicon (bytes arrived, no word was ever formed). The FIFO asserts wait with margin
// from its registered occupancy and drains to the writer paced by the writer's (registered) busy, so
// the writer always sees a clean even/odd pair and ioctl_wait never races the current byte.
wire        fifo_out_we;
wire [19:0] fifo_out_off;
wire [7:0]  fifo_out_data;
// The region each queued byte belongs to has to travel WITH it: the FIFO delays bytes, so
// by the time one drains, ioctl_addr may already have moved into the next region.
// Three regions now, so the tag carries the region CODE the writer decodes, not a flag.
wire [2:0]  fifo_in_region = mcw_region ? 3'd1 : tlw_region ? 3'd3 : 3'd4;
wire [2:0]  fifo_out_region;
sys2_sprite_byte_fifo #(.OFF_W(20)) sprite_byte_fifo
(
	.clk        (clk_sys),
	.in_we      (spw_we),
	.in_off     (spw_off[19:0]),
	.in_data    (spw_data),
	.wait_req   (spw_busy),          // -> ioctl_wait (margin-based; replaces the racy per-byte stall)
	.writer_busy(ldwr_wr_busy),
	.out_we     (fifo_out_we),
	.out_off    (fifo_out_off),
	.out_data   (fifo_out_data),
	.overflow   ()
);

// Region code, carried through a FIFO of its own so it cannot drift out of step with the
// bytes. Three bits wide; the depth must match the byte FIFO's.
sys2_sprite_byte_fifo #(.OFF_W(3)) region_tag_fifo
(
	.clk        (clk_sys),
	.in_we      (spw_we),
	.in_off     (fifo_in_region),
	.in_data    (8'h00),
	.wait_req   (),
	.writer_busy(ldwr_wr_busy),
	.out_we     (),
	.out_off    (fifo_out_region),
	.out_data   (),
	.overflow   ()
);

sys2_sdram_loader_writer #(.SDRAM_AW(24), .BASE_MAINCPU(SYS2_MAINCPU_SDRAM_WORD_BASE),
                              .BASE_SPRITE(SYS2_SPRITE_SDRAM_WORD_BASE_LIVE),
                              .BASE_TILES(SYS2_TILES_SDRAM_WORD_BASE)) loader_writer
(
	.clk        (clk_sys),
	.reset      (sdram_init_reset),   // run DURING the download (reset excludes the ROM-load window)
	.ld_we      (fifo_out_we),
	.ld_region  (fifo_out_region),               // R_MAINCPU / R_TILES / R_SPRITE, from the tag FIFO
	.ld_addr    (fifo_out_off),
	.ld_data    (fifo_out_data),
	.wr_busy    (ldwr_wr_busy),
	.sdram_addr (ldwr_addr),
	.sdram_wdata(ldwr_wdata),
	.sdram_we   (ldwr_we),
	.sdram_req  (ldwr_req),
	.sdram_ready(sdram_ready)
);

// Maincpu T-11 program ROM in on-chip BRAM (reverted from SDRAM). Loaded by a top-level raw-ioctl
// DECODE of the maincpu span (index-0 stream, ioctl_addr 0..0x08ffff) straight into the BRAM write
// port -- the EXACT proven pattern the audio ROM uses (sl_wr_audio). The previous loader-FIFO-drain
// path (dedicated region-1 strobe ld_we_maincpu) NEVER wrote the boot window on silicon: the
// maincpu region never drained with region==MAINCPU, so the BRAM stayed EMPTY -> the CPU fetched
// 0x0000 (HALT) at 0x8004 (and the maincpu-ROM integrity check gave a false 0==0 pass;
// observed on hardware). The FIFO/drain far-BRAM
// write is the same silicon hazard that forced audio to raw-ioctl; bypass it the same way.
wire sl_wr_maincpu = ioctl_download && ioctl_wr && (ioctl_index == 16'd0) && (ioctl_addr < SYS2_BASE_AUDIO);
sys2_maincpu_rom maincpu_rom
(
	// The window compaction is per-game DATA: Paperboy fills 5 windows, the Sprints 7 at
	// different indices. Feeding the wrong map makes a game fetch 0xffff from windows it
	// really uses, which looks like a dead CPU with nothing to point at.
	.game_id  (game_id),
	.clk      (clk_t11),
	.reset    (reset),
	.wr_clk   (clk_sys),
	.ld_we    (sl_wr_maincpu),       // top-level raw-ioctl decode (proven audio pattern)
	.ld_addr  (ioctl_addr[19:0]),    // logical maincpu byte offset 0..0x08ffff
	.ld_data  (ioctl_dout),
	.rom_req  (rom_req),
	.rom_addr (rom_addr_cpu),
	.rom_rdata(bram_rom_rdata),
	.rom_ack  (bram_rom_ack)
);

// Sprite graphics ROM in SDRAM. 256 KiB (131072 words) placed immediately
// after the maincpu region (81920 words). Pre-inverted by the loader.
//
// The playfield line fill shares the controller with sprite fetches, and four single-word
// reads per sprite row broke the fill's line deadline on MO-heavy screens (33 late lines on
// Super Sprint's "CHOOSE A TRACK" screen). Sprite reads are 2+2-word bursts on the arbiter's c1 (MO) client -- c1
// outranks the fill's c2 by design, and the walker's <=160 words/line is the exact load
// measured to fit (1190 clk vs the 1279 deadline). See sys2_sprite_rom_arb's header for the
// mechanism and the measurements.
//
// sprc_* connect this fetcher to the arbiter's c1 client below.
wire        sprc_req, sprc_req2;
wire [23:0] sprc_addr, sprc_addr2;
wire [8:0]  sprc_len, sprc_len2;
wire        sprc_valid, sprc_done;

// Per-game sprite geometry: 720 and APB carry 1 MiB of motion-object ROM (8192 tiles, plane
// stride 0x40000 words); everything else 256 KiB (2048 tiles, 0x10000). Both the code mask and
// the plane stride follow from it -- see the reader's header. Read-side use of game_id, so the
// descriptor has long landed.
wire spr_big = (game_id == SYS2_GAME_720) || (game_id == SYS2_GAME_APB);

sys2_sprite_rom_arb #(.SDRAM_AW(24), .SDRAM_BASE(SYS2_SPRITE_SDRAM_WORD_BASE_LIVE)) sprite_rom
(
	.clk      (clk_sys),
	.reset    (reset),
	.spr_big  (spr_big),
	.sp_req   (sp_req),
	.sp_tile  (sp_tile),
	.sp_row   (sp_row),
	.sp_valid (sp_valid),
	.sp_data  (sp_data),
	.arb_req  (sprc_req),
	.arb_addr (sprc_addr),
	.arb_len  (sprc_len),
	.arb_req2 (sprc_req2),
	.arb_addr2(sprc_addr2),
	.arb_len2 (sprc_len2),
	.arb_valid(sprc_valid),
	.arb_rdata(arb_rdata),
	.arb_done (sprc_done)
);

// Sound 6502 program ROM (0x4000-0xffff, 48 KiB) in on-chip BRAM -- the proven MiSTer/reference
// pattern (Arcade-Atari-system1 ap_srom*). Loaded by a top-level decode of the audiocpu span of
// the index-0 ioctl stream (0x094000-0x09ffff) straight into the BRAM write port, bypassing the
// loader FIFO and the loader's region-2 drain bus entirely. Read on clk_t11 (the 6502's clock),
// so there is NO clk_t11<->clk_sys read CDC. Replaces the earlier SDRAM-backed audio path (which
// used an arbiter read port) that repeatedly failed on real hardware.
// ld_addr below takes ioctl_addr[15:0] directly as the 6502 address. That works because the
// audiocpu region is 64 KiB based at a 64 KiB boundary, which v2 preserves (0x090000 unchanged).
wire sl_wr_audio = ioctl_download && ioctl_wr && (ioctl_index == 16'd0) &&
                   (ioctl_addr >= SYS2_BASE_AUDIO + SYS2_AUDIO_ROM_OFF) &&
                   (ioctl_addr < SYS2_BASE_TILES);
// The 6502's whole ROM lives in the stream BEFORE the tiles region, so "my region is
// fully streamed" = ioctl_addr reaching SYS2_BASE_TILES. Gating the 6502 on this (instead
// of on rom_loaded, or on snd_reset's old power-on-1 init) hands it the entire
// tile+sprite download time as a boot head start over the rom_loaded-gated T-11 -- the
// ordering real hardware gets from a shared power-on reset, made explicit. ssprint's boot
// requires it: see the snd_reset init note in sys2_main_bus.
logic audio_loaded = 1'b0;
always_ff @(posedge clk_sys) begin
	if (ioctl_download && (ioctl_index == 16'd0) && (ioctl_addr < 27'd16))
		audio_loaded <= 1'b0;                    // a fresh index-0 download restarts the gate
	else if (ioctl_download && (ioctl_index == 16'd0) && (ioctl_addr >= SYS2_BASE_TILES))
		audio_loaded <= 1'b1;
end

sys2_audio_rom_bram #(.PROG_BYTES('hC000)) audio_rom
(
	.wr_clk     (clk_sys),
	.ld_we      (sl_wr_audio),
	.ld_addr    (ioctl_addr[15:0]),   // 6502-style address (0x4000..0xffff) within the audio span
	.ld_data    (ioctl_dout),
	.clk        (clk_t11),
	.cpu_hold   (snd_reset | ~audio_loaded), // held until ITS region is streamed (see audio_loaded)
	.cpu_addr   (audio_rom_addr),
	.cpu_data   (audio_rom_data),
	.data_ready (audio_data_ready)
);


// (The old multi-port SDRAM arbiter was removed.) The maincpu and audio ROMs are now on-chip
// BRAM, which eliminated the per-client arbiter handshake that wedged the maincpu read FSM on
// silicon. What still reads SDRAM reaches it through sys2_sdram_arb's burst clients instead.


// Arbiter <-> controller burst wires. Declared HERE, ahead of both users: they were
// previously declared below the generate block that drives them, which is use-before-declare
// and only survived because nothing on this end consumed them.
wire [23:0] pf_brst_addr;
wire [8:0]  pf_brst_len;
wire        pf_brst_req, pf_brst_abort, pf_brst_done, pf_brst_accept, pf_brst_start;
// Burst-ONLY return strobe. See the note on brst_valid in sys2_sdram.sv: the shared
// `valid` also carries the sprite ROM's single-word reads, which the arbiter would hand to
// whoever owns the burst.
wire        pf_brst_valid;

sys2_sdram #(.CLK_MHZ(32)) sdram_ctrl
(
	.clk   (clk_sys),
	.reset (sdram_init_reset),   // stay initialized through the ROM download (CKE up, no re-init)
	.addr  (sdram_addr),
	.wdata (sdram_wdata),
	.we    (sdram_we),
	.req   (sdram_req),
	.rdata (sdram_rdata),
	.valid (sdram_valid),
	.ready (sdram_ready),
	// ---- BURST port -> sys2_sdram_arb (the playfield tile path) ----------------
	// These eight pins were MISSING from this instance. The arbiter drove pf_brst_addr /
	// _len / _req / _abort into nothing, and read back pf_brst_done / _accept / _start as
	// undriven nets, so the tile prefetcher issued column 0
	// and waited forever for a `done` that no controller was listening to produce. Every
	// playfield word stayed at the line buffer's power-on 0 -- pen 0 across the whole
	// playfield, i.e. a black screen, for every game including Paperboy.
	//
	// The equivalence bench could not see it: it instantiates linebuf + arbiter + controller and
	// wires them to each other itself, so it proves the TRIO works and says nothing about how
	// `emu` connects them. Same shape as the c2_req2 miss already recorded in
	// -- named port connections fail silently by omission.
	.brst_req   (pf_brst_req),
	.brst_addr  (pf_brst_addr),
	.brst_len   (pf_brst_len),
	.brst_abort (pf_brst_abort),
	.brst_busy  (),                  // the arbiter tracks its own occupancy
	.brst_valid (pf_brst_valid),    // burst words ONLY -- never the sprite ROM's single-word reads
	.brst_done  (pf_brst_done),
	.brst_accept(pf_brst_accept),
	.brst_start (pf_brst_start),
	.SDRAM_DQ  (SDRAM_DQ),
	.SDRAM_A   (SDRAM_A),
	.SDRAM_BA  (SDRAM_BA),
	.SDRAM_DQML(SDRAM_DQML),
	.SDRAM_DQMH(SDRAM_DQMH),
	.SDRAM_CLK (),            // SDRAM clock now driven by the PLL (outclk_2), not fabric ~clk
	.SDRAM_CKE (SDRAM_CKE),
	.SDRAM_nCS (SDRAM_nCS),
	.SDRAM_nRAS(SDRAM_nRAS),
	.SDRAM_nCAS(SDRAM_nCAS),
	.SDRAM_nWE (SDRAM_nWE)
);

wire [13:0] char_addr;
wire  [7:0] char_data;

// The alpha character bytes (region 5) are a far on-chip BRAM that loaded via the loader
// FIFO/drain's dedicated strobe (ld_we_chars) -- the SAME path that left the maincpu BRAM empty on
// silicon (the region never drained, so the strobe never fired). Load them by a top-level
// raw-ioctl decode instead, the proven audio/maincpu pattern, immune to the FIFO region corruption.
// The span is clamped to the target BRAM size, not the (larger) v2 region size --
// see the target-capacity block in rtl/rom/sys2_rom_layout.vh.
wire [26:0] chars_off = ioctl_addr - SYS2_BASE_CHARS;  // offset within the chars region (region 5)
wire sl_wr_chars = ioctl_download && ioctl_wr && (ioctl_index == 16'd0) &&
                   (ioctl_addr >= SYS2_BASE_CHARS) &&
                   (ioctl_addr < SYS2_BASE_CHARS + SYS2_CHARS_BRAM_BYTES);
// Factory EEPROM (512 B): the ROM-stream region right after chars. Loaded into the sound-bus
// eeprom[] so the game has valid coinage -- otherwise it powers up blank and "INSERT COIN" never
// credits (the coin input path itself is correct; verified vs SP-275 sheet 9A).
wire sl_wr_eeprom = ioctl_download && ioctl_wr && (ioctl_index == 16'd0) &&
                    (ioctl_addr >= SYS2_BASE_EEPROM) && (ioctl_addr < SYS2_END_STREAM);

// EEPROM save-back restore: the MRA declares <nvram index="2" size="512"/> AFTER the main ROM
// stream (index 0), so if a save exists on the SD card MiSTer sends it on this channel right
// after the factory fill above, overwriting it -- a save always wins over the factory default.
// ioctl_addr resets to 0 for this channel's own transfer, same as any other <rom>/<nvram> entry,
// so the existing ioctl_addr[8:0] wiring into eeprom_ld_addr below (from sl_wr_eeprom) is reused
// unchanged.
wire sl_wr_nvram  = ioctl_download && ioctl_wr && (ioctl_index == 16'd2);
sys2_char_rom char_rom
(
	.clk      (clk_sys),
	.ld_we    (sl_wr_chars),          // top-level raw-ioctl decode (proven audio/maincpu pattern)
	// FOURTEEN bits. This was chars_off[12:0] -- Paperboy's 8 KiB -- while the port, the
	// BRAM and SYS2_CHARS_BRAM_BYTES had all been widened to the family's 16 KiB. The 13-bit
	// slice zero-extended into the 14-bit port, so the UPPER 8 KiB was never written. Paperboy
	// cannot see that (its char ROM is 8 KiB and the rest of the region is 0xff fill), but
	// every other game fills all 16 KiB, so half of its character set was missing.
	.ld_addr  (chars_off[13:0]),
	.ld_data  (ioctl_dout),
	.char_addr(char_addr),
	.char_data(char_data)
);

// Motion-object line engine: walks the active list in the MO RAM each scanline and
// fills the double-buffered line buffer the renderer reads per pixel. clk_sys;
// start_link 0 (atarisy2 has no SLIP).
//
// Held in reset until BOTH the framework reset releases AND the ROM image is loaded
// (rom_loaded). The sprite graphics ROM is the sole SDRAM client, wired straight to the
// controller; an ungated mob engine issues sp_req reads to the controller DURING the download
// (it ran on the global `reset`, not cpu_reset, so it was live before rom_loaded), contending
// with the loader-writer's sprite writes for the single controller port. That breaks the
// loader's exclusive-access invariant that the ioctl_wait backpressure relies on and can wedge
// the HPS download (frozen ROM load). rom_loaded is in clk_sys -- this engine's clock -- so no
// extra CDC. OSD "Disable sprites" (status[7]) gates the mob engine off so it issues NO sp_req
// reads.
wire mob_reset = reset | ~rom_loaded;
sys2_mob_engine mob_engine
(
	.clk            (clk_sys),
	.reset          (mob_reset),
	.ce_pix         (ce_pix),
	.h_count        (h_count),
	.v_count        (v_count),
	.start_link     (8'd0),
	.mob_video_addr (mob_video_addr),
	.mob_video_data (mob_video_data),
	.sp_req         (sp_req),
	.sp_tile        (sp_tile),
	.sp_row         (sp_row),
	.sp_valid       (sp_valid),
	.sp_data        (sp_data),
	.rd_col         (mob_lb_col),
	.rd_en          (mob_lb_rd_en),
	.rd_data        (mob_lb_data)
);

// The CPU is released only when the ROM image is loaded AND the index-1 game
// descriptor was accepted (a bad descriptor holds the CPU rather than
// booting a mis-configured machine -- on System 2 a wrong slapstic table
// scrambles which VRAM plane the CPU writes, so "run anyway" is not benign).
// The status LED distinguishes the two cases: fast blink = descriptor rejected.
wire rom_ready = rom_loaded & descriptor_ok;
reg [1:0] rom_loaded_sync;
always @(posedge clk_t11) rom_loaded_sync <= {rom_loaded_sync[0], rom_ready};

// CPU reset. cpu_hold is asserted while the framework holds reset or the ROM
// image has not finished loading. cpu_reset adds the watchdog: an expired main
// watchdog (a genuine CPU hang that stopped servicing 014000) reboots the T-11
// for one clk, exactly as SYSRES reboots the board on the real PCB (SP6B).
// main_bus is reset by cpu_hold only -- deliberately NOT by watchdog_reset -- so
// (a) the watchdog counter is frozen at 0 through the whole download window and
// the CPU gets a full fresh period at boot rather than a stale near-expiry count,
// and (b) a watchdog reboot restarts the CPU without clearing the work RAM (as
// SRAM retains across SYSRES; the rebooting boot code re-initializes it).
wire watchdog_reset;
wire cpu_hold  = reset | ~rom_loaded_sync[1];
wire cpu_reset = cpu_hold | watchdog_reset;

reg [1:0] service_t11_sync;
always @(posedge clk_t11) begin
	if (reset) service_t11_sync <= 2'b00;
	else       service_t11_sync <= {service_t11_sync[0], status[4]};
end
wire service_t11 = service_t11_sync[1];

wire irq_vblank_t11, irq_scanline_t11;
sys2_pulse_cdc vblank_cdc
(
	.src_clk(clk_sys), .src_reset(reset), .src_pulse(vblank_irq),
	.dst_clk(clk_t11), .dst_reset(cpu_reset), .dst_pulse(irq_vblank_t11)
);
sys2_pulse_cdc scanline_cdc
(
	.src_clk(clk_sys), .src_reset(reset), .src_pulse(scanline_irq),
	.dst_clk(clk_t11), .dst_reset(cpu_reset), .dst_pulse(irq_scanline_t11)
);

wire        bus_req, bus_write;
wire [15:0] bus_addr, bus_wdata;
wire  [1:0] bus_byte_en;
wire [15:0] bus_rdata;
wire        bus_ack, bus_error;
wire  [3:0] cpu_cp;
wire  [5:0] rom_bank0, rom_bank1;
wire  [1:0] vmmu_bank;
wire  [3:0] irq_enable;
wire  [7:0] snd_cmd;
wire        snd_cmd_pending, snd_reset;
wire [15:0] xscroll, yscroll;
wire        adc_eoc;
wire        irq_snd_cmd_read;
wire        irq_snd_resp_write;
wire  [7:0] snd_resp;
wire        resp_pending;
wire        xscroll_strobe_t11, yscroll_strobe_t11;
wire        xscroll_update_vid, yscroll_update_vid;
reg  [15:0] xscroll_meta, xscroll_sync;
reg  [15:0] yscroll_meta, yscroll_sync;
wire [11:0] alpha_video_addr;
wire [15:0] alpha_video_data;
wire  [7:0] alpha_palette_addr;
wire  [7:0] palette_video_addr;
wire [15:0] palette_video_data;
wire [11:0] pf_top_render_addr, pf_bot_render_addr;

// ---------------------------------------------------------------------------
// Playfield map, tile PREFETCHER port.
//
// The prefetcher has its own port. It must never share the renderer's -- see the PORT_C
// note in sys2_dpram.sv.
//
// An earlier design avoided a third port by stealing the renderer's on the slots the renderer
// was assumed not to need ("the render video port is idle 7 cycles in 8, because pfx[9:3] only
// advances once per 8 pixels and the pipeline re-reads the same address in between"). The
// premise is false. sys2_video_render drives its map address EVERY clock and re-latches
//     pf_group2    <= pf_word2[13:11]      (palette group)
//     pf_category2 <= ~pf_word2[15:14]     (priority category)
// from the returned word EVERY clock, so a stolen slot does not go unnoticed -- it overwrites
// good values with the prefetcher's. Wrong group = wrong colour, wrong category = wrong
// priority: white streaks behind the alpha layer, which is what hardware showed.
//
// It was worse than one lost slot in eight, because the mux gated on `pfram_req` (REGISTERED)
// while `ram_addr` is COMBINATIONAL -- so the prefetcher's address reached the RAM a cycle after
// it was valid, and sys2_pf_linebuf's M_WAIT captured the RENDERER's word as its map word.
// Both sides read each other's data.
//
// No gate could see this. The equivalence bench gave the prefetcher a DEDICATED third
// port of its own, so its 195,840 fetches proved the prefetcher reads correctly and
// never checked that the renderer still does. Reproduce with +share_port: 25,344 stolen slots
// per frame, the renderer read back someone else's word on ALL of them.
// ---------------------------------------------------------------------------
wire        pfram_req, pfram_bot;
wire [11:0] pfram_addr;


// ---------------------------------------------------------------------------
wire [23:0] pf_tile_base = 24'(SYS2_TILES_SDRAM_WORD_BASE);

// Always granted: the port is the prefetcher's alone, so there is nothing to arbitrate. Kept as
// a signal (rather than deleting it from sys2_pf_linebuf) because the FSM's M_REQ -> M_WAIT
// handshake is what aligns its combinational ram_addr with the RAM's registered read.
wire        pfram_grant = 1'b1;

wire [11:0] pf_top_video_addr = pf_top_render_addr;
wire [11:0] pf_bot_video_addr = pf_bot_render_addr;
wire [15:0] pfram_data = pfram_bot ? pf_bot_pf_data : pf_top_pf_data;
wire [15:0] pf_top_video_data, pf_bot_video_data;
wire [15:0] pf_top_pf_data,    pf_bot_pf_data;

// pf_rom_addr is 18 bits (14-bit code); the SDRAM tile path consumes the full width.
wire [17:0] pf_rom_addr;
// ---------------------------------------------------------------------------
// SDRAM playfield tile path.
//
// Wiring mirrors the equivalence bench, which is what proves this byte-identical to the
// on-chip tile ROM it replaced, over a full frame. The four rules that bench had to learn the
// hard way, all of them silent if broken:
//
//   1. index the line buffer by pfx[9:3] -- the playfield-map column -- on BOTH sides, the
//      renderer's read and the prefetcher's fill.
//      This rule said the exact opposite ("by SCREEN column, not pfx[9:3]; the latter
//      double-applies scroll"), and the inversion WAS the family build's
//      playfield corruption: the buffer holds one tile per entry, while the renderer crosses
//      a tile boundary mid-column whenever sx[2:0] != 0. Nothing is double-applied when both
//      sides live in map space -- the fill simply starts at pfx0[9:3] and runs 65 columns;
//   2. swap buffers at the line boundary, never on fill completion -- swapping mid-line
//      changes what the renderer is reading part-way down a line;
//   3. the fill launched at line L is for line L+2, and that look-ahead must WRAP at
//      V_TOTAL or the first two lines of every frame read the wrong playfield rows;
//   4. take the renderer's EFFECTIVE scroll (sx_fetch_o/sy_fetch_o), not raw xscroll/
//      yscroll -- the pending/immediate-update rules live inside the renderer.
//
// And one this integration adds: hold the scheduler until the controller has finished
// power-up init, or the first fill overlaps tINIT and, because start is ignored while busy,
// takes the next four lines' fills down with it.
// ---------------------------------------------------------------------------
wire [9:0] sx_fetch_w;
wire [8:0] sy_fetch_w;
wire [3:0] bank0_fetch_w, bank1_fetch_w;
wire [6:0] pf_rom_col_w;

localparam int PF_V_TOTAL = 416;   // sys2_video_timing.sv

wire        pf_sd_req, pf_sd_req2;
wire [23:0] pf_sd_addr, pf_sd_addr2;
wire [8:0]  pf_sd_len,  pf_sd_len2;
wire        pf_sd_valid, pf_sd_done;
wire        pf_busy;
wire [15:0] arb_rdata;

logic       pf_start = 0, pf_swap = 0, ctl_up = 0, hb_d = 0;
wire [9:0]  pf_look  = 10'(v_count) + 10'd2;
wire [9:0]  pf_next_line = (pf_look >= 10'(PF_V_TOTAL)) ? (pf_look - 10'(PF_V_TOTAL)) : pf_look;
logic [8:0] pf_pfy;
logic [9:0] pf_pfx0;

always_ff @(posedge clk_sys) begin
	pf_start <= 1'b0;
	pf_swap  <= 1'b0;
	hb_d     <= hblank;
	if (sdram_ready) ctl_up <= 1'b1;
	if (!reset && ctl_up && hblank && !hb_d) begin
		pf_swap  <= 1'b1;
		pf_pfy   <= 9'(pf_next_line) + sy_fetch_w;
		pf_pfx0  <= sx_fetch_w;
		pf_start <= 1'b1;
	end
end

// Tile-code wrap for the SDRAM path: (tile elements - 1) for the running game. The board's
// tile ROM decodes only as many address lines as it has, so an over-range code wraps -- MAME
// writes that as `rawcode % gfx->elements()`. An on-chip tile ROM gets it free (wire it
// pf_rom_addr[15:0] and that IS code % 4096); the SDRAM path builds its own address and
// must be told. elements = HALF/16, i.e. MAME's tiles region / 32.
//
// Reading game_id HERE is fine -- this is a RENDER-time signal, long after the descriptor
// lands. Contrast sys2_maincpu_rom, where a LOAD-time game_id was a bug.
wire [13:0] pf_code_mask_raw = (game_id == SYS2_GAME_PAPERBOY) ? 14'h0fff :   //  4096 tiles
                               (game_id == SYS2_GAME_720)      ? 14'h1fff :   //  8192 tiles
                                                                 14'h3fff;    // 16384 (Sprints, APB)


// Held until rom_loaded, not merely until `reset` RELEASES -- the same gate the motion-object
// engine carries a few hundred lines above, and for the identical reason.
//
// The framework reset drops BEFORE the ROM download finishes, so a client reset only by `reset`
// is LIVE while the loader-writer is streaming the image into SDRAM. The mob engine had exactly
// this bug: its sp_req reads contended with the loader's writes for the single controller port,
// broke the exclusive-access invariant that the ioctl_wait backpressure depends on, and WEDGED
// the HPS download -- a frozen ROM load, so rom_loaded never asserts, the T-11 stays in reset
// and the screen is black for EVERY game.
//
// The tile prefetcher is driven by free-running video timing, so it starts issuing burst reads
// the moment it leaves reset. That was harmless only for as long as the controller's brst_* pins
// were unconnected; connecting them made it a live SDRAM client during the download
// and reintroduced the wedge. rom_loaded is in clk_sys, this module's clock, so no extra CDC.
wire pf_reset = reset | ~rom_loaded;

// COLS 128 / FILL_COLS 65: the buffer is indexed in playfield-map column space (pfx[9:3]),
// not screen columns, and a fine scroll pushes the last visible pixel into map column S+64.
// See the header of sys2_pf_linebuf.sv -- screen-column indexing corrupted the tile bitmap
// on every non-tile-aligned scroll while leaving the colour right.
sys2_pf_linebuf #(.AW(24), .COLS(128), .FILL_COLS(65)) pf_linebuf (
	.clk(clk_sys), .reset(pf_reset), .start(pf_start), .swap(pf_swap),
	.pfy(pf_pfy), .pfx0(pf_pfx0),
	.bank0(bank0_fetch_w), .bank1(bank1_fetch_w),
	.tile_base(pf_tile_base),
	.code_mask(pf_code_mask_raw),
	.ram_req(pfram_req), .ram_grant(pfram_grant), .ram_addr(pfram_addr),
	.ram_bot(pfram_bot), .ram_data(pfram_data),
	.sd_req(pf_sd_req), .sd_addr(pf_sd_addr), .sd_len(pf_sd_len),
	.sd_req2(pf_sd_req2), .sd_addr2(pf_sd_addr2), .sd_len2(pf_sd_len2),
	.sd_valid(pf_sd_valid), .sd_rdata(arb_rdata), .sd_done(pf_sd_done),
	.rd_col(pf_rom_col_w), .rd_half(pf_rom_addr[0]), .rd_data(pf_sdram_data),
	// The colour group / priority category of the map word THIS column's bitmap was built
	// from. Without it the renderer took them from its own, two-lines-later map read and a
	// CPU write in between drew a stale bitmap under a current colour -- see rd_attr in
	// sys2_pf_linebuf.sv.
	.rd_attr(pf_sdram_attr),
	.busy(pf_busy)
);

// The arbiter drives only the controller's BURST port. Its CPU client stays tied off
// (the T-11 program is in BRAM for every game this build serves). The c1 (motion-object)
// slot carries SPRITE fetches -- 2+2-word bursts from
// sys2_sprite_rom_arb, outranking the playfield fill by the fixed priority this slot
// was designed with -- so sprite reads no longer make four single-word round trips that
// break the fill's line deadline on MO-heavy screens.

// ---------------------------------------------------------------------------
// The T-11 program cache is the arbiter's client 0 -- single word, highest priority,
// which is exactly what c0 already was. It had been tied off since the arbiter was written.
//
// Fills are 8 single-word reads rather than a burst, deliberately: at ~49 misses/frame that
// costs ~2730 clk/frame against ~1170 as bursts, and 0.3 % of a frame does not justify widening
// c0 or re-ranking an arbiter whose ordering rules have their own recorded failure history.
// ---------------------------------------------------------------------------
wire        ic_sd_req, ic_sd_valid, ic_sd_done;
wire [23:0] ic_sd_addr;
wire [8:0]  ic_sd_len;
sys2_cpu_icache #(
	// Geometry back to the proven 2 KiB / 8-word lines. It was briefly LINE_WORDS(2)/LINES(512)
	// as a workaround while the fill was single-word: cost then tracked CPU WORDS per scanline,
	// so short lines cut over-budget scanlines 256->30 of 32218. With the fill issued as ONE
	// BURST that trade reverses -- a burst amortises its round trip over the whole line, so long
	// lines are strictly better (3,934 misses at 8 words vs 9,623 at 2). The burst fill holds
	// past 32 misses/scanline against a worst measured cluster of 8, so the workaround is no
	// longer needed and its +1,730 ALM tag store is given back.
	.AW(24), .REGION_BASE(SYS2_MAINCPU_SDRAM_WORD_BASE), .LINE_WORDS(8), .LINES(128)
) cpu_icache (
	.clk(clk_sys),
	// pf_reset = reset | ~rom_loaded -- the same hold every SDRAM client here needs. A client
	// gated only on `reset` contends with the loader and wedges the download; that mistake
	// has reached hardware twice.
	.reset(pf_reset),
	.mem_ready(rom_loaded),
	.rom_req(ic_rom_req), .rom_addr(rom_addr_cpu),
	.rom_rdata(ic_rom_rdata), .rom_ack(ic_rom_ack),
	.sd_req(ic_sd_req), .sd_addr(ic_sd_addr), .sd_len(ic_sd_len),
	.sd_valid(ic_sd_valid), .sd_rdata(arb_rdata)
);

sys2_sdram_arb #(.AW(24)) pf_arb (
	// Same rom_loaded gate as the prefetcher above: the arbiter must not present a burst to the
	// controller while the loader-writer owns it.
	.clk(clk_sys), .reset(pf_reset),
	// The sprite-bus mask on c0_req is GONE: it guarded c0 sharing the controller's
	// single-word mux with the loader. c0 rides `brst_*` now, which the
	// arbiter owns exclusively, so that collision cannot happen -- and keeping the mask would
	// be harmful, since dropping c0_req mid-burst would clear the head interlock under a
	// running transfer. The download is covered by the icache's own `mem_ready` gate.
	.c0_req(ic_sd_req), .c0_addr(ic_sd_addr), .c0_len(ic_sd_len),
	.c0_valid(ic_sd_valid), .c0_done(ic_sd_done),
	.c1_req(sprc_req), .c1_addr(sprc_addr), .c1_len(sprc_len),
	.c1_req2(sprc_req2), .c1_addr2(sprc_addr2), .c1_len2(sprc_len2), .c1_valid(sprc_valid), .c1_done(sprc_done),
	.c2_req(pf_sd_req), .c2_addr(pf_sd_addr), .c2_len(pf_sd_len),
	.c2_req2(pf_sd_req2), .c2_addr2(pf_sd_addr2), .c2_len2(pf_sd_len2),
	.c2_valid(pf_sd_valid), .c2_done(pf_sd_done),
	.rdata(arb_rdata),
	.sd_addr(arb_sd_addr), .sd_req(arb_sd_req),
	.sd_brst_addr(pf_brst_addr), .sd_brst_len(pf_brst_len), .sd_brst_req(pf_brst_req),
	.sd_brst_abort(pf_brst_abort),
	// Two DIFFERENT return strobes: sd_valid is the controller's shared `valid` (single-word
	// path, client 0) and sd_brst_valid is burst-only (clients 1 and 2). See the notes on
	// sys2_sdram.brst_valid -- hardening against an ordering assumption, not a bug fix;
	// Simulation measured that the two never actually coincide.
	.sd_ready(sdram_ready), .sd_rdata(sdram_rdata),
	.sd_valid(sdram_valid), .sd_brst_valid(pf_brst_valid),
	.sd_brst_done(pf_brst_done), .sd_brst_accept(pf_brst_accept), .sd_brst_start(pf_brst_start)
);


wire [15:0] pf_sdram_data;  // from the SDRAM line buffer
wire [4:0]  pf_sdram_attr;  // ...and that tile's own colour group / priority category

// ---------------------------------------------------------------------------

// Motion-object engine <-> main-bus MO RAM (mob_video_*), sprite graphics ROM
// (sp_*), and the renderer's line-buffer read port (mob_lb_*). All in clk_sys.
wire  [9:0] mob_video_addr;
wire [15:0] mob_video_data;
wire        sp_req, sp_valid;
wire [13:0] sp_tile;
wire  [3:0] sp_row;
wire [63:0] sp_data;
wire  [8:0] mob_lb_col;
wire        mob_lb_rd_en;
wire  [7:0] mob_lb_data;

// ---------------------------------------------------------------------------
// Player controls. Paperboy's cabinet is a handlebar (analog steer X on ADC
// ch0, speed Y on ch1) plus one paper-throw button; coins are read by the 6502.
// We surface the controls active-high here -- the main bus inverts btn1/btn2/
// service into the 014000 switch word, and coin1_n is active-low at the 6502.
//   joystick_0[4]=Throw R  [5]=Throw L  [6]=Coin  [7]=Service Credit
//   (J1 order in CONF_STR; the release MRA <buttons> MUST list them in the SAME
//    order -- an MRA name list overrides the OSD labels, and a mismatch silently
//    drives the wrong core bit.)
// Paperboy has two separate throw inputs -- Button 1 (IN0 bit7) and Button 2
// (IN0 bit6) -- and BOTH throw papers (verified: operator manual TM-275 "press
// either button on the handlebar control to throw papers", distinct left-hand /
// right-hand throw switches; MAME defines both as IPT_BUTTON1 / IPT_BUTTON2).
// They are the cabinet's two grip buttons and likely fling toward OPPOSITE sides
// of the street. So we keep them independent: A -> btn2, B -> btn1; the bus reads
// both bits symmetrically, so btn2 is NOT tied off. (HW: bit7's throw was obvious
// in testing; bit6's was not seen -- it may throw to the RIGHT, or that pad
// button may not be binding; to be measured, not assumed.)
// ---------------------------------------------------------------------------
wire m_right   = joystick_0[0];
wire m_left    = joystick_0[1];
wire m_down    = joystick_0[2];
wire m_up      = joystick_0[3];
wire m_throw_r = joystick_0[4];   // A -> btn2 / IN0 bit6 (a paper throw)
wire m_throw_l = joystick_0[5];   // B -> btn1 / IN0 bit7 (a paper throw)
wire m_coin    = joystick_0[6];   // Select -> coin

// ---------------------------------------------------------------------------
// Per-game IN0 sources. Paperboy uses none of these -- the
// main bus's game_id case ignores them for SYS2_GAME_PAPERBOY, so they are inert
// on the shipping game and cannot regress it.
//
// The Sprints put START1/START2 (and START3 on Super Sprint) on the bits Paperboy
// uses for its two throw buttons, so they reuse the same pad buttons rather than
// getting new ones. APB's second/third buttons land on IN0 bits 1 and 3.
// Player 2's start comes from joystick_1 so a 2-player cabinet maps naturally.
// ---------------------------------------------------------------------------
// These get their OWN pad bits, [8] and [9], rather than sharing Paperboy's.
// An earlier version had m_btn3 on joystick_0[6] and m_start1 on [4]|[7] -- which are the
// COIN and service credit buttons. One bitstream serves all five games, so those bits are
// live on every game: inserting a coin on APB would also press Button 3, and Service Credit
// on a Sprint would also press Start. Generating the per-game MRAs is what surfaced it,
// because the <buttons> list has to name J1 entries in order and there was nothing left to
// name. J1 must cover the UNION of the family's controls, not one game's.
wire m_start1  = joystick_0[8];                  // Sprints: 1P start
wire m_start2  = joystick_1[8];                  // 2P pad
wire m_start3  = joystick_2[8];                  // 3P pad (Super Sprint only)
wire m_btn3    = joystick_0[9];                  // APB third button
// Coin-door service credit button -> 0x1840 bit5, exposed by the sound bus as
// coin3_n (active low). SP-275 sheet 9A has it as the third/aux coin-door input;
// MAME called it IPT_COIN3 until commit 76f3936 reclassified it as
// IPT_SERVICE1, which is what the cabinet actually wires there.
//
// Verified live in the 6502 firmware (disassembly), so this is not a
// dead OSD button: the coin service routine at $6262 loops X=2,1,0 and shifts the
// status byte so carry = bit7 (X=2, COIN R), bit6 (X=1, COIN L), bit5 (X=0, here),
// each with its own debounce slot $37/$36/$35 and its own mechanical counter cell
// $30/$2F/$2E. At $62A6 the slot is special-cased: X=0 contributes exactly ONE coin
// unit with NO multiplier, while X=1/X=2 first apply the "Left Coin"/"Right Coin"
// DIP multipliers -- the classic Atari service-credit behaviour.
// It still goes through coins-per-credit ($2C), so under a 2C/1C setting two
// presses are needed for one credit. It is not an unconditional free credit.
// Debounce needs a sustained press (~156 ms), same as the coin input.
wire m_service_credit = joystick_0[7];

// ---------------------------------------------------------------------------
// Physical option DIP switches (SP-275 sheet 9B, TM-275 tables 1-2/1-3), exposed in the OSD
// "Game Options" page and fed to the two POKEYs' pot lines (ALLPOT). The PCB has an 8-position
// switch at 6/7A (coin options -> POKEY1) and one at 5/6A (game options -> POKEY2). On the PCB
// switch position N wires to POKEY P(8-N). POLARITY (verified against TM-275 Table 1-2 +
// the 6502 ROM, not the earlier "OFF=1" guess): the value the 6502 reads at $2B (POKEY allpot, which
// our POKEY.vhd passes straight through from .PIN -- no inversion) has bit=1 meaning the switch is ON.
// PROOF: Table 1-2 Free Play = sw1,2,3 ON, and the ROM ($63F4) reads Free Play as $2B & 0xE0 == 0xE0
// (top 3 bits = 111). So sw ON -> 1. The byte is { sw1..sw8 } MSB-first. Factory defaults (Table 1-2
// arrows) are all-OFF except sw8 (2C/1C), i.e. dip_coin = 0x01; sim-verified that dip_coin=0x00 (1C/1C)
// gives one-coin-one-credit and dip_coin top3=111 gives free play.
//
// Coin DIP fields (table 1-2): sw1-3 Bonus Adder (On On On = Free Play), sw4 Left coin mech,
// sw5-6 Right coin mech, sw7-8 Coins-Per-Credit. We expose Free Play + Coins/Credit and hold the
// coin-multipliers at the factory 1:1 (OFF=0). Game DIP fields (table 1-3): sw3-4 Lives, sw5-6 Bonus
// life, sw7-8 Difficulty (sw1-2 unused). The OSD 2-bit values are encoded to MATCH the switch pair
// directly (ON=1), so they go in straight (no inversion).
wire       o_free_play   = status[26];
wire [1:0] o_coins_cred  = status[28:27];  // 0:1C/1C 1:2C/1C 2:3C/1C 3:4C/1C  (= sw7,8 value, ON=1)

// Paperboy's factory coinage is 2 COINS/1 credit, and it is the only game that differs.
// Read straight off the arrows in each manual's coin table (each defines as "options
// preset at the factory"): TM-275 Table 1-2 arrows Coins Per Credit at "2 Coins 1 Credit"
// (sw7 Off, sw8 On = value 1), while TM-294 (720), TM-290 (ssprint), TM-292 (csprint) and
// TM-308 (apb) all arrow 1 Coin 1 Credit = value 0. 720's own self-test screen corroborates
// it twice over (TM-294 Fig 3-21 reads "COIN VALUE: 1 COIN 1 CREDIT" and its Operator Hints
// call the factory setting "two coins per play", which is 1C/1C x the 2:1 start ratio).
// MAME's ioport `defvalue` is NOT a transcription of those arrows -- it says 00 for
// paperboy DSW0 and 0x55 for 720 DSW1, and BOTH disagree with the manuals. Do not re-derive
// this from MAME.
//
// MiSTer powers `status` up at 0, so the factory value has to BE index 0. Paperboy therefore
// gets its OWN status bits with the option list permuted by the same XOR applied here, which
// keeps the shipped "menu index == DSW switch value" property intact for the other four
// rather than special-casing their encoding. Separate bits are mandatory, not tidiness: all
// five games share one .rbf and therefore ONE MiSTer config file, so shared bits would make a
// coinage chosen in Paperboy reappear as a different coinage in Super Sprint -- the same shape
// as the old bug that put Super Sprint's Wrenches under the name "Lives".
wire       is_paperboy      = (game_id == SYS2_GAME_PAPERBOY);
wire [1:0] o_coins_cred_pb  = status[97:96] ^ 2'b01;   // index 0 -> DSW value 1 = 2C/1C
wire [1:0] coins_cred_eff   = is_paperboy ? o_coins_cred_pb : o_coins_cred;
wire [1:0] o_difficulty  = status[30:29];  // 0:Med-Hard 1:Easy 2:Medium 3:Hard (= sw7,8 value)
wire [1:0] o_bonus_life  = status[32:31];  // 0:15000 1:None 2:10000 3:20000   (= sw5,6 value)
wire [1:0] o_lives       = status[34:33];  // 0:4 1:Demo 2:3 3:5               (= sw3,4 value)

// ON=1, OFF=0 (the value at $2B). Free play = sw1-3 ON = top3 111; coin-mode No-Bonus = top3 000.
//
// DSW0 is game-independent as far as we expose it: Coins-Per-Credit (sw8,7) and the
// Bonus-Adder Free Play code (sw3,2,1 = on on on) are identical on all five, and sw6,5,4 --
// Paperboy/720/APB's Left+Right coin multipliers, or the Sprints' single "All Coin
// Mechanisms" multiplier -- is held at the factory 1:1, which is 000 under either reading.
wire [7:0] dip_coin = { o_free_play ? 3'b111 : 3'b000, // sw1-3 Bonus Adder: On on on=Free Play, else No Bonus
                        1'b0,                           // sw4  Left coin mech = 1 coin (OFF)
                        2'b00,                           // sw5-6 Right coin mech = 1 coin (off off)
                        coins_cred_eff };                // sw7-8 Coins-Per-Credit (00=1C/1C..11=4C/1C)

// DSW1 is NOT game-independent -- see PB_DIP_MENU for the sourcing and the csprint deviation
// from MAME. Only the field WIDTHS coincide, which is exactly why the Paperboy-shaped pack
// this replaced looked right: on Super Sprint it was setting Wrenches under the name "Lives".
// Each game reads its own status bits, so switching games cannot reinterpret the previous
// game's settings (all five share one .rbf and therefore one MiSTer config file).
wire [1:0] o720_bonus    = status[65:64];
wire [1:0] o720_diff     = status[67:66];
wire [1:0] o720_maxadd   = status[69:68];
wire [1:0] o720_coinsreq = status[71:70];
wire [1:0] oss_diff      = status[73:72];
wire [1:0] oss_obstacles = status[75:74];
wire [1:0] oss_wrenches  = status[77:76];
wire [1:0] ocs_diff      = status[79:78];
wire [1:0] ocs_hazard    = status[81:80];
wire [1:0] ocs_wrenches  = status[83:82];
wire       ocs_hsreset   = status[84];
wire       oapb_attract  = status[85];
wire [1:0] oapb_continue = status[87:86];
wire [2:0] oapb_diff     = status[90:88];
wire [1:0] oapb_coinsreq = status[92:91];

logic [7:0] dip_game;
always @* begin
	case (game_id)
		// 720: sw2,1 Coins Required / sw4,3 Max Add. A. Coins / sw6,5 Difficulty / sw8,7 Bonus Life
		SYS2_GAME_720:
			dip_game = { o720_coinsreq, o720_maxadd, o720_diff, o720_bonus };
		// ssprint: sw2,1 unused / sw4,3 Wrenches / sw6,5 Obstacles / sw8,7 Difficulty
		SYS2_GAME_SSPRINT:
			dip_game = { 2'b00, oss_wrenches, oss_obstacles, oss_diff };
		// csprint (TM-292 Table 1-3): sw1 Auto High Score Reset / sw2 unused /
		// sw4,3 Wrenches / sw6,5 Track Hazard Difficulty / sw8,7 Drone Difficulty
		SYS2_GAME_CSPRINT:
			dip_game = { ocs_hsreset, 1'b0, ocs_wrenches, ocs_hazard, ocs_diff };
		// apb: sw2,1 Coins Required / sw5,4,3 Difficulty / sw7,6 Max Continues / sw8 Attract Lights
		SYS2_GAME_APB:
			dip_game = { oapb_coinsreq, oapb_diff, oapb_continue, oapb_attract };
		// Paperboy: sw2,1 unused / sw4,3 Lives / sw6,5 Bonus Life / sw8,7 Difficulty
		default:
			dip_game = { 2'b00, o_lives, o_bonus_life, o_difficulty };
	endcase
end

// Handlebar: an analog stick drives each ADC axis proportionally (centre 0x80 via the sign-bit
// flip, matching the chip's idle value); with no stick the D-pad slams the axis to its extreme.
//
// The D-pad arm must agree with its own analog axis, and for a long time the Y one did not.
// These were two separate inline expressions, and they disagreed about what 0xff means: X gave
// 0xff to `m_right`, Y gave 0xff to `m_up`. But 0xff is whatever the ANALOG flip produces at the
// positive end, and on a pad positive X is right while positive Y is down. So X agreed with its
// stick and Y was inverted against it. Hardware, Paperboy: left/right played fine while up was
// BRAKE and down was GAS, with the stick correct in both axes -- exactly that asymmetry.
//
// Positive Y is down, and that is MEASURED, not assumed: the same sign convention is what
// made the Sprints accelerate when the stick was pushed DOWN (see the gas note below). One
// hardware fact, two bugs, and this is the axis where it is written down.
//
// Now one module, instantiated per axis, so the only per-axis decision left is naming which
// D-pad bit is the axis's POSITIVE end. Two axes can no longer drift apart.
// `dig_pos`/`dig_neg` are named by axis end, not by screen direction -- which is why the Y
// instance reads m_down as positive. That is not a typo; see sys2_analog_axis.
// ---------------------------------------------------------------------------
// Is the player holding the stick? (from a hardware report)
//
// `sys2_analog_axis` ignores the D-pad arm while a stick is live, so this predicate decides
// which arm the player gets. It was `joystick_l_analog_0 != 16'h0000` -- exact zero -- and a
// real thumbstick does not return to exact zero: it settles at +/-1..3. So the first touch of
// the stick locked the D-pad out until the stick happened to land on 0,0 EXACTLY, which is
// precisely the reported symptom: after using the thumbstick the D-pad "sometimes" works
// again straight away and sometimes does not.
//
// The threshold must match what that axis's consumer ignores, or a band opens where the
// stick counts as live -- D-pad locked out -- and yet produces nothing, which is the same bug
// one step smaller. The ADC axes are POSITION, with no rate deadzone below them, so they take
// a fixed rest allowance; the LETA steering axes take the OSD's own `Steering deadzone`, so
// the two can never disagree about what "centred" means.
//
// Derived from the WHOLE stick word, per sys2_analog_axis's `stick_active` contract: a player
// holding a pure-X deflection still has a live stick and its Y really is centred.
// Sign-extend to 9 bits BEFORE negating -- -128 has no positive counterpart in 8 bits, the
// same trap sys2_analog_spin documents at `ax9`.
function automatic logic stick_live(input logic [15:0] word, input logic [8:0] dz);
	logic [8:0] ax, ay, mx, my;
	begin
		ax = {word[7],  word[7:0]};
		ay = {word[15], word[15:8]};
		mx = ax[8] ? (9'd0 - ax) : ax;
		my = ay[8] ? (9'd0 - ay) : ay;
		stick_live = (mx > dz) || (my > dz);
	end
endfunction

// Rest jitter only. Matches sys2_analog_spin's own documented allowance for a springy pad.
localparam logic [8:0] STICK_REST_DZ = 9'd12;

wire        stick_active = stick_live(joystick_l_analog_0, STICK_REST_DZ);
wire [7:0]  steer_x, speed_y;

sys2_analog_axis u_axis_x (
	.stick_active (stick_active),
	.axis         (joystick_l_analog_0[7:0]),
	.dig_pos      (m_right),
	.dig_neg      (m_left),
	.value        (steer_x)
);

sys2_analog_axis u_axis_y (
	.stick_active (stick_active),
	.axis         (joystick_l_analog_0[15:8]),
	.dig_pos      (m_down),    // positive Y is DOWN on the pad -- confirmed on hardware
	.dig_neg      (m_up),
	.value        (speed_y)
);

// ---------------------------------------------------------------------------
// Gas is a button, not just a direction (from a hardware report).
//
// The digital accelerate fallback was `joystick_0[3]` alone -- the D-pad up bit. On a pad that
// is the wrong shape twice over: a cabinet's foot pedal is not a direction, and on the Sprints
// and APB the same stick is also the steering wheel, so "up" fights the axis the player is
// already using. There was no button that did it at all.
//
// Which slot is free is per-game, so the choice is too (see sys2_main_bus's switch_word):
//   ssprint/csprint  use only start1/2/3, so J1 slot 1 = joystick_*[4] is free  -> Gas
//   apb              already uses [4] as Button 2 and [9] as Button 3, so slot 2 = [5] -> Gas
// Paperboy and 720 never call pedal() -- Paperboy's throttle is its handlebar ADC axis.
//
// The D-pad up bit is deliberately *not* or'd IN. It was, briefly, to keep older mappings
// working -- and hardware showed why that is wrong: UP is also how you move through the game's
// own menus, so an accelerate-on-up makes level select hard to operate. A control that doubles
// as a menu direction is not a spare input. Gas is the button and only the button.
//
// And that principle was only half applied at first. The reasoning above -- "the
// same stick is also the STEERING WHEEL, so up fights the axis the player is already using" --
// argues just as strongly against the left stick's analog Y, which stayed wired to
// sys2_analog_map as the analog pedal. Hardware, Super Sprint: pressing DOWN on the stick
// accelerated. Positive Y is DOWN on the pad, and no polarity fixes it, because the coupling is
// the problem: X on that stick is the wheel, and a player mid-corner cannot hold Y at zero.
// The stick_* ports are gone; gas reaches sys2_analog_map only through press_*.
// If analog throttle is ever wanted back, it belongs on the RIGHT stick -- an axis the
// steering does not already own -- not on this one.
wire apb_gas = (game_id == SYS2_GAME_APB);
wire m_gas_0 = apb_gas ? joystick_0[5] : joystick_0[4];
wire m_gas_1 = apb_gas ? joystick_1[5] : joystick_1[4];
wire m_gas_2 = apb_gas ? joystick_2[5] : joystick_2[4];

// APB's FIRE and SIREN were SWAPPED in an earlier revision of this core.
// The bus's bit numbers were always right -- sys2_main_bus puts btn2 on IN0 bit1 and btn3 on
// bit3, matching MAME's 0x02/0x08 -- but the joystick bits feeding them were inverted, so the
// button the MRA labels "Siren" sounded nothing and fired instead.
//
// SP-308 is the authority, and it is unambiguous:
//   sheet 1B "Control Panel"  S5A/S5B = FIRE  (J24 pin 7, orange)
//                             S3A/S3B = SIREN (J24 pin 8, yellow)
//   sheet 6A "Control Panel Inputs"  SW5 -> LS244 5P in 11 -> out 9  = DAL1  (IN0 bit 1)
//                                    SW3 -> LS244 5P in 13 -> out 7  = DAL3  (IN0 bit 3)
//   sheet 3A memory map      014000 bit1 = SW5, bit3 = SW3
// => FIRE = IN0 bit1 = MAME BUTTON2, SIREN = IN0 bit3 = MAME BUTTON3.
//
// Neither MAME nor the TM can tell you this. APB's PORT_MODIFY uses bare IPT_BUTTON2 /
// IPT_BUTTON3 with no PORT_NAME, and TM-308's prose only says the panel has "a SIREN button, a
// FIRE button, and a gas pedal" -- true, and silent on which is which. Two sources agreed and
// neither answered the question. Only the wiring diagram does.
//
// The MRA labels are the contract (J1 slot 1 = "Siren", slot 6 = "Fire"), so the fix is here,
// in the mapping, not in the labels.
wire m_apb_btn2 = apb_gas ? m_btn3    : m_throw_r;   // APB: Fire  -> IN0 bit1 (BUTTON2)
wire m_apb_btn3 = apb_gas ? m_throw_r : m_btn3;      // APB: Siren -> IN0 bit3 (BUTTON3)

// Per-game ADC0809 channel map. The table itself lives in sys2_analog_map so it can be
// gated without elaborating `emu`; see that module for the channel assignment, and for
// the VERIFY on the Sprint/APB pedal calibration.
logic [7:0] adc_in [0:7];
sys2_analog_map u_analog_map (
	.game_id (game_id),
	.steer_x (steer_x),
	.speed_y (speed_y),
	// No stick_* ports any more: the left stick's Y was the analog pedal in an earlier revision,
	// when hardware showed DOWN accelerating on Super Sprint. That stick's X is the STEERING
	// WHEEL (it feeds sys2_analog_spin below), so its Y cannot be the throttle at any
	// polarity. Gas is press_* and nothing else. See sys2_analog_map's pedal().
	.press_0 (m_gas_0),
	.press_1 (m_gas_1),
	.press_2 (m_gas_2),
	.adc_out (adc_in)
);

// DEC T-11 main CPU. Boots from mode word 0x36ff (PC = 0x8000), runs on the /2
// cpu_clk_en (10 MHz exec) in the clk_t11 domain, and is held in reset until the
// ROM download completes (cpu_reset). Coded interrupts arrive on cpu_cp.
wire        bus_ifetch;
wire        cpu_reset_out, cpu_halted, cpu_waiting, cpu_retire;

t11_core cpu
(
	.clk         (clk_t11),
	.reset       (cpu_reset),
	.ce          (cpu_clk_en),
	.bus_req     (bus_req),
	.bus_write   (bus_write),
	.bus_ifetch  (bus_ifetch),
	.bus_addr    (bus_addr),
	.bus_wdata   (bus_wdata),
	.bus_byte_en (bus_byte_en),
	.bus_rdata   (bus_rdata),
	.bus_ack     (bus_ack),
	.bus_error   (bus_error),
	.cp          (cpu_cp),
	.halt        (1'b0),
	.power_fail  (1'b0),
	.reset_out   (cpu_reset_out),
	.halted      (cpu_halted),
	.waiting     (cpu_waiting),
	.retire      (cpu_retire)
);

wire signed [15:0] ym_left, ym_right;
wire signed [5:0]  pokey1_snd, pokey2_snd;
wire signed [13:0] tms_snd;
wire [8:0]         mix_gain_ym, mix_gain_pk, mix_gain_tms;  // 0x187a per-chip gains, Q0.8 (256=unity)
wire        snd_eeprom_wr;         // pulses on every 6502 write to the EEPROM (settings/high score)
wire  [7:0] snd_eeprom_dump_data;  // live EEPROM byte at ioctl_addr, for the NVRAM upload

// ---------------------------------------------------------------------------
// LETA quadrature inputs.
//
// leta_present comes straight from the game descriptor: Paperboy (game 0) has no LETA
// and its window must read open bus.
//
// Rather than writing spinner deltas into the counter -- which is what MAME does, and
// which would bypass the 156 kHz sample rate, the 2x decode and the resolution bit --
// each channel gets a `sys2_quad_gen` that emits REAL quadrature edges for the real
// `sys2_leta` to count. Stepping on the chip's own phi2 (exported as leta_ce_phi2)
// guarantees every emitted edge is sampled exactly once.
//
// Channel map, from each game's MAME PORT_MODIFY("LETAn"):
//
//   channel   720            ssprint      csprint      apb        paperboy
//   LETA0     centre disc    P1 wheel     P1 wheel     P1 wheel   --
//   LETA1     rotate disc    P2 wheel     P2 wheel     --         --
//   LETA2     --             P3 wheel     --           --         --
//   LETA3     --             --           --           --         --
//
// 720's ROTATE disc is on LETA1, not LETA0 -- so on 720 spinner_0 (the player's one
// control) feeds channel 1, and channel 0 is the separate 2-tooth CENTRE disc.
//
// Both discs are on ONE physical shaft, so the centre channel is DERIVED from the same
// spinner rather than given its own input: 2 teeth against the rotate disc's 72, both
// read at 2x, is 4 centre counts per revolution against 144 -- exactly 1:36. See
// sys2_center_disc. What is OPEN is the DISTRIBUTION of those
// four counts, NOT a phase offset: MAME emits all four within +/-3 of the top, we emit one
// every 36. Same rate, different waveform. The full reasoning and the cheap hardware test
// (720's Control Test: how many angles produce ZEROED?) are in sys2_center_disc's header --
// read it before changing anything here.
//
// Unused channels stay idle-high: the schematic pulls all eight inputs up through R67.
// ---------------------------------------------------------------------------
wire       leta_present = (game_id != SYS2_GAME_PAPERBOY);
wire       leta_ce_phi2;

// Which spinner drives each channel. 720 is the exception: its single control is the
// rotate disc on channel 1.
wire is_720 = (game_id == SYS2_GAME_720);
wire [8:0] spin_ch0 = is_720 ? 9'd0     : spinner_0;   // 720 ch0 comes from the divider below
wire [8:0] spin_ch1 = is_720 ? spinner_0 : spinner_1;
wire [8:0] spin_ch2 = spinner_2;

// ---------------------------------------------------------------------------
// Analog stick -> steering. Reported on hardware: on a gamepad, steering did
// not work on ANY of the four LETA games -- only Paperboy, whose handlebar is an ADC axis.
// Cause was structural rather than subtle: these generators were driven ONLY by MiSTer's
// `spinner_*`, which a thumbstick never moves, and the analog stick reached only
// `sys2_analog_map` (Paperboy's handlebar and the Sprint/APB pedals). `sys2_quad_gen`'s
// `delta`/`delta_valid` port was written for exactly this and had never been fed by
// anything but the centre-disc divider below.
//
// Rate conversion (deflection -> turn rate) lives in sys2_analog_spin; see its header for
// why a springing thumbstick cannot map one-to-one onto a continuous-rotation encoder, and
// for the deliberate divergence from MAME's angle mapping on 720.
// Both sources stay live: a real spinner still works, and the analog path is inert when no
// analog controller is connected (an absent stick reads 0 = centre).
// Steering POLARITY is un-asserted, exactly like the ADC axes above -- if a hardware
// check shows a game steering the wrong way, flip its `invert` bit here rather than
// negating inside the converter, so the per-game facts stay in one place.
// And the D-pad still did not steer them, also reported from hardware: the D-pad worked in
// Paperboy and nowhere else, while a thumbstick worked everywhere. The fix above wired the
// analog STICK in and stopped there -- these instances read `joystick_l_analog_*` RAW, so the
// digital arm had no route to the encoder at all. Paperboy was unaffected for the third time
// for the same structural reason: its handlebar goes through sys2_analog_axis, which merges
// both arms, and nothing else did.
//
// The fix is to feed the SAME merged axis Paperboy already uses, not a second blend.
// sys2_analog_axis emits the ADC0809's offset-binary scale (centre 0x80) and this module wants
// MiSTer's SIGNED one (centre 0x00), so the value is XORed back: `^ 8'h80` is exactly the
// inverse of the flip inside the module, so the analog path is bit-identical to what it was
// (`(a ^ 8'h80) ^ 8'h80 == a`) and the digital ends land on full scale -- 0xff -> 0x7f = +127,
// 0x00 -> 0x80 = -128 -- well past this module's DEADZONE of 12. One merge implementation for
// every axis in the core; see sys2_analog_axis on why two would drift.
//
// Player 0 gets its OWN steering axis rather than reusing `steer_x`. They are
// the same stick and the same D-pad bits, but not the same "is the stick live" threshold:
// `steer_x` is an ADC position axis whose live-test is rest jitter alone, while these three
// feed a RATE converter that ignores everything inside `steer_deadzone`. Sharing the instance
// would have pinned all three steering axes to the ADC's threshold and reopened the band where
// the stick is live but produces nothing. One instance per consumer, each asking about its own
// deadzone -- see stick_live() at the ADC axes above.
wire [7:0]  steer_x0, steer_x1, steer_x2;

sys2_analog_axis u_axis_steer0 (
	.stick_active (stick_live(joystick_l_analog_0, steer_deadzone)),
	.axis         (joystick_l_analog_0[7:0]),
	.dig_pos      (m_right),
	.dig_neg      (m_left),
	.value        (steer_x0)
);

sys2_analog_axis u_axis_steer1 (
	.stick_active (stick_live(joystick_l_analog_1, steer_deadzone)),
	.axis         (joystick_l_analog_1[7:0]),
	.dig_pos      (joystick_1[0]),   // [0] = right = this axis's positive end
	.dig_neg      (joystick_1[1]),   // [1] = left
	.value        (steer_x1)
);

sys2_analog_axis u_axis_steer2 (
	.stick_active (stick_live(joystick_l_analog_2, steer_deadzone)),
	.axis         (joystick_l_analog_2[7:0]),
	.dig_pos      (joystick_2[0]),
	.dig_neg      (joystick_2[1]),
	.value        (steer_x2)
);

// OSD steering feel (PB_STEER_MENU; the four LETA games only). The menu-to-value table lives
// HERE rather than in sys2_analog_spin: the module stays a rate converter that knows nothing
// about status bits, the same split as the per-game `invert` beside it.
//
// Rates at full deflection with f_tick = 156.25 kHz phi2, and what each is on 720's 144-count
// rotate disc:
//   Normal  acc_bits 15  ~548 counts/s  3.8 rev/s   <- index 0, the rate that has always shipped
//   High             14  ~1097          7.6
//   Low              16  ~274           1.9
//   Lowest           17  ~137           0.95
// Deadzone is in axis units against a +/-127 stick. Index 0 keeps the 12 that shipped; Small is
// for a tight stick, the two large ones for a worn one that no longer returns to centre.
// Both default to index 0 = the previous fixed behaviour, so a player who never opens the
// menu gets exactly what they had. Shared across all five games -- MiSTer keeps one .cfg per
// CORE, not per MRA -- which is why the values are absolute rather than per-game trims.
wire [1:0] steer_rate_sel = status[99:98];
wire [1:0] steer_dz_sel   = status[101:100];
wire [4:0] steer_acc_bits = (steer_rate_sel == 2'd1) ? 5'd14 :
                            (steer_rate_sel == 2'd2) ? 5'd16 :
                            (steer_rate_sel == 2'd3) ? 5'd17 : 5'd15;
wire [8:0] steer_deadzone = (steer_dz_sel   == 2'd1) ? 9'd4  :
                            (steer_dz_sel   == 2'd2) ? 9'd24 :
                            (steer_dz_sel   == 2'd3) ? 9'd40 : 9'd12;

wire signed [8:0] ana_d0, ana_d1, ana_d2;
wire              ana_v0, ana_v1, ana_v2;
sys2_analog_spin u_aspin0 (
	.clk(clk_t11), .reset(reset), .tick(leta_ce_phi2),
	.axis(steer_x0 ^ 8'h80), .invert(1'b0),
	.acc_bits(steer_acc_bits), .deadzone(steer_deadzone),
	.delta(ana_d0), .delta_valid(ana_v0)
);
sys2_analog_spin u_aspin1 (
	.clk(clk_t11), .reset(reset), .tick(leta_ce_phi2),
	.axis(steer_x1 ^ 8'h80), .invert(1'b0),
	.acc_bits(steer_acc_bits), .deadzone(steer_deadzone),
	.delta(ana_d1), .delta_valid(ana_v1)
);
sys2_analog_spin u_aspin2 (
	.clk(clk_t11), .reset(reset), .tick(leta_ce_phi2),
	.axis(steer_x2 ^ 8'h80), .invert(1'b0),
	.acc_bits(steer_acc_bits), .deadzone(steer_deadzone),
	.delta(ana_d2), .delta_valid(ana_v2)
);

// 720's centre disc. Fed from the SAME rotate movement that drives the rotate channel --
// one shaft, so they cannot drift apart. Inert on every other game: ctr_valid only pulses
// when the divider is handed movement, and only 720 hands it any.
// The two rotate sources are merged with the spinner winning a same-cycle tie. A tie
// costs ONE analog count and cannot drift systematically (the analog path emits at most
// ~548/s and spinner updates are sporadic), which is the same order as the count loss a
// real encoder suffers at speed. Do NOT "fix" it by summing: sys2_center_disc takes one
// delta per valid, and summing two independent streams into one pulse loses the other.
logic       spin0_tog_d;
wire        spin0_new   = (spinner_0[8] != spin0_tog_d);
wire signed [8:0] spin0_delta = {spinner_0[7], spinner_0[7:0]};
always_ff @(posedge clk_t11) spin0_tog_d <= spinner_0[8];

wire signed [8:0] rot720_delta = spin0_new ? spin0_delta : ana_d0;
wire              rot720_valid = (spin0_new || ana_v0) && is_720;

wire signed [8:0] ctr_delta;
wire              ctr_valid;
sys2_center_disc u_center (
	.clk       (clk_t11),
	.reset     (reset),
	.rot_delta (rot720_delta),
	.rot_valid (rot720_valid),
	.ctr_delta (ctr_delta),
	.ctr_valid (ctr_valid)
);

wire [2:0] qx, qy;
wire signed [15:0] qbl0, qbl1, qbl2;

// Analog routing follows the SAME channel map as the spinners above, so 720's exception
// applies to both: its one control is the ROTATE disc on channel 1, and channel 0 is the
// derived centre disc. The wheel games put player N's wheel on channel N.
//
//   channel   720                    ssprint / csprint / apb
//   0         centre disc (derived)  P1 wheel <- stick 0
//   1         rotate <- stick 0      P2 wheel <- stick 1
//   2         --                     P3 wheel <- stick 2 (ssprint only)
wire signed [8:0] q0_delta = is_720 ? ctr_delta : ana_d0;
wire              q0_valid = is_720 ? ctr_valid : ana_v0;
wire signed [8:0] q1_delta = is_720 ? ana_d0    : ana_d1;
wire              q1_valid = is_720 ? ana_v0    : ana_v1;

sys2_quad_gen u_quad0 (
	.clk(clk_t11), .reset(reset), .ce_step(leta_ce_phi2),
	.spinner(spin_ch0), .delta(q0_delta), .delta_valid(q0_valid),
	.quad_x(qx[0]), .quad_y(qy[0]), .backlog(qbl0)
);
sys2_quad_gen u_quad1 (
	.clk(clk_t11), .reset(reset), .ce_step(leta_ce_phi2),
	.spinner(spin_ch1), .delta(q1_delta), .delta_valid(q1_valid),
	.quad_x(qx[1]), .quad_y(qy[1]), .backlog(qbl1)
);
sys2_quad_gen u_quad2 (
	.clk(clk_t11), .reset(reset), .ce_step(leta_ce_phi2),
	.spinner(spin_ch2), .delta(ana_d2), .delta_valid(ana_v2),
	.quad_x(qx[2]), .quad_y(qy[2]), .backlog(qbl2)
);

// Channel 3 is unused on every game. Channel 0 is live on all of them now: the wheel
// games drive it from spinner_0 and 720 from the centre-disc divider.
wire [3:0] leta_x = {1'b1, qx[2], qx[1], leta_present ? qx[0] : 1'b1};
wire [3:0] leta_y = {1'b1, qy[2], qy[1], leta_present ? qy[0] : 1'b1};

sys2_sound_bus sound_bus
(
	.leta_present (leta_present),
	.game_id      (game_id),
	.tms_present  (game_flags[SYS2_FLAG_TMS_PRESENT]),
	// Electromechanical coin counters and cabinet lamps: no MiSTer pins, deliberately open.
	// Written as explicit empty connections rather than omitted, so lint's unconnected-pin
	// check stays meaningful -- an omitted pin is how the SDRAM burst port went missing.
	.coin_counter (),
	.lamp         (),
	.leta_x       (leta_x),
	.leta_y       (leta_y),
	.leta_ce_phi2 (leta_ce_phi2),
	.clk             (clk_t11),
	.reset           (reset),
	.snd_cpu_en      (snd_cpu_en),
	.snd_ym_en       (snd_ym_en),
	.snd_ym_cen_p1   (snd_ym_cen_p1),
	.snd_pokey_en    (snd_pokey_en),
	.snd_reset       (snd_reset | ~audio_loaded), // 6502 boots the moment its ROM is in -- head start
	.snd_cmd         (snd_cmd),
	.snd_cmd_pending (snd_cmd_pending),
	.resp_pending    (resp_pending),
	.snd_resp        (snd_resp),
	.snd_cmd_read    (irq_snd_cmd_read),
	.snd_resp_write  (irq_snd_resp_write),
	.coin1_n         (~m_coin),
	.coin2_n         (1'b1),
	.coin3_n         (~m_service_credit),   // 0x1840 bit5 = coin-door service credit
	.self_test_n     (~service_t11),
	.dip_coin        (dip_coin),             // coin-option DIP (6/7A) -> POKEY1
	.dip_game        (dip_game),             // game-option DIP (5/6A) -> POKEY2
	.audio_rom_addr  (audio_rom_addr),       // -> sys2_audio_rom_bram (6502 address)
	.audio_rom_data  (audio_rom_data),       // <- fetched program byte
	.audio_data_ready(audio_data_ready),     // <- gates snd_cpu_en (stall on cache-miss fetch)
	.eeprom_ld_we    (sl_wr_eeprom | sl_wr_nvram), // factory fill (index 0) or NVRAM restore (index 2)
	.eeprom_ld_addr  (ioctl_addr[8:0]),
	.eeprom_ld_data  (ioctl_dout),
	.eeprom_wr       (snd_eeprom_wr),        // -> nvram_dirty (settings/high-score byte written)
	.eeprom_dump_addr(ioctl_addr[8:0]),      // upload readback address (HPS paces ioctl_addr)
	.eeprom_dump_data(snd_eeprom_dump_data), // -> ioctl_din during the save
	.ym_left         (ym_left),
	.ym_right        (ym_right),
	.pokey1_snd      (pokey1_snd),
	.pokey2_snd      (pokey2_snd),
	.tms_snd         (tms_snd),
	.mix_gain_ym     (mix_gain_ym),         // 0x187a Stage-1 analog-mixer gains (Q0.8, 256 = unity)
	.mix_gain_pk     (mix_gain_pk),
	.mix_gain_tms    (mix_gain_tms)
);

// ---------------------------------------------------------------------------------------------
// EEPROM save-back (NVRAM index 2, declared <nvram index="2" size="512"/> in the MRA). Without
// this the 2804 EEPROM's coin settings, game options, and high scores revert to the factory
// image (loaded via sl_wr_eeprom above) every power cycle. HPS actually writes the save file
// only while the OSD is open, so a slow heartbeat is enough: nvram_dirty latches on any 6502
// EEPROM write and clears once HPS starts reading this channel back (snd_eeprom_dump_data is
// live, so nothing is lost by clearing early); nvram_req toggles on a free-running counter
// whenever dirty, which re-arms hps_io's edge-triggered ioctl_upload_req every ~2 s until the
// pending save is picked up.
// ---------------------------------------------------------------------------------------------
reg         nvram_dirty;
always @(posedge clk_sys) begin
	if (reset)                                        nvram_dirty <= 1'b0;
	else if (snd_eeprom_wr)                            nvram_dirty <= 1'b1;
	else if (ioctl_upload && (ioctl_index == 16'd2))   nvram_dirty <= 1'b0;
end

reg [25:0] nvram_hb;
always @(posedge clk_sys) nvram_hb <= reset ? 26'd0 : nvram_hb + 26'd1;

assign ioctl_upload_req   = nvram_dirty & nvram_hb[25];
assign ioctl_upload_index = 8'd2;
assign ioctl_din          = snd_eeprom_dump_data;

// Stereo sound mixer (after the reference ATARISYS1 p_volmux): each channel sums the YM2151
// full-resolution output with one POKEY (POKEY 1 -> left, POKEY 2 -> right) and the (mono) TMS5220
// speech chip, saturated to 16-bit.
//
// LEVELS (schematic-derived rebalance): SP-275 sheet 9B "Audio Output Drivers" sums each
// chip into the TDA2002 through its own input resistor -- YM (YAM) R141/R144=47K, POKEY (PAUD)
// R139/R142=47K, TMS (TIAUD) R140/R143=68K -- so the board's intended weights are
// YM : POKEY : TMS = 1.00 : 1.00 : 0.69 (47/68). The per-chip preamps normalise the chips to a common
// level BEFORE that equal-weight sum (the small YM3012 is gained up by 9J/K's 100K feedback; the large
// POKEY aud pin is attenuated ~0.47x by 10C's 2.2K/4.7K), so a full YM and a full POKEY are meant to
// be EQUALLY loud. Our cores already emit normalised full-scale, so we match that with EQUAL full-scale
// for YM and POKEY.
//   This base was previously by-ear: YM at 3/4 FS vs POKEY <<8 (~1/4 FS) = ~3:1 YM:POKEY, which made
//   music dominate on HW (SFX/speech buried under it). Now YM at 1/2 FS (>>1) and POKEY <<9 (~1/2 FS)
//   => ~1:1. The POKEY term MUST stay a bipolar AC value: its offset-binary silence sits at -32 and the
//   MiSTer audio path's DC filter removes the small idle DC; a UNIPOLAR term here would add a
//   near-full-scale DC step onto the YM and hard-clip the 18->16 saturate (the popping bug).
//   Peak YM+POKEY ~= 0.98 FS, the same headroom as the old 3/4+1/4 split, so the saturate stays a
//   backstop only.
//
// TMS5220 speech (tms_snd, signed 14-bit) is mono on real hardware (a single "TIAUD" net feeding
// the shared resistor mixer, sheet 9A) -- summed into BOTH channels equally.
//
// 0x187a programmable gain (D5): the fixed `tms_snd <<2` above was a hand-tuned stand-in
// for the PCB's Stage-1 per-chip resistor networks, which we now model. sys2_sound_bus decodes
// 0x187a into three Q0.8 gains (mix_gain_ym/pk/tms, 256 = unity) via the MAME atarisy2 mixer_w
// resistor model; each chip's base term below is scaled by its gain. Two consequences:
//   (1) The TMS `<<2` becomes the *unity* (loudest) speech level; the game runs speech at TMS bits
//       5-7 = 000 (gain 0.5, verified in MAME: 0x187a only ever = 0x1f/0x1e), so typical speech now
//       plays at <<2 * 0.5 = <<1 -- i.e. 6 dB quieter than before, matching the authentic ~0.65x YM
//       ratio (schematic 47/68 = 0.69x; MAME TMS~0.63x YM). Louder shouts (game raising the TMS bits)
//       scale up toward the <<2 reference. This is the "speech ran 6 dB hot" fix.
//   (2) The game DUCKS the music under speech by lowering the YM bits 0-2 (0x1f->0x1e = YM 1.0->0.885
//       during a boot phrase); that ducking envelope now takes effect instead of being ignored.
// At unity code (all bits 1) each term reduces to its rebalanced base above; the 0x187a network only
// attenuates from there (gains <= unity). The saturate stays a backstop.
wire signed [17:0] ym_l18  = {{2{ym_left[15]}},  ym_left};
wire signed [17:0] ym_r18  = {{2{ym_right[15]}}, ym_right};

// Base per-chip terms (schematic-derived, see LEVELS above): YM at 1/2 FS and POKEY <<9 (~1/2 FS) =>
// ~1:1 (SP-275 sheet 9B equal 47K sums); TMS <<2 (mono -> summed into both channels equally; real
// speech is well under its 8191 FS, so <<2 is a deliberate boost, knocked to ~0.69x by its 0x187a gain).
// Schematic-accurate chain (derived from SP-275 sheets
// 9A/9B). Replaces the previous "base terms -> 0x187a gain -> sum -> one 12 kHz pole + one 9.8 Hz
// DC blocker" topology, which could not be right in principle: the board filters each chip
// separately, with different corners, before the summing node. Each chip now runs its own
// sys2_audio_mix (see rtl/sound/sys2_audio_mix.sv: per-chip strips = preamp pole ->
// interstage DC block -> 0x187a gain -> output-stage pole, then summing weights + output network),
// then the three are summed through their real 47K/47K/68K resistor weights, then the shared output
// network runs.
//
// Analog audio chain -- SP-275 sheets 9A/9B. Extracted to rtl/sound/sys2_audio_mix.sv so that
// the top level and every verification bench instantiate the SAME logic. It used to be inline
// here with the constants re-typed in three other places, which is how a 9.2 dB speech
// regression reached hardware: the top level and the model it was checked against disagreed
// and nothing said so. Do not re-inline it.
sys2_audio_mix u_audio_mix (
	.clk        (clk_sys),
	.ce         (snd_pokey_en),
	.rst        (reset),
	.ym_l18     (ym_l18),
	.ym_r18     (ym_r18),
	.pokey1_snd (pokey1_snd),
	.pokey2_snd (pokey2_snd),
	.tms_snd    (tms_snd),
	.gain_ym    (mix_gain_ym),
	.gain_pk    (mix_gain_pk),
	.gain_tms   (mix_gain_tms),
	.audio_l    (AUDIO_L),
	.audio_r    (AUDIO_R)
);


sys2_main_bus main_bus
(
	.slapstic_type  (slapstic_type),   // from the MRA index-1 game descriptor
	.clk                (clk_t11),
	.video_clk          (clk_sys),
	.reset              (cpu_hold),
	.cpu_clk_en         (cpu_clk_en),
	.irq_snd_cmd_read   (irq_snd_cmd_read),
	.irq_snd_resp_write (irq_snd_resp_write),
	.irq_scanline_32v   (irq_scanline_t11),
	.irq_vblank         (irq_vblank_t11),
	.snd_resp           (snd_resp),
	.bus_req            (bus_req),
	.bus_write          (bus_write),
	.bus_addr           (bus_addr),
	.bus_wdata          (bus_wdata),
	.bus_byte_en        (bus_byte_en),
	.bus_rdata          (bus_rdata),
	.bus_ack            (bus_ack),
	.bus_error          (bus_error),
	.rom_req            (cpu_rom_req),
	.rom_addr           (cpu_rom_addr),
	.rom_rdata          (rom_rdata),
	.rom_ack            (rom_ack),
	.alpha_video_addr   (alpha_video_addr),
	.alpha_video_data   (alpha_video_data),
	.mob_video_addr     (mob_video_addr),
	.mob_video_data     (mob_video_data),
	.playfield_top_video_addr    (pf_top_video_addr),
	.playfield_top_video_data    (pf_top_video_data),
	.playfield_bottom_video_addr (pf_bot_video_addr),
	.playfield_bottom_video_data (pf_bot_video_data),
	// Dedicated tile-prefetch port (sys2_dpram PORT_C). pfram_addr is one address for both
	// halves; pfram_bot picks which result the prefetcher consumes.
	.playfield_top_pf_addr       (pfram_addr),
	.playfield_top_pf_data       (pf_top_pf_data),
	.playfield_bottom_pf_addr    (pfram_addr),
	.playfield_bottom_pf_data    (pf_bot_pf_data),
	.palette_video_addr (palette_video_addr),
	.palette_video_data (palette_video_data),
	.rom_bank0          (rom_bank0),
	.rom_bank1          (rom_bank1),
	.vmmu_bank          (vmmu_bank),
	.cpu_cp             (cpu_cp),
	.irq_enable         (irq_enable),
	.snd_cmd            (snd_cmd),
	.snd_cmd_pending    (snd_cmd_pending),
	.resp_pending       (resp_pending),
	.snd_reset          (snd_reset),
	.btn1               (m_throw_l),   // B / IN0 bit7 -- a paper throw
	// Paperboy/720: A / IN0 bit6 -- a paper throw (BOTH buttons throw, per TM-275).
	// APB: IN0 bit1 = BUTTON2 = FIRE, so this carries Fire there. See the SP-308 note above.
	.btn2               (m_apb_btn2),
	// Per-game IN0 extras; the bus selects on game_id and ignores these for Paperboy.
	.start1             (m_start1),
	.start2             (m_start2),
	.start3             (m_start3),
	// APB: IN0 bit3 = BUTTON3 = SIREN.
	.btn3               (m_apb_btn3),
	.game_id            (game_id),
	.service            (service_t11),
	.adc_clk_en         (adc_clk_en),
	.adc_in             (adc_in),
	.adc_eoc            (adc_eoc),
	.xscroll            (xscroll),
	.yscroll            (yscroll),
	.xscroll_strobe     (xscroll_strobe_t11),
	.yscroll_strobe     (yscroll_strobe_t11),
	.watchdog_reset     (watchdog_reset)
);

sys2_pulse_cdc xscroll_cdc
(
	.src_clk(clk_t11), .src_reset(reset), .src_pulse(xscroll_strobe_t11),
	.dst_clk(clk_sys), .dst_reset(reset), .dst_pulse(xscroll_update_vid)
);
sys2_pulse_cdc yscroll_cdc
(
	.src_clk(clk_t11), .src_reset(reset), .src_pulse(yscroll_strobe_t11),
	.dst_clk(clk_sys), .dst_reset(reset), .dst_pulse(yscroll_update_vid)
);

// The scroll update pulses cross as toggle events; the 16-bit scroll registers
// are independently synchronized into the video domain. The synchronized word is
// stable BEFORE the crossed pulse is edge-detected, and the margin is structural rather
// than a hope about CPU write spacing: sys2_pulse_cdc's `dst_sync[2] ^ dst_sync[1]` asserts
// three dst clocks after the source toggle, while `*_sync` is two flops deep -- so the data
// has been settled for a full cycle when the pulse arrives. The renderer therefore reads
// `*_sync` DIRECTLY.
//
// It used to read a third register, `*_vid`, loaded by that same pulse -- so every
// scroll and bank update was applied one write late. `*_vid <= *_sync` happens at the END of
// the clock the pulse is high, so during that clock the renderer's `yscroll` input still held
// the PREVIOUS write, and `bank1 <= yscroll[3:0]` latched it. The extra register defeated the
// very margin the comment above describes.
//
// Measured in the real `emu` top level, running APB: on EVERY
// update `vid` and `sync` disagreed --
//     YSU f=1840 v=385 vid=00f0 sync=0ef0 bus=0ef0
//     YSU f=1840 v=387 vid=0ef0 sync=00f0 bus=00f0
// -- the renderer trailing the bus by exactly one write, forever.
//
// It hid because consecutive scroll writes almost always carry the SAME value,
// so a one-update lag is invisible in the scroll position. The playfield tile BANK is the
// first field where consecutive writes differ (APB's raster split writes 1,1,2,2,1 in one
// frame), which is what finally made it visible -- as a whole band drawn from the wrong page.
always @(posedge clk_sys) begin
	if (reset) begin
		xscroll_meta <= 16'd0; xscroll_sync <= 16'd0;
		yscroll_meta <= 16'd0; yscroll_sync <= 16'd0;
	end else begin
		xscroll_meta <= xscroll;
		xscroll_sync <= xscroll_meta;
		yscroll_meta <= yscroll;
		yscroll_sync <= yscroll_meta;
	end
end

sys2_video_render video_render
(
	.clk        (clk_sys),
	.reset      (reset),
	.ce_pix     (ce_pix),
	.h_count    (h_count),
	.v_count    (v_count),
	.hblank     (hblank),
	.vblank     (vblank),
	.hsync      (hsync),
	.vsync      (vsync),
	.xscroll    (xscroll_sync),
	.yscroll    (yscroll_sync),
	.xscroll_update(xscroll_update_vid),
	.yscroll_update(yscroll_update_vid),
	.alpha_addr (alpha_video_addr),
	.alpha_data (alpha_video_data),
	.char_addr  (char_addr),
	.char_data  (char_data),
	.pf_top_addr(pf_top_render_addr),
	.pf_top_data(pf_top_video_data),
	.pf_bot_addr(pf_bot_render_addr),
	.pf_bot_data(pf_bot_video_data),
	.pf_rom_addr(pf_rom_addr),
	.sx_fetch_o(sx_fetch_w), .sy_fetch_o(sy_fetch_w),
	.bank0_fetch_o(bank0_fetch_w), .bank1_fetch_o(bank1_fetch_w),
	.pf_rom_col (pf_rom_col_w),
	.pf_rom_data(pf_sdram_data),
	// Colour/priority from the SAME map word as the bitmap (the look-ahead race fix).
	.pf_attr    (pf_sdram_attr),
	.pf_attr_en (1'b1),
	.pal_addr   (alpha_palette_addr),
	.pal_data   (palette_video_data),
	// Motion-object line buffer, driven by sys2_mob_engine (above): the renderer
	// issues the per-pixel read-and-clear column and consumes the composited entry.
	.mob_lb_col   (mob_lb_col),
	.mob_lb_rd_en (mob_lb_rd_en),
	.mob_lb_data  (mob_lb_data),
	.red        (video_red),
	.green      (video_green),
	.blue       (video_blue),
	.hsync_o    (video_hsync),
	.vsync_o    (video_vsync),
	.de_o       (video_de)
);


assign palette_video_addr = alpha_palette_addr;


reg [24:0] heartbeat;
always @(posedge clk_sys) begin
	if (reset) heartbeat <= 0;
	else       heartbeat <= heartbeat + 1'd1;
end

// Bring-up status LED, readable while the screen is black:
//   fast blink = bad game descriptor (MRA index 1): wrong length, unknown game id,
//                reserved bits set, or a slapstic type that disagrees with the game
//                id. An MRA still carrying the v1 single slapstic byte lands here.
//   slow blink = ROM not loaded yet (CPU held in reset)
//   solid on   = ROM loaded, CPU released
// So a black screen with a slow-blinking LED means the load never completed
// (the reset gate is stuck); solid + black means the CPU is running but not
// writing video RAM.
// Solid used to be ambiguous. It was
//     descriptor_bad ? fast : rom_loaded ? SOLID : slow
// so it read SOLID whenever the image had loaded -- including the case where the index-1
// descriptor never arrived at all. That state has descriptor_bad LOW (nothing was seen, so
// nothing was rejected) and descriptor_ok LOW, which holds the T-11 in reset via
// rom_ready = rom_loaded & descriptor_ok. Black screen, CPU held, LED solid: exactly the
// reading that was reported, and it did not distinguish "running" from "still held".
//
// Four states now, and SOLID means the CPU is genuinely out of reset:
//   fast blink  (~0.4 s)  descriptor REJECTED  -- wrong length, unknown game id, reserved
//                         bits set, or a slapstic type disagreeing with the game id
//   medium      (~0.8 s)  ROM loaded but descriptor not validated -- the index-1 payload
//                         never arrived, or arrived incomplete. T-11 still held.
//   slow blink  (~1.6 s)  ROM not loaded yet -- T-11 held
//   SOLID                 T-11 released and running
assign LED_USER = descriptor_bad             ? heartbeat[22] :
                  ~rom_loaded                ? heartbeat[24] :
                  ~descriptor_ok             ? heartbeat[23] :
                                               1'b1;

endmodule
