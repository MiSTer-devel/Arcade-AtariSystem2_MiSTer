// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

// ---------------------------------------------------------------------------
// Atari System 2 download stream layout -- v2, THE single source of truth.
//
// Every region is stretched to the family maximum so these decodes stay
// compile-time constants for all five games (Paperboy, 720, Super Sprint,
// Championship Sprint, APB). A game that does not fill a region leaves 0xff.
//
//   region     offset      size       note
//   maincpu    0x000000    0x090000   even/odd interleave baked into the stream
//   audiocpu   0x090000    0x010000   6502 ROM starts 0x4000 into the region
//   tiles      0x0a0000    0x080000
//   sprites    0x120000    0x100000   XOR 0xff -- the only load transform
//   chars      0x220000    0x004000
//   eeprom     0x224000    0x000200
//                          ---------
//   total      0x224200
//
// The sizes are the ROM_REGION declarations in MAME's atarisy2.cpp, NOT extents
// derived from -listxml: where MAME uses ROM_CONTINUE the XML's last
// offset+size overruns the region end, which is how an earlier draft of this
// layout arrived at the non-existent sizes 0x88000 and 0x108000.
//
// v1 (Paperboy-only, tiles 0x20000 / sprites 0x40000 / total 0x102200) is
// history. It is NOT selectable at runtime -- the MRA and the RTL move together.
//
// Included by sys2_rom_loader.sv and atarisys2.sv. Do not copy these
// numbers anywhere else.
// ---------------------------------------------------------------------------
// NO `ifndef include guard -- deliberately. These are module-scope localparams, not
// macros, and every includer needs its own textual copy. Quartus compiles the whole
// project as ONE compilation unit, so a guard would make the second and third `include
// silent no-ops and leave those modules with the names undeclared. Verilator lints each
// file separately and so never sees that failure mode: it took a real synthesis run to
// find it. Include this once per module, never twice in the same file.
//
// Each includer uses only the subset it needs, so the rest are legitimately unused
// here. (Verilator pragmas are plain comments to Quartus.)
/* verilator lint_off UNUSEDPARAM */

localparam logic [26:0] SYS2_BASE_MAINCPU = 27'h00_0000;
localparam logic [26:0] SYS2_BASE_AUDIO   = 27'h09_0000;
localparam logic [26:0] SYS2_BASE_TILES   = 27'h0a_0000;
localparam logic [26:0] SYS2_BASE_SPRITE  = 27'h12_0000;
localparam logic [26:0] SYS2_BASE_CHARS   = 27'h22_0000;
localparam logic [26:0] SYS2_BASE_EEPROM  = 27'h22_4000;
localparam logic [26:0] SYS2_END_STREAM   = 27'h22_4200; // one past the last byte

// The 6502 program ROM occupies the top 48 KiB of the audiocpu region; the low
// 16 KiB (0x090000-0x093fff) is the unpopulated 0x4000-0x7fff socket on
// ssprint/csprint and is 0xff there.
localparam logic [26:0] SYS2_AUDIO_ROM_OFF = 27'h00_4000;

// ---------------------------------------------------------------------------
// Target memory capacities.
//
// Each decode below is clamped to what its target memory can actually hold, so
// a stream region can never write past the end of its target. Where a game's
// image is smaller than a region, the remainder is 0xff fill in that game's
// stream, so nothing real is lost.
//
// Family state: chars hold the full 0x04000 region in BRAM; sprites hold the
// full 0x100000 family maximum in SDRAM; tiles hold 0x80000 in SDRAM on the
// shipping path (a 128 KiB on-chip tile BRAM would only fit Paperboy's
// 0x20000 image).
// ---------------------------------------------------------------------------

// Sprite bytes forwarded to SDRAM -- the full 1 MiB family maximum. 720 and APB carry
// 1 MiB of motion-object ROM and every eighth of it is >90 % non-zero -- real content,
// not a padded slot. The hazard this size interacts with is the address map below:
// growing it without moving the sprite base would have written 768 KiB straight through
// the tile image (the Paperboy-era base sat immediately below another region);
// base and size move in the same edit.
localparam logic [26:0] SYS2_SPRITE_SDRAM_BYTES = 27'h10_0000;

// ---------------------------------------------------------------------------
// The SDRAM address map -- one source of truth.
//
// Until now only the TILES base was a named constant. The sprite base was the bare
// literal `81920` in FOUR places: the loader-writer's default parameter, its instantiation,
// and both sprite-reader instantiations. Nothing tied the WRITE view to the
// READ view -- change one and the sprite ROM is written to one address and read from
// another, with no gate able to see it. That is the same duplicate-mapping shape as the
// stale window map just deleted from `sys2_sdram_loader_writer`, and growing the regions walks
// straight into it: growing sprites to the family's 1 MiB means editing that literal
// everywhere and missing one.
//
// So the whole map lives here, as sizes plus derived bases. Add a region by adding its
// size and chaining its base; never write a base as a literal anywhere else.
//
// Tiles keeps its proven base. 262144 is what is running on silicon today, and the tile
// path is the one that cost weeks to get right -- it is not moved for tidiness. The map is
// laid out AROUND it: sprites move up above tiles (they must, to reach 1 MiB), and the
// 0..262143 words the old Paperboy-era maincpu+sprite+audio allocation left behind become
// free space. Region ORDER here is chosen to keep that constant fixed, not for elegance.
//
// Sizes are the family maxima, so the map does not have to change again per game:
//   tiles   0x40000 words (512 KiB) -- apb is the largest
//   sprites 0x80000 words (1 MiB)   -- 720 and apb
//   maincpu 0x24000 words (576 KiB) -- apb; stored FLAT, no window compaction
// Total 0xE4000 words = 1.78 MiB of a 32 MiB device, so there is no reason to be clever.
localparam int SYS2_TILES_SDRAM_WORD_BASE   = 262144;              // 0x40000, proven on silicon
localparam int SYS2_TILES_SDRAM_WORDS       = 262144;              // 0x40000 (512 KiB)
localparam int SYS2_SPRITE_SDRAM_WORD_BASE  = SYS2_TILES_SDRAM_WORD_BASE + SYS2_TILES_SDRAM_WORDS;
localparam int SYS2_SPRITE_SDRAM_WORDS      = 524288;              // 0x80000 (1 MiB), family max
localparam int SYS2_MAINCPU_SDRAM_WORD_BASE = SYS2_SPRITE_SDRAM_WORD_BASE + SYS2_SPRITE_SDRAM_WORDS;
// 0x48000 WORDS, not 0x24000. 576 KiB is 0x90000 bytes = 0x48000 words; writing the
// byte-count's hex digits as a word count halves the region. Simulation caught exactly that
// on this constant's first run, which is the entire reason the check exists.
localparam int SYS2_MAINCPU_SDRAM_WORDS     = 294912;              // 0x48000 (576 KiB), flat
localparam int SYS2_SDRAM_WORDS_USED        = SYS2_MAINCPU_SDRAM_WORD_BASE + SYS2_MAINCPU_SDRAM_WORDS;

// Sprites have moved. The Paperboy-era base 81920 is gone: at 1 MiB the
// region no longer fits below the tile image, so the growth and the move were necessarily one
// edit -- the two always move together. LIVE now aliases the planned base,
// and is kept as a name only so the writer and both readers still resolve one symbol.
localparam int SYS2_SPRITE_SDRAM_WORD_BASE_LIVE = SYS2_SPRITE_SDRAM_WORD_BASE;

// Alpha character BRAM is now the family-sized 16 KiB (14-bit address), so this is
// no longer a clamp -- it matches the region exactly.
localparam logic [26:0] SYS2_CHARS_BRAM_BYTES = 27'h00_4000;

// ---------------------------------------------------------------------------
// Game descriptor -- MRA index 1, v2
//
//   [0] slapstic type  105/107/108/109/110 decimal, one per game
//   [1] game id        SYS2_GAME_* below
//   [2] flags          bit0 = TMS5220 populated; bits 7:1 reserved 0
//   [3] reserved       must be 0
//
// v1 was a single slapstic byte. A v1 MRA is REJECTED rather than tolerated: with
// only byte 0 present the other three read as zero, which is a plausible-looking
// descriptor (game 0 = Paperboy, no speech). Requiring all four bytes makes an
// out-of-date MRA fail visibly rather than boot subtly wrong.
// ---------------------------------------------------------------------------
localparam int SYS2_DESC_BYTES = 4;

localparam logic [7:0] SYS2_GAME_PAPERBOY = 8'd0;
localparam logic [7:0] SYS2_GAME_720      = 8'd1;
localparam logic [7:0] SYS2_GAME_SSPRINT  = 8'd2;
localparam logic [7:0] SYS2_GAME_CSPRINT  = 8'd3;
localparam logic [7:0] SYS2_GAME_APB      = 8'd4;
localparam logic [7:0] SYS2_GAME_MAX      = SYS2_GAME_APB;

// Flags byte.
localparam int SYS2_FLAG_TMS_PRESENT = 0;

// Expected slapstic type per game id. The pairing is checked so a descriptor that
// is individually well-formed but internally inconsistent (right slapstic, wrong
// game, or vice versa) still fails loudly -- on System 2 the slapstic bank IS the
// VRAM view select, so a wrong table scrambles VRAM writes rather than merely
// mis-banking ROM.
localparam logic [7:0] SYS2_SLAPSTIC_PAPERBOY = 8'd105;
localparam logic [7:0] SYS2_SLAPSTIC_720      = 8'd107;
localparam logic [7:0] SYS2_SLAPSTIC_SSPRINT  = 8'd108;
localparam logic [7:0] SYS2_SLAPSTIC_CSPRINT  = 8'd109;
localparam logic [7:0] SYS2_SLAPSTIC_APB      = 8'd110;

/* verilator lint_on UNUSEDPARAM */
