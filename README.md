# Atari System 2 for MiSTer FPGA

FPGA hardware recreation of **Atari System 2** — the 1984 cartridge platform
built around a DEC T-11 main CPU, a 6502 sound board with a YM2151, two POKEYs
and a TMS5220C speech synthesiser, and an Atari Slapstic protection PAL.

One bitstream plays all five System 2 games; the MRA file you launch decides
which one runs.

<img src="https://img.shields.io/badge/Quartus-17.0.2-blue" alt="Quartus 17.0.2"> <img src="https://img.shields.io/badge/license-GPL--3.0--or--later-blue" alt="GPL-3.0-or-later">

---

## Supported games

| MRA | Set | Slapstic | Speech |
|-----|-----|----------|--------|
| `Paperboy (rev 3).mra` | `paperboy` | 105 | yes |
| `720 Degrees (rev 4).mra` | `720` | 107 | yes |
| `Super Sprint (rev 4).mra` | `ssprint` | 108 | no |
| `Championship Sprint (rev 3).mra` | `csprint` | 109 | no |
| `APB - All Points Bulletin (rev 7).mra` | `apb` | 110 | yes |

All 29 remaining MAME clone sets — other revisions and the German, French and
Spanish releases — ship under `releases/_alternatives/`.

`Paperboy (prototype)` is `MACHINE_NOT_WORKING` in MAME with every ROM a
`BAD_DUMP`.

---

## Installing

1. Copy `releases/atarisys2_<date>.rbf` to `/media/fat/_Arcade/cores/`.
2. Copy the `.mra` files you want from `releases/` to `/media/fat/_Arcade/`, and
   any alternates from `releases/_alternatives/<Game>/` to
   `/media/fat/_Arcade/_alternatives/<Game>/`.
3. Put the matching MAME ROM sets in `/media/fat/games/mame/`.

**Keep only one `atarisys2*` file in `_Arcade/cores/`.** An MRA's
`<rbf>atarisys2</rbf>` resolves by prefix, so an older dated release left in that
folder is an equally valid match and may load instead of the one you just copied.
Quote the filename you are running in any bug report.

**Upgrading from a core named `Arcade-Paperboy`?** Delete every
`Arcade-Paperboy*` file from `_Arcade/cores/`, and replace the old MRAs with the
ones in `releases/` — the old MRAs still match the old core, which is how you end
up running an outdated bitstream by accident. The OSD name changed too, so MiSTer
starts a fresh `.cfg` and OSD settings reset once; per-game NVRAM and high scores
are untouched.

**Using an alternate? Keep the parent ROM set too.** Each alternate MRA declares
`zip="<clone>.zip|<parent>.zip"` and takes from the parent archive whatever a
split set does not duplicate, so both zips have to be in `games/mame/`.

**No ROM data is in this repository.** The MRA files reference ROM parts by name,
CRC and SHA-1 only.

### Saving settings and high scores

Each MRA declares `<nvram index="2" size="512"/>`. The board's X2804A EEPROM
holds coin settings, game options, high scores *and the analog control
calibration*, so a blank EEPROM makes Paperboy's handlebars and 720's joystick
behave as if they were never calibrated. Leave the OSD `Autosave` option on;
MiSTer collects the save request when the OSD is opened, so settings land on disk
at menu-open time.

Paperboy and 720 ship with a corrected factory EEPROM image inlined in their
MRAs. The genuine Atari factory payload is *valid but miscalibrated* — its
checksum is correct, so the game accepts it, and the result is a permanent
hard-left steering bias.

---

## Controls

The five games share one button list, so every J1 slot has to cover their union.
A game that does not use a slot leaves it blank in its own MRA.

| Game | Button 1 | Button 2 | Button 3 | Button 4 | Button 5 | Button 6 |
|------|----------|----------|----------|----------|----------|----------|
| Paperboy | Throw Right | Throw Left | Coin | Service Credit | — | — |
| 720 Degrees | Jump | Kick | Coin | Service Credit | — | — |
| Super Sprint | Gas | — | Coin | Service Credit | Start | — |
| Championship Sprint | Gas | — | Coin | Service Credit | Start | — |
| APB | Siren | Gas | Coin | Service Credit | — | Fire |

Paperboy has no separate Start: either throw button starts the game, per TM-275.
Service Credit is the coin-door button that grants a credit without advancing the
coin counter.

### Analog controls

Every System 2 cabinet has analog inputs, and all of them work from a normal USB
pad:

- **Paperboy** steers on ADC channel 0 (handlebars) with speed on channel 1. Both
  the left analog stick and the D-pad drive it.
- **720 Degrees** reads a joystick plus a rotating shaft carrying two optical
  discs: a 72-tooth *rotate* disc and a 2-tooth *centre* disc that tells the game
  where "top" is. A player has one spinner, not a shaft, so the centre channel is
  derived from the same motion at the geometric 1:36 ratio.
- **Super Sprint** (three players) and **Championship Sprint** (two) read a
  steering wheel per player through Atari's LETA quadrature encoder, plus a
  pedal. The wheels come from the players' spinners or analog sticks.
- **APB** reads a steering wheel and a pedal.

The **D-pad steers the four wheel games too**, not just the analog stick — it
slams the axis to full scale, which is why the two options below matter most
there.

The wheel games' encoders report *movement*, not position, so a springing
thumbstick cannot map onto them one-to-one. Deflection is converted to a turn
rate instead: push further, turn faster, hold to keep turning. **Steering
sensitivity** sets that rate and **Steering deadzone** sets how much slop near
centre is ignored — useful on a worn stick that no longer returns to zero. Both
appear only on the four wheel games; Paperboy's handlebar is a position axis, so
neither applies.

| Steering sensitivity | Counts/sec at full deflection | 720 revolutions/sec |
|--------|--------|--------|
| Normal (default) | ~548 | 3.8 |
| High | ~1097 | 7.6 |
| Low | ~274 | 1.9 |
| Lowest | ~137 | 0.95 |

One setting covers every game in the core — MiSTer keeps one configuration file
per core rather than per ROM set.

Super Sprint's own self-test asks for three sets of controls and will report
faults for players 2 and 3 if only one controller is connected. That is the game
testing hardware that is not plugged in, not a core fault.

### OSD options

| Option | Effect |
|--------|--------|
| Aspect ratio | Original / Full Screen / custom. "Original" follows the orientation |
| Scale | Integer-scaling mode passed to `video_freak` |
| Orientation (APB only) | APB is the family's one vertical cabinet: `Vert` rotates it for a normal monitor, the two `No Rotate` states suit a physically rotated one |
| Steering sensitivity (wheel games) | Turn rate at full stick deflection — see the table above |
| Steering deadzone (wheel games) | Slop near centre to ignore: Normal / Small / Large / Largest |
| Game Options | The two physical DIP switches, presented per game with that game's own labels |
| CRT Alignment | Analog-output H-Position and H/V sync shift, ±32 |
| Service switch | Routes the Service input to the board's self-test line |

Every alignment control is a true bypass at zero, so a default OSD leaves the
analog output identical to the unmodified video path.

DIP switch labels come from each game's own operator manual rather than from
MAME, which uses generic stand-in labels where a game has no matching enum: 720's
"Bonus Life" is really *First Bonus Ticket At*, APB's "Max Continues" is
*Add-A-Coin Control*, and Championship Sprint's table is TM-292's rather than the
Lives/Bonus Life pair MAME inherits from Paperboy.

---

## Accuracy

- **Slapstic.** All five protection types (105, 107, 108, 109, 110) are
  implemented as one parameterised state machine. The MRA names the type and the
  core cross-checks it against the game id, so a mismatched or stale MRA fails
  visibly rather than corrupting a bank quietly.

- **Exact clocks.** `clk_sys` is 32 MHz and every chip enable is a *fractional*
  divider landing on the real crystal: T-11 execution at 10.000 MHz, the ADC at
  625 kHz, the YM2151 at 3.579545 MHz and the 6502 / POKEYs at 1.789772 MHz.
  Integer division of 32 MHz cannot hit those, and the audible difference in a
  YM2151 is not subtle.

- **Native raster.** 640 × 416 total, 512 × 384 visible, 16 MHz pixel clock,
  ~60.096 Hz, with the blanking, sync, 32V and VBLANK edges transcribed off
  SP-275 sheet 11A rather than inferred.

- **Analog audio chain.** The board filters **per chip, not on the sum** —
  SP-275 sheets 9A/9B give each of the YM, POKEY and TMS paths its own preamp
  pole, interstage DC block, programmable `0x187a` gain and output pole before
  the three are summed through their real 47K/47K/68K weights. The TMS path in
  particular runs through a ×4.84 preamp that a naive mix leaves 9 dB low.

- **Real chip models, audited.** POKEY is the FPGAArcade VHDL core with its
  polynomial generators corrected — poly5 was cross-coupled to poly4, and poly9
  and poly17 were replaced against the real chip's shift-register topology. The
  TMS5220 speech core's lattice registers were widened to match its ±16383
  datapath, which is what produced a level-select "pop".

- **Motion objects follow the silicon, not MAME.** 40 link slots, no repeat
  detection, and the low 3 tile-code bits wrap without carrying into the upper
  bits. MAME differs on all three, and following MAME instead breaks Championship
  Sprint and 720. 720's "MO Height Test" produces a ragged staircase 44 pixels
  taller than MAME's — that staircase is what the hardware draws.

- **Playfield coherence.** Tile bitmap and tile colour come from the *same* map
  word, so a CPU write between the prefetcher's read and the renderer's cannot
  draw a stale bitmap under a current colour — invisible on a static screen and
  obvious on a scrolling one.

- **Watchdog, EEPROM and service mode** are all modelled, so the games' own
  built-in self-tests run and pass rather than being bypassed.

The video path is pixel-exact against MAME frame captures on four of the five
games, and service-mode video RAM is byte-identical to MAME on every game that
keeps it on-chip. APB differs only where the game itself rewrites a motion-object
link twice per frame, which no single-snapshot comparison can represent. The core
was developed simulation-first, with AI assistance, under human review and
hardware validation on real MiSTer hardware.

Resource use of the shipped build (Cyclone V 5CSEBA6U23I7): 17,514 / 41,910 ALMs
(42 %), 465 / 553 RAM blocks (84 %), 84 / 112 DSP blocks, 3 / 6 PLLs, 24,108
registers. Timing closes with **TNS 0.000 on all 36 clock domains**: worst setup
+0.654 ns and worst hold +0.244 ns, both on PLL output counters.

### Known issues

- **Paperboy's D-pad and analog stick disagree on X.** Players using the analog
  stick are unaffected.
- **720's centre-disc encoding** emits the right number of counts at the right
  rate but distributes them evenly around the rotation, where the real optical
  disc may cluster them near the top. The game plays correctly either way.

---

## Building

Quartus Prime **17.0.2** (Lite or Standard), as required for MiSTer cores.

1. Open `atarisys2.qpf`.
2. Compile. The output is `output_files/atarisys2.rbf`.
3. For a release, copy it to `releases/atarisys2_YYYYMMDD.rbf`.

From a shell: `quartus_sh --flow compile atarisys2`.

Sources are listed in `files.qip` — add files there by hand, not through the
Quartus IDE, which writes them into the `.qsf` instead and lets the two lists
drift apart. If the `.qsf` ever grows a pile of pin and instance assignments,
Quartus has regurgitated what `sys/sys.tcl` already provides; delete everything
after `source files.qip` except the `PARTITION_HIERARCHY` line.

The tightest setup path in this design is not in the core, it is in the MiSTer
framework's HDMI pipeline at the 148.5 MHz 1080p60 pixel clock, and placement
decides whether it closes — which is why `atarisys2.qsf` pins a fitter seed.
Check the TNS column after compiling: Quartus reports "Full Compilation was
successful, 0 errors" and writes an RBF even when timing has not met.

---

## Repository layout

```text
atarisys2.qpf / .qsf / .sdc / .srf   Quartus project
atarisys2.sv                         Glue between the MiSTer framework and the core
files.qip                            Source file list
rtl/                                 The core
  main/  mem/  rom/  video/  sound/  clocks/  t11/  slapstic/  adc/
  lib/                               Vendored CPU and sound chip cores
  pll/                               PLL megafunction
sys/                                 MiSTer framework -- unmodified
releases/                            Dated RBF + MRA files
  _alternatives/<Game>/              Other revisions and regional releases
```

`sys/` is a verbatim copy of [Template_MiSTer](https://github.com/MiSTer-devel/Template_MiSTer)
at commit `df59d67`, byte-identical, with no core-specific edits. The `emu` port
list comes from the framework's own `sys/emu_ports.vh` rather than being copied
into `atarisys2.sv`, so a framework update cannot leave the two out of step.

---

## Licensing

The combined work is **GPL-3.0-or-later** (see `LICENSE`). The core's own RTL is
GPL-2.0-or-later, but it links components that require v3:

| Component | License |
|-----------|---------|
| `rtl/**/sys2_*.sv`, `rtl/t11/`, `rtl/slapstic/`, `rtl/adc/` (this core) | GPL-2.0-or-later |
| `atarisys2.sv` — MiSTer framework glue layer | GPL-2.0-or-later |
| `rtl/lib/T65/` — T65 6502 core, © Daniel Wallner and Mike Johnson | BSD-style, 3-clause |
| `rtl/lib/jt51/` — jt51 YM2151 core, © Jose Tejada (jotego) | GPL-3.0-or-later |
| `rtl/lib/POKEY.vhd` — © MikeJ, FPGAArcade | BSD-style, 3-clause |
| `rtl/lib/TMS5220.vhd` — © d18c7db | GPL-3.0-or-later |
| `rtl/video/sys2_analog_hpos.sv`, `sys2_analog_sync_shift.sv` — ported from Arcade-Raiden_MiSTer, © Umberto Parisi (rmonic79) | GPL-3.0-or-later |
| `sys/` — MiSTer framework, © Alexey Melnikov and contributors | GPL-2.0-or-later / GPL-3.0-or-later |

Every source file carries an `SPDX-License-Identifier` on its first line, so the table
above can be checked mechanically rather than trusted. Files ported from someone else's
work keep their original licence and attribution — they are not restamped.

---

## Credits

- **Atari Games** — the original hardware and the five games.
- **Daniel Wallner** and **Mike Johnson (MikeJ, FPGAArcade)** — the T65 6502 core.
- **Jose Tejada (jotego)** — the jt51 YM2151 core.
- **MikeJ / FPGAArcade** — the POKEY core.
- **d18c7db** — the TMS5220 speech core.
- **Umberto Parisi (rmonic79)** — the analog alignment controls, ported from
  [Arcade-Raiden_MiSTer](https://github.com/rmonic79/Arcade-Raiden_MiSTer).
- **Alexey Melnikov (Sorgelig)** and the MiSTer-devel community — the MiSTer
  framework and the arcade infrastructure this core plugs into.
- **The MAME team** — `atarisy2.cpp` and `atarisy2_v.cpp` were the functional
  cross-reference for everything the schematics do not settle. References in the
  RTL comments of the form `mame/atarisy2.cpp` point at MAME's own sources.
- **RetroShrimp** — this core.
