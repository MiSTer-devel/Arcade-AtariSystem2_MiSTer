derive_pll_clocks
derive_clock_uncertainty

# core specific constraints

# ---------------------------------------------------------------------------
# ASPECT-RATIO HANDOFF TO THE HDMI DOMAIN -- the design's worst setup path.
#
# `emu`'s VIDEO_ARX / VIDEO_ARY are combinational functions of `status` (and, since the
# Orientation option, of `game_id` and `direct_video`), so they live in clk_sys. sys_top captures
# them in registers clocked by **clk_vid**, the HDMI PLL output
# (sys/sys_top.v:899-931) -- a clk_sys -> pll_hdmi crossing on signals that change only when a
# user moves an OSD option and are then stable for millions of cycles.
#
# That crossing has always been the design's worst setup path. It is the
# `pll_hdmi|...|counter[0].output_counter|divclk` row in the Setup Summary, and it is the path
# the .qsf's SEED note is about: it has sat within a fraction of a nanosecond of zero across
# seeds and design revisions, while every other domain in the design has >= 3.9 ns.
#
# THE FRAMEWORK ALREADY CUTS EVERY SIBLING OF THESE REGISTERS. In that same
# `always @(posedge clk_vid)` block, sys/sys_top.sdc:36-37,54-55 false-path `wcalc`/`hcalc`,
# `hdmi_width`/`hdmi_height`, and the whole `WIDTH`/`HFP`/`HS`/`HBP`/`HEIGHT`/`VFP`/`VS`/`VBP`
# group. `arx`/`ary`/`arxy` are simply the members it left timed. Cutting them is consistency
# with the framework's own treatment of this block, not a novel exemption -- which is why the
# constraint is safe to state as a false path rather than a multicycle.
#
# Consequence if a bus captures a mixed value while the user is moving an OSD option: the
# clk_vid state machine recomputes hmin/hmax/vmin/vmax from a garbled arx for a frame, then
# recomputes correctly on the next edge, because the always block re-reads arx/ary every cycle.
# One transient frame of wrong geometry during an OSD change, self-correcting. That is exactly
# the trade the framework already accepted for wcalc/hcalc.
#
# MAX_FANOUT WAS CONSIDERED AND IS THE WRONG AXIS -- worth recording, because the tooling's
# vocabulary offers it first. It caps loads per driver and replicates the driver; it cannot
# shorten a combinational chain, and it is not a constraint at all -- it perturbs placement, so
# its result is another draw from the same seed distribution the .qsf already documents. Three
# things said it was not the mechanism: End Point TNS is 0.000 with a SINGLE marginal endpoint
# (fanout starvation makes many endpoints slightly late), the .qsf already runs
# PHYSICAL_SYNTHESIS_REGISTER_DUPLICATION / ROUTER_LCELL_INSERTION_AND_LOGIC_DUPLICATION at
# HIGH PERFORMANCE EFFORT so a fanout-limited path gets duplicated already, and the suspect is
# depth on a quasi-static CDC, which replication does not touch.
#
# If a bound is ever preferred to a cut, `set_max_delay 20 -to {arx[*] ary[*] arxy}` is the
# conservative variant -- 20 ns is far beyond any real settling need on a signal that moves once
# per OSD keypress, and it keeps a sanity check that the path is not wildly unrouted.
#
# RE-CHECK AFTER ANY CHANGE HERE: the worst setup slack should now be the NEXT pll_hdmi path,
# not this one. Have the Timing Analyzer report the actual worst-path ENDPOINT rather than
# inferring from the slack number alone -- for a long time this path was identified only by
# its clock name.
set_false_path -to {arx[*] ary[*] arxy}

# CLOCK COLLAPSE: clk_t11 is now the SAME net as clk_sys (32 MHz), so the design is
# a SINGLE clock domain. Every former clk_sys<->clk_t11 crossing (the vblank/scanline pulse_cdc,
# rom_loaded_sync, service_t11_sync, the maincpu ROM-read req/ack + data, the scroll registers)
# is now an ordinary same-clock reg-to-reg path and must simply meet the 32 MHz period -- so the
# old CDC false_paths / set_max_delay were DELETED (left in place they would wrongly cut real
# same-clock paths). The synchroniser flops still exist (harmless added latency) but need no
# special timing. This removes the metastable-CDC bug class that black-screened the board
# (notably the vblank IRQ never serviced and the O[8] maincpu-read corruption).

# ---------------------------------------------------------------------------
# SDRAM external I/O timing (MT48LC16M16 class, CL2, 32 MHz controller).
# SDRAM_CLK is PLL outclk_2 (32 MHz) wired straight to the pin -- a clean, phase-controlled
# clock instead of the old fabric `~clk`, so these delays can actually close. Values adapted
# from the proven reference core (Arcade-Atari-system1 rtl/lib/mem/sdram.sdc).
# ITS PHASE IS LOAD-BEARING AND IS 15.625 ns (-180 deg). It was once moved to 26.875 ns to
# close the early side of the capture slot below, and that build CORRUPTED SDRAM ON HARDWARE
# -- it was reverted, and the slot was closed from the CONTROLLER's end
# instead (negedge DQ capture). Do not retune this phase to fix read timing: a later
# SDRAM_CLK buys DQ hold by spending the chip's command/address hold one-for-one, against
# set_output_delay values that model NO board skew. See the capture-slot note below.
#
# NOTE (verify on first compile): the -source PLL counter node below is the
# expected name for outclk_2 of emu|pll (altera_pll_i, general[2]). If Quartus
# reports it cannot be found, open Timing Analyzer -> Report Clocks and substitute
# the actual `...|general[2].gpll~PLL_OUTPUT_COUNTER|divclk` path. If the constraint
# does not apply, the SDRAM I/O paths fall back to unconstrained (today's behaviour)
# -- it degrades gracefully, it will not break the build.
create_generated_clock -name SDRAM_CLK \
  -source [get_pins -compatibility_mode {*|pll|pll_inst|altera_pll_i|general[2].gpll~PLL_OUTPUT_COUNTER|divclk}] \
  [get_ports {SDRAM_CLK}]

# Read capture: data access time tAC = 6.0 ns (max), output hold tOH = 2.5 ns (min).
set_input_delay  -clock SDRAM_CLK -max 6.0 [get_ports {SDRAM_DQ[*]}]
set_input_delay  -clock SDRAM_CLK -min 2.5 [get_ports {SDRAM_DQ[*]}]

# Command/address/data launch: input setup tIS = 1.5 ns (max), hold tIH = 0.8 ns (min).
set_output_delay -clock SDRAM_CLK -max  1.5 [get_ports {SDRAM_A[*] SDRAM_BA[*] SDRAM_DQ[*] SDRAM_DQML SDRAM_DQMH SDRAM_nCS SDRAM_nRAS SDRAM_nCAS SDRAM_nWE SDRAM_CKE}]
set_output_delay -clock SDRAM_CLK -min -0.8 [get_ports {SDRAM_A[*] SDRAM_BA[*] SDRAM_DQ[*] SDRAM_DQML SDRAM_DQMH SDRAM_nCS SDRAM_nRAS SDRAM_nCAS SDRAM_nWE SDRAM_CKE}]

# ---------------------------------------------------------------------------
# THE DQ CAPTURE SLOT -- both sides are constrained here, by DEFAULT analysis. Do not add a
# multicycle back (history at the end of this block explains why one used to be here).
#
# The controller launches on clk_sys (outclk_0) and captures SDRAM_DQ on its **NEGEDGE**;
# SDRAM_CLK rises at 15.625 ns of the 31.250 ns period (-180 deg).
# This constrains exactly ONE path group: SDRAM_DQ[*] -> sys2_sdram|dq_in_r[*], the
# unconditional every-cycle DQ capture register (all 16 bits pack into the IOE -- confirmed
# "Packed Register / Fast Input Register assignment" in fit.rpt; re-check it after any change
# here, since a negedge register that fails to pack silently moves into the fabric).
#
# THIS IS A SOURCE-SYNCHRONOUS CAPTURE, NOT A SLOW LOGIC PATH. The word must land in ONE
# specific fabric slot. In a burst the NEXT word has already overwritten the bus, so a one-cycle
# slip returns V(a)=img(a|1), every read returning its neighbouring word's data (see sys2_sdram.sv's
# dq_in_r note). BOTH sides of the slot must therefore be real checks:
#   setup = the word must be captured by the intended edge
#   hold  = the word must MISS the edge one period earlier
#
# WITH THE NEGEDGE CAPTURE THOSE ARE THE DEFAULT EDGE RELATIONSHIPS, SO NO MULTICYCLE IS
# NEEDED AND NONE MAY BE ADDED. Launch SDRAM_CLK rise @15.625 -> latch clk_sys FALL @46.875 is
# one full period (setup), and the hold check falls on @15.625, the edge the data must miss.
# That is the constraint the design actually wants, expressed with nothing but the clock
# definitions -- and it is why the early side is now SHIPPED rather than tracked in a gate.
# If dq_in_r is ever moved back to the posedge, these defaults become wrong (the setup
# relationship collapses to a half period) and BOTH lines of defence disappear at once.
#
# WHY THE CAPTURE MOVED INSTEAD OF THE PHASE -- the short version; full derivation in
# sys2_sdram.sv at dq_in_r. Measured on the fitted netlist, the data reaches the capture flop
# at 35.003..42.192 ns while the clk_sys posedge at 31.250 reaches it at 39.217..40.679: the
# unwanted edge sat INSIDE the arrival window, so which slot won was decided by PVT, not by
# design. Root cause is the clock round trip, not logic -- SDRAM_CLK needs ~13.6 ns to reach the
# pin through its output buffer while the capture clock reaches the flop in ~7.8 ns, so the
# nominal 15.625 ns of phase separation was eroded to ~9.8 ns before tAC was even added. The
# fitter could not help: given ~28 ns of unused setup room it returned bit-identical numbers.
# Moving the LATCH 15.625 ns earlier spends that unused setup room on hold margin and touches
# nothing else. Measured per corner (setup -> / hold ->):
#   slow 100c 28.093 -> 12.468 | -5.707 -> +9.918      fast 100c 33.363 -> 17.738 | -8.889 -> +6.736
#   slow -40c 28.882 -> 13.257 | -6.198 -> +9.427      fast -40c 34.365 -> 18.740 | -9.851 -> +5.774
# The long-quoted -5.676 ns deficit was the SLOW CORNER ONLY; the true worst was -9.851 at
# fast/-40c. slow/100c is the right corner for SETUP and the OPTIMISTIC one for HOLD.
#
# THE REMEDY WAS BELIEVED TO BE THE PHASE. IT IS NOT, AND THAT WAS PROVEN ON HARDWARE.
# Moving SDRAM_CLK to 26.875 ns also closed every STA corner -- and CORRUPTED SDRAM on the board
# (Paperboy's sprites came back shredded; it survived a power cycle). Instrumented on the
# board rather than guessed: the HPS delivered our exact bytes and every one of them reached
# the writer, but the image read back wrong and two reads of the same address DISAGREED.
# Cause is the half of the trade a later phase SPENDS: the SDRAM's command/address hold falls
# 12.461 -> 1.846 ns, and the `set_output_delay` above carries the chip's tIS/tIH and **nothing
# for the board skew between the SDRAM_CLK trace and the address lines**. At 12.5 ns that
# omission is irrelevant; at 1.85 ns it is not, so addresses latch marginally and reads land on
# the wrong row. The capture-edge fix is immune to that entire failure mode by construction:
# it does not appear in any clk_sys -> SDRAM_* output path, so out_setup/out_hold are unchanged
# to the picosecond, which is the property that matters most here.
#
# HISTORY -- WHY A MULTICYCLE USED TO BE HERE, AND WHY REMOVING IT IS NOT A RELAXATION.
# With a POSEDGE capture the default setup relationship was a half period (launch 15.625 ->
# latch 31.250 = setup -2.975 ns), which does not close, so `-setup -end 2` pushed the latch to
# 62.500. That was load-bearing, NOT a cosmetic workaround -- and an earlier comment calling the
# June -9.665 ns failure "a false analysis artifact" was RETRACTED on measurement.
# Its `-hold` companion was `-end 1`: the textbook pairing for `-setup -end N` (N-1), correct
# for a slow LOGIC path where the point is "don't hold-check an edge the data cannot reach".
# For a source-synchronous capture it is exactly backwards -- it put the hold check on latch
# edge 0.000, a full cycle before the edge that matters, where it passed vacuously at +25.574 ns.
# GENERALISE: pair `-setup -end N` with `-hold -end N-1` for LOGIC, but for a CAPTURE pair it
# with the edge the data must MISS. Better still, arrange the clocking so the DEFAULT pairing is
# the one the design wants -- which is what the negedge capture achieves, and it is why there is
# now no multicycle to get wrong.
# (For the record, should one ever be needed again: `-rise_to [get_pins ...]` is REJECTED by
# Quartus -- multicycle endpoints must be CLOCK collections -- and get_clocks does Tcl string
# matching where [..] is a char class, so clk_sys must be matched as `general?0?` with the
# *|pll|pll_inst| prefix to disambiguate it from the audio PLL's general[0].)
#
# Re-check setup AND hold at every corner after ANY change to these constraints, the
# SDRAM_CLK phase, `dq_in_r`, or the DQ pin assignments.
# ---------------------------------------------------------------------------

