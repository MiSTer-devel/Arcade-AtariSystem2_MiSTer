`timescale 1ns/10ps
module  pll_0002(

	// interface 'refclk'
	input wire refclk,

	// interface 'reset'
	input wire rst,

	// interface 'outclk0'
	output wire outclk_0,

	// interface 'outclk1'
	output wire outclk_1,

	// interface 'outclk2'  -- SDRAM clock (32 MHz, rise at 15.625 ns of clk_sys's 31.250 ns
	// period = -180 deg). That phase is a load-bearing hand edit and must not be retuned;
	// see phase_shift2 below for what happened the one time it was.
	output wire outclk_2,

	// interface 'locked'
	output wire locked
);

	altera_pll #(
		.fractional_vco_multiplier("false"),
		.reference_clock_frequency("50.0 MHz"),
		.operation_mode("direct"),
		.number_of_clocks(3),
		.output_clock_frequency0("32.000000 MHz"),
		.phase_shift0("0 ps"),
		.duty_cycle0(50),
		.output_clock_frequency1("20.000000 MHz"),
		.phase_shift1("0 ps"),
		.duty_cycle1(50),
		.output_clock_frequency2("32.000000 MHz"),
		// SDRAM_CLK PHASE. This is a HAND EDIT to a wizard-generated file, and it is
		// load-bearing: it is what holds the SDRAM read word in the correct fabric capture
		// slot. Re-opening this IP in the MegaWizard WILL clobber it (rtl/pll.v's "Retrieval
		// info" block is already stale and still says 0 ps). Check this value first after any
		// regeneration.
		//
		// WHAT SHIPS IS -15625 ps (= -180 deg, rise at 15.625 ns), AND RETUNING IT IS THE
		// ONE CHANGE HERE THAT HARDWARE HAS ALREADY REJECTED. The read-capture problem this
		// phase used to be blamed for is real, but it was fixed somewhere else -- see below.
		//
		// The problem, measured on the fitted netlist: the clk_sys edge at 31.250 -- the edge
		// the word must MISS -- sat INSIDE the DQ arrival window (hold -5.680 ns on all 16 bits
		// at slow 1100mv 100c; -9.851 ns at the true worst corner). Which slot won was decided
		// by silicon, not by design, and the wrong slot is exactly V(a)=img(a|1) -- every read
		// returning its neighbouring word's data. Root cause is the clock round trip, not logic: SDRAM_CLK
		// needs ~13.6 ns to reach the pin through its output buffer while the capture clock
		// reaches the flop in ~7.8 ns, eroding the nominal 15.625 ns of separation to ~9.8 ns
		// before tAC is added.
		//
		// THE REMEDY WAS BELIEVED TO BE THIS PHASE. IT IS NOT, AND THAT WAS SETTLED ON THE
		// BOARD, NOT IN STA. -4375 ps puts the rise at 26.875 ns, 11.250 ns LATER, and closes
		// the early side at every corner in STA -- and a build carrying it CORRUPTED SDRAM
		// ON HARDWARE. It was reverted, and the slot was then closed from the
		// CONTROLLER's end instead, by capturing DQ on the NEGEDGE of clk_sys (sys2_sdram.sv at
		// dq_in_r): worst corner -9.851 -> +5.872 ns, both sides on default edge relationships,
		// and the SDRAM command/address side untouched to the picosecond.
		//
		// WHY A LATER PHASE LOSES EVEN THOUGH STA LIKES IT. It BUYS DQ hold margin at the
		// 31.250 edge and SPENDS command/address hold margin at the SDRAM, one for one:
		//
		//     DQ  early hold @31.250   moves +D   must go POSITIVE
		//     SDRAM output hold        moves -D   must stay positive <- the ceiling
		//     DQ  late setup @62.500   moves -D   not binding (~19 ns)
		//     SDRAM output setup       moves +D   only improves
		//
		// THE TWO SIDES SUM TO A CONSTANT, so there is no phase that makes both comfortable
		// and the split is the whole design decision. Measured ACROSS ALL FOUR OPERATING
		// CORNERS (not just the slow one -- see below), the worst-corner sum is 3.255 ns, so
		// ~1.6 ns per side is the best ANY phase achieves on this interface. And
		// set_output_delay models NO BOARD SKEW, so the command/address hold STA calls positive
		// is spent against a margin the constraints cannot see. That is the half the -4375 ps
		// build spent, and the board is where the bill arrived.
		//
		// SCORING ONE CORNER WOULD HAVE SHIPPED A STILL-BROKEN SLOT. At -6250 ps (9.375 ns)
		// the slow corner reads a healthy +3.695 -- and MIN_fast_1100mv_-40c reads -0.466. The
		// data path runs SDRAM_CLK out through a pad and back in while the capture clock crosses
		// only the internal network, so at the fast corner the data arrives earlier by MORE than
		// the capture edge does: the early side is ~4.2 ns worse there. An early analysis here
		// scored only the slow 1100mV 100C corner, which for a HOLD check is the optimistic
		// one. Anyone re-tuning this must sweep all four corners -- a
		// single-corner number here is the same one-sided reasoning that left the slot half
		// constrained in the first place.
		//
		// This phase does NOT set which fabric cycle captures the word -- the 62.500 slot is
		// kept -- so sys2_sdram's RD_LAT and the dq_in_r indirection are independent of it.
		// Full derivation: atarisys2.sdc (the DQ capture slot block) and rtl/mem/sys2_sdram.sv.
		.phase_shift2("-15625 ps"),
		.duty_cycle2(50),
		.output_clock_frequency3("0 MHz"),
		.phase_shift3("0 ps"),
		.duty_cycle3(50),
		.output_clock_frequency4("0 MHz"),
		.phase_shift4("0 ps"),
		.duty_cycle4(50),
		.output_clock_frequency5("0 MHz"),
		.phase_shift5("0 ps"),
		.duty_cycle5(50),
		.output_clock_frequency6("0 MHz"),
		.phase_shift6("0 ps"),
		.duty_cycle6(50),
		.output_clock_frequency7("0 MHz"),
		.phase_shift7("0 ps"),
		.duty_cycle7(50),
		.output_clock_frequency8("0 MHz"),
		.phase_shift8("0 ps"),
		.duty_cycle8(50),
		.output_clock_frequency9("0 MHz"),
		.phase_shift9("0 ps"),
		.duty_cycle9(50),
		.output_clock_frequency10("0 MHz"),
		.phase_shift10("0 ps"),
		.duty_cycle10(50),
		.output_clock_frequency11("0 MHz"),
		.phase_shift11("0 ps"),
		.duty_cycle11(50),
		.output_clock_frequency12("0 MHz"),
		.phase_shift12("0 ps"),
		.duty_cycle12(50),
		.output_clock_frequency13("0 MHz"),
		.phase_shift13("0 ps"),
		.duty_cycle13(50),
		.output_clock_frequency14("0 MHz"),
		.phase_shift14("0 ps"),
		.duty_cycle14(50),
		.output_clock_frequency15("0 MHz"),
		.phase_shift15("0 ps"),
		.duty_cycle15(50),
		.output_clock_frequency16("0 MHz"),
		.phase_shift16("0 ps"),
		.duty_cycle16(50),
		.output_clock_frequency17("0 MHz"),
		.phase_shift17("0 ps"),
		.duty_cycle17(50),
		.pll_type("General"),
		.pll_subtype("General")
	) altera_pll_i (
		.rst	(rst),
		.outclk	({outclk_2, outclk_1, outclk_0}),
		.locked	(locked),
		.fboutclk	( ),
		.fbclk	(1'b0),
		.refclk	(refclk)
	);
endmodule

