// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Byte-writable dual-port RAM that infers clean M10K blocks. Port A is the CPU
// (per-byte write + registered read at a_addr); port B is a registered read-only
// port (the video fetch at b_addr). Both ports have one-cycle read latency, and
// the video read uses its own clock so the CPU and raster domains stay explicit.
//
// The CPU and video ports read independent addresses, which -- together with the
// write -- would be three accesses to a single array. M10K has only two ports,
// and relying on true-dual-port inference is fragile here: Quartus 17.0
// mis-detected the second read as asynchronous logic, left the array uninferred,
// and crashed the decoder optimiser (opt_op_decsel / replace_decoder_nlut). So
// instead each read port gets its own copy of the array, both written in
// lockstep. The CPU copy is single-port (write + read at a_addr); the video copy
// is simple-dual-port (write a_addr, read b_addr). Each byte lane is its own
// 8-bit array (written wholly per lane) to avoid byte-enable fragmentation.
module sys2_dpram #(
	parameter int AW = 11,          // address width; depth = 2**AW
	// Port C: a THIRD independent read-only copy. Off by default -- only the two playfield
	// map RAMs need it, and each enabled instance costs another 2 lanes of M10K.
	//
	// It exists because the playfield tile PREFETCHER needs the map at the same time the
	// RENDERER does, and the two read different rows (the prefetcher runs two lines ahead).
	// An earlier design avoided a third port by having the prefetcher STEAL the renderer's on the
	// slots the renderer was assumed not to need ("the render video port is idle 7 cycles in
	// 8"). That assumption is FALSE: sys2_video_render drives its map address every clock
	// and re-latches pf_group2 (palette group) and pf_category2 (priority) from the returned
	// word every clock, so a stolen slot overwrites good values with the prefetcher's. Measured
	// 25,344 stolen slots per frame, the renderer read back someone else's word on
	// ALL of them -- wrong colour and wrong priority across the playfield.
	parameter bit PORT_C = 1'b0
) (
	input  logic           clk,
	input  logic           video_clk,

	// Port A: CPU read/write.
	input  logic [AW-1:0]  a_addr,
	input  logic [15:0]    a_wdata,
	input  logic [1:0]     a_we,    // per-byte write enable ({high, low})
	output logic [15:0]    a_rdata,

	// Port B: read only (video fetch).
	input  logic [AW-1:0]  b_addr,
	output logic [15:0]    b_rdata,

	// Port C: read only, independent address (playfield tile prefetch). Reads 0 when
	// PORT_C is 0 and costs nothing -- the copy is not instantiated.
	input  logic [AW-1:0]  c_addr,
	output logic [15:0]    c_rdata
);

localparam int DEPTH = 1 << AW;

// ramstyle pins each lane to an M10K block. Whenever a design's total RAM demand
// exceeds the device's 553 M10K, Quartus 17.0 A&S caps inferred block RAM at device
// capacity and spills the excess to flip-flops, picking arbitrary victims per run.
// Pinning keeps these latency-critical CPU/video dual-port fetches out of any spill
// set (with the big ROMs in SDRAM, total demand now sits below 553, so this is
// insurance rather than the fix).
// no_rw_check drops read-during-write bypass MUXes; reads are already old-data and
// the design never relies on read-during-write forwarding.
//
// ...Except that the read copies must agree with each other, which is not the same
// REQUIREMENT. `no_rw_check` does not mean "old data", it means UNCONSTRAINED: on a
// same-address read-during-write Quartus is free to return anything, per physical block.
// vid_* and pfx_* are SEPARATE M10Ks holding the same contents, so an unconstrained RDW
// lets them return DIFFERENT words for the same address in the same cycle. Simulation
// models both as plain old-data reads and they agree by construction, so no bench can
// ever see it -- but the CPU rewrites the playfield map continuously, so the coincidence
// happens many times a frame on silicon. That is a wrong tile code reaching the SDRAM
// prefetcher: right colours, right structure, wrong tiles.
//
// Hardware confirmed it: the two replicated read copies did not agree.
// The fitter had these as `Port B RDW Mode = New data` while the RTL means old-data.
// Dropping no_rw_check on the two REPLICATED read copies costs bypass MUXes in ALMs (43%
// used, ample) and no extra M10K, and makes both copies deterministic and identical.
// cpu_* keeps it: nothing compares against that copy, and its RDW is genuinely don't-care.
(* ramstyle = "no_rw_check, M10K" *) logic [7:0] cpu_lo [0:DEPTH-1];   // CPU read copy
(* ramstyle = "no_rw_check, M10K" *) logic [7:0] cpu_hi [0:DEPTH-1];
(* ramstyle = "M10K" *) logic [7:0] vid_lo [0:DEPTH-1];   // video read copy (same contents)
(* ramstyle = "M10K" *) logic [7:0] vid_hi [0:DEPTH-1];

// CPU port: single-port per lane (write + read at a_addr).
always_ff @(posedge clk) begin
	if (a_we[0]) cpu_lo[a_addr] <= a_wdata[7:0];
	if (a_we[1]) cpu_hi[a_addr] <= a_wdata[15:8];
	a_rdata[7:0]  <= cpu_lo[a_addr];
	a_rdata[15:8] <= cpu_hi[a_addr];
end

// Video copy: dual-clock simple-dual-port per lane. CPU writes update it in the
// CPU domain; the independent registered read is wholly in the raster domain.
always_ff @(posedge clk) begin
	if (a_we[0]) vid_lo[a_addr] <= a_wdata[7:0];
	if (a_we[1]) vid_hi[a_addr] <= a_wdata[15:8];
end

always_ff @(posedge video_clk) begin
	b_rdata[7:0]  <= vid_lo[b_addr];
	b_rdata[15:8] <= vid_hi[b_addr];
end

// Prefetch copy: same construction as the video copy above, written in lockstep from the CPU
// port and read at an independent address. Generated, so a PORT_C=0 instance is bit-for-bit
// what it was and infers no extra blocks.
generate if (PORT_C) begin : g_port_c
	// No no_rw_check here -- see the RDW note above the vid_* declarations. This copy exists
	// ONLY to be read at an independent address while holding the same contents as vid_*, so
	// an unconstrained read-during-write is precisely the thing that breaks it.
	(* ramstyle = "M10K" *) logic [7:0] pfx_lo [0:DEPTH-1];
	(* ramstyle = "M10K" *) logic [7:0] pfx_hi [0:DEPTH-1];

	always_ff @(posedge clk) begin
		if (a_we[0]) pfx_lo[a_addr] <= a_wdata[7:0];
		if (a_we[1]) pfx_hi[a_addr] <= a_wdata[15:8];
	end

	always_ff @(posedge video_clk) begin
		c_rdata[7:0]  <= pfx_lo[c_addr];
		c_rdata[15:8] <= pfx_hi[c_addr];
	end
end else begin : g_no_port_c
	/* verilator lint_off UNUSEDSIGNAL */
	wire [AW-1:0] c_addr_unused = c_addr;
	/* verilator lint_on UNUSEDSIGNAL */
	assign c_rdata = 16'h0000;
end
endgenerate

endmodule
