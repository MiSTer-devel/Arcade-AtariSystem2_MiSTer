// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// T-11 program ROM (BRAM games). BRAM storage with the read/ack handshake the
// main bus expects, written by a top-level decode of the maincpu byte stream.
//
// The MAME maincpu region is a sparse 0x90000-byte logical image
// populated only in 0x8000-byte windows selected by the
// 0x8000-granular index addr[19:15] -- Paperboy uses {1, 2, 6, 10, 14}, the
// family union is {1, 2, 3, 6, 10, 11, 14, 15}. This module compacts the
// mapped windows into WINDOWS x 32 KiB of BRAM and returns 0xffff for the
// all-0xff holes, so no hole storage is spent. The loader write and the CPU read
// apply the identical logical->physical map, so the placement stays
// observationally identical to the logical image (the change permitted by
// the download stream layout as long as the mapping is preserved).
//
// The integrator gates ld_we to the maincpu region and feeds the region byte
// offset on ld_addr.
// Dual-clock: the loader writes on wr_clk (the 32 MHz hps_io/clk_sys domain)
// while the T-11 reads on clk (clk_t11 -- the same 32 MHz net in this core).
// The two ports never access the array simultaneously because the CPU
// is held in reset until the download completes, so a simple dual-port BRAM with
// independent port clocks is sufficient.
module sys2_maincpu_rom (
	input  logic        clk,       // CPU read-port clock (clk_t11)
	input  logic        reset,     // reset for the read-port ack (clk domain)

	// Loader write port (one byte per strobe; even offset = low byte).
	input  logic        wr_clk,    // loader write-port clock (download domain)
	// Which game's window map to use (SYS2_GAME_*). The compaction below is per-game DATA,
	// not a Paperboy constant -- see the header note.
	input  logic [7:0]  game_id,

	input  logic        ld_we,
	input  logic [19:0] ld_addr,   // logical byte offset, 0x00000-0x8ffff
	input  logic [7:0]  ld_data,

	// CPU read port from sys2_main_bus (rom_addr is even). One-cycle latency;
	// rom_ack is held while rom_req stands once the word is presented.
	input  logic        rom_req,
	/* verilator lint_off UNUSEDSIGNAL */
	input  logic [19:0] rom_addr,
	/* verilator lint_on UNUSEDSIGNAL */
	output logic [15:0] rom_rdata,
	output logic        rom_ack
);

// SYS2_GAME_* for the per-game window map. One include per module.
`include "rtl/rom/sys2_rom_layout.vh"

// ---------------------------------------------------------------------------
// Window compaction -- per-game, not a Paperboy constant.
//
// The T-11's logical program space is 0x90000, but no game populates more than a fraction
// of it, so only the populated 0x8000-windows are stored and the rest reads 0xffff. Which
// windows those ARE differs per game, measured from the ROM manifests:
//
//   paperboy         1, 2, 6, 10, 14                 5 windows, 160 KiB
//   ssprint/csprint  1, 2, 3, 10, 11, 14, 15         7 windows, 224 KiB
//   720              1..13                          13 windows, 416 KiB
//   apb              1..9, 14..17                   13 windows, 416 KiB
//
// This module used to hardcode Paperboy's five. That made the standing decision ("the T-11 program stays in
// BRAM wherever it fits -- Paperboy, ssprint, csprint") wrong: a Sprint would have fetched
// 0xffff from three of its seven windows and died on the first bank switch, with nothing in
// simulation to say why.
//
// The map is the union, not per-game, and it must stay that way.
// The per-game version selected `win_pop`/`win_idx` from `game_id` on BOTH ports -- and the
// WRITE port runs during the index-0 download, when `game_id` is still 0.
//
//   MiSTer sends index 1 AFTER the index-0 stream completes, whatever order the MRA
// lists them in. That is hardware-observed: a trailing download re-fires `ioctl_download`
// after the main stream has already been placed.
//
// So every maincpu byte was placed with Paperboy's map and then read back with the
// running game's. Paperboy cannot see it (its map IS the default). Super Sprint had three of
// its seven windows dropped outright and the other four stored at the wrong physical indices
// -- the T-11 fetched garbage from every window. A black screen with no other symptom.
//
// The union of the three BRAM-resident games is {1,2,3,6,10,11,14,15} = 8 windows, 256 KiB.
// Both ports use it unconditionally, so descriptor timing cannot affect placement. `game_id`
// is still read, but ONLY on the read port and ONLY to report 720/APB as unpopulated -- by
// then the descriptor has long since landed.
//
// Do not "optimise" this back to a per-game map. Anything the load path derives from the
// descriptor is wrong by construction; the descriptor does not exist yet.
//
// 720 and APB need thirteen windows (~333 M10K) and are out of reach for BRAM -- they need
// the program in SDRAM. This module reports them as fully unpopulated so the
// failure is loud rather than a partial image, and so synthesis does not size for them.
// 8 windows is the UNION of paperboy + the Sprints. (Was 7 while the map was per-game; the
// union costs one more window and buys immunity to the descriptor-timing bug above.) M10K is
// the binding constraint and this fits only because the playfield tile image is in SDRAM: the
// same count with a resident 128 KiB tile BRAM measured 553/553, exactly 100 % -- legal,
// but with nothing left for any future change.
localparam int WINDOWS = 8;
localparam int WORDS = WINDOWS * 'h4000;

// Split into byte lanes, each written wholly (one lane per loader byte) and read
// back as {hi, lo}. A single 16-bit array written one byte at a time (lane
// selected by ld_addr[0]) does NOT infer as BRAM in Quartus 17.0 -- it becomes a
// 1.3 Mbit logic decoder that blows the optimiser up to ~15 GB and crashes it
// (opt_op_decsel / replace_decoder_nlut). Two 8-bit simple-dual-port arrays map
// straight to M10K.
logic [7:0] rom_lo [0:WORDS-1];
logic [7:0] rom_hi [0:WORDS-1];

// Is this 0x8000-granular logical index stored? Game-independent by design -- see the header:
// the write port runs before the descriptor exists, so anything derived from game_id here
// would place bytes using the wrong map.
//
//   family build : the UNION of paperboy {1,2,6,10,14} and the Sprints {1,2,3,10,11,14,15}
//   Paperboy-only: Paperboy's five, the only game that build supports
//
// ---------------------------------------------------------------------------
// Mirror keys (corrected twice -- read all of this).
//
// MAME's ASSEMBLED region replicates each banked ROM across the region (ROM_RELOAD):
// paperboy's units mirror 4-way (2=3=4=5, 6=7=8=9, 10=11=12=13, 14=15=16=17), the
// Sprints' pairwise (2=4, 3=5, 10=12, 11=13, 14=16, 15=17). The TRUE bankselect decode
// (bitswap(page ^ 3), see sys2_main_bus.decode_rom_bank) lands in mirror units the
// old map rejected as holes, so READS must fold mirror keys onto the stored copy.
//
// The fold is per-game and read-side only. Both properties were gotten wrong on the
// first attempt, each breaking a game the other attempt served:
//   * Our stream is not the region: manifests do not replicate ROM_RELOAD, so mirror
//     units are FILL in the stream (verified directly on the real download stream, paperboy AND
//     ssprint). A write-side fold let mirror-position fill OVERWRITE freshly loaded
//     canonical windows (Paperboy dropped to 4 distinct tile codes). The write port
//     therefore keeps the ORIGINAL canonical-set gate -- mirror-key writes are dropped.
//   * The mirror structure differs per game: unit 5 folds to 3 for a Sprint but to 2 for
//     Paperboy, and Paperboy's stream leaves unit 3's window (phys2, a Sprint window)
//     as FILL -- a game-blind Sprint-shaped fold sent Paperboy's banks into it (the
//     second 4-distinct-codes regression, same day). game_id on the READ side is safe
//     (reads happen long after the descriptor lands -- game_in_bram already relies on
//     that) and the fold happens BEFORE the rom_idx_q register, so the registered
// win_idx -> M10K hop is unchanged.
function automatic logic [4:0] canon_hi(input logic [4:0] hi, input logic [7:0] gid);
	if (WINDOWS >= 8) begin
		if (gid == SYS2_GAME_PAPERBOY)
			case (hi)
				5'd3,  5'd4,  5'd5:  canon_hi = 5'd2;   // quad 2..5
				5'd7,  5'd8,  5'd9:  canon_hi = 5'd6;   // quad 6..9
				5'd11, 5'd12, 5'd13: canon_hi = 5'd10;  // quad 10..13
				5'd15, 5'd16, 5'd17: canon_hi = 5'd14;  // quad 14..17
				default:             canon_hi = hi;
			endcase
		else
			case (hi)
				5'd4:    canon_hi = 5'd2;   // Sprint pair 2=4
				5'd5:    canon_hi = 5'd3;   // Sprint pair 3=5
				5'd12:   canon_hi = 5'd10;  // Sprint pair 10=12
				5'd13:   canon_hi = 5'd11;  // Sprint pair 11=13
				5'd16:   canon_hi = 5'd14;  // Sprint pair 14=16
				5'd17:   canon_hi = 5'd15;  // Sprint pair 15=17
				default: canon_hi = hi;     // 6..9 stay themselves (unpopulated for Sprints)
			endcase
	end else
		// Paperboy-only build: fold each quad onto its stored unit.
		case (hi)
			5'd3,  5'd4,  5'd5:  canon_hi = 5'd2;
			5'd7,  5'd8,  5'd9:  canon_hi = 5'd6;
			5'd11, 5'd12, 5'd13: canon_hi = 5'd10;
			5'd15, 5'd16, 5'd17: canon_hi = 5'd14;
			default:             canon_hi = hi;
		endcase
endfunction

// Canonical-set membership. The WRITE port uses this raw (mirror keys are not in the
// set, so mirror-position stream bytes -- fill -- are dropped); the READ port applies it
// to the already-folded rom_idx_q.
function automatic logic win_pop(input logic [4:0] hi);
	if (WINDOWS >= 8)
		win_pop = (hi == 5'd1)  || (hi == 5'd2)  || (hi == 5'd3)  || (hi == 5'd6) ||
		          (hi == 5'd10) || (hi == 5'd11) || (hi == 5'd14) || (hi == 5'd15);
	else
		win_pop = (hi == 5'd1) || (hi == 5'd2) || (hi == 5'd6) ||
		          (hi == 5'd10) || (hi == 5'd14);
endfunction

// Physical window number for CANONICAL keys. Game-independent, and it MUST match win_pop
// above: the two together are the placement, and the write and read ports both call them.
//
// Paperboy's physical indices MOVE in the family build (window 6 is now 3, not 2). That is
// safe only because BOTH ports use this one map -- which is exactly the property the per-game
// version lacked. Never let the two diverge.
function automatic logic [2:0] win_idx(input logic [4:0] hi);
	if (WINDOWS >= 8)
		case (hi)
			5'd1:    win_idx = 3'd0;  // logical 0x08000  (fixed ROM pair, all games)
			5'd2:    win_idx = 3'd1;  // logical 0x10000
			5'd3:    win_idx = 3'd2;  // logical 0x18000  (Sprints only)
			5'd6:    win_idx = 3'd3;  // logical 0x30000  (Paperboy only)
			5'd10:   win_idx = 3'd4;  // logical 0x50000
			5'd11:   win_idx = 3'd5;  // logical 0x58000  (Sprints only)
			5'd14:   win_idx = 3'd6;  // logical 0x70000
			5'd15:   win_idx = 3'd7;  // logical 0x78000  (Sprints only)
			default: win_idx = 3'd0;
		endcase
	else
		case (hi)
			5'd1:    win_idx = 3'd0; // logical 0x08000  (fixed ROM pair)
			5'd2:    win_idx = 3'd1; // logical 0x10000  (paged banks 0-3)
			5'd6:    win_idx = 3'd2; // logical 0x30000  (paged banks 16-19)
			5'd10:   win_idx = 3'd3; // logical 0x50000  (paged banks 32-35)
			5'd14:   win_idx = 3'd4; // logical 0x70000  (paged banks 48-51)
			default: win_idx = 3'd0;
		endcase
endfunction

// Which games this build can actually serve from BRAM. Read-side ONLY: 720 and APB need
// thirteen windows and are SDRAM-bound, so they read all-0xffff and fail loudly
// rather than running a partial image. Safe to use game_id here -- a read happens long after
// the descriptor has landed, unlike a write.
function automatic logic game_in_bram(input logic [7:0] gid);
	if (WINDOWS >= 8)
		game_in_bram = (gid == SYS2_GAME_PAPERBOY) || (gid == SYS2_GAME_SSPRINT) ||
		               (gid == SYS2_GAME_CSPRINT);
	else
		game_in_bram = (gid == SYS2_GAME_PAPERBOY);
endfunction

// Write port: assemble 16-bit words from the byte stream (even = low byte).
// Canonical units only -- win_pop is the canonical-set test, so mirror-position stream
// bytes (FILL: manifests do not replicate ROM_RELOAD) are dropped rather than allowed to
// overwrite the real window that loaded moments earlier (an earlier regression).
// This is byte-for-byte the pre-mirror-keys write behaviour; the fold lives on the read
// side, per-game (see canon_hi's header).
logic [16:0] wr_word;
assign wr_word = {win_idx(ld_addr[19:15]), ld_addr[14:1]};

always_ff @(posedge wr_clk) begin
	if (ld_we && win_pop(ld_addr[19:15])) begin
		if (ld_addr[0]) rom_hi[wr_word] <= ld_data;
		else            rom_lo[wr_word] <= ld_data;
	end
end

// Read port. The CPU drives rom_addr COMBINATIONALLY (sys2_main_bus computes it
// from the bus_addr register through a mux); that long bus_addr -> main_bus mux ->
// win_idx -> M10K read-address path is marginal on silicon, so the CPU fetched
// 0x0000 (HALT) at 0x8004 while a REGISTERED-address read of the same array came
// back byte-perfect on hardware.
// So register the address slices here first: bus_addr(reg) -> main_bus mux ->
// rom_idx_q/rom_off_q(reg) -> win_idx -> M10K splits the long path into two short
// registered hops, matching that registered read path. The latency grows to two clocks;
// the extra cycle is absorbed by the held ack (the T-11 waits on rom_ack).
logic [4:0]  rom_idx_q;   // registered rom_addr[19:15] (0x8000-granular window index)
logic [13:0] rom_off_q;   // registered rom_addr[14:1]  (word offset within a window)
logic [16:0] rd_word;
logic        rd_pop;
assign rd_word = {win_idx(rom_idx_q), rom_off_q};
// game_id is legitimate HERE (a read is long after the descriptor lands) and only narrows the
// map: 720/APB read all-0xffff so their absence is loud. Placement itself never consults it.
assign rd_pop  = win_pop(rom_idx_q) && game_in_bram(game_id);

// Reads are word-aligned; the main bus drives rom_addr[0] = 0. The port stays
// byte-addressed so it wires directly to the bus.

logic [15:0] rd_data;
logic        rd_hole;
logic        req_d, req_d2;

always_ff @(posedge clk) begin
	if (reset) begin req_d <= 1'b0; req_d2 <= 1'b0; end
	else       begin req_d <= rom_req; req_d2 <= req_d; end
	// The per-game mirror fold happens HERE, before the register, so rom_idx_q is already
	// canonical and the registered win_idx -> M10K hop is exactly as deep as
	// before the mirror keys existed. game_id is stable long before any read.
	rom_idx_q <= canon_hi(rom_addr[19:15], game_id);
	rom_off_q <= rom_addr[14:1];
	rd_data   <= {rom_hi[rd_word], rom_lo[rd_word]};
	rd_hole   <= ~rd_pop;
end

// Level-held ack: asserted two clocks after rom_req (the address is registered
// once, then the BRAM read is registered) and kept high while the request stands,
// so a CPU running on a divided clock enable cannot miss it between its active
// edges. The T-11 holds rom_addr/rom_req until it sees the ack, then deasserts.
assign rom_rdata = rd_hole ? 16'hffff : rd_data;
assign rom_ack   = rom_req & req_d2;

endmodule
