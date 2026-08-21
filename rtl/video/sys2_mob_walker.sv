// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// Paperboy motion-object linked-list walker (MAME atarimo.cpp build_active_list /
// render_object @ d066f16). Once per
// scanline build it walks the active list and composites every object covering the
// target line into the line buffer's build half.
//
// This follows the SCHEMATIC, not MAME. The board resets the link
// address to 0 at the line boundary and then blindly loads the next link from
// each object's link field, for exactly MAXPERLINE (40) slots -- there is no
// visited[] array, no end marker and no repeat detection anywhere in the link
// latch/timing logic. MAME does two things the PCB does not: it sets
// maxperline = 0 (effectively unlimited) and it keeps a visited[] bitmap and
// stops the moment a link repeats. We used to copy both. Consequences of the
// real behaviour: a terminal self-linked object is re-processed until the 40
// slots expire (harmless when it re-renders identical pixels), and a short
// CYCLIC list of overlapping visible objects can end with a different final
// overwrite order than MAME produces. The walk cannot hang because the slot
// budget bounds it -- the visited bitmap was never what made it safe.
// The starting object IS rendered (start_link is the object-zero/link latch
// head). Objects are composited in walk order with later writes
// overwriting earlier opaque pixels -- MAME draws the active list front-to-back
// into one bitmap, so the last list entry covering a pixel wins.
//
// render_object coordinate order (verbatim from MAME, atarisy2 xoffset/scroll/
// nextneighbor all 0): xpos = X_field; (next_xpos hold inert); if neighbor xpos =
// last_xpos + 16; last_xpos = xpos (pre-fold); then xpos &= 0x3ff and if
// xpos >= 512 xpos -= 1024. Vertical placement and the sprite-internal row are the
// sys2_mob_vscan. last_xpos is an 11-bit register here (MAME
// uses an unbounded int); they differ only for a neighbour chain whose raw xpos
// exceeds 2047, which does not occur in Paperboy.
//
// The sprite ROM is read through an abstract, latency-tolerant request/valid port
// (one 16-pixel row per request: tile + sub-row -> 8 planar bytes), matching the
// SDRAM graphics burst shape; the walker stalls on the handshake so any read
// latency is tolerated.
module sys2_mob_walker #(
	// Motion-object list-traversal budget per scanline.
	//
	// This is the real hardware budget, not MAME's. The linked-list
	// address advances on a strobe generated once every 16 of the 640 horizontal
	// clocks and the pointer resets to the head each line, so the board visits
	// exactly 640/16 = 40 entries per scanline (schematic-derived, from the
	// 82S131 / MO timing analysis). Entries whose vertical-match test FAILS still
	// consume a slot -- `count` increments once per entry processed, the same
	// semantics -- so this is a TRAVERSAL budget, not a "40 visible sprites"
	// limit. Objects past the 40th visited entry are deterministically omitted
	// for that line (they only appear to flicker if software reorders the list).
	// The board runs ALL 40 slots unconditionally -- it never stops early, even
	// on a repeated or self-referencing link.
	//
	// MAME uses maxperline = 0 (unlimited) and we previously followed it at 256,
	// so this is a DELIBERATE divergence from our own oracle. Override to 256 to
	// reproduce MAME when comparing against MAME rather than against the board.
	parameter int MAXPERLINE = 40
) (
	input  logic        clk,
	input  logic        reset,

	// Build trigger: 1-clk `start` (re)initialises the walk for `build_line`.
	input  logic        start,
	input  logic [8:0]  build_line,   // target scanline (0..383), valid at `start`
	input  logic [7:0]  start_link,   // list head (object-zero/link latch; tie 0)

	// Motion-object RAM read port (sys2_main_bus mob_video_*; 1-cyc latency).
	output logic [9:0]  mob_addr,
	input  logic [15:0] mob_data,

	// Abstract sprite-ROM row read port (latency-tolerant request/valid).
	output logic        sp_req,
	output logic [13:0] sp_tile,
	output logic [3:0]  sp_row,
	input  logic        sp_valid,
	input  logic [63:0] sp_data,      // {row_second[31:0], row_first[31:0]}

	// Line-buffer composite write (build buffer).
	output logic        lb_we,
	output logic [8:0]  lb_col,
	output logic [7:0]  lb_data,      // {priority[1:0], color[1:0], pen[3:0]}

	output logic        busy
);


// ---- FSM ----
typedef enum logic [2:0] {S_IDLE, S_FETCH, S_DECIDE, S_FROW, S_COMP, S_NEXT} state_t;
state_t state;

logic [7:0]  cur_link;
logic [8:0]  bline;
logic [8:0]  count;            // objects processed (0..256)
logic [10:0] last_xpos;        // raw (pre-fold) X carried for the neighbor chain
logic [2:0]  fc;               // word-fetch phase 0..4

// Latched entry words.
logic [15:0] word0, word1, word2, word3;

// Latched per-object render context (valid from S_FROW onward).
logic signed [10:0] comp_x;    // screen left column, [-512, 511]
logic [1:0]  comp_color, comp_prio;
logic        comp_hflip;
logic [13:0] fetch_tile;
logic [3:0]  sub_row;
logic [3:0]  cc;               // composite column 0..15
logic [7:0]  nlink;            // this entry's link, latched for S_NEXT

logic [31:0] row_first, row_second;

// ---- combinational decode of the latched entry ----
logic [7:0]  d_link;
logic [13:0] d_code;
logic [1:0]  d_color;
logic [9:0]  d_xpos;
logic [8:0]  d_ypos;
logic [3:0]  d_height;
logic        d_hflip, d_neighbor;
logic [1:0]  d_prio;

sys2_mob_decode u_decode (
	.word0(word0), .word1(word1), .word2(word2), .word3(word3),
	.link(d_link), .code(d_code), .color(d_color), .xpos(d_xpos), .ypos(d_ypos),
	.height(d_height), .hflip(d_hflip), .neighbor(d_neighbor), .priority_lvl(d_prio)
);

logic        d_on_line;
logic [6:0]  d_v;
sys2_mob_vscan u_vscan (
	.y_field(d_ypos), .height(d_height), .scanline(bline),
	.on_line(d_on_line), .v_in_sprite(d_v)
);

// raw X for this object (neighbor override), and the screen-folded left column.
// rawx is 11-bit; the neighbor add wraps mod 2048 (the documented finite-width
// deviation from MAME's unbounded int, immaterial for Paperboy's lists).
logic [10:0] rawx;
logic signed [10:0] xleft;
always_comb begin
	rawx  = d_neighbor ? (last_xpos + 11'd16) : {1'b0, d_xpos};
	// Fold the 10-bit wrapped X into the signed screen range [-512,511]. x >= 512 <=> bit 9
	// set, and (x - 1024) in 11-bit two's complement == {1'b1, x[9:0]}, so the fold is a plain
	// sign-extension of bit 9 -- bit-identical to the previous explicit subtract, without the
	// former `11'sd1024` literal that overflowed an 11-bit signed (Quartus constant-overflow
	// warning; the literal silently read as -1024, still correct modulo 2048).
	xleft = $signed({rawx[9], rawx[9:0]});
end

// ---- sprite pens for the composite ----
logic [63:0] pens_screen;
sys2_mob_pen u_pen (
	.row_first(row_first), .row_second(row_second), .hflip(comp_hflip),
	.pens_screen(pens_screen)
);

// ---- combinational outputs ----
assign busy    = (state != S_IDLE);
assign mob_addr = {cur_link, (fc >= 3'd3) ? 2'd3 : fc[1:0]};
assign sp_req  = (state == S_FROW);
assign sp_tile = fetch_tile;
assign sp_row  = sub_row;

logic signed [10:0] sc;
logic [3:0] cur_pen;
// `!start` is the buffer-swap race, and it is a CORRECTNESS gate, not an optimisation.
// The engine's buf_sel is v_count[0], and sys2_video_timing advances v_count on the same
// clk_32 edge that raises ce_pix -- so in the cycle where `start` is high, buf_sel ALREADY
// holds the new line's value. If the previous line's build has not finished, this walk is
// still in S_COMP that cycle and lb_we is still high, so its last composite write is steered
// by sys2_mob_linebuf into the buffer that has just become the DISPLAY buffer. One stray
// motion-object pixel is deposited on the line now being shown, at a column nothing drew.
//
// That pixel belongs to a line-build being abandoned on the very next edge (`start` forces
// state <= S_FETCH), so suppressing it can never remove a wanted pixel: when the build DID
// finish, state is S_IDLE here and lb_we is already low. Measured in simulation
// 720/moheight, which overruns once the line budget is squeezed: +extralat 2/3/4 gave 2/1/2
// differing bytes on realrom=1, and +cpu_load=8 gave 2 on realrom=2, the shipping path. All of
// them "RTL draws where model does not", with ZERO stale fetches, and the sprite fetch counts
// are identical with and without this gate -- the data was always right and the walk always
// behaved the same; the pixel was written to the wrong buffer.
// Generalise: A double buffer's swap is a deadline, and any write still in flight when it
// passes is applied to the other buffer, not dropped.
always_comb begin
	lb_we = 1'b0; lb_col = 9'd0; lb_data = 8'd0;
	sc      = comp_x + $signed({7'b0, cc});
	cur_pen = pens_screen[4*cc +: 4];
	if (state == S_COMP && !start && sc >= 0 && sc <= 11'sd511 && cur_pen != 4'hf) begin
		lb_we   = 1'b1;
		lb_col  = sc[8:0];
		lb_data = {comp_prio, comp_color, cur_pen};
	end
end

// The ONLY stop condition is the hardware slot budget. Deliberately no repeat or
// self-link test -- see the header note.
wire cap_reached = (count + 9'd1) >= 9'(MAXPERLINE);

always_ff @(posedge clk) begin
	if (reset) begin
		state <= S_IDLE; cur_link <= 8'd0; bline <= 9'd0;
		count <= 9'd0; last_xpos <= 11'd0; fc <= 3'd0; cc <= 4'd0;
		word0 <= 0; word1 <= 0; word2 <= 0; word3 <= 0; nlink <= 0;
		comp_x <= 0; comp_color <= 0; comp_prio <= 0; comp_hflip <= 0;
		fetch_tile <= 0; sub_row <= 0; row_first <= 0; row_second <= 0;
	end else if (start) begin
		// (Re)start a build, preempting any in-progress walk.
		cur_link  <= start_link;
		bline     <= build_line;
		count     <= 9'd0;
		last_xpos <= 11'd0;
		fc        <= 3'd0;
		state     <= S_FETCH;
	end else begin
		case (state)
			S_FETCH: begin
				// mob_addr is combinational; capture word N one cycle after issue.
				case (fc)
					3'd1: word0 <= mob_data;
					3'd2: word1 <= mob_data;
					3'd3: word2 <= mob_data;
					3'd4: word3 <= mob_data;
					default: ;
				endcase
				if (fc == 3'd4) begin state <= S_DECIDE; fc <= 3'd0; end
				else fc <= fc + 3'd1;
			end

			S_DECIDE: begin
				// last_xpos updates for every object (MAME: unconditional, pre-fold).
				last_xpos <= rawx[10:0];
				nlink     <= d_link;
				if (d_on_line) begin
					comp_x     <= xleft;
					comp_color <= d_color;
					comp_prio  <= d_prio;
					comp_hflip <= d_hflip;
					// Tile-row stepping wraps within the low three bits -- no carry into
// bit 3. The 82S131 at 5P (confirmed from schematic
	// sheet 12A) takes only MOPIC[2:0] as input and its three outputs
					// supply MO graphics ROM address [8:6]; the higher MOPIC bits bypass
					// the PROM entirely into the ROM address latches. Its decoded equation
					// ends in `& 7`, so the sum can never reach the bits that bypassed it.
					// MAME increments the combined integer code and DOES carry, so this is
					// a deliberate divergence from our own oracle -- and one no
					// MAME-comparison can detect, because MAME shares the error. It is also
					// invisible in all three golden scenes (none has an object where
					// (code & 7) + tile_row >= 8), so the walker bench carries it.
					fetch_tile <= {d_code[13:3], d_code[2:0] + d_v[6:4]};
					sub_row    <= d_v[3:0];
					cc         <= 4'd0;
					state      <= S_FROW;
				end else begin
					state <= S_NEXT;
				end
			end

			S_FROW: begin
				if (sp_valid) begin
					row_first  <= sp_data[31:0];
					row_second <= sp_data[63:32];
					cc         <= 4'd0;
					state      <= S_COMP;
				end
			end

			S_COMP: begin
				// lb_* are driven combinationally for the current cc.
				if (cc == 4'd15) state <= S_NEXT;
				else cc <= cc + 4'd1;
			end

			S_NEXT: begin
				count <= count + 9'd1;
				if (cap_reached) begin
					state <= S_IDLE;
				end else begin
					cur_link <= nlink;
					fc       <= 3'd0;
					state    <= S_FETCH;
				end
			end

			default: state <= S_IDLE;   // S_IDLE
		endcase
	end
end

endmodule
