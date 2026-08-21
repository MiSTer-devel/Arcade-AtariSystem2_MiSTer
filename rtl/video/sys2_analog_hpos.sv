// SPDX-License-Identifier: GPL-3.0-or-later
`timescale 1ns/1ps

//============================================================================
//  sys2_analog_hpos.sv -- analog "CRT H-Position" fine alignment.
//
//  Derived from analog_hsize.sv by Umberto Parisi (rmonic79), GPL v3 or later,
//  in Arcade-Raiden_MiSTer (https://github.com/rmonic79/Arcade-Raiden_MiSTer).
//  That module also implements a variable pixel STRETCH (H-Size) by reading the
//  line buffer at a slower clock enable. Paperboy cannot use that half: its
//  pixel clock is 16 MHz on a 32 MHz clk_sys, i.e. only TWO CLK_VIDEO periods
//  per pixel, and sys_top samples the analog path on CLK_VIDEO gated by
//  CE_PIXEL -- so the finest stretch step available here is half a pixel
//  (Raiden has 16 clocks per pixel, where the same step is 1/16 pixel and
//  invisible). Only the horizontal REPOSITION half is kept, which is exact at
//  any clock ratio because it moves whole pixels.
//
//  What it does: the picture -- and the active window it sits in -- slide
//  horizontally inside an untouched HSync. Sync timing is not altered at all,
//  so no CRT can lose lock however far the picture is moved; the limit is the
//  blanking budget (Paperboy has a 32-pixel front porch and a 32-pixel back
//  porch, so +/-32 is the useful range before the picture runs into HSync).
//
//  Cost: one 24-bit ping-pong line buffer (2 * 2**AW * 24 bits of M10K) plus a
//  handful of ALMs. The read side emits the PREVIOUS line, which is what lets
//  the offset go negative (read ahead of the write pointer); the whole analog
//  picture therefore sits one scanline lower while this is enabled. The top
//  level bypasses the module completely at hoffset == 0, so the default build
//  is bit-identical to having no line buffer at all.
//
//  Alignment: r/g/b_out at raster position p carry the source pixel from
//  p - hoffset, and de_out is the source active window moved by the same
//  amount. The module's own read latency is folded into hoff_eff, so hoffset is
//  exact in pixels and hoffset == 0 reproduces the input stream (one line late)
//  sample for sample. Simulation checks that identity against the real raster.
//============================================================================

module sys2_analog_hpos #(
	// Per-bank address bits. 2**AW must be >= the horizontal total (640 here).
	parameter int AW = 10
) (
	input  logic              clk,
	input  logic              reset,
	input  logic              ce_pix,

	// Signed pixel offset. >0 moves the picture RIGHT, <0 moves it LEFT.
	input  logic signed [6:0] hoffset,

	input  logic [7:0]        r_in,
	input  logic [7:0]        g_in,
	input  logic [7:0]        b_in,
	input  logic              hs_in,   // NATIVE HSync: line/bank boundary only, never the
	                                   // OSD-shifted one (a large H-Shift would otherwise
	                                   // reset the write pointer inside the active region)
	input  logic              de_in,   // active video = ~(hblank | vblank)

	output logic [7:0]        r_out,
	output logic [7:0]        g_out,
	output logic [7:0]        b_out,
	output logic              de_out
);

// Positional read delay, in ce_pix ticks. The output is two registers deep (memory read register,
// then output register) but the read ADDRESS is combinational from wrp, so only one of the two
// moves the sample relative to the raster -- exactly like the renderer's sync pipeline, whose
// first stage also samples the counters with no positional delay.
localparam int RD_DELAY = 1;
// Compare width: AW bits of raster position, plus sign and one bit of headroom
// so hb0 + hoffset cannot overflow.
localparam int EW = AW + 2;

// ---------------------------------------------------------------------------
//  Ping-pong line buffer. Written by the current line into `bank`, read by the
//  previous line out of `~bank`, so a read may run ahead of the write pointer.
// ---------------------------------------------------------------------------
// no_rw_check drops the read-during-write bypass MUXes: the read is old-data by
// construction (it is on the other bank) and nothing here wants forwarding.
(* ramstyle = "no_rw_check, M10K" *) logic [23:0] mem [0:(2**(AW+1))-1];

logic [AW-1:0] wrp;                 // write/read pointer, zeroed at each HSync
logic          bank;
logic          hs_d, de_d;
logic [AW-1:0] hb1, hb0;            // active window in wrp units: [hb1, hb0)
// Vertical gate. It is tempting to take vblank as an input and delay it a line, which is what the
// module this came from does -- but a buffer line runs HSync-to-HSync while a raster line runs from
// h=0, so the two are offset by the back porch: the active pixels inside a buffer line belong to
// the NEXT raster line. Gating on a vblank sampled at the HSync boundary therefore blanks the first
// active line and emits a spurious one at the bottom. Latching whether the line actually carried
// any active video is self-aligning and needs no vblank input at all.
logic          de_seen, de_seen_prev;
logic [23:0]   rd_data;
logic          pass_q;

wire hs_rise = hs_in & ~hs_d;

wire signed [EW-1:0] hoff_ext = $signed({{(EW-7){hoffset[6]}}, hoffset});
wire signed [EW-1:0] hoff_eff = hoff_ext - EW'(RD_DELAY);

wire signed [EW-1:0] wrp_s = $signed({2'b00, wrp});
wire signed [EW-1:0] hb1_s = $signed({2'b00, hb1});
wire signed [EW-1:0] hb0_s = $signed({2'b00, hb0});

wire [AW-1:0] rd_addr = AW'(wrp_s - hoff_eff);
wire          in_win  = (wrp_s >= (hb1_s + hoff_eff)) && (wrp_s < (hb0_s + hoff_eff));

// The RAM block is deliberately RESET-free. A reset on rd_data makes it a register the M10K's own
// output register cannot be, so Quartus has to read the array combinationally and reports
// "uninferred due to asynchronous read logic" -- which lands the whole 2048x24 buffer in ~49k
// flip-flops instead of 5 M10K. sys2_dpram.sv keeps its RAM blocks reset-free for the same
// reason. mem[wrp] holds the sample taken at wrp: the write address is the pre-increment pointer,
// so the write and read sides share one phase.
always_ff @(posedge clk) if (ce_pix) begin
	mem[{bank, wrp}] <= {r_in, g_in, b_in};
	rd_data          <= mem[{~bank, rd_addr}];
end

always_ff @(posedge clk) begin
	if (reset) begin
		wrp <= '0; bank <= 1'b0;
		hs_d <= 1'b0; de_d <= 1'b0;
		hb1 <= '0; hb0 <= '0;
		de_seen <= 1'b0; de_seen_prev <= 1'b0;
		pass_q <= 1'b0;
		r_out <= '0; g_out <= '0; b_out <= '0; de_out <= 1'b0;
	end else if (ce_pix) begin
		hs_d <= hs_in;
		de_d <= de_in;

		if (hs_rise) begin
			wrp          <= '0;
			bank         <= ~bank;
			de_seen_prev <= de_seen;      // did the line now being emitted carry video?
			de_seen      <= de_in;
		end else begin
			wrp     <= wrp + 1'b1;
			de_seen <= de_seen | de_in;
		end

		if ( de_in & ~de_d) hb1 <= wrp;   // first active sample of the line
		if (~de_in &  de_d) hb0 <= wrp;   // first sample past the active region

		pass_q <= in_win & de_seen_prev;

		{r_out, g_out, b_out} <= pass_q ? rd_data : 24'd0;
		de_out                <= pass_q;
	end
end

endmodule
