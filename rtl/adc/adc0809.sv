// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// ADC0808/0809 eight-channel, eight-bit successive-approximation ADC, modelled
// at the digital behavior Paperboy software can observe. The analog inputs are
// presented already digitized as eight-bit values, one per channel.
//
// A one-clk `start` strobe latches the channel address (the ADC's ALE) and
// begins a conversion. While converting, `eoc` is low (busy); after CONV_CLOCKS
// `clk_en` ticks the selected channel is captured into the output latch and
// `eoc` returns high. The output latch holds the last completed result until the
// next conversion finishes, so a bus read between conversions returns the prior
// value exactly like the chip's three-state output buffer. A `start` during a
// conversion re-latches the address and restarts, matching the device.
//
// Source: National ADC0808/ADC0809 data sheet (8 clocks/bit, 64-clock nominal
// conversion at f_CLK); MAME d066f16 adc0808_device functional model. The conversion
// time is settled: the default 64 clocks on the exact 625 kHz enable = 102.4 us sits
// inside the data-sheet 90-116 us window, and the handlebar, calibration and TM-275
// control diagnostics all behave correctly on hardware at this setting.
// CONV_CLOCKS stays a parameter for accelerated bench use.
module adc0809 #(
	parameter int unsigned CONV_CLOCKS = 64
) (
	input  logic        clk,
	input  logic        reset,
	input  logic        clk_en,            // ADC clock enable (~625 kHz nominal)

	input  logic [7:0]  channel_in [0:7],  // eight analog channels, pre-digitized

	input  logic        start,             // ALE + START strobe, one clk pulse
	input  logic [2:0]  channel,           // channel address, latched at start

	output logic [7:0]  data,              // output latch: last completed result
	output logic        eoc                // 1 = idle/complete, 0 = converting
);

localparam int CW = (CONV_CLOCKS < 2) ? 1 : $clog2(CONV_CLOCKS);
localparam logic [CW-1:0] CONV_LAST = CW'(CONV_CLOCKS - 1);

logic [CW-1:0] conv_count;
logic [2:0]    sel;
logic          busy;

assign eoc = ~busy;

always_ff @(posedge clk) begin
	if (reset) begin
		busy       <= 1'b0;
		conv_count <= '0;
		sel        <= 3'd0;
		data       <= 8'd0;
	end else if (start) begin
		// ALE latches the address; START (re)starts the conversion.
		sel        <= channel;
		busy       <= 1'b1;
		conv_count <= CONV_LAST;
	end else if (busy && clk_en) begin
		if (conv_count == '0) begin
			busy <= 1'b0;
			data <= channel_in[sel];   // capture the selected channel at completion
		end else begin
			conv_count <= conv_count - 1'b1;
		end
	end
end

endmodule
