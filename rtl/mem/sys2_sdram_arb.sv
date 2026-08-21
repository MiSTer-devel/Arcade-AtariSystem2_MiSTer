// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.

`timescale 1ns/1ps

// SDRAM read-side arbiter for the Atari System 2 core.
//
// Three read clients share one controller, in fixed priority:
//
//   0  CPU          highest. Single word. The T-11 has ~300 ns per bus transaction and
//                   no way to stall gracefully, so it must never queue behind a burst.
//   1  MO fetch     burst. Per-line slack: a scanline's worth of motion-object rows.
//   2  PF prefetch  lowest. A full line of slack -- it is filling the NEXT line's buffer.
//
// Fixed priority rather than round-robin is deliberate: the slack ordering above is a
// property of the video pipeline, not a fairness question, and CPU latency is the one
// budget with no headroom to give away.
//
// ---------------------------------------------------------------------------
// Two-deep requests -- why the interface looks like this
//
// Measured for the playfield's one-burst-per-column pattern: only
// 13.3 % of cycles carry data, and 33.6 % have no burst active at all -- the controller
// idling in S_IDLE plus this arbiter's request handshake, between one column's burst and
// the next. At CLK_MHZ=32 tRCD and tRP are both ONE cycle, so none of that is memory
// timing. It is request-handoff latency, and it is the largest single cost.
//
// Closing it needs the next burst already latched in the controller when the current one
// retires. The controller side provides that (`sd_brst_accept` latches from any state,
// S_BTRP chains straight into it). But a client that raises `req`, waits for `done`, then
// raises the next cannot supply it: its next address does not exist until the current
// burst has already retired, which is precisely the gap being removed.
//
// An earlier attempt pre-granted from the SAME live `req` while that client's burst was
// still running. That is wrong and fails loudly: `*_left` counts words not yet DELIVERED,
// so the arbiter queues a second burst at an address INSIDE the burst already in flight.
// It showed up as MO receiving PF's data ("expected 10 words, got 15"). Do not retry it.
//
// So each burst client presents TWO requests: a head (`cN_req/addr/len`) and an optional
// second (`cN_req2/addr2/len2`). The arbiter hands the head over first, then the second,
// and `cN_done` pulses when the HEAD completes -- at which point the client shifts its
// second up into the head slot. sys2_pf_linebuf already computes the next column's
// address ahead of time (its map fetch is pipelined against the data burst), so it has a
// real second request to offer.
//
// `done` is a PULSE again, not the held level it briefly was. Two-deep forces it: with
// two requests outstanding, two completions must be distinguishable, and a level held
// until `req` drops would merge them. Clients must therefore sample `done` every cycle.
// Every real client does; the one thing that ever missed it was a bench blocked in an
// unrelated wait loop, which is what motivated the level in the first place.
//
// PREEMPTION, and the ordering rule that makes two-deep safe. A burst is aborted at a word
// boundary when a higher-priority client asks -- but ONLY when nothing is buffered behind
// it. If a second burst is already latched in the controller, aborting the running one
// would let the buffered one run first, and for the same client that delivers its two
// requests out of order. So with something buffered the arbiter simply waits: the CPU
// then waits at most the running burst plus the buffered one.
//
// That is a real, accepted cost: worst-case CPU grant latency roughly doubles. It is
// acceptable because an early decision settled that the T-11 program stays in BRAM wherever it
// fits (Paperboy, ssprint, csprint), so CPU-through-SDRAM applies only to 720/APB, where
// the latency was already outside the ~300 ns budget and needs its own answer regardless.
// The alternative -- cancelling the buffered burst and re-queueing both in order -- costs
// a controller cancel port and a re-order path for no benefit on any shipping game.
//
// sys2_sdram still returns the reads already in flight (up to CL) after an abort, and
// those words belong to the burst owner, so ownership is not released until the burst
// actually retires. The arbiter counts what each client received and resumes the remainder
// from `addr + delivered`, so a preempted burst stays transparent to its client: it still
// sees exactly `len` words, in order, and one `done`.
//
// Ownership moves on `sd_brst_start`, NOT on `sd_brst_done`. With chaining the two land
// on the same edge, so `done` alone cannot tell a chained handover from a plain
// retirement -- and getting that wrong misroutes a CPU read that slips in between, because
// `sd_valid` is shared and `c0_valid` is gated on `owner`.
//
// The WRITE side is not arbitrated. During ROM download the loader owns the controller
// exclusively -- the pattern proven on silicon (raw-ioctl writes, no
// loader-FIFO drain) -- and no read client is running then.
// ---------------------------------------------------------------------------
module sys2_sdram_arb #(
	parameter int AW = 24
) (
	input  logic          clk,
	input  logic          reset,

	// ---- client 0: CPU instruction fetch, BURST, highest priority ----
	//
	// Was single-word at first, and that is what corrupted the playfield background
	// on 720/APB -- the only games reading their program from SDRAM. sys2_cpu_icache fills a
	// line through c0 and `can_hand` carries `&& !c0_req`, so the playfield's two-deep handoff
	// is fenced off for the WHOLE fill. As single words that is one round trip per word.
	// Measured in simulation (deadline 1279 clk): the fill overruns past ~24 CPU WORDS on
	// one scanline, and cost tracks WORDS not misses. As one burst a fill costs one round trip.
	//
	// Granted only when the controller is completely idle (`owner == OWN_NONE &&
	// queued == OWN_NONE`) -- the SAME condition the old single-word path used, deliberately.
	// Letting the CPU take the queue slot while a burst was RUNNING (tried first) changes when
	// PF's second is handed, and then a head plus its promoted second can both complete before
	// the client shifts -- the head-consumed interlock remembers only ONE, so one request is
	// fetched twice ("expected 32 words, got 48"). Keeping the CPU out of the queue behind a
	// running burst leaves MO/PF timing exactly as it was proven.
	//
	// The CPU is never preempted (`sd_brst_abort` is only raised for MO/PF), so a CPU burst
	// always completes -- no delivered-count and no resume-from-partial.
	input  logic          c0_req,
	input  logic [AW-1:0] c0_addr,
	input  logic [8:0]    c0_len,
	output logic          c0_valid,
	output logic          c0_done,

	// ---- client 1: motion objects, burst ----
	// Head request; held (with stable addr/len) until c1_done pulses.
	input  logic          c1_req,
	input  logic [AW-1:0] c1_addr,
	input  logic [8:0]    c1_len,
	// Second request, presented while the head is still running. Tie req2 low if the
	// client does not pipeline; everything still works, just without the win.
	input  logic          c1_req2,
	input  logic [AW-1:0] c1_addr2,
	input  logic [8:0]    c1_len2,
	output logic          c1_valid,
	output logic          c1_done,     // PULSE, on completion of the HEAD request

	// ---- client 2: playfield prefetch, burst, lowest priority ----
	input  logic          c2_req,
	input  logic [AW-1:0] c2_addr,
	input  logic [8:0]    c2_len,
	input  logic          c2_req2,
	input  logic [AW-1:0] c2_addr2,
	input  logic [8:0]    c2_len2,
	output logic          c2_valid,
	output logic          c2_done,     // PULSE, on completion of the HEAD request

	// Shared read data. Each client latches it on its own valid.
	output logic [15:0]   rdata,

	// ---- to sys2_sdram ----
	output logic [AW-1:0] sd_addr,
	output logic          sd_req,
	output logic [AW-1:0] sd_brst_addr,
	output logic [8:0]    sd_brst_len,
	output logic          sd_brst_req,
	output logic          sd_brst_abort,
	input  logic          sd_ready,
	input  logic [15:0]   sd_rdata,
	// Single-word return strobe -- routes client 0 ONLY. On sys2_sdram this is the shared
	// `valid`, which also pulses for any single-word client wired straight to the controller
	// past this arbiter (the top level's sprite ROM). Burst clients must never watch it.
	input  logic          sd_valid,
	// BURST return strobe, one pulse per burst word and nothing else (sys2_sdram's
	// brst_valid). Clients 1 and 2 and their delivery counters route off THIS.
	//
	// Hardening, not a bug fix -- see the same note on sys2_sdram.brst_valid. Routing
	// bursts off `sd_valid` was safe in practice because a single word can only be accepted
	// from S_IDLE, which a burst must have retired to reach, and ownership is released on that
	// retirement. Simulation measured the coincidence directly: 50,287 single-word returns,
	// `owner` was OWN_NONE at all of them. The split removes the dependence on that ordering
	// rather than fixing an observed failure, and changes no behaviour in any build.
	input  logic          sd_brst_valid,
	input  logic          sd_brst_done,
	input  logic          sd_brst_accept,   // controller can latch another burst now
	input  logic          sd_brst_start     // a burst actually began this cycle
);

typedef enum logic [1:0] { OWN_NONE, OWN_CPU, OWN_MO, OWN_PF } owner_t;

owner_t owner;    // whose data is coming back right now
owner_t queued;   // whose burst sits in the controller's buffer, not yet started

// Words of the HEAD request already delivered to each burst client. Survives preemption;
// cleared when that head completes and the client shifts its second request up.
logic [8:0] d_mo, d_pf;

// Which of each client's two requests are currently handed to the controller.
//
// These are two separate flags, NOT a 0..2 count. A count conflates "head handed,
// second not yet" with "both handed, head has since completed" -- both read 1 -- and on
// that ambiguity the arbiter hands the still-asserted `req2` a SECOND time: the same
// 4-word request is fetched twice and the duplicate leaks into whatever reads next.
logic h_c0;         // head handed, CPU (no second slot: one line fill at a time)
logic h_mo, s_mo;   // head handed / second handed, motion objects
logic h_pf, s_pf;   // ditto, playfield

// ---------------------------------------------------------------------------
// Head-consumed interlock.
//
// `d_*` must be cleared the instant a head completes, because the SECOND request may
// already be running and its words start landing on the very next cycle -- clearing later
// would fold the head's count into the second's.
//
// But the client is still presenting the completed head for a cycle or two (it shifts on
// `done`), and with `d_*` back at zero the head looks freshly requested and would be
// handed over AGAIN. That is a real bug: MO re-received its whole
// previous request, "expected 10 words, got 42", the 42 being 32 stale + 10 real.
//
// So a head that has completed is marked consumed, and stays that way until the client
// presents something different. Comparing {addr,len} rather than waiting for `req` to drop
// lets a pipelined client shift its second up without ever deasserting `req`.
//
// Consequence: a client must not issue two IDENTICAL consecutive requests (same addr
// and len) without dropping `req` between them, or the second is swallowed as already
// done. The playfield CAN present identical consecutive requests -- two adjacent columns
// showing the same tile -- so it drops `req` for a cycle between fills (see
// sys2_pf_linebuf); the motion-object fetcher walks distinct slots. The `!req` clear
// below covers any client that does the same.
// ---------------------------------------------------------------------------
logic          hd_c0, hd_mo, hd_pf;
logic [AW-1:0] hd_addr_c0, hd_addr_mo, hd_addr_pf;
logic [8:0]    hd_len_c0,  hd_len_mo,  hd_len_pf;

// c0's interlock records what was HANDED, latched at grant. Sampling c0_addr at retire reads
// whatever the CPU moved on to -- it keys off `valid`, not `done` -- and marks that NEW
// address already-consumed, which deadlocks the fill.
wire c0_head_stale = hd_c0 && (c0_addr == hd_addr_c0) && (c0_len == hd_len_c0);

wire mo_head_stale = hd_mo && (c1_addr == hd_addr_mo) && (c1_len == hd_len_mo);
wire pf_head_stale = hd_pf && (c2_addr == hd_addr_pf) && (c2_len == hd_len_pf);

// The SECOND slot needs the same interlock, for the same reason one step later. When a
// head completes the second is PROMOTED to head (h <= s), which clears the second slot --
// but the client is still presenting the old req2 for a cycle or two until it shifts.
// Without this it gets handed again as a fresh second.
logic          sd_mo, sd_pf;
logic [AW-1:0] sd_addr2_mo, sd_addr2_pf;
logic [8:0]    sd_len2_mo,  sd_len2_pf;

wire mo_2nd_stale = sd_mo && (c1_addr2 == sd_addr2_mo) && (c1_len2 == sd_len2_mo);
wire pf_2nd_stale = sd_pf && (c2_addr2 == sd_addr2_pf) && (c2_len2 == sd_len2_pf);

assign rdata = sd_rdata;

// Route each returned word to whoever owns the transfer. Client 0 is the single-word path,
// clients 1 and 2 are bursts -- so they take DIFFERENT strobes. See sd_brst_valid above.
assign c0_valid = sd_brst_valid && (owner == OWN_CPU);   // a burst client now
assign c1_valid = sd_brst_valid && (owner == OWN_MO);
assign c2_valid = sd_brst_valid && (owner == OWN_PF);

// Remaining work on each head, and where a resumed head picks up.
wire [8:0]    mo_left = c1_len - d_mo;
wire [8:0]    pf_left = c2_len - d_pf;
wire [AW-1:0] mo_next = c1_addr + AW'(d_mo);
wire [AW-1:0] pf_next = c2_addr + AW'(d_pf);

// What each client would like handed over next, in its own order: head first (possibly
// resuming mid-way), then its second request.
wire mo_wants_head = !h_mo && c1_req  && (mo_left != 0) && !mo_head_stale;
wire mo_wants_2nd  =  h_mo && !s_mo && c1_req2 && (c1_len2 != 0) && !mo_2nd_stale;
wire pf_wants_head = !h_pf && c2_req  && (pf_left != 0) && !pf_head_stale;
wire pf_wants_2nd  =  h_pf && !s_pf && c2_req2 && (c2_len2 != 0) && !pf_2nd_stale;

wire mo_wants = mo_wants_head || mo_wants_2nd;
wire pf_wants = pf_wants_head || pf_wants_2nd;

// The controller can take another burst, nothing is already waiting in it, and the CPU is
// not asking. The CPU check keeps the latency-critical client from finding a burst queued
// ahead of it on the single-word path.
wire can_hand = sd_brst_accept && (queued == OWN_NONE) && !c0_req;

// Abort ONLY when nothing is buffered -- see the ordering rule in the header.
wire nothing_buffered = (queued == OWN_NONE);
wire preempt_mo = (owner == OWN_MO) && nothing_buffered && c0_req;
wire preempt_pf = (owner == OWN_PF) && nothing_buffered && (c0_req || c1_req);
assign sd_brst_abort = preempt_mo || preempt_pf;

// Delivered count for the running head, including a word landing this cycle. Burst words
// only -- counting `sd_valid` here would let foreign single-word traffic retire a burst early.
wire [8:0] mo_dlv_now = d_mo + ((sd_brst_valid && owner == OWN_MO) ? 9'd1 : 9'd0);
wire [8:0] pf_dlv_now = d_pf + ((sd_brst_valid && owner == OWN_PF) ? 9'd1 : 9'd0);

always_ff @(posedge clk) begin
	sd_req      <= 1'b0;
	sd_brst_req <= 1'b0;
	c1_done     <= 1'b0;
	c2_done     <= 1'b0;

	if (reset) begin
		owner    <= OWN_NONE;
		queued   <= OWN_NONE;
		d_mo     <= '0;
		d_pf     <= '0;
		h_c0     <= 1'b0; hd_c0 <= 1'b0;
		h_mo     <= 1'b0; s_mo <= 1'b0;
		h_pf     <= 1'b0; s_pf <= 1'b0;
		hd_mo    <= 1'b0; sd_mo <= 1'b0;
		hd_pf    <= 1'b0; sd_pf <= 1'b0;
	end else begin
		// Release the head-consumed interlock as soon as the client offers something else,
		// or drops the request entirely.
		//
		// This MUST come before the retire block below, not after it. Written afterwards
		// it compares the client's new head against `hd_addr_*` BEFORE the retire block has
		// written it -- so on the very cycle the interlock is set, the stale comparison
		// clears it again and the later assignment wins. The head is then re-handed and the
		// client receives its previous request a second time ("expected 10 words, got 42",
		// 32 stale plus 10 real). Same-block assignment order is load-bearing here.
		if (!c0_req || (c0_addr != hd_addr_c0) || (c0_len != hd_len_c0)) hd_c0 <= 1'b0;
		if (!c1_req || (c1_addr != hd_addr_mo) || (c1_len != hd_len_mo)) hd_mo <= 1'b0;
		if (!c2_req || (c2_addr != hd_addr_pf) || (c2_len != hd_len_pf)) hd_pf <= 1'b0;
		if (!c1_req2 || (c1_addr2 != sd_addr2_mo) || (c1_len2 != sd_len2_mo)) sd_mo <= 1'b0;
		if (!c2_req2 || (c2_addr2 != sd_addr2_pf) || (c2_len2 != sd_len2_pf)) sd_pf <= 1'b0;

		// Count words as they are delivered, so a preempted head knows where to resume.
		// sd_brst_valid, not sd_valid: only burst words count against a burst's length.
		if (sd_brst_valid && owner == OWN_MO) d_mo <= d_mo + 1'b1;
		if (sd_brst_valid && owner == OWN_PF) d_pf <= d_pf + 1'b1;

		// ---- a burst retires ----
		// The running burst is always the owner's HEAD: requests are handed in order and
		// the controller runs them in order, so the second cannot start before the head
		// has completed and been shifted up by the client.
		if (sd_brst_done) begin
			case (owner)
				// Never preempted, so a done here means every word was delivered.
				OWN_CPU: begin
					c0_done <= 1'b1;
					hd_c0   <= 1'b1;     // addr/len were latched at grant, below
					h_c0    <= 1'b0;
				end
				OWN_MO: begin
					if (mo_dlv_now >= c1_len) begin
						c1_done    <= 1'b1;      // head complete
						d_mo       <= '0;
						// The second is PROMOTED to head; the second slot empties. Marking
						// it stale stops the client's still-asserted req2 being taken as a
						// new second before it has shifted.
						h_mo       <= s_mo;
						s_mo       <= 1'b0;
						sd_mo      <= s_mo;
						hd_mo      <= 1'b1;      // do not re-hand what just finished
						hd_addr_mo <= c1_addr;
						hd_len_mo  <= c1_len;
					end else begin
						// Preempted partway: un-hand the head so it is re-granted and
						// resumes from c1_addr + d_mo. The second cannot be buffered here
						// -- the arbiter does not abort with anything queued behind.
						h_mo <= 1'b0;
					end
				end
				OWN_PF: begin
					if (pf_dlv_now >= c2_len) begin
						c2_done    <= 1'b1;
						d_pf       <= '0;
						h_pf       <= s_pf;
						s_pf       <= 1'b0;
						sd_pf      <= s_pf;
						hd_pf      <= 1'b1;
						hd_addr_pf <= c2_addr;
						hd_len_pf  <= c2_len;
					end else begin
						h_pf <= 1'b0;
					end
				end
				default: ;
			endcase
		end

		// ---- ownership handover ----
		// A burst BEGINNING is what transfers ownership of returning data; a retirement
		// with no start in the same cycle means nothing is running.
		if (sd_brst_start) begin
			owner  <= queued;
			queued <= OWN_NONE;
		end else if (sd_brst_done) begin
			owner <= OWN_NONE;
		end

		// ---- hand the next burst to the controller's buffer ----
		// Runs while the current burst is still draining; that is the whole point.
		if (can_hand) begin
			if (mo_wants) begin
				sd_brst_addr <= mo_wants_head ? mo_next : c1_addr2;
				sd_brst_len  <= mo_wants_head ? mo_left : c1_len2;
				sd_brst_req  <= 1'b1;
				queued       <= OWN_MO;
				if (mo_wants_head) h_mo <= 1'b1;
				else begin
					s_mo        <= 1'b1;
					sd_addr2_mo <= c1_addr2;
					sd_len2_mo  <= c1_len2;
				end
			end else if (pf_wants) begin
				sd_brst_addr <= pf_wants_head ? pf_next : c2_addr2;
				sd_brst_len  <= pf_wants_head ? pf_left : c2_len2;
				sd_brst_req  <= 1'b1;
				queued       <= OWN_PF;
				if (pf_wants_head) h_pf <= 1'b1;
				else begin
					s_pf        <= 1'b1;
					sd_addr2_pf <= c2_addr2;
					sd_len2_pf  <= c2_len2;
				end
			end
		end

		// ---- CPU burst path ----
		// Same guard the single-word version used -- fully idle controller, nothing queued --
		// so the CPU never sits behind a running burst and MO/PF two-deep timing is unchanged.
		// Mutually exclusive with the hand block above, which requires !c0_req.
		if (c0_req && !h_c0 && !c0_head_stale && (c0_len != 0) &&
		    sd_brst_accept && (owner == OWN_NONE) && (queued == OWN_NONE)) begin
			sd_brst_addr <= c0_addr;
			sd_brst_len  <= c0_len;
			sd_brst_req  <= 1'b1;
			queued       <= OWN_CPU;
			h_c0         <= 1'b1;
			hd_addr_c0   <= c0_addr;   // identity as HANDED
			hd_len_c0    <= c0_len;
		end
	end
end

endmodule
