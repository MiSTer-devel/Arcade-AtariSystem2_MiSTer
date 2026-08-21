--
-- Copyright (c) MikeJ - May 2004
--
-- All rights reserved
--
-- Redistribution and use in source and synthezised forms, with or without
-- modification, are permitted provided that the following conditions are met:
--
-- Redistributions of source code must retain the above copyright notice,
-- this list of conditions and the following disclaimer.
--
-- Redistributions in synthesized form must reproduce the above copyright
-- notice, this list of conditions and the following disclaimer in the
-- documentation and/or other materials provided with the distribution.
--
-- Neither the name of the author nor the names of other contributors may
-- be used to endorse or promote products derived from this software without
-- specific prior written permission.
--
-- THIS CODE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
-- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
-- THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
-- PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
-- LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
-- CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
-- SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
-- INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
-- CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
-- ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
-- POSSIBILITY OF SUCH DAMAGE.
--
-- You are responsible for any legal issues arising from your use of this code.
--
-- The latest version of this file can be found at: www.fpgaarcade.com
--
-- Email support@fpgaarcade.com
--
-- Revision list
--
-- version 002 return 00 on allpot when fast scan completed to fix self test
-- version 001 initial release (this version should be considered Beta
--   it seems to make all the right sort of sounds however ... )
--
library ieee;
	use ieee.std_logic_1164.all;
	use ieee.std_logic_unsigned.all;
	use ieee.numeric_std.all;

-- There is deliberately NO high-pass propagation-delay generic here. An earlier revision
-- carried one (a 1.5-POKEY-clock delay) to explain a "narrow half-clock pulse instead of
-- clean cancellation" symptom, but that symptom was an artefact of the capture bug fixed
-- below: the capture FF took a 1-clock pulse instead of the channel state, so it sat at 0
-- essentially always and nothing could cancel. With the capture source corrected the
-- premise is gone, and both MAME's pokey.cpp and Videodr0me's Major Havoc core model zero
-- delay here. Reinstating a delay would need fresh evidence.
entity POKEY is
port (
	ADDR      : in  std_logic_vector(3 downto 0);
	DIN       : in  std_logic_vector(7 downto 0);
	DOUT      : out std_logic_vector(7 downto 0);
	DOUT_OE_L : out std_logic;
	RW_L      : in  std_logic;
	CS        : in  std_logic; -- used as enable
	CS_L      : in  std_logic;
	--
	AUDIO_OUT : out signed(5 downto 0);
	--
	PIN       : in  std_logic_vector(7 downto 0);
	ENA       : in  std_logic;
	CLK       : in  std_logic  -- note 6 Mhz
);
end;

architecture RTL of POKEY is
	type  array_8x8   is array (0 to 7) of std_logic_vector(7 downto 0);
	type  array_4x8   is array (1 to 4) of std_logic_vector(7 downto 0);
	type  array_4x4   is array (1 to 4) of std_logic_vector(3 downto 0);
	type  array_4x9   is array (1 to 4) of std_logic_vector(8 downto 0);
	type  array_2x17  is array (1 to 2) of std_logic_vector(16 downto 0);
	type  bool_4      is array (1 to 4) of boolean;

	signal we                   : std_logic;
	signal oe                   : std_logic;
	--
	signal ena_64k_15k          : std_logic;
	signal cnt_64k              : std_logic_vector(4 downto 0) := (others => '0');
	signal ena_64k              : std_logic;
	signal cnt_15k              : std_logic_vector(6 downto 0) := (others => '0');
	signal ena_15k              : std_logic;
	--
	signal poly4                : std_logic_vector(3 downto 0)  := "0001";              -- hardware sequence start
	signal poly5                : std_logic_vector(4 downto 0)  := "00001";             -- hardware sequence start
	signal poly9                : std_logic_vector(8 downto 0)  := "011111111";         -- XOR feedback: MUST be non-zero
	signal poly17               : std_logic_vector(16 downto 0) := "11111111101111111"; -- XOR feedback: MUST be non-zero
	signal poly_17_9            : std_logic;

	-- registers
	signal audf                 : array_4x8 := (others => (others => '0'));
	signal audc                 : array_4x8 := (others => (others => '0'));
	signal audctl               : std_logic_vector(7 downto 0) := (others => '0');
	signal stimer               : std_logic_vector(7 downto 0);
	signal skres                : std_logic_vector(7 downto 0);
	signal potgo                : std_logic;
	signal serout               : std_logic_vector(7 downto 0);
	signal irqen                : std_logic_vector(7 downto 0);
	signal skctls               : std_logic_vector(7 downto 0);
	signal reset                : std_logic;
	--
	signal kbcode               : std_logic_vector(7 downto 0);
	signal random               : std_logic_vector(7 downto 0);
	signal serin                : std_logic_vector(7 downto 0);
	signal irqst                : std_logic_vector(7 downto 0);
	signal skstat               : std_logic_vector(7 downto 0);
	--
	signal pot_fin              : std_logic;
	signal pot_cnt              : std_logic_vector(7 downto 0);
	signal pot_val              : array_8x8;
	signal pin_reg              : std_logic_vector(7 downto 0);
	signal pin_reg_gated        : std_logic_vector(7 downto 0);
	--
	signal chan_ena             : std_logic_vector(4 downto 1);
	signal tone_gen_div         : std_logic_vector(4 downto 1);
	signal tone_gen_cnt         : array_4x8 := (others => (others => '0'));
	signal tone_gen_div_mux     : std_logic_vector(4 downto 1);
	signal tone_gen_zero        : std_logic_vector(4 downto 1);
	signal tone_gen_zero_t      : array_4x8 := (others => (others => '0'));
	signal chan_done_load       : std_logic_vector(4 downto 1) := (others => '0');
	--
	-- Channel output flip-flop, split the way the real chip splits it (see p_poly_gating):
	--   audio_clock  = the CLOCK into the FF   (timer pulse, optionally gated by poly5)
	--   audio_sample = the DATA into the FF    (poly4, or the 17/9-bit poly)
	signal audio_clock          : std_logic_vector(4 downto 1);
	signal audio_sample         : std_logic_vector(4 downto 1);
	signal tone_gen_final       : std_logic_vector(4 downto 1) := (others => '0');
	-- High-pass capture FFs (channels 1 and 2 only -- 3 and 4 have no high-pass).
	signal hp_capture           : std_logic_vector(2 downto 1) := (others => '0');
	-- With the high-pass DISABLED, does the capture FF hold 1 (channels 1/2 come out INVERTED -- what
	-- MAME `pokey.cpp` and Videodr0me's Major Havoc core both do) or 0 (pass-through)? Left at
	-- the reference behaviour; see p_audio_out. Kept as a named constant because the choice is
	-- subtle. Measured: toggling it changes NOTHING about POKEY's DC on a continuously clocked
	-- channel (inverting is then a pure phase flip); it could only matter for a channel PARKED
	-- with non-zero volume.
	constant POKEY_HP_INVERT_WHEN_OFF : boolean := true;
	-- Post-high-pass channel state -- this, NOT tone_gen_final, is what the mixer sees.
	signal channel_output       : std_logic_vector(4 downto 1);
	--
	-- ---------------------------------------------------------------------------
	-- REAL POKEY OUTPUT MODEL (rather than a plain linear sum).
	--
	-- The four channel DACs do NOT contribute equal binary steps, and the SUMMED
	-- analog output compresses. Measured contributions per volume bit (Altirra
	-- Hardware Reference Manual): bit0 0.12 V, bit1 0.26 V, bit2 0.56 V, bit3
	-- 1.12 V -> integer weights 12/26/56/112, so one channel spans 0..206 and the
	-- four-channel sum spans 0..824. Note the steps are UNEQUAL: 3->4, 7->8 and
	-- 11->12 jump +18 where the others are +12/+14, which a 6-bit linear sum of
	-- the raw volumes cannot represent.
	--
	-- The compression is applied to the SUM, not per channel:
	--     x = weighted_sum / 824
	--     y = 2.171*x                                              for x <= 0.14
	--     y = 2.171*(0.14 + (1-exp(-2.85*(x-0.14)))/2.85)          for x >= 0.14
	-- Consequence: two channels at volume 15 give only ~1.56x one channel, not 2x.
	--
	-- NLMIX is that curve baked to 825 entries scaled to the ORIGINAL 0..60 output
	-- span, so the external contract is unchanged: silence still reads -32 and
	-- full scale still reads +28 (see the AUDIO_OUT assignment below). Only the
	-- interior mapping changes, so the top-level mixer scaling stays valid.
	-- ---------------------------------------------------------------------------
	-- WHY POKEY_DAC_SATURATE IS OFF FOR THIS BOARD. Altirra's compression curve was
	-- measured on Atari 8-bit computers, where the AUD pin drives a RESISTIVE load and the pin
	-- voltage swings. On Paperboy the AUD pin feeds a TRANSIMPEDANCE stage (SP-275 sheet 9B: LM324
	-- 10C inverting input held at +5AUD, R128 2.2K in feedback), so the pin sits at a VIRTUAL GROUND
	-- and barely swings -- the output is a current converted by R128. That splits the model in two:
	--   * the UNEQUAL BIT WEIGHTS are a property of the current DAC itself   -> apply on any board
	--   * the SATURATION is a property of the output stage driving a load    -> does NOT
	--     apply into a virtual ground
	-- With saturation ON, one voice at volume 15 would read 31 counts above silence instead of
	-- 15 (+6.3 dB), and Paperboy effects are 1-2 voices. With it OFF the weights alone map one
	-- voice at volume 15 back to 15,
	-- because 12/26/56/112 are nearly proportional to 1/2/4/8; only the small step non-uniformity
	-- (3->4, 7->8, 11->12) survives, which is the part that is definitely a DAC property.
	-- The load dependence is confirmed: Altirra's curve was tuned on an
	-- NTSC 800XL whose AUD pin sits under a 1K pull-up to +5V -- a node whose voltage falls as more
	-- DAC devices turn on -- and its per-bit drops across that 1K are equivalently 0.12/0.26/0.56/
	-- 1.12 mA, matching Atari's patent describing the amplitude DAC as weighted MOS pull-down
	-- CURRENT sources. Paperboy holds AUD at a fixed +5V virtual ground, so the mechanism is absent.
	-- It stays OFF; see POKEY_LM324_CLIP for the limit this board DOES have.
	constant POKEY_DAC_SATURATE : boolean := false;
	-- Weighted-sum value at which the external LM324 preamp runs out of output swing (0 disables).
	-- Derived in the mixer process below from SP-275 sheet 9B: +-15V rails (pins 4/11 of the LM324
	-- at 10C, read off the sheet), +5AUD reference, 2.2K feedback, 0.01 mA per weight unit.
	-- 386 uses the typical V+ - 1.5V limit; a more conservative V+ - 1.75V would give 375.
	constant POKEY_LM324_CLIP : integer := 386;
	type wgt_lut_t is array (0 to 15) of integer range 0 to 206;
	constant WGT : wgt_lut_t := (0, 12, 26, 38, 56, 68, 82, 94, 112, 124, 138, 150, 168, 180, 194, 206);
	type nlmix_lut_t is array (0 to 824) of integer range 0 to 60;
	constant NLMIX : nlmix_lut_t := (
		 0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  3,  3,  3,  3,
		 3,  3,  3,  4,  4,  4,  4,  4,  4,  5,  5,  5,  5,  5,  5,  6,  6,  6,  6,  6,
		 6,  6,  7,  7,  7,  7,  7,  7,  8,  8,  8,  8,  8,  8,  9,  9,  9,  9,  9,  9,
		 9, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12,
		13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 16,
		16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 18, 19, 19,
		19, 19, 19, 19, 20, 20, 20, 20, 20, 20, 20, 21, 21, 21, 21, 21, 21, 22, 22, 22,
		22, 22, 22, 22, 23, 23, 23, 23, 23, 23, 23, 24, 24, 24, 24, 24, 24, 24, 25, 25,
		25, 25, 25, 25, 25, 25, 26, 26, 26, 26, 26, 26, 26, 26, 27, 27, 27, 27, 27, 27,
		27, 28, 28, 28, 28, 28, 28, 28, 28, 29, 29, 29, 29, 29, 29, 29, 29, 29, 30, 30,
		30, 30, 30, 30, 30, 30, 31, 31, 31, 31, 31, 31, 31, 31, 31, 32, 32, 32, 32, 32,
		32, 32, 32, 32, 33, 33, 33, 33, 33, 33, 33, 33, 33, 34, 34, 34, 34, 34, 34, 34,
		34, 34, 34, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 36, 36, 36, 36, 36, 36, 36,
		36, 36, 36, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 38, 38, 38, 38, 38, 38,
		38, 38, 38, 38, 38, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 40, 40, 40,
		40, 40, 40, 40, 40, 40, 40, 40, 40, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41,
		41, 41, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 43, 43, 43, 43, 43,
		43, 43, 43, 43, 43, 43, 43, 43, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44,
		44, 44, 44, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 46, 46,
		46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 47, 47, 47, 47, 47, 47,
		47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 48, 48, 48, 48, 48, 48, 48, 48, 48,
		48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49,
		49, 49, 49, 49, 49, 49, 49, 49, 49, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50,
		50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51,
		51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 52, 52, 52, 52, 52, 52, 52, 52,
		52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 53, 53, 53, 53,
		53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53,
		53, 53, 53, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54,
		54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 55, 55, 55, 55, 55, 55, 55, 55,
		55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55,
		55, 55, 55, 55, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56,
		56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56, 56,
		56, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57,
		57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57,
		57, 57, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58,
		58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58,
		58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 59, 59, 59, 59, 59, 59, 59, 59, 59,
		59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59,
		59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59,
		59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60,
		60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60,
		60, 60, 60, 60, 60);
	--
	-- The shared polynomial generators reach the four audio
	-- channels PHASE-STAGGERED -- channel 1 sees the current bit, channels 2/3/4
	-- see it delayed by 1/2/3 POKEY clocks. Modelling all four from the same bit
	-- on the same cycle is subtly wrong, so keep 4 stages of history per polynomial.
	signal poly4_hist           : std_logic_vector(3 downto 0) := (others => '0');
	signal poly5_hist           : std_logic_vector(3 downto 0) := (others => '0');
	signal poly917_hist         : std_logic_vector(3 downto 0) := (others => '0');
begin

	p_we : process(RW_L, CS_L, CS, ENA)
	begin
		we <= (not CS_L) and CS and (not RW_L) and ENA;
	end process;

	p_oe : process(RW_L, CS_L, CS)
	begin
		oe <= (not CS_L) and CS and RW_L;
	end process;
	DOUT_OE_L <= not oe;

	p_ipreg : process
	begin
		wait until rising_edge(CLK);
		pin_reg <= PIN;
	end process;

	p_dividers : process
	begin
		wait until rising_edge(CLK);
		if (ENA = '1') then
			ena_64k <= '0';
			if cnt_64k = "00000" then
				cnt_64k <= "11011"; -- 28 - 1
				ena_64k <= '1';
			else
				cnt_64k <= cnt_64k - "1";
			end if;

			ena_15k <= '0';
			if cnt_15k = "0000000" then
				cnt_15k <= "1110001"; -- 114 - 1
				ena_15k <= '1';
			else
				cnt_15k <= cnt_15k - "1";
			end if;
		end if;
	end process;

	p_ena_64k_15k : process(ena_64k, ena_15k, audctl)
	begin
		if (audctl(0) = '1') then
			ena_64k_15k <= ena_15k;
		else
			ena_64k_15k <= ena_64k;
		end if;
	end process;

	-- POLYNOMIAL GENERATORS -- three corrections vs the original file.
	--
	-- Found by comparing against Videodr0me's Arcade-Tempest_MiSTer `rtl/pokey.vhd` (GPL-2.0), which
	-- is the SAME MikeJ/FPGAArcade lineage as this file but carries fixes this copy never got. All
	-- three feed the AUDC distortion selects, so they affect every noise-based effect -- which is
	-- most of Paperboy's SFX.
	--
	--  1. poly5 was CROSS-COUPLED TO poly4: `not (poly5(4) xor poly4(2))` tapped bit 2 of a
	--     DIFFERENT register, so it was not a 5-bit LFSR at all and had no maximal-length period.
	--     Unambiguous bug under any bit-order convention. Now `poly5(2)`.
	--  2. poly9 was wrong and the original file said so -- there was a literal `-- not correct` comment
	--     above it. Replaced with the hardware sequence (shift right, taps 0 and 5).
	--  3. poly17 used a plain left shift with taps 16/2; the real chip has a SPLIT path where the
	--     feedback lands in the middle of the register (bit 7 = bit 8 xor bit 13), not at the end.
	--
	-- NOTE ON BIT ORDER: poly9/poly17 now shift RIGHT where they used to shift left, so the output
	-- taps moved with them -- see p_random_mux below. Getting only half of that right would just
	-- have traded one wrong sequence for another. poly4/poly5 keep their existing top-bit output
	-- taps: the shift expressions are identical to Tempest's, only the tapped bit differs, which is
	-- a 3-clock phase offset -- and per-channel phase is modelled separately by poly*_hist.
	--
	-- SEEDS: poly9/poly17 use XOR feedback, for which all-zeros is a lock-up state (poly4/poly5 use
	-- XNOR and escape it). They are therefore seeded to the first entries of the hardware sequences,
	-- both at declaration and on `reset` (SKCTL[1:0]="00", the LFSR-init the CPU pulses -- the real
	-- chip resets its polynomial counters there too). The zero-guards are kept as cheap insurance in
	-- case a toolchain drops the declaration initialiser; with a non-zero seed they never fire.
	p_poly : process
		variable poly9_zero : std_logic;
		variable poly17_zero : std_logic;
	begin
		wait until rising_edge(CLK);
		if (ENA = '1') then
			if (reset = '1') then
				poly4  <= "0001";
				poly5  <= "00001";
				poly9  <= "011111111";
				poly17 <= "11111111101111111";
			else
				poly4 <= poly4(2 downto 0) & not (poly4(3) xor poly4(2));
				poly5 <= poly5(3 downto 0) & not (poly5(4) xor poly5(2));

				poly9_zero := '0';
				if (poly9 = "000000000") then poly9_zero := '1'; end if;
				poly9 <= (poly9(0) xor poly9(5) xor poly9_zero) & poly9(8 downto 1);

				poly17_zero := '0';
				if (poly17 = "00000000000000000") then poly17_zero := '1'; end if;
				poly17(16)          <= poly17(0);
				poly17(15 downto 8) <= poly17(16 downto 9);
				poly17(7)           <= poly17(8) xor poly17(13) xor poly17_zero;
				poly17(6 downto 0)  <= poly17(7 downto 1);
			end if;
		end if;
	end process;

	-- Output taps MUST match the shift direction chosen in p_poly. poly9/poly17 now shift right, so
	-- the readable window and the noise tap come off the LOW end, exactly as in the Tempest core.
	-- (Paperboy's sound ROM never reads RANDOM -- the only reference to $180a in the whole ROM is a
	-- `dec $180a` at $ef4c, which is data disassembled as code -- so the readable-window change is
	-- software-invisible here. It still has to be consistent, because poly_17_9 drives the noise.)
	p_random_mux : process(audctl, poly9, poly17)
	begin
		if (audctl(7) = '1') then          -- 9-bit poly selected
			random    <= poly9(7 downto 0);
			poly_17_9 <= poly9(0);
		else
			random    <= poly17(15 downto 8);
			poly_17_9 <= poly17(0);
		end if;
	end process;

	p_wdata : process
	begin
		wait until rising_edge(CLK);
		potgo <= '0';

		--if (reset = '1') then
			-- no idea what the reset state is
			--audf <= (others => (others => '0'));
			--audc <= (others => (others => '0'));
			--audctl <= x"00";
		--else
		if (we = '1') then
			case ADDR is
				when x"0" => audf(1)  <= DIN;
				when x"1" => audc(1)  <= DIN;
				when x"2" => audf(2)  <= DIN;
				when x"3" => audc(2)  <= DIN;
				when x"4" => audf(3)  <= DIN;
				when x"5" => audc(3)  <= DIN;
				when x"6" => audf(4)  <= DIN;
				when x"7" => audc(4)  <= DIN;
				when x"8" => audctl   <= DIN;
				when x"9" => stimer   <= DIN;
				when x"A" => skres    <= DIN;
				when x"B" => potgo    <= '1';
				--when x"C" =>
				when x"D" => serout   <= DIN;
				when x"E" => irqen    <= DIN;
				when x"F" => skctls   <= DIN;
				when others => null;
			end case;
		end if;
		--end if;
	end process;

	p_reset : process(skctls)
	begin
		-- chip in reset if bits 1..0 of skctls are both zero
		reset <= '0';
		if (skctls(1 downto 0) = "00") then
			reset <= '1';
		end if;
	end process;

	p_rdata : process(oe, ADDR, pot_val, pin_reg_gated, kbcode, random, serin, irqst, skstat)
	begin
		DOUT <= x"00";
		if (oe = '1') then -- keep things quiet
			case ADDR IS
				when x"0" => DOUT <= pot_val(0);   -- pot 0
				when x"1" => DOUT <= pot_val(1);   -- pot 1
				when x"2" => DOUT <= pot_val(2);   -- pot 2
				when x"3" => DOUT <= pot_val(3);   -- pot 3
				when x"4" => DOUT <= pot_val(4);   -- pot 4
				when x"5" => DOUT <= pot_val(5);   -- pot 5
				when x"6" => DOUT <= pot_val(6);   -- pot 6
				when x"7" => DOUT <= pot_val(7);   -- pot 7
				when x"8" => DOUT <= pin_reg_gated;-- allpot
				when x"9" => DOUT <= kbcode;
				when x"A" => DOUT <= random;
				when x"B" => DOUT <= x"FF";
				when x"C" => DOUT <= x"FF";
				when x"D" => DOUT <= serin;
				when x"E" => DOUT <= irqst;
				when x"F" => DOUT <= skstat;
				when others => null;
			end case;
		end if;
	end process;

	-- POT ANALOGUE IN UNTESTED !!
	p_pot_cnt : process
	begin
		wait until rising_edge(CLK);
		if (potgo = '1') then
			pot_cnt <= x"00";
		elsif ((ena_15k = '1') or (skctls(2) = '1')) and (ENA = '1') then -- fast scan mode
			pot_cnt <= pot_cnt + "1";
		end if;
	end process;

	p_pot_comp : process
	begin
		wait until rising_edge(CLK);
		if (reset = '1') then
			pot_fin <= '1';
		else
			if (potgo = '1') then
				pot_fin <= '0';
			elsif (pot_cnt = x"E4") then -- 228
				pot_fin <= '1';
			end if;
		end if;
	end process;

	p_pot_val : process
	begin
		wait until rising_edge(CLK);
		for i in 0 to 7 loop
			if (pot_fin = '0') and (pin_reg(i) = '0') then
				-- continue latching counter value until input reaches ViH threshold
				pot_val(i) <= pot_cnt;
			end if;
		end loop;
	end process;

	-- dump transistors
	--PIN <= x"00" when (pot_fin = '1') else (others => 'Z');
	p_in_gate : process(pin_reg, reset, pot_fin) -- dump transistor fakeup
	begin
		pin_reg_gated <= pin_reg;
		-- I think the datasheet lies about dump transistors being disabled
		-- in fast scan mode, as the self test fails ....
		if (reset = '1') or (pot_fin = '1') then --and (skctls(2) = '0'))
			pin_reg_gated <= x"00";
		end if;
	end process;

	p_tone_cnt_ena : process(audctl, ena_64k_15k, tone_gen_div)
		variable chan_ena1, chan_ena3 : std_ulogic;
	begin

		if (audctl(6) = '1') then
			chan_ena1 := '1'; -- 1.5 MHz,
		else
			chan_ena1 := ena_64k_15k;
		end if;
		chan_ena(1) <= chan_ena1;

		if (audctl(4) = '1') then -- chan 1/2 joined
			chan_ena(2) <= chan_ena1;
		else
			chan_ena(2) <= ena_64k_15k;
		end if;

		if (audctl(5) = '1') then
			chan_ena3 := '1'; -- 1.5 MHz,
		else
			chan_ena3 := ena_64k_15k; -- 64 KHz
		end if;
		chan_ena(3) <= chan_ena3;

		if (audctl(3) = '1') then -- chan 3/4 joined
			chan_ena(4) <= chan_ena3;
		else
			chan_ena(4) <= ena_64k_15k; -- 64 KHz
		end if;
	end process;

	p_tone_generator_zero : process(tone_gen_cnt, chan_ena)
	begin
		for i in 1 to 4 loop
			if (tone_gen_cnt(i) = "00000000") and (chan_ena(i) = '1') then
				tone_gen_zero(i) <= '1';
			else
				tone_gen_zero(i) <= '0';
			end if;
		end loop;
	end process;

	p_tone_generators : process
		variable chan_load : std_logic_vector(4 downto 1);
		variable chan_dec : std_logic_vector(4 downto 1);
	begin
		-- quite tricky this .. but I think it does the correct stuff
		-- bet this is not how is was done originally !
		--
		-- nasty frig to easily get exact chip behaviour in high speed mode
		-- fout = fin / 2(audf + n) when n=4 or 7 in 16 bit mode
		wait until rising_edge(CLK);
		if (ENA = '1') then
			tone_gen_div <= "0000";

			if (audctl(4) = '1') then -- chan 1/2 joined
				chan_load(1) := '0';
				chan_load(2) := '0';
				if (tone_gen_zero_t(1)(5) = '1') and (tone_gen_zero_t(2)(5) = '1') and (chan_done_load(1) = '0') then
					chan_load(1) := '1';
					chan_load(2) := '1';
				end if;
				chan_dec(1) := '1';
				chan_dec(2) := tone_gen_zero(1);
			else
				chan_load(1) := tone_gen_zero_t(1)(2) and not chan_done_load(1);
				chan_load(2) := tone_gen_zero_t(2)(2) and not chan_done_load(2);

				chan_dec(1) := '1';
				chan_dec(2) := '1';
			end if;

			if (audctl(3) = '1') then -- chan 1/2 joined
				chan_load(3) := '0';
				chan_load(4) := '0';
				if (tone_gen_zero_t(3)(5) = '1') and (tone_gen_zero_t(4)(5) = '1') and (chan_done_load(3) = '0') then
					chan_load(3) := '1';
					chan_load(4) := '1';
				end if;
				chan_dec(3) := '1';
				chan_dec(4) := tone_gen_zero(3);
			else
				chan_load(3) := tone_gen_zero_t(3)(2) and not chan_done_load(3);
				chan_load(4) := tone_gen_zero_t(4)(2) and not chan_done_load(4);

				chan_dec(3) := '1';
				chan_dec(4) := '1';
			end if;

			for i in 1 to 4 loop

				if (chan_load(i) = '1') then
					chan_done_load(i) <= '1';
					tone_gen_div(i) <= '1';
					tone_gen_cnt(i) <= audf(i);
				elsif (chan_dec(i) = '1') and (chan_ena(i) = '1') then
					chan_done_load(i) <= '0';
					tone_gen_cnt(i) <= tone_gen_cnt(i) - "1";
				end if;

				tone_gen_div(i) <= chan_load(i);
				tone_gen_zero_t(i)(7 downto 0) <= tone_gen_zero_t(i)(6 downto 0) & tone_gen_zero(i);
			end loop;

		end if;
	end process;

	p_tone_generator_mux : process(audctl, tone_gen_div)
	begin
		if (audctl(4) = '1') then -- chan 1/2 joined
			tone_gen_div_mux(1) <= tone_gen_div(1); -- do they both waggle
			tone_gen_div_mux(2) <= tone_gen_div(2); -- or do I mute chan 1?
		else
			tone_gen_div_mux(1) <= tone_gen_div(1);
			tone_gen_div_mux(2) <= tone_gen_div(2);
		end if;

		if (audctl(3) = '1') then -- chan 3/4 joined
			tone_gen_div_mux(3) <= tone_gen_div(3); -- ditto
			tone_gen_div_mux(4) <= tone_gen_div(4);
		else
			tone_gen_div_mux(3) <= tone_gen_div(3);
			tone_gen_div_mux(4) <= tone_gen_div(4);
		end if;
	end process;

	-- Keep 4 stages of each polynomial so channel N can see the
	-- bit as it arrived N-1 POKEY clocks ago (channel 1 leads, channel 4 lags by 3).
	p_poly_hist : process
	begin
		wait until rising_edge(CLK);
		if (ENA = '1') then
			poly4_hist   <= poly4_hist(2 downto 0)   & poly4(3);
			poly5_hist   <= poly5_hist(2 downto 0)   & poly5(4);
			poly917_hist <= poly917_hist(2 downto 0) & poly_17_9;
		end if;
	end process;

	-- CHANNEL OUTPUT FLIP-FLOP -- topology corrected vs the original file.
	--
	-- The chip does NOT gate the polynomial into the timer pulse and toggle on the result. Each
	-- channel has ONE flip-flop, CLOCKED by the timer pulse (optionally gated by poly5) whose D
	-- input is either /Q (pure tone) or the selected polynomial bit. MAME `pokey.cpp`:
	--
	--     if ((AUDC & NOTPOLY5) || (poly5 & 1)) {
	--         if      (AUDC & PURE)   output ^= 1;         -- toggle
	--         else if (AUDC & POLY4)  output = poly4  & 1; -- SAMPLE
	--         else if (AUDCTL & POLY9)output = poly9  & 1;
	--         else                    output = poly17 & 1;
	--     }
	--
	-- We used to AND the poly into the pulse and toggle on the rising edge, which makes the output
	-- the running PARITY of the polynomial rather than the polynomial itself -- a different, and
	-- uncorrelated, waveform. Paperboy sits on this path: AUDC1 = 0x02/0x05 means bit7=0 (poly5
	-- gates) and bit5=0 (distortion), i.e. exactly the "sample the 17-bit poly" case.
	-- Corroborated independently by MAME and by Videodr0me's Major Havoc core.
	--
	-- Measured on Paperboy's own mode, old vs new: the sequences agree only ~50% (uncorrelated) but
	-- transition counts land within 1% and band energy within +0.6 dB (20-200 Hz) / -1.4 dB
	-- (0.5-3 kHz). So this is a CORRECTNESS fix, not a large level or timbre change.
	p_poly_gating : process(audc, poly4_hist, poly5_hist, poly917_hist, tone_gen_div_mux)
		variable p5, p4, p917 : std_logic;
	begin
		for i in 1 to 4 loop
			-- stage 0 = current bit (channel 1) ... stage 3 = 3 clocks old (channel 4)
			p5   := poly5_hist(i - 1);
			p4   := poly4_hist(i - 1);
			p917 := poly917_hist(i - 1);

			-- CLOCK into the channel FF: the timer pulse, gated by poly5 unless AUDC bit7 says not to
			if (audc(i)(7) = '0') then
				audio_clock(i) <= p5 and tone_gen_div_mux(i);
			else
				audio_clock(i) <= tone_gen_div_mux(i);
			end if;

			-- DATA into the channel FF: 4-bit poly if AUDC bit6, else the 17/9-bit poly
			if (audc(i)(6) = '1') then
				audio_sample(i) <= p4;
			else
				audio_sample(i) <= p917;
			end if;
		end loop;
	end process;

	-- HIGH-PASS FILTER -- capture source and XOR placement, two errors an earlier revision had:
	--
	--  1. Capturing the wrong signal: the capture FF took `poly_sel`, which is a ONE-CLOCK PULSE
	--     (high 0.093% of clocks), where the chip captures the channel OUTPUT FF STATE. MAME:
	--     `m_channel[CHAN1].m_filter_sample = m_channel[CHAN1].m_output`. Simulated at ch2/ch4
	--     periods 263/519 our capture FF saw '1' on 1 of 2021 timer events (0.05%) against
	--     992/2021 (49.08%) for the chip -- i.e. THE FILTER WAS EFFECTIVELY INERT. Paperboy does
	--     use it: AUDCTL 0x6a and 0x52 both set bit1 (channel 2 high-pass).
	--
	--  2. XORing in the wrong place: the XOR belongs at the MIXER, on the channel state
	--     (`m_output ^ m_filter_sample`), not on the toggle-detector input. Applied to the pulse it
	--     did NOT invert channels 1/2 when the filter is disabled -- the behaviour the old comment
	--     claimed -- it merely shifted the toggle edge by one clock.
	--
	-- Still true and still modelled: the XOR is NEVER bypassed. With the filter off the capture FF
	-- is FORCED TO 1, so channels 1/2 come out inverted. Channels 3/4 have no filter (MAME seeds
	-- their m_filter_sample to 0, i.e. XOR with 0 = pass-through).
	p_high_pass_filters : process(tone_gen_final, hp_capture)
	begin
		channel_output <= tone_gen_final;
		channel_output(1) <= tone_gen_final(1) xor hp_capture(1);
		channel_output(2) <= tone_gen_final(2) xor hp_capture(2);
	end process;

	p_audio_out : process
	begin
		wait until rising_edge(CLK);
		if (ENA = '1') then
			-- Capture FFs: channel 1's state is latched by tone generator 3, channel 2's by 4.
			-- When the filter is DISABLED the chip holds the FF at 1 rather than bypassing the XOR
			-- (MAME: `else m_channel[CHAN1].m_filter_sample = 1`), so channels 1/2 come out inverted.
			-- Holding the register (rather than muxing a 1 in downstream) also matches the reference
			-- on the enable transition: the filter starts from 1, not from a stale live capture.
			if (audctl(2) = '0') then
				if POKEY_HP_INVERT_WHEN_OFF then hp_capture(1) <= '1';
				else                             hp_capture(1) <= '0'; end if;
			elsif (tone_gen_div(3) = '1') then
				hp_capture(1) <= tone_gen_final(1);
			end if;

			if (audctl(1) = '0') then
				if POKEY_HP_INVERT_WHEN_OFF then hp_capture(2) <= '1';
				else                             hp_capture(2) <= '0'; end if;
			elsif (tone_gen_div(4) = '1') then
				hp_capture(2) <= tone_gen_final(2);
			end if;

			-- Channel output FFs: clocked by audio_clock, D = /Q (pure tone) or the polynomial bit.
			for i in 1 to 4 loop
				if (audio_clock(i) = '1') then
					if (audc(i)(5) = '1') then
						tone_gen_final(i) <= not tone_gen_final(i);
					else
						tone_gen_final(i) <= audio_sample(i);
					end if;
				end if;
			end loop;
		end if;
	end process;

	p_op_mixer : process
		variable vol : array_4x4;
		variable ws    : integer range 0 to 824;   -- weighted 4-channel sum
		variable sum   : std_logic_vector(5 downto 0);
	begin
		wait until rising_edge(CLK);
		if (ENA = '1') then
			for i in 1 to 4 loop
				if (audc(i)(4) = '1') then -- vol only
					vol(i) := audc(i)(3 downto 0);
				else
					-- channel_output, NOT tone_gen_final -- the high-pass XOR belongs
					-- here at the mixer (MAME: `m_output ^ m_filter_sample`), not upstream.
					if (channel_output(i) = '1') then
						vol(i) := audc(i)(3 downto 0);
					else
						vol(i) := "0000";
					end if;
				end if;
			end loop;

			-- Weight each channel by the real DAC bit voltages
			-- and run the SUM through the measured compression curve (see NLMIX).
			ws := WGT(to_integer(unsigned(vol(1)))) + WGT(to_integer(unsigned(vol(2))))
			    + WGT(to_integer(unsigned(vol(3)))) + WGT(to_integer(unsigned(vol(4))));
			-- The BOARD's own clipping limit. POKEY sinks
			-- current into an LM324 held at +5AUD with 2.2K feedback (R125/R128), so the preamp
			-- output is 5V + I*2.2K -- and that op-amp cannot swing to its rail. SP-275 sheet 9B
			-- verified directly: the LM324 at 10C has pin 4 = +15V and pin 11 = -15V, so with a
			-- typical LM324 high limit of V+ - 1.5V = 13.5V the stage runs out of swing at
			-- (13.5 - 5)/2.2k = 3.86 mA. The DAC weights ARE currents (0.12/0.26/0.56/1.12 mA for
			-- the four volume bits, i.e. one weight unit = 0.01 mA), so that is a weighted sum of
			-- 386 out of the 824 full-scale -- clipping bites above ~47% of full scale.
			-- Note this is NOT Altirra's compression, which belongs to the Atari
			-- 800XL's 1K pull-up and must not be applied here; it is the external amplifier's
			-- limit, so it clamps rather than compresses, and the scale below is unchanged (the
			-- downstream 0x187a attenuator and 47K summing resistor are fixed, so re-normalising
			-- here would move the YM:POKEY balance confirmed on hardware).
			if POKEY_LM324_CLIP > 0 and ws > POKEY_LM324_CLIP then
				ws := POKEY_LM324_CLIP;
			end if;
			if POKEY_DAC_SATURATE then
				sum := std_logic_vector(to_unsigned(NLMIX(ws), sum'length));
			else
				-- Weights only, linearly scaled to the same 0..60 span (no output-stage compression).
				sum := std_logic_vector(to_unsigned((ws * 60) / 824, sum'length));
			end if;

			if (reset = '1') then
				-- Output SILENCE (-32 = the sum=0 offset-binary level), NOT 0. The internal `reset`
				-- is SKCTL[1:0]="00" (the LFSR-init the CPU pulses), not a power reset; forcing 0 here
				-- is 32 steps from running-silence (-32), so every voice re-init stepped AUDIO_OUT
				-- 0<->-32 = an audible POP in the mixer (Paperboy attract is silent yet popped). The
				-- real chip stays silent when AUDC=0 regardless of the LFSR reset.
				AUDIO_OUT <= to_signed(-32, AUDIO_OUT'length);
			else
				AUDIO_OUT <= signed((not sum(sum'left)) & sum(sum'left-1 downto 0) );
			end if;
		end if;
	end process;

	-- keyboard / serial etc to do
end architecture RTL;
