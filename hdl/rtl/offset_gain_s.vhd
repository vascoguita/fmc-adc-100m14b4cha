--------------------------------------------------------------------------------
-- SPDX-FileCopyrightText: 2020 CERN (home.cern)
-- SPDX-License-Identifier: CERN-OHL-W-2.0+
-- Offset and gain correction, signed data input and output (two's complement)
-- http://www.ohwr.org/projects/fmc-adc-100m14b4cha
--------------------------------------------------------------------------------
--
-- unit name: offset_gain_corr_s (offset_gain_corr_s.vhd)
--
-- author: Matthieu Cattin (matthieu.cattin@cern.ch)
--
-- date: 24-11-2011
--
-- version: 1.0
--
-- description: Offset and gain correction with configurable saturation.
--              Input and output are signed.
--              Latency = 2
--
--                           ___               ___           ________
--                          |   | offset_data |   | product |        |
--              data_i ---->| + |------------>| X |-------->|saturate|--> data_o
--                          |___|             |___|         |________|
--                            ^                 ^               ^
--                            |                 |               |
--                         offset_i           gain_i          sat_i
--
--
-- dependencies:
--
--------------------------------------------------------------------------------
-- last changes: see svn log.
--------------------------------------------------------------------------------
-- TODO: - 
--------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;


library UNISIM;
use UNISIM.vcomponents.all;

library UNIMACRO;
use UNIMACRO.vcomponents.all;


------------------------------------------------------------------------------
-- Entity declaration
------------------------------------------------------------------------------
entity offset_gain_s is
  port (
    rst_n_i  : in  std_logic;                      --! Reset (active low)
    clk_i    : in  std_logic;                      --! Clock
    offset_i : in  std_logic_vector(15 downto 0);  --! Signed offset input (two's complement)
    gain_i   : in  std_logic_vector(15 downto 0);  --! Unsigned gain input
    sat_i    : in  std_logic_vector(14 downto 0);  --! Unsigned saturation value input
    data_i   : in  std_logic_vector(15 downto 0);  --! Signed data input (two's complement)
    data_o   : out std_logic_vector(15 downto 0);   --! Signed data output (two's complement)
    product_o : out std_logic_vector(16 downto 0)  --! Signed intermediate output (two's complement)
    );
end entity offset_gain_s;


------------------------------------------------------------------------------
-- Architecture declaration
------------------------------------------------------------------------------
architecture rtl of offset_gain_s is

  ------------------------------------------------------------------------------
  -- Constants declaration
  ------------------------------------------------------------------------------
  constant c_one : signed(16 downto 0) := to_signed(1, 17);

  ------------------------------------------------------------------------------
  -- Signals declaration
  ------------------------------------------------------------------------------
  signal rst         : std_logic                     := '0';
  signal data_offset : std_logic_vector(17 downto 0) := (others => '0');
  signal gain        : std_logic_vector(17 downto 0) := (others => '0');
  signal product_t   : std_logic_vector(35 downto 0) := (others => '0');
  signal product     : std_logic_vector(16 downto 0);
  signal pos_sat     : signed(16 downto 0);
  signal neg_sat     : signed(16 downto 0);


begin


  ------------------------------------------------------------------------------
  -- Active high reset for MULT_MACRO
  ------------------------------------------------------------------------------
  rst <= not(rst_n_i);

  ------------------------------------------------------------------------------
  -- Add offset to input data
  ------------------------------------------------------------------------------
  p_offset : process (clk_i)
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        data_offset <= (others => '0');
        gain        <= (others => '0');
      else
        -- propagate sign for signed offset_i
        data_offset <= std_logic_vector(signed(data_i(15) & data_i(15) & data_i) +
                                        signed(offset_i(15) & offset_i(15) & offset_i));
        gain <= "00" & gain_i;
      end if;
    end if;
  end process p_offset;


  ------------------------------------------------------------------------------
  -- Multiple input data + offset by gain
  ------------------------------------------------------------------------------
  -- MULT_MACRO: Multiply Function implemented in a DSP48E
  -- Xilinx HDL Libraries Guide, version 12.4
  ------------------------------------------------------------------------------
  cmp_multiplier : MULT_MACRO
    generic map (
      DEVICE  => "SPARTAN6",            -- Target Device: "VIRTEX5", "VIRTEX6", "SPARTAN6"
      LATENCY => 0,                     -- Desired clock cycle latency, 0-4
      WIDTH_A => 18,                    -- Multiplier A-input bus width, 1-25
      WIDTH_B => 18)                    -- Multiplier B-input bus width, 1-18
    port map (
      P   => product_t,                 -- Multiplier ouput, WIDTH_A+WIDTH_B
      A   => gain,                      -- Multiplier input A, WIDTH_A
      B   => data_offset,               -- Multiplier input B, WIDTH_B
      CE  => '1',                       -- 1-bit active high input clock enable
      CLK => clk_i,                     -- 1-bit positive edge clock input
      RST => rst                        -- 1-bit input active high reset
      );

  -- Additional register stage to solve timing issues
  p_pipeline : process (clk_i)
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        product <= (others => '0');
      else
        product <= product_t(31 downto 15);
      end if;
    end if;
  end process p_pipeline;

  product_o <= product;

  ------------------------------------------------------------------------------
  -- Saturate addition and multiplication result
  ------------------------------------------------------------------------------
  pos_sat <= signed("00" & sat_i);
  neg_sat <= signed(not(pos_sat))+c_one;

  p_saturate : process (clk_i)
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        data_o <= (others => '0');
      elsif signed(product) >= pos_sat then
        data_o <= std_logic_vector(pos_sat(15 downto 0));  -- saturate positive
      elsif signed(product) <= neg_sat then
        data_o <= std_logic_vector(neg_sat(15 downto 0));  -- saturate negative
      else
        data_o <= product(15 downto 0);
      end if;
    end if;
  end process p_saturate;


end architecture rtl;
