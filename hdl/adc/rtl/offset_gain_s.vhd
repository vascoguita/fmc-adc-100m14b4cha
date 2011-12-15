--------------------------------------------------------------------------------
-- CERN (BE-CO-HT)
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
-- description: Offset and gain correction with saturation.
--              Latency = 2
--
--                           ___               ___           ________
--                          |   | offset_data |   | product |        |
--              data_i ---->| + |------------>| X |-------->|saturate|--> data_o
--                          |___|             |___|         |________|
--                            ^                 ^
--                            |                 |
--                         offset_i           gain_i
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
    data_i   : in  std_logic_vector(15 downto 0);  --! Signed data input (two's complement)
    data_o   : out std_logic_vector(15 downto 0)   --! Signed data output (two's complement)
    );
end entity offset_gain_s;


------------------------------------------------------------------------------
-- Architecture declaration
------------------------------------------------------------------------------
architecture rtl of offset_gain_s is

  ------------------------------------------------------------------------------
  -- Signals declaration
  ------------------------------------------------------------------------------
  signal rst         : std_logic                     := '0';
  signal data_in_d   : std_logic_vector(15 downto 0) := (others => '0');
  signal data_offset : std_logic_vector(17 downto 0) := (others => '0');
  signal gain        : std_logic_vector(17 downto 0) := (others => '0');
  signal product     : std_logic_vector(35 downto 0) := (others => '0');


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
        data_in_d   <= (others => '0');
      else
        data_in_d <= data_i;
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
      P   => product,                   -- Multiplier ouput, WIDTH_A+WIDTH_B
      A   => gain,                      -- Multiplier input A, WIDTH_A
      B   => data_offset,               -- Multiplier input B, WIDTH_B
      CE  => '1',                       -- 1-bit active high input clock enable
      CLK => clk_i,                     -- 1-bit positive edge clock input
      RST => rst                        -- 1-bit input active high reset
      );


  ------------------------------------------------------------------------------
  -- Saturate addition and multiplication result
  ------------------------------------------------------------------------------
  p_saturate : process (clk_i)
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        data_o <= (others => '0');
      else
        if (data_offset(15) = '1' and data_offset(16) = '0') then
          data_o <= X"7FFF";            -- saturate positive
        elsif (data_offset(15) = '0' and data_offset(16) = '1') then
          data_o <= X"8000";            -- saturate negative
        else
          data_o <= product(30 downto 15);
        end if;
      end if;
    end if;
  end process p_saturate;


end architecture rtl;