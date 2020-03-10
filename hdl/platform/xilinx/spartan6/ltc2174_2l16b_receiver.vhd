--------------------------------------------------------------------------------
-- CERN BE-CO-HT
-- FMC ADC 100M 14B 4CHA gateware
-- http://www.ohwr.org/projects/fmc-adc-100m14b4cha-gw
--------------------------------------------------------------------------------
--
-- unit name:   ltc2174_2l16b_receiver
--
-- description: LTC2174 receiver for Xilinx Spartan-6
--
-- This module implements all the platform-specific logic necessary to receive
-- data from an LTC2174 working in 2-lane, 16-bit serialization mode.
--
--------------------------------------------------------------------------------
-- Copyright CERN 2019
--------------------------------------------------------------------------------
-- Copyright and related rights are licensed under the Solderpad Hardware
-- License, Version 2.0 (the "License"); you may not use this file except
-- in compliance with the License. You may obtain a copy of the License at
-- http://solderpad.org/licenses/SHL-2.0.
-- Unless required by applicable law or agreed to in writing, software,
-- hardware and materials distributed under this License is distributed on an
-- "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
-- or implied. See the License for the specific language governing permissions
-- and limitations under the License.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library unisim;
use unisim.vcomponents.all;

entity ltc2174_2l16b_receiver is
  generic (
    -- There are two topologies available for the generation of the IO clock.
    -- One is based on a BUFIO2+PLL+BUFPLL to generate a double frequency
    -- (800MHz) IO clock which is fed to the SERDES in SDR. This is the default
    -- (g_USE_PLL=TRUE). The other one uses two BUFIO2 to generate two 400MHz
    -- clocks in 180 phase, which are fed to the SERDES in DDR. The second
    -- option saves one PLL, which can be very helpful when dealing with FPGAs
    -- with limited number of available PLLs, but it can be unroutable in some
    -- corner cases. This is why the dual BUFIO2 solution is not the default
    -- option.
    g_USE_PLL : boolean := TRUE);
  port (
    -- ADC data clock
    adc_dco_p_i     : in  std_logic;
    adc_dco_n_i     : in  std_logic;
    -- ADC frame start
    adc_fr_p_i      : in  std_logic;
    adc_fr_n_i      : in  std_logic;
    -- ADC serial data in (odd bits)
    adc_outa_p_i    : in  std_logic_vector(3 downto 0);
    adc_outa_n_i    : in  std_logic_vector(3 downto 0);
    -- ADC serial data in (even bits)
    adc_outb_p_i    : in  std_logic_vector(3 downto 0);
    adc_outb_n_i    : in  std_logic_vector(3 downto 0);
    -- Async reset input (active high) for iserdes
    serdes_arst_i   : in  std_logic := '0';
    -- Manual bitslip command (optional)
    serdes_bslip_i  : in  std_logic := '0';
    -- SERDES BUFPLL lock status flag
    -- (used when g_USE_PLL=TRUE, otherwise it is tied to '1')
    serdes_locked_o : out std_logic;
    -- Indication that SERDES is ok and locked to
    -- frame start pattern
    serdes_synced_o : out std_logic;
    -- ADC parallel data out
    --  (15:0)  = CH1, (31:16) = CH2, (47:32) = CH3, (63:48) = CH4
    --  The two LSBs of each channel are always '0'
    adc_data_o      : out std_logic_vector(63 downto 0);
    -- ADC divided clock, for FPGA logic
    adc_clk_o       : out std_logic);

end entity ltc2174_2l16b_receiver;

architecture arch of ltc2174_2l16b_receiver is

  signal adc_dco           : std_logic;
  signal adc_fr            : std_logic;
  signal adc_outa          : std_logic_vector(3 downto 0);
  signal adc_outb          : std_logic_vector(3 downto 0);
  signal clk_serdes_p      : std_logic;
  signal clk_serdes_n      : std_logic;
  signal clk_div_buf       : std_logic;
  signal serdes_strobe     : std_logic                    := '0';
  signal serdes_auto_bslip : std_logic                    := '0';
  signal serdes_bitslip    : std_logic                    := '0';
  signal serdes_synced     : std_logic                    := '0';
  signal serdes_m2s_shift  : std_logic_vector(8 downto 0) := (others => '0');
  signal serdes_s2m_shift  : std_logic_vector(8 downto 0) := (others => '0');
  signal serdes_serial_in  : std_logic_vector(8 downto 0) := (others => '0');
  signal serdes_out_fr     : std_logic_vector(7 downto 0) := (others => '0');

  signal bitslip_sreg : unsigned(7 downto 0) := to_unsigned(1, 8);

  type serdes_array is array (0 to 8) of std_logic_vector(7 downto 0);
  signal serdes_parallel_out : serdes_array := (others => (others => '0'));

  -- used to select the data rate of the ISERDES blocks
  function f_data_rate_sel (
    constant SDR : boolean)
    return string is
  begin
    if SDR = TRUE then
      return "SDR";
    else
      return "DDR";
    end if;
  end function f_data_rate_sel;

begin  -- architecture arch

  ------------------------------------------------------------------------------
  -- Differential input buffers per input pair
  ------------------------------------------------------------------------------

  -- ADC data clock
  cmp_adc_dco_buf : IBUFGDS
    generic map (
      DIFF_TERM    => TRUE,
      IBUF_LOW_PWR => TRUE,
      IOSTANDARD   => "LVDS_25")
    port map (
      I  => adc_dco_p_i,
      IB => adc_dco_n_i,
      O  => adc_dco);

  -- ADC frame start
  cmp_adc_fr_buf : IBUFDS
    generic map (
      DIFF_TERM    => TRUE,
      IBUF_LOW_PWR => TRUE,
      IOSTANDARD   => "LVDS_25")
    port map (
      I  => adc_fr_p_i,
      IB => adc_fr_n_i,
      O  => adc_fr);

  gen_adc_data_buf : for I in 0 to 3 generate

    cmp_adc_outa_buf : IBUFDS
      generic map (
        DIFF_TERM    => TRUE,
        IBUF_LOW_PWR => TRUE,
        IOSTANDARD   => "LVDS_25")
      port map (
        I  => adc_outa_p_i(i),
        IB => adc_outa_n_i(i),
        O  => adc_outa(i));

    cmp_adc_outb_buf : IBUFDS
      generic map (
        DIFF_TERM    => TRUE,
        IBUF_LOW_PWR => TRUE,
        IOSTANDARD   => "LVDS_25")
      port map (
        I  => adc_outb_p_i(i),
        IB => adc_outb_n_i(i),
        O  => adc_outb(i));

  end generate gen_adc_data_buf;

  ------------------------------------------------------------------------------
  -- Clock generation for deserializer
  --
  -- We use two of the schemes proposed in XAPP1064, v1.2
  ------------------------------------------------------------------------------

  -- XAPP1064, v1.2, page 4, figure 5, without calibration
  gen_pll_bufpll : if g_USE_PLL = TRUE generate

    -- some signals for local interconnect,
    -- not used outside of this if-gen
    signal l_pll_clkin   : std_logic;
    signal l_pll_clkfbin : std_logic;
    signal l_pll_locked  : std_logic;
    signal l_pll_clkout0 : std_logic;
    signal l_pll_clkout1 : std_logic;

  begin

    -- Use the DIVCLK output which is connected to I when
    -- DIVIDE_BYPASS=TRUE and which can be connected to a
    -- PLL_BASE (as opposed to the IOCLK output which cannot).
    cmp_dco_bufio : BUFIO2
      generic map (
        DIVIDE        => 1,
        DIVIDE_BYPASS => TRUE,
        I_INVERT      => FALSE,
        USE_DOUBLER   => FALSE)
      port map (
        I            => adc_dco,
        IOCLK        => open,
        DIVCLK       => l_pll_clkin,
        SERDESSTROBE => open);

    BUFIO2FB_inst : BUFIO2FB
      generic map (
        -- Should match the BUFIO2 setting
        DIVIDE_BYPASS => TRUE)
      port map (
        O => l_pll_clkfbin,
        I => clk_serdes_p);

    cmp_dco_pll : PLL_BASE
      generic map (
        BANDWIDTH      => "OPTIMIZED",
        CLKFBOUT_MULT  => 2,
        CLKIN_PERIOD   => 2.5,
        CLKOUT0_DIVIDE => 1,
        CLKOUT1_DIVIDE => 8,
        CLK_FEEDBACK   => "CLKOUT0",
        COMPENSATION   => "SOURCE_SYNCHRONOUS",
        DIVCLK_DIVIDE  => 1,
        REF_JITTER     => 0.01)
      port map (
        CLKOUT0 => l_pll_clkout0,
        CLKOUT1 => l_pll_clkout1,
        LOCKED  => l_pll_locked,
        CLKFBIN => l_pll_clkfbin,
        CLKIN   => l_pll_clkin,
        RST     => '0');

    cmp_clk_div_buf : BUFG
      port map (
        I => l_pll_clkout1,
        O => clk_div_buf);

    cmp_dco_bufpll : BUFPLL
      generic map (
        DIVIDE => 8)
      port map (
        IOCLK        => clk_serdes_p,
        LOCK         => serdes_locked_o,
        SERDESSTROBE => serdes_strobe,
        GCLK         => clk_div_buf,
        LOCKED       => l_pll_locked,
        PLLIN        => l_pll_clkout0);

    -- not used in this case
    clk_serdes_n <= '0';

  end generate gen_pll_bufpll;

  -- XAPP1064, v1.2, page 5, figure 6, without calibration
  gen_dual_bufio2 : if g_USE_PLL = FALSE generate

    -- some signals for local interconnect,
    -- not used outside of this if-gen
    signal l_clk_div : std_logic;

  begin

    cmp_dco_bufio_p : BUFIO2
      generic map (
        DIVIDE        => 8,
        DIVIDE_BYPASS => FALSE,
        I_INVERT      => FALSE,
        USE_DOUBLER   => TRUE)
      port map (
        I            => adc_dco,
        IOCLK        => clk_serdes_p,
        DIVCLK       => l_clk_div,
        SERDESSTROBE => serdes_strobe);

    cmp_dco_bufio_n : BUFIO2
      generic map (
        DIVIDE        => 8,
        DIVIDE_BYPASS => FALSE,
        I_INVERT      => TRUE,
        USE_DOUBLER   => FALSE)
      port map (
        I            => adc_dco,
        IOCLK        => clk_serdes_n,
        DIVCLK       => open,
        SERDESSTROBE => open);

    cmp_clk_div_buf : BUFG
      port map (
        I => l_clk_div,
        O => clk_div_buf);

    -- not used in this case
    serdes_locked_o <= '1';

  end generate gen_dual_bufio2;

  -- drive out the divided clock, to be used by the FPGA logic
  adc_clk_o <= clk_div_buf;

  ------------------------------------------------------------------------------
  -- Bitslip mechanism for deserializer
  ------------------------------------------------------------------------------

  p_auto_bitslip : process (clk_div_buf)
  begin
    if rising_edge(clk_div_buf) then
      -- Shift register to generate bitslip enable once every 8 clock ticks
      bitslip_sreg <= bitslip_sreg(0) & bitslip_sreg(bitslip_sreg'length-1 downto 1);

      -- Generate bitslip and synced signal
      if(bitslip_sreg(bitslip_sreg'LEFT) = '1') then
        -- use fr_n pattern (fr_p and fr_n are swapped on the adc mezzanine)
        if(serdes_out_fr /= "00001111") then
          serdes_auto_bslip <= '1';
          serdes_synced     <= '0';
        else
          serdes_auto_bslip <= '0';
          serdes_synced     <= '1';
        end if;
      else
        serdes_auto_bslip <= '0';
      end if;
    end if;
  end process p_auto_bitslip;

  serdes_bitslip  <= serdes_auto_bslip or serdes_bslip_i;
  serdes_synced_o <= serdes_synced;

  ------------------------------------------------------------------------------
  -- Data deserializer
  --
  -- For the ISERDES, we use the template proposed in UG615, v14.7, pages
  -- 138-139. Since DATA_WIDTH parameter is greater than four, we need two
  -- ISERDES2 blocks, cascaded.
  ------------------------------------------------------------------------------

  -- serdes inputs forming
  serdes_serial_in <= adc_fr
                      & adc_outa(3) & adc_outb(3)
                      & adc_outa(2) & adc_outb(2)
                      & adc_outa(1) & adc_outb(1)
                      & adc_outa(0) & adc_outb(0);

  gen_adc_data_iserdes : for I in 0 to 8 generate

    cmp_adc_iserdes_master : ISERDES2
      generic map (
        BITSLIP_ENABLE => TRUE,
        DATA_RATE      => f_data_rate_sel(g_USE_PLL),
        DATA_WIDTH     => 8,
        INTERFACE_TYPE => "RETIMED",
        SERDES_MODE    => "MASTER")
      port map (
        CFB0      => open,
        CFB1      => open,
        DFB       => open,
        FABRICOUT => open,
        INCDEC    => open,
        Q1        => serdes_parallel_out(I)(3),
        Q2        => serdes_parallel_out(I)(2),
        Q3        => serdes_parallel_out(I)(1),
        Q4        => serdes_parallel_out(I)(0),
        SHIFTOUT  => serdes_m2s_shift(I),
        VALID     => open,
        BITSLIP   => serdes_bitslip,
        CE0       => '1',
        CLK0      => clk_serdes_p,
        CLK1      => clk_serdes_n,
        CLKDIV    => clk_div_buf,
        D         => serdes_serial_in(I),
        IOCE      => serdes_strobe,
        RST       => serdes_arst_i,
        SHIFTIN   => serdes_s2m_shift(I));

    cmp_adc_iserdes_slave : ISERDES2
      generic map (
        BITSLIP_ENABLE => TRUE,
        DATA_RATE      => f_data_rate_sel(g_USE_PLL),
        DATA_WIDTH     => 8,
        INTERFACE_TYPE => "RETIMED",
        SERDES_MODE    => "SLAVE")
      port map (
        CFB0      => open,
        CFB1      => open,
        DFB       => open,
        FABRICOUT => open,
        INCDEC    => open,
        Q1        => serdes_parallel_out(I)(7),
        Q2        => serdes_parallel_out(I)(6),
        Q3        => serdes_parallel_out(I)(5),
        Q4        => serdes_parallel_out(I)(4),
        SHIFTOUT  => serdes_s2m_shift(I),
        VALID     => open,
        BITSLIP   => serdes_bitslip,
        CE0       => '1',
        CLK0      => clk_serdes_p,
        CLK1      => clk_serdes_n,
        CLKDIV    => clk_div_buf,
        D         => '0',
        IOCE      => serdes_strobe,
        RST       => serdes_arst_i,
        SHIFTIN   => serdes_m2s_shift(I));

  end generate gen_adc_data_iserdes;

  -- Get the Frame start directly from the iserdes output
  serdes_out_fr <= serdes_parallel_out(8);

  -- Data re-ordering for serdes outputs
  gen_serdes_dout_reorder : for I in 0 to 3 generate
    gen_serdes_dout_reorder_bits : for J in 0 to 7 generate
      -- OUT#B: even bits
      adc_data_o(I*16 + 2*J)     <= serdes_parallel_out(2*I)(J);
      -- OUT#A: odd bits
      adc_data_o(I*16 + 2*J + 1) <= serdes_parallel_out(2*I + 1)(J);
    end generate gen_serdes_dout_reorder_bits;
  end generate gen_serdes_dout_reorder;

end architecture arch;
