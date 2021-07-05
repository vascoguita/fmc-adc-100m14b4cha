--------------------------------------------------------------------------------
-- SPDX-License-Identifier: CERN-OHL-W-2.0+
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
  signal adc_out, adc_out_dly_m, adc_out_dly_s : std_logic_vector(8 downto 0);
  signal serdes_incdec, serdes_valid : std_logic_vector(8 downto 0);
  signal clk_serdes_p      : std_logic;
  signal clk_serdes_n      : std_logic;
  signal clk_div_buf       : std_logic;
  signal serdes_strobe     : std_logic                    := '0';
  signal serdes_auto_bslip : std_logic                    := '0';
  signal serdes_bitslip    : std_logic                    := '0';
  signal serdes_synced     : std_logic                    := '0';
  signal serdes_m2s_shift  : std_logic_vector(8 downto 0) := (others => '0');
  signal serdes_s2m_shift  : std_logic_vector(8 downto 0) := (others => '0');
  signal serdes_out_fr     : std_logic_vector(6 downto 0) := (others => '0');

  signal bitslip_sreg : unsigned(7 downto 0) := to_unsigned(1, 8);

  type serdes_array is array (0 to 8) of std_logic_vector(6 downto 0);
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
      O  => adc_out(8));

  gen_adc_data_buf : for I in 0 to 3 generate

    cmp_adc_outa_buf : IBUFDS
      generic map (
        DIFF_TERM    => TRUE,
        IBUF_LOW_PWR => TRUE,
        IOSTANDARD   => "LVDS_25")
      port map (
        I  => adc_outa_p_i(i),
        IB => adc_outa_n_i(i),
        O  => adc_out(2 * i + 1));

    cmp_adc_outb_buf : IBUFDS
      generic map (
        DIFF_TERM    => TRUE,
        IBUF_LOW_PWR => TRUE,
        IOSTANDARD   => "LVDS_25")
      port map (
        I  => adc_outb_p_i(i),
        IB => adc_outb_n_i(i),
        O  => adc_out(2 * i));

  end generate gen_adc_data_buf;

  gen_adc_idelay: for I in adc_out'range generate
    signal cal_m, inc_m, ce_m, rst_m, busy_m : std_logic;
    signal cal_s, inc_s, ce_s, rst_s, busy_s : std_logic;
  begin
    cmp_idelay_master: IODELAY2
      generic map (
        DATA_RATE      		 => f_data_rate_sel(g_USE_PLL),   -- <SDR>, DDR
        IDELAY_VALUE  		 => 0, 			-- {0 ... 255}
        IDELAY2_VALUE 		 => 0, 			-- {0 ... 255}
        IDELAY_MODE  		   => "NORMAL",-- NORMAL, PCI
        ODELAY_VALUE  		 => 0, 			-- {0 ... 255}
        IDELAY_TYPE   		 => "DIFF_PHASE_DETECTOR",-- "DEFAULT", "DIFF_PHASE_DETECTOR", "FIXED", "VARIABLE_FROM_HALF_MAX", "VARIABLE_FROM_ZERO"
        COUNTER_WRAPAROUND => "WRAPAROUND", 	-- <STAY_AT_LIMIT>, WRAPAROUND
        DELAY_SRC     		 => "IDATAIN", 		-- "IO", "IDATAIN", "ODATAIN"
        SERDES_MODE   		 => "MASTER", 		-- <NONE>, MASTER, SLAVE
        SIM_TAPDELAY_VALUE => 49) 			--
      port map (
        IDATAIN  		=> adc_out(i), 	-- data from primary IOB
        TOUT     		=> open, 		-- tri-state signal to IOB
        DOUT     		=> open, 		-- output data to IOB
        T        		=> '1', 		-- tri-state control from OLOGIC/OSERDES2 				
        ODATAIN  		=> '0', 		-- data from OLOGIC/OSERDES2
        DATAOUT  		=> adc_out_dly_m(i), 		-- Output data 1 to ILOGIC/ISERDES2
        DATAOUT2 		=> open, 		-- Output data 2 to ILOGIC/ISERDES2
        IOCLK0   		=> clk_serdes_p, 		-- High speed clock for calibration
        IOCLK1   		=> clk_serdes_n, 		-- High speed clock for calibration
        CLK      		=> clk_div_buf,   	-- Fabric clock (GCLK) for control signals
        CAL      		=> cal_m,	          -- Calibrate control signal
        INC      		=> inc_m,		        -- Increment counter
        CE       		=> ce_m,      		  -- Clock Enable
        RST      		=> rst_m,	        	-- Reset delay line
        BUSY      	=> busy_m) ;        -- output signal indicating sync circuit has finished / calibration has finished 

      cmp_idelay_slave: IODELAY2
        generic map (
          DATA_RATE      		 => f_data_rate_sel(g_USE_PLL),   -- <SDR>, DDR
          IDELAY_VALUE  		 => 0, 			-- {0 ... 255}
          IDELAY2_VALUE 		 => 0, 			-- {0 ... 255}
          IDELAY_MODE  		   => "NORMAL",-- NORMAL, PCI
          ODELAY_VALUE  		 => 0, 			-- {0 ... 255}
          IDELAY_TYPE   		 => "DIFF_PHASE_DETECTOR",-- "DEFAULT", "DIFF_PHASE_DETECTOR", "FIXED", "VARIABLE_FROM_HALF_MAX", "VARIABLE_FROM_ZERO"
          COUNTER_WRAPAROUND => "WRAPAROUND", 	-- <STAY_AT_LIMIT>, WRAPAROUND
          DELAY_SRC     		 => "IDATAIN", 		-- "IO", "IDATAIN", "ODATAIN"
          SERDES_MODE   		 => "SLAVE", 		-- <NONE>, MASTER, SLAVE
          SIM_TAPDELAY_VALUE => 49) 			--
        port map (
          IDATAIN  		=> adc_out(i), 	-- data from primary IOB
          TOUT     		=> open, 		-- tri-state signal to IOB
          DOUT     		=> open, 		-- output data to IOB
          T        		=> '1', 		-- tri-state control from OLOGIC/OSERDES2 				
          ODATAIN  		=> '0', 		-- data from OLOGIC/OSERDES2
          DATAOUT  		=> adc_out_dly_s(i), 		-- Output data 1 to ILOGIC/ISERDES2
          DATAOUT2 		=> open, 		-- Output data 2 to ILOGIC/ISERDES2
          IOCLK0   		=> clk_serdes_p, 		-- High speed clock for calibration
          IOCLK1   		=> clk_serdes_n, 		-- High speed clock for calibration
          CLK      		=> clk_div_buf,   	-- Fabric clock (GCLK) for control signals
          CAL      		=> cal_s,	          -- Calibrate control signal
          INC      		=> inc_s,		        -- Increment counter
          CE       		=> ce_s,      		  -- Clock Enable
          RST      		=> rst_s,	        	-- Reset delay line
          BUSY      	=> busy_s) ;        -- output signal indicating sync circuit has finished / calibration has finished 

      cal_m <= '0';
      cal_s <= '0';
      ce_s <= '0';
      ce_m <= '0';
      rst_m <= '0';
      rst_s <= '0';
    end generate;

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
        I            => adc_dco,  --  350Mhz
        IOCLK        => open,
        DIVCLK       => l_pll_clkin,
        SERDESSTROBE => open);

    BUFIO2FB_inst : BUFIO2FB
      generic map (
        -- Should match the BUFIO2 setting
        DIVIDE_BYPASS => TRUE)
      port map (
        O => l_pll_clkfbin,
        I => clk_serdes_p);             --  100Mhz

    cmp_dco_pll : PLL_BASE
      generic map (
        BANDWIDTH      => "OPTIMIZED",
        CLKFBOUT_MULT  => 2,           --  M=2, Fvco=700Mhz
        CLKIN_PERIOD   => 2.86,
        CLKOUT0_DIVIDE => 1,
        CLKOUT1_DIVIDE => 7,
        CLK_FEEDBACK   => "CLKOUT0",
        COMPENSATION   => "SOURCE_SYNCHRONOUS",
        DIVCLK_DIVIDE  => 1,
        REF_JITTER     => 0.01)
      port map (
        CLKOUT0 => l_pll_clkout0,      --  700Mhz (to BUFPLL)
        CLKOUT1 => l_pll_clkout1,      --  100Mhz
        LOCKED  => l_pll_locked,
        CLKFBIN => l_pll_clkfbin,
        CLKIN   => l_pll_clkin,        --  350Mhz
        RST     => '0');

    cmp_clk_div_buf : BUFG
      port map (
        I => l_pll_clkout1,            --  100Mhz
        O => clk_div_buf);

    cmp_dco_bufpll : BUFPLL
      generic map (
        DIVIDE => 7)
      port map (
        IOCLK        => clk_serdes_p,    -- 700Mhz (to BUFIO2FB and serdes)
        LOCK         => serdes_locked_o,
        SERDESSTROBE => serdes_strobe,   -- 100Mhz
        GCLK         => clk_div_buf,
        LOCKED       => l_pll_locked,
        PLLIN        => l_pll_clkout0);  -- 700Mhz

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
        DIVIDE        => 7,
        DIVIDE_BYPASS => FALSE,
        I_INVERT      => FALSE,
        USE_DOUBLER   => TRUE)
      port map (
        I            => adc_dco,        -- 350Mhz DDR
        IOCLK        => clk_serdes_p,   -- 350Mhz
        DIVCLK       => l_clk_div,      -- 100Mhz (2*350/7)
        SERDESSTROBE => serdes_strobe);

    cmp_dco_bufio_n : BUFIO2
      generic map (
        DIVIDE        => 7,
        DIVIDE_BYPASS => FALSE,
        I_INVERT      => TRUE,
        USE_DOUBLER   => FALSE)
      port map (
        I            => adc_dco,
        IOCLK        => clk_serdes_n,   --  350Mhz, 180 phase shift.
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
      if bitslip_sreg(bitslip_sreg'LEFT) = '1' then
        if serdes_out_fr /= "0000000" and serdes_out_fr /= "1111111" then
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
  gen_adc_data_iserdes : for I in 0 to 8 generate

    cmp_adc_iserdes_master : ISERDES2
      generic map (
        BITSLIP_ENABLE => TRUE,
        DATA_RATE      => f_data_rate_sel(g_USE_PLL),
        DATA_WIDTH     => 7,
        INTERFACE_TYPE => "RETIMED",
        SERDES_MODE    => "MASTER")
      port map (
        CFB0      => open,
        CFB1      => open,
        DFB       => open,
        FABRICOUT => open,
        INCDEC    => serdes_incdec(i),
        Q1        => serdes_parallel_out(I)(3),
        Q2        => serdes_parallel_out(I)(2),
        Q3        => serdes_parallel_out(I)(1),
        Q4        => serdes_parallel_out(I)(0),
        SHIFTOUT  => serdes_m2s_shift(I),
        VALID     => serdes_valid(i),
        BITSLIP   => serdes_bitslip,
        CE0       => '1',
        CLK0      => clk_serdes_p,
        CLK1      => clk_serdes_n,
        CLKDIV    => clk_div_buf,
        D         => adc_out_dly_m(i),
        IOCE      => serdes_strobe,
        RST       => serdes_arst_i,
        SHIFTIN   => serdes_s2m_shift(I));

    cmp_adc_iserdes_slave : ISERDES2
      generic map (
        BITSLIP_ENABLE => TRUE,
        DATA_RATE      => f_data_rate_sel(g_USE_PLL),
        DATA_WIDTH     => 7,
        INTERFACE_TYPE => "RETIMED",
        SERDES_MODE    => "SLAVE")
      port map (
        CFB0      => open,
        CFB1      => open,
        DFB       => open,
        FABRICOUT => open,
        INCDEC    => open,
        Q1        => open,
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
        D         => adc_out_dly_s(i),
        IOCE      => serdes_strobe,
        RST       => serdes_arst_i,
        SHIFTIN   => serdes_m2s_shift(I));

  end generate gen_adc_data_iserdes;

  -- Get the Frame start directly from the iserdes output
  serdes_out_fr <= serdes_parallel_out(8);

  -- Data re-ordering for serdes outputs
  gen_serdes_dout_reorder : for I in 0 to 3 generate
    gen_serdes_dout_reorder_bits : for J in 0 to 6 generate
      -- OUT#B: even bits
      adc_data_o(I*16 + 2*J + 2)     <= serdes_parallel_out(2*I)(J);
      -- OUT#A: odd bits
      adc_data_o(I*16 + 2*J + 3) <= serdes_parallel_out(2*I + 1)(J);
    end generate gen_serdes_dout_reorder_bits;
    adc_data_o(I*16 + 0) <= '0';
    adc_data_o(I*16 + 1) <= '0';
  end generate gen_serdes_dout_reorder;

end architecture arch;
