-------------------------------------------------------------------------------
-- SPDX-License-Identifier: CERN-OHL-W-2.0+
-- Title      : FMC ADC mezzanine
-- Project    : FMC ADC 100M 14B 4CHA gateware
-- URL        : http://www.ohwr.org/projects/fmc-adc-100m14b4cha-gw
-------------------------------------------------------------------------------
-- File       : fmc_adc_mezzanine.vhd
-- Company    : CERN (BE-CO-HT)
-- Created    : 2013-05-07
-- Standard   : VHDL'93/02
-------------------------------------------------------------------------------
-- Description: The FMC ADC mezzanine is wrapper around the fmc-adc-100ms core
-- and the other wishbone slaves connected to a FMC ADC mezzanine.
-------------------------------------------------------------------------------
-- Copyright (c) 2013-2018 CERN (BE-CO-HT)
-------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;


library work;
use work.fmc_adc_100Ms_core_pkg.all;
use work.wishbone_pkg.all;
use work.timetag_core_regs_pkg.all;
use work.timetag_core_defs_pkg.all;

entity fmc_adc_mezzanine is
  generic (
    g_MULTISHOT_RAM_SIZE : natural := 2048;
    -- Only used on Xilinx Spartan6 FPGAs
    g_SPARTAN6_USE_PLL   : boolean                        := TRUE;
    -- External trigger delay calibration value
    g_TRIG_DELAY_EXT     : natural                        := 8;
    -- Software and time trigger delay calibration value
    g_TRIG_DELAY_SW      : natural                        := 12;
    -- Value to be subtracted from trigger tag coarse counter.
    -- This is useful if you know that the system introduces
    -- some systematic delay wrt the actual trigger time
    g_TAG_ADJUST         : natural                        := 27;
    -- FMC-ADC identification number
    g_FMC_ADC_NR         : natural                        := 0;
    -- WB interface configuration
    g_WB_MODE            : t_wishbone_interface_mode      := PIPELINED;
    g_WB_GRANULARITY     : t_wishbone_address_granularity := BYTE);
  port (
    -- Clock, reset
    sys_clk_i   : in std_logic;
    sys_rst_n_i : in std_logic;

    -- CSR wishbone interface
    wb_csr_slave_i : in  t_wishbone_slave_in;
    wb_csr_slave_o : out t_wishbone_slave_out;

    -- DDR wishbone interface
    wb_ddr_clk_i    : in  std_logic;
    wb_ddr_rst_n_i  : in  std_logic;
    wb_ddr_master_i : in  t_wishbone_master_data64_in;
    wb_ddr_master_o : out t_wishbone_master_data64_out;

    -- Interrupts and status
    ddr_wr_fifo_empty_i : in  std_logic;
    trig_irq_o          : out std_logic;
    acq_end_irq_o       : out std_logic;
    eic_irq_o           : out std_logic;
    acq_cfg_ok_o        : out std_logic;

    -- Auxiliary trigger input wishbone interface
    wb_trigin_slave_i : in  t_wishbone_slave_in;
    wb_trigin_slave_o : out t_wishbone_slave_out;

    -- Trigout wishbone interface
    wb_trigout_slave_i : in  t_wishbone_slave_in;
    wb_trigout_slave_o : out t_wishbone_slave_out;

    -- FMC interface
    ext_trigger_p_i : in std_logic;               -- External trigger
    ext_trigger_n_i : in std_logic;

    adc_dco_p_i  : in std_logic;                     -- ADC data clock
    adc_dco_n_i  : in std_logic;
    adc_fr_p_i   : in std_logic;                     -- ADC frame start
    adc_fr_n_i   : in std_logic;
    adc_outa_p_i : in std_logic_vector(3 downto 0);  -- ADC serial data (odd bits)
    adc_outa_n_i : in std_logic_vector(3 downto 0);
    adc_outb_p_i : in std_logic_vector(3 downto 0);  -- ADC serial data (even bits)
    adc_outb_n_i : in std_logic_vector(3 downto 0);

    gpio_dac_clr_n_o : out std_logic;             -- offset DACs clear (active low)
    gpio_led_acq_o   : out std_logic;             -- Mezzanine front panel power LED (PWR)
    gpio_led_trig_o  : out std_logic;             -- Mezzanine front panel trigger LED (TRIG)
    gpio_ssr_ch1_o   : out std_logic_vector(6 downto 0);  -- Channel 1 solid state relays control
    gpio_ssr_ch2_o   : out std_logic_vector(6 downto 0);  -- Channel 2 solid state relays control
    gpio_ssr_ch3_o   : out std_logic_vector(6 downto 0);  -- Channel 3 solid state relays control
    gpio_ssr_ch4_o   : out std_logic_vector(6 downto 0);  -- Channel 4 solid state relays control
    gpio_si570_oe_o  : out std_logic;             -- Si570 (programmable oscillator) output enable

    spi_din_i       : in  std_logic;              -- SPI data from FMC
    spi_dout_o      : out std_logic;              -- SPI data to FMC
    spi_sck_o       : out std_logic;              -- SPI clock
    spi_cs_adc_n_o  : out std_logic;              -- SPI ADC chip select (active low)
    spi_cs_dac1_n_o : out std_logic;  -- SPI channel 1 offset DAC chip select (active low)
    spi_cs_dac2_n_o : out std_logic;  -- SPI channel 2 offset DAC chip select (active low)
    spi_cs_dac3_n_o : out std_logic;  -- SPI channel 3 offset DAC chip select (active low)
    spi_cs_dac4_n_o : out std_logic;  -- SPI channel 4 offset DAC chip select (active low)

    si570_scl_b : inout std_logic;                -- I2C bus clock (Si570)
    si570_sda_b : inout std_logic;                -- I2C bus data (Si570)

    mezz_one_wire_b : inout std_logic;  -- Mezzanine 1-wire interface (DS18B20 thermometer + unique ID)

    wr_tm_link_up_i    : in std_logic;            -- WR link status bit
    wr_tm_time_valid_i : in std_logic;            -- WR timecode valid status bit
    wr_tm_tai_i        : in std_logic_vector(39 downto 0);  -- WR timecode seconds
    wr_tm_cycles_i     : in std_logic_vector(27 downto 0);  -- WR timecode 8ns ticks
    wr_enable_i        : in std_logic);           -- enable white rabbit features on mezzanine

end fmc_adc_mezzanine;

architecture rtl of fmc_adc_mezzanine is

  ------------------------------------------------------------------------------
  -- Constants declaration
  ------------------------------------------------------------------------------

  -- Number of slaves on the wishbone crossbar
  constant c_NUM_WB_SLAVES : integer := 6;

  -- Wishbone slave(s)
  constant c_WB_SLAVE_FMC_ADC     : integer := 0;  -- Mezzanine ADC core
  constant c_WB_SLAVE_FMC_EIC     : integer := 1;  -- Mezzanine interrupt controller
  constant c_WB_SLAVE_FMC_I2C     : integer := 2;  -- Mezzanine I2C controller
  constant c_WB_SLAVE_FMC_ONEWIRE : integer := 3;  -- Mezzanine onewire interface
  constant c_WB_SLAVE_FMC_SPI     : integer := 4;  -- Mezzanine SPI interface
  constant c_WB_SLAVE_TIMETAG     : integer := 5;  -- Mezzanine timetag core

  ------------------------------------------------------------------------------
  -- Signals declaration
  ------------------------------------------------------------------------------

  -- Wishbone buse(s) from crossbar master port(s) to slave(s)
  signal cnx_slave_out : t_wishbone_slave_out_array(c_NUM_WB_SLAVES-1 downto 0);
  signal cnx_slave_in  : t_wishbone_slave_in_array(c_NUM_WB_SLAVES-1 downto 0);

  -- Wishbone buse(s) from master(s) to crossbar slave port(s)
  signal wb_csr_out : t_wishbone_slave_in;
  signal wb_csr_in  : t_wishbone_slave_out;

  -- Mezzanine SPI
  signal spi_din_t : std_logic_vector(3 downto 0) := (others => '0');
  signal spi_ss_t  : std_logic_vector(7 downto 0);

  -- Mezzanine I2C for Si570
  signal si570_scl_in   : std_logic;
  signal si570_scl_out  : std_logic;
  signal si570_scl_oe_n : std_logic;
  signal si570_sda_in   : std_logic;
  signal si570_sda_out  : std_logic;
  signal si570_sda_oe_n : std_logic;

  -- Interrupts (eic)
  signal ddr_wr_fifo_empty_p : std_logic;
  signal acq_end_irq_p       : std_logic;
  signal acq_end_extend      : std_logic;

  -- Time-tagging core
  signal trigger_p    : std_logic;
  signal acq_start_p  : std_logic;
  signal acq_stop_p   : std_logic;
  signal acq_end_p    : std_logic;
  signal trigger_tag  : t_timetag;
  signal time_trigger : std_logic;

  -- Aux time trigger
  signal aux_trigin_enable_in  : std_logic;
  signal aux_trigin_enable_out : std_logic;
  signal aux_trigin_enable_wr  : std_logic;
  signal aux_trigin_tag        : t_timetag;
  signal aux_time_trigger      : std_logic;
  signal aux_trigin_secs       : std_logic_vector(63 downto 0);
  signal aux_trigin_cycs       : std_logic_vector(31 downto 0);
begin

  ------------------------------------------------------------------------------
  -- Main wishbone crossbar for mezzanine
  ------------------------------------------------------------------------------

  cmp_fmc_wb_slave_adapter_in : wb_slave_adapter
    generic map (
      g_master_use_struct  => TRUE,
      g_master_mode        => PIPELINED,
      g_master_granularity => BYTE,
      g_slave_use_struct   => TRUE,
      g_slave_mode         => g_WB_MODE,
      g_slave_granularity  => g_WB_GRANULARITY)
    port map (
      clk_sys_i => sys_clk_i,
      rst_n_i   => sys_rst_n_i,
      slave_i   => wb_csr_slave_i,
      slave_o   => wb_csr_slave_o,
      master_i  => wb_csr_in,
      master_o  => wb_csr_out);

  cmp_crossbar : entity work.fmc_adc_mezzanine_mmap
    port map (
      rst_n_i                  => sys_rst_n_i,
      clk_i                    => sys_clk_i,
      wb_i                     => wb_csr_out,
      wb_o                     => wb_csr_in,
      fmc_adc_100m_csr_i       => cnx_slave_out(c_WB_SLAVE_FMC_ADC),
      fmc_adc_100m_csr_o       => cnx_slave_in(c_WB_SLAVE_FMC_ADC),
      fmc_adc_eic_i            => cnx_slave_out(c_WB_SLAVE_FMC_EIC),
      fmc_adc_eic_o            => cnx_slave_in(c_WB_SLAVE_FMC_EIC),
      si570_i2c_master_i       => cnx_slave_out(c_WB_SLAVE_FMC_I2C),
      si570_i2c_master_o       => cnx_slave_in(c_WB_SLAVE_FMC_I2C),
      ds18b20_onewire_master_i => cnx_slave_out(c_WB_SLAVE_FMC_ONEWIRE),
      ds18b20_onewire_master_o => cnx_slave_in(c_WB_SLAVE_FMC_ONEWIRE),
      fmc_spi_master_i         => cnx_slave_out(c_WB_SLAVE_FMC_SPI),
      fmc_spi_master_o         => cnx_slave_in(c_WB_SLAVE_FMC_SPI),
      timetag_core_i           => cnx_slave_out(c_WB_SLAVE_TIMETAG),
      timetag_core_o           => cnx_slave_in(c_WB_SLAVE_TIMETAG));

  ------------------------------------------------------------------------------
  -- Mezzanine SPI master
  --    Offset DACs control
  --    ADC control
  ------------------------------------------------------------------------------
  cmp_fmc_spi : xwb_spi
    generic map(
      g_interface_mode      => PIPELINED,
      g_address_granularity => BYTE
      )
    port map (
      clk_sys_i => sys_clk_i,
      rst_n_i   => sys_rst_n_i,

      slave_i => cnx_slave_in(c_WB_SLAVE_FMC_SPI),
      slave_o => cnx_slave_out(c_WB_SLAVE_FMC_SPI),
      desc_o  => open,

      pad_cs_o   => spi_ss_t,
      pad_sclk_o => spi_sck_o,
      pad_mosi_o => spi_dout_o,
      pad_miso_i => spi_din_t(spi_din_t'LEFT)
      );

  -- Assign slave select lines
  spi_cs_adc_n_o  <= spi_ss_t(0);
  spi_cs_dac1_n_o <= spi_ss_t(1);
  spi_cs_dac2_n_o <= spi_ss_t(2);
  spi_cs_dac3_n_o <= spi_ss_t(3);
  spi_cs_dac4_n_o <= spi_ss_t(4);

  -- Add some FF after the input pin to solve timing problem
  p_fmc_spi : process (sys_clk_i)
  begin
    if rising_edge(sys_clk_i) then
      spi_din_t <= spi_din_t(spi_din_t'LEFT-1 downto 0) & spi_din_i;
    end if;
  end process p_fmc_spi;

  ------------------------------------------------------------------------------
  -- Mezzanine I2C
  --    Si570 control
  --
  -- Note: I2C registers are 8-bit wide, but accessed as 32-bit registers
  ------------------------------------------------------------------------------
  cmp_fmc_i2c : xwb_i2c_master
    generic map(
      g_interface_mode      => PIPELINED,
      g_address_granularity => BYTE
      )
    port map (
      clk_sys_i => sys_clk_i,
      rst_n_i   => sys_rst_n_i,

      slave_i => cnx_slave_in(c_WB_SLAVE_FMC_I2C),
      slave_o => cnx_slave_out(c_WB_SLAVE_FMC_I2C),
      desc_o  => open,

      scl_pad_i(0)    => si570_scl_in,
      scl_pad_o(0)    => si570_scl_out,
      scl_padoen_o(0) => si570_scl_oe_n,
      sda_pad_i(0)    => si570_sda_in,
      sda_pad_o(0)    => si570_sda_out,
      sda_padoen_o(0) => si570_sda_oe_n
      );

  -- Tri-state buffer for SDA and SCL
  si570_scl_b  <= si570_scl_out when si570_scl_oe_n = '0' else 'Z';
  si570_scl_in <= si570_scl_b;

  si570_sda_b  <= si570_sda_out when si570_sda_oe_n = '0' else 'Z';
  si570_sda_in <= si570_sda_b;

  ------------------------------------------------------------------------------
  -- ADC core
  --    Solid State Relays control
  --    Si570 output enable
  --    Offset DACs control (CLR_N)
  --    ADC core control and status
  ------------------------------------------------------------------------------

  cmp_fmc_adc_100Ms_core : entity work.fmc_adc_100Ms_core
    generic map (
      g_MULTISHOT_RAM_SIZE => g_MULTISHOT_RAM_SIZE,
      g_SPARTAN6_USE_PLL   => g_SPARTAN6_USE_PLL,
      g_TRIG_DELAY_EXT     => g_TRIG_DELAY_EXT,
      g_TRIG_DELAY_SW      => g_TRIG_DELAY_SW,
      g_FMC_ADC_NR         => g_FMC_ADC_NR,
      g_WB_CSR_MODE        => PIPELINED,
      g_WB_CSR_GRANULARITY => BYTE)
    port map (
      sys_clk_i   => sys_clk_i,
      sys_rst_n_i => sys_rst_n_i,

      wb_csr_slave_i => cnx_slave_in(c_WB_SLAVE_FMC_ADC),
      wb_csr_slave_o => cnx_slave_out(c_WB_SLAVE_FMC_ADC),

      wb_ddr_clk_i    => wb_ddr_clk_i,
      wb_ddr_rst_n_i  => wb_ddr_rst_n_i,
      wb_ddr_master_o => wb_ddr_master_o,
      wb_ddr_master_i => wb_ddr_master_i,

      acq_cfg_ok_o => acq_cfg_ok_o,

      wb_trigout_slave_i => wb_trigout_slave_i,
      wb_trigout_slave_o => wb_trigout_slave_o,

      trigger_p_o   => trigger_p,
      acq_start_p_o => acq_start_p,
      acq_stop_p_o  => acq_stop_p,
      acq_end_p_o   => acq_end_p,

      trigger_tag_i   => trigger_tag,
      time_trig_i     => time_trigger,
      aux_time_trig_i => aux_time_trigger,

      wr_tm_link_up_i    => wr_tm_link_up_i,
      wr_tm_time_valid_i => wr_tm_time_valid_i,
      wr_enable_i        => wr_enable_i,

      ext_trigger_p_i => ext_trigger_p_i,
      ext_trigger_n_i => ext_trigger_n_i,

      adc_dco_p_i  => adc_dco_p_i,
      adc_dco_n_i  => adc_dco_n_i,
      adc_fr_p_i   => adc_fr_p_i,
      adc_fr_n_i   => adc_fr_n_i,
      adc_outa_p_i => adc_outa_p_i,
      adc_outa_n_i => adc_outa_n_i,
      adc_outb_p_i => adc_outb_p_i,
      adc_outb_n_i => adc_outb_n_i,

      gpio_dac_clr_n_o => gpio_dac_clr_n_o,
      gpio_led_acq_o   => gpio_led_acq_o,
      gpio_led_trig_o  => gpio_led_trig_o,
      gpio_ssr_ch1_o   => gpio_ssr_ch1_o,
      gpio_ssr_ch2_o   => gpio_ssr_ch2_o,
      gpio_ssr_ch3_o   => gpio_ssr_ch3_o,
      gpio_ssr_ch4_o   => gpio_ssr_ch4_o,
      gpio_si570_oe_o  => gpio_si570_oe_o);

  ------------------------------------------------------------------------------
  -- Mezzanine 1-wire master
  --    DS18B20 (thermometer + unique ID)
  ------------------------------------------------------------------------------

  cmp_fmc_onewire : entity work.xwb_ds182x_readout
    generic map (
      g_CLOCK_FREQ_KHZ   => 125000,
      g_USE_INTERNAL_PPS => TRUE)
    port map (
      clk_i     => sys_clk_i,
      rst_n_i   => sys_rst_n_i,
      wb_i      => cnx_slave_in(c_WB_SLAVE_FMC_ONEWIRE),
      wb_o      => cnx_slave_out(c_WB_SLAVE_FMC_ONEWIRE),
      pps_p_i   => '0',
      onewire_b => mezz_one_wire_b);

  ------------------------------------------------------------------------------
  -- FMC0 interrupt controller
  ------------------------------------------------------------------------------

  cmp_fmc_adc_eic : entity work.fmc_adc_eic
    port map (
      rst_n_i       => sys_rst_n_i,
      clk_i         => sys_clk_i,
      wb_i          => cnx_slave_in(c_WB_SLAVE_FMC_EIC),
      wb_o          => cnx_slave_out(c_WB_SLAVE_FMC_EIC),
      irq_trig_i    => trigger_p,
      irq_acq_end_i => acq_end_irq_p,
      int_o         => eic_irq_o);

  -- Detects end of adc core writing to ddr
  cmp_ddr_wr_fifo_empty_posedge : entity work.gc_posedge
    port map (
      clk_i   => sys_clk_i,
      rst_n_i => '1',
      data_i  => ddr_wr_fifo_empty_i,
      pulse_o => ddr_wr_fifo_empty_p);

  -- End of acquisition interrupt generation
  p_acq_end_extend : process (sys_clk_i)
  begin
    if rising_edge(sys_clk_i) then
      if sys_rst_n_i = '0' then
        acq_end_extend <= '0';
      elsif acq_end_p = '1' then
        acq_end_extend <= '1';
      elsif ddr_wr_fifo_empty_p = '1' then
        acq_end_extend <= '0';
      end if;
    end if;
  end process p_acq_end_extend;

  acq_end_irq_p <= ddr_wr_fifo_empty_p and acq_end_extend;

  trig_irq_o    <= trigger_p;
  acq_end_irq_o <= acq_end_irq_p;

  ------------------------------------------------------------------------------
  -- Time-tagging core
  ------------------------------------------------------------------------------
  cmp_timetag_core : entity work.timetag_core
    generic map (
      g_WB_MODE        => PIPELINED,
      g_WB_GRANULARITY => BYTE,
      -- Systematic delay introduced to the time tag by the FMC-ADC-100M core.
      -- Measured experimentally.
      g_TAG_ADJUST     => g_TAG_ADJUST)
    port map(
      clk_i   => sys_clk_i,
      rst_n_i => sys_rst_n_i,

      trigger_p_i   => trigger_p,
      acq_start_p_i => acq_start_p,
      acq_stop_p_i  => acq_stop_p,
      acq_end_p_i   => acq_end_p,

      wr_enabled_i => wr_enable_i,

      wr_tm_time_valid_i => wr_tm_time_valid_i,
      wr_tm_tai_i        => wr_tm_tai_i,
      wr_tm_cycles_i     => wr_tm_cycles_i,

      trig_tag_o  => trigger_tag,
      time_trig_o => time_trigger,

      aux_trigin_enable_o    => aux_trigin_enable_in,
      aux_trigin_enable_i    => aux_trigin_enable_out,
      aux_trigin_enable_wr_i => aux_trigin_enable_wr,
      aux_trigin_tag_i       => aux_trigin_tag,
      aux_trigin_o           => aux_time_trigger,

      wb_i => cnx_slave_in(c_WB_SLAVE_TIMETAG),
      wb_o => cnx_slave_out(c_WB_SLAVE_TIMETAG));

  cmp_aux_trigin : entity work.aux_trigin
    port map (
      rst_n_i    => sys_rst_n_i,
      clk_i      => sys_clk_i,
      wb_i       => wb_trigin_slave_i,
      wb_o       => wb_trigin_slave_o,

      ctrl_enable_i  => aux_trigin_enable_in,
      ctrl_enable_o  => aux_trigin_enable_out,
      ctrl_wr_o      => aux_trigin_enable_wr,

      seconds_o      => aux_trigin_secs,
      cycles_o       => aux_trigin_cycs
      );

  aux_trigin_tag <= (seconds => aux_trigin_secs(39 downto 0),
                     coarse => aux_trigin_cycs(27 downto 0));

end rtl;
