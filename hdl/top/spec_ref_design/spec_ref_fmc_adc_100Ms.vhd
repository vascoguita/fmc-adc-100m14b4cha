-------------------------------------------------------------------------------
-- Title      : FMC ADC 100Ms/s SPEC top-level
-- Project    : FMC ADC 100M 14B 4CHA gateware
-- URL        : http://www.ohwr.org/projects/fmc-adc-100m14b4cha-gw
-------------------------------------------------------------------------------
-- File       : spec_ref_fmc_adc_100Ms.vhd
-- Company    : CERN (BE-CO-HT)
-- Created    : 2011-02-24
-- Standard   : VHDL'93/02
-------------------------------------------------------------------------------
-- Description: Top entity of FMC ADC 100Ms/s design for Simple PCIe FMC
-- Carrier (SPEC). See also: http://www.ohwr.org/projects/spec
-------------------------------------------------------------------------------
-- Copyright (c) 2011-2020 CERN (BE-CO-HT)
-------------------------------------------------------------------------------
-- GNU LESSER GENERAL PUBLIC LICENSE
-------------------------------------------------------------------------------
-- This source file is free software; you can redistribute it and/or modify it
-- under the terms of the GNU Lesser General Public License as published by the
-- Free Software Foundation; either version 2.1 of the License, or (at your
-- option) any later version. This source is distributed in the hope that it
-- will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
-- of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
-- See the GNU Lesser General Public License for more details. You should have
-- received a copy of the GNU Lesser General Public License along with this
-- source; if not, download it from http://www.gnu.org/licenses/lgpl-2.1.html
-------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;

library UNISIM;
use UNISIM.vcomponents.all;

library work;

use work.gencores_pkg.all;
use work.wishbone_pkg.all;
use work.fmc_adc_mezzanine_pkg.all;
use work.wr_board_pkg.all;

entity spec_ref_fmc_adc_100Ms is
  generic(
    g_SIMULATION         : integer := 0;
    g_MULTISHOT_RAM_SIZE : natural := 2048;
    g_WRPC_INITF         : string  := "../../ip_cores/wr-cores/bin/wrpc/wrc_phy8.bram");
  port
    (
      -- Reset button
      button1_n_i : in  std_logic;

      -- Local oscillators
      clk_20m_vcxo_i : in std_logic;              -- 20MHz VCXO clock

      clk_125m_pllref_p_i : in std_logic;         -- 125 MHz PLL reference
      clk_125m_pllref_n_i : in std_logic;

      clk_125m_gtp_n_i : in std_logic;            -- 125 MHz GTP reference
      clk_125m_gtp_p_i : in std_logic;

      -- DAC interface (20MHz and 25MHz VCXO)
      pll25dac_cs_n_o : out std_logic;            -- 25MHz VCXO
      pll20dac_cs_n_o : out std_logic;            -- 20MHz VCXO
      plldac_din_o    : out std_logic;
      plldac_sclk_o   : out std_logic;

      -- Carrier front panel LEDs
      led_act_o   : out std_logic;
      led_link_o : out std_logic;

      -- Auxiliary pins
      aux_leds_o : out std_logic_vector(3 downto 0);

      -- PCB version
      pcbrev_i : in std_logic_vector(3 downto 0);

      -- Carrier 1-wire interface (DS18B20 thermometer + unique ID)
      onewire_b : inout std_logic;

      -- SFP
      sfp_txp_o         : out   std_logic;
      sfp_txn_o         : out   std_logic;
      sfp_rxp_i         : in    std_logic;
      sfp_rxn_i         : in    std_logic;
      sfp_mod_def0_i    : in    std_logic;        -- sfp detect
      sfp_mod_def1_b    : inout std_logic;        -- scl
      sfp_mod_def2_b    : inout std_logic;        -- sda
      sfp_rate_select_o : out   std_logic;
      sfp_tx_fault_i    : in    std_logic;
      sfp_tx_disable_o  : out   std_logic;
      sfp_los_i         : in    std_logic;

      -- SPI
      spi_sclk_o : out std_logic;
      spi_ncs_o  : out std_logic;
      spi_mosi_o : out std_logic;
      spi_miso_i : in  std_logic := 'L';

      -- UART
      uart_rxd_i : in  std_logic;
      uart_txd_o : out std_logic;

      ------------------------------------------
      -- GN4124 interface
      --
      -- gn_gpio_b[1] -> AB19 -> GN4124 GPIO9
      -- gn_gpio_b[0] -> U16  -> GN4124 GPIO8
      ------------------------------------------
      gn_rst_n_i      : in    std_logic;
      gn_p2l_clk_n_i  : in    std_logic;
      gn_p2l_clk_p_i  : in    std_logic;
      gn_p2l_rdy_o    : out   std_logic;
      gn_p2l_dframe_i : in    std_logic;
      gn_p2l_valid_i  : in    std_logic;
      gn_p2l_data_i   : in    std_logic_vector(15 downto 0);
      gn_p_wr_req_i   : in    std_logic_vector(1 downto 0);
      gn_p_wr_rdy_o   : out   std_logic_vector(1 downto 0);
      gn_rx_error_o   : out   std_logic;
      gn_l2p_clk_n_o  : out   std_logic;
      gn_l2p_clk_p_o  : out   std_logic;
      gn_l2p_dframe_o : out   std_logic;
      gn_l2p_valid_o  : out   std_logic;
      gn_l2p_edb_o    : out   std_logic;
      gn_l2p_data_o   : out   std_logic_vector(15 downto 0);
      gn_l2p_rdy_i    : in    std_logic;
      gn_l_wr_rdy_i   : in    std_logic_vector(1 downto 0);
      gn_p_rd_d_rdy_i : in    std_logic_vector(1 downto 0);
      gn_tx_error_i   : in    std_logic;
      gn_vc_rdy_i     : in    std_logic_vector(1 downto 0);
      gn_gpio_b       : inout std_logic_vector(1 downto 0);

      ------------------------------------------
      -- DDR (bank 3)
      ------------------------------------------
      ddr_a_o       : out   std_logic_vector(13 downto 0);
      ddr_ba_o      : out   std_logic_vector(2 downto 0);
      ddr_cas_n_o   : out   std_logic;
      ddr_ck_n_o    : out   std_logic;
      ddr_ck_p_o    : out   std_logic;
      ddr_cke_o     : out   std_logic;
      ddr_dq_b      : inout std_logic_vector(15 downto 0);
      ddr_ldm_o     : out   std_logic;
      ddr_ldqs_n_b  : inout std_logic;
      ddr_ldqs_p_b  : inout std_logic;
      ddr_odt_o     : out   std_logic;
      ddr_ras_n_o   : out   std_logic;
      ddr_reset_n_o : out   std_logic;
      ddr_rzq_b     : inout std_logic;
      ddr_udm_o     : out   std_logic;
      ddr_udqs_n_b  : inout std_logic;
      ddr_udqs_p_b  : inout std_logic;
      ddr_we_n_o    : out   std_logic;

      ------------------------------------------
      -- FMC slot
      ------------------------------------------
      adc_ext_trigger_p_i : in std_logic;        -- External trigger
      adc_ext_trigger_n_i : in std_logic;

      adc_dco_p_i  : in std_logic;                     -- ADC data clock
      adc_dco_n_i  : in std_logic;
      adc_fr_p_i   : in std_logic;                     -- ADC frame start
      adc_fr_n_i   : in std_logic;
      adc_outa_p_i : in std_logic_vector(3 downto 0);  -- ADC serial data (odd bits)
      adc_outa_n_i : in std_logic_vector(3 downto 0);
      adc_outb_p_i : in std_logic_vector(3 downto 0);  -- ADC serial data (even bits)
      adc_outb_n_i : in std_logic_vector(3 downto 0);

      adc_spi_din_i       : in  std_logic;       -- SPI data from FMC
      adc_spi_dout_o      : out std_logic;       -- SPI data to FMC
      adc_spi_sck_o       : out std_logic;       -- SPI clock
      adc_spi_cs_adc_n_o  : out std_logic;       -- SPI ADC chip select (active low)
      adc_spi_cs_dac1_n_o : out std_logic;  -- SPI channel 1 offset DAC chip select (active low)
      adc_spi_cs_dac2_n_o : out std_logic;  -- SPI channel 2 offset DAC chip select (active low)
      adc_spi_cs_dac3_n_o : out std_logic;  -- SPI channel 3 offset DAC chip select (active low)
      adc_spi_cs_dac4_n_o : out std_logic;  -- SPI channel 4 offset DAC chip select (active low)

      adc_gpio_dac_clr_n_o : out std_logic;      -- offset DACs clear (active low)
      adc_gpio_led_acq_o   : out std_logic;      -- Mezzanine front panel power LED (PWR)
      adc_gpio_led_trig_o  : out std_logic;      -- Mezzanine front panel trigger LED (TRIG)
      adc_gpio_ssr_ch1_o   : out std_logic_vector(6 downto 0);  -- Channel 1 solid state relays control
      adc_gpio_ssr_ch2_o   : out std_logic_vector(6 downto 0);  -- Channel 2 solid state relays control
      adc_gpio_ssr_ch3_o   : out std_logic_vector(6 downto 0);  -- Channel 3 solid state relays control
      adc_gpio_ssr_ch4_o   : out std_logic_vector(6 downto 0);  -- Channel 4 solid state relays control
      adc_gpio_si570_oe_o  : out std_logic;      -- Si570 (programmable oscillator) output enable

      adc_si570_scl_b : inout std_logic;         -- I2C bus clock (Si570)
      adc_si570_sda_b : inout std_logic;         -- I2C bus data (Si570)

      adc_one_wire_b : inout std_logic;  -- Mezzanine 1-wire interface (DS18B20 thermometer + unique ID)

      ------------------------------------------
      -- FMC slot management
      ------------------------------------------
      fmc0_prsnt_m2c_n_i : in    std_logic;       -- Mezzanine present (active low)
      fmc0_scl_b         : inout std_logic;       -- Mezzanine system I2C clock (EEPROM)
      fmc0_sda_b         : inout std_logic        -- Mezzanine system I2C data (EEPROM)
      );

end spec_ref_fmc_adc_100Ms;


architecture arch of spec_ref_fmc_adc_100Ms is

  ------------------------------------------------------------------------------
  -- Constants declaration
  ------------------------------------------------------------------------------

  -- Number of slaves on the wishbone crossbar
  constant c_NUM_WB_SLAVES : integer := 2;

  -- Wishbone slave(s)
  constant c_WB_SLAVE_METADATA : integer := 0;
  constant c_WB_SLAVE_FMC_ADC  : integer := 1;    -- FMC ADC mezzanine

  -- Convention metadata base address
  constant c_METADATA_ADDR : t_wishbone_address := x"0000_2000";

  ------------------------------------------------------------------------------
  -- Signals declaration
  ------------------------------------------------------------------------------

  -- Clocks and resets
  signal clk_sys_62m5       : std_logic;
  signal clk_ref_125m       : std_logic;

  signal rst_sys_62m5_n  : std_logic := '0';
  signal rst_ref_125m_n  : std_logic := '0';

  -- Wishbone buse(s) from master(s) to crossbar slave port(s)
  signal cnx_master_out : t_wishbone_master_out;
  signal cnx_master_in  : t_wishbone_master_in;

  -- Wishbone buse(s) from crossbar master port(s) to slave(s)
  signal cnx_slave_out : t_wishbone_slave_out_array(c_NUM_WB_SLAVES-1 downto 0);
  signal cnx_slave_in  : t_wishbone_slave_in_array(c_NUM_WB_SLAVES-1 downto 0);

  -- Wishbone bus from cross-clocking module to FMC mezzanine
  signal cnx_fmc_sync_master_out : t_wishbone_master_out;
  signal cnx_fmc_sync_master_in  : t_wishbone_master_in;

  -- FMC ADC core to DDR wishbone bus
  signal fmc_wb_ddr_in  : t_wishbone_master_data64_in;
  signal fmc_wb_ddr_out : t_wishbone_master_data64_out;

  -- Interrupts and status
  signal ddr_wr_fifo_empty      : std_logic;
  signal ddr_wr_fifo_empty_sync : std_logic;
  signal fmc_irq                : std_logic;
  signal fmc_acq_cfg_ok         : std_logic;
  signal irq_vector             : std_logic_vector(0 downto 0);
  signal gn4124_access          : std_logic;

  -- WR PTP core timing interface
  signal tm_link_up         : std_logic;
  signal tm_tai             : std_logic_vector(39 downto 0);
  signal tm_cycles          : std_logic_vector(27 downto 0);
  signal tm_time_valid      : std_logic;
  signal tm_time_valid_sync : std_logic;
  signal wrabbit_en         : std_logic;
  signal pps_led            : std_logic;

begin  -- architecture arch

  inst_spec_base : entity work.spec_base_wr
    generic map (
      g_WITH_VIC      => TRUE,
      g_WITH_ONEWIRE  => FALSE,
      g_WITH_SPI      => FALSE,
      g_WITH_WR       => TRUE,
      g_WITH_DDR      => TRUE,
      g_DDR_DATA_SIZE => 64,
      g_APP_OFFSET    => c_METADATA_ADDR,
      g_NUM_USER_IRQ  => 1,
      g_DPRAM_INITF   => g_WRPC_INITF,
      g_AUX_CLKS      => 0,
      g_FABRIC_IFACE  => plain,
      g_SIMULATION    => f_int2bool(g_SIMULATION))
    port map (
      clk_125m_pllref_p_i => clk_125m_pllref_p_i,
      clk_125m_pllref_n_i => clk_125m_pllref_n_i,
      clk_20m_vcxo_i      => clk_20m_vcxo_i,
      clk_125m_gtp_n_i    => clk_125m_gtp_n_i,
      clk_125m_gtp_p_i    => clk_125m_gtp_p_i,
      gn_rst_n_i          => gn_rst_n_i,
      gn_p2l_clk_n_i      => gn_p2l_clk_n_i,
      gn_p2l_clk_p_i      => gn_p2l_clk_p_i,
      gn_p2l_rdy_o        => gn_p2l_rdy_o,
      gn_p2l_dframe_i     => gn_p2l_dframe_i,
      gn_p2l_valid_i      => gn_p2l_valid_i,
      gn_p2l_data_i       => gn_p2l_data_i,
      gn_p_wr_req_i       => gn_p_wr_req_i,
      gn_p_wr_rdy_o       => gn_p_wr_rdy_o,
      gn_rx_error_o       => gn_rx_error_o,
      gn_l2p_clk_n_o      => gn_l2p_clk_n_o,
      gn_l2p_clk_p_o      => gn_l2p_clk_p_o,
      gn_l2p_dframe_o     => gn_l2p_dframe_o,
      gn_l2p_valid_o      => gn_l2p_valid_o,
      gn_l2p_edb_o        => gn_l2p_edb_o,
      gn_l2p_data_o       => gn_l2p_data_o,
      gn_l2p_rdy_i        => gn_l2p_rdy_i,
      gn_l_wr_rdy_i       => gn_l_wr_rdy_i,
      gn_p_rd_d_rdy_i     => gn_p_rd_d_rdy_i,
      gn_tx_error_i       => gn_tx_error_i,
      gn_vc_rdy_i         => gn_vc_rdy_i,
      gn_gpio_b           => gn_gpio_b,
      fmc0_scl_b          => fmc0_scl_b,
      fmc0_sda_b          => fmc0_sda_b,
      fmc0_prsnt_m2c_n_i  => fmc0_prsnt_m2c_n_i,
      onewire_b           => onewire_b,
      spi_sclk_o          => spi_sclk_o,
      spi_ncs_o           => spi_ncs_o,
      spi_mosi_o          => spi_mosi_o,
      spi_miso_i          => spi_miso_i,
      pcbrev_i            => pcbrev_i,
      led_act_o           => led_act_o,
      led_link_o          => led_link_o,
      button1_n_i         => button1_n_i,
      uart_rxd_i          => uart_rxd_i,
      uart_txd_o          => uart_txd_o,
      plldac_sclk_o       => plldac_sclk_o,
      plldac_din_o        => plldac_din_o,
      pll25dac_cs_n_o     => pll25dac_cs_n_o,
      pll20dac_cs_n_o     => pll20dac_cs_n_o,
      sfp_txp_o           => sfp_txp_o,
      sfp_txn_o           => sfp_txn_o,
      sfp_rxp_i           => sfp_rxp_i,
      sfp_rxn_i           => sfp_rxn_i,
      sfp_mod_def0_i      => sfp_mod_def0_i,
      sfp_mod_def1_b      => sfp_mod_def1_b,
      sfp_mod_def2_b      => sfp_mod_def2_b,
      sfp_rate_select_o   => sfp_rate_select_o,
      sfp_tx_fault_i      => sfp_tx_fault_i,
      sfp_tx_disable_o    => sfp_tx_disable_o,
      sfp_los_i           => sfp_los_i,
      ddr_a_o             => ddr_a_o,
      ddr_ba_o            => ddr_ba_o,
      ddr_cas_n_o         => ddr_cas_n_o,
      ddr_ck_n_o          => ddr_ck_n_o,
      ddr_ck_p_o          => ddr_ck_p_o,
      ddr_cke_o           => ddr_cke_o,
      ddr_dq_b            => ddr_dq_b,
      ddr_ldm_o           => ddr_ldm_o,
      ddr_ldqs_n_b        => ddr_ldqs_n_b,
      ddr_ldqs_p_b        => ddr_ldqs_p_b,
      ddr_odt_o           => ddr_odt_o,
      ddr_ras_n_o         => ddr_ras_n_o,
      ddr_reset_n_o       => ddr_reset_n_o,
      ddr_rzq_b           => ddr_rzq_b,
      ddr_udm_o           => ddr_udm_o,
      ddr_udqs_n_b        => ddr_udqs_n_b,
      ddr_udqs_p_b        => ddr_udqs_p_b,
      ddr_we_n_o          => ddr_we_n_o,
      ddr_dma_clk_i       => clk_ref_125m,
      ddr_dma_rst_n_i     => rst_ref_125m_n,
      ddr_dma_wb_cyc_i    => fmc_wb_ddr_out.cyc,
      ddr_dma_wb_stb_i    => fmc_wb_ddr_out.stb,
      ddr_dma_wb_adr_i    => fmc_wb_ddr_out.adr,
      ddr_dma_wb_sel_i    => fmc_wb_ddr_out.sel,
      ddr_dma_wb_we_i     => fmc_wb_ddr_out.we,
      ddr_dma_wb_dat_i    => fmc_wb_ddr_out.dat,
      ddr_dma_wb_ack_o    => fmc_wb_ddr_in.ack,
      ddr_dma_wb_stall_o  => fmc_wb_ddr_in.stall,
      ddr_dma_wb_dat_o    => fmc_wb_ddr_in.dat,
      ddr_wr_fifo_empty_o => ddr_wr_fifo_empty,
      clk_62m5_sys_o      => clk_sys_62m5,
      rst_62m5_sys_n_o    => rst_sys_62m5_n,
      clk_125m_ref_o      => clk_ref_125m,
      rst_125m_ref_n_o    => rst_ref_125m_n,
      irq_user_i          => irq_vector,
      tm_link_up_o        => tm_link_up,
      tm_time_valid_o     => tm_time_valid,
      tm_tai_o            => tm_tai,
      tm_cycles_o         => tm_cycles,
      pps_p_o             => open,
      pps_led_o           => pps_led,
      link_ok_o           => wrabbit_en,
      app_wb_o            => cnx_master_out,
      app_wb_i            => cnx_master_in);

  fmc_wb_ddr_in.err <= '0';
  fmc_wb_ddr_in.rty <= '0';

  ------------------------------------------------------------------------------
  -- Primary wishbone crossbar
  ------------------------------------------------------------------------------

  cmp_crossbar : entity work.spec_ref_fmc_adc_100m_mmap
    port map (
      rst_n_i             => rst_sys_62m5_n,
      clk_i               => clk_sys_62m5,
      wb_i                => cnx_master_out,
      wb_o                => cnx_master_in,
      metadata_i          => cnx_slave_out(c_WB_SLAVE_METADATA),
      metadata_o          => cnx_slave_in(c_WB_SLAVE_METADATA),
      fmc_adc_mezzanine_i => cnx_slave_out(c_WB_SLAVE_FMC_ADC),
      fmc_adc_mezzanine_o => cnx_slave_in(c_WB_SLAVE_FMC_ADC));

  ------------------------------------------------------------------------------
  -- Application-specific metadata ROM
  ------------------------------------------------------------------------------

  cmp_xwb_metadata : entity work.xwb_metadata
    generic map (
      g_VENDOR_ID    => x"0000_10DC",
      g_DEVICE_ID    => x"4144_4301", -- "ADC1"
      g_VERSION      => x"0500_0000",
      g_CAPABILITIES => x"0000_0000",
      g_COMMIT_ID    => (others => '0'))
    port map (
      clk_i   => clk_sys_62m5,
      rst_n_i => rst_sys_62m5_n,
      wb_i    => cnx_slave_in(c_WB_SLAVE_METADATA),
      wb_o    => cnx_slave_out(c_WB_SLAVE_METADATA));

  ------------------------------------------------------------------------------
  -- FMC ADC mezzanines (wb bridge with cross-clocking)
  --    Mezzanine system managment I2C master
  --    Mezzanine SPI master
  --    Mezzanine I2C
  --    ADC core
  --    Mezzanine 1-wire master
  ------------------------------------------------------------------------------

  cmp_xwb_clock_bridge : xwb_clock_bridge
    generic map (
      g_SLAVE_PORT_WB_MODE  => CLASSIC,
      g_MASTER_PORT_WB_MODE => PIPELINED)
    port map (
      slave_clk_i    => clk_sys_62m5,
      slave_rst_n_i  => rst_sys_62m5_n,
      slave_i        => cnx_slave_in(c_WB_SLAVE_FMC_ADC),
      slave_o        => cnx_slave_out(c_WB_SLAVE_FMC_ADC),
      master_clk_i   => clk_ref_125m,
      master_rst_n_i => rst_ref_125m_n,
      master_i       => cnx_fmc_sync_master_in,
      master_o       => cnx_fmc_sync_master_out);

  cmp_tm_time_valid_sync : gc_sync
    port map (
      clk_i     => clk_ref_125m,
      rst_n_a_i => '1',
      d_i       => tm_time_valid,
      q_o       => tm_time_valid_sync);

  cmp_fmc_ddr_wr_fifo_sync : gc_sync
    port map (
      clk_i     => clk_ref_125m,
      rst_n_a_i => '1',
      d_i       => ddr_wr_fifo_empty,
      q_o       => ddr_wr_fifo_empty_sync);

  cmp_fmc_irq_sync : gc_sync
    port map (
      clk_i     => clk_sys_62m5,
      rst_n_a_i => '1',
      d_i       => fmc_irq,
      q_o       => irq_vector(0));

  cmp_fmc_adc_mezzanine : fmc_adc_mezzanine
    generic map (
      g_MULTISHOT_RAM_SIZE => g_MULTISHOT_RAM_SIZE,
      g_SPARTAN6_USE_PLL   => FALSE,
      g_FMC_ADC_NR         => 0,
      g_WB_MODE            => PIPELINED,
      g_WB_GRANULARITY     => BYTE)
    port map (
      sys_clk_i   => clk_ref_125m,
      sys_rst_n_i => rst_ref_125m_n,

      wb_csr_slave_i => cnx_fmc_sync_master_out,
      wb_csr_slave_o => cnx_fmc_sync_master_in,

      wb_ddr_clk_i    => clk_ref_125m,
      wb_ddr_rst_n_i  => rst_ref_125m_n,
      wb_ddr_master_i => fmc_wb_ddr_in,
      wb_ddr_master_o => fmc_wb_ddr_out,

      ddr_wr_fifo_empty_i => ddr_wr_fifo_empty_sync,
      trig_irq_o          => open,
      acq_end_irq_o       => open,
      eic_irq_o           => fmc_irq,
      acq_cfg_ok_o        => fmc_acq_cfg_ok,

      ext_trigger_p_i => adc_ext_trigger_p_i,
      ext_trigger_n_i => adc_ext_trigger_n_i,

      adc_dco_p_i  => adc_dco_p_i,
      adc_dco_n_i  => adc_dco_n_i,
      adc_fr_p_i   => adc_fr_p_i,
      adc_fr_n_i   => adc_fr_n_i,
      adc_outa_p_i => adc_outa_p_i,
      adc_outa_n_i => adc_outa_n_i,
      adc_outb_p_i => adc_outb_p_i,
      adc_outb_n_i => adc_outb_n_i,

      gpio_dac_clr_n_o => adc_gpio_dac_clr_n_o,
      gpio_led_acq_o   => adc_gpio_led_acq_o,
      gpio_led_trig_o  => adc_gpio_led_trig_o,
      gpio_ssr_ch1_o   => adc_gpio_ssr_ch1_o,
      gpio_ssr_ch2_o   => adc_gpio_ssr_ch2_o,
      gpio_ssr_ch3_o   => adc_gpio_ssr_ch3_o,
      gpio_ssr_ch4_o   => adc_gpio_ssr_ch4_o,
      gpio_si570_oe_o  => adc_gpio_si570_oe_o,

      spi_din_i       => adc_spi_din_i,
      spi_dout_o      => adc_spi_dout_o,
      spi_sck_o       => adc_spi_sck_o,
      spi_cs_adc_n_o  => adc_spi_cs_adc_n_o,
      spi_cs_dac1_n_o => adc_spi_cs_dac1_n_o,
      spi_cs_dac2_n_o => adc_spi_cs_dac2_n_o,
      spi_cs_dac3_n_o => adc_spi_cs_dac3_n_o,
      spi_cs_dac4_n_o => adc_spi_cs_dac4_n_o,

      si570_scl_b => adc_si570_scl_b,
      si570_sda_b => adc_si570_sda_b,

      mezz_one_wire_b => adc_one_wire_b,

      wr_tm_link_up_i    => tm_link_up,
      wr_tm_time_valid_i => tm_time_valid_sync,
      wr_tm_tai_i        => tm_tai,
      wr_tm_cycles_i     => tm_cycles,
      wr_enable_i        => wrabbit_en
      );

  ------------------------------------------------------------------------------
  -- Carrier LEDs
  ------------------------------------------------------------------------------

  cmp_pci_access_led : gc_extend_pulse
    generic map (
      g_width => 2500000)
    port map (
      clk_i      => clk_sys_62m5,
      rst_n_i    => rst_sys_62m5_n,
      pulse_i    => cnx_master_out.cyc,
      extended_o => gn4124_access);

  aux_leds_o(0) <= not gn4124_access;
  aux_leds_o(1) <= not fmc_acq_cfg_ok;
  aux_leds_o(2) <= not tm_time_valid;
  aux_leds_o(3) <= not pps_led;

end architecture arch;
