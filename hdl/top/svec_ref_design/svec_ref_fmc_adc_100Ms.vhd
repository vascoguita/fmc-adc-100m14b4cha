-------------------------------------------------------------------------------
-- SPDX-License-Identifier: CERN-OHL-W-2.0+
-- Title      : FMC ADC 100Ms/s SVEC top-level
-- Project    : FMC ADC 100M 14B 4CHA gateware
-- URL        : http://www.ohwr.org/projects/fmc-adc-100m14b4cha-gw
-------------------------------------------------------------------------------
-- File       : svec_ref_fmc_adc_100Ms.vhd
-- Company    : CERN (BE-CO-HT)
-- Created    : 2013-07-04
-- Standard   : VHDL'93/02
-------------------------------------------------------------------------------
-- Description: Top entity of FMC ADC 100Ms/s design for Simple VME FMC
-- Carrier (SVEC). See also: http://www.ohwr.org/projects/svec
-------------------------------------------------------------------------------
-- Copyright (c) 2013-2020 CERN (BE-CO-HT)
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

entity svec_ref_fmc_adc_100Ms is
  generic(
    g_SIMULATION         : integer := 0;
    g_NB_FMC_SLOTS       : natural := 2;
    g_MULTISHOT_RAM_SIZE : natural := 8192;
    g_WRPC_INITF         : string  := "../../ip_cores/wr-cores/bin/wrpc/wrc_phy8.bram");
  port
    (
      -- Reset from system fpga
      rst_n_i : in std_logic;

      -- Local oscillators
      clk_20m_vcxo_i : in std_logic;              -- 20MHz VCXO clock

      clk_125m_pllref_p_i : in std_logic;         -- 125 MHz PLL reference
      clk_125m_pllref_n_i : in std_logic;

      clk_125m_gtp_n_i : in std_logic;            -- 125 MHz GTP reference
      clk_125m_gtp_p_i : in std_logic;

      -- DAC interface (20MHz and 25MHz VCXO)
      pll20dac_din_o    : out std_logic;
      pll20dac_sclk_o   : out std_logic;
      pll20dac_sync_n_o : out std_logic;
      pll25dac_din_o    : out std_logic;
      pll25dac_sclk_o   : out std_logic;
      pll25dac_sync_n_o : out std_logic;

      -- Carrier front panel LEDs
      fp_led_line_oen_o : out std_logic_vector(1 downto 0);
      fp_led_line_o     : out std_logic_vector(1 downto 0);
      fp_led_column_o   : out std_logic_vector(3 downto 0);

      -- Carrier I2C eeprom
      carrier_scl_b : inout std_logic;
      carrier_sda_b : inout std_logic;

      -- PCB revision
      pcbrev_i : in std_logic_vector(4 downto 0);

      -- Carrier 1-wire interface (DS18B20 thermometer + unique ID)
      onewire_b : inout std_logic;

      -- SFP
      sfp_txp_o         : out   std_logic;
      sfp_txn_o         : out   std_logic;
      sfp_rxp_i         : in    std_logic;
      sfp_rxn_i         : in    std_logic;
      sfp_mod_def0_i    : in    std_logic;  -- sfp detect
      sfp_mod_def1_b    : inout std_logic;  -- scl
      sfp_mod_def2_b    : inout std_logic;  -- sda
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

      -- GPIO
      fp_gpio1_b      : out std_logic;  -- PPS output
      fp_gpio2_b      : out std_logic;  -- not used
      fp_gpio3_b      : in  std_logic;  -- ext 10MHz clock input
      fp_gpio4_b      : in  std_logic;  -- ext PPS input
      fp_term_en_o    : out std_logic_vector(4 downto 1);
      fp_gpio1_a2b_o  : out std_logic;
      fp_gpio2_a2b_o  : out std_logic;
      fp_gpio34_a2b_o : out std_logic;

      ------------------------------------------
      -- VME interface
      ------------------------------------------
      vme_write_n_i    : in    std_logic;
      vme_sysreset_n_i : in    std_logic;
      vme_retry_oe_o   : out   std_logic;
      vme_retry_n_o    : out   std_logic;
      vme_lword_n_b    : inout std_logic;
      vme_iackout_n_o  : out   std_logic;
      vme_iackin_n_i   : in    std_logic;
      vme_iack_n_i     : in    std_logic;
      vme_gap_i        : in    std_logic;
      vme_dtack_oe_o   : out   std_logic;
      vme_dtack_n_o    : out   std_logic;
      vme_ds_n_i       : in    std_logic_vector(1 downto 0);
      vme_data_oe_n_o  : out   std_logic;
      vme_data_dir_o   : out   std_logic;
      vme_berr_o       : out   std_logic;
      vme_as_n_i       : in    std_logic;
      vme_addr_oe_n_o  : out   std_logic;
      vme_addr_dir_o   : out   std_logic;
      vme_irq_o        : out   std_logic_vector(7 downto 1);
      vme_ga_i         : in    std_logic_vector(4 downto 0);
      vme_data_b       : inout std_logic_vector(31 downto 0);
      vme_am_i         : in    std_logic_vector(5 downto 0);
      vme_addr_b       : inout std_logic_vector(31 downto 1);

      ------------------------------------------
      -- DDR (banks 4 and 5)
      ------------------------------------------
      ddr4_a_o       : out   std_logic_vector(13 downto 0);
      ddr4_ba_o      : out   std_logic_vector(2 downto 0);
      ddr4_cas_n_o   : out   std_logic;
      ddr4_ck_n_o    : out   std_logic;
      ddr4_ck_p_o    : out   std_logic;
      ddr4_cke_o     : out   std_logic;
      ddr4_dq_b      : inout std_logic_vector(15 downto 0);
      ddr4_ldm_o     : out   std_logic;
      ddr4_ldqs_n_b  : inout std_logic;
      ddr4_ldqs_p_b  : inout std_logic;
      ddr4_odt_o     : out   std_logic;
      ddr4_ras_n_o   : out   std_logic;
      ddr4_reset_n_o : out   std_logic;
      ddr4_rzq_b     : inout std_logic;
      ddr4_udm_o     : out   std_logic;
      ddr4_udqs_n_b  : inout std_logic;
      ddr4_udqs_p_b  : inout std_logic;
      ddr4_we_n_o    : out   std_logic;

      ddr5_a_o       : out   std_logic_vector(13 downto 0);
      ddr5_ba_o      : out   std_logic_vector(2 downto 0);
      ddr5_cas_n_o   : out   std_logic;
      ddr5_ck_n_o    : out   std_logic;
      ddr5_ck_p_o    : out   std_logic;
      ddr5_cke_o     : out   std_logic;
      ddr5_dq_b      : inout std_logic_vector(15 downto 0);
      ddr5_ldm_o     : out   std_logic;
      ddr5_ldqs_n_b  : inout std_logic;
      ddr5_ldqs_p_b  : inout std_logic;
      ddr5_odt_o     : out   std_logic;
      ddr5_ras_n_o   : out   std_logic;
      ddr5_reset_n_o : out   std_logic;
      ddr5_rzq_b     : inout std_logic;
      ddr5_udm_o     : out   std_logic;
      ddr5_udqs_n_b  : inout std_logic;
      ddr5_udqs_p_b  : inout std_logic;
      ddr5_we_n_o    : out   std_logic;

      ------------------------------------------
      -- FMC slots
      ------------------------------------------
      adc_ext_trigger_p_i : in std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);  -- External trigger
      adc_ext_trigger_n_i : in std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);

      adc_dco_p_i  : in std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);  -- ADC data clock
      adc_dco_n_i  : in std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);
      adc_fr_p_i   : in std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);  -- ADC frame start
      adc_fr_n_i   : in std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);
      adc_outa_p_i : in std_logic_vector(4*g_NB_FMC_SLOTS-1 downto 0);  -- ADC serial data (odd bits)
      adc_outa_n_i : in std_logic_vector(4*g_NB_FMC_SLOTS-1 downto 0);
      adc_outb_p_i : in std_logic_vector(4*g_NB_FMC_SLOTS-1 downto 0);  -- ADC serial data (even bits)
      adc_outb_n_i : in std_logic_vector(4*g_NB_FMC_SLOTS-1 downto 0);

      adc_spi_din_i       : in  std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);  -- SPI data from FMC
      adc_spi_dout_o      : out std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);  -- SPI data to FMC
      adc_spi_sck_o       : out std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);  -- SPI clock
      adc_spi_cs_adc_n_o  : out std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);  -- SPI ADC chip select (active low)
      adc_spi_cs_dac1_n_o : out std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);  -- SPI channel 1 offset DAC chip select (active low)
      adc_spi_cs_dac2_n_o : out std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);  -- SPI channel 2 offset DAC chip select (active low)
      adc_spi_cs_dac3_n_o : out std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);  -- SPI channel 3 offset DAC chip select (active low)
      adc_spi_cs_dac4_n_o : out std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);  -- SPI channel 4 offset DAC chip select (active low)

      adc_gpio_dac_clr_n_o : out std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);  -- offset DACs clear (active low)
      adc_gpio_led_acq_o   : out std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);  -- Mezzanine front panel power LED (PWR)
      adc_gpio_led_trig_o  : out std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);  -- Mezzanine front panel trigger LED (TRIG)
      adc_gpio_ssr_ch1_o   : out std_logic_vector(7*g_NB_FMC_SLOTS-1 downto 0);  -- Channel 1 solid state relays control
      adc_gpio_ssr_ch2_o   : out std_logic_vector(7*g_NB_FMC_SLOTS-1 downto 0);  -- Channel 2 solid state relays control
      adc_gpio_ssr_ch3_o   : out std_logic_vector(7*g_NB_FMC_SLOTS-1 downto 0);  -- Channel 3 solid state relays control
      adc_gpio_ssr_ch4_o   : out std_logic_vector(7*g_NB_FMC_SLOTS-1 downto 0);  -- Channel 4 solid state relays control
      adc_gpio_si570_oe_o  : out std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);  -- Si570 (programmable oscillator) output enable

      adc_si570_scl_b : inout std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);  -- I2C bus clock (Si570)
      adc_si570_sda_b : inout std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);  -- I2C bus data (Si570)

      adc_one_wire_b : inout std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);  -- Mezzanine 1-wire interface (DS18B20 thermometer + unique ID)

      ------------------------------------------
      -- FMC slot management
      ------------------------------------------
      fmc0_prsnt_m2c_n_i : in std_logic;
      fmc1_prsnt_m2c_n_i : in std_logic;

      fmc0_scl_b : inout std_logic;
      fmc0_sda_b : inout std_logic;
      fmc1_scl_b : inout std_logic;
      fmc1_sda_b : inout std_logic);

end svec_ref_fmc_adc_100Ms;


architecture arch of svec_ref_fmc_adc_100Ms is

  ------------------------------------------------------------------------------
  -- Constants declaration
  ------------------------------------------------------------------------------

  -- Number of slaves on the wishbone crossbar
  constant c_NUM_WB_SLAVES : integer := 3;

  -- Wishbone slave(s)
  constant c_WB_SLAVE_METADATA : integer := 0;
  constant c_WB_SLAVE_FMC0_ADC : integer := 1;  -- FMC slot 1 ADC mezzanine
  constant c_WB_SLAVE_FMC1_ADC : integer := 2;  -- FMC slot 2 ADC mezzanine

  -- Convention metadata base address
  constant c_METADATA_ADDR : t_wishbone_address := x"0000_4000";

  ------------------------------------------------------------------------------
  -- Signals declaration
  ------------------------------------------------------------------------------

  subtype t_fmc_slot_vec is std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);

  -- Clocks and resets
  signal clk_sys_62m5 : std_logic;
  signal clk_ref_125m : std_logic;
  signal clk_ext_ref  : std_logic;

  signal rst_sys_62m5_n : std_logic := '0';
  signal rst_ref_125m_n : std_logic := '0';
  signal areset_n       : std_logic := '0';

  -- Wishbone buse(s) from master(s) to crossbar slave port(s)
  signal cnx_master_out : t_wishbone_master_out;
  signal cnx_master_in  : t_wishbone_master_in;

  -- Wishbone buse(s) from crossbar master port(s) to slave(s)
  signal cnx_slave_out : t_wishbone_slave_out_array(c_NUM_WB_SLAVES-1 downto 0);
  signal cnx_slave_in  : t_wishbone_slave_in_array(c_NUM_WB_SLAVES-1 downto 0);

  -- Wishbone bus from cross-clocking module to FMC mezzanines
  signal cnx_fmc_sync_master_out : t_wishbone_master_out_array(g_NB_FMC_SLOTS-1 downto 0);
  signal cnx_fmc_sync_master_in  : t_wishbone_master_in_array(g_NB_FMC_SLOTS-1 downto 0);

  -- Wishbone buses from FMC ADC cores to DDR controller
  signal fmc_wb_ddr_in  : t_wishbone_master_data64_in_array(g_NB_FMC_SLOTS-1 downto 0);
  signal fmc_wb_ddr_out : t_wishbone_master_data64_out_array(g_NB_FMC_SLOTS-1 downto 0);

  type t_fmc_acq_led is array (0 to g_NB_FMC_SLOTS-1) of std_logic_vector(1 downto 0);

  -- Interrupts and status
  signal ddr_wr_fifo_empty      : t_fmc_slot_vec;
  signal ddr_wr_fifo_empty_sync : t_fmc_slot_vec;
  signal fmc_irq                : t_fmc_slot_vec;
  signal fmc_acq_trig           : t_fmc_slot_vec;
  signal fmc_acq_trig_sync      : t_fmc_slot_vec;
  signal fmc_acq_cfg_ok         : t_fmc_slot_vec;
  signal fmc_acq_cfg_ok_sync    : t_fmc_slot_vec;
  signal fmc_acq_led            : t_fmc_acq_led;
  signal irq_vector             : t_fmc_slot_vec;
  signal vme_access             : std_logic;

  -- Front panel LED control
  signal svec_led : std_logic_vector(15 downto 0);

  -- WR PTP core timing interface
  signal tm_link_up         : std_logic;
  signal tm_tai             : std_logic_vector(39 downto 0);
  signal tm_cycles          : std_logic_vector(27 downto 0);
  signal tm_time_valid      : std_logic;
  signal tm_time_valid_sync : std_logic;
  signal wrabbit_en         : std_logic;
  signal pps                : std_logic;
  signal pps_led            : std_logic;
  signal pps_ext_in         : std_logic;
  signal wr_led_link        : std_logic;
  signal wr_led_act         : std_logic;

begin -- architecture arch

  areset_n <= vme_sysreset_n_i and rst_n_i;

  inst_svec_base : entity work.svec_base_wr
    generic map (
      g_DECODE_AM     => FALSE,
      g_WITH_VIC      => TRUE,
      g_WITH_ONEWIRE  => FALSE,
      g_WITH_SPI      => FALSE,
      g_WITH_WR       => TRUE,
      g_WITH_DDR4     => TRUE,
      g_WITH_DDR5     => TRUE,
      g_APP_OFFSET    => c_METADATA_ADDR,
      g_NUM_USER_IRQ  => 2,
      g_DPRAM_INITF   => g_WRPC_INITF,
      g_AUX_CLKS      => 0,
      g_FABRIC_IFACE  => plain,
      g_SIMULATION    => g_SIMULATION)
    port map (
      rst_n_i              => areset_n,
      clk_125m_pllref_p_i  => clk_125m_pllref_p_i,
      clk_125m_pllref_n_i  => clk_125m_pllref_n_i,
      clk_20m_vcxo_i       => clk_20m_vcxo_i,
      clk_125m_gtp_n_i     => clk_125m_gtp_n_i,
      clk_125m_gtp_p_i     => clk_125m_gtp_p_i,
      clk_10m_ext_i        => clk_ext_ref,
      pps_ext_i            => pps_ext_in,
      vme_write_n_i        => vme_write_n_i,
      vme_sysreset_n_i     => vme_sysreset_n_i,
      vme_retry_oe_o       => vme_retry_oe_o,
      vme_retry_n_o        => vme_retry_n_o,
      vme_lword_n_b        => vme_lword_n_b,
      vme_iackout_n_o      => vme_iackout_n_o,
      vme_iackin_n_i       => vme_iackin_n_i,
      vme_iack_n_i         => vme_iack_n_i,
      vme_gap_i            => vme_gap_i,
      vme_dtack_oe_o       => vme_dtack_oe_o,
      vme_dtack_n_o        => vme_dtack_n_o,
      vme_ds_n_i           => vme_ds_n_i,
      vme_data_oe_n_o      => vme_data_oe_n_o,
      vme_data_dir_o       => vme_data_dir_o,
      vme_berr_o           => vme_berr_o,
      vme_as_n_i           => vme_as_n_i,
      vme_addr_oe_n_o      => vme_addr_oe_n_o,
      vme_addr_dir_o       => vme_addr_dir_o,
      vme_irq_o            => vme_irq_o,
      vme_ga_i             => vme_ga_i,
      vme_data_b           => vme_data_b,
      vme_am_i             => vme_am_i,
      vme_addr_b           => vme_addr_b,
      fmc0_scl_b           => fmc0_scl_b,
      fmc0_sda_b           => fmc0_sda_b,
      fmc1_scl_b           => fmc1_scl_b,
      fmc1_sda_b           => fmc1_sda_b,
      fmc0_prsnt_m2c_n_i   => fmc0_prsnt_m2c_n_i,
      fmc1_prsnt_m2c_n_i   => fmc1_prsnt_m2c_n_i,
      onewire_b            => onewire_b,
      carrier_scl_b        => carrier_scl_b,
      carrier_sda_b        => carrier_sda_b,
      spi_sclk_o           => spi_sclk_o,
      spi_ncs_o            => spi_ncs_o,
      spi_mosi_o           => spi_mosi_o,
      spi_miso_i           => spi_miso_i,
      uart_rxd_i           => uart_rxd_i,
      uart_txd_o           => uart_txd_o,
      plldac_sclk_o        => pll20dac_sclk_o,
      plldac_din_o         => pll20dac_din_o,
      pll20dac_din_o       => pll20dac_din_o,
      pll20dac_sclk_o      => pll20dac_sclk_o,
      pll20dac_sync_n_o    => pll20dac_sync_n_o,
      pll25dac_din_o       => pll25dac_din_o,
      pll25dac_sclk_o      => pll25dac_sclk_o,
      pll25dac_sync_n_o    => pll25dac_sync_n_o,
      sfp_txp_o            => sfp_txp_o,
      sfp_txn_o            => sfp_txn_o,
      sfp_rxp_i            => sfp_rxp_i,
      sfp_rxn_i            => sfp_rxn_i,
      sfp_mod_def0_i       => sfp_mod_def0_i,
      sfp_mod_def1_b       => sfp_mod_def1_b,
      sfp_mod_def2_b       => sfp_mod_def2_b,
      sfp_rate_select_o    => sfp_rate_select_o,
      sfp_tx_fault_i       => sfp_tx_fault_i,
      sfp_tx_disable_o     => sfp_tx_disable_o,
      sfp_los_i            => sfp_los_i,
      ddr4_a_o             => ddr4_a_o,
      ddr4_ba_o            => ddr4_ba_o,
      ddr4_cas_n_o         => ddr4_cas_n_o,
      ddr4_ck_n_o          => ddr4_ck_n_o,
      ddr4_ck_p_o          => ddr4_ck_p_o,
      ddr4_cke_o           => ddr4_cke_o,
      ddr4_dq_b            => ddr4_dq_b,
      ddr4_ldm_o           => ddr4_ldm_o,
      ddr4_ldqs_n_b        => ddr4_ldqs_n_b,
      ddr4_ldqs_p_b        => ddr4_ldqs_p_b,
      ddr4_odt_o           => ddr4_odt_o,
      ddr4_ras_n_o         => ddr4_ras_n_o,
      ddr4_reset_n_o       => ddr4_reset_n_o,
      ddr4_rzq_b           => ddr4_rzq_b,
      ddr4_udm_o           => ddr4_udm_o,
      ddr4_udqs_n_b        => ddr4_udqs_n_b,
      ddr4_udqs_p_b        => ddr4_udqs_p_b,
      ddr4_we_n_o          => ddr4_we_n_o,
      ddr5_a_o             => ddr5_a_o,
      ddr5_ba_o            => ddr5_ba_o,
      ddr5_cas_n_o         => ddr5_cas_n_o,
      ddr5_ck_n_o          => ddr5_ck_n_o,
      ddr5_ck_p_o          => ddr5_ck_p_o,
      ddr5_cke_o           => ddr5_cke_o,
      ddr5_dq_b            => ddr5_dq_b,
      ddr5_ldm_o           => ddr5_ldm_o,
      ddr5_ldqs_n_b        => ddr5_ldqs_n_b,
      ddr5_ldqs_p_b        => ddr5_ldqs_p_b,
      ddr5_odt_o           => ddr5_odt_o,
      ddr5_ras_n_o         => ddr5_ras_n_o,
      ddr5_reset_n_o       => ddr5_reset_n_o,
      ddr5_rzq_b           => ddr5_rzq_b,
      ddr5_udm_o           => ddr5_udm_o,
      ddr5_udqs_n_b        => ddr5_udqs_n_b,
      ddr5_udqs_p_b        => ddr5_udqs_p_b,
      ddr5_we_n_o          => ddr5_we_n_o,
      pcbrev_i             => pcbrev_i,
      ddr4_clk_i           => clk_ref_125m,
      ddr4_rst_n_i         => rst_ref_125m_n,
      ddr4_wb_i            => fmc_wb_ddr_out(0),
      ddr4_wb_o            => fmc_wb_ddr_in(0),
      ddr5_clk_i           => clk_ref_125m,
      ddr5_rst_n_i         => rst_ref_125m_n,
      ddr5_wb_i            => fmc_wb_ddr_out(1),
      ddr5_wb_o            => fmc_wb_ddr_in(1),
      ddr4_wr_fifo_empty_o => ddr_wr_fifo_empty(0),
      ddr5_wr_fifo_empty_o => ddr_wr_fifo_empty(1),
      clk_sys_62m5_o       => clk_sys_62m5,
      rst_sys_62m5_n_o     => rst_sys_62m5_n,
      clk_ref_125m_o       => clk_ref_125m,
      rst_ref_125m_n_o     => rst_ref_125m_n,
      irq_user_i           => irq_vector,
      tm_link_up_o         => tm_link_up,
      tm_time_valid_o      => tm_time_valid,
      tm_tai_o             => tm_tai,
      tm_cycles_o          => tm_cycles,
      pps_p_o              => pps,
      pps_led_o            => pps_led,
      link_ok_o            => wrabbit_en,
      led_link_o           => wr_led_link,
      led_act_o            => wr_led_act,
      app_wb_o             => cnx_master_out,
      app_wb_i             => cnx_master_in);

  ------------------------------------------------------------------------------
  -- Primary wishbone crossbar
  ------------------------------------------------------------------------------
  cmp_crossbar : entity work.svec_ref_fmc_adc_100m_mmap
    port map (
      rst_n_i              => rst_sys_62m5_n,
      clk_i                => clk_sys_62m5,
      wb_i                 => cnx_master_out,
      wb_o                 => cnx_master_in,
      metadata_i           => cnx_slave_out(c_WB_SLAVE_METADATA),
      metadata_o           => cnx_slave_in(c_WB_SLAVE_METADATA),
      fmc1_adc_mezzanine_i => cnx_slave_out(c_WB_SLAVE_FMC0_ADC),
      fmc1_adc_mezzanine_o => cnx_slave_in(c_WB_SLAVE_FMC0_ADC),
      fmc2_adc_mezzanine_i => cnx_slave_out(c_WB_SLAVE_FMC1_ADC),
      fmc2_adc_mezzanine_o => cnx_slave_in(c_WB_SLAVE_FMC1_ADC));

  ------------------------------------------------------------------------------
  -- Application-specific metadata ROM
  ------------------------------------------------------------------------------

  cmp_xwb_metadata : entity work.xwb_metadata
    generic map (
      g_VENDOR_ID    => x"0000_10DC",
      g_DEVICE_ID    => x"4144_4302", -- "ADC2"
      g_VERSION      => x"0500_0002",
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

  cmp_tm_time_valid_sync : gc_sync
    port map (
      clk_i     => clk_ref_125m,
      rst_n_a_i => '1',
      d_i       => tm_time_valid,
      q_o       => tm_time_valid_sync);

  gen_fmc_mezzanine : for I in 0 to g_NB_FMC_SLOTS - 1 generate

    cmp_xwb_clock_bridge : xwb_clock_bridge
      generic map (
        g_SLAVE_PORT_WB_MODE  => CLASSIC,
        g_MASTER_PORT_WB_MODE => PIPELINED)
      port map (
        slave_clk_i    => clk_sys_62m5,
        slave_rst_n_i  => rst_sys_62m5_n,
        slave_i        => cnx_slave_in(c_WB_SLAVE_FMC0_ADC + I),
        slave_o        => cnx_slave_out(c_WB_SLAVE_FMC0_ADC + I),
        master_clk_i   => clk_ref_125m,
        master_rst_n_i => rst_ref_125m_n,
        master_i       => cnx_fmc_sync_master_in(I),
        master_o       => cnx_fmc_sync_master_out(I));

    cmp_fmc_ddr_wr_fifo_sync : gc_sync
      port map (
        clk_i     => clk_ref_125m,
        rst_n_a_i => '1',
        d_i       => ddr_wr_fifo_empty(I),
        q_o       => ddr_wr_fifo_empty_sync(I));

    cmp_fmc_irq_sync : gc_sync
      port map (
        clk_i     => clk_sys_62m5,
        rst_n_a_i => '1',
        d_i       => fmc_irq(I),
        q_o       => irq_vector(I));

    cmp_fmc_adc_mezzanine : entity work.fmc_adc_mezzanine
      generic map (
        g_MULTISHOT_RAM_SIZE => g_MULTISHOT_RAM_SIZE,
        g_SPARTAN6_USE_PLL   => TRUE,
        g_BYTE_SWAP          => TRUE,
        g_FMC_ADC_NR         => I,
        g_WB_MODE            => PIPELINED,
        g_WB_GRANULARITY     => BYTE)
      port map (
        sys_clk_i   => clk_ref_125m,
        sys_rst_n_i => rst_ref_125m_n,

        wb_csr_slave_i => cnx_fmc_sync_master_out(I),
        wb_csr_slave_o => cnx_fmc_sync_master_in(I),

        wb_ddr_clk_i    => clk_ref_125m,
        wb_ddr_rst_n_i  => rst_ref_125m_n,
        wb_ddr_master_i => fmc_wb_ddr_in(I),
        wb_ddr_master_o => fmc_wb_ddr_out(I),

        ddr_wr_fifo_empty_i => ddr_wr_fifo_empty_sync(I),
        trig_irq_o          => fmc_acq_trig(I),
        acq_end_irq_o       => open,
        eic_irq_o           => fmc_irq(I),
        acq_cfg_ok_o        => fmc_acq_cfg_ok(I),

        ext_trigger_p_i => adc_ext_trigger_p_i(I),
        ext_trigger_n_i => adc_ext_trigger_n_i(I),

        adc_dco_p_i  => adc_dco_p_i(I),
        adc_dco_n_i  => adc_dco_n_i(I),
        adc_fr_p_i   => adc_fr_p_i(I),
        adc_fr_n_i   => adc_fr_n_i(I),
        adc_outa_p_i => adc_outa_p_i(4*(I+1)-1 downto 4*I),
        adc_outa_n_i => adc_outa_n_i(4*(I+1)-1 downto 4*I),
        adc_outb_p_i => adc_outb_p_i(4*(I+1)-1 downto 4*I),
        adc_outb_n_i => adc_outb_n_i(4*(I+1)-1 downto 4*I),

        gpio_dac_clr_n_o => adc_gpio_dac_clr_n_o(I),
        gpio_led_acq_o   => adc_gpio_led_acq_o(I),
        gpio_led_trig_o  => adc_gpio_led_trig_o(I),
        gpio_ssr_ch1_o   => adc_gpio_ssr_ch1_o(7*(I+1)-1 downto 7*I),
        gpio_ssr_ch2_o   => adc_gpio_ssr_ch2_o(7*(I+1)-1 downto 7*I),
        gpio_ssr_ch3_o   => adc_gpio_ssr_ch3_o(7*(I+1)-1 downto 7*I),
        gpio_ssr_ch4_o   => adc_gpio_ssr_ch4_o(7*(I+1)-1 downto 7*I),
        gpio_si570_oe_o  => adc_gpio_si570_oe_o(I),

        spi_din_i       => adc_spi_din_i(I),
        spi_dout_o      => adc_spi_dout_o(I),
        spi_sck_o       => adc_spi_sck_o(I),
        spi_cs_adc_n_o  => adc_spi_cs_adc_n_o(I),
        spi_cs_dac1_n_o => adc_spi_cs_dac1_n_o(I),
        spi_cs_dac2_n_o => adc_spi_cs_dac2_n_o(I),
        spi_cs_dac3_n_o => adc_spi_cs_dac3_n_o(I),
        spi_cs_dac4_n_o => adc_spi_cs_dac4_n_o(I),

        si570_scl_b => adc_si570_scl_b(I),
        si570_sda_b => adc_si570_sda_b(I),

        mezz_one_wire_b => adc_one_wire_b(I),

        wr_tm_link_up_i    => tm_link_up,
        wr_tm_time_valid_i => tm_time_valid_sync,
        wr_tm_tai_i        => tm_tai,
        wr_tm_cycles_i     => tm_cycles,
        wr_enable_i        => wrabbit_en);

  end generate gen_fmc_mezzanine;

  ------------------------------------------------------------------------------
  -- Carrier front panel LEDs and LEMOs
  ------------------------------------------------------------------------------
  cmp_led_controller : gc_bicolor_led_ctrl
    generic map(
      g_nb_column    => 4,
      g_nb_line      => 2,
      g_clk_freq     => 62500000,    -- in Hz
      g_refresh_rate => 250)         -- in Hz
    port map(
      rst_n_i         => rst_sys_62m5_n,
      clk_i           => clk_sys_62m5,
      led_intensity_i => "1100100",  -- in %
      led_state_i     => svec_led,
      column_o        => fp_led_column_o,
      line_o          => fp_led_line_o,
      line_oen_o      => fp_led_line_oen_o);

  cmp_vme_access_led : gc_extend_pulse
    generic map (
      g_width => 2500000)
    port map (
      clk_i      => clk_sys_62m5,
      rst_n_i    => rst_sys_62m5_n,
      pulse_i    => cnx_master_out.cyc,
      extended_o => vme_access);

  gen_fmc_led : for I in 0 to g_NB_FMC_SLOTS - 1 generate

    cmp_fmc_cfg_ok_sync : gc_sync
      port map (
        clk_i     => clk_sys_62m5,
        rst_n_a_i => '1',
        d_i       => fmc_acq_cfg_ok(I),
        q_o       => fmc_acq_cfg_ok_sync(I));

    cmp_fmc_trig_sync : gc_sync
      port map (
        clk_i     => clk_sys_62m5,
        rst_n_a_i => '1',
        d_i       => fmc_acq_trig(I),
        q_o       => fmc_acq_trig_sync(I));

    p_fmc_acq_led: process (fmc_acq_cfg_ok_sync, fmc_acq_trig_sync) is
    begin
      if fmc_acq_cfg_ok_sync(I) = '0' then
        fmc_acq_led(I) <= c_LED_RED;
      elsif fmc_acq_trig_sync(I) = '1' then
        fmc_acq_led(I) <= c_LED_RED_GREEN;
      else
        fmc_acq_led(I) <= c_LED_GREEN;
      end if;
    end process p_fmc_acq_led;

  end generate gen_fmc_led;

  -- LED order on front panel (top to bottom)
  -- 1..0 |  9..8
  -- 3..2 | 11..10
  -- 5..4 | 13..12
  -- 7..6 | 15..14
  svec_led(1 downto 0)   <= c_LED_GREEN     when wr_led_link = '1'  else c_LED_RED;
  svec_led(3 downto 2)   <= fmc_acq_led(1);
  svec_led(5 downto 4)   <= c_LED_GREEN     when tm_time_valid = '1'else c_LED_RED;
  svec_led(7 downto 6)   <= c_LED_RED_GREEN when vme_access = '1'   else c_LED_OFF;
  svec_led(9 downto 8)   <= c_LED_RED_GREEN when wr_led_act = '1'   else c_LED_OFF;
  svec_led(11 downto 10) <= fmc_acq_led(0);
  svec_led(13 downto 12) <= c_LED_OFF;
  svec_led(15 downto 14) <= c_LED_GREEN     when pps_led = '1'      else c_LED_OFF;

  -- Front panel IO configuration
  fp_gpio1_b      <= pps;
  fp_gpio2_b      <= '0';
  clk_ext_ref     <= fp_gpio3_b;
  pps_ext_in      <= fp_gpio4_b;
  fp_term_en_o    <= (others => '0');
  fp_gpio1_a2b_o  <= '1';
  fp_gpio2_a2b_o  <= '1';
  fp_gpio34_a2b_o <= '0';

end architecture arch;
