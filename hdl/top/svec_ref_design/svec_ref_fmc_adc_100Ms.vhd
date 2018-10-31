-------------------------------------------------------------------------------
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
-- Copyright (c) 2013-2018 CERN (BE-CO-HT)
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

use work.ddr3_ctrl_pkg.all;
use work.gencores_pkg.all;
use work.wishbone_pkg.all;
use work.fmc_adc_mezzanine_pkg.all;
use work.synthesis_descriptor.all;
use work.vme64x_pkg.all;
use work.timetag_core_pkg.all;
use work.carrier_csr_wbgen2_pkg.all;
use work.wr_board_pkg.all;
use work.wr_svec_pkg.all;

entity svec_ref_fmc_adc_100Ms is
  generic(
    g_SIMULATION         : integer := 0;
    g_MULTISHOT_RAM_SIZE : natural := 8192;
    g_CALIB_SOFT_IP      : string  := "TRUE";
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
      carrier_onewire_b : inout std_logic;

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
      fp_gpio1_o      : out std_logic;  -- PPS output
      fp_gpio2_o      : out std_logic;  -- not used
      fp_gpio3_o      : out std_logic;  -- not used
      fp_gpio4_o      : out std_logic;  -- not used
      fp_term_en_o    : out std_logic_vector(4 downto 1);
      fp_gpio1_a2b_o  : out std_logic;
      fp_gpio2_a2b_o  : out std_logic;
      fp_gpio34_a2b_o : out std_logic;

      ------------------------------------------
      -- VME interface
      ------------------------------------------
      vme_write_n_i    : in    std_logic;
      vme_rst_n_i      : in    std_logic;
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
      -- DDR0 (bank 4)
      ------------------------------------------
      ddr0_we_n_o    : out   std_logic;
      ddr0_udqs_p_b  : inout std_logic;
      ddr0_udqs_n_b  : inout std_logic;
      ddr0_udm_o     : out   std_logic;
      ddr0_reset_n_o : out   std_logic;
      ddr0_ras_n_o   : out   std_logic;
      ddr0_odt_o     : out   std_logic;
      ddr0_ldqs_p_b  : inout std_logic;
      ddr0_ldqs_n_b  : inout std_logic;
      ddr0_ldm_o     : out   std_logic;
      ddr0_cke_o     : out   std_logic;
      ddr0_ck_p_o    : out   std_logic;
      ddr0_ck_n_o    : out   std_logic;
      ddr0_cas_n_o   : out   std_logic;
      ddr0_dq_b      : inout std_logic_vector(15 downto 0);
      ddr0_ba_o      : out   std_logic_vector(2 downto 0);
      ddr0_a_o       : out   std_logic_vector(13 downto 0);
      ddr0_rzq_b     : inout std_logic;

      ------------------------------------------
      -- DDR1 (bank 5)
      ------------------------------------------
      ddr1_we_n_o    : out   std_logic;
      ddr1_udqs_p_b  : inout std_logic;
      ddr1_udqs_n_b  : inout std_logic;
      ddr1_udm_o     : out   std_logic;
      ddr1_reset_n_o : out   std_logic;
      ddr1_ras_n_o   : out   std_logic;
      ddr1_odt_o     : out   std_logic;
      ddr1_ldqs_p_b  : inout std_logic;
      ddr1_ldqs_n_b  : inout std_logic;
      ddr1_ldm_o     : out   std_logic;
      ddr1_cke_o     : out   std_logic;
      ddr1_ck_p_o    : out   std_logic;
      ddr1_ck_n_o    : out   std_logic;
      ddr1_cas_n_o   : out   std_logic;
      ddr1_dq_b      : inout std_logic_vector(15 downto 0);
      ddr1_ba_o      : out   std_logic_vector(2 downto 0);
      ddr1_a_o       : out   std_logic_vector(13 downto 0);
      ddr1_rzq_b     : inout std_logic;

      ------------------------------------------
      -- FMC slot 0
      ------------------------------------------
      adc0_ext_trigger_p_i : in std_logic;        -- External trigger
      adc0_ext_trigger_n_i : in std_logic;

      adc0_dco_p_i  : in std_logic;                     -- ADC data clock
      adc0_dco_n_i  : in std_logic;
      adc0_fr_p_i   : in std_logic;                     -- ADC frame start
      adc0_fr_n_i   : in std_logic;
      adc0_outa_p_i : in std_logic_vector(3 downto 0);  -- ADC serial data (odd bits)
      adc0_outa_n_i : in std_logic_vector(3 downto 0);
      adc0_outb_p_i : in std_logic_vector(3 downto 0);  -- ADC serial data (even bits)
      adc0_outb_n_i : in std_logic_vector(3 downto 0);

      adc0_spi_din_i       : in  std_logic;       -- SPI data from FMC
      adc0_spi_dout_o      : out std_logic;       -- SPI data to FMC
      adc0_spi_sck_o       : out std_logic;       -- SPI clock
      adc0_spi_cs_adc_n_o  : out std_logic;       -- SPI ADC chip select (active low)
      adc0_spi_cs_dac1_n_o : out std_logic;  -- SPI channel 1 offset DAC chip select (active low)
      adc0_spi_cs_dac2_n_o : out std_logic;  -- SPI channel 2 offset DAC chip select (active low)
      adc0_spi_cs_dac3_n_o : out std_logic;  -- SPI channel 3 offset DAC chip select (active low)
      adc0_spi_cs_dac4_n_o : out std_logic;  -- SPI channel 4 offset DAC chip select (active low)

      adc0_gpio_dac_clr_n_o : out std_logic;      -- offset DACs clear (active low)
      adc0_gpio_led_acq_o   : out std_logic;      -- Mezzanine front panel power LED (PWR)
      adc0_gpio_led_trig_o  : out std_logic;      -- Mezzanine front panel trigger LED (TRIG)
      adc0_gpio_ssr_ch1_o   : out std_logic_vector(6 downto 0);  -- Channel 1 solid state relays control
      adc0_gpio_ssr_ch2_o   : out std_logic_vector(6 downto 0);  -- Channel 2 solid state relays control
      adc0_gpio_ssr_ch3_o   : out std_logic_vector(6 downto 0);  -- Channel 3 solid state relays control
      adc0_gpio_ssr_ch4_o   : out std_logic_vector(6 downto 0);  -- Channel 4 solid state relays control
      adc0_gpio_si570_oe_o  : out std_logic;      -- Si570 (programmable oscillator) output enable

      adc0_si570_scl_b : inout std_logic;         -- I2C bus clock (Si570)
      adc0_si570_sda_b : inout std_logic;         -- I2C bus data (Si570)

      adc0_one_wire_b : inout std_logic;  -- Mezzanine 1-wire interface (DS18B20 thermometer + unique ID)

      ------------------------------------------
      -- FMC slot 1
      ------------------------------------------
      adc1_ext_trigger_p_i : in std_logic;        -- External trigger
      adc1_ext_trigger_n_i : in std_logic;

      adc1_dco_p_i  : in std_logic;                     -- ADC data clock
      adc1_dco_n_i  : in std_logic;
      adc1_fr_p_i   : in std_logic;                     -- ADC frame start
      adc1_fr_n_i   : in std_logic;
      adc1_outa_p_i : in std_logic_vector(3 downto 0);  -- ADC serial data (odd bits)
      adc1_outa_n_i : in std_logic_vector(3 downto 0);
      adc1_outb_p_i : in std_logic_vector(3 downto 0);  -- ADC serial data (even bits)
      adc1_outb_n_i : in std_logic_vector(3 downto 0);

      adc1_spi_din_i       : in  std_logic;       -- SPI data from FMC
      adc1_spi_dout_o      : out std_logic;       -- SPI data to FMC
      adc1_spi_sck_o       : out std_logic;       -- SPI clock
      adc1_spi_cs_adc_n_o  : out std_logic;       -- SPI ADC chip select (active low)
      adc1_spi_cs_dac1_n_o : out std_logic;  -- SPI channel 1 offset DAC chip select (active low)
      adc1_spi_cs_dac2_n_o : out std_logic;  -- SPI channel 2 offset DAC chip select (active low)
      adc1_spi_cs_dac3_n_o : out std_logic;  -- SPI channel 3 offset DAC chip select (active low)
      adc1_spi_cs_dac4_n_o : out std_logic;  -- SPI channel 4 offset DAC chip select (active low)

      adc1_gpio_dac_clr_n_o : out std_logic;      -- offset DACs clear (active low)
      adc1_gpio_led_acq_o   : out std_logic;      -- Mezzanine front panel power LED (PWR)
      adc1_gpio_led_trig_o  : out std_logic;      -- Mezzanine front panel trigger LED (TRIG)
      adc1_gpio_ssr_ch1_o   : out std_logic_vector(6 downto 0);  -- Channel 1 solid state relays control
      adc1_gpio_ssr_ch2_o   : out std_logic_vector(6 downto 0);  -- Channel 2 solid state relays control
      adc1_gpio_ssr_ch3_o   : out std_logic_vector(6 downto 0);  -- Channel 3 solid state relays control
      adc1_gpio_ssr_ch4_o   : out std_logic_vector(6 downto 0);  -- Channel 4 solid state relays control
      adc1_gpio_si570_oe_o  : out std_logic;      -- Si570 (programmable oscillator) output enable

      adc1_si570_scl_b : inout std_logic;         -- I2C bus clock (Si570)
      adc1_si570_sda_b : inout std_logic;         -- I2C bus data (Si570)

      adc1_one_wire_b : inout std_logic;  -- Mezzanine 1-wire interface (DS18B20 thermometer + unique ID)

      ------------------------------------------
      -- FMC slot management
      ------------------------------------------
      fmc0_prsnt_m2c_n_i : in    std_logic;       -- Mezzanine present (active low)
      fmc0_scl_b         : inout std_logic;       -- Mezzanine system I2C clock (EEPROM)
      fmc0_sda_b         : inout std_logic;       -- Mezzanine system I2C data (EEPROM)

      fmc1_prsnt_m2c_n_i : in    std_logic;       -- Mezzanine present (active low)
      fmc1_scl_b         : inout std_logic;       -- Mezzanine system I2C clock (EEPROM)
      fmc1_sda_b         : inout std_logic        -- Mezzanine system I2C data (EEPROM)
      );
end svec_ref_fmc_adc_100Ms;


architecture rtl of svec_ref_fmc_adc_100Ms is

  ------------------------------------------------------------------------------
  -- SDB crossbar constants declaration
  ------------------------------------------------------------------------------

  -- Number of master port(s) on the wishbone crossbar
  constant c_NUM_WB_MASTERS : integer := 9;

  -- Number of slave port(s) on the wishbone crossbar
  constant c_NUM_WB_SLAVES : integer := 1;

  -- Wishbone master(s)
  constant c_WB_MASTER_VME : integer := 0;

  -- Wishbone slave(s)
  constant c_WB_SLAVE_SVEC_CSR     : integer := 0;  -- SVEC control and status registers
  constant c_WB_SLAVE_VIC          : integer := 1;  -- Vectored interrupt controller
  constant c_WB_SLAVE_FMC0_ADC     : integer := 2;  -- FMC slot 1 ADC mezzanine
  constant c_WB_SLAVE_FMC0_DDR_ADR : integer := 3;  -- FMC slot 1 DDR address
  constant c_WB_SLAVE_FMC0_DDR_DAT : integer := 4;  -- FMC slot 1 DDR data
  constant c_WB_SLAVE_FMC1_ADC     : integer := 5;  -- FMC slot 2 ADC mezzanine
  constant c_WB_SLAVE_FMC1_DDR_ADR : integer := 6;  -- FMC slot 2 DDR address
  constant c_WB_SLAVE_FMC1_DDR_DAT : integer := 7;  -- FMC slot 2 DDR data
  constant c_WB_SLAVE_WR_CORE      : integer := 8;  -- WR PTP core

  -- SDB meta info
  constant c_SDB_GIT_REPO_URL : integer := c_NUM_WB_MASTERS;
  constant c_SDB_SYNTHESIS    : integer := c_NUM_WB_MASTERS + 1;
  constant c_SDB_INTEGRATE    : integer := c_NUM_WB_MASTERS + 2;

  -- Devices sdb description
  constant c_wb_svec_csr_sdb : t_sdb_device := (
    abi_class     => x"0000",                     -- undocumented device
    abi_ver_major => x"01",
    abi_ver_minor => x"01",
    wbd_endian    => c_sdb_endian_big,
    wbd_width     => x"4",                        -- 32-bit port granularity
    sdb_component => (
      addr_first  => x"0000000000000000",
      addr_last   => x"000000000000001F",
      product     => (
        vendor_id => x"000000000000CE42",         -- CERN
        device_id => x"00006603",
        version   => x"00000001",
        date      => x"20121116",
        name      => "WB-SVEC-CSR        ")));

  constant c_wb_ddr_dat_sdb : t_sdb_device := (
    abi_class     => x"0000",                     -- undocumented device
    abi_ver_major => x"01",
    abi_ver_minor => x"01",
    wbd_endian    => c_sdb_endian_big,
    wbd_width     => x"4",                        -- 32-bit port granularity
    sdb_component => (
      addr_first  => x"0000000000000000",
      addr_last   => x"0000000000000FFF",
      product     => (
        vendor_id => x"000000000000CE42",         -- CERN
        device_id => x"10006610",
        version   => x"00000001",
        date      => x"20130704",
        name      => "WB-DDR-Data-Access ")));

  constant c_wb_ddr_adr_sdb : t_sdb_device := (
    abi_class     => x"0000",                     -- undocumented device
    abi_ver_major => x"01",
    abi_ver_minor => x"01",
    wbd_endian    => c_sdb_endian_big,
    wbd_width     => x"4",                        -- 32-bit port granularity
    sdb_component => (
      addr_first  => x"0000000000000000",
      addr_last   => x"0000000000000003",
      product     => (
        vendor_id => x"000000000000CE42",         -- CERN
        device_id => x"10006611",
        version   => x"00000001",
        date      => x"20130704",
        name      => "WB-DDR-Addr-Access ")));

  -- f_xwb_bridge_manual_sdb(size, sdb_addr)
  -- Note: sdb_addr is the sdb records address relative to the bridge base address
  constant c_fmc0_bridge_sdb    : t_sdb_bridge := f_xwb_bridge_manual_sdb(x"00001fff", x"00000000");
  constant c_fmc1_bridge_sdb    : t_sdb_bridge := f_xwb_bridge_manual_sdb(x"00001fff", x"00000000");
  constant c_wr_core_bridge_sdb : t_sdb_bridge := f_xwb_bridge_manual_sdb(x"0003ffff", x"00030000");

  -- sdb header address
  constant c_SDB_ADDRESS : t_wishbone_address := x"00000000";

  -- sdb integration record
  constant c_integration_sdb : t_sdb_integration := (
    product     => (
      vendor_id => x"000000000000CE42",  -- CERN
      device_id => x"5c01a632",          -- echo "svec_fmc-adc-100m14b4cha" | md5sum | cut -c1-8
      version   => x"00050000",          -- bcd encoded, [31:16] = major, [15:0] = minor
      date      => x"20181025",          -- yyyymmdd
      name      => "svec_fmcadc100m14b "));

  -- Wishbone crossbar layout
  constant c_INTERCONNECT_LAYOUT : t_sdb_record_array(c_NUM_WB_MASTERS + 2 downto 0) :=
    (
      c_WB_SLAVE_SVEC_CSR     => f_sdb_embed_device(c_wb_svec_csr_sdb, x"00001200"),
      c_WB_SLAVE_VIC          => f_sdb_embed_device(c_xwb_vic_sdb, x"00001300"),
      c_WB_SLAVE_FMC0_ADC     => f_sdb_embed_bridge(c_fmc0_bridge_sdb, x"00002000"),
      c_WB_SLAVE_FMC0_DDR_ADR => f_sdb_embed_device(c_wb_ddr_adr_sdb, x"00004000"),
      c_WB_SLAVE_FMC0_DDR_DAT => f_sdb_embed_device(c_wb_ddr_dat_sdb, x"00005000"),
      c_WB_SLAVE_FMC1_ADC     => f_sdb_embed_bridge(c_fmc1_bridge_sdb, x"00006000"),
      c_WB_SLAVE_FMC1_DDR_ADR => f_sdb_embed_device(c_wb_ddr_adr_sdb, x"00008000"),
      c_WB_SLAVE_FMC1_DDR_DAT => f_sdb_embed_device(c_wb_ddr_dat_sdb, x"00009000"),
      c_WB_SLAVE_WR_CORE      => f_sdb_embed_bridge(c_wr_core_bridge_sdb, x"00040000"),
      c_SDB_GIT_REPO_URL      => f_sdb_embed_repo_url(c_sdb_repo_url),
      c_SDB_SYNTHESIS         => f_sdb_embed_synthesis(c_sdb_synthesis_info),
      c_SDB_INTEGRATE         => f_sdb_embed_integration(c_integration_sdb)
      );

  -- VIC default vector setting
  constant c_VIC_VECTOR_TABLE : t_wishbone_address_array(0 to 1) :=
    (0 => x"00003500",
     1 => x"00007500");

  ------------------------------------------------------------------------------
  -- Other constants declaration
  ------------------------------------------------------------------------------

  -- SVEC carrier CSR constants
  constant c_CARRIER_TYPE : std_logic_vector(15 downto 0) := X"0002";

  -- Number of FMC slots
  constant c_NB_FMC_SLOTS : natural := 2;

  -- Conversion of g_simulation to string needed for DDR3 controller
  function f_int2string (n : natural) return string is
  begin
    if n = 0 then
      return "FALSE";
    else
      return "TRUE ";
    end if;
  end;

  constant c_SIMULATION_STR : string := f_int2string(g_simulation);

  ------------------------------------------------------------------------------
  -- Signals declaration
  ------------------------------------------------------------------------------

  -- Clocks and resets
  signal clk_sys_62m5       : std_logic;
  signal clk_ref_125m       : std_logic;
  signal sys_clk_pll_locked : std_logic;
  signal clk_ddr_333m_buf   : std_logic;
  signal clk_ddr_333m       : std_logic;
  signal clk_fb_buf         : std_logic;
  signal clk_fb             : std_logic;
  signal ddr_clk_pll_locked : std_logic;
  signal ddr_clk_pll_rst    : std_logic;
  signal areset_n           : std_logic := '0';
  signal rst_sys_62m5_n     : std_logic;
  signal rst_ref_125m_n     : std_logic;
  signal rst_ddr_333m_n     : std_logic;
  signal sw_rst_fmc0        : std_logic := '1';
  signal sw_rst_fmc1        : std_logic := '1';
  signal sw_rst_ddr0_sync   : std_logic;
  signal sw_rst_ddr1_sync   : std_logic;
  signal fmc0_rst_n         : std_logic;
  signal fmc1_rst_n         : std_logic;
  signal ddr_arst_n         : std_logic;
  signal ddr0_rst_n         : std_logic;
  signal ddr1_rst_n         : std_logic;

  -- VME
  signal vme_data_b_out    : std_logic_vector(31 downto 0);
  signal vme_addr_b_out    : std_logic_vector(31 downto 1);
  signal vme_lword_n_b_out : std_logic;
  signal Vme_data_dir_int  : std_logic;
  signal vme_addr_dir_int  : std_logic;
  signal vme_ga            : std_logic_vector(5 downto 0);
  signal vme_berr_n        : std_logic;
  signal vme_irq_n         : std_logic_vector(7 downto 1);
  signal vme_access        : std_logic;

  -- Wishbone buse(s) from crossbar master port(s)
  signal cnx_master_out : t_wishbone_master_out_array(c_NUM_WB_MASTERS-1 downto 0);
  signal cnx_master_in  : t_wishbone_master_in_array(c_NUM_WB_MASTERS-1 downto 0);

  -- Wishbone buse(s) to crossbar slave port(s)
  signal cnx_slave_out : t_wishbone_slave_out_array(c_NUM_WB_SLAVES-1 downto 0);
  signal cnx_slave_in  : t_wishbone_slave_in_array(c_NUM_WB_SLAVES-1 downto 0);

  -- Wishbone bus from cross-clocking module to FMC0 mezzanine
  signal cnx_fmc0_sync_master_out : t_wishbone_master_out;
  signal cnx_fmc0_sync_master_in  : t_wishbone_master_in;

  -- Wishbone bus from cross-clocking module to FMC1 mezzanine
  signal cnx_fmc1_sync_master_out : t_wishbone_master_out;
  signal cnx_fmc1_sync_master_in  : t_wishbone_master_in;

  -- Wishbone buses from FMC ADC cores to DDR controller
  signal wb_ddr0_in  : t_wishbone_master_data64_in;
  signal wb_ddr0_out : t_wishbone_master_data64_out;
  signal wb_ddr1_in  : t_wishbone_master_data64_in;
  signal wb_ddr1_out : t_wishbone_master_data64_out;

  -- Interrupts
  signal ddr_wr_fifo_empty   : std_logic_vector(c_NB_FMC_SLOTS-1 downto 0);
  signal acq_end_irq_p       : std_logic_vector(c_NB_FMC_SLOTS-1 downto 0);
  signal trig_irq_p          : std_logic_vector(c_NB_FMC_SLOTS-1 downto 0);
  signal fmc_trig_irq_led    : std_logic_vector(c_NB_FMC_SLOTS-1 downto 0);
  signal fmc_acq_end_irq_led : std_logic_vector(c_NB_FMC_SLOTS-1 downto 0);
  signal irq_to_vme          : std_logic;
  signal fmc_irq             : std_logic_vector(c_NB_FMC_SLOTS-1 downto 0);

  -- Front panel LED control
  signal svec_led      : std_logic_vector(15 downto 0);
  signal led_state     : std_logic_vector(15 downto 0);
  signal led_state_csr : std_logic_vector(15 downto 0);

  -- DDR0 (bank 4)
  signal ddr0_status      : std_logic_vector(31 downto 0);
  signal ddr0_calib_done  : std_logic;
  signal ddr0_addr_cnt    : unsigned(31 downto 0);
  signal ddr0_dat_cyc_d   : std_logic;
  signal ddr0_addr_cnt_en : std_logic;

  -- DDR1 (bank 5)
  signal ddr1_status      : std_logic_vector(31 downto 0);
  signal ddr1_calib_done  : std_logic;
  signal ddr1_addr_cnt    : unsigned(31 downto 0);
  signal ddr1_dat_cyc_d   : std_logic;
  signal ddr1_addr_cnt_en : std_logic;

  -- led pwm
  signal led_pwm_update_cnt : unsigned(9 downto 0);
  signal led_pwm_update     : std_logic;
  signal led_pwm_val        : unsigned(16 downto 0);
  signal led_pwm_val_down   : std_logic;
  signal led_pwm_cnt        : unsigned(16 downto 0);
  signal led_pwm            : std_logic;

  -- SFP
  signal sfp_scl_out : std_logic;
  signal sfp_sda_out : std_logic;
  signal sfp_scl_in  : std_logic;
  signal sfp_sda_in  : std_logic;

  -- OneWire
  signal onewire_data : std_logic;
  signal onewire_oe   : std_logic;

  -- White Rabbit
  signal wrabbit_en  : std_logic;
  signal wrc_scl_out : std_logic;
  signal wrc_scl_in  : std_logic;
  signal wrc_sda_out : std_logic;
  signal wrc_sda_in  : std_logic;
  signal pps         : std_logic;
  signal pps_led     : std_logic;
  signal wr_led_act  : std_logic;
  signal wr_led_link : std_logic;

  -- WR PTP core timing interface
  signal tm_link_up    : std_logic;
  signal tm_tai        : std_logic_vector(39 downto 0);
  signal tm_cycles     : std_logic_vector(27 downto 0);
  signal tm_time_valid : std_logic;

  -- IO for CSR registers
  signal csr_regin  : t_carrier_csr_in_registers;
  signal csr_regout : t_carrier_csr_out_registers;

begin

  ------------------------------------------------------------------------------
  -- 333MHz DDR clock PLL and BUFG based on DCM_SP without feedback (since we
  -- are only using the CLKFX output). Input clock is 125MHz ref from WRPC PLLs.
  ------------------------------------------------------------------------------

  cmp_sys_clk_pll : DCM_SP
    generic map (
      CLKFX_DIVIDE          => 3,
      CLKFX_MULTIPLY        => 8,
      CLKIN_DIVIDE_BY_2     => FALSE,
      CLKIN_PERIOD          => 8.0,
      CLKOUT_PHASE_SHIFT    => "NONE",
      CLK_FEEDBACK          => "1X",
      DESKEW_ADJUST         => "SYSTEM_SYNCHRONOUS",
      DFS_FREQUENCY_MODE    => "LOW",
      DLL_FREQUENCY_MODE    => "LOW",
      DSS_MODE              => "NONE",
      DUTY_CYCLE_CORRECTION => TRUE,
      FACTORY_JF            => X"c080",
      PHASE_SHIFT           => 0,
      STARTUP_WAIT          => FALSE
      )
    port map (
      CLK0     => clk_fb_buf,
      CLKFX    => clk_ddr_333m_buf,
      CLKFX180 => open,
      LOCKED   => ddr_clk_pll_locked,
      CLKFB    => clk_fb,
      CLKIN    => clk_ref_125m,
      DSSEN    => '0',
      PSCLK    => '0',
      PSEN     => '0',
      PSINCDEC => '0',
      RST      => ddr_clk_pll_rst);

  cmp_fb_clk_buf : BUFG
    port map (
      O => clk_fb,
      I => clk_fb_buf);

  cmp_ddr_clk_buf : BUFG
    port map (
      O => clk_ddr_333m,
      I => clk_ddr_333m_buf);

  ------------------------------------------------------------------------------
  -- Reset logic
  ------------------------------------------------------------------------------

  areset_n <= vme_rst_n_i and rst_n_i;

  sys_clk_pll_locked <= '1';

  ddr_clk_pll_rst <= sys_clk_pll_locked;

  -- logic AND of all async reset sources for DDR (active low)
  ddr_arst_n <= sys_clk_pll_locked and ddr_clk_pll_locked;

  cmp_powerup_reset : gc_reset
    generic map (
      g_clocks    => 1,  -- 333MHz
      g_logdelay  => 4,  -- 16 clock cycles
      g_syncdepth => 3)  -- length of sync chains
    port map (
      free_clk_i => clk_ref_125m,
      locked_i   => ddr_arst_n,
      clks_i(0)  => clk_ddr_333m,
      rstn_o(0)  => rst_ddr_333m_n);

  -- reset for mezzanines
  -- (including soft reset, no need to re-sync from 62.5MHz domain)
  fmc0_rst_n <= rst_ref_125m_n and (not sw_rst_fmc0);
  fmc1_rst_n <= rst_ref_125m_n and (not sw_rst_fmc1);

  -- reset for DDR
  -- (including soft reset, with re-sync from 62.5MHz domain)
  cmp_ddr0_sw_reset_sync : gc_sync_ffs
    port map (
      clk_i    => clk_ddr_333m,
      rst_n_i  => '1',
      data_i   => sw_rst_fmc0,
      synced_o => sw_rst_ddr0_sync);

  cmp_ddr1_sw_reset_sync : gc_sync_ffs
    port map (
      clk_i    => clk_ddr_333m,
      rst_n_i  => '1',
      data_i   => sw_rst_fmc1,
      synced_o => sw_rst_ddr1_sync);

  ddr0_rst_n <= rst_ddr_333m_n and (not sw_rst_ddr0_sync);
  ddr1_rst_n <= rst_ddr_333m_n and (not sw_rst_ddr0_sync);

  ------------------------------------------------------------------------------
  -- VME interface
  ------------------------------------------------------------------------------
  cmp_vme_core : xvme64x_core
    generic map (
      g_CLOCK_PERIOD    => 16,
      g_DECODE_AM       => TRUE,
      g_USER_CSR_EXT    => FALSE,
      g_WB_GRANULARITY  => BYTE,
      g_MANUFACTURER_ID => c_CERN_ID,
      g_BOARD_ID        => c_SVEC_ID,
      g_REVISION_ID     => c_SVEC_REVISION_ID,
      g_PROGRAM_ID      => c_SVEC_PROGRAM_ID)
    port map (
      clk_i           => clk_sys_62m5,
      rst_n_i         => rst_sys_62m5_n,
      vme_i.as_n      => vme_as_n_i,
      vme_i.rst_n     => vme_rst_n_i,
      vme_i.write_n   => vme_write_n_i,
      vme_i.am        => vme_am_i,
      vme_i.ds_n      => vme_ds_n_i,
      vme_i.ga        => vme_ga,
      vme_i.lword_n   => vme_lword_n_b,
      vme_i.addr      => vme_addr_b,
      vme_i.data      => vme_data_b,
      vme_i.iack_n    => vme_iack_n_i,
      vme_i.iackin_n  => vme_iackin_n_i,
      vme_o.berr_n    => vme_berr_n,
      vme_o.dtack_n   => vme_dtack_n_o,
      vme_o.retry_n   => vme_retry_n_o,
      vme_o.retry_oe  => vme_retry_oe_o,
      vme_o.lword_n   => vme_lword_n_b_out,
      vme_o.data      => vme_data_b_out,
      vme_o.addr      => vme_addr_b_out,
      vme_o.irq_n     => vme_irq_n,
      vme_o.iackout_n => vme_iackout_n_o,
      vme_o.dtack_oe  => vme_dtack_oe_o,
      vme_o.data_dir  => vme_data_dir_int,
      vme_o.data_oe_n => vme_data_oe_n_o,
      vme_o.addr_dir  => vme_addr_dir_int,
      vme_o.addr_oe_n => vme_addr_oe_n_o,
      wb_o            => cnx_slave_in(c_WB_MASTER_VME),
      wb_i            => cnx_slave_out(c_WB_MASTER_VME),
      int_i           => irq_to_vme);


  vme_ga     <= vme_gap_i & vme_ga_i;
  vme_berr_o <= not vme_berr_n;
  vme_irq_o  <= not vme_irq_n;

  -- VME tri-state buffers
  vme_data_b    <= vme_data_b_out    when vme_data_dir_int = '1' else (others => 'Z');
  vme_addr_b    <= vme_addr_b_out    when vme_addr_dir_int = '1' else (others => 'Z');
  vme_lword_n_b <= vme_lword_n_b_out when vme_addr_dir_int = '1' else 'Z';

  vme_addr_dir_o <= vme_addr_dir_int;
  vme_data_dir_o <= vme_data_dir_int;

  ------------------------------------------------------------------------------
  -- Primary wishbone crossbar
  ------------------------------------------------------------------------------
  cmp_sdb_crossbar : xwb_sdb_crossbar
    generic map (
      g_num_masters => c_NUM_WB_SLAVES,
      g_num_slaves  => c_NUM_WB_MASTERS,
      g_registered  => TRUE,
      g_wraparound  => TRUE,
      g_layout      => c_INTERCONNECT_LAYOUT,
      g_sdb_addr    => c_SDB_ADDRESS)
    port map (
      clk_sys_i => clk_sys_62m5,
      rst_n_i   => rst_sys_62m5_n,
      slave_i   => cnx_slave_in,
      slave_o   => cnx_slave_out,
      master_i  => cnx_master_in,
      master_o  => cnx_master_out);

  -------------------------------------------------------------------------------
  -- White Rabbit Core (SVEC board package)
  -------------------------------------------------------------------------------

  -- Tristates for Carrier EEPROM
  carrier_scl_b <= '0' when (wrc_scl_out = '0') else 'Z';
  carrier_sda_b <= '0' when (wrc_sda_out = '0') else 'Z';
  wrc_scl_in    <= carrier_scl_b;
  wrc_sda_in    <= carrier_sda_b;

  -- Tristates for SFP EEPROM
  sfp_mod_def1_b <= '0' when sfp_scl_out = '0' else 'Z';
  sfp_mod_def2_b <= '0' when sfp_sda_out = '0' else 'Z';
  sfp_scl_in     <= sfp_mod_def1_b;
  sfp_sda_in     <= sfp_mod_def2_b;

  carrier_onewire_b <= '0' when onewire_oe = '1' else 'Z';
  onewire_data      <= carrier_onewire_b;

  cmp_xwrc_board_svec : xwrc_board_svec
    generic map (
      g_simulation                => g_simulation,
      g_with_external_clock_input => FALSE,
      g_dpram_initf               => g_wrpc_initf,
      g_fabric_iface              => PLAIN)
    port map (
      clk_20m_vcxo_i      => clk_20m_vcxo_i,
      clk_125m_pllref_p_i => clk_125m_pllref_p_i,
      clk_125m_pllref_n_i => clk_125m_pllref_n_i,
      clk_125m_gtp_n_i    => clk_125m_gtp_n_i,
      clk_125m_gtp_p_i    => clk_125m_gtp_p_i,
      areset_n_i          => areset_n,
      clk_sys_62m5_o      => clk_sys_62m5,
      clk_ref_125m_o      => clk_ref_125m,
      rst_sys_62m5_n_o    => rst_sys_62m5_n,
      rst_ref_125m_n_o    => rst_ref_125m_n,
      pll20dac_din_o      => pll20dac_din_o,
      pll20dac_sclk_o     => pll20dac_sclk_o,
      pll20dac_sync_n_o   => pll20dac_sync_n_o,
      pll25dac_din_o      => pll25dac_din_o,
      pll25dac_sclk_o     => pll25dac_sclk_o,
      pll25dac_sync_n_o   => pll25dac_sync_n_o,
      sfp_txp_o           => sfp_txp_o,
      sfp_txn_o           => sfp_txn_o,
      sfp_rxp_i           => sfp_rxp_i,
      sfp_rxn_i           => sfp_rxn_i,
      sfp_det_i           => sfp_mod_def0_i,
      sfp_sda_i           => sfp_sda_in,
      sfp_sda_o           => sfp_sda_out,
      sfp_scl_i           => sfp_scl_in,
      sfp_scl_o           => sfp_scl_out,
      sfp_rate_select_o   => sfp_rate_select_o,
      sfp_tx_fault_i      => sfp_tx_fault_i,
      sfp_tx_disable_o    => sfp_tx_disable_o,
      sfp_los_i           => sfp_los_i,
      eeprom_sda_i        => wrc_sda_in,
      eeprom_sda_o        => wrc_sda_out,
      eeprom_scl_i        => wrc_scl_in,
      eeprom_scl_o        => wrc_scl_out,
      onewire_i           => onewire_data,
      onewire_oen_o       => onewire_oe,
      uart_rxd_i          => uart_rxd_i,
      uart_txd_o          => uart_txd_o,
      spi_sclk_o          => spi_sclk_o,
      spi_ncs_o           => spi_ncs_o,
      spi_mosi_o          => spi_mosi_o,
      spi_miso_i          => spi_miso_i,
      wb_slave_o          => cnx_master_in(c_WB_SLAVE_WR_CORE),
      wb_slave_i          => cnx_master_out(c_WB_SLAVE_WR_CORE),
      tm_link_up_o        => tm_link_up,
      tm_time_valid_o     => tm_time_valid,
      tm_tai_o            => tm_tai,
      tm_cycles_o         => tm_cycles,
      pps_p_o             => pps,
      pps_led_o           => pps_led,
      led_link_o          => wr_led_link,
      led_act_o           => wr_led_act,
      link_ok_o           => wrabbit_en);

  ------------------------------------------------------------------------------
  -- Carrier CSR
  --    Carrier type and PCB version
  --    Carrier status (PLL, FMC presence)
  --    Front panel LED manual control
  ------------------------------------------------------------------------------
  cmp_carrier_csr : entity work.carrier_csr
    port map(
      rst_n_i    => rst_sys_62m5_n,
      clk_sys_i  => clk_sys_62m5,
      wb_adr_i   => cnx_master_out(c_WB_SLAVE_SVEC_CSR).adr(3 downto 2),  -- cnx_master_out.adr is byte address
      wb_dat_i   => cnx_master_out(c_WB_SLAVE_SVEC_CSR).dat,
      wb_dat_o   => cnx_master_in(c_WB_SLAVE_SVEC_CSR).dat,
      wb_cyc_i   => cnx_master_out(c_WB_SLAVE_SVEC_CSR).cyc,
      wb_sel_i   => cnx_master_out(c_WB_SLAVE_SVEC_CSR).sel,
      wb_stb_i   => cnx_master_out(c_WB_SLAVE_SVEC_CSR).stb,
      wb_we_i    => cnx_master_out(c_WB_SLAVE_SVEC_CSR).we,
      wb_ack_o   => cnx_master_in(c_WB_SLAVE_SVEC_CSR).ack,
      wb_stall_o => open,
      regs_i     => csr_regin,
      regs_o     => csr_regout);

  csr_regin.carrier_pcb_rev_i    <= pcbrev_i;
  csr_regin.carrier_reserved_i   <= (others => '0');
  csr_regin.carrier_type_i       <= c_CARRIER_TYPE;
  csr_regin.stat_fmc0_pres_i     <= fmc0_prsnt_m2c_n_i;
  csr_regin.stat_fmc1_pres_i     <= fmc1_prsnt_m2c_n_i;
  csr_regin.stat_sys_pll_lck_i   <= sys_clk_pll_locked;
  csr_regin.stat_ddr0_cal_done_i <= ddr0_calib_done;
  csr_regin.stat_ddr1_cal_done_i <= ddr1_calib_done;

  led_state_csr <= csr_regout.ctrl_fp_leds_man_o;
  sw_rst_fmc0   <= csr_regout.rst_fmc0_o;
  sw_rst_fmc1   <= csr_regout.rst_fmc1_o;

  -- Unused wishbone signals
  cnx_master_in(c_WB_SLAVE_SVEC_CSR).err   <= '0';
  cnx_master_in(c_WB_SLAVE_SVEC_CSR).rty   <= '0';
  cnx_master_in(c_WB_SLAVE_SVEC_CSR).stall <= '0';

  ------------------------------------------------------------------------------
  -- Vectored interrupt controller (VIC)
  ------------------------------------------------------------------------------
  cmp_vic : xwb_vic
    generic map (
      g_interface_mode      => PIPELINED,
      g_address_granularity => BYTE,
      g_num_interrupts      => 2,
      g_init_vectors        => c_VIC_VECTOR_TABLE)
    port map (
      clk_sys_i    => clk_sys_62m5,
      rst_n_i      => rst_sys_62m5_n,
      slave_i      => cnx_master_out(c_WB_SLAVE_VIC),
      slave_o      => cnx_master_in(c_WB_SLAVE_VIC),
      irqs_i(0)    => fmc_irq(0),
      irqs_i(1)    => fmc_irq(1),
      irq_master_o => irq_to_vme);

  ------------------------------------------------------------------------------
  -- Slot 1 : FMC ADC mezzanine (wb bridge with cross-clocking)
  --    Mezzanine system managment I2C master
  --    Mezzanine SPI master
  --    Mezzanine I2C
  --    ADC core
  --    Mezzanine 1-wire master
  ------------------------------------------------------------------------------

  cmp_xwb_clock_crossing_0 : xwb_clock_crossing
    generic map(
      g_size => 16
      )
    port map(
      slave_clk_i    => clk_sys_62m5,
      slave_rst_n_i  => rst_sys_62m5_n,
      slave_i        => cnx_master_out(c_WB_SLAVE_FMC0_ADC),
      slave_o        => cnx_master_in(c_WB_SLAVE_FMC0_ADC),
      master_clk_i   => clk_ref_125m,
      master_rst_n_i => fmc0_rst_n,
      master_i       => cnx_fmc0_sync_master_in,
      master_o       => cnx_fmc0_sync_master_out
      );

  cmp_fmc_adc_mezzanine_0 : fmc_adc_mezzanine
    generic map(
      g_MULTISHOT_RAM_SIZE => g_MULTISHOT_RAM_SIZE
      )
    port map(
      sys_clk_i   => clk_ref_125m,
      sys_rst_n_i => fmc0_rst_n,

      wb_csr_slave_i => cnx_fmc0_sync_master_out,
      wb_csr_slave_o => cnx_fmc0_sync_master_in,

      wb_ddr_clk_i    => clk_ref_125m,
      wb_ddr_rst_n_i  => fmc0_rst_n,
      wb_ddr_master_i => wb_ddr0_in,
      wb_ddr_master_o => wb_ddr0_out,

      ddr_wr_fifo_empty_i => ddr_wr_fifo_empty(0),
      trig_irq_o          => trig_irq_p(0),
      acq_end_irq_o       => acq_end_irq_p(0),
      eic_irq_o           => fmc_irq(0),

      ext_trigger_p_i => adc0_ext_trigger_p_i,
      ext_trigger_n_i => adc0_ext_trigger_n_i,

      adc_dco_p_i  => adc0_dco_p_i,
      adc_dco_n_i  => adc0_dco_n_i,
      adc_fr_p_i   => adc0_fr_p_i,
      adc_fr_n_i   => adc0_fr_n_i,
      adc_outa_p_i => adc0_outa_p_i,
      adc_outa_n_i => adc0_outa_n_i,
      adc_outb_p_i => adc0_outb_p_i,
      adc_outb_n_i => adc0_outb_n_i,

      gpio_dac_clr_n_o => adc0_gpio_dac_clr_n_o,
      gpio_led_acq_o   => adc0_gpio_led_acq_o,
      gpio_led_trig_o  => adc0_gpio_led_trig_o,
      gpio_ssr_ch1_o   => adc0_gpio_ssr_ch1_o,
      gpio_ssr_ch2_o   => adc0_gpio_ssr_ch2_o,
      gpio_ssr_ch3_o   => adc0_gpio_ssr_ch3_o,
      gpio_ssr_ch4_o   => adc0_gpio_ssr_ch4_o,
      gpio_si570_oe_o  => adc0_gpio_si570_oe_o,

      spi_din_i       => adc0_spi_din_i,
      spi_dout_o      => adc0_spi_dout_o,
      spi_sck_o       => adc0_spi_sck_o,
      spi_cs_adc_n_o  => adc0_spi_cs_adc_n_o,
      spi_cs_dac1_n_o => adc0_spi_cs_dac1_n_o,
      spi_cs_dac2_n_o => adc0_spi_cs_dac2_n_o,
      spi_cs_dac3_n_o => adc0_spi_cs_dac3_n_o,
      spi_cs_dac4_n_o => adc0_spi_cs_dac4_n_o,

      si570_scl_b => adc0_si570_scl_b,
      si570_sda_b => adc0_si570_sda_b,

      mezz_one_wire_b => adc0_one_wire_b,

      sys_scl_b => fmc0_scl_b,
      sys_sda_b => fmc0_sda_b,

      wr_tm_link_up_i    => tm_link_up,
      wr_tm_time_valid_i => tm_time_valid,
      wr_tm_tai_i        => tm_tai,
      wr_tm_cycles_i     => tm_cycles,
      wr_enable_i        => wrabbit_en

      );

  ------------------------------------------------------------------------------
  -- Slot 2 : FMC ADC mezzanine (wb bridge with cross-clocking)
  --    Mezzanine system managment I2C master
  --    Mezzanine SPI master
  --    Mezzanine I2C
  --    ADC core
  --    Mezzanine 1-wire master
  ------------------------------------------------------------------------------

  cmp_xwb_clock_crossing_1 : xwb_clock_crossing
    generic map(
      g_size => 16
      )
    port map(
      slave_clk_i    => clk_sys_62m5,
      slave_rst_n_i  => rst_sys_62m5_n,
      slave_i        => cnx_master_out(c_WB_SLAVE_FMC1_ADC),
      slave_o        => cnx_master_in(c_WB_SLAVE_FMC1_ADC),
      master_clk_i   => clk_ref_125m,
      master_rst_n_i => fmc1_rst_n,
      master_i       => cnx_fmc1_sync_master_in,
      master_o       => cnx_fmc1_sync_master_out
      );

  cmp_fmc_adc_mezzanine_1 : fmc_adc_mezzanine
    generic map(
      g_MULTISHOT_RAM_SIZE => g_MULTISHOT_RAM_SIZE
      )
    port map(
      sys_clk_i   => clk_ref_125m,
      sys_rst_n_i => fmc1_rst_n,

      wb_csr_slave_i => cnx_fmc1_sync_master_out,
      wb_csr_slave_o => cnx_fmc1_sync_master_in,

      wb_ddr_clk_i    => clk_ref_125m,
      wb_ddr_rst_n_i  => fmc1_rst_n,
      wb_ddr_master_i => wb_ddr1_in,
      wb_ddr_master_o => wb_ddr1_out,

      ddr_wr_fifo_empty_i => ddr_wr_fifo_empty(1),
      trig_irq_o          => trig_irq_p(1),
      acq_end_irq_o       => acq_end_irq_p(1),
      eic_irq_o           => fmc_irq(1),

      ext_trigger_p_i => adc1_ext_trigger_p_i,
      ext_trigger_n_i => adc1_ext_trigger_n_i,

      adc_dco_p_i  => adc1_dco_p_i,
      adc_dco_n_i  => adc1_dco_n_i,
      adc_fr_p_i   => adc1_fr_p_i,
      adc_fr_n_i   => adc1_fr_n_i,
      adc_outa_p_i => adc1_outa_p_i,
      adc_outa_n_i => adc1_outa_n_i,
      adc_outb_p_i => adc1_outb_p_i,
      adc_outb_n_i => adc1_outb_n_i,

      gpio_dac_clr_n_o => adc1_gpio_dac_clr_n_o,
      gpio_led_acq_o   => adc1_gpio_led_acq_o,
      gpio_led_trig_o  => adc1_gpio_led_trig_o,
      gpio_ssr_ch1_o   => adc1_gpio_ssr_ch1_o,
      gpio_ssr_ch2_o   => adc1_gpio_ssr_ch2_o,
      gpio_ssr_ch3_o   => adc1_gpio_ssr_ch3_o,
      gpio_ssr_ch4_o   => adc1_gpio_ssr_ch4_o,
      gpio_si570_oe_o  => adc1_gpio_si570_oe_o,

      spi_din_i       => adc1_spi_din_i,
      spi_dout_o      => adc1_spi_dout_o,
      spi_sck_o       => adc1_spi_sck_o,
      spi_cs_adc_n_o  => adc1_spi_cs_adc_n_o,
      spi_cs_dac1_n_o => adc1_spi_cs_dac1_n_o,
      spi_cs_dac2_n_o => adc1_spi_cs_dac2_n_o,
      spi_cs_dac3_n_o => adc1_spi_cs_dac3_n_o,
      spi_cs_dac4_n_o => adc1_spi_cs_dac4_n_o,

      si570_scl_b => adc1_si570_scl_b,
      si570_sda_b => adc1_si570_sda_b,

      mezz_one_wire_b => adc1_one_wire_b,

      sys_scl_b => fmc1_scl_b,
      sys_sda_b => fmc1_sda_b,

      wr_tm_link_up_i    => tm_link_up,
      wr_tm_time_valid_i => tm_time_valid,
      wr_tm_tai_i        => tm_tai,
      wr_tm_cycles_i     => tm_cycles,
      wr_enable_i        => wrabbit_en

      );

  ------------------------------------------------------------------------------
  -- DDR0 controller (bank 4)
  ------------------------------------------------------------------------------
  cmp_ddr_ctrl_bank4 : ddr3_ctrl
    generic map(
      g_BANK_PORT_SELECT   => "SVEC_BANK4_64B_32B",
      g_MEMCLK_PERIOD      => 3000,
      g_SIMULATION         => c_SIMULATION_STR,
      g_CALIB_SOFT_IP      => g_CALIB_SOFT_IP,
      g_P0_MASK_SIZE       => 8,
      g_P0_DATA_PORT_SIZE  => 64,
      g_P0_BYTE_ADDR_WIDTH => 30,
      g_P1_MASK_SIZE       => 4,
      g_P1_DATA_PORT_SIZE  => 32,
      g_P1_BYTE_ADDR_WIDTH => 30)
    port map (
      clk_i   => clk_ddr_333m,
      rst_n_i => ddr0_rst_n,

      status_o => ddr0_status,

      ddr3_dq_b     => ddr0_dq_b,
      ddr3_a_o      => ddr0_a_o,
      ddr3_ba_o     => ddr0_ba_o,
      ddr3_ras_n_o  => ddr0_ras_n_o,
      ddr3_cas_n_o  => ddr0_cas_n_o,
      ddr3_we_n_o   => ddr0_we_n_o,
      ddr3_odt_o    => ddr0_odt_o,
      ddr3_rst_n_o  => ddr0_reset_n_o,
      ddr3_cke_o    => ddr0_cke_o,
      ddr3_dm_o     => ddr0_ldm_o,
      ddr3_udm_o    => ddr0_udm_o,
      ddr3_dqs_p_b  => ddr0_ldqs_p_b,
      ddr3_dqs_n_b  => ddr0_ldqs_n_b,
      ddr3_udqs_p_b => ddr0_udqs_p_b,
      ddr3_udqs_n_b => ddr0_udqs_n_b,
      ddr3_clk_p_o  => ddr0_ck_p_o,
      ddr3_clk_n_o  => ddr0_ck_n_o,
      ddr3_rzq_b    => ddr0_rzq_b,

      wb0_rst_n_i => rst_ref_125m_n,
      wb0_clk_i   => clk_ref_125m,
      wb0_sel_i   => wb_ddr0_out.sel,
      wb0_cyc_i   => wb_ddr0_out.cyc,
      wb0_stb_i   => wb_ddr0_out.stb,
      wb0_we_i    => wb_ddr0_out.we,
      wb0_addr_i  => wb_ddr0_out.adr,
      wb0_data_i  => wb_ddr0_out.dat,
      wb0_data_o  => wb_ddr0_in.dat,
      wb0_ack_o   => wb_ddr0_in.ack,
      wb0_stall_o => wb_ddr0_in.stall,

      p0_cmd_empty_o   => open,
      p0_cmd_full_o    => open,
      p0_rd_full_o     => open,
      p0_rd_empty_o    => open,
      p0_rd_count_o    => open,
      p0_rd_overflow_o => open,
      p0_rd_error_o    => open,
      p0_wr_full_o     => open,
      p0_wr_empty_o    => ddr_wr_fifo_empty(0),
      p0_wr_count_o    => open,
      p0_wr_underrun_o => open,
      p0_wr_error_o    => open,

      wb1_rst_n_i => rst_sys_62m5_n,
      wb1_clk_i   => clk_sys_62m5,
      wb1_sel_i   => cnx_master_out(c_WB_SLAVE_FMC0_DDR_DAT).sel,
      wb1_cyc_i   => cnx_master_out(c_WB_SLAVE_FMC0_DDR_DAT).cyc,
      wb1_stb_i   => cnx_master_out(c_WB_SLAVE_FMC0_DDR_DAT).stb,
      wb1_we_i    => cnx_master_out(c_WB_SLAVE_FMC0_DDR_DAT).we,
      wb1_addr_i  => std_logic_vector(ddr0_addr_cnt),
      wb1_data_i  => cnx_master_out(c_WB_SLAVE_FMC0_DDR_DAT).dat,
      wb1_data_o  => cnx_master_in(c_WB_SLAVE_FMC0_DDR_DAT).dat,
      wb1_ack_o   => cnx_master_in(c_WB_SLAVE_FMC0_DDR_DAT).ack,
      wb1_stall_o => cnx_master_in(c_WB_SLAVE_FMC0_DDR_DAT).stall,

      p1_cmd_empty_o   => open,
      p1_cmd_full_o    => open,
      p1_rd_full_o     => open,
      p1_rd_empty_o    => open,
      p1_rd_count_o    => open,
      p1_rd_overflow_o => open,
      p1_rd_error_o    => open,
      p1_wr_full_o     => open,
      p1_wr_empty_o    => open,
      p1_wr_count_o    => open,
      p1_wr_underrun_o => open,
      p1_wr_error_o    => open

      );

  wb_ddr0_in.err <= '0';
  wb_ddr0_in.rty <= '0';

  ddr0_calib_done <= ddr0_status(0);

  -- DDR0 (bank 4) address counter
  --  The address counter is set by writing to the c_WB_SLAVE_FMC0_DDR_ADR wishbone periph.
  --  Than the counter is incremented on every access to the c_WB_SLAVE_FMC0_DDR_DAT wishbone periph.
  --  The counter is incremented on the falling edge of cyc. This is because the ddr controller
  --  samples the address on (cyc_re and stb)+1

  p_ddr0_dat_cyc : process (clk_sys_62m5)
  begin
    if rising_edge(clk_sys_62m5) then
      if (rst_sys_62m5_n = '0' or sw_rst_fmc0 = '1') then
        ddr0_dat_cyc_d <= '0';
      else
        ddr0_dat_cyc_d <= cnx_master_out(c_WB_SLAVE_FMC0_DDR_DAT).cyc;
      end if;
    end if;
  end process p_ddr0_dat_cyc;

  ddr0_addr_cnt_en <= not(cnx_master_out(c_WB_SLAVE_FMC0_DDR_DAT).cyc) and ddr0_dat_cyc_d;

  -- address counter
  p_ddr0_addr_cnt : process (clk_sys_62m5)
  begin
    if rising_edge(clk_sys_62m5) then
      if (rst_sys_62m5_n = '0' or sw_rst_fmc0 = '1') then
        ddr0_addr_cnt <= (others => '0');
      elsif (cnx_master_out(c_WB_SLAVE_FMC0_DDR_ADR).we = '1' and
             cnx_master_out(c_WB_SLAVE_FMC0_DDR_ADR).stb = '1' and
             cnx_master_out(c_WB_SLAVE_FMC0_DDR_ADR).cyc = '1') then
        ddr0_addr_cnt <= unsigned(cnx_master_out(c_WB_SLAVE_FMC0_DDR_ADR).dat);
      elsif (ddr0_addr_cnt_en = '1') then
        ddr0_addr_cnt <= ddr0_addr_cnt + 1;
      end if;
    end if;
  end process p_ddr0_addr_cnt;

  -- ack generation
  p_ddr0_addr_ack : process (clk_sys_62m5)
  begin
    if rising_edge(clk_sys_62m5) then
      if (rst_sys_62m5_n = '0' or sw_rst_fmc0 = '1') then
        cnx_master_in(c_WB_SLAVE_FMC0_DDR_ADR).ack <= '0';
      elsif (cnx_master_out(c_WB_SLAVE_FMC0_DDR_ADR).stb = '1' and
             cnx_master_out(c_WB_SLAVE_FMC0_DDR_ADR).cyc = '1') then
        cnx_master_in(c_WB_SLAVE_FMC0_DDR_ADR).ack <= '1';
      else
        cnx_master_in(c_WB_SLAVE_FMC0_DDR_ADR).ack <= '0';
      end if;
    end if;
  end process p_ddr0_addr_ack;

  -- Address counter read back
  cnx_master_in(c_WB_SLAVE_FMC0_DDR_ADR).dat <= std_logic_vector(ddr0_addr_cnt);

  -- Unused wishbone signals
  cnx_master_in(c_WB_SLAVE_FMC0_DDR_DAT).err   <= '0';
  cnx_master_in(c_WB_SLAVE_FMC0_DDR_DAT).rty   <= '0';
  cnx_master_in(c_WB_SLAVE_FMC0_DDR_ADR).err   <= '0';
  cnx_master_in(c_WB_SLAVE_FMC0_DDR_ADR).rty   <= '0';
  cnx_master_in(c_WB_SLAVE_FMC0_DDR_ADR).stall <= '0';

  ------------------------------------------------------------------------------
  -- DDR1 controller (bank 5)
  ------------------------------------------------------------------------------
  cmp_ddr_ctrl_bank5 : ddr3_ctrl
    generic map(
      g_BANK_PORT_SELECT   => "SVEC_BANK5_64B_32B",
      g_MEMCLK_PERIOD      => 3000,
      g_SIMULATION         => c_SIMULATION_STR,
      g_CALIB_SOFT_IP      => g_CALIB_SOFT_IP,
      g_P0_MASK_SIZE       => 8,
      g_P0_DATA_PORT_SIZE  => 64,
      g_P0_BYTE_ADDR_WIDTH => 30,
      g_P1_MASK_SIZE       => 4,
      g_P1_DATA_PORT_SIZE  => 32,
      g_P1_BYTE_ADDR_WIDTH => 30)
    port map (
      clk_i   => clk_ddr_333m,
      rst_n_i => ddr1_rst_n,

      status_o => ddr1_status,

      ddr3_dq_b     => ddr1_dq_b,
      ddr3_a_o      => ddr1_a_o,
      ddr3_ba_o     => ddr1_ba_o,
      ddr3_ras_n_o  => ddr1_ras_n_o,
      ddr3_cas_n_o  => ddr1_cas_n_o,
      ddr3_we_n_o   => ddr1_we_n_o,
      ddr3_odt_o    => ddr1_odt_o,
      ddr3_rst_n_o  => ddr1_reset_n_o,
      ddr3_cke_o    => ddr1_cke_o,
      ddr3_dm_o     => ddr1_ldm_o,
      ddr3_udm_o    => ddr1_udm_o,
      ddr3_dqs_p_b  => ddr1_ldqs_p_b,
      ddr3_dqs_n_b  => ddr1_ldqs_n_b,
      ddr3_udqs_p_b => ddr1_udqs_p_b,
      ddr3_udqs_n_b => ddr1_udqs_n_b,
      ddr3_clk_p_o  => ddr1_ck_p_o,
      ddr3_clk_n_o  => ddr1_ck_n_o,
      ddr3_rzq_b    => ddr1_rzq_b,

      wb0_rst_n_i => rst_ref_125m_n,
      wb0_clk_i   => clk_ref_125m,
      wb0_sel_i   => wb_ddr1_out.sel,
      wb0_cyc_i   => wb_ddr1_out.cyc,
      wb0_stb_i   => wb_ddr1_out.stb,
      wb0_we_i    => wb_ddr1_out.we,
      wb0_addr_i  => wb_ddr1_out.adr,
      wb0_data_i  => wb_ddr1_out.dat,
      wb0_data_o  => wb_ddr1_in.dat,
      wb0_ack_o   => wb_ddr1_in.ack,
      wb0_stall_o => wb_ddr1_in.stall,

      p0_cmd_empty_o   => open,
      p0_cmd_full_o    => open,
      p0_rd_full_o     => open,
      p0_rd_empty_o    => open,
      p0_rd_count_o    => open,
      p0_rd_overflow_o => open,
      p0_rd_error_o    => open,
      p0_wr_full_o     => open,
      p0_wr_empty_o    => ddr_wr_fifo_empty(1),
      p0_wr_count_o    => open,
      p0_wr_underrun_o => open,
      p0_wr_error_o    => open,

      wb1_rst_n_i => rst_sys_62m5_n,
      wb1_clk_i   => clk_sys_62m5,
      wb1_sel_i   => cnx_master_out(c_WB_SLAVE_FMC1_DDR_DAT).sel,
      wb1_cyc_i   => cnx_master_out(c_WB_SLAVE_FMC1_DDR_DAT).cyc,
      wb1_stb_i   => cnx_master_out(c_WB_SLAVE_FMC1_DDR_DAT).stb,
      wb1_we_i    => cnx_master_out(c_WB_SLAVE_FMC1_DDR_DAT).we,
      wb1_addr_i  => std_logic_vector(ddr1_addr_cnt),
      wb1_data_i  => cnx_master_out(c_WB_SLAVE_FMC1_DDR_DAT).dat,
      wb1_data_o  => cnx_master_in(c_WB_SLAVE_FMC1_DDR_DAT).dat,
      wb1_ack_o   => cnx_master_in(c_WB_SLAVE_FMC1_DDR_DAT).ack,
      wb1_stall_o => cnx_master_in(c_WB_SLAVE_FMC1_DDR_DAT).stall,

      p1_cmd_empty_o   => open,
      p1_cmd_full_o    => open,
      p1_rd_full_o     => open,
      p1_rd_empty_o    => open,
      p1_rd_count_o    => open,
      p1_rd_overflow_o => open,
      p1_rd_error_o    => open,
      p1_wr_full_o     => open,
      p1_wr_empty_o    => open,
      p1_wr_count_o    => open,
      p1_wr_underrun_o => open,
      p1_wr_error_o    => open

      );

  wb_ddr1_in.err <= '0';
  wb_ddr1_in.rty <= '0';

  ddr1_calib_done <= ddr1_status(0);

  -- DDR1 (bank 5) address counter
  --  The address counter is set by writing to the c_WB_SLAVE_FMC1_DDR_ADR wishbone periph.
  --  Than the counter is incremented on every access to the c_WB_SLAVE_FMC1_DDR_DAT wishbone periph.
  --  The counter is incremented on the falling edge of cyc. This is because the ddr controller
  --  samples the address on (cyc_re and stb)+1

  p_ddr1_dat_cyc : process (clk_sys_62m5)
  begin
    if rising_edge(clk_sys_62m5) then
      if (rst_sys_62m5_n = '0' or sw_rst_fmc1 = '1') then
        ddr1_dat_cyc_d <= '0';
      else
        ddr1_dat_cyc_d <= cnx_master_out(c_WB_SLAVE_FMC1_DDR_DAT).cyc;
      end if;
    end if;
  end process p_ddr1_dat_cyc;

  ddr1_addr_cnt_en <= not(cnx_master_out(c_WB_SLAVE_FMC1_DDR_DAT).cyc) and ddr1_dat_cyc_d;

  -- address counter
  p_ddr1_addr_cnt : process (clk_sys_62m5)
  begin
    if rising_edge(clk_sys_62m5) then
      if (rst_sys_62m5_n = '0' or sw_rst_fmc1 = '1') then
        ddr1_addr_cnt <= (others => '0');
      elsif (cnx_master_out(c_WB_SLAVE_FMC1_DDR_ADR).we = '1' and
             cnx_master_out(c_WB_SLAVE_FMC1_DDR_ADR).stb = '1' and
             cnx_master_out(c_WB_SLAVE_FMC1_DDR_ADR).cyc = '1') then
        ddr1_addr_cnt <= unsigned(cnx_master_out(c_WB_SLAVE_FMC1_DDR_ADR).dat);
      elsif (ddr1_addr_cnt_en = '1') then
        ddr1_addr_cnt <= ddr1_addr_cnt + 1;
      end if;
    end if;
  end process p_ddr1_addr_cnt;

  -- ack generation
  p_ddr1_addr_ack : process (clk_sys_62m5)
  begin
    if rising_edge(clk_sys_62m5) then
      if (rst_sys_62m5_n = '0' or sw_rst_fmc1 = '1') then
        cnx_master_in(c_WB_SLAVE_FMC1_DDR_ADR).ack <= '0';
      elsif (cnx_master_out(c_WB_SLAVE_FMC1_DDR_ADR).stb = '1' and
             cnx_master_out(c_WB_SLAVE_FMC1_DDR_ADR).cyc = '1') then
        cnx_master_in(c_WB_SLAVE_FMC1_DDR_ADR).ack <= '1';
      else
        cnx_master_in(c_WB_SLAVE_FMC1_DDR_ADR).ack <= '0';
      end if;
    end if;
  end process p_ddr1_addr_ack;

  -- Address counter read back
  cnx_master_in(c_WB_SLAVE_FMC1_DDR_ADR).dat <= std_logic_vector(ddr1_addr_cnt);

  -- Unused wishbone signals
  cnx_master_in(c_WB_SLAVE_FMC1_DDR_DAT).err   <= '0';
  cnx_master_in(c_WB_SLAVE_FMC1_DDR_DAT).rty   <= '0';
  cnx_master_in(c_WB_SLAVE_FMC1_DDR_ADR).err   <= '0';
  cnx_master_in(c_WB_SLAVE_FMC1_DDR_ADR).rty   <= '0';
  cnx_master_in(c_WB_SLAVE_FMC1_DDR_ADR).stall <= '0';

  ------------------------------------------------------------------------------
  -- Carrier front panel LEDs and LEMOs
  ------------------------------------------------------------------------------
  cmp_led_controller : gc_bicolor_led_ctrl
    generic map(
      g_nb_column    => 4,
      g_nb_line      => 2,
      g_clk_freq     => 62500000,                 -- in Hz
      g_refresh_rate => 250                       -- in Hz
      )
    port map(
      rst_n_i => rst_sys_62m5_n,
      clk_i   => clk_sys_62m5,

      led_intensity_i => "1100100",               -- in %

      led_state_i => svec_led,

      column_o   => fp_led_column_o,
      line_o     => fp_led_line_o,
      line_oen_o => fp_led_line_oen_o
      );

  cmp_vme_access_led : gc_extend_pulse
    generic map (
      g_width => 2500000)
    port map (
      clk_i      => clk_sys_62m5,
      rst_n_i    => rst_sys_62m5_n,
      pulse_i    => cnx_slave_in(c_WB_MASTER_VME).cyc,
      extended_o => vme_access
      );

  gen_fmc_irq_led : for I in 0 to c_NB_FMC_SLOTS - 1 generate
    cmp_fmc_trig_irq_led : gc_extend_pulse
      generic map (
        g_width => 2500000)
      port map (
        clk_i      => clk_sys_62m5,
        rst_n_i    => rst_sys_62m5_n,
        pulse_i    => trig_irq_p(I),
        extended_o => fmc_trig_irq_led(I)
        );

    cmp_fmc_acq_end_irq_led : gc_extend_pulse
      generic map (
        g_width => 2500000)
      port map (
        clk_i      => clk_sys_62m5,
        rst_n_i    => rst_sys_62m5_n,
        pulse_i    => acq_end_irq_p(0),
        extended_o => fmc_acq_end_irq_led(I)
        );
  end generate gen_fmc_irq_led;

  -- Logic OR of signals and CSR register for LED control
  svec_led <= led_state or led_state_csr;

  -- LED 1 : VME access
  led_state(1 downto 0) <= c_led_green when vme_access = '1' else c_led_off;

  -- LED 2 :
  led_state(3 downto 2) <= c_led_red when wr_led_act = '1' else c_led_off;

  -- LED 3 :
  led_state(5 downto 4) <= c_led_green when wr_led_link = '1' else c_led_off;

  -- LED 4 :
  led_state(7 downto 6) <= c_led_green when led_pwm = '1' else c_led_off;

  -- LED 5 :
  led_state(9 downto 8) <= c_led_red_green when fmc_trig_irq_led(0) = '1' else c_led_off;

  -- LED 6 :
  led_state(11 downto 10) <= c_led_red_green when fmc_acq_end_irq_led(0) = '1' else c_led_off;

  -- LED 7 :
  led_state(13 downto 12) <= c_led_red_green when fmc_trig_irq_led(1) = '1' else c_led_off;

  -- LED 8 :
  led_state(15 downto 14) <= c_led_red_green when fmc_acq_end_irq_led(1) = '1' else c_led_off;

  -- Front panel IO configuration
  fp_gpio1_o      <= pps;
  fp_gpio2_o      <= '0';
  fp_gpio3_o      <= '0';
  fp_gpio4_o      <= '0';
  fp_term_en_o    <= (others => '0');
  fp_gpio1_a2b_o  <= '1';
  fp_gpio2_a2b_o  <= '1';
  fp_gpio34_a2b_o <= '1';

  ------------------------------------------------------------------------------
  -- FPGA loaded led (heart beat)
  ------------------------------------------------------------------------------
  p_led_pwn_update_cnt : process (clk_sys_62m5)
  begin
    if rising_edge(clk_sys_62m5) then
      if (rst_sys_62m5_n = '0') then
        led_pwm_update_cnt <= (others => '0');
        led_pwm_update     <= '0';
      elsif (led_pwm_update_cnt = to_unsigned(477, 10)) then
        led_pwm_update_cnt <= (others => '0');
        led_pwm_update     <= '1';
      else
        led_pwm_update_cnt <= led_pwm_update_cnt + 1;
        led_pwm_update     <= '0';
      end if;
    end if;
  end process p_led_pwn_update_cnt;

  p_led_pwn_val : process (clk_sys_62m5)
  begin
    if rising_edge(clk_sys_62m5) then
      if (rst_sys_62m5_n = '0') then
        led_pwm_val      <= (others => '0');
        led_pwm_val_down <= '0';
      elsif (led_pwm_update = '1') then
        if led_pwm_val_down = '1' then
          if led_pwm_val = X"100" then
            led_pwm_val_down <= '0';
          end if;
          led_pwm_val <= led_pwm_val - 1;
        else
          if led_pwm_val = X"1FFFE" then
            led_pwm_val_down <= '1';
          end if;
          led_pwm_val <= led_pwm_val + 1;
        end if;
      end if;
    end if;
  end process p_led_pwn_val;

  p_led_pwn_cnt : process (clk_sys_62m5)
  begin
    if rising_edge(clk_sys_62m5) then
      if (rst_sys_62m5_n = '0') then
        led_pwm_cnt <= (others => '0');
      else
        led_pwm_cnt <= led_pwm_cnt + 1;
      end if;
    end if;
  end process p_led_pwn_cnt;

  p_led_pwn : process (clk_sys_62m5)
  begin
    if rising_edge(clk_sys_62m5) then
      if (rst_sys_62m5_n = '0') then
        led_pwm <= '0';
      elsif (led_pwm_cnt = 0) then
        led_pwm <= '1';
      elsif (led_pwm_cnt = led_pwm_val) then
        led_pwm <= '0';
      end if;
    end if;
  end process p_led_pwn;

end rtl;
