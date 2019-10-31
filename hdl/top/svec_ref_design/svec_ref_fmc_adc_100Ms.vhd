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

use work.vme64x_pkg.all;
use work.ddr3_ctrl_pkg.all;
use work.gencores_pkg.all;
use work.wishbone_pkg.all;
use work.fmc_adc_mezzanine_pkg.all;
use work.synthesis_descriptor.all;
use work.svec_carrier_csr_pkg.all;
use work.wr_xilinx_pkg.all;
use work.wr_board_pkg.all;
use work.wr_svec_pkg.all;

entity svec_ref_fmc_adc_100Ms is
  generic(
    g_SIMULATION         : integer := 0;
    g_NB_FMC_SLOTS       : natural := 2;
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
      -- DDR (banks 4 and 5)
      ------------------------------------------
      ddr_a_o       : out   std_logic_vector(14*g_NB_FMC_SLOTS-1 downto 0);
      ddr_ba_o      : out   std_logic_vector(3*g_NB_FMC_SLOTS-1 downto 0);
      ddr_cas_n_o   : out   std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);
      ddr_ck_n_o    : out   std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);
      ddr_ck_p_o    : out   std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);
      ddr_cke_o     : out   std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);
      ddr_dq_b      : inout std_logic_vector(16*g_NB_FMC_SLOTS-1 downto 0);
      ddr_ldm_o     : out   std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);
      ddr_ldqs_n_b  : inout std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);
      ddr_ldqs_p_b  : inout std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);
      ddr_odt_o     : out   std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);
      ddr_ras_n_o   : out   std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);
      ddr_reset_n_o : out   std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);
      ddr_rzq_b     : inout std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);
      ddr_udm_o     : out   std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);
      ddr_udqs_n_b  : inout std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);
      ddr_udqs_p_b  : inout std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);
      ddr_we_n_o    : out   std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);

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
      fmc_prsnt_m2c_n_i : in    std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);  -- Mezzanine present (active low)
      fmc_scl_b         : inout std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);  -- Mezzanine system I2C clock (EEPROM)
      fmc_sda_b         : inout std_logic_vector(g_NB_FMC_SLOTS-1 downto 0));  -- Mezzanine system I2C data (EEPROM)
end svec_ref_fmc_adc_100Ms;


architecture rtl of svec_ref_fmc_adc_100Ms is

  function f_ddr_bank_sel (
    constant idx : natural)
    return string is
  begin
    if idx = 0 then
      return "SVEC_BANK4_64B_32B";
    else
      return "SVEC_BANK5_64B_32B";
    end if;
  end function f_ddr_bank_sel;

  ------------------------------------------------------------------------------
  -- SDB crossbar constants declaration
  ------------------------------------------------------------------------------

  -- Number of masters on the wishbone crossbar
  constant c_NUM_WB_MASTERS : integer := 1;

  -- Number of slaves on the wishbone crossbar
  constant c_NUM_WB_SLAVES : integer := 9;

  -- Wishbone master(s)
  constant c_WB_MASTER_VME : integer := 0;

  -- Wishbone slave(s)
  -- IMPORTANT: FMC1 peripherals need always be at +3 index offset from the
  -- respective FMC0 ones, in order for the FMC+DDR generating loop to work
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
  constant c_SDB_GIT_REPO_URL : integer := c_NUM_WB_SLAVES;
  constant c_SDB_SYNTHESIS    : integer := c_NUM_WB_SLAVES + 1;

  -- Devices sdb description
  constant c_WB_SVEC_CSR_SDB : t_sdb_device := (
    abi_class     => x"0000",                     -- undocumented device
    abi_ver_major => x"01",
    abi_ver_minor => x"01",
    wbd_endian    => c_SDB_ENDIAN_BIG,
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

  constant c_WB_DDR_DAT_SDB : t_sdb_device := (
    abi_class     => x"0000",                     -- undocumented device
    abi_ver_major => x"01",
    abi_ver_minor => x"01",
    wbd_endian    => c_SDB_ENDIAN_BIG,
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

  constant c_WB_DDR_ADR_SDB : t_sdb_device := (
    abi_class     => x"0000",                     -- undocumented device
    abi_ver_major => x"01",
    abi_ver_minor => x"01",
    wbd_endian    => c_SDB_ENDIAN_BIG,
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
  constant c_FMC0_BRIDGE_SDB    : t_sdb_bridge := f_xwb_bridge_manual_sdb(x"00001fff", x"00000000");
  constant c_FMC1_BRIDGE_SDB    : t_sdb_bridge := f_xwb_bridge_manual_sdb(x"00001fff", x"00000000");
  constant c_WR_CORE_BRIDGE_SDB : t_sdb_bridge := f_xwb_bridge_manual_sdb(x"0003ffff", x"00030000");

  -- sdb header address
  constant c_SDB_ADDRESS : t_wishbone_address := x"00000000";

  -- Wishbone crossbar layout
  constant c_INTERCONNECT_LAYOUT : t_sdb_record_array(c_NUM_WB_SLAVES + 1 downto 0) :=
    (
      c_WB_SLAVE_SVEC_CSR     => f_sdb_embed_device(c_WB_SVEC_CSR_SDB,    x"00001200"),
      c_WB_SLAVE_VIC          => f_sdb_embed_device(c_XWB_VIC_SDB,        x"00001300"),
      c_WB_SLAVE_FMC0_ADC     => f_sdb_embed_bridge(c_FMC0_BRIDGE_SDB,    x"00002000"),
      c_WB_SLAVE_FMC0_DDR_ADR => f_sdb_embed_device(c_WB_DDR_ADR_SDB,     x"00004000"),
      c_WB_SLAVE_FMC0_DDR_DAT => f_sdb_embed_device(c_WB_DDR_DAT_SDB,     x"00005000"),
      c_WB_SLAVE_FMC1_ADC     => f_sdb_embed_bridge(c_FMC1_BRIDGE_SDB,    x"00006000"),
      c_WB_SLAVE_FMC1_DDR_ADR => f_sdb_embed_device(c_WB_DDR_ADR_SDB,     x"00008000"),
      c_WB_SLAVE_FMC1_DDR_DAT => f_sdb_embed_device(c_WB_DDR_DAT_SDB,     x"00009000"),
      c_WB_SLAVE_WR_CORE      => f_sdb_embed_bridge(c_WR_CORE_BRIDGE_SDB, x"00040000"),
      c_SDB_GIT_REPO_URL      => f_sdb_embed_repo_url(c_SDB_REPO_URL),
      c_SDB_SYNTHESIS         => f_sdb_embed_synthesis(c_SDB_SYNTHESIS_INFO));

  -- VIC default vector setting
  constant c_VIC_VECTOR_TABLE : t_wishbone_address_array(0 to 1) :=
    (0 => x"00003500",
     1 => x"00007500");

  ------------------------------------------------------------------------------
  -- Other constants declaration
  ------------------------------------------------------------------------------

  -- WRPC Xilinx platform auxiliary clock configuration, used for DDR clock
  constant c_WRPC_PLL_CONFIG : t_auxpll_cfg_array := (
    0      => (enabled => TRUE, bufg_en => TRUE, divide => 3),
    others => c_AUXPLL_CFG_DEFAULT);

  -- SVEC carrier CSR constants
  constant c_CARRIER_TYPE : std_logic_vector(15 downto 0) := X"0002";

  -- Conversion of g_simulation to string needed for DDR controller
  function f_int2string (n : natural) return string is
  begin
    if n = 0 then
      return "FALSE";
    else
      return "TRUE ";
    end if;
  end;

  constant c_SIMULATION_STR : string := f_int2string(g_SIMULATION);

  ------------------------------------------------------------------------------
  -- Signals declaration
  ------------------------------------------------------------------------------

  subtype t_fmc_slot_vec is std_logic_vector(g_NB_FMC_SLOTS-1 downto 0);

  -- Clocks and resets
  signal clk_sys_62m5       : std_logic;
  signal clk_ref_125m       : std_logic;
  signal sys_clk_pll_locked : std_logic;
  signal clk_ddr_333m       : std_logic;
  signal clk_pll_aux        : std_logic_vector(3 downto 0);

  signal rst_pll_aux_n      : std_logic_vector(3 downto 0) := (others => '0');
  signal areset_n           : std_logic            := '0';
  signal rst_sys_62m5_n     : std_logic            := '0';
  signal rst_ref_125m_n     : std_logic            := '0';
  signal rst_ddr_333m_n     : std_logic            := '0';
  signal sw_rst_fmc         : t_fmc_slot_vec       := (others => '1');
  signal sw_rst_fmc_sync    : t_fmc_slot_vec       := (others => '1');
  signal fmc_rst_ref_n      : t_fmc_slot_vec       := (others => '0');
  signal fmc_rst_sys_n      : t_fmc_slot_vec       := (others => '0');
  signal ddr_rst            : t_fmc_slot_vec       := (others => '1');

  attribute keep                 : string;
  attribute keep of clk_sys_62m5 : signal is "TRUE";
  attribute keep of clk_ref_125m : signal is "TRUE";
  attribute keep of clk_ddr_333m : signal is "TRUE";
  attribute keep of ddr_rst      : signal is "TRUE";

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

  -- Wishbone buse(s) from master(s) to crossbar slave port(s)
  signal cnx_master_out : t_wishbone_master_out_array(c_NUM_WB_MASTERS-1 downto 0);
  signal cnx_master_in  : t_wishbone_master_in_array(c_NUM_WB_MASTERS-1 downto 0);

  -- Wishbone buse(s) from crossbar master port(s) to slave(s)
  signal cnx_slave_out : t_wishbone_slave_out_array(c_NUM_WB_SLAVES-1 downto 0);
  signal cnx_slave_in  : t_wishbone_slave_in_array(c_NUM_WB_SLAVES-1 downto 0);

  -- Wishbone bus from cross-clocking module to FMC mezzanines
  signal cnx_fmc_sync_master_out : t_wishbone_master_out_array(g_NB_FMC_SLOTS-1 downto 0);
  signal cnx_fmc_sync_master_in  : t_wishbone_master_in_array(g_NB_FMC_SLOTS-1 downto 0);

  -- Wishbone buses from FMC ADC cores to DDR controller
  signal fmc_wb_ddr_in  : t_wishbone_master_data64_in_array(g_NB_FMC_SLOTS-1 downto 0);
  signal fmc_wb_ddr_out : t_wishbone_master_data64_out_array(g_NB_FMC_SLOTS-1 downto 0);

  -- Interrupts and status
  signal ddr_wr_fifo_empty   : t_fmc_slot_vec;
  signal irq_to_vme          : std_logic;
  signal fmc_irq             : t_fmc_slot_vec;
  signal fmc_acq_cfg_ok      : t_fmc_slot_vec;

  -- Resync interrupts to sys domain
  signal ddr_wr_fifo_empty_sync : t_fmc_slot_vec;
  signal fmc_irq_sync           : t_fmc_slot_vec;
  signal fmc_acq_cfg_ok_sync    : t_fmc_slot_vec;

  -- Front panel LED control
  signal svec_led      : std_logic_vector(15 downto 0);
  signal led_state     : std_logic_vector(15 downto 0);
  signal led_state_csr : std_logic_vector(15 downto 0);

  -- DDR
  type t_fmc_adc_ddr_status_array is array (natural range <>) of std_logic_vector(31 downto 0);
  type t_fmc_adc_ddr_addr_cnt_array is array (natural range <>) of unsigned(31 downto 0);

  signal ddr_status      : t_fmc_adc_ddr_status_array(g_NB_FMC_SLOTS-1 downto 0);
  signal ddr_calib_done  : t_fmc_slot_vec;
  signal ddr_addr_cnt    : t_fmc_adc_ddr_addr_cnt_array(g_NB_FMC_SLOTS-1 downto 0);
  signal ddr_dat_cyc_d   : t_fmc_slot_vec;
  signal ddr_addr_cnt_en : t_fmc_slot_vec;

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
  signal csr_regin  : t_carrier_csr_master_in;
  signal csr_regout : t_carrier_csr_master_out;

begin

  ------------------------------------------------------------------------------
  -- Reset logic
  ------------------------------------------------------------------------------

  areset_n <= vme_rst_n_i and rst_n_i;

  sys_clk_pll_locked <= '1';

  gen_fmc_rst : for I in 0 to g_NB_FMC_SLOTS-1 generate

    -- reset for mezzanines
    -- including soft reset, with re-sync from 62.5MHz domain
    cmp_fmc_sw_reset_sync : gc_sync_ffs
      port map (
        clk_i    => clk_ref_125m,
        rst_n_i  => '1',
        data_i   => sw_rst_fmc(I),
        synced_o => sw_rst_fmc_sync(I));

    fmc_rst_ref_n(I) <= rst_ref_125m_n and not sw_rst_fmc_sync(I);
    fmc_rst_sys_n(I) <= rst_sys_62m5_n and not sw_rst_fmc(I);

    -- reset for DDR including soft reset.
    -- This is treated as async and will be re-synced by the DDR controller
    ddr_rst(I) <= not rst_ddr_333m_n or sw_rst_fmc(I);

  end generate gen_fmc_rst;

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
      wb_o            => cnx_master_out(c_WB_MASTER_VME),
      wb_i            => cnx_master_in(c_WB_MASTER_VME),
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
      g_NUM_MASTERS => c_NUM_WB_MASTERS,
      g_NUM_SLAVES  => c_NUM_WB_SLAVES,
      g_REGISTERED  => TRUE,
      g_WRAPAROUND  => TRUE,
      g_LAYOUT      => c_INTERCONNECT_LAYOUT,
      g_SDB_ADDR    => c_SDB_ADDRESS)
    port map (
      clk_sys_i => clk_sys_62m5,
      rst_n_i   => rst_sys_62m5_n,
      slave_i   => cnx_master_out,
      slave_o   => cnx_master_in,
      master_i  => cnx_slave_out,
      master_o  => cnx_slave_in);

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

  -- Tristates for Carrier OneWire
  carrier_onewire_b <= '0' when onewire_oe = '1' else 'Z';
  onewire_data      <= carrier_onewire_b;

  cmp_xwrc_board_svec : xwrc_board_svec
    generic map (
      g_SIMULATION                => g_SIMULATION,
      g_WITH_EXTERNAL_CLOCK_INPUT => FALSE,
      g_DPRAM_INITF               => g_WRPC_INITF,
      g_AUX_PLL_CFG               => c_WRPC_PLL_CONFIG,
      g_FABRIC_IFACE              => PLAIN)
    port map (
      clk_20m_vcxo_i      => clk_20m_vcxo_i,
      clk_125m_pllref_p_i => clk_125m_pllref_p_i,
      clk_125m_pllref_n_i => clk_125m_pllref_n_i,
      clk_125m_gtp_n_i    => clk_125m_gtp_n_i,
      clk_125m_gtp_p_i    => clk_125m_gtp_p_i,
      areset_n_i          => areset_n,
      clk_sys_62m5_o      => clk_sys_62m5,
      clk_ref_125m_o      => clk_ref_125m,
      clk_pll_aux_o       => clk_pll_aux,
      rst_sys_62m5_n_o    => rst_sys_62m5_n,
      rst_ref_125m_n_o    => rst_ref_125m_n,
      rst_pll_aux_n_o     => rst_pll_aux_n,
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
      wb_slave_o          => cnx_slave_out(c_WB_SLAVE_WR_CORE),
      wb_slave_i          => cnx_slave_in(c_WB_SLAVE_WR_CORE),
      tm_link_up_o        => tm_link_up,
      tm_time_valid_o     => tm_time_valid,
      tm_tai_o            => tm_tai,
      tm_cycles_o         => tm_cycles,
      pps_p_o             => pps,
      pps_led_o           => pps_led,
      led_link_o          => wr_led_link,
      led_act_o           => wr_led_act,
      link_ok_o           => wrabbit_en);

  clk_ddr_333m   <= clk_pll_aux(0);
  rst_ddr_333m_n <= rst_pll_aux_n(0);

  ------------------------------------------------------------------------------
  -- Carrier CSR
  --    Carrier type and PCB version
  --    Carrier status (PLL, FMC presence)
  --    Front panel LED manual control
  ------------------------------------------------------------------------------
  cmp_carrier_csr : entity work.svec_carrier_csr
    port map (
      rst_n_i       => rst_sys_62m5_n,
      clk_i         => clk_sys_62m5,
      wb_i          => cnx_slave_in(c_WB_SLAVE_SVEC_CSR),
      wb_o          => cnx_slave_out(c_WB_SLAVE_SVEC_CSR),
      carrier_csr_i => csr_regin,
      carrier_csr_o => csr_regout);

  csr_regin.carrier_pcb_rev    <= pcbrev_i;
  csr_regin.carrier_reserved   <= (others => '0');
  csr_regin.carrier_type       <= c_CARRIER_TYPE;
  csr_regin.stat_fmc0_pres     <= fmc_prsnt_m2c_n_i(0);
  csr_regin.stat_fmc1_pres     <= fmc_prsnt_m2c_n_i(1);
  csr_regin.stat_sys_pll_lck   <= sys_clk_pll_locked;
  csr_regin.stat_ddr0_cal_done <= ddr_calib_done(0);
  csr_regin.stat_ddr1_cal_done <= ddr_calib_done(1);

  led_state_csr <= csr_regout.ctrl_fp_leds_man;
  sw_rst_fmc(0) <= csr_regout.rst_fmc0;
  sw_rst_fmc(1) <= csr_regout.rst_fmc1;

  ------------------------------------------------------------------------------
  -- Vectored interrupt controller (VIC)
  ------------------------------------------------------------------------------

  gen_fmc_irq : for I in 0 to g_NB_FMC_SLOTS - 1 generate

    cmp_fmc_irq_sync : gc_sync_ffs
      port map (
        clk_i    => clk_sys_62m5,
        rst_n_i  => '1',
        data_i   => fmc_irq(I),
        synced_o => fmc_irq_sync(I));

  end generate gen_fmc_irq;

  cmp_vic : xwb_vic
    generic map (
      g_INTERFACE_MODE      => PIPELINED,
      g_ADDRESS_GRANULARITY => BYTE,
      g_NUM_INTERRUPTS      => 2,
      g_INIT_VECTORS        => c_VIC_VECTOR_TABLE)
    port map (
      clk_sys_i    => clk_sys_62m5,
      rst_n_i      => rst_sys_62m5_n,
      slave_i      => cnx_slave_in(c_WB_SLAVE_VIC),
      slave_o      => cnx_slave_out(c_WB_SLAVE_VIC),
      irqs_i(0)    => fmc_irq_sync(0),
      irqs_i(1)    => fmc_irq_sync(1),
      irq_master_o => irq_to_vme);

  ------------------------------------------------------------------------------
  -- FMC ADC mezzanines (wb bridge with cross-clocking)
  --    Mezzanine system managment I2C master
  --    Mezzanine SPI master
  --    Mezzanine I2C
  --    ADC core
  --    Mezzanine 1-wire master
  ------------------------------------------------------------------------------

  gen_fmc_mezzanine : for I in 0 to g_NB_FMC_SLOTS - 1 generate

    cmp_xwb_clock_bridge : xwb_clock_bridge
      port map (
        slave_clk_i    => clk_sys_62m5,
        slave_rst_n_i  => fmc_rst_sys_n(I),
        slave_i        => cnx_slave_in(c_WB_SLAVE_FMC0_ADC + 3*I),
        slave_o        => cnx_slave_out(c_WB_SLAVE_FMC0_ADC + 3*I),
        master_clk_i   => clk_ref_125m,
        master_rst_n_i => fmc_rst_ref_n(I),
        master_i       => cnx_fmc_sync_master_in(I),
        master_o       => cnx_fmc_sync_master_out(I));

    cmp_fmc_ddr_wr_fifo_sync : gc_sync_ffs
      port map (
        clk_i    => clk_ref_125m,
        rst_n_i  => '1',
        data_i   => ddr_wr_fifo_empty(I),
        synced_o => ddr_wr_fifo_empty_sync(I));

    cmp_fmc_adc_mezzanine : fmc_adc_mezzanine
      generic map (
        g_MULTISHOT_RAM_SIZE => g_MULTISHOT_RAM_SIZE,
        g_WB_MODE            => PIPELINED,
        g_WB_GRANULARITY     => BYTE)
      port map (
        sys_clk_i   => clk_ref_125m,
        sys_rst_n_i => fmc_rst_ref_n(I),

        wb_csr_slave_i => cnx_fmc_sync_master_out(I),
        wb_csr_slave_o => cnx_fmc_sync_master_in(I),

        wb_ddr_clk_i    => clk_ref_125m,
        wb_ddr_rst_n_i  => fmc_rst_ref_n(I),
        wb_ddr_master_i => fmc_wb_ddr_in(I),
        wb_ddr_master_o => fmc_wb_ddr_out(I),

        ddr_wr_fifo_empty_i => ddr_wr_fifo_empty_sync(I),
        trig_irq_o          => open,
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

        sys_scl_b => fmc_scl_b(I),
        sys_sda_b => fmc_sda_b(I),

        wr_tm_link_up_i    => tm_link_up,
        wr_tm_time_valid_i => tm_time_valid,
        wr_tm_tai_i        => tm_tai,
        wr_tm_cycles_i     => tm_cycles,
        wr_enable_i        => wrabbit_en);

  end generate gen_fmc_mezzanine;

  ------------------------------------------------------------------------------
  -- DDR controllers (banks 4 and 5)
  ------------------------------------------------------------------------------
  gen_ddr_ctrl : for I in 0 to g_NB_FMC_SLOTS - 1 generate

    cmp_ddr_ctrl_bank : ddr3_ctrl
      generic map (
        g_RST_ACT_LOW        => 0, -- active high reset (simpler internal logic)
        g_BANK_PORT_SELECT   => f_ddr_bank_sel(I),
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
        rst_n_i => ddr_rst(I),

        status_o => ddr_status(I),

        ddr3_dq_b     => ddr_dq_b(16*(I+1)-1 downto 16*I),
        ddr3_a_o      => ddr_a_o(14*(I+1)-1 downto 14*I),
        ddr3_ba_o     => ddr_ba_o(3*(I+1)-1 downto 3*I),
        ddr3_ras_n_o  => ddr_ras_n_o(I),
        ddr3_cas_n_o  => ddr_cas_n_o(I),
        ddr3_we_n_o   => ddr_we_n_o(I),
        ddr3_odt_o    => ddr_odt_o(I),
        ddr3_rst_n_o  => ddr_reset_n_o(I),
        ddr3_cke_o    => ddr_cke_o(I),
        ddr3_dm_o     => ddr_ldm_o(I),
        ddr3_udm_o    => ddr_udm_o(I),
        ddr3_dqs_p_b  => ddr_ldqs_p_b(I),
        ddr3_dqs_n_b  => ddr_ldqs_n_b(I),
        ddr3_udqs_p_b => ddr_udqs_p_b(I),
        ddr3_udqs_n_b => ddr_udqs_n_b(I),
        ddr3_clk_p_o  => ddr_ck_p_o(I),
        ddr3_clk_n_o  => ddr_ck_n_o(I),
        ddr3_rzq_b    => ddr_rzq_b(I),

        wb0_rst_n_i => fmc_rst_ref_n(I),
        wb0_clk_i   => clk_ref_125m,
        wb0_sel_i   => fmc_wb_ddr_out(I).sel,
        wb0_cyc_i   => fmc_wb_ddr_out(I).cyc,
        wb0_stb_i   => fmc_wb_ddr_out(I).stb,
        wb0_we_i    => fmc_wb_ddr_out(I).we,
        wb0_addr_i  => fmc_wb_ddr_out(I).adr,
        wb0_data_i  => fmc_wb_ddr_out(I).dat,
        wb0_data_o  => fmc_wb_ddr_in(I).dat,
        wb0_ack_o   => fmc_wb_ddr_in(I).ack,
        wb0_stall_o => fmc_wb_ddr_in(I).stall,

        p0_cmd_empty_o   => open,
        p0_cmd_full_o    => open,
        p0_rd_full_o     => open,
        p0_rd_empty_o    => open,
        p0_rd_count_o    => open,
        p0_rd_overflow_o => open,
        p0_rd_error_o    => open,
        p0_wr_full_o     => open,
        p0_wr_empty_o    => ddr_wr_fifo_empty(I),
        p0_wr_count_o    => open,
        p0_wr_underrun_o => open,
        p0_wr_error_o    => open,

        wb1_rst_n_i => rst_sys_62m5_n,
        wb1_clk_i   => clk_sys_62m5,
        wb1_sel_i   => cnx_slave_in(c_WB_SLAVE_FMC0_DDR_DAT + 3*I).sel,
        wb1_cyc_i   => cnx_slave_in(c_WB_SLAVE_FMC0_DDR_DAT + 3*I).cyc,
        wb1_stb_i   => cnx_slave_in(c_WB_SLAVE_FMC0_DDR_DAT + 3*I).stb,
        wb1_we_i    => cnx_slave_in(c_WB_SLAVE_FMC0_DDR_DAT + 3*I).we,
        wb1_data_i  => cnx_slave_in(c_WB_SLAVE_FMC0_DDR_DAT + 3*I).dat,
        wb1_addr_i  => std_logic_vector(ddr_addr_cnt(I)),
        wb1_data_o  => cnx_slave_out(c_WB_SLAVE_FMC0_DDR_DAT + 3*I).dat,
        wb1_ack_o   => cnx_slave_out(c_WB_SLAVE_FMC0_DDR_DAT + 3*I).ack,
        wb1_stall_o => cnx_slave_out(c_WB_SLAVE_FMC0_DDR_DAT + 3*I).stall,

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

    fmc_wb_ddr_in(I).err <= '0';
    fmc_wb_ddr_in(I).rty <= '0';

    ddr_calib_done(I) <= ddr_status(I)(0);

    -- DDR address counter
    --  The address counter is set by writing to the c_WB_SLAVE_FMC_DDR_ADR wb peripheral.
    --  Then the counter is incremented on every access to the c_WB_SLAVE_FMC_DDR_DAT wb peripheral.
    --  The counter is incremented on the falling edge of cyc. This is because the ddr controller
    --  samples the address on (cyc_re and stb)+1

    p_ddr_dat_cyc : process (clk_sys_62m5)
    begin
      if rising_edge(clk_sys_62m5) then
        if rst_sys_62m5_n = '0' then
          ddr_dat_cyc_d(I) <= '0';
        else
          ddr_dat_cyc_d(I) <= cnx_slave_in(c_WB_SLAVE_FMC0_DDR_DAT + 3*I).cyc;
        end if;
      end if;
    end process p_ddr_dat_cyc;

    ddr_addr_cnt_en(I) <= not(cnx_slave_in(c_WB_SLAVE_FMC0_DDR_DAT + 3*I).cyc) and ddr_dat_cyc_d(I);

    -- address counter
    p_ddr_addr_cnt : process (clk_sys_62m5)
    begin
      if rising_edge(clk_sys_62m5) then
        if rst_sys_62m5_n = '0' then
          ddr_addr_cnt(I) <= (others => '0');
        elsif (cnx_slave_in(c_WB_SLAVE_FMC0_DDR_ADR + 3*I).we = '1' and
               cnx_slave_in(c_WB_SLAVE_FMC0_DDR_ADR + 3*I).stb = '1' and
               cnx_slave_in(c_WB_SLAVE_FMC0_DDR_ADR + 3*I).cyc = '1') then
          ddr_addr_cnt(I) <= unsigned(cnx_slave_in(c_WB_SLAVE_FMC0_DDR_ADR + 3*I).dat);
        elsif (ddr_addr_cnt_en(I) = '1') then
          ddr_addr_cnt(I) <= ddr_addr_cnt(I) + 1;
        end if;
      end if;
    end process p_ddr_addr_cnt;

    -- ack generation
    p_ddr_addr_ack : process (clk_sys_62m5)
    begin
      if rising_edge(clk_sys_62m5) then
        if rst_sys_62m5_n = '0' then
          cnx_slave_out(c_WB_SLAVE_FMC0_DDR_ADR + 3*I).ack <= '0';
        elsif (cnx_slave_in(c_WB_SLAVE_FMC0_DDR_ADR + 3*I).stb = '1' and
               cnx_slave_in(c_WB_SLAVE_FMC0_DDR_ADR + 3*I).cyc = '1') then
          cnx_slave_out(c_WB_SLAVE_FMC0_DDR_ADR + 3*I).ack <= '1';
        else
          cnx_slave_out(c_WB_SLAVE_FMC0_DDR_ADR + 3*I).ack <= '0';
        end if;
      end if;
    end process p_ddr_addr_ack;

    -- Address counter read back
    cnx_slave_out(c_WB_SLAVE_FMC0_DDR_ADR + 3*I).dat <= std_logic_vector(ddr_addr_cnt(I));

    -- Unused wishbone signals
    cnx_slave_out(c_WB_SLAVE_FMC0_DDR_DAT + 3*I).err   <= '0';
    cnx_slave_out(c_WB_SLAVE_FMC0_DDR_DAT + 3*I).rty   <= '0';
    cnx_slave_out(c_WB_SLAVE_FMC0_DDR_ADR + 3*I).err   <= '0';
    cnx_slave_out(c_WB_SLAVE_FMC0_DDR_ADR + 3*I).rty   <= '0';
    cnx_slave_out(c_WB_SLAVE_FMC0_DDR_ADR + 3*I).stall <= '0';

  end generate gen_ddr_ctrl;

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
      pulse_i    => cnx_slave_in(c_WB_MASTER_VME).cyc,
      extended_o => vme_access);

  gen_fmc_led : for I in 0 to g_NB_FMC_SLOTS - 1 generate

    cmp_fmc_cfg_ok_sync : gc_sync_ffs
      port map (
        clk_i    => clk_sys_62m5,
        rst_n_i  => '1',
        data_i   => fmc_acq_cfg_ok(I),
        synced_o => fmc_acq_cfg_ok_sync(I));

  end generate gen_fmc_led;

  -- Logic OR of signals and CSR register for LED control
  svec_led <= led_state or led_state_csr;

  -- LED order on front panel (top to bottom)
  -- 1..0 |  9..8
  -- 3..2 | 11..10
  -- 5..4 | 13..12
  -- 7..6 | 15..14
  led_state(1 downto 0)   <= c_led_off;
  led_state(3 downto 2)   <= c_led_green     when fmc_acq_cfg_ok_sync(0) = '1' else c_led_red;
  led_state(5 downto 4)   <= c_led_green     when pps_led = '1'                else c_led_off;
  led_state(7 downto 6)   <= c_led_green     when wr_led_link = '1'            else c_led_red;
  led_state(9 downto 8)   <= c_led_red_green when vme_access = '1'             else c_led_off;
  led_state(11 downto 10) <= c_led_green     when fmc_acq_cfg_ok_sync(1) = '1' else c_led_red;
  led_state(13 downto 12) <= c_led_green     when tm_time_valid = '1'          else c_led_red;
  led_state(15 downto 14) <= c_led_red_green when wr_led_act = '1'             else c_led_off;

  -- Front panel IO configuration
  fp_gpio1_o      <= pps;
  fp_gpio2_o      <= '0';
  fp_gpio3_o      <= '0';
  fp_gpio4_o      <= '0';
  fp_term_en_o    <= (others => '0');
  fp_gpio1_a2b_o  <= '1';
  fp_gpio2_a2b_o  <= '1';
  fp_gpio34_a2b_o <= '1';

end rtl;
