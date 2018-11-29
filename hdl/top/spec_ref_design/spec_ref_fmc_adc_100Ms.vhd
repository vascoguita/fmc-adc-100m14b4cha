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
-- Copyright (c) 2011-2018 CERN (BE-CO-HT)
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

use work.gn4124_core_pkg.all;
use work.ddr3_ctrl_pkg.all;
use work.gencores_pkg.all;
use work.wishbone_pkg.all;
use work.fmc_adc_mezzanine_pkg.all;
use work.synthesis_descriptor.all;
use work.timetag_core_pkg.all;
use work.carrier_csr_wbgen2_pkg.all;
use work.wr_xilinx_pkg.all;
use work.wr_board_pkg.all;
use work.wr_spec_pkg.all;

entity spec_ref_fmc_adc_100Ms is
  generic(
    g_SIMULATION         : integer := 0;
    g_MULTISHOT_RAM_SIZE : natural := 4096;
    g_CALIB_SOFT_IP      : string  := "TRUE";
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
      pll25dac_sync_n_o : out std_logic;          -- 25MHz VCXO
      pll20dac_sync_n_o : out std_logic;          -- 20MHz VCXO
      plldac_din_o      : out std_logic;
      plldac_sclk_o     : out std_logic;

      -- Carrier front panel LEDs
      led_sfp_red_o   : out std_logic;
      led_sfp_green_o : out std_logic;

      -- Auxiliary pins
      aux_leds_o : out std_logic_vector(3 downto 0);

      -- PCB version
      pcbrev_i : in std_logic_vector(3 downto 0);

      -- Carrier 1-wire interface (DS18B20 thermometer + unique ID)
      carrier_onewire_b : inout std_logic;

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
      -- gn_gpio_b[0] -> AB19 -> GN4124 GPIO9
      -- gn_gpio_b[1] -> U16  -> GN4124 GPIO8
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
      fmc_prsnt_m2c_n_i : in    std_logic;       -- Mezzanine present (active low)
      fmc_scl_b         : inout std_logic;       -- Mezzanine system I2C clock (EEPROM)
      fmc_sda_b         : inout std_logic        -- Mezzanine system I2C data (EEPROM)
      );
end spec_ref_fmc_adc_100Ms;


architecture rtl of spec_ref_fmc_adc_100Ms is

  ------------------------------------------------------------------------------
  -- SDB crossbar constants declaration
  ------------------------------------------------------------------------------

  -- Number of masters on the wishbone crossbar
  constant c_NUM_WB_MASTERS : integer := 1;

  -- Number of slaves on the wishbone crossbar
  constant c_NUM_WB_SLAVES : integer := 6;

  -- Wishbone master(s)
  constant c_WB_MASTER_GENNUM : integer := 0;

  -- Wishbone slave(s)
  constant c_WB_SLAVE_DMA      : integer := 0;    -- DMA controller in the Gennum core
  constant c_WB_SLAVE_SPEC_CSR : integer := 1;    -- SPEC control and status registers
  constant c_WB_SLAVE_VIC      : integer := 2;    -- Vectored interrupt controller
  constant c_WB_SLAVE_DMA_EIC  : integer := 3;    -- DMA interrupt controller
  constant c_WB_SLAVE_FMC_ADC  : integer := 4;    -- FMC ADC mezzanine
  constant c_WB_SLAVE_WR_CORE  : integer := 5;    -- WR PTP core

  -- SDB meta info
  constant c_SDB_GIT_REPO_URL : integer := c_NUM_WB_SLAVES;
  constant c_SDB_SYNTHESIS    : integer := c_NUM_WB_SLAVES + 1;

  -- Devices sdb description
  constant c_WB_DMA_CTRL_SDB : t_sdb_device := (
    abi_class     => x"0000",                     -- undocumented device
    abi_ver_major => x"01",
    abi_ver_minor => x"01",
    wbd_endian    => c_SDB_ENDIAN_BIG,
    wbd_width     => x"4",                        -- 32-bit port granularity
    sdb_component => (
      addr_first  => x"0000000000000000",
      addr_last   => x"000000000000003F",
      product     => (
        vendor_id => x"000000000000CE42",         -- CERN
        device_id => x"00000601",
        version   => x"00000001",
        date      => x"20121116",
        name      => "WB-DMA.Control     ")));

  constant c_WB_SPEC_CSR_SDB : t_sdb_device := (
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
        device_id => x"00000603",
        version   => x"00000001",
        date      => x"20121116",
        name      => "WB-SPEC-CSR        ")));

  constant c_WB_DMA_EIC_SDB : t_sdb_device := (
    abi_class     => x"0000",                     -- undocumented device
    abi_ver_major => x"01",
    abi_ver_minor => x"01",
    wbd_endian    => c_SDB_ENDIAN_BIG,
    wbd_width     => x"4",                        -- 32-bit port granularity
    sdb_component => (
      addr_first  => x"0000000000000000",
      addr_last   => x"000000000000000F",
      product     => (
        vendor_id => x"000000000000CE42",         -- CERN
        device_id => x"d5735ab4",                 -- echo "WB-DMA.EIC         " | md5sum | cut -c1-8
        version   => x"00000001",
        date      => x"20131204",
        name      => "WB-DMA.EIC         ")));

  -- f_xwb_bridge_manual_sdb(size, sdb_addr)
  -- Note: sdb_addr is the sdb records address relative to the bridge base address
  constant c_FMC_BRIDGE_SDB     : t_sdb_bridge := f_xwb_bridge_manual_sdb(x"00001fff", x"00000000");
  constant c_WR_CORE_BRIDGE_SDB : t_sdb_bridge := f_xwb_bridge_manual_sdb(x"0003ffff", x"00030000");

  -- sdb header address
  constant c_SDB_ADDRESS : t_wishbone_address := x"00000000";

  -- Wishbone crossbar layout
  constant c_INTERCONNECT_LAYOUT : t_sdb_record_array(c_NUM_WB_SLAVES + 1 downto 0) :=
    (
      c_WB_SLAVE_DMA      => f_sdb_embed_device(c_WB_DMA_CTRL_SDB,    x"00001000"),
      c_WB_SLAVE_SPEC_CSR => f_sdb_embed_device(c_WB_SPEC_CSR_SDB,    x"00001200"),
      c_WB_SLAVE_VIC      => f_sdb_embed_device(c_XWB_VIC_SDB,        x"00001300"),
      c_WB_SLAVE_DMA_EIC  => f_sdb_embed_device(c_WB_DMA_EIC_SDB,     x"00001400"),
      c_WB_SLAVE_FMC_ADC  => f_sdb_embed_bridge(c_FMC_BRIDGE_SDB,     x"00002000"),
      c_WB_SLAVE_WR_CORE  => f_sdb_embed_bridge(c_WR_CORE_BRIDGE_SDB, x"00040000"),
      c_SDB_GIT_REPO_URL  => f_sdb_embed_repo_url(c_SDB_REPO_URL),
      c_SDB_SYNTHESIS     => f_sdb_embed_synthesis(c_SDB_SYNTHESIS_INFO));

  -- VIC default vector setting
  constant c_VIC_VECTOR_TABLE : t_wishbone_address_array(0 to 1) :=
    (0 => x"00003500",
     1 => x"00001400");

  ------------------------------------------------------------------------------
  -- Other constants declaration
  ------------------------------------------------------------------------------

  -- WRPC Xilinx platform auxiliary clock configuration, used for DDR clock
  constant c_WRPC_PLL_CONFIG : t_px_pll_cfg := (
    enabled => TRUE, divide => 3, multiply => 8 );

  -- SPEC carrier CSR constants
  constant c_CARRIER_TYPE : std_logic_vector(15 downto 0) := X"0001";

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

  -- Clocks and resets
  signal clk_sys_62m5       : std_logic;
  signal clk_ref_125m       : std_logic;
  signal sys_clk_pll_locked : std_logic;
  signal clk_ddr_333m       : std_logic;

  signal rst_sys_62m5_n  : std_logic := '0';
  signal rst_ref_125m_n  : std_logic := '0';
  signal rst_ddr_333m_n  : std_logic := '0';
  signal sw_rst_fmc      : std_logic := '1';
  signal sw_rst_fmc_sync : std_logic := '1';
  signal fmc_rst_ref_n   : std_logic := '0';
  signal fmc_rst_sys_n   : std_logic := '0';
  signal ddr_rst         : std_logic := '1';

  attribute keep                 : string;
  attribute keep of clk_sys_62m5 : signal is "TRUE";
  attribute keep of clk_ref_125m : signal is "TRUE";
  attribute keep of clk_ddr_333m : signal is "TRUE";
  attribute keep of ddr_rst      : signal is "TRUE";

  -- GN4124
  signal gn4124_status : std_logic_vector(31 downto 0);
  signal gn4124_access : std_logic;

  -- Wishbone buse(s) from master(s) to crossbar slave port(s)
  signal cnx_master_out : t_wishbone_master_out_array(c_NUM_WB_MASTERS-1 downto 0);
  signal cnx_master_in  : t_wishbone_master_in_array(c_NUM_WB_MASTERS-1 downto 0);

  -- Wishbone buse(s) from crossbar master port(s) to slave(s)
  signal cnx_slave_out : t_wishbone_slave_out_array(c_NUM_WB_SLAVES-1 downto 0);
  signal cnx_slave_in  : t_wishbone_slave_in_array(c_NUM_WB_SLAVES-1 downto 0);

  -- Wishbone bus from cross-clocking module to FMC mezzanine
  signal cnx_fmc_sync_master_out : t_wishbone_master_out;
  signal cnx_fmc_sync_master_in  : t_wishbone_master_in;

  -- GN4124 core DMA port to DDR wishbone bus
  signal gn_wb_ddr_in  : t_wishbone_master_in;
  signal gn_wb_ddr_out : t_wishbone_master_out;

  -- FMC ADC core to DDR wishbone bus
  signal fmc_wb_ddr_in  : t_wishbone_master_data64_in;
  signal fmc_wb_ddr_out : t_wishbone_master_data64_out;

  -- Interrupts and status
  signal dma_irq           : std_logic_vector(1 downto 0);
  signal irq_sources       : std_logic_vector(3 downto 0);
  signal irq_to_gn4124     : std_logic;
  signal irq_sources_2_led : std_logic_vector(3 downto 0);
  signal ddr_wr_fifo_empty : std_logic;
  signal dma_eic_irq       : std_logic;
  signal fmc_irq           : std_logic;
  signal fmc_acq_cfg_ok    : std_logic;

  -- Resync interrupts to sys domain
  signal dma_irq_sync           : std_logic_vector(1 downto 0);
  signal ddr_wr_fifo_empty_sync : std_logic;
  signal fmc_irq_sync           : std_logic;

  -- Front panel LED control
  signal led_red   : std_logic;
  signal led_green : std_logic;

  -- DDR
  signal ddr_status     : std_logic_vector(31 downto 0);
  signal ddr_calib_done : std_logic;

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
  signal pps_led     : std_logic;
  signal wr_led_act  : std_logic;
  signal wr_led_link : std_logic;

  -- WR PTP core timing interface
  signal tm_link_up         : std_logic;
  signal tm_tai             : std_logic_vector(39 downto 0);
  signal tm_cycles          : std_logic_vector(27 downto 0);
  signal tm_time_valid      : std_logic;
  -- re-synced to ref clock
  signal tm_time_valid_sync : std_logic;

  -- IO for CSR registers
  signal csr_regin  : t_carrier_csr_in_registers;
  signal csr_regout : t_carrier_csr_out_registers;

begin

  ------------------------------------------------------------------------------
  -- Reset logic
  ------------------------------------------------------------------------------

  sys_clk_pll_locked <= '1';

  -- reset for mezzanine
  -- including soft reset, with re-sync from 62.5MHz domain
  -- and registers to help with timing
  cmp_fmc_sw_reset_sync : gc_sync_ffs
    port map (
      clk_i    => clk_ref_125m,
      rst_n_i  => '1',
      data_i   => sw_rst_fmc,
      synced_o => sw_rst_fmc_sync);

  fmc_rst_ref_n <= rst_ref_125m_n and not sw_rst_fmc_sync;
  fmc_rst_sys_n <= rst_sys_62m5_n and not sw_rst_fmc;

  -- reset for DDR including soft reset.
  -- This is treated as async and will be re-synced by the DDR controller
  ddr_rst <= not rst_ddr_333m_n or sw_rst_fmc;

  ------------------------------------------------------------------------------
  -- GN4124 interface
  ------------------------------------------------------------------------------
  cmp_gn4124_core : xwb_gn4124_core
    port map (
      rst_n_a_i          => gn_rst_n_i,
      status_o           => gn4124_status,
      p2l_clk_p_i        => gn_p2l_clk_p_i,
      p2l_clk_n_i        => gn_p2l_clk_n_i,
      p2l_data_i         => gn_p2l_data_i,
      p2l_dframe_i       => gn_p2l_dframe_i,
      p2l_valid_i        => gn_p2l_valid_i,
      p2l_rdy_o          => gn_p2l_rdy_o,
      p_wr_req_i         => gn_p_wr_req_i,
      p_wr_rdy_o         => gn_p_wr_rdy_o,
      rx_error_o         => gn_rx_error_o,
      l2p_clk_p_o        => gn_l2p_clk_p_o,
      l2p_clk_n_o        => gn_l2p_clk_n_o,
      l2p_data_o         => gn_l2p_data_o,
      l2p_dframe_o       => gn_l2p_dframe_o,
      l2p_valid_o        => gn_l2p_valid_o,
      l2p_edb_o          => gn_l2p_edb_o,
      l2p_rdy_i          => gn_l2p_rdy_i,
      l_wr_rdy_i         => gn_l_wr_rdy_i,
      p_rd_d_rdy_i       => gn_p_rd_d_rdy_i,
      tx_error_i         => gn_tx_error_i,
      vc_rdy_i           => gn_vc_rdy_i,
      dma_irq_o          => dma_irq,
      irq_p_i            => irq_to_gn4124,
      irq_p_o            => gn_gpio_b(1),
      wb_master_clk_i    => clk_sys_62m5,
      wb_master_rst_n_i  => rst_sys_62m5_n,
      wb_master_i        => cnx_master_in(c_WB_MASTER_GENNUM),
      wb_master_o        => cnx_master_out(c_WB_MASTER_GENNUM),
      wb_dma_cfg_clk_i   => clk_sys_62m5,
      wb_dma_cfg_rst_n_i => rst_sys_62m5_n,
      wb_dma_cfg_i       => cnx_slave_in(c_WB_SLAVE_DMA),
      wb_dma_cfg_o       => cnx_slave_out(c_WB_SLAVE_DMA),
      wb_dma_dat_clk_i   => clk_sys_62m5,
      wb_dma_dat_rst_n_i => rst_sys_62m5_n,
      wb_dma_dat_i       => gn_wb_ddr_in,
      wb_dma_dat_o       => gn_wb_ddr_out);

  -- Assign unused outputs
  gn_gpio_b(0) <= '0';

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
  -- White Rabbit Core (SPEC board package)
  -------------------------------------------------------------------------------

  -- Tristates for SFP EEPROM
  sfp_mod_def1_b <= '0' when sfp_scl_out = '0' else 'Z';
  sfp_mod_def2_b <= '0' when sfp_sda_out = '0' else 'Z';
  sfp_scl_in     <= sfp_mod_def1_b;
  sfp_sda_in     <= sfp_mod_def2_b;

  -- Tristates for Carrier OneWire
  carrier_onewire_b <= '0' when onewire_oe = '1' else 'Z';
  onewire_data      <= carrier_onewire_b;

  cmp_xwrc_board_spec : xwrc_board_spec
    generic map (
      g_SIMULATION                => g_SIMULATION,
      g_WITH_EXTERNAL_CLOCK_INPUT => FALSE,
      g_DPRAM_INITF               => g_WRPC_INITF,
      g_AUX_PLL_CONFIG            => c_WRPC_PLL_CONFIG,
      g_FABRIC_IFACE              => PLAIN)
    port map (
      clk_20m_vcxo_i      => clk_20m_vcxo_i,
      clk_125m_pllref_p_i => clk_125m_pllref_p_i,
      clk_125m_pllref_n_i => clk_125m_pllref_n_i,
      clk_125m_gtp_n_i    => clk_125m_gtp_n_i,
      clk_125m_gtp_p_i    => clk_125m_gtp_p_i,
      areset_n_i          => button1_n_i,
      areset_edge_n_i     => gn_rst_n_i,
      clk_sys_62m5_o      => clk_sys_62m5,
      clk_ref_125m_o      => clk_ref_125m,
      clk_pll_aux_o       => clk_ddr_333m,
      rst_sys_62m5_n_o    => rst_sys_62m5_n,
      rst_ref_125m_n_o    => rst_ref_125m_n,
      rst_pll_aux_n_o     => rst_ddr_333m_n,
      plldac_sclk_o       => plldac_sclk_o,
      plldac_din_o        => plldac_din_o,
      pll25dac_cs_n_o     => pll25dac_sync_n_o,
      pll20dac_cs_n_o     => pll20dac_sync_n_o,
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
      onewire_i           => onewire_data,
      onewire_oen_o       => onewire_oe,
      uart_rxd_i          => uart_rxd_i,
      uart_txd_o          => uart_txd_o,
      flash_sclk_o        => spi_sclk_o,
      flash_ncs_o         => spi_ncs_o,
      flash_mosi_o        => spi_mosi_o,
      flash_miso_i        => spi_miso_i,
      wb_slave_o          => cnx_slave_out(c_WB_SLAVE_WR_CORE),
      wb_slave_i          => cnx_slave_in(c_WB_SLAVE_WR_CORE),
      tm_link_up_o        => tm_link_up,
      tm_time_valid_o     => tm_time_valid,
      tm_tai_o            => tm_tai,
      tm_cycles_o         => tm_cycles,
      pps_p_o             => open,
      pps_led_o           => pps_led,
      led_link_o          => wr_led_link,
      led_act_o           => wr_led_act,
      link_ok_o           => wrabbit_en);

  ------------------------------------------------------------------------------
  -- Carrier CSR
  --    Carrier type and PCB version
  --    Bitstream (firmware) type and date
  --    Release tag
  --    VCXO DAC control (CLR_N)
  ------------------------------------------------------------------------------
  cmp_carrier_csr : entity work.carrier_csr
    port map (
      rst_n_i    => rst_sys_62m5_n,
      clk_sys_i  => clk_sys_62m5,
      wb_adr_i   => cnx_slave_in(c_WB_SLAVE_SPEC_CSR).adr(3 downto 2),  -- cnx_slave_in.adr is byte address
      wb_dat_i   => cnx_slave_in(c_WB_SLAVE_SPEC_CSR).dat,
      wb_dat_o   => cnx_slave_out(c_WB_SLAVE_SPEC_CSR).dat,
      wb_cyc_i   => cnx_slave_in(c_WB_SLAVE_SPEC_CSR).cyc,
      wb_sel_i   => cnx_slave_in(c_WB_SLAVE_SPEC_CSR).sel,
      wb_stb_i   => cnx_slave_in(c_WB_SLAVE_SPEC_CSR).stb,
      wb_we_i    => cnx_slave_in(c_WB_SLAVE_SPEC_CSR).we,
      wb_ack_o   => cnx_slave_out(c_WB_SLAVE_SPEC_CSR).ack,
      wb_stall_o => open,
      regs_i     => csr_regin,
      regs_o     => csr_regout);

  csr_regin.carrier_pcb_rev_i    <= pcbrev_i;
  csr_regin.carrier_reserved_i   <= (others => '0');
  csr_regin.carrier_type_i       <= c_CARRIER_TYPE;
  csr_regin.stat_fmc_pres_i      <= fmc_prsnt_m2c_n_i;
  csr_regin.stat_p2l_pll_lck_i   <= gn4124_status(0);
  csr_regin.stat_sys_pll_lck_i   <= sys_clk_pll_locked;
  csr_regin.stat_ddr3_cal_done_i <= ddr_calib_done;

  led_red    <= csr_regout.ctrl_led_red_o;
  led_green  <= csr_regout.ctrl_led_green_o;
  sw_rst_fmc <= csr_regout.rst_fmc0_o;

  -- Unused wishbone signals
  cnx_slave_out(c_WB_SLAVE_SPEC_CSR).err   <= '0';
  cnx_slave_out(c_WB_SLAVE_SPEC_CSR).rty   <= '0';
  cnx_slave_out(c_WB_SLAVE_SPEC_CSR).stall <= '0';

  ------------------------------------------------------------------------------
  -- Vectored interrupt controller (VIC)
  ------------------------------------------------------------------------------

  cmp_fmc_irq_sync : gc_sync_ffs
    port map (
      clk_i    => clk_sys_62m5,
      rst_n_i  => '1',
      data_i   => fmc_irq,
      synced_o => fmc_irq_sync);

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
      irqs_i(0)    => fmc_irq_sync,
      irqs_i(1)    => dma_eic_irq,
      irq_master_o => irq_to_gn4124);

  ------------------------------------------------------------------------------
  -- GN4124 DMA interrupt controller
  ------------------------------------------------------------------------------

  gen_dma_irq : for I in 0 to 1 generate

    cmp_dma_irq_sync : gc_sync_ffs
      port map (
        clk_i    => clk_sys_62m5,
        rst_n_i  => '1',
        data_i   => dma_irq(I),
        synced_o => dma_irq_sync(I));

  end generate gen_dma_irq;

  cmp_dma_eic : entity work.dma_eic
    port map (
      rst_n_i         => rst_sys_62m5_n,
      clk_sys_i       => clk_sys_62m5,
      wb_adr_i        => cnx_slave_in(c_WB_SLAVE_DMA_EIC).adr(3 downto 2),  -- cnx_slave_in.adr is byte address
      wb_dat_i        => cnx_slave_in(c_WB_SLAVE_DMA_EIC).dat,
      wb_dat_o        => cnx_slave_out(c_WB_SLAVE_DMA_EIC).dat,
      wb_cyc_i        => cnx_slave_in(c_WB_SLAVE_DMA_EIC).cyc,
      wb_sel_i        => cnx_slave_in(c_WB_SLAVE_DMA_EIC).sel,
      wb_stb_i        => cnx_slave_in(c_WB_SLAVE_DMA_EIC).stb,
      wb_we_i         => cnx_slave_in(c_WB_SLAVE_DMA_EIC).we,
      wb_ack_o        => cnx_slave_out(c_WB_SLAVE_DMA_EIC).ack,
      wb_stall_o      => cnx_slave_out(c_WB_SLAVE_DMA_EIC).stall,
      wb_int_o        => dma_eic_irq,
      irq_dma_done_i  => dma_irq_sync(0),
      irq_dma_error_i => dma_irq_sync(1)
      );

  -- Unused wishbone signals
  cnx_slave_out(c_WB_SLAVE_DMA_EIC).err <= '0';
  cnx_slave_out(c_WB_SLAVE_DMA_EIC).rty <= '0';

  ------------------------------------------------------------------------------
  -- FMC ADC mezzanines (wb bridge with cross-clocking)
  --    Mezzanine system managment I2C master
  --    Mezzanine SPI master
  --    Mezzanine I2C
  --    ADC core
  --    Mezzanine 1-wire master
  ------------------------------------------------------------------------------

  cmp_xwb_clock_bridge : xwb_clock_bridge
    port map (
      slave_clk_i    => clk_sys_62m5,
      slave_rst_n_i  => fmc_rst_sys_n,
      slave_i        => cnx_slave_in(c_WB_SLAVE_FMC_ADC),
      slave_o        => cnx_slave_out(c_WB_SLAVE_FMC_ADC),
      master_clk_i   => clk_ref_125m,
      master_rst_n_i => fmc_rst_ref_n,
      master_i       => cnx_fmc_sync_master_in,
      master_o       => cnx_fmc_sync_master_out
      );

  cmp_fmc_ddr_wr_fifo_sync : gc_sync_ffs
    port map (
      clk_i    => clk_ref_125m,
      rst_n_i  => '1',
      data_i   => ddr_wr_fifo_empty,
      synced_o => ddr_wr_fifo_empty_sync);

  cmp_fmc_adc_mezzanine : fmc_adc_mezzanine
    generic map (
      g_MULTISHOT_RAM_SIZE => g_MULTISHOT_RAM_SIZE,
      g_WB_MODE            => PIPELINED,
      g_WB_GRANULARITY     => BYTE)
    port map (
      sys_clk_i   => clk_ref_125m,
      sys_rst_n_i => fmc_rst_ref_n,

      wb_csr_slave_i => cnx_fmc_sync_master_out,
      wb_csr_slave_o => cnx_fmc_sync_master_in,

      wb_ddr_clk_i    => clk_ref_125m,
      wb_ddr_rst_n_i  => fmc_rst_ref_n,
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

      sys_scl_b => fmc_scl_b,
      sys_sda_b => fmc_sda_b,

      wr_tm_link_up_i    => tm_link_up,
      wr_tm_time_valid_i => tm_time_valid,
      wr_tm_tai_i        => tm_tai,
      wr_tm_cycles_i     => tm_cycles,
      wr_enable_i        => wrabbit_en
      );

  ------------------------------------------------------------------------------
  -- DMA wishbone bus slaves
  --  -> DDR3 controller
  ------------------------------------------------------------------------------
  cmp_ddr_ctrl_bank3 : ddr3_ctrl
    generic map(
      g_RST_ACT_LOW        => 0, -- active high reset (simpler internal logic)
      g_BANK_PORT_SELECT   => "SPEC_BANK3_64B_32B",
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
      rst_n_i => ddr_rst,

      status_o => ddr_status,

      ddr3_dq_b     => ddr_dq_b,
      ddr3_a_o      => ddr_a_o,
      ddr3_ba_o     => ddr_ba_o,
      ddr3_ras_n_o  => ddr_ras_n_o,
      ddr3_cas_n_o  => ddr_cas_n_o,
      ddr3_we_n_o   => ddr_we_n_o,
      ddr3_odt_o    => ddr_odt_o,
      ddr3_rst_n_o  => ddr_reset_n_o,
      ddr3_cke_o    => ddr_cke_o,
      ddr3_dm_o     => ddr_ldm_o,
      ddr3_udm_o    => ddr_udm_o,
      ddr3_dqs_p_b  => ddr_ldqs_p_b,
      ddr3_dqs_n_b  => ddr_ldqs_n_b,
      ddr3_udqs_p_b => ddr_udqs_p_b,
      ddr3_udqs_n_b => ddr_udqs_n_b,
      ddr3_clk_p_o  => ddr_ck_p_o,
      ddr3_clk_n_o  => ddr_ck_n_o,
      ddr3_rzq_b    => ddr_rzq_b,

      wb0_rst_n_i => fmc_rst_ref_n,
      wb0_clk_i   => clk_ref_125m,
      wb0_sel_i   => fmc_wb_ddr_out.sel,
      wb0_cyc_i   => fmc_wb_ddr_out.cyc,
      wb0_stb_i   => fmc_wb_ddr_out.stb,
      wb0_we_i    => fmc_wb_ddr_out.we,
      wb0_addr_i  => fmc_wb_ddr_out.adr,
      wb0_data_i  => fmc_wb_ddr_out.dat,
      wb0_data_o  => fmc_wb_ddr_in.dat,
      wb0_ack_o   => fmc_wb_ddr_in.ack,
      wb0_stall_o => fmc_wb_ddr_in.stall,

      p0_cmd_empty_o   => open,
      p0_cmd_full_o    => open,
      p0_rd_full_o     => open,
      p0_rd_empty_o    => open,
      p0_rd_count_o    => open,
      p0_rd_overflow_o => open,
      p0_rd_error_o    => open,
      p0_wr_full_o     => open,
      p0_wr_empty_o    => ddr_wr_fifo_empty,
      p0_wr_count_o    => open,
      p0_wr_underrun_o => open,
      p0_wr_error_o    => open,

      wb1_rst_n_i => rst_sys_62m5_n,
      wb1_clk_i   => clk_sys_62m5,
      wb1_sel_i   => gn_wb_ddr_out.sel,
      wb1_cyc_i   => gn_wb_ddr_out.cyc,
      wb1_stb_i   => gn_wb_ddr_out.stb,
      wb1_we_i    => gn_wb_ddr_out.we,
      wb1_addr_i  => gn_wb_ddr_out.adr,
      wb1_data_i  => gn_wb_ddr_out.dat,
      wb1_data_o  => gn_wb_ddr_in.dat,
      wb1_ack_o   => gn_wb_ddr_in.ack,
      wb1_stall_o => gn_wb_ddr_in.stall,

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

  ddr_calib_done <= ddr_status(0);

  -- unused Wishbone signals
  gn_wb_ddr_in.err <= '0';
  gn_wb_ddr_in.rty <= '0';

  fmc_wb_ddr_in.err <= '0';
  fmc_wb_ddr_in.rty <= '0';

  ------------------------------------------------------------------------------
  -- Carrier LEDs
  ------------------------------------------------------------------------------

  cmp_pci_access_led : gc_extend_pulse
    generic map (
      g_width => 2500000)
    port map (
      clk_i      => clk_sys_62m5,
      rst_n_i    => rst_sys_62m5_n,
      pulse_i    => cnx_slave_in(c_WB_MASTER_GENNUM).cyc,
      extended_o => gn4124_access);

  aux_leds_o(0) <= not gn4124_access;
  aux_leds_o(1) <= not fmc_acq_cfg_ok;
  aux_leds_o(2) <= not tm_time_valid;
  aux_leds_o(3) <= not pps_led;

  -- SPEC front panel leds
  led_sfp_red_o   <= led_red or wr_led_act;
  led_sfp_green_o <= led_green or wr_led_link;

end rtl;
