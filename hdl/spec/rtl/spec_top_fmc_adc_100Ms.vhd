-------------------------------------------------------------------------------
-- Title      : FMC ADC 100Ms/s SPEC top-level
-- Project    : FMC ADC 100M 14B 4CHA gateware
-- URL        : http://www.ohwr.org/projects/fmc-adc-100m14b4cha-gw
-------------------------------------------------------------------------------
-- File       : spec_top_fmc_adc_100Ms.vhd
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
use work.sdb_meta_pkg.all;
use work.timetag_core_pkg.all;
use work.carrier_csr_wbgen2_pkg.all;
use work.wrcore_pkg.all;
use work.endpoint_pkg.all;
use work.wr_board_pkg.all;
use work.wr_xilinx_pkg.all;

entity spec_top_fmc_adc_100Ms is
  generic(
    g_simulation         : integer := 0;
    g_multishot_ram_size : natural := 4096;
    g_CALIB_SOFT_IP      : string  := "TRUE";
    g_wrpc_initf         : string  := "../../ip_cores/wr-cores/bin/wrpc/wrc_phy8.bram");
  port
    (
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
      led_red_o   : out std_logic;
      led_green_o : out std_logic;

      -- Reset button
      button1_n_i : in  std_logic;

      -- Auxiliary pins
      aux_leds_o : out std_logic_vector(3 downto 0);

      -- PCB version
      pcb_ver_i : in std_logic_vector(3 downto 0);

      -- Carrier 1-wire interface (DS18B20 thermometer + unique ID)
      carrier_onewire_b : inout std_logic;

      -- GN4124 interface
      L_RST_N      : in    std_logic;                      -- Reset from GN4124 (RSTOUT18_N)
      P2L_RDY      : out   std_logic;                      -- Rx Buffer Full Flag
      P2L_CLKn     : in    std_logic;                      -- Receiver Source Synchronous Clock-
      P2L_CLKp     : in    std_logic;                      -- Receiver Source Synchronous Clock+
      P2L_DATA     : in    std_logic_vector(15 downto 0);  -- Parallel receive data
      P2L_DFRAME   : in    std_logic;                      -- Receive Frame
      P2L_VALID    : in    std_logic;                      -- Receive Data Valid
      --P_WR_REQ     : in    std_logic_vector(1 downto 0);   -- PCIe Write Request
      P_WR_RDY     : out   std_logic_vector(1 downto 0);   -- PCIe Write Ready
      RX_ERROR     : out   std_logic;                      -- Receive Error
      L2P_DATA     : out   std_logic_vector(15 downto 0);  -- Parallel transmit data
      L2P_DFRAME   : out   std_logic;                      -- Transmit Data Frame
      L2P_VALID    : out   std_logic;                      -- Transmit Data Valid
      L2P_CLKn     : out   std_logic;                      -- Transmitter Source Synchronous Clock-
      L2P_CLKp     : out   std_logic;                      -- Transmitter Source Synchronous Clock+
      L2P_EDB      : out   std_logic;                      -- Packet termination and discard
      L2P_RDY      : in    std_logic;                      -- Tx Buffer Full Flag
      L_WR_RDY     : in    std_logic_vector(1 downto 0);   -- Local-to-PCIe Write
      P_RD_D_RDY   : in    std_logic_vector(1 downto 0);   -- PCIe-to-Local Read Response Data Ready
      TX_ERROR     : in    std_logic;                      -- Transmit Error
      --VC_RDY       : in    std_logic_vector(1 downto 0);   -- Channel ready
      GPIO         : inout std_logic_vector(1 downto 0);   -- GPIO[0] -> GN4124 GPIO8
                                                           -- GPIO[1] -> GN4124 GPIO9
      -- DDR3 interface
      DDR3_CAS_N   : out   std_logic;
      DDR3_CK_N    : out   std_logic;
      DDR3_CK_P    : out   std_logic;
      DDR3_CKE     : out   std_logic;
      DDR3_LDM     : out   std_logic;
      DDR3_LDQS_N  : inout std_logic;
      DDR3_LDQS_P  : inout std_logic;
      DDR3_ODT     : out   std_logic;
      DDR3_RAS_N   : out   std_logic;
      DDR3_RESET_N : out   std_logic;
      DDR3_UDM     : out   std_logic;
      DDR3_UDQS_N  : inout std_logic;
      DDR3_UDQS_P  : inout std_logic;
      DDR3_WE_N    : out   std_logic;
      DDR3_DQ      : inout std_logic_vector(15 downto 0);
      DDR3_A       : out   std_logic_vector(13 downto 0);
      DDR3_BA      : out   std_logic_vector(2 downto 0);
      DDR3_ZIO     : inout std_logic;
      DDR3_RZQ     : inout std_logic;

      -- SFP
      sfp_txp_o         : out   std_logic;
      sfp_txn_o         : out   std_logic;
      sfp_rxp_i         : in    std_logic;
      sfp_rxn_i         : in    std_logic;
      sfp_mod_def0_i    : in    std_logic;        -- sfp detect
      sfp_mod_def1_b    : inout std_logic;        -- scl
      sfp_mod_def2_b    : inout std_logic;        -- sda
      sfp_rate_select_b : inout std_logic;
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

      -- FMC slot
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

      -- FMC slot management
      fmc0_prsnt_m2c_n_i : in    std_logic;       -- Mezzanine present (active low)
      fmc0_sys_scl_b     : inout std_logic;       -- Mezzanine system I2C clock (EEPROM)
      fmc0_sys_sda_b     : inout std_logic        -- Mezzanine system I2C data (EEPROM)
      );
end spec_top_fmc_adc_100Ms;


architecture rtl of spec_top_fmc_adc_100Ms is

  ------------------------------------------------------------------------------
  -- Components declaration
  ------------------------------------------------------------------------------
  component carrier_csr is
    port (
      rst_n_i    : in  std_logic;
      clk_sys_i  : in  std_logic;
      wb_adr_i   : in  std_logic_vector(1 downto 0);
      wb_dat_i   : in  std_logic_vector(31 downto 0);
      wb_dat_o   : out std_logic_vector(31 downto 0);
      wb_cyc_i   : in  std_logic;
      wb_sel_i   : in  std_logic_vector(3 downto 0);
      wb_stb_i   : in  std_logic;
      wb_we_i    : in  std_logic;
      wb_ack_o   : out std_logic;
      wb_stall_o : out std_logic;
      regs_i     : in  t_carrier_csr_in_registers;
      regs_o     : out t_carrier_csr_out_registers);
  end component carrier_csr;

  component dma_eic
    port (
      rst_n_i         : in  std_logic;
      clk_sys_i       : in  std_logic;
      wb_adr_i        : in  std_logic_vector(1 downto 0);
      wb_dat_i        : in  std_logic_vector(31 downto 0);
      wb_dat_o        : out std_logic_vector(31 downto 0);
      wb_cyc_i        : in  std_logic;
      wb_sel_i        : in  std_logic_vector(3 downto 0);
      wb_stb_i        : in  std_logic;
      wb_we_i         : in  std_logic;
      wb_ack_o        : out std_logic;
      wb_stall_o      : out std_logic;
      wb_int_o        : out std_logic;
      irq_dma_done_i  : in  std_logic;
      irq_dma_error_i : in  std_logic
      );
  end component dma_eic;

  ------------------------------------------------------------------------------
  -- SDB crossbar constants declaration
  --
  -- WARNING: All address in sdb and crossbar are BYTE addresses!
  ------------------------------------------------------------------------------

  -- Number of master port(s) on the wishbone crossbar
  constant c_NUM_WB_MASTERS : integer := 6;

  -- Number of slave port(s) on the wishbone crossbar
  constant c_NUM_WB_SLAVES : integer := 1;

  -- Wishbone master(s)
  constant c_MASTER_GENNUM : integer := 0;

  -- Wishbone slave(s)
  constant c_WB_SLAVE_DMA      : integer := 0;    -- DMA controller in the Gennum core
  constant c_WB_SLAVE_SPEC_CSR : integer := 1;    -- SPEC control and status registers
  constant c_WB_SLAVE_VIC      : integer := 2;    -- Vectored interrupt controller
  constant c_WB_SLAVE_DMA_EIC  : integer := 3;    -- DMA interrupt controller
  constant c_WB_SLAVE_FMC_ADC  : integer := 4;    -- FMC ADC mezzanine
  constant c_WB_SLAVE_WR_CORE  : integer := 5;    -- WR PTP core

  -- SDB meta info
  constant c_SDB_REPO_URL  : integer := c_NUM_WB_MASTERS;
  constant c_SDB_SYNTHESIS : integer := c_NUM_WB_MASTERS + 1;
  constant c_SDB_INTEGRATE : integer := c_NUM_WB_MASTERS + 2;

  -- Devices sdb description
  constant c_wb_dma_ctrl_sdb : t_sdb_device := (
    abi_class     => x"0000",                     -- undocumented device
    abi_ver_major => x"01",
    abi_ver_minor => x"01",
    wbd_endian    => c_sdb_endian_big,
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

  constant c_wb_spec_csr_sdb : t_sdb_device := (
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
        device_id => x"00000603",
        version   => x"00000001",
        date      => x"20121116",
        name      => "WB-SPEC-CSR        ")));

  constant c_wb_dma_eic_sdb : t_sdb_device := (
    abi_class     => x"0000",                     -- undocumented device
    abi_ver_major => x"01",
    abi_ver_minor => x"01",
    wbd_endian    => c_sdb_endian_big,
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
  constant c_fmc0_bridge_sdb    : t_sdb_bridge := f_xwb_bridge_manual_sdb(x"00001fff", x"00000000");
  constant c_wr_core_bridge_sdb : t_sdb_bridge := f_xwb_bridge_manual_sdb(x"0003ffff", x"00030000");

  -- sdb header address
  constant c_SDB_ADDRESS : t_wishbone_address := x"00000000";

  -- Wishbone crossbar layout
  constant c_INTERCONNECT_LAYOUT : t_sdb_record_array(c_NUM_WB_MASTERS + 2 downto 0) :=
    (
      c_WB_SLAVE_DMA      => f_sdb_embed_device(c_wb_dma_ctrl_sdb, x"00001000"),
      c_WB_SLAVE_SPEC_CSR => f_sdb_embed_device(c_wb_spec_csr_sdb, x"00001200"),
      c_WB_SLAVE_VIC      => f_sdb_embed_device(c_xwb_vic_sdb, x"00001300"),
      c_WB_SLAVE_DMA_EIC  => f_sdb_embed_device(c_wb_dma_eic_sdb, x"00001400"),
      c_WB_SLAVE_FMC_ADC  => f_sdb_embed_bridge(c_fmc0_bridge_sdb, x"00002000"),
      c_WB_SLAVE_WR_CORE  => f_sdb_embed_bridge(c_wr_core_bridge_sdb, x"00040000"),
      c_SDB_REPO_URL      => f_sdb_embed_repo_url(c_repo_url_sdb),
      c_SDB_SYNTHESIS     => f_sdb_embed_synthesis(c_synthesis_sdb),
      c_SDB_INTEGRATE     => f_sdb_embed_integration(c_integration_sdb)
      );

  -- VIC default vector setting
  constant c_VIC_VECTOR_TABLE : t_wishbone_address_array(0 to 1) :=
    (0 => x"00003500",
     1 => x"00001400");

  ------------------------------------------------------------------------------
  -- Other constants declaration
  ------------------------------------------------------------------------------

  -- SPEC carrier CSR constants
  constant c_CARRIER_TYPE : std_logic_vector(15 downto 0) := X"0001";

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

  -- System clock
  signal sys_clk_62_5_buf    : std_logic;
  signal sys_clk_62_5        : std_logic;
  signal sys_clk_125_buf     : std_logic;
  signal sys_clk_125         : std_logic;
  signal sys_clk_pll_locked  : std_logic;
  signal clk_dmtd_locked     : std_logic;
  signal dmtd_clk_fb         : std_logic;
  signal clk_125m_pllref     : std_logic;
  signal clk_125m_pllref_buf : std_logic;
  signal clk_20m_vcxo_buf    : std_logic;
  signal pllout_clk_dmtd     : std_logic;
  signal clk_dmtd            : std_logic;
  signal pll_locked          : std_logic;

  -- DDR3 clock
  signal ddr_clk     : std_logic;
  signal ddr_clk_buf : std_logic;

  -- Reset
  signal powerup_arst_n   : std_logic := '0';
  signal powerup_clk_in   : std_logic_vector(2 downto 0);
  signal powerup_rst_out  : std_logic_vector(2 downto 0);
  signal sys_rst_62_5_n   : std_logic;
  signal sys_rst_125_n    : std_logic;
  signal sw_rst_fmc0      : std_logic := '1';
  signal sw_rst_ddr_sync  : std_logic;
  signal fmc0_rst_n       : std_logic;
  signal ddr_rst_n        : std_logic;
  signal pll_arst         : std_logic;
  signal arst_edge_ppulse : std_logic;

  -- Wishbone buse(s) from crossbar master port(s)
  signal cnx_master_out : t_wishbone_master_out_array(c_NUM_WB_MASTERS-1 downto 0);
  signal cnx_master_in  : t_wishbone_master_in_array(c_NUM_WB_MASTERS-1 downto 0);

  -- Wishbone buse(s) to crossbar slave port(s)
  signal cnx_slave_out : t_wishbone_slave_out_array(c_NUM_WB_SLAVES-1 downto 0);
  signal cnx_slave_in  : t_wishbone_slave_in_array(c_NUM_WB_SLAVES-1 downto 0);

  -- Wishbone bus from cross-clocking module to FMC0 mezzanine
  signal cnx_fmc0_sync_master_out : t_wishbone_master_out;
  signal cnx_fmc0_sync_master_in  : t_wishbone_master_in;

  -- Wishbone address from GN4124 core (32-bit word address)
  signal gn_wb_adr : std_logic_vector(31 downto 0);

  -- Wishbone address from to DMA controller (32-bit word address)
  signal dma_ctrl_wb_adr : std_logic_vector(31 downto 0);

  -- GN4124 core DMA port to DDR wishbone bus
  signal wb_dma_adr   : std_logic_vector(31 downto 0);
  signal wb_dma_dat_i : std_logic_vector(31 downto 0);
  signal wb_dma_dat_o : std_logic_vector(31 downto 0);
  signal wb_dma_sel   : std_logic_vector(3 downto 0);
  signal wb_dma_cyc   : std_logic;
  signal wb_dma_stb   : std_logic;
  signal wb_dma_we    : std_logic;
  signal wb_dma_ack   : std_logic;
  signal wb_dma_stall : std_logic;
  signal wb_dma_err   : std_logic;
  signal wb_dma_rty   : std_logic;
  signal wb_dma_int   : std_logic;

  -- FMC ADC core to DDR wishbone bus
  signal wb_ddr_adr   : std_logic_vector(31 downto 0);
  signal wb_ddr_dat_o : std_logic_vector(63 downto 0);
  signal wb_ddr_sel   : std_logic_vector(7 downto 0);
  signal wb_ddr_cyc   : std_logic;
  signal wb_ddr_stb   : std_logic;
  signal wb_ddr_we    : std_logic;
  signal wb_ddr_ack   : std_logic;
  signal wb_ddr_stall : std_logic;

  -- Interrupts stuff
  signal dma_irq           : std_logic_vector(1 downto 0);
  signal dma_irq_p         : std_logic_vector(1 downto 0);
  signal trig_irq_p        : std_logic;
  signal acq_end_irq_p     : std_logic;
  signal irq_sources       : std_logic_vector(3 downto 0);
  signal irq_to_gn4124     : std_logic;
  signal irq_sources_2_led : std_logic_vector(3 downto 0);
  signal ddr_wr_fifo_empty : std_logic;
  signal dma_eic_irq       : std_logic;
  signal fmc0_eic_irq      : std_logic;

  -- LED control from carrier CSR register
  signal led_red   : std_logic;
  signal led_green : std_logic;

  -- CSR whisbone slaves for test
  signal gpio_stat     : std_logic_vector(31 downto 0);
  signal gpio_ctrl_1   : std_logic_vector(31 downto 0);
  signal gpio_ctrl_2   : std_logic_vector(31 downto 0);
  signal gpio_ctrl_3   : std_logic_vector(31 downto 0);
  signal gpio_led_ctrl : std_logic_vector(31 downto 0);

  -- GN4124
  signal gn4124_status  : std_logic_vector(31 downto 0);
  signal p2l_pll_locked : std_logic;

  -- DDR3
  signal ddr3_status     : std_logic_vector(31 downto 0);
  signal ddr3_calib_done : std_logic;

  -- SPI
  signal spi_din_t : std_logic_vector(3 downto 0);
  signal spi_ss_t  : std_logic_vector(7 downto 0);

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

  -- PHY
  signal phy8_to_wrc   : t_phy_8bits_to_wrc;
  signal phy8_from_wrc : t_phy_8bits_from_wrc;

  -- White Rabbit
  signal wrabbit_en  : std_logic;
  signal wrc_scl_out : std_logic;
  signal wrc_scl_in  : std_logic;
  signal wrc_sda_out : std_logic;
  signal wrc_sda_in  : std_logic;
  signal wrc_owr_en  : std_logic_vector(1 downto 0);
  signal wrc_owr_in  : std_logic_vector(1 downto 0);

  -- DACs
  signal dac_hpll_load_p1 : std_logic;
  signal dac_dpll_load_p1 : std_logic;
  signal dac_hpll_data    : std_logic_vector(15 downto 0);
  signal dac_dpll_data    : std_logic_vector(15 downto 0);

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
  -- Xilinx platform-specific (PHY, PLL, etc)
  --
  -- Unfortunately, the SPEC FPGA does not have enough PLL_ADV blocks to be
  -- able to use the default WRPC PLL scheme (due to the presence of the ADC
  -- sampling clock and the DDR). Therefore, we bypass the default scheme and
  -- define our own. Notably, we use a DCM_SP instead of a PLL_BASE to generate
  -- system clocks, and we piggy-back the DDR clock as well.
  --
  -- Once the PLLs are generated, we use the default WRPC Xilinx platform for
  -- the PHY.
  ------------------------------------------------------------------------------

  -- diff clock buffer from 125MHz clock reference
  cmp_pll_clk_dsbuf : IBUFGDS
    generic map (
      DIFF_TERM    => TRUE,
      IBUF_LOW_PWR => TRUE)
    port map (
      O  => clk_125m_pllref_buf,
      I  => clk_125m_pllref_p_i,
      IB => clk_125m_pllref_n_i);

  cmp_pll_clk_buf : BUFG
    port map (
      O => clk_125m_pllref,
      I => clk_125m_pllref_buf);

  cmp_clk_vcxo : BUFG
    port map (
      O => clk_20m_vcxo_buf,
      I => clk_20m_vcxo_i);

  pll_arst <= not button1_n_i;

  cmp_sys_clk_pll : DCM_SP
    generic map (
      CLKDV_DIVIDE          => 2.0,
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
      CLK0     => sys_clk_125_buf,
      CLK180   => open,
      CLK270   => open,
      CLK2X    => open,
      CLK2X180 => open,
      CLK90    => open,
      CLKDV    => sys_clk_62_5_buf,
      CLKFX    => ddr_clk_buf,
      CLKFX180 => open,
      LOCKED   => sys_clk_pll_locked,
      PSDONE   => open,
      STATUS   => open,
      CLKFB    => sys_clk_125,
      CLKIN    => clk_125m_pllref,
      DSSEN    => '0',
      PSCLK    => '0',
      PSEN     => '0',
      PSINCDEC => '0',
      RST      => pll_arst);

  cmp_dmtd_clk_pll : PLL_BASE
    generic map (
      BANDWIDTH          => "OPTIMIZED",
      CLK_FEEDBACK       => "CLKFBOUT",
      COMPENSATION       => "INTERNAL",
      DIVCLK_DIVIDE      => 1,
      CLKFBOUT_MULT      => 50,
      CLKFBOUT_PHASE     => 0.000,
      CLKOUT0_DIVIDE     => 16,
      CLKOUT0_PHASE      => 0.000,
      CLKOUT0_DUTY_CYCLE => 0.500,
      CLKOUT1_DIVIDE     => 16,
      CLKOUT1_PHASE      => 0.000,
      CLKOUT1_DUTY_CYCLE => 0.500,
      CLKOUT2_DIVIDE     => 16,
      CLKOUT2_PHASE      => 0.000,
      CLKOUT2_DUTY_CYCLE => 0.500,
      CLKIN_PERIOD       => 50.0,
      REF_JITTER         => 0.016)
    port map (
      CLKFBOUT => dmtd_clk_fb,
      CLKOUT0  => pllout_clk_dmtd,
      CLKOUT1  => open,
      CLKOUT2  => open,
      CLKOUT3  => open,
      CLKOUT4  => open,
      CLKOUT5  => open,
      LOCKED   => clk_dmtd_locked,
      RST      => pll_arst,
      CLKFBIN  => dmtd_clk_fb,
      CLKIN    => clk_20m_vcxo_buf);

  cmp_clk_62_5_buf : BUFG
    port map (
      O => sys_clk_62_5,
      I => sys_clk_62_5_buf);

  cmp_clk_125_buf : BUFG
    port map (
      O => sys_clk_125,
      I => sys_clk_125_buf);

  cmp_ddr_clk_buf : BUFG
    port map (
      O => ddr_clk,
      I => ddr_clk_buf);

  cmp_clk_dmtd_buf : BUFG
    port map (
      O => clk_dmtd,
      I => pllout_clk_dmtd);

  cmp_xwrc_platform_xilinx: xwrc_platform_xilinx
    generic map (
      g_fpga_family               => "spartan6",
      g_use_default_plls          => FALSE,
      g_simulation                => g_simulation)
    port map (
      areset_n_i            => button1_n_i,
      clk_125m_gtp_p_i      => clk_125m_gtp_p_i,
      clk_125m_gtp_n_i      => clk_125m_gtp_n_i,
      clk_62m5_dmtd_i       => clk_dmtd,
      clk_dmtd_locked_i     => clk_dmtd_locked,
      clk_62m5_sys_i        => sys_clk_62_5,
      clk_sys_locked_i      => sys_clk_pll_locked,
      clk_125m_ref_i        => sys_clk_125,
      clk_ref_locked_i      => sys_clk_pll_locked,
      sfp_txn_o             => sfp_txn_o,
      sfp_txp_o             => sfp_txp_o,
      sfp_rxn_i             => sfp_rxn_i,
      sfp_rxp_i             => sfp_rxp_i,
      sfp_tx_fault_i        => sfp_tx_fault_i,
      sfp_los_i             => sfp_los_i,
      sfp_tx_disable_o      => open,
      pll_locked_o          => pll_locked,
      phy8_o                => phy8_to_wrc,
      phy8_i                => phy8_from_wrc);

  sfp_tx_disable_o <= '0';
  
  ------------------------------------------------------------------------------
  -- System reset
  ------------------------------------------------------------------------------

  -- Detect when L_RST_N goes high (end of reset) and use this edge to generate
  -- powerup_arst_n. This is needed to connect optional reset like PCIe
  -- reset. When board runs standalone, we need to ignore PCIe reset being
  -- constantly low.
  cmp_arst_edge: gc_sync_ffs
    generic map (
      g_sync_edge => "positive")
    port map (
      clk_i    => sys_clk_62_5,
      rst_n_i  => '1',
      data_i   => L_RST_N,
      ppulse_o => arst_edge_ppulse);

  -- logic AND of all async reset sources (active low)
  powerup_arst_n <= pll_locked and button1_n_i and (not arst_edge_ppulse);

  -- concatenation of all clocks required to have synced resets
  powerup_clk_in(0) <= sys_clk_62_5;
  powerup_clk_in(1) <= sys_clk_125;
  powerup_clk_in(2) <= ddr_clk;

  cmp_powerup_reset : gc_reset
    generic map (
      g_clocks    => 3,                           -- 62.5MHz, 125MHz, 333MHz
      g_logdelay  => 4,                           -- 16 clock cycles
      g_syncdepth => 3)                           -- length of sync chains
    port map (
      free_clk_i => clk_125m_pllref,
      locked_i   => powerup_arst_n,
      clks_i     => powerup_clk_in,
      rstn_o     => powerup_rst_out);

  -- distribution of resets (already synchronized to their clock domains)
  sys_rst_62_5_n <= powerup_rst_out(0);
  sys_rst_125_n  <= powerup_rst_out(1);

  -- reset for mezzanine
  -- (including soft reset, no need to re-sync from 62.5MHz domain)
  fmc0_rst_n <= sys_rst_125_n and (not sw_rst_fmc0);

  -- reset for ddr
  -- (including soft reset, with re-sync from 62.5MHz domain)
  cmp_ddr_sw_reset_sync : gc_sync_ffs
    port map (
      clk_i    => ddr_clk,
      rst_n_i  => ddr_rst_n,
      data_i   => sw_rst_fmc0,
      synced_o => sw_rst_ddr_sync);

  ddr_rst_n  <= powerup_rst_out(2) and (not sw_rst_ddr_sync);

  ------------------------------------------------------------------------------
  -- GN4124 interface
  ------------------------------------------------------------------------------
  cmp_gn4124_core : gn4124_core
    port map(
      rst_n_a_i       => L_RST_N,
      status_o        => gn4124_status,
      -- P2L Direction Source Sync DDR related signals
      p2l_clk_p_i     => P2L_CLKp,
      p2l_clk_n_i     => P2L_CLKn,
      p2l_data_i      => P2L_DATA,
      p2l_dframe_i    => P2L_DFRAME,
      p2l_valid_i     => P2L_VALID,
      -- P2L Control
      p2l_rdy_o       => P2L_RDY,
      p_wr_req_i      => "00",
      p_wr_rdy_o      => P_WR_RDY,
      rx_error_o      => RX_ERROR,
      -- L2P Direction Source Sync DDR related signals
      l2p_clk_p_o     => L2P_CLKp,
      l2p_clk_n_o     => L2P_CLKn,
      l2p_data_o      => L2P_DATA,
      l2p_dframe_o    => L2P_DFRAME,
      l2p_valid_o     => L2P_VALID,
      l2p_edb_o       => L2P_EDB,
      -- L2P Control
      l2p_rdy_i       => L2P_RDY,
      l_wr_rdy_i      => L_WR_RDY,
      p_rd_d_rdy_i    => P_RD_D_RDY,
      tx_error_i      => TX_ERROR,
      vc_rdy_i        => "00",
      -- Interrupt interface
      dma_irq_o       => dma_irq,
      irq_p_i         => irq_to_gn4124,
      irq_p_o         => GPIO(0),
      -- DMA registers wishbone interface (slave classic)
      dma_reg_clk_i   => sys_clk_62_5,
      dma_reg_adr_i   => dma_ctrl_wb_adr,
      dma_reg_dat_i   => cnx_master_out(c_WB_SLAVE_DMA).dat,
      dma_reg_sel_i   => cnx_master_out(c_WB_SLAVE_DMA).sel,
      dma_reg_stb_i   => cnx_master_out(c_WB_SLAVE_DMA).stb,
      dma_reg_we_i    => cnx_master_out(c_WB_SLAVE_DMA).we,
      dma_reg_cyc_i   => cnx_master_out(c_WB_SLAVE_DMA).cyc,
      dma_reg_dat_o   => cnx_master_in(c_WB_SLAVE_DMA).dat,
      dma_reg_ack_o   => cnx_master_in(c_WB_SLAVE_DMA).ack,
      dma_reg_stall_o => cnx_master_in(c_WB_SLAVE_DMA).stall,
      -- CSR wishbone interface (master pipelined)
      csr_clk_i       => sys_clk_62_5,
      csr_adr_o       => gn_wb_adr,
      csr_dat_o       => cnx_slave_in(c_MASTER_GENNUM).dat,
      csr_sel_o       => cnx_slave_in(c_MASTER_GENNUM).sel,
      csr_stb_o       => cnx_slave_in(c_MASTER_GENNUM).stb,
      csr_we_o        => cnx_slave_in(c_MASTER_GENNUM).we,
      csr_cyc_o       => cnx_slave_in(c_MASTER_GENNUM).cyc,
      csr_dat_i       => cnx_slave_out(c_MASTER_GENNUM).dat,
      csr_ack_i       => cnx_slave_out(c_MASTER_GENNUM).ack,
      csr_stall_i     => cnx_slave_out(c_MASTER_GENNUM).stall,
      csr_err_i       => cnx_slave_out(c_MASTER_GENNUM).err,
      csr_rty_i       => cnx_slave_out(c_MASTER_GENNUM).rty,
      csr_int_i       => cnx_slave_out(c_MASTER_GENNUM).int,
      -- DMA wishbone interface (pipelined)
      dma_clk_i       => sys_clk_62_5,
      dma_adr_o       => wb_dma_adr,
      dma_dat_o       => wb_dma_dat_o,
      dma_sel_o       => wb_dma_sel,
      dma_stb_o       => wb_dma_stb,
      dma_we_o        => wb_dma_we,
      dma_cyc_o       => wb_dma_cyc,
      dma_dat_i       => wb_dma_dat_i,
      dma_ack_i       => wb_dma_ack,
      dma_stall_i     => wb_dma_stall,
      dma_err_i       => wb_dma_err,
      dma_rty_i       => wb_dma_rty,
      dma_int_i       => wb_dma_int
      );

  p2l_pll_locked <= gn4124_status(0);

  -- Convert 32-bit word address into byte address for crossbar
  cnx_slave_in(c_MASTER_GENNUM).adr <= gn_wb_adr(29 downto 0) & "00";

  -- Convert 32-bit byte address into word address for DMA controller
  dma_ctrl_wb_adr <= "00" & cnx_master_out(c_WB_SLAVE_DMA).adr(31 downto 2);

  -- Unused wishbone signals
  cnx_master_in(c_WB_SLAVE_DMA).err <= '0';
  cnx_master_in(c_WB_SLAVE_DMA).rty <= '0';
  cnx_master_in(c_WB_SLAVE_DMA).int <= '0';

  ------------------------------------------------------------------------------
  -- CSR wishbone crossbar
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
      clk_sys_i => sys_clk_62_5,
      rst_n_i   => sys_rst_62_5_n,
      slave_i   => cnx_slave_in,
      slave_o   => cnx_slave_out,
      master_i  => cnx_master_in,
      master_o  => cnx_master_out);

  -----------------------------------------------------------------------------
  -- 2x SPI DAC
  -----------------------------------------------------------------------------

  U_DAC_ARB : spec_serial_dac_arb
    generic map (
      g_invert_sclk    => FALSE,
      g_num_extra_bits => 8)
    port map (
      clk_i         => sys_clk_62_5,
      rst_n_i       => sys_rst_62_5_n,
      val1_i        => dac_dpll_data,
      load1_i       => dac_dpll_load_p1,
      val2_i        => dac_hpll_data,
      load2_i       => dac_hpll_load_p1,
      dac_cs_n_o(0) => pll25dac_sync_n_o,
      dac_cs_n_o(1) => pll20dac_sync_n_o,
      dac_sclk_o    => plldac_sclk_o,
      dac_din_o     => plldac_din_o);

  -------------------------------------------------------------------------------
  -- White Rabbit Core + PHY
  -------------------------------------------------------------------------------

  -- Tristates for FMC EEPROM
  --fmc0_sys_scl_b <= '0' when (wrc_scl_out = '0') else 'Z';
  --fmc0_sys_sda_b <= '0' when (wrc_sda_out = '0') else 'Z';
  --wrc_scl_in     <= fmc0_sys_scl_b;
  --wrc_sda_in     <= fmc0_sys_sda_b;
  wrc_scl_in <= '0';
  wrc_sda_in <= '0';

  -- Tristates for SFP EEPROM
  sfp_mod_def1_b <= '0' when sfp_scl_out = '0' else 'Z';
  sfp_mod_def2_b <= '0' when sfp_sda_out = '0' else 'Z';
  sfp_scl_in     <= sfp_mod_def1_b;
  sfp_sda_in     <= sfp_mod_def2_b;

  carrier_onewire_b <= '0' when wrc_owr_en(0) = '1' else 'Z';
  wrc_owr_in(0)     <= carrier_onewire_b;
  wrc_owr_in(1)     <= '1';

  cmp_xwrc_board_common : xwrc_board_common
    generic map (
      g_simulation  => g_simulation,
      g_board_name  => "SPEC",
      g_dpram_initf => g_wrpc_initf)
    port map (
      clk_sys_i            => sys_clk_62_5,
      clk_dmtd_i           => clk_dmtd,
      clk_ref_i            => clk_125m_pllref,
      rst_n_i              => sys_rst_62_5_n,
      dac_hpll_load_p1_o   => dac_hpll_load_p1,
      dac_hpll_data_o      => dac_hpll_data,
      dac_dpll_load_p1_o   => dac_dpll_load_p1,
      dac_dpll_data_o      => dac_dpll_data,
      phy8_o               => phy8_from_wrc,
      phy8_i               => phy8_to_wrc,
      scl_o                => wrc_scl_out,
      scl_i                => wrc_scl_in,
      sda_o                => wrc_sda_out,
      sda_i                => wrc_sda_in,
      sfp_scl_o            => sfp_scl_out,
      sfp_scl_i            => sfp_scl_in,
      sfp_sda_o            => sfp_sda_out,
      sfp_sda_i            => sfp_sda_in,
      sfp_det_i            => sfp_mod_def0_i,
      spi_sclk_o           => spi_sclk_o,
      spi_ncs_o            => spi_ncs_o,
      spi_mosi_o           => spi_mosi_o,
      spi_miso_i           => spi_miso_i,
      uart_rxd_i           => uart_rxd_i,
      uart_txd_o           => uart_txd_o,
      owr_en_o             => wrc_owr_en,
      owr_i                => wrc_owr_in,
      wb_slave_i           => cnx_master_out(c_WB_SLAVE_WR_CORE),
      wb_slave_o           => cnx_master_in(c_WB_SLAVE_WR_CORE),
      tm_link_up_o         => tm_link_up,
      tm_time_valid_o      => tm_time_valid,
      tm_tai_o             => tm_tai,
      tm_cycles_o          => tm_cycles,
      led_act_o            => led_red,
      led_link_o           => wrabbit_en);

  ------------------------------------------------------------------------------
  -- Carrier CSR
  --    Carrier type and PCB version
  --    Bitstream (firmware) type and date
  --    Release tag
  --    VCXO DAC control (CLR_N)
  ------------------------------------------------------------------------------
  cmp_carrier_csr : carrier_csr
    port map(
      rst_n_i    => sys_rst_62_5_n,
      clk_sys_i  => sys_clk_62_5,
      wb_adr_i   => cnx_master_out(c_WB_SLAVE_SPEC_CSR).adr(3 downto 2),  -- cnx_master_out.adr is byte address
      wb_dat_i   => cnx_master_out(c_WB_SLAVE_SPEC_CSR).dat,
      wb_dat_o   => cnx_master_in(c_WB_SLAVE_SPEC_CSR).dat,
      wb_cyc_i   => cnx_master_out(c_WB_SLAVE_SPEC_CSR).cyc,
      wb_sel_i   => cnx_master_out(c_WB_SLAVE_SPEC_CSR).sel,
      wb_stb_i   => cnx_master_out(c_WB_SLAVE_SPEC_CSR).stb,
      wb_we_i    => cnx_master_out(c_WB_SLAVE_SPEC_CSR).we,
      wb_ack_o   => cnx_master_in(c_WB_SLAVE_SPEC_CSR).ack,
      wb_stall_o => open,
      regs_i     => csr_regin,
      regs_o     => csr_regout);

  csr_regin.carrier_pcb_rev_i    <= pcb_ver_i;
  csr_regin.carrier_reserved_i   <= X"000";
  csr_regin.carrier_type_i       <= c_CARRIER_TYPE;
  csr_regin.stat_fmc_pres_i      <= fmc0_prsnt_m2c_n_i;
  csr_regin.stat_p2l_pll_lck_i   <= p2l_pll_locked;
  csr_regin.stat_sys_pll_lck_i   <= sys_clk_pll_locked;
  csr_regin.stat_ddr3_cal_done_i <= ddr3_calib_done;

  sw_rst_fmc0 <= csr_regout.rst_fmc0_o;

  -- Unused wishbone signals
  cnx_master_in(c_WB_SLAVE_SPEC_CSR).err   <= '0';
  cnx_master_in(c_WB_SLAVE_SPEC_CSR).rty   <= '0';
  cnx_master_in(c_WB_SLAVE_SPEC_CSR).stall <= '0';
  cnx_master_in(c_WB_SLAVE_SPEC_CSR).int   <= '0';

  -- SPEC front panel leds
  led_red_o   <= led_red or csr_regout.ctrl_led_red_o;
  led_green_o <= led_green or csr_regout.ctrl_led_green_o;

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
      clk_sys_i    => sys_clk_62_5,
      rst_n_i      => sys_rst_62_5_n,
      slave_i      => cnx_master_out(c_WB_SLAVE_VIC),
      slave_o      => cnx_master_in(c_WB_SLAVE_VIC),
      irqs_i(0)    => fmc0_eic_irq,
      irqs_i(1)    => dma_eic_irq,
      irq_master_o => irq_to_gn4124);

  ------------------------------------------------------------------------------
  -- GN4124 DMA interrupt controller
  ------------------------------------------------------------------------------
  cmp_dma_eic : dma_eic
    port map(
      rst_n_i         => sys_rst_62_5_n,
      clk_sys_i       => sys_clk_62_5,
      wb_adr_i        => cnx_master_out(c_WB_SLAVE_DMA_EIC).adr(3 downto 2),  -- cnx_master_out.adr is byte address
      wb_dat_i        => cnx_master_out(c_WB_SLAVE_DMA_EIC).dat,
      wb_dat_o        => cnx_master_in(c_WB_SLAVE_DMA_EIC).dat,
      wb_cyc_i        => cnx_master_out(c_WB_SLAVE_DMA_EIC).cyc,
      wb_sel_i        => cnx_master_out(c_WB_SLAVE_DMA_EIC).sel,
      wb_stb_i        => cnx_master_out(c_WB_SLAVE_DMA_EIC).stb,
      wb_we_i         => cnx_master_out(c_WB_SLAVE_DMA_EIC).we,
      wb_ack_o        => cnx_master_in(c_WB_SLAVE_DMA_EIC).ack,
      wb_stall_o      => cnx_master_in(c_WB_SLAVE_DMA_EIC).stall,
      wb_int_o        => dma_eic_irq,
      irq_dma_done_i  => dma_irq(0),
      irq_dma_error_i => dma_irq(1)
      );

  -- Unused wishbone signals
  cnx_master_in(c_WB_SLAVE_DMA_EIC).err <= '0';
  cnx_master_in(c_WB_SLAVE_DMA_EIC).rty <= '0';
  cnx_master_in(c_WB_SLAVE_DMA_EIC).int <= '0';

  ------------------------------------------------------------------------------
  -- FMC ADC mezzanine (wb bridge with cross-clocking)
  --    System managment I2C master
  --    SPI master
  --    I2C
  --    ADC core csr
  --    1-wire master
  --    eic
  --    timetag core
  ------------------------------------------------------------------------------

  cmp_xwb_clock_crossing_0 : xwb_clock_crossing
    generic map(
      g_size => 16
      )
    port map(
      slave_clk_i    => sys_clk_62_5,
      slave_rst_n_i  => sys_rst_62_5_n,
      slave_i        => cnx_master_out(c_WB_SLAVE_FMC_ADC),
      slave_o        => cnx_master_in(c_WB_SLAVE_FMC_ADC),
      master_clk_i   => sys_clk_125,
      master_rst_n_i => sys_rst_125_n,
      master_i       => cnx_fmc0_sync_master_in,
      master_o       => cnx_fmc0_sync_master_out
      );

  cmp_fmc_adc_mezzanine_0 : fmc_adc_mezzanine
    generic map(
      g_multishot_ram_size => g_multishot_ram_size,
      g_carrier_type       => "SPEC"
      )
    port map(
      sys_clk_i   => sys_clk_125,
      sys_rst_n_i => fmc0_rst_n,

      wb_csr_adr_i   => cnx_fmc0_sync_master_out.adr,
      wb_csr_dat_i   => cnx_fmc0_sync_master_out.dat,
      wb_csr_dat_o   => cnx_fmc0_sync_master_in.dat,
      wb_csr_cyc_i   => cnx_fmc0_sync_master_out.cyc,
      wb_csr_sel_i   => cnx_fmc0_sync_master_out.sel,
      wb_csr_stb_i   => cnx_fmc0_sync_master_out.stb,
      wb_csr_we_i    => cnx_fmc0_sync_master_out.we,
      wb_csr_ack_o   => cnx_fmc0_sync_master_in.ack,
      wb_csr_stall_o => cnx_fmc0_sync_master_in.stall,

      wb_ddr_clk_i   => sys_clk_125,
      wb_ddr_adr_o   => wb_ddr_adr,
      wb_ddr_dat_o   => wb_ddr_dat_o,
      wb_ddr_sel_o   => wb_ddr_sel,
      wb_ddr_stb_o   => wb_ddr_stb,
      wb_ddr_we_o    => wb_ddr_we,
      wb_ddr_cyc_o   => wb_ddr_cyc,
      wb_ddr_ack_i   => wb_ddr_ack,
      wb_ddr_stall_i => wb_ddr_stall,

      ddr_wr_fifo_empty_i => ddr_wr_fifo_empty,
      trig_irq_o          => trig_irq_p,
      acq_end_irq_o       => acq_end_irq_p,
      eic_irq_o           => fmc0_eic_irq,

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

      sys_scl_b => fmc0_sys_scl_b,
      sys_sda_b => fmc0_sys_sda_b,

      wr_tm_link_up_i    => tm_link_up,
      wr_tm_time_valid_i => tm_time_valid,
      wr_tm_tai_i        => tm_tai,
      wr_tm_cycles_i     => tm_cycles,
      wr_enable_i        => wrabbit_en
      );

  -- Unused wishbone signals
  cnx_fmc0_sync_master_in.err <= '0';
  cnx_fmc0_sync_master_in.rty <= '0';
  cnx_fmc0_sync_master_in.int <= '0';

  ------------------------------------------------------------------------------
  -- DMA wishbone bus slaves
  --  -> DDR3 controller
  ------------------------------------------------------------------------------
  cmp_ddr_ctrl : ddr3_ctrl
    generic map(
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
      clk_i   => ddr_clk,
      rst_n_i => ddr_rst_n,

      status_o => ddr3_status,

      ddr3_dq_b     => DDR3_DQ,
      ddr3_a_o      => DDR3_A,
      ddr3_ba_o     => DDR3_BA,
      ddr3_ras_n_o  => DDR3_RAS_N,
      ddr3_cas_n_o  => DDR3_CAS_N,
      ddr3_we_n_o   => DDR3_WE_N,
      ddr3_odt_o    => DDR3_ODT,
      ddr3_rst_n_o  => DDR3_RESET_N,
      ddr3_cke_o    => DDR3_CKE,
      ddr3_dm_o     => DDR3_LDM,
      ddr3_udm_o    => DDR3_UDM,
      ddr3_dqs_p_b  => DDR3_LDQS_P,
      ddr3_dqs_n_b  => DDR3_LDQS_N,
      ddr3_udqs_p_b => DDR3_UDQS_P,
      ddr3_udqs_n_b => DDR3_UDQS_N,
      ddr3_clk_p_o  => DDR3_CK_P,
      ddr3_clk_n_o  => DDR3_CK_N,
      ddr3_rzq_b    => DDR3_RZQ,
      ddr3_zio_b    => DDR3_ZIO,

      wb0_rst_n_i => sys_rst_125_n,
      wb0_clk_i   => sys_clk_125,
      wb0_sel_i   => wb_ddr_sel,
      wb0_cyc_i   => wb_ddr_cyc,
      wb0_stb_i   => wb_ddr_stb,
      wb0_we_i    => wb_ddr_we,
      wb0_addr_i  => wb_ddr_adr,
      wb0_data_i  => wb_ddr_dat_o,
      wb0_data_o  => open,
      wb0_ack_o   => wb_ddr_ack,
      wb0_stall_o => wb_ddr_stall,

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

      wb1_rst_n_i => sys_rst_62_5_n,
      wb1_clk_i   => sys_clk_62_5,
      wb1_sel_i   => wb_dma_sel,
      wb1_cyc_i   => wb_dma_cyc,
      wb1_stb_i   => wb_dma_stb,
      wb1_we_i    => wb_dma_we,
      wb1_addr_i  => wb_dma_adr,
      wb1_data_i  => wb_dma_dat_o,
      wb1_data_o  => wb_dma_dat_i,
      wb1_ack_o   => wb_dma_ack,
      wb1_stall_o => wb_dma_stall,

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

  ddr3_calib_done <= ddr3_status(0);

  -- unused Wishbone signals
  wb_dma_err <= '0';
  wb_dma_rty <= '0';
  wb_dma_int <= '0';

  ------------------------------------------------------------------------------
  -- Assign unused outputs
  ------------------------------------------------------------------------------
  GPIO(1) <= '0';                                 -- connection to GN4124

  ------------------------------------------------------------------------------
  -- FPGA loaded led (heart beat)
  ------------------------------------------------------------------------------
  p_led_pwn_update_cnt : process (sys_clk_62_5)
  begin
    if rising_edge(sys_clk_62_5) then
      if (sys_rst_62_5_n = '0') then
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

  p_led_pwn_val : process (sys_clk_62_5)
  begin
    if rising_edge(sys_clk_62_5) then
      if (sys_rst_62_5_n = '0') then
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

  p_led_pwn_cnt : process (sys_clk_62_5)
  begin
    if rising_edge(sys_clk_62_5) then
      if (sys_rst_62_5_n = '0') then
        led_pwm_cnt <= (others => '0');
      else
        led_pwm_cnt <= led_pwm_cnt + 1;
      end if;
    end if;
  end process p_led_pwn_cnt;

  p_led_pwn : process (sys_clk_62_5)
  begin
    if rising_edge(sys_clk_62_5) then
      if (sys_rst_62_5_n = '0') then
        led_pwm <= '0';
      elsif (led_pwm_cnt = 0) then
        led_pwm <= '1';
      elsif (led_pwm_cnt = led_pwm_val) then
        led_pwm <= '0';
      end if;
    end if;
  end process p_led_pwn;

  aux_leds_o(0) <= led_pwm;

  -- IRQ LEDs
  --   0    -> End of DMA transfer
  --   1    -> DMA transfer error
  --   2    -> Trigger
  --   3    -> End of acquisition (data written to DDR)
  irq_sources(1 downto 0) <= dma_irq;
  irq_sources(2)          <= trig_irq_p;
  irq_sources(3)          <= acq_end_irq_p;

  gen_irq_led : for I in 0 to irq_sources'length-1 generate
    cmp_irq_led : gc_extend_pulse
      generic map (
        g_width => 2500000)
      port map (
        clk_i      => sys_clk_62_5,
        rst_n_i    => sys_rst_62_5_n,
        pulse_i    => irq_sources(I),
        extended_o => irq_sources_2_led(I));
  end generate gen_irq_led;

  aux_leds_o(1) <= not(irq_sources_2_led(2));
  aux_leds_o(2) <= not(irq_sources_2_led(3));
  aux_leds_o(3) <= not(irq_sources_2_led(0));

end rtl;
