-------------------------------------------------------------------------------
-- SPDX-License-Identifier: CERN-OHL-W-2.0
-- Title      : FMC ADC 100Ms/s core package
-- Project    : FMC ADC 100M 14B 4CHA gateware
-- URL        : http://www.ohwr.org/projects/fmc-adc-100m14b4cha-gw
-------------------------------------------------------------------------------
-- File       : fmc_adc_100Ms_core_pkg.vhd
-- Author(s)  : Matthieu Cattin <matthieu.cattin@cern.ch>
--              Theodor Stana <t.stana@cern.ch>
--              Dimitrios Lampridis  <dimitrios.lampridis@cern.ch>
-- Company    : CERN (BE-CO-HT)
-- Created    : 2012-11-16
-- Standard   : VHDL'93/02
-------------------------------------------------------------------------------
-- Description: Package for FMC ADC 100Ms/s core.
-------------------------------------------------------------------------------
-- Copyright (c) 2011-2018 CERN (BE-CO-HT)
-------------------------------------------------------------------------------
-- Revisions  :
-- Date        Version  Author
-- 2012-11-16  1.0      Matthieu Cattin
-------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;
use work.timetag_core_defs_pkg.all;
use work.wishbone_pkg.all;

package fmc_adc_100Ms_core_pkg is

  ------------------------------------------------------------------------------
  -- Constants declaration
  ------------------------------------------------------------------------------


  ------------------------------------------------------------------------------
  -- Components declaration
  ------------------------------------------------------------------------------
  component fmc_adc_100Ms_core
    generic (
      g_MULTISHOT_RAM_SIZE : natural                        := 2048;
      -- Only used on Xilinx Spartan6 FPGAs
      g_SPARTAN6_USE_PLL   : boolean                        := TRUE;
      -- External trigger delay calibration value
      g_TRIG_DELAY_EXT     : natural                        := 7;
      -- Software and time trigger delay calibration value
      g_TRIG_DELAY_SW      : natural                        := 9;
      -- FMC-ADC identification number
      g_FMC_ADC_NR         : natural                        := 0;
      -- WB interface configuration
      g_WB_CSR_MODE        : t_wishbone_interface_mode      := PIPELINED;
      g_WB_CSR_GRANULARITY : t_wishbone_address_granularity := BYTE);
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

      -- Acquisition configuration status flag
      acq_cfg_ok_o : out std_logic;

      -- Trigout wishbone interface
      wb_trigout_slave_i : in  t_wishbone_slave_in;
      wb_trigout_slave_o : out t_wishbone_slave_out;

      -- Events output pulses
      trigger_p_o   : out std_logic;
      acq_start_p_o : out std_logic;
      acq_stop_p_o  : out std_logic;
      acq_end_p_o   : out std_logic;

      -- Trigger time-tag input
      trigger_tag_i   : in t_timetag;
      time_trig_i     : in std_logic;
      aux_time_trig_i : in std_logic;

      -- WR status (for trigout).
      wr_tm_link_up_i    : in std_logic;
      wr_tm_time_valid_i : in std_logic;
      wr_enable_i        : in std_logic;

      -- FMC interface
      ext_trigger_p_i : in std_logic;             -- External trigger
      ext_trigger_n_i : in std_logic;

      adc_dco_p_i  : in std_logic;                     -- ADC data clock
      adc_dco_n_i  : in std_logic;
      adc_fr_p_i   : in std_logic;                     -- ADC frame start
      adc_fr_n_i   : in std_logic;
      adc_outa_p_i : in std_logic_vector(3 downto 0);  -- ADC serial data (odd bits)
      adc_outa_n_i : in std_logic_vector(3 downto 0);
      adc_outb_p_i : in std_logic_vector(3 downto 0);  -- ADC serial data (even bits)
      adc_outb_n_i : in std_logic_vector(3 downto 0);

      gpio_dac_clr_n_o : out std_logic;           -- offset DACs clear (active low)
      gpio_led_acq_o   : out std_logic;           -- Mezzanine front panel power LED (PWR)
      gpio_led_trig_o  : out std_logic;           -- Mezzanine front panel trigger LED (TRIG)
      gpio_ssr_ch1_o   : out std_logic_vector(6 downto 0);  -- Channel 1 solid state relays control
      gpio_ssr_ch2_o   : out std_logic_vector(6 downto 0);  -- Channel 2 solid state relays control
      gpio_ssr_ch3_o   : out std_logic_vector(6 downto 0);  -- Channel 3 solid state relays control
      gpio_ssr_ch4_o   : out std_logic_vector(6 downto 0);  -- Channel 4 solid state relays control
      gpio_si570_oe_o  : out std_logic);          -- Si570 (programmable oscillator) output enable

  end component fmc_adc_100Ms_core;

end fmc_adc_100Ms_core_pkg;
