-------------------------------------------------------------------------------
-- Title      : FMC ADC mezzanine package
-- Project    : FMC ADC 100M 14B 4CHA gateware
-- URL        : http://www.ohwr.org/projects/fmc-adc-100m14b4cha-gw
-------------------------------------------------------------------------------
-- File       : fmc_adc_mezzanine_pkg.vhd
-- Author(s)  : Matthieu Cattin <matthieu.cattin@cern.ch>
--              Dimitrios Lampridis  <dimitrios.lampridis@cern.ch>
-- Company    : CERN (BE-CO-HT)
-- Created    : 2013-07-03
-- Standard   : VHDL'93/02
-------------------------------------------------------------------------------
-- Description: Package for FMC ADC mezzanine
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
-- Revisions  :
-- Date        Version  Author
-- 2013-07-03  1.0      Matthieu Cattin
-------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;
use work.timetag_core_defs_pkg.all;
use work.wishbone_pkg.all;

package fmc_adc_mezzanine_pkg is

  ------------------------------------------------------------------------------
  -- Components declaration
  ------------------------------------------------------------------------------
  component fmc_adc_mezzanine
    generic (
      g_MULTISHOT_RAM_SIZE : natural                        := 2048;
      -- Only used on Xilinx Spartan6 FPGAs
      g_SPARTAN6_USE_PLL   : boolean                        := TRUE;
      -- External trigger delay calibration value
      g_TRIG_DELAY_EXT     : natural                        := 7;
      -- Software and time trigger delay calibration value
      g_TRIG_DELAY_SW      : natural                        := 9;
      -- Value to be subtracted from trigger tag coarse counter.
      -- This is useful if you know that the system introduces
      -- some systematic delay wrt the actual trigger time
      g_TAG_ADJUST         : natural                        := 24;
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

      -- Interrupt and status
      ddr_wr_fifo_empty_i : in  std_logic;
      trig_irq_o          : out std_logic;
      acq_end_irq_o       : out std_logic;
      eic_irq_o           : out std_logic;
      acq_cfg_ok_o        : out std_logic;

      -- Auxiliary trigger input wishbone interface
      wb_trigin_slave_i : in  t_wishbone_slave_in := c_DUMMY_WB_SLAVE_IN;
      wb_trigin_slave_o : out t_wishbone_slave_out;

      -- Trigout wishbone interface
      wb_trigout_slave_i : in  t_wishbone_slave_in := c_DUMMY_WB_SLAVE_IN;
      wb_trigout_slave_o : out t_wishbone_slave_out;

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
      gpio_si570_oe_o  : out std_logic;           -- Si570 (programmable oscillator) output enable

      spi_din_i       : in  std_logic;            -- SPI data from FMC
      spi_dout_o      : out std_logic;            -- SPI data to FMC
      spi_sck_o       : out std_logic;            -- SPI clock
      spi_cs_adc_n_o  : out std_logic;            -- SPI ADC chip select (active low)
      spi_cs_dac1_n_o : out std_logic;  -- SPI channel 1 offset DAC chip select (active low)
      spi_cs_dac2_n_o : out std_logic;  -- SPI channel 2 offset DAC chip select (active low)
      spi_cs_dac3_n_o : out std_logic;  -- SPI channel 3 offset DAC chip select (active low)
      spi_cs_dac4_n_o : out std_logic;  -- SPI channel 4 offset DAC chip select (active low)

      si570_scl_b : inout std_logic;              -- I2C bus clock (Si570)
      si570_sda_b : inout std_logic;              -- I2C bus data (Si570)

      mezz_one_wire_b : inout std_logic;  -- Mezzanine 1-wire interface (DS18B20 thermometer + unique ID)

      sys_scl_b : inout std_logic;                -- Mezzanine system I2C clock (EEPROM)
      sys_sda_b : inout std_logic;                -- Mezzanine system I2C data (EEPROM)

      wr_tm_link_up_i    : in std_logic;          -- WR link status bit
      wr_tm_time_valid_i : in std_logic;          -- WR timecode valid status bit
      wr_tm_tai_i        : in std_logic_vector(39 downto 0);  -- WR timecode seconds
      wr_tm_cycles_i     : in std_logic_vector(27 downto 0);  -- WR timecode 8ns ticks
      wr_enable_i        : in std_logic);         -- enable white rabbit features on mezzanine

  end component fmc_adc_mezzanine;

end fmc_adc_mezzanine_pkg;
