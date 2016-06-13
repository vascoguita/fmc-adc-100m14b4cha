-------------------------------------------------------------------------------
-- Title      : Timetag core package
-- Project    : FMC ADC 100M 14B 4CHA gateware
-- URL        : http://www.ohwr.org/projects/fmc-adc-100m14b4cha-gw
-------------------------------------------------------------------------------
-- File       : timetag_core_pkg.vhd
-- Author(s)  : Matthieu Cattin <matthieu.cattin@cern.ch>
--            : Dimitrios Lampridis  <dimitrios.lampridis@cern.ch>
-- Company    : CERN (BE-CO-HT)
-- Created    : 2013-07-05
-- Last update: 2016-06-09
-- Standard   : VHDL'93/02
-------------------------------------------------------------------------------
-- Description: Package for timetag core
-------------------------------------------------------------------------------
-- Copyright (c) 2013-2016 CERN (BE-CO-HT)
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
-- 2013-07-05  1.0      Matthieu Cattin
-------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;

package timetag_core_pkg is

  ------------------------------------------------------------------------------
  -- Constants declaration
  ------------------------------------------------------------------------------

  ------------------------------------------------------------------------------
  -- Types declaration
  ------------------------------------------------------------------------------
  type t_timetag is record
    seconds : std_logic_vector(39 downto 0);
    coarse  : std_logic_vector(27 downto 0);
  end record t_timetag;

  ------------------------------------------------------------------------------
  -- Components declaration
  ------------------------------------------------------------------------------
  component timetag_core is
    port (
      clk_i              : in  std_logic;
      rst_n_i            : in  std_logic;
      trigger_p_i        : in  std_logic;
      acq_start_p_i      : in  std_logic;
      acq_stop_p_i       : in  std_logic;
      acq_end_p_i        : in  std_logic;
      wr_enabled_i       : in  std_logic;
      wr_tm_time_valid_i : in  std_logic;
      wr_tm_tai_i        : in  std_logic_vector(39 downto 0);
      wr_tm_cycles_i     : in  std_logic_vector(27 downto 0);
      trig_tag_o         : out t_timetag;
      wb_adr_i           : in  std_logic_vector(3 downto 0);
      wb_dat_i           : in  std_logic_vector(31 downto 0);
      wb_dat_o           : out std_logic_vector(31 downto 0);
      wb_cyc_i           : in  std_logic;
      wb_sel_i           : in  std_logic_vector(3 downto 0);
      wb_stb_i           : in  std_logic;
      wb_we_i            : in  std_logic;
      wb_ack_o           : out std_logic); 
  end component timetag_core;

end timetag_core_pkg;

package body timetag_core_pkg is

end timetag_core_pkg;
