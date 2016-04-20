-------------------------------------------------------------------------------
-- Title      : FMC ADC 100Ms/s SVEC SDB meta-information
-- Project    : FMC ADC 100M 14B 4CHA gateware
-- URL        : http://www.ohwr.org/projects/fmc-adc-100m14b4cha-gw
-------------------------------------------------------------------------------
-- File       : sdb_meta_pkg.vhd
-- Author(s)  : Matthieu Cattin <matthieu.cattin@cern.ch>
--            : Dimitrios Lampridis  <dimitrios.lampridis@cern.ch>
-- Company    : CERN (BE-CO-HT)
-- Created    : 2013-07-05
-- Last update: 2016-04-19
-- Standard   : VHDL'93/02
-------------------------------------------------------------------------------
-- Description: SDB meta-information for the FMC ADC 100Ms/s design for SVEC.
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
-- 2016-04-20  4.1      Dimitrios Lampridis
-- 2014-04-25  4.0      Matthieu Cattin
-- 2014-01-16  3.0      Matthieu Cattin
-- 2013-07-29  1.0      Matthieu Cattin
-------------------------------------------------------------------------------

library ieee;

use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.wishbone_pkg.all;

package sdb_meta_pkg is

  ------------------------------------------------------------------------------
  -- Meta-information sdb records
  ------------------------------------------------------------------------------

  -- Top module repository url
  constant c_repo_url_sdb : t_sdb_repo_url := (
    -- url (string, limited to 63 char, full URL does not fit unfortunately)
    repo_url => "fmc-projects/fmc-adc-100m14b4cha/fmc-adc-100m14b4cha-gw.git    ");

  -- Synthesis informations
  constant c_synthesis_sdb : t_sdb_synthesis := (
    -- Top module name (string, 16 char)
    syn_module_name  => "svec_top_fmc_adc",
    -- Commit ID (hex string, 128-bit = 32 char)
    -- git log -1 --format="%H" | cut -c1-32
    syn_commit_id    => "26749f0a1873c215abb33942a8a335db",
    -- Synthesis tool name (string, 8 char)
    syn_tool_name    => "ISE     ",
    -- Synthesis tool version (bcd encoded, 32-bit)
    syn_tool_version => x"00000147",
    -- Synthesis date (bcd encoded, 32-bit, yyyymmdd)
    syn_date         => x"20160420",
    -- Synthesised by (string, 15 char)
    syn_username     => "dlamprid       ");

  -- Integration record
  constant c_integration_sdb : t_sdb_integration := (
    product     => (
      vendor_id => x"000000000000CE42",  -- CERN
      device_id => x"5c01a632",          -- echo "svec_fmc-adc-100m14b4cha" | md5sum | cut -c1-8
      version   => x"00040001",          -- bcd encoded, [31:16] = major, [15:0] = minor
      date      => x"20160420",          -- yyyymmdd
      name      => "svec_fmcadc100m14b "));


end sdb_meta_pkg;


package body sdb_meta_pkg is
end sdb_meta_pkg;
