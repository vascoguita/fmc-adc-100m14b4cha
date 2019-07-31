-------------------------------------------------------------------------------
-- Title      : Time-tagging core
-- Project    : FMC ADC 100M 14B 4CHA gateware
-- URL        : http://www.ohwr.org/projects/fmc-adc-100m14b4cha-gw
-------------------------------------------------------------------------------
-- File       : timetag_core.vhd
-- Company    : CERN (BE-CO-HT)
-- Created    : 2011-11-18
-- Standard   : VHDL'93/02
-------------------------------------------------------------------------------
-- Description: Implements a UTC seconds counter and a 125MHz system clock
-- ticks counter to time-tag trigger, acquisition start and stop events.
-------------------------------------------------------------------------------
-- Copyright (c) 2011-2019 CERN (BE-CO-HT)
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

package timetag_core_defs_pkg is

  type t_timetag is record
    seconds : std_logic_vector(39 downto 0);
    coarse  : std_logic_vector(27 downto 0);
  end record t_timetag;

  constant c_TAG_COARSE_MAX : unsigned := to_unsigned(125000000, 28);

end timetag_core_defs_pkg;

library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;

use work.wishbone_pkg.all;
use work.timetag_core_regs_pkg.all;
use work.timetag_core_defs_pkg.all;

entity timetag_core is
  generic (
    -- WB interface configuration
    g_WB_MODE        : t_wishbone_interface_mode      := PIPELINED;
    g_WB_GRANULARITY : t_wishbone_address_granularity := BYTE;
    -- Value to be subtracted from trigger tag coarse counter.
    -- This is useful if you know that the system introduces
    -- some systematic delay wrt the actual trigger time
    g_TAG_ADJUST     : natural := 0);
  port (
    -- Clock, reset
    clk_i   : in std_logic;  -- Must be 125MHz
    rst_n_i : in std_logic;

    -- Input pulses to time-tag
    trigger_p_i   : in std_logic;
    acq_start_p_i : in std_logic;
    acq_stop_p_i  : in std_logic;
    acq_end_p_i   : in std_logic;

    -- White Rabbit enabled flag
    wr_enabled_i : in std_logic;

    -- White Rabbit timecode interface input
    wr_tm_time_valid_i : in std_logic;
    wr_tm_tai_i        : in std_logic_vector(39 downto 0);
    wr_tm_cycles_i     : in std_logic_vector(27 downto 0);

    -- Trigger time-tag output
    trig_tag_o  : out t_timetag;
    time_trig_o : out std_logic;

    --  Alternative trigger in time
    alt_trigin_enable_o    : out std_logic;
    alt_trigin_enable_i    : in  std_logic;
    alt_trigin_enable_wr_i : in  std_logic;
    alt_trigin_tag_i       : in  t_timetag;
    alt_trigin_o           : out std_logic;

    -- Wishbone interface
    wb_i : in  t_wishbone_slave_in;
    wb_o : out t_wishbone_slave_out);
end timetag_core;

architecture rtl of timetag_core is

  ------------------------------------------------------------------------------
  -- Signals declaration
  ------------------------------------------------------------------------------
  signal current_time  : t_timetag;
  signal time_counter  : t_timetag;
  signal time_trigger  : t_timetag;
  signal trig_tag      : t_timetag;
  signal acq_start_tag : t_timetag;
  signal acq_stop_tag  : t_timetag;
  signal acq_end_tag   : t_timetag;

  signal time_trig   : std_logic;
  signal time_trig_d : std_logic;

  signal local_pps : std_logic;

  signal wr_enabled : std_logic := '0';

  signal regin  : t_timetag_core_master_in;
  signal regout : t_timetag_core_master_out;

  signal alt_trigin        : std_logic;
  signal alt_trigin_d      : std_logic;
  signal alt_trigin_enable : std_logic;

  signal wb_in  : t_wishbone_slave_in;
  signal wb_out : t_wishbone_slave_out;

begin

  -- logic to detect if WR is enabled and timecode is valid
  wr_enabled <= wr_enabled_i and wr_tm_time_valid_i;

  ------------------------------------------------------------------------------
  -- Wishbone interface to UTC core registers
  ------------------------------------------------------------------------------

  cmp_timetag_wb_slave_adapter_in : wb_slave_adapter
    generic map (
      g_master_use_struct  => TRUE,
      g_master_mode        => PIPELINED,
      g_master_granularity => BYTE,
      g_slave_use_struct   => TRUE,
      g_slave_mode         => g_WB_MODE,
      g_slave_granularity  => g_WB_GRANULARITY)
    port map (
      clk_sys_i => clk_i,
      rst_n_i   => rst_n_i,
      slave_i   => wb_i,
      slave_o   => wb_o,
      master_i  => wb_out,
      master_o  => wb_in);

  cmp_timetag_core_regs : entity work.timetag_core_regs
    port map (
      rst_n_i        => rst_n_i,
      clk_i          => clk_i,
      wb_i           => wb_in,
      wb_o           => wb_out,
      timetag_core_i => regin,
      timetag_core_o => regout);

  regin.seconds_upper               <= current_time.seconds(39 downto 32);
  regin.seconds_lower               <= current_time.seconds(31 downto 0);
  regin.coarse                      <= current_time.coarse;
  regin.trig_tag_seconds_upper      <= trig_tag.seconds(39 downto 32);
  regin.trig_tag_seconds_lower      <= trig_tag.seconds(31 downto 0);
  regin.trig_tag_coarse             <= trig_tag.coarse;
  regin.acq_start_tag_seconds_upper <= acq_start_tag.seconds(39 downto 32);
  regin.acq_start_tag_seconds_lower <= acq_start_tag.seconds(31 downto 0);
  regin.acq_start_tag_coarse        <= acq_start_tag.coarse;
  regin.acq_stop_tag_seconds_upper  <= acq_stop_tag.seconds(39 downto 32);
  regin.acq_stop_tag_seconds_lower  <= acq_stop_tag.seconds(31 downto 0);
  regin.acq_stop_tag_coarse         <= acq_stop_tag.coarse;
  regin.acq_end_tag_seconds_upper   <= acq_end_tag.seconds(39 downto 32);
  regin.acq_end_tag_seconds_lower   <= acq_end_tag.seconds(31 downto 0);
  regin.acq_end_tag_coarse          <= acq_end_tag.coarse;

  time_trigger.seconds <= regout.time_trig_seconds_upper & regout.time_trig_seconds_lower;
  time_trigger.coarse  <= regout.time_trig_coarse;

  ------------------------------------------------------------------------------
  -- UTC seconds counter
  ------------------------------------------------------------------------------
  p_timetag_seconds_cnt : process (clk_i)
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        time_counter.seconds <= (others => '0');
      elsif regout.seconds_upper_wr = '1' then
        time_counter.seconds(39 downto 32) <= regout.seconds_upper;
      elsif regout.seconds_lower_wr = '1' then
        time_counter.seconds(31 downto 0) <= regout.seconds_lower;
      elsif local_pps = '1' then
        time_counter.seconds <= std_logic_vector(unsigned(current_time.seconds) + 1);
      else
        time_counter.seconds <= current_time.seconds;
      end if;
    end if;
  end process p_timetag_seconds_cnt;

  current_time.seconds <= wr_tm_tai_i when wr_enabled = '1' else time_counter.seconds;

  ------------------------------------------------------------------------------
  -- UTC 125MHz clock ticks counter
  ------------------------------------------------------------------------------
  p_timetag_coarse_cnt : process (clk_i)
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        time_counter.coarse <= (others => '0');
        local_pps           <= '0';
      elsif regout.coarse_wr = '1' then
        time_counter.coarse <= regout.coarse;
        local_pps           <= '0';
      elsif time_counter.coarse = std_logic_vector(c_TAG_COARSE_MAX - 1) then
        time_counter.coarse <= (others => '0');
        local_pps           <= '1';
      else
        time_counter.coarse <= std_logic_vector(unsigned(current_time.coarse) + 1);
        local_pps           <= '0';
      end if;
    end if;
  end process p_timetag_coarse_cnt;

  current_time.coarse <= wr_tm_cycles_i when wr_enabled = '1' else time_counter.coarse;

  ------------------------------------------------------------------------------
  -- Time trigger signal generation (stretched to two 125MHz cycles)
  ------------------------------------------------------------------------------

  time_trig <= '1' when (time_trigger = current_time) else '0';

  process (clk_i)
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        time_trig_d <= '0';
      else
        time_trig_d <= time_trig;
      end if;
    end if;
  end process;

  time_trig_o <= time_trig or time_trig_d;

  --  Alternative time trigger generation (also stretched).
  alt_trigin <= alt_trigin_enable when alt_trigin_tag_i = current_time else '0';

  process (clk_i)
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        alt_trigin_enable <= '0';
      else
        if alt_trigin_enable_wr_i = '1' then
          --  User write.
          alt_trigin_enable <= alt_trigin_enable_i;
        elsif alt_trigin = '1' then
          --  Auto clear after trigger.
          alt_trigin_enable <= '0';
        end if;
      end if;
    end if;
  end process;

  alt_trigin_enable_o <= alt_trigin_enable;

  process (clk_i)
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        alt_trigin_d <= '0';
      else
        alt_trigin_d <= alt_trigin;
      end if;
    end if;
  end process;

  alt_trigin_o <= alt_trigin or alt_trigin_d;

  ------------------------------------------------------------------------------
  -- Last trigger event time-tag
  ------------------------------------------------------------------------------

  gen_trig_tag_no_adjust : if g_TAG_ADJUST = 0 generate
    p_trig_tag : process (clk_i)
    begin
      if rising_edge(clk_i) then
        if rst_n_i = '0' then
          trig_tag.seconds <= (others => '0');
          trig_tag.coarse  <= (others => '0');
        elsif trigger_p_i = '1' then
          trig_tag <= current_time;
        end if;
      end if;
    end process p_trig_tag;
  end generate gen_trig_tag_no_adjust;


  gen_trig_tag : if g_TAG_ADJUST /= 0 generate
    p_trig_tag : process (clk_i)
      variable v_seconds_now  : unsigned(39 downto 0) := (others => '0');
      variable v_coarse_now   : unsigned(27 downto 0) := (others => '0');
      variable v_seconds_next : unsigned(39 downto 0) := (others => '0');
      variable v_coarse_next  : unsigned(27 downto 0) := (others => '0');
    begin
      if rising_edge(clk_i) then
        if rst_n_i = '0' then
          trig_tag.seconds <= (others => '0');
          trig_tag.coarse  <= (others => '0');
        elsif trigger_p_i = '1' then
          v_seconds_now := unsigned(current_time.seconds);
          v_coarse_now  := unsigned(current_time.coarse);
          if g_TAG_ADJUST > v_coarse_now then
            v_seconds_next := v_seconds_now - 1;
            v_coarse_next  := c_TAG_COARSE_MAX - (g_TAG_ADJUST - v_coarse_now);
          else
            v_seconds_next := v_seconds_now;
            v_coarse_next  := v_coarse_now - g_TAG_ADJUST;
          end if;
          trig_tag.seconds <= std_logic_vector(v_seconds_next);
          trig_tag.coarse  <= std_logic_vector(v_coarse_next);
        end if;
      end if;
    end process p_trig_tag;
  end generate gen_trig_tag;

  trig_tag_o <= trig_tag;

  ------------------------------------------------------------------------------
  -- Last acquisition start event time-tag
  ------------------------------------------------------------------------------
  p_acq_start_tag : process (clk_i)
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        acq_start_tag.seconds <= (others => '0');
        acq_start_tag.coarse  <= (others => '0');
      elsif acq_start_p_i = '1' then
        acq_start_tag <= current_time;
      end if;
    end if;
  end process p_acq_start_tag;

  ------------------------------------------------------------------------------
  -- Last acquisition stop event time-tag
  ------------------------------------------------------------------------------
  p_acq_stop_tag : process (clk_i)
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        acq_stop_tag.seconds <= (others => '0');
        acq_stop_tag.coarse  <= (others => '0');
      elsif acq_stop_p_i = '1' then
        acq_stop_tag <= current_time;
      end if;
    end if;
  end process p_acq_stop_tag;

  ------------------------------------------------------------------------------
  -- Last acquisition end event time-tag
  ------------------------------------------------------------------------------
  p_acq_end_tag : process (clk_i)
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        acq_end_tag.seconds <= (others => '0');
        acq_end_tag.coarse  <= (others => '0');
      elsif acq_end_p_i = '1' then
        acq_end_tag <= current_time;
      end if;
    end if;
  end process p_acq_end_tag;


end rtl;
