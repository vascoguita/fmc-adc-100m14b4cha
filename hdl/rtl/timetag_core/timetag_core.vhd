-------------------------------------------------------------------------------
-- Title      : Time-tagging core
-- Project    : FMC ADC 100M 14B 4CHA gateware
-- URL        : http://www.ohwr.org/projects/fmc-adc-100m14b4cha-gw
-------------------------------------------------------------------------------
-- File       : timetag_core.vhd
-- Author(s)  : Matthieu Cattin <matthieu.cattin@cern.ch>
--            : Dimitrios Lampridis  <dimitrios.lampridis@cern.ch>
-- Company    : CERN (BE-CO-HT)
-- Created    : 2011-11-18
-- Last update: 2019-05-02
-- Standard   : VHDL'93/02
-------------------------------------------------------------------------------
-- Description: Implements a UTC seconds counter and a 125MHz system clock
-- ticks counter to time-tag trigger, acquisition start and stop events.
-------------------------------------------------------------------------------
-- Copyright (c) 2011-2016 CERN (BE-CO-HT)
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
-- 2011-11-18  1.0      Matthieu Cattin
-------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;
use work.timetag_core_pkg.all;
use work.timetag_core_wbgen2_pkg.all;

entity timetag_core is
  port (
    -- Clock, reset
    clk_i   : in std_logic;                       -- Must be 125MHz
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
    wb_adr_i : in  std_logic_vector(4 downto 0);
    wb_dat_i : in  std_logic_vector(31 downto 0);
    wb_dat_o : out std_logic_vector(31 downto 0);
    wb_cyc_i : in  std_logic;
    wb_sel_i : in  std_logic_vector(3 downto 0);
    wb_stb_i : in  std_logic;
    wb_we_i  : in  std_logic;
    wb_ack_o : out std_logic
    );
end timetag_core;


architecture rtl of timetag_core is


  ------------------------------------------------------------------------------
  -- Components declaration
  ------------------------------------------------------------------------------
  component timetag_core_regs is
    port (
      rst_n_i    : in  std_logic;
      clk_sys_i  : in  std_logic;
      wb_adr_i   : in  std_logic_vector(4 downto 0);
      wb_dat_i   : in  std_logic_vector(31 downto 0);
      wb_dat_o   : out std_logic_vector(31 downto 0);
      wb_cyc_i   : in  std_logic;
      wb_sel_i   : in  std_logic_vector(3 downto 0);
      wb_stb_i   : in  std_logic;
      wb_we_i    : in  std_logic;
      wb_ack_o   : out std_logic;
      wb_stall_o : out std_logic;
      regs_i     : in  t_timetag_core_in_registers;
      regs_o     : out t_timetag_core_out_registers);
  end component timetag_core_regs;

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

  signal time_trig     : std_logic;
  signal time_trig_d   : std_logic;

  signal local_pps : std_logic;

  signal wr_enabled : std_logic := '0';

  signal regin  : t_timetag_core_in_registers;
  signal regout : t_timetag_core_out_registers;

  signal alt_trigin        : std_logic;
  signal alt_trigin_d      : std_logic;
  signal alt_trigin_enable : std_logic;

begin

  -- logic to detect if WR is enabled and timecode is valid
  wr_enabled <= wr_enabled_i and wr_tm_time_valid_i;

  ------------------------------------------------------------------------------
  -- Wishbone interface to UTC core registers
  ------------------------------------------------------------------------------
  cmp_timetag_core_regs : timetag_core_regs
    port map (
      rst_n_i    => rst_n_i,
      clk_sys_i  => clk_i,
      wb_adr_i   => wb_adr_i,
      wb_dat_i   => wb_dat_i,
      wb_dat_o   => wb_dat_o,
      wb_cyc_i   => wb_cyc_i,
      wb_sel_i   => wb_sel_i,
      wb_stb_i   => wb_stb_i,
      wb_we_i    => wb_we_i,
      wb_ack_o   => wb_ack_o,
      wb_stall_o => open,
      regs_i     => regin,
      regs_o     => regout);

  regin.seconds_upper_i               <= current_time.seconds(39 downto 32);
  regin.seconds_lower_i               <= current_time.seconds(31 downto 0);
  regin.coarse_i                      <= current_time.coarse;
  regin.trig_tag_seconds_upper_i      <= trig_tag.seconds(39 downto 32);
  regin.trig_tag_seconds_lower_i      <= trig_tag.seconds(31 downto 0);
  regin.trig_tag_coarse_i             <= trig_tag.coarse;
  regin.acq_start_tag_seconds_upper_i <= acq_start_tag.seconds(39 downto 32);
  regin.acq_start_tag_seconds_lower_i <= acq_start_tag.seconds(31 downto 0);
  regin.acq_start_tag_coarse_i        <= acq_start_tag.coarse;
  regin.acq_stop_tag_seconds_upper_i  <= acq_stop_tag.seconds(39 downto 32);
  regin.acq_stop_tag_seconds_lower_i  <= acq_stop_tag.seconds(31 downto 0);
  regin.acq_stop_tag_coarse_i         <= acq_stop_tag.coarse;
  regin.acq_end_tag_seconds_upper_i   <= acq_end_tag.seconds(39 downto 32);
  regin.acq_end_tag_seconds_lower_i   <= acq_end_tag.seconds(31 downto 0);
  regin.acq_end_tag_coarse_i          <= acq_end_tag.coarse;

  time_trigger.seconds <= regout.time_trig_seconds_upper_o & regout.time_trig_seconds_lower_o;
  time_trigger.coarse  <= regout.time_trig_coarse_o;

  ------------------------------------------------------------------------------
  -- UTC seconds counter
  ------------------------------------------------------------------------------
  p_timetag_seconds_cnt : process (clk_i)
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        time_counter.seconds <= (others => '0');
      elsif regout.seconds_upper_load_o = '1' then
        time_counter.seconds(39 downto 32) <= regout.seconds_upper_o;
      elsif regout.seconds_lower_load_o = '1' then
        time_counter.seconds(31 downto 0) <= regout.seconds_lower_o;
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
      elsif regout.coarse_load_o = '1' then
        time_counter.coarse <= regout.coarse_o;
        local_pps           <= '0';
      elsif time_counter.coarse = std_logic_vector(to_unsigned(124999999, 28)) then
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
