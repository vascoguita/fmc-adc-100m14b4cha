-- Do not edit; this file was generated by Cheby using these options:
--  -i fmc_adc_100Ms_channel_regs.cheby --gen-hdl=fmc_adc_100Ms_channel_regs.vhd

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.wishbone_pkg.all;

package fmc_adc_100ms_channel_regs_pkg is
  type t_fmc_adc_100ms_ch_master_out is record
    ctl_ssr          : std_logic_vector(6 downto 0);
    calib_gain       : std_logic_vector(15 downto 0);
    calib_offset     : std_logic_vector(15 downto 0);
    sat_val          : std_logic_vector(14 downto 0);
    trig_thres_val   : std_logic_vector(15 downto 0);
    trig_thres_hyst  : std_logic_vector(15 downto 0);
    trig_dly         : std_logic_vector(31 downto 0);
  end record t_fmc_adc_100ms_ch_master_out;
  subtype t_fmc_adc_100ms_ch_slave_in is t_fmc_adc_100ms_ch_master_out;

  type t_fmc_adc_100ms_ch_slave_out is record
    sta_val          : std_logic_vector(15 downto 0);
  end record t_fmc_adc_100ms_ch_slave_out;
  subtype t_fmc_adc_100ms_ch_master_in is t_fmc_adc_100ms_ch_slave_out;
end fmc_adc_100ms_channel_regs_pkg;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.wishbone_pkg.all;
use work.fmc_adc_100ms_channel_regs_pkg.all;

entity fmc_adc_100ms_channel_regs is
  port (
    rst_n_i              : in    std_logic;
    clk_i                : in    std_logic;
    wb_i                 : in    t_wishbone_slave_in;
    wb_o                 : out   t_wishbone_slave_out;

    -- Wires and registers
    fmc_adc_100ms_ch_i   : in    t_fmc_adc_100ms_ch_master_in;
    fmc_adc_100ms_ch_o   : out   t_fmc_adc_100ms_ch_master_out
  );
end fmc_adc_100ms_channel_regs;

architecture syn of fmc_adc_100ms_channel_regs is
  signal rd_int                         : std_logic;
  signal wr_int                         : std_logic;
  signal rd_ack_int                     : std_logic;
  signal wr_ack_int                     : std_logic;
  signal wb_en                          : std_logic;
  signal ack_int                        : std_logic;
  signal wb_rip                         : std_logic;
  signal wb_wip                         : std_logic;
  signal ctl_ssr_reg                    : std_logic_vector(6 downto 0);
  signal calib_gain_reg                 : std_logic_vector(15 downto 0);
  signal calib_offset_reg               : std_logic_vector(15 downto 0);
  signal sat_val_reg                    : std_logic_vector(14 downto 0);
  signal trig_thres_val_reg             : std_logic_vector(15 downto 0);
  signal trig_thres_hyst_reg            : std_logic_vector(15 downto 0);
  signal trig_dly_reg                   : std_logic_vector(31 downto 0);
  signal reg_rdat_int                   : std_logic_vector(31 downto 0);
  signal rd_ack1_int                    : std_logic;
begin

  -- WB decode signals
  wb_en <= wb_i.cyc and wb_i.stb;

  process (clk_i) begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        wb_rip <= '0';
      else
        wb_rip <= (wb_rip or (wb_en and not wb_i.we)) and not rd_ack_int;
      end if;
    end if;
  end process;
  rd_int <= (wb_en and not wb_i.we) and not wb_rip;

  process (clk_i) begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        wb_wip <= '0';
      else
        wb_wip <= (wb_wip or (wb_en and wb_i.we)) and not wr_ack_int;
      end if;
    end if;
  end process;
  wr_int <= (wb_en and wb_i.we) and not wb_wip;

  ack_int <= rd_ack_int or wr_ack_int;
  wb_o.ack <= ack_int;
  wb_o.stall <= not ack_int and wb_en;
  wb_o.rty <= '0';
  wb_o.err <= '0';

  -- Assign outputs
  fmc_adc_100ms_ch_o.ctl_ssr <= ctl_ssr_reg;
  fmc_adc_100ms_ch_o.calib_gain <= calib_gain_reg;
  fmc_adc_100ms_ch_o.calib_offset <= calib_offset_reg;
  fmc_adc_100ms_ch_o.sat_val <= sat_val_reg;
  fmc_adc_100ms_ch_o.trig_thres_val <= trig_thres_val_reg;
  fmc_adc_100ms_ch_o.trig_thres_hyst <= trig_thres_hyst_reg;
  fmc_adc_100ms_ch_o.trig_dly <= trig_dly_reg;

  -- Process for write requests.
  process (clk_i) begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        wr_ack_int <= '0';
        ctl_ssr_reg <= "0000000";
        calib_gain_reg <= "0000000000000000";
        calib_offset_reg <= "0000000000000000";
        sat_val_reg <= "000000000000000";
        trig_thres_val_reg <= "0000000000000000";
        trig_thres_hyst_reg <= "0000000000000000";
        trig_dly_reg <= "00000000000000000000000000000000";
      else
        wr_ack_int <= '0';
        case wb_i.adr(4 downto 2) is
        when "000" => 
          -- Register ctl
          if wr_int = '1' then
            ctl_ssr_reg <= wb_i.dat(6 downto 0);
          end if;
          wr_ack_int <= wr_int;
        when "001" => 
          -- Register sta
        when "010" => 
          -- Register calib
          if wr_int = '1' then
            calib_gain_reg <= wb_i.dat(15 downto 0);
            calib_offset_reg <= wb_i.dat(31 downto 16);
          end if;
          wr_ack_int <= wr_int;
        when "011" => 
          -- Register sat
          if wr_int = '1' then
            sat_val_reg <= wb_i.dat(14 downto 0);
          end if;
          wr_ack_int <= wr_int;
        when "100" => 
          -- Register trig_thres
          if wr_int = '1' then
            trig_thres_val_reg <= wb_i.dat(15 downto 0);
            trig_thres_hyst_reg <= wb_i.dat(31 downto 16);
          end if;
          wr_ack_int <= wr_int;
        when "101" => 
          -- Register trig_dly
          if wr_int = '1' then
            trig_dly_reg <= wb_i.dat;
          end if;
          wr_ack_int <= wr_int;
        when others =>
          wr_ack_int <= wr_int;
        end case;
      end if;
    end if;
  end process;

  -- Process for registers read.
  process (clk_i) begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        rd_ack1_int <= '0';
      else
        reg_rdat_int <= (others => 'X');
        case wb_i.adr(4 downto 2) is
        when "000" => 
          -- ctl
          reg_rdat_int(6 downto 0) <= ctl_ssr_reg;
          rd_ack1_int <= rd_int;
        when "001" => 
          -- sta
          reg_rdat_int(15 downto 0) <= fmc_adc_100ms_ch_i.sta_val;
          rd_ack1_int <= rd_int;
        when "010" => 
          -- calib
          reg_rdat_int(15 downto 0) <= calib_gain_reg;
          reg_rdat_int(31 downto 16) <= calib_offset_reg;
          rd_ack1_int <= rd_int;
        when "011" => 
          -- sat
          reg_rdat_int(14 downto 0) <= sat_val_reg;
          rd_ack1_int <= rd_int;
        when "100" => 
          -- trig_thres
          reg_rdat_int(15 downto 0) <= trig_thres_val_reg;
          reg_rdat_int(31 downto 16) <= trig_thres_hyst_reg;
          rd_ack1_int <= rd_int;
        when "101" => 
          -- trig_dly
          reg_rdat_int <= trig_dly_reg;
          rd_ack1_int <= rd_int;
        when others =>
          reg_rdat_int <= (others => 'X');
          rd_ack1_int <= rd_int;
        end case;
      end if;
    end if;
  end process;

  -- Process for read requests.
  process (wb_i.adr, reg_rdat_int, rd_ack1_int, rd_int) begin
    -- By default ack read requests
    wb_o.dat <= (others => '0');
    case wb_i.adr(4 downto 2) is
    when "000" => 
      -- ctl
      wb_o.dat <= reg_rdat_int;
      rd_ack_int <= rd_ack1_int;
    when "001" => 
      -- sta
      wb_o.dat <= reg_rdat_int;
      rd_ack_int <= rd_ack1_int;
    when "010" => 
      -- calib
      wb_o.dat <= reg_rdat_int;
      rd_ack_int <= rd_ack1_int;
    when "011" => 
      -- sat
      wb_o.dat <= reg_rdat_int;
      rd_ack_int <= rd_ack1_int;
    when "100" => 
      -- trig_thres
      wb_o.dat <= reg_rdat_int;
      rd_ack_int <= rd_ack1_int;
    when "101" => 
      -- trig_dly
      wb_o.dat <= reg_rdat_int;
      rd_ack_int <= rd_ack1_int;
    when others =>
      rd_ack_int <= rd_int;
    end case;
  end process;
end syn;
