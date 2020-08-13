-------------------------------------------------------------------------------
-- Title      : FMC ADC 100Ms/s core
-- Project    : FMC ADC 100M 14B 4CHA gateware
-- URL        : http://www.ohwr.org/projects/fmc-adc-100m14b4cha-gw
-------------------------------------------------------------------------------
-- File       : fmc_adc_100Ms_core.vhd
-- Company    : CERN (BE-CO-HT)
-- Created    : 2011-02-24
-- Standard   : VHDL'93/02
-------------------------------------------------------------------------------
-- Description: FMC ADC 100Ms/s core.
-------------------------------------------------------------------------------
-- Copyright (c) 2011-2020 CERN (BE-CO-HT)
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

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library unisim;
use unisim.vcomponents.all;

library work;
use work.timetag_core_defs_pkg.all;
use work.genram_pkg.all;
use work.gencores_pkg.all;
use work.wishbone_pkg.all;
use work.fmc_adc_100ms_csr_pkg.all;

entity fmc_adc_100Ms_core is
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

    -- Trigger time-tag inputs
    trigger_tag_i   : in t_timetag;
    time_trig_i     : in std_logic;
    aux_time_trig_i : in std_logic;

    -- WR status (for trigout).
    wr_tm_link_up_i    : in std_logic;
    wr_tm_time_valid_i : in std_logic;
    wr_enable_i        : in std_logic;

    -- FMC interface
    ext_trigger_p_i : in std_logic;               -- External trigger
    ext_trigger_n_i : in std_logic;

    adc_dco_p_i  : in std_logic;                     -- ADC data clock
    adc_dco_n_i  : in std_logic;
    adc_fr_p_i   : in std_logic;                     -- ADC frame start
    adc_fr_n_i   : in std_logic;
    adc_outa_p_i : in std_logic_vector(3 downto 0);  -- ADC serial data (odd bits)
    adc_outa_n_i : in std_logic_vector(3 downto 0);
    adc_outb_p_i : in std_logic_vector(3 downto 0);  -- ADC serial data (even bits)
    adc_outb_n_i : in std_logic_vector(3 downto 0);

    gpio_dac_clr_n_o : out std_logic;             -- offset DACs clear (active low)
    gpio_led_acq_o   : out std_logic;             -- Mezzanine front panel power LED (PWR)
    gpio_led_trig_o  : out std_logic;             -- Mezzanine front panel trigger LED (TRIG)
    gpio_ssr_ch1_o   : out std_logic_vector(6 downto 0);  -- Channel 1 solid state relays control
    gpio_ssr_ch2_o   : out std_logic_vector(6 downto 0);  -- Channel 2 solid state relays control
    gpio_ssr_ch3_o   : out std_logic_vector(6 downto 0);  -- Channel 3 solid state relays control
    gpio_ssr_ch4_o   : out std_logic_vector(6 downto 0);  -- Channel 4 solid state relays control
    gpio_si570_oe_o  : out std_logic);            -- Si570 (programmable oscillator) output enable

end fmc_adc_100Ms_core;

architecture rtl of fmc_adc_100Ms_core is

  ------------------------------------------------------------------------------
  -- Constants declaration
  ------------------------------------------------------------------------------
  constant c_DPRAM_DEPTH : integer := f_log2_size(g_MULTISHOT_RAM_SIZE);

  -- Calculate the maximum number of available samples per multishot trigger
  -- Note: we subtract 2 for the timetag, and 1 more because of bug when number
  -- of samples equals the size of the dpram
  constant c_MULTISHOT_SAMPLE_DEPTH : std_logic_vector(31 downto 0) :=
    std_logic_vector(to_unsigned(g_MULTISHOT_RAM_SIZE - 3, 32));

  ------------------------------------------------------------------------------
  -- Types declaration
  ------------------------------------------------------------------------------
  type t_acq_fsm_state is (IDLE, PRE_TRIG, WAIT_TRIG, POST_TRIG, TRIG_TAG, DECR_SHOT);
  type t_fmc_adc_vec16_array is array (positive range<>) of std_logic_vector(15 downto 0);
  type t_fmc_adc_vec32_array is array (positive range<>) of std_logic_vector(31 downto 0);
  type t_fmc_adc_uint32_array is array (positive range<>) of unsigned(31 downto 0);

  ------------------------------------------------------------------------------
  -- Signals declaration
  ------------------------------------------------------------------------------

  -- Reset
  signal fs_rst_n    : std_logic;
  signal serdes_arst : std_logic;

  -- Clocks and PLL
  signal clk_fb        : std_logic;
  signal clk_fb_buf    : std_logic;
  signal locked_in     : std_logic;
  signal serdes_clk    : std_logic;
  signal fs_clk        : std_logic;
  signal fs_clk_buf    : std_logic;
  signal fs_freq       : std_logic_vector(31 downto 0);
  signal fs_freq_t     : std_logic_vector(31 downto 0);
  signal fs_freq_valid : std_logic;

  attribute keep           : string;
  attribute keep of fs_clk : signal is "TRUE";

  -- SerDes
  signal serdes_out_data         : std_logic_vector(63 downto 0);
  signal serdes_out_data_synced  : std_logic_vector(63 downto 0);
  signal serdes_man_bitslip      : std_logic;
  signal serdes_man_bitslip_sync : std_logic;
  signal serdes_locked           : std_logic;
  signal serdes_locked_sync      : std_logic;
  signal serdes_synced           : std_logic;
  signal serdes_synced_sync      : std_logic;

  -- Trigger
  signal ext_trig_a, ext_trig       : std_logic;
  signal ext_trig_d                 : std_logic;
  signal ext_trig_delay             : std_logic_vector(31 downto 0);
  signal ext_trig_delay_cnt         : unsigned(31 downto 0);
  signal ext_trig_delay_bsy         : std_logic;
  signal ext_trig_en, ext_trig_sync : std_logic;
  signal ext_trig_fixed_delay       : std_logic_vector(g_TRIG_DELAY_EXT+2 downto 0);
  signal ext_trig_p, ext_trig_n     : std_logic;
  signal ext_trig_pol               : std_logic;
  signal int_trig                   : std_logic_vector(1 to 4);
  signal int_trig_d                 : std_logic_vector(1 to 4);
  signal int_trig_data              : t_fmc_adc_vec16_array(1 to 4);
  signal int_trig_delay_in          : t_fmc_adc_vec32_array(1 to 4);
  signal int_trig_delay             : t_fmc_adc_vec32_array(1 to 4);
  signal int_trig_delay_cnt         : t_fmc_adc_uint32_array(1 to 4);
  signal int_trig_delay_bsy         : std_logic_vector(1 to 4);
  signal int_trig_en_in             : std_logic_vector(1 to 4);
  signal int_trig_pol_in            : std_logic_vector(1 to 4);
  signal int_trig_en                : std_logic_vector(1 to 4);
  signal int_trig_pol               : std_logic_vector(1 to 4);
  signal int_trig_thres             : t_fmc_adc_vec16_array(1 to 4);
  signal int_trig_thres_hyst        : t_fmc_adc_vec16_array(1 to 4);
  signal int_trig_thres_in          : t_fmc_adc_vec16_array(1 to 4);
  signal int_trig_thres_hyst_in     : t_fmc_adc_vec16_array(1 to 4);
  signal sw_trig                    : std_logic;
  signal sw_trig_fixed_delay        : std_logic_vector(g_TRIG_DELAY_SW+2 downto 0);
  signal sw_trig_in                 : std_logic := '0';
  signal sw_trig_sync_ack           : std_logic := '0';
  signal time_trig, time_trig_sync  : std_logic;
  signal time_trig_en               : std_logic;
  signal time_trig_fixed_delay      : std_logic_vector(g_TRIG_DELAY_SW+2 downto 0);
  signal aux_time_trig              : std_logic;
  signal aux_time_trig_sync         : std_logic;
  signal aux_time_trig_fixed_delay  : std_logic_vector(g_TRIG_DELAY_SW+2 downto 0);
  signal trig                       : std_logic;
  signal trig_align                 : std_logic_vector(8 downto 0);
  signal trig_storage               : std_logic_vector(31 downto 0);
  signal trig_storage_clear         : std_logic;
  signal trig_src_vector            : std_logic_vector(7 downto 0);

  -- Down-sampling
  signal downsample_factor : std_logic_vector(31 downto 0) := (others => '0');
  signal downsample_cnt    : unsigned(31 downto 0);
  signal downsample_en     : std_logic;

  -- Sync FIFO (from fs_clk to sys_clk_i)
  signal sync_fifo_din   : std_logic_vector(72 downto 0);
  signal sync_fifo_dout  : std_logic_vector(72 downto 0);
  signal sync_fifo_empty : std_logic;
  signal sync_fifo_full  : std_logic;
  signal sync_fifo_wr    : std_logic;
  signal sync_fifo_rd    : std_logic;
  signal sync_fifo_valid : std_logic;

  -- Gain/offset calibration and saturation value
  signal sync_calib_apply   : std_logic;
  signal sync_calib_busy    : std_logic;
  signal gain_calibr        : std_logic_vector(63 downto 0);
  signal offset_calibr      : std_logic_vector(63 downto 0);
  signal sync_calib_in      : std_logic_vector(127 downto 0);
  signal sync_calib_out     : std_logic_vector(127 downto 0);
  signal data_calibr_in     : std_logic_vector(63 downto 0);
  signal data_calibr_out    : std_logic_vector(63 downto 0);
  signal data_calibr_out_d1 : std_logic_vector(63 downto 0);
  signal data_calibr_out_d2 : std_logic_vector(63 downto 0);
  signal data_calibr_out_d3 : std_logic_vector(63 downto 0);
  signal sat_val            : std_logic_vector(59 downto 0);
  signal sat_val_in         : std_logic_vector(59 downto 0);

  -- Acquisition FSM
  signal acq_fsm_current_state : t_acq_fsm_state;
  signal acq_fsm_state         : std_logic_vector(2 downto 0);
  signal fsm_cmd               : std_logic_vector(1 downto 0);
  signal ctl_reg_wr            : std_logic;
  signal acq_start             : std_logic;
  signal acq_stop              : std_logic;
  signal acq_trig              : std_logic;
  signal acq_trig_d            : std_logic;
  signal acq_end               : std_logic;
  signal acq_end_d             : std_logic;
  signal acq_in_pre_trig       : std_logic;
  signal acq_in_wait_trig      : std_logic;
  signal acq_in_post_trig      : std_logic;
  signal acq_in_trig_tag       : std_logic;
  signal acq_in_trig_tag_d     : std_logic;
  signal samples_wr_en         : std_logic;
  signal acq_config_ok         : std_logic;

  -- Trigger tag insertion in data
  signal trig_tag_done : std_logic;
  signal trig_tag_data : std_logic_vector(63 downto 0);

  -- pre/post trigger and shots counters
  signal pre_trig_value       : std_logic_vector(31 downto 0);
  signal pre_trig_cnt         : unsigned(31 downto 0);
  signal pre_trig_done        : std_logic;
  signal post_trig_value      : std_logic_vector(31 downto 0);
  signal post_trig_cnt        : unsigned(31 downto 0);
  signal post_trig_done       : std_logic;
  signal samples_cnt          : unsigned(31 downto 0);
  signal shots_value          : std_logic_vector(15 downto 0);
  signal shots_cnt            : unsigned(15 downto 0);
  signal remaining_shots      : std_logic_vector(15 downto 0);
  signal shots_done           : std_logic;
  signal shots_decr           : std_logic;
  signal single_shot          : std_logic;
  signal multishot_buffer_sel : std_logic;

  -- Multi-shot mode
  signal dpram_addra_cnt       : unsigned(c_DPRAM_DEPTH-1 downto 0);
  signal dpram_addra_trig      : unsigned(c_DPRAM_DEPTH-1 downto 0);
  signal dpram_addra_post_done : unsigned(c_DPRAM_DEPTH-1 downto 0);
  signal dpram_addrb_cnt       : unsigned(c_DPRAM_DEPTH-1 downto 0);
  signal dpram_dout            : std_logic_vector(63 downto 0);
  signal dpram_valid           : std_logic;
  signal dpram_valid_t         : std_logic;

  signal dpram0_dina  : std_logic_vector(63 downto 0);
  signal dpram0_addra : std_logic_vector(c_DPRAM_DEPTH-1 downto 0);
  signal dpram0_wea   : std_logic;
  signal dpram0_addrb : std_logic_vector(c_DPRAM_DEPTH-1 downto 0);
  signal dpram0_doutb : std_logic_vector(63 downto 0);

  signal dpram1_dina  : std_logic_vector(63 downto 0);
  signal dpram1_addra : std_logic_vector(c_DPRAM_DEPTH-1 downto 0);
  signal dpram1_wea   : std_logic;
  signal dpram1_addrb : std_logic_vector(c_DPRAM_DEPTH-1 downto 0);
  signal dpram1_doutb : std_logic_vector(63 downto 0);

  -- Wishbone to DDR flowcontrol FIFO
  signal wb_ddr_fifo_din   : std_logic_vector(64 downto 0);
  signal wb_ddr_fifo_dout  : std_logic_vector(64 downto 0);
  signal wb_ddr_fifo_empty : std_logic;
  signal wb_ddr_fifo_full  : std_logic;
  signal wb_ddr_fifo_wr    : std_logic;
  signal wb_ddr_fifo_rd    : std_logic;
  signal wb_ddr_fifo_wr_en : std_logic;

  -- RAM address counter
  signal ram_addr_cnt : unsigned(28 downto 0);
  signal test_data_en : std_logic;
  signal trig_addr    : std_logic_vector(31 downto 0);
  signal mem_ovr      : std_logic;

  -- IO from CSR registers
  signal csr_regin  : t_fmc_adc_100ms_csr_master_in;
  signal csr_regout : t_fmc_adc_100ms_csr_master_out;

  -- LEDs
  signal trig_led     : std_logic;
  signal trig_led_man : std_logic;
  signal acq_led      : std_logic;
  signal acq_led_man  : std_logic;

  -- from/to wb slave adapters
  signal wb_csr_in  : t_wishbone_slave_in;
  signal wb_csr_out : t_wishbone_slave_out;

begin

  ------------------------------------------------------------------------------
  -- WB slave adapters to/from the outside
  ------------------------------------------------------------------------------

  cmp_csr_wb_slave_adapter : wb_slave_adapter
    generic map (
      g_MASTER_USE_STRUCT  => TRUE,
      g_MASTER_MODE        => PIPELINED,
      g_MASTER_GRANULARITY => BYTE,
      g_SLAVE_USE_STRUCT   => TRUE,
      g_SLAVE_MODE         => g_WB_CSR_MODE,
      g_SLAVE_GRANULARITY  => g_WB_CSR_GRANULARITY)
    port map (
      clk_sys_i => sys_clk_i,
      rst_n_i   => sys_rst_n_i,
      slave_i   => wb_csr_slave_i,
      slave_o   => wb_csr_slave_o,
      master_i  => wb_csr_out,
      master_o  => wb_csr_in);

  ------------------------------------------------------------------------------
  -- LEDs
  ------------------------------------------------------------------------------
  cmp_acq_led: gc_extend_pulse
    generic map (
      g_WIDTH => 12500000)
    port map (
      clk_i      => sys_clk_i,
      rst_n_i    => sys_rst_n_i,
      pulse_i    => samples_wr_en,
      extended_o => acq_led);

  gpio_led_acq_o <= acq_led or acq_led_man;

  cmp_trig_led: gc_extend_pulse
    generic map (
      g_WIDTH => 12500000)
    port map (
      clk_i      => sys_clk_i,
      rst_n_i    => sys_rst_n_i,
      pulse_i    => acq_trig,
      extended_o => trig_led);

  gpio_led_trig_o <= trig_led or trig_led_man;

  ------------------------------------------------------------------------------
  -- Resets
  ------------------------------------------------------------------------------

  cmp_sys_rst_fs_resync : gc_sync
    port map (
      clk_i     => fs_clk,
      rst_n_a_i => '1',
      d_i       => sys_rst_n_i,
      q_o       => fs_rst_n);

  serdes_arst <= not fs_rst_n;

  ------------------------------------------------------------------------------
  -- Sampinling clock frequency meter
  ------------------------------------------------------------------------------

  cmp_fs_freq : gc_frequency_meter
    generic map(
      g_WITH_INTERNAL_TIMEBASE => TRUE,
      g_CLK_SYS_FREQ           => 125000000,
      g_SYNC_OUT               => TRUE,
      g_COUNTER_BITS           => 32
      )
    port map(
      clk_sys_i    => sys_clk_i,
      clk_in_i     => fs_clk,
      rst_n_i      => '1',
      pps_p1_i     => '0',
      freq_o       => fs_freq_t,
      freq_valid_o => fs_freq_valid
      );

  p_fs_freq : process (sys_clk_i)
  begin
    if rising_edge(sys_clk_i) then
      if fs_freq_valid = '1' then
        fs_freq <= fs_freq_t;
      end if;
    end if;
  end process p_fs_freq;

  ------------------------------------------------------------------------------
  -- ADC SerDes
  ------------------------------------------------------------------------------
  cmp_man_bitslip_sync : gc_sync
    port map (
      clk_i     => fs_clk,
      rst_n_a_i => '1',
      d_i       => serdes_man_bitslip,
      q_o       => serdes_man_bitslip_sync);

  cmp_adc_serdes : entity work.ltc2174_2l16b_receiver
    generic map (
      g_USE_PLL => g_SPARTAN6_USE_PLL)
    port map (
      adc_dco_p_i     => adc_dco_p_i,
      adc_dco_n_i     => adc_dco_n_i,
      adc_fr_p_i      => adc_fr_p_i,
      adc_fr_n_i      => adc_fr_n_i,
      adc_outa_p_i    => adc_outa_p_i,
      adc_outa_n_i    => adc_outa_n_i,
      adc_outb_p_i    => adc_outb_p_i,
      adc_outb_n_i    => adc_outb_n_i,
      serdes_arst_i   => serdes_arst,
      serdes_bslip_i  => serdes_man_bitslip_sync,
      serdes_locked_o => serdes_locked,
      serdes_synced_o => serdes_synced,
      adc_data_o      => serdes_out_data,
      adc_clk_o       => fs_clk);

  cmp_serdes_synced_sync : gc_sync
    port map (
      clk_i     => sys_clk_i,
      rst_n_a_i => '1',
      d_i       => serdes_synced,
      q_o       => serdes_synced_sync);

  cmp_serdes_locked_sync : gc_sync
    port map (
      clk_i     => sys_clk_i,
      rst_n_a_i => '1',
      d_i       => serdes_locked,
      q_o       => serdes_locked_sync);

  ------------------------------------------------------------------------------
  -- ADC core control and status registers (CSR)
  ------------------------------------------------------------------------------
  cmp_fmc_adc_100Ms_csr : entity work.fmc_adc_100Ms_csr
    port map (
      rst_n_i             => sys_rst_n_i,
      clk_i               => sys_clk_i,
      wb_i                => wb_csr_in,
      wb_o                => wb_csr_out,
      fmc_adc_100Ms_csr_i => csr_regin,
      fmc_adc_100Ms_csr_o => csr_regout);

  csr_regin.sta_fsm           <= acq_fsm_state;
  csr_regin.sta_serdes_pll    <= serdes_locked_sync;
  csr_regin.sta_serdes_synced <= serdes_synced_sync;
  csr_regin.sta_acq_cfg       <= acq_config_ok;
  csr_regin.sta_fmc_nr        <= std_logic_vector(to_unsigned(g_FMC_ADC_NR, 2));
  csr_regin.sta_calib_busy    <= sync_calib_busy;
  csr_regin.trig_stat_ext     <= trig_storage(0);
  csr_regin.trig_stat_sw      <= trig_storage(1);
  csr_regin.trig_stat_time    <= trig_storage(4);
  csr_regin.trig_stat_ch1     <= trig_storage(8);
  csr_regin.trig_stat_ch2     <= trig_storage(9);
  csr_regin.trig_stat_ch3     <= trig_storage(10);
  csr_regin.trig_stat_ch4     <= trig_storage(11);
  csr_regin.trig_en_sw        <= '1';
  csr_regin.trig_en_aux_time  <= '1';
  csr_regin.shots_remain      <= remaining_shots;
  csr_regin.trig_pos          <= trig_addr;
  csr_regin.fs_freq           <= fs_freq;
  csr_regin.samples_cnt       <= std_logic_vector(samples_cnt);
  csr_regin.ch1_sta_val       <= serdes_out_data_synced(15 downto 0);
  csr_regin.ch2_sta_val       <= serdes_out_data_synced(31 downto 16);
  csr_regin.ch3_sta_val       <= serdes_out_data_synced(47 downto 32);
  csr_regin.ch4_sta_val       <= serdes_out_data_synced(63 downto 48);
  csr_regin.multi_depth       <= c_MULTISHOT_SAMPLE_DEPTH;

  ctl_reg_wr                <= csr_regout.ctl_wr;
  fsm_cmd                   <= csr_regout.ctl_fsm_cmd;
  serdes_man_bitslip        <= csr_regout.ctl_man_bitslip and ctl_reg_wr;
  test_data_en              <= csr_regout.ctl_test_data_en;
  trig_led_man              <= csr_regout.ctl_trig_led;
  acq_led_man               <= csr_regout.ctl_acq_led;
  trig_storage_clear        <= csr_regout.ctl_clear_trig_stat and ctl_reg_wr;
  sync_calib_apply          <= csr_regout.ctl_calib_apply and ctl_reg_wr;
  int_trig_delay_in(1)      <= csr_regout.ch1_trig_dly;
  int_trig_delay_in(2)      <= csr_regout.ch2_trig_dly;
  int_trig_delay_in(3)      <= csr_regout.ch3_trig_dly;
  int_trig_delay_in(4)      <= csr_regout.ch4_trig_dly;
  int_trig_en_in(1)         <= csr_regout.trig_en_ch1;
  int_trig_en_in(2)         <= csr_regout.trig_en_ch2;
  int_trig_en_in(3)         <= csr_regout.trig_en_ch3;
  int_trig_en_in(4)         <= csr_regout.trig_en_ch4;
  int_trig_pol_in(1)        <= csr_regout.trig_pol_ch1;
  int_trig_pol_in(2)        <= csr_regout.trig_pol_ch2;
  int_trig_pol_in(3)        <= csr_regout.trig_pol_ch3;
  int_trig_pol_in(4)        <= csr_regout.trig_pol_ch4;
  int_trig_thres_in(1)      <= csr_regout.ch1_trig_thres_val;
  int_trig_thres_in(2)      <= csr_regout.ch2_trig_thres_val;
  int_trig_thres_in(3)      <= csr_regout.ch3_trig_thres_val;
  int_trig_thres_in(4)      <= csr_regout.ch4_trig_thres_val;
  int_trig_thres_hyst_in(1) <= csr_regout.ch1_trig_thres_hyst;
  int_trig_thres_hyst_in(2) <= csr_regout.ch2_trig_thres_hyst;
  int_trig_thres_hyst_in(3) <= csr_regout.ch3_trig_thres_hyst;
  int_trig_thres_hyst_in(4) <= csr_regout.ch4_trig_thres_hyst;
  shots_value               <= csr_regout.shots_nbr;
  pre_trig_value            <= csr_regout.pre_samples;
  post_trig_value           <= csr_regout.post_samples;

  sync_calib_in <= csr_regout.ch4_calib_offset & csr_regout.ch3_calib_offset &
                   csr_regout.ch2_calib_offset & csr_regout.ch1_calib_offset &
                   csr_regout.ch4_calib_gain & csr_regout.ch3_calib_gain &
                   csr_regout.ch2_calib_gain & csr_regout.ch1_calib_gain;

  sat_val_in <= csr_regout.ch4_sat_val & csr_regout.ch3_sat_val &
                csr_regout.ch2_sat_val & csr_regout.ch1_sat_val;

  -- Delays for user-controlled GPIO outputs to help with timing
  p_delay_gpio_ssr : process (sys_clk_i) is
  begin
    if rising_edge(sys_clk_i) then
      gpio_si570_oe_o  <= csr_regout.ctl_fmc_clk_oe;
      gpio_dac_clr_n_o <= csr_regout.ctl_offset_dac_clr_n;
      gpio_ssr_ch1_o   <= csr_regout.ch1_ctl_ssr;
      gpio_ssr_ch2_o   <= csr_regout.ch2_ctl_ssr;
      gpio_ssr_ch3_o   <= csr_regout.ch3_ctl_ssr;
      gpio_ssr_ch4_o   <= csr_regout.ch4_ctl_ssr;
    end if;
  end process p_delay_gpio_ssr;

  cmp_ext_trig_en_sync : gc_sync
    port map (
      clk_i     => fs_clk,
      rst_n_a_i => '1',
      d_i       => csr_regout.trig_en_ext,
      q_o       => ext_trig_en);

  cmp_ext_trig_pol_sync : gc_sync
    port map (
      clk_i     => fs_clk,
      rst_n_a_i => '1',
      d_i       => csr_regout.trig_pol_ext,
      q_o       => ext_trig_pol);

  cmp_time_trig_en_sync : gc_sync
    port map (
      clk_i     => fs_clk,
      rst_n_a_i => '1',
      d_i       => csr_regout.trig_en_time,
      q_o       => time_trig_en);

  cmp_downsample_sync : gc_sync_word_wr
    generic map (
      g_AUTO_WR => TRUE,
      g_WIDTH   => 32)
    port map (
      clk_in_i    => sys_clk_i,
      rst_in_n_i  => '1',
      clk_out_i   => fs_clk,
      rst_out_n_i => '1',
      data_i      => csr_regout.downsample,
      data_o      => downsample_factor);

  cmp_ch_sta_sync : gc_sync_word_wr
    generic map (
      g_AUTO_WR => TRUE,
      g_WIDTH   => 64)
    port map (
      clk_in_i    => fs_clk,
      rst_in_n_i  => '1',
      clk_out_i   => sys_clk_i,
      rst_out_n_i => '1',
      data_i      => serdes_out_data,
      data_o      => serdes_out_data_synced);

  cmp_ext_trig_delay_sync : gc_sync_word_wr
    generic map (
      g_AUTO_WR => TRUE,
      g_WIDTH   => 32)
    port map (
      clk_in_i    => sys_clk_i,
      rst_in_n_i  => '1',
      clk_out_i   => fs_clk,
      rst_out_n_i => '1',
      data_i      => csr_regout.ext_trig_dly,
      data_o      => ext_trig_delay);

  gen_ch_reg_sync : for I in 1 to 4 generate

    cmp_int_trig_en_sync : gc_sync
      port map (
        clk_i     => fs_clk,
        rst_n_a_i => '1',
        d_i       => int_trig_en_in(I),
        q_o       => int_trig_en(I));

    cmp_int_trig_pol_sync : gc_sync
      port map (
        clk_i     => fs_clk,
        rst_n_a_i => '1',
        d_i       => int_trig_pol_in(I),
        q_o       => int_trig_pol(I));

    cmp_ch_trig_thres_sync : gc_sync_word_wr
      generic map (
        g_AUTO_WR => TRUE,
        g_WIDTH   => 32)
      port map (
        clk_in_i             => sys_clk_i,
        rst_in_n_i           => '1',
        clk_out_i            => fs_clk,
        rst_out_n_i          => '1',
        data_i(15 downto 0)  => int_trig_thres_in(I),
        data_i(31 downto 16) => int_trig_thres_hyst_in(I),
        data_o(15 downto 0)  => int_trig_thres(I),
        data_o(31 downto 16) => int_trig_thres_hyst(I));

    cmp_ch_sat_sync : gc_sync_word_wr
      generic map (
        g_AUTO_WR => TRUE,
        g_WIDTH   => 15)
      port map (
        clk_in_i    => sys_clk_i,
        rst_in_n_i  => '1',
        clk_out_i   => fs_clk,
        rst_out_n_i => '1',
        data_i      => sat_val_in(15*I-1 downto 15*(I-1)),
        data_o      => sat_val(15*I-1 downto 15*(I-1)));

    cmp_ch_trig_delay_sync : gc_sync_word_wr
      generic map (
        g_AUTO_WR => TRUE,
        g_WIDTH   => 32)
      port map (
        clk_in_i    => sys_clk_i,
        rst_in_n_i  => '1',
        clk_out_i   => fs_clk,
        rst_out_n_i => '1',
        data_i      => int_trig_delay_in(I),
        data_o      => int_trig_delay(I));
  end generate gen_ch_reg_sync;

  cmp_ch_calib_sync : gc_sync_word_wr
    generic map (
      g_AUTO_WR => FALSE,
      g_WIDTH   => 128)
    port map (
      clk_in_i    => sys_clk_i,
      rst_in_n_i  => '1',
      clk_out_i   => fs_clk,
      rst_out_n_i => '1',
      data_i      => sync_calib_in,
      wr_i        => sync_calib_apply,
      busy_o      => sync_calib_busy,
      data_o      => sync_calib_out);

  offset_calibr <= sync_calib_out(127 downto 64);
  gain_calibr   <= sync_calib_out(63 downto 0);

  cmp_sw_trig_sync : gc_pulse_synchronizer2
    port map (
      clk_in_i    => sys_clk_i,
      rst_in_n_i  => '1',
      clk_out_i   => fs_clk,
      rst_out_n_i => '1',
      d_ack_p_o   => sw_trig_sync_ack,
      d_p_i       => sw_trig_in,
      q_p_o       => sw_trig);

  p_sw_trig_gen : process (sys_clk_i) is
  begin
    if rising_edge(sys_clk_i) then
      if csr_regout.sw_trig_wr = '1' then
        sw_trig_in <= '1';
      elsif sw_trig_in = '1' and sw_trig_sync_ack = '1' then
        sw_trig_in <= '0';
      end if;
    end if;
  end process p_sw_trig_gen;

  ------------------------------------------------------------------------------
  -- Offset and gain calibration
  ------------------------------------------------------------------------------
  l_offset_gain_calibr : for I in 0 to 3 generate
    cmp_offset_gain_calibr : entity work.offset_gain_s
      port map(
        rst_n_i  => fs_rst_n,
        clk_i    => fs_clk,
        offset_i => offset_calibr((I+1)*16-1 downto I*16),
        gain_i   => gain_calibr((I+1)*16-1 downto I*16),
        sat_i    => sat_val((I+1)*15-1 downto I*15),
        data_i   => data_calibr_in((I+1)*16-1 downto I*16),
        data_o   => data_calibr_out((I+1)*16-1 downto I*16)
        );
  end generate l_offset_gain_calibr;

  data_calibr_in <= serdes_out_data;

  ------------------------------------------------------------------------------
  -- Trigger
  ------------------------------------------------------------------------------

  -- External hardware trigger differential to single-ended buffer
  cmp_ext_trig_buf : IBUFDS
    port map (
      O  => ext_trig_a,
      I  => ext_trigger_p_i,
      IB => ext_trigger_n_i
      );

  -- External hardware trigger synchronization
  cmp_ext_trig_sync : gc_sync
    port map (
      clk_i     => fs_clk,
      rst_n_a_i => '1',
      d_i       => ext_trig_a,
      q_o       => ext_trig_sync);

  cmp_ext_trig_negedge : gc_negedge
    port map (
      clk_i   => fs_clk,
      rst_n_i => '1',
      data_i  => ext_trig_sync,
      pulse_o => ext_trig_n);

  cmp_ext_trig_posedge : gc_posedge
    port map (
      clk_i   => fs_clk,
      rst_n_i => '1',
      data_i  => ext_trig_sync,
      pulse_o => ext_trig_p);

  -- select external trigger pulse polarity
  with ext_trig_pol select
    ext_trig <=
    ext_trig_p when '0',
    ext_trig_n when '1',
    '0'        when others;

  -- Configurable trigger delay, adds ext_trig_delay+1 clock cycles
  -- to the trigger signal
  p_ext_trig_delay_cnt : process(fs_clk)
  begin
    if rising_edge(fs_clk) then
      if fs_rst_n = '0' then
        ext_trig_delay_cnt <= (others => '0');
        ext_trig_delay_bsy <= '0';
      else
        if ext_trig = '1' and ext_trig_delay_bsy = '0' then
          ext_trig_delay_cnt <= unsigned(ext_trig_delay);
          ext_trig_delay_bsy <= '1';
        elsif ext_trig_delay_cnt /= 0 then
          ext_trig_delay_cnt <= ext_trig_delay_cnt - 1;
        else
          -- when counter reaches zero
          ext_trig_delay_bsy <= '0';
        end if;
      end if;
    end if;
  end process p_ext_trig_delay_cnt;

  p_ext_trig_delay : process(fs_clk)
  begin
    if rising_edge(fs_clk) then
      if fs_rst_n = '0' then
        ext_trig_d <= '0';
      else
        if ext_trig_delay = X"00000000" then
          if ext_trig = '1' then
            ext_trig_d <= '1';
          else
            ext_trig_d <= '0';
          end if;
        else
          if ext_trig_delay_cnt = X"00000001" then
            ext_trig_d <= '1';
          else
            ext_trig_d <= '0';
          end if;
        end if;
      end if;
    end if;
  end process p_ext_trig_delay;

  -- Time trigger synchronization (from 125MHz timetag core)
  cmp_time_trig_sync : gc_sync
    port map (
      clk_i     => fs_clk,
      rst_n_a_i => '1',
      d_i       => time_trig_i,
      q_o       => time_trig_sync);

  cmp_time_trig_posedge : gc_posedge
    port map (
      clk_i   => fs_clk,
      rst_n_i => '1',
      data_i  => time_trig_sync,
      pulse_o => time_trig);

  cmp_aux_time_trig_sync : gc_sync
    port map (
      clk_i     => fs_clk,
      rst_n_a_i => '1',
      d_i       => aux_time_trig_i,
      q_o       => aux_time_trig_sync);

  cmp_aux_time_trig_posedge : gc_posedge
    port map (
      clk_i   => fs_clk,
      rst_n_i => '1',
      data_i  => aux_time_trig_sync,
      pulse_o => aux_time_trig);

  -- Internal hardware trigger
  gen_int_trig : for I in 1 to 4 generate
    int_trig_data(I) <= data_calibr_out(16*I-1 downto 16*I-16);

    cmp_gc_comparator: gc_comparator
      generic map (
        g_IN_WIDTH => 16)
      port map (
        clk_i     => fs_clk,
        rst_n_i   => fs_rst_n,
        pol_inv_i => int_trig_pol(I),
        enable_i  => int_trig_en(I),
        inp_i     => int_trig_data(I),
        inn_i     => int_trig_thres(I),
        hys_i     => int_trig_thres_hyst(I),
        out_o     => open,
        out_p_o   => int_trig(I));

    -- Configurable trigger delay, adds int_trig_delay(I)+1 clock cycles
    -- to the trigger signal
    p_int_trig_delay_cnt : process(fs_clk)
    begin
      if rising_edge(fs_clk) then
        if fs_rst_n = '0' then
          int_trig_delay_cnt(I) <= (others => '0');
          int_trig_delay_bsy(I) <= '0';
        else
          if int_trig(I) = '1' and int_trig_delay_bsy(I) = '0' then
            int_trig_delay_cnt(I) <= unsigned(int_trig_delay(I));
            int_trig_delay_bsy(I) <= '1';
          elsif int_trig_delay_cnt(I) /= 0 then
            int_trig_delay_cnt(I) <= int_trig_delay_cnt(I) - 1;
          else
          -- when counter reaches zero
            int_trig_delay_bsy(I) <= '0';
          end if;
        end if;
      end if;
    end process p_int_trig_delay_cnt;

    p_int_trig_delay : process(fs_clk)
    begin
      if rising_edge(fs_clk) then
        if fs_rst_n = '0' then
          int_trig_d(I) <= '0';
        else
          if int_trig_delay(I) = X"00000000" then
            --  No delay: direct assignment.
            int_trig_d(I) <= int_trig(I);
          else
            --  Delay set by the counter.
            if int_trig_delay_cnt(I) = X"00000001" then
              int_trig_d(I) <= '1';
            else
              int_trig_d(I) <= '0';
            end if;
          end if;
        end if;
      end if;
    end process p_int_trig_delay;

  end generate gen_int_trig;

  -- Due to the comparator, configurable trigger delay and trigger align logic,
  -- internal threshold triggers are misaligned with respect to the incoming
  -- data (triggers are late by 3 sampling clock cycles).
  --
  -- We solve this by delaying the sampled data by 3 clock cycles on-chip.
  --
  -- At the same time, all the other triggers (external, time and soft) are
  -- also misaligned with respect to the incoming data (triggers arrive earlier
  -- in these cases) because it takes more time to digitize the analogue signals
  -- serialize them, transmit them, receive them in the FPGA, de-serialize, etc.
  --
  -- We solve this by introducing individual delays to the other triggers. In doing
  -- so, we always add more to account for the 3 clock cycles data delays mentioned
  -- before. Thus:
  -- * EXT  triggers are delayed by g_TRIG_DELAY_EXT + 3 cycles
  -- * TIME triggers are delayed by g_TRIG_DELAY_SW  + 3 cycles
  -- * SOFT triggers are delayed by g_TRIG_DELAY_SW  + 3 cycles

  p_data_shift : process (fs_clk)
  begin
    if rising_edge(fs_clk) then
      data_calibr_out_d1 <= data_calibr_out;
      data_calibr_out_d2 <= data_calibr_out_d1;
      data_calibr_out_d3 <= data_calibr_out_d2;
    end if;
  end process p_data_shift;

  p_trig_shift : process(fs_clk)
  begin
    if rising_edge(fs_clk) then
      if fs_rst_n = '0' then
        sw_trig_fixed_delay       <= (others => '0');
        ext_trig_fixed_delay      <= (others => '0');
        time_trig_fixed_delay     <= (others => '0');
        aux_time_trig_fixed_delay <= (others => '0');
      else
        sw_trig_fixed_delay   <= sw_trig_fixed_delay(sw_trig_fixed_delay'high -1 downto 0) & sw_trig;
        ext_trig_fixed_delay  <= ext_trig_fixed_delay(ext_trig_fixed_delay'high -1 downto 0) & ext_trig_d;
        time_trig_fixed_delay <= time_trig_fixed_delay(time_trig_fixed_delay'high -1 downto 0) & time_trig;
        aux_time_trig_fixed_delay <= aux_time_trig_fixed_delay(aux_time_trig_fixed_delay'high -1 downto 0) & aux_time_trig;
      end if;
    end if;
  end process p_trig_shift;

  trig_src_vector <= sw_trig_fixed_delay(sw_trig_fixed_delay'high) &
                     (ext_trig_fixed_delay(ext_trig_fixed_delay'high) and ext_trig_en) &
                     aux_time_trig_fixed_delay(aux_time_trig_fixed_delay'high) &
                     (time_trig_fixed_delay(time_trig_fixed_delay'high) and time_trig_en) &
                     (int_trig_d(4) and int_trig_en(4)) &
                     (int_trig_d(3) and int_trig_en(3)) &
                     (int_trig_d(2) and int_trig_en(2)) &
                     (int_trig_d(1) and int_trig_en(1));

  -- Trigger sources ORing
  trig <= f_reduce_or (trig_src_vector);

  ------------------------------------------------------------------------------
  -- Down-sampling and trigger alignment
  --    When down-sampling is enabled, if the trigger occurs between two
  --    samples it will be realigned to the next sample
  ------------------------------------------------------------------------------
  p_downsample_cnt : process (fs_clk)
  begin
    if rising_edge(fs_clk) then
      if fs_rst_n = '0' then
        downsample_cnt <= to_unsigned(1, downsample_cnt'length);
        downsample_en  <= '0';
      else
        if downsample_cnt = to_unsigned(0, downsample_cnt'length) then
          if downsample_factor /= X"00000000" then
            downsample_cnt <= unsigned(downsample_factor) - 1;
          end if;
          downsample_en <= '1';
        else
          downsample_cnt <= downsample_cnt - 1;
          downsample_en  <= '0';
        end if;
      end if;
    end if;
  end process p_downsample_cnt;

  p_trig_align : process (fs_clk)
  begin
    if rising_edge(fs_clk) then
      if fs_rst_n = '0' then
        trig_align <= (others => '0');
      else
        if trig = '1' then
          trig_align <= trig_src_vector & trig;
        elsif downsample_en = '1' then
          trig_align <= (others => '0');
        end if;
      end if;
    end if;
  end process p_trig_align;

  ------------------------------------------------------------------------------
  -- Synchronisation FIFO to system clock domain
  ------------------------------------------------------------------------------

  cmp_adc_sync_fifo : generic_async_fifo_dual_rst
    generic map (
      g_DATA_WIDTH => 73,
      g_SIZE       => 16,
      g_SHOW_AHEAD => TRUE)
     port map(
      rst_wr_n_i        => fs_rst_n,
      clk_wr_i          => fs_clk,
      d_i               => sync_fifo_din,
      we_i              => sync_fifo_wr,
      wr_full_o         => sync_fifo_full,
      rst_rd_n_i        => sys_rst_n_i,
      clk_rd_i          => sys_clk_i,
      q_o               => sync_fifo_dout,
      rd_i              => sync_fifo_rd,
      rd_empty_o        => sync_fifo_empty);

  -- Data to FIFO
  --     72 : sw trigger
  --     71 : ext trigger
  --     70 : aux time trigger
  --     69 : time trigger
  --     68 : int4 trigger
  --     67 : int3 trigger
  --     66 : int2 trigger
  --     65 : int1 trigger
  --     64 : trigger pulse signal
  -- 63..00 : sample data
  sync_fifo_din(72 downto 64) <= trig_align;
  sync_fifo_din(63 downto 0)  <= data_calibr_out_d3;

  -- FIFO control
  sync_fifo_wr    <= downsample_en and serdes_synced and (not sync_fifo_full);
  sync_fifo_rd    <= not sync_fifo_empty;
  sync_fifo_valid <= not sync_fifo_empty;

  --============================================================================
  -- System clock domain
  --============================================================================

  ------------------------------------------------------------------------------
  -- Shots counter
  ------------------------------------------------------------------------------
  p_shots_cnt : process (sys_clk_i)
  begin
    if rising_edge(sys_clk_i) then
      if sys_rst_n_i = '0' then
        shots_cnt   <= to_unsigned(0, shots_cnt'length);
        single_shot <= '0';
      else
        if acq_start = '1' then
          shots_cnt <= unsigned(shots_value);
        elsif shots_decr = '1' then
          shots_cnt <= shots_cnt - 1;
        end if;
        if shots_value = std_logic_vector(to_unsigned(1, shots_value'length)) then
          single_shot <= '1';
        else
          single_shot <= '0';
        end if;
      end if;
    end if;
  end process p_shots_cnt;

  multishot_buffer_sel <= not std_logic(shots_cnt(0));
  shots_done           <= '1' when shots_cnt = to_unsigned(1, shots_cnt'length) else '0';
  remaining_shots      <= std_logic_vector(shots_cnt);

  ------------------------------------------------------------------------------
  -- Pre-trigger counter
  ------------------------------------------------------------------------------
  p_pre_trig_cnt : process (sys_clk_i)
  begin
    if rising_edge(sys_clk_i) then
      if sys_rst_n_i = '0' then
        pre_trig_cnt <= to_unsigned(1, pre_trig_cnt'length);
      else
        if (acq_start = '1' or pre_trig_done = '1') then
          if unsigned(pre_trig_value) = to_unsigned(0, pre_trig_value'length) then
            pre_trig_cnt <= (others => '0');
          else
            pre_trig_cnt <= unsigned(pre_trig_value) - 1;
          end if;
        elsif (acq_in_pre_trig = '1' and sync_fifo_valid = '1') then
          pre_trig_cnt <= pre_trig_cnt - 1;
        end if;
      end if;
    end if;
  end process p_pre_trig_cnt;


  pre_trig_done <= '1' when (pre_trig_cnt = to_unsigned(0, pre_trig_cnt'length) and
                             sync_fifo_valid = '1' and acq_in_pre_trig = '1') else '0';

  ------------------------------------------------------------------------------
  -- Post-trigger counter
  ------------------------------------------------------------------------------
  p_post_trig_cnt : process (sys_clk_i)
  begin
    if rising_edge(sys_clk_i) then
      if sys_rst_n_i = '0' then
        post_trig_cnt <= to_unsigned(1, post_trig_cnt'length);
      else
        if (acq_start = '1' or post_trig_done = '1') then
          post_trig_cnt <= unsigned(post_trig_value) - 1;
        elsif (acq_in_post_trig = '1' and sync_fifo_valid = '1') then
          post_trig_cnt <= post_trig_cnt - 1;
        end if;
      end if;
    end if;
  end process p_post_trig_cnt;

  post_trig_done <= '1' when (post_trig_cnt = to_unsigned(0, post_trig_cnt'length) and
                              sync_fifo_valid = '1' and acq_in_post_trig = '1') else '0';

  ------------------------------------------------------------------------------
  -- Samples counter
  ------------------------------------------------------------------------------
  p_samples_cnt : process (sys_clk_i)
  begin
    if rising_edge(sys_clk_i) then
      if sys_rst_n_i = '0' then
        samples_cnt <= (others => '0');
      else
        if (acq_start = '1') then
          samples_cnt <= (others => '0');
        elsif ((acq_in_pre_trig = '1' or acq_in_post_trig = '1') and sync_fifo_valid = '1') then
          samples_cnt <= samples_cnt + 1;
        end if;
      end if;
    end if;
  end process p_samples_cnt;

  ------------------------------------------------------------------------------
  -- Aqcuisition FSM
  ------------------------------------------------------------------------------

  -- Event pulses to time-tag
  trigger_p_o   <= acq_trig;
  acq_start_p_o <= acq_start;
  acq_stop_p_o  <= acq_stop;

  -- End of acquisition pulse generation
  p_acq_end : process (sys_clk_i)
  begin
    if rising_edge(sys_clk_i) then
      acq_end_d <= acq_end;
    end if;
  end process p_acq_end;

  acq_end_p_o <= acq_end and not(acq_end_d);

  -- FSM commands
  acq_start <= '1' when ctl_reg_wr = '1' and fsm_cmd = "01" else '0';
  acq_stop  <= '1' when ctl_reg_wr = '1' and fsm_cmd = "10" else '0';
  acq_trig  <= sync_fifo_valid and sync_fifo_dout(64) and acq_in_wait_trig;
  acq_end   <= trig_tag_done and shots_done;

  -- Check acquisition configuration
  --   Post-trigger sample must be > 0
  --   Shot number must be > 0
  --   Number of samples (+time-tag) in multi-shot must be <= multi-shot ram size
  --     Number of samples = pre+1+post+2  (1 for trigger sample, 2 for time-tag)
  -- TODO: because of a -yet to be fully understood- bug, acquisition produces
  -- corrupted samples when number_of_samples is exactly equal to multi_shot ram
  -- size. So for now, number_of_samples should be less than multi_shot ram size.
  p_acq_cfg_ok : process (sys_clk_i)
  begin
    if rising_edge(sys_clk_i) then
      if sys_rst_n_i = '0' then
        acq_config_ok <= '0';
      elsif unsigned(post_trig_value) = to_unsigned(0, post_trig_value'length) then
        acq_config_ok <= '0';
      elsif unsigned(shots_value) = to_unsigned(0, shots_value'length) then
        acq_config_ok <= '0';
      elsif single_shot = '0' and
        unsigned(pre_trig_value) + unsigned(post_trig_value) + 3 >= to_unsigned(g_MULTISHOT_RAM_SIZE, pre_trig_value'length) then
        acq_config_ok <= '0';
      else
        acq_config_ok <= '1';
      end if;
    end if;
  end process p_acq_cfg_ok;

  acq_cfg_ok_o <= acq_config_ok;

  -- FSM transitions
  p_acq_fsm_transitions : process(sys_clk_i)
  begin
    if rising_edge(sys_clk_i) then
      if sys_rst_n_i = '0' then
        acq_fsm_current_state <= IDLE;
      else
        case acq_fsm_current_state is

          when IDLE =>
            if acq_start = '1' and acq_config_ok = '1' then
              if unsigned(pre_trig_value) = to_unsigned(0, pre_trig_value'length) then
                acq_fsm_current_state <= WAIT_TRIG;
              else
                acq_fsm_current_state <= PRE_TRIG;
              end if;
            end if;

          when PRE_TRIG =>
            if acq_stop = '1' then
              acq_fsm_current_state <= IDLE;
            elsif pre_trig_done = '1' then
              acq_fsm_current_state <= WAIT_TRIG;
            end if;

          when WAIT_TRIG =>
            if acq_stop = '1' then
              acq_fsm_current_state <= IDLE;
            elsif acq_trig = '1' then
              acq_fsm_current_state <= POST_TRIG;
            end if;

          when POST_TRIG =>
            if acq_stop = '1' then
              acq_fsm_current_state <= IDLE;
            elsif post_trig_done = '1' then
              acq_fsm_current_state <= TRIG_TAG;
            end if;

          when TRIG_TAG =>
            if acq_stop = '1' then
              acq_fsm_current_state <= IDLE;
            elsif trig_tag_done = '1' then
              acq_fsm_current_state <= DECR_SHOT;
            end if;

          when DECR_SHOT =>
            if acq_stop = '1' then
              acq_fsm_current_state <= IDLE;
            else
              if shots_done = '1' then
                acq_fsm_current_state <= IDLE;
              else
                if unsigned(pre_trig_value) = to_unsigned(0, pre_trig_value'length) then
                  acq_fsm_current_state <= WAIT_TRIG;
                else
                  acq_fsm_current_state <= PRE_TRIG;
                end if;
              end if;
            end if;

          when others =>
            acq_fsm_current_state <= IDLE;

        end case;
      end if;
    end if;
  end process p_acq_fsm_transitions;

  -- FSM outputs
  p_acq_fsm_outputs : process(acq_fsm_current_state)
  begin

    case acq_fsm_current_state is

      when IDLE =>
        shots_decr       <= '0';
        acq_in_pre_trig  <= '0';
        acq_in_wait_trig <= '0';
        acq_in_post_trig <= '0';
        acq_in_trig_tag  <= '0';
        samples_wr_en    <= '0';
        acq_fsm_state    <= "001";

      when PRE_TRIG =>
        shots_decr       <= '0';
        acq_in_pre_trig  <= '1';
        acq_in_wait_trig <= '0';
        acq_in_post_trig <= '0';
        acq_in_trig_tag  <= '0';
        samples_wr_en    <= '1';
        acq_fsm_state    <= "010";

      when WAIT_TRIG =>
        shots_decr       <= '0';
        acq_in_pre_trig  <= '0';
        acq_in_wait_trig <= '1';
        acq_in_post_trig <= '0';
        acq_in_trig_tag  <= '0';
        samples_wr_en    <= '1';
        acq_fsm_state    <= "011";

      when POST_TRIG =>
        shots_decr       <= '0';
        acq_in_pre_trig  <= '0';
        acq_in_wait_trig <= '0';
        acq_in_post_trig <= '1';
        acq_in_trig_tag  <= '0';
        samples_wr_en    <= '1';
        acq_fsm_state    <= "100";

      when TRIG_TAG =>
        shots_decr       <= '0';
        acq_in_pre_trig  <= '0';
        acq_in_wait_trig <= '0';
        acq_in_post_trig <= '0';
        acq_in_trig_tag  <= '1';
        samples_wr_en    <= '0';
        acq_fsm_state    <= "101";

      when DECR_SHOT =>
        shots_decr       <= '1';
        acq_in_pre_trig  <= '0';
        acq_in_wait_trig <= '0';
        acq_in_post_trig <= '0';
        acq_in_trig_tag  <= '0';
        samples_wr_en    <= '0';
        acq_fsm_state    <= "110";

      when others =>
        shots_decr       <= '0';
        acq_in_pre_trig  <= '0';
        acq_in_wait_trig <= '0';
        acq_in_post_trig <= '0';
        acq_in_trig_tag  <= '0';
        samples_wr_en    <= '0';
        acq_fsm_state    <= "111";

    end case;
  end process p_acq_fsm_outputs;

  ------------------------------------------------------------------------------
  -- Inserting trigger information after post_trigger samples
  ------------------------------------------------------------------------------
  p_trig_storage_sys: process (sys_clk_i) is
  begin
    if rising_edge(sys_clk_i) then
      if sys_rst_n_i = '0' or trig_storage_clear = '1' then
        trig_storage <= (others => '0');
      elsif acq_trig = '1' then
        trig_storage <= X"0000" &
                        X"0" & sync_fifo_dout(68 downto 65) &
                        "00" & sync_fifo_dout(70 downto 69) &
                        "00" & sync_fifo_dout(72 downto 71);
      end if;
    end if;
  end process p_trig_storage_sys;

  p_trig_tag_done : process (sys_clk_i)
  begin
    if rising_edge(sys_clk_i) then
      acq_in_trig_tag_d <= acq_in_trig_tag;
    end if;
  end process p_trig_tag_done;

  trig_tag_done <= acq_in_trig_tag and acq_in_trig_tag_d;

  -- We first send "ACCE55" followed by trigger tag seconds, and then the trigger status
  -- followed by trigger tag clock ticks.
  trig_tag_data <= trig_storage & X"0" & trigger_tag_i.coarse when trig_tag_done = '1' else
                   X"ACCE55" & trigger_tag_i.seconds;

  ------------------------------------------------------------------------------
  -- Dual DPRAM buffers for multi-shots acquisition
  ------------------------------------------------------------------------------

  -- DPRAM input address counter
  p_dpram_addra_cnt : process (sys_clk_i)
  begin
    if rising_edge(sys_clk_i) then
      if sys_rst_n_i = '0' or single_shot = '1' then
        dpram_addra_cnt       <= (others => '0');
        dpram_addra_trig      <= (others => '0');
        dpram_addra_post_done <= (others => '0');
      else
        if shots_decr = '1' then
          dpram_addra_cnt <= to_unsigned(0, dpram_addra_cnt'length);
        elsif (samples_wr_en = '1' and sync_fifo_valid = '1') or (acq_in_trig_tag = '1') then
          dpram_addra_cnt <= dpram_addra_cnt + 1;
        end if;
        if acq_trig = '1' then
          dpram_addra_trig <= dpram_addra_cnt;
        end if;
        if post_trig_done = '1' then
          dpram_addra_post_done <= dpram_addra_cnt;
        end if;
      end if;
    end if;
  end process p_dpram_addra_cnt;

  -- DPRAM inputs
  dpram0_addra <= std_logic_vector(dpram_addra_cnt);
  dpram1_addra <= std_logic_vector(dpram_addra_cnt);
  dpram0_dina  <= sync_fifo_dout(63 downto 0)
                 when acq_in_trig_tag = '0' else trig_tag_data;
  dpram1_dina <= sync_fifo_dout(63 downto 0)
                 when acq_in_trig_tag = '0' else trig_tag_data;
  dpram0_wea <= not single_shot and ((samples_wr_en and sync_fifo_valid) or acq_in_trig_tag)
                when multishot_buffer_sel = '0' else '0';
  dpram1_wea <= not single_shot and ((samples_wr_en and sync_fifo_valid) or acq_in_trig_tag)
                when multishot_buffer_sel = '1' else '0';

  -- DPRAMs
  cmp_multishot_dpram0 : generic_dpram
    generic map
    (
      g_DATA_WIDTH               => 64,
      g_SIZE                     => g_MULTISHOT_RAM_SIZE,
      g_WITH_BYTE_ENABLE         => FALSE,
      g_ADDR_CONFLICT_RESOLUTION => "read_first",
      g_DUAL_CLOCK               => FALSE
      -- default values for the rest of the generics are okay
      )
    port map
    (
      rst_n_i => sys_rst_n_i,
      clka_i  => sys_clk_i,
      bwea_i  => (others => '0'),
      wea_i   => dpram0_wea,
      aa_i    => dpram0_addra,
      da_i    => dpram0_dina,
      qa_o    => open,
      clkb_i  => '0',
      bweb_i  => (others => '0'),
      web_i   => '0',
      ab_i    => dpram0_addrb,
      db_i    => (others => '0'),
      qb_o    => dpram0_doutb
      );

  cmp_multishot_dpram1 : generic_dpram
    generic map
    (
      g_DATA_WIDTH               => 64,
      g_SIZE                     => g_MULTISHOT_RAM_SIZE,
      g_WITH_BYTE_ENABLE         => FALSE,
      g_ADDR_CONFLICT_RESOLUTION => "read_first",
      g_DUAL_CLOCK               => FALSE
      -- default values for the rest of the generics are okay
      )
    port map
    (
      rst_n_i => sys_rst_n_i,
      clka_i  => sys_clk_i,
      bwea_i  => (others => '0'),
      wea_i   => dpram1_wea,
      aa_i    => dpram1_addra,
      da_i    => dpram1_dina,
      qa_o    => open,
      clkb_i  => '0',
      bweb_i  => (others => '0'),
      web_i   => '0',
      ab_i    => dpram1_addrb,
      db_i    => (others => '0'),
      qb_o    => dpram1_doutb
      );

  -- DPRAM output address counter
  p_dpram_addrb_cnt : process (sys_clk_i)
  begin
    if rising_edge(sys_clk_i) then
      if sys_rst_n_i = '0' or single_shot = '1' then
        dpram_valid_t <= '0';
        dpram_valid   <= '0';
      else
        if trig_tag_done = '1' then
          dpram_addrb_cnt <= dpram_addra_trig - unsigned(pre_trig_value(c_DPRAM_DEPTH-1 downto 0));
          dpram_valid_t   <= '1';
        elsif (dpram_addrb_cnt = dpram_addra_post_done + 2) then  -- reads 2 extra addresses -> trigger time-tag
          dpram_valid_t <= '0';
        else
          dpram_addrb_cnt <= dpram_addrb_cnt + 1;
        end if;
        dpram_valid <= dpram_valid_t;
      end if;
    end if;
  end process p_dpram_addrb_cnt;

  -- DPRAM output mux
  dpram_dout   <= dpram0_doutb when multishot_buffer_sel = '1' else dpram1_doutb;
  dpram0_addrb <= std_logic_vector(dpram_addrb_cnt);
  dpram1_addrb <= std_logic_vector(dpram_addrb_cnt);

  ------------------------------------------------------------------------------
  -- Flow control FIFO for data to DDR
  ------------------------------------------------------------------------------
  cmp_wb_ddr_fifo : generic_sync_fifo
    generic map (
      g_DATA_WIDTH             => 65,
      g_SIZE                   => 256,
      g_SHOW_AHEAD             => TRUE,
      g_WITH_EMPTY             => TRUE,
      g_WITH_FULL              => TRUE,
      g_WITH_ALMOST_EMPTY      => FALSE,
      g_WITH_ALMOST_FULL       => FALSE,
      g_WITH_COUNT             => FALSE,
      g_ALMOST_EMPTY_THRESHOLD => 0,
      g_ALMOST_FULL_THRESHOLD  => 0
      )
    port map(
      rst_n_i        => sys_rst_n_i,
      clk_i          => sys_clk_i,
      d_i            => wb_ddr_fifo_din,
      we_i           => wb_ddr_fifo_wr,
      q_o            => wb_ddr_fifo_dout,
      rd_i           => wb_ddr_fifo_rd,
      empty_o        => wb_ddr_fifo_empty,
      full_o         => wb_ddr_fifo_full,
      almost_empty_o => open,
      almost_full_o  => open,
      count_o        => open
      );

  -- One clock cycle delay for the FIFO's VALID signal. Since the General Cores
  -- package does not offer the possibility to use the FWFT feature of the FIFOs,
  -- we simulate the valid flag here according to Figure 4-7 in ref. [1].
  -- p_wb_ddr_fifo_valid : process (sys_clk_i) is
  -- begin
  --   if rising_edge(sys_clk_i) then
  --     wb_ddr_fifo_valid <= wb_ddr_fifo_rd;
  --     if (wb_ddr_fifo_empty = '1') then
  --       wb_ddr_fifo_valid <= '0';
  --     end if;
  --   end if;
  -- end process;

  p_wb_ddr_fifo_input : process (sys_clk_i)
  begin
    if rising_edge(sys_clk_i) then
      if sys_rst_n_i = '0' then
        wb_ddr_fifo_din   <= (others => '0');
        wb_ddr_fifo_wr_en <= '0';
      else
        if single_shot = '1' then
          if acq_in_trig_tag = '1' then
            wb_ddr_fifo_din   <= '0' & trig_tag_data;
            wb_ddr_fifo_wr_en <= acq_in_trig_tag;
          else
            wb_ddr_fifo_din   <= acq_trig & sync_fifo_dout(63 downto 0);  -- trigger + data
            wb_ddr_fifo_wr_en <= samples_wr_en and sync_fifo_valid;
          end if;
        else
          wb_ddr_fifo_din   <= '0' & dpram_dout;
          wb_ddr_fifo_wr_en <= dpram_valid;
        end if;
      end if;
    end if;
  end process p_wb_ddr_fifo_input;

  wb_ddr_fifo_wr <= wb_ddr_fifo_wr_en and not(wb_ddr_fifo_full);

  wb_ddr_fifo_rd <= not(wb_ddr_fifo_empty or wb_ddr_master_i.stall);

  ------------------------------------------------------------------------------
  -- Wishbone master (to DDR)
  ------------------------------------------------------------------------------

  -- RAM address counter (64-bit word address)
  p_ram_addr_cnt : process (wb_ddr_clk_i)
  begin
    if rising_edge(wb_ddr_clk_i) then
      if wb_ddr_rst_n_i = '0' then
        ram_addr_cnt <= (others => '0');
      else
        if acq_start = '1' then
          ram_addr_cnt <= (others => '0');
        elsif wb_ddr_fifo_empty = '0' and wb_ddr_master_i.stall = '0' then
          ram_addr_cnt <= ram_addr_cnt + 1;
        end if;
      end if;
    end if;
  end process p_ram_addr_cnt;

  with acq_fsm_state select
    wb_ddr_master_o.cyc <=
    dpram_valid or not wb_ddr_fifo_empty when "001",
    '1'         when others;

  wb_ddr_master_o.stb <= not wb_ddr_fifo_empty;
  -- Convert to 32-bit word addressing for Wishbone
  wb_ddr_master_o.adr <= "00" & std_logic_vector(ram_addr_cnt) & "0";
  wb_ddr_master_o.we  <= '1';
  wb_ddr_master_o.sel <= X"FF";

  with test_data_en select
    wb_ddr_master_o.dat <=
    x"00000000" & "000" & std_logic_vector(ram_addr_cnt) when '1',
    wb_ddr_fifo_dout(63 downto 0)                        when others;

  -- Store trigger DDR address (byte address)
  p_trig_addr : process (wb_ddr_clk_i)
  begin
    if rising_edge(wb_ddr_clk_i) then
      if wb_ddr_rst_n_i = '0' then
        trig_addr <= (others => '0');
      else
        if wb_ddr_fifo_dout(64) = '1' and wb_ddr_fifo_empty = '0' then
          -- Convert to byte addressing
          trig_addr <= std_logic_vector(ram_addr_cnt) & "000";
        end if;
      end if;
    end if;
  end process p_trig_addr;

  -- p_wb_master : process (wb_ddr_clk_i)
  -- begin
  --   if rising_edge(wb_ddr_clk_i) then
  --     if wb_ddr_rst_n_i = '0' then
  --       wb_ddr_master_o.cyc <= '0';
  --       wb_ddr_master_o.we  <= '0';
  --       wb_ddr_master_o.stb <= '0';
  --       wb_ddr_master_o.adr <= (others => '0');
  --       wb_ddr_master_o.dat <= (others => '0');
  --       wb_ddr_stall_t      <= '0';
  --     else
  --       if wb_ddr_fifo_valid = '1' then
  --         wb_ddr_master_o.stb <= '1';
  --         wb_ddr_master_o.adr <= "0000000" & std_logic_vector(ram_addr_cnt);
  --         if test_data_en = '1' then
  --           wb_ddr_master_o.dat <= x"00000000" & "0000000" & std_logic_vector(ram_addr_cnt);
  --         else
  --           wb_ddr_master_o.dat <= wb_ddr_fifo_dout(63 downto 0);
  --         end if;
  --       else
  --         wb_ddr_master_o.stb <= '0';
  --       end if;

  --       if wb_ddr_fifo_valid = '1' then
  --         wb_ddr_master_o.cyc <= '1';
  --         wb_ddr_master_o.we  <= '1';
  --       elsif (wb_ddr_fifo_empty = '1') and (acq_fsm_state = "001") then
  --         wb_ddr_master_o.cyc <= '0';
  --         wb_ddr_master_o.we  <= '0';
  --       end if;

  --       wb_ddr_stall_t <= wb_ddr_master_i.stall;

  --     end if;
  --   end if;
  -- end process p_wb_master;


  -- Trigout

  b_trigout : block
    subtype t_trigout_channels is std_logic_vector(4 downto 0);
    signal trigout_triggers : t_trigout_channels;

    signal trigout_trig : std_logic;

    subtype t_trigout_data_seconds is std_logic_vector(39 downto 0);
    subtype t_trigout_data_coarse is std_logic_vector(67 downto 40);
    subtype t_trigout_data_channels is std_logic_vector(72 downto 68);
    subtype t_trigout_data is std_logic_vector(72 downto 0);

    signal trigout_fifo_dout      : t_trigout_data;
    signal trigout_fifo_din       : t_trigout_data;
    signal trigout_fifo_empty     : std_logic;
    signal trigout_fifo_full      : std_logic;
    signal trigout_fifo_wr        : std_logic;
    signal trigout_fifo_not_empty : std_logic;
    signal trigout_fifo_rd_rq     : std_logic;
    signal trigout_fifo_rd        : std_logic;

  begin
    cmp_aux_trigout : entity work.aux_trigout
      port map (
        rst_n_i        => sys_rst_n_i,
        clk_i          => sys_clk_i,
        wb_i           => wb_trigout_slave_i,
        wb_o           => wb_trigout_slave_o,
        wr_enable_i    => wr_enable_i,
        wr_link_i      => wr_tm_link_up_i,
        wr_valid_i     => wr_tm_time_valid_i,
        ts_present_i   => trigout_fifo_not_empty,
        ts_sec_i       => trigout_fifo_dout(t_trigout_data_seconds'range),
        ch1_mask_i     => trigout_fifo_dout(t_trigout_data_channels'RIGHT + 0),
        ch2_mask_i     => trigout_fifo_dout(t_trigout_data_channels'RIGHT + 1),
        ch3_mask_i     => trigout_fifo_dout(t_trigout_data_channels'RIGHT + 2),
        ch4_mask_i     => trigout_fifo_dout(t_trigout_data_channels'RIGHT + 3),
        ext_mask_i     => trigout_fifo_dout(t_trigout_data_channels'RIGHT + 4),
        cycles_i       => trigout_fifo_dout(t_trigout_data_coarse'range),
        ts_cycles_rd_o => trigout_fifo_rd_rq);

    trigout_fifo_rd <= trigout_fifo_rd_rq and not trigout_fifo_empty;

    trigout_triggers(0) <= trig_storage(8);
    trigout_triggers(1) <= trig_storage(9);
    trigout_triggers(2) <= trig_storage(10);
    trigout_triggers(3) <= trig_storage(11);
    trigout_triggers(4) <= trig_storage(0);

    trigout_trig <= f_reduce_or (trigout_triggers);

    -- Acquisition trigger delayed pulse
    p_acq_end : process (sys_clk_i)
    begin
      if rising_edge(sys_clk_i) then
        acq_trig_d <= acq_trig;
      end if;
    end process p_acq_end;

    trigout_fifo_wr <= trigout_trig and not trigout_fifo_full and acq_trig_d;

    cmp_trigout_fifo : generic_sync_fifo
      generic map (
        g_DATA_WIDTH             => t_trigout_data'length,
        g_SIZE                   => 16,
        g_SHOW_AHEAD             => TRUE,
        g_WITH_EMPTY             => TRUE,
        g_WITH_FULL              => TRUE,
        g_WITH_ALMOST_EMPTY      => FALSE,
        g_WITH_ALMOST_FULL       => FALSE,
        g_WITH_COUNT             => FALSE,
        g_ALMOST_EMPTY_THRESHOLD => 0,
        g_ALMOST_FULL_THRESHOLD  => 0
        )
      port map(
        rst_n_i        => sys_rst_n_i,
        clk_i          => sys_clk_i,
        d_i            => trigout_fifo_din,
        we_i           => trigout_fifo_wr,
        q_o            => trigout_fifo_dout,
        rd_i           => trigout_fifo_rd,
        empty_o        => trigout_fifo_empty,
        full_o         => trigout_fifo_full,
        almost_empty_o => open,
        almost_full_o  => open,
        count_o        => open
        );

    trigout_fifo_not_empty <= not trigout_fifo_empty;

    trigout_fifo_din(t_trigout_data_seconds'range)  <= trigger_tag_i.seconds;
    trigout_fifo_din(t_trigout_data_coarse'range)   <= trigger_tag_i.coarse;
    trigout_fifo_din(t_trigout_data_channels'range) <= trigout_triggers;
  end block b_trigout;
end rtl;
