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
use work.timetag_core_pkg.all;
use work.genram_pkg.all;
use work.gencores_pkg.all;
use work.fmc_adc_100Ms_csr_wbgen2_pkg.all;

entity fmc_adc_100Ms_core is
  generic(
    g_multishot_ram_size : natural := 2048;
    g_carrier_type       : string  := "SPEC"
    );
  port (
    -- Clock, reset
    sys_clk_i   : in std_logic;
    sys_rst_n_i : in std_logic;

    -- CSR wishbone interface
    wb_csr_adr_i : in  std_logic_vector(7 downto 0);
    wb_csr_dat_i : in  std_logic_vector(31 downto 0);
    wb_csr_dat_o : out std_logic_vector(31 downto 0);
    wb_csr_cyc_i : in  std_logic;
    wb_csr_sel_i : in  std_logic_vector(3 downto 0);
    wb_csr_stb_i : in  std_logic;
    wb_csr_we_i  : in  std_logic;
    wb_csr_ack_o : out std_logic;

    -- DDR wishbone interface
    wb_ddr_clk_i   : in  std_logic;
    wb_ddr_adr_o   : out std_logic_vector(31 downto 0);
    wb_ddr_dat_o   : out std_logic_vector(63 downto 0);
    wb_ddr_sel_o   : out std_logic_vector(7 downto 0);
    wb_ddr_stb_o   : out std_logic;
    wb_ddr_we_o    : out std_logic;
    wb_ddr_cyc_o   : out std_logic;
    wb_ddr_ack_i   : in  std_logic;
    wb_ddr_stall_i : in  std_logic;

    -- Events output pulses
    trigger_p_o   : out std_logic;
    acq_start_p_o : out std_logic;
    acq_stop_p_o  : out std_logic;
    acq_end_p_o   : out std_logic;

    -- Trigger time-tag inputs
    trigger_tag_i : in t_timetag;
    time_trig_i   : in std_logic;

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
    gpio_si570_oe_o  : out std_logic              -- Si570 (programmable oscillator) output enable
    );
end fmc_adc_100Ms_core;


architecture rtl of fmc_adc_100Ms_core is


  ------------------------------------------------------------------------------
  -- Components declaration
  ------------------------------------------------------------------------------

  component adc_serdes
    generic
      (
        sys_w : integer := 9;                     -- width of the data for the system
        dev_w : integer := 72                     -- width of the data for the device
        );
    port
      (
        -- Datapath
        DATA_IN_FROM_PINS_P : in  std_logic_vector(sys_w-1 downto 0);
        DATA_IN_FROM_PINS_N : in  std_logic_vector(sys_w-1 downto 0);
        DATA_IN_TO_DEVICE   : out std_logic_vector(dev_w-1 downto 0);
        -- Data control
        BITSLIP             : in  std_logic;
        -- Clock and reset signals
        CLK_IN              : in  std_logic;      -- Fast clock from PLL/MMCM
        CLK_OUT             : out std_logic;
        CLK_DIV_IN          : in  std_logic;      -- Slow clock from PLL/MMCM
        LOCKED_IN           : in  std_logic;
        LOCKED_OUT          : out std_logic;
        CLK_RESET           : in  std_logic;      -- Reset signal for Clock circuit
        IO_RESET            : in  std_logic       -- Reset signal for IO circuit
        );
  end component adc_serdes;

  component fmc_adc_100Ms_csr is
    port (
      rst_n_i    : in  std_logic;
      clk_sys_i  : in  std_logic;
      wb_adr_i   : in  std_logic_vector(7 downto 0);
      wb_dat_i   : in  std_logic_vector(31 downto 0);
      wb_dat_o   : out std_logic_vector(31 downto 0);
      wb_cyc_i   : in  std_logic;
      wb_sel_i   : in  std_logic_vector(3 downto 0);
      wb_stb_i   : in  std_logic;
      wb_we_i    : in  std_logic;
      wb_ack_o   : out std_logic;
      wb_stall_o : out std_logic;
      fs_clk_i   : in  std_logic;
      regs_i     : in  t_fmc_adc_100Ms_csr_in_registers;
      regs_o     : out t_fmc_adc_100Ms_csr_out_registers);
  end component fmc_adc_100Ms_csr;

  component offset_gain_s
    port (
      rst_n_i  : in  std_logic;                      --! Reset (active low)
      clk_i    : in  std_logic;                      --! Clock
      offset_i : in  std_logic_vector(15 downto 0);  --! Signed offset input (two's complement)
      gain_i   : in  std_logic_vector(15 downto 0);  --! Unsigned gain input
      sat_i    : in  std_logic_vector(14 downto 0);  --! Unsigned saturation value input
      data_i   : in  std_logic_vector(15 downto 0);  --! Signed data input (two's complement)
      data_o   : out std_logic_vector(15 downto 0)   --! Signed data output (two's complement)
      );
  end component offset_gain_s;

  ------------------------------------------------------------------------------
  -- Constants declaration
  ------------------------------------------------------------------------------
  constant c_dpram_depth : integer := f_log2_size(g_multishot_ram_size);

  -- Calculate the maximum number of available samples per multishot trigger
  -- Note: we subtract 2 for the timetag, and 1 more because of bug when number
  -- of samples equals the size of the dpram
  constant c_MULTISHOT_SAMPLE_DEPTH : std_logic_vector(31 downto 0) :=
    std_logic_vector(to_unsigned(g_multishot_ram_size - 3, 32));

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
  signal sys_rst  : std_logic;
  signal fs_rst_n : std_logic;

  -- Clocks and PLL
  signal dco_clk       : std_logic;
  signal dco_clk_buf   : std_logic;
  signal clk_fb        : std_logic;
  signal clk_fb_buf    : std_logic;
  signal locked_in     : std_logic;
  signal locked_out    : std_logic;
  signal serdes_clk    : std_logic;
  signal fs_clk        : std_logic;
  signal fs_clk_buf    : std_logic;
  signal fs_freq       : std_logic_vector(31 downto 0);
  signal fs_freq_t     : std_logic_vector(31 downto 0);
  signal fs_freq_valid : std_logic;

  -- SerDes
  signal serdes_in_p         : std_logic_vector(8 downto 0);
  signal serdes_in_n         : std_logic_vector(8 downto 0);
  signal serdes_out_raw      : std_logic_vector(71 downto 0);
  signal serdes_out_data     : std_logic_vector(63 downto 0);
  signal serdes_out_fr       : std_logic_vector(7 downto 0);
  signal serdes_auto_bitslip : std_logic;
  signal serdes_man_bitslip  : std_logic;
  signal serdes_bitslip      : std_logic;
  signal serdes_synced       : std_logic;
  signal bitslip_sreg        : std_logic_vector(7 downto 0);

  -- Trigger
  signal ext_trig_a, ext_trig       : std_logic;
  signal ext_trig_d                 : std_logic;
  signal ext_trig_delay             : std_logic_vector(31 downto 0);
  signal ext_trig_delay_cnt         : unsigned(31 downto 0);
  signal ext_trig_delay_bsy         : std_logic;
  signal ext_trig_en                : std_logic;
  signal ext_trig_fixed_delay       : std_logic_vector(7 downto 0);
  signal ext_trig_p, ext_trig_n     : std_logic;
  signal ext_trig_pol               : std_logic;
  signal int_trig                   : std_logic_vector(1 to 4);
  signal int_trig_d                 : std_logic_vector(1 to 4);
  signal int_trig_data              : t_fmc_adc_vec16_array(1 to 4);
  signal int_trig_delay             : t_fmc_adc_vec32_array(1 to 4);
  signal int_trig_delay_cnt         : t_fmc_adc_uint32_array(1 to 4);
  signal int_trig_delay_bsy         : std_logic_vector(1 to 4);
  signal int_trig_en                : std_logic_vector(1 to 4);
  signal int_trig_pol               : std_logic_vector(1 to 4);
  signal int_trig_thres             : t_fmc_adc_vec16_array(1 to 4);
  signal int_trig_thres_hyst        : t_fmc_adc_vec16_array(1 to 4);
  signal sw_trig                    : std_logic;
  signal sw_trig_en                 : std_logic;
  signal sw_trig_fixed_delay        : std_logic_vector(4 downto 0);
  signal time_trig                  : std_logic;
  signal time_trig_en               : std_logic;
  signal time_trig_fixed_delay      : std_logic_vector(4 downto 0);
  signal trig                       : std_logic;
  signal trig_align                 : std_logic;
  signal trig_fifo_din              : std_logic_vector(32 downto 0);
  signal trig_fifo_dout             : std_logic_vector(32 downto 0);
  signal trig_fifo_empty            : std_logic;
  signal trig_fifo_full             : std_logic;
  signal trig_fifo_rd               : std_logic;
  signal trig_fifo_wr               : std_logic;
  signal trig_storage               : std_logic_vector(31 downto 0);
  signal trig_storage_clear         : std_logic;

  -- Under-sampling
  signal undersample_factor : std_logic_vector(31 downto 0);
  signal undersample_cnt    : unsigned(31 downto 0);
  signal undersample_en     : std_logic;

  -- Sync FIFO (from fs_clk to sys_clk_i)
  signal sync_fifo_din   : std_logic_vector(64 downto 0);
  signal sync_fifo_dout  : std_logic_vector(64 downto 0);
  signal sync_fifo_empty : std_logic;
  signal sync_fifo_full  : std_logic;
  signal sync_fifo_wr    : std_logic;
  signal sync_fifo_rd    : std_logic;
  signal sync_fifo_valid : std_logic;

  -- Gain/offset calibration and saturation value
  signal gain_calibr        : std_logic_vector(63 downto 0);
  signal offset_calibr      : std_logic_vector(63 downto 0);
  signal data_calibr_in     : std_logic_vector(63 downto 0);
  signal data_calibr_out    : std_logic_vector(63 downto 0);
  signal data_calibr_out_d1 : std_logic_vector(63 downto 0);
  signal data_calibr_out_d2 : std_logic_vector(63 downto 0);
  signal data_calibr_out_d3 : std_logic_vector(63 downto 0);
  signal sat_val            : std_logic_vector(59 downto 0);

  -- Acquisition FSM
  signal acq_fsm_current_state : t_acq_fsm_state;
  signal acq_fsm_state         : std_logic_vector(2 downto 0);
  signal fsm_cmd               : std_logic_vector(1 downto 0);
  signal fsm_cmd_wr            : std_logic;
  signal acq_start             : std_logic;
  signal acq_stop              : std_logic;
  signal acq_trig              : std_logic;
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
  signal dpram_addra_cnt       : unsigned(c_dpram_depth-1 downto 0);
  signal dpram_addra_trig      : unsigned(c_dpram_depth-1 downto 0);
  signal dpram_addra_post_done : unsigned(c_dpram_depth-1 downto 0);
  signal dpram_addrb_cnt       : unsigned(c_dpram_depth-1 downto 0);
  signal dpram_dout            : std_logic_vector(63 downto 0);
  signal dpram_valid           : std_logic;
  signal dpram_valid_t         : std_logic;

  signal dpram0_dina  : std_logic_vector(63 downto 0);
  signal dpram0_addra : std_logic_vector(c_dpram_depth-1 downto 0);
  signal dpram0_wea   : std_logic;
  signal dpram0_addrb : std_logic_vector(c_dpram_depth-1 downto 0);
  signal dpram0_doutb : std_logic_vector(63 downto 0);

  signal dpram1_dina  : std_logic_vector(63 downto 0);
  signal dpram1_addra : std_logic_vector(c_dpram_depth-1 downto 0);
  signal dpram1_wea   : std_logic;
  signal dpram1_addrb : std_logic_vector(c_dpram_depth-1 downto 0);
  signal dpram1_doutb : std_logic_vector(63 downto 0);

  -- Wishbone to DDR flowcontrol FIFO
  signal wb_ddr_fifo_din   : std_logic_vector(64 downto 0);
  signal wb_ddr_fifo_dout  : std_logic_vector(64 downto 0);
  signal wb_ddr_fifo_empty : std_logic;
  signal wb_ddr_fifo_full  : std_logic;
  signal wb_ddr_fifo_wr    : std_logic;
  signal wb_ddr_fifo_rd    : std_logic;
  signal wb_ddr_fifo_valid : std_logic;
  signal wb_ddr_fifo_dreq  : std_logic;
  signal wb_ddr_fifo_wr_en : std_logic;

  -- RAM address counter
  signal ram_addr_cnt : unsigned(24 downto 0);
  signal test_data_en : std_logic;
  signal trig_addr    : std_logic_vector(31 downto 0);
  signal mem_ovr      : std_logic;

  -- Wishbone interface to DDR
  signal wb_ddr_stall_t : std_logic;

  -- IO from CSR registers
  signal csr_regin  : t_fmc_adc_100Ms_csr_in_registers;
  signal csr_regout : t_fmc_adc_100Ms_csr_out_registers;

  -- LEDs
  signal trig_led     : std_logic;
  signal trig_led_man : std_logic;
  signal acq_led      : std_logic;
  signal acq_led_man  : std_logic;

begin


  ------------------------------------------------------------------------------
  -- LEDs
  ------------------------------------------------------------------------------
  cmp_acq_led: gc_extend_pulse
    generic map (
      g_width => 12500000)
    port map (
      clk_i      => sys_clk_i,
      rst_n_i    => sys_rst_n_i,
      pulse_i    => samples_wr_en,
      extended_o => acq_led);

  gpio_led_acq_o <= acq_led or acq_led_man;

  cmp_trig_led: gc_extend_pulse
    generic map (
      g_width => 12500000)
    port map (
      clk_i      => sys_clk_i,
      rst_n_i    => sys_rst_n_i,
      pulse_i    => acq_trig,
      extended_o => trig_led);

  gpio_led_trig_o <= trig_led or trig_led_man;

  ------------------------------------------------------------------------------
  -- Resets
  ------------------------------------------------------------------------------
  sys_rst  <= not(sys_rst_n_i);
  fs_rst_n <= sys_rst_n_i and locked_out;

  ------------------------------------------------------------------------------
  -- ADC data clock buffer
  ------------------------------------------------------------------------------
  cmp_dco_buf : IBUFDS
    generic map (
      DIFF_TERM  => TRUE,                         -- Differential termination
      IOSTANDARD => "LVDS_25")
    port map (
      I  => adc_dco_p_i,
      IB => adc_dco_n_i,
      O  => dco_clk_buf
      );

  cmp_dco_bufio : BUFIO2
    generic map (
      DIVIDE        => 1,
      DIVIDE_BYPASS => TRUE,
      I_INVERT      => FALSE,
      USE_DOUBLER   => FALSE)
    port map (
      I            => dco_clk_buf,
      IOCLK        => open,
      DIVCLK       => dco_clk,
      SERDESSTROBE => open
      );

  ------------------------------------------------------------------------------
  -- Clock PLL for SerDes
  -- LTC2174-14 must be configured in 16-bit serialization
  --    dco_clk = 4*fs_clk = 400MHz
  -- WARNING : The PLL expects a 400MHz input frequency, therefore the sampling
  --           frequency has to be 100MHz and can't be change dynamically.
  ------------------------------------------------------------------------------
  cmp_serdes_clk_pll : PLL_BASE
    generic map (
      BANDWIDTH          => "OPTIMIZED",
      CLK_FEEDBACK       => "CLKOUT0",
      COMPENSATION       => "SYSTEM_SYNCHRONOUS",
      DIVCLK_DIVIDE      => 1,
      CLKFBOUT_MULT      => 2,
      CLKFBOUT_PHASE     => 0.000,
      CLKOUT0_DIVIDE     => 1,
      CLKOUT0_PHASE      => 0.000,
      CLKOUT0_DUTY_CYCLE => 0.500,
      CLKOUT1_DIVIDE     => 8,
      CLKOUT1_PHASE      => 0.000,
      CLKOUT1_DUTY_CYCLE => 0.500,
      CLKIN_PERIOD       => 2.5,
      REF_JITTER         => 0.010)
    port map (
      -- Output clocks
      CLKFBOUT => open,
      CLKOUT0  => serdes_clk,
      CLKOUT1  => fs_clk_buf,
      CLKOUT2  => open,
      CLKOUT3  => open,
      CLKOUT4  => open,
      CLKOUT5  => open,
      -- Status and control signals
      LOCKED   => locked_in,
      RST      => sys_rst,
      -- Input clock control
      CLKFBIN  => clk_fb,
      CLKIN    => dco_clk);

  cmp_fs_clk_buf : BUFG
    port map (
      O => fs_clk,
      I => fs_clk_buf
      );

  cmp_fb_clk_bufio : BUFIO2FB
    generic map (
      DIVIDE_BYPASS => TRUE)
    port map (
      I => clk_fb_buf,
      O => clk_fb
      );

  -- Sampinling clock frequency meter
  cmp_fs_freq : gc_frequency_meter
    generic map(
      g_with_internal_timebase => TRUE,
      g_clk_sys_freq           => 125000000,
      g_counter_bits           => 32
      )
    port map(
      clk_sys_i    => sys_clk_i,
      clk_in_i     => fs_clk,
      rst_n_i      => sys_rst_n_i,
      pps_p1_i     => '0',
      freq_o       => fs_freq_t,
      freq_valid_o => fs_freq_valid
      );

  p_fs_freq : process (fs_clk, fs_rst_n)
  begin
    if fs_rst_n = '0' then
      fs_freq <= (others => '0');
    elsif rising_edge(fs_clk) then
      if fs_freq_valid = '1' then
        fs_freq <= fs_freq_t;
      end if;
    end if;
  end process p_fs_freq;

  --gen_fb_clk_check : if (g_carrier_type /= "SPEC" and
  --                      g_carrier_type /= "SVEC") generate
  --  assert false report "[fmc_adc_100Ms_core] Selected carrier type not supported. Must be SPEC or SVEC." severity failure;
  --end generate gen_fb_clk_check;

  --gen_fb_clk_spec : if g_carrier_type = "SPEC" generate
  --  cmp_fb_clk_buf : BUFG
  --    port map (
  --      O => clk_fb,
  --      I => clk_fb_buf
  --      );
  --end generate gen_fb_clk_spec;

  --gen_fb_clk_svec : if g_carrier_type = "SVEC" generate
  --  clk_fb <= clk_fb_buf;
  --end generate gen_fb_clk_svec;

  ------------------------------------------------------------------------------
  -- ADC data and frame SerDes
  ------------------------------------------------------------------------------
  cmp_adc_serdes : adc_serdes
    port map(
      DATA_IN_FROM_PINS_P => serdes_in_p,
      DATA_IN_FROM_PINS_N => serdes_in_n,
      DATA_IN_TO_DEVICE   => serdes_out_raw,
      BITSLIP             => serdes_bitslip,
      CLK_IN              => serdes_clk,
      CLK_OUT             => clk_fb_buf,
      CLK_DIV_IN          => fs_clk,
      LOCKED_IN           => locked_in,
      LOCKED_OUT          => locked_out,
      CLK_RESET           => '0',                 -- unused
      IO_RESET            => sys_rst
      );


  --============================================================================
  -- Sampling clock domain
  --============================================================================

  -- serdes inputs forming
  serdes_in_p <= adc_fr_p_i
                 & adc_outa_p_i(3) & adc_outb_p_i(3)
                 & adc_outa_p_i(2) & adc_outb_p_i(2)
                 & adc_outa_p_i(1) & adc_outb_p_i(1)
                 & adc_outa_p_i(0) & adc_outb_p_i(0);
  serdes_in_n <= adc_fr_n_i
                 & adc_outa_n_i(3) & adc_outb_n_i(3)
                 & adc_outa_n_i(2) & adc_outb_n_i(2)
                 & adc_outa_n_i(1) & adc_outb_n_i(1)
                 & adc_outa_n_i(0) & adc_outb_n_i(0);

  -- serdes outputs re-ordering (time slices -> channel)
  --    out_raw :(71:63)(62:54)(53:45)(44:36)(35:27)(26:18)(17:9)(8:0)
  --                |      |      |      |      |      |      |    |
  --                V      V      V      V      V      V      V    V
  --              CH1D12 CH1D10 CH1D8  CH1D6  CH1D4  CH1D2  CH1D0  0   = CH1_B
  --              CH1D13 CH1D11 CH1D9  CH1D7  CH1D5  CH1D3  CH1D1  0   = CH1_A
  --              CH2D12 CH2D10 CH2D8  CH2D6  CH2D4  CH2D2  CH2D0  0   = CH2_B
  --              CH2D13 CH2D11 CH2D9  CH2D7  CH2D5  CH2D3  CH2D1  0   = CH2_A
  --              CH3D12 CH3D10 CH3D8  CH3D6  CH3D4  CH3D2  CH3D0  0   = CH3_B
  --              CH3D13 CH3D11 CH3D9  CH3D7  CH3D5  CH3D3  CH3D1  0   = CH3_A
  --              CH4D12 CH4D10 CH4D8  CH4D6  CH4D4  CH4D2  CH4D0  0   = CH4_B
  --              CH4D13 CH4D11 CH4D9  CH4D7  CH4D5  CH4D3  CH4D1  0   = CH4_A
  --              FR7    FR6    FR5    FR4    FR3    FR2    FR1    FR0 = FR
  --
  --    out_data(15:0)  = CH1
  --    out_data(31:16) = CH2
  --    out_data(47:32) = CH3
  --    out_data(63:48) = CH4
  --    Note: The two LSBs of each channel are always '0' => 14-bit ADC
  gen_serdes_dout_reorder : for I in 0 to 7 generate
    serdes_out_data(0*16 + 2*i)   <= serdes_out_raw(0 + i*9);  -- CH1 even bits
    serdes_out_data(0*16 + 2*i+1) <= serdes_out_raw(1 + i*9);  -- CH1 odd bits
    serdes_out_data(1*16 + 2*i)   <= serdes_out_raw(2 + i*9);  -- CH2 even bits
    serdes_out_data(1*16 + 2*i+1) <= serdes_out_raw(3 + i*9);  -- CH2 odd bits
    serdes_out_data(2*16 + 2*i)   <= serdes_out_raw(4 + i*9);  -- CH3 even bits
    serdes_out_data(2*16 + 2*i+1) <= serdes_out_raw(5 + i*9);  -- CH3 odd bits
    serdes_out_data(3*16 + 2*i)   <= serdes_out_raw(6 + i*9);  -- CH4 even bits
    serdes_out_data(3*16 + 2*i+1) <= serdes_out_raw(7 + i*9);  -- CH4 odd bits
    serdes_out_fr(i)              <= serdes_out_raw(8 + i*9);  -- FR
  end generate gen_serdes_dout_reorder;


  -- serdes bitslip generation
  p_auto_bitslip : process (fs_clk, sys_rst_n_i)
  begin
    if sys_rst_n_i = '0' then
      bitslip_sreg        <= std_logic_vector(to_unsigned(1, bitslip_sreg'length));
      serdes_auto_bitslip <= '0';
      serdes_synced       <= '0';
    elsif rising_edge(fs_clk) then

      -- Shift register to generate bitslip enable (serdes_clk/8)
      bitslip_sreg <= bitslip_sreg(0) & bitslip_sreg(bitslip_sreg'length-1 downto 1);

      -- Generate bitslip and synced signal
      if(bitslip_sreg(bitslip_sreg'LEFT) = '1') then
        if(serdes_out_fr /= "00001111") then  -- use fr_n pattern (fr_p and fr_n are swapped on the adc mezzanine)
          serdes_auto_bitslip <= '1';
          serdes_synced       <= '0';
        else
          serdes_auto_bitslip <= '0';
          serdes_synced       <= '1';
        end if;
      else
        serdes_auto_bitslip <= '0';
      end if;

    end if;
  end process;

  serdes_bitslip <= serdes_auto_bitslip or serdes_man_bitslip;

  ------------------------------------------------------------------------------
  -- ADC core control and status registers (CSR)
  ------------------------------------------------------------------------------
  cmp_fmc_adc_100Ms_csr : fmc_adc_100Ms_csr
    port map(
      rst_n_i    => sys_rst_n_i,
      clk_sys_i  => sys_clk_i,
      wb_adr_i   => wb_csr_adr_i,
      wb_dat_i   => wb_csr_dat_i,
      wb_dat_o   => wb_csr_dat_o,
      wb_cyc_i   => wb_csr_cyc_i,
      wb_sel_i   => wb_csr_sel_i,
      wb_stb_i   => wb_csr_stb_i,
      wb_we_i    => wb_csr_we_i,
      wb_ack_o   => wb_csr_ack_o,
      wb_stall_o => open,
      fs_clk_i   => fs_clk,
      regs_i     => csr_regin,
      regs_o     => csr_regout);

  csr_regin.sta_fsm_i           <= acq_fsm_state;
  csr_regin.sta_serdes_pll_i    <= locked_out;
  csr_regin.sta_serdes_synced_i <= serdes_synced;
  csr_regin.sta_acq_cfg_i       <= acq_config_ok;
  csr_regin.trig_stat_ext_i     <= trig_storage(0);
  csr_regin.trig_stat_sw_i      <= trig_storage(1);
  csr_regin.trig_stat_time_i    <= trig_storage(4);
  csr_regin.trig_stat_ch1_i     <= trig_storage(8);
  csr_regin.trig_stat_ch2_i     <= trig_storage(9);
  csr_regin.trig_stat_ch3_i     <= trig_storage(10);
  csr_regin.trig_stat_ch4_i     <= trig_storage(11);
  csr_regin.shots_cnt_val_i     <= remaining_shots;
  csr_regin.trig_pos_i          <= trig_addr;
  csr_regin.fs_freq_i           <= fs_freq;
  csr_regin.samples_cnt_i       <= std_logic_vector(samples_cnt);
  csr_regin.ch1_sta_val_i       <= serdes_out_data(15 downto 0);
  csr_regin.ch2_sta_val_i       <= serdes_out_data(31 downto 16);
  csr_regin.ch3_sta_val_i       <= serdes_out_data(47 downto 32);
  csr_regin.ch4_sta_val_i       <= serdes_out_data(63 downto 48);
  csr_regin.multi_depth_i       <= c_MULTISHOT_SAMPLE_DEPTH;

  fsm_cmd                <= csr_regout.ctl_fsm_cmd_o;
  fsm_cmd_wr             <= csr_regout.ctl_fsm_cmd_wr_o;
  gpio_si570_oe_o        <= csr_regout.ctl_fmc_clk_oe_o;
  gpio_dac_clr_n_o       <= csr_regout.ctl_offset_dac_clr_n_o;
  serdes_man_bitslip     <= csr_regout.ctl_man_bitslip_o;
  test_data_en           <= csr_regout.ctl_test_data_en_o;
  trig_led_man           <= csr_regout.ctl_trig_led_o;
  acq_led_man            <= csr_regout.ctl_acq_led_o;
  trig_storage_clear     <= csr_regout.ctl_clear_trig_stat_o;
  ext_trig_delay         <= csr_regout.ext_trig_dly_o;
  ext_trig_en            <= csr_regout.trig_en_ext_o;
  ext_trig_pol           <= csr_regout.trig_pol_ext_o;
  int_trig_delay(1)      <= csr_regout.ch1_trig_dly_o;
  int_trig_delay(2)      <= csr_regout.ch2_trig_dly_o;
  int_trig_delay(3)      <= csr_regout.ch3_trig_dly_o;
  int_trig_delay(4)      <= csr_regout.ch4_trig_dly_o;
  int_trig_en(1)         <= csr_regout.trig_en_ch1_o;
  int_trig_en(2)         <= csr_regout.trig_en_ch2_o;
  int_trig_en(3)         <= csr_regout.trig_en_ch3_o;
  int_trig_en(4)         <= csr_regout.trig_en_ch4_o;
  int_trig_pol(1)        <= csr_regout.trig_pol_ch1_o;
  int_trig_pol(2)        <= csr_regout.trig_pol_ch2_o;
  int_trig_pol(3)        <= csr_regout.trig_pol_ch3_o;
  int_trig_pol(4)        <= csr_regout.trig_pol_ch4_o;
  int_trig_thres(1)      <= csr_regout.ch1_trig_thres_val_o;
  int_trig_thres(2)      <= csr_regout.ch2_trig_thres_val_o;
  int_trig_thres(3)      <= csr_regout.ch3_trig_thres_val_o;
  int_trig_thres(4)      <= csr_regout.ch4_trig_thres_val_o;
  int_trig_thres_hyst(1) <= csr_regout.ch1_trig_thres_hyst_o;
  int_trig_thres_hyst(2) <= csr_regout.ch2_trig_thres_hyst_o;
  int_trig_thres_hyst(3) <= csr_regout.ch3_trig_thres_hyst_o;
  int_trig_thres_hyst(4) <= csr_regout.ch4_trig_thres_hyst_o;
  sw_trig                <= csr_regout.sw_trig_wr_o;
  sw_trig_en             <= csr_regout.trig_en_sw_o;
  time_trig_en           <= csr_regout.trig_en_time_o;
  shots_value            <= csr_regout.shots_nb_o;
  undersample_factor     <= csr_regout.sr_undersample_o;
  pre_trig_value         <= csr_regout.pre_samples_o;
  post_trig_value        <= csr_regout.post_samples_o;
  gpio_ssr_ch1_o         <= csr_regout.ch1_ctl_ssr_o;
  gpio_ssr_ch2_o         <= csr_regout.ch2_ctl_ssr_o;
  gpio_ssr_ch3_o         <= csr_regout.ch3_ctl_ssr_o;
  gpio_ssr_ch4_o         <= csr_regout.ch4_ctl_ssr_o;

  gain_calibr <= csr_regout.ch4_gain_val_o & csr_regout.ch3_gain_val_o &
                 csr_regout.ch2_gain_val_o & csr_regout.ch1_gain_val_o;

  offset_calibr <= csr_regout.ch4_offset_val_o & csr_regout.ch3_offset_val_o &
                   csr_regout.ch2_offset_val_o & csr_regout.ch1_offset_val_o;

  sat_val <= csr_regout.ch4_sat_val_o & csr_regout.ch3_sat_val_o &
             csr_regout.ch2_sat_val_o & csr_regout.ch1_sat_val_o;

  ------------------------------------------------------------------------------
  -- Offset and gain calibration
  ------------------------------------------------------------------------------
  l_offset_gain_calibr : for I in 0 to 3 generate
    cmp_offset_gain_calibr : offset_gain_s
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
  cmp_ext_trig_sync : gc_sync_ffs
    port map (
      clk_i    => fs_clk,
      rst_n_i  => fs_rst_n,
      data_i   => ext_trig_a,
      synced_o => open,
      npulse_o => ext_trig_n,
      ppulse_o => ext_trig_p);

  -- select external trigger pulse polarity
  with ext_trig_pol select
    ext_trig <=
    ext_trig_p when '0',
    ext_trig_n when '1',
    '0'        when others;

  -- Configurable trigger delay, adds ext_trig_delay+1 clock cycles
  -- to the trigger signal
  p_ext_trig_delay_cnt : process(fs_clk, fs_rst_n)
  begin
    if fs_rst_n = '0' then
      ext_trig_delay_cnt <= (others => '0');
      ext_trig_delay_bsy <= '0';
    elsif rising_edge(fs_clk) then
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
  end process p_ext_trig_delay_cnt;

  p_ext_trig_delay : process(fs_clk, fs_rst_n)
  begin
    if fs_rst_n = '0' then
      ext_trig_d <= '0';
    elsif rising_edge(fs_clk) then
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
  end process p_ext_trig_delay;

  -- Time trigger synchronization (from 125MHz timetag core)
  cmp_time_trig_sync : gc_sync_ffs
    port map (
      clk_i    => fs_clk,
      rst_n_i  => fs_rst_n,
      data_i   => time_trig_i,
      synced_o => open,
      npulse_o => open,
      ppulse_o => time_trig);

  -- Internal hardware trigger
  g_int_trig : for I in 1 to 4 generate
    int_trig_data(I) <= data_calibr_out(16*I-1 downto 16*I-16);

    cmp_gc_comparator: entity work.gc_comparator
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
    p_int_trig_delay_cnt : process(fs_clk, fs_rst_n)
    begin
      if fs_rst_n = '0' then
        int_trig_delay_cnt(I) <= (others => '0');
        int_trig_delay_bsy(I) <= '0';
      elsif rising_edge(fs_clk) then
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
    end process p_int_trig_delay_cnt;

    p_int_trig_delay : process(fs_clk, fs_rst_n)
    begin
      if fs_rst_n = '0' then
        int_trig_d(I) <= '0';
      elsif rising_edge(fs_clk) then
        if int_trig_delay(I) = X"00000000" then
          if int_trig(I) = '1' then
            int_trig_d(I) <= '1';
          else
            int_trig_d(I) <= '0';
          end if;
        else
          if int_trig_delay_cnt(I) = X"00000001" then
            int_trig_d(I) <= '1';
          else
            int_trig_d(I) <= '0';
          end if;
        end if;
      end if;
    end process p_int_trig_delay;

  end generate g_int_trig;

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
  -- * EXT triggers are delayed by 8 (5+3) cycles
  -- * TIME triggers are delayed by 5 (2+3) cycles TODO: confirm
  -- * SOFT triggers are delayed by 5 (2+3) cycles TODO: confirm

  p_data_shift : process (fs_clk)
  begin
    if rising_edge(fs_clk) then
      data_calibr_out_d1 <= data_calibr_out;
      data_calibr_out_d2 <= data_calibr_out_d1;
      data_calibr_out_d3 <= data_calibr_out_d2;
    end if;
  end process p_data_shift;

  p_trig_shift : process(fs_clk, fs_rst_n)
  begin
    if fs_rst_n = '0' then
      sw_trig_fixed_delay   <= (others => '0');
      ext_trig_fixed_delay  <= (others => '0');
      time_trig_fixed_delay <= (others => '0');
    elsif rising_edge(fs_clk) then
      sw_trig_fixed_delay   <= sw_trig_fixed_delay(sw_trig_fixed_delay'HIGH -1 downto 0) & sw_trig;
      ext_trig_fixed_delay  <= ext_trig_fixed_delay(ext_trig_fixed_delay'HIGH -1 downto 0) & ext_trig_d;
      time_trig_fixed_delay <= time_trig_fixed_delay(time_trig_fixed_delay'HIGH -1 downto 0) & time_trig;
    end if;
  end process p_trig_shift;

  -- Trigger sources ORing
  trig <= (sw_trig_fixed_delay(sw_trig_fixed_delay'HIGH) and sw_trig_en) or
          (ext_trig_fixed_delay(ext_trig_fixed_delay'HIGH) and ext_trig_en) or
          (int_trig_d(1) and int_trig_en(1)) or
          (int_trig_d(2) and int_trig_en(2)) or
          (int_trig_d(3) and int_trig_en(3)) or
          (int_trig_d(4) and int_trig_en(4)) or
          (time_trig_fixed_delay(time_trig_fixed_delay'HIGH) and time_trig_en);

  ------------------------------------------------------------------------------
  -- Trigger source storage and synchronisation to system clock domain
  ------------------------------------------------------------------------------

  trig_fifo_din <= trig & X"00000" &
                   int_trig_d(4) & int_trig_d(3) &
                   int_trig_d(2) & int_trig_d(1) &
                   "000" & time_trig_fixed_delay(time_trig_fixed_delay'HIGH) &
                   "00" & sw_trig_fixed_delay(sw_trig_fixed_delay'HIGH) &
                   ext_trig_fixed_delay(ext_trig_fixed_delay'HIGH);

  trig_fifo_wr <= not trig_fifo_full and acq_in_wait_trig;

  cmp_trig_sync_fifo : generic_async_fifo
    generic map (
      g_data_width             => 33,
      g_size                   => 16,
      g_show_ahead             => FALSE,
      g_with_rd_empty          => TRUE,
      g_with_rd_full           => FALSE,
      g_with_rd_almost_empty   => FALSE,
      g_with_rd_almost_full    => FALSE,
      g_with_rd_count          => FALSE,
      g_with_wr_empty          => FALSE,
      g_with_wr_full           => TRUE,
      g_with_wr_almost_empty   => FALSE,
      g_with_wr_almost_full    => FALSE,
      g_with_wr_count          => FALSE,
      g_almost_empty_threshold => 0,
      g_almost_full_threshold  => 0
      )
    port map(
      rst_n_i           => fs_rst_n,
      clk_wr_i          => fs_clk,
      d_i               => trig_fifo_din,
      we_i              => trig_fifo_wr,
      wr_empty_o        => open,
      wr_full_o         => trig_fifo_full,
      wr_almost_empty_o => open,
      wr_almost_full_o  => open,
      wr_count_o        => open,
      clk_rd_i          => sys_clk_i,
      q_o               => trig_fifo_dout,
      rd_i              => trig_fifo_rd,
      rd_empty_o        => trig_fifo_empty,
      rd_full_o         => open,
      rd_almost_empty_o => open,
      rd_almost_full_o  => open,
      rd_count_o        => open
      );

  trig_fifo_rd <= not trig_fifo_empty;

  p_trig_storage_sys: process (sys_clk_i) is
  begin
    if rising_edge(sys_clk_i) then
      if sys_rst_n_i = '0' or trig_storage_clear = '1' then
        trig_storage <= (others => '0');
      elsif trig_fifo_dout(32) = '1' and trig_fifo_empty = '0' then
        trig_storage <= trig_fifo_dout(31 downto 0);
      end if;
    end if;
  end process p_trig_storage_sys;

  ------------------------------------------------------------------------------
  -- Under-sampling and trigger alignment
  --    When under-sampling is enabled, if the trigger occurs between two
  --    samples it will be realigned to the next sample
  ------------------------------------------------------------------------------
  p_undersample_cnt : process (fs_clk, fs_rst_n)
  begin
    if fs_rst_n = '0' then
      undersample_cnt <= to_unsigned(1, undersample_cnt'length);
      undersample_en  <= '0';
    elsif rising_edge(fs_clk) then
      if undersample_cnt = to_unsigned(0, undersample_cnt'length) then
        if undersample_factor /= X"00000000" then
          undersample_cnt <= unsigned(undersample_factor) - 1;
        end if;
        undersample_en <= '1';
      else
        undersample_cnt <= undersample_cnt - 1;
        undersample_en  <= '0';
      end if;
    end if;
  end process p_undersample_cnt;

  p_trig_align : process (fs_clk, fs_rst_n)
  begin
    if fs_rst_n = '0' then
      trig_align <= '0';
    elsif rising_edge(fs_clk) then
      if trig = '1' then
        trig_align <= '1';
      elsif undersample_en = '1' then
        trig_align <= '0';
      end if;
    end if;
  end process p_trig_align;

  ------------------------------------------------------------------------------
  -- Synchronisation FIFO to system clock domain
  ------------------------------------------------------------------------------
  cmp_adc_sync_fifo : generic_async_fifo
    generic map (
      g_data_width             => 65,
      g_size                   => 16,
      g_show_ahead             => FALSE,
      g_with_rd_empty          => TRUE,
      g_with_rd_full           => FALSE,
      g_with_rd_almost_empty   => FALSE,
      g_with_rd_almost_full    => FALSE,
      g_with_rd_count          => FALSE,
      g_with_wr_empty          => FALSE,
      g_with_wr_full           => TRUE,
      g_with_wr_almost_empty   => FALSE,
      g_with_wr_almost_full    => FALSE,
      g_with_wr_count          => FALSE,
      g_almost_empty_threshold => 0,
      g_almost_full_threshold  => 0
      )
    port map(
      rst_n_i           => fs_rst_n,
      clk_wr_i          => fs_clk,
      d_i               => sync_fifo_din,
      we_i              => sync_fifo_wr,
      wr_empty_o        => open,                  -- sync_fifo_empty,
      wr_full_o         => sync_fifo_full,
      wr_almost_empty_o => open,
      wr_almost_full_o  => open,
      wr_count_o        => open,
      clk_rd_i          => sys_clk_i,
      q_o               => sync_fifo_dout,
      rd_i              => sync_fifo_rd,
      rd_empty_o        => sync_fifo_empty,
      rd_full_o         => open,
      rd_almost_empty_o => open,
      rd_almost_full_o  => open,
      rd_count_o        => open
      );

  -- One clock cycle delay for the FIFO's VALID signal. Since the General Cores
  -- package does not offer the possibility to use the FWFT feature of the FIFOs,
  -- we simulate the valid flag here according to Figure 4-7 in Xilinx UG175.
  p_sync_fifo_valid : process (sys_clk_i) is
  begin
    if rising_edge(sys_clk_i) then
      sync_fifo_valid <= sync_fifo_rd;
      if (sync_fifo_empty = '1') then
        sync_fifo_valid <= '0';
      end if;
    end if;
  end process;

  -- Data to FIFO
  sync_fifo_din(64) <= trig_align;
  sync_fifo_din(63 downto 0) <= data_calibr_out_d3;

  sync_fifo_wr <= undersample_en and serdes_synced and not(sync_fifo_full);

  sync_fifo_rd <= not(sync_fifo_empty);           -- read sync fifo as soon as data are available


  --============================================================================
  -- System clock domain
  --============================================================================

  ------------------------------------------------------------------------------
  -- Shots counter
  ------------------------------------------------------------------------------
  p_shots_cnt : process (sys_clk_i, sys_rst_n_i)
  begin
    if sys_rst_n_i = '0' then
      shots_cnt   <= to_unsigned(0, shots_cnt'length);
      single_shot <= '0';
    elsif rising_edge(sys_clk_i) then
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
  end process p_shots_cnt;

  multishot_buffer_sel <= std_logic(shots_cnt(0));
  shots_done           <= '1' when shots_cnt = to_unsigned(1, shots_cnt'length) else '0';
  remaining_shots      <= std_logic_vector(shots_cnt);

  ------------------------------------------------------------------------------
  -- Pre-trigger counter
  ------------------------------------------------------------------------------
  p_pre_trig_cnt : process (sys_clk_i, sys_rst_n_i)
  begin
    if sys_rst_n_i = '0' then
      pre_trig_cnt <= to_unsigned(1, pre_trig_cnt'length);
    elsif rising_edge(sys_clk_i) then
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
  end process p_pre_trig_cnt;


  pre_trig_done <= '1' when (pre_trig_cnt = to_unsigned(0, pre_trig_cnt'length) and
                             sync_fifo_valid = '1' and acq_in_pre_trig = '1') else '0';

  ------------------------------------------------------------------------------
  -- Post-trigger counter
  ------------------------------------------------------------------------------
  p_post_trig_cnt : process (sys_clk_i, sys_rst_n_i)
  begin
    if sys_rst_n_i = '0' then
      post_trig_cnt <= to_unsigned(1, post_trig_cnt'length);
    elsif rising_edge(sys_clk_i) then
      if (acq_start = '1' or post_trig_done = '1') then
        post_trig_cnt <= unsigned(post_trig_value) - 1;
      elsif (acq_in_post_trig = '1' and sync_fifo_valid = '1') then
        post_trig_cnt <= post_trig_cnt - 1;
      end if;
    end if;
  end process p_post_trig_cnt;

  post_trig_done <= '1' when (post_trig_cnt = to_unsigned(0, post_trig_cnt'length) and
                              sync_fifo_valid = '1' and acq_in_post_trig = '1') else '0';

  ------------------------------------------------------------------------------
  -- Samples counter
  ------------------------------------------------------------------------------
  p_samples_cnt : process (sys_clk_i, sys_rst_n_i)
  begin
    if sys_rst_n_i = '0' then
      samples_cnt <= (others => '0');
    elsif rising_edge(sys_clk_i) then
      if (acq_start = '1') then
        samples_cnt <= (others => '0');
      elsif ((acq_in_pre_trig = '1' or acq_in_post_trig = '1') and sync_fifo_valid = '1') then
        samples_cnt <= samples_cnt + 1;
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
      if sys_rst_n_i = '0' then
        acq_end_d <= '0';
      else
        acq_end_d <= acq_end;
      end if;
    end if;
  end process p_acq_end;

  acq_end_p_o <= acq_end and not(acq_end_d);

  -- FSM commands
  acq_start <= '1' when fsm_cmd_wr = '1' and fsm_cmd = "01" else '0';
  acq_stop  <= '1' when fsm_cmd_wr = '1' and fsm_cmd = "10" else '0';
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
        unsigned(pre_trig_value) + unsigned(post_trig_value) + 3 >= to_unsigned(g_multishot_ram_size, pre_trig_value'length) then
        acq_config_ok <= '0';
      else
        acq_config_ok <= '1';
      end if;
    end if;
  end process p_acq_cfg_ok;

  -- FSM transitions
  p_acq_fsm_transitions : process(sys_clk_i, sys_rst_n_i)
  begin
    if sys_rst_n_i = '0' then
      acq_fsm_current_state <= IDLE;
    elsif rising_edge(sys_clk_i) then

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
  p_trig_tag_done : process (sys_clk_i, sys_rst_n_i)
  begin
    if sys_rst_n_i = '0' then
      acq_in_trig_tag_d <= '0';
    elsif rising_edge(sys_clk_i) then
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
  p_dpram_addra_cnt : process (sys_clk_i, sys_rst_n_i)
  begin
    if sys_rst_n_i = '0' then
      dpram_addra_cnt       <= (others => '0');
      dpram_addra_trig      <= (others => '0');
      dpram_addra_post_done <= (others => '0');
    elsif rising_edge(sys_clk_i) then
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
  end process p_dpram_addra_cnt;

  -- DPRAM inputs
  dpram0_addra <= std_logic_vector(dpram_addra_cnt);
  dpram1_addra <= std_logic_vector(dpram_addra_cnt);
  dpram0_dina  <= sync_fifo_dout(63 downto 0)                            when acq_in_trig_tag = '0'      else trig_tag_data;
  dpram1_dina  <= sync_fifo_dout(63 downto 0)                            when acq_in_trig_tag = '0'      else trig_tag_data;
  dpram0_wea   <= (samples_wr_en and sync_fifo_valid) or acq_in_trig_tag when multishot_buffer_sel = '0' else '0';
  dpram1_wea   <= (samples_wr_en and sync_fifo_valid) or acq_in_trig_tag when multishot_buffer_sel = '1' else '0';

  -- DPRAMs
  cmp_multishot_dpram0 : generic_dpram
    generic map
    (
      g_data_width               => 64,
      g_size                     => g_multishot_ram_size,
      g_with_byte_enable         => FALSE,
      g_addr_conflict_resolution => "read_first",
      g_dual_clock               => FALSE
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
      g_data_width               => 64,
      g_size                     => g_multishot_ram_size,
      g_with_byte_enable         => FALSE,
      g_addr_conflict_resolution => "read_first",
      g_dual_clock               => FALSE
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
  p_dpram_addrb_cnt : process (sys_clk_i, sys_rst_n_i)
  begin
    if sys_rst_n_i = '0' then
      dpram_addrb_cnt <= (others => '0');
      dpram_valid_t   <= '0';
      dpram_valid     <= '0';
    elsif rising_edge(sys_clk_i) then
      if trig_tag_done = '1' then
        dpram_addrb_cnt <= dpram_addra_trig - unsigned(pre_trig_value(c_dpram_depth-1 downto 0));
        dpram_valid_t   <= '1';
      elsif (dpram_addrb_cnt = dpram_addra_post_done + 2) then  -- reads 2 extra addresses -> trigger time-tag
        dpram_valid_t <= '0';
      else
        dpram_addrb_cnt <= dpram_addrb_cnt + 1;
      end if;
      dpram_valid <= dpram_valid_t;
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
      g_data_width             => 65,
      g_size                   => 256,
      g_show_ahead             => FALSE,
      g_with_empty             => TRUE,
      g_with_full              => TRUE,
      g_with_almost_empty      => FALSE,
      g_with_almost_full       => FALSE,
      g_with_count             => FALSE,
      g_almost_empty_threshold => 0,
      g_almost_full_threshold  => 0
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
  p_wb_ddr_fifo_valid : process (sys_clk_i) is
  begin
    if rising_edge(sys_clk_i) then
      wb_ddr_fifo_valid <= wb_ddr_fifo_rd;
      if (wb_ddr_fifo_empty = '1') then
        wb_ddr_fifo_valid <= '0';
      end if;
    end if;
  end process;

  p_wb_ddr_fifo_input : process (sys_clk_i, sys_rst_n_i)
  begin
    if sys_rst_n_i = '0' then
      wb_ddr_fifo_din   <= (others => '0');
      wb_ddr_fifo_wr_en <= '0';
    elsif rising_edge(sys_clk_i) then
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
  end process p_wb_ddr_fifo_input;

  wb_ddr_fifo_wr <= wb_ddr_fifo_wr_en and not(wb_ddr_fifo_full);

  wb_ddr_fifo_rd   <= wb_ddr_fifo_dreq and not(wb_ddr_fifo_empty) and not(wb_ddr_stall_t);
  wb_ddr_fifo_dreq <= '1';

  ------------------------------------------------------------------------------
  -- RAM address counter (32-bit word address)
  ------------------------------------------------------------------------------
  p_ram_addr_cnt : process (wb_ddr_clk_i, sys_rst_n_i)
  begin
    if sys_rst_n_i = '0' then
      ram_addr_cnt <= (others => '0');
    elsif rising_edge(wb_ddr_clk_i) then
      if acq_start = '1' then
        ram_addr_cnt <= (others => '0');
      elsif wb_ddr_fifo_valid = '1' then
        ram_addr_cnt <= ram_addr_cnt + 1;
      end if;
    end if;
  end process p_ram_addr_cnt;

  ------------------------------------------------------------------------------
  -- Store trigger DDR address (byte address)
  ------------------------------------------------------------------------------
  p_trig_addr : process (wb_ddr_clk_i, sys_rst_n_i)
  begin
    if sys_rst_n_i = '0' then
      trig_addr <= (others => '0');
    elsif rising_edge(wb_ddr_clk_i) then
      if wb_ddr_fifo_dout(64) = '1' and wb_ddr_fifo_valid = '1' then
        trig_addr <= "0000" & std_logic_vector(ram_addr_cnt) & "000";
      end if;
    end if;
  end process p_trig_addr;

  ------------------------------------------------------------------------------
  -- Wishbone master (to DDR)
  ------------------------------------------------------------------------------
  p_wb_master : process (wb_ddr_clk_i, sys_rst_n_i)
  begin
    if sys_rst_n_i = '0' then
      wb_ddr_cyc_o   <= '0';
      wb_ddr_we_o    <= '0';
      wb_ddr_stb_o   <= '0';
      wb_ddr_adr_o   <= (others => '0');
      wb_ddr_dat_o   <= (others => '0');
      wb_ddr_stall_t <= '0';
    elsif rising_edge(wb_ddr_clk_i) then

      if wb_ddr_fifo_valid = '1' then  --if (wb_ddr_fifo_valid = '1') and (wb_ddr_stall_i = '0') then
        wb_ddr_stb_o <= '1';
        wb_ddr_adr_o <= "0000000" & std_logic_vector(ram_addr_cnt);
        if test_data_en = '1' then
          wb_ddr_dat_o <= x"00000000" & "0000000" & std_logic_vector(ram_addr_cnt);
        else
          wb_ddr_dat_o <= wb_ddr_fifo_dout(63 downto 0);
        end if;
      else
        wb_ddr_stb_o <= '0';
      end if;

      if wb_ddr_fifo_valid = '1' then
        wb_ddr_cyc_o <= '1';
        wb_ddr_we_o  <= '1';
      --elsif (wb_ddr_fifo_empty = '1') and (acq_end = '1') then
      elsif (wb_ddr_fifo_empty = '1') and (acq_fsm_state = "001") then
        wb_ddr_cyc_o <= '0';
        wb_ddr_we_o  <= '0';
      end if;

      wb_ddr_stall_t <= wb_ddr_stall_i;

    end if;
  end process p_wb_master;

  wb_ddr_sel_o <= X"FF";

end rtl;
