--------------------------------------------------------------------------------
-- SPDX-License-Identifier: CERN-OHL-W-2.0
-- CERN BE-CO-HT
-- FMC ADC 100M 14B 4CHA
-- http://www.ohwr.org/projects/fmc-adc-100m14b4cha-gw
--------------------------------------------------------------------------------
--
-- unit name:   fmc_adc_eic.vhd
--
-- description: FMC ADC 100Ms/s Embedded Interrupt Controller
--
--------------------------------------------------------------------------------
-- Copyright CERN 2020
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.wishbone_pkg.all;
use work.fmc_adc_eic_regs_pkg.all;

entity fmc_adc_eic is
  port (
    rst_n_i       : in  std_logic;
    clk_i         : in  std_logic;
    wb_i          : in  t_wishbone_slave_in;
    wb_o          : out t_wishbone_slave_out;
    irq_trig_i    : in  std_logic;
    irq_acq_end_i : in  std_logic;
    int_o         : out std_logic);
end fmc_adc_eic;

architecture arch of fmc_adc_eic is

  signal irq_vector : std_logic_vector(1 downto 0);
  signal regs_in    : t_fmc_adc_eic_regs_master_in;
  signal regs_out   : t_fmc_adc_eic_regs_master_out;

begin

  cmp_fmc_adc_eic_regs : entity work.fmc_adc_eic_regs
    port map (
      rst_n_i            => rst_n_i,
      clk_i              => clk_i,
      wb_i               => wb_i,
      wb_o               => wb_o,
      fmc_adc_eic_regs_i => regs_in,
      fmc_adc_eic_regs_o => regs_out);

  -- by default, wbgen2_eic interrupts are rising-edge sensitive
  cmp_fmc_adc_eic_controller : entity work.wbgen2_eic
    generic map (
      g_NUM_INTERRUPTS => 2)
    port map (
      clk_i            => clk_i,
      rst_n_i          => rst_n_i,
      irq_i            => irq_vector,
      irq_ack_o        => open,
      reg_imr_o        => regs_in.imr(1 downto 0),
      reg_ier_i        => regs_out.ier(1 downto 0),
      reg_ier_wr_stb_i => regs_out.ier_wr,
      reg_idr_i        => regs_out.idr(1 downto 0),
      reg_idr_wr_stb_i => regs_out.idr_wr,
      reg_isr_o        => regs_in.isr(1 downto 0),
      reg_isr_i        => regs_out.isr(1 downto 0),
      reg_isr_wr_stb_i => regs_out.isr_wr,
      wb_irq_o         => int_o);

  irq_vector(0) <= irq_trig_i;
  irq_vector(1) <= irq_acq_end_i;

end architecture arch;
