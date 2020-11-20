// SPDX-FileCopyrightText: 2020 CERN (home.cern)
//
// SPDX-License-Identifier: CC0-1.0

`timescale 1ns/1ps

`include "gn4124_bfm.svh"
`include "spec_ref_fmc_adc_100Ms_mmap.v"
`include "fmc_adc_mezzanine_mmap.v"
`include "fmc_adc_100Ms_csr.v"
`include "fmc_adc_eic_regs.v"
`include "timetag_core_regs.v"

`define DMA_BASE 'h00c0
`define VIC_BASE 'h0100

`define ADC_OFFSET `ADDR_SPEC_REF_FMC_ADC_100M_MMAP_FMC_ADC_MEZZANINE
`define CSR_BASE   `ADC_OFFSET + `ADDR_FMC_ADC_MEZZANINE_MMAP_FMC_ADC_100M_CSR
`define EIC_BASE   `ADC_OFFSET + `ADDR_FMC_ADC_MEZZANINE_MMAP_FMC_ADC_EIC
`define TAG_BASE   `ADC_OFFSET + `ADDR_FMC_ADC_MEZZANINE_MMAP_TIMETAG_CORE

module main;

   reg rst_n = 0;

   reg clk_125m_pllref = 0;

   initial begin
      repeat(20) @(posedge clk_125m_pllref);
      rst_n = 1;
   end

   IGN4124PCIMaster i_gn4124 ();

   reg adc_dco = 1'b0;
   reg adc_fr = 1'b0;
   reg ext_trig = 1'b0;
   reg adc_data_dir = 1'b0;

   reg[3:0] adc_dat_odd  = 4'h0;
   reg[3:0] adc_dat_even = 4'h0;
   reg signed [13:0] adc_data = 0;

   wire ddr_cas_n, ddr_ck_p, ddr_ck_n, ddr_cke;
   wire [1:0] ddr_dm, ddr_dqs_p, ddr_dqs_n;
   wire ddr_odt, ddr_ras_n, ddr_reset_n, ddr_we_n;
   wire [15:0] ddr_dq;
   wire [13:0] ddr_a;
   wire [2:0]  ddr_ba;
   wire        ddr_rzq;

   pulldown(ddr_rzq);

   // 400Mhz
   always #1.25ns adc_dco <= ~adc_dco;

   // 125Mhz
   always #4ns clk_125m_pllref <= ~clk_125m_pllref;

   bit [4:0] slot_id = 8;

   spec_ref_fmc_adc_100Ms
     #(
       .g_SIMULATION(1)
       )
   DUT
     (
      .button1_n_i              (rst_n),
      .clk_125m_pllref_p_i      (clk_125m_pllref),
      .clk_125m_pllref_n_i      (~clk_125m_pllref),
      .clk_125m_gtp_p_i         (clk_125m_pllref),
      .clk_125m_gtp_n_i         (~clk_125m_pllref),
      .adc_ext_trigger_p_i      (ext_trig),
      .adc_ext_trigger_n_i      (~ext_trig),
      .adc_dco_p_i              (adc_dco),
      .adc_dco_n_i              (~adc_dco),
      .adc_fr_p_i               (~adc_fr),
      .adc_fr_n_i               (adc_fr),
      .adc_outa_p_i             (adc_dat_odd),
      .adc_outa_n_i             (~adc_dat_odd),
      .adc_outb_p_i             (adc_dat_even),
      .adc_outb_n_i             (~adc_dat_even),
      .gn_rst_n_i                (i_gn4124.rst_n),
      .gn_p2l_clk_n_i            (i_gn4124.p2l_clk_n),
      .gn_p2l_clk_p_i            (i_gn4124.p2l_clk_p),
      .gn_p2l_rdy_o              (i_gn4124.p2l_rdy),
      .gn_p2l_dframe_i           (i_gn4124.p2l_dframe),
      .gn_p2l_valid_i            (i_gn4124.p2l_valid),
      .gn_p2l_data_i             (i_gn4124.p2l_data),
      .gn_p_wr_req_i             (i_gn4124.p_wr_req),
      .gn_p_wr_rdy_o             (i_gn4124.p_wr_rdy),
      .gn_rx_error_o             (i_gn4124.rx_error),
      .gn_l2p_clk_n_o            (i_gn4124.l2p_clk_n),
      .gn_l2p_clk_p_o            (i_gn4124.l2p_clk_p),
      .gn_l2p_dframe_o           (i_gn4124.l2p_dframe),
      .gn_l2p_valid_o            (i_gn4124.l2p_valid),
      .gn_l2p_edb_o              (i_gn4124.l2p_edb),
      .gn_l2p_data_o             (i_gn4124.l2p_data),
      .gn_l2p_rdy_i              (i_gn4124.l2p_rdy),
      .gn_l_wr_rdy_i             (i_gn4124.l_wr_rdy),
      .gn_p_rd_d_rdy_i           (i_gn4124.p_rd_d_rdy),
      .gn_tx_error_i             (i_gn4124.tx_error),
      .gn_vc_rdy_i               (i_gn4124.vc_rdy),
      .gn_gpio_b                 (),
      .ddr_a_o                   (ddr_a),
      .ddr_ba_o                  (ddr_ba),
      .ddr_cas_n_o               (ddr_cas_n),
      .ddr_ck_n_o                (ddr_ck_n),
      .ddr_ck_p_o                (ddr_ck_p),
      .ddr_cke_o                 (ddr_cke),
      .ddr_dq_b                  (ddr_dq),
      .ddr_ldm_o                 (ddr_dm[0]),
      .ddr_ldqs_n_b              (ddr_dqs_n[0]),
      .ddr_ldqs_p_b              (ddr_dqs_p[0]),
      .ddr_odt_o                 (ddr_odt),
      .ddr_ras_n_o               (ddr_ras_n),
      .ddr_reset_n_o             (ddr_reset_n),
      .ddr_rzq_b                 (ddr_rzq),
      .ddr_udm_o                 (ddr_dm[1]),
      .ddr_udqs_n_b              (ddr_dqs_n[1]),
      .ddr_udqs_p_b              (ddr_dqs_p[1]),
      .ddr_we_n_o                (ddr_we_n)
      );

   ddr3 #
     (
      .DEBUG(0),
      .check_strict_timing(0),
      .check_strict_mrbits(0)
      )
   cmp_ddr0
     (
      .rst_n   (ddr_reset_n),
      .ck      (ddr_ck_p),
      .ck_n    (ddr_ck_n),
      .cke     (ddr_cke),
      .cs_n    (1'b0),
      .ras_n   (ddr_ras_n),
      .cas_n   (ddr_cas_n),
      .we_n    (ddr_we_n),
      .dm_tdqs (ddr_dm),
      .ba      (ddr_ba),
      .addr    (ddr_a),
      .dq      (ddr_dq),
      .dqs     (ddr_dqs_p),
      .dqs_n   (ddr_dqs_n),
      .tdqs_n  (),
      .odt     (ddr_odt)
      );

   int adc_div = 0;

   always@(negedge adc_dco)
     begin
	#625ps;
	if(adc_div == 1) begin
	   adc_fr <= ~adc_fr;
	   adc_div <= 0;
	end
	else begin
	   adc_div <= adc_div + 1;
	end
     end

   //  Generate a triangular waveform on all channels.
   always@(posedge adc_fr)
     begin
	if ((adc_data > 400) || (adc_data < -400)) begin
	   adc_data_dir = ~adc_data_dir;
	end
	if (adc_data_dir == 0) begin
	   adc_data = adc_data + 8;
	end
	else begin
	   adc_data = adc_data - 8;
	end
	adc_dat_odd  = {4{adc_data[13]}};
	adc_dat_even = {4{adc_data[12]}};
	#1250ps;
	adc_dat_odd  = {4{adc_data[11]}};
	adc_dat_even = {4{adc_data[10]}};
	#1250ps;
	adc_dat_odd  = {4{adc_data[9]}};
	adc_dat_even = {4{adc_data[8]}};
	#1250ps;
	adc_dat_odd  = {4{adc_data[7]}};
	adc_dat_even = {4{adc_data[6]}};
	#1250ps;
	adc_dat_odd  = {4{adc_data[5]}};
	adc_dat_even = {4{adc_data[4]}};
	#1250ps;
	adc_dat_odd  = {4{adc_data[3]}};
	adc_dat_even = {4{adc_data[2]}};
	#1250ps;
	adc_dat_odd  = {4{adc_data[1]}};
	adc_dat_even = {4{adc_data[0]}};
	#1250ps;
	adc_dat_odd  = {4{1'b0}};
	adc_dat_even = {4{1'b0}};
     end

   task adc_status_print (input uint64_t val);
      string msg;
      msg = $sformatf ("<%t> ADC STATUS: FSM_STATE=%0d, PLL_LOCKED=%0d, PLL_SYNCED=%0d, CFG_OK=%0d",
		       $realtime,
		       (val &  `FMC_ADC_100MS_CSR_STA_FSM) >> `FMC_ADC_100MS_CSR_STA_FSM_OFFSET,
		       (val &  `FMC_ADC_100MS_CSR_STA_SERDES_PLL) >> `FMC_ADC_100MS_CSR_STA_SERDES_PLL_OFFSET,
		       (val &  `FMC_ADC_100MS_CSR_STA_SERDES_SYNCED) >> `FMC_ADC_100MS_CSR_STA_SERDES_SYNCED_OFFSET,
		       (val &  `FMC_ADC_100MS_CSR_STA_ACQ_CFG) >> `FMC_ADC_100MS_CSR_STA_ACQ_CFG_OFFSET);
      $display(msg);
   endtask // adc_status_print

   initial begin

      int i;

      uint64_t val, expected;

      CBusAccessor acc;

      acc = i_gn4124.get_accessor();
      acc.set_default_xfer_size(4);

      $timeformat (-6, 3, "us", 10);

      #2us;

      expected = 'h19;
      acc.read(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_STA, val);
      if (val != expected)
	begin
	   adc_status_print(val);
	   $fatal (1, "ADC status error (got 0x%8x, expected 0x%8x).", val, expected);
	end

      // Configure the EIC for an interrupt on ACQ_END
      acc.write(`EIC_BASE + `ADDR_FMC_ADC_EIC_REGS_IER, 'h2);

      // Configure the VIC
      acc.write(`VIC_BASE + 'h8, 'h7f);
      acc.write(`VIC_BASE + 'h0, 'h1);

      // FMC-ADC core general configuration
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_PRE_SAMPLES,  'h00000000);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_POST_SAMPLES, 'h00000001);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_SHOTS,        'h00000001);

      // FMC-ADC core channel configuration
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CH1_CALIB, 'h00008000);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CH2_CALIB, 'h00008000);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CH3_CALIB, 'h00008000);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CH4_CALIB, 'h00008000);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CTL,       'h00008000); // apply calibration
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CH1_SAT,   'h00007fff);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CH2_SAT,   'h00007fff);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CH3_SAT,   'h00007fff);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CH4_SAT,   'h00007fff);

      // FMC-ADC core trigger configuration
      val = (16'h100 << `FMC_ADC_100MS_CSR_CH1_TRIG_THRES_HYST_OFFSET) |
	    (16'h300 << `FMC_ADC_100MS_CSR_CH1_TRIG_THRES_VAL_OFFSET);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CH1_TRIG_THRES, val);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CH2_TRIG_THRES, val);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CH3_TRIG_THRES, val);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CH4_TRIG_THRES, val);
      val = (1'b1 << `FMC_ADC_100MS_CSR_TRIG_EN_SW_OFFSET);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_TRIG_EN, val);

      expected = 'h39;
      acc.read(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_STA, val);
      if (val != expected)
	begin
	   adc_status_print(val);
	   $fatal (1, "ADC status error (got 0x%8x, expected 0x%8x).", val, expected);
	end

      #1us;

      acc.write(`TAG_BASE + `ADDR_TIMETAG_CORE_REGS_SECONDS_UPPER, 'h00000032);
      acc.write(`TAG_BASE + `ADDR_TIMETAG_CORE_REGS_SECONDS_LOWER, 'h00005a34);
      acc.write(`TAG_BASE + `ADDR_TIMETAG_CORE_REGS_COARSE,        'h00000000);

      $display("<%t> START ACQ 1/4", $realtime);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CTL, 'h00000001); // FSM start

      wait (DUT.cmp_fmc_adc_mezzanine.cmp_fmc_adc_100Ms_core.acq_in_wait_trig == 1);

      #200ns;

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_SW_TRIG, 'hFFFFFFFF); // soft trigger

      wait (DUT.cmp_fmc_adc_mezzanine.acq_end_irq_o == 1);
      $display("<%t> END ACQ 1/4", $realtime);
      acc.write(`EIC_BASE + `ADDR_FMC_ADC_EIC_REGS_ISR, 'h2);
      acc.write(`VIC_BASE + 'h1c, 'h0);

      #200ns;

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_SHOTS, 'h00000003); // #nshots: 3x multi-shot acq

      $display("<%t> START ACQ 2/4", $realtime);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CTL, 'h00000001); // FSM start

      #500ns;

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_SW_TRIG, 'hFFFFFFFE); // soft trigger

      #500ns;

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_SW_TRIG, 'hFFFFFFFD); // soft trigger

      #500ns;

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_SW_TRIG, 'hFFFFFFFC); // soft trigger

      wait (DUT.cmp_fmc_adc_mezzanine.acq_end_irq_o == 1);
      $display("<%t> END ACQ 2/4", $realtime);
      acc.write(`EIC_BASE + `ADDR_FMC_ADC_EIC_REGS_ISR, 'h2);
      acc.write(`VIC_BASE + 'h1c, 'h0);

      #1us;

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_SHOTS, 'h0000005);

      // FMC-ADC core trigger configuration
      val = (1'b1    << `FMC_ADC_100MS_CSR_TRIG_EN_SW_OFFSET)  |
	    (1'b1    << `FMC_ADC_100MS_CSR_TRIG_EN_CH1_OFFSET) |
	    (1'b1    << `FMC_ADC_100MS_CSR_TRIG_EN_CH3_OFFSET);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_TRIG_EN, val);

      $display("<%t> START ACQ 3/4", $realtime);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CTL, 'h00000001); // FSM start

      #1us;

      wait (DUT.cmp_fmc_adc_mezzanine.acq_end_irq_o == 1);
      $display("<%t> END ACQ 3/4", $realtime);
      acc.write(`EIC_BASE + `ADDR_FMC_ADC_EIC_REGS_ISR, 'h2);
      acc.write(`VIC_BASE + 'h1c, 'h0);

      #1us;

      // set time trigger
      acc.write(`TAG_BASE + `ADDR_TIMETAG_CORE_REGS_TIME_TRIG_SECONDS_UPPER, 'h00000032);
      acc.write(`TAG_BASE + `ADDR_TIMETAG_CORE_REGS_TIME_TRIG_SECONDS_LOWER, 'h00005a34);
      acc.write(`TAG_BASE + `ADDR_TIMETAG_CORE_REGS_TIME_TRIG_COARSE,        'h00001100);

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_PRE_SAMPLES,  'h00000000);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_POST_SAMPLES, 'h000001fd);

      // FMC-ADC core trigger configuration
      val = (1'b1 << `FMC_ADC_100MS_CSR_TRIG_EN_TIME_OFFSET);

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_TRIG_EN, val);

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_SHOTS, 'h0000001);

      $display("<%t> START ACQ 4/4", $realtime);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CTL, 'h00000001); // FSM start

      #5us;

      wait (DUT.cmp_fmc_adc_mezzanine.acq_end_irq_o == 1);
      $display("<%t> END ACQ 4/4", $realtime);
      acc.write(`EIC_BASE + `ADDR_FMC_ADC_EIC_REGS_ISR, 'h2);
      acc.write(`VIC_BASE + 'h1c, 'h0);

      #1us;

      expected = 'h39;
      acc.read(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_STA, val);
      if (val != expected)
	begin
	   adc_status_print(val);
	   $fatal (1, "ADC status error (got 0x%8x, expected 0x%8x).", val, expected);
	end

      acc.read(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_TRIG_POS, val);
      $display("<%t> Start DMA, trigger position in DDR: %.8x", $realtime, val);

      // DMA transfer
      acc.write(`DMA_BASE + 'h08, val); // dma start addr

      acc.write(`DMA_BASE + 'h0C, 'h00001000); // host addr
      acc.write(`DMA_BASE + 'h10, 'h00000000);

      // length = (samples + trigger sample + 2 for tag) * 8 bytes
      acc.write(`DMA_BASE + 'h14, 'h00001000); // length

      acc.write(`DMA_BASE + 'h18, 'h00000000); // next
      acc.write(`DMA_BASE + 'h1C, 'h00000000);

      acc.write(`DMA_BASE + 'h20, 'h00000000); // attrib: pcie -> host

      acc.write(`DMA_BASE + 'h00, 'h00000001); // xfer start

      wait (DUT.inst_spec_base.irqs[2] == 1);
      $display("<%t> END DMA", $realtime);
      acc.write(`DMA_BASE + 'h04, 'h04); // clear DMA IRQ
      acc.write(`VIC_BASE + 'h1c, 'h0);

      $display ("Simulation PASSED");
      $finish;

   end

   initial begin
      // Silence Xilinx unisim DSP48A1 warnings about invalid OPMODE
      force DUT.inst_spec_base.gen_wr.cmp_xwrc_board_spec.cmp_board_common.cmp_xwr_core.
        WRPC.LM32_CORE.gen_profile_medium_icache.U_Wrapped_LM32.cpu.
          multiplier.D1.OPMODE_dly = 0;
      force DUT.inst_spec_base.gen_wr.cmp_xwrc_board_spec.cmp_board_common.cmp_xwr_core.
        WRPC.LM32_CORE.gen_profile_medium_icache.U_Wrapped_LM32.cpu.
          multiplier.D2.OPMODE_dly = 0;
      force DUT.inst_spec_base.gen_wr.cmp_xwrc_board_spec.cmp_board_common.cmp_xwr_core.
        WRPC.LM32_CORE.gen_profile_medium_icache.U_Wrapped_LM32.cpu.
          multiplier.D3.OPMODE_dly = 0;
   end // initial begin

endmodule // main
