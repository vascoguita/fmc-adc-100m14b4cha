// SPDX-FileCopyrightText: 2020 CERN (home.cern)
//
// SPDX-License-Identifier: CC0-1.0

`timescale 1ns/1ps

`include "vme64x_bfm.svh"
`include "svec_vme_buffers.svh"
`include "svec_ref_fmc_adc_100Ms_mmap.v"
`include "fmc_adc_mezzanine_mmap.v"
`include "fmc_adc_100Ms_csr.v"
`include "fmc_adc_100Ms_channel_regs.v"
`include "fmc_adc_eic_regs.v"
`include "timetag_core_regs.v"

`define VME_OFFSET 'h80000000
`define ADC1_OFFSET `VME_OFFSET + `ADDR_SVEC_REF_FMC_ADC_100M_MMAP_FMC1_ADC_MEZZANINE
`define ADC2_OFFSET `VME_OFFSET + `ADDR_SVEC_REF_FMC_ADC_100M_MMAP_FMC2_ADC_MEZZANINE

`define CSR_BASE   `ADC1_OFFSET + `ADDR_FMC_ADC_MEZZANINE_MMAP_FMC_ADC_100M_CSR
`define CH1_BASE   `CSR_BASE   + `ADDR_FMC_ADC_100MS_CSR_FMC_ADC_CH1
`define CH2_BASE   `CSR_BASE   + `ADDR_FMC_ADC_100MS_CSR_FMC_ADC_CH2
`define CH3_BASE   `CSR_BASE   + `ADDR_FMC_ADC_100MS_CSR_FMC_ADC_CH3
`define CH4_BASE   `CSR_BASE   + `ADDR_FMC_ADC_100MS_CSR_FMC_ADC_CH4
`define EIC_BASE   `ADC1_OFFSET + `ADDR_FMC_ADC_MEZZANINE_MMAP_FMC_ADC_EIC
`define TAG_BASE   `ADC1_OFFSET + `ADDR_FMC_ADC_MEZZANINE_MMAP_TIMETAG_CORE

module main;

   reg rst_n = 0;

   reg clk_125m_pllref = 0;

   initial begin
      repeat(20) @(posedge clk_125m_pllref);
      rst_n = 1;
   end

   IVME64X VME(rst_n);

   `DECLARE_VME_BUFFERS(VME.slave);

   reg[1:0] adc_dco = 2'b00;
   reg[1:0] adc_fr = 2'b00;
   reg[1:0] ext_trig = 2'b00;
   reg adc_data_dir = 1'b0;

   reg[7:0] adc_dat_odd  = 8'h00;
   reg[7:0] adc_dat_even = 8'h00;
   reg signed [13:0] adc_data = 0;

   logic             ddr_reset_n [1:0];
   logic             ddr_ck_p    [1:0];
   logic             ddr_ck_n    [1:0];
   logic             ddr_cke     [1:0];
   logic             ddr_ras_n   [1:0];
   logic             ddr_cas_n   [1:0];
   logic             ddr_we_n    [1:0];
   wire  [1:0]       ddr_dm      [1:0];
   logic [5:0]       ddr_ba;
   logic [27:0]      ddr_a;
   wire  [31:0]      ddr_dq;
   wire  [1:0]       ddr_dqs_p   [1:0];
   wire  [1:0]       ddr_dqs_n   [1:0];
   wire              ddr_rzq     [1:0];
   logic             ddr_odt     [1:0];

   // 400Mhz
   always #1.25ns adc_dco <= ~adc_dco;

   // 125Mhz
   always #4ns clk_125m_pllref <= ~clk_125m_pllref;

   bit [4:0] slot_id = 8;

   svec_ref_fmc_adc_100Ms
     #(
       .g_SIMULATION(1)
       )
   DUT
     (
      .clk_125m_pllref_p_i      (clk_125m_pllref),
      .clk_125m_pllref_n_i      (~clk_125m_pllref),
      .clk_125m_gtp_p_i         (clk_125m_pllref),
      .clk_125m_gtp_n_i         (~clk_125m_pllref),
      .rst_n_i                  (rst_n),
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
      .vme_as_n_i               (VME_AS_n),
      .vme_sysreset_n_i         (VME_RST_n),
      .vme_write_n_i            (VME_WRITE_n),
      .vme_am_i                 (VME_AM),
      .vme_ds_n_i               (VME_DS_n),
      .vme_gap_i                (^slot_id),
      .vme_ga_i                 (~slot_id),
      .vme_berr_o               (VME_BERR),
      .vme_dtack_n_o            (VME_DTACK_n),
      .vme_retry_n_o            (VME_RETRY_n),
      .vme_retry_oe_o           (VME_RETRY_OE),
      .vme_lword_n_b            (VME_LWORD_n),
      .vme_addr_b               (VME_ADDR),
      .vme_data_b               (VME_DATA),
      .vme_irq_o                (VME_IRQ_n),
      .vme_iack_n_i             (VME_IACK_n),
      .vme_iackin_n_i           (VME_IACKIN_n),
      .vme_iackout_n_o          (VME_IACKOUT_n),
      .vme_dtack_oe_o           (VME_DTACK_OE),
      .vme_data_dir_o           (VME_DATA_DIR),
      .vme_data_oe_n_o          (VME_DATA_OE_N),
      .vme_addr_dir_o           (VME_ADDR_DIR),
      .vme_addr_oe_n_o          (VME_ADDR_OE_N),
      .ddr4_reset_n_o           (ddr_reset_n[0]),
      .ddr4_ck_p_o              (ddr_ck_p[0]),
      .ddr4_ck_n_o              (ddr_ck_n[0]),
      .ddr4_cke_o               (ddr_cke[0]),
      .ddr4_ras_n_o             (ddr_ras_n[0]),
      .ddr4_cas_n_o             (ddr_cas_n[0]),
      .ddr4_we_n_o              (ddr_we_n[0]),
      .ddr4_udm_o               (ddr_dm[1][0]),
      .ddr4_ldm_o               (ddr_dm[0][0]),
      .ddr4_ba_o                (ddr_ba[2:0]),
      .ddr4_a_o                 (ddr_a[13:0]),
      .ddr4_dq_b                (ddr_dq[15:0]),
      .ddr4_udqs_p_b            (ddr_dqs_p[1][0]),
      .ddr4_udqs_n_b            (ddr_dqs_n[1][0]),
      .ddr4_ldqs_p_b            (ddr_dqs_p[0][0]),
      .ddr4_ldqs_n_b            (ddr_dqs_n[0][0]),
      .ddr4_odt_o               (ddr_odt[0]),
      .ddr4_rzq_b               (ddr_rzq[0]),
      .ddr5_reset_n_o           (ddr_reset_n[1]),
      .ddr5_ck_p_o              (ddr_ck_p[1]),
      .ddr5_ck_n_o              (ddr_ck_n[1]),
      .ddr5_cke_o               (ddr_cke[1]),
      .ddr5_ras_n_o             (ddr_ras_n[1]),
      .ddr5_cas_n_o             (ddr_cas_n[1]),
      .ddr5_we_n_o              (ddr_we_n[1]),
      .ddr5_udm_o               (ddr_dm[1][1]),
      .ddr5_ldm_o               (ddr_dm[0][1]),
      .ddr5_ba_o                (ddr_ba[5:3]),
      .ddr5_a_o                 (ddr_a[27:14]),
      .ddr5_dq_b                (ddr_dq[31:16]),
      .ddr5_udqs_p_b            (ddr_dqs_p[1][1]),
      .ddr5_udqs_n_b            (ddr_dqs_n[1][1]),
      .ddr5_ldqs_p_b            (ddr_dqs_p[0][1]),
      .ddr5_ldqs_n_b            (ddr_dqs_n[0][1]),
      .ddr5_odt_o               (ddr_odt[1]),
      .ddr5_rzq_b               (ddr_rzq[1])
      );

   ddr3 #
     (
      .DEBUG(0),
      .check_strict_timing(0),
      .check_strict_mrbits(0)
      )
     cmp_ddr0
       (
	.rst_n   (ddr_reset_n[0]),
	.ck      (ddr_ck_p[0]),
	.ck_n    (ddr_ck_n[0]),
	.cke     (ddr_cke[0]),
	.cs_n    (1'b0),
	.ras_n   (ddr_ras_n[0]),
	.cas_n   (ddr_cas_n[0]),
	.we_n    (ddr_we_n[0]),
	.dm_tdqs ({ddr_dm[1][0], ddr_dm[0][0]}),
	.ba      (ddr_ba[2:0]),
	.addr    (ddr_a[13:0]),
	.dq      (ddr_dq[15:0]),
	.dqs     ({ddr_dqs_p[1][0],ddr_dqs_p[0][0]}),
	.dqs_n   ({ddr_dqs_n[1][0],ddr_dqs_n[0][0]}),
	.odt     (ddr_odt[0]),
	.tdqs_n  ()
	);

   ddr3 #
     (
      .DEBUG(0),
      .check_strict_timing(0),
      .check_strict_mrbits(0)
      )
     cmp_ddr1
       (
	.rst_n   (ddr_reset_n[1]),
	.ck      (ddr_ck_p[1]),
	.ck_n    (ddr_ck_n[1]),
	.cke     (ddr_cke[1]),
	.cs_n    (1'b0),
	.ras_n   (ddr_ras_n[1]),
	.cas_n   (ddr_cas_n[1]),
	.we_n    (ddr_we_n[1]),
	.dm_tdqs ({ddr_dm[1][1], ddr_dm[0][1]}),
	.ba      (ddr_ba[5:3]),
	.addr    (ddr_a[27:14]),
	.dq      (ddr_dq[31:16]),
	.dqs     ({ddr_dqs_p[1][1],ddr_dqs_p[0][1]}),
	.dqs_n   ({ddr_dqs_n[1][1],ddr_dqs_n[0][1]}),
	.odt     (ddr_odt[1]),
	.tdqs_n  ()
	);

   int adc_div = 0;

   always@(negedge adc_dco[0])
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
   always@(posedge adc_fr[0])
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

   wire[2:0] acq_fsm_state = DUT.gen_fmc_mezzanine[0].cmp_fmc_adc_mezzanine.cmp_fmc_adc_100Ms_core.acq_fsm_state;

   task automatic init_vme64x_core(ref CBusAccessor_VME64x acc);
      uint64_t rv;

      /* map func0 to 0x80000000, A32 */

      acc.write('h7ff63, 'h80, CR_CSR|A32|D08Byte3);
      acc.write('h7ff67, 0,    CR_CSR|A32|D08Byte3);
      acc.write('h7ff6b, 0,    CR_CSR|A32|D08Byte3);
      acc.write('h7ff6f, 36,   CR_CSR|A32|D08Byte3);
      acc.write('h7ff33, 1,    CR_CSR|A32|D08Byte3);
      acc.write('h7fffb, 'h10, CR_CSR|A32|D08Byte3); /* enable module (BIT_SET = 0x10) */

      acc.set_default_modifiers(A32 | D32 | SINGLE);
   endtask // init_vme64x_core

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

      CBusAccessor_VME64x acc;

      acc = new(VME.tb);

      $timeformat (-6, 3, "us", 10);

      #2us;

      init_vme64x_core(acc);

      #1us;

      expected = 'h19;
      acc.read(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_STA, val);
      if (val != expected)
	begin
	   adc_status_print(val);
	   $fatal (1, "ADC status error (got 0x%8x, expected 0x%8x).", val, expected);
	end

      // FMC-ADC core general configuration
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_PRE_SAMPLES,  'h00000000);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_POST_SAMPLES, 'h00000001);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_SHOTS,        'h00000001);

      // FMC-ADC core channel configuration
      acc.write(`CH1_BASE + `ADDR_FMC_ADC_100MS_CHANNEL_REGS_CALIB, 'h00008000);
      acc.write(`CH2_BASE + `ADDR_FMC_ADC_100MS_CHANNEL_REGS_CALIB, 'h00008000);
      acc.write(`CH3_BASE + `ADDR_FMC_ADC_100MS_CHANNEL_REGS_CALIB, 'h00008000);
      acc.write(`CH4_BASE + `ADDR_FMC_ADC_100MS_CHANNEL_REGS_CALIB, 'h00008000);
      acc.write(`CH1_BASE + `ADDR_FMC_ADC_100MS_CHANNEL_REGS_SAT,   'h00007fff);
      acc.write(`CH2_BASE + `ADDR_FMC_ADC_100MS_CHANNEL_REGS_SAT,   'h00007fff);
      acc.write(`CH3_BASE + `ADDR_FMC_ADC_100MS_CHANNEL_REGS_SAT,   'h00007fff);
      acc.write(`CH4_BASE + `ADDR_FMC_ADC_100MS_CHANNEL_REGS_SAT,   'h00007fff);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CTL, `FMC_ADC_100MS_CSR_CTL_CALIB_APPLY);

      // FMC-ADC core trigger configuration
      val = (16'h100 << `FMC_ADC_100MS_CHANNEL_REGS_TRIG_THRES_HYST_OFFSET) |
	    (16'h300 << `FMC_ADC_100MS_CHANNEL_REGS_TRIG_THRES_VAL_OFFSET);
      acc.write(`CH1_BASE + `ADDR_FMC_ADC_100MS_CHANNEL_REGS_TRIG_THRES, val);
      acc.write(`CH2_BASE + `ADDR_FMC_ADC_100MS_CHANNEL_REGS_TRIG_THRES, val);
      acc.write(`CH3_BASE + `ADDR_FMC_ADC_100MS_CHANNEL_REGS_TRIG_THRES, val);
      acc.write(`CH4_BASE + `ADDR_FMC_ADC_100MS_CHANNEL_REGS_TRIG_THRES, val);
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

      acc.write(`TAG_BASE + 0, 'h00000032); // timetag core seconds high
      acc.write(`TAG_BASE + 4, 'h00005a34); // timetag core seconds low
      acc.write(`TAG_BASE + 8, 'h00000000); // timetag core ticks

      wait (acq_fsm_state == 1);
      $display("<%t> START ACQ 1/4", $realtime);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CTL, 'h00000001); // FSM start

      #200ns;

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_SW_TRIG, 'hFFFFFFFF); // soft trigger

      wait (acq_fsm_state == 1);
      $display("<%t> END ACQ 1/4", $realtime);

      #100ns;

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_PRE_SAMPLES,  'h00000004);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_POST_SAMPLES, 'h00000008);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_SHOTS,        'h00000001);

      val = (1'b1    << `FMC_ADC_100MS_CSR_TRIG_EN_CH1_OFFSET);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_TRIG_EN, val);

      $display("<%t> START ACQ 1b/4", $realtime);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CTL, 'h00000001); // FSM sta

      #1us;
      wait (acq_fsm_state == 1);
      $display("<%t> END ACQ 1b/4", $realtime);

      //  TODO: check results (they are only displayed)
      acc.read(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_TRIG_POS, val);
      $display("trig pos: %x", val);
      acc.write(`VME_OFFSET + 16'h58, val);
      for(i = 0; i < 8*2 + 4; i+=1) begin
         acc.read(`VME_OFFSET + 16'h2000, val);
         $display("%d: %08x", i, val);
      end

      $stop;
      
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

      wait (acq_fsm_state == 1);
      $display("<%t> END ACQ 2/4", $realtime);

      #1us;

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_SHOTS, 'h0000008);

      // FMC-ADC core trigger configuration
      val = (1'b1    << `FMC_ADC_100MS_CSR_TRIG_EN_SW_OFFSET)  |
	    (1'b1    << `FMC_ADC_100MS_CSR_TRIG_EN_CH1_OFFSET) |
	    (1'b1    << `FMC_ADC_100MS_CSR_TRIG_EN_CH3_OFFSET);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_TRIG_EN, val);

      $display("<%t> START ACQ 3/4", $realtime);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CTL, 'h00000001); // FSM start

      #1us;

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_SW_TRIG, 'hFFFFFFFE); // soft trigger

      #1us;

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_SW_TRIG, 'hFFFFFFFD); // soft trigger

      wait (acq_fsm_state == 1);
      $display("<%t> END ACQ 3/4", $realtime);

      #1us;

      // set time trigger
      acc.write(`TAG_BASE + 'h0c, 'h00000032); // timetag core seconds high
      acc.write(`TAG_BASE + 'h10, 'h00005a34); // timetag core seconds low
      acc.write(`TAG_BASE + 'h14, 'h00001100); // timetag core ticks

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_PRE_SAMPLES,  'h00000010);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_POST_SAMPLES, 'h00000080);

      // FMC-ADC core trigger configuration
      val = (1'b1 << `FMC_ADC_100MS_CSR_TRIG_EN_TIME_OFFSET) |
	    (1'b1 << `FMC_ADC_100MS_CSR_TRIG_EN_EXT_OFFSET);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_TRIG_EN, val);

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_EXT_TRIG_DLY, 3);

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_SHOTS, 'h0000002);

      $display("<%t> START ACQ 4/4", $realtime);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CTL, 'h00000001); // FSM start

      #1us;

      ext_trig <= 1'b1;
      #10ns;
      ext_trig <= 1'b0;
      #10ns;
      ext_trig <= 1'b1;
      #10ns;
      ext_trig <= 1'b0;
      #10ns;
      ext_trig <= 1'b1;
      #100ns;
      ext_trig <= 1'b0;

      wait (acq_fsm_state == 1);
      $display("<%t> END ACQ 4/4", $realtime);

      #1us;

      expected = 'h39;
      acc.read(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_STA, val);
      if (val != expected)
	begin
	   adc_status_print(val);
	   $fatal (1, "ADC status error (got 0x%8x, expected 0x%8x).", val, expected);
	end

      $display ("Simulation PASSED");
      $finish;

   end

   initial begin
      // Silence Xilinx unisim DSP48A1 warnings about invalid OPMODE
      force DUT.inst_svec_base.gen_wr.cmp_xwrc_board_svec.cmp_board_common.cmp_xwr_core.
        WRPC.LM32_CORE.gen_profile_medium_icache.U_Wrapped_LM32.cpu.
          multiplier.D1.OPMODE_dly = 0;
      force DUT.inst_svec_base.gen_wr.cmp_xwrc_board_svec.cmp_board_common.cmp_xwr_core.
        WRPC.LM32_CORE.gen_profile_medium_icache.U_Wrapped_LM32.cpu.
          multiplier.D2.OPMODE_dly = 0;
      force DUT.inst_svec_base.gen_wr.cmp_xwrc_board_svec.cmp_board_common.cmp_xwr_core.
        WRPC.LM32_CORE.gen_profile_medium_icache.U_Wrapped_LM32.cpu.
          multiplier.D3.OPMODE_dly = 0;
   end // initial begin

endmodule // main
