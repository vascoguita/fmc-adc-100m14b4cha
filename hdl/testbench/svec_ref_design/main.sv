`timescale 1ns/1ps

`include "vme64x_bfm.svh"
`include "svec_vme_buffers.svh"
`include "fmc_adc_100Ms_csr.v"

`define VME_OFFSET 'h80000000
`define ADC_OFFSET 'h2000

`define SDB_ADDR `VME_OFFSET + 0
`define CSR_BASE `VME_OFFSET + `ADC_OFFSET + 'h1000
`define OWC_BASE `VME_OFFSET + `ADC_OFFSET + 'h1700
`define TAG_BASE `VME_OFFSET + `ADC_OFFSET + 'h1900

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

   // 400Mhz
   always #1.25ns adc_dco <= ~adc_dco;

   // 125Mhz
   always #4ns clk_125m_pllref <= ~clk_125m_pllref;

   bit [4:0] slot_id = 8;

   svec_ref_fmc_adc_100Ms
     #(
       .g_SIMULATION(1),
       .g_CALIB_SOFT_IP("FALSE")
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
      .vme_rst_n_i              (VME_RST_n),
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
      .vme_addr_oe_n_o          (VME_ADDR_OE_N)
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

      acc.write('h7ff63, 'h80, A32|CR_CSR|D08Byte3);
      acc.write('h7ff67, 0, CR_CSR|A32|D08Byte3);
      acc.write('h7ff6b, 0, CR_CSR|A32|D08Byte3);
      acc.write('h7ff6f, 36, CR_CSR|A32|D08Byte3);
      acc.write('h7ff33, 1, CR_CSR|A32|D08Byte3);
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

      expected = 'h5344422d;
      acc.read(`SDB_ADDR, val);
      if (val != expected)
	$fatal (1, "Unable to detect SDB header at offset 0x%8x (got 0x%8x, expected 0x%8x).",
		`SDB_ADDR, val , expected);

      expected = 'h5344422d;
      acc.read(`ADC_OFFSET+`SDB_ADDR, val);
      if (val != expected)
	$fatal (1, "Unable to detect SDB header at offset 0x%8x (got 0x%8x, expected 0x%8x).",
		`ADC_OFFSET+`SDB_ADDR, val , expected);

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
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CH1_GAIN, 'h00008000);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CH2_GAIN, 'h00008000);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CH3_GAIN, 'h00008000);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CH4_GAIN, 'h00008000);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CH1_SAT,  'h00007fff);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CH2_SAT,  'h00007fff);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CH3_SAT,  'h00007fff);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CH4_SAT,  'h00007fff);

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
      acc.write(`TAG_BASE + 'h14, 'h00001000); // timetag core ticks

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


endmodule // main
