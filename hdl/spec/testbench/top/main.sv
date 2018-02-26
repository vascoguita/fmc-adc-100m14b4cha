`timescale 1ns/1ps

`include "gn4124_bfm.svh"
`include "if_wb_master.svh"
`include "if_wb_slave.svh"
`include "fmc_adc_100Ms_csr.v"

`define CSR_BASE 'h3000
`define TAG_BASE 'h3900

module main;
   reg clk_125m_pllref_p = 0;
   reg clk_125m_pllref_n = 1;

   reg clk_125m_gtp_p = 0;
   reg clk_125m_gtp_n = 1;

   reg clk_20m_vcxo = 0;

   reg rst_n = 0;
   reg adc0_dco = 0;
   reg adc0_fr = 1'b0;
   reg ext_trig = 1'b0;
   reg adc_data_dir = 1'b0;

   reg[3:0] adc0_dat_odd  = 4'h0;
   reg[3:0] adc0_dat_even = 4'h0;
   reg signed [13:0] adc0_data = 0;

   always #1.25ns adc0_dco <= ~adc0_dco;
   always #4ns clk_125m_pllref_p <= ~clk_125m_pllref_p;
   always #4ns clk_125m_pllref_n <= ~clk_125m_pllref_n;
   always #4ns clk_125m_gtp_p <= ~clk_125m_gtp_p;
   always #4ns clk_125m_gtp_n <= ~clk_125m_gtp_n;
   always #25ns clk_20m_vcxo <= ~clk_20m_vcxo;


   IGN4124PCIMaster I_Gennum ();

   wire ddr_cas_n, ddr_ck_p, ddr_ck_n, ddr_cke;
   wire ddr_ldm, ddr_ldqs_p, ddr_ldqs_n, ddr_odt, ddr_ras_n, ddr_reset_n;
   wire ddr_udm, ddr_udqs_n, ddr_udqs_p, ddr_we_n;
   wire [15:0] ddr_dq;
   wire [13:0] ddr_a;
   wire [2:0]  ddr_ba;
   wire        ddr_rzq;
   wire        sfp_txp, sfp_txn, sfp_scl, sfp_sda, sfp_sda_en;

   pulldown(ddr_rzq);

   sfp_i2c_adapter 
     SFP_I2C (
	      .clk_i(clk_125m_pllref_p),
	      .rst_n_i(1'b1),
	      .scl_i(sfp_scl),
	      .sda_i(sfp_sda),
	      .sda_en_o(sfp_sda_en),
	      .sfp_det_valid_i(1'b1),
	      .sfp_data_i(128'h0123456789ABCDEF0123456789ABCDEF)
	      );

   assign sfp_sda = (sfp_sda_en) ? 1'b0:1'bz;
   
   spec_top_fmc_adc_100Ms
     #(
       .g_simulation(1),
       .g_wrpc_initf("../../../ip_cores/wr-cores/bin/wrpc/wrc_phy8_sim.bram"),
       .g_calib_soft_ip("FALSE")
       ) DUT (
	      .button1_n_i(1'b1),
	      .clk_20m_vcxo_i(clk_20m_vcxo),
	      .clk_125m_pllref_p_i(clk_125m_pllref_p),
	      .clk_125m_pllref_n_i(clk_125m_pllref_n),
	      .clk_125m_gtp_p_i(clk_125m_gtp_p),
	      .clk_125m_gtp_n_i(clk_125m_gtp_n),
	      .sfp_txp_o(sfp_txp),
	      .sfp_txn_o(sfp_txn),
	      .sfp_rxp_i(sfp_txp),
	      .sfp_rxn_i(sfp_txn),
	      .sfp_los_i(1'b0),
	      .sfp_tx_fault_i(1'b0),
	      .sfp_mod_def0_i(1'b0),
	      .sfp_mod_def1_b(sfp_scl),
	      .sfp_mod_def2_b(sfp_sda),
	      .adc0_ext_trigger_p_i(ext_trig),
	      .adc0_ext_trigger_n_i(~ext_trig),
	      .adc0_dco_p_i(adc0_dco),
	      .adc0_dco_n_i(~adc0_dco),
	      .adc0_fr_p_i(~adc0_fr),
	      .adc0_fr_n_i(adc0_fr),
	      .adc0_outa_p_i(adc0_dat_odd),
	      .adc0_outa_n_i(~adc0_dat_odd),
	      .adc0_outb_p_i(adc0_dat_even),
	      .adc0_outb_n_i(~adc0_dat_even),
	      .DDR3_CAS_N (ddr_cas_n),
	      .DDR3_CK_N(ddr_ck_n),
	      .DDR3_CK_P  (ddr_ck_p),
	      .DDR3_CKE    (ddr_cke),
	      .DDR3_LDM    (ddr_ldm),
	      .DDR3_LDQS_N (ddr_ldqs_n),
	      .DDR3_LDQS_P (ddr_ldqs_p),
	      .DDR3_ODT    (ddr_odt),
	      .DDR3_RAS_N  (ddr_ras_n),
	      .DDR3_RESET_N (ddr_reset_n),
	      .DDR3_UDM     (ddr_udm),
	      .DDR3_UDQS_N  (ddr_udqs_n),
	      .DDR3_UDQS_P  (ddr_udqs_p),
	      .DDR3_WE_N    (ddr_we_n),
	      .DDR3_DQ     (ddr_dq),
	      .DDR3_A       (ddr_a),
	      .DDR3_BA      (ddr_ba),
	      .DDR3_RZQ     (ddr_rzq),


	      `GENNUM_WIRE_SPEC_PINS(I_Gennum)
	      );

   ddr3 #(
	  .DEBUG(1)
	  ) mem (
		 .rst_n(ddr_reset_n),
		 .ck(ddr_ck_p),
		 .ck_n(ddr_ck_n),
		 .cke(ddr_cke),
		 .cs_n(1'b0),
		 .ras_n(ddr_ras_n),
		 .cas_n(ddr_cas_n),
		 .we_n(ddr_we_n),
		 .dm_tdqs({ddr_udm, ddr_ldm}),
		 .ba(ddr_ba),
		 .addr(ddr_a),
		 .dq(ddr_dq),
		 .dqs({ddr_udqs_p, ddr_ldqs_p}),
		 .dqs_n({ddr_udqs_n, ddr_ldqs_n}),
		 .tdqs_n(),
		 .odt(ddr_odt)
		 );


   int	       adc_div = 0;

   always@(negedge adc0_dco)
     begin
	#625ps;
	if(adc_div == 1) begin
	   adc0_fr <= ~adc0_fr;
	   adc_div <= 0;
	end
	else begin
	   adc_div <= adc_div + 1;
	end
     end

   always@(posedge adc0_fr)
     begin
	if ((adc0_data > 400) || (adc0_data < -400)) begin
	   adc_data_dir = ~adc_data_dir;
	end
	if (adc_data_dir == 0) begin
	   adc0_data = adc0_data + 8;
	end
	else begin
	   adc0_data = adc0_data - 8;
	end
	adc0_dat_odd  = {4{adc0_data[13]}};
	adc0_dat_even = {4{adc0_data[12]}};
	#1250ps;
	adc0_dat_odd  = {4{adc0_data[11]}};
	adc0_dat_even = {4{adc0_data[10]}};
	#1250ps;
	adc0_dat_odd  = {4{adc0_data[9]}};
	adc0_dat_even = {4{adc0_data[8]}};
	#1250ps;
	adc0_dat_odd  = {4{adc0_data[7]}};
	adc0_dat_even = {4{adc0_data[6]}};
	#1250ps;
	adc0_dat_odd  = {4{adc0_data[5]}};
	adc0_dat_even = {4{adc0_data[4]}};
	#1250ps;
	adc0_dat_odd  = {4{adc0_data[3]}};
	adc0_dat_even = {4{adc0_data[2]}};
	#1250ps;
	adc0_dat_odd  = {4{adc0_data[1]}};
	adc0_dat_even = {4{adc0_data[0]}};
	#1250ps;
	adc0_dat_odd  = {4{1'b0}};
	adc0_dat_even = {4{1'b0}};
     end

   wire[2:0] acq_fsm_state = DUT.cmp_fmc_adc_mezzanine_0.cmp_fmc_adc_100Ms_core.acq_fsm_state;

   initial begin
      CBusAccessor acc;
      uint64_t val;

      @(posedge I_Gennum.ready);

      acc = I_Gennum.get_accessor();

      acc.set_default_xfer_size(4);

      //@(posedge DUT.sys_clk_pll_locked);

      #15us;

      acc.read(0, val);
      $display("ID: %x", val);

      // FMC software reset
      acc.write('h120c, 'h00000001);
      #1us;
      acc.write('h120c, 'h00000000);

      acc.read(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_STA, val); // status
      $display("STATUS: %x", val);

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

      acc.read(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_STA, val);
      $display("STATUS: %x", val);

      #5us;

      acc.write(`TAG_BASE + 0, 'h00000032); // timetag core seconds high
      acc.write(`TAG_BASE + 4, 'h00005a34); // timetag core seconds low
      acc.write(`TAG_BASE + 8, 'h00000000); // timetag core ticks

      wait (acq_fsm_state == 1);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CTL, 'h00000001); // FSM start

      #1us;

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_SW_TRIG, 'hFFFFFFFF); // soft trigger

      wait (acq_fsm_state == 1);

      #1us;

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_SHOTS, 'h00000003); // #nshots: 3x multi-shot acq

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CTL, 'h00000001); // FSM start

      #1us;

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_SW_TRIG, 'hFFFFFFFE); // soft trigger

      #1us;

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_SW_TRIG, 'hFFFFFFFD); // soft trigger

      #1us;

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_SW_TRIG, 'hFFFFFFFC); // soft trigger

      wait (acq_fsm_state == 1);

      #1us;

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_SHOTS, 'h0000008);

      // FMC-ADC core trigger configuration
      val = (1'b1    << `FMC_ADC_100MS_CSR_TRIG_EN_SW_OFFSET)  |
	    (1'b1    << `FMC_ADC_100MS_CSR_TRIG_EN_CH1_OFFSET) |
	    (1'b1    << `FMC_ADC_100MS_CSR_TRIG_EN_CH3_OFFSET);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_TRIG_EN, val);

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CTL, 'h00000001); // FSM start

      #1us;

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_SW_TRIG, 'hFFFFFFFE); // soft trigger

      #1us;

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_SW_TRIG, 'hFFFFFFFD); // soft trigger

      wait (acq_fsm_state == 1);

      #1us;

      // set time trigger
      acc.write(`TAG_BASE + 'h0c, 'h00000032); // timetag core seconds high
      acc.write(`TAG_BASE + 'h10, 'h00005a34); // timetag core seconds low
      acc.write(`TAG_BASE + 'h14, 'h00000e00); // timetag core ticks

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_PRE_SAMPLES,  'h00000010);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_POST_SAMPLES, 'h00000080);

      // FMC-ADC core trigger configuration
      val = (1'b1 << `FMC_ADC_100MS_CSR_TRIG_EN_TIME_OFFSET) |
	    (1'b1 << `FMC_ADC_100MS_CSR_TRIG_EN_EXT_OFFSET);
      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_TRIG_EN, val);

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_EXT_TRIG_DLY, 3);

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_SHOTS, 'h0000002);

      acc.write(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_CTL, 'h00000001); // FSM start

      #5us;

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

      #1us;

      // DMA transfer
      acc.write('h100C, 'h00001000); // host addr
      acc.write('h1010, 'h00000000);

      acc.write('h1014, 'h00000100); // len

      acc.write('h1018, 'h00000000); // next
      acc.write('h101C, 'h00000000);

      acc.write('h1008, 'h00000000);

      acc.write('h1020, 'h00000000); // attrib: pcie -> host

      acc.write('h1000, 'h00000001); // xfer start

      acc.read(`CSR_BASE + `ADDR_FMC_ADC_100MS_CSR_STA, val);
      $display("STATUS: %x", val);

   end


endmodule // main
