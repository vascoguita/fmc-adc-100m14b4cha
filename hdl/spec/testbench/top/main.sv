`timescale 1ns/1ps

`include "gn4124_bfm.svh"
`include "if_wb_master.svh"
`include "if_wb_slave.svh"



module main;
   reg clk_125m_pllref_p = 0;
   reg clk_125m_pllref_n = 1;

   reg rst_n = 0;
   reg adc0_dco = 0;
   reg adc0_fr = 0;


   always #1.25ns adc0_dco <= ~adc0_dco;
   always #4ns clk_125m_pllref_p <= ~clk_125m_pllref_p;
   always #4ns clk_125m_pllref_n <= ~clk_125m_pllref_n;


   IGN4124PCIMaster I_Gennum ();

   wire ddr_cas_n, ddr_ck_p, ddr_ck_n, ddr_cke;
   wire ddr_ldm, ddr_ldqs_p, ddr_ldqs_n, ddr_odt, ddr_ras_n, ddr_reset_n;
   wire ddr_udm, ddr_udqs_n, ddr_udqs_p, ddr_we_n;
   wire [15:0] ddr_dq;
   wire [13:0] ddr_a;
   wire [2:0]  ddr_ba;
   wire        ddr_zio, ddr_rzq;

   pulldown(ddr_rzq);

   spec_top_fmc_adc_100Ms
     #(
       .g_simulation("TRUE"),
       .g_calib_soft_ip("FALSE")
       ) DUT (
	      .clk_125m_pllref_p_i(clk_125m_pllref_p),
	      .clk_125m_pllref_n_i(clk_125m_pllref_n),
	      .adc0_dco_p_i(adc0_dco),
	      .adc0_dco_n_i(~adc0_dco),
	      .adc0_fr_p_i(adc0_fr),
	      .adc0_fr_n_i(~adc0_fr),

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
	      .DDR3_ZIO     (ddr_zio),
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

   always@(posedge adc0_dco)
     if(adc_div==1) begin
	adc0_fr <= ~adc0_fr;
	adc_div <= 0;
     end
     else begin
	adc_div <= adc_div + 1;
     end




   initial begin
      CBusAccessor acc;
      uint64_t val;

      @(posedge I_Gennum.ready);

      acc = I_Gennum.get_accessor();

      acc.set_default_xfer_size(4);

      @(posedge DUT.sys_clk_pll_locked);

      #15us;

      acc.read(0, val);
      $display("ID: %x", val);

      acc.read('h3304, val); // status
      $display("STATUS: %x", val);

      acc.write('h3308, 'h00000008); // trigger cfg: enable sw trigger
      acc.write('h3328, 'h00000000); // #pre-samples
      acc.write('h332C, 'h00000010); // #post-samples
      acc.write('h3314, 'h00000001); // #nshots: single-shot acq

      acc.read('h3304, val); // status
      $display("STATUS: %x", val);

      #5us;
      acc.write('h3600, 'h00000032); // timetag core seconds high
      acc.write('h3604, 'h00005a34); // timetag core seconds low
      acc.write('h3608, 'h00000000); // timetag core ticks
      
      
      acc.write('h3300, 'h00000001); // FSM start

      #1us;

      acc.write('h3310, 'hFFFFFFFF); // soft trigger

      #2us;

      acc.write('h3314, 'h00000003); // #nshots: 3x multi-shot acq

      acc.write('h3300, 'h00000001); // FSM start

      #1us;

      acc.write('h3310, 'hFFFFFFFE); // soft trigger

      #1us;

      acc.write('h3310, 'hFFFFFFFD); // soft trigger

      #1us;

      acc.write('h3310, 'hFFFFFFFC); // soft trigger

      #2us;

      // DMA transfer
      acc.write('h100C, 'h00001000); // host addr
      acc.write('h1010, 'h00000000);

      acc.write('h1014, 'h00001000); // len

      acc.write('h1018, 'h00000000); // next
      acc.write('h101C, 'h00000000);

      acc.write('h1008, 'h00000000);

      acc.write('h1020, 'h00000000); // attrib: pcie -> host

      acc.write('h1000, 'h00000001); // xfer start

      acc.read('h3304, val); // status
      $display("STATUS: %x", val);

   end


endmodule // main
