`timescale 1ns/1ps

`include "gn4124_bfm.svh"
`include "if_wb_master.svh"
`include "if_wb_slave.svh"



module main;
   reg clk_125m_pllref = 0;
   reg clk_20m_vcxo = 0;
   reg clk_ext = 0;
   
   reg rst_n = 0;
   reg adc0_dco = 0;
   reg adc0_fr = 0;
   

   always #5ns adc0_dco <= ~adc0_dco;
   always #50ns clk_ext <= ~clk_ext;
   always #4ns clk_125m_pllref <= ~clk_125m_pllref;
   always #20ns clk_20m_vcxo <= ~clk_20m_vcxo;

   
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
	      .clk20_vcxo_i(clk_20m_vcxo),
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

    
   int adc_div = 0;
   
   always@(posedge adc0_dco)
     if(adc_div==3)
       begin
	  adc0_fr <= 1;
	  adc_div <= 0;
       end else begin
	  adc0_fr <= 0;
       adc_div <= adc_div + 1;
       end
   
	  
   
   
   initial begin
      CBusAccessor acc;
      uint64_t rv;

      @(posedge I_Gennum.ready);

      acc =      I_Gennum.get_accessor();
      #40us;

      acc.set_default_xfer_size(4);
      
      acc.read(0, rv);

      $display("ID: %x", rv);
      
      acc.write('h100c,'h1000); // host addr
      acc.write('h1010,0);

      acc.write('h1014,'h1000); // len
      acc.write('h1018, 0); // next
      acc.write('h101c,0);

      acc.write('h1008,'h0);
      acc.write('h1020,'h0); // attrib: pcie -> host
      acc.write('h1000,'h1); // xfer start
      
      

      
      
      
   end
   
   
endmodule // main



