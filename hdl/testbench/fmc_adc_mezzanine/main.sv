`timescale 1ns/1ps

`include "vhd_wishbone_master.svh"
`include "fmc_adc_100Ms_csr.v"
`include "timetag_core_regs.v"

`define SDB_ADDR 'h0000
`define CSR_BASE 'h1000
`define TAG_BASE 'h1900

module main;

   reg rst_n = 0;
   reg clk_sys = 0;

   reg adc0_dco = 0;
   reg adc0_fr = 1'b0;
   reg ext_trig = 1'b0;
   reg adc_data_dir = 1'b0;

   reg[3:0] adc0_dat_odd  = 4'h0;
   reg[3:0] adc0_dat_even = 4'h0;
   reg signed [13:0] adc0_data = 0;

   // 400Mhz
   always #1.25ns adc0_dco <= ~adc0_dco;

   // 125Mhz
   always #4ns clk_sys <= ~clk_sys;

   initial begin
      repeat(20) @(posedge clk_sys);
      rst_n = 1;
   end

   IVHDWishboneMaster Host ( clk_sys, rst_n );
   IVHDWishboneMaster Trigin ( clk_sys, rst_n );

   wire t_wishbone_slave_data64_out dummy_wb64_out =
        '{ack: 1'b1, err: 1'b0, rty: 1'b0, stall: 1'b0, dat: 64'bx};

   fmc_adc_mezzanine
     #(
       .g_multishot_ram_size(2048)
       ) DUT (
	      .sys_clk_i           (clk_sys),
	      .sys_rst_n_i         (rst_n),
	      .wb_csr_slave_i      (Host.out),
	      .wb_csr_slave_o      (Host.in),
	      .wb_ddr_clk_i        (clk_sys),
              .wb_ddr_rst_n_i      (rst_n),
	      .wb_ddr_master_i     (dummy_wb64_out),
              .wb_ddr_master_o     (),
	      .ddr_wr_fifo_empty_i (),
	      .trig_irq_o          (),
	      .acq_end_irq_o       (),
	      .eic_irq_o           (),
	      .acq_cfg_ok_o        (),
              .wb_trigin_slave_i   (Trigin.out),
              .wb_trigin_slave_o   (Trigin.in),
	      .ext_trigger_p_i     (ext_trig),
	      .ext_trigger_n_i     (~ext_trig),
	      .adc_dco_p_i         (adc0_dco),
	      .adc_dco_n_i         (~adc0_dco),
	      .adc_fr_p_i          (~adc0_fr),
	      .adc_fr_n_i          (adc0_fr),
	      .adc_outa_p_i        (adc0_dat_odd),
	      .adc_outa_n_i        (~adc0_dat_odd),
	      .adc_outb_p_i        (adc0_dat_even),
	      .adc_outb_n_i        (~adc0_dat_even),
	      .gpio_dac_clr_n_o    (),
	      .gpio_led_acq_o      (),
	      .gpio_led_trig_o     (),
	      .gpio_ssr_ch1_o      (),
	      .gpio_ssr_ch2_o      (),
	      .gpio_ssr_ch3_o      (),
	      .gpio_ssr_ch4_o      (),
	      .gpio_si570_oe_o     (),
	      .spi_din_i           (),
	      .spi_dout_o          (),
	      .spi_sck_o           (),
	      .spi_cs_adc_n_o      (),
	      .spi_cs_dac1_n_o     (),
	      .spi_cs_dac2_n_o     (),
	      .spi_cs_dac3_n_o     (),
	      .spi_cs_dac4_n_o     (),
	      .si570_scl_b         (),
	      .si570_sda_b         (),
	      .mezz_one_wire_b     (),
	      .sys_scl_b           (),
	      .sys_sda_b           (),
	      .wr_tm_link_up_i     (),
	      .wr_tm_time_valid_i  (),
	      .wr_tm_tai_i         (),
	      .wr_tm_cycles_i      (),
	      .wr_enable_i         ());

   int adc_div = 0;

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

   //  Generate a triangular waveform on all channels.
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

   wire[2:0] acq_fsm_state = DUT.cmp_fmc_adc_100Ms_core.acq_fsm_state;

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

      CWishboneAccessor acc;
      uint64_t val, expected;

      $timeformat (-6, 3, "us", 10);

      acc = Host.get_accessor();
      acc.set_mode(PIPELINED);

      acc.set_mode(PIPELINED);

      #1us;

      expected = 'h5344422d;
      acc.read(`SDB_ADDR, val);
      if (val != expected)
	$fatal (1, "Unable to detect SDB header at offset 0x%8x.", `SDB_ADDR);

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

      acc.write(`TAG_BASE + `ADDR_TIMETAG_CORE_SECONDS_UPPER, 'h00000032); // timetag core seconds high
      acc.write(`TAG_BASE + `ADDR_TIMETAG_CORE_SECONDS_LOWER, 'h00005a34); // timetag core seconds low
      acc.write(`TAG_BASE + `ADDR_TIMETAG_CORE_COARSE, 'h00000000); // timetag core ticks

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
      $display("<%t> END ACQ 3", $realtime);

      #1us;

      // set time trigger
      acc.write(`TAG_BASE + `ADDR_TIMETAG_CORE_TIME_TRIG_SECONDS_UPPER,
                'h00000032); // timetag core seconds high
      acc.write(`TAG_BASE + `ADDR_TIMETAG_CORE_TIME_TRIG_SECONDS_LOWER,
                'h00005a34); // timetag core seconds low
      acc.write(`TAG_BASE + `ADDR_TIMETAG_CORE_TIME_TRIG_COARSE,
                'h00000e00); // timetag core ticks

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

/* -----\/----- EXCLUDED -----\/-----
      // DMA transfer
      acc.write('h100C, 'h00001000); // host addr
      acc.write('h1010, 'h00000000);

      acc.write('h1014, 'h00000100); // len

      acc.write('h1018, 'h00000000); // next
      acc.write('h101C, 'h00000000);

      acc.write('h1008, 'h00000000);

      acc.write('h1020, 'h00000000); // attrib: pcie -> host

      acc.write('h1000, 'h00000001); // xfer start
 -----/\----- EXCLUDED -----/\----- */

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
