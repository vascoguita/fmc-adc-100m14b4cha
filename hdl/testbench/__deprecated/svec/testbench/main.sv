`include "vme64x_bfm.svh"
`include "svec_vme_buffers.svh"
`include "fmc.svh"
`include "ddr3.svh"

module main;

   reg rst_n = 0;
   reg clk_125m_pllref_p = 0;
   reg clk_125m_pllref_n = 1;

   initial begin
      repeat(20) @(posedge clk_125m_pllref_p);
      rst_n = 1;
   end

   IVME64X VME(rst_n);

   `DECLARE_VME_BUFFERS(VME.slave);

   `DECLARE_DDR(0);
   `DECLARE_DDR(1);

   `DECLARE_FMC(0);
   `DECLARE_FMC(1);

   logic [1:0] fp_led_line_oen;
   logic [1:0] fp_led_line;
   logic [3:0] fp_led_column;
   wire        carrier_scl;
   wire        carrier_sda;
   wire        carrier_one_wire;

   logic [7:0] adc_frame = 'h0F;

   always #1250ps adc0_dco_p <= ~adc0_dco_p;
   always #4ns clk_125m_pllref_p <= ~clk_125m_pllref_p;
   always #4ns clk_125m_pllref_n <= ~clk_125m_pllref_n;

   typedef struct {
                   rand bit [15:0] data;
                   } adc_channel_t;

   adc_channel_t adc0_channels[4];
   int         i, j;

   always begin
      for(i=0; i<8; i++)
        begin
           @(posedge adc0_dco_p);
           #625ps;
           for(j=0; j<4; j++)
             begin
                std::randomize(adc0_channels);
                //$display("FMC0: ch%d=0x%x\n", j, adc0_channels[j].data);
                adc0_outa_p[j] = adc0_channels[j].data[2*i+1];
                adc0_outb_p[j] = adc0_channels[j].data[2*i];
             end
           adc0_fr_p = adc_frame[i];
        end // for (i=0; i<8; i++)
   end

   assign adc0_dco_n = ~adc0_dco_p;
   assign adc0_fr_n = ~adc0_fr_p;
   assign adc0_outa_n = ~adc0_outa_p;
   assign adc0_outb_n = ~adc0_outb_p;


   svec_top_fmc_adc_100Ms
     #(
       .g_SIMULATION("TRUE"),
       .g_CALIB_SOFT_IP("FALSE")
       )
   DUT
     (
      .clk_125m_pllref_p_i(clk_125m_pllref_p),
      .clk_125m_pllref_n_i(clk_125m_pllref_n),
      .rst_n_i(rst_n),

      .fp_led_line_oen_o(fp_led_line_oen),
      .fp_led_line_o(fp_led_line),
      .fp_led_column_o(fp_led_column),
      .carrier_scl_b(carrier_scl),
      .carrier_sda_b(carrier_sda),
      .pcbrev_i(5'b00001),
      .carrier_one_wire_b(carrier_one_wire),

      `WIRE_DDR(0)
      `WIRE_DDR(1)

      `WIRE_FMC(0)
      `WIRE_FMC(1)

      `WIRE_VME_PINS(8) // slot number in parameter
      );

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


   initial begin
      uint64_t d;
      uint32_t wr_data;
      uint64_t blt_addr[];
      uint64_t blt_data[];

      int i, result;

      CBusAccessor_VME64x acc;
      
      acc = new(VME.master);


      #20us;
      init_vme64x_core(acc);

      $display("Release FMC0/1 reset\n");
      acc.write('h120C, 'h0, A32|SINGLE|D32);

      // Trigger setup (time trigger)
      $display("Trigger setup\n");
      acc.write('h3308, 'hA, A32|SINGLE|D32);

      // Acquisition setup
      $display("Acquisition setup\n");
      acc.write('h3328, 'h1, A32|SINGLE|D32); // 1 pre-trigger samples
      acc.write('h332c, 'hA, A32|SINGLE|D32); // 10 post-trigger samples
      acc.write('h3314, 'h1, A32|SINGLE|D32); // 1 shot

      // Make sure no acquisition is running
      acc.write('h3300, 'h2, A32|SINGLE|D32); // Send STOP command

      #2.5us;
      acc.write('h1208, 'h10000);
      #0.5us;
      acc.write('h1208, 'h00000);
      #2.0us;      
      
      acc.write('h3600, 'h00000032); // timetag core seconds high
      acc.write('h3604, 'h00005a34); // timetag core seconds low
      acc.write('h3608, 'h07735930); // timetag core ticks

      // Start acquisition
      $display("Start acquisition\n");
      acc.write('h3300, 'h1, A32|SINGLE|D32); // Send START command

      
      // Start acquisition
      $display("Schedule time trigger\n");
      acc.write('h360C, 'h00000032); // time trigger seconds high
      acc.write('h3610, 'h00005a35); // time trigger seconds low
      acc.write('h3614, 'h00000300); // time trigger core ticks

      // Sw trigger
/* -----\/----- EXCLUDED -----\/-----
      #1us
      $display("Set trigger time\n");
      acc.write('h3310, 'hFF, A32|SINGLE|D32);
 -----/\----- EXCLUDED -----/\----- */

      #20us;
      
      /*
      // Data "FIFO" test
      acc.write('h2200, 'h0, A32|SINGLE|D32);
      acc.read('h2200, d, A32|SINGLE|D32);
      $display("Read DDR_ADR: 0x%x\n", d);

      $display("Write data to DDR in BLT\n");
      blt_addr = {'h3000};
      blt_data = {'h1, 'h2, 'h3, 'h4, 'h5, 'h6, 'h7, 'h8 ,'h9, 'hA};
      acc.writem(blt_addr, blt_data, A32|BLT|D32, result);

      acc.write('h2200, 'h0, A32|SINGLE|D32);
      acc.read('h2200, d, A32|SINGLE|D32);
      $display("Read DDR_ADR: 0x%x\n", d);

      $display("Read data from DDR in BLT");
      blt_data = {};
      acc.readm(blt_addr, blt_data, A32|BLT|D32, result);

      for(i=0; i<10; i++)
        begin
           $display("Data %d: 0x%x\n", i, blt_data[i]);
        end
      */

/* -----\/----- EXCLUDED -----\/-----
      acc.write('h2200, 'h0, A32|SINGLE|D32);
      for(i=0; i<5; i++)
        begin
           acc.read('h3000, d, A32|SINGLE|D32);
           $display("Read %d: 0x%x\n", i, d);
        end

      acc.write('h2200, 'h0, A32|SINGLE|D32);
      for(i=0; i<2; i++)
        begin
           wr_data = i;
           acc.write('h3000, wr_data, A32|SINGLE|D32);
           $display("Write %d: 0x%x\n", i, wr_data);
        end

      acc.write('h2200, 'h0, A32|SINGLE|D32);
      for(i=0; i<5; i++)
        begin
           acc.read('h3000, d, A32|SINGLE|D32);
           $display("Read %d: 0x%x\n", i, d);
        end

 -----/\----- EXCLUDED -----/\----- */

   end


endmodule // main



