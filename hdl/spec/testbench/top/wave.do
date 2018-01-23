onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/clk_i
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/rst_n_i
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_ctrl_irq_o
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/dma_ctrl_target_addr_i
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/dma_ctrl_host_addr_h_i
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/dma_ctrl_host_addr_l_i
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/dma_ctrl_len_i
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/dma_ctrl_start_l2p_i
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/dma_ctrl_done_o
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/dma_ctrl_error_o
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/dma_ctrl_byte_swap_i
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/dma_ctrl_abort_i
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/ldm_arb_valid_o
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/ldm_arb_dframe_o
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/ldm_arb_data_o
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/ldm_arb_req_o
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/arb_ldm_gnt_i
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/l2p_edb_o
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/l_wr_rdy_i
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/l2p_rdy_i
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/tx_error_i
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/l2p_dma_clk_i
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/l2p_dma_adr_o
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/l2p_dma_dat_i
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/l2p_dma_dat_o
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/l2p_dma_sel_o
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/l2p_dma_cyc_o
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/l2p_dma_stb_o
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/l2p_dma_we_o
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/l2p_dma_ack_i
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/l2p_dma_stall_i
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/p2l_dma_cyc_i
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/fifo_rst_n
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/data_fifo_rd
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/data_fifo_wr
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/data_fifo_empty
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/data_fifo_full
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/data_fifo_dout
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/data_fifo_din
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/addr_fifo_rd
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/addr_fifo_wr
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/addr_fifo_empty
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/addr_fifo_full
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/addr_fifo_dout
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/addr_fifo_din
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/l2p_dma_current_state
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/s_l2p_header
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/l2p_len_cnt
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/l2p_address_h
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/l2p_address_l
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/l2p_data_cnt
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/l2p_64b_address
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/l2p_len_header
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/l2p_byte_swap
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/l2p_last_packet
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/l2p_lbe_header
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/data_fifo_valid
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/addr_fifo_valid
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/target_addr_cnt
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/dma_length_cnt
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/l2p_dma_cyc_t
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/l2p_dma_stb_t
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/wb_ack_cnt
add wave -noupdate -group l2p_dma /main/DUT/cmp_gn4124_core/cmp_l2p_dma_master/wb_read_cnt
#add wave -noupdate -group Top /main/DUT/clk20_vcxo_i
add wave -noupdate -group Top /main/DUT/clk_125m_pllref*
add wave -noupdate -group Top /main/DUT/powerup*
add wave -noupdate -group Top /main/DUT/pll25dac_sync_n_o
add wave -noupdate -group Top /main/DUT/pll20dac_sync_n_o
add wave -noupdate -group Top /main/DUT/plldac_din_o
add wave -noupdate -group Top /main/DUT/plldac_sclk_o
add wave -noupdate -group Top /main/DUT/led_red_o
add wave -noupdate -group Top /main/DUT/led_green_o
add wave -noupdate -group Top /main/DUT/aux_leds_o
#add wave -noupdate -group Top /main/DUT/aux_buttons_i
add wave -noupdate -group Top /main/DUT/pcb_ver_i
add wave -noupdate -group Top /main/DUT/carrier_one_wire_b
add wave -noupdate -group Top /main/DUT/L_RST_N
add wave -noupdate -group Top /main/DUT/P2L_RDY
add wave -noupdate -group Top /main/DUT/P2L_CLKn
add wave -noupdate -group Top /main/DUT/P2L_CLKp
add wave -noupdate -group Top /main/DUT/P2L_DATA
add wave -noupdate -group Top /main/DUT/P2L_DFRAME
add wave -noupdate -group Top /main/DUT/P2L_VALID
add wave -noupdate -group Top /main/DUT/P_WR_RDY
add wave -noupdate -group Top /main/DUT/RX_ERROR
add wave -noupdate -group Top /main/DUT/L2P_DATA
add wave -noupdate -group Top /main/DUT/L2P_DFRAME
add wave -noupdate -group Top /main/DUT/L2P_VALID
add wave -noupdate -group Top /main/DUT/L2P_CLKn
add wave -noupdate -group Top /main/DUT/L2P_CLKp
add wave -noupdate -group Top /main/DUT/L2P_EDB
add wave -noupdate -group Top /main/DUT/L2P_RDY
add wave -noupdate -group Top /main/DUT/L_WR_RDY
add wave -noupdate -group Top /main/DUT/P_RD_D_RDY
add wave -noupdate -group Top /main/DUT/TX_ERROR
add wave -noupdate -group Top /main/DUT/GPIO
add wave -noupdate -group Top /main/DUT/DDR3_CAS_N
add wave -noupdate -group Top /main/DUT/DDR3_CK_N
add wave -noupdate -group Top /main/DUT/DDR3_CK_P
add wave -noupdate -group Top /main/DUT/DDR3_CKE
add wave -noupdate -group Top /main/DUT/DDR3_LDM
add wave -noupdate -group Top /main/DUT/DDR3_LDQS_N
add wave -noupdate -group Top /main/DUT/DDR3_LDQS_P
add wave -noupdate -group Top /main/DUT/DDR3_ODT
add wave -noupdate -group Top /main/DUT/DDR3_RAS_N
add wave -noupdate -group Top /main/DUT/DDR3_RESET_N
add wave -noupdate -group Top /main/DUT/DDR3_UDM
add wave -noupdate -group Top /main/DUT/DDR3_UDQS_N
add wave -noupdate -group Top /main/DUT/DDR3_UDQS_P
add wave -noupdate -group Top /main/DUT/DDR3_WE_N
add wave -noupdate -group Top /main/DUT/DDR3_DQ
add wave -noupdate -group Top /main/DUT/DDR3_A
add wave -noupdate -group Top /main/DUT/DDR3_BA
add wave -noupdate -group Top /main/DUT/DDR3_ZIO
add wave -noupdate -group Top /main/DUT/DDR3_RZQ
add wave -noupdate -group Top /main/DUT/adc0_ext_trigger_p_i
add wave -noupdate -group Top /main/DUT/adc0_ext_trigger_n_i
add wave -noupdate -group Top /main/DUT/adc0_dco_p_i
add wave -noupdate -group Top /main/DUT/adc0_dco_n_i
add wave -noupdate -group Top /main/DUT/adc0_fr_p_i
add wave -noupdate -group Top /main/DUT/adc0_fr_n_i
add wave -noupdate -group Top /main/DUT/adc0_outa_p_i
add wave -noupdate -group Top /main/DUT/adc0_outa_n_i
add wave -noupdate -group Top /main/DUT/adc0_outb_p_i
add wave -noupdate -group Top /main/DUT/adc0_outb_n_i
add wave -noupdate -group Top /main/DUT/adc0_spi_din_i
add wave -noupdate -group Top /main/DUT/adc0_spi_dout_o
add wave -noupdate -group Top /main/DUT/adc0_spi_sck_o
add wave -noupdate -group Top /main/DUT/adc0_spi_cs_adc_n_o
add wave -noupdate -group Top /main/DUT/adc0_spi_cs_dac1_n_o
add wave -noupdate -group Top /main/DUT/adc0_spi_cs_dac2_n_o
add wave -noupdate -group Top /main/DUT/adc0_spi_cs_dac3_n_o
add wave -noupdate -group Top /main/DUT/adc0_spi_cs_dac4_n_o
add wave -noupdate -group Top /main/DUT/adc0_gpio_dac_clr_n_o
add wave -noupdate -group Top /main/DUT/adc0_gpio_led_acq_o
add wave -noupdate -group Top /main/DUT/adc0_gpio_led_trig_o
add wave -noupdate -group Top /main/DUT/adc0_gpio_ssr_ch1_o
add wave -noupdate -group Top /main/DUT/adc0_gpio_ssr_ch2_o
add wave -noupdate -group Top /main/DUT/adc0_gpio_ssr_ch3_o
add wave -noupdate -group Top /main/DUT/adc0_gpio_ssr_ch4_o
add wave -noupdate -group Top /main/DUT/adc0_gpio_si570_oe_o
add wave -noupdate -group Top /main/DUT/adc0_si570_scl_b
add wave -noupdate -group Top /main/DUT/adc0_si570_sda_b
add wave -noupdate -group Top /main/DUT/adc0_one_wire_b
add wave -noupdate -group Top /main/DUT/fmc0_prsnt_m2c_n_i
add wave -noupdate -group Top /main/DUT/fmc0_sys_scl_b
add wave -noupdate -group Top /main/DUT/fmc0_sys_sda_b
add wave -noupdate -group Top /main/DUT/sys_clk_in
add wave -noupdate -group Top /main/DUT/sys_clk_125_buf
add wave -noupdate -group Top /main/DUT/sys_clk_125
add wave -noupdate -group Top /main/DUT/sys_clk_fb
add wave -noupdate -group Top /main/DUT/sys_clk_pll_locked
add wave -noupdate -group Top /main/DUT/ddr_clk
add wave -noupdate -group Top /main/DUT/ddr_clk_buf
add wave -noupdate -group Top /main/DUT/sw_rst_fmc0
add wave -noupdate -group Top /main/DUT/fmc0_rst_n
add wave -noupdate -group Top /main/DUT/cnx_master_out
add wave -noupdate -group Top /main/DUT/cnx_master_in
add wave -noupdate -group Top /main/DUT/cnx_slave_out
add wave -noupdate -group Top /main/DUT/cnx_slave_in
add wave -noupdate -group Top /main/DUT/gn_wb_adr
add wave -noupdate -group Top /main/DUT/dma_ctrl_wb_adr
add wave -noupdate -group Top /main/DUT/wb_dma_adr
add wave -noupdate -group Top /main/DUT/wb_dma_dat_i
add wave -noupdate -group Top /main/DUT/wb_dma_dat_o
add wave -noupdate -group Top /main/DUT/wb_dma_sel
add wave -noupdate -group Top /main/DUT/wb_dma_cyc
add wave -noupdate -group Top /main/DUT/wb_dma_stb
add wave -noupdate -group Top /main/DUT/wb_dma_we
add wave -noupdate -group Top /main/DUT/wb_dma_ack
add wave -noupdate -group Top /main/DUT/wb_dma_stall
add wave -noupdate -group Top /main/DUT/wb_dma_err
add wave -noupdate -group Top /main/DUT/wb_dma_rty
add wave -noupdate -group Top /main/DUT/wb_dma_int
add wave -noupdate -group Top /main/DUT/wb_ddr_adr
add wave -noupdate -group Top /main/DUT/wb_ddr_dat_o
add wave -noupdate -group Top /main/DUT/wb_ddr_sel
add wave -noupdate -group Top /main/DUT/wb_ddr_cyc
add wave -noupdate -group Top /main/DUT/wb_ddr_stb
add wave -noupdate -group Top /main/DUT/wb_ddr_we
add wave -noupdate -group Top /main/DUT/wb_ddr_ack
add wave -noupdate -group Top /main/DUT/wb_ddr_stall
add wave -noupdate -group Top /main/DUT/dma_irq
add wave -noupdate -group Top /main/DUT/dma_irq_p
add wave -noupdate -group Top /main/DUT/trig_irq_p
add wave -noupdate -group Top /main/DUT/acq_end_irq_p
add wave -noupdate -group Top /main/DUT/irq_sources
add wave -noupdate -group Top /main/DUT/irq_to_gn4124
add wave -noupdate -group Top /main/DUT/irq_sources_2_led
add wave -noupdate -group Top /main/DUT/ddr_wr_fifo_empty
add wave -noupdate -group Top /main/DUT/dma_eic_irq
add wave -noupdate -group Top /main/DUT/fmc0_eic_irq
add wave -noupdate -group Top /main/DUT/led_red
add wave -noupdate -group Top /main/DUT/led_green
add wave -noupdate -group Top /main/DUT/gpio_stat
add wave -noupdate -group Top /main/DUT/gpio_ctrl_1
add wave -noupdate -group Top /main/DUT/gpio_ctrl_2
add wave -noupdate -group Top /main/DUT/gpio_ctrl_3
add wave -noupdate -group Top /main/DUT/gpio_led_ctrl
add wave -noupdate -group Top /main/DUT/gn4124_status
add wave -noupdate -group Top /main/DUT/p2l_pll_locked
add wave -noupdate -group Top /main/DUT/ddr3_status
add wave -noupdate -group Top /main/DUT/ddr3_calib_done
add wave -noupdate -group Top /main/DUT/spi_din_t
add wave -noupdate -group Top /main/DUT/spi_ss_t
add wave -noupdate -group Top /main/DUT/carrier_owr_en
add wave -noupdate -group Top /main/DUT/carrier_owr_i
add wave -noupdate -group Top /main/DUT/led_pwm_update_cnt
add wave -noupdate -group Top /main/DUT/led_pwm_update
add wave -noupdate -group Top /main/DUT/led_pwm_val
add wave -noupdate -group Top /main/DUT/led_pwm_val_down
add wave -noupdate -group Top /main/DUT/led_pwm_cnt
add wave -noupdate -group Top /main/DUT/led_pwm
add wave -noupdate -radix hexadecimal -group MEZ /main/DUT/cmp_fmc_adc_mezzanine_0/cnx_*
add wave -noupdate -radix hexadecimal -group ADC -group SERDES /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/cmp_adc_serdes/*
add wave -noupdate -radix hexadecimal -group ADC -group CSR /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/csr_regout
add wave -noupdate -radix hexadecimal -group ADC -group CSR /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/wb_csr_*
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/sys_clk_i
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/sys_rst_n_i
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/wb_csr_adr_i
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/wb_csr_dat_i
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/wb_csr_dat_o
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/wb_csr_cyc_i
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/wb_csr_sel_i
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/wb_csr_stb_i
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/wb_csr_we_i
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/wb_csr_ack_o
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/wb_ddr_clk_i
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/wb_ddr_adr_o
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/wb_ddr_dat_o
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/wb_ddr_sel_o
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/wb_ddr_stb_o
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/wb_ddr_we_o
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/wb_ddr_cyc_o
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/wb_ddr_ack_i
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/wb_ddr_stall_i
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/ext_trigger_p_i
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/ext_trigger_n_i
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/adc_dco_p_i
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/adc_dco_n_i
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/adc_fr_p_i
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/adc_fr_n_i
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/adc_outa_p_i
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/adc_outa_n_i
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/adc_outb_p_i
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/adc_outb_n_i
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/gpio_dac_clr_n_o
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/gpio_led_acq_o
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/gpio_led_trig_o
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/gpio_ssr_ch1_o
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/gpio_ssr_ch2_o
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/gpio_ssr_ch3_o
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/gpio_ssr_ch4_o
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/gpio_si570_oe_o
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/sys_rst
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/fs_rst
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/fs_rst_n
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/dco_clk
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/clk_fb
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/locked_in
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/locked_out
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/serdes_clk
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/fs_clk
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/fs_clk_buf
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/serdes_in_p
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/serdes_in_n
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/serdes_out_raw
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/serdes_out_data
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/data_calibr_in
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/offset_calibr
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/gain_calibr
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/data_calibr_out
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/serdes_out_fr
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/serdes_auto_bitslip
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/serdes_man_bitslip
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/serdes_bitslip
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/serdes_synced
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/bitslip_sreg
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/ext_trig_a
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/ext_trig
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/int_trig
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/int_trig_thres
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/int_trig_data
add wave -noupdate -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/int_trig_over_thres
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/hw_trig
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/hw_trig_t
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/hw_trig_sel
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/hw_trig_en
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/sw_trig
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/sw_trig_t
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/sw_trig_en
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/trig
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/trig_delay
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/trig_delay_cnt
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/trig_d
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/trig_align
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/undersample_factor
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/undersample_cnt
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/undersample_en
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/sync_fifo_din
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/sync_fifo_dout
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/sync_fifo_empty
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/sync_fifo_full
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/sync_fifo_wr
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/sync_fifo_rd
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/sync_fifo_valid
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/gain_calibr
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/offset_calibr
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/acq_fsm_current_state
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/acq_fsm_state
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/fsm_cmd
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/fsm_cmd_wr
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/acq_start
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/acq_stop
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/acq_trig
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/acq_end
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/acq_in_pre_trig
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/acq_in_post_trig
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/samples_wr_en
add wave -noupdate -radix unsigned -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/pre_trig_value
add wave -noupdate -radix unsigned -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/pre_trig_cnt
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/pre_trig_done
add wave -noupdate -radix unsigned -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/post_trig_value
add wave -noupdate -radix unsigned -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/post_trig_cnt
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/post_trig_done
add wave -noupdate -radix unsigned -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/samples_cnt
add wave -noupdate -radix unsigned -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/shots_value
add wave -noupdate -radix unsigned -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/shots_cnt
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/shots_done
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/shots_decr
add wave -noupdate -radix hexadecimal -group ADC /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_fmc_adc_100Ms_core/single_shot
add wave -noupdate -radix hexadecimal -group TIMETAG /main/DUT/cmp_fmc_adc_mezzanine_0/cmp_timetag_core/*
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/clk_i
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/rst_n_i
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/status_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/ddr3_dq_b
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/ddr3_a_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/ddr3_ba_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/ddr3_ras_n_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/ddr3_cas_n_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/ddr3_we_n_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/ddr3_odt_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/ddr3_rst_n_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/ddr3_cke_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/ddr3_dm_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/ddr3_udm_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/ddr3_dqs_p_b
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/ddr3_dqs_n_b
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/ddr3_udqs_p_b
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/ddr3_udqs_n_b
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/ddr3_clk_p_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/ddr3_clk_n_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/ddr3_rzq_b
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/ddr3_zio_b
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/wb0_clk_i
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/wb0_sel_i
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/wb0_cyc_i
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/wb0_stb_i
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/wb0_we_i
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/wb0_addr_i
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/wb0_data_i
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/wb0_data_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/wb0_ack_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/wb0_stall_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_cmd_empty_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_cmd_full_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_rd_full_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_rd_empty_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_rd_count_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_rd_overflow_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_rd_error_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_wr_full_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_wr_empty_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_wr_count_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_wr_underrun_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_wr_error_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/wb1_clk_i
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/wb1_sel_i
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/wb1_cyc_i
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/wb1_stb_i
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/wb1_we_i
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/wb1_addr_i
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/wb1_data_i
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/wb1_data_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/wb1_ack_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/wb1_stall_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_cmd_empty_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_cmd_full_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_rd_full_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_rd_empty_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_rd_count_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_rd_overflow_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_rd_error_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_wr_full_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_wr_empty_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_wr_count_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_wr_underrun_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_wr_error_o
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_cmd_clk
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_cmd_en
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_cmd_instr
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_cmd_bl
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_cmd_byte_addr
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_cmd_empty
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_cmd_full
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_wr_clk
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_wr_en
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_wr_mask
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_wr_data
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_wr_full
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_wr_empty
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_wr_count
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_wr_underrun
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_wr_error
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_rd_clk
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_rd_en
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_rd_data
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_rd_full
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_rd_empty
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_rd_count
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_rd_overflow
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p0_rd_error
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_cmd_clk
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_cmd_en
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_cmd_instr
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_cmd_bl
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_cmd_byte_addr
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_cmd_empty
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_cmd_full
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_wr_clk
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_wr_en
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_wr_mask
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_wr_data
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_wr_full
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_wr_empty
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_wr_count
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_wr_underrun
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_wr_error
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_rd_clk
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_rd_en
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_rd_data
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_rd_full
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_rd_empty
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_rd_count
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_rd_overflow
add wave -noupdate -group DDRC /main/DUT/cmp_ddr_ctrl/p1_rd_error
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/rst_n_i
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_cmd_clk_o
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_cmd_en_o
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_cmd_instr_o
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_cmd_bl_o
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_cmd_byte_addr_o
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_cmd_empty_i
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_cmd_full_i
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_wr_clk_o
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_wr_en_o
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_wr_mask_o
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_wr_data_o
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_wr_full_i
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_wr_empty_i
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_wr_count_i
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_wr_underrun_i
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_wr_error_i
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_rd_clk_o
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_rd_en_o
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_rd_data_i
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_rd_full_i
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_rd_empty_i
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_rd_count_i
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_rd_overflow_i
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_rd_error_i
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/wb_clk_i
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/wb_sel_i
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/wb_cyc_i
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/wb_stb_i
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/wb_we_i
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/wb_addr_i
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/wb_data_i
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/wb_data_o
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/wb_ack_o
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/wb_stall_o
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_rd_en
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_cmd_en
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/wb_we_d
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/wb_addr_d
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/addr_shift
add wave -noupdate -group WB1 /main/DUT/cmp_ddr_ctrl/cmp_ddr3_ctrl_wb_1/ddr_burst_cnt
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/rst_n_i
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/wb_clk_i
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/wb_addr_i
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/wb_data_i
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/wb_data_o
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/wb_cyc_i
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/wb_sel_i
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/wb_stb_i
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/wb_we_i
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/wb_ack_o
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/clk_i
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_ctrl_o
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_ctrl_i
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_ctrl_load_o
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_stat_o
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_stat_i
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_stat_load_o
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_cstart_o
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_cstart_i
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_cstart_load_o
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_hstartl_o
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_hstartl_i
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_hstartl_load_o
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_hstarth_o
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_hstarth_i
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_hstarth_load_o
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_len_o
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_len_i
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_len_load_o
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_nextl_o
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_nextl_i
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_nextl_load_o
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_nexth_o
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_nexth_i
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_nexth_load_o
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_attrib_o
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_attrib_i
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_attrib_load_o
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_ctrl_int_read
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_ctrl_int_write
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_ctrl_lw
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_ctrl_lw_delay
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_ctrl_lw_read_in_progress
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_ctrl_lw_s0
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_ctrl_lw_s1
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_ctrl_lw_s2
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_ctrl_rwsel
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_stat_int_read
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_stat_int_write
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_stat_lw
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_stat_lw_delay
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_stat_lw_read_in_progress
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_stat_lw_s0
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_stat_lw_s1
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_stat_lw_s2
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_stat_rwsel
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_cstart_int_read
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_cstart_int_write
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_cstart_lw
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_cstart_lw_delay
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_cstart_lw_read_in_progress
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_cstart_lw_s0
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_cstart_lw_s1
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_cstart_lw_s2
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_cstart_rwsel
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_hstartl_int_read
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_hstartl_int_write
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_hstartl_lw
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_hstartl_lw_delay
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_hstartl_lw_read_in_progress
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_hstartl_lw_s0
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_hstartl_lw_s1
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_hstartl_lw_s2
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_hstartl_rwsel
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_hstarth_int_read
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_hstarth_int_write
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_hstarth_lw
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_hstarth_lw_delay
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_hstarth_lw_read_in_progress
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_hstarth_lw_s0
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_hstarth_lw_s1
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_hstarth_lw_s2
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_hstarth_rwsel
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_len_int_read
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_len_int_write
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_len_lw
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_len_lw_delay
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_len_lw_read_in_progress
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_len_lw_s0
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_len_lw_s1
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_len_lw_s2
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_len_rwsel
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_nextl_int_read
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_nextl_int_write
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_nextl_lw
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_nextl_lw_delay
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_nextl_lw_read_in_progress
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_nextl_lw_s0
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_nextl_lw_s1
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_nextl_lw_s2
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_nextl_rwsel
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_nexth_int_read
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_nexth_int_write
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_nexth_lw
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_nexth_lw_delay
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_nexth_lw_read_in_progress
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_nexth_lw_s0
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_nexth_lw_s1
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_nexth_lw_s2
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_nexth_rwsel
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_attrib_int_read
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_attrib_int_write
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_attrib_lw
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_attrib_lw_delay
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_attrib_lw_read_in_progress
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_attrib_lw_s0
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_attrib_lw_s1
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_attrib_lw_s2
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/dma_attrib_rwsel
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/ack_sreg
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/rddata_reg
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/wrdata_reg
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/bwsel_reg
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/rwaddr_reg
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/ack_in_progress
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/wr_int
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/rd_int
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/bus_clock_int
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/allones
add wave -noupdate -group DmaWB /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_controller_wb_slave_0/allzeros
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/clk_i
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/rst_n_i
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_ctrl_irq_o
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_ctrl_carrier_addr_o
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_ctrl_host_addr_h_o
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_ctrl_host_addr_l_o
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_ctrl_len_o
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_ctrl_start_l2p_o
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_ctrl_start_p2l_o
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_ctrl_start_next_o
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_ctrl_byte_swap_o
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_ctrl_abort_o
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_ctrl_done_i
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_ctrl_error_i
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/next_item_carrier_addr_i
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/next_item_host_addr_h_i
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/next_item_host_addr_l_i
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/next_item_len_i
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/next_item_next_l_i
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/next_item_next_h_i
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/next_item_attrib_i
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/next_item_valid_i
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/wb_clk_i
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/wb_adr_i
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/wb_dat_o
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/wb_dat_i
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/wb_sel_i
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/wb_cyc_i
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/wb_stb_i
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/wb_we_i
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/wb_ack_o
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_ctrl
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_stat
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_cstart
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_hstartl
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_hstarth
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_len
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_nextl
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_nexth
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_attrib
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_ctrl_load
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_stat_load
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_cstart_load
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_hstartl_load
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_hstarth_load
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_len_load
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_nextl_load
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_nexth_load
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_attrib_load
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_ctrl_reg
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_stat_reg
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_cstart_reg
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_hstartl_reg
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_hstarth_reg
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_len_reg
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_nextl_reg
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_nexth_reg
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_attrib_reg
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_ctrl_current_state
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_status
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_error_irq
add wave -noupdate -group DMACtrol /main/DUT/cmp_gn4124_core/cmp_dma_controller/dma_done_irq
add wave -noupdate -group Gennum /main/I_Gennum/ready
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/rst_n_a_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/status_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_clk_p_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_clk_n_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_data_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_dframe_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_valid_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_rdy_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p_wr_req_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p_wr_rdy_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/rx_error_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/vc_rdy_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/l2p_clk_p_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/l2p_clk_n_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/l2p_data_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/l2p_dframe_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/l2p_valid_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/l2p_edb_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/l2p_rdy_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/l_wr_rdy_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p_rd_d_rdy_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/tx_error_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_irq_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/irq_p_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/irq_p_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_reg_clk_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_reg_adr_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_reg_dat_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_reg_sel_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_reg_stb_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_reg_we_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_reg_cyc_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_reg_dat_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_reg_ack_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_reg_stall_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/csr_clk_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/csr_adr_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/csr_dat_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/csr_sel_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/csr_stb_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/csr_we_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/csr_cyc_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/csr_dat_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/csr_ack_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/csr_stall_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/csr_err_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/csr_rty_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/csr_int_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_clk_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_adr_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_dat_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_sel_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_stb_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_we_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_cyc_o
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_dat_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_ack_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_stall_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_err_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_rty_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_int_i
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/sys_clk
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/io_clk
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/serdes_strobe
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_pll_locked
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/rst_reg
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/rst_n
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/rst
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/des_pd_valid
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/des_pd_dframe
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/des_pd_data
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p_wr_rdy
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_rdy_wbm
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_rdy_pdm
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_hdr_start
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_hdr_length
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_hdr_cid
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_hdr_last
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_hdr_stat
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_target_mrd
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_target_mwr
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_master_cpld
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_master_cpln
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_d_valid
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_d_last
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_d
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_be
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_addr
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_addr_start
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/arb_ser_valid
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/arb_ser_dframe
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/arb_ser_data
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/l_wr_rdy_t
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/l_wr_rdy_t2
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/l_wr_rdy
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p_rd_d_rdy_t
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p_rd_d_rdy_t2
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p_rd_d_rdy
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/l2p_rdy_t
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/l2p_rdy_t2
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/l2p_rdy
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/l2p_edb
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/l2p_edb_t
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/l2p_edb_t2
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/tx_error_t2
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/tx_error_t
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/tx_error
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/wbm_arb_valid
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/wbm_arb_dframe
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/wbm_arb_data
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/wbm_arb_req
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/arb_wbm_gnt
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/ldm_arb_req
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/arb_ldm_gnt
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/ldm_arb_valid
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/ldm_arb_dframe
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/ldm_arb_data
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/pdm_arb_valid
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/pdm_arb_dframe
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/pdm_arb_data
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/pdm_arb_req
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/arb_pdm_gnt
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_ctrl_carrier_addr
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_ctrl_host_addr_h
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_ctrl_host_addr_l
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_ctrl_len
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_ctrl_start_l2p
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_ctrl_start_p2l
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_ctrl_start_next
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_ctrl_done
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_ctrl_error
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_ctrl_l2p_done
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_ctrl_l2p_error
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_ctrl_p2l_done
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_ctrl_p2l_error
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_ctrl_byte_swap
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_ctrl_abort
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/next_item_carrier_addr
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/next_item_host_addr_h
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/next_item_host_addr_l
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/next_item_len
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/next_item_next_l
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/next_item_next_h
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/next_item_attrib
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/next_item_valid
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/dma_irq
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/csr_adr
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/l2p_dma_adr
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/l2p_dma_dat_s2m
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/l2p_dma_dat_m2s
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/l2p_dma_sel
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/l2p_dma_cyc
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/l2p_dma_stb
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/l2p_dma_we
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/l2p_dma_ack
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/l2p_dma_stall
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_dma_adr
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_dma_dat_s2m
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_dma_dat_m2s
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_dma_sel
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_dma_cyc
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_dma_stb
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_dma_we
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_dma_ack
add wave -noupdate -group Gennum /main/DUT/cmp_gn4124_core/p2l_dma_stall
TreeUpdate [SetDefaultTree]
#WaveRestoreCursors {{Cursor 1} {54925075 ps} 0}
configure wave -namecolwidth 282
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 1
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ps
update
#WaveRestoreZoom {54724018 ps} {55134178 ps}
