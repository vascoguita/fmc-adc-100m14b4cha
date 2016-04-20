onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -radix hexadecimal /tb_spec/u1/cnx_master_out
add wave -noupdate -radix hexadecimal /tb_spec/u1/cnx_master_in
add wave -noupdate -divider {Wishbone CSR interface}
add wave -noupdate /tb_spec/u1/sys_clk_125
add wave -noupdate -radix hexadecimal /tb_spec/u1/cnx_slave_out
add wave -noupdate -radix hexadecimal /tb_spec/u1/cnx_slave_in
add wave -noupdate -divider onewire
add wave -noupdate /tb_spec/u1/adc0_one_wire_b
add wave -noupdate -radix hexadecimal /tb_spec/u1/cmp_fmc_adc_mezzanine_0/cmp_fmc_onewire/owr_pwren_o(0)
add wave -noupdate -radix hexadecimal /tb_spec/u1/cmp_fmc_adc_mezzanine_0/cmp_fmc_onewire/owr_en_o(0)
add wave -noupdate -radix hexadecimal /tb_spec/u1/cmp_fmc_adc_mezzanine_0/cmp_fmc_onewire/owr_i(0)
add wave -noupdate -radix hexadecimal /tb_spec/u1/cmp_fmc_adc_mezzanine_0/cmp_fmc_onewire/clk_sys_i
add wave -noupdate -radix hexadecimal /tb_spec/u1/cmp_fmc_adc_mezzanine_0/cmp_fmc_onewire/rst_n_i
add wave -noupdate /tb_spec/u1/cmp_fmc_adc_mezzanine_0/cmp_fmc_onewire/slave_i
add wave -noupdate /tb_spec/u1/cmp_fmc_adc_mezzanine_0/cmp_fmc_onewire/slave_o
add wave -noupdate -divider l2p
add wave -noupdate -radix hexadecimal /tb_spec/u1/cmp_gn4124_core/cmp_l2p_dma_master/wb_ack_cnt
add wave -noupdate -radix hexadecimal /tb_spec/u1/cmp_gn4124_core/cmp_l2p_dma_master/wb_read_cnt
add wave -noupdate /tb_spec/u1/cmp_gn4124_core/cmp_l2p_dma_master/dma_ctrl_start_l2p_i
add wave -noupdate -radix hexadecimal /tb_spec/u1/cmp_gn4124_core/cmp_l2p_dma_master/dma_ctrl_len_i
add wave -noupdate /tb_spec/u1/cmp_gn4124_core/cmp_l2p_dma_master/l2p_last_packet
add wave -noupdate -radix hexadecimal /tb_spec/u1/cmp_gn4124_core/cmp_l2p_dma_master/l2p_data_cnt
add wave -noupdate -radix hexadecimal /tb_spec/u1/cmp_gn4124_core/cmp_l2p_dma_master/l2p_len_header
add wave -noupdate -radix hexadecimal /tb_spec/u1/cmp_gn4124_core/cmp_l2p_dma_master/l2p_len_cnt
add wave -noupdate /tb_spec/u1/cmp_gn4124_core/cmp_l2p_dma_master/l2p_dma_current_state
add wave -noupdate /tb_spec/u1/cmp_gn4124_core/cmp_l2p_dma_master/dma_ctrl_done_o
add wave -noupdate /tb_spec/u1/cmp_gn4124_core/cmp_dma_controller/dma_ctrl_current_state
add wave -noupdate -radix hexadecimal /tb_spec/u1/cmp_gn4124_core/ldm_arb_data
add wave -noupdate /tb_spec/u1/cmp_gn4124_core/ldm_arb_dframe
add wave -noupdate /tb_spec/u1/cmp_gn4124_core/ldm_arb_req
add wave -noupdate /tb_spec/u1/cmp_gn4124_core/ldm_arb_valid
add wave -noupdate -divider p2l
add wave -noupdate /tb_spec/u1/cmp_gn4124_core/cmp_p2l_dma_master/p2l_dma_current_state
add wave -noupdate -radix hexadecimal /tb_spec/u1/cmp_gn4124_core/cmp_p2l_dma_master/l2p_len_cnt
add wave -noupdate -radix hexadecimal /tb_spec/u1/cmp_gn4124_core/cmp_p2l_dma_master/to_wb_fifo_din
add wave -noupdate /tb_spec/u1/cmp_gn4124_core/cmp_p2l_dma_master/to_wb_fifo_wr
add wave -noupdate -divider {GN4124 LOCAL BUS}
add wave -noupdate -radix hexadecimal /tb_spec/u1/cmp_gn4124_core/l2p_data_o
add wave -noupdate /tb_spec/u1/cmp_gn4124_core/l2p_dframe_o
add wave -noupdate /tb_spec/u1/cmp_gn4124_core/l2p_valid_o
add wave -noupdate /tb_spec/u1/cmp_gn4124_core/l2p_edb_o
add wave -noupdate /tb_spec/u1/cmp_gn4124_core/l_wr_rdy_i
add wave -noupdate -radix hexadecimal /tb_spec/u1/cmp_gn4124_core/p2l_data_i
add wave -noupdate /tb_spec/u1/cmp_gn4124_core/p2l_dframe_i
add wave -noupdate /tb_spec/u1/cmp_gn4124_core/p2l_rdy_o
add wave -noupdate /tb_spec/u1/cmp_gn4124_core/p2l_valid_i
add wave -noupdate /tb_spec/u1/cmp_gn4124_core/p_rd_d_rdy_i
add wave -noupdate /tb_spec/u1/cmp_gn4124_core/p_wr_rdy_o
add wave -noupdate /tb_spec/u1/cmp_gn4124_core/p_wr_req_i
add wave -noupdate /tb_spec/u1/cmp_gn4124_core/tx_error_i
add wave -noupdate /tb_spec/u1/cmp_gn4124_core/vc_rdy_i
add wave -noupdate -divider {gennum global}
add wave -noupdate -expand /tb_spec/u1/irq_sources
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {15313175 ps} 0} {{Cursor 2} {25032000 ps} 0}
configure wave -namecolwidth 496
configure wave -valuecolwidth 172
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
WaveRestoreZoom {13598596 ps} {13854503 ps}
