# SPDX-FileCopyrightText: 2020 CERN (home.cern)
#
# SPDX-License-Identifier: CC0-1.0

onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /main/DUT/cmp_fmc_adc_mezzanine/cmp_fmc_adc_100Ms_core/acq_fsm_current_state
add wave -noupdate /main/DUT/cmp_fmc_adc_mezzanine/cmp_fmc_adc_100Ms_core/acq_start
add wave -noupdate /main/DUT/cmp_fmc_adc_mezzanine/cmp_fmc_adc_100Ms_core/single_shot
add wave -noupdate /main/DUT/cmp_fmc_adc_mezzanine/cmp_fmc_adc_100Ms_core/shots_cnt
add wave -noupdate /main/DUT/cmp_fmc_adc_mezzanine/cmp_fmc_adc_100Ms_core/shots_decr
add wave -noupdate /main/DUT/cmp_fmc_adc_mezzanine/cmp_fmc_adc_100Ms_core/samples_wr_en
add wave -noupdate /main/DUT/cmp_fmc_adc_mezzanine/cmp_fmc_adc_100Ms_core/acq_in_trig_tag
add wave -noupdate /main/DUT/cmp_fmc_adc_mezzanine/cmp_fmc_adc_100Ms_core/acq_trig
add wave -noupdate /main/DUT/cmp_fmc_adc_mezzanine/cmp_fmc_adc_100Ms_core/trig_addr
add wave -noupdate -group {SYNC FIFO} /main/DUT/cmp_fmc_adc_mezzanine/cmp_fmc_adc_100Ms_core/sync_fifo_wr
add wave -noupdate -group {SYNC FIFO} /main/DUT/cmp_fmc_adc_mezzanine/cmp_fmc_adc_100Ms_core/sync_fifo_full
add wave -noupdate -group {SYNC FIFO} /main/DUT/cmp_fmc_adc_mezzanine/cmp_fmc_adc_100Ms_core/sync_fifo_din
add wave -noupdate -group {SYNC FIFO} /main/DUT/cmp_fmc_adc_mezzanine/cmp_fmc_adc_100Ms_core/sync_fifo_empty
add wave -noupdate -group {SYNC FIFO} /main/DUT/cmp_fmc_adc_mezzanine/cmp_fmc_adc_100Ms_core/sync_fifo_rd
add wave -noupdate -group {SYNC FIFO} /main/DUT/cmp_fmc_adc_mezzanine/cmp_fmc_adc_100Ms_core/sync_fifo_valid
add wave -noupdate -group {SYNC FIFO} /main/DUT/cmp_fmc_adc_mezzanine/cmp_fmc_adc_100Ms_core/sync_fifo_dout
add wave -noupdate -group {DDR FIFO} /main/DUT/cmp_fmc_adc_mezzanine/cmp_fmc_adc_100Ms_core/wb_ddr_fifo_wr
add wave -noupdate -group {DDR FIFO} /main/DUT/cmp_fmc_adc_mezzanine/cmp_fmc_adc_100Ms_core/wb_ddr_fifo_wr_en
add wave -noupdate -group {DDR FIFO} /main/DUT/cmp_fmc_adc_mezzanine/cmp_fmc_adc_100Ms_core/wb_ddr_fifo_full
add wave -noupdate -group {DDR FIFO} /main/DUT/cmp_fmc_adc_mezzanine/cmp_fmc_adc_100Ms_core/wb_ddr_fifo_din
add wave -noupdate -group {DDR FIFO} /main/DUT/cmp_fmc_adc_mezzanine/cmp_fmc_adc_100Ms_core/wb_ddr_fifo_empty
add wave -noupdate -group {DDR FIFO} /main/DUT/cmp_fmc_adc_mezzanine/cmp_fmc_adc_100Ms_core/wb_ddr_fifo_rd
add wave -noupdate -group {DDR FIFO} /main/DUT/cmp_fmc_adc_mezzanine/cmp_fmc_adc_100Ms_core/wb_ddr_fifo_dout
add wave -noupdate -group DDR_WB0 /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/rst_n_i
add wave -noupdate -group DDR_WB0 /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/wb_clk_i
add wave -noupdate -group DDR_WB0 -expand -group Wishbone -color Cyan /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/wb_cyc_i
add wave -noupdate -group DDR_WB0 -expand -group Wishbone -color Cyan /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/wb_stb_i
add wave -noupdate -group DDR_WB0 -expand -group Wishbone -color Cyan /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/wb_stb_valid
add wave -noupdate -group DDR_WB0 -expand -group Wishbone -color Cyan /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/wb_addr_i
add wave -noupdate -group DDR_WB0 -expand -group Wishbone -color Cyan /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/wb_sel_i
add wave -noupdate -group DDR_WB0 -expand -group Wishbone -color Cyan /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/wb_stall_o
add wave -noupdate -group DDR_WB0 -expand -group Wishbone -color Cyan /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/wb_ack_o
add wave -noupdate -group DDR_WB0 -expand -group Wishbone -color Cyan /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/wb_data_o
add wave -noupdate -group DDR_WB0 -expand -group Wishbone -color Cyan /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/wb_we_i
add wave -noupdate -group DDR_WB0 -expand -group Wishbone -color Cyan /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/wb_data_i
add wave -noupdate -group DDR_WB0 -expand -group {CMD port} -color Magenta /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/cmd_fsm_state
add wave -noupdate -group DDR_WB0 -expand -group {CMD port} -color Magenta /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/ddr_cmd_stall
add wave -noupdate -group DDR_WB0 -expand -group {CMD port} -color Magenta /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/ddr_cmd_bl_o
add wave -noupdate -group DDR_WB0 -expand -group {CMD port} -color Magenta /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/ddr_burst_cnt
add wave -noupdate -group DDR_WB0 -expand -group {CMD port} -color Magenta /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/ddr_cmd_byte_addr_o
add wave -noupdate -group DDR_WB0 -expand -group {CMD port} -color Magenta /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/ddr_cmd_instr_o
add wave -noupdate -group DDR_WB0 -expand -group {CMD port} -color Magenta /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/ddr_cmd_empty_i
add wave -noupdate -group DDR_WB0 -expand -group {CMD port} -color Magenta /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/ddr_cmd_en_o
add wave -noupdate -group DDR_WB0 -expand -group {CMD port} -color Magenta /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/ddr_cmd_full_i
add wave -noupdate -group DDR_WB0 -group {RD port} -color Gold /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/ddr_rd_count_i
add wave -noupdate -group DDR_WB0 -group {RD port} -color Gold /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/ddr_rd_data_i
add wave -noupdate -group DDR_WB0 -group {RD port} -color Gold /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/ddr_rd_empty_i
add wave -noupdate -group DDR_WB0 -group {RD port} -color Gold /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/ddr_rd_en_o
add wave -noupdate -group DDR_WB0 -group {RD port} -color Gold /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/ddr_rd_error_i
add wave -noupdate -group DDR_WB0 -group {RD port} -color Gold /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/ddr_rd_full_i
add wave -noupdate -group DDR_WB0 -group {RD port} -color Gold /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/ddr_rd_overflow_i
add wave -noupdate -group DDR_WB0 -expand -group {WR port} /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/ddr_wr_en_o
add wave -noupdate -group DDR_WB0 -expand -group {WR port} /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/ddr_wr_mask_o
add wave -noupdate -group DDR_WB0 -expand -group {WR port} /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/ddr_wr_data_o
add wave -noupdate -group DDR_WB0 -expand -group {WR port} /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/ddr_wr_full_i
add wave -noupdate -group DDR_WB0 -expand -group {WR port} /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/ddr_wr_empty_i
add wave -noupdate -group DDR_WB0 -expand -group {WR port} /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/ddr_wr_count_i
add wave -noupdate -group DDR_WB0 -expand -group {WR port} /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/ddr_wr_underrun_i
add wave -noupdate -group DDR_WB0 -expand -group {WR port} /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_0/ddr_wr_error_i
add wave -noupdate -expand -group DDR_WB1 /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/rst_n_i
add wave -noupdate -expand -group DDR_WB1 /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/wb_clk_i
add wave -noupdate -expand -group DDR_WB1 -expand -group Wishbone -color Cyan /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/wb_cyc_i
add wave -noupdate -expand -group DDR_WB1 -expand -group Wishbone -color Cyan /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/wb_stb_i
add wave -noupdate -expand -group DDR_WB1 -expand -group Wishbone -color Cyan /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/wb_stb_valid
add wave -noupdate -expand -group DDR_WB1 -expand -group Wishbone -color Cyan /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/wb_addr_i
add wave -noupdate -expand -group DDR_WB1 -expand -group Wishbone -color Cyan /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/wb_sel_i
add wave -noupdate -expand -group DDR_WB1 -expand -group Wishbone -color Cyan /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/wb_stall_o
add wave -noupdate -expand -group DDR_WB1 -expand -group Wishbone -color Cyan /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/wb_ack_o
add wave -noupdate -expand -group DDR_WB1 -expand -group Wishbone -color Cyan /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/wb_data_o
add wave -noupdate -expand -group DDR_WB1 -expand -group Wishbone -color Cyan /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/wb_we_i
add wave -noupdate -expand -group DDR_WB1 -expand -group Wishbone -color Cyan /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/wb_data_i
add wave -noupdate -expand -group DDR_WB1 -expand -group {CMD port} -color Magenta /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/cmd_fsm_state
add wave -noupdate -expand -group DDR_WB1 -expand -group {CMD port} -color Magenta /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/ddr_cmd_stall
add wave -noupdate -expand -group DDR_WB1 -expand -group {CMD port} -color Magenta /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/ddr_cmd_bl_o
add wave -noupdate -expand -group DDR_WB1 -expand -group {CMD port} -color Magenta /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/ddr_burst_cnt
add wave -noupdate -expand -group DDR_WB1 -expand -group {CMD port} -color Magenta /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/ddr_cmd_byte_addr_o
add wave -noupdate -expand -group DDR_WB1 -expand -group {CMD port} -color Magenta /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/ddr_cmd_instr_o
add wave -noupdate -expand -group DDR_WB1 -expand -group {CMD port} -color Magenta /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/ddr_cmd_empty_i
add wave -noupdate -expand -group DDR_WB1 -expand -group {CMD port} -color Magenta /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/ddr_cmd_en_o
add wave -noupdate -expand -group DDR_WB1 -expand -group {CMD port} -color Magenta /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/ddr_cmd_full_i
add wave -noupdate -expand -group DDR_WB1 -expand -group {RD port} -color Gold /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/ddr_rd_count_i
add wave -noupdate -expand -group DDR_WB1 -expand -group {RD port} -color Gold /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/ddr_rd_data_i
add wave -noupdate -expand -group DDR_WB1 -expand -group {RD port} -color Gold /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/ddr_rd_empty_i
add wave -noupdate -expand -group DDR_WB1 -expand -group {RD port} -color Gold /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/ddr_rd_en_o
add wave -noupdate -expand -group DDR_WB1 -expand -group {RD port} -color Gold /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/ddr_rd_error_i
add wave -noupdate -expand -group DDR_WB1 -expand -group {RD port} -color Gold /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/ddr_rd_full_i
add wave -noupdate -expand -group DDR_WB1 -expand -group {RD port} -color Gold /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/ddr_rd_overflow_i
add wave -noupdate -expand -group DDR_WB1 -group {WR port} /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/ddr_wr_en_o
add wave -noupdate -expand -group DDR_WB1 -group {WR port} /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/ddr_wr_mask_o
add wave -noupdate -expand -group DDR_WB1 -group {WR port} /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/ddr_wr_data_o
add wave -noupdate -expand -group DDR_WB1 -group {WR port} /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/ddr_wr_full_i
add wave -noupdate -expand -group DDR_WB1 -group {WR port} /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/ddr_wr_empty_i
add wave -noupdate -expand -group DDR_WB1 -group {WR port} /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/ddr_wr_count_i
add wave -noupdate -expand -group DDR_WB1 -group {WR port} /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/ddr_wr_underrun_i
add wave -noupdate -expand -group DDR_WB1 -group {WR port} /main/DUT/inst_spec_base/gen_with_ddr/cmp_ddr_ctrl_bank3/cmp_ddr3_ctrl_wb_1/ddr_wr_error_i
add wave -noupdate /main/DUT/inst_spec_base/cmp_gn4124_core/cmp_wrapped_gn4124/gen_with_dma/cmp_l2p_dma_master/clk_i
add wave -noupdate /main/DUT/inst_spec_base/cmp_gn4124_core/cmp_wrapped_gn4124/gen_with_dma/cmp_l2p_dma_master/ldm_arb_req_o
add wave -noupdate /main/DUT/inst_spec_base/cmp_gn4124_core/cmp_wrapped_gn4124/gen_with_dma/cmp_l2p_dma_master/ldm_arb_dframe_o
add wave -noupdate /main/DUT/inst_spec_base/cmp_gn4124_core/cmp_wrapped_gn4124/gen_with_dma/cmp_l2p_dma_master/l2p_64b_address
add wave -noupdate /main/DUT/inst_spec_base/cmp_gn4124_core/cmp_wrapped_gn4124/gen_with_dma/cmp_l2p_dma_master/l2p_dma_current_state
add wave -noupdate -color Coral /main/DUT/inst_spec_base/cmp_gn4124_core/cmp_wrapped_gn4124/gen_with_dma/cmp_l2p_dma_master/data_fifo_full
add wave -noupdate -color Coral /main/DUT/inst_spec_base/cmp_gn4124_core/cmp_wrapped_gn4124/gen_with_dma/cmp_l2p_dma_master/data_fifo_empty
add wave -noupdate -color Coral /main/DUT/inst_spec_base/cmp_gn4124_core/cmp_wrapped_gn4124/gen_with_dma/cmp_l2p_dma_master/data_fifo_rd
add wave -noupdate -color Coral /main/DUT/inst_spec_base/cmp_gn4124_core/cmp_wrapped_gn4124/gen_with_dma/cmp_l2p_dma_master/data_fifo_dout
add wave -noupdate /main/DUT/inst_spec_base/cmp_gn4124_core/cmp_wrapped_gn4124/gen_with_dma/cmp_l2p_dma_master/ldm_arb_valid_o
add wave -noupdate /main/DUT/inst_spec_base/cmp_gn4124_core/cmp_wrapped_gn4124/gen_with_dma/cmp_l2p_dma_master/ldm_arb_data_o
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {49697641190 fs} 0}
quietly wave cursor active 1
configure wave -namecolwidth 366
configure wave -valuecolwidth 182
configure wave -justifyvalue left
configure wave -signalnamewidth 2
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 400000
configure wave -gridperiod 800000
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
WaveRestoreZoom {0 fs} {52312050 ps}
