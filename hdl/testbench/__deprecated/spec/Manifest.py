sim_tool="modelsim"
top_module="main"
action = "simulation"
target = "xilinx"
fetchto = "../../ip_cores"
syn_device="xc6slx45t"
include_dirs=["../include","gn4124_bfm", "ddr3"]

files = [
    "sfp_i2c_adapter.vhd",
    "main.sv",
    "ddr3/ddr3.v"]

modules = { "local" :  [ "gn4124_bfm",
                         "../../rtl",
                         "../../../adc/rtl",
                         "../../../ip_cores/wr-cores/board/common",
                         "../../../ip_cores/wr-cores",
			 "../../../ip_cores/general-cores",
			 "../../../ip_cores/etherbone-core",
			 "../../../ip_cores/ddr3-sp6-core",
                         "../../../ip_cores/gn4124-core" ]};

ctrls = ["bank3_64b_32b" ]
