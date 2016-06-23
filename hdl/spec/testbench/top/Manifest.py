sim_tool="modelsim"
top_module="main"
action = "simulation"
target = "xilinx"
fetchto = "../../ip_cores"
syn_device="xc6slx45t"
include_dirs=["../include","gn4124_bfm", "ddr3"]

files = [
    "main.sv",
    "ddr3/ddr3.v"]

modules = { "local" :  [ "gn4124_bfm",
                         "../../rtl",
                         "../../../adc/rtl",
			 "../../../ip_cores/general-cores",
			 "../../../ip_cores/ddr3-sp6-core",
                         "../../../ip_cores/gn4124-core" ]};

ctrls = ["bank3_64b_32b" ]
