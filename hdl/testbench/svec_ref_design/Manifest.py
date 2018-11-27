board      = "svec"
sim_tool   = "modelsim"
top_module = "main"
action     = "simulation"
target     = "xilinx"
syn_device = "xc6slx150t"

vcom_opt = "-93 -mixedsvvh"

fetchto = "../../ip_cores"

include_dirs = [
    "../include",
    fetchto + "/general-cores/sim/",
    fetchto + "/general-cores/modules/wishbone/wb_spi/",
    fetchto + "/vme64x-core/hdl/sim/vme64x_bfm/",
]

files = [
    "main.sv",
    "synthesis_descriptor.vhd",
]

modules = {
    "local" : [
        "../../top/svec_ref_design",
    ],
    "git" : [
        "git://ohwr.org/hdl-core-lib/general-cores.git",
        "git://ohwr.org/hdl-core-lib/wr-cores.git",
        "git://ohwr.org/hdl-core-lib/ddr3-sp6-core.git",
        "git://ohwr.org/hdl-core-lib/vme64x-core.git",
    ],
}

ctrls = ["bank4_64b_32b", "bank5_64b_32b"]
