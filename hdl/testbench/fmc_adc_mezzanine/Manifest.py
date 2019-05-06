board      = "spec"
sim_tool   = "modelsim"
top_module = "main"
action     = "simulation"
target     = "xilinx"
syn_device = "xc6slx45t"

vcom_opt = "-93 -mixedsvvh"

fetchto = "../../ip_cores"

include_dirs = [
    "../include",
    fetchto + "/general-cores/sim/",
    fetchto + "/general-cores/modules/wishbone/wb_spi/",
]

files = [
    "main.sv",
]

modules = {
    "local" : [
        "../../../",
    ],
    "git" : [
        "git://ohwr.org/hdl-core-lib/general-cores.git",
    ],
}
