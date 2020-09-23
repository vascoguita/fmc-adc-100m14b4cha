board      = "svec"
sim_tool   = "modelsim"
sim_top    = "main"
action     = "simulation"
target     = "xilinx"
syn_device = "xc6slx150t"

vcom_opt = "-93 -mixedsvvh"

# Allow the user to override fetchto using:
#  hdlmake -p "fetchto='xxx'"
if locals().get('fetchto', None) is None:
    fetchto="../../ip_cores"

include_dirs = [
    "../include",
    fetchto + "/general-cores/sim/",
    fetchto + "/general-cores/modules/wishbone/wb_spi/",
    fetchto + "/ddr3-sp6-core/hdl/sim/",
    fetchto + "/vme64x-core/hdl/sim/vme64x_bfm/",
]

files = [
    "main.sv",
    "buildinfo_pkg.vhd",
]

modules = {
    "local" : [
        "../../top/svec_ref_design",
    ],
}

# Do not fail during hdlmake fetch
try:
  exec(open(fetchto + "/general-cores/tools/gen_buildinfo.py").read())
except:
  pass

ctrls = ["bank4_64b_32b", "bank5_64b_32b"]
