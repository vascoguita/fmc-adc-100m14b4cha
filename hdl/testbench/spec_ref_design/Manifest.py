# SPDX-FileCopyrightText: 2020 CERN (home.cern)
#
# SPDX-License-Identifier: CC0-1.0

board      = "spec"
sim_tool   = "modelsim"
sim_top    = "main"
action     = "simulation"
target     = "xilinx"
syn_device = "xc6slx45t"

vcom_opt = "-93 -mixedsvvh"

# Allow the user to override fetchto using:
#  hdlmake -p "fetchto='xxx'"
if locals().get('fetchto', None) is None:
    fetchto="../../ip_cores"

include_dirs = [
    "../include",
    fetchto + "/gn4124-core/hdl/sim/gn4124_bfm",
    fetchto + "/general-cores/sim/",
    fetchto + "/general-cores/modules/wishbone/wb_spi/",
    fetchto + "/ddr3-sp6-core/hdl/sim/",
]

files = [
    "main.sv",
    "buildinfo_pkg.vhd",
]

modules = {
    "local" : [
        "../../top/spec_ref_design",
    ],
}

# Do not fail during hdlmake fetch
try:
  exec(open(fetchto + "/general-cores/tools/gen_buildinfo.py").read())
except:
  pass

ctrls = ["bank3_64b_32b" ]
