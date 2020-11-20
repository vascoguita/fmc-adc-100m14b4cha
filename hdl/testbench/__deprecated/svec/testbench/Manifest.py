# SPDX-FileCopyrightText: 2020 CERN (home.cern)
#
# SPDX-License-Identifier: CC0-1.0

sim_tool="modelsim"
top_module="main"
action = "simulation"
target = "xilinx"
syn_device = "xc6slx150t"
include_dirs=["../vme64x_bfm", "../2048Mb_ddr3", "../../../ip_cores/general-cores/sim"]

files = [ "main.sv"]

modules = { "local" :  [ "../2048Mb_ddr3",
                         "../../rtl",
                         "../../../adc/rtl"],
            "git" : [ "git://ohwr.org/hdl-core-lib/general-cores.git::sdb_extension",
                      "git://ohwr.org/hdl-core-lib/ddr3-sp6-core.git::svec_bank4_64b_32b_bank5_64b_32b",
                      "git://ohwr.org/hdl-core-lib/vme64x-core.git::master"]}

fetchto="../../../ip_cores"

ctrls = ["bank4_64b_32b", "bank5_64b_32b"]
