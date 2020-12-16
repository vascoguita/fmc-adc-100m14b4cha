# SPDX-FileCopyrightText: 2020 CERN (home.cern)
#
# SPDX-License-Identifier: CC0-1.0

board  = "spec"
target = "xilinx"
action = "synthesis"

syn_device  = "xc6slx150t"
syn_grade   = "-3"
syn_package = "fgg484"
syn_top     = "spec_ref_fmc_adc_100Ms"
syn_project = "spec150_ref_fmc_adc_100ms_wr.xise"
syn_tool    = "ise"

# Allow the user to override fetchto using:
#  hdlmake -p "fetchto='xxx'"
if locals().get('fetchto', None) is None:
    fetchto="../../ip_cores"

files = [
    syn_top + "_wr.ucf",
    "buildinfo_pkg.vhd",
]

modules = {
    "local" : [
        "../../top/spec_ref_design"
    ],
}

# Do not fail during hdlmake fetch
try:
  exec(open(fetchto + "/general-cores/tools/gen_buildinfo.py").read())
except:
  pass

syn_post_project_cmd = "$(TCL_INTERPRETER) syn_extra_steps.tcl $(PROJECT_FILE)"

spec_base_ucf = ['wr', 'ddr3', 'onewire', 'spi']

ctrls = ["bank3_64b_32b" ]
