board  = "svec"
target = "xilinx"
action = "synthesis"

syn_device  = "xc6slx150t"
syn_grade   = "-3"
syn_package = "fgg900"
syn_top     = "svec_ref_fmc_adc_100Ms"
syn_project = syn_top + "_wr.xise"
syn_tool    = "ise"

files = [
    syn_top + "_wr.ucf",
]

modules = {
    "local" : [
        "../../top/svec_ref_design"
    ],
    "git" : [
        "git://ohwr.org/hdl-core-lib/general-cores.git",
        "git://ohwr.org/hdl-core-lib/ddr3-sp6-core.git",
        "git://ohwr.org/hdl-core-lib/vme64x-core.git",
        "git://ohwr.org/hdl-core-lib/wr-cores.git",
    ],
}

fetchto="../../ip_cores"

ctrls = ["bank4_64b_32b", "bank5_64b_32b"]

syn_post_project_cmd = "$(TCL_INTERPRETER) " + \
                       fetchto + "/general-cores/tools/sdb_desc_gen.tcl " + \
                       syn_tool + " $(PROJECT_FILE)"
