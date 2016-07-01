target = "xilinx"
action = "synthesis"

syn_device = "xc6slx45t"
syn_grade = "-3"
syn_package = "fgg484"
syn_top = "spec_top_fmc_adc_100Ms"
syn_project = "spec_fmc_adc_100Ms.xise"
syn_tool = "ise"

files = ["wrc.ram",
         "../spec_top_fmc_adc_100Ms.ucf"]

modules = {
    "local" : [
        "../rtl",
        "../../adc/rtl",
    ],
    "git" : [
        "git://ohwr.org/hdl-core-lib/general-cores.git",
        "git://ohwr.org/hdl-core-lib/ddr3-sp6-core.git",
        "git://ohwr.org/hdl-core-lib/gn4124-core.git",
        "git://ohwr.org/hdl-core-lib/wr-cores.git",
    ],
}

fetchto="../../ip_cores"

ctrls = ["bank3_64b_32b" ]
