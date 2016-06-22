target = "xilinx"
action = "synthesis"

syn_device = "xc6slx45t"
syn_grade = "-3"
syn_package = "fgg484"
syn_top = "spec_top_fmc_adc_100Ms"
syn_project = "spec_fmc_adc_100Ms.xise"
syn_tool = "ise"

files = [
    "../spec_top_fmc_adc_100Ms.ucf",
    "../../ip_cores/adc_serdes.vhd"]

modules = { "local" : ["../rtl",
                       "../../adc/rtl",
                       "../../ip_cores/timetag_core/rtl"],
            "git" : ["git://ohwr.org/hdl-core-lib/general-cores.git@@c26ee857158e4a65fd9d2add8b63fcb6fb4691ea",
                     "git://ohwr.org/hdl-core-lib/ddr3-sp6-core.git@@503171933f184ae878836f28e67a78a7c81b4325",
                     "git://ohwr.org/hdl-core-lib/gn4124-core.git@@e0dcb3f9a3e6804f64c544743bdf46b5fcbbefab"]}

fetchto="../../ip_cores"

ctrls = ["bank3_64b_32b" ]
