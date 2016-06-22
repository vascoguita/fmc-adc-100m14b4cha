target = "xilinx"
action = "synthesis"

syn_device = "xc6slx150t"
syn_grade = "-3"
syn_package = "fgg900"
syn_top = "svec_top_fmc_adc_100Ms"
syn_project = "svec_fmc_adc_100Ms.xise"
syn_tool = "ise"

files = [
    "../svec_top_fmc_adc_100Ms.ucf",
    "../../ip_cores/adc_sync_fifo.ngc",
    "../../ip_cores/multishot_dpram.ngc",
    "../../ip_cores/wb_ddr_fifo.ngc",
    "../../ip_cores/adc_serdes.vhd",
    "../../ip_cores/monostable/monostable_rtl.vhd"]

modules = { "local" : ["../rtl",
                       "../../adc/rtl",
                       "../../ip_cores/timetag_core/rtl"],
            "git" : ["git://ohwr.org/hdl-core-lib/general-cores.git@@c26ee857158e4a65fd9d2add8b63fcb6fb4691ea",
                     "git://ohwr.org/hdl-core-lib/ddr3-sp6-core.git@@503171933f184ae878836f28e67a78a7c81b4325",
                     "git://ohwr.org/hdl-core-lib/vme64x-core.git@@b2fc3ce76485404f831d15f7ce31fdde08e234d5"]}

fetchto="../../ip_cores"

ctrls = ["bank4_64b_32b", "bank5_64b_32b"]
