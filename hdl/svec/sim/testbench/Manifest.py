sim_tool="modelsim"
top_module="main"
action = "simulation"
target = "xilinx"
syn_device = "xc6slx150t"
include_dirs=["../vme64x_bfm", "../2048Mb_ddr3", "../../../ip_cores/general-cores/sim"]

files = [ "main.sv",
          "../../../ip_cores/adc_sync_fifo.vhd",
          "../../../ip_cores/multishot_dpram.vhd",
          "../../../ip_cores/wb_ddr_fifo.vhd",
          "../../../ip_cores/adc_serdes.vhd",
          "../../../ip_cores/monostable/monostable_rtl.vhd",
          "../../../ip_cores/ext_pulse_sync/ext_pulse_sync_rtl.vhd",
          "../../../ip_cores/utils/utils_pkg.vhd"]

modules = { "local" :  [ "../../rtl",
                         "../2048Mb_ddr3",
                         "../../../adc/rtl",
                         "../../../ip_cores/timetag_core/rtl"],
            "git" : [ "git://ohwr.org/hdl-core-lib/general-cores.git::sdb_extension",
                      "git://ohwr.org/hdl-core-lib/ddr3-sp6-core.git::svec_bank4_64b_32b_bank5_64b_32b",
                      "git://ohwr.org/hdl-core-lib/vme64x-core.git::master"]}

fetchto="../../../ip_cores"

ctrls = ["bank4_64b_32b", "bank5_64b_32b"]
