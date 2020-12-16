# SPDX-FileCopyrightText: 2020 CERN (home.cern)
#
# SPDX-License-Identifier: CC0-1.0

files = [
    "svec_ref_fmc_adc_100Ms.vhd",
    "../../cheby/svec_ref_fmc_adc_100Ms_mmap.vhd",
]

fetchto = "../../ip_cores"

modules = {
    "local" : [
    "../../../",
    ],
    "git" : [
        "https://ohwr.org/project/general-cores.git",
        "https://ohwr.org/project/wr-cores.git",
        "https://ohwr.org/project/ddr3-sp6-core.git",
        "https://ohwr.org/project/vme64x-core.git",
        "https://ohwr.org/project/svec.git",
    ],
}
