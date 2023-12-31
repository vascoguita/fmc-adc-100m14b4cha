# SPDX-FileCopyrightText: 2020 CERN (home.cern)
#
# SPDX-License-Identifier: CC0-1.0

files = [
    "fmc_adc_mezzanine.vhd",
    "fmc_adc_mezzanine_pkg.vhd",
    "fmc_adc_100Ms_core.vhd",
    "fmc_adc_100Ms_core_pkg.vhd",
    "fmc_adc_eic.vhd",
    "offset_gain_s.vhd",
    "timetag_core.vhd",
    "../cheby/fmc_adc_mezzanine_mmap.vhd",
    "../cheby/fmc_adc_100Ms_csr.vhd",
    "../cheby/fmc_adc_100Ms_channel_regs.vhd",
    "../cheby/fmc_adc_eic_regs.vhd",
    "../cheby/fmc_adc_aux_trigin.vhd",
    "../cheby/fmc_adc_aux_trigout.vhd",
    "../cheby/timetag_core_regs.vhd",
]
