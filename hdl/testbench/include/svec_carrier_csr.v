// SPDX-FileCopyrightText: 2020 CERN (home.cern)
//
// SPDX-License-Identifier: CC0-1.0

`define SVEC_CARRIER_CSR_SIZE 16
`define ADDR_SVEC_CARRIER_CSR_CARRIER 'h0
`define SVEC_CARRIER_CSR_CARRIER_PCB_REV_OFFSET 0
`define SVEC_CARRIER_CSR_CARRIER_PCB_REV 'h1f
`define SVEC_CARRIER_CSR_CARRIER_RESERVED_OFFSET 5
`define SVEC_CARRIER_CSR_CARRIER_RESERVED 'hffe0
`define SVEC_CARRIER_CSR_CARRIER_TYPE_OFFSET 16
`define SVEC_CARRIER_CSR_CARRIER_TYPE 'hffff0000
`define ADDR_SVEC_CARRIER_CSR_STAT 'h4
`define SVEC_CARRIER_CSR_STAT_FMC0_PRES_OFFSET 0
`define SVEC_CARRIER_CSR_STAT_FMC0_PRES 'h1
`define SVEC_CARRIER_CSR_STAT_FMC1_PRES_OFFSET 1
`define SVEC_CARRIER_CSR_STAT_FMC1_PRES 'h2
`define SVEC_CARRIER_CSR_STAT_SYS_PLL_LCK_OFFSET 2
`define SVEC_CARRIER_CSR_STAT_SYS_PLL_LCK 'h4
`define SVEC_CARRIER_CSR_STAT_DDR0_CAL_DONE_OFFSET 3
`define SVEC_CARRIER_CSR_STAT_DDR0_CAL_DONE 'h8
`define SVEC_CARRIER_CSR_STAT_DDR1_CAL_DONE_OFFSET 4
`define SVEC_CARRIER_CSR_STAT_DDR1_CAL_DONE 'h10
`define ADDR_SVEC_CARRIER_CSR_CTRL 'h8
`define SVEC_CARRIER_CSR_CTRL_FP_LEDS_MAN_OFFSET 0
`define SVEC_CARRIER_CSR_CTRL_FP_LEDS_MAN 'hffff
`define ADDR_SVEC_CARRIER_CSR_RST 'hc
`define SVEC_CARRIER_CSR_RST_FMC0_OFFSET 0
`define SVEC_CARRIER_CSR_RST_FMC0 'h1
`define SVEC_CARRIER_CSR_RST_FMC1_OFFSET 1
`define SVEC_CARRIER_CSR_RST_FMC1 'h2
