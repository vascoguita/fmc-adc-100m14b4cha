// SPDX-FileCopyrightText: 2020 CERN (home.cern)
//
// SPDX-License-Identifier: CC0-1.0

`define FMC_ADC_100MS_CSR_SIZE 512
`define ADDR_FMC_ADC_100MS_CSR_CTL 'h0
`define FMC_ADC_100MS_CSR_CTL_FSM_CMD_OFFSET 0
`define FMC_ADC_100MS_CSR_CTL_FSM_CMD 'h3
`define FMC_ADC_100MS_CSR_CTL_FMC_CLK_OE_OFFSET 2
`define FMC_ADC_100MS_CSR_CTL_FMC_CLK_OE 'h4
`define FMC_ADC_100MS_CSR_CTL_OFFSET_DAC_CLR_N_OFFSET 3
`define FMC_ADC_100MS_CSR_CTL_OFFSET_DAC_CLR_N 'h8
`define FMC_ADC_100MS_CSR_CTL_MAN_BITSLIP_OFFSET 4
`define FMC_ADC_100MS_CSR_CTL_MAN_BITSLIP 'h10
`define FMC_ADC_100MS_CSR_CTL_TRIG_LED_OFFSET 6
`define FMC_ADC_100MS_CSR_CTL_TRIG_LED 'h40
`define FMC_ADC_100MS_CSR_CTL_ACQ_LED_OFFSET 7
`define FMC_ADC_100MS_CSR_CTL_ACQ_LED 'h80
`define FMC_ADC_100MS_CSR_CTL_CLEAR_TRIG_STAT_OFFSET 8
`define FMC_ADC_100MS_CSR_CTL_CLEAR_TRIG_STAT 'h100
`define FMC_ADC_100MS_CSR_CTL_CALIB_APPLY_OFFSET 15
`define FMC_ADC_100MS_CSR_CTL_CALIB_APPLY 'h8000
`define ADDR_FMC_ADC_100MS_CSR_STA 'h4
`define FMC_ADC_100MS_CSR_STA_FSM_OFFSET 0
`define FMC_ADC_100MS_CSR_STA_FSM 'h7
`define FMC_ADC_100MS_CSR_STA_SERDES_PLL_OFFSET 3
`define FMC_ADC_100MS_CSR_STA_SERDES_PLL 'h8
`define FMC_ADC_100MS_CSR_STA_SERDES_SYNCED_OFFSET 4
`define FMC_ADC_100MS_CSR_STA_SERDES_SYNCED 'h10
`define FMC_ADC_100MS_CSR_STA_ACQ_CFG_OFFSET 5
`define FMC_ADC_100MS_CSR_STA_ACQ_CFG 'h20
`define FMC_ADC_100MS_CSR_STA_FMC_NR_OFFSET 6
`define FMC_ADC_100MS_CSR_STA_FMC_NR 'hc0
`define FMC_ADC_100MS_CSR_STA_CALIB_BUSY_OFFSET 15
`define FMC_ADC_100MS_CSR_STA_CALIB_BUSY 'h8000
`define ADDR_FMC_ADC_100MS_CSR_TRIG_STAT 'h8
`define FMC_ADC_100MS_CSR_TRIG_STAT_EXT_OFFSET 0
`define FMC_ADC_100MS_CSR_TRIG_STAT_EXT 'h1
`define FMC_ADC_100MS_CSR_TRIG_STAT_SW_OFFSET 1
`define FMC_ADC_100MS_CSR_TRIG_STAT_SW 'h2
`define FMC_ADC_100MS_CSR_TRIG_STAT_TIME_OFFSET 4
`define FMC_ADC_100MS_CSR_TRIG_STAT_TIME 'h10
`define FMC_ADC_100MS_CSR_TRIG_STAT_CH1_OFFSET 8
`define FMC_ADC_100MS_CSR_TRIG_STAT_CH1 'h100
`define FMC_ADC_100MS_CSR_TRIG_STAT_CH2_OFFSET 9
`define FMC_ADC_100MS_CSR_TRIG_STAT_CH2 'h200
`define FMC_ADC_100MS_CSR_TRIG_STAT_CH3_OFFSET 10
`define FMC_ADC_100MS_CSR_TRIG_STAT_CH3 'h400
`define FMC_ADC_100MS_CSR_TRIG_STAT_CH4_OFFSET 11
`define FMC_ADC_100MS_CSR_TRIG_STAT_CH4 'h800
`define ADDR_FMC_ADC_100MS_CSR_TRIG_EN 'hc
`define FMC_ADC_100MS_CSR_TRIG_EN_EXT_OFFSET 0
`define FMC_ADC_100MS_CSR_TRIG_EN_EXT 'h1
`define FMC_ADC_100MS_CSR_TRIG_EN_SW_OFFSET 1
`define FMC_ADC_100MS_CSR_TRIG_EN_SW 'h2
`define FMC_ADC_100MS_CSR_TRIG_EN_TIME_OFFSET 4
`define FMC_ADC_100MS_CSR_TRIG_EN_TIME 'h10
`define FMC_ADC_100MS_CSR_TRIG_EN_AUX_TIME_OFFSET 5
`define FMC_ADC_100MS_CSR_TRIG_EN_AUX_TIME 'h20
`define FMC_ADC_100MS_CSR_TRIG_EN_CH1_OFFSET 8
`define FMC_ADC_100MS_CSR_TRIG_EN_CH1 'h100
`define FMC_ADC_100MS_CSR_TRIG_EN_CH2_OFFSET 9
`define FMC_ADC_100MS_CSR_TRIG_EN_CH2 'h200
`define FMC_ADC_100MS_CSR_TRIG_EN_CH3_OFFSET 10
`define FMC_ADC_100MS_CSR_TRIG_EN_CH3 'h400
`define FMC_ADC_100MS_CSR_TRIG_EN_CH4_OFFSET 11
`define FMC_ADC_100MS_CSR_TRIG_EN_CH4 'h800
`define ADDR_FMC_ADC_100MS_CSR_TRIG_POL 'h10
`define FMC_ADC_100MS_CSR_TRIG_POL_EXT_OFFSET 0
`define FMC_ADC_100MS_CSR_TRIG_POL_EXT 'h1
`define FMC_ADC_100MS_CSR_TRIG_POL_CH1_OFFSET 8
`define FMC_ADC_100MS_CSR_TRIG_POL_CH1 'h100
`define FMC_ADC_100MS_CSR_TRIG_POL_CH2_OFFSET 9
`define FMC_ADC_100MS_CSR_TRIG_POL_CH2 'h200
`define FMC_ADC_100MS_CSR_TRIG_POL_CH3_OFFSET 10
`define FMC_ADC_100MS_CSR_TRIG_POL_CH3 'h400
`define FMC_ADC_100MS_CSR_TRIG_POL_CH4_OFFSET 11
`define FMC_ADC_100MS_CSR_TRIG_POL_CH4 'h800
`define ADDR_FMC_ADC_100MS_CSR_EXT_TRIG_DLY 'h14
`define ADDR_FMC_ADC_100MS_CSR_SW_TRIG 'h18
`define ADDR_FMC_ADC_100MS_CSR_SHOTS 'h1c
`define FMC_ADC_100MS_CSR_SHOTS_NBR_OFFSET 0
`define FMC_ADC_100MS_CSR_SHOTS_NBR 'hffff
`define FMC_ADC_100MS_CSR_SHOTS_REMAIN_OFFSET 16
`define FMC_ADC_100MS_CSR_SHOTS_REMAIN 'hffff0000
`define ADDR_FMC_ADC_100MS_CSR_MULTI_DEPTH 'h20
`define ADDR_FMC_ADC_100MS_CSR_TRIG_POS 'h24
`define ADDR_FMC_ADC_100MS_CSR_FS_FREQ 'h28
`define ADDR_FMC_ADC_100MS_CSR_DOWNSAMPLE 'h2c
`define ADDR_FMC_ADC_100MS_CSR_PRE_SAMPLES 'h30
`define ADDR_FMC_ADC_100MS_CSR_POST_SAMPLES 'h34
`define ADDR_FMC_ADC_100MS_CSR_SAMPLES_CNT 'h38
`define ADDR_FMC_ADC_100MS_CSR_FMC_ADC_CH1 'h80
`define FMC_ADC_100MS_CSR_FMC_ADC_CH1_SIZE 32
`define ADDR_FMC_ADC_100MS_CSR_FMC_ADC_CH2 'hc0
`define FMC_ADC_100MS_CSR_FMC_ADC_CH2_SIZE 32
`define ADDR_FMC_ADC_100MS_CSR_FMC_ADC_CH3 'h100
`define FMC_ADC_100MS_CSR_FMC_ADC_CH3_SIZE 32
`define ADDR_FMC_ADC_100MS_CSR_FMC_ADC_CH4 'h140
`define FMC_ADC_100MS_CSR_FMC_ADC_CH4_SIZE 32
