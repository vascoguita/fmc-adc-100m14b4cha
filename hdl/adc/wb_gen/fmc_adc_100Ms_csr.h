/*
  Register definitions for slave core: FMC ADC 100MS/s core registers

  * File           : fmc_adc_100Ms_csr.h
  * Author         : auto-generated by wbgen2 from fmc_adc_100Ms_csr.wb
  * Created        : Tue Dec 17 09:57:20 2013
  * Standard       : ANSI C

    THIS FILE WAS GENERATED BY wbgen2 FROM SOURCE FILE fmc_adc_100Ms_csr.wb
    DO NOT HAND-EDIT UNLESS IT'S ABSOLUTELY NECESSARY!

*/

#ifndef __WBGEN2_REGDEFS_FMC_ADC_100MS_CSR_WB
#define __WBGEN2_REGDEFS_FMC_ADC_100MS_CSR_WB

#include <inttypes.h>

#if defined( __GNUC__)
#define PACKED __attribute__ ((packed))
#else
#error "Unsupported compiler?"
#endif

#ifndef __WBGEN2_MACROS_DEFINED__
#define __WBGEN2_MACROS_DEFINED__
#define WBGEN2_GEN_MASK(offset, size) (((1<<(size))-1) << (offset))
#define WBGEN2_GEN_WRITE(value, offset, size) (((value) & ((1<<(size))-1)) << (offset))
#define WBGEN2_GEN_READ(reg, offset, size) (((reg) >> (offset)) & ((1<<(size))-1))
#define WBGEN2_SIGN_EXTEND(value, bits) (((value) & (1<<bits) ? ~((1<<(bits))-1): 0 ) | (value))
#endif


/* definitions for register: Control register */

/* definitions for field: State machine commands (ignore on read) in reg: Control register */
#define FMC_ADC_CORE_CTL_FSM_CMD_MASK         WBGEN2_GEN_MASK(0, 2)
#define FMC_ADC_CORE_CTL_FSM_CMD_SHIFT        0
#define FMC_ADC_CORE_CTL_FSM_CMD_W(value)     WBGEN2_GEN_WRITE(value, 0, 2)
#define FMC_ADC_CORE_CTL_FSM_CMD_R(reg)       WBGEN2_GEN_READ(reg, 0, 2)

/* definitions for field: FMC Si750 output enable in reg: Control register */
#define FMC_ADC_CORE_CTL_FMC_CLK_OE           WBGEN2_GEN_MASK(2, 1)

/* definitions for field: Offset DACs clear (active low) in reg: Control register */
#define FMC_ADC_CORE_CTL_OFFSET_DAC_CLR_N     WBGEN2_GEN_MASK(3, 1)

/* definitions for field: Manual serdes bitslip (ignore on read) in reg: Control register */
#define FMC_ADC_CORE_CTL_MAN_BITSLIP          WBGEN2_GEN_MASK(4, 1)

/* definitions for field: Enable test data in reg: Control register */
#define FMC_ADC_CORE_CTL_TEST_DATA_EN         WBGEN2_GEN_MASK(5, 1)

/* definitions for field: Manual TRIG LED in reg: Control register */
#define FMC_ADC_CORE_CTL_TRIG_LED             WBGEN2_GEN_MASK(6, 1)

/* definitions for field: Manual ACQ LED in reg: Control register */
#define FMC_ADC_CORE_CTL_ACQ_LED              WBGEN2_GEN_MASK(7, 1)

/* definitions for field: Reserved in reg: Control register */
#define FMC_ADC_CORE_CTL_RESERVED_MASK        WBGEN2_GEN_MASK(8, 24)
#define FMC_ADC_CORE_CTL_RESERVED_SHIFT       8
#define FMC_ADC_CORE_CTL_RESERVED_W(value)    WBGEN2_GEN_WRITE(value, 8, 24)
#define FMC_ADC_CORE_CTL_RESERVED_R(reg)      WBGEN2_GEN_READ(reg, 8, 24)

/* definitions for register: Status register */

/* definitions for field: State machine status in reg: Status register */
#define FMC_ADC_CORE_STA_FSM_MASK             WBGEN2_GEN_MASK(0, 3)
#define FMC_ADC_CORE_STA_FSM_SHIFT            0
#define FMC_ADC_CORE_STA_FSM_W(value)         WBGEN2_GEN_WRITE(value, 0, 3)
#define FMC_ADC_CORE_STA_FSM_R(reg)           WBGEN2_GEN_READ(reg, 0, 3)

/* definitions for field: SerDes PLL status in reg: Status register */
#define FMC_ADC_CORE_STA_SERDES_PLL           WBGEN2_GEN_MASK(3, 1)

/* definitions for field: SerDes synchronization status in reg: Status register */
#define FMC_ADC_CORE_STA_SERDES_SYNCED        WBGEN2_GEN_MASK(4, 1)

/* definitions for field: Acquisition configuration status in reg: Status register */
#define FMC_ADC_CORE_STA_ACQ_CFG              WBGEN2_GEN_MASK(5, 1)

/* definitions for field: Reserved in reg: Status register */
#define FMC_ADC_CORE_STA_RESERVED_MASK        WBGEN2_GEN_MASK(6, 26)
#define FMC_ADC_CORE_STA_RESERVED_SHIFT       6
#define FMC_ADC_CORE_STA_RESERVED_W(value)    WBGEN2_GEN_WRITE(value, 6, 26)
#define FMC_ADC_CORE_STA_RESERVED_R(reg)      WBGEN2_GEN_READ(reg, 6, 26)

/* definitions for register: Trigger configuration */

/* definitions for field: Hardware trigger selection in reg: Trigger configuration */
#define FMC_ADC_CORE_TRIG_CFG_HW_TRIG_SEL     WBGEN2_GEN_MASK(0, 1)

/* definitions for field: Hardware trigger polarity in reg: Trigger configuration */
#define FMC_ADC_CORE_TRIG_CFG_HW_TRIG_POL     WBGEN2_GEN_MASK(1, 1)

/* definitions for field: Hardware trigger enable in reg: Trigger configuration */
#define FMC_ADC_CORE_TRIG_CFG_HW_TRIG_EN      WBGEN2_GEN_MASK(2, 1)

/* definitions for field: Software trigger enable in reg: Trigger configuration */
#define FMC_ADC_CORE_TRIG_CFG_SW_TRIG_EN      WBGEN2_GEN_MASK(3, 1)

/* definitions for field: Channel selection for internal trigger in reg: Trigger configuration */
#define FMC_ADC_CORE_TRIG_CFG_INT_TRIG_SEL_MASK WBGEN2_GEN_MASK(4, 2)
#define FMC_ADC_CORE_TRIG_CFG_INT_TRIG_SEL_SHIFT 4
#define FMC_ADC_CORE_TRIG_CFG_INT_TRIG_SEL_W(value) WBGEN2_GEN_WRITE(value, 4, 2)
#define FMC_ADC_CORE_TRIG_CFG_INT_TRIG_SEL_R(reg) WBGEN2_GEN_READ(reg, 4, 2)

/* definitions for field: Reserved in reg: Trigger configuration */
#define FMC_ADC_CORE_TRIG_CFG_RESERVED_MASK   WBGEN2_GEN_MASK(6, 10)
#define FMC_ADC_CORE_TRIG_CFG_RESERVED_SHIFT  6
#define FMC_ADC_CORE_TRIG_CFG_RESERVED_W(value) WBGEN2_GEN_WRITE(value, 6, 10)
#define FMC_ADC_CORE_TRIG_CFG_RESERVED_R(reg) WBGEN2_GEN_READ(reg, 6, 10)

/* definitions for field: Threshold for internal trigger in reg: Trigger configuration */
#define FMC_ADC_CORE_TRIG_CFG_INT_TRIG_THRES_MASK WBGEN2_GEN_MASK(16, 16)
#define FMC_ADC_CORE_TRIG_CFG_INT_TRIG_THRES_SHIFT 16
#define FMC_ADC_CORE_TRIG_CFG_INT_TRIG_THRES_W(value) WBGEN2_GEN_WRITE(value, 16, 16)
#define FMC_ADC_CORE_TRIG_CFG_INT_TRIG_THRES_R(reg) WBGEN2_GEN_READ(reg, 16, 16)

/* definitions for register: Trigger delay */

/* definitions for register: Software trigger */

/* definitions for register: Number of shots */

/* definitions for field: Number of shots in reg: Number of shots */
#define FMC_ADC_CORE_SHOTS_NB_MASK            WBGEN2_GEN_MASK(0, 16)
#define FMC_ADC_CORE_SHOTS_NB_SHIFT           0
#define FMC_ADC_CORE_SHOTS_NB_W(value)        WBGEN2_GEN_WRITE(value, 0, 16)
#define FMC_ADC_CORE_SHOTS_NB_R(reg)          WBGEN2_GEN_READ(reg, 0, 16)

/* definitions for field: Reserved in reg: Number of shots */
#define FMC_ADC_CORE_SHOTS_RESERVED_MASK      WBGEN2_GEN_MASK(16, 16)
#define FMC_ADC_CORE_SHOTS_RESERVED_SHIFT     16
#define FMC_ADC_CORE_SHOTS_RESERVED_W(value)  WBGEN2_GEN_WRITE(value, 16, 16)
#define FMC_ADC_CORE_SHOTS_RESERVED_R(reg)    WBGEN2_GEN_READ(reg, 16, 16)

/* definitions for register: Trigger address register */

/* definitions for register: Sample rate */

/* definitions for field: Sample rate decimation in reg: Sample rate */
#define FMC_ADC_CORE_SR_DECI_MASK             WBGEN2_GEN_MASK(0, 32)
#define FMC_ADC_CORE_SR_DECI_SHIFT            0
#define FMC_ADC_CORE_SR_DECI_W(value)         WBGEN2_GEN_WRITE(value, 0, 32)
#define FMC_ADC_CORE_SR_DECI_R(reg)           WBGEN2_GEN_READ(reg, 0, 32)

/* definitions for register: Pre-trigger samples */

/* definitions for register: Post-trigger samples */

/* definitions for register: Samples counter */

/* definitions for register: Channel 1 control register */

/* definitions for field: Solid state relays control for channel 1 in reg: Channel 1 control register */
#define FMC_ADC_CORE_CH1_CTL_SSR_MASK         WBGEN2_GEN_MASK(0, 7)
#define FMC_ADC_CORE_CH1_CTL_SSR_SHIFT        0
#define FMC_ADC_CORE_CH1_CTL_SSR_W(value)     WBGEN2_GEN_WRITE(value, 0, 7)
#define FMC_ADC_CORE_CH1_CTL_SSR_R(reg)       WBGEN2_GEN_READ(reg, 0, 7)

/* definitions for field: Reserved in reg: Channel 1 control register */
#define FMC_ADC_CORE_CH1_CTL_RESERVED_MASK    WBGEN2_GEN_MASK(7, 25)
#define FMC_ADC_CORE_CH1_CTL_RESERVED_SHIFT   7
#define FMC_ADC_CORE_CH1_CTL_RESERVED_W(value) WBGEN2_GEN_WRITE(value, 7, 25)
#define FMC_ADC_CORE_CH1_CTL_RESERVED_R(reg)  WBGEN2_GEN_READ(reg, 7, 25)

/* definitions for register: Channel 1 status register */

/* definitions for field: Channel 1 current ADC value in reg: Channel 1 status register */
#define FMC_ADC_CORE_CH1_STA_VAL_MASK         WBGEN2_GEN_MASK(0, 16)
#define FMC_ADC_CORE_CH1_STA_VAL_SHIFT        0
#define FMC_ADC_CORE_CH1_STA_VAL_W(value)     WBGEN2_GEN_WRITE(value, 0, 16)
#define FMC_ADC_CORE_CH1_STA_VAL_R(reg)       WBGEN2_GEN_READ(reg, 0, 16)

/* definitions for field: Reserved in reg: Channel 1 status register */
#define FMC_ADC_CORE_CH1_STA_RESERVED_MASK    WBGEN2_GEN_MASK(16, 16)
#define FMC_ADC_CORE_CH1_STA_RESERVED_SHIFT   16
#define FMC_ADC_CORE_CH1_STA_RESERVED_W(value) WBGEN2_GEN_WRITE(value, 16, 16)
#define FMC_ADC_CORE_CH1_STA_RESERVED_R(reg)  WBGEN2_GEN_READ(reg, 16, 16)

/* definitions for register: Channel 1 gain calibration register */

/* definitions for field: Gain calibration for channel 1 in reg: Channel 1 gain calibration register */
#define FMC_ADC_CORE_CH1_GAIN_VAL_MASK        WBGEN2_GEN_MASK(0, 16)
#define FMC_ADC_CORE_CH1_GAIN_VAL_SHIFT       0
#define FMC_ADC_CORE_CH1_GAIN_VAL_W(value)    WBGEN2_GEN_WRITE(value, 0, 16)
#define FMC_ADC_CORE_CH1_GAIN_VAL_R(reg)      WBGEN2_GEN_READ(reg, 0, 16)

/* definitions for field: Reserved in reg: Channel 1 gain calibration register */
#define FMC_ADC_CORE_CH1_GAIN_RESERVED_MASK   WBGEN2_GEN_MASK(16, 16)
#define FMC_ADC_CORE_CH1_GAIN_RESERVED_SHIFT  16
#define FMC_ADC_CORE_CH1_GAIN_RESERVED_W(value) WBGEN2_GEN_WRITE(value, 16, 16)
#define FMC_ADC_CORE_CH1_GAIN_RESERVED_R(reg) WBGEN2_GEN_READ(reg, 16, 16)

/* definitions for register: Channel 1 offset calibration register */

/* definitions for field: Offset calibration for channel 1 in reg: Channel 1 offset calibration register */
#define FMC_ADC_CORE_CH1_OFFSET_VAL_MASK      WBGEN2_GEN_MASK(0, 16)
#define FMC_ADC_CORE_CH1_OFFSET_VAL_SHIFT     0
#define FMC_ADC_CORE_CH1_OFFSET_VAL_W(value)  WBGEN2_GEN_WRITE(value, 0, 16)
#define FMC_ADC_CORE_CH1_OFFSET_VAL_R(reg)    WBGEN2_GEN_READ(reg, 0, 16)

/* definitions for field: Reserved in reg: Channel 1 offset calibration register */
#define FMC_ADC_CORE_CH1_OFFSET_RESERVED_MASK WBGEN2_GEN_MASK(16, 16)
#define FMC_ADC_CORE_CH1_OFFSET_RESERVED_SHIFT 16
#define FMC_ADC_CORE_CH1_OFFSET_RESERVED_W(value) WBGEN2_GEN_WRITE(value, 16, 16)
#define FMC_ADC_CORE_CH1_OFFSET_RESERVED_R(reg) WBGEN2_GEN_READ(reg, 16, 16)

/* definitions for register: Channel 2 control register */

/* definitions for field: Solid state relays control for channel 2 in reg: Channel 2 control register */
#define FMC_ADC_CORE_CH2_CTL_SSR_MASK         WBGEN2_GEN_MASK(0, 7)
#define FMC_ADC_CORE_CH2_CTL_SSR_SHIFT        0
#define FMC_ADC_CORE_CH2_CTL_SSR_W(value)     WBGEN2_GEN_WRITE(value, 0, 7)
#define FMC_ADC_CORE_CH2_CTL_SSR_R(reg)       WBGEN2_GEN_READ(reg, 0, 7)

/* definitions for field: Reserved in reg: Channel 2 control register */
#define FMC_ADC_CORE_CH2_CTL_RESERVED_MASK    WBGEN2_GEN_MASK(7, 25)
#define FMC_ADC_CORE_CH2_CTL_RESERVED_SHIFT   7
#define FMC_ADC_CORE_CH2_CTL_RESERVED_W(value) WBGEN2_GEN_WRITE(value, 7, 25)
#define FMC_ADC_CORE_CH2_CTL_RESERVED_R(reg)  WBGEN2_GEN_READ(reg, 7, 25)

/* definitions for register: Channel 2 status register */

/* definitions for field: Channel 2 current ACD value in reg: Channel 2 status register */
#define FMC_ADC_CORE_CH2_STA_VAL_MASK         WBGEN2_GEN_MASK(0, 16)
#define FMC_ADC_CORE_CH2_STA_VAL_SHIFT        0
#define FMC_ADC_CORE_CH2_STA_VAL_W(value)     WBGEN2_GEN_WRITE(value, 0, 16)
#define FMC_ADC_CORE_CH2_STA_VAL_R(reg)       WBGEN2_GEN_READ(reg, 0, 16)

/* definitions for field: Reserved in reg: Channel 2 status register */
#define FMC_ADC_CORE_CH2_STA_RESERVED_MASK    WBGEN2_GEN_MASK(16, 16)
#define FMC_ADC_CORE_CH2_STA_RESERVED_SHIFT   16
#define FMC_ADC_CORE_CH2_STA_RESERVED_W(value) WBGEN2_GEN_WRITE(value, 16, 16)
#define FMC_ADC_CORE_CH2_STA_RESERVED_R(reg)  WBGEN2_GEN_READ(reg, 16, 16)

/* definitions for register: Channel 2 gain calibration register */

/* definitions for field: Gain calibration for channel 2 in reg: Channel 2 gain calibration register */
#define FMC_ADC_CORE_CH2_GAIN_VAL_MASK        WBGEN2_GEN_MASK(0, 16)
#define FMC_ADC_CORE_CH2_GAIN_VAL_SHIFT       0
#define FMC_ADC_CORE_CH2_GAIN_VAL_W(value)    WBGEN2_GEN_WRITE(value, 0, 16)
#define FMC_ADC_CORE_CH2_GAIN_VAL_R(reg)      WBGEN2_GEN_READ(reg, 0, 16)

/* definitions for field: Reserved in reg: Channel 2 gain calibration register */
#define FMC_ADC_CORE_CH2_GAIN_RESERVED_MASK   WBGEN2_GEN_MASK(16, 16)
#define FMC_ADC_CORE_CH2_GAIN_RESERVED_SHIFT  16
#define FMC_ADC_CORE_CH2_GAIN_RESERVED_W(value) WBGEN2_GEN_WRITE(value, 16, 16)
#define FMC_ADC_CORE_CH2_GAIN_RESERVED_R(reg) WBGEN2_GEN_READ(reg, 16, 16)

/* definitions for register: Channel 2 offset calibration register */

/* definitions for field: Offset calibration for channel 2 in reg: Channel 2 offset calibration register */
#define FMC_ADC_CORE_CH2_OFFSET_VAL_MASK      WBGEN2_GEN_MASK(0, 16)
#define FMC_ADC_CORE_CH2_OFFSET_VAL_SHIFT     0
#define FMC_ADC_CORE_CH2_OFFSET_VAL_W(value)  WBGEN2_GEN_WRITE(value, 0, 16)
#define FMC_ADC_CORE_CH2_OFFSET_VAL_R(reg)    WBGEN2_GEN_READ(reg, 0, 16)

/* definitions for field: Reserved in reg: Channel 2 offset calibration register */
#define FMC_ADC_CORE_CH2_OFFSET_RESERVED_MASK WBGEN2_GEN_MASK(16, 16)
#define FMC_ADC_CORE_CH2_OFFSET_RESERVED_SHIFT 16
#define FMC_ADC_CORE_CH2_OFFSET_RESERVED_W(value) WBGEN2_GEN_WRITE(value, 16, 16)
#define FMC_ADC_CORE_CH2_OFFSET_RESERVED_R(reg) WBGEN2_GEN_READ(reg, 16, 16)

/* definitions for register: Channel 3 control register */

/* definitions for field: Solid state relays control for channel 3 in reg: Channel 3 control register */
#define FMC_ADC_CORE_CH3_CTL_SSR_MASK         WBGEN2_GEN_MASK(0, 7)
#define FMC_ADC_CORE_CH3_CTL_SSR_SHIFT        0
#define FMC_ADC_CORE_CH3_CTL_SSR_W(value)     WBGEN2_GEN_WRITE(value, 0, 7)
#define FMC_ADC_CORE_CH3_CTL_SSR_R(reg)       WBGEN2_GEN_READ(reg, 0, 7)

/* definitions for field: Reserved in reg: Channel 3 control register */
#define FMC_ADC_CORE_CH3_CTL_RESERVED_MASK    WBGEN2_GEN_MASK(7, 25)
#define FMC_ADC_CORE_CH3_CTL_RESERVED_SHIFT   7
#define FMC_ADC_CORE_CH3_CTL_RESERVED_W(value) WBGEN2_GEN_WRITE(value, 7, 25)
#define FMC_ADC_CORE_CH3_CTL_RESERVED_R(reg)  WBGEN2_GEN_READ(reg, 7, 25)

/* definitions for register: Channel 3 status register */

/* definitions for field: Channel 3 current ADC value in reg: Channel 3 status register */
#define FMC_ADC_CORE_CH3_STA_VAL_MASK         WBGEN2_GEN_MASK(0, 16)
#define FMC_ADC_CORE_CH3_STA_VAL_SHIFT        0
#define FMC_ADC_CORE_CH3_STA_VAL_W(value)     WBGEN2_GEN_WRITE(value, 0, 16)
#define FMC_ADC_CORE_CH3_STA_VAL_R(reg)       WBGEN2_GEN_READ(reg, 0, 16)

/* definitions for field: Reserved in reg: Channel 3 status register */
#define FMC_ADC_CORE_CH3_STA_RESERVED_MASK    WBGEN2_GEN_MASK(16, 16)
#define FMC_ADC_CORE_CH3_STA_RESERVED_SHIFT   16
#define FMC_ADC_CORE_CH3_STA_RESERVED_W(value) WBGEN2_GEN_WRITE(value, 16, 16)
#define FMC_ADC_CORE_CH3_STA_RESERVED_R(reg)  WBGEN2_GEN_READ(reg, 16, 16)

/* definitions for register: Channel 3 gain calibration register */

/* definitions for field: Gain calibration for channel 3 in reg: Channel 3 gain calibration register */
#define FMC_ADC_CORE_CH3_GAIN_VAL_MASK        WBGEN2_GEN_MASK(0, 16)
#define FMC_ADC_CORE_CH3_GAIN_VAL_SHIFT       0
#define FMC_ADC_CORE_CH3_GAIN_VAL_W(value)    WBGEN2_GEN_WRITE(value, 0, 16)
#define FMC_ADC_CORE_CH3_GAIN_VAL_R(reg)      WBGEN2_GEN_READ(reg, 0, 16)

/* definitions for field: Reserved in reg: Channel 3 gain calibration register */
#define FMC_ADC_CORE_CH3_GAIN_RESERVED_MASK   WBGEN2_GEN_MASK(16, 16)
#define FMC_ADC_CORE_CH3_GAIN_RESERVED_SHIFT  16
#define FMC_ADC_CORE_CH3_GAIN_RESERVED_W(value) WBGEN2_GEN_WRITE(value, 16, 16)
#define FMC_ADC_CORE_CH3_GAIN_RESERVED_R(reg) WBGEN2_GEN_READ(reg, 16, 16)

/* definitions for register: Channel 3 offset calibration register */

/* definitions for field: Offset calibration for channel 3 in reg: Channel 3 offset calibration register */
#define FMC_ADC_CORE_CH3_OFFSET_VAL_MASK      WBGEN2_GEN_MASK(0, 16)
#define FMC_ADC_CORE_CH3_OFFSET_VAL_SHIFT     0
#define FMC_ADC_CORE_CH3_OFFSET_VAL_W(value)  WBGEN2_GEN_WRITE(value, 0, 16)
#define FMC_ADC_CORE_CH3_OFFSET_VAL_R(reg)    WBGEN2_GEN_READ(reg, 0, 16)

/* definitions for field: Reserved in reg: Channel 3 offset calibration register */
#define FMC_ADC_CORE_CH3_OFFSET_RESERVED_MASK WBGEN2_GEN_MASK(16, 16)
#define FMC_ADC_CORE_CH3_OFFSET_RESERVED_SHIFT 16
#define FMC_ADC_CORE_CH3_OFFSET_RESERVED_W(value) WBGEN2_GEN_WRITE(value, 16, 16)
#define FMC_ADC_CORE_CH3_OFFSET_RESERVED_R(reg) WBGEN2_GEN_READ(reg, 16, 16)

/* definitions for register: Channel 4 control register */

/* definitions for field: Solid state relays control for channel 4 in reg: Channel 4 control register */
#define FMC_ADC_CORE_CH4_CTL_SSR_MASK         WBGEN2_GEN_MASK(0, 7)
#define FMC_ADC_CORE_CH4_CTL_SSR_SHIFT        0
#define FMC_ADC_CORE_CH4_CTL_SSR_W(value)     WBGEN2_GEN_WRITE(value, 0, 7)
#define FMC_ADC_CORE_CH4_CTL_SSR_R(reg)       WBGEN2_GEN_READ(reg, 0, 7)

/* definitions for field: Reserved in reg: Channel 4 control register */
#define FMC_ADC_CORE_CH4_CTL_RESERVED_MASK    WBGEN2_GEN_MASK(7, 25)
#define FMC_ADC_CORE_CH4_CTL_RESERVED_SHIFT   7
#define FMC_ADC_CORE_CH4_CTL_RESERVED_W(value) WBGEN2_GEN_WRITE(value, 7, 25)
#define FMC_ADC_CORE_CH4_CTL_RESERVED_R(reg)  WBGEN2_GEN_READ(reg, 7, 25)

/* definitions for register: Channel 4 status register */

/* definitions for field: Channel 4 current ADC value in reg: Channel 4 status register */
#define FMC_ADC_CORE_CH4_STA_VAL_MASK         WBGEN2_GEN_MASK(0, 16)
#define FMC_ADC_CORE_CH4_STA_VAL_SHIFT        0
#define FMC_ADC_CORE_CH4_STA_VAL_W(value)     WBGEN2_GEN_WRITE(value, 0, 16)
#define FMC_ADC_CORE_CH4_STA_VAL_R(reg)       WBGEN2_GEN_READ(reg, 0, 16)

/* definitions for field: Reserved in reg: Channel 4 status register */
#define FMC_ADC_CORE_CH4_STA_RESERVED_MASK    WBGEN2_GEN_MASK(16, 16)
#define FMC_ADC_CORE_CH4_STA_RESERVED_SHIFT   16
#define FMC_ADC_CORE_CH4_STA_RESERVED_W(value) WBGEN2_GEN_WRITE(value, 16, 16)
#define FMC_ADC_CORE_CH4_STA_RESERVED_R(reg)  WBGEN2_GEN_READ(reg, 16, 16)

/* definitions for register: Channel 4 gain calibration register */

/* definitions for field: Gain calibration for channel 4 in reg: Channel 4 gain calibration register */
#define FMC_ADC_CORE_CH4_GAIN_VAL_MASK        WBGEN2_GEN_MASK(0, 16)
#define FMC_ADC_CORE_CH4_GAIN_VAL_SHIFT       0
#define FMC_ADC_CORE_CH4_GAIN_VAL_W(value)    WBGEN2_GEN_WRITE(value, 0, 16)
#define FMC_ADC_CORE_CH4_GAIN_VAL_R(reg)      WBGEN2_GEN_READ(reg, 0, 16)

/* definitions for field: Reserved in reg: Channel 4 gain calibration register */
#define FMC_ADC_CORE_CH4_GAIN_RESERVED_MASK   WBGEN2_GEN_MASK(16, 16)
#define FMC_ADC_CORE_CH4_GAIN_RESERVED_SHIFT  16
#define FMC_ADC_CORE_CH4_GAIN_RESERVED_W(value) WBGEN2_GEN_WRITE(value, 16, 16)
#define FMC_ADC_CORE_CH4_GAIN_RESERVED_R(reg) WBGEN2_GEN_READ(reg, 16, 16)

/* definitions for register: Channel 4 offset calibration register */

/* definitions for field: Offset calibration for channel 4 in reg: Channel 4 offset calibration register */
#define FMC_ADC_CORE_CH4_OFFSET_VAL_MASK      WBGEN2_GEN_MASK(0, 16)
#define FMC_ADC_CORE_CH4_OFFSET_VAL_SHIFT     0
#define FMC_ADC_CORE_CH4_OFFSET_VAL_W(value)  WBGEN2_GEN_WRITE(value, 0, 16)
#define FMC_ADC_CORE_CH4_OFFSET_VAL_R(reg)    WBGEN2_GEN_READ(reg, 0, 16)

/* definitions for field: Reserved in reg: Channel 4 offset calibration register */
#define FMC_ADC_CORE_CH4_OFFSET_RESERVED_MASK WBGEN2_GEN_MASK(16, 16)
#define FMC_ADC_CORE_CH4_OFFSET_RESERVED_SHIFT 16
#define FMC_ADC_CORE_CH4_OFFSET_RESERVED_W(value) WBGEN2_GEN_WRITE(value, 16, 16)
#define FMC_ADC_CORE_CH4_OFFSET_RESERVED_R(reg) WBGEN2_GEN_READ(reg, 16, 16)

PACKED struct FMC_ADC_CORE_WB {
  /* [0x0]: REG Control register */
  uint32_t CTL;
  /* [0x4]: REG Status register */
  uint32_t STA;
  /* [0x8]: REG Trigger configuration */
  uint32_t TRIG_CFG;
  /* [0xc]: REG Trigger delay */
  uint32_t TRIG_DLY;
  /* [0x10]: REG Software trigger */
  uint32_t SW_TRIG;
  /* [0x14]: REG Number of shots */
  uint32_t SHOTS;
  /* [0x18]: REG Trigger address register */
  uint32_t TRIG_POS;
  /* [0x1c]: REG Sample rate */
  uint32_t SR;
  /* [0x20]: REG Pre-trigger samples */
  uint32_t PRE_SAMPLES;
  /* [0x24]: REG Post-trigger samples */
  uint32_t POST_SAMPLES;
  /* [0x28]: REG Samples counter */
  uint32_t SAMPLES_CNT;
  /* [0x2c]: REG Channel 1 control register */
  uint32_t CH1_CTL;
  /* [0x30]: REG Channel 1 status register */
  uint32_t CH1_STA;
  /* [0x34]: REG Channel 1 gain calibration register */
  uint32_t CH1_GAIN;
  /* [0x38]: REG Channel 1 offset calibration register */
  uint32_t CH1_OFFSET;
  /* [0x3c]: REG Channel 2 control register */
  uint32_t CH2_CTL;
  /* [0x40]: REG Channel 2 status register */
  uint32_t CH2_STA;
  /* [0x44]: REG Channel 2 gain calibration register */
  uint32_t CH2_GAIN;
  /* [0x48]: REG Channel 2 offset calibration register */
  uint32_t CH2_OFFSET;
  /* [0x4c]: REG Channel 3 control register */
  uint32_t CH3_CTL;
  /* [0x50]: REG Channel 3 status register */
  uint32_t CH3_STA;
  /* [0x54]: REG Channel 3 gain calibration register */
  uint32_t CH3_GAIN;
  /* [0x58]: REG Channel 3 offset calibration register */
  uint32_t CH3_OFFSET;
  /* [0x5c]: REG Channel 4 control register */
  uint32_t CH4_CTL;
  /* [0x60]: REG Channel 4 status register */
  uint32_t CH4_STA;
  /* [0x64]: REG Channel 4 gain calibration register */
  uint32_t CH4_GAIN;
  /* [0x68]: REG Channel 4 offset calibration register */
  uint32_t CH4_OFFSET;
};

#endif
