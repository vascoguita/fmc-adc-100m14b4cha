// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright CERN 2012-2019
 * Author: Federico Vaga <federico.vaga@gmail.com>
 */

#ifndef FMC_ADC_100M14B4C_H_
#define FMC_ADC_100M14B4C_H_

#ifndef __KERNEL__
#include <stdint.h>
#ifndef BIT
#define BIT(nr) (1UL << (nr))
#endif
#endif

/* Trigger sources */
#define FA100M14B4C_TRG_SRC_EXT BIT(0)
#define FA100M14B4C_TRG_SRC_SW BIT(1)
#define FA100M14B4C_TRG_SRC_TIM BIT(4)
#define FA100M14B4C_TRG_SRC_ALT BIT(5)
#define FA100M14B4C_TRG_SRC_CH1 BIT(8)
#define FA100M14B4C_TRG_SRC_CH2 BIT(9)
#define FA100M14B4C_TRG_SRC_CH3 BIT(10)
#define FA100M14B4C_TRG_SRC_CH4 BIT(11)
#define FA100M14B4C_TRG_SRC_CHx(_x) (FA100M14B4C_TRG_SRC_CH1 << ((_x) - 1))

/* Trigger Polarity */
#define FA100M14B4C_TRG_POL_EXT FA100M14B4C_TRG_SRC_EXT
#define FA100M14B4C_TRG_POL_CH1 FA100M14B4C_TRG_SRC_CH1
#define FA100M14B4C_TRG_POL_CH2 FA100M14B4C_TRG_SRC_CH2
#define FA100M14B4C_TRG_POL_CH3 FA100M14B4C_TRG_SRC_CH3
#define FA100M14B4C_TRG_POL_CH4 FA100M14B4C_TRG_SRC_CH4
#define FA100M14B4C_TRG_POL_CHx(_x) (FA100M14B4C_TRG_POL_CH1 << ((_x) - 1))

/*
 * Trigger Extended Attribute Enumeration
 */
enum fa100m14b4c_trg_ext_attr {
	/*
	 * The trigger extended attribute order is the same in the declaration
	 * and in the zio_control, so we can always use enumeration. But, the
	 * enumeration must start with 0 followed by only consecutive value.
	 *
	 * The parameters are not exposed to user space by zio_controle, so it
	 * is not necessary to export to user space the correspondent enum
	 */
	FA100M14B4C_TATTR_STA = 0,
	FA100M14B4C_TATTR_SRC,
	FA100M14B4C_TATTR_POL,
	FA100M14B4C_TATTR_EXT_DLY,
	FA100M14B4C_TATTR_CH1_THRES,
	FA100M14B4C_TATTR_CH2_THRES,
	FA100M14B4C_TATTR_CH3_THRES,
	FA100M14B4C_TATTR_CH4_THRES,
	FA100M14B4C_TATTR_CH1_HYST,
	FA100M14B4C_TATTR_CH2_HYST,
	FA100M14B4C_TATTR_CH3_HYST,
	FA100M14B4C_TATTR_CH4_HYST,
	FA100M14B4C_TATTR_CH1_DLY,
	FA100M14B4C_TATTR_CH2_DLY,
	FA100M14B4C_TATTR_CH3_DLY,
	FA100M14B4C_TATTR_CH4_DLY,
	FA100M14B4C_TATTR_TRG_TIM_SU,
	FA100M14B4C_TATTR_TRG_TIM_SL,
	FA100M14B4C_TATTR_TRG_TIM_C,
	__FA100M14B4C_TATTR_TRG_MAX,
};

/*
 * Device Extended Attribute Enumeration
 */
enum fa100m14b4c_dev_ext_attr {
	/*
	 * NOTE: At the moment the only extended attributes we have in
	 * the device hierarchy are in the cset level, so we can safely
	 * start from index 0
	 */
	FA100M14B4C_DATTR_DECI = 0,
	FA100M14B4C_DATTR_CH0_OFFSET,
	FA100M14B4C_DATTR_CH1_OFFSET,
	FA100M14B4C_DATTR_CH2_OFFSET,
	FA100M14B4C_DATTR_CH3_OFFSET,
	FA100M14B4C_DATTR_CH0_VREF,
	FA100M14B4C_DATTR_CH1_VREF,
	FA100M14B4C_DATTR_CH2_VREF,
	FA100M14B4C_DATTR_CH3_VREF,
	FA100M14B4C_DATTR_CH0_50TERM,
	FA100M14B4C_DATTR_CH1_50TERM,
	FA100M14B4C_DATTR_CH2_50TERM,
	FA100M14B4C_DATTR_CH3_50TERM,
	FA100M14B4C_DATTR_ACQ_START_S,
	FA100M14B4C_DATTR_ACQ_START_C,
	FA100M14B4C_DATTR_ACQ_START_F,
};

#define FA100M14B4C_UTC_CLOCK_FREQ 125000000
#define FA100M14B4C_UTC_CLOCK_NS  8
#define FA100M14B4C_NCHAN 4 /* We have 4 of them,no way out of it */
#define FA100M14B4C_NBIT 14

/* ADC DDR memory */
#define FA100M14B4C_MAX_ACQ_BYTE 0x10000000 /* 256MB */

enum fa100m14b4c_input_range {
	FA100M14B4C_RANGE_10V = 0x0,
	FA100M14B4C_RANGE_1V,
	FA100M14B4C_RANGE_100mV,
	FA100M14B4C_RANGE_OPEN,	/* Channel disconnected from ADC */
	FA100M14B4C_RANGE_10V_CAL,	/* Channel disconnected from ADC */
	FA100M14B4C_RANGE_1V_CAL,	/* Channel disconnected from ADC */
	FA100M14B4C_RANGE_100mV_CAL,	/* Channel disconnected from ADC */
};

enum fa100m14b4c_fsm_cmd {
	FA100M14B4C_CMD_NONE =	0x0,
	FA100M14B4C_CMD_START =	0x1,
	FA100M14B4C_CMD_STOP =	0x2,
};
/* All possible state of the state machine, other values are invalid*/
enum fa100m14b4c_fsm_state {
	FA100M14B4C_STATE_IDLE = 0x1,
	FA100M14B4C_STATE_PRE,
	FA100M14B4C_STATE_WAIT,
	FA100M14B4C_STATE_POST,
	FA100M14B4C_STATE_DECR,
};

/* ADC and DAC Calibration, from  EEPROM */
struct fa_calib_stanza {
	int16_t offset[4]; /* One per channel */
	uint16_t gain[4];  /* One per channel */
	uint16_t temperature;
};

#define FA_CALIB_STANZA_N 3
struct fa_calib {
	struct fa_calib_stanza adc[FA_CALIB_STANZA_N];  /* For input, one per range */
	struct fa_calib_stanza dac[FA_CALIB_STANZA_N];  /* For user offset, one per range */
};

#define FA_VERSION_DRV FA_VERSION_BLD
#define FA_VERSION_MAJ(_VER) ((_VER >> 24) & 0xFF)
#define FA_VERSION_MIN(_VER) ((_VER >> 16) & 0xFF)
#define FA_VERSION_PATCH(_VER) (_VER & 0xFFFF)

#define PCI_VENDOR_ID_CERN      (0x10DC)

#define FA_META_VENDOR_ID PCI_VENDOR_ID_CERN
#define FA_META_DEVICE_ID_SPEC 0x41444301
#define FA_META_DEVICE_ID_SVEC_DBL_ADC 0x41444302

#endif /*  FMC_ADC_H_ */
