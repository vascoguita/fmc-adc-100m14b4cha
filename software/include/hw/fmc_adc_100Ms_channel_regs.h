// SPDX-FileCopyrightText: 2022 CERN (home.cern)
// SPDX-License-Identifier: GPL-2.0-or-later

#ifndef __CHEBY__FMC_ADC_100MS_CHANNEL_REGS__H__
#define __CHEBY__FMC_ADC_100MS_CHANNEL_REGS__H__
#define FMC_ADC_100MS_CHANNEL_REGS_SIZE 32 /* 0x20 */

/* Channel control register */
#define FMC_ADC_100MS_CHANNEL_REGS_CTL 0x0UL
#define FMC_ADC_100MS_CHANNEL_REGS_CTL_SSR_MASK 0x7fUL
#define FMC_ADC_100MS_CHANNEL_REGS_CTL_SSR_SHIFT 0

/* Channel status register */
#define FMC_ADC_100MS_CHANNEL_REGS_STA 0x4UL
#define FMC_ADC_100MS_CHANNEL_REGS_STA_VAL_MASK 0xffffUL
#define FMC_ADC_100MS_CHANNEL_REGS_STA_VAL_SHIFT 0

/* Channel calibration register */
#define FMC_ADC_100MS_CHANNEL_REGS_CALIB 0x8UL
#define FMC_ADC_100MS_CHANNEL_REGS_CALIB_GAIN_MASK 0xffffUL
#define FMC_ADC_100MS_CHANNEL_REGS_CALIB_GAIN_SHIFT 0
#define FMC_ADC_100MS_CHANNEL_REGS_CALIB_OFFSET_MASK 0xffff0000UL
#define FMC_ADC_100MS_CHANNEL_REGS_CALIB_OFFSET_SHIFT 16

/* Channel saturation register */
#define FMC_ADC_100MS_CHANNEL_REGS_SAT 0xcUL
#define FMC_ADC_100MS_CHANNEL_REGS_SAT_VAL_MASK 0x7fffUL
#define FMC_ADC_100MS_CHANNEL_REGS_SAT_VAL_SHIFT 0

/* Channel trigger threshold configuration register */
#define FMC_ADC_100MS_CHANNEL_REGS_TRIG_THRES 0x10UL
#define FMC_ADC_100MS_CHANNEL_REGS_TRIG_THRES_VAL_MASK 0xffffUL
#define FMC_ADC_100MS_CHANNEL_REGS_TRIG_THRES_VAL_SHIFT 0
#define FMC_ADC_100MS_CHANNEL_REGS_TRIG_THRES_HYST_MASK 0xffff0000UL
#define FMC_ADC_100MS_CHANNEL_REGS_TRIG_THRES_HYST_SHIFT 16

/* Channel trigger delay */
#define FMC_ADC_100MS_CHANNEL_REGS_TRIG_DLY 0x14UL

/* Channel calibration value (read from hw) */
#define FMC_ADC_100MS_CHANNEL_REGS_CALIB_VAL 0x18UL
#define FMC_ADC_100MS_CHANNEL_REGS_CALIB_VAL_GAIN_MASK 0xffffUL
#define FMC_ADC_100MS_CHANNEL_REGS_CALIB_VAL_GAIN_SHIFT 0
#define FMC_ADC_100MS_CHANNEL_REGS_CALIB_VAL_OFFSET_MASK 0xffff0000UL
#define FMC_ADC_100MS_CHANNEL_REGS_CALIB_VAL_OFFSET_SHIFT 16

/* Channel saturation register (read from hw) */
#define FMC_ADC_100MS_CHANNEL_REGS_SAT_VAL 0x1cUL
#define FMC_ADC_100MS_CHANNEL_REGS_SAT_VAL_VAL_MASK 0x7fffUL
#define FMC_ADC_100MS_CHANNEL_REGS_SAT_VAL_VAL_SHIFT 0

struct fmc_adc_100ms_channel_regs {
  /* [0x0]: REG (rw) Channel control register */
  uint32_t ctl;

  /* [0x4]: REG (ro) Channel status register */
  uint32_t sta;

  /* [0x8]: REG (rw) Channel calibration register */
  uint32_t calib;

  /* [0xc]: REG (rw) Channel saturation register */
  uint32_t sat;

  /* [0x10]: REG (rw) Channel trigger threshold configuration register */
  uint32_t trig_thres;

  /* [0x14]: REG (rw) Channel trigger delay */
  uint32_t trig_dly;

  /* [0x18]: REG (ro) Channel calibration value (read from hw) */
  uint32_t calib_val;

  /* [0x1c]: REG (ro) Channel saturation register (read from hw) */
  uint32_t sat_val;
};

#endif /* __CHEBY__FMC_ADC_100MS_CHANNEL_REGS__H__ */
