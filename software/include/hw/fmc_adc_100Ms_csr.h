#ifndef __CHEBY__FMC_ADC_100MS_CSR__H__
#define __CHEBY__FMC_ADC_100MS_CSR__H__

#include "fmc_adc_100ms_channel_regs.h"
#define FMC_ADC_100MS_CSR_SIZE 512 /* 0x200 */

/* Control register */
#define FMC_ADC_100MS_CSR_CTL 0x0UL
#define FMC_ADC_100MS_CSR_CTL_FSM_CMD_MASK 0x3UL
#define FMC_ADC_100MS_CSR_CTL_FSM_CMD_SHIFT 0
#define FMC_ADC_100MS_CSR_CTL_FMC_CLK_OE 0x4UL
#define FMC_ADC_100MS_CSR_CTL_OFFSET_DAC_CLR_N 0x8UL
#define FMC_ADC_100MS_CSR_CTL_MAN_BITSLIP 0x10UL
#define FMC_ADC_100MS_CSR_CTL_TRIG_LED 0x40UL
#define FMC_ADC_100MS_CSR_CTL_ACQ_LED 0x80UL
#define FMC_ADC_100MS_CSR_CTL_CLEAR_TRIG_STAT 0x100UL
#define FMC_ADC_100MS_CSR_CTL_CALIB_APPLY 0x8000UL

/* Status register */
#define FMC_ADC_100MS_CSR_STA 0x4UL
#define FMC_ADC_100MS_CSR_STA_FSM_MASK 0x7UL
#define FMC_ADC_100MS_CSR_STA_FSM_SHIFT 0
#define FMC_ADC_100MS_CSR_STA_SERDES_PLL 0x8UL
#define FMC_ADC_100MS_CSR_STA_SERDES_SYNCED 0x10UL
#define FMC_ADC_100MS_CSR_STA_ACQ_CFG 0x20UL
#define FMC_ADC_100MS_CSR_STA_FMC_NR_MASK 0xc0UL
#define FMC_ADC_100MS_CSR_STA_FMC_NR_SHIFT 6
#define FMC_ADC_100MS_CSR_STA_CALIB_BUSY 0x8000UL

/* Trigger status */
#define FMC_ADC_100MS_CSR_TRIG_STAT 0x8UL
#define FMC_ADC_100MS_CSR_TRIG_STAT_EXT 0x1UL
#define FMC_ADC_100MS_CSR_TRIG_STAT_SW 0x2UL
#define FMC_ADC_100MS_CSR_TRIG_STAT_TIME 0x10UL
#define FMC_ADC_100MS_CSR_TRIG_STAT_CH1 0x100UL
#define FMC_ADC_100MS_CSR_TRIG_STAT_CH2 0x200UL
#define FMC_ADC_100MS_CSR_TRIG_STAT_CH3 0x400UL
#define FMC_ADC_100MS_CSR_TRIG_STAT_CH4 0x800UL

/* Trigger enable */
#define FMC_ADC_100MS_CSR_TRIG_EN 0xcUL
#define FMC_ADC_100MS_CSR_TRIG_EN_EXT 0x1UL
#define FMC_ADC_100MS_CSR_TRIG_EN_SW 0x2UL
#define FMC_ADC_100MS_CSR_TRIG_EN_TIME 0x10UL
#define FMC_ADC_100MS_CSR_TRIG_EN_AUX_TIME 0x20UL
#define FMC_ADC_100MS_CSR_TRIG_EN_CH1 0x100UL
#define FMC_ADC_100MS_CSR_TRIG_EN_CH2 0x200UL
#define FMC_ADC_100MS_CSR_TRIG_EN_CH3 0x400UL
#define FMC_ADC_100MS_CSR_TRIG_EN_CH4 0x800UL

/* Trigger polarity */
#define FMC_ADC_100MS_CSR_TRIG_POL 0x10UL
#define FMC_ADC_100MS_CSR_TRIG_POL_EXT 0x1UL
#define FMC_ADC_100MS_CSR_TRIG_POL_CH1 0x100UL
#define FMC_ADC_100MS_CSR_TRIG_POL_CH2 0x200UL
#define FMC_ADC_100MS_CSR_TRIG_POL_CH3 0x400UL
#define FMC_ADC_100MS_CSR_TRIG_POL_CH4 0x800UL

/* External trigger delay */
#define FMC_ADC_100MS_CSR_EXT_TRIG_DLY 0x14UL

/* Software trigger */
#define FMC_ADC_100MS_CSR_SW_TRIG 0x18UL

/* Number of shots */
#define FMC_ADC_100MS_CSR_SHOTS 0x1cUL
#define FMC_ADC_100MS_CSR_SHOTS_NBR_MASK 0xffffUL
#define FMC_ADC_100MS_CSR_SHOTS_NBR_SHIFT 0
#define FMC_ADC_100MS_CSR_SHOTS_REMAIN_MASK 0xffff0000UL
#define FMC_ADC_100MS_CSR_SHOTS_REMAIN_SHIFT 16

/* Multi-shot sample depth register */
#define FMC_ADC_100MS_CSR_MULTI_DEPTH 0x20UL

/* Trigger address register */
#define FMC_ADC_100MS_CSR_TRIG_POS 0x24UL

/* Sampling clock frequency */
#define FMC_ADC_100MS_CSR_FS_FREQ 0x28UL

/* Downsampling ratio */
#define FMC_ADC_100MS_CSR_DOWNSAMPLE 0x2cUL

/* Pre-trigger samples */
#define FMC_ADC_100MS_CSR_PRE_SAMPLES 0x30UL

/* Post-trigger samples */
#define FMC_ADC_100MS_CSR_POST_SAMPLES 0x34UL

/* Samples counter */
#define FMC_ADC_100MS_CSR_SAMPLES_CNT 0x38UL

/* Channel 1 registers */
#define FMC_ADC_100MS_CSR_FMC_ADC_CH1 0x80UL
#define FMC_ADC_100MS_CSR_FMC_ADC_CH1_SIZE 32 /* 0x20 */

/* Channel 2 registers */
#define FMC_ADC_100MS_CSR_FMC_ADC_CH2 0xc0UL
#define FMC_ADC_100MS_CSR_FMC_ADC_CH2_SIZE 32 /* 0x20 */

/* Channel 3 registers */
#define FMC_ADC_100MS_CSR_FMC_ADC_CH3 0x100UL
#define FMC_ADC_100MS_CSR_FMC_ADC_CH3_SIZE 32 /* 0x20 */

/* Channel 4 registers */
#define FMC_ADC_100MS_CSR_FMC_ADC_CH4 0x140UL
#define FMC_ADC_100MS_CSR_FMC_ADC_CH4_SIZE 32 /* 0x20 */

struct fmc_adc_100ms_csr {
  /* [0x0]: REG (rw) Control register */
  uint32_t ctl;

  /* [0x4]: REG (ro) Status register */
  uint32_t sta;

  /* [0x8]: REG (ro) Trigger status */
  uint32_t trig_stat;

  /* [0xc]: REG (rw) Trigger enable */
  uint32_t trig_en;

  /* [0x10]: REG (rw) Trigger polarity */
  uint32_t trig_pol;

  /* [0x14]: REG (rw) External trigger delay */
  uint32_t ext_trig_dly;

  /* [0x18]: REG (wo) Software trigger */
  uint32_t sw_trig;

  /* [0x1c]: REG (rw) Number of shots */
  uint32_t shots;

  /* [0x20]: REG (ro) Multi-shot sample depth register */
  uint32_t multi_depth;

  /* [0x24]: REG (ro) Trigger address register */
  uint32_t trig_pos;

  /* [0x28]: REG (ro) Sampling clock frequency */
  uint32_t fs_freq;

  /* [0x2c]: REG (rw) Downsampling ratio */
  uint32_t downsample;

  /* [0x30]: REG (rw) Pre-trigger samples */
  uint32_t pre_samples;

  /* [0x34]: REG (rw) Post-trigger samples */
  uint32_t post_samples;

  /* [0x38]: REG (ro) Samples counter */
  uint32_t samples_cnt;

  /* padding to: 32 words */
  uint32_t __padding_0[17];

  /* [0x80]: SUBMAP Channel 1 registers */
  struct fmc_adc_100ms_channel_regs fmc_adc_ch1;

  /* padding to: 48 words */
  uint32_t __padding_1[10];

  /* [0xc0]: SUBMAP Channel 2 registers */
  struct fmc_adc_100ms_channel_regs fmc_adc_ch2;

  /* padding to: 64 words */
  uint32_t __padding_2[10];

  /* [0x100]: SUBMAP Channel 3 registers */
  struct fmc_adc_100ms_channel_regs fmc_adc_ch3;

  /* padding to: 80 words */
  uint32_t __padding_3[10];

  /* [0x140]: SUBMAP Channel 4 registers */
  struct fmc_adc_100ms_channel_regs fmc_adc_ch4;

  /* padding to: 80 words */
  uint32_t __padding_4[42];
};

#endif /* __CHEBY__FMC_ADC_100MS_CSR__H__ */
