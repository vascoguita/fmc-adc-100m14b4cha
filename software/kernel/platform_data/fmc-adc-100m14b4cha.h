// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2020 CERN (www.cern.ch)
 * Author: Federico Vaga <federico.vaga@cern.ch>
 */

#ifndef __FMC_ADC_PDATA_H__
#define __FMC_ADC_PDATA_H__

#define FMC_ADC_BIG_ENDIAN BIT(0)
#define FMC_ADC_NOSQUASH_SCATTERLIST BIT(1)
/*
 * In principle this should not be necessary. The two variants should
 * be as close as possible to each other. But this is not the case, the DMA
 * interface is different and we need to distinguish between SPEC and SVEC.
 * NOTE any other carrier is not supported!
 */
#define FMC_ADC_SVEC BIT(3)

struct fmc_adc_platform_data {
	unsigned long flags;
	unsigned long vme_ddr_offset;
	uint8_t calib_trig_time;
	uint8_t calib_trig_threshold;
	uint8_t calib_trig_internal;
};

#endif
