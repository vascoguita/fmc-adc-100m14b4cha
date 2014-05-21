/*
 * The ADC library for the specific card
 *
 * Copyright (C) 2014 CERN (www.cern.ch)
 * Author: Federico Vaga <federico.vaga@cern.ch>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * version 2 as published by the Free Software Foundation or, at your
 * option, any later version.
 */

#ifndef FMCA_ADC_100MA4B4CHA_H_
#define FMCA_ADC_100MA4B4CHA_H_

#define FA_UTC_CLOCK_NS  8

/*
 * Trigger Extended Attribute Enumeration
 */
enum fa_trig_ext_attributes {
	/*
	 * The trigger extended attribute order is the same in the declaration
	 * and in the zio_control, so we can always use enumeration. But, the
	 * enumeration must start with 0 followed by only consecutive value.
	 *
	 * NOTE: this values are temporary copied from the adc driver, so
	 * do not change this enum
	 */
	ZFAT_ATTR_EXT = 0,
	ZFAT_ATTR_POL,
	ZFAT_ATTR_INT_CHAN,
	ZFAT_ATTR_INT_THRES,
	ZFAT_ATTR_DELAY,
};

/*
 * Device Extended Attribute Enumeration
 */
enum fa_dev_ext_attributes {
	/*
	 * NOTE: At the moment the only extended attributes we have in
	 * the device hierarchy are in the cset level, so we can safely
	 * start from index 0
	 * NOTE: this values are temporary copied from the adc driver, so
	 * do not change this enum
	 */
	ZFAD_ATTR_DECI = 0,
	ZFAD_ATTR_CH0_OFFSET,
	ZFAD_ATTR_CH1_OFFSET,
	ZFAD_ATTR_CH2_OFFSET,
	ZFAD_ATTR_CH3_OFFSET,
	ZFAD_ATTR_CH0_VREF,
	ZFAD_ATTR_CH1_VREF,
	ZFAD_ATTR_CH2_VREF,
	ZFAD_ATTR_CH3_VREF,
	ZFAD_ATTR_CH0_50TERM,
	ZFAD_ATTR_CH1_50TERM,
	ZFAD_ATTR_CH2_50TERM,
	ZFAD_ATTR_CH3_50TERM,
	ZFAD_ATTR_ACQ_START_S,
	ZFAD_ATTR_ACQ_START_C,
	ZFAD_ATTR_ACQ_START_F,
};

#endif /* FMCA_ADC_100MA4B4CHA_H_ */
