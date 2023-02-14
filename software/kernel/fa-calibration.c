// SPDX-License-Identifier: GPL-2.0-or-later
// SPDX-FileCopyrightText: 2020 CERN (home.cern)

/*
 * EEPROM calibration block retreival code for fa-dev
 */
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/byteorder/generic.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/zio.h>
#include <linux/moduleparam.h>
#include <linux/jiffies.h>
#include "fmc-adc-100m14b4cha-private.h"

static int fa_calib_temp_period;
module_param_named(temp_calib_period, fa_calib_temp_period, int, 0444);

static int fa_calib_temp;
module_param_named(temp_calib, fa_calib_temp, int, 0444);

/* This identity calibration is used as default */
static const struct fa_calib_stanza fa_identity_calib = {
	.offset = { 0, },
	.gain = {0x8000, 0x8000, 0x8000, 0x8000},
	.temperature = 50 * 100, /* 50 celsius degrees */
};

/* Max difference from identity thing */
#define FA_CALIB_MAX_DELTA_OFFSET	0x1000
#define FA_CALIB_MAX_DELTA_GAIN		0x1000
#define FA_CALIB_MAX_DELTA_TEMP		(40 * 100) /* 10-90 celsius */

static bool fa_calib_is_busy(struct fa_dev *fa)
{
	return !!fa_readl(fa, fa->fa_adc_csr_base, &zfad_regs[ZFA_STA_CALIB_BUSY]);
}

static int fa_calib_apply(struct fa_dev *fa)
{
	if (fa_calib_is_busy(fa)) {
		dev_err(&fa->pdev->dev,
			"%s Can't apply calibration values\n",
			__func__);
		return -EBUSY;
	}
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFA_CTL_CALIB_APPLY], 1);
	ndelay(100);
	if (fa_calib_is_busy(fa)) {
		dev_err(&fa->pdev->dev,
			"%s Calibration value applied but still 'busy'\n",
			__func__);
		return -EBUSY;
	}

	return 0;
}

static void fa_calib_gain_set(struct fa_dev *fa, unsigned int chan, int val)
{
	int attr_idx;

	attr_idx = zfad_get_chx_index(ZFA_CHx_GAIN, chan);
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[attr_idx], val);
}

static void fa_calib_offset_set(struct fa_dev *fa, unsigned int chan, int val)
{
	int attr_idx;

	attr_idx = zfad_get_chx_index(ZFA_CHx_OFFSET, chan);
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[attr_idx],
		  val & 0xFFFF /* prevent warning */);
}

/*
 * Empirical values for the gain error slope
 *   10V  0.0012500
 *    1V -0.0000233
 * 100mV -0.0000163
 *
 * Multiply by 0x8000 to have the same range as the calibration data.
 * To do integer math we also multiply by 0x2000.
 */
static const int64_t gain_adc_error_slope_fix[] = {
	[FA100M14B4C_RANGE_10V] = 335544,
	[FA100M14B4C_RANGE_1V] = -6255,
	[FA100M14B4C_RANGE_100mV] = -4375,
};

/**
 * Compute the correct gain
 * @range: voltage range
 * @gain_c: calibration value
 * @delta_temp: temperature difference: (current temp. - calibration temp.)
 *              the unit must be centi-degree
 */
static int fa_calib_adc_gain_fix(int range, int32_t gain_c,
				 int32_t delta_temp)
{
	int64_t error;

	error = gain_adc_error_slope_fix[range] * delta_temp;
	error /= 0x2000; /* see comment above for gain_adc_error_slope_fix */
	error /= 100; /* convert to degree */

	return gain_c - error;
}

/*
 * Empirical values for the gain error slope
 *   10V  0.00017100
 *    1V -0.00000349
 * 100mV  0.00001540
 * Multiply by 0x8000 to have the same range as the calibration data.
 * To do integer math we also multiply by 0x2000.
 */
static const int64_t gain_dac_error_slope_fix[] = {
	[FA100M14B4C_RANGE_10V] = 459025,
	[FA100M14B4C_RANGE_1V] = -937,
	[FA100M14B4C_RANGE_100mV] = 4134,
};

/**
 * Compute the correct gain
 * @range: voltage range
 * @gain_c: calibration value
 * @delta_temp: temperature difference: (current temp. - calibration temp.)
 *              the unit must be centi-degree
 */
static int fa_calib_dac_gain_fix(int range, uint32_t gain_c,
				 int32_t delta_temp)
{
	int64_t error;

	error = gain_dac_error_slope_fix[range] * delta_temp;
	error /= 0x2000; /* see comment above for gain_dac_error_slope_fix */
	error /= 100; /* convert to degree */

	return gain_c - error;
}

static bool fa_calib_is_compensation_on(struct fa_dev *fa)
{
	if (unlikely((fa->flags & FA_DEV_F_PATTERN_DATA)))
		return false;

	if (unlikely(fa_calib_temp))
		return true;

	return false;
}

/**
 * Calibrate a ADC channel
 * @fa ADC instance
 * @chan channel number
 * @temperature temperature
 *
 * You must hold &fa->zdev->cset->lock while calling this function
 */
void fa_calib_adc_config_chan(struct fa_dev *fa, unsigned int chan,
			      int32_t temperature, unsigned int flags)
{
	int range = fa->range[chan];
	struct fa_calib_stanza *cal = &fa->calib.adc[range];
	int gain;
	int err;

	if (fa_calib_is_compensation_on(fa)) {
		int32_t delta_temp;

		if (flags & FA_CALIB_FLAG_READ_TEMP) {
			err = fa_temperature_read(fa, &temperature);
			if(err)
				temperature = 45000; /* 45.000 degrees as safe value */
		}
		delta_temp = (temperature / 10) - cal->temperature;
		gain = fa_calib_adc_gain_fix(range, cal->gain[chan],
					     delta_temp);
		dev_dbg(&fa->pdev->dev,
			"%s: {delta-temperature: %d, chan: %d, range: %d, gain: 0x%x, offset: 0x%x}\n",
			__func__, delta_temp, chan, range, gain,
			cal->offset[chan]);

	} else {
		gain = cal->gain[chan];
		dev_dbg(&fa->pdev->dev,
			"%s: {chan: %d, range: %d, gain: 0x%x, offset: 0x%x}\n",
			__func__, chan, range, gain, cal->offset[chan]);
	}

	fa_calib_gain_set(fa, chan, gain);
	fa_calib_offset_set(fa, chan, cal->offset[chan]);
	fa_calib_apply(fa);
}

/**
 * It sets the DAC voltage to apply an offset on the input channel
 * @fa ADC device
 * @chan channel number
 * @val DAC values (-5V: 0x0000, 0V: 0x8000, +5V: 0x7FFF)
 *
 * Return: 0 on success, otherwise a negative error number
 */
static int fa_dac_offset_set(struct fa_dev *fa, unsigned int chan,
			     uint32_t val)
{
	return fa_spi_xfer(fa, FA_SPI_SS_DAC(chan), 16, val, NULL);
}

static uint16_t fa_dac_offset_raw_calibrate(struct fa_dev *fa,
					    uint16_t raw_offset,
					    int gain, int offset)
{
	int32_t signed_offset = raw_offset - 0x8000;
	int64_t hwval;

	hwval = ((signed_offset + offset) * gain) >> 15; /* signed */
	hwval += 0x8000; /* offset binary */
	dev_dbg(&fa->pdev->dev,
		"Final DAC calibrated value: (0x%08x + 0x%08x) * 0x%08x = 0x%08llx\n",
		signed_offset, offset, gain, hwval);

	/* Saturate */
	if (hwval < 0) {
		hwval = 0;
		dev_warn(&fa->pdev->dev,
			 "Final DAC calibrated value: lower saturation, set 0x%04llx",
			 hwval);
	}
	if (hwval > 0xffff) {
		hwval = 0xffff;
		dev_warn(&fa->pdev->dev,
			 "Final DAC calibrated value: lower saturation, set 0x%04llx",
			 hwval);
	}

	return hwval;
}

static int fa_dac_offset_get(struct fa_dev *fa, unsigned int chan, uint16_t *offset)
{
	int32_t user = fa->user_offset[chan];
	int32_t zero = fa->zero_offset[chan];
	int32_t __offset = (user + zero) - 0x8000; /* Bring back to DAC format */

	if (__offset & ~DAC_VAL_MASK) {
		dev_err(&fa->pdev->dev,
			 "DAC offset value overflows 16bits. {user: 0x%04x, zero: 0x%04x, sum: 0x%08x}\n",
			user, zero, __offset);
		return -EINVAL;
	}

	*offset = __offset;

	return 0;
}

/**
 * Calibrate a DAC channel
 * @fa ADC instance
 * @chan channel number
 * @temperature temperature
 *
 * You must hold &fa->zdev->cset->lock while calling this function
 */
int fa_calib_dac_config_chan(struct fa_dev *fa, unsigned int chan,
			     int32_t temperature, unsigned int flags)
{
	uint16_t value;
	int range = fa->range[chan];
	struct fa_calib_stanza *cal = &fa->calib.dac[range];
	int gain;
	int hwval;
	int err;

	err = fa_dac_offset_get(fa, chan, &value);
	if (err)
		return err;

	if (fa_calib_is_compensation_on(fa)) {
		int32_t delta_temp;

		if (flags & FA_CALIB_FLAG_READ_TEMP) {
			err = fa_temperature_read(fa, &temperature);
			if(err)
				temperature = 45000; /* 45.000 degrees as safe value */
		}
		delta_temp = (temperature / 10) - cal->temperature;
		gain = fa_calib_dac_gain_fix(range, cal->gain[chan],
					     delta_temp);
		dev_dbg(&fa->pdev->dev,
			"%s: {delta-temperature: %d, chan: %d, range: %d, gain: 0x%x, offset: 0x%x}\n",
			__func__, delta_temp, chan, range, gain, cal->offset[chan]);
	} else {
		gain = cal->gain[chan];
		dev_dbg(&fa->pdev->dev,
			"%s: {chan: %d, range: %d, gain: 0x%x, offset: 0x%x}\n",
			__func__, chan, range, gain, cal->offset[chan]);
	}

	hwval = fa_dac_offset_raw_calibrate(fa, value, gain,
					    cal->offset[chan]);

	return fa_dac_offset_set(fa, chan, hwval);
}

void fa_calib_config_chan(struct fa_dev *fa, unsigned int chan,
			     int32_t temperature, unsigned int flags)
{
	fa_calib_adc_config_chan(fa, chan, temperature, flags);
	fa_calib_dac_config_chan(fa, chan, temperature, flags);
}

void fa_calib_config(struct fa_dev *fa)
{
	int32_t temperature;
	int i;
	int err;

	err = fa_temperature_read(fa, &temperature);
	if(err)
		temperature = 45000; /* 45.000 degrees as safe value */
	spin_lock(&fa->zdev->cset->lock);
	for (i = 0; i < FA100M14B4C_NCHAN; ++i)
		fa_calib_config_chan(fa, i, temperature, 0);
	spin_unlock(&fa->zdev->cset->lock);
}

static void __fa_calib_gain_update(struct fa_dev *fa)
{
	fa_calib_config(fa);
	mod_timer(&fa->calib_timer, jiffies + HZ * fa_calib_temp_period);
}


/**
 * Periodically update gain calibration values
 * @fa: FMC ADC device
 *
 * In the ADC we have a calibration value that is good for a small
 * temperature range. We proved empirically that the gain error has a
 * linear behavior with respect to the temperature.
 *
 */
#if KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE
static void fa_calib_gain_update(struct timer_list *timer)
{
	struct fa_dev *fa = from_timer(fa, timer, calib_timer);
	__fa_calib_gain_update(fa);
}
#else
static void fa_calib_gain_update(unsigned long arg)
{
	__fa_calib_gain_update((void *)arg);
}
#endif
/* Actual verification code */
static int fa_verify_calib_stanza(struct device *msgdev, char *name, int r,
				    struct fa_calib_stanza *cal)
{
	const struct fa_calib_stanza *iden = &fa_identity_calib;
	int i;

	for (i = 0; i < ARRAY_SIZE(cal->offset); i++) {
		if (abs(cal->offset[i] - iden->offset[i])
		    > FA_CALIB_MAX_DELTA_OFFSET) {
			return -EINVAL;
		}
		if (abs((s16)(cal->gain[i] - iden->gain[i]))
		    > FA_CALIB_MAX_DELTA_GAIN) {
			return -EINVAL;
		}
	}
	if (abs((s16)(cal->temperature - iden->temperature))
	    > FA_CALIB_MAX_DELTA_TEMP) {
		dev_err(msgdev, "invalid temper 0x%x\n", cal->temperature);
		return -EINVAL;
	}

	return 0;
}

static int fa_verify_calib(struct device *msgdev, struct fa_calib *calib)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(calib->adc); i++) {
		int err;

		err = fa_verify_calib_stanza(msgdev, "adc", i, calib->adc + i);
		if (err)
			return err;
		err = fa_verify_calib_stanza(msgdev, "dac", i, calib->dac + i);
		if (err)
			return err;
	}

	return 0;
}

/**
 * @calib: calibration data
 *
 * We know for sure that our structure is only made of 16bit fields
 */
static void fa_calib_le16_to_cpus(struct fa_calib *calib)
{
	int i;
	uint16_t *p = (void *)calib;

	for (i = 0; i < sizeof(*calib) / sizeof(uint16_t); i++)
		le16_to_cpus(p + i); /* s == in situ */
}

/**
 * @calib: calibration data
 *
 * We know for sure that our structure is only made of 16bit fields
 */
static void fa_calib_cpu_to_le16s(struct fa_calib *calib)
{
	int i;
	uint16_t *p = (void *)calib;

	for (i = 0; i < sizeof(*calib) / sizeof(uint16_t); i++)
		cpu_to_le16s(p + i); /* s == in situ */
}

static void fa_identity_calib_set(struct fa_calib *calib)
{
	int i;

	/* Retrieve calibration data from the eeprom, then verify it */
	for (i = 0; i < FA_CALIB_STANZA_N; ++i) {
		memcpy(&calib->adc[i], &fa_identity_calib,
		       sizeof(calib->adc[i]));
		memcpy(&calib->dac[i], &fa_identity_calib,
		       sizeof(calib->dac[i]));
	}
	fa_calib_le16_to_cpus(calib);
}

static void fa_calib_write(struct fa_dev *fa, struct fa_calib *calib)
{
	int err;

	fa_calib_le16_to_cpus(calib);
	err = fa_verify_calib(fa->msgdev, calib);
	if (err) {
		dev_info(fa->msgdev,
			 "Apply Calibration Identity (invalid calibration values)\n");
		fa_identity_calib_set(&fa->calib);
	} else {
		memcpy(&fa->calib, calib, sizeof(*calib));
	}
}

static ssize_t fa_write_eeprom(struct file *file, struct kobject *kobj,
			       struct bin_attribute *attr,
			       char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct fa_dev *fa = get_zfadc(dev);
	struct fa_calib *calib = (struct fa_calib *) buf;

	if (off != 0 || count != sizeof(*calib))
		return -EINVAL;

	fa_calib_write(fa, calib);
	fa_calib_config(fa);

	return count;
}

static ssize_t fa_read_eeprom(struct file *file, struct kobject *kobj,
			      struct bin_attribute *attr,
			      char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct fa_dev *fa = get_zfadc(dev);
	struct fa_calib *calib = (struct fa_calib *) buf;


	if (off != 0 || count < sizeof(fa->calib))
		return -EINVAL;

	memcpy(calib, &fa->calib, sizeof(fa->calib));
	fa_calib_cpu_to_le16s(calib);

	return count;
}

struct bin_attribute dev_attr_calibration = {
	.attr = {
		.name = "calibration_data",
		.mode = 0644,
	},
	.size = sizeof(struct fa_calib),
	.write = fa_write_eeprom,
	.read = fa_read_eeprom,
};


int fa_calib_init(struct fa_dev *fa)
{
	struct fa_calib calib;
	int ret;

	ret = fmc_slot_eeprom_read(fa->slot, &calib,
				   FA_CAL_OFFSET, sizeof(calib));
	if (ret < 0) {
		dev_warn(fa->msgdev,
			 "Failed to read calibration from EEPROM: using identity calibration %d\n",
			 ret);
		fa_identity_calib_set(&calib);
		goto out;
	}

	fa_calib_write(fa, &calib);

	/* First calibration.
	 * The board has just been reset by the carrier before calling this
	 * driver and reading the temperature read needs at least 350ms
	 */
	msleep(400);
	fa_calib_config(fa);

	/* Prepare the timely recalibration */
	if (fa_calib_is_compensation_on(fa) && fa_calib_temp_period) {
#if KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE
		timer_setup(&fa->calib_timer, fa_calib_gain_update, 0);
#else
		setup_timer(&fa->calib_timer, fa_calib_gain_update, (unsigned long)fa);
#endif
		mod_timer(&fa->calib_timer,
			  jiffies + HZ * fa_calib_temp_period);
	}

out:
	return 0;
}

void fa_calib_exit(struct fa_dev *fa)
{
	if (fa_calib_temp_period)
		del_timer_sync(&fa->calib_timer);
	fa_identity_calib_set(&fa->calib);
	fa_calib_config(fa);
}
