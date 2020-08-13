// SPDX-License-Identifier: GPL-2.0-or-later

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
#include <fmc-adc-100m14b4cha.h>

static int fa_calib_period_s = 60;
module_param_named(calib_s, fa_calib_period_s, int, 0444);

/* This identity calibration is used as default */
static const struct fa_calib_stanza fa_identity_calib = {
	.offset = { 0, },
	.gain = {0x8000, 0x8000, 0x8000, 0x8000},
	.temperature = 50 * 1000, /* 50 celsius degrees */
};

/* Max difference from identity thing */
#define FA_CALIB_MAX_DELTA_OFFSET	0x1000
#define FA_CALIB_MAX_DELTA_GAIN		0x1000
#define FA_CALIB_MAX_DELTA_TEMP		(40 * 1000) /* 10-90 celsius */

static bool fa_calib_is_busy(struct fa_dev *fa)
{
	return !!fa_readl(fa, fa->fa_adc_csr_base, &zfad_regs[ZFA_STA_CALIB_BUSY]);
}

static int fa_calib_apply(struct fa_dev *fa)
{
	if (fa_calib_is_busy(fa))
		return -EBUSY;
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFA_CTL_CALIB_APPLY], 1);
        ndelay(100);
	if (fa_calib_is_busy(fa))
		return -EBUSY;

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
 *              the unit must be milli-degree
 */
static int fa_calib_adc_gain_fix(int range, int32_t gain_c,
				 int32_t delta_temp)
{
	int64_t error;

	error = gain_adc_error_slope_fix[range] * delta_temp;
	error /= 0x2000; /* see comment above for gain_adc_error_slope_fix */
	error /= 1000; /* convert to degree */

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
 *              the unit must be milli-degree
 */
static int fa_calib_dac_gain_fix(int range, uint32_t gain_c,
				 int32_t delta_temp)
{
        int64_t error;

	error = gain_dac_error_slope_fix[range] * delta_temp;
	error /= 0x2000; /* see comment above for gain_dac_error_slope_fix */
	error /= 1000; /* convert to degree */

	return gain_c - error;
}

static void fa_calib_adc_config_chan(struct fa_dev *fa, unsigned int chan,
				     int32_t temperature)
{
	int range = fa->range[chan];
	struct fa_calib_stanza *cal = &fa->calib.adc[range];
	int32_t delta_temp = temperature - cal->temperature;
	int offset = cal->offset[chan];
	int gain = cal->gain[chan];

	if (unlikely(!(fa->flags & FA_DEV_F_PATTERN_DATA)))
		gain = fa_calib_adc_gain_fix(range, gain, delta_temp);

	dev_dbg(&fa->pdev->dev, "%s: {chan: %d, range: %d, gain: 0x%x, offset: 0x%x}\n",
		__func__, chan, range, gain, offset);

	fa_calib_gain_set(fa, chan, gain);
	fa_calib_offset_set(fa, chan, offset);
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

static int64_t fa_dac_offset_raw_get(int32_t offset)
{
	int64_t hwval;

	hwval = offset * 0x8000LL / 5000000;
	if (hwval == 0x8000)
		hwval = 0x7fff; /* -32768 .. 32767 */
	return hwval;
}

static int64_t fa_dac_offset_raw_calibrate(int32_t raw_offset,
					   int gain, int offset)
{
	int64_t hwval;

	hwval = ((raw_offset + offset) * gain) >> 15; /* signed */
        hwval += 0x8000; /* offset binary */
	if (hwval < 0)
		hwval = 0;
	if (hwval > 0xffff)
		hwval = 0xffff;

	return hwval;
}

static int fa_dac_offset_get(struct fa_dev *fa, unsigned int chan)
{
	int32_t off_uv = fa->user_offset[chan] + fa->zero_offset[chan];

	if (WARN(off_uv < DAC_SAT_LOW,
		 "DAC lower saturation %d < %d\n",
		 off_uv, DAC_SAT_LOW)) {
		off_uv = DAC_SAT_LOW;
	}
	if (WARN(off_uv > DAC_SAT_UP,
		 "DAC upper saturation %d > %d\n",
		 off_uv, DAC_SAT_UP)) {
		off_uv = DAC_SAT_UP;
	}

        return off_uv;
}

static int fa_calib_dac_config_chan(struct fa_dev *fa, unsigned int chan,
				    int32_t temperature)
{
	int range = fa->range[chan];
	int32_t off_uv = fa_dac_offset_get(fa, chan);
	int32_t off_uv_raw = fa_dac_offset_raw_get(off_uv);
	struct fa_calib_stanza *cal = &fa->calib.dac[range];
	int32_t delta_temp = temperature - cal->temperature;
	int offset = cal->offset[chan];
	int gain = cal->gain[chan];
	int hwval;

	if (unlikely(!(fa->flags & FA_DEV_F_PATTERN_DATA)))
		gain = fa_calib_dac_gain_fix(range, gain, delta_temp);

	dev_dbg(&fa->pdev->dev, "%s: {chan: %d, range: %d, gain: 0x%x, offset: 0x%x}\n",
		__func__, chan, range, gain, offset);
	hwval = fa_dac_offset_raw_calibrate(off_uv_raw, gain, offset);

        return  fa_dac_offset_set(fa, chan, hwval);
}

void fa_calib_dac_config(struct fa_dev *fa, int32_t temperature)
{
	int i;

	if (temperature == 0xFFFFFFFF)
	    temperature = fa_temperature_read(fa);
	dev_dbg(&fa->pdev->dev, "%s: {temperature: %d}\n",
		__func__, temperature);

	spin_lock(&fa->zdev->cset->lock);
	for (i = 0; i < FA100M14B4C_NCHAN; ++i)
		fa_calib_dac_config_chan(fa, i, temperature);
	spin_unlock(&fa->zdev->cset->lock);
}

static void fa_calib_adc_config(struct fa_dev *fa, int32_t temperature)
{
	int err;
	int i;

	if (temperature == 0xFFFFFFFF)
	    temperature = fa_temperature_read(fa);
	dev_dbg(&fa->pdev->dev, "%s: {temperature: %d}\n",
		__func__, temperature);

	spin_lock(&fa->zdev->cset->lock);
	for (i = 0; i < FA100M14B4C_NCHAN; ++i)
		fa_calib_adc_config_chan(fa, i, temperature);
	spin_unlock(&fa->zdev->cset->lock);

        err = fa_calib_apply(fa);
	if (err)
		dev_err(&fa->pdev->dev, "Can't apply calibration values\n");
}

void fa_calib_config(struct fa_dev *fa)
{
	int32_t temperature;

	temperature = fa_temperature_read(fa);
	fa_calib_adc_config(fa, temperature);
	fa_calib_dac_config(fa, temperature);
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
static void fa_calib_gain_update(unsigned long arg)
{
	struct fa_dev *fa = (void *)arg;

	fa_calib_config(fa);
	mod_timer(&fa->calib_timer, jiffies + HZ * fa_calib_period_s);
}

/* Actual verification code */
static int fa_verify_calib_stanza(struct device *msgdev, char *name, int r,
				    struct fa_calib_stanza *cal)
{
	const struct fa_calib_stanza *iden = &fa_identity_calib;
	int i;

	for (i = 0; i < ARRAY_SIZE(cal->offset); i++) {
		if (abs(cal->offset[i] - iden->offset[i])
		    > FA_CALIB_MAX_DELTA_OFFSET) {
			dev_err(msgdev, "wrong offset (%i) 0x%x\n",
				i, cal->offset[i]);
			return -EINVAL;
		}
		if (abs((s16)(cal->gain[i] - iden->gain[i]))
		    > FA_CALIB_MAX_DELTA_GAIN) {
			dev_err(msgdev, "invalid gain   (%i) 0x%x\n",
				i, cal->gain[i]);
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
	int i, err = 0;

	for (i = 0; i < ARRAY_SIZE(calib->adc); i++) {
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

/**
 * Calculate calibrated values for offset and range using current values
 * @fa: FMC ADC device
 */
static void fa_apply_calib(struct fa_dev *fa)
{
	int i;

	for (i = 0; i < FA100M14B4C_NCHAN; ++i) {
		struct zio_channel *chan = &fa->zdev->cset->chan[i];
		int reg = zfad_get_chx_index(ZFA_CHx_CTL_RANGE, chan->index);
		int range = fa_readl(fa, fa->fa_adc_csr_base, &zfad_regs[reg]);

		fa_adc_range_set(fa, chan, zfad_convert_hw_range(range));
		fa_calib_config(fa);
	}
}


static void fa_calib_write(struct fa_dev *fa, struct fa_calib *calib)
{
	int err;

	fa_calib_le16_to_cpus(calib);
	err = fa_verify_calib(fa->msgdev, calib);
	if (err) {
		dev_info(fa->msgdev, "Apply Calibration Identity\n");
		fa_identity_calib_set(&fa->calib);
	} else {
		memcpy(&fa->calib, calib, sizeof(*calib));
	}
	fa_apply_calib(fa);
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

	/* Prepare the timely recalibration */
	fa_calib_config(fa);
	setup_timer(&fa->calib_timer, fa_calib_gain_update, (unsigned long)fa);
	if (fa_calib_period_s)
		mod_timer(&fa->calib_timer, jiffies + HZ * fa_calib_period_s);

out:
	return 0;
}

void fa_calib_exit(struct fa_dev *fa)
{
	del_timer_sync(&fa->calib_timer);
	fa_identity_calib_set(&fa->calib);
}
