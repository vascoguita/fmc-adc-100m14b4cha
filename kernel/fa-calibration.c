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

static int fa_calib_adc_offset_fix(struct fa_dev *fa, int range, int offset,
				   uint32_t temperature)
{
        return offset;
}

static int fa_calib_adc_gain_fix(struct fa_dev *fa, int range, int gain,
				 uint32_t temperature)
{
        return gain;
}

static void fa_calib_adc_config_chan(struct fa_dev *fa, unsigned int chan,
				     uint32_t temperature)
{
	int range = fa->range[chan];
	int offset = fa->calib.adc[range].offset[chan];
	int gain = fa->calib.adc[range].gain[chan];

	offset = fa_calib_adc_offset_fix(fa, range, offset, temperature);
	gain = fa_calib_adc_gain_fix(fa, range, gain, temperature);

	dev_dbg(&fa->pdev->dev, "%s: {chan: %d, range: %d, gain: 0x%x, offset: 0x%x}\n",
		__func__, chan, range, gain, offset);

	fa_calib_gain_set(fa, chan, gain);
	fa_calib_offset_set(fa, chan, offset);
}

int fa_calib_adc_config(struct fa_dev *fa)
{
	int i;
	uint32_t temperature;

	temperature = fa_temperature_read(fa);
	dev_dbg(&fa->pdev->dev, "%s: {temperature: %d}\n", __func__, temperature);
	for (i = 0; i < FA100M14B4C_NCHAN; ++i)
		fa_calib_adc_config_chan(fa, i, temperature);

        return fa_calib_apply(fa);
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

	fa_calib_adc_config(fa);
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

		zfad_set_range(fa, chan, zfad_convert_hw_range(range));
		zfad_apply_offset(chan);
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
	fa_calib_adc_config(fa);
	setup_timer(&fa->calib_timer, fa_calib_gain_update, (unsigned long)fa);
	if (fa_calib_period_s)
		mod_timer(&fa->calib_timer, jiffies + HZ * fa_calib_period_s);

out:
	return 0;
}

void fa_calib_exit(struct fa_dev *fa)
{
	del_timer_sync(&fa->calib_timer);
}
