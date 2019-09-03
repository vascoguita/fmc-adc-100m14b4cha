// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * EEPROM calibration block retreival code for fa-dev
 */

#include "fmc-adc-100m14b4cha.h"

/* This identity calibration is used as default */
static struct fa_calib_stanza fa_identity_calib = {
	.offset = { 0, },
	.gain = {0x8000, 0x8000, 0x8000, 0x8000},
	.temperature = 50 * 100, /* 50 celsius degrees */
};
/* Max difference from identity thing */
#define FA_CALIB_MAX_DELTA_OFFSET	0x1000
#define FA_CALIB_MAX_DELTA_GAIN		0x1000
#define FA_CALIB_MAX_DELTA_TEMP		(40 * 100) /* 10-90 celsius */

/* Actual verification code */
static int fa_verify_calib_stanza(struct device *msgdev, char *name, int r,
				    struct fa_calib_stanza *cal,
				    struct fa_calib_stanza *iden)
{
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

static void fa_verify_calib(struct device *msgdev,
			      struct fa_calib *calib,
			      struct fa_calib_stanza *identity)
{
	int i, err = 0;

	for (i = 0; i < ARRAY_SIZE(calib->adc); i++) {
		err = fa_verify_calib_stanza(msgdev, "adc", i,
					     calib->adc + i, identity);
		if (err)
			break;
		err = fa_verify_calib_stanza(msgdev, "dac", i,
					     calib->dac + i, identity);
		if (err)
			break;
	}
	if (err) {
		dev_warn(msgdev,
			 "Invalid calibration in EEPROM (err: %i, stanza: %i)\n",
			 err, i);
		dev_info(msgdev, "Using identity calibration\n");
		for (i = 0; i < ARRAY_SIZE(calib->adc); i++) {
			calib->adc[i] = *identity;
			calib->dac[i] = *identity;
		}
	}
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

/**
 * Calculate calibrated values for offset and range using current values
 * @fa: FMC ADC device
 */
static void fa_apply_calib(struct fa_dev *fa)
{
	int i;

	for (i = 0; i < FA100M14B4C_NCHAN; ++i) {
		struct zio_channel *chan = &fa->zdev->cset->chan[i];
		int reg = zfad_get_chx_index(ZFA_CHx_CTL_RANGE, chan);
		int range = fa_readl(fa, fa->fa_adc_csr_base, &zfad_regs[reg]);

		zfad_set_range(fa, chan, range);
		zfad_apply_offset(chan);
	}
}


static void fa_calib_write(struct fa_dev *fa, struct fa_calib *calib)
{
	fa_calib_le16_to_cpus(calib);
	fa_verify_calib(fa->msgdev, calib, &fa_identity_calib);

	/*
	 * The user should be careful enough to not change calibration
	 * values while running an acquisition
	 */
	memcpy(&fa->calib, calib, sizeof(*calib));
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
		memcpy(&calib, &fa_identity_calib, sizeof(fa->calib));
	}

	fa_calib_write(fa, &calib);


	return 0;
}

void fa_calib_exit(struct fa_dev *fa)
{

}
