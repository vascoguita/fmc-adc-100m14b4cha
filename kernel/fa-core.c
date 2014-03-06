/*
 * core fmc-adc-100m14b driver
 *
 * Copyright (C) 2012 CERN (www.cern.ch)
 * Author: Federico Vaga <federico.vaga@gmail.com>
 *		Copied from fine-delay fd-core.c
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/fmc.h>

#include "fmc-adc.h"

static struct fmc_driver fa_dev_drv;
FMC_PARAM_BUSID(fa_dev_drv);
FMC_PARAM_GATEWARE(fa_dev_drv);

static const int zfad_hw_range[] = {
	[ZFA_RANGE_10V]   = 0x45,
	[ZFA_RANGE_1V]    = 0x11,
	[ZFA_RANGE_100mV] = 0x23,
	[ZFA_RANGE_OPEN]  = 0x00,
};

/*
 * zfad_convert_hw_range
 * @usr_val: range value
 *
 * return the enum associated to the range value
 */
int zfad_convert_hw_range(uint32_t bitmask)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(zfad_hw_range); i++)
		if (zfad_hw_range[i] == bitmask)
			return i;
	return -EINVAL;
}

/* Calculate correct index in fa_regfield array for channel from CHx indexes */
int zfad_get_chx_index(unsigned long addr, struct zio_channel *chan)
{
	int offset;

	offset = ZFA_CHx_MULT  * (FA_NCHAN - chan->index);

	return addr - offset;
}

/*
 * zfad_apply_user_offset
 * @fa: the fmc-adc descriptor
 * @chan: the channel where apply offset
 * @usr_val: the offset value to apply, expressed as millivolts (-5000..5000)
 *
 * Apply user offset to the channel input. Before apply the user offset it must
 * be corrected with offset and gain calibration value. An open input does not
 * need any correction.
 */
int zfad_apply_user_offset(struct fa_dev *fa, struct zio_channel *chan,
				  uint32_t usr_val)
{
	uint32_t range_reg;
	int32_t uval =  (int32_t)usr_val;
	int offset, gain, hwval, i, range;

	if (uval < -5000 || uval > 5000)
		return -EINVAL;

	i = zfad_get_chx_index(ZFA_CHx_CTL_RANGE, chan);
	zfa_hardware_read(fa, i, &range_reg);

	range = zfad_convert_hw_range(range_reg);
	if (range < 0)
		return range;

	if (range == ZFA_RANGE_OPEN) {
		offset = FA_CAL_NO_OFFSET;
		gain = FA_CAL_NO_GAIN;
	} else {
		offset = fa->calib.dac[range].offset[chan->index];
		gain = fa->calib.dac[range].gain[chan->index];
	}

	hwval = uval * 0x8000 / 5000;
	if (hwval == 0x8000)
		hwval = 0x7fff; /* -32768 .. 32767 */

	hwval = ((hwval + offset) * gain) >> 15; /* signed */
	hwval += 0x8000; /* offset binary */
	if (hwval < 0)
		hwval = 0;
	if (hwval > 0xffff)
		hwval = 0xffff;

	/* Apply calibrated offset to DAC */
	return fa_spi_xfer(fa, FA_SPI_SS_DAC(chan->index), 16, hwval, NULL);
}

/*
 * zfad_reset_offset
 * @fa: the fmc-adc descriptor
 *
 * Reset channel's offsets
 */
void zfad_reset_offset(struct fa_dev *fa)
{
	int i;

	for (i = 0; i < FA_NCHAN; ++i)
		zfad_apply_user_offset(fa, &fa->zdev->cset->chan[i], 0);
}

/*
 * zfad_set_range
 * @fa: the fmc-adc descriptor
 * @chan: the channel to calibrate
 * @usr_val: the volt range to set and calibrate
 *
 * When the input range changes, we must write new fixup values
 */
int zfad_set_range(struct fa_dev *fa, struct zio_channel *chan,
			  int range)
{
	int i, offset, gain;

	/* Actually set the range */
	i = zfad_get_chx_index(ZFA_CHx_CTL_RANGE, chan);
	zfa_hardware_write(fa, i, zfad_hw_range[range]);

	if (range == ZFA_RANGE_OPEN) {
		offset = FA_CAL_NO_OFFSET;
		gain = FA_CAL_NO_GAIN;
	} else {
		if (range < 0 || range > ARRAY_SIZE(fa->calib.adc)) {
			dev_info(&fa->fmc->dev, "Invalid range %i or ch %i\n",
				 range, chan->index);
			return -EINVAL;
		}
		offset = fa->calib.adc[range].offset[chan->index];
		gain = fa->calib.adc[range].gain[chan->index];
	}

	i = zfad_get_chx_index(ZFA_CHx_OFFSET, chan);
	zfa_hardware_write(fa, i, offset & 0xffff); /* prevent warning */
	i = zfad_get_chx_index(ZFA_CHx_GAIN, chan);
	zfa_hardware_write(fa, i, gain);

	/* recalculate user offset for the new range */
	zfad_apply_user_offset(fa, chan, fa->user_offset[chan->index]);

	return 0;
}


/*
 * zfad_fsm_command
 * @fa: the fmc-adc descriptor
 * @command: the command to apply to FSM
 *
 * This function checks if the command can be done and performs some
 * preliminary operation beforehand
 */
int zfad_fsm_command(struct fa_dev *fa, uint32_t command)
{
	struct device *dev = &fa->fmc->dev;
	struct zio_cset *cset = fa->zdev->cset;
	uint32_t val;

	if (command != ZFA_START && command != ZFA_STOP) {
		dev_info(dev, "Invalid command %i\n", command);
		return -EINVAL;
	}

	/*
	 * When any command occurs we are ready to start a new acquisition, so
	 * we must abort any previous one. If it is STOP, we abort because we
	 * abort an acquisition. If it is START, we abort because if there was
	 * a previous start but the acquisition end interrupt doesn't occurs,
	 * START mean RESTART. If it is a clean START, the abort has not
	 * effects.
	 *
	 * This is done only if ADC is using its own trigger, otherwise it is
	 * not necessary.
	 *
	 * The case of fmc-adc-trg is optimized because is the most common
	 * case
	 */
	if (likely(cset->trig == &zfat_type || command == ZFA_STOP))
		zio_trigger_abort_disable(cset, 0);

	/* Reset counters */
	fa->n_shots = 0;
	fa->n_fires = 0;

	/* If START, check if we can start */
	if (command == ZFA_START) {
		/* Verify that SerDes PLL is lockes */
		zfa_hardware_read(fa, ZFA_STA_SERDES_PLL, &val);
		if (!val) {
			dev_info(dev, "Cannot start acquisition: "
				 "SerDes PLL not locked\n");
			return -EBUSY;
		}
		/* Verify that SerDes is synched */
		zfa_hardware_read(fa, ZFA_STA_SERDES_SYNCED, &val);
		if (!val) {
			dev_info(dev, "Cannot start acquisition: "
				 "SerDes not synchronized\n");
			return -EBUSY;
		}

		/* Now we can arm the trigger for the incoming acquisition */
		zio_arm_trigger(cset->ti);
		/*
		 *  FIXME maybe zio_arm_trigger() can return an error when it
		 * is not able to arm a trigger.
		 *
		 * It returns -EPERM, but the error can be -ENOMEM or -EINVAL
		 * from zfat_arm_trigger() or zfad_input_cset()
		 */
		if (!(cset->ti->flags & ZIO_TI_ARMED)) {
			dev_info(dev, "Cannot start acquisition: "
				 "Trigger refuses to arm\n");
			return -EIO;
		}

		dev_dbg(dev, "FSM START Command, Enable interrupts\n");
		zfa_hardware_write(fa, ZFA_IRQ_MASK, ZFAT_ALL);
	} else {
		dev_dbg(dev, "FSM STOP Command, Disable interrupts\n");
		zfa_hardware_write(fa, ZFA_IRQ_MASK, ZFAT_NONE);
	}

	zfa_hardware_write(fa, ZFA_CTL_FMS_CMD, command);
	return 0;
}

/* This structure lists the various subsystems */
struct fa_modlist {
	char *name;
	int (*init)(struct fa_dev *);
	void (*exit)(struct fa_dev *);
};

static struct fa_modlist mods[] = {
	{"spi", fa_spi_init, fa_spi_exit},
	{"onewire", fa_onewire_init, fa_onewire_exit},
	{"zio", fa_zio_init, fa_zio_exit},
};

/* probe and remove are called by fa-spec.c */
int fa_probe(struct fmc_device *fmc)
{
	struct fa_modlist *m = NULL;
	struct fa_dev *fa;
	int err, i = 0;
	char *fwname;

	/* Validate the new FMC device */
	i = fmc->op->validate(fmc, &fa_dev_drv);
	if (i < 0) {
		dev_info(&fmc->dev, "not using \"%s\" according to "
			 "modparam\n", KBUILD_MODNAME);
		return -ENODEV;
	}

	/* Driver data */
	fa = devm_kzalloc(&fmc->dev, sizeof(struct fa_dev), GFP_KERNEL);
	if (!fa)
		return -ENOMEM;
	fmc_set_drvdata(fmc, fa);
	fa->fmc = fmc;

	if (fa_dev_drv.gw_n)
		fwname = "";	/* reprogram will pick from module parameter */
	else
		fwname = FA_GATEWARE_DEFAULT_NAME;
	dev_info(fmc->hwdev, "Gateware (%s)\n", fwname);
	/* We first write a new binary (and lm32) within the carrier */
	err = fmc->op->reprogram(fmc, &fa_dev_drv, fwname);
	if (err) {
		dev_err(fmc->hwdev, "write firmware \"%s\": error %i\n",
				fwname, err);
		goto out;
	}
	dev_info(fmc->hwdev, "Gateware successfully loaded\n");

	/* init all subsystems */
	for (i = 0, m = mods; i < ARRAY_SIZE(mods); i++, m++) {
		dev_dbg(&fmc->dev, "Calling init for \"%s\"\n", m->name);
		err = m->init(fa);
		if (err) {
			dev_err(&fmc->dev, "error initializing %s\n", m->name);
			goto out;
		}

	}

	return 0;
out:
	while (--m, --i >= 0)
		if (m->exit)
			m->exit(fa);
	return err;
}
int fa_remove(struct fmc_device *fmc)
{
	struct fa_dev *fa = fmc_get_drvdata(fmc);
	struct fa_modlist *m;
	int i = ARRAY_SIZE(mods);

	while (--i >= 0) {
		m = mods + i;
		if (m->exit)
			m->exit(fa);
	}

	return 0;
}

static struct fmc_fru_id fa_fru_id[] = {
	{
		.product_name = "FmcAdc100m14b4cha",
	},
};

static struct fmc_driver fa_dev_drv = {
	.version = FMC_VERSION,
	.driver.name = KBUILD_MODNAME,
	.probe = fa_probe,
	.remove = fa_remove,
	.id_table = {
		.fru_id = fa_fru_id,
		.fru_id_nr = ARRAY_SIZE(fa_fru_id),
	},
};

static int fa_init(void)
{
	int ret;

	/* First trigger and zio driver */
	ret = fa_trig_init();
	if (ret)
		return ret;

	ret = fa_zio_register();
	if (ret) {
		fa_trig_exit();
		return ret;
	}
	/* Finally the fmc driver, whose probe instantiates zio devices */
	ret = fmc_driver_register(&fa_dev_drv);
	if (ret) {
		fa_trig_exit();
		fa_zio_unregister();
		return ret;
	}
	return 0;
}

static void fa_exit(void)
{
	fmc_driver_unregister(&fa_dev_drv);
	fa_zio_unregister();
	fa_trig_exit();
}

module_init(fa_init);
module_exit(fa_exit);

MODULE_AUTHOR("Federico Vaga");
MODULE_DESCRIPTION("FMC-ADC-100MS-14b Linux Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(GIT_VERSION);
