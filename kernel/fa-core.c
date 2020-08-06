// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2012-2019 CERN (www.cern.ch)
 * Author: Federico Vaga <federico.vaga@gmail.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/dmaengine.h>
#include <linux/mod_devicetable.h>
#include <linux/ipmi-fru.h>
#include <linux/fmc.h>

#include "fmc-adc-100m14b4cha.h"
#include <platform_data/fmc-adc-100m14b4cha.h>

static int fa_enable_test_data_fpga;
module_param_named(enable_test_data_fpga, fa_enable_test_data_fpga, int, 0444);
int fa_enable_test_data_adc = 0;
module_param_named(enable_test_data_adc, fa_enable_test_data_adc, int, 0444);

#define FA_EEPROM_TYPE "at24c64"


static const int zfad_hw_range[] = {
	[FA100M14B4C_RANGE_10V_CAL]   = 0x44,
	[FA100M14B4C_RANGE_1V_CAL]    = 0x40,
	[FA100M14B4C_RANGE_100mV_CAL] = 0x42,
	[FA100M14B4C_RANGE_10V]       = 0x45,
	[FA100M14B4C_RANGE_1V]        = 0x11,
	[FA100M14B4C_RANGE_100mV]     = 0x23,
	[FA100M14B4C_RANGE_OPEN]      = 0x00,
};

/* fmc-adc specific workqueue */
struct workqueue_struct *fa_workqueue;


/**
 * Read FMC mezzanine temperature
 * @fa: the adc descriptor
 *
 * DS18B20 returns units of 1/16 degree. We return units
 * of 1/1000 of a degree instead.
 */
uint32_t fa_temperature_read(struct fa_dev *fa)
{
	uint32_t raw_temp;

	raw_temp = fa_readl(fa, fa->fa_ow_base, &zfad_regs[ZFA_DS18B20_TEMP]);

	return (raw_temp * 1000 + 8) / 16;
}

/**
 * Description:
 *    The version from the Linux kernel automatically squash contiguous pages.
 *    Sometimes we do not want to squash (e.g. SVEC)
 */
static int sg_alloc_table_from_pages_no_squash(struct sg_table *sgt,
					       struct page **pages,
					       unsigned int n_pages,
					       unsigned int offset,
					       unsigned long size,
					       unsigned int max_segment,
					       gfp_t gfp_mask)
{
	struct scatterlist *sg;
	int err, i;

	err = sg_alloc_table(sgt, n_pages, GFP_KERNEL);
	if (unlikely(err))
		return err;

	for_each_sg(sgt->sgl, sg, sgt->orig_nents, i) {
		unsigned long chunk_size;

		chunk_size = PAGE_SIZE - offset;
		sg_set_page(sg, pages[i], min(size, chunk_size), offset);
		offset = 0;
		size -= chunk_size;
	}

	return 0;
}


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
int zfad_get_chx_index(unsigned long addr, unsigned int chan)
{
	int offset;

	offset = ZFA_CHx_MULT  * (FA100M14B4C_NCHAN - chan);

	return addr - offset;
}


/**
 * It enables or disables the pattern data on the ADC
 * @fa The ADC device instance
 * @pattern the pattern data to get from the ADC
 * @enable 0 to disable, 1 to enable
 */
int zfad_pattern_data_enable(struct fa_dev *fa, uint16_t pattern,
			     unsigned int enable)
{
	uint32_t frame_tx;
	int err;

	frame_tx  = 0x0000; /* write mode */
	frame_tx |= 0x0400; /* A4 pattern */
	frame_tx |= pattern & 0xFF; /* LSB pattern */
	err = fa_spi_xfer(fa, FA_SPI_SS_ADC, 16, frame_tx, NULL);
	if (err)
		return err;

	frame_tx  = 0x0000; /* write mode */
	frame_tx |= 0x0300; /* A3 pattern + enable */
	frame_tx |= (pattern & 0xFF00) >> 8; /* MSB pattern */
	frame_tx |= (enable ? 0x80 : 0x00); /* Enable the pattern data */
	err = fa_spi_xfer(fa, FA_SPI_SS_ADC, 16, frame_tx, NULL);
	if (err)
		return err;

	return 0;
}

/**
 * It sets the DAC voltage to apply an offset on the input channel
 * @chan
 * @val DAC values (-5V: 0x0000, 0V: 0x8000, +5V: 0x7FFF)
 *
 * Return: 0 on success, otherwise a negative error number
 */
static int zfad_dac_set(struct zio_channel *chan, uint32_t val)
{
	struct fa_dev *fa = get_zfadc(&chan->cset->zdev->head.dev);

	return fa_spi_xfer(fa, FA_SPI_SS_DAC(chan->index), 16, val, NULL);
}

static int zfad_offset_to_dac(struct zio_channel *chan,
			      int32_t uval,
			      enum fa100m14b4c_input_range range)
{
	struct fa_dev *fa = get_zfadc(&chan->cset->zdev->head.dev);
	int offset, gain;
	int64_t hwval;

	hwval = uval * 0x8000LL / 5000000;
	if (hwval == 0x8000)
		hwval = 0x7fff; /* -32768 .. 32767 */

	offset = fa->calib.dac[range].offset[chan->index];
	gain = fa->calib.dac[range].gain[chan->index];

	hwval = ((hwval + offset) * gain) >> 15; /* signed */
	hwval += 0x8000; /* offset binary */
	if (hwval < 0)
		hwval = 0;
	if (hwval > 0xffff)
		hwval = 0xffff;

	return hwval;
}

#define DAC_SAT_LOW -5000000
#define DAC_SAT_UP 5000000
static int fa_dac_offset_get(struct fa_dev *fa, unsigned int chan)
{
	int32_t off_uv = fa->user_offset[chan] + fa->zero_offset[chan];

	if (off_uv < DAC_SAT_LOW) {
		dev_warn(&fa->pdev->dev, "DAC lower saturation %d\n",
			 DAC_SAT_LOW);
		off_uv = DAC_SAT_LOW;
	}
	if (off_uv > DAC_SAT_UP) {
		dev_warn(&fa->pdev->dev, "DAC upper saturation %d\n",
			 DAC_SAT_UP);
		off_uv = DAC_SAT_UP;
	}

        return off_uv;
}
/*
 * zfad_apply_user_offset
 * @chan: the channel where apply offset
 *
 * Apply user offset to the channel input. Before apply the user offset it must
 * be corrected with offset and gain calibration value.
 *
 * Offset values are taken from `struct fa_dev`, so they must be there before
 * calling this function
 */
int zfad_apply_offset(struct zio_channel *chan)
{
	struct fa_dev *fa = get_zfadc(&chan->cset->zdev->head.dev);
	int32_t off_uv = fa_dac_offset_get(fa, chan->index);
	int hwval;
	int range;

	spin_lock(&fa->zdev->cset->lock);
	range = fa->range[chan->index];
	spin_unlock(&fa->zdev->cset->lock);

	hwval = zfad_offset_to_dac(chan, off_uv, range);
	return zfad_dac_set(chan, hwval);
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

	for (i = 0; i < FA100M14B4C_NCHAN; ++i) {
		fa->user_offset[i] = 0;
		fa->zero_offset[i] = 0;
		zfad_apply_offset(&fa->zdev->cset->chan[i]);
	}
}

/*
 * zfad_init_saturation
 * @fa: the fmc-adc descriptor
 *
 * Initialize all saturation registers to the maximum value
 */
void zfad_init_saturation(struct fa_dev *fa)
{
	int idx, i;

	for (i = 0, idx = ZFA_CH1_SAT; i < FA100M14B4C_NCHAN; ++i, idx += ZFA_CHx_MULT)
		fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[idx], 0x7fff);
}

/*
 * zfad_set_range
 * @fa: the fmc-adc descriptor
 * @chan: the channel to calibrate
 * @usr_val: the volt range to set and calibrate
 *
 * When the input range changes, we must write new fixup values.
 * Gain ad offsets must be corrected with offset and gain calibration value.
 * An open input and test data do not need any correction.
 */
int zfad_set_range(struct fa_dev *fa, struct zio_channel *chan,
			  int range)
{
	int i;

	/* Actually set the range */
	i = zfad_get_chx_index(ZFA_CHx_CTL_RANGE, chan->index);
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[i], zfad_hw_range[range]);

	if (range == FA100M14B4C_RANGE_OPEN || fa_enable_test_data_adc)
		range = FA100M14B4C_RANGE_1V;
	else if (range >= FA100M14B4C_RANGE_10V_CAL)
		range -= FA100M14B4C_RANGE_10V_CAL;

	if (range < 0 || range > ARRAY_SIZE(fa->calib.adc)) {
		dev_info(fa->msgdev, "Invalid range %i or ch %i\n",
			 range, chan->index);
		return -EINVAL;
	}

	spin_lock(&fa->zdev->cset->lock);
	fa->range[chan->index] = range;
	spin_unlock(&fa->zdev->cset->lock);
	/* recalculate user offset for the new range */
	zfad_apply_offset(chan);
	return fa_calib_adc_config(fa);
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
	struct zio_cset *cset = fa->zdev->cset;
	uint32_t val;

	if (command != FA100M14B4C_CMD_START &&
	    command != FA100M14B4C_CMD_STOP) {
		dev_info(fa->msgdev, "Invalid command %i\n", command);
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
	if (likely(cset->trig == &zfat_type || command == FA100M14B4C_CMD_STOP))
		zio_trigger_abort_disable(cset, 0);

	/* Reset counters */
	fa->n_shots = 0;
	fa->n_fires = 0;

	/* If START, check if we can start */
	if (command == FA100M14B4C_CMD_START) {
		/* Verify that SerDes PLL is lockes */
		val = fa_readl(fa, fa->fa_adc_csr_base,
			       &zfad_regs[ZFA_STA_SERDES_PLL]);
		if (!val) {
			dev_info(fa->msgdev, "Cannot start acquisition: "
				 "SerDes PLL not locked\n");
			return -EBUSY;
		}
		/* Verify that SerDes is synched */
		val = fa_readl(fa, fa->fa_adc_csr_base,
			       &zfad_regs[ZFA_STA_SERDES_SYNCED]);
		if (!val) {
			dev_info(fa->msgdev, "Cannot start acquisition: "
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
			dev_info(fa->msgdev, "Cannot start acquisition: "
				 "Trigger refuses to arm\n");
			return -EIO;
		}

		dev_dbg(fa->msgdev, "FSM START Command\n");
		fa_enable_irqs(fa);

		fa_writel(fa, fa->fa_adc_csr_base,
			  &zfad_regs[ZFA_CTL_RST_TRG_STA], 1);
	} else {
		dev_dbg(fa->msgdev, "FSM STOP Command\n");
		fa->enable_auto_start = 0;
		fa_disable_irqs(fa);
	}

	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFA_CTL_FMS_CMD],
		  command);
	return 0;
}

static void fa_init_timetag(struct fa_dev *fa)
{
	unsigned long seconds;

	seconds = get_seconds();
	fa_writel(fa, fa->fa_utc_base, &zfad_regs[ZFA_UTC_SECONDS_U],
		  (seconds >> 32) & 0xFFFFFFFF);
	fa_writel(fa, fa->fa_utc_base, &zfad_regs[ZFA_UTC_SECONDS_L],
		  (seconds >> 00) & 0xFFFFFFFF);
}

/*
 * Specific check and init
 */
static int __fa_init(struct fa_dev *fa)
{
	struct zio_device *zdev = fa->zdev;
	int i, addr;

	/* Use identity calibration */
	fa->mshot_max_samples = fa_readl(fa, fa->fa_adc_csr_base,
					 &zfad_regs[ZFA_MULT_MAX_SAMP]);

	/* Force stop FSM to prevent early trigger fire */
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFA_CTL_FMS_CMD],
		   FA100M14B4C_CMD_STOP);
	/* Initialize channels to use 1V range */
	for (i = 0; i < 4; ++i) {
		addr = zfad_get_chx_index(ZFA_CHx_CTL_RANGE,
					  zdev->cset->chan[i].index);
		fa_writel(fa,  fa->fa_adc_csr_base, &zfad_regs[addr],
			  FA100M14B4C_RANGE_1V);
		zfad_set_range(fa, &zdev->cset->chan[i], FA100M14B4C_RANGE_1V);
	}
	zfad_reset_offset(fa);

	/* Enable mezzanine clock */
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFA_CTL_CLK_EN], 1);
	/* Set decimation to minimum */
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFAT_SR_UNDER], 1);
	/* Set test data register */
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFA_CTL_TEST_DATA_EN],
		  fa_enable_test_data_fpga);

	/* Set to single shot mode by default */
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFAT_SHOTS_NB], 1);

	zfat_trigger_source_reset(fa);

	/* Zero offsets and release the DAC clear */
	zfad_reset_offset(fa);
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFA_CTL_DAC_CLR_N], 1);

	/* Initialize channel saturation values */
	zfad_init_saturation(fa);

	fa_init_timetag(fa);

	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFAT_EXT_DLY], 0);

	/* disable auto_start */
	fa->enable_auto_start = 0;
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
	{"zio", fa_zio_init, fa_zio_exit},
	{"debug", fa_debug_init, fa_debug_exit},
	{"calibration", fa_calib_init, fa_calib_exit},
};


static int fa_resource_validation(struct platform_device *pdev)
{
	struct resource *r;

	r = platform_get_resource(pdev, IORESOURCE_IRQ, ADC_IRQ_TRG);
	if (!r) {
		dev_err(&pdev->dev,
			"The ADC needs an interrupt number for the IRQ\n");
		return -ENXIO;
	}

	r = platform_get_resource(pdev, IORESOURCE_MEM, ADC_MEM_BASE);
	if (!r) {
		dev_err(&pdev->dev,
			"The ADC needs base address\n");
		return -ENXIO;
	}

	return 0;
}

#define FA_FMC_NAME "FmcAdc100m14b4cha"

static bool fa_fmc_slot_is_valid(struct fa_dev *fa)
{
	int ret;
	void *fru = NULL;
	char *fmc_name = NULL;

	if (!fmc_slot_fru_valid(fa->slot)) {
		dev_err(fa->msgdev, "Can't identify FMC card: invalid FRU\n");
		return -EINVAL;
	}

	fru = kmalloc(FRU_SIZE_MAX, GFP_KERNEL);
	if (!fru)
		return -ENOMEM;

	ret = fmc_slot_eeprom_read(fa->slot, fru, 0x0, FRU_SIZE_MAX);
	if (ret != FRU_SIZE_MAX) {
		dev_err(fa->msgdev, "Failed to read FRU header\n");
		goto err;
	}

	fmc_name = fru_get_product_name(fru);
	ret = strcmp(fmc_name, FA_FMC_NAME);
	if (ret) {
		dev_err(fa->msgdev,
			"Invalid FMC card: expectd '%s', found '%s'\n",
			FA_FMC_NAME, fmc_name);
		goto err;
	}

	kfree(fmc_name);
	kfree(fru);

	return true;
err:
	kfree(fmc_name);
	kfree(fru);
	return false;
}

static void fa_memops_detect(struct fa_dev *fa)
{
	if (fa_is_flag_set(fa, FMC_ADC_BIG_ENDIAN)) {
		fa->memops.read = ioread32be;
		fa->memops.write = iowrite32be;
	} else {
		fa->memops.read = ioread32;
		fa->memops.write = iowrite32;
	}
}

static void fa_sg_alloc_table_init(struct fa_dev *fa)
{
	if (fa_is_flag_set(fa, FMC_ADC_NOSQUASH_SCATTERLIST))
		fa->sg_alloc_table_from_pages = sg_alloc_table_from_pages_no_squash;
	else
		fa->sg_alloc_table_from_pages = __sg_alloc_table_from_pages;

}

static struct fmc_adc_platform_data fmc_adc_pdata_default = {
	.flags = 0,
	.calib_trig_time = 0,
	.calib_trig_threshold = 0,
	.calib_trig_internal = 0,
};

/* probe and remove are called by fa-spec.c */
int fa_probe(struct platform_device *pdev)
{
	struct fa_modlist *m = NULL;
	struct fa_dev *fa;
	struct resource *r;
	int err, i = 0, slot_nr;

	err = fa_resource_validation(pdev);
	if (err)
		return err;

	/* Driver data */
	fa = devm_kzalloc(&pdev->dev, sizeof(struct fa_dev), GFP_KERNEL);
	if (!fa)
		return -ENOMEM;

	platform_set_drvdata(pdev, fa);
	fa->pdev = pdev;
	fa->msgdev = &fa->pdev->dev;
	if (!pdev->dev.platform_data) {
		dev_err(fa->msgdev, "Missing platform data, use default\n");
		pdev->dev.platform_data = &fmc_adc_pdata_default;
	}

	fa_memops_detect(fa);
	fa_sg_alloc_table_init(fa);

	r = platform_get_resource(pdev, IORESOURCE_MEM, ADC_MEM_BASE);
	fa->fa_top_level = ioremap(r->start, resource_size(r));
	fa->fa_adc_csr_base = fa->fa_top_level + ADC_CSR_OFF;
	fa->fa_irq_adc_base = fa->fa_top_level + ADC_EIC_OFF;
	fa->fa_ow_base =      fa->fa_top_level + ADC_OW_OFF;
	fa->fa_spi_base =     fa->fa_top_level + ADC_SPI_OFF;
	fa->fa_utc_base =     fa->fa_top_level + ADC_UTC_OFF;

	slot_nr = fa_readl(fa, fa->fa_adc_csr_base,
			   &zfad_regs[ZFA_STA_FMC_NR]) + 1;
	fa->slot = fmc_slot_get(pdev->dev.parent->parent, slot_nr);
	if (IS_ERR(fa->slot)) {
		dev_err(fa->msgdev, "Can't find FMC slot %d err: %ld\n",
			slot_nr, PTR_ERR(fa->slot));
		goto out_fmc;
	}

	if (!fmc_slot_present(fa->slot)) {
		dev_err(fa->msgdev, "Can't identify FMC card: missing card\n");
		goto out_fmc_pre;
	}

	if (strcmp(fmc_slot_eeprom_type_get(fa->slot), FA_EEPROM_TYPE)) {
		dev_warn(fa->msgdev, "use non standard EERPOM type \"%s\"\n",
			 FA_EEPROM_TYPE);
		err = fmc_slot_eeprom_type_set(fa->slot, FA_EEPROM_TYPE);
		if (err) {
			dev_err(fa->msgdev,
				"Failed to change EEPROM type to \"%s\"",
				FA_EEPROM_TYPE);
			goto out_fmc_eeprom;
		}
	}

	if(!fa_fmc_slot_is_valid(fa))
		goto out_fmc_err;

	/* init all subsystems */
	for (i = 0, m = mods; i < ARRAY_SIZE(mods); i++, m++) {
		dev_dbg(fa->msgdev, "Calling init for \"%s\"\n", m->name);
		err = m->init(fa);
		if (err) {
			dev_err(fa->msgdev, "error initializing %s\n", m->name);
			goto out;
		}
	}

	/* time to execute specific driver init */
	err = __fa_init(fa);
	if (err < 0)
		goto out;

	err = fa_setup_irqs(fa);
	if (err < 0)
		goto out_irq;

	return 0;

out_irq:
out:
	while (--m, --i >= 0)
		if (m->exit)
			m->exit(fa);
	iounmap(fa->fa_top_level);
out_fmc_err:
out_fmc_eeprom:
out_fmc_pre:
	fmc_slot_put(fa->slot);
out_fmc:
	devm_kfree(&pdev->dev, fa);
	platform_set_drvdata(pdev, NULL);
	return err;
}

int fa_remove(struct platform_device *pdev)
{
	struct fa_dev *fa = platform_get_drvdata(pdev);
	struct fa_modlist *m;
	int i = ARRAY_SIZE(mods);

	if (WARN(!fa, "asked to remove fmc-adc-100m device but it does not exists\n"))
		return 0;

	fa_free_irqs(fa);
	flush_workqueue(fa_workqueue);

	while (--i >= 0) {
		m = mods + i;
		if (m->exit)
			m->exit(fa);
	}
	iounmap(fa->fa_top_level);

	fmc_slot_put(fa->slot);

	return 0;
}


static const struct platform_device_id fa_id[] = {
	{
		.name = "fmc-adc-100m",
		.driver_data = ADC_VER,
	}
	/* TODO we should support different version */
};

static struct platform_driver fa_dev_drv = {
	.driver = {
		.name = KBUILD_MODNAME,
	},
	.probe = fa_probe,
	.remove = fa_remove,
	.id_table = fa_id,
};

static int fa_init(void)
{
	int ret;

	#if LINUX_VERSION_CODE < KERNEL_VERSION(3,15,0)
	fa_workqueue = alloc_workqueue(fa_dev_drv.driver.name,
					WQ_NON_REENTRANT | WQ_UNBOUND |
					WQ_MEM_RECLAIM, 1);
	#else
	fa_workqueue = alloc_workqueue(fa_dev_drv.driver.name,
				       WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	#endif
	if (fa_workqueue == NULL)
		return -ENOMEM;

	/* First trigger and zio driver */
	ret = fa_trig_init();
	if (ret)
		goto out1;

	ret = fa_zio_register();
	if (ret)
		goto out2;

	/* Finally the fmc driver, whose probe instantiates zio devices */
	ret = platform_driver_register(&fa_dev_drv);
	if (ret)
		goto out3;

	return ret;

out3:
	fa_zio_unregister();
out2:
	fa_trig_exit();
out1:
	destroy_workqueue(fa_workqueue);

	return ret;
}

static void fa_exit(void)
{
	platform_driver_unregister(&fa_dev_drv);
	fa_zio_unregister();
	fa_trig_exit();
	if (fa_workqueue != NULL)
		destroy_workqueue(fa_workqueue);
}

module_init(fa_init);
module_exit(fa_exit);

MODULE_AUTHOR("Federico Vaga");
MODULE_DESCRIPTION("FMC-ADC-100MS-14b Linux Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(VERSION);

ADDITIONAL_VERSIONS;
