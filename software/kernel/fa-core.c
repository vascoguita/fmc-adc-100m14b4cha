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
#include <uapi/linux/ipmi/fru.h>
#include <linux/fmc.h>

#include "fmc-adc-100m14b4cha-private.h"
#include <platform_data/fmc-adc-100m14b4cha.h>

static int fa_enable_test_data_fpga;
module_param_named(enable_test_data_fpga, fa_enable_test_data_fpga, int, 0444);

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
 * Enable/Disable Data Output Randomizer
 * @fa: the adc descriptor
 * @enable:
 */
int fa_adc_output_randomizer_set(struct fa_dev *fa, bool enable)
{
	uint32_t tx, rx;
	int err;

        tx  = 0x8000;
	tx |= (1 << 8);
	err = fa_spi_xfer(fa, FA_SPI_SS_ADC, 16, tx, &rx);
	if (err)
		return err;

	if (enable)
		rx |= BIT(6);
	else
		rx &= ~BIT(6);

	tx  = 0x0000;
	tx |= (1 << 8);
	tx |= (rx & 0xFF);
	err = fa_spi_xfer(fa, FA_SPI_SS_ADC, 16, tx, NULL);
	if (err)
		return err;
	return 0;
}

/**
 * Check if the Data Output Randomizer is enabled
 * @fa: the adc descriptor
 * Return: true if the feature is enabled, otherwise false
 */
bool fa_adc_is_output_randomizer(struct fa_dev *fa)
{
	uint32_t tx, rx;
	int err;

        tx  = 0x8000;
	tx |= (1 << 8);
	err = fa_spi_xfer(fa, FA_SPI_SS_ADC, 16, tx, &rx);
	if (err)
		return false;

	return !!(rx & BIT(6));
}

/**
 * Read FMC mezzanine temperature
 * @fa: the adc descriptor
 *
 * DS18B20 returns units of 1/16 degree. We return units
 * of 1/1000 of a degree instead.
 */
int32_t fa_temperature_read(struct fa_dev *fa)
{
	uint32_t reg;
	int16_t raw_temp;

	reg = fa_ioread(fa, fa->fa_ow_base + 0x08);
	if (reg & BIT(31)) {
		dev_err(&fa->pdev->dev, "Temperature sensor failure\n");
		return 45000; /* 45.000 degrees as save value */
	}
	raw_temp = reg & 0xFFFF;

	return (raw_temp * 1000UL + 8) / 16;
}

/**
 * Do a software trigger
 * @fa: the adc descriptor
 *
 * Return: 0 on success, otherwise a negative error number
 */
int fa_trigger_software(struct fa_dev *fa)
{
	struct zio_ti *ti = fa->zdev->cset->ti;
	struct zio_attribute *ti_zattr = ti->zattr_set.std_zattr;
	unsigned int timeout;
	int err;

	/* Fire if software trigger is enabled (index 5) */
	if (!(ti->zattr_set.ext_zattr[FA100M14B4C_TATTR_SRC].value &
	      FA100M14B4C_TRG_SRC_SW)) {
		dev_info(&fa->pdev->dev, "sw trigger is not enabled\n");
		return -EPERM;
	}

        /* Fire if nsamples!=0 */
	if (!ti->nsamples) {
		dev_info(&fa->pdev->dev, "pre + post = 0: cannot acquire\n");
		return -EINVAL;
	}

        /*
	 * We can do a software trigger if the FSM is not in
	 * the WAIT trigger status. Wait for it.
	 * Remember that: timeout is in us, a sample takes 10ns
	 */
	timeout = ti_zattr[ZIO_ATTR_TRIG_PRE_SAMP].value / 10;
	err = fa_fsm_wait_state(fa, FA100M14B4C_STATE_WAIT, timeout);
	if (err)
		return err;
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFAT_SW], 1);

	return 0;
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
		sg_set_page(sg, pages[i], min_t(unsigned long, size, chunk_size), offset);
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
 * fa_adc_range_set
 * @fa: the fmc-adc descriptor
 * @chan: the channel to calibrate
 * @usr_val: the volt range to set and calibrate
 *
 * When the input range changes, we must write new fixup values.
 * Gain ad offsets must be corrected with offset and gain calibration value.
 * An open input and test data do not need any correction.
 */
int fa_adc_range_set(struct fa_dev *fa, struct zio_channel *chan, int range)
{
	int i;

	/* Actually set the range */
	i = zfad_get_chx_index(ZFA_CHx_CTL_RANGE, chan->index);
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[i], zfad_hw_range[range]);

	if (range == FA100M14B4C_RANGE_OPEN)
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

	return 0;
}

static enum fa100m14b4c_fsm_state fa_fsm_get_state(struct fa_dev *fa)
{
	return fa_readl(fa, fa->fa_adc_csr_base, &zfad_regs[ZFA_STA_FSM]);
}

static bool fa_fsm_is_state(struct fa_dev *fa,
			    enum fa100m14b4c_fsm_state state)
{
	return fa_fsm_get_state(fa) == state;
}

int fa_fsm_wait_state(struct fa_dev *fa,
		      enum fa100m14b4c_fsm_state state,
		      unsigned int timeout_us)
{
	unsigned long timeout;

	timeout = jiffies + usecs_to_jiffies(timeout_us);
	while (!fa_fsm_is_state(fa, state)) {
		cpu_relax();

		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
	}

        return 0;
}

static void fa_fpga_data_pattern_set(struct fa_dev *fa, unsigned int enable)
{
	fa_writel(fa, fa->fa_adc_csr_base,
		  &zfad_regs[ZFA_CTL_TEST_DATA_EN], enable);
}

/**
 * It enables or disables the pattern data on the ADC
 * @fa The ADC device instance
 * @pattern the pattern data to get from the ADC
 * @enable 0 to disable, 1 to enable
 */
int fa_adc_data_pattern_set(struct fa_dev *fa, uint16_t pattern,
			    unsigned int enable)
{
	uint32_t frame_tx;
	int err;

	dev_dbg(&fa->pdev->dev, "%s {patter: 0x%04x, enable: %d}\n", __func__, pattern, enable);

	spin_lock(&fa->zdev->cset->lock);
	/* Disable before Setting the pattern data */
	frame_tx  = 0x0000; /* write mode */
	frame_tx |= 0x0300; /* A3 pattern + enable */
	err = fa_spi_xfer(fa, FA_SPI_SS_ADC, 16, frame_tx, NULL);
	if (err)
		goto err;

	if (!enable) {
		fa->flags &= ~FA_DEV_F_PATTERN_DATA;
		goto err;
	}

	frame_tx  = 0x0000; /* write mode */
	frame_tx |= 0x0400; /* A4 pattern */
	frame_tx |= pattern & 0xFF; /* LSB pattern */
	err = fa_spi_xfer(fa, FA_SPI_SS_ADC, 16, frame_tx, NULL);
	if (err)
		goto err;

	frame_tx  = 0x0000; /* write mode */
	frame_tx |= 0x0300; /* A3 pattern + enable */
	frame_tx |= (pattern & 0x3F00) >> 8; /* MSB pattern */
	frame_tx |= 0x80; /* Enable the pattern data */
	err = fa_spi_xfer(fa, FA_SPI_SS_ADC, 16, frame_tx, NULL);
	if (err)
		goto err;


	fa->flags |= FA_DEV_F_PATTERN_DATA;
err:
	spin_unlock(&fa->zdev->cset->lock);
	return err;
}

/**
 * Get current status for data pattern
 * @fa The ADC device instance
 * @pattern the pattern data to get from the ADC
 * @enable 0 to disable, 1 to enable
 */
int fa_adc_data_pattern_get(struct fa_dev *fa, uint16_t *pattern,
                            unsigned int *enable)
{
	uint32_t tx, rx;
	int err;

	tx = 0x8000 | (3 << 8);
	err = fa_spi_xfer(fa, FA_SPI_SS_ADC, 16, tx, &rx);
	if (err)
		return err;
	*enable = !!(rx & 0x80);
	*pattern = ((rx & 0x3F) << 8);

	tx = 0x8000 | (4 << 8);
	err = fa_spi_xfer(fa, FA_SPI_SS_ADC, 16, tx, &rx);
	if (err)
		return err;
	*pattern |= (rx & 0xFF);

	return 0;
}

static bool fa_adc_is_serdes_pll_locked(struct fa_dev *fa)
{
	return fa_readl(fa, fa->fa_adc_csr_base,
			&zfad_regs[ZFA_STA_SERDES_PLL]);
}

static bool fa_adc_is_serdes_synced(struct fa_dev *fa)
{
	return fa_readl(fa, fa->fa_adc_csr_base,
			&zfad_regs[ZFA_STA_SERDES_SYNCED]);
}

static bool fa_adc_is_serdes_ready(struct fa_dev *fa)
{
	return fa_adc_is_serdes_pll_locked(fa) && fa_adc_is_serdes_synced(fa);
}

static int fa_adc_wait_serdes_ready(struct fa_dev *fa, unsigned int timeout)
{
	unsigned long j;

	j = jiffies + timeout;

	while (!fa_adc_is_serdes_ready(fa)) {
		if (time_after(jiffies, j))
			return -ETIMEDOUT;
		cpu_relax();
	}

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
	struct zio_cset *cset = fa->zdev->cset;

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
		if (!fa_adc_is_serdes_ready(fa)) {
			dev_err(fa->msgdev,
				"Cannot start acquisition: "
				"SerDes PLL not locked or synchronized (0x%08x)\n",
				fa_ioread(fa, fa->fa_adc_csr_base + ADC_CSR_STA_REG_OFFSET));
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

static void fa_clock_enable(struct fa_dev *fa)
{
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFA_CTL_CLK_EN], 1);
}

static void fa_clock_disable(struct fa_dev *fa)
{
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFA_CTL_CLK_EN], 0);
}

/*
 * Specific check and init
 */
static int __fa_init(struct fa_dev *fa)
{
	struct zio_device *zdev = fa->zdev;
	int i;

	/* Use identity calibration */
	fa->mshot_max_samples = fa_readl(fa, fa->fa_adc_csr_base,
					 &zfad_regs[ZFA_MULT_MAX_SAMP]);

	/* Force stop FSM to prevent early trigger fire */
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFA_CTL_FMS_CMD],
		   FA100M14B4C_CMD_STOP);
	/* Initialize channels to use 1V range */
	for (i = 0; i < 4; ++i) {
		int addr = zfad_get_chx_index(ZFA_CHx_CTL_RANGE,
					      zdev->cset->chan[i].index);
		fa_writel(fa,  fa->fa_adc_csr_base, &zfad_regs[addr],
			  FA100M14B4C_RANGE_1V);
	        fa_adc_range_set(fa, &zdev->cset->chan[i],
				 FA100M14B4C_RANGE_1V);
		/* reset channel offset */
		fa->user_offset[i] = 0;
		fa->zero_offset[i] = 0;
	}

	/* Set decimation to minimum */
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFAT_SR_UNDER], 1);
	/* Set test data register */
	fa_fpga_data_pattern_set(fa, fa_enable_test_data_fpga);
	/* disable test pattern data in the ADC */
	fa_adc_data_pattern_set(fa, 0, 0);

	/* Set to single shot mode by default */
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFAT_SHOTS_NB], 1);

	zfat_trigger_source_reset(fa);

	/* Zero offsets and release the DAC clear */
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
	.vme_reg_offset = 0,
        .vme_dma_offset = 0,
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

	err = sysfs_create_link(&fa->pdev->dev.kobj, &fa->slot->dev.kobj,
				dev_name(&fa->slot->dev));
	if (err) {
		dev_err(&fa->pdev->dev, "Failed to create FMC symlink to %s\n",
			dev_name(&fa->slot->dev));
		goto err_fmc_link;
	}

	err = fa_dma_request_channel(fa);
	if (err)
		goto out_dma;

	fa_clock_enable(fa);

	err = fa_adc_wait_serdes_ready(fa, msecs_to_jiffies(10));
	if (err)
		goto out_serdes;

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
out_serdes:
	fa_clock_disable(fa);
	fa_dma_release_channel(fa);
out_dma:
	sysfs_remove_link(&fa->pdev->dev.kobj, dev_name(&fa->slot->dev));
err_fmc_link:
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
	int i = ARRAY_SIZE(mods);

	if (WARN(!fa, "asked to remove fmc-adc-100m device but it does not exists\n"))
		return 0;

	fa_free_irqs(fa);
	flush_workqueue(fa_workqueue);

	while (--i >= 0) {
		struct fa_modlist *m = mods + i;
		if (m->exit)
			m->exit(fa);
	}
	fa_clock_disable(fa);
	fa_dma_release_channel(fa);

	iounmap(fa->fa_top_level);

	sysfs_remove_link(&fa->pdev->dev.kobj, dev_name(&fa->slot->dev));
	fmc_slot_put(fa->slot);

	return 0;
}


static const struct platform_device_id fa_id[] = {
	{
		.name = "fmc-adc-100m",
		.driver_data = ADC_VER,
	},
	{},
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
MODULE_DEVICE_TABLE(platform, fa_id);

ADDITIONAL_VERSIONS;
