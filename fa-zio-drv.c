/*
 * Copyright CERN 2012
 * Author: Federico Vaga <federico.vaga@gmail.com>
 *
 * Driver for the mezzanine ADC for the SPEC
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/list.h>

#include <linux/zio.h>
#include <linux/zio-buffer.h>
#include <linux/zio-trigger.h>

#include "spec.h"
#include "fmc-adc.h"

/* Definition of the fmc-adc registers address - mask - mask offset */
const struct zio_reg_desc zfad_regs[] = {
	/* Control registers */
	[ZFA_CTL_FMS_CMD] =		{FA_ADC_MEM_OFF + 0x00, 0x0003, 0},
	[ZFA_CTL_CLK_EN] =		{FA_ADC_MEM_OFF + 0x00, 0x0001, 2},
	[ZFA_CTL_DAC_CLR_N] =		{FA_ADC_MEM_OFF + 0x00, 0x0001, 3},
	[ZFA_CTL_BSLIP] =		{FA_ADC_MEM_OFF + 0x00, 0x0001, 4},
	[ZFA_CTL_TEST_DATA_EN] =	{FA_ADC_MEM_OFF + 0x00, 0x0001, 5},
	[ZFA_CTL_TRIG_LED] =		{FA_ADC_MEM_OFF + 0x00, 0x0001, 6},
	[ZFA_CTL_ACQ_LED] =		{FA_ADC_MEM_OFF + 0x00, 0x0001, 7},
	/* Status registers */
	[ZFA_STA_FSM] =			{FA_ADC_MEM_OFF + 0x04, 0x0007, 0},
	[ZFA_STA_SERDES_PLL] =		{FA_ADC_MEM_OFF + 0x04, 0x0001, 3},
	[ZFA_STA_SERDES_SYNCED] =	{FA_ADC_MEM_OFF + 0x04, 0x0001, 4},
	/* Trigger */
		/* Config register */
	[ZFDAC_CFG_HW_SEL] =		{FA_ADC_MEM_OFF + 0x08, 0x00000001, 0},
	[ZFAT_CFG_HW_POL] =		{FA_ADC_MEM_OFF + 0x08, 0x00000001, 1},
	[ZFAT_CFG_HW_EN] =		{FA_ADC_MEM_OFF + 0x08, 0x00000001, 2},
	[ZFAT_CFG_SW_EN] =		{FA_ADC_MEM_OFF + 0x08, 0x00000001, 3},
	[ZFAT_CFG_INT_SEL] =		{FA_ADC_MEM_OFF + 0x08, 0x00000003, 4},
	[ZFAT_CFG_THRES] =		{FA_ADC_MEM_OFF + 0x08, 0x0000FFFF, 16},
		/* Delay */
	[ZFAT_DLY] =			{FA_ADC_MEM_OFF + 0x0C, 0xFFFFFFFF, 0},
		/* Software */
	[ZFAT_SW] =			{FA_ADC_MEM_OFF + 0x10, 0xFFFFFFFF, 0},
		/* Number of shots */
	[ZFAT_SHOTS_NB] =		{FA_ADC_MEM_OFF + 0x14, 0x0000FFFF, 0},
		/* Sample rate */
	[ZFAT_SR_DECI] =		{FA_ADC_MEM_OFF + 0x1C, 0xFFFF, 0},
		/* Position address */
	[ZFAT_POS] =			{FA_ADC_MEM_OFF + 0x18, 0xFFFFFFFF, 0},
		/* Pre-sample */
	[ZFAT_PRE] =			{FA_ADC_MEM_OFF + 0x20, 0xFFFFFFFF, 0},
		/* Post-sample */
	[ZFAT_POST] =			{FA_ADC_MEM_OFF + 0x24, 0xFFFFFFFF, 0},
		/* Sample counter */
	[ZFAT_CNT] =			{FA_ADC_MEM_OFF + 0x28, 0xFFFFFFFF, 0},
	/* Channel 1 */
	[ZFA_CH1_CTL_RANGE] =		{FA_ADC_MEM_OFF + 0x2C, 0x007F, 0},
	[ZFA_CH1_CTL_TERM] =		{FA_ADC_MEM_OFF + 0x2C, 0x0001, 3},
	[ZFA_CH1_STA] =			{FA_ADC_MEM_OFF + 0x30, 0xFFFF, 0},
	[ZFA_CH1_GAIN] =		{FA_ADC_MEM_OFF + 0x34, 0xFFFF, 0},
	[ZFA_CH1_OFFSET] =		{FA_ADC_MEM_OFF + 0x38, 0xFFFF, 0},
	/* Channel 2 */
	[ZFA_CH2_CTL_RANGE] =		{FA_ADC_MEM_OFF + 0x3C, 0x007F, 0},
	[ZFA_CH2_CTL_TERM] =		{FA_ADC_MEM_OFF + 0x3C, 0x0001, 3},
	[ZFA_CH2_STA] =			{FA_ADC_MEM_OFF + 0x40, 0xFFFF, 0},
	[ZFA_CH2_GAIN] =		{FA_ADC_MEM_OFF + 0x44, 0xFFFF, 0},
	[ZFA_CH2_OFFSET] =		{FA_ADC_MEM_OFF + 0x48, 0xFFFF, 0},
	/* Channel 3 */
	[ZFA_CH3_CTL_RANGE] =		{FA_ADC_MEM_OFF + 0x4C, 0x007F, 0},
	[ZFA_CH3_CTL_TERM] =		{FA_ADC_MEM_OFF + 0x4C, 0x0001, 3},
	[ZFA_CH3_STA] =			{FA_ADC_MEM_OFF + 0x50, 0xFFFF, 0},
	[ZFA_CH3_GAIN] =		{FA_ADC_MEM_OFF + 0x54, 0xFFFF, 0},
	[ZFA_CH3_OFFSET] =		{FA_ADC_MEM_OFF + 0x58, 0xFFFF, 0},
	/* Channel 4 */
	[ZFA_CH4_CTL_RANGE] =		{FA_ADC_MEM_OFF + 0x5C, 0x007F, 0},
	[ZFA_CH4_CTL_TERM] =		{FA_ADC_MEM_OFF + 0x5C, 0x0001, 3},
	[ZFA_CH4_STA] =			{FA_ADC_MEM_OFF + 0x60, 0xFFFF, 0},
	[ZFA_CH4_GAIN] =		{FA_ADC_MEM_OFF + 0x64, 0xFFFF, 0},
	[ZFA_CH4_OFFSET] =		{FA_ADC_MEM_OFF + 0x68, 0xFFFF, 0},
	/* DMA */
	[ZFA_DMA_CTL_SWP] =		{FA_DMA_MEM_OFF + 0x00, 0x0003, 2},
	[ZFA_DMA_CTL_ABORT] =		{FA_DMA_MEM_OFF + 0x00, 0x0001, 1},
	[ZFA_DMA_CTL_START] =		{FA_DMA_MEM_OFF + 0x00, 0x0001, 0},
	[ZFA_DMA_STA] =			{FA_DMA_MEM_OFF + 0x04, 0x0007, 0},
	[ZFA_DMA_ADDR] =		{FA_DMA_MEM_OFF + 0x08, 0xFFFFFFFF, 0},
	[ZFA_DMA_ADDR_L] =		{FA_DMA_MEM_OFF + 0x0C, 0xFFFFFFFF, 0},
	[ZFA_DMA_ADDR_H] =		{FA_DMA_MEM_OFF + 0x10, 0xFFFFFFFF, 0},
	[ZFA_DMA_LEN] =			{FA_DMA_MEM_OFF + 0x14, 0xFFFFFFFF, 0},
	[ZFA_DMA_NEXT_L] =		{FA_DMA_MEM_OFF + 0x18, 0xFFFFFFFF, 0},
	[ZFA_DMA_NEXT_H] =		{FA_DMA_MEM_OFF + 0x1C, 0xFFFFFFFF, 0},
	[ZFA_DMA_BR_DIR] =		{FA_DMA_MEM_OFF + 0x20, 0x0001, 1},
	[ZFA_DMA_BR_LAST] =		{FA_DMA_MEM_OFF + 0x20, 0x0001, 0},
	/* IRQ */
	[ZFA_IRQ_MULTI] =		{FA_IRQ_MEM_OFF + 0x00, 0x000F, 0},
	[ZFA_IRQ_SRC] =			{FA_IRQ_MEM_OFF + 0x04, 0x000F, 0},
	[ZFA_IRQ_MASK] =		{FA_IRQ_MEM_OFF + 0x08, 0x000F, 0},
	/* UTC */
	[ZFA_UTC_SECONDS] =		{FA_UTC_MEM_OFF + 0x00, ~0x0, 0},
	[ZFA_UTC_COARSE] =		{FA_UTC_MEM_OFF + 0x04, ~0x0, 0},
	[ZFA_UTC_TRIG_META] =		{FA_UTC_MEM_OFF + 0x08, ~0x0, 0},
	[ZFA_UTC_TRIG_SECONDS] =	{FA_UTC_MEM_OFF + 0x0C, ~0x0, 0},
	[ZFA_UTC_TRIG_COARSE] =		{FA_UTC_MEM_OFF + 0x10, ~0x0, 0},
	[ZFA_UTC_TRIG_FINE] =		{FA_UTC_MEM_OFF + 0x14, ~0x0, 0},
	[ZFA_UTC_ACQ_START_META] 	{FA_UTC_MEM_OFF + 0x18, ~0x0, 0},
	[ZFA_UTC_ACQ_START_SECONDS] =	{FA_UTC_MEM_OFF + 0x1C, ~0x0, 0},
	[ZFA_UTC_ACQ_START_COARSE] =	{FA_UTC_MEM_OFF + 0x20, ~0x0, 0},
	[ZFA_UTC_ACQ_START_FINE] =	{FA_UTC_MEM_OFF + 0x24, ~0x0, 0},
	[ZFA_UTC_ACQ_STOP_META] =	{FA_UTC_MEM_OFF + 0x28, ~0x0, 0},
	[ZFA_UTC_ACQ_STOP_SECONDS] =	{FA_UTC_MEM_OFF + 0x2C, ~0x0, 0},
	[ZFA_UTC_ACQ_STOP_COARSE] =	{FA_UTC_MEM_OFF + 0x30, ~0x0, 0},
	[ZFA_UTC_ACQ_STOP_FINE] =	{FA_UTC_MEM_OFF + 0x34, ~0x0, 0},
	[ZFA_UTC_ACQ_END_META] =	{FA_UTC_MEM_OFF + 0x38, ~0x0, 0},
	[ZFA_UTC_ACQ_END_SECONDS] =	{FA_UTC_MEM_OFF + 0x3C, ~0x0, 0},
	[ZFA_UTC_ACQ_END_COARSE] =	{FA_UTC_MEM_OFF + 0x40, ~0x0, 0},
	[ZFA_UTC_ACQ_END_FINE] =	{FA_UTC_MEM_OFF + 0x44, ~0x0, 0},
};
#define FA_CHAN_REG_OFF 5

/* zio device attributes */
static DEFINE_ZATTR_STD(ZDEV, zfad_dev_std_zattr) = {
	ZATTR_REG(zdev, ZATTR_NBITS, S_IRUGO, ZFA_SW_R_NOADDRES, 14),
	/* Sample rate */
	ZATTR_REG(zdev, ZATTR_MAXRATE, S_IRUGO | S_IWUGO, ZFAT_SR_DECI, 0),
};
static struct zio_attribute zfad_dev_ext_zattr[] = {
	/* Control register */
	/*
	 * State machine commands
	 * 1: start
	 * 2: stop
	 */
	PARAM_EXT_REG("fsm-cmd", S_IWUGO, ZFA_CTL_FMS_CMD, 0),
	/* FMC clock, must be enabled  */
	ZATTR_EXT_REG("fmc-clk-en", S_IRUGO | S_IWUGO, ZFA_CTL_CLK_EN, 1),
	ZATTR_EXT_REG("offset-dac-clr-n", S_IRUGO | S_IWUGO, ZFA_CTL_DAC_CLR_N, 0),
	ZATTR_EXT_REG("bitslip", S_IRUGO | S_IWUGO, ZFA_CTL_BSLIP, 0),
	ZATTR_EXT_REG("test-data-en", S_IRUGO | S_IWUGO, ZFA_CTL_TEST_DATA_EN, 0),
	PARAM_EXT_REG("trig-led", S_IRUGO | S_IWUGO, ZFA_CTL_TRIG_LED, 0),
	PARAM_EXT_REG("acq-led", S_IRUGO | S_IWUGO, ZFA_CTL_ACQ_LED, 0),

	/* Status register */
	/*
	 * fsm - status of the state machine:
	 * 1: IDLE
	 * 2: PRE_TRIG
	 * 3: WAIT_TRIG
	 * 4: POST_TRIG
	 * 5: DECR_SHOT
	 * 7: Illegal
	 * */
	PARAM_EXT_REG("fsm-state", S_IRUGO, ZFA_STA_FSM, 0),
	PARAM_EXT_REG("serdes-pll", S_IRUGO, ZFA_STA_SERDES_PLL, 0),
	PARAM_EXT_REG("serdes-synced", S_IRUGO, ZFA_STA_SERDES_SYNCED, 0),

	/* TIME STAMPS */
	PARAM_EXT_REG("trig-sec", S_IRUGO, ZFA_UTC_TRIG_SECONDS, 0),
	PARAM_EXT_REG("trig-ticks", S_IRUGO, ZFA_UTC_TRIG_COARSE, 0),
	PARAM_EXT_REG("trig-bins", S_IRUGO, ZFA_UTC_TRIG_FINE, 0),
	PARAM_EXT_REG("acq-start-sec", S_IRUGO, ZFA_UTC_ACQ_START_SECONDS, 0),
	PARAM_EXT_REG("acq-start-ticks", S_IRUGO, ZFA_UTC_ACQ_START_COARSE, 0),
	PARAM_EXT_REG("acq-start-bins", S_IRUGO, ZFA_UTC_ACQ_START_FINE, 0),
	PARAM_EXT_REG("acq-end-sec", S_IRUGO, ZFA_UTC_ACQ_END_SECONDS, 0),
	PARAM_EXT_REG("acq-end-ticks", S_IRUGO, ZFA_UTC_ACQ_END_COARSE, 0),
	PARAM_EXT_REG("acq-end-bins", S_IRUGO, ZFA_UTC_ACQ_END_FINE, 0),
	PARAM_EXT_REG("acq-stop-sec", S_IRUGO, ZFA_UTC_ACQ_STOP_SECONDS, 0),
	PARAM_EXT_REG("acq-stop-ticks", S_IRUGO, ZFA_UTC_ACQ_STOP_COARSE, 0),
	PARAM_EXT_REG("acq-stop-bins", S_IRUGO, ZFA_UTC_ACQ_STOP_FINE, 0),
};
static DEFINE_ZATTR_STD(ZDEV, zfad_chan_std_zattr) = {
	ZATTR_REG(zdev, ZATTR_GAIN, S_IRUGO | S_IWUGO, ZFA_CHx_GAIN, 0x30),
	ZATTR_REG(zdev, ZATTR_OFFSET, S_IRUGO | S_IWUGO, ZFA_CHx_OFFSET, 14),
};

static struct zio_attribute zfad_chan_ext_zattr[] = {
	/*
	 * in-range
	 * 0x23 (35): 100mV range
	 * 0x11 (17): 1V range
	 * 0x45 (69): 10V range
	 * 0x00 (0): Open input
	 * 0x42 (66): 100mV range calibration
	 * 0x40 (64): 1V range calibration
	 * 0x44 (68): 10V range calibration
	 */
	ZATTR_EXT_REG("in-range", S_IRUGO, ZFA_CHx_CTL_RANGE, 0),
	ZATTR_EXT_REG("termination-en", S_IRUGO, ZFA_CHx_CTL_TERM, 0),
	PARAM_EXT_REG("current-value", S_IRUGO, ZFA_CHx_STA, 0),
};


/* set a value to a FMC-ADC registers */
static int zfad_conf_set(struct device *dev, struct zio_attribute *zattr,
		uint32_t usr_val)
{
	const struct zio_reg_desc *reg;
	int i, chan_index;

	switch (zattr->priv.addr) {
		case ZFA_CHx_CTL_RANGE:
			if (usr_val != 0x23 || usr_val != 0x11 ||
			    usr_val != 0x45 || usr_val != 0x00) {
				dev_err(dev, " in_range valid value: 0 17 35 69");
				return -EINVAL;
			}
		case ZFA_CHx_CTL_TERM:
		case ZFA_CHx_GAIN:
		case ZFA_CHx_OFFSET:
			chan_index = to_zio_chan(dev)->index;
			i = zattr->priv.addr + ZFA_CHx_MULT * (chan_index -4);
			reg = &zfad_regs[ZFA_CH1_OFFSET + i];
			break;
		default:
			reg = &zfad_regs[zattr->priv.addr];
	}

	return zfa_common_conf_set(dev, reg, usr_val);
}
/* get the value of a FMC-ADC register */
static int zfad_info_get(struct device *dev, struct zio_attribute *zattr,
		uint32_t *usr_val)
{
	const struct zio_reg_desc *reg;
	int i, chan_index;

	switch (zattr->priv.addr) {
		case ZFA_CHx_CTL_RANGE:
		case ZFA_CHx_CTL_TERM:
		case ZFA_CHx_GAIN:
		case ZFA_CHx_OFFSET:
			chan_index = to_zio_chan(dev)->index;
			i = zattr->priv.addr + ZFA_CHx_MULT * (chan_index -4);
			reg = &zfad_regs[ZFA_CH1_OFFSET + i];
			break;
		default:
			reg = &zfad_regs[zattr->priv.addr];
	}

	zfa_common_info_get(dev, reg, usr_val);

	return 0;
}
static const struct zio_sysfs_operations zfad_s_op = {
	.conf_set = zfad_conf_set,
	.info_get = zfad_info_get,
};


/*
 * Prepare the FMC-ADC for the DMA transfer. FMC-ADC fire the hardware trigger,
 * it acquires all samples in its DDR memory and then it allows the driver to
 * transfer data through DMA. So zfad_input_cset only configure and start the
 * DMA transfer.
 */
static int zfad_input_cset(struct zio_cset *cset)
{
	int err;

	/* ZIO should configure only the interleaved channel */
	if (!cset->interleave)
		return -EINVAL;
	/* nsamples can't be 0 */
	if (!cset->interleave->current_ctrl->nsamples)
		return -EINVAL;

	err = zfad_map_dma(cset);
	if (err)
		return err;

	/* Start DMA transefer */
	zfa_common_conf_set(&cset->head.dev, &zfad_regs[ZFA_DMA_CTL_START], 1);
	return -EAGAIN; /* data_done on DMA_DONE interrupt */
}

static int zfad_zio_probe(struct zio_device *zdev)
{
	const struct zio_reg_desc *reg;
	int i;

	dev_dbg(&zdev->head.dev, "%s:%d", __func__, __LINE__);
	/* Force stop FSM */
	zfa_common_conf_set(&zdev->head.dev, &zfad_regs[ZFA_CTL_FMS_CMD],
			    ZFA_STOP);

	/* Initialize channels gain to 1 and range to 10V */
	for (i = 0; i < 4; ++i) {
		reg = &zfad_regs[ZFA_CH1_GAIN + (i * FA_CHAN_REG_OFF)];
		zfa_common_conf_set(&zdev->head.dev, reg, 0x8000);
		reg = &zfad_regs[ZFA_CH1_CTL_RANGE + (i * FA_CHAN_REG_OFF)];
		zfa_common_conf_set(&zdev->head.dev, reg, 0x45);
	}
	/* Enable mezzanine clock */
	zfa_common_conf_set(&zdev->head.dev, &zfad_regs[ZFA_CTL_CLK_EN], 1);
	/* Enable offset DACs FIXME clear active low ???? */
	zfa_common_conf_set(&zdev->head.dev, &zfad_regs[ZFA_CTL_DAC_CLR_N], 0);
	/* Set DMA to transfer data from device to host */
	zfa_common_conf_set(&zdev->head.dev, &zfad_regs[ZFA_DMA_BR_DIR], 0);
	/* Enable all interrupt FIXME enable one at time? */
	zfa_common_conf_set(&zdev->head.dev, &zfad_regs[ZFA_IRQ_MASK],
			    ZFAT_ALL);

	return 0;
}


/* Device description */
static struct zio_channel zfad_chan_tmpl = {
	.zattr_set = {
		.std_zattr = zfad_chan_std_zattr,
		.ext_zattr = zfad_chan_ext_zattr,
		.n_ext_attr = ARRAY_SIZE(zfad_chan_ext_zattr),
	},
};
static struct zio_cset zfad_cset[] = {
	{
		.raw_io = zfad_input_cset,
		.ssize = 2,
		.n_chan = 4,
		.chan_template = &zfad_chan_tmpl,
		.flags =  ZCSET_TYPE_ANALOG |	/* is analog */
			  ZIO_DIR_INPUT |	/* is input */
			  ZCSET_INTERLEAVE_ONLY,/* interleave only */
		/* .priv_d is used by DMA code */
	}
};

static struct zio_device zfad_tmpl = {
	.owner = THIS_MODULE,
	.s_op = &zfad_s_op,
	.flags = 0,
	.cset = zfad_cset,
	.n_cset = ARRAY_SIZE(zfad_cset),
	.zattr_set = {
		.std_zattr = zfad_dev_std_zattr,
		.ext_zattr = zfad_dev_ext_zattr,
		.n_ext_attr = ARRAY_SIZE(zfad_dev_ext_zattr),
	},
	/* This driver work only with the fmc-adc-trig */
	.preferred_trigger = "fmc-adc-trig",
	.preferred_buffer = "vmalloc",
};

static const struct zio_device_id zfad_table[] = {
	{"fmc-adc", &zfad_tmpl},
	{},
};

static struct zio_driver fa_zdrv = {
	.driver = {
		.name = "fmc-adc",
		.owner = THIS_MODULE,
	},
	.id_table = zfad_table,
	.probe = zfad_zio_probe,
};

/* Register and unregister are used to set up the template driver */
int fa_zio_register(void)
{
	return zio_register_driver(&fa_zdrv);
}

void fa_zio_unregister(void)
{
	zio_unregister_driver(&fa_zdrv);
}


/* Init and exit are called for each FMC-ADC card we have */
int fa_zio_init(struct spec_fa *fa)
{
	struct device *hwdev = fa->fmc->hwdev;
	struct spec_dev *spec = fa->fmc->carrier_data;
	struct pci_dev *pdev = spec->pdev;
	uint32_t dev_id;
	int err;

	/* Check if hardware support 64-bit DMA */
	if(dma_set_mask(hwdev, DMA_BIT_MASK(64))) {
		dev_err(hwdev, "64-bit DMA addressing not available\n");
		return -EINVAL;
	}

	/* Allocate the hardware zio_device for registration */
	fa->hwzdev = zio_allocate_device();
	if (IS_ERR(fa->hwzdev))
		return PTR_ERR(fa->hwzdev);

	/* Mandatory fields */
	fa->hwzdev->owner = THIS_MODULE;
	fa->hwzdev->priv_d = fa;

	/* Our dev_id is bus+devfn*/
	dev_id = (pdev->bus->number << 8) | pdev->devfn;

	/* Register our trigger hardware */
	err = zio_register_trig(&zfat_type, "fmc-adc-trig");
	if (err)
		goto out_trg;
	/* Register our device */
	err = zio_register_device(fa->hwzdev, "fmc-adc", dev_id);
	if (err)
		goto out_dev;

	return 0;

out_dev:
	zio_unregister_device(fa->hwzdev);
out_trg:
	zio_free_device(fa->hwzdev);
	return err;
}

void fa_zio_exit(struct spec_fa *fa)
{
	zio_unregister_device(fa->hwzdev);
	zio_free_device(fa->hwzdev);
	zio_unregister_trig(&zfat_type);
}
