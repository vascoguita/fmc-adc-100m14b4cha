/*
 * Copyright CERN 2012, author Federico Vaga
 *
 * Trigger for the driver of the mezzanine ADC
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>

#include <linux/zio.h>
#include <linux/zio-sysfs.h>
#include <linux/zio-buffer.h>
#include <linux/zio-trigger.h>
#include <linux/zio-utils.h>

#include "spec.h"
#include "fmc-adc.h"

struct zfat_instance {
	struct zio_ti ti;
	struct fa_dev *fa;
	unsigned int n_acq_dev;	/* number of acquisitions on device memory */
	unsigned int n_err;	/* number of errors */
};

#define to_zfat_instance(_ti) container_of(_ti, struct zfat_instance, ti)

/* zio trigger attributes */
static ZIO_ATTR_DEFINE_STD(ZIO_TRG, zfat_std_zattr) = {
	/* Number of shots */
	ZIO_ATTR(trig, ZIO_ATTR_TRIG_N_SHOTS, ZIO_RW_PERM, ZFAT_SHOTS_NB, 1),
	ZIO_ATTR(trig, ZIO_ATTR_TRIG_PRE_SAMP, ZIO_RW_PERM, ZFAT_PRE, 0),
	ZIO_ATTR(trig, ZIO_ATTR_TRIG_POST_SAMP, ZIO_RW_PERM, ZFAT_POST, 0),
};
static struct zio_attribute zfat_ext_zattr[] = {
	/* Config register */
	/* Hardware trigger selction
	 * 0: internal (data threshold)
	 * 1: external (front panel trigger input)
	 */
	ZIO_ATTR_EXT("external", ZIO_RW_PERM, ZFAT_CFG_HW_SEL, 0),
	/*
	 * Internal Hardware trigger polarity
	 * 0: positive edge/slope
	 * 1: negative edge/slope
	 */
	ZIO_ATTR_EXT("polarity", ZIO_RW_PERM, ZFAT_CFG_HW_POL, 0),
	/*
	 * Channel selection for internal trigger
	 * 0: channel 1, 1: channel 2, 2: channel 3, 3: channel 4
	 */
	ZIO_ATTR_EXT("int-channel", ZIO_RW_PERM, ZFAT_CFG_INT_SEL, 0),
	/* Internal trigger threshold value is 2 complement format */
	ZIO_ATTR_EXT("int-threshold", ZIO_RW_PERM, ZFAT_CFG_THRES, 0),
	/*
	 * Delay to apply on the trigger in sampling clock period. The default
	 * clock frequency is 100MHz (period = 10ns)
	 */
	ZIO_ATTR_EXT("delay", ZIO_RW_PERM, ZFAT_DLY, 0),

	/* Software Trigger */
	/* Enable (1) or disable (0) software trigger */
	ZIO_PARAM_EXT("sw-trg-enable", ZIO_RW_PERM, ZFAT_CFG_SW_EN, 0),
	ZIO_PARAM_EXT("sw-trg-fire", S_IWUGO, ZFAT_SW, 0),

	/* last trigger time stamp */
	ZIO_PARAM_EXT("tstamp-trg-lst-s", ZIO_RO_PERM, ZFA_UTC_TRIG_SECONDS, 0),
	ZIO_PARAM_EXT("tstamp-trg-lst-t", ZIO_RO_PERM, ZFA_UTC_TRIG_COARSE, 0),
	ZIO_PARAM_EXT("tstamp-trg-lst-b", ZIO_RO_PERM, ZFA_UTC_TRIG_FINE, 0),
};


/* set a value to a FMC-ADC trigger register */
static int zfat_conf_set(struct device *dev, struct zio_attribute *zattr,
			 uint32_t usr_val)
{
	struct fa_dev *fa = get_zfadc(dev);
	struct zio_ti *ti = to_zio_ti(dev);
	uint32_t tmp_val = usr_val;
	int err = 0;

	switch (zattr->id) {
	case ZFAT_SHOTS_NB:
	case ZFAT_PRE:
	case ZFAT_POST:
		err = zfat_overflow_detection(ti, zattr->id, tmp_val);
		if (err)
			return err;
		break;
	case ZFAT_SW:
		/* Fire if software trigger is enabled (index 5) */
		if (!ti->zattr_set.ext_zattr[5].value) {
			dev_err(dev, "sw trigger must be enable");
			return -EPERM;
		}
		/* Fire if nsamples!=0 */
		if (!ti->nsamples) {
			dev_err(dev, "there aren't samples to acquire");
			return -EINVAL;
		}
		/*
		 * The software trigger will be fired to force
		 * acquisition, so we don't care about current
		 * acquisition or other problems:
		 */
		break;
	}

	return zfa_common_conf_set(fa, zattr->id, tmp_val);
}
/* get the value of a FMC-ADC trigger register */
static int zfat_info_get(struct device *dev, struct zio_attribute *zattr,
			 uint32_t *usr_val)
{
	struct fa_dev *fa = get_zfadc(dev);

	zfa_common_info_get(fa, zattr->id, usr_val);

	return 0;
}
static const struct zio_sysfs_operations zfat_s_op = {
	.conf_set = zfat_conf_set,
	.info_get = zfat_info_get,
};


/* create an instance of the FMC-ADC trigger */
static struct zio_ti *zfat_create(struct zio_trigger_type *trig,
				 struct zio_cset *cset,
				 struct zio_control *ctrl, fmode_t flags)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	struct zfat_instance *zfat;

	if (!fa) {
		dev_err(&cset->head.dev, "no spec device defined\n");
		return ERR_PTR(-ENODEV);
	}

	zfat = kzalloc(sizeof(struct zfat_instance), GFP_KERNEL);
	if (!zfat)
		return ERR_PTR(-ENOMEM);

	/* Trigger registers */
	/* Set to single shot mode by default */

	/* Disable Software trigger*/
	zfa_common_conf_set(fa, ZFAT_CFG_SW_EN, 0);
	/* Enable Hardware trigger*/
	zfa_common_conf_set(fa, ZFAT_CFG_HW_EN, 1);
	/* Select external trigger (index 0) */
	zfa_common_conf_set(fa, ZFAT_CFG_HW_SEL, 1);
	cset->ti->zattr_set.ext_zattr[0].value = 1;

	zfat->fa = fa;
	zfat->ti.cset = cset;

	return &zfat->ti;
}

static void zfat_destroy(struct zio_ti *ti)
{
	struct fa_dev *fa = ti->cset->zdev->priv_d;
	struct zfat_instance *zfat = to_zfat_instance(ti);

	/* Disable Hardware trigger */
	zfa_common_conf_set(fa, ZFAT_CFG_HW_EN, 0);
	/* Enable Software trigger */
	zfa_common_conf_set(fa, ZFAT_CFG_SW_EN, 1);
	/* Disable all interrupt */
	zfa_common_conf_set(fa, ZFA_IRQ_MASK, ZFAT_NONE);

	kfree(zfat);
}

/*
 * Enable or disable the hardware trigger. The hardware trigger is the prefered
 * trigger so it correspond to the ZIO enable of the trigger.Status is active
 * low on ZIO but active high on the FMC-ADC, then use '!' on status
 */
static void zfat_change_status(struct zio_ti *ti, unsigned int status)
{
	struct fa_dev *fa = ti->cset->zdev->priv_d;

	zfa_common_conf_set(fa, ZFAT_CFG_HW_EN, !status);
}

/*
 * Transfer is over for the active block, so store the interleaved block
 * and start the next dma transfer for the next block (if any)
 */
static void zfat_data_done(struct zio_cset *cset)
{
	struct zio_block *block = cset->interleave->active_block;
	struct zio_bi *bi = cset->interleave->bi;

	if (!block)
		return;

	/* Store block in the ZIO buffer */
	if (bi->b_op->store_block(bi, block)) { /* may fail, no prob */
		bi->b_op->free_block(bi, block);
	}
	/* Clear active block */
	cset->interleave->active_block = NULL;
	/* Start next block DMA transfer */
	zfat_start_next_dma(cset->ti);
}

/*
 * Abort acquisition empty the list of prepared buffer. If a DMA
 * transfer is active, it will complete only the block that it takes
 */
static void zfat_abort(struct zio_cset *cset)
{
	struct zio_bi *bi = cset->interleave->bi;
	struct zfat_instance *zfat = to_zfat_instance(cset->ti);
	struct zfat_block *zfat_block, *node;
	unsigned long flags;

	/*
	 * Disable all interrupt. We are aborting acquisition, we don't need
	 * any interrupt
	 */
	dev_dbg(zfat->fa->fmc->hwdev, "Disable interrupts\n");
	zfa_common_conf_set(zfat->fa, ZFA_IRQ_MASK, ZFAT_NONE);
	spin_lock_irqsave(&zfat->lock, flags);
	list_for_each_entry_safe(zfat_block, node, &zfat->list_block, list) {
		bi->b_op->free_block(bi, zfat_block->block);
		list_del(&zfat_block->list);
		kfree(zfat_block);
	}
	spin_unlock_irqrestore(&zfat->lock, flags);
	zfat->fa->cur_dev_mem = 0;
	zfat->fa->lst_dev_mem = 0;
}

static const struct zio_trigger_operations zfat_ops = {
	.create =		zfat_create,
	.destroy =		zfat_destroy,
	.change_status =	zfat_change_status,
	.data_done =		zfat_data_done,
	.arm =			zfat_start_next_dma,
	.abort =		zfat_abort,
};

/* Definition of the trigger type */
struct zio_trigger_type zfat_type = {
	.owner = THIS_MODULE,
	.zattr_set = {
		.std_zattr = zfat_std_zattr,
		.ext_zattr = zfat_ext_zattr,
		.n_ext_attr = ARRAY_SIZE(zfat_ext_zattr),
	},
	.s_op = &zfat_s_op,
	.t_op = &zfat_ops,
};

