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

#include <linux/fmc.h>

#include <linux/zio.h>
#include <linux/zio-sysfs.h>
#include <linux/zio-buffer.h>
#include <linux/zio-trigger.h>

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
	ZIO_PARAM_EXT("sw-trg-fire", ZIO_WO_PERM, ZFAT_SW, 0),

	/* last trigger time stamp */
	ZIO_PARAM_EXT("tstamp-trg-lst-s", ZIO_RO_PERM, ZFA_UTC_TRIG_SECONDS, 0),
	ZIO_PARAM_EXT("tstamp-trg-lst-t", ZIO_RO_PERM, ZFA_UTC_TRIG_COARSE, 0),
	ZIO_PARAM_EXT("tstamp-trg-lst-b", ZIO_RO_PERM, ZFA_UTC_TRIG_FINE, 0),
};


/*
 * zfat_conf_set
 *
 * set a value to a FMC-ADC trigger register
 */
static int zfat_conf_set(struct device *dev, struct zio_attribute *zattr,
			 uint32_t usr_val)
{
	struct fa_dev *fa = get_zfadc(dev);
	struct zio_ti *ti = to_zio_ti(dev);
	uint32_t tmp_val = usr_val;
	int err = 0;

	switch (zattr->id) {
	case ZFAT_SHOTS_NB:
		if (!tmp_val) {
			dev_err(dev, "nshots cannot be 0\n");
			return -EINVAL;
		}
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

	return zfa_hardware_write(fa, zattr->id, tmp_val);
}


/* zfat_info_get
 *
 * get the value of a FMC-ADC trigger register
 */
static int zfat_info_get(struct device *dev, struct zio_attribute *zattr,
			 uint32_t *usr_val)
{
	struct fa_dev *fa = get_zfadc(dev);

	zfa_hardware_read(fa, zattr->id, usr_val);

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

	/* Disable Software trigger*/
	zfa_hardware_write(fa, ZFAT_CFG_SW_EN, 0);
	/* Enable Hardware trigger*/
	zfa_hardware_write(fa, ZFAT_CFG_HW_EN, 1);

	zfat->fa = fa;
	zfat->ti.cset = cset;

	return &zfat->ti;
}

static void zfat_destroy(struct zio_ti *ti)
{
	struct fa_dev *fa = ti->cset->zdev->priv_d;
	struct zfat_instance *zfat = to_zfat_instance(ti);

	/* Enable Software trigger */
	zfa_hardware_write(fa, ZFAT_CFG_SW_EN, 1);
	/* Disable Hardware trigger */
	zfa_hardware_write(fa, ZFAT_CFG_HW_EN, 0);
	/* Other triggers cannot use pre-samples */
	zfa_hardware_write(fa, ZFAT_PRE, 0);
	/* Reset post samples */
	zfa_hardware_write(fa, ZFAT_POST, 0);
	/* Other triggers can handle only 1 shot */
	zfa_hardware_write(fa, ZFAT_SHOTS_NB, 1);

	kfree(zfat);
}

/*
 *
 * Enable or disable the hardware trigger. The hardware trigger is the prefered
 * trigger so it correspond to the ZIO enable of the trigger.Status is active
 * low on ZIO but active high on the FMC-ADC, then use '!' on status
 */
static void zfat_change_status(struct zio_ti *ti, unsigned int status)
{
	struct fa_dev *fa = ti->cset->zdev->priv_d;

	zfa_hardware_write(fa, ZFAT_CFG_HW_EN, !status);
}

/*
 * zfat_data_done
 * @cset: channels set
 *
 * Transfer is over for all blocks. Store all blocks and free
 * zfad_blocks vector. Here we are storing only blocks
 * that were filled by a trigger fire. If for some reason the data_done
 * occurs before the natural end of the acquisition, un-filled block
 * are not stored.
 */
static void zfat_data_done(struct zio_cset *cset)
{
	struct zfad_block *zfad_block = cset->interleave->priv_d;
	struct zio_bi *bi = cset->interleave->bi;
	struct fa_dev *fa = cset->zdev->priv_d;
	unsigned int i;

	dev_dbg(&cset->head.dev, "Data done\n");

	/* Nothing to store */
	if (!zfad_block)
		return;

	/* Store blocks */
	for(i = 0; i < fa->n_shots; ++i)
		if (likely(i < fa->n_fires)) {/* Store filled blocks */
			dev_dbg(&cset->head.dev, "Store Block %i/%i\n",
				i + 1, fa->n_shots);
			bi->b_op->store_block(bi, zfad_block[i].block);
		} else {	/* Free un-filled blocks */
			dev_dbg(&cset->head.dev, "Free un-acquired block %d/%d "
					"(received %d shots)\n",
					i + 1, fa->n_shots, fa->n_fires);
			bi->b_op->free_block(bi, zfad_block[i].block);
		}
	/* Clear active block */
	fa->n_shots = 0;
	fa->n_fires = 0;
	kfree(zfad_block);
	cset->interleave->priv_d = NULL;

	return;
}

/*
 * zfat_arm_trigger
 * @ti: trigger instance
 *
 * The ADC in multi-shot mode need to allocate a block for each programmed shot.
 * We need a custom arm_trigger to allocate N blocks at times.
 */
static int zfat_arm_trigger(struct zio_ti *ti)
{
	struct zio_channel *interleave = ti->cset->interleave;
	struct zio_buffer_type *zbuf = ti->cset->zbuf;
	struct fa_dev *fa = ti->cset->zdev->priv_d;
	struct zio_block *block;
	struct zfad_block *zfad_block;
	unsigned int size;
	uint32_t dev_mem_ptr;
	int i, err = 0;

	dev_dbg(&ti->head.dev, "Arming trigger\n");
	/* Update the current control: sequence, nsamples and tstamp */
	interleave->current_ctrl->nsamples = ti->nsamples;

	/* Allocate the necessary blocks for multi-shot acquisition */
	fa->n_shots = ti->zattr_set.std_zattr[ZIO_ATTR_TRIG_N_SHOTS].value;
	dev_dbg(&ti->head.dev, "programmed shot %i\n", fa->n_shots);

	if (!fa->n_shots) {
		dev_err(&ti->head.dev, "Cannot arm. No programmed shots\n");
		return -EINVAL;
	}

	/* Allocate a new block for DMA transfer */
	zfad_block = kmalloc(sizeof(struct zfad_block) * fa->n_shots,
				GFP_KERNEL);
	if (!zfad_block)
		return -ENOMEM;

	interleave->priv_d = zfad_block;

	/*
	 * Calculate the required size to store all channels.
	 * This is an interleaved acquisition, so nsamples represents the
	 * number of sample on all channels (n_chan * chan_samples)
	 */
	size = interleave->current_ctrl->ssize * ti->nsamples;

	dev_mem_ptr = 0;
	/* Allocate ZIO blocks */
	for (i = 0; i < fa->n_shots; ++i) {
		dev_dbg(&ti->cset->head.dev, "Allocating block %d ...\n", i);
		block = zbuf->b_op->alloc_block(interleave->bi, size,
						GFP_KERNEL);
		if (!block) {
			dev_err(&ti->cset->head.dev,
				"arm trigger fail, cannot allocate block\n");
			err = -ENOMEM;
			goto out_allocate;
		}
		/* Copy the updated control into the block */
		memcpy(zio_get_ctrl(block), interleave->current_ctrl,
		       zio_control_size(interleave));
		/* Add to the vector of prepared blocks */
		zfad_block[i].block = block;
		zfad_block[i].dev_mem_ptr = dev_mem_ptr;
		dev_mem_ptr += size;
		dev_dbg(&ti->cset->head.dev, "next dev_mem_ptr 0x%x (+%d)",
			dev_mem_ptr, size);
	}

	err = ti->cset->raw_io(ti->cset);
	if (err != -EAGAIN && err != 0)
		goto out_allocate;

	return err;

out_allocate:
	while((--i) >= 0)
		zbuf->b_op->free_block(interleave->bi, zfad_block[i].block);
	kfree(zfad_block);
	interleave->priv_d = NULL;
	return err;
}


/*
 * zfat_abort
 * @cset: channel set to abort
 *
 * Abort acquisition empty the list of prepared buffer.
 */
static void zfat_abort(struct zio_ti *ti)
{
	struct zio_cset *cset = ti->cset;
	struct fa_dev *fa = cset->zdev->priv_d;
	struct zio_bi *bi = cset->interleave->bi;
	struct zfad_block *zfad_block = cset->interleave->priv_d;
	unsigned int i;

	dev_dbg(&ti->head.dev, "Aborting trigger");
	/* Free all blocks */
	for(i = 0; i < fa->n_shots; ++i)
		bi->b_op->free_block(bi, zfad_block[i].block);
	kfree(zfad_block);
	cset->interleave->priv_d = NULL;
}

/* Output it is not supported by this trigger */
static int zfat_push(struct zio_ti *ti, struct zio_channel *chan,
		     struct zio_block *block)
{
	dev_err(&ti->head.dev, "trigger \"%s\" does not support output",
		ti->head.name);
	BUG();

	return -EIO;
}

static const struct zio_trigger_operations zfat_ops = {
	.create =		zfat_create,
	.destroy =		zfat_destroy,
	.change_status =	zfat_change_status,
	.data_done =		zfat_data_done,
	.arm =			zfat_arm_trigger,
	.abort =		zfat_abort,
	.push_block =		zfat_push,
};

/* Definition of the trigger type -- can't be static */
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

int fa_trig_init(void)
{
	int err;

	err = zio_register_trig(&zfat_type, "adc-100m14b");
	if (err)
		pr_err("%s: Cannot register ZIO trigger type"
		       " \"adc-100m14b\" (error %i)\n", KBUILD_MODNAME, err);
	return err;
}

void fa_trig_exit(void)
{
	zio_unregister_trig(&zfat_type);
}
