// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright CERN 2012-2019
 * Author: Federico Vaga
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#include "fmc-adc-100m14b4cha.h"

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
	ZIO_ATTR(trig, ZIO_ATTR_TRIG_POST_SAMP, ZIO_RW_PERM, ZFAT_POST, 1),
};
static struct zio_attribute zfat_ext_zattr[] = {
	[FA100M14B4C_TATTR_STA] = ZIO_ATTR_EXT("source-triggered", ZIO_RW_PERM,
					       ZFAT_CFG_STA, 0),
	[FA100M14B4C_TATTR_SRC] = ZIO_ATTR_EXT("source", ZIO_RW_PERM,
					       ZFAT_CFG_SRC,
					       FA100M14B4C_TRG_SRC_SW),
	[FA100M14B4C_TATTR_POL] = ZIO_ATTR_EXT("polarity", ZIO_RW_PERM,
					       ZFAT_CFG_POL, 0),

	/* Internal trigger threshold value is 2 complement format */
	[FA100M14B4C_TATTR_CH1_THRES] = ZIO_ATTR_EXT("ch0-threshold",
						     ZIO_RW_PERM,
						     ZFA_CH1_THRES, 0),
	[FA100M14B4C_TATTR_CH2_THRES] = ZIO_ATTR_EXT("ch1-threshold",
						     ZIO_RW_PERM,
						     ZFA_CH2_THRES, 0),
	[FA100M14B4C_TATTR_CH3_THRES] = ZIO_ATTR_EXT("ch2-threshold",
						     ZIO_RW_PERM,
						     ZFA_CH3_THRES, 0),
	[FA100M14B4C_TATTR_CH4_THRES] = ZIO_ATTR_EXT("ch3-threshold",
						     ZIO_RW_PERM,
						     ZFA_CH4_THRES, 0),

	[FA100M14B4C_TATTR_CH1_HYST] = ZIO_ATTR_EXT("ch0-hysteresis",
						    ZIO_RW_PERM,
						    ZFA_CH1_HYST, 0),
	[FA100M14B4C_TATTR_CH2_HYST] = ZIO_ATTR_EXT("ch1-hysteresis",
						    ZIO_RW_PERM,
						    ZFA_CH2_HYST, 0),
	[FA100M14B4C_TATTR_CH3_HYST] = ZIO_ATTR_EXT("ch2-hysteresis",
						    ZIO_RW_PERM,
						    ZFA_CH3_HYST, 0),
	[FA100M14B4C_TATTR_CH4_HYST] = ZIO_ATTR_EXT("ch3-hysteresis",
						    ZIO_RW_PERM,
						    ZFA_CH4_HYST, 0),

	[FA100M14B4C_TATTR_CH1_DLY] = ZIO_ATTR_EXT("ch0-delay",
						    ZIO_RW_PERM,
						    ZFA_CH1_DLY, 0),
	[FA100M14B4C_TATTR_CH2_DLY] = ZIO_ATTR_EXT("ch1-delay",
						    ZIO_RW_PERM,
						    ZFA_CH2_DLY, 0),
	[FA100M14B4C_TATTR_CH3_DLY] = ZIO_ATTR_EXT("ch2-delay",
						    ZIO_RW_PERM,
						    ZFA_CH3_DLY, 0),
	[FA100M14B4C_TATTR_CH4_DLY] = ZIO_ATTR_EXT("ch3-delay",
						    ZIO_RW_PERM,
						    ZFA_CH4_DLY, 0),
	/* Time Trigger */
	[FA100M14B4C_TATTR_TRG_TIM_SU] = ZIO_ATTR_EXT("trg-time-su",
						      ZIO_RW_PERM,
						      ZFA_UTC_TRIG_TIME_SECONDS_U,
						      0),
	[FA100M14B4C_TATTR_TRG_TIM_SL] = ZIO_ATTR_EXT("trg-time-sl",
						      ZIO_RW_PERM,
						      ZFA_UTC_TRIG_TIME_SECONDS_L,
						      0),
	[FA100M14B4C_TATTR_TRG_TIM_C] = ZIO_ATTR_EXT("trg-time-t",
						     ZIO_RW_PERM,
						     ZFA_UTC_TRIG_TIME_COARSE,
						     0),

	/*
	 * Delay to apply on the trigger in sampling clock period. The default
	 * clock frequency is 100MHz (period = 10ns)
	 */
	[FA100M14B4C_TATTR_EXT_DLY] = ZIO_ATTR_EXT("ext-delay", ZIO_RW_PERM,
							ZFAT_EXT_DLY, 0),

	/* Software Trigger */
	[FA100M14B4C_TATTR_SW_FIRE] = ZIO_PARAM_EXT("sw-trg-fire", ZIO_WO_PERM,
							ZFAT_SW, 0),

	/* last trigger time stamp */
	[FA100M14B4C_TATTR_TRG_SU] = ZIO_PARAM_EXT("tstamp-trg-lst-su",
						   ZIO_RO_PERM,
						   ZFA_UTC_TRIG_SECONDS_U, 0),
	[FA100M14B4C_TATTR_TRG_SL] = ZIO_PARAM_EXT("tstamp-trg-lst-sl",
						   ZIO_RO_PERM,
						   ZFA_UTC_TRIG_SECONDS_L, 0),
	[FA100M14B4C_TATTR_TRG_C] = ZIO_PARAM_EXT("tstamp-trg-lst-t",
						  ZIO_RO_PERM,
						  ZFA_UTC_TRIG_COARSE, 0),
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

	switch (zattr->id) {
	case ZFAT_SHOTS_NB:
		if (!tmp_val) {
			dev_err(fa->msgdev, "nshots cannot be 0\n");
			return -EINVAL;
		}
		break;
	case ZFAT_POST:
		if (tmp_val < 2) {
			dev_err(fa->msgdev, "minimum post samples 2 (HW limitation)\n");
			return -EINVAL;
		}
		tmp_val--;  /* Remove one sample for the trigger */
		break;
	case ZFAT_SW:
		/* Fire if software trigger is enabled (index 5) */
		if (!(ti->zattr_set.ext_zattr[FA100M14B4C_TATTR_SRC].value &
		      FA100M14B4C_TRG_SRC_SW)) {
			dev_info(fa->msgdev, "sw trigger is not enabled\n");
			return -EPERM;
		}
		/* Fire if nsamples!=0 */
		if (!ti->nsamples) {
			dev_info(fa->msgdev, "pre + post = 0: cannot acquire\n");
			return -EINVAL;
		}
		/*
		 * The software trigger will be fired to force
		 * acquisition, so we don't care about current
		 * acquisition or other problems:
		 */
		break;
	case ZFAT_CFG_SRC:
		/*
		 * Do not copy to hardware when globally disabled
		 * We tell ZIO to save the value locally and will do
		 * it when the user starts an acquisition
		 *
		 * We cannot save the value in cache only when disabled
		 * because the trigger is always disabled during configuration
		 */
		return 0;
	}

	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[zattr->id], tmp_val);
	return 0;
}


/* zfat_info_get
 *
 * get the value of a FMC-ADC trigger register
 */
static int zfat_info_get(struct device *dev, struct zio_attribute *zattr,
			 uint32_t *usr_val)
{
	struct fa_dev *fa = get_zfadc(dev);

	switch (zattr->id) {
	case ZFAT_CFG_SRC:
		/*
		 * The good value for the trigger source is always in
		 * the ZIO cache.
		 */
		return 0;
	}

	*usr_val = fa_readl(fa, fa->fa_adc_csr_base, &zfad_regs[zattr->id]);
	switch (zattr->id) {
	case ZFAT_POST:
		(*usr_val)++; /* add the trigger sample */
		break;
	}

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
		/* This only happens if we have a bug in the init sequence */
		dev_err(fa->msgdev, "No FMC device associated\n");
		return ERR_PTR(-ENODEV);
	}

	zfat = kzalloc(sizeof(struct zfat_instance), GFP_KERNEL);
	if (!zfat)
		return ERR_PTR(-ENOMEM);

	zfat->fa = fa;
	zfat->ti.cset = cset;

	return &zfat->ti;
}

static void zfat_destroy(struct zio_ti *ti)
{
	struct fa_dev *fa = ti->cset->zdev->priv_d;
	struct zfat_instance *zfat = to_zfat_instance(ti);

	/* Disable all trigger sources */
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFAT_CFG_SRC], 0);

	/* Other triggers cannot use pre-samples */
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFAT_PRE], 0);
	/* Reset post samples */
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFAT_POST], 0);
	/* Other triggers can handle only 1 shot */
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFAT_SHOTS_NB], 1);

	kfree(zfat);
}

/*
 *
 * Enable or disable the trigger sources globally.
 * On disable (status > 0), we disable all the trigger sources
 * On enable (status == 0), we enable the trigger soruces specified in the
 * correspondent sysfs attribute
 */
static void zfat_change_status(struct zio_ti *ti, unsigned int status)
{
	struct fa_dev *fa = ti->cset->zdev->priv_d;
	uint32_t src = ti->zattr_set.ext_zattr[FA100M14B4C_TATTR_SRC].value;

	if (status)
		fa_writel(fa, fa->fa_adc_csr_base,
			  &zfad_regs[ZFAT_CFG_SRC], 0);
	else
		fa_writel(fa, fa->fa_adc_csr_base,
			  &zfad_regs[ZFAT_CFG_SRC], src);
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
static int zfat_data_done(struct zio_cset *cset)
{
	struct zfad_block *zfad_block = cset->interleave->priv_d;
	struct zio_bi *bi = cset->interleave->bi;
	struct fa_dev *fa = cset->zdev->priv_d;
	unsigned int i;

	dev_dbg(fa->msgdev, "Data done\n");

	/* Nothing to store */
	if (!zfad_block)
		return 0;

	/* Store blocks */
	for (i = 0; i < fa->n_shots; ++i)
		if (likely(i < fa->n_fires)) {/* Store filled blocks */
			dev_dbg(fa->msgdev, "Store Block %i/%i\n",
				i + 1, fa->n_shots);
			zio_buffer_store_block(bi, zfad_block[i].block);
		} else {	/* Free un-filled blocks */
			dev_dbg(fa->msgdev, "Free un-acquired block %d/%d "
					"(received %d shots)\n",
					i + 1, fa->n_shots, fa->n_fires);
			zio_buffer_free_block(bi, zfad_block[i].block);
		}
	/* Clear active block */
	fa->n_shots = 0;
	fa->n_fires = 0;
	kfree(zfad_block);
	cset->interleave->priv_d = NULL;

	return 0;
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
	struct fa_dev *fa = ti->cset->zdev->priv_d;
	struct zio_block *block;
	struct zfad_block *zfad_block;
	unsigned int size;
	uint32_t dev_mem_off;
	int i, err = 0;

	dev_dbg(fa->msgdev, "Arming trigger\n");

	/* Update the current control: sequence, nsamples and tstamp */
	interleave->current_ctrl->nsamples = ti->nsamples;

	/* Allocate the necessary blocks for multi-shot acquisition */
	fa->n_shots = ti->zattr_set.std_zattr[ZIO_ATTR_TRIG_N_SHOTS].value;
	dev_dbg(fa->msgdev, "programmed shot %i\n", fa->n_shots);

	if (!fa->n_shots) {
		dev_info(fa->msgdev, "Cannot arm. No programmed shots\n");
		return -EINVAL;
	}

	/*
	 * Allocate a new block for DMA transfer. Sometimes we are in an
	 * atomic context and we cannot use in_atomic()
	 */
	zfad_block = kmalloc(sizeof(struct zfad_block) * fa->n_shots,
			     GFP_ATOMIC);
	if (!zfad_block)
		return -ENOMEM;

	interleave->priv_d = zfad_block;

	/*
	 * Calculate the required size to store all channels.  This is
	 * an interleaved acquisition, so nsamples represents the
	 * number of sample on all channels (n_chan * chan_samples)
	 * Trig time stamp are appended after the post samples
	 * (4*32bits word) size should be 32bits word aligned
	 * ti->nsamples is the sum of (pre-samp+ post-samp)*4chan
	 * because it's the interleave channel.
	 */
	size = (interleave->current_ctrl->ssize * ti->nsamples)
		+ FA_TRIG_TIMETAG_BYTES;
	/* check if size is 32 bits word aligned: should be always the case */
	if (size % 4) {
		/* should never happen: increase the size accordling */
		dev_warn(fa->msgdev,
			"\nzio data block size should 32bit word aligned."
			"original size:%d was increased by %d bytes\n",
			size, size%4);
		size += size % 4;
	}
	dev_mem_off = 0;
	/* Allocate ZIO blocks */
	for (i = 0; i < fa->n_shots; ++i) {
		dev_dbg(fa->msgdev, "Allocating block %d ...\n", i);
		block = zio_buffer_alloc_block(interleave->bi, size,
					       GFP_ATOMIC);
		if (!block) {
			dev_err(fa->msgdev,
				"\narm trigger fail, cannot allocate block\n");
			err = -ENOMEM;
			goto out_allocate;
		}
		/* Copy the updated control into the block */
		memcpy(zio_get_ctrl(block), interleave->current_ctrl,
		       zio_control_size(interleave));
		/* Add to the vector of prepared blocks */
		zfad_block[i].block = block;
		zfad_block[i].cset = ti->cset;
	}

	err = ti->cset->raw_io(ti->cset);
	if (err != -EAGAIN && err != 0)
		goto out_allocate;

	return err;

out_allocate:
	while ((--i) >= 0)
		zio_buffer_free_block(interleave->bi, zfad_block[i].block);
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

	dev_dbg(fa->msgdev, "Aborting trigger\n");

	/* Nothing to free */
	if (!zfad_block)
		return;

	/* Free all blocks */
	for (i = 0; i < fa->n_shots; ++i)
		zio_buffer_free_block(bi, zfad_block[i].block);
	kfree(zfad_block);
	cset->interleave->priv_d = NULL;
}

/* Output it is not supported by this trigger */
static int zfat_push(struct zio_ti *ti, struct zio_channel *chan,
		     struct zio_block *block)
{
	struct zio_cset *cset = ti->cset;
	struct fa_dev *fa = cset->zdev->priv_d;

	dev_err(fa->msgdev, "trigger \"%s\" does not support output\n",
		ti->head.name);
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
