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

#include "spec.h"
#include "fmc-adc.h"

struct zfat_instance {
	struct zio_ti ti;
	struct fa_dev *fa;
	unsigned int n_acq_dev;	/* number of acquisitions on device memory */
	unsigned int n_err;	/* number of errors */

	spinlock_t lock;
	struct list_head list_block; /* list of blocks during multi-shot*/
};
struct zfat_block {
	struct zio_block *block;
	unsigned int dev_addr;
	struct list_head list;
};

#define to_zfat_instance(_ti) container_of(_ti, struct zfat_instance, ti)

/* zio trigger attributes */
static DEFINE_ZATTR_STD(TRIG, zfat_std_zattr) = {
	/* Number of shots */
	ZATTR_REG(trig, ZATTR_TRIG_REENABLE, S_IRUGO | S_IWUGO, ZFAT_SHOTS_NB, 0),
	ZATTR_REG(trig, ZATTR_TRIG_PRE_SAMP, S_IRUGO | S_IWUGO, ZFAT_PRE, 0),
	ZATTR_REG(trig, ZATTR_TRIG_POST_SAMP, S_IRUGO | S_IWUGO, ZFAT_POST, 0),
};
static struct zio_attribute zfat_ext_zattr[] = {
	/* Config register */
	/* Hardware trigger selction
	 * 0: internal (data threshold)
	 * 1: external (front panel trigger input)
	 */
	ZATTR_EXT_REG("external", S_IRUGO | S_IWUGO, ZFAT_CFG_HW_SEL, 0),
	/*
	 * Internal Hardware trigger polarity
	 * 0: positive edge/slope
	 * 1: negative edge/slope
	 */
	ZATTR_EXT_REG("polarity", S_IRUGO | S_IWUGO, ZFAT_CFG_HW_POL, 0),
	/*
	 * Channel selection for internal trigger
	 * 0: channel 1, 1: channel 2, 2: channel 3, 3: channel 4
	 */
	ZATTR_EXT_REG("int-channel", S_IRUGO | S_IWUGO, ZFAT_CFG_INT_SEL, 0),
	/* Internal trigger threshold value is 2 complement format */
	ZATTR_EXT_REG("int-threshold", S_IRUGO | S_IWUGO, ZFAT_CFG_THRES, 0),
	/*
	 * Delay to apply on the trigger in sampling clock period. The default
	 * clock frequency is 100MHz (period = 10ns)
	 */
	ZATTR_EXT_REG("delay", S_IRUGO | S_IWUGO, ZFAT_DLY, 0),

	/* Software Trigger */
	/* Enable (1) or disable (0) software trigger */
	PARAM_EXT_REG("sw-trg-enable", S_IRUGO | S_IWUGO, ZFAT_CFG_SW_EN, 0),
	PARAM_EXT_REG("sw-trg-fire", S_IWUGO, ZFAT_SW, 0),

	/* last trigger time stamp */
	PARAM_EXT_REG("tstamp-trg-lst-s", S_IRUGO, ZFA_UTC_TRIG_SECONDS, 0),
	PARAM_EXT_REG("tstamp-trg-lst-t", S_IRUGO, ZFA_UTC_TRIG_COARSE, 0),
	PARAM_EXT_REG("tstamp-trg-lst-b", S_IRUGO, ZFA_UTC_TRIG_FINE, 0),
};

static int zfat_overflow_detection(struct zio_ti *ti, unsigned int addr,
				   uint32_t val)
{
	struct zio_attribute *ti_zattr = ti->zattr_set.std_zattr;
	uint32_t pre_t, post_t, nshot_t;

	if (!addr)
		return 0;

	pre_t = addr == ZFAT_PRE ? val : ti_zattr[ZATTR_TRIG_PRE_SAMP].value;
	post_t = addr == ZFAT_POST ? val : ti_zattr[ZATTR_TRIG_POST_SAMP].value;
	nshot_t = addr == ZFAT_SHOTS_NB ? val : ti_zattr[ZATTR_TRIG_REENABLE].value + 1;

	if (((pre_t + post_t) * ti->cset->ssize * nshot_t) >= FA_MAX_ACQ_BYTE) {
		dev_err(&ti->head.dev, "cannot acquire, device memory overflow\n");
		return -ENOMEM;
	}
	return 0;
}
/* set a value to a FMC-ADC trigger register */
static int zfat_conf_set(struct device *dev, struct zio_attribute *zattr,
			 uint32_t usr_val)
{
	const struct zio_reg_desc *reg = &zfad_regs[zattr->priv.addr];
	struct fa_dev *fa = get_zfadc(dev);
	struct zio_ti *ti = to_zio_ti(dev);
	uint32_t tmp_val = usr_val;
	int err = 0;

	switch (zattr->priv.addr) {
	case ZFAT_SHOTS_NB:
		/*
		 * Increase the reenable by 1 to be choerent with the
		 * re-enable meaning. On the ADC there is NSHOTS which
		 * is the number of shots to do, so to do 1 shot this
		 * register must be set to 1. ZIO use re-enable to do
		 * multiple shots, so 1 mean one more shot after the
		 * first one.
		 */
		++tmp_val;
	case ZFAT_PRE:
	case ZFAT_POST:
		err = zfat_overflow_detection(ti, zattr->priv.addr, tmp_val);
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
		if (!ti->cset->interleave->current_ctrl->nsamples) {
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

	return zfa_common_conf_set(fa, reg, tmp_val);
}
/* get the value of a FMC-ADC trigger register */
static int zfat_info_get(struct device *dev, struct zio_attribute *zattr,
			 uint32_t *usr_val)
{
	struct fa_dev *fa = get_zfadc(dev);

	zfa_common_info_get(fa, &zfad_regs[zattr->priv.addr], usr_val);
	if (zattr->priv.addr == ZFAT_SHOTS_NB)
		(*usr_val)--;
	return 0;
}
static const struct zio_sysfs_operations zfat_s_op = {
	.conf_set = zfat_conf_set,
	.info_get = zfat_info_get,
};


/*
 * .... In this second step we know that all data from all triggers is ready.
 * So we get the first block from the list and start the acquisition on it.
 * When the DMA will end, the interrupt handler will invoke again this function
 * until the list is empty
 */
static void zfat_start_next_dma(struct zio_ti *ti)
{
	struct zfat_instance *zfat = to_zfat_instance(ti);
	struct fa_dev *fa = ti->cset->zdev->priv_d;
	struct zio_channel *interleave = ti->cset->interleave;
	struct zfat_block *zfat_block;
	struct zio_buffer_type *zbuf = ti->cset->zbuf;
	unsigned long flags;
	int err;

	if (list_empty(&zfat->list_block)) {
		dev_dbg(&ti->head.dev, "All DMA transfers finish\n");
		/* ADC will restart fill data from 0 */
		fa->lst_dev_mem = 0;
		/*
		 * All DMA transfers finish, so we can safely re-enable
		 * triggers. Hardware trigger depends on the enable status
		 * of the trigger. Software trigger depends on the previous
		 * status taken form zio attributes (index 5 of extended one)
		 */
		zfa_common_conf_set(fa, &zfad_regs[ZFAT_CFG_HW_EN],
				    (ti->flags & ZIO_STATUS ? 0 : 1));
		zfa_common_conf_set(fa, &zfad_regs[ZFAT_CFG_SW_EN],
				    ti->zattr_set.ext_zattr[5].value);
		/* Automatic start next acquisition */
		if (enable_auto_start) {
			dev_dbg(&ti->head.dev, "Automatic start\n");
			zfad_fsm_command(fa, ZFA_START);
			zfa_common_conf_set(fa, &zfad_regs[ZFA_CTL_FMS_CMD],
					    ZFA_START);
		}
		return;
	}

	/* Get the first block */
	spin_lock_irqsave(&zfat->lock, flags);
	zfat_block = list_first_entry(&zfat->list_block, struct zfat_block, list);
	list_del(&zfat_block->list);
	spin_unlock_irqrestore(&zfat->lock, flags);

	/* Acquire data from device */
	fa->cur_dev_mem = zfat_block->dev_addr;
	interleave->active_block = zfat_block->block;
	err = ti->cset->raw_io(ti->cset);
	if (err != -EAGAIN) {
		dev_err(&ti->cset->head.dev,
			"Cannot transfer block (err %i)\n", err);
		zbuf->b_op->free_block(interleave->bi, zfat_block->block);
	}
	/* Clear memory */
	kfree(zfat_block);
}


/* IRQ functions handler */

/* Get irq and clear the register. To clear an interrupt we have to write 1
 * on the handled interrupt. We handle all interrupt so we clear all interrupts
 */
static void zfat_get_irq_status(struct zfat_instance *zfat,
				uint32_t *irq_status, uint32_t *irq_multi)
{
	struct fa_dev *fa = zfat->ti.cset->zdev->priv_d;

	/* Get current interrupts status */
	zfa_common_info_get(fa, &zfad_regs[ZFA_IRQ_SRC], irq_status);
	zfa_common_info_get(fa, &zfad_regs[ZFA_IRQ_MULTI], irq_multi);
	dev_dbg(&zfat->ti.head.dev, "irq status = 0x%x multi = 0x%x\n",
			*irq_status, *irq_multi);
	/* Clear current interrupts status */
	zfa_common_conf_set(fa, &zfad_regs[ZFA_IRQ_SRC], *irq_status);
	zfa_common_conf_set(fa, &zfad_regs[ZFA_IRQ_MULTI], *irq_multi);
	/* ack the irq */
	zfat->fa->fmc->op->irq_ack(zfat->fa->fmc);
}

/*
 * The DMA is finish due to an error or not. We must handle both the condition.
 * If DMA aborted by an error we abort the acquisition, so we lost all the data.
 * If the DMA complete successfully, we can call zio_trigger_data_done which
 * will complete the transfer
 */
static void zfat_irq_dma_done(struct fmc_device *fmc,
			      struct zfat_instance *zfat, int status)
{
	struct fa_dev *fa = zfat->ti.cset->zdev->priv_d;
	uint32_t val;

	/* unmap dma */
	zfad_unmap_dma(zfat->ti.cset);
	if (status & ZFAT_DMA_DONE) { /* DMA done*/
		dev_dbg(&zfat->ti.head.dev, "DMA transfer done\n");
		zio_trigger_data_done(zfat->ti.cset);
		zfat->n_acq_dev--;
	} else {	/* DMA error */
		zfa_common_info_get(fa, &zfad_regs[ZFA_DMA_STA], &val);
		dev_err(fmc->hwdev,
			"DMA error (status 0x%x). All acquisition lost\n", val);
		zio_trigger_abort(zfat->ti.cset);
		zfat->n_err++;
	}
}
/* Get the last trigger time-stamp from device */
static void zfat_get_time_stamp(struct fa_dev *fa, struct zio_timestamp *ts)
{
	zfa_common_info_get(fa, &zfad_regs[ZFA_UTC_TRIG_SECONDS],
			    (uint32_t *)&ts->secs);
	zfa_common_info_get(fa, &zfad_regs[ZFA_UTC_TRIG_COARSE],
			    (uint32_t *)&ts->ticks);
	zfa_common_info_get(fa, &zfad_regs[ZFA_UTC_TRIG_FINE],
			    (uint32_t *)&ts->bins);
}
/*
 * Trigger fires, but ZIO allow only one trigger at time, and the ADC in
 * multi-shot mode don't complete an acquisition until all triggers fire; more
 * over, the ADC save only the last trigger timestamp. So, we need to handle
 * separately each trigger fire if we want to get all time-stamp. In this first
 * step we prepare all necessary blocks and set the time-stamp ....
 */
static void zfat_irq_trg_fire(struct zfat_instance *zfat)
{
	struct zio_ti *ti = &zfat->ti;
	struct zio_channel *interleave = ti->cset->interleave;
	struct zio_buffer_type *zbuf = ti->cset->zbuf;
	struct fa_dev *fa = ti->cset->zdev->priv_d;
	struct zio_control *ctrl;
	struct zio_block *block;
	struct zfat_block *zfat_block;
	unsigned long flags;
	unsigned int size;

	dev_dbg(&zfat->ti.head.dev, "Trigger fire\n");
	zfat->n_acq_dev++;
	/*
	 * Calculate the required size to store all channels.
	 * n_chan - 1 because of interleaved channel
	 */
	size = interleave->current_ctrl->ssize *
	       interleave->current_ctrl->nsamples * (ti->cset->n_chan - 1);
	/*
	 * Immediately update the pointer to the last block of data because if
	 * block allocation fail, we must ready for the next block. If we don't
	 * Proceed in this way we will have time-stamp which doesn't correspond
	 * to the data block
	 */
	fa->lst_dev_mem += size; /* next data will be here */

	/* Create a list of prepared block */
	zfat_block = kmalloc(sizeof(struct zfat_block), GFP_ATOMIC);
	if (!zfat_block)
		return;
	zfat_block->dev_addr = fa->lst_dev_mem - size; /* previous block */

	/* Allocate and update control */
	ctrl = zio_alloc_control(GFP_ATOMIC);
	if (!ctrl)
		goto out;
	interleave->current_ctrl->seq_num++;
	zfat_get_time_stamp(fa, &interleave->current_ctrl->tstamp);
	memcpy(ctrl, interleave->current_ctrl, ZIO_CONTROL_SIZE);

	/* Allocate a new block for DMA transfer */
	block = zbuf->b_op->alloc_block(interleave->bi, ctrl, size,
					GFP_ATOMIC);
	if (IS_ERR(block)) {
		dev_err(&ti->cset->head.dev, "can't alloc block\n");
		goto out_block;
	}
	zfat_block->block = block;

	/* Add to the list of prepared blocks */
	spin_lock_irqsave(&zfat->lock, flags);
	list_add_tail(&zfat_block->list, &zfat->list_block);
	spin_unlock_irqrestore(&zfat->lock, flags);
	return;

out_block:
	zio_free_control(ctrl);
out:
	kfree(zfat_block);
}
/*
 * The ADC and the acquisition, so, if the state machine is idle, we can
 * retrieve data from the ADC DDR memory. To start the transfer we fire the zio
 * trigger. We can't use zio_fire_trigger when the real trigger fire due to
 * multi-shot capabilities of the ADC
 * FIXME maybe here reset cur|lst_dev_mem
 */
static void zfat_irq_acq_end(struct zfat_instance *zfat)
{
	struct fa_dev *fa = zfat->ti.cset->zdev->priv_d;
	uint32_t val;

	dev_dbg(zfat->fa->fmc->hwdev, "Acquisition done\n");
	/*
	 * All programmed triggers fire, so the acquisition is ended.
	 * If the state machine is _idle_ we can start the DMA transfer.
	 */
	zfa_common_info_get(fa, &zfad_regs[ZFA_STA_FSM],&val);
	if (val == ZFA_STATE_IDLE) {
		/*
		 * Disable all triggers to prevent fires between
		 * different DMA transfers required for multi-shots
		 */
		zfa_common_conf_set(fa, &zfad_regs[ZFAT_CFG_HW_EN], 0);
		zfa_common_conf_set(fa, &zfad_regs[ZFAT_CFG_SW_EN], 0);
		dev_dbg(&zfat->ti.head.dev, "Start DMA from device\n");
		zio_fire_trigger(&zfat->ti);
	} else {
		/* we can't DMA if the state machine is not idle */
		dev_warn(&zfat->ti.cset->head.dev,
			 "Can't start DMA on the last acquisition, "
			 "it will be done on the next acquisition end\n");
	}
}
/*
 * The different irq status are handled in different if statement. At the end
 * of the if statement we don't call return because it is possible that there
 * are others irq to handle. The order of irq handlers is based on the
 * possibility to have many irq rise at the same time. It is possible that
 * ZFAT_TRG_FIRE and ZFAT_ACQ_END happens simultaneously, so we handle FIRE
 * before END. If ZFAT_DMA_DONE happens with ZFAT_TRG_FIRE and ZFAT_ACQ_END, we
 * handle DMA_DONE before the other because we must conclude a previous
 * acquisition before start a new one
 */
static irqreturn_t zfadc_irq(int irq, void *ptr)
{
	struct fmc_device *fmc = ptr;
	struct fa_dev *fa = fmc_get_drvdata(fmc);
	struct zfat_instance *zfat = to_zfat_instance(fa->zdev->cset->ti);
	uint32_t status, multi;

	/* irq to handle */
	zfat_get_irq_status(zfat, &status, &multi);
	/*
	 * It cannot happen that DMA_DONE is in the multi register.
	 * It should not happen ...
	 */
	if (status & (ZFAT_DMA_DONE | ZFAT_DMA_ERR))
		zfat_irq_dma_done(fmc, zfat, status);

	/* Fire the trigger for each interrupt */
	if (status & ZFAT_TRG_FIRE)
		zfat_irq_trg_fire(zfat);
	if (multi & ZFAT_TRG_FIRE)
		zfat_irq_trg_fire(zfat);
	/*
	 * If it is too fast the ACQ_END interrupt can be in the
	 * multi register
	 */
	if ((status | multi) & ZFAT_ACQ_END)
		zfat_irq_acq_end(zfat);
	return IRQ_HANDLED;
}

struct fmc_gpio zfat_gpio_cfg[] = {
	{
		.gpio = FMC_GPIO_IRQ(0),
		.mode = GPIOF_DIR_IN,
		.irqmode = IRQF_TRIGGER_RISING,
	}
};

/* create an instance of the FMC-ADC trigger */
static struct zio_ti *zfat_create(struct zio_trigger_type *trig,
				 struct zio_cset *cset,
				 struct zio_control *ctrl, fmode_t flags)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	struct zfat_instance *zfat;
	int err;

	if (!fa) {
		dev_err(&cset->head.dev, "no spec device defined\n");
		return ERR_PTR(-ENODEV);
	}

	zfat = kzalloc(sizeof(struct zfat_instance), GFP_KERNEL);
	if (!zfat)
		return ERR_PTR(-ENOMEM);

	zfat->fa = fa;

	/* Configure GPIO for IRQ */
	fa->fmc->op->gpio_config(fa->fmc, zfat_gpio_cfg,
				 ARRAY_SIZE(zfat_gpio_cfg));
	/* Request IRQ */
	err = fa->fmc->op->irq_request(fa->fmc, zfadc_irq, "fmc-adc",
				       IRQF_SHARED);
	if (err) {
		dev_err(zfat->fa->fmc->hwdev, "can't request irq %i (err %i)\n",
			fa->fmc->irq, err);
		return ERR_PTR(err);
	}

	INIT_LIST_HEAD(&zfat->list_block);
	spin_lock_init(&zfat->lock);

	return &zfat->ti;
}
static void zfat_destroy(struct zio_ti *ti)
{
	struct fa_dev *fa = ti->cset->zdev->priv_d;
	struct zfat_instance *zfat = to_zfat_instance(ti);

	/* Disable all interrupt */
	zfa_common_conf_set(fa, &zfad_regs[ZFA_IRQ_MASK], ZFAT_NONE);
	fa->fmc->op->irq_free(fa->fmc);
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

	zfa_common_conf_set(fa, &zfad_regs[ZFAT_CFG_HW_EN], !status);
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
	zfa_common_conf_set(zfat->fa, &zfad_regs[ZFA_IRQ_MASK], ZFAT_NONE);
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
	.input_fire =		zfat_start_next_dma,
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

