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
	struct spec_fa *fa;
	unsigned int n_acq_dev;	/* number of acquisitions on device memory */
	unsigned int n_err;	/* number of errors */
};
#define to_zfat_instance(_ti) container_of(_ti, struct zfat_instance, ti)

/* Device registers */
extern const struct zio_reg_desc zfad_regs[];

/* zio trigger attributes */
static DEFINE_ZATTR_STD(TRIG, zfat_std_zattr) = {
	/* Number of shots */
	ZATTR_REG(trig, ZATTR_TRIG_REENABLE, S_IRUGO | S_IWUGO, ZFAT_SHOTS_NB, 0),

	/*
	 * The ADC has pre-sample and post-sample configuration. NSAMPLES doesn't
	 * apply in this case, so it is a read-only attribute update with the
	 * value calculated pre-sample + post-sample.
	 * If read from device, it contains the numer of samples acquired in a
	 * particular moment
	 */
	ZATTR_REG(trig, ZATTR_TRIG_NSAMPLES, S_IRUGO, ZFAT_CNT, 0),
};
static struct zio_attribute zfat_ext_zattr[] = {
	/* Config register */
	/* Hardware trigger selction
	 * 0: internal (data threshold)
	 * 1: external (front panel trigger input)
	 */
	ZATTR_EXT_REG("hw_select", S_IRUGO | S_IWUGO, ZFAT_CFG_INT_SEL, 0),
	/*
	 * Hardware trigger polarity
	 * 0: positive edge/slope
	 * 1: negative edge/slope
	 */
	ZATTR_EXT_REG("polarity", S_IRUGO | S_IWUGO, ZFAT_CFG_HW_POL, 0),
	/* Enable (1) or disable (0) hardware trigger */
	ZATTR_EXT_REG("hw_trig_enable", S_IRUGO | S_IWUGO, ZFAT_CFG_HW_EN, 0),
	/* Enable (1) or disable (0) software trigger */
	ZATTR_EXT_REG("sw_trig_enable", S_IRUGO | S_IWUGO, ZFAT_CFG_SW_EN, 0),
	/*
	 * Channel selection for internal trigger
	 * 0: channel 1
	 * 1: channel 2
	 * 2: channel 3
	 * 3: channel 4
	 */
	ZATTR_EXT_REG("int_select", S_IRUGO | S_IWUGO, ZFAT_CFG_INT_SEL, 0),
	/* Threshold value for internal trigger */
	ZATTR_EXT_REG("int_threshold", S_IRUGO | S_IWUGO, ZFAT_CFG_THRES, 0),

	/* Delay */
	ZATTR_EXT_REG("delay", S_IRUGO | S_IWUGO, ZFAT_DLY, 0),

	/* Software */
	PARAM_EXT_REG("sw_fire", S_IWUGO, ZFAT_SW, 0),

	/* Position address */
	ZATTR_EXT_REG("position_addr", S_IRUGO, ZFAT_POS, 0),

	/* Pre-sample*/
	ZATTR_EXT_REG("pre-sample", S_IRUGO | S_IWUGO, ZFAT_PRE, 0),

	/* Post-sample*/
	ZATTR_EXT_REG("post-sample", S_IRUGO | S_IWUGO, ZFAT_POST, 0),
};


/* FIXME: zio need an update, introduce PRE and POST and replace NSAMPLES*/
static void zfat_update_nsample(struct zio_ti *ti, enum zfadc_dregs_enum type,
				uint32_t val)
{
	struct zio_attribute_set *zattr_set =  &ti->zattr_set;
	struct zio_attribute *n = &zattr_set->std_zattr[ZATTR_TRIG_NSAMPLES];
	 /* 9-th attribute is PRE */
	struct zio_attribute *pre = &zattr_set->ext_zattr[9];
	/* 10-th attribute is POST */
	struct zio_attribute *post = &zattr_set->ext_zattr[10];

	if (type == ZFAT_PRE)
		n->value = val + post->value;
	else if (type == ZFAT_POST)
		n->value = pre->value + val;
	/*
	 * FIXME n->value must update current_control. Make propagete_value
	 * a public function?
	 */
}

/* set a value to a FMC-ADC trigger register */
static int zfat_conf_set(struct device *dev, struct zio_attribute *zattr,
			 uint32_t usr_val)
{
	const struct zio_reg_desc *reg = &zfad_regs[zattr->priv.addr];
	struct zio_ti *ti = to_zio_ti(dev);
	int err;

	switch (zattr->priv.addr) {
		case ZFAT_SW:
			/* Fire if software trigger is enabled */
			if (!ti->zattr_set.ext_zattr[3].value) {
				dev_err(dev, "sw trigger must be enable");
				return -EPERM;
			}
			/* Abort current acquisition if any */
			err = zfa_common_conf_set(dev,
					&zfad_regs[ZFA_CTL_FMS_CMD], 2);
			if (err)
				return err;
			zio_trigger_abort(ti->cset);
			/* Start a new acquisition */
			zio_fire_trigger(ti);
			break;
	}

	err = zfa_common_conf_set(dev, reg, usr_val);
	if (err)
		return err;

	if ( zattr->priv.addr == ZFAT_PRE ||  zattr->priv.addr == ZFAT_POST) {
		zfat_update_nsample(to_zio_ti(dev), zattr->priv.addr, usr_val);
	}

	return 0;
}
/* get the value of a FMC-ADC trigger register */
static int zfat_info_get(struct device *dev, struct zio_attribute *zattr,
			 uint32_t *usr_val)
{
	zfa_common_info_get(dev, &zfad_regs[zattr->priv.addr], usr_val);

	return 0;
}
static const struct zio_sysfs_operations zfat_s_op = {
	.conf_set = zfat_conf_set,
	.info_get = zfat_info_get,
};

/*
 *
 */
static void zfad_unmap_dma(struct zio_cset *cset)
{
	struct spec_fa *fa = cset->zdev->priv_d;

	dma_unmap_sg(&fa->spec->pdev->dev);
	sg_free_table(&fa->sgt);
}

irqreturn_t zfadc_irq(int irq, void *ptr)
{
	struct zfat_instance *zfat = ptr;
	uint32_t irq_status = 0, val;

	zfa_common_info_get(&zfat->ti.cset->head.dev,
			    &zfad_regs[ZFA_IRQ_SRC],&irq_status);
	dev_dbg(&zfat->ti.head.dev, "irq status = 0x%x\n", irq_status);

	if (irq_status & (ZFAT_DMA_DONE | ZFAT_DMA_ERR)) {
		if (irq_status & ZFAT_DMA_DONE) { /* DMA done*/
			zio_trigger_data_done(zfat->ti.cset);
			zfat->n_acq_dev--;
		} else {	/* DMA error */
			zio_trigger_abort(zfat->ti.cset);
			zfat->n_err++;
		}
		/* unmap dma */
		zfad_unmap_dma(&zfat->ti.cset);
		/* Enable all triggers */
		zfa_common_conf_set(&zfat->ti.cset->head.dev,
				    &zfad_regs[ZFAT_CFG_SW_EN], 0);
		zfa_common_conf_set(&zfat->ti.cset->head.dev,
				    &zfad_regs[ZFAT_CFG_HW_EN], 0);
		/* Start state machine */
		zfa_common_conf_set(&zfat->ti.cset->head.dev,
				    &zfad_regs[ZFA_CTL_FMS_CMD], 2);
	}

	if (irq_status & ZFAT_TRG_FIRE) { /* Trigger fire */
		/*
		 * FIXME: I think we don't care about this because ZIO fires
		 * a fake trigger before DMA
		 */
		zfat->n_acq_dev++;
	}
	if (irq_status & ZFAT_ACQ_END) { /* Acquisition end */
		/*
		 * We fire the trigger at the and of the acquisition because
		 * FMC-ADC allow DMA only when acquisition end and the
		 * state machine is in the idle status. When the real trigger
		 * fire the state machine is not idle. This has not any
		 * influence on the ZIO side: we are interested to DMA data
		 */
		zfa_common_info_get(&zfat->ti.cset->head.dev,
				    &zfad_regs[ZFA_STA_FSM],&val);
		if (val == ZFA_STATE_IDLE) {
			/* Stop state machine */
			zfa_common_conf_set(&zfat->ti.cset->head.dev,
					    &zfad_regs[ZFA_CTL_FMS_CMD], 2);
			/* Disable all triggers */
			zfa_common_conf_set(&zfat->ti.cset->head.dev,
					    &zfad_regs[ZFAT_CFG_HW_EN], 0);
			zfa_common_conf_set(&zfat->ti.cset->head.dev,
					    &zfad_regs[ZFAT_CFG_SW_EN], 0);
			/* Fire ZIO trigger so it will DMA */
			zio_fire_trigger(&zfat->ti);
		} else {
			/* we can't DMA if the state machine is not idle */
			dev_warn(&zfat->ti.cset->head.dev,
				 "Can't start DMA on the last acquisition\n");
		}
	}

	return IRQ_HANDLED;
}

/* create an instance of the FMC-ADC trigger */
static struct zio_ti *zfat_create(struct zio_trigger_type *trig,
				 struct zio_cset *cset,
				 struct zio_control *ctrl, fmode_t flags)
{
	struct spec_dev *spec = cset->zdev->priv_d;
	struct zfat_instance *zfat;
	int err;

	if (!spec) {
		dev_err(&cset->head.dev, "no spec device defined\n");
		return ERR_PTR(-ENODEV);
	}

	zfat = kzalloc(sizeof(struct zfat_instance), GFP_KERNEL);
	if (!zfat)
		return ERR_PTR(-ENOMEM);

	zfat->fa = spec->sub_priv;

	err = request_irq(spec->pdev->irq, zfadc_irq, IRQF_SHARED, "wr-nic", zfat);
	if (err) {
		dev_err(&spec->pdev->dev, "can't request irq %i (err %i)\n",
				spec->pdev->irq, err);
		return ERR_PTR(err);
	}

	/* Enable all interrupt */
	zfa_common_conf_set(&cset->head.dev, &zfad_regs[ZFA_IRQ_MASK],
			    ZFAT_ALL);

	return &zfat->ti;
}

static void zfat_destroy(struct zio_ti *ti)
{
	/* Disable all interrupt */
	zfa_common_conf_set(&ti->cset->head.dev, &zfad_regs[ZFA_IRQ_MASK],
			    ZFAT_NONE);
	kfree(to_zfat_instance(ti));
}

/* status is active low on ZIO but active high on the FMC-ADC */
static void zfat_change_status(struct zio_ti *ti, unsigned int status)
{
	/* Enable/Disable HW trigger */
	zfa_common_conf_set(&ti->head.dev, &zfad_regs[ZFAT_CFG_HW_EN], !status);
	/* Enable/Disable SW trigger */
	zfa_common_conf_set(&ti->head.dev, &zfad_regs[ZFAT_CFG_SW_EN], !status);
}

/*
 * Abort depends on the state machine status and DMA status.*/
static void zfat_abort(struct zio_cset *cset)
{

}

static const struct zio_trigger_operations zfat_ops = {
	.create =		zfat_create,
	.destroy =		zfat_destroy,
	.change_status =	zfat_change_status,
	.abort =		zfat_abort,
};

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

