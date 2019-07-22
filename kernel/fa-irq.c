// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright CERN 2012-2019
 * Author: Federico Vaga <federico.vaga@gmail.com>
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/bitops.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/interrupt.h>

#include "fmc-adc-100m14b4cha.h"
#include "fa-spec.h"

/**
 * It maps the ZIO blocks with an sg table, then it starts the DMA transfer
 * from the ADC to the host memory.
 *
 * @param cset
 */
int zfad_dma_start(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	struct zfad_block *zfad_block = cset->interleave->priv_d;
	uint32_t dev_mem_off, trg_pos, pre_samp;
	uint32_t val = 0;
	int try = 5, err;

	/*
	 * All programmed triggers fire, so the acquisition is ended.
	 * If the state machine is _idle_ we can start the DMA transfer.
	 * If the state machine it is not idle, try again 5 times
	 */
	while (try-- && val != FA100M14B4C_STATE_IDLE) {
		/* udelay(2); */
		val = fa_readl(fa, fa->fa_adc_csr_base,
			       &zfad_regs[ZFA_STA_FSM]);
	}

	if (val != FA100M14B4C_STATE_IDLE) {
		/* we can't DMA if the state machine is not idle */
		dev_warn(fa->msgdev,
			 "Can't start DMA on the last acquisition, "
			 "State Machine is not IDLE (status:%d)\n", val);
		return -EBUSY;
	}

	/*
	 * Disable all triggers to prevent fires between
	 * different DMA transfers required for multi-shots
	 */
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFAT_CFG_SRC], 0);

	/* Fix dev_mem_addr in single-shot mode */
	if (fa->n_shots == 1) {
		int nchan = FA100M14B4C_NCHAN;
		struct zio_control *ctrl = cset->chan[nchan].current_ctrl;

		/* get pre-samples from the current control (interleave chan) */
		pre_samp = ctrl->attr_trigger.std_val[ZIO_ATTR_TRIG_PRE_SAMP];
		/* Get trigger position in DDR */
		trg_pos = fa_readl(fa, fa->fa_adc_csr_base,
				   &zfad_regs[ZFAT_POS]);
		/*
		 * compute mem offset (in bytes): pre-samp is converted to
		 * bytes
		 */
		dev_mem_off = trg_pos - (pre_samp * cset->ssize * nchan);
		dev_dbg(fa->msgdev,
			"Trigger @ 0x%08x, pre_samp %i, offset 0x%08x\n",
			trg_pos, pre_samp, dev_mem_off);

		zfad_block[0].dev_mem_off = dev_mem_off;
	}

	dev_dbg(fa->msgdev, "Start DMA transfer\n");
	err = fa->carrier_op->dma_start(cset);
	if (err)
		return err;

	return 0;
}

/**
 * It completes a DMA transfer.
 * It tells to the ZIO framework that all blocks are done. Then, it re-enable
 * the trigger for the next acquisition. If the device is configured for
 * continuous acquisition, the function automatically start the next
 * acquisition
 *
 * @param cset
 */
void zfad_dma_done(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	struct zio_channel *interleave = cset->interleave;
	struct zfad_block *zfad_block = interleave->priv_d;
	struct zio_control *ctrl = NULL;
	struct zio_ti *ti = cset->ti;
	struct zio_block *block;
	struct zio_timestamp ztstamp;
	int i;
	uint32_t *trig_timetag;

	fa->carrier_op->dma_done(cset);

	/* for each shot, set the timetag of each ctrl block by reading the
	 * trig-timetag appended after the samples. Set also the acquisition
	 * start timetag on every blocks
	 */
	ztstamp.secs = fa_readl(fa, fa->fa_utc_base,
				&zfad_regs[ZFA_UTC_ACQ_START_SECONDS]);
	ztstamp.ticks = fa_readl(fa, fa->fa_utc_base,
				 &zfad_regs[ZFA_UTC_ACQ_START_COARSE]);
	ztstamp.bins = fa_readl(fa, fa->fa_utc_base,
				&zfad_regs[ZFA_UTC_ACQ_START_FINE]);
	for (i = 0; i < fa->n_shots; ++i) {
		block = zfad_block[i].block;
		ctrl = zio_get_ctrl(block);
		trig_timetag = (uint32_t *)(block->data + block->datalen
					    - FA_TRIG_TIMETAG_BYTES);

		if (unlikely((*(trig_timetag + 1) >> 8) != 0xACCE55))
			dev_err(fa->msgdev,
				"Wrong acquisition TAG, expected 0xACCE55 but got 0x%X (0x%X)\n",
				(*(trig_timetag + 1) >> 8), *trig_timetag);
		ctrl->tstamp.secs = ((uint64_t)*(trig_timetag + 1) & 0xFF) << 32;
		ctrl->tstamp.secs |= *(trig_timetag);
		ctrl->tstamp.ticks = *(trig_timetag + 2);
		ctrl->tstamp.bins = 0;
		ctrl->attr_trigger.ext_val[FA100M14B4C_TATTR_STA]= *(trig_timetag + 3);

		/* Acquisition start Timetag */
		ctrl->attr_channel.ext_val[FA100M14B4C_DATTR_ACQ_START_S] =
								ztstamp.secs;
		ctrl->attr_channel.ext_val[FA100M14B4C_DATTR_ACQ_START_C] =
								ztstamp.ticks;
		ctrl->attr_channel.ext_val[FA100M14B4C_DATTR_ACQ_START_F] =
								ztstamp.bins;

		/* resize the datalen, by removing the trigger tstamp */
		block->datalen = block->datalen - FA_TRIG_TIMETAG_BYTES;

		/* update seq num */
		ctrl->seq_num = i;
	}
	/* Sync the channel current control with the last ctrl block*/
	memcpy(&interleave->current_ctrl->tstamp,
		&ctrl->tstamp, sizeof(struct zio_timestamp));
	/* Update sequence number */
	interleave->current_ctrl->seq_num = ctrl->seq_num;

	/*
	 * All DMA transfers done! Inform the trigger about this, so
	 * it can store blocks into the buffer
	 */
	dev_dbg(fa->msgdev, "%i blocks transfered\n", fa->n_shots);
	zio_trigger_data_done(cset);

	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFAT_CFG_SRC],
		  ti->zattr_set.ext_zattr[FA100M14B4C_TATTR_SRC].value);
}


/**
 * It handles the error condition of a DMA transfer.
 * The function turn off the state machine by sending the STOP command
 *
 * @param cset
 */
void zfad_dma_error(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;

	fa->carrier_op->dma_error(cset);

	zfad_fsm_command(fa, FA100M14B4C_CMD_STOP);
	fa->n_dma_err++;

	if (fa->n_fires == 0)
		dev_err(fa->msgdev,
			"DMA error occurs but no block was acquired\n");
}


/*
 * job executed within a work thread
 * Depending of the carrier the job slightly differs:
 * SVEC: dma_start() blocks till the the DMA ends
 *      (fully managed by the vmebus driver)
 * Therefore the DMA outcome can be processed immediately
 * SPEC: dma_start() launch the job an returns immediately.
 * An interrupt DMA_DONE or ERROR is expecting to signal the end
 *       of the DMA transaction
 * (See fa-spec-irq.c::fa-spec_irq_handler)
 */

static void fa_irq_work(struct work_struct *work)
{
	struct fa_dev *fa = container_of(work, struct fa_dev, irq_work);
	struct zio_cset *cset = fa->zdev->cset;
	int res;

	/*
	 * This check is not crucial because the HW implements
	 * a solid state machine and acq-end can happens only after
	 * the execution of the n requested shots.
	 */
	fa->n_fires = fa->n_shots - fa_readl(fa, fa->fa_adc_csr_base,
					     &zfad_regs[ZFAT_SHOTS_REM]);

	if (fa->n_fires != fa->n_shots) {
		dev_err(fa->msgdev,
			"Expected %i trigger fires, but %i occurs\n",
			fa->n_shots, fa->n_fires);
	}

	res = zfad_dma_start(cset);
	if (!res) {
		/*
		 * No error.
		 * If there is an IRQ DMA src to notify the ends of the DMA,
		 * leave the workqueue.
		 * dma_done will be proceed on DMA_END reception.
		 * Otherwhise call dma_done in sequence
		 */
		if (fa->irq_src & FA_IRQ_SRC_DMA)
			/*
			 * waiting for END_OF_DMA IRQ
			 * with the CSET_BUSY flag Raised
			 * The flag will be lowered by the irq_handler
			 * handling END_DMA
			 */
			goto end;

		zfad_dma_done(cset);
	}
	/*
	 * Lower CSET_HW_BUSY
	 */
	spin_lock(&cset->lock);
	cset->flags &= ~ZIO_CSET_HW_BUSY;
	spin_unlock(&cset->lock);

end:
	if (res) {
		/* Stop acquisition on error */
		zfad_dma_error(cset);
	} else if (fa->enable_auto_start) {
		/* Automatic start next acquisition */
		dev_dbg(fa->msgdev, "Automatic start\n");
		zfad_fsm_command(fa, FA100M14B4C_CMD_START);
	}
}

/*
 * fat_get_irq_status
 * @fa: adc descriptor
 * @irq_status: destination of irq status
 *
 * Get irq and clear the register. To clear an interrupt we have to write 1
 * on the handled interrupt. We handle all interrupt so we clear all interrupts
 */
static void fa_get_irq_status(struct fa_dev *fa, uint32_t *irq_status)
{
	/* Get current interrupts status */
	*irq_status = fa_readl(fa, fa->fa_irq_adc_base, &zfad_regs[ZFA_IRQ_ADC_SRC]);
	dev_dbg(fa->msgdev,
		"IRQ fired an interrupt. IRQ status register: 0x%x\n",
		*irq_status);
	if (*irq_status)
		/* Clear current interrupts status */
		fa_writel(fa, fa->fa_irq_adc_base, &zfad_regs[ZFA_IRQ_ADC_SRC],
				*irq_status);
}

/*
 * fa_irq_handler
 * @irq:
 * @arg: pointer to fa_dev
 *
 * The ADC svec firmware fires interrupt from a single wishbone core
 * and throught the VIC ACQ_END and TRIG events.  Note about "TRIG"
 * event: the main reason to listen this interrupt was to read the
 * intermediate time stamps in case of multishots.
 * With the new firmware (>=3.0) the stamps come with the data,
 * therefore the driver doesn't have to listen "TRIG" event. This
 * enhancement remove completely the risk of loosing interrupt in case
 * of small number of samples and makes the retry loop in the hanlder
 * obsolete.
 */
irqreturn_t fa_irq_handler(int irq, void *arg)
{
	struct fa_dev *fa = arg;
	struct zio_cset *cset = fa->zdev->cset;
	uint32_t status;
	unsigned long flags;
	struct zfad_block *zfad_block;

	/* irq to handle */
	fa_get_irq_status(fa, &status);
	if (!status)
		return IRQ_NONE; /* No interrupt fired by this mezzanine */

	dev_dbg(fa->msgdev, "Handle ADC interrupts\n");

	if (status & FA_IRQ_ADC_ACQ_END) {
		/*
		 * Acquiring samples is a critical section
		 * protected against any concurrent abbort trigger.
		 * This is done by raising the flag CSET_BUSY at ACQ_END
		 * and lowered it at the end of DMA_END.
		 */
		spin_lock_irqsave(&cset->lock, flags);
		zfad_block = cset->interleave->priv_d;
		/* Check first if any concurrent trigger stop */
		/* has deleted zio blocks. In such a case */
		/* the flag is not raised and nothing is done */
		if (zfad_block != NULL && (cset->ti->flags & ZIO_TI_ARMED))
			cset->flags |= ZIO_CSET_HW_BUSY;
		spin_unlock_irqrestore(&cset->lock, flags);
		if (cset->flags & ZIO_CSET_HW_BUSY) {
			/* Job deferred to the workqueue: */
			/* Start DMA and ack irq on the carrier */
			queue_work(fa_workqueue, &fa->irq_work);
			/* register the core firing the IRQ in order to */
			/* check right IRQ seq.: ACQ_END followed by DMA_END */
			fa->last_irq_core_src = irq;
		}
	} else { /* unexpected interrupt we have to ack anyway */
		dev_err(fa->msgdev,
			"%s unexpected interrupt 0x%x\n",
			__func__, status);
	}

	return IRQ_HANDLED;

}

int fa_setup_irqs(struct fa_dev *fa)
{
	struct resource *r;
	int err;

	/* Request IRQ */
	dev_dbg(fa->msgdev, "Request irq\n");

	r = platform_get_resource(fa->pdev, IORESOURCE_IRQ, ADC_IRQ_TRG);
	err = request_any_context_irq(r->start, fa_irq_handler, 0,
				      r->name, fa);
	if (err < 0) {
		dev_err(fa->msgdev, "can't request irq %lli (error %i)\n",
			r->start, err);
		return err;
	}
	/* workqueue is required to execute DMA transaction */
	INIT_WORK(&fa->irq_work, fa_irq_work);

	/* set IRQ sources to listen */
	fa->irq_src = FA_IRQ_SRC_ACQ;

	if (fa->carrier_op->setup_irqs)
		err = fa->carrier_op->setup_irqs(fa);

	return err;
}

int fa_free_irqs(struct fa_dev *fa)
{
	/*
	 * When we unload the driver the FPGA is still running so it may
	 * rises interrupts. Disable IRQs in order to prevent spurious
	 * interrupt when the driver is not there to handle them.
	 */
	fa_disable_irqs(fa);

	/* Release carrier IRQs (if any) */
	if (fa->carrier_op->free_irqs)
		fa->carrier_op->free_irqs(fa);

	/* Release ADC IRQs */
	free_irq(platform_get_irq(fa->pdev, ADC_IRQ_TRG), fa);

	return 0;
}

int fa_enable_irqs(struct fa_dev *fa)
{
	dev_dbg(fa->msgdev, "Enable interrupts\n");

	fa_writel(fa, fa->fa_irq_adc_base,
			&zfad_regs[ZFA_IRQ_ADC_ENABLE_MASK],
			FA_IRQ_ADC_ACQ_END);

	if (fa->carrier_op->enable_irqs)
		fa->carrier_op->enable_irqs(fa);
	return 0;
}

int fa_disable_irqs(struct fa_dev *fa)
{
	dev_dbg(fa->msgdev, "Disable interrupts\n");

	fa_writel(fa, fa->fa_irq_adc_base,
			&zfad_regs[ZFA_IRQ_ADC_DISABLE_MASK],
			FA_IRQ_ADC_ACQ_END);

	if (fa->carrier_op->disable_irqs)
		fa->carrier_op->disable_irqs(fa);
	return 0;
}

int fa_ack_irq(struct fa_dev *fa, int irq_id)
{
	return 0;
}
