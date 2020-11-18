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

#include "fmc-adc-100m14b4cha-private.h"

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

	/* irq to handle */
	fa_get_irq_status(fa, &status);
	if (!status)
		return IRQ_NONE; /* No interrupt fired by this mezzanine */

	if (status & FA_IRQ_ADC_ACQ_END) {
		struct zfad_block *zfad_block;
		unsigned long flags;

		dev_dbg(fa->msgdev, "Handle ADC interrupts\n");

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

	/* Release ADC IRQs */
	free_irq(platform_get_irq(fa->pdev, ADC_IRQ_TRG), fa);

	return 0;
}

int fa_enable_irqs(struct fa_dev *fa)
{
	fa_writel(fa, fa->fa_irq_adc_base,
			&zfad_regs[ZFA_IRQ_ADC_ENABLE_MASK],
			FA_IRQ_ADC_ACQ_END);
	return 0;
}

int fa_disable_irqs(struct fa_dev *fa)
{
	fa_writel(fa, fa->fa_irq_adc_base,
			&zfad_regs[ZFA_IRQ_ADC_DISABLE_MASK],
			FA_IRQ_ADC_ACQ_END);

	return 0;
}
