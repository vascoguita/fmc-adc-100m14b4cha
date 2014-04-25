/*
 * Copyright CERN 2012
 * Author: Federico Vaga <federico.vaga@gmail.com>
 *
 * IRQ function handler
 */


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/bitops.h>
#include <linux/spinlock.h>
#include <linux/io.h>

#include <linux/zio.h>
#include <linux/zio-trigger.h>

#include "fmc-adc.h"
#include "fa-spec.h"

/*
 * fa_get_irq_status
 * @fa: adc descriptor
 * @irq_status: destination of irq status
 * @irq_multi: destination of irq multi
 *
 * Get irq and clear the register. To clear an interrupt we have to write 1
 * on the handled interrupt. We handle all interrupt so we clear all interrupts
 */
static void fa_get_irq_status(struct fa_dev *fa, int irq_core_base,
			      uint32_t *irq_status)
{

	/* Get current interrupts status */
	*irq_status = fa_readl(fa, irq_core_base,
			&fa_spec_regs[ZFA_IRQ_DMA_SRC]);
	dev_dbg(&fa->fmc->dev, "core DMA: 0x%x fired an interrupt. IRQ status register: 0x%x\n",
			irq_core_base, *irq_status);
	if (*irq_status)
		/* Clear current interrupts status */
		fa_writel(fa, irq_core_base,
			&fa_spec_regs[ZFA_IRQ_DMA_SRC], *irq_status);
}

/*
 * zfad_irq
 * @irq:
 * @ptr: pointer to fmc_device
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
irqreturn_t fa_spec_irq_handler(int irq_core_base, void *ptr)
{
	struct fmc_device *fmc = ptr;
	struct fa_dev *fa = fmc_get_drvdata(fmc);
	struct zio_cset *cset = fa->zdev->cset;
	uint32_t status;

	if (unlikely(fa->last_irq_core_src == irq_core_base)) {
		WARN(1, "Cannot handle two consecutives %s interrupt."
			"The ADC doesn't behave properly\n",
			(irq_core_base == fa->fa_irq_adc_base) ? "ACQ" : "DMA");
		/* Stop Acquisition, ADC it is not working properly */
		zfad_fsm_command(fa, ZFA_STOP);
		fa->last_irq_core_src = FA_SPEC_IRQ_SRC_NONE;
		return IRQ_HANDLED;
	}
	/* irq to handle */
	fa_get_irq_status(fa, irq_core_base, &status);
	if (!status)
		return IRQ_NONE;
	dev_dbg(&fa->fmc->dev, "Handle ADC interrupts\n");

	if (status & FA_SPEC_IRQ_DMA_DONE)
		zfad_dma_done(cset);
	else if (unlikely(status  & FA_SPEC_IRQ_DMA_ERR))
		zfad_dma_error(cset);

	/* register the core which just fired the IRQ */
	/* check proper sequence of IRQ in case of multi IRQ (ACQ + DMA)*/
	fa->last_irq_core_src = irq_core_base;

	/*
	 * DMA transaction is finished
	 * we can safely lower CSET_BUSY
	 */
	spin_lock(&cset->lock);
	cset->flags &= ~ZIO_CSET_HW_BUSY;
	spin_unlock(&cset->lock);

	/* ack the irq */
	fa->fmc->op->irq_ack(fa->fmc);

	return IRQ_HANDLED;
}

