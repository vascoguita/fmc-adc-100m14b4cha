// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2012-2019 CERN (www.cern.ch)
 * Author: Federico Vaga <federico.vaga@gmail.com>
 */

#include <linux/time.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

#include "fmc-adc-100m14b4cha.h"
#include "fa-spec.h"

static int fa_spec_init(struct fa_dev *fa)
{
	struct resource *r;
	struct fa_spec_data *cdata;

	cdata = kzalloc(sizeof(struct fa_spec_data), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	r = platform_get_resource(fa->pdev, IORESOURCE_MEM, ADC_CARR_DMA);
	cdata->fa_dma_base = ioremap(r->start, resource_size(r));
	cdata->fa_irq_dma_base = cdata->fa_dma_base + 0x0200;

	dev_info(fa->msgdev,
		"Spec Base addrs: irq_dmma: %p, dma_ctrl: %p\n",
		cdata->fa_irq_dma_base, cdata->fa_dma_base);

	/* Set DMA to transfer data from device to host */
	fa_writel(fa, cdata->fa_dma_base,
			&fa_spec_regs[ZFA_DMA_BR_DIR], 0);

	/* register carrier data */
	fa->carrier_data = cdata;
	dev_info(fa->msgdev, "spec::%s successfully executed\n", __func__);
	return 0;
}

static int fa_spec_reset(struct fa_dev *fa)
{
	/*struct spec_dev *spec = fa->fmc->carrier_data;*/

	dev_info(fa->msgdev, "%s: resetting ADC core through Gennum.\n",
		 __func__);
	return 0;
}

static void fa_spec_exit(struct fa_dev *fa)
{
	kfree(fa->carrier_data);
}

/* Unfortunately, on the spec this is GPIO9, i.e. IRQ(1) */
/* FIXME find a way to get rid of fmc here
 * This is used only by the SPEC design, is it not possible to avoid it
 * and let the VHDL configure the GPIO?
 */
/* static struct fmc_gpio fa_gpio_on[] = { */
/* 	{ */
/* 	 .gpio = FMC_GPIO_IRQ(0), */
/* 	 .mode = GPIOF_DIR_IN, */
/* 	 .irqmode = IRQF_TRIGGER_RISING, */
/* 	 } */
/* }; */

/* static struct fmc_gpio fa_gpio_off[] = { */
/* 	{ */
/* 	 .gpio = FMC_GPIO_IRQ(0), */
/* 	 .mode = GPIOF_DIR_IN, */
/* 	 .irqmode = 0, */
/* 	 } */
/* }; */

static int fa_spec_setup_irqs(struct fa_dev *fa)
{
	struct resource *r;
	int err;

	r = platform_get_resource(fa->pdev, IORESOURCE_IRQ, ADC_IRQ_DMA);
	err = request_any_context_irq(r->start, fa_spec_irq_handler, 0,
				      r->name, fa);
	if (err < 0) {
		dev_err(fa->msgdev, "can't request irq 0x%llx (error %i)\n",
			r->start, err);
		return err;
	}
	//fmc_gpio_config(fmc, fa_gpio_on, ARRAY_SIZE(fa_gpio_on));
	dev_info(fa->msgdev, "spec::%s successfully executed\n", __func__);

	/* Add SPEC specific IRQ sources to listen */
	fa->irq_src |= FA_IRQ_SRC_DMA;

	return 0;
}

static int fa_spec_free_irqs(struct fa_dev *fa)
{
	/* Release DMA IRQs */
	free_irq(platform_get_irq(fa->pdev, ADC_IRQ_DMA), fa);

	/* fmc_gpio_config(fmc, fa_gpio_off, ARRAY_SIZE(fa_gpio_off)); */

	return 0;
}

static int fa_spec_enable_irqs(struct fa_dev *fa)
{
	struct fa_spec_data *spec_data = fa->carrier_data;

	fa_writel(fa, spec_data->fa_irq_dma_base,
			&fa_spec_regs[ZFA_IRQ_DMA_ENABLE_MASK],
			FA_SPEC_IRQ_DMA_ALL);

	return 0;
}

static int fa_spec_disable_irqs(struct fa_dev *fa)
{
	struct fa_spec_data *spec_data = fa->carrier_data;

	fa_writel(fa, spec_data->fa_irq_dma_base,
			&fa_spec_regs[ZFA_IRQ_DMA_DISABLE_MASK],
			FA_SPEC_IRQ_DMA_NONE);

	return 0;
}

static int fa_spec_ack_irq(struct fa_dev *fa, int irq_id)
{
	return 0;
}

struct fa_carrier_op fa_spec_op = {
	.init = fa_spec_init,
	.reset_core = fa_spec_reset,
	.exit = fa_spec_exit,
	.setup_irqs = fa_spec_setup_irqs,
	.free_irqs = fa_spec_free_irqs,
	.enable_irqs = fa_spec_enable_irqs,
	.disable_irqs = fa_spec_disable_irqs,
	.ack_irq = fa_spec_ack_irq,
};
