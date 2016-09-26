// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2012-2019 CERN (www.cern.ch)
 * Author: Federico Vaga <federico.vaga@gmail.com>
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include "fmc-adc-100m14b4cha.h"
#include "fa-svec.h"

static int fa_svec_init(struct fa_dev *fa)
{
	struct fa_svec_data *cdata;
	struct resource *r;
	unsigned int res_i;

	cdata = kzalloc(sizeof(struct fa_svec_data), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	r = platform_get_resource(fa->pdev, IORESOURCE_BUS, ADC_CARR_VME_ADDR);
	cdata->vme_ddr_data = r->start;

	r = platform_get_resource(fa->pdev, IORESOURCE_BUS, ADC_BUS_FMC_SLOT);
	switch(r->start) {
	case 1:
		res_i = FA_CAR_FMC0_RES;
		break;
	case 2:
		res_i = FA_CAR_FMC1_RES;
		break;
	default:
		return -EINVAL;
	}
	cdata->fa_dma_ddr_addr = fa->fa_top_level + 0x2000;

	fa_writel(fa, fa->fa_carrier_csr_base, &fa_svec_regfield[res_i], 1);
	mdelay(50);
	fa_writel(fa, fa->fa_carrier_csr_base, &fa_svec_regfield[res_i], 0);
	mdelay(50);

	/* register carrier data */
	fa->carrier_data = cdata;

	return 0;
}

static void fa_svec_exit(struct fa_dev *fa)
{
	kfree(fa->carrier_data);
}

struct fa_carrier_op fa_svec_op = {
	.init = fa_svec_init,
	.exit = fa_svec_exit,
	.dma_start = fa_svec_dma_start,
	.dma_done = fa_svec_dma_done,
	.dma_error = fa_svec_dma_error,
};
