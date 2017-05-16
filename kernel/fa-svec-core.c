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

	cdata = kzalloc(sizeof(struct fa_svec_data), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	r = platform_get_resource(fa->pdev, IORESOURCE_BUS, ADC_CARR_VME_ADDR);
	cdata->vme_ddr_data = r->start;
	cdata->fa_dma_ddr_addr = fa->fa_top_level + 0x2000;

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
};
