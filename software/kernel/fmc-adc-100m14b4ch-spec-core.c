// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2020 CERN (www.cern.ch)
 * Author: Federico Vaga <federico.vaga@cern.ch>
 */

#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/fmc.h>

#include "platform_data/fmc-adc-100m14b4cha.h"

enum fa_spec_dev_offsets {
	FA_SPEC_DBL_ADC_META_START = 0x00000000,
	FA_SPEC_DBL_ADC_META_END   = 0x00000040,
	FA_SPEC_ADC_MEM_START = 0x000002000,
	FA_SPEC_ADC_MEM_END = 0x000003FFF,
};

static const struct fmc_adc_platform_data fmc_adc_pdata = {
	.flags = 0,
	.calib_trig_time = 0,
	.calib_trig_threshold = 0,
	.calib_trig_internal = 0,
};

static int fa_spec_probe(struct platform_device *pdev)
{
	static struct resource fa_spec_fdt_res[] = {
		{
			.name = "fmc-adc-mem",
			.flags = IORESOURCE_MEM,
		},
		{
			.name = "fmc-adc-dma",
			.flags = IORESOURCE_DMA,
		},
		{
			.name = "fmc-adc-irq",
			.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
		},
		{
		.name = "fmc-adc-meta",
		.flags = IORESOURCE_MEM,
		.start = FA_SPEC_DBL_ADC_META_START,
		.end = FA_SPEC_DBL_ADC_META_END,
		},
	};
	struct platform_device_info pdevinfo = {
		.parent = &pdev->dev,
		.name = "fmc-adc-100m",
		.id = PLATFORM_DEVID_AUTO,
		.res = fa_spec_fdt_res,
		.num_res = ARRAY_SIZE(fa_spec_fdt_res),
		.data = &fmc_adc_pdata,
		.size_data = sizeof(fmc_adc_pdata),
		.dma_mask = DMA_BIT_MASK(32),
	};
	struct platform_device *pdev_child;
	struct resource *rmem;
	struct resource *r;
	int irq;
	int dma_dev_chan;
	struct fmc_slot *slot;
	bool present;

	slot = fmc_slot_get(pdev->dev.parent, 1);
	if (IS_ERR(slot)) {
		dev_err(&pdev->dev, "Can't find FMC slot 1 err: %ld\n",
			PTR_ERR(slot));
		return PTR_ERR(slot);
	}
	present = fmc_slot_present(slot);
	fmc_slot_put(slot);
	if (!present) {
		dev_err(&pdev->dev,
			"FMC slot: 1, not present\n");
		return -ENODEV;
	}

	rmem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!rmem) {
		dev_err(&pdev->dev, "Missing memory resource\n");
		return -EINVAL;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "Missing IRQ number\n");
		return -EINVAL;
	}

	r = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (!r) {
		dev_err(&pdev->dev, "Missing DMA engine\n");
		return -EINVAL;
	}
	dma_dev_chan = r->start;

	fa_spec_fdt_res[0].parent = rmem;
	fa_spec_fdt_res[0].start = rmem->start + FA_SPEC_ADC_MEM_START;
	fa_spec_fdt_res[0].end = rmem->start + FA_SPEC_ADC_MEM_END;
	fa_spec_fdt_res[1].start = dma_dev_chan;
	fa_spec_fdt_res[2].start = irq;
	fa_spec_fdt_res[3].start = rmem->start + FA_SPEC_DBL_ADC_META_START;
	fa_spec_fdt_res[3].end = rmem->start + FA_SPEC_DBL_ADC_META_END;

	pdev_child = platform_device_register_full(&pdevinfo);
	if (IS_ERR(pdev_child))
		return PTR_ERR(pdev_child);
	platform_set_drvdata(pdev, pdev_child);
	return 0;
}

static int fa_spec_remove(struct platform_device *pdev)
{
	struct platform_device *pdev_child = platform_get_drvdata(pdev);

	platform_device_unregister(pdev_child);

	return 0;
}

/**
 * List of supported platform
 */
enum fa_spec_version {
	FA_SPEC_VER = 0,
};

static const struct platform_device_id fa_spec_id_table[] = {
	{
		.name = "fmc-adc-100m-spec",
		.driver_data = FA_SPEC_VER,
	},
	{
		.name = "id:000010DC41444301",
		.driver_data = FA_SPEC_VER,
	},
	{
		.name = "id:000010dc41444301",
		.driver_data = FA_SPEC_VER,
	},
	{},
};

static struct platform_driver fa_spec_driver = {
	.driver = {
		.name = "fmc-adc-100m-spec",
		.owner = THIS_MODULE,
	},
	.id_table = fa_spec_id_table,
	.probe = fa_spec_probe,
	.remove = fa_spec_remove,
};
module_platform_driver(fa_spec_driver);

MODULE_AUTHOR("Federico Vaga <federico.vaga@cern.ch>");
MODULE_LICENSE("GPL");
MODULE_VERSION(VERSION);
MODULE_DESCRIPTION("Driver for the SPEC ADC 100 MSamples 14 bits 4 Channels");
MODULE_DEVICE_TABLE(platform, fa_spec_id_table);

MODULE_SOFTDEP("pre: spec_fmc_carrier fmc-adc-100m14b4ch");
