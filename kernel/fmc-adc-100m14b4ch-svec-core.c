// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2020 CERN (www.cern.ch)
 * Author: Federico Vaga <federico.vaga@cern.ch>
 */

#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/mfd/core.h>
#include <linux/fmc.h>

#include "platform_data/fmc-adc-100m14b4cha.h"

enum fa_svec_dev_offsets {
	FA_SVEC_ADC1_MEM_START = 0x000002000,
	FA_SVEC_ADC1_MEM_END = 0x00003FFF,
	FA_SVEC_ADC2_MEM_START = 0x00004000,
	FA_SVEC_ADC2_MEM_END = 0x000005FFF,
};

static struct fmc_adc_platform_data fmc_adc_pdata = {
	.flags = FMC_ADC_BIG_ENDIAN | FMC_ADC_SVEC,
	.calib_trig_time = 0,
	.calib_trig_threshold = 0,
	.calib_trig_internal = 0,

};

/* MFD devices */
enum svec_fpga_mfd_devs_enum {
	FA_SVEC_MFD_FA1 = 0,
	FA_SVEC_MFD_FA2,
};

static struct resource fa_svec_res1[] = {
	{
		.name = "fmc-adc-100m-mem.1",
		.flags = IORESOURCE_MEM,
		.start = FA_SVEC_ADC1_MEM_START,
		.end = FA_SVEC_ADC1_MEM_END,
	},
	{
		.name = "fmc-adc-100m-dma.1",
		.flags = IORESOURCE_DMA,
	},
	{
		.name = "fmc-adc-100m-irq.1",
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
		.start = 0,
		.end = 0,
	},
};
static struct resource fa_svec_res2[] = {
	{
		.name = "fmc-adc-100m-mem.2",
		.flags = IORESOURCE_MEM,
		.start = FA_SVEC_ADC2_MEM_START,
		.end = FA_SVEC_ADC2_MEM_END,
	},
	{
		.name = "fmc-adc-100m-dma.2",
		.flags = IORESOURCE_DMA,
	},
	{
		.name = "fmc-adc-100m-irq.2",
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
		.start = 1,
		.end = 1,
	},
};


#define MFD_ADC(_n)                                               \
	{                                                         \
		.name = "fmc-adc-100m",                           \
		.platform_data = &fmc_adc_pdata,                  \
		.pdata_size = sizeof(fmc_adc_pdata),              \
		.num_resources = ARRAY_SIZE(fa_svec_res##_n), \
		.resources = fa_svec_res##_n,                 \
	}

static const struct mfd_cell fa_svec_mfd_devs1[] = {
	MFD_ADC(1),
};
static const struct mfd_cell fa_svec_mfd_devs2[] = {
	MFD_ADC(2),
};
static const struct mfd_cell fa_svec_mfd_devs3[] = {
	MFD_ADC(1),
	MFD_ADC(2),
};

static const struct mfd_cell *fa_svec_mfd_devs[] = {
	fa_svec_mfd_devs1,
	fa_svec_mfd_devs2,
	fa_svec_mfd_devs3,
};

static int fa_svec_probe(struct platform_device *pdev)
{
	struct resource *rmem;
	int idev = 0;
	int ndev;
	int irq;
	int i;

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

	for (i = 1; i <= 2; ++i) {
		struct fmc_slot *slot = fmc_slot_get(pdev->dev.parent, i);
		int present;

                if (IS_ERR(slot)) {
			dev_err(&pdev->dev,
				"Can't find FMC slot %d err: %ld\n",
				i, PTR_ERR(slot));
			return PTR_ERR(slot);
		}

		present = fmc_slot_present(slot);
		fmc_slot_put(slot);
		dev_dbg(&pdev->dev, "FMC slot: %d, present: %d\n",
			i, present);
		if (present)
			idev |= BIT(i - 1);
	}

	if (idev == 0)
		return -ENODEV;
	idev--;

	/*
	 * We know that this design uses the HTVIC IRQ controller.
	 * This IRQ controller has a linear mapping, so it is enough
	 * to give the first one as input
	 */
	ndev = 1 + !!(idev & 0x2);
	dev_dbg(&pdev->dev, "Found %d, point to mfd_cell %d\n", ndev, idev);
	return mfd_add_devices(&pdev->dev, PLATFORM_DEVID_AUTO,
			       fa_svec_mfd_devs[idev], ndev,
			       rmem, irq, NULL);
}

static int fa_svec_remove(struct platform_device *pdev)
{
	mfd_remove_devices(&pdev->dev);

	return 0;
}

/**
 * List of supported platform
 */
enum fa_svec_version {
	FA_SVEC_VER = 0,
};

static const struct platform_device_id fa_svec_id_table[] = {
	{
		.name = "fmc-adc-100m-svec",
		.driver_data = FA_SVEC_VER,
	},
	{
		.name = "id:000010DC41444302",
		.driver_data = FA_SVEC_VER,
	},
	{
		.name = "id:000010dc41444302",
		.driver_data = FA_SVEC_VER,
	},
	{},
};

static struct platform_driver fa_svec_driver = {
	.driver = {
		.name = "fmc-adc-100m-svec",
		.owner = THIS_MODULE,
	},
	.id_table = fa_svec_id_table,
	.probe = fa_svec_probe,
	.remove = fa_svec_remove,
};
module_platform_driver(fa_svec_driver);

MODULE_AUTHOR("Federico Vaga <federico.vaga@cern.ch>");
MODULE_LICENSE("GPL");
MODULE_VERSION(VERSION);
MODULE_DESCRIPTION("Driver for the SVEC ADC 100 MSamples 14 bits 4 Channels");
MODULE_DEVICE_TABLE(platform, fa_svec_id_table);

MODULE_SOFTDEP("pre: svec_fmc_carrier fmc-adc-100m14b4ch");
