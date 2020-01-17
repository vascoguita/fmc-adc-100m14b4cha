// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2019 CERN (www.cern.ch)
 * Author: Federico Vaga <federico.vaga@cern.ch>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mfd/core.h>
#include <linux/mod_devicetable.h>

enum ft_svec_dev_offsets {
	FT_SVEC_FA100_S1_MEM_START = 0x00002000,
	FT_SVEC_FA100_S1_MEM_END   = 0x00003A00,
	FT_SVEC_FA100_S2_MEM_START = 0x00004000,
	FT_SVEC_FA100_S2_MEM_END   = 0x00005A00,
};

/* MFD devices */
enum svec_fpga_mfd_devs_enum {
	FT_SVEC_MFD_FA100_S1,
	FT_SVEC_MFD_FA100_S2,
};

static struct resource ft_svec_fdt100_res_s1[] = {
	{
		.name  = "fmc-adc-100m14b4ch-mem",
		.flags = IORESOURCE_MEM,
		.start = FT_SVEC_FA100_S1_MEM_START,
		.end   = FT_SVEC_FA100_S1_MEM_END,
	}, {
		.name  = "fmc-adc-100m14b4ch-irq",
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
		.start = 0,
		.end   = 0,
	},
};

static struct resource ft_svec_fdt100_res_s2[] = {
	{
		.name  = "fmc-adc-100m14b4ch-mem",
		.flags = IORESOURCE_MEM,
		.start = FT_SVEC_FA100_S2_MEM_START,
		.end   = FT_SVEC_FA100_S2_MEM_END,
	}, {
		.name  = "fmc-adc-100m14b4ch-irq",
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
		.start = 1,
		.end   = 1,
	},
};

static const struct mfd_cell ft_svec_mfd_devs[] = {
	[FT_SVEC_MFD_FA100_S1] = {
		.name = "adc-100m-svec",
		.platform_data = NULL,
		.pdata_size = 0,
		.num_resources = ARRAY_SIZE(ft_svec_fdt100_res_s1),
		.resources = ft_svec_fdt100_res_s1,
	},
	[FT_SVEC_MFD_FA100_S2] = {
		.name = "adc-100m-svec",
		.platform_data = NULL,
		.pdata_size = 0,
		.num_resources = ARRAY_SIZE(ft_svec_fdt100_res_s2),
		.resources = ft_svec_fdt100_res_s2,
	},
};


static int ft_svec_probe(struct platform_device *pdev)
{
	struct resource *rmem;
	int irq;

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

	/*
	 * We know that this design uses the HTVIC IRQ controller.
	 * This IRQ controller has a linear mapping, so it is enough
	 * to give the first one as input
	 */

	return mfd_add_devices(&pdev->dev, PLATFORM_DEVID_AUTO,
			       ft_svec_mfd_devs,
			       ARRAY_SIZE(ft_svec_mfd_devs),
			       rmem, irq, NULL);
}

static int ft_svec_remove(struct platform_device *pdev)
{
	mfd_remove_devices(&pdev->dev);

	return 0;
}

static const struct platform_device_id ft_svec_id_table[] = {
	{
		.name = "fmc-adc-100m-svec",
		.driver_data = 0,
	}, {
		.name = "id:000010DC41444302",
		.driver_data = 0,
	}, {
		.name = "id:000010dc41444302",
		.driver_data = 0,
	},
	{},
};

static struct platform_driver ft_svec_driver = {
	.driver = {
		.name = "fmc-adc-100m-svec",
		.owner = THIS_MODULE,
	},
	.id_table = ft_svec_id_table,
	.probe = ft_svec_probe,
	.remove = ft_svec_remove,
};
module_platform_driver(ft_svec_driver);

MODULE_AUTHOR("Federico Vaga <federico.vaga@cern.ch>");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
MODULE_DESCRIPTION("Driver for the FMC ADC 100M SVEC REF");
MODULE_DEVICE_TABLE(platform, ft_svec_id_table);

MODULE_SOFTDEP("pre: svec_fmc_carrier fmc-adc-100m14b4ch");
