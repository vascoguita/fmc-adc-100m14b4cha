// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2020 CERN (www.cern.ch)
 * Author: Federico Vaga <federico.vaga@cern.ch>
 */

#include <linux/slab.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/mfd/core.h>
#include <linux/fmc.h>

#include "platform_data/fmc-adc-100m14b4cha.h"

#define SVEC_FMC_SLOTS 2

/*
 * From SVEC but we do not want to add a dependency for these 4 registers
 * which should never change by design. If they do, and you end up here:
 * sorry! It shouldn't have happened.
 */
#define SVEC_BASE_REGS_CSR 0x40UL
#define SVEC_FPGA_CSR_DDR4_ADDR (SVEC_BASE_REGS_CSR + 0x18)
#define SVEC_FPGA_DDR4_DMA (0x2000)
#define SVEC_FPGA_CSR_DDR5_ADDR (SVEC_BASE_REGS_CSR + 0x1C)
#define SVEC_FPGA_DDR5_DMA (0x3000)

enum fa_svec_dev_offsets {
  FA_SVEC_ADC1_MEM_START = 0x00002000,
  FA_SVEC_ADC1_MEM_END   = 0x00003FFF,
  FA_SVEC_ADC2_MEM_START = 0x00004000,
  FA_SVEC_ADC2_MEM_END   = 0x00005FFF,
};

static inline struct platform_device *platform_device_register_resndata_mask(
		struct device *parent, const char *name, int id,
		const struct resource *res, unsigned int num,
		const void *data, size_t size, u64 dma_mask) {

	struct platform_device_info pdevinfo = {
		.parent = parent,
		.name = name,
		.id = id,
		.res = res,
		.num_res = num,
		.data = data,
		.size_data = size,
		.dma_mask = dma_mask,
	};

	return platform_device_register_full(&pdevinfo);
}


static struct fmc_adc_platform_data fa_svec_adc_pdata[] = {
	{
		.flags = FMC_ADC_BIG_ENDIAN |
		         FMC_ADC_SVEC |
		         FMC_ADC_NOSQUASH_SCATTERLIST,
		.vme_reg_offset = SVEC_FPGA_CSR_DDR4_ADDR,
                .vme_dma_offset = SVEC_FPGA_DDR4_DMA,
		.calib_trig_time = 0,
		.calib_trig_threshold = 0,
		.calib_trig_internal = 0,
	}, {
		.flags = FMC_ADC_BIG_ENDIAN |
		         FMC_ADC_SVEC |
		         FMC_ADC_NOSQUASH_SCATTERLIST,
		.vme_reg_offset = SVEC_FPGA_CSR_DDR5_ADDR,
                .vme_dma_offset = SVEC_FPGA_DDR5_DMA,
		.calib_trig_time = 0,
		.calib_trig_threshold = 0,
		.calib_trig_internal = 0,
	}
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

static struct resource *fa_svec_res[] = {
	fa_svec_res1,
	fa_svec_res2,
};

struct fa_svec_data {
	struct platform_device *adc[2];
};


static int fa_svec_probe(struct platform_device *pdev)
{
	struct fa_svec_data *pdev_data;
	struct resource *rmem;
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

	pdev_data = kmalloc(sizeof(*pdev_data), GFP_KERNEL);
	if (!pdev_data)
		return -ENOMEM;

	for (i = 0; i < SVEC_FMC_SLOTS; ++i) {
		unsigned int res_n = ARRAY_SIZE(fa_svec_res1);
		struct resource res[res_n];
		struct fmc_slot *slot = fmc_slot_get(pdev->dev.parent, i + 1);
		int present;

                if (IS_ERR(slot)) {
			dev_err(&pdev->dev,
				"Can't find FMC slot %d err: %ld\n",
				i + 1, PTR_ERR(slot));
			continue;
		}

		present = fmc_slot_present(slot);
		fmc_slot_put(slot);
		dev_dbg(&pdev->dev, "FMC slot: %d, present: %d\n",
			i + 1, present);
		if (!present)
			continue;

		memcpy(res, fa_svec_res[i], sizeof(res));

	        res[0].parent = rmem;
		res[0].start += rmem->start;
		res[0].end += rmem->start;
		res[2].start += irq;
		pdev_data->adc[i] = platform_device_register_resndata_mask(&pdev->dev,
									   "fmc-adc-100m",
									   PLATFORM_DEVID_AUTO,
									   res,
									   res_n,
									   &fa_svec_adc_pdata[i],
									   sizeof(fa_svec_adc_pdata[i]),
									   DMA_BIT_MASK(32));
		if (IS_ERR(pdev_data->adc[i])) {
			dev_err(&pdev->dev,
				"Faild to register ADC instance %d\n",
				i);
			pdev_data->adc[i] = NULL;
		}
	}

	platform_set_drvdata(pdev, pdev_data);
        return 0;
}

static int fa_svec_remove(struct platform_device *pdev)
{
	struct fa_svec_data *pdev_data = platform_get_drvdata(pdev);
	int i;

	if (!pdev_data)
		return 0;

        for (i = 0; i < SVEC_FMC_SLOTS; ++i)
		if (pdev_data->adc[i])
			platform_device_unregister(pdev_data->adc[i]);
	kfree(pdev_data);

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
