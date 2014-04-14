/*
 * Copyright CERN 2012
 * Author: Federico Vaga <federico.vaga@gmail.com>
 *
 * handle DMA mapping
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>

#include <linux/fmc.h>

#include <linux/zio.h>
#include <linux/zio-buffer.h>
#include <linux/zio-trigger.h>

#include "fmc-adc-100m14b4cha.h"
#include "fa-spec.h"

#include "zio-helpers.h"

int fa_spec_dma_start(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	struct fa_spec_data *spec_data = fa->carrier_data;
	struct device *dev = &fa->fmc->dev;
	struct zio_channel *interleave = cset->interleave;
	struct zfad_block *zfad_block = interleave->priv_d;
	struct zio_block *blocks[fa->n_shots];
	uint32_t dev_mem_off = 0;
	dma_addr_t tmp;
	struct scatterlist *sg;
	unsigned int i, sglen, size, i_blk;
	int err;

	/*
	 *  FIXME very inefficient because arm trigger already prepare
	 * something like zio_block_sg. In the future ZIO can alloc more
	 * than 1 block at time
	 */
	for (i = 0; i < fa->n_shots; ++i)
		blocks[i] = zfad_block[i].block;

	fa->zdma = zio_dma_alloc_sg(fa->fmc->hwdev, blocks, fa->n_shots,
				    GFP_ATOMIC);

	/* Limited to 32-bit (kernel limit) TODO the type should be generic */
	size = sizeof(struct fa_dma_item) * fa->zdma->sgt.nents;
	spec_data->items = kzalloc(size, GFP_ATOMIC);
	if (!spec_data->items) {
		dev_err(fa->fmc->hwdev, "cannot allocate coherent dma memory\n");
		err = -ENOMEM;
		goto out_alloc_item;
	}
	spec_data->dma_list_item = dma_map_single(fa->fmc->hwdev,
						  spec_data->items, size,
						  DMA_TO_DEVICE);
	if (!spec_data->dma_list_item) {
		err = -ENOMEM;
		goto out_map_single;
	}

	/* Map DMA buffers */
	sglen = dma_map_sg(fa->fmc->hwdev, fa->zdma->sgt.sgl,
			   fa->zdma->sgt.nents, DMA_FROM_DEVICE);
	if (!sglen) {
		dev_err(dev, "cannot map dma memory\n");
		goto out_map_sg;
	}
	/* Configure DMA items */
	i_blk = 0;
	for_each_sg(fa->zdma->sgt.sgl, sg, fa->zdma->sgt.nents, i) {
		if (i_blk < fa->n_shots && i == zfad_block[i_blk].first_nent) {
			/*
			 * FIXME if we trust our configuration, dev_mem_off is
			 * useless in multishot
			 */
			dev_mem_off = zfad_block[i_blk].dev_mem_off;

			i_blk++; /* index the next block */
			if (unlikely(i_blk > fa->n_shots)) {
				dev_err(dev, "DMA map out of block\n");
				BUG();
			}
		}

		/* Prepare DMA item */
		spec_data->items[i].start_addr = dev_mem_off;
		spec_data->items[i].dma_addr_l = sg_dma_address(sg) & 0xFFFFFFFF;
		spec_data->items[i].dma_addr_h = (uint64_t)sg_dma_address(sg) >> 32;
		spec_data->items[i].dma_len = sg_dma_len(sg);
		dev_mem_off += spec_data->items[i].dma_len;
		if (!sg_is_last(sg)) {/* more transfers */
			/* uint64_t so it works on 32 and 64 bit */
			tmp = spec_data->dma_list_item;
			tmp += (sizeof(struct fa_dma_item) * (i + 1));
			spec_data->items[i].next_addr_l = ((uint64_t)tmp) & 0xFFFFFFFF;
			spec_data->items[i].next_addr_h = ((uint64_t)tmp) >> 32;
			spec_data->items[i].attribute = 0x1;	/* more items */
		} else {
			spec_data->items[i].attribute = 0x0;	/* last item */
		}
		pr_debug("configure DMA item %d "
			"(addr: 0x%llx len: %d)(dev off: 0x%x)"
			"(next item: 0x%x)\n",
			i, (long long)sg_dma_address(sg),
			sg_dma_len(sg), dev_mem_off, spec_data->items[i].next_addr_l);

		/* The first item is written on the device */
		if (i == 0) {
			fa_writel(fa, spec_data->fa_dma_base,
				  &fa_spec_regs[ZFA_DMA_ADDR],
					    spec_data->items[i].start_addr);
			fa_writel(fa, spec_data->fa_dma_base,
				  &fa_spec_regs[ZFA_DMA_ADDR_L],
					    spec_data->items[i].dma_addr_l);
			fa_writel(fa, spec_data->fa_dma_base,
				  &fa_spec_regs[ZFA_DMA_ADDR_H],
					    spec_data->items[i].dma_addr_h);
			fa_writel(fa, spec_data->fa_dma_base,
				  &fa_spec_regs[ZFA_DMA_LEN],
					    spec_data->items[i].dma_len);
			fa_writel(fa, spec_data->fa_dma_base,
				  &fa_spec_regs[ZFA_DMA_NEXT_L],
					    spec_data->items[i].next_addr_l);
			fa_writel(fa, spec_data->fa_dma_base,
				  &fa_spec_regs[ZFA_DMA_NEXT_H],
					    spec_data->items[i].next_addr_h);
			/* Set that there is a next item */
			fa_writel(fa, spec_data->fa_dma_base,
				  &fa_spec_regs[ZFA_DMA_BR_LAST],
					    spec_data->items[i].attribute);
		}
	}
	/* Start DMA transfer */
	fa_writel(fa, spec_data->fa_dma_base,
			&fa_spec_regs[ZFA_DMA_CTL_START], 1);
	return 0;

out_map_sg:
	dma_unmap_single(fa->fmc->hwdev, spec_data->dma_list_item, size,
			 DMA_TO_DEVICE);
out_map_single:
	kfree(spec_data->items);
out_alloc_item:
	zio_dma_free_sg(fa->zdma);
	return err;
}

void fa_spec_dma_done(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	unsigned int size;
	struct fa_spec_data *spec_data = fa->carrier_data;

	size = sizeof(struct fa_dma_item) * fa->zdma->sgt.nents;
	dma_unmap_sg(fa->fmc->hwdev, fa->zdma->sgt.sgl, fa->zdma->sgt.nents,
			     DMA_FROM_DEVICE);
	dma_unmap_single(fa->fmc->hwdev, spec_data->dma_list_item, size,
			 DMA_TO_DEVICE);
	kfree(spec_data->items);
	zio_dma_free_sg(fa->zdma);
	spec_data->items = NULL;
	spec_data->dma_list_item = 0;
}

void fa_spec_dma_error(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	struct fa_spec_data *spec_data = fa->carrier_data;
	uint32_t val;

	fa_spec_dma_done(cset);
	val = fa_readl(fa, spec_data->fa_dma_base,
			&fa_spec_regs[ZFA_DMA_STA]);
	if (val)
		dev_err(&fa->fmc->dev,
			"DMA error (status 0x%x). All acquisition lost\n", val);
}
