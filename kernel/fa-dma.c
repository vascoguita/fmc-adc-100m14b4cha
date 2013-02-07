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

#include <linux/zio.h>
#include <linux/zio-buffer.h>
#include <linux/zio-trigger.h>

#include "spec.h"
#include "fmc-adc.h"


/*
 * zfat_calculate_nents
 *
 * It calculates the number of necessary nents
 */
static int zfat_calculate_nents(struct zio_block *block)
{
	int bytesleft = block->datalen;
	void *bufp = block->data;
	int mapbytes;
	int nents = 0;

	while (bytesleft) {
		nents++;
		if (bytesleft < (PAGE_SIZE - offset_in_page(bufp)))
			mapbytes = bytesleft;
		else
			mapbytes = PAGE_SIZE - offset_in_page(bufp);
		bufp += mapbytes;
		bytesleft -= mapbytes;
	}
	return nents;
}

/*
 * zfad_setup_dma_scatter
 *
 * Initialize each element of the scatter list
 */
static void zfad_setup_dma_scatter(struct fa_dev *fa, struct zfad_block *zfad_block)
{
	struct scatterlist *sg;
	int bytesleft = zfad_block->block->datalen;
	void *bufp = zfad_block->block->data;
	int mapbytes;
	int i;

	dev_dbg(&fa->zdev->head.dev, "Setup dma scatterlist for %zu bytes",
			zfad_block->block->datalen);
	for_each_sg(zfad_block->sgt.sgl, sg, zfad_block->sgt.nents, i) {
		/*
		 * If there are less bytes left than what fits
		 * in the current page (plus page alignment offset)
		 * we just feed in this, else we stuff in as much
		 * as we can.
		 */
		if (bytesleft < (PAGE_SIZE - offset_in_page(bufp)))
			mapbytes = bytesleft;
		else
			mapbytes = PAGE_SIZE - offset_in_page(bufp);
		/* Map the page */
		if (is_vmalloc_addr(bufp))
			sg_set_page(sg, vmalloc_to_page(bufp), mapbytes,
				    offset_in_page(bufp));
		else
			sg_set_buf(sg, bufp, mapbytes);
		/* Configure next values */
		bufp += mapbytes;
		bytesleft -= mapbytes;
		pr_debug("sg item (%p(+0x%lx), len:%d, left:%d)\n",
			virt_to_page(bufp), offset_in_page(bufp), mapbytes, bytesleft);
	}

	WARN(bytesleft, "Something Wrong");
}

/*
 * zfad_map_dma
 * @cset: channel set
 * @zfad_block: the block to map through DMA
 *
 * Map a scatter/gather table for the DMA transfer from the FMC-ADC.
 * The DMA controller can store a single item, but more then one transfer
 * could be necessary
 */
int zfad_map_dma(struct zio_cset *cset, struct zfad_block *zfad_block)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	struct zio_block *block = zfad_block->block;
	struct scatterlist *sg;
	struct dma_item *items;
	uint32_t dev_mem_ptr;
	unsigned int i, pages, sglen, size;
	dma_addr_t tmp;
	int err;

	pages = zfat_calculate_nents(block);
	if (!pages) {
		dev_err(&cset->head.dev, "No pages to transfer (datalen=%li)\n",
			block->datalen);
		return -EINVAL;
	}
	dev_dbg(&cset->head.dev, "using %d pages for transfer\n", pages);

	/* Create sglists for the transfers */
	err = sg_alloc_table(&zfad_block->sgt, pages, GFP_ATOMIC);
	if (err) {
		dev_err(&cset->head.dev, "cannot allocate sg table\n");
		goto out;
	}

	/* Limited to 32-bit (kernel limit) */
	size = sizeof(struct dma_item) * zfad_block->sgt.nents;
	items = kzalloc(size, GFP_ATOMIC);
	if (!items) {
		dev_err(fa->fmc->hwdev, "cannot allocate coherent dma memory\n");
		goto out_mem;
	}
	zfad_block->items = items;
	zfad_block->dma_list_item = dma_map_single(fa->fmc->hwdev, items, size,
						   DMA_FROM_DEVICE);
	if (!zfad_block->dma_list_item) {
		goto out_free;
	}
	/* Setup the scatter list for the provided block */
	zfad_setup_dma_scatter(fa, zfad_block);
	/* Map DMA buffers */
	sglen = dma_map_sg(fa->fmc->hwdev, zfad_block->sgt.sgl, zfad_block->sgt.nents,
			   DMA_FROM_DEVICE);
	if (!sglen) {
		dev_err(fa->fmc->hwdev, "cannot map dma memory\n");
		goto out_map;
	}

	/* Configure DMA items */
	dev_mem_ptr = zfad_block->dev_mem_ptr;
	for_each_sg(zfad_block->sgt.sgl, sg, zfad_block->sgt.nents, i) {
		dev_dbg(&cset->head.dev, "configure DMA item %d"
			"(addr: 0x%llx len: %d)(dev off: 0x%x)\n",
			i, sg_dma_address(sg), sg_dma_len(sg),
			fa->cur_dev_mem);
		/* Prepare DMA item */
		items[i].start_addr = dev_mem_ptr;
		items[i].dma_addr_l = sg_dma_address(sg) & 0xFFFFFFFF;
		items[i].dma_addr_h = sg_dma_address(sg) >> 32;
		items[i].dma_len = sg_dma_len(sg);
		dev_mem_ptr += items[i].dma_len;
		if (!sg_is_last(sg)) {/* more transfers */
			/* uint64_t so it works on 32 and 64 bit */
			tmp = zfad_block->dma_list_item;
			tmp += (sizeof(struct dma_item) * ( i+ 1 ));
			items[i].next_addr_l = ((uint64_t)tmp) & 0xFFFFFFFF;
			items[i].next_addr_h = ((uint64_t)tmp) >> 32;
			items[i].attribute = 0x1;	/* more items */
		} else {
			items[i].attribute = 0x0;	/* last item */
		}
		/* The first item is written on the device */
		if (i == 0) {
			zfa_common_conf_set(fa, ZFA_DMA_ADDR,
					    items[i].start_addr);
			zfa_common_conf_set(fa, ZFA_DMA_ADDR_L,
					    items[i].dma_addr_l);
			zfa_common_conf_set(fa, ZFA_DMA_ADDR_H,
					    items[i].dma_addr_h);
			zfa_common_conf_set(fa, ZFA_DMA_LEN,
					    items[i].dma_len);
			zfa_common_conf_set(fa, ZFA_DMA_NEXT_L,
					    items[i].next_addr_l);
			zfa_common_conf_set(fa, ZFA_DMA_NEXT_H,
					    items[i].next_addr_h);
			/* Set that there is a next item */
			zfa_common_conf_set(fa, ZFA_DMA_BR_LAST,
					    items[i].attribute);
		}
	}

	return 0;

out_map:
	dma_unmap_single(fa->fmc->hwdev, zfad_block->dma_list_item, size,
		       DMA_FROM_DEVICE);
out_free:
	kfree(zfad_block->items);
out_mem:
	sg_free_table(&zfad_block->sgt);
out:
	return -ENOMEM;
}


/*
 * zfad_unmap_dma
 * @cset: channel set
 * @zfad_block: the block to map through DMA
 *
 * It unmaps a blocks
 */
void zfad_unmap_dma(struct zio_cset *cset, struct zfad_block *zfad_block)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	unsigned int size;

	dev_dbg(fa->fmc->hwdev, "unmap DMA\n");
	size = sizeof(struct dma_item) * zfad_block->sgt.nents;
	dma_unmap_single(fa->fmc->hwdev, zfad_block->dma_list_item, size,
			       DMA_FROM_DEVICE);
	dma_unmap_sg(fa->fmc->hwdev, zfad_block->sgt.sgl, zfad_block->sgt.nents,
		     DMA_FROM_DEVICE);

	kfree(zfad_block->items);
	zfad_block->items = NULL;
	zfad_block->dma_list_item = 0;
	sg_free_table(&zfad_block->sgt);
}
