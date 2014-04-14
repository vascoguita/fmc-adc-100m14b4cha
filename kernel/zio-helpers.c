/*
 * Copyright CERN 2014
 * Author: Federico Vaga <federico.vaga@gmail.com>
 *
 * handle DMA mapping
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>

#include "zio-helpers.h"

static int zio_calculate_nents(struct zio_blocks_sg *sg_blocks,
			       unsigned int n_blocks)
{
	int i, bytesleft;
	void *bufp;
	int mapbytes;
	int nents = 0;

	for (i = 0; i < n_blocks; ++i) {
		bytesleft = sg_blocks[i].block->datalen;
		bufp = sg_blocks[i].block->data;
		sg_blocks[i].first_nent = nents;
		while (bytesleft) {
			nents++;
			if (bytesleft < (PAGE_SIZE - offset_in_page(bufp)))
				mapbytes = bytesleft;
			else
				mapbytes = PAGE_SIZE - offset_in_page(bufp);
			bufp += mapbytes;
			bytesleft -= mapbytes;
		}
	}
	return nents;
}

static void zio_dma_setup_scatter(struct zio_dma_sg *zdma)
{
	struct scatterlist *sg;
	int bytesleft = 0;
	void *bufp = NULL;
	int mapbytes;
	int i, i_blk;

	i_blk = 0;
	for_each_sg(zdma->sgt.sgl, sg, zdma->sgt.nents, i) {
		if (i_blk < zdma->n_blocks && i == zdma->sg_blocks[i_blk].first_nent) {
			WARN(bytesleft, "unmapped byte in block %i\n",
			     i_blk - 1);
			/*
			 * Configure the DMA for a new block, reset index and
			 * data pointer
			 */
			bytesleft = zdma->sg_blocks[i_blk].block->datalen;
			bufp = zdma->sg_blocks[i_blk].block->data;

			i_blk++; /* index the next block */
			if (unlikely(i_blk > zdma->n_blocks))
				BUG();
		}

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
			 virt_to_page(bufp), offset_in_page(bufp),
			 mapbytes, bytesleft);
	}
}

/*
 * zio_alloc_scatterlist
 * @chan: zio channel associated to this scatterlist
 * @hwdev: low level device responsible of the DMA
 * @blocks: array of zio_block to transfer
 * @n_blocks: number of blocks to transfer
 * @gfp: gfp flags for memory allocation
 *
 * The function allocates and initializes a scatterlist ready for DMA
 * transfer
 */
struct zio_dma_sg *zio_dma_alloc_sg(struct device *hwdev,
				    struct zio_block **blocks, /* FIXME to array */
				    unsigned int n_blocks, gfp_t gfp)
{
	struct zio_dma_sg *zdma;
	unsigned int i, pages;
	int err;

	if (unlikely(!chan || !hwdev || !blocks || !n_blocks))
		return -EINVAL;

	/*
	 * Allocate a new zio_dma_sg structure that will contains all necessary
	 * information for DMA
	 */
	zdma = kzalloc(sizeof(struct zio_dma_sg), gfp);
	if (!zdma)
		return ERR_PTR(-ENOMEM);

	/* Allocate a new list of blocks with sg information */
	zdma->sg_blocks = kzalloc(sizeof(struct zio_blocks_sg) * n_blocks, gfp);
	if (!zdma->sg_blocks) {
		err = -ENOMEM;
		goto out;
	}

	/* fill the zio_dma_sg structure */
	zdma->hwdev = hwdev;
	zdma->n_blocks = n_blocks;
	for (i = 0; i < n_blocks; ++i)
		zdma->sg_blocks[i].block = blocks[i];


	/* calculate the number of necessary pages to transfer */
	pages = zio_calculate_nents(zdma->sg_blocks, zdma->n_blocks);
	if (!pages) {
		err = -EINVAL;
		goto out_calc_nents;
	}

	/* Create sglists for the transfers */
	err = sg_alloc_table(&zdma->sgt, pages, gfp);
	if (err)
		goto out_alloc_sg;

	/* Setup the scatter list for the provided block */
	zio_dma_setup_scatter(zdma);

	return zdma;

out_alloc_sg:
out_calc_nents:
	kfree(zdma->sg_blocks);
out:
	kfree(zdma);

	return ERR_PTR(err);
}
EXPORT_SYMBOL(zio_dma_alloc_sg);


/*
 * zio_free_scatterlist
 * @zdma: zio DMA transfer descriptor
 *
 * It releases resources
 */
void zio_dma_free_sg(struct zio_dma_sg *zdma)
{
	kfree(zdma->sg_blocks);
	kfree(zdma);
}
EXPORT_SYMBOL(zio_dma_free_sg);
