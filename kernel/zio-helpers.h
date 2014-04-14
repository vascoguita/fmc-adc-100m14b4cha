/*
 * Copyright CERN 2014
 * Author: Federico Vaga <federico.vaga@gmail.com>
 *
 * handle DMA mapping
 */

#ifndef ZIO_HELPERS_H_
#define ZIO_HELPERS_H_

#include <linux/zio.h>
#include <linux/scatterlist.h>

/*
 * It describe a zio block to be mapped with sg
 * @block: is the block to map
 * @first_nent: it tells the index of the first DMA transfer corresponding to
 *              the start of this block
 * @dev_mem_off: device memory offset where retrieve data for this block
 */
struct zio_blocks_sg {
	struct zio_block *block;
	unsigned int first_nent;
	unsigned long dev_mem_off;
};

/*
 * it describes the DMA sg mapping
 * @hwdev: the low level driver which will do DMA
 * @sg_blocks: one or more blocks to map
 * @n_blocks: number of blocks to map
 * @sgt: scatter gather table
 */
struct zio_dma_sg {
	struct device *hwdev;
	struct zio_blocks_sg *sg_blocks;
	unsigned int n_blocks;
	struct sg_table sgt;
};

/*
 * It describe the current sg item
 * @blk_index: current block index
 * @page_index: current page index
 * @is_first_nent_block: it tells if this description point to the first page
 *                       transfer for current block
 */
struct zio_dma_sg_desc {
	struct scatterlist *sg;
	unsigned int blk_index;
	unsigned int page_index;
	int is_first_nent_block;
};

extern struct zio_dma_sg *zio_dma_alloc_sg(struct device *hwdev,
					   struct zio_block **blocks,
					   unsigned int n_blocks,
					   gfp_t gfp);
extern void zio_dma_free_sg(struct zio_dma_sg *zdma);

#endif /* ZIO_HELPERS_H_ */
