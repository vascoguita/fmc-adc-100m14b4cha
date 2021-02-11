// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright CERN 2012-2019
 * Author: Federico Vaga <federico.vaga@cern.ch>
 */

#include <linux/errno.h>
#ifdef CONFIG_FMC_ADC_SVEC
#include "vmebus.h"
#endif

#include "fmc-adc-100m14b4cha-private.h"

/* Endianess */
#ifndef LITTLE_ENDIAN
#define LITTLE_ENDIAN 0
#endif

#ifndef BIG_ENDIAN
#define BIG_ENDIAN 1
#endif

static void zfad_dma_done(struct zio_cset *cset);


static int __get_endian(void)
{
	int i = 1;
	char *p = (char *)&i;

	if (p[0] == 1)
		return LITTLE_ENDIAN;
	else
		return BIG_ENDIAN;
}


/**
 * Fix endianess from big to host endianess (32bit)
 */
static void __endianness(unsigned int byte_length, void *buffer)
{
	/* CPU may be little endian, VME is big endian */
	if (__get_endian() == LITTLE_ENDIAN) {
		/* swap samples and trig timetag all seen as 32bits words */
		int i;
		int size = byte_length / 4;
		uint32_t *ptr = buffer;

		for (i = 0; i < size; ++i, ++ptr)
			*ptr = __be32_to_cpu(*ptr);
	}
}

struct zfad_timetag {
	uint32_t sec_low;
	uint32_t sec_high;
	uint32_t ticks;
	uint32_t status;
};

/**
 * It matches a valid DMA channel
 */
static bool fa_dmaengine_filter(struct dma_chan *dchan, void *arg)
{
	struct dma_device *ddev = dchan->device;
	int dev_id = (*((int *)arg) >> 16) & 0xFFFF;
	int chan_id = *((int *)arg) & 0xFFFF;

	return ddev->dev_id == dev_id && dchan->chan_id == chan_id;
}

static bool fa_dmaengine_filter_svec(struct dma_chan *dchan, void *arg)
{
	struct fa_dev *fa = arg;
	struct device *device_ref;

        device_ref = fa->pdev->dev.parent->parent->parent->parent->parent->parent;

	return (dchan->device->dev == device_ref);
}

int fa_dma_request_channel(struct fa_dev *fa)
{
	dma_cap_mask_t dma_mask;
	struct resource *r;
	int dma_dev_id;

        if (fa_is_flag_set(fa, FMC_ADC_SVEC))
		return 0;

	r = platform_get_resource(fa->pdev, IORESOURCE_DMA, ADC_DMA);
	if (!r) {
		dev_err(&fa->pdev->dev, "Can't set find DMA channel\n");
		return -ENODEV;
	}
	dma_dev_id = r->start;

	dma_cap_zero(dma_mask);
	dma_cap_set(DMA_SLAVE, dma_mask);
	dma_cap_set(DMA_PRIVATE, dma_mask);
	fa->dchan = dma_request_channel(dma_mask,
					fa_dmaengine_filter, &dma_dev_id);
	if (!fa->dchan)
		return -ENODEV;
	return 0;
}

/**
 * Remove this function and its use when VME Bridge will support virtual
 * DMA channels. Then we will request a channel only once at probe like
 * on SPEC
 */
static int fa_dma_request_channel_svec(struct fa_dev *fa)
{
	dma_cap_mask_t dma_mask;

        if (!fa_is_flag_set(fa, FMC_ADC_SVEC))
		return 0;

	dma_cap_zero(dma_mask);
	dma_cap_set(DMA_SLAVE, dma_mask);
	dma_cap_set(DMA_PRIVATE, dma_mask);
	fa->dchan = dma_request_channel(dma_mask,
					fa_dmaengine_filter_svec, fa);
	if (!fa->dchan)
		return -ENODEV;
	return 0;
}

static void __fa_dma_release_channel(struct fa_dev *fa)
{
	dma_release_channel(fa->dchan);
}

void fa_dma_release_channel(struct fa_dev *fa)
{
	if (fa_is_flag_set(fa, FMC_ADC_SVEC))
		return;
	__fa_dma_release_channel(fa);
}

/**
 * Remove this function and its use when VME Bridge will support virtual
 * DMA channels. Then we will request a channel only once at probe like
 * on SPEC
 */
static void fa_dma_release_channel_svec(struct fa_dev *fa)
{
	if (fa_is_flag_set(fa, FMC_ADC_SVEC))
		__fa_dma_release_channel(fa);
}


static uint32_t fa_ddr_offset_single(struct fa_dev *fa)
{
	struct zio_cset *cset = fa->zdev->cset;
	uint32_t off, trg_pos, pre_samp;
	int nchan = FA100M14B4C_NCHAN;
	struct zio_control *ctrl = cset->chan[nchan].current_ctrl;

	/* get pre-samples from the current control (interleave chan) */
	pre_samp = ctrl->attr_trigger.std_val[ZIO_ATTR_TRIG_PRE_SAMP];
	/* Get trigger position in DDR */
	trg_pos = fa_readl(fa, fa->fa_adc_csr_base,
			   &zfad_regs[ZFAT_POS]);
	/*
	 * compute mem offset (in bytes): pre-samp is converted to
	 * bytes
	 */
	off = trg_pos - (pre_samp * cset->ssize * nchan);
	dev_dbg(fa->msgdev,
		"Trigger @ 0x%08x, pre_samp %i, offset 0x%08x\n",
		trg_pos, pre_samp, off);

	return off;
}

static uint32_t fa_ddr_offset_multi(struct fa_dev *fa, uint32_t shot_n)
{
	struct zio_cset *cset = fa->zdev->cset;
	uint32_t off;

        off = cset->interleave->current_ctrl->ssize * cset->ti->nsamples;
	off += FA_TRIG_TIMETAG_BYTES;
	off *= shot_n;

	return off;
}

static uint32_t fa_ddr_offset(struct fa_dev *fa, uint32_t shot_n)
{
	WARN(fa->n_shots == 1 && shot_n != 0,
	     "Inconsistent shot number %d\n", shot_n);
	if (fa->n_shots == 1) {
		return fa_ddr_offset_single(fa);
	} else {
		return fa_ddr_offset_multi(fa, shot_n);
	}
}

static unsigned int zfad_block_n_pages(struct zio_block *block)
{
	unsigned int nr_pages;
	long kaddr = (long)block->data;

	nr_pages = ((kaddr & ~PAGE_MASK) + block->datalen + ~PAGE_MASK);
	nr_pages >>= PAGE_SHIFT;

	return nr_pages;
}


#ifdef CONFIG_FMC_ADC_SVEC

#define SVEC_FUNC_NR 1 /* HARD coded in SVEC */

static inline struct vme_dev *fa_to_vme_dev(struct fa_dev *fa)
{
	return to_vme_dev(fa->pdev->dev.parent->parent->parent->parent);
}

static unsigned long fa_ddr_data_vme_addr(struct fa_dev *fa)
{
	struct fmc_adc_platform_data *data = fa->pdev->dev.platform_data;
	struct vme_dev *vdev = fa_to_vme_dev(fa);
	unsigned long addr;

	if (WARN(vdev->map[SVEC_FUNC_NR].kernel_va == NULL,
		 "Invalid VME function\n"))
		return ~0; /* invalid address, we will see VME errors */

	WARN(data->vme_dma_offset == 0, "Invalid DDR DMA offset");
	addr = vdev->map[SVEC_FUNC_NR].vme_addrl;
	addr += data->vme_dma_offset;

	return addr;
}

static void *fa_ddr_addr_reg_off(struct fa_dev *fa)
{
	struct fmc_adc_platform_data *data = fa->pdev->dev.platform_data;
	struct vme_dev *vdev = fa_to_vme_dev(fa);
	void *addr;

	if (WARN(vdev->map[SVEC_FUNC_NR].kernel_va == NULL,
		 "Invalid VME function\n"))
		return NULL; /* invalid address, we will see VME errors */

	addr = vdev->map[SVEC_FUNC_NR].kernel_va;
	addr += data->vme_reg_offset;

	return addr;
}

#define VME_NO_ADDR_INCREMENT 1

/* FIXME: move to include again */
#ifndef lower_32_bits
#define lower_32_bits(n) ((u32)(n))
#endif /* lower_32_bits */

static void build_dma_desc(struct vme_dma *desc, unsigned long vme_addr,
			void *addr_dest, ssize_t len)
{
	struct vme_dma_attr *vme;
	struct vme_dma_attr *pci;

	memset(desc, 0, sizeof(struct vme_dma));

	vme = &desc->src;
	pci = &desc->dst;

	desc->dir	= VME_DMA_FROM_DEVICE;
	desc->length    = len;
	desc->novmeinc  = VME_NO_ADDR_INCREMENT;

	desc->ctrl.pci_block_size   = VME_DMA_BSIZE_4096;
	desc->ctrl.pci_backoff_time = VME_DMA_BACKOFF_0;
	desc->ctrl.vme_block_size   = VME_DMA_BSIZE_4096;
	desc->ctrl.vme_backoff_time = VME_DMA_BACKOFF_0;

	vme->data_width = VME_D32;
	vme->am         = VME_A24_USER_MBLT;
	vme->addru	= upper_32_bits(vme_addr);
	vme->addrl	= lower_32_bits(vme_addr);

	pci->addru = upper_32_bits((unsigned long)addr_dest);
	pci->addrl = lower_32_bits((unsigned long)addr_dest);
}
#endif


static int zfad_dma_block_to_pages(struct page **pages, unsigned int nr_pages,
				   struct zio_block *block)
{
	int i;
	void *data = (void *) block->data;

	if (is_vmalloc_addr(data)) {
		for (i = 0; i < nr_pages; ++i)
			pages[i] = vmalloc_to_page(data + PAGE_SIZE * i);
	} else {
		for (i = 0; i < nr_pages; ++i)
			pages[i] = virt_to_page(data + PAGE_SIZE * i);
	}

	return 0;
}


static void zfad_dma_context_exit_svec(struct zio_cset *cset,
				       struct zfad_block *zfad_block)
{

	__endianness(zfad_block->block->datalen,
		     zfad_block->block->data);

	kfree(zfad_block->dma_ctx);
}

static void zfad_dma_context_exit(struct zio_cset *cset,
				  struct zfad_block *zfad_block)
{
	struct fa_dev *fa = cset->zdev->priv_d;

	if (fa_is_flag_set(fa, FMC_ADC_SVEC))
		zfad_dma_context_exit_svec(cset, zfad_block);
}

static int zfad_dma_context_init_svec(struct zio_cset *cset,
				      struct zfad_block *zfad_block)
{
#ifdef CONFIG_FMC_ADC_SVEC
	struct fa_dev *fa = cset->zdev->priv_d;
	struct vme_dma *desc;
	void *addr;

	dev_dbg(&fa->pdev->dev, "SVEC build DMA context\n");
	zfad_block->dma_ctx = NULL;

	addr = fa_ddr_addr_reg_off(fa);
	if (!addr)
		return -ENODEV;

	desc = kmalloc(sizeof(struct vme_dma), GFP_ATOMIC);
	if (!desc)
		return -ENOMEM;

	/*
	 * For the first block of each shot:
	 * write the start address to the ddr_reg register: this
	 * address has been computed after ACQ_END by looking to the
	 * trigger position see fa-irq.c::irq_acq_end.
	 */
	fa_iowrite(fa, zfad_block->sconfig.src_addr, addr);

	zfad_block->dma_ctx = desc;
	build_dma_desc(desc, fa_ddr_data_vme_addr(fa),
		       zfad_block->block->data,
		       zfad_block->block->datalen);
	return 0;
#else
	return 0;
#endif
}

/**
 * It initialize the DMA context for the given block transfer
 */
static int zfad_dma_context_init(struct zio_cset *cset,
				 struct zfad_block *zfad_block)
{
	struct fa_dev *fa = cset->zdev->priv_d;

	if (fa_is_flag_set(fa, FMC_ADC_SVEC))
		return zfad_dma_context_init_svec(cset, zfad_block);
	return 0;
}


/**
 * The proper function from the DMA engine does not allow us to set
 * the context, but we need it (e.g. VME bus)
 */
static inline struct dma_async_tx_descriptor *dmaengine_prep_slave_sg_ctx(
		struct dma_chan *chan,
		struct scatterlist *sgl, unsigned int sg_len,
		enum dma_transfer_direction dir,
		unsigned long flags, void *ctx)
{
	if (!(chan->device && chan->device->device_prep_slave_sg))
		return NULL;

	return chan->device->device_prep_slave_sg(chan, sgl, sg_len,
						  DMA_DEV_TO_MEM, 0, ctx);
}


/**
 * zfad_dma_complete
 * @arg: data block instance
 *
 * It handles the data transfer completion of a block
 */
static void fa_dma_complete(void *arg,
			    const struct dmaengine_result *result)
{
	struct zfad_block *zfad_block = arg;
	struct zio_cset *cset = zfad_block->cset;
	struct fa_dev *fa = cset->zdev->priv_d;

	if (result->result != DMA_TRANS_NOERROR)
		dev_err(&fa->pdev->dev, "DMA failed %d\n", result->result);

	/* Release DMA resources */
	dma_unmap_sg(&fa->pdev->dev,
		     zfad_block->sgt.sgl,
		     zfad_block->sgt.nents,
		     DMA_DEV_TO_MEM);
	sg_free_table(&zfad_block->sgt);

	/* Clean/fix the context */
	zfad_dma_context_exit(cset, zfad_block);

	/* Complete the full acquisition when the last transfer is over */
	--fa->transfers_left;
	if (!fa->transfers_left) {
		fa_dma_release_channel_svec(fa);
		zfad_dma_done(cset);
	}

	complete(&zfad_block->shot_done);
}


/**
 * zfad_dma_prep_slave_sg
 * @dchan: DMA channel to use
 * @cset: ZIO channel set that owns the data
 * @zfad_block: data block instance to transfer
 *
 * It prepare the scatterlist for the block transfer and it submits it
 * to the dma engine.
 */
static int zfad_dma_prep_slave_sg(struct dma_chan *dchan,
				  struct zio_cset *cset,
				  struct zfad_block *zfad_block)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	struct dma_async_tx_descriptor *tx;
	struct page **pages;
	unsigned int nr_pages;
	int sg_mapped;
	size_t max_segment_size;
	int err;

	/* prepare the context for the block transfer */
	zfad_dma_context_init(cset, zfad_block);

	/* Convert buffer to pages */
	nr_pages = zfad_block_n_pages(zfad_block->block);
	pages = kcalloc(nr_pages, sizeof(struct page *), GFP_KERNEL);
	if (!pages) {
		err = -ENOMEM;
		goto err_alloc_pages;
	}
	err = zfad_dma_block_to_pages(pages, nr_pages, zfad_block->block);
	if (err)
		goto err_to_pages;

	max_segment_size = min(zfad_block->block->datalen + PAGE_SIZE, /* PAGE aligned */
			       (size_t)dma_get_max_seg_size(dchan->device->dev));
	/* Find something that fits in the [SW-]IOMMU */
	do {
		dev_dbg(&fa->pdev->dev, "DMA max segment %ld\n",
			max_segment_size);
		max_segment_size &= PAGE_MASK; /* to make alloc_table happy */
		err = fa->sg_alloc_table_from_pages(&zfad_block->sgt, pages,
						    nr_pages,
						    offset_in_page(zfad_block->block->data),
						    zfad_block->block->datalen,
						    max_segment_size,
						    GFP_KERNEL);
		if (unlikely(err))
			goto err_sgt;

		sg_mapped = dma_map_sg(&fa->pdev->dev,
				       zfad_block->sgt.sgl,
				       zfad_block->sgt.nents,
				       DMA_DEV_TO_MEM);
		if (sg_mapped <= 0) {
			sg_free_table(&zfad_block->sgt);
			err = sg_mapped ? sg_mapped : -ENOMEM;
			max_segment_size /= 2;
		} else {
			err = 0;
			break;
		}
	} while (max_segment_size >= PAGE_SIZE);
	if (err)
		goto err_map;
	/* Prepare the DMA transmisison */
	tx = dmaengine_prep_slave_sg_ctx(dchan, zfad_block->sgt.sgl, sg_mapped,
					 DMA_DEV_TO_MEM, 0, zfad_block->dma_ctx);
	if (!tx) {
		dev_err(&cset->head.dev,
			"Failed to prepare dmaengine transfer descriptor\n");
		return -EBUSY;
	}

	tx->callback_result = fa_dma_complete;
	tx->callback_param = (void *)zfad_block;
	zfad_block->tx = tx;

	/* Submit the DMA transmission to the DMA engine */
	zfad_block->cookie = dmaengine_submit(tx);
	if (zfad_block->cookie < 0) {
		err = zfad_block->cookie;
		goto err_submit;
	}

	/* we do not need the pages anymore */
	kfree(pages);

	return 0;

err_submit:
	dma_unmap_sg(&fa->pdev->dev,
		     zfad_block->sgt.sgl,
		     zfad_block->sgt.nents,
		     DMA_DEV_TO_MEM);
err_map:
	sg_free_table(&zfad_block->sgt);
err_sgt:
err_to_pages:
	kfree(pages);
err_alloc_pages:
	return err;
}

static int fa_dmaengine_slave_config(struct fa_dev *fa,
				     struct dma_slave_config *sconfig)
{
	/*
	 * For SVEC we must set the DMA context, there is not slave config
	 */
	if (fa_is_flag_set(fa, FMC_ADC_SVEC))
		return 0;
	return dmaengine_slave_config(fa->dchan, sconfig);
}


/*
 * In multishot mode we must wait for the previous
 * transfer to complete because:
 * - of the MBLT prefetch, so we need to do separate DMA
 *   transfers;
 * - of the SVEC DMA DDR offset register that can't be
 *   overwritten while running a DMA transfer
 */
static int fa_dma_shot_wait_svec(struct fa_dev *fa,
				 struct zfad_block *zfad_block,
				 unsigned int timeout_ms)
{
	int err;

        if (fa->n_shots == 1)
		return 0;

	err = wait_for_completion_interruptible_timeout(&zfad_block->shot_done,
							msecs_to_jiffies(timeout_ms));

	/* Check the status of our transfer */
	if (err <= 0) {
		/* timeout elapsed or signal interruption */
		dev_err(&fa->pdev->dev, "%s\n", (err == 0) ? "DMA timeout elapsed" :
			"DMA interrupted by a signal");
		return -EINVAL;
	}

        return 0;
}

static int fa_dma_start_svec(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	struct zfad_block *zfad_block = cset->interleave->priv_d;
	int err, i;

        err = fa_dma_request_channel_svec(fa);
	if (err)
		return err;

	fa->transfers_left = fa->n_shots;
	for (i = 0; i < fa->n_shots; ++i) {
		zfad_block[i].sconfig.direction = DMA_DEV_TO_MEM;
		zfad_block[i].sconfig.src_addr_width = 8; /* 2 bytes for each channel (4) */
		zfad_block[i].sconfig.src_addr = fa_ddr_offset(fa, i);
		err = fa_dmaengine_slave_config(fa, &zfad_block[i].sconfig);
		if (err)
			goto err_config;
		err = zfad_dma_prep_slave_sg(fa->dchan, cset, &zfad_block[i]);
		if (err)
			goto err_prep;
		init_completion(&zfad_block[i].shot_done);
		dma_async_issue_pending(fa->dchan);

		err = fa_dma_shot_wait_svec(fa, &zfad_block[i], 60000);
		if (err)
			goto err_wait;
	}

	return 0;

err_wait:
	dmaengine_terminate_all(fa->dchan);
	dma_unmap_sg(&fa->pdev->dev,
		     zfad_block[i].sgt.sgl,
		     zfad_block[i].sgt.nents,
		     DMA_DEV_TO_MEM);
	sg_free_table(&zfad_block[i].sgt);

	/* Clean/fix the context */
	zfad_dma_context_exit(cset, &zfad_block[i]);
err_prep:
err_config:
	fa_dma_release_channel_svec(fa);
	return err;
}

static int fa_dma_start_spec(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	struct zfad_block *zfad_block = cset->interleave->priv_d;
	int err, i;

	for (i = 0; i < fa->n_shots; ++i) {
		zfad_block[i].sconfig.direction = DMA_DEV_TO_MEM;
		zfad_block[i].sconfig.src_addr_width = 8; /* 2 bytes for each channel (4) */
		zfad_block[i].sconfig.src_addr = fa_ddr_offset(fa, i);
		err = fa_dmaengine_slave_config(fa, &zfad_block[i].sconfig);
		if (err)
			goto err_config;
		err = zfad_dma_prep_slave_sg(fa->dchan, cset, &zfad_block[i]);
		if (err)
			goto err_prep;
	}
	fa->transfers_left = fa->n_shots;
	dma_async_issue_pending(fa->dchan);

	return 0;

err_prep:
err_config:
	dmaengine_terminate_all(fa->dchan);
	while (--i >= 0) {
		dma_unmap_sg(&fa->pdev->dev,
			     zfad_block[i].sgt.sgl,
			     zfad_block[i].sgt.nents,
			     DMA_DEV_TO_MEM);
		sg_free_table(&zfad_block[i].sgt);
	}
	return err;

}

/**
 * It maps the ZIO blocks with an sg table, then it starts the DMA transfer
 * from the ADC to the host memory.
 *
 * @param cset
 */
static int zfad_dma_start(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	int err;

	err = fa_fsm_wait_state(fa, FA100M14B4C_STATE_IDLE, 10);
	if (err) {
		dev_warn(&fa->pdev->dev,
			 "Can't start DMA on the last acquisition, "
			 "State Machine is not IDLE\n");
		return err;
	}

        dev_dbg(&fa->pdev->dev,
		"Start DMA transfer for %i shots of %i samples\n",
		fa->n_shots, cset->ti->nsamples);

	/*
	 * Disable all triggers to prevent fires between
	 * different DMA transfers required for multi-shots
	 */
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFAT_CFG_SRC], 0);
        if (fa_is_flag_set(fa, FMC_ADC_SVEC))
		err = fa_dma_start_svec(cset);
	else
		err = fa_dma_start_spec(cset);
	if (err)
		goto err_start;

        return 0;

err_start:
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFAT_CFG_SRC],
		  cset->ti->zattr_set.ext_zattr[FA100M14B4C_TATTR_SRC].value);
	dev_err(&fa->pdev->dev, "Failed to run a DMA transfer (%d)\n", err);
	return err;
}

static void zfad_tstamp_start_get(struct fa_dev *fa,
				  struct zio_timestamp *ztstamp)
{
	ztstamp->secs = fa_readl(fa, fa->fa_utc_base,
				&zfad_regs[ZFA_UTC_ACQ_START_SECONDS_U]);
	ztstamp->secs <<= 32;
	ztstamp->secs |= fa_readl(fa, fa->fa_utc_base,
				  &zfad_regs[ZFA_UTC_ACQ_START_SECONDS_L]);
	ztstamp->ticks = fa_readl(fa, fa->fa_utc_base,
				 &zfad_regs[ZFA_UTC_ACQ_START_COARSE]);
	ztstamp->bins = 0;
}

static int zfad_block_timetag_extract(struct zio_block *block,
				      struct zfad_timetag *timetag)
{
	struct zfad_timetag *tg;

	tg = block->data + block->datalen - FA_TRIG_TIMETAG_BYTES;
	/* resize the datalen, by removing the trigger tstamp */
	block->datalen = block->datalen - FA_TRIG_TIMETAG_BYTES;
	memcpy(timetag, tg, sizeof(*timetag));
	if (unlikely((tg->sec_high >> 8) != 0xACCE55))
		return -EINVAL;
	return 0;
}

static void zfad_block_ctrl_tstamp_update(struct zio_block *block,
					  struct zfad_timetag *timetag)
{
	struct zio_control *ctrl = zio_get_ctrl(block);
	struct zio_timestamp *ztstamp = &ctrl->tstamp;

	ztstamp->secs = ((uint64_t)timetag->sec_high & 0xFF) << 32;
	ztstamp->secs |= timetag->sec_low;
	ztstamp->ticks = timetag->ticks;
	ztstamp->bins = 0;
}

static void zfad_block_ctrl_attr_update(struct zio_block *block,
					struct zfad_timetag *timetag,
					unsigned int seq_num)
{
	struct zio_control *ctrl = zio_get_ctrl(block);
	uint32_t *ext_val = ctrl->attr_channel.ext_val;

	ext_val[FA100M14B4C_TATTR_STA]= timetag->status;
	ctrl->seq_num = seq_num;
}


static void zfad_block_ctrl_tstamp_start_update(struct zio_block *block,
						struct zio_timestamp *ztstamp)
{
	struct zio_control *ctrl = zio_get_ctrl(block);
	uint32_t *ext_val;

	ext_val = ctrl->attr_channel.ext_val;
	ext_val[FA100M14B4C_DATTR_ACQ_START_S] = ztstamp->secs;
	ext_val[FA100M14B4C_DATTR_ACQ_START_C] = ztstamp->ticks;
	ext_val[FA100M14B4C_DATTR_ACQ_START_F] = ztstamp->bins;
}

static void zfad_curr_ctrl_sync(struct zio_cset *cset,
				struct zio_block *block)
{
	struct zio_channel *interleave = cset->interleave;
	struct zio_control *ctrl;

	if (WARN(!block, "Missing block\n"))
		return;

	ctrl = zio_get_ctrl(block);
	/* Sync the channel current control with the last ctrl block*/
	memcpy(&interleave->current_ctrl->tstamp,
		&ctrl->tstamp, sizeof(struct zio_timestamp));
	interleave->current_ctrl->seq_num = ctrl->seq_num;
}

/**
 * It completes a DMA transfer.
 * It tells to the ZIO framework that all blocks are done. Then, it re-enable
 * the trigger for the next acquisition. If the device is configured for
 * continuous acquisition, the function automatically start the next
 * acquisition
 *
 * @param cset
 */
static void zfad_dma_done(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	struct zfad_block *zfad_block = cset->interleave->priv_d;
	struct zio_ti *ti = cset->ti;
	struct zio_block *block = NULL;
	struct zio_timestamp ztstamp;
	int i;

	/*
	 * Lower CSET_HW_BUSY
	 */
	spin_lock(&cset->lock);
	cset->flags &= ~ZIO_CSET_HW_BUSY;
	spin_unlock(&cset->lock);

	/* for each shot, set the timetag of each ctrl block by reading the
	 * trig-timetag appended after the samples. Set also the acquisition
	 * start timetag on every blocks
	 */
	zfad_tstamp_start_get(fa, &ztstamp);
	for (i = 0; i < fa->n_shots; ++i) {
		struct zfad_timetag timetag;
		int err;

		block = zfad_block[i].block;
		err = zfad_block_timetag_extract(block, &timetag);
		if (err) {
			dev_err(&fa->pdev->dev,
				"Failed to extract Timetag from acquisition from shot %i :0x%x 0x%x 0x%x 0x%x\n",
				i + 1,
				timetag.sec_high, timetag.sec_low,
				timetag.ticks, timetag.status);

			memset(&timetag, 0, sizeof(timetag));
		}
		zfad_block_ctrl_tstamp_start_update(block, &ztstamp);
		zfad_block_ctrl_tstamp_update(block, &timetag);
		zfad_block_ctrl_attr_update(block, &timetag, i);
	}
	zfad_curr_ctrl_sync(cset, block);

	/*
	 * All DMA transfers done! Inform the trigger about this, so
	 * it can store blocks into the buffer
	 */
	dev_dbg(fa->msgdev, "%i blocks transfered\n", fa->n_shots);
	zio_trigger_data_done(cset);

	/* we can safely re-enable triggers */
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFAT_CFG_SRC],
		  ti->zattr_set.ext_zattr[FA100M14B4C_TATTR_SRC].value);

	if (fa->enable_auto_start) {
		/* Automatic start next acquisition */
		dev_dbg(fa->msgdev, "Automatic start\n");
		zfad_fsm_command(fa, FA100M14B4C_CMD_START);
	}
}


/**
 * It handles the error condition of a DMA transfer.
 * The function turn off the state machine by sending the STOP command
 *
 * @param cset
 */
static void zfad_dma_error(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;

	/*
	 * Lower CSET_HW_BUSY
	 */
	spin_lock(&cset->lock);
	cset->flags &= ~ZIO_CSET_HW_BUSY;
	spin_unlock(&cset->lock);

	zfad_fsm_command(fa, FA100M14B4C_CMD_STOP);
	fa->n_dma_err++;

	/* FIXME stop pending */
	//dmaengine_terminate_all();

	if (fa->n_fires == 0)
		dev_err(fa->msgdev,
			"DMA error occurs but no block was acquired\n");
}


/*
 * job executed within a work thread
 * Depending of the carrier the job slightly differs:
 * SVEC: dma_start() blocks till the the DMA ends
 *      (fully managed by the vmebus driver)
 * Therefore the DMA outcome can be processed immediately
 * SPEC: dma_start() launch the job an returns immediately.
 * An interrupt DMA_DONE or ERROR is expecting to signal the end
 *       of the DMA transaction
 * (See fa-spec-irq.c::fa-spec_irq_handler)
 */
void fa_irq_work(struct work_struct *work)
{
	struct fa_dev *fa = container_of(work, struct fa_dev, irq_work);
	struct zio_cset *cset = fa->zdev->cset;
	int res;

	/*
	 * This check is not crucial because the HW implements
	 * a solid state machine and acq-end can happens only after
	 * the execution of the n requested shots.
	 */
	fa->n_fires = fa->n_shots - fa_readl(fa, fa->fa_adc_csr_base,
					     &zfad_regs[ZFAT_SHOTS_REM]);

	if (fa->n_fires != fa->n_shots) {
		dev_err(fa->msgdev,
			"Expected %i trigger fires, but %i occurs\n",
			fa->n_shots, fa->n_fires);
	}

	res = zfad_dma_start(cset);
	if (res) {
		/* Stop acquisition on error */
		zfad_dma_error(cset);
	}
}
