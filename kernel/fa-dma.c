// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright CERN 2012-2019
 * Author: Federico Vaga <federico.vaga@cern.ch>
 */

#include <linux/errno.h>
#ifdef CONFIG_FMC_ADC_SVEC
#include "vmebus.h"
#endif

#include "fmc-adc-100m14b4cha.h"

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
	int i, size;
	uint32_t *ptr;

	/* CPU may be little endian, VME is big endian */
	if (__get_endian() == LITTLE_ENDIAN) {
		ptr = buffer;
		/* swap samples and trig timetag all seen as 32bits words */
		size = byte_length/4;
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


static uint32_t zfad_dev_mem_offset(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	uint32_t dev_mem_off, trg_pos, pre_samp;
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
	dev_mem_off = trg_pos - (pre_samp * cset->ssize * nchan);
	dev_dbg(fa->msgdev,
		"Trigger @ 0x%08x, pre_samp %i, offset 0x%08x\n",
		trg_pos, pre_samp, dev_mem_off);

	return dev_mem_off;
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
	vme->am         = VME_A24_USER_DATA_SCT;
	/*vme->am         = VME_A24_USER_MBLT;*/
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


static void zfad_dma_context_exit(struct zio_cset *cset,
				  struct zfad_block *zfad_block)
{
	struct fa_dev *fa = cset->zdev->priv_d;

	if (fa_is_flag_set(fa, FMC_ADC_SVEC)) {
		__endianness(zfad_block->block->datalen,
			     zfad_block->block->data);

		kfree(zfad_block->dma_ctx);
	}
}


/**
 * It initialize the DMA context for the given block transfer
 */
static int zfad_dma_context_init(struct zio_cset *cset,
				 struct zfad_block *zfad_block)
{
#ifdef CONFIG_FMC_ADC_SVEC
	struct fa_dev *fa = cset->zdev->priv_d;

	if (fa_is_flag_set(fa, FMC_ADC_SVEC))) {
		struct fa_svec_data *svec_data = fa->carrier_data;
		unsigned long vme_addr;
		struct vme_dma *desc;

		desc = kmalloc(sizeof(struct vme_dma), GFP_ATOMIC);
		if (!desc)
			return -ENOMEM;

		if (zfad_block == cset->interleave->priv_d) {
			/*
			 * Only for the first block:
			 * write the data address in the ddr_addr register: this
			 * address has been computed after ACQ_END by looking to the
			 * trigger position see fa-irq.c::irq_acq_end.
			 * Be careful: the SVEC HW version expects an address of 32bits word
			 * therefore mem-offset in byte is translated into 32bit word
			 */
			fa_writel(fa, svec_data->fa_dma_ddr_addr,
				  &fa_svec_regfield[FA_DMA_DDR_ADDR],
				  zfad_block->dev_mem_off/4);
		}

		zfad_block->dma_ctx = desc;
		vme_addr = svec_data->vme_base + svec_data->fa_dma_ddr_data;
		build_dma_desc(desc, vme_addr,
			       zfad_block->block->data,
			       zfad_block->block->datalen);
	}
#endif

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
static void zfad_dma_complete(void *arg)
{
	struct zfad_block *zfad_block = arg;
	struct zio_cset *cset = zfad_block->cset;
	struct fa_dev *fa = cset->zdev->priv_d;

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
		dma_release_channel(zfad_block->tx->chan);
		zfad_dma_done(cset);
	}
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
	unsigned int nr_pages, sg_mapped;
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


	max_segment_size = dma_get_max_seg_size(dchan->device->dev);
	max_segment_size &= PAGE_MASK; /* to make alloc_table happy */
	err = fa->sg_alloc_table_from_pages(&zfad_block->sgt, pages, nr_pages,
					    offset_in_page(zfad_block->block->data),
					    zfad_block->block->datalen,
					    max_segment_size, GFP_KERNEL);
	if (unlikely(err))
		goto err_sgt;

	sg_mapped = dma_map_sg(&fa->pdev->dev,
			       zfad_block->sgt.sgl,
			       zfad_block->sgt.nents,
			       DMA_DEV_TO_MEM);
	if (sg_mapped <= 0) {
		err = sg_mapped;
		goto err_map;
	}

	/* Prepare the DMA transmisison */
	tx = dmaengine_prep_slave_sg_ctx(dchan, zfad_block->sgt.sgl, sg_mapped,
					 DMA_DEV_TO_MEM, 0, zfad_block->dma_ctx);
	if (!tx) {
		dev_err(&cset->head.dev,
			"Failed to prepare dmaengine transfer descriptor\n");
		return -EBUSY;
	}
	tx->callback = zfad_dma_complete;
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


/**
 * It maps the ZIO blocks with an sg table, then it starts the DMA transfer
 * from the ADC to the host memory.
 *
 * @param cset
 */
static int zfad_dma_start(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	struct zfad_block *zfad_block = cset->interleave->priv_d;
	struct dma_chan *dchan;
	struct dma_slave_config sconfig;
	dma_cap_mask_t dma_mask;
	unsigned int data_offset;
	int err, i, dma_dev_id;
	struct resource *r;

	r = platform_get_resource(fa->pdev, IORESOURCE_DMA, ADC_DMA);
	if (!r) {
		dev_err(&fa->pdev->dev, "Can't set find DMA channel\n");
		return -ENODEV;
	}
	dma_dev_id = r->start;

	err = fa_fsm_wait_state(fa, FA100M14B4C_STATE_IDLE, 10);
	if (err) {
		dev_warn(fa->msgdev,
			 "Can't start DMA on the last acquisition, "
			 "State Machine is not IDLE\n");
		return err;
	}

	/*
	 * Disable all triggers to prevent fires between
	 * different DMA transfers required for multi-shots
	 */
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFAT_CFG_SRC], 0);

	dev_dbg(fa->msgdev, "Start DMA transfer\n");
	dma_cap_zero(dma_mask);
	dma_cap_set(DMA_SLAVE, dma_mask);
	dma_cap_set(DMA_PRIVATE, dma_mask);
	dchan = dma_request_channel(dma_mask, fa_dmaengine_filter, &dma_dev_id);
	if (!dchan) {
		err = -ENODEV;
		goto err;
	}

	memset(&sconfig, 0, sizeof(sconfig));
	sconfig.direction = DMA_DEV_TO_MEM;
	sconfig.src_addr_width = 8; /* 2 bytes for each channel (4) */
	data_offset = (cset->interleave->current_ctrl->ssize * cset->ti->nsamples) + FA_TRIG_TIMETAG_BYTES;
	for (i = 0; i < fa->n_shots; ++i) {
		/*
		 * TODO
		 * Let's see what to do with SVEC. SVEC need to set
		 * the DMA_DDR_ADDR in hardware before starting the DMA
		 * (it configures the DMA window).
		 * In single shot is not a big deal, in multishot we may have
		 * to issue_pending many time since we can't update the
		 * DMA_DDR_ADDR for each block submitted to the dma engine.
		 * But sice the blocks are contigous, perhaps there is no need
		 * because the address of shot 2 is exactly after shot 1
		 */
		if (!fa_is_flag_set(fa, FMC_ADC_SVEC) && fa->n_shots == 1)
			sconfig.src_addr = zfad_dev_mem_offset(cset);
		else
			sconfig.src_addr = i * data_offset;
		err = dmaengine_slave_config(dchan, &sconfig);
		if (err)
			goto err_config;
		err = zfad_dma_prep_slave_sg(dchan, cset, &zfad_block[i]);
		if (err)
			goto err_prep;
	}

	fa->transfers_left = fa->n_shots;
	dma_async_issue_pending(dchan);

	return 0;

err_prep:
err_config:
	dmaengine_terminate_all(dchan);
	dma_release_channel(dchan);
err:
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFAT_CFG_SRC],
		  cset->ti->zattr_set.ext_zattr[FA100M14B4C_TATTR_SRC].value);
	dev_err(fa->msgdev, "Failed to run a DMA transfer (%d)\n", err);
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
	if (unlikely((tg->sec_high >> 8) != 0xACCE55))
		return -EINVAL;

	/* resize the datalen, by removing the trigger tstamp */
	block->datalen = block->datalen - FA_TRIG_TIMETAG_BYTES;

	memcpy(timetag, tg, sizeof(*timetag));
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
		if (err)
			memset(&timetag, 0, sizeof(timetag));
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
