// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright CERN 2012-2019
 * Author: Federico Vaga <federico.vaga@cern.ch>
 */

#include <linux/errno.h>
#include "fmc-adc-100m14b4cha.h"

struct zfad_timetag {
	uint32_t sec_low;
	uint32_t sec_high;
	uint32_t ticks;
	uint32_t status;
};

static int zfad_wait_idle(struct fa_dev *fa)
{
	uint32_t val = 0;
	int try = 5;

	/*
	 * All programmed triggers fire, so the acquisition is ended.
	 * If the state machine is _idle_ we can start the DMA transfer.
	 * If the state machine it is not idle, try again 5 times
	 */
	while (try-- && val != FA100M14B4C_STATE_IDLE) {
		/* udelay(2); */
		val = fa_readl(fa, fa->fa_adc_csr_base,
			       &zfad_regs[ZFA_STA_FSM]);
	}

	return val != FA100M14B4C_STATE_IDLE ? -EBUSY : 0;
}

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

/**
 * It maps the ZIO blocks with an sg table, then it starts the DMA transfer
 * from the ADC to the host memory.
 *
 * @param cset
 */
int zfad_dma_start(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	struct zfad_block *zfad_block = cset->interleave->priv_d;
	int err;

	err = zfad_wait_idle(fa);
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

	/* Fix dev_mem_addr in single-shot mode */
	if (fa->n_shots == 1)
		zfad_block[0].dev_mem_off = zfad_dev_mem_offset(cset);

	dev_dbg(fa->msgdev, "Start DMA transfer\n");
	err = fa->carrier_op->dma_start(cset);
	if (err)
		return err;

	return 0;
}

static void zfad_tstamp_start_get(struct fa_dev *fa,
				  struct zio_timestamp *ztstamp)
{
	ztstamp->secs = fa_readl(fa, fa->fa_utc_base,
				&zfad_regs[ZFA_UTC_ACQ_START_SECONDS]);
	ztstamp->ticks = fa_readl(fa, fa->fa_utc_base,
				 &zfad_regs[ZFA_UTC_ACQ_START_COARSE]);
	ztstamp->bins = fa_readl(fa, fa->fa_utc_base,
				&zfad_regs[ZFA_UTC_ACQ_START_FINE]);
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
void zfad_dma_done(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	struct zfad_block *zfad_block = cset->interleave->priv_d;
	struct zio_control *ctrl = NULL;
	struct zio_ti *ti = cset->ti;
	struct zio_block *block = NULL;
	struct zio_timestamp ztstamp;
	int i;

	fa->carrier_op->dma_done(cset);

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

	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFAT_CFG_SRC],
		  ti->zattr_set.ext_zattr[FA100M14B4C_TATTR_SRC].value);
}


/**
 * It handles the error condition of a DMA transfer.
 * The function turn off the state machine by sending the STOP command
 *
 * @param cset
 */
void zfad_dma_error(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;

	fa->carrier_op->dma_error(cset);

	zfad_fsm_command(fa, FA100M14B4C_CMD_STOP);
	fa->n_dma_err++;

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
	if (!res) {
		/*
		 * No error.
		 * If there is an IRQ DMA src to notify the ends of the DMA,
		 * leave the workqueue.
		 * dma_done will be proceed on DMA_END reception.
		 * Otherwhise call dma_done in sequence
		 */
		if (fa->irq_src & FA_IRQ_SRC_DMA)
			/*
			 * waiting for END_OF_DMA IRQ
			 * with the CSET_BUSY flag Raised
			 * The flag will be lowered by the irq_handler
			 * handling END_DMA
			 */
			goto end;

		zfad_dma_done(cset);
	}
	/*
	 * Lower CSET_HW_BUSY
	 */
	spin_lock(&cset->lock);
	cset->flags &= ~ZIO_CSET_HW_BUSY;
	spin_unlock(&cset->lock);

end:
	if (res) {
		/* Stop acquisition on error */
		zfad_dma_error(cset);
	} else if (fa->enable_auto_start) {
		/* Automatic start next acquisition */
		dev_dbg(fa->msgdev, "Automatic start\n");
		zfad_fsm_command(fa, FA100M14B4C_CMD_START);
	}
}
