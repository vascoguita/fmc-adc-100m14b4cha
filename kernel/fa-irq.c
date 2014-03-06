/*
 * Copyright CERN 2012
 * Author: Federico Vaga <federico.vaga@gmail.com>
 *
 * IRQ-related code
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/bitops.h>
#include <linux/spinlock.h>
#include <linux/io.h>

#include <linux/zio.h>
#include <linux/zio-trigger.h>

#include "fmc-adc.h"

/**
 * It maps the ZIO blocks with an sg table, then it starts the DMA transfer
 * from the ADC to the host memory.
 *
 * @param cset
 */
void zfad_dma_start(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	struct zio_channel *interleave = cset->interleave;
	struct zfad_block *zfad_block = interleave->priv_d;
	int err;

	/* Map ZIO block for DMA acquisition */
	err = zfad_map_dma(cset, zfad_block, fa->n_shots);
	if (err)
		return;

	/* Start DMA transefer */
	zfa_hardware_write(fa, ZFA_DMA_CTL_START, 1);
	dev_dbg(&fa->fmc->dev, "Start DMA transfer\n");
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
	struct zio_channel *interleave = cset->interleave;
	struct zfad_block *zfad_block = interleave->priv_d;
	struct zio_ti *ti = cset->ti;

	zfad_unmap_dma(cset, zfad_block);

	/*
	 * All DMA transfers done! Inform the trigger about this, so
	 * it can store blocks into the buffer
	 */
	zio_trigger_data_done(cset);
	dev_dbg(&fa->fmc->dev, "%i blocks transfered\n", fa->n_shots);

	/*
	 * we can safely re-enable triggers.
	 * Hardware trigger depends on the enable status
	 * of the trigger. Software trigger depends on the previous
	 * status taken form zio attributes (index 5 of extended one)
	 * If the user is using a software trigger, enable the software
	 * trigger.
	 */
	if (cset->trig == &zfat_type) {
		zfa_hardware_write(fa, ZFAT_CFG_HW_EN,
				    (ti->flags & ZIO_STATUS ? 0 : 1));
		zfa_hardware_write(fa, ZFAT_CFG_SW_EN,
				    ti->zattr_set.ext_zattr[5].value);
	} else {
		dev_dbg(&fa->fmc->dev, "Software acquisition over\n");
		zfa_hardware_write(fa, ZFAT_CFG_SW_EN, 1);
	}

	/* Automatic start next acquisition */
	if (fa->enable_auto_start) {
		dev_dbg(&fa->fmc->dev, "Automatic start\n");
		zfad_fsm_command(fa, ZFA_START);
	}
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
	struct zio_bi *bi = cset->interleave->bi;
	struct zfad_block *zfad_block = cset->interleave->priv_d;
	int i;
	uint32_t val;

	zfad_unmap_dma(cset, zfad_block);

	zfa_hardware_read(fa, ZFA_DMA_STA, &val);
	dev_err(&fa->fmc->dev,
		"DMA error (status 0x%x). All acquisition lost\n", val);
	zfad_fsm_command(fa, ZFA_STOP);
	fa->n_dma_err++;

	if (fa->n_fires == 0)
		dev_err(&fa->fmc->dev,
			"DMA error (status 0x%x) occurs but no block was acquired\n", val);

	/* Remove invalid blocks */
	for (i = 0; i < fa->n_shots; ++i) {
		bi->b_op->store_block(bi, zfad_block[i].block);
	}
	kfree(zfad_block);
	cset->interleave->priv_d = NULL;
}

/*
 * zfat_irq_acq_end
 * @fa: fmc-adc descriptor
 *
 * The ADC end the acquisition, so, if the state machine is idle, we can
 * retrieve data from the ADC DDR memory.
 */
void zfat_irq_acq_end(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	uint32_t val;
	int try = 5;

	dev_dbg(&fa->fmc->dev, "Acquisition done\n");
	if (fa->n_fires != fa->n_shots) {
		dev_err(&fa->fmc->dev,
			"Expected %i trigger fires, but %i occurs\n",
			fa->n_shots, fa->n_fires);
	}
	/*
	 * All programmed triggers fire, so the acquisition is ended.
	 * If the state machine is _idle_ we can start the DMA transfer.
	 * If the state machine it is not idle, try again 5 times
	 */
	do {
		zfa_hardware_read(fa, ZFA_STA_FSM, &val);
	} while (try-- && val != ZFA_STATE_IDLE);

	if (val != ZFA_STATE_IDLE) {
		/* we can't DMA if the state machine is not idle */
		dev_warn(&fa->fmc->dev,
			 "Can't start DMA on the last acquisition, "
			 "State Machine is not IDLE (status:%d)\n", val);
		zfad_fsm_command(fa, ZFA_STOP);
		return;
	}

	/*
	 * Disable all triggers to prevent fires between
	 * different DMA transfers required for multi-shots
	 */
	zfa_hardware_write(fa, ZFAT_CFG_HW_EN, 0);
	zfa_hardware_write(fa, ZFAT_CFG_SW_EN, 0);

	/* Start the DMA transfer */
	zfad_dma_start(cset);
}
