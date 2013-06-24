/*
 * Copyright CERN 2012
 * Author: Federico Vaga <federico.vaga@gmail.com>
 *
 * Driver for the mezzanine ADC for the SPEC
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/moduleparam.h>
#include <linux/time.h>
#include <linux/delay.h>

#include <linux/fmc.h>

#include <linux/zio.h>
#include <linux/zio-buffer.h>
#include <linux/zio-trigger.h>

#include "fmc-adc.h"

int enable_auto_start = 0;
static int enable_test_data = 0;

module_param(enable_test_data, int, 0444);


/*
 * zio device attributes
 */
static ZIO_ATTR_DEFINE_STD(ZIO_DEV, zfad_cset_std_zattr) = {
	ZIO_ATTR(zdev, ZIO_ATTR_NBITS, ZIO_RO_PERM, ZFA_SW_R_NOADDRES_NBIT, 14),
};

/*
 * In the following table, "ATTR_EXT" are extended attributes that
 * end up in the control structure (i.e. they are metadata for the
 * acquisition), while PARAM_EXT are parameters: sysfs files that
 * don't end up in the control structure of ZIO blocks
 */

static struct zio_attribute zfad_cset_ext_zattr[] = {
	/*
	 * sample-decimation
	 * ADC acquire always at the maximum sample rate, to make "slower"
	 * acquisition you can decimate samples. 0 is a forbidden value, 1
	 * for the maximum speed.
	 */
	ZIO_ATTR_EXT("sample-decimation", ZIO_RW_PERM, ZFAT_SR_DECI, 1),

	/*
	 * State machine commands
	 * 1: start
	 * 2: stop
	 */
	ZIO_ATTR_EXT("ch0-offset", ZIO_RW_PERM, ZFA_CH1_OFFSET, 0),
	ZIO_ATTR_EXT("ch1-offset", ZIO_RW_PERM, ZFA_CH2_OFFSET, 0),
	ZIO_ATTR_EXT("ch2-offset", ZIO_RW_PERM, ZFA_CH3_OFFSET, 0),
	ZIO_ATTR_EXT("ch3-offset", ZIO_RW_PERM, ZFA_CH4_OFFSET, 0),

	ZIO_ATTR_EXT("ch0-vref", ZIO_RW_PERM, ZFA_CH1_CTL_RANGE, 0),
	ZIO_ATTR_EXT("ch1-vref", ZIO_RW_PERM, ZFA_CH2_CTL_RANGE, 0),
	ZIO_ATTR_EXT("ch2-vref", ZIO_RW_PERM, ZFA_CH3_CTL_RANGE, 0),
	ZIO_ATTR_EXT("ch3-vref", ZIO_RW_PERM, ZFA_CH4_CTL_RANGE, 0),

	ZIO_ATTR_EXT("ch0-50ohm-term", ZIO_RW_PERM, ZFA_CH1_CTL_TERM, 0),
	ZIO_ATTR_EXT("ch1-50ohm-term", ZIO_RW_PERM, ZFA_CH2_CTL_TERM, 0),
	ZIO_ATTR_EXT("ch2-50ohm-term", ZIO_RW_PERM, ZFA_CH3_CTL_TERM, 0),
	ZIO_ATTR_EXT("ch3-50ohm-term", ZIO_RW_PERM, ZFA_CH4_CTL_TERM, 0),

	/* Parameters (not attributes) follow */

	ZIO_PARAM_EXT("fsm-command", ZIO_WO_PERM, ZFA_CTL_FMS_CMD, 0),
	/*
	 * Automatic start acquisition
	 * 1: enabled
	 * 0: disabled
	 */
	ZIO_PARAM_EXT("fsm-auto-start", ZIO_RW_PERM, ZFA_SW_R_NOADDERS_AUTO, 0),
	/*
	 * fsm - status of the state machine:
	 * 1: IDLE
	 * 2: PRE_TRIG
	 * 3: WAIT_TRIG
	 * 4: POST_TRIG
	 * 5: DECR_SHOT
	 * 7: Illegal
	 * */
	ZIO_PARAM_EXT("fsm-state", ZIO_RO_PERM, ZFA_STA_FSM, 0),
	/* last acquisition start time stamp */
	ZIO_PARAM_EXT("tstamp-acq-str-s", ZIO_RO_PERM,
			ZFA_UTC_ACQ_START_SECONDS, 0),
	ZIO_PARAM_EXT("tstamp-acq-str-t", ZIO_RO_PERM,
			ZFA_UTC_ACQ_START_COARSE, 0),
	ZIO_PARAM_EXT("tstamp-acq-str-b", ZIO_RO_PERM,
			ZFA_UTC_ACQ_START_FINE, 0),
	/* last acquisition end time stamp */
	ZIO_PARAM_EXT("tstamp-acq-end-s", ZIO_RO_PERM,
			ZFA_UTC_ACQ_END_SECONDS, 0),
	ZIO_PARAM_EXT("tstamp-acq-end-t", ZIO_RO_PERM,
			ZFA_UTC_ACQ_END_COARSE, 0),
	ZIO_PARAM_EXT("tstamp-acq-end-b", ZIO_RO_PERM,
			ZFA_UTC_ACQ_END_FINE, 0),
	/* last acquisition stop time stamp */
	ZIO_PARAM_EXT("tstamp-acq-stp-s", ZIO_RO_PERM,
			ZFA_UTC_ACQ_STOP_SECONDS, 0),
	ZIO_PARAM_EXT("tstamp-acq-stp-t", ZIO_RO_PERM,
			ZFA_UTC_ACQ_STOP_COARSE, 0),
	ZIO_PARAM_EXT("tstamp-acq-stp-b", ZIO_RO_PERM,
			ZFA_UTC_ACQ_STOP_FINE, 0),
	/* Reset all channel offset */
	ZIO_PARAM_EXT("rst-ch-offset", ZIO_WO_PERM, ZFA_CTL_DAC_CLR_N, 1),


};

#if 0 /* FIXME Unused until TLV control will be available */
static ZIO_ATTR_DEFINE_STD(ZIO_DEV, zfad_chan_std_zattr) = {
	/* the offset is complement 2 format */
	ZIO_ATTR(zdev, ZIO_ATTR_OFFSET, ZIO_RW_PERM, ZFA_CHx_OFFSET, 0),
	/*
	 * in-range
	 * 0x23 (35): 100mV range
	 * 0x11 (17): 1V range
	 * 0x45 (69): 10V range
	 * 0x00 (0): Open input
	 */
	ZIO_ATTR(zdev, ZIO_ATTR_VREFTYPE, ZIO_RW_PERM, ZFA_CHx_CTL_RANGE, 0x11),
};
#endif

static struct zio_attribute zfad_chan_ext_zattr[] = {
	/*ZIO_ATTR(zdev, "50ohm-termination", ZIO_RW_PERM, ZFA_CHx_CTL_TERM, 0x11),*/
	ZIO_PARAM_EXT("current-value", ZIO_RO_PERM, ZFA_CHx_STA, 0),
};

static struct zio_attribute zfad_dev_ext_zattr[] = {
	/* Get Mezzanine temperature from onewire */
	ZIO_PARAM_EXT("temperature", ZIO_RO_PERM, ZFA_SW_R_NOADDRES_TEMP, 0),
};

/* Prototypes */
static int zfad_apply_user_offset(struct fa_dev *fa, struct zio_channel *chan,
				  uint32_t usr_val);

/* Calculate correct index for channel from CHx indexes */
static inline int zfad_get_chx_index(unsigned long addr,
				     struct zio_channel *chan)
{
	int offset;

	/* (n_chan - 1) because of interleave */
	offset = ZFA_CHx_MULT  * ((chan->cset->n_chan - 1) - chan->index);

	return addr - offset;
}


/*
 * zfad_fsm_command
 * @fa: the fmc-adc descriptor
 * @command: the command to apply to FSM
 *
 * This function check if the command can be done and perform some
 * preliminary operation before
 */
int zfad_fsm_command(struct fa_dev *fa, uint32_t command)
{
	uint32_t val;

	if (command != ZFA_START && command != ZFA_STOP) {
		dev_err(fa->fmc->hwdev, "Invalid command\n");
		return -EINVAL;
	}

	/*
	 * When any command occurs we are ready to start a new acquisition, so
	 * we must abort any previous one. If it is STOP, we abort because we
	 * abort an acquisition. If it is START, we abort because if there was
	 * a previous start but the acquisition end interrupt doesn't occurs,
	 * START mean RESTART. If it is a clean START, the abort has not
	 * effects.
	 *
	 * This is done only if ADC is using its own trigger, otherwise it is
	 * not necessary.
	 *
	 * The case of fmc-adc-trg is optimized because is the most common
	 * case
	 */
	if (likely(fa->zdev->cset->trig == &zfat_type || command == ZFA_STOP))
		zio_trigger_abort_disable(fa->zdev->cset, 0);

	/* Reset counters */
	fa->n_shots = 0;
	fa->n_fires = 0;

	/* If START, check if we can start */
	if (command == ZFA_START) {
		/* Verify that SerDes PLL is lockes */
		zfa_hardware_read(fa, ZFA_STA_SERDES_PLL, &val);
		if (!val) {
			dev_err(fa->fmc->hwdev,
			"Cannot start acquisition: SerDes PLL not locked\n");
			return -EBUSY;
		}
		/* Verify that SerDes is synched */
		zfa_hardware_read(fa, ZFA_STA_SERDES_SYNCED, &val);
		if (!val) {
			dev_err(fa->fmc->hwdev,
			"Cannot start acquisition: SerDes not synchronized\n");
			return -EBUSY;
		}

		/* Now we can arm the trigger for the incoming acquisition */
		zio_arm_trigger(fa->zdev->cset->ti);
		/*
		 *  FIXME maybe zio_arm_trigger() can return an error when it
		 * is not able to arm a trigger.
		 *
		 * It returns -EPERM, but the error can be -ENOMEM or -EINVAL
		 * from zfat_arm_trigger() or zfad_input_cset()
		 */
		if (!(fa->zdev->cset->ti->flags & ZIO_TI_ARMED)) {
			dev_err(fa->fmc->hwdev,
				"Trigger not armed, cannot start acquisition\n");
			return -EPERM;
		}

		dev_dbg(fa->fmc->hwdev, "FSM START Command, Enable interrupts\n");
		zfa_hardware_write(fa, ZFA_IRQ_MASK, ZFAT_ALL);
	} else {
		dev_dbg(fa->fmc->hwdev, "FSM STOP Command, Disable interrupts\n");
		zfa_hardware_write(fa, ZFA_IRQ_MASK, ZFAT_NONE);
	}

	zfa_hardware_write(fa, ZFA_CTL_FMS_CMD, command);
	return 0;
}


static const int zfad_hw_range[] = {
	[ZFA_RANGE_10V]   = 0x45,
	[ZFA_RANGE_1V]    = 0x11,
	[ZFA_RANGE_100mV] = 0x23,
	[ZFA_RANGE_OPEN]  = 0x00,
};

/*
 * zfad_convert_hw_range
 * @usr_val: range value
 *
 * return the enum associated to the range value
 */
static int zfad_convert_hw_range(uint32_t bitmask)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(zfad_hw_range); i++)
		if (zfad_hw_range[i] == bitmask)
			return i;
	return -EINVAL;
}

/* Temporarily, user values are the same as hardware values */
static int zfad_convert_user_range(uint32_t user_val)
{
	return zfad_convert_hw_range(user_val);
}

/*
 * zfad_set_range
 * @fa: the fmc-adc descriptor
 * @chan: the channel to calibrate
 * @usr_val: the volt range to set and calibrate
 *
 * When the input range changes, we must write new fixup values
 */
static int zfad_set_range(struct fa_dev *fa, struct zio_channel *chan,
			  int range)
{
	int i, offset, gain;

	dev_dbg(&chan->head.dev, "Set offset and gain for range %d\n",
		range);

	/* Actually set the range */
	i = zfad_get_chx_index(ZFA_CHx_CTL_RANGE, chan);
	zfa_hardware_write(fa, i, zfad_hw_range[range]);

	if (range == ZFA_RANGE_OPEN) {
		offset = FA_CAL_NO_OFFSET;
		gain = FA_CAL_NO_GAIN;
	} else {
		if (range < 0 || range > ARRAY_SIZE(fa->calib.adc)) {
			dev_info(&fa->fmc->dev, "Invalid range %i or ch %i\n",
				 range, chan->index);
			return -EINVAL;
		}
		offset = fa->calib.adc[range].offset[chan->index];
		gain = fa->calib.adc[range].gain[chan->index];
	}

	dev_dbg(&fa->fmc->dev, "Chan %i, range %i, offset 0x%x, gain 0x%x\n",
		chan->index, range, offset, gain);

	i = zfad_get_chx_index(ZFA_CHx_OFFSET, chan);
	zfa_hardware_write(fa, i, offset & 0xffff); /* prevent warning */
	i = zfad_get_chx_index(ZFA_CHx_GAIN, chan);
	zfa_hardware_write(fa, i, gain);

	/* recalculate user offset for the new range */
	zfad_apply_user_offset(fa, chan, fa->user_offset[chan->index]);

	return 0;
}


/*
 * zfad_apply_user_offset
 * @fa: the fmc-adc descriptor
 * @chan: the channel where apply offset
 * @usr_val: the offset value to apply, expressed as millivolts (-5000..5000)
 *
 * Apply user offset to the channel input. Before apply the user offset it must
 * be corrected with offset and gain calibration value. An open input does not
 * need any correction.
 */
static int zfad_apply_user_offset(struct fa_dev *fa, struct zio_channel *chan,
				  uint32_t usr_val)
{
	uint32_t range_reg;
	int32_t uval =  (int32_t)usr_val;
	int offset, gain, hwval, i, range;

	if (uval < -5000 || uval > 5000)
		return -EINVAL;

	i = zfad_get_chx_index(ZFA_CHx_CTL_RANGE, chan);
	zfa_hardware_read(fa, i, &range_reg);

	range = zfad_convert_hw_range(range_reg);
	if (range < 0)
		return range;

	if (range == ZFA_RANGE_OPEN) {
		offset = FA_CAL_NO_OFFSET;
		gain = FA_CAL_NO_GAIN;
	} else {
		offset = fa->calib.dac[range].offset[chan->index];
		gain = fa->calib.dac[range].gain[chan->index];
	}

	hwval = uval * 0x8000 / 5000;
	if (hwval == 0x8000) hwval = 0x7fff; /* -32768 .. 32767 */

	dev_dbg(&fa->fmc->dev, "Chan %i: uval %i -> 0x%04x "
		"(r %i, o 0x%x, g 0x%x)\n", chan->index, uval, hwval,
		range, offset, gain);

	hwval = ((hwval + offset) * gain) >> 15; /* signed */
	hwval += 0x8000; /* offset binary */
	if (hwval < 0)
		hwval = 0;
	if (hwval > 0xffff)
		hwval = 0xffff;

	dev_dbg(&chan->head.dev, "DAC offset calibration 0x%x\n", hwval);
	/* Apply calibrated offset to DAC */
	return fa_spi_xfer(fa, FA_SPI_SS_DAC(chan->index), 16, hwval, NULL);
}


/*
 * zfad_reset_offset
 * @fa: the fmc-adc descriptor
 *
 * Reset channel's offsets
 */
static void zfad_reset_offset(struct fa_dev *fa)
{
	int i;

	/* -1 because of interleaved channel */
	for (i = 0; i < fa->zdev->cset->n_chan - 1; ++i)
		zfad_apply_user_offset(fa, &fa->zdev->cset->chan[i], 0);
}


/*
 * zfad_conf_set
 *
 * set a value to a FMC-ADC registers
 */
static int zfad_conf_set(struct device *dev, struct zio_attribute *zattr,
			 uint32_t usr_val)
{
	struct fa_dev *fa = get_zfadc(dev);
	struct zio_channel *chan;
	int i, range, err, reg_index;

	dev_dbg(dev, "Writing %d in the sysfs attribute %s\n",
		usr_val, zattr->attr.attr.name);
	reg_index = zattr->id;
	i = fa->zdev->cset->n_chan - 1 ; /* -1 because of interleaved channel */

	switch (reg_index) {
		/*
		 * Most of the following "case" statements are simply
		 * error checking and ancillary operations.  The actual
		 * programming of hardware is done at the end of the
		 * switch, in the catch-all final zfa_hardware_write()
		 */

	case ZFA_SW_R_NOADDERS_AUTO:
		enable_auto_start = usr_val;
		return 0;
	/* FIXME temporary until TLV control */
	case ZFA_CH1_OFFSET:
		i--;
	case ZFA_CH2_OFFSET:
		i--;
	case ZFA_CH3_OFFSET:
		i--;
	case ZFA_CH4_OFFSET:
		i--;
		chan = to_zio_cset(dev)->chan + i;
		err = zfad_apply_user_offset(fa, chan, usr_val);
		if (err)
			return err;
		fa->user_offset[chan->index] = usr_val;
		return 0;

	case ZFA_CHx_OFFSET:
		chan = to_zio_chan(dev),
			err = zfad_apply_user_offset(fa, chan, usr_val);
		if (err)
			return err;
		fa->user_offset[chan->index] = usr_val;
		return 0;

	case ZFA_CTL_DAC_CLR_N:
		zfad_reset_offset(fa);
		return 0;
	case ZFAT_SR_DECI:
		if (usr_val == 0) {
			dev_err(dev, "max-sample-rate minimum value is 1\n");
			return -EINVAL;
		}
		break;
	/* FIXME temporary until TLV control */
	case ZFA_CH1_CTL_TERM:
	case ZFA_CH2_CTL_TERM:
	case ZFA_CH3_CTL_TERM:
	case ZFA_CH4_CTL_TERM:
	case ZFA_CHx_CTL_TERM:
		if (usr_val != 0 && usr_val != 1) {
			dev_err(dev, "50 Ohm termination can be activated (1) "
				     "or deactivated (0)\n");
			return -EINVAL;
		}
		break;

	/* FIXME temporary until TLV control */
	case ZFA_CH1_CTL_RANGE:
		i--;
	case ZFA_CH2_CTL_RANGE:
		i--;
	case ZFA_CH3_CTL_RANGE:
		i--;
	case ZFA_CH4_CTL_RANGE:
		i--;
		range = zfad_convert_user_range(usr_val);
		if (range < 0)
			return range;
		return zfad_set_range(fa, &to_zio_cset(dev)->chan[i], range);

	case ZFA_CHx_CTL_RANGE:
		range = zfad_convert_user_range(usr_val);
		if (range < 0)
			return range;
		return zfad_set_range(fa, to_zio_chan(dev), range);

	case ZFA_CHx_STA:
		reg_index = zfad_get_chx_index(reg_index, to_zio_chan(dev));
		break;
	case ZFA_CTL_FMS_CMD:
		return zfad_fsm_command(fa, usr_val);
	}

	return zfa_hardware_write(fa, reg_index, usr_val);
}

/*
 * zfad_info_get
 *
 * get a register value from FMC-ADC.
 */
static int zfad_info_get(struct device *dev, struct zio_attribute *zattr,
			 uint32_t *usr_val)
{
	struct fa_dev *fa = get_zfadc(dev);
	int i, reg_index;

	i = fa->zdev->cset->n_chan - 1 ; /* -1 because of interleaved channel */

	switch (zattr->id) {
	/* FIXME temporary until TLV control */
	case ZFA_CH1_OFFSET:
		i--;
	case ZFA_CH2_OFFSET:
		i--;
	case ZFA_CH3_OFFSET:
		i--;
	case ZFA_CH4_OFFSET:
		i--;
		*usr_val = fa->user_offset[i];
		return 0;

	case ZFA_CHx_OFFSET:
		*usr_val = fa->user_offset[to_zio_chan(dev)->index];
		return 0;

	case ZFA_SW_R_NOADDRES_NBIT:
	case ZFA_SW_R_NOADDERS_AUTO:
		/* ZIO automatically return the attribute value */
		return 0;
	case ZFA_SW_R_NOADDRES_TEMP:
		/*
		 * Onewire returns units of 1/16 degree. We return units
		 * of 1/1000 of a degree instead.
		 */
		*usr_val = fa_read_temp(fa, 0);
		*usr_val = (*usr_val * 1000 + 8) / 16;
		return 0;
	case ZFA_CHx_CTL_TERM:
	case ZFA_CHx_CTL_RANGE:
		reg_index = zfad_get_chx_index(zattr->id, to_zio_chan(dev));
		break;

	case ZFA_CHx_STA:
		reg_index = zfad_get_chx_index(zattr->id, to_zio_chan(dev));
		zfa_hardware_read(fa, reg_index, usr_val);
		i = (int16_t)(*usr_val); /* now signed integer */
		*usr_val = i;
		break;
	default:
		reg_index = zattr->id;
	}

	zfa_hardware_read(fa, reg_index, usr_val);
	dev_dbg(dev, "Reading %d from the sysfs attribute %s (%d)\n",
		*usr_val, zattr->attr.attr.name, reg_index);
	return 0;
}
static const struct zio_sysfs_operations zfad_s_op = {
	.conf_set = zfad_conf_set,
	.info_get = zfad_info_get,
};


/*
 * zfad_input_cset_software
 * @fa the adc instance to use
 * @cset channel set to acquire
 *
 * If the user is using the ADC trigger, then it can do a multi-shot
 * acquisition.
 * If the user is using a software trigger, it cannot do multi-shot.
 * The generic arm trigger used by software trigger returns a
 * zio_block. We must convert it into a zfad_block to perform DMA
 */
static int zfad_input_cset_software(struct fa_dev *fa, struct zio_cset *cset)
{
	struct zfad_block *tmp;
	int err;

	/* Check if device memory allows this acquisition */
	err = zfat_overflow_detection(cset->ti, ZFAT_POST, cset->ti->nsamples);
	if (err)
		return err;
	tmp = kzalloc(sizeof(struct zfad_block), GFP_ATOMIC);
	if (!tmp)
		return -ENOMEM;
	tmp->block = cset->interleave->active_block;
	cset->interleave->priv_d = tmp;
	tmp->dev_mem_ptr = 0; /* Always the first block */

	/* Configure post samples */
	zfa_hardware_write(fa, ZFAT_POST, cset->ti->nsamples);
	/* Start the acquisition */
	zfad_fsm_command(fa, ZFA_START);

	fa->n_shots = 1;
	/* Fire software trigger */
	zfa_hardware_write(fa, ZFAT_SW, 1);

	return -EAGAIN;
}


/*
 * zfad_input_cset
 * @cset: channel set to acquire
 *
 * Prepare the FMC-ADC for the acquisition.
 */
static int zfad_input_cset(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;

	dev_dbg(fa->fmc->hwdev, "Ready to acquire\n");
	/* ZIO should configure only the interleaved channel */
	if (!cset->interleave)
		return -EINVAL;
	/* nsamples can't be 0 */
	if (!cset->interleave->current_ctrl->nsamples) {
		dev_err(&cset->head.dev, "no post/pre-sample configured\n");
		return -EINVAL;
	}

	/* If not the fmc-adc-trg, then is a ZIO software trigger */
	if (unlikely(cset->trig != &zfat_type)) {
		return zfad_input_cset_software(fa, cset);
	}

	return -EAGAIN; /* data_done on DMA_DONE interrupt */
}

/*
 * zfad_stop_cset
 * @cset: channel set to stop
 *
 * Stop an acquisition, reset indexes and disable interrupts. This function
 * is useful only if the driver is using a software trigger.
 */
static void zfad_stop_cset(struct zio_cset *cset)
{

	struct fa_dev *fa = cset->zdev->priv_d;

	/* If the user is using a software trigger */
	if (cset->trig != &zfat_type) {
		/* Force the acquisition to stop */
		zfad_fsm_command(fa, ZFA_STOP);
		/* Release zfad_block */
		kfree(cset->interleave->priv_d);
		cset->interleave->priv_d = NULL;
		/* Clear active block */
		cset->interleave->active_block = NULL;
	}
}
/* * * * * * * * * * * * * IRQ functions handler * * * * * * * * * * * * * * */

/*
 * zfat_get_irq_status
 * @fa: adc descriptor
 * @irq_status: destination of irq status
 * @irq_multi: destination of irq multi
 *
 * Get irq and clear the register. To clear an interrupt we have to write 1
 * on the handled interrupt. We handle all interrupt so we clear all interrupts
 */
static void zfat_get_irq_status(struct fa_dev *fa,
				uint32_t *irq_status, uint32_t *irq_multi)
{

	/* Get current interrupts status */
	zfa_hardware_read(fa, ZFA_IRQ_SRC, irq_status);
	zfa_hardware_read(fa, ZFA_IRQ_MULTI, irq_multi);
	dev_dbg(fa->fmc->hwdev, "irq status = 0x%x multi = 0x%x\n",
			*irq_status, *irq_multi);

	/* Clear current interrupts status */
	zfa_hardware_write(fa, ZFA_IRQ_SRC, *irq_status);
	zfa_hardware_write(fa, ZFA_IRQ_MULTI, *irq_multi);
}


/**
 * It maps the ZIO blocks with an sg table, then it starts the DMA transfer
 * from the ADC to the host memory.
 *
 * @param cset
 */
static void zfad_dma_start(struct zio_cset *cset)
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
	dev_dbg(fa->fmc->hwdev, "Start DMA transfer\n");
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
	struct zio_channel *interleave = cset->interleave;
	struct zfad_block *zfad_block = interleave->priv_d;
	struct zio_ti *ti = cset->ti;

	zfad_unmap_dma(cset, zfad_block);

	/*
	 * All DMA transfers done! Inform the trigger about this, so
	 * it can store blocks into the buffer
	 */
	zio_trigger_data_done(cset);
	dev_dbg(fa->fmc->hwdev, "%i blocks transfered\n", fa->n_shots);

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
		dev_dbg(&cset->head.dev, "Software acquisition over");
		zfa_hardware_write(fa, ZFAT_CFG_SW_EN, 1);
	}

	/* Automatic start next acquisition */
	if (enable_auto_start) {
		dev_dbg(fa->fmc->hwdev, "Automatic start\n");
		zfad_fsm_command(fa, ZFA_START);
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
	struct zfad_block *zfad_block = cset->interleave->priv_d;
	uint32_t val;

	zfad_unmap_dma(cset, zfad_block);

	zfa_hardware_read(fa, ZFA_DMA_STA, &val);
	dev_err(fa->fmc->hwdev,
		"DMA error (status 0x%x). All acquisition lost\n", val);
	zfad_fsm_command(fa, ZFA_STOP);
	fa->n_dma_err++;
}


/*
 * zfat_get_time_stamp
 *
 * Get the last trigger time-stamp from device
 */
static void zfat_get_time_stamp(struct fa_dev *fa, struct zio_timestamp *ts)
{
	zfa_hardware_read(fa, ZFA_UTC_TRIG_SECONDS, (uint32_t *)&ts->secs);
	zfa_hardware_read(fa, ZFA_UTC_TRIG_COARSE, (uint32_t *)&ts->ticks);
	zfa_hardware_read(fa, ZFA_UTC_TRIG_FINE, (uint32_t *)&ts->bins);
}


/*
 * zfat_irq_trg_fire
 * @fa: fmc-adc descriptor
 *
 * Trigger fires. This function stores the time-stamp in the current_ctrl and
 * in the pre-allocated block. Then it increments the sequence number both in
 * current_ctrl and in the pre-allocated block.
 *
 * If the device is working in single-shot mode, once the trigger fire
 * interrupt occurs we must fix the dev_mem_addr of the block.
 */
static void zfat_irq_trg_fire(struct zio_cset *cset)
{
	struct zio_channel *interleave = cset->interleave;
	struct fa_dev *fa = cset->zdev->priv_d;
	struct zfad_block *zfad_block = interleave->priv_d;
	struct zio_control *ctrl;
	uint32_t fixed_mem_ptr, trg_pos, pre_samp;

	dev_dbg(fa->fmc->hwdev, "Trigger fire %i/%i\n",
		fa->n_fires + 1, fa->n_shots);
	if (fa->n_fires >= fa->n_shots) {
		WARN(1, "Invalid Fire, STOP acquisition");
		zfad_fsm_command(fa, ZFA_STOP);
		return;
	}

	/* Fix dev_mem_addr in single-shot mode */
	if (fa->n_shots == 1) {
		pre_samp = cset->trig->zattr_set
				.std_zattr[ZIO_ATTR_TRIG_PRE_SAMP].value;
		/* Get trigger position in DDR */
		zfa_hardware_read(fa, ZFAT_POS, &trg_pos);
		/* translate from sample count to memory offset */
		fixed_mem_ptr = (trg_pos - pre_samp) * cset->ssize;
		/* -1 because of interleaved channel */
		fixed_mem_ptr *= (cset->n_chan - 1);
		dev_dbg(fa->fmc->hwdev,
			"Trigger position 0x%x, bytes 0x%x\n",
			trg_pos, fixed_mem_ptr);

		zfad_block[fa->n_fires].dev_mem_ptr = fixed_mem_ptr;
	}

	/* Get control from pre-allocated block */
	ctrl = zio_get_ctrl(zfad_block[fa->n_fires].block);
	/* Update timestamp */
	zfat_get_time_stamp(fa, &interleave->current_ctrl->tstamp);
	memcpy(&ctrl->tstamp, &interleave->current_ctrl->tstamp,
	       sizeof(struct zio_timestamp));
	/* Update sequence number */
	interleave->current_ctrl->seq_num++;
	ctrl->seq_num = interleave->current_ctrl->seq_num;

	/* Count fire */
	fa->n_fires++;
}


/*
 * zfat_irq_acq_end
 * @fa: fmc-adc descriptor
 *
 * The ADC end the acquisition, so, if the state machine is idle, we can
 * retrieve data from the ADC DDR memory.
 */
static void zfat_irq_acq_end(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	uint32_t val = 0;
	int try = 5;

	dev_dbg(fa->fmc->hwdev, "Acquisition done\n");
	if (fa->n_fires != fa->n_shots) {
		dev_err(fa->fmc->hwdev,
			"Expected %i trigger fires, but %i occurs\n",
			fa->n_shots, fa->n_fires);
	}
	/*
	 * All programmed triggers fire, so the acquisition is ended.
	 * If the state machine is _idle_ we can start the DMA transfer.
	 * If the state machine it is not idle, try again 5 times
	 */
	while (try-- && val != ZFA_STATE_IDLE) {
		//udelay(2);
		zfa_hardware_read(fa, ZFA_STA_FSM, &val);
	}

	if (val != ZFA_STATE_IDLE) {
		/* we can't DMA if the state machine is not idle */
		dev_warn(fa->fmc->hwdev,
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


/*
 * zfad_irq
 * @irq:
 * @ptr: pointer to fmc_device
 *
 * The different irq status are handled in different if statement. At the end
 * of the if statement we don't call return because it is possible that there
 * are others irq to handle. The order of irq handlers is based on the
 * possibility to have many irq rised at the same time. It is possible that
 * ZFAT_TRG_FIRE and ZFAT_ACQ_END happens simultaneously, so we handle FIRE
 * before END. It should be impossible to have simultaneously both DMA
 * interrupts and acquisition interrupts active at the same time.
 */
static irqreturn_t zfad_irq(int irq, void *ptr)
{
	struct fmc_device *fmc = ptr;
	struct fa_dev *fa = fmc_get_drvdata(fmc);
	struct zio_cset *cset = fa->zdev->cset;
	uint32_t status, multi;
	int max_try = 10;

	/* irq to handle */
	zfat_get_irq_status(fa, &status, &multi);
	if (!status)
		return IRQ_NONE;

irq_handler:
	dev_dbg(fa->fmc->hwdev, "Handle ADC interrupts\n");
	if (unlikely((status & (ZFAT_DMA_DONE | ZFAT_DMA_ERR)) &&
	    (status & (ZFAT_TRG_FIRE | ZFAT_ACQ_END)))) {
		WARN(1, "Cannot handle trigger interrupt and DMA interrupt at "
			"the same time\n");
		/* Stop Acquisition, ADC it is not working properly */
		zfad_fsm_command(fa, ZFA_STOP);
		return IRQ_HANDLED;
	}
	/*
	 * It cannot happen that DMA_DONE is in the multi register.
	 * It should not happen ...
	 */
	if (status & ZFAT_DMA_DONE) {
		zfad_dma_done(cset);
	}
	if (unlikely((status | multi) & ZFAT_DMA_ERR)) {
		zfad_dma_error(cset);
	}

	/* Fire the trigger for each interrupt */
	if (status & ZFAT_TRG_FIRE)
		zfat_irq_trg_fire(cset);
	if (multi & ZFAT_TRG_FIRE)
		zfat_irq_trg_fire(cset);
	/*
	 * If it is too fast the ACQ_END interrupt can be in the
	 * multi register.
	 */
	if ((status | multi) & ZFAT_ACQ_END)
		zfat_irq_acq_end(cset);

	/* ack the irq */
	fa->fmc->op->irq_ack(fa->fmc);

	/*
	 * Read again the interrupt status. It can happen that an interrupt
	 * is raised while we was processing a previous interrupt. If there
	 * are new active interrupts, then handle them.
	 * This handler cannot monopolize the processor, so it check for
	 * new interrupts only 'max_try' times.
	 */
	zfat_get_irq_status(fa, &status, &multi);
	if ((status || multi) && --max_try)
		goto irq_handler;

	return IRQ_HANDLED;
}


/*
 * zfat_gpio_cfg
 *
 * GPIO configuration for FMC ADC. It configure only the interrupt GPIO
 */
struct fmc_gpio zfat_gpio_cfg[] = {
	{
		.gpio = FMC_GPIO_IRQ(0),
		.mode = GPIOF_DIR_IN,
		.irqmode = IRQF_TRIGGER_RISING,
	}
};


/* * * * * * * * * * * * * * * * Initialization * * * * * * * * * * * * * * */

/*
 * zfad_zio_probe
 * @zdev: the real zio device
 *
 * The device registration completes. Copy the calibration data from the
 * eeprom and initialize some registers
 */
static int zfad_zio_probe(struct zio_device *zdev)
{
	struct fa_dev *fa = zdev->priv_d;
	int err = 0, i, addr;

	dev_dbg(&zdev->head.dev, "%s:%d", __func__, __LINE__);
	/* Save also the pointer to the real zio_device */
	fa->zdev = zdev;

	/* Retrieve calibration data from the eeprom. FIXME: verify it */
	memcpy(&fa->calib, fa->fmc->eeprom + FA_CAL_OFFSET, sizeof(fa->calib));

	/* Configure GPIO for IRQ */
	fa->fmc->op->gpio_config(fa->fmc, zfat_gpio_cfg,
				 ARRAY_SIZE(zfat_gpio_cfg));
	/* Request IRQ */
	err = fa->fmc->op->irq_request(fa->fmc, zfad_irq, "fmc-adc-100m14b",
				       IRQF_SHARED);
	if (err)
		dev_err(fa->fmc->hwdev, "can't request irq %i (err %i)\n",
			fa->fmc->irq, err);

	/* Force stop FSM to prevent early trigger fire */
	zfa_hardware_write(fa, ZFA_CTL_FMS_CMD, ZFA_STOP);
	/* Initialize channels to use 1V range */
	for (i = 0; i < 4; ++i) {
		addr = zfad_get_chx_index(ZFA_CHx_CTL_RANGE,
					  &zdev->cset->chan[i]);
		zfa_hardware_write(fa, addr, ZFA_RANGE_1V);
		zfad_set_range(fa, &zdev->cset->chan[i], ZFA_RANGE_1V);
	}
	zfad_reset_offset(fa);

	/* Enable mezzanine clock */
	zfa_hardware_write(fa, ZFA_CTL_CLK_EN, 1);
	/* Set DMA to transfer data from device to host */
	zfa_hardware_write(fa, ZFA_DMA_BR_DIR, 0);
	/* Set decimation to minimum */
	zfa_hardware_write(fa, ZFAT_SR_DECI, 1);
	/* Set test data register */
	zfa_hardware_write(fa, ZFA_CTL_TEST_DATA_EN, enable_test_data);
	/* Set to single shot mode by default */
	zfa_hardware_write(fa, ZFAT_SHOTS_NB, 1);
	if (zdev->cset->ti->cset->trig == &zfat_type) {
		/* Select external trigger (index 0) */
		zfa_hardware_write(fa, ZFAT_CFG_HW_SEL, 1);
		zdev->cset->ti->zattr_set.ext_zattr[0].value = 1;
	} else {
		/* Enable Software trigger*/
		zfa_hardware_write(fa, ZFAT_CFG_SW_EN, 1);
		/* Disable Hardware trigger*/
		zfa_hardware_write(fa, ZFAT_CFG_HW_EN, 0);
	}

	/* Zero offsets and release the DAC clear */
	zfad_reset_offset(fa);
	zfa_hardware_write(fa, ZFA_CTL_DAC_CLR_N, 1);


	/* Set UTC seconds from the kernel seconds */
	zfa_hardware_write(fa, ZFA_UTC_SECONDS, get_seconds());

	return err;
}

/*
 * zfad_zio_remove
 * @zdev: the real zio device
 *
 * Release FMC interrupt handler
 */
static int zfad_zio_remove(struct zio_device *zdev)
{
	struct fa_dev *fa = zdev->priv_d;

	fa->fmc->op->irq_free(fa->fmc);
	return 0;
}


/* Device description */
static struct zio_channel zfad_chan_tmpl = {
	.zattr_set = {
		/* FIXME usable only when TLV control is available */
		/*.std_zattr = zfad_chan_std_zattr,*/
		.ext_zattr = zfad_chan_ext_zattr,
		.n_ext_attr = ARRAY_SIZE(zfad_chan_ext_zattr),
	},
};
static struct zio_cset zfad_cset[] = {
	{
		.raw_io = zfad_input_cset,
		.stop_io = zfad_stop_cset,
		.ssize = 2,
		.n_chan = 4,
		.chan_template = &zfad_chan_tmpl,
		.flags =  ZIO_CSET_TYPE_ANALOG |	/* is analog */
			  ZIO_DIR_INPUT |	/* is input */
			  ZIO_CSET_INTERLEAVE_ONLY,/* interleave only */
		.zattr_set = {
			.std_zattr = zfad_cset_std_zattr,
			.ext_zattr = zfad_cset_ext_zattr,
			.n_ext_attr = ARRAY_SIZE(zfad_cset_ext_zattr),
		},
	}
};
static struct zio_device zfad_tmpl = {
	.owner = THIS_MODULE,
	.s_op = &zfad_s_op,
	.flags = 0,
	.cset = zfad_cset,
	.n_cset = ARRAY_SIZE(zfad_cset),
	.zattr_set = {
		.ext_zattr = zfad_dev_ext_zattr,
		.n_ext_attr = ARRAY_SIZE(zfad_dev_ext_zattr),
	},
	/* This driver prefers its own trigger */
	.preferred_trigger = "adc-100m14b",
	.preferred_buffer = "vmalloc",
};


/* List of supported boards */
static const struct zio_device_id zfad_table[] = {
	{"adc-100m14b", &zfad_tmpl},
	{},
};

static struct zio_driver fa_zdrv = {
	.driver = {
		.name = "adc-100m14b",
		.owner = THIS_MODULE,
	},
	.id_table = zfad_table,
	.probe = zfad_zio_probe,
	.remove = zfad_zio_remove,
};


/*
 * fa_zio_unregister
 *
 * It is a simple wrapper invoked by module_init to register this zio driver
 */
int fa_zio_register(void)
{
	return zio_register_driver(&fa_zdrv);
}


/*
 * fa_zio_unregister
 *
 * It is a simple wrapper invoked by module_exit to unregister this zio driver
 */
void fa_zio_unregister(void)
{
	zio_unregister_driver(&fa_zdrv);
}


/*
 * fa_zio_init
 *
 * It checks if we can register this device.  If it is possibile, the function
 * registers both device and trigger. The FMC probe invokes this function.
 */
int fa_zio_init(struct fa_dev *fa)
{
	struct device *hwdev = fa->fmc->hwdev;
	uint32_t val;
	int err;

	/* Check if hardware supports 64-bit DMA */
	if(dma_set_mask(hwdev, DMA_BIT_MASK(64))) {
		dev_err(hwdev, "64-bit DMA addressing not available, try 32\n");
		/* Check if hardware supports 32-bit DMA */
		if(dma_set_mask(hwdev, DMA_BIT_MASK(32))) {
			dev_err(hwdev, "32-bit DMA addressing not available\n");
			return -EINVAL;
		}
	}
	/* Wait 50ms, so device has time to calibrate */
	mdelay(50);
	/* Verify that the FMC is plugged (0 is plugged) */
	zfa_hardware_read(fa, ZFA_CAR_FMC_PRES, &val);
	if (val) {
		dev_err(hwdev, "No FCM ADC plugged\n");
		return -ENODEV;
	}
	/* Verify that system PLL is locked (1 is calibrated) */
	zfa_hardware_read(fa, ZFA_CAR_SYS_PLL, &val);
	if (!val) {
		dev_err(hwdev, "System PLL not locked\n");
		return -ENODEV;
	}
	/* Verify that DDR3 calibration is done (1 is calibrated) */
	zfa_hardware_read(fa, ZFA_CAR_DDR_CAL, &val);
	if (!val) {
		dev_err(hwdev, "DDR3 Calibration not done\n");
		return -ENODEV;
	}

	/* Register our trigger hardware */
	err = zio_register_trig(&zfat_type, "adc-100m14b");
	if (err) {
		dev_err(hwdev, "Cannot register ZIO trigger type"
			" \"adc-100m14b\"\n");
		goto out_trg;
	}

	/* Allocate the hardware zio_device for registration */
	fa->hwzdev = zio_allocate_device();
	if (IS_ERR(fa->hwzdev)) {
		dev_err(hwdev, "Cannot allocate ZIO device\n");
		err = PTR_ERR(fa->hwzdev);
		goto out_allocate;
	}

	/* Mandatory fields */
	fa->hwzdev->owner = THIS_MODULE;
	fa->hwzdev->priv_d = fa;

	/* Register the hardware zio_device */
	err = zio_register_device(fa->hwzdev, "adc-100m14b",
				  fa->fmc->device_id);
	if (err) {
		dev_err(hwdev, "Cannot register ZIO device fmc-adc-100m14b\n");
		goto out_dev;
	}
	return 0;

out_dev:
	zio_free_device(fa->hwzdev);
out_allocate:
	zio_unregister_trig(&zfat_type);
out_trg:
	return err;
}

/*
 * fa_zio_exit
 *
 * It removes both device and trigger form the ZIO framework. The FMC remove
 * invokes this function.
 */
void fa_zio_exit(struct fa_dev *fa)
{
	zio_unregister_device(fa->hwzdev);
	zio_free_device(fa->hwzdev);
	zio_unregister_trig(&zfat_type);
}
