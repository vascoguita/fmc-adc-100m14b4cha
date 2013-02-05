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

#include <linux/zio.h>
#include <linux/zio-buffer.h>
#include <linux/zio-trigger.h>
#include <linux/zio-utils.h>

#include "spec.h"
#include "fmc-adc.h"

int enable_auto_start = 0;
static int enable_test_data = 0;

module_param(enable_test_data, int, 0444);

/* Definition of the fmc-adc registers address - mask - mask offset */
const struct zio_field_desc zfad_regs[] = {
	/* Control registers */
	[ZFA_CTL_FMS_CMD] =		{FA_ADC_MEM_OFF + 0x00, 0x0003, 0},
	[ZFA_CTL_CLK_EN] =		{FA_ADC_MEM_OFF + 0x00, 0x0001, 2},
	[ZFA_CTL_DAC_CLR_N] =		{FA_ADC_MEM_OFF + 0x00, 0x0001, 3},
	[ZFA_CTL_BSLIP] =		{FA_ADC_MEM_OFF + 0x00, 0x0001, 4},
	[ZFA_CTL_TEST_DATA_EN] =	{FA_ADC_MEM_OFF + 0x00, 0x0001, 5},
	[ZFA_CTL_TRIG_LED] =		{FA_ADC_MEM_OFF + 0x00, 0x0001, 6},
	[ZFA_CTL_ACQ_LED] =		{FA_ADC_MEM_OFF + 0x00, 0x0001, 7},
	/* Status registers */
	[ZFA_STA_FSM] =			{FA_ADC_MEM_OFF + 0x04, 0x0007, 0},
	[ZFA_STA_SERDES_PLL] =		{FA_ADC_MEM_OFF + 0x04, 0x0001, 3},
	[ZFA_STA_SERDES_SYNCED] =	{FA_ADC_MEM_OFF + 0x04, 0x0001, 4},
	/* Trigger */
		/* Config register */
	[ZFAT_CFG_HW_SEL] =		{FA_ADC_MEM_OFF + 0x08, 0x00000001, 0},
	[ZFAT_CFG_HW_POL] =		{FA_ADC_MEM_OFF + 0x08, 0x00000001, 1},
	[ZFAT_CFG_HW_EN] =		{FA_ADC_MEM_OFF + 0x08, 0x00000001, 2},
	[ZFAT_CFG_SW_EN] =		{FA_ADC_MEM_OFF + 0x08, 0x00000001, 3},
	[ZFAT_CFG_INT_SEL] =		{FA_ADC_MEM_OFF + 0x08, 0x00000003, 4},
	[ZFAT_CFG_THRES] =		{FA_ADC_MEM_OFF + 0x08, 0x0000FFFF, 16},
		/* Delay */
	[ZFAT_DLY] =			{FA_ADC_MEM_OFF + 0x0C, 0xFFFFFFFF, 0},
		/* Software */
	[ZFAT_SW] =			{FA_ADC_MEM_OFF + 0x10, 0xFFFFFFFF, 0},
		/* Number of shots */
	[ZFAT_SHOTS_NB] =		{FA_ADC_MEM_OFF + 0x14, 0x0000FFFF, 0},
		/* Sample rate */
	[ZFAT_SR_DECI] =		{FA_ADC_MEM_OFF + 0x1C, 0xFFFF, 0},
		/* Position address */
	[ZFAT_POS] =			{FA_ADC_MEM_OFF + 0x18, 0xFFFFFFFF, 0},
		/* Pre-sample */
	[ZFAT_PRE] =			{FA_ADC_MEM_OFF + 0x20, 0xFFFFFFFF, 0},
		/* Post-sample */
	[ZFAT_POST] =			{FA_ADC_MEM_OFF + 0x24, 0xFFFFFFFF, 0},
		/* Sample counter */
	[ZFAT_CNT] =			{FA_ADC_MEM_OFF + 0x28, 0xFFFFFFFF, 0},
	/* Channel 1 */
	[ZFA_CH1_CTL_RANGE] =		{FA_ADC_MEM_OFF + 0x2C, 0x007F, 0},
	[ZFA_CH1_STA] =			{FA_ADC_MEM_OFF + 0x30, 0xFFFF, 0},
	[ZFA_CH1_GAIN] =		{FA_ADC_MEM_OFF + 0x34, 0xFFFF, 0},
	[ZFA_CH1_OFFSET] =		{FA_ADC_MEM_OFF + 0x38, 0xFFFF, 0},
	/* Channel 2 */
	[ZFA_CH2_CTL_RANGE] =		{FA_ADC_MEM_OFF + 0x3C, 0x007F, 0},
	[ZFA_CH2_STA] =			{FA_ADC_MEM_OFF + 0x40, 0xFFFF, 0},
	[ZFA_CH2_GAIN] =		{FA_ADC_MEM_OFF + 0x44, 0xFFFF, 0},
	[ZFA_CH2_OFFSET] =		{FA_ADC_MEM_OFF + 0x48, 0xFFFF, 0},
	/* Channel 3 */
	[ZFA_CH3_CTL_RANGE] =		{FA_ADC_MEM_OFF + 0x4C, 0x007F, 0},
	[ZFA_CH3_STA] =			{FA_ADC_MEM_OFF + 0x50, 0xFFFF, 0},
	[ZFA_CH3_GAIN] =		{FA_ADC_MEM_OFF + 0x54, 0xFFFF, 0},
	[ZFA_CH3_OFFSET] =		{FA_ADC_MEM_OFF + 0x58, 0xFFFF, 0},
	/* Channel 4 */
	[ZFA_CH4_CTL_RANGE] =		{FA_ADC_MEM_OFF + 0x5C, 0x007F, 0},
	[ZFA_CH4_STA] =			{FA_ADC_MEM_OFF + 0x60, 0xFFFF, 0},
	[ZFA_CH4_GAIN] =		{FA_ADC_MEM_OFF + 0x64, 0xFFFF, 0},
	[ZFA_CH4_OFFSET] =		{FA_ADC_MEM_OFF + 0x68, 0xFFFF, 0},
	/* DMA */
	[ZFA_DMA_CTL_SWP] =		{FA_DMA_MEM_OFF + 0x00, 0x0003, 2},
	[ZFA_DMA_CTL_ABORT] =		{FA_DMA_MEM_OFF + 0x00, 0x0001, 1},
	[ZFA_DMA_CTL_START] =		{FA_DMA_MEM_OFF + 0x00, 0x0001, 0},
	[ZFA_DMA_STA] =			{FA_DMA_MEM_OFF + 0x04, 0x0007, 0},
	[ZFA_DMA_ADDR] =		{FA_DMA_MEM_OFF + 0x08, 0xFFFFFFFF, 0},
	[ZFA_DMA_ADDR_L] =		{FA_DMA_MEM_OFF + 0x0C, 0xFFFFFFFF, 0},
	[ZFA_DMA_ADDR_H] =		{FA_DMA_MEM_OFF + 0x10, 0xFFFFFFFF, 0},
	[ZFA_DMA_LEN] =			{FA_DMA_MEM_OFF + 0x14, 0xFFFFFFFF, 0},
	[ZFA_DMA_NEXT_L] =		{FA_DMA_MEM_OFF + 0x18, 0xFFFFFFFF, 0},
	[ZFA_DMA_NEXT_H] =		{FA_DMA_MEM_OFF + 0x1C, 0xFFFFFFFF, 0},
	[ZFA_DMA_BR_DIR] =		{FA_DMA_MEM_OFF + 0x20, 0x0001, 1},
	[ZFA_DMA_BR_LAST] =		{FA_DMA_MEM_OFF + 0x20, 0x0001, 0},
	/* IRQ */
	[ZFA_IRQ_MULTI] =		{FA_IRQ_MEM_OFF + 0x00, 0x000F, 0},
	[ZFA_IRQ_SRC] =			{FA_IRQ_MEM_OFF + 0x04, 0x000F, 0},
	[ZFA_IRQ_MASK] =		{FA_IRQ_MEM_OFF + 0x08, 0x000F, 0},
	/* UTC */
	[ZFA_UTC_SECONDS] =		{FA_UTC_MEM_OFF + 0x00, ~0x0, 0},
	[ZFA_UTC_COARSE] =		{FA_UTC_MEM_OFF + 0x04, ~0x0, 0},
	[ZFA_UTC_TRIG_META] =		{FA_UTC_MEM_OFF + 0x08, ~0x0, 0},
	[ZFA_UTC_TRIG_SECONDS] =	{FA_UTC_MEM_OFF + 0x0C, ~0x0, 0},
	[ZFA_UTC_TRIG_COARSE] =		{FA_UTC_MEM_OFF + 0x10, ~0x0, 0},
	[ZFA_UTC_TRIG_FINE] =		{FA_UTC_MEM_OFF + 0x14, ~0x0, 0},
	[ZFA_UTC_ACQ_START_META] 	{FA_UTC_MEM_OFF + 0x18, ~0x0, 0},
	[ZFA_UTC_ACQ_START_SECONDS] =	{FA_UTC_MEM_OFF + 0x1C, ~0x0, 0},
	[ZFA_UTC_ACQ_START_COARSE] =	{FA_UTC_MEM_OFF + 0x20, ~0x0, 0},
	[ZFA_UTC_ACQ_START_FINE] =	{FA_UTC_MEM_OFF + 0x24, ~0x0, 0},
	[ZFA_UTC_ACQ_STOP_META] =	{FA_UTC_MEM_OFF + 0x28, ~0x0, 0},
	[ZFA_UTC_ACQ_STOP_SECONDS] =	{FA_UTC_MEM_OFF + 0x2C, ~0x0, 0},
	[ZFA_UTC_ACQ_STOP_COARSE] =	{FA_UTC_MEM_OFF + 0x30, ~0x0, 0},
	[ZFA_UTC_ACQ_STOP_FINE] =	{FA_UTC_MEM_OFF + 0x34, ~0x0, 0},
	[ZFA_UTC_ACQ_END_META] =	{FA_UTC_MEM_OFF + 0x38, ~0x0, 0},
	[ZFA_UTC_ACQ_END_SECONDS] =	{FA_UTC_MEM_OFF + 0x3C, ~0x0, 0},
	[ZFA_UTC_ACQ_END_COARSE] =	{FA_UTC_MEM_OFF + 0x40, ~0x0, 0},
	[ZFA_UTC_ACQ_END_FINE] =	{FA_UTC_MEM_OFF + 0x44, ~0x0, 0},
	/* Carrier CSR */
	[ZFA_CAR_FMC_PRES] =		{FA_CAR_MEM_OFF + 0x0C, 0x1, 0},
	[ZFA_CAR_P2L_PLL] =		{FA_CAR_MEM_OFF + 0x0C, 0x1, 1},
	[ZFA_CAR_SYS_PLL] =		{FA_CAR_MEM_OFF + 0x0C, 0x1, 2},
	[ZFA_CAR_DDR_CAL] =		{FA_CAR_MEM_OFF + 0x0C, 0x1, 3},
};

/* zio device attributes */
static ZIO_ATTR_DEFINE_STD(ZIO_DEV, zfad_cset_std_zattr) = {
	ZIO_ATTR(zdev, ZIO_ATTR_NBITS, ZIO_RO_PERM, ZFA_SW_R_NOADDRES_NBIT, 14),
};
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
	ZIO_PARAM_EXT("tstamp-acq-str-s", ZIO_RO_PERM, ZFA_UTC_ACQ_START_SECONDS, 0),
	ZIO_PARAM_EXT("tstamp-acq-str-t", ZIO_RO_PERM, ZFA_UTC_ACQ_START_COARSE, 0),
	ZIO_PARAM_EXT("tstamp-acq-str-b", ZIO_RO_PERM, ZFA_UTC_ACQ_START_FINE, 0),
	/* last acquisition end time stamp */
	ZIO_PARAM_EXT("tstamp-acq-end-s", ZIO_RO_PERM, ZFA_UTC_ACQ_END_SECONDS, 0),
	ZIO_PARAM_EXT("tstamp-acq-end-t", ZIO_RO_PERM, ZFA_UTC_ACQ_END_COARSE, 0),
	ZIO_PARAM_EXT("tstamp-acq-end-b", ZIO_RO_PERM, ZFA_UTC_ACQ_END_FINE, 0),
	/* last acquisition stop time stamp */
	ZIO_PARAM_EXT("tstamp-acq-stp-s", ZIO_RO_PERM, ZFA_UTC_ACQ_STOP_SECONDS, 0),
	ZIO_PARAM_EXT("tstamp-acq-stp-t", ZIO_RO_PERM, ZFA_UTC_ACQ_STOP_COARSE, 0),
	ZIO_PARAM_EXT("tstamp-acq-stp-b", ZIO_RO_PERM, ZFA_UTC_ACQ_STOP_FINE, 0),
	/* Reset all channel offset */
	ZIO_PARAM_EXT("rst-ch-offset", ZIO_WO_PERM, ZFA_CTL_DAC_CLR_N, 1),

	ZIO_ATTR_EXT("ch0-offset", ZIO_RW_PERM, ZFA_CH1_OFFSET, 0),
	ZIO_ATTR_EXT("ch1-offset", ZIO_RW_PERM, ZFA_CH2_OFFSET, 0),
	ZIO_ATTR_EXT("ch2-offset", ZIO_RW_PERM, ZFA_CH3_OFFSET, 0),
	ZIO_ATTR_EXT("ch3-offset", ZIO_RW_PERM, ZFA_CH4_OFFSET, 0),

	ZIO_ATTR_EXT("ch0-vref", ZIO_RW_PERM, ZFA_CH1_CTL_RANGE, 0),
	ZIO_ATTR_EXT("ch1-vref", ZIO_RW_PERM, ZFA_CH2_CTL_RANGE, 0),
	ZIO_ATTR_EXT("ch2-vref", ZIO_RW_PERM, ZFA_CH3_CTL_RANGE, 0),
	ZIO_ATTR_EXT("ch3-vref", ZIO_RW_PERM, ZFA_CH4_CTL_RANGE, 0),
};

/* FIXME Unused until TLV control will be available */
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

static struct zio_attribute zfad_chan_ext_zattr[] = {

	ZIO_PARAM_EXT("current-value", ZIO_RO_PERM, ZFA_CHx_STA, 0),
};

static struct zio_attribute zfad_dev_ext_zattr[] = {
	/* Get Mezzanine temperature from onewire */
	ZIO_PARAM_EXT("temperature", ZIO_RO_PERM, ZFA_SW_R_NOADDRES_TEMP, 0),
};
/* Calculate correct index for channel from CHx indexes */
static inline int zfad_get_chx_index(unsigned long addr,
				     struct zio_channel *chan)
{
	return addr + ZFA_CHx_MULT  * (1 + chan->index - chan->cset->n_chan);
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
	 * effects
	 */
	zio_trigger_abort_disable(fa->zdev->cset, 0);

	/* If START, check if we can start */
	if (command == ZFA_START) {
		/* Verify that SerDes PLL is lockes */
		zfa_common_info_get(fa, ZFA_STA_SERDES_PLL, &val);
		if (!val) {
			dev_err(fa->fmc->hwdev,
			"Cannot start acquisition: SerDes PLL not locked\n");
			return -EBUSY;
		}
		/* Verify that SerDes is synched */
		zfa_common_info_get(fa, ZFA_STA_SERDES_SYNCED, &val);
		if (!val) {
			dev_err(fa->fmc->hwdev,
			"Cannot start acquisition: SerDes not synchronized\n");
			return -EBUSY;
		}

		/* Now we can arm the trigger for the incoming acquisition */
		zio_arm_trigger(fa->zdev->cset->ti);

		dev_dbg(fa->fmc->hwdev, "Enable interrupts\n");
		zfa_common_conf_set(fa, ZFA_IRQ_MASK, ZFAT_ALL);
	} else {
		dev_dbg(fa->fmc->hwdev, "Disable interrupts\n");
		zfa_common_conf_set(fa, ZFA_IRQ_MASK, ZFAT_NONE);
	}

	zfa_common_conf_set(fa, ZFA_CTL_FMS_CMD, command);
	return 0;
}


/*
 * zfad_reset_offset
 * @fa: the fmc-adc descriptor
 *
 * Reset channel's offsets
 */
static void zfad_reset_offset(struct fa_dev *fa)
{
	zfa_common_conf_set(fa, ZFA_CTL_DAC_CLR_N, 0);
	udelay(20);
	zfa_common_conf_set(fa, ZFA_CTL_DAC_CLR_N, 1);

	/*
	 * FIXME update the value with the default value for the selected range
	 */
}


/*
 * zfad_get_range
 * @usr_val: range value
 *
 * return the enum associated to the range value
 */
static int zfad_get_range(uint32_t usr_val)
{
	switch (usr_val) {
	case 0x23:
		return ZFA_100mV;
	case 0x11:
		return ZFA_1V;
	case 0x45:
		return ZFA_10V;
	case 0x00:
		return ZFA_OPEN;
	}

	return -EINVAL;
}


/*
 * zfad_calibration
 * @fa: the fmc-adc descriptor
 * @chan: the channel to calibrate
 * @usr_val: the volt range to set and calibrate
 *
 * When the input range change, we must calibrate the offset and the gain.
 */
static int zfad_calibration(struct fa_dev *fa, struct zio_channel *chan,
			    uint32_t usr_val)
{
	int i, cal_val, range;

	range = zfad_get_range(usr_val);
	if (range < 0)
		return range;

	/* Apply the ADC calibration value for the offset */
	i = zfad_get_chx_index(ZFA_CHx_OFFSET, chan);
	cal_val = fa->adc_cal_data[range].offset[chan->index];
	dev_dbg(&chan->head.dev, "offset calibration value 0x%x\n", cal_val);
	zfa_common_conf_set(fa, i, cal_val);
	/* Apply the ADC calibration value for the gain */
	i = zfad_get_chx_index(ZFA_CHx_GAIN, chan);
	cal_val = fa->adc_cal_data[range].gain[chan->index];
	dev_dbg(&chan->head.dev, "gain calibration value 0x%x\n", cal_val);
	zfa_common_conf_set(fa, i, cal_val);

	return 0;
}


/*
 * zfad_apply_user_offset
 * @fa: the fmc-adc descriptor
 * @chan: the channel where apply offset
 * @usr_val: the offset value to apply
 *
 * Apply user offset to the channel input. Before apply the user offset it must
 * be corrected with offset and gain calibration value. An open input does not
 * need any correction.
 */
static int zfad_apply_user_offset(struct fa_dev *fa, struct zio_channel *chan,
				  uint32_t usr_val)
{
	uint32_t tmp, cal_val, offset, gain, range_reg;
	int i, range;

	i = zfad_get_chx_index(ZFA_CHx_CTL_RANGE, chan);
	zfa_common_info_get(fa, i, &range_reg);

	range = zfad_get_range(range_reg);
	if (range < 0)
		return range;
	if (range != 0) {
		/* Get calibration offset and gain for DAC */
		offset = fa->dac_cal_data[range].offset[chan->index];
		gain = fa->dac_cal_data[range].gain[chan->index];

		/* Calculate calibrater value for DAC */
		cal_val = (((((usr_val - 0x8000 + offset) << 15) * gain) >> 30));
		cal_val += 0x8000;
	} else {	/* Open range */
		cal_val = usr_val;
	}

	if (cal_val > 0xFFFF)
		cal_val = 0xFFFF;

	dev_dbg(&chan->head.dev, "DAC offset calibration value 0x%x\n", cal_val);
	/* Apply calibrated offset to DAC */
	return fa_spi_xfer(fa, chan->index, 16, cal_val, &tmp);
}


/* set a value to a FMC-ADC registers */
static int zfad_conf_set(struct device *dev, struct zio_attribute *zattr,
			 uint32_t usr_val)
{
	struct fa_dev *fa = get_zfadc(dev);
	int i, err, reg_index;

	reg_index = zattr->id;
	i = fa->zdev->cset->n_chan -1 ; /* -1 because of interleaved channel */
	switch (reg_index) {
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
		err = zfad_apply_user_offset(fa, &to_zio_cset(dev)->chan[i], usr_val);
		if (err)
			return err;
		return 0;
	case ZFA_CHx_OFFSET:
		err = zfad_apply_user_offset(fa, to_zio_chan(dev), usr_val);
		if (err)
			return err;
		return 0;
		break;
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
	case ZFA_CH1_CTL_RANGE:
		i--;
	case ZFA_CH2_CTL_RANGE:
		i--;
	case ZFA_CH3_CTL_RANGE:
		i--;
	case ZFA_CH4_CTL_RANGE:
		i--;
		err = zfad_calibration(fa, &to_zio_cset(dev)->chan[i], usr_val);
		if (err)
			return err;
		return err;
	case ZFA_CHx_CTL_RANGE:
		err = zfad_calibration(fa, to_zio_chan(dev), usr_val);
		if (err)
			return err;
	case ZFA_CHx_STA:
		reg_index = zfad_get_chx_index(reg_index, to_zio_chan(dev));
		break;
	case ZFA_CTL_FMS_CMD:
		return zfad_fsm_command(fa, usr_val);
	}

	return zfa_common_conf_set(fa, reg_index, usr_val);
}

/* get the value of a FMC-ADC register */
static int zfad_info_get(struct device *dev, struct zio_attribute *zattr,
			 uint32_t *usr_val)
{
	struct fa_dev *fa = get_zfadc(dev);
	int reg_index;

	switch (zattr->id) {
	/* FIXME temporary until TLV control */
	case ZFA_CH1_OFFSET:
	case ZFA_CH2_OFFSET:
	case ZFA_CH3_OFFSET:
	case ZFA_CH4_OFFSET:
	case ZFA_CHx_OFFSET:
	case ZFA_SW_R_NOADDRES_NBIT:
	case ZFA_SW_R_NOADDERS_AUTO:
		/* ZIO automatically return the attribute value */
		return 0;
	case ZFA_SW_R_NOADDRES_TEMP:
		/* Read temperature from onewire */
		*usr_val = fa_read_temp(fa, 0);
		*usr_val = ((*usr_val) / 16) + (((*usr_val) & 0xf) * 1000 / 16);
		return 0;
	case ZFA_CHx_CTL_RANGE:
	case ZFA_CHx_STA:
		reg_index = zfad_get_chx_index(zattr->id, to_zio_chan(dev));
		break;
	default:
		reg_index = zattr->id;
	}

	zfa_common_info_get(fa, reg_index, usr_val);

	return 0;
}
static const struct zio_sysfs_operations zfad_s_op = {
	.conf_set = zfad_conf_set,
	.info_get = zfad_info_get,
};

/*
 * zfad_input_cset
 * @cset: channel set to acquire
 *
 * Prepare the FMC-ADC for the acquisition.
 */
static int zfad_input_cset(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;

	/* ZIO should configure only the interleaved channel */
	if (!cset->interleave)
		return -EINVAL;
	/* nsamples can't be 0 */
	if (!cset->interleave->current_ctrl->nsamples) {
		dev_err(&cset->head.dev, "no post/pre-sample configured\n");
		return -EINVAL;
	}

	/*
	 * If the user is using the ADC trigger, then it can do a multi-shot
	 * acquisition.
	 * If the user is using a software trigger, it cannot do multi-shot.
	 * The generic arm trigger used by software trigger returns a
	 * zio_block. We must convert it into a zfad_block to perform DMA
	 */
	if (cset->trig != &zfat_type) {
		struct zfad_block *tmp;
		int err;

		/* Check if device memory allows this acquisition */
		err = zfat_overflow_detection(cset->ti, ZFAT_POST,
					      cset->ti->nsamples);
		if (err)
			return err;

		tmp = kmalloc(sizeof(struct zfad_block), GFP_ATOMIC);
		if (!tmp)
			return -ENOMEM;
		tmp->block = cset->interleave->active_block;
		cset->interleave->priv_d = tmp;

		fa->n_shots = 1;
		/* Configure post samples */
		zfa_common_conf_set(fa, ZFAT_POST, cset->ti->nsamples);

		/* Start the acquisition */
		zfad_fsm_command(fa, ZFA_START);
		/* Fire software trigger */
		zfa_common_conf_set(fa, ZFAT_SW, 1);
	}

	return -EAGAIN; /* data_done on DMA_DONE interrupt */
}

/*
 * zfad_stop_cset
 * @cset: channel set to stop
 *
 * Stop an acquisition
 */
static void zfad_stop_cset(struct zio_cset *cset)
{

	struct fa_dev *fa = cset->zdev->priv_d;
	/* Force the acquisition to stop */
	zfad_fsm_command(fa, ZFA_STOP);
	/*
	 * Disable all interrupt. We are aborting acquisition, we don't need
	 * any interrupt
	 */
	dev_dbg(fa->fmc->hwdev, "Disable interrupts\n");
	zfa_common_conf_set(fa, ZFA_IRQ_MASK, ZFAT_NONE);

	/* reset counters */
	fa->n_shots = 0;
	fa->n_fires = 0;
	/* Clear active block */
	cset->interleave->active_block = NULL;
	/* Clear DMA pointer */
	fa->cur_dev_mem = 0;
	fa->lst_dev_mem = 0;

	/* If the user is using a software trigger, then free zfad_block */
	if (cset->trig != &zfat_type) {
		kfree(cset->interleave->priv_d);
		cset->interleave->priv_d = NULL;
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
	zfa_common_info_get(fa, ZFA_IRQ_SRC, irq_status);
	zfa_common_info_get(fa, ZFA_IRQ_MULTI, irq_multi);
	dev_dbg(fa->fmc->hwdev, "irq status = 0x%x multi = 0x%x\n",
			*irq_status, *irq_multi);

	/* Clear current interrupts status */
	zfa_common_conf_set(fa, ZFA_IRQ_SRC, *irq_status);
	zfa_common_conf_set(fa, ZFA_IRQ_MULTI, *irq_multi);

	/* ack the irq */
	fa->fmc->op->irq_ack(fa->fmc);
}

/*
 * zfad_start_dma
 * When all data from all triggers are ready, this function start DMA.
 * We get the first block from the list and start the acquisition on it.
 * When the DMA will end, the interrupt handler will invoke again this function
 * until the list is empty
 */
static void zfad_start_dma(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	struct zio_channel *interleave = cset->interleave;
	struct zfad_block *zfad_block = interleave->priv_d;
	struct zio_ti *ti = cset->ti;
	int err;

	if (fa->n_trans == fa->n_fires) {
		/*
		 * All DMA transfers done! Inform the trigger about this, so
		 * it can store blocks into the buffer
		 */
		zio_trigger_data_done(cset);
		dev_dbg(fa->fmc->hwdev, "%i blocks transfered\n", fa->n_trans);
		fa->n_trans = 0;
		/*
		 * we can safely re-enable triggers.
		 * Hardware trigger depends on the enable status
		 * of the trigger. Software trigger depends on the previous
		 * status taken form zio attributes (index 5 of extended one)
		 * If the user is using a software trigger, enable the software
		 * trigger.
		 */
		if (cset->trig == &zfat_type) {
			zfa_common_conf_set(fa, ZFAT_CFG_HW_EN,
					    (ti->flags & ZIO_STATUS ? 0 : 1));
			zfa_common_conf_set(fa, ZFAT_CFG_SW_EN,
					    ti->zattr_set.ext_zattr[5].value);
		} else {
			zfa_common_conf_set(fa, ZFAT_CFG_SW_EN, 1);
		}

		/* Automatic start next acquisition */
		if (enable_auto_start) {
			dev_dbg(fa->fmc->hwdev, "Automatic start\n");
			zfad_fsm_command(fa, ZFA_START);
		}
	} else {
		err = zfad_map_dma(cset, &zfad_block[fa->n_trans++]);
		if (err)
			return;

		/* Start DMA transefer */
		zfa_common_conf_set(fa, ZFA_DMA_CTL_START, 1);
		dev_dbg(fa->fmc->hwdev, "Start DMA transfer\n");
	}
}

/*
 * zfat_irq_dma_done
 * The DMA is finish due to an error or not. We must handle both the condition.
 * If DMA aborted by an error we abort the acquisition, so we lost all the data.
 * If the DMA complete successfully, we can call zio_trigger_data_done which
 * will complete the transfer
 */
static void zfat_irq_dma_done(struct zio_cset *cset, int status)
{
	struct zfad_block *zfad_block = cset->interleave->priv_d;
	struct fa_dev *fa = cset->zdev->priv_d;
	uint32_t val;

	/* unmap previous acquisition in any */
	if (fa->n_trans > 0)
		zfad_unmap_dma(cset, &zfad_block[fa->n_trans - 1]);

	if (status & ZFAT_DMA_DONE) { /* DMA done*/
		dev_dbg(fa->fmc->hwdev, "DMA transfer done\n");

		zfad_start_dma(cset);
	} else {	/* DMA error */
		zfa_common_info_get(fa, ZFA_DMA_STA, &val);
		dev_err(fa->fmc->hwdev,
			"DMA error (status 0x%x). All acquisition lost\n", val);
		zio_trigger_abort_disable(cset, 0);
		/* Stop the acquisition */
		zfad_fsm_command(fa, ZFA_STOP);
		fa->n_dma_err++;
	}
}


/* Get the last trigger time-stamp from device */
static void zfat_get_time_stamp(struct fa_dev *fa, struct zio_timestamp *ts)
{
	zfa_common_info_get(fa, ZFA_UTC_TRIG_SECONDS, (uint32_t *)&ts->secs);
	zfa_common_info_get(fa, ZFA_UTC_TRIG_COARSE, (uint32_t *)&ts->ticks);
	zfa_common_info_get(fa, ZFA_UTC_TRIG_FINE, (uint32_t *)&ts->bins);
}


/*
 * zfat_irq_trg_fire
 * @fa: fmc-adc descriptor
 *
 * Trigger fires. This function store the timestamp in the current_ctrl and
 * in the pre-allocated block. Then it increments the sequence number both in
 * current_ctrl and int the pre-allocated block;
 */
static void zfat_irq_trg_fire(struct zio_cset *cset)
{
	struct zio_channel *interleave = cset->interleave;
	struct fa_dev *fa = cset->zdev->priv_d;
	struct zfad_block *zfad_block = interleave->priv_d;
	struct zio_control *ctrl;

	dev_dbg(fa->fmc->hwdev, "Trigger fire %i/%i\n",
		fa->n_fires + 1, fa->n_shots);
	if (fa->n_fires >= fa->n_shots) {
		WARN(1, "Invalid Fire");
		dev_err(fa->fmc->hwdev, "Invalid fire, skip\n");
		return;
	}
	pr_info("%s:%d %p\n", __func__, __LINE__, zfad_block);
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
 * FIXME maybe here reset cur|lst_dev_mem
 */
static void zfat_irq_acq_end(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	uint32_t val;

	dev_dbg(fa->fmc->hwdev, "Acquisition done\n");
	/*
	 * All programmed triggers fire, so the acquisition is ended.
	 * If the state machine is _idle_ we can start the DMA transfer.
	 */
	zfa_common_info_get(fa, ZFA_STA_FSM, &val);
	if (val == ZFA_STATE_IDLE) {
		/*
		 * Disable all triggers to prevent fires between
		 * different DMA transfers required for multi-shots
		 */
		zfa_common_conf_set(fa, ZFAT_CFG_HW_EN, 0);
		zfa_common_conf_set(fa, ZFAT_CFG_SW_EN, 0);
		/* Start DMA acquisition */
		zfad_start_dma(cset);
	} else {
		/* we can't DMA if the state machine is not idle */
		dev_warn(fa->fmc->hwdev,
			 "Can't start DMA on the last acquisition, "
			 "it will be done on the next acquisition end\n");
	}
}


/*
 * zfad_irq
 * @irq:
 * @ptr: pointer to fmc_device
 *
 * The different irq status are handled in different if statement. At the end
 * of the if statement we don't call return because it is possible that there
 * are others irq to handle. The order of irq handlers is based on the
 * possibility to have many irq rise at the same time. It is possible that
 * ZFAT_TRG_FIRE and ZFAT_ACQ_END happens simultaneously, so we handle FIRE
 * before END. If ZFAT_DMA_DONE happens with ZFAT_TRG_FIRE and ZFAT_ACQ_END, we
 * handle DMA_DONE before the other because we must conclude a previous
 * acquisition before start a new one
 */
static irqreturn_t zfad_irq(int irq, void *ptr)
{
	struct fmc_device *fmc = ptr;
	struct fa_dev *fa = fmc_get_drvdata(fmc);
	struct zio_cset *cset = fa->zdev->cset;
	uint32_t status, multi;

	/* irq to handle */
	zfat_get_irq_status(fa, &status, &multi);
	if (!status)
		return IRQ_NONE;
	if (unlikely((status & (ZFAT_DMA_DONE | ZFAT_DMA_ERR)) &&
	    (status & (ZFAT_TRG_FIRE | ZFAT_ACQ_END)))) {
		WARN(1, "Cannot handle trigger interrupt and DMA interrupt at "
			"the same time\n");
		/* Stop Acquisition, ADC it is not working properly */
		zfad_fsm_command(fa, ZFA_STOP);
		return IRQ_NONE;
	}
	/*
	 * It cannot happen that DMA_DONE is in the multi register.
	 * It should not happen ...
	 */
	if (status & (ZFAT_DMA_DONE | ZFAT_DMA_ERR))
		zfat_irq_dma_done(cset, status);

	/* Fire the trigger for each interrupt */
	if (status & ZFAT_TRG_FIRE)
		zfat_irq_trg_fire(cset);
	if (multi & ZFAT_TRG_FIRE)
		zfat_irq_trg_fire(cset);
	/*
	 * If it is too fast the ACQ_END interrupt can be in the
	 * multi register
	 */
	if ((status | multi) & ZFAT_ACQ_END)
		zfat_irq_acq_end(cset);

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
 * The device registration completes. Copy the calibration data from the eeprom
 */
static int zfad_zio_probe(struct zio_device *zdev)
{
	struct fa_dev *fa = zdev->priv_d;
	int err = 0;

	dev_dbg(&zdev->head.dev, "%s:%d", __func__, __LINE__);
	/* Save also the pointer to the real zio_device */
	fa->zdev = zdev;
	/* be sure to initialize these values for DMA transfer */
	fa->lst_dev_mem = 0;
	fa->cur_dev_mem = 0;

	/*
	 * Get Calibration Data. ADC calibration value and DAC calibration
	 * value are consecutive; I can fill both array with a single memcpy
	 */
	memcpy(fa->adc_cal_data, fa->fmc->eeprom + FA_CAL_PTR, FA_CAL_LEN);

	/* Configure GPIO for IRQ */
	fa->fmc->op->gpio_config(fa->fmc, zfat_gpio_cfg,
				 ARRAY_SIZE(zfat_gpio_cfg));
	/* Request IRQ */
	err = fa->fmc->op->irq_request(fa->fmc, zfad_irq, "fmc-adc",
				       IRQF_SHARED);
	if (err)
		dev_err(fa->fmc->hwdev, "can't request irq %i (err %i)\n",
			fa->fmc->irq, err);

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


/*
 * zfad_init_cset
 * @cset: channel set to initialize
 *
 * The initialization stop the FSM to prevent any early trigger fire, then it
 * force the default value of ADC registers.
 */
static int zfad_init_cset(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	int i;

	dev_dbg(&cset->head.dev, "%s:%d", __func__, __LINE__);
	/* Force stop FSM to prevent early trigger fire */
	zfa_common_conf_set(fa, ZFA_CTL_FMS_CMD, ZFA_STOP);
	/* Initialize channels gain to 1 and range to 1V */
	for (i = 0; i < 4; ++i) {
		zfa_common_conf_set(fa, ZFA_CH1_GAIN + (i * ZFA_CHx_MULT),
				    0x8000);
		zfa_common_conf_set(fa, ZFA_CH1_CTL_RANGE + (i * ZFA_CHx_MULT),
				    0x0011);
	}
	/* Enable mezzanine clock */
	zfa_common_conf_set(fa, ZFA_CTL_CLK_EN, 1);
	/* Reset channel offset to mid-scale */
	zfad_reset_offset(fa);
	/* Set DMA to transfer data from device to host */
	zfa_common_conf_set(fa, ZFA_DMA_BR_DIR, 0);
	/* Set decimation to minimum */
	zfa_common_conf_set(fa, ZFAT_SR_DECI, 1);
	/* Set test data register */
	zfa_common_conf_set(fa, ZFA_CTL_TEST_DATA_EN, enable_test_data);
	/* Set to single shot mode by default */
	zfa_common_conf_set(fa, ZFAT_SHOTS_NB, 1);
	if (cset->ti->cset->trig == &zfat_type) {
		/* Select external trigger (index 0) */
		zfa_common_conf_set(fa, ZFAT_CFG_HW_SEL, 1);
		cset->ti->zattr_set.ext_zattr[0].value = 1;
	} else {
		/* Enable Software trigger*/
		zfa_common_conf_set(fa, ZFAT_CFG_SW_EN, 1);
		/* Disable Hardware trigger*/
		zfa_common_conf_set(fa, ZFAT_CFG_HW_EN, 0);
	}

	/* Set UTC seconds from the kernel seconds */
	zfa_common_conf_set(fa, ZFA_UTC_SECONDS, get_seconds());
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
		.init = zfad_init_cset,
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
	/* This driver work only with the fmc-adc-trg */
	.preferred_trigger = "fmc-adc-trg",
	.preferred_buffer = "vmalloc",
};


/* List of supported boards */
static const struct zio_device_id zfad_table[] = {
	{"fmc-adc", &zfad_tmpl},
	{},
};

static struct zio_driver fa_zdrv = {
	.driver = {
		.name = "fmc-adc",
		.owner = THIS_MODULE,
	},
	.id_table = zfad_table,
	.probe = zfad_zio_probe,
	.remove = zfad_zio_remove,
};

/* Register and unregister are used to set up the template driver */
int fa_zio_register(void)
{
	return zio_register_driver(&fa_zdrv);
}

void fa_zio_unregister(void)
{
	zio_unregister_driver(&fa_zdrv);
}


/* Init and exit are called for each FMC-ADC card we have */
int fa_zio_init(struct fa_dev *fa)
{
	struct device *hwdev = fa->fmc->hwdev;
	struct spec_dev *spec = fa->fmc->carrier_data;
	struct pci_dev *pdev = spec->pdev;
	uint32_t dev_id, val;
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
	zfa_common_info_get(fa, ZFA_CAR_FMC_PRES, &val);
	if (val) {
		dev_err(hwdev, "No FCM ADC plugged\n");
		return -ENODEV;
	}
	/* Verify that system PLL is locked (1 is calibrated) */
	zfa_common_info_get(fa, ZFA_CAR_SYS_PLL, &val);
	if (!val) {
		dev_err(hwdev, "System PLL not locked\n");
		return -ENODEV;
	}
	/* Verify that DDR3 calibration is done (1 is calibrated) */
	zfa_common_info_get(fa, ZFA_CAR_DDR_CAL, &val);
	if (!val) {
		dev_err(hwdev, "DDR3 Calibration not done\n");
		return -ENODEV;
	}

	/* Register our trigger hardware */
	err = zio_register_trig(&zfat_type, "fmc-adc-trg");
	if (err) {
		dev_err(hwdev, "Cannot register ZIO trigger fmc-adc-trig\n");
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

	/* Our dev_id is bus+devfn */
	dev_id = (pdev->bus->number << 8) | pdev->devfn;

	/* Register the hardware zio_device */
	err = zio_register_device(fa->hwzdev, "fmc-adc", dev_id);
	if (err) {
		dev_err(hwdev, "Cannot register ZIO device fmc-adc\n");
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

void fa_zio_exit(struct fa_dev *fa)
{
	zio_unregister_device(fa->hwzdev);
	zio_free_device(fa->hwzdev);
	zio_unregister_trig(&zfat_type);
}
