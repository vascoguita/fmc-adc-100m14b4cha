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
	ZIO_ATTR(zdev, ZIO_ATTR_NBITS, S_IRUGO, ZFA_SW_R_NOADDRES_NBIT, 14),
};
static struct zio_attribute zfad_cset_ext_zattr[] = {
	/*
	 * sample-decimation
	 * ADC acquire always at the maximum sample rate, to make "slower"
	 * acquisition you can decimate samples. 0 is a forbidden value, 1
	 * for the maximum speed.
	 */
	ZIO_ATTR_EXT("sample-decimation", S_IRUGO | S_IWUGO, ZFAT_SR_DECI, 1),

	/*
	 * State machine commands
	 * 1: start
	 * 2: stop
	 */
	ZIO_PARAM_EXT("fsm-command", S_IWUGO, ZFA_CTL_FMS_CMD, 0),
	/*
	 * Automatic start acquisition
	 * 1: enabled
	 * 0: disabled
	 */
	ZIO_PARAM_EXT("fsm-auto-start", S_IRUGO | S_IWUGO, ZFA_SW_R_NOADDERS_AUTO, 0),
	/*
	 * fsm - status of the state machine:
	 * 1: IDLE
	 * 2: PRE_TRIG
	 * 3: WAIT_TRIG
	 * 4: POST_TRIG
	 * 5: DECR_SHOT
	 * 7: Illegal
	 * */
	ZIO_PARAM_EXT("fsm-state", S_IRUGO, ZFA_STA_FSM, 0),
	/* last acquisition start time stamp */
	ZIO_PARAM_EXT("tstamp-acq-str-s", S_IRUGO, ZFA_UTC_ACQ_START_SECONDS, 0),
	ZIO_PARAM_EXT("tstamp-acq-str-t", S_IRUGO, ZFA_UTC_ACQ_START_COARSE, 0),
	ZIO_PARAM_EXT("tstamp-acq-str-b", S_IRUGO, ZFA_UTC_ACQ_START_FINE, 0),
	/* last acquisition end time stamp */
	ZIO_PARAM_EXT("tstamp-acq-end-s", S_IRUGO, ZFA_UTC_ACQ_END_SECONDS, 0),
	ZIO_PARAM_EXT("tstamp-acq-end-t", S_IRUGO, ZFA_UTC_ACQ_END_COARSE, 0),
	ZIO_PARAM_EXT("tstamp-acq-end-b", S_IRUGO, ZFA_UTC_ACQ_END_FINE, 0),
	/* last acquisition stop time stamp */
	ZIO_PARAM_EXT("tstamp-acq-stp-s", S_IRUGO, ZFA_UTC_ACQ_STOP_SECONDS, 0),
	ZIO_PARAM_EXT("tstamp-acq-stp-t", S_IRUGO, ZFA_UTC_ACQ_STOP_COARSE, 0),
	ZIO_PARAM_EXT("tstamp-acq-stp-b", S_IRUGO, ZFA_UTC_ACQ_STOP_FINE, 0),
	/* Reset all channel offset */
	ZIO_PARAM_EXT("rst-ch-offset", S_IWUGO, ZFA_CTL_DAC_CLR_N, 1),

	ZIO_ATTR_EXT("ch0-offset", S_IRUGO | S_IWUGO, ZFA_CH1_OFFSET, 0),
	ZIO_ATTR_EXT("ch1-offset", S_IRUGO | S_IWUGO, ZFA_CH2_OFFSET, 0),
	ZIO_ATTR_EXT("ch2-offset", S_IRUGO | S_IWUGO, ZFA_CH3_OFFSET, 0),
	ZIO_ATTR_EXT("ch3-offset", S_IRUGO | S_IWUGO, ZFA_CH4_OFFSET, 0),

	ZIO_ATTR_EXT("ch0-vref", S_IRUGO | S_IWUGO, ZFA_CH1_CTL_RANGE, 0),
	ZIO_ATTR_EXT("ch1-vref", S_IRUGO | S_IWUGO, ZFA_CH2_CTL_RANGE, 0),
	ZIO_ATTR_EXT("ch2-vref", S_IRUGO | S_IWUGO, ZFA_CH3_CTL_RANGE, 0),
	ZIO_ATTR_EXT("ch3-vref", S_IRUGO | S_IWUGO, ZFA_CH4_CTL_RANGE, 0),
};

/* FIXME Unused until TLV control will be available */
static ZIO_ATTR_DEFINE_STD(ZIO_DEV, zfad_chan_std_zattr) = {
	/* the offset is complement 2 format */
	ZIO_ATTR(zdev, ZIO_ATTR_OFFSET, S_IRUGO | S_IWUGO, ZFA_CHx_OFFSET, 0),
	/*
	 * in-range
	 * 0x23 (35): 100mV range
	 * 0x11 (17): 1V range
	 * 0x45 (69): 10V range
	 * 0x00 (0): Open input
	 */
		ZIO_ATTR(zdev, ZIO_ATTR_VREFTYPE, S_IRUGO  | S_IWUGO, ZFA_CHx_CTL_RANGE, 0x11),
};

static struct zio_attribute zfad_chan_ext_zattr[] = {

	ZIO_PARAM_EXT("current-value", S_IRUGO, ZFA_CHx_STA, 0),
};

static struct zio_attribute zfad_dev_ext_zattr[] = {
	/* Get Mezzanine temperature from onewire */
	ZIO_PARAM_EXT("temperature", S_IRUGO, ZFA_SW_R_NOADDRES_TEMP, 0),
};
/* Calculate correct index for channel from CHx indexes */
static inline int zfad_get_chx_index(unsigned long addr,
				     struct zio_channel *chan)
{
	return addr + ZFA_CHx_MULT  * (1 + chan->index - chan->cset->n_chan);
}
int zfad_fsm_command(struct fa_dev *fa, uint32_t command)
{
	uint32_t val;

	/* If START, check if we can start */
	if (command == ZFA_START) {
		/* Verify that SerDes PLL is lockes */
		zfa_common_info_get(fa, &zfad_regs[ZFA_STA_SERDES_PLL], &val);
		if (!val) {
			dev_err(fa->fmc->hwdev,
			"Cannot start acquisition: SerDes PLL not locked\n");
			return -EBUSY;
		}
		/* Verify that SerDes is synched */
		zfa_common_info_get(fa, &zfad_regs[ZFA_STA_SERDES_SYNCED], &val);
		if (!val) {
			dev_err(fa->fmc->hwdev,
			"Cannot start acquisition: SerDes not synchronized\n");
			return -EBUSY;
		}
	}
	/*
	 * When any command occurs we are ready to start a new
	 * acquisition, so we must abort any previous one.
	 * If it is STOP, we abort because we abort
	 * an acquisition.
	 * If it is START, we abort because if there was a
	 * previous start but the acquisition end interrupt
	 * doesn't occurs, START mean RESTART.
	 * If it is a clean START, the abort has not effects
	 */
	zio_trigger_abort(fa->zdev->cset);
	if (command == ZFA_START) {
		dev_dbg(fa->fmc->hwdev, "Enable interrupts\n");
		zfa_common_conf_set(fa, &zfad_regs[ZFA_IRQ_MASK], ZFAT_ALL);
	}
	return 0;
}
static void zfad_reset_offset(struct fa_dev *fa)
{
	zfa_common_conf_set(fa, &zfad_regs[ZFA_CTL_DAC_CLR_N], 0);
	udelay(20);
	zfa_common_conf_set(fa, &zfad_regs[ZFA_CTL_DAC_CLR_N], 1);
}

static int zfad_get_range(uint32_t usr_val)
{
	switch (usr_val) {
	case 0x23:
		return ZFA_100mV;
		break;
	case 0x11:
		return ZFA_1V;
		break;
	case 0x45:
		return ZFA_10V;
		break;
	case 0x00:
		return ZFA_OPEN;
		break;
	default:
		break;
	}

	return -EINVAL;
}
/*
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
	zfa_common_conf_set(fa, &zfad_regs[i], cal_val);
	/* Apply the ADC calibration value for the gain */
	i = zfad_get_chx_index(ZFA_CHx_GAIN, chan);
	cal_val = fa->adc_cal_data[range].gain[chan->index];
	dev_dbg(&chan->head.dev, "gain calibration value 0x%x\n", cal_val);
	zfa_common_conf_set(fa, &zfad_regs[i], cal_val);

	return 0;
}

/*
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
	zfa_common_info_get(fa, &zfad_regs[i], &range_reg);

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
	const struct zio_field_desc *reg;
	int i, err;

	i = 4; /* FIXME temporary, to get chan number */
	switch (zattr->id) {
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
		reg = &zfad_regs[zattr->id];
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
		i = zfad_get_chx_index(zattr->id, to_zio_chan(dev));
		reg = &zfad_regs[i];
		break;
	case ZFA_CTL_FMS_CMD:
		err = zfad_fsm_command(fa, usr_val);
		if (err)
			return err;
	default:
		reg = &zfad_regs[zattr->id];
	}

	return zfa_common_conf_set(fa, reg, usr_val);
}
/* get the value of a FMC-ADC register */
static int zfad_info_get(struct device *dev, struct zio_attribute *zattr,
			 uint32_t *usr_val)
{
	const struct zio_field_desc *reg;
	struct fa_dev *fa = get_zfadc(dev);
	int i;

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
		return 0;
	case ZFA_CHx_CTL_RANGE:
	case ZFA_CHx_STA:
		i = zfad_get_chx_index(zattr->id, to_zio_chan(dev));
		reg = &zfad_regs[i];
		break;
	default:
		reg = &zfad_regs[zattr->id];
	}

	zfa_common_info_get(fa, reg, usr_val);

	return 0;
}
static const struct zio_sysfs_operations zfad_s_op = {
	.conf_set = zfad_conf_set,
	.info_get = zfad_info_get,
};


/*
 * Prepare the FMC-ADC for the DMA transfer. FMC-ADC fire the hardware trigger,
 * it acquires all samples in its DDR memory and then it allows the driver to
 * transfer data through DMA. So zfad_input_cset only configure and start the
 * DMA transfer.
 */
static int zfad_input_cset(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	int err;

	/* ZIO should configure only the interleaved channel */
	if (!cset->interleave)
		return -EINVAL;
	/* nsamples can't be 0 */
	if (!cset->interleave->current_ctrl->nsamples) {
		dev_err(&cset->head.dev, "no post/pre-sample configured\n");
		return -EINVAL;
	}
	err = zfad_map_dma(cset);
	if (err)
		return err;

	/* Start DMA transefer */
	zfa_common_conf_set(fa, &zfad_regs[ZFA_DMA_CTL_START], 1);
	dev_dbg(&cset->head.dev, "Start DMA transfer\n");

	return -EAGAIN; /* data_done on DMA_DONE interrupt */
}

static int zfad_zio_probe(struct zio_device *zdev)
{
	struct fa_dev *fa = zdev->priv_d;

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

	return 0;
}

static int zfad_init_cset(struct zio_cset *cset)
{
	const struct zio_field_desc *reg;
	struct fa_dev *fa = cset->zdev->priv_d;
	int i;

	dev_dbg(&cset->head.dev, "%s:%d", __func__, __LINE__);
	/* Force stop FSM to prevent early trigger fire */
	zfa_common_conf_set(fa, &zfad_regs[ZFA_CTL_FMS_CMD], ZFA_STOP);
	/* Initialize channels gain to 1 and range to 1V */
	for (i = 0; i < 4; ++i) {
		reg = &zfad_regs[ZFA_CH1_GAIN + (i * ZFA_CHx_MULT)];
		zfa_common_conf_set(fa, reg, 0x8000);
		reg = &zfad_regs[ZFA_CH1_CTL_RANGE + (i * ZFA_CHx_MULT)];
		zfa_common_conf_set(fa, reg, 0x11);
	}
	/* Enable mezzanine clock */
	zfa_common_conf_set(fa, &zfad_regs[ZFA_CTL_CLK_EN], 1);
	/* Reset channel offset to mid-scale */
	zfad_reset_offset(fa);
	/* Set DMA to transfer data from device to host */
	zfa_common_conf_set(fa, &zfad_regs[ZFA_DMA_BR_DIR], 0);
	/* Set decimation to minimum */
	zfa_common_conf_set(fa, &zfad_regs[ZFAT_SR_DECI], 1);
	/* Set test data register */
	zfa_common_conf_set(fa, &zfad_regs[ZFA_CTL_TEST_DATA_EN],
			    enable_test_data);

	/* Trigger registers */
	/* Set to single shot mode by default */
	zfa_common_conf_set(fa, &zfad_regs[ZFAT_SHOTS_NB], 1);
	cset->ti->zattr_set.std_zattr[ZIO_ATTR_TRIG_REENABLE].value = 0;
	/* Disable Software trigger*/
	zfa_common_conf_set(fa, &zfad_regs[ZFAT_CFG_SW_EN], 0);
	/* Enable Hardware trigger*/
	zfa_common_conf_set(fa, &zfad_regs[ZFAT_CFG_HW_EN], 1);
	/* Select external trigger (index 0) */
	zfa_common_conf_set(fa, &zfad_regs[ZFAT_CFG_HW_SEL], 1);
	cset->ti->zattr_set.ext_zattr[0].value = 1;
	/* Set UTC seconds from the kernel seconds */
	zfa_common_conf_set(fa, &zfad_regs[ZFA_UTC_SECONDS], get_seconds());
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
	zfa_common_info_get(fa, &zfad_regs[ZFA_CAR_FMC_PRES], &val);
	if (val) {
		dev_err(hwdev, "No FCM ADC plugged\n");
		return -ENODEV;
	}
	/* Verify that system PLL is locked (1 is calibrated) */
	zfa_common_info_get(fa, &zfad_regs[ZFA_CAR_SYS_PLL], &val);
	if (!val) {
		dev_err(hwdev, "System PLL not locked\n");
		return -ENODEV;
	}
	/* Verify that DDR3 calibration is done (1 is calibrated) */
	zfa_common_info_get(fa, &zfad_regs[ZFA_CAR_DDR_CAL], &val);
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
