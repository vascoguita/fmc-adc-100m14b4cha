// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright CERN 2012-2019
 * Author: Federico Vaga <federico.vaga@gmail.com>
 *
 * Table of register masks, used by driver functions
 */
#include "fmc-adc-100m14b4cha.h"

/* Definition of the fmc-adc registers fields: offset - mask - isbitfield */
const struct zfa_field_desc zfad_regs[] = {
	/* Control registers */
	[ZFA_CTL_FMS_CMD] =		{0x00, 0x00000003, 1},
	[ZFA_CTL_CLK_EN] =		{0x00, 0x00000004, 1},
	[ZFA_CTL_DAC_CLR_N] =		{0x00, 0x00000008, 1},
	[ZFA_CTL_BSLIP] =		{0x00, 0x00000010, 1},
	[ZFA_CTL_TEST_DATA_EN] =	{0x00, 0x00000020, 1},
	[ZFA_CTL_TRIG_LED] =		{0x00, 0x00000040, 1},
	[ZFA_CTL_ACQ_LED] =		{0x00, 0x00000080, 1},
	[ZFA_CTL_RST_TRG_STA] =	{0x00, 0x00000100, 1},
	/* Status registers */
	[ZFA_STA_FSM] =		{0x04, 0x00000007, 1},
	[ZFA_STA_SERDES_PLL] =		{0x04, 0x00000008, 1},
	[ZFA_STA_SERDES_SYNCED] =	{0x04, 0x00000010, 1},
	/* Trigger */
		/* Config register */
	[ZFAT_CFG_STA] =		{0x08, 0xFFFFFFFF, 0},
	[ZFAT_CFG_SRC] =		{0x0C, 0xFFFFFFFF, 0},
	[ZFAT_CFG_POL] =		{0x10, 0xFFFFFFFF, 0},
		/* Delay */
	[ZFAT_EXT_DLY] =		{0x14, 0xFFFFFFFF, 0},
		/* Software */
	[ZFAT_SW] =			{0x18, 0xFFFFFFFF, 0},
		/* Number of shots */
	[ZFAT_SHOTS_NB] =		{0x1C, 0x0000FFFF, 1},
		/* Remaining shots counter */
	[ZFAT_SHOTS_REM] =		{0x1C, 0xFFFF0000, 1},
		/* Multishot max samples*/
	[ZFA_MULT_MAX_SAMP] =		{0x20, 0xFFFFFFFF, 0},
		/* Position address */
	[ZFAT_POS] =			{0x24, 0xFFFFFFFF, 0},
		/* Sampling clock frequency */
	[ZFAT_SAMPLING_HZ] =		{0x28, 0xFFFFFFFF, 0},
		/* Sample rate */
	[ZFAT_SR_UNDER] =		{0x2C, 0xFFFFFFFF, 0},
		/* Pre-sample */
	[ZFAT_PRE] =			{0x30, 0xFFFFFFFF, 0},
		/* Post-sample */
	[ZFAT_POST] =			{0x34, 0xFFFFFFFF, 0},
		/* Sample counter */
	[ZFAT_CNT] =			{0x38, 0xFFFFFFFF, 0},

	/* Channel 1 */
	[ZFA_CH1_CTL_RANGE] =		{0x80, 0x00000077, 1},
	[ZFA_CH1_CTL_TERM] =		{0x80, 0x00000008, 1},
	[ZFA_CH1_STA] =		{0x84, 0x0000FFFF, 0},
	[ZFA_CH1_GAIN] =		{0x88, 0x0000FFFF, 1},
	[ZFA_CH1_OFFSET] =		{0x88, 0xFFFF0000, 1},
	[ZFA_CH1_SAT] =		{0x8C, 0x00007FFF, 0},
	[ZFA_CH1_HYST] =		{0x90, 0xFFFF0000, 1},
	[ZFA_CH1_THRES] =		{0x90, 0x0000FFFF, 1},
	[ZFA_CH1_DLY] =		{0x94, 0xFFFFFFFF, 0},

	/* Channel 2 */
	[ZFA_CH2_CTL_RANGE] =		{0xC0, 0x00000077, 1},
	[ZFA_CH2_CTL_TERM] =		{0xC0, 0x00000008, 1},
	[ZFA_CH2_STA] =		{0xC4, 0x0000FFFF, 0},
	[ZFA_CH2_GAIN] =		{0xC8, 0x0000FFFF, 1},
	[ZFA_CH2_OFFSET] =		{0xC8, 0xFFFF0000, 1},
	[ZFA_CH2_SAT] =		{0xCC, 0x00007FFF, 0},
	[ZFA_CH2_HYST] =		{0xD0, 0xFFFF0000, 1},
	[ZFA_CH2_THRES] =		{0xD0, 0x0000FFFF, 1},
	[ZFA_CH2_DLY] =		{0xD4, 0xFFFFFFFF, 0},

	/* Channel 3 */
	[ZFA_CH3_CTL_RANGE] =		{0x100, 0x00000077, 1},
	[ZFA_CH3_CTL_TERM] =		{0x100, 0x00000008, 1},
	[ZFA_CH3_STA] =		{0x104, 0x0000FFFF, 0},
	[ZFA_CH3_GAIN] =		{0x108, 0x0000FFFF, 1},
	[ZFA_CH3_OFFSET] =		{0x108, 0xFFFF0000, 1},
	[ZFA_CH3_SAT] =		{0x10C, 0x00007FFF, 0},
	[ZFA_CH3_HYST] =		{0x110, 0xFFFF0000, 1},
	[ZFA_CH3_THRES] =		{0x110, 0x0000FFFF, 1},
	[ZFA_CH3_DLY] =		{0x114, 0xFFFFFFFF, 0},

	/* Channel 4 */
	[ZFA_CH4_CTL_RANGE] =		{0x140, 0x00000077, 1},
	[ZFA_CH4_CTL_TERM] =		{0x140, 0x00000008, 1},
	[ZFA_CH4_STA] =		{0x144, 0x0000FFFF, 0},
	[ZFA_CH4_GAIN] =		{0x148, 0x0000FFFF, 1},
	[ZFA_CH4_OFFSET] =		{0x148, 0xFFFF0000, 1},
	[ZFA_CH4_SAT] =		{0x14C, 0x00007FFF, 0},
	[ZFA_CH4_HYST] =		{0x150, 0xFFFF0000, 1},
	[ZFA_CH4_THRES] =		{0x150, 0x0000FFFF, 1},
	[ZFA_CH4_DLY] =		{0x154, 0xFFFFFFFF, 0},

	/* IRQ */
	[ZFA_IRQ_ADC_DISABLE_MASK] =	{0x00, 0x00000003, 0},
	[ZFA_IRQ_ADC_ENABLE_MASK] =	{0x04, 0x00000003, 0},
	[ZFA_IRQ_ADC_MASK_STATUS] =	{0x08, 0x00000003, 0},
	[ZFA_IRQ_ADC_SRC] =		{0x0C, 0x00000003, 0},
	[ZFA_IRQ_VIC_CTRL] =		{0x00, 0x000FFFFF, 0},
	[ZFA_IRQ_VIC_ENABLE_MASK] =     {0x08, 0x00000003, 0},
	[ZFA_IRQ_VIC_DISABLE_MASK] =    {0x0C, 0x00000003, 0},
	[ZFA_IRQ_VIC_MASK_STATUS] =     {0x10, 0x00000003, 0},

	/* UTC */
	[ZFA_UTC_SECONDS] =		{0x00, ~0x0, 0},
	[ZFA_UTC_COARSE] =		{0x04, ~0x0, 0},
	[ZFA_UTC_TRIG_META] =		{0x08, ~0x0, 0},
	[ZFA_UTC_TRIG_SECONDS] =	{0x0C, ~0x0, 0},
	[ZFA_UTC_TRIG_COARSE] =	{0x10, ~0x0, 0},
	[ZFA_UTC_TRIG_FINE] =		{0x14, ~0x0, 0},
	[ZFA_UTC_ACQ_START_META] =	{0x18, ~0x0, 0},
	[ZFA_UTC_ACQ_START_SECONDS] =	{0x1C, ~0x0, 0},
	[ZFA_UTC_ACQ_START_COARSE] =	{0x20, ~0x0, 0},
	[ZFA_UTC_ACQ_START_FINE] =	{0x24, ~0x0, 0},
	[ZFA_UTC_ACQ_STOP_META] =	{0x28, ~0x0, 0},
	[ZFA_UTC_ACQ_STOP_SECONDS] =	{0x2C, ~0x0, 0},
	[ZFA_UTC_ACQ_STOP_COARSE] =	{0x30, ~0x0, 0},
	[ZFA_UTC_ACQ_STOP_FINE] =	{0x34, ~0x0, 0},
	[ZFA_UTC_ACQ_END_META] =	{0x38, ~0x0, 0},
	[ZFA_UTC_ACQ_END_SECONDS] =	{0x3C, ~0x0, 0},
	[ZFA_UTC_ACQ_END_COARSE] =	{0x40, ~0x0, 0},
	[ZFA_UTC_ACQ_END_FINE] =	{0x44, ~0x0, 0},
};
