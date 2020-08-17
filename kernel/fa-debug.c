// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright CERN 2018-2019
 * Author: Federico Vaga <federico.vaga@cern.ch>
 */

#include "fmc-adc-100m14b4cha.h"

#define FA_DBG_REG32_CH(_n) \
	{.name = "ADC-CSR:ch"#_n"_ctl", .offset = ADC_CSR_OFF + 0x080 + ((_n - 1) * 0x40)}, \
	{.name = "ADC-CSR:ch"#_n"_sta", .offset = ADC_CSR_OFF + 0x084 + ((_n - 1) * 0x40)}, \
	{.name = "ADC-CSR:ch"#_n"_cal_nb", .offset = ADC_CSR_OFF + 0x088 + ((_n - 1) * 0x40)}, \
	{.name = "ADC-CSR:ch"#_n"_sat", .offset = ADC_CSR_OFF + 0x08C + ((_n - 1) * 0x40)}, \
	{.name = "ADC-CSR:ch"#_n"_trig_thres", .offset = ADC_CSR_OFF + 0x090 + ((_n - 1) * 0x40)}, \
	{.name = "ADC-CSR:ch"#_n"_trig_dly", .offset = ADC_CSR_OFF + 0x094 + ((_n - 1) * 0x40)}

#define FA_DBG_REG32_TIM(_name, _off)					\
	{								\
		.name = "TIME-TAG:"#_name"_seconds_upper",		\
		.offset = ADC_UTC_OFF + _off				\
	}, {								\
		.name = "TIME-TAG:"#_name"_seconds_lower",		\
		.offset = ADC_UTC_OFF + _off + 0x4,			\
	}, {								\
		.name = "TIME-TAG:"#_name"_coarse", 			\
		.offset = ADC_UTC_OFF + _off + 0x8,			\
	}


static const struct debugfs_reg32 fa_debugfs_reg32[] = {
	{
		.name = "ADC-CSR:ctl",
		.offset = ADC_CSR_OFF + 0x000,
	},
	{
		.name = "ADC-CSR:sta",
		.offset = ADC_CSR_OFF + 0x004,
	},
	{
		.name = "ADC-CSR:trig_stat",
		.offset = ADC_CSR_OFF + 0x008,
	},
	{
		.name = "ADC-CSR:trig_en",
		.offset = ADC_CSR_OFF + 0x00C,
	},
	{
		.name = "ADC-CSR:trig_pol",
		.offset = ADC_CSR_OFF + 0x010,
	},
	{
		.name = "ADC-CSR:ext_trig_dly",
		.offset = ADC_CSR_OFF + 0x014,
	},
	{
		.name = "ADC-CSR:sw_trig",
		.offset = ADC_CSR_OFF + 0x018,
	},
	{
		.name = "ADC-CSR:shots",
		.offset = ADC_CSR_OFF + 0x01C,
	},
	{
		.name = "ADC-CSR:multi_depth",
		.offset = ADC_CSR_OFF + 0x020,
	},
	{
		.name = "ADC-CSR:trig_pos",
		.offset = ADC_CSR_OFF + 0x024,
	},
	{
		.name = "ADC-CSR:fs_freq",
		.offset = ADC_CSR_OFF + 0x028,
	},
	{
		.name = "ADC-CSR:downsample",
		.offset = ADC_CSR_OFF + 0x02C,
	},
	{
		.name = "ADC-CSR:pre_samples",
		.offset = ADC_CSR_OFF + 0x030,
	},
	{
		.name = "ADC-CSR:post_samples",
		.offset = ADC_CSR_OFF + 0x034,
	},
	{
		.name = "ADC-CSR:samples_cnt",
		.offset = ADC_CSR_OFF + 0x038,
	},
	FA_DBG_REG32_CH(1),
	FA_DBG_REG32_CH(2),
	FA_DBG_REG32_CH(3),
	FA_DBG_REG32_CH(4),
	{
		.name = "ADC-EIC:disable_mask",
		.offset = ADC_EIC_OFF + 0x0,
	},
	{
		.name = "ADC-EIC:enable_mask",
		.offset = ADC_EIC_OFF + 0x4,
	},
	{
		.name = "ADC-EIC:status_mask",
		.offset = ADC_EIC_OFF + 0x8,
	},
	{
		.name = "ADC-EIC:source",
		.offset = ADC_EIC_OFF + 0xC,
	},
	FA_DBG_REG32_TIM(base_time, 0x00),
	FA_DBG_REG32_TIM(time_trig, 0x0C),
	FA_DBG_REG32_TIM(trig_tag, 0x18),
	FA_DBG_REG32_TIM(acq_start_tag, 0x24),
	FA_DBG_REG32_TIM(acq_stop_tag, 0x30),
	FA_DBG_REG32_TIM(acq_end_tag, 0x3C),
};


static void fa_regdump_seq_read_spi(struct fa_dev *fa, struct seq_file *s)
{
	int i, err;

	seq_printf(s, "ADC SPI registers\n");
	seq_printf(s, "Address   Data\n");
	for (i = 0; i < 5; ++i) {
		uint32_t tx, rx;

		tx = 0x8000 | (i << 8);
		err = fa_spi_xfer(fa, FA_SPI_SS_ADC, 16, tx, &rx);
		rx &= 0xFF; /* the value is 8bit */
		if (err)
			seq_printf(s, "A%d %02xh    read failure!\n",
				   i, i);
		else
			seq_printf(s, "A%d %02xh    0x%02x\n",
				   i, i, rx);
	}
}

static int fa_regdump_seq_read(struct seq_file *s, void *data)
{
	struct fa_dev *fa = s->private;

	fa_regdump_seq_read_spi(fa, s);

	return 0;
}


static int fa_regdump_open(struct inode *inode, struct file *file)
{
	return single_open(file, fa_regdump_seq_read, inode->i_private);
}


static const struct file_operations fa_regdump_ops = {
	.owner = THIS_MODULE,
	.open = fa_regdump_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static ssize_t fa_trg_sw_write(struct file *file, const char __user *buf,
                               size_t count, loff_t *ppos)
{
	struct fa_dev *fa = file->private_data;
	int err;

	err = fa_trigger_software(fa);

	return err ? err : count;
}

static const struct file_operations fa_trg_sw_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = fa_trg_sw_write,
};

#define FA_ADC_DATA_PATTERN_CMD_SIZE 16UL
static int fa_data_pattern_adc_write(struct fa_dev *fa, const char __user *buf,
				     size_t count)
{
	char buf_l[FA_ADC_DATA_PATTERN_CMD_SIZE];
	int err;

	memset(buf_l, 0, FA_ADC_DATA_PATTERN_CMD_SIZE);
	err = copy_from_user(buf_l, buf, min(count,
					     FA_ADC_DATA_PATTERN_CMD_SIZE));
	if (err)
		return -EFAULT;

	if ((count == 1 || count == 2)&& buf_l[0] == '0') {
		err = fa_adc_data_pattern_set(fa, 0, 0);
		fa_calib_init(fa);
		return err;
	} else if (count > 2 && buf_l[0] == '1' && buf_l[1] == ' ') {
		uint16_t pattern = 0;

		err = kstrtou16(buf_l + 2, 0, &pattern);
		if (err)
			return err;
		err = fa_adc_data_pattern_set(fa, pattern & 0x3FFF, 1);
		fa_calib_exit(fa);
		return err;
	} else {
		return -EINVAL;
	}
}

static ssize_t fa_data_pattern_write(struct file *file, const char __user *buf,
				     size_t count, loff_t *ppos)
{
	struct fa_dev *fa = file->private_data;

	if (strncmp(buf, "adc ", 4) == 0) {
		int err;

		err = fa_data_pattern_adc_write(fa, buf + 4, count - 4);

		return err ? err : count;
	} else {
		dev_err(&fa->pdev->dev, "Unknown command \"%s\"\n", buf);
		return -EINVAL;
	}
}

static ssize_t fa_data_pattern_read(struct file *file, char __user *buf,
				     size_t count, loff_t *ppos)
{
	struct fa_dev *fa = file->private_data;
	char buf_l[FA_ADC_DATA_PATTERN_CMD_SIZE];
	uint16_t pattern;
	unsigned int enable;
	int err;

        if (*ppos > 0)
		return 0;

        err = fa_adc_data_pattern_get(fa, &pattern, &enable);
	if (err)
		return err;
	snprintf(buf_l, FA_ADC_DATA_PATTERN_CMD_SIZE, "adc %d 0x%02x\n",
		 enable, pattern);
	count = min(count, strlen(buf_l));
	err = copy_to_user(buf, buf_l, count);
	*ppos += count;
	return err ? err : count;
}

static const struct file_operations fa_data_pattern_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = fa_data_pattern_write,
	.read = fa_data_pattern_read,
};

int fa_debug_init(struct fa_dev *fa)
{
	int err;

	fa->dbg_dir = debugfs_create_dir(dev_name(&fa->zdev->head.dev), NULL);
	if (IS_ERR_OR_NULL(fa->dbg_dir)) {
		err = PTR_ERR(fa->dbg_dir);
		dev_err(&fa->zdev->head.dev,
			"Cannot create debugfs directory \"%s\" (%d)\n",
			dev_name(&fa->zdev->head.dev), err);
		return err;
	}

	fa->dbg_reg32.regs = fa_debugfs_reg32;
	fa->dbg_reg32.nregs = ARRAY_SIZE(fa_debugfs_reg32);
	fa->dbg_reg32.base = fa->fa_top_level;
	fa->dbg_reg = debugfs_create_regset32("regs", 0200, fa->dbg_dir,
					      &fa->dbg_reg32);
	if (IS_ERR_OR_NULL(fa->dbg_reg)) {
		err = PTR_ERR(fa->dbg_reg);
		dev_warn(fa->msgdev,
			"Cannot create debugfs file \"regs\" (%d)\n",
			 err);
	}

	fa->dbg_reg_spi = debugfs_create_file("spi-regs", 0444,
					      fa->dbg_dir, fa,
					      &fa_regdump_ops);
	if (IS_ERR_OR_NULL(fa->dbg_reg_spi)) {
		dev_warn(fa->msgdev,
			"Cannot create regdump debugfs file\n");
	}

	fa->dbg_trg_sw = debugfs_create_file("trigger_software", 0200,
					      fa->dbg_dir, fa,
					      &fa_trg_sw_ops);
	if (IS_ERR_OR_NULL(fa->dbg_trg_sw)) {
		dev_warn(&fa->pdev->dev,
			"Cannot create software trigger file\n");
	}

	fa->dbg_data_pattern = debugfs_create_file("data_pattern", 0644,
					      fa->dbg_dir, fa,
					      &fa_data_pattern_ops);
	if (IS_ERR_OR_NULL(fa->dbg_data_pattern)) {
		dev_warn(&fa->pdev->dev,
			"Cannot create ADC data pattern file\n");
	}

	return 0;
}


void fa_debug_exit(struct fa_dev *fa)
{
	debugfs_remove_recursive(fa->dbg_dir);
}
