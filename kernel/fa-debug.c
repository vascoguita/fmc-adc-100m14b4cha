// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright CERN 2018-2019
 * Author: Federico Vaga <federico.vaga@cern.ch>
 */

#include "fmc-adc-100m14b4cha.h"


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


int fa_debug_init(struct fa_dev *fa)
{
	fa->reg_dump = debugfs_create_file(dev_name(&fa->zdev->head.dev), 0444,
					   NULL, fa, &fa_regdump_ops);
	if (IS_ERR_OR_NULL(fa->reg_dump)) {
		dev_err(fa->msgdev,
			"Cannot create regdump debugfs file\n");
	}

	return 0;
}


void fa_debug_exit(struct fa_dev *fa)
{
	debugfs_remove_recursive(fa->reg_dump);
}
