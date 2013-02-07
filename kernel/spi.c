/*
 * SPI access to fine-delay internals
 *
 * Copyright (C) 2012 CERN (www.cern.ch)
 * Author: Tomasz Wlostowski <tomasz.wlostowski@cern.ch>
 * Author: Alessandro Rubini <rubini@gnudd.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * version 2 as published by the Free Software Foundation or, at your
 * option, any later version.
 */

#include <linux/jiffies.h>
#include <linux/io.h>
#include <linux/delay.h>
#include "fmc-adc.h"

/* SPI register */
#define FA_SPI_RX(x) (x * 4)
#define FA_SPI_TX(x) (x * 4)
#define FA_SPI_CTRL 0x10
#define FA_SPI_DIV 0x14
#define FA_SPI_CS 0x18

#define FA_SPI_REG(_off) (FA_SPI_MEM_OFF + _off)

/* SPI control register fields mask */
#define FA_SPI_CTRL_BUF		0x007F
#define FA_SPI_CTRL_BUSY	0x0100
#define FA_SPI_CTRL_Rx_NEG	0x0200
#define FA_SPI_CTRL_Tx_NEG	0x0400
#define FA_SPI_CTRL_LSB		0x0800
#define FA_SPI_CTRL_IE		0x1000
#define FA_SPI_CTRL_ASS		0x2000


int fa_spi_xfer(struct fa_dev *fa, int cs, int num_bits,
		uint32_t tx, uint32_t *rx)
{
	uint32_t ctrl = 0;
	unsigned long j = jiffies + HZ;
	int err = 0;

	/* Put out value in the T0 register*/
	fmc_writel(fa->fmc, (tx & (num_bits-1)), FA_SPI_MEM_OFF + FA_SPI_TX(0));
	/* Configure SPI controller */
	ctrl |= FA_SPI_CTRL_ASS | /* Automatic handle ChipSelect*/
		num_bits; /* Transfer num_bits bits */
	fmc_writel(fa->fmc, ctrl, FA_SPI_REG(FA_SPI_CTRL));
	/* Set Chip Select */
	fmc_writel(fa->fmc, (1 << cs), FA_SPI_REG(FA_SPI_CTRL_ASS));
	/* Start transfer */
	ctrl |= FA_SPI_CTRL_BUSY;
	fmc_writel(fa->fmc, ctrl, FA_SPI_REG(FA_SPI_CTRL));
	/* Wait transfer complete */
	while(fmc_readl(fa->fmc, FA_SPI_REG(FA_SPI_CTRL)) & FA_SPI_CTRL_BUSY) {
		if (jiffies > j) {
			dev_err(&fa->fmc->dev, "SPI transfer error\n");
			err = -EIO;
			goto out;
		}
	}
	/* Transfer compleate, read data */
	*rx = fmc_readl(fa->fmc, FA_SPI_REG(FA_SPI_RX(0))) & (num_bits-1);
	dev_dbg(&fa->fmc->dev, "SPI transfer CS %d, NBIT %d, TX 0x%x RX 0x%x\n",
		cs, num_bits, tx, *rx);
out:
	/* Clear Chip Select */
	fmc_writel(fa->fmc, 0, FA_SPI_REG(FA_SPI_CTRL_ASS));

	return err;
}


int fa_spi_init(struct fa_dev *fa)
{
	/* nothing to do */
	return 0;
}

void fa_spi_exit(struct fa_dev *fa)
{
	/* nothing to do */
}
