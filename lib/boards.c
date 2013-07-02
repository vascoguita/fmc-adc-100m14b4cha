/*
 * All the boards in the library
 *
 * Copyright (C) 2013 CERN (www.cern.ch)
 * Author: Federico Vaga <federico.vaga@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * version 2 as published by the Free Software Foundation or, at your
 * option, any later version.
 */
#include <string.h>
#include <errno.h>
#include "fmcadc-lib.h"
#include "fmcadc-lib-int.h"

#define FMCADC_ZIO_TRG_MASK (1LL << FMCADC_CONF_TRG_SOURCE) |      \
			    (1LL << FMCADC_CONF_TRG_SOURCE_CHAN) | \
			    (1LL << FMCADC_CONF_TRG_THRESHOLD) |   \
			    (1LL << FMCADC_CONF_TRG_POLARITY) |    \
			    (1LL << FMCADC_CONF_TRG_DELAY)
#define FMCADC_ZIO_ACQ_MASK (1LL << FMCADC_CONF_ACQ_N_SHOTS) |     \
			    (1LL << FMCADC_CONF_ACQ_POST_SAMP) |   \
			    (1LL << FMCADC_CONF_ACQ_PRE_SAMP) |    \
			    (1LL << FMCADC_CONF_ACQ_DECIMATION) |  \
			    (1LL << FMCADC_CONF_ACQ_FREQ_HZ) |     \
			    (1LL << FMCADC_CONF_ACQ_N_BITS)
#define FMCADC_ZIO_CHN_MASK (1LL << FMCADC_CONF_CHN_RANGE) |       \
			    (1LL << FMCADC_CONF_CHN_TERMINATION) | \
			    (1LL << FMCADC_CONF_CHN_OFFSET)
#define FMCADC_ZIO_BRD_MASK (1LL << FMCADC_CONF_BRD_STATE_MACHINE_STATUS) | \
			    (1LL << FMCADC_CONF_BRD_N_CHAN)

struct fmcadc_op fa_100ms_4ch_14bit_op = {
	.open = fmcadc_zio_open,
	.open_by_lun = fmcadc_zio_open_by_lun,
	.close = fmcadc_zio_close,
	.start_acquisition = fmcadc_zio_start_acquisition,
	.stop_acquisition = fmcadc_zio_stop_acquisition,
	.apply_config = fmcadc_zio_apply_config,
	.retrieve_config = fmcadc_zio_retrieve_config,
	.request_buffer = fmcadc_zio_request_buffer,
	.release_buffer = fmcadc_zio_release_buffer,
};
struct fmcadc_board_type fmcadc_100ms_4ch_14bit = {
	.name = "fmcadc_100MS_4ch_14bit",
	.devname = "adc-100m14b",
	.driver_type = "zio",
	.capabilities = {
		FMCADC_ZIO_TRG_MASK,
		FMCADC_ZIO_ACQ_MASK,
		FMCADC_ZIO_CHN_MASK,
		FMCADC_ZIO_BRD_MASK,
	},
	.fa_op = &fa_100ms_4ch_14bit_op,
};


/* fmcadc_open
 * @name: name of the device type to open
 * @dev_id: device identificator of a particular device connected to the system
 */
struct fmcadc_dev *fmcadc_open(char *name, unsigned int dev_id,
				      unsigned long buffersize,
				      unsigned int nbuffer,
				      unsigned long flags)
{
	struct fmcadc_dev *dev = NULL;
	int i, found = 0;

	/* name cannot be NULL */
	if (!name)
		return NULL;

	/* Look in the list of supported board if the "name" board is there */
	for (i = 0; i < __FMCADC_SUPPORTED_BOARDS_LAST_INDEX; ++i) {
		if (!strcmp(name, fmcadc_board_types[i]->name)) {
			found = 1;
			break;
		}
	}
	if (!found) {
		errno = ENODEV;
		return NULL; /* Not found */
	}

	/* The library supports this board */
	if (fmcadc_board_types[i]->fa_op && fmcadc_board_types[i]->fa_op->open) {
		dev = fmcadc_board_types[i]->fa_op->open(fmcadc_board_types[i],
							 dev_id, flags);
	} else {
		errno = FMCADC_ENOP;
	}

	return dev;
}

/*
 * fmcadc_open_by_lun
 * @name: name of the device type to open
 * @lun: Logical Unit Number of the device
 *
 * TODO
 */
struct fmcadc_dev *fmcadc_open_by_lun(char *name, int lun,
					     unsigned long buffersize,
					     unsigned int nbuffer,
					     unsigned long flags)
{
	if (!name)
		return NULL;

	return NULL;
}

/*
 * fmcadc_close
 * @dev: the device to close
 */
int fmcadc_close(struct fmcadc_dev *dev)
{
	struct fmcadc_gid *b = (void *)dev;

	if (!dev) {
		/* dev cannot be NULL */
		errno = EINVAL;
		return -1;
	}

	if (b->board->fa_op->close) {
		return b->board->fa_op->close(dev);
	} else {
		errno = FMCADC_ENOP;
		return -1;
	}
}

