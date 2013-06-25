/*
 * Initializing and cleaning up the fmc adc library
 *
 * Copyright (C) 2013 CERN (www.cern.ch)
 * Author: Federico Vaga <federico.vaga@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * version 2 as published by the Free Software Foundation or, at your
 * option, any later version.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>

#define FMCADCLIB_INTERNAL
#include "fmcadc-lib.h"

const struct fmcadc_board_type
		*fmcadc_board_types[__FMCADC_SUPPORTED_BOARDS_LAST_INDEX] = {
	&fmcadc_100ms_4ch_14bit,
};


/* * * * * * * * * * * * * * * * * Handle Device * * * * * * * * * * * * * */
/* fmcadc_open
 * @name: name of the device type to open
 * @dev_id: device identificator of a particular device connected to the system
 */
struct fmcadc_dev *fmcadc_open(char *name, unsigned int dev_id,
			       unsigned int details)
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
							 dev_id, details);
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
struct fmcadc_dev *fmcadc_open_by_lun(char *name, int lun)
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


/* * * * * * * * * * * * * * * Handle Acquisition * * * * * * * * * * * * * */
/*
 * fmcadc_acq_start
 * @dev: device where to start acquiring
 * @flags:
 * @timeout: it can be used to specify how much time wait that acquisition is
 *           over. This value follow the select() policy: NULL to wait until
 *           acquisition is over; {0, 0} to return immediately without wait;
 *           {x, y} to wait acquisition end for a specified time
 */
int fmcadc_acq_start(struct fmcadc_dev *dev,
			     unsigned int flags,
			     struct timeval *timeout)
{
	struct fmcadc_gid *b = (void *)dev;

	if (!dev) {
		/* dev cannot be NULL */
		errno = EINVAL;
		return -1;
	}

	if (b->board->fa_op->start_acquisition) {
		return b->board->fa_op->start_acquisition(dev, flags, timeout);
	} else {
		errno = FMCADC_ENOP;
		return -1;
	}
}

/*
 * fmcadc_acq_stop
 * @dev: device where to stop acquisition
 * @flags:
 */
int fmcadc_acq_stop(struct fmcadc_dev *dev, unsigned int flags)
{
	struct fmcadc_gid *b = (void *)dev;

	if (!dev) {
		/* dev cannot be NULL */
		errno = EINVAL;
		return -1;
	}

	if (b->board->fa_op->stop_acquisition) {
		return b->board->fa_op->stop_acquisition(dev, flags);
	} else {
		errno = FMCADC_ENOP;
		return -1;
	}
}


/* * * * * * * * * * * * * * Handle Configuration * * * * * * * * * * * * * */
/*
 * fmcadc_apply_config
 * @dev: device to configure
 * @flags:
 * @conf: configuration to apply on device.
 */
int fmcadc_apply_config(struct fmcadc_dev *dev, unsigned int flags,
			struct fmcadc_conf *conf)
{
	struct fmcadc_gid *b = (void *)dev;
	uint64_t cap_mask;

	if (!conf || !dev) {
		/* conf and dev cannot be NULL*/
		errno = EINVAL;
		return -1;
	}
	if (!conf->mask) {
		errno = FMCADC_ENOMASK;
		return -1; /* Nothing to do */
	}
	cap_mask = b->board->capabilities[conf->type];
	if ((cap_mask & conf->mask) != conf->mask) {
		/* Unsupported capabilities */
		fprintf(stderr, "Apply Config, wrong mask 0x%llx (0x%llx)",
			conf->mask, cap_mask);
		errno = FMCADC_ENOCAP;
		return -1;
	}

	if (b->board->fa_op->apply_config) {
		/* Apply config */
		return b->board->fa_op->apply_config(dev, flags, conf);
	} else {
		/* Unsupported */
		errno = FMCADC_ENOP;
		return -1;
	}
}

/*
 * fmcadc_retrieve_config
 * @dev: device where retireve configuration
 * @flags:
 * @conf: configuration to retrieve. The mask tell which value acquire, then
 *        the library will acquire and set the value in the "value" array
 */
int fmcadc_retrieve_config(struct fmcadc_dev *dev, struct fmcadc_conf *conf)
{
	struct fmcadc_gid *b = (void *)dev;
	uint64_t cap_mask;

	if (!conf || !dev) {
		/* conf and dev cannot be NULL*/
		errno = EINVAL;
		return -1;
	}
	if (!conf->mask) {
		errno = FMCADC_ENOMASK;
		return -1; /* Nothing to do */
	}
	cap_mask = b->board->capabilities[conf->type];
	if ((cap_mask & conf->mask) != conf->mask) {
		/* Unsupported capabilities */
		fprintf(stderr, "Apply Config, wrong mask 0x%llx (0x%llx)",
			conf->mask, cap_mask);
		errno = FMCADC_ENOCAP;
		return -1;
	}

	if (b->board->fa_op->retrieve_config) {
		/* Apply config */
		return b->board->fa_op->retrieve_config(dev, conf);
	} else {
		/* Unsupported */
		errno = FMCADC_ENOP;
		return -1;
	}
}

/*
 * fmcadc_request_buffer
 * @dev: device where look for a buffer
 * @buf: where store the buffer. The user must allocate this structure.
 * @flags:
 * @timeout: it can be used to specify how much time wait that a buffer is
 *           ready. This value follow the select() policy: NULL to wait until
 *           acquisition is over; {0, 0} to return immediately without wait;
 *           {x, y} to wait acquisition end for a specified time
 */
int fmcadc_request_buffer(struct fmcadc_dev *dev,
			  struct fmcadc_buffer *buf,
			  unsigned int flags,
			  struct timeval *timeout)
{
	struct fmcadc_gid *b = (void *)dev;

	if (!dev || !buf) {
		/* dev and buf cannot be NULL */
		errno = EINVAL;
		return -1;
	}

	if (b->board->fa_op->request_buffer) {
		return b->board->fa_op->request_buffer(dev, buf, flags, timeout);
	} else {
		/* Unsupported */
		errno = FMCADC_ENOP;
		return -1;
	}
}

/*
 * fmcadc_release_buffer
 * @dev: device that generate the buffer
 * @buf: buffer to release
 */
int fmcadc_release_buffer(struct fmcadc_dev *dev, struct fmcadc_buffer *buf)
{
	struct fmcadc_gid *b = (void *)dev;

	if (!dev || !buf) {
		/* dev and buf cannot be NULL */
		errno = EINVAL;
		return -1;
	}

	if (b->board->fa_op->release_buffer) {
		return b->board->fa_op->release_buffer(dev, buf);
	} else {
		/* Unsupported */
		errno = FMCADC_ENOP;
		return -1;
	}
}


/* * * * * * * * * * * * * * * * Utilities * * * * * * * * * * * * * * * * */
/*
 * fmcadc_strerror
 * @dev: device for which you want to know the meaning of the error
 * @errnum: error number
 */
char *fmcadc_strerror(struct fmcadc_dev *dev, int errnum)
{
	struct fmcadc_gid *b = (void *)dev;
	char *str = NULL;

	if (!dev || !errnum) {
		/* dev and buf cannot be NULL */
		return NULL;
	}

	if (errnum >= FMCADC_ENOP && errnum <= FMCADC_ENOMASK) {
		switch (errnum) {
		case FMCADC_ENOP:
			str = "Operation not supported";
			break;
		case FMCADC_ENOCAP:
			str = "Capabilities not supported";
			break;
		case FMCADC_ENOCFG:
			str = "Configuration type not supported";
			break;
		case FMCADC_ENOGET:
			str = "Cannot get capabilities information";
			break;
		case FMCADC_ENOSET:
			str = "Cannot set capabilities information";
			break;
		case FMCADC_ENOCHAN:
			str = "Invalid channel";
			break;
		case FMCADC_ENOMASK:
			str = "Missing configuration mask";
			break;
		}
		goto out;
	}

	if (!str && b->board->fa_op->strerror)
		str = b->board->fa_op->strerror(errnum);
	if (!str)
		str = strerror(errnum);
out:
	return str;
}

/*
 * fmcadc_get_driver_type
 * @dev: device which want to know the driver type
 */
char *fmcadc_get_driver_type(struct fmcadc_dev *dev)
{
	struct fmcadc_gid *b = (void *)dev;

	return b->board->driver_type;
}
