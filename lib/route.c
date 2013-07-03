/*
 * Routing public functions to device-specific code
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

#include "fmcadc-lib.h"
#include "fmcadc-lib-int.h"

int fmcadc_acq_start(struct fmcadc_dev *dev,
			     unsigned int flags,
			     struct timeval *timeout)
{
	struct fmcadc_gid *g = (void *)dev;
	const struct fmcadc_board_type *b = g->board;

	return b->fa_op->acq_start(dev, flags, timeout);
}

int fmcadc_acq_poll(struct fmcadc_dev *dev, unsigned int flags,
		    struct timeval *timeout)
{
	struct fmcadc_gid *g = (void *)dev;
	const struct fmcadc_board_type *b = g->board;

	return b->fa_op->acq_poll(dev, flags, timeout);
}

int fmcadc_acq_stop(struct fmcadc_dev *dev, unsigned int flags)
{
	struct fmcadc_gid *g = (void *)dev;
	const struct fmcadc_board_type *b = g->board;

	return b->fa_op->acq_stop(dev, flags);
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
 * @nsamples: size of this buffer
 * @alloc: user-defined allocator
 * @flags:
 * @timeout: it can be used to specify how much time wait that a buffer is
 *           ready. This value follow the select() policy: NULL to wait until
 *           acquisition is over; {0, 0} to return immediately without wait;
 *           {x, y} to wait acquisition end for a specified time
 */
struct fmcadc_buffer *fmcadc_request_buffer(struct fmcadc_dev *dev,
					    int nsamples,
					    void *(*alloc)(size_t),
					    unsigned int flags,
					    struct timeval *timeout)
{
	struct fmcadc_gid *b = (void *)dev;

	if (!dev) {
		/* dev cannot be NULL */
		errno = EINVAL;
		return NULL;
	}

	if (b->board->fa_op->request_buffer) {
		return b->board->fa_op->request_buffer(dev, nsamples,
						       alloc, flags, timeout);
	} else {
		/* Unsupported */
		errno = FMCADC_ENOP;
		return NULL;
	}
}

/*
 * fmcadc_release_buffer
 * @dev: device that generate the buffer
 * @buf: buffer to release
 */
int fmcadc_release_buffer(struct fmcadc_dev *dev, struct fmcadc_buffer *buf,
			  void (*free)(void *))
{
	struct fmcadc_gid *b = (void *)dev;

	if (!dev || !buf) {
		/* dev and buf cannot be NULL */
		errno = EINVAL;
		return -1;
	}

	if (b->board->fa_op->release_buffer) {
		return b->board->fa_op->release_buffer(dev, buf, free);
	} else {
		/* Unsupported */
		errno = FMCADC_ENOP;
		return -1;
	}
}
