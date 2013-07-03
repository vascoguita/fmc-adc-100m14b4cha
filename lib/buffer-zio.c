/*
 * ZIO-wide buffer management (device-independent)
 *
 * Copyright (C) 2013 CERN (www.cern.ch)
 * Author: Federico Vaga <federico.vaga@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * version 2 as published by the Free Software Foundation or, at your
 * option, any later version.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/select.h>
#include <linux/zio-user.h>

#include "fmcadc-lib.h"
#include "fmcadc-lib-int.h"


struct zio_control *fmcadc_zio_read_ctrl(struct __fmcadc_dev_zio *fa)
{
	struct zio_control *ctrl;
	int i;

	ctrl = malloc(sizeof(struct zio_control));
	if (!ctrl)
		return NULL;

	i = read(fa->fdc, ctrl, sizeof(struct zio_control));
	switch (i) {
	case -1:
		fprintf(stderr, "control read: %s\n",
			strerror(errno));
		return NULL;
	case 0:
		fprintf(stderr, "control read: unexpected EOF\n");
		return NULL;
	default:
		fprintf(stderr, "ctrl read: %i bytes (expected %zi)\n",
			i, sizeof(ctrl));
		return NULL;
	case sizeof(struct zio_control):
		break; /* ok */
	}

	return ctrl;
}
static void fmcadc_zio_release_ctrl(struct zio_control *ctrl)
{
	free(ctrl);
}

static void *fmcadc_zio_read_data(struct __fmcadc_dev_zio *fa,
				  unsigned int datalen)
{
	void *data;
	int i;

	data = malloc(datalen);
	if (!data)
		return NULL;

	i = read(fa->fdd, data, datalen);
	if (i > 0 && i <= datalen){
		if (i < datalen)
			fprintf(stderr, "data read: %i bytes (expected %i)\n",
				i, datalen);
		return data;
	} else {
		if (i == 0)
			fprintf(stderr, "data read: unexpected EOF\n");
		else
			fprintf(stderr, "data read: %s\n", strerror(errno));
		return NULL;
	}
}
static void fmcadc_zio_release_data(void *data)
{
	free(data);
}
struct fmcadc_buffer *fmcadc_zio_request_buffer(struct fmcadc_dev *dev,
						int nsamples,
						void *(*alloc)(size_t),
						unsigned int flags)
{
	struct __fmcadc_dev_zio *fa = to_dev_zio(dev);
	struct fmcadc_buffer *buf;
	struct zio_control *ctrl;
	void *data;
	fd_set set;
	int err;
	unsigned int len;

	buf = calloc(1, sizeof(*buf));
	if (!buf)
		return NULL;

	/* So, first sample and blocking read. Wait.. */
	FD_ZERO(&set);
	FD_SET(fa->fdc, &set);
	err = select(fa->fdc + 1, &set, NULL, NULL, NULL);
	if (err == 0) {
		errno = EAGAIN;
		return NULL; /* no free, but the function is wrong generally */
	} else if (err < 0) {
		return NULL;
	}

	/* Ready to read */
	ctrl = fmcadc_zio_read_ctrl(fa);
	if (!ctrl)
		goto out_ctrl;

	len = (ctrl->nsamples * ctrl->ssize);
	data = fmcadc_zio_read_data(fa, len);
	if (!data)
		goto out_data;

	buf->data = data;
	buf->metadata = (void *) ctrl;
	return buf;

out_data:
	fmcadc_zio_release_ctrl(ctrl);
out_ctrl:
	return NULL;
}

int fmcadc_zio_fill_buffer(struct fmcadc_dev *dev,
			   struct fmcadc_buffer *buf,
			   unsigned int flags,
			   struct timeval *timeout)
{
	return -1;
}

struct fmcadc_timestamp *fmcadc_zio_tstamp_buffer(struct fmcadc_buffer *buf,
						  struct fmcadc_timestamp *ts)
{
	return NULL;
}

int fmcadc_zio_release_buffer(struct fmcadc_dev *dev,
				     struct fmcadc_buffer *buf,
				     void (*free_fn)(void *))
{
	fmcadc_zio_release_ctrl(buf->metadata);
	fmcadc_zio_release_data(buf->data);
	free(buf);
	return 0;
}
