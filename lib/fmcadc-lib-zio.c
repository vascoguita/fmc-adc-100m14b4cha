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
#define _GNU_SOURCE
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

#define FMCADCLIB_INTERNAL
#include "fmcadc-lib.h"

#define ZIO_DEV_PATH "/dev/zio"
#define ZIO_SYS_PATH "/sys/bus/zio/devices"

#define FMCADC_CONF_GET 0
#define FMCADC_CONF_SET 1

/* Internal structure (ZIO specific) */
struct __fmcadc_dev_zio {
	unsigned int cset;
	int fdc;
	int fdd;
	int dev_id;
	char *devbase;
	char *sysbase;
	/* Mandatory field */
	struct fmcadc_gid gid;
};

#define offsetof(TYPE, MEMBER) ((size_t) &((TYPE *)0)->MEMBER)
#define container_of(ptr, type, member) ({			\
	const typeof( ((type *)0)->member ) *__mptr = (ptr);	\
	(type *)( (char *)__mptr - offsetof(type,member) );})
#define to_dev_zio(_dev) (container_of(dev, struct __fmcadc_dev_zio, gid))

/* * * * * * * * * ZIO specific function * * * * * * * * * * */
static int __fa_zio_sysfs_get(char *path, uint32_t *resp)
{
	FILE *f = fopen(path, "r");

	if (!f)
		return -1;
	errno = 0;
	if (fscanf(f, "%i", resp) != 1) {
		fclose(f);
		if (!errno)
			errno = EINVAL;
		return -1;
	}
	fclose(f);
	return 0;
}

static int __fa_zio_sysfs_set(char *path, uint32_t *value)
{
	char s[16];
	int fd, ret, len;

	len = sprintf(s, "%i\n", *value);
	fd = open(path, O_WRONLY);
	if (fd < 0)
		return -1;
	ret = write(fd, s, len);
	close(fd);
	if (ret < 0)
		return -1;
	if (ret == len)
		return 0;
	errno = EINVAL;
	return -1;
}

/* And these two for the board structure */
static int fa_zio_sysfs_get(struct __fmcadc_dev_zio *fa, char *name,
		uint32_t *resp)
{
	char pathname[128];

	sprintf(pathname, "%s/%s", fa->sysbase, name);
	return __fa_zio_sysfs_get(pathname, resp);
}

static int fa_zio_sysfs_set(struct __fmcadc_dev_zio *fa, char *name,
		uint32_t *value)
{
	char pathname[128];

	sprintf(pathname, "%s/%s", fa->sysbase, name);
	return __fa_zio_sysfs_set(pathname, value);
}

static int fmcadc_zio_stop_acquisition(struct fmcadc_dev *dev,
		unsigned int flags);

/* * * * * * * * * * * * * * * * Handle board * * * * * * * * * * * * * * * */
static struct fmcadc_dev *fmcadc_zio_open(const struct fmcadc_board_type *dev,
					  unsigned int dev_id,
					  unsigned int details)
{
	struct __fmcadc_dev_zio *fa = NULL;
	struct stat st;
	char *syspath, *devpath, fname[128];
	int udev_zio_dir = 1;

	if (strlen(dev->devname) > 12) {
		fprintf(stderr,
				"%s: name \"%s\" is too long. ZIO's name are 12byte\n",
				__func__, dev->devname);
		return NULL ;
	}

	/* check if device exists by looking in ZIO sysfs */
	asprintf(&syspath, "%s/%s-%04x", ZIO_SYS_PATH, dev->devname, dev_id);
	if (stat(syspath, &st)) {
		goto out_fa_stat;
	}

	/* Check where are ZIO char devices x*/
	if (stat(ZIO_DEV_PATH, &st)) {
		/*
		 * ZIO driver are not in /dev/zio, but in /dev with all other
		 * drivers
		 */
		udev_zio_dir = 0;
	}
	asprintf(&devpath, "%s/%s-%04x", (udev_zio_dir ? ZIO_DEV_PATH : "/dev"),
			dev->devname, dev_id);

	/* Path exists, so device is there */

	fa = malloc(sizeof *fa);
	if (!fa) {
		goto out_fa_alloc;
	}
	fa->sysbase = syspath;
	fa->devbase = devpath;
	fa->cset = details;

	/* Open char devices */
	sprintf(fname, "%s-0-i-ctrl", fa->devbase);
	fa->fdc = open(fname, O_RDONLY);
	sprintf(fname, "%s-0-i-data", fa->devbase);
	fa->fdd = open(fname, O_RDONLY);
	if (fa->fdc < 0 || fa->fdd < 0)
	    goto out_fa_open;

	fa->gid.board = dev;
	return (void *) &fa->gid;

out_fa_open:
	free(fa);
out_fa_alloc:
	free(devpath);
out_fa_stat:
	free(syspath);

	return NULL ;
}

static struct fmcadc_dev *fmcadc_zio_open_by_lun(char *name, int lun)
{
	/* TODO implement*/
	return NULL ;
}
static int fmcadc_zio_close(struct fmcadc_dev *dev)
{
	struct __fmcadc_dev_zio *fa = to_dev_zio(dev);

	/* If char device are open, close it */
	if (fa->fdc >= 0)
		close(fa->fdc);
	fa->fdc = -1;
	if (fa->fdd >= 0)
		close(fa->fdd);
	fa->fdd = -1;

	/* Stop active acquisition */
	fmcadc_zio_stop_acquisition(dev, 0);

	free(fa->sysbase);
	free(fa->devbase);
	free(fa);
	return 0;
}
/* Handle acquisition */
static int fmcadc_zio_start_acquisition(struct fmcadc_dev *dev,
		unsigned int flags, struct timeval *timeout)
{
	struct __fmcadc_dev_zio *fa = to_dev_zio(dev);
	uint32_t cmd;
	fd_set set;
	int err;


	cmd = 1;
	err = fa_zio_sysfs_set(fa, "cset0/fsm-command", &cmd);
	if (err) {
		/*
		 * It returns error when we cannot start
		 */
		return err;
	}

	/* So, first sample and blocking read. Wait.. */
	FD_ZERO(&set);
	FD_SET(fa->fdc, &set);
	err = select(fa->fdc + 1, &set, NULL, NULL, timeout);
	switch (err) {
	case 0:
		errno = EAGAIN;
		return -1;
	case 1:
		return 0;
	default:
		return err;
	}
}
static int fmcadc_zio_stop_acquisition(struct fmcadc_dev *dev,
		unsigned int flags)
{
	struct __fmcadc_dev_zio *fa = to_dev_zio(dev);
	uint32_t cmd = 2;

	return fa_zio_sysfs_set(fa, "cset0/fsm-command", &cmd);
}


/* * * * * * * * * * * * * * * Handle configuration * * * * * * * * * * * * */
static int fmcadc_zio_config_trg(struct __fmcadc_dev_zio *fa,
		unsigned int index, uint32_t *value, unsigned int direction)
{
	switch (index) {
	case FMCADC_CONF_TRG_SOURCE:
		if (direction)
			return fa_zio_sysfs_set(fa, "cset0/trigger/external",
					value);
		else
			return fa_zio_sysfs_get(fa, "cset0/trigger/external",
					value);
		break;
	case FMCADC_CONF_TRG_SOURCE_CHAN:
		if (direction)
			return fa_zio_sysfs_set(fa, "cset0/trigger/int-channel",
					value);
		else
			return fa_zio_sysfs_get(fa, "cset0/trigger/int-channel",
					value);
		break;
	case FMCADC_CONF_TRG_THRESHOLD:
		if (direction)
			return fa_zio_sysfs_set(fa,
					"cset0/trigger/int-threshold", value);
		else
			return fa_zio_sysfs_get(fa,
					"cset0/trigger/int-threshold", value);
		break;
	case FMCADC_CONF_TRG_POLARITY:
		if (direction)
			return fa_zio_sysfs_set(fa, "cset0/trigger/polarity",
					value);
		else
			return fa_zio_sysfs_get(fa, "cset0/trigger/polarity",
					value);
		break;
	case FMCADC_CONF_TRG_DELAY:
		if (direction)
			return fa_zio_sysfs_set(fa, "cset0/trigger/delay",
					value);
		else
			return fa_zio_sysfs_get(fa, "cset0/trigger/delay",
					value);
		break;
	default:
		errno = FMCADC_ENOCAP;
		return -1;
	}
}
static int fmcadc_zio_config_acq(struct __fmcadc_dev_zio *fa,
		unsigned int index, uint32_t *value, unsigned int direction)
{
	switch (index) {
	case FMCADC_CONF_ACQ_N_SHOTS:
		if (direction)
			return fa_zio_sysfs_set(fa, "cset0/trigger/nshots",
					value);
		else
			return fa_zio_sysfs_get(fa, "cset0/trigger/nshots",
					value);
		break;
	case FMCADC_CONF_ACQ_POST_SAMP:
		if (direction)
			return fa_zio_sysfs_set(fa,
					"cset0/trigger/post-samples", value);
		else
			return fa_zio_sysfs_get(fa,
					"cset0/trigger/post-samples", value);
		break;
	case FMCADC_CONF_ACQ_PRE_SAMP:
		if (direction)
			return fa_zio_sysfs_set(fa, "cset0/trigger/pre-samples",
					value);
		else
			return fa_zio_sysfs_get(fa, "cset0/trigger/pre-samples",
					value);
		break;
	case FMCADC_CONF_ACQ_DECIMATION:
		if (direction)
			return fa_zio_sysfs_set(fa, "cset0/sample-decimation",
					value);
		else
			return fa_zio_sysfs_get(fa, "cset0/sample-decimation",
					value);
		break;
	case FMCADC_CONF_ACQ_FREQ_HZ:
		if (direction) {
			errno = FMCADC_ENOSET;
			return -1;
		} else {
			*value = 100000000; /* 100Mhz */
			return 0;
		}
		break;
	case FMCADC_CONF_ACQ_N_BITS:
		if (direction) {
			errno = FMCADC_ENOSET;
			return -1;
		} else {
			*value = 14;
			return 0;
		}
		break;
	default:
		errno = FMCADC_ENOCAP;
		return -1;
	}
}
static int fmcadc_zio_config_chn(struct __fmcadc_dev_zio *fa, unsigned int ch,
		unsigned int index, uint32_t *value, unsigned int direction)
{
	char path[128];

	switch (index) {
	case FMCADC_CONF_CHN_RANGE:
		sprintf(path, "cset%d/ch%d-vref", fa->cset, ch);
		if (direction)
			return fa_zio_sysfs_set(fa, path, value);
		else
			return fa_zio_sysfs_get(fa, path, value);
		break;
	case FMCADC_CONF_CHN_TERMINATION:
		sprintf(path, "cset%d/ch%d-50ohm-term", fa->cset, ch);
		if (direction)
			return fa_zio_sysfs_set(fa, path, value);
		else
			return fa_zio_sysfs_get(fa, path, value);
		break;
	case FMCADC_CONF_CHN_OFFSET:
		sprintf(path, "cset%d/ch%d-offset", fa->cset, ch);
		if (direction)
			return fa_zio_sysfs_set(fa, path, value);
		else
			return fa_zio_sysfs_get(fa, path, value);
		break;
	default:
		errno = FMCADC_ENOCAP;
		return -1;
	}

	return 0;
}
static int fmcadc_zio_config_brd(struct __fmcadc_dev_zio *fa,
		unsigned int index, uint32_t *value, unsigned int direction)
{
	switch (index) {
	case FMCADC_BOARD_STATE_MACHINE_STATUS:
		if (!direction)
			return fa_zio_sysfs_get(fa, "cset0/fsm-state",
						value);
		errno = EINVAL;
		return -1;
	case FMCADC_BOARD_N_CHAN:
		if (!direction) {
			*value = 4;
			return 0;
		}
		errno = EINVAL;
		return -1;
	default:
		errno = FMCADC_ENOCAP;
		return -1;
	}
}

static int fmcadc_zio_config(struct __fmcadc_dev_zio *fa, unsigned int flags,
		struct fmcadc_conf *conf, unsigned int direction)
{

	int err, i;

	for (i = 0; i < FMCADC_N_ATTRIBUTES; ++i) {
		if (!(conf->mask & (1 << i)))
			continue;

		/* Parameter to configure */
		switch (conf->type) {
		case FMCADC_CONF_TYPE_TRG:
			err = fmcadc_zio_config_trg(fa, i, &conf->value[i],
					direction);
			break;
		case FMCADC_CONF_TYPE_ACQ:
			err = fmcadc_zio_config_acq(fa, i, &conf->value[i],
					direction);
			break;
		case FMCADC_CONF_TYPE_CHN:
			if (conf->route_to > 3) {
				errno = FMCADC_ENOCHAN;
				return -1;
			}
			err = fmcadc_zio_config_chn(fa, conf->route_to,
					i, &conf->value[i],
					direction);
			break;
		case FMCADC_CONT_TYPE_BRD:
			err = fmcadc_zio_config_brd(fa, i, &conf->value[i],
					direction);
			break;
		default:
			errno = FMCADC_ENOCFG;
			return -1;
		}
		if (err)
			return err;
	}

	return 0;
}

static int fmcadc_zio_apply_config(struct fmcadc_dev *dev, unsigned int flags,
		struct fmcadc_conf *conf)
{
	struct __fmcadc_dev_zio *fa = to_dev_zio(dev);

	return fmcadc_zio_config(fa, flags, conf, FMCADC_CONF_SET);
}

static int fmcadc_zio_retrieve_config(struct fmcadc_dev *dev,
		struct fmcadc_conf *conf)
{
	struct __fmcadc_dev_zio *fa = to_dev_zio(dev);

	return fmcadc_zio_config(fa, 0, conf, FMCADC_CONF_GET);
}

/* * * * * * * * * * * * * * * * * Handle buffer * * * * * * * * * * * * * * */
static int fmcadc_zio_request_buffer(struct fmcadc_dev *dev,
		struct fmcadc_buffer *buf, unsigned int flags,
		struct timeval *timeout)
{
	return -1;
}
static int fmcadc_zio_release_buffer(struct fmcadc_dev *dev,
		struct fmcadc_buffer *buf)
{
	return -1;
}

#define FMCADC_ZIO_TRG_MASK (1 << FMCADC_CONF_TRG_SOURCE) |      \
			    (1 << FMCADC_CONF_TRG_SOURCE_CHAN) | \
			    (1 << FMCADC_CONF_TRG_THRESHOLD) |   \
			    (1 << FMCADC_CONF_TRG_POLARITY) |    \
			    (1 << FMCADC_CONF_TRG_DELAY)
#define FMCADC_ZIO_ACQ_MASK (1 << FMCADC_CONF_ACQ_N_SHOTS) |     \
			    (1 << FMCADC_CONF_ACQ_POST_SAMP) |   \
			    (1 << FMCADC_CONF_ACQ_PRE_SAMP) |    \
			    (1 << FMCADC_CONF_ACQ_DECIMATION) |  \
			    (1 << FMCADC_CONF_ACQ_FREQ_HZ) |     \
			    (1 << FMCADC_CONF_ACQ_N_BITS)
#define FMCADC_ZIO_CHN_MASK (1 << FMCADC_CONF_CHN_RANGE) |       \
			    (1 << FMCADC_CONF_CHN_TERMINATION) | \
			    (1 << FMCADC_CONF_CHN_OFFSET)
#define FMCADC_ZIO_BRD_MASK (1 << FMCADC_BOARD_STATE_MACHINE_STATUS) | \
			    (1 << FMCADC_BOARD_N_CHAN)

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
	.devname = "fmc-adc",
	.driver_type = "zio",
	.capabilities = {
		FMCADC_ZIO_TRG_MASK,
		FMCADC_ZIO_ACQ_MASK,
		FMCADC_ZIO_CHN_MASK,
		FMCADC_ZIO_BRD_MASK,
	},
	.fa_op = &fa_100ms_4ch_14bit_op,
};

