/*
 * ZIO-specific configuration (mostly device-independent)
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
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "fmcadc-lib.h"
#include "fmcadc-lib-int.h"

#define FMCADC_CONF_GET 0
#define FMCADC_CONF_SET 1

static int __fa_zio_sysfs_get(char *path, uint32_t *resp)
{
	FILE *f = fopen(path, "r");
	int ret;

	if (!f)
		return -1;
	errno = EINVAL; /* in case it's a scanf-only error */
	if (fscanf(f, "%i", resp) != 1)
		ret = -1;
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

static int fa_zio_sysfs_get(struct __fmcadc_dev_zio *fa, char *name,
		uint32_t *resp)
{
	char pathname[128];
	int ret;

	snprintf(pathname, sizeof(pathname), "%s/%s", fa->sysbase, name);
	ret = __fa_zio_sysfs_get(pathname, resp);
	if (!(fa->flags & FMCADC_FLAG_VERBOSE))
		return ret;
	/* verbose tail */
	if (ret)
		fprintf(stderr, "lib-fmcadc: Error reading %s (%s)\n",
			pathname, strerror(errno));
	else
		fprintf(stderr, "lib-fmcadc: %08x %5i <- %s\n",
			(int)*resp, (int)*resp, pathname);
	return ret;
}

/*
 * fa_zio_sysfs_set
 * @fa: device owner of the attribute
 * @name: relative path to the sysfs attribute within the device
 * @value: value to set in the sysfs attribute
 */
int fa_zio_sysfs_set(struct __fmcadc_dev_zio *fa, char *name,
		uint32_t *value)
{
	char pathname[128];
	int ret;

	snprintf(pathname, sizeof(pathname), "%s/%s", fa->sysbase, name);
	ret = __fa_zio_sysfs_set(pathname, value);
	if (!(fa->flags & FMCADC_FLAG_VERBOSE))
		return ret;
	/* verbose tail */
	if (ret)
		fprintf(stderr, "lib-fmcadc: Error writing %s (%s)\n",
			pathname, strerror(errno));
	else
		fprintf(stderr, "lib-fmcadc: %08x %5i -> %s\n",
			(int)*value, (int)*value, pathname);
	return ret;
}

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
	case FMCADC_CONF_BRD_STATE_MACHINE_STATUS:
		if (!direction)
			return fa_zio_sysfs_get(fa, "cset0/fsm-state",
						value);
		errno = EINVAL;
		return -1;
	case FMCADC_CONF_BRD_N_CHAN:
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

	for (i = 0; i < __FMCADC_CONF_LEN; ++i) {
		if (!(conf->mask & (1LL << i)))
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

int fmcadc_zio_apply_config(struct fmcadc_dev *dev, unsigned int flags,
		struct fmcadc_conf *conf)
{
	struct __fmcadc_dev_zio *fa = to_dev_zio(dev);

	return fmcadc_zio_config(fa, flags, conf, FMCADC_CONF_SET);
}

int fmcadc_zio_retrieve_config(struct fmcadc_dev *dev,
		struct fmcadc_conf *conf)
{
	struct __fmcadc_dev_zio *fa = to_dev_zio(dev);

	return fmcadc_zio_config(fa, 0, conf, FMCADC_CONF_GET);
}
