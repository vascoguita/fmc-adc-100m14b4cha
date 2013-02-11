/*
 * Initializing and cleaning up the fmc adc library
 *
 * Copyright (C) 2013 CERN (www.cern.ch)
 * Author: Federico Vaga <federico.vaga@gmail.com>
 *
 * Code mainly copied from fdelay-init.c from the fine-delay mezzanine
 * repository, author: Alessandro Rubini <rubini@gnudd.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * version 2 as published by the Free Software Foundation or, at your
 * option, any later version.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <glob.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <linux/zio.h>
#include <linux/zio-user.h>
#define FMC_ADC_INTERNAL
#include "fa-lib.h"

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

static struct __fa_board *fa_boards;
static int fa_nboards;

/* Init the library: return the number of boards found */
int fa_init(void)
{
	glob_t glob_dev, glob_sys;
	struct __fa_board *b;
	int i, j;
	uint32_t v;

	/* Look for boards in /dev: old and new pathnames: only one matches */
	glob("/dev/fmc-adc-*-0-0-ctrl", 0, NULL, &glob_dev);
	glob("/dev/zio/fmc-adc-*-0-0-ctrl", GLOB_APPEND, NULL, &glob_dev);
	glob("/dev/zio-fmc-adc-*-0-0-ctrl", GLOB_APPEND, NULL, &glob_dev);
	glob("/dev/zio/zio-fmc-adc-*-0-0-ctrl", GLOB_APPEND, NULL, &glob_dev);

	/* And look in /sys as well */
	glob("/sys/bus/zio/devices/fmc-adc-*", 0, NULL, &glob_sys);
	glob("/sys/bus/zio/devices/zio-fmc-adc-*", GLOB_APPEND , NULL, &glob_sys);
	assert(glob_dev.gl_pathc == glob_sys.gl_pathc);

	/* Allocate as needed */
	fa_nboards = glob_dev.gl_pathc;
	if (!fa_nboards) {
		fa_boards = NULL;
		return 0;
	}
	fa_boards = calloc(glob_dev.gl_pathc, sizeof(fa_boards[0]));
	if (!fa_boards) {
		globfree(&glob_dev);
		globfree(&glob_sys);
		return -1;
	}

	for (i = 0, b = fa_boards; i < fa_nboards; i++, b++) {
		b->sysbase = strdup(glob_sys.gl_pathv[i]);
		b->devbase = strdup(glob_dev.gl_pathv[i]);
		/* trim the "-0-0-ctrl" at the end */
		b->devbase[strlen(b->devbase) - strlen("-0-0-ctrl")] = '\0';
		/* extract dev_id */
		sscanf(b->sysbase, "%*[^f]fmc-adc-%x", &b->dev_id);
		b->fac = -1;
		b->fad = -1;
		if (fa_is_verbose()) {
			fprintf(stderr, "%s: %04x %s %s\n", __func__,
				b->dev_id, b->sysbase, b->devbase);
		}
	}
	globfree(&glob_dev);
	globfree(&glob_sys);

	/* Now, if at least one board is there, check the version */
	if (fa_nboards == 0)
		return 0;

	return fa_nboards;
}

/* Free and check */
void fa_exit(void)
{
	struct __fa_board *b;
	int i, err;

	for (i = 0, err = 0, b = fa_boards; i < fa_nboards; i++, b++) {
		if (b->fac >= 0) {
			close(b->fac);
			b->fac = -1;
			err++;
		}
		if (b->fad >= 0) {
			close(b->fad);
			b->fad = -1;
			err++;
		}
		if (err)
			fprintf(stderr, "%s: device %s was still open\n",
				__func__, b->devbase);
		free(b->sysbase);
		free(b->devbase);
	}
	if(fa_nboards)
		free(fa_boards);
}

/* Open one specific device. -1 arguments mean "not installed" */
struct fa_board *fa_open(int offset, int dev_id)
{
	struct __fa_board *b = NULL;
	uint32_t nsamples = 1;
	int i;

	if (offset >= fa_nboards) {
		errno = ENODEV;
		return NULL;
	}
	if (offset >= 0) {
		b = fa_boards + offset;
		if (dev_id >= 0 && dev_id != b->dev_id) {
			errno = EINVAL;
			return NULL;
		}
		goto found;
	}
	if (dev_id < 0) {
		errno = EINVAL;
		return NULL;
	}
	for (i = 0, b = fa_boards; i < fa_nboards; i++, b++)
		if (b->dev_id == dev_id)
			goto found;
	errno = ENODEV;
	return NULL;

found:
	/* Trim all block sizes to 1 sample (i.e. 4 bytes) */
	fa_sysfs_set(b, "fd-input/trigger/nsamples", &nsamples);
	fa_sysfs_set(b, "fd-ch1/trigger/nsamples", &nsamples);
	fa_sysfs_set(b, "fd-ch2/trigger/nsamples", &nsamples);
	fa_sysfs_set(b, "fd-ch3/trigger/nsamples", &nsamples);
	fa_sysfs_set(b, "fd-ch4/trigger/nsamples", &nsamples);

	return (void *)b;
}

/* Open one specific device by logical unit number (CERN/CO-like) */
struct fa_board *fa_open_by_lun(int lun)
{
	ssize_t ret;
	char dev_id_str[4];
	char path_pattern[] = "/dev/fmc-adc.%d";
	char path[sizeof(path_pattern) + 1];
	int dev_id;

	ret = snprintf(path, sizeof(path), path_pattern, lun);
	if (ret < 0 || ret >= sizeof(path)) {
		errno = EINVAL;
		return NULL;
	}
	ret = readlink(path, dev_id_str, sizeof(dev_id_str));
	if (sscanf(dev_id_str, "%4x", &dev_id) != 1) {
		errno = ENODEV;
		return NULL;
	}
	return fa_open(-1, dev_id);
}

int fa_close(struct fa_board *userb)
{
	__define_board(b, userb);


	close(b->fac);
	b->fac = -1;
	close(b->fad);
	b->fad = -1;

	return 0;

}

