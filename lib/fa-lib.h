/*
 * The "official" fmc-adc API
 *
 * Copyright (C) 2013 CERN (www.cern.ch)
 * Author: Alessandro Rubini <rubini@gnudd.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * version 2 as published by the Free Software Foundation or, at your
 * option, any later version.
 */
#ifndef __FMCADC_H__
#define __FMCADC_H__
#include <stdint.h>


/* Opaque data type used as token */
struct fa_board;

struct fa_time {
	uint64_t utc;
	uint32_t coarse;
	uint32_t frac;
	uint32_t seq_id;
	uint32_t channel;
};


/*
 * Please see the manual for the meaning of arguments and return values
 */

extern int fa_init(void);
extern void fa_exit(void);

extern struct fa_board *fa_open(int offset, int dev_id);
extern struct fa_board *fa_open_by_lun(int lun);
extern int fa_close(struct fa_board *);

#ifdef FMC_ADC_INTERNAL /* Libray users should ignore what follows */
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

/* Internal structure */
struct __fa_board {
	int dev_id;
	char *devbase;
	char *sysbase;
	int fac; /* The control char device */
	int fad; /* The data char device */
};

static inline int fa_is_verbose(void)
{
	return getenv("FMC_ADC_LIB_VERBOSE") != 0;
}

#define __define_board(b, ub)	struct __fa_board *b = (void *)(ub)

static inline int __fa_sysfs_get(char *path, uint32_t *resp)
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

static inline int __fa_sysfs_set(char *path, uint32_t *value)
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
static inline int fa_sysfs_get(struct __fa_board *b, char *name,
			       uint32_t *resp)
{
	char pathname[128];

	sprintf(pathname, "%s/%s", b->sysbase, name);
	return __fa_sysfs_get(pathname, resp);
}

static inline int fa_sysfs_set(struct __fa_board *b, char *name,
			       uint32_t *value)
{
	char pathname[128];

	sprintf(pathname, "%s/%s", b->sysbase, name);
	return __fa_sysfs_set(pathname, value);
}

static inline int __fa_command(struct __fa_board *b, uint32_t cmd)
{
	return fa_sysfs_set(b, "command", &cmd);
}



#endif /* FMC_ADC_INTERNAL */
#endif /* __FMCADC_H__ */
