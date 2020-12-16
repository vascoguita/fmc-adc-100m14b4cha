// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * a simple output tool to make a burst on a parallel port
 *
 * Copyright (C) 2012 CERN (www.cern.ch)
 * Author: Alessandro Rubini <rubini@gnudd.com>
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <glob.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

static char git_version[] = "version: " GIT_VERSION;

/* Returns the numer of microsecond timer ticks (Tomasz Wlostowski) */
static int64_t get_tics()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (int64_t)tv.tv_sec * 1000 * 1000 + tv.tv_usec;
}

/* Microsecond-accurate delay-to */
static void delay_to(int64_t until)
{
	while (get_tics() < until)
		;
}

static void print_version(char *pname)
{
	printf("%s %s\n", pname, git_version);
}

int main(int argc, char **argv)
{
	unsigned int addr;
	int fd, count, usec, ret;
	int64_t tics;

	if ((argc == 2) &&
	    (!strcmp(argv[1], "-V") || !strcmp(argv[1], "--version"))) {
		print_version(argv[0]);
		exit(0);
	}

	if (argc != 4) {
		fprintf(stderr,
			"%s: Use \"%s <hexaddr> <count> <period-usec>\"\n",
			argv[0], argv[0]);
		exit(1);
	}
	if (sscanf(argv[1], "%x", &addr) != 1) {
		fprintf(stderr, "%s: wrong hex \"%s\"\n", argv[0], argv[1]);
		exit(1);
	}
	if (sscanf(argv[2], "%i", &count) != 1) {
		fprintf(stderr, "%s: wrong count \"%s\"\n", argv[0], argv[2]);
		exit(1);
	}
	if (sscanf(argv[3], "%i", &usec) != 1) {
		fprintf(stderr, "%s: wrong period \"%s\"\n", argv[0], argv[3]);
		exit(1);
	}
	fprintf(stderr, "%s: using port 0x%x, %i pulses, period %i us\n",
		argv[0], addr, count, usec);

	fd = open("/dev/port", O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "%s: /dev/port: %s\n", argv[0],
			strerror(errno));
		exit(1);
	}

	tics = get_tics();
	do {
		char b[]={0x00, 0xff};

		lseek(fd, addr, SEEK_SET);
		ret = write(fd, b + 1, 1);
		if (ret < 0)
			break;
		lseek(fd, addr, SEEK_SET);
		ret = write(fd, b + 0, 1);
		if (ret < 0)
			break;

		if (count > 1) {
			tics += usec;
			delay_to(tics);
		}
	} while (--count);
	return ret;
}

