// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright 2012-2019 CERN
 * Author: Federico Vaga <federico.vaga@gmail.com>
 *
 * This is a simple program to configure the FMC ADC trigger. It is not bug
 * aware because it is only a demo program to show you how you can handle the
 * trigger.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <getopt.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <limits.h>
#include <fcntl.h>
#include <errno.h>

static char git_version[] = "version: " GIT_VERSION;

#define path_len 200
#define buf_len 50
#define base_len 40
/* user will edit by adding the device name */
char basepath[base_len] = "/sys/bus/zio/devices";

enum fau_attribute {
	FAU_TRG_EN,
	FAU_TRG_PRE,
	FAU_TRG_PST,
	FAU_TRG_RE_EN,
	FAU_TRG_EXT,
	FAU_SW_TRG_EN,
	FAU_TRG_DLY,
	FAU_TRG_THR,
	FAU_TRG_CHN,
	FAU_TRG_POL,
	FAU_TRIG_NUM_ATTR,
};

const char *attribute[] = {
	[FAU_TRG_EN] = "/cset0/trigger/enable",
	[FAU_TRG_PRE] = "/cset0/trigger/pre-samples",
	[FAU_TRG_PST] = "/cset0/trigger/post-samples",
	[FAU_TRG_RE_EN] = "/cset0/trigger/nshots",
	[FAU_TRG_EXT] = "/cset0/trigger/external",
	[FAU_SW_TRG_EN] = "/cset0/trigger/sw-trg-enable",
	[FAU_TRG_DLY] = "/cset0/trigger/delay",
	[FAU_TRG_THR] = "/cset0/trigger/int-threshold",
	[FAU_TRG_CHN] = "/cset0/trigger/int-channel",
	[FAU_TRG_POL] = "/cset0/trigger/polarity",
};

/* Write a sysfs attribute */
int fau_write_attribute(enum fau_attribute attr, uint32_t val)
{
	int  ret, fd;
	char buf[buf_len], fullpath[path_len];

	/* convert val to string */
	snprintf(buf, buf_len, "%u",val);
	/* build the attribute path */
	if (path_len > 0)
		snprintf(fullpath, path_len, "%s%s", basepath, attribute[attr]);
	/* Write the attribute */
	printf("Writing %s in %s\n", buf, fullpath);
	fd = open(fullpath, O_WRONLY);
	if (fd < 0)
		return -ENOENT;
	ret = write(fd, buf, strnlen(buf, buf_len));
	close(fd);
	return ret;
}

static void fau_help()
{
	printf("\nfau-trg-config [OPTIONS] <DEVICE>\n\n");
	printf("  <DEVICE>: ZIO name of the device to use\n");
	printf("  --pre|-p <value>: number of pre samples\n");
	printf("  --post|-P <value>: number of pre samples\n");
	printf("  --nshots|-n <value>: number of trigger shots\n");
	printf("  --delay|-d <value>: set the ticks delay of the trigger\n");
	printf("  --threshold|-t <value>: set internal trigger threshold\n");
	printf("  --channel|-c <value>: select the internal channel as "
	       "trigger\n");
	printf("  --external: set to external trigger. The default is the "
	       "internal trigger.\n");
	printf("  --negative-edge: set internal trigger polarity to negative "
	       "edge. The default\n                    is positive edge.\n");
	printf("  --enable-sw-trg: enable the software trigger. By default is "
	       "disabled.\n");
	printf("  --disable-hw-trg: disable the hardware trigger. By default "
	       "is enabled\n");
	printf("  --force: force all attribute to the program default\n");
	printf("  --version|-V: print version information\n");
	printf("  --help|-h: show this help\n\n");
	printf("NOTE: The software trigger works only if also hardware trigger "
	       "is enabled\n\n");
}

static void print_version(char *pname)
{
	printf("%s %s\n", pname, git_version);
}


static long strtol_or_die(const char *arg)
{
	long val = strtol(arg, NULL, 0);

	if ((errno == ERANGE && (val == LONG_MAX || val == LONG_MIN)) || (errno != 0 && val == 0)) {
		fprintf(stderr, "Can't convert \"%s\" to integer\n", optarg);
		exit(EXIT_FAILURE);
	}

	return val;
}

int main(int argc, char *argv[])
{
	/* default attribute */
	const static int attrdef[FAU_TRIG_NUM_ATTR] = {1, 0, 0, 0, 0, 0, 0, 0, 0 ,0};
	/* getop attribute */
	static int attrval[FAU_TRIG_NUM_ATTR] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
	static int force = 0;
	static struct option options[] = {
		{"pre",required_argument, 0, 'p'},
		{"post",required_argument, 0, 'P'},
		{"nshots",required_argument, 0, 'n'},
		{"delay",required_argument, 0, 'd'},
		{"threshold",required_argument, 0, 't'},
		{"negative-edge",no_argument, &attrval[FAU_TRG_POL], 1},
		{"channel",required_argument, 0, 'c'},
		{"external", no_argument, &attrval[FAU_TRG_EXT], 1},
		{"enable-sw-trg", no_argument, &attrval[FAU_SW_TRG_EN], 1},
		{"disable-hw-trg", no_argument, &attrval[FAU_TRG_EN], 0},
		{"force", no_argument, &force, 1},
		{"version",no_argument, 0, 'V'},
		{"help",no_argument, 0, 'h'},
		{0, 0, 0, 0}
	};
	int i, opt_index = 0, err = 0;
	char c;

	if (argc == 1) {
		fau_help();
		exit(1);
	}

	while( (c = getopt_long(argc, argv, "p:P:n:d:t:c:Vh",
						options, &opt_index)) >=0 ){
		switch(c){
		case 'p':
			attrval[FAU_TRG_PRE] = strtol_or_die(optarg);
			break;
		case 'P':
			attrval[FAU_TRG_PST] = strtol_or_die(optarg);
			break;
		case 'n':
			attrval[FAU_TRG_RE_EN] = strtol_or_die(optarg);
			break;
		case 'd':
			attrval[FAU_TRG_DLY] = strtol_or_die(optarg);
			break;
		case 't':
			attrval[FAU_TRG_THR] = strtol_or_die(optarg);
			break;
		case 'c':
			attrval[FAU_TRG_CHN] = strtol_or_die(optarg);
			break;
		case 'V':
			print_version(argv[0]);
			exit(1);
		case 'h':
			fau_help();
			exit(1);
			break;
		}
	}

	if (optind != argc - 1 ) {
		fprintf(stderr, "%s: DEVICE-ID is a mandatory argument\n",
			argv[0]);
		fau_help();
		exit(1);
	}

	strncat(basepath, argv[optind], base_len - strlen(basepath));
	printf("Sysfs path to device is: %s\n", basepath);

	for (i = 0; i < FAU_TRIG_NUM_ATTR; ++i) {
		int cur_val = attrval[i];
		if (cur_val == -1) {
			if (force)
				cur_val = attrdef[i];
			else
				continue; /* skip unsetted attribute */
		}
		err = fau_write_attribute(i, cur_val);
		if (err) {
			fprintf(stderr, "%s: error %d for attribute %d\n",
				argv[0], err, i);
			exit(1);
		}
	}

	exit(0);
}
