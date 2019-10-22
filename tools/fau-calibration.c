// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2019 CERN (www.cern.ch)
 * Author: Federico Vaga <federico.vaga@cern.ch>
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <endian.h>

#include <fmc-adc-100m14b4cha.h>

static char options[] = "hf:o:D:b";
static const char help_msg[] =
	"Usage: fau-calibration [options]\n"
	"\n"
	"It reads calibration data from a file that contains it in binary\n"
	"form and it shows it on STDOUT in binary form or in human readable\n"
	"one (default).\n"
	"This could be used to change the ADC calibration data at runtime\n"
	"by redirectiong the binary output of this program to the proper \n"
	"sysfs binary attribute\n"
	"Rembember that we expect all values to be little endian\n"
	"\n"
	"General options:\n"
	"-h                 Print this message\n"
	"-b                 Show Calibration in binary form \n"
	"\n"
	"Read options:\n"
	"-f                 Source file where to read calibration data from\n"
	"-o                 Offset in bytes within the file (default 0)\n"
	"Write options:\n"
	"-D                 FMC ADC Target Device ID\n"
	"\n";

/**
 * Read calibration data from file
 * @path: file path
 * @calib: calibration data
 * @offset: offset in file
 *
 * Return: number of bytes read
 */
static int fau_calibration_read(char *path, struct fa_calib *calib,
				off_t offset)
{
	int fd;
	int ret = 0;
	uint16_t *data16 = (uint16_t *)calib;
	int i;

	fd = open(path, O_RDONLY);
	if (fd < 0)
		return -1;
	ret = lseek(fd, offset, SEEK_SET);
	if (ret >= 0)
		ret = read(fd, calib, sizeof(*calib));
	close(fd);

	/* Fix endianess */
	for (i = 0; i < sizeof(*calib) / sizeof(uint16_t); ++i)
		data16[i] = le16toh(data16[i]);

	return ret;
}

static void fau_calibration_dump_stanza(struct fa_calib_stanza *stanza)
{
	fprintf(stdout, "  temperature: %f C\n",
		stanza->temperature * 0.01);
	fprintf(stdout, "  gain: [0x%04"PRIx16", 0x%04"PRIx16", 0x%04"PRIx16", 0x%04"PRIx16"]\n",
		stanza->gain[0],
		stanza->gain[1],
		stanza->gain[2],
		stanza->gain[3]);
	fprintf(stdout, "  offset: [0x%04"PRIx16", 0x%04"PRIx16", 0x%04"PRIx16", 0x%04"PRIx16"]\n",
		stanza->offset[0],
		stanza->offset[1],
		stanza->offset[2],
		stanza->offset[3]);
}

/**
 * Print calibration data on stdout in humand readable format
 * @calib: calibration data
 */
static void fau_calibration_dump_human(struct fa_calib *calib)
{
	fputs("ADC Range 10V\n", stdout);
	fau_calibration_dump_stanza(&calib->adc[FA100M14B4C_RANGE_10V]);
	fputs("DAC Range 10V\n", stdout);
	fau_calibration_dump_stanza(&calib->dac[FA100M14B4C_RANGE_10V]);

	fputs("ADC Range 1V\n", stdout);
	fau_calibration_dump_stanza(&calib->adc[FA100M14B4C_RANGE_1V]);
	fputs("DAC Range 1V\n", stdout);
	fau_calibration_dump_stanza(&calib->dac[FA100M14B4C_RANGE_1V]);

	fputs("ADC Range 100mV\n", stdout);
	fau_calibration_dump_stanza(&calib->adc[FA100M14B4C_RANGE_100mV]);
	fputs("DAC Range 100mV\n", stdout);
	fau_calibration_dump_stanza(&calib->dac[FA100M14B4C_RANGE_100mV]);
	fputc('\n', stdout);
}

/**
 * Print binary calibration data on stdout
 * @calib: calibration data
 */
static int fau_calibration_dump_machine(struct fa_calib *calib)
{
	return write(fileno(stdout), calib, sizeof(*calib));
}

/**
 * Write calibration data to device
 * @devid: Device ID
 * @calib: calibration data
 *
 * Return: number of bytes wrote
 */
static int fau_calibration_write(unsigned int devid, struct fa_calib *calib)
{
	struct fa_calib calib_cp;
	char path[128];
	uint16_t *data16;
	int i;
	int fd;
	int ret;

	sprintf(path,
		"/sys/bus/zio/devices/adc-100m14b-%04x/calibration_data",
		devid);

	/* Fix endianess */
	memcpy(&calib_cp, calib, sizeof(calib_cp));
	data16 = (uint16_t *) &calib_cp;
	for (i = 0; i < sizeof(calib_cp) / sizeof(uint16_t); ++i)
		data16[i] = htole16(data16[i]);

	fd = open(path, O_WRONLY);
	if (fd < 0)
		return -1;
	ret = write(fd, &calib_cp, sizeof(calib_cp));
	close(fd);

	return ret;
}

int main(int argc, char *argv[])
{
	char c;
	int ret;
	char *path = NULL;
	unsigned int offset = 0;
	unsigned int devid = 0;
	int show_bin = 0, write = 0;
	struct fa_calib calib;

	while ((c = getopt(argc, argv, options)) != -1) {
		switch (c) {
		default:
		case 'h':
			fprintf(stderr, help_msg);
			exit(EXIT_SUCCESS);
		case 'D':
			ret = sscanf(optarg, "0x%x", &devid);
			if (ret != 1) {
				fprintf(stderr,
					"Invalid devid %s\n",
					optarg);
				exit(EXIT_FAILURE);
			}
			write = 1;
			break;
		case 'f':
			path = optarg;
			break;
		case 'o':
			ret = sscanf(optarg, "0x%x", &offset);
			if (ret != 1) {
				ret = sscanf(optarg, "%u", &offset);
				if (ret != 1) {
					fprintf(stderr,
						"Invalid offset %s\n",
						optarg);
					exit(EXIT_FAILURE);
				}
			}
			break;
		case 'b':
			show_bin = 1;
			break;
		}
	}

	if (!path) {
		fputs("Calibration file is mandatory\n", stderr);
		exit(EXIT_FAILURE);
	}

	/* Read EEPROM file */
	ret = fau_calibration_read(path, &calib, offset);
	if (ret < 0) {
		fprintf(stderr, "Can't read calibration data from '%s'. %s\n",
			path, strerror(errno));
		exit(EXIT_FAILURE);
	}
	if (ret != sizeof(calib)) {
		fprintf(stderr,
			"Can't read all calibration data from '%s'. %s\n",
			path, strerror(errno));
		exit(EXIT_FAILURE);
	}

	/* Show calibration data*/
	if (show_bin)
		fau_calibration_dump_machine(&calib);
	else if(!write)
		fau_calibration_dump_human(&calib);

	/* Write calibration data */
	if (write) {
		ret = fau_calibration_write(devid, &calib);
		if (ret < 0) {
			fprintf(stderr,
				"Can't write calibration data to '0x%x'. %s\n",
				devid, strerror(errno));
			exit(EXIT_FAILURE);
		}
		if (ret != sizeof(calib)) {
			fprintf(stderr,
				"Can't write all calibration data to '0x%x'. %s\n",
				devid, strerror(errno));
			exit(EXIT_FAILURE);
		}
	}
	exit(EXIT_SUCCESS);
}
