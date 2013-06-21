/* Copyright 2013 CERN
 * Author: Federico Vaga <federico.vaga@gmail.comZ
 * License: GPLv2
 *
 * This is a simple program to configure the FMC ADC trigger. It is not bug
 * aware because it is only a demo program to show you how you can handle the
 * trigger.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <getopt.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/zio-user.h>
#include <fmcadc-lib.h>

static void fald_help()
{
	printf("\nfald-simple-acq [OPTIONS] <DEVID>\n\n");
	printf("  <DEVID>: hexadecimal identifier (e.g.: \"0x200\")\n");
	printf("  --pre|-p <value>         number of pre samples\n");
	printf("  --post|-P <value>        number of post samples (default: 16)\n");
	printf("  --nshots|-n <value>      number of trigger shots\n");
	printf("  --delay|-d <value>       ticks delay of the trigger\n");
	printf("  --threshold|-t <value>   internal trigger threshold\n");
	printf("  --channel|-c <value>     internal channel to use as trigger (0..3)\n");
	printf("  --external               use external trigger (default: internal)\n");
	printf("  --negative-edge          internal trigger is falling edge\n");
	printf("  --help|-h                show this help\n\n");
}

int main(int argc, char *argv[])
{
	struct fmcadc_buffer buf;
	struct fmcadc_dev *adc;
	struct fmcadc_conf trg, acq;
	static int trgval[FMCADC_N_ATTRIBUTES];
	static struct option options[] = {
		{"pre",required_argument, 0, 'p'},
		{"post",required_argument, 0, 'P'},
		{"nshots",required_argument, 0, 'n'},
		{"delay",required_argument, 0, 'd'},
		{"decimation",required_argument, 0, 'D'},
		{"threshold",required_argument, 0, 't'},
		{"channel",required_argument, 0, 'c'},
		{"negative-edge", no_argument, &trgval[FMCADC_CONF_TRG_POLARITY], 1},
		{"help",no_argument, 0, 'h'},
		{0, 0, 0, 0}
	};
	int opt_index = 0, err = 0, presamples = 0, i;
	unsigned int dev_id = 0;
	char c;

	if (argc == 1) {
		fald_help();
		exit(1);
	}

	/* reset attributes and provide defaults */
	memset(&trg, 0, sizeof(trg));
	trg.type = FMCADC_CONF_TYPE_TRG;
	fmcadc_set_attr(&trg, FMCADC_CONF_TRG_SOURCE, 1); /* external */
	fmcadc_set_attr(&trg, FMCADC_CONF_TRG_POLARITY, 0); /* positive edge */

	memset(&acq, 0, sizeof(acq));
	acq.type = FMCADC_CONF_TYPE_ACQ;
	fmcadc_set_attr(&acq, FMCADC_CONF_ACQ_POST_SAMP, 16);
	fmcadc_set_attr(&acq, FMCADC_CONF_ACQ_N_SHOTS, 1);

	/* Parse options */
	while( (c = getopt_long(argc, argv, "p:P:n:d:D:t:c:h",
						options, &opt_index)) >=0 ){
		switch(c){
		case 'p':
			presamples = atoi(optarg),
			fmcadc_set_attr(&acq, FMCADC_CONF_ACQ_PRE_SAMP,
					presamples);
			break;
		case 'P':
			fmcadc_set_attr(&acq, FMCADC_CONF_ACQ_POST_SAMP,
					atoi(optarg));
			break;
		case 'n':
			fmcadc_set_attr(&acq, FMCADC_CONF_ACQ_N_SHOTS,
					atoi(optarg));
			break;
		case 'd':
			fmcadc_set_attr(&trg, FMCADC_CONF_TRG_DELAY,
					atoi(optarg));
			break;
		case 'D':
			fmcadc_set_attr(&acq, FMCADC_CONF_ACQ_DECIMATION,
					atoi(optarg));
			break;
		case 't':
			fmcadc_set_attr(&trg, FMCADC_CONF_TRG_THRESHOLD,
					atoi(optarg));
			break;
		case 'c':
			/* set internal, and then the channel */
			fmcadc_set_attr(&trg, FMCADC_CONF_TRG_SOURCE, 0);
			fmcadc_set_attr(&trg, FMCADC_CONF_TRG_SOURCE_CHAN,
					atoi(optarg));
			break;
		case 'h':
			fald_help();
			exit(1);
			break;
		}
	}

	if (optind != argc - 1 ) {
		fprintf(stderr, "%s: DEVICE-ID is a mandatory argument\n",
			argv[0]);
		fald_help();
		exit(1);
	} else {
		sscanf(argv[optind], "0x%x", &dev_id);
	}

	printf("Open ADC fmcadc_100MS_4ch_14bit dev_id 0x%04x ...\n", dev_id);
	/* Open the ADC */
	adc = fmcadc_open("fmcadc_100MS_4ch_14bit", dev_id, 0);
	if (!adc) {
		fprintf(stderr, "%s: cannot open device: %s",
			argv[0], fmcadc_strerror(adc, errno));
		exit(1);
	}

	if (strcmp(fmcadc_get_driver_type(adc), "zio")) {
		fprintf(stderr, "%s: not a zio driver, aborting\n", argv[0]);
		exit(1);
	}

	printf("Configuring trigger ...\n");
	/* Configure trigger parameter */
	err = fmcadc_apply_config(adc, 0 , &trg);
	if (err && errno != FMCADC_ENOMASK) {
		fprintf(stderr, "%s: cannot configure trigger: %s\n",
			argv[0], fmcadc_strerror(adc, errno));
		exit(1);
	}

	printf("Configuring acquisition ...\n");
	/* Configure acquisition parameter */
	err = fmcadc_apply_config(adc, 0 , &acq);
	if (err && errno != FMCADC_ENOMASK) {
		fprintf(stderr, "%s: cannot configure acquisition: %s\n",
			argv[0], fmcadc_strerror(adc, errno));
		exit(1);
	}

	printf("Start Acquisition ...\n");
	/* Start acquisition and wait until it completes */
	err = fmcadc_acq_start(adc, 0 , NULL);
	if (err) {
		fprintf(stderr, "%s: cannot start acquisition: %s\n",
			argv[0], fmcadc_strerror(adc, errno));
		exit(1);
	}

	/* Retrieve buffer for each shot */
	for (i = 0; i < acq.value[FMCADC_CONF_ACQ_N_SHOTS]; ++i) {
		struct zio_control *ctrl;
		int j, ch;
		int16_t *data;

		err = fmcadc_request_buffer(adc, &buf, 0, NULL);
		if (err) {
			fprintf(stderr, "%s: shot %i/%i: cannot get a buffer:"
				" %s\n", argv[0], i + i,
				acq.value[FMCADC_CONF_ACQ_N_SHOTS],
				fmcadc_strerror(adc, errno));
			exit(1);
		}
		ctrl = buf.metadata;
		printf("\nRead %d samples from shot %i/%i\n", ctrl->nsamples,
		       i + 1, acq.value[FMCADC_CONF_ACQ_N_SHOTS]);

		/* we lazily know samplesize is 2 bytes and chcount is 4 */
		data = buf.data;
		for (j = 0; j < ctrl->nsamples / 4; j++) {
			printf("%5i     ", j - presamples);
			for (ch = 0; ch < 4; ch++)
				printf("%7i", *(data++));
			printf("\n");
		}
		fmcadc_release_buffer(adc, &buf);
	}

	printf("Acquisition completed\n");
	fmcadc_close(adc);
	exit(0);
}
