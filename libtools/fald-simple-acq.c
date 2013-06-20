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
	printf("\nfald-simple-acq [OPTIONS] 0x<DEVICE ID>\n\n");
	printf("  <DEVICE>: hexadecimal string which represent the device "
	       "identificator of an fmc-adc\n");
	printf("  --pre|-p <value>: number of pre samples\n");
	printf("  --post|-P <value>: number of post samples\n");
	printf("  --nshots|-n <value>: number of trigger shots\n");
	printf("  --delay|-d <value>: set the ticks delay of the trigger\n");
	printf("  --threshold|-t <value>: set internal trigger threshold\n");
	printf("  --channel|-c <value>: select the internal channel as "
	       "trigger (0-based index)\n");
	printf("  --external: set to external trigger. The default is the "
	       "internal trigger.\n");
	printf("  --negative-edge: set internal trigger polarity to negative "
	       "edge. The default\n                    is positive edge.\n");
	printf("  --help|-h: show this help\n\n");
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
		{"src-channel",required_argument, 0, 'c'},
		{"external", no_argument, &trgval[FMCADC_CONF_TRG_SOURCE], 1},
		{"negative-edge", no_argument, &trgval[FMCADC_CONF_TRG_POLARITY], 1},
		{"help",no_argument, 0, 'h'},
		{0, 0, 0, 0}
	};
	int opt_index = 0, err = 0, i;
	unsigned int dev_id = 0;
	char c;

	if (argc == 1) {
		fald_help();
		exit(1);
	}

	/* reset attribute's mask */
	trg.mask = 0;
	acq.mask = 0;

	/* Parse options */
	while( (c = getopt_long(argc, argv, "p:P:n:d:D:t:c:h",
						options, &opt_index)) >=0 ){
		switch(c){
		case 'p':
			fmcadc_set_attr(&acq, FMCADC_CONF_ACQ_PRE_SAMP,
					atoi(optarg));
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
		printf("Error: DEVICE ID is a mandatory argument\n");
		fald_help();
		exit(1);
	} else {
		sscanf(argv[optind], "0x%x", &dev_id);
	}

	printf("Open ADC fmcadc_100MS_4ch_14bit dev_id 0x%04x ...\n", dev_id);
	/* Open the ADC */
	adc = fmcadc_open("fmcadc_100MS_4ch_14bit", dev_id, 0);
	if (!adc) {
		printf("Cannot open: (%d) %s", errno, fmcadc_strerror(adc, errno));
		exit(1);
	}

	printf("Configuring trigger ...\n");
	/* Configure trigger parameter */
	trg.type = FMCADC_CONF_TYPE_TRG;
	/* Set internal/external trigger according to trgval flag */
	fmcadc_set_attr(&trg, FMCADC_CONF_TRG_SOURCE,
					trgval[FMCADC_CONF_TRG_SOURCE]);
	/* Set trigger polarity negative/positive according to trgval flag */
	fmcadc_set_attr(&trg, FMCADC_CONF_TRG_POLARITY,
					trgval[FMCADC_CONF_TRG_POLARITY]);
	err = fmcadc_apply_config(adc, 0 , &trg);
	if (err && errno != FMCADC_ENOMASK) {
		printf("Cannot apply trigger configuration: (%d) %s\n",
			errno, fmcadc_strerror(adc, errno));
		exit(1);
	}

	printf("Configuring acquisition ...\n");
	/* Configure acquisition parameter */
	acq.type = FMCADC_CONF_TYPE_ACQ;
	err = fmcadc_apply_config(adc, 0 , &acq);
	if (err && errno != FMCADC_ENOMASK) {
		printf("Cannot apply acquisition configuration: (%d) %s\n",
			errno, fmcadc_strerror(adc, errno));
		exit(1);
	}

	printf("Start Acquisition ...\n");
	/* Start acquisition and wait until it completes */
	err = fmcadc_acq_start(adc, 0 , NULL);
	if (err) {
		printf("Cannot apply acquisition configuration: (%d) %s\n",
			errno, fmcadc_strerror(adc, errno));
		exit(1);
	}

	/* Retrieve buffer for each shot */
	for (i = 0; i < acq.value[FMCADC_CONF_ACQ_N_SHOTS]; ++i) {
		err = fmcadc_request_buffer(adc, &buf, 0, NULL);
		if (err) {
			printf("Cannot retrieve a buffer: (%d) %s\n",
				errno, fmcadc_strerror(adc, errno));
			exit(1);
		}

		if (strcmp(fmcadc_get_driver_type(adc), "zio") == 0) {
			struct zio_control *ctrl = buf.metadata;
			int len, j;

			len = ctrl->nsamples * ctrl->ssize;
			printf("\nRead %d bytes from shot %d\n", len, i + 1);
			for (j = 0; j < len; j++) {
				if (!(j & 0xf))
					printf("Data:");
				printf(" %02x", buf.data[j]);
				if ((j & 0xf) == 0xf || j == len - 1)
					putchar('\n');
			}
		}
		fmcadc_release_buffer(adc, &buf);
	}

	printf("Acquisition completed\n");
	fmcadc_close(adc);
	exit(0);
}
