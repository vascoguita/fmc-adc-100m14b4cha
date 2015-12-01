/*
 * Copyright CERN 2013, GNU GPL 2 or later.
 * Author: Alessandro Rubini
 */

#include "fmcadc-lib.h"

const char * const libfmcadc_version_s = "libfmcadc version: " GIT_VERSION;
const char * const libfmcadc_zio_version_s = "libfmcadc is using zio version: " ZIO_GIT_VERSION;

/* We currently do nothing in init/exit. We might check /proc/meminfo... */
int fmcadc_init(void)
{
	return 0;
}

void fmcadc_exit(void)
{
	return;
}
