/*
 * SPEC interface for the fmc-adc driver
 *
 * Copyright (C) 2012 CERN (www.cern.ch)
 * Author: Federico Vaga <federico.vaga@gmail.com>
 *	Copied from fine-delay fd-spec.c
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/io.h>

#include "spec.h"
#include "fmc-adc.h"

static int fa_is_valid(int bus, int devfn)
{
	/* FIXME: restrict to some of the spec devices with moduleparam */
	pr_debug("%s: %x %x\n", __func__, bus, devfn);
	return 1;
}

int fa_spec_init(void)
{
	struct spec_dev *dev;
	int ret, success = 0, retsave = 0, err = 0;

	/* Scan the list and see what is there. Take hold of everything */
	list_for_each_entry(dev, &spec_list, list) {
		if (!fa_is_valid(dev->pdev->bus->number, dev->pdev->devfn))
			continue;
		pr_debug("%s: init %04x:%04x (%pR - %p)\n", __func__,
		       dev->pdev->bus->number, dev->pdev->devfn,
		       dev->area[0], dev->remap[0]);
		ret = fa_probe(dev);
		if (ret < 0) {
			retsave = ret;
			err++;
		} else {
			success++;
		}
	}
	if (err) {
		pr_err("%s: Setup of %i boards failed (%i succeeded)\n",
		       KBUILD_MODNAME, err, success);
		pr_err("%s: last error: %i\n", KBUILD_MODNAME, retsave);
	}
	if (success) {
		/* At least one board has been successfully initialized */
		return 0;
	}
	return retsave; /* last error code */
}

void fa_spec_exit(void)
{
	struct spec_dev *dev;

	list_for_each_entry(dev, &spec_list, list) {
		if (!fa_is_valid(dev->pdev->bus->number, dev->pdev->devfn))
			continue;
		pr_debug("%s: release %04x:%04x (%pR - %p)\n", __func__,
		       dev->pdev->bus->number, dev->pdev->devfn,
		       dev->area[0], dev->remap[0]);
		fa_remove(dev);
	}
}
