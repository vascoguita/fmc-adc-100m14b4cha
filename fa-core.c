/*
 * core fmc-adc driver
 *
 * Copyright (C) 2012 CERN (www.cern.ch)
 * Author: Federico Vaga <federico.vaga@gmail.com>
 *		Copied from fine-delay fd-core.c
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include "spec.h"
#include "fmc-adc.h"

int fa_probe(struct spec_dev *spec)
{
	struct spec_fa *fa;
	int err;

	pr_info("%s:%d\n", __func__, __LINE__);
	fa = kzalloc(sizeof(struct spec_fa), GFP_KERNEL);
	if (!fa)
		return -ENOMEM;

	spec->sub_priv = fa;
	fa->spec = spec;
	fa->base = spec->remap[0];

	/* Initliaze sub-system (FIXME only ZIO at the moment) */
	err = fa_zio_init(fa);
	if (err) {
		kfree(fa);
		return err;
	}
	return 0;
}
void fa_remove(struct spec_dev *dev)
{
	fa_zio_exit(dev->sub_priv);
	kfree(dev->sub_priv);
}

static int fa_init(void)
{
	int ret;

	pr_debug("%s\n",__func__);
	ret = fa_zio_register();
	if (ret < 0)
		return ret;
	ret = fa_spec_init();
	if (ret < 0) {
		fa_zio_unregister();
		return ret;
	}
	return 0;
}

static void fa_exit(void)
{
	fa_spec_exit();
	fa_zio_unregister();
}

module_init(fa_init);
module_exit(fa_exit);

MODULE_AUTHOR("Federico Vaga");
MODULE_DESCRIPTION("FMC-ADC Linux Driver");
MODULE_LICENSE("GPL");
