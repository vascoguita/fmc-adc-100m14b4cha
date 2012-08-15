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

static struct fmc_driver fmc_adc__drv;
static char *fa_binaries = FA_GATEWARE_DEFAULT_NAME;
module_param_named(file, fa_binaries, charp, 0444);

/* This structure lists the various subsystems */
struct fa_modlist {
	char *name;
	int (*init)(struct spec_fa *);
	void (*exit)(struct spec_fa *);
};
#define SUBSYS(x) { #x, fa_ ## x ## _init, fa_ ## x ## _exit }
static struct fa_modlist mods[] = {
	//SUBSYS(onewire),
	SUBSYS(zio),
};

/* probe and remove are called by fa-spec.c */
int fa_probe(struct fmc_device *fmc)
{
	struct fa_modlist *m = NULL;
	struct spec_fa *fa;
	struct spec_dev *spec = fmc->carrier_data;
	int err, i = 0;

	pr_info("%s:%d\n", __func__, __LINE__);
	/* Driver data */
	fa = devm_kzalloc(&fmc->dev, sizeof(struct spec_fa), GFP_KERNEL);
	if (!fa)
		return -ENOMEM;
	fmc_set_drvdata(fmc, fa);

	fa->fmc = fmc;
	fa->base = spec->remap[0];

	/* We first write a new binary (and lm32) within the spec */
	err = fmc->op->reprogram(fmc, &fmc_adc__drv, fa_binaries);
	if (err) {
		dev_err(fmc->hwdev, "write firmware \"%s\": error %i\n",
				fa_binaries, err);
		goto out;
	}

	/* init all subsystems */
	for (i = 0, m = mods; i < ARRAY_SIZE(mods); i++, m++) {
		dev_dbg(fmc->hwdev, "Calling init for \"%s\"\n", m->name);
		err = m->init(fa);
		if (err) {
			dev_err(fmc->hwdev, "error initializing %s\n", m->name);
			goto out;
		}

	}

	return 0;
out:
	while (--m, --i >= 0)
		if (m->exit)
			m->exit(fa);
	devm_kfree(&fmc->dev, fa);
	return err;
}
int fa_remove(struct fmc_device *fmc)
{
	struct spec_fa *fa = fmc_get_drvdata(fmc);

	fa_zio_exit(fa);
	devm_kfree(&fmc->dev, fa);
	return 0;
}
static struct fmc_driver fmc_adc__drv = {
	.driver.name = KBUILD_MODNAME,
	.probe = fa_probe,
	.remove = fa_remove,
	/* no table, as the current match just matches everything */
};

static int fa_init(void)
{
	int ret;

	pr_debug("%s\n",__func__);
	ret = fmc_driver_register(&fmc_adc__drv);
	if (ret)
		return ret;

	ret = fa_zio_register();
	if (ret) {
		fmc_driver_unregister(&fmc_adc__drv);
		return ret;
	}
	return 0;
}

static void fa_exit(void)
{
	fa_zio_unregister();
	fmc_driver_unregister(&fmc_adc__drv);
}

module_init(fa_init);
module_exit(fa_exit);

MODULE_AUTHOR("Federico Vaga");
MODULE_DESCRIPTION("FMC-ADC Linux Driver");
MODULE_LICENSE("GPL");
