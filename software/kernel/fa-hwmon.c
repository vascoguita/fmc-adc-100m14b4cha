// SPDX-License-Identifier: GPL-2.0-or-later
// SPDX-FileCopyrightText: 2023 CERN (home.cern)

/*
 * Author: Vaibhav Gupta <vaibhav.gupta@cern.ch>
 *
 * Hardware Monitoring for fa-dev
 */

#include <linux/hwmon.h>
#include "fmc-adc-100m14b4cha-private.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,10,0)

static umode_t fa_hwmon_temp_is_visible(const void *_data,
				  enum hwmon_sensor_types type, u32 attr,
				  int channel)
{
	return 0444;
}

static int fa_hwmon_temp_read(struct device *dev, enum hwmon_sensor_types type,
				u32 attr, int channel, long *val)
{
	int32_t value;
	struct fa_dev *fa = dev_get_drvdata(dev);

	switch(attr) {
		case hwmon_temp_min:
			*val = 30*1000;
			return 0;

		case hwmon_temp_max:
			*val = 60*1000;
			return 0;

		case hwmon_temp_crit:
			*val = 65*1000;
			return 0;
	}

	value = fa_temperature_read(fa);

	*val = (long)value;

	return 0;
}

static int fa_hwmon_temp_sensor_id_read(struct device *dev,
				   enum hwmon_sensor_types type,
				   u32 attr, int channel, const char **str)
{
	struct fa_dev *fa = dev_get_drvdata(dev);

	*str = fa->hwmon_temp_sensor_id;

	return 0;
}

static const struct hwmon_channel_info *fa_hwmon_info[] = {
	HWMON_CHANNEL_INFO(temp, HWMON_T_INPUT | HWMON_T_LABEL | HWMON_T_MIN |
			   HWMON_T_MAX | HWMON_T_CRIT),
	NULL
};

static const struct hwmon_ops fa_hwmon_temp_ops = {
	.is_visible = fa_hwmon_temp_is_visible,
	.read = fa_hwmon_temp_read,
	.read_string = fa_hwmon_temp_sensor_id_read
};

static const struct hwmon_chip_info fa_hwmon_temp_chip_info = {
	.ops = &fa_hwmon_temp_ops,
	.info = fa_hwmon_info,
};

int fa_hwmon_init(struct fa_dev *fa)
{
	struct device *dev = &fa->pdev->dev;

	fa->hwmon_dev = devm_hwmon_device_register_with_info(dev,
							     "FMC_ADC_100M14B4C",
							     fa,
							     &fa_hwmon_temp_chip_info,
							     NULL);
	if(!IS_ERR(fa->hwmon_dev)) {
		fa->hwmon_temp_sensor_id = devm_kasprintf(fa->hwmon_dev,
							  GFP_KERNEL,
							  "Temperature [%s]",
							  dev_name(&fa->slot->dev));
		if(!fa->hwmon_temp_sensor_id) {
			devm_hwmon_device_unregister(dev);
			return -ENOMEM;
		}

		return 0;
	}

	return PTR_ERR(fa->hwmon_dev);
}

#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(5,10,0) */
