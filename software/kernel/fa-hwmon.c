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
	struct fa_dev *fa = dev_get_drvdata(dev);

	*val = fa_temperature_read(fa);

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
	HWMON_CHANNEL_INFO(temp, HWMON_T_INPUT | HWMON_T_LABEL),
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
	int size;
	char device_type[] = "Temperature - FMC ADC 100M14B4C - ";
	struct device *dev = &fa->pdev->dev;

	fa->hwmon_dev = devm_hwmon_device_register_with_info(dev,
							     "fa_temperature",
							     fa,
							     &fa_hwmon_temp_chip_info,
							     NULL);
	if(!IS_ERR(fa->hwmon_dev)) {
		size = strlen(dev_name(&fa->slot->dev));
		size += strlen(device_type);
		size++;

		fa->hwmon_temp_sensor_id = devm_kzalloc(fa->hwmon_dev,
							size, GFP_KERNEL);
		if(!fa->hwmon_temp_sensor_id) {
			devm_hwmon_device_unregister(dev);
			return -ENOMEM;
		}

		snprintf(fa->hwmon_temp_sensor_id, size, "%s%s",
			 device_type, dev_name(&fa->slot->dev));

		return 0;
	}

	return PTR_ERR(fa->hwmon_dev);
}

#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(5,10,0) */
