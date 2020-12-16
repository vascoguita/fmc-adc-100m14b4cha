..
   SPDX-FileCopyrightText: 2020 CERN (home.cern)
   SPDX-License-Identifier: CC0-1.0

=========
Changelog
=========

5.0.0 - 2020-01-01
==================
Changes
-------
- channel sysfs attribute 'chx-offset' does not accept mV (milli-volts) values
  anymore. Now the unit is uV (micro-Volts)
- acquisition sysfs attribute 'decimation' is now named 'undersample'
- software trigger is enable by default
- on DAC offset saturation set the maximum/minimum value instead of error
- the software trigger is not anymore a ZIO attribute. It is now in debugfs

Added
-----
- multiple trigger sources at the same time
- trigger threshold per-channel
- channel sysfs attributes to set trigger threshold
- sysfs binary attribute to overwrite run-time calibration data
- add tool to get/set run-time calibration data
- periodically update gain calibration for DAC and ADC
- trigger time

Removed
-------
- library is not supported anymore, use adc-lib (https://www.ohwr.org/projects/adc-lib)
- fald-acq tool is not supported anymore, use adc-acq from adc-lib (https://www.ohwr.org/projects/adc-lib)

