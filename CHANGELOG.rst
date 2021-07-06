..
   SPDX-FileCopyrightText: 2020 CERN (home.cern)
   SPDX-License-Identifier: CC0-1.0

=========
Changelog
=========

5.0.4 - 2021-07-xx
==================

Fixed
-----
- sw: fix endianess only when required

5.0.3 - 2021-07-05
==================
Added:
-----
- bld: CI support

Fixed
-----
- doc: add offset units
- sw: calibration value at boot time incorrect
- sw: use YAML for fau-calibration

5.0.2 - 2021-07-03
==================
Fixed
-----
- sw: calibration value for ADC were not applied correctly

5.0.1 - 2021-06-11
==================
Fixed
-----
- sw: concurrent DMA transfers are possible thanks to a wait and retry algorithm
- sw: wait 400ms before reading the temperature the first time (the hardware
  takes time to setup the thermometers)
- doc: use cheby files from hdl instead of special implementations

5.0.0 - 2021-02-11
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
- MBLT support for SVEC

Removed
-------
- library is not supported anymore, use adc-lib (https://www.ohwr.org/projects/adc-lib)
- fald-acq tool is not supported anymore, use adc-acq from adc-lib (https://www.ohwr.org/projects/adc-lib)
