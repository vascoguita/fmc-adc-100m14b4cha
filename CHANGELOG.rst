..
   SPDX-FileCopyrightText: 2020 CERN (home.cern)
   SPDX-License-Identifier: CC0-1.0

=========
Changelog
=========

6.0.1 - 2022-01-11
==================
Changed
-------
- hdl: ip_cores have been updated

6.0.0 - 2021-09-10
==================
Added
-----
- hdl: configurable auto byte swap in hardware, useful for SVEC to reduce software complexity
- hdl,sw: DMA data is always little-endian
- sw: software version validation against FPGA version
- bld: flawfinder check on software tools

Changed
-------
- sw: offsets are not anymore in uV but they are just raw values

Fixed
-----
- sw: security fixes detected by flawfinder
- sw: fixes detected by checkpatch.pl
- sw: style fixes detected by checkpatch.pl
- sw: improve compatibility with newer ( > 3.10) Linux kernel versions

5.0.4 - 2021-07-09
==================
Fixed
-----
- sw: fix endianess only when required
- sw: ADC chip reset is BIT(7)

5.0.3 - 2021-07-05
==================
Added:
-----
- bld: CI support

Changed
-------
- sw: use YAML for fau-calibration

Fixed
-----
- doc: add offset units
- sw: calibration value at boot time incorrect

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
Changed
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
