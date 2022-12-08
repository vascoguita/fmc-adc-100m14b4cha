.. Copyright (c) 2013-2020 CERN (home.cern)
   SPDX-License-Identifier: CC-BY-SA-4.0

.. _introduction:

------------
Introduction
------------

This document describes the gateware developed to support the
FmcAdc100m14b4cha (later refered to as fmc-adc) mezzanine card on the
`SPEC`_ and `SVEC`_ carrier cards. The gateware is the HDL code used
to generate the bitstream that configures the FPGA on the carrier
(sometimes also called firmware).  The gateware architecture is
described in detail.  The configuration and operation of the fmc-adc
is also explained. The Linux driver and basic tools are explained as
well.  On the other hand, this manual is not intended to provide
information about the hardware design.

Repositories and Releases
=========================

The `FMC ADC 100M 14 bits 4 Channels`_ is hosted on
the `Open HardWare Repository`_. The main development happens
here. You can clone the GIT project with the following command::

  git clone https://ohwr.org/project/fmc-adc-100m14b4cha.git

Within the GIT respository, releases are marked with a TAG named
using the `Semantic Versioning`_. For example the latest release is
|latest_release|. You can also find older releases with a different versioning
mechanism.

For each release we will publish the FPGA bitstream for all supported
carrier cards (`FPGA Bitstream Page
<https://ohwr.org/project/fmc-adc-100m14b4cha/wikis/Documents/Bitstreams>`_).
For the Linux driver we can't release the binary because it depends on
the Linux version on which it will run. For details about how to build
the Linux driver for your kernel please have a look at :ref:`Compile And Install <drv_build_install>`
section in :doc:`Driver's Documentation <software/driver>`.

Documentation License
=====================

This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International
License. To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/.

.. _SPEC: http://www.ohwr.org/projects/spec
.. _SVEC: http://www.ohwr.org/projects/svec
.. _`FMC ADC 100M 14 bits 4 Channels`: https://ohwr.org/project/fmc-adc-100m14b4cha
.. _`Open HardWare Repository`: https://ohwr.org/
.. _`Semantic Versioning`: https://semver.org/
