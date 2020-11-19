.. Copyright (c) 2013-2020 CERN (home.cern)
   SPDX-License-Identifier: CC-BY-SA-4.0

.. _introduction:

------------
Introduction
------------

This document describes the gateware developed to support the FmcAdc100m14b4cha (later refered to as
fmc-adc) mezzanine card on the `SPEC`_ and `SVEC`_ carrier cards. The gateware is the HDL code
used to generate the bitstream that configures the FPGA on the carrier (sometimes also called
firmware).  The gateware architecture is described in detail.  The configuration and operation of
the fmc-adc is also explained.  On the other hand, this manual is not intended to provide
information about the software used to control the fmc-adc board, nor details about it's hardware
design.

Repositories and Releases
=========================


.. _SPEC: http://www.ohwr.org/projects/spec
.. _SVEC: http://www.ohwr.org/projects/svec
