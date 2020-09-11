..
  SPDX-License-Identifier: CC-BY-SA-4.0
  SPDX-FileCopyrightText: 2020 CERN

======
Driver
======

Driver Features
===============

Requirements
============

The fmcadc100m14b4ch device driver has been developed and tested on Linux
3.10. Other Linux versions might work as well but it is not guaranteed.

This driver depends on the `zio`_ framework and `fmc`_ library; we
developed and tested against version `zio`_ 1.4 and `fmc`_ 1.0.

The FPGA address space must be visible on the host system. This requires
a driver for the FPGA carrier that exports the FPGA address space to the
host. As of today we support `SPEC`_ and `SVEC`_.


Compile And Install
====================

The compile and install the fmcadc100m14b4ch device driver you need
first to export the path to its direct dependencies, and then you
execute ``make``. This driver depends on the `zio`_ framework and `fmc`_ library; on a VME system it depends also on the VME bridge driver from CERN BE-CO.

::

      $ cd /path/to/fmc-adc-100m14b4cha-sw/kernel
      $ export LINUX=/path/to/linux/sources
      $ export ZIO=/path/to/zio
      $ export FMC=/path/to/fmc-sw
      $ export VMEBUS=/path/to/vmebridge
      $ make
      $ make install

.. note::
   Since version v5.0.0 the fmcadc100m14b4ch device driver does not
   depend anymore on `fmc-bus`_ subsystem, instead it uses a new
   `fmc`_ library

The building process generates 3 Linux modules:
*kernel/fmc-adc-100m14b4ch.ko*, *kernel/fmc-adc-100m14b4ch-spec.ko*, and
*kernel/fmc-adc-100m14b4ch-svec.ko*.

Top Level Driver
================

The fmcadc100m14b4ch is a generic driver for an FPGA device that could
be instanciated on a number of FMC carriers. For each carrier we write
a little Linux module which acts as a top level driver (like the MFD
drivers in the Linux kernel). In these modules there is the knowledge
about the virtual memory range, the IRQ lines, and the DMA engine to
be used.

The top level driver is a platform driver that matches a string
containing the application identifier. The carrier driver builds this
identification string from the device ID embedded into the FPGA
(https://ohwr.org/project/fpga-dev-id).

Module Parameters
=================

The driver accepts a few load-time parameters for configuration. You can
pass them to insmod directly, or write them in ``/etc/modules.conf`` or
the proper file in ``/etc/modutils/``.

The following parameters are used:

enable_test_data=[0, 1]
     This is for testing purpose. When set to 1, this option enables the
     testing data, so the ADC doesn't store samples, but fills memory with
     sequential numbers. The 64 bit data vector is filled with sequential
     values from a free-running 25 bit counter: channel 0 sweeps the full
     range, channel 1 goes from 0 to 511, other channel always report 0.
     Trigger detection is unaffected by use of test data.

fa_calib_period_s=NUMBER
     The ADC periodically adjusts the calibration data based on the
     current temperature. You can use this module parameter to pass
     your period in seconds. By default the value is 0 (meaning
     feature disabled).

.. warning::
   The periodical calibration is by default disabled because we were
   not able to perform a complete validation. Use it at your own risk.

.. _zio: https://www.ohwr.org/project/zio
.. _fmc: https://www.ohwr.org/project/fmc-sw
.. _`fmc-bus`: http://www.ohwr.org/projects/fmc-bus
.. _`SVEC`: https://www.ohwr.org/projects/svec
.. _`SPEC`: https://www.ohwr.org/projects/spec

Device Abstraction
==================

This driver is based on the ZIO framework and the fmc-bus. It supports
initial setup of the board; it allows users to manually configure the
board, to start and stop acquisitions, to force trigger, to read
acquisition time-stamps and to read acquired samples.

The driver is designed as a ZIO driver. ZIO is a framework for
input/output hosted on http://www.ohwr.org/projects/zio.

ZIO devices are organized as csets (channel sets), and each of them
includes channels. This device offers one cset and four channels.
However, the device can only stores interleaved data for all four
channels.

The current approach to this is defining 5 channels: channels 0 to 3 are
the actual input connectors, and their software counterpart is used to
configure the channels; the last channel is called *i*, and is the
interleave channel where data is retrieved.

The Overall Device
''''''''''''''''''

As said, the device has 1 cset with 4+1 channels. Channels from 0 to 3
represent the physical channels 1 to 4. The 5th channel *chani* represent
a virtual channel created automatically by the ZIO framework; this
channel represent the interleave acquisition on the cset.

.. graphviz::
  :align: center

    graph layers {
     node [shape=box];
     adc [label="FMC ADC 100M4B4CHA"];

     adc -- cset0;
     cset0 -- chan0;
     cset0 -- chan1;
     cset0 -- chan2;
     cset0 -- chan3;
     cset0 -- chani;
    }

The ADC registers can be accessed in the proper sysfs directory:::

  cd /sys/bus/zio/devices/adc-100m14b-${ID}.

The overall device (*adc-100m14b*) provides the following attributes:

calibration_data
  It is a binary attribute which allows the user to change the runt-time
  calibration data (the EEPROM will not be touched). The ``fau-calibration``
  tool can be used to read write calibration data.
  To be consistent, this binary interface expects **only** little endian
  values because this is the endianess used to store calibration data for
  this device.

temperature
  It shows the current temperature

The Channel Set
'''''''''''''''

The ADC has 1 Channel Set named ``cset0``. Its attributes are used to
control the ADC state machine, the channel parameters and so on.

Some attributes are channel-specific, and one may thing they should live
at channel-level. Unfortunately, ZIO currently lacks the mechanisms to
convey channel attributes in the meta-data associated with an
interleaved acquisition (where several channels coexist), and for this
reason we chose to put them all at cset level. This may change in future
releases, but the library implementation will follow, so there will be
no effect on API users.

The description of attributes that follows is mainly useful for the
shell user, to diagnose the system and hack around with parameters.

Channel-specific Cset Attributes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The cset includes three attributes for each channel, as follows:

chN-50ohm-term
     The read-write attribute accepts values 0 or 1.  By writing 1, you
     turn on the termination resistor. Default is 0.

chN-offset
     The user offset is an integer value in the range [-5000,5000], and
     it represents millivolts.  The offset represents the center-scale
     of conversion for the input channel.  Internally, a DAC is used to
     generate the requested voltage, which is then subtracted from the
     input signal.  DAC values are corrected according to the
     calibration values retrieved from the FMC EEPROM. For this reason,
     the offset may saturate at values less than +/- 5V.

chN-offset-zero
     The necessary offset to to bring the signal to 0.

chN-vref
     The "voltage reference" used for conversion. This attribute may be
     renamed to "range" in the future (again, with no effect on API
     users).  Changing the range does not reset the user offset, which
     is automatically adjusted according to the new calibration values.
     The attribute accepts three values: 35 represents the 100mV range
     (-50mV to +50mV); 17 represents 1V range; 69 represents 10V range
     (-5V to +5V); 0 detaches the input connector from the ADC. The
     numbers used here derive from hardware values, and the attributes
     refuses any other value.

cnN-saturation
     The user saturation level in the range [0, 32767].  Users can use
     this value to configure their own saturation level.  The hardware
     applies this value symmetrically on the negative side.  By default
     is set at the maximum value.

Generic Cset Attributes
~~~~~~~~~~~~~~~~~~~~~~~

This section lists the attributes that are defined by this driver;
ZIO-wide attributes (current_buffer, enable and so on) are not
described.

fsm-auto-start
     This attribute can be set to 1 or 0.  It is 0 by default.  If set
     to 1, the acquisition state-machine is automatically restarted
     after the previous run is complete.  Thus, for example, a card
     configured for external trigger, after the first acquisition will
     continue acquiring and storing blocks to the ZIO buffer every time a
     new trigger event is detected.  Applications can read such blocks
     from the char device.

fsm-command
     Write-only: start (1) or stop (2) the state machine.  The values
     used reflects the hardware registers.  Stopping the state machine
     aborts any ongoing acquisition.  Starting the state machine is
     required in order to run an acquisition (the library manages this
     internally).  The green LED ACQ on the front panel reflect the fact
     that the state machine has started.  Restarting a running state
     machine is equivalent to first stopping it.

fsm-state
     Read-only current state of the FSM. Useful for diagnostics in
     strange situation.  Please refer to the firmware manual (or to
     source code) about the various states.

resolution-bits
     This read-only attribute returns 14, the number of valid bits in
     the ADC data stream.

rst-ch-offset
     This write-only attributes zeroes all offset DACs when written,
     independently of the value being written.  The driver applies the
     current calibration values, instead of writing 0 directly to the
     hardware.

undersample
     The ADC always acquires at 100MSamples/s and this value cannot be
     changed (it actually can, but it is not currently supported nor
     even tested).  If you need less samples you can tell the card to
     decimate (or under-sample) the data stream.  The attribute accepts
     an integer value, 1 to 65536; it means to pick one sample every
     that many.  Thus, but writing 100 you get a 1Ms data stream, and by
     writing 2 you get a 50Ms data stream.

sample-frequency
     This read-only attributes returns the measured sampling frequency

sample-counter
     Number of samples acquired on each channel during the last
     acquisition.  If queried while the acquisition is running you will
     get the number of samples acquired till that moment.  It can be
     used to evaluate the progress of a slow acquisition.

max-sample-mshot
     Maximum number of samples that can be stored in the FPGA memory in
     multi-shot mode

output-randomizer
     It allows to enable or disable the *Data Output Randomizer* in the ADC
     chip.

Timestamp Attributes
~~~~~~~~~~~~~~~~~~~~

The ADC mark with a timestamp all these events: state machine start,
state machine stop and acquisition end. The device split each timestamp
in 3 attributes named: second (s), ticks (t) and bins (b).

Seconds represents (by default) the number of second since the epoch;
ticks is the number of clocks at 125Mhz, the value is between 0 and
125000000 and it increments seconds when it overflow. At the moment, the
bins register is unused.

For example, to read the entire timestamp of the state machine start
event you should do::

     cat /sys/bus/zio/devices/adc-100m14b-0200/cset0/tstamp-acq-str-s
     cat /sys/bus/zio/devices/adc-100m14b-0200/cset0/tstamp-acq-str-t
     cat /sys/bus/zio/devices/adc-100m14b-0200/cset0/tstamp-acq-str-b

The channel set exports 3 time stamps:

tstamp-acq-str-{s|t|b}
     this is the time stamp of the last acquisition start command
     execution

tstamp-acq-end-{s|t|b}
     it is the time of last sample acquired

tstamp-acq-stop-{s|t|b}
     this is the time stamp of the last acquisition stop command
     execution

While the trigger instance export a time stamp:

tstamp-trg-lst-{s|t|b}
     this is the time stamp of the last trigger fire.  Please bear in
     mind that in multi-shot acquisition you have several trigger fire,
     so this time stamp refers only to the last one.  If you need the
     time stamp for each trigger fire you have to get it from the
     zio_control of the associated acquisition block.

By default these time stamps represent (more or less) the time since the
epoch. The user can change this and configure a different timing base.
The following attributes show the current base time:

tstamp-base-{su|sl|b}
     The current time known by the FPGA bitstream. It could be an
     internal clock or white-rabbit.

The Channels
''''''''''''

The ADC has 4 input channels. Each channel features one attribute, other
attributes in the directory are defined by the kernel or by ZIO.

current-value
     the current value is a 16 bit number, resulting from the 14 bit ADC
     value and calibration correction. The value is reported as unsigned,
     even if it actually represents a signed 16-bit integer. (This because
     ZIO manages 32-bit attributes and the value shown comes directly from
     the hardware)

The Trigger
'''''''''''

In ZIO, the trigger is a separate software module, that can be replaced
at run time. This driver includes its own ZIO trigger type, that is
selected by default when the driver is initialized. You can change
trigger type (for example use the timer ZIO trigger) but this is not the
typical use case for this board.

The name of the ADC trigger is adc-100m14b. Like all other ZIO objects,
each instance of the trigger has a sysfs directory with its own
attributes:

The ADC has its own zio_trigger_type and it can not work with any other
ZIO's trigger. The ADC trigger is called fmc-adc-trg. We advise you
against replacing the trigger with another one.

The trigger supports four operating modes: the external trigger is
driven by a specific LEMO connector on the front panel of the card. The
internal trigger activates on data threshold in one of the four input
channels - either positive-going or negative-going. The timer trigger
that fires a trigger a given time. The software trigger is activated by
simply writing to a register.

This is the list of attributes (excluding kernel-generic and ZIO-generic
ones):

source-triggered
     It is a bitmask where only one bit is set and it identifies the trigger
     type that triggered the last acquisition. Look at the header file, or
     the gateware document, for the meaning of each bit.


source
     It is a bitmask that enable (1) or disable (0) the available triggers.
     It supports multi-triggers, so you can enable more than one trigger at
     the same time. Look at the header file, or the gateware document, for
     the meaning of each bit.

polarity
     It is a bitmask that set the trigger polarity to positive (0) on
     negative (1) for each trigger that supports it. Look at the header file,
     or the gateware document, for the meaning of each bit.

chN-threshold
     These attributes choose the value of the data threshold (as a signed
     16-bit value).

chN-hysteresis
     These attributes choose the value of hysteresis associated to the
     threshold.

chN-delay, ext-delay
     The delay attribute tells how many samples to delay actual
     acquisition since the trigger fired.  Being sample-based, the
     resolution is 10ns. By default delay is 0. The undersampling
     does not have effect.

enable
     This is a standard zio attribute, and the code uses it to enable or
     disable the hardware trigger (i.e.  internal and external).  By
     default the trigger is enabled.

int-channel, int-threshold
     If the internal trigger is selected, these attributes choose the
     channel being monitored (range is 0..3) and the value of the data
     threshold (as a signed 16-bit value).

nshots
     Number of trigger shots.  The state machine acquires all trigger
     events to internal on-board memory, and performs DMA only at the
     end.  In single-shot, the acquisition can be as long ad 32Msamples
     (on-board memory is 256MB), but in multi-shot acquisition is first
     done to in-FPGA memory, and thus each shot can only acquire 2048
     samples.

post-samples, pre-samples
     Number of samples to acquire.  The pre-samples are acquired before
     the actual trigger event (plus its optional delay).  The post
     samples start from the trigger-sample itself.  The total number of
     samples acquired corresponds to the sum of the two numbers.  For
     multi-shot acquisition, each shot acquires that many sample, but
     pre + post must be at most 2048.

trg-time-{su|sl|t}
     When the trigger time is enabled, then these attributes can be used
     to program the trigger time.

The Buffer
''''''''''

In ZIO, buffers are separate objects. The framework offers two buffer
types: kmalloc and vmalloc. The former uses the kmalloc function to
allocate each block, the latter uses vmalloc to allocate the whole data
area. While the kmalloc buffer is linked with the core ZIO kernel
module, vmalloc is a separate module. The driver currently prefers
kmalloc, but even when it preferred vmalloc (up to mid June 2013), if
the respective module wad not loaded, ZIO would instantiate kmalloc.

You can change the buffer type, while not acquiring, by writing its name
to the proper attribute. For example::

     echo vmalloc > /sys/bus/zio/devices/adc-100m14b-0200/cset0/current_buffer

The disadvantage of kmalloc is that each block is limited in size.
usually 128kB (but current kernels allows up to 4MB blocks). The bigger
the block the more likely allocation fails. If you make a multi-shot
acquisition you need to ensure the buffer can fit enough blocks, and the
buffer size is defined for each buffer instance, i.e. for each channel.
In this case we acquire only from the interleaved channel, so before
making a 1000-long multishot acquisition you can do::

     export DEV=/sys/bus/zio/devices/adc-100m14b-0200
     echo 1000 > $DEV/cset0/chani/buffer/max-buffer-len

The vmalloc buffer allows mmap support, so when using vmalloc you can
save a copy of your data (actually, you save it automatically if you use
the library calls to allocate and fill the user-space buffer). However,
a vmalloc buffer allocates the whole data space at the beginning, which
may be unsuitable if you have several cards and acquire from one of them
at a time.

The vmalloc buffer type starts off with a size of 128kB, but you can
change it (while not acquiring), by writing to the associated attribute
of the interleaved channel. For example this sets it to 10MB::

     export DEV=/sys/bus/zio/devices/adc-100m14b-0200
     echo 10000 > $DEV/cset0/chani/buffer/max-buffer-kb

The debugfs Interface
=====================

The fmcadc100m14b4cha driver exports a set of debugfs attributes which
are supposed to be used only for debugging activities. For each device
instance you will see a directory in ``/sys/kernel/debug/adc-100m14b-*``.

data_pattern
   It set/unset the data pattern in the ADC chip. It uses the
   following syntax: "adc <enable> <pattern>", where *<enable>* could
   be 0 (disable), or (enable); and *<pattern>* could be any 14bit
   value. If you are disabling the feature, then the pattern is not
   necessary

trigger_software
   Write to this file to instantaneously trigger an acquisition.

spi-regs
   It dumps the ADC's registers.

regs
   It dumps the FPGA registers


Reading Data with Char Devices
==============================

To read data from user-space, applications should use the ZIO char
device interface. ZIO creates 2 char devices for each channel (as
documented in ZIO documentation). The ADC acquires only interleaved
samples, so ZIO creates two char device, as shown below::

     $ ls -l /dev/zio/
     total 0
     crw------- 1 root root 250, 8 Aug 23 22:21 adc-100m14b-0200-0-i-ctrl
     crw------- 1 root root 250, 9 Aug 23 22:21 adc-100m14b-0200-0-i-data

The actual path names depend on the version of udev you are running. The
fmc-adc library tries both names (the new one shown above, and the older
one, without a ``zio`` sub-directory). Also, please note that a still-newer
version of udev obeys device permissions, so you'll have read-only and
write-only device files (in this case they are both read-only).

If more than one board is probed for, you'll have two or more similar
pairs of devices, differing in the dev_id field, i.e. the ``0200`` shown
above. The dev_id field is built using the PCI bus and the devfn octet;
the example above refers to slot 0 of bus 2. (Most of the time each
PCI-E physical slot is mapped as a bus, so the slot number is usually
zero).

The ADC hardware does not allow to read data from a specific channel;
data is only transferred as an interleaved block of samples. Neither the
ZIO core nor the driver split interleaved data into 4 different buffers,
because that task is computationally intensive, and is better left to
the application (which may or may not need to do it). Thus, the driver
returns to user-space a block of interleaved samples.

To read this interleaved block you can read directly the interleaved
data char device adc-100m14b-0200-0-i-data using any program, for
example cat or hexdump::

     $ hexdump -n 8 -e '"" 1/2 "%x\n"' /dev/zio/adc-100m14b-0200-0-i-data
     fffc
     e474
     8034
     8084

The ADC hardware always interleaves all 4 channels, and you cannot
acquire a subset of the channels. The acquired stream, thus, follows
this format:

.. figure:: ../img/interleaved.pdf
   :alt: ADC interleaved data

The char-device model of ZIO is documented in the ZIO manual; basically,
the ctrl device returns metadata and the data device returns data. Items
in there are strictly ordered, so you can read metadata and then the
associated data, or read only data blocks and discard the associated
metadata.

The ``zio-dump`` tool, part of the ZIO distribution, turns metadata and data
into a meaningful grep-friendly text stream.

User Header Files
=================

Internally the driver uses the header file ``fmc-adc-100m14b4cha.h`` for the
declaration of all the functions, constants and structures. Some of these are
also available for the user-space programs; especially the constants to be
used to properly interpret the ``zio_control`` attributes, or the bitmask
fields definitions.

Troubleshooting
'''''''''''''''

This chapter lists a few errors that may happen and how to deal with
them.

Installation issue with modules_install
'''''''''''''''''''''''''''''''''''''''

The command ``sudo make modules_install`` may place the modules in the wrong
directory or fail with an error like::

        make: *** /lib/modules/<kernel-version>/build: No such file or directory.

This happens when you compiled by setting ``LINUX=`` and your sudo is not
propagating the environment to its child processes. In this case, you
should run this command instead::

        sudo make modules_install  LINUX=$LINUX
