# Copyright (C) 2019 CERN
# SPDX-FileCopyrightText: 2020 CERN (home.cern)
#
# SPDX-License-Identifier: CC0-1.0

-include Makefile.specific
# include parent_common.mk for buildsystem's defines
# use absolute path for REPO_PARENT

-include $(REPO_PARENT)/parent_common.mk

CPPCHECK ?= cppcheck
DKMS ?= 0
CURDIR := $(shell /bin/pwd)
KVERSION ?= $(shell uname -r)
KERNELSRC ?= /lib/modules/$(KVERSION)/build

ifdef REPO_PARENT
ZIO ?= $(REPO_PARENT)/fmc/zio
FMC ?= $(REPO_PARENT)/fmc-sw
VMEBUS ?= $(REPO_PARENT)/vmebridge-ng
endif

ifeq ($(DKMS), 1)
# Take last installed version (if installed using RPM it should be OK)
ZIO_VERSION ?= $(shell basename $(shell ls -d $(DKMSTREE)/zio/* | grep -E "\/[0-9]+\.[0-9]+\.[0-9]+" | sort -V | tail -n 1))
ZIO_ABS ?= $(DKMSTREE)/zio/$(ZIO_VERSION)/source
ZIO_EXTRA_SYMBOLS-y = $(DKMSTREE)/zio/kernel-$(KVERSION)-$(shell uname -p)/module/Module.symvers

# Take last installed version (if installed using RPM it should be OK)
FMC_VERSION ?= $(shell basename $(shell ls -d $(DKMSTREE)/fmc-sw/* | grep -E "\/[0-9]+\.[0-9]+\.[0-9]+" | sort -V | tail -n 1))
FMC_ABS ?= $(DKMSTREE)/zio/$(FMC_VERSION)/source
FMC_EXTRA_SYMBOLS-y = $(DKMSTREE)/fmc-sw/kernel-$(KVERSION)-$(shell uname -p)/module/Module.symvers
else

ifndef ZIO
$(error "Missing ZIO environment variable")
endif
ifndef FMC
$(error "Missing FMC environment variable")
endif
ifndef VMEBUS
$(error "Missing VMEBUS environment variable")
endif

ZIO_ABS ?= $(abspath $(ZIO))
ZIO_EXTRA_SYMBOLS-y = $(ZIO_ABS)/drivers/zio/Module.symvers
ZIO_VERSION ?= $(shell cd $(ZIO_ABS); git describe --always --dirty --long --tags)
FMC_ABS ?= $(abspath $(FMC))
FMC_EXTRA_SYMBOLS-y = $(FMC_ABS)/drivers/fmc/Module.symvers
endif

VMEBUS_ABS ?= $(abspath $(VMEBUS) )

GIT_VERSION = $(shell git describe --always --dirty --long --tags)

all: modules

clean modules help modules_install coccicheck:
	$(MAKE) -C $(KERNELSRC) M=$(CURDIR) ZIO_ABS=$(ZIO_ABS) FMC_ABS=$(FMC_ABS) \
		ZIO_EXTRA_SYMBOLS-y=$(ZIO_EXTRA_SYMBOLS-y) \
		FMC_EXTRA_SYMBOLS-y=$(FMC_EXTRA_SYMBOLS-y) \
		ZIO_VERSION=$(ZIO_VERSION) \
		GIT_VERSION=$(GIT_VERSION) \
		VMEBUS_ABS=$(VMEBUS_ABS) $@

cppcheck:
	$(CPPCHECK) -q -I. -I$(ZIO_ABS)/include -I$(FMC_BUS_ABS)/  --suppress=missingIncludeSystem --enable=all *.c *.h

.PHONY: all install coccicheck cppcheck
