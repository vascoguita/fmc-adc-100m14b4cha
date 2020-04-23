-include Makefile.specific
# include parent_common.mk for buildsystem's defines
# use absolute path for REPO_PARENT
CURDIR:=$(shell /bin/pwd)
REPO_PARENT ?= $(CURDIR)/../..
-include $(REPO_PARENT)/parent_common.mk

DKMS ?= 0

KVERSION ?= $(shell uname -r)
LINUX ?= /lib/modules/$(KVERSION)/build
ZIO ?= $(REPO_PARENT)/fmc/zio
FMC ?= $(REPO_PARENT)/fmc-sw
VMEBUS ?= $(REPO_PARENT)/vmebridge-ng

ifeq ($(DKMS), 1)
# Take last installed version (if installed using RPM it should be OK)
ZIO_VERSION ?= $(shell basename $(shell ls -d $(DKMSTREE)/zio/* | grep -E "\/[0-9]+\.[0-9]+\.[0-9]+" | sort -V | tail -n 1))
ZIO_ABS ?= $(DKMSTREE)/zio/$(ZIO_VERSION)/source
ZIO_EXTRA_SYMBOLS-y = $(DKMSTREE)/zio/kernel-$(KVERSION)-$(shell uname -p)/module/Module.symvers
else
ZIO_ABS ?= $(abspath $(ZIO))
ZIO_EXTRA_SYMBOLS-y = $(ZIO_ABS)/drivers/zio/Module.symvers
ZIO_VERSION ?= $(shell cd $(ZIO_ABS); git describe --always --dirty --long --tags)
FMC_ABS ?= $(abspath $(FMC))
FMC_EXTRA_SYMBOLS-y = $(FMC_ABS)/drivers/fmc/Module.symvers
endif

VMEBUS_ABS ?= $(abspath $(VMEBUS) )

GIT_VERSION = $(shell git describe --always --dirty --long --tags)

all modules:
	$(MAKE) -C $(LINUX) M=$(CURDIR) ZIO_ABS=$(ZIO_ABS) FMC_ABS=$(FMC_ABS) \
		ZIO_EXTRA_SYMBOLS-y=$(ZIO_EXTRA_SYMBOLS-y) \
		FMC_EXTRA_SYMBOLS-y=$(FMC_EXTRA_SYMBOLS-y) \
		ZIO_VERSION=$(ZIO_VERSION) \
		GIT_VERSION=$(GIT_VERSION) \
		VMEBUS_ABS=$(VMEBUS_ABS) modules

install modules_install: modules
	$(MAKE) -C $(LINUX) M=$(CURDIR) modules_install

# be able to run the "clean" rule even if $(LINUX) is not valid
clean:
	rm -rf *.o *~  .*.cmd *.ko *.mod.c .tmp_versions Module.symvers \
		Module.markers modules.order
