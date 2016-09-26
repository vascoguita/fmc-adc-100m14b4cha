-include Makefile.specific
# include parent_common.mk for buildsystem's defines
# use absolute path for REPO_PARENT
CURDIR:=$(shell /bin/pwd)
REPO_PARENT ?= $(CURDIR)/../..
-include $(REPO_PARENT)/parent_common.mk

KVERSION ?= $(shell uname -r)
LINUX ?= /lib/modules/$(KVERSION)/build
ZIO ?= ../zio
VMEBUS ?= $(REPO_PARENT)/../vmebridge

# FMC_BUS_ABS and ZIO_ABS has to be absolut path,
# due to beeing passed to the Kbuild
ZIO_ABS ?= $(abspath $(ZIO) )
VMEBUS_ABS ?= $(abspath $(VMEBUS) )

GIT_VERSION = $(shell git describe --always --dirty --long --tags)
export GIT_VERSION

ZIO_VERSION     ?= $(shell cd $(ZIO_ABS);     git describe --always --dirty --long --tags)

export ZIO_VERSION

all modules:
	$(MAKE) -C $(LINUX) M=$(CURDIR) ZIO_ABS=$(ZIO_ABS) \
		VMEBUS_ABS=$(VMEBUS_ABS) modules

install modules_install: modules
	$(MAKE) -C $(LINUX) M=$(CURDIR) modules_install

# be able to run the "clean" rule even if $(LINUX) is not valid
clean:
	rm -rf *.o *~  .*.cmd *.ko *.mod.c .tmp_versions Module.symvers \
		Module.markers modules.order
