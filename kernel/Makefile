# include parent_common.mk for buildsystem's defines
# use absolute path for REPO_PARENT
CURDIR:=$(shell /bin/pwd)
REPO_PARENT ?= $(CURDIR)/../..
-include $(REPO_PARENT)/parent_common.mk

LINUX ?= /lib/modules/$(shell uname -r)/build
FMC_BUS ?= ../fmc-bus
ZIO ?= ../zio
SVEC_SW ?= ../svec-sw
VMEBUS ?= $(REPO_PARENT)/../vmebridge

# FMC_BUS_ABS and ZIO_ABS has to be absolut path,
# due to beeing passed to the Kbuild
FMC_BUS_ABS ?= $(abspath $(FMC_BUS) )
ZIO_ABS ?= $(abspath $(ZIO) )
SVEC_SW_ABS ?= $(abspath $(SVEC_SW) )
VMEBUS_ABS ?= $(abspath $(VMEBUS) )

GIT_VERSION = $(shell git describe --always --dirty --long --tags)
export GIT_VERSION

FMC_BUS_VERSION ?= $(shell cd $(FMC_BUS_ABS); git describe --always --dirty --long --tags)
ZIO_VERSION     ?= $(shell cd $(ZIO_ABS);     git describe --always --dirty --long --tags)

export FMC_BUS_VERSION
export ZIO_VERSION

all modules:
	$(MAKE) -C $(LINUX) M=$(CURDIR) FMC_BUS_ABS=$(FMC_BUS_ABS) \
		ZIO_ABS=$(ZIO_ABS) SVEC_SW_ABS=$(SVEC_SW_ABS) \
		VMEBUS_ABS=$(VMEBUS_ABS) modules

install modules_install: modules
	$(MAKE) -C $(LINUX) M=$(CURDIR) modules_install

# be able to run the "clean" rule even if $(LINUX) is not valid
clean:
	rm -rf *.o *~  .*.cmd *.ko *.mod.c .tmp_versions Module.symvers \
		Module.markers modules.order
