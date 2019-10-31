-include Makefile.specific

# include parent_common.mk for buildsystem's defines
#use absolute path for REPO_PARENT
REPO_PARENT ?= $(shell /bin/pwd)/../..
-include $(REPO_PARENT)/parent_common.mk

LINUX ?= /lib/modules/$(shell uname -r)/build

DRV_VERSION := $(shell git describe --always --dirty --long --tags)

all: modules

.PHONY: all modules clean help install modules_install

modules help modules_install:
	$(MAKE) -C $(LINUX) M=$(shell pwd) DRV_VERSION=$(DRV_VERSION) $@

install:
	$(MAKE) -C $(LINUX) M=$(shell pwd) DRV_VERSION=$(DRV_VERSION) modules_install

# be able to run the "clean" rule even if $(LINUX) is not valid
clean:
	rm -rf *.o *~  .*.cmd *.ko *.mod.c .tmp_versions Module.symvers \
                         Module.markers modules.order
