
LINUX ?= /lib/modules/$(shell uname -r)/build
ZIO ?= $(HOME)/zio
SPEC_SW ?= $(HOME)/spec-sw

KBUILD_EXTRA_SYMBOLS := $(ZIO)/Module.symvers $(SPEC_SW)/kernel/Module.symvers

ccflags-y = -I$(ZIO)/include -I$(SPEC_SW)/kernel -I$(SPEC_SW)/kernel/include -I$M

ccflags-y += -DDEBUG # temporary

subdirs-ccflags-y = $(ccflags-y)


obj-m := spec-fmc-adc.o

spec-fmc-adc-objs =  fa-zio-drv.o 
spec-fmc-adc-objs += fa-core.o
spec-fmc-adc-objs += fa-zio-trg.o
spec-fmc-adc-objs += fa-dma.o
spec-fmc-adc-objs += onewire.o


all: modules

modules_install clean modules:
	$(MAKE) -C $(LINUX) M=$(shell /bin/pwd) $@
