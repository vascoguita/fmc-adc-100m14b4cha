
LINUX ?= /lib/modules/$(shell uname -r)/build
ZIO ?= $(HOME)/zio
SPEC_SW ?= $(HOME)/spec-sw
FMC_BUS ?= $(HOME)/fmc-bus

KBUILD_EXTRA_SYMBOLS := $(ZIO)/Module.symvers $(SPEC_SW)/kernel/Module.symvers

ccflags-y = -I$(ZIO)/include -I$(FMC_BUS)/kernel/include -I$(SPEC_SW)/kernel -I$M

ccflags-y += -DDEBUG # temporary

subdirs-ccflags-y = $(ccflags-y)


obj-m := fmc-adc.o

fmc-adc-objs =  fa-zio-drv.o 
fmc-adc-objs += fa-core.o
fmc-adc-objs += fa-zio-trg.o
fmc-adc-objs += fa-dma.o
fmc-adc-objs += onewire.o
fmc-adc-objs += spi.o


all: modules

modules_install clean modules:
	$(MAKE) -C $(LINUX) M=$(shell /bin/pwd) $@
