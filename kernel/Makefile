
LINUX ?= /lib/modules/$(shell uname -r)/build
ZIO ?= $(M)/../zio
FMC_BUS ?= $(M)/../fmc-bus

KBUILD_EXTRA_SYMBOLS := \
	$(ZIO)/Module.symvers \
	$(FMC_BUS)/kernel/Module.symvers

GIT_VERSION = $(shell cd $(src); git describe --always --dirty --long --tags)

ccflags-y = -DGIT_VERSION=\"$(GIT_VERSION)\" \
	-I$(ZIO)/include \
	-I$(FMC_BUS)/kernel/include \
	-I$M

ccflags-$(CONFIG_FMC_ADC_DEBUG) += -DDEBUG

subdirs-ccflags-y = $(ccflags-y)

obj-m := fmc-adc-100m14b.o
fmc-adc-100m14b-objs =  fa-core.o
fmc-adc-100m14b-objs += fa-zio-drv.o
fmc-adc-100m14b-objs += fa-calibration.o
fmc-adc-100m14b-objs += fa-regtable.o
fmc-adc-100m14b-objs += fa-zio-trg.o
fmc-adc-100m14b-objs += fa-spec-dma.o
fmc-adc-100m14b-objs += fa-irq.o
fmc-adc-100m14b-objs += onewire.o
fmc-adc-100m14b-objs += spi.o

all modules:
	$(MAKE) -C $(LINUX) M=$(shell /bin/pwd) modules

install modules_install:
	$(MAKE) -C $(LINUX) M=$(shell /bin/pwd) modules_install

# be able to run the "clean" rule even if $(LINUX) is not valid
clean:
	rm -rf *.o *~  .*.cmd *.ko *.mod.c .tmp_versions Module.symvers \
		Module.markers modules.order
