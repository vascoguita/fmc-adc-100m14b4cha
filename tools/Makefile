# user-space tools for spec-fine-delay
M = $(shell /bin/pwd)/../kernel
DESTDIR ?= /usr/local

HOST_EXTRACFLAGS += -I$(M) -I$(ZIO)/include -Wno-trigraphs -Wall -ggdb

HOSTCC ?= gcc

hostprogs-y := fau-trg-config
hostprogs-y += fau-acq-time
hostprogs-y += parport-burst

# we are not in the kernel, so we need to piggy-back on "make modules"
all modules: $(hostprogs-y)

clean:
	rm -f $(hostprogs-y) *.o *~

# make nothing for modules_install, but avoid errors
modules_install:

install:
	install -d $(DESTDIR)/bin
	install -D $(hostprogs-y) $(DESTDIR)/bin

# we need this as we are out of the kernel
%: %.c
	$(HOSTCC) $(HOST_EXTRACFLAGS) -O2 -Wall $^ -o $@
