# If it exists includes Makefile.specific. In this Makefile, you should put
# specific Makefile code that you want to run before this. For example,
# build a particular environment.
-include Makefile.specific

# include parent_common.mk for buildsystem's defines
REPO_PARENT=../..
-include $(REPO_PARENT)/parent_common.mk

DESTDIR ?= /usr/local

LIBADC = ../lib/
ZIO ?= ../zio
ZIO_ABS ?= $(abspath $(ZIO) )

GIT_VERSION := $(shell git describe --dirty --long --tags)
ZIO_GIT_VERSION := $(shell cd $(ZIO_ABS); git describe --dirty --long --tags)

CFLAGS = -Wall -g -ggdb -I$(LIBADC) -I$(ZIO_ABS)/include -I../kernel $(EXTRACFLAGS)
CFLAGS += -DGIT_VERSION="\"$(GIT_VERSION)\""
CFLAGS += -DZIO_GIT_VERSION="\"$(ZIO_GIT_VERSION)\""

LDFLAGS = -L$(LIBADC)
LDLIBS = -lfmcadc -lpthread -lrt

DEMOS := fald-simple-acq fald-acq fald-trg-cfg 
DEMOS += fald-simple-get-conf
DEMOS += fald-test
DEMOS += fald-bad-clock


all: demo

demo: $(DEMOS)

install:
	install -d $(DESTDIR)/bin
	install -D $(DEMOS) $(DESTDIR)/bin

%: %.c $(LIBADC)/libfmcadc.a
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS) $(LDLIBS)

# make nothing for modules_install, but avoid errors
modules_install:

clean:
	rm -f $(DEMOS) *.o *~

.PHONY: all, clean
