DESTDIR ?= /usr/local

LIBADC = ../lib/
ZIO ?= ../zio

CFLAGS = -Wall -g -ggdb -I$(LIBADC) -I$(ZIO)/include $(EXTRACFLAGS)
LDFLAGS = -L$(LIBADC) -lfmcadc -lpthread

DEMOS := fald-simple-acq fald-acq fald-trg-cfg 
DEMOS += fald-simple-get-conf
DEMOS += fald-test


all: demo

demo: $(DEMOS)

install:
	install -d $(DESTDIR)/bin
	install -D $(DEMOS) $(DESTDIR)/bin

%: %.c $(LIBADC)/libfmcadc.a
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

# make nothing for modules_install, but avoid errors
modules_install:

clean:
	rm -f $(DEMOS) *.o *~

.PHONY: all, clean
