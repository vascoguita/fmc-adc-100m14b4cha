LIBADC = ../lib/
ZIO ?= ../zio

CFLAGS = -Wall -g -ggdb -I$(LIBADC) -I$(ZIO)/include
LDFLAGS = -L$(LIBADC) -lfmcadc -lpthread

DEMOS := fald-simple-acq fald-acq fald-trg-cfg 
DEMOS += fald-simple-get-conf
DEMOS += fald-test

all: demo

demo: $(DEMOS)

%: %.c $(LIBADC)/libfmcadc.a
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

clean:
	rm -f $(DEMOS) *.o *~

.PHONY: all, clean
