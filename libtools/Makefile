LIBADC = ../lib/
ZIO ?= ../zio

CFLAGS = -Wall -ggdb -O2 -I$(LIBADC) -I$(ZIO)/include
LDFLAGS = -static -L$(LIBADC) -lfmcadc

DEMOS := fald-simple-acq
DEMOS += fald-simple-get-conf

all: demo

demo: $(DEMOS)

%: %.c $(LIBADC)/libfmcadc.a
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

clean:
	rm -f $(DEMOS) *.o *~

.PHONY: all, clean
