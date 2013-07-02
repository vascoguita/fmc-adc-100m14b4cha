# This is not a kbuild Makefile. It is a plain Makefile so it can be copied
ZIO ?= ../zio

LIB = libfmcadc.a
LOBJ := route.o
LOBJ += boards.o
LOBJ += config-zio.o
LOBJ += lib.o
LOBJ += fmcadc-lib-zio.o

CFLAGS = -Wall -ggdb -O2 -I../kernel -I$(ZIO)/include
LDFLAGS = -L. -lfmcadc

modules all: $(LIB)


%: %.c $(LIB)
	$(CC) $(CFLAGS) $*.c $(LDFLAGS) -o $@

$(LIB): $(LOBJ)
	ar r $@ $^

clean:
	rm -f $(LIB) .depend *.o *~

.depend: Makefile $(wildcard *.c *.h ../*.h)
	$(CC) $(CFLAGS) -M $(LOBJ:.o=.c) -o $@

install modules_install:

-include .depend

