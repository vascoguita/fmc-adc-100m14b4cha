# This is not a kbuild Makefile. It is a plain Makefile so it can be copied

LIB = libfmcadc.a
LOBJ := fa-init.o

CFLAGS = -Wall -ggdb -O2 -I../kernel -I../zio/include
LDFLAGS = -L. -lfmcadc

modules all: lib

lib: $(LIB)


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

