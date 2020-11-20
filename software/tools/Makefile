# SPDX-FileCopyrightText: 2020 CERN (home.cern)
#
# SPDX-License-Identifier: CC0-1.0

# If it exists includes Makefile.specific. In this Makefile, you should put
# specific Makefile code that you want to run before this. For example,
# build a particular environment.
-include Makefile.specific

# include parent_common.mk for buildsystem's defines
REPO_PARENT ?= ../..
-include $(REPO_PARENT)/parent_common.mk

# user-space tools for spec-fine-delay
DESTDIR ?= /usr/local

GIT_VERSION := $(shell git describe --dirty --long --tags)
CFLAGS += -I../kernel -Wno-trigraphs -Wall -Werror -ggdb -O2  $(EXTRACFLAGS)
CFLAGS += -DGIT_VERSION="\"$(GIT_VERSION)\""

CPPCHECK ?= cppcheck
CC ?= $(CROSS_COMPILE)gcc

progs := fau-trg-config
progs += fau-acq-time
progs += fau-calibration
progs += parport-burst

# we are not in the kernel, so we need to piggy-back on "make modules"
all modules: $(progs)

clean:
	rm -f $(progs) *.o *~

# make nothing for modules_install, but avoid errors
modules_install:

install:
	install -d $(DESTDIR)/bin
	install -D $(progs) $(DESTDIR)/bin

# we need this as we are out of the kernel
%: %.c
	$(CC) $(CFLAGS) $^ -o $@

cppcheck:
	$(CPPCHECK) -q -I. --suppress=missingIncludeSystem --enable=all *.c *.h  --error-exitcode=1
