# SPDX-FileCopyrightText: 2020 CERN (home.cern)
#
# SPDX-License-Identifier: CC0-1.0

SIM   =../testbench/include

SOURCES = $(wildcard *.cheby)
TARGETS = $(SOURCES:.cheby=.vhd)

CHEBY ?= cheby

all: $(TARGETS)

.PHONY: $(TARGETS)

$(TARGETS): %.vhd : %.cheby
	@echo -e "\n\033[34m\033[1m-> Processing file $<\033[0m"
	$(CHEBY) -i $< --gen-hdl=$@
	$(CHEBY) -i $< --gen-consts=$(SIM)/$(@:.vhd=.v)
	$(CHEBY) -i $< --gen-c=../../software/include/hw/$(<:.cheby=.h)
