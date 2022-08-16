# SPDX-FileCopyrightText: 2020 CERN (home.cern)
#
# SPDX-License-Identifier: CC0-1.0

SIM   =../../testbench/include
DOC   =../../../doc/manual
SW    =../../../software/include/hw

SOURCES = $(wildcard *.cheby)
TARGETS = $(SOURCES:.cheby=.vhd)

CHEBY ?= cheby

all: $(TARGETS)

.PHONY: $(TARGETS)

$(TARGETS): %.vhd : %.cheby
	@echo "\n\033[34m\033[1m-> Processing file $<\033[0m"
	@$(CHEBY) -i $< --gen-hdl=$@
	@$(CHEBY) -i $< \
		--gen-doc=$(DOC)/$(@:.vhd=.html) \
	        --gen-consts=$(SIM)/$(@:.vhd=.v) \
		--gen-c=$(SW)/$(@:.vhd=.h)
