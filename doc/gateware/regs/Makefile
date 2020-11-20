# SPDX-FileCopyrightText: 2020 CERN (home.cern)
#
# SPDX-License-Identifier: CC0-1.0

SOURCES = $(wildcard *.cheby) ../../../hdl/cheby/fmc_adc_mezzanine_mmap.cheby
TARGETS = $(SOURCES:.cheby=.htm)

all: $(TARGETS)

.PHONY: $(TARGETS) clean

$(TARGETS): %.htm : %.cheby
	@echo -e "\n\033[34m\033[1m-> Processing file $<\033[0m"
	@cheby -i $< --gen-doc=$(shell basename $@) --doc html

clean:
	@rm -f *.md *.rst *.htm
