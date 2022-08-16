# SPDX-FileCopyrightText: 2020 CERN (home.cern)
#
# SPDX-License-Identifier: CC0-1.0

TOP_DIR = ../../..

FILES := spec_base_regs
FILES += svec_base_regs
FILES += svec_ref_fmc_adc_100Ms_mmap
FILES += spec_ref_fmc_adc_100Ms_mmap
FILES += fmc_adc_mezzanine_mmap


CHEBY ?= cheby

all: html rest

html : $(addsuffix .htm, $(FILES))
rest : $(addsuffix .rst, $(FILES))

.PHONY: clean html rest

vpath %.cheby $(TOP_DIR)/hdl/cheby/
vpath %.cheby $(TOP_DIR)/hdl/ip_cores/spec/hdl/rtl/
vpath %.cheby $(TOP_DIR)/hdl/ip_cores/svec/hdl/rtl/

%.htm: %.cheby
	$(CHEBY) -i $< --gen-doc --doc html > $@

%.rst: %.cheby
	$(CHEBY) -i $< --gen-doc --doc rest > $@

clean:
	@rm -f *.md *.rst *.htm
