# SPDX-FileCopyrightText: 2020 CERN (home.cern)
#
# SPDX-License-Identifier: CC0-1.0

TOP_DIR = ../../..

HTML := spec_base_regs.htm
HTML += svec_base_regs.htm
HTML += svec_ref_fmc_adc_100Ms_mmap.htm
HTML += spec_ref_fmc_adc_100Ms_mmap.htm
HTML += fmc_adc_mezzanine_mmap.htm

CHEBY ?= cheby

all: $(HTML)

.PHONY: clean

vpath %.cheby $(TOP_DIR)/hdl/cheby/
vpath %.cheby $(TOP_DIR)/hdl/ip_cores/spec/hdl/rtl/
vpath %.cheby $(TOP_DIR)/hdl/ip_cores/svec/hdl/rtl/

%.htm: %.cheby
	$(CHEBY) -i $< --gen-doc --doc html > $@

clean:
	@rm -f *.md *.rst *.htm
