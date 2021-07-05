# SPDX-FileCopyrightText: 2020 CERN (home.cern)
#
# SPDX-License-Identifier: CC0-1.0

TOP_DIR = ../../..

all: spec_base_regs.htm svec_base_regs.htm svec_ref_fmc_adc_100Ms_mmap.htm spec_ref_fmc_adc_100Ms_mmap.htm

.PHONY: clean

CHEBY_BUILD=(cd $(dir $<); cheby -i $(notdir $<) --gen-doc --doc html) > $@

spec_ref_fmc_adc_100Ms_mmap.htm: $(TOP_DIR)/hdl/cheby/spec_ref_fmc_adc_100Ms_mmap.cheby
	$(CHEBY_BUILD)

svec_ref_fmc_adc_100Ms_mmap.htm: $(TOP_DIR)/hdl/cheby/svec_ref_fmc_adc_100Ms_mmap.cheby
	$(CHEBY_BUILD)

svec_base_regs.htm: $(TOP_DIR)/hdl/ip_cores/svec/hdl/rtl/svec_base_regs.cheby
	$(CHEBY_BUILD)

spec_base_regs.htm: $(TOP_DIR)/hdl/ip_cores/spec/hdl/rtl/spec_base_regs.cheby
	$(CHEBY_BUILD)

clean:
	@rm -f *.md *.rst *.htm
