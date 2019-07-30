WBGEN2=$(shell which wbgen2)
RTL=../
SIM=../../testbench/include/
TEX=../../../doc/manual/

all: fmc_adc_eic

fmc_adc_eic:
	$(WBGEN2) -l vhdl -V $(RTL)$@.vhd -f html -D $@.htm -C $@.h $@.wb
	$(WBGEN2) -f texinfo -D $(TEX)$@.tex $@.wb
