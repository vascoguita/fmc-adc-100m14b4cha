WBGEN2=$(shell which wbgen2)
CHEBY=cheby
RTL=../
SIM=../../testbench/include/
TEX=../../../doc/manual/

all: fmc_adc_100Ms_csr fmc_adc_eic

fmc_adc_100Ms_csr:
	$(WBGEN2) -l vhdl -H record -V $(RTL)$@.vhd -p $(RTL)$@_wbgen2_pkg.vhd -f html -D $@.htm -C $@.h  -K $(SIM)$@.v $@.wb
	$(WBGEN2) -f texinfo -D $(TEX)$@.tex $@.wb

fmc_adc_eic:
	$(WBGEN2) -l vhdl -V $(RTL)$@.vhd -f html -D $@.htm -C $@.h $@.wb
	$(WBGEN2) -f texinfo -D $(TEX)$@.tex $@.wb

fmc_adc_alt_trigin:
	$(CHEBY) --gen-hdl=$(RTL)/$@.vhd --gen-consts=$(SIM)/$@.v -i $@.cheby

fmc_adc_alt_trigout:
	$(CHEBY) --gen-hdl=$(RTL)/$@.vhd --gen-consts=$(SIM)/$@.v -i $@.cheby
