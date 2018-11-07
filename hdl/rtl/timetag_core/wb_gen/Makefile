WBGEN2=$(shell which wbgen2)
RTL=../
TEX=../../../../doc/manual/
SIM=../../../testbench/include/

timetag_core_regs:
	$(WBGEN2) -l vhdl -H record -V $(RTL)$@.vhd -p $(RTL)$@_wbgen2_pkg.vhd -f html -D $@.htm -C $@.h -K $(SIM)$@.v $@.wb
	$(WBGEN2) -f texinfo -D $(TEX)$@.tex $@.wb
