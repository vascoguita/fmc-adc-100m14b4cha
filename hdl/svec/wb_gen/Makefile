WBGEN2=$(shell which wbgen2)
RTL=../rtl/
TEX=../../../doc/manual/svec/

carrier_csr:
	$(WBGEN2) -l vhdl -H record -V $(RTL)$@.vhd -p $(RTL)$@_wbgen2_pkg.vhd -f html -D $@.htm -C $@.h $@.wb
	$(WBGEN2) -f texinfo -D $(TEX)$@.tex $@.wb
