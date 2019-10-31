WBGEN2=$(shell which wbgen2)
RTL=../
TEX=../../../../doc/manual/spec/

all: dma_eic

dma_eic:
	$(WBGEN2) -l vhdl -V $(RTL)$@.vhd -f html -D $@.htm -C $@.h $@.wb
	$(WBGEN2) -f texinfo -D $(TEX)$@.tex $@.wb
