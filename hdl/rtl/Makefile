SIM   =../testbench/include
DOC   =../../doc/manual

SOURCES = $(wildcard *.cheby)
TARGETS = $(SOURCES:.cheby=.vhd)

all: $(TARGETS)

.PHONY: $(TARGETS)

$(TARGETS): %.vhd : %.cheby
	@echo "\n\033[34m\033[1m-> Processing file $<\033[0m"
	@cheby -i $< --gen-hdl=$@
	@cheby -i $< --doc=md --gen-doc=$(DOC)/$(@:.vhd=.adoc)
	@cheby -i $< --gen-consts=$(SIM)/$(@:.vhd=.v)
