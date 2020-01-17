SIM   =../testbench/include
SW    =../../software/include/hw

SOURCES = $(wildcard *.cheby)
TARGETS = $(SOURCES:.cheby=.vhd)

all: $(TARGETS)

.PHONY: $(TARGETS)

$(TARGETS): %.vhd : %.cheby
	@echo "\n\033[34m\033[1m-> Processing file $<\033[0m"
	@cheby -i $< --gen-hdl=$@
	@cheby -i $< \
	        --gen-consts=$(SIM)/$(@:.vhd=.v) \
		--gen-c=$(SW)/$(@:.vhd=.h)
