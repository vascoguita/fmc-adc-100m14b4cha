SOURCES = $(wildcard *.cheby)
TARGETS = $(SOURCES:.cheby=.htm)

all: $(TARGETS)

.PHONY: $(TARGETS) clean

$(TARGETS): %.htm : %.cheby
	@echo "\n\033[34m\033[1m-> Processing file $<\033[0m"
	@cheby -i $< --gen-doc=$@             --doc html
#	@cheby -i $< --gen-doc=$(@:.htm=.rst) --doc rest
#	@cheby -i $< --gen-doc=$(@:.htm=.md)  --doc md

clean:
	@rm -f *.md *.rst *.htm
