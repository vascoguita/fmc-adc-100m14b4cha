SVG=$(wildcard *.svg)
PDF=$(SVG:.svg=.pdf)

.PHONY: all
all: $(PDF)

%.pdf : %.svg
	inkscape --without-gui $< -A $@

clean:
	rm -f $(PDF)
	rm -f acq_fsm.pdf

show:
	$(info all svg: $(SVG))
	$(info all pdf: $(PDF))
