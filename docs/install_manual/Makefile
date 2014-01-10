
MAINFILE := Install_Manual.tex
PDFFILE := Install_Manual.pdf

SECTION_FILES := $(shell find sections -name '*.tex')
PICTURE_FILES := $(shell find pics -name '*.png' -o -name '*.pdf' -o -name '*.jpg')

pdf: $(PDFFILE)

# executie pdflatex three times to be sure to have all references right.
$(PDFFILE): $(MAINFILE) $(SECTION_FILES)
	pdflatex $(MAINFILE)
	pdflatex $(MAINFILE)
	pdflatex $(MAINFILE)

clean:
	rm -f *.aux *.log *.out *.pdf *.toc sections/*.aux
