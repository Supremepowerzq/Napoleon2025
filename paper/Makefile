# LaTeX build Makefile
# Usage:
#   make        → compile PDF (pdflatex + bibtex + pdflatex ×2)
#   make clean  → remove auxiliary files
#   make open   → open the PDF

MAIN = main
LATEX = pdflatex
BIBTEX = bibtex

.PHONY: all clean open

all:
	$(LATEX) -interaction=nonstopmode $(MAIN).tex
	$(BIBTEX) $(MAIN)
	$(LATEX) -interaction=nonstopmode $(MAIN).tex
	$(LATEX) -interaction=nonstopmode $(MAIN).tex

clean:
	rm -f $(MAIN).aux $(MAIN).bbl $(MAIN).blg $(MAIN).log \
	      $(MAIN).out $(MAIN).toc $(MAIN).lof $(MAIN).lot \
	      $(MAIN).fls $(MAIN).fdb_latexmk

open:
	start $(MAIN).pdf
