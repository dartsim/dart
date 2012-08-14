all: pdf

pdf: angmomentum.pdf

angmomentum.dvi: *.tex *.eps *.bib
	latex angmomentum.tex
	bibtex angmomentum
	latex angmomentum.tex
	latex angmomentum.tex

angmomentum.pdf: angmomentum.dvi
	dvips -Ppdf -t letter -G0 -o angmomentum.ps angmomentum.dvi
	ps2pdf -dPDFSETTINGS=/prepress -dCompatibilityLevel=1.3 \
	-dAutoFilterColorImages=false -dAutoFilterGrayImages=false \
	-dColorImageFilter=/FlateEncode -dGrayImageFilter=/FlateEncode \
	-dMonoImageFilter=/FlateEncode -dDownsampleColorImages=false \
	-dDownsampleGrayImages=false angmomentum.ps angmomentum.pdf

clean:
	rm -f *.log *.pdf *.ps *.dvi *~ *.aux *.blg
