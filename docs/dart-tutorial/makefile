all: pdf

pdf: dart-tutorial.pdf

dart-tutorial.dvi: *.tex *.eps *.bib
	latex dart-tutorial.tex
	bibtex dart-tutorial
	latex dart-tutorial.tex
	latex dart-tutorial.tex

dart-tutorial.pdf: dart-tutorial.dvi
	dvips -Ppdf -t letter -G0 -o dart-tutorial.ps dart-tutorial.dvi
	ps2pdf -dPDFSETTINGS=/prepress -dCompatibilityLevel=1.3 \
	-dAutoFilterColorImages=false -dAutoFilterGrayImages=false \
	-dColorImageFilter=/FlateEncode -dGrayImageFilter=/FlateEncode \
	-dMonoImageFilter=/FlateEncode -dDownsampleColorImages=false \
	-dDownsampleGrayImages=false dart-tutorial.ps dart-tutorial.pdf

clean:
	rm -f *.log *.pdf *.ps *.dvi *~ *.aux *.blg
