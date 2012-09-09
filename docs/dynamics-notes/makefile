all: pdf

pdf: dynamics.pdf

dynamics.dvi: *.tex *.eps *.bib
	latex dynamics.tex
	bibtex dynamics
	latex dynamics.tex
	latex dynamics.tex

dynamics.pdf: dynamics.dvi
	dvips -Ppdf -t letter -G0 -o dynamics.ps dynamics.dvi
	ps2pdf -dPDFSETTINGS=/prepress -dCompatibilityLevel=1.3 \
	-dAutoFilterColorImages=false -dAutoFilterGrayImages=false \
	-dColorImageFilter=/FlateEncode -dGrayImageFilter=/FlateEncode \
	-dMonoImageFilter=/FlateEncode -dDownsampleColorImages=false \
	-dDownsampleGrayImages=false dynamics.ps dynamics.pdf

clean:
	rm -f *.log *.pdf *.ps *.dvi *~ *.aux *.blg
