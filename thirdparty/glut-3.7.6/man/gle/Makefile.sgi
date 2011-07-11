#!smake
#
# Copyright (c) Mark J. Kilgard, 1995.
#

IDB_TAG = GLE_DEV_MAN
IDB_PATH = /usr/share/catman/p_man/cat3/GLE

MANPAGES = gle.man gleExtrusion.man gleHelicoid.man gleLathe.man \
  glePolyCone.man glePolyCylinder.man gleScrew.man gleSetJoinStyle.man \
  gleSpiral.man gleSuperExtrusion.man gleTextureMode.man gleToroid.man \
  gleTwistExtrusion.man

FORMATTED = $(MANPAGES:.man=.z)

POSTSCRIPT = $(MANPAGES:.man=.ps)

.SUFFIXES: .man .z .ps

default all: $(FORMATTED)

postscript: $(POSTSCRIPT)

install: $(FORMATTED)
	$(INSTALL) -idb "$(IDB_TAG)" -m 444 -F $(IDB_PATH) *.z
	$(INSTALL) -idb "$(IDB_TAG)" -lns glut.z -F $(IDB_PATH) intro.z
	$(INSTALL) -idb "$(IDB_TAG)" -lns gleSetJoinStyle.z -F $(IDB_PATH) gleGetJoinStyle.z
	
.man.z:
	nroff -man < $< > $*.nr
	pack $*.nr
	mv $*.nr.z $@

.man.ps:
	psroff -t -man < $< > $*.ps

clobber clean:
	-rm -rf *.z *.nr *.ps *.o *~
