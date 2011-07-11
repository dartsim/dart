#!smake
#
# Copyright (c) Mark J. Kilgard, 1995, 1997.
#

TOP = ../..
include $(TOP)/glutdefs
include $(ROOT)/usr/include/make/commondefs

LN = ln -s
MV = mv
RM = -rm -rf

#-- begin Ada defs -------------------------------------------

# the C compiler
CC_PATH = $(ROOT)/usr/local/bin
CC = $(CC_PATH)/gcc

INCS = -I/usr/adainclude/sys -I/usr/adainclude -I/usr/adainclude/GL -I$(TOP)/adainclude/GL

# the Ada Compiler
ADAOPTS = -g
ADAC = $(CC) $(ADAOPTS)

# Gnat1 compilation flags
# GF = -gnatg
GF =

GNATMAKEFLAGS = -v -g $(INCS)
GNATBLFLAGS   = -v -g $(INCS)
# Gnatbind binder flags
# BF = -B/vobs/gnat/bin/
# BL = -L/vobs/gnat/adalib
# BF = -B/usr/local/bin/
# BL = -L/usr/local/lib/gcc-lib/mips-sgi-irix5.2/2.6.3/adalib

#-- end Ada defs ---------------------------------------------

TARGETS = glut.o

default : $(TARGETS)

glut.o: glut.adb glut.ads

#-- begin Ada rules ------------------------------------------

.SUFFIXES: .adb .ads .ali .o

.adb.o:
	$(ADAC) -c $(GF) $(INCS) $<

.ads.o:
	$(ADAC) -c $(GF) $(INCS) $<

.adb.s:
	$(ADAC) -c $(GF) -S $(INCS) $<

.ads.s:
	$(ADAC) -c $(GF) -S $(INCS) $<

.c.o:
	PATH=$(PATH):$(CC_PATH)  $(CC) -c $(CFLAGS) $<

% : %.o
	PATH=$(PATH):$(CC_PATH) $(CC_PATH)/gnatbl $(BF) $@.ali -o $@ $($@_DEP)
	./$@

#-- end Ada rules --------------------------------------------

LDIRT = *~ *.bak *.pure *.ali

default : $(TARGETS)

include $(COMMONRULES)
