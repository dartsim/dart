#!smake
#
# Copyright (c) Mark J. Kilgard, 1995.
#

TOP = ../..

sinclude $(TOP)/lib/fglut/Makefile.sgi
sinclude ObjectType.mk

LDIRT += $(SRCS)

$(ORIGSRCS) :
	$(RM) $@
	ln -s $(TOP)/lib/fglut/$@ $@
