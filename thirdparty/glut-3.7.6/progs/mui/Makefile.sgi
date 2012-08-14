#!smake
#
# Copyright (c) Mark J. Kilgard, 1995, 1997.
#
include $(ROOT)/usr/include/make/commondefs

TOP = ../..

SUBDIRS = calc

default $(COMMONTARGS): $(_FORCE)
	$(SUBDIRS_MAKERULE)

$(SUBDIRS): $(_FORCE)
	cd $@; $(MAKE)

$(_FORCE):
