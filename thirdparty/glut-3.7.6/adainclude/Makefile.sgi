#!smake
#
# Copyright (c) Mark J. Kilgard, 1995.
#
include $(ROOT)/usr/include/make/commondefs

TOP = ..

SUBDIRS = GL

default $(COMMONTARGS): $(_FORCE)
	$(SUBDIRS_MAKERULE)

$(SUBDIRS): $(_FORCE)
	cd $@; $(MAKE)

$(_FORCE):
