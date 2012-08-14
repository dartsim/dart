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

TARGETS = stereo-plane

LLDLIBS = $(GLUT) -lGLU -lGL -lXmu -lXext -lX11 -lm

SRCS = stereo-plane.c
OBJS = $(SRCS:.c=.o)

LCOPTS = -I$(TOP)/include -I$(TOP)/lib/glut -fullwarn
LWOFF = ,813,852,827,826
LDIRT = *~ *.bak *.pure

default : $(TARGETS)

stereo-plane: stereo-plane.o fullscreen_stereo.o
	$(CC) -o $@ $(LDFLAGS) stereo-plane.o fullscreen_stereo.o $(LDLIBS)

include $(COMMONRULES)
