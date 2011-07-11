#!smake
#
# Copyright (c) Mark J. Kilgard, 1995, 1997.
#

# Force O32 compilation because SGI's gnat_dev Ada development system
# does not support N32 (at this writing)
OBJECT_STYLE = 32

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

TARGETS = ada_sphere bezmesh cone texturesurf texgen teapots scenebamb pickdepth dof fog

LLDLIBS = $(GLUT) -lGLU -lGL -lXmu -lXext -lX11 -lm

default : $(TARGETS)

ada_sphere: ada_sphere.o ada_sphere_procs.o $(TOP)/adainclude/GL/glut.o $(GLUT)
	gnatmake $(GNATMAKEFLAGS) -c ada_sphere
	gnatbl $(GNATBLFLAGS) $(BF) $(BL) -o ada_sphere ada_sphere.ali $(LDLIBS)

bezmesh: bezmesh.o bezmesh_procs.o $(TOP)/adainclude/GL/glut.o $(GLUT)
	gnatmake $(GNATMAKEFLAGS) -c bezmesh
	gnatbl $(GNATBLFLAGS) $(BF) $(BL) -o bezmesh bezmesh.ali $(LDLIBS)

cone: cone.o cone_procs.o $(TOP)/adainclude/GL/glut.o $(GLUT)
	gnatmake $(GNATMAKEFLAGS) -c cone
	gnatbl $(GNATBLFLAGS) $(BF) $(BL) -o cone cone.ali $(LDLIBS)

texturesurf: texturesurf.o texturesurf_procs.o $(TOP)/adainclude/GL/glut.o $(GLUT)
	gnatmake $(GNATMAKEFLAGS) -c texturesurf
	gnatbl $(GNATBLFLAGS) $(BF) $(BL) -o texturesurf texturesurf.ali $(LDLIBS)

texgen: texgen.o texgen_procs.o $(TOP)/adainclude/GL/glut.o $(GLUT)
	gnatmake $(GNATMAKEFLAGS) -c texgen
	gnatbl $(GNATBLFLAGS) $(BF) $(BL) -o texgen texgen.ali $(LDLIBS)

teapots: teapots.o teapots_procs.o $(TOP)/adainclude/GL/glut.o $(GLUT)
	gnatmake $(GNATMAKEFLAGS) -c teapots
	gnatbl $(GNATBLFLAGS) $(BF) $(BL) -o teapots teapots.ali $(LDLIBS)

scenebamb: scenebamb.o scenebamb_procs.o $(TOP)/adainclude/GL/glut.o $(GLUT)
	gnatmake $(GNATMAKEFLAGS) -c scenebamb
	gnatbl $(GNATBLFLAGS) $(BF) $(BL) -o scenebamb scenebamb.ali $(LDLIBS)

pickdepth: pickdepth.o pickdepth_procs.o $(TOP)/adainclude/GL/glut.o $(GLUT)
	gnatmake $(GNATMAKEFLAGS) -c pickdepth
	gnatbl $(GNATBLFLAGS) $(BF) $(BL) -o pickdepth pickdepth.ali $(LDLIBS)

dof: dof.o dof_procs.o jitter.o $(TOP)/adainclude/GL/glut.o $(GLUT)
	gnatmake $(GNATMAKEFLAGS) -c dof
	gnatbl $(GNATBLFLAGS) $(BF) $(BL) -o dof dof.ali $(LDLIBS)

fog: fog.o fog_procs.o $(TOP)/adainclude/GL/glut.o $(GLUT)
	gnatmake $(GNATMAKEFLAGS) -c fog
	gnatbl $(GNATBLFLAGS) $(BF) $(BL) -o fog fog.ali $(LDLIBS)

$(TOP)/adainclude/GL/glut.o:
	cd $(TOP)/adainclude/GL ; $(MAKE) glut.o

ada_sphere.o: ada_sphere.adb ada_sphere_procs.ads
ada_sphere_procs.o: ada_sphere_procs.adb ada_sphere_procs.ads
bezmesh.o: bezmesh.adb bezmesh_procs.ads
bezmesh_procs.o: bezmesh_procs.adb bezmesh_procs.ads
cone.o: cone.adb cone_procs.ads
cone_procs.o: cone_procs.adb cone_procs.ads
texturesurf.o: texturesurf.adb texturesurf_procs.ads
texturesurf_procs.o: texturesurf_procs.adb texturesurf_procs.ads
texgen.o: texgen.adb texgen_procs.ads
texgen_procs.o: texgen_procs.adb texgen_procs.ads
teapots.o: teapots.adb teapots_procs.ads
teapots_procs.o: teapots_procs.adb teapots_procs.ads
scenebamb.o: scenebamb.adb scenebamb_procs.ads
scenebamb_procs.o: scenebamb_procs.adb scenebamb_procs.ads
pickdepth.o: pickdepth.adb pickdepth_procs.ads
pickdepth_procs.o: pickdepth_procs.adb pickdepth_procs.ads
dof.o: dof.adb dof_procs.ads
dof_procs.o: dof_procs.adb dof_procs.ads
fog.o: fog.adb fog_procs.ads
fog_procs.o: fog_procs.adb fog_procs.ads

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

LDIRT = *~ *.bak *.pure *.ali b_*.c

default : $(TARGETS)

include $(COMMONRULES)
