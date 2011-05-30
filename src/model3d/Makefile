include ../Makefile.defs

RELEASE_LIB = $(LIB_DIR)/libModel3D.a
DEBUG_LIB = $(LIB_DIR)/libModel3Dd.a

OBJS = \
	marker.o \
	dof.o \
	joint.o \
	bodynode.o \
	primitive.o \
	primitive_cube.o \
	primitive_ellipsoid.o \
	transformation.o \
	trfm_rotate_euler.o \
	trfm_rotate_expmap.o \
	trfm_rotate_quat.o \
	trfm_translate.o \
	skeleton.o \
	c3d.o \
	fileinfo_c3d.o \
	fileinfo_dof.o \
	parser_model.o \
	skel_scanner.o \
	parser_vsk.o \
	fileinfo_model.o \
	$(NULL)

IDIRS = -I. -I.. -I$(EIGEN) -I/usr/include/malloc -I$(TICPP) -I$(BOOST)

RELEASE_OBJS := $(foreach f,$(OBJS),$(RELEASE_DIR)/$(patsubst %.cpp,%.o,$(f)))

DEBUG_OBJS := $(foreach f,$(OBJS),$(DEBUG_DIR)/$(patsubst %.cpp,%.o,$(f)))

default: release
all: release debug

release : $(RELEASE_OBJS)
	$(AR) cr $(RELEASE_LIB) $(RELEASE_OBJS)
	$(RANLIB) $(RELEASE_LIB)

debug : $(DEBUG_OBJS)
	$(AR) cr $(DEBUG_LIB) $(DEBUG_OBJS)
	$(RANLIB) $(DEBUG_LIB)

$(RELEASE_DIR)/%.o: %.cpp
	@echo $<
	@$(CPP) $(CPPFLAGS_RELEASE) -DAPPLE -D_RENDERER_TEST $(IDIRS) -MMD -c -o $@ $<
	@cp $(RF).d $(RF).P; \
		sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
		-e '/^$$/ d' -e 's/$$/ :/' < $(RF).P >> $(RF).d; \
		rm -f $(RF).P

$(DEBUG_DIR)/%.o: %.cpp
	@echo $<
	@$(CPP) $(CPPFLAGS_DEBUG) -DAPPLE -D_RENDERER_TEST $(IDIRS) -MMD -c -o $@ $<
	@cp $(DF).d $(DF).P; \
		sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
		-e '/^$$/ d' -e 's/$$/ :/' < $(DF).P >> $(DF).d; \
		rm -f $(DF).P

-include $(OBJS:%.o=$(RELEASE_DIR)/%.d)
-include $(OBJS:%.o=$(DEBUG_DIR)/%.d)

clean:
	rm -f $(RELEASE_LIB) $(RELEASE_OBJS) $(RELEASE_DIR)/*.d $(DEBUG_LIB) $(DEBUG_OBJS) $(DEBUG_DIR)/*.d
