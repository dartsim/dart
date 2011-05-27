include ../Makefile.defs

RELEASE_LIB = $(LIB_DIR)/libRenderer.a
DEBUG_LIB = $(LIB_DIR)/libRendererd.a

RELEASE_DIR=.
DEBUG_DIR=.
SRC=.

OBJS = \
	OpenGLRenderInterface.o \
	$(NULL)

IDIRS = -I. -I$(OPENGL) -I$(GLUT) -I$(EIGEN)

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

$(RELEASE_DIR)/%.o: $(SRC)/%.cpp
	@echo $<
	@$(CPP) $(CPPFLAGS_RELEASE) -DAPPLE $(IDIRS) -MMD -c -o $@ $<
	@cp $(RF).d $(RF).P; \
		sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
		-e '/^$$/ d' -e 's/$$/ :/' < $(RF).P >> $(RF).d; \
		rm -f $(RF).P

$(DEBUG_DIR)/%.o: $(SRC)/%.cpp
	@echo $<
	@$(CPP) $(CPPFLAGS_DEBUG) -DAPPLE $(IDIRS) -MMD -c -o $@ $<
	@cp $(DF).d $(DF).P; \
		sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
		-e '/^$$/ d' -e 's/$$/ :/' < $(DF).P >> $(DF).d; \
		rm -f $(DF).P

-include $(OBJS:%.o=$(RELEASE_DIR)/%.d)
-include $(OBJS:%.o=$(DEBUG_DIR)/%.d)

clean:
	rm -f $(RELEASE_LIB) $(RELEASE_OBJS) $(RELEASE_DIR)/*.d $(DEBUG_LIB) $(DEBUG_OBJS) $(DEBUG_DIR)/*.d
