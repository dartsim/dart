include ../../Makefile.defs

RELEASE_BIN = viewer
DEBUG_BIN = viewerd

RF = $(*F)
DF = $(*F)

RELEASE_DIR = .
DEBUG_DIR = .

OBJS =  \
	MyWindow.o \
	Main.o \
	$(NULL)

IDIRS = -I. -I../../ -I$(EIGEN)
LDIRS = -L../$(LIB_DIR) -L$(TICPP)/lib -L$(GLOG)/lib
LIBS = \
	-lticpp \
	$(NULL)

RELEASE_LIBS = \
	   -lModel3D \
	   -lUtils \
	   -lYUI \
	   -lRenderer \
	   $(NULL)

DEBUG_LIBS = \
	   -lModel3Dd \
	   -lUtilsd \
	   -lYUId \
	   -lRenderer \
	   $(NULL)

RELEASE_OBJS := $(foreach f,$(OBJS),$(RELEASE_DIR)/$(patsubst %.cpp,%.o,$(f)))
DEBUG_OBJS := $(foreach f,$(OBJS),$(DEBUG_DIR)/$(patsubst %.cpp,%.o,$(f)))

default: release
all: debug release

release : $(RELEASE_OBJS)
	$(CPP) $(CPPFLAGS_RELEASE) -DAPPLE -o $(RELEASE_BIN) $(RELEASE_OBJS) $(LIBS) $(LDIRS) $(RELEASE_LIBS) -bind_at_load $(GLDLIBS) $(LDFLAGS)
	$(CP) $(RELEASE_BIN) ../$(BIN_DIR)/$(RELEASE_BIN)

debug : $(DEBUG_OBJS)
	$(CPP) $(CPPFLAGS_DEBUG) -DAPPLE -o $(DEBUG_BIN) $(DEBUG_OBJS) $(LDIRS) $(LIBS) $(DEBUG_LIBS) -bind_at_load $(GLDLIBS) $(LDFLAGS)
	$(CP) $(DEBUG_BIN) ../$(BIN_DIR)/$(DEBUG_BIN)

$(RELEASE_DIR)/%.o: %.cpp
	@echo $<
	@$(CPP) $(CPPFLAGS_RELEASE) -DAPPLE $(IDIRS) -MMD -c -o $@ $<
	@cp $(RF).d $(RF).P; \
		sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
		-e '/^$$/ d' -e 's/$$/ :/' < $(RF).P >> $(RF).d; \
		rm -f $(RF).P

$(DEBUG_DIR)/%.o: %.cpp
	@echo $<
	@$(CPP) $(CPPFLAGS_DEBUG) -DAPPLE $(IDIRS) -MMD -c -o $@ $<
	@cp $(DF).d $(DF).P; \
		sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
		-e '/^$$/ d' -e 's/$$/ :/' < $(DF).P >> $(DF).d; \
		rm -f $(DF).P

-include $(OBJS:%.o=$(RELEASE_DIR)/%.d)
-include $(OBJS:%.o=$(DEBUG_DIR)/%.d)

clean:
	rm -f $(BIN).exe $(BIN).exe.stackdump $(RELEASE_OBJS) $(DRAW_OBJS) $(RELEASE_DIR)/*.d $(DEBUG_OBJS) $(DEBUG_DIR)/*.d $(BIN)
