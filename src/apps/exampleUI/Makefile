include ../../Makefile.defs

RELEASE_BIN = exampleUI
DEBUG_BIN = exampleUId

RF = $(*F)
DF = $(*F)

OBJS =  \
	Main.o \
	$(NULL)

IDIRS = -I. -I../../ -I$(EIGEN) -I$(TICPP)/include
LDIRS = -L../$(LIB_DIR) -L$(TICPP)/lib -L$(GLOG)/lib
LIBS = \
	$(NULL)

RELEASE_LIBS = \
	   -lYUI \
	   $(NULL)

DEBUG_LIBS = \
	   -lYUId \
	   $(NULL)

RELEASE_OBJS := $(foreach f,$(OBJS),$(patsubst %.cpp,%.o,$(f)))
DEBUG_OBJS := $(foreach f,$(OBJS),$(patsubst %.cpp,%.o,$(f)))

all: release debug
default: release

release : $(RELEASE_OBJS)
	$(CPP) $(CPPFLAGS_RELEASE) -DAPPLE -o $(RELEASE_BIN) $(RELEASE_OBJS) $(LIBS) $(LDIRS) $(RELEASE_LIBS) -bind_at_load $(GLDLIBS) $(LDFLAGS)
	$(CP) $(RELEASE_BIN) ../$(BIN_DIR)/$(RELEASE_BIN)

debug : $(DEBUG_OBJS)
	$(CPP) $(CPPFLAGS_DEBUG) -DAPPLE -o $(DEBUG_BIN) $(DEBUG_OBJS) $(LDIRS) $(LIBS) $(DEBUG_LIBS) -bind_at_load $(GLDLIBS) $(LDFLAGS)
	$(CP) $(DEBUG_BIN) ../$(BIN_DIR)/$(DEBUG_BIN)

%.o: %.cpp
	@echo $<
	@$(CPP) $(CPPFLAGS_RELEASE) -DAPPLE $(IDIRS) -MMD -c -o $@ $<
	@cp $(RF).d $(RF).P; \
		sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
		-e '/^$$/ d' -e 's/$$/ :/' < $(RF).P >> $(RF).d; \
		rm -f $(RF).P

%.o: %.cpp
	@echo $<
	@$(CPP) $(CPPFLAGS_DEBUG) -DAPPLE $(IDIRS) -MMD -c -o $@ $<
	@cp $(DF).d $(DF).P; \
		sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
		-e '/^$$/ d' -e 's/$$/ :/' < $(DF).P >> $(DF).d; \
		rm -f $(DF).P

-include $(OBJS:%.o=%.d)
-include $(OBJS:%.o=%.d)

clean:
	rm -f *.o *.d $(RELEASE_BIN) $(DEBUG_BIN)
