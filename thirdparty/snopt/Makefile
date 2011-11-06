F77 = gfortran
#F77FLAGS = -m32
RANLIB = ranlib

LIB = libsnopt.a
OBJS = \
	SN09TIME.o \
	SN10UNIX.o \
	SN11USER.o \
	SN12NPZZ.o \
	SN12SNZZ.o \
	SN12SQZZ.o \
	SN15BLAS.o \
	SN17UTIL.o \
	SN20AMAT.o \
	SN25BFAC.o \
	SN27LU.o \
	SN30SPEC.o \
	SN35MPS.o \
	SN40BFIL.o \
	SN50LP.o \
	SN55QP.o \
	SN57QOPT.o \
	SN60SRCH.o \
	SN65RMOD.o \
	SN70NOBJ.o \
	SN80NCON.o \
	SN85HESS.o \
	SN87SOPT.o \
	SN90LMQN.o \
	SN95FMQN.o \
	SNWORKSP.o \
	$(NULL)

.SUFFIXES: .o .F

.F.o:
	$(F77) $(F77FLAGS) -c -o $*.o $<

all: $(LIB)

$(LIB): $(OBJS)
	$(AR) cr $@ $(OBJS)
	$(RANLIB) $@

clean:
	rm -f $(LIB) $(OBJS)
