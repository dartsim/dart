*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*     
*     File  sn09time.f 
*
*     s1cpu
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s1cpu ( mode, time )

      integer            mode
      real               time

*     ------------------------------------------------------------------
*     s1cpu is a machine-dependent routine to return time = cpu time
*     in seconds, so that 2 consecutive calls will indicate the
*     time difference of operations between the 2 calls.
*     The parameter 'mode' indicates what function should be done
*     to the timer.  This allows necessary initialization for certain
*     machines.
*     mode =  1  indicates initialization,
*     mode =  0  indicates normal use,
*     mode = -1  indicates stop the timer.
*
*     1988:  Used Irv Lustig's approach...
*     On DEC VAX/VMS systems we need to call the correct library
*     routine to get the timer statistics.  These statistics are
*     found by using the times() function in the VAX C Runtime library.
*     To use this version of s1cpu, one must create an options file
*     called  vmsc.opt  with the line
*        SYS$LIBRARY:VAXCRTL/SHARE
*     in it.   Then link using the usual command and append ,vmsc/opt
*     to the end of the line.  The name vmsc can be anything.
*     
*     02 Apr 1993: Went back to VMS Fortran routines to avoid linking
*                  to the C library.  (On DEC AXP, the C runtime lib
*                  appears to be translated from the VAX executable,
*                  and therefore requires linking with /NONATIVE,
*                  which possibly adds a small overhead to all
*                  subroutine calls.
*     ------------------------------------------------------------------

*-->  DEC OpenVMS with Fortran runtime library.
*-->  external        lib$init_timer, lib$stat_timer, lib$free_timer
*-->  integer         itimad, istatu, idata
*-->  save            itimad

*-->  DEC VAX/VMS with C runtime library.
*-->  integer            itimad(4)
*-->  PC Lahey Fortran
*-->  integer            itimad(4)

*-->  AIX
*-->  integer          mclock
*-->  intrinsic        real

*-->  Unix (SGI, Sun, DECstation)
      real               tarray(2)

      if (mode .eq. 1) then
*        ---------------------------------------------------------------
*        Initialize.
*        ---------------------------------------------------------------
         time   = 0.0
*-->     DEC OpenVMS with Fortran library.
*-->     istatu = lib$init_timer( itimad )
*-->     if (.not. istatu) call lib$signal( %val(istatu) )

      else if (mode .eq. 0) then
*        ---------------------------------------------------------------
*        Normal call.
*        Return current timer value here.
*        ---------------------------------------------------------------

*-->     DEC OpenVMS with Fortran library.
*-->     istatu returns the number of  centiseconds.
*-->     istatu = lib$stat_timer( 2, idata, itimad )
*-->     if (.not. istatu) call lib$signal( %val(istatu) )
*-->     time   = idata
*-->     time   = time * 0.01d+0

*-->     DEC VAX/VMS with C library.
*-->     itimad(1) returns the number of  centiseconds.
*-->     call times ( itimad )
*-->     time   = itimad(1)
*-->     time   = time * 0.01d+0

*-->     PC Lahey Fortran, itimad(1) returns the number of  centiseconds.
*-->     call timer ( itimad )
*-->     time   = itimad(1)
*-->     time   = time * 0.01d+0

*-->     On AIX, mclock returns hundredths of a second
*-->     time = real( mclock( ) ) / 100.0

*-->     On Unix (SGI MIPS RISC F77), etime returns seconds.
*-->     On Sun SPARC 10 also.
         time   = etime ( tarray )

*-->     On UNIX (NeXTstation M68040), using routine in ftime.c
*-->     call ftime(time)

*-->     On other machines, to forget about timing, just say
*-->     time   = -1.0

      else if (mode .eq. -1) then
*        ---------------------------------------------------------------
*        Stop the clock.
*        ---------------------------------------------------------------
         time   = 0.0

*-->     DEC OpenVMS with Fortran library.
*-->     istatu = lib$free_timer( itimad )
*-->     if (.not. istatu) call lib$signal( %val(istatu) )
      end if

*     end of s1cpu
      end

