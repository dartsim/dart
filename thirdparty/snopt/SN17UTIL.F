*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
*     File  sn17util.f
*     
*     Various utility routines for SNOPT.
*     This file does not contain Level 1 or Level 2  BLAS routines.
*
*     ddiv     dddiv    ddscl    dnrm1s   dload    ccopy
*     icopy    iload
*
*     These could be tuned to the machine being used.
*     dload  is used the most.
*
*     ddrand
*
*     Bibs and bobs 
*
*     s1init   s1time   s1timp
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      double precision   function ddiv( a, b, fail )

      implicit           double precision (a-h,o-z)
      logical            fail

*     ==================================================================
*     ddiv  returns the value div given by
*
*     div = ( a/b                 if a/b does not overflow,
*           (
*           ( 0.0                 if a .eq. 0.0,
*           (
*           ( sign( a/b )*flmax   if a .ne. 0.0  and a/b would overflow,
*
*     where  flmax  is a large value, via the function name. 
*     In addition, if  a/b  would overflow then  fail  is returned as
*     true, otherwise  fail  is returned as false.
*     
*     Note that when  a and b  are both zero, fail is returned as true, 
*     but  div  is returned as  0.0. In all other cases of overflow 
*     div is such that  abs( div ) = flmax.
*
*     When  b = 0  then  sign( a/b )  is taken as  sign( a ).
*
*     15-Nov-91: First version based on Nag routine f06.
*     22-Jul-97: Current version.
*     ==================================================================
      double precision      s1flmn, s1flmx
      parameter           ( one = 1.0d+0, zero = 0.0d+0 )
      intrinsic             abs, sign
*     ------------------------------------------------------------------

      flmax = s1flmx( )
      flmin = s1flmn( )

      if (a .eq. zero) then
         div = zero
         if (b .eq. zero) then
            fail = .true.
         else
            fail = .false.
         end if
      else
         if (b .eq. zero) then
            div  =  sign( flmax, a )
            fail = .true.
         else
            absb = abs( b )
            if (absb .ge. one) then
               fail = .false.
               if (abs( a ) .ge. absb*flmin) then
                  div = a/b
               else
                  div = zero
               end if
            else
               if (abs( a ) .le. absb*flmax) then
                  fail = .false.
                  div  =  a/b
               else
                  fail = .true.
                  div  = flmax
                  if (((a .lt. zero) .and. (b .gt. zero))  .or.
     $                ((a .gt. zero) .and. (b .lt. zero)))
     $               div = -div
               end if
            end if
         end if
      end if

      ddiv = div

*     end of ddiv
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine dddiv ( n, d, incd, x, incx )

      implicit           double precision (a-h,o-z)
      double precision   d(*), x(*)

*     dddiv  performs the diagonal scaling  x  =  x / d.

      integer            i, id, ix
      external           dscal
      intrinsic          abs
      parameter        ( one = 1.0d+0 )

      if (n .gt. 0) then
         if (incd .eq. 0  .and.  incx .ne. 0) then
            call dscal ( n, one/d(1), x, abs(incx) )
         else if (incd .eq. incx  .and.  incd .gt. 0) then
            do 10 id = 1, 1 + (n - 1)*incd, incd
               x(id) = x(id) / d(id)
   10       continue
         else
            if (incx .ge. 0) then
               ix = 1
            else
               ix = 1 - (n - 1)*incx
            end if
            if (incd .gt. 0) then
               do 20 id = 1, 1 + (n - 1)*incd, incd
                  x(ix) = x(ix) / d(id)
                  ix    = ix   + incx
   20          continue
            else
               id = 1 - (n - 1)*incd
               do 30  i = 1, n
                  x(ix) = x(ix) / d(id)
                  id    = id + incd
                  ix    = ix + incx
   30          continue
            end if
         end if
      end if

*     end of dddiv
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine ddscl ( n, d, incd, x, incx )

      integer            incd, incx, n
      double precision   d(*), x(*)

*     ddscl  performs the diagonal scaling  x  =  d * x.

      integer            i, id, ix
      external           dscal
      intrinsic          abs

      if (n .gt. 0) then
         if (incd .eq. 0  .and.  incx .ne. 0) then
            call dscal ( n, d(1), x, abs(incx) )
         else if (incd .eq. incx  .and.  incd .gt. 0) then
            do 10 id = 1, 1 + (n - 1)*incd, incd
               x(id) = d(id)*x(id)
   10       continue
         else
            if (incx .ge. 0) then
               ix = 1
            else
               ix = 1 - (n - 1)*incx
            end if
            if (incd .gt. 0) then
               do 20 id = 1, 1 + (n - 1)*incd, incd
                  x(ix) = d(id)*x(ix)
                  ix    = ix + incx
   20          continue
            else
               id = 1 - (n - 1)*incd
               do 30  i = 1, n
                  x(ix) = d(id)*x(ix)
                  id    = id + incd
                  ix    = ix + incx
   30          continue
            end if
         end if
      end if

*     end of ddscl
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      function   dnrm1s( n, x, incx )

      implicit           double precision (a-h,o-z)
      double precision   x(n)

*     ==================================================================
*     dnrm1s  returns the 1-norm of the vector  x,  scaled by root(n).
*     This approximates the two-norm of x without the expense.
*     ==================================================================
      parameter           ( zero = 0.0d+0 )

      if (n .lt. 1) then
         dnrm1s = zero
      else
         d      = n
         d      = dasum( n, x, incx ) / sqrt(d)
         dnrm1s = d
      end if

*     end of dnrm1s
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine dload ( n, const, x, incx )

      implicit           double precision (a-h,o-z)
      double precision   x(*)
*     ==================================================================
*     dload  loads every component of a vector x with the constant.
*     Special attention is given to the case incx = 1, const = zero.
*     ==================================================================
      parameter        ( zero = 0.0d+0 )
      intrinsic          mod
*     ------------------------------------------------------------------

      if (n .gt. 0) then
         if (const .eq. zero  .and.  incx .eq. 1) then
            m    = mod( n, 7 )
            do 30, i = 1, m
               x(i) = zero
   30       continue

            do  50, i = m+1, n, 7
               x(i)   = zero
               x(i+1) = zero
               x(i+2) = zero
               x(i+3) = zero
               x(i+4) = zero
               x(i+5) = zero
               x(i+6) = zero
   50       continue

         else if (const .eq. zero) then
            do 10, ix = 1, 1+(n-1)*incx, incx
               x(ix)  = zero
   10       continue
         else 
            do 20, ix = 1, 1+(n-1)*incx, incx
               x(ix)  = const
   20       continue
         end if
      end if

*     end of dload
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine ccopy ( n, x, incx, y, incy )

      character*8        x(*), y(*)
      integer            incx, incy

*     ccopy  is the character*8 version of dcopy.

      integer            ix, iy

      if (n .gt. 0) then
         if (incx .eq. incy  .and.  incy .gt. 0) then
            do 10 iy = 1, 1 + (n - 1)*incy, incy
               y(iy) = x(iy)
   10       continue
         else
            if (incx .ge. 0) then
               ix = 1
            else
               ix = 1 - (n - 1)*incx
            end if
            if (incy .gt. 0) then
               do 20 iy = 1, 1 + ( n - 1 )*incy, incy
                  y(iy) = x(ix)
                  ix    = ix + incx
   20          continue
            else
               iy = 1 - (n - 1)*incy
               do 30 i  = 1, n
                  y(iy) = x(ix)
                  iy    = iy + incy
                  ix    = ix + incx
   30          continue
            end if
         end if
      end if

*     end of ccopy
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine icopy ( n, x, incx, y, incy )

      integer            x(*), y(*)
      integer            incx, incy

*     icopy  is the integer version of dcopy.

      integer            ix, iy

      if (n .gt. 0) then
         if (incx .eq. incy  .and.  incy .gt. 0) then
            do 10 iy = 1, 1 + (n - 1)*incy, incy
               y(iy) = x(iy)
   10       continue
         else
            if (incx .ge. 0) then
               ix = 1
            else
               ix = 1 - (n - 1)*incx
            end if
            if (incy .gt. 0) then
               do 20 iy = 1, 1 + ( n - 1 )*incy, incy
                  y(iy) = x(ix)
                  ix    = ix + incx
   20          continue
            else
               iy = 1 - (n - 1)*incy
               do 30 i  = 1, n
                  y(iy) = x(ix)
                  iy    = iy + incy
                  ix    = ix + incx
   30          continue
            end if
         end if
      end if

*     end of icopy
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      
      subroutine iload ( n, const, x, incx )

      integer            incx, n
      integer            const
      integer            x(*)

*     iload  loads elements of x with const.

      integer            ix

      if (n .gt. 0) then
         if (incx .eq. 1  .and.  const .eq. 0) then
            do 10 ix = 1, n
               x(ix) = 0
   10       continue
         else
            do 20 ix = 1, 1 + (n - 1)*incx, incx
               x(ix) = const
   20       continue
         end if
      end if

*     end of iload
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine ddrand( n, x, incx, iseed1, iseed2, iseed3 )

      implicit           double precision(a-h, o-z)
      integer            iseed1, iseed2, iseed3
      double precision   x(*)

*     ------------------------------------------------------------------
*     ddrand fills a vector x with uniformly distributed random numbers
*     in the interval (0, 1) using a method due to  Wichman and Hill.
*
*     ia, ib and ic should be set to integer values between 1 and 30000
*     before the first entry.
*
*     Integer arithmetic up to 30323 is required.
*
*     Blatantly copied from Wichman and Hill 19-January-1987.
*     14-Feb-94. Origial version.
*     19-Jul-97. This version of ddrand.
*     ------------------------------------------------------------------
      if (n .lt. 1) return

      do 100, ix = 1, 1+(n-1)*incx, incx
         iseed1     = 171*mod(iseed1, 177) -  2*(iseed1/177)
         iseed2     = 172*mod(iseed2, 176) - 35*(iseed2/176)
         iseed3     = 170*mod(iseed3, 178) - 63*(iseed3/178)
         
         if (iseed1 .lt. 0) iseed1 = iseed1 + 30269
         if (iseed2 .lt. 0) iseed2 = iseed2 + 30307
         if (iseed3 .lt. 0) iseed3 = iseed3 + 30323
         
         x(ix)  = mod( real(iseed1)/30269.0 +
     $                 real(iseed2)/30307.0 + real(iseed3)/30323.0, 1.0)
  100 continue

*     end of ddrand
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s1init( title, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*30       title
      integer            iw(leniw)
      double precision   rw(lenrw)
*     ==================================================================
*     s1init saves some machine-dependent constants in iw and rw. 
*     ==================================================================
      integer            eps, eps0, eps1, eps2, eps3, eps4, eps5
      integer            flmax, flmin, rtUndf
      parameter         (eps       =   1)
      parameter         (eps0      =   2)
      parameter         (eps1      =   3)
      parameter         (eps2      =   4)
      parameter         (eps3      =   5)
      parameter         (eps4      =   6)
      parameter         (eps5      =   7)
      parameter         (flmax     =   8)
      parameter         (flmin     =   9)
      parameter         (rtundf    =  10)
*     ------------------------------------------------------------------
      rw(eps   )  = s1eps ( )
      rw(flmax )  = s1flmx( )
      rw(flmin )  = s1flmn( )
      rw(rtUndf)  = sqrt( rw(flmin) )

*     Use eps to set other machine precision constants.

      rw(eps0)   = rw(eps)**(0.80d+0)
      rw(eps1)   = rw(eps)**(0.67d+0)
      rw(eps2)   = rw(eps)**(0.50d+0)
      rw(eps3)   = rw(eps)**(0.33d+0)
      rw(eps4)   = rw(eps)**(0.25d+0)
      rw(eps5)   = rw(eps)**(0.20d+0)

*     Set the environment (for later use).

      call s1envt( 0, iw, leniw )

*     end of s1init
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s1time( clock, prtopt, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      integer            iw(leniw)
      double precision   rw(lenrw)

      integer            clock, prtopt

      parameter         (ntime   =   5)
      parameter         (lnumt   = 451)
      parameter         (ltlast  = 451)
      parameter         (ltsum   = 461)
      parameter         (lvlTim  =  77)
*     ------------------------------------------------------------------
      iPrint  = iw( 12)
      iSumm   = iw( 13)

      call s1body( iPrint, iSumm, clock, prtopt, ntime, iw(lvlTim), 
     $             rw(ltlast), rw(ltsum), iw(lnumt) )

      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s1body( iPrint, iSumm, clock, prtopt, ntime, lvlTim, 
     $                   tlast, tsum, numt )

      implicit           double precision (a-h,o-z)
      integer            clock, prtopt
      integer            numt(ntime)
      double precision   tlast(ntime), tsum(ntime)

*     ------------------------------------------------------------------
*     s1time, s1timp and s1cpu are derived from timer, timout and nowcpu
*     written for DEC VAX/VMS systems by Irvin Lustig,
*     Department of Operations Research, Stanford University, 1987.
*
*     SNOPT  calls s1time only.  s1time calls s1cpu  and  s1timp.
*     Only s1cpu is intrinsically machine dependent.
*
*     If a timer is available, call it in s1cpu  and arrange that
*     s1cpu  returns the current CPU time in seconds.
*
*     If a timer is not available or not wanted, set time = 0.0 in s1cpu.
*     Timing will be turned off and s1timp will not be called.
*     ------------------------------------------------------------------
*
*     s1time turns on or off a selected clock and optionally prints
*     statistics regarding all clocks or just the clock chosen.
*
*     The value of abs(clock) is which clock to use.
*     If clock  = 0 and prtopt = 0, all clocks and statistics are reset.
*     If clock  > 0, the clock is reset to start timing at the
*                    current time (determined by calling the
*                    machine-dependent subroutine s1cpu).
*     If clock  < 0, the clock is turned off and the statistic is
*                    recorded for the amount of time since the clock
*                    was turned on.
*
*     prtopt is the print option.
*     If lvlTim < 0, nothing is printed.  Otherwise,
*     prtopt =  0 indicates print nothing,
*            =  1 indicates print last time for this clock,
*                 only if clock < 0 (it has just been turned off),
*            =  2 indicates print total time for all clocks,
*            =  3 indicates print mean  time for all clocks.
*
*     The procedure for adding a new timer n is as follows:
*     1)  Change ntime to n in the parameter statement in s1time.
*     2)  Expand the array "label" to length n in subroutine s1timp.
*
*     04 Jun 1989: Irv's VMS/VAXC version of s1cpu installed,
*                  with changes to return time in seconds.
*     10 Jul 1992: More clocks added for use in AMPL (and elsewhere).
*     ------------------------------------------------------------------
*
*        Clock 1 is for input time.                         
*        Clock 2 is for solve time.                         
*        Clock 3 is for output time.                        
*        Clock 4 is for the nonlinear constraint functions. 
*        Clock 5 is for the nonlinear objective.
*
*        numt(i)  is the number of times clock i has been turned on.
*        tlast(i) is the time at which clock i was last turned on.
*        tsum(i)  is the total time elapsed while clock i was on.
*        lvlTim   is the Timing level set in the Specs file.
*     ------------------------------------------------------------------
      external           s1cpu
      real               time
      integer            iclock, ilo, ihi

      if (lvlTim .eq. 0) return
      iclock = iabs(clock)

      if (clock .eq. 0) then
         if (prtopt .eq. 0) then

*           clock = 0, prtopt = 0.  Reset everything.

            call s1cpu ( 1, time )
            call s1cpu ( 0, time )
            do 100, i = 1, ntime
               dtime    = dble(time)
               tlast(i) = dtime
               tsum(i)  = 0.0d+0
               numt(i)  = 0
  100       continue

*           If the s1cpu( 0, time ) gave time = 0.0, we assume that
*           the clock is a dummy.  Turn off future timing.

            if (time .le. 0.0) lvlTim = 0
         end if

      else
         call s1cpu ( 0, time )
         dtime = dble(time)           
         if (clock .gt. 0) then
            tlast(iclock) = dtime
         else
            stat         = dtime - tlast(iclock)
            tsum(iclock) = tsum(iclock) + stat
            numt(iclock) = numt(iclock) + 1
         end if
      end if

*     Now deal with print options.

      if (prtopt .eq. 0  .or.  lvlTim .lt. 0) then

*        Do nothing.

      else if (prtopt .eq. 1) then

*        Print statistic for last clock if just turned off.
      
         if (clock .lt. 0) then
            call s1timp( iPrint, iSumm, iclock, 'Last time', stat )
         end if

      else

*        prtopt >= 2.  Print all statistics if clock = 0,
*        or print statistic for individual clock.
      
         if (clock .eq. 0) then
            call s1cpu ( -1, time )
            ilo   = 1
            ihi   = ntime
         else
            ilo   = iclock
            ihi   = iclock
         end if
      
         do 400 i = ilo, ihi
            stat  = tsum(i)
            if (prtopt .eq. 2) then
               call s1timp(  iPrint, iSumm, i, 'Time', stat )
            else if (prtopt .eq. 3) then
               istat = numt(i)
               if (istat .gt. 0) stat = stat / istat
               call s1timp(  iPrint, iSumm, i, 'Mean time', stat )
            end if
  400    continue
      end if

*     end of s1time
      end

*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s1timp( iPrint, iSumm, iclock, lstat, stat )

      implicit           double precision (a-h,o-z)
      integer            iclock
      character*(*)      lstat

*     ------------------------------------------------------------------
*     s1timp  prints CPU time for s1time on file iPrint and/or iSumm.
*     It is not intrinsically machine dependent.
*
*     iclock  selects the correct label.
*     lstat   is a string to print to tell which type of statistic.
*     stat    is the statistic to print out.
*             If it is zero, we figure it was never timed, so no print.
*
*     ==================================================================
      character*24       label(5)
      data               label
     $                 / 'for MPS input',          
     $                   'for solving problem',    
     $                   'for solution output',    
     $                   'for constraint functions',
     $                   'for objective function' /

      if (iclock .eq. 1) then
         if (iPrint .gt. 0) write(iPrint, 1000)
         if (iSumm  .gt. 0) write(iSumm , 1000)
      end if

      if (iPrint .gt. 0) write(iPrint, 1000) lstat, label(iclock), stat
      if (iSumm  .gt. 0) write(iSumm , 1000) lstat, label(iclock), stat
      return

 1000 format( 1x, a, 1x, a, t38, f13.2,' seconds')

*     end of s1timp
      end

