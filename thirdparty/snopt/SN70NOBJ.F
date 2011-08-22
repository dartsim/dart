*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
*     File  sn70nobj.f
*
*     s7chkd   s7chkg   s7chkJ
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s7chkd( n, bl, bu, x, dx, d, nfeas )

      implicit           double precision (a-h,o-z)
      double precision   bl(n), bu(n), x(n), dx, d(n)

*     ==================================================================
*     s7chkd  checks that x + dx*d is feasible.
*     It is used by s7chkg and s8chkJ for the cheap gradient checks.
*
*     Original:    Just looked at the sign of d for variables on a bound.
*     13 Mar 1992: dx added as a parameter to make certain that
*                  x + dx*d does not lie outside the bounds.
*                  d may be altered to achieve this.
*     09-Aug-92: First version based on Minos routine m7chkd.
*     09-Aug-92: Current version.
*     ==================================================================
      parameter        ( zero  = 0.0d+0 )
*     ------------------------------------------------------------------
      nfeas  = 0
      do 500, j = 1, n
         xj     = x(j)
         b1     = bl(j)
         b2     = bu(j)
         if (b1   .eq. b2  ) d(j) = zero

         if (d(j) .ne. zero) then

*           x(j) is not fixed, so there is room to move.
*           If xj + dx*dj is beyond one bound, reverse dj
*           and make sure it is not beyond the other.
*           Give up and use set dj = zero if both bounds are too close.

            dj     = d(j)
            xnew   = xj  +  dx*dj

            if (dj .gt. zero) then
               if (xnew .gt. b2) then
                  dj     = - dj
                  xnew   =   xj  +  dx*dj
                  if (xnew .lt. b1) dj = zero
               end if
            else
               if (xnew .lt. b1) then
                  dj     = - dj
                  xnew   =   xj  +  dx*dj
                  if (xnew .gt. b2) dj = zero
               end if
            end if

            d(j)   = dj
            if (dj .ne. zero) nfeas  = nfeas + 1
         end if
  500 continue

*     end of s7chkd
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s7chkg( ierror, nnObj, 
     $                   sfgObj, fgObj,
     $                   bl, bu, f, g, g2,
     $                   x, da, db,
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      external           sfgObj, fgObj
      double precision   bl(nnObj), bu(nnObj), da(nnObj), db(nnObj),
     $                   g(nnObj), g2(nnObj), x(nnObj)

      character*8        cu(lencu), cw(lencw)
      integer            iu(leniu), iw(leniw)
      double precision   ru(lenru), rw(lenrw)

*     ==================================================================
*     This routine checks that the gradient of an n-dimensional
*     function has been defined and programmed correctly.
*
*     First, a cheap heuristic test is performed, as in
*     subroutine chkgrd by the following authors:
*     Philip E. Gill, Walter Murray, Susan M. Picken and Hazel M. Barber
*     D.N.A.C., National Physical Laboratory, England  (circa 1975).
*
*     Next, a more reliable test is performed on each component of the
*     gradient, for indices in the range  jverif(1)  thru  jverif(2).
*
*     lvlVer is the verify level, which has the following meaning:
*
*     -1         do not perform any check.
*      0         do the cheap test only.
*      1 or 3    do both cheap and full test on objective gradients.
*      2 or 3    do both cheap and full test on the Jacobian.
*
*     09-Aug-92: First version based on Minos routine m7chkg.
*     23-Dec-94: Base gradient computed before entry.
*                Scaling option added.
*     19-Mar-97: sfgObj included as an argument.
*     19-Jul-97: First thread-safe version.
*     12-Aug-97: Current version of s7chkg.
*     ==================================================================
      logical            cheap, nodd
      character*4        key
      parameter         (zero = 0.0d+0, one = 1.0d+0)
      character*4        lbad        , lgood
      data               lbad/'bad?'/, lgood/'ok  '/
*     ------------------------------------------------------------------
      iPrint    = iw( 12)
      iSumm     = iw( 13)

      lvlVer    = iw( 78)
      j1        = iw( 98)       ! jverif(1)
      j2        = iw( 99)       ! jverif(2)
      nGotg1    = iw(186)       ! nGotg(1)

      if (lvlVer .lt. 0  .or.  nGotg1 .eq. 0) return

      eps0      = rw(  2)
      eps5      = rw(  7)
      fdint1    = rw( 76)
      gdummy    = rw( 69)

      j1        = max( j1, 1 )
      j2        = min( j2, nnObj )
      cheap     = lvlVer .eq. 0  .or.  lvlVer .eq. 2  .or.  j1 .gt. j2

      if (iPrint .gt. 0) then
         if ( cheap ) then
            write(iPrint, 1100)
         else
            write(iPrint, 1000)
         end if
      end if

      nState    = 0

*     --------------------------
*     Cheap test.
*     --------------------------
*     If nnObj is odd, u is set to 1/(nnObj - 1).  Otherwise, u = 1/nnObj.

      nodd = 2*(nnObj/2) .ne. nnObj
      rnnObj   = nnObj
      r    = one/rnnObj
      u    = r
      if (nodd  .and.  nnObj .ne. 1) u = one/(rnnObj - one)

*     Set arrays da and db to be (almost) orthogonal.
*     We must not perturb x(j) if g(j) is unknown.

      call dload ( nnObj, zero, da, 1 )
      call dload ( nnObj, zero, db, 1 )
      do 20, j = 1, nnObj
         if (g(j) .ne. gdummy) then
            db(j) = r
            da(j) = u
         end if
   20 continue

      do 30, j = 1, nnObj, 2
         if (da(j) .ne. zero) da(j) = - da(j)  -  j / rnnObj
   30 continue
      if (nodd) da(nnObj) = zero

*     Define a difference interval.
*     Make sure da and db are feasible directions.

      dx     = fdint1 * (one + dnrm1s( nnObj, x, 1 ))
      call s7chkd( nnObj, bl, bu, x, dx, da, nfeas1 )
      call s7chkd( nnObj, bl, bu, x, dx, db, nfeas2 )

      if (nfeas1 + nfeas2 .eq. 0) then
         if (iPrint .gt. 0) write(iPrint, 1200)
      else

*        Set w1 = g(t)*da and w2 = g(t)*db.

         w1     = ddot  ( nnObj, da, 1, g, 1 )
         w2     = ddot  ( nnObj, db, 1, g, 1 )

*        Make a forward-difference approximation to the gradient
*        along da and db.

         do 40, i = 1, nnObj
            da(i) = x(i) + dx*da(i)
   40    continue
         f1     = f
         if (nnObj .ne. 1) then
            call sfgObj( 0, ierror, nState, nnObj, 
     $                   fgObj, f1, g2, da,
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )
            if (ierror .ne. 0) go to 900
         end if 

         v1     = (f1 - f)/dx

         do 50, i = 1, nnObj
            db(i) = x(i) + dx*db(i)
   50    continue
         call sfgObj( 0, ierror, nState, nnObj, 
     $                fgObj, f2, g2, db,
     $                cu, lencu, iu, leniu, ru, lenru, 
     $                cw, lencw, iw, leniw, rw, lenrw )
         if (ierror .ne. 0) go to 900
         v2     = (f2 - f)/dx

*        c1 and c2 are the differences between approximated and
*        programmed gradient projected along da and db respectively.

         c1     = v1 - w1
         c2     = v2 - w2

*        Set an error indicator if c1 or c2 is too large.

         ifail  = 0
         if (c1*c1 .ge. dx*(w1*w1 + one)  .or.
     $       c2*c2 .ge. dx*(w2*w2 + one)) ifail = 2

         if (ifail .eq. 0) then
            if (iPrint .gt. 0) write(iPrint, 1400)
         else
            if (iPrint .gt. 0) write(iPrint, 1500)
            if (iSumm  .gt. 0) write(iSumm , 1500)
         end if
         if (iPrint .gt. 0) write(iPrint, 1600) w1, w2, v1, v2
      end if

*     ------------------------------------------------------------------
*     Check each component by differencing along
*     the coordinate directions.
*     Don't bother printing a line if it looks like an exact zero.
*     ------------------------------------------------------------------
      if (cheap) go to 900
      if (iPrint .gt. 0) write(iPrint, 2000)
      emax   = - one
      jmax   =   0
      nWrong =   0
      nGood  =   0

      do 200, j = j1, j2
         xj     = x(j)
         gj     = g(j)
         if (gj .ne. gdummy) then
            dx    = fdint1 * (one + abs(xj))
            gabs  = one + abs( gj )
            x(j)  = xj + dx
            call sfgObj( 0, ierror, nState, nnObj, 
     $                   fgObj, fforwd, g2, x, 
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )
            if (ierror .ne. 0) goto 900

            gdiff = (fforwd - f) / dx
            err   = abs( gdiff - gj ) / gabs

            if (emax .lt. err) then
               emax  = err
               jmax  = j
            end if

            key   = lgood
            if (err .gt.  eps5) key    = lbad
            if (key .eq.  lbad) nWrong = nWrong + 1
            if (key .eq. lgood) nGood  = nGood  + 1
            if (abs( gj ) + err  .gt.  eps0) then
               if (iPrint .gt. 0) 
     $            write(iPrint, 2100) j, xj, dx, gj, gdiff, key
            end if
            x(j)  = xj
         end if
  200 continue

      if (iPrint .gt. 0) then
         if (nWrong .eq. 0) then
            write(iPrint, 2500) nGood , j1, j2
         else
            write(iPrint, 2600) nWrong, j1, j2
         end if
         write(iPrint, 2700) emax, jmax
      end if

      if (emax .ge. one) then

*        Bad gradients in  funobj.

         ierror = 7
         call s1envt( 1, iw, leniw )
         if (iPrint .gt. 0) write(iPrint, 9070)
         if (iSumm  .gt. 0) write(iSumm , 9070)
      end if

*     -------------------------------------------------------------
*     Print a message if the problem functions are undefined at x.
*     -------------------------------------------------------------
  900 if (ierror .lt. 0) then
         ierror = 6
         write(iPrint, 3000)
      end if

      return

 1000 format(/// ' Verification of objective gradients',
     $   ' returned by subroutine funobj.')
 1100 format(/ ' Cheap test on funobj...')
 1200 format(/ ' XXX  Can''t find a feasible step -',
     $   ' objective gradients not verified.')
 1400 format(  ' The objective gradients seem to be OK.')
 1500 format(  ' XXX  The objective gradients seem to be incorrect.')
 1600 format(  ' Gradient projected in two directions', 1p, 2e20.11,
     $       / ' Difference approximations           ', 2e20.11)
 2000 format(// 6x, 'j', 7x, 'x(j)', 8x, 'dx(j)',
     $   11x, 'g(j)', 9x, 'Difference approxn' /)
 2100 format(i7, 1p, e16.8, e10.2, 2e18.8, 2x, a4)
 2500 format(/ i7, '  objective gradients out of', i6, '  thru', i6,
     $         '  seem to be OK.')
 2600 format(/ ' XXX  There seem to be', i6,
     $   '  incorrect objective gradients in cols', i6, '  thru', i6)
 2700 format(/ ' -->  The largest relative error was', 1p, e12.2,
     $   '   in column', i6 /)
 3000 format(// ' EXIT -- Objective function is undefined',
     $          ' during gradient checking.')
 9070 format(/ ' EXIT -- subroutine funobj appears to be',
     $         ' giving incorrect gradients')

*     end of s7chkg
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s7chkJ( ierror, n, nnCon, nnJac, neJac, nb,
     $                   sfgCon, fgCon,
     $                   ne, nka, ha, ka,
     $                   bl, bu, f, f2, g, g2,
     $                   x, y, y2, 
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      external           sfgCon, fgCon
      integer            ha(ne)
      integer            ka(nka)
      double precision   bl(nnJac), bu(nnJac)
      double precision   f(nnCon), f2(nnCon), g(neJac), g2(neJac)
      double precision   x(nnJac), y(nnJac), y2(nb)

      character*8        cu(lencu), cw(lencw)
      integer            iu(leniu), iw(leniw)
      double precision   ru(lenru), rw(lenrw)

*     ==================================================================
*     s7chkJ  verifies the Jacobian  gCon  using
*     finite differences on the constraint functions  f.
*     The various verify levels are described in  s7chkg.
*
*     15-Nov-91: First version based on Minos routine m8chkj.
*     23-Dec-94: Base gradient computed before entry.
*     19-Mar-97: sfgCon included as an argument.
*     19-Jul-97: First thread-safe version.
*     09-Nov-97: Check for nonzeros outside Jacobian structure.
*     31-Dec-97: Current version of s7chkJ.
*     ==================================================================
      logical            cheap, first
      character*4        key
      parameter         (zero = 0.0d+0, one = 1.0d+0, ok = 0.1d+0)
      character*4        lbad         , lgood
      data               lbad /'bad?'/, lgood /'ok  '/
*     ------------------------------------------------------------------
      iPrint    = iw( 12)
      iSumm     = iw( 13)

      lvlVer    = iw( 78)
      j1        = iw(100)       ! jverif(3)
      j2        = iw(101)       ! jverif(4)
      nGotg2    = iw(187)

      if (lvlVer .lt. 0  .or.  nGotg2 .eq. 0) return

      eps0      = rw(  2)
      eps5      = rw(  7)
      fdint1    = rw( 76)
      gdummy    = rw( 69)

      j1        = max( j1, 1     )
      j2        = min( j2, nnJac )
      cheap     = lvlVer .le. 1  .or.  j1 .gt. j2

      if (iPrint .gt. 0) then
         if (cheap) then
            write(iPrint, 1100)
         else
            write(iPrint, 1000)
         end if
      end if

*     --------------------------------------------
*     Cheap test.
*     --------------------------------------------
*     Generate a direction in which to perturb  x.

      yj    = one/nnJac
      do 10, j =  1, nnJac
         y(j)  =   yj
         y2(j) =   yj
         yj    = - yj * 0.99999d+0
   10 continue

*     If needed, alter y to ensure that it will be a feasible direction.
*     If this gives zero, go back to original y and forget feasibility.

      dx     = fdint1 * (one + dnrm1s( nnJac, x, 1 ))
      call s7chkd( nnJac, bl, bu, x, dx, y, nfeas )
      if (nfeas .eq. 0) call dcopy ( nnJac, y2, 1, y, 1 )

*     ------------------------------------------------------------------
*     We must not perturb x(j) if the jth column of the Jacobian
*     contains any unknown elements.
*     ------------------------------------------------------------------
      nder   = 0
      l      = 0
      do 30, j = 1, nnJac
         k1    = ka(j)
         k2    = ka(j+1) - 1

         do 20, k = k1, k2
            ir    = ha(k)
            if (ir .gt. nnCon) go to 25
            l     = l + 1
            if (g(l) .eq. gdummy) y(j) = zero
   20    continue

   25    if (y(j) .ne. zero) nder = nder + 1
   30 continue

      if (nder .eq. 0) go to 900

*     Set f2 = constraint values at a short step along  y.

      dx     = fdint1 * (one + dasum( nnJac, x, 1 ))
      do 40, j = 1, nnJac
         y2(j) = x(j) + dx*y(j)
   40 continue

      nState    = 0
      call sfgCon( 0, ierror, nState,
     $             n, nnCon, nnJac, neJac, fgCon,
     $             ne, nka, ha, ka,
     $             f2, g2, y2,
     $             cu, lencu, iu, leniu, ru, lenru, 
     $             cw, lencw, iw, leniw, rw, lenrw )
      if (ierror .ne. 0) go to 900

*     Set   y2  =  (f2 - f)/dx  -  Jacobian*y.  This should be small.
*     At the same time, find the first Jacobian element in column j1.

      do 60, i  = 1, nnCon
         y2(i)  = (f2(i) - f(i))/dx
   60 continue

      l      = 0
      lsave  = 0
      do 100, j = 1, nnJac
         yj     = y(j)
         k1     = ka(j)
         k2     = ka(j+1) - 1
         do 80, k = k1, k2
            ir    = ha(k)
            if (ir .gt. nnCon) go to 100
            l     = l + 1
            if (j .lt. j1) lsave = l
            y2(ir) = y2(ir) - g(l)*yj
   80    continue
  100 continue

      imax   = idamax( nnCon, y2, 1 )
      gmax   = (f2(imax) - f(imax))/dx
      emax   = abs( y2(imax) )/(one + abs( gmax ))
      if (emax .le. ok) then
         if (iPrint .gt. 0) write(iPrint, 1400)
      else
         if (iPrint .gt. 0) write(iPrint, 1500)
         if (isumm  .gt. 0) write(iSumm , 1500)
      end if
      if (iPrint .gt. 0) write(iPrint, 1600) emax,imax
      if (cheap) go to 900

*     ----------------------------------------------------
*     Proceed with the verification of columns j1 thru j2.
*     ----------------------------------------------------
      if (iPrint .gt. 0) write(iPrint, 2000)
      l      =   lsave
      nWrong =   0
      nGood  =   0
      emax   = - one
      jmax   =   0

      do 200, j = j1, j2

*        See if there are any known gradients in this column.

         k1     = ka(j)
         k2     = ka(j+1) - 1
         lsave  = l

         do 120, k = k1, k2
            ir     = ha(k)
            if (ir .gt. nnCon) go to 200
            l      = l + 1
            if (g(l) .ne. gdummy) go to 140
  120    continue
         go to 200

*        Found one.

  140    xj     = x(j)
         dx     = fdint1 * (one + abs( xj ))
         if (bl(j) .lt. bu(j)  .and.  xj .ge. bu(j)) dx = -dx
         x(j)   = xj + dx
         call sfgCon( 0, ierror, nState,
     $                n, nnCon, nnJac, neJac, fgCon,
     $                ne, nka, ha, ka,
     $                f2, g2, x,
     $                cu, lencu, iu, leniu, ru, lenru, 
     $                cw, lencw, iw, leniw, rw, lenrw )
         if (ierror .ne. 0) go to 900

*        Estimate the jth column of the Jacobian.
*        Check the estimate against the user-supplied values.
*        Don't bother printing a line if it looks like an exact zero.
*        Check for nonzeros that lie outside the sparse data structure,

         do 150, i  = 1, nnCon
            y2(i)  = (f2(i) - f(i))/dx
  150    continue

         l     = lsave
         first = .true.

         do 160, k = k1, k2
            ir     = ha(k)
            if (ir .gt. nnCon) go to 180
            l      = l + 1
            gi     = g(l)

            if (gi .ne. gdummy) then
               gdiff  = y2(ir)
               err    = abs( gdiff - gi )/(one + abs( gi ))

               if (emax .lt. err) then
                  emax  = err
                  imax  = ir
                  jmax  = j
               end if

               if (err .le.  eps5) then
                  key    = lgood
                  nGood  = nGood  + 1
               else
                  key    = lbad
                  nWrong = nWrong + 1
               end if
                
               if (iPrint .gt. 0  .and. (abs(gi) + err .gt. eps0)) then
                  if (first) then
                     write(iPrint, 2100) j, xj, dx, 
     $                                             l, ir, gi, gdiff, key
                     first  = .false.
                  else
                     write(iPrint, 2200)           l, ir, gi, gdiff, key
                  end if
               end if
               y2(ir) = gdummy
            end if
  160    continue

*        Check that all remaining unverified elements of y2 are zero.

         do 170, i = 1, nnCon
            if ( y2(i) .ne. gdummy) then
               err =  abs(y2(i))

               if (err .gt. eps0) then
                  nWrong = nWrong + 1
                  write(iPrint, 2300) i, y2(i), lbad
                  if (emax .lt. err) then
                     emax  = err
                     imax  = i
                     jmax  = j
                  end if
               end if
            end if
  170    continue

  180    x(j)  = xj
  200 continue

      if (iPrint .gt. 0) then
         if (nWrong .eq. 0) then
            write(iPrint, 2500) nGood , j1, j2
         else
            write(iPrint, 2600) nWrong, j1, j2
         end if
         write(iPrint, 2700) emax, imax, jmax
      end if

      if (emax .ge. one) then

*        Bad gradients in  funcon.

         ierror = 8
         call s1envt( 1, iw, leniw )
         if (iPrint .gt. 0) write(iPrint, 9080)
         if (iSumm  .gt. 0) write(iSumm , 9080)
      end if

*     -------------------------------------------------------------
*     Print a message if the problem functions are undefined at x.
*     -------------------------------------------------------------
  900 if (ierror .lt. 0) then
         ierror = 6
         write(iPrint, 3000)
      end if

      return

 1000 format(//// ' Verification of constraint gradients',
     $          ' returned by subroutine funcon.')
 1100 format(/  ' Cheap test on funcon...')
 1400 format(/  ' The Jacobian seems to be OK.')
 1500 format(/  ' XXX  The Jacobian seems to be incorrect.')
 1600 format(/  ' -->  The largest discrepancy was', 1p, e12.2,
     $          '  in constraint', i6 /)
 2000 format(// ' Column       x(j)        dx(j)', 3x,
     $   ' Element no.    Row    Jacobian value    Difference approxn')
 2100 format(/ i7, 1p, e16.8, e10.2, 2i10, 2e18.8, 2x, a4)
 2200 format(           33x, 2i10, 1pe18.8, e18.8, 2x, a4)
* 2300 format(  9x, 'Extra nonzero', 19x, '--', 4x, i6, 18x,
 2300 format(  9x, 'Nonzero not in sparse structure ', '??', 4x, i6,18x,
     $          1p, e18.8, 2x, a4 )
 2500 format(/ i7, '  Jacobian elements in cols ', i6, '  thru', i6,
     $             '  seem to be OK.')
 2600 format(/ ' XXX  There seem to be', i6,
     $    '  incorrect Jacobian elements in cols', i6, '  thru', i6)
 2700 format(/ ' -->  The largest relative error was', 1p, e12.2,
     $         '   in row', i6, ',  column', i6 /)
 3000 format(// ' EXIT -- Constraint functions are undefined',
     $          ' during gradient checking.')

 9080 format(/ ' EXIT -- subroutine funcon appears to be',
     $         ' giving incorrect gradients')

*     end of s7chkJ
      end

