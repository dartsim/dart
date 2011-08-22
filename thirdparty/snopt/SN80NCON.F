*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
*     File  sn80ncon.f
*
*     s8aux    s8aux1   s8feas   s8fixm   s8Hcur   s8Hfix   s8Infs
*     s8iqp    s8Jcpy   s8Jprd   s8log    s8mrt    s8nslk   s8rc
*     s8setJ   s8sclJ   s8sInf   s8step   s8viol   s8wInf   s8x1
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8aux ( job, nb, inform,
     $                   ne, nka, a, ha, ka,
     $                   bl, bu, xs, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character          job*(*)
      integer            ha(ne)
      integer            ka(nka)
      integer            iw(leniw)
      double precision   rw(lenrw)
      double precision   a(ne), bl(nb), bu(nb), xs(nb)

*     ==================================================================
*     s8aux   is a front-end for s8aux1.  It is called by  s8solv  and
*     s8solv  at various stages of the SQP  algorithm.
*
*     15 Nov 1991: First version based on Minos 5.4 routine m8augl.
*     12 Nov 1994: Current version.
*     ==================================================================
      neJac     = iw( 20)
      nnCon     = iw( 21)
      nnJac     = iw( 22)

      lvlDer    = iw( 71)
      lgConU    = iw(306)
      lgCon     = iw(305)
      lgCon2    = iw(307)
      lkg       = iw(312)
      nkg       = nnJac + 1

      call s8aux1( job, lvlDer, nnCon, nnJac, nb, inform,
     $             ne, nka, a, ha, ka,
     $             neJac, nkg, rw(lgCon), iw(lkg),
     $             bl, bu, rw(lgConU), rw(lgCon2), xs )

*     end of s8aux 
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8aux1( job, lvlDer, nnCon, nnJac, nb, inform,
     $                   ne, nka, a, ha, ka,
     $                   neJac, nkg, gCon, kg,
     $                   bl, bu, gConU, gCon2, xs )

      implicit           double precision (a-h,o-z)
      character          job*(*)
      integer            ha(ne)
      integer            ka(nka), kg(nkg)
      double precision   a(ne), bl(nb), bu(nb),
     $                   gCon(neJac), gConU(neJac), gCon2(neJac), xs(nb)

*     ==================================================================
*     s8aux1  does the work for  s8aux.
*
*     15 Nov 1991: First version based on Minos routine m8aug1.
*     19 Jul 1997: Current version.
*     ==================================================================
      inform = 0

      if (job(1:1) .eq. 'L') then
*        ---------------------------------------------------------------
*        Load the Jacobian from A into gCon and gCon2.
*        ---------------------------------------------------------------
         l      = 0
         do 140, j = 1, nnJac
            do 120,   k = ka(j), ka(j+1)-1
               ir       = ha(k)
               if (ir .gt. nnCon) go to 140
               l        = l + 1
               gCon (l) = a(k)
               gCon2(l) = a(k)
  120       continue
  140    continue

*        ---------------------------------------------------------------
*        If gCon is known, gConU  holds constant Jacobian elements.
*        ---------------------------------------------------------------
         if (lvlDer .ge. 2) call dcopy ( neJac, gCon, 1, gConU, 1 )

      else if (job(1:1) .eq. 'A') then
*        ---------------------------------------------------------------
*        Assemble the column pointers in kg.
*        ---------------------------------------------------------------
         l      = 0
         kg(1)  = 1
         do 240, j = 1, nnJac
            do 220,   k = ka(j), ka(j+1)-1
               ir       = ha(k)
               if (ir .gt. nnCon) go to 230
               l        = l + 1
  220       continue
  230       kg(j+1) = l + 1
  240    continue

      else if (job(1:1) .eq. 'C') then
*        ---------------------------------------------------------------
*        Check the initial xs values are feasible.
*        Do the slacks too, in case the RHS has been perturbed.
*        ---------------------------------------------------------------
         do 410, j = 1, nb
            xs(j)  = max( xs(j), bl(j) )
            xs(j)  = min( xs(j), bu(j) )
  410    continue

      else if (job(1:1) .eq. 'F') then
*        ---------------------------------------------------------------
*        Load the Jacobian and gCon2 with fake (random) elements.
*        ---------------------------------------------------------------
         iseed1 = 1547
         iseed2 = 2671
         iseed3 = 3770

         call ddrand( neJac, gCon2, 1, iseed1, iseed2, iseed3 )

         l      = 0
         do 510, j = 1, nnJac
            do 500, k = ka(j), ka(j+1)-1
               ir     = ha(k)
               if (ir .gt. nnCon) go to 510
               l      = l + 1
               a(k)   = gCon2(l)
  500       continue
  510    continue

      else if (job(1:1) .eq. 'R') then
*        ---------------------------------------------------------------
*        Restore the constant elements of the Jacobian from gConU.
*        ---------------------------------------------------------------
         if (lvlDer .ge. 2) then
            call dcopy ( neJac, gConU, 1, gCon , 1 )
            call dcopy ( neJac, gConU, 1, gCon2, 1 )
         end if
      end if

*     end of s8aux1
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8feas( ierror, m, mBS, n, nb, nnCon, nnL, nScl,
     $                   nDegen, numLC, numLIQ,
     $                   itn, itnlim, itQP, MnrPrt, tolQP, tolx,
     $                   nInf, sInf, wtInf, piNorm, rgNorm,
     $                   ne, nka, a, ha, ka,
     $                   hElast, hEstat, hfeas, hs, kBS, aScale, 
     $                   rhs, bl, bu, bltru, butru, blBS, buBS,
     $                   gBS, pi, rc, x0, xs, xBS,
     $                   iy, iy2, y, y1, y2,
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      integer            ha(ne), hfeas(mBS), hEstat(nb)
      integer            hElast(nb), hs(nb)
      integer            ka(nka), kBS(mBS)
      integer            iy(mBS), iy2(m)
      double precision   a(ne), aScale(nScl)
      double precision   bl(nb), bu(nb), bltru(nb), butru(nb)
      double precision   rc(nb), xs(nb)
      double precision   blBS(mBS), buBS(mBS), gBS(mBS), xBS(mBS)
      double precision   rhs(m), pi(m)
      double precision   x0(*)
      double precision   y(nb), y1(nb), y2(m)

      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     s8feas   finds a feasible point for a set of linear constraints.
*     A basis is assumed
*     to be specified by nS, hs(*), xs(*) and the superbasic parts of
*     kBS(*).  In particular, there must be nS values hs(j) = 2, and
*     the corresponding j's must be listed in kBS(m+1) thru kBS(m+nS).
*     The ordering in kBS matches the reduced Hessian R (if any).
*
*     11 May 1994: First version of s8feas.
*     19 Aug 1996: First minsum version.
*     05 Feb 1998: Proximal point norm changed to one-norm.
*     20 May 1998: Current version.
*     ==================================================================
      logical            Elastc, needLU, needx
      parameter         (zero = 0.0d+0, one = 1.0d+0)
      character*20       contyp
*     ------------------------------------------------------------------
      iPrint    = iw( 12)
      iSumm     = iw( 13)

      eps0      = rw(  2)
      eps1      = rw(  3)

      iObjFP    = 0
      Obja      = zero
      minimz    = 1
      lvlInf    = 2
      lEmode    = 1             !  Enter elastic mode if infeasible

      Elastc    = .false.
      needLU    = .true.
      needx     =  needLU

*     Set "rhs" to make xs satisfy the (relaxed) nonlinear rows.
*     Use a fairly tight optimality tolerance for phase 1.

      lenrhs = nnCon
      if (lenrhs .gt. 0) then
         call dcopy ( lenrhs, xs(n+1), 1, rhs, 1 )
         call s2Aprd( 'No transpose', eps0, ne, nka, a, ha, ka, 
     $                one, xs, n, (-one), rhs, lenrhs )
      end if

      tolFP     = eps1

      if (numLIQ .gt. 0  .or.  nInf .gt. 0) then
*        ---------------------------------------------------------------
*        Find a feasible point for the linear constraints.
*        If none exists, minimize the sum of infeasibilities of the
*        linear rows, subject to the column bounds.
*        ---------------------------------------------------------------
         call iload ( numLC, 3, hElast(n+nnCon+1), 1 )

         contyp = 'linear rows'

         call s5LP  ( 'FPLinear constrnts', 
     $                contyp, Elastc, needLU, needx, ierror,
     $                m, n, nb, nDegen, itnlim, itQP, itn,
     $                lEmode, lvlInf, MnrPrt,
     $                minimz, Obja, iObjFP, tolFP, tolQP, tolx,
     $                nInf, sInf, wtInf, piNorm, rgNorm, xNorm,
     $                ne, nka, a, ha, ka,
     $                hElast, hEstat, hfeas, hs, kBS, aScale, 
     $                rhs, lenrhs, bl, bu, blBS, buBS,
     $                gBS, pi, rc, xBS, xs,
     $                iy, iy2, y, y2, y1,
     $                cw, lencw, iw, leniw, rw, lenrw )

*        Stop if the linear constraints are infeasible.
*        s5LP should have minimized the sum of infeasibilities.

         if (ierror .eq. 0  .and.  nInf .gt. 0) ierror = 1
         if (ierror .gt. 0) go to 800

*        Now the linear rows are feasible, they are never allowed
*        to be infeasible again.

         call iload ( numLC, 0, hElast(n+nnCon+1), 1 )

*        Print something brief if s5LP didn't already do so.

         if (iPrint .gt. 0) write(iPrint, 1000)

         if (MnrPrt .eq. 0) then
            if (iPrint .gt. 0) write(iPrint, 8010) itn
            if (iSumm  .gt. 0) write(iSumm , 8010) itn
         end if
      end if

      if (nnL .gt. 0) then
*        ---------------------------------------------------------------
*        xs is feasible for the linear constraints.
*        Minimize the one-norm of (x-x0) by fixing the nonlinear 
*        variables so that bl = x0 = bu.  Any bl or bu that is moved to
*        x0 is allowed to be elastic.
*        ---------------------------------------------------------------
         do 100, j = 1, nnL
            blj    = bl(j)
            buj    = bu(j)

            if (blj .eq. buj) then
*              Relax
            else
               x0j       = x0(j)
               bl(j)     = x0j
               bu(j)     = x0j
               hElast(j) = 3

               if (hs(j) .le. 1) then
                  xs(j) = x0j
               end if
            end if
  100    continue

         contyp = 'norm(x-x0) problem  '
         needx  = .true.

         call s5LP  ( 'FPLinear constrnts', 
     $                contyp, Elastc, needLU, needx, ierror,
     $                m, n, nb, nDegen, itnlim, itQP, itn,
     $                lEmode, lvlInf, MnrPrt,
     $                minimz, Obja, iObjFP, tolFP, tolQP, tolx,
     $                nInf, sInf, wtInf, piNorm, rgNorm, xNorm,
     $                ne, nka, a, ha, ka,
     $                hElast, hEstat, hfeas, hs, kBS, aScale, 
     $                rhs, lenrhs, bl, bu, blBS, buBS,
     $                gBS, pi, rc, xBS, xs,
     $                iy, iy2, y, y2, y1,
     $                cw, lencw, iw, leniw, rw, lenrw )

*        Some elastic variables may have moved outside their bounds. 
*        Count them.  Reset the true bounds.

         nviol = 0
         do 200, j = 1, nnL
            bl(j) = bltru(j)
            bu(j) = butru(j)
            
            if (xs(j) .lt. bl(j)-tolx  .or.  xs(j) .gt. bu(j)+tolx) then
               nviol = nviol + 1
            end if
  200    continue

         if (ierror .gt. 0) go to 800

         if (iPrint .gt. 0) write(iPrint, 8020) itn, sInf
         if (iSumm  .gt. 0) write(iSumm , 8020) itn, sInf

         if (nviol .gt. 0) then
            contyp = 'linear rows again   '
            Elastc = .false.
            needx  = .true.

            call s5LP  ( 'FPLinear constrnts', 
     $                   contyp, Elastc, needLU, needx, ierror,
     $                   m, n, nb, nDegen, itnlim, itQP, itn,
     $                   lEmode, lvlInf, MnrPrt,
     $                   minimz, Obja, iObjFP, tolFP, tolQP, tolx,
     $                   nInf, sInf, wtInf, piNorm, rgNorm, xNorm,
     $                   ne, nka, a, ha, ka,
     $                   hElast, hEstat, hfeas, hs, kBS, aScale, 
     $                   rhs, lenrhs, bl, bu, blBS, buBS,
     $                   gBS, pi, rc, xBS, xs,
     $                   iy, iy2, y, y2, y1,
     $                   cw, lencw, iw, leniw, rw, lenrw )
            if (ierror .eq. 0) then
               if (iPrint .gt. 0) write(iPrint, 8030) itn, nviol
               if (iSumm  .gt. 0) write(iSumm , 8030) itn, nviol
            end if
         end if

         nInf = 0
         sInf = zero

*        Now the nonlinear variables are feasible, they are never 
*        allowed to be infeasible again.

         call iload ( nnL, 0, hElast, 1 )

      end if ! nnL > 0

  800 if (ierror .eq. 0 ) then
*        --------------------------------------------------
*        Relax, a feasible point for the linear constraints
*        has been found.
*        --------------------------------------------------
      else
         call s1page( 2, iw, leniw )

         if (ierror .eq.  1) then
*           -------------------------------------------
*           Infeasible.
*           -------------------------------------------
            if (iPrint .gt. 0) write(iPrint, 9010)
            if (iSumm  .gt. 0) write(iSumm , 9010)

         else if (ierror .eq.  3) then
*           -------------------------------------------
*           Too many iterations.
*           -------------------------------------------
            if (iPrint .gt. 0) write(iPrint, 9031)
            if (iSumm  .gt. 0) write(iSumm , 9031)
         end if
      end if

      return

 1000 format(' ')
 8010 format(  ' Itn', i7, ': Feasible linear rows')
 8020 format(  ' Itn', i7, ': Norm(x-x0) minimized  (', 1p, e8.2, ')')
 8030 format(  ' Itn', i7, ': ',
     $                 i7, ' nonlinear variables made feasible')

 9010 format(  ' EXIT -- the linear constraints are infeasible')
 9031 format(  ' EXIT -- iteration limit exceeded')

*     end of s8feas
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8fixm( n, nb, nnCon, hs, bl, bu, pi )

      implicit           double precision (a-h,o-z)
      integer            hs(nb)
      double precision   bl(nb), bu(nb)
      double precision   pi(nnCon)

*     ==================================================================
*     s8fixm   checks the pi's for nonlinear inequality constraints.
*     If they are active and have the wrong sign, we just use zero.
*
*     03 Nov 1993: First version of s8fixm.
*     03 Nov 1993: Current version.
*     ==================================================================
      parameter         (zero   =  0.0d+0)
      parameter         (xMulmx =  1.0d+12)

      do 100, i = 1, nnCon
         j      = n + i
         js     = hs(j)
         py     = pi(i)
         xMuli  = py
         if (bl(j) .lt. bu(j)) then
            if (js .eq. 0) xMuli = min( zero, py )
            if (js .eq. 1) xMuli = max( zero, py )
         end if
         xMuli  = min( xMuli,    xMulmx  )
         xMuli  = max( xMuli, (- xMulmx) )
         pi(i)  = xMuli
  100 continue

*     end of s8fixm
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8Hcur( minimz, m, 
     $                   nnCon, nnObj, nnObj0, nnJac, nnL, tolz,
     $                   ne, nka, ha, ka,
     $                   neJac, nkg, kg, 
     $                   curvL, gObj1, gObj2, gCon1, gCon2,
     $                   xdif, gdif, pi )

      implicit           double precision (a-h,o-z)
      integer            ha(ne)
      integer            ka(nka), kg(nkg)
      double precision   gCon1(neJac), gCon2(neJac)
      double precision   gObj1(nnObj0), gObj2(nnObj0)
      double precision   xdif(nnL), gdif(nnL)
      double precision   pi(m)

*     ==================================================================
*     s8Hcur  computes curvL, the approximate curvature of the
*     Lagrangian along the search direction  xdif.
*
*     On entry,
*     gCon1    is the Jacobian at the old x.
*     gCon2    is the Jacobian at the new x. 
*
*     On exit,
*     gdif     is the Lagrangian gradient difference.
*
*     09 Sep 1994: First version based on  Npsol  routine npupdt.
*     02 Dec 1996: Current version.
*     ==================================================================
      parameter         (zero   = 0.0d+0, one = 1.0d+0)
*     ------------------------------------------------------------------

      if (nnL .le. 0) return

*     Find gdif = sgnObj*(gObj2 - gObj1) - (J2 - J1)' pi.

      if (nnObj .gt. 0) then
         sgnObj = minimz
         do 100,  i = 1, nnObj
            gdif(i) = gObj2(i) - gObj1(i)
  100    continue

         if (minimz .lt. 0) then 
            call dscal ( nnObj, sgnObj, gdif, 1 )
         end if 
      end if

      nzero = nnL - nnObj
      if (nzero .gt. 0) call dload ( nzero, zero, gdif(nnObj+1), 1 )

      if (nnCon  .gt. 0) then
         call s8Jprd( 'Transpose', tolz, 
     $                ne, nka, ha, ka,
     $                neJac, nkg, gCon2, kg, 
     $                (-one), pi, nnCon, one, gdif, nnJac )
         call s8Jprd( 'Transpose', tolz,
     $                ne, nka, ha, ka,
     $                neJac, nkg, gCon1, kg, 
     $                  one , pi, nnCon, one, gdif, nnJac )
      end if

      curvL  = ddot ( nnL, gdif, 1, xdif, 1 )

*     end of s8Hcur
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8Hfix( nnCon, nnJac, tolz,
     $                   ne, nka, ha, ka,
     $                   neJac, nkg, kg, 
     $                   curvL, curvmn, PenUnm, step, 
     $                   fCon1, fCon2, gCon1, gCon2,
     $                   xdif, gdif, PenU, v, w )

      implicit           double precision (a-h,o-z)
      integer            ha(ne)
      integer            ka(nka), kg(nkg)
      double precision   gCon1(neJac), gCon2(neJac)
      double precision   fCon1(nnCon), fCon2(nnCon), PenU(nnCon)
      double precision   v(nnCon), w(nnCon)
      double precision   xdif(nnJac), gdif(nnJac)

*     ==================================================================
*     s8Hfix  attempts to find the a vector xPen  of minimum two-norm
*     such that there exists a BFGS update for the modified Lagrangian
*       La   = f(x) - lambda'(fCon2 - Lfcon)
*                   + 1/2 (fCon2 - LfCon)'*diag(PenU)*(fCon2 - LfCon),
*
*     where  LfCon = fCon1 + J(x1)*xdif.
* 
*     On entry,
*     xdif     is the nonlinear part of the search direction x2 - x1.
*     gdif     is the Lagrangian gradient difference.
*     gCon1    is the Jacobian at the old x.
*     gCon2    is the Jacobian at the new x. 
*     curvL    is the approximate curvature of the Lagrangian.
*     curvmn   (curvL < curvmn) is the smallest acceptable approximate
*              curvature.
*
*     On exit,
*     gdif     is the augmented Lagrangian gradient difference.
*     PenU     are the penalty parameters.
*     curvL    is unchanged unless gotPen is true, in which case
*              curvL = curvmn.
*
*     08 Dec 1991: First version based on  Npsol  routine npupdt.
*     27 Dec 1994: Current version.
*     ==================================================================
      logical            gotPen, overfl
*-->  parameter         (Penmax = 1.0d+5)
*-->  parameter         (PenMax = 1.0d+16)
      parameter         (PenMax = 1.0d+5)
      parameter         (zero   = 0.0d+0, one = 1.0d+0)
*     ------------------------------------------------------------------
*     Try an augmented Lagrangian term to increase curvL. 

      PenUnm = zero

*     Compute  v = J1*xdif and w = (J2 - J1)*xdif = J2*xdif - v.

      call s8Jprd( 'No transpose', tolz,
     $             ne, nka, ha, ka,
     $             neJac, nkg, gCon1, kg, 
     $             one, xdif, nnJac, zero, v, nnCon )
      call s8Jprd( 'No transpose', tolz,
     $             ne, nka, ha, ka,
     $             neJac, nkg, gCon2, kg, 
     $             one, xdif, nnJac, zero, w, nnCon )

      call daxpy ( nnCon, (-one), v, 1, w, 1 )

*     Compute the difference between c and its linearization.
*     v  =  c - cL = fCon2 - (fCon1 + J1*s) = fCon2 - fCon1 - J1*s.

      do 200, i = 1, nnCon
         v(i)   = fCon2(i) - fCon1(i) - step*v(i)
  200 continue

*     ---------------------------------------------------------
*     Compute the minimum-length vector of penalty parameters
*     that makes the approximate curvature equal to  curvmn.
*     ---------------------------------------------------------
*     Use w to hold the constraint on PenU.
*     Minimize           norm(PenU)  
*     subject to   ( Sum( w(i)*PenU(i) )  =   const,
*                  (           PenU(i)   .ge. 0.

      wmax = zero
      do 300, i = 1, nnCon
         wi     = w(i)*v(i)
         wmax   = max( wmax, wi )
         w(i)   = max( zero, wi )
  300 continue

      wnorm  = dnrm2 ( nnCon, w, 1 )
      diff   = curvmn - curvL
      beta   = ddiv  ( wmax*diff, wnorm**2, overfl )
      gotPen = .not. overfl  .and.  wmax .gt. zero  
     $                       .and.  beta .lt. penMax

      if ( gotPen ) then
         beta   = diff/wnorm**2

         do 310,  i = 1, nnCon
            wi      = w(i)
            Peni    = beta*wi
            v(i)    =         Peni*v(i)
            curvL   = curvL + Peni*wi
            PenU(i) = Peni
  310    continue
         curvL  = max( curvL, curvmn )
         PenUnm = dnrm2 ( nnCon, PenU, 1 )

*        Update  gdif  by the term  (J2' - J1')*v,
*        with v = diag(PenU)*(fCon2 - fCon1 - J1*s) from above.

         call s8Jprd( 'Transpose', tolz,
     $                ne, nka, ha, ka,
     $                neJac, nkg, gCon2, kg, 
     $                  one , v, nnCon, one, gdif, nnJac )
         call s8Jprd( 'Transpose', tolz,
     $                ne, nka, ha, ka,
     $                neJac, nkg, gCon1, kg, 
     $                (-one), v, nnCon, one, gdif, nnJac )
      end if ! gotPen

*     end of s8Hfix
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8Infs( Elastc, n, nb, nnCon, nnCon0, wtInf,
     $                   prInf, duInf, jprInf, jduInf,
     $                   bl, bu, rc, xs, y )

      implicit           double precision (a-h,o-z)
      logical            Elastc
      double precision   bl(nb), bu(nb), rc(nb), xs(nb)
      double precision   y(nnCon0)

*     ==================================================================
*     s8Infs computes the maximum primal and dual infeasibilities,
*     using bl, bu, rc, xs and the true nonlinear slacks yslk.
*     The linear constraints and bounds are assumed to be satisfied.
*     The primal infeasibility is therefore the maximum violation of
*     the nonlinear constraints.
*     The dual infeasibility is the maximum complementarity gap
*     for the bound constraints (with bounds assumed to be no further
*     than 1.0 from each xs(j)).
*
*     prInf, duInf   return the max primal and dual infeas.
*
*     20 Feb 1994: First version based on Minos 5.5 routine m8infs.
*     25 Oct 1996: Elastic mode added.
*     11 Jan 1997: Current version.
*     ==================================================================
      parameter        ( zero = 0.0d+0,  one = 1.0d+0 )
*     ------------------------------------------------------------------

      jprInf = 0
      prInf  = zero

      if (nnCon .gt. 0) then

*        See how much  y  violates the bounds on the nonlinear slacks.
*        prInf is the maximum violation.

         do 200, i = 1, nnCon
            j      = n + i
            slack  = y(i)
            viol   = max( zero, bl(j) - slack, slack - bu(j) )
            if (prInf .lt. viol) then
               prInf  = viol
               jprInf = j
            end if
  200    continue
      end if ! nnCon > 0

*     ------------------------------------------------------------------
*     + rc(j)  is the multiplier for lower bound constraints.
*     - rc(j)  is the multiplier for upper bound constraints.
*     duInf is the maximum complementarity gap.
*     ------------------------------------------------------------------
      jduInf = 0
      duInf  = zero
      do 500, j = 1, nb
         if (bl(j) .lt. bu(j)) then
            dj     = rc(j)
            if (dj .ne. zero) then
               if (dj .gt. zero) then
                  dj =   dj * min( xs(j) - bl(j), one )
               else
                  dj = - dj * min( bu(j) - xs(j), one )
               end if
               
               if (duInf .lt. dj) then
                  duInf   =  dj
                  jduInf  =  j
               end if
            end if ! dj nonzero
         end if
  500 continue

*     ------------------------------------------------------------------
*     Include contributions from the elastic variables.
*     ------------------------------------------------------------------
      if ( Elastc ) then
         do 510, j = n+1, n+nnCon
            dj = rc(j)
            v  = bl(j) - xs(j)
            w  = xs(j) - bu(j)

            if      (v .gt. zero) then
               dj = abs(wtInf - dj) * min( v, one )
            else if (w .gt. zero) then
               dj = abs(wtInf + dj) * min( w, one )
            else
               dj = zero
            end if
               
            if (duInf .lt. dj) then
               duInf   =  dj
               jduInf  =  j
            end if
  510    continue
      end if

*     end of s8Infs
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8iqp ( Reset, QPmsg, Elastc, gotR,
     $                   ierror, itn, lenr, m, maxS, mBS, 
     $                   n, nb, nnL, nnL0, nnCon, nnObj, nnObj0,
     $                   nS, nHx, nMinor, nDegen, MjrPrt, MnrPrt, 
     $                   minimz, ObjAdd, fObjqp, iObj,
     $                   tolFP, tolQP, tolx,
     $                   nInf, sInf, wtInf, piNorm, xNorm,
     $                   ne, nka, a, ha, ka, 
     $                   hElast, hEstat, hfeas, hs, kBS, 
     $                   bl, bu, blBS, buBS,
     $                   gObj, pi, r, rc, 
     $                   x0, xs, xBS, xdif, xsfeas,
     $                   iy, iy2, t, y, y1, y2, 
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*1        Reset
      character*2        QPmsg
      logical            Elastc, gotR

      integer            hElast(nb), hs(nb)
      integer            ha(ne), hfeas(mBS), hEstat(nb)
      integer            ka(nka), kBS(mBS)
      integer            iy(mBS), iy2(m)
      double precision   a(ne)
      double precision   blBS(mBS), buBS(mBS)
      double precision   xBS(mBS)
      double precision   bl(nb), bu(nb), rc(nb)
      double precision   gObj(nnObj0)
      double precision   r(lenr), pi(m)
      double precision   x0(*), xs(nb), xdif(nb), xsfeas(nb)
      double precision   y(nb), y1(m), y2(nb), t(nb)

      character*8        cu(lencu), cw(lencw)
      integer            iu(leniu), iw(leniw)
      double precision   ru(lenru), rw(lenrw)

*     ==================================================================
*     s8iqp   computes  xs, the solution of the QP subproblem.  
*     By construction, the problem has  n  nonlinear variables, 
*
*     The array  xdif  is used as workspace.
*     The SQP base point  x0  is not altered.
*
*     On entry, the LU factorization is assumed to be known.
*     The arrays  xBS, blBS and buBS are defined. 
*
*     On output, 
*     QPmsg is   ti or ui
*
*     30 Dec 1991: First version.
*     19 Jul 1997: Thread-safe version.
*     07 Feb 1998: Current version of s8iqp.
*     ==================================================================
      character*20       contyp
      character*2        typeLU
      logical            needLU, needx , newB  , newLU, NormIn, solved
      external           s8Hx, s8HDum
      parameter         (mtry = 1)
      parameter         (MnrHdg    = 223)
*     ------------------------------------------------------------------
      iPrint    = iw( 12)
      iSumm     = iw( 13)

      itnlim    = iw( 89)
      mMinor    = iw( 91)

*     Pointers

      laScal    = iw(274)
      lgObjQ    = iw(289)
      lgBS      = iw(273)
      lrg       = iw(277)
      lrhs      = iw(290)

      QPmsg     = '  '
      needLU    = .false.
      needx     =  needLU
      NormIn    = .not. Elastc

      typeLU    = 'BT'
      contyp    = 'QP subproblem'

      nMinor     = 0
      iw(MnrHdg) = 0
      lenx0      = nb

*     Enable a switch to Elastic mode on infeasibility.
*     In theory, this should never happen for linear constraints.

      lEmode    = 1

*     ==================================================================
*     Find a feasible point for this linearization.
*     If the constraints are linear, xs is already feasible.
*     ==================================================================
      if (nnCon .gt. 0) then
         gotR   = .false.
         mxItQP = itnlim

*        ---------------------------------------------------------------
*        Find a feasible point. 
*        If the constraints are infeasible, minimize the sum of the
*        elastic variables, subject to keeping the non-elastic variables
*        feasible.  Elastic variables can move outside their bounds.
*        ---------------------------------------------------------------
         lvlInf = 2

         call s5QP  ( 'FPSubproblem',
     $                contyp, Elastc, ierror,
     $                s8Hx, s8HDum, gotR, needLU, typeLU, needx, 
     $                lenr, lenx0, m, maxS, mBS, n, nb, nDegen,
     $                nnL, nnL0, nnObj, nnObj0, nnL, nS, nHx,
     $                mxItQP, nMinor, itn,
     $                lEmode, lvlInf, MnrPrt,
     $                minimz, ObjAdd, fObjqp, iObj, 
     $                tolFP, tolQP, tolx,
     $                nInf, sInf, wtInf, piNorm, rgNorm, xNorm,
     $                ne, nka, a, ha, ka,
     $                hElast, hEstat, hfeas, hs, kBS, rw(laScal), 
     $                rw(lrhs), nnCon, bl, bu, blBS, buBS,
     $                gObj, rw(lgObjQ), rw(lgBS),
     $                pi, r, rc, rw(lrg), 
     $                xBS, x0, xs, xdif,
     $                iy, iy2, y, y2, y1, t,
     $                cu, lencu, iu, leniu, ru, lenru, 
     $                cw, lencw, iw, leniw, rw, lenrw )

*        Check for fatal errors that have already been signalled.

         if (ierror .ge. 20) return
         if (ierror .eq.  3) go to 900

*        Set Elastc here if the phase 1 was unbounded.  This can only
*        happen if a bad basis gave an enormous search direction.
*        If phase 1 stopped before the non-elastics were feasible,  a
*        feasibility phase will be needed before solving the QP.
*        In this case,  xsfeas may not satisfy the linear constraints. 

         if (ierror .eq.  2  .and. nInf .gt. 0) Elastc = .true.  

         if (Elastc  .and.  NormIn) then 
*           ------------------------------------------------------------
*           The QP switched to elastic mode.  
*           The linearized constraints are infeasible.
*           ------------------------------------------------------------
            if (MjrPrt .ge. 1  .or.  MnrPrt .ge. 1) then
               if (iPrint .gt. 0) write(iPrint, 1100) itn, wtInf
               if (iSumm  .gt. 0) write(iSumm , 1100) itn, wtInf
            end if

            gotR   = .false.
            call s8H0  ( Reset, nnL, iw, leniw, rw, lenrw )

         else if (MnrPrt .ge. 1) then

*           No change in mode.

            if ( Elastc ) then 
               if (iPrint .gt. 0) write(iPrint, 1200) itn
               if (iSumm  .gt. 0) write(iSumm , 1200) itn
            else
               if (iPrint .gt. 0) write(iPrint, 1300) itn
               if (iSumm  .gt. 0) write(iSumm , 1300) itn
            end if
          end if
      end if ! nlnCon

*     ------------------------------------------------------------------
*     The x's and linear slacks are now feasible.
*     Save them in xsfeas for use with the BFGS update.
*
*     Solve the QP subproblem.
*     Loop back sometimes if we need a BS factorize.
*     ------------------------------------------------------------------
      call dcopy ( nb, xs, 1, xsfeas, 1 )

      nTry   = 0
      solved = .false.

*     ==================================================================
*+    while (.not. solved  .and.  ntry .le. mtry) do                    
  600 if    (.not. solved  .and.  ntry .le. mtry) then

*        ---------------------------------------------------------------
*        Solve the QP.
*        The problem has  nnL  nonlinear variables.
*        Only the first  nnL  elements of  x0  and  xdif  are used.
*        ---------------------------------------------------------------
         if ( Elastc ) then
            lvlInf = 1
         end if

         ierror = 0
         typeLU = 'BT'
         call s5QP  ( 'QPSubproblem',
     $                contyp, Elastc, ierror,
     $                s8Hx, s8HDum, gotR, needLU, typeLU, needx, 
     $                lenr, lenx0, m, maxS, mBS, n, nb, nDegen,
     $                nnL, nnL0, nnObj, nnObj0, nnL, nS, nHx,
     $                mMinor, nMinor, itn,
     $                lEmode, lvlInf, MnrPrt,
     $                minimz, ObjAdd, fObjqp, iObj,
     $                tolFP, tolQP, tolx,
     $                nInf, sInf, wtInf, piNorm, rgNorm, xNorm,
     $                ne, nka, a, ha, ka,
     $                hElast, hEstat, hfeas, hs, kBS, rw(laScal), 
     $                rw(lrhs), nnCon, bl, bu, blBS, buBS,
     $                gObj, rw(lgObjQ), rw(lgBS),
     $                pi, r, rc, rw(lrg), 
     $                xBS, x0, xs, xdif,
     $                iy, iy2, y, y2, y1, t,
     $                cu, lencu, iu, leniu, ru, lenru, 
     $                cw, lencw, iw, leniw, rw, lenrw )

         if (ierror .ge. 20) return
         if (ierror .eq.  3  .or.  ierror .ge. 12 
     $                       .or.  ierror .eq.  5) go to 900

         solved = ierror .eq. 0

         if (ntry .lt. mtry) then

            if ( solved ) then
               if (.not. Elastc) then
*                 ------------------------------------------------------
*                 If there are big nonlinear pi's, set elastic mode.
*                 ------------------------------------------------------
                  nBigpi = 0
                  do 710, i = 1, nnCon
                     if (abs(pi(i)) .gt. wtInf) then
                        nBigpi = nBigpi + 1
                     end if
  710             continue
                  
                  if (nBigpi .gt. 0) then
                     Elastc = .true.
                     solved = .false.
                     if (iPrint .gt. 0) write(iPrint, 1400) itn, wtInf
                     if (iSumm  .gt. 0) write(iSumm , 1400) itn, wtInf
                   end if
               end if
            else 
*              ---------------------------------------------------------
*              Trouble. Force a BS factorize, reset H  and start again.
*              ---------------------------------------------------------
               if (ierror .eq. 6  .or.  ierror .eq. 11) then
                  if (iPrint .gt. 0) write(iPrint, 1500) itn
                  if (iSumm  .gt. 0) write(iSumm , 1500) itn
                  call s8H0  ( Reset, nnL, iw, leniw, rw, lenrw )
               end if
               
               ierr   = ierror
               gotR   = .false.
               LUreq  = 11
               needLU = .true.
               typeLU = 'BS'
               call s2Bfac( typeLU, needLU, newLU, newB,
     $                      ierror, iObj, itn, MjrPrt, LUreq,
     $                      m, mBS, n, nb, nnL, nS, nSwap, xNorm,
     $                      ne, nka, a, ha, ka, 
     $                      kBS, hs, rw(lrhs), nnCon,
     $                      bl, bu, blBS, buBS, xBS, xs,
     $                      iy, iy2, y, y2, 
     $                      iw, leniw, rw, lenrw )
               if (ierror .gt. 0) return
               ierror = ierr
               needLU = .false.
               needx  =  needLU
            end if
         end if

         nTry = nTry + 1
         go to 600
      end if
*+    end while

*     ------------------------------------------------------------------
  900 if (nInf .gt. 0) then
         QPmsg(2:2) = 'i'       ! infeasible
      end if

      if (.not. solved) then
         if (nnCon .gt. 0) then
*           ------------------------------------------------------------
*           The subproblem may have been stopped early.
*           Check the nonlinear inequality pi's. If a slack is nonbasic  
*           and its pi has the wrong sign,  use zero.
*           ------------------------------------------------------------
            call s8fixm( n, nb, nnCon, hs, bl, bu, pi )
         end if

         if (ierror .eq. 3) then

*           Too many iterations.
*           This error is fatal.

            if (nMinor .le. mMinor) ierror = 3
            QPmsg(1:1) = 't'    ! too many iterations

         else if (ierror .eq. 5) then

*           Superbasics limit has been reached.
*           This error is fatal too.

         else

*           Press on regardless.

            if (ierror .eq. 2) then
               QPmsg(1:1) = 'u' ! unbounded subproblem

            else if (ierror .eq. 4) then
               QPmsg(1:1) = 'w' ! weak QP solution
            end if
            ierror = 0
         end if
      end if

      return

 1100 format(' Itn', i7, ': Infeasible subproblem.',
     $       ' Elastic mode started with weight = ', 1p, e8.1 )
 1200 format(' Itn', i7, ': Feasible QP non-elastics' )
 1300 format(' Itn', i7, ': Feasible QP subproblem ' )
 1400 format(' Itn', i7, ': Large multipliers.',
     $       ' Elastic mode started with weight = ', 1p, e8.1 )
 1500 format(' Itn', i7, ': Hessian reset ')

*     end of s8iqp
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8Jcpy( nnCon, nnJac, 
     $                   ne, nka, ha, ka, 
     $                   neJac1, nkg1, gCon1, kg1, 
     $                   neJac2, nkg2, gCon2, kg2 )

      implicit           double precision (a-h,o-z)
      integer            ha(ne)
      integer            ka(nka)
      integer            kg1(nkg1), kg2(nkg2)
      double precision   gCon1(neJac1), gCon2(neJac2)

*     ==================================================================
*     s8Jcpy  copies gCon1 into gCon2 when either  gCon1 or  gCon2 
*     is stored in the upper-left hand corner of A.
*
*     16 Sep 1993: First version.
*     16 Sep 1994: Current version.
*     ==================================================================

      do 200, j = 1, nnJac
         l1     = kg1(j)
         l2     = kg2(j)
         do 100,    k = ka(j), ka(j+1)-1
            ir        = ha(k)
            if (ir .gt. nnCon) go to 200
            gCon2(l2) = gCon1(l1)
            l1        = l1 + 1
            l2        = l2 + 1
  100    continue
  200 continue

*     end of s8Jcpy
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8Jprd( task, tolz, 
     $                   ne, nka, ha, ka,
     $                   neJac, nkg, gCon, kg, 
     $                   alpha, x, lenx, beta, y, leny )

      implicit           double precision (a-h,o-z)
      character          task*(*)
      integer            ha(ne)
      integer            ka(nka), kg(nkg)
      double precision   gCon(neJac)
      double precision   x(lenx), y(leny)

*     ==================================================================
*     s8Jprd computes matrix-vector products involving J and x.  The 
*     variable task specifies the operation to be performed as follows:
*       task = 'N' (normal)          y := alpha*J *x + beta*y,
*       task = 'T' (transpose)       y := alpha*J'*x + beta*y,
*     where alpha and beta are scalars, x and y are vectors and J is a
*     sparse matrix whose columns are in natural order.
*
*     14 May 1994: Current version.
*     ==================================================================
      parameter         (zero = 0.0d+0, one = 1.0d+0)
*     ------------------------------------------------------------------

      if (alpha .eq. zero  .and.  beta .eq. one)
     $   return

*     First form  y := beta*y.

      if (beta .ne. one) then
         if (beta .eq. zero) then
            do 10, i = 1, leny
               y(i)  = zero
   10       continue
         else
            do 20, i = 1, leny
               y(i)  = beta*y(i)
   20       continue
         end if
      end if

      if (alpha .eq. zero) then

*        Relax

      else if (alpha .eq. (-one)) then

         if (task(1:1) .eq. 'N') then
            do 110, j = 1, lenx
               xj     = x(j)
               if (abs( xj ) .gt. tolz) then
                  ig  = kg(j)
                  do 100, ia = ka(j), ka(j+1)-1
                     ir      = ha(ia)
                     if (ir .gt. leny) go to 110
                     y(ir)   = y(ir) - gCon(ig)*xj
                     ig      = ig + 1
  100             continue
               end if
  110       continue

         else if (task(1:1) .eq. 'T') then

            do 220, j = 1, leny
               sum    = y(j)
               ig     = kg(j)
               do 200, ia = ka(j), ka(j+1)-1
                  ir      = ha(ia)
                  if (ir .gt. lenx) go to 210
                  sum    = sum - gCon(ig)*x(ir)
                  ig     = ig + 1
  200          continue
  210          y(j) = sum
  220       continue
         end if                          

      else ! General alpha

         if (task(1:1) .eq. 'N') then
            do 310, j = 1, lenx
               alphxj = alpha*x(j)
               if (abs( alphxj ) .gt. tolz) then
                  ig  = kg(j)
                  do 300, ia = ka(j), ka(j+1)-1
                     ir      = ha(ia)
                     if (ir .gt. leny) go to 310
                     y(ir)   = y(ir) + gCon(ig)*alphxj
                     ig      = ig + 1
  300             continue
               end if
  310       continue

         else if (task(1:1) .eq. 'T') then
            do 420, j = 1, leny
               sum    = zero
               ig     = kg(j)
               do 400, ia = ka(j), ka(j+1)-1
                  ir      = ha(ia)
                  if (ir .gt. lenx) go to 410
                  sum     = sum + gCon(ig)*x(ir)
                  ig      = ig + 1
  400          continue
  410          y(j) = y(j) + alpha*sum
  420       continue
         end if ! task .eq. 'Transpose'
      end if ! general alpha

*     end of s8Jprd
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8log ( MjrMsg, KTcond, MjrPrt, minimz,  
     $                   n, nb, nnCon0, nS, 
     $                   nMajor, nMinor, nSwap,
     $                   condHz, iObj, ObjAdd, fmrt, PenNrm, step,
     $                   prInf, duInf, vimax, virel,
     $                   ne, nka, a, ha, ka,
     $                   hs, aScale, bl, bu, fCon, xMul, xs,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*6        MjrMsg
      logical            KTcond(2)
      integer            ha(ne), hs(nb)
      integer            ka(nka)
      integer            iw(leniw)
      double precision   rw(lenrw)
      double precision   a(ne), aScale(*)
      double precision   fCon(nnCon0), xMul(nnCon0)
      double precision   bl(nb), bu(nb), xs(nb)

*     ==================================================================
*     s8log   prints the major iteration log.
*
*     15 Nov 1991: First version.
*     19 Jul 1997: Thread-safe version.
*     12 Aug 1997: Current version of s8log.
*     ==================================================================
      integer            nfCon(4), nfObj(4)
      double precision   merit
      logical            Prnt1 , PrntC , Summ1
      logical            nlnCon, prtHdg, Major0, pHead
      logical            prtx  , prtl  , prtf  , prtj , scaled

      integer            sclObj
      parameter         (sclObj    = 188)
      parameter         (mLine     =  20)
      parameter         (MjrHdg    = 224)
      parameter         (MjrSum    = 225)

      character*1        flag(0:1), cflag
      character*2        key(-1:4)
*     data               flag /'#', ' '/
      data               flag /' ', ' '/
      data               key  /'fr', 'lo', 'up', 'sb', 'bs', 'fx'/
*     ------------------------------------------------------------------
      iPrint    = iw( 12)
      iSumm     = iw( 13)

      nnCon     = iw( 21)
      nnJac     = iw( 22)
      lvlScl    = iw( 75)

      lenL      = iw(173)
      lenU      = iw(174)

      nfCon(1)  = iw(189)
      nfCon(2)  = iw(190)
      nfCon(3)  = iw(191)
      nfCon(4)  = iw(192)

      nfObj(1)  = iw(194)
      nfObj(2)  = iw(195)
      nfObj(3)  = iw(196)
      nfObj(4)  = iw(197)

      plInfy    = rw( 70)

      nlnCon    = nnCon  .gt. 0
      Prnt1     = iPrint .gt. 0  .and.  MjrPrt .ge. 1
      PrntC     = iPrint .gt. 0  .and.  MjrPrt .ge. 100
      Summ1     = iSumm  .gt. 0  .and.  MjrPrt .ge. 1
      Major0    = nMajor .eq. 0  

      if (Major0  .and.  Prnt1) then
         write(iPrint, 1000)
      end if

      Mjr       = nMajor
      Mnr       = nMinor

      sgnObj    = minimz
      merit     = sgnObj*(ObjAdd + fmrt)

      if ( Prnt1 ) then
*        ------------------------------
*        Terse line for the Print file.
*        ------------------------------
         prtHdg = iw(MjrHdg) .gt. 0
         nLine  = mod( Mjr, mLine )
         pHead  = nLine  .eq. 0  .or.  prtHdg

         if (pHead) then
            cflag  = flag(min( nLine, 1 ))
            if (nlnCon) then
               write(iPrint, 2000) cflag
            else
               write(iPrint, 3000) cflag         
            end if
            iw(MjrHdg) = 0
         end if

         LU     = lenL + lenU
         if (nlnCon) then
            write(iPrint, 2100) Mjr, Mnr, step, nfObj(2), nfCon(2),
     $                          merit, prInf, duInf, nS,  
     $                          PenNrm, LU, nSwap, condHz,
     $                          KTcond, flag(0), MjrMsg
         else
            write(iPrint, 3100) Mjr, Mnr, step, nfObj(2),
     $                          merit,        duInf, nS, 
     $                                  LU, condHz,
     $                          KTcond, flag(0), MjrMsg
         end if
*         call fflush(iPrint, ierr)
      end if
      
      if ( Summ1 ) then
*        --------------------------------
*        Terse line for the Summary file.
*        --------------------------------
         prtHdg = iw(MjrSum)       .gt. 0
         pHead  = mod(Mjr, 10) .eq. 0  .or.  prtHdg
     $                                 .or.  Major0

         if (pHead) then
            iw(MjrSum) = 0
            if (nlnCon) then
               write(iSumm , 4000)
            else
               write(iSumm , 5000)               
            end if
         end if

         if (nlnCon) then
            write(iSumm , 4100) Mjr, Mnr, step, nfCon(2), 
     $                          merit, prInf, duInf, nS, PenNrm,
     $                          KTcond, MjrMsg
         else
            write(iSumm , 5100) Mjr, Mnr, step, nfObj(2), 
     $                          merit,        duInf, nS, 
     $                          KTcond, MjrMsg
         end if
      end if

      if (PrntC  .and.  nnCon .gt. 0) then
*        ---------------------------------------------------------------
*        Output heading for detailed log.
*        ---------------------------------------------------------------
         call s1page( 0, iw, leniw )
         if (Major0) write(iPrint, 1000)

*        Unscale everything if necessary.

         scaled = lvlScl .ge. 2
         if ( scaled ) then 
            call s2Scla( 'Unscale', nnCon, n, nb,
     $                   iObj, plInfy, rw(sclObj), 
     $                   ne, nka, a, ha, ka, 
     $                   aScale, bl, bu, xMul, xs )
            call ddscl ( nnCon, aScale(n+1), 1, fCon, 1 )
         end if

         l      = MjrPrt/100
         prtx   = mod( l,10 ) .gt. 0
         l      = l/10
         prtl   = mod( l,10 ) .gt. 0  .and.  Mjr .gt. 0
         l      = l/10
         prtf   = mod( l,10 ) .gt. 0
         l      = l/10
         prtj   = mod( l,10 ) .gt. 0

         if ( prtx ) write(iPrint, 7100) (xs(j)  , j=1,nnJac)
         if ( prtl ) write(iPrint, 7200) (xMul(i), i=1,nnCon)
         if ( prtf ) write(iPrint, 7300) (fCon(i), i=1,nnCon)
         if ( prtf ) write(iPrint, 7600) vimax, virel
         if ( prtj ) then
            write(iPrint, 7400)
            do 100, j = 1, nnJac
               k1     = ka(j)
               k2     = ka(j+1) - 1
               do 50, k = k1, k2
                  ir    = ha(k)
                  if (ir .gt. nnCon) go to 60
   50          continue
               k    = k2 + 1
   60          k2   = k  - 1
               l    = hs(j)
               write(iPrint, 7410) j, xs(j),key(l),(ha(k),a(k), k=k1,k2)
  100       continue
         end if
   
*        Scale again if necessary.
         
         if (scaled) then
            call s2Scla( 'Scale again', nnCon, n, nb,
     $                   iObj, plInfy, rw(sclObj),
     $                   ne, nka, a, ha, ka, 
     $                   aScale, bl, bu, xMul, xs )
            call dddiv ( nnCon, aScale(n+1), 1, fCon, 1 )
         end if
      end if
      return

 1000 format(' ')

*     Print file

 2000 format(/   ' Major Minor   Step   nObj  nCon',
     $           '      Merit      Feasibl Optimal   nS', 
     $           ' Penalty     LU Swp Cond Hz', ' PD', a1)
 2100 format(    i6, i6, 1p, e8.1, 2i6, e16.8, 2e8.1, i5, 
     $           e8.1, i7, i4, e8.1, 
     $           1x, 2l1, a1, a6 )
 3000 format(/   ' Major Minor   Step   nObj', 
     $           '     Objective   Optimal   nS', 
     $           '     LU Cond Hz', ' PD', a1)
 3100 format(    i6, i6, 1p, e8.1, i6, e16.8, e8.1, i5,
     $                   i7, e8.1, 
     $           1x, 2l1, a1, a6 )

*     Summary file

 4000 format(/   ' Major Minor   Step   nCon',
     $           '    Merit      Feasibl Optimal   nS', 
     $           ' Penalty',
     $           ' PD' )
 4100 format(    i6, i6, 1p, e8.1, i6, e14.6, 2e8.1, i5, e8.1, 
     $           1x, 2l1, 1x, a6 )

 5000 format(/   ' Major Minor   Step   nObj', 
     $           '   Objective   Optimal   nS', 
     $           ' PD' )
 5100 format(    i6, i6, 1p, e8.1, i6, e14.6, e8.1, i5,
     $           1x, 2l1, 1x, a6 )

 7100 format(/ ' Jacobian variables'
     $       / ' ------------------'   / 1p, (5e16.7))
 7200 format(/ ' Multiplier estimates'
     $       / ' --------------------' / 1p, (5e16.7))
 7300 format(/ ' Constraint functions'
     $       / ' --------------------' / 1p, (5e16.7))
 7400 format(/ ' x  and  Jacobian' / ' ----------------')
 7410 format(i6, 1p, e13.5, 1x, a2, 1x, 4(i9, e13.5)
     $       / (22x, 4(i9, e13.5)))

 7600 format(  ' Maximum constraint violation    =', 1p, e12.4,
     $     4x,  ' ( =', e11.4, ' normalized)' )

*     end of s8log 
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8mrt ( nnCon, f, gtp, pHp,
     $                   incRun, penDmp, penMax, PenNrm,
     $                   viol, xMul, xdMul, xPen, y, rw, lenrw )

      implicit           double precision (a-h,o-z)
      logical            incRun
      double precision   rw(lenrw)
      double precision   xdMul(nnCon), viol(nnCon)
      double precision   xMul(nnCon), xPen(nnCon), y(nnCon)

*     ==================================================================
*     s8mrt  computes the contributions to the merit function and its
*     directional derivative from the nonlinear constraints.
*     The penalty parameters  xPen(j)  are increased if 
*     the directional derivative is not sufficiently negative.
*
*     On entry:
*         pi     is the vector of  QP  multipliers.
*         viol   is the violation c(x) + A(linear)x - s,  where
*                s  minimizes the merit function with respect to the 
*                nonlinear slacks only.
*
*     30 Dec 1991: First version based on Npsol 4.0 routine npmrt.
*     02 Nov 1996: Multipliers no longer updated here.
*     19 Jul 1997: Thread-safe version.
*     19 Jul 1997: Current version of s8mrt.
*     ==================================================================
      logical            boost, overfl
      parameter         (zero = 0.0d+0, half = 0.5d+0, two = 2.0d+0)
*     ------------------------------------------------------------------
      eps0      = rw(  2)
      rtundf    = rw( 10)
      xPen0     = rw( 89)

*     Compute the directional derivative of the merit function.
*     If necessary, increase the penalty parameters to give descent.

      f    = f    - ddot  ( nnCon,  xMul, 1, viol, 1 )
      gtp  = gtp  + ddot  ( nnCon,  xMul, 1, viol, 1 )
      gtp  = gtp  - ddot  ( nnCon, xdMul, 1, viol, 1 )

*     Find the quantities that define  penMin, the vector of minimum
*     two-norm such that the directional derivative is one half of
*     approximate curvature   - (p)'H(p).
*     The factor  rtUndf  tends to keep  xPen  sparse.

      do 350, i = 1, nnCon
         if (abs( viol(i) ) .le. rtUndf) then
            y(i) = zero
         else
            y(i) = viol(i)**2
         end if
  350 continue

      ynorm  = dnrm2 ( nnCon, y, 1 )
      ppscl  = ddiv  ( gtp + half*pHp, ynorm, overfl )
      if (abs( ppscl ) .le. penMax  .and.  .not. overfl) then
*        ---------------------------------------------------------------
*        Bounded  penMin  found.  The final value of  xPen(i)  will
*        never be less than  penMin(i).  A trial value  penNew  is
*        computed that is equal to the geometric mean of the previous
*        xPen  and a damped value of penMin.  The new  xPen  is defined
*        as  penNew  if it is less than half the previous  xPen  and
*        greater than  penMin.
*        ---------------------------------------------------------------
         do 400, i = 1, nnCon
            penMin = max( (y(i)/ynorm)*ppscl, zero )
            xPeni  = xPen(i)

            penNew = sqrt( xPeni*(PenDmp + penMin) )
            if (penNew .lt. half*xPeni ) xPeni = penNew
            xPeni   = max (xPeni, penMin)
            xPen(i) = max (xPeni, xPen0 )
  400    continue

         PenOld  = PenNrm
         PenNrm = dnrm2( nnCon, xPen, 1 )

*        ---------------------------------------------------------------
*        If  IncRun = true,  there has been a run of iterations in
*        which the norm of  xPen  has not decreased.  Conversely,
*        IncRun = false  implies that there has been a run of
*        iterations in which the norm of xPen has not increased.  If
*        IncRun changes during this iteration the damping parameter
*        PenDmp is increased by a factor of two.  This ensures that
*        xPen(j) will oscillate only a finite number of times.
*        ---------------------------------------------------------------
         boost  = .false.
         if (      IncRun  .and.  PenNrm .lt. PenOld) boost = .true.
         if (.not. IncRun  .and.  PenNrm .gt. PenOld) boost = .true.
         if (boost) then
            PenDmp = min( 1/eps0, two*PenDmp )
            IncRun = .not. IncRun
         end if
      end if

*     ------------------------------------------------------------------
*     Compute the new value and directional derivative of the
*     merit function.
*     ------------------------------------------------------------------
      call dcopy ( nnCon, viol, 1, y, 1 )
      call ddscl ( nnCon, xPen, 1, y, 1 )

      penlty = ddot  ( nnCon, y, 1, viol, 1 )
      f      = f    + half*penlty
      gtp    = gtp  -      penlty

*     end of  s8mrt
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8nslk( n, nnCon, nnJac, tolz,
     $                   maxvi, vimax, virel, xNorm,
     $                   ne, nka, a, ha, ka,
     $                   bl, bu, fCon, xs, y )

      implicit           double precision (a-h,o-z)
      integer            ha(ne)
      integer            ka(nka)
      double precision   a(ne)
      double precision   bl(n+nnCon), bu(n+nnCon), xs(n+nnCon)
      double precision   fCon(nnCon), y(nnCon)

*     ==================================================================
*     s8nslk  finds the nonlinear constraint violations (true slacks) 
*     y   =  true nonlinear slack = fCon + A(linear)*x,
*     maxvi   points to the biggest violation in y.
*     vimax   is the biggest violation in y.
*     virel   is the biggest violation normalized by xNorm.
*
*     09 Jan 1992: First version based on Minos routine m8viol.
*     26 Dec 1992: Current version.
*     ==================================================================
      parameter         (zero = 0.0d+0, one = 1.0d+0)
*     ------------------------------------------------------------------
*     Compute the nonlinear constraint value.
*     Set  y  =  fCon + (linear A)*x,   excluding slacks.

      call dcopy ( nnCon, fCon, 1, y, 1 )

      nlin = n - nnJac
      if (nlin .gt. 0) then
         call s2Aprd( 'No transpose', tolz, 
     $                ne, nlin+1, a, ha, ka(nnJac+1), 
     $                one, xs(nnJac+1), nlin, one, y, nnCon )
      end if

*     See how much  y  violates the bounds on the nonlinear slacks.

      vimax  = zero
      maxvi  = 1

      do 100, i = 1, nnCon
         j      = n + i
         slacki = y(i)
         viol   = max( zero, bl(j) - slacki, slacki - bu(j) )
         if (vimax .lt. viol) then
             vimax =  viol
             maxvi =  i
         end if
  100 continue

      xNorm  = dnrm1s( n+nnCon, xs, 1 )
      virel  = vimax / (one + xNorm)

*     end of  s8nslk
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8rc  ( sclObj, minimz, iObj,
     $                   m, n, nb, 
     $                   nnObj, nnObj0, nnCon, nnJac, neJac,
     $                   ne, nka, a, ha, ka,
     $                   gObj, gCon, pi, rc )

      implicit           double precision (a-h,o-z)
      integer            ha(ne)
      integer            ka(nka)
      double precision   a(ne)
      double precision   gObj(nnObj0), gCon(neJac), pi(m), rc(nb)

*     ==================================================================
*     s8rc   computes reduced costs rc = gObj - ( A  -I )'*pi,
*     using  gCon  as the top left-hand corner of A.
*     gCon, gObj and pi are assumed to exist.
*
*     s8rc   is called by sncore.
*
*     28 Sep 1993: First version, derived from m4rc.
*     31 Oct 1996: Min sum option added.
*     19 Jul 1997: Current version of s8rc.
*     ==================================================================
      parameter        ( zero = 0.0d+0 )
*     ------------------------------------------------------------------
      l     = 0

      do 200, j = 1, nnJac
         dj     = zero
         do 150, k = ka(j), ka(j+1) - 1
            i      = ha(k)
            if (i .le. nnCon) then
               l   = l  + 1
               dj  = dj + pi(i) * gCon(l)
            else
               dj  = dj + pi(i) * a(k)
            end if
  150    continue
         rc(j) = - dj
  200 continue

      do 300, j = nnJac+1, n
         dj     = zero
         do 250, k = ka(j), ka(j+1) - 1
            i      = ha(k)
            dj     = dj  +  pi(i) * a(k)
  250    continue
         rc(j) = - dj
  300 continue

      call dcopy ( m, pi, 1, rc(n+1), 1 )

*     Include the nonlinear objective gradient.

      sgnObj = minimz
      if (nnObj .gt. 0) then
         call daxpy ( nnObj, sgnObj, gObj, 1, rc, 1 )
      end if

      if (iObj .gt. 0) rc(n+iObj) =  rc(n+iObj) + sgnObj*sclObj

*     end of s8rc
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8setJ( nb, nnCon, nnJac, neJac, tolz,
     $                   ne, nka, a, ha, ka, 
     $                   xs, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      integer            ha(ne)
      integer            ka(nka)
      integer            iw(leniw)
      double precision   rw(lenrw)
      double precision   a(ne)
      double precision   xs(nb)

*     ==================================================================
*     s8setJ  loads the Jacobian J(x) into  A  and computes the QP
*     right-hand side   rhs  =  J x - fCon.
*
*     29 Oct 1992: First version.
*     16 Sep 1993: Current version.
*     ==================================================================
      parameter         (one = 1.0d+0)
*     ------------------------------------------------------------------
      lkg       = iw(312)
      lfCon     = iw(302)
      lgCon     = iw(305)
      lrhs      = iw(290)

      nkg       = nnJac + 1

*     Store the new Jacobian gCon in A.

      call s8Jcpy( nnCon, nnJac, 
     $             ne, nka, ha, ka, 
     $             neJac, nkg, rw(lgCon), iw(lkg), 
     $             ne   , nka, a       , ka    )

*     Set rhs = J*x - fCon.

      call dcopy ( nnCon, rw(lfCon), 1, rw(lrhs), 1 )
      call s2Aprd( 'No transpose', tolz,
     $             ne, nka, a, ha, ka, 
     $             one, xs, nnJac, (-one), rw(lrhs), nnCon )

*     end of s8setJ
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8sclg( nnObj, aScale, gObj, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      integer            iw(leniw)
      double precision   rw(lenrw)
      double precision   aScale(nnObj), gObj(nnObj)

*     ==================================================================
*     s8sclg  scales the objective gradient.
*     s8sclg is called by sfgObj only if modefg = 2.
*     Hence, it is used to scale known gradient elements (if any),
*     but is not called when missing gradients are being estimated
*     by s6dobj.
*
*     17 Feb 1992: First version.
*     16 Jul 1997: Thread-safe version.
*     23 Jul 1997: Current version of s8sclg.
*     ==================================================================
      gdummy = rw( 69)
      nGotg1 = iw(186)

      if (nGotg1 .gt. 0) then
*        ---------------------------------------------------------------
*        Scale known objective gradients.
*        ---------------------------------------------------------------
         do 100, j = 1, nnObj
            grad   = gObj(j)
            if (grad .ne. gdummy) gObj(j) = grad*aScale(j)
  100    continue
      end if

*     end of s8sclg
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8sclJ( nnCon, nnJac, neJac, n, ne, nka, 
     $                   aScale, ha, ka, gCon,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      integer            ha(ne)
      integer            ka(nka)
      integer            iw(leniw)
      double precision   rw(lenrw)
      double precision   aScale(n+nnCon), gCon(neJac)

*     ==================================================================
*     s8sclg  scales the Jacobian.
*     nn, ng, g  are  nnObj, nnObj, gObj  or  nnJac, neJac, gCon  resp.
*
*     s8sclJ is called by sfgCon only if modefg = 2.
*     Hence, it is used to scale known gradient elements (if any),
*     but is not called when missing gradients are being estimated
*     by s6dcon.
*
*     17 Feb 1992: First version based on Minos routine m8sclj.
*     16 Jul 1997: Thread-safe version.
*     19 Jul 1997: Current version of s8sclJ.
*     ==================================================================
      gdummy = rw( 69)
      nGotg2 = iw(187)

      if (nGotg2 .gt. 0) then
*        ---------------------------------------------------------------
*        Scale known Jacobian elements.
*        ---------------------------------------------------------------
         l     = 0
         do 300, j = 1, nnJac
            cscale = aScale(j)

            do 250, k = ka(j), ka(j+1)-1
               ir     = ha(k)
               if (ir .gt. nnCon) go to 300
               l      = l + 1
               grad   = gCon(l)
               if (grad .ne. gdummy)
     $              gCon(l)   = grad*cscale/aScale(n+ir)
  250       continue
  300    continue
      end if

*     end of s8sclJ
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8sInf( n, nb, nnCon, tolx, nInf, sInf, bl, bu, xs )

      implicit           double precision (a-h,o-z)
      double precision   bl(nb), bu(nb), xs(nb)

*     ==================================================================
*     s8sInf computes the sum of infeasibilities of the nonlinear slacks
*     using bl, bu and xs.
*
*     10 Jan 1997: First version of s8sInf.
*     27 Mar 1997: Current version.
*     ==================================================================
      parameter        ( zero = 0.0d+0 )
*     ------------------------------------------------------------------
      nInf   = 0
      sInf   = zero
      tol    = tolx

*     See how much  xs(n+1:n+nnCon) violates its bounds.

      do 100, i = 1, nnCon
         j     = n + i
         slack = xs(j)
         violL = bl(j) - slack
         violU = slack - bu(j)
         if (violL .gt. tol  .or.  violU .gt. tol) then
            nInf = nInf + 1
            sInf = sInf + max (violL, violU )
         end if
  100 continue

*     end of s8sInf
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8step( nb, stepmx, stepQP, tolpiv, tolz,
     $                   bl, bu, xs, xdif, step )

      implicit           double precision (a-h,o-z)
      double precision   bl(nb), bu(nb), xs(nb), xdif(nb)

*     ==================================================================
*     s8step  finds the largest step such that the point  xs + step*xdif
*     reaches one of its bounds.
*
*     04 Dec 1992: First version of s8step based on npsol routine npalf.
*     19 Jul 1997: Current version.
*     ==================================================================
      parameter         (zero = 0.0d+0)
*     ------------------------------------------------------------------
      step = stepmx
      j    = 1

*+    while (j .le. nb  .and.  step .gt. stepQP) do
  100 if    (j .le. nb  .and.  step .gt. stepQP) then
         pivot   = xdif(j)
         pivabs  = abs( pivot )
         if (pivabs .gt. tolpiv) then
            if (pivot  .le. zero  ) then
               res    = xs(j) - bl(j)
               if (step*pivabs .gt. res) step = res / pivabs
            else 
               res    = bu(j) - xs(j)
               if (step*pivabs .gt. res) step = res / pivabs
            end if
         end if
         j = j + 1
         go to 100
*+    end while
      end if

      step   = max( step, stepQP )
      if (step .lt. stepQP + tolz) step = stepQP

*     end of s8step
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8viol( Elastc, n, nnCon, tolz, wtInf, 
     $                   bl, bu, viol, xs, xMul, xPen, yslk )

      implicit           double precision (a-h,o-z)
      logical            Elastc
      double precision   bl(n+nnCon), bu(n+nnCon), xs(n+nnCon)
      double precision   viol(nnCon)
      double precision   xMul(nnCon), xPen(nnCon), yslk(nnCon)

*     ==================================================================
*     s8viol computes the vector of nonlinear constraint violations:
*        viol = fCon + A(linear)*x - (optimal nonlinear slacks)
*
*     The optimal nonlinear slacks are computed as follows:
*     (1) Feasible  nonlinear slacks are adjusted so that they minimize
*         the merit function subject to  xs  and  xMul  being held
*         constant.
*     (2) Infeasible slacks are compared with the true nonlinear slacks,
*         and, if necessary, they are adjusted so that the sum of 
*         infeasibilities is reduced.
* 
*     If xMul is zero, the violation can be set to any value without
*     changing the merit function.  In this case we choose the slack to
*     so that  the violation is zero (subject to the constraints above).

*
*     On entry,
*        xs    =  the current xs.
*        yslk  =  fCon + A(linear)*x,   defined in s8nslk.
*
*     On exit, 
*        xs    =  xs containing the optimal slacks.
*        viol  =  fCon + A(linear)*x - optimal slacks.
*        yslk  =  unaltered.
*
*     09 Jan 1992: First version based on Npsol routine npslk.
*     09 Oct 1996: First infeasible slack version.
*     19 Jul 1997: Current version.
*     ==================================================================
      parameter         (zero   =  0.0d+0, one = 1.0d+0)
      parameter         (factor = 10.0d+0)
*     ------------------------------------------------------------------

      do 100,  i = 1, nnCon
         j     = n + i
         con   = yslk(i)
         xj    = xs(j)
         vi    = con - xj

         xPeni = xPen(i)
         xMuli = xMul(i)

         blj   = bl(j)
         buj   = bu(j)

         vU    = con - buj
         vL    = con - blj

*        ---------------------------------------------------------------
*        Redefine  xj  so that it minimizes the merit function 
*        subject to upper and lower bounds determined by the current
*        multipliers.  For computational convenience (but not clarity),
*        instead of checking that  xj  is within these bounds, the
*        violation  vi = c - xj  is checked against  vLow  and  vUpp,
*        the violations at the upper and lower bounds on xj. 
*        ---------------------------------------------------------------
*        First, impose artificial bounds (tbl, tbu).

         dvMax = factor*(one + abs( vi )) 
         vLow  = vi - dvMax
         vUpp  = vi + dvMax

         if      (Elastc  .and.  xj .le. blj) then
*           ------------------------------------------------------------
*           This slack is at or below its lower bound in elastic mode.
*           ------------------------------------------------------------
            if (     xMuli .lt. zero) then

*              xj is eligible to increase.
*              Require                  bl <=  xj <= min( bu,tbu ).

               vLow  = max( vU, vLow )
               vUpp  = vL
                 
            else if (xMuli .gt. zero) then

*              xj is eligible to decrease and violate its lower bound.
*              Require              -infty <=  xj <= bl

               xMuli = xMuli - wtInf
               vLow  = vL

            else

*              xj can either increase or decrease.
*              Require              -infty <=  xj <= min( bu,tbu ).

               vLow  = max( vU, vLow )
            end if

         else if (Elastc  .and.  xj .ge. buj) then
*           ------------------------------------------------------------
*           This slack is at or above its upper bound in elastic mode.
*           ------------------------------------------------------------
            if (     xMuli .gt. zero) then

*              xj is eligible to decrease.
*              Require      max( bl, tbl ) <=  xj <= bu.

               vLow  = vU
               vUpp  = min( vL, vUpp )
                
            else if (xMuli .lt. zero) then

*              xj is eligible to increase and violate its upper bound.
*              Require                  bu <=  xj <= +infty

               xMuli = xMuli + wtInf
               vUpp  = vU
            else

*              xj can either increase or decrease.
*              Require      max( bl, tbl ) <=  xj <= +infty

               vUpp  = min( vL, vUpp )
            end if

         else
*           ------------------------------------------------------------
*           Feasible slack.  xj can move either way.
*           ------------------------------------------------------------
*              Require      max( bl, tbl ) <=  xj <= min( bu,tbu ).

            vLow  = max( vU, vLow )
            vUpp  = min( vL, vUpp )
         end if

         if (abs( xmuli ) .le. tolz) then
            vi = min( max( zero, vLow ), vUpp )
            
         else if (xPeni .ge. tolz) then
            if (xMuli .ge. xPeni*vUpp) then
               vi = vUpp
            else if (xMuli .le. xPeni*vLow) then
               vi = vLow
            else
               vi = xMuli / xPeni
            end if
         end if

         xj      = con - vi
         viol(i) = vi
         xs(j)   = xj
  100 continue

*     end of  s8viol
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8wInf( job,
     $                   boostd, itn, gNorm, wtInf, wtInf0,
     $                   weight, wtFac, wtScal, iw, leniw )

      implicit           double precision (a-h,o-z)
      logical            boostd
      character          job*(*)
      integer            iw(leniw)

*     ==================================================================
*     s8wInf  initializes or updates the weight on the infeasibilities. 
*
*     weight, wtFac and wtScal are 'saved' local variables.
*
*     20 Feb 1997: First version written by PEG.
*     25 Feb 1997: Current version.
*     ==================================================================
      parameter         (ten   = 10.0d+0)
      parameter         (wtMax = 1.0d+10)
*     ------------------------------------------------------------------
      iPrint    = iw( 12)
      iSumm     = iw( 13)

      if (job(1:1) .eq. 'S') then

*        Set the weight.
*        weight is the ``unscaled'' weight on the infeasibilities.
*        wtScal is a scale factor based on the current gradient.

         wtScal = gNorm
         wtFac  = ten
         weight = wtInf0
         wtInf  = wtScal*weight

      else if (job(1:1) .eq. 'B') then

*        Boost the weight

         boostd = wtFac*weight .le. wtMax

         if ( boostd ) then
            weight = wtFac*weight
            wtFac  = ten*wtFac
            wtInf  = weight*wtScal
            if (iPrint .gt. 0) write(iPrint, 1000) itn, wtInf
            if (iSumm  .gt. 0) write(iSumm , 1000) itn, wtInf
         end if
      end if

      return

 1000 format(' Itn', i7, ': Elastic weight increased to ', 1p, e11.3)

*     end of s8wInf
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8x1  ( ierror, n, nb,
     $                   neJac, nnCon, nnJac, nnObj, nnObj0,
     $                   sfgCon, fgCon, sfgObj, fgObj,
     $                   ne, nka, ha, ka,
     $                   step, fCon1, gCon1, gObj1,
     $                   x1, xdif, xfeas,
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      external           sfgCon, fgCon, sfgObj, fgObj
      integer            ha(ne)
      integer            ka(nka)
      double precision   fCon1(nnCon), gCon1(neJac), gObj1(nnObj0)
      double precision   x1(nb), xdif(nb), xfeas(nb)

      character*8        cu(lencu), cw(lencw)
      integer            iu(leniu), iw(leniw)
      double precision   ru(lenru), rw(lenrw)

*     ==================================================================
*     s8x1   redefines the quantities x1 and xdif used for the
*     quasi-Newton update.  The problem functions are recomputed at x1. 
*     
*     The new  x1  is  x1 + step*(xfeas - x1),  where xfeas is the first
*     (nonelastic) feasible point from the QP subproblem.
*     
*     02 Dec 1994: First version written by PEG.
*     23 Jul 1997: Current version.
*     ==================================================================
      parameter         (one = 1.0d+0)
*     ------------------------------------------------------------------
      nState = 0
      mode   = 2

*     ------------------------------------------------
*     xdif =  xdif -      (xfeas - x1).
*     x1   =  x1   + step*(xfeas - x1).
*     ------------------------------------------------
      call daxpy ( nb, (-one), x1   , 1, xfeas, 1 )
      call daxpy ( nb,   step, xfeas, 1, x1   , 1 )
      call daxpy ( nb, (-one), xfeas, 1, xdif , 1 )

*     ------------------------------------------------------------------
*     Evaluate the functions at x1 (constraints first, then objective).
*     ------------------------------------------------------------------
      if (nnCon .gt. 0) then
         call sfgCon( mode, ierror, nState,
     $                n, nnCon, nnJac, neJac, fgCon, 
     $                ne, nka, ha, ka,
     $                fCon1, gCon1, x1,
     $                cu, lencu, iu, leniu, ru, lenru, 
     $                cw, lencw, iw, leniw, rw, lenrw )
         if (ierror .ne. 0) return
      end if

      if (nnObj .gt. 0) then
         call sfgObj( mode, ierror, nState, nnObj, fgobj,
     $                fObj1, gObj1, x1,
     $                cu, lencu, iu, leniu, ru, lenru, 
     $                cw, lencw, iw, leniw, rw, lenrw )
         if (ierror  .ne. 0) return
      end if
 
*     end of s8x1
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8Ix  ( mode, nnObj,
     $                   x, fObj, gObj, nState,
     $                   cu, lencu, iu, leniu, ru, lenru )

      implicit           double precision (a-h,o-z)
      double precision   x(nnObj), gObj(nnObj)

      character*8        cu(lencu)
      integer            iu(leniu)
      double precision   ru(lenru)

*     ==================================================================
*     funobj for feasible point calculation.
*     ==================================================================
      parameter         (half = 0.5d+0)
*     ------------------------------------------------------------------
      call dcopy ( nnObj, x, 1, gObj, 1 )
      fObj    = half*ddot ( nnObj, x, 1, x, 1 )

*     end of funobj for feasible point
      end

