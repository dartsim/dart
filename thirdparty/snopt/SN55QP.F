*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
*     file  sn55qp.f
*
*     s5Bswp   s5BSg    s5chkd   s5chzq   s5fixS   s5getd   s5Hfac
*     s5HZ     s5log    s5QP     s5QPfg   s5QPit   s5Rcol   s5rg
*     s5Rsng
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5Bswp( gotR, lenr, m, mBS, n, nb, nS,
     $                   jBSr, LUreq, pivot, 
     $                   ne, nka, a, ha, ka, 
     $                   hs, kBS, blBS, buBS, r, xBS, 
     $                   y, y2, y1,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      logical            gotR
      integer            ha(ne), hs(nb)
      integer            kBS(mBS)
      integer            ka(nka)
      integer            iw(leniw)
      double precision   rw(lenrw)
      double precision   a(ne), r(lenr)
      double precision   blBS(mBS), buBS(mBS), xBS(mBS)
      double precision   y(mBS), y2(nb), y1(m)

*     ==================================================================
*     s5Bswp is called if the last superbasic is a slack.
*     The slack is swapped with some nonslack in the basis. 
*     This preserves the property that slacks are never superbasic.
*
*     08-Nov-96: First version of s5Bswp.
*     25-Jul-97: Current version.
*     ==================================================================
      parameter         (zero      = 0.0d+0, one = 1.0d+0)
      parameter         (LUmod     = 216)
*     ------------------------------------------------------------------
      eps0      = rw(  2)
      iPrint    = iw( 12)

      nBS       = m  + nS
      jq        = kBS(nBS)
      iq        = jq - n

*     Solve  B*y = eq.
*     The altered  y1  satisfies  L*y1 = eq. 
*     It is used later to modify L and U.

      call dload ( m, zero, y1, 1 )
      y1(iq) = - one
      call s2Bsol( 'B y = yq', inform, m, y1, y, 
     $             iw, leniw, rw, lenrw )

*     Find the largest pivot.
*     Beware -- we must not choose any slacks!

      kp     = 0
      ymax   = zero
      do 550, k = 1, m
         if (kBS(k) .le. n) then
            yk  = abs( y(k) )
            if (ymax .lt. yk) then
               ymax = yk
               kp   = k
            end if
         end if
  550 continue

      if (kp .eq. 0) then       ! This can never happen!
         write (iPrint, 1100)
         stop
      end if

*     Solve  B'*y = ep.

      call dload ( m, zero, y2, 1 )
      y2(kp) = one
      call s2Bsol( 'Bt y = y2', inform, m, y2, y, 
     $             iw, leniw, rw, lenrw )

      if ( gotR ) then

*        Set yS = S'*y.

         m1     = m  + 1
         call s2Bprd( 'Transpose', eps0, n, nS, kBS(m1),
     $                ne, nka, a, ha, ka,
     $                one, y, m, zero, y(m1), nS ) 
         pivot  = y(nBS)
         y(nBS) = one + pivot
         call dscal ( nS, (-one/pivot), y(m+1), 1 )
         call s6Rswp( nS, lenr, r, y2, y(m+1), nS )
      end if

      jBSr       = kBS(kp)
      hs(jBSr)   = 2
      hs(jq)     = 3
      kBS (nBS)  = jBSr
      kBS (kp)   = jq

      tmp        = blBS(kp)
      blBS(kp)   = blBS(nBS)
      blBS(nBS)  = tmp 

      tmp        = buBS(kp)
      buBS(kp)   = buBS(nBS)
      buBS(nBS)  = tmp 

      tmp        = xBS(kp)
      xBS(kp)    = xBS(nBS)
      xBS(nBS)   = tmp 

*     Update the LU factors.

      iw(LUmod)  = iw(LUmod) + 1
      call s2Bmod( inform, kp, m, y1,
     $             iw, leniw, rw, lenrw )
      if (inform .eq. 7) then
         LUreq  =   7           ! No free memory.
      end if

      return

 1100 format(' XXX  s5Bswp:  No maximum pivot!!')

*     end of s5Bswp
      end 

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5BSg ( nBS, ngObj, ngObj0, minimz, kBS, gObj, gBS )

      implicit           double precision (a-h,o-z)
      integer            kBS(nBS)
      double precision   gObj(ngObj0), gBS(nBS)

*     ==================================================================
*     s5BSg   sets  gBS = basic and superbasic components of the 
*             scaled gObj.
*
*     On entry, gBS is zero.
*     Note that the objective row is not included here.
*
*     01-Aug-95: First version of s5BSg based on Minos routine m5bsg.
*     12-Jul-97: Current version.
*     ==================================================================

      if (minimz .gt. 0) then 
         do 20, k = 1, nBS
            j     = kBS(k)
            if (j .le. ngObj) gBS(k) =   gObj(j)
   20    continue
      else if (minimz .lt. 0) then 
         do 30, k = 1, nBS
            j     = kBS(k)
            if (j .le. ngObj) gBS(k) = - gObj(j)
   30    continue
      end if

*     end of s5BSg
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5chkd( inform, nBS, jqSave, kBS, gtd, d, iw, leniw )

      implicit           double precision (a-h,o-z)
      integer            kBS(nBS)
      integer            iw(leniw)
      double precision   d(nBS)
*     ==================================================================
*     s5chkd  makes  d  a feasible direction.
*
*     16-Jun-95: First version of s5chkd written by PEG.
*     05-Jan-96: Current version.
*     ==================================================================
      parameter         (zero = 0.0d+0, one = 1.0d+0)
*     ------------------------------------------------------------------
      iPrint    = iw( 12)
      iSumm     = iw( 13)

      inform    = 0

*     ------------------------------------------------------------------
*     Find the element of  d  corresponding to the most recently freed
*     variable. Usually, it will be d(nBS).
*     ------------------------------------------------------------------
      jq    = abs(jqSave)

      kSave = 0
      do 100, k =  nBS, 1, -1
         j = kBS(k)
         if (j .eq. jq) then
            kSave = k
            go to 110
         end if
  100 continue

*     ------------------------------------------------------------------
*     Choose the sign of  d  so that the most recently freed
*     variable continues to increase or decrease.
*     ------------------------------------------------------------------
  110 if (kSave .gt. 0) then
         dSave = d(kSave)

         if (jqSave .lt. 0  .and.  dSave .gt. zero  .or.
     $       jqSave .gt. 0  .and.  dSave .lt. zero      ) then
            call dscal ( nBS, (-one), d, 1 )
            gtd  = - gtd
         end if

         if (gtd .gt. zero) then
*           ------------------------------------------------------------
*           Looks as though the sign of gtd cannot be relied upon.
*           In later versions we'll fix this variable.
*           For now, we just print a warning and stop.
*           ------------------------------------------------------------
            if (iSumm  .gt. 0) write(iSumm , 1000) gtd
            if (iPrint .gt. 0) write(iPrint, 1000) gtd
            inform = 1
         end if
      else
*        ---------------------------------------------------------------
*        Couldn't find the index of the most recently freed variable.
*        This should never happen!
*        ---------------------------------------------------------------
         if (iSumm  .gt. 0) write(iSumm , 9000) jqSave
         if (iPrint .gt. 0) write(iPrint, 9000) jqSave
      end if

      return

 1000 format(' XXX  Small directional derivative ', 1p, e9.1 )
 9000 format(' XXX  s5chkd.  kSave not found. jqSave = ', i5 )

*     end of s5chkd
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5chzq( m, mBS, n, nb, nS, kBSq, pivot,
     $                   ne, nka, a, ha, ka,
     $                   kBS, bl, bu, xBS, y,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      integer            ha(ne)
      integer            ka(nka), kBS(mBS)
      integer            iw(leniw)
      double precision   rw(lenrw)
      double precision   a(ne), bl(nb), bu(nb)
      double precision   xBS(mBS), y(mBS)

*     ==================================================================
*     s5chzq  selects a superbasic to replace the kp-th basic variable.
*     On entry,  y  contains the kp-th row of B(inverse).
*     On exit, pivot and  y(m+1), ..., y(m+nS) define the S-part of
*     the modifying vector w.
*
*     01-Dec-91: First version of s5chzq based on Minos routine m7chzq.
*     30-Jul-95: Current version.
*     ==================================================================
      parameter        (zero = 0.0d+0, point1 = 0.1d+0, one = 1.0d+0)
*     ------------------------------------------------------------------
      eps0      = rw(  2)
      tolpiv    = rw( 60)
      iPrint    = iw( 12)

*     Set yS = 0 -  S'*y.

      m1        = m  + 1
      call s2Bprd( 'Transpose', eps0, n, nS, kBS(m1),
     $             ne, nka, a, ha, ka,
     $             (-one), y, m, zero, y(m1), nS ) 

      kBSq   = m  +  idamax( nS, y(m1), 1 )
      pivot  = abs( y(kBSq) )

*     Exit if the pivot is too small.

      if (pivot .lt. tolpiv) then
         if (iPrint .gt. 0) then
            write(iPrint, '(/ a, 1p, e11.1)')
     $         ' XXX  s5chzq.  Max pivot is too small:', pivot
         end if
         kBSq   = - (m + nS)
      else

*        Choose one away from its bounds if possible.

         tol    =   point1*pivot
         dmax   = - one

         do 200, k = m1, m+nS
            if (abs( y(k) ) .ge. tol) then
               j     = kBS(k)
               xj    = xBS(k)
               d1    = xj    - bl(j)
               d2    = bu(j) - xj
               d1    = min( abs( d1 ), abs( d2 ) )
               if (dmax .le. d1) then
                  dmax  = d1
                  kBSq  = k
               end if
            end if
  200    continue

         pivot   = - y(kBSq)

      end if ! pivot .ge. tolpiv

*     end of s5chzq
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5fixS( task,
     $                   m, maxS, mBS, n, nb, nS, hs, kBS,
     $                   bl, bu, blBS, buBS, xs, xBS )

      implicit           double precision (a-h,o-z)
      character          task*(*)
      integer            hs(nb)
      integer            kBS(mBS)

      double precision   bl(nb), bu(nb), xs(nb)
      double precision   blBS(mBS), buBS(mBS), xBS(mBS)

*     ==================================================================
*     s5fixS   concerns temporary bounds on superbasic variables.
*     If task = 'Fix ', s5fixS sets hs(j) = -1, 0, 1 or 4 for certain
*     superbasic variables.
*
*     If task = 'Free', s5fixS changes -1 values to hs(j) = 2.
*
*     30-May-95: First version of s5fixS.
*     17-Jul-95: Current version.
*     ==================================================================
      nS = 0

      if (task(1:4) .eq. 'Fix ') then
*        ---------------------------------------------------------------
*        Change superbasic hs(j) to be temporarily fixed.
*        ---------------------------------------------------------------
         do 100, j = 1, nb
            if (hs(j) .eq. 2) then
               if (bl(j) .eq. bu(j)) then
                  hs(j) =  4
               else if (xs(j) .le. bl(j)) then
                  hs(j) =  0
               else if (xs(j) .ge. bu(j)) then
                  hs(j) =  1
               else
                  hs(j) = -1
               end if
            end if
  100    continue

      else if (task(1:4) .eq. 'Free') then
*        ---------------------------------------------------------------
*        Free the temporarily fixed structurals.
*        Load the superbasic variables/bounds into xBS, blBS, buBS.
*        ---------------------------------------------------------------
         do 200, j = 1, n
            if (hs(j) .eq. -1) then
               if (nS .ge. maxS) go to 210
               nS      = nS + 1
               k       = m  + nS
               hs(j)   = 2
               xBS (k) = xs(j) 
               blBS(k) = bl(j) 
               buBS(k) = bu(j)
               kBS (k) = j
            end if
  200    continue
  210 end if

*     end of s5fixS
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5getd( ObjPhs, gotR, posdef,
     $                   lenr, n, r, g, d, gd, dHd )

      implicit           double precision (a-h,o-z)
      logical            ObjPhs, gotR, posdef
      double precision   r(lenr), g(n), d(n)

*     ==================================================================
*     s5getd  computes a search direction  d  for the superbasic
*     variables, using the current reduced gradient  g.
*
*     14-Nov-96: Current version.
*     ==================================================================
      parameter         (zero = 0.0d+0, one = 1.0d+0)
*     ------------------------------------------------------------------

      if (ObjPhs  .and.  gotR) then
*        ---------------------------------------------------------------
*        Compute either the Newton direction of a direction of zero
*        curvature.
*        ---------------------------------------------------------------
         if ( posdef ) then
            call dcopy ( n,         g, 1, d, 1 )
            call s6Rsol( 'Rt ', lenr, n, r, d )
            dHd  = ddot  ( n, d, 1, d, 1 )
            call s6Rsol( 'R ',  lenr, n, r, d )
         else
            lR1  = n*(n-1)/2
            lR   = lR1 + n
            dHd  = r(lR)**2
            call dcopy ( n-1, r(lR1+1), 1, d, 1 )
            if (n .gt. 1) then
               call s6Rsol( 'R ',  lenr, n-1, r, d )
            end if
            d(n) = - one
         end if
      else 
*        ---------------------------------------------------------------
*        Direction of steepest-descent.
*        ---------------------------------------------------------------
         call dcopy ( n, g, 1, d, 1 )
         dHd = zero
      end if ! ObjPhs and gotR

*     ------------------------------------------------------------------
*     Fix the sign of d.
*     ------------------------------------------------------------------
      call dscal ( n, (-one), d, 1 )
      gd  = ddot  ( n, g, 1, d, 1 )

*     end of s5getd
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5Hfac( pivot, ierror,
     $                   lenr, m, maxR, mBS, nb, nS, 
     $                   Hdmax, hs, kBS, perm, 
     $                   bl, bu, blBS, buBS, xs, xBS, r,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character          pivot*(*)
      integer            hs(nb)
      integer            kBS(mBS), perm(maxR)
      integer            iw(leniw)
      double precision   rw(lenrw)
      double precision   blBS(mBS), buBS(mBS), xBS(mBS)
      double precision   bl(nb), bu(nb), xs(nb)
      double precision   r(lenr)

*     ==================================================================
*     s5Hfac  factorizes the reduced Hessian Z'HZ.
*
*     13-Oct-92: First version based on Qpsol routine Qpcrsh.
*     15-Oct-94: Dependent columns fixed at their current value.
*     14-Dec-96: Current version.
*     ==================================================================
      parameter         (one   = 1.0d+0)
*     ------------------------------------------------------------------
      iPrint    = iw( 12)
      ifDefH    = iw(200)
      Hcndbd    = rw( 85)

*     ==================================================================
*     Compute the Cholesky factor of the reduced Hessian.
*     ==================================================================
      eps   = max ( Hdmax, one )/Hcndbd
      Hdmin = max ( Hdmax/Hcndbd, eps )

      call s6chol( pivot, inform, nS, 
     $             Hdmin, dmax, iRank, perm, lenr, r )

      if (pivot(1:1) .ne. 'N') then
*        -----------------------
*        Apply the interchanges.
*        -----------------------
         do 200, j = 1, min(iRank,nS)
            jmax = perm(j)
            if (jmax .gt. j) then
               kmax       = m + jmax
               k          = m + j

               ksave      = kBS(kmax)
               kBS(kmax)  = kBS(k)
               kBS(k)     = ksave

               s          = xBS(kmax)
               xBS(kmax)  = xBS(k)
               xBS(k)     = s

               s          = blBS(kmax)
               blBS(kmax) = blBS(k)
               blBS(k)    = s

               s          = buBS(kmax)
               buBS(kmax) = buBS(k)
               buBS(k)    = s
            end if
  200    continue
      end if 

      if (iRank .lt. nS) then

         if    (ifDefH .eq. 1) then

*           Relax, the Hessian is supposed to be positive definite.
*           The iterations will be terminated.

         else ! ifDefH .eq. 0
            nSsave = nS

            do 500, jS = iRank+1, nSsave
               k   = m + jS
               j   = kBS(k)

*              Make variable  j  nonbasic (it is already feasible).
*              hs(j) = -1 means xs(j) is strictly between its bounds.

               if      (xs(j) .le. bl(j)) then
                  xs(j) =  bl(j)
                  hs(j) =  0
               else if (xs(j) .ge. bu(j)) then
                  xs(j) =  bu(j)
                  hs(j) =  1
               else
                  hs(j) = -1
               end if
               if (bl(j) .eq. bu(j)) hs(j) = 4

               nS = nS - 1
  500       continue
            nS = min( nS, iRank )
         end if ! iDefH = 0
      end if ! iRank lt nS

      if (inform .gt. 0) then
         if (iPrint .gt. 0) write(iPrint, 9000) iRank, dmax, Hdmin
         if (ifDefH .eq. 1  .or.  dmax .lt. (-Hdmin)) then
            ierror = 6
            return
         end if
      end if

 9000 format(  ' ==>  Factorize detected a singular Hessian.',
     $         '  Rank = ', i6, '.  Diag, min diag = ', 1p, 2e14.2 )

*     end of s5Hfac
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5HZ  ( Hx, Hx1, lenr, minimz,
     $                   m, mBS, n, nb, nnH, nS, nHx, 
     $                   ne, nka, a, ha, ka,
     $                   Hdmax, kBS, 
     $                   r, v, w, y, 
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      external           Hx, Hx1
      integer            ha(ne)
      integer            ka(nka), kBS(mBS)
      double precision   a(ne)
      double precision   v(nb), w(nb)
      double precision   r(lenr), y(m)

      character*8        cu(lencu), cw(lencw)
      integer            iu(leniu), iw(leniw)
      double precision   ru(lenru), rw(lenrw)

*     ==================================================================
*     s5HZ    computes the reduced Hessian and loads it by columns into
*     the upper triangle r.
*
*     13-Oct-92: First version based on Qpsol routine Qpcrsh.
*     15-Oct-94: Dependent columns fixed at their current value.
*     30-Jul-95: Current version.
*     ==================================================================
      parameter         (zero  = 0.0d+0, half   =   0.5d+0)
      parameter         (one   = 1.0d+0                   )
*     ------------------------------------------------------------------
      eps0      = rw(  2)

      if (nS .eq. 0) return

      nState    = 0
      nBS       = m + nS
      Hdmax     = zero
      sgnObj    = minimz

      l         = 1
*     ------------------------------------------------------------------
*     Main loop to find a column of Z'HZ.
*     ------------------------------------------------------------------
      do 200, jS = 1, nS
*        ---------------------------------------------------------------
*        Get the nonlinear elements of the column of Z. Store them in w.
*        ---------------------------------------------------------------
*        Find y such that B y = column jq.
*        Expand the nonlinear part of y into w.

         jq  = kBS(m+jS)
         call s2unpk( jq, m, n, ne, nka, a, ha, ka, w )
         call s2Bsol( 'B y = w', inform, m, w, y, 
     $                iw, leniw, rw, lenrw )
         call dload ( nnH, zero, w, 1 )

         do 110, k = 1, m
            j      = kBS(k)
            if (j .le. nnH) w(j) = - y(k)
  110    continue
         if (jq .le. nnH) w(jq) = one

*        Multiply the column  w  by  H.

         call Hx    ( Hx1, minimz, n, nnH, ne, nka, a, ha, ka, 
     $                w, v, nState,
     $                cu, lencu, iu, leniu, ru, lenru, 
     $                cw, lencw, iw, leniw, rw, lenrw )
         nHx = nHx + 1
         if (minimz .lt. 0) call dscal ( nnH, sgnObj, v, 1 )

*        ------------------------------------------------------------
*        Expand v ( = Hx) into w.
*        Compute  v = Z' w.
*        ------------------------------------------------------------
*        Set  w = vBS.

         do 130, k = 1, nBS
            j      = kBS(k)
            if (j .le. nnH) then
               w(k) = v(j)
            else
               w(k) = zero
            end if
  130    continue

*        Solve  B' vB = wB  and  form  wS = wS - S' vB.

         call s2Bsol( 'Bt vB = wB', inform, m, w, v,
     $                iw, leniw, rw, lenrw )
         call dcopy ( m, v, 1, w, 1 )

         call s2Bprd( 'Transpose', eps0, n, nS, kBS(m+1),
     $                ne, nka, a, ha, ka, 
     $                (-one), w, m, one, w(m+1), nS ) 

*        ------------------------------------------------------
*        Store w in the jS-th row and column of r. 
*        The column elements are symmetrized. 
*        ------------------------------------------------------
         do 140, i = 1, jS-1
            k    = m + i
            r(l) = half*(r(l) + w(k))
            l    = l + 1
  140    continue

         jc = l 
         do 150, j = jS, nS
            k      = m + j
            r(jc)  = w(k)
            jc     = jc + j
  150    continue
         Hdmax = max( Hdmax, abs(r(l)) )
         l     = l + 1
  200 continue ! for jS = 1, nS

*     end of s5HZ
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5log ( prbtyp, contyp, 
     $                   Elastc, gotR, prtFea, jstFea, ObjPhs,
     $                   m, mBS, nS,
     $                   jSq, jBr, jSr, jBSr,
     $                   lines1, lines2,
     $                   itn, kPrc, lvlInf, lPrint, 
     $                   sgnObj, pivot, step, nInf, sInf, wtInf, 
     $                   Obj, condHz, djqPrt, rgNorm, kBS, xBS,
     $                   iw, leniw )

      implicit           double precision (a-h,o-z)
      character          prbtyp*(*)
      character*20       contyp
      logical            Elastc, gotR, jstFea, prtFea, ObjPhs
      integer            kBS(mBS)
      integer            iw(leniw)
      double precision   xBS(mBS)
 
*     ==================================================================
*     s5log  prints the LP/QP iteration log.
*     Normally the only parameters used are nnH, nS and f.
*     The others are there to allow monitoring of various items for
*     experimental purposes.
*     mBS = m + maxS  .ge.  m + nS.
*
*     The print controls are as follows:
*
*     prnt0 is true if lPrint = 0.
*     For LP and QP, a brief log is output every  k  minor iterations,
*     where  k  is the log frequency.
*     For NLP, a brief log is output by s8log every major iteration.
*
*     prnt1 is true if lPrint > 0.
*     A fuller log is output every  k  minor iterations.
*
*     summ0 and summ1 are the same as prnt0 and prnt1, but are false
*     if there is no summary file.  summary frequency defines  k.
*
*     MnrHdg is  1  if a new heading is required for some reason other
*     than frequency (e.g., after a basis factorization).
*
*     The output consists of a number of ``sections'' of one line
*     summaries, with each section preceded by a header message.
*     lines1 and lines2 count the number of lines remaining to be
*     printed in each section of the 
*     print and summary files respectively.
*     They too may force a new heading.
*
*     01-Dec-91: First version based on Minos routine m5log.
*     15-Nov-97: Current version.
*     ==================================================================
      logical            prtHdr, newSet, pHead , pLine
      logical            prnt1 , summ1
      logical            long  , PrntOK
      parameter         (zero   = 0.0d+0)
      parameter         (mLine1 = 40, mLine2 = 10)

      parameter         (MnrHdg    = 223)
      parameter         (MjrHdg    = 224)
      parameter         (MjrSum    = 225)
*     ------------------------------------------------------------------
      iPrint    = iw( 12)
      iSumm     = iw( 13)
      klog      = iw( 63)
      kSumm     = iw( 64)
      lprDbg    = iw( 80)
      ncp       = iw(176)

      lenL      = iw(173)
      lenU      = iw(174)
      LUitn     = iw(215)

      long      = nS         .gt. 0  .or.   prbtyp(1:2) .eq. 'QP'
      prtHdr    = iw(MnrHdg) .gt. 0
      Prnt1     = iPrint     .gt. 0  .and.  lPrint .ge. 1  
      Summ1     = iSumm      .gt. 0  .and.  lPrint .ge. 1  

      it        = mod( itn, 100000 )

      if ( Prnt1 ) then
*        --------------------------------------
*        Terse line for the Print file.
*        --------------------------------------
         newset = lines1            .eq. 0
         pLine  = mod( itn, klog  ) .eq. 0
         pHead  = pLine   .and.  (prtHdr  .or.  newSet)

         if ( ObjPhs ) then
            if (nInf .eq. 0) then
               sumObj = Obj
            else
               sumObj = Obj + sgnObj*wtInf*sInf
            end if
         else
            sumObj = sInf
         end if

         if ( pHead ) then
            iw(MnrHdg) = 0
            lines1 = mline1
            if ( long ) then
               if ( gotR ) then
                  write(iPrint, 1010)
               else
                  write(iPrint, 1011)
               end if
            else
               write(iPrint, 1000)
            end if
         end if

         if ( pline ) then
            iw(MjrHdg) = 1
            lines1 = lines1 - 1

            if ( long ) then 
               if ( gotR ) then
                  write(iPrint, 1200) it, kPrc, djqPrt, 
     $                                jSq, jSr, jBr, jBSr, step, pivot, 
     $                                nInf, sumObj, lenL, lenU, ncp,
     $                                rgnorm, nS, condHz
               else
                  write(iPrint, 1200) it, kPrc, djqPrt, 
     $                                jSq, jSr, jBr, jBSr, step, pivot, 
     $                                nInf, sumObj, lenL, lenU, ncp,
     $                                rgnorm, nS
               end if
            else
               write(iPrint, 1200) it, kPrc, djqPrt, 
     $                             jSq, jSr, jBr, jBSr, step, pivot,
     $                             nInf, sumObj, lenL, lenU, ncp
            end if
         end if
      end if

      if ( Summ1 ) then
*        --------------------------------
*        Terse line for the Summary file.
*        --------------------------------
         newset = lines2            .eq. 0
         pLine  = mod( itn, kSumm ) .eq. 0
         pHead  = pLine   .and.  newSet

         if ( ObjPhs ) then
            if (nInf .eq. 0) then
               sumObj = Obj
            else
               sumObj = Obj + sgnObj*wtInf*sInf
            end if
         else
            sumObj = zero
         end if

         if ( pHead ) then
            lines2 = mline2
            if ( long ) then
               if (lvlInf .ne. 1) then
                  write(iSumm , 2010)
               else
                  write(iSumm , 2011)
               end if
            else
               if (lvlInf .ne. 1) then
                  write(iSumm , 2000)
               else
                  write(iSumm , 2001)
               end if
            end if
         end if

         if ( pLine ) then
            iw(MjrSum) = 1
            lines2 = lines2 - 1
            if ( long ) then
               write(iSumm , 2200) it, djqPrt, step, nInf, 
     $                             sInf, sumObj, rgnorm,  nS
            else
               write(iSumm , 2200) it, djqPrt, step, nInf, 
     $                             sInf, sumObj
            end if
         end if
      end if

*     ------------------------------------------------------------------
*     If we are newly feasible, print something.
*     Suppress printing when solving one of a sequence of QP problems.
*     ------------------------------------------------------------------
      if ( jstFea ) then
         PrntOK = lPrint .ge. 1  .and. (LUitn  .gt. 0  .or.   prtFea)
     $                           .and.  prbtyp(1:3) .ne. 'QPS'
     $                           .and.  prbtyp(1:3) .ne. 'FPS'

*        contyp = 'QP problem'
*        contyp = 'LP problem'
*        contyp = 'QP subproblem'

         if ( PrntOK ) then
            if (.not. Elastc) then

*              All constraints are feasible in Normal mode.
*              Print a message if it is not done elsewhere.

               if (lPrint .ge. 1  .and.  prbtyp(1:3) .ne. 'FPE') then
                  if (iPrint .gt. 0) write(iPrint, 8010) itn, contyp
                  if (iSumm  .gt. 0) write(iSumm , 8010) itn, contyp
               end if
            else

*              Elastic mode (the constraints are infeasible).
*              Elastic Phase 1 has completed.

               if (lvlInf .eq. 2) then

*                 Infinite weight on the infeasibilities.
*                 Proceed to minimize the infeasible elastics.

                  if (lPrint .ge. 1) then
                     if (iPrint .gt. 0) write(iPrint, 8030) itn
                     if (iSumm  .gt. 0) write(iSumm , 8030) itn
                  end if

               else if (lvlInf .eq. 1) then

*                 Finite nonzero weight on the infeasibilities.
*                 Proceed to minimize a weighted objective.

                  if (lPrint .ge. 1) then
                     if (iPrint .gt. 0) write(iPrint, 8040) itn
                     if (iSumm  .gt. 0) write(iSumm , 8040) itn
                  end if
               end if
            end if
         end if
      end if

*     ------------------------------------------------------------------
*     Debug output.
*     ------------------------------------------------------------------
      if (lprDbg .eq. 100) then
         nBS = m + nS
         write(iPrint, 5000) (kBS(k), xBS(k), k = 1, nBS)
      end if

      return

 1000 format(/ '   Itn', ' pp', '       dj', 
     $         '  +SBS  -SBS   -BS    -B',
     $         '    Step', '    Pivot', ' nInf',
     $         '  sInf,Objective', '     L     U ncp')
 1010 format(/ '   Itn', ' pp', '       dj',
     $         '  +SBS  -SBS   -BS    -B',
     $         '    Step', '    Pivot', ' nInf',
     $         '  sInf,Objective', '     L     U ncp',
     $         ' Norm rg   nS cond Hz'  )
 1011 format(/ '   Itn', ' pp', '       dj',
     $         '  +SBS  -SBS   -BS    -B',
     $         '    Step', '    Pivot', ' nInf',
     $         '  sInf,Objective', '     L     U ncp',
     $         ' Norm rg   nS'  )
 1200 format(1p, i6, i3, e9.1, 
     $           4i6, e8.1, e9.1, 
     $           i5, e16.8, 2i6, i4,
     $           e8.1, i5, e8.1 )

 2000 format(/ '   Itn', '       dj', '    Step', ' nInf', 
     $         '    SumInf', '       Objective' ) 
 2001 format(/ '   Itn', '       dj', '    Step', ' nInf', 
     $         '    SumInf', '   Composite Obj' ) 
 2010 format(/ '   Itn', '       dj', '    Step', ' nInf', 
     $         '    SumInf', '       Objective', ' Norm rg',
     $         '   nS' )
 2011 format(/ '   Itn', '       dj', '    Step', ' nInf', 
     $         '    SumInf', '   Composite Obj', ' Norm rg',
     $         '   nS' )
 2200 format(1p, i6, e9.1, e8.1, i5, e10.3, e16.8, e8.1, i5)

 5000 format(/ ' BS and SB values...' / (5(i7, g17.8)))

 8010 format(  ' Itn', i7, ': Feasible ', a)
 8030 format(  ' Itn', i7, ': Elastic Phase 2 -- minimizing',
     $                     ' elastic variables')
 8040 format(  ' Itn', i7, ': Elastic Phase 2 -- minimizing',
     $                     ' obj + weighted elastics')

*     end of s5log
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5QP  ( prbtyp,
     $                   contyp, Elastc, ierror,
     $                   Hx, Hx1, gotR, needLU, typeLU, needx,
     $                   lenr, lenx0, m, maxS, mBS, n, nb, nDegen,
     $                   ngQP, ngQP0, ngObj, ngObj0, nnH, nS, nHx,
     $                   itMax, itQP, itn, 
     $                   lEmode, lvlInf, lPrint, 
     $                   minimz, ObjAdd, ObjQP, iObj, 
     $                   tolFP, tolQP, tolx,
     $                   nInf, sInf, wtInf, piNorm, rgNorm, xNorm,
     $                   ne, nka, a, ha, ka,
     $                   hElast, hEstat, hfeas, hs, kBS, aScale, 
     $                   b, lenb, bl, bu, blBS, buBS,
     $                   gObj, gObjQP, gBS, 
     $                   pi, r, rc, rg, 
     $                   xBS, x0, xs, xdif,
     $                   iy, iy2, y, y2, y1, w,
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      logical            Elastc, gotR, needLU, needx
      character          prbtyp*(*)
      character*2        typeLU
      character*20       contyp
      external           Hx, Hx1
      integer            hElast(nb), hEstat(nb), hs(nb)
      integer            ha(ne), hfeas(mBS)
      integer            ka(nka), kBS(mBS)
      integer            iy(mBS), iy2(m)
      double precision   a(ne), aScale(*), b(*)
      double precision   bl(nb), bu(nb), rc(nb), xs(nb)
      double precision   blBS(mBS), buBS(mBS), gBS(mBS), xBS(mBS)
      double precision   pi(m), y1(m)
      double precision   gObj(ngObj0), gObjQP(ngQP0)
      double precision   x0(*), xdif(ngQP0)
      double precision   r(lenr), rg(maxS)
      double precision   y(nb), y2(nb), w(nb)

      character*8        cu(lencu), cw(lencw)
      integer            iu(leniu), iw(leniw)
      double precision   ru(lenru), rw(lenrw)

*     ==================================================================
*     s5QP   solves a linear or quadratic program.
*     The problem type can be:
*       prbtyp = 'FP '  feasible point only
*       prbtyp = 'FPE'  feasible point for equalities only
*       prbtyp = 'FPS'  feasible point for QP subproblem
*       prbtyp = 'LP '  LP problem
*       prbtyp = 'QP '  QP problem
*       prbtyp = 'QPP'  FP subproblem with proximal point objective
*       prbtyp = 'QPS'  QP subproblem
*
*
*     The optimization can pass through the following phases:
*
*       Phase 1               find a feasible point for all variables
*
*       Elastic Phase 1       make the non-elastic variables feasible
*                             while allowing infeasible elastics
*
*       Phase 2               minimize the objective
*
*       Elastic Phase 2       minimize a composite objective while
*                             keeping the non-elastics feasible
*
*                             In this phase, lvlInf means the following:
*
*                 lvlInf = 0  zero     weight on the infeasibilities
*                                      (infeasibillities are ignored)
*                          1  finite   weight on the infeasibilities
*                          2  infinite weight on the infeasibilities
*                                      (the objective is ignored)
*
*     The array kBS is a permutation on the column indices. 
*     kBS(1  :m )    holds the col. indices of the basic variables.
*     kBS(m+1:m+nS)  holds the col. indices of the superbasic variables.
*                    These nS columns indices must have hs(j) = 2.
*
*     On exit:
*        ierror       Status
*        ------       ------
*           0         QP solution found
*           1         QP is infeasible
*           2         QP is unbounded
*           3         Too many iterations
*           4         Weak QP minimizer 
*           5         Too many superbasics
*           6         QP Hessian not positive semidefinite
*
*     30-Sep-91: First version of s5qp  based on Qpsol routine qpcore.
*     29-Oct-93: QP objective computed separately.
*     19-May-95: Bordered Hessian updated.
*     30-Jul-95: Border updates removed.
*     04-Jan-96: Positive semi-definite H treated correctly.
*     20-Jul-96: Slacks changed to be the row value.
*     09-Aug-96: First Min Sum version.
*     15-Jul-97: Thread-safe version.
*     02-Feb-98: Piecewise linear line search added.
*     11-Feb-98: Current version of s5QP.
*     ==================================================================
      logical            FP, LP, QP
      logical            bndswp, checkx, deadpt, nochng, optiml, statpt
      logical            feasbl, giveup, gotE  , gotg  , gotH  , incres
      logical            needLM, needpi, newSB
      logical            chkFea, jstFea, prtFea, rgfail
      logical            newB  , newLU , ObjPhs, useg 
      logical            QPdone
      logical            posdef
      logical            Prnt1 , Prnt10
      integer            nfix(2)
      parameter         (zero      = 0.0d+0)
      parameter         (mUncon    = 1)

      integer            toldj1, toldj2, toldj3
      parameter         (kObj      = 220)
      parameter         (lenL0     = 171)
      parameter         (lenU0     = 172)
      parameter         (lenL      = 173)
      parameter         (lenU      = 174)
      parameter         (toldj1    = 184)
      parameter         (toldj2    = 185)
      parameter         (toldj3    = 186)
      parameter         (LUitn     = 215)
      parameter         (LUmod     = 216)
      parameter         (MnrHdg    = 223)
*     ------------------------------------------------------------------
      eps0      = rw(  2)
      plInfy    = rw( 70)
      sclObj    = rw(188)
                         
      iPrint    = iw( 12)
      iSumm     = iw( 13)
      maxR      = iw( 56)
      kchk      = iw( 60)
      kfac      = iw( 61)
      ksav      = iw( 62)
      kDegen    = iw( 65)
      itnlim    = iw( 89)
      nFac      = iw(210)

      if (nFac .gt. 0) then
         LUsiz0    = iw(lenL0) + iw(lenU0)
         LUmax     = 2*LUsiz0
      end if

      FP     = prbtyp(1:2) .eq. 'FP'
      LP     = prbtyp(1:2) .eq. 'LP'  .or.  nnH .eq. 0
      QP     = prbtyp(1:2) .eq. 'QP'

      gotg   = ngQP .gt. 0
      gotH   = nnH  .gt. 0 

      Prnt1  = iPrint .gt. 0  .and.  lPrint .ge.  1
      Prnt10 = iPrint .gt. 0  .and.  lPrint .ge. 10

*     ------------------------------------------------------------------
*     s5QP operates in either ``Normal'' or ``Elastic'' mode.
*     Everything is normal unless a weighted sum is being minimized or
*     the constraints are infeasible.
*     The logical feasbl refers to the non-elastic variables.
*     Note that feasbl can be false while in elastic mode. 
*     wtInf  is the optional parameter Infeasibility Weight.
*     ------------------------------------------------------------------
      kPrc       = 0            ! last sec scanned in part. prc
      ierror     = 0
      LUreq      = 0
      lines1     = 0
      lines2     = 0
      iw(MnrHdg) = 0

      chkFea = .true.
      jstFea = .false.
      prtFea = .true.
      feasbl = .false.
      gotE   = .false.

      QPdone = .false.

      bndswp = .false.
      needpi = .true.
      newLU  = .true.
      nochng = .false.
      ObjPhs = .false.
      posdef = .false.

      pivot  = zero
      step   = zero
      ObjQP  = zero
      ObjLP  = zero
      Obj    = zero
      nInfE  = 0
      jq     = 0
      djq    = zero
      jBq    = 0
      jBr    = 0
      jBSr   = 0
      jSq    = 0
      jSr    = 0
      jqSave = 0
      kPrPrt = 0
      sgnObj = minimz

      rw(toldj1) = 100.0d+0
      rw(toldj2) = 100.0d+0

*     nUncon  counts the number of unconstrained (i.e., Newton) steps.
*             If the test for a minimizer were scale-independent, 
*             Uncon would never be larger than 1.

      nState = 1
      nUncon = 0 

      call s5hs  ( 'Internal', nb, bl, bu, hs, xs )
      call iload ( nb, 0, hEstat, 1 )
      call s5dgen( 'Initialize', inform, lPrint,
     $             nb, nInf, itn,
     $             featol, tolx, tolinc, hs, bl, bu, xs,
     $             itnfix, nfix, tolx0,
     $             iw, leniw, rw, lenrw )

**    ======================Start of main loop==========================
*+    do while (.not. QPdone  .and.  ierror .eq. 0)
  100 if       (.not. QPdone  .and.  ierror .eq. 0) then
*        ===============================================================
*        Check the initial  xs  and move it onto  ( A  -I )*xs = b.
*        If needLU is true, this will require a basis factorization.
*        ===============================================================
*        If necessary,  factorize the basis  ( B = LU ) and set xs.
*        If needLU is false on entry to s5QP, the first call to s2Bfac
*        will try to use existing factors.
*        If needLU is true on entry to s5QP, an LU factorization of 
*        type typeLU is computed.

         if (needx  .or.  needLU) then
            call s2Bfac( typeLU, needLU, newLU, newB, 
     $                   ierror, iObj, itn, lPrint, LUreq,
     $                   m, mBS, n, nb, nnH, nS, nSwap, xNorm,
     $                   ne, nka, a, ha, ka, 
     $                   kBS, hs, b, lenb,
     $                   bl, bu, blBS, buBS, xBS, xs,
     $                   iy, iy2, y, y2, 
     $                   iw, leniw, rw, lenrw )
            LUsiz0 = iw(lenL0) + iw(lenU0)
            LUmax  = 2*LUsiz0

            if (ierror .ne. 0) go to 100

            if ( newLU )
     $      gotR   = .false.    ! Reset R.
            gotE   = .false.    ! Reset hEstat in elastic mode.
            needpi = .true.     ! Recalculate the pi's.
            needx  = .false.
            chkFea = .true.

            jqSave = 0
            nUncon = 0
            if (lPrint .ge. 10) iw(MnrHdg) = 1
         end if

         nBS    = m + nS
         newSB  = .false.

*        If the last superbasic is a slack, swap it into the basis.

         if (nS .gt. 0  .and.  kBS(nBS) .gt. n) then
            call s5Bswp( gotR, lenr, m, mBS, n, nb, nS,
     $                   jBSr, LUreq, pivot, 
     $                   ne, nka, a, ha, ka, 
     $                   hs, kBS, blBS, buBS, r, xBS, 
     $                   y, y2, y1, iw, leniw, rw, lenrw )
            if (LUreq .gt. 0) then
               needLU = .true.
               go to 100
            end if
         end if

         nInf   = 0
         sInf   = zero
         optiml = .false.
         call dload ( nBS, zero, gBS, 1 )

         if (Elastc  .and.  .not. gotE) then
*           ------------------------------------------------------------
*           Set hEstat and identify the infeasible elastics.
*           Reset blBS and buBS for any infeasible elastics.
*           These values are used in s5step.
*           ------------------------------------------------------------
            call iload ( nb, 0, hEstat, 1 )
            call s5setE( nb, nBS, featol, plInfy,
     $                   hElast, hEstat, kBS,
     $                   bl, bu, blBS, buBS, xBS )
            gotE  = .true.
         end if

         if ( chkFea ) then

*           In Phase 1 or just after a factorize, check the feasibility 
*           of the basic and superbasic non-elastics.
*           jstFea  indicates that we have just become feasible.
*           jstFea is turned off once a step is taken.

            call s5Inf ( nBS, featol, 
     $                   nInf, sInf, hfeas, blBS, buBS, gBS, xBS )

            if (nInf .gt. 0) then

*              Non-elastics are infeasible.
*              Print something if the basis has just been refactorized.

               if (Prnt10  .and.  iw(LUitn) .eq. 0) then
                  write(iPrint, 1030) itn, nInf, sInf
               end if
            end if

*           Feasbl = true means that the non-elastics are feasible.
*                    This defines Phase 2.

            if (.not. feasbl) 
     $      jstFea = nInf .eq. 0
            feasbl = nInf .eq. 0
            chkFea = nInf .gt. 0
         end if

         if ( Elastc ) then
*           ------------------------------------------------------------
*           Compute the sum of infeasibilities of the elastic variables.
*           ------------------------------------------------------------
            call s5InfE( nb, nBS, hEstat, kBS,
     $                   nInfE, sInfE, bl, bu, xs )
            nInf = nInf + nInfE
            sInf = sInf + sInfE
         end if

         if (jstFea  .and.  (FP .or. (Elastc .and. lvlInf .eq. 0))) then
*           ------------------------------------------------------------
*           The non-elastic variables just became feasible. Exit.
*           ------------------------------------------------------------
            condHz = zero
            djqPrt = zero
            rgnorm = zero
            pinorm = zero
            call dload ( m, zero, pi, 1 )
            deadpt = .false.
            optiml = .true.

         else
*           ------------------------------------------------------------
*           If (x,s) is feasible or a composite objective is being
*           minimized, use the LP/QP gradient.
*           Otherwise, use the gradient of the sum of infeasibilities. 
*           ------------------------------------------------------------
            ObjPhs = feasbl  .and.  (nInf .eq. 0  .or.  lvlInf .ne. 2)

            if ( feasbl ) then
*              ---------------------------------------------------------
*              Feasible with respect to the non-elastic variables.
*              We are in  phase 2 (in either normal or elastic mode).
*              ---------------------------------------------------------
*              If just feasible, compute the QP objective (and gradient)
*              and R.

               ObjLP = zero

               if ( ObjPhs ) then
                  if (iObj .ne. 0) then
                     ObjLP = xBS(iw(kObj))*sclObj
                  end if
                  Obj = sgnObj*ObjLP
               end if

               if (jstFea  .or.  newLU) then

*                 Change some of the blBS, buBS  bounds.
                     
                  if ( Elastc ) 
     $            call s5bndE( nb, nBS, plInfy, hEstat, kBS,
     $                         bl, bu, blBS, buBS )
                  
                  if ( ObjPhs ) then
*                    ===================================================
*                    Initialize ObjQP and gObjQP, the value and
*                    derivative of the quadratic part of the objective.
*                    ===================================================
                     if ( gotg ) then
                        call s5QPfg( Hx, Hx1, minimz, lenx0, 
     $                               n, ngQP, ngObj, ngObj0, nnH,
     $                               nState, nHx, ObjQP,
     $                               ne, nka, a, ha, ka, 
     $                               gObj, gObjQP, x0, xs, xdif,
     $                               cu, lencu, iu, leniu, ru, lenru,
     $                               cw, lencw, iw, leniw, rw, lenrw )
                        Obj    = Obj + sgnObj*ObjQP
                        nState = 0
                     end if ! gotg

                     if (gotH  .and.  .not. gotR) then
*                       ------------------------------------------------
*                       If R is not supplied on entry, load and
*                       factorize the reduced Hessian.
*                       This will happen after every refactorize.
*                       ------------------------------------------------
                        if (nS .gt. 0) then
                           call s5HZ  ( Hx, Hx1, lenr, minimz,
     $                                  m, mBS, n, nb, nnH, nS, nHx,
     $                                  ne, nka, a, ha, ka,
     $                                  Hdmax, kBS, 
     $                                  r, w, y2, y1, 
     $                                  cu, lencu, iu, leniu, ru, lenru,
     $                                  cw, lencw, iw, leniw, rw, lenrw)
*                          call s5Hfac( 'Interchanges', ierror,
                           call s5Hfac( 'No interchanges', ierror,
     $                                  lenr, m, maxR, mBS, nb, nS,
     $                                  Hdmax, hs, kBS, iy, 
     $                                  bl, bu, blBS, buBS, xs, xBS, r,
     $                                  iw, leniw, rw, lenrw )
                          nBS    = m + nS
                           if (ierror .ne. 0) go to 100
                        end if ! nS > 0
                        gotR = .true.
                     end if ! gotH and not gotR
                     posdef = gotR  .or.  nS .eq. 0
                  end if ! ObjPhs
               end if ! jstFea .or. newLU

*              ---------------------------------------------------------
*              Place the gradient in BS order.
*              Assign the nonzero components of gBS.
*              ---------------------------------------------------------
               if ( ObjPhs ) then
                  if ( gotg ) then
                     call s5BSg ( nBS, ngQP, ngQP0, minimz, 
     $                            kBS, gObjQP, gBS )
                  end if

                  if (iObj .ne. 0) gBS(iw(kObj)) = sgnObj*sclObj
               end if

               if ( Elastc ) then
                  call s5grdE( nb, nBS, wtInf, hEstat, kBS, gBS )
               end if
            end if ! feasible

            if ( needpi ) then
               call dcopy ( m, gBS, 1, y, 1 )
               call s5setp( m, pinorm, y, pi, iw, leniw, rw, lenrw )
               needpi = .false.
            end if

            rgnorm = zero
            if (nS .gt. 0) then
               call s5rg  ( m, nBS, n, nS, eps0, gBS, pi, rg, rgnorm,
     $                      ne, nka, a, ha, ka, kBS )
            end if

*           ============================================================
*           Determine if the reduced Hessian is definite.
*           ============================================================
            condHz = zero
            if ( gotR ) then
               call s6Rcnd( nS, lenr, r, dRmax, dRmin, Rmax, condHz )
            end if

            if (.not. posdef) then
               if (ObjPhs  .and.  gotR) then
                  if (nS .gt. 0) then
                     lRs  = nS*(nS + 1)/2
                     dRsq = r(lRs)**2
                  end if
                  call s5Rsng( posdef, inform, itn, lenr, nS, dRsq, r,
     $                         iw, leniw, rw, lenrw )
               else
                  posdef = nS .eq. 0
               end if
            end if

*           ============================================================
*           Check for optimality. 
*           If xs is a minimizer,  reduced costs are calculated.
*           In theory, the reduced gradient is zero after a bound swap.
*           ============================================================
            if ( feasbl ) then
               rw(toldj3) = tolQP
            else
               rw(toldj3) = tolFP
            end if

            statpt = rgnorm .le. eps0*pinorm
            deadpt = .false.
            needLM = statpt

            if ( bndswp ) then
               if (QP  .and.  ObjPhs) nUncon = nUncon + 1
               jqSave = 0
               bndswp = .false.
            end if

            if (ObjPhs  .and.  QP) then

*              Refactorize if iterative refinement could not reduce
*              rgnorm/pinorm  below 1.0d-4.

               giveup = nUncon  .gt. mUncon
               rgfail = giveup  .and.  (rgnorm .gt. 1.0d-4*pinorm)

               if ( rgfail ) then
                  if (iw(LUitn) .gt. 0) then
                     nUncon = 0
                     LUreq  = 11
                     typeLU = 'BS'
                     needLU = .true.
                  else
                     ierror = 11
                  end if
                  go to 100
               end if

               deadpt = statpt .and. .not. posdef
               needLM = statpt .or.        giveup  .or.  nochng
            end if

            kPrPrt = kPrc   
            jq     = 0
            djqPrt = djq
            djq    = zero

            if ( needLM ) then
*              ---------------------------------------------------------
*              Compute Lagrange multipliers.
*              ---------------------------------------------------------
               nUncon = 0
               useg   = ObjPhs  .and.  gotg
               weight = zero
               if (Elastc  .and.  feasbl) then
                  weight = wtInf
               end if

               call s5pric( Elastc, feasbl, incres, useg,
     $                      itn, m, n, nb, ngQP, ngQP0, nnH,
     $                      nS, nonOpt, weight, sgnObj, piNorm,
     $                      jq, djq, kPrc, rw(toldj1),
     $                      ne, nka, a, ha, ka,
     $                      hElast, hs, bl, bu, gObjQP, pi, rc,
     $                      iw, leniw, rw, lenrw )

               optiml = nonOpt .eq. 0
               newSB  = nonOpt .gt. 0
            end if ! needLM
         end if ! jstFea

         QPdone = optiml  .or.  deadpt  .or.  nochng

         if ( QPdone ) then 
*           ------------------------------------------------------------
*           We are apparently finished.
*           See if any nonbasics have to be set back on their bounds.
*           ------------------------------------------------------------
            call s5dgen( 'Optimal', inform, lPrint, 
     $                   nb, nInf, itn,
     $                   featol, tolx, tolinc, hs, bl, bu, xs,
     $                   itnfix, nfix, tolx0,
     $                   iw, leniw, rw, lenrw )

            QPdone = inform .eq. 0

            if ( QPdone ) then
*              -----------------------------------------
*              So far so good.  Check the row residuals.
*              -----------------------------------------
               if (iw(LUitn) .gt. 0) then
                  call s5setx( 'Check residuals', inform, itn,
     $                         m, n, nb, nBS, rowerr, xNorm,
     $                         ne, nka, a, ha, ka, 
     $                         b, lenb, kBS, xBS, 
     $                         xs, w, y, iw, leniw, rw, lenrw )
                  QPdone = inform .eq. 0
                  if (.not. QPdone) then
                     LUreq  = 10
                     typeLU = 'BT'
                     needLU = .true.
                  end if
               end if
            end if

            if ( QPdone ) then
               if (deadpt  .or.  nochng) ierror = 4
            else
               needx  = .true.
               feasbl = .false.
               nochng = .false.
               needpi = .true.
               go to 100
            end if
         end if ! done

*        ===============================================================
*        Print the details of this iteration.
*        ===============================================================
         if ( ObjPhs ) ObjPrt = ObjAdd + ObjLP + ObjQP

         call s5log ( prbtyp, contyp, 
     $                Elastc, gotR, prtFea, jstFea, ObjPhs,
     $                m, mBS, nS,
     $                jSq, jBr, jSr, jBSr,
     $                lines1, lines2,
     $                itn, kPrPrt, lvlInf, lPrint, 
     $                sgnObj, pivot, step, nInf, sInf, wtInf,
     $                ObjPrt, condHz, djqPrt, rgnorm, kBS, xBS,
     $                iw, leniw )

         jBq    = 0
         jBr    = 0
         jBSr   = 0
         jSq    = 0
         jSr    = 0
         kPrPrt = 0

         if ( QPdone ) then

            if (nInf .gt. 0) then
*              ---------------------------------------------------------
*              Convergence, but no feasible point.
*              Stop or continue in elastic mode, depending on the
*              specified level of infeasibility.
*              ---------------------------------------------------------
               if (lEmode .gt. 0) then
                  if (.not. Elastc) then

*                    The constraints are infeasible in Normal mode.
*                    Print a message and start Elastic Phase 1.

                     if (lPrint .gt. 0) then
                        if (iPrint .gt. 0) then
                           write(iPrint, 8050) itn, contyp
                           write(iPrint, 8060) itn
                        end if
                        if (iSumm  .gt. 0) then
                           write(iSumm , 8050) itn, contyp 
                           write(iSumm , 8060) itn
                        end if
                     end if
                     Elastc = .true.
                     needpi = .true.
                     QPdone = .false.
                     djq    = zero
                     step   = zero
                  else if (.not. feasbl) then

*                    The non-elastic bounds cannot be satisfied 
*                    by relaxing the elastic variables.

                     ierror = 1
                  end if
                  go to 100
               end if
            end if

            if (LP  .or.  QP) then
               If (jq .ne. 0) then
                  djq    = sgnObj*djq
                  if (Prnt1) write(iPrint, 1010) djq, jq, rgnorm, pinorm
               else
                  if (Prnt1) write(iPrint, 1020)          rgnorm, pinorm
               end if
            end if
         else 
*           ------------------------------------------------------------
*           A nonbasic has been selected to become superbasic.
*           Compute the vector y such that B y = column jq.
*           ------------------------------------------------------------
            if ( newSB ) then
*              ---------------------------------------------------------
*              The price has selected a nonbasic to become superbasic.
*              ---------------------------------------------------------
               if (nS+1 .gt. maxS) then 
                  ierror = 5
                  go to 100
               end if

*              ---------------------------------------------------------
*              Compute the vector y such that B y = column jq.
*              y is a multiple of the new column of  Z  and is used
*              as the first search direction.
*              ---------------------------------------------------------
*              Unpack column jq into  y1  and solve  B*y = y1.
*              The altered  y1  satisfies  L*y1 = ajq. 
*              It is used later to modify L and U.

               call s2unpk( jq, m, n, ne, nka, a, ha, ka, y1 )
               call s2Bsol( 'B y = yq', inform, m, y1, y, 
     $                      iw, leniw, rw, lenrw )
            end if

*           ============================================================
*           Take a step.
*           ============================================================
            if (itn  .ge. itnlim  .or.  itQP .ge. itMax) then 
               ierror = 3
               go to 100
            end if

            itQP      = itQP   + 1
            itn       = itn    + 1
            iw(LUitn) = iw(LUitn)  + 1
            newLU     = .false.
            jstFea    = .false.

*           ------------------------------------------------------------
*           Take a ``reduced gradient'' step.
*           The new  xs  will either minimize the objective on the
*           working set or lie on the boundary of a new constraint.
*           ------------------------------------------------------------
            call s5QPit( Hx, Hx1, bndswp, Elastc, feasbl,
     $                   gotg, gotH, gotR, incres, ObjPhs, 
     $                   needpi, newSB, posdef,
     $                   ierror, itn, lenr, m, mBS, maxS, 
     $                   n, nb, ngQP0, ngQP, nnH,
     $                   nS, nDegen, nHx, LUreq,
     $                   kp, jBq, jSq, jBr, jSr, jq,
     $                   jqSave, nUncon, djq, 
     $                   minimz, Obj, ObjQP,
     $                   featol, pivot, step, tolinc, wtInf,
     $                   ne, nka, a, ha, ka, 
     $                   hElast, hEstat, hfeas, hs, kBS, 
     $                   bl, bu, blBS, buBS,
     $                   gBS, gObjQP, rg, r, w, xBS, xs, 
     $                   y, y2, y1, 
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )
            if (ierror .ne. 0) then

*              If unbounded or a weak solution, refactorize the basis.

               if (ierror .eq. 2  .or.  ierror .eq. 4) then
                  nochng = .true.
                  if (iw(LUitn) .gt. 0) then
                     needLU = .true.
                     typeLU = 'BS'
                     LUreq  = 11
                     ierror = 0
                  end if
               end if
               go to 100
            end if

*           Increment featol every iteration.

            featol = featol + tolinc

*           ============================================================
*           Test for error condition and/or frequency interrupts.
*           ============================================================
*           (1) Save a basis map (frequency controlled).
*           (2) Every kdegen iterations, reset featol and move nonbasic 
*               variables onto their bounds if they are very close.
*           (3) Refactorize the basis if it has been modified too many
*               times.
*           (4) Update the LU factors of the basis if requested.
*           (5) Check row error (frequency controlled).

            if (mod(itn,ksav) .eq. 0) then
               call s5hs  ( 'External', nb, bl, bu, hs, xs )
               call s4ksav( minimz, m, n, nb, nS, mBS, ObjQP,
     $                      itn, nInf, sInf,
     $                      kBS, hs, aScale, bl, bu, xBS, xs,
     $                      cw, lencw, iw, leniw )
               call s5hs  ( 'Internal', nb, bl, bu, hs, xs )
            end if

            if (mod( itn, kdegen ) .eq. 0) then
               call s5dgen( 'End of cycle', inform, lPrint, 
     $                      nb, nInf, itn,
     $                      featol, tolx, tolinc, hs, bl, bu, xs,
     $                      itnfix, nfix, tolx0,
     $                      iw, leniw, rw, lenrw )
               needx  = inform .gt. 0
            end if

            if (LUreq .eq. 0) then

               if (     iw(LUmod) .ge. kfac-1) then
                  LUreq  = 1
               else if (iw(LUmod) .ge. 20  .and. 
     $                                iw(lenL)+iw(lenU) .gt. LUmax) then
                  Bgrwth = iw(lenL) + iw(lenU)
                  Bold   = LUsiz0
                  Bgrwth = Bgrwth/Bold
                  if ( Prnt10 ) write(iPrint, 1000) Bgrwth
                  LUreq  = 2
               else 
                  checkx = mod(iw(LUitn),kchk) .eq. 0
                  if (checkx  .and.  .not. needx) then
                     call s5setx( 'Check residuals', inform, itn,
     $                            m, n, nb, nBS, rowerr, xNorm,
     $                            ne, nka, a, ha, ka, 
     $                            b, lenb, kBS, xBS, 
     $                            xs, w, y, iw, leniw, rw, lenrw )
                     if (inform .gt. 0) then
                        LUreq  = 10
                     end if
                  end if
               end if

               if (LUreq  .gt. 0) then
                  needLU = .true.
                  typeLU = 'BT'
               end if
            end if
         end if ! not optiml

         go to 100
*+    end while
      end if
*     ======================end of main loop============================
*
      call s5hs  ( 'External', nb, bl, bu, hs, xs )

      return

 1000 format(  ' ==> LU file has increased by a factor of', f6.1)
 1010 format(/ ' Biggest dj =', 1p, e11.3, ' (variable', i7, ')',
     $         '    norm rg =',     e11.3, '   norm pi =', e11.3)
 1020 format(/    ' Norm rg =', 1p, e11.3, '   norm pi =', e11.3)
 1030 Format(' Itn', i7, ': Infeasible nonelastics.  Num =', i5, 1p,
     $       '   Sum of Infeasibilities =', e8.1 )

 2000 format(' ')

 8050 format(  ' Itn', i7, ': Infeasible ', a)
 8060 format(  ' Itn', i7, ': Elastic Phase 1 -- making',
     $                     ' non-elastic variables feasible')

*     end of s5QP
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5QPfg( Hx, Hx1, minimz, lenx0, 
     $                   n, ngQP, ngObj, ngObj0, nnH, 
     $                   nState, nHx, ObjQP,
     $                   ne, nka, a, ha, ka, 
     $                   gObj, gObjQP, x0, x, xdif, 
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      external           Hx, Hx1
      integer            ha(ne)
      integer            ka(nka)
      double precision   a(ne)
      double precision   gObj(ngObj0), gObjQP(ngQP)
      double precision   x0(*), x(ngQP), xdif(ngQP) 

      character*8        cu(lencu), cw(lencw)
      integer            iu(leniu), iw(leniw)
      double precision   ru(lenru), rw(lenrw)

*     ==================================================================
*     s5qpfg  computes various quantities associated with the LP/QP.
*
*       1.  ObjQP  =  gObj'*(x-x0)  + half*(x - x0)'*H*(x - x0)
*       2.  gObjQP =  gradient of ObjQP
*
*     On entry, 
*     x(ngQP)      are the nonlinear variables
*     x0(ngQP)     is the base point x0
*     gObj(ngObj)  defines the explicit QP linear term
*
*     On exit,
*     ObjQP        is the QP quadratic term (1) above
*     gObjQP(ngQP) is the gradient of ObjQP
*     xdif(ngQP)   is  x-x0
*
*     02-May-92: First version of s5qpfg.
*     23-Oct-93: Hx added as an argument.
*     29-Oct-93: Modified to compute only the QP objective.
*     07-Oct-94: gObjQP added as an argument.
*     26-Jul-97: Current version.
*     ==================================================================
      parameter         (zero = 0.0d+0, half = 0.5d+0, one = 1.0d+0)
*     ------------------------------------------------------------------

      if (ngQP .le. 0) return

      call dcopy ( ngQP,         x , 1, xdif, 1 )
      if (lenx0 .gt. 0)
     $call daxpy ( ngQP, (-one), x0, 1, xdif, 1 )
               
      ObjQP  = zero

      if (nnH .gt. 0) then
         call Hx    ( Hx1, minimz, n, nnH, ne, nka, a, ha, ka, 
     $                xdif, gObjQP, nState, 
     $                cu, lencu, iu, leniu, ru, lenru, 
     $                cw, lencw, iw, leniw, rw, lenrw )
         nHx   = nHx + 1
         ObjQP = half*ddot( nnH, xdif, 1, gObjQP, 1 )
      end if

      nzero = ngQP - nnH
      if (nzero .gt. 0) call dload ( nzero, zero, gObjQP(nnH+1), 1 )

      if (ngObj .gt. 0) then
         ObjQP = ObjQP + ddot( ngObj, gObj, 1, xdif, 1 )
         call daxpy ( ngObj, one, gObj, 1, gObjQP, 1 )
      end if

*     end of s5QPfg
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5QPit( Hx, Hx1, bndswp, Elastc, feasbl, 
     $                   gotg, gotH, gotR, incres, ObjPhs,
     $                   needpi, newSB, posdef,
     $                   ierror, itn, lenr, m, mBS, maxS, 
     $                   n, nb, ngQP0, ngQP, nnH,
     $                   nS, nDegen, nHx, LUreq,
     $                   kp, jBq, jSq, jBr, jSr, jq,
     $                   jqSave, nUncon, djq, 
     $                   minimz, Obj, ObjQP, 
     $                   featol, pivot, step, tolinc, wtInf,
     $                   ne, nka, a, ha, ka, 
     $                   hElast, hEstat, hfeas, hs, kBS, 
     $                   bl, bu, blBS, buBS,
     $                   gBS, gObjQP, rg, r, Hy, xBS, xs, 
     $                   y, y2, y1, 
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      external           Hx, Hx1
      logical            bndswp, Elastc, feasbl
      logical            gotg, gotH, gotR, incres, ObjPhs
      logical            needpi, newSB, posdef
      integer            ha(ne), hElast(nb), hEstat(nb), hs(nb)
      integer            hfeas(mBS), kBS(mBS)
      integer            ka(nka)
      double precision   a(ne), r(lenr), rg(maxS)
      double precision   bl(nb), bu(nb), xs(nb)
      double precision   Hy(nb), y2(nb)
      double precision   gObjQP(ngQP0)
      double precision   blBS(mBS), buBS(mBS), gBS(mBS), xBS(mBS)
      double precision   y(mBS)
      double precision   y1(m)

      character*8        cu(lencu), cw(lencw)
      integer            iu(leniu), iw(leniw)
      double precision   ru(lenru), rw(lenrw)

*     ==================================================================
*     s5QPit performs a QP step.
*
*     On entry, 
*        newSB = true implies that variable jq just went sb. 
*                In this case:
*                y  contains the vector  y   such that B y  = a(jq).
*                y1 contains the vector  y1  such that L y1 = a(jq).
*
*     25-Nov-91: First version of s5QPit.
*     05-Jan-96: Positive semidefinite R treated correctly.
*     29-Aug-96: First min sum version added.
*     27-Jul-97: Thread-safe version.
*     02-Feb-98: Piecewise linear line search added.
*     11-Feb-98: Current version of s5QPit.
*     ==================================================================
      logical            done, hitcon, hitlow, move
      logical            onbnd , unbndd, Uncon , vertex
      logical            Hposdf, singlr
      parameter         (zero      = 0.0d+0, half =  0.5d+0)
      parameter         (one       = 1.0d+0, ten  = 10.0d+0)

      parameter         (LUmod     = 216)
*     ------------------------------------------------------------------
      eps       = rw(  1)
      eps0      = rw(  2)
      tolpiv    = rw( 60)
      plInfy    = rw( 70)
      bigdx     = rw( 72)

      iPrint    = iw( 12)
      ifDefH    = iw(200)

      inform    = 0
      nState    = 0

      unbndd    = .false.
      vertex    = nS .eq. 0
      sgnObj    = minimz

      nBS       = m + nS

      if ( newSB ) then
*        ---------------------------------------------------------------
*        New superbasic. 
*        posdef must be true if there is a new superbasic.
*        ---------------------------------------------------------------
         nS1    = nS   + 1
         nBS1   = nBS  + 1

         bndswp = .false.
         posdef = .false.

         if ( gotR ) then 
*           ------------------------------------------------------------
*           Add the new column to R at position nS+1.
*           Check for a singular or indefinite reduced Hessian.
*           Use Hy as workspace.
*           ------------------------------------------------------------
            kBS(nBS1) = jq
            call s5Rcol( Hx, Hx1, inform, minimz,
     $                   jq, nS1, lenr, 
     $                   m, mBS, n, nb, nnH, nS1, nHx,
     $                   ne, nka, a, ha, ka,
     $                   kBS, r, y2, Hy, y, 
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

            lRnew = nS1*(nS1 + 1)/2
            dRsq  = r(lRnew)
            call s5Rsng( Hposdf, inform, itn, lenr, nS1, dRsq, r,
     $                   iw, leniw, rw, lenrw )
            if ( ObjPhs ) then
               posdef = Hposdf
               if (.not. posdef  .and.  ifDefH .eq. 1) then
                  ierror = 6   ! Hessian is not sufficiently definite
                  go to 900
               end if
            else
               gotR   = Hposdf 
            end if

            if (inform .gt. 0  .and.  ObjPhs) then
*              ---------------------------------------------------------
*              Reduced Hessian is not sufficiently positive definite.
*              ---------------------------------------------------------
               ierror = 6
               go to 900
            end if

            if ( gotR ) r(lRnew) = sqrt( dRsq )  

         end if ! ObjPhs and gotR

*-->     R can be checked here.

         if ( incres ) then
            jqSave =   jq
         else
            jqSave = - jq               
         end if

         nS     =  nS1
         nBS    = nBS1

         hfeas(nBS) =     0
         kBS  (nBS) =    jq
         xBS  (nBS) = xs(jq)
         blBS (nBS) = bl(jq)
         buBS (nBS) = bu(jq)

         if (ObjPhs  .and.  gotg   .and.  jq .le. ngQP) then
            gBS(nBS) = sgnObj*gObjQP(jq)
         else
            gBS(nBS) = zero
         end if

*        ===============================================================
*        Set hEstat(jq) and the elastic parts of blBS and buBS.
*        ===============================================================
         if ( Elastc ) then

*           If the new superbasic is an elastic variable
*           and it wants to move infeasible, set its elastic state.

            if (hElast(jq) .gt. 0) then 
               js  = hs(jq)
               if ( incres ) then
                  if (js .eq. 1  .or.  js .eq. 4) then
                     hEstat(jq) =   2
                     buBS(nBS)  =   plInfy
                     if ( feasbl ) then
                        gBS (nBS) = gBS(nBS) + wtInf
                        blBS(nBS) = bu(jq)
                     end if
                  end if
               else
                  if (js .eq. 0  .or.  js .eq. 4) then
                     hEstat(jq) =   1 
                     blBS(nBS)  = - plInfy
                     if ( feasbl ) then
                        gBS (nBS) = gBS(nBS) - wtInf
                        buBS(nBS) = bl(jq)
                     end if
                  end if
               end if
            end if
         end if ! Elastc

*        ---------------------------------------------------------------
*        In phase 1, or in phase 2 for an LP, price can select nonbasics
*        floating free between their bounds with zero reduced cost. 
*        We have to check that dqj is not zero.
*        ---------------------------------------------------------------
         rg(nS) = djq
         if (.not. feasbl  .or.  nnH .eq. 0) then
            if (hs(jq) .eq. -1) then
               if (incres) then
                  rg(nS) = - one
               else
                  rg(nS) =   one
               end if
            end if
         end if
         jSq    = jq
         hs(jq) = 2

      end if ! newSB

      singlr = .not. posdef

*     ------------------------------------------------------------------
*     Get a search direction yS for the superbasics, store it in
*     y(m+1:nBS), and find its norm.  Put the search direction for the 
*     basic variables in y(1)  ,...,y(m).
*     ------------------------------------------------------------------
      call s5getd( ObjPhs, gotR, posdef,
     $             lenr, nS, r, rg, y(m+1), gy, yHy )

      ynorm  = dasum( nS, y(m+1), 1 )

*     Compute  y2 = - S*yS and prepare to solve  B*y = y2
*     to get y, the search direction for the basic variables.
*     We first normalize y2 so the LU solver won't ignore
*     too many "small" elements while computing y.

      call dscal ( nS, (one/ynorm), y(m+1), 1 )
      call s2Bprd( 'No transpose', eps0, n, nS, kBS(m+1),
     $             ne, nka, a, ha, ka,
     $             (-one), y(m+1), nS, zero, y2, m ) 

*     Solve  B*y = y2  and then unnormalize all of y.

      call s2Bsol( 'B y = y2', inform, m, y2, y, 
     $             iw, leniw, rw, lenrw  )
      call dscal ( nBS , ynorm, y, 1 )
      ynorm  = ynorm + dasum( m, y, 1 )

      if ( ObjPhs ) then
*        ---------------------------------------------------------------
*        If R is singular, ensure that y is a feasible direction.
*        ---------------------------------------------------------------
         if (gotR  .and.  singlr) then
            call s5chkd( inform, nBS, jqSave, kBS, gy, y, iw, leniw )
            if (inform .gt. 0) then
               ierror = 4
               go to 900
            end if
         end if

*        ---------------------------------------------------------------
*        Compute the first- and second-order terms of the Taylor-series
*        expansion of ObjQP. This is used for printing purposes.
*        Compute the product H*y, where y is an nnH-vector.
*        ---------------------------------------------------------------
         if (gotg  .or.  gotH) then
            call dload ( ngQP, zero, y2, 1 )
            do 100, k = 1, nBS
               j      = kBS(k)
               if (j .le. ngQP) y2(j) = y(k)
  100       continue

            if ( gotg ) then 
               gyQP  = ddot ( ngQP, gObjQP, 1, y2, 1 )
            end if

            if ( gotH ) then
               yHyQP = zero
               call Hx    ( Hx1, minimz, n, nnH, ne, nka, a, ha, ka, 
     $                      y2, Hy, nState,
     $                      cu, lencu, iu, leniu, ru, lenru, 
     $                      cw, lencw, iw, leniw, rw, lenrw )
               nHx   = nHx + 1
               yHyQP = ddot( nnH, y2, 1, Hy, 1 )
            end if
         end if
      end if ! ObjPhs

*     ------------------------------------------------------------------
*     Find the nearest constraint in direction  xs + step*y (step > 0).
*     Exact  is the step that takes xBS(kp) exactly onto bound.
*     It may be positive or slightly negative. (Not defined if unbndd.)
*
*     If exact isn't positive and we are not at a vertex, we change
*     step and stepmx to zero and don't move.
*
*     If onbnd  is true, step is a step that reaches a bound exactly.
*     xBS(kp) reaches the value bound.  If we take a constrained step,
*     bound is used to put the new nonbasic variable xs(jr) exactly on
*     its bound.
*
*     If unbndd is true, step = stepmx.
*     ------------------------------------------------------------------
      gtol   = zero
      gtest  = gtol*gy

      tStep  = zero
      stepmx = bigdx /ynorm
      tolp   = tolpiv*ynorm

**    ==================================================================
*+    repeat                                      (until gy >= gtol*gy0)

  200    call s5step( nBS, nDegen,
     $                featol, plInfy, stepmx, tolinc, tolp,
     $                hfeas, blBS, buBS, xBS, y,
     $                hitlow, move, onbnd, unbndd,
     $                kp, bound, exact, stepB, stepP )

*        Find if the step is constrained or unconstrained.
*        If R has been flagged as singular, we double check by trying
*        to compute the QP minimizer along y.  If the minimizer exists, 
*        the singularity tolerance must be too large.

         if ( ObjPhs ) then
            if ( posdef ) then
               Uncon = stepP .gt. one
            else
               Uncon = stepP*yHy .gt. (- gy)
            end if
         else
            Uncon = .false.
         end if

         unbndd = (unbndd  .and.  .not. Uncon)  .or.  stepmx .le. one
         if ( unbndd ) then 
            ierror = 2
            go to 900
         end if

         hitcon = .not. Uncon
         needpi = .true.

         if ( hitcon ) then
            nUncon = 0
            if (vertex  .or.  exact .gt. zero) then
               step  = stepB
            else
               step  = zero
               onbnd = .false.
            end if
         else
            nUncon = nUncon + 1
            pivot  = zero
            if ( posdef ) then
               step   = one
            else
               step   = (- gy)/yHy
               posdef = .true.
            end if
         end if

*        ----------------------------------------------
*        Compute ObjChg, the predicted change in ObjQP.
*        If ObjChg is not sufficiently negative and R
*        is singular, R may not be singular after all.
*        ----------------------------------------------
         if ( ObjPhs ) then
            ObjChg = step*gy + half*yHy*step**2
            if ( singlr  .and.  move ) then
               if (ObjChg .ge. (-ten*eps*abs(Obj))) then
                  ierror = 4
                  go to 900
               end if
            end if
            if (gotg) ObjQP = ObjQP + step*gyQP
            if (gotH) ObjQP = ObjQP             + half*yHyQP*step**2
            Obj   = Obj   + ObjChg

            if (gotH  .and.  (step .gt. zero)) then
               call daxpy ( nnH, step, Hy, 1, gObjQP, 1 )
            end if
         end if

         tStep = tStep + step

*        ---------------------------------------------------------------
*        Update the basic variables xBS.
*        ---------------------------------------------------------------
         call daxpy ( nBS, step, y, 1, xBS, 1 )

         je = 0
         if ( hitcon ) then
            jr = kBS(kp)
            je = hElast(jr)
         end if

         done  = .not. (Elastc  .and.  hitcon  .and.  feasbl
     $                          .and.  move    .and.  je .gt. 0)

         if (.not. done) then
*           ------------------------------------------------------------
*           See if x(jr) can pass through its bound.
*           First, compute gy at the new point.
*           ------------------------------------------------------------
            jEs    = hEstat(jr)
            jEsNew = 0

            if ( ObjPhs ) then
               gy = gy + step*yHy
            end if

            if (jEs .gt. 0) then 

*              x(jr) has reached its bound from the infeasible side.
*              Remove its contribution from the gradient of the sum of
*              infeasibilities.

               blBS(kp) = bl(jr)
               buBS(kp) = bu(jr)
               gy       = gy + wtInf*abs(y(kp))

*              For variables that are meant to be fixed, we can consider
*              violating the opposite bound.

               if (bl(jr) .eq. bu(jr)) then

                  if (jEs .eq. 1 .and. (je .eq. 2 .or. je .eq. 3)) then 
                     gy       =   gy + wtInf*y(kp)
                     blBS(kp) =   bu(jr)
                     buBS(kp) =   plInfy
                     jEsNew   =   2
                     
                  else if (je .eq. 1 .or. je .eq. 3) then 
                     gy       =   gy - wtInf*y(kp)
                     blBS(kp) = - plInfy
                     buBS(kp) =   bl(jr)
                     jEsNew   =   1
                  end if
               end if 
            else ! jEs .eq. 0

*              x(jr) has reached its bound from the feasible side.
*              Compute the effect of violating the bound on the gradient
*              of the sum of infeasibilities.

               if ( hitlow  .and. (je .eq. 1  .or.  je .eq. 3)) then
                  gy       =   gy - wtInf*y(kp) ! y(yp) < 0
                  blBS(kp) = - plInfy
                  buBS(kp) =   bl(jr)
                  jEsNew   =   1
               else if (je .eq. 2  .or.  je .eq. 3) then
                  gy       =   gy + wtInf*y(kp) ! y(yp) > 0
                  blBS(kp) =   bu(jr)
                  buBS(kp) =   plInfy
                  jEsNew   =   2
               end if
            end if

            done = gy .ge. gtest

            if (.not. done ) then
               hEstat(jr) = jEsNew                  
            end if
         end if
*+    until     done
      if (.not. done) go to 200

*     ------------------------------------------------------------------
*     Copy the basic variables xBS into xs.
*     ------------------------------------------------------------------
      step    = tStep
      call s5BSx ( 'xBS to x', nBS, nb, kBS, xBS, xs )

      if ( hitcon ) then
*        ===============================================================
*        There is a blocking variable.
*        It could be a fixed variable, whose new state must be 4.
*        ===============================================================
         pivot  = - y(kp)
         jr     =   kBS(kp)

         bndswp = jr .eq. abs(jqSave)

         if (onbnd) xs(jr) = bound

         jEs    = hEstat(jr)
         hEstat(jr) = 0

         if      (jEs .eq. 0) then 
            if (blBS(kp) .eq. buBS(kp)) then
               jrstat = 4
            else if (hitlow) then
               jrstat = 0
            else
               jrstat = 1
            end if

         else if (jEs .eq. 1) then
            if (bl(jr) .eq. bu(jr)) then
               jrstat =   4
            else if (onbnd) then
               jrstat =   0
            else if (xs(jr) .lt. bu(jr)) then
               jrstat = - 1
            else
               jrstat =   1
            end if

         else !   jEs .eq. 2
            if (bl(jr) .eq. bu(jr)) then
               jrstat =   4
            else if (onbnd) then
               jrstat =   1
            else if (xs(jr) .gt. bl(jr)) then
               jrstat = - 1
            else
               jrstat =   0
            end if
         end if

         if (kp .le. m) then
*           ============================================================
*           A variable in B hit a bound.
*           Find column kSq = kBSq-m  of S to replace column kp of B.
*           ============================================================
            call dload ( m, zero, y2, 1 )
            y2(kp) = one
            call s2Bsol( 'Bt y = y2', inform, m, y2, y, 
     $                   iw, leniw, rw, lenrw  )
            call s5chzq( m, mBS, n, nb, nS, kBSq, pivot,
     $                   ne, nka, a, ha, ka,
     $                   kBS, bl, bu, xBS, y,
     $                   iw, leniw, rw, lenrw )
            
            if (kBSq .le. 0) then
               write (iPrint, 1000) 
               kBSq   = nBS
            end if

            kSq        = kBSq - m

            hs(jr)     = jrStat
            jBr        = jr                     ! Outgoing basic
            jSr        = kBS(kBSq)              ! Outgoing superbasic
            kBS (kBSq) = jBr
            jBq        = jSr                    ! Incoming basic
            kBS (kp)   = jSr
            blBS(kp)   = blBS(kBSq)
            buBS(kp)   = buBS(kBSq)
            xBS (kp)   = xBS (kBSq)
            hs(jBq)    = 3

            if ( gotR ) then

*              Finish computing y(m+1), ..., y(m+nS).

               y(kBSq) = - (one + pivot)
               call dscal ( nS, (one/pivot), y(m+1), 1 )
               call s6Rswp( nS, lenr, r, y2, y(m+1), kSq )
            end if

*           ------------------------------------------------------------
*           Get  y1, used to modify L and U.  If the outgoing
*           superbasic just came in, we already have it.
*           ------------------------------------------------------------
            if (jSr .ne. jq) then
               call s2unpk( jBq , m, n, ne, nka, a, ha, ka, y1 )
               call s2Bsol( 'L ', inform, m, y1, y, 
     $                      iw, leniw, rw, lenrw  )
            end if

*           Update the LU factors.

            iw(LUmod)  = iw(LUmod) + 1
            call s2Bmod( inform, kp, m, y1, 
     $                   iw, leniw, rw, lenrw )
            if (inform .eq. 7) then
               LUreq  = 7                       ! No free memory.
            end if

*-->        R can be checked here.

         else
*           ============================================================
*           A variable in S hit a bound.
*           ============================================================
            hs(jr) = jrStat
            jSr    = jr
            kBSq   = kp
            kSq    = kBSq - m
         end if

*        Cyclically demote the kSq-th superbasic to position nS
*        and adjust all arrays that reflect the BS order. 

         do 300, j  = kSq, nS-1
            k       = m + j
            kBS (k) = kBS (k+1)
            blBS(k) = blBS(k+1)
            buBS(k) = buBS(k+1)
            xBS (k) = xBS (k+1)
  300    continue

         if ( gotR ) then
*           ------------------------------------------------------------
*           Cyclically demote column kSq of R to position nS.
*           ------------------------------------------------------------
            if (kSq .lt. nS) then
               call s6Rcyc( kSq, nS, eps, lenr, nS, r, y )
            end if
         end if ! ObjPhs and gotH

         nS  = nS  - 1
         nBS = nBS - 1

*-->     R can be checked here.

      end if ! hitcon

  900 return

 1000 format(' XXX  s5QPit:  chzq failed!!')

*     end of s5QPit
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5Rcol( Hx, Hx1, inform, minimz,
     $                   jq, jRadd, lenr, 
     $                   m, mBS, n, nb, nnH, nS, nHx,
     $                   ne, nka, a, ha, ka, 
     $                   kBS, r, v, w, y, 
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      external           Hx, Hx1
      integer            ha(ne)
      integer            ka(nka), kBS(mBS)
      double precision   a(ne), r(lenr)
      double precision   v(nb), w(nb), y(m)

      character*8        cu(lencu), cw(lencw)
      integer            iu(leniu), iw(leniw)
      double precision   ru(lenru), rw(lenrw)

*     ==================================================================
*     s5Rcol  computes column jRadd of the Cholesky factor R such that 
*     R'R = Z'HZ.  The update corresponds to the addition of a new 
*     column to Z.
*
*     On entry, 
*        R     holds the columns of the factor associated with the 
*              first jRadd-1 columns of Q.
*
*        y     is the vector such that B y = a(jq).
*
*        nS    is the number of columns in R.
*
*     11-Dec-91: First version of s5Rcol based on Qpsol routine Qpcolr.
*     24-Apr-94: Columns of Nx no longer in Q.
*     26-Jul-97: Current version.
*     ==================================================================
      parameter         (zero = 0.0d+0, one = 1.0d+0)
*     ------------------------------------------------------------------
      eps0      = rw(  2)

      nBS       = m   + nS
      sgnObj    = minimz

      lencol    = min( jRadd-1, nnH )

      lcR       = 1   + (jRadd - 1)*jRadd/2
      ldR       = lcR +  jRadd - 1

*     ------------------------------------------------------------------
*     Get w, the vector of nonlinear components of the new column of Z.
*     ------------------------------------------------------------------
*     The input vector y satisfies B y = column jq.
*     Expand the nonlinear components of y into w.

      call dload ( nnH, zero, w, 1 )
      do 100, k = 1, m
         j      = kBS(k)
         if (j .le. nnH) w(j) = - y(k)
  100 continue
      if (jq .le. nnH) w(jq) = one

      nState = 0
      call Hx    ( Hx1, minimz, n, nnH, ne, nka, a, ha, ka, 
     $             w, v, nState, 
     $             cu, lencu, iu, leniu, ru, lenru, 
     $             cw, lencw, iw, leniw, rw, lenrw )
      nHx    = nHx + 1
      if (minimz .lt. 0) call dscal ( nnH, sgnObj, v, 1 )
      wHw    = ddot ( nnH, w, 1, v, 1 ) 
      Rnrmsq = zero

      if (jRadd .gt. 1) then
*        ---------------------------------------------------------------
*        Compute  Z'Hw  and store it in column jRadd of R. 
*        ---------------------------------------------------------------
*        Set  w = vBS.

         do 300, k = 1, nBS
            j      = kBS(k)
            if (j .le. nnH) then
               w(k) = v(j)
            else
               w(k) = zero
            end if
  300    continue

*        Solve  B' vB = wB  and  form  wS = wS - S' vB.

         call s2Bsol( 'Bt vB = wB', inform, m, w, v, 
     $                iw, leniw, rw, lenrw  )

         if (nS .gt. 0) then
            call s2Bprd( 'Transpose', eps0, n, nS, kBS(m+1),
     $                   ne, nka, a, ha, ka, 
     $                   (-one), v, m, one, w(m+1), nS ) 
         end if

*        -----------------------------------------------------
*        Solve  R' v = Z(j)'Hw.  Store v in column jRadd of R.
*        -----------------------------------------------------
         call dcopy ( lencol, w(m+1), 1, r(lcR), 1 )
         call s6Rsol( 'Rt', lenr, lencol, r, r(lcR) )
         Rnrmsq = ddot  ( lencol, r(lcR), 1, r(lcR), 1 )
      end if

      if (jRadd .le. nnH) then
         r(ldR) = wHw - Rnrmsq
      else
         r(ldR) = zero
      end if

*     end of s5Rcol
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5rg  ( m, nBS, n, nS, tolz, gBS, pi, rg, rgnorm,
     $                   ne, nka, a, ha, ka, kBS )

      implicit           double precision (a-h,o-z)
      integer            ha(ne)
      integer            ka(nka), kBS(nBS)
      double precision   a(ne), gBS(nBS), pi(m), rg(nS)

*     ==================================================================
*     s5rg    calculates the reduced gradient  rg = gS - S'*pi.
*
*     23-Nov-91: First version based on Minos routine m7rg.
*     12-Dec-92: Current version.
*     ==================================================================
      parameter         (one = 1.0d+0)
*     ------------------------------------------------------------------

      call dcopy ( nS, gBS(m+1), 1, rg, 1 )

      call s2Bprd( 'Transpose', tolz, n, nS, kBS(m+1),
     $             ne, nka, a, ha, ka,
     $             (-one), pi, m, one, rg, nS ) 

      kmax   = idamax( nS, rg, 1 )
      rgnorm = abs( rg(kmax) )

*     end of s5rg
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5Rsng( posdef, inform, itn, lenr, nS, dRsq, r,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      logical            posdef
      integer            iw(leniw)
      double precision   rw(lenrw)
      double precision   r(lenr)

*     ==================================================================
*     s5Rsng  finds the inertia of the current reduced Hessian.
*
*     15-Jul-95: First version of s5Rsng written by PEG.
*     12-Aug-95: Current version.
*     ==================================================================
      logical            singlr
      parameter         (zero = 0.0d+0, one = 1.0d+0)
*     ------------------------------------------------------------------
      Hcndbd    = rw( 85)
      iPrint    = iw( 12)
      iSumm     = iw( 13)

      Rsize     = one

      if (nS .eq. 0) then
*        ---------------------------------------------------------------
*        Vertices are positive definite by definition.
*        ---------------------------------------------------------------
         inform = 0
         posdef = .true.
         singlr = .false.

      else
*        ---------------------------------------------------------------
*        Compute dRsqmn, the square of the smallest possible diagonal
*        of a positve-definite reduced Hessian.
*        ---------------------------------------------------------------
         if (nS .eq. 1) then
            dRsqmn = Rsize*(Rsize/Hcndbd)

         else 

*           Find dRmax, the magnitude of the largest diagonal of R.

            dRmax  = abs( r(1) )

            l      = 1
            do 100, j = 2, nS-1
               l      = l + j
               d      = abs( r(l) )
               dRmax  = max( dRmax, d )
  100       continue

            dRsqmn = dRmax*(dRmax/Hcndbd)
         end if

         posdef =      dRsq  .ge. dRsqmn
         singlr =  abs(dRsq) .lt. dRsqmn

         if (singlr  .or.  posdef) then
            inform = 0

            if (dRsq .lt. zero) then
               if (iPrint .gt. 0) then
                  write(iPrint, 1010) itn, dRsq, dRsqmn
               end if
               dRsq = max( zero, dRsq )
            end if

         else
            inform = 1
            if (iSumm  .gt. 0) write(iSumm , 1100) itn
            if (iPrint .gt. 0) write(iPrint, 1110) itn, dRsq, dRsqmn
         end if
      end if

      return

 1010 format(' ==>  Hessian numerically indefinite at itn ', i6,
     $       '.  Square of diag, min diag = ', 1p, 2e9.1 )
 1100 format(' XXX  Indefinite reduced Hessian at itn ', i6)
 1110 format(' XXX  Indefinite reduced Hessian at itn ', i6,
     $       '.  Square of diag, min diag = ', 1p, 2e9.1 )
            
*     end of s5Rsng
      end

