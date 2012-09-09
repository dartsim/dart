*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
*     File  sn87sopt.f
*
*     s8core   s8dflt   s8Mem    s8savB   s8solv
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8core( FP, ierror, iObj, itn, lenr,
     $                   m, maxS, mBS, n, nb, nS, 
     $                   nnCon, nnCon0, nnObj, nnObj0, nnL, nnL0,
     $                   nMajor, nMinor, nDegen,
     $                   sfgCon, fgCon, sfgObj, fgObj,
     $                   duInf, minimz, ObjAdd, fObj, fMrt,
     $                   vimax, virel, viSup, nInf, sInf,
     $                   wtInf0, wtInf, PenNrm, piNorm, xNorm,
     $                   ne, nka, a, ha, ka,
     $                   neJac, nkg, kg, 
     $                   hElast, hEstat, hfeas, hs, kBS,
     $                   bl, bu, blBS, buBS,
     $                   fCon, gCon, gObj, viol, yslk,
     $                   fCon1, gObj1, fCon2, gCon2, gObj2,
     $                   gdif, Hp, pi, r, rc,
     $                   xs, x2, xBS, xdif, xsfeas,
     $                   xMul, xMul2, xdMul, xPen,
     $                   iy, iy2, y, y1, y2, w,
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      external           sfgCon, fgCon, sfgObj, fgObj
      logical            FP

      integer            ha(ne), hfeas(mBS)
      integer            hElast(nb), hs(nb), hEstat(nb)
      integer            ka(nka), kg(nkg), kBS(mBS)
      integer            iy(mBS), iy2(m)
      double precision   a(ne)
      double precision   bl(nb), bu(nb)
      double precision   blBS(mBS), buBS(mBS), xBS(mBS)
      double precision   fCon(nnCon0) , gCon(neJac) , gObj(nnObj0)
      double precision   fCon1(nnCon0),               gObj1(nnObj0)
      double precision   fCon2(nnCon0), gCon2(neJac), gObj2(nnObj0)
      double precision   viol(nnCon0), yslk(nnCon0)
      double precision   xMul(nnCon0), xMul2(nnCon0), xdMul(nnCon0)
      double precision   xPen(nnCon0)
      double precision   gdif(nnL0), Hp(nnL0)
      double precision   rc(nb), xs(nb), x2(nb), xdif(nb), xsfeas(nb)
      double precision   pi(m), r(lenr)
      double precision   y(nb), y1(nb), y2(nb), w(nb)

      character*8        cu(lencu), cw(lencw)
      integer            iu(leniu), iw(leniw)
      double precision   ru(lenru), rw(lenrw)

*     ==================================================================
*     s8core  solves a nonlinear programming problem.
*     A basis is assumed to be specified by nS, hs, xs and the
*     superbasic parts of kBS.
*     In particular, there must be nS values hs(j) = 2, and the
*     corresponding j's must be listed in kBS(m+1) thru kBS(m+ns).
*     The ordering in kBS(m+1:m+nS) matches the reduced Hessian R.
*
*     On entry, if there are nonlinear constraints, yslk contains
*     the true nonlinear slacks (i.e., constraint values)
*     yslk  =  fCon + (linear A)*x,   excluding slacks.
*
*     On exit, if  ierror .lt. 30  it is safe to save the final
*     basis files and print the solution.  Otherwise, a fatal error
*     condition exists and numerous items will be undefined.
*     The last basis map saved (if any) retains the only useful
*     information.
*
*     30 Dec 1991: First version based on npsol routine npcore.
*     23 Oct 1993: Proximal point FP added.
*     29 Oct 1993: Crash on LG rows moved outside s5QP.
*     24 Apr 1994: Nx columns no longer in Q.
*     26 May 1995: Column order of R defined by kBS.
*     04 Aug 1995: Limited memory update
*     11 Aug 1995: tolg changed from 0.1 to 1.0d-4.
*     09 Nov 1995: Updated multipliers used to define Lagrangian.
*     19 Dec 1995: Finite-differences added.
*     09 Oct 1996: First Min Sum version.
*     16 Jul 1997: First thread-safe version.
*     07 Feb 1998: Current version of s8core.
*     ==================================================================
      integer            nGotg(2)
      integer            cdItns
      logical            KTcond(2)
      logical            boostd, centrl, debug , done  , Elastc
      logical            feaSlk, frstQP, gotR  , goodg , incRun, needLU
      logical            newB  , newg  , newLU , nlnCon, nlnObj, maxItn
      logical            maxMjr, maxMnr, maxnS , optiml, overfl, rowFea
      logical            switch, updatd, useFD , usefLS, QPpi0
      external           s8Hx  , s8HDum
      character*2        typeLU
      character*6        MjrMsg

      parameter         (zero  =  0.0d+0, half   = 0.5d+0, one = 1.0d+0)
      parameter         (ten   = 10.0d+0, hundrd = 100.0d+0)
      parameter         (tolg  =  1.0d-4, tolg2  = 1.0d-4)
      parameter         (mReset= 2, mSkip = 10)

      integer            Utol1, Utol2
      parameter         (Utol1     = 154)
      parameter         (Utol2     = 155)

      parameter         (mMinor    =  91)
      parameter         (lvlDif    = 182)
      parameter         (nQNmod    = 201)
      parameter         (MnrHdg    = 223)
      parameter         (MjrHdg    = 224)
      parameter         (MjrSum    = 225)

      character*4        line
      character*19       msg(4:8)
      data               msg/'max step too small.',
     $                       'step too small.    ',
     $                       'no minimizer.      ',
     $                       'too many functions.',
     $                       'uphill direction.  '/
      data               line /'----'/
*     ------------------------------------------------------------------
      iPrint    = iw( 12)
      iSumm     = iw( 13)

      nnJac     = iw( 22)

      lvlSch    = iw( 76)
      lprSch    = iw( 82)

      iCrash    = iw( 88)
      itnlim    = iw( 89)
      mMajor    = iw( 90)
      MjrPrt    = iw( 92)
      MnrPrt    = iw( 93)

      iw(MnrHdg) = 0
      iw(MjrHdg) = 0
      iw(MjrSum) = 0

      lvlDer    = iw( 71)
      nConfd    = iw(183)
      nObjfd    = iw(184)

      nGotg(1)  = iw(186)
      nGotg(2)  = iw(187)

*     Pointers

      laScal    = iw(274)
      lrhs      = iw(290)

*     Constants

      eps       = rw(  1)
      eps0      = rw(  2)
      eps1      = rw(  3)
      eps5      = rw(  7)

      tolFP     = rw( 51)
      tolQP     = rw( 52)
      tolNLP    = rw( 53)
      tolx      = rw( 56)
      tolCon    = rw( 57)
      tolpiv    = rw( 60)

      bigfx     = rw( 71)
      bigdx     = rw( 72)
      epsrf     = rw( 73)
      fdint1    = rw( 76)

      xdlim     = rw( 80)
      xPen0     = rw( 89)
      sclObj    = rw(188)
                  
      nlnCon = nnCon  .gt. 0
      nlnObj = nnObj  .gt. 0

*     ------------------------------------------------------------------
*     s8core operates in either ``Normal'' or ``Elastic'' mode.
*     In elastic mode, the nonlinear slacks are allowed to be infeasible
*     while a weighted sum of the slack infeasibilities is minimized.
*     ------------------------------------------------------------------
      feaSlk = .true.
*     Elastc =       FP  .and.  nlnCon
      Elastc = .false.

      nInf   = 0
      sInf   = zero

      iReset = 0
      iSkip  = 0
      ierror = 0
      LUreq  = 0
      nState = 0
      nHx    = 0

      rviol  = zero
      prInf  = zero
      duInf  = zero

      gMrt   = zero
      step   = zero

      sgnObj = minimz

      KTcond(1) =  .false.
      KTcond(2) =  .false.

      done   = .false.
      frstQP = .true.
      QPpi0  = .false.      ! Use zero initial multipliers
*     QPpi0  = .true.       ! Use QP initial multipliers

      useFD  = lvlDer .eq. 0  .or.  lvlDer .eq. 2
     $                        .or. (lvlDer .eq. 1  .and.  nnCon .gt. 0)
      usefLS = useFD          .or.  lvlSch .eq. 0

      if (MjrPrt .ge. 10  .or.  MnrPrt .ge. 1) then
         if (iPrint .gt. 0) then
            write(iPrint, 1010) (line, j=1,29), nMajor
         end if
         if (iSumm  .gt. 0  .and.  MnrPrt .ge. 1) then
            write(iSumm , 1020) (line, j=1,19), nMajor
         end if
      end if

      MjrMsg = '      '

      if (nnL .gt. 0) then
         gNorm  = one
         if (nlnObj      ) gNorm = gNorm + dnrm1s( nnObj, gObj, 1 )
         if (iObj  .gt. 0) gNorm = gNorm + sclObj
      end if

      if ( nlnCon ) then
*        ---------------------------------------------
*        Initialize the penalty parameters.
*        Set an initial elastic weight.
*        ---------------------------------------------
         incRun = .true.
         PenDmp = one
         PenMax = one / eps
         PenNrm = xPen0
         call dload ( nnCon, xPen0, xPen, 1 )
         call dload ( nnCon, zero , xMul, 1 )

         if ( nlnCon ) then
            call s8wInf( 'Set wtInf',
     $                   boostd, itn, gNorm, wtInf, wtInf0,
     $                   weight, wtFac, wtScal, iw, leniw )
         else
            wtInf = wtInf0
         end if
      end if

      if (nnL .gt. 0) then
         nReset    = nMajor
         call s8H0  ( MjrMsg(2:2), nnL, iw, leniw, rw, lenrw )
      end if

      call dcopy ( nb, xs, 1, x2, 1 )
      cdItns = -1
      newg   = .false.

**    ======================Start of main loop==========================
*     Start of a Major Iteration.
*     ==================================================================
*+    do while (.not. done  .and.  ierror .eq. 0)
  100 if       (.not. done  .and.  ierror .eq. 0) then

         nMinor = 0

*        ===============================================================
*        Repeat                    (until an accurate gradient is found)

  110       centrl = iw(lvlDif) .eq. 2

            if ( newg ) then 
               if ( useFD ) then
*                 ------------------------------------------------------
*                 Compute any missing derivatives.
*                 ------------------------------------------------------
                  call s6fd  ( ierror,
     $                         n, neJac, nnCon, nnCon0, nnJac, nnObj,
     $                         sfgCon, fgCon, sfgObj, fgObj,
     $                         ne, nka, ha, ka, fObj, xs, w, 
     $                         cu, lencu, iu, leniu, ru, lenru, 
     $                         cw, lencw, iw, leniw, rw, lenrw )
                  if (ierror  .ne. 0) go to 100 ! Break
               end if ! useFD
               newg = .false.
            end if

            if ( nlnCon ) then
*              ---------------------------------------------------------
*              Load the scaled Jacobian in A.
*              Compute the QP right-hand side   rhs  =  Jx - fCon.
*              Find the nonlinear constraint violations.
*              We first find the nonlinear slacks  s  that minimize the
*              merit function with  xs(1:n)  and  xMul  held fixed.
*              The optimal slacks are loaded into  xs(n+1:nb)  and the
*              violations  viol = fCon + A(linear)x - nonlinear slacks
*              are defined.
*              ---------------------------------------------------------
               call s8setJ( nb, nnCon, nnJac, neJac, eps0,
     $                      ne, nka, a, ha, ka, 
     $                      xs, iw, leniw, rw, lenrw )
               call s8viol( Elastc, n, nnCon, eps0, wtInf,
     $                      bl, bu, viol, xs, xMul, xPen, yslk )
            end if

*           ------------------------------------------------------------
*           Factorize the basis at xs.
*           Compute x2 such that (A -I)*x2 = rhs.
*           ------------------------------------------------------------
            if ( frstQP ) then
*              ---------------------------------------------------------
*              First QP subproblem.
*              ---------------------------------------------------------
*              To avoid an unnecessarily ill-conditioned starting basis
*              for the first QP, use the BS factorization with big
*              singularity tols.
*              Allow an extra  m  minor iterations if the first QP 
*              starts with an an all-slack basis.

               needLU  = .true.
               gotR    = .false.
               nSwap   = 0

               Utol1s     = rw(Utol1)
               Utol2s     = rw(Utol2)
               rw(Utol1)  = max( Utol1s, eps5 )
               rw(Utol2)  = max( Utol2s, eps5 )
               typeLU     = 'BS' 
               MnrSv      = iw(mMinor)
               if (iCrash .eq. 0) then
                  iw(mMinor) = iw(mMinor) + m
               end if
            else
*              ---------------------------------------------------------
*              Subsequent factorizations.
*              ---------------------------------------------------------
*              For linearly constrained problems, the factors L, U and R
*              can be saved as long as a poor x does not force a
*              new factorization. (Even in this case, R can be saved if
*              there are no swaps.)

               needLU = nlnCon      
               typeLU = 'BT'
            end if

            call s2Bfac( typeLU, needLU, newLU, newB, 
     $                   ierror, iObj, itn, MjrPrt, LUreq,
     $                   m, mBS, n, nb, nnL, nS, nSwap, xNorm,
     $                   ne, nka, a, ha, ka, 
     $                   kBS, hs, rw(lrhs), nnCon,
     $                   bl, bu, blBS, buBS, xBS, x2, 
     $                   iy, iy2, y, y2, 
     $                   iw, leniw, rw, lenrw )
            if (ierror  .ne. 0) go to 100 ! Break

            gotR    = gotR  .and.  .not. newB
            needLU  = .false.
            if (MjrPrt .ge. 10) iw(MjrHdg) = 1

            if ( frstQP ) then
               rw(Utol1) = Utol1s 
               rw(Utol2) = Utol2s
            end if

*           ------------------------------------------------------------
*           Solve the QP subproblem to obtain kBS, x2 and pi.
*           The search direction will be xdif = x2 - xs.
*           ------------------------------------------------------------
  300       call s8iqp ( MjrMsg(2:2), MjrMsg(5:6), Elastc, gotR,
     $                   ierror, itn, lenr, m, maxS, mBS,
     $                   n, nb, nnL, nnL0, nnCon, nnObj, nnObj0,
     $                   nS, nHx, iMinor, nDegen, MjrPrt, MnrPrt,
     $                   minimz, (ObjAdd+fObj), fObjQP, iObj, 
     $                   tolFP, tolQP, tolx,
     $                   nInf2, sInf2, wtInf, piNorm, xNorm,
     $                   ne, nka, a, ha, ka,
     $                   hElast, hEstat, hfeas, hs, kBS, 
     $                   bl, bu, blBS, buBS,
     $                   gObj, pi, r, rc, 
     $                   xs, x2, xBS, xdif, xsfeas,
     $                   iy, iy2, w, y, y1, y2,
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

            nMinor = nMinor + iMinor
            if (ierror .ge. 10) go to 100

            if ( frstQP ) then
               iw(mMinor) = MnrSv
               frstQP     = .false.
            end if

            if ( nlnCon ) then
               if ( QPpi0 ) then
                  call dcopy ( nnCon,            pi, 1,  xMul, 1 )
                  call dload ( nnCon, (zero), xdMul, 1 )
               else
                  call dcopy ( nnCon,            pi, 1, xdMul, 1 )
                  call daxpy ( nnCon, (-one),  xMul, 1, xdMul, 1 )
               end if

               if (Elastc  .and.  feaSlk) then
                  call dcopy ( nnCon, yslk, 1, xs(n+1), 1 )
               end if

*              If xMul or xs  changed, recompute viol.

               if (QPpi0  .or.  (Elastc  .and.  feaSlk)) then
                  call s8viol( Elastc, n, nnCon, eps0, wtInf,
     $                         bl, bu, viol, xs, xMul, xPen, yslk )
                  if (Elastc  .and.  feaSlk) feaSlk = .false.
                  if (QPpi0                ) QPpi0  = .false.
              end if

*              Find the sum of infeasibilities of the nonlinear slacks.

               call s8sInf( n, nb, nnCon, tolx, nInf, sInf, bl, bu, xs )
            end if

            call dcopy ( nb,         x2, 1, xdif, 1 )
            call daxpy ( nb, (-one), xs, 1, xdif, 1 )

            maxItn = ierror .eq. 3  .and.  itn .ge. itnlim
            maxMjr = nMajor .ge. mMajor
            maxMnr = nMinor .ge. iw(mMinor)
            maxnS  = ierror .eq. 5

            if ( maxMjr ) ierror = 3
            nBS    = m + nS
            xNorm  = dnrm1s( nb, xs  , 1 )
            xNrm   = dnrm1s( n , xs  , 1 )
            xdnorm = dnrm1s( nb, xdif, 1 )
            xdnrm  = dnrm1s( n , xdif, 1 )

*           Compute the maximum dual infeasibility.
*           Use the QP multipliers for the reduced costs.

            call s8rc  ( sclObj, minimz, iObj,
     $                   m, n, nb, 
     $                   nnObj, nnObj0, nnCon, nnJac, neJac,
     $                   ne, nka, a, ha, ka,
     $                   gObj, gCon, pi, rc )
            call s8Infs( Elastc, n, nb, nnCon, nnCon0, wtInf,
     $                   prInf, duInf, jprInf, jduInf,
     $                   bl, bu, rc, xs, yslk )

*           Compute the largest nonlinear row violation.

            if ( nlnCon ) then
               jrviol = idamax( nnCon, viol, 1 ) 
               rviol  = abs( viol(jrviol) )
            end if

*           ------------------------------------------------------------
*           Test for convergence.
*           ------------------------------------------------------------
            call s6Rcnd( nS, lenr, r, dRzmax, dRzmin, Rzmax, condHz )
            rviol     = rviol /(one + xNorm)
            prInf     = prInf /(one + xNorm)
            duInf     = duInf /(one + piNorm)

            rowFea    = rviol  .lt. tolCon  .and.  nInf .gt. 0
            KTcond(1) = prInf  .le. tolCon
            KTcond(2) = duInf  .lt. tolNLP

            optiml    = (KTcond(1)  .or.  rowFea)  .and.  KTcond(2)

            if (nlnCon  .and.  optiml  .and.  nInf .gt. 0) then
               call s8wInf( 'Boost wtInf',
     $                      boostd, itn, gNorm, wtInf, wtInf0,
     $                      weight, wtFac, wtScal, iw, leniw )
               if ( boostd ) then
                  Elastc = .true.
                  go to 300
               end if
            end if

*           ------------------------------------------------------------
*           Compute the current augmented Lagrangian merit function.
*           ------------------------------------------------------------
            if (iObj .eq. 0) then
               fMrt = zero
            else
               fMrt = sgnObj*xs(n+iObj)*sclObj
            end if

            if ( nlnObj ) then
               fMrt =  fMrt + sgnObj*fObj
            end if

            if ( nlnCon ) then 
               call dcopy ( nnCon, viol, 1, y, 1 )
               call ddscl ( nnCon, xPen, 1, y, 1 )
               fMrt = fMrt -      ddot  ( nnCon, xMul, 1, viol, 1 )
     $                     + half*ddot  ( nnCon,    y, 1, viol, 1 )

               if ( Elastc ) then
                  fMrt = fMrt + wtInf*sInf
               end if
            end if

*           ------------------------------------------------------------
*           If the forward-difference estimate of the reduced
*           gradient of the Lagrangian is small,  switch to
*           central differences, recompute the derivatives and solve
*           the QP again.  If central differences give a large
*           reduced-gradient norm, switch back to forward differences.
*           Check that we didn't just switch to central differences.
*           ------------------------------------------------------------
            cNorm  = zero
            if (nlnCon) cNorm  = dnrm1s( nnCon, fCon, 1 )
            ObjSiz = zero
            if (nlnObj) ObjSiz = abs(fObj)

            goodg  = .true.
            gLtest = (one + ObjSiz + cNorm)*epsrf/fdint1
            gLnorm = duInf

            if ( centrl ) then
               if (gLnorm .gt. ten*gLtest  .and.  cdItns .gt. 0) then
                  iw(lvlDif)     =  1
                  centrl         = .false.
                  if ( useFD ) then
                     MjrMsg(4:4) = ' '
                  end if
               end if
            else
               if (gLnorm .le.     gLtest) then
                  cdItns         = 0
                  iw(lvlDif)     = 2
                  if ( useFD ) then
                     goodg       = .false.
                     newg        = .true.
                     MjrMsg(4:4) = 'c'
                     if (iPrint .gt. 0) write(iPrint, 3010) itn
                     if (iSumm  .gt. 0) write(iSumm , 3010) itn
                  end if
               end if
            end if

*           ------------------------------------------------------------
*           Print the details of this iteration.
*           Call user-supplied monitor s8User.
*           ------------------------------------------------------------
            call s8log ( MjrMsg, KTcond, MjrPrt, minimz,
     $                   n, nb, nnCon0, nS,
     $                   nMajor, nMinor, nSwap,
     $                   condHz, iObj, ObjAdd, fMrt, PenNrm, step,
     $                   prInf, duInf, vimax, virel,
     $                   ne, nka, a, ha, ka,
     $                   hs, rw(laScal), bl, bu, fCon, xMul, xs,
     $                   iw, leniw, rw, lenrw )

            iAbort = 0
            call s1User( iAbort, MjrMsg, KTcond, 
     $                   m, n, nb, nS, nS, nMajor, nMinor, nSwap,
     $                   condHz, duInf, dummy, fObj, 
     $                   fMrt, gMrt, PenNrm, prInf, step,
     $                   vimax, dummy, dummy, 
     $                   ne, nka, a, ha, ka,
     $                   hs, bl, bu, pi, rc, xs, 
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

            if (iAbort .ne. 0) then
               ierror = 12
               go to 100 
            end if

*+       until (.not. (useFD  .and.  .not.goodg))
         if (          useFD  .and.  .not.goodg ) go to 110
*        ===============================================================
         done = optiml  .or.  maxnS  .or.  maxItn  .or.  maxMjr
         if ( done ) go to 100

         MjrMsg = '      '
         step   = zero
         nSwap  = 0
         ierror = 0

*        ---------------------------------------------------------------
*        Compute  ptHp = xdif' H xdif and other directional derivatives.
*        Be prepared to fix up pHpMrt if there are linear variables.
*        ---------------------------------------------------------------
         ptHp = zero
         if (nnL .gt. 0) then
            call s8Hx  ( s8HDum, minimz, n, nnL, ne, nka, a, ha, ka, 
     $                   xdif, Hp, nState, 
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )
            nHx    = nHx + 1
            if (minimz .lt. 0) then
               call dscal ( nnL, sgnObj, Hp, 1 )
            end if
            ptHp   = max( zero, ddot  ( nnL, xdif, 1, Hp, 1 ) )
         end if

         if (nnL .eq. n) then
            if (ptHp .le. zero  .and.  nReset .ne. nMajor) then
               nReset = nMajor
               call s8H0  ( MjrMsg(2:2), nnL, iw, leniw, rw, lenrw )
               go to 100
            end if
            pHpMrt = ptHp
         else
            pHpMrt = max( eps1*xdnrm*xdnrm, ptHp )
         end if

         if (iObj .eq. 0) then
            f   = zero
            gdx = zero
         else
            f   = sgnObj*xs  (n+iObj)*sclObj
            gdx = sgnObj*xdif(n+iObj)*sclObj
         end if
            
         if ( nlnObj ) then
            f   = f   + sgnObj*fObj
            call dcopy ( nnObj, gObj, 1, gObj1, 1 )
            gdx = gdx + sgnObj*ddot  ( nnObj, gObj1, 1, xdif, 1 )
         end if

         if ( Elastc ) then
            f    = f     + sInf*wtInf
            gInf = sInf2 - sInf
            gdx  = gdx   + gInf*wtInf
         end if

*        ---------------------------------------------------------------
*        Compute the search direction for the multipliers and nonlinear
*        slacks, and the contributions to the merit function and its
*        directional derivative from the nonlinear constraints.
*        The penalty parameters  xPen(j)  are increased if the
*        directional derivative is not sufficiently negative.
*        ---------------------------------------------------------------
         fMrt   = f
         gMrt   = gdx

         if ( nlnCon ) then
            call s8mrt ( nnCon, fMrt, gMrt, pHpMrt, 
     $                   incRun, penDmp, penMax, PenNrm,
     $                   viol, xMul, xdMul, xPen, y, rw, lenrw )
            call dcopy ( nnCon, fCon, 1, fCon1, 1 )
            call dcopy ( nnCon, pi  , 1, xMul2, 1 )
         end if

*        ---------------------------------------------------------------
*        Prepare for the linesearch to find a better point
*           x2 = xs + step*dx  and  xMul2 = xMul + step*dxMul.
*        where, on entry,  x2 = xQP and  xMul = pi.
*
*        switch  indicates if there is an option to switch to
*                central differences to get a better search direction.
*        stepqp  is the step predicted by the QP subproblem (usually 1).
*        stepmx  is the largest feasible steplength subject to a
*                user-defined limit, bigdx, on the change in  x.
*        step    is initialized subject to a user-defined limit, xdlim.
*        ---------------------------------------------------------------
         debug  = nMajor .ge. lprSch
         switch = .not. centrl     .and.
     $            ((nObjfd .gt. 0  .and.  nGotg(1) .lt. nnObj)  .or.
     $             (nConfd .gt. 0  .and.  nGotg(2) .lt. neJac)      )

         stepmn = zero
         if (usefLS  .and.  switch) then
            stepmn = fdint1*(one + xNrm) / xdnrm
         end if

         stepQP = one
         if ( nlnCon ) then
            stepmx = one
         else
            tolp   = tolpiv*xdnorm
            stepmx = ddiv  ( bigdx, xdnorm, overfl )
            call s8step( nb, stepmx, stepQP, tolp, eps0,
     $                   bl, bu, xs, xdif, step )
            stepmx = step
         end if

         steplm = ddiv( (one+xNorm)*xdlim, xdnorm, overfl )
         stepmx = min (            steplm, stepmx)
         step   = min (            steplm, one )

*        ===============================================================
*        Use a linesearch to obtain  x2 = xs + step*xdif.
*
*        fCon ,  gCon  and  gObj  contain the new problem functions.
*        fCon2,  gCon2 and  gObj2 are temporary arrays.  
*        fCon1,  a     and  gObj1 contain the old problem functions
*
*        s6srch returns the following values:
*
*        inform =-1 (and ierror = 6) if the user wants to stop.
*        inform = 1 if the search is successful and  step < stepmx.
*               = 2 if the search is successful and  step = stepmx.
*               = 3 if a better point was found but too many functions
*                   were needed (not sufficient decrease).
*               = 4 if stepmx < tolabs (too small to do a search).
*               = 5 if step   < alfsml (srchq only -- maybe switch
*                   to central differences to get a better direction).
*               = 6 if the search found that there is no useful step.
*                   The interval of uncertainty is less than 2*tolabs.
*                   The minimizer is very close to step = zero
*                   or the gradients are not sufficiently accurate.
*               = 7 if there were too many function calls.
*               = 8 if the input parameters were bad
*                   (stepmx le toltny  or  uphill).
*               = 9 if the objective is unbounded below.
*        ===============================================================
         fMrt0  = fMrt
         gMrt0  = gMrt
         call dcopy ( nb, x2, 1, y1, 1 )

  500    call s6srch( inform, debug, Elastc, usefLS, minimz,
     $                n, nb, nnCon, nnCon0, nnObj, nnObj0, iObj, itn,
     $                sfgCon, fgCon, sfgObj, fgObj,
     $                ne   , nka, a, ha, ka,
     $                neJac, nkg,        kg, 
     $                fMrt, gMrt, gInf, sInf, sInf2, wtInf,
     $                step, stepmn, stepmx, xdnorm, xNorm, fObj,  
     $                fCon, gCon, gObj, fCon2, gCon2, gObj2,
     $                xs, x2, xdif, xMul, xMul2, xdMul, xPen,
     $                w, y, y2,
     $                cu, lencu, iu, leniu, ru, lenru, 
     $                cw, lencw, iw, leniw, rw, lenrw )
         if (inform .lt. 0) return

         if (step .ge. steplm) then
            MjrMsg(3:3) = 'l'
         end if

         if (inform .ge. 4) then
*           ------------------------------------------------------------
*           The line search failed to find a better point.
*           For exact gradients or central differences, stop.
*           Otherwise, switch to central differences and resolve the QP.
*           ------------------------------------------------------------
            if (useFD  .and.  .not. centrl) then
               cdItns      = 0
               MjrMsg(4:4) = 'c'
               if (iPrint .gt. 0) write(iPrint, 3020) itn
               if (iSumm  .gt. 0) write(iSumm , 3020) itn
               iw(lvlDif)  = 2
               newg        = .true.

            else if (KTcond(1)  .and.  abs(fObj) .ge. bigfx/ten) then
*              ------------------------------
*              The problem is unbounded.
*              ------------------------------
               ierror = 2

            else if (iReset .lt. mReset  .and.  nnL .gt. 0) then
*              ------------------------------
*              Reset the approximate Hessian.
*              ------------------------------
               nReset = nMajor
               if (iw(nQNmod) .gt. 0) then
                  iReset = 0
               else
                  iReset = iReset + 1
               end if
               call s8H0  ( MjrMsg(2:2), nnL, iw, leniw, rw, lenrw )
               incRun = .true.
               PenDmp = one
               PenMax = one / eps
               PenNrm = xPen0
               call dload ( nnCon, xPen0, xPen, 1 )
               ierror = 0
            else if (KTcond(1)  .and.  duInf .lt. hundrd*tolNLP) then
               ierror = 4
            else if ( maxMnr ) then
               ierror = 3
            else if (MjrMsg(3:3) .eq. 'd') then
               if ( KTcond(1) ) then
                  ierror = 2
               else if ( nlnCon ) then
                  call s8wInf( 'Boost wtInf',
     $                         boostd, itn, gNorm, wtInf, wtInf0,
     $                         weight, wtFac, wtScal, iw, leniw )

                  if ( boostd ) then
                     ierror = 0
                     Elastc = .true.
                     go to 300
                  end if
               end if
            else if (inform .eq. 9) then
               ierror = 2
            else
               ierror = 9
               if (iPrint .gt. 0)
     $            write(iPrint, 1050) inform, msg(inform), nMajor, duInf
               if (iSumm  .gt. 0)
     $            write(iSumm , 1050) inform, msg(inform), nMajor, duInf
            end if
            go to 100
         end if ! inform ge 4

         inform = 0
         if ( nlnCon ) then
            call s8nslk( n, nnCon, nnJac, eps0,
     $                   maxvi, vimax, virel, xNorm,
     $                   ne, nka, a, ha, ka,
     $                   bl, bu, fCon, x2, yslk )
         
*           ------------------------------------------------------------
*           If the max violation is bigger than viSup, redo the line 
*           search with a smaller stepmx.
*           ------------------------------------------------------------
            back   = 0.1d+0

            if (vimax .gt. viSup) then
               MjrMsg(3:3) = 'd'
               stepmx = back * step
               step   = stepmx
               fMrt   = fMrt0
               gMrt   = gMrt0
               go to 500
            end if ! vimax .gt. viSup
         end if ! nlnCon

         if ( usefLS )  then
*           =========================================================
*           A function-only line search was used.
*           Compute the new gObj and gCon.
*           =========================================================
*           Compute known derivatives at the new x.
*           Use dummy fCon and fObj to avoid trashing the new values.

            modefg = 1
            if ( nlnCon ) then
               if (lvlSch .eq. 0  .or.
     $             lvlDer .eq. 0  .or.  lvlDer .eq. 1) then
                  call sfgCon( modefg, ierror, nState,
     $                         n, nnCon, nnJac, neJac, fgCon,
     $                         ne, nka, ha, ka,
     $                         fCon2, gCon, x2,
     $                         cu, lencu, iu, leniu, ru, lenru, 
     $                         cw, lencw, iw, leniw, rw, lenrw )
                  if (ierror  .ne. 0) go to 100 ! Break
               end if
            end if

            if ( nlnObj ) then
               if (lvlSch .eq. 0  .or.
     $             lvlDer .eq. 0  .or.  lvlDer .eq. 2) then
                  call sfgObj( modefg, 
     $                         ierror, nState, nnObj,
     $                         fgObj,
     $                         fObj2, gObj, x2,
     $                         cu, lencu, iu, leniu, ru, lenru, 
     $                         cw, lencw, iw, leniw, rw, lenrw )
                  if (ierror  .ne. 0) go to 100 ! Break
               end if
            end if
*           ------------------------------------------------------------
*           Compute any missing derivatives.
*           ------------------------------------------------------------
            if ( useFD ) then
               call s6fd  ( ierror,
     $                      n, neJac, nnCon, nnCon0, nnJac, nnObj,
     $                      sfgCon, fgCon, sfgObj, fgObj,
     $                      ne, nka, ha, ka, fObj, x2, w,
     $                      cu, lencu, iu, leniu, ru, lenru, 
     $                      cw, lencw, iw, leniw, rw, lenrw )
               if (ierror  .ne. 0) go to 100 ! Break
            end if
         end if

         inform = 0
         nMajor = nMajor + 1
         if ( centrl )
     $   cdItns = cdItns + 1

         if (MjrPrt .ge. 10  .or.  MnrPrt .ge. 1) then
            if (iPrint .gt. 0) then
               call s1page( 0, iw, leniw )
               write(iPrint, 1010) (line, j=1,29), nMajor
            end if
            if (iSumm  .gt. 0  .and.  MnrPrt .ge. 1) then
               write(iSumm , 1020) (line, j=1,19), nMajor
            end if
         end if

*        ===============================================================
*        The problem functions have been defined at the new x.
*        ===============================================================
         if (nnL .gt. 0) then
            gNorm  = one
            if (nlnObj      ) gNorm  = gNorm + dnrm1s( nnObj, gObj, 1 )
            if (iObj  .gt. 0) gNorm  = gNorm + sclObj
         end if

*        ---------------------------------------------------------------
*        In normal mode, compute a new scale for the infeasibilities.
*        ---------------------------------------------------------------
         if (.not. Elastc  .and.  nlnCon) then
            call s8wInf( 'Set wtInf', 
     $                   boostd, itn, gNorm, wtInf, wtInf0,
     $                   weight, wtFac, wtScal, iw, leniw )
         end if

         MjrMsg(1:1) = ' '
         ytp         = zero

         if (nnL .gt. 0) then
*           ============================================================
*           Define the BFGS update.
*           ============================================================
*           Place a copy of the old Jacobian in gCon2.
*           It is used to define the BFGS update in s8Hcur.

            if ( nlnCon ) then
               call s8Jcpy( nnCon, nnJac, 
     $                      ne   , nka, ha   , ka, 
     $                      ne   , nka, a    , ka,
     $                      neJac, nkg, gCon2, kg ) 
            end if

*           Compute gdif = gL2 - gL1.

            call s8Hcur( minimz, m,
     $                   nnCon, nnObj, nnObj0, nnJac, nnL, eps0,
     $                   ne, nka, ha, ka,
     $                   neJac, nkg, kg, 
     $                   ytp, gObj1, gObj, gCon2, gCon,
     $                   xdif, gdif, xMul2 )

            ytpmin = tolg*ptHp       ! The smallest allowable curvature.
            updatd = ytp .ge. ytpmin  .and.  ptHp .gt. zero

            if (nlnCon  .and. .not. updatd) then
*              ---------------------------------------------------------
*              Redefine  xdif  and  gdif  by redefining xs.  The arrays
*              fCon2, gCon2 and gObj2 are computed at the new xs.
*              The old xs is saved in case the problems are undefined
*              at the new xs.
*              ---------------------------------------------------------
               call dcopy ( nb, xs, 1, w , 1 )
               call s8x1  ( ierror, n, nb,
     $                      neJac, nnCon, nnJac, nnObj, nnObj0,
     $                      sfgCon, fgCon, sfgObj, fgObj,
     $                      ne, nka, ha, ka,
     $                      step, fCon2, gCon2, gObj2,
     $                      xs, xdif, xsfeas,
     $                      cu, lencu, iu, leniu, ru, lenru, 
     $                      cw, lencw, iw, leniw, rw, lenrw )

               if      (ierror .ge. 0) then

*                 No problem.  Copy the new derivatives
*                 into fCon1, gObj1 (gCon2 has already been set).

                  call dcopy ( nnCon, fCon2, 1, fCon1, 1 )
                  if (nnObj .gt. 0)
     $            call dcopy ( nnObj, gObj2, 1, gObj1, 1 )

               else if (ierror .eq. -1) then

*                 Although the reomputed base point xs must be feasible
*                 for the nonelastics, the user has indicated that some
*                 problem functions are not defined there.  We have no 
*                 choice but to abandon the modification, in which case
*                 we reset the old xs and recopy the old Jacobian from
*                 the matrix A.  Bummer!

                  call dcopy ( nb, w , 1, xs, 1 )
                  call s8Jcpy( nnCon, nnJac, 
     $                         ne   , nka, ha   , ka, 
     $                         ne   , nka, a    , ka,
     $                         neJac, nkg, gCon2, kg ) 
                  call dcopy ( nb,         x2, 1, xdif, 1 )
                  call daxpy ( nb, (-one), xs, 1, xdif, 1 )
                  ierror = 0

               else if (ierror .eq.  6) then

*                 The user wants to stop.

                  go to 100

               end if

*              ---------------------------------------------------------
*              Redefine the minimum curvature.
*              This value of ptHp can be zero.
*              ---------------------------------------------------------
               nState = 0
               call s8Hx  ( s8HDum, minimz, n, nnL, ne, nka, a, ha, ka, 
     $                      xdif, Hp, nState, 
     $                      cu, lencu, iu, leniu, ru, lenru, 
     $                      cw, lencw, iw, leniw, rw, lenrw )
               nHx    = nHx + 1
               if (minimz .lt. 0) then
                  call dscal ( nnL, sgnObj, Hp, 1 )
               end if

               call s8Hcur( minimz, m,
     $                      nnCon, nnObj, nnObj0, nnJac, nnL, eps0,
     $                      ne, nka, ha, ka,
     $                      neJac, nkg, kg, 
     $                      ytp, gObj1, gObj, gCon2, gCon,
     $                      xdif, gdif, xMul2 )
               ptHp2  = max( zero, ddot  ( nnL, xdif, 1, Hp, 1 ) )
               ytpmin = max( tolg*ptHp2, tolg2*ptHp, eps )
               updatd = ytp .ge. ytpmin  .and.  ptHp2 .gt. zero

               if ( updatd ) then
                  MjrMsg(1:1) = 'M'
                  ptHp        = ptHp2
               end if

               if (.not. updatd  .and.  ptHp2 .gt. ytpmin) then
*                 ------------------------------------------------------
*                 If all else fails, try adding an augmented Lagrangian
*                 term to increase ytp. 
*                 ------------------------------------------------------
                  call s8Hfix( nnCon, nnJac, eps0,
     $                         ne, nka, ha, ka,
     $                         neJac, nkg, kg, 
     $                         ytp, ytpmin, PenUnm, step, 
     $                         fCon1, fCon, gCon2, gCon,
     $                         xdif, gdif, y, y2, w )

                  updatd = ytp .ge. ytpmin

                  if ( updatd ) then
                     MjrMsg(1:1) = 'm'
                     ptHp        = ptHp2
                  end if
               end if
            end if ! nlnCon

            if ( updatd ) then
               iSkip = 0
            else
               MjrMsg(1:1) = 'n'
               iSkip       = iSkip + 1
               if (iSkip .ge. mSkip) then
*                 ------------------------------------------------------
*                 Reinitialize the multipliers and penalty parameters.
*                 ------------------------------------------------------
                  incRun = .true.
                  PenDmp = one
                  PenMax = one / eps
                  call dload ( nnCon, xPen0, xPen , 1 )
                  call dload ( nnCon, zero , xMul2, 1 )
                  iSkip  = 0
               end if
            end if

*           ------------------------------------------------------------
*           Store the modification associated with the new update.
*           ------------------------------------------------------------
            call s8Hmod( MjrMsg(1:1), MjrMsg(2:2), nnL, nnCon,
     $                   step, ytp, ytpmin, ptHp, Hp, gdif,
     $                   iw, leniw, rw, lenrw )

            gotR = nS .gt. 0 .and. nnCon .eq. 0 .and. iw(nQNmod) .gt. 0

            if (gotR  .and.  updatd) then 
               call s8Rupd( MjrMsg(1:1), lenr, m, n, nBS, nnL, nS,
     $                      ytp*step, ptHp,
     $                      ne, nka, a, ha, ka,
     $                      kBS, gdif, Hp, r, w, y, y2,
     $                      iw, leniw, rw, lenrw )
            end if
         end if

*        ---------------------------------------------------------------
*        Update the variables.
*        The QP point, saved in y1, is used to start the next QP.
*        (If a unit step was not taken last iteration, some more
*        nonbasics may be between their bounds.
*        Nov 10, 1994. Tried leaving the nonbasics between their
*        bounds after short step. In some cases, the number of minor
*        iterations increased dramatically with a very short step.)
*        ---------------------------------------------------------------
         call dcopy ( nb, x2, 1, xs, 1 )
         call dcopy ( nb, y1, 1, x2, 1 )

         if ( nlnCon )
     $   call dcopy ( nnCon, xMul2, 1, xMul, 1 )
         sInf = sInf2
         nInf = nInf2

         go to 100
*+    end while
      end if
*     ======================end of main loop============================
*     Exit.

      if (ierror .eq. 0) then
*        ===============================================================
*        Optimal.
*        ===============================================================
         call s1page( 1, iw, leniw )

         if (nInf .eq. 0) then
*           ----------------------------------------
*           Optimal and feasible.
*           ----------------------------------------
            if ( FP ) then
               if (iPrint .gt. 0) write(iPrint, 9001)
               if (iSumm  .gt. 0) write(iSumm , 9001)
            else
               if (iPrint .gt. 0) write(iPrint, 9000)
               if (iSumm  .gt. 0) write(iSumm , 9000)
            end if
         else
*           ----------------------------------------
*           Optimal but Infeasible.
*           ----------------------------------------
            ierror = 1

            if (iPrint .gt. 0) write(iPrint, 9010)
            if (iSumm  .gt. 0) write(iSumm , 9010)
         end if

         if (duInf .gt. 0.1d+0) then 
            if (iPrint .gt. 0) write(iPrint, 9005)
            if (iSumm  .gt. 0) write(iSumm , 9005)
         end if

      else
*        ===============================================================
*        An error flag was set.
*        ===============================================================
         call s1page( 2, iw, leniw )

         if (ierror .lt. 0) then
*           -------------------------------------------------
*           Undefined functions (same as ierror = 6).
*           -------------------------------------------------
            ierror = 6
            if (iPrint .gt. 0) write(iPrint, 9060)
            if (iSumm  .gt. 0) write(iSumm , 9060)

         else if (ierror .eq. 2) then
*           -------------------------------------------------
*           Unbounded.
*           -------------------------------------------------
            if (abs(fObj) .gt. bigfx/ten  .or.  KTcond(1)) then
               if (iPrint .gt. 0) write(iPrint, 9020)
               if (iSumm  .gt. 0) write(iSumm , 9020)
            else
               if (iPrint .gt. 0) write(iPrint, 9021)
               if (iSumm  .gt. 0) write(iSumm , 9021)
            end if
        
         else if (ierror .eq. 3) then
*           -------------------------------------------------
*           Too many iterations.
*           -------------------------------------------------
            if      (nMajor .ge. mMajor) then 
               if (iPrint .gt. 0) write(iPrint, 9031)
               if (iSumm  .gt. 0) write(iSumm , 9031)

            else if (nMinor .ge. iw(mMinor)) then
               if (iPrint .gt. 0) write(iPrint, 9032)
               if (iSumm  .gt. 0) write(iSumm , 9032)

            else if (itn    .ge. itnlim) then
               if (iPrint .gt. 0) write(iPrint, 9033)
               if (iSumm  .gt. 0) write(iSumm , 9033)
            end if

         else if (ierror .eq. 4) then
*           -------------------------------------------------
*           Requested accuracy could not be achieved.
*           -------------------------------------------------
            if (iPrint .gt. 0) write(iPrint, 9040)
            if (iSumm  .gt. 0) write(iSumm , 9040)

         else if (ierror .eq. 5) then
*           -------------------------------------------------
*           Superbasic limit too small.
*           -------------------------------------------------
            if (iPrint .gt. 0) write(iPrint, 9050) maxS
            if (iSumm  .gt. 0) write(iSumm , 9050) maxS

         else if (ierror .eq. 9) then
*           -------------------------------------------------
*           Line search failure.
*           -------------------------------------------------
            if (iPrint .gt. 0) write(iPrint, 9090)
            if (iSumm  .gt. 0) write(iSumm , 9090)

         else if (ierror .eq. 12) then
*           -------------------------------------------------
*           User termination via s1User.
*           -------------------------------------------------
            if (iPrint .gt. 0) write(iPrint, 9120)
            if (iSumm  .gt. 0) write(iSumm , 9120)
         end if
      end if

      return

 1000 format(' ')
 1010 format(  1x, 29a4 / ' Start of major itn', i6)
 1020 format(/ 1x, 19a4 / ' Start of major itn', i6)
 1050 format(  ' Search exit', i3, ' -- ', a,
     $         '   Itn =', i7, '  Dual Inf =', 1p, e11.3)
 3010 format( ' Itn', i7, ' -- Central differences invoked.',
     $       '  Small reduced gradient.' )
 3020 format( ' Itn', i7, ' -- Central differences invoked.',
     $       '  Small step length.' )

 9000 format(  ' EXIT -- optimal solution found')
 9001 format(  ' EXIT -- feasible point found')
 9005 format(/ ' XXX  WARNING -- reduced gradient is large --',
     $         ' solution is not really optimal.')
 9010 format(  ' EXIT -- infeasible problem, nonlinear infeasibilities',
     $         ' minimized')
 9020 format(  ' EXIT -- the problem is unbounded',
     $         ' (or badly scaled)')
 9021 format(  ' EXIT -- violation limit exceeded --',
     $         ' the problem may be unbounded')
 9031 format(  ' EXIT -- major iteration limit exceeded')
 9032 format(  ' EXIT -- minor iteration limit exceeded')
 9033 format(  ' EXIT -- iteration limit exceeded')
 9040 format(  ' EXIT -- requested accuracy could not be achieved')
 9060 format(  ' EXIT -- constraint and objective values',
     $         ' could not be calculated')
 9050 format(  ' EXIT -- the superbasics limit is too small:', i7)
 9090 format(  ' EXIT -- the current point cannot be improved')
 9120 format(  ' EXIT -- terminated from subroutine s1User')

*     end of s8core
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8dflt( job,
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character          job*(*)
      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     Job                   Action
*     ---                   ------
*     'C'heck     the parameter values are checked and possibly
*                 changed to reasonable values. 
*     'P'rint     If iPrint > 0 and lprPrm > 0, the parameters are
*                 printed.  (In the specs file,  Suppress parameters
*                 sets iw(3) = 0.)
* 
*     Note that checking occurs before the amount of working storage has
*     been defined. 
*
*     See snworkspace.doc for full documentation of the
*     cw, iw and rw arrays.
*
*     15 Nov 1991: first version.
*     14 May 1998: Current version of s8dflt.
*     ==================================================================
      integer            tolFP , tolQP , tolNLP, tolx  , tolCon,
     $                   tolpiv, tolrow, tCrash, tolswp, tolfac,
     $                   tolupd, plInfy, bigfx , bigdx , epsrf ,
     $                   fdint1, fdint2, xdlim , vilim , eta   ,
     $                   Hcndbd, wtInf0, xPen0 , scltol,
     $                   eLmax1, eLmax2, small ,
     $                   Utol1 , Utol2 , Uspace, Dens1 , Dens2 ,
     $                   toldj3

      parameter         (tolFP     =  51)
      parameter         (tolQP     =  52)
      parameter         (tolNLP    =  53)
      parameter         (tolx      =  56)
      parameter         (tolCon    =  57)
      parameter         (tolpiv    =  60)
      parameter         (tolrow    =  61)
      parameter         (tCrash    =  62)
      parameter         (tolswp    =  65)
      parameter         (tolfac    =  66)
      parameter         (tolupd    =  67)
      parameter         (plInfy    =  70)
      parameter         (bigfx     =  71)
      parameter         (bigdx     =  72)
      parameter         (epsrf     =  73)
      parameter         (fdint1    =  76)
      parameter         (fdint2    =  77)
      parameter         (xdlim     =  80)
      parameter         (vilim     =  81)
      parameter         (eta       =  84)
      parameter         (Hcndbd    =  85)
      parameter         (wtInf0    =  88)
      parameter         (xPen0     =  89)
      parameter         (scltol    =  92)
      parameter         (eLmax1    = 151)
      parameter         (eLmax2    = 152)
      parameter         (small     = 153)
      parameter         (Utol1     = 154)
      parameter         (Utol2     = 155)
      parameter         (Uspace    = 156)
      parameter         (Dens1     = 157)
      parameter         (Dens2     = 158)

      parameter         (toldj3    = 186)

      parameter         (maxru     =   2)
      parameter         (maxrw     =   3)
      parameter         (maxiu     =   4)
      parameter         (maxiw     =   5)
      parameter         (maxcu     =   6)
      parameter         (maxcw     =   7)
      parameter         (iRead     =  10)

      parameter         (nnCon     =  21)
      parameter         (nnJac     =  22)
      parameter         (nnObj     =  23)
      parameter         (nnL       =  24)
      parameter         (lEmode    =  51)
      parameter         (lHtype    =  53)
      parameter         (maxR      =  56)
      parameter         (maxS      =  57)
      parameter         (kchk      =  60)
      parameter         (kfac      =  61)
      parameter         (ksav      =  62)
      parameter         (klog      =  63)
      parameter         (kSumm     =  64)
      parameter         (kDegen    =  65)
      parameter         (mQNmod    =  66)
      parameter         (kReset    =  67)
      parameter         (mFlush    =  68)
      parameter         (mSkip     =  69)
      parameter         (lvlSrt    =  70)
      parameter         (lvlDer    =  71)
      parameter         (lvlExi    =  72)
      parameter         (lvlInf    =  73)
      parameter         (lvlPrt    =  74)
      parameter         (lvlScl    =  75)
      parameter         (lvlSch    =  76)
      parameter         (lvlTim    =  77)
      parameter         (lvlVer    =  78)
      parameter         (lprDbg    =  80)
      parameter         (lprPrm    =  81)
      parameter         (lprSch    =  82)
      parameter         (lprScl    =  83)
      parameter         (lprSol    =  84)
      parameter         (minmax    =  87)
      parameter         (iCrash    =  88)
      parameter         (itnlim    =  89)
      parameter         (mMajor    =  90)
      parameter         (mMinor    =  91)
      parameter         (MjrPrt    =  92)
      parameter         (MnrPrt    =  93)
      parameter         (nParPr    =  94)
      parameter         (jverf1    =  98)
      parameter         (jverf2    =  99)
      parameter         (jverf3    = 100)
      parameter         (jverf4    = 101)
      parameter         (iBack     = 120)
      parameter         (iDump     = 121)
      parameter         (iLoadB    = 122)
      parameter         (iNewB     = 124)
      parameter         (iInsrt    = 125)
      parameter         (iOldB     = 126)
      parameter         (iPnch     = 127)
      parameter         (iReprt    = 130)
      parameter         (iSoln     = 131)
      parameter         (nout      = 151)
      parameter         (LUprnt    = 152)
      parameter         (maxcol    = 153)
      parameter         (minimz    = 199)
 
      parameter         (idummy = -11111)
      parameter         (zero   =  0.0d+0, one    =      1.0d+0)
      parameter         (ten    = 10.0d+0)
      parameter         (tenp6  = 1.0d+6,  hundrd = 100.0d+0)

      logical            linCon, nlnCon, nonlin
      character*11       lsrch(0:1)
      character*24       Hestyp(2), prbtyp(3)
      data               Hestyp /' Limited-Memory Hessian.',
     $                           ' Full-Memory Hessian....'/
      data               prbtyp /' Maximize...............',
     $                           ' Feasible point only....',
     $                           ' Minimize...............'/
      data               lsrch  /' Nonderiv. ',
     $                           ' Derivative'/
*     ------------------------------------------------------------------
*     Set some local machine-dependent constants.

      eps        = rw(  1) 
      eps0       = rw(  2) 
      eps1       = rw(  3) 
      eps2       = rw(  4) 
      eps3       = rw(  5) 
      eps4       = rw(  6) 
      c4         = max( 1.0d-4, eps3 )
      c6         = max( 1.0d-6, eps2 )
      never      = 99999999

      iPrint     = iw( 12)
      iSpecs     = iw( 11)

      m          = iw( 15)
      n          = iw( 16)

      if (job(1:1) .eq. 'C') then
*        ---------------------------------------------------------------
*        Job  = 'Check'.   Check the optional parameters.
*        ---------------------------------------------------------------
*        Check the values

         if (iw(nnCon) .eq. 0) iw(nnJac) = 0
         if (iw(nnJac) .eq. 0) iw(nnCon) = 0
         iw(nnL) = max( iw(nnJac), iw(nnObj) )
         linCon  = iw(nnCon)   .eq. 0
         nlnCon  = iw(nnCon)   .gt. 0
         nonlin  = iw(nnL)     .gt. 0

         if (iw(iBack ) .eq. idummy ) iw(iBack ) =     0
         if (iw(iDump ) .eq. idummy ) iw(iDump ) =     0
         if (iw(iLoadB) .eq. idummy ) iw(iLoadB) =     0
         if (iw(iNewB ) .eq. idummy ) iw(iNewB ) =     0
         if (iw(iInsrt) .eq. idummy ) iw(iInsrt) =     0
         if (iw(iOldB ) .eq. idummy ) iw(iOldB ) =     0
         if (iw(iPnch ) .eq. idummy ) iw(iPnch ) =     0
         if (iw(iReprt) .eq. idummy ) iw(iReprt) =     0
         if (iw(iSoln ) .eq. idummy ) iw(iSoln ) =     0

*        Set unspecified frequencies or silly values to defaults.

         if (iw(kchk)   .eq. idummy ) iw(kchk)   =    60
         if (iw(kfac)   .le.    0   ) then
                                      iw(kfac)   =   100
                     if (nlnCon     ) iw(kfac)   =    50
         end if
         if (iw(klog)  .eq. idummy  ) iw(klog)   =     1
         if (iw(kSumm) .eq. idummy  ) iw(kSumm)  =     1
         if (iw(ksav)  .eq. idummy  ) iw(ksav)   =   100
         if (iw(kDegen).eq. idummy  ) iw(kDegen) = 10000
         if (iw(mFlush).eq. idummy  ) iw(mFlush) =     0

*        Sometimes, frequency 0 means "almost never".

         if (iw(kchk)   .le. 0      ) iw(kchk)   = never
         if (iw(mFlush) .le. 0      ) iw(mFlush) = never
         if (iw(klog)   .le. 0      ) iw(klog)   = never
         if (iw(ksav)   .le. 0      ) iw(ksav)   = never
         if (iw(kSumm)  .le. 0      ) iw(kSumm)  = never
         if (iw(kDegen) .le. 0      ) iw(kDegen) = never
         if (iw(kReset) .le. 0      ) iw(kReset) = never

         if (iw(iCrash) .lt. 0 .and. iw(nnCon) .eq.  0)
     $                                iw(iCrash) = 3
         if (iw(iCrash) .lt. 0 .and. iw(nnCon) .gt.  0)
     $                                iw(iCrash) = 0
         if (iw(lHtype) .lt. 0 .and.  iw(nnL)  .gt. 75)
     $                                iw(lHtype) = 1
         if (iw(lHtype) .lt. 0 .and.  iw(nnL)  .le. 75)
     $                                iw(lHtype) = 2
         if (iw(lHtype) .eq. 2      ) iw(mQNmod) = iw(kReset)
         if (iw(mQNmod) .lt. 0      ) iw(mQNmod) = 20

         if (iw(minmax) .eq. idummy ) iw(minmax) =  1
         if (iw(minmax) .eq. -1) then
                                      iw(minimz) = -1
         else
                                      iw(minimz) =  1
         end if
         if (iw(MjrPrt) .eq. idummy ) iw(MjrPrt) =  1
         if (iw(MnrPrt) .eq. idummy ) iw(MnrPrt) =  0

         if (iw(maxcu)  .lt. 500    ) iw(maxcu)  = 500
         if (iw(maxiu)  .lt. 500    ) iw(maxiu)  = 500
         if (iw(maxru)  .lt. 500    ) iw(maxru)  = 500
         if (iw(maxcw)  .lt. lencw  ) iw(maxcw)  = lencw
         if (iw(maxiw)  .lt. leniw  ) iw(maxiw)  = leniw
         if (iw(maxrw)  .lt. lenrw  ) iw(maxrw)  = lenrw

         if (iw(mMinor) .lt. 0      ) iw(mMinor) = max( 1000,5*m )
         if (iw(mMajor) .lt. 0      ) iw(mMajor) = max( 1000,  m )
         if (iw(mSkip ) .lt. 0      ) iw(mSkip ) =   2

         if (iw(lprDbg) .lt. 0      ) iw(lprDbg) = 0
         if (iw(lprPrm) .lt. 0      ) iw(lprPrm) = 1
         if (iw(lprSch) .lt. 0      ) iw(lprSch) = never
         if (iw(lprScl) .lt. 0      ) iw(lprScl) = 0
         if (iw(lprSol) .lt. 0      ) iw(lprSol) = 2

         if (iw(lvlSrt) .lt. 0      ) iw(lvlSrt) =   0
         if (iw(lvlDer) .lt. 0      ) iw(lvlDer) =   3
         if (iw(lvlVer) .lt. 0      ) iw(lvlVer) =   0

         if (iw(lvlPrt) .lt. 0      ) iw(lvlPrt) = 0
         if (iw(lvlPrt) .gt. 0      ) iw(MjrPrt) = iw(lvlPrt)

         if (iw(lvlExi) .lt. 0  .or. iw(lvlExi) .gt. 1
     $                              ) iw(lvlExi) = idummy
         if (iw(lvlExi) .eq. idummy ) iw(lvlExi) = 0
                                      iw(lvlInf) = 2
         if (iw(lvlSch) .lt. 0      ) iw(lvlSch) = 1
                                      iw(lEmode) = 1

*        Check superbasics limit and reduced Hessian size.

         if ( nonlin ) then
            if (iw(maxR) .gt. 0  .and.  iw(maxS) .lt. 0 
     $                              ) iw(maxS)   = iw(maxR)
            if (iw(maxS) .gt. 0  .and.  iw(maxR) .lt. 0
     $                              ) iw(maxR)   = iw(maxS)

            if (iw(maxS) .lt. 0     ) iw(maxS)   = min( 500, iw(nnL)+1 )
            if (iw(maxR) .lt. 0     ) iw(maxR)   = iw(maxS)
         end if
         if (iw(maxS)   .le. 0      ) iw(maxS)   = 1
         if (iw(maxR)   .lt. 0      ) iw(maxR)   = 0
         if (iw(maxR)   .lt. iw(maxS))iw(maxR)   = iw(maxS)

         iw(maxR) = max( min( iw(maxR) ,n ) , 1 )
         iw(maxS) = max( min( iw(maxS) ,n ) , 1 )

*        Check other options.

         if (iw(lvlScl) .lt. 0   ) then
                                      iw(lvlScl) = 2
            if ( nlnCon )             iw(lvlScl) = 1
         end if
                                      iw(lvlScl) = min( iw(lvlScl), 2 )

         if (iw(nParPr) .le. 0   ) then
                                      iw(nParPr) = 10
            if ( nlnCon )             iw(nParPr) =  1
         end if

         cHzbnd = max ( one/(hundrd*eps*dble(iw(maxS))), tenp6 )

         if (rw(plInfy)   .lt. zero ) rw(plInfy) = 1.0d+20
         if (rw(epsrf)    .le. zero ) rw(epsrf)  = eps0

         if (rw(bigfx)    .le. zero ) rw(bigfx)  = 1.0d+15
         if (rw(bigdx)    .le. zero ) rw(bigdx)  = rw(plInfy)
         if (rw(Hcndbd)   .le. zero ) rw(Hcndbd) = cHzbnd
         if (rw(xdlim)    .le. zero ) rw(xdlim)  = 2.0d+0
         if (rw(vilim)    .le. zero ) rw(vilim)  = ten
         if (rw(xPen0)    .lt. zero ) rw(xPen0)  = zero

         if (rw(tCrash)   .lt. zero  .or.
     $       rw(tCrash)   .ge. one  ) rw(tCrash) = 0.1d+0
         if (rw(eta)      .lt. zero  .or.
     $       rw(eta)      .gt. one  ) rw(eta)    = 0.9d+0

         if (rw(fdint1).le. zero    ) rw(fdint1) = sqrt(rw(epsrf))
         if (rw(fdint2).le. zero    ) rw(fdint2) = rw(epsrf)**0.33333d+0

*        ---------------------------------
*        Set up the parameters for lu1fac.
*        ---------------------------------
         if (iw(maxcol) .lt.  0     ) iw(maxcol) =   5
         if (iw(LUprnt) .eq.  idummy) iw(LUprnt) =  -1

                                      iw(nout)   =  iPrint
         if (iw(MnrPrt) .gt. 10     ) iw(LUprnt) =  0
         if (iw(lprDbg) .eq. 51     ) iw(LUprnt) =  1
         if (iw(lprDbg) .eq. 52     ) iw(LUprnt) =  2
         if (iPrint     .lt.  0     ) iw(LUprnt) = -1
         if (linCon) then
            if (rw(tolFac) .lt. one ) rw(tolFac) =  100.0d+0
            if (rw(tolUpd) .lt. one ) rw(tolUpd) =   10.0d+0
         else
            if (rw(tolFac) .lt. one ) rw(tolFac) =    5.0d+0
            if (rw(tolUpd) .lt. one ) rw(tolUpd) =    5.0d+0
         end if
                                      rw(eLmax1) = rw(tolFac)
                                      rw(eLmax2) = rw(tolUpd)
         if (rw(Utol1)    .le. zero ) rw(Utol1 ) =  eps1
         if (rw(Utol2)    .le. zero ) rw(Utol2 ) =  eps1
         if (rw(Dens2)    .lt. zero ) rw(Dens2 ) =  0.6d+0

         if (rw(small )   .le. zero ) rw(small ) =  eps0
         if (rw(Uspace)   .le. zero ) rw(Uspace) =  3.0d+0
         if (rw(Dens1 )   .le. zero ) rw(Dens1 ) =  0.3d+0

*        Set some SQP tolerances.
*        Set the minor and major optimality tolerances.
*        Solve the QP subproblems fairly accurately even if the 
*        NLP Optimality Tolerance is big.

         if (rw(tolQP) .le. zero) then
            rw(tolQP) = c6
         end if
         if (rw(tolNLP) .le. zero) then
            rw(tolNLP) = c6
            if (rw(epsrf) .gt. zero ) rw(tolNLP) = sqrt(ten*rw(epsrf))
         end if

         if (rw(tolFP)    .lt. zero ) rw(tolFP)  =  c6
         if (rw(tolrow)   .le. zero ) rw(tolrow) =  c4
         if (rw(tolswp)   .le. zero ) rw(tolswp) =  eps4
         if (rw(tolx)     .le. zero ) rw(tolx)   =  c6
         if (rw(tolCon)   .le. eps  ) rw(tolCon) =  c6
                                      rw(toldj3) =  rw(tolQP)
         if (rw(scltol)   .le. zero ) rw(scltol) =  0.90d+0
         if (rw(scltol)   .ge. one  ) rw(scltol) =  0.99d+0
         if (rw(tolpiv)   .le. zero ) rw(tolpiv) =  eps1

         if (linCon) then
            if (rw(wtInf0).lt. zero ) rw(wtInf0) =  1.0d+0
         else
            if (rw(wtInf0).lt. zero ) rw(wtInf0) =  1.0d+2
         end if

*        Check the START and STOP column numbers for gradient checking.

         if (iw(jverf1) .lt. 0      ) iw(jverf1) = 1
         if (iw(jverf2) .lt. 0      ) iw(jverf2) = iw(nnObj)
         if (iw(jverf3) .lt. 0      ) iw(jverf3) = 1
         if (iw(jverf4) .lt. 0      ) iw(jverf4) = iw(nnJac)

         if (iw(iBack)  .eq. iw(iNewB) 
     $                             ) iw(iBack)   = 0
         if (iw(itnlim) .lt. 0     ) iw(itnlim)  = max( 10000,m )

      else if (job(1:1) .eq. 'P'  .and. iPrint .gt. 0) then
*        ---------------------------------------------------------------
*        Job = 'P'rint    Print parameters except if PRINT LEVEL = 0
*                         or SUPPRESS PARAMETERS was specified.
*        ---------------------------------------------------------------
         if (iw(MjrPrt) .gt. 0  .and.  iw(lprPrm) .gt. 0) then
            linCon = iw(nnCon) .eq. 0
            nlnCon = iw(nnCon) .gt. 0
            nonlin = iw(nnL)   .gt. 0

            call s1page( 1, iw, leniw )
            write(iPrint, 1000)
*           --------------------
*           Files.
*           --------------------
            write(iPrint, 2100) iw(iSoln) , iw(iOldB) , iw(iRead),
     $                          iw(iInsrt), iw(iNewB) , iPrint,
     $                          iw(iPnch) , iw(iBack) , iSpecs,
     $                                      iw(iLoadB), iw(iDump)
*           --------------------
*           Frequencies.
*           --------------------
            write(iPrint, 2200) iw(klog)  , iw(kchk)  , iw(ksav)  ,
     $                          iw(kSumm) , iw(kfac)  , iw(kDegen)
*           --------------------
*           QP subproblems.
*           --------------------
            write(iPrint, 2300) rw(scltol), rw(tolx)  , iw(itnlim),
     $                          iw(lvlScl), rw(tolQP) , iw(nParPr),
     $                          rw(tCrash), rw(tolpiv), iw(MnrPrt),
     $                          iw(iCrash), rw(wtInf0)
*           --------------------
*           SQP method.
*           --------------------
            write(iPrint, 2400) prbtyp(2+iw(minmax)),
     $                          iw(nnObj) , rw(tolNLP), rw(epsrf) ,
     $                          rw(bigdx) , iw(maxS)  , rw(fdint1),
     $                          rw(bigfx) , iw(maxR)  , rw(fdint2),
     $                          rw(xdlim) , lsrch(iw(lvlSch))
     $                                                , iw(lvlDer),
     $                          iw(mMajor), rw(eta)   , iw(lvlVer),
     $                          iw(mMinor), rw(xPen0) , iw(MjrPrt)
*           --------------------
*           Hessian approximation.
*           --------------------
            if ( nonlin )
     $      write(iPrint, 2500) Hestyp(iw(lHtype)),
     $                                      iw(mQNmod), iw(kReset),
     $                                                  iw(mFlush)
*           --------------------
*           Nonlinear constraints.
*           --------------------
            if ( nlnCon )
     $      write(iPrint, 2600) iw(nnCon) , rw(tolCon), rw(vilim),
     $                          iw(nnJac) 
*           --------------------
*           Miscellaneous
*           --------------------
            write(iPrint, 2700) rw(tolFac), rw(Utol1) , iw(lvlTim),
     $                          rw(tolUpd), rw(tolswp), iw(lprDbg),
     $                                      eps
         end if
      end if

      return

 1000 format(  ' Parameters' 
     $       / ' ==========')
 2100 format(/ ' Files'
     $       / ' -----'
     $/ ' Solution file..........', i10, 6x,
     $  ' Old basis file ........', i10, 6x,
     $  ' Standard input.........', i10
     $/ ' Insert file............', i10, 6x,
     $  ' New basis file ........', i10, 6x,
     $  ' (Printer)..............', i10
     $/ ' Punch file.............', i10, 6x,
     $  ' Backup basis file......', i10, 6x,
     $  ' (Specs file)...........', i10
     $/ 40x,
     $  ' Load file..............', i10, 6x,
     $  ' Dump file..............', i10)
 2200 format(/ ' Frequencies'
     $       / ' -----------'
     $/ ' Print frequency........', i10, 6x,
     $  ' Check frequency........', i10, 6x,
     $  ' Save new basis map.....', i10
     $/ ' Summary frequency......', i10, 6x,
     $  ' Factorization frequency', i10, 6x,
     $  ' Expand frequency.......', i10)
 2300 format(/ ' QP subproblems'
     $       / ' --------------'
     $/ ' Scale tolerance........', 0p, f10.3, 6x,
     $  ' Minor feasibility tol..', 1p, e10.2, 6x,
     $  ' Iteration limit........', i10
     $/ ' Scale option...........', i10,       6x,
     $  ' Minor optimality  tol..', 1p, e10.2, 6x,
     $  ' Partial  price.........', i10
     $/ ' Crash tolerance........', 0p, f10.3, 6x,
     $  ' Pivot tolerance........', 1p, e10.2, 6x,
     $  ' Minor Print Level......', i10,
     $/ ' Crash option...........', i10,       6x,
     $  ' Elastic weight.........', 1p, e10.2)
 2400 format(/ ' The SQP Method'
     $       / ' --------------'
     $/   a24,                                16x
     $/ ' Nonlinear objectiv vars', i10,       6x,
     $  ' Major optimality tol...', 1p, e10.2, 6x,
     $  ' Function precision.....', 1p, e10.2
     $/ ' Unbounded step size....', 1p, e10.2, 6x,
     $  ' Superbasics limit......', i10,       6x,
     $  ' Difference interval....', 1p, e10.2
     $/ ' Unbounded objective....', 1p, e10.2, 6x,
     $  ' Reduced Hessian dim....', i10,       6x,
     $  ' Central difference int.', 1p, e10.2
     $/ ' Major step limit.......', 1p, e10.2, 6x, 
     $         A11,' linesearch..',           16x,
     $  ' Derivative level.......', i10
     $/ ' Major iterations limit.', i10,       6x,
     $  ' Linesearch tolerance...', 0p, f10.5, 6x,
     $  ' Verify level...........', i10
     $/ ' Minor iterations limit.', i10,       6x,
     $  ' Penalty parameter......', 1p, e10.2, 6x,
     $  ' Major Print Level......', i10 )
 2500 format(/ ' Hessian Approximation'
     $       / ' ---------------------'
     $/  a24,                                 16x, 
     $  ' Hessian updates........', i10,       6x,
     $  ' Hessian frequency......', i10    
     $/  80x,
     $  ' Hessian flush..........', i10 )
 2600 format(/ ' Nonlinear constraints'
     $       / ' ---------------------'
     $/ ' Nonlinear constraints..', i10,       6x,
     $  ' Major feasibility tol..', 1p, e10.2, 6x,
     $  ' Violation limit........',     e10.2
     $/ ' Nonlinear Jacobian vars', i10 )

 2700 format(/ ' Miscellaneous'
     $       / ' -------------'
     $/ ' LU factor tolerance....', 0p, f10.2, 6x,
     $  ' LU singularity tol.....', 1p, e10.2, 6x,
     $  ' Timing level...........', i10
     $/ ' LU update tolerance....', 0p, f10.2, 6x,
     $  ' LU swap tolerance......', 1p, e10.2, 6x,
     $  ' Debug level............', i10
     $/ 40x,
     $  ' eps (machine precision)', e10.2)

*     end of s8dflt
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8Mem ( ierror, iPrint, iSumm,
     $                   m, n, ne, neJac,
     $                   nnCon, nnJac, nnObj,
     $                   maxR, maxS, 
     $                   maxcw, maxiw, maxrw,
     $                   lencw, leniw, lenrw, 
     $                   mincw, miniw, minrw, iw )

      implicit           double precision (a-h,o-z)
      integer            iw(leniw)

*     ==================================================================
*     s8Mem   allocates all array storage for snopt,
*     using the values:
*        m    , n    , ne
*        maxR , maxS             Set in s8dflt.
*        nnObj, nnCon, nnJac     Set by specs file or argument list.
*        neJac                   Set in calling program
*
*     These values are used to compute the minimum required storage:
*     mincw, miniw, minrw.
*
*     The SPECS file has been read and values are known for
*     maxcu, maxiu, maxru  (upper limit of user  partition 1)
*     maxcw, maxiw, maxrw  (upper limit of SNOPT partition)
*
*     The default values for the first six are
*     maxcu = 500  ,   maxiu = 500  ,   maxru = 500,
*     maxcw = lencw,   maxiw = leniw,   maxrw = lenrw,
*     but we allow the user to alter these in the SPECS file via
*     lines of the form
*     
*        User  character workspace      10000    (Sets maxcu)
*        User  integer   workspace      10000    (Sets maxiu)
*        User  real      workspace      10000    (Sets maxru)
*        Total character workspace      90000    (Sets maxcw)
*        Total integer   workspace      90000    (Sets maxiw)
*        Total real      workspace      90000    (Sets maxrw)
*
*     SNOPT will use only rw(maxru+1:maxrw).  Hence, rw(501:maxru)
*     and possibly rw(maxrw+1:lenrw) may be used as workspace by the
*     user during solution of the problem (e.g., within funobj or
*     funcon).  Similarly for iw(501:maxiu) and iw(maxiw:leniw).
*
*     Setting maxiw and maxrw less than leniw and lenrw may serve to
*     reduce paging activity on a machine with virtual memory, by
*     confining SNOPT (in particular the LU-factorization routines)
*     to an area of memory that is sensible for the current problem.
*     This often allows cw(*), iw(*) and rw(*) to be declared
*     arbitrarily large at compile time.
*
*     On exit.
*        If ierror = 0,  mincw, miniw, minrw give the amounts of
*        character, integer and real workspace needed to hold
*        the problem. (The LU factorization routines may 
*        subsequently ask for more.) 
*
*        If ierror > 0,  insufficient storage is provided to hold the
*        problem.  In this case, mincw, miniw and minrw give estimates
*        of reasonable lengths for cw(*), iw(*) and rw(*).
*
*     15 Nov 1991: First version based on Minos 5.4 routine m2core.
*     29 Mar 1998: First version called by snMem. This simplified 
*                  version may slightly overestimate needed memory.
*     29 Mar 1998: Current version of s8Mem.
*     ==================================================================
      parameter         (maxru     =   2)
      parameter         (maxiu     =   4)
      parameter         (maxcu     =   6)

      parameter         (lenr      =  28)

      parameter         (mincu1    =  31)
      parameter         (maxcu1    =  32)
      parameter         (mincu2    =  33)
      parameter         (maxcu2    =  34)

      parameter         (miniu1    =  36)
      parameter         (maxiu1    =  37)
      parameter         (miniu2    =  38)
      parameter         (maxiu2    =  39)

      parameter         (minru1    =  41)
      parameter         (maxru1    =  42)
      parameter         (minru2    =  43)
      parameter         (maxru2    =  44)
*     ------------------------------------------------------------------
      ierror = 0

*     All dimensions are computed from 
*        m    , n    , ne
*        maxR , maxS , nnL
*        nnObj,    
*        neJac, nnCon, nnJac

      nnL      = max( nnJac, nnObj )
      leng     = nnL            ! Allows for feasible exit.
            
      mBS      = m     + maxS
      nb       = n     + m
      iw(lenr) = maxR*(maxR + 1)/2

      nScl     = nb
      nx2      = nnL

*     Nonlinear constraints.

      nkg       = nnJac  + 1

*     snopt can use all of cw, iw and rw
*     except the first user workspace partitions.

      lhfeas = miniw
      lkBS   = lhfeas + mBS
      miniw  = lkBS   + mBS

*     QP subproblem.

      lhEsta = miniw
      lhElas = lhEsta + nb
      liy    = lhElas + nb 
      liy2   = liy    + mBS
      miniw  = liy2   + m

      laScal = minrw
      lw     = laScal + nScl
      ly     = lw     + nb
      ly1    = ly     + nb
      ly2    = ly1    + nb
      lblBS  = ly2    + nb
      lbuBS  = lblBS  + mBS
      lxBS   = lbuBS  + mBS
      lxScl  = lxBS   + mBS
      lgBS   = lxScl  + nx2
      lgObjQ = lgBS   + mBS
      lHp    = lgObjQ + nnL
      lgdif  = lHp    + nnL
      lr     = lgdif  + nnL
      lrg    = lr     + iw(lenr)
      minrw  = lrg    + maxS

*     Nonlinear Objective.

      lgObj1 = minrw
      lgObj  = lgObj1 + leng
      lgObj2 = lgObj  + leng
      lgObjU = lgObj2 + leng
      minrw  = lgObjU + leng

*     Nonlinear constraints.

      lkg    = miniw
      miniw  = lkg    + nkg

      lfCon  = minrw
      lfCon2 = lfCon  + nnCon
      lviol  = lfCon2 + nnCon
      lblTru = lviol  + nnCon
      lbuTru = lblTru + nb
      lxMul  = lbuTru + nb
      lxMul2 = lxMul  + nnCon
      lxdMul = lxMul2 + nnCon
      lxPen  = lxdMul + nnCon
      lfCon1 = lxPen  + nnCon
      lyslk  = lfCon1 + nnCon
      lgConU = lyslk  + nnCon
      lgCon  = lgConU + neJac
      lgCon2 = lgCon  + neJac
      lrhs   = lgCon2 + neJac
      lxdif  = lrhs   + m
      lxfeas = lxdif  + nb
      lx2    = lxfeas + nb
      minrw  = lx2    + nb

*     Store the addresses in iw.

      iw(260) = lhElas
      iw(267) = lhfeas
      iw(268) = lhEsta
      iw(269) = lkBS
      iw(270) = lblBS
      iw(271) = lbuBS
      iw(272) = lxBS
      iw(273) = lgBS
      iw(274) = laScal
      iw(275) = lxScl
      iw(276) = lx2
      iw(277) = lrg
      iw(278) = lr
      iw(279) = liy
      iw(280) = liy2
      iw(281) = ly
      iw(282) = ly2
      iw(283) = lw
      iw(284) = ly1

      iw(289) = lgObjQ
      iw(290) = lrhs

      iw(295) = lHp
      iw(296) = lxdif
      iw(297) = lgdif
      iw(298) = lgObj
      iw(299) = lgObjU
      iw(300) = lgObj1
      iw(301) = lgObj2
      iw(302) = lfCon
      iw(303) = lfCon1
      iw(304) = lfCon2
      iw(305) = lgCon
      iw(306) = lgConU
      iw(307) = lgCon2

      iw(312) = lkg
      iw(313) = lviol
      iw(314) = lblTru
      iw(315) = lbuTru
      iw(316) = lxMul
      iw(317) = lxMul2
      iw(318) = lxdMul
      iw(319) = lxPen
      iw(320) = lyslk
      iw(321) = lxfeas

*     Allocate space for the approximate Hessian. 
*     The amount will depend on the method selected.

      call s8Hmap( nnL, nnCon, minrw, iw, leniw )

*     Allocate arrays for the basis factorization routines.
*     miniw, minrw point to the beginning of the LU factorization.

      call s2Bmap( m, n, ne,
     $             miniw, minrw, maxiw, maxrw, liwEst, lrwEst,
     $             iw, leniw )

      mincw  = mincw  -  1      ! Character storage can be exact

*     ------------------------------------------------------------------
*     Set the limits of the two user-accessible workspace partitions.
*     The upper limits of the partition 1 are set in the specs file.
*     The lower limits of the partition 2 are set here.
*     ------------------------------------------------------------------
      iw(mincu1) = 501          ! Lower limits on partition 1 
      iw(miniu1) = 501
      iw(minru1) = 501

      iw(maxcu1) = iw(maxcu)    ! User-defined upper limits on
      iw(maxiu1) = iw(maxiu)    ! partition 1 (default 500).
      iw(maxru1) = iw(maxru)

      iw(mincu2) = mincw + 1    ! Lower limits on partition 2 
      iw(miniu2) = miniw + 1
      iw(minru2) = minrw + 1

      iw(maxcu2) = lencw        ! Upper limits on partition 2
      iw(maxiu2) = leniw
      iw(maxru2) = lenrw

*     ---------------------------------------------------------------
*     Print details of the workspace.
*     ---------------------------------------------------------------
      if (iPrint .gt. 0) then
         write(iPrint, 1100)       maxcw,      maxiw,     maxrw,
     $                             mincw,      miniw,     minrw
         if (iw(maxcu1) .ge. iw(mincu1))
     $      write(iPrint, 1201) iw(mincu1), iw(maxcu1)
         if (iw(maxiu1) .ge. iw(miniu1))
     $      write(iPrint, 1202) iw(miniu1), iw(maxiu1)
         if (iw(maxru1) .ge. iw(minru1))
     $      write(iPrint, 1203) iw(minru1), iw(maxru1)
      end if

      if (mincw .gt. maxcw   .or.  miniw .gt. maxiw 
     $                       .or.  minrw .gt. maxrw) then 
*        ---------------------------------------------------------------
*        Not enough workspace to solve the problem.
*        ---------------------------------------------------------------
         if (iPrint .gt. 0) write(iPrint, 9400)
         if (iSumm  .gt. 0) write(iSumm , 9400)

         if (mincw  .gt. lencw ) then
*           ------------------------------------------------------------
*           Not enough character workspace.
*           ------------------------------------------------------------
            ierror = 42
            if (iPrint .gt. 0) write(iPrint, 9420) mincw
            if (iSumm  .gt. 0) write(iSumm , 9420) mincw
         end if

         if (     miniw .gt. leniw) then
*           ------------------------------------------------------------
*           Not enough integer workspace.
*           ------------------------------------------------------------
            miniw  = liwEst
            ierror = 43
            if (iPrint .gt. 0) write(iPrint, 9430) miniw
            if (iSumm  .gt. 0) write(iSumm , 9430) miniw
         end if

         if (minrw  .gt. lenrw ) then
*           ------------------------------------------------------------
*           Not enough real    workspace.
*           ------------------------------------------------------------
            minrw  = lrwEst
            ierror = 44
            if (iPrint .gt. 0) write(iPrint, 9440) minrw
            if (iSumm  .gt. 0) write(iSumm , 9440) minrw
         end if
      end if

      return

 1100 format(/ ' Total char*8  workspace', i10, 6x,
     $         ' Total integer workspace', i10, 6x,
     $         ' Total real    workspace', i10
     $       / ' Total char*8  (minimum)', i10, 6x,
     $         ' Total integer (minimum)', i10, 6x,
     $         ' Total real    (minimum)', i10/)
 1201 format(  ' Elements cw(', i10, ':',i10, ')', 6x, 'are free',
     $         ' for USER CHAR*8  WORKSPACE')
 1202 format(  ' Elements iw(', i10, ':',i10, ')', 6x, 'are free',
     $         ' for USER INTEGER WORKSPACE')
 1203 format(  ' Elements rw(', i10, ':',i10, ')', 6x, 'are free',
     $         ' for USER REAL    WORKSPACE')

 9400 format(  ' EXIT -- not enough storage to start solving',
     $         ' the problem...' )
 9420 format(/ ' Total character workspace should be significantly',
     $         ' more than', i8)
 9430 format(/ ' Total integer   workspace  should be significantly',
     $         ' more than', i8)
 9440 format(/ ' Total real      workspace  should be significantly',
     $         ' more than', i8)

*     end of s8Mem
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8savB( job, ierror, minimz,  
     $                   m, n, nb, nnL, 
     $                   nnCon, nnJac, nnObj,
     $                   nName, nS, nScl, sclObj,
     $                   itn, nInf, sInf, wtInf,
     $                   iObj, Objtru, vimax,
     $                   pNorm1, pNorm2, piNorm, xNorm,
     $                   ne, nka, a, ha, ka,
     $                   hEstat, hs, aScale, bl, bu,
     $                   Names, pi, rc, xs, y, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character          job*(*)
      character*8        Names(nName)
      integer            ha(ne), hEstat(nb), hs(nb)
      integer            ka(nka)
      double precision   a(ne), aScale(nScl), bl(nb), bu(nb)
      double precision   pi(m), rc(nb), xs(nb), y(m)

      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     s8savB  saves basis files  and/or  prints the solution.
*
*     If job = 'S'ave, the problem is first unscaled, then from 0 to 4 
*     files are saved (Punch file, Dump file, Solution file, 
*     Report file, in that order).
*     A new BASIS file, if any, will already have been saved by s5QP.
*
*     A call with job = 'S'ave must precede a call with job = 'P'rint.
*
*     If job = 'P'rint, the solution is printed under the control of 
*     lprSol (which is set by the solution keyword in the specs file).
*
*     15 Nov 1991: First version based on Minos 5.4 routine m4savb.
*     19 Feb 1994: Use s4rc to compute reduced costs.
*     05 Apr 1996: Call s2rcA  to get the reduced costs (as in Minos 5.5).
*                Maximum primal and dual infeasibilities computed
*                and printed here. 
*     12 Nov 1996: Current version of s8savB.
*     ==================================================================
      character*4        istate(3)
      logical            feasbl, prnt
      parameter         (one = 1.0d+0)
*     ------------------------------------------------------------------
      lgObj     = iw(298)
      lfCon     = iw(302)

      iPrint    = iw( 12)
      iSumm     = iw( 13)
      iDump     = iw(121)
      iPnch     = iw(127)
      iReprt    = iw(130)
      iSoln     = iw(131)

      lvlScl    = iw( 75)
      lprSol    = iw( 84)

      eps0      = rw(  2)
      tolx      = rw( 56)
      plInfy    = rw( 70)

      feasbl    = nInf .eq. 0
      k         = 1 + ierror
      call s4stat( k, istate )

      if (job(1:1) .eq. 'S') then
*        ---------------------------------------------------------------
*        job = 'S'ave.
*        Compute rc and unscale a, bl, bu, pi, xs, fCon, gObj, xNorm and
*        piNorm  (but s4soln uses scaled piNorm, so save it).
*        Then save basis files.
*        ---------------------------------------------------------------
*        Compute reduced costs rc(*) for all columns and rows.

         call s2rcA ( feasbl, tolx, iObj, minimz, wtInf,
     $                m, n, nb, nnObj,
     $                ne, nka, a, ha, ka,
     $                hEstat, hs, bl, bu, rw(lgObj), pi, rc, xs )
         call s2Binf( nb, bl, bu, xs, Binf, jBinf )
         call s2Dinf( n, nb, iObj, bl, bu, rc, xs, Dinf, jDinf )

         Binf1  = Binf
         Dinf1  = Dinf
         jBinf1 = jBinf
         jDinf1 = jDinf

         pNorm1 = piNorm
         xNorm1 = xNorm

         if (lvlScl .gt. 0) then
            call s2scla( 'Unscale', m, n, nb,
     $                   iObj, plInfy, sclObj,
     $                   ne, nka, a, ha, ka,
     $                   aScale, bl, bu, pi, xs )
         
            call dddiv ( nb, aScale, 1, rc, 1 )

            if (lvlScl .eq. 2) then
               if (nnCon .gt. 0)
     $         call ddscl ( nnCon, aScale(n+1), 1, rw(lfCon), 1 )
         
               if (nnObj .gt. 0)
     $         call dddiv ( nnObj, aScale     , 1, rw(lgObj), 1 )
            end if

            xNorm  = dnrm1s( nb, xs , 1 )
            piNorm = max( dnrm1s( m, pi, 1 ), one )
            call s2Binf( nb, bl, bu, xs, Binf, jBinf )
            call s2Dinf( n, nb, iObj, bl, bu, rc, xs, Dinf, jDinf )
         end if
         pNorm2 = piNorm

*        ---------------------------------------------------------------
*        Print various scaled and unscaled norms.
*        ---------------------------------------------------------------
         if (lvlScl .gt. 0) then
            if (iPrint .gt. 0) write(iPrint, 1010) xNorm1, pNorm1
            if (iSumm  .gt. 0) write(iSumm , 1010) xNorm1, pNorm1
         end if
            if (iPrint .gt. 0) write(iPrint, 1020) xNorm , piNorm
            if (iSumm  .gt. 0) write(iSumm , 1020) xNorm , piNorm
         if (lvlScl .gt. 0) then
            if (iPrint .gt. 0) write(iPrint, 1030) jBinf1, Binf1 ,
     $                                             jDinf1, Dinf1
            if (iSumm  .gt. 0) write(iSumm , 1030) jBinf1, Binf1 ,
     $                                             jDinf1, Dinf1
         end if
            if (iPrint .gt. 0) write(iPrint, 1040) jBinf , Binf  ,
     $                                             jDinf , Dinf
            if (iSumm  .gt. 0) write(iSumm , 1040) jBinf , Binf  ,
     $                                             jDinf , Dinf

*        Change the sign of pi and rc if feasible and maximizing.

         if (nInf .eq. 0  .and.  minimz .lt. 0) then
            call dscal ( m , (-one), pi, 1 )
            call dscal ( nb, (-one), rc, 1 )
         end if

*        Compute nonlinear constraint infeasibilities (violations).

         if (nnCon .gt. 0) then
            call s8nslk( n, nnCon, nnJac, eps0,
     $                   maxvi, vimax, virel, xNorm,
     $                   ne, nka, a, ha, ka,
     $                   bl, bu, rw(lfCon), xs, y )
            if (iPrint .gt. 0) write(iPrint, 1080) vimax
            if (iSumm  .gt. 0) write(iSumm , 1080) vimax
         end if

*        ---------------------------------------------------------------
*        Output PUNCH, DUMP, SOLUTION and/or REPORT files.
*        ---------------------------------------------------------------
         if (iPnch .gt. 0)
     $   call s4pnch( iPnch, n, nb, hs, bl, xs, Names,
     $                cw, lencw, iw, leniw )

         if (iDump .gt. 0)
     $   call s4dump( iDump, nb, hs, xs, Names,
     $                cw, lencw, iw, leniw )

         piNorm = pNorm1
         if (iSoln .gt. 0)
     $   call s4soln( .true., minimz, m, n, nb, nName, 
     $                nnObj, nS,
     $                itn, nInf, sInf,
     $                iObj, Objtru, piNorm,
     $                ne, nka, a, ha, ka,
     $                hs, aScale, bl, bu,
     $                rw(lgObj), pi, rc, xs, y,
     $                Names, istate,
     $                cw, lencw, iw, leniw, rw, lenrw )

         if (iReprt .gt. 0)
     $   call s4rept( .true., m, n, nb, nName,
     $                nnL, nnObj, nS,
     $                ne, nka, a, ha, ka,
     $                hs, aScale, bl, bu,
     $                rw(lgObj), pi, xs, y,
     $                Names, istate,
     $                iw, leniw )
         piNorm = pNorm2
      else
*        ---------------------------------------------------------------
*        job = 'P'rint.    Print solution if requested.
*
*        lprSol = 0   means   no
*               = 1   means   if optimal, infeasible or unbounded
*               = 2   means   yes
*               = 3   means   if error condition
*        ---------------------------------------------------------------
         prnt   = iPrint .gt. 0  .and.  lprSol .gt. 0
         if ((lprSol .eq. 1  .and.  ierror .gt. 2)  .or.
     $       (lprSol .eq. 3  .and.  ierror .le. 2)) prnt = .false.

         if ( prnt ) then
            piNorm = pNorm1
            call s4soln( .false., minimz, m, n, nb, nName, 
     $                   nnObj, nS,
     $                   itn, nInf, sInf,
     $                   iObj, Objtru, piNorm,
     $                   ne, nka, a, ha, ka,
     $                   hs, aScale, bl, bu,
     $                   rw(lgObj), pi, rc, xs, y,
     $                   Names, istate,
     $                   cw, lencw, iw, leniw, rw, lenrw )
            piNorm = pNorm2
            if (iSumm  .gt. 0) write(iSumm, 1200) iPrint
         else
            if (iSumm  .gt. 0) write(iSumm, 1300)
         end if
      end if

      return

 1010 format(  ' Norm of x   (scaled)', 1p, e17.1,
     $     2x, ' Norm of pi  (scaled)',     e17.1)
 1020 format(  ' Norm of x ', 1p, e27.1,
     $     2x, ' Norm of pi',     e27.1)
 1030 format(  ' Max Prim inf(scaled)', i9, 1p, e8.1,
     $     2x, ' Max Dual inf(scaled)', i9,     e8.1)
 1040 format(  ' Max Primal infeas   ', i9, 1p, e8.1,
     $     2x, ' Max Dual infeas     ', i9,     e8.1)
 1080 format(  ' Nonlinear constraint violn', 1p, e11.1)

 1100 format(2a4)
 1200 format(/ ' Solution printed on file', i4)
 1300 format(/ ' Solution not printed')

*     end of s8savB
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8solv( start, 
     $                   m, n, nb, ne, nka, nName,
     $                   iObj, ObjAdd, fObj, Objtru,
     $                   nInf, sInf,
     $                   sfgCon, fgCon, sfgObj, fgObj,
     $                   a, ha, ka, bl, bu, Names, 
     $                   hs, xs, pi, rc,
     $                   inform, nMajor, nS,
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      external           sfgCon, fgCon, sfgObj, fgObj
      character*8        Names(nName)
      character*(*)      start
      integer            ha(ne), hs(nb)
      integer            ka(nka)
      double precision   a(ne), bl(nb), bu(nb)
      double precision   xs(nb), pi(m), rc(nb)

      character*8        cu(lencu), cw(lencw)
      integer            iu(leniu), iw(leniw)
      double precision   ru(lenru), rw(lenrw)

*     ==================================================================
*     s8solv solves the current problem.
*
*     On entry,
*     the specs file has been read,
*     all data items have been loaded (including a, ha, ka, ...),
*     and workspace has been allocated within z.
*
*     On exit,
*     inform  =  0 if an optimal solution was found,
*             =  1 if the problem was infeasible,
*             =  2 if the problem was unbounded,
*             =  3 if the Iteration limit was exceeded,
*            ge  4 if iterations were terminated by some other
*                  error condition (see the SNOPT user's guide).
*
*     15 Nov 1991: First version based on Minos 5.4 routine misolv.
*     13 Feb 1994: Eliminated "Cycle" options.
*                Simplified s4getb.
*     12 Nov 1994: Integer workspace added.
*     25 Jul 1996: Sign of the slacks changed. 
*     28 Sep 1997: Character workspace added.
*     11 Nov 1997: Backtracking for undefined functions.
*     26 Dec 1997: Dummy Jacobian scaled in feasibility phase.
*     07 Feb 1998: Current version of s8solv.
*     ==================================================================
      character*1        ch1
      logical            FP
      logical            infsbl, linFea
      logical            nlncon, nlnObj, nonlin
      logical            needB , done
      external           s8Ix
      parameter         (zero = 0.0d+0, one = 1.0d+0, ten = 10.0d+0)
      character*4        istate(3)
      parameter         (mtry = 1)

      integer            sclObj
      parameter         (sclObj    = 188)

      parameter         (lvlScl    =  75)
      parameter         (nParPr    =  94)
      parameter         (lvlDer    =  71)
      parameter         (lvlDif    = 182)
      parameter         (nfCon1    = 189)
      parameter         (nfCon2    = 190)
      parameter         (nfCon3    = 191)
      parameter         (nfCon4    = 192)
      parameter         (nfObj1    = 194)
      parameter         (nfObj2    = 195)
      parameter         (nfObj3    = 196)
      parameter         (nfObj4    = 197)
      parameter         (ifDefH    = 200)
      parameter         (nFac      = 210)

      parameter         (mName     =  51)
*     ------------------------------------------------------------------
      iPrint    = iw( 12)
      iSumm     = iw( 13)
      iLoadB    = iw(122)
      iNewB     = iw(124)
      insrt     = iw(125)
      iOldB     = iw(126)

      neJac     = iw( 20)
      nnCon     = iw( 21)
      nnJac     = iw( 22)
      nnObj     = iw( 23)
      nnL       = iw( 24)

      lenr      = iw( 28)
      maxS      = iw( 57)

      lvlExi    = iw( 72)
      lvlSch    = iw( 76)

      minmax    = iw( 87)
      iCrash    = iw( 88)
      itnlim    = iw( 89)
      MnrPrt    = iw( 93)
      MjrPrt    = iw( 92)
      minimz    = iw(199)

      mBS       = m     + maxS
      nkg       = nnJac + 1

      eps0      = rw(  2)

      tolFP     = rw( 51)
      tolQP     = rw( 52)
      tolx      = rw( 56)
      tCrash    = rw( 62)
      plInfy    = rw( 70)
      bigfx     = rw( 71)
      vilim     = rw( 81)
      wtInf0    = rw( 88)

*     Addresses

      lhElas    = iw(260)
      lhfeas    = iw(267)
      lhEsta    = iw(268)
      lkBS      = iw(269)
      lblBS     = iw(270)
      lbuBS     = iw(271)
      lxBS      = iw(272)
      lgBS      = iw(273)
      laScal    = iw(274)
      lx0       = iw(276)
      lr        = iw(278)
      liy       = iw(279)
      liy2      = iw(280)
      ly        = iw(281)
      ly2       = iw(282)
      lw        = iw(283)
      ly1       = iw(284)
      lrhs      = iw(290)
      lHp       = iw(295)
      lxdif     = iw(296)
      lgdif     = iw(297)
      lgObj     = iw(298)
      lgObj1    = iw(300)
      lgObj2    = iw(301)
      lfCon     = iw(302)
      lfCon1    = iw(303)
      lfCon2    = iw(304)
      lgCon     = iw(305)
      lgCon2    = iw(307)
      lkg       = iw(312)
      lviol     = iw(313)
      lblTru    = iw(314)
      lbuTru    = iw(315)
      lxMul     = iw(316)
      lxMul2    = iw(317)
      lxdMul    = iw(318)
      lxPen     = iw(319)
      lyslk     = iw(320)
      lxfeas    = iw(321)

      FP     = minmax .eq. 0

      nlnObj = nnObj  .gt. 0
      nnObj0 = max( nnObj, 1 )

      nlnCon = nnCon  .gt. 0
      nonlin = nnL    .gt. 0

      nnCon0 = max( nnCon, 1 )
      nnL0   = max( nnL  , 1 )

*     Count the linear equality and inequality constraints.

      numLEQ = 0
      do 50, j = n+nnCon+1, nb
         if (bl(j) .eq. bu(j)) numLEQ = numLEQ + 1
   50 continue

      numLC  = m     - nnCon
      numLIQ = numLC - numLEQ

*     Initialize xMul from pi.
*     Zap the linear pi(i) in case they are printed without being set.

      if (nlnCon      ) call dcopy ( nnCon, pi, 1, rw(lxMul), 1 )
      if (numLC .gt. 0) call dload ( numLC, zero, pi(nnCon+1), 1 )

*     Initialize a few things.

      if (nnL .lt. n) then
         iw(ifDefH) = 0  ! SQP Hessian is positive semi-definite
      else
         iw(ifDefH) = 1  ! SQP Hessian is positive      definite
      end if

      ierror     = 0
      iw(lvlDif) = 1
      iw(nFac)   = 0
      nInf       = 0
      wtInf      = one

      if (iw(lvlScl) .eq. 0) then 
         nScl = 1
      else
         nScl = nb
      end if      

      duInf      = zero
      fMrt       = zero
      fObj       = zero
      Objtru     = zero
      PenNrm     = zero
      call iload ( 4, 0, iw(nfCon1), 1 )
      call iload ( 4, 0, iw(nfObj1), 1 )

      rw(sclObj) = one

      itn        = 0
      nDegen     = 0
      nMajor     = 0
      nMinor     = 0

      lblslk     = lblTru + n
      lbuslk     = lbuTru + n

*     ==================================================================
*     Decode 'start'.
*     ==================================================================
      needB  = .true.
      ch1    = start(1:1)

      if      (ch1 .eq. 'C'  .or.  ch1 .eq. 'c'  .or.
     $         ch1 .eq. 'B'  .or.  ch1 .eq. 'b') then
*        --------------------------------
*        Cold start  or  Basis file.
*        --------------------------------
         iStart = 0
         needB  = max( ioldb, insrt, iloadB ) .le. 0
         nS     = 0

      else if (ch1 .eq. 'W'  .or.  ch1 .eq. 'w') then
*        --------------------------------
*        Warm start.
*        --------------------------------
         istart = 1
         needB  = .false.
         
      else
         istart = 0
         if (iPrint .gt. 0) write(iPrint, 1030) start
         if (iSumm  .gt. 0) write(iSumm , 1030) start
      end if

*     ------------------------------------------------------------------
*     Fiddle with partial price parameter to avoid foolish values.
*     Reduce nParPr if both the row and column section sizes
*     would be smaller than minprc (= 10 say).
*     ------------------------------------------------------------------
      minprc = 10
      npr1   = n / iw(nParPr)
      npr2   = m / iw(nParPr)
      if (max( npr1, npr2 ) .lt. minprc) then
         maxmn      = max( m, n )
         iw(nParPr) = maxmn / min( maxmn, minprc )
         npr1       = n / iw(nParPr)
         npr2       = m / iw(nParPr)
      end if

      if (iw(lvlScl) .eq. 1  .and.  nnL .eq. n) iw(lvlScl) = 0

      if (iPrint .gt. 0) then
         write(iPrint, 1100) iw(lvlScl), iw(nParPr), npr1, npr2
      end if
      if (iSumm  .gt. 0) then
         write(iSumm , 1110) iw(lvlScl), iw(nParPr)
      end if

*     ------------------------------------------------------------------
*     Set the vector of row types and print the matrix statistics.
*     ------------------------------------------------------------------
      call s2Amat( 'Statistics too', 
     $             m, n, nb,
     $             nnCon, nnJac, nnObj, iObj,
     $             ne, nka, a, ha, ka,
     $             bl, bu, iw(lhEsta),
     $             iw, leniw, rw, lenrw )

*     ------------------------------------------------------------------
*     Construct column pointers for the Jacobian.
*     Copy A into gCon and gCon2.  This has the effect of loading
*     constant Jacobian elements in gCon and gCon2, ready to be scaled.
*     If lvlDer is 2 or 3, make a permanent copy of A in gConU.
*     Load the nonlinear part of A with dummy (random) elements.
*     ------------------------------------------------------------------
      if ( nlnCon ) then
         call s8aux ( 'Assemble gCon column pointers', 
     $                nb, inform,
     $                ne, nka, a, ha, ka,
     $                bl, bu, xs, iw, leniw, rw, lenrw )
         call s8aux ( 'Load constant Jacobian elements',
     $                nb, inform,
     $                ne, nka, a, ha, ka,
     $                bl, bu, xs, iw, leniw, rw, lenrw )
         call s8aux ( 'Fake', nb, inform,
     $                ne, nka, a, ha, ka,
     $                bl, bu, xs, iw, leniw, rw, lenrw )
      end if

      call s1page( 1, iw, leniw )

      if (istart .eq. 0) then
*        ---------------------------------------------------------------
*        Cold start, or Basis file provided.
*        Input a basis file if one exists, thereby defining hs and xs.
*        (Otherwise, s2crsh will be called later to define hs.)
*        ---------------------------------------------------------------
*        We have to initialize xs(n+1:nb) and pi(nnCon+1:m)
*        before the problem is scaled.
*        The basis files initialize all of xs.
*        One day they may load pi for nonlinear problems.

         call dload ( m      , zero, xs(n+1)    , 1 )
         if (nnCon .lt. m)
     $   call dload ( m-nnCon, zero, pi(nnCon+1), 1 )

         if (iPrint .gt. 0) write(iPrint, 1200)

         if ( needB ) then
            if (iPrint .gt. 0) write(iPrint, 1205)

            if (icrash .eq. 0) then
               needB  = .false.
               lcrash = 0
               call s2crsh( lcrash, MjrPrt, m, n, nb,
     $                      iCrash, tCrash,
     $                      ne, nka, a, ha, ka,
     $                      iw(lkBS), hs, iw(lhEsta),
     $                      bl, bu, xs,
     $                      iw, leniw, rw, lenrw )
            end if
         else
            call s4getB( ierror, m, n, nb, nName, nS, iObj, 
     $                   hs, bl, bu, xs, Names,
     $                   iw, leniw, rw, lenrw )
            if (ierror .ne. 0) go to 800
         end if

      else if (istart .eq. 1) then
        if (iPrint .gt. 0) write(iPrint, 1210)
      end if ! istart = 0

*     ------------------------------------------------------------------
*     Move xs inside its bounds.
*     Save the nonlinear part of xs in x0. 
*     x0 = Proximal Point base point.
*     ------------------------------------------------------------------
      do 120, j = 1, n
         xs(j)   = max( xs(j), bl(j) )
         xs(j)   = min( xs(j), bu(j) )
  120 continue

*     ------------------------------------------------------------------
*     Scale the linear constraints.
*     (The nonlinear elements of A contain fake nonzeros.) 
*     ------------------------------------------------------------------
      if (iw(lvlScl) .gt. 0  .and.  numLC .gt. 0) then
         call s2scal( MjrPrt, m, n, nb, nnL, nnCon, nnJac, iw(lhEsta), 
     $                ne, nka, a, ha, ka,
     $                rw(laScal), bl, bu, rw(ly), rw(ly2),
     $                iw, leniw, rw, lenrw )
         call s2scla( 'Scale', m, n, nb,
     $                iObj, plInfy, rw(sclObj),
     $                ne, nka, a, ha, ka, 
     $                rw(laScal), bl, bu, pi, xs )
      end if

*     Save all the bounds.

      call dcopy ( nb, bl, 1, rw(lblTru), 1 )
      call dcopy ( nb, bu, 1, rw(lbuTru), 1 )

      if (nnL .gt. 0) 
     $   call dcopy ( nnL, xs, 1, rw(lx0), 1 )

*     ------------------------------------------------------------------
*     Prepare to get feasible for the linear constraints.
*     Relax any nonlinear rows. 
*     Allow the linear rows to be elastic.
*     ------------------------------------------------------------------
      if (nlnCon  .and.  numLC .gt. 0) then
         call dload ( nnCon, (-plInfy), bl(n+1), 1 )
         call dload ( nnCon,   plInfy , bu(n+1), 1 )
      end if

*     ------------------------------------------------------------------
*     Compute a starting basis.
*     This may require getting feasible for the linear equality rows.
*     No variables are elastic yet. 
*     ------------------------------------------------------------------
      call iload ( nb, 0, iw(lhElas), 1 )
      lenb   = 0
      call s5getB( needB, ierror, lenb, m, maxS, mBS, n, nb,
     $             nnCon, nnJac, nnObj, iObj,
     $             nS, nScl, nDegen, numLC, numLEQ, numLIQ,
     $             itnlim, nMinor, itn, MnrPrt, 
     $             ObjAdd, tolFP, tolQP, tolx,
     $             nInf, sInf, wtInf, piNorm, rgNorm, xNorm,
     $             ne, nka, a, ha, ka,
     $             iw(lhElas), iw(lhEsta), iw(lhfeas), hs, iw(lkBS),
     $             rw(laScal), rw(lrhs), bl, bu,
     $             rw(lblBS), rw(lbuBS), rw(lblslk), rw(lbuslk),
     $             rw(lgBS), pi, rc, rw(lrhs), 
     $             xs, rw(lxBS),
     $             iw(liy), iw(liy2), rw(ly), rw(ly2), rw(ly1),
     $             cw, lencw, iw, leniw, rw, lenrw )

*     ==================================================================
*     Satisfy the linear constraints.
*     The norm of xs is minimized via a proximal-point QP.
*     If no feasible point can be found, the linear rows can be elastic.
*     ==================================================================
      if (numLC .gt. 0) then
         if (ierror .eq. 0) then
            call s8feas( ierror, m, mBS, n, nb, nnCon, nnL, nScl,
     $                   nDegen, numLC, numLIQ,
     $                   itn, itnlim, nMinor, MnrPrt, tolQP, tolx,
     $                   nInf, sInf, wtInf, piNorm, rgNorm,
     $                   ne, nka, a, ha, ka,
     $                   iw(lhElas), iw(lhEsta), iw(lhfeas), hs,
     $                   iw(lkBS), rw(laScal), rw(lrhs),
     $                   bl, bu, rw(lblTru), rw(lbuTru), rw(lblBS),
     $                   rw(lbuBS), rw(lgBS), pi, rc, rw(lx0), xs,
     $                   rw(lxBS), iw(liy), iw(liy2),
     $                   rw(ly), rw(ly1), rw(ly2),
     $                   cw, lencw, iw, leniw, rw, lenrw )
         end if
*        ---------------------------------------------------------------
*        Reinstate the nonlinear constraints.
*        ---------------------------------------------------------------
         if ( nlnCon ) then
               
*           Restore the bounds on the nonlinear constraints.
               
            call dcopy ( nnCon, rw(lblslk), 1, bl(n+1), 1 )
            call dcopy ( nnCon, rw(lbuslk), 1, bu(n+1), 1 )

*           Restore the constant Jacobian elements in gCon and gCon2.

            call s8aux ( 'Restore constants', nb, inform,
     $                   ne, nka, a, ha, ka,
     $                   bl, bu, xs, iw, leniw, rw, lenrw )
         end if ! nlnCon

*        ---------------------------------------------------------------
*        Unscale the linear constraints.
*        ---------------------------------------------------------------
         if (iw(lvlScl) .gt. 0) then
            call s2scla( 'Unscale', m, n, nb,
     $                   iObj, plInfy, rw(sclObj),
     $                   ne, nka, a, ha, ka,
     $                   rw(laScal), bl, bu, pi, xs )
         end if
      end if ! numLC > 0

      linFea = nInf .eq. 0

      if (ierror .gt. 0) go to 800

*     ------------------------------------------------------------------
*     Reset hElast so that only nonlinear rows are elastic.
*     Make sure variables are not outside their bounds
*     (in particular, check the nonlinear slacks).
*     ------------------------------------------------------------------
      if ( nlncon ) call iload ( nnCon, 3, iw(lhElas+n), 1 )

      do 130, j = 1, nb
         xs(j)  = max( xs(j), bl(j) )
         xs(j)  = min( xs(j), bu(j) )
  130 continue

*     ==================================================================
*     ==================================================================
*     The linear constraints have been satisfied!
*     Compute the problem functions at this all-important point.
*     No scaling yet.
*     ==================================================================
*     ==================================================================

      if (nnL .gt. 0) then
         lsSave     = iw(lvlScl)
         iw(lvlScl) = 0
      
         call s6init( ierror, n,
     $                nnCon, nnCon0, nnJac, neJac,
     $                nnL, nnObj, nnObj0,
     $                sfgCon, fgCon, sfgObj, fgObj,
     $                ne, nka, ha, ka,
     $                fObj, rw(lfCon), rw(lgCon), rw(lgObj), xs,
     $                cu, lencu, iu, leniu, ru, lenru, 
     $                cw, lencw, iw, leniw, rw, lenrw )
         if (ierror .ne. 0) go to 800

*        ------------------------------------------------------------------
*        Check gradients.
*        (One day, we should do this on the SCALED problem.)
*        ------------------------------------------------------------------
         if ( nlncon ) then
            call s7chkJ( ierror, n, nnCon, nnJac, neJac, nb,
     $                   sfgCon, fgCon, 
     $                   ne, nka, ha, ka,
     $                   bl, bu, rw(lfCon), rw(lfCon2),
     $                   rw(lgCon), rw(lgCon2),
     $                   xs, rw(ly), rw(ly2), 
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )
            if (ierror .ne. 0) go to 800
         end if

         if ( nlnObj ) then
            call s7chkg( ierror, nnObj,
     $                   sfgObj, fgObj,
     $                   bl, bu, fObj, rw(lgObj), rw(lgObj2),
     $                   xs, rw(ly), rw(ly2), 
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )
            if (ierror .ne. 0) go to 800
         end if

*        ---------------------------------------------------------------
*        Compute any missing derivatives.
*        Load the Jacobian in A.
*        ---------------------------------------------------------------
         call s6fd  ( ierror,
     $                n, neJac, nnCon, nnCon0, nnJac, nnObj,
     $                sfgCon, fgCon, sfgObj, fgObj,
     $                ne, nka, ha, ka, fObj, xs, rw(lw), 
     $                cu, lencu, iu, leniu, ru, lenru, 
     $                cw, lencw, iw, leniw, rw, lenrw )
         if (ierror .ne. 0) go to 800

         if ( nlnCon ) then
            call s8setJ( nb, nnCon, nnJac, neJac, eps0,
     $                   ne, nka, a, ha, ka, 
     $                   xs, iw, leniw, rw, lenrw )
         end if

         iw(lvlScl) = lsSave
      end if

*     ==================================================================
*     Scale the problem.
*     ==================================================================
      if (iw(lvlScl) .gt. 0) then
*        ---------------------------------------------------------------
*        Reset the vector of row types.
*        ---------------------------------------------------------------
         call s2amat( 'Define row types',
     $                m, n, nb,
     $                nnCon, nnJac, nnObj, iObj,
     $                ne, nka, a, ha, ka,
     $                bl, bu, iw(lhfeas),
     $                iw, leniw, rw, lenrw )
         call s2scal( MjrPrt, m, n, nb, nnL, nnCon, nnJac, iw(lhfeas), 
     $                ne, nka, a, ha, ka,
     $                rw(laScal), bl, bu, rw(ly), rw(ly2),
     $                iw, leniw, rw, lenrw )
         call s2scla( 'Scale', m, n, nb,
     $                iObj, plInfy, rw(sclObj),
     $                ne, nka, a, ha, ka, 
     $                rw(laScal), bl, bu, pi, xs )

*        ---------------------------------------------------------------
*        The objective and constraint functions haven't been scaled yet.
*        Don't forget the initial pi.
*        ---------------------------------------------------------------
         if (nnCon .gt. 0) then
            call dddiv ( nnCon, rw(laScal+n), 1, rw(lfCon), 1 )
            call s8sclJ( nnCon, nnJac, neJac, n, ne, nka, 
     $                   rw(laScal), ha, ka, rw(lgCon),
     $                   iw, leniw, rw, lenrw )
            call ddscl ( nnCon, rw(laScal+n), 1, rw(lxMul), 1 )
         end if

         if (nnObj .gt. 0) then
            call s8sclg( nnObj, rw(laScal), rw(lgObj), 
     $                   iw, leniw, rw, lenrw )
         end if
      end if ! iw(lvlScl) > 0

      call s1time( 2, 0, iw, leniw, rw, lenrw )

      nTry   = 0
      done   = .false.

*     ==================================================================
*+    while (.not. done  .and.  ntry .le. mtry) do                    
  200 if    (.not. done  .and.  ntry .le. mtry) then

         ierror = 0
         nMajor = 0
         if (ntry .gt. 0) nMinor = 0 

         if ( nlnCon ) then
*           ============================================================
*           s8nslk temporarily loads rc with the nonlinear slacks.
*           Copy these into xs(n+i) and make sure they are feasible.
*           Crash uses them to decide which slacks to grab for the basis
*           If any nonbasic nonlinear slacks are close to a bound,
*           move them exactly onto the bound to avoid very small steps.
*           ============================================================
            call s8nslk( n, nnCon, nnJac, eps0,
     $                   maxvi, vimax, virel, xNorm,
     $                   ne, nka, a, ha, ka,
     $                   bl, bu, rw(lfCon), xs, rw(lyslk) )
            viSup = max( ten*vimax, vilim )

            call dcopy ( nnCon, rw(lyslk), 1, xs(n+1), 1 )

            do 210, j = n+1, n+nnCon
               xj     = max( xs(j), bl(j) )
               xj     = min( xj   , bu(j) )
               if (hs(j) .le. 1) then
                  if (xj .le. bl(j) + tolx) xj = bl(j)
                  if (xj .ge. bu(j) - tolx) xj = bu(j)
               end if
               xs(j)   = xj
  210       continue

*           ============================================================
*           Crash on the nonlinear rows.
*           hs(*) already defines a basis for the full problem,  but we
*           want to do better by not including all of the slacks.
*           ============================================================
            if ( needB ) then

*              Load  hfeas  with the row types.
*              s2crsh uses kBS as workspace.  It may alter xs(n+i) for
*              nonlinear slacks.

               call s2amat( 'Define row types',
     $                      m, n, nb,
     $                      nnCon, nnJac, nnObj, iObj,
     $                      ne, nka, a, ha, ka,
     $                      bl, bu, iw(lhfeas),
     $                      iw, leniw, rw, lenrw )
               lcrash = 5
               call s2crsh( lcrash, MjrPrt, m, n, nb,
     $                      iCrash, tCrash,
     $                      ne, nka, a, ha, ka,
     $                      iw(lkBS), hs, iw(lhfeas),
     $                      bl, bu, xs,
     $                      iw, leniw, rw, lenrw )
               needB = .false.
            end if ! needB
         end if ! nlnCon

*        ---------------------------------------------------------------
*        Solve the problem.
*        ---------------------------------------------------------------
         call s1page( 1, iw, leniw )

         if ( FP ) then
*           ------------------------------------------------------------
*           Find a feasible point for the nonlinear constraints.
*           Minimize 1/2 norm(x)**2
*           ------------------------------------------------------------
            iObjt  = 0
            nnObjt = nnJac
            nnObj0 = max( nnJac, 1 )
            call dcopy ( nnObjt, xs, 1, rw(lgObj), 1 )
            fObjt  = 0.5d+0*ddot ( nnObjt, xs, 1, xs, 1 )
            
            call s8core( FP, ierror, iObjt, itn, lenr, 
     $                   m, maxS, mBS, n, nb, nS, 
     $                   nnCon, nnCon0, nnObjt, nnObj0, nnJac, nnObj0,
     $                   nMajor, nMinor, nDegen,
     $                   sfgCon, fgCon, sfgObj, s8Ix,
     $                   duInf, minimz, zero, fObjt, fMrt,
     $                   vimax, virel, viSup, nInf, sInf,
     $                   wtInf0, wtInf, PenNrm, piNorm, xNorm,
     $                   ne, nka, a, ha, ka,
     $                   neJac, nkg, iw(lkg), 
     $                   iw(lhElas), iw(lhEsta),
     $                   iw(lhfeas), hs, iw(lkBS),
     $                   bl, bu, rw(lblBS), rw(lbuBS),
     $                   rw(lfCon), rw(lgCon), rw(lgObj),
     $                   rw(lviol), rw(lyslk), rw(lfCon1), rw(lgObj1),
     $                   rw(lfCon2), rw(lgCon2), rw(lgObj2),
     $                   rw(lgdif), rw(lHp), pi, rw(lr), rc,
     $                   xs, rw(lx0), rw(lxBS), rw(lxdif), rw(lxfeas),
     $                   rw(lxMul), rw(lxMul2), rw(lxdMul), rw(lxPen),
     $                   iw(liy), iw(liy2),
     $                   rw(ly), rw(ly1), rw(ly2), rw(lw),
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )
         else
*           ------------------------------------------------------------
*           Solve the NLP problem.
*           ------------------------------------------------------------
            call s8core( FP, ierror, iObj, itn, lenr, 
     $                   m, maxS, mBS, n, nb, nS, 
     $                   nnCon, nnCon0, nnObj, nnObj0, nnL, nnL0,
     $                   nMajor, nMinor, nDegen,
     $                   sfgCon, fgCon, sfgObj, fgObj,
     $                   duInf, minimz, ObjAdd, fObj, fMrt,
     $                   vimax, virel, viSup, nInf, sInf,
     $                   wtInf0, wtInf, PenNrm, piNorm, xNorm,
     $                   ne, nka, a, ha, ka,
     $                   neJac, nkg, iw(lkg), 
     $                   iw(lhElas), iw(lhEsta),
     $                   iw(lhfeas), hs, iw(lkBS),
     $                   bl, bu, rw(lblBS), rw(lbuBS),
     $                   rw(lfCon), rw(lgCon), rw(lgObj),
     $                   rw(lviol), rw(lyslk), rw(lfCon1), rw(lgObj1),
     $                   rw(lfCon2), rw(lgCon2), rw(lgObj2),
     $                   rw(lgdif), rw(lHp), pi, rw(lr), rc,
     $                   xs, rw(lx0), rw(lxBS), rw(lxdif), rw(lxfeas),
     $                   rw(lxMul), rw(lxMul2), rw(lxdMul), rw(lxPen),
     $                   iw(liy), iw(liy2),
     $                   rw(ly), rw(ly1), rw(ly2), rw(lw),
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )
         end if

         if (ierror .eq.  9  .and.  abs(fObj) .gt. bigfx) ierror = 2
         if (ierror .ge. 30                      ) go to 800
         if (ierror .ge. 20  .and.  itn    .eq. 0) go to 800
         
         done = lvlExi .eq. 0  
     $    .or.  FP
     $    .or.  ierror .eq. 0  .and.  nInf .eq. 0  ! Already feasible
     $    .or.  ierror .eq. 3  .and.  itn  .ge. itnlim  ! too many itns
     $    .or.  ierror .eq. 6                      ! User wants to stop

         if (.not. done) then
            FP = .true.
            if (iPrint .gt. 0) write(iPrint, 2000) itn
            if (iSumm  .gt. 0) write(iSumm , 2000) itn
         end if

         nTry = nTry + 1
         go to 200
      end if
*+    end while

      call s1time(-2, 0, iw, leniw, rw, lenrw )
 
*     ==================================================================
*     Exit.
*     Set output variables and print a summary of the final solution.
*     ==================================================================
  800 if (ierror .lt. 0) then
         ierror = 6
         if (nnCon .lt. m) then
            if (iPrint .gt. 0) write(iPrint, 8060)
            if (iSumm  .gt. 0) write(iSumm , 8060)
         else
            if (iPrint .gt. 0) write(iPrint, 8065)
            if (iSumm  .gt. 0) write(iSumm , 8065)
         end if
      end if
         
      inform  = ierror
      degen   = 100.0d+0 * nDegen / max( itn, 1 )

      if (iObj .eq. 0) then
         flin = ObjAdd
      else
         flin = ObjAdd + xs(n+iObj)*rw(sclObj)
      end if
      Objtru  = flin + fObj

*     Save some things needed by the AMPL interface.

      iw(400) = itn
      rw(400) = ObjTru

      infsbl  = nInf .gt. 0
      xNorm   = dnrm1s( nb, xs, 1 )
      
*     Count basic nonlinear variables (used only for printing).

      nnb    = 0
      do 810, j = 1, nnL
         if (hs(j) .eq. 3) nnb = nnb + 1
  810 continue

      if (inewB .gt. 0  .and.  ierror .lt. 20) then
         k      = 1 + ierror
         call s4stat( k, istate )
         call s4newB( 2, inewB, minimz,
     $                m, n, nb, nS, mBS, Objtru,
     $                itn, nInf, sInf,
     $                iw(lkBS), hs, rw(laScal), bl, bu, rw(lxBS), 
     $                xs, istate,
     $                cw, lencw, iw, leniw )
      end if

*     Print statistics.

      if (iPrint .gt. 0) then
                     write(iPrint, 1900) cw(mName), itn, Objtru
         if (infsbl) then
                     write(iPrint, 1910) nInf, sInf
            if (linFea) 
     $               write(iPrint, 1915) wtInf, fMrt/wtInf
         end if
         if (nonlin) write(iPrint, 1920) nMajor, flin, PenNrm, fObj
         if (nonlin) write(iPrint, 1950) iw(nfObj1), iw(nfCon1)
         if (iw(lvlDer) .lt. 3  .or.  (nonlin  .and.  lvlSch .eq. 0))
     $               write(iPrint, 1955) iw(nfObj2), iw(nfCon2)
         if (iw(lvlDer) .lt. 3)
     $               write(iPrint, 1960) iw(nfObj3), iw(nfCon3),
     $                                   iw(nfObj4), iw(nfCon4)
         if (nS .gt. 0)
     $               write(iPrint, 1970) nS, nnb
                     write(iPrint, 1975) nDegen, degen
      end if

      if (iSumm  .gt. 0) then
                     write(iSumm , 1900) cw(mName), itn, Objtru
         if (infsbl) then
                     write(iSumm , 1910) nInf, sInf
            if (linFea) 
     $               write(iSumm , 1915) wtInf, fMrt/wtInf
         end if
         if (nonlin) write(iSumm , 1920) nMajor, flin, PenNrm, fObj
         if (nonlin) write(iSumm , 1950) iw(nfObj1), iw(nfCon1)
         if (iw(lvlDer) .lt. 3  .or.  (nonlin  .and.  lvlSch .eq. 0))
     $               write(iSumm , 1955) iw(nfObj2), iw(nfCon2)
         if (iw(lvlDer) .lt. 3)
     $               write(iSumm , 1960) iw(nfObj3), iw(nfCon3),
     $                                   iw(nfObj4), iw(nfCon4)
         if (nS .gt. 0)
     $               write(iSumm , 1970) nS, nnb
                     write(iSumm , 1975) nDegen, degen
      end if

*     ------------------------------------------------------------------
*     Unscale, compute nonlinear constraint violations,
*     save basis files and prepare to print the solution.
*     Clock 3 is "Output time".
*     ------------------------------------------------------------------
      call s1time( 3, 0, iw, leniw, rw, lenrw )

*     Skip the functions if the linear constraints are infeasible.
*     Skip unscaling everything too, since infeasible linear constraints
*     have already been unscaled.

      lsSave     = iw(lvlScl)

      if (linfea) then
         nnCon1 = nnCon
         nnJac1 = nnJac
         nnObj1 = nnObj
      else
         nnCon1 = 0
         nnJac1 = 0
         nnObj1 = 0
         iw(lvlScl) = 0
      end if

      call s8savB( 'Save', ierror, minimz, 
     $             m, n, nb, nnL,
     $             nnCon1, nnJac1, nnObj1,
     $             nName, nS, nScl, rw(sclObj),
     $             itn, nInf, sInf, wtInf,
     $             iObj, Objtru, vimax,
     $             pNorm1, pNorm2, piNorm, xNorm,
     $             ne, nka, a, ha, ka,
     $             iw(lhEsta), hs, rw(laScal), bl, bu,
     $             Names, pi, rc, xs, rw(ly),
     $             cw, lencw, iw, leniw, rw, lenrw )

*     If task = 'Print', s8savB prints the solution under the control
*     of lprSol (set by the  Solution  keyword in the SPECS file).
*     The printed solution may or may not be wanted, as follows:
*     
*     lprSol = 0   means      No
*            = 1   means      If optimal, infeasible or unbounded
*            = 2   means      Yes
*            = 3   means      If error condition

      call s8savB( 'Print', ierror, minimz,
     $             m, n, nb, nnL,
     $             nnCon1, nnJac1, nnObj1,
     $             nName, nS, nScl, rw(sclObj),
     $             itn, nInf, sInf, wtInf,
     $             iObj, Objtru, vimax,
     $             pNorm1, pNorm2, piNorm, xNorm,
     $             ne, nka, a, ha, ka,
     $             iw(lhEsta), hs, rw(laScal), bl, bu,
     $             Names, pi, rc, xs, rw(ly), 
     $             cw, lencw, iw, leniw, rw, lenrw )
      iw(lvlScl) = lsSave
      call s1time(-3, 0, iw, leniw, rw, lenrw )

*     ------------------------------------------------------------------
*     If the user hasn't already pulled the plug,
*     call the functions one last time with  nState .ge. 2.
*     Everything has been  unscaled, so we have to disable scaling.
*     mode = 0  tells the functions that gradients are not required.
*     ------------------------------------------------------------------
      if (linfea  .and.  ierror .ne. 6) then
         lsSave = iw(lvlScl)
         iw(lvlScl) = 0
         nStat1 = 2 + ierror
         nStat2 = nStat1
         mode   = 0
         if ( nlncon ) then
            call sfgCon( mode, ierror, nStat2,
     $                   n, nnCon, nnJac, neJac, fgCon,
     $                   ne, nka, ha, ka,
     $                   rw(lfCon), rw(lgCon2), xs,
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )
         end if

         if ( nlnObj ) then
            call sfgObj( mode, 
     $                   ierror, nStat1, nnObj,
     $                   fgObj,
     $                   fObj, rw(lgObj2), xs, 
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )
         end if
         iw(lvlScl) = lsSave
         if (mode .ge. 0) then
            nStat1 = 0
            nStat2 = 0
         end if
      end if

      return

 1000 format(' ')
 1030 format(/   ' XXX Start parameter not recognized:  ', a)
 1100 format(/   ' Scale option', i3, ',      Partial price', i8
     $       /   ' Partial price section size ( A)', i12
     $       /   ' Partial price section size (-I)', i12)
 1110 format(/   ' Scale option', i3, ',    Partial price', i4)
 1200 format(    ' Initial basis' / ' -------------')
 1205 format(/   ' No basis file supplied')
 1210 format(    ' Warm Start' / ' ----------')
 1310 format(/1x, 32a4 
     $       /   ' Checking feasibility of the linear constraints')
 1315 format(/1x, 16a4 
     $       /   ' Checking feasibility of the linear constraints')
 1900 format(/   ' Problem name', 17x, a8
     $       /   ' No. of iterations', i20,
     $        2x,' Objective value', 1p, e22.10)
 1910 format(    ' No. of infeasibilities', i15,
     $        2x,' Sum of infeas', 1p, e24.10)
 1915 format(    ' Elastic weight            ', 1p, e11.1,
     $        2x,' Scaled Merit ', 1p, e24.10)
 1920 format(    ' No. of major iterations', i14,
     $        2x,' Linear objective', 1p, e21.10
     $       /   ' Penalty parameter', 1p, e20.3,
     $        2x,' Nonlinear objective', 1p, e18.10)
 1950 format(    ' No. of calls to funobj', i15,
     $        2x,' No. of calls to funcon', i15)
 1955 format(    ' Calls with modes 1,2 (known g)', i7,
     $        2x,' Calls with modes 1,2 (known g)', i7)
 1960 format(    ' Calls for forward differencing', i7,
     $        2x,' Calls for forward differencing', i7
     $       /   ' Calls for central differencing', i7,
     $        2x,' Calls for central differencing', i7)
 1970 format(    ' No. of superbasics', i19,
     $        2x,' No. of basic nonlinears', i14)
 1975 format(    ' No. of degenerate steps', i14,
     $        2x,' Percentage', f27.2)
 2000 format(/' Itn', i7, ': Making the nonlinear rows feasible'/)

 8010 format( ' Itn', i7, ' -- infeasible problem,',
     $                    ' nonlinear infeasibilities minimized')
 8020 format( ' Itn', i7, ' -- the problem is unbounded ',
     $                    ' (or badly scaled)')
 8021 format( ' Itn', i7, ' -- violation limit exceeded',
     $                    ' the problem may be unbounded')
 8031 format( ' Itn', i7, ' -- major iteration limit exceeded')
 8032 format( ' Itn', i7, ' -- minor iteration limit exceeded')
 8033 format( ' Itn', i7, ' -- iteration limit exceeded')
 8040 format( ' Itn', i7, ' -- requested accuracy could not',
     $                    ' be achieved')
 8050 format( ' Itn', i7, ' -- the superbasics limit is too small')
 8060 format(  ' EXIT -- undefined functions in funcon and funobj',
     $         ' at the first LC feasible point.')
 8065 format(  ' EXIT -- undefined functions in funcon and funobj',
     $         ' at the initial point.')
 8090 format( ' Itn', i7, ' -- the current point cannot be improved')

*     end of s8solv
      end

