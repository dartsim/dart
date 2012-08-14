*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
*     File  sn50lp.f
*
*     s5LP     s5bndE   s5BSx    s5dgen   s5getB   s5grdE   s5hs     
*     s5Inf    s5InfE   s5LPit   s5pric   s5rc     s5rcE    s5setE
*     s5setp   s5setx   s5step
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5LP  ( prbtyp,
     $                   contyp, Elastc, needLU, needx, ierror,
     $                   m, n, nb, nDegen, itMax, itLP, itn,
     $                   lEmode, lvlInf, lPrint,
     $                   minimz, ObjAdd, iObj, tolFP, tolLP, tolx,
     $                   nInf, sInf, wtInf, piNorm, rgNorm, xNorm,
     $                   ne, nka, a, ha, ka,
     $                   hElast, hEstat, hfeas, hs, kBS, aScale, 
     $                   b, lenb, bl, bu, blBS, buBS,
     $                   gBS, pi, rc, xBS, xs,
     $                   iy, iy2, y, y2, y1,
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      logical            Elastc, needLU, needx
      character          prbtyp*(*)
      character*20       contyp
      integer            ha(ne), hElast(nb), hEstat(nb), hs(nb)
      integer            ka(nka)
      integer            kBS(m+1), hfeas(m+1)
      integer            iy(m), iy2(m)
      double precision   a(ne), aScale(*), b(*)
      double precision   bl(nb), bu(nb), rc(nb), xs(nb)
      double precision   blBS(m+1), buBS(m+1), xBS(m+1)
      double precision   gBS(m), pi(m), y1(m)
      double precision   y(nb), y2(nb)

      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     s5LP   solves a linear program.
*
*     The problem type can be:
*       type = 'FP '  feasible point only
*       type = 'FPE'  feasible point for equalities only
*       type = 'FPS'  feasible point for QP subproblem
*       type = 'LP '  LP problem
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
*     kBS(1  :m )  holds the column indices of the basic variables.
*     Superbasics have been temporarily fixed at their current value.
*
*     30-Sep-91: First version of s5LP  based on Qpsol routine lpcore.
*     20-Jul-96: Slacks changed to be the row value.
*     06-Aug-96: First Min Sum version.
*     14-Jul-97: Thread-safe version.
*     07-Feb-98: Current version of s5LP.
*     ==================================================================
      logical            checkx, FP    , LP
      logical            incres, gotE  , gotg  , gotR, needpi
      logical            chkFea, jstFea, prtFea
      logical            newB  , newLU , ObjPhs
      logical            feasbl, optiml
      logical            Prnt1 , Prnt10, Srnt1
      integer            nfix(2)
      double precision   dummy(1)
      parameter         (zero = 0.0d+0)

      integer            toldj1,  toldj2, toldj3
      parameter         (toldj1    = 184)
      parameter         (toldj2    = 185)
      parameter         (toldj3    = 186)

      parameter         (lenL0     = 171)
      parameter         (lenU0     = 172)
      parameter         (lenL      = 173)
      parameter         (lenU      = 174)
      parameter         (LUitn     = 215)
      parameter         (LUmod     = 216)
      parameter         (kObj      = 220)
      parameter         (MnrHdg    = 223)
*     ------------------------------------------------------------------
      iPrint    = iw( 12)
      iSumm     = iw( 13)

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

      plInfy    = rw( 70)
      sclObj    = rw(188)

      FP        = prbtyp(1:2) .eq. 'FP'
      LP        = prbtyp(1:2) .eq. 'LP'

*     ------------------------------------------------------------------
*     s5LP operates in either ``Normal'' or ``Elastic'' mode.
*     Everything is normal unless a weighted sum is being minimized or
*     the constraints are infeasible.
*     The logical feasbl refers to the non-elastic variables.
*     Note that feasbl can be false while in elastic mode. 
*     wtInf  is the optional parameter Infeasibility Weight.
*     ------------------------------------------------------------------
      nnL    = 0
      nng    = 0
      nng0   = 1
      nnH    = 0
      nS     = 0                ! local value
      mBS    = m + 1

      Prnt1  = iPrint .gt. 0  .and.  lPrint .ge.  1
      Prnt10 = iPrint .gt. 0  .and.  lPrint .ge. 10
      Srnt1  = iSumm  .gt. 0  .and.  lPrint .ge.  1

      kPrc       = 0            ! last section scanned in part. pricing
      ierror     = 0
      LUreq      = 0
      iw(MnrHdg) = 0
      lines1     = 0
      lines2     = 0

      chkFea = .true.
      jstFea = .false.
      prtFea = .true.
      feasbl = .false.
      gotE   = .false.
      gotR   = .false.

      needpi = .true.
      newLU  = .true.
      ObjPhs = .false.
      optiml = .false.

      condHz = zero
      pivot  = zero
      rgnorm = zero
      step   = zero
      ObjLP  = zero
      nInfE  = 0
      jq     = 0
      djq    = zero
      jBq    = 0                ! xs(jBq) is the incoming   BS
      jBr    = 0                ! xs(jBr) is the outgoing   BS
      jBSr   = 0                ! xs(jBSr) leaves B in special BS swap
      jSq    = 0                ! xs(jSq) is the incoming SBS
      jSr    = 0                ! xs(jSr) is the outgoing SBS
      kPrPrt = 0
      sgnObj = minimz

      rw(toldj1) = 100.0d+0     ! Used only for LP partial pricing
      rw(toldj2) = 100.0d+0     ! 

      call s5hs  ( 'Internal', nb, bl, bu, hs, xs )
      call iload ( nb, 0, hEstat, 1 )
      call s5dgen( 'Initialize', inform, lPrint,
     $             nb, nInf, itn, 
     $             featol, tolx, tolinc, hs, bl, bu, xs,
     $             itnfix, nfix, tolx0,
     $             iw, leniw, rw, lenrw )

**    ======================Start of main loop==========================
*+    do while (.not. optiml  .and.  ierror .eq. 0)
  100 if       (.not. optiml  .and.  ierror .eq. 0) then
*        ===============================================================
*        Check the initial  xs  and move it onto  ( A -I )*xs = b.
*        If needLU is true, this will require a basis factorization.
*        ===============================================================
*        If necessary,  factorize the basis  ( B = LU ) and set xs.
*        If needLU is false on entry to s5LP, the first call to s2Bfac
*        will try to use existing factors.
*        If needLU is true on entry to s5LP, an LU factorization of 
*        type typeLU is computed.

         if (needx  .or.  needLU) then
            call s2Bfac( 'B ', needLU, newLU, newB,
     $                   ierror, iObj, itn, lPrint, LUreq,
     $                   m, mBS, n, nb, nnL, nS, nSwap, xNorm,
     $                   ne, nka, a, ha, ka, 
     $                   kBS, hs, b, lenb,
     $                   bl, bu, blBS, buBS, xBS, xs,
     $                   iy, iy2, y, y2, 
     $                   iw, leniw, rw, lenrw )
            LUsiz0    = iw(lenL0) + iw(lenU0)
            LUmax     = 2*LUsiz0

            if (ierror .gt. 0) go to 100

            gotE   = .false.    ! Reset hEstat in elastic mode.
            needpi = .true.     ! Recalculate the pi's.
            needx  = .false.
            chkFea = .true.

            if (lPrint .ge. 10) iw(MnrHdg) = 1
         end if

         nBS    = m + nS
         nInf   = 0
         sInf   = zero
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

         ObjLP  = zero
         if (iObj .ne. 0) then
            ObjLP  = xBS(iw(kObj))*sclObj
         end if

         if (jstFea  .and.  (FP .or. (Elastc .and. lvlInf .eq. 0))) then
*           ------------------------------------------------------------
*           The non-elastic variables just became feasible.
*           That is all that is needed.
*           ------------------------------------------------------------
            djqPrt = zero
            pinorm = zero
            call dload ( m, zero, pi, 1 )
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
               if ( ObjPhs ) then
                  if (iObj .ne. 0) then
                     gBS(iw(kObj)) = sgnObj*sclObj
                  end if
               end if

               if ( Elastc ) then
                  call s5grdE( nb, nBS, wtInf, hEstat, kBS, gBS )

                  if (jstFea  .or.  newLU) then 

*                    Change some of the blBS, buBS  bounds.
                     
                     call s5bndE( nb, nBS, plInfy, hEstat, kBS,
     $                            bl, bu, blBS, buBS )
                  end if
               end if
            end if

            if ( needpi ) then
               call dcopy ( m, gBS, 1, y, 1 )
               call s5setp( m, pinorm, y, pi, iw, leniw, rw, lenrw )
               needpi = .false.
            end if
         
*           ============================================================
*           Check for optimality.  
*           Find the reduced costs.
*           ============================================================
            if ( feasbl ) then
               rw(toldj3) = tolLP
            else
               rw(toldj3) = tolFP
            end if

            kPrPrt = kPrc   
            jq     = 0
            djqPrt = djq
            djq    = zero
            gotg   = .false.
            weight = zero
            if (Elastc  .and.  feasbl) then
               weight = wtInf
            end if

            call s5pric( Elastc, feasbl, incres, gotg,
     $                   itn, m, n, nb, nng, nng0, nnH, 
     $                   nS, nonOpt, weight, sgnObj, piNorm,
     $                   jq, djq, kPrc, rw(toldj1),
     $                   ne, nka, a, ha, ka,
     $                   hElast, hs, bl, bu, dummy, pi, rc,
     $                   iw, leniw, rw, lenrw )
            optiml = nonOpt .eq. 0
         end if ! jstFea

         if ( optiml ) then 
*           ------------------------------------------------------------
*           We are apparently optimal.
*           See if any nonbasics have to be set back on their bounds.
*           ------------------------------------------------------------
            call s5dgen( 'Optimal', inform, lPrint, 
     $                   nb, nInf, itn,
     $                   featol, tolx, tolinc, hs, bl, bu, xs,
     $                   itnfix, nfix, tolx0,
     $                   iw, leniw, rw, lenrw )

            optiml = inform .eq. 0
            if ( optiml ) then

*              So far so good.   Check the row residuals.

               if (iw(LUitn) .gt. 0) then
                  call s5setx( 'Check residuals', inform, itn,
     $                         m, n, nb, nBS, rowerr, xNorm,
     $                         ne, nka, a, ha, ka, 
     $                         b, lenb, kBS, xBS, 
     $                         xs, y2, y, iw, leniw, rw, lenrw )
                  optiml = inform .eq. 0
                  if (.not. optiml) then
                     needLU = .true.
                     LUreq  = 10
                  end if
               end if
            end if

*           If xs is not optimal, set  xs  so that ( A  -I )*xs = b
*           and check feasibility.

            if (.not. optiml) then
               needx  = .true.
               feasbl = .false.
               go to 100
            end if
         end if ! optiml

*        ===============================================================
*        Compute all data concerning the current  xs.
*        Print the details of this iteration.
*        Compute reduced costs and check for optimality.
*        ===============================================================
         if ( ObjPhs ) ObjPrt = ObjAdd + ObjLP
         call s5log ( prbtyp, contyp, 
     $                Elastc, gotR, prtFea, jstFea, ObjPhs,
     $                m, mBS, nS,
     $                jSq, jBr, jSr, jBSr,
     $                lines1, lines2,
     $                itn, kPrPrt, lvlInf, lPrint, 
     $                sgnObj, pivot, step, nInf, sInf, wtInf,
     $                ObjPrt, condHz, djqPrt, rgNorm, kBS, xBS,
     $                iw, leniw )

         jBq    = 0
         jBr    = 0
         jBSr   = 0
         jSq    = 0
         jSr    = 0
         kPrPrt = 0

         if ( optiml ) then

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

                     if (Prnt1) then
                        write(iPrint, 8050) itn, contyp
                        write(iPrint, 8060) itn
                     end if

                     if (Srnt1) then
                        write(iSumm , 8050) itn, contyp 
                        write(iSumm , 8060) itn
                     end if
                     Elastc = .true.
                     needpi = .true.
                     optiml = .false.
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

            if ( LP ) then
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
*           Unpack column jq into  y1  and solve  B*y = y1.
*           The altered  y1  satisfies  L*y1 = ajq. 
*           It is used later to modify L and U.

            call s2unpk( jq, m, n, ne, nka, a, ha, ka, y1 )
            call s2Bsol( 'B y = yq', inform, m, y1, y, 
     $                   iw, leniw, rw, lenrw )

*           ============================================================
*           Take a simplex step.  A variable will become nonbasic
*           at the new xs.
*           ============================================================
            if (itn  .ge. itnlim  .or.  itLP .ge. itMax) then 
               ierror = 3
               go to 100
            end if

            itLP   = itLP   + 1
            itn    = itn    + 1
            iw(LUitn)  = iw(LUitn)  + 1
            newLU  = .false.
            jstFea = .false.

            call s5LPit( feasbl, incres, needpi, Elastc,  
     $                   ierror, m+1, m, nb, nDegen, LUreq,
     $                   kp, jBq, jSq, jBr, jSr, jq,
     $                   featol, pivot, step, tolinc,
     $                   hElast, hEstat, hfeas, hs, kBS,
     $                   bl, bu, blBS, buBS,
     $                   xBS, xs, y, y1, 
     $                   iw, leniw, rw, lenrw )
            if (ierror .gt. 0) go to 100

*           Increment featol every iteration.

            featol = featol + tolinc

*           ============================================================
*           Test for error condition and/or frequency interrupts.
*           ============================================================
*           (1) Save a basis map (frequency controlled).
*           (2) Every kdegen iterations, reset featol and move nonbasic 
*               variables onto their bounds if they are very close.
*           (3) Refactorize the basis if it has been modified
*               too many times.
*           (4) Update the LU factors of the basis if requested.
*           (5) Check row error (frequency controlled).

            if (mod(itn,ksav) .eq. 0) then
               call s5hs  ( 'External', nb, bl, bu, hs, xs )
               call s4ksav( minimz, m, n, nb, nS, mBS, ObjLP,
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
               iw(lenL)      = iw(173)
               iw(lenU)      = iw(174)
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
     $                            xs, y2, y, iw, leniw, rw, lenrw )
                     if (inform .gt. 0) then
                        LUreq  = 10
                     end if
                  end if
               end if
            end if

            needLU = LUreq  .gt. 0
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

*     end of s5LP
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5bndE( nb, nBS, plInfy, hEstat, kBS,
     $                   bl, bu, blBS, buBS )

      implicit           double precision (a-h,o-z)
      integer            hEstat(nb), kBS(nBS)
      double precision   bl(nb), bu(nb)
      double precision   blBS(nBS), buBS(nBS)

*     ==================================================================
*     s5bndE  sets the upper and lower bounds on the elastic variables
*     for Elastic Phase 1 and 2.
*
*     s5bndE is called in elastic mode, i.e., when Elastc = true.  In
*     elastic mode, the elastic variables are allowed to violate their
*     true bounds bl and bu.  The bounds blBS and buBS are redefined for
*     the basic and superbasic elastic variables.
*
*     23-Aug-96: First version of s5bndE.
*     10-Jul-97: Current version.
*     ==================================================================

      do 100, k = 1, nBS
         j   = kBS(k)
         jEs = hEstat(j)

         if (jEs .eq. 1) then
            blBS(k) = - plInfy
            buBS(k) =   bl(j)
         else if (jEs .eq. 2) then
            buBS(k) =   plInfy
            blBS(k) =   bu(j)
         end if
  100 continue

*     end of s5bndE
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5BSx ( job, ms, nb, kBS, xBS, xs )

      implicit           double precision (a-h,o-z)
      character          job*(*)
      integer            kBS(ms)
      double precision   xBS(ms), xs(nb)

*     =================================================================
*     s5BSx   copies free variables from  xBS  into  xs  or vice versa,
*             depending on whether  job is 'xBS to x' or 'x to xBS'. 
*     If  ms = m,   only the basic variables are copied.
*     If  ms = nBS  both the basic and superbasic variables are copied.
*
*     07-Nov-91: First version based on Minos routine m5bsx.
*     09-Dec-92: Current version of s5BSx.
*     =================================================================

      if (job(2:2) .eq. 'B') then
         do 10,  k = 1, ms
            j      = kBS(k)
            xs(j)  = xBS(k)
   10    continue
      else
         do 30,  k = 1, ms
            j      = kBS(k)
            xBS(k) = xs(j)
   30    continue
      end if

*     end of s5BSx
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5dgen( job, inform, lPrint, 
     $                   nb, nInf, itn,  
     $                   featol, tolx, tolinc, hs, bl, bu, xs,
     $                   itnfix, nfix, tolx0,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character          job*(*)
      integer            hs(nb)
      double precision   bl(nb), bu(nb), xs(nb)
      integer            nfix(2)

      double precision   rw(lenrw)
      integer            iw(leniw)

*     ==================================================================
*     s5dgen performs most of the manoeuvres associated with degeneracy.
*     The degeneracy-resolving strategy operates in the following way.
*
*     Over a cycle of iterations, the feasibility tolerance featol
*     increases slightly (from tolx0 to tolx1 in steps of tolinc).
*     This ensures that all steps taken will be positive.
*
*     After kdegen consecutive iterations, nonbasic variables within
*     featol of their bounds are set exactly on their bounds and the
*     basic variables are recomputed to satisfy ( A  -I )*xs = b.
*     featol is then reduced to tolx0 for the next cycle of iterations.
*
*
*     If job = 'I'nitialize, s5dgen initializes the parameters:
*
*     featol  is the current feasibility tolerance.
*     tolx0   is the minimum feasibility tolerance.
*     tolx1   is the maximum feasibility tolerance.
*     tolinc  is the increment to featol.
*     kdegen  is the expand frequency (specified by the user).
*             it is the frequency of resetting featol to tolx0.
*     nDegen  counts the number of degenerate steps (not used here, but 
*             incremented by s5step).
*     itnfix  is the last iteration at which an 'Optimal' or 'Cycle'
*             entry set nonbasics onto their bound.
*     nfix(j) counts the number of times an 'Optimal' entry has
*             set nonbasics onto their bound,
*             where j=1 if infeasible, j=2 if feasible.
*
*     tolx0 and tolx1 are both close to the feasibility tolerance tolx
*     specified by the user.  (They must both be less than tolx.)
*
*
*     If job = 'E'nd of cycle,  s5dgen has been called after a cycle of
*     kdegen iterations.  Nonbasic xs(j)s are examined to see if any are 
*     off their bounds by an amount approaching featol.  inform returns 
*     how many.  Deviations as small as tolz (e.g. 1.0d-11) are not
*     counted. If inform is positive, the basic variables are 
*     recomputed.  It is assumed that s5LP or s5QP will then continue 
*     iterations.
*
*     itnfix, nfix, tolx0 could be treated as SAVED variables.
*     They are included as arguments to prevent threading problems in a
*     multiprocessing environment.
*
*     If job = 'O'ptimal,  s5dgen is being called after a subproblem 
*     has been judged optimal, infeasible or unbounded.  
*     Nonbasic xs(j)s are examined as above.
*
*     07-Nov-91: First version based on Minos routine m5dgen.
*     12-Jul-97: Thread-safe version.
*     12-Jul-97: Current version of s5dgen.
*     ==================================================================
      parameter         (zero = 0.0d+0)
*     ------------------------------------------------------------------
      eps1      = rw(  3)
      iPrint    = iw( 12)
      iSumm     = iw( 13)
      kDegen    = iw( 65)

      inform    = 0
      if (job(1:1) .eq. 'I') then

*        job = 'I'nitialize.
*        Initialize at the start of each major iteration.
*        kdegen is the expand frequency      and
*        tolx   is the feasibility tolerance
*        (specified by the user).  They are not changed.
*        nDegen counts the total number of degenerate steps, and is
*        initialized by s8solv.

         itnfix  = 0
         nfix(1) = 0
         nfix(2) = 0
         tolx0   = 0.5d+0 *tolx
         tolx1   = 0.99d+0*tolx
         if (kdegen .lt. 99999999) then
            tolinc = (tolx1 - tolx0) / kdegen
         else
            tolinc = zero 
         end if 
         featol  = tolx0
      else
*        ---------------------------------------------------------------
*        job = 'E'nd of cycle or 'O'ptimal.
*        initialize local variables maxfix and tolz.
*        ---------------------------------------------------------------
         maxfix = 2
         tolz   = eps1
         if (job(1:1) .eq. 'O') then

*           job = 'O'ptimal.
*           Return with inform = 0 if the last call was at the
*           same itn, or if there have already been maxfix calls
*           with the same state of feasibility.

            if (itnfix .eq. itn   ) return
            if (nInf   .gt.   0   ) then
               j = 1
            else
               j = 2
            end if
            if (nfix(j) .ge. maxfix) return
            nfix(j) = nfix(j) + 1
         end if

*        Set nonbasics on their nearest bound if they are within
*        the current featol of that bound.

         itnfix = itn

         do 250, j = 1, nb
            if (hs(j) .le. 1  .or.  hs(j) .eq. 4) then
               b1    = bl(j)
               b2    = bu(j)
               d1    = abs( xs(j) - b1 )
               d2    = abs( xs(j) - b2 )
               if (d1 .gt. d2) then
                  b1   = b2
                  d1   = d2
               end if
               if (d1 .le. featol) then
                  if (d1 .gt. tolz) inform = inform + 1
                  xs(j) = b1
               end if
            end if
  250    continue

*        Reset featol to its minimum value.
             
         featol = tolx0
         if (inform .gt. 0) then

*           The basic variables will be reset.

            if (lPrint .ge. 1) then
               if (iPrint .gt. 0) write(iPrint, 1000) itn, inform
               if (iSumm  .gt. 0) write(iSumm , 1000) itn, inform
            end if
         end if
      end if

      return

 1000 format(' Itn', i7, ': Basics recomputed after ', i7,
     $       '  nonbasics set on bound')

*     end of s5dgen
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5getB( needB, ierror, lenb, m, maxS, mBS, n, nb,
     $                   nnCon, nnJac, nnObj, iObj,
     $                   nS, nScl, nDegen, numLC, numLEQ, numLIQ,
     $                   itMax, itQP, itn, lPrint, 
     $                   ObjAdd, tolFP, tolQP, tolx,
     $                   nInf, sInf, wtInf, piNorm, rgNorm, xNorm,
     $                   ne, nka, a, ha, ka,
     $                   hElast, hEstat, hfeas, hs, kBS, aScale, 
     $                   b, bl, bu,
     $                   blBS, buBS, blslk, buslk,
     $                   gBS, pi, rc, rhs, 
     $                   xs, xBS,
     $                   iy, iy2, y, y2, y1,
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      logical            needB
      integer            ha(ne), hEstat(nb), hfeas(mBS)
      integer            hElast(nb), hs(nb)
      integer            ka(nka), kBS(mBS)
      integer            iy(mBS), iy2(m)
      double precision   a(ne), aScale(nScl)
      double precision   bl(nb), bu(nb), rc(nb), xs(nb)
      double precision   b(*)
      double precision   blBS(mBS), buBS(mBS), gBS(mBS), xBS(mBS)
      double precision   rhs(m), blslk(m), buslk(m), pi(m)
      double precision   y(nb), y2(nb), y1(m)

      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     s5getB   finds an initial basis kBS(1:m) for the linear
*     constraints and bounds.
*     
*     In general, the constraints are scaled.
*     First, we attempt to find a  feasible point for the bounds and
*     general linear equalities. This may fail, so there is the chance
*     that the initial basis is not feasible. 
*     This dificulty is taken care of later.
*
*     31-Jul-96: First version of s5getB.
*     12-Jul-97: Thread-safe version.
*     21-Feb-98: Current version of s5getB.
*     ==================================================================
      logical            Elastc, needLU, needx
      character*20       contyp
      parameter         (zero = 0.0d+0, one = 1.0d+0)
      parameter         (iCrash    =  88)
*     ------------------------------------------------------------------
      iPrint    = iw( 12)
      iSumm     = iw( 13)

      eps0      = rw(  2)
      plInfy    = rw( 70)
      tCrash    = rw( 62)

      minimz    = 1

      if ( needB ) then
*        ---------------------------------------------------------------
*        Crash is needed to find a basis.
*        ---------------------------------------------------------------
*        Treat Crash 1 the same as Crash 2.

         iw(iCrash)  = max( iw(iCrash), 2 )
         iw(iCrash)  = min( iw(iCrash), 3 )

         if (numLC .gt. 0) then
*           ============================================================
*           Find a feasible point for the linear constraints.
*           ============================================================
*           Crash 2 finds a basis for ALL the LINEAR constraints
*                   (equalities and inequalities).
*           Crash 3 treats linear EQUALITIES separately.
*           ------------------------------------------------------------
            if (iw(iCrash) .eq. 2) then

*              Relax.

            else if (iw(iCrash) .eq. 3) then

               if (numLEQ .gt. 0) then

*                 Find a basis for the linear EQUALITIES.
*                 Linear inequality rows are made to appear to be free.

                  if (lPrint .ge. 1) then
                     if (iPrint .gt. 0) write(iPrint, 1100) itn
                     if (iSumm  .gt. 0) write(iSumm , 1100) itn
                  end if
               end if ! numLEQ > 0
            end if ! iCrash = 2 or 3
         end if ! numLC > 0

*        Call Crash even if there are no linear constraints.
*        We haven't done any Solve yet, so we should NOT need
*        call s5hs  ( 'External', nb, bl, bu, hs, xs ) .

         lCrash = iw(iCrash)
         call s2crsh( lCrash, lPrint, m, n, nb,
     $                iw(iCrash), tCrash,
     $                ne, nka, a, ha, ka,
     $                kBS, hs, hEstat,
     $                bl, bu, xs,
     $                iw, leniw, rw, lenrw )
      end if ! needB

*     ==================================================================
*     1. Set nS to match hs(*).
*     2. Set kBS(m+1) thru kBS(m+nS) to define the initial superbasics.
*     3. Check that nonbasic xs are within bounds.
*     ==================================================================
      call s4chek( m, maxS, mBS, n, nb, nS, iObj,
     $             hs, kBS, bl, bu, xs,
     $             iw, leniw, rw, lenrw )

      if (numLC .gt. 0) then
*        ===============================================================
*        Fix any superbasics. This lets us do simplex steps.
*        ===============================================================
         call s5fixS( 'Fix any superbasics', 
     $                m, maxS, mBS, n, nb, nS, hs, kBS, 
     $                bl, bu, blBS, buBS, xs, xBS )

         if (numLEQ .gt. 0) then

*           Relax the linear INEQUALITIES.
*           We have to set "rhs" to make the relaxed rows satisfied
*           at the current xs.  Then xs will not be disturbed more than
*           necessary during a Warm start.

            lenrhs = 0
            contyp = 'linear equality rows'

            if (numLIQ .gt. 0) then
               lenrhs = m
               call s2Aprd( 'No transpose', eps0, ne, nka, a, ha, ka, 
     $                      one, xs, n, zero, y, lenrhs )
               call daxpy ( lenrhs, (-one), xs(n+1), 1, y, 1 )
               if (lenb .gt. 0) 
     $         call daxpy ( lenb, (-one), b, 1, y, 1 )

               call dload ( lenrhs, zero, rhs, 1 )
               do 160, i = nnCon+1, m
                  j      = n + i
                  if (bl(j) .lt. bu(j)) then
                     bl(j)  = - plInfy
                     bu(j)  = + plInfy
                     rhs(i) =   y(i)
                  end if
  160          continue
            end if

            Elastc  = .false.
            lvlInf  = 0
            lEmode  = 0           ! Suspend Elastic mode for equalities 
            needLU  = .true.
            needx   =  needLU
            call s5hs  ( 'Internal', nb, bl, bu, hs, xs )
            call s5LP  ( 'FPEqualities',
     $                   contyp, Elastc, needLU, needx, ierror,
     $                   m, n, nb, nDegen, itMax, itQP, itn,
     $                   lEmode, lvlInf, lPrint,
     $                   minimz, ObjAdd, iObj, tolFP, tolQP, tolx,
     $                   nInf, sInf, wtInf, piNorm, rgNorm, xNorm,
     $                   ne, nka, a, ha, ka,
     $                   hElast, hEstat, hfeas, hs, kBS, aScale, 
     $                   rhs, lenrhs, bl, bu, blBS, buBS,
     $                   gBS, pi, rc, xBS, xs,
     $                   iy, iy2, y, y2, y1,
     $                   cw, lencw, iw, leniw, rw, lenrw )

*           Reset the bounds on the inequality rows.

            if (numLIQ .gt. 0) then
               lc1    = n + nnCon + 1
               call dcopy ( numLC, blslk(nnCon+1), 1, bl(lc1), 1 )
               call dcopy ( numLC, buslk(nnCon+1), 1, bu(lc1), 1 )
            end if ! numLIQ > 0

            if (lPrint .ge. 1) then
               if (nInf .eq. 0) then

*                 The linear E rows are now satisfied.

                  if (iPrint .gt. 0) then
                     write(iPrint, 1000)
                     write(iPrint, 1200) itn
                  end if
                  if (iSumm  .gt. 0) write(iSumm , 1200) itn
               else

*                 The linear E rows are infeasible.

                  if (iPrint .gt. 0) write(iPrint, 1300) itn
                  if (iSumm  .gt. 0) write(iSumm , 1300) itn
               end if
            end if
         end if ! numLEQ > 0

         if (ierror .eq. 0) then
*           ------------------------------------------------------------
*           Go ahead and include the linear INEQUALITIES.
*           ------------------------------------------------------------
            if (needB) then
               if (numLIQ .gt. 0) then 
                  if (lPrint .ge. 1) then
                     if (numLEQ .gt. 0) then
                        if (iPrint .gt. 0) write(iPrint, 1400) itn
                        if (iSumm  .gt. 0) write(iSumm , 1400) itn
                     else
                        if (iPrint .gt. 0) write(iPrint, 1410) itn
                        if (iSumm  .gt. 0) write(iSumm , 1410) itn
                     end if
                  end if

                  call s5hs  ( 'External', nb, bl, bu, hs, xs )
                  call s2amat( 'Define row types',
     $                         m, n, nb,
     $                         nnCon, nnJac, nnObj, iObj,
     $                         ne, nka, a, ha, ka, 
     $                         bl, bu, hEstat,
     $                         iw, leniw, rw, lenrw )
                  lCrash = 4
                  call s2crsh( lCrash, lPrint, m, n, nb,
     $                         iw(iCrash), tCrash,
     $                         ne, nka, a, ha, ka,
     $                         kBS, hs, hEstat,
     $                         bl, bu, xs,
     $                         iw, leniw, rw, lenrw )
               end if ! numLIQ > 0
            end if ! needB

         else if (ierror .eq. 3) then
*           -------------------------------------------
*           Too many iterations. We have to stop.
*           -------------------------------------------
            call s1page( 2, iw, leniw )
            if (iPrint .gt. 0) then
               write(iPrint, 9000)
               write(iPrint, 9010)
            end if
            if (iSumm  .gt. 0) then
               write(iSumm , 9000)
               write(iSumm , 9010)
            end if

         else
            if (iPrint .gt. 0) write(iPrint, 9020) ierror
            if (iSumm  .gt. 0) write(iSumm , 9020) ierror
        end if
      end if ! numLC > 0

      return

 1000 format(' ')
 1100 format(  ' Itn', i7, ': Phase 1A -- making the linear',
     $                     ' equality rows feasible')
 1200 format(  ' Itn', i7, ': Feasible linear equality rows')
 1300 format(  ' Itn', i7, ': Infeasible linear equality rows')
 1400 format(  ' Itn', i7, ': Phase 1B -- making all linear',
     $                     ' rows feasible')
 1410 format(  ' Itn', i7, ': Phase 1  -- making the linear',
     $                     ' rows feasible')

 9000 format(' XXX  Iteration limit exceeded ')
 9010 format(' EXIT -- End of Phase 1. ',
     $       ' Infeasible linear constraints')
 9020 format(' XXX  Unexpected error in s5getb.  ierror =', i4)

*     end of s5getB
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5grdE( nb, nBS, wtInf, hEstat, kBS, gBS )

      implicit           double precision (a-h,o-z)
      integer            hEstat(nb), kBS(nBS)
      double precision   gBS(nBS)

*     ==================================================================
*     s5grdE  is called when elastic variables are allowed to violate
*     their true bounds bl and bu.  It updates the gradient gBS
*     to include the gradient of the penalty term.
*
*     On exit,
*     gBS(nBS)    is the rhs for the equations for pi.
*
*     08-Oct-96: First version of s5grdE written by PEG & MAS.
*     10-Nov-96: Current version.
*     ==================================================================

      do 100, k = 1, nBS
         j   = kBS(k)
         jEs = hEstat(j)
         if      (jEs .eq. 0) then
*           Relax
         else if (jEs .eq. 1) then
            gBS(k) = gBS(k) - wtInf
         else !  (jEs .eq. 2)
            gBS(k) = gBS(k) + wtInf
         end if
  100 continue

*     end of s5grdE
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5hs  ( mode, nb, bl, bu, hs, xs )

      implicit           double precision (a-h,o-z)
      character*8        mode
      double precision   bl(nb), bu(nb)
      integer            hs(nb)
      double precision   xs(nb)

*     ------------------------------------------------------------------
*     s5hs   is called from s8solv and s8setj.
*     if mode = 'Internal', s5hs sets hs(j) = -1 or 4 for certain
*        nonbasic variables.  This allows s5pric to operate more
*        efficiently.  The internal values of hs are now as follows:
*
*        hs(j) = -1  Nonbasic between bounds (bl     <  xs <  bu    )
*        hs(j) =  0  Nonbasic on lower bound (bl-tol <  xs <= bl    )
*        hs(j) =  1  Nonbasic on upper bound (bu     <= xs <  bu+tol)
*        hs(j) =  2  Superbasic
*        hs(j) =  3  Basic
*        hs(j) =  4  Nonbasic and fixed      (bl     = xs  =  bu    )
*
*        where 0 <= tol < the feasibility tolerance.
*
*     if mode = 'External', s5hs changes -1 or 4 values to hs(j) = 0,
*        ready for basis saving and the outside world.
*
*     08 Apr 1992: First version.
*     ------------------------------------------------------------------

      if (mode .eq. 'Internal') then
*        ---------------------------------------------------------------
*        Change nonbasic hs(j) to internal values (including 4 and -1).
*        This may change existing internal values if bl and bu have been
*        changed -- e.g. at the start of each major iteration.
*        ---------------------------------------------------------------
         do 100, j = 1, nb
            if (hs(j) .le. 1) then
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

      else
*        ---------------------------------------------------------------
*        Change hs to external values.
*        Some nonbasic hs(j) may be 4 or -1.  Change them to 0.
*        ---------------------------------------------------------------
         do 200 j = 1, nb
            if (hs(j) .eq. 4  .or.  hs(j) .eq. -1) hs(j) = 0
  200    continue
      end if

*     end of s5hs
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5Inf ( nBS, featol,
     $                   nInf, sInf, hfeas, blBS, buBS, gBS, xBS )

      implicit           double precision (a-h,o-z)
      integer            hfeas(nBS)
      double precision   blBS(nBS), buBS(nBS), gBS(nBS), xBS(nBS)

*     ==================================================================
*     s5Inf  computes the sum and number of infeasibilities.
*
*     hfeas     xs(j)                             Meaning
*     -----     -----                             -------
*      -2   infeasible                              xs(j) .le. bl(j)-tol
*       0     feasible               bl(j)-tol .le. xs(j) .le. bu(j)+tol
*      +2   infeasible                              xs(j) .ge. bu(j)+tol 
*
*     On exit,
*     nInf        is the number violated non-elastic bounds.
*     gBS(nBS)    is the rhs for the equations for pi.
*
*     29-Oct-93: First version of s5Inf.
*     10-Jul-97: Current version.
*     ==================================================================
      parameter         (zero = 0.0d+0, one  = 1.0d+0)
*     ------------------------------------------------------------------

*     ------------------------------------------------------------------
*     Find the number and sum of the non-elastic infeasibilities.
*     ------------------------------------------------------------------
      numinf = 0
      suminf = zero

*     ------------------------------------------------------------------
*     Set hfeas, used in s5step.
*     ---------------------------------------------------------------
      do 100,    k = 1, nBS
         hfeas(k)  = 0
         xk        = xBS (k)
         res       = blBS(k) - xk
         if (res .le. featol) go to 50
         gBS  (k)  = - one
         hfeas(k)  = - 2
         go to 60

   50    res       = xk - buBS(k)
         if (res .le. featol) go to 100
         gBS  (k)  =   one
         hfeas(k)  =   2

   60    numinf    = numinf + 1
         suminf    = suminf + res
  100 continue

      sInf  = suminf
      nInf  = numinf

*     end of s5Inf
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5InfE( nb, nBS, hEstat, kBS,
     $                   nInfE, sInfE, bl, bu, xs )

      implicit           double precision (a-h,o-z)
      integer            hEstat(nb)
      integer            kBS(nBS)
      double precision   bl(nb), bu(nb), xs(nb)

*     ==================================================================
*     s5InfE  computes the sum and number of elastic infeasibilities.
*
*     On exit,
*     nInfE is the number of elastics allowed to go infeasible.
*
*     20-Aug-96: First version of s5InfE.
*     29-Nov-96: Current version.
*     ==================================================================
      parameter         (zero = 0.0d+0)
*     ------------------------------------------------------------------
*     Find the number and sum of the elastic infeasibilities.
*     ------------------------------------------------------------------
      numInf = 0
      sumInf = zero

      do 100, k = 1, nBS
         j   = kBS(k)
         jEs = hEstat(j)

         if      (jEs .eq. 0) then

*           Relax

         else if (jEs .eq. 1) then
            numinf = numinf + 1
            res    = bl(j) - xs(j)

            if (res .gt. zero) then
               suminf = suminf + res
            end if
         else if (jEs .eq. 2) then
            numinf = numinf + 1
            res    = xs(j) - bu(j)

            if (res .gt. zero) then
               suminf = suminf + res
            end if
         end if
  100 continue

      sInfE = suminf
      nInfE = numInf

*     end of s5InfE
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5LPit( feasbl, incres, needpi, Elastc,
     $                   ierror, m1, m, nb, nDegen, LUreq,
     $                   kp, jBq, jSq, jBr, jSr, jq,
     $                   featol, pivot, step, tolinc,
     $                   hElast, hEstat, hfeas, hs, kBS, 
     $                   bl, bu, blBS, buBS,
     $                   xBS, xs, y, y1,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      logical            feasbl, incres, needpi, Elastc
      integer            hfeas(m1), kBS(m1)
      integer            hElast(nb), hEstat(nb), hs(nb)
      integer            iw(leniw)
      double precision   rw(lenrw)
      double precision   bl(nb), bu(nb), xs(nb)
      double precision   blBS(m1), buBS(m1), xBS(m1)
      double precision   y(m1), y1(m)

*     ==================================================================
*     s5LPit computes one step of the primal simplex method.
*     jq is the variable entering the basis and djq is its reduced cost.
*
*     10-Sep-91: First version of s5LPit based on Minos routine m5lpit.
*     02-Aug-96: First min sum version added by PEG.
*     12-Jul-97: Thread-safe version.
*     15-Jul-97: Current version of s5LPit.
*     =================================================================
      logical            hitlow, move, onbnd, unbndd
      parameter         (zero      = 0.0d+0, one      = 1.0d+0)
      parameter         (LUmod     = 216)
*     ------------------------------------------------------------------
      tolpiv    = rw( 60)
      plInfy    = rw( 70)
      bigdx     = rw( 72)
      iPrint    = iw( 12)

*     s5step assumes that the first m1 components of xBS can move.

      jSq       =     jq             ! Entering superbasic
      jSr       =     jq             ! leaving  superbasic
      hfeas(m1) =      0
      kBS  (m1) =     jq
      xBS  (m1) =  xs(jq)
      blBS (m1) =  bl(jq)
      buBS (m1) =  bu(jq)

*     ==================================================================
*     Set hEstat(jq) and the elastic parts of blBS and buBS.
*     ==================================================================
      if ( Elastc ) then

*        If the new superbasic is an elastic variable
*        and it wants to move infeasible, set its elastic state.

         if (hElast(jq) .gt. 0) then 
            js  = hs(jq)
            if ( incres ) then
               if (js .eq. 1  .or.  js .eq. 4) then
                  hEstat(jq) =   2
                  buBS(m1)   =   plInfy
                  if ( feasbl ) then
                     blBS(m1) = bu(jq)
                  end if
               end if
            else
               if (js .eq. 0  .or.  js .eq. 4) then
                  hEstat(jq) =   1 
                  blBS(m1)   = - plInfy
                  if ( feasbl ) then
                     buBS(m1) = bl(jq)
                  end if
               end if
            end if
         end if
      end if ! Elastic mode

*     ==================================================================
*     Select a variable to be dropped from B.
*     s5step  uses the (m+1)th element of  blBS, buBS, xBS and y.
*     ==================================================================
      if (incres) then
         call dscal ( m, (- one), y, 1 )
         y(m1) =   one
      else
         y(m1) = - one
      end if

      stepmx = bigdx
      call s5step( m1, nDegen, 
     $             featol, plInfy, stepmx, tolinc, tolpiv,
     $             hfeas, blBS, buBS, xBS, y,
     $             hitlow, move, onbnd, unbndd,
     $             kp, bound, exact, step, stepP )

      if (.not. unbndd) then
*        ---------------------------------------------------------------
*        Update the basic variables xBS and copy them into xs.
*        ---------------------------------------------------------------
         jr      = kBS(kp)

         call daxpy ( m1, step, y, 1, xBS, 1 )
         call s5BSx ( 'xBS to x', m1, nb, kBS, xBS, xs )
         if (onbnd) xs(jr) = bound

         if (kp .eq. m1) then
*           ------------------------------------------------------------
*           Variable jq reaches its opposite bound.
*           ------------------------------------------------------------
            if (incres) then
               hs(jq)  = 1
            else
               hs(jq)  = 0
            end if
            hfeas(kp)  = 0               
            pivot      = zero
            if (.not. feasbl  .or.  Elastc) needpi = .true.

         else
*           ------------------------------------------------------------
*           Variable jq replaces the kp-th variable of  B.
*           It could be a fixed variable, whose new state must be 4.
*           ------------------------------------------------------------
            needpi =   .true.
            jBq    =   jq
            jBr    =   jr
            hs(jq) =   3
            pivot  = - y(kp)

            jEs    = hEstat(jr)
            hEstat(jr) = 0

            if (jEs .eq. 0) then 
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

            hs(jr)   = jrstat
            kBS(kp)  = jq
            xBS(kp)  =  xBS(m1)
            blBS(kp) = blBS(m1)
            buBS(kp) = buBS(m1)

*           Update the LU factors.

            iw(LUmod)  = iw(LUmod) + 1
            call s2Bmod( inform, kp, m, y1,
     $                   iw, leniw, rw, lenrw )
            if (inform .ne. 0) then
               LUreq  = 7       ! No free memory.
            end if
         end if ! kp ne m1

      else

*        The solution is apparently unbounded.

         if (incres) then
            write(iPrint, 1000) jq
         else
            write(iPrint, 1100) jq
         end if
         ierror = 2
      end if

      return

 1000 format(' Variable', i6, '  can increase indefinitely')
 1100 format(' Variable', i6, '  can decrease indefinitely')

*     end of s5LPit
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5pric( Elastc, feasbl, incres, gotg,
     $                   itn, m, n, nb, nng, nng0, nnH, 
     $                   nS, nonOpt, weight, sgnObj, piNorm,
     $                   jq, djq, kPrc, toldj,
     $                   ne, nka, a, ha, ka,
     $                   hElast, hs, bl, bu, g, pi, rc,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      logical            Elastc, feasbl, incres, gotg
      integer            ha(ne), hElast(nb), hs(nb)
      integer            ka(nka)
      integer            iw(leniw)
      double precision   rw(lenrw)
      double precision   toldj(3)
      double precision   a(ne), bl(nb), bu(nb)
      double precision   g(nng0), pi(m), rc(nb)

*     ------------------------------------------------------------------
*     s5pric  selects a nonbasic variable to enter the basis,
*     using the reduced gradients  dj = g(j) - pi'*a(j).
*
*     This version does partial pricing on both structurals and slacks.
*     Dynamic tolerances are used if partial price is in effect.
*
*     Partial pricing here means sectional pricing, because the three 
*     blocks of  (A  -I)  are sliced up into nParPr sections
*     of equal size.  (The last section of each may be a little bigger,
*     since nParPr is unlikely to divide evenly into  n  or  m.)
*
*     input    g      = gradient for nonlinear variables
*                       (for the subproblem objective function).
*              pi     = pricing vector.
*              kPrc   = the no. of the section where  s5pric  last found
*                       a useful dj.
*                       (kPrc = 0 at the start of each major iteration.)
*              toldj(1-2) hold the current told if partial pricing, for
*                       phase 1 and 2 respectively.  told is used to
*                       determine if a dj is significant.
*              toldj(3) holds the specified optimality tolerance.
*              biggst   keeps track of the biggest dj found during the
*                       last scan of all sections of ( A  -I ).
*
*     output   kPrc   = the last section scanned.
*              nonOpt = the no. of useful djs found in that section.
*              jq     = best column found.
*              djq    = best dj.
*              toldj(1-2) save the current told if partial pricing.
*              incres   says if variable jq should increase or decrease.
*
*     In the code below,
*     the next section of  A  contains nPr1 structurals (j1+1 thru k1),
*     the next section of -I  contains nPr2 slacks      (j2+1 thru k2).
*     If  nParPr  is rather large, either nPr1 or nPr2 could be zero,
*     but not both.
*
*     ------------------------------------------------------------------
*     09-Aug-92: First version of s5pric based on Minos 5.4 m5pric.
*     29-Jul-96: Multiple pricing removed.
*     05-Aug-96: First version with Elastic mode.
*     12-Jul-97: Thread-safe version.
*     07-Feb-98: Current version of s5pric.
*     ==================================================================
*-->  parameter         (zero = 0.0d+0, reduce = 0.25d+0)
      parameter         (zero = 0.0d+0, reduce = 0.2d+0 )
*     ------------------------------------------------------------------
      plInfy    = rw( 70)
      iPrint    = iw( 12)
      iSumm     = iw( 13)
      lprDbg    = iw( 80)
      nParPr    = iw( 94)

      djmax     = - plInfy
      djq       =   zero

      jq        = 0
      jfree     = 0
      nonOpt    = 0
      nPrc      = 0
      nParP     = nParPr
      nPr1      = n  / nParP
      nPr2      = m  / nParP
      if (max( nPr1, nPr2 ) .le. 0) nParP = 1

*     Set the tolerance for a significant dj.

      tolmin    = toldj(3) * piNorm
      if ( feasbl ) then
         lvldj  = 2
      else
         lvldj  = 1
      end if
      told      = toldj(lvldj)
      if (nParP .eq. 1) told = tolmin

*     Set pointers to the next section of  A  and  -I.
*     nPrc counts how many sections have been scanned in this call.
*     kPrc keeps track of which one to start with.

  100 nPrc = nPrc + 1
      kPrc = kPrc + 1
      if (kPrc .gt. nParP) kPrc = 1

      nPr1 = n  / nParP
      j1   =      (kPrc - 1)*nPr1
      k1   = j1 + nPr1
      if (kPrc .eq. nParP) k1 = n
      nPr1 = max( 0, k1-j1 )

      nPr2 = m  / nParP
      j2   = n  + (kPrc - 1)*nPr2
      k2   = j2 + nPr2
      if (kPrc .eq. nParP) k2 = nb
      nPr2 = max( 0, k2-j2 )

*     ------------------------------------------------------------------
*     Main loops for partial pricing (or full pricing).
*     Compute reduced costs rc(*)
*     for the kPrc-th section of structurals
*     and the kPrc-th section of slacks.
*     ------------------------------------------------------------------
      call s5rc  ( j1+1, k1, gotg,
     $             m, n, nng, nng0, sgnObj, 
     $             ne, nka, a, ha, ka,
     $             hs, g, pi, rc )

      do 200 j = j2+1, k2
         rc(j) = pi(j-n)
  200 continue

*     ------------------------------------------------------------------
*     Main loop for pricing structural and slack reduced costs.
*     dj is rc(j), the reduced cost.
*     d  is -dj or +dj, depending on which way xs(j) can move.
*     We are looking for the largest d (which will be positive).
*     ------------------------------------------------------------------
      np   = nPr1 + nPr2
      j    = j1
      jSlk = nPr1 + 1

      do 400, jj = 1, np
         if (jj .eq. jSlk) j = j2
         j       = j + 1
         js      = hs(j)

         if (js .le. 1) then
            dj     = rc(j)

            if      (js .eq. 0) then
*              xj  is allowed to increase.
               d      = - dj
            else if (js .eq. 1) then
*              xj  is allowed to decrease.
               d      =   dj
            else
*              xj  is free to move either way.
*              Remember him as jfree in case he is the only one.
               d      = abs( dj )
               jfree  = j
            end if

*           See if this dj is significant.
*           Also see if it is the biggest dj so far.

            if (d  .gt. told) nonOpt = nonOpt + 1
            if (djmax .lt. d) then
               djmax  = d
               djq    = dj
               jq     = j
               kPsav  = kPrc
            end if
         end if
  400 continue

      if ( Elastc ) then
*        ---------------------------------------------------------------
*        Scan this section again, looking for nonbasic elastics.
*        ---------------------------------------------------------------

*        Compute reduced costs rc(j) for fixed nonbasic columns.

         call s5rcE ( j1+1, k1, gotg,
     $                m, n, nng, nng0, sgnObj,
     $                ne, nka, a, ha, ka,
     $                hElast, hs, g, pi, rc )

         j    = j1
         do 500, jj = 1, np
            if (jj .eq. jSlk) j = j2
            j      = j + 1
            je     = hElast(j)

            if (je .gt. 0) then
               js  = hs(j)
               dj  = rc(j)

               if      (js .eq. 0) then
*                 ------------------------------------------------------
*                 Nonbasic at its lower bound. 
*                 An elastic xj can decrease through the bound.
*                 ------------------------------------------------------
                  if (je .eq. 1  .or.  je .eq. 3) then
                     dj  =   dj - weight
                     d   =   dj
                  end if

               else if (js .eq. 1) then
*                 ------------------------------------------------------
*                 Nonbasic at its upper bound. 
*                 The default is to allow xj to decrease.
*                 However, an elastic xj can increase through the bound.
*                 ------------------------------------------------------
                  if (je .eq. 2  .or.  je .eq. 3) then
                     dj  =   dj + weight
                     d   = - dj 
                  end if

               else if (js .eq. 4) then
*                 ------------------------------------------------------
*                 Fixed elastic variable. 
*                 xj is free to move either way.
*                 ------------------------------------------------------
                  if (je .eq. 2) then 
                     dj1 =   zero
                     d1  =   zero
                  else
                     dj1 =   dj - weight
                     d1  =   dj1 
                  end if

                  if (je .eq. 1) then 
                     dj2 =   zero
                     d2  =   zero
                  else
                     dj2 =   dj + weight
                     d2  = - dj2 
                  end if

                  if (d1 .ge. d2) then
*                    xj  is allowed to decrease.
                     dj =   dj1
                     d  =   d1
                  else
*                    xj  is allowed to increase.
                     dj =   dj2
                     d  =   d2
                  end if
               else
                  d  = zero
                  dj = zero
               end if

*              See if this dj is significant.
*              Also see if it is the biggest dj so far.

               if (d  .gt. told) nonOpt = nonOpt + 1
               if (djmax .lt. d) then
                  djmax  = d
                  djq    = dj
                  jq     = j
                  kPsav  = kPrc
               end if
            end if
  500    continue
      end if

*     ------------------------------------------------------------------
*     End of loop looking for biggest dj in the kPrc-th section.
*     ------------------------------------------------------------------
      if (nonOpt .eq. 0) then
         if (nParP .gt. 1) then
*           ============================================================
*           No significant dj has been found.  (All are less than told.)
*           Price the next section, if any remain.
*           ============================================================
            if (nPrc .lt. nParP) go to 100

*           ============================================================
*           All sections have been scanned.  Reduce told
*           and grab the best dj if it is bigger than tolmin.
*           ============================================================
            if (djmax .gt. tolmin) then
               nonOpt = 1
               kPrc   = kPsav
               told   = max( reduce * djmax, tolmin  )
               toldj(lvldj) = told
               if (lprDbg .ge. 1) then
                  if (iPrint .gt. 0)
     $            write(iPrint, 1000) itn, told, piNorm, weight
                  if (iSumm  .gt. 0)
     $            write(iSumm , 1000) itn, told, piNorm, weight
               end if
            end if
         end if
      end if

*     -----------------------------------------------------------------
*     Finish if we found someone nonoptimal (nonOpt .gt. 0)
*     or if there's a nonbasic floating free
*     between its bounds                    (jfree  .eq. 0)
*     or if the problem is nonlinear        (nnH    .gt. 0)
*     or there are some superbasics         (nS     .gt. 0).
*     -----------------------------------------------------------------
      incres = djq .lt. zero
      if (nonOpt .gt. 0) go to 900
      if (jfree  .eq. 0) go to 900
      if (nS     .gt. 0) go to 900
      if ( feasbl ) then
         if (nnH .gt. 0) go to 900
      end if

*     -----------------------------------------------------------------
*     jfree > 0 and nS = 0.
*     We prefer not to end an LP problem (or an infeasible problem)
*     with some nonbasic variables floating free between their bounds
*     (hs(j) = -1).  Their dj's will be essentially zero
*     but we might as well jam them into the basis.
*
*     We leave true free variables alone -- they will probably be zero.
*     (To be rigorous, we should test if the ARE zero and move them
*     towards zero if not.  This has yet to be done...)
*     -----------------------------------------------------------------
      do 600, j = 1, nb
         if (hs(j) .eq. -1) then
            b1     = bl(j)
            b2     = bu(j)
            if (b1 .gt. -plInfy  .or.  b2 .lt. plInfy) then
      
*              We just found a floating variable with finite bounds.
*              Ask for a move towards the bound nearest zero.
      
               incres = abs(b1) .ge. abs(b2)
               nonOpt = 1
               jq     = j
               djq    = rc(j)
               go to 900
            end if
         end if
  600 continue

*     Exit.

  900 return

 1000 format(' Itn', i7, ': toldj =', 1p, e8.1,
     $       '    Norm pi =', e8.1, '    weight = ', e8.1)

*     end of s5pric
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5rc  ( j1, j2, gotg,
     $                   m, n, nng, nng0, sgnObj,
     $                   ne, nka, a, ha, ka,
     $                   hs, g, pi, rc )

      implicit           double precision (a-h,o-z)
      logical            gotg
      integer            ha(ne), hs(n)
      integer            ka(nka)
      double precision   a(ne)
      double precision   g(nng0), pi(m), rc(n)

*     ==================================================================
*     s5rc   computes reduced costs rc(j) for nonbasic columns of A
*     in the range j = j1 to j2.  It is called by s5pric.
*
*     The loop for computing dj for each column could conceivably be
*     optimized on some machines.  However, there are seldom more than
*     5 or 10 entries in a column.
*
*     Note that we could skip fixed variables by passing in the bounds
*     and testing if bl(j) .eq. bu(j), but these are relatively rare.
*     But see comment for 08 Apr 1992 in m5pric.
*
*     31 Jan 1992: First version
*     08 Apr 1992: Internal values of hs(j) are now used, so fixed
*                  variables (hs(j) = 4) are skipped as we would like.
*     11-Jan-96:   Current version.
*     ==================================================================
      parameter         (zero = 0.0d+0)
*     ------------------------------------------------------------------

      do 300 j = j1, j2
         if (hs(j) .le. 1) then
            dj    = zero

            do 200 i = ka(j), ka(j+1)-1
               ir    = ha(i)
               dj    = dj  +  pi(ir) * a(i)
  200       continue

            rc(j) = - dj
         end if
  300 continue

      if ( gotg ) then

*        Include the nonlinear objective gradient if relevant.

         if (j1 .le. nng) then
            do 350 j = j1, min( j2, nng )
               if (hs(j) .le. 1) rc(j) = rc(j) + sgnObj*g(j)
  350       continue
         end if
      end if

*     end of s5rc
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5rcE ( j1, j2, gotg,
     $                   m, n, nng, nng0, sgnObj,
     $                   ne, nka, a, ha, ka,
     $                   hElast, hs, g, pi, rc )

      implicit           double precision (a-h,o-z)
      logical            gotg
      integer            ha(ne), hElast(n), hs(n)
      integer            ka(nka)
      double precision   a(ne)
      double precision   g(nng0), pi(m), rc(n)

*     ==================================================================
*     s5rcE  computes reduced costs rc(j) in the range j = j1 to j2
*     for fixed nonbasic columns that have one or more elastic bounds.
*     It is called by s5pric.
*
*     07-Feb-98: First version based on s5rc. 
*     07-Feb-98: Current version.
*     ==================================================================
      parameter         (zero = 0.0d+0)
*     ------------------------------------------------------------------

      do 300, j = j1, j2
         if (hs(j) .eq. 4  .and.  hElast(j) .gt. 0) then
            dj    = zero

            do 200 i = ka(j), ka(j+1)-1
               ir    = ha(i)
               dj    = dj  +  pi(ir) * a(i)
  200       continue

            rc(j) = - dj
         end if
  300 continue

      if ( gotg ) then

*        Include the nonlinear objective gradient if relevant.

         if (j1 .le. nng) then
            do 350, j = j1, min( j2, nng )
               if (hs(j) .eq. 4  .and.  hElast(j) .gt. 0) then
                  rc(j) = rc(j) + sgnObj*g(j)
               end if
  350       continue
         end if
      end if

*     end of s5rcE
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5setE( nb, nBS, featol, plInfy,
     $                   hElast, hEstat, kBS, 
     $                   bl, bu, blBS, buBS, xBS )

      implicit           double precision (a-h,o-z)
      integer            hElast(nb), hEstat(nb), kBS(nBS)
      double precision   bl(nb), bu(nb)
      double precision   blBS(nBS), buBS(nBS), xBS(nBS)

*     ==================================================================
*     s5setE  initializes hEstat and adjusts the temporary upper and
*     lower bounds for the elastic variables.
*
*     08-Aug-96: First version of s5setE written by PEG.
*     14-Nov-96: Current version.
*     ==================================================================
      do 100,   k = 1, nBS
         j        = kBS(k)
         je       = hElast(j)

         if (je .eq. 1  .or.  je .eq. 3) then
            res = bl(j) - xBS(k)
            if (res .gt. featol) then
               hEstat(j) =   1
               blBS(k)   = - plInfy
            end if
         end if

         if (je .eq. 2  .or.  je .eq. 3) then
            res = xBS(k) - bu(j)
            if (res .gt. featol) then
               hEstat(j) =   2
               buBS(k)   =   plInfy
            end if
         end if
  100 continue

*     end of s5setE
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5setp( m, piNorm, rhs, pi, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      integer            iw(leniw)

      double precision   pi(m), rhs(m)
      double precision   rw(lenrw)

*     ==================================================================
*     s5setp solves  B' pi = rhs.  Beware -- rhs is altered by s2Bsol.
*     ==================================================================
      parameter        (one = 1.0d+0)
*     ------------------------------------------------------------------

      call s2Bsol( 'Bt', inform, m, rhs, pi, iw, leniw, rw, lenrw  )

      piNorm = max( dnrm1s( m, pi, 1 ), one )

*     end of s5setp
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5setx( job, inform, itn,
     $                   m, n, nb, nBS, rowerr, xNorm,
     $                   ne, nka, a, ha, ka,
     $                   b, lenb, kBS, xBS, 
     $                   xs, y2, y,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character          job*(*)
      integer            ha(ne)
      integer            ka(nka), kBS(nBS)
      integer            iw(leniw)
      double precision   rw(lenrw)
      double precision   a(ne), b(*)
      double precision   xBS(nBS), y(m), y2(m)
      double precision   xs(nb)

*     ==================================================================
*     s5setx performs the following functions:
*      Job            Function        
*      ====           ========
*     'R'eset xBS,    the basic components of xs are computed to satisfy
*                     Ax - s = b; that is  (A -I)*xs = b. Then a row
*                     check is performed to see how well  (A -I)*xs = b 
*                     is satisfied.  y is set to be the row residuals, 
*                     y = b - Ax + s,  and the row error is norm(y).
*
*     'C'heck resid   just get the row error.
*
*     The row error is a measure of how well xs satisfies (A -I)*xs = b.
*     The right-hand side  b  is zero if lenb = 0.
*
*     18-Nov-91: First version of s5setx based on Minos routine m5setx.
*     12-Jul-97: Thread-safe version.
*     12-Jan-98: Current version of s5setx.
*     ==================================================================
      parameter         (zero = 0.0d+0, one = 1.0d+0)
*     ------------------------------------------------------------------
      eps0      = rw(  2)
      tolrow    = rw( 61)
      iPrint    = iw( 12)
      iSumm     = iw( 13)
      lprDbg    = iw( 80)

      inform    = 0

      call s5BSx ( 'x  to xBS', nBS, nb, kBS, xBS, xs )

*     ------------------------------------------------------------------
*     Compute row residuals  y  =  b - (A -I)*xs.
*     The slack columns are done separately.
*     ------------------------------------------------------------------
      if (lenb .gt. 0) call dcopy ( lenb, b, 1, y, 1 )
      if (lenb .lt. m) call dload ( m-lenb, zero, y(lenb+1), 1 )

      call s2Aprd( 'Normal', eps0, ne, nka, a, ha, ka, 
     $             (-one), xs, n, one, y, m )

*     ------------------------------------------------------------------
*     Do a row check, perhaps after recomputing the basic xs.
*     -----------------------------------------------------------------
*     Get the full row residuals  y  =  b - (A -I)*xs.

      call daxpy ( m, one, xs(n+1), 1, y, 1 )
      rowerr = zero

      if (job(1:1) .eq. 'R') then
*        ================================================
*        Extract xBS, the basics and superbasics, from xs.
*        See if iterative refinement is worth doing.
*        ================================================
         ir     = idamax( m, y, 1 )
         rmax   = abs( y(ir) )
         rowerr = rmax / (one + rowerr)

         if (rowerr .gt. eps0) then
*           ------------------------------------------------------------
*           Compute a correction to basic xs from  B*y2 = y.
*           Extract the basic and superbasic variables from xs.
*           Set basic xs = xs + y2.
*           Store the new basic variables in xs.
*           ------------------------------------------------------------
            call s2Bsol( 'B y2 = y ', inform, m, y, y2, 
     $                   iw, leniw, rw, lenrw  )
            call daxpy ( m, one, y2, 1, xBS, 1 )
            call s5BSx ( 'xB to x ', m, nb, kBS, xBS, xs )

*           Compute  y  =  b  -  (A -I)*xs  again for the new xs.
         
            if (lenb .gt. 0) call dcopy ( lenb, b, 1, y, 1 )
            if (lenb .lt. m) call dload ( m-lenb, zero, y(lenb+1), 1 )

            call s2Aprd( 'Normal', eps0, ne, nka, a, ha, ka, 
     $                   (-one), xs, n, one, y, m )
            call daxpy ( m, one, xs(n+1), 1, y, 1 )

         end if ! rowerr .gt. eps0
      end if ! job .eq. 'Reset'

*     Find the norm of xBS, the basic and superbasic xs.
*     Find the maximum row residual.

      xNorm  = dnrm1s( nBS, xBS, 1 )
      irmax  = idamax( m, y, 1 )
      rmax   = abs( y(irmax) )
      rowerr = rmax / (one + rowerr)
      if (rowerr .gt. tolRow) inform = 10
      if (rowerr .gt. tolRow  .or.  lprDbg .ge. 2) then
         if (iPrint .gt. 0) then
            write(iPrint, 1000) itn, rmax, irmax, xNorm
         end if
         if (iSumm  .gt. 0) then
            write(iSumm , 1001) itn, rmax, irmax
         end if
      end if

      return

 1000 format(/ ' Row check at itn =', i7,
     $         '.   Max residual =', 1p, e9.2, '  on row', i5,
     $         '.   Norm x =', e9.2)
 1001 format(/ ' Row check at itn =', i7,
     $         '.   Max residual =', 1p, e9.2, '  on row', i5 )

*     end of s5setx
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5step( nBS, nDegen, 
     $                   featol, plInfy, stepmx, tolinc, tolpiv,
     $                   hfeas, blBS, buBS, xBS, y,
     $                   hitlow, move, onbnd, unbndd,
     $                   kp, bound, exact, alpha, alphap )

      implicit           double precision (a-h,o-z)
      integer            hfeas(nBS)
      double precision   blBS(nBS), buBS(nBS), xBS(nBS), y(nBS)
      logical            hitlow, move, onbnd, unbndd

*     ==================================================================
*     s5step  finds a step  alpha  such that the point  xs + alpha*y
*     reaches one of the bounds on  xs.
*
*     In this version of s5step, when xs is infeasible, the number of
*     infeasibilities will never increase.  If the number stays the
*     same, the sum of infeasibilities will decrease.
*     If the number decreases by one or more,
*     the sum of infeasibilities will usually decrease also, but
*     occasionally it will increase after the step  alpha  is taken.
*     (Convergence is still assured because the number has decreased.)
*
*     Two possible steps are computed as follows:
*
*     alphaf = the maximum step that can be taken without violating
*              one of the bounds that are currently satisfied.
*
*     alphai = the maximum (nonzero) step that has the property of
*              reaching a bound that is currently violated,
*              subject to the pivot being reasonably close to the
*              maximum pivot among infeasible variables.
*              (alphai is not defined if xs is feasible.)
*
*     alphai is needed occasionally when infeasible, to prevent
*     going unnecessarily far when alphaf is quite large.  It will
*     always come into effect when xs is about to become feasible.
*     The sum of infeasibilities will decrease initially as alpha
*     increases from zero, but may start increasing for larger steps.
*     choosing a large alphai allows several elements of  xs  to
*     become feasible at the same time.
*
*     In the end, we take  alpha = alphaf  if xs is feasible, or if
*     alphai > alphap (where alphap is the perturbed step from pass 1).
*     Otherwise,  we take  alpha = alphai.
*
*     Input parameters
*     ----------------
*     nBS    is  m + 1  for s5lpit,  m + nBS  for s5QPit.
*     stepmx defines what should be treated as an unbounded step.
*     plInfy provides insurance for detecting unboundedness.
*            if alpha reaches a bound as large as plInfy, it is
*            classed as an unbounded step.
*     tolpiv is a tolerance to exclude negligible elements of y.
*     featol is the current feasibility tolerance used by s5QP.
*            Typically in the range 0.5*tolx to 0.99*tolx,
*            where tolx is the featol specified by the user.
*     tolinc is used to determine stepmn (see below), the minimum
*            positive step.
*     hfeas  is set by  s5Inf  as follows:
*            hfeas(j) = -2  if xs(j) .lt. bl(j) - featol
*                     =  0  if xs(j)  is feasible
*                     = +2  if xs(j) .gt. bu(j) + featol
*     blBS   the lower bounds on the basic and superbasic variables.
*     BLBU   the upper bounds on ditto.
*     xBS    the values of       ditto.
*     y      the search direction.
*
*
*     Output parameters
*     -----------------
*     hitlow  = true  if a lower bound restricted alpha.
*             = false otherwise.
*     move    = true  if exact ge stepmn (defined at end of code).
*     onbnd   = true  if alpha =  exact.
*                     this means that the step alpha moves xBS(kp)
*                     exactly onto one of its bounds, namely bound.
*             = false if the exact step would be too small
*                     ( exact lt stepmn ).
*               (with these definitions,  move = onbnd).
*     unbndd  = true  if alpha = stepmx.  kp may possibly be zero.
*               the parameters hitlow, move, onbnd, bound and exact
*               should not be used.
*     kp      = the index (if any) such that xBS(kp) reaches a bound.
*     bound   = the bound value blBS(kp) or buBS(kp) corresponding
*               to hitlow.
*     exact   = the step that would take xBS(kp) exactly onto bound.
*     alpha   = an allowable, positive step.
*               if unbndd is true,  alpha = stepmx.
*               otherwise,          alpha = max( stepmn, exact ).
*     alphap  = the perturbed step from pass 1.
*
*     07-Nov-91: First version of s5step based on Minos routine m5chzr.
*     23-Dec-92: Current version.
*     ==================================================================
      logical            blockf, blocki
      parameter         (zero = 0.0d+0, gamma = 0.001d+0)

*     ------------------------------------------------------------------
*     First pass.
*     For feasible variables, find the step alphap that reaches the
*     nearest perturbed (expanded) bound.  alphap will be slight larger
*     than the step to the nearest true bound.
*     For infeasible variables, find the maximum pivot pivmxi.
*     ------------------------------------------------------------------
      delta  = featol
      alphap = stepmx
      pivmxi = zero

      do 200, j  = 1, nBS
         pivot   = y(j)
         pivabs  = abs( pivot )
         if (pivabs .le. tolpiv) go to 200
         jtype  = hfeas(j)
         if (pivot  .gt. zero  ) go to 150

*        x  is decreasing.
*        Test for smaller alphap if lower bound is satisfied.

         if (jtype .lt. 0) go to 200
         res    = xBS(j) - blBS(j) + delta
         if (alphap*pivabs .gt. res) alphap = res / pivabs

*        Test for bigger pivot if upper bound is violated.

         if (jtype .gt. 0) pivmxi = max( pivmxi, pivabs )
         go to 200

*        x  is increasing.
*        Test for smaller alphap if upper bound is satisfied.

  150    if (jtype .gt. 0) go to 200
         res    = buBS(j) - xBS(j) + delta
         if (alphap*pivabs .gt. res) alphap = res / pivabs

*        Test for bigger pivot if lower bound is violated.

         if (jtype .lt. 0) pivmxi = max( pivmxi, pivabs )
  200 continue

*     ------------------------------------------------------------------
*     Second pass.
*     For feasible variables, recompute steps without perturbation.
*     Choose the largest pivot element subject to the step being
*     no greater than alphap.
*     For infeasible variables, find the largest step subject to the
*     pivot element being no smaller than gamma * pivmxi.
*     ------------------------------------------------------------------

      alphai = zero
      pivmxf = zero
      pivmxi = gamma * pivmxi
      jhitf  = 0
      jhiti  = 0

      do 400, j = 1, nBS
         pivot  = y(j)
         pivabs = abs( pivot )
         if (pivabs .le. tolpiv) go to 400
         jtype  = hfeas(j)
         if (pivot  .gt. zero  ) go to 350

*        x  is decreasing.
*        Test for bigger pivot if lower bound is satisfied.

         if (jtype    .lt.     0   ) go to 400
         if (pivabs   .le.   pivmxf) go to 340
         res    = xBS(j) - blBS(j)
         if (alphap*pivabs .lt. res) go to 340
         pivmxf = pivabs
         jhitf  = j

*        Test for bigger alphai if upper bound is violated.

  340    if (jtype    .eq.     0   ) go to 400
         if (pivabs   .lt.   pivmxi) go to 400
         res    = xBS(j) - buBS(j)
         if (alphai*pivabs .ge. res) go to 400
         alphai = res / pivabs
         jhiti  = j
         go to 400

*        x  is increasing.
*        Test for bigger pivot if upper bound is satisfied.

  350    if (jtype    .gt.     0   ) go to 400
         if (pivabs   .le.   pivmxf) go to 360
         res    = buBS(j) - xBS(j)
         if (alphap*pivabs .lt. res) go to 360
         pivmxf = pivabs
         jhitf  = j

*        Test for bigger alphai if lower bound is violated.

  360    if (jtype    .eq.     0   ) go to 400
         if (pivabs   .lt.   pivmxi) go to 400
         res    = blBS(j) - xBS(j)
         if (alphai*pivabs .ge. res) go to 400
         alphai = res / pivabs
         jhiti  = j
  400 continue

*     ------------------------------------------------------------------
*     See if a feasible and/or infeasible variable blocks.
*     ------------------------------------------------------------------
      blockf = jhitf .gt. 0
      blocki = jhiti .gt. 0
      unbndd = .not. ( blockf  .or.  blocki )
      if (       unbndd ) go to 900
      if ( .not. blockf ) go to 500

*     ------------------------------------------------------------------
*     A variable hits a bound for which it is currently feasible.
*     the corresponding step alphaf is not used, so no need to get it,
*     but we know that alphaf .le. alphap, the step from pass 1.
*     ------------------------------------------------------------------
      kp     = jhitf
      pivot  = y(kp)
      hitlow = pivot .lt. zero

*     If there is a choice between alphaf and alphai, it is probably
*     best to take alphai (so we can kick the infeasible variable jhiti
*     out of the basis).
*     However, we can't if alphai is bigger than alphap.

      if (   .not. blocki   ) go to 600
      if (alphai .gt. alphap) go to 600

*     ------------------------------------------------------------------
*     An infeasible variable reaches its violated bound.
*     ------------------------------------------------------------------
  500 kp     = jhiti
      pivot  = y(kp)
      hitlow = pivot .gt. zero

*     ------------------------------------------------------------------
*     Try to step exactly onto bound, but make sure the exact step
*     is sufficiently positive.  (exact will be alphaf or alphai.)
*     Since featol increases by tolinc each iteration, we know that
*     a step as large as stepmn (below) will not cause any feasible
*     variables to become infeasible (where feasibility is measured
*     by the current featol).
*     ------------------------------------------------------------------
  600 if ( hitlow ) then
         bound = blBS(kp)
      else
         bound = buBS(kp)
      end if
      unbndd = abs( bound ) .ge. plinfy
      if ( unbndd ) go to 900

      stepmn = tolinc / abs( pivot )
      exact  = (bound - xBS(kp)) / pivot
      alpha  = max( stepmn, exact )
      onbnd  = alpha .eq. exact
      move   = exact .ge. stepmn
      if (.not. move) nDegen = nDegen + 1
      return

*     ------------------------------------------------------------------
*     Unbounded.
*     ------------------------------------------------------------------
  900 alpha  = stepmx
      move   = .true.
      onbnd  = .false.

*     end of s5step
      end

