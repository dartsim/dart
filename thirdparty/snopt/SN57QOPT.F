*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
*     File  sn57qopt.f
*
*     s5core   s5dflt   s5Mem   s5savB   s5solv
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5core( sqHx, qpHx, prbtyp, ierror, itn,
     $                   lenb, lenb0, lenr, lenx0,
     $                   m, maxS, mBS, 
     $                   n, nb, ngQP0, ngObj0, ngObj, nnH, 
     $                   nDegen, nS, nHx, itMax, itQP, 
     $                   minimz, ObjAdd, ObjQP, iObj,
     $                   tolFP, tolQP, tolx,
     $                   nInf, sInf, wtInf, piNorm, xNorm,
     $                   ne, nka, a, ha, ka,
     $                   hElast, hEstat, hfeas, hs, kBS, 
     $                   b, bl, bu, blBS, buBS,
     $                   gObj, gObjQP, gBS, 
     $                   pi, r, rc, rg, rhs, 
     $                   xBS, x0, xs, xdif,
     $                   iy, iy2, y, y2, y1, w,
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      external           sqHx, qpHx
      character*3        prbtyp
      integer            ha(ne), hfeas(mBS)
      integer            hElast(nb), hEstat(nb), hs(nb)
      integer            ka(nka), kBS(mBS)
      integer            iy(mBS), iy2(m)
      double precision   a(ne)
      double precision   b(lenb0)
      double precision   bl(nb), bu(nb), rc(nb), xs(nb)
      double precision   blBS(mBS), buBS(mBS), gBS(mBS), xBS(mBS)
      double precision   rhs(m), pi(m)
      double precision   gObj(ngObj0), gObjQP(ngQP0)
      double precision   x0(*), xdif(ngQP0)
      double precision   r(lenr), rg(maxS)
      double precision   y(nb), y2(nb), y1(m), w(nb)

      character*8        cu(lencu), cw(lencw)
      integer            iu(leniu), iw(leniw)
      double precision   ru(lenru), rw(lenrw)

*     ==================================================================
*     s5core  solves the current problem.  A basis is assumed to be
*     specified by nS, hs, x and the superbasic parts of kBS.
*     In particular, there must be nS values hs(j) = 2, and the
*     corresponding j's must be listed in kBS(m+1) thru kBS(m+nS).
*     The ordering in kBS matches the reduced Hessian R (if any).
*
*     A basis is assumed
*     to be specified by nS, hs(*), xs(*) and the superbasic parts of
*     kBS(*).  In particular, there must be nS values hs(j) = 2, and
*     the corresponding j's must be listed in kBS(m+1) thru kBS(m+nS).
*     The ordering in kBS matches the reduced Hessian R (if any).
*
*     05 Oct 1994: First version of s5core.
*     06 Aug 1996: Min Sum option added.
*     14 Jul 1997: Thread-safe version.
*     07 Feb 1998: Current version of s5core.
*     ==================================================================
      logical            Elastc, gotR  , needLU, needx
      parameter         (zero = 0.0d+0)
      character*2        typeLU, prob
      character*20       contyp

      parameter         (MnrHdg    = 223)
      parameter         (MjrHdg    = 224)
      parameter         (MjrSum    = 225)
*     ------------------------------------------------------------------
      iPrint     = iw( 12)
      iSumm      = iw( 13)
      lvlInf     = iw( 73)
      MnrPrt     = iw( 93)
      lEmode     = iw( 51)
      laScal     = iw(274)

      lPrint     = MnrPrt
      iw(MnrHdg) = 0
      iw(MjrHdg) = 0
      iw(MjrSum) = 0

      ngQP      = max( ngObj, nnH )
      Elastc    = lEmode .eq. 2

      ObjQP     = zero
      prob      = prbtyp(1:2)
      contyp    = 'linear rows'

      lenrhs = lenb
      if (lenrhs .gt. 0) then
         call dcopy ( lenrhs, b, 1, rhs, 1 )
      end if

      needLU = .true.
      needx  =  needLU

      if (prob .eq. 'FP') then
*        ---------------------------------------------------------------
*        Find a feasible point.
*        ---------------------------------------------------------------
         call s5LP  ( 'FPLinear constrnts',
     $                contyp, Elastc, needLU, needx, ierror,
     $                m, n, nb, nDegen, itMax, itQP, itn,
     $                lEmode, lvlInf, lPrint,
     $                minimz, ObjAdd, iObj, tolFP, tolQP, tolx,
     $                nInf, sInf, wtInf, piNorm, rgNorm, xNorm,
     $                ne, nka, a, ha, ka,
     $                hElast, hEstat, hfeas, hs, kBS, rw(laScal), 
     $                rhs, lenrhs, bl, bu, blBS, buBS,
     $                gBS, pi, rc, xBS, xs,
     $                iy, iy2, y, y2, y1,
     $                cw, lencw, iw, leniw, rw, lenrw )

      else if (prob .eq. 'LP'  .and.  ngObj .eq. 0) then
*        ---------------------------------------------------------------
*        LP with objective row in A.
*        ---------------------------------------------------------------
         call s5LP  ( 'LP ', 
     $                contyp, Elastc, needLU, needx, ierror,
     $                m, n, nb, nDegen, itMax, itQP, itn,
     $                lEmode, lvlInf, lPrint,
     $                minimz, ObjAdd, iObj, tolFP, tolQP, tolx,
     $                nInf, sInf, wtInf, piNorm, rgNorm, xNorm, 
     $                ne, nka, a, ha, ka,
     $                hElast, hEstat, hfeas, hs, kBS, rw(laScal), 
     $                rhs, lenrhs, bl, bu, blBS, buBS,
     $                gBS, pi, rc, xBS, xs,
     $                iy, iy2, y, y2, y1,
     $                cw, lencw, iw, leniw, rw, lenrw )

      else
*        ---------------------------------------------------------------
*        QP or an LP with an explicit objective row.
*        ---------------------------------------------------------------
         call s5fixS( 'Free superbasics', 
     $                m, maxS, mBS, n, nb, nS, hs, kBS, 
     $                bl, bu, blBS, buBS, xs, xBS )
         typeLU = 'BS'
         gotR   = .false.

         call s5QP  ( prbtyp,
     $                contyp, Elastc, ierror,
     $                sqHx, qpHx, gotR, needLU, typeLU, needx,
     $                lenr, lenx0, m, maxS, mBS, n, nb, nDegen,
     $                ngQP, ngQP0, ngObj, ngObj0, nnH, nS, nHx,
     $                itMax, itQP, itn,
     $                lEmode, lvlInf, lPrint, 
     $                minimz, ObjAdd, ObjQP, iObj,
     $                tolFP, tolQP, tolx,
     $                nInf, sInf, wtInf, piNorm, rgNorm, xNorm,
     $                ne, nka, a, ha, ka,
     $                hElast, hEstat, hfeas, hs, kBS, rw(laScal), 
     $                rhs, lenrhs, bl, bu, blBS, buBS,
     $                gObj, gObjQP, gBS, 
     $                pi, r, rc, rg, 
     $                xBS, x0, xs, xdif,
     $                iy, iy2, y, y2, y1, w, 
     $                cu, lencu, iu, leniu, ru, lenru, 
     $                cw, lencw, iw, leniw, rw, lenrw )
      end if

*     Check for fatal errors that have already been signalled.

      if (ierror .ge. 30                      ) return
      if (ierror .ge. 20  .and.  itn    .eq. 0) return

*     ==================================================================
*     Error exits.
*     No error messages have been printed so far.
*     ==================================================================
  800 if (ierror .eq. 0  .or.  ierror .eq. 1) then
*        -------------------------------------------
*        Optimal.
*        -------------------------------------------
*        call s1page( 2, iw, leniw )

         if (nInf .gt. 0) then
*           ----------------------------------------
*           Infeasible.
*           ----------------------------------------
            if (ierror .eq. 1) then
               if (iPrint .gt. 0) write(iPrint, 7010) itn
               if (iSumm  .gt. 0) write(iSumm , 7010) itn
            else if (lEmode .gt. 0) then
               ierror = 1
               if (     lvlInf .eq. 0) then
                  if (iPrint .gt. 0) write(iPrint, 8010) itn
                  if (iSumm  .gt. 0) write(iSumm , 8010) itn
               else if (lvlInf .eq. 1) then
                  if (iPrint .gt. 0) write(iPrint, 8011) itn
                  if (iSumm  .gt. 0) write(iSumm , 8011) itn
               else if (lvlInf .eq. 2) then
                  if (iPrint .gt. 0) write(iPrint, 8012) itn
                  if (iSumm  .gt. 0) write(iSumm , 8012) itn
               end if
            end if
            if (iPrint .gt. 0) write(iPrint, 9010)
            if (iSumm  .gt. 0) write(iSumm , 9010)

         else if (minimz .eq. 0) then
            if (iPrint .gt. 0) write(iPrint, 9015)
            if (iSumm  .gt. 0) write(iSumm , 9015)

         else if (iObj .ne. 0  .or.  ngQP .gt. 0) then
            if (iPrint .gt. 0) write(iPrint, 9000)
            if (iSumm  .gt. 0) write(iSumm , 9000)

            rgtest = rgnorm / piNorm
            if (rgtest .gt. 0.1d+0) then 
               if (iPrint .gt. 0) write(iPrint, 9005)
               if (iSumm  .gt. 0) write(iSumm , 9005)
            end if

         else
            if (iPrint .gt. 0) write(iPrint, 9015)
            if (iSumm  .gt. 0) write(iSumm , 9015)
         end if

      else

         call s1page( 2, iw, leniw )

         if (ierror .eq. 2) then
*           -------------------------------------------
*           Unbounded.
*           -------------------------------------------
            if (iPrint .gt. 0) write(iPrint, 9200)
            if (iSumm  .gt. 0) write(iSumm , 9200)

         else if (ierror .eq. 3) then
*           -------------------------------------------
*           Too many iterations.
*           -------------------------------------------
            if (itQP .ge. itMax) then
               if (iPrint .gt. 0) write(iPrint, 9301)
               if (iSumm  .gt. 0) write(iSumm , 9301)
            else
               if (iPrint .gt. 0) write(iPrint, 9302)
               if (iSumm  .gt. 0) write(iSumm , 9302)
            end if

         else if (ierror .eq. 4) then
*           -------------------------------
*           Weak solution
*           -------------------------------
            if (iPrint .gt. 0) write(iPrint, 9600)
            if (iSumm  .gt. 0) write(iSumm , 9600)

         else if (ierror .eq. 5) then
*           -------------------------------
*           Superbasic limit too small.
*           -------------------------------
            if (iPrint .gt. 0) write(iPrint, 9500) maxS
            if (iSumm  .gt. 0) write(iSumm , 9500) maxS

         else if (ierror .eq. 6) then
*           ---------------------------------
*           Hessian appears to be indefinite.
*           ---------------------------------
            if (iPrint .gt. 0) write(iPrint, 9400)
            if (iSumm  .gt. 0) write(iSumm , 9400)
         end if
      end if

      return

 7010 format(  ' Itn', i7, ': Infeasible non-elastic variables')
 8010 format(  ' Itn', i7, ': Infeasible elastic variables')
 8011 format(  ' Itn', i7, ': Obj + weighted elastics minimized')
 8012 format(  ' Itn', i7, ': Elastic variables minimized')

 9000 format(  ' EXIT -- optimal solution found')
 9005 format(/ ' XXX  WARNING -- reduced gradient is large --',
     $         ' solution is not really optimal')
 9010 format(  ' EXIT -- the problem is infeasible')
 9015 format(  ' EXIT -- feasible point found')
 9200 format(  ' EXIT -- the problem is unbounded ',
     $         ' (or badly scaled)')
 9301 format(  ' EXIT -- too many iterations')
 9302 format(  ' EXIT -- iteration limit exceeded')
 9400 format(  ' EXIT -- QP Hessian appears to be indefinite')
 9500 format(  ' EXIT -- the superbasics limit is too small:', i7)
 9600 format(  ' EXIT -- weak solution found')

*     end of s5core
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5dflt( job,
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
*     See snworkspace.doc for full documentation of the
*     cw, iw and rw arrays.
*
*     15 Nov 1991: first version.
*     14 May 1998: Current version of s5dflt.
*     ==================================================================
      integer            tolFP , tolQP , tolx  ,
     $                   tolpiv, tolrow, tCrash, tolswp, tolfac,
     $                   tolupd, plInfy, bigfx , bigdx , xdlim ,
     $                   Hcndbd, wtInf0, scltol,
     $                   eLmax1, eLmax2, small ,
     $                   Utol1 , Utol2 , Uspace, Dens1 , Dens2 ,
     $                   toldj3

      parameter         (tolFP     =  51)
      parameter         (tolQP     =  52)
      parameter         (tolx      =  56)
      parameter         (tolpiv    =  60)
      parameter         (tolrow    =  61)
      parameter         (tCrash    =  62)
      parameter         (tolswp    =  65)
      parameter         (tolfac    =  66)
      parameter         (tolupd    =  67)
      parameter         (plInfy    =  70)
      parameter         (bigfx     =  71)
      parameter         (bigdx     =  72)
      parameter         (xdlim     =  80)
      parameter         (Hcndbd    =  85)
      parameter         (wtInf0    =  88)
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
      parameter         (maxiu     =   4)
      parameter         (maxcu     =   6)
      parameter         (iRead     =  10)

      parameter         (ngObj     =  26)
      parameter         (nnH       =  27)
      parameter         (lEmode    =  51)
      parameter         (maxR      =  56)
      parameter         (maxS      =  57)
      parameter         (kchk      =  60)
      parameter         (kfac      =  61)
      parameter         (ksav      =  62)
      parameter         (klog      =  63)
      parameter         (kSumm     =  64)
      parameter         (kDegen    =  65)
      parameter         (lvlSrt    =  70)
      parameter         (lvlInf    =  73)
      parameter         (lvlPrt    =  74)
      parameter         (lvlScl    =  75)
      parameter         (lvlSch    =  76)
      parameter         (lvlTim    =  77)
      parameter         (lprDbg    =  80)
      parameter         (lprPrm    =  81)
      parameter         (lprSch    =  82)
      parameter         (lprScl    =  83)
      parameter         (lprSol    =  84)
      parameter         (minmax    =  87)
      parameter         (iCrash    =  88)
      parameter         (itnlim    =  89)
      parameter         (mMinor    =  91)
      parameter         (MnrPrt    =  93)
      parameter         (nParPr    =  94)
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
      parameter         (tenp6  = 1.0d+6,  hundrd = 100.0d+0)

      logical            QP
      character*24       prbtyp(3)
      data               prbtyp /' Maximize...............',
     $                           ' Feasible point only....',
     $                           ' Minimize...............'/
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

      QP         = iw(nnH) .gt. 0

      if (job(1:1) .eq. 'C') then
*        ---------------------------------------------------------------
*        Job  = 'Check'.   Check the optional parameters.
*        ---------------------------------------------------------------
*        Check the values

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
         if (iw(kfac)   .le.    0   ) iw(kfac)   =   100
         if (iw(klog)  .eq. idummy  ) iw(klog)   =   100
         if (iw(kSumm) .eq. idummy  ) iw(kSumm)  =   100
         if (iw(ksav)  .eq. idummy  ) iw(ksav)   =   100
         if (iw(kDegen).eq. idummy  ) iw(kDegen) = 10000

*        Sometimes, frequency 0 means "almost never".

         if (iw(kchk)   .le. 0      ) iw(kchk)   = never
         if (iw(klog)   .le. 0      ) iw(klog)   = never
         if (iw(ksav)   .le. 0      ) iw(ksav)   = never
         if (iw(kSumm)  .le. 0      ) iw(kSumm)  = never
         if (iw(kDegen) .le. 0      ) iw(kDegen) = never

         if (iw(iCrash) .lt. 0      ) iw(iCrash) =  3
         if (iw(minmax) .eq. idummy ) iw(minmax) =  1
         if (iw(minmax) .eq. -1) then
                                      iw(minimz) = -1
         else
                                      iw(minimz) =  1
         end if

         if (iw(maxru)  .lt. 500    ) iw(maxru)  = 500
         if (iw(maxiu)  .lt. 500    ) iw(maxiu)  = 500
         if (iw(maxcu)  .lt. 500    ) iw(maxcu)  = 500

         if (iw(mMinor) .lt. 0      ) iw(mMinor) = max( 1000,5*m )

         if (iw(lprDbg) .lt. 0      ) iw(lprDbg) = 0
         if (iw(lprPrm) .lt. 0      ) iw(lprPrm) = 1
         if (iw(lprSch) .lt. 0      ) iw(lprSch) = never
         if (iw(lprScl) .lt. 0      ) iw(lprScl) = 0
         if (iw(lprSol) .lt. 0      ) iw(lprSol) = 2
         if (iw(lvlSrt) .lt. 0      ) iw(lvlSrt) = 0
         if (iw(lvlPrt) .lt. 0      ) iw(lvlPrt) = 1
                                      iw(MnrPrt) = iw(lvlPrt)

         if (iw(lvlInf) .lt. 0  .or. iw(lvlInf) .gt. 2
     $                              ) iw(lvlInf) = idummy
         if (iw(lvlInf) .eq. idummy ) iw(lvlInf) = 2
         if (iw(lvlSch) .lt. 0      ) iw(lvlSch) = 1

         if (iw(lEmode) .lt. 0  .or.   iw(lEmode) .gt. 2
     $                              ) iw(lEmode) = idummy
         if (iw(lEmode) .eq. idummy ) iw(lEmode) = 1

*        Check superbasics limit and reduced Hessian size.

         if ( QP ) then
            if (iw(maxS) .lt. 0     ) iw(maxS)   = min( 500, iw(nnH)+1 )
         end if
         if (iw(maxS)   .le. 0      ) iw(maxS)   = 1
         if (iw(maxR)   .lt. 0      ) iw(maxR)   = iw(maxS)

         iw(maxR) = max( min( iw(maxR) ,n ) , 1 )
         iw(maxS) = max( min( iw(maxS) ,n ) , 1 )

*        Check other options.

         if (iw(lvlScl) .lt. 0      ) iw(lvlScl) = 2
                                      iw(lvlScl) = min( iw(lvlScl), 2 )
         if (iw(nParPr) .le. 0      ) iw(nParPr) = 10

         cHzbnd = max ( one/(hundrd*eps*dble(iw(maxS))), tenp6 )

         if (rw(plInfy)   .lt. zero ) rw(plInfy) = 1.0d+20
         if (rw(bigfx)    .le. zero ) rw(bigfx)  = 1.0d+15
         if (rw(bigdx)    .le. zero ) rw(bigdx)  = rw(plInfy)
         if (rw(Hcndbd)   .le. zero ) rw(Hcndbd) = cHzbnd
         if (rw(xdlim)    .le. zero ) rw(xdlim)  = 2.0d+0

         if (rw(tCrash)   .lt. zero  .or.
     $       rw(tCrash)   .ge. one  ) rw(tCrash) = 0.1d+0

*        ---------------------------------
*        Set up the parameters for lu1fac.
*        ---------------------------------
         if (iw(maxcol) .lt.  0     ) iw(maxcol) =   5
         if (iw(LUprnt) .eq.  idummy) iw(LUprnt) =  -1

                                      iw(nout)   =  iPrint
         if (iw(lvlPrt) .gt. 10     ) iw(LUprnt) =  0
         if (iw(lprDbg) .eq. 51     ) iw(LUprnt) =  1
         if (iw(lprDbg) .eq. 52     ) iw(LUprnt) =  2
         if (iPrint     .lt.  0     ) iw(LUprnt) = -1
         if (rw(tolFac) .lt. one    ) rw(tolFac) =  100.0d+0
         if (rw(tolUpd) .lt. one    ) rw(tolUpd) =   10.0d+0
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

         if (rw(tolFP)    .le. zero ) rw(tolFP)  =  rw(tolQP)
         if (rw(tolrow)   .le. zero ) rw(tolrow) =  c4
         if (rw(tolswp)   .le. zero ) rw(tolswp) =  eps4
         if (rw(tolx)     .le. zero ) rw(tolx)   =  c6
                                      rw(toldj3) =  rw(tolQP)
         if (rw(scltol)   .le. zero ) rw(scltol) =  0.90d+0
         if (rw(scltol)   .ge. one  ) rw(scltol) =  0.99d+0
         if (rw(tolpiv)   .le. zero ) rw(tolpiv) =  eps1

         if (rw(wtInf0).lt. zero ) rw(wtInf0) =  1.0d+0

         if (iw(iBack)  .eq. iw(iNewB) 
     $                             ) iw(iBack)   = 0
         if (iw(itnlim) .lt. 0     ) iw(itnlim)  = max( 10000, m )

      else if (job(1:1) .eq. 'P'  .and. iPrint .gt. 0) then
*        ---------------------------------------------------------------
*        Job = 'P'rint    Print parameters except if PRINT LEVEL = 0
*                         or SUPPRESS PARAMETERS was specified.
*        ---------------------------------------------------------------
         if (iw(lvlPrt) .gt. 0  .and.  iw(lprPrm) .gt. 0) then
            nQP    = max( iw(ngObj), iw(nnH) )

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
*           LP/QP parameters.
*           --------------------
            write(iPrint, 2300) prbtyp(2+iw(minmax)),
     $                          rw(scltol), rw(tolx)  , iw(itnlim),
     $                          iw(lvlScl), rw(tolQP) , iw(nParPr),
     $                          rw(tCrash), rw(tolpiv), iw(lvlPrt),
     $                          iw(iCrash), rw(wtInf0), iw(lEmode),
     $                                      iw(lvlInf)
*           --------------------
*           QP objective
*           --------------------
            if ( QP ) 
     $      write(iPrint, 2400) nQP       , iw(nnH)   , iw(maxS)  ,
     $                          iw(nnH)   , rw(bigdx) , 
     $                          iw(ngObj) 
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
 2300 format(/ ' LP/QP Parameters'
     $       / ' ----------------'
     $/   a24,                                16x
     $/ ' Scale tolerance........', 0p, f10.3, 6x,
     $  ' Feasibility tolerance..', 1p, e10.2, 6x,
     $  ' Iteration limit........', i10
     $/ ' Scale option...........', i10,       6x,
     $  ' Optimality tolerance...', 1p, e10.2, 6x,
     $  ' Partial  price.........', i10
     $/ ' Crash tolerance........', 0p, f10.3, 6x,
     $  ' Pivot tolerance........', 1p, e10.2, 6x,
     $  ' Print level............', i10
     $/ ' Crash option...........', i10,       6x,
     $  ' Elastic weight.........', 1p, e10.2, 6x,
     $  ' Elastic mode...........', i10
     $/ 80x, 
     $  ' Elastic objective......', i10)
 2400 format(/ ' QP objective'
     $       / ' ------------'
     $/ ' Objective variables....', i10,       6x,
     $  ' Hessian columns........', i10,       6x,
     $  ' Superbasics limit......', i10
     $/ ' Nonlin Objective vars..', i10,       6x,
     $  ' Unbounded step size....', 1p, e10.2
     $/ ' Linear Objective vars..', i10)
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

*     end of s5dflt
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5Mem ( ierror, iPrint, iSumm,
     $                   m, n, ne,
     $                   ngObj, nnH,
     $                   maxR, maxS, 
     $                   maxcw, maxiw, maxrw,
     $                   lencw, leniw, lenrw,
     $                   mincw, miniw, minrw, iw )

      implicit           double precision (a-h,o-z)
      integer            iw(leniw)

*     ==================================================================
*     s5Mem   allocates all array storage for sqopt.
*     using the values:
*        m    , n    , ne
*        maxR , maxS             Set in sqdflt.
*        ngObj, nnH              Set by specs file or argument list.
*
*     These values are used to compute the minimum required storage:
*     mincw, miniw, minrw.
*
*     The SPECS file has been read and values are known for
*     maxcu, maxiu, maxru  (upper limit of user  partition 1)
*     maxcw, maxiw, maxrw  (upper limit of SQOPT partition)
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
*     SQOPT will use only rw(maxru+1:maxrw).  Hence, rw(501:maxru)
*     and possibly rw(maxrw+1:lenrw) may be used as workspace by the
*     user during solution of the problem (e.g., within funobj or
*     funcon).  Similarly for iw(501:maxiu) and iw(maxiw:leniw).
*
*     Setting maxiw and maxrw less than leniw and lenrw may serve to
*     reduce paging activity on a machine with virtual memory, by
*     confining SQOPT (in particular the LU-factorization routines)
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
*     12 Nov 1994: Converted to integer and real storage.
*     06 Aug 1996: First min sum version.
*     14 Jul 1997: Thread-safe version.
*     01 May 1998: First version called by sqMem. This simplified 
*                  version may slightly overestimate needed memory.
*     01 May 1998: Current version of s5Mem.
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
*        maxR , maxS , nnH
*        ngObj    

      mBS      = m     + maxS
      nb       = n     + m
      ngQP     = max( ngObj, nnH )
      iw(lenr) = maxR*(maxR + 1)/2

      nScl     = nb
      nx2      = nnH

*     sqopt can use all of cw, iw and rw
*     except the first user workspace partitions.

      lhfeas = miniw
      lhEsta = lhfeas + mBS
      lkBS   = lhEsta + nb
      liy    = lkBS   + mBS
      liy2   = liy    + mBS
      miniw  = liy2   + m

      laScal = minrw
      lw     = laScal + nScl
      ly     = lw     + nb    
      ly1    = ly     + nb
      ly2    = ly1    + nb
      lxBS   = ly2    + m
      lblBS  = lxBS   + mBS
      lbuBS  = lblBS  + mBS
      lxScl  = lbuBS  + mBS
      lgObjQ = lxScl  + nx2
      lgBS   = lgObjQ + ngQP
      lr     = lgBS   + mBS
      lrg    = lr     + iw(lenr)
      lblslk = lrg    + maxS
      lbuslk = lblslk + m
      lrhs   = lbuslk + m
      lxdif  = lrhs   + m
      minrw  = lxdif  + ngQP

*     Store the addresses in iw.

      iw(267) = lhfeas
      iw(268) = lhEsta
      iw(269) = lkBS
      iw(270) = lblBS
      iw(271) = lbuBS
      iw(272) = lxBS
      iw(273) = lgBS
      iw(274) = laScal
      iw(275) = lxScl
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
      iw(296) = lxdif
      iw(314) = lblslk
      iw(315) = lbuslk

*     Allocate arrays for the basis factorization routines.
*     miniw, minrw point to the beginning of the LU factorization.
*     This is the beginning of free space between calls
*     if the LU factors are allowed to be overwritten.

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

*     end of s5Mem
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5savB( job, ierror, minimz, 
     $                   m, n, nb, ngQP, ngQP0, ngObj, ngObj0,
     $                   nnL, nName, nS, nScl, sclObj,
     $                   itn, nInf, sInf, wtInf,
     $                   iObj, Objtru, 
     $                   pNorm1, pNorm2, piNorm, xNorm,
     $                   ne, nka, a, ha, ka,
     $                   hEstat, hs, aScale, bl, bu, 
     $                   gObj, gObjQP,
     $                   Names, pi, rc, xs, y,
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character          job*(*)
      character*8        Names(nName)
      integer            ha(ne), hEstat(nb), hs(nb)
      integer            ka(nka)
      double precision   a(ne), aScale(nScl), bl(nb), bu(nb)
      double precision   gObjQP(ngQP0), gObj(ngObj0)
      double precision   pi(m), rc(nb), xs(nb), y(m)

      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     s5savB  saves basis files  and/or  prints the solution.
*
*     If job = 'S'ave, the problem is first unscaled, then from 0 to 4 
*     files are saved (Punch file, Dump file, Solution file, 
*     Report file, in that order).
*     A new BASIS file, if any, will already have been saved by s5core.
*     A call with job = 'S'ave must precede a call with job = 'P'rint.
*
*     If job = 'P'rint, the solution is printed under the control of 
*     lprSol (which is set by the solution keyword in the specs file).
*
*     05 Oct 1994: First version based on Minos routine m4savB.
*     07 Oct 1994: Modified specially for QP.
*     12 Nov 1996: Elastic option added
*     14 Jul 1997: Thread-safe version.
*     17 Jul 1997: Current version of s5savB.
*     ==================================================================
      character*4        istate(3)
      logical            feasbl, prnt
      parameter         (one = 1.0d+0)
*     ------------------------------------------------------------------
      tolx      = rw( 56)
      plInfy    = rw( 70)

      iPrint    = iw( 12)
      iSumm     = iw( 13)
      iDump     = iw(121)
      iPnch     = iw(127)
      iReprt    = iw(130)
      iSoln     = iw(131)

      lvlScl    = iw( 75)
      lprSol    = iw( 84)

      feasbl    = nInf .eq. 0
      k         = 1 + ierror
      call s4stat( k, istate )

      if (job(1:1) .eq. 'S') then
*        ---------------------------------------------------------------
*        job = 'S'ave.
*        Compute rc and unscale a, bl, bu, pi, xs, gObj, xNorm
*        and piNorm (but s4soln uses scaled piNorm, so save it).
*        Then save basis files.
*        ---------------------------------------------------------------
*        Compute reduced costs rc(*) for all columns and rows.

         call s2rcA ( feasbl, tolx, iObj, minimz, wtInf,
     $                m, n, nb, ngQP,
     $                ne, nka, a, ha, ka,
     $                hEstat, hs, bl, bu, gObjQP, pi, rc, xs )
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
         
            if (ngObj .gt. 0) 
     $      call dddiv ( ngObj, aScale, 1, gObj  , 1 )
            if (ngQP  .gt. 0) 
     $      call dddiv ( ngQP , aScale, 1, gObjQP, 1 )
            call dddiv ( nb   , aScale, 1, rc    , 1 )

            xNorm  = dnrm1s( nb, xs, 1 )
            piNorm = dnrm1s( m , pi, 1 )
            piNorm = max( piNorm, one )
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

*        Output Punch, Dump, Solution and/or Report files.

         if (iPnch .gt. 0)
     $   call s4pnch( iPnch, n, nb, hs, bl, xs, Names,
     $                cw, lencw, iw, leniw )

         if (iDump .gt. 0)
     $   call s4dump( iDump, nb, hs, xs, Names, 
     $                cw, lencw, iw, leniw )

         piNorm = pNorm1
         if (iSoln .gt. 0)
     $   call s4soln( .true., minimz, m, n, nb, nName, 
     $                ngQP, nS,
     $                itn, nInf, sInf,
     $                iObj, Objtru, piNorm,
     $                ne, nka, a, ha, ka,
     $                hs, aScale, bl, bu,
     $                gObjQP, pi, rc, xs, y,
     $                Names, istate,
     $                cw, lencw, iw, leniw, rw, lenrw )

         if (iReprt .gt. 0)
     $   call s4rept( .true., m, n, nb, nName,
     $                nnL, ngQP, nS,
     $                ne, nka, a, ha, ka,
     $                hs, aScale, bl, bu,
     $                gObjQP, pi, xs, y,
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
     $                   ngQP, nS,
     $                   itn, nInf, sInf,
     $                   iObj, Objtru, piNorm,
     $                   ne, nka, a, ha, ka,
     $                   hs, aScale, bl, bu,
     $                   gObjQP, pi, rc, xs, y,
     $                   Names, istate,
     $                   cw, lencw, iw, leniw, rw, lenrw )
            piNorm = pNorm2
            if (iSumm  .gt. 0) write(iSumm, 1200) iPrint
         else
            if (iSumm  .gt. 0) write(iSumm, 1300)
         end if
      end if

      return

 1010 format(  ' Norm of xs  (scaled)', 1p, e17.1,
     $     2x, ' Norm of pi  (scaled)',     e17.1)
 1020 format(  ' Norm of xs', 1p, e27.1,
     $     2x, ' Norm of pi',     e27.1)
 1030 format(  ' Max Prim inf(scaled)', i9, 1p, e8.1,
     $     2x, ' Max Dual inf(scaled)', i9,     e8.1)
 1040 format(  ' Max Primal infeas   ', i9, 1p, e8.1,
     $     2x, ' Max Dual infeas     ', i9,     e8.1)

 1200 format(/ ' Solution printed on file', i4)
 1300 format(/ ' Solution not printed')

*     end of s5savB
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s5solv( Hx, Hx1, start, lenb, lenx0, m, 
     $                   n, nb, ne, nka, nName,
     $                   iObj, ObjAdd, ObjQP, Objtru, 
     $                   nInf, sInf,
     $                   a, ha, ka, b, bl, bu, gObj, 
     $                   Names, 
     $                   hElast, hs, x0, xs, pi, rc,
     $                   inform, nS, 
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      external           Hx1, Hx
      character*(*)      start
      character*8        Names(nName)
      integer            ha(ne), hElast(nb), hs(nb)
      integer            ka(nka)
      double precision   gObj(*), b(*), x0(*)
      double precision   a(ne), bl(nb), bu(nb)
      double precision   xs(nb), pi(m), rc(nb)

      character*8        cu(lencu), cw(lencw)
      integer            iu(leniu), iw(leniw)
      double precision   ru(lenru), rw(lenrw)

*     ==================================================================
*     s5solv solves the current problem.
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
*                  error condition (see the SQOPT user's guide).
*
*     01 Oct 1994: First version of s5solv.
*     06 Aug 1996: Min Sum option added.
*     14 Jul 1997: Thread-safe version.
*     12 Aug 1997: Current version of s5solv.
*     ==================================================================
      character*8        mName
      logical            infsbl
      logical            needB 
      logical            FP, LP, QP 
      character*4        istate(3)
      character*3        prbtyp
      character*1        ch1

      parameter         (zero = 0.0d+0, one = 1.0d+0)
      integer            sclObj
      parameter         (sclObj = 188)

      parameter         (lvlScl =  75)
      parameter         (nParPr =  94)
      parameter         (ifDefH = 200)
      parameter         (nFac   = 210)

      character*4        line
      data               line /'----'/
*     ------------------------------------------------------------------
      tolFP     = rw( 51)
      tolQP     = rw( 52)
      tolx      = rw( 56)
      tCrash    = rw( 62)
      plInfy    = rw( 70)
      wtInf0    = rw( 88)

      iPrint    = iw( 12)
      iSumm     = iw( 13)
      iLoadB    = iw(122)
      iNewB     = iw(124)
      insrt     = iw(125)
      iOldB     = iw(126)

      ngObj     = iw( 26)
      nnH       = iw( 27)
      lenr      = iw( 28)
      maxS      = iw( 57)

      lEmode    = iw( 51)
      lvlInf    = iw( 73)

      minmax    = iw( 87)
      iCrash    = iw( 88)
      itnlim    = iw( 89)
      MnrPrt    = iw( 93)
      minimz    = iw(199)

      mName     = cw( 51)

      nnCon     = 0
      nnJac     = 0
      nnObj     = 0
      nnL       = 0

*     Addresses

      lhfeas    = iw(267)
      lhEsta    = iw(268)
      lkBS      = iw(269)
      lblBS     = iw(270)
      lbuBS     = iw(271)
      lxBS      = iw(272)
      lgBS      = iw(273)
      laScal    = iw(274)
      lrg       = iw(277)
      lr        = iw(278)
      liy       = iw(279)
      liy2      = iw(280)
      ly        = iw(281)
      ly2       = iw(282)
      lw        = iw(283)
      ly1       = iw(284)
      lgObjQ    = iw(289)
      lrhs      = iw(290)
      lblSlk    = iw(314)
      lbuSlk    = iw(315)
      lxdif     = iw(296)

      lPrint    = MnrPrt
      lenb0     = max( 1    , lenb  )
      ngObj0    = max( 1    , ngObj )
      ngQP      = max( ngObj, nnH   )
      ngQP0     = max( 1    , ngQP  )

      mBS       = m + maxS

*     Figure out what type of problem we have.

      if (minmax .eq. 0  .or.(lEmode .eq. 2  .and.  lvlInf .eq. 2)) then
         prbtyp = 'FP '

      else if (ngQP .eq. 0) then

*        No explicit objective.  Must be some form of LP.

         if (iObj .eq. 0) then
            prbtyp = 'FP '
         else
            prbtyp = 'LP '
         end if
      else

*        Explicit objective. Check if there is a quadratic term.

         if (nnH .gt. 0) then
            prbtyp = 'QP '
         else
            prbtyp = 'LP '
         end if
      end if

      FP = prbtyp .eq. 'FP '
      LP = prbtyp .eq. 'LP '
      QP = prbtyp .eq. 'QP '

*     Count the linear equality and inequality constraints.

      numLEQ = 0
      do 50, j = n+1, nb
         if (bl(j) .eq. bu(j)) numLEQ = numLEQ + 1
   50 continue
      numLC  = m
      numLIQ = numLC - numLEQ

*     Zap the pi(i) in case they are printed without being set.

      call dload ( m, zero, pi, 1 )

*     Initialize a few things.

      ierror     = 0
      iw(ifDefH) = 0          ! QP Hessian may or may not be definite
      iw(nFac)   = 0
      nInf       = 0
      if (iw(lvlScl) .eq. 0) then 
         nScl    = 1
      else
         nScl    = nb
      end if      

      rw(sclObj) = one
      itn        = 0
      itQP       = 0 
      nDegen     = 0
      nHx        = 0
      mxItQP     = itnlim 

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
         needB  = max( iOldB, insrt, iLoadB ) .le. 0
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

      call s1page( 1, iw, leniw )

      if (istart .eq. 0) then
*        ---------------------------------------------------------------
*        Cold start, or Basis file provided.
*        Input a basis file if one exists, thereby defining hs and xs.
*        (Otherwise, s2crsh will be called later to define hs.)
*        ---------------------------------------------------------------
*        We have to initialize xs(n+1:nb) and pi(1:m)
*        before the problem is scaled.
*        The basis files initialize all of xs.
*        One day they may load pi for nonlinear problems.

         call dload ( m, zero, xs(n+1), 1 )
         call dload ( m, zero, pi     , 1 )

         if (iPrint .gt. 0) write(iPrint, 1200)

         if ( needB ) then
            if (iPrint .gt. 0) write(iPrint, 1205)

            if (icrash .eq. 0) then
               needB  = .false.
               lcrash = 0
               call s2crsh( lcrash, lPrint, m, n, nb,
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
            if (ierror .ne. 0) go to 900
         end if

      else if (istart .eq. 1) then
        if (iPrint .gt. 0) write(iPrint, 1210)
      end if ! istart = 0

*     ------------------------------------------------------------------
*     Move x inside its bounds.
*     ------------------------------------------------------------------
      do 120, j = 1, n
         xs(j)   = max( xs(j), bl(j) )
         xs(j)   = min( xs(j), bu(j) )
  120 continue

*     ------------------------------------------------------------------
*     Scale the constraints.
*     ------------------------------------------------------------------
      if (iw(lvlScl) .gt. 0) then
         lsSave     = iw(lvlScl)
         iw(lvlScl) = 1
         call s2scal( lPrint, m, n, nb, nnL, nnCon, nnJac, iw(lhEsta), 
     $                ne, nka, a, ha, ka,
     $                rw(laScal), bl, bu, rw(ly), rw(ly2),
     $                iw, leniw, rw, lenrw )
         iw(lvlScl) = lsSave

         call s2scla( 'Scale', m, n, nb,
     $                iObj, plInfy, rw(sclObj),
     $                ne, nka, a, ha, ka, 
     $                rw(laScal), bl, bu, pi, xs )
         if (ngObj .gt. 0)
     $   call ddscl ( ngObj, rw(laScal), 1, gObj, 1 )
      end if

*     Save all slack bounds.

      call dcopy ( m, bl(n+1), 1, rw(lblSlk), 1 )
      call dcopy ( m, bu(n+1), 1, rw(lbuSlk), 1 )

*     ------------------------------------------------------------------
*     Compute a starting basis for the linear constraints.
*     This means attempting to get feasible for the linear equalities.
*     If the E rows are infeasible, s5core will take care of it.
*     ------------------------------------------------------------------
      if (lprint .gt. 0) then
         if (iPrint .gt. 0) write(iPrint, 1332) (line, j=1,28) 
         if (iSumm  .gt. 0) write(iSumm , 1316) (line, j=1,16)
      end if

      call s5getB( needB, ierror, lenb, m, maxS, mBS, n, nb,
     $             nnCon, nnJac, nnObj, iObj,
     $             nS, nScl, nDegen, numLC, numLEQ, numLIQ,
     $             mxItQP, itQP, itn, lPrint, 
     $             ObjAdd, tolFP, tolQP, tolx,
     $             nInf, sInf, wtInf0, piNorm, rgNorm, xNorm,
     $             ne, nka, a, ha, ka,
     $             hElast, iw(lhEsta), iw(lhfeas), hs, iw(lkBS), 
     $             rw(laScal), b, bl, bu,
     $             rw(lblBS), rw(lbuBS), rw(lblslk), rw(lbuslk),
     $             rw(lgBS), pi, rc, rw(lrhs), 
     $             xs, rw(lxBS),
     $             iw(liy), iw(liy2), rw(ly), rw(ly2), rw(ly1),
     $             cw, lencw, iw, leniw, rw, lenrw )

*     ==================================================================
*     Solve the problem.
*     ==================================================================
      call s1time( 2, 0, iw, leniw, rw, lenrw )
      call s5core( Hx, Hx1, prbtyp, ierror, itn,
     $             lenb, lenb0, lenr, lenx0,
     $             m, maxS, mBS, 
     $             n, nb, ngQP0, ngObj0, ngObj, nnH,
     $             nDegen, nS, nHx, mxItQP, itQP, 
     $             minimz, ObjAdd, ObjQP, iObj,
     $             tolFP, tolQP, tolx,
     $             nInf, sInf, wtInf0, piNorm, xNorm,
     $             ne, nka, a, ha, ka,
     $             hElast, iw(lhEsta),  iw(lhfeas), hs, iw(lkBS),
     $             b, bl, bu, rw(lblBS), rw(lbuBS),
     $             gObj, rw(lgObjQ), rw(lgBS), pi, 
     $             rw(lr), rc, rw(lrg), rw(lrhs),
     $             rw(lxBS), x0, xs, rw(lxdif),
     $             iw(liy), iw(liy2), rw(ly), rw(ly2), rw(ly1), rw(lw),
     $             cu, lencu, iu, leniu, ru, lenru, 
     $             cw, lencw, iw, leniw, rw, lenrw )

      call s1time(-2, 0, iw, leniw, rw, lenrw )

*     ==================================================================
*     Exit.
*     Set output variables and print a summary of the final solution.
*     Objtru is printed in s4newB
*     ==================================================================
  900 inform = ierror
      degen  = 100.0d+0 * nDegen / max( itn, 1 )

      if (iObj .eq. 0) then
         ObjLP  = ObjAdd
      else
         ObjLP  = ObjAdd + xs(n+iObj)*rw(sclObj)
      end if

      if (LP .or. QP) then
         Objtru = ObjLP + ObjQP
      else
         Objtru = zero
      end if

      infsbl = nInf .gt. 0
      xNorm  = dnrm1s( nb, xs, 1 )

*     Count basic nonlinear variables (used only for printing).

      nnb    = 0
      do 910, j = 1, nnH
         if (hs(j) .eq. 3) nnb = nnb + 1
  910 continue

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
                     write(iPrint, 1900) mName, itn, Objtru
         if (infsbl) write(iPrint, 1910) nInf, sInf
         if (QP    ) write(iPrint, 1920) nHx, ObjLP, ObjQP
         if (nS .gt. 0)
     $               write(iPrint, 1970) nS, nnb
                     write(iPrint, 1975) nDegen, degen
      end if

      if (iSumm  .gt. 0) then
                     write(iSumm , 1900) mName, itn, Objtru
         if (infsbl) write(iSumm , 1910) nInf, sInf
         if (QP    ) write(iSumm , 1920) nHx, ObjLP, ObjQP
         if (nS .gt. 0)
     $               write(iSumm , 1970) nS, nnb
                     write(iSumm , 1975) nDegen, degen
      end if

*     ------------------------------------------------------------------
*     Unscale, save basis files and prepare to print the solution.
*     Clock 3 is "Output time".
*     ------------------------------------------------------------------
      call s1time( 3, 0, iw, leniw, rw, lenrw )
      call s5savB( 'Save', ierror, minimz,
     $             m, n, nb, ngQP, ngQP0, ngObj, ngObj0,
     $             nnL, nName, nS, nScl, rw(sclObj),
     $             itn, nInf, sInf, wtInf0,
     $             iObj, Objtru,
     $             pNorm1, pNorm2, piNorm, xNorm,
     $             ne, nka, a, ha, ka,
     $             iw(lhEsta), hs, rw(laScal), bl, bu,
     $             gObj, rw(lgObjQ),
     $             Names, pi, rc, xs, rw(ly),
     $             cw, lencw, iw, leniw, rw, lenrw )

*     If task = 'Print', s5savB prints the solution under the control
*     of lprSol (set by the  Solution  keyword in the SPECS file).
*     The printed solution may or may not be wanted, as follows:
*     
*     lprSol = 0   means      No
*            = 1   means      If optimal, infeasible or unbounded
*            = 2   means      Yes
*            = 3   means      If error condition

      call s5savB( 'Print', ierror, minimz,
     $             m, n, nb, ngQP, ngQP0, ngObj, ngObj0,
     $             nnL, nName, nS, nScl, rw(sclObj),
     $             itn, nInf, sInf, wtInf0,
     $             iObj, Objtru,
     $             pNorm1, pNorm2, piNorm, xNorm,
     $             ne, nka, a, ha, ka,
     $             iw(lhEsta), hs, rw(laScal), bl, bu,
     $             gObj, rw(lgObjQ),
     $             Names, pi, rc, xs, rw(ly),
     $             cw, lencw, iw, leniw, rw, lenrw )

      call s1time(-3, 0, iw, leniw, rw, lenrw )

*     ------------------------------------------------------------------
*     Set Obj for output.
*     Call  Hx  one last time with  nState .ge. 2.
*     Everything has been  unscaled, so we have to disable scaling.
*     ------------------------------------------------------------------
      lsSave     = iw(lvlScl)
      iw(lvlScl) = 0
      nStat1     = 2 + ierror

      if ( FP ) then
         Objtru = zero
      else
         ObjLP  = zero
         if (iObj .ne. 0) then
            ObjLP = xs(n+iObj)*rw(sclObj)
         end if
         Objtru = ObjAdd + ObjLP
         
         if (ngQP .gt. 0) then
            call s5QPfg( Hx, Hx1, minimz, lenx0, 
     $                   n, ngQP, ngObj, ngObj0, nnH, 
     $                   nStat1, nHx, ObjQP,
     $                   ne, nka, a, ha, ka, 
     $                   gObj, rw(lgObjQ), x0, xs, rw(lxdif), 
     $                   cu, lencu, iu, leniu, ru, lenru,
     $                   cw, lencw, iw, leniw, rw, lenrw )
            Objtru = Objtru + ObjQP
         end if
         iw(lvlScl) = lsSave
      end if

      return

 1030 format(/   ' XXX Start parameter not recognized:  ', a)
 1100 format(/   ' Scale option', i3, ',      Partial price', i8
     $       /   ' Partial price section size ( A)', i12
     $       /   ' Partial price section size (-I)', i12)
 1110 format(/   ' Scale option', i3, ',    Partial price', i4)
 1200 format(    ' Initial basis' / ' -------------')
 1205 format(/   ' No basis file supplied')
 1210 format(    ' Warm Start' / ' ----------')
 1316 format(/1x, 16a4 )
 1332 format(/1x, 28a4 ) 

 1900 format(/   ' Problem name', 17x, a8
     $       /   ' No. of iterations', i20,
     $        2x,' Objective value', 1p, e22.10)
 1910 format(    ' No. of infeasibilities', i15,
     $        2x,' Sum of infeas', 1p, e24.10)
 1920 format(    ' No. of Hessian products', i14,
     $        2x,' Linear objective', 1p, e21.10
     $      /40x,' Quadratic objective', 1p, e18.10)
 1970 format(    ' No. of superbasics', i19,
     $        2x,' No. of basic nonlinears', i14)
 1975 format(    ' No. of degenerate steps', i14,
     $        2x,' Percentage', f27.2)

*     end of s5solv
      end

