*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
*     File  sn25bfac.f
*
*     s2Bfac   s2Bmap   s2newB   s2BLU    s2Bmod   s2Bsol   s2sing
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s2Bfac( typeLU, needLU, newLU, newB,
     $                   ierror, iObj, itn, lPrint, LUreq,
     $                   m, mBS, n, nb, nnL, nS, nSwap, xNorm,
     $                   ne, nka, a, ha, ka, 
     $                   kBS, hs, b, lenb,
     $                   bl, bu, blBS, buBS, xBS, xs,
     $                   iy, iy2, y, y2, 
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*2        typeLU
      logical            needLU, newLU, newB
      integer            ha(ne), hs(nb)
      integer            ka(nka), kBS(mBS)
      integer            iy(mBS), iy2(m)
      integer            iw(leniw)
      double precision   rw(lenrw)
      double precision   a(ne), b(*)
      double precision   bl(nb), bu(nb), xs(nb)
      double precision   blBS(mBS), buBS(mBS), xBS(mBS)
      double precision   y(mBS), y2(m)

*     =================================================================
*     s2Bfac  computes an acceptable xs such that  ( A  -I )*xs = b.
*     The LU factorization of the basis is computed if necessary.
*
*     If typeLU = 'B ', the usual B = LU is computed.
*     If typeLU = 'BS', there are some superbasics and we want to
*                       choose a good basis from the columns of (B S).
*                       We first factorize (B S)' to obtain a new B.
*                       Then B = LU is computed as usual.
*     If typeLU = 'BT', we should TRY 'B ' first and go back to 'BS'
*                       only if B seems ill-conditioned.
*
*     15 Nov 1991: First version based on Minos routine m2bfac.
*     29 Oct 1993: typeLU options implemented.
*                nSwap returns the number of (B S) changes.
*     22 Apr 1994: Retry with reduced LU Factor tol
*                if s2BLU says there was large growth in U.
*     22 Apr 1994: 'BT' option implemented to save R more often each
*                major iteration.
*     02 Apr 1996: kObj added to mark position of Obj slack in B.
*     14 Jul 1997: Thread-safe version.
*     15 Jul 1997: Current version of s2Bfac.
*     =================================================================
      logical            BS, BT, needx, prnt10
      parameter         (maxLU     = 15)
      parameter         (zero      = 0.0d+0)
      integer            dUmin, Umin
      parameter         (dUmin     = 164)
      parameter         (Umin      = 190)
      parameter         (nSing     = 161)
      parameter         (minlen    = 163)
      parameter         (nFac      = 210)
      parameter         (nBFac     = 211)
      parameter         (LUitn     = 215)
      parameter         (LUmod     = 216)
      parameter         (kObj      = 220)
*     ------------------------------------------------------------------
      iPrint    = iw( 12)
      iSumm     = iw( 13)

      eps2      = rw(  4)
      maxrw     = iw(  3)
      maxiw     = iw(  5)
      lprDbg    = iw( 80)
      lena      = iw(213)
      maxLUi    = iw(331)
      maxLUr    = iw(332)
      ip        = iw(333)
      iq        = iw(334)
      locr      = iw(338)
      LUa       = iw(341)
      indc      = iw(343)
      indr      = iw(344)

      needx     = .true.
      newB      = .false.
      BS        = .false.
      BT        = .false.

      nSwap     = 0
      nLU       = 0
      nrefn     = 0
      nBS       = m + nS
      prnt10    = iPrint .gt. 0  .and.  lPrint .ge. 10

*     Initialize Umin and nBfac on first entry.
*     nBfac  counts consecutive B factorizations (reset if BS is done).
*     Umin   is the smallest diagonal of U after last BS factor.

      if (iw(nFac) .eq. 0) then
         rw(Umin)  = zero
         iw(nBfac) = 0
      end if

      if (needLU) then
         iw(nFac)  = iw(nFac)  + 1
         iw(nBfac) = iw(nBfac) + 1
         if ( prnt10 ) write(iPrint, 1005) iw(nFac), LUreq, itn
         iw(LUitn) = 0
         iw(LUmod) = 0
         LUreq     = 0

*        ---------------------------------------------------------------
*        Set local logicals to select the required type of LU.
*        We come back to 100 if a BT factorize looks doubtful.
*        If BT was requested but we haven't done BS yet,
*        might as well do BS now.
*        ---------------------------------------------------------------
         BT     =  typeLU .eq. 'BT'  .and.  nS       .gt. 0
         BS     = (typeLU .eq. 'BS'  .and.  nS       .gt. 0   )  .or.
     $            (       BT         .and.  rw(Umin) .eq. zero)
      end if

  100 ierror    = 0
      iw(nsing) = 0

      if ( BS ) then
*        ---------------------------------------------------------------
*        Repartition (B S) to get a better B.
*        ---------------------------------------------------------------
         BT        = .false.
         iw(nBfac) = 1

*        Load the basics into kBS.
   
         k      = 0
         do 105, j  = 1, nb
            if (hs(j) .eq. 3) then
               k      = k + 1
               kBS(k) = j
            end if
  105    continue

         if (k .eq. m) then
*           ------------------------------------------------------------
*           We have the right number of basics.
*           1. Factorize (B S)'.
*           2. Apply the resulting row permutation to the columns
*              of (B S).
*           ------------------------------------------------------------
*
*           s2BLU  returns the following values:
*           inform = 0  if the LU factors were computed.
*                  = 1  if there are singularities (nSing gt 0).
*                  = 2  if there was large growth in U.
*                  = 3  if the matrix B has an illegal row or 
*                       column index.
*                  = 4  if an entry of B has the same indices as
*                       an earlier entry.
*                  = 5  Not used.
*                  = 6  Not used.
*                  = 7  if insufficient storage for the LU.
*                       minlen is an estimate of the necessary
*                       value of  lena.
*                  = 8  if there is a fatal error in lu1fac.

            call s2BLU ( 'BS', inform, lPrint, m, n, nBS,
     $                   ne, nka, a, ha, ka,
     $                   kBS, iw(ip), rw(LUa), iw(indc), iw(indr), lena, 
     $                   iy, iy2, y,
     $                   iw, leniw, rw, lenrw )

            if (inform .ge. 7) then
               ierror = 20
               go to 400
            else if (inform  .ge. 3) then
               ierror = 21
               go to 400
            end if

            call s2newB( nBS, m, nb,
     $                   hs, iw(ip), kBS, iy, iw(locr), nSwap )

            if (nSwap .gt. 0) newB = .true.
            if ( prnt10 ) then
               write(iPrint, 1000) nSwap
            end if
         end if
      end if ! BS

*     ==================================================================
*     Main loop to obtain a successful LU factorization.
*     typeLU is not used.  (We are always factoring just B.)
*     ==================================================================
*+    while (needx  .and.  ierror .eq. 0) do
  200 if    (needx  .and.  ierror .eq. 0) then
         if (needLU) then
            if (iw(nSing) .gt. 0) then
*              ---------------------------------------------------------
*              The basis was judged to be singular during the previous
*              factorize.  Suspect columns are indicated by non-positive
*              components of y(*).  Replace them by suitable slacks and 
*              try again.
*              11-Nov-94: Superbasic slacks no longer exist, so we don't
*                         have to update kBS(m+1:nS).
*              ---------------------------------------------------------
               newB = .true.
               call s2sing( mBS, m, n, nb, iPrint, iSumm,
     $                      y, iw(ip), iw(iq), bl, bu, hs, kBS, xs )
            end if

            nBS    = m + nS

*           If a BS swap was delayed by a refactorization, the last
*           superbasic will be a slack.  Check to see if s2sing made the
*           slack basic.

            if (nS .gt. 0) then
               jq  = kBS(nBS)
               if (hs(jq) .eq. 3) then
                  nS    = nS    - 1
                  nBS   = m     + nS
                  nSwap = nSwap + 1
               end if
            end if

*           ------------------------------------------------------------
*           Normal B = LU.
*           Load the basic variables into  kBS,  slacks first.
*           Set  kObj  to tell us where the linear objective is.
*           ------------------------------------------------------------
            jObj     = n + iObj
            iw(kObj) = 0
            k        = 0
            do 310, j = n+1, nb
               if (hs(j) .eq. 3) then 
                  k      = k + 1
                  kBS(k) = j
                  if (j .eq. jObj) iw(kObj) = k
               end if
  310       continue

            nBelem  = k
            nslack  = k
            nonlin  = 0

            do 320, j = 1, n
               if (hs(j) .eq. 3) then
                  k = k + 1
                  if (k .le. m) then
                     kBS(k) = j
                     nBelem = nBelem + ka(j+1) - ka(j)
                     if (j .le. nnL) nonlin = nonlin + 1
                  end if
               end if
  320       continue

            nBasic = k

*           ------------------------------------------------------------
*           Break if the number of basics is inconsistent.
*           ------------------------------------------------------------
            if (nBasic .gt. m) then
               ierror = 32
               go to 200
            end if

            if (nBasic .lt. m) then

*              Not enough basics.
*              Set the remaining kBS(k) = 0 for s2BLU and s2sing.

               call iload ( m-nBasic, 0, kBS(nBasic+1), 1 )
            end if

            lin    = nBasic - nSlack - nonlin
            if (lin .lt. 0) lin = 0
            Bdnsty = 100.0d+0*nBelem / (m*m)
            if ( prnt10 ) then
               write(iPrint, 1010) nonlin, lin, nSlack, nBelem, Bdnsty
            end if

*           ------------------------------------------------------------
*           Load the basis matrix into the LU arrays and factorize it.
*           ------------------------------------------------------------
            call s2BLU ( 'B ', inform, lPrint, m, n, nBS,
     $                   ne, nka, a, ha, ka,
     $                   kBS, iw(ip), rw(LUa), iw(indc), iw(indr), lena, 
     $                   iy, iy2, y,
     $                   iw, leniw, rw, lenrw )

            nLU       = nLU + 1
            needLU    = inform .gt. 0
            
            if (     inform .ge. 7) then
               ierror = 20
            else if (inform .eq. 2) then
*              --------------------------------------------------------
*              s2BLU says there was large growth in U.
*              LU Factor tol has been reduced.  Try again.
*              --------------------------------------------------------
               ierror = 0
               nLU    = 0
               go to 200
            else if (inform  .ge. 3) then
               ierror = 21
            else if (inform  .gt. 0  .and.  nLU .gt. maxLU) then
               ierror = 22
            end if

            if (ierror .gt. 0) go to 200
         end if

         if ( BS ) then
*           -----------------------------------------------------------
*           We did a BS factorize this time.
*           Save the smallest diag of U.
*           -----------------------------------------------------------
            rw(Umin) = rw(dUmin)

         else if ( BT ) then
*           -----------------------------------------------------------
*           (We come here only once.)
*           See if we should have done a BS factorize after all.
*           In this version we do it if any of the following hold:
*           1. dUmin (the smallest diagonal of U) is noticeably smaller
*              than Umin (its value at the last BS factor).
*           2. dUmin is pretty small anyway.
*           3. B was singular.
*           nBfac  makes BS increasingly likely the longer we
*           keep doing B and not BS.
*           -----------------------------------------------------------
            BT     = .false.
            Utol   = rw(Umin) * 0.1d+0 * iw(nBfac)
            BS     = rw(dUmin) .le. Utol   .or.
     $               rw(dUmin) .le. eps2   .or.
     $               iw(nSing) .gt. 0
            if ( BS ) then
               needLU = .true.
               go to 100
            end if

         else if (nS .eq. 0) then
            rw(Umin) = rw(dUmin)
         end if ! BS

         if (.not. needLU) then
*           -----------------------------------------------------------
*           We have a nonsingular B such that B = LU.
*           Compute the basic variables and check that  (A -I)*xs = b.
*           s5setx also loads the basic/superbasic variables in xBS.
*           -----------------------------------------------------------
            call s5setx( 'Reset xBS', inform, itn,
     $                   m, n, nb, nBS, rowerr, xNorm,
     $                   ne, nka, a, ha, ka, 
     $                   b, lenb, kBS, xBS, 
     $                   xs, y, y2, iw, leniw, rw, lenrw )
            
            nrefn = nrefn + 1
            needx = inform .gt. 0

            if (needx) then
               if (nLU .gt. 0) then
                  if (nrefn .gt. 2) then

*                    Give up after 2 tries.

                     needx  = .false.
                  end if
               else
                  needLU = .true.
               end if
            end if
         end if
         go to 200
      end if
*+    end while
*     ==================================================================
*     Tidy up

  400 if (ierror .eq. 0) then
*        --------------------------------------------------------------
*        Normal exit.
*        Load the basic/superbasic bounds into blBS, buBS.
*        --------------------------------------------------------------
         do 500, k  = 1, nBS
            j       = kBS(k)
            blBS(k) = bl(j) 
            buBS(k) = bu(j)
  500    continue

         newLU = nLU .gt. 0

         if (lprDbg .eq. 100) then
            write(iPrint, 7000) (kBS(k), xBS(k), k=1, nBS)
         end if
      else 
*        --------------------------------------------------------------
*        Error exits.
*        --------------------------------------------------------------
         if (ierror .eq. 20) then
*           -------------------------------------------
*           Insufficient storage to factorize B.
*           -------------------------------------------
            more   = iw(minlen) -   lena 
            newi   = maxiw      + 2*more
            newr   = maxrw      +   more
            if (maxLUi .lt. maxLUr) then
               if (iPrint .gt. 0) write(iPrint, 9201)
               if (iSumm  .gt. 0) write(iSumm , 9201)
            else
               if (iPrint .gt. 0) write(iPrint, 9202)
               if (iSumm  .gt. 0) write(iSumm , 9202)
            end if

            if (iPrint .gt. 0) then
               write(iPrint, 9203) maxiw, newi, maxrw, newr
            end if
            if (iSumm  .gt. 0) then
               write(iSumm , 9203) maxiw, newi, maxrw, newr
            end if
         else if (ierror .eq. 21) then
*           -------------------------------------------
*           Error in the LU package.
*           -------------------------------------------
            if (iPrint .gt. 0) write(iPrint, 9210)
            if (iSumm  .gt. 0) write(iSumm , 9210)
         else if (ierror .eq. 22) then
*           -------------------------------------------
*           The basis is singular after maxLU tries.
*               Time to give up.
*           -------------------------------------------
            if (iPrint .gt. 0) write(iPrint, 9220) maxLU
            if (iSumm  .gt. 0) write(iSumm , 9220) maxLU
         else if (ierror .eq. 32) then
*           -------------------------------------------
*           Wrong number of basics.
*           -------------------------------------------
            if (iPrint .gt. 0) write(iPrint, 9320) nBasic
            if (iSumm  .gt. 0) write(iSumm , 9320) nBasic
         end if
      end if

      return

 1000 format(  ' --> BS Factorize.   nSwap = ', i6 )
 1005 format(/ ' Factorize', i6,
     $         '    Demand', i9, '    Iteration', i6)
 1010 format(  ' Nonlinear', i6, '    Linear', i9, '    Slacks', i9,
     $         '    Elems', i10, '    Density', f8.2)
 7000 format(/ ' BS and SB values:' // (5(i7, g17.8)))

 9201 format(  ' EXIT -- not enough integer storage',
     $         ' for the basis factors')
 9202 format(  ' EXIT -- not enough real storage',
     $         ' for the basis factors')
 9203 format(/ 24x, '        Current    Recommended'
     $       / ' Total integer workspace', 2i15
     $       / ' Total real    workspace', 2i15)
 9210 format(  ' EXIT -- error in basis package')
 9220 format(  ' EXIT -- singular basis',
     $         ' after ', i2, ' factorization attempts')
 9320 format(  ' EXIT -- system error.  Wrong no. of basic variables:',
     $           i8)

*     end of s2Bfac
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s2Bmap( m, n, ne,
     $                   miniw, minrw, maxiw, maxrw, niwmin, nrwmin,
     $                   iw, leniw )

      implicit           double precision (a-h,o-z)
      integer            iw(leniw)

*     ==================================================================
*     s2Bmap sets up the core allocation for the basis factors.
*     It is called by s5Mem and s8Mem.
*
*     Normally the storage is for B = LU.
*     For nonlinear problems, we may also need to factorize (B S)' = LU,
*     where S has at most maxS columns.
*
*     On entry:
*     miniw, minrw  say where the LU arrays can start in iw(*), rw(*).
*     maxiw, maxrw  say where the LU arrays must end  in iw(*), rw(*).
*
*     On exit:
*     niwmin, nrwmin  estimate the minimum length of iw(*), rw(*).
*     The LU routines will have some room to maneuver if the arrays
*     are at least that long.  (Later LU messages may ask for more.)
*     ------------------------------------------------------------------
*
*     15 Nov 1991: First version based on Minos 5.4 routine m2bmap.
*     08 Nov 1993: Generalized to allow room for (B S)'.
*     11 Nov 1994: rw(*) replaced by iw(*) and rw(*).
*     14 Jul 1997: Thread-safe version.
*     18 Jul 1997: Current version of s2Bmap.
*     ==================================================================
      maxS      = iw( 57)

      mBS       = m + maxS

*     Allocate arrays for an  mLU x nLU  matrix.

      mLU    = mBS
      nLU    = m

*     LU integer workspace is  iw(ip : maxiw).
*     indc and indr are made as long as possible.

      ip     = miniw
      iq     = ip     + mLU
      lenc   = iq     + nLU
      lenri  = lenc   + nLU
      locc   = lenri  + mLU
      locr   = locc   + nLU
      iploc  = locr   + mLU
      iqloc  = iploc  + nLU
      lastiw = iqloc  + mLU

      maxLUi = (maxiw - lastiw) / 2
      indc   = lastiw
      indr   = indc   + maxLUi

*     LU real workspace is  rw(LUa:maxrw)

      LUa    = minrw
      lastrw = minrw
      maxLUr = maxrw - lastrw

*     LUSOL thinks indc(*), indr(*) and A(*) are all of length lena.

      lena   = min( maxLUi, maxLUr )

*     Estimate the number of nonzeros in the basis factorization.
*     necolA = estimate of nonzeros per column of  a.
*     We guess that the density of the basis factorization is
*     2 times as great, and then allow 1 more such lot for elbow room.

      necolA = ne / n
      necolA = max( necolA, 5 )
      minA   = 3 * min( m, n ) * necolA
      niwmin = lastiw + 2*minA
      nrwmin = lastrw +   minA

      iw(213) = lena  
      iw(331) = maxLUi
      iw(332) = maxLUr
      iw(333) = ip    
      iw(334) = iq    
      iw(335) = lenc  
      iw(336) = lenri 
      iw(337) = locc  
      iw(338) = locr  
      iw(339) = iploc 
      iw(340) = iqloc 
      iw(341) = LUa   
      iw(343) = indc  
      iw(344) = indr   

*     end of s2Bmap
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s2newB( nBS, m, nb, hs, ip, kBS, kBSold, locr, nSwap )

      implicit           double precision (a-h,o-z)
      integer            hs(nb), ip(nBS)
      integer            kBS(nBS), kBSold(nBS), locr(nBS)

*     ------------------------------------------------------------------
*     s2newB  permutes kBS(*) to reflect the permutation (B S)P,
*     where P is in ip(*).  It updates hs(*) accordingly.
*     kBSold(*) and locr(*) are needed for workspace.
*
*     30 Oct 1993: First version.
*     04 Nov 1993: kBSold, nSwap used to save old R if there's no
*                  change in the set of superbasics.
*     ------------------------------------------------------------------
      nSwap = 0
      m1    = m   + 1
      nS    = nBS - m
      call icopy ( nBS, kBS    , 1, locr      , 1 )
      call icopy ( nS , kBS(m1), 1, kBSold(m1), 1 )

      do 100, k = 1, nBS
         i      = ip(k)
         j      = locr(i)
         kBS(k) = j
         if (k .le. m) then
            hs(j) = 3
         else
            if (hs(j) .ne. 2) nSwap = nSwap + 1
            hs(j) = 2
         end if
  100 continue

*     Restore the old S ordering if S contains the same variables.

      if (nSwap .eq. 0) then
         call icopy ( nS, kBSold(m1), 1, kBS(m1), 1 )
      end if

*     end of s2newB
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s2BLU ( job, inform, lPrint, m, n, nBS,
     $                   ne, nka, a, ha, ka,
     $                   kBS, ip, alu, indc, indr, lena, 
     $                   iy, iy2, y,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character          job*(*)
      double precision   a(ne), alu(lena)
      integer            ha(ne)
      integer            ip(nBS), indc(lena), indr(lena)
      integer            ka(nka), kBS(nBS)
      integer            iy(nBS), iy2(m)
      integer            iw(leniw)
      double precision   rw(lenrw)
      double precision   y(nBS)

*     ==================================================================
*     s2BLU  factorizes the basis.
*
*     Job = 'B '  Extract the basis elements from the constraint matrix,
*                 and factorize  B  from scratch, so that  B = L U.
*
*     Job = 'BS'  Factorize transpose of (B S), so that  (B') = L*U,
*                                                        (S')
*                 without saving L and U.  Get a new partition of (B S).
*
*     The following tolerances are used...
*
*     luparm(3) = maxcol   lu1fac: Maximum number of columns
*                          searched allowed in a Markowitz-type
*                          search for the next pivot element.
*     luparm(8) = keepLU   lu1fac: keepLU = 1 means keep L and U,
*                                           0 means discard them.
*     parmlu(1) = eLmax1 = Maximum multiplier allowed in  L  during
*                          refactorization.
*     parmlu(2) = eLmax2 = Maximum multiplier allowed during updates.
*     parmlu(3) = small  = Minimum element kept in  B  or in
*                          transformed matrix during elimination.
*     parmlu(4) = Utol1  = Abs tol for flagging small diagonals of  U.
*     parmlu(5) = Utol2  = Rel tol for flagging small diagonals of  U.
*     parmlu(6) = Uspace = Factor allowing waste space in row/col lists.
*     parmlu(7) = dens1    The density at which the Markowitz
*                          strategy should search maxcol columns
*                          and no rows.
*     parmlu(8) = dens2    The density at which the Markowitz
*                          strategy should search only 1 column.
*                          (In one version of lu1fac, the remaining
*                          matrix is treated as dense if there is
*                          sufficient storage.)
*
*     On exit,
*     inform = 2 if the LU tolerances were tightened because there was
*                excessive growth in U.
*
*     20 Oct 1990  Initial version based on Minos routine m2bsol.
*     07 Nov 1993: Add option to factorize (B S)'
*     06 Mar 1994: Include all rows of (B S), even if B contains slacks.
*     22 Apr 1994: Test for excessive growth in U.
*     14 Jul 1997: Thread-safe version.
*     14 Jul 1997: Current version of s2BLU.
*     ==================================================================
      real               bdincr 
      logical            overfl
      parameter         (one = 1.0d+0, two = 2.0d+0)
      integer            eLmax1, eLmax2
      parameter         (eLmax1 = 151)
      parameter         (eLmax2 = 152)
      parameter         (keepLU = 158)
      parameter         (minlen = 163)
*     ------------------------------------------------------------------
      iPrint    = iw( 12)
      iSumm     = iw( 13)

      iq        = iw(334)
      lenc      = iw(335)
      lenri     = iw(336)
      locc      = iw(337)
      locr      = iw(338)
      iploc     = iw(339)
      iqloc     = iw(340)

      inform    = 0

      if (job(1:2) .eq. 'B ') then
*        ---------------------------------------------------------------
*        Form B and load it into LUSOL.
*        ---------------------------------------------------------------
         nz   = 0
         do 200, k = 1, m
            j      = kBS(k)
            if (j .eq. 0) then
*              --------------------------
*              Relax, just a zero column.
*              --------------------------
            else if (j .le. n) then
*              ---------------------
*              Basic column is in A.
*              ---------------------
               do 100, i   = ka(j), ka(j+1)-1
                  ir       = ha(i)
                  nz       = nz + 1
                  alu(nz)  = a(i)
                  indc(nz) = ir
                  indr(nz) = k
  100          continue
            else 
*              ---------------------
*              Basic slacks.
*              ---------------------
               nz       =   nz + 1
               alu(nz)  = - one
               indc(nz) =   j - n
               indr(nz) =   k
            end if
  200    continue

         iw(minlen) = nz*5/4
         if (iw(minlen) .gt. lena) go to 900
         iw(keepLU) = 1

*        iy and iy2 are work vectors
*        y is an output parameter, used by s2sing.

         call lu1fac( m, m, nz, lena, iw(151), rw(151),
     $                aLU, indc, indr, ip, iw(iq),
     $                iw(lenc), iw(lenri), iw(locc), iw(locr),
     $                iw(iploc), iw(iqloc), iy, iy2, y, inform )

         amax      = rw(160)
         eLmax     = rw(161)
         Umax      = rw(162)
         dUmin     = rw(164)
         nDens1    = iw(167)
         nDens2    = iw(168)
         lenL      = iw(173)
         lenU      = iw(174)
         ncp       = iw(176)
         mersum    = iw(177)
         nUtri     = iw(178)
         nLtri     = iw(179)

         bdincr    = real((lenL + lenU - nz)*100.0e+0/max( nz, 1 ))
         avgmer    = mersum
         floatm    = m
         avgmer    = avgmer / floatm
         growth    = ddiv( Umax, Amax, overfl )
         nbump     = m - nUtri - nLtri
         if (iPrint .gt. 0  .and.  lPrint .ge. 10) then
            write(iPrint, 1000) ncp   , avgmer, lenL , lenU  ,
     $                          bdincr, m     , nUtri, nDens1,
     $                          eLmax , Amax  , Umax , dUmin ,
     $                          growth, nLtri , nbump, nDens2
         end if

*        Test for excessive growth in U.
*        Reduce LU Factor tol and LU Update tol if necessary.
*        (Default values are 100.0 and 10.0)

         if (inform .eq. 0  .and.  growth .ge. 1.0d+8) then
            if (rw(eLmax1) .ge. 2.0d+0) then
                rw(eLmax1) = sqrt( rw(eLmax1) )
                inform     = 2
                if (iPrint .gt. 0) write(iPrint, 1010) rw(eLmax1)
                if (iSumm  .gt. 0) write(iSumm , 1010) rw(eLmax1)
            end if

            if (rw(eLmax2) .gt. rw(eLmax1)) then
                rw(eLmax2) = rw(eLmax1)
                if (iPrint .gt. 0) write(iPrint, 1020) rw(eLmax1)
                if (iSumm  .gt. 0) write(iSumm , 1020) rw(eLmax1)
            end if
         end if            

      else if (job(1:2) .eq. 'BS') then
*        ---------------------------------------------------------------
*        Factorize (B S)' = LU without keeping L and U.
*        ---------------------------------------------------------------
*        Extract (B S)'.
*        ip is needed for workspace.
*        ip(i) = 0 except for rows containing a basic slack.
*        We can ignore all of these rows except for the slack itself.
*        06 Mar 1994: Keep all rows.  (Made a difference in MINOS)
*        22 Apr 1994: Make sure the objective slack remains basic!!
*        11 Nov 1994: Go back to ignoring rows with a slack in B.
*                   This means we don't have to worry about the Obj.
*        ---------------------------------------------------------------
         call iload ( m, 0, ip, 1 )
         do 300, k = 1, nBS
            j      = kBS(k)
            if (j .gt. n) ip(j-n) = 1
  300    continue

         nz     = 0
         do 400, k = 1, nBS
            j      = kBS(k)
            if (j .le. n) then
               do 350, i   = ka(j), ka(j+1)-1
                  ir       = ha(i)
                  if (ip(ir) .eq. 0) then
                     nz       = nz + 1
                     alu(nz)  = a(i)
                     indc(nz) = k
                     indr(nz) = ir
                  end if
  350          continue
            else

*              Treat slacks specially.

               nz       =   nz + 1
               alu(nz)  = - one
               indc(nz) =   k
               indr(nz) =   j - n
            end if
  400    continue

         iw(minlen) = nz*5/4
         if (iw(minlen) .gt. lena) go to 900
         iw(keepLU) = 0

*        Save eLmax1 (the existing LU Factor tol) and set it to a small
*        value for this LU, to give a good (B S) partitioning.

         tolfac     = rw(eLmax1)
         rw(eLmax1) = two
         call lu1fac( nBS, m, nz, lena, iw(151), rw(151),
     $                alu, indc, indr, ip, iw(iq),
     $                iw(lenc ), iw(lenri), iw(locc), iw(locr),
     $                iw(iploc), iw(iqloc), iy, iy2, y, inform )
         rw(eLmax1) = tolfac

         amax      = rw(160)
         eLmax     = rw(161)
         Umax      = rw(162)
         dUmin     = rw(164)

         nDens1    = iw(167)
         nDens2    = iw(168)
         lenL      = iw(173)
         lenU      = iw(174)
         ncp       = iw(176)
         mersum    = iw(177)
         nUtri     = iw(178)
         nLtri     = iw(179)

         bdincr    = real((lenL + lenU - nz)*100.0d+0 / max( nz, 1 ))
         avgmer    = mersum
         floatm    = m
         avgmer    = avgmer / floatm
         nbump     = nBS - nUtri - nLtri
         if (iPrint .gt. 0  .and.  lPrint .ge. 10) then
            write(iPrint, 1100) ncp   , avgmer, lenL , lenU  ,
     $                          bdincr, nBS   , nUtri, ndens1,
     $                                  amax  ,
     $                                  nLtri , nbump, ndens2
         end if
      end if

      return

*     Not enough storage.

  900 inform = 7
      return

 1000 format(' Compressns', i5, '    Merit', f10.2,
     $       '    lenL', i11, '    lenU', i11, '    Increase', f7.2,
     $       '    m ', i6, '  Ut', i6, '  d1', i6, 1p
     $       /  ' Lmax', e11.1, '    Bmax', e11.1,
     $       '    Umax', e11.1, '    Umin', e11.1,
     $       '    Growth',e9.1, '    Lt', i6, '  bp', i6, '  d2', i6)
 1010 format(/ ' LU Factor tol reduced to', f10.2)
 1020 format(/ ' LU Update tol reduced to', f10.2)
 1100 format(' Compressns', i5, '    Merit', f10.2,
     $       '    lenL', i11, '    lenU', i11, '    Increase', f7.2,
     $       '    m ', i6, '  Ut', i6, '  d1', i6, 1p
     $       /             16x, '    BSmax', e10.1,
     $                     57x, '    Lt', i6, '  bp', i6, '  d2', i6)

*     end of s2BLU
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s2Bmod( inform, jrep, m, z,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      double precision   z(m)
      double precision   rw(lenrw)
      integer            iw(leniw)

*     ==================================================================
*     s2Bmod  updates the LU factors of B when column "jrep" is replaced
*             by a vector  v.  On entry,   z  must satisfy  L z = v. 
*             It is overwritten.
*
*     20 Oct 1990  Initial version.
*     14 Jul 1997: Thread-safe version.
*     15 Jul 1997: Current version of s2Bmod.
*     ==================================================================
      lena      = iw(213)
      ip        = iw(333)
      iq        = iw(334)
      lenc      = iw(335)
      lenri     = iw(336)
      locr      = iw(338)
      locc      = iw(337)
      LUa       = iw(341)
      indc      = iw(343)
      indr      = iw(344)

      call lu8rpc( 1, 2, m, m, jrep, z, z,
     $             lena, iw(151), rw(151),
     $             rw(LUa ), iw(indc), iw(indr), iw(ip), iw(iq),
     $             iw(lenc), iw(lenri), iw(locc), iw(locr),
     $             inform, diag, znorm )

*     end of s2Bmod
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s2Bsol( job, inform, m, z, y, iw, leniw, rw, lenrw  )

      implicit           double precision (a-h,o-z)
      character          job*(*)
      double precision   z(m), y(m)
      double precision   rw(lenrw)
      integer            iw(leniw)

*     ==================================================================
*     s2Bsol  solves various systems with the LU factors of B.
*     job  selects one of the following:
*      Job          Action
*      ---          ------
*      'L '    Solve  L*z = z(input).    y  is not touched.
*      'B '    Solve  L*z = z(input)  and solve  B*y = z(input).
*      'Bt'    Solve  B(transpose)*y = z.   Note that  z  is destroyed.
*
*     20 Oct 1990  Initial version.
*     25 Jul 1997: Current version.
*     ==================================================================
      integer            small
      parameter         (small     = 153)
*     ------------------------------------------------------------------
      lena      = iw(213)
      ip        = iw(333)
      iq        = iw(334)
      lenc      = iw(335)
      lenri     = iw(336)
      locc      = iw(337)
      locr      = iw(338)
      LUa       = iw(341)
      indc      = iw(343)
      indr      = iw(344)

      if (job(1:2) .eq. 'L '  .or.  job(1:2) .eq. 'B ') then
*        ---------------------------------------------------------------
*        Solve   L*z = z(input).
*        When LU*y = z is being solved in SNOPT, norm(z) will sometimes
*        be small (e.g. after periodic refactorization).  Hence for
*        solves with L we scale parmlu(3) to alter what lu6sol thinks 
*        is small.
*        ---------------------------------------------------------------
         small0    = rw(small)
         if (job(1:2) .eq. 'B ') rw(small) = small0 * dnrm1s( m, z, 1 )

         call lu6sol( 1, m, m, z, y, 
     $                lena,  iw(151), rw(151),
     $                rw(LUa ), iw(indc), iw(indr), iw(ip), iw(iq),
     $                iw(lenc), iw(lenri), iw(locc), iw(locr),
     $                inform )
         rw(small) = small0

         if (job(1:2) .eq. 'B ') then
*           ------------------------------------------------------------
*           job = 'B 'solve.   Solve  U*y = z.
*           ------------------------------------------------------------
            call lu6sol( 3, m, m, z, y, 
     $                   lena,  iw(151), rw(151),
     $                   rw(LUa ), iw(indc), iw(indr), iw(ip), iw(iq),
     $                   iw(lenc), iw(lenri), iw(locc), iw(locr),
     $                   inform )
         end if

      else if (job(1:2) .eq. 'Bt') then
*        ---------------------------------------------------------------
*        job = 'Bt'ranspose solve.  Solve  B'*y = z.
*        ---------------------------------------------------------------
         call lu6sol( 6, m, m, y, z, 
     $                lena,  iw(151), rw(151),
     $                rw(LUa ), iw(indc), iw(indr), iw(ip), iw(iq),
     $                iw(lenc), iw(lenri), iw(locc), iw(locr),
     $                inform )
      end if

*     end of s2Bsol
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s2sing( mBS, m, n, nb, iPrint, iSumm,
     $                   z, ip, iq, bl, bu, hs, kBS, xs )

      implicit           double precision (a-h,o-z)
      integer            ip(m), iq(m), hs(nb)
      integer            kBS(mBS)
      double precision   bl(nb), bu(nb), z(m), xs(nb)

*     =================================================================
*     s2sing  is called if the LU factorization of the basis appears
*     to be singular.   If  z(j)  is not positive, the  jth  basic
*     variable  kBS(j)  is replaced by the appropriate slack.
*     If any kBS(j) = 0, only a partial basis was supplied.
*     If kBS has length nb, replace the slack by j in kBS(nS+1:nb).
*
*     30 Sep 1991: First version based on minos routine m2sing.
*     29 May 1995: Optional swapping of slack and basic.
*     12 Jul 1997: Thread-safe version.
*     14 Jul 1997: Current version of s2sing.
*     =================================================================
      parameter         (zero = 0.0d+0, nPrint = 5)
*     -----------------------------------------------------------------
      nSing  = 0
      do 200, k = 1, m
         j      = iq(k)
         if (z(j) .le. zero) then
            j   = kBS(j)

            if (j .gt. 0) then

*              Make variable  j  nonbasic (and feasible).
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
            end if

*           Make the appropriate slack basic.
         
            i       = ip(k)
            hs(n+i) = 3
            nSing   = nSing + 1
            if (nSing .le. nPrint) then
               if (iPrint .gt. 0) write(iPrint, 1000) j, i
               if (iSumm  .gt. 0) write(iSumm , 1000) j, i
            end if
         end if
  200 continue

      if (nSing .gt. nPrint) then
         if (iPrint .gt. 0) write(iPrint, 1100) nSing
         if (iSumm  .gt. 0) write(iSumm , 1100) nSing
      end if
      return

 1000 format(' Column', i7, '  replaced by slack', i7)
 1100 format(' and so on.  Total slacks inserted =', i6)

*     end of s2sing
      end

