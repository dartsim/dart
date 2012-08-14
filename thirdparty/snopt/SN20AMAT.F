*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
*     File  sn20amat.f
*
*     s2amat   s2Aprd   s2Bprd   s2Binf   s2Dinf   s2crsh   s2rcA
*     s2scal   s2scla   s2unpk
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s2amat( job,
     $                   m, n, nb,
     $                   nnCon, nnJac, nnObj, iObj,
     $                   ne, nka, a, ha, ka,
     $                   bl, bu, hrtype,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character          job*(*)
      integer            ha(ne), hrtype(m)
      integer            ka(nka)
      integer            iw(leniw)
      double precision   rw(lenrw)
      double precision   a(ne), bl(nb), bu(nb)

*     ==================================================================
*     s2amat defines hrtype, the set of row types.
*     If 'job = 'S',  s2amat also prints the matrix statistics.
*
*     The vector of row-types is as follows:
*        hrtype(i) = 0  for E rows      (equalities)
*        hrtype(i) = 1  for L or G rows (inequalities)
*        hrtype(i) = 2  for N rows      (objective or free rows)
*     They are used in s2scal and s2crsh.
*
*     15 Feb 1991: First version based on Minos 5.4 routine m2amat.
*     21 Feb 1998: Current version.
*     ==================================================================
      parameter        ( zero = 0.0d+0 )
*     ------------------------------------------------------------------
      plInfy = rw( 70)
      iPrint = iw( 12)
      nnL    = max( nnJac, nnObj )

      bplus  =   0.9d+0*plInfy
      bminus = - bplus

*     Construct the vector of row-types.

      nfixed = 0
      nfree  = 0
      nnorml = 0
      do 100, i = 1, m
         j      = n + i
         b1     = bl(j)
         b2     = bu(j)
         itype  = 1
         if (b1 .eq. b2                          ) itype = 0
         if (b1 .le. bminus  .and.  b2 .ge. bplus) itype = 2
         hrtype(i) = itype
         if (itype .eq. 0) nfixed = nfixed + 1
         if (itype .eq. 2) nfree  = nfree  + 1
         if (itype .ne. 1) go to 100
         if (b1 .le. bminus  .or.   b2 .ge. bplus) nnorml = nnorml + 1
  100 continue

      if (job(1:1) .ne. 'S') go to 900
      if (iPrint   .le. 0  ) go to 900

      nbnded = m - nfixed - nfree - nnorml
      write(iPrint, 2200)
      write(iPrint, 2300) ' Rows   ', m, nnorml, nfree, nfixed, nbnded

      nfixed = 0
      nfree  = 0
      nnorml = 0

      do 200, j  = 1, n
         b1     = bl(j)
         b2     = bu(j)
         if (b1 .eq. b2) then
             nfixed = nfixed + 1
         else
             if      (b1 .eq. zero  ) then
                if      (b2 .ge. bplus) then
                    nnorml = nnorml + 1
                end if
             else if (b1 .le. bminus) then
                if      (b2 .eq. zero ) then
                    nnorml = nnorml + 1
                else if (b2 .ge. bplus) then
                    nfree  = nfree  + 1
                end if
             end if
         end if
  200 continue
               
      nbnded = n - nfixed - nfree - nnorml
      write(iPrint, 2300) ' Columns', n, nnorml, nfree, nfixed, nbnded

*     Find the biggest and smallest elements in a, excluding free rows
*     and fixed columns.
*     Also find the largest objective coefficients in non-fixed columns.

      Aijmax = zero
      Aijmin = bplus
      cmax   = zero
      cmin   = bplus
      ncost  = 0
                          
      do 300, j = 1, n
         if (bl(j) .lt. bu(j)) then
            do 250, k = ka(j), ka(j+1) - 1
               i     = ha(k)
               if (hrtype(i) .eq. 2) then
                   if (i .eq. iObj) then
                       Aij   = abs( a(k) )
                       if (Aij .gt. zero) then
                           ncost  = ncost + 1
                           cmax   = max( cmax, Aij )
                           cmin   = min( cmin, Aij )
                       end if
                   end if
               else
                   Aij    = abs( a(k) )
                   Aijmax = max( Aijmax, Aij )
                   Aijmin = min( Aijmin, Aij )
               end if
  250       continue
         end if
  300 continue

      if (Aijmin .eq. bplus) Aijmin = zero
      Adnsty = 100.0d+0*ne / (m*n)
      write(iPrint, 2400) ne, Adnsty, Aijmax, Aijmin, ncost
      if (ncost .gt. 0) write(iPrint, 2410) cmax, cmin

*     Print a few things to be gathered as statistics
*     from a bunch of test runs.

      lvar  = n - nnL
      lcon  = m - nnCon
      write(iPrint, 2500) nnCon, lcon, nnL, lvar, nnJac, nnObj

  900 return

 2200 format(///
     $   ' Matrix statistics' /
     $   ' -----------------' /
     $   15x, 'Total', 6x, 'Normal', 8x, 'Free',  7x, 'Fixed',
     $    5x, 'Bounded')
 2300 format(a, 5i12)
 2400 format(/ ' No. of matrix elements', i21, 5x, 'Density', f12.3
     $       / ' Biggest ', 1p, e35.4, '  (excluding fixed columns,'
     $       / ' Smallest',     e35.4, '   free rows, and RHS)'
     $      // ' No. of objective coefficients', i14)
 2410 format(  ' Biggest ', 1p, e35.4, '  (excluding fixed columns)'
     $       / ' Smallest',     e35.4)
 2500 format(
     $ / ' Nonlinear constraints ', i7, 5x, 'Linear constraints ', i7,
     $ / ' Nonlinear variables   ', i7, 5x, 'Linear variables   ', i7,
     $ / ' Jacobian  variables   ', i7, 5x, 'Objective variables', i7)

*     end of s2Amat
      end           

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s2Aprd( job, tolz, ne, lenka, a, ha, ka, 
     $                   alpha, x, lenx, beta, y, leny )

      implicit           double precision (a-h,o-z)
      character          job*(*)
      integer            ha(ne)
      integer            ka(lenka)
      double precision   a(ne), x(lenx), y(leny)

*     ==================================================================
*     s2Aprd computes matrix-vector products involving A and x.  The 
*     variable job specifies the operation to be performed as follows:
*       job = 'N' (normal)          y := alpha*A *x + beta*y,
*       job = 'T' (transpose)       y := alpha*A'*x + beta*y,
*     where alpha and beta are scalars, x and y are vectors and A is a 
*     sparse matrix whose columns are in natural order.
*
*     23-Nov-91: First version implemented only with job = 'N'
*     07-Dec-92: job = 'T' implemented.
*     11-Dec-92: All rows are scanned, but
*                for job = 'N', only the first leny rows are used,
*                for job = 'T', only the first lenx rows are used,
*     11-Dec-92: Current version.
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
               y(i) = beta*y(i)
   20       continue
         end if
      end if

      if (alpha .eq. zero) then
*        Relax
      else if (alpha .eq. (-one)) then
         if (job(1:1) .eq. 'N') then
            do 110, j = 1, lenx
               xj     = x(j)
               if (abs( xj ) .gt. tolz) then
                  do 100, k = ka(j), ka(j+1)-1
                     i      = ha(k)
                     if (i .le. leny) y(i) = y(i) - a(k)*xj
  100             continue
               end if
  110       continue
         else
            do 210, j = 1, leny
               sum    = y(j)
               do 200, k = ka(j), ka(j+1)-1
                  i      = ha(k)
                  if (i .le. lenx) sum = sum - a(k)*x(i)
  200          continue
               y(j) = sum
  210       continue
         end if                          
      else 
         if (job(1:1) .eq. 'N') then
            do 310, j = 1, lenx
               alphxj = alpha*x(j)
               if (abs( alphxj ) .gt. tolz) then
                  do 300, k = ka(j), ka(j+1)-1
                     i      = ha(k)
                     if (i .le. leny) y(i) = y(i) + a(k)*alphxj
  300             continue
               end if
  310       continue
         else
            do 410, j = 1, leny
               sum    = zero
               do 400, k = ka(j), ka(j+1)-1
                  i      = ha(k)
                  if (i .le. lenx) sum = sum + a(k)*x(i)
  400          continue
               y(j) = y(j) + alpha*sum
  410       continue
         end if                          
      end if 

*     end of s2Aprd
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s2Bprd( job, tolz, n, lenkBS, kBS,
     $                   ne, nka, a, ha, ka,
     $                   alpha, x, lenx, beta, y, leny ) 

      implicit           double precision (a-h,o-z)
      character          job*(*)
      integer            ha(ne)
      integer            ka(nka), kBS(lenkBS)
      double precision   a(ne), x(lenx), y(leny)

*     ==================================================================
*     s2Bprd computes various matrix-vector products involving
*     B  and  S,  the basic and superbasic columns of  A. The variable
*     job specifies the operation to be performed as follows:
*         job = 'N'                 y := alpha*A *x + beta*y,
*         job = 'T'                 y := alpha*A'*x + beta*y,
*     where alpha and beta are scalars, x and y are vectors, and A is a 
*     sparse matrix whose columns are in the basic-superbasic order
*     A = ( B  S ).
*
*     23-Nov-91: First version of s2Bprd.
*     17-Jul-96: Standard implementation for slacks.
*     17-Jul-96: Current version.
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
*         Relax
      else if (job(1:1) .eq. 'N') then
         do 200, k = 1, lenx
            alphxj = alpha*x(k)
            if (abs(alphxj) .gt. tolz) then
               j   = kBS(k)
               if (j .le. n) then
*                 -------------------
*                 Column of A.
*                 -------------------
                  do 155, l = ka(j), ka(j+1)-1
                     i      = ha(l)
                     y(i)   = y(i) + a(l)*alphxj
  155             continue
               else
*                 --------------------
*                 Slack column.
*                 --------------------
                  i    = j    - n
                  y(i) = y(i) - alphxj
               end if
            end if
  200    continue
      else 
         do 400, k = 1, leny
            t      = zero
            j      = kBS(k)

            if (j .le. n) then
*              -------------------
*              Column of A.
*              -------------------
               do 350, l = ka(j), ka(j+1)-1
                  i      = ha(l)
                  t      = t + a(l)*x(i)
  350          continue
            else
*              -------------------
*              Slack column.
*              -------------------
               t   = - x(j-n )
            end if
            y(k) = y(k) + alpha*t
  400    continue
      end if

*     end of s2Bprd
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s2Binf( nb, bl, bu, xs, Binf, jBinf )

      implicit           double precision (a-h,o-z)
      double precision   bl(nb), bu(nb), xs(nb)

*     ==================================================================
*     s2Binf  computes the maximum infeasibility with respect to
*     the bounds on xs.
*     s2Binf  is called by s5savB and s8savB before and after unscaling.
*
*     On exit,
*      Binf  is the maximum bound infeasibility.
*     jBinf  is the corresponding variable.
*
*     05-Apr-96: First version based on Minos routine m2Binf.
*     ==================================================================
      parameter        ( zero = 0.0d+0 )
*     ------------------------------------------------------------------
      jBinf = 0
      Binf  = zero
      
      do 500, j = 1, nb
         d1     = bl(j) - xs(j)
         d2     = xs(j) - bu(j)
         if (Binf .lt. d1) then
             Binf   =  d1
             jBinf  =  j
         end if
         if (Binf .lt. d2) then
             Binf   =  d2
             jBinf  =  j
         end if
  500 continue

*     end of s2Binf
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s2Dinf( n, nb, iObj, bl, bu, rc, xs, Dinf, jDinf )

      implicit           double precision (a-h,o-z)
      double precision   bl(nb), bu(nb), rc(nb), xs(nb)

*     ==================================================================
*     s2Dinf  computes the maximum dual infeasibility.
*     s2Dinf  is called by s5savB and s8savB before and after unscaling.
*     
*
*     On exit,
*      Dinf  is the maximum dual infeasibility.
*     jDinf  is the corresponding variable.
*
*     05-Apr-96: First version based on Minos routine m2Dinf.
*     ==================================================================
      parameter         (zero = 0.0d+0)
*     ------------------------------------------------------------------
      jObj = n + iObj

      if (iObj .gt. 0) then
         blObj    = bl(jObj) 
         bl(jObj) = bu(jObj)
      end if

      jDinf = 0
      Dinf  = zero
      
      do 500, j = 1, nb
         if (bl(j) .lt. bu(j)) then
            dj  = rc(j)
            d1  = bl(j) - xs(j)
            d2  = xs(j) - bu(j)
            dj  = rc(j)
            if      (d1 .ge. zero) then
               dj  = - dj
            else if (d2 .ge. zero) then
*              dj  = + dj
            else
               dj  = abs( dj )
            end if
            
            if (Dinf .lt. dj) then
                Dinf   =  dj
                jDinf  =  j
            end if
         end if
  500 continue

      if (iObj .gt. 0) then
         bl(jObj) = blObj
      end if

*     end of s2Dinf
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s2crsh( lCrash, lPrint, m, n, nb,
     $                   iCrash, tCrash,
     $                   ne, nka, a, ha, ka,
     $                   hpiv, hs, hrtype,
     $                   bl, bu, xs,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      integer            ha(ne), hpiv(m), hs(nb), hrtype(m)
      integer            ka(nka)
      integer            iw(leniw)
      double precision   rw(lenrw)
      double precision   a(ne), bl(nb), bu(nb), xs(nb)

*     ==================================================================
*     s2crsh  looks for a basis in the columns of ( A  -I ).
*
*     ON ENTRY
*
*     iCrash    = the Crash option has been used by s5getB
*                 to set lCrash.
*     tCrash    = the Crash tolerance.  Default = 0.1
*
*     lCrash      specifies the action to be taken by Crash.
*        0,1,2,3  The call is from s4getB.
*        4,5      The call is from s8solv.
*
*        0        The all-slack basis is set up.
*
*        1        A triangular Crash is applied to the columns of A.
*                 hs(1:n) is used to help select columns.
*                 tCrash is used to ignore small entries in each column.
*                 Depending on the size of tCrash, the resulting basis
*                 will be nearly (but not strictly) lower triangular.
*
*        2        As for 1, but nonlinear rows are ignored.
*
*        3        As for 2, but linear LG rows are also ignored.
*
*        4        Linear LG rows are now included.
*                 All hs(1:nb) and xs(n+i) are defined.
*                 Slack values of xs(n+i) are used to select LG rows.
*
*        5        Nonlinear rows are now included.
*
*     hrtype(*)   should be defined as described in s2amat:
*     hrtype(i) = 0  for E rows      (equalities)
*     hrtype(i) = 1  for L or G rows (inequalities)
*     hrtype(i) = 2  for N rows      (objective or free rows)
*
*     xs          If lCrash <= 4, xs(1:n) is used to initialize
*                 slacks as xs(n+1:nb) = A*x.
*                 Used to select slacks from LG rows to be in B (basis).
*                 If lCrash  = 5, xs(n+1:n+nnCon) contains slack values
*                 evaluated from xs(1:n) and fcon(*).
*                 Used to select slacks from nonlinear rows to be in B.
*
*     hs          If lCrash = 1, 2 or 3, hs(1:n)  is used.
*                 If lCrash =    4 or 5, hs(1:nb) is used.
*                 If hs(j) =  0, 1 or 3, column j is eligible for B,
*                                        with 3 being "preferred".
*                 If hs(j) =  2, 4 or 5, column j is ignored.
*
*
*     Crash has several stages.
*
*     Stage 1: Insert any slacks (N, L or G rows, hrtype = 1 or 2).
*
*     Stage 2: Do triangular Crash on any free columns (wide bounds)
*
*     Stage 3: Do triangular Crash on "preferred" columns (hs(j) < 0).
*              For the linear Crash, this includes variables set
*              between their bounds in the MPS file via FR INITIAL.
*              For the nonlinear Crash, it includes nonbasics
*              between their bounds.
*              (That is, "pegged" variables in both cases.)
*
*     Stage 4: Grab unit columns.
*
*     Stage 5: Grab double columns.
*
*     Stage 6: Do triangular Crash on all columns.
*
*     Slacks are then used to pad the basis.
*
*
*     ON EXIT
*
*     hs          is set to denote an initial (B S N) partition.
*                 hs(j) = 3 denotes variables for the initial basis.
*                 If hs(j) = 2 still, variable j will be superbasic.
*                 If hs(j) = 4 or 5 still, it will be changed to 0 or 1
*                 by s4chek and variable j will be nonbasic.
*
*     xs          If lCrash <= 4, slacks xs(n+1:nb) are initialized.
*
*     ------------------------------------------------------------------
*        Nov 1986: Essentially the same as in 1976.
*                  Crash tolerance added.
*                  Attention paid to various hs values.
*
*     12 Nov 1988: After free rows and columns have been processed
*                  (stage 1 and 2), slacks on L or G rows are inserted
*                  if their rows have not yet been assigned.
*
*     28 Dec 1988: Refined as follows.
*                  Stage 1 inserts free and preferred rows (slacks).
*                  Stage 2 performs a triangular Crash on free or
*                          preferred columns, ignoring free rows.
*                          Unpivoted L or G slacks are then inserted.
*                  Stage 3 performs a triangular Crash on the other
*                          columns, ignoring rows whose slack is basic.
*                          (Such rows form the top part of U.  The
*                          remaining rows form the beginning of L.)
*
*     30 Apr 1989: Stage 1 now also looks for singleton columns
*                  (ignoring free and preferred rows).
*     05 May 1989: Stage 2 doesn't insert slacks if Crash option < 0.
*
*     06 Dec 1989: Stage 2, 3, 4 modified.  Columns of length 2 are
*                  now treated specially.
*
*     20 Dec 1989: Stage 2 thru 5 modified.  Free columns done before
*                  unit and double columns.
*
*     19 May 1992: x now used to help initialize slacks.
*                  Stage 1 thru 7 redefined as above.
*
*     01 Jun 1992: abs used to define closeness of slacks to bounds.
*                  Unfortunately, x(1:n) seldom has meaningful values.
*
*     02 Jun 1992: Poor performance on the larger problems.
*                  Reverted to simple approach: all slacks grabbed.
*
*     04 Jun 1992: Compromise -- Crash 3 now has 3 phases:
*                  (a) E rows.
*                  (b) LG rows.
*                  (c) Nonlinear rows.
*                  x(1:n) should then define the slack values better
*                  for (b) and (c).
*     17 Jul 1996: Standard implementation for slacks.
*     ==================================================================
      double precision   apiv(2)
      integer            ipiv(2)
      parameter         (nstage = 6) 
      integer            num(nstage), stage
      logical            free, prefer, gotslk,
     $                   stage2, stage3, stage4, stage5
      logical            prnt10
      parameter         (zero  = 0.0d+0, one = 1.0d+0)
      parameter         (small = 1.0d-3, big = 1.0d+4)
*     ------------------------------------------------------------------
      iPrint    = iw( 12)
      nnCon     = iw( 21)
      eps0      = rw(  2)

      prnt10    = iPrint .gt. 0  .and.  lPrint .ge. 10

      if ( prnt10 ) then
         if (lCrash .le. 3) write(iPrint, 1000) iCrash
         if (lCrash .eq. 3  .and.  nnCon .lt. m) write(iPrint, 1030)
         if (lCrash .eq. 4) write(iPrint, 1040)
         if (lCrash .eq. 5) write(iPrint, 1050)
      end if

      if (lCrash .le. 4) then

*        Sets slacks xs(n+1:nb) = A*x.
*        This is where the slacks are initialized.
*        They may be altered later (see the end of Crash).

         call s2Aprd( 'No transpose', eps0, ne, nka, a, ha, ka, 
     $                   one, xs, n, zero, xs(n+1), m )
      end if

*     ------------------------------------------------------------------
*     For Crash option 0, set hs(j) = 3 for all slacks and quit.
*     ------------------------------------------------------------------
      if (lCrash .eq. 0) then
         call iload ( n, 0, hs     , 1 )
         call iload ( m, 3, hs(n+1), 1 )
         go to 900
      end if

*     ------------------------------------------------------------------
*     Crash option 1, 2 or 3.   lCrash = 1, 2, 3, 4, or 5.
*     tolslk measures closeness of slacks to bounds.
*     i1,i2  are the first and last rows of A involved in Stage 1.
*     ------------------------------------------------------------------
*-->  tolslk = 0.25
      tolslk = 1.0d-2
      call iload ( nstage, 0, num, 1 )

      if (lCrash .le. 3) then
*        ---------------------------------------------------------------
*        First call.   lCrash = 1, 2 or 3.
*        Initialize hpiv(*) for all rows and hs(*) for all slacks.
*        ---------------------------------------------------------------
         i1     = 1
         if (lCrash .ge. 2) i1 = nnCon + 1
         i2     = m
         nrows  = i2 - i1 + 1

*        Make sure there are no basic columns already (hs(j) = 3).
*        If there are, make them "preferred".

         do 20, j = 1, n
            if (hs(j) .eq. 3) hs(j) = -1
   20    continue

*        Make relevant rows available:  hpiv(i) = 1, hs(n+i) = 0.

         if (nrows .gt. 0) then
            call iload ( nrows, 1, hpiv(i1), 1 )
            call iload ( nrows, 0, hs(n+i1), 1 )
         end if

         if (lCrash .eq. 1) then
            nbasic = 0
         else

*           lCrash = 2 or 3:  Insert nonlinear slacks.

            nbasic = nnCon
            if (nnCon .gt. 0) then
               call iload ( nnCon, 3, hpiv   , 1 )
               call iload ( nnCon, 3, hs(n+1), 1 )
            end if
         end if

         if (lCrash .eq. 3) then

*           Insert linear inequality slacks (including free rows).

            do 25, i = i1, m
               if (hrtype(i) .ge. 1) then
                  nbasic  = nbasic + 1
                  nrows   = nrows  - 1
                  hpiv(i) = 3
                  hs(n+i) = 3
               end if
   25       continue
         end if

*        We're done if there are no relevant rows.

         if (nrows .eq. 0) go to 800

      else
*        ---------------------------------------------------------------
*        Second or third call.  lCrash = 4 or 5.
*        Initialize hpiv(*) for all rows.
*        hs(*) already defines a basis for the full problem,
*        but we want to do better by including only some of the slacks.
*        ---------------------------------------------------------------
         if (lCrash .eq. 4) then
*           ------------------------------------------------------------
*           Crash on linear LG rows.
*           ------------------------------------------------------------
            if (nnCon .eq. m) go to 900
            i1     = nnCon + 1
            i2     = m

*           Mark nonlinear rows as pivoted: hpiv(i) = 3.

            nbasic = nnCon
            if (nbasic .gt. 0) then
               call iload ( nbasic, 3, hpiv, 1 )
            end if

*           Mark linear E  rows as pivoted: hpiv(i) = 3
*           Make linear LG rows available:  hpiv(i) = 1, hs(n+i) = 0.

            do 30, i = i1, m
               if (hrtype(i) .eq. 0) then
                  nbasic  = nbasic + 1
                  hpiv(i) = 3
               else
                  hpiv(i) = 1
                  hs(n+i) = 0
               end if
   30       continue

*           Mark linear LG rows with hpiv(i) = 2
*           if any basic columns contain a nonzero in row i.

            do 50, j = 1, n
               if (hs(j) .eq. 3) then
                  do 40, k = ka(j), ka(j+1) - 1
                     i     = ha(k)
                     if (hrtype(i) .eq. 1) then
                        if (i .gt. nnCon) then
                           if (a(k) .ne. zero) hpiv(i) = 2
                        end if
                     end if
   40             continue
               end if
   50       continue

         else
*           ------------------------------------------------------------
*           lCrash = 5.  Crash on nonlinear rows.
*           ------------------------------------------------------------
            i1     = 1
            i2     = nnCon

*           Mark all linear rows as pivoted: hpiv(i) = 3

            nbasic = m - nnCon
            if (nbasic .gt. 0) then
               call iload ( nbasic, 3, hpiv(nnCon+1), 1 )
            end if

*           Make nonlinear rows available:  hpiv(i) = 1, hs(n+i) = 0.

            call iload ( nnCon, 1, hpiv   , 1 )
            call iload ( nnCon, 0, hs(n+1), 1 )

*           Mark nonlinear rows with hpiv(i) = 2
*           if any basic columns contain a nonzero in row i.

            do 70, j = 1, n
               if (hs(j) .eq. 3) then
                  do 60, k = ka(j), ka(j+1) - 1
                     i     = ha(k)
                     if (i .le. nnCon) then
                        if (a(k) .ne. zero) hpiv(i) = 2
                     end if
   60             continue
               end if
   70       continue
         end if
      end if

*     ------------------------------------------------------------------
*     Stage 1: Insert relevant slacks (N, L or G rows, hrtype = 1 or 2).
*              If lCrash = 4 or 5, grab them only if they are more than
*              tolslk from their bound.
*     ------------------------------------------------------------------
      stage  = 1
      gotslk = lCrash .eq. 4  .or.  lCrash .eq. 5

      do 100, i = i1, i2
         j      = n + i
         if (hs(j) .le. 1  .and.  hrtype(i) .gt. 0) then
            if (gotslk) then
               d1 = xs(j) - bl(j)
               d2 = bu(j) - xs(j)
               if (min( d1, d2 ) .le. tolslk) then
              
*                 The slack is close to a bound or infeasible.
*                 Move it exactly onto the bound.
              
                  if (d1 .le. d2) then
                     xs(j) = bl(j)
                     hs(j) = 0
                  else
                     xs(j) = bu(j)
                     hs(j) = 1
                  end if
                  go to 100
               end if
            end if

            nbasic     = nbasic     + 1
            num(stage) = num(stage) + 1
            hpiv(i)    = 3
            hs(j)      = 3
         end if
  100 continue

      if (nbasic .eq. m) go to 700

*     ------------------------------------------------------------------
*     Apply a triangular Crash to various subsets of the columns of A.
*
*        hpiv(i) = 1  if row i is unmarked (initial state).
*        hpiv(i) = 3  if row i has been given a pivot
*                     in one of a set of triangular columns.
*        hpiv(i) = 2  if one of the triangular columns contains
*                     a nonzero in row i below the triangle.
*     ------------------------------------------------------------------

      do 600, stage = 2, nstage
         stage2     = stage .eq. 2
         stage3     = stage .eq. 3
         stage4     = stage .eq. 4
         stage5     = stage .eq. 5

*        ---------------------------------------------------------------
*        Main loop for triangular Crash.
*        ---------------------------------------------------------------
         do 200,  j = 1, n
            js      = hs(j)
            if (js    .gt.    1 ) go to 200
            if (bl(j) .eq. bu(j)) go to 200

            if ( stage2 ) then
               free   = bl(j) .le. - big  .and.  bu(j) .ge. big
               if ( .not. free  ) go to 200

            else if ( stage3 ) then
               prefer = js .lt. 0
               if ( .not. prefer) go to 200
            end if

*           Find the biggest aij, ignoring free rows.

            k1     = ka(j)
            k2     = ka(j+1) - 1
            aimax  = zero

            do 130, k = k1, k2
               i      = ha(k)
               if (hrtype(i) .ne. 2) then
                  ai    = a(k)
                  aimax = max( aimax, abs( ai ) )
               end if
  130       continue

*           Prevent small pivots if Crash tol is too small.

            if (aimax .le. small) go to 200

*           Find the biggest pivots in rows that are still
*           unpivoted and unmarked.  Ignore smallish elements.
*           nz counts the number of relevant nonzeros.

            aitol   = aimax * tCrash
            nz      = 0
            npiv    = 0
            ipiv(1) = 0
            ipiv(2) = 0
            apiv(1) = zero
            apiv(2) = zero

            do 150, k = k1, k2
               i      = ha(k)
               if (hs(n+i) .ne. 3) then
                  ai = abs( a(k) )
                  if (ai .gt. aitol) then
                     nz = nz + 1
                     ip = hpiv(i)
                     if (ip .le. 2) then
                        if (apiv(ip) .lt. ai) then
                            apiv(ip) = ai
                            ipiv(ip) = i
                        end if
                     else
                        npiv = npiv + 1
                     end if
                  end if
               end if
  150       continue

*           Grab unit or double columns.

            if      ( stage4 ) then
               if (nz .ne. 1) go to 200
            else if ( stage5 ) then
               if (nz .ne. 2) go to 200
            end if

*           See if the column contained a potential pivot.
*           An unmarked row is favored over a marked row.

            ip     = 1
            if (ipiv(1)  .eq.  0  .and.  npiv .eq. 0) ip = 2
            i      = ipiv(ip)

            if (i .gt. 0) then
               nbasic     = nbasic     + 1
               num(stage) = num(stage) + 1
               hpiv(i)    = 3
               hs(j)      = 3
               if (nbasic .ge. m) go to 700

*              Mark off all relevant unmarked rows.

               do 180 k = k1, k2
                  i     = ha(k)
                  if (hs(n+i) .ne. 3) then
                     ai = abs( a(k) )
                     if (ai .gt. aitol) then
                        if (hpiv(i) .eq. 1) hpiv(i) = 2
                     end if
                  end if
  180          continue
            end if
  200    continue
  600 continue

*     ------------------------------------------------------------------
*     All stages finished.
*     Fill remaining gaps with slacks.
*     ------------------------------------------------------------------
  700 npad   = m - nbasic
      if ( prnt10 ) write(iPrint, 1200) num, npad

      if (npad .gt. 0) then
         do 720, i = 1, m
            if (hpiv(i) .lt. 3) then
               nbasic  = nbasic + 1
               hs(n+i) = 3
               if (nbasic .ge. m) go to 800
            end if
  720    continue
      end if

*     ------------------------------------------------------------------
*     Make sure there aren't lots of nonbasic slacks floating off
*     their bounds.  They could take lots of iterations to move.
*     ------------------------------------------------------------------
  800 do 850, i = i1, i2
         j      = n + i
         if (hs(j) .le. 1  .and.  hrtype(i) .gt. 0) then
            d1 = xs(j) - bl(j)
            d2 = bu(j) - xs(j)
            if (min( d1, d2 ) .le. tolslk) then
              
*              The slack is close to a bound or infeasible.
*              Move it exactly onto the bound.
              
               if (d1 .le. d2) then
                  xs(j) = bl(j)
                  hs(j) = 0
               else
                  xs(j) = bu(j)
                  hs(j) = 1
               end if
            end if
         end if
  850 continue

  900 return

 1000 format(// ' Crash option', i3)
 1030 format(/  ' Crash on linear E  rows:')
 1040 format(/  ' Crash on linear LG rows:')
 1050 format(/  ' Crash on nonlinear rows:')
 1200 format(
     $   ' Slacks', i6, '  Free cols', i6, '  Preferred', i6
     $ / ' Unit  ', i6, '  Double   ', i6, '  Triangle ', i6,
     $     '  Pad', i6)

*     end of s2crsh
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s2rcA ( feasbl, featol, iObj, minimz, wtInf,
     $                   m, n, nb, nnObj,
     $                   ne, nka, a, ha, ka,
     $                   hEstat, hs, bl, bu, gObj, pi, rc, xs )

      implicit           double precision (a-h,o-z)
      logical            feasbl
      integer            ha(ne), hEstat(nb), hs(nb)
      integer            ka(nka)
      double precision   a(ne), bl(nb), bu(nb)
      double precision   gObj(*), pi(m), rc(nb), xs(nb)

*     ==================================================================
*     s2rcA  computes reduced costs rc(*) for all columns of ( A  -I ).
*     If xs is feasible, the true nonlinear objective gradient gObj(*)
*     is used.  Otherwise, the Phase-1 objective is included.
*
*     s2rcA  is called by s4savb BEFORE unscaling.
*     External values of hs(*) are used (0, 1, 2, 3),
*     but internal would be ok too since we only test for > 1.
*
*     19-Feb-94: First version based on Minos routine m4rc.
*     17-Jul-96: Standard implementation for slacks.
*     17-Jul-97: Current version.
*     ==================================================================
      parameter        ( zero = 0.0d+0,  one = 1.0d+0 )
*     ------------------------------------------------------------------
      do 300, j = 1, n
         dj     = zero
         do 200, k = ka(j), ka(j+1) - 1
            i      = ha(k)
            dj     = dj  +  pi(i) * a(k)
  200    continue
         rc(j) = - dj
  300 continue

      call dcopy ( m, pi, 1, rc(n+1), 1 )

      if ( feasbl ) then

*        Include the nonlinear objective gradient.
*        Include the gradient of the linear term.

         jObj   = n + iObj
         sgnObj = minimz

         if (nnObj .gt. 0) then
            call daxpy ( nnObj, sgnObj, gObj, 1, rc, 1 )
         end if
         if (iObj .gt. 0) rc(jObj) =  rc(jObj) + sgnObj
      else

*        Include the Phase 1 objective.
*        Only basics and superbasics can be infeasible.

         do 500, j = 1, nb
            if (hs(j) .gt. 1) then
               d1  = bl(j) - xs(j)
               d2  = xs(j) - bu(j)
               if (hEstat(j) .eq. 0) then
                  if (d1 .gt. featol) rc(j) = rc(j) - one
                  if (d2 .gt. featol) rc(j) = rc(j) + one
               else
                  if (d1 .gt. featol) rc(j) = rc(j) - wtInf
                  if (d2 .gt. featol) rc(j) = rc(j) + wtInf
               end if
            end if
  500    continue
      end if

*     end of s2rcA
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s2scal( lPrint, m, n, nb, nnL, nnCon, nnJac, hrtype, 
     $                   ne, nka, a, ha, ka, 
     $                   aScale, bl, bu, rmin, rmax,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      integer            hrtype(m), ha(ne)
      integer            ka(nka)
      integer            iw(leniw)
      double precision   rw(lenrw)
      double precision   a(ne), aScale(nb), bl(nb), bu(nb)
      double precision   rmin(m), rmax(m)

*     ==================================================================
*     s2Scal computes scale factors aScale from A, bl, bu.
*
*     In phase 1, an iterative procedure based on geometric means is
*     used to compute scales from a alone.  This procedure is derived
*     from a routine written by Robert Fourer, 1979.  The main steps
*     are:
*
*        (1) Compute Aratio = max(i1,i2,j)  A(i1,j) / A(i2,j).
*        (2) Divide each row i by
*               ( min(j) A(i,j) * max(j) A(i,j) ) ** 1/2.
*        (3) Divide each column j by
*               ( min(i) A(i,j) * max(i) A(i,j) ) ** 1/2.
*        (4) Compute sratio as in (1).
*        (5) If sratio .lt. scltol * Aratio, repeat from step (1).
*
*        Free rows (hrtype=2) and fixed columns (bl=bu) are not used
*        at this stage.
*
*     In phase 2, the scales for free rows are set to be their largest
*     element.
*
*     In phase 3, fixed columns are summed in order to compute
*     a scale factor sigma that allows for the effective rhs of the
*     constraints.  All scales are then multiplied by sigma.
*
*
*     If lvlScl = 1, the first nnCon rows and the first nnL columns will
*     retain scales of 1.0 during phases 1-2, and phase 3 will not be
*     performed.  (This takes effect if the problem is nonlinear but
*     the user has specified 'scale linear variables' only.)
*     However, all rows    contribute to the linear column scales,
*     and      all columns contribute to the linear row    scales.
*
*     If lvlScl = 2, all rows and columns are scaled.  To guard against
*     misleadingly small Jacobians, if the maximum element in any of
*     the first nnCon rows or the first nnJac columns is less than
*     smallj, the corresponding row or column retains a scale of 1.0.
*
*     15-Nov-91: First version based on Minos 5.4 routine m2scal.
*     09-Nov-92: Current version.
*     ==================================================================
      double precision   Ac, Ar, Amin, Amax, Aratio, bnd, b1, b2,
     $                   cmin, cmax, cratio, damp, sigma,
     $                   small, smallj, sratio
      logical            lonly, Prnt1
      parameter        ( zero = 0.0d+0,  one    = 1.0d+0,
     $                   damp = 1.0d-4,  smallj = 1.0d-2 )
*     ------------------------------------------------------------------
      iPrint    = iw( 12)
      lvlScl    = iw( 75)
      lprScl    = iw( 83)

      tolx      = rw( 56)
      plInfy    = rw( 70)
      scltol    = rw( 92)

      Prnt1     = iPrint .gt.  0  .and.  lPrint .ge. 1
      if ( Prnt1 ) write(iPrint, 1000)
      bplus     = 0.1d+0*plInfy
      Aratio    = bplus
      maxk      = 11
      istart    = 1
      jstart    = 1
      lonly     = lvlScl .eq. 1
      if (lonly) then
         istart = nnCon + 1
         jstart = nnL   + 1
      end if

      do 50, j = 1, nb
         aScale(j) = one
   50 continue

      if (jstart .gt. n) return

*     ------------------------------------------------------------------
*     Main loop for phase 1.
*     Only the following row-types are used:
*        hrtype(i) = 2       for type N rows (objective or free rows),
*        hrtype(i) = 0 or 1  otherwise.
*     ------------------------------------------------------------------

      do 400, kk = 1, maxk

*        Find the largest column ratio.
*        Also set new column scales (except on pass 0).

         nPass   = kk - 1
         Amin    = bplus
         Amax    = zero
         small   = smallj
         sratio  = one

         do 250, j = jstart, n
            if (bl(j) .lt. bu(j)) then
               cmin   = bplus
               cmax   = zero

               do 230, k = ka(j), ka(j+1) - 1
                  i      = ha(k)
                  if (hrtype(i) .ne. 2) then
                     Ar     = abs( a(k) )

                     if (Ar .gt. zero) then
                        Ar   = Ar / aScale(n+i)
                        cmin = min( cmin, Ar )
                        cmax = max( cmax, Ar )
                     end if
                  end if
  230          continue

               Ac     = max( cmin, damp*cmax )
               Ac     = sqrt( Ac ) * sqrt( cmax )
               if (j     .gt. nnJac) small     = zero
               if (cmax  .le. small) Ac        = one
               if (nPass .gt. 0    ) aScale(j) = Ac
               Amin   = min( Amin, cmin / aScale(j) )
               Amax   = max( Amax, cmax / aScale(j) )
               cratio = cmax / cmin
               sratio = max( sratio, cratio )
            end if
  250    continue

         if ( Prnt1 ) then
            write(iPrint, 1200) nPass, Amin, Amax, sratio
         end if
         if (nPass  .ge. 3       .and.
     $       sratio .ge. Aratio*scltol) go to 420
         if (kk     .eq. maxk         ) go to 420
         Aratio = sratio

*        Set new row scales for the next pass.

         if (istart .gt. m) go to 400

         do 300, i  = istart, m
            rmin(i) = bplus
            rmax(i) = zero
  300    continue

         do 350, j = 1, n
            if (bl(j) .lt. bu(j)) then
               Ac  = aScale(j)

               do 330, k = ka(j), ka(j+1) - 1
                  i      = ha(k)

                  if (i  .ge. istart) then
                     Ar  = abs( a(k) )

                     if (Ar .gt. zero) then
                        Ar      = Ar / Ac
                        rmin(i) = min( rmin(i), Ar )
                        rmax(i) = max( rmax(i), Ar )
                     end if
                  end if
  330          continue
            end if
  350    continue

         do 360, i = istart, m
            j      = n + i
            Ar     = rmax(i)

            if (i .le. nnCon  .and.  Ar .le. smallj) then
               aScale(j) = one
            else
               Ac        = max( rmin(i), damp*Ar )
               aScale(j) = sqrt( Ac ) * sqrt( Ar )
            end if
  360    continue
  400 continue

*     ------------------------------------------------------------------
*     End of main loop.
*     ------------------------------------------------------------------

*     Invert the column scales, so that structurals and logicals
*     can be treated the same way during subsequent unscaling.
*     Find the min and max column scales while we're at it.
*     Nov 1989: nclose counts how many are "close" to 1.
*     For problems that are already well-scaled, it seemed sensible to
*     set the "close" ones exactly equal to 1.
*     Tried "close" = (0.5,2.0) and (0.9,1.1), but they helped only
*     occasionally.  Have decided not to interfere.

  420 Amin   = bplus
      Amax   = zero
      close1 = 0.5d+0
      close2 = 2.0d+0
      nclose = 0
      

      do 430, j = 1, n
         Ac     = one / aScale(j)

         if (Amin .gt. Ac) then
             Amin   =  Ac
             jmin   =  j
         end if
         if (Amax .lt. Ac) then
             Amax   =  Ac
             jmax   =  j
         end if

         if (Ac .gt. close1  .and.  Ac .lt. close2) then
             nclose =  nclose + 1
*----        Ac     =  one
         end if

         aScale(j)  = Ac
  430 continue

*     Remember, column scales are upside down.

      Amax   = one / Amax
      Amin   = one / Amin
      if ( Prnt1 ) then
         write(iPrint, 1300)
         write(iPrint, 1310) 'Col', jmax, Amax, 'Col', jmin, Amin,
     $                       nclose, nclose * 100.0 / n
      end if

*     ------------------------------------------------------------------
*     Phase 2.  Deal with empty rows and free rows.
*     Find the min and max row scales while we're at it.
*     ------------------------------------------------------------------
      Amin   = bplus
      Amax   = zero
      imin   = 0
      imax   = 0
      nclose = 0

      do 440, i = istart, m
         j      = n + i
         Ar     = aScale(j)
         if (hrtype(i) .eq. 2) Ar = rmax(i)
         if (Ar   .eq.   zero) Ar = one
         if (hrtype(i) .eq. 2) go to 435

         if (Amin .gt. Ar) then
             Amin  =   Ar
             imin  =   i
         end if
         if (Amax .lt. Ar) then
             Amax  =   Ar
             imax  =   i
         end if

         if (Ar .gt. close1  .and.  Ar .lt. close2) then
             nclose =  nclose + 1
*----        Ar     =  one
         end if

  435    aScale(j) = Ar
  440 continue

      if (imin .eq. 0) then
          Amin   = zero
          Amax   = zero
      end if
      if ( Prnt1 ) then
         write(iPrint, 1310) 'Row', imin, Amin, 'Row', imax, Amax,
     $                       nclose, nclose * 100.0 / m
      end if

*     ------------------------------------------------------------------
*     Phase 3.
*     Compute what is effectively the rhs for the constraints.
*     We set  rmax  =  ( A  -I )*x  for fixed columns and slacks,
*     including positive lower bounds and negative upper bounds.
*     ------------------------------------------------------------------
      if (lonly) go to 700
      call dload ( m, zero, rmax, 1 )

      do 500, j = 1, nb
         bnd    = zero
         b1     = bl(j)
         b2     = bu(j)
         if (b1  .eq. b2  ) bnd = b1
         if (b1  .gt. zero) bnd = b1
         if (b2  .lt. zero) bnd = b2
         if (bnd .eq. zero) go to 500

         if (j   .le. n   ) then
            do 480,  k = ka(j), ka(j+1) - 1
               i       = ha(k)
               rmax(i) = rmax(i)  +  a(k) * bnd
  480       continue
         else
*           Free slacks never get here, so we don't have to skip them.
            i       = j - n
            rmax(i) = rmax(i)  -  bnd
         end if
  500 continue

*     We don't want nonzeros in free rows to interfere.

      do 520, i = 1, m
         if (hrtype(i) .eq. 2) rmax(i) = zero
  520 continue

*     Scale rmax = rmax / (row scales),  and use its norm sigma
*     to adjust both row and column scales.

      Ac     = dnrm1s( m, rmax, 1 )
      call dddiv ( m, aScale(n+1), 1, rmax, 1 )
      sigma  = dnrm1s( m, rmax, 1 )
      if ( Prnt1 ) write(iPrint, 1400) Ac, sigma
      sigma  = max( sigma, one )
      call dscal ( nb, sigma, aScale, 1 )

*     Big scales might lead to excessive infeasibility when the
*     problem is unscaled.  If any are too big, scale them down.

      Amax  = zero
      do 540, j = 1, n
         Amax   = max( Amax, aScale(j) )
  540 continue

      do 550, i = 1, m
         if (hrtype(i) .eq. 2) go to 550
         Amax   = max( Amax, aScale(n + i) )
  550 continue

      big    = 0.1d+0 / tolx
      sigma  = big    / Amax
      if (sigma .lt. one) then
         call dscal ( nb, sigma, aScale, 1 )
         if ( Prnt1 ) write(iPrint, 1410) sigma
      end if
      
  700 if (lprScl .eq.  0) return
      if (iPrint .gt. 0) then
        if (istart .le. m) write(iPrint,1500) (i,aScale(n+i),i=istart,m)
        if (jstart .le. n) write(iPrint,1600) (j,aScale(j  ),j=jstart,n)
      end if
      return

 1000 format(// ' Scaling' / ' -------'
     $        / '             Min elem    Max elem       Max col ratio')
 1200 format(   ' After', i3, 1p, e12.2, e12.2, 0p, f20.2)
 1300 format(/  12x, 'Min scale', 23x, 'Max scale', 6x,
     $          'Between 0.5 and 2.0')
 1310 format(1x, a, i7, 1p, e10.1, 12x, a, i7, e10.1, i17, 0p, f8.1)
 1400 format(/  ' Norm of fixed columns and slacks', 1p, e20.1
     $       /  ' (before and after row scaling)  ', 1p, e20.1)
 1410 format(   ' Scales are large --- reduced by ', 1p, e20.1)
 1500 format(// ' Row scales  r(i)',
     $      8x, ' a(i,j)  =   r(i)  *  scaled a(i,j)  /  c(j)'
     $        / ' ----------------'    // 5(i6, g16.5))
 1600 format(// ' Column scales  c(j)',
     $      5x, ' x(j)    =   c(j)  *  scaled x(j)'
     $        / ' -------------------' // 5(i6, g16.5))

*     end of s2Scal
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s2Scla( job, m, n, nb,
     $                   iObj, plInfy, sclObj,
     $                   ne, nka, a, ha, ka,
     $                   aScale, bl, bu, pi, xs )

      implicit           double precision (a-h,o-z)
      character          job*(*)
      integer            ha(ne)
      integer            ka(nka)
      double precision   a(ne), aScale(nb), bl(nb), bu(nb)
      double precision   pi(m), xs(nb)

*     ==================================================================
*     s2Scla scales or unscales A, bl, bu, pi, xs using aScale.
*
*     15-Nov-91: First version based on Minos 5.4 routine m2scla.
*     21-Jul-97: Current version of s2Scla
*     ==================================================================

      bplus  = 0.1d+0*plInfy
      if (job(1:1) .eq. 'S') then
*        ---------------------------------------------------------------
*        job = 'S'cale  ---  scale A, bl, bu, xs and pi.
*        ---------------------------------------------------------------
         do 150, j = 1, nb
            scale = aScale(j)
            if (j .le. n) then
               do 110, k = ka(j), ka(j+1)-1
                  i      = ha(k)
                  a(k)   = a(k) * ( scale / aScale(n+i) )
  110          continue
            end if
            xs(j) = xs(j) / scale
            if (bl(j) .gt. -bplus) bl(j) = bl(j) / scale
            if (bu(j) .lt.  bplus) bu(j) = bu(j) / scale
  150    continue
      
         call ddscl ( m, aScale(n+1), 1, pi, 1 )
         if (iObj  .gt. 0) sclObj = aScale(n+iObj)
      else
*        ---------------------------------------------------------------
*        job = 'U'nscale  ---  unscale everything.
*        ---------------------------------------------------------------
         do 250, j = 1, nb
            scale  = aScale(j)
            if (j .le. n) then
               do 210, k = ka(j), ka(j+1)-1
                  i      = ha(k)
                  a(k)   = a(k) * ( aScale(n+i) / scale )
  210          continue
            end if
            xs(j) = xs(j) * scale
            if (bl(j) .gt. -bplus)  bl(j) = bl(j) * scale
            if (bu(j) .lt.  bplus)  bu(j) = bu(j) * scale
  250    continue
      
         call dddiv ( m, aScale(n+1), 1, pi, 1 )
         sclObj = 1.0d+0
      end if

*     end of s2Scla
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s2unpk( jq, m, n, ne, nka, a, ha, ka, y )

      implicit           double precision (a-h,o-z)
      double precision   a(ne)
      integer            ha(ne)
      integer            ka(nka)
      double precision   y(m)

*     ==================================================================
*     s2unpk  expands the jq-th column of  ( A  -I )  into  y.
*     ==================================================================
      parameter        ( zero = 0.0d+0, one = 1.0d+0 )
*     ------------------------------------------------------------------

      call dload ( m, zero, y, 1 )

      if (jq .le. n) then
         do 10, k = ka(jq), ka(jq+1) - 1
            i     = ha(k)
            y(i)  = a(k)
   10    continue
      else
         islack    =   jq - n
         y(islack) = - one
      end if

*     end of s2unpk
      end
