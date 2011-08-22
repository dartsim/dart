*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
*     file  sn65rmod.f
*
*     s6chol   s6mchl   s6Rcnd   s6Rcyc   s6Rmod   s6Rsol   s6Rswp
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s6chol( pivot, inform, n,
     $                   Hdmin, dmax, iRank, perm, lenH, H )

      implicit           double precision (a-h,o-z)
      character          pivot*(*)
      integer            perm(n)
      double precision   H(lenH)

*     ==================================================================
*     s6chol  forms the upper-triangular Cholesky factor R  such that 
*                            H = R'R.
*
*     On entry,
*        pivot  specifies the pivoting strategy. 
*               pivot = 'N'o  specifies no pivoting, otherwise
*                       complete (i.e., diagonal) pivoting is used.
*
*        Hdmin  is the magnitude of the square of the smallest 
*               allowable diagonal of R.
*
*     On exit, 
*        perm   contains details of the permutation matrix P, such that
*               perm( k ) = k  if no column interchange occured at the
*               the kth step and  perm( k ) = j, ( k .lt. j .le. n ), 
*               if columns  k and j  were interchanged at the kth step.
*
*     Only the diagonal and super-diagonal elements of H are used,
*     and they are overwritten by the Cholesky factor, R.
*
*     28-May-94: First version of s6chol based on routine chlfac.
*     17-Jan-96: Current version.
*     ==================================================================
      parameter         (zero = 0.0d+0)
*     ------------------------------------------------------------------
      inform = 0
      iRank  = 0
      if (n .eq. 0) return
      dmin   = Hdmin

*     ------------------------------------------------------------------
*     Form the Cholesky factorization R'R = Z'HZ.
*     Process the first n rows of H.
*     Use symmetric interchanges if requested.
*     ------------------------------------------------------------------
      l = 1
      do 300, j = 1, n
         dmax = abs( H(l) )
         kmax = j
         lmax = l 

         if (pivot(1:1) .ne. 'N') then

*           Find the diagonal of the Schur complement with maximum
*           absolute value.

            ls   = l + j + 1 
            do 200,  k = j+1, n
               if (dmax .lt. abs( H(ls) )) then
                  dmax = abs( H(ls) )
                  kmax = k
                  lmax = ls
               end if
               ls = ls + k + 1
  200       continue
         end if

         dmax   = H(lmax)

*        Check that the diagonal is big enough.

         if (dmax .le. dmin) go to 800

*        If necessary, perform a symmetric interchange.

         perm(j) = kmax

         if (kmax .ne. j) then

            lcolj = l - j + 1
            lcolk = kmax*(kmax - 1)/2 + 1

*           call dswap ( kmax-j, H(j+1,kmax), 1, H(j,j+1), ldH )

            lck   = lcolk + j
            lrj   = l     + j
            do 210, i = j+1, kmax
               s      = H(lrj) 
               H(lrj) = H(lck)  
               H(lck) = s 
               lrj    = lrj + i
               lck    = lck + 1
  210       continue

*           call dswap ( j, H(1,j), 1, H(1,kmax), 1 )

            lcj   = lcolj
            lck   = lcolk
            do 220, i = 1, j
               s      = H(lcj) 
               H(lcj) = H(lck)  
               H(lck) = s 
               lcj    = lcj + 1
               lck    = lck + 1
  220       continue

*           call dswap ( n-kmax+1, H(kmax,kmax), ldH, H(j,kmax), ldH )

            lrk   = lcolk + kmax - 1
            lrj   = lcolk + j    - 1
            do 230, i = kmax, n
               s      = H(lrj) 
               H(lrj) = H(lrk)  
               H(lrk) = s 
               lrj    = lrj + i
               lrk    = lrk + i
  230       continue
         end if ! kmax ne j

*        Set the diagonal of  R.

         d     = sqrt( dmax )
         H(l)  = d
         iRank = iRank + 1

         if (j .lt. n) then

*           Set the super-diagonal elements of the jth row of R.
*           Do a rank-one update to the Schur complement.

            jx = l + j
            do 240, k = j+1, n
               H(jx)  = H(jx)/d
               jx     = jx + k
  240       continue

            jx = l + j
            ls = l + j + 1 

            do 260, js = j+1, n

*              Form the upper-triangular part of 
*                 H(:,js) = H(:,js) - H(j,js)*H(:,j).

               if (H(jx) .ne. zero) then
                  Hjx = H(jx)
                  ix  = l + j
                  do 250,  is = j+1, js
                     H(ls) = H(ls) - Hjx*H(ix)
                     ix    = ix + is
                     ls    = ls + 1
  250             continue
               end if
               jx = jx + js
               ls = jx + 1
  260       continue
         end if
         l = l + j + 1
  300 continue

*     ==================================================================
*     Test if  H  is not positive definite.
*     ==================================================================
  800 if (iRank .lt. n) inform = 1

*     end of s6chol
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s6mchl( nS, n, cndbnd, eps, 
     $                   perm, lenH, H, EmaxS, EmaxN )

      implicit           double precision (a-h,o-z)
      integer            perm(n)
      double precision   H(lenH)

*     ==================================================================
*     s6mchl  forms a modified Cholesky factorization of the matrix H,
*     such that
*                H  +  E  =  R'R,
*     where
*                E is diagonal,
*                R is upper triangular.
*     If H is sufficiently positive definite, E will be zero.
*
*     Only the diagonal and super-diagonal elements of H are used,
*     and they are overwritten by the Cholesky factor, R.
*
*     Let H_S be the first nS rows and cols of H.  These are treated
*     first.
*
*     On exit, 
*        perm   contains details of the permutation matrix P, such that
*               perm( k ) = k  if no column interchange occured at the
*               the kth step and  perm( k ) = j, ( k .lt. j .le. n ), 
*               if columns  k and j  were interchanged at the kth step.
*
*     Only the diagonal and super-diagonal elements of H are used,
*     and they are overwritten by the Cholesky factor, R.
*
*     EmaxS returns the largest (non-negative) diagonal of E_S.
*     If H_S was sufficiently positive definite, EmaxS will be zero.
*
*     Let H_N be the remaining matrix after H_S has been factorized.
*     EmaxN returns the largest (non-negative) diagonal of E_N.
*
*     eps must be at least as big as the relative machine precision.
*     The diagonals of R will be at least as large as sqrt(eps). 
*
*     21-Oct-93: First version of s6mchl based on routine chlfac.
*     22-Oct-93: The first nS rows and cols of H are treated separately.
*     20-May-94: Only the upper triangle of H is stored.
*     15-Oct-94: Symmetric interchanges added to H_S
*     15-Oct-94: Current version.
*     ==================================================================
      parameter         (zero = 0.0d+0)
*     ------------------------------------------------------------------

      EmaxS  = zero
      EmaxN  = zero

*     ------------------------------------------------------------------
*     Process the first nS rows of H.
*     ------------------------------------------------------------------
*     Find the maximum diagonal and super-diagonal elements of 
*     the first nS rows of H.

      dmax   = eps
      supmax = zero

      l = 1 
      do 110, j = 1, nS
         dmax   = max( dmax, abs( H(l) ) )
         jc     = l + j
         do 100, jx = j+1, nS
            supmax  = max( supmax, abs( H(jc) ) )
            jc      = jc + jx
  100    continue
         l = l + j + 1
  110 continue

*     Compute the bound on the off-diagonal elements of the first
*     nS rows and columns of R.

      if (nS .gt. 0) betasq = max( dmax, (supmax/nS) )
      delta  = sqrt( max(( dmax + supmax )/cndbnd, eps ) )

*     ------------------------------------------------------------------
*     Main loop for computing first nS rows of  R.
*     ------------------------------------------------------------------
      l = 1
      do 300, j = 1, nS
         dmax   = abs( H(l) )
         kmax   = j
         lmax   = l 

*        Find the diagonal of the Schur complement with maximum
*        absolute value.

         ls     = l + j + 1 
         do 200,  k = j+1, nS
            if (dmax .lt. abs( H(ls) )) then
               dmax = abs( H(ls) )
               kmax = k
               lmax = ls
            end if
            ls = ls + k + 1
  200    continue

*        ******************************************************
         kmax   = j
         lmax   = l 
*        ******************************************************

         dmax   = H(lmax)

*        If necessary, perform a symmetric interchange.

         perm(j) = kmax

         if (kmax .ne. j) then

            lcolj = l - j + 1
            lcolk = kmax*(kmax - 1)/2 + 1

*           call dswap ( kmax-j, H(j+1,kmax), 1, H(j,j+1), ldH )

            lck   = lcolk + j
            lrj   = l     + j
            do 210, i = j+1, kmax
               s      = h(lrj) 
               h(lrj) = h(lck)  
               h(lck) = s 
               lrj    = lrj + i
               lck    = lck + 1
  210       continue

*           call dswap ( j, H(1,j), 1, H(1,kmax), 1 )

            lcj   = lcolj
            lck   = lcolk
            do 220, i = 1, j
               s      = h(lcj) 
               h(lcj) = h(lck)  
               h(lck) = s 
               lcj    = lcj + 1
               lck    = lck + 1
  220       continue

*           call dswap ( n-kmax+1, H(kmax,kmax), ldH, H(j,kmax), ldH )

            lrk   = lcolk + kmax - 1
            lrj   = lcolk + j    - 1
            do 230, i = kmax, n
               s      = h(lrj) 
               h(lrj) = h(lrk)  
               h(lrk) = s 
               lrj    = lrj + i
               lrk    = lrk + i
  230       continue
         end if

*        Find the largest super-diagonal in the jth row.
                          
         supj  = zero
         jc    = l + j
         do 240, jx = j+1, nS
            supj    = max( supj, abs( H(jc) ) )
            jc      = jc + jx
  240    continue
         if (supj .lt. delta) supj = zero

*        Set the diagonal of  R.

         d     = max( delta, abs( dmax ), supj**2/betasq )
         e     = d - dmax
         EmaxS = max( EmaxS, e )
         d     = sqrt( d )
         H(l)  = d

         if (j .lt. n) then

*           Set the super-diagonal elements of the jth row of R.
*           Do a rank-one update to the Schur complement.

            jx = l + j
            do 250, k = j+1, n
               H(jx)  = H(jx)/d
               jx     = jx + k
  250       continue

            jx = l + j
            ls = l + j + 1 

            do 270, js = j+1, n

*              Form the upper-triangular part of 
*                 H(:,js) = H(:,js) - H(j,js)*H(:,j).

               if (H(jx) .ne. zero) then
                  Hjx = H(jx)
                  ix  = l + j
                  do 260, is = j+1, js
                     H(ls)   = H(ls) - Hjx*H(ix)
                     ix      = ix + is
                     ls      = ls + 1
  260             continue
               end if
               jx = jx + js
               ls = jx + 1
  270       continue
         end if
         l = l + j + 1
  300 continue

      if (nS .ge. n) return

*     ------------------------------------------------------------------
*     Process H_N, the remaining rows and columns of the transformed H.
*     ------------------------------------------------------------------

*     Find the maximum diagonal and super-diagonal elements of H_N.

      lhN    = l
      nN     = n - nS
      dmax   = eps
      supmax = zero

      do 510, j = nS+1, n
         dmax   = max( dmax, abs( H(l) ) )
         jc     = l + j
         do 500, jx = j+1, n
            supmax  = max( supmax, abs( H(jc) ) )
            jc      = jc + jx
  500    continue
         l = l + j + 1
  510 continue

*     Compute the bound on the off-diagonal elements of R_N.

      betasq = max( dmax, (supmax/nN)  )
      delta  = sqrt( max(( dmax + supmax )/cndbnd, eps ) )

*     ------------------------------------------------------------------
*     Main loop for computing the last n-nS rows of  R.
*     ------------------------------------------------------------------
      l = lhN

      do 560, j = nS+1, n
         dj     = H(l)

*        Find the largest super-diagonal in the jth row.
                          
         supj  = zero
         jc    = l + j
         do 520, jx = j+1, n
            supj    = max( supj, abs( H(jc) ) )
            jc      = jc + jx
  520    continue
         if (supj .lt. delta) supj = zero

*        Set the diagonal of  R.

         d     = max( delta, abs( dj ), supj**2/betasq )
         e     = d - dj
         EmaxN = max( EmaxN, e )
         d     = sqrt( d )
         H(l)  = d

         if (j .lt. n) then

*           Set the super-diagonal elements of the jth row of R.
*           Do a rank-one update to the Schur complement.

            jx = l + j
            do 530, k = j+1, n
               H(jx)  = H(jx)/d
               jx     = jx + k
  530       continue

            jx = l + j
            ls = l + j + 1 

            do 550, js = j+1, n

*              Form the upper-triangular part of 
*                 H(:,js) = H(:,js) - H(j,js)*H(:,j).

               if (H(jx) .ne. zero) then
                  Hjx = H(jx)
                  ix  = l + j
                  do 540,  is = j+1, js
                     H(ls) = H(ls) - Hjx*H(ix)
                     ix    = ix + is
                     ls    = ls + 1
  540             continue
               end if
               jx = jx + js
               ls = jx + 1
  550       continue
         end if
         l = l + j + 1
  560 continue

*     end of s6mchl
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s6Rcnd( nR, lenr, r, dRmax, dRmin, Rmax, condH )

      implicit           double precision (a-h,o-z)
      double precision   r(lenr)

*     ==================================================================
*     s6Rcnd  finds the largest and smallest diagonals of the
*     upper triangular matrix  R, and returns the square of their ratio.
*     This is a lower bound on the condition number of  R' R.
*     ==================================================================
      logical            overfl
      parameter         (one = 1.0d+0)
*     ------------------------------------------------------------------

      if (nR .eq. 0) then     
         condH = one
         dRmin = one
         dRmax = one
         Rmax  = one
      else
         dRmax = abs( r(1) )
         dRmin = dRmax
         Rmax  = dRmax

         if (nR .eq. 1) then     
            condH = ddiv( one, dRmin**2, overfl )
            if (condH .lt. one) condH = one/condH
         else 
            l     = 2
            do 200, j = 2, nR
               do 100, i = 1, j
                  d      = abs( r(l) )
                  Rmax   = max( Rmax, d )
                  l      = l + 1
  100          continue

               dRmin = min( dRmin, d )
               dRmax = max( dRmax, d )
  200       continue
            
            condH  = (dRmax/dRmin)**2
         end if
      end if

*     end of s6Rcnd
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s6Rcyc( jq, jr, eps, lenr, nR, r, w )

      implicit           double precision (a-h,o-z)
      double precision   r(lenr), w(nR)

*     ==================================================================
*     s6Rcyc  moves column jq of the upper-triangular matrix R
*     to position jr, and cyclically shifts the intervening columns
*     to the right or left (depending on whether jq > jr or jq < jr).
*     R is retriangularized using a partial sweep of plane rotations.
*
*     13-Aug-92: First version of s6Rcyc.
*     19-Jul-97: Current version.
*     ==================================================================
      parameter        ( zero = 0.0d+0 )
*     ------------------------------------------------------------------
      tolz   = eps
      tolz   = zero

      jq0    = jq - 1
      jr0    = jr - 1
      jq1    = jq + 1
      jr1    = jr + 1
      lqdiag = jq * jq1 / 2
      lrdiag = jr * jr1 / 2

      if (jq .gt. jr) then
*        ===============================================================
*        jq > jr.  Move column jq to the left.
*
*        We first permute the middle rows and columns of R
*        from  ( R1  S   w1  T1 )  to  ( R1  w1  S   T1 ),
*              (     R2  w2  T2 )      (     d   0   v' )
*              (         d   v' )      (     w2  R2  T2 )
*              (             R3 )      (             R3 )
*        where columns jq and jr of the initial R correspond to
*        w1 and the first column of S respectively.
*
*        We then reduce w2 to zero with a backward sweep of rotations
*        into the new row jr, altering (d  0  v') and (R2  T2).        
*        ===============================================================
*        Extract d, w1 and w2.
*        w' contains  ( w1'  d  w2' )

         d      = r(lqdiag)
         lr     = lqdiag - 1
         lw     = jq

         do 60, i = jq0, 1, -1
            w(lw) = r(lr)
            lr    = lr - 1
            lw    = lw - 1
            if (i .eq. jr) lw = lw - 1
  60     continue

*        ---------------------------------------------------------------
*        Move R2 and S, running backwards up the columns.
*        ---------------------------------------------------------------
         k      = lqdiag - jq
         l      = lqdiag

         do 120, j = jq0, jr, -1
            do 100, i = j, 1, -1
               r(l)   = r(k)
               k      = k - 1
               l      = l - 1
               if (i .eq. jr) l = l - 1
  100       continue
  120    continue

*        Put w1 in between R1 and S.

         lr     = lrdiag - jr0
         call dcopy ( jr0, w, 1, r(lr), 1 )

*        ---------------------------------------------------------------
*        Change  ( T2 )  to  ( v' ).
*                ( v' )      ( T2 )
*        ---------------------------------------------------------------
         lv     = lqdiag + jq

         do 160, j = jq1, nR
            v      = r(lv)
            lt     = lv

            do 140, i = jq, jr1, -1
               r(lt)  = r(lt - 1)
               lt     =   lt - 1
  140       continue

            r(lt)  = v
            lv     = lv + j
  160    continue

*        ---------------------------------------------------------------
*        Triangularize  ( d   0   v' )  using a partial backward sweep
*                       ( w2  R2  T2 )
*        of rotations to eliminate w2.
*        lv and lr mark the start of the rows involved.
*        ---------------------------------------------------------------
         lv     = lqdiag - jq + jr
         lr     = lqdiag

         do 220, i = jq, jr1, -1
            t      = w(i)
            root   = sqrt( d**2 + t**2 )
            cs     = d/root
            sn     = t/root

*           Apply rotation to rows jr and i.

            r(lv)  = zero
            k      = lv
            l      = lr

            if (abs(sn) .gt. tolz) then
               d     = root
               do 200, j = i, nR
                  s      = r(k)
                  t      = r(l)
                  r(k)   = cs*s + sn*t
                  r(l)   = sn*s - cs*t
                  k      = k + j
                  l      = l + j
  200          continue
            end if

            lv     = lv - i + 1
            lr     = lr - i
  220    continue

*        ---------------------------------------------------------------
*        Store the final diagonal.
*        ---------------------------------------------------------------
         r(lrdiag) = d

      else if (jq .lt. jr) then
*        ===============================================================
*        jq < jr.  Move column jq to the right.
*
*        We first permute the middle rows and columns of R
*        from  ( R1  w1  S   T1 )  to  ( R1  S   w1  T1 ),
*              (     d   w2' w3')      (     R2  0   T2 )
*              (         R2  T2 )      (     w2' d   w3')
*              (             R3 )      (             R3 )
*        where columns jq and jr of the initial R correspond to
*        w1 and the last column of S respectively.
*
*        We then reduce w2' to zero with a forward sweep of rotations
*        into the new row jq, altering (R2  0  T2) and (d  w3').
*        ===============================================================
*        Extract d, w1, w2 and w3.
*        w' contains  ( w1'  w2'  d  w3' )

         d      = r(lqdiag)
         lr     = lqdiag - jq0
         call dcopy ( jq0, r(lr), 1, w, 1 )

         lr     = lqdiag + jq
         lw     = jq

         do 260, j = jq1, nR
            w(lw)  = r(lr)
            lr     = lr + j
            lw     = lw + 1
            if (j .eq. jr) lw = lw + 1
 260     continue

         w(jr)     = d

*        ---------------------------------------------------------------
*        Move S and R2, running forwards down the columns.
*        ---------------------------------------------------------------
         l      = lqdiag - jq0
         k      = lqdiag + 1

         do 320, j = jq1, jr
            do 300, i = 1, j-1
               if (i .eq. jq) k = k + 1
               r(l)   = r(k)
               l      = l + 1
               k      = k + 1
  300       continue
  320    continue

*        Put w1 in between S and T1.

         lr     = lrdiag - jr0
         call dcopy ( jq0, w, 1, r(lr), 1 )

*        ---------------------------------------------------------------
*        Move T2 up one row.
*        ---------------------------------------------------------------
         lr     = lrdiag + jq

         do 360, j = jr1, nR
            lt     = lr
            do 340, i = jq, jr0
               r(lt)  = r(lt + 1)
               lt     =   lt + 1
  340       continue

            lr     = lr + j
  360    continue

*        ---------------------------------------------------------------
*        Triangularize  ( R2  0  T2 )  using a partial backward sweep
*                       ( w2' d  w3')
*        of rotations to eliminate w2'.
*        lr marks the start of the rows of R2.
*        lz marks the zero to be inserted between R2 and T2.
*        ---------------------------------------------------------------
         lr     = lqdiag
         lz     = lrdiag - jr + jq

         do 420, i = jq, jr0
            r(lz)  = zero
            d      = r(lr)
            t      = w(i)
            root   = sqrt( d**2 + t**2 )
            cs     = d/root
            sn     = t/root

*           Apply rotation to rows i and jr  (where row jr is in w').

            k      = lr + i

            if (abs(sn) .gt. tolz) then
               r(lr)  = root
               do 400, j = i+1, nR
                  s      = r(k)
                  t      = w(j)
                  r(k)   = cs*s + sn*t
                  w(j)   = sn*s - cs*t
                  k      = k + j
  400          continue
            end if

            lr     = lr + i + 1
            lz     = lz + 1
  420    continue

*        ---------------------------------------------------------------
*        Store w(jr) ... w(nR) as the final jr-th row of R.
*        ---------------------------------------------------------------
         lr     = lrdiag

         do 460, j = jr, nR
            r(lr)  = w(j)
            lr     = lr + j
  460    continue
      end if

*     end of s6Rcyc
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s6Rmod( n, lenr, r, v, w, lastv, vnorm, tolz )

      implicit           double precision (a-h,o-z)
      double precision   r(lenr), v(n), w(n)

*     ==================================================================
*     s6Rmod  modifies the first n rows and columns of the (n+1)x(n+1)
*     upper-triangular matrix  R  so that  Q*(R + v w')  is upper
*     triangular, where  Q  is orthogonal.
*     v  and  w  are (n+1)-vectors whose first n elements are stored in
*     arrays v and w.  The last component of v is held in vlast.
*     The new  R  overwrites the old.
*
*     Q  is the product of two sweeps of plane rotations (not stored).
*     These affect the  (lastv)th  row of  R,  which is temporarily held
*     in the array  v.  Thus,  v  is overwritten.  w  is not altered.
*
*     lastv  points to the last nonzero of  v.  The value lastv = n
*            would always be ok, but sometimes it is known to be less
*            than  n,  so  Q  reduces to two partial sweeps of
*            rotations.
*
*     vnorm  is the norm of the (n+1)-vector v.
*
*     tolz   is a tolerance for negligible elements in  v.
*
*     06-Sep-91: First version based on Minos routine m6rmod.
*     11-Sep-94: Modified to update a leading section of R.
*     12-Sep-94: Current version.
*     ==================================================================
      parameter        ( zero = 0.0d+0 )
*     ------------------------------------------------------------------

      lastR  = lastv*(lastv + 1)/2

      if (lastv .le. n) then
         vlast = v(lastv)
      else
         vlast = sqrt( max(zero, vnorm**2 - dnrm2( n, v, 1 )**2) )
      end if

*     Copy the (lastv)th row of  R  into the end of  v.

      l      = lastR
      do 100, j = lastv, n
         v(j)   = r(l)
         l      = l + j
  100 continue

*     ------------------------------------------------------------------
*     Reduce v to a multiple of e(lastv) using a partial backward sweep
*     of rotations.  This fills in the (lastv)th row of R (held in v).
*     ------------------------------------------------------------------
      if (lastv .gt. 1) then
         v2     = vlast**2
         ldiag  = lastR

         do 300, i = lastv-1, 1, -1
            ldiag  = ldiag - i - 1
            s      = v(i)
            v(i)   = zero
            if (abs(s) .gt. tolz) then
               v2     = s**2 + v2
               root   = sqrt(v2)
               cs     = vlast/root
               sn     = s    /root
               vlast  = root
               l      = ldiag

               do 200, j = i, n
                  s      = v(j)
                  t      = r(l)
                  v(j)   = cs*s + sn*t
                  r(l)   = sn*s - cs*t
                  l      = l + j
  200          continue
            end if
  300    continue
      end if

*     Set  v  =  v  +  vlast*w.

      call daxpy ( n, vlast, w, 1, v, 1 )

*     ------------------------------------------------------------------
*     Eliminate the front of the (lastv)th row of R (held in v) using a
*     partial forward sweep of rotations.
*     ------------------------------------------------------------------
      if (lastv .gt. 1) then
         ldiag  = 0

         do 700, i = 1, lastv-1
            ldiag  = ldiag + i
            t      = v(i)
            if (abs(t) .gt. tolz) then
               s        = r(ldiag)
               root     = sqrt(s**2 + t**2)
               cs       = s/root
               sn       = t/root
               r(ldiag) = root
               l        = ldiag + i

               do 600, j = i+1, n
                  s      = r(l)
                  t      = v(j)
                  r(l)   = cs*s + sn*t
                  v(j)   = sn*s - cs*t
                  l      = l + j
  600          continue
            end if
  700    continue
      end if

*     -----------------------------------
*     Insert the new (lastv)th row of  R.
*     -----------------------------------
      l  = lastR
      do 900, j = lastv, n
         r(l)   = v(j)
         l      = l + j
  900 continue

*     end of s6Rmod
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s6Rsol( job, lenr, nR, r, y )

      implicit           double precision (a-h,o-z)
      character          job*(*)
      double precision   r(lenr), y(nR)

*     ==================================================================
*     s6Rsol  solves  R x = y  or  R' x = y,  where  R  is an
*     upper-triangular matrix stored by columns in  r(lenr).
*     The solution  x  overwrites  y.
*     ==================================================================

      if (job(1:2) .eq. 'R ') then

*        Job = 'R y = y'.

         l   = nR*(nR + 1)/2
         do 120, ii = 1, nR
            i       = nR + 1 - ii
            t       = y(i)/r(l)
            y(i)    = t
            if (i .ne. 1) then
               l    = l - i
               call daxpy ( i-1, (-t), r(l+1), 1, y, 1 )
            end if
  120    continue
      else 

*        Job  = 'Rtranspose y = y'.

         y(1) = y(1)/r(1)
         if (nR .gt. 1) then
            l = 1
            do 220, i = 2, nR
               t      = ddot  ( i-1, r(l+1), 1, y, 1 )
               l      = l + i
               y(i)   = (y(i) - t)/r(l)
  220       continue
         end if
      end if

*     end of s6Rsol
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s6Rswp( nR, lenr, r, v, w, lastv )

      implicit           double precision (a-h,o-z)
      double precision   r(lenr), v(nR), w(nR)

*     ==================================================================
*     s6Rswp  modifies the upper-triangular matrix R to account for a 
*     basis exchange in which the  (lastv)th superbasic variable becomes
*     basic.  R  is changed to R  +  vw',  which is triangularized by
*     s6Rmod.  v  is the  (lastv)th column of R,  and w is input.
*
*     01-Dec-91: First version of s6Rswp based on Minos routine m6bswp.
*     24-Apr-94: Nx no longer in Q.
*     24-Jan-96: v left unscaled.
*     24-Jan-96: Current version.
*     ==================================================================
      parameter         (zero   = 0.0d+0)
*     ------------------------------------------------------------------
      tolz   = zero

*     Set  v =  lastv-th column of  R  and find its norm.

      l      = (lastv - 1)*lastv/2
      call dcopy ( lastv, r(l+1), 1, v, 1 )
      vnorm  = dasum ( lastv, v, 1 )
      call s6Rmod( nR, lenr, r, v, w, lastv, vnorm, (vnorm*tolz) )

*     end of s6Rswp
      end

