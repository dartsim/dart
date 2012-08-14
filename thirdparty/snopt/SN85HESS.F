*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
*     File  sn85Hess.f
*
*     s8BFGS   s8H0     s8HDum   s8Hmap   s8Hx     s8Hmod   s8Rupd
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8BFGS( lenr, n, nFull, ys, pHp, r, Hp, y, Rp )

      implicit           double precision (a-h,o-z)
      double precision   r(lenr)
      double precision   Hp(n), y(n), Rp(n)

*     ==================================================================
*     s8BFGS  applies the quasi-Newton update to the Cholesky factor R
*     of the approximate Hessian.
*
*     On entry:
*     r      contains the first n rows and columns of an
*            upper-triangular matrix R such that R'R = H.
*
*     Hp     contains the first n components of the vector Hp such that
*            Hp = - g.
*
*     y      contains the first  n  components of the difference g2 - g.
*
*     Rp     is the vector such that  R'*(Rp) = Hp.  It is overwritten.
*
*     w      is a work vector.
*
*     On exit:
*
*     30 Dec 1991: First version based on  Npsol  routine npbfgs.
*     24 Jan 1996: Current version.
*     ==================================================================
      parameter         (zero = 0.0d+0, one = 1.0d+0)
*     ------------------------------------------------------------------
      tolz   = zero
      rtys   = sqrt( ys )
      Rpnorm = sqrt( pHp )

*     Normalize  Rp.
*     Scale Hp.

      c      = one/Rpnorm
      call dscal ( n, (-c), Hp, 1 )
      call dscal ( n,   c , Rp, 1 )
 
*     ------------------------------------------------------------------
*     Apply the update to R. 
*     Set w for the update  R + v w'.
*     w is held in Hp,  v  is held in Rp.
*     ------------------------------------------------------------------
      call daxpy ( n, (one/rtys), y, 1, Hp, 1 )

      if (n .lt. nFull) then
*        --------------------------------------------
*        The leading nxn section of R is updated.
*        --------------------------------------------
         lastnz = n + 1
      else
*        --------------------------------------------
*        The full R is updated.
*        Find the last nonzero in v.
*        --------------------------------------------
         do 300, k = n, 1, -1
            lastnz = k
            if (abs( Rp(lastnz) ) .gt. tolz) go to 500
  300    continue
      end if
*     -------------------------------------------------------
*     Restore  R + v w'  to triangular form  (overwriting v).
*     -------------------------------------------------------
  500 call s6Rmod( n, lenr, r, Rp, Hp, lastnz, one, tolz )

*     end of s8BFGS
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8H0  ( Reset, nnL, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*1        Reset
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     s8H0    initializes the BFGS approximate Hessian.
*
*     s8H0   calls one of the Hessian routines s8LMH, s8FMH, s8SDH, ... 
*     according to the value of the option iHtype.
*     Each of these routines defines a particular form of the Hessian.
*     At the moment the options are:
*        iHtype = 1   Limited-Memory (LM) BFGS Hessian  (the default).
*        iHtype = 2   Full-Memory    (FM) BFGS Hessian.
*        iHtype = 3   second derivative or fd Hessian plus LM updates.
*                     (coming real soon now).
*
*     19 Jul 1995: First version of s8H0   written by PEG.
*     12 Jan 1996: Full memory Hessian option added.
*     20 Jul 1997: Current version.
*     ==================================================================
      parameter         (nQNmod = 201)
      parameter         (nSkip  = 388)
      parameter         (nFlush = 389)
*     ------------------------------------------------------------------
      lHtype    = iw( 53)

      if      (lHtype .eq. 1) then
         lH0       = iw(381)
         mFlush    = iw( 68)

         call s8LMH0( Reset, nnL, iw(nQNmod),
     $                mFlush, iw(nFlush), iw(nSkip), rw(lH0) )

      else if (lHtype .eq. 2) then
         lH        = iw(371)
         lenH      = iw(372)
         mFlush    = iw( 68)

         call s8FMH0( Reset, nnL, iw(nQNmod),
     $                mFlush, iw(nFlush), iw(nSkip), rw(lH), lenH )

*     else if (lHtype .eq. 3) then
*        call s8SDH0( )

      end if

*     end of s8H0
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8Hmap( nnL, nnCon, minrw, iw, leniw )

      implicit           double precision (a-h,o-z)
      integer            iw(leniw)

*     =================================================================
*     s8Hmap is called by s8Mem to set up the memory allocation for the
*     approximate Hessian.
*
*     It calls one of the Hessian routines s8LMH, s8FMH, s8SDH, ... 
*     according to the value of the option lHtype.
*     Each of these routines defines a particular form of the Hessian.
*     At the moment the options are:
*        lHtype = 1   Limited-Memory (LM) BFGS Hessian  (the default).
*        lHtype = 2   Full-Memory    (FM) BFGS Hessian.
*        lHtype = 3   second derivative or fd Hessian plus LM updates.
*                     (coming real soon now).
*
*     13 Dec 1995: First version of s8Hmap.
*     12 Jan 1996: Full memory Hessian option added.
*     16 May 1998: Current version.
*     ==================================================================
      lHtype    = iw( 53)

      if      (lHtype .eq. 1) then
         call s8LMmp( nnL, minrw, iw, leniw )

      else if (lHtype .eq. 2) then
         call s8FMmp( nnL, minrw, iw, leniw )

*     else if (lHtype .eq. 3) then
*        call s8SDmp( nnL, minrw, iw, leniw )
      end if

*     end of s8Hmap
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8HDum( )

*     ==================================================================
*     s8HDum  is the dummy argument for s8Hx.
*
*     18 Dec 1995: Current version.
*     ==================================================================
*     Relax

*     end of s8HDum
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8Hx  ( s8HDum, minimz, n, nnL, ne, nka, a, ha, ka,
     $                   x, Hx, nState, 
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      external           s8HDum
      integer            ha(ne)
      integer            ka(nka)
      double precision   a(ne)
      double precision   Hx(nnL), x(nnL)

      character*8        cu(lencu), cw(lencw)
      integer            iu(leniu), iw(leniw)
      double precision   ru(lenru), rw(lenrw)

*     ==================================================================
*     s8Hx  multiplies the QP Hessian  H by the vector  x.
*     It is used to define Hx for the QP subproblem.
*
*     On entry:
*        nState  = 0  => a normal call for H*x.
*        nState  = 1  => the first entry for a given QP.
*        nState ge 2  => last call for a given QP. nState = 2+ierror.
*
*     On exit:
*        nstate lt 0   the user wants to stop.
*
*     s8Hx calls one of the Hessian routines s8LMH, s8FMH, s8SDH, ... 
*     according to the value of the option lHtype.
*     Each of these routines defines a particular form of the Hessian.
*     At the moment the options are:
*        lHtype = 1   Limited-Memory (LM) BFGS Hessian  (the default).
*        lHtype = 2   Full-Memory    (FM) BFGS Hessian.
*        lHtype = 3   second derivative or fd Hessian plus LM updates.
*                     (coming real soon now).
*
*     30 Dec 1991: First version of s8Hx.
*     12 Jan 1996: Full memory Hessian option added.
*     19 Jul 1997: Current version.
*     ==================================================================
      parameter         (nQNmod = 201)
*     ------------------------------------------------------------------
      lHtype    = iw( 53)

      if      (lHtype .eq. 1) then
         mQNmod    = iw( 66)

         lH0       = iw(381)
         lgdsav    = iw(382)
         lHpsav    = iw(383)
         lyts      = iw(384)
         lptHp     = iw(385)

         call s8LMHx( nnL, x, Hx, mQNmod, iw(nQNmod),
     $                rw(lH0), rw(lgdsav), rw(lHpsav),
     $                rw(lyts), rw(lptHp) )

      else if (lHtype .eq. 2) then
         lH        = iw(371)
         lenH      = iw(372)

         call s8FMHx( nnL, lenH, rw(lH), x, Hx )

*     else if (lHtype .eq. 3) then
*        call s8SDH ( )

      end if

      sgnObj = minimz
      if (minimz .lt. 0) call dscal ( nnL, sgnObj, Hx, 1 )

*     end of s8Hx
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8Hmod( QNmsg, Reset, nnL, nnCon, 
     $                   step, yp, ypmin, pHp, Hp, y,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*1        QNmsg, Reset
      integer            iw(leniw)
      double precision   rw(lenrw)
      double precision   Hp(nnL), y(nnL)

*     ==================================================================
*     s8Hmod  applies the pair of vectors that define the BFGS update
*     or self-scaled BFGS update.
*
*     On entry:
*
*     Hp    contains the first nnL components of the vector Hp such
*           that  Hp = - g.
*
*     y     contains the first  nnL components of the difference g2 - g.
*
*     s8Hmod calls one of the Hessian routines s8LMH, s8FMH, s8SDH, ... 
*     according to the value of the option lHtype.
*     Each of these routines defines a particular form of the Hessian.
*     At the moment the options are:
*        lHtype = 1   Limited-Memory (LM) BFGS Hessian  (the default).
*        lHtype = 2   Full-Memory    (FM) BFGS Hessian.
*        lHtype = 3   second derivative or fd Hessian plus LM updates.
*                     (coming real soon now).
*
*     19 Jul 1995: First version of s8Hmod written by PEG.
*     12 Jan 1996: Full-memory Hessian option added.
*     20 Jul 1997: Current version.
*     ==================================================================
      integer            H0scal
      parameter         (H0scal = 361)
      parameter         (nQNmod = 201)
      parameter         (nSkip  = 388)
      parameter         (nFlush = 389)
*     ------------------------------------------------------------------
      lHtype    = iw( 53)

      if      (lHtype .eq. 1) then
         lH0       = iw(381)
         lgdsav    = iw(382)
         lHpsav    = iw(383)
         lyts      = iw(384)
         lptHp     = iw(385)

         mQNmod    = iw( 66)
         mSkip     = iw( 69)

         call s8LMsv( QNmsg, Reset,
     $                nnL, nnCon, mQNmod, iw(nQNmod),
     $                mSkip, iw(nSkip), iw(nFlush), rw(H0scal),
     $                step, yp, ypmin, pHp, Hp, y, rw(lH0),
     $                rw(lgdsav), rw(lHpsav), rw(lyts), rw(lptHp) )

      else if (lHtype .eq. 2) then
         lH        = iw(371)
         lenH      = iw(372)

         mQNmod    = iw( 66)
         mSkip     = iw( 69)

         call s8FMsv( QNmsg, Reset, 
     $                lenH, nnL, nnCon, mQNmod, iw(nQNmod),
     $                mSkip, iw(nSkip), iw(nFlush), rw(H0scal),
     $                step, yp, ypmin, pHp, rw(lH), Hp, y )

*     else if (lHtype .eq. 3) then
*        call s8SDH ( )
 
      end if

*     end of s8Hmod
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8Rupd( QNmsg, lenr, m, n, nBS, nnL, nS,
     $                   ys, pHp,
     $                   ne, nka, a, ha, ka,
     $                   kBS, gdif, Hp, r, w, y, y2,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision(a-h,o-z)
      character*1        QNmsg
      integer            ha(ne)
      integer            ka(nka), kBS(nBS)
      integer            iw(leniw)
      double precision   rw(lenrw)
      double precision   a(ne)
      double precision   r(lenr)
      double precision   gdif(nnL), Hp(nnL)
      double precision   y(nBS), y2(nBS), w(nBS)

*     ==================================================================
*     s8Rupd updates the Cholesky factor of the reduced Hessian.
*     
*     04 Aug 1995: First version based on NPSOL routine npupdt.
*     15 Jan 1996: Current version.
*     ==================================================================
      parameter         (zero = 0.0d+0, one = 1.0d+0)
*     ------------------------------------------------------------------
      eps0      = rw(  2)
     
*     -----------------------
*     Find Z'*gdif and  Z'Hp.
*     -----------------------
      do 100, k = 1, nBS
         j      = kBS(k)
         if (j .le. nnL) then
            y (k) = gdif(j)
            y2(k) = Hp  (j)
         else
            y (k) = zero
            y2(k) = zero
         end if
  100 continue

      call s5setp( m, piNorm, y, w, iw, leniw, rw, lenrw )

      if (nS .gt. 0) then
         call s2Bprd( 'Transpose', eps0, n, nS, kBS(m+1),
     $                ne, nka, a, ha, ka,
     $                (-one), w, m, one, y(m+1), nS ) 
      end if

      call s5setp( m, piNorm, y2, w, iw, leniw, rw, lenrw )

      if (nS .gt. 0) then
         call s2Bprd( 'Transpose', eps0, n, nS, kBS(m+1),
     $                ne, nka, a, ha, ka,
     $                (-one), w, m, one, y2(m+1), nS ) 
      end if

      if (QNmsg .eq. 's'  .or.  QNmsg .eq. 'S') then
*        --------------------
*        Self-scaled BFGS.
*        --------------------
         H0scal    = rw(361)
         rH0scl    = sqrt( H0scal )
         lenr0     = nS*(nS + 1)/2
         call dscal( lenr0, rH0scl, r , 1 )                
      end if

*     ---------------------------------------------------------
*     Apply the BFGS update to the first nS columns of R.
*     y = Z'gdif,  y2 = Rp  and  w = Z'Hp.
*     ---------------------------------------------------------
      call dcopy ( nS, y2(m+1), 1, w(m+1), 1 )
      call s6Rsol( 'Rt', lenr, nS, r, y2(m+1) )
      call s8BFGS( lenr, nS, nnL, ys, pHp, r, w(m+1), y(m+1), y2(m+1) ) 

*     end of s8Rupd
      end

