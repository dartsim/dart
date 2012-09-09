*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
*     File  sn90lmqn.f.   Limited-memory BFGS routine.
*
*     s8LMH0    s8LMHmp   s8LMsv   s8LMHx
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8LMH0( Reset, nnL, nQNmod,
     $                   mFlush, nFlush, nSkip, H0 )

      implicit           double precision (a-h,o-z)
      character*1        Reset
      double precision   H0(nnL)

*     ==================================================================
*     s8LMH0 initializes the approximate Hessian.
*
*     The Hessian is flushed once at iteration mFlush
*
*     19-Jul-95: First version of s8LMH0 written by PEG.
*     19-Jul-97: Current version.
*     ==================================================================
      parameter         (one = 1.0d+0)
*     ------------------------------------------------------------------
      nQNmod    = 0
      nFlush    = mFlush
      nSkip     = 0

      Reset     = 'R'
      H0diag    = one
      call dload ( nnL, H0diag, H0, 1 )

*     end of s8LMH0
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8LMmp( nnL, minrw, iw, leniw )

      implicit           double precision (a-h,o-z)
      integer            iw(leniw)

*     ==================================================================
*     s8LMH0 computes the addresses of the quasi-Newton arrays.
*     These are saved and used for subsequent entries.
*
*     This version defines a limited-memory BFGS form of H.
*
*     30-Dec-91: First version of s8LMH0.
*     19-Jul-97: Current version.
*     ==================================================================

      mQNmod    = iw( 66)

*     ------------------------------------------------------------------
*     Compute the addresses of the limited-memory arrays.
*     These are saved and used for subsequent entries.
*     ------------------------------------------------------------------
      lH0       = minrw
      lgdsav    = lH0    + nnL
      lHpsav    = lgdsav + nnL*mQNmod
      lyts      = lHpsav + nnL*mQNmod
      lptHp     = lyts   +     mQNmod
      minrw     = lptHp  +     mQNmod

      iw(381)   = lH0    
      iw(382)   = lgdsav 
      iw(383)   = lHpsav 
      iw(384)   = lyts   
      iw(385)   = lptHp  

*     end of s8LMmp
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8LMsv( QNmsg, Reset,
     $                   nnL, nnCon, mQNmod, nQNmod,
     $                   mSkip, nSkip, nFlush, H0scal,
     $                   step, yp, ypmin, pHp, Hp, y, H0,
     $                   gdif, Hxdif, yts, ptHp )

      implicit           double precision (a-h,o-z)
      character*1        QNmsg, Reset
      double precision   Hp(nnL), y(nnL)
      double precision   yts(mQNmod), ptHp(mQNmod)
      double precision   H0(nnL)
      double precision   gdif(nnL,mQNmod), Hxdif(nnL,mQNmod)

*     ==================================================================
*     s8LMsv stores the update (Hp,y) from this iteration.
*     If necessary, the self-scaling parameter is saved in case there 
*     are linear constraints.
*
*     19-Jul-95: First version of s8LMsv written by PEG.
*     19-Jul-97: Current version.
*     ==================================================================
      parameter         (zero = 0.0d+0, one = 1.0d+0)
*     ------------------------------------------------------------------

*     ------------------------------------------------------------------
*     If the BFGS update can be done, prepare Hp and y.
*     ------------------------------------------------------------------
      if (yp .ge. ypmin  .and.  pHp .gt. zero) then
         ys    = step*yp
         ysi   = one / ys
         pHpi  = one / pHp

         if (nQNmod .eq. 0) then
            if (QNmsg .eq. ' ') then
               QNmsg = 's'
            else
               QNmsg = 'S'
            end if
            sHs     = pHp*step*step
            H0scal  = ys / sHs

            call dscal ( nnL, H0scal, Hp, 1 )
            pHp     = pHp  * H0scal
            pHpi    = pHpi / H0scal
            call dscal ( nnL, H0scal, H0, 1 )
         end if

         if (nQNmod .ge. mQNmod  .or. nQNmod .eq. nFlush) then
*           ------------------------------------------------------------
*           Insufficient space for storing the new (Hp,y).
*           Discard any updates accumulated so far.
*           Reset H0 to be the diagonal of the current H.
*           ------------------------------------------------------------
            do 110, k = 1, min(mQNmod, nQNmod)
               do 100, i = 1, nnL
                  H0(i) = H0(i) - Hxdif(i,k)*Hxdif(i,k)*ptHp(k)
     $                          +  gdif(i,k)* gdif(i,k)* yts(k)
  100          continue
  110       continue

            do 120, i = 1, nnL
               H0(i) = H0(i) - Hp(i)*Hp(i)*pHpi + y(i)*y(i)*ysi
  120       continue

            if (nQNmod .eq. nFlush) then
               nFlush = - 1
               Reset  = 'f'
            else
               Reset  = 'R'
            end if
            nQNmod = 0
            nSkip  = 0
         else
*           ------------------------------------------------------------
*           Space remains. Store Hp and y.
*           ------------------------------------------------------------
            nQNmod = nQNmod + 1
            nSkip  = 0
            call dcopy ( nnL, Hp, 1, Hxdif(1,nQNmod), 1 )
            call dcopy ( nnL, y , 1,  gdif(1,nQNmod), 1 )
            yts (nQNmod) = ysi
            ptHp(nQNmod) = pHpi
         end if ! nQNmod ge mQNmod
      else
*        ---------------------------------------------------------------
*        The update will be skipped.
*        If there are nonlinear constraints, reset H if the update was
*        skipped more than mSkip times.
*        ---------------------------------------------------------------
         nSkip  = nSkip  + 1            

         if (nnCon .gt. 0  .and.  nSkip .gt. mSkip) then
            if (nQNmod .eq. 0) then

*              No successful update since the last reset.
*              Reset H0 to a multiple of the identity.

               H0diag = one
               call dload ( nnL, H0diag, H0, 1 )
            else
               do 210, k = 1,  min(mQNmod, nQNmod)
                  do 200, i = 1, nnL
                     H0(i) = H0(i) - Hxdif(i,k)*Hxdif(i,k)*ptHp(k)
     $                             +  gdif(i,k)* gdif(i,k)* yts(k)
  200             continue
  210          continue
               nQNmod = 0
               Reset  = 'r'
            end if ! nQNmod = 0
            nSkip  = 0
         end if ! nnCon > 0
      end if ! yp ge ypmin

*     end of s8LMsv
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8LMHx( nnL, x, Hx, mQNmod, nQNmod,
     $                   H0, gdif, Hxdif, yts, ptHp )

      implicit           double precision (a-h,o-z)
      double precision   Hx(nnL), x(nnL)
      double precision   H0(nnL), yts(mQNmod), ptHp(mQNmod)
      double precision   gdif(nnL,mQNmod), Hxdif(nnL,mQNmod)

*     ==================================================================
*     s8LMHx does the work for s8Hx.
*
*     19-Jul-95: First version of s8LMHx
*     15-Jan-96: Current version
*     ==================================================================
      call dcopy ( nnL,  x, 1, Hx, 1 )
      call ddscl ( nnL, H0, 1, Hx, 1 )

      if (nQNmod .gt. 0) then
         do 100, k = 1, nQNmod
            c1  = - ddot ( nnL, Hxdif(1,k), 1, x, 1 )*ptHp(k)
            call daxpy ( nnL, c1, Hxdif(1,k), 1, Hx, 1 )

            c2  =   ddot ( nnL,  gdif(1,k), 1, x, 1 )*yts(k)
            call daxpy ( nnL, c2, gdif (1,k), 1, Hx, 1 )
  100    continue
      end if

*     end of s8LMHx
      end
