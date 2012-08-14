*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
*     File  sn95fmqn.f.   Full memory BFGS routines.
*
*     s8FMmp   s8FMH0   s8FMsv   s8FMHx
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8FMmp( nnL, minrw, iw, leniw )

      implicit           double precision (a-h,o-z)
      integer            iw(leniw)

*     ==================================================================
*     s8FMmp computes the addresses of the quasi-Newton arrays.
*     These are saved and used for subsequent entries.
*
*     This version defines a full-memory BFGS form of H.
*
*     30-Dec-91: First version of s8FMmp.
*     19-Jul-97: Current version.
*     ==================================================================

      lenH    = nnL*(nnL + 1)/2
      lH      = minrw
      minrw   = lH    + lenH

      iw(371) = lH
      iw(372) = lenH

*     end of s8FMmp
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8FMH0( Reset, nnL, nQNmod,
     $                   mFlush, nFlush, nSkip, H, lenH )

      implicit           double precision (a-h,o-z)
      character*1        Reset
      double precision   H(lenH)

*     ==================================================================
*     s8FMH0 initializes the approximate Hessian.
*
*     The Hessian is flushed once at iteration mFlush
*
*     19-Jul-95: First version of s8FMH0 written by PEG.
*     19-Jul-97: Current version.
*     ==================================================================
      parameter         (zero = 0.0d+0, one = 1.0d+0)
*     ------------------------------------------------------------------
*     In this version we set  H to the identity.

      nQNmod    = 0
      nFlush    = mFlush
      nSkip     = 0

      Reset     = 'R'
      H0diag    = one

      l         = 1
      incr      = nnL
      nzero     = nnL - 1

      do 100, k = 1, nnL-1
         H(l)   = H0diag
         call dload ( nzero, zero, H(l+1), 1 )
         l      = l     + incr
         incr   = incr  - 1
         nzero  = nzero - 1
  100 continue
      H(l) = H0diag

*     end of s8FMH0
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8FMsv( QNmsg, Reset,
     $                   lenH, nnL, nnCon, mQNmod, nQNmod, 
     $                   mSkip, nSkip, nFlush, H0scal,
     $                   step, yp, ypmin, pHp, H, Hp, y )

      implicit           double precision (a-h,o-z)
      character*1        QNmsg, Reset
      double precision   H(lenH), Hp(nnL), y(nnL)

*     ==================================================================
*     s8FMsv stores the update (Hp,y) from this iteration.
*     If defined, the self-scaling BFGS update parameter is saved.
*     It is needed to update the reduced Hessian when there are only
*     linear constraints.
*
*     19-Jul-95: First version of s8FMsv written by PEG.
*     19-Jul-97: Current version.
*     ==================================================================
      parameter         (zero = 0.0d+0, one = 1.0d+0)
*     ------------------------------------------------------------------

      if (nQNmod .ge. mQNmod  .or. nQNmod .eq. nFlush) then
*        ---------------------------------------------------
*        Nope, the user wants to reset H to the identity.
*        ---------------------------------------------------
         if (nQNmod .eq. nFlush) then
            nFlush = - 1
            Reset  = 'f'
         else
            Reset  = 'R'
         end if
         nQNmod = 0
         nSkip  = 0
         H0diag = one

         l     = 1
         incr  = nnL
         nzero = nnL - 1

         do 200, k = 1, nnL-1
            H(l)  = H0diag
            call dload ( nzero, zero, H(l+1), 1 )
            l     = l     + incr
            incr  = incr  - 1
            nzero = nzero - 1
  200    continue
         H(l) = H0diag

      else if (yp .ge. ypmin  .and.  pHp .gt. zero) then
*        ---------------------------------------------------
*        Apply the update to H.
*        ---------------------------------------------------
         ys    = step*yp
         ysi   = one / ys
         pHpi  = one / pHp

         if (nQNmod .eq. 0) then
            if (QNmsg .eq. ' ') then
               QNmsg = 's'
            else
               QNmsg = 'S'
            end if
            sHs    = pHp*step*step
            H0scal = ys / sHs
            call dscal ( nnL, H0scal, Hp, 1 )
            pHp    = pHp  * H0scal
            pHpi   = pHpi / H0scal

            l      = 1
            incr   = nnL

            do 210, k = 1, nnL
               H(l)   = H(l)*H0scal
               l      = l    + incr
               incr   = incr - 1
  210       continue
         end if

         l     = 0
         do 230, i = 1, nnL
            c1 = Hp(i)*pHpi
            c2 = y(i) *ysi
            do 220, j = i, nnL
               l      = l + 1
               H(l)   = H(l) - c1*Hp(j) + c2*y(j)
  220       continue
  230    continue
         nQNmod = nQNmod + 1
         nSkip  = 0

      else if (yp .lt. ypmin  .or.  pHp .le. zero) then
*        ------------------------------------------------------------
*        The update was skipped.
*        No problem if there are only linear constraints.
*        Reset H if the update was skipped more than mSkip times.
*        ------------------------------------------------------------
         nSkip  = nSkip  + 1            

         if (nnCon .gt. 0  .and.  nSkip .gt. mSkip) then
            nSkip  = 0

            if (nQNmod .gt. 0) then
*              ------------------------------------------------------
*              Zero the off-diagonal elements H.
*              ------------------------------------------------------
               l     = 1
               incr  = nnL
               nzero = nnL - 1

               do 240, k = 1, nnL-1
                  call dload ( nzero, zero, H(l+1), 1 )
                  l      = l     + incr
                  incr   = incr  - 1
                  nzero  = nzero - 1
  240          continue

               nQNmod = 0
               Reset  = 'r'
            else
*              ------------------------------------------------------
*              Still no successful update since the last reset.
*              Reset H0 to a multiple of the identity.
*              ------------------------------------------------------
               H0diag = one

               l     = 1
               incr  = nnL
               nzero = nnL - 1

               do 250, k = 1, nnL-1
                  H(l)   = H0diag
                  call dload ( nzero, zero, H(l+1), 1 )
                  l      = l     + incr
                  incr   = incr  - 1
                  nzero  = nzero - 1
  250          continue
               H(l) = H0diag
            end if ! nQNmod = 0
         end if
      end if

*     end of s8FMsv
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s8FMHx( n, lenH, H, x, Hx )

      implicit           double precision (a-h,o-z)
      double precision   H(lenH), Hx(n), x(n)

*     ==================================================================
*     s8FMHx  computes the product Hx, where the symmetric part of H is
*     stored by rows in the one-dimensional array  H.  Note that
*     lenH is used to define the length of H,  and must
*     be at least n*(n + 1)/2. 
*
*     12-Jan-96: First version of s8FMHx
*     12-Jan-96: Current version.
*     ==================================================================
      parameter         (zero = 0.0d+0)
*     ------------------------------------------------------------------

      l = 0
      do 110, i = 1, n
         s     = zero
         do 100, j = i, n
            l     = l + 1
            s     = s + H(l)*x(j)
  100    continue
         Hx(i) = s
  110 continue
      
      l = 0
      do 210, j = 1, n-1
         xj  = x(j)
         l   = l + 1
         do 200, i = j+1, n
            l      = l + 1
            Hx(i)  = Hx(i) + H(l)*xj
  200    continue
  210 continue

*     end of s8FMHx
      end

