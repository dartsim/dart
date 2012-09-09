*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
*     file  sn60srch.f
*
*     s6fd     s6dcon   s6dobj   s6init   s6srch
*     lsrchc   lsrchq
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s6fd  ( ierror,
     $                   n, neJac, nnCon, nnCon0, nnJac, nnObj, 
     $                   sfgCon, fgCon, sfgObj, fgObj,
     $                   ne, nka, ha, ka, fObj, x, w, 
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      external           sfgCon, fgCon, sfgObj, fgObj
      integer            ha(ne)
      integer            ka(nka)
      double precision   x(n), w(nnCon0)

      character*8        cu(lencu), cw(lencw)
      integer            iu(leniu), iw(leniw)
      double precision   ru(lenru), rw(lenrw)

*     ==================================================================
*     s6fd    computes any missing objective and constraint gradients 
*     for the current value of the variables in  x.
*
*     NOTE --  s6dcon overwrites the first  nnCon  elements of  y
*     if central differences are needed.
*
*     30-Dec-91: First version based on Minos 5.4 routine m6grd.
*     17-Jul-97: First thread-safe version.
*     07-Feb-98: Current version.
*     ==================================================================
      nConfd    = iw(183)
      nObjfd    = iw(184)
      lgObj     = iw(298)
      lfCon     = iw(302)
      lfCon2    = iw(304)
      lgConU    = iw(306)
      lgCon     = iw(305)
      lgObjU    = iw(299)

      if (nConfd .gt. 0) then
         call s6dcon( ierror, n, nnCon, nnJac, neJac,
     $                sfgCon, fgCon,
     $                ne, nka, ha, ka,
     $                rw(lfCon), rw(lfCon2), rw(lgCon), rw(lgConU),
     $                x, w, 
     $                cu, lencu, iu, leniu, ru, lenru, 
     $                cw, lencw, iw, leniw, rw, lenrw )
         if (ierror .ne. 0) return
      end if

      if (nObjfd .gt. 0) then
         call s6dobj( ierror, nnObj, sfgObj, fgObj,
     $                fObj, rw(lgObj), rw(lgObjU), x, 
     $                cu, lencu, iu, leniu, ru, lenru, 
     $                cw, lencw, iw, leniw, rw, lenrw )
         if (ierror .ne. 0) return
      end if

*     end of s6fd 
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s6dcon( ierror, n, nnCon, nnJac, neJac, 
     $                   sfgCon, fgCon,
     $                   ne, nka, ha, ka,
     $                   fCon, fCon2, gCon, gConU, x, y, 
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      external           sfgCon, fgCon
      integer            ha(ne)
      integer            ka(nka)
      double precision   fCon(nnCon), fCon2(nnCon),
     $                   gCon(neJac), gConU(neJac),
     $                   x(nnJac), y(nnCon)

      character*8        cu(lencu), cw(lencw)
      integer            iu(leniu), iw(leniw)
      double precision   ru(lenru), rw(lenrw)

*     ==================================================================
*     s6dcon  estimates missing elements in the columns of the
*     Jacobian (the first  nnCon  rows and  nnJac  cols of  A)
*     using finite differences of the constraint functions  fCon.
*
*     The array fCon2 is used as workspace.
*     The array gConU defines the unknown derivatives of fCon.
*
*     09-Jan-92: First version based on Minos 5.4 routine m6dcon.
*     17-Jul-97: Thread-safe version.
*     19-Mar-97: Current version.
*     ==================================================================
      double precision   fdint(2)
      logical            centrl
      parameter         (one   = 1.0d+0)
      parameter         (nfCon = 189)
*     ------------------------------------------------------------------
      lvlDif    = iw(182)
      gdummy    = rw( 69)
      fdint(1)  = rw( 76)
      fdint(2)  = rw( 77)

      nState    = 0
      centrl    = lvlDif .eq. 2
      delta     = fdint(lvlDif)
      numf      = 0
      l         = 0

*     If central differences are needed, we first save  fCon  in  y.
*     When  fCon  is later loaded with  f(x - h)  for each pertbn  h,
*     it can be used the same as when forward differences are in effect.

      if (centrl) call dcopy ( nnCon, fCon, 1, y, 1 )

      do 200, j = 1, nnJac

*        Look for the first missing element in this column.

         lsave  = l
         k1     = ka(j)
         k2     = ka(j+1) - 1
         do 100, k = k1, k2
            ir     = ha(k)
            if (ir .gt. nnCon) go to 200
            l      = l + 1
            if (gConU(l) .eq. gdummy) go to 120
  100    continue
         go to 200

*        Found one. Approximate it and any others by finite differences.

  120    l      = lsave
         xj     = x(j)
         dx     = delta*(one + abs( xj ))
         x(j)   = xj   + dx
         numf   = numf + 1
         call sfgCon( 0, ierror, nState,
     $                n, nnCon, nnJac, neJac, fgCon,
     $                ne, nka, ha, ka,
     $                fCon2, gConU, x,
     $                cu, lencu, iu, leniu, ru, lenru, 
     $                cw, lencw, iw, leniw, rw, lenrw )
         if (ierror .ne. 0) go to 900

         if (centrl) then
            x(j)   = xj - dx
            dx     = dx + dx
            numf   = numf + 1
            call sfgCon( 0, ierror, nState,
     $                   n, nnCon, nnJac, neJac, fgCon,
     $                   ne, nka, ha, ka,
     $                   fCon, gConU, x,
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )
            if (ierror .ne. 0) go to 900
         end if

         do 150, k = k1, k2
            ir     = ha(k)
            if (ir .gt. nnCon) go to 190
            l      = l + 1
            if (gConU(l) .eq. gdummy) then
                gCon(l) = (fCon2(ir) - fCon(ir)) / dx
            end if
  150    continue
  190    x(j)   = xj
  200 continue

  900 if (centrl) call dcopy ( nnCon, y, 1, fCon, 1 )
      l           = lvlDif      + 1
      iw(nfCon+l) = iw(nfCon+l) + numf

*     end of s6dcon
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s6dobj( ierror, nnObj, sfgObj, fgObj,
     $                   fObj, gObj, gObjU, x, 
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      external           sfgObj, fgObj
      double precision   gObj(nnObj), gObjU(nnObj)
      double precision   x(nnObj)

      character*8        cu(lencu), cw(lencw)
      integer            iu(leniu), iw(leniw)
      double precision   ru(lenru), rw(lenrw)

*     ==================================================================
*     s6dobj  estimates missing objective gradients
*     using finite differences of the objective function  fObj.
*
*     The array gObjU is used as workspace.
*     The elements of gObj2 associated with unknown derivatives
*     must contain the standard dummy element gdummy. 
*
*     09-Jan-92: First version based on Minos 5.4 routine m6dobj.
*     17-Jul-97: Thread-safe version.
*     22-Jul-97: Current version.
*     ==================================================================
      double precision   fdint(2)
      logical            centrl
      parameter         (one   = 1.0d+0)
      parameter         (nfObj = 194)
*     ------------------------------------------------------------------
      lvlDif    = iw(182)
      gdummy    = rw( 69)

      fdint(1)  = rw( 76)
      fdint(2)  = rw( 77)

      nState    = 0
      centrl    = lvlDif .eq. 2
      delta     = fdint(lvlDif)
      numf      = 0
      fback     = fObj

      do 200, j  = 1, nnObj
         if (gObjU(j) .eq. gdummy) then
            xj     = x(j)
            dx     = delta*(one + abs( xj ))
            x(j)   = xj + dx
            numf   = numf + 1
            call sfgObj( 0, ierror, nState, nnObj, 
     $                   fgObj,
     $                   fforwd, gObjU, x, 
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )
            if (ierror .ne. 0) go to 900

            if (centrl) then
               x(j)   = xj - dx
               dx     = dx + dx
               numf   = numf + 1
               call sfgObj( 0, ierror, nState, nnObj, 
     $                      fgObj,
     $                      fback, gObjU, x, 
     $                      cu, lencu, iu, leniu, ru, lenru, 
     $                      cw, lencw, iw, leniw, rw, lenrw )
               if (ierror .ne. 0) go to 900
            end if

            gObj(j) = (fforwd - fback) / dx
            x(j)    = xj
         end if
  200 continue

  900 l           = lvlDif      + 1
      iw(nfObj+l) = iw(nfObj+l) + numf

*     end of s6dobj
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s6init( ierror, n, 
     $                   nnCon, nnCon0, nnJac, neJac,
     $                   nnL, nnObj, nnObj0,
     $                   sfgCon, fgCon, sfgObj, fgObj,
     $                   ne, nka, ha, ka,
     $                   fObj, fCon, gCon, gObj, x,
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      external           sfgCon, fgCon, sfgObj, fgObj
      integer            ha(ne)
      integer            ka(nka)
      double precision   fCon(nnCon0), gCon(neJac)
      double precision   gObj(nnObj0)
      double precision   x(nnL)

      character*8        cu(lencu), cw(lencw)
      integer            iu(leniu), iw(leniw)
      double precision   ru(lenru), rw(lenrw)

*     ==================================================================
*     s6init  computes the objective and constraint functions at the
*     first point that satisfies the linear constraints.
*
*     22-Dec-94: First version of s6init based on NPSOL routine npchkd.
*     17-Jul-97: Thread-safe version.
*     07-Feb-98: Current version.
*     ==================================================================
      parameter         (lvlDer    =  71)
      parameter         (nConfd    = 183)
      parameter         (nObjfd    = 184)
      parameter         (nGotg1    = 186)
      parameter         (nGotg2    = 187)
*     ------------------------------------------------------------------
      iPrint     = iw( 12)
      iSumm      = iw( 13)
      gdummy     = rw( 69)

      lgConU     = iw(306)
      lgObjU     = iw(299)

      iw(nConfd) =  0
      iw(nObjfd) =  0
      iw(nGotg1) =  0
      iw(nGotg2) =  0
      modefg     =  2
 
      ierror     = -1
      nState     =  1
      if (iPrint .gt. 0) write(iPrint, 1000)
      if (iSumm  .gt. 0) write(iSumm , 1000)

*     ==================================================================
*     Evaluate the problem functions at x.
*     ==================================================================
      if (nnCon .gt. 0) then
         mode     = modefg
         call dload ( neJac, gdummy, gCon, 1 )
         call sfgCon( mode, ierror, nState,
     $                n, nnCon, nnJac, neJac, fgCon,
     $                ne, nka, ha, ka,
     $                fCon, gCon, x,
     $                cu, lencu, iu, leniu, ru, lenru, 
     $                cw, lencw, iw, leniw, rw, lenrw )
      end if

      if (nnObj .gt. 0 .and. (ierror .eq. 0 .or. nnCon .eq. 0)) then
         mode     = modefg
         call dload ( nnObj, gdummy, gObj, 1 )
         call sfgObj( modefg, ierror, nState, nnObj,
     $                fgObj,
     $                fObj, gObj, x, 
     $                cu, lencu, iu, leniu, ru, lenru, 
     $                cw, lencw, iw, leniw, rw, lenrw )
      end if
      
      if (ierror .ne. 0) go to 900

*     ==================================================================
*     Satisfactory problem functions have been defined
*     ==================================================================
      nState = 0

      if (nnCon .gt. 0) then
*        ---------------------------------------------------------------
*        Count how many Jacobian elements are provided.
*        ---------------------------------------------------------------
         ngrad     = 0
         do 100, l = 1, neJac
            if (gCon(l) .ne. gdummy) ngrad  = ngrad + 1
  100    continue

         iw(nGotg2) = ngrad
         if (iPrint .gt. 0) then
            write(iPrint, 1100) ngrad, neJac
         end if
         if (iSumm  .gt. 0) then
            write(iSumm , 1100) ngrad, neJac
         end if

         if (ngrad .lt. neJac) then

*           Some Jacobian elements are missing.
 
            if (iw(lvlDer) .ge. 2) then
*              ---------------------------------------------------------
*              The Jacobian is supposedly known. Any missing elements
*              are assumed to be constant, and must be restored. 
*              They have been extracted from the MPS file by
*              s8aux( 1, ... ) and saved in gConU.  Make another call
*              to funcon to reset the non-constant elements.
*              ---------------------------------------------------------
               call dcopy ( neJac, rw(lgConU), 1, gCon, 1 )
               call sfgCon( mode, ierror, nState,
     $                      n, nnCon, nnJac, neJac, fgCon,
     $                      ne, nka, ha, ka,
     $                      fCon, gCon, x,
     $                      cu, lencu, iu, leniu, ru, lenru, 
     $                      cw, lencw, iw, leniw, rw, lenrw )
               if (ierror .ne. 0) go to 900
            else
*              ---------------------------------------------------------
*              Copy gCon into gCon2 so that we have a permanent record
*              of which derivatives need to be estimated.
*              ---------------------------------------------------------
               call dcopy ( neJac, gCon, 1, rw(lgConU), 1 )
            end if
         end if ! ngrad < neJac
         if (iw(lvlDer) .lt. 2) iw(nConfd) = neJac - ngrad
      end if

      if (nnObj .gt. 0) then
*        ---------------------------------------------------------------
*        Count how many gradient elements are known.
*        ---------------------------------------------------------------
         ngrad  = 0
         do 200, l = 1, nnObj
            if (gObj(l) .ne. gdummy) ngrad = ngrad + 1
  200    continue
         iw(nGotg1) = ngrad

         if (iPrint .gt. 0) write(iPrint, 2000) ngrad, nnObj
         if (iSumm  .gt. 0) write(iSumm , 2000) ngrad, nnObj

         if (ngrad .lt. nnObj) then

*           Some objective gradients are missing.

            if (iw(lvlDer) .eq. 1  .or.  iw(lvlDer) .eq. 3) then
*              ---------------------------------------------------------
*              The objective gradient was meant to be known.
*              ---------------------------------------------------------
               iw(lvlDer) = iw(lvlDer) - 1
               if (iPrint .gt. 0) write(iPrint, 2100) iw(lvlDer)
               if (iSumm  .gt. 0) write(iSumm , 2100) iw(lvlDer)
            else
*              ---------------------------------------------------------
*              Copy gObj into gObjU.
*              ---------------------------------------------------------
               call dcopy ( nnObj, gObj, 1, rw(lgObjU), 1 )
            end if
            iw(nObjfd) = nnObj - ngrad
         end if
      end if

*     ------------------------------------------------------------------
*     Done
*     ------------------------------------------------------------------
  900 return

 1000 format(/)
 1100 format(  ' funcon  sets', i8, '   out of', i8,
     $         '   constraint gradients.')
 2000 format(  ' funobj  sets', i8, '   out of', i8,
     $         '   objective  gradients.')
 2100 format(/ ' XXX  Some objective derivatives are missing ---',
     $         ' derivative level reduced to', i3)

*     end of s6init
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s6srch( inform, debug, Elastc, fonly, minimz,
     $                   n, nb, nnCon, nnCon0, nnObj, nnObj0, iObj, itn,
     $                   sfgCon, fgCon, sfgObj, fgObj,
     $                   ne   , nka, a, ha, ka,
     $                   neJac, nkg,        kg, 
     $                   fMrt, gMrt, gInf, sInf, sInf2, wtInf, 
     $                   step, stepmn, stepmx, pnorm, xnorm, fObj,
     $                   fCon, gCon, gObj, tfCon, tgCon, tgObj,
     $                   xs, x2, p, xMul, xMul2, xdMul, xPen,
     $                   w, y, y2,
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      external           sfgCon, fgCon, sfgObj, fgObj
      logical            debug, Elastc, fonly
      integer            ha(ne)
      integer            ka(nka), kg(nkg)
      double precision   a(ne)
      double precision   fCon(nnCon0), gCon(neJac), gObj(nnObj0)
      double precision   tfCon(nnCon0), tgCon(neJac), tgObj(nnObj0)
      double precision   xs(nb), x2(nb), p(nb)
      double precision   xMul(nnCon0), xMul2(nnCon0), xdMul(nnCon0)
      double precision   xPen(nnCon0)
      double precision   w(nnCon0), y(nnCon0), y2(nnCon0)

      character*8        cu(lencu), cw(lencw)
      integer            iu(leniu), iw(leniw)
      double precision   ru(lenru), rw(lenrw)

*     ==================================================================
*     s6srch  finds a step along the search direction  p,  such that
*     the function  fMrt  is sufficiently reduced, i.e.,
*               fMrt(xs + step*p)  <  fMrt(xs).
*
*     Input parameters...
*
*     nb          The number of line search variables.
*     step        Initial estimate of the step.
*     stepmx      Maximum allowable step.
*     eps         The unit round-off.
*     epsrf       Relative function precision.
*     eta         Line search accuracy parameter in the range (0,1).
*                 0.001 means very accurate.   0.99 means quite sloppy.
*     fonly       true if function-only search should be used.
*     gMrt        The directional derivative of the merit function.
*     fMrt        Current value of fMrt(x).
*     p(nb)       The search direction.
*     xs(nb)      The base point for the search.
*     x2(nb)      x2 = xs + p,  the QP solution.
*
*     Output parameters...
*
*     step        The final step.
*     fMrt        The final value of fMrt.
*     x2          Final value of the variables.
*     inform = -1 if the user wants to stop.
*            =  1 to 8 -- see below.
*
*     30-Dec-91: First version based on NPSOL 4.6 routine npsrch.
*     28-Sep-93: Allow functions to say "undefined at this point".
*                (Back up and try again.)
*     18-Feb-94: Back up in a similar way if vimax increases a lot.
*                Deleted after first limited-memory version.
*     29-Dec-94: Merit function calculations included explicitly.
*     06-Apr-96: Special coding for the unit step.
*                On entry, x2 is the QP solution.
*     18-Oct-96: First Min-sum version.
*     17-Jul-97: First thread-safe version.
*     31-Dec-97: Current version.
*     ==================================================================
      logical            gknown, nlnCon, nlnObj
      logical            done  , first , imprvd
      logical            braktd, crampd, extrap, moved , vset  , wset
      parameter         (zero = 0.0d+0, half = 0.5d+0, one = 1.0d+0)
      double precision   mu
      parameter         (mu   = 1.0d-4)
*     ------------------------------------------------------------------
      iPrint    = iw( 12)
      iSumm     = iw( 13)

      nnJac     = iw( 22)

      eps       = rw(  1)
      eps0      = rw(  2)
      bigfx     = rw( 71)
      epsrf     = rw( 73)
      eta       = rw( 84)
      sclObj    = rw(188)

*     ------------------------------------------------------------------
*     Set the input parameters for lsrchc or lsrchq.
*
*     stepmn  is used by lsrchq.  If  step  would be less than  stepmn,
*             the search will be terminated early.
*             If p was found using forward or backward differences,
*             stepmn  should be positive (and related to the difference
*             interval used).
*             If p was found using central differences (lvlDif = 2)
*             stepmn  should be zero.
*
*     epsrf   is the relative function precision (specified by user).
*
*     epsaf   is the absolute function precision. If f(x1) and f(x2) are
*             as close as  epsaf,  we cannot safely conclude from the
*             function values alone which of x1 or x2 is a better point.
*
*     tolabs  is an estimate of the absolute spacing between points
*             along  p.  This step should produce a perturbation
*             of  epsaf  in the merit function.
*
*     tolrel  is an estimate of the relative spacing between points
*             along  p.
*
*     toltny  is the minimum allowable absolute spacing between points
*             along  p.
*     ------------------------------------------------------------------
      ierror = 0
      stpmax = stepmx
      gknown = .not. fonly

      if ( fonly ) then
         maxf   = 15
         modefg =  0
      else
         maxf   = 10
         modefg =  2
      end if

      nlin   = n - nnJac
      nnJ1   = nnJac + 1
      nout   = iPrint
      nState = 0
      nlnCon = nnCon .gt. 0
      nlnObj = nnObj .gt. 0

      epsaf  = max( epsrf, eps )*(one + abs( fMrt ))
      oldf   = fMrt
      oldg   = gMrt
      sgnObj = minimz
      tolax  = eps0
      tolrx  = eps0
      t      = xnorm*tolrx  +  tolax
      if (t .lt. pnorm*stepmx) then
         tolabs = t/pnorm
      else
         tolabs = stepmx
      end if
      tolrel = max( tolrx, eps )

      t      = zero
      do 10, j = 1, nb
         s     = abs( p(j))
         q     = abs(xs(j))*tolrx + tolax
         if (s .gt. q*t) t = s / q
   10 continue

      if (t*tolabs .gt. one) then
         toltny = one / t
      else
         toltny = tolabs
      end if

*     Line search starts (or restarts) here.

  100 first  = .true.
      sbest  = zero
      fbest  = zero
      gbest  = (one - mu )*oldg
      targtg = (mu  - eta)*oldg
      g0     = gbest

      if (debug) write(nout, 1000) itn, pnorm

*     ------------------------------------------------------------------
*     Commence main loop, entering lsrchc or lsrchq two or more times.
*     first = true for the first entry,  false for subsequent entries.
*     done  = true indicates termination, in which case the value of
*     inform gives the result of the search.
*     inform = 1 if the search is successful and step < stpmax.
*            = 2 if the search is successful and step = stpmax.
*            = 3 if a better point was found but too many functions
*                were needed (not sufficient decrease).
*            = 4 if stpmax < tolabs (too small to do a search).
*            = 5 if step   < stepmn (lsrchq only -- maybe want to switch
*                to central differences to get a better direction).
*            = 6 if the search found that there is no useful step.
*                The interval of uncertainty is less than 2*tolabs.
*                The minimizer is very close to step = zero
*                or the gradients are not sufficiently accurate.
*            = 7 if there were too many function calls.
*            = 8 if the input parameters were bad
*                (stpmax le toltny  or  oldg ge 0).
*            = 9 if the objective is unbounded below.
*     ------------------------------------------------------------------
*+    repeat
  200    if ( gknown ) then
            call lsrchc( first , debug , done  , imprvd, inform,
     $                   maxf  , numf  , nout  ,
     $                   stpmax,         epsaf , 
     $                   g0    , targtg, ftry  , gtry  , 
     $                   tolabs, tolrel, toltny,
     $                   step  , sbest , fbest , gbest ,
     $                   braktd, crampd, extrap, moved , wset  ,
     $                   nsamea, nsameb,
     $                   aLow  , bUpp  , factor,
     $                   xtry  , xw    , fw    , gw    , tolmax )
         else
            call lsrchq( first , debug , done  , imprvd, inform,
     $                   maxf  , numf  , nout  , 
     $                   stpmax, stepmn, epsaf , 
     $                   g0    , targtg, ftry  ,         
     $                   tolabs, tolrel, toltny,
     $                   step  , sbest , fbest ,
     $                   braktd, crampd, extrap, moved , vset  , wset  ,
     $                   nsamea, nsameb,
     $                   aLow  , bUpp  , fa    , factor,
     $                   xtry  , xw    , fw    , xv    , fv    , tolmax)
         end if

         if ( imprvd ) then
 
            if (nlnObj ) then
               if (abs(tfObj) .gt. bigfx) then
                  ierror = -1
                  go to 900
               end if

               fObj = tfObj
               if ( gknown ) call dcopy ( nnObj, tgObj, 1, gObj, 1 )
            end if

            fMrt = tfMrt
 
            if ( nlnCon ) then
               call dcopy ( nnCon, tfCon, 1, fCon, 1 )
               if ( Elastc ) sInf2 = tsInf
               if ( gknown ) call dcopy ( neJac, tgCon, 1, gCon, 1 )
            end if
         end if ! imprvd

*        ---------------------------------------------------------------
*           done = false   first time through.
*        If done = false,  the problem functions must be computed for
*        the next entry to lsrchc or lsrchq.
*        If done = true,   this is the last time through.
*        ---------------------------------------------------------------
         if (.not. done) then
            if (step .ne. one) then
               call dcopy ( nb,       xs, 1, x2, 1 )
               call daxpy ( nb, step, p , 1, x2, 1 )
            end if

            if ( nlnCon ) then
               call sfgCon( modefg, ierror, nState,
     $                      n, nnCon, nnJac, neJac, fgCon,
     $                      ne, nka, ha, ka,
     $                      tfCon, tgCon, x2,
     $                      cu, lencu, iu, leniu, ru, lenru, 
     $                      cw, lencw, iw, leniw, rw, lenrw )
               if (ierror .ne. 0) go to 900
            end if

            if (iObj .eq. 0) then
               tfMrt = zero
            else
               tfMrt = sgnObj*x2(n+iObj)*sclObj
            end if

            if ( nlnObj ) then
               call sfgObj( modefg, ierror, nState, nnObj, 
     $                      fgObj,
     $                      tfObj, tgObj, x2,
     $                      cu, lencu, iu, leniu, ru, lenru, 
     $                      cw, lencw, iw, leniw, rw, lenrw )
               if (ierror  .ne. 0) go to 900
               tfMrt = tfMrt + sgnObj*tfObj
            end if

            if ( nlnCon ) then
*              ---------------------------------------------------------
*              Compute w and y, the constraint violations and the 
*              negative of the gradient of the merit function with 
*              respect to the nonlinear slacks. These quantities define
*              the directional derivative of the merit function. 
*              Finally, add the nonlinear terms to the merit function.
*              ---------------------------------------------------------
               if (step .le. one) then
                  call dcopy ( nnCon,       xMul , 1, xMul2, 1 )
                  call daxpy ( nnCon, step, xdMul, 1, xMul2, 1 )
               end if

*              Compute the constraint violations and aux. multipliers:
*              viol = fCon + A(linear) x - nonlinear slacks.
*              y    = xMul2 - xPen*viol. 

               call dcopy ( nnCon,           tfCon, 1, w, 1 )
               call daxpy ( nnCon, (-one), x2(n+1), 1, w, 1 )
               if (nlin .gt. 0) then
                  call s2Aprd( 'Normal', eps0, 
     $                         ne, nlin+1, a, ha, ka(nnJ1), 
     $                         one, x2(nnJ1), nlin, one, w, nnCon )
               end if

               call dcopy ( nnCon,    w, 1, y, 1 )
               call ddscl ( nnCon, xPen, 1, y, 1 )
               tfMrt = tfMrt -      ddot( nnCon, xMul2, 1, w, 1 )
     $                       + half*ddot( nnCon, y    , 1, w, 1 )

*              If we are in elastic mode, include the contribution from
*              the violations of the elastic variables.

               if ( Elastc ) then
                  tsInf = sInf  +  step*gInf 
                  tfMrt = tfMrt + wtInf*tsInf
               end if

               if ( gknown ) then
                  call daxpy ( nnCon, (-one), xMul2 , 1, y, 1 )
                  call dscal ( nnCon, (-one), y     , 1 )
               end if
            end if ! nlnCon

            ftry  = tfMrt - oldf - mu*oldg*step 

            if ( gknown ) then
*              ---------------------------------------------------------
*              A gradient search is requested.
*              Compute the directional derivative gtry.
*              ---------------------------------------------------------
               if (iObj .eq. 0) then
                  tgMrt = zero
               else
                  tgMrt = sgnObj*p(n+iObj)*sclObj
               end if

               if ( nlnObj ) then
                  tgMrt = tgMrt + sgnObj*ddot( nnObj, tgObj, 1, p, 1 )
               end if

               if ( nlnCon ) then
*                 ------------------------------------------------------
*                 Form  Jp (including linear columns).
*                 Set  y2 = Jp + A(linear) p - p(slacks).
*                 ------------------------------------------------------
                  call s8Jprd( 'No transpose', eps0, 
     $                         ne, nka, ha, ka,
     $                         neJac, nkg, tgCon, kg, 
     $                         one, p, nnJac, zero, y2, nnCon )

                  if (nlin .gt. 0) then
                     call s2Aprd( 'Normal', eps0,
     $                            ne, nlin+1, a, ha, ka(nnJ1), 
     $                            one, p(nnJ1), nlin, one, y2, nnCon )
                  end if
                  call daxpy ( nnCon, (-one), p(n+1), 1, y2, 1 )

                  tgMrt = tgMrt - ddot  ( nnCon, y, 1, y2   , 1 )
     $                          - ddot  ( nnCon, w, 1, xdMul, 1 )

*                 If we are elastic mode, include the contribution from
*                 the violations of the elastic variables.

                  if ( Elastc ) tgMrt = tgMrt + wtInf*gInf

               end if ! nlnCon

               gtry   = tgMrt - mu*oldg

            end if ! gknown
         end if ! not done
*+    until (      done)
      if    (.not. done) go to 200

*     ==================================================================
*     The search has finished.
*     ==================================================================
  900 if (ierror .ne.  0) then
*        ---------------------------------------------------------------
*        sfgCon or sfgObj set ierror ne 0 (mode < 0 in funobj/funcon).
*        ierror = -1 means "undefined function at current step".
*        We start the whole search again with a smaller stpmax and step.
*        (The search routines will say if this is too small.)
*        ---------------------------------------------------------------
         if (ierror .eq. -1) then

*           The problem functions are undefined.
*           Reset x2 and the problem functions at the base point.

            ierror = 0
            if ( nlnCon ) then
               call sfgCon( modefg, ierror, nState,
     $                      n, nnCon, nnJac, neJac, fgCon,
     $                      ne, nka, ha, ka,
     $                      fCon, gCon, xs,
     $                      cu, lencu, iu, leniu, ru, lenru, 
     $                      cw, lencw, iw, leniw, rw, lenrw )
               if (ierror .ne. 0) then
                  inform = -1
                  return
               end if
            end if

            if ( nlnObj ) then
               call sfgObj( modefg, ierror, nState, nnObj, 
     $                      fgObj,
     $                      fObj, gObj, xs,
     $                      cu, lencu, iu, leniu, ru, lenru, 
     $                      cw, lencw, iw, leniw, rw, lenrw )
               if (ierror .ne. 0) then
                  inform = -1
                  return
               end if
             end if

            stpmax = 0.1d+0 * step
            step   = stpmax
            go to 100
         else 

*           The user wants to stop.

            inform = -1
         end if
      end if

      step = sbest 
      if (.not. imprvd) then
         call dcopy ( nb,       xs, 1, x2, 1 )
         call daxpy ( nb, step, p , 1, x2, 1 )

         if ( nlnCon ) then
            call dcopy ( nnCon,       xMul , 1, xMul2, 1 )
            call daxpy ( nnCon, step, xdMul, 1, xMul2, 1 )
            if ( Elastc ) then
               sInf2 = sInf + step*gInf
            end if
         end if
      end if

*     ==================================================================
*     The search has finished.
*     If necessary, print a few error msgs.
*     ==================================================================
      if (inform .ge. 7) then
         if      (inform .eq. 7) then
            if (iPrint .gt. 0) write(iPrint, 1700) numf
            if (iSumm  .gt. 0) write(iSumm , 1700) numf
         else if (oldg .ge. zero) then
            if (iPrint .gt. 0) write(iPrint, 1600) oldg
         end if

         if (.not. debug  .and.  inform .ne. 9) then
            if (iPrint .gt. 0) then
               write(iPrint, 1800) stepmx, pnorm, oldg, numf
            end if
         end if
      end if

      return

 1000 format(// ' --------------------------------------------'
     $       /  ' Output from s6srch following iteration', i7,
     $      5x, ' Norm p =', 1p, e11.2 )
 1600 format(/  ' XXX  The search direction is uphill.  gMrt  =',
     $          1p, e9.1)
 1700 format(   ' XXX  The line search has evaluated the functions', i5,
     $          '  times')
 1800 format(   ' stepmx =', 1p, e11.2, '    pnorm =', e11.2,
     $          '    gMrt  =',     e11.2, '    numf =', i3)

*     end of s6srch
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lsrchc( first , debug , done  , imprvd, inform,
     $                   maxf  , numf  , nout  ,
     $                   alfmax,         epsaf , 
     $                   g0    , targtg, ftry  , gtry  , 
     $                   tolabs, tolrel, toltny,
     $                   alfa  , alfbst, fbest , gbest ,
     $                   braktd, crampd, extrap, moved , wset  ,
     $                   nsamea, nsameb,
     $                   a     , b     , factor,
     $                   xtry  , xw    , fw    , gw    , tolmax )

      implicit           double precision (a-h,o-z)
      logical            first , debug , done  , imprvd
      logical            braktd, crampd, extrap, moved , wset

*     ==================================================================
*     lsrchc  finds a sequence of improving estimates of a minimizer of
*     the univariate function f(alpha) in the interval (0,alfmax].
*     f(alpha) is a smooth function such that  f(0) = 0  and  f'(0) < 0.
*     lsrchc requires both  f(alpha)  and  f'(alpha)  to be evaluated at
*     points in the interval.  Estimates of the minimizer are computed
*     using safeguarded cubic interpolation.
*
*     Reverse communication is used to allow the calling program to
*     evaluate f and f'.  Some of the parameters must be set or tested
*     by the calling program.  The remainder would ordinarily be local
*     variables.
*
*     Input parameters (relevant to the calling program)
*     --------------------------------------------------
*
*     first         must be true on the first entry. It is subsequently
*                   altered by lsrchc.
*
*     debug         specifies whether detailed output is wanted.
*
*     maxf          is an upper limit on the number of times lsrchc is
*                   to be entered consecutively with done = false
*                   (following an initial entry with first = true).
*
*     alfa          is the first estimate of a minimizer.  alfa is
*                   subsequently altered by lsrchc (see below).
*
*     alfmax        is the upper limit of the interval to be searched.
*
*     epsaf         is an estimate of the absolute precision in the
*                   computed value of f(0).
*
*     ftry, gtry    are the values of f, f'  at the new point
*                   alfa = alfbst + xtry.
*
*     g0            is the value of f'(0).  g0 must be negative.
*
*     tolabs,tolrel define a function tol(alfa) = tolrel*alfa + tolabs
*                   such that if f has already been evaluated at alfa,
*                   it will not be evaluated closer than tol(alfa).
*                   These values may be reduced by lsrchc.
*
*     targtg        is the target value of abs(f'(alfa)). The search
*                   is terminated when 
*                    abs(f'(alfa)) le targtg and f(alfa) lt 0.
*
*     toltny        is the smallest value that tolabs is allowed to be
*                   reduced to.
*
*     Output parameters (relevant to the calling program)
*     ---------------------------------------------------
*
*     imprvd        is true if the previous alfa was the best point so
*                   far.  Any related quantities should be saved by the
*                   calling program (e.g., gradient arrays) before
*                   paying attention to the variable done.
*
*     done = false  means the calling program should evaluate
*                      ftry = f(alfa),  gtry = f'(alfa)
*                   for the new trial alfa, and re-enter lsrchc.
*
*     done = true   means that no new alfa was calculated.  The value
*                   of inform gives the result of the search as follows
*
*                   inform = 1 means the search has terminated
*                              successfully with alfbst < alfmax.
*
*                   inform = 2 means the search has terminated
*                              successfully with alfbst = alfmax.
*
*                   inform = 3 means that the search failed to find a 
*                              point of sufficient decrease in maxf
*                              functions, but a lower point was found.
*
*                   inform = 4 means alfmax is so small that a search
*                              should not have been attempted.
*
*                   inform = 5 is never set by lsrchc.
*
*                   inform = 6 means the search has failed to find a
*                              useful step.  The interval of uncertainty 
*                              is [0,b] with b < 2*tolabs. A minimizer
*                              lies very close to alfa = 0, or f'(0) is
*                              not sufficiently accurate.
*
*                   inform = 7 if no better point could be found after 
*                              maxf  function calls.
*
*                   inform = 8 means the input parameters were bad.
*                              alfmax le toltny  or g0 ge zero.
*                              No function evaluations were made.
*
*     numf          counts the number of times lsrchc has been entered
*                   consecutively with done = false (i.e., with a new
*                   function value ftry).
*
*     alfa          is the point at which the next function ftry and
*                   derivative gtry must be computed.
*
*     alfbst        should be accepted by the calling program as the
*                   approximate minimizer, whenever lsrchc returns
*                   inform = 1 or 2 (and possibly 3).
*
*     fbest, gbest  will be the corresponding values of f, f'.
*
*
*     The following parameters retain information between entries
*     -----------------------------------------------------------
*
*     braktd        is false if f and f' have not been evaluated at
*                   the far end of the interval of uncertainty.  In this
*                   case, the point b will be at alfmax + tol(alfmax).
*
*     crampd        is true if alfmax is very small (le tolabs).  If the
*                   search fails, this indicates that a zero step should
*                   be taken.
*
*     extrap        is true if xw lies outside the interval of
*                   uncertainty.  In this case, extra safeguards are
*                   applied to allow for instability in the polynomial
*                   fit.
*
*     moved         is true if a better point has been found, i.e.,
*                   alfbst gt 0.
*
*     wset          records whether a second-best point has been
*                   determined it will always be true when convergence
*                   is tested.
*
*     nsamea        is the number of consecutive times that the 
*                   left-hand end point of the interval of uncertainty
*                   has remained the same.
*
*     nsameb        similarly for the right-hand end.
*
*     a, b, alfbst  define the current interval of uncertainty. 
*                   A minimizer lies somewhere in the interval 
*                   [alfbst + a, alfbst + b].
*
*     alfbst        is the best point so far.  It is always at one end 
*                   of the interval of uncertainty.  hence we have
*                   either  a lt 0,  b = 0  or  a = 0,  b gt 0.
*
*     fbest, gbest  are the values of f, f' at the point alfbst.
*
*     factor        controls the rate at which extrapolated estimates 
*                   of alfa may expand into the interval of uncertainty.
*                   factor is not used if a minimizer has been bracketed
*                   (i.e., when the variable braktd is true).
*
*     fw, gw        are the values of f, f' at the point alfbst + xw.
*                   they are not defined until wset is true.
*
*     xtry          is the trial point in the shifted interval (a, b).
*
*     xw            is such that  alfbst + xw  is the second-best point.
*                   it is not defined until  wset  is true.
*                   in some cases,  xw  will replace a previous  xw  
*                   that has a lower function but has just been excluded
*                   from the interval of uncertainty.
*
*
*     Systems Optimization Laboratory, Stanford University, California.
*     Original version February 1982.  Rev. May 1983.
*     Original f77 version 22-August-1985.
*     14-Sep-92: Introduced quitI, quitF, etc.
*     22-Nov-95: Altered criterion for reducing the step below tolabs. 
*     17-Jul-97: Removed saved variables for thread-safe version.
*     26-Jul-97: Current version.
*     ==================================================================
      logical            badfun, closef, found 
      logical            quitF , quitI
      logical            fitok , setxw 
      intrinsic          abs   , sqrt

      parameter         (zero = 0.0d+0, point1 = 0.1d+0, half = 0.5d+0)
      parameter         (one  = 1.0d+0, three  = 3.0d+0, five = 5.0d+0)
      parameter         (ten  = 1.0d+1, eleven = 1.1d+1               )

*     ------------------------------------------------------------------
*     Local variables
*     ===============
*
*     closef     is true if the new function ftry is within epsaf of
*                fbest (up or down).
*
*     found      is true if the sufficient decrease conditions hold at
*                alfbst.
*
*     quitF      is true when  maxf  function calls have been made.
*
*     quitI      is true when the interval of uncertainty is less than
*                2*tol.
*  ---------------------------------------------------------------------

      badfun = .false.
      quitF  = .false.
      quitI  = .false.
      imprvd = .false.

      if (first) then
*        ---------------------------------------------------------------
*        First entry.  Initialize various quantities, check input data
*        and prepare to evaluate the function at the initial alfa.
*        ---------------------------------------------------------------
         first  = .false.
         numf   = 0
         alfbst = zero
         badfun = alfmax .le. toltny  .or.  g0 .ge. zero
         done   = badfun
         moved  = .false.

         if (.not. done) then
            braktd = .false.
            crampd = alfmax .le. tolabs
            extrap = .false.
            wset   = .false.
            nsamea = 0
            nsameb = 0

            tolmax = tolabs + tolrel*alfmax
            a      = zero
            b      = alfmax + tolmax
            factor = five
            tol    = tolabs
            xtry   = alfa
            if (debug) then
               write (nout, 1000) g0    , tolabs, alfmax, 
     $                            targtg, tolrel, epsaf , crampd
            end if
         end if
      else
*        ---------------------------------------------------------------
*        Subsequent entries. The function has just been evaluated at
*        alfa = alfbst + xtry,  giving ftry and gtry.
*        ---------------------------------------------------------------
         if (debug) write (nout, 1100) alfa, ftry, gtry

         numf   = numf   + 1
         nsamea = nsamea + 1
         nsameb = nsameb + 1

         if (.not. braktd) then
            tolmax = tolabs + tolrel*alfmax
            b      = alfmax - alfbst + tolmax
         end if

*        See if the new step is better.  If alfa is large enough that
*        ftry can be distinguished numerically from zero,  the function
*        is required to be sufficiently negative.

         closef = abs( ftry - fbest ) .le.  epsaf
         if (closef) then
            imprvd =  abs( gtry ) .le. abs( gbest )
         else
            imprvd = ftry .lt. fbest
         end if

         if (imprvd) then

*           We seem to have an improvement.  The new point becomes the
*           origin and other points are shifted accordingly.

            fw     = fbest
            fbest  = ftry
            gw     = gbest
            gbest  = gtry
            alfbst = alfa
            moved  = .true.

            a      = a    - xtry
            b      = b    - xtry
            xw     = zero - xtry
            wset   = .true.
            extrap =       xw .lt. zero  .and.  gbest .lt. zero
     $               .or.  xw .gt. zero  .and.  gbest .gt. zero

*           Decrease the length of the interval of uncertainty.

            if (gtry .le. zero) then
               a      = zero
               nsamea = 0
            else
               b      = zero
               nsameb = 0
               braktd = .true.
            end if
         else

*           The new function value is not better than the best point so
*           far.  The origin remains unchanged but the new point may
*           qualify as xw.  xtry must be a new bound on the best point.

            if (xtry .le. zero) then
               a      = xtry
               nsamea = 0
            else
               b      = xtry
               nsameb = 0
               braktd = .true.
            end if

*           If xw has not been set or ftry is better than fw, update the
*           points accordingly.

            if (wset) then
               setxw = ftry .lt. fw  .or.  .not. extrap
            else
               setxw = .true.
            end if

            if (setxw) then
               xw     = xtry
               fw     = ftry
               gw     = gtry
               wset   = .true.
               extrap = .false.
            end if
         end if

*        ---------------------------------------------------------------
*        Check the termination criteria.  wset will always be true.
*        ---------------------------------------------------------------
         tol    = tolabs + tolrel*alfbst
         truea  = alfbst + a
         trueb  = alfbst + b

         found  = abs(gbest) .le. targtg
         quitF  = numf       .ge. maxf
         quitI  = b - a      .le. tol + tol

         if (quitI  .and. .not. moved) then

*           The interval of uncertainty appears to be small enough,
*           but no better point has been found.  Check that changing 
*           alfa by b-a changes f by less than epsaf.

            tol    = tol/ten
            tolabs = tol
            quitI  =     tol .le. toltny  .or.
     $               abs(fw) .le. epsaf   .and.  gw .le. epsaf
         end if

         done  = quitF  .or.  quitI  .or.  found

         if (debug) then
            write (nout, 1200) truea    , trueb , b - a , tol   ,
     $                         nsamea   , nsameb, numf  , 
     $                         braktd   , extrap, closef, imprvd,
     $                         found    , quitI ,
     $                         alfbst   , fbest , gbest ,
     $                         alfbst+xw, fw    , gw
         end if

*        ---------------------------------------------------------------
*        Proceed with the computation of a trial steplength.
*        The choices are...
*        1. Parabolic fit using derivatives only, if the f values are
*           close.
*        2. Cubic fit for a minimizer, using both f and f'.
*        3. Damped cubic or parabolic fit if the regular fit appears to
*           be consistently overestimating the distance to a minimizer.
*        4. Bisection, geometric bisection, or a step of  tol  if
*           choices 2 or 3 are unsatisfactory.
*        ---------------------------------------------------------------
         if (.not. done) then
            xmidpt = half*(a + b)
            s      = zero
            q      = zero

            if (closef) then
*              ---------------------------------------------------------
*              Fit a parabola to the two best gradient values.
*              ---------------------------------------------------------
               s      = gbest
               q      = gbest - gw
               if (debug) write (nout, 2200)
            else
*              ---------------------------------------------------------
*              Fit cubic through  fbest  and  fw.
*              ---------------------------------------------------------
               if (debug) write (nout, 2100)
               fitok  = .true.
               r      = three*(fbest - fw)/xw + gbest + gw
               absr   = abs( r )
               s      = sqrt( abs( gbest ) ) * sqrt( abs( gw ) )

*              Compute  q =  the square root of  r*r - gbest*gw.
*              The method avoids unnecessary underflow and overflow.

               if ((gw .lt. zero  .and.  gbest .gt. zero) .or.
     $             (gw .gt. zero  .and.  gbest .lt. zero)) then
                  scale  = absr + s
                  if (scale .eq. zero) then
                     q  = zero
                  else
                     q  = scale*sqrt( (absr/scale)**2 + (s/scale)**2 )
                  end if
               else if (absr .ge. s) then
                  q     = sqrt(absr + s)*sqrt(absr - s)
               else
                  fitok = .false.
               end if

               if (fitok) then

*                 Compute a minimizer of the fitted cubic.

                  if (xw .lt. zero) q = - q
                  s  = gbest -  r - q
                  q  = gbest - gw - q - q
               end if
            end if
*           ------------------------------------------------------------
*           Construct an artificial interval  (artifa, artifb)  in which
*           the new estimate of a minimizer must lie.  Set a default
*           value of xtry that will be used if the polynomial fit fails.
*           ------------------------------------------------------------
            artifa = a
            artifb = b
            if (.not. braktd) then

*              A minimizer has not been bracketed.  Set an artificial
*              upper bound by expanding the interval  xw  by a suitable
*              factor.

               xtry   = - factor*xw
               artifb =   xtry
               if (alfbst + xtry .lt. alfmax) factor = five*factor

            else if (extrap) then

*              The points are configured for an extrapolation.
*              Set a default value of  xtry  in the interval  (a, b)
*              that will be used if the polynomial fit is rejected.  In
*              the following,  dtry  and  daux  denote the lengths of
*              the intervals  (a, b)  and  (0, xw)  (or  (xw, 0),  if
*              appropriate).  The value of  xtry is the point at which
*              the exponents of  dtry  and  daux  are approximately
*              bisected.

               daux = abs( xw )
               dtry = b - a
               if (daux .ge. dtry) then
                  xtry = five*dtry*(point1 + dtry/daux)/eleven
               else
                  xtry = half * sqrt( daux ) * sqrt( dtry )
               end if
               if (xw .gt. zero)   xtry = - xtry
               if (debug) write (nout, 2400) xtry, daux, dtry

*              Reset the artificial bounds.  If the point computed by
*              extrapolation is rejected,  xtry will remain at the
*              relevant artificial bound.

               if (xtry .le. zero) artifa = xtry
               if (xtry .gt. zero) artifb = xtry
            else

*              The points are configured for an interpolation.  The
*              default value xtry bisects the interval of uncertainty.
*              the artificial interval is just (a, b).

               xtry   = xmidpt
               if (debug) write (nout, 2300) xtry
               if (nsamea .ge. 3  .or.  nsameb .ge. 3) then

*                 If the interpolation appears to be overestimating the
*                 distance to a minimizer,  damp the interpolation.

                  factor = factor / five
                  s      = factor * s
               else
                  factor = one
               end if
            end if
*           ------------------------------------------------------------
*           The polynomial fits give  (s/q)*xw  as the new step.
*           Reject this step if it lies outside  (artifa, artifb).
*           ------------------------------------------------------------
            if (q .ne. zero) then
               if (q .lt. zero) s = - s
               if (q .lt. zero) q = - q
               if (s*xw .ge. q*artifa  .and.  s*xw .le. q*artifb) then

*                 Accept the polynomial fit.

                  if (abs( s*xw ) .ge. q*tol) then
                     xtry = (s/q)*xw
                  else
                     xtry = zero
                  end if
                  if (debug) write (nout, 2500) xtry
               end if
            end if
         end if
      end if

*     ==================================================================

      if (.not. done) then
         alfa  = alfbst + xtry
         if (braktd  .or.  alfa .lt. alfmax - tolmax) then

*           The function must not be evaluated too close to a or b.
*           (It has already been evaluated at both those points.)

            if (xtry .le. a + tol  .or.  xtry .ge. b - tol) then
               if (half*(a + b) .le. zero) then
                  xtry = - tol
               else
                  xtry =   tol
               end if
               alfa = alfbst + xtry
            end if
         else

*           The step is close to, or larger than alfmax, replace it by
*           alfmax to force evaluation of  f  at the boundary.

            braktd = .true.
            xtry   = alfmax - alfbst
            alfa   = alfmax
         end if
      end if

*     ------------------------------------------------------------------
*     Exit.
*     ------------------------------------------------------------------
      if (done) then
         if      (badfun) then
            inform = 8
         else if (found) then
            if (alfbst .lt. alfmax) then
               inform = 1
            else
               inform = 2
            end if
         else if (moved ) then
            inform = 3
         else if (quitF) then
            inform = 7
         else if (crampd) then
            inform = 4
         else
            inform = 6
         end if
      end if

      if (debug) write (nout, 3000)
      return

 1000 format(/'     g0  tolabs  alfmax        ', 1p, 2e22.14,   e16.8
     $       /' targtg  tolrel   epsaf        ', 1p, 2e22.14,   e16.8
     $       /' crampd                        ',  l3)
 1100 format(/' alfa    ftry    gtry          ', 1p, 2e22.14,   e16.8)
 1200 format(/' a       b       b - a   tol   ', 1p, 2e22.14,  2e16.8
     $       /' nsamea  nsameb  numf          ', 3i3
     $       /' braktd  extrap  closef  imprvd', 4l3
     $       /' found   quitI                 ', 2l3
     $       /' alfbst  fbest   gbest         ', 1p, 3e22.14          
     $       /' alfaw   fw      gw            ', 1p, 3e22.14)
 2100 format( ' Cubic.   ')
 2200 format( ' Parabola.')
 2300 format( ' Bisection.              xmidpt', 1p,  e22.14)
 2400 format( ' Geo. bisection. xtry,daux,dtry', 1p, 3e22.14)
 2500 format( ' Polynomial fit accepted.  xtry', 1p,  e22.14)
 3000 format( ' ----------------------------------------------------'/)

*     end of lsrchc
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lsrchq( first , debug , done  , imprvd, inform,
     $                   maxf  , numf  , nout  , 
     $                   alfmax, alfsml, epsaf , 
     $                   g0    , targtg, ftry  , 
     $                   tolabs, tolrel, toltny,
     $                   alfa  , alfbst, fbest ,
     $                   braktd, crampd, extrap, moved , vset  , wset,
     $                   nsamea, nsameb,
     $                   a     , b     , fa    , factor,
     $                   xtry  , xw    , fw    , xv    , fv    , tolmax)

      implicit           double precision (a-h,o-z)
      logical            first , debug , done  , imprvd
      logical            braktd, crampd, extrap, moved , vset  , wset

*     ==================================================================
*     lsrchq  finds a sequence of improving estimates of a minimizer of
*     the univariate function f(alpha) in the interval (0,alfmax].
*     f(alpha) is a smooth function such that  f(0) = 0  and  f'(0) < 0.
*     lsrchq  requires  f(alpha) (but not f'(alpha)) to be evaluated
*     in the interval.  New estimates of a minimizer are computed using
*     safeguarded quadratic interpolation.
*
*     Reverse communication is used to allow the calling program to
*     evaluate f.  Some of the parameters must be set or tested by the
*     calling program.  The remainder would ordinarily be local
*     variables.
*
*     Input parameters (relevant to the calling program)
*     --------------------------------------------------
*
*     first         must be true on the first entry.  It is subsequently
*                   altered by lsrchq.
*
*     debug         specifies whether detailed output is wanted.
*
*     maxf          is an upper limit on the number of times lsrchq is
*                   to be entered consecutively with done = false 
*                   (following an initial entry with first = true).
*
*     alfa          is the first estimate of a minimizer.  alfa is
*                   subsequently altered by lsrchq (see below).
*
*     alfmax        is the upper limit of the interval to be searched.
*
*     alfsml        is intended to prevent inefficiency when a minimizer
*                   is very small, for cases where the calling program
*                   would prefer to redefine f'(alfa).  alfsml is
*                   allowed to be zero.  Early termination will occur if
*                   lsrchq determines that a minimizer lies somewhere in
*                   the interval [0, alfsml) (but not if alfmax is 
*                   smaller that alfsml).
*
*     epsaf         is an estimate of the absolute precision in the
*                   computed value of f(0).
*
*     ftry          the value of f at the new point
*                   alfa = alfbst + xtry.
*
*     g0            is the value of f'(0).  g0 must be negative.
*
*     tolabs,tolrel define a function tol(alfa) = tolrel*alfa + tolabs
*                   such that if f has already been evaluated at alfa,
*                   it will not be evaluated closer than tol(alfa).
*                   These values may be reduced by lsrchq.
*
*     targtg        is the target value of abs(f'(alfa)). The search
*                   is terminated when 
*                    abs(f'(alfa)) le targtg and f(alfa) lt 0.
*
*     toltny        is the smallest value that tolabs is allowed to be
*                   reduced to.
*
*     Output parameters (relevant to the calling program)
*     ---------------------------------------------------
*
*     imprvd        is true if the previous alfa was the best point so
*                   far.  Any related quantities should be saved by the
*                   calling program (e.g., arrays) before paying
*                   attention to the variable done.
*
*     done = false  means the calling program should evaluate ftry
*                   for the new trial step alfa, and reenter lsrchq.
*
*     done = true   means that no new alfa was calculated.  The value
*                   of inform gives the result of the search as follows
*
*                   inform = 1 means the search has terminated 
*                              successfully with alfbst < alfmax.
*
*                   inform = 2 means the search has terminated
*                              successfully with alfbst = alfmax.
*
*                   inform = 3 means that the search failed to find a 
*                              point of sufficient decrease in maxf
*                              functions, but a lower point was found.
*
*                   inform = 4 means alfmax is so small that a search
*                              should not have been attempted.
*
*                   inform = 5 means that the search was terminated
*                              because of alfsml (see above).
*
*                   inform = 6 means the search has failed to find a
*                              useful step.  The interval of uncertainty 
*                              is [0,b] with b < 2*tolabs. A minimizer
*                              lies very close to alfa = 0, or f'(0) is
*                              not sufficiently accurate.
*
*                   inform = 7 if no better point could be found after 
*                              maxf  function calls.
*
*                   inform = 8 means the input parameters were bad.
*                              alfmax le toltny  or  g0 ge zero.
*                              No function evaluations were made.
*
*     numf          counts the number of times lsrchq has been entered
*                   consecutively with done = false (i.e., with a new
*                   function value ftry).
*
*     alfa          is the point at which the next function ftry must 
*                   be computed.
*
*     alfbst        should be accepted by the calling program as the
*                   approximate minimizer, whenever lsrchq returns
*                   inform = 1, 2 or 3.
*
*     fbest         will be the corresponding value of f.
*
*     The following parameters retain information between entries
*     -----------------------------------------------------------
*
*     braktd        is false if f has not been evaluated at the far end
*                   of the interval of uncertainty.  In this case, the
*                   point b will be at alfmax + tol(alfmax).
*
*     crampd        is true if alfmax is very small (le tolabs).  If the
*                   search fails, this indicates that a zero step should
*                   be taken.
*
*     extrap        is true if alfbst has moved at least once and xv 
*                   lies outside the interval of uncertainty.  In this
*                   case, extra safeguards are applied to allow for
*                   instability in the polynomial fit.
*
*     moved         is true if a better point has been found, i.e., 
*                   alfbst gt 0.
*
*     vset          records whether a third-best point has been defined.
*
*     wset          records whether a second-best point has been 
*                   defined.  It will always be true by the time the
*                   convergence test is applied.
*
*     nsamea        is the number of consecutive times that the
*                   left-hand end point of the interval of uncertainty
*                   has remained the same.
*
*     nsameb        similarly for the right-hand end.
*
*     a, b, alfbst  define the current interval of uncertainty.
*                   A minimizer lies somewhere in the  interval
*                   [alfbst + a, alfbst + b].
*
*     alfbst        is the best point so far.  It lies strictly within
*                   [atrue,btrue]  (except when alfbst has not been
*                   moved, in which case it lies at the left-hand end
*                   point).  Hence we have a .le. 0 and b .gt. 0.
*
*     fbest         is the value of f at the point alfbst.
*
*     fa            is the value of f at the point alfbst + a.
*
*     factor        controls the rate at which extrapolated estimates of
*                   alfa  may expand into the interval of uncertainty.
*                   Factor is not used if a minimizer has been bracketed
*                   (i.e., when the variable braktd is true).
*
*     fv, fw        are the values of f at the points alfbst + xv  and
*                   alfbst + xw.  They are not defined until  vset  or
*                   wset  are true.
*
*     xtry          is the trial point within the shifted interval
*                   (a, b).  The new trial function value must be
*                   computed at the point alfa = alfbst + xtry.
*
*     xv            is such that alfbst + xv is the third-best point. 
*                   It is not defined until vset is true.
*
*     xw            is such that alfbst + xw is the second-best point. 
*                   It is not defined until wset is true.  In some
*                   cases,  xw will replace a previous xw that has a
*                   lower function but has just been excluded from 
*                   (a,b).
*
*     Systems Optimization Laboratory, Stanford University, California.
*     Original version February 1982.  Rev. May 1983.
*     Original F77 version 22-August-1985.
*     17-Jul-97: Removed saved variables for thread-safe version.
*     26-Jul-97: Current version.
*     ==================================================================
      logical            badfun, closef, found 
      logical            quitF , quitFZ, quitI , quitS 
      logical            setxv , xinxw
      intrinsic          abs   , sqrt

      parameter         (zero = 0.0d+0, point1 = 0.1d+0, half = 0.5d+0)
      parameter         (one  = 1.0d+0, two    = 2.0d+0, five = 5.0d+0)
      parameter         (ten  = 1.0d+1, eleven = 1.1d+1               )

*     ------------------------------------------------------------------
*     Local variables
*     ===============
*
*     closef     is true if the worst function fv is within epsaf of
*                fbest (up or down).
*
*     found      is true if the sufficient decrease conditions holds at
*                alfbst.
*
*     quitF      is true when  maxf  function calls have been made.
*
*     quitFZ     is true when the three best function values are within
*                epsaf of each other, and the new point satisfies
*                fbest le ftry le fbest+epsaf.
*
*     quitI      is true when the interval of uncertainty is less than
*                2*tol.
*
*     quitS      is true as soon as alfa is too small to be useful;
*                i.e., btrue le alfsml.
*
*     xinxw      is true if xtry is in (xw,0) or (0,xw).
*     ------------------------------------------------------------------

      imprvd = .false.
      badfun = .false.
      quitF  = .false.
      quitFZ = .false.
      quitS  = .false.
      quitI  = .false.

      if (first) then
*        ---------------------------------------------------------------
*        First entry.  Initialize various quantities, check input data
*        and prepare to evaluate the function at the initial step alfa.
*        ---------------------------------------------------------------
         first  = .false.
         numf   = 0
         alfbst = zero
         badfun = alfmax .le. toltny  .or.  g0 .ge. zero
         done   = badfun
         moved  = .false.

         if (.not. done) then
            braktd = .false.
            crampd = alfmax .le. tolabs
            extrap = .false.
            vset   = .false.
            wset   = .false.
            nsamea = 0
            nsameb = 0

            tolmax = tolrel*alfmax + tolabs
            a      = zero
            b      = alfmax + tolmax
            fa     = zero
            factor = five
            tol    = tolabs
            xtry   = alfa
            if (debug) then
               write (nout, 1000) g0    , tolabs, alfmax, 
     $                            targtg, tolrel, epsaf , crampd
            end if
         end if
      else
*        ---------------------------------------------------------------
*        Subsequent entries.  The function has just been evaluated at
*        alfa = alfbst + xtry,  giving ftry.
*        ---------------------------------------------------------------
         if (debug) write (nout, 1100) alfa, ftry

         numf   = numf   + 1
         nsamea = nsamea + 1
         nsameb = nsameb + 1

         if (.not. braktd) then
            tolmax = tolabs + tolrel*alfmax
            b      = alfmax - alfbst + tolmax
         end if

*        Check if xtry is in the interval (xw,0) or (0,xw).

         if (wset) then
            xinxw =        zero .lt. xtry  .and.  xtry .le. xw
     $               .or.    xw .le. xtry  .and.  xtry .lt. zero
         else
            xinxw = .false.
         end if

         imprvd = ftry .lt. fbest
         if (vset) then
            closef = abs( fbest - fv ) .le. epsaf
         else
            closef = .false.
         end if

         if (imprvd) then

*           We seem to have an improvement.  The new point becomes the
*           origin and other points are shifted accordingly.

            if (wset) then
               xv     = xw - xtry
               fv     = fw
               vset   = .true.
            end if

            xw     = zero - xtry
            fw     = fbest
            wset   = .true.
            fbest  = ftry
            alfbst = alfa
            moved  = .true.

            a      = a    - xtry
            b      = b    - xtry
            extrap = .not. xinxw

*           Decrease the length of (a,b).

            if (xtry .ge. zero) then
               a      = xw
               fa     = fw
               nsamea = 0
            else
               b      = xw
               nsameb = 0
               braktd = .true.
            end if
         else if (closef  .and.  ftry - fbest .lt. epsaf) then

*           Quit if there has been no progress and ftry, fbest, fw
*           and fv are all within epsaf of each other.

            quitFZ = .true.
         else

*           The new function value is no better than the current best
*           point.  xtry must an end point of the new (a,b).

            if (xtry .lt. zero) then
               a      = xtry
               fa     = ftry
               nsamea = 0
            else
               b      = xtry
               nsameb = 0
               braktd = .true.
            end if

*           The origin remains unchanged but xtry may qualify as xw.

            if (wset) then
               if (ftry .lt. fw) then
                  xv     = xw
                  fv     = fw
                  vset   = .true.

                  xw     = xtry
                  fw     = ftry
                  if (moved) extrap = xinxw
               else if (moved) then
                  if (vset) then
                     setxv = ftry .lt. fv  .or.  .not. extrap
                  else
                     setxv = .true.
                  end if

                  if (setxv) then
                     if (vset  .and.  xinxw) then
                        xw = xv
                        fw = fv
                     end if
                     xv   = xtry
                     fv   = ftry
                     vset = .true.
                  end if
               else
                  xw  = xtry
                  fw  = ftry
               end if
            else
               xw     = xtry
               fw     = ftry
               wset   = .true.
            end if
         end if

*        ---------------------------------------------------------------
*        Check the termination criteria.
*        ---------------------------------------------------------------
         tol    = tolabs + tolrel*alfbst
         truea  = alfbst + a
         trueb  = alfbst + b

         found  = moved  .and.  abs(fa - fbest) .le. -a*targtg
         quitF  = numf  .ge. maxf
         quitI  = b - a .le. tol + tol
         quitS  = trueb .le. alfsml

         if (quitI  .and.  .not. moved) then

*           The interval of uncertainty appears to be small enough,
*           but no better point has been found.  Check that changing 
*           alfa by b-a changes f by less than epsaf.

            tol    = tol/ten
            tolabs = tol
            quitI  = abs(fw) .le. epsaf  .or.  tol .le. toltny
         end if

         done  = quitF  .or.  quitFZ  .or.  quitS  .or.  quitI
     $                  .or.  found

         if (debug) then
            write (nout, 1200) truea    , trueb , b-a   , tol   ,
     $                         nsamea   , nsameb, numf  ,
     $                         braktd   , extrap, closef, imprvd,
     $                         found    , quitI , quitFZ, quitS ,
     $                         alfbst   , fbest ,
     $                         alfbst+xw, fw
            if (vset) then
               write (nout, 1300) alfbst + xv, fv
            end if
         end if

*        ---------------------------------------------------------------
*        Proceed with the computation of an estimate of a minimizer.
*        The choices are...
*        1. Parabolic fit using function values only.
*        2. Damped parabolic fit if the regular fit appears to be
*           consistently overestimating the distance to a minimizer.
*        3. Bisection, geometric bisection, or a step of tol if the
*           parabolic fit is unsatisfactory.
*        ---------------------------------------------------------------
         if (.not. done) then
            xmidpt = half*(a + b)
            s      = zero
            q      = zero

*           ============================================================
*           Fit a parabola.
*           ============================================================
*           See if there are two or three points for the parabolic fit.

            gw = (fw - fbest)/xw
            if (vset  .and.  moved) then

*              Three points available.  Use fbest, fw and fv.

               gv = (fv - fbest)/xv
               s  = gv - (xv/xw)*gw
               q  = two*(gv - gw)
               if (debug) write (nout, 2200)
            else

*              Only two points available.  Use fbest, fw and g0.

               if (moved) then
                  s  = g0 - two*gw
               else
                  s  = g0
               end if
               q = two*(g0 - gw)
               if (debug) write (nout, 2100)
            end if

*           ------------------------------------------------------------
*           Construct an artificial interval (artifa, artifb) in which 
*           the new estimate of the steplength must lie.  Set a default
*           value of  xtry  that will be used if the polynomial fit is
*           rejected. In the following, the interval (a,b) is considered
*           the sum of two intervals of lengths  dtry  and  daux, with
*           common end point the best point (zero).  dtry is the length
*           of the interval into which the default xtry will be placed
*           and endpnt denotes its non-zero end point.  The magnitude of
*           xtry is computed so that the exponents of dtry and daux are
*           approximately bisected.
*           ------------------------------------------------------------
            artifa = a
            artifb = b
            if (.not. braktd) then

*              A minimizer has not yet been bracketed.  
*              Set an artificial upper bound by expanding the interval
*              xw  by a suitable factor.

               xtry   = - factor*xw
               artifb =   xtry
               if (alfbst + xtry .lt. alfmax) factor = five*factor
            else if (vset .and. moved) then

*              Three points exist in the interval of uncertainty.
*              Check if the points are configured for an extrapolation
*              or an interpolation.

               if (extrap) then

*                 The points are configured for an extrapolation.

                  if (xw .lt. zero) endpnt = b
                  if (xw .gt. zero) endpnt = a
               else

*                 If the interpolation appears to be overestimating the
*                 distance to a minimizer,  damp the interpolation step.

                  if (nsamea .ge. 3  .or.   nsameb .ge. 3) then
                     factor = factor / five
                     s      = factor * s
                  else
                     factor = one
                  end if

*                 The points are configured for an interpolation.  The
*                 artificial interval will be just (a,b).  Set endpnt so
*                 that xtry lies in the larger of the intervals (a,b) 
*                 and  (0,b).

                  if (xmidpt .gt. zero) then
                     endpnt = b
                  else
                     endpnt = a
                  end if

*                 If a bound has remained the same for three iterations,
*                 set endpnt so that  xtry  is likely to replace the
*                 offending bound.

                  if (nsamea .ge. 3) endpnt = a
                  if (nsameb .ge. 3) endpnt = b
               end if

*              Compute the default value of  xtry.

               dtry = abs( endpnt )
               daux = b - a - dtry
               if (daux .ge. dtry) then
                  xtry = five*dtry*(point1 + dtry/daux)/eleven
               else
                  xtry = half*sqrt( daux )*sqrt( dtry )
               end if
               if (endpnt .lt. zero) xtry = - xtry
               if (debug) write (nout, 2500) xtry, daux, dtry

*              If the points are configured for an extrapolation set the
*              artificial bounds so that the artificial interval lies
*              within (a,b).  If the polynomial fit is rejected,  xtry 
*              will remain at the relevant artificial bound.

               if (extrap) then
                  if (xtry .le. zero) then
                     artifa = xtry
                  else
                     artifb = xtry
                  end if
               end if
            else

*              The gradient at the origin is being used for the
*              polynomial fit.  Set the default xtry to one tenth xw.

               if (extrap) then
                  xtry = - xw
               else
                  xtry   = xw/ten
               end if
               if (debug) write (nout, 2400) xtry
            end if

*           ------------------------------------------------------------
*           The polynomial fits give (s/q)*xw as the new step.  Reject
*           this step if it lies outside (artifa, artifb).
*           ------------------------------------------------------------
            if (q .ne. zero) then
               if (q .lt. zero) s = - s
               if (q .lt. zero) q = - q
               if (s*xw .ge. q*artifa   .and.   s*xw .le. q*artifb) then
 
*                 Accept the polynomial fit.

                  if (abs( s*xw ) .ge. q*tol) then
                     xtry = (s/q)*xw
                  else
                     xtry = zero
                  end if
                  if (debug) write (nout, 2600) xtry
               end if
            end if
         end if
      end if
*     ==================================================================

      if (.not. done) then
         alfa  = alfbst + xtry
         if (braktd  .or.  alfa .lt. alfmax - tolmax) then

*           The function must not be evaluated too close to a or b.
*           (It has already been evaluated at both those points.)

            xmidpt = half*(a + b)
            if (xtry .le. a + tol  .or.  xtry .ge. b - tol) then
               if (xmidpt .le. zero) then
                  xtry = - tol
               else
                  xtry =   tol
               end if
            end if

            if (abs( xtry ) .lt. tol) then
               if (xmidpt .le. zero) then
                  xtry = - tol
               else
                  xtry =   tol
               end if
            end if
            alfa  = alfbst + xtry
         else

*           The step is close to or larger than alfmax, replace it by
*           alfmax to force evaluation of the function at the boundary.

            braktd = .true.
            xtry   = alfmax - alfbst
            alfa   = alfmax
         end if
      end if
*     ------------------------------------------------------------------
*     Exit.
*     ------------------------------------------------------------------
      if (done) then
         if      (badfun) then
            inform = 8
         else if (quitS ) then
            inform = 5
         else if (found) then
            if (alfbst .lt. alfmax) then
               inform = 1
            else
               inform = 2
            end if
         else if (moved ) then
            inform = 3
         else if (quitF) then
            inform = 7
         else if (crampd) then
            inform = 4
         else
            inform = 6
         end if
      end if

      if (debug) write (nout, 3000)
      return

 1000 format(/'     g0  tolabs  alfmax        ', 1p, 2e22.14,   e16.8
     $       /' targtg  tolrel   epsaf        ', 1p, 2e22.14,   e16.8
     $       /' crampd                        ',  l3)
 1100 format(/' alfa    ftry                  ', 1p,2e22.14          )
 1200 format(/' a       b       b - a   tol   ', 1p,2e22.14,   2e16.8
     $       /' nsamea  nsameb  numf          ', 3i3
     $       /' braktd  extrap  closef  imprvd', 4l3
     $       /' found   quitI   quitFZ  quitS ', 4l3
     $       /' alfbst  fbest                 ', 1p,2e22.14          
     $       /' alfaw   fw                    ', 1p,2e22.14)
 1300 format( ' alfav   fv                    ', 1p,2e22.14 /)
 2100 format( ' Parabolic fit,    two points. ')
 2200 format( ' Parabolic fit,  three points. ')
 2400 format( ' Exponent reduced.  Trial point', 1p,  e22.14)
 2500 format( ' Geo. bisection. xtry,daux,dtry', 1p, 3e22.14)
 2600 format( ' Polynomial fit accepted.  xtry', 1p,  e22.14)
 3000 format( ' ----------------------------------------------------'/)

*     end of lsrchq
      end
