*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
*     File  sn12npzz.f
*
*     npopt    npfcon   npfobj   npload   npprnt   
*     nptitl   npInit   npSpec   npMem
*     npset    npseti   npsetr   npgeti   npgetr
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine npopt ( n, nclin, ncnln, ldA, ldcJ, ldH,
     $                   A, bl, bu,
     $                   funcon, funobj,
     $                   inform, majIts, iState,
     $                   c, cJac, cMul, Objf, grad, Hess, x,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)

      external           funcon, funobj

      integer            iState(n+nclin+ncnln)
      double precision   A(ldA,*), bl(n+nclin+ncnln),
     $                   bu(n+nclin+ncnln)
      double precision   c(*), cJac(ldcJ,*), cMul(n+nclin+ncnln)
      double precision   grad(n), Hess(ldH,*), x(n)

      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     npopt    solves the nonlinear programming problem
*
*            minimize                   f(x)
*
*                                    (    x  )
*            subject to    bl  .le.  (  A*x  )  .le.  bu
*                                    (  c(x) )
*
*     where  f(x)  is a smooth scalar function,  A  is a constant matrix
*     and  c(x)  is a vector of smooth nonlinear functions. The feasible
*     region is defined by a mixture of linear and nonlinear equality or
*     inequality constraints on  x.
*
*     The calling sequence of NPOPT and the user-defined functions
*     funcon and funobj are identical to those of the dense code NPSOL
*     (see the User's Guide for NPSOL (Version 4.0): a Fortran Package
*     for Nonlinear Programming, Systems Optimization Laboratory Report
*     SOL 86-2, Department of Operations Research, Stanford University,
*     1986.)
*
*     The dimensions of the problem are...
*
*     n        the number of variables (dimension of  x),
*
*     nclin    the number of linear constraints (rows of the matrix  A),
*
*     ncnln    the number of nonlinear constraints (dimension of  c(x)),
*
*     22 Mar 1997: First version of npopt.
*     17 Jul 1997: First thread-safe version.
*     16 May 1998: Current version.
*     ==================================================================
      character*4        Start
      character*8        cdummy

      parameter         (maxru     =   2)
      parameter         (maxiu     =   4)
      parameter         (maxcu     =   6)

      parameter         (iPrint    =  12)
      parameter         (iSumm     =  13)
      parameter         (lvlSrt    =  70)
      parameter         (nName     = 233)

      integer            tolfac, tolupd
      parameter         (tolfac    =  66)
      parameter         (tolupd    =  67)

      parameter         (lHtype    =  53)
      parameter         (lprSol    =  84)

      parameter         (mName     =  51)
      parameter         (cdummy = '-1111111')
      parameter         (zero   = 0.0d+0,  one    = 1.0d+0)
      parameter         (lencw     = 500)
      character*8        cw(lencw)

      external           npfobj, npfcon
*     ------------------------------------------------------------------

      if (leniw .lt. 500 .or. lenrw .lt. 500) then 
*        ---------------------------------------------------------------
*        Not enough workspace to do ANYTHING!
*        ---------------------------------------------------------------
         inform = 41
         if (iw(iPrint) .gt. 0) write(iw(iPrint), 9000) 
         if (iw(iSumm ) .gt. 0) write(iw(iSumm ), 9000) 
         if (iw(iPrint) .le. 0  .and.  iw(iSumm)  .le. 0) then
            write(*, 9000)
         end if
         go to 999
      end if

*     Initialize timers and the standard input file.

      call s1time( 0, 0, iw, leniw, rw, lenrw  )
      call s1file( 'Standard input', iw, leniw )

      ierror     = 0

*     Load the local problem dimensions.

      nCon       = nclin + ncnln

      if (nCon .eq. 0) then

*        The problem is unconstrained.
*        Include a dummy row of zeros.

         nnCol = 0
         m     = 1
         ne    = 1
      else
         nnCol = n
         m     = nCon
         ne    = m*n 
      end if

      nb         = n     + m
      nka        = n     + 1

*     Load the iw array with various problem dimensions.
*     First record problem dimensions for smart users to access in iw.

      iObj       = 0
      neJac      = ncnln*n
      nnCon      = ncnln
      nnJac      = nnCol
      nnObj      = n
      
      iw( 15)    = m
      iw( 16)    = n
      iw( 17)    = ne
      iw( 20)    = neJac
      iw( 21)    = nnCon
      iw( 22)    = nnJac
      iw( 23)    = nnObj
      iw(218)    = iObj

      iw(nName)  = 1

      nnCon0     = max ( nnCon, 1 )

*     The obligatory call to npInit has already ``unset'' 
*     the optional parameters.  However, it could not undefine 
*     the char*8 options.  Do it now.

      do 100, i = 51, 180
         cw(i)  = cdummy
  100 continue

*     Set options that relate specially to npopt.

      if (rw(tolfac) .lt. one) rw(tolfac) = 1.1d+0
      if (rw(tolupd) .lt. one) rw(tolupd) = 1.1d+0
      if (iw(lHtype) .lt. 0  ) iw(lHtype) = 2

      ObjAdd   = zero

*     Check that the optional parameters have sensible values.

      call s8dflt( 'Check optional parameters', 
     $             cw, lencw, iw, leniw, rw, lenrw )

*     ------------------------------------------------------------------
*     Print the options if iPrint > 0, Print level > 0 and lvlPrm > 0.
*     ------------------------------------------------------------------
      call s8dflt( 'Print the options', 
     $             cw, lencw, iw, leniw, rw, lenrw )

*     ------------------------------------------------------------------
*     Determine storage requirements using the
*     following Common variables:
*         maxm,   maxn, maxne
*         maxR ,  maxS, nnL       (set in s8dflt)
*         nnObj,
*         neJac, nnCon, nnJac
*     All have to be known exactly before calling s8Mem.
*     ------------------------------------------------------------------
      mincw  = iw(maxcu) + 1
      miniw  = iw(maxiu) + 1
      minrw  = iw(maxru) + 1

*     Allocate arrays that are arguments of s8solv.
*     These are for the data,
*              a, ha, ka, bl, bu, Names,
*     and for the solution
*              hs, xs, pi, rc, hs.

      lNames     = mincw  - 1      ! No names

      lha        = miniw
      lka        = lha    + ne
      lhs        = lka    + nka
      miniw      = lhs    + nb

      la         = minrw
      lbl        = la     + ne
      lbu        = lbl    + nb
      lxs        = lbu    + nb
      lpi        = lxs    + nb
      lrc        = lpi    + m
      minrw      = lrc    + nb

*     Fetch the limits on NPOPT real and integer workspace.
*     They are used to define the limits on LU workspace. 

      maxrw = iw(  3)          ! Extent of NPOPT real    workspace
      maxiw = iw(  5)          ! Extent of NPOPT integer workspace
      maxcw = iw(  7)          ! Extent of NPOPT char*8  workspace

      maxR  = iw( 56)
      maxS  = iw( 57)

      call s8Mem ( ierror, iw(iPrint), iw(iSumm), 
     $             m, n, ne, neJac,
     $             nnCon, nnJac, nnObj,
     $             maxR, maxS, 
     $             maxcw, maxiw, maxrw,
     $             lencw, leniw, lenrw,
     $             mincw, miniw, minrw, iw )
      if (ierror .ne. 0) then
         inform = ierror
         go to 999
      end if

      iw(251)   = la     
      iw(252)   = lha    
      iw(253)   = lka    
      iw(254)   = lbl    
      iw(255)   = lbu    
      iw(256)   = lxs    
      iw(257)   = lpi    
      iw(258)   = lrc    
      iw(259)   = lhs    
      iw(261)   = lNames 

      lgObj    = iw(298)  
      lfCon    = iw(302)  
      lgCon    = iw(305)  

*     ------------------------------------------------------------------
*     Load a generic problem name.
*     ------------------------------------------------------------------
      cw(mName) = '     NLP'

      if (iw(lvlSrt) .eq. 0) then
         Start = 'Cold'
      else
         Start = 'Warm'
      end if

*     ------------------------------------------------------------------
*     Load the SNOPT arrays.
*     ------------------------------------------------------------------
      call npload( 'Load', Start,
     $             ldA, ldcJ, ldH, m, n, ncLin, nCon, nnCol,
     $             nb, nnCon, nnCon0,
     $             iw(lhs), iState,
     $             A, ne, nka, rw(la), iw(lha), iw(lka),
     $             bl, bu, rw(lbl), rw(lbu), c, cJac, cMul,
     $             rw(lfCon), rw(lgCon), rw(lgObj), grad, 
     $             Hess, rw(lrc), x, rw(lxs),
     $             iw, leniw, rw, lenrw )

*     ------------------------------------------------------------------
*     Solve the problem.
*     ------------------------------------------------------------------
      lprsv      = iw(lprSol)
      iw(lprSol) = 0

      call s8solv( 'Cold', 
     $             m, n, nb, ne, nka, iw(nName), 
     $             iObj, ObjAdd, fObj, Objtru,
     $             nInf, sInf,
     $             npfcon, funcon, npfobj, funobj,
     $             rw(la), iw(lha), iw(lka), rw(lbl),rw(lbu),cw(lNames),
     $             iw(lhs), rw(lxs), rw(lpi), rw(lrc),
     $             inform, nMajor, nS, 
     $             cw, lencw, iw, leniw, rw, lenrw,
     $             cw, lencw, iw, leniw, rw, lenrw )
      iw(lprSol) = lprsv

      Objf   = fObj
      MajIts = nMajor

*     ------------------------------------------------------------------
*     Unload the SNOPT arrays.
*     ------------------------------------------------------------------
      call npload( 'Unload', 'Cold',
     $             ldA, ldcJ, ldH, m, n, ncLin, nCon, nnCol,
     $             nb, nnCon, nnCon0,
     $             iw(lhs), iState,
     $             A, ne, nka, rw(la), iw(lha), iw(lka),
     $             bl, bu, rw(lbl), rw(lbu), c, cJac, cMul,
     $             rw(lfCon), rw(lgCon), rw(lgObj), grad, 
     $             Hess, rw(lrc), x, rw(lxs),
     $             iw, leniw, rw, lenrw )

      call npprnt( n, (n+nCon), ncLin, nnCon0, ldA, lprSol,
     $             iState, A, bl, bu, c, cMul, x, rw(lxs),
     $             iw, leniw, rw, lenrw )

*     Print times for all clocks (if lvlTim > 0).

      call s1time( 0, 2, iw, leniw, rw, lenrw )

  999 return

 9000 format(  ' EXIT -- NPOPT integer and real work arrays',
     $         ' must each have at least 500 elements')

*     end of  npopt
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine npfcon( modefg, ierror, nState,
     $                   n, nnCon, nnJac, neJac, fgCon,
     $                   ne, nka, ha, ka,
     $                   fCon, gCon, x, 
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      external           fgCon
      integer            ha(ne)
      integer            ka(nka)
      double precision   fCon(nnCon), gCon(neJac), x(nnJac)

      character*8        cu(lencu), cw(lencw)
      integer            iu(leniu), iw(leniw)
      double precision   ru(lenru), rw(lenrw)

*     ==================================================================
*     npfcon  calls the user-written routine  funcon  to evaluate
*     the nonlinear constraints and possibly their gradients.
*
*     This routine is an instance of sfgcon for SNOPT 5.3
*
*     09 Jan 1992: First version based on Minos 5.4 routine m6fcon.
*     28 Sep 1993: Test for mode = -1 exit from funcon.
*     22 Dec 1994: Simplified to conform to npfcon in npopt.
*     19 Mar 1997: First instantiated version for SNOPT 5.1.
*     17 Jul 1997: First thread-safe version.
*     26 Jul 1997: User workspace added.
*     11 Nov 1997: Current version of npfcon.
*     ==================================================================
      logical            scaled
      parameter         (nfCon1    = 189)
      parameter         (nfCon2    = 190)
*     ------------------------------------------------------------------
      iPrint    = iw( 12)
      iSumm     = iw( 13)
      lvlDer    = iw( 71)
      lvlScl    = iw( 75)
      lvlTim    = iw( 77)
      if (nState .ne. 1) then
         nGotg2  = iw(187)
      end if

      laScal    = iw(274)
      lxScl     = iw(275)
      lgConU    = iw(306)
      liy2      = iw(280)

      if (lvlTim .ge. 2) call s1time( 4, 0, iw, leniw, rw, lenrw )
      ierror   = 0
      scaled   = lvlScl .eq. 2
      mode     = modefg
      iw(nfCon1) = iw(nfCon1) + 1
      if (mode .eq. 1  .or.  mode .eq. 2) iw(nfCon2) = iw(nfCon2) + 1

*     ------------------------------------------------------------------
*     If the Jacobian is known and scaled, and if there are some
*     constant elements saved in gConU, we have to copy them into gCon.
*     ------------------------------------------------------------------
      if ( scaled ) then
         call dcopy ( nnJac, x         , 1, rw(lxScl), 1 )
         call ddscl ( nnJac, rw(laScal), 1, x        , 1 )
         if (lvlDer .ge. 2  .and.  nGotg2 .lt. neJac
     $                      .and.  modefg .eq. 2    ) then
            call dcopy ( neJac, rw(lgConU), 1, gCon, 1 )
         end if
      end if

      call iload ( nnCon, (1), iw(liy2), 1 )
      call fgcon ( mode, nnCon, nnJac, nnCon,
     $             iw(liy2), x, fCon, gCon, nState )

*     ------------------------------------------------------------------
*     To enable the passing of user workspace, replace the
*     call to fgcon with the following:
*
*     call fgcon ( mode, nnCon, nnJac, nnCon,
*    $             iw(liy2), x, fCon, gCon, nState,
*    $             cu, lencu, iu, leniu, ru, lenru )
*     ------------------------------------------------------------------

      if ( scaled ) then
         call dcopy ( nnJac, rw(lxScl)   , 1, x, 1 )
         call dddiv ( nnCon, rw(laScal+n), 1, fCon, 1 )
         if (modefg .eq. 2) then
            call s8sclJ( nnCon, nnJac, neJac, n, ne, nka, 
     $                   rw(laScal), ha, ka, gCon,
     $                   iw, leniw, rw, lenrw )
         end if
      end if

      if (mode .lt. 0) then
*        ---------------------------------------------------------------
*        The user may be saying the function is undefined (mode = -1)
*        or may just want to stop                         (mode < -1).
*        ---------------------------------------------------------------
         if (mode .eq. -1) then
            ierror = -1
         else
            ierror =  6
            if (iPrint .gt. 0) write(iPrint, 9060) iw(nfCon1)
            if (iSumm  .gt. 0) write(iSumm , 9060) iw(nfCon1)
         end if
      end if

      if (lvltim .ge. 2) call s1time(-4, 0, iw, leniw, rw, lenrw )

      return

 9060 format(  ' EXIT -- Termination requested by User in',
     $         ' constraint subroutine after', i8, '  calls')

*     end of npfcon
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine npfobj( modefg, ierror, nState,
     $                   nnObj, fgObj,
     $                   fObj, gObj, x, 
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      external           fgObj
      double precision   gObj(nnObj), x(nnObj)

      character*8        cu(lencu), cw(lencw)
      integer            iu(leniu), iw(leniw)
      double precision   ru(lenru), rw(lenrw)

*     ==================================================================
*     npfobj  calls the user-written routine  funobj  to evaluate
*     the nonlinear objective function and possibly its gradient.
*
*     Instantiation of sfgobj for SNOPT 5.3
*
*     The user functions funcon and/or funobj are called
*     via s6fcon and npfobj, using modefg to control the gradients
*     as follows:
*
*     If modefg = 2, gradients will be requested.
*     If modefg = 0, gradients will NOT be requested.  This is used
*     by s6srch during a function-only linesearch.  An extra call
*     with modefg = 2 is needed once the linesearch finds a good
*     point, but this may be cheaper than getting gradients every
*     time, e.g. if the user estimates gradients by differencing.
*
*     09 Jan 1992: First version based on Minos 5.4 routine m6fobj.
*     28 Sep 1993: Test for mode = -1 exit from funobj.
*     22 Dec 1994: Simplified to conform to npfobj in npopt.
*     17 Jul 1997: first thread-safe version.
*     26 Jul 1997: User workspace added.
*     11 Nov 1997: Current version of npfobj
*     ==================================================================
      logical            scaled
      parameter         (nfObj1    = 194)
      parameter         (nfObj2    = 195)
*     ------------------------------------------------------------------
      iPrint     = iw( 12)
      iSumm      = iw( 13)
      lvlScl     = iw( 75)
      lvlTim     = iw( 77)

      laScal     = iw(274)
      lxScl      = iw(275)

      if (lvlTim  .ge. 2) call s1time( 5, 0, iw, leniw, rw, lenrw )
      ierror     = 0
      scaled     = lvlScl .eq. 2
      mode       = modefg
      iw(nfObj1) = iw(nfObj1) + 1
      if (mode .eq. 1  .or.  mode .eq. 2) iw(nfObj2) = iw(nfObj2) + 1
 
      if ( scaled ) then
         call dcopy ( nnObj, x         , 1, rw(lxScl), 1 )
         call ddscl ( nnObj, rw(laScal), 1, x        , 1 )
      end if

      call fgobj ( mode, nnObj, x, fObj, gObj, nState )

*     ------------------------------------------------------------------
*     To enable the passing of user workspace, replace the
*     call to fgobj with the following:
*
*     call fgobj ( mode, nnObj, x, fObj, gObj, nState,
*    $             cu, lencu, iu, leniu, ru, lenru )
*     ------------------------------------------------------------------

      if ( scaled ) then
         call dcopy ( nnObj, rw(lxScl), 1, x, 1 )
         if (modefg .eq. 2) then
            call s8sclg( nnObj, rw(laScal), gObj, iw, leniw, rw, lenrw )
         end if
      end if

      if (mode .lt. 0) then
*        ---------------------------------------------------------------
*        The user may be saying the function is undefined (mode = -1)
*        or may just want to stop                         (mode < -1).
*        ---------------------------------------------------------------
         if (mode .eq. -1) then
            ierror = -1
         else
            ierror =  6
            call s1page( 2, iw, leniw )
            if (iPrint .gt. 0) write(iPrint, 9060) iw(nfObj1)
            if (iSumm  .gt. 0) write(iSumm , 9060) iw(nfObj1)
         end if
      end if

      if (lvlTim .ge. 2) call s1time(-5, 0, iw, leniw, rw, lenrw )

      return

 9060 format(  ' EXIT -- Termination requested by User in',
     $         ' objective subroutine after', i8, '  calls')

*     end of npfobj
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine npload( task, Start,
     $                   ldA, ldcJ, ldH, m, n, ncLin, nCon, nnCol, 
     $                   nb, nnCon, nnCon0,
     $                   hs, iState,
     $                   Alin, ne, nka, a, ha, ka,
     $                   bl, bu, bbl, bbu, c, cJac, cMul,
     $                   fCon, gCon, gObj, grad,
     $                   Hess, rc, x, xs,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*(*)      task
      character*(*)      Start
 
      integer            ha(ne), hs(nb), iState(n+nCon)
      integer            ka(nka)
      integer            iw(leniw)
      double precision   rw(lenrw)
      double precision   a(ne)
      double precision   Alin(ldA,*), bl(n+nCon), bu(n+nCon)
      double precision   bbl(nb), bbu(nb)
      double precision   c(nnCon0), cJac(ldcJ,*), cMul(n+nCon)
      double precision   fCon(nnCon0), gCon(nnCon0,*)
      double precision   grad(n), gObj(n), Hess(ldH,*)
      double precision   x(n), xs(nb)

      double precision   rc(nb)

*     ==================================================================
*     npload loads the problem into SNOPT format.
*
*     22 Mar 1997: First version of npload.
*     13 Oct 1997: Current version.
*     ==================================================================
      character*1        ch1
      parameter         (zero  = 0.0d+0)
*     ------------------------------------------------------------------
      plInfy    = rw( 70)

      if (task(1:1) .eq. 'L') then
*        ---------------------------------------------------------------
*        Load the snopt arrays.
*        Copy the bounds, x's first, linears next, then nonlinears.
*        ---------------------------------------------------------------
         call dcopy ( n, bl, 1, bbl, 1 ) 
         call dcopy ( n, bu, 1, bbu, 1 ) 

         if (ncLin .gt. 0) then
            call dcopy ( ncLin, bl(n+1)      , 1, bbl(n+nnCon+1), 1 ) 
            call dcopy ( ncLin, bu(n+1)      , 1, bbu(n+nnCon+1), 1 ) 
         end if

         if (nnCon .gt. 0) then
            call dcopy ( nnCon, bl(n+ncLin+1), 1, bbl(n+1)      , 1 ) 
            call dcopy ( nnCon, bu(n+ncLin+1), 1, bbu(n+1)      , 1 ) 
         end if

         if (nCon .eq. 0) then
            bbl(nb) = - plInfy  
            bbu(nb) =   plInfy 
         end if

         ch1    = Start(1:1)

         if (ch1 .eq. 'C'  .or.  ch1 .eq. 'c') then

*           Cold Start.

            do 110, j = 1, n
               xj = x(j)
               if (     xj .le. bl(j)) then
                  hs(j) = 4
               else if (xj .ge. bu(j)) then
                  hs(j) = 5
               else
                  hs(j) = 0
               end if
               xs(j) = xj
  110       continue

         else if (ch1 .eq. 'W'  .or.  ch1 .eq. 'w') then

*           Warm Start.
*           Input values of x and hs are used.

            call dcopy ( n, x, 1, xs, 1 )

            if (nnCon .gt. 0) then
               call dcopy ( nnCon, c, 1, xs(n+1), 1 )
            end if

            if (ncLin .gt. 0) then
               call dload ( ncLin, (zero), xs(n+nnCon+1), 1 )
               do 120, j = 1, n
                  call daxpy( ncLin, xs(j), Alin(1,j)    , 1,
     $                                        xs(n+nnCon+1), 1 )
  120          continue
            end if

            do 130, j = 1, n+nCon
               is    = istate(j)
               js    = 0
               if      (is .eq. 1) then
                  js = 0
               else if (is .eq. 2) then
                  js = 1
               end if
               hs(j) = js
  130       continue
         end if

*        Load the linear part of A with the linear constraints.

         
         if (nnCol .eq. 0) then

*           Sparse dummy row

            a (1) = zero
            ha(1) = 1
            ka(1) = 1

            do 200, j = 2, n+1
               ka(j) = 2
  200       continue

         else 
            ia    = 1
            ka(1) = 1
            do 310, j = 1, n
               do 300, i = 1, m
                  ha(ia) = i
                  if (i .le. nnCon) then
                     a(ia) = zero
                  else if (i .le. nCon) then
                     a(ia) = Alin(i-nnCon,j)
                  end if
                  ia     = ia + 1
  300          continue
               ka(j+1) = ia 
  310       continue
         end if

      else if (task(1:1) .eq. 'U') then
*        ---------------------------------------------------------------
*        Unload the SNOPT solution into the npopt arrays
*        Copy gCon, gObj into cJac and grad,
*        ---------------------------------------------------------------
         do 510, j = 1, n+nCon
            js = hs(j)
            is = 0
            if (     js .eq. 0) then
               is = 1
            else if (js .eq. 1) then
               is = 2
            end if
            istate(j) = is
  510    continue

*        ---------------------------------------------------------------
*        Copy gCon, gObj into cJac and grad,
*        ---------------------------------------------------------------
         call dcopy ( n, xs  , 1, x   , 1 )
         call dcopy ( n, gObj, 1, grad, 1 )
         call dcopy ( n    , rc           , 1, cMul           , 1 ) 
         if (ncLin .gt. 0)
     $   call dcopy ( ncLin, rc(n+nnCon+1), 1, cMul(n+1)      , 1 )

         if (nnCon .gt. 0) then
            call dcopy ( nnCon, fCon   , 1, c              , 1 )
            call dcopy ( nnCon, rc(n+1), 1, cMul(n+ncLin+1), 1 )

            do 530, j = 1, n
               do 520, i = 1, nnCon
                  cJac(i,j) = gCon(i,j)
  520          continue
  530       continue
         end if
      end if

*     end of  npload
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine npprnt( n, nb, ncLin, nnCon0, ldA, lprSol,
     $                   iState, A, bl, bu, c, cMul, x, r,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision(a-h,o-z)
      integer            iState(nb)
      integer            iw(leniw)
      double precision   rw(lenrw)
      double precision   A(ldA,*), c(nnCon0)
      double precision   bl(nb), bu(nb), cMul(nb), r(nb), x(n)

*     ==================================================================
*     npprnt  prints  x,  A*x, c(x), the bounds, the
*     multipliers, and the slacks (distance to the nearer bound).
*
*     22 Mar 1997: First version of npprnt.
*     22 Mar 1997: Current version.
*     ==================================================================
      character*1        key
      character*2        lstate(-2:4), state
      character*8        name
      character*102      line

      parameter         (zero  = 0.0d+0)
      data               lstate(-2) / '--' /, lstate(-1) / '++' /
      data               lstate( 0) / 'FR' /, lstate( 1) / 'LL' /
      data               lstate( 2) / 'UL' /, lstate( 3) / 'EQ' /
      data               lstate( 4) / 'TF' /
*     ------------------------------------------------------------------
      iPrint    = iw( 12)

      tolx      = rw( 56)
      plInfy    = rw( 70)

      if (iPrint .eq. 0  .or.  lprSol .eq. 0) return

      nplin  = n     + ncLin
      bigbnd = plInfy
      tol    = tolx

      write(iPrint, 1000) 'Variable       '
      name   = 'variable'
      nplin  = n + ncLin

      do 500, j = 1, nb
         b1     = bl(j)
         b2     = bu(j)
         wlam   = cMul(j)

         if (j .le. n) then
            rj  = x(j)
         else if (j .le. nplin) then
            i   = j - n
            rj  = ddot  ( n, A(i,1), ldA, x, 1 )
         else
            i   = j - nplin
            rj  = c(i)
         end if

         slk1   = rj - b1
         slk2   = b2 - rj
         r(j)   = rj

*        Reset istate if necessary.

         is     = istate(j)
         if (                  slk1 .lt. -tol) is = - 2
         if (                  slk2 .lt. -tol) is = - 1
         if (is .eq. 1  .and.  slk1 .gt.  tol) is =   0
         if (is .eq. 2  .and.  slk2 .gt.  tol) is =   0
         istate(j) = is
         state     = lstate(is)


         if (j .le. n) then
            number = j
         else if (j .le. nplin) then
            number = j - n
            if (number .eq. 1) then
               write(iPrint, 1000) 'Linear constrnt'
               name = 'lincon  '
            end if
         else
            number = j - nplin
            if (number .eq. 1) then
               write(iPrint, 1000) 'Nonlin constrnt'
               name = 'nlncon  '
            end if
         end if

*        ------------------------------------------------
*        Print a line for the jth variable or constraint.
*        ------------------------------------------------
         if (abs(slk1) .lt. abs(slk2)) then
            slk = slk1
            if (b1 .le. - bigbnd) slk = slk2 
         else
            slk = slk2
            if (b2 .ge.   bigbnd) slk = slk1 
         end if

*        Flag infeasibilities, primal and dual degeneracies, 
*        and active QP constraints that are loose in NP.
*      
         key    = ' ' 
         if (slk1 .lt. -tol  .or.       slk2  .lt. -tol) key = 'I'
         if (is   .eq.  0    .and.  abs(slk ) .le.  tol) key = 'D'
         if (is   .ge.  1    .and.  abs(wlam) .le.  tol) key = 'A'

         write(line, 2000) name, number, key, state, 
     $                     rj, b1, b2, wlam, slk

*        Reset special cases:
*           Infinite bounds
*           Zero bounds
*           Lagrange multipliers for inactive constraints
*           Lagrange multipliers for infinite bounds
*           Infinite slacks
*           Zero slacks

         if (b1  .le. - bigbnd) line(39: 54) = '      None      '
         if (b2  .ge.   bigbnd) line(55: 70) = '      None      '
         if (b1  .eq.   zero  ) line(39: 54) = '        .       '
         if (b2  .eq.   zero  ) line(55: 70) = '        .       '
         if (is  .eq.   0       .or.    
     $       wlam.eq.   zero  ) line(71: 86) = '        .       '
         if (b1  .le. - bigbnd  .and. 
     $       b2  .ge.   bigbnd) then
                                line(71: 86) = '                '
                                line(87:102) = '                '
         end if
         if (slk .eq.   zero  ) line(87:102) = '        .       '

         write(iPrint, '(a)') line
  500 continue

      return

 1000 format(//  1x,  a15, 2x, 'State', 6x, 'Value',
     $           7x, 'Lower bound', 5x, 'Upper bound',
     $           3x, 'Lagr multiplier', 4x, '   Slack' / )
 2000 format( 1x, a8, i6, 3x, a1, 1x, a2, 4g16.7, g16.4 )

*     end of npprnt
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine nptitl( title )

      character*30       title

*     ==================================================================
*     npinit sets the title.
*     ==================================================================

      title  = 'N P O P T  5.3-2    (Mar 1998)'
*---------------123456789|123456789|123456789|--------------------------

*     end of nptitl
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine npInit( iPrint, iSumm,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     npInit  is called by the user to do the following:
*     1. Open default files (Print, Summary).
*     2. Initialize title.
*     3. Set options to default values.
*
*     15 Nov 1991: First version.
*     14 Jul 1997: Thread-safe version.
*     21 Mar 1997: First version based on snopt routine snInit
*     14 Jul 1997: Thread-safe version.
*     02 Oct 1997: Character workspace added.
*     12 Feb 1998: Current version of npInit.
*     ==================================================================
      parameter         (maxru     =   2)
      parameter         (maxrw     =   3)
      parameter         (maxiu     =   4)
      parameter         (maxiw     =   5)
      parameter         (maxcu     =   6)
      parameter         (maxcw     =   7)
      parameter         (lvlTim    =  77)

      character*30       title
      character*30       dashes
      parameter         (lencw     = 500)
      character*8        cw(lencw)
      data               dashes /'=============================='/
*     ------------------------------------------------------------------

      if (leniw .lt. 500 .or. lenrw .lt. 500) then 
*        ---------------------------------------------------------------
*        Not enough workspace to do ANYTHING!
*        ---------------------------------------------------------------
         if (iPrint .gt. 0) write(iPrint, 9000) 
         if (iSumm  .gt. 0) write(iSumm , 9000) 
         if (iPrint .le. 0  .and.  iSumm  .le. 0) then
            write(*, 9000)
         end if
         go to 999
      end if

      iw( 12)   = iPrint
      iw( 13)   = iSumm

      iw(maxcu) = 500
      iw(maxiu) = 500
      iw(maxru) = 500
      iw(maxcw) = lencw
      iw(maxiw) = leniw
      iw(maxrw) = lenrw

      call nptitl( title )
      call s1init( title, iw, leniw, rw, lenrw )

      if (iPrint .gt. 0) then
         write (iPrint, '(  9x, a )') ' ', dashes, title, dashes
      end if
      if (iSumm .gt. 0) then
         write (iSumm , '(  1x, a )') ' ', dashes, title, dashes
      end if

*     ------------------------------------------------------------------
*     Set the options to default values.
*     npopt  will check the options later and maybe print them.
*     ------------------------------------------------------------------
      call s3undf( cw, lencw, iw, leniw, rw, lenrw )

*     ------------------------------------------------------------------
*     Initialize some global values.
*     ------------------------------------------------------------------
      iw(lvlTim) = 3

  999 return

 9000 format(  ' EXIT from npInit --',
     $         ' integer and real work arrays',
     $         ' must each have at least 500 elements')

*     end of npInit
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine npSpec( iSpecs, inform, 
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     npSpec  is called by the user to read the Specs file.
*
*     07 Feb 1998: First version.
*     12 Feb 1998: Current version of npSpec.
*     ==================================================================
      external           s3opt
      parameter         (lencw     = 500)
      character*8        cw(lencw)
*     ------------------------------------------------------------------
      iw( 11)   = iSpecs

      iPrint    = iw( 12)
      iSumm     = iw( 13)

      inform    = 0
      nCalls    = 1

*     ------------------------------------------------------------------
*     Read the Specs file.
*     npopt  will check the options later and maybe print them.
*     ------------------------------------------------------------------
      if (iSpecs .gt. 0) then
         call s3file( nCalls, iSpecs, s3opt,
     $                iPrint, iSumm, inform,
     $                cw, lencw, iw, leniw, rw, lenrw )
      end if

*     end of npSpec
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine npMem ( n, nclin, ncnln,
     $                   mincw, miniw, minrw,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     npMem   estimates the memory requirements for npopt,
*     using the values:
*     n      the number of variables (dimension of  x),
*
*     nclin  the number of linear constraints (rows of the matrix  A),
*
*     ncnln  the number of nonlinear constraints (dimension of  c(x)),
*
*     These values are used to compute the minimum required storage:
*     miniw, minrw.
*
*     Note: 
*     1. All default parameters must be set before calling npMem,
*        since some values affect the amount of memory required.
*
*     2. The arrays rw and iw hold  constants and work-space addresses.
*        They must have dimension at least 500.
*
*     3. This version of npMem does not allow user accessible
*        partitions of iw and rw.
*
*     01 May 1998: First version.
*     01 May 1998: Current version of npMem.
*     ==================================================================
      parameter         (lencw     = 500)
      character*8        cw(lencw)
*     ------------------------------------------------------------------
      iPrint  = iw( 12)
      iSumm   = iw( 13)

      if (lencw .lt. 500 .or. leniw .lt. 500 .or. lenrw .lt. 500) then 
*        ---------------------------------------------------------------
*        Not enough workspace to do ANYTHING!
*        ---------------------------------------------------------------
         if (iPrint .gt. 0) write(iPrint, 9000) 
         if (iSumm  .gt. 0) write(iSumm , 9000) 
         if (iPrint .le. 0  .and.  iSumm  .le. 0) then
            write(*, 9000)
         end if
         go to 999
      end if

      inform  = 0

*     Error messages from s8Mem are suppressed.

      iPrinx  = 0
      iSummx  = 0

*     Assign fake values for lencw, leniw, lenrw.
*     This will force s8Mem to estimate the memory requirements.

      llenrw  = 500
      lleniw  = 500
      llencw  = 500

*     Compute the problem dimensions.

      nCon       = nclin + ncnln

      if (nCon .eq. 0) then

*        The problem is unconstrained.
*        A dummy row of zeros will be included.

         nnCol = 0
         m     = 1
         ne    = 1
      else
         nnCol = n
         m     = nCon
         ne    = m*n 
      end if

      neJac      = ncnln*n
      nnCon      = ncnln
      nnJac      = nnCol
      nnObj      = n

*     An obligatory call to npInit has `undefined' all options.
*     Check the user-defined values and assign undefined values.
*     s8dflt needs various problem dimensions in iw.

      iw( 15) = m
      iw( 16) = n
      iw( 17) = ne
      iw( 21) = nnCon
      iw( 22) = nnJac
      iw( 23) = nnObj

      call s8dflt( 'Check optional parameters', 
     $             cw, llencw, iw, lleniw, rw, llenrw )

      mincw   = 501
      miniw   = 501
      minrw   = 501

      maxcw   = lencw
      maxiw   = leniw
      maxrw   = lenrw

      maxR    = iw( 56)
      maxS    = iw( 57)

      call s8Mem ( inform, iPrinx, iSummx, 
     $             m, n, ne, neJac,
     $             nnCon, nnJac, nnObj,
     $             maxR, maxS, 
     $             maxcw, maxiw, maxrw,
     $             llencw, lleniw, llenrw,
     $             mincw, miniw, minrw, iw )

  999 return

 9000 format(  ' EXIT from npMem --',
     $         ' character, integer and real work arrays',
     $         ' must each have at least 500 elements')

*     end of npMem.
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine npset ( buffer, iPrint, iSumm, inform,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*(*)      buffer
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     npset  decodes the option contained in  buffer.
*
*     The buffer is output to file iPrint, minus trailing blanks.
*     Error messages are output to files iPrint and iSumm.
*     Buffer is echoed to iPrint but normally not to iSumm.
*     It is echoed to iSumm before any error msg.
*
*     On entry,
*     iPrint is the print   file.  no output occurs if iPrint .le 0.
*     iSumm  is the Summary file.  no output occurs if iSumm  .le 0.
*     inform is the number of errors so far.
*
*     On exit,
*     inform is the number of errors so far.
*
*     27 Nov 1991: first version.
*     ==================================================================
      character*16       key
      character*8        cvalue
      parameter         (lencw     = 500)
      character*8        cw(lencw)
*     ------------------------------------------------------------------
      call s3opt ( .true., buffer, key, cvalue, ivalue, rvalue, 
     $             iPrint, iSumm, inform,
     $             cw, lencw, iw, leniw, rw, lenrw )

*     end of npset
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine npseti( buffer, ivalue, iPrint, iSumm, inform,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*(*)      buffer
      integer            ivalue
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     npseti decodes the option contained in  buffer // ivalue.
*     The parameters other than ivalue are as in npset.
*
*     27 Nov 1991: first version.
*     ==================================================================
      character*16       key
      character*72       buff72
      character*8        cvalue
      parameter         (lencw     = 500)
      character*8        cw(lencw)
*     ------------------------------------------------------------------
      write(key, '(i16)') ivalue
      lenbuf = len(buffer)
      buff72 = buffer
      buff72(lenbuf+1:lenbuf+16) = key
      call s3opt ( .true., buff72, key, cvalue, ivalue, rvalue, 
     $             iPrint, iSumm, inform,
     $             cw, lencw, iw, leniw, rw, lenrw )

*     end of npseti
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine npsetr( buffer, rvalue, iPrint, iSumm, inform,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*(*)      buffer
      double precision   rvalue
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     npsetr decodes the option contained in  buffer // rvalue.
*     The parameters other than rvalue are as in npset.
*
*     27 Nov 1991: first version.
*     ==================================================================
      character*16       key
      character*72       buff72
      character*8        cvalue
      parameter         (lencw     = 500)
      character*8        cw(lencw)
*     ------------------------------------------------------------------
      write(key, '(1p, e16.8)') rvalue
      lenbuf = len(buffer)
      buff72 = buffer
      buff72(lenbuf+1:lenbuf+16) = key
      call s3opt ( .true., buff72, key, cvalue, ivalue, rvalue,
     $             iPrint, iSumm, inform,
     $             cw, lencw, iw, leniw, rw, lenrw )

*     end of npsetr
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine npgeti( buffer, ivalue,
     $                   inform, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*(*)      buffer
      integer            ivalue

      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     npgeti gets the value of the option contained in  buffer.
*     The parameters other than ivalue are as in npset.
*
*     17 May 1998: first version.
*     ==================================================================
      character*16       key
      character*8        cvalue
      parameter         (lencw     = 500)
      character*8        cw(lencw)
*     ------------------------------------------------------------------
      call s3opt ( .false., buffer, key, cvalue, ivalue, rvalue,
     $             0, 0, inform,
     $             cw, lencw, iw, leniw, rw, lenrw )

*     end of npgeti
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine npgetr( buffer, rvalue,
     $                   inform, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*(*)      buffer
      double precision   rvalue

      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     npgetr gets the value of the option contained in  buffer.
*     The parameters other than rvalue are as in npset.
*
*     17 May 1998: first version.
*     ==================================================================
      character*16       key
      character*8        cvalue
      parameter         (lencw     = 500)
      character*8        cw(lencw)
*     ------------------------------------------------------------------
      call s3opt ( .false., buffer, key, cvalue, ivalue, rvalue,
     $             0, 0, inform,
     $             cw, lencw, iw, leniw, rw, lenrw )

*     end of npsetr
      end
