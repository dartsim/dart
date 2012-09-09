*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
*     File  sn12snzz.f
*
*     snopt    snfcon   snfobj
*     sntitl   snInit   snSpec   snMem
*     snset    snseti   snsetr   sngetc   sngeti   sngetr
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine snopt ( start, m, n, ne, nName,
     $                   nnCon, nnObj, nnJac,
     $                   iObj, ObjAdd, Prob,
     $                   fgCon, fgObj,
     $                   a, ha, ka, bl, bu, Names,
     $                   hs, xs, pi, rc, 
     $                   inform, mincw, miniw, minrw,
     $                   nS, nInf, sInf, Obj,
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      external           fgCon, fgObj
      character*(*)      start
      character*8        Prob, Names(nName)
      integer            ha(ne), hs(n+m)
      integer            ka(n+1)
      double precision   a(ne), bl(n+m), bu(n+m)
      double precision   xs(n+m), pi(m), rc(n+m)

      character*8        cu(lencu), cw(lencw)
      integer            iu(leniu), iw(leniw)
      double precision   ru(lenru), rw(lenrw)

*     ------------------------------------------------------------------
*     snopt is a Fortran subroutine for constrained nonlinear
*     optimization.  The constraints take the form
*
*                            (   x  )
*                      bl <= (      ) <= bu,
*                            ( F(x) )
*
*     where bl and bu are constant lower and upper bounds.
*
*     o If all constraints are linear, F = A x for some sparse matrix A.
*
*     o If all constraints are nonlinear, F = fcon(x) for some vector
*       fcon of smooth functions.
*
*     o In general, there is a mixture of constraints of the form
*                      ( fcon(x1) +  A2 x2 ),
*                      (   A3 x1  +  A4 x2 )
*       where the nonlinear variables x1 must appear first as shown.
*
*     o fcon(x1) and (optionally) its partial derivatives A1(x) are set
*       in subroutine funcon (see below).
*
*     o The matrices A2, A3, A4 and the sparsity pattern of A1(x) are
*       entered column-wise in the arrays a, ha, ka (below).
*
*     o Internally, the constraints are converted into the form
*
*           fcon(x1) +  A2 x2  - s1      = 0,     bl <= ( x ) <= bu    
*             A3 x1  +  A4 x2       - s2 = 0            ( s )
*
*       where s = (s1,s2)  and the components of (x,s) are the
*       variables and slacks respectively.
*
*     ------------------------------------------------------------------
*     NOTE: Before calling snopt, your calling program must call:
*     call snInit( iPrint, iSumm,
*    $             cw, lencw, iw, leniw, rw, lenrw )
*     This sets the default values of the optional parameters. You can
*     also alter the default values of iPrint and iSumm before snopt 
*     is used.  iPrint = 0, etc, is OK.
*     ------------------------------------------------------------------
*
*     ON ENTRY:
*
*     start   specifies how a starting basis (and certain other items)
*             are to be obtained.
*             start = 'Cold' means that Crash should be used to choose
*                      an initial basis, unless a basis file is given
*                      via Old basis, Insert or Load in the Specs file.
*             start = 'Basis file' means the same (but is more
*                      meaningful in the latter case).
*             start = 'Warm' means that a basis is already defined in hs
*                      (probably from an earlier call).
*
*     m       is the number of slacks (i.e., general constraints). 
*             For LP, QP or LC  problems this means the number of rows
*             in the constraint matrix A.
*             m > 0.
*
*             For problems with no general constraints, set m = 1 and
*             impose the constraint that the sum of the variables 
*             must lie between plus and minus infinity. This gives
*             A one ``free row'' that will not alter the solution.
*
*     n       is the number of variables, excluding slacks.
*             For LP problems, this is the number of columns in A.
*             n > 0.
*
*     ne      is the number of nonzero entries in A (including the
*             Jacobian for any nonlinear constraints).
*             ne gt 0.
*
*     nName   is the number of column and row names provided in the
*             array  Names.  If nName = 1, there are NO names.
*             Generic names will be used in the printed solution.
*             Otherwise, nName = n+m and all names must be provided.
*
*     nnCon   is the number of nonlinear constraints.
*             nnCon ge 0.
*
*             a nonzero nnCon defines the row dimension of the 
*             constraint Jacobian A1(x) defined in subroutine funcon.
*
*     nnObj   is the number of nonlinear Objective variables.
*             nnObj ge 0.
*
*     nnJac   is the number of nonlinear Jacobian variables.
*             If nnCon = 0, nnJac = 0.
*             if nnCon > 0, nnJac > 0.
*
*             a nonzero nnJac defines the column dimension of the 
*             constraint Jacobian A1(x) defined in subroutine funcon.
*
*     iObj    says which row of A is a free row containing a linear
*             objective vector  c  (iObj = 0 if none).
*             iObj = 0  or  nnCon < iObj le m.
*
*     ObjAdd  is a constant that will be added to the objective.
*             Typically ObjAdd = 0.0d+0.
*
*     Prob    is an 8-character name for the problem, used in the
*             output.  A blank name can be assigned if necessary.
*
*     a(ne)   is the constraint matrix (Jacobian), stored column-wise.
*             Every element of a(*) must be assigned a value.  Elements
*             in the nonlinear part (see NOTE 2 below) can be any dummy
*             value (e.g., zero) since they are initialized by SNOPT at
*             the first point that is feasible with respect to the
*             linear constraints.  The linear part of a(*) must contain
*             the constant Jacobian elements. 
*            
*     ha(ne)  is the list of row indices for each nonzero in a(*).
*
*     ka(n+1) is a set of pointers to the beginning of each column of
*             the constraint matrix within a(*) and ha(*).
*             Must have ka(1) = 1 and ka(n+1) = ne+1.
*
*  NOTES:  1. If the problem has a nonlinear objective,
*             the first nnObj columns of a and ha belong to the
*             nonlinear objective variables.
*             Subroutine funobj deals with these variables.
*          
*          2. If the problem has nonlinear constraints,
*             the first nnJac columns of a and ha belong to the
*             nonlinear Jacobian variables, and
*             the first nnCon rows of a and ha belong to the
*             nonlinear constraints.
*             Subroutine funcon deals with these variables and
*             constraints.
*          
*          3. If nnObj > 0 and nnJac > 0, the two sets of
*             nonlinear variables overlap.  The total number of
*             nonlinear variables is nnL = max( nnObj, nnJac ).
*          
*          4. The Jacobian forms the top left corner of a and ha.
*             If a Jacobian column j (1 le j le nnJac) contains
*             any entries a(k), ha(k) associated with nonlinear
*             constraints (1 le ha(k) le nnCon), those entries must
*             come before any other (linear) entries.
*          
*          5. The row indices ha(k) for a column may be in any order
*             (subject to Jacobian entries appearing first).
*             Subroutine funcon must define Jacobian entries in the
*             same order.
*          
*     bl(n+m) is the lower bounds on each variable (x,s).
*
*     bu(n+m) is the upper bounds on each variable (x,s).
*
*     Names(nName) is an character*8 array.
*             If nName =  1, Names is not used.  The printed solution
*             will use generic names for the columns and row.
*             If nName = n+m, Names(j) should contain an 8 character
*             name of the jth variable (j = 1, n+m).
*             If j = n+i, the jth variable is the ith row.
*
*     hs(n+m) sometimes contains a set of initial states for each
*             variable (x, s).  See the following NOTES.
*
*     xs(n+m) is a set of initial values for each variable (x, s).
*
*  NOTES:  1. If start = 'Cold' or 'Basis file' and a BASIS file
*             of some sort is to be input
*             (an OLD BASIS file, INSERT file or LOAD file),
*             hs and xs need not be set at all.
*
*          2. Otherwise, hs(1:n) must be defined for a cold start.
*             If nothing special is known about the problem, or if
*             there is no wish to provide special information,
*             you may set hs(j) = 0, xs(j) = 0.0d+0 for all j=1:n.
*             All variables will be eligible for the initial basis.
*        
*             Less trivially, to say that variable j will probably
*             be equal to one of its bounds,
*             set hs(j) = 4 and xs(j) = bl(j)
*             or  hs(j) = 5 and xs(j) = bu(j) as appropriate.
*        
*          3. For Cold starts with no basis file, a Crash procedure
*             is used to select an initial basis.  The initial basis
*             matrix will be triangular (ignoring certain small
*             entries in each column).
*             The values hs(j) = 0, 1, 2, 3, 4, 5 have the following
*             meaning:
*                
*             hs(j)    State of variable j during Crash
*        
*             0, 1, 3  Eligible for the basis.  3 is given preference.
*             2, 4, 5  Ignored.
*        
*             After Crash, hs(j) = 2 entries are made superbasic.
*             Other entries not selected for the basis are made
*             nonbasic at the value xs(j) if bl(j) <= xs(j) <= bu(j),
*             or at the value bl(j) or bu(j) closest to xs(j).
*
*          4. For Warm starts, all of hs(1:n+m) is assumed to be
*             set to the values 0, 1, 2 or 3 from some previous call.
*        
*     pi(m)   contains an estimate of the vector of Lagrange multipliers
*             (shadow prices) for the NONLINEAR constraints.  The first
*             nnCon components must be defined.  They will be used as
*             lambda in the subproblem objective function for the first
*             major iteration.  If nothing is known about lambda,
*             set pi(i) = 0.0d+0, i = 1 to nnCon.
*
*     nS      need not be specified for Cold starts,
*             but should retain its value from a previous call
*             when a Warm start is used.
*
*
*     ON EXIT:
*
*     hs(n+m) is the final state vector:
*
*                hs(j)    State of variable j    Normal value of xs(j)
*
*                  0      nonbasic               bl(j)
*                  1      nonbasic               bu(j)
*                  2      superbasic             Between bl(j) and bu(j)
*                  3      basic                  ditto
*
*             Very occasionally there may be nonbasic variables for
*             which xs(j) lies strictly between its bounds.
*             If nInf = 0, basic and superbasic variables may be outside
*             their bounds by as much as the Feasibility tolerance.
*             Note that if Scale is specified, the Feasibility tolerance
*             applies to the variables of the SCALED problem. 
*             In this case, the variables of the original problem may be
*             as much as 0.1 outside their bounds, but this is unlikely
*             unless the problem is very badly scaled.
*
*     xs(n+m) contains the final variables and slacks (x, s).
*
*     pi(m)   is the vector of Lagrange multipliers (shadow prices)
*             for the general constraints.
*
*     rc(n+m) is a vector of reduced costs: rc = g - (A -I)'pi, where g
*             is the gradient of the objective if xs is feasible
*             (or the gradient of the Phase-1 objective otherwise).
*             If nInf = 0, the last m entries are pi.
*
*     inform  says what happened; see the User's Guide.
*             A summary of possible values follows:
*
*             Inform   Meaning
*
*                0     Optimal solution found.
*                1     The problem is infeasible.
*                2     The problem is unbounded (or badly scaled).
*                3     Too many iterations.
*                5     The Superbasics limit is too small.
*                4     Final solution is feasible, but final optimality 
*                      could not quite be achieved,
*                6     Subroutine funobj or funcon requested termination
*                      by returning mode < 0.
*                7     Subroutine funobj seems to be giving incorrect
*                      gradients.
*                8     Subroutine funcon seems to be giving incorrect
*                      gradients.
*                9     The current point cannot be improved.
*               10     Numerical error in trying to satisfy the linear
*                      constraints (or the linearized nonlinear
*                      constraints).  The basis is very ill-conditioned.
*
*               20     Not enough storage for the basis factorization.
*               21     Error in basis package.
*               22     The basis is singular after several attempts to
*                      factorize it (and add slacks where necessary).
*
*               30     An OLD BASIS file had dimensions that did not
*                      match the current problem.
*               32     System error.  Wrong number of basic variables.
*
*               41     Not enough storage to hold SNOPT local variables.
*               42     Not enough char*8  storage to solve the problem.
*               43     Not enough integer storage to solve the problem.
*               44     Not enough real    storage to solve the problem.
*
*     mincw   says how much character storage is needed to solve the
*             problem.  If inform = 42, the work array cw(lencw) was 
*             too small.  SNOPT may be called again with lencw suitably 
*             larger than mincw.
*
*     miniw   says how much integer storage is needed to solve the
*             problem.  If inform = 43, the work array iw(leniw) was too
*             small.  SNOPT may be called again with leniw suitably 
*             larger than miniw.  (The bigger the better, since it is
*             not certain how much storage the basis factors need.)
*
*     minrw   says how much real storage is needed to solve the problem.
*             If inform = 44, the work array rw(lenrw) was too small.
*             (See the comments above for miniw.)
*
*     nS      is the final number of superbasics.
*
*     nInf    is the number of infeasibilities.
*
*     sInf    is the sum    of infeasibilities.
*
*     Obj     is the value of the nonlinear part of the objective.
*             If nInf = 0, Obj includes the nonlinear objective if any.
*             If nInf > 0, Obj is just the linear objective if any.
*
*     cu(lencu), iu(leniu), ru(lenru)  are character, integer and real
*             arrays of USER workspace.  These arrays are available to
*             pass data to the user-defined routines funobj and funcon. 
*             If no workspace is required, you can either use dummy
*             arrays for cu, iu and ru, or use cw, iw and rw
*             (see below).
*
*     cw(lencw), iw(leniw), rw(lenrw)  are character*8, integer and real
*             arrays of workspace used by SNOPT.
*             lencw  should be about at least 500.
*             leniw  should be about max( 500, 20(m+n) ) or larger.
*             lenrw  should be about max( 500, 40(m+n) ) or larger.
*
*     12 Nov 1994: Workspace separated into iw(*) and rw(*).
*     08 Aug 1996: First Min Sum version.
*     17 Jul 1997: first thread-safe version.
*     26 Jul 1997: User workspace added.
*     02 Oct 1997: Character workspace added.
*     16 May 1998: Current version of snopt.
*     ==================================================================
      parameter         (maxru     =   2)
      parameter         (maxiu     =   4)
      parameter         (maxcu     =   6)

      parameter         (iPrint    =  12)
      parameter         (iSumm     =  13)
      parameter         (neJac     =  20)

      parameter         (mName     =  51)
      external           snfcon, snfobj
*     ------------------------------------------------------------------

      if (lencw .lt. 500 .or. leniw .lt. 500 .or. lenrw .lt. 500) then 
*        ---------------------------------------------------------------
*        Not enough workspace to do ANYTHING!
*        ---------------------------------------------------------------
         inform = 41
         mincw  = 500
         miniw  = 500
         minrw  = 500
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

*     Load the iw array with various problem dimensions.

      iw( 15)    = m
      iw( 16)    = n
      iw( 17)    = ne
      iw( 21)    = nnCon
      iw( 23)    = nnObj
      iw( 22)    = nnJac
      iw(218)    = iObj

*     The obligatory call to snInit has already set the defaults.
*     Check that the optional parameters have sensible values.

      call s8dflt( 'Check optional parameters', 
     $             cw, lencw, iw, leniw, rw, lenrw )

*     ------------------------------------------------------------------
*     Print the options if iPrint > 0, Print level > 0 and lvlPrm > 0.
*     ------------------------------------------------------------------
      call s8dflt( 'Print the options', 
     $             cw, lencw, iw, leniw, rw, lenrw )

*     ------------------------------------------------------------------
*     Compute the storage requirements for SNOPT  from the following
*     variables:
*         m,      n,    ne
*         maxR ,  maxS, nnL
*         nnObj,
*         neJac, nnCon, nnJac
*     All have to be known exactly before calling s8Mem.
*     The only one in doubt is neJac, the number of Jacobian elements.
*     Count them here.
*     ------------------------------------------------------------------
      iw(neJac) = 0
      if (nnCon .gt. 0) then
         last = ka(nnJac+1) - 1
         if (nnCon .eq. m) then
            iw(neJac) = last
         else
            do 100, k = 1, last
               i      = ha(k)
               if (i .le. nnCon) iw(neJac) = iw(neJac) + 1
  100       continue         
         end if
      end if

      iw(neJac) = max( 1, iw(neJac) )
 
*     Fetch the limits on SNOPT real and integer workspace.
*     They are used to define the limits on LU workspace. 

      maxrw = iw(  3)          ! Extent of SNOPT real    workspace
      maxiw = iw(  5)          ! Extent of SNOPT integer workspace
      maxcw = iw(  7)          ! Extent of SNOPT char*8  workspace

      mincw = iw(maxcu) + 1
      miniw = iw(maxiu) + 1
      minrw = iw(maxru) + 1

      maxR  = iw( 56)
      maxS  = iw( 57)

      call s8Mem ( ierror, iw(iPrint), iw(iSumm), 
     $             m, n, ne, iw(neJac),
     $             nnCon, nnJac, nnObj,
     $             maxR, maxS, 
     $             maxcw, maxiw, maxrw,
     $             lencw, leniw, lenrw,
     $             mincw, miniw, minrw, iw )
      if (ierror .ne. 0) then
         inform = ierror
         go to 999
      end if

*     ------------------------------------------------------------------
*     Copy the problem name into the work array.
*     ------------------------------------------------------------------
      cw(mName) = Prob

*     ------------------------------------------------------------------
*     Solve the problem.
*     ------------------------------------------------------------------
      nb     = n + m
      nka    = n + 1
      call s8solv( start, 
     $             m, n, nb, ne, nka, nName, 
     $             iObj, ObjAdd, fObj, Objtru,
     $             nInf, sInf,
     $             snfcon, fgCon, snfobj, fgObj,
     $             a, ha, ka, bl, bu, Names,
     $             hs, xs, pi, rc,
     $             inform, nMajor, nS, 
     $             cu, lencu, iu, leniu, ru, lenru, 
     $             cw, lencw, iw, leniw, rw, lenrw )

      Obj    = ObjTru

*     Print times for all clocks (if lvlTim > 0).

      call s1time( 0, 2, iw, leniw, rw, lenrw )

  999 return

 9000 format(  ' EXIT -- SNOPT character, integer and real',
     $         ' work arrays each must have at least 500 elements')

*     end of snopt
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine snfcon( modefg, ierror, nState, 
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
*     snfcon  calls the user-written routine  fgCon  to evaluate
*     the nonlinear constraints and possibly their gradients.
*
*     Instance of sfgcon for SNOPT 5.3
*
*
*     09-Jan 1992: First version based on Minos 5.4 routine m6fcon.
*     28-Sep 1993: Test for mode = -1 exit from funcon.
*     22-Dec 1994: Simplified to conform to npfcon in npopt.
*     19-Mar 1997: First instantiated version for SNOPT 5.1.
*     17-Jul 1997: First thread-safe version.
*     02-Oct 1997: Character workspace added.
*     24-Dec 1997: Current version.
*     ==================================================================
      logical            scaled
      parameter         (nfCon1    = 189)
      parameter         (nfCon2    = 190)
*     ------------------------------------------------------------------
      iPrint     = iw( 12)
      iSumm      = iw( 13)
      lvlDer     = iw( 71)
      lvlScl     = iw( 75)
      lvlTim     = iw( 77)
      if (nState .ne. 1) then
         nGotg2  = iw(187)
      end if

      laScal     = iw(274)
      lxScl      = iw(275)
      lgConU     = iw(306)

      if (lvlTim .ge. 2) call s1time( 4, 0, iw, leniw, rw, lenrw )
      ierror     = 0
      scaled     = lvlScl .eq. 2

      mode       = modefg
      iw(nfCon1) = iw(nfCon1) + 1
      if (mode .gt. 0)
     $iw(nfCon2) = iw(nfCon2) + 1

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

      call fgcon ( mode, nnCon, nnJac, neJac,
     $             x, fCon, gCon, nState,
     $             cu, lencu, iu, leniu, ru, lenru )

      if ( scaled ) then
         call dcopy ( nnJac, rw(lxScl)   , 1, x, 1 )
         call dddiv ( nnCon, rw(laScal+n), 1, fCon, 1 )
         if (modefg .gt. 0) then
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

*     end of snfcon
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine snfobj( modefg, ierror, nState, nnObj, 
     $                   fgObj,
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
*     snfobj  calls the user-written routine  fgObj  to evaluate
*     the nonlinear objective function and possibly its gradient.
*
*     Instance of sfgobj for SNOPT 5.3
*
*     The user functions funcon and/or funobj are called
*     via s6fcon and snfobj, using modefg to control the gradients
*     as follows:
*
*     If modefg = 2, gradients will be requested.
*     If modefg = 0, gradients will NOT be requested.  This is used
*     by s6srch during a function-only linesearch.  An extra call
*     with modefg = 2 is needed once the linesearch finds a good
*     point, but this may be cheaper than getting gradients every
*     time, e.g. if the user estimates gradients by differencing.
*
*     09-Jan 1992: First version based on Minos 5.4 routine m6fobj.
*     28-Sep 1993: Test for mode = -1 exit from funobj.
*     22-Dec 1994: Simplified to conform to npfobj in npopt.
*     16-Jan 1997: fgObj added as argument.
*     19-Mar 1997: First instantiated version for SNOPT 5.1.
*     17-Jul 1997: First thread-safe version.
*     02-Oct 1997: Character workspace added.
*     11-Nov 1997: Current version.
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
      if (mode .gt. 0)
     $iw(nfObj2) = iw(nfObj2) + 1
 
      if ( scaled ) then
         call dcopy ( nnObj, x         , 1, rw(lxScl), 1 )
         call ddscl ( nnObj, rw(laScal), 1, x        , 1 )
      end if

      call fgobj ( mode, nnObj,
     $             x, fObj, gObj, nState,
     $             cu, lencu, iu, leniu, ru, lenru )

      if ( scaled ) then
         call dcopy ( nnObj, rw(lxScl), 1, x, 1 )
         if (modefg .gt. 0) then
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

*     end of snfobj
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine sntitl( title )

      character*30       title

*     ==================================================================
*     sntitl sets the title for SNOPT.
*     ==================================================================

      title  = 'S N O P T  5.3-2    (May 1998)'
*---------------123456789|123456789|123456789|--------------------------

*     end of sntitl
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine snInit( iPrint, iSumm,
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     snInit  is called by the user (not by snopt3) to do the following:
*     1. Open default files (Print, Summary).
*     2. Initialize title.
*     3. Set options to default values.
*
*     15 Nov 1991: First version.
*     14 Jul 1997: Thread-safe version.
*     02 Oct 1997: Character workspace added.
*     15 Feb 1998: Current version of snInit.
*     ==================================================================
      parameter         (maxru     =   2)
      parameter         (maxrw     =   3)
      parameter         (maxiu     =   4)
      parameter         (maxiw     =   5)
      parameter         (maxcu     =   6)
      parameter         (maxcw     =   7)

      parameter         (nnCon     =  21)
      parameter         (nnJac     =  22)
      parameter         (nnObj     =  23)

      parameter         (lvlTim    =  77)

      character*30       title
      character*30       dashes
      data               dashes /'=============================='/
*     ------------------------------------------------------------------

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

      iw( 12)   = iPrint
      iw( 13)   = iSumm

      iw(maxcu) = 500
      iw(maxiu) = 500
      iw(maxru) = 500
      iw(maxcw) = lencw
      iw(maxiw) = leniw
      iw(maxrw) = lenrw

*     These dimensions need to be initialized for an MPS run.

      iw(nnCon) = 0
      iw(nnJac) = 0
      iw(nnObj) = 0

      call sntitl( title )
      call s1init( title, iw, leniw, rw, lenrw )

      if (iPrint .gt. 0) then
         write (iPrint, '(  9x, a )') ' ', dashes, title, dashes
      end if
      if (iSumm .gt. 0) then
         write (iSumm , '(  1x, a )') ' ', dashes, title, dashes
      end if

*     ------------------------------------------------------------------
*     Set the options to default values.
*     snopt  will check the options later and maybe print them.
*     ------------------------------------------------------------------
      call s3undf( cw, lencw, iw, leniw, rw, lenrw )

*     ------------------------------------------------------------------
*     Initialize some global values.
*     ------------------------------------------------------------------
      iw(lvlTim) = 3

  999 return

 9000 format(  ' EXIT from snInit --',
     $         ' character, integer and real work arrays',
     $         ' must each have at least 500 elements')

*     end of snInit
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine snSpec( iSpecs, inform, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     snSpec  is called by the user to read the Specs file.
*
*     07-Feb 1998: First version.
*     07-Feb 1998: Current version of snSpec.
*     ==================================================================
      external           s3opt
*     ------------------------------------------------------------------
      iw( 11)   = iSpecs

      iPrint    = iw( 12)
      iSumm     = iw( 13)

      inform    = 0
      nCalls    = 1

*     ------------------------------------------------------------------
*     Read the Specs file.
*     snopt  will check the options later and maybe print them.
*     ------------------------------------------------------------------
      if (iSpecs .gt. 0) then
*     zp change; Thu Jun  4 18:09:42 EDT 1998
         close(iSpecs)
         call s3file( nCalls, iSpecs, s3opt,
     $                iPrint, iSumm, inform,
     $                cw, lencw, iw, leniw, rw, lenrw )
      end if

*     end of snSpec
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine snMem ( m, n, ne, neJac,
     $                   nnCon, nnJac, nnObj,
     $                   mincw, miniw, minrw,
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     snMem   estimates the memory requirements for snopt,
*     using the values:
*        m    , n    , ne    neJac
*        nnObj, nnCon, nnJac     
*
*     These values are used to compute the minimum required storage:
*     mincw, miniw, minrw.
*
*     Note: 
*     1. All default parameters must be set before calling snMem,
*        since some values affect the amount of memory required.
*
*     2. The arrays rw and iw hold  constants and work-space addresses.
*        They must have dimension at least 500.
*
*     3. This version of snMem does not allow user accessible
*        partitions of cw, iw and rw.
*
*     29 Mar 1998: First version.
*     29 Mar 1998: Current version of snMem.
*     ==================================================================
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

*     An obligatory call to snInit has `undefined' all options.
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

 9000 format(  ' EXIT from snMem --',
     $         ' character, integer and real work arrays',
     $         ' must each have at least 500 elements')

*     end of snMem.
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine snset ( buffer, iPrint, iSumm, inform,
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*(*)      buffer

      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     snset  decodes the option contained in  buffer.
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
*     ------------------------------------------------------------------
      call s3opt ( .true., buffer, key, cvalue, ivalue, rvalue, 
     $             iPrint, iSumm, inform,
     $             cw, lencw, iw, leniw, rw, lenrw )

*     end of snset
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine snseti( buffer, ivalue, iPrint, iSumm, inform,
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*(*)      buffer
      integer            ivalue

      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     snseti decodes the option contained in  buffer // ivalue.
*     The parameters other than ivalue are as in snset.
*
*     27 Nov 1991: first version.
*     ==================================================================
      character*16       key
      character*72       buff72
      character*8        cvalue
*     ------------------------------------------------------------------
      write(key, '(i16)') ivalue
      lenbuf = len(buffer)
      buff72 = buffer
      buff72(lenbuf+1:lenbuf+16) = key
      call s3opt ( .true., buff72, key, cvalue, ivalue, rvalue, 
     $             iPrint, iSumm, inform,
     $             cw, lencw, iw, leniw, rw, lenrw )

*     end of snseti
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine snsetr( buffer, rvalue, iPrint, iSumm, inform,
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*(*)      buffer
      double precision   rvalue

      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     snsetr decodes the option contained in  buffer // rvalue.
*     The parameters other than rvalue are as in snset.
*
*     27 Nov 1991: first version.
*     ==================================================================
      character*16       key
      character*72       buff72
      character*8        cvalue
*     ------------------------------------------------------------------
      write(key, '(1p, e16.8)') rvalue
      lenbuf = len(buffer)
      buff72 = buffer
      buff72(lenbuf+1:lenbuf+16) = key
      call s3opt ( .true., buff72, key, cvalue, ivalue, rvalue,
     $             iPrint, iSumm, inform,
     $             cw, lencw, iw, leniw, rw, lenrw )

*     end of snsetr
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine sngetc( buffer, cvalue,
     $                   inform, cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*(*)      buffer
      character*8        cvalue

      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     sngetc gets the value of the option contained in  buffer.
*     The parameters other than cvalue are as in snset.
*
*     17 May 1998: first version.
*     ==================================================================
      character*16       key
*     ------------------------------------------------------------------
      call s3opt ( .false., buffer, key, cvalue, ivalue, rvalue,
     $             0, 0, inform,
     $             cw, lencw, iw, leniw, rw, lenrw )

*     end of sngetc
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine sngeti( buffer, ivalue,
     $                   inform, cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*(*)      buffer
      integer            ivalue

      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     sngeti gets the value of the option contained in  buffer.
*     The parameters other than ivalue are as in snset.
*
*     17 May 1998: first version.
*     ==================================================================
      character*16       key
      character*8        cvalue
*     ------------------------------------------------------------------
      call s3opt ( .false., buffer, key, cvalue, ivalue, rvalue,
     $             0, 0, inform,
     $             cw, lencw, iw, leniw, rw, lenrw )

*     end of sngeti
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine sngetr( buffer, rvalue,
     $                   inform, cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*(*)      buffer
      double precision   rvalue

      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     sngetr gets the value of the option contained in  buffer.
*     The parameters other than rvalue are as in snset.
*
*     17 May 1998: first version.
*     ==================================================================
      character*16       key
      character*8        cvalue
*     ------------------------------------------------------------------
      call s3opt ( .false., buffer, key, cvalue, ivalue, rvalue,
     $             0, 0, inform,
     $             cw, lencw, iw, leniw, rw, lenrw )

*     end of snsetr
      end
