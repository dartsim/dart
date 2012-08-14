*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
*     File  sn12sqzz.f
*
*     sqopt    sqHx    
*     sqtitl   sqInit   sqSpec   sqMem 
*     sqset    sqseti   sqsetr   sqgetc   sqgeti   sqgetr
*     nullHx
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine sqopt ( start, qpHx, m,
     $                   n, ne, nName, lenc, ncolH,
     $                   iObj, ObjAdd, Prob,
     $                   a, ha, ka, bl, bu, c, Names,
     $                   hElast, hs, xs, pi, rc,
     $                   inform, mincw, miniw, minrw,
     $                   nS, nInf, sInf, Obj,
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      external           qpHx
      character*(*)      start
      character*8        Prob
      character*8        Names(nName)
      integer            ha(ne), hElast(n+m), hs(n+m)
      integer            ka(n+1)
      double precision   a(ne), bl(n+m), bu(n+m), xs(n+m)
      double precision   c(*)
      double precision   pi(m), rc(n+m)

      character*8        cu(lencu), cw(lencw)
      integer            iu(leniu), iw(leniw)
      double precision   ru(lenru), rw(lenrw)

*     ------------------------------------------------------------------
*     sqopt    solves the linear/quadratic programming problem
*
*     Min/max     ObjAdd + <a(iObj),x> + <c,x> + half <x,H*x>
*        x
*     subject to linear constraints, and upper and lower bounds on x
*     (see below).
*
*     ObjAdd  is a constant scalar.
*     a(iObj) is the iObj-th row of the constraint matrix A (see below).
*     c       is a constant vector.
*     H       is a constant symmetric matrix, defined implicitly in the 
*             user-supplied subroutine qpHx.
*             o Subroutine qpHx must evaluate products of H with a given
*               vector x. This implies that H is defined as an operator,
*               and need never be defined explicitly.
*             o If H has zero rows and columns, it is most efficient to
*               order the variables so that
*                      Hx = ( H1  0 )( X1 ) = ( H1 x1 ),
*                           ( 0   0 )( X2)    (   0   )
*               where the nonlinear variables x1 appear first as shown.
*
*     The constraints take the form
*
*                            (  x )
*                      bl <= (    ) <= bu,
*                            ( Ax )
*
*     where bl and bu are constant lower and upper bounds. 
*
*     Internally, the constraints are rewritten as
*
*                                         ( x )
*                   Ax - s = 0,     bl <= (   ) <= bu,
*                                         ( s )
*
*     where s is an m-vector of slack variables.
*     components of (x,s) are the variables and slacks respectively.
*     The sparse matrix A is entered column-wise in the arrays
*     a, ha, ka (below).
*
*     ------------------------------------------------------------------
*     NOTE: Before calling sqopt, the calling program must call:
*     call sqInit( iPrint, iSumm,
*    $             cw, lencw, iw, leniw, rw, lenrw )
*     This sets the default values of the optional parameters. You can
*     also alter the default values of iPrint and iSumm before sqopt 
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
*     m       is the number of rows in the constraint matrix A.
*             m > 0.
*
*     n       is the number of variables, excluding slacks.
*             For LP problems, this is the number of columns in A.
*             n > 0.
*
*     ne      is the number of nonzero entries in A.
*             ne > 0.
*
*     nName   is the number of column and row names provided in the
*             array  Names.  If nName = 1, there are NO names.
*             Generic names will be used in the printed solution.
*             Otherwise, nName = n+m and all names must be provided.
*
*     lenc    is the number of elements in the constant objective c.
*             lenc ge 0.
*
*     ncolH   is the number of leading nonzero columns of the
*             QP Hessian.  
*             If ncolH > 0, the subroutine qpHx must provide the
*             matrix-product Hx.
*             ncolH ge 0.
*
*     iObj    says which row of A is a free row containing a linear
*             objective vector  c  (iObj = 0 if none).
*             iObj = 0  or  iObj le m.
*
*     ObjAdd  is a constant that will be added to the objective before
*             printing.  Typically,  ObjAdd = 0.0d+0.
*
*     Prob    is an 8-character name for the problem, used in the
*             output.  A blank name can be assigned if necessary.
*
*     a(ne)   is the constraint matrix, stored column-wise.
*
*     ha(ne)  is the list of row indices for each nonzero in a(*).
*
*     ka(n+1) is a set of pointers to the beginning of each column of
*             the constraint matrix within a(*) and ha(*).
*             MUST HAVE ka(1) = 1 AND ka(n+1) = ne+1.
*
*  NOTES:  1. If lenc > 0, the first lenc columns of a and ha belong
*             to variables corresponding to the constant
*             objective term c.
*          
*          2. If the problem has a quadratic objective,
*             The first ncolH columns of a and ha belong to variables
*             corresponding to the nonzero block of the QP Hessian.
*             Subroutine qpHx deals with these variables.
*             
*          3. If lenc > 0 and ncolH > 0, the two sets of
*             objective variables overlap.  The total number of
*             objective variables is nQP = max( lenc, ncolH ).
*          
*          4. The row indices ha(k) for a column may be in any order.
*          
*     bl(n+m) is the lower bounds on the variables and slacks (x, s).
*
*     bu(n+m) is the upper bounds on (x, s).
*
*     Names(nName) is an character*8 array.
*             If nName =  1, Names is not used.  The printed solution
*             will use generic names for the columns and row.
*             If nName = n+m, Names(j) should contain an 8 character
*             name of the jth variable (j = 1, n+m).
*             If j = n+i, the jth variable is the ith row.
*
*     hElast(n+m) indicate if the variable can violate its bound in 
*             Elastic mode.
*             if hElast(j) = 0, variable j is non-elastic and must not
*                               be violated.
*             if hElast(j) = 1, variable j can violate its lower bound.
*             if hElast(j) = 2, variable j can violate its upper bound.
*             if hElast(j) = 3, variable j can violate either its 
*                                          lower or upper bound.
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
*     xs(n+m) is the final variables and slacks (x, s).
*
*     pi(m)   is the vector of Lagrange multipliers (shadow prices)
*             for the general constraints.
*
*     rc(n+m) is a vector of reduced costs: rc = g - (A -I)'*pi, where g
*             is the gradient of the objective if xs is feasible
*             (or the gradient of the Phase 1 objective otherwise).
*             If nInf = 0, the last m entries are pi.
*
*     inform  says what happened; see the User's Guide.
*             A summary of possible values follows:
*
*             inform   Meaning
*
*                0     Optimal solution found
*                1     The problem is infeasible
*                2     The problem is unbounded (or badly scaled)
*                3     Too many iterations
*                4     Weak solution
*                5     The superbasics limit is too small.
*                6     H not positive semi-definite
*
*               10     Numerical error in trying to satisfy the linear
*                      constraints.  The basis is very ill-conditioned.
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
*               41     Not enough storage to hold SQOPT local variables.
*               42     Not enough char    storage to solve the problem.
*               43     Not enough integer storage to solve the problem.
*               44     Not enough real    storage to solve the problem.
*
*     mincw   says how much character storage is needed to solve the
*             problem.  If inform = 42, the work array cw(lencw) was 
*             too small.  SQOPT may be called again with lencw suitably 
*             larger than mincw.
*
*     miniw   says how much integer storage is needed to solve the
*             problem.  If inform = 43, the work array iw(leniw) was too
*             small.  SQOPT  may be called again with leniw suitably 
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
*     Obj     is the value of the QP objective function.
*             Obj does NOT include ObjAdd or the objective row.
*             If nInf = 0, Obj includes the quadratic objective if any.
*             If nInf > 0, Obj is just the linear objective if any.
*
*     cu(lencu), iu(leniu), ru(lenru)  are character, integer and real
*             arrays of USER workspace.  These arrays are available to
*             pass data to the user-defined routine qpHx.
*             If no workspace is required, you can either use dummy
*             arrays for cu, iu and ru, or use cw, iw and rw
*             (see below).
*
*     cw(lencw), iw(leniw), rw(lenrw)  are character*8, integer and real
*             arrays of workspace used by SQOPT.
*             lencw  should be about at least 500.
*             leniw  should be about max( 500, 10(m+n) ) or larger.
*             lenrw  should be about max( 500, 20(m+n) ) or larger.
*
*     12 Nov 1994: Workspace separated into iw(*) and rw(*).
*     20 Jul 1996: Slacks changed to be the row value.
*     09 Aug 1996: First Min Sum version.
*     17 Jul 1997: Thread-safe version.
*     26 Jul 1997: User workspace added.
*     02 Oct 1997: Character workspace added.
*     16 May 1998: Current version of sqopt.
*     ==================================================================
      double precision   dummyb(1), dummyx(1)
      parameter         (maxru     =   2)
      parameter         (maxiu     =   4)
      parameter         (maxcu     =   6)

      parameter         (iPrint    =  12)
      parameter         (iSumm     =  13)

      parameter         (mName     =  51)

      external           sqHx
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
*     First record problem dimensions for smart users to access in iw.

      nnCon      = 0            ! Not used in sqopt
      nnObj      = 0            ! ditto
      nnJac      = 0            ! ditto

      iw( 15)    = m
      iw( 16)    = n
      iw( 17)    = ne
      iw( 21)    = nnCon
      iw( 23)    = nnObj
      iw( 22)    = nnJac
      iw( 26)    = lenc
      iw( 27)    = ncolH
      iw(218)    = iObj

*     The obligatory call to sqInit has already set the defaults.
*     Check that the optional parameters have sensible values.

      call s5dflt( 'Check optional parameters', 
     $             cw, lencw, iw, leniw, rw, lenrw )

*     ------------------------------------------------------------------
*     Print the options if iPrint > 0, Print level > 0 and lvlPrm > 0.
*     ------------------------------------------------------------------
      call s5dflt( 'Print the options', 
     $             cw, lencw, iw, leniw, rw, lenrw )

*     ------------------------------------------------------------------
*     Compute the storage requirements for SQOPT  from the following
*     variables:
*         m   , n   , ne
*         maxR, maxS, lenc (ngObj),  ncolH (nnH)
*     ------------------------------------------------------------------
*     Fetch the limits on SQOPT real and integer workspace.
*     They are used to define the limits on LU workspace. 

      maxrw = iw(  3)           ! Extent of SQOPT real    workspace
      maxiw = iw(  5)           ! Extent of SQOPT integer workspace
      maxcw = iw(  7)           ! Extent of SQOPT char*8  workspace

      mincw = iw(maxcu) + 1
      miniw = iw(maxiu) + 1
      minrw = iw(maxru) + 1

      maxR  = iw( 56)
      maxS  = iw( 57)

      call s5Mem ( ierror, iw(iPrint), iw(iSumm), 
     $             m, n, ne,
     $             lenc, ncolH,
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
*     Set default values for the unused features in sqopt.
*     ------------------------------------------------------------------
      lenb  = 0                       ! No constraint rhs vector,
      lenx0 = 0                       ! No constant shift for x.

*     ------------------------------------------------------------------
*     Solve the problem.
*     ------------------------------------------------------------------
      nka    = n + 1
      nb     = n + m
      call s5solv( sqHx, qpHx, start, lenb, lenx0, m, 
     $             n, nb, ne, nka, nName,
     $             iObj, ObjAdd, ObjQP, Objtru, 
     $             nInf, sInf,
     $             a, ha, ka, 
     $             dummyb, bl, bu, c, Names,
     $             hElast, hs, dummyx, xs, pi, rc,
     $             inform, nS, 
     $             cu, lencu, iu, leniu, ru, lenru, 
     $             cw, lencw, iw, leniw, rw, lenrw )

      Obj    = Objtru

*     Print times for all clocks (if lvlTim > 0).

      call s1time( 0, 2, iw, leniw, rw, lenrw )

  999 return

 9000 format(  ' EXIT -- SQOPT character, integer and real work arrays',
     $         ' each must have at least 500 elements')

*     end of sqopt
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine sqHx  ( qpHx, minimz, n, nnH, ne, nka, a, ha, ka,
     $                   x, Hx, nState, 
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      external           qpHx
      integer            ha(ne)
      integer            ka(nka)
      double precision   a(ne)
      double precision   Hx(nnH), x(nnH)

      character*8        cu(lencu), cw(lencw)
      integer            iu(leniu), iw(leniw)
      double precision   ru(lenru), rw(lenrw)

*     ==================================================================
*     sqHx  is the generic Hessian times vector routine for sqopt.
*     It computes the product Hx via the user-supplied routine sqHx
*     and scales it. 
*
*     09-Oct 1994: First version of sqHx
*     25-Jun 1997: nProb removed from qpHx argument list
*     25-Jun 1997: Current version of sqHx
*     ==================================================================
      logical            scaled
*     ------------------------------------------------------------------
      lvlScl    = iw( 75)
      laScal    = iw(274)
      lxScl     = iw(275)

      scaled    = lvlScl .gt. 0

      if (scaled) then
         call dcopy ( nnH, x         , 1, rw(lxScl), 1 )
         call ddscl ( nnH, rw(laScal), 1, x        , 1 )
      end if

      call qpHx  ( nnH, x, Hx, nState, cu, lencu, iu, leniu, ru, lenru )

      if (scaled) then
         call dcopy ( nnH, rw(lxScl),  1, x , 1 )
         call ddscl ( nnH, rw(laScal), 1, Hx, 1 )
      end if

*     end of sqHx
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine sqtitl( title )

      character*30       title

*     ==================================================================
*     sqtitl sets the title.
*     ==================================================================

      title  = 'S Q O P T  5.3-2    (May 1998)'
*---------------123456789|123456789|123456789|--------------------------

*     end of sqtitl
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine sqInit( iPrint, iSumm,
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     sqInit  is called by the user to do the following:
*     1. Open default files (Print, Summary).
*     2. Initialize title.
*     3. Set options to default values.
*
*     15 Nov 1991: First version.
*     14 Jul 1997: Thread-safe version.
*     21 Mar 1997: First version based on snopt routine snInit
*     14 Jul 1997: Thread-safe version.
*     02 Oct 1997: Character workspace added.
*     15 Feb 1998: Current version of sqInit.
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
      parameter         (nnH       =  27)

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
      iw(nnH  ) = 0

      call sqtitl( title )
      call s1init( title, iw, leniw, rw, lenrw )

      if (iPrint .gt. 0) then
         write (iPrint, '(  9x, a )') ' ', dashes, title, dashes
      end if
      if (iSumm .gt. 0) then
         write (iSumm , '(  1x, a )') ' ', dashes, title, dashes
      end if

*     ------------------------------------------------------------------
*     Set the options to default values.
*     sqopt  will check the options later and maybe print them.
*     ------------------------------------------------------------------
      call s3undf( cw, lencw, iw, leniw, rw, lenrw )

*     ------------------------------------------------------------------
*     Initialize some global values.
*     ------------------------------------------------------------------
      iw(lvlTim) = 3

  999 return

 9000 format(  ' EXIT from sqInit --',
     $         ' character, integer and real work arrays',
     $         ' must each have at least 500 elements')

*     end of sqInit
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine sqSpec( iSpecs, inform, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     sqSpec  is called by the user to read the Specs file.
*
*     07-Feb 1998: First version.
*     07-Feb 1998: Current version of sqSpec.
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
*     sqopt  will check the options later and maybe print them.
*     ------------------------------------------------------------------
      if (iSpecs .gt. 0) then
         call s3file( nCalls, iSpecs, s3opt,
     $                iPrint, iSumm, inform,
     $                cw, lencw, iw, leniw, rw, lenrw )
      end if

*     end of sqSpec
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine sqMem ( m, n, ne,
     $                   lenc, ncolH,
     $                   mincw, miniw, minrw,
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     sqMem   estimates the memory requirements for sqopt,
*     using the values:
*        m    , n    , ne
*        lenc , ncolH     
*
*     These values are used to compute the minimum required storage:
*     mincw, miniw, minrw.
*
*     Note: 
*     1. All default parameters must be set before calling sqMem,
*        since some values affect the amount of memory required.
*
*     2. The arrays rw and iw hold  constants and work-space addresses.
*        They must have dimension at least 500.
*
*     3. This version of sqMem does not allow user accessible
*        partitions of cw, iw and rw.
*
*     01 May 1998: First version.
*     01 May 1998: Current version of sqMem.
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

*     Error messages from s5Mem are suppressed.

      iPrinx  = 0
      iSummx  = 0

*     Assign fake values for lencw, leniw, lenrw.
*     This will force s5Mem to estimate the memory requirements.

      llenrw  = 500
      lleniw  = 500
      llencw  = 500

*     An obligatory call to sqInit has `undefined' all options.
*     Check the user-defined values and assign undefined values.
*     s5dflt needs various problem dimensions in iw.

      nnCon   = 0               ! Not used in sqopt
      nnObj   = 0               ! ditto
      nnJac   = 0               ! ditto

      iw( 15) = m
      iw( 16) = n
      iw( 17) = ne

      iw( 21) = nnCon
      iw( 22) = nnJac
      iw( 23) = nnObj
 
      iw( 26) = lenc
      iw( 27) = ncolH
 
      call s5dflt( 'Check optional parameters', 
     $             cw, llencw, iw, lleniw, rw, llenrw )

      mincw   = 501
      miniw   = 501
      minrw   = 501

      maxcw   = lencw
      maxiw   = leniw
      maxrw   = lenrw

      maxR    = iw( 56)
      maxS    = iw( 57)

      call s5Mem ( inform, iPrinx, iSummx, 
     $             m, n, ne,
     $             lenc, ncolH,
     $             maxR, maxS, 
     $             maxcw, maxiw, maxrw,
     $             llencw, lleniw, llenrw,
     $             mincw, miniw, minrw, iw )

  999 return

 9000 format(  ' EXIT from sqMem --',
     $         ' character, integer and real work arrays',
     $         ' must each have at least 500 elements')

*     end of sqMem.
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine sqset ( buffer, iPrint, iSumm, inform,
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*(*)      buffer

      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     sqset  decodes the option contained in  buffer.
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

*     end of sqset
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine sqseti( buffer, ivalue, iPrint, iSumm, inform,
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*(*)      buffer
      integer            ivalue

      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     sqseti decodes the option contained in  buffer // ivalue.
*     The parameters other than ivalue are as in sqset.
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

*     end of sqseti
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine sqsetr( buffer, rvalue, iPrint, iSumm, inform,
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*(*)      buffer
      double precision   rvalue

      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     sqsetr decodes the option contained in  buffer // rvalue.
*     The parameters other than rvalue are as in sqset.
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

*     end of sqsetr
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine sqgetc( buffer, cvalue,
     $                   inform, cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*(*)      buffer
      character*8        cvalue

      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     sqgetc gets the value of the option contained in  buffer.
*     The parameters other than cvalue are as in sqset.
*
*     17 May 1998: first version.
*     ==================================================================
      character*16       key
*     ------------------------------------------------------------------
      call s3opt ( .false., buffer, key, cvalue, ivalue, rvalue,
     $             0, 0, inform,
     $             cw, lencw, iw, leniw, rw, lenrw )

*     end of sqgetc
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine sqgeti( buffer, ivalue,
     $                   inform, cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*(*)      buffer
      integer            ivalue

      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     sqgeti gets the value of the option contained in  buffer.
*     The parameters other than ivalue are as in sqset.
*
*     17 May 1998: first version.
*     ==================================================================
      character*16       key
      character*8        cvalue
*     ------------------------------------------------------------------
      call s3opt ( .false., buffer, key, cvalue, ivalue, rvalue,
     $             0, 0, inform,
     $             cw, lencw, iw, leniw, rw, lenrw )

*     end of sqgeti
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine sqgetr( buffer, rvalue,
     $                   inform, cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*(*)      buffer
      double precision   rvalue

      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     sqgetr gets the value of the option contained in  buffer.
*     The parameters other than rvalue are as in sqset.
*
*     17 May 1998: first version.
*     ==================================================================
      character*16       key
      character*8        cvalue
*     ------------------------------------------------------------------
      call s3opt ( .false., buffer, key, cvalue, ivalue, rvalue,
     $             0, 0, inform,
     $             cw, lencw, iw, leniw, rw, lenrw )

*     end of sqsetr
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine nullHx( nnH, x, Hx, nState, 
     $                   cu, lencu, iu, leniu, ru, lenru )

      implicit           double precision (a-h,o-z)
      double precision   x(nnH), Hx(nnH)

      character*8        cu(lencu)
      integer            iu(leniu)
      double precision   ru(lenru)

*     ==================================================================
*     This is the dummy (empty) version of the routine qpHx.
*     It should never be called by SQOPT.
*     
*     Warn the user (on the standard output) that it has been called.
*     ==================================================================
      nOut = 6
      if (nState .eq. 1) then
         if (nOut .gt. 0) write(nOut, 1000)
      end if

      return

 1000 format(//
     $     ' XXX  The default (dummy) version of subroutine Hx',
     $     '     has been called. '
     $    /' XXX  A user-defined version is required when solving a QP')

*     end of nullHx
      end

