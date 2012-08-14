************************************************************************
*
*     File  LU1.for  Fortran
*
*     lu1fac   lu1fad   lu1gau   lu1mar   lu1pen
*     lu1max   lu1or1   lu1or2   lu1or3   lu1or4
*     lu1pq1   lu1pq2   lu1pq3   lu1rec
*     lu1ful   lu1den
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lu1fac( m    , n    , nelem, lena , luparm, parmlu,
     $                   a    , indc , indr , ip   , iq    ,
     $                   lenc , lenr , locc , locr ,
     $                   iploc, iqloc, ipinv, iqinv, w     , inform )

      implicit           double precision (a-h,o-z)
      integer            luparm(30)
      double precision   parmlu(30), a(lena)   , w(n)
      integer*4          indc(lena), indr(lena), ip(m)   , iq(n),
     $                   lenc(n)   , lenr(m)   ,
     $                   iploc(n)  , iqloc(m)  , ipinv(m), iqinv(n)
      integer            locc(n)   , locr(m)

*     ------------------------------------------------------------------
*     lu1fac computes a factorization A = L*U, where A is a sparse
*     matrix with m rows and n columns, P*L*P' is lower triangular
*     and P*U*Q is upper triangular for certain permutations P, Q
*     (which are returned in the arrays ip, iq).
*     Stability is ensured by limiting the size of the elements of L.
*
*     The nonzeros of A are input via the parallel arrays a, indc, indr,
*     which should contain nelem entries of the form    aij,    i,    j
*     in any order.  There should be no duplicate pairs         i,    j.
*
*     ******************************************************************
*     *        Beware !!!   The row indices i must be in indc,         *
*     *              and the column indices j must be in indr.         *
*     *              (Not the other way round!)                        *
*     ******************************************************************
*
*     It does not matter if some of the entries in a(*) are zero.
*     Entries satisfying  abs( a(i) ) .le. parmlu(3)  are ignored.
*     Other parameters in luparm and parmlu are described below.
*
*     The matrix A may be singular.  On exit, nsing = luparm(11) gives
*     the number of apparent singularities.  This is the number of
*     "small" diagonals of the permuted factor U, as judged by
*     the input tolerances utol1 = parmlu(4) and  utol2 = parmlu(5).
*     The diagonal element diagj associated with column j of A is
*     "small" if
*                 abs( diagj ) .le. utol1
*     or
*                 abs( diagj ) .le. utol2 * max( uj ),
*
*     where max( uj ) is the maximum element in the j-th column of U.
*     The position of such elements is returned in w(*).  In general,
*     w(j) = + max( uj ),  but if column j is a singularity,
*     w(j) = - max( uj ).  Thus, w(j) .le. 0 if column j appears to be
*     dependent on the other columns of A.
*     ==================================================================
*
*
*     Notes on the array names
*     ------------------------
*
*     During the LU factorization, the sparsity pattern of the matrix
*     being factored is stored twice: in a column list and a row list.
*
*     The column list is ( a, indc, locc, lenc )
*     where
*           a(*)    holds the nonzeros,
*           indc(*) holds the indices for the column list,
*           locc(j) points to the start of column j in a(*) and indc(*),
*           lenc(j) is the number of nonzeros in column j.
*
*     The row list is    (    indr, locr, lenr )
*     where
*           indr(*) holds the indices for the row list,
*           locr(i) points to the start of row i in indr(*),
*           lenr(i) is the number of nonzeros in row i.
*
*
*     At all stages of the LU factorization, ip contains a complete
*     row permutation.  At the start of stage k,  ip(1), ..., ip(k-1)
*     are the first k-1 rows of the final row permutation P.
*     The remaining rows are stored in an ordered list
*                          ( ip, iploc, ipinv )
*     where
*           iploc(nz) points to the start in ip(*) of the set of rows
*                     that currently contain nz nonzeros,
*           ipinv(i)  points to the position of row i in ip(*).
*
*     For example,
*           iploc(1) = k   (and this is where rows of length 1 begin),
*           iploc(2) = k+p  if there are p rows of length 1
*                          (and this is where rows of length 2 begin).
*
*     Similarly for iq, iqloc, iqinv.
*     ==================================================================
*
*
*     00 Jun 1983  Original version.
*     00 Jul 1987  nrank  saved in luparm(16).
*     12 Apr 1989  ipinv, iqinv added as workspace.
*     26 Apr 1989  maxtie replaced by maxcol in Markowitz search.
*     16 Mar 1992  jumin  saved in luparm(19).
*     10 Jun 1992  lu1fad has to move empty rows and cols to the bottom
*                  (via lu1pq3) before doing the dense LU.
*     12 Jun 1992  Deleted dense LU (lu1ful, lu1vlu).
*     25 Oct 1993  keepLU implemented.
*     07 Feb 1994  Added new dense LU (lu1ful, lu1den).
*     21 Dec 1994  Bugs fixed in lu1fad (nrank) and lu1ful (ipvt).
*     08 Aug 1995  Use ip instead of w as parameter to lu1or3 (for F90).
*
*     Systems Optimization Laboratory, Stanford University.
*  ---------------------------------------------------------------------
*                        
*
*  INPUT PARAMETERS
*
*  m      (not altered) is the number of rows in A.
*  n      (not altered) is the number of columns in A.
*  nelem  (not altered) is the number of matrix entries given in
*         the arrays a, indc, indr.
*  lena   (not altered) is the dimension of  a, indc, indr.
*         This should be significantly larger than nelem.
*         Typically one should have
*            lena > max( 2*nelem, 10*m, 10*n, 10000 )
*         but some applications may need more.
*         On machines with virtual memory it is safe to have
*         lena "far bigger than necessary", since not all of the
*         arrays will be used.
*  a      (overwritten) contains entries   Aij  in   a(1:nelem).
*  indc   (overwritten) contains the indices i in indc(1:nelem).
*  indr   (overwritten) contains the indices j in indr(1:nelem).
*
*  luparm input parameters:                                Typical value
*
*  luparm( 1) = nout     File number for printed messages.         6
*  luparm( 2) = lprint   Print level.                              0
*                    < 0 suppresses output.
*                    = 0 gives error messages.
*                    = 1 gives debug output from some of the
*                        other routines in LUSOL.
*                   >= 2 gives the pivot row and column and the
*                        no. of rows and columns involved at
*                        each elimination step in lu1fac.
*  luparm( 3) = maxcol   lu1fac: maximum number of columns         5
*                        searched allowed in a Markowitz-type
*                        search for the next pivot element.
*                        For some of the factorization, the
*                        number of rows searched is
*                        maxrow = maxcol - 1.
*  luparm( 8) = keepLU   lu1fac: keepLU = 1 means the numerical    1
*                        factors will be computed if possible.
*                        keepLU = 0 means L and U will be discarded
*                        but other information such as the row and
*                        column permutations will be returned.
*                        The latter option requires less storage.
*
*  parmlu input parameters:                                Typical value
*
*  parmlu( 1) = elmax1   Max multiplier allowed in  L      10.0 or 100.0
*                        during factor.
*  parmlu( 2) = elmax2   Max multiplier allowed in  L           10.0
*                        during updates.
*  parmlu( 3) = small    Absolute tolerance for       eps**0.8 = 3.0d-13
*                        treating reals as zero.
*  parmlu( 4) = utol1    Absolute tol for flagging    eps**0.67= 3.7d-11
*                        small diagonals of U.
*  parmlu( 5) = utol2    Relative tol for flagging    eps**0.67= 3.7d-11
*                        small diagonals of U.      
*                        (eps=machine precision)
*  parmlu( 6) = uspace   Factor limiting waste space in  U.      3.0
*                        In lu1fac, the row or column lists
*                        are compressed if their length
*                        exceeds uspace times the length of
*                        either file after the last compression.
*  parmlu( 7) = dens1    The density at which the Markowitz      0.3
*                        pivot strategy should search maxcol
*                        columns and no rows.
*                        (Use 0.3 unless you are experimenting
*                        with the pivot strategy.)
*  parmlu( 8) = dens2    the density at which the Markowitz      0.5
*                        strategy should search only 1 column,
*                        or (if storage is available)
*                        the density at which all remaining
*                        rows and columns will be processed
*                        by a dense LU code.
*                        For example, if dens2 = 0.1 and lena is
*                        large enough, a dense LU will be used
*                        once more than 10 per cent of the
*                        remaining matrix is nonzero.
*                         
*
*  OUTPUT PARAMETERS
*
*  a, indc, indr     contain the nonzero entries in theLU factors of A.
*         If keepLU = 1, they are in a form suitable for use
*         by other parts of the LUSOL package, such as lu6sol.
*         U is stored by rows at the start of a, indr.
*         L is stored by cols at the end   of a, indc.
*         If keepLU = 0, only the diagonals of U are stored, at the
*         end of a.
*  ip, iq    are the row and column permutations defining the
*         pivot order.  For example, row ip(1) and column iq(1)
*         defines the first diagonal of U.
*  lenc(1:numl0) contains the number of entries in nontrivial
*         columns of L (in pivot order).
*  lenr(1:m) contains the number of entries in each row of U
*         (in original order).
*  locc(1:n) = 0 (ready for the LU update routines).
*  locr(1:m) points to the beginning of the rows of U in a, indr.
*  iploc, iqloc, ipinv, iqinv  are undefined.
*  w      indicates singularity as described above.
*  inform = 0 if the LU factors were obtained successfully.
*         = 3 if some index pair indc(l), indr(l) lies outside
*             the matrix dimensions 1:m , 1:n.
*         = 4 if some index pair indc(l), indr(l) duplicates
*             another such pair.
*         = 7 if the arrays a, indc, indr were not large enough.
*             Their length "lena" should be increase to at least
*             the value "minlen" given in luparm(13).
*         = 8 if there was some other fatal error.  (Shouldn't happen!)
*
*  luparm output parameters:
*
*  luparm(10) = inform   Return code from last call to any LU routine.
*  luparm(11) = nsing    No. of singularities marked in the
*                        output array w(*).
*  luparm(12) = jsing    Column index of last singularity.
*  luparm(13) = minlen   Minimum recommended value for  lena.
*  luparm(14) = maxlen   ?
*  luparm(15) = nupdat   No. of updates performed by the lu8 routines.
*  luparm(16) = nrank    No. of nonempty rows of U.
*  luparm(17) = ndens1   No. of columns remaining when the density of
*                        the matrix being factorized reached dens1.
*  luparm(18) = ndens2   No. of columns remaining when the density of
*                        the matrix being factorized reached dens2.
*  luparm(19) = jumin    The column index associated with dumin.
*  luparm(20) = numl0    No. of columns in initial  L.
*  luparm(21) = lenl0    Size of initial  L  (no. of nonzeros).
*  luparm(22) = lenu0    Size of initial  U.
*  luparm(23) = lenl     Size of current  L.
*  luparm(24) = lenu     Size of current  U.
*  luparm(25) = lrow     Length of row file.
*  luparm(26) = ncp      No. of compressions of LU data structures.
*  luparm(27) = mersum   lu1fac: sum of Markowitz merit counts.
*  luparm(28) = nutri    lu1fac: triangular rows in U.
*  luparm(29) = nltri    lu1fac: triangular rows in L.
*  luparm(30) =
*
*
*
*  parmlu output parameters:
*
*  parmlu(10) = amax     Maximum element in  A.
*  parmlu(11) = elmax    Maximum multiplier in current  L.
*  parmlu(12) = umax     Maximum element in current  U.
*  parmlu(13) = dumax    Maximum diagonal in  U.
*  parmlu(14) = dumin    Minimum diagonal in  U.
*  parmlu(15) =
*  parmlu(16) =
*  parmlu(17) =
*  parmlu(18) =
*  parmlu(19) =
*  parmlu(20) = resid    lu6sol: residual after solve with U or U'.
*  ...
*  parmlu(30) =
*  ---------------------------------------------------------------------

      logical            keepLU

*     Grab relevant input parameters.

      nelem0 = nelem
      nout   = luparm(1)
      lprint = luparm(2)
      keepLU = luparm(8) .ne. 0
      small  = parmlu(3)

*     Initialize output parameters.

      inform = 0
      minlen = nelem + 2*(m + n)
      nrank  = 0
      numl0  = 0
      lenl   = 0
      lenu   = 0
      lrow   = 0
      mersum = 0
      nutri  = m
      nltri  = 0
      amax   = 1.0d+0

*     Initialize workspace parameters.

      luparm(26) = 0
      if (lena .lt. minlen) go to 970


*     ------------------------------------------------------------------
*     Organize the  aij's  in  a, indc, indr.
*     lu1or1  deletes small entries, tests for illegal  i,j's,
*             and counts the nonzeros in each row and column.
*     lu1or2  reorders the elements of  A  by columns.
*     lu1or3  uses the column list to test for duplicate entries
*             (same indices  i,j).
*     lu1or4  constructs a row list from the column list.
*     ------------------------------------------------------------------
      call lu1or1( m   , n    , nelem, small,
     $             a   , indc , indr , lenc , lenr,
     $             amax, numnz, lerr , inform )
      if (inform .ne. 0) go to 930

      nelem  = numnz
      if (nelem .gt. 0) then
         call lu1or2( n, nelem, a, indc, indr, lenc, locc )
         call lu1or3( m, n, nelem, indc, lenc, locc, ip,
     $                lerr, inform )
         if (inform .ne. 0) go to 940

         call lu1or4( m, n, nelem,
     $                indc, indr, lenc, lenr, locc, locr )
      end if

*     ------------------------------------------------------------------
*     Set up lists of rows and columns with equal numbers of nonzeros,
*     using  indc(*)  as workspace.
*     Then compute the factorization  A = L*U.
*     ------------------------------------------------------------------
      call lu1pq1( m, n, lenr, ip, iploc, ipinv, indc(nelem + 1) )
      call lu1pq1( n, m, lenc, iq, iqloc, iqinv, indc(nelem + 1) )

      call lu1fad( m     , n    , nelem, lena  , luparm, parmlu,
     $             a     , indc , indr , ip    , iq    ,
     $             lenc  , lenr , locc , locr  ,
     $             iploc , iqloc, ipinv, iqinv ,
     $             inform, lenl , lenu , minlen, mersum,
     $             nutri , nltri, nrank )

      if (inform .eq. 7) go to 970
      if (inform .gt. 0) go to 980

      luparm(16) = nrank
      luparm(23) = lenl

      if ( keepLU ) then
*        ---------------------------------------------------------------
*        The LU factors are at the top of  a, indc, indr,
*        with the columns of  L  and the rows of  U  in the order
*
*        ( free )   ... ( u3 ) ( l3 ) ( u2 ) ( l2 ) ( u1 ) ( l1 ).
*
*        Starting with ( l1 ) and ( u1 ), move the rows of  U  to the
*        left and the columns of  L  to the right, giving
*
*        ( u1 ) ( u2 ) ( u3 ) ...   ( free )   ... ( l3 ) ( l2 ) ( l1 ).
*
*        Also, set  numl0 = the number of nonempty columns of  U.
*        ---------------------------------------------------------------
         lu     = 0
         ll     = lena + 1
         lm     = ll
         ltopl  = ll - lenl - lenu
         lrow   = lenu

         do 360  k  = 1, nrank
            i       =   ip(k)
            lenuk   = - lenr(i)
            lenr(i) =   lenuk
            j       =   iq(k)
            lenlk   = - lenc(j) - 1
            if (lenlk .gt. 0) then
                numl0        = numl0 + 1
                iqloc(numl0) = lenlk
            end if

            if (lu + lenuk .lt. ltopl) then
*              =========================================================
*              There is room to move ( uk ).  Just right-shift ( lk ).
*              =========================================================
               do 310 idummy = 1, lenlk
                  ll       = ll - 1
                  lm       = lm - 1
                  a(ll)    = a(lm)
                  indc(ll) = indc(lm)
                  indr(ll) = indr(lm)
  310          continue
            else
*              =========================================================
*              There is no room for ( uk ) yet.  We have to
*              right-shift the whole of the remaining LU file.
*              Note that ( lk ) ends up in the correct place.
*              =========================================================
               llsave = ll - lenlk
               nmove  = lm - ltopl

               do 330 idummy = 1, nmove
                  ll       = ll - 1
                  lm       = lm - 1
                  a(ll)    = a(lm)
                  indc(ll) = indc(lm)
                  indr(ll) = indr(lm)
  330          continue

               ltopl  = ll
               ll     = llsave
               lm     = ll
            end if

*           ======================================================
*           Left-shift ( uk ).
*           ======================================================
            locr(i) = lu + 1
            l2      = lm - 1
            lm      = lm - lenuk

            do 350 l = lm, l2
               lu       = lu + 1
               a(lu)    = a(l)
               indr(lu) = indr(l)
  350       continue
  360    continue

*        ---------------------------------------------------------------
*        Save the lengths of the nonempty columns of  L,
*        and initialize  locc(j)  for the LU update routines.
*        ---------------------------------------------------------------
         do 370  k  = 1, numl0
            lenc(k) = iqloc(k)
  370    continue

         do 390  j  = 1, n
            locc(j) = 0
  390    continue

*        ---------------------------------------------------------------
*        Test for singularity.
*        lu6chk  sets  nsing, jsing, jumin, elmax, umax, dumax, dumin.
*        inform = 1  if there are singularities (nsing gt 0).
*        ---------------------------------------------------------------
         call lu6chk( 1, m, n, w, lena, luparm, parmlu,
     $                a, indc, indr, ip, iq,
     $                lenc, lenr, locc, locr, inform )

      else
*        ---------------------------------------------------------------
*        L and U were not kept, just the diagonals of U.
*        At present, we don't do anything.  lu1fac will probably be
*        called again soon with keepLU = .true.
*        ---------------------------------------------------------------
      end if

      go to 990

*     ------------
*     Error exits.
*     ------------
  930 inform = 3
      if (lprint .ge. 0) write(nout, 1300) lerr, indc(lerr), indr(lerr)
      go to 990

  940 inform = 4
      if (lprint .ge. 0) write(nout, 1400) lerr, indc(lerr), indr(lerr)
      go to 990

  970 inform = 7
      if (lprint .ge. 0) write(nout, 1700) lena, minlen
      go to 990

  980 inform = 8
      if (lprint .ge. 0) write(nout, 1800)

*     Store output parameters.

  990 nelem      = nelem0
      luparm(10) = inform
      luparm(13) = minlen
      luparm(15) = 0
      luparm(16) = nrank
      luparm(20) = numl0
      luparm(21) = lenl
      luparm(22) = lenu
      luparm(23) = lenl
      luparm(24) = lenu
      luparm(25) = lrow
      luparm(27) = mersum
      luparm(28) = nutri
      luparm(29) = nltri
      parmlu(10) = amax
      return

 1300 format(/ ' lu1fac  error...  entry  a(', i8, ')  has an illegal',
     $         ' row or column index'
     $       //' indc, indr =', 2i8)
 1400 format(/ ' lu1fac  error...  entry  a(', i8, ')  has the same',
     $         ' indices as an earlier entry'
     $       //' indc, indr =', 2i8)
 1700 format(/ ' lu1fac  error...  insufficient storage'
     $       //' Increase  lena  from', i8, '  to at least', i8)
 1800 format(/ ' lu1fac  error...  fatal bug',
     $         '   (sorry --- this should never happen)')
*     end of lu1fac
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lu1fad( m     , n    , nelem, lena  , luparm, parmlu,
     $                   a     , indc , indr , ip    , iq    ,
     $                   lenc  , lenr , locc , locr  ,
     $                   iploc , iqloc, ipinv, iqinv ,
     $                   inform, lenl , lenu , minlen, mersum,
     $                   nutri , nltri, nrank )

      implicit           double precision (a-h,o-z)
      integer            luparm(30)
      double precision   parmlu(30), a(lena)
      integer*4          indc(lena), indr(lena), ip(m), iq(n)
      integer*4          lenc(n)   , lenr(m)
      integer            locc(n)   , locr(m)
      integer*4          iploc(n)  , iqloc(m), ipinv(m), iqinv(n)

*     ------------------------------------------------------------------
*     lu1fad  is a driver for the numerical phase of lu1fac.
*     At each stage it computes a column of  L  and a row of  U,
*     using a Markowitz criterion to select the pivot element,
*     subject to a stability criterion that bounds the elements of  L.
*
*     00 Jan 1986  Version documented in LUSOL paper:
*                  Gill, Murray, Saunders and Wright (1987),
*                  Maintaining LU factors of a general sparse matrix,
*                  Linear algebra and its applications 88/89, 239-270.
*
*     02 Feb 1989  Following Suhl and Aittoniemi (1987), the largest
*                  element in each column is now kept at the start of
*                  the column, i.e. in position locc(j) of a and indc.
*                  This should speed up the Markowitz searches.
*                  To save time on highly triangular matrices, we wait
*                  until there are no further columns of length 1
*                  before setting and maintaining that property.
*
*     12 Apr 1989  ipinv and iqinv added (inverses of ip and iq)
*                  to save searching ip and iq for rows and columns
*                  altered in each elimination step.  (Used in lu1pq2)
*
*     19 Apr 1989  Code segmented to reduce its size.
*                  lu1gau does most of the Gaussian elimination work.
*                  lu1mar does just the Markowitz search.
*                  lu1max moves biggest elements to top of columns.
*                  lu1pen deals with pending fill-in in the row list.
*                  lu1pq2 updates the row and column permutations.
*
*     26 Apr 1989  maxtie replaced by maxcol, maxrow in the Markowitz
*                  search.  maxcol, maxrow change as density increases.
*
*     25 Oct 1993  keepLU implemented.
*
*     07 Feb 1994  Exit main loop early to finish off with a dense LU.
*                  densLU tells lu1fad whether to do it.
*     21 Dec 1994  Bug fixed.  nrank was wrong after the call to lu1ful.
*
*     Systems Optimization Laboratory, Stanford University.
*     ------------------------------------------------------------------

      logical            utri, ltri, spars1, spars2, dense
      logical            densLU, keepLU
      double precision   lmax, small

      double precision   one,            lmin
      parameter        ( one  = 1.0d+0,  lmin = 1.0001d+0 )

*     ------------------------------------------------------------------
*     Local variables
*     ---------------
*
*     lcol   is the length of the column file.  It points to the last
*            nonzero in the column list.
*     lrow   is the analogous quantity for the row file.
*     lfile  is the file length (lcol or lrow) after the most recent
*            compression of the column list or row list.
*     nrowd  and  ncold  are the number of rows and columns in the
*            matrix defined by the pivot column and row.  They are the
*            dimensions of the submatrix D being altered at this stage.
*     melim  and  nelim  are the number of rows and columns in the
*            same matrix D, excluding the pivot column and row.
*     mleft  and  nleft  are the number of rows and columns
*            still left to be factored.
*     nzchng is the increase in nonzeros in the matrix that remains
*            to be factored after the current elimination
*            (usually negative).
*     nzleft is the number of nonzeros still left to be factored.
*     nspare is the space we leave at the end of the last row or
*            column whenever a row or column is being moved to the end
*            of its file.  nspare = 1 or 2 might help reduce the
*            number of file compressions when storage is tight.
*
*     The row and column ordering permutes A into the form
*
*                        ------------------------
*                         \                     |
*                          \         U1         |
*                           \                   |
*                            --------------------
*                            |\
*                            | \
*                            |  \
*            P A Q   =       |   \
*                            |    \
*                            |     --------------
*                            |     |            |
*                            |     |            |
*                            | L1  |     A2     |
*                            |     |            |
*                            |     |            |
*                            --------------------
*
*     where the block A2 is factored as  A2 = L2 U2.
*     The phases of the factorization are as follows.
*
*     utri   is true when U1 is being determined.
*            Any column of length 1 is accepted immediately.
*
*     ltri   is true when L1 is being determined.
*            lu1mar exits as soon as an acceptable pivot is found
*            in a row of length 1.
*
*     spars1 is true while the density of the (modified) A2 is less
*            than the parameter dens1 = parmlu(7) = 0.3 say.
*            lu1mar searches maxcol columns and maxrow rows,
*            where  maxcol = luparm(3),  maxrow = maxcol - 1.
*            lu1max is used to keep the biggest element at the top
*            of all remaining columns.
*
*     spars2 is true while the density of the modified A2 is less
*            than the parameter dens2 = parmlu(8) = 0.6 say.
*            lu1mar searches maxcol columns and no rows.
*            lu1max fixes up only the first maxcol columns.
*
*     dense  is true once the density of A2 reaches dens2.
*            lu1mar searches only 1 column (the shortest).
*            lu1max fixes up only the first column.
*
*     ------------------------------------------------------------------
*     To eliminate timings, comment out all lines containing "time".
*     ------------------------------------------------------------------

*     integer            eltime, mktime

*     call timer ( 'start ', 3 )
*     ntime  = (n / 4.0) + lmin

      nout   = luparm(1)
      lprint = luparm(2)
      maxcol = luparm(3)
      keepLU = luparm(8) .ne. 0
      densLU = .false.

      maxrow = maxcol - 1
      lenl   = 0
      lenu   = 0
      lfile  = nelem
      lrow   = nelem
      lcol   = nelem
      minmn  = min( m, n )
      maxmn  = max( m, n )
      nzleft = nelem
      nspare = 1

      if ( keepLU ) then
         lu1    = lena   + 1
      else
*        Store only the diagonals of U in the top of memory.
         ldiagU = lena   - n
         lu1    = ldiagU + 1
      end if

      lmax   = parmlu(1)
      small  = parmlu(3)
      uspace = parmlu(6)
      dens1  = parmlu(7)
      dens2  = parmlu(8)
      utri   = .true.
      ltri   = .false.
      spars1 = .false.
      spars2 = .false.
      dense  = .false.

*     Check parameters.

      lmax   = max( lmax, lmin )
      dens1  = min( dens1, dens2 )

*     Initialize output parameters.

      ndens1 = 0
      ndens2 = 0

*     ------------------------------------------------------------------
*     Start of main loop.
*     ------------------------------------------------------------------
      mleft  = m + 1
      nleft  = n + 1

      do 800 nrowu = 1, minmn

*        mktime = (nrowu / ntime) + 4
*        eltime = (nrowu / ntime) + 9
         mleft  = mleft - 1
         nleft  = nleft - 1

*        Bail out if there are no nonzero rows left.

         if (iploc(1) .gt. m) go to 900

*        ===============================================================
*        Find a suitable pivot element.
*        ===============================================================

         if ( utri ) then
*           ------------------------------------------------------------
*           So far all columns have had length 1.
*           We are still looking for the (backward) triangular part of A
*           that forms the first rows and columns of U.
*           ------------------------------------------------------------
            lq1    = iqloc(1)
            lq2    = n
            if (m   .gt.   1) lq2 = iqloc(2) - 1

            if (lq1 .le. lq2) then

*              We still have a column of length 1.  Grab it!

               jbest  = iq(lq1)
               lc     = locc(jbest)
               ibest  = indc(lc)
               mbest  = 0
            else

*              This is the end of the U triangle.
*              Move the largest elements to the top of each column
*              to make the remaining Markowitz searches more efficient.
*              We will not return to this part of the code.

               utri   = .false.
               ltri   = .true.
               spars1 = .true.
               nutri  =  nrowu - 1
               call lu1max( lq1, n, iq, a, indc, lenc, locc )
            end if
         end if

         if ( spars1 ) then
*           ------------------------------------------------------------
*           Perform a normal Markowitz search.
*           Search cols of length 1, then rows of length 1,
*           then   cols of length 2, then rows of length 2, etc.
*           ------------------------------------------------------------
*           call timer ( 'start ', mktime )
            call lu1mar( m    , n     , lena  , maxmn,
     $                   lmax , maxcol, maxrow,
     $                   ibest, jbest , mbest ,
     $                   a    , indc  , indr  , ip   , iq,
     $                   lenc , lenr  , locc  , locr ,
     $                   iploc, iqloc )
*           call timer ( 'finish', mktime )

            if ( ltri ) then

*              So far all rows have had length 1.
*              We are still looking for the (forward) triangle of A
*              that forms the first rows and columns of L.

               if (mbest .gt. 0) then
                   ltri   = .false.
                   nltri  =  nrowu - 1 - nutri
               end if
            else

*              See if what's left is as dense as dens1.

               if (nzleft  .ge.  (dens1 * mleft) * nleft) then
                   spars1 = .false.
                   spars2 = .true.
                   ndens1 =  nleft
                   maxrow =  0
               end if
            end if

         else if ( spars2 .or. dense ) then
*           ------------------------------------------------------------
*           Perform a restricted Markowitz search,
*           looking at only the first maxcol columns.  (maxrow = 0.)
*           ------------------------------------------------------------
*           call timer ( 'start ', mktime )
            call lu1mar( m    , n     , lena  , maxmn,
     $                   lmax , maxcol, maxrow,
     $                   ibest, jbest , mbest ,
     $                   a    , indc  , indr  , ip   , iq,
     $                   lenc , lenr  , locc  , locr ,
     $                   iploc, iqloc )
*           call timer ( 'finish', mktime )

*           See if what's left is as dense as dens2.

            if ( spars2 ) then
               if (nzleft  .ge.  (dens2 * mleft) * nleft) then
                   spars2 = .false.
                   dense  = .true.
                   ndens2 =  nleft
                   maxcol =  1
               end if
            end if
         end if

*        ---------------------------------------------------------------
*        See if we can knock this guy off quickly.
*        ---------------------------------------------------------------
         if ( dense  ) then
            lend   = mleft * nleft
            nfree  = lu1 - 1

            if (nfree .ge. 2 * lend) then

*              There is room to treat the remaining matrix as dense.
*              We may have to compress the column file first.

               densLU = .true.
               ndens2 = nleft
               ld     = lcol + 1
               nfree  = lu1  - ld
               if (nfree .lt. lend) then
                  call lu1rec( n, .true., luparm,
     $                         lcol, lena, a, indc, lenc, locc )
                  lfile  = lcol
                  ld     = lcol + 1
               end if

               go to 900
            end if
         end if

*        ===============================================================
*        The best  aij  has been found.
*        The pivot row  ibest  and the pivot column  jbest
*        Define a dense matrix  D  of size  nrowd  by  ncold.
*        ===============================================================
         ncold  = lenr(ibest)
         nrowd  = lenc(jbest)
         melim  = nrowd  - 1
         nelim  = ncold  - 1
         mersum = mersum + mbest
         lenl   = lenl   + melim
         lenu   = lenu   + ncold
         if (lprint .ge. 2)
     $   write(nout, 1100) nrowu, ibest, jbest, nrowd, ncold

*        ===============================================================
*        Allocate storage for the next column of  L  and next row of  U.
*        Initially the top of a, indc, indr are used as follows:
*
*                   ncold       melim       ncold        melim
*
*        a      |...........|...........|ujbest..ujn|li1......lim|
*
*        indc   |...........|  lenr(i)  |  lenc(j)  |  markl(i)  |
*
*        indr   |...........| iqloc(i)  |  jfill(j) |  ifill(i)  |
*
*              ^           ^             ^           ^            ^
*              lfree   lsave             lu1         ll1          oldlu1
*
*        Later the correct indices are inserted:
*
*        indc   |           |           |           |i1........im|
*
*        indr   |           |           |jbest....jn|ibest..ibest|
*
*        ===============================================================
         if ( keepLU ) then
*           relax
         else
*           Always point to the top spot.
*           Only the current column of L and row of U will
*           take up space, overwriting the previous ones.
            lu1    = ldiagU + 1
         end if
         ll1    = lu1   - melim
         lu1    = ll1   - ncold
         lsave  = lu1   - nrowd
         lfree  = lsave - ncold

*        Make sure the column file has room.
*        Also force a compression if its length exceeds a certain limit.

         limit  = uspace * lfile  +  m  +  n  +  1000
         minfre = ncold  + melim
         nfree  = lfree  - lcol
         if (nfree .lt. minfre  .or.  lcol .gt. limit) then
            call lu1rec( n, .true., luparm,
     $                   lcol, lena, a, indc, lenc, locc )
            lfile  = lcol
            nfree  = lfree - lcol
            if (nfree .lt. minfre) go to 970
         end if

*        Make sure the row file has room.

         minfre = melim + ncold
         nfree  = lfree - lrow
         if (nfree .lt. minfre  .or.  lrow .gt. limit) then
            call lu1rec( m, .false., luparm,
     $                   lrow, lena, a, indr, lenr, locr )
            lfile  = lrow
            nfree  = lfree - lrow
            if (nfree .lt. minfre) go to 970
         end if

*        ===============================================================
*        Move the pivot element to the front of its row
*        and to the top of its column.
*        ===============================================================
         lpivr  = locr(ibest)
         lpivr1 = lpivr + 1
         lpivr2 = lpivr + nelim

         do 330 l = lpivr, lpivr2
            if (indr(l) .eq. jbest) go to 335
  330    continue

  335    indr(l)     = indr(lpivr)
         indr(lpivr) = jbest

         lpivc  = locc(jbest)
         lpivc1 = lpivc + 1
         lpivc2 = lpivc + melim

         do 340 l = lpivc, lpivc2
            if (indc(l) .eq. ibest) go to 345
  340    continue

  345    indc(l)     = indc(lpivc)
         indc(lpivc) = ibest
         abest       = a(l)
         a(l)        = a(lpivc)
         a(lpivc)    = abest

         if ( keepLU ) then
*           relax
         else
*           Store just the diagonal of U, in natural order.
            a(ldiagU + jbest) = abest
         end if

*        ===============================================================
*        Delete the pivot row from the column file
*        and store it as the next row of  U.
*        set  indr(lu) = 0     to initialize jfill ptrs on columns of D,
*             indc(lu) = lenj  to save the original column lengths.
*        ===============================================================
         a(lu1)    = abest
         indr(lu1) = jbest
         indc(lu1) = nrowd
         lu        = lu1

         do 360 lr   = lpivr1, lpivr2
            lu       = lu + 1
            j        = indr(lr)
            lenj     = lenc(j)
            lenc(j)  = lenj - 1
            lc1      = locc(j)
            last     = lc1 + lenc(j)

            do 350 l = lc1, last
               if (indc(l) .eq. ibest) go to 355
  350       continue

  355       a(lu)      = a(l)
            indr(lu)   = 0
            indc(lu)   = lenj
            a(l)       = a(last)
            indc(l)    = indc(last)
            indc(last) = 0
  360    continue

         if (indc(lcol) .eq. 0) lcol = lcol - 1

*        ===============================================================
*        Delete the pivot column from the row file
*        and store the nonzeros of the next column of  L.
*        Set  indc(ll) = 0     to initialize markl(*) markers,
*             indr(ll) = 0     to initialize ifill(*) row fill-in cntrs,
*             indc(ls) = leni  to save the original row lengths,
*             indr(ls) = iqloc(i)    to save parts of  iqloc(*),
*             iqloc(i) = lsave - ls  to point to the nonzeros of  L
*                      = -1, -2, -3, ... in mark(*).
*        ===============================================================
         indc(lsave) = ncold
         if (melim .eq. 0) go to 700

         ll     = ll1 - 1
         ls     = lsave
         abest  = one / abest

         do 390 lc   = lpivc1, lpivc2
            ll       = ll + 1
            ls       = ls + 1
            i        = indc(lc)
            leni     = lenr(i)
            lenr(i)  = leni - 1
            lr1      = locr(i)
            last     = lr1 + lenr(i)

            do 380 l = lr1, last
               if (indr(l) .eq. jbest) go to 385
  380       continue

  385       indr(l)    = indr(last)
            indr(last) = 0

            a(ll)      = - a(lc) * abest
            indc(ll)   = 0
            indr(ll)   = 0
            indc(ls)   = leni
            indr(ls)   = iqloc(i)
            iqloc(i)   = lsave - ls
  390    continue

         if (indr(lrow) .eq. 0) lrow = lrow - 1

*        ===============================================================
*        Do the Gaussian elimination.
*        This involves adding a multiple of the pivot column
*        to all other columns in the pivot row.
*
*        Sometimes more than one call to lu1gau is needed to allow
*        compression of the column file.
*        lfirst  says which column the elimination should start with.
*        minfre  is a bound on the storage needed for any one column.
*        lu      points to off-diagonals of u.
*        nfill   keeps track of pending fill-in in the row file.
*        ===============================================================
         if (nelim .eq. 0) go to 700
         lfirst = lpivr1
         minfre = mleft + nspare
         lu     = 1
         nfill  = 0

* 400    call timer ( 'start ', eltime )
  400    call lu1gau( m     , melim , ncold , nspare, small,
     $                lpivc1, lpivc2, lfirst, lpivr2, lfree, minfre,
     $                lrow  , lcol  , lu    , nfill ,
     $                a     , indc  , indr  ,
     $                lenc  , lenr  , locc  , locr  ,
     $                iqloc , a(ll1), indc(ll1),
     $                        a(lu1), indr(ll1), indr(lu1) )
*        call timer ( 'finish', eltime )

         if (lfirst .gt. 0) then

*           The elimination was interrupted.
*           Compress the column file and try again.
*           lfirst, lu and nfill have appropriate new values.

            call lu1rec( n, .true., luparm,
     $                   lcol, lena, a, indc, lenc, locc )
            lfile  = lcol
            lpivc  = locc(jbest)
            lpivc1 = lpivc + 1
            lpivc2 = lpivc + melim
            nfree  = lfree - lcol
            if (nfree .lt. minfre) go to 970
            go to 400
         end if

*        ===============================================================
*        The column file has been fully updated.
*        Deal with any pending fill-in in the row file.
*        ===============================================================
         if (nfill .gt. 0) then

*           Compress the row file if necessary.
*           lu1gau has set nfill to be the number of pending fill-ins
*           plus the current length of any rows that need to be moved.

            minfre = nfill
            nfree  = lfree - lrow
            if (nfree .lt. minfre) then
               call lu1rec( m, .false., luparm,
     $                      lrow, lena, a, indr, lenr, locr )
               lfile  = lrow
               lpivr  = locr(ibest)
               lpivr1 = lpivr + 1
               lpivr2 = lpivr + nelim
               nfree  = lfree - lrow
               if (nfree .lt. minfre) go to 970
            end if

*           Move rows that have pending fill-in to end of the row file.
*           Then insert the fill-in.

            call lu1pen( m     , melim , ncold , nspare,
     $                   lpivc1, lpivc2, lpivr1, lpivr2, lrow,
     $                   lenc  , lenr  , locc  , locr  ,
     $                   indc  , indr  , indr(ll1), indr(lu1) )
         end if

*        ===============================================================
*        Restore the saved values of  iqloc.
*        Insert the correct indices for the col of L and the row of U.
*        ===============================================================
  700    lenr(ibest) = 0
         lenc(jbest) = 0

         ll          = ll1 - 1
         ls          = lsave

         do 710  lc  = lpivc1, lpivc2
            ll       = ll + 1
            ls       = ls + 1
            i        = indc(lc)
            iqloc(i) = indr(ls)
            indc(ll) = i
            indr(ll) = ibest
  710    continue

         lu          = lu1 - 1

         do 720  lr  = lpivr, lpivr2
            lu       = lu + 1
            indr(lu) = indr(lr)
  720    continue

*        ===============================================================
*        Free the space occupied by the pivot row
*        and update the column permutation.
*        Then free the space occupied by the pivot column
*        and update the row permutation.
*
*        nzchng is found in both calls to lu1pq2, but we use it only
*        after the second.
*        ===============================================================
         call lu1pq2( ncold, nzchng,
     $                indr(lpivr), indc( lu1 ), lenc, iqloc, iq, iqinv )

         call lu1pq2( nrowd, nzchng,
     $                indc(lpivc), indc(lsave), lenr, iploc, ip, ipinv )

         nzleft = nzleft + nzchng

*        ===============================================================
*        Move the largest element to the top of each relevant column.
*        ===============================================================
         if ( spars1 ) then

*           Do all modified columns.

            if ( nelim .gt. 0 ) then
*              call timer ( 'start ', 16 )
               call lu1max( lu1+1, lu, indr, a, indc, lenc, locc )
*              call timer ( 'finish', 16 )
            end if

         else if ( spars2 .or. dense ) then

*           Just do the maxcol shortest columns.

*           call timer ( 'start ', 16 )
            lq1    = iqloc(1)
            lq2    = min( lq1 + maxcol - 1, n )
            call lu1max( lq1, lq2, iq, a, indc, lenc, locc )
*           call timer ( 'finish', 16 )
         end if

*        ===============================================================
*        Negate lengths of pivot row and column so they will be
*        eliminated during compressions.
*        ===============================================================
         lenr(ibest) = - ncold
         lenc(jbest) = - nrowd

*        Test for fatal bug: row or column lists overwriting L and U.

         if (lrow .gt. lsave) go to 980
         if (lcol .gt. lsave) go to 980

*        Reset the length of the row file if pivot row was at the end.
*        Similarly for the column file.

         if (lrow .eq. lpivr2) then
            lrow   = lpivr
            do 770 l  = 1, lpivr
               if (indr(lrow) .ne. 0) go to 780
               lrow   = lrow - 1
  770       continue
         end if

  780    if (lcol .eq. lpivc2) then
            lcol   = lpivc
            do 790 l  = 1, lpivc
               if (indc(lcol) .ne. 0) go to 800
               lcol   = lcol - 1
  790       continue
         end if
  800 continue

*     ------------------------------------------------------------------
*     End of main loop.
*     ------------------------------------------------------------------

*     ------------------------------------------------------------------
*     Normal exit.
*     Move empty rows and cols to the end of ip, iq.
*     Then finish with a dense LU if necessary.
*     ------------------------------------------------------------------
  900 inform = 0
      call lu1pq3( m, lenr, ip, ipinv, mrank )
      call lu1pq3( n, lenc, iq, iqinv, nrank )
      nrank  = min( mrank, nrank )

      if ( densLU ) then
*        call timer ( 'start ', 17 )
         call lu1ful( m     , n    , lena , lend , lu1 ,
     $                mleft , nleft, nrank, nrowu,
     $                lenl  , lenu , nsing,
     $                keepLU, small,
     $                a     , a(ld), indc , indr , ip  , iq,
     $                lenc  , lenr , locc , ipinv, locr )
****     21 Dec 1994: Bug in next line.
****     nrank  = nrank - nsing
         nrank  = minmn - nsing
*        call timer ( 'finish', 17 )
      end if

      minlen = lenl  +  lenu  +  2*(m + n)
      go to 990

*     Not enough space free after a compress.
*     Set  minlen  to an estimate of the necessary value of  lena.

  970 inform = 7
      minlen = lena  +  lfile  +  2*(m + n)
      go to 990

*     Fatal error.  This will never happen!
*    (Famous last words.)

  980 inform = 8

*     exit.

  990 luparm(17) = ndens1
      luparm(18) = ndens2
*     call timer ( 'finish', 3 )
      return

 1100 format(' lu1fad.  nrowu =', i7, '   ibest, jbest =', 2i7,
     $       '   nrowd, ncold =', 2i5)
*     end of lu1fad
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lu1gau( m     , melim , ncold , nspare, small,
     $                   lpivc1, lpivc2, lfirst, lpivr2, lfree, minfre,
     $                   lrow  , lcol  , lu    , nfill ,
     $                   a     , indc  , indr  ,
     $                   lenc  , lenr  , locc  , locr  ,
     $                   mark  , al    , markl ,
     $                           au    , ifill , jfill )

      implicit           double precision (a-h,o-z)
      double precision   a(*)        , al(melim)   , au(ncold)
      integer*4          indc(*)     , indr(*)     , lenc(*)  , lenr(*),
     $                   mark(*)     , markl(melim),
     $                   ifill(melim), jfill(ncold)
      integer            locc(*)     , locr(*)

*     ------------------------------------------------------------------
*     lu1gau does most of the work for each step of Gaussian elimination.
*     A multiple of the pivot column is added to each other column j
*     in the pivot row.  The column list is fully updated.
*     The row list is updated if there is room, but some fill-ins may
*     remain, as indicated by ifill and jfill.
*
*
*  Input:
*     lfirst   is the first column to be processed.
*     lu + 1   is the corresponding element of U in au(*).
*     nfill    keeps track of pending fill-in.
*     a(*)     contains the nonzeros for each column j.
*     indc(*)  contains the row indices for each column j.
*     al(*)    contains the new column of L.  A multiple of it is
*              used to modify each column.
*     mark(*)  has been set to -1, -2, -3, ... in the rows
*              corresponding to nonzero 1, 2, 3, ... of the col of L.
*     au(*)    contains the new row of U.  Each nonzero gives the
*              required multiple of the column of L.
*
*  Workspace:
*     markl(*) marks the nonzeros of L actually used.
*              (A different mark, namely j, is used for each column.)
*
*  Output:
*     lfirst    = 0 if all columns were completed,
*               > 0 otherwise.
*     lu        returns the position of the last nonzero of U
*               actually used, in case we come back in again.
*     nfill     keeps track of the total extra space needed in the
*               row file.
*     ifill(ll) counts pending fill-in for rows involved in the new
*               column of L.
*     jfill(lu) marks the first pending fill-in stored in columns
*               involved in the new row of U.
*
*     16 Apr 1989  First version.
*     23 Apr 1989  lfirst, lu, nfill are now input and output
*                  to allow re-entry if elimination is interrupted.
*     ------------------------------------------------------------------

      logical            atend

      do 600 lr = lfirst, lpivr2
         j      = indr(lr)
         lenj   = lenc(j)
         nfree  = lfree - lcol
         if (nfree .lt. minfre) go to 900

*        ---------------------------------------------------------------
*        Inner loop to modify existing nonzeros in column  j.
*        Loop 440 performs most of the arithmetic involved in the
*        whole LU factorization.
*        ndone counts how many multipliers were used.
*        ndrop counts how many modified nonzeros are negligibly small.
*        ---------------------------------------------------------------
         lu     = lu + 1
         uj     = au(lu)
         lc1    = locc(j)
         lc2    = lc1 + lenj - 1
         atend  = lcol .eq. lc2
         ndone  = 0
         if (lenj .eq. 0) go to 500

         ndrop  = 0

         do 440 l = lc1, lc2
            i        =   indc(l)
            ll       = - mark(i)
            if (ll .gt. 0) then
               ndone     = ndone + 1
               markl(ll) = j
               a(l)      = a(l)  +  al(ll) * uj
               if (abs( a(l) ) .le. small) then
                  ndrop  = ndrop + 1
               end if
            end if
  440    continue

*        ---------------------------------------------------------------
*        Remove any negligible modified nonzeros from both
*        the column file and the row file.
*        ---------------------------------------------------------------
         if (ndrop .eq. 0) go to 500
         k      = lc1

         do 480 l = lc1, lc2
            i        = indc(l)
            if (abs( a(l) ) .le. small) go to 460
            a(k)     = a(l)
            indc(k)  = i
            k        = k + 1
            go to 480

*           Delete the nonzero from the row file.

  460       lenj     = lenj    - 1
            lenr(i)  = lenr(i) - 1
            lr1      = locr(i)
            last     = lr1 + lenr(i)

            do 470 lrep = lr1, last
               if (indr(lrep) .eq. j) go to 475
  470       continue

  475       indr(lrep) = indr(last)
            indr(last) = 0
            if (lrow .eq. last) lrow = lrow - 1
  480    continue

*        Free the deleted elements.

         do 490  l  = k, lc2
            indc(l) = 0
  490    continue
         if (atend) lcol = k - 1

*        ---------------------------------------------------------------
*        Deal with the fill-in in column  j.
*        ---------------------------------------------------------------
  500    if (ndone .eq. melim) go to 590

*        See if column j already has room for the fill-in.

         if (atend) go to 540
         last   = lc1  + lenj - 1
         l1     = last + 1
         l2     = last + (melim - ndone)

         do 510 l = l1, l2
            if (indc(l) .gt. 0) go to 520
  510    continue
         go to 540

*        We must move column  j  to the end of the column file.
*        Leave some spare room at the end of the current last column.

  520    atend   = .true.

         do 522  l  = lcol + 1, lcol + nspare
            lcol    = l
            indc(l) = 0
  522    continue

         l1      = lc1
         lc1     = lcol + 1
         locc(j) = lc1

         do 525  l     = l1, last
            lcol       = lcol + 1
            a(lcol)    = a(l)
            indc(lcol) = indc(l)
            indc(l)    = 0
  525    continue

*        ---------------------------------------------------------------
*        Inner loop for the fill-in in column  j.
*        This is usually not very expensive.
*        ---------------------------------------------------------------
  540    indr(lrow+1)  = 0
         last          = lc1 + lenj - 1
         ll            = 0

         do 560   lc   = lpivc1, lpivc2
            ll         = ll + 1
            if (markl(ll)  .eq. j    ) go to 560
            aij        = al(ll) * uj
            if (abs( aij ) .le. small) go to 560
            lenj       = lenj + 1
            last       = last + 1
            a(last)    = aij
            i          = indc(lc)
            indc(last) = i
            leni       = lenr(i)

*           Add the fill-in to row  i  if there is room.

            l          = locr(i) + leni
            if (indr(l) .eq. 0) then
               indr(l)    = j
               lenr(i)    = leni + 1
               if (lrow .lt. l) lrow = l
            else

*              Row i does not have room for the fill-in.
*              Increment ifill(ll) to count how often this has
*              happened to row i.  Also, add m to the row index
*              indc(last) in column j to mark it as a fill-in that is
*              still pending.
*
*              If this is the first pending fill-in for row i,
*              nfill includes the current length of row i
*              (since the whole row has to be moved later).
*
*              If this is the first pending fill-in for column j,
*              jfill(lu) records the current length of column j
*              (to shorten the search for pending fill-ins later).

               if (ifill(ll) .eq. 0) nfill     = nfill + leni + nspare
               if (jfill(lu) .eq. 0) jfill(lu) = lenj
               nfill      = nfill     + 1
               ifill(ll)  = ifill(ll) + 1
               indc(last) = m + i
            end if
  560    continue

         if ( atend ) lcol = last

*        End loop for column  j.  Store its final length.

  590    lenc(j) = lenj
  600 continue

*     Successful completion.

      lfirst = 0
      return

*     Interruption.  We have to come back in after the
*     column file is compressed.  Give lfirst a new value.
*     lu and nfill will retain their current values.

  900 lfirst = lr
      return

*     end of lu1gau
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lu1mar( m    , n     , lena  , maxmn,
     $                   lmax , maxcol, maxrow,
     $                   ibest, jbest , mbest ,
     $                   a    , indc  , indr  , ip   , iq,
     $                   lenc , lenr  , locc  , locr ,
     $                   iploc, iqloc )

      implicit           double precision (a-h,o-z)
      double precision   lmax      , a(lena)
      integer*4          indc(lena), indr(lena), ip(m)   , iq(n)   ,
     $                   lenc(n)   , lenr(m)   , iploc(n), iqloc(m)
      integer            locc(n)   , locr(m)

*     ------------------------------------------------------------------
*     lu1mar  uses a Markowitz criterion to select a pivot element
*     for the next stage of a sparse LU factorization,
*     subject to a stability criterion that bounds the elements of  L.
*
*     00 Jan 1986  Version documented in LUSOL paper:
*                  Gill, Murray, Saunders and Wright (1987),
*                  Maintaining LU factors of a general sparse matrix,
*                  Linear algebra and its applications 88/89, 239-270.
*
*     02 Feb 1989  Following Suhl and Aittoniemi (1987), the largest
*                  element in each column is now kept at the start of
*                  the column, i.e. in position locc(j) of a and indc.
*                  This should speed up the Markowitz searches.
*
*     26 Apr 1989  Both columns and rows searched during spars1 phase.
*                  Only columns searched during spars2 phase.
*                  maxtie replaced by maxcol and maxrow.
*     05 Nov 1993  Initializing  "mbest = m * n"  wasn't big enough when
*                  m = 10, n = 3, and last column had 7 nonzeros.
*     09 Feb 1994  Realised that "mbest = maxmn * maxmn" might overflow.
*                  Changed to    "mbest = maxmn * 1000".
*
*     Systems Optimization Laboratory, Stanford University.
*     ------------------------------------------------------------------

      double precision   lbest
      double precision   zero,           gamma
      parameter        ( zero = 0.0d+0,  gamma  = 2.0d+0 )

*     gamma  is "gamma" in the tie-breaking rule TB4 in the LUSOL paper.

*     ------------------------------------------------------------------
*     Search cols of length nz = 1, then rows of length nz = 1,
*     then   cols of length nz = 2, then rows of length nz = 2, etc.
*     ------------------------------------------------------------------
      mbest  = maxmn * 1000
      ncol   = 0
      nrow   = 0

      do 300 nz = 1, maxmn
         nz1    = nz - 1
         if (ncol  .ge. maxcol) go to 200
         if (mbest .le. nz1**2) go to 900
         if (nz    .gt. m     ) go to 200

*        ---------------------------------------------------------------
*        Search the set of columns of length  nz.
*        ---------------------------------------------------------------
         lq1    = iqloc(nz)
         lq2    = n
         if (nz .lt. m) lq2 = iqloc(nz + 1) - 1

         do 180 lq = lq1, lq2
            ncol   = ncol + 1
            j      = iq(lq)
            lc1    = locc(j)
            lc2    = lc1 + nz1
            amax   = abs( a(lc1) )

*           Test all aijs in this column.
*           amax is the largest element (the first in the column).
*           cmax is the largest multiplier if aij becomes pivot.

            do 160 lc = lc1, lc2
               i      = indc(lc)
               merit  = nz1 * (lenr(i) - 1)
               if (merit .gt. mbest) go to 160

*              aij  has a promising merit.
*              Apply the stability test.
*              We require  aij  to be sufficiently large compared to
*              all other nonzeros in column  j.  This is equivalent
*              to requiring cmax to be bounded by lmax.

               if (lc .eq. lc1) then

*                 This is the maximum element, amax.
*                 Find the biggest element in the rest of the column
*                 and hence get cmax.  We know cmax .le. 1, but
*                 we still want it exactly in order to break ties.

                  aij    = amax
                  cmax   = zero
                  do 140 l = lc1 + 1, lc2
                     cmax  = max( cmax, abs( a(l) ) )
  140             continue
                  cmax   = cmax / amax
               else

*                 aij is not the biggest element, so cmax .ge. 1.
*                 Bail out if cmax will be too big.

                  aij    = abs( a(lc) )
                  if (amax  .gt.  aij * lmax) go to 160
                  cmax   = amax / aij
               end if

*              aij  is big enough.  Its maximum multiplier is cmax.
*              Accept it immediately if its merit is less than mbest.

               if (merit .eq. mbest) then

*                 Break ties  (merit = mbest).
*                 In this version we minimize cmax
*                 but if it is already small we maximize the pivot.

                  if (lbest .le. gamma  .and.  cmax .le. gamma) then
                     if (abest .ge. aij ) go to 160
                  else
                     if (lbest .le. cmax) go to 160
                  end if
               end if

*              aij  is the best pivot so far.

               ibest  = i
               jbest  = j
               mbest  = merit
               abest  = aij
               lbest  = cmax
  160       continue

*           Finished with that column.

            if (ncol .ge. maxcol) go to 200
  180    continue

*        ---------------------------------------------------------------
*        Search the set of rows of length  nz.
*        ---------------------------------------------------------------
  200    if (nrow  .ge. maxrow) go to 290
         if (mbest .le. nz*nz1) go to 900
         if (nz    .gt. n     ) go to 300
         lp1    = iploc(nz)
         lp2    = m
         if (nz .lt. n) lp2 = iploc(nz + 1) - 1

         do 280 lp = lp1, lp2
            nrow   = nrow + 1
            i      = ip(lp)
            leni   = lenr(i)
            lr1    = locr(i)
            lr2    = lr1 + leni - 1

            do 260 lr = lr1, lr2
               j      = indr(lr)
               merit  = nz1 * (lenc(j) - 1)
               if (merit .gt. mbest) go to 260

*              aij  has a promising merit.
*              Find where  aij  is in column  j.

               lenj   = lenc(j)
               lc1    = locc(j)
               lc2    = lc1 + lenj - 1
               amax   = abs( a(lc1) )
               do 220 lc = lc1, lc2
                  if (indc(lc) .eq. i) go to 230
  220          continue

*              Apply the same stability test as above.

  230          if (lc .eq. lc1) then

*                 This is the maximum element, amax.
*                 Find the biggest element in the rest of the column
*                 and hence get cmax.  We know cmax .le. 1, but
*                 we still want it exactly in order to break ties.

                  aij    = amax
                  cmax   = zero
                  do 240 l = lc1 + 1, lc2
                     cmax  = max( cmax, abs( a(l) ) )
  240             continue
                  cmax   = cmax / amax
               else

*                 aij is not the biggest element, so cmax .ge. 1.
*                 Bail out if cmax will be too big.

                  aij    = abs( a(lc) )
                  if (amax  .gt.  aij * lmax) go to 260
                  cmax   = amax / aij
               end if

*              aij  is big enough.  Its maximum multiplier is cmax.
*              Accept it immediately if its merit is less than mbest.

               if (merit .eq. mbest) then

*                 Break ties as before (merit = mbest).

                  if (lbest .le. gamma  .and.  cmax .le. gamma) then
                     if (abest .ge. aij ) go to 260
                  else
                     if (lbest .le. cmax) go to 260
                  end if
               end if

*              aij  is the best pivot so far.

               ibest  = i
               jbest  = j
               mbest  = merit
               abest  = aij
               lbest  = cmax
               if (nz .eq. 1) go to 900
  260       continue

*           Finished with that row.

            if (nrow .ge. maxrow) go to 290
  280    continue

*        See if it's time to quit.

  290    if (nrow .ge. maxrow  .and.  ncol .ge. maxcol) go to 900
  300 continue

  900 return

*     end of lu1mar
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lu1pen( m     , melim , ncold , nspare,
     $                   lpivc1, lpivc2, lpivr1, lpivr2, lrow,
     $                   lenc  , lenr  , locc  , locr  ,
     $                   indc  , indr  , ifill , jfill )

      integer*4          indc(*)     , indr(*)     , lenc(*), lenr(*),
     $                   ifill(melim), jfill(ncold)
      integer            locc(*)     , locr(*)

*     ------------------------------------------------------------------
*     lu1pen deals with pending fill-in in the row file.
*     ifill(ll) says if a row involved in the new column of L
*               has to be updated.  If positive, it is the total
*               length of the final updated row.
*     jfill(lu) says if a column involved in the new row of U
*               contains any pending fill-ins.  If positive, it points
*               to the first fill-in in the column that has yet to be
*               added to the row file.
*     ------------------------------------------------------------------

*     Move the rows that have pending fill-in
*     to the end of the row file, leaving space for the fill-in.
*     Leave some spare room at the end of the current last row.

         ll     = 0

         do 650 lc  = lpivc1, lpivc2
            ll      = ll + 1
            if (ifill(ll) .eq. 0) go to 650

            do 620  l  = lrow + 1, lrow + nspare
               lrow    = l
               indr(l) = 0
  620       continue

            i       = indc(lc)
            lr1     = locr(i)
            lr2     = lr1 + lenr(i) - 1
            locr(i) = lrow + 1

            do 630 lr = lr1, lr2
               lrow       = lrow + 1
               indr(lrow) = indr(lr)
               indr(lr)   = 0
  630       continue

            lrow    = lrow + ifill(ll)
  650    continue

*        Scan all columns of  D  and insert the pending fill-in
*        into the row file.

         lu     = 1

         do 680 lr = lpivr1, lpivr2
            lu     = lu + 1
            if (jfill(lu) .eq. 0) go to 680
            j      = indr(lr)
            lc1    = locc(j) + jfill(lu) - 1
            lc2    = locc(j) + lenc(j)   - 1

            do 670 lc = lc1, lc2
               i      = indc(lc) - m
               if (i .gt. 0) then
                  indc(lc)   = i
                  last       = locr(i) + lenr(i)
                  indr(last) = j
                  lenr(i)    = lenr(i) + 1
               end if
  670       continue
  680    continue

*     end of lu1pen
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lu1max( kol1, kol2, kol,
     $                   a   , indc, lenc, locc )

      implicit           double precision (a-h,o-z)
      double precision   a(*)
      integer*4          kol(kol2), indc(*), lenc(*)
      integer            locc(*)

*     ------------------------------------------------------------------
*     lu1max  moves the largest element in each of a set of columns
*     to the top of its column.
*     ------------------------------------------------------------------

      do 120  k = kol1, kol2
         j      = kol(k)
         lc1    = locc(j)

*        The next 10 lines are equivalent to
*        l      = idamax( lenc(j), a(lc1), 1 )  +  lc1 - 1
*>>>>>>>>
         lc2    = lc1 + lenc(j) - 1
         amax   = abs( a(lc1) )
         l      = lc1

         do 110 lc = lc1 + 1, lc2
            if (amax .lt. abs( a(lc) )) then
                amax   =  abs( a(lc) )
                l      =  lc
            end if
  110    continue
*>>>>>>>>
         if (l .gt. lc1) then
            amax      = a(l)
            a(l)      = a(lc1)
            a(lc1)    = amax
            i         = indc(l)
            indc(l)   = indc(lc1)
            indc(lc1) = i
         end if
  120 continue

*     end of lu1max
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lu1or1( m, n, nelem, small,
     $                   a, indc, indr, lenc, lenr,
     $                   amax, numnz, lerr, inform )

      implicit           double precision (a-h,o-z)
      double precision   a(nelem)
      integer*4          indc(nelem), indr(nelem)
      integer*4          lenc(n), lenr(m)

*     ------------------------------------------------------------------
*     lu1or1  organizes the elements of an  m by n  matrix  A  as
*     follows.  On entry, the parallel arrays   a, indc, indr,
*     contain  nelem  entries of the form     aij,    i,    j,
*     in any order.  nelem  must be positive.
*
*     Entries not larger than the input parameter  small  are treated as
*     zero and removed from   a, indc, indr.  The remaining entries are
*     defined to be nonzero.  numnz  returns the number of such nonzeros
*     and  amax  returns the magnitude of the largest nonzero.
*     The arrays  lenc, lenr  return the number of nonzeros in each
*     column and row of  A.
*
*     inform = 0  on exit, except  inform = 1  if any of the indices in
*     indc, indr  imply that the element  aij  lies outside the  m by n
*     dimensions of  A.
*
*     Version of February 1985.
*     ------------------------------------------------------------------
      do 10 i = 1, m
         lenr(i) = 0
   10 continue

      do 20 j = 1, n
         lenc(j) = 0
   20 continue

      amax   = 0.0d+0
      numnz  = nelem
      l      = nelem + 1

      do 100 ldummy = 1, nelem
         l      = l - 1
         if (abs( a(l) ) .gt. small) then
            i      = indc(l)
            j      = indr(l)
            amax   = max( amax, abs( a(l) ) )
            if (i .lt. 1  .or.  i .gt. m) go to 910
            if (j .lt. 1  .or.  j .gt. n) go to 910
            lenr(i) = lenr(i) + 1
            lenc(j) = lenc(j) + 1
         else

*           Replace a negligible element by last element.  Since
*           we are going backwards, we know the last element is ok.

            a(l)    = a(numnz)
            indc(l) = indc(numnz)
            indr(l) = indr(numnz)
            numnz   = numnz - 1
         end if
  100 continue

      inform = 0
      return

  910 lerr   = l
      inform = 1
      return

*     end of lu1or1
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lu1or2( n, numa, a, inum, jnum, len, loc )
      integer*4          inum(numa), jnum(numa), len(n)
      integer            loc(n)
      double precision   a(numa), ace, acep

*     ------------------------------------------------------------------
*     lu1or2  sorts a list of matrix elements  a(i,j)  into column
*     order, given  numa  entries  a(i,j),  i,  j  in the parallel
*     arrays  a, inum, jnum  respectively.  The matrix is assumed
*     to have  n  columns and an arbitrary number of rows.
*
*     On entry,  len(*)  must contain the length of each column.
*
*     On exit,  a(*) and inum(*)  are sorted,  jnum(*) = 0,  and
*     loc(j)  points to the start of column j.
*
*     lu1or2  is derived from  mc20ad,  a routine in the Harwell
*     Subroutine Library, author J. K. Reid.
*     ------------------------------------------------------------------

*     Set  loc(j)  to point to the beginning of column  j.

      l = 1
      do 150 j  = 1, n
         loc(j) = l
         l      = l + len(j)
  150 continue

*     Sort the elements into column order.
*     The algorithm is an in-place sort and is of order  numa.

      do 230 i = 1, numa
*        Establish the current entry.
         jce     = jnum(i)
         if (jce .eq. 0) go to 230
         ace     = a(i)
         ice     = inum(i)
         jnum(i) = 0

*        Chain from current entry.

         do 200 j = 1, numa

*           The current entry is not in the correct position.
*           Determine where to store it.

            l        = loc(jce)
            loc(jce) = loc(jce) + 1

*           Save the contents of that location.

            acep = a(l)
            icep = inum(l)
            jcep = jnum(l)

*           Store current entry.

            a(l)    = ace
            inum(l) = ice
            jnum(l) = 0

*           If next current entry needs to be processed,
*           copy it into current entry.

            if (jcep .eq. 0) go to 230
            ace = acep
            ice = icep
            jce = jcep
  200    continue
  230 continue

*     Reset loc(j) to point to the start of column j.

      ja = 1
      do 250 j  = 1, n
         jb     = loc(j)
         loc(j) = ja
         ja     = jb
  250 continue

*     end of lu1or2
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lu1or3( m, n, nelem,
     $                   indc, lenc, locc, iw,
     $                   lerr, inform )

      integer*4          indc(nelem), lenc(n), iw(m)
      integer            locc(n)

*     ------------------------------------------------------------------
*     lu1or3  looks for duplicate elements in an  m by n  matrix  A
*     defined by the column list  indc, lenc, locc.
*     iw  is used as a work vector of length  m.
*
*     Version of February 1985.
*     ------------------------------------------------------------------

      do 100 i = 1, m
         iw(i) = 0
  100 continue

      do 200 j = 1, n
         if (lenc(j) .gt. 0) then
            l1    = locc(j)
            l2    = l1 + lenc(j) - 1

            do 150 l = l1, l2
               i     = indc(l)
               if (iw(i) .eq. j) go to 910
               iw(i) = j
  150       continue
         end if
  200 continue

      inform = 0
      return

  910 lerr   = l
      inform = 1
      return

*     end of lu1or3
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lu1or4( m, n, nelem,
     $                   indc, indr, lenc, lenr, locc, locr )

      integer*4          indc(nelem), indr(nelem), lenc(n), lenr(m)
      integer            locc(n), locr(m)

*     ------------------------------------------------------------------
*     lu1or4     constructs a row list  indr, locr
*     from a corresponding column list  indc, locc,
*     given the lengths of both columns and rows in  lenc, lenr.
*     ------------------------------------------------------------------

*     Initialize  locr(i)  to point just beyond where the
*     last component of row  i  will be stored.

      l      = 1
      do 10 i = 1, m
         l       = l + lenr(i)
         locr(i) = l
   10 continue

*     By processing the columns backwards and decreasing  locr(i)
*     each time it is accessed, it will end up pointing to the
*     beginning of row  i  as required.

      l2     = nelem
      j      = n + 1

      do 40 jdummy = 1, n
         j  = j - 1
         if (lenc(j) .gt. 0) then
            l1 = locc(j)

            do 30 l = l1, l2
               i        = indc(l)
               lr       = locr(i) - 1
               locr(i)  = lr
               indr(lr) = j
   30       continue

            l2     = l1 - 1
         end if
   40 continue

*     end of lu1or4
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lu1pq1( m, n, len, iperm, loc, inv, num )

      integer*4          len(m), iperm(m), loc(n), inv(m), num(n)

*     ------------------------------------------------------------------
*     lu1pq1  constructs a permutation  iperm  from the array  len.
*
*     On entry:
*     len(i)  holds the number of nonzeros in the i-th row (say)
*             of an m by n matrix.
*     num(*)  can be anything (workspace).
*
*     On exit:
*     iperm   contains a list of row numbers in the order
*             rows of length 0,  rows of length 1,..., rows of length n.
*     loc(nz) points to the first row containing  nz  nonzeros,
*             nz = 1, n.
*     inv(i)  points to the position of row i within iperm(*).
*     ------------------------------------------------------------------

*     Count the number of rows of each length.

      nzero  = 0
      do 10  nz  = 1, n
         num(nz) = 0
         loc(nz) = 0
   10 continue

      do 20  i   = 1, m
         nz      = len(i)
         if (nz .eq. 0) then
            nzero   = nzero   + 1
         else
            num(nz) = num(nz) + 1
         end if
   20 continue

*     Set starting locations for each length.

      l      = nzero + 1
      do 60  nz  = 1, n
         loc(nz) = l
         l       = l + num(nz)
         num(nz) = 0
   60 continue

*     Form the list.

      nzero  = 0
      do 100  i   = 1, m
         nz       = len(i)
         if (nz .eq. 0) then
            nzero    = nzero + 1
            iperm(nzero) = i
         else
            l        = loc(nz) + num(nz)
            iperm(l) = i
            num(nz)  = num(nz) + 1
         end if
  100 continue

*     Define the inverse of iperm.

      do 120 l  = 1, m
         i      = iperm(l)
         inv(i) = l
  120 continue

*     end of lu1pq1
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lu1pq2( nzpiv, nzchng,
     $                   indr , lenold, lennew, iqloc, iq, iqinv )

      integer*4          indr(nzpiv), lenold(nzpiv), lennew(*),
     $                   iqloc(*)   , iq(*)        , iqinv(*)

*     ===============================================================
*     lu1pq2 frees the space occupied by the pivot row,
*     and updates the column permutation iq.
*
*     Also used to free the pivot column and update the row perm ip.
*
*     nzpiv   (input)    is the length of the pivot row (or column).
*     nzchng  (output)   is the net change in total nonzeros.
*
*     14 Apr 1989  First version.
*     ===============================================================

      nzchng = 0

      do 200  lr  = 1, nzpiv
         j        = indr(lr)
         indr(lr) = 0
         nz       = lenold(lr)
         nznew    = lennew(j)

         if (nz .ne. nznew) then
            l        = iqinv(j)
            nzchng   = nzchng + (nznew - nz)

*           l above is the position of column j in iq  (so j = iq(l)).

            if (nz .lt. nznew) then

*              Column  j  has to move towards the end of  iq.

  110          next        = nz + 1
               lnew        = iqloc(next) - 1
               if (lnew .ne. l) then
                  jnew        = iq(lnew)
                  iq(l)       = jnew
                  iqinv(jnew) = l
               end if
               l           = lnew
               iqloc(next) = lnew
               nz          = next
               if (nz .lt. nznew) go to 110
            else

*              Column  j  has to move towards the front of  iq.

  120          lnew        = iqloc(nz)
               if (lnew .ne. l) then
                  jnew        = iq(lnew)
                  iq(l)       = jnew
                  iqinv(jnew) = l
               end if
               l           = lnew
               iqloc(nz)   = lnew + 1
               nz          = nz   - 1
               if (nz .gt. nznew) go to 120
            end if

            iq(lnew) = j
            iqinv(j) = lnew
         end if
  200 continue

*     end of lu1pq2
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lu1pq3( n, len, iperm, iw, nrank )

      integer*4          len(n), iperm(n)
      integer            iw(n)

*     ------------------------------------------------------------------
*     lu1pq3  looks at the permutation  iperm(*)  and moves any entries
*     to the end whose corresponding length  len(*)  is zero.
*
*     09 Feb 1994: Added work array iw(*) to improve efficiency.
*     ------------------------------------------------------------------

      nrank  = 0
      nzero  = 0

      do 10 k = 1, n
         i    = iperm(k)

         if (len(i) .eq. 0) then
            nzero        = nzero + 1
            iw(nzero)    = i
         else
            nrank        = nrank + 1
            iperm(nrank) = i
         end if
   10 continue

      do 20 k = 1, nzero
         iperm(nrank + k) = iw(k)
   20 continue

*     end of lu1pq3
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lu1rec( n, reals, luparm,
     $                   ltop, lena, a, ind, len, loc )

      logical            reals
      integer            luparm(30)
      double precision   a(lena)
      integer*4          ind(lena), len(n)
      integer            loc(n)

*     ------------------------------------------------------------------
*     lu1rec  recovers space in  ind(*)  and optionally  a(*),
*     by eliminating entries for which  ind(l)  is zero.
*     The elements of  ind(*)  must not be negative.
*
*     If  len(i)  is positive, entry  i  contains that many elements,
*     starting at  loc(i).  Otherwise, entry  i  is eliminated.
*     ------------------------------------------------------------------

      do 10 i = 1, n
         leni = len(i)
         if (leni .gt. 0) then
            l      = loc(i) + leni - 1
            len(i) = ind(l)
            ind(l) = - i
         end if
   10 continue

      k      = 0
      last   = 0

      do 20 l = 1, ltop
         if (ind(l) .ne. 0) then
            k      = k + 1
            i      = ind(l)
            ind(k) = i
            if (reals) a(k) = a(l)
            if (i .lt. 0) then

*              This is the end of entry  i.

               i      = - i
               ind(k) = len(i)
               loc(i) = last + 1
               len(i) = k    - last
               last   = k
            end if
         end if
   20 continue

      nout   = luparm(1)
      lprint = luparm(2)
      if (lprint .ge. 1) write(nout, 1000) ltop, k, reals
      luparm(26) = luparm(26) + 1
      ltop       = k
      return

 1000 format(' lu1rec.   File compressed from', i8, '   to', i8, l4)

*     end of lu1rec
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lu1ful( m     , n    , lena , lend , lu1 ,
     $                   mleft , nleft, nrank, nrowu,
     $                   lenl  , lenu , nsing,
     $                   keepLU, small,
     $                   a     , d    , indc , indr , ip  , iq,
     $                   lenc  , lenr , locc , ipinv, ipvt )

      implicit           double precision (a-h,o-z)
      logical            keepLU
      double precision   a(lena)   , d(lend)
      integer*4          indc(lena), indr(lena), ip(m)   , iq(n),
     $                   lenc(n)   , lenr(m)   , ipinv(m)
      integer            locc(n)   , ipvt(m)
 
*     ------------------------------------------------------------------
*     lu1ful computes a dense (full) LU factorization of the
*     mleft by nleft matrix that remains to be factored at the
*     beginning of the nrowu-th pass through the main loop of lu1fad.
*
*     02 May 1989: First version.
*     05 Feb 1994: Column interchanges added to lu1den.
*     08 Feb 1994: ipinv reconstructed, since lu1pq3 may alter ip.
*     ------------------------------------------------------------------

      parameter        ( zero = 0.0d+0 )


*     ------------------------------------------------------------------
*     If lu1pq3 moved any empty rows, reset ipinv = inverse of ip.
*     ------------------------------------------------------------------
      if (nrank .lt. m) then
         do 100 l    = 1, m
            i        = ip(l)
            ipinv(i) = l
  100    continue
      end if

*     ------------------------------------------------------------------
*     Copy the remaining matrix into the dense matrix D.
*     ------------------------------------------------------------------
      call dload ( lend, zero, d, 1 )
      ipbase = nrowu - 1
      ldbase = 1 - nrowu

      do 200 lq = nrowu, n
         j      = iq(lq)
         lc1    = locc(j)
         lc2    = lc1 + lenc(j) - 1 

         do 150 lc = lc1, lc2
            i      = indc(lc)
            ld     = ldbase + ipinv(i)
            d(ld)  = a(lc)
  150    continue

         ldbase = ldbase + mleft
  200 continue

*     ------------------------------------------------------------------
*     Call our favorite dense LU factorizer.
*     ------------------------------------------------------------------
      call lu1den( d, mleft, mleft, nleft, small, nsing,
     $             ipvt, iq(nrowu) )

*     ------------------------------------------------------------------
*     Move D to the beginning of A,
*     and pack L and U at the top of a, indc, indr.
*     In the process, apply the row permutation to ip.
*     lkk points to the diagonal of U.
*     ------------------------------------------------------------------
      call dcopy ( lend, d, 1, a, 1 )

      ldiagU = lena   - n
      lkk    = 1
      lkn    = lend  - mleft + 1
      lu     = lu1

      do 450  k = 1, min( mleft, nleft )
         l1     = ipbase + k
         l2     = ipbase + ipvt(k)
         if (l1 .ne. l2) then
            i      = ip(l1)
            ip(l1) = ip(l2)
            ip(l2) = i
         end if
         ibest  = ip( l1 )
         jbest  = iq( l1 )

         if ( keepLU ) then
*           ===========================================================
*           Pack the next column of L.
*           ===========================================================
            la     = lkk
            ll     = lu
            nrowd  = 1

            do 410  i = k + 1, mleft
               la     = la + 1
               ai     = a(la)
               if (abs( ai ) .gt. small) then
                  nrowd    = nrowd + 1
                  ll       = ll    - 1
                  a(ll)    = ai
                  indc(ll) = ip( ipbase + i )
                  indr(ll) = ibest
               end if
  410       continue

*           ===========================================================
*           Pack the next row of U.
*           We go backwards through the row of D
*           so the diagonal ends up at the front of the row of  U.
*           Beware -- the diagonal may be zero.
*           ===========================================================
            la     = lkn + mleft
            lu     = ll
            ncold  = 0

            do 420  j = nleft, k, -1
               la     = la - mleft
               aj     = a(la)
               if (abs( aj ) .gt. small  .or.  j .eq. k) then
                  ncold    = ncold + 1
                  lu       = lu    - 1
                  a(lu)    = aj
                  indr(lu) = iq( ipbase + j )
               end if
  420       continue
         
            lenr(ibest) = - ncold
            lenc(jbest) = - nrowd
            lenl        =   lenl + nrowd - 1
            lenu        =   lenu + ncold
            lkn         =   lkn  + 1

         else
*           ===========================================================
*           Store just the diagonal of U, in natural order.
*           ===========================================================
            a(ldiagU + jbest) = a(lkk)
         end if

         lkk    = lkk  + mleft + 1
  450 continue

*     end of lu1ful
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lu1den( a, lda, m, n, small, nsing,
     $                   ipvt, iq )

      implicit           double precision (a-h,o-z)
      integer            lda, m, n, nsing
      integer            ipvt(m), iq(n)
      double precision   a(lda,n), small

*     ------------------------------------------------------------------
*     lu1den factors a dense m x n matrix A by Gaussian elimination,
*     using row interchanges for stability, as in dgefa from LINPACK.
*     This version also uses column interchanges if all elements in a
*     pivot column are smaller than (or equal to) "small".  Such columns
*     are changed to zero and permuted to the right-hand end.
*
*     As in LINPACK, ipvt(*) keeps track of pivot rows.
*     Rows of U are interchanged, but we don't have to physically
*     permute rows of L.  In contrast, column interchanges are applied
*     directly to the columns of both L and U, and to the column
*     permutation vector iq(*).
*     
*     02 May 1989: First version derived from dgefa
*                  in LINPACK (version dated 08/14/78).
*     05 Feb 1994: Generalized to treat rectangular matrices
*                  and use column interchanges when necessary.
*                  ipvt is retained, but column permutations are applied
*                  directly to iq(*).
*     21 Dec 1994: Bug found via example from Steve Dirkse.
*                  Loop 100 added to set ipvt(*) for singular rows.
*     ------------------------------------------------------------------
*
*     On entry:
*
*        a       Array holding the matrix A to be factored.
*
*        lda     The leading dimension of the array  a.
*
*        m       The number of rows    in  A.
*
*        n       The number of columns in  A.
*
*        small   A drop tolerance.  Must be zero or positive.
*
*     On exit:
*
*        a       An upper triangular matrix and the multipliers
*                which were used to obtain it.
*                The factorization can be written  A = L*U  where
*                L  is a product of permutation and unit lower
*                triangular matrices and  U  is upper triangular.
*
*        nsing   Number of singularities detected.
*
*        ipvt    Records the pivot rows.
*
*        iq      A vector to which column interchanges are applied.
*     ------------------------------------------------------------------

      double precision    t
      integer             idamax, i, j, k, kp1, l, last
      double precision    zero         ,  one
      parameter         ( zero = 0.0d+0,  one = 1.0d+0 )


      nsing  = 0
      k      = 1
      last   = n

*     ------------------------------------------------------------------
*     Start of elimination loop.
*     ------------------------------------------------------------------
   10 kp1    = k + 1
      lencol = m - k + 1

*     Find l, the pivot row.

      l       = idamax( lencol, a(k,k), 1 ) + k - 1
      ipvt(k) = l

      if (abs( a(l,k) ) .le. small) then
*        ===============================================================
*        Do column interchange, changing old pivot column to zero.
*        Reduce  "last"  and try again with same k.
*        ===============================================================
         nsing    = nsing + 1
         j        = iq(last)
         iq(last) = iq(k)
         iq(k)    = j

         do 20 i = 1, k - 1
            t         = a(i,last)
            a(i,last) = a(i,k)
            a(i,k)    = t
   20    continue

         do 30 i = k, m
            t         = a(i,last)
            a(i,last) = zero
            a(i,k)    = t
   30    continue

         last     = last - 1
         if (k .le. last) go to 10

      else if (m .gt. k) then
*        ===============================================================
*        Do row interchange if necessary.
*        ===============================================================
         if (l .ne. k) then
            t      = a(l,k)
            a(l,k) = a(k,k)
            a(k,k) = t
         end if

*        ===============================================================
*        Compute multipliers.
*        Do row elimination with column indexing.
*        ===============================================================
         t = - one / a(k,k)
         call dscal ( m-k, t, a(kp1,k), 1 )

         do 40 j = kp1, last
            t    = a(l,j)
            if (l .ne. k) then
               a(l,j) = a(k,j)
               a(k,j) = t
            end if
            call daxpy ( m-k, t, a(kp1,k), 1, a(kp1,j), 1 )
   40    continue

         k = k + 1
         if (k .le. last) go to 10
      end if

*     Set ipvt(*) for singular rows.

      do 100 k = last + 1, m
         ipvt(k) = k
  100 continue
      
*     end of lu1den
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
*     file  lu6a    fortran
*
*     lu6chk   lu6sol
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lu6chk( mode, m, n, w,
     $                   lena, luparm, parmlu,
     $                   a, indc, indr, ip, iq,
     $                   lenc, lenr, locc, locr,
     $                   inform )

      implicit           double precision (a-h,o-z)
      integer            luparm(30)
      double precision   parmlu(30), a(lena), w(n)
      integer*4          indc(lena), indr(lena), ip(m), iq(n)
      integer*4          lenc(n), lenr(m)
      integer            locc(n), locr(m)

*     ------------------------------------------------------------------
*     lu6chk  looks at the LU factorization  A = L*U.
*
*     If  mode = 1,  the important input parameters are
*
*                    utol1  = parmlu(4),
*                    utol2  = parmlu(5),
*
*     and the significant output parameters are
*
*                    inform = luparm(10),
*                    nsing  = luparm(11),
*                    jsing  = luparm(12),
*                    jumin  = luparm(19),
*                    elmax  = parmlu(11),
*                    umax   = parmlu(12).
*                    dumax  = parmlu(13),
*                    dumin  = parmlu(14),
*                    and      w(*).
*
*     elmax  and  umax   return the largest elements in  L  and  U.
*     dumax  and  dumin  return the largest and smallest diagonals of  U
*                        (excluding diagonals that are exactly zero).
*
*     In general,  w(j)  is set to the maximum absolute element in
*     the j-th column of  U.  However, if the corresponding diagonal
*     of  U  is small in absolute terms or relative to  w(j)
*     (as judged by the parameters  utol1, utol2  respectively),
*     then  w(j)  is changed to  - w(j).
*
*     Thus, if  w(j)  is not positive, the j-th column of  A
*     appears to be dependent on the other columns of  A.
*     The number of such columns, and the position of the last one,
*     are returned as  nsing  and  jsing.
*
*     Note that nrank is assumed to be set already, and is not altered.
*     typically, nsing will satisfy     nrank + nsing  = n,  but if
*     utol1 and utol2 are rather large, nsing may exceed n - nrank.
*
*     inform = 0  if  A  appears to have full column rank  (nsing = 0).
*     inform = 1  otherwise  (nsing .gt. 0).
*
*     Version of July 1987.
*     9 may 1988: f77 version.
*     ------------------------------------------------------------------

      parameter        ( zero = 0.0d+0 )

      nout   = luparm(1)
      lprint = luparm(2)
      nrank  = luparm(16)
      lenl   = luparm(23)
      utol1  = parmlu(4)
      utol2  = parmlu(5)
      inform = 0

*     ------------------------------------------------------------------
*     Find  elmax.
*     ------------------------------------------------------------------
      elmax  = zero

      do 120 l = lena + 1 - lenl, lena
         elmax = max( elmax, abs(a(l)) )
  120 continue

*     ------------------------------------------------------------------
*     Find  umax,  and set  w(j) = maximum element in j-th column of  U.
*     ------------------------------------------------------------------
      umax   = zero
      do 220 j = 1, n
         w(j)  = zero
  220 continue

      do 260 k = 1, nrank
         i     = ip(k)
         l1    = locr(i)
         l2    = l1 + lenr(i) - 1

         do 250 l = l1, l2
            j     = indr(l)
            aij   = abs( a(l) )
            w(j)  = max( w(j), aij )
            umax  = max( umax, aij )
  250    continue
  260 continue

*     ------------------------------------------------------------------
*     Negate  w(j)  if the corresponding diagonal of  U  is
*     too small in absolute terms or relative to the other elements
*     in the same column of  U.
*     Also find  dumax  and  dumin,  the extreme diagonals of  U.
*     ------------------------------------------------------------------
      nsing  = 0
      jsing  = 0
      jumin  = 0
      dumax  = zero
      dumin  = 1.0d+30

      do 320 k = 1, n
         j     = iq(k)
         if (k .gt. nrank) go to 310
            i      = ip(k)
            l1     = locr(i)
            diag   = abs( a(l1) )
            dumax  = max( dumax, diag )
            if (dumin .gt. diag) then
                dumin  =   diag
                jumin  =   j
            end if
         if (diag .gt. utol1  .and.  diag .gt. utol2 * w(j)) go to 320

  310       nsing  = nsing + 1
            jsing  = j
            w(j)   = - w(j)
  320 continue

      if (jumin .eq. 0) dumin = zero
      luparm(11) = nsing
      luparm(12) = jsing
      luparm(19) = jumin
      parmlu(11) = elmax
      parmlu(12) = umax
      parmlu(13) = dumax
      parmlu(14) = dumin
      if (nsing .gt. 0) then

*        The matrix has been judged singular.

         inform = 1
         ndefic = n - nrank
         if (lprint .ge. 0)
     $   write(nout, 1100) nrank, ndefic, nsing, jsing, dumax, dumin
      end if

*     Exit.

      luparm(10) = inform
      return

 1100 format(/ ' lu6chk  warning.  The matrix appears to be singular.'
     $       / '     nrank =', i7,       8x, 'rank of U'
     $       / ' n - nrank =', i7,       8x, 'rank deficiency'
     $       / '     nsing =', i7,       8x, 'singularities'
     $       / '     jsing =', i7,       8x, 'last singular column'
     $       / '     dumax =', 1p,e11.2, 4x, 'largest  triangular diag'
     $       / '     dumin =', 1p,e11.2, 4x, 'smallest triangular diag')
*     end of lu6chk
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lu6sol( mode, m, n, v, w,
     $                   lena, luparm, parmlu,
     $                   a, indc, indr, ip, iq,
     $                   lenc, lenr, locc, locr,
     $                   inform )

      implicit           double precision (a-h,o-z)
      integer            luparm(30)
      double precision   parmlu(30), a(lena), v(m), w(n)
      integer*4          indc(lena), indr(lena), ip(m), iq(n)
      integer*4          lenc(n), lenr(m)
      integer            locc(n), locr(m)

*     ------------------------------------------------------------------
*     lu6sol  uses the factorization   A = L*U   as follows...
*
*     mode
*     ----
*      1    v  solves   L*v    = v(input).     w  is not touched.
*      2    v  solves   L(t)*v = v(input).     w  is not touched.
*      3    w  solves   U*w    = v.            v  is not altered.
*      4    v  solves   U(t)*v = w.            w  is destroyed.
*      5    w  solves   A*w    = v.            v  is altered as in 1.
*      6    v  solves   A(t)*v = w.            w  is destroyed.
*
*     if  mode .ge. 3,  v  and  w  must not be the same arrays.
*
*     ip(*), iq(*)      hold row and column numbers in pivotal order.
*     lenc(k)           is the length of the k-th column of initial  L.
*     lenr(i)           is the length of the i-th row of  U.
*     locc(*)           is not used.
*     locr(i)           is the start  of the i-th row of  U.
*
*     U is assumed to be in upper-trapezoidal form (nrank by n).
*     the first entry for each row is the diagonal element
*     (according to the permutations  ip, iq).  It is stored at
*     location  locr(i)  in  a(*), indr(*).
*
*     On exit,  inform = 0  except as follows.
*     if  mode .ge. 3  and if  U  (and hence  A)  is singular, then
*     inform = 1  if there is a nonzero residual in solving the system
*     involving  U.  parmlu(20)  returns the norm of the residual.
*
*     Version of July 1987.
*     9 May 1988: f77 version.
*     ------------------------------------------------------------------

      parameter        ( zero = 0.0d+0 )

      nrank  = luparm(16)
      numl0  = luparm(20)
      lenl0  = luparm(21)
      if (numl0 .eq. 0) lenl0 = 0
      lenl   = luparm(23)
      small  = parmlu(3)
      inform = 0
      nrank1 = nrank + 1
      resid  = zero
      go to (100, 200, 300, 400, 100, 400), mode

*     ==================================================================
*     mode = 1 or 5.    Solve  L * v(new) = v(old).
*     ==================================================================
  100 l1     = lena + 1

      do 120 k = 1, numl0
         len   = lenc(k)
         l     = l1
         l1    = l1 - len
         ipiv  = indr(l1)
         vpiv  = v(ipiv)
         if (abs( vpiv ) .le. small) go to 120

*        ***** The following loop could be coded specially.

         do 110 ldummy = 1, len
            l    = l - 1
            j    = indc(l)
            v(j) = v(j)  +  a(l) * vpiv
  110    continue
  120 continue

  150 l      = lena - lenl0 + 1
      numl   = lenl - lenl0

*     ***** The following loop could be coded specially.

      do 160 ldummy = 1, numl
         l     = l - 1
         i     = indr(l)
         if (abs( v(i) ) .le. small) go to 160
         j     = indc(l)
         v(j)  = v(j)  +  a(l) * v(i)
  160 continue

  190 if (mode .eq. 5) go to 300
      go to 900

*     ==================================================================
*     mode = 2.    Solve  L(transpose) * v(new) = v(old).
*     ==================================================================
  200 l1     = lena - lenl + 1
      l2     = lena - lenl0

*     ***** The following loop could be coded specially.

      do 220 l = l1, l2
         j     = indc(l)
         if (abs( v(j) ) .le. small) go to 220
         i     = indr(l)
         v(i)  = v(i)  +  a(l) * v(j)
  220 continue

      do 280 k = numl0, 1, -1
         len   = lenc(k)
         sum   = zero
         l1    = l2 + 1
         l2    = l2 + len

*        ***** The following loop could be coded specially.

         do 270 l = l1, l2
            j     = indc(l)
            sum   = sum  +  a(l) * v(j)
  270    continue

         ipiv    = indr(l1)
         v(ipiv) = v(ipiv) + sum
  280 continue

      go to 900

*     ==================================================================
*     mode = 3.    Solve   U * w  =  v.
*     ==================================================================

*     Find the first nonzero in  v(1) ... v(nrank),  counting backwards.

  300 do 310 klast = nrank, 1, -1
         i      = ip(klast)
         if (abs( v(i) ) .gt. small) go to 320
  310 continue

  320 do 330 k = klast + 1, n
         j     = iq(k)
         w(j)  = zero
  330 continue

*     Do the back-substitution, using rows  1  to  klast  of  U.

  340 do 380 k  = klast, 1, -1
         i      = ip(k)
         t      = v(i)
         l1     = locr(i)
         l2     = l1 + 1
         l3     = l1 + lenr(i) - 1

*        ***** The following loop could be coded specially.

         do 350 l = l2, l3
            j     = indr(l)
            t     = t  -  a(l) * w(j)
  350    continue

         j      = iq(k)
         if (abs( t ) .le. small) then
            w(j)   = zero
         else
            w(j)   = t / a(l1)
         end if
  380 continue

*     Compute residual for overdetermined systems.

      do 390 k = nrank1, m
         i     = ip(k)
         resid = resid  +  abs( v(i) )
  390 continue

      go to 900

*     ==================================================================
*     mode = 4 or 6.    Solve   U(transpose) * v  =  w.
*     ==================================================================
  400 do 410 k = nrank1, m
         i     = ip(k)
         v(i)  = zero
  410 continue

*     Do the forward-substitution, skipping columns of  U(transpose)
*     when the associated element of  w(*)  is negligible.

      do 480 k = 1, nrank
         i      = ip(k)
         j      = iq(k)
         t      = w(j)
         if (abs( t ) .le. small) then
            v(i) = zero
            go to 480
         end if

         l1     = locr(i)
         t      = t / a(l1)
         v(i)   = t
         l2     = l1 + lenr(i) - 1
         l1     = l1 + 1

*        ***** The following loop could be coded specially.

         do 450 l = l1, l2
            j     = indr(l)
            w(j)  = w(j)  -  t * a(l)
  450    continue
  480 continue

*     Compute residual for overdetermined systems.

      do 490 k = nrank1, n
         j     = iq(k)
         resid = resid  +  abs( w(j) )
  490 continue

      if (mode .eq. 6) go to 200

*     Exit.

  900 if (resid .gt. zero) inform = 1
      luparm(10) = inform
      parmlu(20) = resid
      return

*     end of lu6sol
      end

************************************************************************
*
*     file  lu7a    fortran
*
*     lu7add   lu7cyc   lu7elm   lu7for   lu7rnk   lu7zap
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lu7add( m, n, jadd, v,
     $                   lena, luparm, parmlu,
     $                   lenl, lenu, lrow, nrank,
     $                   a, indr, ip, lenr, locr,
     $                   inform, klast, vnorm )

      implicit           double precision (a-h,o-z)
      integer            luparm(30)
      double precision   parmlu(30), a(lena), v(m)
      integer*4          indr(lena), ip(m), lenr(m)
      integer            locr(m)

*     ------------------------------------------------------------------
*     lu7add  inserts the first nrank elements of the vector v(*)
*     as column  jadd  of  U.  We assume that  U  does not yet have any
*     entries in this column.
*     Elements no larger than  parmlu(3)  are treated as zero.
*     klast  will be set so that the last row to be affected
*     (in pivotal order) is row  ip(klast).
*
*     09 May 1988: First f77 version.
*     ------------------------------------------------------------------

      parameter        ( zero = 0.0d+0 )

      small  = parmlu(3)
      vnorm  = zero
      klast  = 0

      do 200 k  = 1, nrank
         i      = ip(k)
         if (abs( v(i) ) .le. small) go to 200
         klast  = k
         vnorm  = vnorm  +  abs( v(i) )
         leni   = lenr(i)

*        Compress row file if necessary.

         minfre = leni + 1
         nfree  = lena - lenl - lrow
         if (nfree .lt. minfre) then
            call lu1rec( m, .true., luparm, lrow, lena,
     $                   a, indr, lenr, locr )
            nfree  = lena - lenl - lrow
            if (nfree .lt. minfre) go to 970
         end if

*        Move row  i  to the end of the row file,
*        unless it is already there.
*        No need to move if there is a gap already.

         if (leni .eq. 0) locr(i) = lrow + 1
         lr1    = locr(i)
         lr2    = lr1 + leni - 1
         if (lr2    .eq.   lrow) go to 150
         if (indr(lr2+1) .eq. 0) go to 180
         locr(i) = lrow + 1

         do 140 l = lr1, lr2
            lrow       = lrow + 1
            a(lrow)    = a(l)
            j          = indr(l)
            indr(l)    = 0
            indr(lrow) = j
  140    continue

  150    lr2     = lrow
         lrow    = lrow + 1

*        Add the element of  v.

  180    lr2       = lr2 + 1
         a(lr2)    = v(i)
         indr(lr2) = jadd
         lenr(i)   = leni + 1
         lenu      = lenu + 1
  200 continue

*     Normal exit.

      inform = 0
      go to 990

*     Not enough storage.

  970 inform = 7

  990 return

*     end of lu7add
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lu7cyc( kfirst, klast, ip )

      integer*4          ip(klast)

*     ------------------------------------------------------------------
*     lu7cyc performs a cyclic permutation on the row or column ordering
*     stored in ip, moving entry kfirst down to klast.
*     If kfirst .ge. klast, lu7cyc should not be called.
*     Sometimes klast = 0 and nothing should happen.
*
*     09 May 1988: First f77 version.
*     ------------------------------------------------------------------

      if (kfirst .lt. klast) then
         ifirst = ip(kfirst)

         do 100 k = kfirst, klast - 1
            ip(k) = ip(k + 1)
  100    continue

         ip(klast) = ifirst
      end if

*     end of lu7cyc
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lu7elm( m, n, jelm, v,
     $                   lena, luparm, parmlu,
     $                   lenl, lenu, lrow, nrank,
     $                   a, indc, indr, ip, iq, lenr, locc, locr,
     $                   inform, diag )

      implicit           double precision (a-h,o-z)
      integer            luparm(30)
      double precision   parmlu(30), a(lena), v(m)
      integer*4          indc(lena), indr(lena), ip(m), iq(n), lenr(m)
      integer            locc(n), locr(m)

*     ------------------------------------------------------------------
*     lu7elm  eliminates the subdiagonal elements of a vector  v(*),
*     where  L*v = y  for some vector y.
*     If  jelm > 0,  y  has just become column  jelm  of the matrix  A.
*     lu7elm  should not be called unless  m  is greater than  nrank.
*
*     09 May 1988: First f77 version.
*                  No longer calls lu7for at end.  lu8rpc, lu8mod do so.
*     ------------------------------------------------------------------

      parameter        ( zero = 0.0d+0 )

      small  = parmlu(3)
      nrank1 = nrank + 1
      diag   = zero

*     Compress row file if necessary.

      minfre = m - nrank
      nfree  = lena - lenl - lrow
      if (nfree .ge. minfre) go to 100
      call lu1rec( m, .true., luparm, lrow, lena, a, indr, lenr, locr )
      nfree  = lena - lenl - lrow
      if (nfree .lt. minfre) go to 970

*     Pack the subdiagonals of  v  into  L,  and find the largest.

  100 vmax   = zero
      kmax   = 0
      l      = lena - lenl + 1

      do 200 k = nrank1, m
         i       = ip(k)
         vi      = abs( v(i) )
         if (vi .le. small) go to 200
         l       = l - 1
         a(l)    = v(i)
         indc(l) = i
         if (vmax .ge. vi ) go to 200
         vmax    = vi
         kmax    = k
         lmax    = l
  200 continue

      if (kmax .eq. 0) go to 900

*     ------------------------------------------------------------------
*     Remove  vmax  by overwriting it with the last packed  v(i).
*     Then set the multipliers in  L  for the other elements.
*     ------------------------------------------------------------------

      imax       = ip(kmax)
      vmax       = a(lmax)
      a(lmax)    = a(l)
      indc(lmax) = indc(l)
      l1         = l + 1
      l2         = lena - lenl
      lenl       = lenl + (l2 - l)

      do 300 l = l1, l2
         a(l)    = - a(l) / vmax
         indr(l) =   imax
  300 continue

*     Move the row containing vmax to pivotal position nrank + 1.

      ip(kmax  ) = ip(nrank1)
      ip(nrank1) = imax
      diag       = vmax

*     ------------------------------------------------------------------
*     If jelm is positive, insert  vmax  into a new row of  U.
*     This is now the only subdiagonal element.
*     ------------------------------------------------------------------

      if (jelm .gt. 0) then
         lrow       = lrow + 1
         locr(imax) = lrow
         lenr(imax) = 1
         a(lrow)    = vmax
         indr(lrow) = jelm
      end if

      inform = 1
      go to 990

*     No elements to eliminate.

  900 inform = 0
      go to 990

*     Not enough storage.

  970 inform = 7

  990 return

*     end of lu7elm
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lu7for( m, n, kfirst, klast,
     $                   lena, luparm, parmlu,
     $                   lenl, lenu, lrow,
     $                   a, indc, indr, ip, iq, lenr, locc, locr,
     $                   inform, diag )

      implicit           double precision (a-h,o-z)
      integer            luparm(30)
      double precision   parmlu(30), a(lena)
      integer*4          indc(lena), indr(lena), ip(m), iq(n), lenr(m)
      integer            locc(n), locr(m)

*     ------------------------------------------------------------------
*     lu7for  (forward sweep) updates the LU factorization  A = L*U
*     when row  iw = ip(klast)  of  U  is eliminated by a forward
*     sweep of stabilized row operations, leaving  ip * U * iq  upper
*     triangular.
*
*     The row permutation  ip  is updated to preserve stability and/or
*     sparsity.  The column permutation  iq  is not altered.
*
*     kfirst  is such that row  ip(kfirst)  is the first row involved
*     in eliminating row  iw.  (Hence,  kfirst  marks the first nonzero
*     in row  iw  in pivotal order.)  If  kfirst  is unknown it may be
*     input as  1.
*
*     klast   is such that row  ip(klast)  is the row being eliminated.
*     klast   is not altered.
*
*     lu7for  should be called only if  kfirst .le. klast.
*     If  kfirst = klast,  there are no nonzeros to eliminate, but the
*     diagonal element of row  ip(klast)  may need to be moved to the
*     front of the row.
*
*     On entry,  locc(*)  must be zero.
*
*     On exit:
*     inform = 0  if row iw has a nonzero diagonal (could be small).
*     inform = 1  if row iw has no diagonal.
*     inform = 7  if there is not enough storage to finish the update.
*
*     On a successful exit (inform le 1),  locc(*)  will again be zero.
*
*        Jan 1985: Final f66 version.
*     09 May 1988: First f77 version.
*     ------------------------------------------------------------------

      parameter        ( zero = 0.0d+0 )

      double precision   lmax
      logical            swappd

      lmax   = parmlu(2)
      small  = parmlu(3)
      uspace = parmlu(6)
      kbegin = kfirst
      swappd = .false.

*     We come back here from below if a row interchange is performed.

  100 iw     = ip(klast)
      lenw   = lenr(iw)
      if (lenw   .eq.   0  ) go to 910
      lw1    = locr(iw)
      lw2    = lw1 + lenw - 1
      jfirst = iq(kbegin)
      if (kbegin .ge. klast) go to 700

*     Make sure there is room at the end of the row file
*     in case row  iw  is moved there and fills in completely.

      minfre = n + 1
      nfree  = lena - lenl - lrow
      if (nfree .lt. minfre) then
         call lu1rec( m, .true., luparm, lrow, lena,
     $                a, indr, lenr, locr )
         lw1    = locr(iw)
         lw2    = lw1 + lenw - 1
         nfree  = lena - lenl - lrow
         if (nfree .lt. minfre) go to 970
      end if

*     Set markers on row  iw.

      do 120 l = lw1, lw2
         j       = indr(l)
         locc(j) = l
  120 continue


*     ==================================================================
*     Main elimination loop.
*     ==================================================================
      kstart = kbegin
      kstop  = min( klast, n )

      do 500 k  = kstart, kstop
         jfirst = iq(k)
         lfirst = locc(jfirst)
         if (lfirst .eq. 0) go to 490

*        Row  iw  has its first element in column  jfirst.

         wj     = a(lfirst)
         if (k .eq. klast) go to 490

*        ---------------------------------------------------------------
*        We are about to use the first element of row  iv
*               to eliminate the first element of row  iw.
*        However, we may wish to interchange the rows instead,
*        to preserve stability and/or sparsity.
*        ---------------------------------------------------------------
         iv     = ip(k)
         lenv   = lenr(iv)
         lv1    = locr(iv)
         vj     = zero
         if (lenv      .eq.   0   ) go to 150
         if (indr(lv1) .ne. jfirst) go to 150
         vj     = a(lv1)
         if (            swappd               ) go to 200
         if (lmax * abs( wj )  .lt.  abs( vj )) go to 200
         if (lmax * abs( vj )  .lt.  abs( wj )) go to 150
         if (            lenv  .le.  lenw     ) go to 200

*        ---------------------------------------------------------------
*        Interchange rows  iv  and  iw.
*        ---------------------------------------------------------------
  150    ip(klast) = iv
         ip(k)     = iw
         kbegin    = k
         swappd    = .true.
         go to 600

*        ---------------------------------------------------------------
*        Delete the eliminated element from row  iw
*        by overwriting it with the last element.
*        ---------------------------------------------------------------
  200    a(lfirst)    = a(lw2)
         jlast        = indr(lw2)
         indr(lfirst) = jlast
         indr(lw2)    = 0
         locc(jlast)  = lfirst
         locc(jfirst) = 0
         lenw         = lenw - 1
         lenu         = lenu - 1
         if (lrow .eq. lw2) lrow = lrow - 1
         lw2          = lw2  - 1

*        ---------------------------------------------------------------
*        Form the multiplier and store it in the  L  file.
*        ---------------------------------------------------------------
         if (abs( wj ) .le. small) go to 490
         amult   = - wj / vj
         l       = lena - lenl
         a(l)    = amult
         indr(l) = iv
         indc(l) = iw
         lenl    = lenl + 1

*        ---------------------------------------------------------------
*        Add the appropriate multiple of row  iv  to row  iw.
*        We use two different inner loops.  The first one is for the
*        case where row  iw  is not at the end of storage.
*        ---------------------------------------------------------------
         if (lenv .eq. 1) go to 490
         lv2    = lv1 + 1
         lv3    = lv1 + lenv - 1
         if (lw2 .eq. lrow) go to 400

*        ...............................................................
*        This inner loop will be interrupted only if
*        fill-in occurs enough to bump into the next row.
*        ...............................................................
         do 350 lv = lv2, lv3
            jv     = indr(lv)
            lw     = locc(jv)
            if (lw .gt. 0) then

*              No fill-in.

               a(lw)  = a(lw)  +  amult * a(lv)
               if (abs( a(lw) ) .le. small) then

*                 Delete small computed element.

                  a(lw)     = a(lw2)
                  j         = indr(lw2)
                  indr(lw)  = j
                  indr(lw2) = 0
                  locc(j)   = lw
                  locc(jv)  = 0
                  lenu      = lenu - 1
                  lenw      = lenw - 1
                  lw2       = lw2  - 1
               end if
            else

*              Row  iw  doesn't have an element in column  jv  yet
*              so there is a fill-in.

               if (indr(lw2+1) .ne. 0) go to 360
               lenu      = lenu + 1
               lenw      = lenw + 1
               lw2       = lw2  + 1
               a(lw2)    = amult * a(lv)
               indr(lw2) = jv
               locc(jv)  = lw2
            end if
  350    continue

         go to 490

*        Fill-in interrupted the previous loop.
*        Move row  iw  to the end of the row file.

  360    lv2      = lv
         locr(iw) = lrow + 1

         do 370 l = lw1, lw2
            lrow       = lrow + 1
            a(lrow)    = a(l)
            j          = indr(l)
            indr(l)    = 0
            indr(lrow) = j
            locc(j)    = lrow
  370    continue

         lw1    = locr(iw)
         lw2    = lrow

*        ...............................................................
*        Inner loop with row  iw  at the end of storage.
*        ...............................................................
  400    do 450 lv = lv2, lv3
            jv     = indr(lv)
            lw     = locc(jv)
            if (lw .gt. 0) then

*              No fill-in.

               a(lw)  = a(lw)  +  amult * a(lv)
               if (abs( a(lw) ) .le. small) then

*                 Delete small computed element.

                  a(lw)     = a(lw2)
                  j         = indr(lw2)
                  indr(lw)  = j
                  indr(lw2) = 0
                  locc(j)   = lw
                  locc(jv)  = 0
                  lenu      = lenu - 1
                  lenw      = lenw - 1
                  lw2       = lw2  - 1
               end if
            else

*              Row  iw  doesn't have an element in column  jv  yet
*              so there is a fill-in.

               lenu      = lenu + 1
               lenw      = lenw + 1
               lw2       = lw2  + 1
               a(lw2)    = amult * a(lv)
               indr(lw2) = jv
               locc(jv)  = lw2
            end if
  450    continue

         lrow   = lw2

*        The  k-th  element of row  iw  has been processed.
*        Reset  swappd  before looking at the next element.

  490    swappd = .false.
  500 continue

*     ==================================================================
*     End of main elimination loop.
*     ==================================================================

*     Cancel markers on row  iw.

  600 lenr(iw) = lenw
      if (lenw .eq. 0) go to 910
      do 620 l = lw1, lw2
         j       = indr(l)
         locc(j) = 0
  620 continue

*     Move the diagonal element to the front of row  iw.
*     At this stage,  lenw gt 0  and  klast le n.

  700 do 720 l = lw1, lw2
         ldiag = l
         if (indr(l) .eq. jfirst) go to 730
  720 continue
      go to 910

  730 diag        = a(ldiag)
      a(ldiag)    = a(lw1)
      a(lw1)      = diag
      indr(ldiag) = indr(lw1)
      indr(lw1)   = jfirst

*     If an interchange is needed, repeat from the beginning with the
*     new row  iw,  knowing that the opposite interchange cannot occur.

      if (swappd) go to 100
      inform = 0
      go to 950

*     Singular.

  910 diag   = zero
      inform = 1

*     Force a compression if the file for  U  is much longer than the
*     no. of nonzeros in  U  (i.e. if  lrow  is much bigger than  lenu).
*     This should prevent memory fragmentation when there is far more
*     memory than necessary  (i.e. when  lena  is huge).

  950 limit  = uspace * lenu + m + n + 1000
      if (lrow .gt. limit) then
         call lu1rec( m, .true., luparm, lrow, lena,
     $                a, indr, lenr, locr )
      end if
      go to 990

*     Not enough storage.

  970 inform = 7

*     Exit.

  990 return

*     end of lu7for
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lu7rnk( m, n, jsing,
     $                   lena, luparm, parmlu,
     $                   lenl, lenu, lrow, nrank,
     $                   a, indc, indr, ip, iq, lenr, locc, locr,
     $                   inform, diag )

      implicit           double precision (a-h,o-z)
      integer            luparm(30)
      double precision   parmlu(30), a(lena)
      integer*4          indc(lena), indr(lena), ip(m), iq(n), lenr(m)
      integer            locc(n), locr(m)

*     ------------------------------------------------------------------
*     lu7rnk (check rank) assumes U is currently nrank by n
*     and determines if row nrank contains an acceptable pivot.
*     If not, the row is deleted and nrank is decreased by 1.
*
*     jsing is an input parameter (not altered).  If jsing is positive,
*     column jsing has already been judged dependent.  A substitute
*     (if any) must be some other column.
*
*     -- Jul 1987: First version.
*     09 May 1988: First f77 version.
*     ------------------------------------------------------------------

      parameter        ( zero = 0.0d+0 )

      utol1    = parmlu(4)
      diag     = zero

*     Find umax, the largest element in row nrank.

      iw       = ip(nrank)
      lenw     = lenr(iw)
      if (lenw .eq. 0) go to 400
      l1       = locr(iw)
      l2       = l1 + lenw - 1
      umax     = zero
      lmax     = l1

      do 100 l = l1, l2
         if (umax .lt. abs( a(l) )) then
             umax   =  abs( a(l) )
             lmax   =  l
         end if
  100 continue

*     Find which column that guy is in (in pivotal order).
*     Interchange him with column nrank, then move him to be
*     the new diagonal at the front of row nrank.

      diag   = a(lmax)
      jmax   = indr(lmax)

      do 300 kmax = nrank, n
         if (iq(kmax) .eq. jmax) go to 320
  300 continue

  320 iq(kmax)  = iq(nrank)
      iq(nrank) = jmax
      a(lmax)   = a(l1)
      a(l1)     = diag
      indr(lmax)= indr(l1)
      indr(l1)  = jmax

*     See if the new diagonal is big enough.

      if (umax .le. utol1) go to 400
      if (jmax .eq. jsing) go to 400

*     ------------------------------------------------------------------
*     The rank stays the same.
*     ------------------------------------------------------------------
      inform = 0
      return

*     ------------------------------------------------------------------
*     The rank decreases by one.
*     ------------------------------------------------------------------
  400 inform = -1
      nrank  = nrank - 1
      if (lenw .gt. 0) then

*        Delete row nrank from U.

         lenu     = lenu - lenw
         lenr(iw) = 0
         do 420 l = l1, l2
            indr(l) = 0
  420    continue

         if (l2 .eq. lrow) then

*           This row was at the end of the data structure.
*           We have to reset lrow.
*           Preceding rows might already have been deleted, so we
*           have to be prepared to go all the way back to 1.

            do 450 l = 1, l2
               if (indr(lrow) .gt. 0) go to 900
               lrow  = lrow - 1
  450       continue
         end if
      end if

  900 return

*     end of lu7rnk
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lu7zap( m, n, jzap, kzap,
     $                   lena, lenu, lrow, nrank,
     $                   a, indr, ip, iq, lenr, locr )

      implicit           double precision (a-h,o-z)
      double precision   a(lena)
      integer*4          indr(lena), ip(m), iq(n), lenr(m)
      integer            locr(m)

*     ------------------------------------------------------------------
*     lu7zap  eliminates all nonzeros in column  jzap  of  U.
*     It also sets  kzap  to the position of  jzap  in pivotal order.
*     Thus, on exit we have  iq(kzap) = jzap.
*
*     -- Jul 1987: nrank added.
*     10 May 1988: First f77 version.
*     ------------------------------------------------------------------

      do 100 k  = 1, nrank
         i      = ip(k)
         leni   = lenr(i)
         if (leni .eq. 0) go to 90
         lr1    = locr(i)
         lr2    = lr1 + leni - 1
         do 50 l = lr1, lr2
            if (indr(l) .eq. jzap) go to 60
   50    continue
         go to 90

*        Delete the old element.

   60    a(l)      = a(lr2)
         indr(l)   = indr(lr2)
         indr(lr2) = 0
         lenr(i)   = leni - 1
         lenu      = lenu - 1

*        Stop if we know there are no more rows containing  jzap.

   90    kzap   = k
         if (iq(k) .eq. jzap) go to 800
  100 continue

*     nrank must be smaller than n because we haven't found kzap yet.

      do 200 k = nrank+1, n
         kzap  = k
         if (iq(k) .eq. jzap) go to 800
  200 continue

*     See if we zapped the last element in the file.

  800 if (lrow .gt. 0) then
         if (indr(lrow) .eq. 0) lrow = lrow - 1
      end if

*     end of lu7zap
      end

************************************************************************
*
*     file  lu8a    fortran
*
*     lu8rpc
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine lu8rpc( mode1, mode2, m, n, jrep, v, w,
     $                   lena, luparm, parmlu,
     $                   a, indc, indr, ip, iq,
     $                   lenc, lenr, locc, locr,
     $                   inform, diag, vnorm )

      implicit           double precision(a-h,o-z)
      integer            luparm(30)
      double precision   parmlu(30), a(lena), v(m), w(n)
      integer*4          indc(lena), indr(lena), ip(m), iq(n)
      integer*4          lenc(n), lenr(m)
      integer            locc(n), locr(m)

*     ------------------------------------------------------------------
*     lu8rpc  updates the LU factorization  A = L*U  when column  jrep
*     is replaced by some vector  a(new).
*
*     lu8rpc  is an implementation of the Bartels-Golub update,
*     designed for the case where A is rectangular and/or singular.
*     L is a product of stabilized eliminations (m x m, nonsingular).
*     P U Q is upper trapezoidal (m x n, rank nrank).
*
*     If  mode1 = 0,  the old column is taken to be zero
*                     (so it does not have to be removed from  U).
*
*     If  mode1 = 1,  the old column need not have been zero.
*
*     If  mode2 = 0,  the new column is taken to be zero.
*                     v(*)  is not used or altered.
*
*     If  mode2 = 1,  v(*)  must contain the new column  a(new).
*                     On exit,  v(*)  will satisfy  L*v = a(new).
*
*     If  mode2 = 2,  v(*)  must satisfy  L*v = a(new).
*
*     The array  w(*)  is not used or altered.
*
*     On entry, all elements of  locc  are assumed to be zero.
*     On a successful exit (inform ne 7), this will again be true.
*
*     On exit:
*     inform = -1  if the rank of U decreased by 1.
*     inform =  0  if the rank of U stayed the same.
*     inform =  1  if the rank of U increased by 1.
*     inform =  7  if the update was not completed (lack of storage).
*     inform =  8  if jrep is not between 1 and n.
*
*     -- Jan 1985: Original F66 version.
*     -- Jul 1987: Modified to maintain U in trapezoidal form.
*     10 May 1988: First f77 version.
*     ------------------------------------------------------------------

      logical            singlr
      parameter        ( zero = 0.0d+0 )

      nout   = luparm(1)
      lprint = luparm(2)
      nrank  = luparm(16)
      lenl   = luparm(23)
      lenu   = luparm(24)
      lrow   = luparm(25)
      utol1  = parmlu(4)
      utol2  = parmlu(5)
      nrank0 = nrank
      diag   = zero
      vnorm  = zero
      if (jrep .lt. 1) go to 980
      if (jrep .gt. n) go to 980

*     ------------------------------------------------------------------
*     If mode1 = 0, there are no elements to be removed from  U
*     but we still have to set  krep  (using a backward loop).
*     Otherwise, use lu7zap to remove column  jrep  from  U
*     and set  krep  at the same time.
*     ------------------------------------------------------------------
      if (mode1 .eq. 0) then
         krep   = n + 1

   10    krep   = krep - 1
         if (iq(krep) .ne. jrep) go to 10
      else
         call lu7zap( m, n, jrep, krep,
     $                lena, lenu, lrow, nrank,
     $                a, indr, ip, iq, lenr, locr )
      end if

*     ------------------------------------------------------------------
*     Insert a new column of u and find klast.
*     ------------------------------------------------------------------

      if (mode2 .eq. 0) then
         klast  = 0
      else
         if (mode2 .eq. 1) then

*           Transform v = a(new) to satisfy  L*v = a(new).

            call lu6sol( 1, m, n, v, w, lena, luparm, parmlu,
     $                   a, indc, indr, ip, iq,
     $                   lenc, lenr, locc, locr, inform )
         end if

*        Insert into  U  any nonzeros in the top of  v.
*        row  ip(klast)  will contain the last nonzero in pivotal order.
*        Note that  klast  will be in the range  ( 0, nrank ).

         call lu7add( m, n, jrep, v,
     $                lena, luparm, parmlu,
     $                lenl, lenu, lrow, nrank,
     $                a, indr, ip, lenr, locr,
     $                inform, klast, vnorm )
         if (inform .eq. 7) go to 970
      end if

*     ------------------------------------------------------------------
*     In general, the new column causes U to look like this:
*
*                 krep        n                 krep  n
*
*                ....a.........          ..........a...
*                 .  a        .           .        a  .
*                  . a        .            .       a  .
*                   .a        .             .      a  .
*        P U Q =     a        .    or        .     a  .
*                    b.       .               .    a  .
*                    b .      .                .   a  .
*                    b  .     .                 .  a  .
*                    b   ......                  ..a...  nrank
*                    c                             c
*                    c                             c
*                    c                             c     m
*
*     klast points to the last nonzero "a" or "b".
*     klast = 0 means all "a" and "b" entries are zero.
*     ------------------------------------------------------------------

      if (mode2 .eq. 0) then
         if (krep .gt. nrank) go to 900
      else if (nrank .lt. m) then

*        Eliminate any "c"s (in either case).
*        Row nrank + 1 may end up containing one nonzero.

         call lu7elm( m, n, jrep, v,
     $                lena, luparm, parmlu,
     $                lenl, lenu, lrow, nrank,
     $                a, indc, indr, ip, iq, lenr, locc, locr,
     $                inform, diag )
         if (inform .eq. 7) go to 970

         if (inform .eq. 1) then

*           The nonzero is apparently significant.
*           Increase nrank by 1 and make klast point to the bottom.

            nrank = nrank + 1
            klast = nrank
         end if
      end if

      if (nrank .lt. n) then

*        The column rank is low.
*
*        In the first case, we want the new column to end up in
*        position nrank, so the trapezoidal columns will have a chance
*        later on (in lu7rnk) pivot in that position.
*
*        Otherwise the new column is not part of the triangle.  We
*        swap it into position nrank so we can judge it for singularity.
*        lu7rnk might choose some other trapezoidal column later.

         if (krep .lt. nrank) then
            klast     = nrank
         else
            iq(krep ) = iq(nrank)
            iq(nrank) = jrep
            krep      = nrank
         end if
      end if

*     ------------------------------------------------------------------
*     If krep .lt. klast, there are some "b"s to eliminate:
*
*                  krep
*
*                ....a.........
*                 .  a        .
*                  . a        .
*                   .a        .
*        P U Q =     a        .  krep
*                    b.       .
*                    b .      .
*                    b  .     .
*                    b   ......  nrank
*
*     If krep .eq. klast, there are no "b"s, but the last "a" still
*     has to be moved to the front of row krep (by lu7for).
*     ------------------------------------------------------------------

      if (krep .le. klast) then

*        Perform a cyclic permutation on the current pivotal order,
*        and eliminate the resulting row spike.  krep becomes klast.
*        The final diagonal (if any) will be correctly positioned at
*        the front of the new krep-th row.  nrank stays the same.

         call lu7cyc( krep, klast, ip )
         call lu7cyc( krep, klast, iq )

         call lu7for( m, n, krep, klast,
     $                lena, luparm, parmlu,
     $                lenl, lenu, lrow,
     $                a, indc, indr, ip, iq, lenr, locc, locr,
     $                inform, diag )
         if (inform .eq. 7) go to 970
         krep   = klast
      end if

*     ------------------------------------------------------------------
*     Test for singularity in column krep (where krep .le. nrank).
*     ------------------------------------------------------------------

      diag   = zero
      iw     = ip(krep)
      singlr = lenr(iw) .eq. 0

      if (.not. singlr) then
         l1     = locr(iw)
         j1     = indr(l1)
         singlr = j1 .ne. jrep

         if (.not. singlr) then
            diag   = a(l1)
            singlr = abs( diag ) .le. utol1          .or.
     $               abs( diag ) .le. utol2 * vnorm
         end if
      end if

      if ( singlr  .and.  krep .lt. nrank ) then

*        Perform cyclic permutations to move column jrep to the end.
*        Move the corresponding row to position nrank
*        then eliminate the resulting row spike.

         call lu7cyc( krep, nrank, ip )
         call lu7cyc( krep, n    , iq )

         call lu7for( m, n, krep, nrank,
     $                lena, luparm, parmlu,
     $                lenl, lenu, lrow,
     $                a, indc, indr, ip, iq, lenr, locc, locr,
     $                inform, diag )
         if (inform .eq. 7) go to 970
      end if

*     Find the best column to be in position nrank.
*     If singlr, it can't be the new column, jrep.
*     If nothing satisfactory exists, nrank will be decreased.

      if ( singlr  .or.  nrank .lt. n ) then
         jsing  = 0
         if ( singlr ) jsing = jrep

         call lu7rnk( m, n, jsing,
     $                lena, luparm, parmlu,
     $                lenl, lenu, lrow, nrank,
     $                a, indc, indr, ip, iq, lenr, locc, locr,
     $                inform, diag )
      end if

*     ------------------------------------------------------------------
*     Set inform for exit.
*     ------------------------------------------------------------------

  900 if (nrank .eq. nrank0) then
         inform =  0
      else if (nrank .lt. nrank0) then
         inform = -1
         if (nrank0 .eq. n) then
            if (lprint .ge. 0) write(nout, 1100) jrep, diag
         end if
      else
         inform =  1
      end if
      go to 990

*     Not enough storage.

  970 inform = 7
      if (lprint .ge. 0) write(nout, 1700) lena
      go to 990

*     jrep  is out of range.

  980 inform = 8
      if (lprint .ge. 0) write(nout, 1800) m, n, jrep

*     Exit.

  990 luparm(10) = inform
      luparm(15) = luparm(15) + 1
      luparm(16) = nrank
      luparm(23) = lenl
      luparm(24) = lenu
      luparm(25) = lrow
      return

 1100 format(/ ' lu8rpc  warning.  Singularity after replacing column.',
     $       '    jrep =', i8, '    diag =', 1p, e12.2 )
 1700 format(/ ' lu8rpc  error...  Insufficient storage.',
     $         '    lena =', i8)
 1800 format(/ ' lu8rpc  error...  jrep  is out of range.',
     $         '    m =', i8, '    n =', i8, '    jrep =', i8)

*     end of lu8rpc
      end
