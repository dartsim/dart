*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
*     File  sn35mps.f
*
*     MPSinp
*     s3dflt   s3getp   s3hash   s3inpt   s3Mem    s3mps
*     s3mpsa   s3mpsb   s3mpsc   s3read
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine MPSinp( iMPS,
     $                   maxm, maxn, maxne,
     $                   nnCon, nnJac, nnObj, 
     $                   m, n, ne,
     $                   iObj, ObjAdd, PrbNms,
     $                   a, ha, ka, bl, bu, Names,
     $                   hs, x, pi,
     $                   inform, nS,
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*8        PrbNms(5)
      character*8        Names(maxm+maxn)
      integer*4          ha(maxne), hs(maxm+maxn)
      integer            ka(maxn+1)
      double precision   a(maxne), bl(maxm+maxn), bu(maxm+maxn)
      double precision   x(maxn), pi(maxm)

      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ------------------------------------------------------------------
*     MPSinp  inputs constraint data for a linear or nonlinear program
*     in MPS format, consisting of NAME, ROWS, COLUMNS, RHS, RANGES and
*     BOUNDS sections in that order.  The RANGES and BOUNDS sections are
*     optional.
*
*     In the LP case, MPS format defines a set of constraints of the
*     form
*              l <= x <= u,      b1 <=  Ax  <= b2,
*     where l and u are specified by the BOUNDS section, and b1 and b2
*     are defined somewhat indirectly by the ROWS, RHS and RANGES
*     sections.  snmps converts these constraints into the equivalent
*     form
*              Ax - s = 0,       bl <= ( x ) <= bu,
*                                      ( s )
*     where s is a set of slack variables.  This is the way SNOPT deals
*     with the data.  The first n components of bl and bu are the same
*     as l and u.  The last m components are b1 and b2.
*
*     MPS format gives 8-character names to the rows and columns of A.
*     One of the rows of A may be regarded as a linear objective row.
*     This will be row iObj, where iObj = 0 means there is no such row.
*
*     The data defines a linear program if nnCon = nnJac = nnObj = 0.
*     The nonlinear case is the same except for a few details.
*     1. If nnCon = nnJac = 0 but nnObj > 0, the first nnObj columns
*        are associated with a nonlinear objective function.
*     2. If nnCon > 0, then nnJac > 0 and nnObj may be zero or positive.
*        The first nnCon rows and the first nnJac columns are associated
*        with a set of nonlinear constraints.
*     3. Let nnL = max( nnJac, nnObj ).  The first nnL columns correspond
*        to "nonlinear variables".
*     4. If an objective row is specified (iObj > 0), then it must be
*        such that iObj > nnCon.
*     5. "Small" elements (below the Aij tolerance) are ignored only if
*        they lie outside the nnCon by nnJac Jacobian, i.e. outside
*        the top-left corner of A.
*     6. No warning is given if some of the first nnL columns are empty.
*
*
*     ON ENTRY
*     ========
*     iMPS   is the unit containing the MPS file.  On some systems, it
*            may be necessary to open file iMPS before calling snmps.
*
*     maxm   is an overestimate of the number of rows in the ROWS
*            section of the MPS file.
*
*     maxn   is an overestimate of the number of columns in the COLUMNS
*            section of the MPS file.
*
*     maxne  is an overestimate of the number of elements (matrix
*            coefficients) in the COLUMNS section.
*
*     nnCon  is the no. of nonlinear constraints in the problem.
*            These must be the FIRST rows in the ROWS section.
*
*     nnJac  is the no. of nonlinear Jacobian variables in the problem.
*            These must be the FIRST columns in the COLUMNS section.
*
*     nnObj  is the no. of nonlinear objective variables in the problem.
*            These must be the FIRST columns in the COLUMNS section,
*            overlapping where necessary with the Jacobian variables.
*
*     PrbNms is an array of five 8-character names.
*            PrbNms(1) need not be specified... it will be changed to
*            the name on the NAME card of the MPS file.
*            PrbNms(2) is the name of the objective row to be selected
*            from the ROWS section, or blank if snmps should select
*            the first type N row encountered.
*            Similarly,
*            PrbNms(3), PrbNms(4) and PrbNms(5) are the names of the
*            RHS, RANGES and BOUNDS to be selected from the
*            RHS, RANGES and BOUNDS sections respectively, or blank
*            if snmps should select the first ones encountered.
*
*     iw(*)  is a workspace array of length leniw.  It is needed to
*            hold the row-name hash table and a few other things.
*
*     leniw  is the length of iw(*).  It should be at least 4*maxm.
*
*     rw(*)  is a workspace array of length lenrw.  It is needed to
*            hold the row-name hash table and a few other things.
*
*     lenrw  is the length of rw(*).  It should be at least 4*maxm.
*
*
*     ON EXIT
*     =======
*     m      is the number of rows in the ROWS section.
*
*     n      is the number of columns in the COLUMNS section.
*
*     ne     is the number of matrix coefficients in COLUMNS section.
*
*     iObj   is the row number of the specified objective row,
*            or zero if no such row was found.
*
*     ObjAdd is a real constant extracted from row iObj of the RHS.
*            It is zero if the RHS contained no objective entry.
*            SNOPT adds ObjAdd to the objective function.
*
*     PrbNms(1)-PrbNms(5) contain the names of the
*               Problem, Objective row, RHS, RANGES and BOUNDS
*            respectively.
*
*     a(*)   contains the ne entries for each column of the matrix
*            specified in the COLUMNS section.
*
*     ha(*)  contains the corresponding row indices.
*
*     ka(j)  (j = 1 to n) points to the beginning of column j
*            in the parallel arrays a(*), ha(*).
*     ka(n+1) = ne+1.
*
*     bl(*)  contains n+m lower bounds for the columns and slacks.
*            If there is no lower bound on x(j), then bl(j) = - 1.0d+20.
*
*     bu(*)  contains n+m lower bounds for the columns and slacks.
*            If there is no upper bound on x(j), then bu(j) = + 1.0d+20.
*
*     Names(*) contains n+m column and row names in character*8 format.
*            The j-th column name is stored in Names(j).
*            The i-th row    name is stored in Names(k),
*            where k = n + i.  
*
*     hs(*)  contains an initial state for each column and slack.
*
*     x(*)   contains an initial value for each column and slack.
*
*            If there is no INITIAL bounds set,
*               x(j) = 0 if that value lies between bl(j) and bu(j),
*                     = the bound closest to zero otherwise,
*               hs(j) = 0 if x(j) < bu(j),
*                     = 1 if x(j) = bu(j).
*
*            If there is an INITIAL bounds set, x(j) and hs(j) are
*            set as follows.  Suppose the j-th variable has the name Xj,
*            and suppose any numerical value specified happens to be 3.
*                                                   x(j)    hs(j)
*             FR INITIAL   Xj         3.0           3.0       -1
*             FX INITIAL   Xj         3.0           3.0        2
*             LO INITIAL   Xj                       bl(j)      4
*             UP INITIAL   Xj                       bu(j)      5
*             MI INITIAL   Xj         3.0           3.0        4
*             PL INITIAL   Xj         3.0           3.0        5
*
*     pi(*)  contains a vector defined by a special RHS called LAGRANGE.
*            If the MPS file contains no such RHS, pi(i) = 0.0, i=1:m.
*
*     inform =  0 if no fatal errors were encountered,
*            = 40 if the ROWS or COLUMNS sections were empty
*                 or iObj > 0 but iObj <= nnCon,
*            = 41 if maxm, maxn or maxne were too small.
*
*     nS     is the no. of FX INITIAL entries in the INITIAL bounds set.
*              
*
*     09 Jul 1997: Original version, derived from mimps.
*     20 Feb 1998: Current version of MPSinp.
*     ==================================================================
      character*4        key

      parameter         (mProb     =  51)
      parameter         (mObj      =  52)
      parameter         (mRhs      =  53)
      parameter         (mRng      =  54)
      parameter         (mBnd      =  55)
*     ------------------------------------------------------------------
      iPrint    = iw( 12)
      iSumm     = iw( 13)

      if (lencw .lt. 500 .or. leniw .lt. 500 .or. lenrw .lt. 500) then 
*        ---------------------------------------------------------------
*        Not enough workspace to do ANYTHING!
*        ---------------------------------------------------------------
         ierror = 42
         go to 800
      end if

*     Set undefined MPS options to default values.
*     Quit if no MPS file is specified.

      call s3dflt( cw, lencw, iw, leniw, rw, lenrw )

      iw( 21)   = nnCon
      iw( 22)   = nnJac
      iw( 23)   = nnObj

      cw(mProb) = PrbNms(1)
      cw(mObj ) = PrbNms(2)
      cw(mRhs ) = PrbNms(3)
      cw(mRng ) = PrbNms(4)
      cw(mBnd ) = PrbNms(5)

      iw(123)   = iMPS
      if (iMPS .le. 0) then
         ierror = 39
         go to 800
      end if

      nnL    = max( nnJac, nnObj )

*     ------------------------------------------------------------------
*     Allocate workspace for the MPS input routines.
*     s3getp finds a prime number for the length of the row hash table.
*     ------------------------------------------------------------------
      call s3getp( maxm, lenh )

      lhrtyp = 501
      lkBS   = lhrtyp  + 1 + maxm
      lkeynm = lkBS    + 1 + maxm
      minmps = lkeynm  + lenh

      if (leniw .lt. minmps) then
         ierror = 41
      else
         call s3mps ( ierror, key,
     $                maxm, maxn, maxne, lenh, nnL, nnCon, nnJac,
     $                m, n, ne, neJac, nS,
     $                iObj, ObjAdd,
     $                a, ha, ka, bl, bu, Names,
     $                hs, iw(lhrtyp), iw(lkBS), iw(lkeynm),
     $                x, pi,
     $                cw, lencw, iw, leniw, rw, lenrw )
      end if

*     ==================================================================
*     Done. Check for errors
*     ==================================================================
  800 if (     ierror .eq.  0) then

*        Relax

      else if (ierror .eq. 39) then
*        ---------------------------
*        No MPS file specified.
*        ---------------------------
         call s1page( 2, iw, leniw )
         if (iPrint .gt. 0) write(iPrint, 1000)
         if (iSumm  .gt. 0) write(iSumm , 1000)

      else if (ierror .eq. 40) then
*        ---------------------------
*        Fatal error in MPS file.
*        ---------------------------
         call s1page( 2, iw, leniw )
         if (iPrint .gt. 0) write(iPrint, 1100)
         if (iSumm  .gt. 0) write(iSumm , 1100)

      else if (ierror .eq. 41) then
*        ------------------------------------
*        Not enough storage to read MPS file.
*        ------------------------------------
         call s1page( 2, iw, leniw )
         if (iPrint .gt. 0) write(iPrint, 1110)
         if (iSumm  .gt. 0) write(iSumm , 1110)

      else if (ierror .eq. 42) then
         if (iPrint .gt. 0) write(iPrint, 9000) 
         if (iSumm  .gt. 0) write(iSumm , 9000) 
         if (iPrint .le. 0  .and.  iSumm .le. 0) then
            write(*, 9000)
         end if
      end if

*     Exit.

      PrbNms(1) = cw(mProb)
      PrbNms(2) = cw(mObj )
      PrbNms(3) = cw(mRhs )
      PrbNms(4) = cw(mRng )
      PrbNms(5) = cw(mBnd )
      inform    = ierror

      return

 1000 format(' EXIT -- no MPS file specified for MPS input')
 1100 format(' EXIT -- fatal errors in the MPS file')
 1110 format(' EXIT -- not enough storage to read the MPS file')
 1300 format(/ ' Length of row-name hash table  ', i12
     $       / ' Collisions during table lookup ', i12)
 1320 format(  ' No. of rejected coefficients   ', i12)
 1350 format(  ' No. of Jacobian entries specified', i10)
 1400 format(  ' No. of INITIAL  bounds  specified', i10
     $       / ' No. of superbasics specified   ', i12)
 9000 format(  ' EXIT -- MPSinp character, integer and real work',
     $         ' arrays must each have at least 500 elements')

*     end of MPSinp
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s3dflt( cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     s3dflt  sets undefined MPS options to their default values and
*     prints them.
*
*     15 Feb 1998: First version.
*     23 Feb 1998: Current version of s3dflt.
*     ==================================================================
      character*4        id(2)
      character*8        cBlank, cdummy
      integer            Aijtol, bStrc1, bStrc2, plInfy, rdummy

      parameter         (plInfy    =  70)
      parameter         (rdummy    =  69)

      parameter         (Aijtol    =  95)
      parameter         (bStrc1    =  96)
      parameter         (bStrc2    =  97)

      parameter         (lvlTim    =  77)
      parameter         (lprPrm    =  81)
      parameter         (minmax    =  87)
      parameter         (lDenJ     = 105)
      parameter         (mEr       = 106)
      parameter         (mLst      = 107)
      parameter         (nProb     = 108)
      parameter         (iMPS      = 123)

      parameter         (nnCon     =  21) 
      parameter         (maxm      = 133)
      parameter         (maxn      = 134)
      parameter         (maxne     = 135)

      parameter         (mProb     =  51)
      parameter         (mObj      =  52)
      parameter         (mRhs      =  53)
      parameter         (mRng      =  54)
      parameter         (mBnd      =  55)

      parameter         (idummy = -11111)
      parameter         (cdummy = '-1111111')
      parameter         (zero   =  0.0d+0)

      data               cblank /'        '/
      data               id     /' Den', 'Spar'/
*     ------------------------------------------------------------------

      iPrint     = iw( 12)

      if (cw(mProb)  .eq. cdummy     ) cw(mProb)  = cBlank
      if (cw(mObj )  .eq. cdummy     ) cw(mObj )  = cBlank
      if (cw(mRhs )  .eq. cdummy     ) cw(mRhs )  = cBlank
      if (cw(mRng )  .eq. cdummy     ) cw(mRng )  = cBlank
      if (cw(mBnd )  .eq. cdummy     ) cw(mBnd )  = cBlank

      if (iw(maxm)   .le. 0          ) iw(maxm)   = 100
      if (iw(maxn)   .le. 0          ) iw(maxn)   = 3*iw(maxm)
      if (iw(maxne)  .le. 0          ) iw(maxne)  = 5*iw(maxn)

      if (iw(lprPrm) .lt. 0          ) iw(lprPrm) =  1
      if (iw(iMPS  ) .eq. idummy     ) iw(iMPS  ) =  0
         
      if (iw(lvlTim) .lt. 0          ) iw(lvlTim) =  3
      if (iw(nProb ) .lt. 0          ) iw(nProb ) =  0
      if (iw(lDenJ ) .lt. 0          ) iw(lDenJ ) =  1
      if (iw(mEr   ) .lt. 0          ) iw(mEr   ) = 10
      if (iw(mLst  ) .lt. 0          ) iw(mLst  ) =  0
      if (iw(minmax) .eq. idummy     ) iw(minmax) =  1

      if (rw(plInfy) .lt. zero       ) rw(plInfy) = 1.0d+20
      if (rw(Aijtol) .lt. zero       ) rw(Aijtol) = 1.0d-10
      if (rw(bStrc1) .eq. rw(rdummy) ) rw(bStrc1) = zero
      if (rw(bStrc2) .eq. rw(rdummy) ) rw(bStrc2) = rw(plInfy)

      if (rw(Bstrc1) .gt. rw(Bstrc2)) then
         t          =   rw(Bstrc1)
         rw(Bstrc1) =   rw(Bstrc2)
         rw(Bstrc2) =   t
      end if

*     ------------------------------------------------------------------
*     Print parameters unless PRINT LEVEL = 0 or SUPPRESS PARAMETERS 
*     was specified.
*     ------------------------------------------------------------------
      if (iPrint .gt. 0  .and.  iw(lprPrm) .gt. 0) then
         call s1page( 1, iw, leniw )
         write(iPrint, 2000) iw(iMPS)  ,
     $                       iw(maxm)  , iw(nProb), rw(Bstrc1),
     $                       iw(maxn)  , iw(mLst) , rw(Bstrc2),
     $                       iw(maxne) , iw(mEr)  , rw(Aijtol)
         if (iw(nnCon) .gt. 0)
     $        write(iPrint, 2010) id(iw(lDenJ))
      end if

      return

 2000 format(  ' MPS Input Data'
     $       / ' =============='
     $/ ' MPS file ..............', i10
     $/ ' Row limit..............', i10, 6x,
     $  ' Problem Number.........', i10, 6x, 
     $  ' Lower bound default....', 1p,  e10.2
     $/ ' Column limit...........', i10, 6x,
     $  ' List limit.............', i10, 6x,
     $  ' Upper bound default....', 1p,  e10.2
     $/ ' Elements limit ........', i10, 6x,
     $  ' Error message limit....', i10, 6x,
     $  ' Aij tolerance..........', 1p,  e10.2)
 2010 format(  
     $  ' Jacobian...............', 4x, a4, 'se')

*     end of s3dflt
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s3getp( maxm, lenh )

*     ------------------------------------------------------------------
*     s3getp finds a prime number lenh suitably larger than maxm.
*     It is used as the length of the hash table for the MPS row names.
*     ------------------------------------------------------------------

      lenh   = maxm*2
      lenh   = max( lenh, 100 )
      lenh   = (lenh/2)*2 - 1
      k      = lenh/20 + 6

  100 k      = k + 1
      lenh   = lenh + 2
      do 120 i = 3, k, 2
         if (mod(lenh,i) .eq. 0) go to 100
  120 continue

*     end of s3getp
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s3hash( len, nen, ncoll,
     $                   key, mode, keytab,
     $                   Names, ka, found )

      integer            len, nen, ncoll, mode, ka, keytab(len)
      character*8        key, Names(nen)
      logical            found

*     ==================================================================
*     s3hash looks up and/or inserts keys in a hash table.
*     Reference:  R.P. Brent, CACM 16,2 (Feb 1973), pp. 105-109.
*     This version is simplified for the case where no entries are
*     deleted.
*
*     keytab is used as an index into a consecutive list of unique
*     identifiers Names(*), to find if "key" is in the list.
*
*     On entry,
*     len     is the (fixed) length of the hash table.
*             It must be significantly more than nen.
*     nen     is the number of unique identifiers.
*     ncoll   is the number of collisions so far.
*     key     is the identifier to be looked up or inserted.
*     mode    = 1 if key is just being looked up,
*             = 2 if key should be inserted into the hash table.
*     keytab  is the integer hash table.
*     Names   is the list of identifiers.
*
*     On exit,
*     ncoll   is (new) number of collisions so far.
*     ka      satisfies Names(ka) = key if key is in the hash table.
*     found   is true if key was already in the hash table on entry.
*
*     Approx 1975: First f66 version written for MINOS, using integer
*                  names.
*     16 Sep 1997: key and Names(*) are now character*8.
*                  Statement function "hash" introduced.  It should be
*                  the only thing different between f77 and f90.
*     ==================================================================
*     The following statement function "hash" returns an integer from
*     its character*8 argument.  f77 doesn't permit great efficiency
*     (or we would have done it this way long ago!).  f90 should be
*     better with the transfer function.
    
      integer            hash
      character*8        p

*-->  f77
*     16 Sep 1997: Try looking at just a few important characters.
*     02 Oct 1997: Adding ichar of p(1,2,5,8) gave over a million
*                  collisions on pilots.mps.
*                  Try left shifts and more chars.

      hash(p) = 16*(16*(16*(16*(16*ichar( p(1:1) )
     $                           + ichar( p(2:2) ))
     $                           + ichar( p(3:3) ))
     $                           + ichar( p(6:6) ))
     $                           + ichar( p(7:7) ))
     $                           + ichar( p(8:8) )

*-->  f90
*     16 Sep 1997: Simulate the function used in MINOS (with Name(*)
*                  treated as two integer arrays).  It seems reasonably
*                  efficient in terms of the number of collisions.

*     hash(p) =   transfer( p(1:4), mode )
*    $          - transfer( p(5:8), mode )
*     ==================================================================

      len2  = len - 2
      ic    = -1

*     Compute address of first probe (ir) and increment (iq).

      ikey  = hash( key )
      iq    = mod(ikey, len2) + 1
      ir    = mod(ikey, len)  + 1
      ka    = ir

*     Look in the table.

   20 kt    = keytab(ka)

*     Check for an empty space or a match.

      if (kt  .eq. 0       ) go to 30
      if (key .eq. Names(kt)) go to 60
      ic    = ic + 1
      ncoll = ncoll + 1

*     Compute address of next probe.
      ka    = ka + iq
      if (ka .gt. len) ka = ka - len

*     See if whole table has been searched.
      if (ka .ne. ir ) go to 20

*     The key is not in the table.
   30 found = .false.

*     Return with ka = 0 unless an entry has to be made.

      if (mode .eq. 2  .and.  ic .le. len2) go to 70
      ka    = 0
      return

   60 found = .true.
      return

*     Look for the best way to make an entry.

   70 if (ic .le. 0) return
      ia    = ka
      is    = 0

*     Compute the maximum length to search along current chain.
   80 ix    = ic - is
      kt    = keytab(ir)

*     Compute increment jq for current chain.

      ikey  = hash( Names(kt) )
      jq    = mod(ikey, len2) + 1
      jr    = ir

*     Look along the chain.
   90 jr    = jr + jq
      if (jr .gt. len) jr = jr - len

*     Check for a hole.

      if (keytab(jr) .eq. 0) go to 100
      ix    = ix - 1
      if (ix .gt. 0) go to 90
      go to 110

*     Save location of hole.

  100 ia    = jr
      ka    = ir
      ic    = ic - ix

*     Move down to the next chain.

  110 is    = is + 1
      ir    = ir + iq
      if (ir .gt. len) ir = ir - len

*     Go back if a better hole might still be found.
      if (ic .gt. is ) go to 80

*     If necessary move an old entry.
      if (ia .ne. ka ) keytab(ia) = keytab(ka)

*     end of s3hash
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s3inpt( ierror,
     $                   maxm, maxn, maxne, nnCon, nnJac, nnObj, 
     $                   m, n, ne,
     $                   iObj, ObjAdd,
     $                   mincw, miniw, minrw,
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)

      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     s3inpt  inputs constraint data in MPS format, and sets up
*     various quantities as follows:
*
*     ObjAdd     (output) is minus the coefficient in row iObj of the
*                RHS section (zero by default).  SNOPT adds it to the
*                objective function.
*               
*     m, n, ne   are the number of rows, columns and elements in A.
*
*     iObj       is the row number for the linear objective (if any).
*                It must come after any nonlinear rows.
*                iObj = 0 if there is no linear objective.
*
*     a, ha, ka  is the matrix A stored in at locations la, lha, lka.
*     bl, bu     are the bounds  stored at locations lbl, lbu.
*     hs, xs     are states and values   stored at   lhs, lxs.
*
*     hs(j)      is set to  0, 1  to indicate a plausible initial state
*                (at lo or up bnd) for each variable j  (j = 1 to n+m).
*                If crash is to be used, i.e., crash option gt 0 and
*                if no basis file will be supplied, the initial bounds
*                set may initialize hs(j) as follows to assist crash:
*
*     -1      if column or row j is likely to be in the optimal basis,
*      4      if column j is likely to be nonbasic at its lower bound,
*      5      if column j is likely to be nonbasic at its upper bound,
*      2      if column or row j should initially be superbasic,
*      0 or 1 otherwise.
*
*     xs(j)      is a corresponding set of initial values.
*                Safeguards are applied later by SNOPT, so the
*                values of hs and xs are not desperately critical.
*
*     18 Nov 1991: First version based on Minos routine m3inpt.
*     18 Feb 1998: Current version of s3inpt.
*     ==================================================================
      character*4        key, lENDA
      parameter         (neJac     =  20)
      parameter         (lENDA     = 'ENDA')
*     ------------------------------------------------------------------
      iRead  = iw( 10)
      iSpecs = iw( 11)
      iPrint = iw( 12)
      iSumm  = iw( 13)
      iMPS   = iw(123)

      lDenJ  = iw(105)

      maxcu  = iw(  6)
      maxiu  = iw(  4) 
      maxru  = iw(  2)

      nnL    = max( nnJac, nnObj )

*     Start.  We may come back here to try again with more workspace.
*     key retains the first 4 characters of the NAME, ROWS, COLUMNS
*     RHS, RANGES and BOUNDS cards.
*     s3getp finds a prime number for the length of the row hash table.

  100 ierror = 0

      call s3getp( maxm, lenh )

      mincw  = maxcu + 1
      miniw  = maxiu + 1
      minrw  = maxru + 1

*     Allocate temporary addresses for the problem data
*     based on maxm, maxn and maxne.

      call s3Mem ( ierror, maxm, maxn, maxne, lenh,
     $             mincw, miniw, minrw, 
     $             lencw, leniw, lenrw, iw )
      if (ierror .gt. 0) go to 700

*     Get the data from an MPS file.
*     Allocate space for the arguments of sqopt and s5solv.
*     These are for the data,
*        a, ha, ka, bl, bu, Names
*     and for the solution
*        hs, xs, pi, rc, hs.
*     The true values of m, n and ne are also found.

      la     = iw(251)
      lha    = iw(252)
      lka    = iw(253)
      lbl    = iw(254)
      lbu    = iw(255)
      lxs    = iw(256)
      lpi    = iw(257)
      lhs    = iw(259)
      lNames = iw(261)

*     These are used as work arrays

      lhrtyp = iw(267)
      lkBS   = iw(269)
      lkeynm = iw(327)

      call s3mps ( ierror, key,
     $             maxm, maxn, maxne, lenh, nnL, nnCon, nnJac,
     $             m, n, ne, iw(neJac), nS,
     $             iObj, ObjAdd,
     $             rw(la), iw(lha), iw(lka),
     $             rw(lbl), rw(lbu), cw(lNames),
     $             iw(lhs), iw(lhrtyp), iw(lkBS), iw(lkeynm),
     $             rw(lxs), rw(lpi),
     $             cw, lencw, iw, leniw, rw, lenrw )
      if (ierror .gt. 0) go to 700
                   
*     neJac  counted the actual Jacobian entries in the MPS file,
*     but for Jacobian = Dense, we have to reset it.

      if (lDenJ .eq. 1) iw(neJac) = nnCon * nnJac
 
*     ------------------------------------------------------------------
*     Compress storage, now that we know the size of everything.
*     ------------------------------------------------------------------
*     Save current positions of  bl, bu, etc.

      kNames = lNames

      kka    = lka
      khs    = lhs

      kbl    = lbl
      kbu    = lbu
      kxs    = lxs
      kpi    = lpi

      mincw  = maxcu + 1
      miniw  = maxiu + 1
      minrw  = maxru + 1

      call s3Mem ( ierror, m, n, ne, lenh,
     $             mincw, miniw, minrw, 
     $             lencw, leniw, lenrw, iw )
      if (ierror .gt. 0) go to 700

      la     = iw(251)
      lha    = iw(252)
      lka    = iw(253)
      lbl    = iw(254)
      lbu    = iw(255)
      lxs    = iw(256)
      lpi    = iw(257)
      lhs    = iw(259)
      lhrtyp = iw(267)
      lkBS   = iw(269)
      lNames = iw(261)
      lkeynm = iw(327)

*     Move bl, bu, etc. into their final positions.

      call icopy ( n+1, iw(kka)   , 1, iw(lka)   , 1 )
      call ccopy ( n+m, cw(kNames), 1, cw(lNames), 1 )
      call icopy ( n+m, iw(khs)   , 1, iw(lhs)   , 1 )

      call dcopy ( n+m, rw(kbl)   , 1, rw(lbl)   , 1 )
      call dcopy ( n+m, rw(kbu)   , 1, rw(lbu)   , 1 )
      call dcopy ( n+m, rw(kxs)   , 1, rw(lxs)   , 1 )
      call dcopy (   m, rw(kpi)   , 1, rw(lpi)   , 1 )

  700 if (     ierror .eq.  0) then

*        Relax

      else if (     ierror .ge. 42) then
*        ---------------------------------------------
*        Not enough character, integer or real memory.
*        ---------------------------------------------
         if (iPrint .gt. 0) write(iPrint, 9800)
         if (iSumm  .gt. 0) write(iSumm , 9800)
      
         if (ierror .eq. 42) then
            if (iPrint .gt. 0) write(iPrint, 9820) mincw
            if (iSumm  .gt. 0) write(iSumm , 9820) mincw
         else if (ierror .eq. 43) then
            if (iPrint .gt. 0) write(iPrint, 9830) miniw
            if (iSumm  .gt. 0) write(iSumm , 9830) miniw
         else if (ierror .eq. 44) then
            if (iPrint .gt. 0) write(iPrint, 9840) minrw
            if (iSumm  .gt. 0) write(iSumm , 9840) minrw
         end if

      else
         if (ierror .eq. 40) then 
*           ---------------------------------------
*           Fatal error in MPS file.
*           ---------------------------------------
            if (iPrint .gt. 0) write(iPrint, 9400)
            if (iSumm  .gt. 0) write(iSumm , 9400)

         else if (ierror .eq. 41) then
*           ---------------------------------------
*           Estimates of problem size were too low.
*           MPS file could not be read.
*           ---------------------------------------
            if (m .ge. maxm) then 

*              Too many rows.

               maxm   = m

            else 

*              Too many columns or elements.

               maxn   = n
               maxne  = ne
            end if

*           Try again if possible.

            if (iMPS .ne. iRead  .and.  iMPS .ne. iSpecs) then
               rewind iMPS
               go to 100
            end if

            ierror = 41
            if (iPrint .gt. 0) write(iPrint, 9410)
            if (iSumm  .gt. 0) write(iSumm , 9410)
         end if

*        ------------------------------------
*        Flush MPS file to the ENDATA card
*        if it is the same as the SPECS file.
*        ------------------------------------
         if (iMPS .eq. iSpecs) then
            do 800, idummy = 1, 100000
               if (key .eq. lENDA) go to 900
               read(iMPS, '(a4)') key
  800       continue
         end if
      end if

*     Exit.

  900 if (iMPS .ne. iRead  .and.  iMPS .ne. iSpecs) rewind iMPS

      return

 9400 format(  ' EXIT -- fatal errors in the MPS file')
 9410 format(  ' EXIT -- MPS file cannot be read.   ',
     $         ' Increase  Rows, Columns or Elements')
 9800 format(  ' EXIT -- not enough memory to start solving',
     $         ' the problem...' )
 9820 format(/ ' Total character workspace should be significantly',
     $         ' more than', i8)
 9830 format(/ ' Total integer   workspace should be significantly',
     $         ' more than', i8)
 9840 format(/ ' Total real      workspace should be significantly',
     $         ' more than', i8)

*     end of s3inpt
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s3Mem ( ierror, m, n, ne, lenh,
     $                   mincw, miniw, minrw, 
     $                   lencw, leniw, lenrw, iw )

      implicit           double precision (a-h,o-z)
      integer            iw(leniw)

*     ==================================================================
*     s3Mem   is called from s3inpt to find the minimum storage needed
*     for mps input.
*     
*     Storage is estimated using the dimensions:
*        m    , n    , ne
*
*     18 Feb 1998: First version.
*     18 Feb 1998: Current version of s3Mem.
*     ==================================================================

      nb     = n + m

*     Allocate space for the data,
*              a, ha, ka, hElast, bl, bu, Names
*     and for the solution
*              hs, xs, pi, rc, hs.
      
      lNames = mincw
      mincw  = lNames + nb

      lha    = miniw
      lka    = lha    + ne
      lhElas = lka    + n  +  1
      lhs    = lhElas + nb
      miniw  = lhs    + nb
      
      la     = minrw
      lbl    = la     + ne
      lbu    = lbl    + nb
      lxs    = lbu    + nb
      lpi    = lxs    + nb
      lrc    = lpi    + m
      minrw  = lrc    + nb

*     Allocate arrays needed during and after MPS input.

      lhfeas = miniw
      lkBS   = lhfeas + nb
      lkeynm = lkBS   + nb
      miniw  = lkeynm + lenh

      iw(252) = lha      
      iw(253) = lka   
      iw(259) = lhs   
      iw(260) = lhElas
      iw(261) = lNames

      iw(251) = la    
      iw(254) = lbl   
      iw(255) = lbu   
      iw(256) = lxs   
      iw(257) = lpi   
      iw(258) = lrc   


      iw(267) = lhfeas
      iw(269) = lkBS
      iw(327) = lkeynm

      ierror  = 0
      if (     mincw .gt. lencw) then
         ierror = 42
      else if (miniw .gt. leniw) then
         ierror = 43
      else if (minrw .gt. lenrw) then
         ierror = 44
      end if

*     end of s3Mem
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s3mps ( ierror, key,
     $                   maxm, maxn, maxne, lenh, nnL, nnCon, nnJac,
     $                   m, n, ne, neJac, nS,
     $                   iObj, ObjAdd,
     $                   a, ha, ka, bl, bu, Names,
     $                   hs, hrtype, kBS, keynam,
     $                   x, pi,
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*4        key
      character*8        Names(maxm+maxn)
      integer            keynam(lenh), kBS(maxm+maxn)
      integer            ha(maxne), hs(maxm+maxn), hrtype(maxm)
      integer            ka(maxn+1)
      double precision   a(maxne), bl(maxm+maxn), bu(maxm+maxn)
      double precision   x(maxn), pi(maxm)

      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     s3mps  does the work for MPSinp and s3inpt.
*
*     18 Feb 1998: First version.
*     18 Feb 1998: Current version of s3mps.
*     ==================================================================
      integer            ncard(6), iEr(20)
*     ------------------------------------------------------------------
      iPrint    = iw( 12)

*     ncard  counts the number of data records in each section.
                                      
      ierror = 0
      ncoll  = 0
      key    = '    '
      call iload ( 6, ncoll, ncard, 1 )

*     ------------------------------------------------------------------
*     Input ROWS.
*     lrow   is the location of the first rowname in Names.
*            The column names will later go at the front.
*     lenNam is the initial length of Names,
*            i.e. the maximum no. of names allowed for.
*     ------------------------------------------------------------------
      lrow   = maxn + 1
      lenNam = maxn + maxm
      call s3mpsa( ierror, iEr, line, maxm, maxn,
     $             iObj, ncoll, m,
     $             lrow, lenNam, lenh, nnL, nnCon, key, ncard,
     $             hrtype, keynam, Names, 
     $             cw, lencw, iw, leniw )
      if (ierror .ne. 0) go to 900

*     ------------------------------------------------------------------
*     m  is now known.
*     Input COLUMNS, RHS, RANGES.
*     ------------------------------------------------------------------
      call s3mpsb( ierror, iEr, line, maxn, maxne,
     $             lrow, lenNam, lenh, 
     $             ncoll, iObj, ObjAdd, m, n, ne,
     $             nnL, nnCon, nnJac, neJac, nA0, key, ncard,
     $             hrtype, keynam, Names,
     $             ka, ha, a, bl, bu, kBS, pi,
     $             cw, lencw, iw, leniw, rw, lenrw )
      if (ierror .ne. 0) go to 900

*     ------------------------------------------------------------------
*     n  and  ne  are now known.
*     Move the row names to be contiguous with the column names.
*     Input BOUNDS.
*     ------------------------------------------------------------------
      if (lrow .gt. n+1) then
         call ccopy ( m, Names(lrow), 1, Names(n+1), 1 )
      end if

      call s3mpsc( iEr, line, m, n, nS, lenNam,
     $             key, ncard, Names,
     $             bl, bu, hs, x,
     $             cw, lencw, iw, leniw, rw, lenrw )

      if (iPrint .gt. 0) then
                            write(iPrint, 1300) lenh, ncoll
         if (nA0    .gt. 0) write(iPrint, 1320) nA0
         if (nnCon  .gt. 0) write(iPrint, 1350) neJac
         if (nnL    .gt. 0  .or.  ncard(6) .gt. 0)
     $                      write(iPrint, 1400) ncard(6), nS
      end if

  900 return

 1300 format(/ ' Length of row-name hash table  ', i12
     $       / ' Collisions during table lookup ', i12)
 1320 format(  ' No. of rejected coefficients   ', i12)
 1350 format(  ' No. of Jacobian entries specified', i10)
 1400 format(  ' No. of INITIAL  bounds  specified', i10
     $       / ' No. of superbasics specified   ', i12)

*     end of s3mps
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s3mpsa( ierror, iEr, line, maxm, maxn,
     $                   iObj, ncoll, m,
     $                   lrow, lennm, lenh, nnL, nnCon, key, ncard,
     $                   hrtype, keynam, Names,
     $                   cw, lencw, iw, leniw )

      implicit           double precision (a-h,o-z)
      character*4        key
      character*8        Names(lennm)
      integer            hrtype(maxm)
      integer            iEr(20), ncard(6), keynam(lenh)

      character*8        cw(lencw)
      integer            iw(leniw)

*     ==================================================================
*     s3mpsa  inputs the name and rows sections of an MPS file.
*
*     Original version written by Keith Morris, Wellington, 1973.
*
*     15 Nov 1991: First version based on Minos routine m3mpsa.
*     19 Jul 1997: Thread-safe version.
*     24 Sep 1997: key and Names(*) are now character*8.
*     23 Sep 1997: Current version of s3mpsa.
*     ==================================================================
      logical            found, gotnm
      character*8        id(3)
      double precision   Aelem(2)
      character*8        cBlank
      character*4        lNAME, lROWS, lCOLU,
     $                   lEX, lGX, lLX, lNX, lXE, lXG, lXL, lXN

      parameter         (mProb     =  51)
      parameter         (mObj      =  52)

      data               cBlank           
     $                   /'        '/
      data                lNAME, lROWS, lCOLU 
     $                   /'NAME','ROWS','COLU'/
      data                 lEX,   lGX,   lLX,   lNX 
     $                   /' E  ',' G  ',' L  ',' N  '/
      data                 lXE,   lXG,   lXL,   lXN
     $                   /'  E ','  G ','  L ','  N '/
*     ------------------------------------------------------------------

      iPrint    = iw( 12)
      iSumm     = iw( 13)
      iMPS      = iw(123)
      mEr       = iw(106)
      mLst      = iw(107)

      call s1page( 1, iw, leniw )
      if (iPrint .gt. 0) write(iPrint, 1000)

      inform    = 0
      iObj      = 0
      line      = 0
      m         = 0
      gotnm     = cw(mObj) .ne. cBlank
      call iload ( 20  , 0, iEr  , 1 )
      call iload ( lenh, 0, keynam, 1 )

*     Look for the NAME card.

   10 call s3read( 1, iMPS, iPrint, line, 5, key, id, Aelem, inform )
      if (key .ne. lNAME) then
         if (iEr(1) .eq. 0) then
             iEr(1) = 1
             if (iPrint .gt. 0) write(iPrint, 1100)
             if (iSumm  .gt. 0) write(iSumm , 1100)
         end if
         go to 10
      end if

      cw(mProb) = id(2)
      if (iSumm  .gt. 0) write(iSumm, 5000) cw(mProb)

*     Look for the ROWS card.

      call s3read( 1, iMPS, iPrint, line, 5, key, id, Aelem, inform )
      inform = 0
      if (key .ne. lROWS) then
         iEr(1) = iEr(1) + 1
         if (iPrint .gt. 0) write(iPrint, 1120)
         if (iSumm  .gt. 0) write(iSumm , 1120)
         go to 35
      end if

*     ==================================================================
*     Read the row names and check if the relationals are valid.
*     ==================================================================
   30 call s3read( 1, iMPS, iPrint, line, mLst, key, id, Aelem, inform )
      if (inform .ne. 0) go to 110

   35 if      (key .eq. lGX  .or.  key .eq. lXG) then
         it  = -1
      else if (key .eq. lEX  .or.  key .eq. lXE) then
         it  =  0
      else if (key .eq. lLX  .or.  key .eq. lXL) then
         it  =  1
      else if (key .eq. lNX  .or.  key .eq. lXN) then
         it  =  2

*        Record objective name if we don't already have one.

         if (iObj .eq. 0) then
            if (.not. gotnm) then
               cw(mObj) = id(1)
               if (nnL .gt. 0) then
                  if (iPrint .gt. 0) 
     $               write(iPrint, 1170) cw(mObj)
                  if (iSumm  .gt. 0)
     $               write(iSumm , 1170) cw(mObj)
               end if
            end if
         
            if (id(1) .eq. cw(mObj)) then
               iObj     = m + 1
               ncard(1) = ncard(1) + 1
            end if
         end if
      else
         iEr(3) = iEr(3) + 1
         if (iEr(3) .le. mEr) then
            if (iPrint .gt. 0) write(iPrint, 1160) line, key, id
            if (iSumm  .gt. 0) write(iSumm , 1160) line, key, id
         end if
         go to 30
      end if

*     ------------------------------------------------------------------
*     Look up the row name  id(1)  in the hash table.
*     ------------------------------------------------------------------
      call s3hash( lenh, maxm, ncoll, 
     $             id(1), 2, keynam, 
     $             Names(lrow), ia, found )

*     Error if the row name was already there.
*     Otherwise, enter the new name into the hash table.

      if ( found ) then
         iEr(4) = iEr(4) + 1
         if (iEr(4) .le. mEr) then
            if (iPrint .gt. 0) write(iPrint, 1200) id
            if (iSumm  .gt. 0) write(iSumm , 1200) id
         end if
      else
         m      = m + 1
         if (m .le. maxm) then
            jrow        = maxn + m
            keynam(ia)  = m
            Names(jrow) = id(1)
            hrtype(m)   = it
         end if
      end if
      go to 30

*     ==================================================================
*     Should be COLUMNS card.
*     ==================================================================
  110 if (key .ne. lCOLU) then
         iEr(1) = iEr(1) + 1
         if (iPrint .gt. 0) write(iPrint, 1130)
         if (iSumm  .gt. 0) write(iSumm , 1130)
      end if

*     Error if no rows or too many rows.

      if (m .le. 0) then
         if (iPrint .gt. 0) write(iPrint, 1300)
         if (iSumm  .gt. 0) write(iSumm , 1300)
         iEr(1) = iEr(1) + 1
         ierror = 40
         go to 900

      else if (m .gt. maxm) then
         if (iPrint .gt. 0) write(iPrint, 3030) maxm, m
         if (iSumm  .gt. 0) write(iSumm , 3030) maxm, m
         iEr(1) = iEr(1) + 1
         ierror = 41
         go to 900

      end if

*     Warning if no objective row found.
*     Error if linear objective is ahead of nonlinear rows.

      if (iObj .eq. 0) then
         if (iPrint .gt. 0) write(iPrint, 1600)
         if (iSumm  .gt. 0) write(iSumm , 1600)

      else if (iObj .le. nnCon) then
         if (iPrint .gt. 0) write(iPrint, 1180) cw(mObj)
         if (iSumm  .gt. 0) write(iSumm , 1180) cw(mObj)
         ierror = 40
         go to 900

      end if

      if (iSumm  .gt. 0) write(iSumm, 5100) m

*     ------------------------------------------------------------------
*     Exit
*     ------------------------------------------------------------------
  900 return

 1000 format(' MPS file' / ' --------')
 1100 format(' XXXX  Garbage before NAME card')
 1120 format(' XXXX  ROWS card not found')
 1130 format(' XXXX  COLUMNS card not found')
 1160 format(' XXXX  Illegal row type at line', i7, '... ', a4, a8)
 1170 format(' ===>  Note --- row  ', a8,
     $   '  selected as linear part of objective.')
 1180 format(/ ' XXXX  The linear objective card      N ', a8
     $       / ' XXXX  is out of place.    Nonlinear constraints'
     $       / ' XXXX  must be listed first in the ROWS section.')
 1200 format(' XXXX  Duplicate row name --', a8, ' -- ignored')
 1300 format(' XXXX  No rows specified')
 1600 format(' ===>  Warning - no linear objective selected')
 3030 format(' XXXX  Too many rows.  Limit was', i8,
     $   4x, '  Actual number is', i8)
 5000 format(' Name   ', a8)
 5100 format(' Rows   ', i8)

*     end of s3mpsa
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s3mpsb( ierror, iEr, line, maxn, maxne, 
     $                   lrow, lennm, lenh,
     $                   ncoll, iObj, ObjAdd, m, n, ne,
     $                   nnL, nnCon, nnJac, neJac, nA0, key, ncard,
     $                   hrtype, keynam, Names,
     $                   ka, ha, a, bl, bu, kBS, pi,
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*4        key
      character*8        Names(lennm)
      integer            hrtype(m), ha(maxne)
      integer            iEr(20), ncard(6)
      integer            keynam(lenh), ka(maxn+1), kBS(m)
      double precision   a(maxne), bl(maxn+m), bu(maxn+m)
      double precision   pi(m)

      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     s3mpsb inputs the COLUMNS, RHS and RANGES sections of an MPS file.
*
*     Original version written by Keith Morris, Wellington, 1973.
*
*     19 Jul 1997: Thread-safe version.
*     24 Sep 1997: key and Names(*) are now character*8.
*     24 Sep 1997: Current version of s3mpsb.
*     ==================================================================
      character*8        id(3), RowNm, ColNm
      double precision   Aelem(2), bStruc(2)
      logical            dense, found, gotnm
      character*8        cBlank,  lLAGR
      character*4        lRHS  , lRHSX, lRANG

      parameter         (zero      = 0.0d+0)
      parameter         (mRhs      =  53)
      parameter         (mRng      =  54)

      data               cBlank           
     $                   /'        '/
      data               lRHS  , lRHSX, lRANG /'RHS ','RHS''','RANG'/
      data               lLAGR               /'LAGRANGE'/
*     ------------------------------------------------------------------
      iPrint    = iw( 12)
      iSumm     = iw( 13)
      iMPS      = iw(123)

      lDenJ     = iw(105)
      mEr       = iw(106)
      mLst      = iw(107)

      bStruc(1) = rw( 96)
      bStruc(2) = rw( 97)
      Aijtol    = rw( 95)
      plInfy    = rw( 70)

      ObjAdd    =   zero
      dense     =   lDenJ .eq. 1
      bigUpp    =   plInfy
      bigLow    = - bigUpp
      ColNm     =   '12345678'
      n         =   0
      na0       =   0
      ne        =   0
      ne1       = - 1
      neJac     =   0
      inform    =   0
      call iload ( m,    0, kBS, 1 )
      call dload ( m, zero, pi , 1 )

*     ==================================================================
*     Read the next columns card.
*     ==================================================================
  210 call s3read( 2, iMPS, iPrint, line, mLst, key, id, Aelem, inform )
      if (inform .ne. 0) go to 310

  220 if (id(1) .ne. ColNm) then

*        Start a new column.

         if (ne .le. ne1) go to 310
         n         = n + 1
         ne1       = ne
         ColNm     = id(1)

         if (n .le. maxn) then
            ka(n)    = ne + 1
            Names(n) = ColNm
         
*           Make room for a dense Jacobian column.
         
            if (nnCon .gt. 0) then
               lJac   = ne
               if (dense  .and.  n .le. nnJac) then
                  ne     = ne + nnCon
                  if (ne .le. maxne) then
                     ne  = ne - nnCon
                     do 225, i = 1, nnCon
                        ne     = ne + 1
                        ha(ne) = i
                        a(ne)  = zero
  225                continue
                  end if
               end if
            end if
         end if
      end if

*     Process two row names and values.

      do 260, i  = 1, 2

*        Check for only one on the card.

         RowNm  = id(i+1)

         if (RowNm .ne. cBlank) then

*           Look up the row name.

            call s3hash( lenh, m, ncoll,
     $                   RowNm, 1, keynam,
     $                   Names(lrow), ia, found )

            if ( found ) then
               aij    = aelem(i)
               irow   = keynam(ia)
         
*              Test for a duplicate entry.
         
               if (kBS(irow) .eq. n) then
                  iEr(8) = iEr(8) + 1
                  if (iPrint .gt. 0  .and.  iEr(8) .le. mEr) then
                     write(iPrint, 1420) ColNm, RowNm, Aij, line
                  end if
                  go to 260
               end if
         
               kBS(irow) = n
               if (irow .le. nnCon  .and.  n .le. nnJac) then
         
*                 Deal with Jacobian elements.
         
                  neJac  = neJac + 1
                  if ( dense ) then
                     a(ne1 + irow) = aij
                     go to 260
                  end if
         
*                 Sparse Jacobian -- make sure the new element is
*                 squeezed in ahead of any linear-constraint elements.
         
                  lJac   = lJac + 1
                  if (lJac .le. ne) then
                     aij      = a(lJac)
                     irow     = ha(lJac)
                     a(lJac)  = aelem(i)
                     ha(lJac) = keynam(ia)
                  end if

               else if (abs(aij) .lt. aijtol) then
         
*                 Ignore small aijs.
            
                  nA0    = nA0 + 1
                  go to 260
               end if
         
*              Pack the nonzero.
         
               ne     = ne + 1
               if (ne .le. maxne) then
                  ha(ne) = irow
                  a(ne)  = Aij
               end if
            else
               iEr(5) = iEr(5) + 1
               if (iPrint .gt. 0  .and.  iEr(5) .le. mEr) then
                  write(iPrint, 1400) RowNm, line
               end if
            end if
         end if ! RowNm ne Blank
  260 continue

      go to 210

*     Test for an empty column.

  310 if (ne .le. ne1) then
     
*        Column with no rows.   Warning unless variable is nonlinear.
*        Insert dummy column with zero in first row.
      
         if (n .gt. nnL) then
            iEr(6) = iEr(6) + 1
            if (iPrint .gt. 0  .and.  iEr(6) .le. mEr) then
               write(iPrint, 1500) ColNm
            end if
         end if

         ne     = ne + 1
         if (ne .le. maxne) then
            ha(ne) = 1
            a(ne)  = zero
         end if
         if (inform .eq. 0) go to 220
      end if

*     ==================================================================
*     See if we have hit the RHS.
*     ==================================================================
      if (key .ne. lRHS  .and.  key .ne. lRHSX) then

*        Nope sumpins rong.
*        Terminate the COLUMNS section anyway.
      
         iEr(7) = iEr(7) + 1
         if (iPrint .gt. 0) write(iPrint, 1140)
         if (iSumm  .gt. 0) write(iSumm , 1140)
      end if
      
*     Are there any columns at all?
*     Or too many columns or elements?

      if (n .le. 0) then
         if (iPrint .gt. 0) write(iPrint, 1610)
         if (iSumm  .gt. 0) write(iSumm , 1610)
         iEr(2) = iEr(2) + 1
         ierror = 40
         return

      else if (n .gt. maxn) then
         if (iPrint .gt. 0) write(iPrint, 3040) maxn, n
         if (iSumm  .gt. 0) write(iSumm , 3040) maxn, n
         iEr(2) = iEr(2) + 1
         ierror = 41
         return

      else if (ne .gt. maxne) then
         if (iPrint .gt. 0) write(iPrint, 3050) maxne, ne
         if (iSumm  .gt. 0) write(iSumm , 3050) maxne, ne
         iEr(2) = iEr(2) + 1
         ierror = 41
         return
      end if

*     ------------------------------------------------------------------
*     Input the RHS.
*     ------------------------------------------------------------------
      if (iSumm  .gt. 0) write(iSumm, 5200) n, ne

*     We finally know how big the problem is.

      ka(n+1)   = ne + 1

*     Set bounds to default values.

      call dload ( n, bStruc(1), bl, 1 )
      call dload ( n, bStruc(2), bu, 1 )

      do 410, i  = 1, m
         k       = hrtype(i)
         jslack  = n + i

         if (k .le. 0) bl(jslack) = zero
         if (k .lt. 0) bu(jslack) = bigUpp
         if (k .gt. 0) bl(jslack) = bigLow
         if (k .ge. 0) bu(jslack) = zero
         if (k .eq. 2) bu(jslack) = bigUpp
  410 continue

*     Check for no RHS.

      if (key .ne. lRHS  .and.  key .ne. lRHSX) go to 600
      gotnm  = cw(mRhs) .ne. cBlank
      inform = 0

*     ==================================================================
*     Read next RHS card and see if it is the one we want.
*     ==================================================================
  420 call s3read( 2, iMPS, iPrint, line, mLst, key, id, aelem, inform )
      if (inform .ne. 0) go to 600

*     A normal RHS is terminated if LAGRANGE is found.

      if (id(1) .eq. lLAGR) go to 490

      if (.not. gotnm) then
         gotnm     = .true.
         cw(mRhs) = id(1)
      end if

      if (id(1) .eq. cw(mRhs)) then

*        Look at both halves of the record.
      
         do 440, i  = 1, 2

            RowNm   = id(i+1)

            if (RowNm .ne. cBlank) then
               call s3hash( lenh, m, ncoll,
     $                      RowNm, 1, keynam,
     $                      Names(lrow), ia, found )
      
               if ( found ) then
                  ncard(2) = ncard(2) + 1
                  bnd      = aelem(i)
                  irow     = keynam(ia)
                  jslack   = n + irow
                  k        = hrtype(irow)

                  if (irow .eq. iObj) then
                     ObjAdd = bnd

                  else if (k .ne. 2) then
                     if (k .le. 0) bl(jslack) = bnd
                     if (k .ge. 0) bu(jslack) = bnd
                  end if

               else
                  iEr(5) = iEr(5) + 1
                  if (iPrint .gt. 0  .and.  iEr(5) .le. mEr) then
                     write(iPrint, 1400) RowNm, line
                  end if
               end if
            end if ! if RowNm ne Blank
  440    continue
      end if

      go to 420

*     LAGRANGE RHS found.

  490 if (ncard(2) .eq. 0) then
         cw(mRhs) = cBlank
         if (iPrint .gt. 0) write(iPrint, 1720)
      end if
      go to 520

*     ==================================================================
*     Read next RHS card and see if it is a LAGRANGE one.
*     ==================================================================
  510 call s3read( 2, iMPS, iPrint, line, mLst, key, id, aelem, inform )
      if (inform .ne. 0) go to 600

      if (id(1) .ne. lLAGR) go to 510

*     Find which row.
*     Look at both halves of the record.

  520 do 540, i = 1, 2

         RowNm   = id(i+1)

         if (RowNm .ne. cBlank) then
            call s3hash( lenh, m, ncoll,
     $                   RowNm, 1, keynam,
     $                   Names(lrow), ia, found )
         
            if ( found ) then
               ncard(5) = ncard(5) + 1
               irow     = keynam(ia)
               pi(irow) = aelem(i)
            else
               iEr(5) = iEr(5) + 1
               if (iPrint .gt. 0  .and.  iEr(5) .le. mEr) then
                  write(iPrint, 1400) RowNm, line
               end if
            end if
         end if ! RowNm ne Blank
  540 continue

      go to 510

*     ------------------------------------------------------------------
*     RHS has been input.
*     ------------------------------------------------------------------
  600 if (ncard(2) .eq. 0) then
         if (iPrint .gt. 0) write(iPrint, 1620)
         if (iSumm  .gt. 0) write(iSumm , 1620)
      end if

      if (ObjAdd .ne. zero) then
         if (iPrint .gt. 0) write(iPrint, 1630) ObjAdd
         if (iSumm  .gt. 0) write(iSumm , 1630) ObjAdd
      end if

*     ------------------------------------------------------------------
*     Input RANGES.
*     ------------------------------------------------------------------
*     Check for no RANGES.

      if (key .ne. lRANG) go to 800
      gotnm  = cw(mRng) .ne. cBlank
      inform = 0
                                  
*     ==================================================================
*     Read card and see if it is the range we want.
*     ==================================================================
  610 call s3read( 2, iMPS, iPrint, line, mLst, key, id, aelem, inform )
      if (inform .ne. 0) go to 800

      if (.not. gotnm) then
         gotnm     = .true.
         cw(mRng) = id(1)
      end if

      if (id(1) .eq. cw(mRng)) then

*        Look at both halves of the record.
      
         do 640, i  = 1, 2
            RowNm   = id(i+1)

            if (RowNm .ne. cBlank) then
               call s3hash( lenh, m, ncoll, RowNm, 1,
     $                      keynam, Names(lrow), ia, found )

               if ( found ) then
                  ncard(3) = ncard(3)+1
                  brng     = aelem(i)
                  arng     = abs( brng )
                  irow     = keynam(ia)
                  jslack   = n + irow
                  k        = hrtype(irow)

                  if      (k .eq. 0) then
                     if (brng .gt. zero) bu(jslack) = bl(jslack) + arng
                     if (brng .lt. zero) bl(jslack) = bu(jslack) - arng

                  else if (k .eq. 2) then
*                    Relax

                  else if (k .lt. 0) then
                     bu(jslack) = bl(jslack) + arng

                  else if (k .gt. 0) then
                     bl(jslack) = bu(jslack) - arng
                  end if

               else
                  iEr(5) = iEr(5) + 1
                  if (iPrint .gt. 0  .and.  iEr(5) .le. mEr) then
                     write(iPrint, 1400) RowNm, line
                  end if
               end if
            end if ! RowNm ne Blank
  640    continue
      end if

      go to 610

*     RANGES have been input.

  800 return

 1140 format(' XXXX  RHS card not found')
 1400 format(' XXXX  Non-existent row    specified -- ', a8,
     $   ' -- entry ignored in line', i7)
 1420 format(' XXXX  Column  ', a8, '  has more than one entry',
     $       ' in row  ', a8
     $     / ' XXXX  Coefficient', 1p, e15.5, '  ignored in line', i10)
 1500 format(' XXXX  No valid row entries in column  ', a8)
 1610 format(' XXXX  No columns specified')
 1620 format(' ===>  Warning - the RHS is zero')
 1630 format(' ===>  Note:  constant', 1p, e15.7,
     $   '  is added to the objective.')
 1720 format(' ===>  Warning - first RHS is LAGRANGE.',
     $       '   Other RHS''s will be ignored.')
 3040 format(' XXXX  Too many columns.   The limit was', i8,
     $   4x, '  Actual number is', i8)
 3050 format(' XXXX  Too many elements.  The limit was', i8,
     $   4x, '  Actual number is', i8)
 5200 format(' Columns', i8 / ' Elements', i7)

*     end of s3mpsb
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s3mpsc( iEr, line, m, n, nS, lennm,
     $                   key, ncard, Names, 
     $                   bl, bu, hs, xs,
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*4        key
      character*8        Names(lennm)
      integer            hs(n+m)
      integer            iEr(20), ncard(6)
      double precision   bl(n+m), bu(n+m)
      double precision   xs(n+m)

      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     s3mpsc inputs the BOUNDS section of an MPS file.
*
*     Original version written by Keith Morris, Wellington, 1973.
*
*     19 Jul 1997: Thread-safe version.
*     24 Sep 1997: key and Names(*) are now character*8.
*     24 Sep 1997: Current version of s3mpsc.
*     ==================================================================
      character*8        id(3)
      character*8        Objtyp(3)
      character*8        cBlank, lINIT
      character*4        lBOUN, lENDA,
     $                   lFR  , lFX  , lLO, lMI, lPL, lUP
      double precision   aelem(2)
      logical            gotnm, ignore

      parameter         (zero      = 0.0d+0)
      parameter         (mObj      =  52)
      parameter         (mRhs      =  53)
      parameter         (mRng      =  54)
      parameter         (mBnd      =  55)

      data               cBlank               /'        '/
      data               lINIT                /'INITIAL '/
      data               lBOUN, lENDA         /'BOUN','ENDA'/
      data               lFR  , lFX, lLO      /' FR ',' FX ',' LO '/
      data               lMI  , lPL, lUP      /' MI ',' PL ',' UP '/
      data               Objtyp /'Max     ', 'Feas    ', 'Min     '/
*     ------------------------------------------------------------------
      iPrint    = iw( 12)
      iSumm     = iw( 13)
      iLoadB    = iw(122)
      insrt     = iw(125)
      iOldB     = iw(126)
      iMPS      = iw(123)
      minmax    = iw( 87)

      mEr       = iw(106)
      mLst      = iw(107)

      plInfy    = rw( 70)

      inform    =   1
      bigUpp    =   plInfy
      bigLow    = - bigUpp

*     Check for no BOUNDS.

      if (key .ne. lBOUN) go to 700
      gotnm     = cw(mBnd) .ne. cBlank
      inform    = 0
      jmark     = 1

*     ==================================================================
*     Read and check BOUNDS cards.  Notice the double plural.
*     ==================================================================
  610 call s3read( 3, iMPS, iPrint, line, mLst, key, id, aelem, inform )
      if (inform .ne. 0) go to 700

*     A normal bounds set is terminated if INITIAL is found.

      bnd     = aelem(1)
      if (id(1) .eq. lINIT) go to 690

      if (.not. gotnm) then
         gotnm    = .true.
         cw(mBnd) = id(1)
      end if

      if (id(1) .eq. cw(mBnd)) then

*        Find which column.

         call s4name( n, iPrint, Names, id(2),
     $                line, iEr(10), 0, 1, n, jmark, j )

         if (j .le. 0) then
            if (iPrint .gt. 0  .and.  iEr(10) .le. mEr) then
               write(iPrint, 1400) id(2), line
            end if
         else

*           Select bound type for column j.
      
            ncard(4) = ncard(4) + 1
            if      (key .eq. lUP) then
               bu(j)  = bnd

            else if (key .eq. lLO) then
               bl(j)  = bnd

            else if (key .eq. lFX) then
               bu(j)  = bnd
               bl(j)  = bnd

            else if (key .eq. lFR) then
               bu(j)  = bigUpp
               bl(j)  = bigLow

            else if (key .eq. lMI) then
               if (bu(j) .ge. bigUpp) bu(j)  = zero
               bl(j)  = bigLow

            else if (key .eq. lPL) then
               bu(j)  = bigUpp

            else
*              This lad didn't even make it to Form 1.
         
               iEr(11) = iEr(11) + 1
               if (iPrint .gt. 0  .and.  iEr(11) .le. mEr) then
                  write(iPrint, 1700) line, key, (id(i), i=1,2)
               end if
            end if
         end if
      end if

      go to 610

*     INITIAL bounds set found.

  690 if (ncard(4) .eq. 0) then
         cw(mBnd) = cBlank
         if (iPrint .gt. 0) write(iPrint, 1720)
      end if

*     ------------------------------------------------------------------
*     End of normal bounds.
*     ------------------------------------------------------------------
  700 nS     =     0
      bigUpp =     bigUpp*0.9d+0
      bigLow =   - bigUpp

*     Set variables to be nonbasic at zero (as long as that's feasible).
*     All variables will be eligible for the initial basis.

      do 706, j = 1, n+m
         xs(j)  = max(  zero, bl(j) )
         xs(j)  = min( xs(j), bu(j) )
         hs(j)  = 0
         if (xs(j)  .eq. bu(j)) hs(j) = 1
  706 continue

*     Ignore INITIAL bounds if a basis will be loaded.

      if (inform .ne. 0) go to 790
      ignore = ioldB .gt. 0  .or.  insrt .gt. 0  .or.  iLoadB .gt. 0
      if (.not. ignore ) then
         jmark  = 1
         go to 720
      end if

*     ==================================================================
*     Read INITIAL bounds set.
*     ==================================================================
  710 call s3read( 3, iMPS, iPrint, line, mLst, key, id, aelem, inform )
      if (inform .ne. 0) go to 790

      bnd    = aelem(1)
      if (ignore  .or.  id(1) .ne. lINIT) go to 710

*     Find which column.

  720 call s4name( n, iPrint, Names, id(2),
     $             line, iEr(12), 0, 1, n, jmark, j )

      if (j .le. 0) then
         if (iPrint .gt. 0  .and.  iEr(12) .le. mEr)
     $      write(iPrint, 1400) id(2), line
      else

*        Select bound type for column j.
      
         ncard(6) = ncard(6)+1
         if      (key .eq. lFR) then
            js  = -1
         else if (key .eq. lFX) then
            js  =  2
            nS  = nS + 1
         else if (key .eq. lLO) then
            js  =  4
            bnd = bl(j)
         else if (key .eq. lUP) then
            js  =  5
            bnd = bu(j)
         else if (key .eq. lMI) then
            js  =  4
         else if (key .eq. lPL) then
            js  =  5
         else
            iEr(13) = iEr(13) + 1
            if (iPrint .gt. 0  .and.  iEr(13) .le. mEr)
     $         write(iPrint, 1700) line, key, (id(i), i=1,2)
            go to 710
         end if
      end if

      if (abs( bnd ) .ge. bigUpp) bnd = zero
      xs(j)  = bnd
      hs(j)  = js
      go to 710

*     Should be ENDATA card.

  790 if (key .ne. lENDA) then
         iEr(14) = 1
         if (iPrint .gt. 0) write(iPrint, 1150)
         if (iSumm  .gt. 0) write(iSumm , 1150)
      end if

*     ------------------------------------------------------------------
*     Pass the Buck - not got to Truman yet.
*     ------------------------------------------------------------------
*     Check that  bl .le. bu

      do 802, j = 1, n
         b1     = bl(j)
         b2     = bu(j)
         if (b1 .gt. b2) then
            iEr(20) = iEr(20) + 1
            if (iPrint .gt. 0  .and.  iEr(20) .le. mEr)
     $         write(iPrint, 1740) j, b1, b2
            bl(j) = b2
            bu(j) = b1
         end if
  802 continue

*     Count the errors.

      k      = 0
      do 804, i = 1, 20
         k      = k + iEr(i)
  804 continue
      if (k .gt. 0) then
         if (iPrint .gt. 0) write(iPrint, 1900) k
         if (iSumm  .gt. 0) write(iSumm , 1900) k
      end if
      if (iPrint .gt. 0) then
         write(iPrint, 2100) cw(mObj), Objtyp(minmax+2)(1:3), ncard(1),
     $                       cw(mRhs),                        ncard(2),
     $                       cw(mRng),                        ncard(3),
     $                       cw(mBnd),                        ncard(4)
      end if

      return

 1150 format(' XXXX  ENDATA card not found')
 1400 format(' XXXX  Non-existent column specified -- ', a8,
     $   ' -- entry ignored in line', i7)
 1700 format(' XXXX  Illegal bound type at line', i7, '... ',
     $       a4, a8, 2x, a8)
 1720 format(' ===>  Warning - first bounds set is  INITIAL .',
     $       '   Other bounds will be ignored.')
 1740 format(/' XXXX  Bounds back to front on column', i6,' :',
     $       1p, 2e15.5)
 1900 format(/' XXXX  Total no. of errors in MPS file', i6)
 2100 format(///
     $   ' Names selected' /
     $   ' --------------' /
     $   ' Objective', 6x, a8, ' (', a3, ')', i8 /
     $   ' RHS      ', 6x, a8, i14 /
     $   ' RANGES   ', 6x, a8, i14 /
     $   ' BOUNDS   ', 6x, a8, i14)

*     end of s3mpsc
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s3read( mode, iMPS, iPrint, line, mxlist, 
     $                   key, id, Aelem, inform )

      implicit           double precision (a-h,o-z)
      character*4        key
      character*8        id(3)
      double precision   Aelem(2)

*     ==================================================================
*     s3read  reads data from file iMPS and prints a listing on file
*     iPrint.  The data is assumed to be in MPS format, with items of
*     interest in the following six fields...
*
*     Field:     1         2         3         4         5         6
*
*     Columns: 01-04     05-12     15-22     25-36     40-47     50-61
*
*     Format:    a4        a8        a8      e12.0       a8      e12.0
*
*     Data:     key      id(1)     id(2)   Aelem(1)    id(3)   Aelem(2)
*
*
*     Comments may contain a * in column 1 and anything in columns 2-61.
*     They are listed and then ignored.
*
*
*     On entry,  mode    specifies which fields are to be processed.
*     On exit ,  inform  is set to 1 if column 1 is not blank.
*
*     15 Nov 1991: First version based on Minos routine m3read.
*     24 Sep 1997: key and id(*) are now character*8.
*     24 Sep 1997: Current version of s3read.
*     ==================================================================
      character*61       buffer
      character*1        buff1
      character*1        lblank, lstar
      data               lblank/' '/, lstar/'*'/
*     ------------------------------------------------------------------
*     Read a data card and look for keywords and comments.
*     ------------------------------------------------------------------
   10 read (iMPS, 1000) buffer
      buff1  = buffer(1:1)
      line   = line + 1

*     Print the buffer if column 1 is nonblank
*     or if a listing is wanted.

      if (buff1 .ne. lblank  .or.  line .le. mxlist) then

*        Find the last nonblank character.

         do 20, last = 61, 2, -1
            if (buffer(last:last) .ne. lblank) go to 30
   20    continue
         last   = 1

   30    if (iPrint .gt. 0) write(iPrint, 2000) line, buffer(1:last)
      end if

*     Ignore comments.

      if (buff1 .eq. lstar ) go to 10

*     If column 1 is nonblank, load key and exit.
*     The NAME card is unusual in having some data in field 3.
*     We have to load it into id(2).

      if (buff1 .ne. lblank) then
         read(buffer, 1100) key, id(1), id(2)
         inform = 1
         return
      end if

*     ------------------------------------------------------------------
*     Process normal data cards.
*     ------------------------------------------------------------------
      if (mode .eq. 1) then

*        NAME or ROWS sections.

         read(buffer, 1100) key, id(1), id(2)
      else if (mode .eq. 2) then

*        COLUMNS, RHS or RANGES sections.

         read(buffer, 1100) key, id(1), id(2), aelem(1), id(3), aelem(2)
      else

*        BOUNDS section.

         read(buffer, 1100) key, id(1), id(2), aelem(1)
      end if

      return

 1000 format(a61)
 1100 format(a4, a8, 2x, a8, 2x, bn, e12.0, 3x, a8, 2x, e12.0)
 2000 format(i7, 4x, a)

*     end of s3read
      end

