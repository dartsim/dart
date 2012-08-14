*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
*     File  sn40bfil.f
*
*     s4getB   s4id     s4ksav   s4name   s4inst   s4load   s4oldb
*     s4chek   s4dump   s4newB   s4pnch   s4rept   s4soln   s4solp
*     s4stat   
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s4getB( ierror, m, n, nb, nName, nS, iObj,
     $                   hs, bl, bu, xs, Names,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      integer            hs(nb)
      character*8        Names(nName)
      integer            iw(leniw)
      double precision   rw(lenrw)
      double precision   bl(nb), bu(nb)
      double precision   xs(nb)

*     ==================================================================
*     s4getb loads one of the basis files.
*
*     15-Nov-91: First version of s4getb based on Minos routine m4getb.
*     13-Feb-94: Current version.
*     ==================================================================
      iLoadB    = iw(122)
      insrt     = iw(125)
      iOldB     = iw(126)

*     Load a basis file if one exists and istart = 0 (Cold start).

      if      (iOldB .gt. 0) then
         call s4oldb( ierror, m, n, nb, nS, hs, bl, bu, xs,
     $                iw, leniw, rw, lenrw )

      else if (insrt .gt. 0) then
         call s4inst( n, nb, nS, iObj,
     $                hs, bl, bu, xs, Names,
     $                iw, leniw, rw, lenrw )

      else if (iLoadB .gt. 0) then
         call s4load( n, nb, nS, iObj, 
     $                hs, xs, Names,
     $                iw, leniw, rw, lenrw )
      end if

*     end of s4getB
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s4id  ( j, n, nb, nName, Names, id )

      character*8        id
      character*8        Names(nName)

*     ==================================================================
*     s4id   returns a name id for the j-th variable.
*     If nName = nb, the name is already in Names.
*     Otherwise nName = 1. Some generic column or row name is cooked up
*
*     15-Nov-91: First version based on Minos 5.4 routine m4id.
*     16-Sep-97: Current version.
*     ==================================================================
      character*1        ColNm, RowNm
      data               ColNm /'x'/,     
     $                   RowNm /'r'/
*     ------------------------------------------------------------------
      if (nName .eq. nb) then
         id  = Names(j)
      else if (j .le. n) then
         write(id, '(a1,i7)') ColNm, j
      else
         i   = j - n
         write(id, '(a1,i7)') RowNm, i
      end if         

*     end of s4id
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s4ksav( minimz, m, n, nb, nS, mBS, f,
     $                   itn, nInf, sInf,
     $                   kBS, hs, aScale, bl, bu, xBS, xs,
     $                   cw, lencw, iw, leniw )

      implicit           double precision (a-h,o-z)
      integer            hs(nb)
      integer            kBS(mBS)
      double precision   aScale(*), bl(nb), bu(nb)
      double precision   xBS(mBS), xs(nb)
      character*8        cw(lencw)
      integer            iw(leniw)

*     ==================================================================
*     s4ksav  saves various quantities as determined by the frequency 
*     control ksav.
*
*     15-Nov-91: First version.
*     19-Feb-92: Current version.
*     ==================================================================
      character*4        istate(3)
*     ------------------------------------------------------------------
      iBack     = iw(120)
      iNewB     = iw(124)
      itnlim    = iw( 89)

      if (inewB .gt. 0  .and.  itn .lt. itnlim) then
         k = 0
         call s4stat( k, istate )
         call s4newB( 1, iNewB, minimz,
     $                m, n, nb, nS, mBS, f,
     $                itn, nInf, sInf,
     $                kBS, hs, aScale, bl, bu, xBS, xs, istate,
     $                cw, lencw, iw, leniw )

         if (iback .gt. 0) then
            call s4newB( 1, iBack, minimz,
     $                   m, n, nb, nS, mBS, f,
     $                   itn, nInf, sInf,
     $                   kBS, hs, aScale, bl, bu, xBS, xs, istate,
     $                   cw, lencw, iw, leniw )
         end if
      end if

*     end of s4ksav
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s4name( n, iPrint, Names, id,
     $                   ncard, notfnd, maxmsg, j1, j2, jmark, jfound )

      implicit           double precision (a-h,o-z)
      character*8        Names(n), id
           
*     ==================================================================
*     s4name searches for names in the array  Names(j), j = j1, j2.
*     jmark  will probably speed the search on the next entry.
*     Used by subroutines s3mpsc, s4inst, s4load.
*
*     Left-justified alphanumeric data is being tested for a match.
*
*     15-Nov-91: First version based on Minos 5.4 routine m4name.
*     04-Feb-92: Current version.
*     ==================================================================
      do 50, j = jmark, j2
         if (id .eq. Names(j)) go to 100
   50 continue

      do 60, j = j1, jmark
         if (id .eq. Names(j)) go to 100
   60 continue

*     Not found.

      jfound = 0
      jmark  = j1
      notfnd = notfnd + 1
      if (notfnd .le. maxmsg) then
         if (iPrint .gt. 0) write(iPrint, 1000) ncard, id
      end if
      return

*     Got it.

  100 jfound = j
      jmark  = j
      return

 1000 format(' XXX  Line', i6, '  --  name not found:', 8x, a8)

*     end of s4name
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s4inst( n, nb, nS, iObj,
     $                   hs, bl, bu, xs, Names,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*8        Names(nb)
      integer            hs(nb)
      integer            iw(leniw)
      double precision   rw(lenrw)
      double precision   bl(nb), bu(nb)
      double precision   xs(nb)

*     ==================================================================
*     This impression of INSERT reads a file produced by  s4pnch.
*     It is intended to read files similar to those produced by
*     standard MPS systems.  It recognizes SB as an additional key.
*     Also, values are extracted from columns 25--36.
*
*     15-Nov-91: First version based on Minos 5.4 routine m4inst.
*     04-Feb-92: Current version.
*     ==================================================================
      character*8        Name1, Name2
      character*4        id(5)
      character*4        key
      character*4        lLL  , lUL  , lXL  , lXU  , lSB  , lEND
      data               lLL  , lUL  , lXL  , lXU  , lSB  , lEND
     $                 /' LL ',' UL ',' XL ',' XU ',' SB ','ENDA'/
*     ------------------------------------------------------------------
      plInfy    = rw( 70)
      iRead     = iw( 10)
      iPrint    = iw( 12)
      iSumm     = iw( 13)
      insrt     = iw(125)
      mEr       = iw(106)

      bplus  = 0.9d+0*plInfy
      if (iPrint .gt. 0) write(iPrint, 1999) insrt
      if (iSumm  .gt. 0) write(iSumm , 1999) insrt

                         read (insrt , 1000) id
      if (iPrint .gt. 0) write(iPrint, 2000) id
      l1    = n + 1

*     Make logicals basic.

      do 20, j  = l1, nb
         hs(j) = 3
   20 continue

      ignord = 0
      nbs    = 0
      nS     = 0
      notfnd = 0
      ncard  = 0
      jmark  = 1
      lmark  = l1
      ndum   = n + 100000

      if (iObj .eq. 0) then
         jObj = 0
      else
         jObj = n + iObj
      end if

*     Read names until ENDATA

      do 300, nloop = 1, ndum
         read(insrt, 1020) key, Name1, Name2, xj
         if (key .eq. lEND) go to 310

*        Look for  Name1.  It may be a column or a row,
*        since a superbasic variable could be either.

         ncard  = nloop
         call s4name( nb, iPrint, Names, Name1,
     $                ncard, notfnd, mer, 1, nb, jmark, j )
         if (   j  .le. 0) go to 300
         if (hs(j) .gt. 1) go to 290
         if (key .ne. lXL  .and.  key .ne. lXU) go to 70

*        Look for  Name2.  It has to be a row.

         call s4name( nb, iPrint, Names, Name2,
     $                ncard, notfnd, mer, l1, nb, lmark, l )
         if (l .le. 0) go to 300

*        XL, XU (exchange card)  --  make col j basic,  row l nonbasic.

         if (l  .eq. jObj) go to 290
         if (hs(l) .ne. 3) go to 290
         nbs    = nbs + 1
         hs(j)  = 3
         if (key .eq. lXU) go to 50
         hs(l)  = 0
         if (bl(l) .gt. -bplus) xs(l) = bl(l)
         go to 250

   50    hs(l)  = 1
         if (bu(l) .lt.  bplus) xs(l) = bu(l)
         go to 250

*        LL, UL, SB  --  only  j  and  xj  are relevant.

   70    if (key .eq. lLL) go to 100
         if (key .eq. lUL) go to 150
         if (key .eq. lSB) go to 200
         go to 290

*        LO or UP

  100    hs(j)  = 0
         go to 250

  150    hs(j)  = 1
         go to 250

*        Make superbasic.

  200    hs(j)  = 2
         nS     = nS + 1

*        Save  xs  values.

  250    if (abs(xj) .lt. bplus) xs(j) = xj
         go to 300

*        Card ignored.

  290    ignord = ignord + 1
         if (iPrint .gt. 0  .and.  ignord .le. mer) then
            write(iPrint, 2010) ncard, key, Name1, Name2
         end if
  300 continue

  310 ignord = ignord + notfnd
      if (iPrint .gt. 0) write(iPrint, 2050) ncard, ignord, nBS, nS
      if (iSumm  .gt. 0) write(iSumm , 2050) ncard, ignord, nBS, nS
      if (insrt  .ne. iRead) rewind insrt
      return

 1000 format(14x, 2a4, 2x, 3a4)
 1005 format(2a4)
 1020 format(a4, a8, 2x, a8, 2x, e12.5)
 1999 format(/ ' INSERT file to be input from file', i4)
 2000 format(/ ' NAME', 10x, 2a4, 2x, 3a4)
 2010 format(' XXX  Line', i6, '  ignored:', 8x, a4, a8, 2x, a8)
 2050 format(/ ' No. of lines read      ', i6, '  Lines ignored', i6
     $       / ' No. of basics specified', i6, '  Superbasics  ', i6)

*     end of s4inst
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s4load( n, nb, nS, iObj,
     $                   hs, xs, Names,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      integer            hs(nb)
      character*8        Names(nb)
      integer            iw(leniw)
      double precision   rw(lenrw)
      double precision   xs(nb)

*     ==================================================================
*     s4load  inputs a load file, which may contain a full or partial
*     list of row and column names and their states and values.
*     Valid keys are   BS, LL, UL, SB.
*
*     15-Nov-91: First version based on Minos 5.4 routine m4load.
*     04-Feb-92: Current version.
*     ==================================================================
      character*8        Name
      character*4        id(5)
      character*4        key
      character*4        lBS  , lLL  , lUL  , lSB  , lEND
      data               lBS  , lLL  , lUL  , lSB  , lEND
     $                 /' BS ',' LL ',' UL ',' SB ','ENDA'/
*     ------------------------------------------------------------------
      plInfy    = rw( 70)
      iPrint    = iw( 12)
      iSumm     = iw( 13)
      iRead     = iw( 10)
      iLoadB    = iw(122)
      mEr       = iw(106)

      bplus  = 0.9d+0*plInfy
      if (iPrint .gt. 0) write(iPrint, 1999) iLoadB
      if (iSumm  .gt. 0) write(iSumm , 1999) iLoadB
                         read (iLoadB, 1000) id
      if (iPrint .gt. 0) write(iPrint, 2000) id
      ignord = 0
      nBS    = 0
      nS     = 0
      notfnd = 0
      ncard  = 0
      jmark  = 1
      ndum   = n + 100000

      if (iObj .eq. 0) then
         jObj = 0
      else
         jObj = n + iObj
      end if

*     Read names until ENDATA is found.

      do 300, nloop = 1, ndum
         read (iLoadB, 1020) key, Name, xj
         if (key .eq. lend) go to 310

         ncard  = nloop
         call s4name( nb, iPrint, Names, Name,
     $                ncard, notfnd, mer, 1, nb, jmark, j )
         if (j .le. 0) go to 300

*        The name Name belongs to the j-th variable.

         if (hs(j) .gt. 1) go to 290
         if (j   .eq.jObj) go to  90
         if (key .eq. lBS) go to  90
         if (key .eq. lLL) go to 100
         if (key .eq. lUL) go to 150
         if (key .eq. lSB) go to 200
         go to 290

*        Make basic.

   90    nBS    = nBS + 1
         hs(j)  = 3
         go to 250

*        LO or UP.

  100    hs(j)  = 0
         go to 250

  150    hs(j)  = 1
         go to 250

*        Make superbasic.

  200    nS     = nS + 1
         hs(j)  = 2

*        Save  xs  values.

  250    if (abs(xj) .lt. bplus) xs(j) = xj
         go to 300

*        Card ignored.

  290    ignord = ignord + 1
         if (ignord .le. mer) then
            if (iPrint .gt. 0) write(iPrint, 2010) ncard, key, Name
         end if
  300 continue

  310 ignord = ignord + notfnd
      if (iPrint .gt. 0) write(iPrint, 2050) ncard, ignord, nBS, nS
      if (iSumm  .gt. 0) write(iSumm , 2050) ncard, ignord, nBS, nS

*     Make sure the linear Objective is basic.

      if (iObj  .gt. 0) then
         if (hs(jObj) .ne. 3) then
            hs(jObj) = 3

*           Swap Obj with last basic variable.

            do 850 j = nb, 1, -1
               if (hs(j) .eq. 3) go to 860
  850       continue

  860       hs(j)  = 0
         end if
      end if

      if (iLoadB .ne. iRead) rewind iLoadB
      return

 1000 format(14x, 2a4, 2x, 3a4)
 1005 format(2a4)
 1020 format(a4, a8, 12x, e12.5)
 1999 format(/ ' LOAD file to be input from file', i4)
 2000 format(/ ' NAME', 10x, 2a4, 2x, 3a4)
 2010 format(' XXX  Line', i6, '  ignored:', 8x, a4, a8, 2x, a8)
 2050 format(/ ' No. of lines read      ', i6, '  Lines ignored', i6
     $       / ' No. of basics specified', i6, '  Superbasics  ', i6)

*     end of s4load
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s4oldb( ierror, m, n, nb, nS, hs, bl, bu, xs,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      integer            hs(nb)
      integer            iw(leniw)
      double precision   rw(lenrw)
      double precision   bl(nb), bu(nb)
      double precision   xs(nb)

*     ==================================================================
*     s4oldb  inputs a compact basis file from file  iOldB.
*
*     15-Nov-91: First version based on Minos 5.4 routine m4OldB.
*     04-Feb-92: Current version.
*     ==================================================================
      integer            id(20)
*     ------------------------------------------------------------------
      plInfy    = rw( 70)
      iRead     = iw( 10)
      iPrint    = iw( 12)
      iSumm     = iw( 13)
      iOldB     = iw(126)

      bplus  = 0.9d+0*plInfy
      if (iPrint .gt. 0) write(iPrint, 1999) iOldB
      if (iSumm  .gt. 0) write(iSumm , 1999) iOldB
         read (iOldB , 1000) id
      if (iPrint .gt. 0) then
         write(iPrint, 2000) id
      end if

         read (iOldB , 1005) (id(i), i=1,13), newm, newn, nS
      if (iPrint .gt. 0) then
         write(iPrint, 2005) (id(i), i=1,13), newm, newn, nS
      end if

      if (newm .ne. m  .or.  newn .ne. n) go to 900
      read (iOldB , 1010) hs

*     Set values for nonbasic variables.

      do 200, j = 1, nb
         js     = hs(j)
         if (js .le. 1) then
            if (js .eq. 0) xj = bl(j)
            if (js .eq. 1) xj = bu(j)
            if (abs(xj) .lt. bplus) xs(j) = xj
         end if
  200 continue

*     Load superbasics.

      nS     = 0
      ndummy = m + n + 10000

      do 300, idummy = 1, ndummy
         read(iOldB, 1020) j, xj
         if (j .le.  0) go to 310
         if (j .le. nb) then
            xs(j)  = xj
            if (hs(j) .eq. 2) nS = nS + 1
         end if
  300 continue

  310 if (nS .gt. 0) then
         if (iPrint .gt. 0) write(iPrint, 2010) nS
         if (iSumm  .gt. 0) write(iSumm , 2010) nS
      end if
      go to 990

*     Error exits.

*     -------------------------------------------
*     Incompatible basis file dimensions.
*     -------------------------------------------
  900 ierror = 30
      call s1page( 1, iw, leniw )
      if (iPrint .gt. 0) write(iPrint, 9300)
      if (iSumm  .gt. 0) write(iSumm , 9300)

  990 if (iOldB .ne. iRead) rewind iOldB
      return

 1000 format(20a4)
 1005 format(13a4, 2x, i7, 3x, i7, 4x, i5)
 1010 format(80i1)
 1020 format(i8, e24.14)
 1999 format(/ ' OLD BASIS file to be input from file', i4)
 2000 format(1x, 20a4)
 2005 format(1x, 13a4,
     $       'm=', i7, ' n=', i7, ' sb=', i5)
 2010 format(' No. of superbasics loaded', i7)

 9300 format(  ' EXIT -- the basis file dimensions do not match',
     $         ' this problem')

*     end of s4oldb
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s4chek( m, maxS, mBS, n, nb, nS, iObj,
     $                   hs, kBS, bl, bu, xs,
     $                   iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      integer            hs(nb)
      integer            kBS(mBS)
      integer            iw(leniw)
      double precision   rw(lenrw)
      double precision   bl(nb), bu(nb)
      double precision   xs(nb)

*     ==================================================================
*     s4chek  takes hs and xs and checks they contain reasonable values.
*     The entries hs(j) = 2 are used to set  nS  and possibly
*     the list of superbasic variables kBS(m+1) thru kBS(m+nS).
*     Scaling, if any, has taken place by this stage.
*
*     15-Nov-91: First version of s4chek based on Minos routine m4chek.
*     29-Jul-96: Current version.
*     ==================================================================
      parameter        ( zero = 0.0d+0,  tolb = 1.0d-4 )
*     ------------------------------------------------------------------
      plInfy    = rw( 70)
      iPrint    = iw( 12)
      iSumm     = iw( 13)

*     Make sure hs(j) = 0, 1, 2 or 3 only.

      do 5, j = 1, nb
         js   = hs(j)
         if (js .lt. 0) hs(j) = 0
         if (js .ge. 4) hs(j) = js - 4
    5 continue

*     ------------------------------------------------------------------
*     Make sure the Objective is basic and free.
*     Then count the basics and superbasics, making sure they don't
*     exceed m and maxS respectively.  Also, set nS and possibly
*     kBS(m+1) thru kBS(m+ns) to define the list of superbasics.
*     Mar 1988: Loop 100 now goes backwards to make sure we grab Obj.
*     ------------------------------------------------------------------
   10 nBasic = 0
      nS     = 0
      if (iObj .gt. 0) then
         hs(n+iObj) =   3
         bl(n+iObj) = - plInfy
         bu(n+iObj) =   plInfy
      end if

*     If too many basics or superbasics, make them nonbasic.
*     Do slacks first to make sure we grab the objective slack.

      j = n

      do 100, jj = 1, nb
         j       = j + 1
         if (j .gt. nb) j = 1
         js      = hs(j)
         if (js .eq. 2) then
            nS   = nS + 1
            if (nS .le. maxS) then
               kBS(m+nS) = j
            else
               hs(j)     = 0
            end if

         else if (js .eq. 3) then
            nBasic = nBasic + 1
            if (nBasic .gt. m) hs(j) = 0
         end if
  100 continue

*     Check the number of basics.

      nS     = min( nS, maxS )
      if (nBasic .ne. m ) then
         if (iPrint .gt. 0) write(iPrint, 1000) nBasic, m
         if (iSumm  .gt. 0) write(iSumm , 1000) nBasic, m
      end if

*     ------------------------------------------------------------------
*     Set each nonbasic xs(j) to be exactly on its
*     nearest bound if it is within tolb of that bound.
*     ------------------------------------------------------------------
      bigBnd = 0.1d+0 * plInfy
      do 300, j = 1, nb
         xj     = xs(j)
         if (abs( xj ) .ge.  bigBnd) xj = zero
         if (hs(j)     .le.  1     ) then
            b1  = bl(j)
            b2  = bu(j)
            xj  = max( xj, b1 )
            xj  = min( xj, b2 )
            if (   (xj - b1) .gt. (b2 - xj)) b1 = b2
            if (abs(xj - b1) .le.  tolb    ) xj = b1
            hs(j) = 0
            if (xj .gt. bl(j)) hs(j) = 1
         end if
         xs(j) = xj
  300 continue

      return

 1000 format(/ ' WARNING:', i7, ' basics specified;',
     $         ' preferably should have been', i7)

*     end of s4chek
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s4dump( iDump, nb, hs, xs, Names, 
     $                   cw, lencw, iw, leniw )

      implicit           double precision (a-h,o-z)
      character*8        Names(nb)
      integer            hs(nb)
      double precision   xs(nb)
      character*8        cw(lencw)
      integer            iw(leniw)

*     ==================================================================
*     s4dump outputs basis names in a format compatible with s4load.
*     This file is normally easier to modify than a punch file.
*
*     15-Nov-91: First version based on Minos 5.4 routine m4dump.
*     04-Feb-92: Current version.
*     ==================================================================
      character*8        mName
      character*4        key(4)
      data               key/' LL ', ' UL ', ' SB ', ' BS '/
*     ------------------------------------------------------------------
      iPrint    = iw( 12)
      iSumm     = iw( 13)
      mName     = cw( 51)
      write(iDump, 2000) mName

      do 500, j = 1, nb
         k      = hs(j) + 1
         write(iDump, 2100) key(k), Names(j), xs(j)
  500 continue

      write(iDump , 2200)
      if (iPrint .gt. 0) write(iPrint, 3000) iDump
      if (iSumm  .gt. 0) write(iSumm , 3000) iDump
      if (iDump .ne. iPrint) rewind iDump
      return

 2000 format('NAME', 10x, a8, 2x, '   DUMP/LOAD')
 2100 format(a4, a8, 12x, 1p, e12.5)
 2200 format('ENDATA')
 3000 format(/ ' DUMP file saved on file', i4)

*     end of s4dump
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s4newB( mode, iNewB, minimz, 
     $                   m, n, nb, nS, mBS, f,
     $                   itn, nInf, sInf,
     $                   kBS, hs, aScale, bl, bu, xBS, xs, istate,
     $                   cw, lencw, iw, leniw )

      implicit           double precision (a-h,o-z)
      integer            hs(nb)
      integer            kBS(mBS)
      double precision   aScale(*), bl(nb), bu(nb)
      double precision   xBS(mBS), xs(nb)
      character*4        istate(3)

      character*8        cw(lencw)
      integer            iw(leniw)

*     ==================================================================
*     s4newB  saves a compact basis on file inewB.  Called from s5qp.
*     If mode = 1, the save is a periodic one due to the save frequency.
*     If mode = 2, s5qp has just finished the current problem.
*                 (Hence, mode 2 calls occur at the end of every cycle.)
*
*     15-Nov-91: First version based on Minos 5.4 routine m4newb.
*     19-Feb-92: Current version.
*     ==================================================================
      character*8        mName, mObj, mRhs, mRng, mBnd 
      logical            scaled
*     ------------------------------------------------------------------
      if (mode .lt. 1  .or. mode .gt. 2) return

      iPrint    = iw( 12)
      iSumm     = iw( 13)
      nnJac     = iw( 22)
      lvlScl    = iw( 75)

      mName     = cw( 51)
      mObj      = cw( 52)
      mRhs      = cw( 53)
      mRng      = cw( 54)
      mBnd      = cw( 55)

      scaled    = lvlScl .gt. 0

      if (nInf .eq. 0) then
         Obj = minimz * f
      else
         Obj = sInf
      end if

*     Output header cards and the state vector.

      write(inewB, 1000) mName, itn , istate, nInf, Obj
      write(inewB, 1005) mObj , mRhs, mRng  , mBnd, m, n, nS
      write(inewB, 1010) hs

*     Output the superbasic variables.

      do 50, k = m+1, m+nS
         j     = kBS(k)
         xj    = xBS(k)
         if (scaled) xj = xj * aScale(j)
         write(inewB, 1020) j, xj
   50 continue

*     If there are nonlinear constraints,
*     output the values of all other (non-sb) nonlinear variables.

      do 100, j = 1, nnJac
         if (hs(j) .ne. 2) then
            xj    = xs(j)
            if (scaled) xj = xj * aScale(j)
            write(inewB, 1020) j, xj
         end if
  100 continue

*     Output nonbasic variables that are not at a bound.

      do 250, j = nnJac+1, nb
         if (hs(j) .le. 1 ) then
            xj    = xs(j)
            if (xj .eq. bl(j)) go to 250
            if (xj .eq. bu(j)) go to 250
            if (scaled) xj = xj * aScale(j)
            write(inewB, 1020) j, xj
         end if
  250 continue

*     Terminate the list with a zero.

      j     = 0
      write(inewB, 1020) j
      if (inewB .ne. iPrint) rewind inewB
      if (iPrint .gt. 0) write(iPrint, 1030) inewB, itn
      if (iSumm  .gt. 0) write(iSumm , 1030) inewB, itn
      return

 1000 format(a8, '  ITN', i8, 4x, 3a4, '  NINF', i7,
     $       '      OBJ', 1p, e21.12)
 1005 format('OBJ=',a8, ' RHS=',a8, ' RNG=',a8, ' BND=',a8,
     $       ' M=', i7,  ' N=', i7, ' SB=', i5)
 1010 format(80i1)
 1020 format(i8, 1p, e24.14)
 1030 format(/ ' NEW BASIS file saved on file', i4, '    itn =', i7)

*     end of s4newB
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s4pnch( iPnch, n, nb, hs, bl, xs, Names,
     $                   cw, lencw, iw, leniw )

      implicit           double precision (a-h,o-z)
      integer            hs(nb)
      character*8        Names(nb)
      double precision   bl(nb)
      double precision   xs(nb)

      character*8        cw(lencw)
      integer            iw(leniw)

*     ==================================================================
*     s4pnch  outputs a PUNCH file (list of basis names, states and
*     values) in a format that is compatible with MPS/360.
*
*     15-Nov-91: First version based on Minos 5.4 routine m4pnch.
*     04-Feb-92: Current version.
*     ==================================================================
      character*8        lblank, ColNm, mName, Name
      character*4        key(5)
      parameter         (zero = 0.0d+0)
      data               key  /' LL ', ' UL ', ' SB ', ' XL ', ' XU '/
      data               lblank  /'        '/
*     ------------------------------------------------------------------
      iPrint    = iw( 12)
      iSumm     = iw( 13)
      mName     = cw( 51)

      write(iPnch, 2000) mName
      irow      = n

      do 500,  j = 1, n
         ColNm   = Names(j)
         k       = hs(j)

         if (k .eq. 3) then

*           Basics -- find the next row that isn't basic.

  300       irow   = irow + 1
            if (irow .le. nb) then
               k      = hs(irow)
               if (k .eq. 3) go to 300

               if (k .eq. 2) k = 0
               Name = Names(j)
               write(iPnch, 2100) key(k+4), ColNm, Name, xs(j)
            end if
         else

*           Skip nonbasic variables with zero lower bounds.

            if (k .le. 1) then
               if (bl(j) .eq. zero  .and.  xs(j) .eq. zero) go to 500
            end if
            write(iPnch, 2100) key(k+1), ColNm, lblank, xs(j)
         end if
  500 continue

*     Output superbasic slacks.

      do 700, j = n+1, nb
         if (hs(j) .eq. 2) then
            Name = Names(j)
            write(iPnch, 2100) key(3), Name, lblank, xs(j)
         end if
  700 continue

      write(iPnch , 2200)
      if (iPrint .gt. 0) write(iPrint, 3000) iPnch
      if (iSumm  .gt. 0) write(iSumm , 3000) iPnch
      if (iPnch .ne. iPrint) rewind iPnch
      return

 2000 format('NAME', 10x, a8, 2x, 'PUNCH/INSERT')
 2100 format(a4, a8, 2x, a8, 2x, 1p, e12.5)
 2200 format('ENDATA')
 3000 format(/ ' PUNCH file saved on file', i4)

*     end of s4pnch
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s4rept( ondisk, m, n, nb, nName,
     $                   nnL, nnObj, nS,
     $                   ne, nka, a, ha, ka,
     $                   hs, aScale, bl, bu, 
     $                   gObj, pi, xs, y,
     $                   Names, istate, 
     $                   iw, leniw )

      implicit           double precision (a-h,o-z)
      logical            ondisk
      character*8        Names(nName)
      character*4        istate(3)
      integer            ha(ne), hs(nb)
      integer            ka(nkA)
      integer            iw(leniw)
      double precision   a(ne), aScale(*), bl(nb), bu(nb)
      double precision   gObj(*), pi(m), xs(nb), y(m)

*     ==================================================================
*     s4rept  has the same parameter list as s4soln, the routine that
*     prints the solution.  It will be called if the SPECS file
*     specifies  REPORT file  n  for some positive value of  n.
*
*     pi contains the unscaled dual solution.
*     xs contains the unscaled primal solution.  There are n + m = nb
*        values (n structural variables and m slacks, in that order).
*     y  contains the true slack values for nonlinear constraints
*        in its first nnCon components (computed by s8nslk).
*
*     This version of s4rept does nothing.    Added for PILOT, Oct 1985.
*
*     15-Nov-91: First version based on Minos 5.4 routine m4rept.
*     04-Feb-92: Current version.
*     ==================================================================
      iPrint    = iw( 12)
      iSumm     = iw( 13)

      if (iPrint .gt. 0) write(iPrint, 1000)
      if (iSumm  .gt. 0) write(iSumm , 1000)
      return

 1000 format(/ ' XXX Report file requested.  s4rept does nothing.')

*     end of s4rept
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s4soln( ondisk, minimz, m, n, nb, nName,
     $                   nnObj, nS,
     $                   itn, nInf, sInf, 
     $                   iObj, Objtru, piNorm,
     $                   ne, nka, a, ha, ka,
     $                   hs, aScale, bl, bu,
     $                   gObj, pi, rc, xs, y,
     $                   Names, istate, 
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      logical            ondisk
      character*4        istate(3)
      integer            ha(ne), hs(nb)
      integer            ka(nka)
      character*8        Names(nName)
      double precision   a(ne), bl(nb), bu(nb), xs(nb)
      double precision   aScale(*), gObj(*)
      double precision   rc(nb), pi(m), y(m)

      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     s4soln  is the standard output routine for printing the solution.
*
*     On entry,
*     pi    contains the dual solution.
*     xs    contains the primal solution.  There are n + m = nb values
*           (n structural variables and m slacks, in that order).
*     y     contains the true slack values for nonlinear constraints
*           in its first nnCon components (computed by s8nslk).
*
*     All quantities a, bl, bu, pi, xs, y, fCon, gObj are unscaled,
*     but certain quantities are rescaled locally before tests are
*     applied.   (fCon is not used here.)
*
*     If ondisk is true, the solution is output to the solution file.
*     Otherwise, it is output to the printer.
*
*     15-Nov-91: First version based on Minos 5.4 routine m4soln.
*     26-Jul-96: Slacks modified.
*     21-Feb-98: Current version.
*     ==================================================================
      character*8        cdummy
      character*8        mName, mObj, mRhs, mRng, mBnd 
      character*8        Objtyp(3)
      character*8        id
      double precision   toldj(3)
      logical            feasbl, infsbl, maximz, scaled
      parameter         (zero = 0.0d+0, one = 1.0d+0)
      parameter         (cdummy = '-1111111')
      data               Objtyp /'Max     ', 'Feas    ', 'Min     '/
*     ------------------------------------------------------------------
      iPrint    = iw( 12)
      iSumm     = iw( 13)
      iSoln     = iw(131)

      tolx      = rw( 56)
      plInfy    = rw( 70)
      toldj(1)  = rw(184)
      toldj(2)  = rw(185)
      toldj(3)  = rw(186)

      nnCon     = iw( 21)
      lvlScl    = iw( 75)
      minmax    = iw( 87)

      mName     = cw( 51)
      mObj      = cw( 52)
      mRhs      = cw( 53)
      mRng      = cw( 54)
      mBnd      = cw( 55)

      bplus     = 0.1d+0*plInfy
      scale     = one
      feasbl    = nInf   .eq. 0
      infsbl    = .not. feasbl
      maximz    = minimz .lt. 0
      scaled    = lvlScl .gt. 0
      lpr       = iPrint
      if ( ondisk ) lpr = iSoln

      call s1page( 1, iw, leniw )
      if (infsbl) write(lpr, 1000) mName, nInf, sInf
      if (feasbl) write(lpr, 1002) mName, Objtru
      write(lpr, 1004) istate, itn, nS
      if (mObj .ne. cdummy)
     $write(lpr, 1005) mObj, Objtyp(minmax+2)(1:3), mRhs, mRng, mBnd
      write(lpr, 1010)
      tolFea = tolx
      tolOpt = toldj(3) * pinorm

*     ------------------------------------------------------------------
*     Output the ROWS section.
*     ------------------------------------------------------------------
      do 300, iloop = 1, m
         i      = iloop
         j      = n + i
         if (scaled) scale = aScale(j)
         js     = hs(j)
         b1     = bl(j)
         b2     = bu(j)
         xj     = xs(j)
         py     = pi(i)
         dj     = rc(j)

*        Define the row value and slack activities.
*        The slack activity is the distance of the row value to its
*        nearest bound. (For a free row, it is just the row value).

         if (i .le. nnCon) xj = y(i)
         row    =      xj
         d1     = b1 - xj
         d2     = xj - b2

         slk    =    - d1
         if (abs( d1  )  .gt.  abs( d2 )) slk =  d2
         if (abs( slk )  .ge.  bplus    ) slk = row
         d1     =   d1 / scale
         d2     =   d2 / scale
         djtest = - dj * scale
         if (feasbl) then
            if (   maximz   ) djtest =  - djtest
         end if

         call s4id  ( j, n, nb, nName, Names, id )
         call s4solp( ondisk, iPrint, bplus, tolFea, tolOpt,
     $                js, d1, d2, djtest,
     $                j, id, row, slk, b1, b2, py, i )
  300 continue

*     ------------------------------------------------------------------
*     Output the COLUMNS section.
*     ------------------------------------------------------------------
      call s1page( 1, iw, leniw )
      write(lpr, 1020)

      do 400, jloop = 1, n
         j      = jloop
         if (scaled) scale = aScale(j)
         js     = hs(j)
         b1     = bl(j)
         b2     = bu(j)
         xj     = xs(j)
         cj     = zero
         dj     = rc(j)

         do 320, k = ka(j), ka(j+1) - 1
            ir     = ha(k)
            if (ir .eq. iObj) cj = a(k)
  320    continue

         d1     =   (b1 - xj) / scale
         d2     =   (xj - b2) / scale
         djtest = - dj * scale
         if (feasbl) then
            if (j .le. nnObj) cj     =    cj + gObj(j)
            if (   maximz   ) djtest =  - djtest
         end if

         call s4id  ( j, n, nb, nName, Names, id )
         call s4solp( ondisk, iPrint, bplus, tolFea, tolOpt,
     $                js, d1, d2, djtest,
     $                j, id, xj, cj, b1, b2, dj, m+j )
  400 continue

      if (ondisk) then
         if (iSoln .ne. iPrint) rewind iSoln
         if (iPrint .gt. 0) write(iPrint, 1400) iSoln
         if (iSumm  .gt. 0) write(iSumm , 1400) iSoln
      end if
      return

 1000 format(' Name', 11x, a8, 16x,
     $   ' Infeasibilities', i7, 1p, e16.4)
 1002 format(' Name', 11x, a8, 16x,
     $   ' Objective Value', 1p, e22.10)
 1004 format(/ ' Status', 9x, 3a4, 12x,
     $   ' Iteration', i7, '    Superbasics', i6)
 1005 format(/
     $   ' Objective', 6x, a8, ' (', a3, ')' /
     $   ' RHS      ', 6x, a8 /
     $   ' Ranges   ', 6x, a8 /
     $   ' Bounds   ', 6x, a8)
 1010 format(/ ' Section 1 - Rows' //
     $   '  Number  ...Row.. State  ...Activity...  Slack Activity',
     $   '  ..Lower Limit.  ..Upper Limit.  .Dual Activity    ..i' /)
 1020 format(  ' Section 2 - Columns' //
     $   '  Number  .Column. State  ...Activity...  .Obj Gradient.',
     $   '  ..Lower Limit.  ..Upper Limit.  Reduced Gradnt    m+j' /)
 1400 format(/ ' SOLUTION file saved on file', i4)

*     end of s4soln
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s4solp( ondisk, iPrint, bplus, tolFea, tolOpt,
     $                   js,  d1,  d2, djtest,
     $                   j , id, xj, cj, b1, b2, dj, k )

      implicit           double precision (a-h,o-z)
      character*8        id
      logical            ondisk

*     ------------------------------------------------------------------
*     s4solp  prints one line of the Solution file.
*
*     The following conditions are marked by key:
*
*        D  degenerate basic or superbasic variable.
*        I  infeasible basic or superbasic variable.
*        A  alternative optimum      (degenerate nonbasic dual).
*        N  nonoptimal superbasic or nonbasic (infeasible dual).
*
*     Tests for these conditions are performed on scaled quantities
*     d1, d2, djtest,
*     since the correct indication is then more likely to be given.
*     On badly scaled problems, the unscaled solution may then appear
*     to be flagged incorrectly, but this is just an illusion.
*
*     15-Nov-91: First version based on Minos 5.4 routine m4solp.
*     18-Oct-93: Replaced by modified Minos 5.4 routine m4solp.
*                Infinite bounds and certain other values treated
*                specially.
*     27-Aug-96: Current version.
*     ==================================================================
      character*111      line
      character*4        jstat, jstate(0:5)
      character*1        key, lblank, laltop, ldegen, linfea, lnotop
      character*16       lzero, lone, lmone
      parameter        ( zero = 0.0d+0,  one = 1.0d+0 )
      data               jstate /' LL ', ' UL ', 'SBS ',
     $                           ' BS' , ' EQ' , ' FR '/
      data               lblank /' '/, lAltOp /'A'/, ldegen /'D'/,
     $                   linfea /'I'/, lNotOp /'N'/
      data               lzero  /'          .     '/
      data               lone   /'         1.0    '/
      data               lmone  /'        -1.0    '/
*     ------------------------------------------------------------------
      key    = lblank
      if (js .eq. -1) js = 2

      if (js .le.  1) then

*        Set key for nonbasic variables.

         if (b1 .eq. b2) js = 4
         if (- d1 .gt. tolFea  .and.  - d2 .gt. tolFea) js = 5
         if (js .eq. 1 ) djtest = - djtest
         if (js .ge. 4 ) djtest =   abs( djtest )
         if (             abs( djtest ) .le. tolOpt) key = lAltOp
         if (js .ne. 4  .and.  djtest   .gt. tolOpt) key = lNotOp
      else

*        Set key for basic and superbasic variables.

         if (abs(   d1   ) .le. tolFea  .or.
     $       abs(   d2   ) .le. tolFea) key = ldegen
         if (           js .eq. 2       .and.
     $       abs( djtest ) .gt. tolOpt) key = lNotOp
         if (           d1 .gt. tolFea  .or.
     $                  d2 .gt. tolFea) key = linfea
      end if

*     Select format for printing.

      jstat   = jstate(js)
      if (ondisk) then
         write(line, 1000) j, id, key, jstat, xj,cj, b1,b2, dj,k
      else
         if (b2 .lt. bplus) then
            if (b1 .gt. - bplus) then
               write(line, 1200) j,id,key,jstat,xj,cj,b1,b2,dj,k
            else
               write(line, 1300) j,id,key,jstat,xj,cj,   b2,dj,k
            end if
         else
            if (b1 .gt. - bplus) then
               write(line, 1400) j,id,key,jstat,xj,cj,b1,   dj,k
            else
               write(line, 1500) j,id,key,jstat,xj,cj,      dj,k
            end if
         end if
      end if

*     Test for 0.0, 1.0 and -1.0

      if (xj .eq. zero) line(25:40) = lzero
      if (xj .eq.  one) line(25:40) = lone
      if (xj .eq. -one) line(25:40) = lmone
      if (cj .eq. zero) line(41:56) = lzero
      if (cj .eq.  one) line(41:56) = lone
      if (cj .eq. -one) line(41:56) = lmone
      if (b1 .eq. zero) line(57:72) = lzero
      if (b1 .eq.  one) line(57:72) = lone
      if (b1 .eq. -one) line(57:72) = lmone
      if (b2 .eq. zero) line(73:88) = lzero
      if (b2 .eq.  one) line(73:88) = lone
      if (b2 .eq. -one) line(73:88) = lmone
      if (dj .eq. zero) line(89:104)= lzero
      if (dj .eq.  one) line(89:104)= lone
      if (dj .eq. -one) line(89:104)= lmone

      write(iPrint, 2000) line
      return

 1000 format(i8, 2x, a8, 1x, a1, 1x, a3, 1p, 5e16.6, i7)
 1200 format(i8, 2x, a8, 1x, a1, 1x, a3, 5f16.5, i7)
 1300 format(i8, 2x, a8, 1x, a1, 1x, a3, 2f16.5,
     $   '           None ', f16.5, f16.5, i7)
 1400 format(i8, 2x, a8, 1x, a1, 1x, a3, 2f16.5,
     $   f16.5, '           None ', f16.5, i7)
 1500 format(i8, 2x, a8, 1x, a1, 1x, a3, 2f16.5,
     $   '           None ', '           None ', f16.5, i7)
 2000 format(a)

*     end of s4solp
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s4stat( k, istate )

      integer            k
      character*4        istate(3)

*     ==================================================================
*     s4stat loads istate(*) with words describing the current state.
*
*     15-Nov-91: First version based on Minos 5.4 routine m4stat.
*     04-Feb-92: Current version.
*     ==================================================================
      character*4        c(18)
      data               c /'Proc', 'eedi', 'ng  ',
     $                      'Opti', 'mal ', 'Soln',
     $                      'Infe', 'asib', 'le  ',
     $                      'Unbo', 'unde', 'd   ',
     $                      'Exce', 'ss i', 'tns ',
     $                      'Erro', 'r co', 'ndn '/
*     ------------------------------------------------------------------
      j      = 3*min( k, 5 )
      do 10  i  = 1, 3
         istate(i) = c(j+i)
   10 continue

*     end of s4stat
      end
