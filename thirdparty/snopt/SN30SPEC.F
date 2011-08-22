*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
*     file  sn30spec.f
*
*     s3optc   s3opti   s3optr
*     s3opt    s3file   s3fils   s3undf   s3keys   s3ties
*     oplook   opnumb   opscan   optokn   opuppr
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s3optc( set, cwork, cvalue )  

      implicit           double precision (a-h,o-z)
      logical            set
      character*8        cwork, cvalue

*     ==================================================================
*     s3optc  sets cwork to cvalue or vice versa depending on the value
*     of set.
*
*     17 May 1998: First version of s3optc.
*     17 May 1998: Current version.
*     ==================================================================

      if ( set ) then
         cwork  = cvalue
      else
         cvalue = cwork
      end if

*     end of s3optc
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s3opti( set, iwork, ivalue )  

      implicit           double precision (a-h,o-z)
      logical            set
      integer            iwork, ivalue

*     ==================================================================
*     s3opti  sets iwork to ivalue or vice versa depending on the value
*     of set.
*
*     17 May 1998: First version of s3opti.
*     17 May 1998: Current version.
*     ==================================================================

      if ( set ) then
         iwork  = ivalue
      else
         ivalue = iwork
      end if

*     end of s3opti
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s3optr( set, rwork, rvalue )  

      implicit           double precision (a-h,o-z)
      logical            set

*     ==================================================================
*     s3optr  sets rwork to rvalue or vice versa depending on the value
*     of set.
*
*     17 May 1998: First version of s3optr.
*     17 May 1998: Current version.
*     ==================================================================

      if ( set ) then
         rwork  = rvalue
      else
         rvalue = rwork
      end if

*     end of s3optr
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s3opt ( s, buffer, key, c, i, r,
     $                   lPrnt, lSumm, inform,
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      logical            s
      character*(*)      buffer, key
      character*8        c
      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     s3opt  decodes the option contained in  buffer  in order to
*     set or get a parameter value in the relevant array iw or rw.
*
*     The buffer is output to file iPrint, minus trailing blanks.
*     Error messages are output to files iPrint and iSumm.
*     buffer is echoed to iPrint but normally not to iSumm.
*     It is echoed to iSumm before any error msg.
*
*     On entry,
*     buffer contains the option string to be processed. 
*     s      is true if an option is to be extracted from buffer.
*            Otherwise, c, i and r are to be assigned the value of the 
*            option defined in the option string.
*     lPrnt  is iPrint as given to s3file.
*     lSumm  is iSumm  as given to s3file.
*     inform is the number of errors so far.
*
*     On exit,
*     key    is the first keyword contained in buffer.
*     If s is true, c, i and r may be ignored.  (They are usually
*            option values that have been saved in cw, iw, rw.)
*     If s is false,
*     c      is the OBJECTIVE, RHS, RANGE or BOUND name if key is
*            one of those words.
*     r      is the first numerical value found in buffer (or zero).  
*     i      is int(r) if  abs(r) < maxint.
*     inform is the number of errors so far.
*            
*
*     s3opt  uses opnumb and the subprograms
*                 lookup, scannr, tokens, upcase
*     (now called oplook, opscan, optokn, opuppr)
*     supplied by Sterling Software, Palo Alto, California.
*
*     15 Nov 1991: First version based on s3key/opkey.
*     17 May 1998: Current version.
*     ==================================================================
      external           opnumb
      logical            opnumb, more, number

      parameter         (      maxtok = 10)
      character*16       token(maxtok)
      character*16       key2, value

      parameter         (maxint = 10000000)
      parameter         (zero   =   0.0d+0)

*     maxint above should be larger than any expected integer value.

      integer            tolFP , tolQP , tolNLP, tolx  , tolCon,
     $                   tolpiv, tCrash, tolswp, tolfac,
     $                   tolupd, plInfy, bigfx , bigdx , epsrf ,
     $                   fdint1, fdint2, xdlim , vilim , eta   ,
     $                   wtInf0, xPen0 , scltol, Aijtol,
     $                   bStrc1, bStrc2,
     $                   Utol1 , Utol2 , Dens2

      parameter         (tolFP     =  51)
      parameter         (tolQP     =  52)
      parameter         (tolNLP    =  53)
      parameter         (tolx      =  56)
      parameter         (tolCon    =  57)
      parameter         (tolpiv    =  60)
      parameter         (tCrash    =  62)
      parameter         (tolswp    =  65)
      parameter         (tolfac    =  66)
      parameter         (tolupd    =  67)
      parameter         (plInfy    =  70)
      parameter         (bigfx     =  71)
      parameter         (bigdx     =  72)
      parameter         (epsrf     =  73)
      parameter         (fdint1    =  76)
      parameter         (fdint2    =  77)
      parameter         (xdlim     =  80)
      parameter         (vilim     =  81)
      parameter         (eta       =  84)
      parameter         (wtInf0    =  88)
      parameter         (xPen0     =  89)
      parameter         (scltol    =  92)
      parameter         (Aijtol    =  95)
      parameter         (bStrc1    =  96)
      parameter         (bStrc2    =  97)
      parameter         (Utol1     = 154)
      parameter         (Utol2     = 155)
      parameter         (Dens2     = 158)
                          
      parameter         (maxru     =   2)
      parameter         (maxrw     =   3)
      parameter         (maxiu     =   4)
      parameter         (maxiw     =   5)
      parameter         (maxcu     =   6)
      parameter         (maxcw     =   7)
      parameter         (iPrint    =  12)
      parameter         (iSumm     =  13)
      parameter         (nnCon     =  21)
      parameter         (nnJac     =  22)
      parameter         (nnObj     =  23)
      parameter         (nnH       =  27)
      parameter         (lEmode    =  51)
      parameter         (lHtype    =  53)
      parameter         (maxR      =  56)
      parameter         (maxS      =  57)
      parameter         (kchk      =  60)
      parameter         (kfac      =  61)
      parameter         (ksav      =  62)
      parameter         (klog      =  63)
      parameter         (kSumm     =  64)
      parameter         (kDegen    =  65)
      parameter         (mQNmod    =  66)
      parameter         (kReset    =  67)
      parameter         (mFlush    =  68)
      parameter         (lvlSrt    =  70)
      parameter         (lvlDer    =  71)
      parameter         (lvlExi    =  72)
      parameter         (lvlInf    =  73)
      parameter         (lvlPrt    =  74)
      parameter         (lvlScl    =  75)
      parameter         (lvlSch    =  76)
      parameter         (lvlTim    =  77)
      parameter         (lvlVer    =  78)
      parameter         (lprDbg    =  80)
      parameter         (lprPrm    =  81)
      parameter         (lprSch    =  82)
      parameter         (lprScl    =  83)
      parameter         (lprSol    =  84)
      parameter         (minmax    =  87)
      parameter         (iCrash    =  88)
      parameter         (itnlim    =  89)
      parameter         (mMajor    =  90)
      parameter         (mMinor    =  91)
      parameter         (MjrPrt    =  92)
      parameter         (MnrPrt    =  93)
      parameter         (nParPr    =  94)
      parameter         (jverf1    =  98)
      parameter         (jverf2    =  99)
      parameter         (jverf3    = 100)
      parameter         (jverf4    = 101)
      parameter         (lDenJ     = 105)
      parameter         (mEr       = 106)
      parameter         (mLst      = 107)
      parameter         (nProb     = 108)
      parameter         (iBack     = 120)
      parameter         (iDump     = 121)
      parameter         (iLoadB    = 122)
      parameter         (iMPS      = 123)
      parameter         (iNewB     = 124)
      parameter         (iInsrt    = 125)
      parameter         (iOldB     = 126)
      parameter         (iPnch     = 127)
      parameter         (iReprt    = 130)
      parameter         (iSoln     = 131)
      parameter         (maxm      = 133)
      parameter         (maxn      = 134)
      parameter         (maxne     = 135)

      parameter         (mObj      =  52)
      parameter         (mRhs      =  53)
      parameter         (mRng      =  54)
      parameter         (mBnd      =  55)

*     ------------------------------------------------------------------
*     Set lenbuf = length of buffer without trailing blanks.
*     Echo to the print file.
*     ------------------------------------------------------------------
      lenbuf  = 1
      do 10 j = 1, len(buffer)
         if (buffer(j:j) .ne. ' ') lenbuf = j
   10 continue

      if (lPrnt .gt. 0) then
         write(lPrnt, '(6x, a)') buffer(1:lenbuf)
      end if

*     Set lenb = length of buffer without trailing comments.
*     Eliminate comments and empty lines.
*     A '*' appearing anywhere in buffer terminates the string.

      i      = index( buffer(1:lenbuf), '*' )
      if (i .eq. 0) then
         lenb = lenbuf
      else
         lenb = i - 1
      end if
      if (lenb .le. 0) then
         key = '*'
         go to 900
      end if

*     ------------------------------------------------------------------
*     Extract up to maxtok tokens from the record.
*     ntoken returns how many were actually found.
*     key, key2, are the first tokens if any, otherwise blank.
*     For some values of key (bounds, objective, ranges, rhs)
*     we have to save key2 before s3tie (and oplook) alter it.
*     For example, if the data is     objective = obj
*     oplook will change obj to objective.
*     ------------------------------------------------------------------
      ntoken = maxtok
      call optokn( buffer(1:lenbuf), ntoken, token )
      key    = token(1)
      key2   = token(2)
      c      = key2(1:8)

*     Certain keywords require no action.

      if (key .eq. '   ') go to 900
      if (key .eq. 'END') go to 900

*     Convert the keywords to their most fundamental form
*     (upper case, no abbreviations).
*     loci   says where the keywords are in the dictionaries.
*     loci = 0 signals that the keyword wasn't there.

      call s3key ( key , loc1 )
      call s3tie ( key2, loc2 )

*     Most keywords will have an associated integer or real value,
*     so look for it no matter what the keyword.

      c      = key2
      i      = 1
      number = .false.

   50 if (i .lt. ntoken  .and.  .not. number) then
         i      = i + 1
         value  = token(i)
         number = opnumb( value )
         go to 50
      end if

      i = 0
      r = zero
      if ( number ) then
         read  (value, '(bn, e16.0)') r
         if (abs(r) .lt. maxint) i = int(r)
      end if

*     ------------------------------------------------------------------
*     Decide what to do about each keyword.
*     The second keyword (if any) might be needed to break ties.
*     Some seemingly redundant testing of more is used
*     to avoid compiler limits on the number of consecutive else ifs.
*     ------------------------------------------------------------------
      m1     = -1
      i0     =  0
      i1     =  1
      i2     =  2
      i3     =  3
      i4     =  4
      more   = .true.

      if (more) then
         more   = .false.
         if      (key .eq. 'BACKUP      ') then
            call s3opti(s, iw(iBack ), i)  
         else if (key .eq. 'CENTRAL     ') then
            call s3optr(s, rw(fdint2), r)
         else if (key .eq. 'CHECK       ') then
            call s3opti(s, iw(kchk  ), i)  
         else if (key .eq. 'COLD        ') then
            call s3opti(s, iw(lvlSrt),i0)

         else if (key .eq. 'CRASH       ') then
              if (loc2.eq.  0            ) go to 820
              if (key2.eq. 'OPTION      ') call s3opti(s, iw(iCrash), i)
              if (key2.eq. 'TOLERANCE   ') call s3optr(s, rw(tCrash), r)

         else if (key .eq. 'DEBUG       ') then
            call s3opti(s, iw(lprDbg), i)
         else if (key .eq. 'DEFAULTS    ') then
            call s3undf( cw, lencw, iw, leniw, rw, lenrw )

         else if (key .eq. 'DERIVATIVE  ') then
              if (loc2.eq.  0            ) go to 820
              if (key2.eq. 'LEVEL       ') call s3opti(s, iw(lvlDer), i)
              if (key2.eq. 'LINESEARCH  ') call s3opti(s, iw(lvlSch),i1)

         else if (key .eq. 'DIFFERENCE  ') then
            call s3optr(s, rw(fdint1), r)
         else if (key .eq. 'DUMP        ') then
            call s3opti(s, iw(iDump ), i)

         else if (key .eq. 'ELASTIC     ') then
              if (loc2.eq.  0            ) go to 820
              if (key2.eq. 'WEIGHT      ') call s3optr(s, rw(wtInf0), r)
              if (key2.eq. 'OBJECTIVE   ') call s3opti(s, iw(lvlInf), i)
              if (key2.eq. 'MODE        ') call s3opti(s, iw(lEmode), i)

         else if (key .eq. 'EXPAND      ') then
            call s3opti(s, iw(kDegen), i)
         else if (key .eq. 'FACTORIZATION') then
            call s3opti(s, iw(kfac  ), i)

         else if (key .eq. 'FEASIBLE    ') then
              if (loc2.eq.  0            ) go to 820
              if (key2.eq. 'POINT       ') call s3opti(s, iw(minmax),i0)
              if (key2.eq. 'EXIT        ') call s3opti(s, iw(lvlExi),i1)

         else if (key .eq. 'FEASIBILITY ') then
              if (loc2.eq.  0            ) go to 820
              if (key2.eq. 'TOLERANCE   ') call s3optr(s, rw(tolx  ), r)

         else if (key .eq. 'FUNCTION    ') then
            call s3optr(s, rw(epsrf ), r)

         else if (key .eq. 'HESSIAN     ') then
              if (loc2.eq.  0            ) go to 820
              if (key2.eq. 'COLUMNS     ') call s3opti(s, iw(nnH   ), i)
              if (key2.eq. 'DIMENSION   ') call s3opti(s, iw(maxR  ), i)
              if (key2.eq. 'FREQUENCY   ') call s3opti(s, iw(kReset), i)
              if (key2.eq. 'FLUSH       ') call s3opti(s, iw(mFlush), i)
              if (key2.eq. 'UPDATES     ') call s3opti(s, iw(mQNmod), i)
              if (key2.eq. 'LIMITED     ') call s3opti(s, iw(lHtype),i1)
              if (key2.eq. 'FULL        ') call s3opti(s, iw(lHtype),i2)
              if (key2.eq. 'DIFFERENCES ') call s3opti(s, iw(lHtype),i3)! coming soon
              if (key2.eq. 'NEWTON      ') call s3opti(s, iw(lHtype),i4)! coming soon

         else if (key .eq. 'INFEASIBLE  ') then
            call s3opti(s, iw(lvlExi),i0)
         else if (key .eq. 'INFINITE    ') then
            call s3optr(s, rw(plInfy), r)
         else if (key .eq. 'INSERT      ') then
            call s3opti(s, iw(iInsrt), i)
         else if (key .eq. 'ITERATIONS  ') then
            call s3opti(s, iw(itnlim), i)
         else
            more   = .true.
         end if
      end if

      if (more) then
         more   = .false.
         if      (key .eq. 'IW          ') then
*           Allow things like  iw 21 = 100  to set iw(21) = 100
            key2   = token(3)
            if (i .ge. 1  .and. i .le. 500) then
               read (key2, '(bn, i16)') iw(i)
            else
               go to 880
            end if

         else if (key .eq. 'LINESEARCH  ') then
              if (loc2.eq.  0            ) go to 820
              if (key2.eq. 'TOLERANCE   ') call s3optr(s, rw(eta   ), r)
              if (key2.eq. 'DEBUG       ') call s3opti(s, iw(lprSch), i)

         else if (key .eq. 'LOAD        ') then
            call s3opti(s, iw(iLoadB), i)
         else if (key .eq. 'LOG         ') then
            call s3opti(s, iw(klog)  , i)

         else if (key .eq. 'LP          ') then
              if (loc2.eq.  0            ) go to 820
              if (key2.eq. 'FEASIBILITY ') call s3optr(s, rw(tolx  ), r)
              if (key2.eq. 'OPTIMALITY  ') call s3optr(s, rw(tolQP ), r)

         else if (key .eq. 'LU          ') then
              if (loc2.eq.  0            ) go to 820
              if (key2.eq. 'FACTORIZATION')call s3optr(s, rw(tolFac), r)
              if (key2.eq. 'UPDATES     ') call s3optr(s, rw(tolUpd), r)
              if (key2.eq. 'DENSITY     ') call s3optr(s, rw(Dens2 ), r)
              if (key2.eq. 'SINGULARITY ') then
                 call s3optr(s, rw(Utol1), r)
                 call s3optr(s, rw(Utol2), r)
              end if

              if (key2.eq. 'SWAP        ') call s3optr(s, rw(tolswp), r)
         else
            more   = .true.
         end if
      end if

      if (more) then
         more   = .false.
         if      (key .eq. 'MAJOR       ') then
              if (loc2.eq.  0            ) go to 820
              if (key2.eq. 'FEASIBILITY ') call s3optr(s, rw(tolCon), r)
              if (key2.eq. 'ITERATIONS  ') call s3opti(s, iw(mMajor), i)
              if (key2.eq. 'OPTIMALITY  ') call s3optr(s, rw(tolNLP), r)
              if (key2.eq. 'PRINT       ') call s3opti(s, iw(MjrPrt), i)
              if (key2.eq. 'STEP        ') call s3optr(s, rw(xdlim ), r)

         else if (key .eq. 'MAXIMIZE    ') then
            call s3opti(s, iw(minmax),m1)
         else if (key .eq. 'MINIMIZE    ') then
            call s3opti(s, iw(minmax),i1)

         else if (key .eq. 'MINOR       ') then
              if (loc2.eq.  0            ) go to 820
              if (key2.eq. 'ITERATIONS  ') call s3opti(s, iw(mMinor), i)
              if (key2.eq. 'FEASIBILITY ') call s3optr(s, rw(tolx  ), r)
              if (key2.eq. 'OPTIMALITY  ') call s3optr(s, rw(tolQP ), r)
              if (key2.eq. 'PHASE1      ') call s3optr(s, rw(tolFP ), r)
              if (key2.eq. 'PHASE2      ') call s3optr(s, rw(tolQP ), r)
              if (key2.eq. 'PRINT       ') call s3opti(s, iw(MnrPrt), i)

         else if (key .eq. 'NEW         ') then
            call s3opti(s, iw(iNewB ), i)

         else if (key .eq. 'NO          '  .or.
     $            key .eq. 'NON         ') then
            call s3opti(s, iw(lvlSch),i0)

         else if (key .eq. 'OLD         ') then
            call s3opti(s, iw(iOldB ), i)
         else if (key .eq. 'OPTIMALITY  ') then
            call s3optr(s, rw(tolNLP), r)
         else if (key .eq. 'PARTIAL     ') then
            call s3opti(s, iw(nParPr), i)
         else if (key .eq. 'PENALTY     ') then
            call s3optr(s, rw(xPen0 ), r)
         else if (key .eq. 'PIVOT       ') then
            call s3optr(s, rw(tolpiv), r)

         else if (key .eq. 'PRINT       ') then
              if (loc2.eq.  0            ) go to 820
              if (key2.eq. 'FILE        ') call s3opti(s, iw(iPrint), i)
              if (key2.eq. 'FREQUENCY   ') call s3opti(s, iw(klog  ), i)
              if (key2.eq. 'LEVEL       ') call s3opti(s, iw(lvlPrt), i)

         else if (key .eq. 'PUNCH       ') then
            call s3opti(s, iw(iPnch ), i)
         else
            more   = .true.
         end if
      end if

      if (more) then
         more   = .false.
         if      (key .eq. 'REPORT      ') then
            call s3opti(s, iw(iReprt), i)
         else if (key .eq. 'QP          ') then
              if (loc2.eq.  0            ) go to 820
              if (key2.eq. 'COLUMNS     ') call s3opti(s, iw(nnH  ), i)
              if (key2.eq. 'FEASIBILITY ') call s3optr(s, rw(tolx ), r)
              if (key2.eq. 'OPTIMALITY  ') call s3optr(s, rw(tolQP), r)

         else if (key .eq. 'REDUCED     ') then
            call s3opti(s, iw(maxR  ), i)

         else if (key .eq. 'ROWS        ') then
*             gams should recognize row tolerance
*             but not just          rows
*             This is a relic from MINOS
              if (key2.eq. 'TOLERANCE   ') then
                 call s3optr(s, rw(tolCon), r)
              else
                 call s3opti(s, iw(maxm  ), i)
              end if

         else if (key .eq. 'RW          ') then
*           allow things like rw 21 = 2  to set rw(21) = 2.0
            key2   = token(3)
            if (i .ge. 1  .and. i .le. 500) then
               read (key2, '(bn, e16.0)') rw(i)
            else
               go to 880
            end if

         else if (key .eq. 'SAVE        ') then
            call s3opti(s, iw(ksav  ), i)

         else if (key .eq. 'SCALE       ') then
              if (loc2.eq.  0            ) go to 820
              if (key2.eq. 'OPTION      ') call s3opti(s, iw(lvlScl), i)
              if (key2.eq. 'TOLERANCE   ') call s3optr(s, rw(scltol), r)
              if (key2.eq. 'PRINT       ') call s3opti(s, iw(lprScl),i1)
         else
            more   = .true.
         end if
      end if

      if (more) then
         more   = .false.
         if      (key .eq. 'SOLUTION    ') then
              if (loc2.eq.  0            ) go to 820
              if (key2.eq. 'FILE        ') call s3opti(s, iw(iSoln ), i)
              if (key2.eq. 'YES         ') call s3opti(s, iw(lprSol),i2)
              if (key2.eq. 'NO          ') call s3opti(s, iw(lprSol),i0)

         else if (key .eq. 'START       ') then
              if (key2.eq. 'OBJECTIVE   ') call s3opti(s, iw(jverf1), i)
              if (key2.eq. 'CONSTRAINTS ') call s3opti(s, iw(jverf3), i)
              if (loc2.eq.  0            ) go to 840
         else if (key .eq. 'STOP        ') then
              if (key2.eq. 'OBJECTIVE   ') call s3opti(s, iw(jverf2), i)
              if (key2.eq. 'CONSTRAINTS ') call s3opti(s, iw(jverf4), i)
              if (loc2.eq.  0            ) go to 820
         else
            more   = .true.
         end if
      end if

      if (more) then
         more   = .false.
         if      (key .eq. 'SUPERBASICS ') then
            call s3opti(s, iw(maxS  ), i)
         else if (key .eq. 'SUMMARY     ') then
              if (loc2.eq.  0            ) go to 820
              if (key2.eq. 'FILE        ') call s3opti(s, iw(iSumm ), i)
              if (key2.eq. 'FREQUENCY   ') call s3opti(s, iw(ksumm ), i)

         else if (key .eq. 'SUPPRESS    ') then
            call s3opti(s, iw(lprPrm), i)
         else if (key .eq. 'TIMING      ') then
            call s3opti(s, iw(lvlTim), i)

         else if (key .eq. 'TOTAL       ') then
              if (loc2.eq.  0            ) go to 820
              if (key2.eq. 'INTEGER     ') call s3opti(s, iw(maxiw ), i)
              if (key2.eq. 'REAL        ') call s3opti(s, iw(maxrw ), i)
              if (key2.eq. 'CHARACTER   ') call s3opti(s, iw(maxcw ), i)

         else if (key .eq. 'UNBOUNDED   ') then
              if (loc2.eq.  0            ) go to 820
              if (key2.eq. 'OBJECTIVE   ') call s3optr(s, rw(bigfx ), r)
              if (key2.eq. 'STEP        ') call s3optr(s, rw(bigdx ), r)

         else if (key .eq. 'USER        ') then
              if (loc2.eq.  0            ) go to 820
              if (key2.eq. 'INTEGER     ') call s3opti(s, iw(maxiu ), i)
              if (key2.eq. 'REAL        ') call s3opti(s, iw(maxru ), i)
              if (key2.eq. 'CHARACTER   ') call s3opti(s, iw(maxcu ), i)

         else if (key .eq. 'VERIFY      ') then
              if (key2.eq. '            ') then
                 loc2   = 1
                 i      = 3
              end if
              if (loc2.eq.  0            ) go to 820
              if (key2.eq. 'OBJECTIVE   ') i = 1
              if (key2.eq. 'CONSTRAINTS ') i = 2
              if (key2.eq. 'GRADIENTS   ') i = 3
              if (key2.eq. 'YES         ') i = 3
              if (key2.eq. 'NO          ') i = 0
              if (key2.eq. 'LEVEL       ') i = i
              call s3opti(s, iw(lvlVer), i)

         else if (key .eq. 'VIOLATION   ') then
            call s3optr(s, rw(vilim ), r)
         else if (key .eq. 'WARM        ') then
            call s3opti(s, iw(lvlSrt),i1)

         else if (key .eq. 'WORKSPACE   ') then
              if (loc2.eq.  0            ) go to 820
              if (key2.eq. '(USER)      ') call s3opti(s, iw(maxru), i)
              if (key2.eq. '(TOTAL)     ') call s3opti(s, iw(maxrw), i)
         else
            more   = .true.
         end if
      end if

      if (.not. more) go to 900

*     ------------------------------------------------------------------
*     Keywords for MPS files.
*     ------------------------------------------------------------------

      if (more) then
         more   = .false.
         if      (key .eq. 'AIJ         ') then
            call s3optr(s, rw(Aijtol), r)
         else if (key .eq. 'BOUNDS      ') then
            call s3optc(s, cw(mBnd  ), c)
         else if (key .eq. 'COEFFICIENTS') then
            call s3opti(s, iw(maxne ), i)
         else if (key .eq. 'COLUMNS     ') then
            call s3opti(s, iw(maxn  ), i)
         else if (key .eq. 'ELEMENTS    ') then
            call s3opti(s, iw(maxne ), i)
         else if (key .eq. 'ERROR       ') then
            call s3opti(s, iw(mEr   ), i)
         else if (key .eq. 'INFINITE    ') then
            call s3optr(s, rw(plInfy), r)

         else if (key .eq. 'JACOBIAN    ') then
              if (loc2.eq.  0            ) go to 820
              if (key2.eq. 'DENSE       ') call s3opti(s, iw(lDenJ ),i1)
              if (key2.eq. 'SPARSE      ') call s3opti(s, iw(lDenJ ),i2)

         else if (key .eq. 'LIST        ') then
            call s3opti(s, iw(mLst  ), i)
         else if (key .eq. 'LOWER       ') then
            call s3optr(s, rw(bStrc1), r)
         else if (key .eq. 'MPS         ') then
            call s3opti(s, iw(iMPS  ), i)

         else if (key .eq. 'NONLINEAR   ') then
              if (loc2.eq.  0            ) go to 820
              if (key2.eq. 'CONSTRAINTS ') call s3opti(s, iw(nnCon ), i)
              if (key2.eq. 'OBJECTIVE   ') call s3opti(s, iw(nnObj ), i)
              if (key2.eq. 'JACOBIAN    ') call s3opti(s, iw(nnJac ), i)
              if (key2.eq. 'VARIABLES   ') then
                 call s3opti(s, iw(nnObj), i)
                 call s3opti(s, iw(nnJac), i)
              end if

         else if (key .eq. 'OBJECTIVE   ') then
            call s3optc(s, cw(mObj  ), c)
         else if (key .eq. 'PROBLEM     ') then
            call s3opti(s, iw(nProb ), i)
         else if (key .eq. 'RANGES      ') then
            call s3optc(s, cw(mRng  ), c)
         else if (key .eq. 'RHS         ') then
            call s3optc(s, cw(mRhs  ), c)
         else if (key .eq. 'UPPER       ') then
            call s3optr(s, rw(Bstrc2), r)
         else
            more   = .true.
         end if
      end if

      if (.not. more) go to 900

*     ------------------------------------------------------------------
*     Error messages.
*     ------------------------------------------------------------------
  800 inform = inform + 1
      if (lPrnt .gt. 0) then
         write (lPrnt, 2300) key
      end if
      if (lSumm .gt. 0) then
         write (lSumm, '(1x, a )') buffer
         write (lSumm, 2300) key
      end if
      go to 900

  820 inform = inform + 1
      if (lPrnt .gt. 0) then
         write (lPrnt, 2320) key2
      end if
      if (lSumm .gt. 0) then
         write (lSumm, '(1x, a )') buffer
         write (lSumm, 2320) key2
      end if
      go to 900

  840 inform = inform + 1
      if (lPrnt .gt. 0) then
         write (lPrnt, 2340) key2
      end if
      if (lSumm .gt. 0) then
         write (lSumm, '(1x, a )') buffer
         write (lSumm, 2340) key2
      end if
      go to 900

  880 inform = inform + 1
      if (lPrnt .gt. 0) then
         write (lPrnt, 2380) i
      end if
      if (lSumm .gt. 0) then
         write (lSumm, '(1x, a )') buffer
         write (lSumm, 2380) i
      end if

  900 return

 2300 format(' XXX  Keyword not recognized:         ', a)
 2320 format(' XXX  Second keyword not recognized:  ', a)
*2330 format(' XXX  Third  keyword not recognized:  ', a)
 2340 format(' XXX  Fourth keyword not recognized:  ', a)
 2380 format(' XXX  The parm subscript is out of range:', i10)

*     end of s3opt
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s3file( ncalls, iOptns, opset,
     $                   iPrint, iSumm, inform,
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      integer            ncalls, iOptns, iPrint, iSumm, inform
      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)
      external           opset

*     ==================================================================
*     s3file  reads the options file from unit  iOptns  and loads the
*     relevant options, using opset to process each line.
*
*     On exit, inform says how many errors were encountered.
*
*     15-Nov 1991: First version based on Minos/Npsol routine s3file.
*     17 May 1998: Current version.
*     ==================================================================
      character*16       key   , token(1)
      character*72       buffer
      character*8        cvalue

*     lPrnt and lSumm and local copies of iPrint and iSumm
*     (which might get changed by s3opt).


      lPrnt  = iPrint
      lSumm  = iSumm
      inform = 0
  
*     Return if the unit number is out of range.
      
      if (iOptns .lt. 0  .or.  iOptns .gt. 99) then
         inform = 1
         return
      end if

*     ------------------------------------------------------------------
*     Look for  Begin, Endrun  or  Skip.
*     ------------------------------------------------------------------
      nread  = 0
   50    read (iOptns, '(a72)', end = 930) buffer



         nread = nread + 1
         nkey  = 1
         call optokn( buffer, nkey, token )
         key   = token(1)
         if (key .eq. 'ENDRUN') go to 940
         if (key .ne. 'BEGIN' ) then
            if (nread .eq. 1  .and.  key .ne. 'SKIP') then
               inform = inform + 1
               if (lPrnt .gt. 0) write (lPrnt, 2000) iOptns, buffer
               if (lSumm .gt. 0) write (lSumm, 2000) iOptns, buffer
            end if
            go to 50
         end if

*     ------------------------------------------------------------------
*     Begin found.
*     This is taken to be the first line of an OPTIONS file.
*     It is printed without the trailing blanks.
*     ------------------------------------------------------------------
      call s1page( 1, iw, leniw )
      lenbuf   = 1
      do 10, j = 1, len(buffer)
         if (buffer(j:j) .ne. ' ') lenbuf = j
   10 continue

      if (lPrnt .gt. 0) then
         write (lPrnt, '(/(1x, a))') 'OPTIONS file', '------------'
         write (lPrnt, '(  6x, a)') buffer(1:lenbuf)
      end if
      if (lSumm .gt. 0) then
         write (lSumm, '(/ 1x, a)') buffer(1:lenbuf)
      end if

*     ------------------------------------------------------------------
*     Read the rest of the file.
*     ------------------------------------------------------------------
*+    while (key .ne. 'END') loop
  100 if    (key .ne. 'END') then
         read  (iOptns, '(a72)', end = 920) buffer
         call opset ( .true., buffer, key, cvalue, ivalue, rvalue,
     $                lPrnt, lSumm, inform,
     $                cw, lencw, iw, leniw, rw, lenrw )
         go to 100
      end if
*+    end while

      return

  920 if (lPrnt .gt. 0) write (lPrnt, 2200) iOptns
      if (lSumm .gt. 0) write (lSumm, 2200) iOptns
      inform = 2
      return

  930 if (ncalls .le. 1) then
         if (lPrnt .gt. 0) write (lPrnt, 2300) iOptns
         if (lSumm .gt. 0) write (lSumm, 2300) iOptns
      else
         if (lPrnt .gt. 0) write (lPrnt, '(a)') ' Endrun'
         if (lSumm .gt. 0) write (lSumm, '(a)') ' Endrun'
      end if
      inform = 3
      return

  940 if (lPrnt .gt. 0) write (lPrnt, '(/ 6x, a)') buffer
      if (lSumm .gt. 0) write (lSumm, '(/ 1x, a)') buffer
      inform = 4
      return

 2000 format(
     $ //' XXX  Error while looking for an OPTIONS file on unit', I7
     $ / ' XXX  The file should start with Begin, Skip or Endrun'
     $ / ' XXX  but the first record found was the following:'
     $ //' ---->', a
     $ //' XXX  Continuing to look for OPTIONS file...')
 2200 format(//' XXX  End-of-file encountered while processing',
     $         ' an OPTIONS file on unit', I6)
 2300 format(//' XXX  End-of-file encountered while looking for',
     $         ' an OPTIONS file on unit', I6)

*     end of s3file
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s3fils( ncalls, iOptns, opset,
     $                   title, iPrint, iSumm, inform,
     $                   cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      integer            ncalls, iOptns, iPrint, iSumm, inform
      character*(*)      title
      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)
      external           opset

*     ==================================================================
*     s3fils  reads an options file from unit  iOptns, which may contain 
*     several sets of options.
*
*     On exit, inform says how many errors were encountered.
*
*     15 Nov 1991: First version based on Minos/Npsol routine s3file.
*     17 May 1998: Current version.
*     ==================================================================
      character*16       key   , token(1)
      character*72       buffer
      character*8        cvalue
      character*30       dashes
      data               dashes /'=============================='/
*     ------------------------------------------------------------------

*     lPrnt and lSumm and local copies of iPrint and iSumm
*     (which might get changed by s3opt).

	print *,'here2'

      lPrnt  = iPrint
      lSumm  = iSumm
      inform = 0
  
*     Return if the unit number is out of range.
      
      if (iOptns .lt. 0  .or.  iOptns .gt. 99) then
         inform = 1
         return
      end if

*     ------------------------------------------------------------------
*     Look for  Begin, Endrun  or  Skip.
*     ------------------------------------------------------------------
      nread  = 0
   50    read (iOptns, '(a72)', end = 930) buffer
         nread = nread + 1
         nkey  = 1
         call optokn( buffer, nkey, token )
         key   = token(1)
         if (key .eq. 'ENDRUN') go to 940
         if (key .ne. 'BEGIN' ) then
            if (nread .eq. 1  .and.  key .ne. 'SKIP') then
               inform = inform + 1
               if (lPrnt .gt. 0) write (lPrnt, 2000) iOptns, buffer
               if (lSumm .gt. 0) write (lSumm, 2000) iOptns, buffer
            end if
            go to 50
         end if

*     ------------------------------------------------------------------
*     Begin found.
*     This is taken to be the first line of an OPTIONS file.
*     It is printed without the trailing blanks.
*     ------------------------------------------------------------------
      call s1page( 1, iw, leniw )
      lenbuf   = 1
      do 10, j = 1, len(buffer)
         if (buffer(j:j) .ne. ' ') lenbuf = j
   10 continue

      if (lPrnt .gt. 0) then
         write (lPrnt, '(  9x, a )') ' ', dashes, title, dashes
         write (lPrnt, '(/(1x, a))') 'OPTIONS file', '------------'
         write (lPrnt, '(  6x, a)') buffer(1:lenbuf)
      end if
      if (lSumm .gt. 0) then
         write (lSumm, '(  1x, a )') ' ', dashes, title, dashes
         write (lSumm, '(/ 1x, a)') buffer(1:lenbuf)
      end if

*     ------------------------------------------------------------------
*     Read the rest of the file.
*     ------------------------------------------------------------------
*+    while (key .ne. 'END') loop
  100 if    (key .ne. 'END') then
         read  (iOptns, '(a72)', end = 920) buffer

	print *, 'Hola? ', buffer

         call opset ( .true., buffer, key, cvalue, ivalue, rvalue,
     $                lPrnt, lSumm, inform,
     $                cw, lencw, iw, leniw, rw, lenrw )
         go to 100
      end if
*+    end while

      return

  920 if (lPrnt .gt. 0) write (lPrnt, 2200) iOptns
      if (lSumm .gt. 0) write (lSumm, 2200) iOptns
      inform = 2
      return

  930 if (ncalls .le. 1) then
         if (lPrnt .gt. 0) write (lPrnt, 2300) iOptns
         if (lSumm .gt. 0) write (lSumm, 2300) iOptns
      else
         if (lPrnt .gt. 0) write (lPrnt, '(a)') ' Endrun'
         if (lSumm .gt. 0) write (lSumm, '(a)') ' Endrun'
      end if
      inform = 3
      return

  940 if (lPrnt .gt. 0) write (lPrnt, '(/ 6x, a)') buffer
      if (lSumm .gt. 0) write (lSumm, '(/ 1x, a)') buffer
      inform = 4
      return

 2000 format(
     $ //' XXX  Error while looking for an OPTIONS file on unit', I7
     $ / ' XXX  The file should start with Begin, Skip or Endrun'
     $ / ' XXX  but the first record found was the following:'
     $ //' ---->', a
     $ //' XXX  Continuing to look for OPTIONS file...')
 2200 format(//' XXX  End-of-file encountered while processing',
     $         ' an OPTIONS file on unit', I6)
 2300 format(//' XXX  End-of-file encountered while looking for',
     $         ' an OPTIONS file on unit', I6)

*     end of s3fils
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s3key ( key, loc )

      character*16       key

*     ==================================================================
*     s3key  sets key to be the standard form for the first keyword
*     on each line of a SPECS file.
*
*     17 May 1998: First version.
*     ==================================================================
      parameter         (     maxkey = 80)
      character*16       keys(maxkey)
      logical            sorted
      parameter         (sorted =   .true.)

      data
     $   keys(  1) /'AIJ          '/,
     $   keys(  2) /'BACKUP       '/,
     $   keys(  3) /'BOUNDS       '/,
     $   keys(  4) /'CENTRAL      '/,
     $   keys(  5) /'CHECK        '/,
     $   keys(  6) /'COEFFICIENTS '/,
     $   keys(  7) /'COLD         '/,
     $   keys(  8) /'COLUMNS      '/,
     $   keys(  9) /'CRASH        '/,
     $   keys( 10) /'CYCLE        '/,
     $   keys( 11) /'DEBUG        '/,
     $   keys( 12) /'DEFAULTS     '/,
     $   keys( 13) /'DERIVATIVE   '/,
     $   keys( 14) /'DIFFERENCE   '/,
     $   keys( 15) /'DUMP         '/,
     $   keys( 16) /'ELASTIC      '/,
     $   keys( 17) /'ELEMENTS     '/,
     $   keys( 18) /'ERROR        '/,
     $   keys( 19) /'EXPAND       '/,
     $   keys( 20) /'FACTORIZATION'/

      data
     $   keys( 21) /'FEASIBILITY  '/,
     $   keys( 22) /'FEASIBLE     '/,
     $   keys( 23) /'FUNCTION     '/,
     $   keys( 24) /'HESSIAN      '/,
     $   keys( 25) /'INFEASIBLE   '/,
     $   keys( 26) /'INFINITE     '/,
     $   keys( 27) /'INSERT       '/,
     $   keys( 28) /'ITERATIONS   '/,
     $   keys( 29) /'IW           '/,
     $   keys( 30) /'JACOBIAN     '/,
     $   keys( 31) /'LINESEARCH   '/,
     $   keys( 32) /'LIST         '/,
     $   keys( 33) /'LOAD         '/,
     $   keys( 34) /'LOG          '/,
     $   keys( 35) /'LOWER        '/,
     $   keys( 36) /'LP           '/,
     $   keys( 37) /'LU           '/,
     $   keys( 38) /'MAJOR        '/,
     $   keys( 39) /'MAXIMIZE     '/,
     $   keys( 40) /'MINIMIZE     '/

      data
     $   keys( 41) /'MINOR        '/,
     $   keys( 42) /'MPS          '/,
     $   keys( 43) /'NEW          '/,
     $   keys( 44) /'NO           '/,
     $   keys( 45) /'NON          '/,
     $   keys( 46) /'NONDERIVATIVE'/,
     $   keys( 47) /'NONLINEAR    '/,
     $   keys( 48) /'OBJECTIVE    '/,
     $   keys( 49) /'OLD          '/,
     $   keys( 50) /'OPTIMALITY   '/,
     $   keys( 51) /'PARTIAL      '/,
     $   keys( 52) /'PENALTY      '/,
     $   keys( 53) /'PIVOT        '/,
     $   keys( 54) /'PRINT        '/,
     $   keys( 55) /'PROBLEM      '/,
     $   keys( 56) /'PUNCH        '/,
     $   keys( 57) /'QP           '/,
     $   keys( 58) /'REDUCED      '/,
     $   keys( 59) /'RANGES       '/,
     $   keys( 60) /'REPORT       '/
      data
     $   keys( 61) /'RHS          '/,
     $   keys( 62) /'ROWS         '/,
     $   keys( 63) /'RW           '/,
     $   keys( 64) /'SAVE         '/,
     $   keys( 65) /'SCALE        '/,
     $   keys( 66) /'SOLUTION     '/,
     $   keys( 67) /'START        '/,
     $   keys( 68) /'STOP         '/,
     $   keys( 69) /'SUMMARY      '/,
     $   keys( 70) /'SUPERBASICS  '/,
     $   keys( 71) /'SUPPRESS     '/,
     $   keys( 72) /'TIMING       '/,
     $   keys( 73) /'TOTAL        '/,
     $   keys( 74) /'UNBOUNDED    '/,
     $   keys( 75) /'UPPER        '/,
     $   keys( 76) /'USER         '/,
     $   keys( 77) /'VERIFY       '/,
     $   keys( 78) /'VIOLATION    '/,
     $   keys( 79) /'WARM         '/,
     $   keys( 80) /'WORKSPACE    '/

      call oplook( maxkey, keys, sorted, key, loc )

*     end of s3key
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s3tie ( tie, loc )

      character*16       tie

*     ==================================================================
*     s3tie  sets key to be the standard form for the second keyword
*     on each line of a SPECS file.
*
*     17 May 1998: First version.
*     ==================================================================
      parameter         (     maxtie = 57)
      character*16       ties(maxtie)
      logical            sorted
      parameter         (sorted =   .true.)

      data
     $   ties(  1) /'(TOTAL)      '/,
     $   ties(  2) /'(USER)       '/,
     $   ties(  3) /'ALL          '/,
     $   ties(  4) /'BASIC        '/,
     $   ties(  5) /'BOUND        '/,
     $   ties(  6) /'CHARACTER    '/,
     $   ties(  7) /'COLUMNS      '/,
     $   ties(  8) /'CONSTRAINTS  '/,
     $   ties(  9) /'DAMPING      '/,
     $   ties( 10) /'DEBUG        '/,
     $   ties( 11) /'DENSE        '/,
     $   ties( 12) /'DENSITY      '/,
     $   ties( 13) /'DERIVATIVE   '/,
     $   ties( 14) /'DIFFERENCES  '/,
     $   ties( 15) /'DIMENSION    '/,
     $   ties( 16) /'ELEMENTS     '/,
     $   ties( 17) /'EXIT         '/,
     $   ties( 18) /'FACTORIZATION'/,
     $   ties( 19) /'FEASIBILITY  '/,
     $   ties( 20) /'FILE         '/
      data
     $   ties( 21) /'FLUSH        '/,
     $   ties( 22) /'FREQUENCY    '/,
     $   ties( 23) /'FULL         '/,
     $   ties( 24) /'GRADIENTS    '/,
     $   ties( 25) /'INTEGER      '/,
     $   ties( 26) /'ITERATIONS   '/,
     $   ties( 27) /'JACOBIAN     '/,
     $   ties( 28) /'LEVEL        '/,
     $   ties( 29) /'LIMITED      '/,
     $   ties( 30) /'LINEAR       '/,
     $   ties( 31) /'LINESEARCH   '/,
     $   ties( 32) /'LOG          '/,
     $   ties( 33) /'MODE         '/,
     $   ties( 34) /'NEWTON       '/,
     $   ties( 35) /'NO           '/,
     $   ties( 36) /'NONLINEAR    '/,
     $   ties( 37) /'OBJECTIVE    '/,
     $   ties( 38) /'OPTIMALITY   '/,
     $   ties( 39) /'OPTION       '/,
     $   ties( 40) /'PARTIAL      '/
      data
     $   ties( 41) /'PHASE1       '/,
     $   ties( 42) /'PHASE2       '/,
     $   ties( 43) /'POINT        '/,
     $   ties( 44) /'PRINT        '/,
     $   ties( 45) /'REAL         '/,
     $   ties( 46) /'SINGULARITY  '/,
     $   ties( 47) /'SPARSE       '/,
     $   ties( 48) /'START        '/,
     $   ties( 49) /'STEP         '/,
     $   ties( 50) /'STOP         '/,
     $   ties( 51) /'SUPERBASIC   '/,
     $   ties( 52) /'SWAP         '/,
     $   ties( 53) /'TOLERANCE    '/,
     $   ties( 54) /'UPDATES      '/,
     $   ties( 55) /'VARIABLES    '/,
     $   ties( 56) /'WEIGHT       '/,
     $   ties( 57) /'YES          '/

      call oplook( maxtie, ties, sorted, tie, loc )

*     end of s3tie
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s3undf( cw, lencw, iw, leniw, rw, lenrw )

      implicit           double precision (a-h,o-z)
      character*8        cw(lencw)
      integer            iw(leniw)
      double precision   rw(lenrw)

*     ==================================================================
*     s3dflt sets all options as undefined.
*
*     07-Feb-98: First version of s3undf.
*     07-Feb-98: Current version.
*     ==================================================================
      character*8        cdummy
      parameter         (idummy = -11111,  rdummy = -11111.0d+0)
      parameter         (cdummy = '-1111111'                   )
*     ------------------------------------------------------------------
      do 100, i = 51, 180
         cw(i)  = cdummy
         iw(i)  = idummy
         rw(i)  = rdummy
  100 continue

*     end of s3undf
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
C
      SUBROUTINE OPLOOK (NDICT, DICTRY, ALPHA, KEY, ENTRY)
C
C
C Description and usage:
C
C       Performs dictionary lookups.  A pointer is returned if a
C    match is found between the input key and the corresponding
C    initial characters of one of the elements of the dictionary.
C    If a "synonym" has been provided for an entry, the search is
C    continued until a match to a primary dictionary entry is found.
C    Cases of no match, or multiple matches, are also provided for.
C
C     Dictionary entries must be left-justified, and may be alphabetized
C    for faster searches.  Secondary entries, if any, are composed of
C    two words separated by one or more characters such as blank, tab,
C    comma, colon, or equal sign which are treated as non-significant
C    by OPSCAN.  The first entry of each such pair serves as a synonym
C    for the second, more fundamental keyword.
C
C       The ordered search stops after the section of the dictionary
C    having the same first letters as the key has been checked, or
C    after a specified number of entries have been examined.  A special
C    dictionary entry, the vertical bar '|', will also terminate the
C    search.  This will speed things up if an appropriate dictionary
C    length parameter cannot be determined.  Both types of search are
C    sequential.  See "Notes" below for some suggestions if efficiency
C    is an issue.
C
C
C Parameters:
C
C    Name    Dimension  Type  I/O/S  Description
C    NDICT               I    I      Number of dictionary entries to be
C                                    examined.
C    DICTRY  NDICT       C    I      Array of dictionary entries,
C                                    left-justified in their fields.
C                                    May be alphabetized for efficiency,
C                                    in which case ALPHA should be
C                                    .TRUE.  Entries with synonyms are
C                                    of the form
C                                    'ENTRY : SYNONYM', where 'SYNONYM'
C                                    is a more fundamental entry in the
C                                    same dictionary.  NOTE: Don't build
C                                    "circular" dictionaries!
C    ALPHA               L    I      Indicates whether the dictionary
C                                    is in alphabetical order, in which
C                                    case the search can be terminated
C                                    sooner.
C    KEY                 C    I/O    String to be compared against the
C                                    dictionary.  Abbreviations are OK
C                                    if they correspond to a unique
C                                    entry in the dictionary.  KEY is
C                                    replaced on termination by its most
C                                    fundamental equivalent dictionary
C                                    entry (uppercase, left-justified)
C                                    if a match was found.
C    ENTRY               I      O    Dictionary pointer.  If > 0, it
C                                    indicates which entry matched KEY.
C                                    In case of trouble, a negative
C                                    value means that a UNIQUE match
C                                    was not found - the absolute value
C                                    of ENTRY points to the second
C                                    dictionary entry that matched KEY.
C                                    Zero means that NO match could be
C                                    found.  ENTRY always refers to the
C                                    last search performed -
C                                    in searching a chain of synonyms,
C                                    a non-positive value will be
C                                    returned if there is any break,
C                                    even if the original input key
C                                    was found.
C
C
C External references:
C
C    Name    Description
C    OPSCAN  Finds first and last significant characters.
C
C
C Environment:  Digital VAX-11/780 VMS FORTRAN (FORTRAN 77).
C               Appears to satisfy the ANSI Fortran 77 standard.
C
C
C Notes:
C
C    (1)  IMPLICIT NONE is non-standard.  (Has been commented out.)
C
C    (2)  We have assumed that the dictionary is not too big.  If
C         many searches are to be done or if the dictionary has more
C         than a dozen or so entries, it may be advantageous to build
C         an index array of pointers to the beginning of the section
C         of the dictionary containing each letter, then pass in the
C         portion of the dictionary beginning with DICTRY (INDEX).
C         (This won't generally work for dictionaries with synonyms.)
C         For very large problems, a completely different approach may
C         be advisable, e.g. a binary search for ordered dictionaries.
C
C    (3)  OPLOOK is case sensitive.  In most applications it will be
C         necessary to use an uppercase dictionary, and to convert the
C         input key to uppercase before calling OPLOOK.  Companion
C         routines OPTOKN and PAIRS, available from the author, already
C         take care of this.
C
C    (4)  The key need not be left-justified.  Any leading (or
C         trailing) characters which are "non-significant" to OPSCAN
C         will be ignored.  These include blanks, horizontal tabs,
C         commas, colons, and equal signs.  See OPSCAN for details.
C
C    (5)  The ASCII collating sequence for character data is assumed.
C         (N.B. This means the numerals precede the alphabet, unlike
C         common practice!)  This should not cause trouble on EBCDIC
C         machines if DICTRY just contains alphabetic keywords.
C         Otherwise it may be necessary to use the FORTRAN lexical
C         library routines to force use of the ASCII sequence.
C
C    (6)  Parameter NUMSIG sets a limit on the length of significant
C         dictionary entries.  Special applications may require that
C         this be increased.  (It is 16 in the present version.)
C
C    (7)  No protection against "circular" dictionaries is provided:
C         don't claim that A is B, and that B is A.  All synonym chains
C         must terminate!  Other potential errors not checked for
C         include duplicate or mis-ordered entries.
C
C    (8)  The handling of ambiguities introduces some ambiguity:
C
C            ALPHA = .TRUE.  A potential problem, when one entry
C                            looks like an abbreviation for another
C                            (eg. does 'A' match 'A' or 'AB'?) was
C                            resolved by dropping out of the search
C                            immediately when an "exact" match is found.
C
C            ALPHA = .FALSE. The programmer must ensure that the above
C                            situation does not arise: each dictionary
C                            entry must be recognizable, at least when
C                            specified to full length.  Otherwise, the
C                            result of a search will depend on the
C                            order of entries.
C
C
C Author:  Robert Kennelly, Informatics General Corporation.
C
C
C Development history:
C
C    24 Feb. 1984  RAK/DAS  Initial design and coding.
C    25 Feb. 1984    RAK    Combined the two searches by suitable
C                           choice of terminator FLAG.
C    28 Feb. 1984    RAK    Optional synonyms in dictionary, no
C                           longer update KEY.
C    29 Mar. 1984    RAK    Put back replacement of KEY by its
C                           corresponding entry.
C    21 June 1984    RAK    Corrected bug in error handling for cases
C                           where no match was found.
C    23 Apr. 1985    RAK    Introduced test for exact matches, which
C                           permits use of dictionary entries which
C                           would appear to be ambiguous (for ordered
C                           case).  Return -I to point to the entry
C                           which appeared ambiguous (had been -1).
C                           Repaired loop termination - had to use
C                           equal length strings or risk quitting too
C                           soon when one entry is an abbreviation
C                           for another.  Eliminated HIT, reduced
C                           NUMSIG to 16.
C    15 Nov. 1985    MAS    Loop 20 now tests .LT. FLAG, not .LE. FLAG.
C                           If ALPHA is false, FLAG is now '|', not '{'.
C    26 Jan. 1986    PEG    Declaration of FLAG and TARGET modified to
C                           conform to ANSI-77 standard.
C-----------------------------------------------------------------------


C     Variable declarations.
C     ----------------------

*     IMPLICIT NONE

C     Parameters.

      INTEGER
     $   NUMSIG
      CHARACTER
     $   VBAR
      PARAMETER
     $   (VBAR = '|', NUMSIG = 16)

C     Variables.

      LOGICAL
     $   ALPHA
      INTEGER
     $   ENTRY, FIRST, I, LAST, LENGTH, MARK, NDICT
*     CHARACTER
*    $   DICTRY (NDICT) * (*), FLAG * (NUMSIG),
*    $   KEY * (*), TARGET * (NUMSIG)
      CHARACTER
     $   DICTRY (NDICT) * (*), FLAG * 16,
     $   KEY * (*), TARGET * 16

C     Procedures.

      EXTERNAL
     $   OPSCAN


C     Executable statements.
C     ----------------------

      ENTRY = 0

C     Isolate the significant portion of the input key (if any).

      FIRST = 1
      LAST  = MIN( LEN(KEY), NUMSIG )
      CALL OPSCAN (KEY, FIRST, LAST, MARK)

      IF (MARK .GT. 0) THEN
         TARGET = KEY (FIRST:MARK)

C        Look up TARGET in the dictionary.

   10    CONTINUE
            LENGTH = MARK - FIRST + 1

C           Select search strategy by cunning choice of termination test
C           flag.  The vertical bar is just about last in both the
C           ASCII and EBCDIC collating sequences.

            IF (ALPHA) THEN
               FLAG = TARGET
            ELSE
               FLAG = VBAR
            END IF


C           Perform search.
C           ---------------

            I = 0
   20       CONTINUE
               I = I + 1
               IF (TARGET (1:LENGTH) .EQ. DICTRY (I) (1:LENGTH)) THEN
                  IF (ENTRY .EQ. 0) THEN

C                    First "hit" - must still guard against ambiguities
C                    by searching until we've gone beyond the key
C                    (ordered dictionary) or until the end-of-dictionary
C                    mark is reached (exhaustive search).

                     ENTRY = I

C                    Special handling if match is exact - terminate
C                    search.  We thus avoid confusion if one dictionary
C                    entry looks like an abbreviation of another.
C                    This fix won't generally work for un-ordered
C                    dictionaries!

                     FIRST = 1
                     LAST = NUMSIG
                     CALL OPSCAN (DICTRY (ENTRY), FIRST, LAST, MARK)
                     IF (MARK .EQ. LENGTH) I = NDICT
                  ELSE


C                    Oops - two hits!  Abnormal termination.
C                    ---------------------------------------

                     ENTRY = -I
                     RETURN
                  END IF
               END IF

C           Check whether we've gone past the appropriate section of the
C           dictionary.  The test on the index provides insurance and an
C           optional means for limiting the extent of the search.

            IF (DICTRY (I) (1:LENGTH) .LT. FLAG  .AND.  I .LT. NDICT)
     $         GO TO 20


C           Check for a synonym.
C           --------------------

            IF (ENTRY .GT. 0) THEN

C              Look for a second entry "behind" the first entry.  FIRST
C              and MARK were determined above when the hit was detected.

               FIRST = MARK + 2
               CALL OPSCAN (DICTRY (ENTRY), FIRST, LAST, MARK)
               IF (MARK .GT. 0) THEN

C                 Re-set target and dictionary pointer, then repeat the
C                 search for the synonym instead of the original key.

                  TARGET = DICTRY (ENTRY) (FIRST:MARK)
                  ENTRY = 0
                  GO TO 10

               END IF
            END IF

      END IF
      IF (ENTRY .GT. 0) KEY = DICTRY (ENTRY)


C     Normal termination.
C     -------------------

      RETURN

C     End of OPLOOK
      END
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      FUNCTION OPNUMB( STRING )

      LOGICAL          OPNUMB
      CHARACTER*(*)    STRING

************************************************************************
*     Description and usage:
*
*        A simple(-minded) test for numeric data is implemented by
*        searching an input string for legitimate characters:
*                digits 0 to 9, D, E, -, + and .
*        Insurance is provided by requiring that a numeric string
*        have at least one digit, at most one D, E or .
*        and at most two -s or +s.  Note that a few ambiguities remain:
*
*           (a)  A string might have the form of numeric data but be
*                intended as text.  No general test can hope to detect
*                such cases.
*
*           (b)  There is no check for correctness of the data format.
*                For example a meaningless string such as 'E1.+2-'
*                will be accepted as numeric.
*
*        Despite these weaknesses, the method should work in the
*        majority of cases.
*
*
*     Parameters:
*
*        Name    Dimension  Type  I/O/S  Description
*        OPNUMB              L      O    Set .TRUE. if STRING appears
*                                        to be numerical data.
*        STRING              C    I      Input data to be tested.
*
*
*     Environment:  ANSI FORTRAN 77.
*
*
*     Notes:
*
*        (1)  It is assumed that STRING is a token extracted by
*             OPTOKN, which will have converted any lower-case
*             characters to upper-case.
*
*        (2)  OPTOKN pads STRING with blanks, so that a genuine
*             number is of the form  '1234        '.
*             Hence, the scan of STRING stops at the first blank.
*
*        (3)  COMPLEX data with parentheses will not look numeric.
*
*
*     Systems Optimization Laboratory, Stanford University.
*     12 Nov  1985    Initial design and coding, starting from the
*                     routine ALPHA from Informatics General, Inc.
************************************************************************

      LOGICAL         NUMBER
      INTEGER         J, LENGTH, NDIGIT, NEXP, NMINUS, NPLUS, NPOINT
      CHARACTER*1     ATOM

      NDIGIT = 0
      NEXP   = 0
      NMINUS = 0
      NPLUS  = 0
      NPOINT = 0
      NUMBER = .TRUE.
      LENGTH = LEN (STRING)
      J      = 0

   10    J    = J + 1
         ATOM = STRING (J:J)
         IF      (ATOM .GE. '0'  .AND.  ATOM .LE. '9') THEN
            NDIGIT = NDIGIT + 1
         ELSE IF (ATOM .EQ. 'D'  .OR.   ATOM .EQ. 'E') THEN
            NEXP   = NEXP   + 1
         ELSE IF (ATOM .EQ. '-') THEN
            NMINUS = NMINUS + 1
         ELSE IF (ATOM .EQ. '+') THEN
            NPLUS  = NPLUS  + 1
         ELSE IF (ATOM .EQ. '.') THEN
            NPOINT = NPOINT + 1
         ELSE IF (ATOM .EQ. ' ') THEN
            J      = LENGTH
         ELSE
            NUMBER = .FALSE.
         END IF

         IF (NUMBER  .AND.  J .LT. LENGTH) GO TO 10

      OPNUMB = NUMBER
     $         .AND.  NDIGIT .GE. 1
     $         .AND.  NEXP   .LE. 1
     $         .AND.  NMINUS .LE. 2
     $         .AND.  NPLUS  .LE. 2
     $         .AND.  NPOINT .LE. 1

      RETURN

*     End of OPNUMB
      END
C+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
C
      SUBROUTINE OPSCAN (STRING, FIRST, LAST, MARK)
C
C
C Description and usage:
C
C       Looks for non-blank fields ("tokens") in a string, where the
C    fields are of arbitrary length, separated by blanks, tabs, commas,
C    colons, or equal signs.  The position of the end of the 1st token
C    is also returned, so this routine may be conveniently used within
C    a loop to process an entire line of text.
C
C       The procedure examines a substring, STRING (FIRST : LAST), which
C    may of course be the entire string (in which case just call OPSCAN
C    with FIRST <= 1 and LAST >= LEN (STRING) ).  The indices returned
C    are relative to STRING itself, not the substring.
C
C
C Parameters:
C
C    Name    Dimension  Type  I/O/S  Description
C    STRING              C    I      Text string containing data to be
C                                    scanned.
C    FIRST               I    I/O    Index of beginning of substring.
C                                    If <= 1, the search begins with 1.
C                                    Output is index of beginning of
C                                    first non-blank field, or 0 if no
C                                    token was found.
C    LAST                I    I/O    Index of end of substring.
C                                    If >= LEN (STRING), the search
C                                    begins with LEN (STRING).  Output
C                                    is index of end of last non-blank
C                                    field, or 0 if no token was found.
C    MARK                I      O    Points to end of first non-blank
C                                    field in the specified substring.
C                                    Set to 0 if no token was found.
C
C
C Environment:  Digital VAX-11/780 VMS FORTRAN (FORTRAN 77).
C               ANSI Fortran 77, except for the tab character HT.
C
C Notes:
C
C    (1)  IMPLICIT NONE is non-standard.  Constant HT (Tab) is defined
C         in a non-standard way:  the CHAR function is not permitted
C         in a PARAMETER declaration (OK on VAX, though).  For Absoft
C         FORTRAN 77 on 68000 machines, use HT = 9.  In other cases, it
C         may be best to declare HT as a variable and assign
C         HT = CHAR(9) on ASCII machines, or CHAR(5) for EBCDIC.
C
C    (2)  The pseudo-recursive structure was chosen for fun.  It is
C         equivalent to three DO loops with embedded GO TOs in sequence.
C
C    (3)  The variety of separators recognized limits the usefulness of
C         this routine somewhat.  The intent is to facilitate handling
C         such tokens as keywords or numerical values.  In other
C         applications, it may be necessary for ALL printing characters
C         to be significant.  A simple modification to statement
C         function SOLID will do the trick.
C
C
C Author:  Robert Kennelly, Informatics General Corporation.
C
C
C Development history:
C
C    29 Dec. 1984    RAK    Initial design and coding, (very) loosely
C                           based on SCAN_STRING by Ralph Carmichael.
C    25 Feb. 1984    RAK    Added ':' and '=' to list of separators.
C    16 Apr. 1985    RAK    Defined SOLID in terms of variable DUMMY
C                           (previous re-use of STRING was ambiguous).
C
C-----------------------------------------------------------------------


C     Variable declarations.
C     ----------------------

*     IMPLICIT NONE

C     Parameters.

      CHARACTER
     $   BLANK, EQUAL, COLON, COMMA, HT
      PARAMETER
     $   (BLANK = ' ', EQUAL = '=', COLON = ':', COMMA = ',')

C     Variables.

      LOGICAL
     $   SOLID
      INTEGER
     $   BEGIN, END, FIRST, LAST, LENGTH, MARK
      CHARACTER
     $   DUMMY, STRING * (*)

C     Statement functions.

      SOLID (DUMMY) = (DUMMY .NE. BLANK) .AND.
     $                (DUMMY .NE. COLON) .AND.
     $                (DUMMY .NE. COMMA) .AND.
     $                (DUMMY .NE. EQUAL) .AND.
     $                (DUMMY .NE. HT)


C     Executable statements.
C     ----------------------

****  HT     = CHAR(9) for ASCII machines, CHAR(5) for EBCDIC.
      HT     = CHAR(9)
      DUMMY  = ' '
      MARK   = 0
      LENGTH = LEN (STRING)
      BEGIN  = MAX (FIRST, 1)
      END    = MIN (LENGTH, LAST)

C     Find the first significant character ...

      DO 30 FIRST = BEGIN, END, +1
         IF (SOLID (STRING (FIRST : FIRST))) THEN

C           ... then the end of the first token ...

            DO 20 MARK = FIRST, END - 1, +1
               IF (.NOT.SOLID (STRING (MARK + 1 : MARK + 1))) THEN

C                 ... and finally the last significant character.

                  DO 10 LAST = END, MARK, -1
                     IF (SOLID (STRING (LAST : LAST))) THEN
                        RETURN
                     END IF
   10             CONTINUE

C                 Everything past the first token was a separator.

                  LAST = LAST + 1
                  RETURN
               END IF
   20       CONTINUE

C           There was nothing past the first token.

            LAST = MARK
            RETURN
         END IF
   30 CONTINUE

C     Whoops - the entire substring STRING (BEGIN : END) was composed of
C     separators !

      FIRST = 0
      MARK = 0
      LAST = 0
      RETURN

C     End of OPSCAN
      END
C+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
C
      SUBROUTINE OPTOKN (STRING, NUMBER, LIST)
C
C
C Description and usage:
C
C       An aid to parsing input data.  The individual "tokens" in a
C    character string are isolated, converted to uppercase, and stored
C    in an array.  Here, a token is a group of significant, contiguous
C    characters.  The following are NON-significant, and hence may
C    serve as separators:  blanks, horizontal tabs, commas, colons,
C    and equal signs.  See OPSCAN for details.  Processing continues
C    until the requested number of tokens have been found or the end
C    of the input string is reached.
C
C
C Parameters:
C
C    Name    Dimension  Type  I/O/S  Description
C    STRING              C    I      Input string to be analyzed.
C    NUMBER              I    I/O    Number of tokens requested (input)
C                                    and found (output).
C    LIST    NUMBER      C      O    Array of tokens, changed to upper
C                                    case.
C
C
C External references:
C
C    Name    Description
C    OPSCAN  Finds positions of first and last significant characters.
C    OPUPPR  Converts a string to uppercase.
C
C
C Environment:  Digital VAX-11/780 VMS FORTRAN (FORTRAN 77).
C               Appears to satisfy the ANSI Fortran 77 standard.
C
C
C Notes:
C
C    (1)  IMPLICIT NONE is non-standard.  (Has been commented out.)
C
C
C Author:  Robert Kennelly, Informatics General Corporation.
C
C
C Development history:
C
C    16 Jan. 1984    RAK    Initial design and coding.
C    16 Mar. 1984    RAK    Revised header to reflect full list of
C                           separators, repaired faulty WHILE clause
C                           in "10" loop.
C    18 Sep. 1984    RAK    Change elements of LIST to uppercase one
C                           at a time, leaving STRING unchanged.
C
C-----------------------------------------------------------------------


C     Variable declarations.
C     ----------------------

*     IMPLICIT NONE

C     Parameters.

      CHARACTER
     $   BLANK
      PARAMETER
     $   (BLANK = ' ')

C     Variables.

      INTEGER
     $   COUNT, FIRST, I, LAST, MARK, NUMBER
      CHARACTER
     $   STRING * (*), LIST (NUMBER) * (*)

C     Procedures.

      EXTERNAL
     $   OPUPPR, OPSCAN


C     Executable statements.
C     ----------------------

C     WHILE there are tokens to find, loop UNTIL enough have been found.

      FIRST = 1
      LAST = LEN (STRING)

      COUNT = 0
   10 CONTINUE

C        Get delimiting indices of next token, if any.

         CALL OPSCAN (STRING, FIRST, LAST, MARK)
         IF (LAST .GT. 0) THEN
            COUNT = COUNT + 1

C           Pass token to output string array, then change case.

            LIST (COUNT) = STRING (FIRST : MARK)
            CALL OPUPPR (LIST (COUNT))
            FIRST = MARK + 2
            IF (COUNT .LT. NUMBER) GO TO 10

         END IF


C     Fill the rest of LIST with blanks and set NUMBER for output.

      DO 20 I = COUNT + 1, NUMBER
         LIST (I) = BLANK
   20 CONTINUE

      NUMBER = COUNT


C     Termination.
C     ------------

      RETURN

C     End of OPTOKN
      END
C+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
C
      SUBROUTINE OPUPPR(STRING)
C
C ACRONYM:  UPper CASE
C
C PURPOSE:  This subroutine changes all lower case letters in the
C           character string to upper case.
C
C METHOD:   Each character in STRING is treated in turn.  The intrinsic
C           function INDEX effectively allows a table lookup, with
C           the local strings LOW and UPP acting as two tables.
C           This method avoids the use of CHAR and ICHAR, which appear
C           be different on ASCII and EBCDIC machines.
C
C ARGUMENTS
C    ARG       DIM     TYPE I/O/S DESCRIPTION
C  STRING       *       C   I/O   Character string possibly containing
C                                 some lower-case letters on input;
C                                 strictly upper-case letters on output
C                                 with no change to any non-alphabetic
C                                 characters.
C
C EXTERNAL REFERENCES:
C  LEN    - Returns the declared length of a CHARACTER variable.
C  INDEX  - Returns the position of second string within first.
C
C ENVIRONMENT:  ANSI FORTRAN 77
C
C DEVELOPMENT HISTORY:
C     DATE  INITIALS  DESCRIPTION
C   06/28/83   CLH    Initial design.
C   01/03/84   RAK    Eliminated NCHAR input.
C   06/14/84   RAK    Used integer PARAMETERs in comparison.
C   04/21/85   RAK    Eliminated DO/END DO in favor of standard code.
C   09/10/85   MAS    Eliminated CHAR,ICHAR in favor of LOW, UPP, INDEX.
C
C AUTHOR: Charles Hooper, Informatics General, Palo Alto, CA.
C
C-----------------------------------------------------------------------

      CHARACTER      STRING * (*)
      INTEGER        I, J
      CHARACTER      C*1, LOW*26, UPP*26
      DATA           LOW /'abcdefghijklmnopqrstuvwxyz'/,
     $               UPP /'ABCDEFGHIJKLMNOPQRSTUVWXYZ'/

      DO 10 J = 1, LEN(STRING)
         C    = STRING(J:J)
         IF (C .GE. 'a'  .AND.  C .LE. 'z') THEN
            I           = INDEX( LOW, C )
            IF (I .GT. 0) STRING(J:J) = UPP(I:I)
         END IF
   10 CONTINUE
      RETURN

*     End of OPUPPR

      END
