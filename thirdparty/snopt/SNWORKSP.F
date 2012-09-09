*     ==================================================================
*     rw(1)--rw(29) contain machine-dependent parameters.
*     ------------------------------------------------------------------
*     eps       = rw(  1) ! unit round-off.
*     eps0      = rw(  2) ! eps**(4/5)
*     eps1      = rw(  3) ! eps**(2/3)
*     eps2      = rw(  4) ! eps**(1/2)
*     eps3      = rw(  5) ! eps**(1/3)
*     eps4      = rw(  6) ! eps**(1/4)
*     eps5      = rw(  7) ! eps**(1/5)
*     flmax     = rw(  8) ! est. of the largest pos. real.
*     flmin     = rw(  9) ! smallest positive real.
*     rtundf    = rw( 10) ! sqrt of flmin.
*     ------------------------------------------------------------------
*     rw(51)--rw(150): optional parameters set via the specs file.
*     ------------------------------------------------------------------
*     tolFP     = rw( 51) ! Minor Phase 1 Opt tol
*     tolQP     = rw( 52) ! Minor Phase 2 Opt tol
*     tolNLP    = rw( 53) ! Major Optimality tolerance
*-->            = rw( 54) !
*-->            = rw( 55) !
*     tolx      = rw( 56) ! Minor feasibility tolerance.
*     tolCon    = rw( 57) ! Major feasibility tolerance.
*-->            = rw( 58) !
*-->            = rw( 59) !
*     tolpiv    = rw( 60) ! excludes small elements of y.
*     tolrow    = rw( 61) ! tolerance for the row error.
*     tCrash    = rw( 62) ! crash tolerance.
*-->            = rw( 63) !
*-->            = rw( 64) !
*     tolswp    = rw( 65) ! LU swap tolerance.
*     tolfac    = rw( 66) ! LU factor tolerance.
*     tolupd    = rw( 67) ! LU update tolerance.
*-->            = rw( 68) !
*     rdummy    = rw( 69) !
*     plInfy    = rw( 70) ! definition of plus infinity.
*     bigfx     = rw( 71) ! unbounded objective.
*     bigdx     = rw( 72) ! unbounded step.
*     epsrf     = rw( 73) ! relative function precision.
*-->            = rw( 74) !
*-->            = rw( 75) !
*     fdint(1)  = rw( 76) ! (1) forwrd diff. interval
*     fdint(2)  = rw( 77) ! (2) cntrl  diff. interval
*-->            = rw( 78) !
*-->            = rw( 79) !
*     xdlim     = rw( 80) ! Step limit
*     vilim     = rw( 81) ! violation limit 
*-->            = rw( 82) !
*-->            = rw( 83) !
*     eta       = rw( 84) ! line search tolerance.
*     Hcndbd    = rw( 85) ! bound on the condition of Hz
*-->            = rw( 86) !
*-->            = rw( 87) !
*     wtInf0    = rw( 88) ! infeasibility weight
*     xPen0     = rw( 89) ! initial penalty parameter.
*-->            = rw( 90) !
*-->            = rw( 91) !
*     scltol    = rw( 92) ! scale tolerance.
*-->            = rw( 93) !
*-->            = rw( 94) !
*     Aijtol    = rw( 95) ! zero Aij tolerance.
*     bStruc(1) = rw( 96) ! default lower bound on x
*     bStruc(2) = rw( 97) ! default upper bound on x
*     ------------------------------------------------------------------
*     rw(151)--z(180) contain  parmLU  parameters for LUSOL.
*     This could be part of an f90 or c structure.
*     ------------------------------------------------------------------
*     eLmax1    = rw(151) ! max L-multiplier in factor.
*     eLmax2    = rw(152) ! max L-multiplier in update.
*     small     = rw(153) ! defn of small real.
*     Utol1     = rw(154) ! abs tol for small diag of U.
*     Utol2     = rw(155) ! rel tol for small diag of U.
*     Uspace    = rw(156) ! limit on waste space in U.
*     Dens1     = rw(157) ! switch to search maxcol columns and no rows.
*     Dens2     = rw(158) ! switch to dense LU.
*-->            = rw(159) !
*     amax      = rw(160) ! maximum element in  A.
*     eLmax     = rw(161) ! maximum multiplier in L.
*     Umax      = rw(162) ! maximum element in U.
*     dUmax     = rw(163) ! maximum diagonal in  U.
*     dUmin     = rw(164) ! minimum diagonal in  U.
*-->            = rw(165) !
*-->            = rw(166) !
*-->            = rw(167) !
*-->            = rw(168) !
*-->            = rw(169) !
*     resid     = rw(170) ! lu6sol: residual.
*     ------------------------------------------------------------------
*     rw(181)--rw(199) pass parameters into various routines.
*     ------------------------------------------------------------------
*-->            = rw(181) !
*-->            = rw(182) !
*-->            = rw(183) !
*     toldj(1)  = rw(184) ! phase 1 dj tol for p.p.
*     toldj(2)  = rw(185) ! phase 2 dj tol for p.p.
*     toldj(3)  = rw(186) ! current optimality tol
*-->            = rw(187) !
*     sclObj    = rw(188) ! scale factor for xBS(kObj)
*-->            = rw(189) !
*     Umin      = rw(190) ! saved smallest U diagonal
*     ------------------------------------------------------------------
*     rw(361)--rw(370) contain variables associated with the QN update.
*     ------------------------------------------------------------------
*     H0scal    = rw(361) ! scale factor for BFGS H0
*     ------------------------------------------------------------------
*     iw(400)--iw(450) information for solvers calling SNOPT
*     ------------------------------------------------------------------
*     ObjTru    = iw(400) ! AMPL: true objective
*     -----------------------------------------------------------------
*     rw(451)--rw(500) timing information
*     -----------------------------------------------------------------
*     tlast( 1) = rw(451) !
*     tlast( 2) = rw(452) !
*     tlast( 3) = rw(453) !
*     tlast( 4) = rw(454) !
*     tlast( 5) = rw(455) !
*     tlast( 6) = rw(456) ! not in use
*     tlast( 7) = rw(457) ! not in use
*     tlast( 8) = rw(458) ! not in use
*     tlast( 9) = rw(459) ! not in use
*     tlast(10) = rw(460) ! not in use
*     tsum( 1)  = rw(461) ! Input time          
*     tsum( 2)  = rw(462) ! Solve time          
*     tsum( 3)  = rw(463) ! Output time         
*     tsum( 4)  = rw(464) ! Constraint functions
*     tsum( 5)  = rw(465) ! Nonlinear objective 
*     tsum( 6)  = rw(466) ! not in use
*     tsum( 7)  = rw(467) ! not in use
*     tsum( 8)  = rw(468) ! not in use
*     tsum( 9)  = rw(469) ! not in use
*     tsum(10)  = rw(470) ! not in use
*     ==================================================================
*     Integer workspace.
*     iw(1)--iw(50): machine-dependent parameters or dimensions.
*     ==================================================================
*-->            = iw(  1) !
*     maxru     = iw(  2) ! maxru+1 is the start of SNOPT part of rw
*     maxrw     = iw(  3) ! end of SNOPT part of rw
*     maxiu     = iw(  4) ! maxiu+1 is the start of SNOPT part of iw
*     maxiw     = iw(  5) ! end of SNOPT part of iw
*     maxcu     = iw(  6) ! maxcu+1 is the start of SNOPT part of cw
*     maxcw     = iw(  7) ! end of SNOPT part of cw
*-->            = iw(  8) !
*-->            = iw(  9) !
*     iRead     = iw( 10) ! Standard Input
*     iSpecs    = iw( 11) ! Specs (options) file
*     iPrint    = iw( 12) ! Print file
*     iSumm     = iw( 13) ! Summary file
*-->            = iw( 14) !
*     ------------------------------------------------------------------
*     Dimensions
*     ------------------------------------------------------------------
*     m         = iw( 15) ! copy of the number of rows
*     n         = iw( 16) ! copy of the number of columns
*     ne        = iw( 17) ! copy of the number of nonzeros in A
*-->            = iw( 18) !
*-->            = iw( 19) !
*     neJac     = iw( 20) ! # of nonzero elems in J
*     nnCon     = iw( 21) ! # of nonlinear constraints
*     nnJac     = iw( 22) ! # nonlinear Jacobian variables
*     nnObj     = iw( 23) ! # variables in gObj
*     nnL       = iw( 24) !   max( nnObj, nnJac )
*-->            = iw( 25) !
*     ngObj     = iw( 26) ! length of QP constant vector
*     nnH       = iw( 27) ! # QP Hessian columns
*     lenr      = iw( 28) ! length of r, i.e., r(lenr)  
*-->            = iw( 29) !
*-->            = iw( 30) !
*     mincu1    = iw( 31) ! Start of first  user partition of cw
*     maxcu1    = iw( 32) ! End   of first  user partition of cw 
*     mincu2    = iw( 33) ! Start of second user partition of cw 
*     maxcu2    = iw( 34) ! End   of second user partition of cw
*-->            = iw( 35) !
*     miniu1    = iw( 36) ! Start of first  user partition of iw
*     maxiu1    = iw( 37) ! End   of first  user partition of iw 
*     miniu2    = iw( 38) ! Start of second user partition of iw 
*     maxiu2    = iw( 39) ! End   of second user partition of iw
*-->            = iw( 40) !
*     minru1    = iw( 41) ! Start of first  user partition of rw
*     maxru1    = iw( 42) ! End   of first  user partition of rw 
*     minru2    = iw( 43) ! Start of second user partition of rw 
*     maxru2    = iw( 44) ! End   of second user partition of rw
*     ------------------------------------------------------------------
*     iw(51)--iw(150): optional parameters set via the specs file.
*     ------------------------------------------------------------------
*     lEmode    = iw( 51) ! >0    => use elastic mode
*-->            = iw( 52) !
*     lHtype    = iw( 53) ! 1(2)    => LM(FM) Hessian
*-->            = iw( 54) !
*-->            = iw( 55) !
*     maxR      = iw( 56) ! max columns of R.
*     maxS      = iw( 57) ! max # of superbasics
*-->            = iw( 58) !
*-->            = iw( 59) !
*     kchk      = iw( 60) ! check (row) frequency
*     kfac      = iw( 61) ! factorization frequency
*     ksav      = iw( 62) ! save basis map
*     klog      = iw( 63) ! log/print frequency
*     kSumm     = iw( 64) ! Summary print frequency
*     kDegen    = iw( 65) ! max. expansions of featol
*     mQNmod    = iw( 66) ! (ge 0) max # of BFGS updates
*     kReset    = iw( 67) ! Hessian frequency
*     mFlush    = iw( 68) ! Hessian flush
*     mSkip     = iw( 69) ! # largest value of nSkip
*     lvlSrt    = iw( 70) ! = 0(1) ==> cold(warm) start
*     lvlDer    = iw( 71) ! derivative level
*     lvlExi    = iw( 72) ! >   =0 exit feasible on error
*     lvlInf    = iw( 73) ! Elastic option
*     lvlPrt    = iw( 74) ! Print Level for the minor itns
*     lvlScl    = iw( 75) ! scale option
*     lvlSch    = iw( 76) ! > 0    => gradient line search
*     lvlTim    = iw( 77) ! Timing level
*     lvlVer    = iw( 78) ! Verify level
*-->            = iw( 79) !
*     lprDbg    = iw( 80) ! > 0    => private debug print
*     lprPrm    = iw( 81) ! > 0    =>  parms are printed
*     lprSch    = iw( 82) ! line search debug starting itn
*     lprScl    = iw( 83) ! > 0    => print the scales
*     lprSol    = iw( 84) ! > 0    =>  print the solution 
*-->            = iw( 85) !
*-->            = iw( 86) !
*     minmax    = iw( 87) ! 1, 0, -1  => MIN, FP, MAX
*     iCrash    = iw( 88) ! Crash option
*     itnlim    = iw( 89) ! limit on total iterations
*     mMajor    = iw( 90) ! limit on major iterations
*     mMinor    = iw( 91) ! limit on minor iterations
*     MjrPrt    = iw( 92) ! Major print level
*     MnrPrt    = iw( 93) ! Minor print level
*     nParPr    = iw( 94) ! # of partial pricing sections
*-->            = iw( 95) !
*-->            = iw( 96) !
*-->            = iw( 97) !
*     jverif(1) = iw( 98) ! (1) stop and start columns
*     jverif(2) = iw( 99) ! (2) stop and start columns
*     jverif(3) = iw(100) ! (3) stop and start columns
*     jverif(4) = iw(101) ! (4) stop and start columns
*-->            = iw(102) !
*-->            = iw(103) !
*-->            = iw(104) !
*     lDenJ     = iw(105) ! 1(2)    => dense(sparse) Jacobian
*     mEr       = iw(106) ! maximum # errors in MPS data
*     mLst      = iw(107) ! maximum # lines  of MPS data
*     nProb     = iw(108) ! problem number
*-->            = iw(109) !
*-->            = iw(110) !
*-->            = iw(111) !
*-->            = iw(112) !
*-->            = iw(113) !
*-->            = iw(114) !
*-->            = iw(115) !
*-->            = iw(116) !
*-->            = iw(117) !
*-->            = iw(118) !
*-->            = iw(119) !
*     iBack     = iw(120) ! backup file
*     iDump     = iw(121) ! dump file
*     iLoadB    = iw(122) ! load file
*     iMPS      = iw(123) ! MPS file
*     iNewB     = iw(124) ! new basis file
*     iInsrt    = iw(125) ! insert file
*     iOldB     = iw(126) ! old basis file
*     iPnch     = iw(127) ! punch file
*-->            = iw(128) !
*-->            = iw(129) !
*     iReprt    = iw(130) ! report file
*     iSoln     = iw(131) ! solution file
*-->            = iw(132) !
*     maxm      = iw(133) ! Estimate of the number of rows
*     maxn      = iw(134) ! Estimate of the number of columns
*     maxne     = iw(135) ! Estimate of the number of elements
*     ------------------------------------------------------------------
*     iw(151)--iw(180) contain luparm parameters for LUSOL.
*     ------------------------------------------------------------------
*     nout      = iw(151) ! unit # for printed messages
*     LUprnt    = iw(152) ! print level in LU routines
*     maxcol    = iw(153) ! lu1fac: max. # columns
*-->            = iw(154) !
*-->            = iw(155) !
*-->            = iw(156) !
*-->            = iw(157) !
*     keepLU    = iw(158) !
*-->            = iw(159) !
*     iErr      = iw(160) ! LU error flag
*     nSing     = iw(161) ! # of singularities in w(*)
*     jsing     = iw(162) ! col. no. of last singularity
*     minlen    = iw(163) ! minimum recommended lena
*     maxlen    = iw(164) !
*     nupdat    = iw(165) ! # of updates by lu8
*     nrank     = iw(166) ! # of nonempty rows of U
*     nDens1    = iw(167) ! # cols undone when dnsty>   =Dens1
*     nDens2    = iw(168) ! # cols undone when dnsty>   =Dens2
*     jUmin     = iw(169) ! col of B corres. to dUmin
*     numL0     = iw(170) ! # columns in initial  L
*     lenL0     = iw(171) ! size of L0
*     lenU0     = iw(172) ! size of initial  U
*     lenL      = iw(173) ! size of current  L
*     lenU      = iw(174) ! size of current  U
*     lrow      = iw(175) ! length of row file
*     ncp       = iw(176) ! no. of compressions
*     mersum    = iw(177) ! tot. of Markowitz merit counts
*     nUtri     = iw(178) ! lu1fac: # of tri. rows in U
*     nLtri     = iw(179) ! lu1fac: # of tri. rows in L
*-->            = iw(180) !
*     ------------------------------------------------------------------
*     iw(181)--iw(199) pass parameters into various routines.
*     ------------------------------------------------------------------
*               = iw(181) !
*     lvlDif    = iw(182) !    =1 (2) for forwd (cntrl) diffs
*     nConfd    = iw(183) ! # of unknown elements of gCon 
*     nObjfd    = iw(184) ! # of unknown elements of gObj
*-->            = iw(185) !
*     nGotg(1)  = iw(186) ! number of g    elements set
*     nGotg(2)  = iw(187) ! number of gCon elements set
*-->            = iw(188) !
*     nfCon(1)  = iw(189) ! number of calls of fCon
*     nfCon(2)  = iw(190) ! number of calls of fCon
*     nfCon(3)  = iw(191) ! number of calls of fCon
*     nfCon(4)  = iw(192) ! number of calls of fCon
*-->            = iw(193) !
*     nfObj(1)  = iw(194) ! number of calls of fObj
*     nfObj(2)  = iw(195) ! number of calls of fObj
*     nfObj(3)  = iw(196) ! number of calls of fObj
*     nfObj(4)  = iw(197) ! number of calls of fObj
*-->            = iw(198) !
*     minimz    = iw(199) ! 1 (-1)    => minimize (maximize)
*     ifDefH    = iw(200) ! =1(0) for definite QP Hessian
*     nQNmod    = iw(201) ! # of BFGS updates since last reset
*-->            = iw(202) !
*-->            = iw(203) !
*-->            = iw(204) !
*-->            = iw(205) !
*-->            = iw(206) !
*     maxvi     = iw(207) ! index of maximum violation
*-->            = iw(208) !
*-->            = iw(209) !
*     nFac      = iw(210) ! # of LU factorizations
*     nBfac     = iw(211) ! # consecutive  `B' factorizes
*-->            = iw(212) !
*     lena      = iw(213) ! space allotted for LU factors
*     LUreq     = iw(214) ! Reason for refactorization
*     LUitn     = iw(215) ! itns since last factorize
*     LUmod     = iw(216) ! number of LU mods
*-->            = iw(217) !
*     iObj      = iw(218) ! position of the objective row in A
*-->            = iw(219) !
*     kObj      = iw(220) ! xBS(kObj) is the obj. slack
*-->            = iw(221) !
*-->            = iw(222) !
*     MnrHdg    = iw(223) ! >0    => Mnr heading for iPrint
*     MjrHdg    = iw(224) ! >0    => Mjr heading for iPrint
*     MjrSum    = iw(225) ! >0    => Mjr heading for iSumm
*-->            = iw(226) !
*-->            = iw(227) !
*     iPrintx   = iw(228) ! Global value of iPrint
*     iSummx    = iw(229) ! Global value of iSumm
*-->            = iw(230) !
*-->            = iw(231) !
*-->            = iw(232) !
*     nName     = iw(233) ! # of row and col. names
*-->            = iw(234) !
*-->            = iw(235) !
*-->            = iw(236) !
*-->            = iw(237) !
*     iALONE    = iw(238) ! > 0     =  stand-alone
*-->            = iw(239) !
*-->            = iw(240) !
*     ipage1    = iw(241) ! > 0 
*     ipage2    = iw(242) ! > 0 
*-->            = iw(243) ! > 0 
*-->            = iw(244) ! > 0 
*     itn       = iw(245) ! Iteration count. Used by the AMPL interface
*     ------------------------------------------------------------------
*     iw(251)--iw(330) hold array addresses.
*     ------------------------------------------------------------------
*     la        = iw(251) ! a(ne) holds the MPS aij
*     lha       = iw(252) ! ha(ne) row numbers for aij
*     lka       = iw(253) ! ka(n+1)    = column pointers
*     lbl       = iw(254) ! bl(nb)    =  lower bounds
*     lbu       = iw(255) ! bu(nb)    =  upper bounds
*     lxs       = iw(256) ! xs(nb)    = the solution (x,s)
*     lpi       = iw(257) ! pi(m)    = the pi-vector
*     lrc       = iw(258) ! rc(nb)    = the reduced costs
*     lhs       = iw(259) ! the column state vector
*     lhElas    = iw(260) ! hElast(nb) list of elastic vars
*     lNames    = iw(261) ! Names(nName)
*-->            = iw(262) !
*-->            = iw(263) !
*-->            = iw(264) !
*-->            = iw(265) !
*-->            = iw(266) !
*     lhfeas    = iw(267) ! hfeas(mBS), feasibility types
*     lhEsta    = iw(268) ! hEstat(nb), status of elastics
*     lkBS      = iw(269) ! kBS(mBS), ( B  S ) list
*     lblBS     = iw(270) ! blBS(mBS), lower bounds for xBS
*     lbuBS     = iw(271) ! buBS(mBS), upper bounds for xBS
*     lxBS      = iw(272) ! xBS(mBS), basics, superbasics 
*     lgBS      = iw(273) ! gBS(mBS)
*     laScal    = iw(274) ! aScale(*), usually length nb
*     lxScl     = iw(275) ! xScl(nx2), nx2 = nnL or 0
*     lx2       = iw(276) ! x2(nb),  new xs.
*     lrg       = iw(277) ! rg (maxS), reduced gradient
*     lr        = iw(278) ! r(lenr)    = factor of Z'HZ
*     liy       = iw(279) ! iy(mBS)    =  work vector
*     liy2      = iw(280) ! iy2(m)    = work vector
*     ly        = iw(281) ! y(nb)    = usually the search dirn
*     ly2       = iw(282) ! y2(nb) work vector
*     lw        = iw(283) ! w(nb)    = QP/LP work vector
*     ly1       = iw(284) ! y1(m) satisfies  B y1    = a(jq)
*-->            = iw(285) !
*-->            = iw(286) !
*-->            = iw(287) !
*-->            = iw(288) !
*     lgObjQ    = iw(289) ! gObjQP(nnL) QP gradient
*     lrhs      = iw(290) ! rhs(m) of QP constraints 
*-->            = iw(291) !
*-->            = iw(292) !
*-->            = iw(293) !
*-->            = iw(294) !
*     lHp       = iw(295) ! Hp(nnL)    = H times p
*     lxdif     = iw(296) ! xdif(nb)    =  x2 - xs
*     lgdif     = iw(297) ! gdif(nnL)    = gradient difference 
*     lgObj     = iw(298) ! gObj (nnObj)    = obj. gradient
*     lgObjU    = iw(299) ! record of unknown derivatives
*     lgObj1    = iw(300) ! gObj1(nnObj)    = base point gObj
*     lgObj2    = iw(301) ! gObj2(nnObj)    = work gObj
*     lfCon     = iw(302) ! fCon (nnCon)    = constr. values
*     lfCon1    = iw(303) ! fCon1(nnCon) fCon at x1
*     lfCon2    = iw(304) ! fCon2(nnCon)    = work vector
*     lgCon     = iw(305) ! gCon (neJac)    = the Jacobian
*     lgConU    = iw(306) ! gConU(neJac)  const elements
*     lgCon2    = iw(307) ! gCon2(neJac)    = user-def gCon
*-->            = iw(308) !
*-->            = iw(309) !
*-->            = iw(310) !
*-->            = iw(311) !
*     lkg       = iw(312) ! kg(nnJac+1) column pointers
*     lviol     = iw(313) ! viol    = c(x) + A(linear)x - s
*     lblTru    = iw(314) ! blTru(nb)    = temp bounds
*     lbuTru    = iw(315) ! buTru(nb)    = temp bounds
*     lxMul     = iw(316) ! xMul(nnCon)  = multpls. for fCon
*     lxMul2    = iw(317) ! xMul2(nnCon) =  new xMul
*     lxdMul    = iw(318) ! xdMul(nnCon) =  xMul2 - xMul
*     lxPen     = iw(319) ! xPen(nnCon)  = penalty params
*     lyslk     = iw(320) ! yslk(nnCon) true nonlinear row
*     lxfeas    = iw(321) ! xfeas(nb)    = QP feasible pt.
*-->            = iw(322) !
*-->            = iw(323) !
*-->            = iw(324) !
*-->            = iw(325) !
*-->            = iw(326) !
*     lkeynm    = iw(327) ! keynm(lenh)    = hash table keys
*     ------------------------------------------------------------------
*     iw(331)--iw(350) contain array addresses for LUsol.
*     ------------------------------------------------------------------
*     maxLUi    = iw(331) ! max LU nonzeros in iw(*)
*     maxLUr    = iw(332) ! max LU nonzeros in rw(*)
*     ip        = iw(333) !
*     iq        = iw(334) !
*     lenc      = iw(335) !
*     lenri     = iw(336) !
*     locc      = iw(337) !
*     locr      = iw(338) !
*     iploc     = iw(339) !
*     iqloc     = iw(340) !
*     LUa       = iw(341) !
*-->            = iw(342) !
*     indc      = iw(343) !
*     indr      = iw(344) !
*     ------------------------------------------------------------------
*     iw(361)--iw(370) contain variables associated with the QN update.
*     ------------------------------------------------------------------
*     nQNmod    = iw(201) ! # of BFGS updates since last reset
*     nSkip     = iw(388) ! # of consecutive skipped updates
*-->            = iw(387) !
*     nFlush    = iw(389) ! Iterations since last Hessian flush
*-->            = iw(390) !
*-->            = iw(361) !
*-->            = iw(362) !
*-->            = iw(363) !
*-->            = iw(364) !
*-->            = iw(365) !
*-->            = iw(366) !
*-->            = iw(367) !
*-->            = iw(368) !
*-->            = iw(369) !
*-->            = iw(370) !
*     ------------------------------------------------------------------
*     iw(371)--iw(380) contain array addresses for FM QN updates.
*     ------------------------------------------------------------------
*     lH        = iw(371) !
*     lenH      = iw(372) !
*     ------------------------------------------------------------------
*     iw(381)--iw(390) contain array addresses for LM QN updates.
*     ------------------------------------------------------------------
*     lH0       = iw(381) !
*     lgdsav    = iw(382) !
*     lHpsav    = iw(383) !
*     lyts      = iw(384) !
*     lptHp     = iw(385) !
*-->            = iw(386) !
*-->            = iw(387) !
*-->            = iw(388) !
*-->            = iw(389) !
*-->            = iw(390) !
*     ------------------------------------------------------------------
*     iw(400)--iw(450) information for solvers calling SNOPT
*     ------------------------------------------------------------------
*     itn       = iw(400) ! AMPL: iteration count.
*     ------------------------------------------------------------------
*     iw(451)--iw(500) timing information
*     ------------------------------------------------------------------
*     numt( 1)  = iw(451) ! Input time                          
*     numt( 2)  = iw(452) ! Solve time          
*     numt( 3)  = iw(453) ! Output time         
*     numt( 4)  = iw(454) ! Constraint functions
*     numt( 5)  = iw(455) ! Nonlinear objective 
*     numt( 6)  = iw(456) ! not used
*     numy( 7)  = iw(457) ! not used
*     numt( 8)  = iw(458) ! not used
*     numt( 9)  = iw(459) ! not used
*     numt(10)  = iw(460) ! not used
*     ==================================================================
*     Character*8  workspace.
*     cw(51)--cw(150): optional parameters set via the specs file.
*     ==================================================================
*     mProb     = cw( 51) ! Problem name
*     mObj      = cw( 52) ! Objective name
*     mRhs      = cw( 53) ! 
*     mRng      = cw( 54) ! 
*     mBnd      = cw( 55) ! 
*-->            = cw( 56) !
*-->            = cw( 57) !
*-->            = cw( 58) !
*-->            = cw( 59) !
*-->            = cw( 60) !
*     ==================================================================
