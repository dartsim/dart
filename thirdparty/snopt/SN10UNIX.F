*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*     
*     File  sn10unix.f 
*
*     s1file
*     s1clos   s1envt   s1eps    s1flmx   s1flmn
*     s1open   s1page
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s1file( job, iw, leniw )

      implicit           double precision (a-h,o-z)
      character          job*(*)
      integer            iw(leniw)

*     ------------------------------------------------------------------
*     s1file  is a machine-dependent routine for opening various files.
*     It calls s1open (which is also machine-dependent).
*
*     SNOPT uses sequential files only
*     and does not need to read and write to the same file.
*
*     iSpecs, iPrint, iSumm  are defined in snopt1 or snInit
*     iRead                  is defined here.
*
*     iRead and iPrint have the following use:
*        Input  files (MPS, Old Basis, Insert, Load)
*        are rewound after being read,
*        but not if they are the same as  iRead.
*        Output files (Backup, New Basis, Punch, Dump, Solution, Report)
*        are rewound after being written,
*        but not if they are the same as  iPrint.
*     
*     iRead  = (conceptually) the Card Reader or the Keyboard that
*              can't be rewound.  SNOPT does not use this file,
*              so there is no 'open'.
*     iSumm  = the SUMMARY file.  Sometimes this is the screen.
*              If so, it may not need to be opened.
*     iSpecs = the SPECS file, containing one or more problem specs.
*              This file is not rewound after use, because it may
*              contain another SPECS file.
*
*     Here are all the files used by SNOPT.
*     The associated Index is passed to s1open
*     and must match the list of names in s1open, if that routine
*     uses method = 1.
*
*        Unit    Index    Description       Status
*        iSpecs     1     Specs     file    In
*        iPrint     2     Print     file    Out
*        iSumm      3     Summary   file    Out
*        iMPS       4     MPS       file    In
*        iOldB      5     Old Basis file    In
*        iInsrt     6     Insert    file    In
*        iLoadB     7     Load      file    In
*        iBack      8     Backup    file    Out
*        iNewB      9     New Basis file    Out
*        iPnch     10     Punch     file    Out
*        iDump     11     Dump      file    Out
*        iSoln     12     Solution  file    Out
*        iReprt    13     Report    file    Out
*        iRead            Not opened, but used as described above
*
*     15-Nov-91: First version based on Minos 5.4 routine mifile.
*     03-Oct-97: Modified to comply with Minos 5.5.
*     02-Feb-98: Current version.
*     ------------------------------------------------------------------
      parameter         (iRead     =  10)
      parameter         (iMPS      = 123)
      parameter         (iPrinx    = 228)
      parameter         (iSummx    = 229)
*     ------------------------------------------------------------------
      iSpecs    = iw( 11)
      iPrint    = iw( 12)
      iSumm     = iw( 13)

      iBack     = iw(120)
      iDump     = iw(121)
      iLoadB    = iw(122)
      iNewb     = iw(124)
      iInsrt    = iw(125)
      iOldb     = iw(126)
      iPnch     = iw(127)
      iReprt    = iw(130)
      iSoln     = iw(131)

*     ------------------------------------------------------------------
*-->  Machine dependency.
*     Set iRead = some input unit number that should not be rewound.
*     Set iRead = 0 if this is irrelevant.
*     ------------------------------------------------------------------
      iw(iRead) = 5

      if (job(1:1) .eq. 'D') then
*        ---------------------------------------------------------------
*        job = 'D'efault: Open the Specs, Print and Summary files.
*        iSpecs         remains the same throughout the run.
*        iPrint, iSumm  may be altered by the SPECS file.  They may
*                       need to be opened by both job `D' and `O'.
*        ---------------------------------------------------------------
         iw(iPrinx) = iPrint
         iw(iSummx) = iSumm
         call s1open( iSpecs, 1, 'IN ' )
         call s1open( iPrint, 2, 'OUT' )
         call s1open( iSumm , 3, 'OUT' )

      else if (job(1:1) .eq. 'O') then      
*        ---------------------------------------------------------------
*        Job = 'O'pen: Open files mentioned in the SPECS file just read.
*        Input files are opened first.  Only one basis file is needed.
*        ---------------------------------------------------------------
         if (iw(iMPS) .le. 0     )              iw(iMPS) = iSpecs
         if (iw(iMPS) .ne. iSpecs) call s1open( iw(iMPS),  4, 'IN ' )

         if      (iOldB  .gt. 0) then
                                   call s1open( iOldB   ,  5, 'IN ' )
         else if (iInsrt .gt. 0) then
                                   call s1open( iInsrt  ,  6, 'IN ' )
         else if (iLoadB .gt. 0) then
                                   call s1open( iLoadB  ,  7, 'IN ' )
         end if

                                   call s1open( iBack   ,  8, 'OUT' )
                                   call s1open( iNewB   ,  9, 'OUT' )
                                   call s1open( iPnch   , 10, 'OUT' )
                                   call s1open( iDump   , 11, 'OUT' )
                                   call s1open( iSoln   , 12, 'OUT' )
                                   call s1open( iReprt  , 13, 'OUT' )

*        Open new Print or Summary files if they were altered
*        by the Specs file.

         if (iPrint .ne. iw(iPrinx)) call s1open( iPrint, 2, 'OUT' )
         if (iSumm  .ne. iw(iSummx)) call s1open( iSumm , 3, 'OUT' )
      end if

*     Check that output files are different from Specs or MPS.

      if (iPrint .gt. 0) then
         if (iSpecs .gt. 0) then
            if (iBack  .eq. iSpecs  ) write(iPrint, 1000) 'Backup'
            if (iNewB  .eq. iSpecs  ) write(iPrint, 1000) 'New Basis'
            if (iPnch  .eq. iSpecs  ) write(iPrint, 1000) 'Punch'
            if (iDump  .eq. iSpecs  ) write(iPrint, 1000) 'Dump'
            if (iSoln  .eq. iSpecs  ) write(iPrint, 1000) 'Solution'
            if (iReprt .eq. iSpecs  ) write(iPrint, 1000) 'Report'
         end if

         if (iw(iMPS) .gt. 0) then
            if (iBack  .eq. iw(iMPS)) write(iPrint, 2000) 'Backup'
            if (iNewB  .eq. iw(iMPS)) write(iPrint, 2000) 'New Basis'
            if (iPnch  .eq. iw(iMPS)) write(iPrint, 2000) 'Punch'
            if (iDump  .eq. iw(iMPS)) write(iPrint, 2000) 'Dump'
            if (iSoln  .eq. iw(iMPS)) write(iPrint, 2000) 'Solution'
            if (iReprt .eq. iw(iMPS)) write(iPrint, 2000) 'Report'
         end if
      end if

      return

 1000 format(/ ' ===>  Warning:',
     $   ' the Specs file and ', a, ' file are on the same unit')
 2000 format(/ ' ===>  Warning:',
     $   ' the  MPS  file and ', a, ' file are on the same unit')

*     end of s1file
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s1clos( lun )

*     ==================================================================
*     s1clos  closes the file with logical unit number lun.
*     This version is trivial and so far is not even used by SNOPT.
*     Perhaps some implementations will need something fancier.
*     ==================================================================

      close( lun )

*     end of s1clos
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s1envt( mode, iw, leniw )

      implicit           double precision (a-h,o-z)
      integer            iw(leniw)

*     ==================================================================
*     s1envt specifies the environment within which SNOPT is being used.
*
*     When mode = 0, information about the environment should be
*     initialized.
*     
*     iPage1 says whether new pages are ever wanted on file iPrint.
*     iPage2 says whether new pages are ever wanted on file iSumm.
*
*     When mode is in the range 1 to 99, each environment does its
*     own thing.
*
*     The only environment at present is:
*     ALONE:
*     This means SNOPT is in stand-alone mode---the normal case.
*     Nothing special is done.
*
*     16 Sep 1987.
*     ==================================================================
      parameter         (iALONE    = 238)
      parameter         (ipage1    = 241)
      parameter         (ipage2    = 242)
*     ------------------------------------------------------------------
      if (mode .le. 0) then
*        ---------------------------------------------------------------
*        mode = 0.    Initialize.
*        ---------------------------------------------------------------
         iw(iALONE) = 1
         iw(iPage1) = 1
         iw(iPage2) = 0

      else

*        Relax, do nothing in this version.

      end if

*     end of s1envt
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      double precision   function s1eps ( )
      implicit           double precision (a-h,o-z)
*
*     Compute the machine precision.
*     IEEE Floating point double precision. 
*

      nbase  = 2
      ndigit = 53
      base   = nbase
      u      = base**(- ndigit)
      s1eps  = 2.0d+0*u

*     end of s1flmx
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      double precision   function s1flmx( )
      implicit           double precision (a-h,o-z)
*
*     IEEE Floating point double precision. 
*
      s1flmx = 1.7977d+307

*     end of s1flmx
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      double precision   function s1flmn( )
      implicit           double precision (a-h,o-z)
*
*     IEEE Floating point double precision. 
*
      s1flmn = 2.2251d-308

*     end of s1flmx
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s1open( lun, index, state )

      integer            lun, index
      character*3        state

*     ------------------------------------------------------------------
*     s1open  is a machine-dependent routine.
*     In principal it opens a file with logical unit number lun
*     and positions it at the beginning.
*
*     Input files are treated that way.
*     An input file is opened with status='OLD'
*     (and F77 will terminate with an error if the file doesn't exist).
*
*     Output files are more machine-dependent.
*     With status='NEW', F77 would terminate if the file DID exist.
*     With status='UNKNOWN', existing files would be overwritten.
*     This is normal with Unix, but on systems that have file
*     version numbers (e.g. DEC OpenVMS), a new version is preferable.
*     It is then better not to open the file at all, but let a new
*     version be created by the first "write".
*
*     Nothing happens if
*     1. lun <= 0.  Saves us from testing lun before calling s1open.
*
*     2. Unit lun is already open.  Helps applications that call
*        snopt -- they can open files themselves if they want to.
*
*     3. lun = screen, where  screen  is a local machine-dependent
*        variable, typically 6.  It seems inadvisable to do an OPEN
*        on a file that is predefined to be an interactive screen.
*        With Unix on DECstations, open(6, file='snopt.sum') sends
*        output to file snopt.sum, not to the screen.
*     
*
*     lun     (input) is the unit number.
*
*     index   (input) points to one of the hardwired names below.
*             Used only if method = 1.
*
*     state   (input) is 'IN ' or 'OUT', indicating whether the file
*             is to be input or output.
*
*     15 Jul 1989: First version, follows some of the advice offered
*                  by David Gay, Bell Laboratories.
*     -- --- 1990: Added parameter "state".
*     03 Feb 1994: Added parameter "index" to help when method = 1.
*                  Local variable "method" must be set here to select
*                  various methods for naming files.
*                  Chris Jaensch, IFR Stuttgart, recommends not opening
*                  input files if they are already open.  This version
*                  ignores all open files (input or output), assuming
*                  that they are taken care of by the calling program.
*     13 May 1994: Local variable "screen" used to avoid opening screen.
*     ------------------------------------------------------------------

      logical            input, uopen
      integer            method, screen
      character*100      filnam

*     ------------------------------------------------------------------
*-->  Machine dependency.
*     names(*) is needed if method = 1 below.
*     Make sure "character*n" sets n big enough below.
*     It doesn't matter if it is bigger than necessary, since
*     "open( lun, file=name )" allows name to have trailing blanks.
*     ------------------------------------------------------------------

      character*9        names(13)
      data   names( 1) /'snopt.spc'/
      data   names( 2) /'snopt.prn'/
      data   names( 3) /'snopt.sum'/
      data   names( 4) /'snopt.mps'/
      data   names( 5) /'snopt.olb'/
      data   names( 6) /'snopt.ins'/
      data   names( 7) /'snopt.lod'/
      data   names( 8) /'snopt.bak'/
      data   names( 9) /'snopt.nwb'/
      data   names(10) /'snopt.pun'/
      data   names(11) /'snopt.dmp'/
      data   names(12) /'snopt.sol'/
      data   names(13) /'snopt.rpt'/

*     ------------------------------------------------------------------
*-->  Machine-dependency.
*     Set "method" to suit your operating system.
*     It determines how a file name is assigned to unit number "lun".
*     Typically,
*     method = 1 for fixed file names (e.g. PCs under DOS).
*     method = 2 for names like fort.7, fort.15 (e.g. Unix on Sun, SGI).
*     method = 3 for names like FTN07 , FTN15   (e.g. Unix on HP).
*     method = 4 if names are assigned by an external routine.
*     method = 5 if explicit file names are not needed (e.g. OpenVMS).
*     See more comments below.
*
*     Set "screen" to a unit number that never needs to be opened.
*     (Typically, screen = 6.  If unknown, set screen = 0.)
*     ------------------------------------------------------------------
      method = 1                ! Default value for SNOPT
*     method = 4                ! Uncomment when using SNOPT in AMPL 
      screen = 6

*     ------------------------------------------------------------------
*     Quit if lun<=0 or lun = iscreen or unit lun is already open.
*     ------------------------------------------------------------------
      if (lun .le. 0     ) go to 900
      if (lun .eq. screen) go to 900
      inquire( lun, opened=uopen )
      if (     uopen     ) go to 900

*     ------------------------------------------------------------------
*     Open file lun by a specified machine-dependent method.
*     ------------------------------------------------------------------
      input  = state .eq. 'IN '  .or.  state .eq. 'in '

      if (method .eq. 1) then
*        ---------------------------------------------------------------
*        Hardwired filename.
*        We use "index" to get it from names(*) above.
*        Typical machines: IBM PC under DOS.
*        ---------------------------------------------------------------
         if ( input ) then
            open( lun, file=names(index), status='OLD' )
         else
            open( lun, file=names(index), status='UNKNOWN' )
         end if

      else if (method .eq. 2) then
*        ---------------------------------------------------------------
*        Construct a name like fort.7 or fort.15.
*        (This approach suggested by Chris Jaensch, IFR Stuttgart.)
*        Typical machines:  Unix on Sun, SGI, DEC.
*        ---------------------------------------------------------------
         if (lun .le. 9) then
             write(filnam, '(a,i1)') 'fort.', lun
         else
             write(filnam, '(a,i2)') 'fort.', lun
         endif

         if ( input ) then

	    write (*,*) filnam

            open( lun, file=filnam, status='OLD' )
         else
            open( lun, file=filnam, status='UNKNOWN' )
         end if

      else if (method .eq. 3) then
*        ---------------------------------------------------------------
*        Construct a name like FTN07 or FTN15.
*        Typical machines:  Unix on HP.
*        ---------------------------------------------------------------
         if (lun .le. 9) then
             write(filnam, '(a,i1)') 'FTN0', lun
         else
             write(filnam, '(a,i2)') 'FTN', lun
         endif

         if ( input ) then
            open( lun, file=filnam, status='OLD' )
         else
            open( lun, file=filnam, status='UNKNOWN' )
         end if

      else if (method .eq. 4) then
*        ---------------------------------------------------------------
*        Assume some routine "getfnm" will provide a name at run-time.
*        (This approach is used by David Gay, Bell Labs.)
*        Typical machines:  Unix with command-line arguments.
*        Note that 'UNKNOWN' is equivalent to trying first with 'OLD',
*        and then with 'NEW' if the file doesn't exist.
*        ---------------------------------------------------------------
         if ( input ) then
            open( lun, file=filnam, form='formatted', status='OLD' )
         else
            open( lun, file=filnam, form='formatted', status='UNKNOWN' )
         end if

*-       When using SNOPT with AMPL, delete the previous if-then-else
*-       and replace by the following:
*-         
*-       call getfnm( lun, filnam )
*-       if ( input ) then
*-          open( lun, file=filnam(1:last), status='OLD' )
*-       else
*-          open( lun, file=filnam(1:last), status='UNKNOWN' )
*-       end if

      else if (method .eq. 5) then
*        ---------------------------------------------------------------
*        Explicit file names are not needed for the open statement.
*        The operating system uses a default name
*        (e.g. fort.7 or for007)
*        or has already assigned a name to this unit
*        (e.g. via a script or command file).
*        Typical machines:  Unix,
*                           DEC OpenVMS,
*                           IBM Mainframes.
*        ---------------------------------------------------------------
         if ( input ) then
            open( lun, status='OLD' )
         else
*           Let the first "write" do it.
         end if
      end if

*     ------------------------------------------------------------------
*     Rewind input files.
*     (Some systems position existing files at the end
*     rather than the beginning.)
*     err=900 covers files that have not yet been opened.
*     ------------------------------------------------------------------
      if ( input ) then
         rewind( lun, err=900 )
      end if

  900 return
      
*     end of s1open
      end

*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      subroutine s1page( mode, iw, leniw )
      implicit           double precision (a-h,o-z)
      integer            iw(leniw)

*     ------------------------------------------------------------------
*     s1page is an installation-dependent routine.  It is called at
*     points where some users might want output to files iPrint or iSumm
*     to begin on a new page.
*
*     iPage1 and iPage2 have already been set by s1envt.
*     If they are true, a page eject and a blank line are output.
*     Otherwise, just a blank line is output.
*
*     If mode = 0  and Summary level = 0, nothing is output to the 
*                  Summary file.  At present, this is so s8log
*                  will print just one line per major iteration, with
*                  no blank line in between.
*     If mode = 1, just the page control is relevant.
*     If mode = 2, SNOPT has encountered an error condition.
*                  At the moment, this case is treated the same as
*                  mode = 1.
*
*     15-Nov-91: First version based on Minos 5.4 routine m1page.
*     24-Dec-97: Current version of s1page.
*     ==================================================================
      iPrint = iw( 12)
      iSumm  = iw( 13)
      iPage1 = iw(241)
      iPage2 = iw(242)

      if (iPrint .gt. 0) then
         if (iPage1 .gt. 0) write(iPrint, 1001)
                            write(iPrint, 1002)
      end if

      if (iSumm  .gt. 0) then
         if (iPage2 .gt. 0) write(iSumm , 1001)
         if (mode   .ne. 0) write(iSumm , 1002)
      end if

      return

 1001 format('1')
 1002 format(' ')

*     end of s1page
      end


