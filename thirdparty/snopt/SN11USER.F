*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*     
*     File  sn11user.f 
*
*     s1User
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 
      subroutine s1User( iAbort, MjrMsg, KTcond, 
     $                   m, n, nb, nR, nS, nMajor, nMinor, nSwap,
     $                   condHz, duInf, emaxS, fObj, 
     $                   fMrt, gMrt, PenNrm, prInf, step,
     $                   vimax, dxnrm, dxrel,
     $                   ne, nka, a, ha, ka,
     $                   hs, bl, bu, pi, rc, xs, 
     $                   cu, lencu, iu, leniu, ru, lenru, 
     $                   cw, lencw, iw, leniw, rw, lenrw )
 
      implicit           double precision (a-h,o-z)
      logical            KTcond(2)
      character*6        MjrMsg
      integer            ha(ne), hs(nb)
      integer            ka(nka)
      double precision   a(ne)
      double precision   bl(nb), bu(nb)
      double precision   rc(nb), xs(nb)
      double precision   pi(m)

      character*8        cu(lencu), cw(lencw)
      integer            iu(leniu), iw(leniw)
      double precision   ru(lenru), rw(lenrw)

*     ==================================================================
*     s1User can be used to print additional information for the user
*     and/or to set a flag that will cause SNOPT to terminate.
*
*     With the exception of iAbort, all variables must be of type INPUT.
*
*     iAbort   (integer) can be used to abort the current run.
*                        On entry,    iAbort =  0
*                        On exit,  if iAbort ne 0, SNOPT is terminated.
*                          
*
*     MjrMsg   (character*6) status string of major iteration.
*                       _________________________
*       Character       | 1 | 2 | 3 | 4 | 5 | 6 |
*                       -------------------------
*       Entries           M   r   l   c   u 
*                         m   R   d       t   i
*                         s       D 
*                         S    
*                         n
*
*                          
*     Char 1      M -- an extra evaluation of the problem functions
*     ------           was used to define an acceptable update.
*                 m -- as in ``M'' except that it was necessary to
*                      modify the update to include an augmented
*                      Lagrangian term. 
*                 s -- self-scaled BFGS update was made. Usually after   
*                      after the Hessian is reset to a diagonal.
*                 S -- as in ``s'' except that it was necessary to 
*                      modify the update.
*                 n -- no BFGS update could be made.
*
*     Char 2      r -- complete Hessian was reset after mskip majors
*     ------           after mskip majors with no BFGS update 
*                 R -- user-requested periodic reset of the limited
*                      memory updates.
*
*     Char 3      l -- the change in the norm of the variables was
*     ------           limited by the ``step limit'' parameter.
*
*     Char 4      c -- central differences computed.
*     ------
*
*     Char 5      u -- unbounded QP subproblem
*     ------      t -- the minor iterations were terminated at the
*                      minor iteration limit.

*     Char 6      i -- infeasible QP subproblem.
*     ------ 
*     KTcond   (logical) array with 2 elements. Indicates if the KKT
*                        conditions are satisfied to within the
*                        user-specified tolerance.
*
*     m        (integer) is the number of linear and nonlinear general
*                        constraints.
*
*     n        (integer) is the number of variables, excluding slacks.
*
*     nb       (integer) is n + m, the number of bounds in bl or bu.
*
*     nR       (integer) is the number of columns of the transformed
*                        Hessian. nR includes nS columns of the reduced
*                        Hessian.
*
*     nS       (integer) is the number of superbasic variables, i.e.,
*                        the number of degrees of freedom.
*
*     nMajor   (integer) is the number of major iterations.
*
*     nMinor   (integer) is the number of minor iterations neede for the
*                        last QP subproblem.
*
*     nSwap    (integer) number of column swaps in the BS factorization.
*
*     condHz   (double)  is a lower bound on the condition number of 
*                        the reduced Hessian.
*
*     duInf    (double)  largest dual infeasibility.
*
*     fmrt     (double)  is the value of the merit function.
*
*     gmrt     (double)  is the directional derivative of the 
*                        merit function (i.e., with respect to alpha).
*
*     PenNrm   (double)  is the two-norm of the vector of merit function
*                        penalty parameters.
*
*     step     (double)  is the line search step size, alpha.
*
*     vimax    (double)  is the infinity-norm of the nonlinear
*                        constraint violations
*
*     virel    (double)  is the norm of the constraint violations,
*                        relative to x, vimax / (1 + norm(x)).
*
*     dxnrm    (double)  is the two-norm of the search direction
*                        norm(delta x).
*
*     dxrel    (double)  is the norm of the search direction relative to
*                        x, i.e., dxnrm / (1 + norm(x)).
*
*     ne       (integer) is the number of nonzeros in the constraint
*                        Jacobian.
*
*     nka      (integer) is the number of elements in the array ka(*).
*
*     a(ne)    (double)  holds the nonzeros of the general constraint
*                        matrix A.
*
*     ha(ne)   (integer) holds the row indices of elements in a(*).
*
*     ka(nka)  (integer) holds the pointers to the start of each column
*                        of A in a(*) and ha(*).
*
*     hs(nb)   (integer) defines the current status of the variables 
*                        and slacks (x,s). See the MINOS/SNOPT document
*                        for more information.
*
*     bl(nb)   (double)  holds the lower bounds on (x,s).
*
*     bu(nb)   (double)  holds the upper bounds on (x,s).
*
*     pi(m)    (double)  is the vector dual variables (i.e., Lagrange
*                        multipliers.) associated with the constraints
*                        Ax=b.
*
*     rc(nb)   (double)  holds the reduced costs, g - (A -I)'*pi. 
*                        The last m entries contain pi, 
*                        i.e. rc(n+1:n+m) = pi.
*
*     xs(nb)   (double)  holds the variables and slacks (x,s).
*
*     iw(leniw)(integer) is the integer work space array.
*
*     leniw    (integer) is the length of iw array.
*
*     rw(lenrw)(double)  is the double precision work space array.
*
*     lenrw    (integer) is the length of the rw array.
*     ==================================================================
 
*     relax

*     if (nMajor .ge. 4) then
*        iAbort = 1
*     end if

*     end of s1User
      end


