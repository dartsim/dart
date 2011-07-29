/*
  RTQL8, Copyright (_dright) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sumit Jain
  Date		07/28/2011
*/
#include "SolverTridiagonal.h"

using namespace Eigen;

namespace math{

    void SolveTridiagonal(const VectorXd &_dleft, const VectorXd &_diag, VectorXd &_dright, VectorXd &_rhs, VectorXd &_soln){
        unsigned int i;

        /* Modify the coefficients. */
        _dright[0] /= _diag[0];    /* Division by zero risk. */
        _rhs[0] /= _diag[0];    /* Division by zero would imply a singular matrix. */
        unsigned n = _dleft.size();
        for(i = 1; i < n; i++){
            double id = (_diag[i] - _dright[i-1] * _dleft[i]);  /* Division by zero risk. */
            _dright[i] /= id;            /* Last value calculated is redundant. */
            _rhs[i] = (_rhs[i] - _rhs[i-1] * _dleft[i])/id;
        }

        /* Now back substitute. */
        _soln[n - 1] = _rhs[n - 1];
        for(i = n - 2; i >= 0; i--)
            _soln[i] = _rhs[i] - _dright[i] * _soln[i + 1];
    }


    void SolveTridiagonal(const MatrixXd &_lhs, const VectorXd &_rhs, VectorXd &_soln) {
        VectorXd dleft = VectorXd::Zero(_lhs.rows());
        VectorXd diag = VectorXd::Zero(_lhs.rows());
        VectorXd dright = VectorXd::Zero(_lhs.rows());
        VectorXd rhsmod = _rhs;

        unsigned int n = _lhs.rows();
        for (unsigned int i = 0; i < n; i++) {
            diag[i] = _lhs(i, i);
            if (i == 0) dleft[i] = 0;
            else dleft[i] = _lhs(i, i-1);

            if (i == n-1) {
                dright[i] = 0;
            }
            else dright[i] = _lhs(i, i+1);
        }

        SolveTridiagonal(dleft, diag, dright, rhsmod, _soln);
    }

}   //namespace math
