/*
RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
All rights reserved.

Author	Sumit Jain
Date		07/21/2011
*/
#include "SolverTridiagonal.h"

using namespace Eigen;

namespace math{

    void SolveTridiagonal(const VectorXd &a, const VectorXd &b, VectorXd &c, VectorXd &d, unsigned int n, VectorXd &x){
        unsigned int i;

        /* Modify the coefficients. */
        c[0] /= b[0];    /* Division by zero risk. */
        d[0] /= b[0];    /* Division by zero would imply a singular matrix. */
        for(i = 1; i < n; i++){
            double id = (b[i] - c[i-1] * a[i]);  /* Division by zero risk. */
            c[i] /= id;            /* Last value calculated is redundant. */
            d[i] = (d[i] - d[i-1] * a[i])/id;
        }

        /* Now back substitute. */
        x[n - 1] = d[n - 1];
        for(i = n - 2; i >= 0; i--)
            x[i] = d[i] - c[i] * x[i + 1];
    }


    void SolveTridiagonal(const MatrixXd &_lhs, const VectorXd &_rhs, VectorXd &_soln) {
        VectorXd a = VectorXd::Zero(_lhs.rows());
        VectorXd b = VectorXd::Zero(_lhs.rows());
        VectorXd c = VectorXd::Zero(_lhs.rows());
        VectorXd d = VectorXd::Zero(_lhs.rows());

        unsigned int n = _lhs.rows();
        for (unsigned int i = 0; i < n; i++) {
            b[i] = _lhs(i, i);
            if (i == 0) a[i] = 0;
            else a[i] = _lhs(i, i-1);

            if (i == n-1) {
                c[i] = 0;
            }
            else c[i] = _lhs(i, i+1);
            d[i] = _rhs[i];
        }

        SolveTridiagonal(a,b,c,d,n, _soln);
    }

}   //namespace math
