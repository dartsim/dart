/*
RTQL8, Copyright (_dright) 2011 Georgia Tech Graphics Lab
All rights reserved.

Author	Sumit Jain
Date		07/21/2011
*/

#ifndef MATH_SOLVERTRIDIAGONAL_H
#define MATH_SOLVERTRIDIAGONAL_H

#include <Eigen/Dense>

namespace math{
	void SolveTridiagonal(const Eigen::VectorXd &_dleft, const Eigen::VectorXd &_diag, Eigen::VectorXd &_dright, Eigen::VectorXd &_rhs, Eigen::VectorXd &_soln);
    void SolveTridiagonal(const Eigen::MatrixXd &_lhs, const Eigen::VectorXd &_rhs, Eigen::VectorXd &_soln);
}

#endif  //MATH_SOLVERTRIDIAGONAL_H
