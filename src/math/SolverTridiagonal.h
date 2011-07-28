/*
RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
All rights reserved.

Author	Sumit Jain
Date		07/21/2011
*/

#ifndef MATH_SOLVERTRIDIAGONAL_H
#define MATH_SOLVERTRIDIAGONAL_H

#include <Eigen/Dense>

namespace math{
	void SolveTridiagonal(const Eigen::VectorXd &a, const Eigen::VectorXd &b, Eigen::VectorXd &c, Eigen::VectorXd &d, unsigned int n, Eigen::VectorXd &x);
    void SolveTridiagonal(const Eigen::MatrixXd &_lhs, const Eigen::VectorXd &_rhs, Eigen::VectorXd &_soln);
}

#endif  //MATH_SOLVERTRIDIAGONAL_H
