/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
 * Date: 06/12/2011
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MATH_UTILSMATH_H
#define MATH_UTILSMATH_H

// Standard Libraries
#include <vector>
#include <cmath>
#include <ctime>
#include <climits>
#include <cassert>
#include <iostream>
using namespace std;
// External Libraries
#include <Eigen/Dense>
using namespace Eigen;
// Local Headers
#include "utils/Misc.h"

namespace dart_math {

// TODO: Is here right place?
typedef Matrix<double, 6, 1> Vector6d;

// a cross b = (CR*a) dot b
/* const Matd CR(2,2,0.0,-1.0,1.0,0.0); */
const Matrix2d CR( (Matrix2d() << 0.0, -1.0, 1.0, 0.0).finished() );

inline int delta(int i, int j) {
    if(i == j) return 1;
    return 0;
}

inline int sgn(double a){
    if(a<0) return -1;
    else if(a==0) return 0;
    else return 1;
}

inline double sqr(double x){
    return x*x;
}

inline double Tsinc(double theta){
    return 0.5-sqrt(theta)/48;
}

inline bool isZero(double theta){
    return(fabs(theta)<M_EPSILON);
}

inline double asinh(double X){
    return log(X + sqrt(X * X + 1));
}

inline double acosh(double X){
    return log(X + sqrt(X * X - 1));
}

inline double atanh(double X){
    return log((1 + X)/(1 - X))/ 2;
}

inline double asech(double X){
    return log((sqrt(-X * X + 1) + 1) / X);
}

inline double acosech(double X){
    return log((sgn(X) * sqrt(X * X + 1) +1) / X);
}

inline double acotanh(double X){
    return log((X + 1) / (X - 1)) / 2;
}

inline double round(double x){
    return floor(x+0.5);
}

inline double round2(double x){
    int gintx = (int)floor(x);
    if(x-gintx<0.5) return (double)gintx;
    else return (double)(gintx+1);
}

inline bool isEqual(double x, double y){
    return (fabs(x - y) < M_EPSILON);
}

// check if it is an integer
inline bool isInt(double x){
    if(isEqual(round(x), x)) return true;
    else return false;
}

inline double random( double min, double max ){
    return min + ((double)rand()/(RAND_MAX + 1.0)) * (max-min);
}

inline unsigned seedRand(){
    time_t now = time(0);
    unsigned char *p = (unsigned char *)&now;
    unsigned seed = 0;
    size_t i;

    for(i=0; i<sizeof(now); i++)
        seed = seed * (UCHAR_MAX + 2U ) + p[i];

    srand( seed );
    return seed;
}

//inline VectorXd xformHom(const MatrixXd& M, const VectorXd& _v) { ///< homogeneous transformation of the vector _v with the last value treated a 1
//    int n = _v.size();
//    assert(M.rows() == n + 1);
//    VectorXd augV(n + 1);
//    augV.head(n) = _v;
//    augV(n) = 1.0;

//    augV = M * augV;
//    VectorXd ret = augV.head(n);
//    return ret;
//}

inline Vector3d xformHom(const Matrix4d& _W, const Vector3d& _x) { ///< homogeneous transformation of the vector _v with the last value treated a 1
    //Vector4d x(_x(0), _x(1), _x(2), 1.0);
    //Vector4d y = _W * x;
    //return Vector3d(y(0), y(1), y(2));
    return _W.topLeftCorner<3,3>() * _x + _W.topRightCorner<3,1>();
}

inline Vector3d xformHomDir(const Matrix4d& _W, const Vector3d& _v) { ///< homogeneous transformation of the vector _v treated as a direction: last value 0
    return _W.topLeftCorner<3,3>() * _v;
}

inline Matrix3d makeSkewSymmetric(const Vector3d& v){
	Matrix3d result = Matrix3d::Zero();

	result(0, 1) = -v(2);
	result(1, 0) =  v(2);
	result(0, 2) =  v(1);
	result(2, 0) = -v(1);
	result(1, 2) = -v(0);
	result(2, 1) =  v(0);

	return result;
}

inline Vector3d fromSkewSymmetric(const Matrix3d& m) {
#if _DEBUG
    if (fabs(m(0, 0)) > M_EPSILON || fabs(m(1, 1)) > M_EPSILON || fabs(m(2, 2)) > M_EPSILON) {
        cout << "Not skew symmetric matrix" << endl;
        cerr << m <<endl;
        return Vector3d::Zero();
    }
#endif
    Vector3d ret;
    ret << m(2,1), m(0,2), m(1,0);
    return ret;
}

inline Vector3d crossOperator(const MatrixXd & m) {
    Vector3d ret = Vector3d::Zero();
    ret[0] = m(1, 2) - m(2, 1);
    ret[1] = m(2, 0) - m(0, 2);
    ret[2] = m(0, 1) - m(1, 0);
    return ret;
}

/// @brief Transform the reference frame of the generalized velocity
/// (angular velocity + linear velocity) from frame 2 to frame 1.
/// @param[in] _T12 Transformation matrix from frame 1 to frame 2.
/// @param[in] _vel2 Generalized velocity represented in frame 2.
/// @return Generalized velocity represented in frame 1.
inline Vector6d Ad(Eigen::Matrix4d& _T12, const Vector6d& _vel2) {
    // _T12 = | R p |, vel1 = | w1 |, _vel2 = | w2 |
    //        | 0 1 |         | v1 |          | v2 |
    //
    // vel1 = Ad(_T12, _vel2)
    //      = Ad(_T12) * _vel2
    //      = | R    0 | * | w2 |
    //        | [p]R R | * | v2 |
    //      = | Rw2         |
    //        | [p]Rw2 + Rv2 |
    // w1 = R * w2
    // v1 = [p]R * w2 + R * v2

    Vector6d vel1;

    vel1(0) = _T12(0,0) * _vel2(0) + _T12(0,1) * _vel2(1) + _T12(0,2) * _vel2(2);
    vel1(1) = _T12(1,0) * _vel2(0) + _T12(1,1) * _vel2(1) + _T12(1,2) * _vel2(2);
    vel1(2) = _T12(2,0) * _vel2(0) + _T12(2,1) * _vel2(1) + _T12(2,2) * _vel2(2);
    vel1(3) = _T12(1,3) * vel1(2) - _T12(2,3) * vel1(1)
              + _T12(0,0) * _vel2(3)
              + _T12(0,1) * _vel2(4)
              + _T12(0,2) * _vel2(5);
    vel1(4) = _T12(2,3) * vel1(0) - _T12(0,3) * vel1(2)
              + _T12(1,0) * _vel2(3)
              + _T12(1,1) * _vel2(4)
              + _T12(1,2) * _vel2(5);
    vel1(5) = _T12(0,3) * vel1(1) - _T12(1,3) * vel1(0)
              + _T12(2,0) * _vel2(3)
              + _T12(2,1) * _vel2(4)
              + _T12(2,2) * _vel2(5);

    return vel1;
}

/// @brief Transform the reference frame of the generalized velocity
/// (angular velocity + linear velocity) from frame 1 to frame 2.
/// @param[in] _T12 Transformation matrix from frame 1 to frame 2.
/// @param[in] _vel1 Generalized velocity represented in frame 1.
/// @return Generalized velocity represented in frame 2.
inline Vector6d InvAd(Eigen::Matrix4d& _T12, const Vector6d& _vel1) {
	Vector6d vel2;

	double _tmp[3] = { _vel1(3) + _vel1(1) * _T12(2,3) - _vel1(2) * _T12(1,3),
					   _vel1(4) + _vel1(2) * _T12(0,3) - _vel1(0) * _T12(2,3),
					   _vel1(5) + _vel1(0) * _T12(1,3) - _vel1(1) * _T12(0,3) };
	vel2(0) = _T12(0,0) * _vel1(0) + _T12(1,0) * _vel1(1) + _T12(2,0) * _vel1(2);
	vel2(1) = _T12(0,1) * _vel1(0) + _T12(1,1) * _vel1(1) + _T12(2,1) * _vel1(2);
	vel2(2) = _T12(0,2) * _vel1(0) + _T12(1,2) * _vel1(1) + _T12(2,2) * _vel1(2);
	vel2(3) = _T12(0,0) * _tmp[0] + _T12(1,0) * _tmp[1] + _T12(2,0) * _tmp[2];
	vel2(4) = _T12(0,1) * _tmp[0] + _T12(1,1) * _tmp[1] + _T12(2,1) * _tmp[2];
	vel2(5) = _T12(0,2) * _tmp[0] + _T12(1,2) * _tmp[1] + _T12(2,2) * _tmp[2];

	return vel2;
}

} // namespace utils

#endif // #ifndef UTILS_UTILSMATH_H


