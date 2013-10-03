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

#ifndef DART_MATH_HELPERS_H
#define DART_MATH_HELPERS_H

// Standard Libraries
#include <vector>
#include <cmath>
#include <ctime>
#include <climits>
#include <cassert>
#include <iostream>

// External Libraries
#include <Eigen/Dense>
// Local Headers
#include "math/MathTypes.h"

namespace dart {
namespace math {

/// @brief a cross b = (CR*a) dot b
/// const Matd CR(2,2,0.0,-1.0,1.0,0.0);
const Eigen::Matrix2d CR( (Eigen::Matrix2d() << 0.0, -1.0, 1.0, 0.0).finished() );

inline int delta(int i, int j)
{
    if (i == j)
        return 1;
    return 0;
}

inline int sgn(double a)
{
    if (a < 0)
        return -1;
    else if (a == 0)
        return 0;
    else
        return 1;
}

inline double sqr(double x)
{
    return x*x;
}

inline double Tsinc(double theta)
{
    return 0.5-sqrt(theta)/48;
}

inline bool isZero(double theta)
{
    return (fabs(theta) < DART_EPSILON);
}

inline double asinh(double X)
{
    return log(X + sqrt(X * X + 1));
}

inline double acosh(double X)
{
    return log(X + sqrt(X * X - 1));
}

inline double atanh(double X)
{
    return log((1 + X)/(1 - X))/ 2;
}

inline double asech(double X)
{
    return log((sqrt(-X * X + 1) + 1) / X);
}

inline double acosech(double X)
{
    return log((sgn(X) * sqrt(X * X + 1) +1) / X);
}

inline double acotanh(double X)
{
    return log((X + 1) / (X - 1)) / 2;
}

inline double round(double x)
{
    return floor(x+0.5);
}

inline double round2(double x)
{
    int gintx = (int)floor(x);
    if (x - gintx < 0.5)
        return (double)gintx;
    else
        return (double)(gintx + 1);
}

inline bool isEqual(double x, double y)
{
    return (fabs(x - y) < DART_EPSILON);
}

// check if it is an integer
inline bool isInt(double x)
{
    if(isEqual(round(x), x)) return true;
    else return false;
}

inline double random( double min, double max )
{
    return min + ((double)rand()/(RAND_MAX + 1.0)) * (max-min);
}

inline unsigned seedRand()
{
    time_t now = time(0);
    unsigned char *p = (unsigned char *)&now;
    unsigned seed = 0;
    size_t i;

    for(i=0; i<sizeof(now); i++)
        seed = seed * (UCHAR_MAX + 2U ) + p[i];

    srand( seed );
    return seed;
}

inline Eigen::Vector3d crossOperator(const Eigen::MatrixXd & m)
{
    Eigen::Vector3d ret = Eigen::Vector3d::Zero();
    ret[0] = m(1, 2) - m(2, 1);
    ret[1] = m(2, 0) - m(0, 2);
    ret[2] = m(0, 1) - m(1, 0);
    return ret;
}

} // namespace utils
} // namespace dart

#endif // #ifndef DART_MATH_UTILSMATH_H


