/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <iostream>
#include <gtest/gtest.h>
#include "TestHelpers.hpp"

#include "dart/common/Timer.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/simulation/World.hpp"

using namespace dart;
using namespace common;
using namespace math;
using namespace dynamics;
using namespace simulation;

#define MATH_TOL 0.000001
#define MATH_EPS 0.000001

class EigenSE3
{
public:
    explicit EigenSE3(const Eigen::Matrix4d& T)
        : mT(T)
    {}

    /// \brief multiplication operator
    inline EigenSE3 operator*(const EigenSE3& T) const
    {
        return EigenSE3(mT * T.mT);
    }

    /// \brief multiplication operator
    inline const EigenSE3& operator*=(const EigenSE3& T)
    {
        mT *= T.mT;
        return *this;
    }

protected:
private:
    Eigen::Matrix4d mT;
};

template<typename T>
EIGEN_DONT_INLINE
void prod(const T& a, const T& b, T& c) { c = a*b; }

template<typename T>
EIGEN_DONT_INLINE
void inv(const T& a, T& b) { b = a.inverse(); }

EIGEN_DONT_INLINE
Isometry3d Inv2(const Isometry3d& I)
{
    Eigen::Isometry3d ret;

    ret(0,0) = I(0,0);
    ret(1,0) = I(0,1);
    ret(2,0) = I(0,2);

    ret(0,1) = I(1,0);
    ret(1,1) = I(1,1);
    ret(2,1) = I(1,2);

    ret(0,2) = I(2,0);
    ret(1,2) = I(2,1);
    ret(2,2) = I(2,2);

    ret(0,3) = -I(0,0) * I(0,3) - I(1,0) * I(1,3) - I(2,0) * I(2,3);
    ret(1,3) = -I(0,1) * I(0,3) - I(1,1) * I(1,3) - I(2,1) * I(2,3);
    ret(2,3) = -I(0,2) * I(0,3) - I(1,2) * I(1,3) - I(2,2) * I(2,3);

    return ret;
}

template<typename T>
EIGEN_DONT_INLINE
void inv_se3(const T& a, T& b) { b = Inv2(a); }

EIGEN_DONT_INLINE
void concatenate(const Affine3d& A1, const Affine3d& A2, Affine3d& res) {
   res(0,0) = A1(0,0) * A2(0,0) + A1(0,1) * A2(1,0) + A1(0,2) * A2(2,0);
   res(1,0) = A1(1,0) * A2(0,0) + A1(1,1) * A2(1,0) + A1(1,2) * A2(2,0);
   res(2,0) = A1(2,0) * A2(0,0) + A1(2,1) * A2(1,0) + A1(2,2) * A2(2,0);

   res(0,1) = A1(0,0) * A2(0,1) + A1(0,1) * A2(1,1) + A1(0,2) * A2(2,1);
   res(1,1) = A1(1,0) * A2(0,1) + A1(1,1) * A2(1,1) + A1(1,2) * A2(2,1);
   res(2,1) = A1(2,0) * A2(0,1) + A1(2,1) * A2(1,1) + A1(2,2) * A2(2,1);

   res(0,2) = A1(0,0) * A2(0,2) + A1(0,1) * A2(1,2) + A1(0,2) * A2(2,2);
   res(1,2) = A1(1,0) * A2(0,2) + A1(1,1) * A2(1,2) + A1(1,2) * A2(2,2);
   res(2,2) = A1(2,0) * A2(0,2) + A1(2,1) * A2(1,2) + A1(2,2) * A2(2,2);

   res(0,3) = A1(0,0) * A2(0,3) + A1(0,1) * A2(1,3) + A1(0,2) * A2(2,3) + A1(0,3);
   res(1,3) = A1(1,0) * A2(0,3) + A1(1,1) * A2(1,3) + A1(1,2) * A2(2,3) + A1(1,3);
   res(2,3) = A1(2,0) * A2(0,3) + A1(2,1) * A2(1,3) + A1(2,2) * A2(2,3) + A1(2,3);

   res(3,0) = 0.0;
   res(3,1) = 0.0;
   res(3,2) = 0.0;
   res(3,3) = 1.0;
}

EIGEN_DONT_INLINE
void concatenate2(const Affine3d& A1, const Affine3d& A2, Affine3d& res) {
   res(0,0) = A1(0,0) * A2(0,0) + A1(0,1) * A2(1,0) + A1(0,2) * A2(2,0);
   res(1,0) = A1(1,0) * A2(0,0) + A1(1,1) * A2(1,0) + A1(1,2) * A2(2,0);
   res(2,0) = A1(2,0) * A2(0,0) + A1(2,1) * A2(1,0) + A1(2,2) * A2(2,0);

   res(0,1) = A1(0,0) * A2(0,1) + A1(0,1) * A2(1,1) + A1(0,2) * A2(2,1);
   res(1,1) = A1(1,0) * A2(0,1) + A1(1,1) * A2(1,1) + A1(1,2) * A2(2,1);
   res(2,1) = A1(2,0) * A2(0,1) + A1(2,1) * A2(1,1) + A1(2,2) * A2(2,1);

   res(0,2) = A1(0,0) * A2(0,2) + A1(0,1) * A2(1,2) + A1(0,2) * A2(2,2);
   res(1,2) = A1(1,0) * A2(0,2) + A1(1,1) * A2(1,2) + A1(1,2) * A2(2,2);
   res(2,2) = A1(2,0) * A2(0,2) + A1(2,1) * A2(1,2) + A1(2,2) * A2(2,2);

   res(0,3) = A1(0,0) * A2(0,3) + A1(0,1) * A2(1,3) + A1(0,2) * A2(2,3) + A1(0,3);
   res(1,3) = A1(1,0) * A2(0,3) + A1(1,1) * A2(1,3) + A1(1,2) * A2(2,3) + A1(1,3);
   res(2,3) = A1(2,0) * A2(0,3) + A1(2,1) * A2(1,3) + A1(2,2) * A2(2,3) + A1(2,3);

   res(3,0) = 0.0;
   res(3,1) = 0.0;
   res(3,2) = 0.0;
   res(3,3) = 1.0;
}

EIGEN_DONT_INLINE
Matrix<double,6,1> Ad_Affine3d_1(const Affine3d& T, const Matrix<double,6,1>& s)
{
    //--------------------------------------------------------------------------
    // w' = R * w
    // v' = r x R * w + R * v
    //--------------------------------------------------------------------------
    double Rw[3] = { T.data()[0] * s.data()[0] + T.data()[4] * s.data()[1] + T.data()[ 8] * s.data()[2],
                     T.data()[1] * s.data()[0] + T.data()[5] * s.data()[1] + T.data()[ 9] * s.data()[2],
                     T.data()[2] * s.data()[0] + T.data()[6] * s.data()[1] + T.data()[10] * s.data()[2] };

    Matrix<double,6,1> ret;

    ret << Rw[0], Rw[1], Rw[2],
            T.data()[13] * Rw[2] - T.data()[14] * Rw[1] + T.data()[0] * s.data()[3] + T.data()[4] * s.data()[4] + T.data()[ 8] * s.data()[5],
            T.data()[14] * Rw[0] - T.data()[12] * Rw[2] + T.data()[1] * s.data()[3] + T.data()[5] * s.data()[4] + T.data()[ 9] * s.data()[5],
            T.data()[12] * Rw[1] - T.data()[13] * Rw[0] + T.data()[2] * s.data()[3] + T.data()[6] * s.data()[4] + T.data()[10] * s.data()[5];

    return ret;
}

EIGEN_DONT_INLINE
Matrix<double,6,1> Ad_Affine3d_2(const Affine3d& T, const Matrix<double,6,1>& s)
{
    //--------------------------------------------------------------------------
    // w' = R * w
    // v' = r x R * w + R * v
    //--------------------------------------------------------------------------
    double Rw[3] = { T(0,0) * s[0] + T(0,1) * s[1] + T(0,2) * s[2],
                     T(1,0) * s[0] + T(1,1) * s[1] + T(1,2) * s[2],
                     T(2,0) * s[0] + T(2,1) * s[1] + T(2,2) * s[2] };

    Matrix<double,6,1> ret;

    ret << Rw[0], Rw[1], Rw[2],
            T(1,3) * Rw[2] - T(2,3) * Rw[1] + T(0,0) * s[3] + T(0,1) * s[4] + T(0,2) * s[5],
            T(2,3) * Rw[0] - T(0,3) * Rw[2] + T(1,0) * s[3] + T(1,1) * s[4] + T(1,2) * s[5],
            T(0,3) * Rw[1] - T(1,3) * Rw[0] + T(2,0) * s[3] + T(2,1) * s[4] + T(2,2) * s[5];

    return ret;
}

EIGEN_DONT_INLINE
Matrix<double,6,1> Ad_Affine3d_3(const Affine3d& T, const Matrix<double,6,1>& s)
{
    //--------------------------------------------------------------------------
    // w' = R * w
    // v' = r x R * w + R * v
    //--------------------------------------------------------------------------

    Matrix<double,6,1> ret;

    ret.topLeftCorner<3,1>() = T.linear() * s.topLeftCorner<3,1>();
    ret.bottomLeftCorner<3,1>() = T.translation().cross(T.linear() * s.topLeftCorner<3,1>()) + T.linear() * s.bottomLeftCorner<3,1>();

    return ret;
}

EIGEN_DONT_INLINE
Matrix<double,6,1> Ad_Isometry3d(const Isometry3d& T, const Matrix<double,6,1>& s)
{
    //--------------------------------------------------------------------------
    // w' = R * w
    // v' = r x R * w + R * v
    //--------------------------------------------------------------------------
    double Rw[3] = { T(0,0) * s[0] + T(0,1) * s[1] + T(0,2) * s[2],
                     T(1,0) * s[0] + T(1,1) * s[1] + T(1,2) * s[2],
                     T(2,0) * s[0] + T(2,1) * s[1] + T(2,2) * s[2] };

    Matrix<double,6,1> ret;

    ret << Rw[0], Rw[1], Rw[2],
            T(1,3) * Rw[2] - T(2,3) * Rw[1] + T(0,0) * s[3] + T(0,1) * s[4] + T(0,2) * s[5],
            T(2,3) * Rw[0] - T(0,3) * Rw[2] + T(1,0) * s[3] + T(1,1) * s[4] + T(1,2) * s[5],
            T(0,3) * Rw[1] - T(1,3) * Rw[0] + T(2,0) * s[3] + T(2,1) * s[4] + T(2,2) * s[5];

    return ret;
}

EIGEN_DONT_INLINE
Matrix<double,6,1> Ad_Matrix4d(const Matrix4d& T, const Matrix<double,6,1>& s)
{
    //--------------------------------------------------------------------------
    // w' = R * w
    // v' = r x R * w + R * v
    //--------------------------------------------------------------------------
    double Rw[3] = { T(0,0) * s[0] + T(0,1) * s[1] + T(0,2) * s[2],
                     T(1,0) * s[0] + T(1,1) * s[1] + T(1,2) * s[2],
                     T(2,0) * s[0] + T(2,1) * s[1] + T(2,2) * s[2] };

    Matrix<double,6,1> ret;

    ret << Rw[0], Rw[1], Rw[2],
            T(1,3) * Rw[2] - T(2,3) * Rw[1] + T(0,0) * s[3] + T(0,1) * s[4] + T(0,2) * s[5],
            T(2,3) * Rw[0] - T(0,3) * Rw[2] + T(1,0) * s[3] + T(1,1) * s[4] + T(1,2) * s[5],
            T(0,3) * Rw[1] - T(1,3) * Rw[0] + T(2,0) * s[3] + T(2,1) * s[4] + T(2,2) * s[5];

    return ret;
}

EIGEN_DONT_INLINE
Matrix<double,6,6> Transform_Affine3d(const Affine3d& T, const Matrix<double,6,6>& AI)
{
    // operation count: multiplication = 186, addition = 117, subtract = 21

    Matrix<double,6,6> ret;

    double d0 = AI(0,3) + T(2,3) * AI(3,4) - T(1,3) * AI(3,5);
    double d1 = AI(1,3) - T(2,3) * AI(3,3) + T(0,3) * AI(3,5);
    double d2 = AI(2,3) + T(1,3) * AI(3,3) - T(0,3) * AI(3,4);
    double d3 = AI(0,4) + T(2,3) * AI(4,4) - T(1,3) * AI(4,5);
    double d4 = AI(1,4) - T(2,3) * AI(3,4) + T(0,3) * AI(4,5);
    double d5 = AI(2,4) + T(1,3) * AI(3,4) - T(0,3) * AI(4,4);
    double d6 = AI(0,5) + T(2,3) * AI(4,5) - T(1,3) * AI(5,5);
    double d7 = AI(1,5) - T(2,3) * AI(3,5) + T(0,3) * AI(5,5);
    double d8 = AI(2,5) + T(1,3) * AI(3,5) - T(0,3) * AI(4,5);
    double e0 = AI(0,0) + T(2,3) * AI(0,4) - T(1,3) * AI(0,5) + d3 * T(2,3) - d6 * T(1,3);
    double e3 = AI(0,1) + T(2,3) * AI(1,4) - T(1,3) * AI(1,5) - d0 * T(2,3) + d6 * T(0,3);
    double e4 = AI(1,1) - T(2,3) * AI(1,3) + T(0,3) * AI(1,5) - d1 * T(2,3) + d7 * T(0,3);
    double e6 = AI(0,2) + T(2,3) * AI(2,4) - T(1,3) * AI(2,5) + d0 * T(1,3) - d3 * T(0,3);
    double e7 = AI(1,2) - T(2,3) * AI(2,3) + T(0,3) * AI(2,5) + d1 * T(1,3) - d4 * T(0,3);
    double e8 = AI(2,2) + T(1,3) * AI(2,3) - T(0,3) * AI(2,4) + d2 * T(1,3) - d5 * T(0,3);
    double f0 = T(0,0) * e0 + T(1,0) * e3 + T(2,0) * e6;
    double f1 = T(0,0) * e3 + T(1,0) * e4 + T(2,0) * e7;
    double f2 = T(0,0) * e6 + T(1,0) * e7 + T(2,0) * e8;
    double f3 = T(0,0) * d0 + T(1,0) * d1 + T(2,0) * d2;
    double f4 = T(0,0) * d3 + T(1,0) * d4 + T(2,0) * d5;
    double f5 = T(0,0) * d6 + T(1,0) * d7 + T(2,0) * d8;
    double f6 = T(0,1) * e0 + T(1,1) * e3 + T(2,1) * e6;
    double f7 = T(0,1) * e3 + T(1,1) * e4 + T(2,1) * e7;
    double f8 = T(0,1) * e6 + T(1,1) * e7 + T(2,1) * e8;
    double g0 = T(0,1) * d0 + T(1,1) * d1 + T(2,1) * d2;
    double g1 = T(0,1) * d3 + T(1,1) * d4 + T(2,1) * d5;
    double g2 = T(0,1) * d6 + T(1,1) * d7 + T(2,1) * d8;
    double g3 = T(0,2) * d0 + T(1,2) * d1 + T(2,2) * d2;
    double g4 = T(0,2) * d3 + T(1,2) * d4 + T(2,2) * d5;
    double g5 = T(0,2) * d6 + T(1,2) * d7 + T(2,2) * d8;
    double h0 = T(0,0) * AI(3,3) + T(1,0) * AI(3,4) + T(2,0) * AI(3,5);
    double h1 = T(0,0) * AI(3,4) + T(1,0) * AI(4,4) + T(2,0) * AI(4,5);
    double h2 = T(0,0) * AI(3,5) + T(1,0) * AI(4,5) + T(2,0) * AI(5,5);
    double h3 = T(0,1) * AI(3,3) + T(1,1) * AI(3,4) + T(2,1) * AI(3,5);
    double h4 = T(0,1) * AI(3,4) + T(1,1) * AI(4,4) + T(2,1) * AI(4,5);
    double h5 = T(0,1) * AI(3,5) + T(1,1) * AI(4,5) + T(2,1) * AI(5,5);

    ret(0,0) = f0 * T(0,0) + f1 * T(1,0) + f2 * T(2,0);
    ret(0,1) = f0 * T(0,1) + f1 * T(1,1) + f2 * T(2,1);
    ret(0,2) = f0 * T(0,2) + f1 * T(1,2) + f2 * T(2,2);
    ret(0,3) = f3 * T(0,0) + f4 * T(1,0) + f5 * T(2,0);
    ret(0,4) = f3 * T(0,1) + f4 * T(1,1) + f5 * T(2,1);
    ret(0,5) = f3 * T(0,2) + f4 * T(1,2) + f5 * T(2,2);
    ret(1,1) = f6 * T(0,1) + f7 * T(1,1) + f8 * T(2,1);
    ret(1,2) = f6 * T(0,2) + f7 * T(1,2) + f8 * T(2,2);
    ret(1,3) = g0 * T(0,0) + g1 * T(1,0) + g2 * T(2,0);
    ret(1,4) = g0 * T(0,1) + g1 * T(1,1) + g2 * T(2,1);
    ret(1,5) = g0 * T(0,2) + g1 * T(1,2) + g2 * T(2,2);
    ret(2,2) = (T(0,2) * e0 + T(1,2) * e3 + T(2,2) * e6) * T(0,2) + (T(0,2) * e3 + T(1,2) * e4 + T(2,2) * e7) * T(1,2) + (T(0,2) * e6 + T(1,2) * e7 + T(2,2) * e8) * T(2,2);
    ret(2,3) = g3 * T(0,0) + g4 * T(1,0) + g5 * T(2,0);
    ret(2,4) = g3 * T(0,1) + g4 * T(1,1) + g5 * T(2,1);
    ret(2,5) = g3 * T(0,2) + g4 * T(1,2) + g5 * T(2,2);
    ret(3,3) = h0 * T(0,0) + h1 * T(1,0) + h2 * T(2,0);
    ret(3,4) = h0 * T(0,1) + h1 * T(1,1) + h2 * T(2,1);
    ret(3,5) = h0 * T(0,2) + h1 * T(1,2) + h2 * T(2,2);
    ret(4,4) = h3 * T(0,1) + h4 * T(1,1) + h5 * T(2,1);
    ret(4,5) = h3 * T(0,2) + h4 * T(1,2) + h5 * T(2,2);
    ret(5,5) = (T(0,2) * AI(3,3) + T(1,2) * AI(3,4) + T(2,2) * AI(3,5)) * T(0,2) + (T(0,2) * AI(3,4) + T(1,2) * AI(4,4) + T(2,2) * AI(4,5)) * T(1,2) + (T(0,2) * AI(3,5) + T(1,2) * AI(4,5) + T(2,2) * AI(5,5)) * T(2,2);

    ret.triangularView<StrictlyLower>() = ret.transpose();

    return ret;
}

EIGEN_DONT_INLINE
Matrix<double,6,6> Transform_Isometry3d(const Isometry3d& T, const Matrix<double,6,6>& AI)
{
    // operation count: multiplication = 186, addition = 117, subtract = 21

    Matrix<double,6,6> ret;

    double d0 = AI(0,3) + T(2,3) * AI(3,4) - T(1,3) * AI(3,5);
    double d1 = AI(1,3) - T(2,3) * AI(3,3) + T(0,3) * AI(3,5);
    double d2 = AI(2,3) + T(1,3) * AI(3,3) - T(0,3) * AI(3,4);
    double d3 = AI(0,4) + T(2,3) * AI(4,4) - T(1,3) * AI(4,5);
    double d4 = AI(1,4) - T(2,3) * AI(3,4) + T(0,3) * AI(4,5);
    double d5 = AI(2,4) + T(1,3) * AI(3,4) - T(0,3) * AI(4,4);
    double d6 = AI(0,5) + T(2,3) * AI(4,5) - T(1,3) * AI(5,5);
    double d7 = AI(1,5) - T(2,3) * AI(3,5) + T(0,3) * AI(5,5);
    double d8 = AI(2,5) + T(1,3) * AI(3,5) - T(0,3) * AI(4,5);
    double e0 = AI(0,0) + T(2,3) * AI(0,4) - T(1,3) * AI(0,5) + d3 * T(2,3) - d6 * T(1,3);
    double e3 = AI(0,1) + T(2,3) * AI(1,4) - T(1,3) * AI(1,5) - d0 * T(2,3) + d6 * T(0,3);
    double e4 = AI(1,1) - T(2,3) * AI(1,3) + T(0,3) * AI(1,5) - d1 * T(2,3) + d7 * T(0,3);
    double e6 = AI(0,2) + T(2,3) * AI(2,4) - T(1,3) * AI(2,5) + d0 * T(1,3) - d3 * T(0,3);
    double e7 = AI(1,2) - T(2,3) * AI(2,3) + T(0,3) * AI(2,5) + d1 * T(1,3) - d4 * T(0,3);
    double e8 = AI(2,2) + T(1,3) * AI(2,3) - T(0,3) * AI(2,4) + d2 * T(1,3) - d5 * T(0,3);
    double f0 = T(0,0) * e0 + T(1,0) * e3 + T(2,0) * e6;
    double f1 = T(0,0) * e3 + T(1,0) * e4 + T(2,0) * e7;
    double f2 = T(0,0) * e6 + T(1,0) * e7 + T(2,0) * e8;
    double f3 = T(0,0) * d0 + T(1,0) * d1 + T(2,0) * d2;
    double f4 = T(0,0) * d3 + T(1,0) * d4 + T(2,0) * d5;
    double f5 = T(0,0) * d6 + T(1,0) * d7 + T(2,0) * d8;
    double f6 = T(0,1) * e0 + T(1,1) * e3 + T(2,1) * e6;
    double f7 = T(0,1) * e3 + T(1,1) * e4 + T(2,1) * e7;
    double f8 = T(0,1) * e6 + T(1,1) * e7 + T(2,1) * e8;
    double g0 = T(0,1) * d0 + T(1,1) * d1 + T(2,1) * d2;
    double g1 = T(0,1) * d3 + T(1,1) * d4 + T(2,1) * d5;
    double g2 = T(0,1) * d6 + T(1,1) * d7 + T(2,1) * d8;
    double g3 = T(0,2) * d0 + T(1,2) * d1 + T(2,2) * d2;
    double g4 = T(0,2) * d3 + T(1,2) * d4 + T(2,2) * d5;
    double g5 = T(0,2) * d6 + T(1,2) * d7 + T(2,2) * d8;
    double h0 = T(0,0) * AI(3,3) + T(1,0) * AI(3,4) + T(2,0) * AI(3,5);
    double h1 = T(0,0) * AI(3,4) + T(1,0) * AI(4,4) + T(2,0) * AI(4,5);
    double h2 = T(0,0) * AI(3,5) + T(1,0) * AI(4,5) + T(2,0) * AI(5,5);
    double h3 = T(0,1) * AI(3,3) + T(1,1) * AI(3,4) + T(2,1) * AI(3,5);
    double h4 = T(0,1) * AI(3,4) + T(1,1) * AI(4,4) + T(2,1) * AI(4,5);
    double h5 = T(0,1) * AI(3,5) + T(1,1) * AI(4,5) + T(2,1) * AI(5,5);

    ret(0,0) = f0 * T(0,0) + f1 * T(1,0) + f2 * T(2,0);
    ret(0,1) = f0 * T(0,1) + f1 * T(1,1) + f2 * T(2,1);
    ret(0,2) = f0 * T(0,2) + f1 * T(1,2) + f2 * T(2,2);
    ret(0,3) = f3 * T(0,0) + f4 * T(1,0) + f5 * T(2,0);
    ret(0,4) = f3 * T(0,1) + f4 * T(1,1) + f5 * T(2,1);
    ret(0,5) = f3 * T(0,2) + f4 * T(1,2) + f5 * T(2,2);
    ret(1,1) = f6 * T(0,1) + f7 * T(1,1) + f8 * T(2,1);
    ret(1,2) = f6 * T(0,2) + f7 * T(1,2) + f8 * T(2,2);
    ret(1,3) = g0 * T(0,0) + g1 * T(1,0) + g2 * T(2,0);
    ret(1,4) = g0 * T(0,1) + g1 * T(1,1) + g2 * T(2,1);
    ret(1,5) = g0 * T(0,2) + g1 * T(1,2) + g2 * T(2,2);
    ret(2,2) = (T(0,2) * e0 + T(1,2) * e3 + T(2,2) * e6) * T(0,2) + (T(0,2) * e3 + T(1,2) * e4 + T(2,2) * e7) * T(1,2) + (T(0,2) * e6 + T(1,2) * e7 + T(2,2) * e8) * T(2,2);
    ret(2,3) = g3 * T(0,0) + g4 * T(1,0) + g5 * T(2,0);
    ret(2,4) = g3 * T(0,1) + g4 * T(1,1) + g5 * T(2,1);
    ret(2,5) = g3 * T(0,2) + g4 * T(1,2) + g5 * T(2,2);
    ret(3,3) = h0 * T(0,0) + h1 * T(1,0) + h2 * T(2,0);
    ret(3,4) = h0 * T(0,1) + h1 * T(1,1) + h2 * T(2,1);
    ret(3,5) = h0 * T(0,2) + h1 * T(1,2) + h2 * T(2,2);
    ret(4,4) = h3 * T(0,1) + h4 * T(1,1) + h5 * T(2,1);
    ret(4,5) = h3 * T(0,2) + h4 * T(1,2) + h5 * T(2,2);
    ret(5,5) = (T(0,2) * AI(3,3) + T(1,2) * AI(3,4) + T(2,2) * AI(3,5)) * T(0,2) + (T(0,2) * AI(3,4) + T(1,2) * AI(4,4) + T(2,2) * AI(4,5)) * T(1,2) + (T(0,2) * AI(3,5) + T(1,2) * AI(4,5) + T(2,2) * AI(5,5)) * T(2,2);

    ret.triangularView<StrictlyLower>() = ret.transpose();

    return ret;
}

EIGEN_DONT_INLINE
Matrix<double,6,6> Transform_Isometry3d_2(const Isometry3d& T, const Matrix<double,6,6>& AI)
{
    // operation count: multiplication = 186, addition = 117, subtract = 21

    Matrix<double,6,6> ret;

    double d0 = AI(0,3) + T(2,3) * AI(3,4) - T(1,3) * AI(3,5);
    double d1 = AI(1,3) - T(2,3) * AI(3,3) + T(0,3) * AI(3,5);
    double d2 = AI(2,3) + T(1,3) * AI(3,3) - T(0,3) * AI(3,4);
    double d3 = AI(0,4) + T(2,3) * AI(4,4) - T(1,3) * AI(4,5);
    double d4 = AI(1,4) - T(2,3) * AI(3,4) + T(0,3) * AI(4,5);
    double d5 = AI(2,4) + T(1,3) * AI(3,4) - T(0,3) * AI(4,4);
    double d6 = AI(0,5) + T(2,3) * AI(4,5) - T(1,3) * AI(5,5);
    double d7 = AI(1,5) - T(2,3) * AI(3,5) + T(0,3) * AI(5,5);
    double d8 = AI(2,5) + T(1,3) * AI(3,5) - T(0,3) * AI(4,5);
    double e0 = AI(0,0) + T(2,3) * AI(0,4) - T(1,3) * AI(0,5) + d3 * T(2,3) - d6 * T(1,3);
    double e3 = AI(0,1) + T(2,3) * AI(1,4) - T(1,3) * AI(1,5) - d0 * T(2,3) + d6 * T(0,3);
    double e4 = AI(1,1) - T(2,3) * AI(1,3) + T(0,3) * AI(1,5) - d1 * T(2,3) + d7 * T(0,3);
    double e6 = AI(0,2) + T(2,3) * AI(2,4) - T(1,3) * AI(2,5) + d0 * T(1,3) - d3 * T(0,3);
    double e7 = AI(1,2) - T(2,3) * AI(2,3) + T(0,3) * AI(2,5) + d1 * T(1,3) - d4 * T(0,3);
    double e8 = AI(2,2) + T(1,3) * AI(2,3) - T(0,3) * AI(2,4) + d2 * T(1,3) - d5 * T(0,3);
    double f0 = T(0,0) * e0 + T(1,0) * e3 + T(2,0) * e6;
    double f1 = T(0,0) * e3 + T(1,0) * e4 + T(2,0) * e7;
    double f2 = T(0,0) * e6 + T(1,0) * e7 + T(2,0) * e8;
    double f3 = T(0,0) * d0 + T(1,0) * d1 + T(2,0) * d2;
    double f4 = T(0,0) * d3 + T(1,0) * d4 + T(2,0) * d5;
    double f5 = T(0,0) * d6 + T(1,0) * d7 + T(2,0) * d8;
    double f6 = T(0,1) * e0 + T(1,1) * e3 + T(2,1) * e6;
    double f7 = T(0,1) * e3 + T(1,1) * e4 + T(2,1) * e7;
    double f8 = T(0,1) * e6 + T(1,1) * e7 + T(2,1) * e8;
    double g0 = T(0,1) * d0 + T(1,1) * d1 + T(2,1) * d2;
    double g1 = T(0,1) * d3 + T(1,1) * d4 + T(2,1) * d5;
    double g2 = T(0,1) * d6 + T(1,1) * d7 + T(2,1) * d8;
    double g3 = T(0,2) * d0 + T(1,2) * d1 + T(2,2) * d2;
    double g4 = T(0,2) * d3 + T(1,2) * d4 + T(2,2) * d5;
    double g5 = T(0,2) * d6 + T(1,2) * d7 + T(2,2) * d8;
    double h0 = T(0,0) * AI(3,3) + T(1,0) * AI(3,4) + T(2,0) * AI(3,5);
    double h1 = T(0,0) * AI(3,4) + T(1,0) * AI(4,4) + T(2,0) * AI(4,5);
    double h2 = T(0,0) * AI(3,5) + T(1,0) * AI(4,5) + T(2,0) * AI(5,5);
    double h3 = T(0,1) * AI(3,3) + T(1,1) * AI(3,4) + T(2,1) * AI(3,5);
    double h4 = T(0,1) * AI(3,4) + T(1,1) * AI(4,4) + T(2,1) * AI(4,5);
    double h5 = T(0,1) * AI(3,5) + T(1,1) * AI(4,5) + T(2,1) * AI(5,5);

    ret(0,0) = f0 * T(0,0) + f1 * T(1,0) + f2 * T(2,0);
    ret(0,1) = f0 * T(0,1) + f1 * T(1,1) + f2 * T(2,1);
    ret(0,2) = f0 * T(0,2) + f1 * T(1,2) + f2 * T(2,2);
    ret(0,3) = f3 * T(0,0) + f4 * T(1,0) + f5 * T(2,0);
    ret(0,4) = f3 * T(0,1) + f4 * T(1,1) + f5 * T(2,1);
    ret(0,5) = f3 * T(0,2) + f4 * T(1,2) + f5 * T(2,2);
    ret(1,0) = ret(0,1);
    ret(1,1) = f6 * T(0,1) + f7 * T(1,1) + f8 * T(2,1);
    ret(1,2) = f6 * T(0,2) + f7 * T(1,2) + f8 * T(2,2);
    ret(1,3) = g0 * T(0,0) + g1 * T(1,0) + g2 * T(2,0);
    ret(1,4) = g0 * T(0,1) + g1 * T(1,1) + g2 * T(2,1);
    ret(1,5) = g0 * T(0,2) + g1 * T(1,2) + g2 * T(2,2);
    ret(2,0) = ret(0,2);
    ret(2,1) = ret(1,2);
    ret(2,2) = (T(0,2) * e0 + T(1,2) * e3 + T(2,2) * e6) * T(0,2) + (T(0,2) * e3 + T(1,2) * e4 + T(2,2) * e7) * T(1,2) + (T(0,2) * e6 + T(1,2) * e7 + T(2,2) * e8) * T(2,2);
    ret(2,3) = g3 * T(0,0) + g4 * T(1,0) + g5 * T(2,0);
    ret(2,4) = g3 * T(0,1) + g4 * T(1,1) + g5 * T(2,1);
    ret(2,5) = g3 * T(0,2) + g4 * T(1,2) + g5 * T(2,2);
    ret(3,0) = ret(0,3);
    ret(3,1) = ret(1,3);
    ret(3,2) = ret(2,3);
    ret(3,3) = h0 * T(0,0) + h1 * T(1,0) + h2 * T(2,0);
    ret(3,4) = h0 * T(0,1) + h1 * T(1,1) + h2 * T(2,1);
    ret(3,5) = h0 * T(0,2) + h1 * T(1,2) + h2 * T(2,2);
    ret(4,0) = ret(0,4);
    ret(4,1) = ret(1,4);
    ret(4,2) = ret(2,4);
    ret(4,3) = ret(3,4);
    ret(4,4) = h3 * T(0,1) + h4 * T(1,1) + h5 * T(2,1);
    ret(4,5) = h3 * T(0,2) + h4 * T(1,2) + h5 * T(2,2);
    ret(5,0) = ret(0,5);
    ret(5,1) = ret(1,5);
    ret(5,2) = ret(2,5);
    ret(5,3) = ret(3,5);
    ret(5,4) = ret(4,5);
    ret(5,5) = (T(0,2) * AI(3,3) + T(1,2) * AI(3,4) + T(2,2) * AI(3,5)) * T(0,2) + (T(0,2) * AI(3,4) + T(1,2) * AI(4,4) + T(2,2) * AI(4,5)) * T(1,2) + (T(0,2) * AI(3,5) + T(1,2) * AI(4,5) + T(2,2) * AI(5,5)) * T(2,2);

    return ret;
}

EIGEN_DONT_INLINE
Matrix<double,6,6> Transform_Isometry3d_3(const Isometry3d& T, const Matrix<double,6,6>& AI)
{
    // operation count: multiplication = 186, addition = 117, subtract = 21

    Matrix<double,6,6> ret;

    double d0 = AI(0,3) + T(2,3) * AI(3,4) - T(1,3) * AI(3,5);
    double d1 = AI(1,3) - T(2,3) * AI(3,3) + T(0,3) * AI(3,5);
    double d2 = AI(2,3) + T(1,3) * AI(3,3) - T(0,3) * AI(3,4);
    double d3 = AI(0,4) + T(2,3) * AI(4,4) - T(1,3) * AI(4,5);
    double d4 = AI(1,4) - T(2,3) * AI(3,4) + T(0,3) * AI(4,5);
    double d5 = AI(2,4) + T(1,3) * AI(3,4) - T(0,3) * AI(4,4);
    double d6 = AI(0,5) + T(2,3) * AI(4,5) - T(1,3) * AI(5,5);
    double d7 = AI(1,5) - T(2,3) * AI(3,5) + T(0,3) * AI(5,5);
    double d8 = AI(2,5) + T(1,3) * AI(3,5) - T(0,3) * AI(4,5);
    double e0 = AI(0,0) + T(2,3) * AI(0,4) - T(1,3) * AI(0,5) + d3 * T(2,3) - d6 * T(1,3);
    double e3 = AI(0,1) + T(2,3) * AI(1,4) - T(1,3) * AI(1,5) - d0 * T(2,3) + d6 * T(0,3);
    double e4 = AI(1,1) - T(2,3) * AI(1,3) + T(0,3) * AI(1,5) - d1 * T(2,3) + d7 * T(0,3);
    double e6 = AI(0,2) + T(2,3) * AI(2,4) - T(1,3) * AI(2,5) + d0 * T(1,3) - d3 * T(0,3);
    double e7 = AI(1,2) - T(2,3) * AI(2,3) + T(0,3) * AI(2,5) + d1 * T(1,3) - d4 * T(0,3);
    double e8 = AI(2,2) + T(1,3) * AI(2,3) - T(0,3) * AI(2,4) + d2 * T(1,3) - d5 * T(0,3);
    double f0 = T(0,0) * e0 + T(1,0) * e3 + T(2,0) * e6;
    double f1 = T(0,0) * e3 + T(1,0) * e4 + T(2,0) * e7;
    double f2 = T(0,0) * e6 + T(1,0) * e7 + T(2,0) * e8;
    double f3 = T(0,0) * d0 + T(1,0) * d1 + T(2,0) * d2;
    double f4 = T(0,0) * d3 + T(1,0) * d4 + T(2,0) * d5;
    double f5 = T(0,0) * d6 + T(1,0) * d7 + T(2,0) * d8;
    double f6 = T(0,1) * e0 + T(1,1) * e3 + T(2,1) * e6;
    double f7 = T(0,1) * e3 + T(1,1) * e4 + T(2,1) * e7;
    double f8 = T(0,1) * e6 + T(1,1) * e7 + T(2,1) * e8;
    double g0 = T(0,1) * d0 + T(1,1) * d1 + T(2,1) * d2;
    double g1 = T(0,1) * d3 + T(1,1) * d4 + T(2,1) * d5;
    double g2 = T(0,1) * d6 + T(1,1) * d7 + T(2,1) * d8;
    double g3 = T(0,2) * d0 + T(1,2) * d1 + T(2,2) * d2;
    double g4 = T(0,2) * d3 + T(1,2) * d4 + T(2,2) * d5;
    double g5 = T(0,2) * d6 + T(1,2) * d7 + T(2,2) * d8;
    double h0 = T(0,0) * AI(3,3) + T(1,0) * AI(3,4) + T(2,0) * AI(3,5);
    double h1 = T(0,0) * AI(3,4) + T(1,0) * AI(4,4) + T(2,0) * AI(4,5);
    double h2 = T(0,0) * AI(3,5) + T(1,0) * AI(4,5) + T(2,0) * AI(5,5);
    double h3 = T(0,1) * AI(3,3) + T(1,1) * AI(3,4) + T(2,1) * AI(3,5);
    double h4 = T(0,1) * AI(3,4) + T(1,1) * AI(4,4) + T(2,1) * AI(4,5);
    double h5 = T(0,1) * AI(3,5) + T(1,1) * AI(4,5) + T(2,1) * AI(5,5);

    ret << f0 * T(0,0) + f1 * T(1,0) + f2 * T(2,0),
            f0 * T(0,1) + f1 * T(1,1) + f2 * T(2,1),
            f0 * T(0,2) + f1 * T(1,2) + f2 * T(2,2),
            f3 * T(0,0) + f4 * T(1,0) + f5 * T(2,0),
            f3 * T(0,1) + f4 * T(1,1) + f5 * T(2,1),
            f3 * T(0,2) + f4 * T(1,2) + f5 * T(2,2),
            ret(0,1),
            f6 * T(0,1) + f7 * T(1,1) + f8 * T(2,1),
            f6 * T(0,2) + f7 * T(1,2) + f8 * T(2,2),
            g0 * T(0,0) + g1 * T(1,0) + g2 * T(2,0),
            g0 * T(0,1) + g1 * T(1,1) + g2 * T(2,1),
            g0 * T(0,2) + g1 * T(1,2) + g2 * T(2,2),
            ret(0,2),
            ret(1,2),
            (T(0,2) * e0 + T(1,2) * e3 + T(2,2) * e6) * T(0,2) + (T(0,2) * e3 + T(1,2) * e4 + T(2,2) * e7) * T(1,2) + (T(0,2) * e6 + T(1,2) * e7 + T(2,2) * e8) * T(2,2),
            g3 * T(0,0) + g4 * T(1,0) + g5 * T(2,0),
            g3 * T(0,1) + g4 * T(1,1) + g5 * T(2,1),
            g3 * T(0,2) + g4 * T(1,2) + g5 * T(2,2),
            ret(0,3),
            ret(1,3),
            ret(2,3),
            h0 * T(0,0) + h1 * T(1,0) + h2 * T(2,0),
            h0 * T(0,1) + h1 * T(1,1) + h2 * T(2,1),
            h0 * T(0,2) + h1 * T(1,2) + h2 * T(2,2),
            ret(0,4),
            ret(1,4),
            ret(2,4),
            ret(3,4),
            h3 * T(0,1) + h4 * T(1,1) + h5 * T(2,1),
            h3 * T(0,2) + h4 * T(1,2) + h5 * T(2,2),
            ret(0,5),
            ret(1,5),
            ret(2,5),
            ret(3,5),
            ret(4,5),
            (T(0,2) * AI(3,3) + T(1,2) * AI(3,4) + T(2,2) * AI(3,5)) * T(0,2) + (T(0,2) * AI(3,4) + T(1,2) * AI(4,4) + T(2,2) * AI(4,5)) * T(1,2) + (T(0,2) * AI(3,5) + T(1,2) * AI(4,5) + T(2,2) * AI(5,5)) * T(2,2);

    return ret;
}

EIGEN_DONT_INLINE
Matrix<double,6,6> Transform_Matrix4d(const Matrix4d& T, const Matrix<double,6,6>& AI)
{
    // operation count: multiplication = 186, addition = 117, subtract = 21

    Matrix<double,6,6> ret;

    double d0 = AI(0,3) + T(2,3) * AI(3,4) - T(1,3) * AI(3,5);
    double d1 = AI(1,3) - T(2,3) * AI(3,3) + T(0,3) * AI(3,5);
    double d2 = AI(2,3) + T(1,3) * AI(3,3) - T(0,3) * AI(3,4);
    double d3 = AI(0,4) + T(2,3) * AI(4,4) - T(1,3) * AI(4,5);
    double d4 = AI(1,4) - T(2,3) * AI(3,4) + T(0,3) * AI(4,5);
    double d5 = AI(2,4) + T(1,3) * AI(3,4) - T(0,3) * AI(4,4);
    double d6 = AI(0,5) + T(2,3) * AI(4,5) - T(1,3) * AI(5,5);
    double d7 = AI(1,5) - T(2,3) * AI(3,5) + T(0,3) * AI(5,5);
    double d8 = AI(2,5) + T(1,3) * AI(3,5) - T(0,3) * AI(4,5);
    double e0 = AI(0,0) + T(2,3) * AI(0,4) - T(1,3) * AI(0,5) + d3 * T(2,3) - d6 * T(1,3);
    double e3 = AI(0,1) + T(2,3) * AI(1,4) - T(1,3) * AI(1,5) - d0 * T(2,3) + d6 * T(0,3);
    double e4 = AI(1,1) - T(2,3) * AI(1,3) + T(0,3) * AI(1,5) - d1 * T(2,3) + d7 * T(0,3);
    double e6 = AI(0,2) + T(2,3) * AI(2,4) - T(1,3) * AI(2,5) + d0 * T(1,3) - d3 * T(0,3);
    double e7 = AI(1,2) - T(2,3) * AI(2,3) + T(0,3) * AI(2,5) + d1 * T(1,3) - d4 * T(0,3);
    double e8 = AI(2,2) + T(1,3) * AI(2,3) - T(0,3) * AI(2,4) + d2 * T(1,3) - d5 * T(0,3);
    double f0 = T(0,0) * e0 + T(1,0) * e3 + T(2,0) * e6;
    double f1 = T(0,0) * e3 + T(1,0) * e4 + T(2,0) * e7;
    double f2 = T(0,0) * e6 + T(1,0) * e7 + T(2,0) * e8;
    double f3 = T(0,0) * d0 + T(1,0) * d1 + T(2,0) * d2;
    double f4 = T(0,0) * d3 + T(1,0) * d4 + T(2,0) * d5;
    double f5 = T(0,0) * d6 + T(1,0) * d7 + T(2,0) * d8;
    double f6 = T(0,1) * e0 + T(1,1) * e3 + T(2,1) * e6;
    double f7 = T(0,1) * e3 + T(1,1) * e4 + T(2,1) * e7;
    double f8 = T(0,1) * e6 + T(1,1) * e7 + T(2,1) * e8;
    double g0 = T(0,1) * d0 + T(1,1) * d1 + T(2,1) * d2;
    double g1 = T(0,1) * d3 + T(1,1) * d4 + T(2,1) * d5;
    double g2 = T(0,1) * d6 + T(1,1) * d7 + T(2,1) * d8;
    double g3 = T(0,2) * d0 + T(1,2) * d1 + T(2,2) * d2;
    double g4 = T(0,2) * d3 + T(1,2) * d4 + T(2,2) * d5;
    double g5 = T(0,2) * d6 + T(1,2) * d7 + T(2,2) * d8;
    double h0 = T(0,0) * AI(3,3) + T(1,0) * AI(3,4) + T(2,0) * AI(3,5);
    double h1 = T(0,0) * AI(3,4) + T(1,0) * AI(4,4) + T(2,0) * AI(4,5);
    double h2 = T(0,0) * AI(3,5) + T(1,0) * AI(4,5) + T(2,0) * AI(5,5);
    double h3 = T(0,1) * AI(3,3) + T(1,1) * AI(3,4) + T(2,1) * AI(3,5);
    double h4 = T(0,1) * AI(3,4) + T(1,1) * AI(4,4) + T(2,1) * AI(4,5);
    double h5 = T(0,1) * AI(3,5) + T(1,1) * AI(4,5) + T(2,1) * AI(5,5);

    ret(0,0) = f0 * T(0,0) + f1 * T(1,0) + f2 * T(2,0);
    ret(0,1) = f0 * T(0,1) + f1 * T(1,1) + f2 * T(2,1);
    ret(0,2) = f0 * T(0,2) + f1 * T(1,2) + f2 * T(2,2);
    ret(0,3) = f3 * T(0,0) + f4 * T(1,0) + f5 * T(2,0);
    ret(0,4) = f3 * T(0,1) + f4 * T(1,1) + f5 * T(2,1);
    ret(0,5) = f3 * T(0,2) + f4 * T(1,2) + f5 * T(2,2);
    ret(1,1) = f6 * T(0,1) + f7 * T(1,1) + f8 * T(2,1);
    ret(1,2) = f6 * T(0,2) + f7 * T(1,2) + f8 * T(2,2);
    ret(1,3) = g0 * T(0,0) + g1 * T(1,0) + g2 * T(2,0);
    ret(1,4) = g0 * T(0,1) + g1 * T(1,1) + g2 * T(2,1);
    ret(1,5) = g0 * T(0,2) + g1 * T(1,2) + g2 * T(2,2);
    ret(2,2) = (T(0,2) * e0 + T(1,2) * e3 + T(2,2) * e6) * T(0,2) + (T(0,2) * e3 + T(1,2) * e4 + T(2,2) * e7) * T(1,2) + (T(0,2) * e6 + T(1,2) * e7 + T(2,2) * e8) * T(2,2);
    ret(2,3) = g3 * T(0,0) + g4 * T(1,0) + g5 * T(2,0);
    ret(2,4) = g3 * T(0,1) + g4 * T(1,1) + g5 * T(2,1);
    ret(2,5) = g3 * T(0,2) + g4 * T(1,2) + g5 * T(2,2);
    ret(3,3) = h0 * T(0,0) + h1 * T(1,0) + h2 * T(2,0);
    ret(3,4) = h0 * T(0,1) + h1 * T(1,1) + h2 * T(2,1);
    ret(3,5) = h0 * T(0,2) + h1 * T(1,2) + h2 * T(2,2);
    ret(4,4) = h3 * T(0,1) + h4 * T(1,1) + h5 * T(2,1);
    ret(4,5) = h3 * T(0,2) + h4 * T(1,2) + h5 * T(2,2);
    ret(5,5) = (T(0,2) * AI(3,3) + T(1,2) * AI(3,4) + T(2,2) * AI(3,5)) * T(0,2) + (T(0,2) * AI(3,4) + T(1,2) * AI(4,4) + T(2,2) * AI(4,5)) * T(1,2) + (T(0,2) * AI(3,5) + T(1,2) * AI(4,5) + T(2,2) * AI(5,5)) * T(2,2);

    ret.triangularView<StrictlyLower>() = ret.transpose();

    return ret;
}

TEST(MATH, ARTICULATED_INERTIA_TRANSFORM)
{
    const int iterations = 100000;

    Affine3d A1 = Translation3d(0.1, 0.2, 0.3) * AngleAxisd(0.5, Vector3d(1.0 / sqrt(2.0), 1.0 / sqrt(2.0), 0.0));
    Matrix<double,6,6> a1 = Matrix<double,6,6>::Identity();
    Matrix<double,6,6> a2;

    Isometry3d I1;
    I1.matrix() = A1.matrix();
    Matrix<double,6,6> i1 = Matrix<double,6,6>::Identity();
    Matrix<double,6,6> i2;

    Matrix4d M1;
    M1 = A1.matrix();
    Matrix<double,6,6> m1 = Matrix<double,6,6>::Identity();
    Matrix<double,6,6> m2;

    clock_t start = clock();
    for (int i = 0; i < iterations; i++) {
        a2 = Transform_Affine3d(A1, a1);
    }
    std::cout << "Transform_Affine3d: " << (double)(clock() - start) / CLOCKS_PER_SEC << " s\n";
    //cout << "result: " << a2 << endl;

    start = clock();
    for (int i = 0; i < iterations; i++) {
        i2 = Transform_Isometry3d(I1, i1);
    }
    std::cout << "Transform_Isometry3d: " << (double)(clock() - start) / CLOCKS_PER_SEC << " s\n";
    //cout << "result: " << i2 << endl;

    start = clock();
    for (int i = 0; i < iterations; i++) {
        i2 = Transform_Isometry3d_2(I1, i1);
    }
    std::cout << "Transform_Isometry3d_2: " << (double)(clock() - start) / CLOCKS_PER_SEC << " s\n";
    //cout << "result: " << i2 << endl;

    start = clock();
    for (int i = 0; i < iterations; i++) {
        i2 = Transform_Isometry3d_3(I1, i1);
    }
    std::cout << "Transform_Isometry3d_3: " << (double)(clock() - start) / CLOCKS_PER_SEC << " s\n";
    //cout << "result: " << i2 << endl;

    start = clock();
    for (int i = 0; i < iterations; i++) {
        m2 = Transform_Matrix4d(M1, m1);
    }
    std::cout << "Transform_Matrix4d: " << (double)(clock() - start) / CLOCKS_PER_SEC << " s\n";
    //cout << "result: " << m2 << endl;
}

TEST(MATH, ADJOINT_MAPPING)
{
    const int iterations = 10000;

    Affine3d A1 = Translation3d(0.1, 0.2, 0.3) * AngleAxisd(0.5, Vector3d(1.0 / sqrt(2.0), 1.0 / sqrt(2.0), 0.0));
    Matrix<double,6,1> a1, a2;
    a1 << 1, 2, 3, 4, 5, 6;

    Affine3d A2 = Translation3d(0.1, 0.2, 0.3) * AngleAxisd(0.5, Vector3d(1.0 / sqrt(2.0), 1.0 / sqrt(2.0), 0.0));
    Matrix<double,6,1> a3, a4;
    a3 << 1, 2, 3, 4, 5, 6;

    // Affine3d A3 = Translation3d(0.1, 0.2, 0.3) * AngleAxisd(0.5, Vector3d(1.0 / sqrt(2.0), 1.0 / sqrt(2.0), 0.0));
    Matrix<double,6,1> a5, a6;
    a5 << 1, 2, 3, 4, 5, 6;

    Isometry3d I1;
    I1.matrix() = A1.matrix();
    Matrix<double,6,1> i1, i2;
    i1 << 1, 2, 3, 4, 5, 6;

    Matrix4d M1;
    M1 = A1.matrix();
    Matrix<double,6,1> m1, m2;
    m1 << 1, 2, 3, 4, 5, 6;

    clock_t start = clock();
    for (int i = 0; i < iterations; i++) {
        a2 = Ad_Affine3d_1(A1, a1);
    }
    std::cout << "Ad_Affine3d_1: " << (double)(clock() - start) / CLOCKS_PER_SEC << " s\n";
    //cout << "result: " << a2 << endl;

    start = clock();
    for (int i = 0; i < iterations; i++) {
        a4 = Ad_Affine3d_2(A2, a3);
    }
    std::cout << "Ad_Affine3d_2: " << (double)(clock() - start) / CLOCKS_PER_SEC << " s\n";
    //cout << "result: " << a4 << endl;

//    start = clock();
//    for (int i = 0; i < iterations; i++) {
//        a6 = Ad_Affine3d_3(A3, a5);
//    }
//    cout << "Ad_Affine3d_3: " << (double)(clock() - start) / CLOCKS_PER_SEC << " s\n";
//    //cout << "result: " << a6 << endl;

    start = clock();
    for (int i = 0; i < iterations; i++) {
        i2 = Ad_Isometry3d(I1, i1);
    }
    std::cout << "Ad_Isometry3d: " << (double)(clock() - start) / CLOCKS_PER_SEC << " s\n";
    //cout << "result: " << i2 << endl;

    start = clock();
    for (int i = 0; i < iterations; i++) {
        m2 = Ad_Matrix4d(M1, m1);
    }
    std::cout << "Ad_Matrix4d: " << (double)(clock() - start) / CLOCKS_PER_SEC << " s\n";
    //cout << "result: " << im << endl;
}


TEST(MATH, TRANSFORMATION)
{
    const int iterations = 10000;

    Affine3d A1 = Translation3d(0.1, 0.2, 0.3) * AngleAxisd(0.5, Vector3d(1.0 / sqrt(2.0), 1.0 / sqrt(2.0), 0.0));
    Affine3d A2 = A1, A3;

    Isometry3d I1;
    I1.matrix() = A1.matrix();
    Isometry3d I2 = I1, I3;

    Matrix4d M1 = A1.matrix();
    Matrix4d M2 = M1, M3;

    EigenSE3 ES1(A1.matrix());
    EigenSE3 ES2(A1.matrix()), ES3(A1.matrix());

    clock_t start = clock();
    for (int i = 0; i < iterations; i++) {
       prod(ES2, ES1, ES3);
    }
    std::cout << "EigenSE3: " << (double)(clock() - start) / CLOCKS_PER_SEC << " s\n";

    start = clock();
    for (int i = 0; i < iterations; i++) {
       prod( A2 , A1, A3);
    }
    std::cout << "Affine3d: " << (double)(clock() - start) / CLOCKS_PER_SEC << " s\n";

    start = clock();
    for (int i = 0; i < iterations; i++) {
       prod( I2 , I1, I3);
    }
    std::cout << "Isometry3d: " << (double)(clock() - start) / CLOCKS_PER_SEC << " s\n";

    start = clock();
    for (int i = 0; i < iterations; i++) {
       prod ( M2 , M1, M3 );
    }
    std::cout << "Matrix4d: " << (double)(clock() - start) / CLOCKS_PER_SEC << " s\n";

    A2 = A1;
    start = clock();
    for (int i = 0; i < iterations; i++) {
       concatenate(A2, A1, A3);
    }
    std::cout << "Hard-coded: " << (double)(clock() - start) / CLOCKS_PER_SEC << " s\n";
}

TEST(MATH, INVERSION)
{
    const int iterations = 10000;

    Affine3d A1 = Translation3d(0.1, 0.2, 0.3) * AngleAxisd(0.5, Vector3d(1.0 / sqrt(2.0), 1.0 / sqrt(2.0), 0.0));
    Affine3d A2 = A1, A3;

    Isometry3d I1;
    I1.matrix() = A1.matrix();
    Isometry3d I2 = I1, I3;

    Matrix4d M1 = A1.matrix();
    Matrix4d M2 = M1, M3;

    clock_t start = clock();
    for (int i = 0; i < iterations; i++) {
        inv(A1, A2);
    }
    std::cout << "Affine3d: " << (double)(clock() - start) / CLOCKS_PER_SEC << " s\n";

    start = clock();
    for (int i = 0; i < iterations; i++) {
        inv(I1, I2);
    }
    std::cout << "Isometry3d_1: " << (double)(clock() - start) / CLOCKS_PER_SEC << " s\n";

    start = clock();
    for (int i = 0; i < iterations; i++) {
        inv_se3(I1, I2);
    }
    std::cout << "Isometry3d_2: " << (double)(clock() - start) / CLOCKS_PER_SEC << " s\n";

    start = clock();
    for (int i = 0; i < iterations; i++) {
       inv(M1, M2);
    }
    std::cout << "Matrix4d: " << (double)(clock() - start) / CLOCKS_PER_SEC << " s\n";
}


/******************************************************************************/
//TEST(MATH, SE3_VS_EIGENMATRIX4D)
//{
//    int n = 10000;
//    double min = -100;
//    double max = 100;

//    SE3 T1 = Exp(se3(random(min,max), random(min,max), random(min,max),
//                     random(min,max), random(min,max), random(min,max)));
//    SE3 T2 = Exp(se3(random(min,max), random(min,max), random(min,max),
//                     random(min,max), random(min,max), random(min,max)));
//    SE3 T3 = Exp(se3(random(min,max), random(min,max), random(min,max),
//                     random(min,max), random(min,max), random(min,max)));
//    SE3 T4 = Exp(se3(random(min,max), random(min,max), random(min,max),
//                     random(min,max), random(min,max), random(min,max)));

//    Eigen::Matrix4d E1 = T1;
//    Eigen::Matrix4d E2 = T2;
//    Eigen::Matrix4d E3 = T3;
//    Eigen::Matrix4d E4 = T4;

//    Eigen::Affine3d A1(E1);
//    Eigen::Affine3d A2(E2);
//    Eigen::Affine3d A3(E3);
//    Eigen::Affine3d A4(E4);

//    Eigen::Isometry3d I1(E1);
//    Eigen::Isometry3d I2(E2);
//    Eigen::Isometry3d I3(E3);
//    Eigen::Isometry3d I4(E4);

//    EigenSE3 ESE3_1(E1);
//    EigenSE3 ESE3_2(E2);
//    EigenSE3 ESE3_3(E3);
//    EigenSE3 ESE3_4(E4);

//    {
//        Timer SE3Timer("SE3 timer");
//        SE3Timer.startTimer();
//        for (int i = 0; i < n; ++i)
//            for (int j = 0; j < n; ++j)
//                T3 = T3 * T2;
//        SE3Timer.stopTimer();
//    }
//    std::cout << T3 << std::endl;

//    {
//        Timer EigenTimer("Eigen::Matrix4d timer");
//        EigenTimer.startTimer();
//        for (int i = 0; i < n; ++i)
//            for (int j = 0; j < n; ++j)
//                E3 = E3 * E2;
//        EigenTimer.stopTimer();
//    }
//    std::cout << E3 << std::endl;

////    {
////        Timer EigenSE3Timer("EigenSE3 timer");
////        EigenSE3Timer.startTimer();
////        for (int i = 0; i < n; ++i)
////            for (int j = 0; j < n; ++j)
////                ESE3_3 *= ESE3_1 * ESE3_2;
////        EigenSE3Timer.stopTimer();
////    }
////    //std::cout << E3 << std::endl;

//    {
//        Timer AffineTimer("Eigen::Affine3d timer");
//        AffineTimer.startTimer();
//        for (int i = 0; i < n; ++i)
//            for (int j = 0; j < n; ++j)
//                A3 = A3 * A2;
//        AffineTimer.stopTimer();
//    }
//    std::cout << A3.matrix() << std::endl;

//    {
//        Timer IsometryTimer("Eigen::Isometry3d timer");
//        IsometryTimer.startTimer();
//        for (int i = 0; i < n; ++i)
//            for (int j = 0; j < n; ++j)
//                I3 = I3 * I2;
//        IsometryTimer.stopTimer();
//    }
//    std::cout << I3.matrix() << std::endl;

////    {
////        Timer SE3Timer("SE3 inverse timer");
////        SE3Timer.startTimer();
////        for (int i = 0; i < n; ++i)
////            for (int j = 0; j < n; ++j)
////                T4 *= Inv(T1) * Inv(T2);
////        SE3Timer.stopTimer();
////    }
////    std::cout << T4 << std::endl;

////    {
////        Timer EigenTimer("Eigen::Matrix4d inverse timer");
////        EigenTimer.startTimer();
////        for (int i = 0; i < n; ++i)
////            for (int j = 0; j < n; ++j)
////                E4 *= E1.inverse() * E2.inverse();
////        EigenTimer.stopTimer();
////    }
////    std::cout << E4 << std::endl;
//}

/******************************************************************************/
//TEST(MATH, SO3)
//{
//    // Exponential and Logarithm mapping
//    for (int i = 0; i < 100; ++i)
//    {
//        double min = -100;
//        double max = 100;

//        // Log(Exp(w)) = w
//        so3 w(random(min,max), random(min,max), random(min,max));
//        SO3 Expw = Exp(w);
//        so3 LogExpw = Log(Expw);
//        SO3 ExpLogExpw = Exp(LogExpw);
//        so3 LogExpLogExpw = Log(ExpLogExpw);

//        for (int j = 0; j < 3; ++j)
//        {
//            for (int k = 0; k < 3; ++k)
//            {
//                EXPECT_NEAR(Expw(j,k), ExpLogExpw(j,k), MATH_EPS);
//            }
//        }

//        for (int j = 0; j < 3; ++j)
//            EXPECT_NEAR(LogExpLogExpw[j], LogExpw[j], MATH_EPS);

//    }
//}

///******************************************************************************/
//TEST(MATH, SE3)
//{
//    // Exponential and Logarithm mapping
//    for (int i = 0; i < 100; ++i)
//    {
//        double min = -100;
//        double max = 100;

//        // Log(Exp(S)) = S
//        se3 S(random(min,max), random(min,max), random(min,max),
//              random(min,max), random(min,max), random(min,max));
//        SE3 ExpS = Exp(S);
//        se3 LogExpS = Log(ExpS);
//        SE3 ExpLogExpS = Exp(S);
//        se3 ExpLogLogExpS = Log(ExpS);

//        for (int j = 0; j < 4; ++j)
//        {
//            for (int k = 0; k < 4; ++k)
//                EXPECT_NEAR(ExpS(j,k), ExpLogExpS(j,k), MATH_EPS);
//        }

//        for (int j = 0; j < 6; ++j)
//            EXPECT_NEAR(LogExpS(j), ExpLogLogExpS(j), MATH_EPS);
//    }

////    // Exp(Log(T)) = T

////    // Ad(T,V) == T * V * invT
////    se3 V(random(-1,1), random(-1,1), random(-1,1),
////          random(-1,1), random(-1,1), random(-1,1));
////    SE3 T = Exp(V);

////    se3 AdTV = Ad(T,V);
////    Matrix4d AdTVmat_eig = AdTV;

////    Matrix4d Teig = T;
////    Matrix4d Veig = V;
////    Matrix4d invTeig = Inv(T);
////    Matrix4d TVinvT = Teig * Veig * invTeig;

////    //EXPECT_EQ(AdTVmat_eig, TVinvT);

////    EXPECT_EQ(Ad(T, Ad(Inv(T), V)), V);
//}

///// \brief
//Eigen::Matrix<double,6,6> Ad(const SE3& T)
//{
//    Eigen::Matrix<double,6,6> AdT = Eigen::Matrix<double,6,6>::Zero();

//    // R
//    AdT(0,0) =  T(0,0);   AdT(0,1) =  T(0,1);   AdT(0,2) =  T(0,2);
//    AdT(1,0) =  T(1,0);   AdT(1,1) =  T(1,1);   AdT(1,2) =  T(1,2);
//    AdT(2,0) =  T(2,0);   AdT(2,1) =  T(2,1);   AdT(2,2) =  T(2,2);

//    // R
//    AdT(3,3) =  T(0,0);   AdT(3,4) =  T(0,1);   AdT(3,5) =  T(0,2);
//    AdT(4,3) =  T(1,0);   AdT(4,4) =  T(1,1);   AdT(4,5) =  T(1,2);
//    AdT(5,3) =  T(2,0);   AdT(5,4) =  T(2,1);   AdT(5,5) =  T(2,2);

//    // [p]R
//    AdT(3,0) = T(1,3)*T(2,0) - T(2,3)*T(1,0);   AdT(3,1) = T(1,3)*T(2,1) - T(2,3)*T(1,1);   AdT(3,2) = T(1,3)*T(2,2) - T(2,3)*T(1,2);
//    AdT(4,0) = T(2,3)*T(0,0) - T(0,3)*T(2,0);   AdT(4,1) = T(2,3)*T(0,1) - T(0,3)*T(2,1);   AdT(4,2) = T(2,3)*T(0,2) - T(0,3)*T(2,2);
//    AdT(5,0) = T(0,3)*T(1,0) - T(1,3)*T(0,0);   AdT(5,1) = T(0,3)*T(1,1) - T(1,3)*T(0,1);   AdT(5,2) = T(0,3)*T(1,2) - T(1,3)*T(0,2);

//    return AdT;
//}

///// \brief
//Eigen::Matrix<double,6,6> dAd(const SE3& T)
//{
//    return Ad(T).transpose();
//}

///******************************************************************************/
//TEST(MATH, ADJOINT_MAPPING)
//{
//    double min = -100;
//    double max = 100;
//    math::se3 t(random(min,max), random(min,max), random(min,max),
//                random(min,max), random(min,max), random(min,max));
//    math::SE3 T = math::Exp(t);
//    math::se3 S(random(min,max), random(min,max), random(min,max),
//                random(min,max), random(min,max), random(min,max));

//    Eigen::VectorXd AdT_S = math::Ad(T, S).getEigenVector();
//    Eigen::VectorXd AdT_S2 = Ad(T) * S.getEigenVector();

//    Eigen::MatrixXd I = Ad(T) * Ad(math::Inv(T));
//    Eigen::MatrixXd SE3Identity = T * (math::Inv(T));

//    for (int i = 0; i < 6; i++)
//        EXPECT_NEAR(AdT_S(i), AdT_S2(i), MATH_TOL);
//}

///******************************************************************************/
//TEST(MATH, INERTIA)
//{
//    for (int k = 0; k < 100; ++k)
//    {
//        double min = -10;
//        double max = 10;
//        math::Vec3 r(random(min,max), random(min,max), random(min,max));
//        math::SE3 Tr(r);
//        math::Inertia I(random(0.1,max), random(0.1,max), random(0.1,max),
//                        random(0.1,max), random(0.1,max), random(0.1,max),
//                        random(min,max), random(min,max), random(min,max),
//                        random(0.1,max));
//        math::Inertia Ioffset = I;

//        I.setOffset(math::Vec3(0, 0, 0));
//        Ioffset.setOffset(r);

//        Eigen::MatrixXd G = I.toTensor();
//        Eigen::MatrixXd Goffset = Ioffset.toTensor();
//        Eigen::MatrixXd dAdinvTGAdinvT = dAd(math::Inv(Tr)) * G * Ad(math::Inv(Tr));

//        for (int i = 0; i < 6; i++)
//            for (int j = 0; j < 6; j++)
//                EXPECT_NEAR(Goffset(i,j), dAdinvTGAdinvT(i,j), MATH_TOL);
//    }
//}

/******************************************************************************/
TEST(MATH, ROTATION) {
  using namespace dart;
  using namespace math;

  // Create Initial ExpMap
  Eigen::Vector3d axis(2.0, 1.0, 1.0);
  axis.normalize();
  double angle = 1.2;
  EXPECT_DOUBLE_EQ(axis.norm(), 1.0);
  Eigen::Vector3d expmap = axis * angle;


  // Test conversion between expmap and quaternion
  Eigen::Quaterniond q = expToQuat(expmap);
  Eigen::Vector3d expmap2 = quatToExp(q);

  EXPECT_NEAR((expmap - expmap2).norm(), 0.0, MATH_EPS)
    << "Orig: " << expmap << " Reconstructed: " << expmap2;

  // Test conversion between matrix and euler
  Eigen::Matrix3d m = q.toRotationMatrix();
  Eigen::Vector3d e = matrixToEulerXYZ(m);
  Eigen::Matrix3d m2 = eulerXYZToMatrix(e);

  EXPECT_NEAR((m - m2).norm(), 0.0, MATH_EPS)
    << "Orig: " << m << " Reconstructed: " << m2;
}

/******************************************************************************/
TEST(MATH, UTILS) {
  // Test CR Matrix
  EXPECT_DOUBLE_EQ(dart::math::CR(0, 1), -1.0);

  // Test randomize function
  double x = dart::math::Random::uniform(0.0, 2.0);
  EXPECT_LT(0.0, x);
  EXPECT_LT(x, 2.0);

  // Test transform
  Eigen::Isometry3d M = Eigen::Isometry3d::Identity();
  M.translation() = Eigen::Vector3d(3.0, 2.0, 1.0);
  Eigen::Vector3d pt(1.0, 0.5, 1.0);
  Eigen::Vector3d result = M * pt;
  Eigen::Vector3d expected(4.0, 2.5, 2.0);
  EXPECT_NEAR( (result - expected).norm(), 0.0, MATH_EPS)
    << "result = " << result << " expected = " << expected;
}

//==============================================================================
Jacobian AdTJac1(const Eigen::Isometry3d& _T, const Jacobian& _J)
{
  Jacobian res = Jacobian::Zero(6, _J.cols());

  for (int i = 0; i < _J.cols(); ++i)
    res.col(i) = AdT(_T, _J.col(i));
  return res;
}

//==============================================================================
template<typename Derived>
typename Derived::PlainObject AdTJac2(const Eigen::Isometry3d& _T,
                                      const Eigen::MatrixBase<Derived>& _J)
{
//  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT(Derived::RowsAtCompileTime == 6,
                      THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

  typename Derived::PlainObject ret(_J.rows(), _J.cols());

  for (int i = 0; i < _J.cols(); ++i)
    ret.col(i) = AdT(_T, _J.col(i));

  return ret;
}

//==============================================================================
template<typename Derived>
typename Derived::PlainObject AdTJac3(const Eigen::Isometry3d& _T,
                                      const Eigen::MatrixBase<Derived>& _J)
{
//  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT(Derived::RowsAtCompileTime == 6,
                      THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

  typename Derived::PlainObject ret(_J.rows(), _J.cols());

  ret.template topRows<3>().noalias() = _T.linear() * _J.template topRows<3>();
  ret.template bottomRows<3>().noalias()
      = -ret.template topRows<3>().colwise().cross(_T.translation())
        + _T.linear() * _J.template bottomRows<3>();

  return ret;
}

//==============================================================================
TEST(MATH, PerformanceComparisonOfAdTJac)
{
#ifndef NDEBUG
  int testCount = 1e+2;
#else
  int testCount = 1e+6;
#endif
  int m = 3;

  Vector6d t = Vector6d::Random();
  Isometry3d T = expMap(t);
  Jacobian dynamicJ = Jacobian::Random(6, m);
  Matrix<double, 6, 3> fixedJ = dynamicJ;//Matrix<double, 6, 3>::Random();

  // Test1: verify the results
  for (int i = 0; i < testCount; ++i)
  {
    Jacobian resJ1 = AdTJac1(T, dynamicJ);
    Jacobian resJ2 = AdTJac2(T, dynamicJ);
    Jacobian resJ3 = AdTJac3(T, fixedJ);

    EXPECT_TRUE(equals(resJ1, resJ2));
    EXPECT_TRUE(equals(resJ2, resJ3));

    Jacobian resJ4 = AdInvTJac(T, dynamicJ);
    Jacobian resJ5 = AdInvTJacFixed(T, fixedJ);

    EXPECT_TRUE(equals(resJ4, resJ5));
  }

  // Test2: performance
  Timer t1dyn("AdTJac1 - dynamic");
  t1dyn.start();
  for (int i = 0; i < testCount; ++i)
  {
    Jacobian resJ1 = AdTJac1(T, dynamicJ);
  }
  t1dyn.stop();

  Timer t1fix("AdTJac1 - fixed");
  t1fix.start();
  for (int i = 0; i < testCount; ++i)
  {
    Jacobian resJ1 = AdTJac1(T, fixedJ);
  }
  t1fix.stop();

  Timer t2dyn("AdTJac2 - dynamic");
  t2dyn.start();
  for (int i = 0; i < testCount; ++i)
  {
    Jacobian resJ2 = AdTJac2(T, dynamicJ);
  }
  t2dyn.stop();

  Timer t2fix("AdTJac2 - fixed");
  t2fix.start();
  for (int i = 0; i < testCount; ++i)
  {
    Jacobian resJ2 = AdTJac2(T, fixedJ);
  }
  t2fix.stop();

  Timer t3dyn("AdTJac3 - dynamic");
  t3dyn.start();
  for (int i = 0; i < testCount; ++i)
  {
    Jacobian resJ3 = AdTJac3(T, dynamicJ);
  }
  t3dyn.stop();

  Timer t3fix("AdTJac3 - fixed");
  t3fix.start();
  for (int i = 0; i < testCount; ++i)
  {
    Jacobian resJ3 = AdTJac3(T, fixedJ);
  }
  t3fix.stop();

  t1dyn.print();
  t2dyn.print();
  t3dyn.print();

  t1fix.print();
  t2fix.print();
  t3fix.print();

  // Note: The best function for dynamic size Jacobian is AdTJac2, and the best
  //       function for fixed size Jacobian is AdTJac3
}
