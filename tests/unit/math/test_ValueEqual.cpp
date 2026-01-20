/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include "dart/math/helpers.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <limits>

#include <cmath>

using dart::math::valueEqual;

TEST(ValueEqualTest, ScalarFloatingPointEquality)
{
  EXPECT_TRUE(valueEqual(1.0, 1.0));
  EXPECT_FALSE(valueEqual(1.0, 2.0));

  const double base = 1.0;
  const double next
      = std::nextafter(base, std::numeric_limits<double>::infinity());
  EXPECT_FALSE(valueEqual(base, next));

  const double nanVal = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(valueEqual(nanVal, nanVal));
  EXPECT_FALSE(valueEqual(nanVal, 0.0));

  EXPECT_TRUE(valueEqual(
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(valueEqual(
      std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity()));
}

TEST(ValueEqualTest, SignedZeroesAreEqual)
{
  EXPECT_TRUE(valueEqual(0.0, -0.0));
  EXPECT_TRUE(valueEqual(-0.0f, 0.0f));
}

TEST(ValueEqualTest, IntegralEquality)
{
  EXPECT_TRUE(valueEqual(7, 7));
  EXPECT_FALSE(valueEqual(7, 8));
  EXPECT_TRUE(valueEqual(static_cast<unsigned>(3), static_cast<unsigned>(3)));
}

TEST(ValueEqualTest, IsEqualAlias)
{
  using dart::math::isEqual;

  EXPECT_TRUE(isEqual(3.0, 3.0));
  EXPECT_FALSE(isEqual(3.0, 4.0));
}

TEST(ValueEqualTest, EigenContainers)
{
  Eigen::Vector2d a;
  a << 0.0, -0.0;

  Eigen::Vector2d b;
  b << -0.0, 0.0;

  EXPECT_TRUE(valueEqual(a, b));

  Eigen::Vector2d c = a;
  c[1] = std::nextafter(1.0, 2.0);
  EXPECT_FALSE(valueEqual(a, c));
}

TEST(ValueEqualTest, EigenDimensionMismatch)
{
  Eigen::Vector2d a = Eigen::Vector2d::Zero();
  Eigen::Vector3d b = Eigen::Vector3d::Zero();

  EXPECT_FALSE(valueEqual(a, b));

  using dart::math::isApprox;
  EXPECT_FALSE(isApprox(a, b));
}

TEST(ValueEqualTest, IsApproxScalar)
{
  using dart::math::isApprox;

  EXPECT_TRUE(isApprox(1.0, 1.0));
  EXPECT_TRUE(isApprox(1.0, 1.0 + 1e-7, 1e-6, 1e-6));
  EXPECT_FALSE(isApprox(1.0, 1.0 + 1e-3, 1e-6, 1e-6));

  EXPECT_TRUE(isApprox(
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(isApprox(
      std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity()));

  EXPECT_FALSE(isApprox(
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()));
}

TEST(ValueEqualTest, IsApproxIntegralScalar)
{
  using dart::math::isApprox;

  EXPECT_TRUE(isApprox(5, 5));
  EXPECT_FALSE(isApprox(5, 6));
}

TEST(ValueEqualTest, IsApproxEigen)
{
  using dart::math::isApprox;

  Eigen::Vector2d v1;
  v1 << 1.0, 2.0;
  Eigen::Vector2d v2 = v1;
  v2[1] += 1e-7;

  EXPECT_TRUE(isApprox(v1, v2, 1e-6, 1e-6));
  EXPECT_FALSE(isApprox(v1, v2, 1e-8, 1e-8));
}

TEST(ValueEqualTest, IsApproxEigenIntegral)
{
  using dart::math::isApprox;

  Eigen::Vector2i i1;
  i1 << 1, 2;
  Eigen::Vector2i i2 = i1;
  i2[1] = 3;

  EXPECT_TRUE(isApprox(i1, i1));
  EXPECT_FALSE(isApprox(i1, i2));
}

TEST(ValueEqualTest, IsZeroScalar)
{
  using dart::math::isZero;

  EXPECT_TRUE(isZero(0.0));
  EXPECT_TRUE(isZero(-0.0));
  EXPECT_TRUE(isZero(1e-7, 1e-6));
  EXPECT_FALSE(isZero(1e-3, 1e-6));
  EXPECT_FALSE(isZero(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_TRUE(isZero(0));
  EXPECT_FALSE(isZero(5));
}

TEST(ValueEqualTest, IsZeroEigen)
{
  using dart::math::isZero;

  Eigen::VectorXd empty;
  EXPECT_TRUE(isZero(empty));

  Eigen::Vector3d zeros = Eigen::Vector3d::Zero();
  EXPECT_TRUE(isZero(zeros));

  Eigen::Vector3d nearZero = Eigen::Vector3d::Constant(5e-7);
  EXPECT_TRUE(isZero(nearZero, 1e-6));

  nearZero[1] = 1e-3;
  EXPECT_FALSE(isZero(nearZero, 1e-6));
}

TEST(ValueEqualTest, IsIntUsesAbsoluteTolerance)
{
  using dart::math::isInt;

  EXPECT_TRUE(isInt(0.0));
  EXPECT_TRUE(isInt(1.0));
  EXPECT_FALSE(isInt(0.5));
  EXPECT_FALSE(isInt(-0.5));

  EXPECT_TRUE(isInt(1.0 + 5e-7));
  EXPECT_FALSE(isInt(1.0 + 2e-6));

  const double large = 1e12;
  EXPECT_TRUE(isInt(large));
  EXPECT_FALSE(isInt(large + 0.5));
}
