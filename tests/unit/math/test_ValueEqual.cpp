/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "dart/math/Helpers.hpp"

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
