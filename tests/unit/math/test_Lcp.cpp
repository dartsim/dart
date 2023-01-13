/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#include "dart/math/lcp/Lcp.hpp"
#include "dart/math/lcp/Utils.hpp"
#include "dart/test/math/LcpUtils.hpp"

#include <gtest/gtest.h>

using namespace dart;
using namespace math;

template <typename S>
struct LcpTest : public testing::Test
{
  using Scalar = S;
};

using Types = testing::Types<float, double>;
TYPED_TEST_SUITE(LcpTest, Types);

//==============================================================================
TYPED_TEST(LcpTest, PivotingMethods)
{
  using S = typename TestFixture::Scalar;

#if defined(NDEBUG)
  const auto numTests = 4;
  for (const auto n : {1, 2, 4, 8, 16, 32, 64, 128, 256})
#else
  const auto numTests = 4;
  for (const auto n : {1, 2, 4, 8, 16, 32, 64})
#endif
  {
    for (auto i = 0; i < numTests; ++i) {
      const auto [A, b] = test::generateLcpProblem<S>(n);
      Eigen::VectorX<S> x = Eigen::VectorX<S>(n);

      EXPECT_TRUE(dart::math::solveLcpLemke(A, b, &x))
          << "n: " << n << ", #" << i;
      EXPECT_TRUE(dart::math::validateLcp(A, b, x))
          << "x     : " << x.transpose() << "\n"
          << "Ax + b: " << (A * x + b).transpose();
    }
  }
}

//==============================================================================
TYPED_TEST(LcpTest, SweepingMethods)
{
  using S = typename TestFixture::Scalar;

  LcpOption<S> option;
  option.maxIterations = 10000;

#if defined(NDEBUG)
  const auto numTests = 5;
  for (const auto n : {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024})
#else
  const auto numTests = 5;
  for (const auto n : {1, 2, 4, 8, 16, 32, 64, 128, 256})
#endif
  {
    for (auto i = 0; i < numTests; ++i) {
      const auto [A, b] = test::generateValidLcpProblem<S>(n);
      Eigen::VectorX<S> x = Eigen::VectorX<S>(n);

      EXPECT_TRUE(dart::math::solveLcpPsor(A, b, &x, option))
          << "n: " << n << ", #" << i;
      EXPECT_TRUE(dart::math::validateLcp(A, b, x))
          << "x     : " << x.transpose() << "\n"
          << "Ax + b: " << (A * x + b).transpose();
    }
  }
}
