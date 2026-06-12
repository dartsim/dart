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

#include "helpers/gtest_utils.hpp"

#include "dart/math/lie_groups.hpp"

#include <gtest/gtest.h>

#include <vector>

using namespace dart;
using namespace math;

namespace {

// The batch layer must produce exactly the same result as applying the
// single-element operation per element (it is a consolidated view over the same
// types, not a separate implementation).
template <typename T>
struct LieBatchTest : public testing::Test
{
  using Scalar = typename T::Scalar;
};

using Types = testing::Types<SO3<float>, SO3<double>, SE3<float>, SE3<double>>;
TYPED_TEST_SUITE(LieBatchTest, Types);

//==============================================================================
TYPED_TEST(LieBatchTest, ComposeMatchesScalar)
{
  using T = TypeParam;
  using S = typename T::Scalar;
  constexpr int P = T::ParamSize;
  const std::size_t n = 8;

  std::vector<S> lhs(n * P), rhs(n * P), out(n * P);
  for (std::size_t i = 0; i < n; ++i) {
    Eigen::Map<T>(lhs.data() + i * P) = T::Random();
    Eigen::Map<T>(rhs.data() + i * P) = T::Random();
  }

  composeBatch<T>(lhs.data(), rhs.data(), out.data(), n);

  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Map<const T> a(lhs.data() + i * P);
    const Eigen::Map<const T> b(rhs.data() + i * P);
    const Eigen::Map<const T> o(out.data() + i * P);
    const T expected = a * b;
    EXPECT_TRUE(o == expected) << "element " << i;
  }
}

//==============================================================================
TYPED_TEST(LieBatchTest, ExpLogRoundTrip)
{
  using T = TypeParam;
  using S = typename T::Scalar;
  constexpr int P = T::ParamSize;
  constexpr int D = T::DoF;
  const std::size_t n = 8;

  using Tangent = typename T::Tangent;
  std::vector<S> tangents(n * D), groups(n * P), recovered(n * D);
  for (std::size_t i = 0; i < n; ++i) {
    Eigen::Map<Eigen::Matrix<S, D, 1>>(tangents.data() + i * D)
        = Tangent::Random().params();
  }

  expBatch<T>(tangents.data(), groups.data(), n);
  logBatch<T>(groups.data(), recovered.data(), n);

  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Map<const Eigen::Matrix<S, D, 1>> in(tangents.data() + i * D);
    const Eigen::Map<const Eigen::Matrix<S, D, 1>> out(
        recovered.data() + i * D);
    EXPECT_TRUE(test::equals(in, out, test::EpsForDiff<S>()))
        << "element " << i;
  }
}

//==============================================================================
TYPED_TEST(LieBatchTest, ExpMatchesScalar)
{
  using T = TypeParam;
  using S = typename T::Scalar;
  constexpr int P = T::ParamSize;
  constexpr int D = T::DoF;
  const std::size_t n = 8;

  using Tangent = typename T::Tangent;
  std::vector<S> tangents(n * D), groups(n * P);
  for (std::size_t i = 0; i < n; ++i) {
    Eigen::Map<Eigen::Matrix<S, D, 1>>(tangents.data() + i * D)
        = Tangent::Random().params();
  }

  expBatch<T>(tangents.data(), groups.data(), n);

  for (std::size_t i = 0; i < n; ++i) {
    const typename T::Tangent tangent(
        Eigen::Map<const Eigen::Matrix<S, D, 1>>(tangents.data() + i * D));
    const Eigen::Map<const T> o(groups.data() + i * P);
    const T expected = Exp(tangent);
    EXPECT_TRUE(o == expected) << "element " << i;
  }
}

//==============================================================================
TYPED_TEST(LieBatchTest, AdjointMatchesScalar)
{
  using T = TypeParam;
  using S = typename T::Scalar;
  constexpr int P = T::ParamSize;
  constexpr int D = T::DoF;
  const std::size_t n = 8;

  using Tangent = typename T::Tangent;
  std::vector<S> groups(n * P), tangents(n * D), out(n * D);
  for (std::size_t i = 0; i < n; ++i) {
    Eigen::Map<T>(groups.data() + i * P) = T::Random();
    Eigen::Map<Eigen::Matrix<S, D, 1>>(tangents.data() + i * D)
        = Tangent::Random().params();
  }

  adjointBatch<T>(groups.data(), tangents.data(), out.data(), n);

  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Map<const T> g(groups.data() + i * P);
    const Tangent tangent(
        Eigen::Map<const Eigen::Matrix<S, D, 1>>(tangents.data() + i * D));
    const Eigen::Map<const Eigen::Matrix<S, D, 1>> result(out.data() + i * D);
    EXPECT_TRUE(result.isApprox(Ad(g, tangent).params(), LieGroupTol<S>()))
        << "element " << i;
  }
}

} // namespace
