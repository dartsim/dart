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

#include "dart/math/LieGroups.hpp"
#include "dart/test/math/GTestUtils.hpp"

#include <gtest/gtest.h>

using namespace dart;
using namespace math;

namespace {

} // namespace

template <typename S>
struct LieGroupProductTest : public testing::Test
{
  using Scalar = S;
};

using Types = testing::Types<float, double>;
TYPED_TEST_SUITE(LieGroupProductTest, Types);

//==============================================================================
TYPED_TEST(LieGroupProductTest, StaticProperties)
{
  using S = typename TestFixture::Scalar;

  const auto total_param_size = LieGroupProduct<S, SE3, SO3>::ParamSize;
  EXPECT_EQ(total_param_size, SE3<S>::ParamSize + SO3<S>::ParamSize);

  const auto product_size = LieGroupProduct<S, SE3, SO3>::ProductSize;
  EXPECT_EQ(product_size, 2);
}

//==============================================================================
TYPED_TEST(LieGroupProductTest, DefaultConstructor)
{
  using S = typename TestFixture::Scalar;

  SE3<S> se3 = SE3<S>::Random();
  SO3<S> so3 = SO3<S>::Random();

  // Default constructor
  auto product = LieGroupProduct<S, SE3, SO3>();
  EXPECT_EQ(product.ParamSize, se3.ParamSize + so3.ParamSize);

  // Check if the components are their identieis
  auto comp_0 = product.template get<0>();
  EXPECT_EQ(comp_0, SE3<S>::Identity());
  EXPECT_TRUE(comp_0.params().isApprox(SE3<S>::Identity().params()));
  auto comp_1 = product.template get<1>();
  EXPECT_EQ(comp_1, SO3<S>::Identity());
  EXPECT_TRUE(comp_1.params().isApprox(SO3<S>::Identity().params()));
}

//==============================================================================
TYPED_TEST(LieGroupProductTest, ConstructFromComponents)
{
  using S = typename TestFixture::Scalar;

  SE3<S> se3 = SE3<S>::Random();
  SO3<S> so3 = SO3<S>::Random();

  // Copy constructor from the components
  auto product = LieGroupProduct<S, SE3, SO3>(se3, so3);

  // Check the parameters
  int offset = 0;
  EXPECT_TRUE(product.params()
                  .template segment<SE3<S>::ParamSize>(offset)
                  .eval()
                  .isApprox(se3.params()));
  offset += SE3<S>::ParamSize;
  EXPECT_TRUE(product.params()
                  .template segment<SO3<S>::ParamSize>(offset)
                  .eval()
                  .isApprox(so3.params()));

  // Check the components
  auto comp_0 = product.template get<0>();
  EXPECT_EQ(comp_0, se3);
  EXPECT_TRUE(comp_0.params().isApprox(se3.params()));
  auto comp_1 = product.template get<1>();
  EXPECT_EQ(comp_1, so3);
  EXPECT_TRUE(comp_1.params().isApprox(so3.params()));
}
