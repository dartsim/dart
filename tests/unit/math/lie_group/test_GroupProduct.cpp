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
struct GroupProductTest : public testing::Test
{
  using Scalar = S;
};

using Types = testing::Types<float, double>;
TYPED_TEST_SUITE(GroupProductTest, Types);

//==============================================================================
TYPED_TEST(GroupProductTest, StaticProperties)
{
  using S = typename TestFixture::Scalar;

  const auto total_param_size = GroupProduct<S, SE3, SO3>::ParamSize;
  EXPECT_EQ(total_param_size, SE3<S>::ParamSize + SO3<S>::ParamSize);

  const auto product_size = GroupProduct<S, SE3, SO3>::ProductSize;
  EXPECT_EQ(product_size, 2);
}

//==============================================================================
TYPED_TEST(GroupProductTest, Random)
{
  using S = typename TestFixture::Scalar;

  auto product_1 = GroupProduct<S, SE3, SO3>::Random();
  EXPECT_EQ(
      product_1.template get<0>(), SE3<S>(product_1.template params<0>()));
  EXPECT_EQ(
      product_1.template get<1>(), SO3<S>(product_1.template params<1>()));
}

//==============================================================================
TYPED_TEST(GroupProductTest, DefaultConstructor)
{
  using S = typename TestFixture::Scalar;

  SE3<S> se3 = SE3<S>::Random();
  SO3<S> so3 = SO3<S>::Random();

  // Default constructor
  auto product = GroupProduct<S, SE3, SO3>();
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
TYPED_TEST(GroupProductTest, Cast)
{
  using S = typename TestFixture::Scalar;

  auto product_1 = GroupProduct<S, SE3, SO3>();
  auto product_2 = product_1.template cast<float>();
  auto product_3 = product_1.template cast<double>();

  for (std::size_t i = 0; i < product_1.ParamSize; ++i) {
    EXPECT_FLOAT_EQ(
        static_cast<float>(product_1.params()[i]), product_2.params()[i]);
    EXPECT_DOUBLE_EQ(
        static_cast<double>(product_1.params()[i]), product_3.params()[i]);
  }
}

//==============================================================================
TYPED_TEST(GroupProductTest, ConstructFromComponents)
{
  using S = typename TestFixture::Scalar;

  {
    SE3<S> se3 = SE3<S>::Random();
    SO3<S> so3 = SO3<S>::Random();

    // Copy constructor from the components
    auto product = GroupProduct<S, SE3, SO3>(se3, so3);

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
}

//==============================================================================
template <typename S, template <typename> class... T>
void TestMapParams()
{
  using ProductType = GroupProduct<S, T...>;
  const auto ParamSize = ProductType::ProductSize;

  VectorX<S> params = VectorX<S>::Random(ParamSize);

  Map<const ProductType> map_const(params.data());
  Map<ProductType> map(params.data());

  // Check that the params() function returns the correct data
  const auto& params_const = map_const.params();
  for (std::size_t i = 0; i < ParamSize; i++) {
    EXPECT_EQ(params_const[i], params[i]);
  }

  for (std::size_t i = 0; i < ParamSize; i++) {
    EXPECT_EQ(map.params()[i], params[i]);
  }

  // Check that the params() function returns a non-const reference for the
  // non-const Map class
  params[0] = 5;
  EXPECT_EQ(params[0], 5);
}

//==============================================================================
TYPED_TEST(GroupProductTest, MapParams)
{
  using S = typename TestFixture::Scalar;

  TestMapParams<S, SO3>();
  TestMapParams<S, SE3>();

  TestMapParams<S, SO3, SO3>();
  TestMapParams<S, SE3, SE3>();

  TestMapParams<S, SO3, SO3, SO3>();
  TestMapParams<S, SE3, SE3, SE3>();

  TestMapParams<S, SO3, SE3>();
  TestMapParams<S, SE3, SO3>();

  TestMapParams<S, SO3, SE3, SO3>();
  TestMapParams<S, SE3, SO3, SE3>();
}
