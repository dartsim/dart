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

template <typename S>
struct SpatialInertiaTest : public testing::Test
{
  using Scalar = S;
};

using Types = testing::Types<float, double>;
TYPED_TEST_SUITE(SpatialInertiaTest, Types);

//==============================================================================
TYPED_TEST(SpatialInertiaTest, DefaultConstructor)
{
  using S = typename TestFixture::Scalar;

  auto G = SpatialInertia<S>();
  EXPECT_S_EQ(G.mass(), 1);
  EXPECT_TRUE(G.inertia().isIdentity());
  EXPECT_TRUE(G.isIdentity());
}

//==============================================================================
TYPED_TEST(SpatialInertiaTest, CopyConstructor)
{
  using S = typename TestFixture::Scalar;

  auto G = SpatialInertia<S>();
  auto copy_spatial_inertia = G;
  EXPECT_EQ(copy_spatial_inertia.inertia(), G.inertia());
  EXPECT_EQ(copy_spatial_inertia.mass(), G.mass());
}

//==============================================================================
TYPED_TEST(SpatialInertiaTest, MoveConstructor)
{
  using S = typename TestFixture::Scalar;

  auto G = SpatialInertia<S>();
  auto move_spatial_inertia = std::move(G);
  EXPECT_EQ(move_spatial_inertia.inertia(), G.inertia());
  EXPECT_EQ(move_spatial_inertia.mass(), G.mass());
}

//==============================================================================
TYPED_TEST(SpatialInertiaTest, MoveAssignment)
{
  using S = typename TestFixture::Scalar;

  auto G = SpatialInertia<S>();
  auto move_spatial_inertia
      = SpatialInertia<TypeParam, SpatialInertiaType::AT_COM>();
  move_spatial_inertia = std::move(G);
  EXPECT_EQ(move_spatial_inertia.inertia(), G.inertia());
  EXPECT_EQ(move_spatial_inertia.mass(), G.mass());
}

//==============================================================================
TYPED_TEST(SpatialInertiaTest, Operators)
{
  using S = typename TestFixture::Scalar;

  SpatialInertia<S> identity;
  SpatialInertia<S> custom_inertia(Matrix3<S>::Identity(), 2.0);

  SpatialInertia<S> sum_inertia = identity + custom_inertia;
  EXPECT_EQ(sum_inertia.inertia(), Matrix3<S>::Identity() * 2.0);
  EXPECT_EQ(sum_inertia.mass(), 3.0);

  SpatialInertia<S> assign_inertia = identity;
  assign_inertia = custom_inertia;
  EXPECT_EQ(assign_inertia.inertia(), Matrix3<S>::Identity());
  EXPECT_EQ(assign_inertia.mass(), 2.0);
}

//==============================================================================
TYPED_TEST(SpatialInertiaTest, CopyAssignment)
{
  using S = typename TestFixture::Scalar;

  auto G = SpatialInertia<S>();
  auto copy_spatial_inertia
      = SpatialInertia<TypeParam, SpatialInertiaType::AT_COM>();
  copy_spatial_inertia = G;
  EXPECT_EQ(copy_spatial_inertia.inertia(), G.inertia());
  EXPECT_EQ(copy_spatial_inertia.mass(), G.mass());
}

//==============================================================================
TYPED_TEST(SpatialInertiaTest, Momentum)
{
  using S = typename TestFixture::Scalar;

  EXPECT_EQ(
      SpatialInertia<S>::Identity() * SE3Tangent<S>::Zero(),
      SE3Tangent<S>::Zero());

  const auto V = SE3Tangent<S>::Random();
  EXPECT_EQ(SpatialInertia<S>::Identity() * V, V);
}
