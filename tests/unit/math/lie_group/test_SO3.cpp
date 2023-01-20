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
struct SO3Test : public testing::Test
{
  using Scalar = S;
};

using Types = testing::Types<float, double>;
TYPED_TEST_SUITE(SO3Test, Types);

//==============================================================================
TYPED_TEST(SO3Test, Identity)
{
  using S = typename TestFixture::Scalar;

  SO3<S> identity = SO3<S>::Identity();
  EXPECT_EQ(identity.quaternion().w(), 1.0);
  EXPECT_EQ(identity.quaternion().x(), 0.0);
  EXPECT_EQ(identity.quaternion().y(), 0.0);
  EXPECT_EQ(identity.quaternion().z(), 0.0);
}

//==============================================================================
TYPED_TEST(SO3Test, Random)
{
  using S = typename TestFixture::Scalar;

  SO3<S> so3 = SO3<S>::Random();
  EXPECT_GE(so3.quaternion().norm(), 0);
  EXPECT_LE(so3.quaternion().norm(), 1);
  EXPECT_TRUE(so3.matrix().determinant() > 0);
}

//==============================================================================
TYPED_TEST(SO3Test, DefaultConstructor)
{
  using S = typename TestFixture::Scalar;

  SO3<S, 0> so3;
  // Test that the data is initialized to the identity matrix
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (i == j) {
        EXPECT_S_EQ(so3.matrix()(i, j), 1);
      } else {
        EXPECT_S_EQ(so3.matrix()(i, j), 0);
      }
    }
  }
}

//==============================================================================
TYPED_TEST(SO3Test, ConstructFromQuaternion)
{
  using S = typename TestFixture::Scalar;

  Quaternion<S> q;
  q.coeffs() = Vector4<S>::Random();
  q.normalize();
  SO3<S> so3(q);
  EXPECT_TRUE(so3.quaternion().isApprox(q));
}

//==============================================================================
TYPED_TEST(SO3Test, CopyConstructor)
{
  using S = typename TestFixture::Scalar;

  SO3<S, 0> so3_1 = SO3<S>::Random();
  SO3<S, 0> so3_2(so3_1);
  EXPECT_EQ(so3_1, so3_2);
}

//==============================================================================
TYPED_TEST(SO3Test, MoveConstructor)
{
  using S = typename TestFixture::Scalar;

  SO3<S, 0> so3_1 = SO3<S>::Random();
  const math::Quaternion<S> data = so3_1.quaternion();
  SO3<S, 0> so3_2(std::move(so3_1));
  // Test that so3_2's data is equal to the original data of so3_1
  EXPECT_EQ(so3_2.quaternion(), data);
}

//==============================================================================
TYPED_TEST(SO3Test, CopyAssignmentOperator)
{
  using S = typename TestFixture::Scalar;

  SO3<S, 0> so3_1 = SO3<S>::Random();
  SO3<S, 0> so3_2;
  so3_2 = so3_1;
  EXPECT_EQ(so3_1, so3_2);
}

//==============================================================================
TYPED_TEST(SO3Test, MoveAssignmentOperator)
{
  using S = typename TestFixture::Scalar;

  SO3<S, 0> so3_1 = SO3<S>::Random();
  const math::Quaternion<S> data = so3_1.quaternion();
  SO3<S, 0> so3_2;
  so3_2 = std::move(so3_1);
  // Test that so3_2's data is equal to the original data of so3_1
  EXPECT_EQ(so3_2.quaternion(), data);
}

//==============================================================================
TYPED_TEST(SO3Test, Matrix)
{
  using S = typename TestFixture::Scalar;

  SO3<S> so3 = SO3<S>::Random();
  Matrix3<S> m = so3.matrix();
  EXPECT_TRUE(m.determinant() > 0);
}

//==============================================================================
TYPED_TEST(SO3Test, Quaternion)
{
  using S = typename TestFixture::Scalar;

  SO3<S> so3 = SO3<S>::Random();
  Quaternion<S> q = so3.quaternion();
  EXPECT_GE(q.norm(), 0);
  EXPECT_LE(q.norm(), 1);
}

//==============================================================================
TYPED_TEST(SO3Test, Data)
{
  using S = typename TestFixture::Scalar;

  SO3<S> so3 = SO3<S>::Random();
  EXPECT_S_EQ(so3.data()[0], so3.quaternion().coeffs()[0]);
  EXPECT_S_EQ(so3.data()[1], so3.quaternion().coeffs()[1]);
  EXPECT_S_EQ(so3.data()[2], so3.quaternion().coeffs()[2]);
  EXPECT_S_EQ(so3.data()[3], so3.quaternion().coeffs()[3]);
}
