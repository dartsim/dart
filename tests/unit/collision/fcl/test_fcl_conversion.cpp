/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include <gtest/gtest.h>

#include <dart/collision/collision.hpp>
#include <dart/math/math.hpp>
#include <dart/test/GTestUtils.hpp>

using namespace dart;

//==============================================================================
template <typename T>
struct FclConversion : public testing::Test
{
  using Type = T;
};

//==============================================================================
using Types = testing::Types<double, float>;

//==============================================================================
TYPED_TEST_CASE(FclConversion, Types);

//==============================================================================
TYPED_TEST(FclConversion, Vector)
{
  using S = typename TestFixture::Type;

  const math::Vector3<S> vec3 = math::Vector3<S>::Random();
  collision::FclVector3<S> fclVec3;
  for (auto i = 0; i < 3; ++i)
  {
    fclVec3[i] = vec3[i];
  }

  EXPECT_VECTOR3S_EQ(fclVec3, collision::toFclVector3<S>(vec3));
  EXPECT_VECTOR3S_EQ(vec3, collision::toVector3<S>(fclVec3));
}

//==============================================================================
TYPED_TEST(FclConversion, Matrix)
{
  using S = typename TestFixture::Type;

  const math::Matrix3<S> mat3 = math::Matrix3<S>::Random();
  collision::FclMatrix3<S> fclMat3;
  for (auto i = 0; i < 3; ++i)
  {
    for (auto j = 0; j < 3; ++j)
    {
      fclMat3(i, j) = mat3(i, j);
    }
  }

  EXPECT_MATRIX3S_EQ(fclMat3, collision::toFclMatrix3<S>(mat3));
  EXPECT_MATRIX3S_EQ(mat3, collision::toMatrix3<S>(fclMat3));
}

//==============================================================================
TYPED_TEST(FclConversion, Transform)
{
  using S = typename TestFixture::Type;

  math::Isometry3<S> tf3 = math::Isometry3<S>::Identity();
  tf3.linear() = math::Random::uniformRotationMatrix3<S>();
  tf3.translation() = math::Vector3<S>::Random();

  EXPECT_TRANSFORM3S_EQ(
      tf3, collision::toTransform3<S>(collision::toFclTransform3<S>(tf3)));
}
