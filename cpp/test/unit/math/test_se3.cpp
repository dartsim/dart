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

#include "dart/math/all.hpp"
#include "dart/math/lie_group/se3.hpp"
#include "dart/test/math/GTestUtils.hpp"

using namespace dart;
using namespace math;

//==============================================================================
template <typename T>
struct SE3Test : public testing::Test
{
  using Type = T;
};

//==============================================================================
using Types = testing::Types<float /*, double, long double*/>;

//==============================================================================
TYPED_TEST_SUITE(SE3Test, Types);

//==============================================================================
TYPED_TEST(SE3Test, StaticProperties)
{
  using Scalar = typename TestFixture::Type;

  EXPECT_EQ(SE3<Scalar>::SpaceDim, 3);
  EXPECT_EQ(SE3<Scalar>::GroupDim, 6);
  EXPECT_EQ(SE3<Scalar>::MatrixDim, 4);
  EXPECT_EQ(SE3<Scalar>::DataDim, SO3<Scalar>::DataDim + R3<Scalar>::DataDim);
}

//==============================================================================
TYPED_TEST(SE3Test, Constructors)
{
  using Scalar = typename TestFixture::Type;

  // Default constructor
  {
    SE3<Scalar> a;
    (void)a;
  }

  // Copy/move constructors
  {
    auto a = SE3<Scalar>();
    auto b = SE3<Scalar>(a);
    auto c = SE3<Scalar>(std::move(a));
    DART_UNUSED(a, b, c);
  }

  // Copy/move constructors for SO3Base
  {
      // TODO(JS)
  }

  // Copy/move constructors for LieGroupBase
  {
      // TODO(JS)
  }

  // From Eigen::Isometry3
  {
    Eigen::Transform<Scalar, 3, Eigen::Isometry> tf;
    auto a = SE3<Scalar>(tf);
    DART_UNUSED(a);
  }

  // From quaternion types
  {
    Eigen::Quaternion<Scalar> quat;
    Eigen::Matrix<Scalar, 3, 1> trans;
    auto a = SE3<Scalar>(quat, trans);
    auto b = SE3<Scalar>(std::move(quat), std::move(trans));
    auto c = SE3<Scalar>(
        Eigen::Quaternion<Scalar>::Identity(),
        Eigen::Matrix<Scalar, 3, 1>::Random());
    auto d = SE3<Scalar>(
        Eigen::Quaternion<Scalar>::UnitRandom(),
        Eigen::Matrix<Scalar, 3, 1>::Random());
    auto e = SE3<Scalar>(
        Eigen::Quaternion<Scalar>::UnitRandom().toRotationMatrix(),
        Eigen::Matrix<Scalar, 3, 1>::Random());
    DART_UNUSED(a, b, c, d, e);
  }

  {
    auto a = SE3<Scalar>(0, 1, 2);
  }
}

//==============================================================================
TYPED_TEST(SE3Test, ComponentAccessors)
{
  using Scalar = typename TestFixture::Type;

  auto a = SE3<Scalar>::Random();
  a.x() = 3;
  a.y() = 4;
  a.z() = 5;
  a.quat_x() = 6;
  a.quat_y() = 7;
  a.quat_z() = 8;
  a.quat_w() = 9;
  EXPECT_S_EQ(a.x(), 3);
  EXPECT_S_EQ(a.y(), 4);
  EXPECT_S_EQ(a.z(), 5);
  EXPECT_S_EQ(a.quat_x(), 6);
  EXPECT_S_EQ(a.quat_y(), 7);
  EXPECT_S_EQ(a.quat_z(), 8);
  EXPECT_S_EQ(a.quat_w(), 9);
}

//==============================================================================
TYPED_TEST(SE3Test, OrientationAccessors)
{
  using Scalar = typename TestFixture::Type;

  auto a = SE3<Scalar>::Random();
  EXPECT_S_EQ(a.quat_x(), a.to_quaternion().x());
  EXPECT_S_EQ(a.quat_y(), a.to_quaternion().y());
  EXPECT_S_EQ(a.quat_z(), a.to_quaternion().z());
  EXPECT_S_EQ(a.quat_w(), a.to_quaternion().w());

  a.orientation().coeffs().x() = 1;
  a.orientation().coeffs().y() = 2;
  a.orientation().coeffs().z() = 3;
  a.orientation().coeffs().w() = 4;
  EXPECT_S_EQ(a.quat_x(), 1);
  EXPECT_S_EQ(a.quat_y(), 2);
  EXPECT_S_EQ(a.quat_z(), 3);
  EXPECT_S_EQ(a.quat_w(), 4);
}

//==============================================================================
TYPED_TEST(SE3Test, Inverse)
{
  using Scalar = typename TestFixture::Type;

  SE3<Scalar> a = SE3<Scalar>::Random();
  SE3<Scalar> a_inv = a.inverse();
  DART_UNUSED(a_inv);

  SO3<Scalar> o = a.orientation();
  //  R3<Scalar> p = a.position();
  DART_UNUSED(o);

  SE3<Scalar> b = SE3<Scalar>::Random();
  b.orientation().inverse_in_place();
}

//==============================================================================
TYPED_TEST(SE3Test, GroupOperations)
{
  using Scalar = typename TestFixture::Type;

  SE3<Scalar> a = SE3<Scalar>::Random();
  SE3<Scalar> b = SE3<Scalar>::Random();
  SO3<Scalar> c = a.orientation() * b.orientation();
  DART_UNUSED(c);

  // auto tmp = a * R3<Scalar>::Random();
}

////==============================================================================
// TYPED_TEST(SE3Test, Inverse)
//{
//  using Scalar = typename TestFixture::Type;

//  SE3<Scalar> a;
//  SE3<Scalar> b;
//  SE3<Scalar> c;

//  b = a.inverse();
//  c = a.inverse() * b;
//  c = a * b.inverse();
//  c = a.inverse().eval() * b;
//  c = a * b.inverse().eval();
//  c.noalias() = a * b.inverse();
//  c.noalias() = a.inverse() * b;
//}

////==============================================================================
// TYPED_TEST(SE3Test, Jacobians)
//{
//  using Scalar = typename TestFixture::Type;
//  const Scalar eps = test::eps_for_diff<Scalar>();

//  for (auto i = 0; i < 100; ++i) {
//    const SE3Tangent<Scalar> q = SE3<Scalar>::Random().log();
//    Eigen::Matrix<Scalar, 6, 6> jac_numeric;
//    for (int j = 0; j < 6; ++j) {
//      SE3Tangent<Scalar> q_a = q;
//      q_a[j] -= Scalar(0.5) * eps;

//      SE3Tangent<Scalar> q_b = q;
//      q_b[j] += Scalar(0.5) * eps;

//      const SE3<Scalar> T_a = exp(q_a);
//      const SE3<Scalar> T_b = exp(q_b);
//      const SE3<Scalar> dT_left = T_b * T_a.inverse();
//      const SE3Tangent<Scalar> dt = log(dT_left);
//      const SE3Algebra<Scalar> dt_dt = dt.hat() / eps;
//      jac_numeric.col(j) = dt_dt.vee().vector();
//    }
//    EXPECT_TRUE(test::equals(jac_numeric, q.left_jacobian()));
//  }
//}
