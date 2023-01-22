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
struct SE3Test : public testing::Test
{
  using Scalar = S;
};

using Types = testing::Types<float, double>;
TYPED_TEST_SUITE(SE3Test, Types);

namespace {

//==============================================================================
template <typename S>
[[nodiscard]] Matrix6<S> dexp(const Vector6<S>& V)
{
  const Matrix6<S> Vx = SE3<S>::Cross(V);
  const Matrix6<S> VxVx = Vx * Vx;
  return Matrix6<S>::Identity() + Vx / 2 + VxVx / 6 + Vx * VxVx / 24
         + VxVx * VxVx / 120;
}

//==============================================================================
template <typename S>
[[nodiscard]] Matrix6<S> dlog(const Vector6<S>& V)
{
  const Matrix6<S> Vx = SE3<S>::Cross(V);
  const Matrix6<S> VxVx = Vx * Vx;
  return Matrix6<S>::Identity() - Vx / 2.0 + VxVx / 12.0 - VxVx * VxVx / 720.0;
}

} // namespace

//==============================================================================
TYPED_TEST(SE3Test, Identity)
{
  using S = typename TestFixture::Scalar;

  SE3<S> identity = SE3<S>::Identity();
  EXPECT_EQ(identity.quat_w(), 1.0);
  EXPECT_EQ(identity.quat_x(), 0.0);
  EXPECT_EQ(identity.quat_y(), 0.0);
  EXPECT_EQ(identity.quat_z(), 0.0);
  EXPECT_EQ(identity.x(), 0.0);
  EXPECT_EQ(identity.y(), 0.0);
  EXPECT_EQ(identity.z(), 0.0);
}

//==============================================================================
TYPED_TEST(SE3Test, Random)
{
  using S = typename TestFixture::Scalar;

  SE3<S> x = SE3<S>::Random();
  EXPECT_GE(x.rotation().quaternion().norm(), 0)
      << "quat_coeffs: " << x.rotation().quaternion().coeffs().transpose();
  EXPECT_LE(x.rotation().quaternion().norm(), 1)
      << "quat_coeffs: " << x.rotation().quaternion().coeffs().transpose();
  EXPECT_TRUE(x.rotation().toMatrix().determinant() > 0);
}

//==============================================================================
TYPED_TEST(SE3Test, Inverse)
{
  using S = typename TestFixture::Scalar;

  SE3<S> x = SE3<S>::Random();
  EXPECT_TRUE(x.inverse().inverse().isApprox(x));
}

//==============================================================================
TYPED_TEST(SE3Test, Exp)
{
  using S = typename TestFixture::Scalar;

  SE3<S> x = SE3<S>::Random();
  EXPECT_TRUE(SE3<S>::Exp(SE3<S>::Log(x)).isApprox(x))
      << "x          : \n"
      << x.toMatrix() << "\n"
      << "Exp(Log(x)): \n"
      << SE3<S>::Exp(SE3<S>::Log(x)).toMatrix() << "\n"
      << "Log(x)          : " << SE3<S>::Log(x).transpose() << "\n"
      << "Log(Exp(Log(x))): "
      << SE3<S>::Log(SE3<S>::Exp(SE3<S>::Log(x))).transpose() << "\n";
}

//==============================================================================
TYPED_TEST(SE3Test, HatAndVeeOperators)
{
  using S = typename TestFixture::Scalar;

  Vector6<S> vec(1, 2, 3, 4, 5, 6);
  Matrix4<S> mat;
  // clang-format off
  mat <<
     0, -3,  2, 4,
     3,  0, -1, 5,
    -2,  1,  0, 6,
     0,  0,  0, 0;
  // clang-format on

  // Check if the result is equal to the expected matrix
  EXPECT_TRUE(SE3<S>::Hat(vec).isApprox(mat));

  // Check if the result is equal to the expected vector
  EXPECT_TRUE(SE3<S>::Vee(mat).isApprox(vec));

  // Check if the result is equal to the original vector
  EXPECT_TRUE(SE3<S>::Vee(SE3<S>::Hat(vec)).isApprox(vec));

  // Check if the result is equal to the original matrix
  EXPECT_TRUE(SE3<S>::Hat(SE3<S>::Vee(mat)).isApprox(mat));
}

//==============================================================================
TYPED_TEST(SE3Test, TestAd)
{
  using S = typename TestFixture::Scalar;

  // Test 1: Test with identity SE3
  SE3<S> identity;
  Vector6<S> xi;
  xi << 1, 2, 3, 4, 5, 6;
  Vector6<S> expected_out;
  expected_out << 1, 2, 3, 4, 5, 6;
  Vector6<S> out = SE3<S>::Ad(identity, xi);
  EXPECT_TRUE(out.isApprox(expected_out));

  // Test 2: Test with non-identity SE3
  SE3<S> x = SE3<S>::Random();
  xi << 1, 2, 3, 4, 5, 6;
  out = SE3<S>::Ad(x, xi);
  // Verify the result by using inverse
  Vector6<S> xi_inv = SE3<S>::Ad(x.inverse(), out);
  EXPECT_TRUE(xi_inv.isApprox(xi));
}

//==============================================================================
TYPED_TEST(SE3Test, Jacobians)
{
  using S = typename TestFixture::Scalar;
  using Vector6 = Vector6<S>;
  using Matrix4 = Matrix4<S>;
  using Matrix6 = Matrix6<S>;
  using SE3 = SE3<S>;

  // TODO(JS): Make this general for all the Lie group types
  auto test_numerical_left_jacobian = [&](const Vector6& x) {
    const S eps = test::EpsForDiff<S>();
    Matrix6 jac_numeric;
    for (int j = 0; j < 6; ++j) {
      Vector6 x_minus = x;
      x_minus[j] -= S(0.5) * eps;

      Vector6 x_plus = x;
      x_plus[j] += S(0.5) * eps;

      const SE3 T_minus = SE3::Exp(x_minus);
      const SE3 T_plus = SE3::Exp(x_plus);
      const SE3 dT_left = T_plus * T_minus.inverse();
      const Vector6 dt = SE3::Log(dT_left);
      const Matrix4 dt_dt = SE3::Hat(dt) / eps;
      jac_numeric.col(j) = SE3::Vee(dt_dt);
    }
    EXPECT_TRUE(test::equals(jac_numeric, SE3::LeftJacobian(x)));
  };

  auto test_numerical_right_jacobian = [&](const Vector6& x) {
    const S eps = test::EpsForDiff<S>();
    Matrix6 jac_numeric;
    for (int j = 0; j < 6; ++j) {
      Vector6 x_minus = x;
      x_minus[j] -= S(0.5) * eps;

      Vector6 x_plus = x;
      x_plus[j] += S(0.5) * eps;

      const SE3 T_minus = SE3::Exp(x_minus);
      const SE3 T_plus = SE3::Exp(x_plus);
      const SE3 dT_right = T_minus.inverse() * T_plus;
      const Vector6 dt = SE3::Log(dT_right);
      const Matrix4 dt_dt = SE3::Hat(dt) / eps;
      jac_numeric.col(j) = SE3::Vee(dt_dt);
    }
    EXPECT_TRUE(test::equals(jac_numeric, SE3::RightJacobian(x)));
  };

  for (auto i = 0; i < 1; ++i) {
    const Vector6 V = Vector6::Random();
    const Vector6 dV = 1e-4 * Vector6::Random();

    test_numerical_left_jacobian(V);
    test_numerical_right_jacobian(V);

    EXPECT_TRUE(SE3::LeftJacobian(V).isApprox(dexp(V), 1e-1))
        << "J_l(V):\n"
        << SE3::LeftJacobian(V) << "\n"
        << "dexp_V     :\n"
        << dexp(V);

    EXPECT_TRUE(SE3::LeftJacobianInverse(V).isApprox(dlog(V), 1e-3))
        << "J_l^{-1}(V):\n"
        << SE3::LeftJacobianInverse(V) << "\n"
        << "dlog_V     :\n"
        << dlog(V);

    // exp(x + dx) ~= exp(J_l(x) * dx) * exp(x)
    EXPECT_TRUE(SE3::Exp(V + dV).isApprox(
        SE3::Exp(SE3::LeftJacobian(V) * dV) * SE3::Exp(V), 1e-6))
        << "exp(x + dx):\n"
        << SE3::Exp(V + dV).toMatrix() << "\n"
        << "exp(J_l(x) * dx) * exp(x):\n"
        << (SE3::Exp(SE3::LeftJacobian(V) * dV) * SE3::Exp(V)).toMatrix();

    // exp(x + dx) ~= exp(x) * exp(J_r(x) * dx)
    EXPECT_TRUE(SE3::Exp(V + dV).isApprox(
        SE3::Exp(V) * SE3::Exp(SE3::RightJacobian(V) * dV), 1e-6))
        << "exp(x + dx):\n"
        << SE3::Exp(V + dV).toMatrix() << "\n"
        << "exp(x) * exp(J_r(x) * dx):\n"
        << (SE3::Exp(V) * SE3::Exp(SE3::RightJacobian(V) * dV)).toMatrix();

    // log(exp(dx) * exp(x)) ~= x + J_l^{-1}(x) * dx
    EXPECT_TRUE(
        (SE3::Exp(dV) * SE3::Exp(V))
            .isApprox(SE3::Exp(V + SE3::LeftJacobianInverse(V) * dV), 1e-6))
        << "exp(dx) * exp(x):\n"
        << (SE3::Exp(dV) * SE3::Exp(V)).toMatrix() << "\n"
        << "exp(x + J_l^{-1}(x) * dx):\n"
        << SE3::Exp(V + SE3::LeftJacobianInverse(V) * dV).toMatrix();

    // log(exp(x) * exp(dx)) ~= x + J_r^{-1}(x) * dx
    EXPECT_TRUE(
        (SE3::Exp(V) * SE3::Exp(dV))
            .isApprox(SE3::Exp(V + SE3::RightJacobianInverse(V) * dV), 1e-6))
        << "exp(x) * exp(dx):\n"
        << (SE3::Exp(V) * SE3::Exp(dV)).toMatrix() << "\n"
        << "exp(x + J_r^{-1}(x) * dx):\n"
        << SE3::Exp(V + SE3::RightJacobianInverse(V) * dV).toMatrix();

    // J_l^{-1} == (J_l)^{-1}
    EXPECT_TRUE(SE3::LeftJacobianInverse(V).isApprox(
        SE3::LeftJacobian(V).inverse(), 1e-6))
        << "J_l^{-1}:\n"
        << SE3::LeftJacobianInverse(V) << "\n"
        << "(J_l(x))^{-1}:\n"
        << SE3::LeftJacobian(V).inverse();

    // J_r^{-1} == (J_r)^{-1}
    EXPECT_TRUE(SE3::RightJacobianInverse(V).isApprox(
        SE3::RightJacobian(V).inverse(), 1e-6))
        << "J_r^{-1}:\n"
        << SE3::RightJacobianInverse(V) << "\n"
        << "(J_r(x))^{-1}:\n"
        << SE3::RightJacobian(V).inverse();
  }
}

//==============================================================================
TYPED_TEST(SE3Test, DefaultConstructor)
{
  using S = typename TestFixture::Scalar;

  SE3<S> x;
  // Test that the data is initialized to the identity matrix
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      if (i == j) {
        EXPECT_S_EQ(x.toMatrix()(i, j), 1);
      } else {
        EXPECT_S_EQ(x.toMatrix()(i, j), 0);
      }
    }
  }
}

//==============================================================================
TYPED_TEST(SE3Test, CopyConstructor)
{
  using S = typename TestFixture::Scalar;

  SE3<S> x1 = SE3<S>::Random();
  SE3<S> x2(x1);
  EXPECT_EQ(x1, x2);
}

//==============================================================================
TYPED_TEST(SE3Test, MoveConstructor)
{
  using S = typename TestFixture::Scalar;

  SE3<S> x1 = SE3<S>::Random();
  const SO3<S> rotation = x1.rotation();
  const Vector3<S> translation = x1.translation();
  SE3<S> x2(std::move(x1));
  // Test that x2's data is equal to the original data of x1
  EXPECT_EQ(x2.rotation(), rotation);
  EXPECT_EQ(x2.translation(), translation);
}

//==============================================================================
TYPED_TEST(SE3Test, CopyAssignmentOperator)
{
  using S = typename TestFixture::Scalar;

  SE3<S> x1 = SE3<S>::Random();
  SE3<S> x2;
  x2 = x1;
  EXPECT_EQ(x1, x2);
}

//==============================================================================
TYPED_TEST(SE3Test, MoveAssignmentOperator)
{
  using S = typename TestFixture::Scalar;

  SE3<S> x1 = SE3<S>::Random();
  const SO3<S> rotation = x1.rotation();
  const Vector3<S> translation = x1.translation();
  SE3<S> x2;
  x2 = std::move(x1);
  // Test that x2's data is equal to the original data of x1
  EXPECT_EQ(x2.rotation(), rotation);
  EXPECT_EQ(x2.translation(), translation);
}

//==============================================================================
TYPED_TEST(SE3Test, MapConstructor)
{
  using S = typename TestFixture::Scalar;

  // Test the constructor for the const specialization
  S data[] = {1, 0, 0, 0, 1, 2, 3};
  ::Eigen::Map<const SE3<S>, Eigen::Unaligned> const_map(data);
  EXPECT_EQ(const_map.coeffs().data(), data);

  // Test the constructor for the non-const specialization
  Map<SE3<S>, Eigen::Unaligned> nonconst_map(data);
  EXPECT_EQ(nonconst_map.coeffs().data(), data);
}

//==============================================================================
TYPED_TEST(SE3Test, TestDataAccess)
{
  using S = typename TestFixture::Scalar;

  S data[] = {1, 0, 0, 0, 1, 2, 3};
  Map<const SE3<S>, Eigen::Unaligned> const_map(data);
  EXPECT_EQ(const_map.coeffs().data(), data);

  Map<SE3<S>, Eigen::Unaligned> nonconst_map(data);
  EXPECT_EQ(nonconst_map.coeffs().data(), data);

  // Modify the underlying data
  data[1] = 1.0;
  data[2] = 2.0;
  data[3] = 3.0;
  data[4] = 4.0;
  data[5] = 5.0;
  data[6] = 6.0;

  // Check that the modifications are visible through the non-const Map
  for (int i = 0; i < 7; ++i) {
    EXPECT_EQ(nonconst_map.coeffs()[i], data[i]);
  }

  // Check that the modifications are visible even through the const Map
  for (int i = 0; i < 7; ++i) {
    EXPECT_EQ(nonconst_map.coeffs()[i], data[i]);
  }

  // Modify the data through the non-const Map
  nonconst_map = SE3<S>::Random();
  for (int i = 0; i < 7; ++i) {
    EXPECT_EQ(nonconst_map.coeffs()[i], data[i]);
    EXPECT_EQ(nonconst_map.coeffs()[i], data[i]);
  }
}
