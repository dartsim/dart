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

//==============================================================================
template <typename S>
Isometry3<S> expMap(const Vector6<S>& _S)
{
  // R = Exp(w)
  // p = sin(t) / t*v + (t - sin(t)) / t^3*<w, v>*w + (1 - cos(t)) / t^2*(w X v)
  // , when S = (w, v), t = |w|

  Isometry3<S> ret = Isometry3<S>::Identity();
  S s2[] = {_S[0] * _S[0], _S[1] * _S[1], _S[2] * _S[2]};
  S s3[] = {_S[0] * _S[1], _S[1] * _S[2], _S[2] * _S[0]};
  S theta = sqrt(s2[0] + s2[1] + s2[2]);
  S cos_t = cos(theta), alpha, beta, gamma;

  if (theta > LieGroupTol<S>()) {
    S sin_t = sin(theta);
    alpha = sin_t / theta;
    beta = (1.0 - cos_t) / theta / theta;
    gamma = (_S[0] * _S[3] + _S[1] * _S[4] + _S[2] * _S[5]) * (theta - sin_t)
            / theta / theta / theta;
  } else {
    alpha = 1.0 - theta * theta / 6.0;
    beta = 0.5 - theta * theta / 24.0;
    gamma = (_S[0] * _S[3] + _S[1] * _S[4] + _S[2] * _S[5]) / 6.0
            - theta * theta / 120.0;
  }

  ret(0, 0) = beta * s2[0] + cos_t;
  ret(1, 0) = beta * s3[0] + alpha * _S[2];
  ret(2, 0) = beta * s3[2] - alpha * _S[1];

  ret(0, 1) = beta * s3[0] - alpha * _S[2];
  ret(1, 1) = beta * s2[1] + cos_t;
  ret(2, 1) = beta * s3[1] + alpha * _S[0];

  ret(0, 2) = beta * s3[2] + alpha * _S[1];
  ret(1, 2) = beta * s3[1] - alpha * _S[0];
  ret(2, 2) = beta * s2[2] + cos_t;

  ret(0, 3)
      = alpha * _S[3] + beta * (_S[1] * _S[5] - _S[2] * _S[4]) + gamma * _S[0];
  ret(1, 3)
      = alpha * _S[4] + beta * (_S[2] * _S[3] - _S[0] * _S[5]) + gamma * _S[1];
  ret(2, 3)
      = alpha * _S[5] + beta * (_S[0] * _S[4] - _S[1] * _S[3]) + gamma * _S[2];

  return ret;
}

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

template <typename S>
struct SE3Test : public testing::Test
{
  using Scalar = S;
};

using Types = testing::Types<float, double>;
TYPED_TEST_SUITE(SE3Test, Types);

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
      << "quat_params: " << x.rotation().quaternion().coeffs().transpose();
  EXPECT_LE(x.rotation().quaternion().norm(), 1)
      << "quat_params: " << x.rotation().quaternion().coeffs().transpose();
  EXPECT_TRUE(x.rotation().toMatrix().determinant() > 0);
}

//==============================================================================
TYPED_TEST(SE3Test, InverseStaticProperties)
{
  using S = typename TestFixture::Scalar;

  constexpr auto is_same
      = std::is_same_v<typename SE3<S>::InverseType, SE3Inverse<SE3<S>>>;
  EXPECT_TRUE(is_same);
}

//==============================================================================
TYPED_TEST(SE3Test, Inverse)
{
  using S = typename TestFixture::Scalar;

  SE3<S> x = SE3<S>::Random();
  EXPECT_TRUE(x.inverse().inverse().isApprox(x));
}

//==============================================================================
TYPED_TEST(SE3Test, InverseInPlace)
{
  using S = typename TestFixture::Scalar;

  SE3<S> x = SE3<S>::Random();
  const SE3<S> x_inverse = x.inverse();
  x.inverseInPlace();
  EXPECT_TRUE(x_inverse.isApprox(x));
}

//==============================================================================
TYPED_TEST(SE3Test, Exp)
{
  using S = typename TestFixture::Scalar;

  const auto num_tests = 100;

  for (auto i = 0; i < num_tests; ++i) {
    SE3Tangent<S> x = SE3Tangent<S>::Random();
    EXPECT_TRUE(Exp(x).toMatrix().isApprox(expMap<S>(x).matrix()))
        << "x          : " << x.params().transpose() << "\n"
        << "Exp(x)   :\n"
        << Exp(x).toMatrix() << "\n"
        << "expMap(x):\n"
        << expMap<S>(x).matrix() << "\n";
  }

  for (auto i = 0; i < num_tests; ++i) {
    SE3<S> x = SE3<S>::Random();
    EXPECT_EQ(Exp(Log(x)), x)
        << "x          : \n"
        << x.toMatrix() << "\n"
        << "Exp(Log(x)): \n"
        << Exp(Log(x)).toMatrix() << "\n"
        << "Log(x)          : " << Log(x).params().transpose() << "\n"
        << "Log(Exp(Log(x))): " << Log(Exp(Log(x))).params().transpose()
        << "\n";
  }
}

//==============================================================================
TYPED_TEST(SE3Test, HatAndVeeOperators)
{
  using S = typename TestFixture::Scalar;

  SE3Tangent<S> vec(1, 2, 3, 4, 5, 6);
  Matrix4<S> mat;
  // clang-format off
  mat <<
     0, -3,  2, 4,
     3,  0, -1, 5,
    -2,  1,  0, 6,
     0,  0,  0, 0;
  // clang-format on

  // Check if the result is equal to the expected matrix
  EXPECT_TRUE(Hat(vec).isApprox(mat));

  // Check if the result is equal to the expected vector
  EXPECT_TRUE(SE3<S>::Vee(mat).isApprox(vec));

  // Check if the result is equal to the original vector
  EXPECT_TRUE(SE3<S>::Vee(Hat(vec)).isApprox(vec));

  // Check if the result is equal to the original matrix
  EXPECT_TRUE(Hat(SE3<S>::Vee(mat)).isApprox(mat));
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
  Vector6<S> xi_inv
      = SE3<S>::Ad(x.inverse().eval(), out); // TODO: Remove eval()
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
  using SE3Tangent = SE3Tangent<S>;

  // TODO(JS): Make this general for all the Lie group types
  auto test_numerical_left_jacobian = [&](const Vector6& x) {
    const S eps = test::EpsForDiff<S>();
    Matrix6 jac_numeric;
    for (int j = 0; j < 6; ++j) {
      Vector6 x_minus = x;
      x_minus[j] -= S(0.5) * eps;

      Vector6 x_plus = x;
      x_plus[j] += S(0.5) * eps;

      const SE3 T_minus = SE3Tangent(x_minus).exp();
      const SE3 T_plus = SE3Tangent(x_plus).exp();
      const SE3 dT_left = T_plus * T_minus.inverse();
      const SE3Tangent dt = dT_left.log();
      const Matrix4 dt_dt = Hat(dt) / eps;
      jac_numeric.col(j) = SE3::Vee(dt_dt).params();
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

      const SE3 T_minus = SE3Tangent(x_minus).exp();
      const SE3 T_plus = SE3Tangent(x_plus).exp();
      const SE3 dT_right = T_minus.inverse() * T_plus;
      const SE3Tangent dt = dT_right.log();
      const Matrix4 dt_dt = Hat(dt) / eps;
      jac_numeric.col(j) = SE3::Vee(dt_dt).params();
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
    EXPECT_TRUE(SE3Tangent(V + dV).exp().isApprox(
        SE3Tangent(SE3::LeftJacobian(V) * dV).exp() * SE3Tangent(V).exp(),
        1e-6))
        << "exp(x + dx):\n"
        << SE3Tangent(V + dV).exp().toMatrix() << "\n"
        << "exp(J_l(x) * dx) * exp(x):\n"
        << (SE3Tangent(SE3::LeftJacobian(V) * dV).exp() * SE3Tangent(V).exp())
               .toMatrix();

    // exp(x + dx) ~= exp(x) * exp(J_r(x) * dx)
    EXPECT_TRUE(SE3Tangent(V + dV).exp().isApprox(
        SE3Tangent(V).exp() * SE3Tangent(SE3::RightJacobian(V) * dV).exp(),
        1e-6))
        << "exp(x + dx):\n"
        << SE3Tangent(V + dV).exp().toMatrix() << "\n"
        << "exp(x) * exp(J_r(x) * dx):\n"
        << (SE3Tangent(V).exp() * SE3Tangent(SE3::RightJacobian(V) * dV).exp())
               .toMatrix();

    // log(exp(dx) * exp(x)) ~= x + J_l^{-1}(x) * dx
    EXPECT_TRUE(
        (SE3Tangent(dV).exp() * SE3Tangent(V).exp())
            .isApprox(
                SE3Tangent(V + SE3::LeftJacobianInverse(V) * dV).exp(), 1e-6))
        << "exp(dx) * exp(x):\n"
        << (SE3Tangent(dV).exp() * SE3Tangent(V).exp()).toMatrix() << "\n"
        << "exp(x + J_l^{-1}(x) * dx):\n"
        << SE3Tangent(V + SE3::LeftJacobianInverse(V) * dV).exp().toMatrix();

    // log(exp(x) * exp(dx)) ~= x + J_r^{-1}(x) * dx
    EXPECT_TRUE(
        (SE3Tangent(V).exp() * SE3Tangent(dV).exp())
            .isApprox(
                SE3Tangent(V + SE3::RightJacobianInverse(V) * dV).exp(), 1e-6))
        << "exp(x) * exp(dx):\n"
        << (SE3Tangent(V).exp() * SE3Tangent(dV).exp()).toMatrix() << "\n"
        << "exp(x + J_r^{-1}(x) * dx):\n"
        << SE3Tangent(V + SE3::RightJacobianInverse(V) * dV).exp().toMatrix();

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
TYPED_TEST(SE3Test, Multiplication)
{
  using S = typename TestFixture::Scalar;

  // Set arbitrary values for se3_1 and se3_2
  SE3<S> x1 = SE3<S>::Random();
  SE3<S> x2 = SE3<S>::Random();

  // Test operator* for SE3Base
  SE3<S> result = x1 * x2;
  // Assert that the result is as expected
  EXPECT_EQ(result.rotation(), x1.rotation() * x2.rotation());
  EXPECT_EQ(
      result.translation(),
      x1.translation() + x1.rotation() * x2.translation());

  // Test operator* for Vector3
  Vector3<S> vec = Vector3<S>::Random(); // Set arbitrary value for vec
  Vector3<S> result_vec = x1 * vec;
  // Assert that the result is as expected
  EXPECT_EQ(result_vec, x1.translation() + x1.rotation() * vec);
}

//==============================================================================
TYPED_TEST(SE3Test, MapConstructor)
{
  using S = typename TestFixture::Scalar;

  // Test the constructor for the const specialization
  S data[] = {1, 0, 0, 0, 1, 2, 3};
  ::Eigen::Map<const SE3<S>, Eigen::Unaligned> const_map(data);
  EXPECT_EQ(const_map.params().data(), data);

  // Test the constructor for the non-const specialization
  Map<SE3<S>, Eigen::Unaligned> nonconst_map(data);
  EXPECT_EQ(nonconst_map.params().data(), data);
}

//==============================================================================
TYPED_TEST(SE3Test, TestDataAccess)
{
  using S = typename TestFixture::Scalar;

  S data[] = {1, 0, 0, 0, 1, 2, 3};
  Map<const SE3<S>, Eigen::Unaligned> const_map(data);
  EXPECT_EQ(const_map.params().data(), data);

  Map<SE3<S>, Eigen::Unaligned> nonconst_map(data);
  EXPECT_EQ(nonconst_map.params().data(), data);

  // Modify the underlying data
  data[1] = 1.0;
  data[2] = 2.0;
  data[3] = 3.0;
  data[4] = 4.0;
  data[5] = 5.0;
  data[6] = 6.0;

  // Check that the modifications are visible through the non-const Map
  for (int i = 0; i < 7; ++i) {
    EXPECT_EQ(nonconst_map.params()[i], data[i]);
  }

  // Check that the modifications are visible even through the const Map
  for (int i = 0; i < 7; ++i) {
    EXPECT_EQ(nonconst_map.params()[i], data[i]);
  }

  // Modify the data through the non-const Map
  nonconst_map = SE3<S>::Random();
  for (int i = 0; i < 7; ++i) {
    EXPECT_EQ(nonconst_map.params()[i], data[i]);
    EXPECT_EQ(nonconst_map.params()[i], data[i]);
  }
}
