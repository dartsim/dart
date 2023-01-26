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
Matrix3<S> expMapRot(const Vector3<S>& _q)
{
  // I + sin(t) / t*[S] + (1 - cos(t)) / t^2*[S]^2, where t = |S|

  S theta = _q.norm();

  Matrix3<S> R = Matrix3<S>::Zero();
  Matrix3<S> qss = SO3<S>::Hat(_q);
  Matrix3<S> qss2 = qss * qss;

  if (theta < SO3<S>::Tolerance())
    R = Matrix3<S>::Identity() + qss + 0.5 * qss2;
  else
    R = Matrix3<S>::Identity() + (sin(theta) / theta) * qss
        + ((1 - cos(theta)) / (theta * theta)) * qss2;

  return R;
}

} // namespace

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
  EXPECT_TRUE(so3.toMatrix().determinant() > 0);
}

//==============================================================================
TYPED_TEST(SO3Test, Inverse)
{
  using S = typename TestFixture::Scalar;

  SO3<S> so3 = SO3<S>::Random();
  EXPECT_TRUE(so3.inverse().inverse().isApprox(so3));
}

//==============================================================================
TYPED_TEST(SO3Test, Exp)
{
  using S = typename TestFixture::Scalar;

  const auto num_tests = 100;

  for (auto i = 0; i < num_tests; ++i) {
    Vector3<S> x = Vector3<S>::Random();

    EXPECT_TRUE(SO3<S>::Exp(x).toMatrix().isApprox(expMapRot<S>(x).matrix()))
        << "x          : " << x.transpose() << "\n"
        << "Exp(x)      :\n"
        << SO3<S>::Exp(x).toMatrix() << "\n"
        << "expMapRot(x):\n"
        << expMapRot<S>(x).matrix() << "\n";
  }

  for (auto i = 0; i < num_tests; ++i) {
    SO3<S> x = SO3<S>::Random();

    EXPECT_TRUE(SO3<S>::Exp(x.log().params()).isApprox(x))
        << "x          : \n"
        << x.toMatrix() << "\n"
        << "Exp(Log(x)): \n"
        << SO3<S>::Exp(x.log().params()).toMatrix() << "\n"
        << "Log(x)          : " << x.log().params().transpose() << "\n"
        << "Log(Exp(Log(x))): "
        << SO3<S>::Exp(x.log().params()).log().params().transpose() << "\n";
  }
}

//==============================================================================
TYPED_TEST(SO3Test, HatAndVeeOperators)
{
  using S = typename TestFixture::Scalar;

  Eigen::Vector3<S> vec(1, 2, 3);
  Eigen::Matrix3<S> mat;
  // clang-format off
  mat <<
     0, -3,  2,
     3,  0, -1,
    -2,  1,  0;
  // clang-format on

  // Check if the result is equal to the expected matrix
  EXPECT_TRUE(SO3<S>::Hat(vec).isApprox(mat));

  // Check if the result is equal to the expected vector
  EXPECT_TRUE(SO3<S>::Vee(mat).isApprox(vec));

  // Check if the result is equal to the original vector
  EXPECT_TRUE(SO3<S>::Vee(SO3<S>::Hat(vec)).isApprox(vec));

  // Check if the result is equal to the original matrix
  EXPECT_TRUE(SO3<S>::Hat(SO3<S>::Vee(mat)).isApprox(mat));
}

//==============================================================================
TYPED_TEST(SO3Test, AdjointTransformationMatrix)
{
  using S = typename TestFixture::Scalar;

  Eigen::Quaternion<S> quat(0.5, 0.5, 0.5, 0.5);
  SO3<S> so3(quat);
  Eigen::Matrix3<S> expected_matrix = quat.toRotationMatrix();

  Eigen::Matrix3<S> result = SO3<S>::Ad(so3);

  // Check if the result is equal to the expected matrix
  EXPECT_TRUE(result.isApprox(expected_matrix));
}

//==============================================================================
TYPED_TEST(SO3Test, Jacobians)
{
  using S = typename TestFixture::Scalar;
  using Vector3 = Vector3<S>;
  using Matrix3 = Matrix3<S>;
  using SO3 = SO3<S>;

  auto test_numerical_left_jacobian = [&](const Vector3& x) {
    const S eps = test::EpsForDiff<S>();
    Matrix3 jac_numeric;
    for (int j = 0; j < 3; ++j) {
      Vector3 x_a = x;
      x_a[j] -= S(0.5) * eps;

      Vector3 x_b = x;
      x_b[j] += S(0.5) * eps;

      const SO3 T_a = SO3::Exp(x_a);
      const SO3 T_b = SO3::Exp(x_b);
      const SO3 dT_left = T_b * T_a.inverse();
      const Vector3 dt = dT_left.log().params();
      const Matrix3 dt_dt = SO3::Hat(dt) / eps;
      jac_numeric.col(j) = SO3::Vee(dt_dt);
    }
    EXPECT_TRUE(test::equals(jac_numeric, SO3::LeftJacobian(x)))
        << "left_J_numeric:\n"
        << jac_numeric << "\n"
        << "left_J        :\n"
        << SO3::LeftJacobian(x);
  };

  auto test_numerical_right_jacobian = [&](const Vector3& x) {
    const S eps = test::EpsForDiff<S>();
    Matrix3 jac_numeric;
    for (int j = 0; j < 3; ++j) {
      Vector3 x_a = x;
      x_a[j] -= S(0.5) * eps;

      Vector3 x_b = x;
      x_b[j] += S(0.5) * eps;

      const SO3 T_a = SO3::Exp(x_a);
      const SO3 T_b = SO3::Exp(x_b);
      const SO3 dT_left = T_a.inverse() * T_b;
      const Vector3 dt = dT_left.log().params();
      const Matrix3 dt_dt = SO3::Hat(dt) / eps;
      jac_numeric.col(j) = SO3::Vee(dt_dt);
    }
    EXPECT_TRUE(test::equals(jac_numeric, SO3::RightJacobian(x)))
        << "right_J_numeric:\n"
        << jac_numeric << "\n"
        << "right_J        :\n"
        << SO3::RightJacobian(x);
  };

  for (auto i = 0; i < 1; ++i) {
    const Vector3 xi = Vector3::Random();
    Vector3 dx;
    if constexpr (std::is_same_v<S, float>) {
      dx = 1e-4 * Vector3::Random();
    } else if constexpr (std::is_same_v<S, double>) {
      dx = 1e-8 * Vector3::Random();
    }

    test_numerical_left_jacobian(xi);
    test_numerical_right_jacobian(xi);

    // Ad(exp(J_l(dx) * xi)) = J_l * Ad(exp(xi))
    EXPECT_TRUE(SO3::Ad(SO3::Exp(SO3::LeftJacobian(dx) * xi))
                    .isApprox(
                        SO3::LeftJacobian(dx) * SO3::Ad(SO3::Exp(xi)),
                        SO3::Tolerance() * 1e+1))
        << "Ad(exp(J_l(dx) * xi)):\n"
        << SO3::Ad(SO3::Exp(SO3::LeftJacobian(dx) * xi)) << "\n"
        << "J_l * Ad(exp(xi))    :\n"
        << SO3::LeftJacobian(dx) * SO3::Ad(SO3::Exp(xi));

    // Ad(exp(J_r(dx) * xi)) = Ad(exp(xi)) * J_r
    EXPECT_TRUE(SO3::Ad(SO3::Exp(SO3::RightJacobian(dx) * xi))
                    .isApprox(
                        SO3::Ad(SO3::Exp(xi)) * SO3::RightJacobian(dx),
                        SO3::Tolerance() * 1e+1))
        << "Ad(exp(J_r(dx) * xi)):\n"
        << SO3::Ad(SO3::Exp(SO3::RightJacobian(dx) * xi)) << "\n"
        << "Ad(exp(xi)) * J_r    :\n"
        << SO3::Ad(SO3::Exp(xi)) * SO3::RightJacobian(dx);

    // exp(x + dx) ~= exp(J_l(x) * dx) * exp(x)
    EXPECT_TRUE(SO3::Exp(xi + dx).isApprox(
        SO3::Exp(SO3::LeftJacobian(xi) * dx) * SO3::Exp(xi)))
        << "exp(x + dx):\n"
        << SO3::Exp(xi + dx).toMatrix() << "\n"
        << "exp(J_l(x) * dx) * exp(x):\n"
        << (SO3::Exp(SO3::LeftJacobian(xi) * dx) * SO3::Exp(xi)).toMatrix();

    // exp(x + dx) ~= exp(x) * exp(J_r(x) * dx)
    EXPECT_EQ(
        SO3::Exp(xi + dx), SO3::Exp(xi) * SO3::Exp(SO3::RightJacobian(xi) * dx))
        << "exp(x + dx):\n"
        << SO3::Exp(xi + dx).toMatrix() << "\n"
        << "exp(x) * exp(J_r(x) * dx):\n"
        << (SO3::Exp(xi) * SO3::Exp(SO3::RightJacobian(xi) * dx)).toMatrix();

    // log(exp(dx) * exp(x)) ~= x + J_l^{-1}(x) * dx
    EXPECT_TRUE((SO3::Exp(dx) * SO3::Exp(xi))
                    .isApprox(SO3::Exp(xi + SO3::LeftJacobianInverse(xi) * dx)))
        << "exp(dx) * exp(x):\n"
        << (SO3::Exp(dx) * SO3::Exp(xi)).toMatrix() << "\n"
        << "exp(x + J_l^{-1}(x) * dx):\n"
        << SO3::Exp(xi + SO3::LeftJacobianInverse(xi) * dx).toMatrix();

    // log(exp(x) * exp(dx)) ~= x + J_r^{-1}(x) * dx
    EXPECT_TRUE(
        (SO3::Exp(xi) * SO3::Exp(dx))
            .isApprox(SO3::Exp(xi + SO3::RightJacobianInverse(xi) * dx)))
        << "exp(x) * exp(dx):\n"
        << (SO3::Exp(xi) * SO3::Exp(dx)).toMatrix() << "\n"
        << "exp(x + J_r^{-1}(x) * dx):\n"
        << SO3::Exp(xi + SO3::RightJacobianInverse(xi) * dx).toMatrix();

    // J_l^{-1} == (J_l)^{-1}
    EXPECT_TRUE(
        SO3::LeftJacobianInverse(xi).isApprox(SO3::LeftJacobian(xi).inverse()))
        << "J_l^{-1}:\n"
        << SO3::LeftJacobianInverse(xi) << "\n"
        << "(J_l(x))^{-1}:\n"
        << SO3::LeftJacobian(xi).inverse();

    // J_r^{-1} == (J_r)^{-1}
    EXPECT_TRUE(SO3::RightJacobianInverse(xi).isApprox(
        SO3::RightJacobian(xi).inverse()))
        << "J_r^{-1}:\n"
        << SO3::LeftJacobianInverse(xi) << "\n"
        << "(J_r(x))^{-1}:\n"
        << SO3::LeftJacobian(xi).inverse();
  }
}

//==============================================================================
TYPED_TEST(SO3Test, DefaultConstructor)
{
  using S = typename TestFixture::Scalar;

  SO3<S> x;
  // Test that the data is initialized to the identity matrix
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (i == j) {
        EXPECT_S_EQ(x.toMatrix()(i, j), 1);
      } else {
        EXPECT_S_EQ(x.toMatrix()(i, j), 0);
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
  EXPECT_TRUE(
      so3.quaternion().toRotationMatrix().isApprox(q.toRotationMatrix()));
}

//==============================================================================
TYPED_TEST(SO3Test, CopyConstructor)
{
  using S = typename TestFixture::Scalar;

  SO3<S> x1 = SO3<S>::Random();
  SO3<S> x2(x1);
  EXPECT_EQ(x1, x2);
}

//==============================================================================
TYPED_TEST(SO3Test, MoveConstructor)
{
  using S = typename TestFixture::Scalar;

  SO3<S> x1 = SO3<S>::Random();
  const math::Quaternion<S> data = x1.quaternion();
  SO3<S> x2(std::move(x1));
  // Test that x2's data is equal to the original data of x1
  EXPECT_EQ(x2.quaternion(), data);
}

//==============================================================================
TYPED_TEST(SO3Test, CopyAssignmentOperator)
{
  using S = typename TestFixture::Scalar;

  SO3<S> x1 = SO3<S>::Random();
  SO3<S> x2;
  x2 = x1;
  EXPECT_EQ(x1, x2);
}

//==============================================================================
TYPED_TEST(SO3Test, MoveAssignmentOperator)
{
  using S = typename TestFixture::Scalar;

  SO3<S> x1 = SO3<S>::Random();
  const math::Quaternion<S> data = x1.quaternion();
  SO3<S> x2;
  x2 = std::move(x1);
  // Test that x2's data is equal to the original data of x1
  EXPECT_EQ(x2.quaternion(), data);
}

//==============================================================================
TYPED_TEST(SO3Test, Multiplication)
{
  using S = typename TestFixture::Scalar;

  // Set arbitrary values for se3_1 and se3_2
  SO3<S> x1 = SO3<S>::Random();
  SO3<S> x2 = SO3<S>::Random();

  // Test operator* for SE3Base
  SO3<S> result = x1 * x2;
  // Assert that the result is as expected
  EXPECT_TRUE(result.toMatrix().isApprox(x1.toMatrix() * x2.toMatrix()));

  // Test operator* for Vector3
  Vector3<S> vec = Vector3<S>::Random(); // Set arbitrary value for vec
  Vector3<S> result_vec = x1 * vec;
  // Assert that the result is as expected
  EXPECT_TRUE(result_vec.isApprox(x1.toMatrix() * vec));
}

//==============================================================================
TYPED_TEST(SO3Test, ToMatrix)
{
  using S = typename TestFixture::Scalar;

  SO3<S> x = SO3<S>::Random();
  Matrix3<S> m = x.toMatrix();
  EXPECT_TRUE(m.determinant() > 0);
}

//==============================================================================
TYPED_TEST(SO3Test, Quaternion)
{
  using S = typename TestFixture::Scalar;

  SO3<S> x = SO3<S>::Random();
  Quaternion<S> q = x.quaternion();
  EXPECT_NEAR(q.norm(), 1, SO3<S>::Tolerance());
}

//==============================================================================
TYPED_TEST(SO3Test, Params)
{
  using S = typename TestFixture::Scalar;

  {
    SO3<S> x = SO3<S>::Random();

    EXPECT_S_EQ(x.params()[0], x.quaternion().coeffs()[0]);
    EXPECT_S_EQ(x.params()[1], x.quaternion().coeffs()[1]);
    EXPECT_S_EQ(x.params()[2], x.quaternion().coeffs()[2]);
    EXPECT_S_EQ(x.params()[3], x.quaternion().coeffs()[3]);

    EXPECT_S_EQ(x.quat_w(), x.quaternion().w());
    EXPECT_S_EQ(x.quat_x(), x.quaternion().x());
    EXPECT_S_EQ(x.quat_y(), x.quaternion().y());
    EXPECT_S_EQ(x.quat_z(), x.quaternion().z());
  }

  {
    const SO3<S> x = SO3<S>::Random();

    EXPECT_S_EQ(x.params()[0], x.quaternion().coeffs()[0]);
    EXPECT_S_EQ(x.params()[1], x.quaternion().coeffs()[1]);
    EXPECT_S_EQ(x.params()[2], x.quaternion().coeffs()[2]);
    EXPECT_S_EQ(x.params()[3], x.quaternion().coeffs()[3]);

    EXPECT_S_EQ(x.quat_w(), x.quaternion().w());
    EXPECT_S_EQ(x.quat_x(), x.quaternion().x());
    EXPECT_S_EQ(x.quat_y(), x.quaternion().y());
    EXPECT_S_EQ(x.quat_z(), x.quaternion().z());
  }
}

//==============================================================================
TYPED_TEST(SO3Test, MapConstructor)
{
  using S = typename TestFixture::Scalar;

  // Test the constructor for the const specialization
  S data[] = {1, 0, 0, 0};
  ::Eigen::Map<const SO3<S>, Eigen::Unaligned> const_map(data);
  EXPECT_EQ(const_map.params().data(), data);

  // Test the constructor for the non-const specialization
  Map<SO3<S>, Eigen::Unaligned> nonconst_map(data);
  EXPECT_EQ(nonconst_map.params().data(), data);
}

//==============================================================================
TYPED_TEST(SO3Test, TestDataAccess)
{
  using S = typename TestFixture::Scalar;

  S data[] = {1.0, 0.0, 0.0, 0.0};
  Map<const SO3<S>, Eigen::Unaligned> const_map(data);
  EXPECT_EQ(const_map.params().data(), data);

  Map<SO3<S>, Eigen::Unaligned> nonconst_map(data);
  EXPECT_EQ(nonconst_map.params().data(), data);

  // Modify the underlying data
  data[1] = 1.0;
  data[2] = 2.0;
  data[3] = 3.0;

  // Check that the modifications are visible through the non-const Map
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(nonconst_map.params()[i], data[i]);
  }

  // Check that the modifications are visible even through the const Map
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(nonconst_map.params()[i], data[i]);
  }

  // Modify the data through the non-const Map
  nonconst_map = SO3<S>::Random();
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(nonconst_map.params()[i], data[i]);
    EXPECT_EQ(nonconst_map.params()[i], data[i]);
  }
}
