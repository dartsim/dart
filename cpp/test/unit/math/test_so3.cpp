/*
 * Copyright (c) 2011-2022, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <chrono>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "dart/common/stopwatch.hpp"
#include "dart/math/all.hpp"
#include "dart/test/math/GTestUtils.hpp"

using namespace dart;
using namespace math;

template <typename S>
struct SO3A
{
  using Quaternion = Eigen::Quaternion<S>;

  SO3A() : m_data(0, 0, 0, 1)
  {
    // Do nothing
  }

  template <typename QuatDerived>
  SO3A(const Eigen::QuaternionBase<QuatDerived>& other) : m_data(other.coeffs())
  {
    // Do nothing
  }

  template <typename QuatDerived>
  SO3A(Eigen::QuaternionBase<QuatDerived>&& other)
    : m_data(std::move(other.coeffs()))
  {
    // Do nothing
    //    std::cout << "hm";
  }

  SO3A operator*(const SO3A& other) const
  {
    return SO3A(quat() * other.quat());
  }

  auto quat() const
  {
    return Eigen::Map<const Quaternion>(m_data.data());
  }

  Eigen::Matrix<S, 4, 1> m_data;
};

template <typename S>
struct SO3B
{
  SO3B() : m_data(1, 0, 0, 0)
  {
    // Do nothing
  }

  template <typename QuatDerived>
  SO3B(const Eigen::QuaternionBase<QuatDerived>& other) : m_data(other)
  {
    // Do nothing
  }

  template <typename QuatDerived>
  SO3B(Eigen::QuaternionBase<QuatDerived>&& other) : m_data(std::move(other))
  {
    // Do nothing
    //    std::cout << "hm2";
  }

  SO3B operator*(const SO3B& other) const
  {
    return SO3B(m_data * other.m_data);
  }

  Eigen::Quaternion<S> m_data;
};

template <typename S>
struct SE3A
{
  using Quaternion = Eigen::Quaternion<S>;
  using QuaternionMap = Eigen::Map<Eigen::Quaternion<S>>;
  using ConstQuaternionMap = Eigen::Map<const Eigen::Quaternion<S>>;

  SE3A() : m_data(1, 0, 0, 0, 0, 0, 0)
  {
    // Do nothing
  }

#if EIGEN_VERSION_AT_LEAST(3, 4, 0)
  template <typename QuatDerived, typename MatrixDerived>
  SE3A(
      const Eigen::QuaternionBase<QuatDerived>& orientation,
      const Eigen::MatrixBase<MatrixDerived>& position)
    : m_data(
        orientation.coeffs()[0],
        orientation.coeffs()[1],
        orientation.coeffs()[2],
        orientation.coeffs()[3],
        position[0],
        position[1],
        position[2])
  {
    // Do nothing
  }
#else
  template <typename QuatDerived, typename MatrixDerived>
  SE3A(
      const Eigen::QuaternionBase<QuatDerived>& orientation,
      const Eigen::MatrixBase<MatrixDerived>& position)
  {
    m_data.template head<4>() = orientation.coeffs();
    m_data.template tail<3>() = position;
  }
#endif

#if EIGEN_VERSION_AT_LEAST(3, 4, 0)
  template <typename QuatDerived, typename MatrixDerived>
  SE3A(
      Eigen::QuaternionBase<QuatDerived>&& orientation,
      Eigen::MatrixBase<MatrixDerived>&& position)
    : m_data(
        orientation.coeffs()[0],
        orientation.coeffs()[1],
        orientation.coeffs()[2],
        orientation.coeffs()[3],
        position[0],
        position[1],
        position[2])
  {
    // Do nothing
  }
#else
  template <typename QuatDerived, typename MatrixDerived>
  SE3A(
      Eigen::QuaternionBase<QuatDerived>&& orientation,
      Eigen::MatrixBase<MatrixDerived>&& position)
  {
    m_data.template head<4>() = std::move(orientation.coeffs());
    m_data.template tail<3>() = std::move(position);
  }
#endif

  SE3A operator*(const SE3A& other) const
  {
    return SE3A(
        orientation() * other.orientation(),
        orientation() * other.position() + position());
  }

  //  Eigen::Map<const Eigen::Quaternion<S>> orientation() const
  //  {
  //    return Eigen::Map<const Eigen::Quaternion<S>>(m_data.data());
  //  }

  //  Eigen::Map<Eigen::Quaternion<S>> orientation()
  //  {
  //    return Eigen::Map<Eigen::Quaternion<S>>(m_data.data());
  //  }

  //  Eigen::Map<const Eigen::Matrix<S, 3, 1>> position() const
  //  {
  //    return Eigen::Map<const Eigen::Matrix<S, 3, 1>>(m_data.data() + 4);
  //  }

  //  Eigen::Map<Eigen::Matrix<S, 3, 1>> position()
  //  {
  //    return Eigen::Map<Eigen::Matrix<S, 3, 1>>(m_data.data() + 4);
  //  }

  Quaternion orientation() const
  {
    return Quaternion(m_data.template head<4>());
  }

  Quaternion orientation()
  {
    return Quaternion(m_data.template head<4>());
  }

  Eigen::Matrix<S, 3, 1> position() const
  {
    return Eigen::Matrix<S, 3, 1>(m_data.template tail<3>());
  }

  Eigen::Matrix<S, 3, 1> position()
  {
    return Eigen::Matrix<S, 3, 1>(m_data.template tail<3>());
  }

  Eigen::Matrix<S, 7, 1> m_data;
};

template <typename S>
struct SE3B
{
  SE3B() : m_orientation(1, 0, 0, 0), m_position(0, 0, 0)
  {
    // Do nothing
  }

  template <typename QuatDerived, typename MatrixDerived>
  SE3B(
      const Eigen::QuaternionBase<QuatDerived>& orientation,
      const Eigen::MatrixBase<MatrixDerived>& position)
    : m_orientation(orientation), m_position(position)
  {
    // Do nothing
  }

  template <typename QuatDerived, typename MatrixDerived>
  SE3B(
      Eigen::QuaternionBase<QuatDerived>&& orientation,
      Eigen::MatrixBase<MatrixDerived>&& position)
    : m_orientation(std::move(orientation)), m_position(std::move(position))
  {
    //    std::cout << "hi2";
  }

  SE3B operator*(const SE3B& other) const
  {
    return SE3B(
        m_orientation * other.m_orientation,
        m_orientation * other.m_position + m_position);
  }

  Eigen::Quaternion<S> m_orientation;
  Eigen::Matrix<S, 3, 1> m_position;
};

template <typename S>
struct SE3C
{
  SE3C() : m_tf(Eigen::Transform<S, 3, Eigen::Isometry>::Identity())
  {
    // Do nothing
  }

  template <typename QuatDerived, typename MatrixDerived>
  SE3C(
      const Eigen::QuaternionBase<QuatDerived>& orientation,
      const Eigen::MatrixBase<MatrixDerived>& position)
  {
    m_tf.setIdentity();
    m_tf.linear() = orientation.toRotationMatrix();
    m_tf.translation() = position;
  }

  template <typename QuatDerived, typename MatrixDerived>
  SE3C(
      Eigen::QuaternionBase<QuatDerived>&& orientation,
      Eigen::MatrixBase<MatrixDerived>&& position)
  {
    m_tf.setIdentity();
    m_tf.linear() = orientation.toRotationMatrix();
    m_tf.translation() = std::move(position);
  }

  SE3C(const Eigen::Transform<S, 3, Eigen::Isometry>& tf) : m_tf(tf)
  {
    // Do nothing
  }

  SE3C(Eigen::Transform<S, 3, Eigen::Isometry>&& tf) : m_tf(std::move(tf))
  {
    // Do nothing
  }

  SE3C operator*(const SE3C& other) const
  {
    return SE3C(m_tf * other.m_tf);
  }

  Eigen::Transform<S, 3, Eigen::Isometry> m_tf;
};

//==============================================================================
template <typename T>
struct SO3Test : public testing::Test
{
  using Type = T;
};

//==============================================================================
using Types = testing::Types<float, double>;

//==============================================================================
TYPED_TEST_SUITE(SO3Test, Types);

//==============================================================================
TYPED_TEST(SO3Test, GlobalProperties)
{
  using Scalar = typename TestFixture::Type;

  auto so3_1 = SO3<Scalar>();
  EXPECT_S_EQ(so3_1.to_quaternion().x(), 0);
  EXPECT_S_EQ(so3_1.to_quaternion().y(), 0);
  EXPECT_S_EQ(so3_1.to_quaternion().z(), 0);
  EXPECT_S_EQ(so3_1.to_quaternion().w(), 1);
  EXPECT_EQ(so3_1.coeffs().size(), 4);
}

//==============================================================================
TYPED_TEST(SO3Test, Constructors)
{
  using Scalar = typename TestFixture::Type;

  // Default constructor
  {
    auto so3 = SO3<Scalar>();
  }

  // Copy/move constructors
  {
    auto so3_a = SO3<Scalar>();
    auto so3_b = SO3<Scalar>(so3_a);
    auto so3_c = SO3<Scalar>(std::move(so3_a));
    DART_UNUSED(so3_a, so3_b, so3_c);
  }

  // Copy/move constructors for SO3Base
  {
      // TODO(JS)
  }

  // Copy/move constructors for LieGroupBase
  {
      // TODO(JS)
  }

  // Copy/move constructors for quaternion types
  {
    Eigen::Quaternion<Scalar> quat;
    auto so3_a = SO3<Scalar>(quat);
    auto so3_b = SO3<Scalar>(std::move(quat));
    auto so3_c = SO3<Scalar>(Eigen::Quaternion<Scalar>::Identity());
    auto so3_d = SO3<Scalar>(Eigen::Quaternion<Scalar>::UnitRandom());
    DART_UNUSED(so3_a, so3_b, so3_c, so3_d);
  }

  // Copy/move constructors for matrix types
  {
    Eigen::Matrix<Scalar, 4, 1> vec4;
    auto so3_a = SO3<Scalar>(vec4);
    auto so3_b = SO3<Scalar>(std::move(vec4));
    auto so3_c = SO3<Scalar>(Eigen::Matrix<Scalar, 4, 1>::Zero());
    auto so3_d = SO3<Scalar>(Eigen::Quaternion<Scalar>::UnitRandom().coeffs());
    DART_UNUSED(so3_a, so3_b, so3_c, so3_d);
  }
}

//==============================================================================
TYPED_TEST(SO3Test, Identity)
{
  using Scalar = typename TestFixture::Type;

  auto so3 = SO3<Scalar>(Eigen::Quaternion<Scalar>::UnitRandom());
  (void)so3;
  // so3.set_identity();
}

//==============================================================================
TYPED_TEST(SO3Test, Random)
{
  using Scalar = typename TestFixture::Type;

  auto so3_a = SO3<Scalar>::Random();
  (void)so3_a;

  auto so3_b = SO3<Scalar>();
  so3_b.set_random();
  // so3.set_identity();
}

//==============================================================================
TYPED_TEST(SO3Test, Inverse)
{
  using Scalar = typename TestFixture::Type;

  SO3<Scalar> a = SO3<Scalar>::Random();
  SO3<Scalar> a_inv = a.inverse();
  (void)a_inv;
  a.inverse_in_place();
}

//==============================================================================
TYPED_TEST(SO3Test, GroupOperations)
{
  using Scalar = typename TestFixture::Type;

  SO3<Scalar> a = SO3<Scalar>::Random();
  SO3<Scalar> b = SO3<Scalar>::Random();
  SO3<Scalar> c = a * b;
  DART_UNUSED(c);

  auto c_log = c.log();
  DART_UNUSED(c_log);
}

//==============================================================================
TYPED_TEST(SO3Test, VectorRotation)
{
  using Scalar = typename TestFixture::Type;

  SO3<Scalar> a = SO3<Scalar>::Random();
  R3<Scalar> vec = R3<Scalar>::Random();

  auto vec2 = a * vec;
  (void)vec2;

  SE3<Scalar> se3 = SE3<Scalar>::Random();

  R3<Scalar> vec3 = a * se3.position();

  DART_UNUSED(vec3);
}

//==============================================================================
TYPED_TEST(SO3Test, AssignOperators)
{
  using Scalar = typename TestFixture::Type;

  {
    auto so3_a = SO3<Scalar>();
    auto so3_b = SO3<Scalar>();
    so3_b = so3_a;
  }

  Eigen::Quaternion<Scalar> eigen_quat1;

  Eigen::Quaternion<Scalar> eigen_quat2(Eigen::Matrix<Scalar, 4, 1>::Zero());

  Eigen::Matrix<Scalar, 4, 1> vec3 = Eigen::Matrix<Scalar, 4, 1>::Zero();
  Eigen::Map<Eigen::Quaternion<Scalar>> eigen_quat3(vec3.data());
  eigen_quat3.x() = 4;
}

//==============================================================================
TYPED_TEST(SO3Test, ScalarCast)
{
  using Scalar = typename TestFixture::Type;

  auto so3 = SO3<Scalar>(Eigen::Quaternion<Scalar>::UnitRandom());

  auto float_cast = so3.template cast<float>();
  EXPECT_FLOAT_EQ(so3.coeffs().x(), float_cast.coeffs().x());
  EXPECT_FLOAT_EQ(so3.coeffs().y(), float_cast.coeffs().y());
  EXPECT_FLOAT_EQ(so3.coeffs().z(), float_cast.coeffs().z());
  EXPECT_FLOAT_EQ(so3.coeffs().w(), float_cast.coeffs().w());

  auto double_cast = so3.template cast<double>();
  EXPECT_FLOAT_EQ(so3.coeffs().x(), double_cast.coeffs().x());
  EXPECT_FLOAT_EQ(so3.coeffs().y(), double_cast.coeffs().y());
  EXPECT_FLOAT_EQ(so3.coeffs().z(), double_cast.coeffs().z());
  EXPECT_FLOAT_EQ(so3.coeffs().w(), double_cast.coeffs().w());
}

//==============================================================================
TYPED_TEST(SO3Test, Map)
{
  // using Scalar = typename TestFixture::Type;

  //  auto a = Eigen::Map<SO3<Scalar>>();
  //  (void)a;
}

//==============================================================================
TYPED_TEST(SO3Test, Conversions)
{
  using Scalar = typename TestFixture::Type;

  SO3<Scalar> a = SO3<Scalar>::Random();

  const Eigen::Quaternion<Scalar> eigen_q_a = a.to_quaternion();
  EXPECT_S_EQ(a.coeffs().x(), eigen_q_a.coeffs().x());
  EXPECT_S_EQ(a.coeffs().y(), eigen_q_a.coeffs().y());
  EXPECT_S_EQ(a.coeffs().z(), eigen_q_a.coeffs().z());
  EXPECT_S_EQ(a.coeffs().w(), eigen_q_a.coeffs().w());

  Eigen::Quaternion<Scalar> eigen_q_b = a.to_quaternion();
  EXPECT_S_EQ(a.coeffs().x(), eigen_q_b.coeffs().x());
  EXPECT_S_EQ(a.coeffs().y(), eigen_q_b.coeffs().y());
  EXPECT_S_EQ(a.coeffs().z(), eigen_q_b.coeffs().z());
  EXPECT_S_EQ(a.coeffs().w(), eigen_q_b.coeffs().w());

  eigen_q_b.coeffs().x() = 1;
  eigen_q_b.coeffs().y() = 2;
  eigen_q_b.coeffs().z() = 3;
  eigen_q_b.coeffs().w() = 4;
  EXPECT_S_EQ(eigen_q_b.coeffs().x(), 1);
  EXPECT_S_EQ(eigen_q_b.coeffs().y(), 2);
  EXPECT_S_EQ(eigen_q_b.coeffs().z(), 3);
  EXPECT_S_EQ(eigen_q_b.coeffs().w(), 4);

  EXPECT_S_EQ(a.coeffs().x(), eigen_q_a.coeffs().x());
  EXPECT_S_EQ(a.coeffs().y(), eigen_q_a.coeffs().y());
  EXPECT_S_EQ(a.coeffs().z(), eigen_q_a.coeffs().z());
  EXPECT_S_EQ(a.coeffs().w(), eigen_q_a.coeffs().w());
}

//==============================================================================
TYPED_TEST(SO3Test, EulerAngles)
{
  using Scalar = typename TestFixture::Type;
  using SO3 = SO3<Scalar>;

  SO3 a;
  SO3 b;

  const Eigen::Matrix<Scalar, 3, 1> angles
      = Eigen::Matrix<Scalar, 3, 1>::Random();

  a.set_from_euler_xyz(angles[0], angles[1], angles[2]);
  EXPECT_EQ(
      a, SO3::RotX(angles[0]) * SO3::RotY(angles[1]) * SO3::RotZ(angles[2]));
  EXPECT_EQ(a, SO3().set_from_euler_xyz(a.euler_angles()));

  a.set_from_rpy(angles);
  b.set_from_rpy(a.rpy());
  EXPECT_EQ(a, b);
}

//==============================================================================
TYPED_TEST(SO3Test, Tangent)
{
  using Scalar = typename TestFixture::Type;

  auto a = SO3Tangent<Scalar>();
  (void)a;
}

//==============================================================================
TYPED_TEST(SO3Test, Algebra)
{
  using Scalar = typename TestFixture::Type;

  auto a = SO3Tangent<Scalar>::Random();
  auto a_hat = a.hat();
  auto b = a_hat.vee();
  DART_UNUSED(a_hat, b);
}

//==============================================================================
TYPED_TEST(SO3Test, Benchmark)
{
  using Scalar = typename TestFixture::Type;

  const long num = 1e+4;

  Eigen::Quaternion<Scalar> q1 = Eigen::Quaternion<Scalar>::UnitRandom();
  Eigen::Quaternion<Scalar> q2 = Eigen::Quaternion<Scalar>::UnitRandom();
  Eigen::Quaternion<Scalar> q3 = Eigen::Quaternion<Scalar>::UnitRandom();

  Eigen::Matrix<Scalar, 3, 1> p1 = Eigen::Matrix<Scalar, 3, 1>::Random();
  Eigen::Matrix<Scalar, 3, 1> p2 = Eigen::Matrix<Scalar, 3, 1>::Random();
  Eigen::Matrix<Scalar, 3, 1> p3 = Eigen::Matrix<Scalar, 3, 1>::Random();

  SO3A<Scalar> so3a_1(q1);
  SO3A<Scalar> so3a_2(q2);
  SO3A<Scalar> so3a_3(q3);
  SO3B<Scalar> so3b_1(q1);
  SO3B<Scalar> so3b_2(q2);
  SO3B<Scalar> so3b_3(q3);

  SE3A<Scalar> se3a_1(q1, p1);
  SE3A<Scalar> se3a_2(q2, p2);
  SE3A<Scalar> se3a_3(q3, p3);
  SE3B<Scalar> se3b_1(q1, p1);
  SE3B<Scalar> se3b_2(q2, p2);
  SE3B<Scalar> se3b_3(q3, p3);
  SE3C<Scalar> se3c_1(q1, p1);
  SE3C<Scalar> se3c_2(q2, p2);
  SE3C<Scalar> se3c_3(q3, p3);

  {
    common::tic();
    for (auto i = 0; i < num; ++i) {
      so3a_2 = so3a_1 * so3a_2;
      so3a_3 = so3a_3 * so3a_2 * so3a_1;
    }
    std::cout << "[DEBUG] A: " << so3a_3.quat().coeffs().transpose()
              << std::endl;
    common::toc_us(true);
  }

  {
    common::tic();
    for (auto i = 0; i < num; ++i) {
      so3b_2 = so3b_1 * so3b_2;
      so3b_3 = so3b_3 * so3b_2 * so3b_1;
    }
    std::cout << "[DEBUG] B: " << so3b_3.m_data.coeffs().transpose()
              << std::endl;
    common::toc_us(true);
  }

  {
    common::tic();
    for (auto i = 0; i < num; ++i) {
      se3a_2 = se3a_1 * se3a_2;
      se3a_3 = se3a_3 * se3a_2 * se3a_1;
    }
    std::cout << "[DEBUG] A: " << se3a_3.m_data.template head<4>().transpose()
              << std::endl;
    common::toc_us(true);
  }

  {
    common::tic();
    for (auto i = 0; i < num; ++i) {
      se3b_2 = se3b_1 * se3b_2;
      se3b_3 = se3b_3 * se3b_2 * se3b_1;
    }
    std::cout << "[DEBUG] B: " << se3b_3.m_orientation.coeffs().transpose()
              << std::endl;
    common::toc_us(true);
  }

  {
    common::tic();
    for (auto i = 0; i < num; ++i) {
      se3c_2 = se3c_1 * se3c_2;
      se3c_3 = se3c_3 * se3c_2 * se3c_1;
    }
    std::cout
        << "[DEBUG] C: "
        << Eigen::Quaternion<Scalar>(se3c_3.m_tf.linear()).coeffs().transpose()
        << std::endl;
    common::toc_us(true);
  }
}
