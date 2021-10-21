/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#pragma once

#include "dart/math/random.hpp"
#include "dart/math/so3.hpp"
// #include "dart/math/linear_algebra.hpp"

namespace dart::math {

//==============================================================================
template <typename Scalar, int Options>
SO3<Scalar, Options> SO3<Scalar, Options>::Identity()
{
  SO3<Scalar, Options> out;
  out.set_identity();
  return out;
}

//==============================================================================
template <typename Scalar, int Options>
SO3<Scalar, Options> SO3<Scalar, Options>::Random()
{
  SO3<Scalar, Options> out;
  out.set_random();
  return out;
}

//==============================================================================
template <typename Scalar, int Options>
SO3<Scalar, Options>::SO3() : m_data(Quaternion::Identity())
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
template <typename OtherDerived>
SO3<Scalar, Options>::SO3(const SO3Base<OtherDerived>& other)
  : m_data(other.quaternion())
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
template <typename OtherDerived>
SO3<Scalar, Options>::SO3(SO3Base<OtherDerived>&& other)
  : m_data(std::move(other.quaternion()))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
SO3<Scalar, Options>::SO3(const QuaternionType& quat) : m_data(quat)
{
  normalize();
}

//==============================================================================
template <typename Scalar, int Options>
SO3<Scalar, Options>::SO3(QuaternionType&& quat) : m_data(std::move(quat))
{
  normalize();
}

//==============================================================================
template <typename Scalar, int Options>
SO3<Scalar, Options>& SO3<Scalar, Options>::operator=(
    const SO3<Scalar, Options>& other)
{
  m_data = other.m_data;
  return *this;
}

//==============================================================================
template <typename Scalar, int Options>
SO3<Scalar, Options>& SO3<Scalar, Options>::operator=(
    SO3<Scalar, Options>&& other)
{
  m_data = std::move(other.m_data);
  return *this;
}

//==============================================================================
template <typename Scalar, int Options>
template <typename Derived>
SO3<Scalar, Options>& SO3<Scalar, Options>::operator=(
    const Eigen::MatrixBase<Derived>& matrix)
{
  m_data = matrix; // assign matrix to Eigen quaternion
  normalize();
  return *this;
}

//==============================================================================
template <typename Scalar, int Options>
SO3<Scalar, Options> SO3<Scalar, Options>::operator*(
    const SO3<Scalar, Options>& other) const
{
  return SO3<Scalar, Options>(m_data * other.m_data);
}

//==============================================================================
template <typename Scalar, int Options>
R3<Scalar> SO3<Scalar, Options>::operator*(const R3<Scalar>& other) const
{
  return R3<Scalar>(m_data._transformVector(other.vector()));
}

//==============================================================================
template <typename Scalar, int Options>
SO3Algebra<Scalar, Options> SO3<Scalar, Options>::operator*(
    const SO3Algebra<Scalar, Options>& dx) const
{
  return SO3Algebra<Scalar, Options>(matrix() * dx.matrix());
}

//==============================================================================
template <typename Scalar, int Options>
Eigen::Matrix<Scalar, 3, 1> SO3<Scalar, Options>::euler_angles(
    int axis1, int axis2, int axis3) const
{
  return euler_angles_intrinsic(axis1, axis2, axis3);
}

//==============================================================================
template <typename Scalar, int Options>
Eigen::Matrix<Scalar, 3, 1> SO3<Scalar, Options>::euler_angles_intrinsic(
    int axis1, int axis2, int axis3) const
{
  return m_data.toRotationMatrix().eulerAngles(axis1, axis2, axis3);
}

//==============================================================================
template <typename Scalar, int Options>
Eigen::Matrix<Scalar, 3, 1> SO3<Scalar, Options>::euler_angles_extrinsic(
    int axis1, int axis2, int axis3) const
{
  return m_data.toRotationMatrix().eulerAngles(axis3, axis2, axis1).reverse();
}

//==============================================================================
template <typename Scalar, int Options>
Eigen::Matrix<Scalar, 3, 1> SO3<Scalar, Options>::rpy() const
{
  return euler_angles_extrinsic(0, 1, 2);
}

//==============================================================================
template <typename Scalar, int Options>
void SO3<Scalar, Options>::set_identity()
{
  m_data.setIdentity();
}

//==============================================================================
template <typename Scalar, int Options>
void SO3<Scalar, Options>::set_random()
{
  m_data = Random::uniformUnitQuaternion<Scalar>();
  normalize();
}

//==============================================================================
template <typename Scalar, int Options>
SO3<Scalar, Options> SO3<Scalar, Options>::inverse() const
{
  return SO3<Scalar, Options>(m_data.inverse());
}

//==============================================================================
template <typename Scalar, int Options>
SO3<Scalar, Options>& SO3<Scalar, Options>::inverse_in_place()
{
  m_data = m_data.inverse();
  return *this;
}

//==============================================================================
template <typename Scalar, int Options>
typename SO3<Scalar, Options>::Tangent SO3<Scalar, Options>::log(
    Jacobian* jacobian, Scalar tolerance) const
{
  const Scalar theta = 2 * std::acos(m_data.w());
  DART_ASSERT(!std::isnan(theta));

  typename Tangent::TangentData vec;
  if (theta < tolerance) {
    vec.noalias() = 2 * m_data.vec();
  } else {
    DART_ASSERT(std::sin(0.5 * theta));
    vec.noalias() = theta * m_data.vec() / std::sin(0.5 * theta);
  }

  const auto out = Tangent(vec);

  if (jacobian) {
    (*jacobian) = out.left_jacobian(tolerance);
  }

  return out;
}

//==============================================================================
template <typename Scalar, int Options>
typename SO3<Scalar, Options>::Tangent SO3<Scalar, Options>::ad(
    const Tangent& V) const
{
  return Tangent(rotation() * V.vector());
}

//==============================================================================
template <typename Scalar, int Options>
typename SO3<Scalar, Options>::Jacobian SO3<Scalar, Options>::ad_matrix() const
{
  return rotation();
}

//==============================================================================
template <typename Scalar, int Options>
typename SO3<Scalar, Options>::Matrix SO3<Scalar, Options>::matrix() const
{
  return m_data.toRotationMatrix();
}

//==============================================================================
template <typename Scalar, int Options>
typename SO3<Scalar, Options>::Rotation SO3<Scalar, Options>::rotation() const
{
  return matrix();
}

//==============================================================================
template <typename Scalar, int Options>
typename SO3<Scalar, Options>::Translation SO3<Scalar, Options>::translation()
    const
{
  return Translation::Zero();
}

//==============================================================================
template <typename Scalar, int Options>
typename SO3<Scalar, Options>::Transformation
SO3<Scalar, Options>::transformation() const
{
  Transformation out = Transformation::Identity();
  out.linear() = rotation();
  return out;
}

//==============================================================================
template <typename Scalar, int Options>
const typename SO3<Scalar, Options>::Quaternion&
SO3<Scalar, Options>::quaternion() const
{
  return m_data;
}

//==============================================================================
template <typename Scalar, int Options>
void SO3<Scalar, Options>::normalize()
{
  if (m_data.w() < 0) {
    m_data.coeffs() *= -1;
  }
  m_data.normalize();
}

//==============================================================================
template <typename Scalar, int Options>
SO3Algebra<Scalar, Options>::SO3Algebra() : m_matrix(LieAlgebraData::Zero())
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
SO3Algebra<Scalar, Options>::SO3Algebra(const SO3Algebra& other)
  : m_matrix(other.m_matrix)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
SO3Algebra<Scalar, Options>::SO3Algebra(SO3Algebra&& other)
  : m_matrix(std::move(other.m_matrix))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
template <typename Derived>
SO3Algebra<Scalar, Options>::SO3Algebra(
    const Eigen::MatrixBase<Derived>& matrix)
  : m_matrix(matrix)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
template <typename Derived>
SO3Algebra<Scalar, Options>::SO3Algebra(Eigen::MatrixBase<Derived>&& matrix)
  : m_matrix(std::move(matrix))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
SO3Algebra<Scalar, Options> SO3Algebra<Scalar, Options>::operator/(
    Scalar scalar) const
{
  return SO3Algebra<Scalar, Options>(m_matrix / scalar);
}

//==============================================================================
template <typename Scalar, int Options>
SO3Tangent<Scalar, Options> SO3Algebra<Scalar, Options>::vee() const
{
  return SO3Tangent<Scalar, Options>(Eigen::Matrix<Scalar, 3, 1>(
      m_matrix(2, 1), m_matrix(0, 2), m_matrix(1, 0)));
}

//==============================================================================
template <typename Scalar, int Options>
typename SO3Algebra<Scalar, Options>::LieAlgebraData&
SO3Algebra<Scalar, Options>::mutable_matrix()
{
  return m_matrix;
}

//==============================================================================
template <typename Scalar, int Options>
const typename SO3Algebra<Scalar, Options>::LieAlgebraData&
SO3Algebra<Scalar, Options>::matrix() const
{
  return m_matrix;
}

//==============================================================================
template <typename Scalar, int Options>
SO3Tangent<Scalar, Options>::SO3Tangent() : m_data(TangentData::Zero())
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
template <typename Derived>
SO3Tangent<Scalar, Options>::SO3Tangent(const SO3TangentBase<Derived>& other)
  : m_data(other.vector())
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
template <typename Derived>
SO3Tangent<Scalar, Options>::SO3Tangent(SO3TangentBase<Derived>&& other)
  : m_data(std::move(other.vector()))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
template <typename Derived>
SO3Tangent<Scalar, Options>::SO3Tangent(
    const Eigen::MatrixBase<Derived>& coeffs)
  : m_data(coeffs)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
template <typename Derived>
SO3Tangent<Scalar, Options>::SO3Tangent(Eigen::MatrixBase<Derived>&& coeffs)
  : m_data(std::move(coeffs))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
template <typename Derived>
SO3Tangent<Scalar, Options>& SO3Tangent<Scalar, Options>::operator=(
    const SO3TangentBase<Derived>& other)
{
  m_data = other.vector();
  return *this;
}

//==============================================================================
template <typename Scalar, int Options>
template <typename Derived>
SO3Tangent<Scalar, Options>& SO3Tangent<Scalar, Options>::operator=(
    SO3TangentBase<Derived>&& other)
{
  m_data = std::move(other.vector());
  return *this;
}

//==============================================================================
template <typename Scalar, int Options>
Scalar SO3Tangent<Scalar, Options>::operator[](int index) const
{
  return m_data[index];
}

//==============================================================================
template <typename Scalar, int Options>
Scalar& SO3Tangent<Scalar, Options>::operator[](int index)
{
  return m_data[index];
}

//==============================================================================
template <typename Scalar, int Options>
SO3Tangent<Scalar, Options> SO3Tangent<Scalar, Options>::operator-() const
{
  return SO3Tangent<Scalar, Options>(-m_data);
}

//==============================================================================
template <typename Scalar, int Options>
SO3Tangent<Scalar, Options> SO3Tangent<Scalar, Options>::operator+(
    const SO3Tangent& other) const
{
  return SO3Tangent(m_data + other.m_data);
}

//==============================================================================
template <typename Scalar, int Options>
Scalar SO3Tangent<Scalar, Options>::operator*(
    const SO3Cotangent<Scalar, Options>& torque) const
{
  return m_data.dot(torque.vector());
}

//==============================================================================
template <typename Scalar, int Options>
void SO3Tangent<Scalar, Options>::set_zero()
{
  m_data.setZero();
}

//==============================================================================
template <typename Scalar, int Options>
void SO3Tangent<Scalar, Options>::set_random()
{
  const SO3<Scalar, Options> R = SO3<Scalar, Options>::Random();
  *this = R.log();
}

//==============================================================================
template <typename Scalar, int Options>
SO3Algebra<Scalar, Options> SO3Tangent<Scalar, Options>::hat() const
{
  return SO3Algebra<Scalar, Options>(skew(vector()));
}

//==============================================================================
template <typename Scalar, int Options>
typename SO3Tangent<Scalar, Options>::LieGroup SO3Tangent<Scalar, Options>::exp(
    Jacobian* jacobian, Scalar tolerance) const
{
  const Scalar theta = m_data.norm();
  typename LieGroup::LieGroupData quat;
  if (theta < tolerance) {
    const TangentData vec = 0.5 * m_data;
    quat = QuaternionType(1, vec[0], vec[1], vec[2]);
  } else {
    const TangentData vec = std::sin(0.5 * theta) * (m_data / theta);
    quat = QuaternionType(std::cos(0.5 * theta), vec[0], vec[1], vec[2]);
  }

  LieGroup out(quat);

  if (jacobian) {
    (*jacobian) = right_jacobian();
  }

  return out;
}

//==============================================================================
template <typename Scalar, int Options>
typename SO3Tangent<Scalar, Options>::Jacobian
SO3Tangent<Scalar, Options>::left_jacobian(Scalar tolerance) const
{
  Jacobian jac;

  const Scalar t = m_data.norm();
  const auto A = hat().matrix();
  if (t < tolerance) {
    jac.noalias() = Jacobian::Identity() + 0.5 * A;
  } else {
    const Scalar t2 = t * t;
    const Scalar t3 = t2 * t;
    const Scalar st = std::sin(t);
    const Scalar ct = std::cos(t);
    // clang-format off
    jac.noalias() = Jacobian::Identity()
        + ((1 - ct) / t2) * A
        + ((t - st) / t3) * A * A;
    // clang-format on
  }

  return jac;
}

//==============================================================================
template <typename Scalar, int Options>
typename SO3Tangent<Scalar, Options>::Jacobian
SO3Tangent<Scalar, Options>::space_jacobian(Scalar tolerance) const
{
  return left_jacobian(tolerance);
}

//==============================================================================
template <typename Scalar, int Options>
typename SO3Tangent<Scalar, Options>::Jacobian
SO3Tangent<Scalar, Options>::right_jacobian(Scalar tolerance) const
{
  return SO3Tangent<Scalar, Options>(-m_data).left_jacobian(tolerance);
}

//==============================================================================
template <typename Scalar, int Options>
typename SO3Tangent<Scalar, Options>::Jacobian
SO3Tangent<Scalar, Options>::body_jacobian(Scalar tolerance) const
{
  return right_jacobian(tolerance);
}

//==============================================================================
template <typename Scalar, int Options>
typename SO3Tangent<Scalar, Options>::Jacobian
SO3Tangent<Scalar, Options>::left_jacobian_inverse(Scalar tolerance) const
{
  Jacobian jac;

  const Scalar theta = m_data.norm();
  const auto A = hat().matrix();
  if (theta < tolerance) {
    jac.noalias() = Jacobian::Identity() + 0.5 * A;
  } else {
    const Scalar theta2 = theta * theta;
    const Scalar sin_theta = std::sin(theta);
    const Scalar cos_theta = std::cos(theta);
    // clang-format off
    jac.noalias() = Jacobian::Identity()
        - 0.5 * A
        + (1 / theta2 - (1 + cos_theta) / (2 * theta * sin_theta)) * A * A;
    // clang-format on
  }

  return jac;
}

//==============================================================================
template <typename Scalar, int Options>
typename SO3Tangent<Scalar, Options>::Jacobian
SO3Tangent<Scalar, Options>::right_jacobian_inverse(Scalar tolerance) const
{
  return SO3Tangent<Scalar, Options>(-m_data).left_jacobian_inverse(tolerance);
}

//==============================================================================
template <typename Scalar, int Options>
template <typename Derived>
typename SO3Tangent<Scalar, Options>::Jacobian
SO3Tangent<Scalar, Options>::left_jacobian_time_derivative(
    const math::MatrixBase<Derived>& dq, Scalar tolerance) const
{
  Jacobian jac;

  const Scalar t = m_data.norm();
  const LieAlgebra A = hat();
  const LieAlgebra dA = skew(dq);
  const Scalar q_dot_dq = m_data.dot(dq);
  const Scalar sin_t = std::sin(t);
  const Scalar cos_t = std::cos(t);

  if (t < tolerance) {
    // clang-format off
    jac = -0.5 * dA
      + (Scalar(1) / Scalar(6)) * (A * dA + dA * A)
      + (Scalar(1) / Scalar(12)) * q_dot_dq * A;
    // clang-format on
  } else {
    const Scalar t2 = t * t;
    const Scalar t3 = t2 * t;
    const Scalar t4 = t3 * t;
    const Scalar t5 = t4 * t;
    // clang-format off
    jac = -((1 - cos_t) / t2) * dA
        + ((t - sin_t) / t3) * (A * dA + dA * A)
        - ((t * sin_t + 2 * cos_t - 2) / t4) * q_dot_dq * A
        + ((3 * sin_t - t * cos_t - 2 * t) / t5) * q_dot_dq * A * A;
    // clang-format on
  }

  return jac;
}

//==============================================================================
template <typename Scalar, int Options>
template <typename Derived>
typename SO3Tangent<Scalar, Options>::Jacobian
SO3Tangent<Scalar, Options>::right_jacobian_time_derivative(
    const math::MatrixBase<Derived>& dq, Scalar tolerance) const
{
  return left_jacobian_time_derivative(dq, tolerance).transpose();
}

//==============================================================================
template <typename Scalar, int Options>
typename SO3Tangent<Scalar, Options>::Jacobian
SO3Tangent<Scalar, Options>::right_jacobian_time_derivative(
    int index, Scalar tolerance) const
{
  return left_jacobian_time_derivative(index, tolerance).transpose();
}

//==============================================================================
template <typename Scalar, int Options>
std::array<typename SO3Tangent<Scalar, Options>::Jacobian, 3>
SO3Tangent<Scalar, Options>::right_jacobian_time_derivative(
    Scalar tolerance) const
{
  std::array<typename SO3Tangent<Scalar, Options>::Jacobian, 3> out;
  for (auto i = 0; i < 3; ++i) {
    out[i] = right_jacobian_time_derivative(i, tolerance).transpose();
  }
  return out;
}

//==============================================================================
template <typename Scalar, int Options>
typename SO3Tangent<Scalar, Options>::Jacobian
SO3Tangent<Scalar, Options>::left_jacobian_time_derivative(
    int index, Scalar tolerance) const
{
  math::Vector3<Scalar> dq = math::Vector3<Scalar>::Zero();
  dq[index] = 1;
  return left_jacobian_time_derivative(dq, tolerance);
}

//==============================================================================
template <typename Scalar, int Options>
std::array<typename SO3Tangent<Scalar, Options>::Jacobian, 3>
SO3Tangent<Scalar, Options>::left_jacobian_time_derivative(
    Scalar tolerance) const
{
  std::array<typename SO3Tangent<Scalar, Options>::Jacobian, 3> out;

  math::Vector3<Scalar> dq = math::Vector3<Scalar>::Zero();
  for (auto i = 0; i < 3; ++i) {
    dq.setZero();
    dq[i] = 1;
    out[i] = left_jacobian_time_derivative(dq, tolerance);
  }

  return out;
}

//==============================================================================
template <typename Scalar, int Options>
SO3Tangent<Scalar, Options> SO3Tangent<Scalar, Options>::ad(
    const SO3Tangent& other) const
{
  return SO3Tangent<Scalar, Options>(m_data.cross(other.m_data));
}

//==============================================================================
template <typename Scalar, int Options>
typename SO3Tangent<Scalar, Options>::Jacobian
SO3Tangent<Scalar, Options>::ad_matrix() const
{
  return hat();
}

//==============================================================================
template <typename Scalar, int Options>
typename SO3Tangent<Scalar, Options>::TangentData&
SO3Tangent<Scalar, Options>::vector()
{
  return m_data;
}

//==============================================================================
template <typename Scalar, int Options>
const typename SO3Tangent<Scalar, Options>::TangentData&
SO3Tangent<Scalar, Options>::vector() const
{
  return m_data;
}

//==============================================================================
template <typename Scalar, int Options>
SO3Tangent<Scalar, Options> SO3Tangent<Scalar, Options>::Random()
{
  SO3Tangent<Scalar, Options> out;
  out.set_random();
  return out;
}

//==============================================================================
template <typename Scalar, int Options>
typename SO3Cotangent<Scalar, Options>::CotangentData&
SO3Cotangent<Scalar, Options>::vector()
{
  return m_data;
}

//==============================================================================
template <typename Scalar, int Options>
const typename SO3Cotangent<Scalar, Options>::CotangentData&
SO3Cotangent<Scalar, Options>::vector() const
{
  return m_data;
}

} // namespace dart::math

namespace Eigen {

//==============================================================================
template <typename Scalar, int Options>
Map<dart::math::SO3<Scalar, Options>, Options>::Map(Scalar* data) : m_data(data)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
const Map<Eigen::Quaternion<Scalar, Options>, Options>&
Map<dart::math::SO3<Scalar, Options>, Options>::quaternion() const
{
  return m_data;
}

//==============================================================================
template <typename Scalar, int Options>
Map<Eigen::Quaternion<Scalar, Options>, Options>&
Map<dart::math::SO3<Scalar, Options>, Options>::quaternion()
{
  return m_data;
}

//==============================================================================
template <typename Scalar, int Options>
Map<const dart::math::SO3<Scalar, Options>, Options>::Map(const Scalar* data)
  : m_data(data)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
const Map<Eigen::Quaternion<Scalar, Options>, Options>&
Map<const dart::math::SO3<Scalar, Options>, Options>::quaternion() const
{
  return m_data;
}

//==============================================================================
template <typename Scalar, int Options>
Map<dart::math::SO3Tangent<Scalar, Options>, Options>::Map(Scalar* data)
  : m_data(data)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
Map<const dart::math::SO3Tangent<Scalar, Options>, Options>::Map(
    const Scalar* data)
  : m_data(data)
{
  // Do nothing
}

} // namespace Eigen
