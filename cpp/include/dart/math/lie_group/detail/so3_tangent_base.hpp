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

#pragma once

#include "dart/math/lie_group/detail/tangent_base.hpp"
#include "dart/math/linear_algebra.hpp"

namespace dart::math {

//==============================================================================
template <typename Derived>
class SO3TangentBase : public TangentBase<Derived>
{
public:
  using This = SO3TangentBase<Derived>;
  using Base = TangentBase<Derived>;

  DART_LIE_GROUP_USE_BASE_TYPES;

  using Quaternion = math::Quaternion<Scalar, Options>;
  using QuaternionMap = Eigen::Map<Quaternion>;
  using ConstQuaternionMap = Eigen::Map<const Quaternion>;

  LieGroup exp(
      Jacobian* jacobian = nullptr, Scalar tolerance = eps<Scalar>()) const;

  template <typename OtherTangent>
  Tangent ad(const SO3TangentBase<OtherTangent>& other) const;

  Jacobian left_jacobian(Scalar tolerance = eps<Scalar>()) const;

  Jacobian right_jacobian(Scalar tolerance = eps<Scalar>()) const;

  Jacobian space_jacobian(Scalar tolerance = eps<Scalar>()) const;

  Jacobian body_jacobian(Scalar tolerance = eps<Scalar>()) const;

  Jacobian left_jacobian_inverse(Scalar tolerance = eps<Scalar>()) const;

  Jacobian right_jacobian_inverse(Scalar tolerance = eps<Scalar>()) const;

  template <typename MatrixDerived>
  Jacobian left_jacobian_time_derivative(
      const Eigen::MatrixBase<MatrixDerived>& dq,
      Scalar tolerance = eps<Scalar>()) const;

  Jacobian left_jacobian_time_derivative(
      int index, Scalar tolerance = eps<Scalar>()) const;

  std::array<Jacobian, 3> left_jacobian_time_derivative(
      Scalar tolerance = eps<Scalar>()) const;

  template <typename MatrixDerived>
  Jacobian right_jacobian_time_derivative(
      const Eigen::MatrixBase<MatrixDerived>& dq,
      Scalar tolerance = eps<Scalar>()) const;

  Jacobian right_jacobian_time_derivative(
      int index, Scalar tolerance = eps<Scalar>()) const;

  std::array<Jacobian, 3> right_jacobian_time_derivative(
      Scalar tolerance = eps<Scalar>()) const;

  Scalar x() const;

  Scalar y() const;

  Scalar z() const;

  Scalar& x();

  Scalar& y();

  Scalar& z();

  SO3Algebra<Scalar> hat() const
  {
    return SO3Algebra<Scalar>(skew(coeffs()));
  }

  using Base::coeffs;
};

//==============================================================================
template <typename Derived>
typename SO3TangentBase<Derived>::LieGroup SO3TangentBase<Derived>::exp(
    Jacobian* jacobian, Scalar tolerance) const
{
  const Scalar theta = coeffs().norm();
  DataType coeffs;
  if (theta < tolerance) {
    const DataType vec = 0.5 * coeffs();
    coeffs << 1, vec;
  } else {
    const DataType vec = std::sin(0.5 * theta) * (coeffs() / theta);
    coeffs << std::cos(0.5 * theta), vec;
  }

  const LieGroup out(std::move(coeffs));

  if (jacobian) {
    (*jacobian) = right_jacobian();
  }

  return out;
}

//==============================================================================
template <typename Derived>
template <typename OtherTangent>
typename SO3TangentBase<Derived>::Tangent SO3TangentBase<Derived>::ad(
    const SO3TangentBase<OtherTangent>& other) const
{
  return Tangent(coeffs().cross(other.coeffs()));
}

//==============================================================================
template <typename Derived>
typename SO3TangentBase<Derived>::Jacobian
SO3TangentBase<Derived>::left_jacobian(Scalar tolerance) const
{
  Jacobian jac;

  const Scalar t = coeffs().norm();
  const Eigen::Matrix<Scalar, 3, 3> A = skew(coeffs());
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
template <typename Derived>
typename SO3TangentBase<Derived>::Jacobian
SO3TangentBase<Derived>::right_jacobian(Scalar tolerance) const
{
  return SO3Tangent<Scalar, Options>(-coeffs()).left_jacobian(tolerance);
}

//==============================================================================
template <typename Derived>
typename SO3TangentBase<Derived>::Jacobian
SO3TangentBase<Derived>::space_jacobian(Scalar tolerance) const
{
  return left_jacobian(tolerance);
}

//==============================================================================
template <typename Derived>
typename SO3TangentBase<Derived>::Jacobian
SO3TangentBase<Derived>::body_jacobian(Scalar tolerance) const
{
  return right_jacobian(tolerance);
}

//==============================================================================
template <typename Derived>
typename SO3TangentBase<Derived>::Jacobian
SO3TangentBase<Derived>::left_jacobian_inverse(Scalar tolerance) const
{
  Jacobian jac;

  const Scalar theta = coeffs().norm();
  const Eigen::Matrix<Scalar, 3, 3> A = skew(coeffs());
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
template <typename Derived>
typename SO3TangentBase<Derived>::Jacobian
SO3TangentBase<Derived>::right_jacobian_inverse(Scalar tolerance) const
{
  return SO3Tangent<Scalar, Options>(-coeffs()).left_jacobian_inverse(
      tolerance);
}

//==============================================================================
template <typename Derived>
template <typename MatrixDerived>
typename SO3TangentBase<Derived>::Jacobian
SO3TangentBase<Derived>::left_jacobian_time_derivative(
    const Eigen::MatrixBase<MatrixDerived>& dq, Scalar tolerance) const
{
  Jacobian jac;

  const Scalar t = coeffs().norm();
  const Eigen::Matrix<Scalar, 3, 3> A = skew(coeffs());
  const Eigen::Matrix<Scalar, 3, 3> dA = skew(dq);
  const Scalar q_dot_dq = coeffs().dot(dq);
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
template <typename Derived>
typename SO3TangentBase<Derived>::Jacobian
SO3TangentBase<Derived>::left_jacobian_time_derivative(
    int index, Scalar tolerance) const
{
  math::Vector3<Scalar> dq = math::Vector3<Scalar>::Zero();
  dq[index] = 1;
  return left_jacobian_time_derivative(dq, tolerance);
}

//==============================================================================
template <typename Derived>
std::array<typename SO3TangentBase<Derived>::Jacobian, 3>
SO3TangentBase<Derived>::left_jacobian_time_derivative(Scalar tolerance) const
{
  return {
      left_jacobian_time_derivative(0, tolerance),
      left_jacobian_time_derivative(1, tolerance),
      left_jacobian_time_derivative(2, tolerance)};
}

//==============================================================================
template <typename Derived>
template <typename MatrixDerived>
typename SO3TangentBase<Derived>::Jacobian
SO3TangentBase<Derived>::right_jacobian_time_derivative(
    const Eigen::MatrixBase<MatrixDerived>& dq, Scalar tolerance) const
{
  return left_jacobian_time_derivative(dq, tolerance).transpose();
}

//==============================================================================
template <typename Derived>
typename SO3TangentBase<Derived>::Jacobian
SO3TangentBase<Derived>::right_jacobian_time_derivative(
    int index, Scalar tolerance) const
{
  return left_jacobian_time_derivative(index, tolerance).transpose();
}

//==============================================================================
template <typename Derived>
std::array<typename SO3TangentBase<Derived>::Jacobian, 3>
SO3TangentBase<Derived>::right_jacobian_time_derivative(Scalar tolerance) const
{
  return {
      right_jacobian_time_derivative(0, tolerance),
      right_jacobian_time_derivative(1, tolerance),
      right_jacobian_time_derivative(2, tolerance)};
}

//==============================================================================
template <typename Derived>
typename SO3TangentBase<Derived>::Scalar SO3TangentBase<Derived>::x() const
{
  return coeffs().x();
}

//==============================================================================
template <typename Derived>
typename SO3TangentBase<Derived>::Scalar SO3TangentBase<Derived>::y() const
{
  return coeffs().y();
}

//==============================================================================
template <typename Derived>
typename SO3TangentBase<Derived>::Scalar SO3TangentBase<Derived>::z() const
{
  return coeffs().z();
}

//==============================================================================
template <typename Derived>
typename SO3TangentBase<Derived>::Scalar& SO3TangentBase<Derived>::x()
{
  return coeffs().x();
}

//==============================================================================
template <typename Derived>
typename SO3TangentBase<Derived>::Scalar& SO3TangentBase<Derived>::y()
{
  return coeffs().y();
}

//==============================================================================
template <typename Derived>
typename SO3TangentBase<Derived>::Scalar& SO3TangentBase<Derived>::z()
{
  return coeffs().z();
}

} // namespace dart::math
