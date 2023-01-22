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

#pragma once

#include <dart/math/lie_group/LieGroupBase.hpp>

namespace dart::math {

/// @brief Base class for SO3
/// @tparam Derived The derived class
template <typename Derived>
class SO3Base : public LieGroupBase<Derived>
{
public:
  using Base = LieGroupBase<Derived>;

  // LieGroupBase types
  using Scalar = typename Base::Scalar;
  using Coeffs = typename Base::Coeffs;
  using PlainObject = typename Base::PlainObject;
  using MatrixType = typename Base::MatrixType;

  // SO3Base specific types
  using QuaternionType = ::Eigen::Map<Quaternion<Scalar>>;
  using ConstQuaternionType = ::Eigen::Map<const Quaternion<Scalar>>;

  using Base::derived;

  using Base::Tolerance;

  /// Assignment operator.
  ///
  /// @param[in] other The other SO3 to assign.
  /// @return Reference to this SO3.
  template <typename OtherDerived>
  SO3Base<Derived>& operator=(const SO3Base<OtherDerived>& other);

  /// Composes this SO3 with other and returns the result.
  ///
  /// @param[in] other The other SO3 to compose with.
  /// @return The composition of this SO3 with other.
  template <typename OtherDerived>
  [[nodiscard]] PlainObject operator*(const SO3Base<OtherDerived>& other) const;

  /// Normalizes this SO3 so that its quaternion representation is always unit
  /// and unique by keeping the real part of the quaternion positive.
  void normalize();

  /// Returns the inverse of this SO3.
  [[nodiscard]] PlainObject inverse() const;

  /// Returns the logarithm of this SO3.
  ///
  /// @param[in] tol Tolerance for checking if the rotation is near identity.
  /// @return The logarithm of this SO3.
  [[nodiscard]] Vector3<Scalar> log(Scalar tol = Tolerance()) const;

  /// Returns the logarithm of this SO3.
  ///
  /// @param[out] jacobian The Jacobian of the logarithm.
  /// @param[in] tol Tolerance for checking if the rotation is near identity.
  /// @return The logarithm of this SO3.
  template <typename MatrixDerived>
  [[nodiscard]] Vector3<Scalar> log(
      Eigen::MatrixBase<MatrixDerived>* jacobian,
      Scalar tol = Tolerance()) const;

  /// Returns the quaternion representation of this SO3.
  [[nodiscard]] const ConstQuaternionType quaternion() const;

  /// Returns the quaternion representation of this SO3.
  [[nodiscard]] QuaternionType quaternion();

  using Base::coeffs;
};

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

namespace dart::math {

//==============================================================================
template <typename Derived>
template <typename OtherDerived>
SO3Base<Derived>& SO3Base<Derived>::operator=(
    const SO3Base<OtherDerived>& other)
{
  quaternion() = other.quaternion();
  return *this;
}

//==============================================================================
template <typename Derived>
template <typename OtherDerived>
typename SO3Base<Derived>::PlainObject SO3Base<Derived>::operator*(
    const SO3Base<OtherDerived>& other) const
{
  return PlainObject(quaternion() * other.quaternion());
}

//==============================================================================
template <typename Derived>
void SO3Base<Derived>::normalize()
{
  if (coeffs().w() < 0) {
    coeffs() *= -1;
  }
  coeffs().normalize();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::PlainObject SO3Base<Derived>::inverse() const
{
  return PlainObject(quaternion().conjugate());
}

//==============================================================================
template <typename Derived>
Vector3<typename SO3Base<Derived>::Scalar> SO3Base<Derived>::log(
    Scalar tol) const
{
  return SO3<Scalar>::Log(derived(), tol);
}

//==============================================================================
template <typename Derived>
template <typename MatrixDerived>
Vector3<typename SO3Base<Derived>::Scalar> SO3Base<Derived>::log(
    Eigen::MatrixBase<MatrixDerived>* jacobian, Scalar tol) const
{
  return SO3<Scalar>::Log(derived(), jacobian, tol);
}

//==============================================================================
template <typename Derived>
const typename SO3Base<Derived>::ConstQuaternionType
SO3Base<Derived>::quaternion() const
{
  return derived().quaternion();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::QuaternionType SO3Base<Derived>::quaternion()
{
  return derived().quaternion();
}

} // namespace dart::math
