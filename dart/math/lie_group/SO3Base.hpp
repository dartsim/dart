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
  using Base::CoeffsDim;
  using Scalar = typename Base::Scalar;
  using Coeffs = typename Base::Coeffs;
  using PlainObject = typename Base::PlainObject;
  using MatrixType = typename Base::MatrixType;
  using Tangent = typename Base::Tangent;

  // SO3Base specific types
  using QuaternionType = ::Eigen::Map<Quaternion<Scalar>>;
  using ConstQuaternionType = ::Eigen::Map<const Quaternion<Scalar>>;

  using Base::Tolerance;

  using Base::operator=;
  using Base::coeffs;
  using Base::derived;

  /// Composes this SO3 with other and returns the result.
  ///
  /// @param[in] other The other SO3 to compose with.
  /// @return The composition of this SO3 with other.
  template <typename OtherDerived>
  [[nodiscard]] PlainObject operator*(const SO3Base<OtherDerived>& other) const;

  /**
   * Transforms a 3D vector by this SO3.
   *
   * @tparam MatrixDerived: Type of the matrix
   * @param[in] vec: 3D vector to be transformed
   * @return Transformed 3D vector
   */
  template <typename MatrixDerived>
  [[nodiscard]] Vector3<Scalar> operator*(
      const Eigen::MatrixBase<MatrixDerived>& vec) const;

  /// Normalizes this SO3 so that its quaternion representation is always unit
  /// and unique by keeping the real part of the quaternion positive.
  void normalize();

  /// Returns the inverse of this SO3.
  [[nodiscard]] PlainObject inverse() const;

  /// Returns the matrix representation of this SO3
  ///
  /// The matrix representation of SO3 is a 3x3 orthogonal matrix
  [[nodiscard]] MatrixType toMatrix() const;

  /// Returns the quaternion representation of this SO3.
  [[nodiscard]] const ConstQuaternionType quaternion() const;

  /// Returns the quaternion representation of this SO3.
  [[nodiscard]] QuaternionType quaternion();

  /// Returns the x component of the orientation part in quaternion.
  [[nodiscard]] Scalar quat_x() const;

  /// Returns the y component of the orientation part in quaternion.
  [[nodiscard]] Scalar quat_y() const;

  /// Returns the z component of the orientation part in quaternion.
  [[nodiscard]] Scalar quat_z() const;

  /// Returns the w component of the orientation part in quaternion.
  [[nodiscard]] Scalar quat_w() const;

  /// Returns the x component of the orientation part in quaternion.
  [[nodiscard]] Scalar& quat_x();

  /// Returns the y component of the orientation part in quaternion.
  [[nodiscard]] Scalar& quat_y();

  /// Returns the z component of the orientation part in quaternion.
  [[nodiscard]] Scalar& quat_z();

  /// Returns the w component of the orientation part in quaternion.
  [[nodiscard]] Scalar& quat_w();
};

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

namespace dart::math {

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
template <typename MatrixDerived>
Vector3<typename SO3Base<Derived>::Scalar> SO3Base<Derived>::operator*(
    const Eigen::MatrixBase<MatrixDerived>& vec) const
{
  return quaternion() * vec;
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
typename SO3Base<Derived>::MatrixType SO3Base<Derived>::toMatrix() const
{
  return quaternion().toRotationMatrix();
}

//==============================================================================
template <typename Derived>
const typename SO3Base<Derived>::ConstQuaternionType
SO3Base<Derived>::quaternion() const
{
  return ConstQuaternionType(coeffs().data());
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::QuaternionType SO3Base<Derived>::quaternion()
{
  return QuaternionType(coeffs().data());
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Scalar SO3Base<Derived>::quat_x() const
{
  return coeffs().x();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Scalar SO3Base<Derived>::quat_y() const
{
  return coeffs().y();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Scalar SO3Base<Derived>::quat_z() const
{
  return coeffs().z();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Scalar SO3Base<Derived>::quat_w() const
{
  return coeffs().w();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Scalar& SO3Base<Derived>::quat_x()
{
  return coeffs().x();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Scalar& SO3Base<Derived>::quat_y()
{
  return coeffs().y();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Scalar& SO3Base<Derived>::quat_z()
{
  return coeffs().z();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Scalar& SO3Base<Derived>::quat_w()
{
  return coeffs().w();
}

} // namespace dart::math
