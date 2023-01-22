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

/// @brief Base class for SE3
/// @tparam Derived The derived class
template <typename Derived>
class SE3Base : public LieGroupBase<Derived>
{
public:
  using Base = LieGroupBase<Derived>;

  // LieGroupBase types
  using Scalar = typename Base::Scalar;
  using Coeffs = typename Base::Coeffs;
  using PlainObject = typename Base::PlainObject;
  using MatrixType = typename Base::MatrixType;
  using Tangent = typename Base::Tangent;

  using Base::Tolerance;

  using Base::operator=;
  using Base::coeffs;
  using Base::derived;

  template <typename OtherDerived>
  PlainObject operator*(const SE3Base<OtherDerived>& other) const;

  /**
   * Returns the inverse of this SO3.
   */
  [[nodiscard]] PlainObject inverse() const;

  /// Returns the Isometry3 representation of the SE3
  [[nodiscard]] Isometry3<Scalar> toIsometry3() const;

  /// Returns the matrix representation of the SE3
  [[nodiscard]] MatrixType toMatrix() const;

  /// Returns the rotation
  [[nodiscard]] const Eigen::Map<const SO3<Scalar>> rotation() const;

  /// Returns the rotation
  [[nodiscard]] Eigen::Map<SO3<Scalar>> rotation();

  /// Returns the translation
  [[nodiscard]] const Eigen::Map<const Eigen::Vector3<Scalar>> translation()
      const;

  /// Returns the translation
  [[nodiscard]] Eigen::Map<Eigen::Vector3<Scalar>> translation();

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

  /// Returns the x component of the position part.
  [[nodiscard]] Scalar x() const;

  /// Returns the y component of the position part.
  [[nodiscard]] Scalar y() const;

  /// Returns the z component of the position part.
  [[nodiscard]] Scalar z() const;

  /// Returns the x component of the position part.
  [[nodiscard]] Scalar& x();

  /// Returns the y component of the position part.
  [[nodiscard]] Scalar& y();

  /// Returns the z component of the position part.
  [[nodiscard]] Scalar& z();
};

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

namespace dart::math {

//==============================================================================
template <typename Derived>
template <typename OtherDerived>
typename SE3Base<Derived>::PlainObject SE3Base<Derived>::operator*(
    const SE3Base<OtherDerived>& other) const
{
  const Eigen::Map<const SO3<Scalar>>& o = rotation();
  return PlainObject(
      o * other.rotation(), o * other.translation() + translation());
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::PlainObject SE3Base<Derived>::inverse() const
{
  const SO3<Scalar> r_inv = rotation().inverse();
  return PlainObject(r_inv, -(r_inv * translation()));
}

//==============================================================================
template <typename Derived>
Isometry3<typename SE3Base<Derived>::Scalar> SE3Base<Derived>::toIsometry3()
    const
{
  Isometry3<Scalar> out;
  out.makeAffine();
  out.linear() = rotation().toMatrix();
  out.translation() = translation();
  return out;
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::MatrixType SE3Base<Derived>::toMatrix() const
{
  return toIsometry3().matrix();
}

//==============================================================================
template <typename Derived>
const Eigen::Map<const SO3<typename SE3Base<Derived>::Scalar>>
SE3Base<Derived>::rotation() const
{
  return Eigen::Map<const SO3<Scalar>>(coeffs().data());
}

//==============================================================================
template <typename Derived>
Eigen::Map<SO3<typename SE3Base<Derived>::Scalar>> SE3Base<Derived>::rotation()
{
  return Eigen::Map<SO3<Scalar>>(coeffs().data());
}

//==============================================================================
template <typename Derived>
const Eigen::Map<const Vector3<typename SE3Base<Derived>::Scalar>>
SE3Base<Derived>::translation() const
{
  return Eigen::Map<const Vector3<Scalar>>(coeffs().data() + 4);
}

//==============================================================================
template <typename Derived>
Eigen::Map<Vector3<typename SE3Base<Derived>::Scalar>>
SE3Base<Derived>::translation()
{
  return Eigen::Map<Vector3<Scalar>>(coeffs().data() + 4);
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar SE3Base<Derived>::quat_x() const
{
  return coeffs().x();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar SE3Base<Derived>::quat_y() const
{
  return coeffs().y();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar SE3Base<Derived>::quat_z() const
{
  return coeffs().z();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar SE3Base<Derived>::quat_w() const
{
  return coeffs().w();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar& SE3Base<Derived>::quat_x()
{
  return coeffs().x();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar& SE3Base<Derived>::quat_y()
{
  return coeffs().y();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar& SE3Base<Derived>::quat_z()
{
  return coeffs().z();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar& SE3Base<Derived>::quat_w()
{
  return coeffs().w();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar SE3Base<Derived>::x() const
{
  return coeffs().x();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar SE3Base<Derived>::y() const
{
  return coeffs().y();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar SE3Base<Derived>::z() const
{
  return coeffs().z();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar& SE3Base<Derived>::x()
{
  return coeffs().x();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar& SE3Base<Derived>::y()
{
  return coeffs().y();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar& SE3Base<Derived>::z()
{
  return coeffs().z();
}

} // namespace dart::math
