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
  using Scalar = typename Base::Scalar;

  // SO3Base specific types
  using QuaternionType = ::Eigen::Map<Quaternion<Scalar>>;
  using ConstQuaternionType = ::Eigen::Map<const Quaternion<Scalar>>;

  // LieGroup common
  using Base::Dim;
  using Base::DoF;
  using Base::MatrixRepDim;
  using Base::ParamSize;
  using LieGroup = typename Base::LieGroup;
  using InverseType = typename Base::InverseType;
  using MatrixType = typename Base::MatrixType;
  using Params = typename Base::Params;
  using Tangent = typename Base::Tangent;

  using Base::operator=;
  using Base::derived;
  using Base::params;

  /// Composes this SO3 with other and returns the result.
  ///
  /// @param[in] other The other SO3 to compose with.
  /// @return The composition of this SO3 with other.
  template <typename OtherDerived>
  [[nodiscard]] LieGroup operator*(
      const LieGroupBase<OtherDerived>& other) const;

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

  /**
   * @brief Set the SO3 to a random rotation
   *
   * @return Reference to this SO3
   */
  Derived& setRandom();

  /// Performs in-place inverse and returns the reference of the derived
  Derived& inverseInPlace();

  /// Returns the logarithm map of the given SO3
  ///
  /// The logarithm map of an SO3 @f$ x @f$ is a vector @f$ \xi @f$ such
  /// that @f$ \log(x) = \xi @f$.
  ///
  /// @param[in] x The SO3 to be converted to a vector.
  /// @param[in] tol The tolerance for the norm of the vector.
  /// @return The vector.
  /// @tparam MatrixDrived The type of the SO3
  /// @see Exp()
  [[nodiscard]] Tangent log(Scalar tol = LieGroupTol<Scalar>()) const;

  /// Returns the logarithm map of the given SO3
  ///
  /// The logarithm map of an SO3 @f$ x @f$ is a vector @f$ \xi @f$ such
  /// that @f$ \log(x) = \xi @f$.
  ///
  /// This function also returns the Jacobian of the logarithm map.
  ///
  /// @param[in] x The SO3 to be converted to a vector.
  /// @param[out] jacobian The Jacobian of the logarithm map.
  /// @param[in] tol The tolerance for the norm of the vector.
  /// @return The vector.
  /// @tparam MatrixDrivedA The type of the SO3
  /// @tparam MatrixDerivedB The type of the Jacobian
  /// @see Exp()
  template <typename MatrixDerived>
  [[nodiscard]] Tangent log(
      Eigen::MatrixBase<MatrixDerived>* jacobian,
      Scalar tol = LieGroupTol<Scalar>()) const;

  /// Returns the adjoint transformation of the given SO3
  ///
  /// The adjoint transformation of SO3 @f$ x @f$ is a matrix @f$ A @f$ such
  /// that @f$ A v = x v x^{-1} @f$ for any vector @f$ v @f$.
  ///
  /// @param[in] x The SO3 to be converted to a matrix.
  template <typename TangentDerived>
  [[nodiscard]] Tangent ad(const TangentBase<TangentDerived>& dx) const
  {
    return Tangent(quaternion() * dx.params());
  }

  [[nodiscard]] Matrix3<Scalar> toAdjointMatrix() const
  {
    return toMatrix();
  }

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
typename SO3Base<Derived>::LieGroup SO3Base<Derived>::operator*(
    const LieGroupBase<OtherDerived>& other) const
{
  return LieGroup(quaternion() * other.derived().quaternion());
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
  if (params().w() < 0) {
    params() *= -1;
  }
  params().normalize();
}

//==============================================================================
template <typename Derived>
Derived& SO3Base<Derived>::setRandom()
{
  quaternion() = Quaternion<Scalar>::UnitRandom();
  normalize();
  return derived();
}

//==============================================================================
template <typename Derived>
Derived& SO3Base<Derived>::inverseInPlace()
{
  quaternion() = quaternion().conjugate();
  return derived();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Tangent SO3Base<Derived>::log(Scalar tol) const
{
  const Scalar cos_theta = quaternion().w();
  const Scalar theta = 2 * std::acos(cos_theta);
  DART_ASSERT(!std::isnan(theta));

  if (theta < tol) {
    return Tangent(2 * quaternion().vec());
  }

  const Scalar theta_over_sin_half_theta = theta / std::sin(0.5 * theta);
  return Tangent(theta_over_sin_half_theta * quaternion().vec());
}

//==============================================================================
template <typename Derived>
template <typename MatrixDerived>
typename SO3Base<Derived>::Tangent SO3Base<Derived>::log(
    Eigen::MatrixBase<MatrixDerived>* jacobian, Scalar tol) const
{
  const Tangent xi = log(tol);
  if (jacobian) {
    (*jacobian) = SO3<Scalar>::RightJacobianInverse(xi, tol);
  }
  return xi;
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
  return ConstQuaternionType(params().data());
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::QuaternionType SO3Base<Derived>::quaternion()
{
  return QuaternionType(params().data());
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Scalar SO3Base<Derived>::quat_x() const
{
  return params().x();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Scalar SO3Base<Derived>::quat_y() const
{
  return params().y();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Scalar SO3Base<Derived>::quat_z() const
{
  return params().z();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Scalar SO3Base<Derived>::quat_w() const
{
  return params().w();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Scalar& SO3Base<Derived>::quat_x()
{
  return params().x();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Scalar& SO3Base<Derived>::quat_y()
{
  return params().y();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Scalar& SO3Base<Derived>::quat_z()
{
  return params().z();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Scalar& SO3Base<Derived>::quat_w()
{
  return params().w();
}

} // namespace dart::math
