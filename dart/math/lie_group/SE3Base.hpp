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
  using Scalar = typename Base::Scalar;

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

  /**
   * @brief Group multiplication operator
   *
   * @tparam OtherDerived The type of the other Lie group point
   * @param[in] other: The other SE3 point
   * @return LieGroup The result of the multiplication
   */
  template <typename OtherDerived>
  LieGroup operator*(const LieGroupBase<OtherDerived>& other) const;

  /**
   * @brief Vector multiplication operator
   *
   * @tparam OtherDerived The type of the other SE3 point
   * @param[in] other: The other SE3 point
   * @return LieGroup& The result of the multiplication
   */
  template <typename MatrixDerived>
  Vector3<Scalar> operator*(const MatrixBase<MatrixDerived>& vec) const;

  /**
   * @brief Sets the group to random
   *
   * @return The reference of this SE3
   */
  Derived& setRandom();

  /**
   * Returns the inverse of this SO3.
   */
  Derived& inverseInPlace();

  /// Returns the logarithm map of the given SE3
  ///
  /// The logarithm map of an SE3 @f$ x @f$ is a vector @f$ \xi @f$ such
  /// that @f$ \log(x) = \xi @f$.
  ///
  /// @param[in] x The SE3 to be converted to a vector.
  /// @param[in] tol The tolerance for the norm of the vector.
  /// @return The vector.
  /// @tparam MatrixDrived The type of the SE3
  /// @see Exp()
  [[nodiscard]] Tangent log(Scalar tol = LieGroupTol<Scalar>()) const;

  /// Returns the logarithm map of the given SE3
  ///
  /// The logarithm map of an SE3 @f$ x @f$ is a vector @f$ \xi @f$ such
  /// that @f$ \log(x) = \xi @f$.
  ///
  /// This function also returns the Jacobian of the logarithm map.
  ///
  /// @param[in] x The SE3 to be converted to a vector.
  /// @param[out] jacobian The Jacobian of the logarithm map.
  /// @param[in] tol The tolerance for the norm of the vector.
  /// @return The vector.
  /// @tparam MatrixDrivedA The type of the SE3
  /// @tparam MatrixDerivedB The type of the Jacobian
  /// @see Exp()
  template <typename MatrixDerived>
  [[nodiscard]] Tangent log(
      Eigen::MatrixBase<MatrixDerived>* jacobian,
      Scalar tol = LieGroupTol<Scalar>()) const;

  /// Performs the adjoint transformation on the given se(3) by the given SE(3)
  template <typename TangentDerived>
  [[nodiscard]] Tangent ad(const TangentBase<TangentDerived>& dx) const
  {
    // Cache the rotation matrix for efficiency when multiplying 3d vector more
    // than once
    const Eigen::Matrix<Scalar, 3, 3> rot = rotation().toMatrix();

    const Vector3<Scalar> angular = rot * dx.derived().angular();

    Vector3<Scalar> linear = rot * dx.derived().linear();
    linear.noalias() += translation().cross(angular);

    return Tangent(std::move(angular), std::move(linear));
  }

  /// Returns the adjoint transformation matrix (6x6) of the given SE(3) point
  [[nodiscard]] Matrix6<Scalar> toAdjointMatrix() const
  {
    Matrix6<Scalar> out;
    out.template topLeftCorner<3, 3>() = rotation().toMatrix();
    out.template topRightCorner<3, 3>().setZero();
    out.template bottomLeftCorner<3, 3>().noalias()
        = skew(translation()) * out.template topLeftCorner<3, 3>();
    out.template bottomRightCorner<3, 3>() = out.template topLeftCorner<3, 3>();
    return out;
  }

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
typename SE3Base<Derived>::LieGroup SE3Base<Derived>::operator*(
    const LieGroupBase<OtherDerived>& other) const
{
  const Eigen::Map<const SO3<Scalar>>& o = rotation();
  return LieGroup(
      o * other.derived().rotation(),
      o * other.derived().translation() + translation());
}

//==============================================================================
template <typename Derived>
template <typename MatrixDerived>
Vector3<typename SE3Base<Derived>::Scalar> SE3Base<Derived>::operator*(
    const MatrixBase<MatrixDerived>& vec) const
{
  return rotation() * vec + translation();
}

//==============================================================================
template <typename Derived>
Derived& SE3Base<Derived>::setRandom()
{
  rotation().setRandom();
  translation().setRandom();
  return derived();
}

//==============================================================================
template <typename Derived>
Derived& SE3Base<Derived>::inverseInPlace()
{
  const SO3<Scalar> r_inv = rotation().inverseInPlace();
  rotation() = r_inv;
  translation() = -(r_inv * translation());
  return derived();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Tangent SE3Base<Derived>::log(Scalar tol) const
{
  const Eigen::Vector3<Scalar> angular = rotation().log(tol).params();
  const Eigen::Vector3<Scalar> linear
      = SO3<Scalar>::LeftJacobianInverse(angular, tol) * translation();
  return Tangent(std::move(angular), std::move(linear));
}

//==============================================================================
template <typename Derived>
template <typename MatrixDerived>
typename SE3Base<Derived>::Tangent SE3Base<Derived>::log(
    Eigen::MatrixBase<MatrixDerived>* jacobian, Scalar tol) const
{
  const Tangent xi = log(tol);
  if (jacobian) {
    (*jacobian) = SE3<Scalar>::RightJacobianInverse(xi, tol);
  }
  return xi;
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
  return Eigen::Map<const SO3<Scalar>>(params().data());
}

//==============================================================================
template <typename Derived>
Eigen::Map<SO3<typename SE3Base<Derived>::Scalar>> SE3Base<Derived>::rotation()
{
  return Eigen::Map<SO3<Scalar>>(params().data());
}

//==============================================================================
template <typename Derived>
const Eigen::Map<const Vector3<typename SE3Base<Derived>::Scalar>>
SE3Base<Derived>::translation() const
{
  return Eigen::Map<const Vector3<Scalar>>(params().data() + 4);
}

//==============================================================================
template <typename Derived>
Eigen::Map<Vector3<typename SE3Base<Derived>::Scalar>>
SE3Base<Derived>::translation()
{
  return Eigen::Map<Vector3<Scalar>>(params().data() + 4);
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar SE3Base<Derived>::quat_x() const
{
  return params().x();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar SE3Base<Derived>::quat_y() const
{
  return params().y();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar SE3Base<Derived>::quat_z() const
{
  return params().z();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar SE3Base<Derived>::quat_w() const
{
  return params().w();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar& SE3Base<Derived>::quat_x()
{
  return params().x();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar& SE3Base<Derived>::quat_y()
{
  return params().y();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar& SE3Base<Derived>::quat_z()
{
  return params().z();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar& SE3Base<Derived>::quat_w()
{
  return params().w();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar SE3Base<Derived>::x() const
{
  return params().x();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar SE3Base<Derived>::y() const
{
  return params().y();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar SE3Base<Derived>::z() const
{
  return params().z();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar& SE3Base<Derived>::x()
{
  return params().x();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar& SE3Base<Derived>::y()
{
  return params().y();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar& SE3Base<Derived>::z()
{
  return params().z();
}

} // namespace dart::math
