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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
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

#include <dart/math/lie_group/lie_group_base.hpp>

#include <type_traits>

namespace dart::math {

/// Base class for additive Euclidean Lie groups R^n.
template <typename Derived>
class RnBase : public LieGroupBase<Derived>
{
public:
  using Base = LieGroupBase<Derived>;
  using Scalar = typename Base::Scalar;

  // LieGroup common
  using Base::Dim;
  using Base::DoF;
  using Base::MatrixRepDim;
  using Base::ParamSize;
  using InverseType = typename Base::InverseType;
  using LieGroup = typename Base::LieGroup;
  using MatrixType = typename Base::MatrixType;
  using Params = typename Base::Params;
  using Tangent = typename Base::Tangent;

  using Base::derived;
  using Base::operator=;
  using Base::params;

  /// Adds this R^n element to another R^n element.
  template <typename OtherDerived>
  [[nodiscard]] LieGroup operator*(
      const LieGroupBase<OtherDerived>& other) const;

  /// Sets this R^n element to a random vector.
  Derived& setRandom();

  /// Negates this R^n element in place.
  Derived& inverseInPlace();

  /// R^n parameters are already normalized.
  void normalize();

  /// Returns this R^n element as its tangent vector.
  [[nodiscard]] Tangent log(Scalar tol = LieGroupTol<Scalar>()) const;

  /// Returns this R^n element as its tangent vector and log Jacobian.
  template <typename MatrixDerived>
  [[nodiscard]] Tangent log(
      Eigen::MatrixBase<MatrixDerived>* jacobian,
      Scalar tol = LieGroupTol<Scalar>()) const;

  /// Returns the identity adjoint action for the abelian R^n group.
  template <typename MatrixDerived>
  [[nodiscard]] Tangent ad(const Eigen::MatrixBase<MatrixDerived>& dx) const;

  /// Returns the identity adjoint matrix for the abelian R^n group.
  [[nodiscard]] Matrix<Scalar, DoF, DoF> toAdjointMatrix() const;

  /// Returns a homogeneous translation matrix representation of R^n.
  [[nodiscard]] MatrixType toMatrix() const;
};

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

namespace dart::math {

//==============================================================================
template <typename Derived>
template <typename OtherDerived>
typename RnBase<Derived>::LieGroup RnBase<Derived>::operator*(
    const LieGroupBase<OtherDerived>& other) const
{
  static_assert(
      std::is_same_v<LieGroup, typename OtherDerived::LieGroup>,
      "R^n operands must have the same scalar type and dimension.");
  return LieGroup(params() + other.derived().params());
}

//==============================================================================
template <typename Derived>
Derived& RnBase<Derived>::setRandom()
{
  params().setRandom();
  return derived();
}

//==============================================================================
template <typename Derived>
Derived& RnBase<Derived>::inverseInPlace()
{
  params() = -params();
  return derived();
}

//==============================================================================
template <typename Derived>
void RnBase<Derived>::normalize()
{
  // Do nothing
}

//==============================================================================
template <typename Derived>
typename RnBase<Derived>::Tangent RnBase<Derived>::log(Scalar) const
{
  return params();
}

//==============================================================================
template <typename Derived>
template <typename MatrixDerived>
typename RnBase<Derived>::Tangent RnBase<Derived>::log(
    Eigen::MatrixBase<MatrixDerived>* jacobian, Scalar tol) const
{
  if (jacobian != nullptr) {
    *jacobian = Matrix<Scalar, DoF, DoF>::Identity();
  }
  return log(tol);
}

//==============================================================================
template <typename Derived>
template <typename MatrixDerived>
typename RnBase<Derived>::Tangent RnBase<Derived>::ad(
    const Eigen::MatrixBase<MatrixDerived>& dx) const
{
  return dx;
}

//==============================================================================
template <typename Derived>
Matrix<typename RnBase<Derived>::Scalar, RnBase<Derived>::DoF>
RnBase<Derived>::toAdjointMatrix() const
{
  return Matrix<Scalar, DoF, DoF>::Identity();
}

//==============================================================================
template <typename Derived>
typename RnBase<Derived>::MatrixType RnBase<Derived>::toMatrix() const
{
  MatrixType out = MatrixType::Identity();
  out.template topRightCorner<DoF, 1>() = params();
  return out;
}

} // namespace dart::math
