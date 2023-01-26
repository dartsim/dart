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

#include <dart/math/lie_group/TangentBase.hpp>

namespace dart::math {

template <typename Derived>
class SO3TangentBase : public TangentBase<Derived>
{
public:
  using Base = TangentBase<Derived>;
  using Scalar = typename Base::Scalar;

  // LieGroup common
  using Base::Dim;
  using Base::DoF;
  using Base::ParamSize;
  using LieGroup = typename Base::LieGroup;
  using Tangent = typename Base::Tangent;
  using Params = typename Base::Params;

  using Base::derived;
  using Base::params;

  /// Returns the exponential map of the given vector
  ///
  /// The exponential map of a vector @f$ \xi @f$ is an SO3 @f$ x @f$ such
  /// that @f$ \log(x) = \xi @f$.
  ///
  /// @param[in] dx The vector to be converted to an SO3.
  /// @param[in] tol The tolerance for the norm of the vector.
  /// @return The SO3.
  /// @tparam MatrixDrived The type of the vector
  [[nodiscard]] LieGroup exp(Scalar tol = LieGroupTol<Scalar>()) const;

  /// Returns the exponential map of the given vector
  ///
  /// The exponential map of a vector @f$ \xi @f$ is an SO3 @f$ x @f$ such
  /// that @f$ \log(x) = \xi @f$.
  ///
  /// This function also returns the Jacobian of the exponential map.
  ///
  /// @param[in] dx The vector to be converted to an SO3.
  /// @param[out] jacobian The Jacobian of the exponential map.
  /// @param[in] tol The tolerance for the norm of the vector.
  /// @return The SO3.
  /// @tparam MatrixDrivedA The type of the vector
  /// @tparam MatrixDerivedB The type of the Jacobian
  template <typename MatrixDerived>
  [[nodiscard]] LieGroup exp(
      Eigen::MatrixBase<MatrixDerived>* jacobian,
      Scalar tol = LieGroupTol<Scalar>()) const;
};

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

#include <dart/math/lie_group/SO3.hpp>

namespace dart::math {

//==============================================================================
template <typename Derived>
typename SO3TangentBase<Derived>::LieGroup SO3TangentBase<Derived>::exp(
    Scalar tol) const
{
  const Scalar theta = params().norm();
  if (theta < tol) {
    const Tangent vec = 0.5 * params();
    return LieGroup(Eigen::Quaternion<Scalar>(1.0, vec[0], vec[1], vec[2]));
  }

  const Scalar half_theta = 0.5 * theta;
  const Scalar sin_half_theta = std::sin(half_theta);
  const Scalar cos_half_theta = std::cos(half_theta);
  const Tangent vec = (sin_half_theta / theta) * params();
  return LieGroup(
      Eigen::Quaternion<Scalar>(cos_half_theta, vec[0], vec[1], vec[2]));
  // TODO(JS): Consider creating a constructor from Params type
}

//==============================================================================
template <typename Derived>
template <typename MatrixDerived>
typename SO3TangentBase<Derived>::LieGroup SO3TangentBase<Derived>::exp(
    Eigen::MatrixBase<MatrixDerived>* jacobian, Scalar tol) const
{
  if (jacobian) {
    *jacobian = SO3<Scalar>::RightJacobian(params(), tol);
  }
  return exp(tol);
}

} // namespace dart::math
