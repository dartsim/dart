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

#include <dart/math/Fwd.hpp>
#include <dart/math/lie_group/Helpers.hpp>

namespace dart::math {

template <typename Derived>
class TangentBase
{
public:
  /// The scalar type for the internal representation of the Lie group
  using Scalar = typename ::Eigen::internal::traits<Derived>::Scalar;

  /// @{ @name Tangent common

  /// The dimension of the Lie group is embedded (e.g., 2D or 3D Eucleadian
  /// space)
  static constexpr int Dim = ::Eigen::internal::traits<Derived>::Dim;

  /// The degree of freedom of the Lie group
  static constexpr int DoF = ::Eigen::internal::traits<Derived>::DoF;

  /// The size of the underlying data
  static constexpr int ParamSize
      = ::Eigen::internal::traits<Derived>::ParamSize;

  using LieGroup = typename ::Eigen::internal::traits<Derived>::LieGroup;

  /// The plain object type of the Lie group
  using Tangent = typename ::Eigen::internal::traits<Derived>::Tangent;

  using LieAlgebra = typename ::Eigen::internal::traits<Derived>::LieAlgebra;

  /// The data type of the underlying data
  using Params = typename ::Eigen::internal::traits<Derived>::Params;

  /// @}

  [[nodiscard]] static Tangent Zero();

  /// Returns a random element of the tangent of the Lie group
  [[nodiscard]] static Tangent Random();

  template <typename OtherDerived>
  Derived& operator=(const TangentBase<OtherDerived>& other);

  template <typename OtherDerived>
  Derived& operator=(TangentBase<OtherDerived>&& other);

  template <typename MatrixDerived>
  Derived& operator=(const MatrixBase<MatrixDerived>& other);

  template <typename MatrixDerived>
  Derived& operator=(MatrixBase<MatrixDerived>&& other);

  /// Returns true if this is approximately equal to other
  template <typename OtherDerived>
  [[nodiscard]] bool operator==(const TangentBase<OtherDerived>& other) const;

  /// Returns true if this is not approximately equal to other
  template <typename OtherDerived>
  [[nodiscard]] bool operator!=(const TangentBase<OtherDerived>& other) const;

  /// Comma initializer
  template <typename T>
  auto operator<<(T&& v);

  /// Returns the hat operator of the tangent element
  [[nodiscard]] LieAlgebra hat() const;

  /// Returns the exponential map of the given tangent
  ///
  /// The exponential map of a vector @f$ \xi @f$ is a Lie group element @f$ x
  /// @f$ such that @f$ \log(x) = \xi @f$.
  ///
  /// @param[in] dx The vector to be converted to a Lie group element.
  /// @param[in] tol The tolerance for the norm of the vector.
  /// @return The Lie group element.
  /// @tparam MatrixDrived The type of the vector
  [[nodiscard]] LieGroup exp(Scalar tol = LieGroupTol<Scalar>()) const;

  /// Returns the exponential map of the given tangent
  ///
  /// The exponential map of a vector @f$ \xi @f$ is a Lie group element @f$ x
  /// @f$ such that @f$ \log(x) = \xi @f$.
  ///
  /// This function also returns the Jacobian of the exponential map.
  ///
  /// @param[in] dx The vector to be converted to a Lie group element.
  /// @param[out] jacobian The Jacobian of the exponential map.
  /// @param[in] tol The tolerance for the norm of the vector.
  /// @return The Lie group element.
  /// @tparam MatrixDrivedA The type of the vector
  /// @tparam MatrixDerivedB The type of the Jacobian
  template <typename MatrixDerived>
  [[nodiscard]] LieGroup exp(
      Eigen::MatrixBase<MatrixDerived>* jacobian,
      Scalar tol = LieGroupTol<Scalar>()) const;

  template <typename OtherDerived>
  [[nodiscard]] Tangent ad(const TangentBase<OtherDerived>& other) const;

  [[nodiscard]] bool isZero(Scalar tol = LieGroupTol<Scalar>()) const;

  template <typename OtherDerived>
  [[nodiscard]] bool isApprox(
      const TangentBase<OtherDerived>& other,
      Scalar tol = LieGroupTol<Scalar>()) const;

  template <typename MatrixDerived>
  [[nodiscard]] bool isApprox(
      const Eigen::MatrixBase<MatrixDerived>& mat,
      Scalar tol = LieGroupTol<Scalar>()) const;
  [[nodiscard]] auto operator[](int i) const;

  [[nodiscard]] auto operator[](int i);

  [[nodiscard]] const Params& params() const;

  [[nodiscard]] Params& params();

  /// Returns the derived class as a const reference
  [[nodiscard]] const Derived& derived() const noexcept;

  /// Returns the derived class as a reference
  [[nodiscard]] Derived& derived() noexcept;
};

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

namespace dart::math {

//==============================================================================
template <typename Derived>
typename TangentBase<Derived>::Tangent TangentBase<Derived>::Zero()
{
  return Tangent(Params::Zero());
}

//==============================================================================
template <typename Derived>
typename TangentBase<Derived>::Tangent TangentBase<Derived>::Random()
{
  return Tangent(Params::Random());
}

//==============================================================================
template <typename Derived>
template <typename OtherDerived>
Derived& TangentBase<Derived>::operator=(const TangentBase<OtherDerived>& other)
{
  params() = other.params();
  return derived();
}

//==============================================================================
template <typename Derived>
template <typename OtherDerived>
Derived& TangentBase<Derived>::operator=(TangentBase<OtherDerived>&& other)
{
  params() = std::move(other.params());
  return derived();
}

//==============================================================================
template <typename Derived>
template <typename MatrixDerived>
Derived& TangentBase<Derived>::operator=(const MatrixBase<MatrixDerived>& other)
{
  params() = other;
  return derived();
}

//==============================================================================
template <typename Derived>
template <typename MatrixDerived>
Derived& TangentBase<Derived>::operator=(MatrixBase<MatrixDerived>&& other)
{
  params() = std::move(other);
  return derived();
}

//==============================================================================
template <typename Derived>
template <typename OtherDerived>
bool TangentBase<Derived>::operator==(
    const TangentBase<OtherDerived>& other) const
{
  return derived().isApprox(other, LieGroupTol<Scalar>());
}

//==============================================================================
template <typename Derived>
template <typename OtherDerived>
bool TangentBase<Derived>::operator!=(
    const TangentBase<OtherDerived>& other) const
{
  return !(*this == other);
}

//==============================================================================
template <typename Derived>
template <typename T>
auto TangentBase<Derived>::operator<<(T&& v)
{
  return params().operator<<(std::forward<T>(v));
}

//==============================================================================
template <typename Derived>
typename TangentBase<Derived>::LieAlgebra TangentBase<Derived>::hat() const
{
  return derived().hat();
}

//==============================================================================
template <typename Derived>
typename TangentBase<Derived>::LieGroup TangentBase<Derived>::exp(
    Scalar tol) const
{
  return derived().exp(tol);
}

//==============================================================================
template <typename Derived>
template <typename MatrixDerived>
typename TangentBase<Derived>::LieGroup TangentBase<Derived>::exp(
    Eigen::MatrixBase<MatrixDerived>* jacobian, Scalar tol) const
{
  return derived().exp(jacobian, tol);
}

//==============================================================================
template <typename Derived>
template <typename OtherDerived>
typename TangentBase<Derived>::Tangent TangentBase<Derived>::ad(
    const TangentBase<OtherDerived>& other) const
{
  return derived().ad(other);
}

//==============================================================================
template <typename Derived>
bool TangentBase<Derived>::isZero(Scalar tol) const
{
  return params().isZero(tol);
}

//==============================================================================
template <typename Derived>
template <typename OtherDerived>
bool TangentBase<Derived>::isApprox(
    const TangentBase<OtherDerived>& other, Scalar tol) const
{
  return isApprox(other.params(), tol);
}

//==============================================================================
template <typename Derived>
template <typename MatrixDerived>
bool TangentBase<Derived>::isApprox(
    const Eigen::MatrixBase<MatrixDerived>& mat, Scalar tol) const
{
  return params().isApprox(mat, tol);
}

//==============================================================================
template <typename Derived>
auto TangentBase<Derived>::operator[](int i) const
{
  return params()[i];
}

//==============================================================================
template <typename Derived>
auto TangentBase<Derived>::operator[](int i)
{
  return params()[i];
}

//==============================================================================
template <typename Derived>
const typename TangentBase<Derived>::Params& TangentBase<Derived>::params()
    const
{
  return derived().params();
}

//==============================================================================
template <typename Derived>
typename TangentBase<Derived>::Params& TangentBase<Derived>::params()
{
  return derived().params();
}

//==============================================================================
template <typename Derived>
const Derived& TangentBase<Derived>::derived() const noexcept
{
  return *static_cast<const Derived*>(this);
}

//==============================================================================
template <typename Derived>
Derived& TangentBase<Derived>::derived() noexcept
{
  return *static_cast<Derived*>(this);
}

} // namespace dart::math
