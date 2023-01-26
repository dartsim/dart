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
#include <dart/math/lie_group/detail/Cast.hpp>

namespace dart::math {

enum NoInitializeTag
{
  NO_INITIALIZE = 0,
};

/// @brief Base class for Lie groups
/// @tparam Derived The derived class
template <typename Derived>
class LieGroupBase
{
public:
  /// The scalar type for the internal representation of the Lie group
  using Scalar = typename ::Eigen::internal::traits<Derived>::Scalar;

  /// @{ @name LieGroup common

  /// The dimension of the Lie group is embedded (e.g., 2D or 3D Eucleadian
  /// space)
  static constexpr int Dim = ::Eigen::internal::traits<Derived>::Dim;

  /// The degree of freedom of the Lie group
  static constexpr int DoF = ::Eigen::internal::traits<Derived>::DoF;

  /// The dimension of the matrix representation of the Lie group
  static constexpr int MatrixRepDim
      = ::Eigen::internal::traits<Derived>::MatrixRepDim;

  /// The number of parameters of the Lie group
  static constexpr int ParamSize
      = ::Eigen::internal::traits<Derived>::ParamSize;

  /// The plain object type of the Lie group
  using PlainObject = typename ::Eigen::internal::traits<Derived>::PlainObject;

  /// The parameter type of the Lie group
  using Params = typename ::Eigen::internal::traits<Derived>::Params;

  /// The matrix type of the Lie group
  using MatrixType = typename ::Eigen::internal::traits<Derived>::MatrixType;

  /// The tangent type of the Lie group
  using Tangent = typename ::Eigen::internal::traits<Derived>::Tangent;

  /// @}

  /// @{ @name LieGroupBase specific

  /// Template type for casting the scalar type of the Lie group
  template <typename OtherScalar>
  using ScalarCastType =
      typename detail::ScalarCast<PlainObject, OtherScalar>::type;

  /// @}

  /// Returns the tolerance for the Lie group
  [[nodiscard]] static constexpr Scalar Tolerance();

  /// Returns the identity of the Lie group
  [[nodiscard]] static PlainObject Identity();

  /// Returns a random element of the Lie group
  [[nodiscard]] static PlainObject Random();

  /// Assignment operator.
  ///
  /// @param[in] other The other Lie group to assign.
  /// @return Reference to this Lie group.
  template <typename OtherDerived>
  LieGroupBase<Derived>& operator=(const LieGroupBase<OtherDerived>& other);

  /// Returns true if this is approximately equal to other
  template <typename OtherDerived>
  [[nodiscard]] bool operator==(const LieGroupBase<OtherDerived>& other) const;

  /// Returns true if this is not approximately equal to other
  template <typename OtherDerived>
  [[nodiscard]] bool operator!=(const LieGroupBase<OtherDerived>& other) const;

  /// Returns true if this SO3 is approximately equal to other within the given
  /// tolerance.
  ///
  /// @param[in] other The other SO3 to compare against.
  /// @param[in] tol The tolerance for equality.
  /// @return True if this SO3 is approximately equal to other within the given
  /// tolerance.
  template <typename OtherDerived>
  [[nodiscard]] bool isApprox(
      const LieGroupBase<OtherDerived>& other, Scalar tol = Tolerance()) const;

  /// Returns the inverse of this Lie group.
  [[nodiscard]] PlainObject inverse() const;

  /// Returns the logarithm of this Lie group.
  [[nodiscard]] Tangent log(Scalar tol = Tolerance()) const;

  /// Returns the matrix representation of this Lie group.
  ///
  /// Note that some Lie groups may need to convert the internal representation
  /// to the matrix representation. This conversion may be expensive.
  ///
  /// @return The matrix representation of this Lie group
  [[nodiscard]] MatrixType toMatrix() const;

  /// Returns the parameters of this Lie group.
  [[nodiscard]] const Params& params() const;

  /// Returns the parameters of this Lie group.
  [[nodiscard]] Params& params();

  /// Returns a new Lie group with the parameters casted to OtherScalar
  template <typename OtherScalar>
  [[nodiscard]] ScalarCastType<OtherScalar> cast() const;

protected:
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
constexpr typename LieGroupBase<Derived>::Scalar
LieGroupBase<Derived>::Tolerance()
{
  // TODO: This is a temporary solution. We need to find a better way to
  // determine the tolerance.
  if constexpr (std::is_same_v<Scalar, float>) {
    return 1e-3;
  } else if constexpr (std::is_same_v<Scalar, double>) {
    return 1e-6;
  } else if constexpr (std::is_same_v<Scalar, long double>) {
    return 1e-9;
  }
}

//==============================================================================
template <typename Derived>
typename LieGroupBase<Derived>::PlainObject LieGroupBase<Derived>::Identity()
{
  const static PlainObject identity = PlainObject();
  return identity;
}

//==============================================================================
template <typename Derived>
typename LieGroupBase<Derived>::PlainObject LieGroupBase<Derived>::Random()
{
  // setRandom() must be implemented in each <group>Base class
  return PlainObject(NO_INITIALIZE).setRandom();
}

//==============================================================================
template <typename Derived>
template <typename OtherDerived>
LieGroupBase<Derived>& LieGroupBase<Derived>::operator=(
    const LieGroupBase<OtherDerived>& other)
{
  params() = other.params();
  return *this;
}

//==============================================================================
template <typename Derived>
template <typename OtherDerived>
bool LieGroupBase<Derived>::operator==(
    const LieGroupBase<OtherDerived>& other) const
{
  return derived().isApprox(other, Tolerance());
}

//==============================================================================
template <typename Derived>
template <typename OtherDerived>
bool LieGroupBase<Derived>::operator!=(
    const LieGroupBase<OtherDerived>& other) const
{
  return !(*this == other);
}

//==============================================================================
template <typename Derived>
template <typename OtherDerived>
bool LieGroupBase<Derived>::isApprox(
    const LieGroupBase<OtherDerived>& other, Scalar tol) const
{
  return (other.inverse() * derived()).log().isZero(tol);
}

//==============================================================================
template <typename Derived>
typename LieGroupBase<Derived>::PlainObject LieGroupBase<Derived>::inverse()
    const
{
  return derived().inverse();
}

//==============================================================================
template <typename Derived>
typename LieGroupBase<Derived>::Tangent LieGroupBase<Derived>::log(
    Scalar tol) const
{
  return PlainObject::Log(derived(), tol);
}

//==============================================================================
template <typename Derived>
typename LieGroupBase<Derived>::MatrixType LieGroupBase<Derived>::toMatrix()
    const
{
  return derived().toMatrix();
}

//==============================================================================
template <typename Derived>
const typename LieGroupBase<Derived>::Params& LieGroupBase<Derived>::params()
    const
{
  return derived().params();
}

//==============================================================================
template <typename Derived>
typename LieGroupBase<Derived>::Params& LieGroupBase<Derived>::params()
{
  return derived().params();
}

//==============================================================================
template <typename Derived>
template <typename OtherScalar>
typename LieGroupBase<Derived>::template ScalarCastType<OtherScalar>
LieGroupBase<Derived>::cast() const
{
  return ScalarCastType<OtherScalar>(params().template cast<OtherScalar>());
}

//==============================================================================
template <typename Derived>
const Derived& LieGroupBase<Derived>::derived() const noexcept
{
  return *static_cast<const Derived*>(this);
}

//==============================================================================
template <typename Derived>
Derived& LieGroupBase<Derived>::derived() noexcept
{
  return *static_cast<Derived*>(this);
}

} // namespace dart::math
