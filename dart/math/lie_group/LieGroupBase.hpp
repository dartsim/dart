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

namespace dart::math {

/// @brief Base class for Lie groups
/// @tparam Derived The derived class
template <typename Derived>
class LieGroupBase
{
public:
  using Scalar = typename ::Eigen::internal::traits<Derived>::Scalar;
  using Data = typename ::Eigen::internal::traits<Derived>::Data;
  using PlainObject = typename ::Eigen::internal::traits<Derived>::PlainObject;
  using MatrixType = typename ::Eigen::internal::traits<Derived>::MatrixType;

  /// Returns the tolerance for the Lie group
  [[nodiscard]] static constexpr Scalar Tolerance();

  /// Returns the identity of the Lie group
  [[nodiscard]] static PlainObject Identity();

  /// Returns a random element of the Lie group
  [[nodiscard]] static PlainObject Random();

  /// Returns true if this is approximately equal to other
  [[nodiscard]] bool operator==(const Derived& other) const;

  /// Returns true if this is not approximately equal to other
  [[nodiscard]] bool operator!=(const Derived& other) const;

  /// Returns the matrix representation of this Lie group
  [[nodiscard]] MatrixType matrix() const;

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
    return 1e-12;
  }
}

//==============================================================================
template <typename Derived>
typename LieGroupBase<Derived>::PlainObject LieGroupBase<Derived>::Identity()
{
  return Derived::Identity();
}

//==============================================================================
template <typename Derived>
typename LieGroupBase<Derived>::PlainObject LieGroupBase<Derived>::Random()
{
  return Derived::Random();
}

//==============================================================================
template <typename Derived>
bool LieGroupBase<Derived>::operator==(const Derived& other) const
{
  return derived().isApprox(other, Tolerance());
}

//==============================================================================
template <typename Derived>
bool LieGroupBase<Derived>::operator!=(const Derived& other) const
{
  return !(*this == other);
}

//==============================================================================
template <typename Derived>
typename LieGroupBase<Derived>::MatrixType LieGroupBase<Derived>::matrix() const
{
  return derived().matrix();
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
