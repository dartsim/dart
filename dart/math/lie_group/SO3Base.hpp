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
  using Data = typename Base::Data;
  using PlainObject = typename Base::PlainObject;
  using MatrixType = typename Base::MatrixType;

  using Base::derived;

  /// Returns true if this SO3 is approximately equal to other within the given
  /// tolerance.
  ///
  /// @param[in] other The other SO3 to compare against.
  /// @param[in] tol The tolerance for equality.
  /// @return True if this SO3 is approximately equal to other within the given
  /// tolerance.
  template <typename OtherDerived>
  [[nodiscard]] bool isApprox(
      const SO3Base<OtherDerived>& other, Scalar tol = Base::Tolerance()) const;

  /// Returns the quaternion representation of this SO3.
  [[nodiscard]] const Data& quaternion() const;

  /// Returns the quaternion representation of this SO3.
  [[nodiscard]] Data& quaternion();
};

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

namespace dart::math {

//==============================================================================
template <typename Derived>
template <typename OtherDerived>
bool SO3Base<Derived>::isApprox(
    const SO3Base<OtherDerived>& other, Scalar tol) const
{
  return quaternion().isApprox(other.quaternion(), tol);
}

//==============================================================================
template <typename Derived>
const typename SO3Base<Derived>::Data& SO3Base<Derived>::quaternion() const
{
  return derived().quaternion();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Data& SO3Base<Derived>::quaternion()
{
  return derived().quaternion();
}

} // namespace dart::math
