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

  /// The plain object type of the Lie group
  using PlainObject = typename ::Eigen::internal::traits<Derived>::PlainObject;

  /// The data type of the underlying data
  using Params = typename ::Eigen::internal::traits<Derived>::Params;

  /// @}

  bool isZero(Scalar tol) const
  {
    return params().isZero(tol);
  }

  auto operator[](int i) const
  {
    return params()[i];
  }

  auto operator[](int i)
  {
    return params()[i];
  }

  const Params& params() const
  {
    return derived().params();
  }

  Params& params()
  {
    return derived().params();
  }

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
