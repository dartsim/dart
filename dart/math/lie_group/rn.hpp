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

#include <dart/math/lie_group/rn_base.hpp>
#include <dart/math/lie_group/rn_inverse.hpp>

#include <dart/common/macros.hpp>

#include <utility>

namespace Eigen::internal {

/// Specialization of Eigen::internal::traits for Rn.
template <typename S, int N>
struct traits<::dart::math::Rn<S, N>>
{
  static_assert(N > 0, "Rn dimension must be positive.");

  using Scalar = S;

  // LieGroup common
  static constexpr int Dim = N;
  static constexpr int DoF = N;
  static constexpr int MatrixRepDim = N + 1;
  static constexpr int ParamSize = N;
  using LieGroup = ::dart::math::Rn<S, N>;
  using InverseType = ::dart::math::RnInverse<LieGroup>;
  using MatrixType = ::Eigen::Matrix<S, MatrixRepDim, MatrixRepDim>;
  using Params = ::Eigen::Matrix<S, ParamSize, 1>;
  using PlainObject = LieGroup;
  using Tangent = ::Eigen::Matrix<S, DoF, 1>;
};

} // namespace Eigen::internal

namespace dart::math {

/// Additive Euclidean Lie group R^n.
template <typename S, int N>
class Rn : public RnBase<Rn<S, N>>
{
public:
  using Base = RnBase<Rn<S, N>>;
  using Scalar = typename Base::Scalar;

  // LieGroup common
  using LieGroup = typename Base::LieGroup;
  using MatrixType = typename Base::MatrixType;
  using Params = typename Base::Params;
  using PlainObject = LieGroup;
  using Tangent = typename Base::Tangent;

  /// Default constructor that initializes the vector to zero.
  Rn();

  /// Constructs without initializing the underlying parameters.
  explicit Rn(NoInitializeTag);

  DART_LIEGROUP_CONSTRUCTORS(Rn);

  /// Constructs one-dimensional R from a scalar.
  explicit Rn(Scalar value)
    requires(N == 1);

  /// Copy assignment operator.
  Rn& operator=(const Rn& other);

  /// Move assignment operator.
  Rn& operator=(Rn&& other) noexcept;

  /// Returns the vector parameters.
  [[nodiscard]] const Params& params() const;

  /// Returns the vector parameters.
  [[nodiscard]] Params& params();

private:
  Params m_params;
};

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

namespace dart::math {

//==============================================================================
template <typename S, int N>
Rn<S, N>::Rn() : m_params(Params::Zero())
{
  // Do nothing
}

//==============================================================================
template <typename S, int N>
Rn<S, N>::Rn(NoInitializeTag)
{
  // Do nothing
}

//==============================================================================
template <typename S, int N>
Rn<S, N>::Rn(Scalar value)
  requires(N == 1)
  : m_params(Params::Constant(value))
{
  // Do nothing
}

//==============================================================================
template <typename S, int N>
Rn<S, N>& Rn<S, N>::operator=(const Rn& other)
{
  m_params = other.m_params;
  return *this;
}

//==============================================================================
template <typename S, int N>
Rn<S, N>& Rn<S, N>::operator=(Rn&& other) noexcept
{
  m_params = std::move(other.m_params);
  return *this;
}

//==============================================================================
template <typename S, int N>
const typename Rn<S, N>::Params& Rn<S, N>::params() const
{
  return m_params;
}

//==============================================================================
template <typename S, int N>
typename Rn<S, N>::Params& Rn<S, N>::params()
{
  return m_params;
}

} // namespace dart::math
