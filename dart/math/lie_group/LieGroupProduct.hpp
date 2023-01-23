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

#include <dart/math/lie_group/LieGroupProductBase.hpp>

namespace Eigen::internal {

// TODO(JS): Move to a dedicated header file
/// @brief Specialization of Eigen::internal::traits for LieGroupProduct
template <typename S, template <typename, int> class... T>
struct traits<::dart::math::LieGroupProduct<S, T...>>
{
  static constexpr std::array<int, sizeof...(T)> CoeffsDimIdx
      = {T<S, 0>::CoeffsDim...};

  static constexpr int CoeffsDim = (T<S, 0>::CoeffsDim + ...);

  using Scalar = S;
  using Coeffs = ::Eigen::Matrix<S, CoeffsDim, 1>;
  using PlainObject = ::dart::math::LieGroupProduct<S, T...>;
  using MatrixType = ::Eigen::Matrix<S, 3, 3>;
  using Tangent = ::Eigen::Matrix<S, 3, 1>;
};

} // namespace Eigen::internal

namespace dart::math {

/// @brief LieGroupProduct is a specialization of LieGroupBase for
/// LieGroupProduct
/// @tparam S The scalar type
/// @tparam Options_ The options for the underlying Eigen::Matrix
template <typename S, template <typename, int> class... T>
class LieGroupProduct : public LieGroupProductBase<LieGroupProduct<S, T...>>
{
public:
  using Base = LieGroupProductBase<LieGroupProduct<S, T...>>;

  // LieGroupBase types
  static constexpr int CoeffsDim = Base::CoeffsDim;
  using Scalar = typename Base::Scalar;
  using Coeffs = typename Base::Coeffs;
  using PlainObject = typename Base::PlainObject;
  using MatrixType = typename Base::MatrixType;
  using Tangent = typename Base::Tangent;

  using Base::Tolerance;

  /// Default constructor that initializes the quaternion to identity
  LieGroupProduct();

  /// Returns the coefficients of the underlying quaternion
  [[nodiscard]] const Coeffs& coeffs() const;

  /// Returns the coefficients of the underlying quaternion
  [[nodiscard]] Coeffs& coeffs();

private:
  /// The underlying quaternion coefficients
  Coeffs m_coeffs;
};

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

namespace dart::math {

//==============================================================================
template <typename S, template <typename, int> class... T>
LieGroupProduct<S, T...>::LieGroupProduct()
  : m_coeffs(Coeffs::Zero()) // TODO(JS): Get coeffs their identities
{
  // Do nothing
}

//==============================================================================
template <typename S, template <typename, int> class... T>
const typename LieGroupProduct<S, T...>::Coeffs&
LieGroupProduct<S, T...>::coeffs() const
{
  return m_coeffs;
}

//==============================================================================
template <typename S, template <typename, int> class... T>
typename LieGroupProduct<S, T...>::Coeffs& LieGroupProduct<S, T...>::coeffs()
{
  return m_coeffs;
}

} // namespace dart::math
