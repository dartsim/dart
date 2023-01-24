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

#include <dart/math/lie_group/SE3.hpp>

namespace Eigen::internal {

/// Specialization of Eigen::internal::traits for const dart::math::SE3
template <typename S>
struct traits<::Eigen::Map<const ::dart::math::SE3<S>>>
  : traits<const ::dart::math::SE3<S>>
{
  using Base = traits<const ::dart::math::SE3<S>>;
  using Scalar = typename Base::Scalar;

  // LieGroup common
  static constexpr int ParamSize = Base::ParamSize;
  using Params = typename ::Eigen::Map<const ::Eigen::Matrix<S, ParamSize, 1>>;
};

/// Specialization of Eigen::internal::traits for dart::math::SE3
template <typename S>
struct traits<::Eigen::Map<::dart::math::SE3<S>>> : traits<::dart::math::SE3<S>>
{
  using Base = traits<::dart::math::SE3<S>>;
  using Scalar = typename Base::Scalar;

  // LieGroup common
  static constexpr int ParamSize = Base::ParamSize;
  using Params = typename ::Eigen::Map<::Eigen::Matrix<S, ParamSize, 1>>;
};

} // namespace Eigen::internal

namespace Eigen {

/// Specialization of Eigen::Map for const dart::math::SE3, and also concrete
/// implementation of dart::math::SE3Base.
///
/// @tparam S The scalar type of the underlying quaternion coefficients
/// @tparam Options_ The options of the underlying quaternion coefficients.
/// Pass Eigen::Aligned if the pointer to the underlying data is aligned. That
/// way, Eigen can use vectorized instructions.
template <typename S>
class Map<const ::dart::math::SE3<S>>
  : public ::dart::math::SE3Base<Map<const ::dart::math::SE3<S>>>
{
public:
  using Base = ::dart::math::SE3Base<Map<const ::dart::math::SE3<S>>>;
  using This = Map<const ::dart::math::SE3<S>>;
  using Scalar = typename Base::Scalar;

  // LieGroup common
  using Params = typename Base::Params;

  /// Default constructor
  ///
  /// @param[in] data Pointer to an array of scalars that this Map will use as
  /// its internal data. The array must have a size of at least ParamSize,
  /// which is 4 for SE3.
  explicit Map(const Scalar* data);

  /// Returns the coefficients of the underlying quaternion
  [[nodiscard]] const Params& params() const;

private:
  Params m_params;
};

/// Specialization of Eigen::Map for dart::math::SE3, and also concrete
/// implementation of dart::math::SE3Base.
///
/// @tparam S The scalar type of the underlying quaternion coefficients
/// @tparam Options_ The options of the underlying quaternion coefficients.
/// Pass Eigen::Aligned if the pointer to the underlying data is aligned. That
/// way, Eigen can use vectorized instructions.
template <typename S>
class Map<::dart::math::SE3<S>>
  : public ::dart::math::SE3Base<Map<::dart::math::SE3<S>>>
{
public:
  using Base = ::dart::math::SE3Base<Map<::dart::math::SE3<S>>>;
  using This = Map<::dart::math::SE3<S>>;
  using Scalar = typename Base::Scalar;

  // LieGroup common
  using Params = typename Base::Params;

  using Base::operator=;

  /// Default constructor
  ///
  /// @param[in] data Pointer to an array of scalars that this Map will use as
  /// its internal data. The array must have a size of at least ParamSize,
  /// which is 4 for SE3.
  explicit Map(Scalar* data);

  /// Returns the coefficients of the underlying quaternion
  [[nodiscard]] const Params& params() const;

  /// Returns the coefficients of the underlying quaternion
  [[nodiscard]] Params& params();

private:
  Params m_params;
};

} // namespace Eigen

//==============================================================================
// Implementation
//==============================================================================

namespace Eigen {

//==============================================================================
template <typename S>
Map<const ::dart::math::SE3<S>>::Map(const Scalar* data) : m_params(data)
{
  // Do nothing
}

//==============================================================================
template <typename S>
const typename Map<const ::dart::math::SE3<S>>::Params&
Map<const ::dart::math::SE3<S>>::params() const
{
  return m_params;
}

//==============================================================================
template <typename S>
Map<::dart::math::SE3<S>>::Map(Scalar* data) : m_params(data)
{
  // Do nothing
}

//==============================================================================
template <typename S>
const typename Map<::dart::math::SE3<S>>::Params&
Map<::dart::math::SE3<S>>::params() const
{
  return m_params;
}

//==============================================================================
template <typename S>
typename Map<::dart::math::SE3<S>>::Params& Map<::dart::math::SE3<S>>::params()
{
  return m_params;
}

} // namespace Eigen
