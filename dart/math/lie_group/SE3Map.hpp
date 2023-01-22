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
template <typename S, int Options_>
struct traits<::Eigen::Map<const ::dart::math::SE3<S, Options_>>>
  : traits<const ::dart::math::SE3<S, Options_>>
{
  using Base = traits<const ::dart::math::SE3<S, Options_>>;

  static constexpr int Options = Options_;
  static constexpr int CoeffsDim = Base::CoeffsDim;

  using Scalar = typename Base::Scalar;
  using Coeffs =
      typename ::Eigen::Map<const ::Eigen::Matrix<S, CoeffsDim, 1>, Options>;
};

/// Specialization of Eigen::internal::traits for dart::math::SE3
template <typename S, int Options_>
struct traits<::Eigen::Map<::dart::math::SE3<S, Options_>>>
  : traits<::dart::math::SE3<S, Options_>>
{
  using Base = traits<::dart::math::SE3<S, Options_>>;

  static constexpr int Options = Options_;
  static constexpr int CoeffsDim = Base::CoeffsDim;

  using Scalar = typename Base::Scalar;
  using Coeffs =
      typename ::Eigen::Map<::Eigen::Matrix<S, CoeffsDim, 1>, Options>;
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
template <typename S, int Options_>
class Map<const ::dart::math::SE3<S, Options_>, Options_>
  : public ::dart::math::SE3Base<
        Map<const ::dart::math::SE3<S, Options_>, Options_>>
{
public:
  using Base = ::dart::math::SE3Base<
      Map<const ::dart::math::SE3<S, Options_>, Options_>>;
  using This = Map<const ::dart::math::SE3<S, Options_>, Options_>;

  // LieGroupBase types
  using Scalar = typename Base::Scalar;
  using Coeffs = typename Base::Coeffs;

  /// Default constructor
  ///
  /// @param[in] data Pointer to an array of scalars that this Map will use as
  /// its internal data. The array must have a size of at least CoeffsDim,
  /// which is 4 for SE3.
  explicit Map(const Scalar* data);

  /// Returns the coefficients of the underlying quaternion
  [[nodiscard]] const Coeffs& coeffs() const;

private:
  Coeffs m_coeffs;
};

/// Specialization of Eigen::Map for dart::math::SE3, and also concrete
/// implementation of dart::math::SE3Base.
///
/// @tparam S The scalar type of the underlying quaternion coefficients
/// @tparam Options_ The options of the underlying quaternion coefficients.
/// Pass Eigen::Aligned if the pointer to the underlying data is aligned. That
/// way, Eigen can use vectorized instructions.
template <typename S, int Options_>
class Map<::dart::math::SE3<S, Options_>, Options_>
  : public ::dart::math::SE3Base<Map<::dart::math::SE3<S, Options_>, Options_>>
{
public:
  using Base
      = ::dart::math::SE3Base<Map<::dart::math::SE3<S, Options_>, Options_>>;
  using This = Map<::dart::math::SE3<S, Options_>, Options_>;

  // LieGroupBase types
  using Scalar = typename Base::Scalar;
  using Coeffs = typename Base::Coeffs;

  using Base::operator=;

  /// Default constructor
  ///
  /// @param[in] data Pointer to an array of scalars that this Map will use as
  /// its internal data. The array must have a size of at least CoeffsDim,
  /// which is 4 for SE3.
  explicit Map(Scalar* data);

  /// Returns the coefficients of the underlying quaternion
  [[nodiscard]] const Coeffs& coeffs() const;

  /// Returns the coefficients of the underlying quaternion
  [[nodiscard]] Coeffs& coeffs();

private:
  Coeffs m_coeffs;
};

} // namespace Eigen

//==============================================================================
// Implementation
//==============================================================================

namespace Eigen {

//==============================================================================
template <typename S, int Options>
Map<const ::dart::math::SE3<S, Options>, Options>::Map(const Scalar* data)
  : m_coeffs(data)
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
const typename Map<const ::dart::math::SE3<S, Options>, Options>::Coeffs&
Map<const ::dart::math::SE3<S, Options>, Options>::coeffs() const
{
  return m_coeffs;
}

//==============================================================================
template <typename S, int Options>
Map<::dart::math::SE3<S, Options>, Options>::Map(Scalar* data) : m_coeffs(data)
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
const typename Map<::dart::math::SE3<S, Options>, Options>::Coeffs&
Map<::dart::math::SE3<S, Options>, Options>::coeffs() const
{
  return m_coeffs;
}

//==============================================================================
template <typename S, int Options>
typename Map<::dart::math::SE3<S, Options>, Options>::Coeffs&
Map<::dart::math::SE3<S, Options>, Options>::coeffs()
{
  return m_coeffs;
}

} // namespace Eigen
