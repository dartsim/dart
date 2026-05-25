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

#include <dart/math/lie_group/rn.hpp>

namespace Eigen::internal {

/// Specialization of Eigen::internal::traits for const dart::math::Rn.
template <typename S, int N, int Options, typename StrideType>
struct traits<::Eigen::Map<const ::dart::math::Rn<S, N>, Options, StrideType>>
  : traits<const ::dart::math::Rn<S, N>>
{
  using Base = traits<const ::dart::math::Rn<S, N>>;
  using Scalar = typename Base::Scalar;

  // LieGroup common
  static constexpr int ParamSize = Base::ParamSize;
  using Params = typename ::Eigen::
      Map<const ::Eigen::Matrix<S, ParamSize, 1>, Options, StrideType>;
};

/// Specialization of Eigen::internal::traits for dart::math::Rn.
template <typename S, int N, int Options, typename StrideType>
struct traits<::Eigen::Map<::dart::math::Rn<S, N>, Options, StrideType>>
  : traits<::dart::math::Rn<S, N>>
{
  using Base = traits<::dart::math::Rn<S, N>>;
  using Scalar = typename Base::Scalar;

  // LieGroup common
  static constexpr int ParamSize = Base::ParamSize;
  using Params = typename ::Eigen::
      Map<::Eigen::Matrix<S, ParamSize, 1>, Options, StrideType>;
};

} // namespace Eigen::internal

namespace Eigen {

/// Specialization of Eigen::Map for const dart::math::Rn.
template <typename S, int N, int Options, typename StrideType>
class Map<const ::dart::math::Rn<S, N>, Options, StrideType>
  : public ::dart::math::RnBase<
        Map<const ::dart::math::Rn<S, N>, Options, StrideType>>
{
public:
  using Base = ::dart::math::RnBase<
      Map<const ::dart::math::Rn<S, N>, Options, StrideType>>;
  using Scalar = typename Base::Scalar;

  // LieGroup common
  using Params = typename Base::Params;

  /// Constructs a const Rn map from raw scalar storage.
  explicit Map(const Scalar* data);

  /// Constructs a const Rn map with an explicit stride.
  Map(const Scalar* data, const StrideType& stride);

  /// Returns the mapped vector parameters.
  [[nodiscard]] const Params& params() const;

private:
  Params m_params;
};

/// Specialization of Eigen::Map for dart::math::Rn.
template <typename S, int N, int Options, typename StrideType>
class Map<::dart::math::Rn<S, N>, Options, StrideType>
  : public ::dart::math::RnBase<
        Map<::dart::math::Rn<S, N>, Options, StrideType>>
{
public:
  using Base
      = ::dart::math::RnBase<Map<::dart::math::Rn<S, N>, Options, StrideType>>;
  using Scalar = typename Base::Scalar;

  // LieGroup common
  using Params = typename Base::Params;

  using Base::operator=;

  /// Constructs an Rn map from raw scalar storage.
  explicit Map(Scalar* data);

  /// Constructs an Rn map with an explicit stride.
  Map(Scalar* data, const StrideType& stride);

  /// Returns the mapped vector parameters.
  [[nodiscard]] const Params& params() const;

  /// Returns the mapped vector parameters.
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
template <typename S, int N, int Options, typename StrideType>
Map<const ::dart::math::Rn<S, N>, Options, StrideType>::Map(const Scalar* data)
  : m_params(data)
{
  // Do nothing
}

//==============================================================================
template <typename S, int N, int Options, typename StrideType>
Map<const ::dart::math::Rn<S, N>, Options, StrideType>::Map(
    const Scalar* data, const StrideType& stride)
  : m_params(data, stride)
{
  // Do nothing
}

//==============================================================================
template <typename S, int N, int Options, typename StrideType>
const typename Map<const ::dart::math::Rn<S, N>, Options, StrideType>::Params&
Map<const ::dart::math::Rn<S, N>, Options, StrideType>::params() const
{
  return m_params;
}

//==============================================================================
template <typename S, int N, int Options, typename StrideType>
Map<::dart::math::Rn<S, N>, Options, StrideType>::Map(Scalar* data)
  : m_params(data)
{
  // Do nothing
}

//==============================================================================
template <typename S, int N, int Options, typename StrideType>
Map<::dart::math::Rn<S, N>, Options, StrideType>::Map(
    Scalar* data, const StrideType& stride)
  : m_params(data, stride)
{
  // Do nothing
}

//==============================================================================
template <typename S, int N, int Options, typename StrideType>
const typename Map<::dart::math::Rn<S, N>, Options, StrideType>::Params&
Map<::dart::math::Rn<S, N>, Options, StrideType>::params() const
{
  return m_params;
}

//==============================================================================
template <typename S, int N, int Options, typename StrideType>
typename Map<::dart::math::Rn<S, N>, Options, StrideType>::Params&
Map<::dart::math::Rn<S, N>, Options, StrideType>::params()
{
  return m_params;
}

} // namespace Eigen
