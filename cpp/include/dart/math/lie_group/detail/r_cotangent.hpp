/*
 * Copyright (c) 2011-2022, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "dart/math/lie_group/detail/r_cotangent_base.hpp"

namespace Eigen::internal {

//==============================================================================
template <typename Scalar_, int Dim, int Options_>
struct traits<::dart::math::RCotangent<Scalar_, Dim, Options_>>
{
  static constexpr int Options = Options_;
  static constexpr int SpaceDim = Dim;
  static constexpr int GroupDim = Dim;
  static constexpr int MatrixDim = GroupDim + 1;
  static constexpr int DataDim = Dim;

  DART_LIE_GROUP_TRAITS_TYPES(R);

  using DataType = Eigen::Matrix<Scalar, 1, DataDim>;
};

//==============================================================================
template <typename Scalar_, int Options_>
struct traits<::dart::math::RCotangent<Scalar_, Eigen::Dynamic, Options_>>
{
  static constexpr int Options = Options_;
  static constexpr int SpaceDim = 3;
  static constexpr int GroupDim = Eigen::Dynamic;
  static constexpr int MatrixDim = Eigen::Dynamic;
  static constexpr int DataDim = Eigen::Dynamic;

  DART_LIE_GROUP_TRAITS_TYPES(R);

  using DataType = Eigen::Matrix<Scalar, 1, DataDim>;
};

} // namespace Eigen::internal

namespace dart::math {

//==============================================================================
template <typename Scalar_, int Dim, int Options_>
class RCotangent : public RCotangentBase<RCotangent<Scalar_, Dim, Options_>>
{
public:
  using This = RCotangent<Scalar_, Dim, Options_>;
  using Base = RCotangentBase<RCotangent<Scalar_, Dim, Options_>>;

  DART_LIE_GROUP_USE_BASE_TYPES;

  /// Default constructor
  RCotangent() : m_data(DataType::Zero())
  {
    // Do nothing
  }

  DART_LIE_GROUP_CONSTRUCTORS(RCotangent);

  const DataType& coeffs() const
  {
    return m_data;
  }

  DataType& coeffs()
  {
    return m_data;
  }

  using Base::data;

private:
  DataType m_data;
};

//==============================================================================
template <typename Scalar_, int Options_>
class RCotangent<Scalar_, Eigen::Dynamic, Options_>
  : public RCotangentBase<RCotangent<Scalar_, Eigen::Dynamic, Options_>>
{
public:
  using This = RCotangent<Scalar_, Eigen::Dynamic, Options_>;
  using Base = RCotangentBase<RCotangent<Scalar_, Eigen::Dynamic, Options_>>;

  DART_LIE_GROUP_USE_BASE_TYPES;

  /// Constructor
  explicit RCotangent(int size = 0) : m_data(DataType::Zero(size))
  {
    // Do nothing
  }

  DART_LIE_GROUP_CONSTRUCTORS(RCotangent);

  const DataType& coeffs() const
  {
    return m_data;
  }

  DataType& coeffs()
  {
    return m_data;
  }

  using Base::data;

private:
  DataType m_data;
};

} // namespace dart::math
