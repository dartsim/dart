/*
 * Copyright (c) 2011-2021, The DART development contributors:
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

#include "dart/math/lie_group/detail/r_tangent_base.hpp"

namespace Eigen::internal {

//==============================================================================
template <typename Scalar_, int Dim, int Options_>
struct traits<::dart::math::RTangent<Scalar_, Dim, Options_>>
{
  static constexpr int Options = Options_;
  static constexpr int SpaceDim = Dim;
  static constexpr int GroupDim = Dim;
  static constexpr int MatrixDim = GroupDim + 1;
  static constexpr int DataDim = Dim;

  DART_LIE_GROUP_TRAITS_TYPES_FOR_R(R);

  using DataType = Eigen::Matrix<Scalar, DataDim, 1>;
};

//==============================================================================
template <typename Scalar_, int Options_>
struct traits<::dart::math::RTangent<Scalar_, Eigen::Dynamic, Options_>>
{
  static constexpr int Options = Options_;
  static constexpr int SpaceDim = Eigen::Dynamic;
  static constexpr int GroupDim = Eigen::Dynamic;
  static constexpr int MatrixDim = Eigen::Dynamic;
  static constexpr int DataDim = Eigen::Dynamic;

  DART_LIE_GROUP_TRAITS_TYPES_FOR_R(R);

  using DataType = Eigen::Matrix<Scalar, DataDim, 1>;
};

} // namespace Eigen::internal

namespace dart::math {

//==============================================================================
template <typename Scalar_, int Dim, int Options_>
class RTangent : public RTangentBase<RTangent<Scalar_, Dim, Options_>>
{
public:
  using This = RTangent<Scalar_, Dim, Options_>;
  using Base = RTangentBase<RTangent<Scalar_, Dim, Options_>>;

  DART_LIE_GROUP_USE_BASE_TYPES;

  /// Default constructor
  RTangent() : m_data(DataType::Zero())
  {
    // Do nothing
  }

  DART_LIE_GROUP_CONSTRUCTORS(RTangent);

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
class RTangent<Scalar_, Eigen::Dynamic, Options_>
  : public RTangentBase<RTangent<Scalar_, Eigen::Dynamic, Options_>>
{
public:
  using This = RTangent<Scalar_, Eigen::Dynamic, Options_>;
  using Base = RTangentBase<RTangent<Scalar_, Eigen::Dynamic, Options_>>;

  DART_LIE_GROUP_USE_BASE_TYPES;

  /// Constructor
  explicit RTangent(int size = 0) : m_data(DataType::Zero(size))
  {
    // Do nothing
  }

  DART_LIE_GROUP_CONSTRUCTORS(RTangent);

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
