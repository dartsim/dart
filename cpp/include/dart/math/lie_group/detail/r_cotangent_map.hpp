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

#include "dart/math/lie_group/detail/r_tangent.hpp"

namespace Eigen {

namespace internal {

//==============================================================================
template <typename Scalar_, int Dim, int Options_>
struct traits<Eigen::Map<dart::math::RCotangent<Scalar_, Dim, Options_>>>
  : traits<dart::math::RCotangent<Scalar_, Dim, Options_>>
{
  using Base = traits<dart::math::RCotangent<Scalar_, Dim, Options_>>;

  using Base::DataDim;
  using Base::Options;
  using typename Base::Scalar;
  using DataType = Eigen::Map<Eigen::Matrix<Scalar, 1, DataDim>, Options>;
};

//==============================================================================
template <typename Scalar_, int Dim, int Options_>
struct traits<Eigen::Map<const dart::math::RCotangent<Scalar_, Dim, Options_>>>
  : traits<const dart::math::RCotangent<Scalar_, Dim, Options_>>
{
  using Base = traits<dart::math::RCotangent<Scalar_, Dim, Options_>>;

  using Base::DataDim;
  using Base::Options;
  using typename Base::Scalar;
  using DataType = Eigen::Map<const Eigen::Matrix<Scalar, 1, DataDim>, Options>;
};

//==============================================================================
template <typename Scalar_, int Options_>
struct traits<
    Eigen::Map<dart::math::RCotangent<Scalar_, Eigen::Dynamic, Options_>>>
  : traits<dart::math::RCotangent<Scalar_, Eigen::Dynamic, Options_>>
{
  using Base
      = traits<dart::math::RCotangent<Scalar_, Eigen::Dynamic, Options_>>;

  using Base::DataDim;
  using Base::Options;
  using typename Base::Scalar;
  using DataType = Eigen::Map<Eigen::Matrix<Scalar, 1, DataDim>, Options>;
};

//==============================================================================
template <typename Scalar_, int Options_>
struct traits<
    Eigen::Map<const dart::math::RCotangent<Scalar_, Eigen::Dynamic, Options_>>>
  : traits<const dart::math::RCotangent<Scalar_, Eigen::Dynamic, Options_>>
{
  using Base
      = traits<dart::math::RCotangent<Scalar_, Eigen::Dynamic, Options_>>;

  using Base::DataDim;
  using Base::Options;
  using typename Base::Scalar;
  using DataType = Eigen::Map<const Eigen::Matrix<Scalar, 1, DataDim>, Options>;
};

} // namespace internal

//==============================================================================
template <typename Scalar_, int Dim, int Options_>
class Map<dart::math::RCotangent<Scalar_, Dim, Options_>, Options_>
  : public dart::math::RCotangentBase<
        Map<dart::math::RCotangent<Scalar_, Dim, Options_>, Options_>>
{
public:
  using Base = dart::math::RCotangentBase<
      Map<dart::math::RCotangent<Scalar_, Dim, Options_>, Options_>>;
  using This = Map<dart::math::RCotangent<Scalar_, Dim, Options_>, Options_>;

  DART_LIE_GROUP_USE_BASE_TYPES;

  /// Constructor
  explicit Map(Scalar* data) : m_data(data)
  {
    // Do nohting
  }

  DART_LIE_GROUP_MAP_ASSIGN_OPERATORS(RCotangent);

  [[nodiscard]] DataType& coeffs()
  {
    return m_data;
  }

  [[nodiscard]] const DataType& coeffs() const
  {
    return m_data;
  }

private:
  DataType m_data;
};

//==============================================================================
template <typename Scalar_, int Dim, int Options_>
class Map<const dart::math::RCotangent<Scalar_, Dim, Options_>, Options_>
  : public dart::math::RCotangentBase<
        Map<const dart::math::RCotangent<Scalar_, Dim, Options_>, Options_>>
{
public:
  using Base = dart::math::RCotangentBase<
      Map<const dart::math::RCotangent<Scalar_, Dim, Options_>, Options_>>;
  using This
      = Map<const dart::math::RCotangent<Scalar_, Dim, Options_>, Options_>;

  DART_LIE_GROUP_USE_BASE_TYPES;

  /// Constructor
  explicit Map(const Scalar* data) : m_data(data)
  {
    // Do nohting
  }

  [[nodiscard]] const DataType& coeffs() const
  {
    return m_data;
  }

private:
  DataType m_data;
};

//==============================================================================
template <typename Scalar_, int Options_>
class Map<dart::math::RCotangent<Scalar_, Eigen::Dynamic, Options_>, Options_>
  : public dart::math::RCotangentBase<
        Map<dart::math::RCotangent<Scalar_, Eigen::Dynamic, Options_>,
            Options_>>
{
public:
  using Base = dart::math::RCotangentBase<
      Map<dart::math::RCotangent<Scalar_, Eigen::Dynamic, Options_>, Options_>>;
  using This = Map<
      dart::math::RCotangent<Scalar_, Eigen::Dynamic, Options_>,
      Options_>;

  DART_LIE_GROUP_USE_BASE_TYPES;

  /// Constructor
  explicit Map(Scalar* data) : m_data(data)
  {
    // Do nohting
  }

  DART_LIE_GROUP_MAP_ASSIGN_OPERATORS(RCotangent);

  [[nodiscard]] DataType& coeffs()
  {
    return m_data;
  }

  [[nodiscard]] const DataType& coeffs() const
  {
    return m_data;
  }

private:
  DataType m_data;
};

//==============================================================================
template <typename Scalar_, int Options_>
class Map<
    const dart::math::RCotangent<Scalar_, Eigen::Dynamic, Options_>,
    Options_>
  : public dart::math::RCotangentBase<
        Map<const dart::math::RCotangent<Scalar_, Eigen::Dynamic, Options_>,
            Options_>>
{
public:
  using Base = dart::math::RCotangentBase<
      Map<const dart::math::RCotangent<Scalar_, Eigen::Dynamic, Options_>,
          Options_>>;
  using This = Map<
      const dart::math::RCotangent<Scalar_, Eigen::Dynamic, Options_>,
      Options_>;

  DART_LIE_GROUP_USE_BASE_TYPES;

  /// Constructor
  explicit Map(const Scalar* data) : m_data(data)
  {
    // Do nohting
  }

  [[nodiscard]] const DataType& coeffs() const
  {
    return m_data;
  }

private:
  DataType m_data;
};

} // namespace Eigen
