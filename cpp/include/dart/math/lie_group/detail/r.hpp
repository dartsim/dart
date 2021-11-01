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

#include "dart/math/constant.hpp"
#include "dart/math/lie_group/detail/r_base.hpp"
#include "dart/math/type.hpp"

namespace Eigen::internal {

//==============================================================================
template <typename Scalar_, int Dim, int Options_>
struct traits<::dart::math::R<Scalar_, Dim, Options_>>
{
  static constexpr int Options = Options_;
  static constexpr int SpaceDim = Dim;
  static constexpr int GroupDim = Dim;
  static constexpr int MatrixDim = Dim + 1;
  static constexpr int DataDim = Dim;

  DART_LIE_GROUP_TRAITS_TYPES_FOR_R(R)
};

//==============================================================================
template <typename Scalar_, int Options_>
struct traits<::dart::math::R<Scalar_, Eigen::Dynamic, Options_>>
{
  static constexpr int Options = Options_;
  static constexpr int SpaceDim = Eigen::Dynamic;
  static constexpr int GroupDim = Eigen::Dynamic;
  static constexpr int MatrixDim = Eigen::Dynamic;
  static constexpr int DataDim = Eigen::Dynamic;

  DART_LIE_GROUP_TRAITS_TYPES_FOR_R(R)
};

} // namespace Eigen::internal

namespace dart::math {

//==============================================================================
/// Translation group where the dimension is determined in compile-time.
template <typename Scalar_, int N_, int Options_>
class R : public RBase<R<Scalar_, N_, Options_>>
{
public:
  using Base = RBase<R<Scalar_, N_, Options_>>;
  using This = R<Scalar_, N_, Options_>;

  DART_LIE_GROUP_USE_BASE_TYPES

  static R Zero();

  /// Default constructor
  R();

  DART_LIE_GROUP_CONSTRUCTORS(R)

  /// Destructor
  ~R() = default;

  DART_LIE_GROUP_ASSIGN_OPERATORS(R)

  template <typename OtherDerived>
  bool operator!=(const RBase<OtherDerived>& other) const
  {
    return m_data != other.vector();
  }

  R operator-() const;

  R operator+(const R& other) const;

  R operator-(const R& other) const;

  constexpr int dimension() const;

  bool is_identity() const
  {
    return this->is_zero();
  }

  const LieGroupData& coeffs() const
  {
    return m_data;
  }

  LieGroupData& coeffs()
  {
    return m_data;
  }

  using Base::data;

protected:
  using Base::derived;

  LieGroupData m_data;
};

//==============================================================================
/// Translation group where the dimension is determined in runtime.
template <typename Scalar_, int Options_>
class R<Scalar_, Eigen::Dynamic, Options_>
  : public RBase<R<Scalar_, Eigen::Dynamic, Options_>>
{
public:
  using Base = RBase<R<Scalar_, Eigen::Dynamic>>;
  using This = R<Scalar_, Options_>;

  DART_LIE_GROUP_USE_BASE_TYPES

  /// Default constructor
  R();

  /// Constructor
  ///
  /// @param[in] dim: Dimension of this group.
  explicit R(int dim);

  DART_LIE_GROUP_CONSTRUCTORS(R)

  /// Destructor
  ~R() = default;

  DART_LIE_GROUP_ASSIGN_OPERATORS(R)

  //  /// Assign operator
  //  template <typename Derived>
  //  R& operator=(const Eigen::MatrixBase<Derived>& matrix);

  //  /// Move operator
  //  template <typename Derived>
  //  R& operator=(Eigen::MatrixBase<Derived>&& matrix);

  const R& operator+() const;

  R operator-() const;

  int dimension() const;

  //  const Vector& vector() const;

  //  Vector& mutable_vector();

  const LieGroupData& coeffs() const
  {
    return m_data;
  }

  LieGroupData& coeffs()
  {
    return m_data;
  }

  using Base::data;

protected:
  using Base::derived;

  LieGroupData m_data;
};

//==============================================================================
template <typename Scalar, int N, int Options>
R<Scalar, N, Options> R<Scalar, N, Options>::Zero()
{
  return R<Scalar, N, Options>(Eigen::Matrix<Scalar, N, 1, Options>::Zero());
}

//==============================================================================
template <typename Scalar, int N, int Options>
R<Scalar, N, Options>::R() : m_data(LieGroupData::Zero())
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int N, int Options>
R<Scalar, N, Options> R<Scalar, N, Options>::operator-() const
{
  return R<Scalar, N, Options>(-m_data);
}

//==============================================================================
// template <typename Scalar, int N, int Options>
// R<Scalar, N, Options> R<Scalar, N, Options>::operator+(
//    const R<Scalar, N, Options>& other) const
//{
//  return R<Scalar, N, Options>(m_data + other.m_data);
//}

//==============================================================================
// template <typename Scalar, int N, int Options>
// R<Scalar, N, Options> R<Scalar, N, Options>::operator-(
//    const R<Scalar, N, Options>& other) const
//{
//  return R<Scalar, N, Options>(m_data - other.m_data);
//}

//==============================================================================
template <typename Scalar, int N, int Options>
constexpr int R<Scalar, N, Options>::dimension() const
{
  return GroupDim;
}

//==============================================================================
template <typename Scalar, int Options>
R<Scalar, Eigen::Dynamic, Options>::R() : m_data(LieGroupData())
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
R<Scalar, Eigen::Dynamic, Options>::R(int dim) : m_data(LieGroupData::Zero(dim))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
const R<Scalar, Eigen::Dynamic, Options>&
R<Scalar, Eigen::Dynamic, Options>::operator+() const
{
  return *this;
}

//==============================================================================
template <typename Scalar, int Options>
R<Scalar, Eigen::Dynamic, Options>
R<Scalar, Eigen::Dynamic, Options>::operator-() const
{
  return R<Scalar, Eigen::Dynamic, Options>(m_data);
}

//==============================================================================
template <typename Scalar, int Options>
int R<Scalar, Eigen::Dynamic, Options>::dimension() const
{
  return m_data.size();
}

} // namespace dart::math
