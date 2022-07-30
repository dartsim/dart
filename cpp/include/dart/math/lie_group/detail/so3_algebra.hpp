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

#include "dart/math/lie_group/detail/so3_algebra_base.hpp"

namespace Eigen::internal {

//==============================================================================
template <typename Scalar_, int Options_>
struct traits<dart::math::SO3Algebra<Scalar_, Options_>>
{
  static constexpr int Options = Options_;
  static constexpr int SpaceDim = 3;
  static constexpr int GroupDim = 3;
  static constexpr int MatrixDim = 3;
  static constexpr int DataDim = 3;

  DART_LIE_GROUP_TRAITS_TYPES(SO3);

  using DataType = Eigen::Matrix<Scalar, DataDim, DataDim>;
};

} // namespace Eigen::internal

namespace dart::math {

//==============================================================================
template <typename Scalar_, int Options_>
class SO3Algebra : public SO3AlgebraBase<SO3Algebra<Scalar_, Options_>>
{
public:
  using Base = SO3AlgebraBase<SO3Algebra<Scalar_, Options_>>;
  using This = SO3Algebra<Scalar_, Options_>;

  DART_LIE_GROUP_USE_BASE_TYPES;

  using Quaternion = typename Base::Quaternion;
  using QuaternionMap = typename Base::QuaternionMap;
  using ConstQuaternionMap = typename Base::ConstQuaternionMap;

  /// Default constructor
  SO3Algebra() : m_data(DataType::Zero())
  {
    // Do nothing
  }

  DART_LIE_GROUP_CONSTRUCTORS(SO3Algebra);

  /** Copy constructor from LieAlgebraBase */
  template <typename OtherDerived>
  SO3Algebra(const LieAlgebraBase<OtherDerived>& other) : m_data(other.coeffs())
  {
    /* Do nothing */
  }

  /** Move constructor from LieAlgebraBase */
  template <typename OtherDerived>
  SO3Algebra(LieAlgebraBase<OtherDerived>&& other)
    : m_data(std::move(other.coeffs()))
  {
    /* Do nothing */
  }

  const DataType& coeffs() const
  {
    return m_data;
  }
  // TODO(JS): Rename to matrix()

  DataType& coeffs()
  {
    return m_data;
  }

  using Base::data;

private:
  DataType m_data;
};

DART_TEMPLATE_CLASS_SCALAR(SO3Algebra)

} // namespace dart::math
