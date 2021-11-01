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

#include "dart/common/macro.hpp"
#include "dart/math/constant.hpp"
#include "dart/math/export.hpp"
#include "dart/math/lie_group/detail/so3_base.hpp"
#include "dart/math/lie_group/detail/so3_tangent.hpp"
#include "dart/math/lie_group/r.hpp"
#include "dart/math/type.hpp"

namespace Eigen::internal {

//==============================================================================
template <typename Scalar_, int Options_>
struct traits<dart::math::SO3<Scalar_, Options_>>
{
  static constexpr int Options = Options_;
  static constexpr int SpaceDim = 3;
  static constexpr int GroupDim = 3;
  static constexpr int MatrixDim = 3;
  static constexpr int DataDim = 4;

  DART_LIE_GROUP_TRAITS_TYPES(SO3)
};

} // namespace Eigen::internal

namespace dart::math {

//==============================================================================
template <typename Scalar_, int Options_>
class SO3 : public SO3Base<SO3<Scalar_, Options_>>
{
public:
  using Base = SO3Base<SO3<Scalar_, Options_>>;
  using This = SO3<Scalar_, Options_>;

  DART_LIE_GROUP_USE_BASE_TYPES

  using Quaternion = typename Base::Quaternion;
  using QuaternionMap = typename Base::QuaternionMap;
  using ConstQuaternionMap = typename Base::ConstQuaternionMap;

  using Rotation = typename Base::Rotation;

  using Translation = typename Base::Translation;
  using TranslationMap = typename Base::TranslationMap;
  using ConstTranslationMap = typename Base::ConstTranslationMap;

  using Transformation = typename Base::Transformation;

  /// Default constructor
  SO3();

  DART_LIE_GROUP_CONSTRUCTORS(SO3)

  /// Constructs from quaternion
  template <typename QuatDerived>
  SO3(const Eigen::QuaternionBase<QuatDerived>& quat);

  /// Constructs from quaternion
  template <typename QuatDerived>
  SO3(Eigen::QuaternionBase<QuatDerived>&& quat);

  /// Destructor
  ~SO3() = default;

  DART_LIE_GROUP_ASSIGN_OPERATORS(SO3)

  /// @{ @name Group operation

  DART_LIE_GROUP_USE_BASE_GROUP_OPERATIONS

  /// @}

  using Base::normalize;

  using Base::to_quaternion;
  using Base::to_rotation_matrix;

  const LieGroupData& coeffs() const;

  LieGroupData& coeffs();

  using Base::data;

protected:
  using Base::derived;
  using Base::quaternion;

  LieGroupData m_data;
};

DART_TEMPLATE_CLASS_SCALAR(SO3)

//==============================================================================
template <typename Scalar, int Options>
SO3<Scalar, Options>::SO3() : m_data(Quaternion::Identity().coeffs())
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
template <typename QuatDerived>
SO3<Scalar, Options>::SO3(const Eigen::QuaternionBase<QuatDerived>& quat)
  : m_data(quat.coeffs())
{
  normalize();
}

//==============================================================================
template <typename Scalar, int Options>
template <typename QuatDerived>
SO3<Scalar, Options>::SO3(Eigen::QuaternionBase<QuatDerived>&& quat)
  : m_data(std::move(quat.coeffs()))
{
  normalize();
}

//==============================================================================
template <typename Scalar, int Options>
const typename SO3<Scalar, Options>::LieGroupData&
SO3<Scalar, Options>::coeffs() const
{
  return m_data;
}

//==============================================================================
template <typename Scalar, int Options>
typename SO3<Scalar, Options>::LieGroupData& SO3<Scalar, Options>::coeffs()
{
  return m_data;
}

} // namespace dart::math
