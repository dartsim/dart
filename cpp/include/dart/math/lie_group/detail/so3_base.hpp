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

#include <Eigen/Geometry>

#include "dart/math/lie_group/detail/lie_group_base.hpp"
#include "dart/math/lie_group/type.hpp"
#include "dart/math/type.hpp"

namespace dart::math {

//==============================================================================
template <typename Derived>
class SO3Base : public LieGroupBase<Derived>
{
public:
  using This = SO3Base<Derived>;
  using Base = LieGroupBase<Derived>;

  DART_LIE_GROUP_USE_BASE_TYPES

  using Quaternion = math::Quaternion<Scalar, Options>;
  using QuaternionMap = Eigen::Map<Quaternion>;
  using ConstQuaternionMap = Eigen::Map<const Quaternion>;

  using Rotation = Eigen::Matrix<Scalar, 3, 3>;

  using Translation = Eigen::Matrix<Scalar, 3, 1>;
  using TranslationMap = Eigen::Map<Translation>;
  using ConstTranslationMap = Eigen::Map<const Translation>;

  using Transformation
      = Eigen::Transform<Scalar, SpaceDim, Eigen::Isometry, Options>;

  DART_LIE_GROUP_BASE_ASSIGN_OPERATORS(SO3Base)

  /// @{ @name Group operations

  [[nodiscard]] LieGroup inverse() const
  {
    return LieGroup(quaternion().conjugate());
  }

  Derived& inverse_in_place()
  {
    quaternion() = quaternion().conjugate();
    return derived();
  }

  template <typename OtherDerived>
  [[nodiscard]] LieGroup operator*(const SO3Base<OtherDerived>& other) const
  {
    return LieGroup(quaternion() * other.quaternion());
  }

  template <typename RDerived>
  [[nodiscard]] typename RDerived::LieGroup operator*(
      const RBase<RDerived>& vector) const
  {
    // TODO(JS): Check if the dimension of vector is (3x1)
    return typename RDerived::LieGroup(quaternion() * vector.coeffs());
  }

  /// @}

  void normalize()
  {
    if (coeffs().w() < 0) {
      coeffs() *= -1;
    }
    coeffs().normalize();
  }

  using Base::coeffs;
  using Base::data;

  Quaternion to_quaternion() const;

  Rotation to_rotation_matrix() const;

protected:
  ConstQuaternionMap quaternion() const;

  QuaternionMap quaternion();

  using Base::derived;
};

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Quaternion SO3Base<Derived>::to_quaternion() const
{
  return Quaternion(coeffs());
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Rotation SO3Base<Derived>::to_rotation_matrix() const
{
  return to_quaternion().toRotationMatrix();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::ConstQuaternionMap SO3Base<Derived>::quaternion()
    const
{
  return ConstQuaternionMap(data());
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::QuaternionMap SO3Base<Derived>::quaternion()
{
  return QuaternionMap(data());
}

namespace detail {

//==============================================================================
template <typename Derived>
struct NormalizationOperator<SO3Base<Derived>>
{
  template <typename T>
  static void run(T& x)
  {
    x.normalize();
  }
};

//==============================================================================
template <typename Derived>
struct RandomSetter<SO3Base<Derived>>
{
  template <typename T>
  static void run(T& x)
  {
    using Scalar = typename SO3Base<Derived>::Scalar;
    using LieGroup = typename SO3Base<Derived>::LieGroup;

    x = LieGroup(Eigen::Quaternion<Scalar>::UnitRandom());
  }
};

} // namespace detail

} // namespace dart::math
