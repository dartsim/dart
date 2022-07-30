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

#include "dart/common/macro.hpp"
#include "dart/math/constant.hpp"
#include "dart/math/lie_group/detail/cast.hpp"
#include "dart/math/lie_group/detail/macro.hpp"
#include "dart/math/lie_group/detail/normalization.hpp"
#include "dart/math/lie_group/detail/random_setter.hpp"
#include "dart/math/type.hpp"

namespace dart::math {

//==============================================================================
/// The base class for matrix Lie groups.
template <typename Derived>
class LieGroupBase
{
public:
  DART_LIE_GROUP_BASE_TYPES;

  template <typename OtherScalar>
  using ScalarCastType =
      typename detail::ScalarCast<LieGroup, OtherScalar>::type;

  /// Creates an identity element
  [[nodiscard]] static LieGroup Identity();

  /// Creates a random element
  [[nodiscard]] static LieGroup Random();

protected:
  /// Default constructor
  LieGroupBase() = default;

  /// Destructor
  ~LieGroupBase() = default;

public:
  DART_LIE_GROUP_BASE_ASSIGN_OPERATORS(LieGroupBase);

  template <typename OtherDerived>
  [[nodiscard]] bool operator==(const LieGroupBase<OtherDerived>& other) const;

  template <typename OtherDerived>
  [[nodiscard]] LieGroup operator*(
      const LieGroupBase<OtherDerived>& other) const;

  [[nodiscard]] Scalar operator[](int i) const;

  [[nodiscard]] Scalar& operator[](int i);

  void set_identity();

  void set_random();

  template <typename OtherDerived>
  [[nodiscard]] bool is_approx(
      const LieGroupBase<OtherDerived>& other,
      const Scalar tolerance = eps<Scalar>()) const;

  [[nodiscard]] bool is_identity(const Scalar tolerance = eps<Scalar>()) const;

  [[nodiscard]] Derived inverse() const;

  Derived& inverse_in_place();

  template <typename OtherDerived>
  [[nodiscard]] Tangent difference(
      const LieGroupBase<OtherDerived>& other) const
  {
    DART_UNUSED(other);
    DART_NOT_IMPLEMENTED;
    return Tangent();
  }

  DART_LIE_GROUP_BASE_DATA(DataType);

  /// Creates the group element with a different scalar type
  template <typename OtherScalar>
  [[nodiscard]] ScalarCastType<OtherScalar> cast() const;

protected:
  DART_LIE_GROUP_BASE_DERIVED;
};

//==============================================================================
template <typename Derived>
typename LieGroupBase<Derived>::LieGroup LieGroupBase<Derived>::Identity()
{
  // Assumed the default constructor creates the identity element
  const static LieGroup identity;
  return identity;
}

//==============================================================================
template <typename Derived>
typename LieGroupBase<Derived>::LieGroup LieGroupBase<Derived>::Random()
{
  auto out = LieGroup();
  out.set_random();
  return out;
}

//==============================================================================
template <typename Derived>
template <typename OtherDerived>
bool LieGroupBase<Derived>::operator==(
    const LieGroupBase<OtherDerived>& other) const
{
  return is_approx(other);
}

//==============================================================================
template <typename Derived>
template <typename OtherDerived>
typename LieGroupBase<Derived>::LieGroup LieGroupBase<Derived>::operator*(
    const LieGroupBase<OtherDerived>& other) const
{
  return derived() * other.derived();
}

//==============================================================================
template <typename Derived>
typename LieGroupBase<Derived>::Scalar LieGroupBase<Derived>::operator[](
    int i) const
{
  return coeffs()[i];
}

//==============================================================================
template <typename Derived>
typename LieGroupBase<Derived>::Scalar& LieGroupBase<Derived>::operator[](int i)
{
  return coeffs()[i];
}

//==============================================================================
template <typename Derived>
void LieGroupBase<Derived>::set_identity()
{
  // Assumed the default constructor creates the identity element
  derived() = LieGroup();
}

//==============================================================================
template <typename Derived>
void LieGroupBase<Derived>::set_random()
{
  detail::RandomSetter<typename Derived::Base>::run(derived());
  detail::NormalizationOperator<typename Derived::Base>::run(derived());
}

//==============================================================================
template <typename Derived>
template <typename OtherDerived>
bool LieGroupBase<Derived>::is_approx(
    const LieGroupBase<OtherDerived>& other, const Scalar tolerance) const
{
  return (other.inverse() * derived()).log().is_zero(tolerance);
}

//==============================================================================
template <typename Derived>
bool LieGroupBase<Derived>::is_identity(const Scalar tolerance) const
{
  return is_approx(LieGroup::Identity(), tolerance);
}

//==============================================================================
template <typename Derived>
Derived LieGroupBase<Derived>::inverse() const
{
  return derived().inverse();
}

//==============================================================================
template <typename Derived>
Derived& LieGroupBase<Derived>::inverse_in_place()
{
  return derived().inverse_in_place();
}

//==============================================================================
template <typename Derived>
template <typename OtherScalar>
typename LieGroupBase<Derived>::template ScalarCastType<OtherScalar>
LieGroupBase<Derived>::cast() const
{
  return ScalarCastType<OtherScalar>(coeffs().template cast<OtherScalar>());
}

} // namespace dart::math
