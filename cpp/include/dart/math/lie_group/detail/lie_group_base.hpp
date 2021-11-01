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

#include "dart/math/lie_group/detail/cast.hpp"
#include "dart/math/lie_group/detail/normalization.hpp"
#include "dart/math/lie_group/detail/random_setter.hpp"
#include "dart/math/lie_group/type.hpp"
#include "dart/math/type.hpp"

namespace dart::math {

//==============================================================================
/// The base class for matrix Lie groups.
template <typename Derived>
class LieGroupBase
{
public:
  DART_LIE_GROUP_BASE_TYPES

  template <typename OtherScalar>
  using ScalarCastType =
      typename detail::ScalarCast<LieGroup, OtherScalar>::type;

  /// Creates an identity element
  static LieGroup Identity();

  /// Creates a random element
  static LieGroup Random();

protected:
  /// Default constructor
  LieGroupBase() = default;

  /// Destructor
  ~LieGroupBase() = default;

public:
  DART_LIE_GROUP_BASE_ASSIGN_OPERATORS(LieGroupBase)

  Derived& set_identity();

  Derived& set_random();

  [[nodiscard]] Derived inverse() const
  {
    return derived().inverse();
  }

  Derived& inverse_in_place()
  {
    return derived().inverse_in_place();
  }

  // DART_LIE_GROUP_BASE_DATA(LieGroupData)
  /** Returns the data of the drived class */
  [[nodiscard]] const LieGroupData& coeffs() const
  {
    return derived().coeffs();
  }

  /** Returns the mutable data of the drived class */
  [[nodiscard]] LieGroupData& coeffs()
  {
    return derived().coeffs();
  }

  /** Returns the pointer to the data of the drived class */
  [[nodiscard]] const Scalar* data() const
  {
    return coeffs().data();
  }

  /** Returns the pointer to the mutable data of the drived class */
  [[nodiscard]] Scalar* data()
  {
    return coeffs().data();
  }

  /// Creates the group element with a different scalar type
  template <typename OtherScalar>
  [[nodiscard]] ScalarCastType<OtherScalar> cast() const;

protected:
  DART_LIE_GROUP_BASE_DERIVED
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
  return LieGroup().set_random();
}

//==============================================================================
template <typename Derived>
Derived& LieGroupBase<Derived>::set_identity()
{
  // Assumed the default constructor creates the identity element
  derived() = LieGroup();
  return derived();
}

//==============================================================================
template <typename Derived>
Derived& LieGroupBase<Derived>::set_random()
{
  detail::RandomSetter<typename Derived::Base>::run(derived());
  detail::NormalizationOperator<typename Derived::Base>::run(derived());
  return derived();
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
