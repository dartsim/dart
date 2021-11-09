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

#include "dart/math/lie_group/type.hpp"
#include "dart/math/type.hpp"

namespace dart::math {

//==============================================================================
template <typename Derived>
class CotangentBase
{
public:
  DART_LIE_GROUP_BASE_TYPES;

  /// Creates an identity element
  [[nodiscard]] static Cotangent Identity();

  /// Creates a random element
  [[nodiscard]] static Cotangent Random();

protected:
  /// Default constructor
  CotangentBase() = default;

  /// Destructor
  ~CotangentBase() = default;

public:
  DART_LIE_GROUP_BASE_ASSIGN_OPERATORS(CotangentBase)

  DART_LIE_GROUP_BASE_DATA(CotangentData)

  Derived& set_identity();

  Derived& set_random();

protected:
  DART_LIE_GROUP_BASE_DERIVED
};

//==============================================================================
template <typename Derived>
typename CotangentBase<Derived>::Cotangent CotangentBase<Derived>::Identity()
{
  // Assumed the default constructor creates the identity element
  const static Cotangent identity;
  return identity;
}

//==============================================================================
template <typename Derived>
typename CotangentBase<Derived>::Cotangent CotangentBase<Derived>::Random()
{
  return Cotangent().set_random();
}

//==============================================================================
template <typename Derived>
Derived& CotangentBase<Derived>::set_identity()
{
  coeffs().set_zero();
  return derived();
}

//==============================================================================
template <typename Derived>
Derived& CotangentBase<Derived>::set_random()
{
  coeffs().setRandom();
  return derived();
}

} // namespace dart::math
