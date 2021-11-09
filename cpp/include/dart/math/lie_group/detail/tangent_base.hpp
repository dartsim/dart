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
class TangentBase
{
public:
  DART_LIE_GROUP_BASE_TYPES;

  /// Creates an identity element
  [[nodiscard]] static Tangent Identity();

  /// Creates a random element
  [[nodiscard]] static Tangent Random();

protected:
  /// Default constructor
  TangentBase() = default;

  /// Destructor
  ~TangentBase() = default;

public:
  DART_LIE_GROUP_BASE_ASSIGN_OPERATORS(TangentBase)

  DART_LIE_GROUP_BASE_DATA(TangentData)

  [[nodiscard]] Tangent operator-() const
  {
    return Tangent(-coeffs());
  }

  template <typename OtherDerived>
  Derived& operator+=(const TangentBase<OtherDerived>& other)
  {
    coeffs() += other.coeffs();
    return derived();
  }

  template <typename OtherDerived>
  Derived& operator-=(const TangentBase<OtherDerived>& other)
  {
    coeffs() -= other.coeffs();
    return derived();
  }

  template <typename MatrixDerived>
  Derived& operator+=(const Eigen::MatrixBase<MatrixDerived>& other)
  {
    coeffs() += other;
    return derived();
  }

  template <typename MatrixDerived>
  Derived& operator-=(const Eigen::MatrixBase<MatrixDerived>& other)
  {
    coeffs() -= other;
    return derived();
  }

  Derived& operator*=(Scalar scalar)
  {
    coeffs() *= scalar;
    return derived();
  }

  Derived& operator/=(Scalar scalar)
  {
    coeffs() /= scalar;
    return derived();
  }

  Derived& set_identity();

  Derived& set_random();

  template <typename OtherDerived>
  [[nodiscard]] Scalar dot(const TangentBase<OtherDerived>& other) const
  {
    return coeffs().dot(other.coeffs());
  }

  template <typename CotangentDerived>
  [[nodiscard]] Scalar dot(const CotangentBase<CotangentDerived>& other) const
  {
    return coeffs().dot(other.coeffs());
  }

  [[nodiscard]] Scalar operator[](int i) const;

  [[nodiscard]] Scalar& operator[](int i);

protected:
  DART_LIE_GROUP_BASE_DERIVED
};

//==============================================================================
template <typename Derived>
typename TangentBase<Derived>::Tangent TangentBase<Derived>::Identity()
{
  // Assumed the default constructor creates the identity element
  const static Tangent identity;
  return identity;
}

//==============================================================================
template <typename Derived>
typename TangentBase<Derived>::Tangent TangentBase<Derived>::Random()
{
  return Tangent().set_random();
}

//==============================================================================
template <typename Derived>
Derived& TangentBase<Derived>::set_identity()
{
  coeffs().set_zero();
  return derived();
}

//==============================================================================
template <typename Derived>
Derived& TangentBase<Derived>::set_random()
{
  coeffs().setRandom();
  return derived();
}

//==============================================================================
template <typename Derived>
typename TangentBase<Derived>::Scalar TangentBase<Derived>::operator[](
    int i) const
{
  return coeffs()[i];
}

//==============================================================================
template <typename Derived>
typename TangentBase<Derived>::Scalar& TangentBase<Derived>::operator[](int i)
{
  return coeffs()[i];
}

} // namespace dart::math
