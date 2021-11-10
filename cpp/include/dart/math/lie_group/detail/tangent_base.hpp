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

#include "dart/math/lie_group/detail/macro.hpp"
#include "dart/math/type.hpp"

namespace dart::math {

//==============================================================================
template <typename Derived>
class TangentBase
{
public:
  DART_LIE_GROUP_BASE_TYPES;

  /// Creates a zero element
  [[nodiscard]] static Tangent Zero();

  /// Creates a random element
  [[nodiscard]] static Tangent Random();

protected:
  /// Default constructor
  TangentBase() = default;

  /// Destructor
  ~TangentBase() = default;

public:
  DART_LIE_GROUP_BASE_ASSIGN_OPERATORS(TangentBase);

  [[nodiscard]] const Derived& operator+() const
  {
    return derived();
  }

  [[nodiscard]] Tangent operator-() const
  {
    return Tangent(-coeffs());
  }

  template <typename OtherDerived>
  Tangent operator+(const TangentBase<OtherDerived>& other) const
  {
    return Tangent(coeffs() + other.coeffs());
  }

  template <typename OtherDerived>
  Tangent operator-(const TangentBase<OtherDerived>& other) const
  {
    return Tangent(coeffs() - other.coeffs());
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

  Derived& set_zero();

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

  template <typename MatrixDerived>
  [[nodiscard]] bool is_approx(
      const Eigen::MatrixBase<MatrixDerived>& other,
      const Scalar tolerance = eps<Scalar>()) const
  {
    // TODO(JS): Improve
    if (std::min(coeffs().norm(), other.norm()) < tolerance) {
      return (coeffs() - other).isZero(tolerance);
    } else {
      return coeffs().isApprox(other, tolerance);
    }
  }

  template <typename OtherDerived>
  [[nodiscard]] bool is_approx(
      const TangentBase<OtherDerived>& other,
      const Scalar tolerance = eps<Scalar>()) const
  {
    return is_approx(other.coeffs(), tolerance);
  }

  [[nodiscard]] bool is_zero(const Scalar tolerance = eps<Scalar>()) const
  {
    if constexpr (DataDim >= 0) {
      return is_approx(DataType::Zero(), tolerance);
    } else {
      return is_approx(DataType::Zero(coeffs().size()), tolerance);
    }
  }

  DART_LIE_GROUP_BASE_DATA(DataType);

protected:
  DART_LIE_GROUP_BASE_DERIVED;
};

//==============================================================================
template <typename Derived>
typename TangentBase<Derived>::Tangent TangentBase<Derived>::Zero()
{
  // Assumed the default constructor creates the zero element
  const static Tangent zero;
  return zero;
}

//==============================================================================
template <typename Derived>
typename TangentBase<Derived>::Tangent TangentBase<Derived>::Random()
{
  return Tangent().set_random();
}

//==============================================================================
template <typename Derived>
Derived& TangentBase<Derived>::set_zero()
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
