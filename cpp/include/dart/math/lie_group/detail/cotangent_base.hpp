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
class CotangentBase
{
public:
  DART_LIE_GROUP_BASE_TYPES;

  /// Creates a zero element
  [[nodiscard]] static Cotangent Zero();

  /// Creates a random element
  [[nodiscard]] static Cotangent Random();

protected:
  /// Default constructor
  CotangentBase() = default;

  /// Destructor
  ~CotangentBase() = default;

public:
  DART_LIE_GROUP_BASE_ASSIGN_OPERATORS(CotangentBase);

  [[nodiscard]] const Derived& operator+() const
  {
    return derived();
  }

  [[nodiscard]] Cotangent operator-() const
  {
    return Cotangent(-coeffs());
  }

  template <typename OtherDerived>
  Cotangent operator+(const CotangentBase<OtherDerived>& other) const
  {
    return Cotangent(coeffs() + other.coeffs());
  }

  template <typename OtherDerived>
  Cotangent operator-(const CotangentBase<OtherDerived>& other) const
  {
    return Cotangent(coeffs() - other.coeffs());
  }

  template <typename OtherDerived>
  Derived& operator+=(const CotangentBase<OtherDerived>& other)
  {
    coeffs() += other.coeffs();
    return derived();
  }

  template <typename OtherDerived>
  Derived& operator-=(const CotangentBase<OtherDerived>& other)
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

  void set_zero();

  void set_random();

  template <typename OtherDerived>
  [[nodiscard]] Scalar dot(const CotangentBase<OtherDerived>& other) const
  {
    return coeffs().dot(other.coeffs());
  }

  template <typename TangentDerived>
  [[nodiscard]] Scalar dot(const TangentBase<TangentDerived>& other) const
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
      const CotangentBase<OtherDerived>& other,
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
typename CotangentBase<Derived>::Cotangent CotangentBase<Derived>::Zero()
{
  // Assumed the default constructor creates the zero element
  const static Cotangent zero;
  return zero;
}

//==============================================================================
template <typename Derived>
typename CotangentBase<Derived>::Cotangent CotangentBase<Derived>::Random()
{
  auto out = Cotangent();
  out.set_random();
  return out;
}

//==============================================================================
template <typename Derived>
void CotangentBase<Derived>::set_zero()
{
  coeffs().set_zero();
}

//==============================================================================
template <typename Derived>
void CotangentBase<Derived>::set_random()
{
  coeffs().setRandom();
}

//==============================================================================
template <typename Derived>
typename CotangentBase<Derived>::Scalar CotangentBase<Derived>::operator[](
    int i) const
{
  return coeffs()[i];
}

//==============================================================================
template <typename Derived>
typename CotangentBase<Derived>::Scalar& CotangentBase<Derived>::operator[](
    int i)
{
  return coeffs()[i];
}

} // namespace dart::math
