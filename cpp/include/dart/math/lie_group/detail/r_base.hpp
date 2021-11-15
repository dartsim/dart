/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <Eigen/Core>

#include "dart/math/lie_group/detail/lie_group_base.hpp"
#include "dart/math/type.hpp"

namespace dart::math {

//==============================================================================
template <typename Derived>
class RBase : public LieGroupBase<Derived>
{
public:
  using This = RBase<Derived>;
  using Base = LieGroupBase<Derived>;

  DART_LIE_GROUP_USE_BASE_TYPES;

  [[nodiscard]] static LieGroup Zero()
  {
    const static LieGroup zero;
    return zero;
  }

  DART_LIE_GROUP_BASE_ASSIGN_OPERATORS(RBase);

  /// Implicit conversion to Eigen::Matrix<Scalar, Dim, 1>
  operator Eigen::Matrix<Scalar, GroupDim, 1>() const
  {
    return coeffs();
  }

  /// @{ @name Group operations

  template <typename OtherDerived>
  [[nodiscard]] LieGroup operator*(const RBase<OtherDerived>& other) const
  {
    return LieGroup(coeffs() + other.coeffs());
  }

  [[nodiscard]] LieGroup inverse() const
  {
    return LieGroup(-coeffs());
  }

  Derived& inverse_in_place()
  {
    coeffs() *= -1;
    return derived();
  }

  [[nodiscard]] Tangent log(
      Jacobian* jacobian = nullptr, Scalar tolerance = eps<Scalar>()) const
  {
    DART_UNUSED(tolerance);

    if (jacobian) {
      (*jacobian).setIdentity();
    }

    return Tangent(coeffs());
  }

  /// @}

  [[nodiscard]] const LieGroup& operator+() const
  {
    return derived();
  }

  [[nodiscard]] LieGroup operator-() const
  {
    return LieGroup(-coeffs());
  }

  template <typename RDerived>
  [[nodiscard]] LieGroup operator+(const RBase<RDerived>& other) const
  {
    return LieGroup(coeffs() + other.coeffs());
  }

  template <typename RDerived>
  [[nodiscard]] LieGroup operator-(const RBase<RDerived>& other) const
  {
    return LieGroup(coeffs() - other.coeffs());
  }

  template <typename OtherDerived>
  Derived& operator+=(const RBase<OtherDerived>& other)
  {
    coeffs() += other.coeffs();
    return derived();
  }

  template <typename OtherDerived>
  Derived& operator-=(const RBase<OtherDerived>& other)
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

  [[nodiscard]] LieGroup operator*(Scalar scalar) const
  {
    return LieGroup(coeffs() * scalar);
  }

  [[nodiscard]] LieGroup operator/(Scalar scalar) const
  {
    return LieGroup(coeffs() / scalar);
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

  void set_zero()
  {
    coeffs().setZero();
  }

  [[nodiscard]] bool is_zero(const Scalar tolerance = eps<Scalar>()) const
  {
    return this->is_approx(LieGroup::Zero(), tolerance);
  }

  [[nodiscard]] Scalar squared_norm() const
  {
    return coeffs().squaredNorm();
  }

  [[nodiscard]] Scalar norm() const
  {
    return coeffs().norm();
  }

  using Base::coeffs;
  using Base::data;

protected:
  using Base::derived;
};

//==============================================================================
template <typename Derived>
[[nodiscard]] typename Derived::LieGroup operator*(
    typename Derived::Scalar scalar, const RBase<Derived>& r)
{
  return typename Derived::LieGroup(scalar * r.coeffs());
}

namespace detail {

//==============================================================================
template <typename Derived>
struct NormalizationOperator<RBase<Derived>>
{
  template <typename T>
  static void run(T& x)
  {
    // TODO(JS)
    (void)x;
  }
};

//==============================================================================
template <typename Derived>
struct RandomSetter<RBase<Derived>>
{
  template <typename T>
  static void run(T& x)
  {
    using LieGroup = typename RBase<Derived>::LieGroup;
    using DataType = typename RBase<Derived>::DataType;

    if constexpr (RBase<Derived>::GroupDim > 0) {
      x = LieGroup(DataType::Random());
    } else {
      if (x.coeffs().size() > 0) {
        x = LieGroup(DataType::Random(x.coeffs().size()));
      }
    }
  }
};

} // namespace detail

} // namespace dart::math
