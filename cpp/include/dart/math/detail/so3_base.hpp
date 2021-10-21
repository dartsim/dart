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

#include "dart/math/type.hpp"

namespace Eigen::internal {

//==============================================================================
template <typename Scalar_, int Options_>
struct traits<dart::math::SO3<Scalar_, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = 3;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = 3;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = 3;

  /// Dimension of internal parameters to represent the group element.
  static constexpr int RepDim = 4;

  using Scalar = Scalar_;

  using LieGroup = dart::math::SO3<Scalar, Options>;
  using Matrix = Eigen::Matrix<Scalar, MatrixDim, MatrixDim, Options>;
  using Quaternion = Eigen::Quaternion<Scalar, Options>;

  using Rotation = Eigen::Matrix<Scalar, 3, 3>;
  using Translation = Eigen::Matrix<Scalar, 3, 1>;
  using Transformation
      = Eigen::Transform<Scalar, SpaceDim, Eigen::Isometry, Options>;

  using Tangent = dart::math::SO3Tangent<Scalar, Options>;
  using TangentVector = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using Cotangent = dart::math::SO3Cotangent<Scalar, Options>;
  using CotangentVector = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::SO3Algebra<Scalar, Options>;
  using LieAlgebraData = Eigen::Matrix<Scalar, MatrixDim, MatrixDim, Options>;

  using Jacobian = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;
};

} // namespace Eigen::internal

namespace dart::math {

//==============================================================================
template <typename Derived>
class SO3Base
{
public:
  using Matrix = typename Eigen::internal::traits<Derived>::Matrix;

  using Tangent = typename Eigen::internal::traits<Derived>::Tangent;
  using Cotangent = typename Eigen::internal::traits<Derived>::Cotangent;
  using Jacobian = typename Eigen::internal::traits<Derived>::Jacobian;

  // Eigen data
  using Rotation = typename Eigen::internal::traits<Derived>::Rotation;
  using Translation = typename Eigen::internal::traits<Derived>::Translation;
  using Transformation =
      typename Eigen::internal::traits<Derived>::Transformation;

  static constexpr int SpaceDim = Eigen::internal::traits<Derived>::SpaceDim;
  static constexpr int GroupDim = Eigen::internal::traits<Derived>::GroupDim;
  static constexpr int MatrixDim = Eigen::internal::traits<Derived>::MatrixDim;

protected:
  /// Default constructor
  SO3Base() = default;

  SO3Base(const SO3Base& other) = default;

  SO3Base(SO3Base&& other) = default;

  ~SO3Base() = default;

public:
  Derived& operator=(const SO3Base& other);

  Derived& operator=(SO3Base&& other);

  template <typename OtherDerived>
  Derived& operator=(const SO3Base<OtherDerived>& other);

  template <typename OtherDerived>
  Derived& operator=(SO3Base<OtherDerived>&& other);

  template <typename OtherDerived>
  bool operator==(const SO3Base<OtherDerived>& other) const
  {
    return quaternion().isApprox(other.quaternion());
  }

  void set_identity()
  {
    derived().quaternion().setIdentity();
  }

  const auto& quaternion() const
  {
    return derived().quaternion();
  }

  auto& quaternion()
  {
    return derived().quaternion();
  }

protected:
  const Derived& derived() const noexcept
  {
    return *static_cast<const Derived*>(this);
  }

  Derived& derived() noexcept
  {
    return *static_cast<Derived*>(this);
  }
};

//==============================================================================
template <typename Derived>
Derived& SO3Base<Derived>::operator=(const SO3Base<Derived>& o)
{
  quaternion() = o.quaternion();
  return derived();
}

//==============================================================================
template <typename Derived>
template <typename OtherDerived>
Derived& SO3Base<Derived>::operator=(const SO3Base<OtherDerived>& other)
{
  quaternion() = other.quaternion();
  return derived();
}

//==============================================================================
template <typename Derived>
Derived& SO3Base<Derived>::operator=(SO3Base&& other)
{
  quaternion().coeffs() = std::move(other.coeffs());
  return derived();
}

//==============================================================================
template <typename Derived>
template <typename OtherDerived>
Derived& SO3Base<Derived>::operator=(SO3Base<OtherDerived>&& other)
{
  quaternion() = std::move(other.quaternion());
  return derived();
}

} // namespace dart::math
