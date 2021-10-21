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

#include "dart/math/type.hpp"

namespace Eigen::internal {

//==============================================================================
template <typename Scalar_, int Dim, int Options_>
struct traits<dart::math::R<Scalar_, Dim, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = Dim;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = Dim;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = GroupDim + 1;

  static constexpr int RepDim = Dim;

  using Scalar = Scalar_;

  using LieGroup = dart::math::R<Scalar, GroupDim, Options>;
  using LieGroupData = Eigen::Matrix<Scalar, MatrixDim, Options>;
  using LieGroupCoeffs = Eigen::Matrix<Scalar, MatrixDim, Options>;

  using Rotation = Eigen::Matrix<Scalar, 3, 3>;
  using Transformation
      = Eigen::Transform<Scalar, SpaceDim, Eigen::Isometry, Options>;

  using Tangent = dart::math::RTangent<Scalar, GroupDim, Options>;
  using TangentVector = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using Cotangent = dart::math::RCotangent<Scalar, GroupDim, Options>;
  using CotangentVector = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::RAlgebra<Scalar, GroupDim, Options>;

  using Jacobian = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;
};

//==============================================================================
template <typename Scalar_, int Options_>
struct traits<dart::math::R<Scalar_, Eigen::Dynamic, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = 3;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = Eigen::Dynamic;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = Eigen::Dynamic;

  static constexpr int RepDim = Eigen::Dynamic;

  using Scalar = Scalar_;

  using LieGroup = dart::math::R<Scalar, GroupDim, Options>;
  using LieGroupData = Eigen::Matrix<Scalar, MatrixDim, Options>;
  using LieGroupCoeffs = Eigen::Matrix<Scalar, MatrixDim, Options>;

  using Rotation = Eigen::Matrix<Scalar, 3, 3>;
  using Transformation
      = Eigen::Transform<Scalar, SpaceDim, Eigen::Isometry, Options>;

  using Tangent = dart::math::RTangent<Scalar, GroupDim, Options>;
  using TangentVector = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using Cotangent = dart::math::RCotangent<Scalar, GroupDim, Options>;
  using CotangentVector = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::RAlgebra<Scalar, GroupDim, Options>;

  using Jacobian = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;
};

//==============================================================================
template <typename Scalar_, int Dim, int Options_>
struct traits<dart::math::RTangent<Scalar_, Dim, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = Dim;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = Dim;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = GroupDim + 1;

  static constexpr int RepDim = Dim;

  using Scalar = Scalar_;

  using LieGroup = dart::math::R<Scalar, GroupDim, Options>;
  using LieGroupData = Eigen::Matrix<Scalar, MatrixDim, Options>;
  using LieGroupCoeffs = Eigen::Matrix<Scalar, MatrixDim, Options>;

  using Rotation = Eigen::Matrix<Scalar, 3, 3>;
  using Transformation
      = Eigen::Transform<Scalar, SpaceDim, Eigen::Isometry, Options>;

  using Tangent = dart::math::RTangent<Scalar, GroupDim, Options>;
  using TangentVector = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using Cotangent = dart::math::RCotangent<Scalar, GroupDim, Options>;
  using CotangentVector = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::RAlgebra<Scalar, GroupDim, Options>;
  using LieAlgebraData = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;

  using Jacobian = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;

  using Data = Eigen::Matrix<Scalar, GroupDim, 1, Options>;
};

//==============================================================================
template <typename Scalar_, int Options_>
struct traits<dart::math::RTangent<Scalar_, Eigen::Dynamic, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = 3;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = Eigen::Dynamic;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = Eigen::Dynamic;

  static constexpr int RepDim = Eigen::Dynamic;

  using Scalar = Scalar_;

  using LieGroup = dart::math::R<Scalar, GroupDim, Options>;
  using LieGroupData = Eigen::Matrix<Scalar, MatrixDim, Options>;
  using LieGroupCoeffs = Eigen::Matrix<Scalar, MatrixDim, Options>;

  using Rotation = Eigen::Matrix<Scalar, 3, 3>;
  using Transformation
      = Eigen::Transform<Scalar, SpaceDim, Eigen::Isometry, Options>;

  using Tangent = dart::math::RTangent<Scalar, GroupDim, Options>;
  using TangentVector = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using Cotangent = dart::math::RCotangent<Scalar, GroupDim, Options>;
  using CotangentVector = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::RAlgebra<Scalar, GroupDim, Options>;
  using LieAlgebraData = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;

  using Jacobian = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;

  using Data = Eigen::Matrix<Scalar, GroupDim, 1, Options>;
};

//==============================================================================
template <typename Scalar_, int Dim, int Options_>
struct traits<dart::math::RCotangent<Scalar_, Dim, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = Dim;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = Dim;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = GroupDim + 1;

  static constexpr int RepDim = Dim;

  using Scalar = Scalar_;

  using LieGroup = dart::math::R<Scalar, GroupDim, Options>;
  using LieGroupData = Eigen::Matrix<Scalar, MatrixDim, Options>;
  using LieGroupCoeffs = Eigen::Matrix<Scalar, MatrixDim, Options>;

  using Rotation = Eigen::Matrix<Scalar, 3, 3>;
  using Transformation
      = Eigen::Transform<Scalar, SpaceDim, Eigen::Isometry, Options>;

  using Tangent = dart::math::RTangent<Scalar, GroupDim, Options>;
  using TangentVector = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using Cotangent = dart::math::RCotangent<Scalar, GroupDim, Options>;
  using CotangentVector = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::RAlgebra<Scalar, GroupDim, Options>;
  using LieAlgebraData = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;

  using Jacobian = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;

  using Data = Eigen::Matrix<Scalar, GroupDim, 1, Options>;
};

//==============================================================================
template <typename Scalar_, int Options_>
struct traits<dart::math::RCotangent<Scalar_, Eigen::Dynamic, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = 3;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = Eigen::Dynamic;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = Eigen::Dynamic;

  static constexpr int RepDim = Eigen::Dynamic;

  using Scalar = Scalar_;

  using LieGroup = dart::math::R<Scalar, GroupDim, Options>;
  using LieGroupData = Eigen::Matrix<Scalar, MatrixDim, Options>;
  using LieGroupCoeffs = Eigen::Matrix<Scalar, MatrixDim, Options>;

  using Rotation = Eigen::Matrix<Scalar, 3, 3>;
  using Transformation
      = Eigen::Transform<Scalar, SpaceDim, Eigen::Isometry, Options>;

  using Tangent = dart::math::RTangent<Scalar, GroupDim, Options>;
  using TangentVector = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using Cotangent = dart::math::RCotangent<Scalar, GroupDim, Options>;
  using CotangentVector = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::RAlgebra<Scalar, GroupDim, Options>;
  using LieAlgebraData = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;

  using Jacobian = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;

  using Data = Eigen::Matrix<Scalar, GroupDim, 1, Options>;
};

//==============================================================================
template <typename Scalar_, int Dim, int Options_>
struct traits<Map<dart::math::RTangent<Scalar_, Dim>, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = Dim;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = Dim;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = GroupDim + 1;

  static constexpr int RepDim = Dim;

  using Scalar = Scalar_;

  using LieGroup = dart::math::R<Scalar, GroupDim, Options>;
  using LieGroupData = Eigen::Matrix<Scalar, MatrixDim, Options>;
  using LieGroupCoeffs = Eigen::Matrix<Scalar, MatrixDim, Options>;

  using Rotation = Eigen::Matrix<Scalar, 3, 3>;
  using Transformation
      = Eigen::Transform<Scalar, SpaceDim, Eigen::Isometry, Options>;

  using Tangent = dart::math::RTangent<Scalar, GroupDim, Options>;
  using TangentVector = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using Cotangent = dart::math::RCotangent<Scalar, GroupDim, Options>;
  using CotangentVector = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::RAlgebra<Scalar, GroupDim, Options>;
  using LieAlgebraData = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;

  using Jacobian = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;

  using Data = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  typedef Eigen::Dense StorageKind;
  typedef Eigen::MatrixXpr XprKind;
  typedef typename Data::StorageIndex StorageIndex;
  enum
  {
    Flags = Eigen::ColMajor,
    RowsAtCompileTime = Data::RowsAtCompileTime,
    ColsAtCompileTime = Data::RowsAtCompileTime,
    MaxRowsAtCompileTime = Data::MaxRowsAtCompileTime,
    MaxColsAtCompileTime = Data::MaxRowsAtCompileTime,

    InnerStrideAtCompileTime = Data::InnerStrideAtCompileTime,
    SizeAtCompileTime = Data::SizeAtCompileTime,
  };
};

//==============================================================================
template <typename Scalar_, int Options_>
struct traits<Map<dart::math::RTangent<Scalar_, Eigen::Dynamic>, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = Eigen::Dynamic;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = Eigen::Dynamic;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = GroupDim + 1;

  static constexpr int RepDim = Eigen::Dynamic;

  using Scalar = Scalar_;

  using LieGroup = dart::math::R<Scalar, GroupDim, Options>;
  using LieGroupData = Eigen::Matrix<Scalar, MatrixDim, Options>;
  using LieGroupCoeffs = Eigen::Matrix<Scalar, MatrixDim, Options>;

  using Rotation = Eigen::Matrix<Scalar, 3, 3>;
  using Transformation
      = Eigen::Transform<Scalar, SpaceDim, Eigen::Isometry, Options>;

  using Tangent = dart::math::RTangent<Scalar, GroupDim, Options>;
  using TangentVector = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using Cotangent = dart::math::RCotangent<Scalar, GroupDim, Options>;
  using CotangentVector = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::RAlgebra<Scalar, GroupDim, Options>;
  using LieAlgebraData = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;

  using Jacobian = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;

  using Data = Eigen::Matrix<Scalar, GroupDim, 1, Options>;
};

} // namespace Eigen::internal

namespace dart::math {

//==============================================================================
template <typename Derived>
class RBase
{
public:
  using Tangent = typename Eigen::internal::traits<Derived>::Tangent;
  using Cotangent = typename Eigen::internal::traits<Derived>::Cotangent;
  using Jacobian = typename Eigen::internal::traits<Derived>::Jacobian;
  using Rotation = typename Eigen::internal::traits<Derived>::Rotation;
  using Transformation =
      typename Eigen::internal::traits<Derived>::Transformation;

  static constexpr int GroupDim = Eigen::internal::traits<Derived>::GroupDim;
  static constexpr int MatrixDim = Eigen::internal::traits<Derived>::MatrixDim;

  template <typename OtherDerived>
  bool operator==(const RBase<OtherDerived>& other) const
  {
    return vector() == other.vector();
  }

  bool is_zero() const
  {
    return derived().vector().isZero();
  }

  auto vector() const
  {
    return derived().vector();
  }

  auto& vector()
  {
    return derived().vector();
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
class RTangentBase
{
public:
  using TangentVector =
      typename Eigen::internal::traits<Derived>::TangentVector;
  using LieGroup = typename Eigen::internal::traits<Derived>::LieGroup;
  using LieAlgebra = typename Eigen::internal::traits<Derived>::LieAlgebra;
  using Jacobian = typename Eigen::internal::traits<Derived>::Jacobian;

  /// Assign operator
  template <typename OtherDerived>
  Derived& operator=(const RTangentBase<OtherDerived>& other)
  {
    derived().vector() = other.vector();
    return derived();
  }

  /// Move operator
  template <typename OtherDerived>
  Derived& operator=(RTangentBase<OtherDerived>&& other)
  {
    derived().vector() = std::move(other.vector());
    return derived();
  }

  /// Assign operator
  ///
  /// Allows to assign the tangent space of SO(3)
  template <typename OtherDerived>
  Derived& operator=(const SO3TangentBase<OtherDerived>& other)
  {
    derived().vector() = other.vector();
    return derived();
  }

  /// Move operator
  ///
  /// Allows to move the tangent space of SO(3)
  template <typename OtherDerived>
  Derived& operator=(SO3TangentBase<OtherDerived>&& other)
  {
    derived().vector() = std::move(other.vector());
    return derived();
  }

  template <typename OtherDerived>
  Derived& operator+=(const RTangentBase<OtherDerived>& other)
  {
    derived().vector() += other.vector();
    return derived();
  }

  template <typename OtherDerived>
  Derived& operator+=(const SO3TangentBase<OtherDerived>& other)
  {
    derived().vector() += other.vector();
    return derived();
  }

  template <typename OtherDerived>
  Derived& operator-=(const RTangentBase<OtherDerived>& other)
  {
    derived().vector() -= other.vector();
    return derived();
  }

  template <typename OtherDerived>
  Derived& operator-=(const SO3TangentBase<OtherDerived>& other)
  {
    derived().vector() -= other.vector();
    return derived();
  }

  /// Returns whether the vector is zero
  bool is_zero() const
  {
    return derived().vector().isZero();
  }

  auto noalias()
  {
    return derived().vector().noalias();
  }

  friend std::ostream& operator<<(
      std::ostream& os, const RTangentBase<Derived>& x)
  {
    os << x.derived().vector().transpose();
    return os;
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
class RCotangentBase
{
public:
  using CotangentVector =
      typename Eigen::internal::traits<Derived>::CotangentVector;
  using LieGroup = typename Eigen::internal::traits<Derived>::LieGroup;
  using LieAlgebra = typename Eigen::internal::traits<Derived>::LieAlgebra;
  using Jacobian = typename Eigen::internal::traits<Derived>::Jacobian;

  /// Assign operator
  template <typename OtherDerived>
  Derived& operator=(const RCotangentBase<OtherDerived>& other)
  {
    derived().vector() = other.vector();
    return derived();
  }

  /// Move operator
  template <typename OtherDerived>
  Derived& operator=(RCotangentBase<OtherDerived>&& other)
  {
    derived().vector() = std::move(other.vector());
    return derived();
  }

  /// Assign operator
  ///
  /// Allows to assign the tangent space of SO(3)
  template <typename OtherDerived>
  Derived& operator=(const SO3TangentBase<OtherDerived>& other)
  {
    derived().vector() = other.vector();
    return derived();
  }

  /// Move operator
  ///
  /// Allows to move the tangent space of SO(3)
  template <typename OtherDerived>
  Derived& operator=(SO3TangentBase<OtherDerived>&& other)
  {
    derived().vector() = std::move(other.vector());
    return derived();
  }

  template <typename OtherDerived>
  Derived& operator+=(const RCotangentBase<OtherDerived>& other)
  {
    derived().vector() += other.vector();
    return derived();
  }

  template <typename OtherDerived>
  Derived& operator-=(const RCotangentBase<OtherDerived>& other)
  {
    derived().vector() -= other.vector();
    return derived();
  }

  template <typename OtherDerived>
  Derived& operator-=(const SO3TangentBase<OtherDerived>& other)
  {
    derived().vector() -= other.vector();
    return derived();
  }

  /// Returns whether the vector is zero
  bool is_zero() const
  {
    return derived().vector().isZero();
  }

  auto noalias()
  {
    return derived().vector().noalias();
  }

  friend std::ostream& operator<<(
      std::ostream& os, const RCotangentBase<Derived>& x)
  {
    os << x.derived().vector().transpose();
    return os;
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
