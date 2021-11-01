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
 * 2. Redistributions in binary form must reproduce the above copyright
 notice,
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

#include "dart/math/lie_group/detail/lie_group_base.hpp"
#include "dart/math/lie_group/detail/so3_map.hpp"
#include "dart/math/type.hpp"

namespace Eigen::internal {

////==============================================================================
// template <typename Scalar_, int Options_>
// struct traits<dart::math::SE3Inverse<Scalar_, Options_>>
//{
//  static constexpr int Options = Options_;

//  /// Dimension of the Euclidean space that the Lie group is embedded.
//  static constexpr int SpaceDim = 3;

//  /// Dimension of the Lie group.
//  static constexpr int GroupDim = 6;

//  /// Dimension of the matrix representation of the Lie group.
//  static constexpr int MatrixDim = 4;

//  /// Dimension of internal parameters to represent the group element.
//  static constexpr int DataDim = traits<dart::math::SO3<Scalar_>>::DataDim
//                                + traits<dart::math::R3<Scalar_>>::DataDim;

//  using Scalar = Scalar_;

//  using LieGroup = dart::math::SE3<Scalar, Options>;
//  using Data = Eigen::Matrix<Scalar, DataDim, 1, Options>;

//  using Tangent = dart::math::SE3Tangent<Scalar, Options>;
//  using TangentData = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

//  using Cotangent = dart::math::SE3Cotangent<Scalar, Options>;
//  using CotangentData = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

//  using LieAlgebra = dart::math::SE3Algebra<Scalar, Options>;
//  using LieAlgebraData = Eigen::Matrix<Scalar, MatrixDim, MatrixDim, Options>;

//  using Jacobian = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;
//};

////==============================================================================
// template <typename Scalar_, int Options_>
// struct traits<dart::math::SE3Algebra<Scalar_, Options_>>
//{
//  static constexpr int Options = Options_;

//  /// Dimension of the Euclidean space that the Lie group is embedded.
//  static constexpr int SpaceDim = 3;

//  /// Dimension of the Lie group.
//  static constexpr int GroupDim = 6;

//  /// Dimension of the matrix representation of the Lie group.
//  static constexpr int MatrixDim = 4;

//  static constexpr int DataDim = MatrixDim * MatrixDim;

//  using Scalar = Scalar_;

//  using LieGroup = dart::math::SE3<Scalar, Options>;
//  using Data = Eigen::Matrix<Scalar, DataDim, 1, Options>;

//  using Tangent = dart::math::SE3Tangent<Scalar, Options>;
//  using TangentData = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

//  using Cotangent = dart::math::SE3Cotangent<Scalar, Options>;
//  using CotangentData = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

//  using LieAlgebra = dart::math::SE3Algebra<Scalar, Options>;
//  using LieAlgebraData = Eigen::Matrix<Scalar, MatrixDim, MatrixDim, Options>;

//  using Jacobian = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;
//};

////==============================================================================
// template <typename Scalar_, int Options_>
// struct traits<dart::math::SE3Tangent<Scalar_, Options_>>
//{
//  static constexpr int Options = Options_;

//  /// Dimension of the Euclidean space that the Lie group is embedded.
//  static constexpr int SpaceDim = 3;

//  /// Dimension of the Lie group.
//  static constexpr int GroupDim = 6;

//  /// Dimension of the matrix representation of the Lie group.
//  static constexpr int MatrixDim = 4;

//  static constexpr int DataDim = 6;

//  using Scalar = Scalar_;

//  using LieGroup = dart::math::SE3<Scalar, Options>;
//  using Data = Eigen::Matrix<Scalar, DataDim, 1, Options>;

//  using Tangent = dart::math::SE3Tangent<Scalar, Options>;
//  using TangentData = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

//  using Cotangent = dart::math::SE3Cotangent<Scalar, Options>;
//  using CotangentData = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

//  using LieAlgebra = dart::math::SE3Algebra<Scalar, Options>;
//  using LieAlgebraData = Eigen::Matrix<Scalar, MatrixDim, MatrixDim, Options>;

//  using Jacobian = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;
//};

////==============================================================================
// template <typename Scalar_, int Options_>
// struct traits<dart::math::SE3Cotangent<Scalar_, Options_>>
//{
//  static constexpr int Options = Options_;

//  /// Dimension of the Euclidean space that the Lie group is embedded.
//  static constexpr int SpaceDim = 3;

//  /// Dimension of the Lie group.
//  static constexpr int GroupDim = 6;

//  /// Dimension of the matrix representation of the Lie group.
//  static constexpr int MatrixDim = 4;

//  static constexpr int DataDim = 6;

//  using Scalar = Scalar_;

//  using LieGroup = dart::math::SE3<Scalar, Options>;
//  using Data = Eigen::Matrix<Scalar, DataDim, 1, Options>;

//  using Tangent = dart::math::SE3Tangent<Scalar, Options>;
//  using TangentData = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

//  using Cotangent = dart::math::SE3Cotangent<Scalar, Options>;
//  using CotangentData = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

//  using LieAlgebra = dart::math::SE3Algebra<Scalar, Options>;
//  using LieAlgebraData = Eigen::Matrix<Scalar, MatrixDim, MatrixDim, Options>;

//  using Jacobian = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;
//};

} // namespace Eigen::internal

namespace dart::math {

// template <typename Scalar, int Options>
// struct Eval<SE3<Scalar, Options>>
//{
//  using Type = SE3<Scalar, Options>;
//};

// template <typename Scalar, int Options>
// struct Eval<SE3Inverse<Scalar, Options>>
//{
//  using Type = SE3<Scalar, Options>;
//};

//==============================================================================
template <typename Derived>
class SE3Base : public LieGroupBase<Derived>
{
public:
  using This = SE3Base<Derived>;
  using Base = LieGroupBase<Derived>;

  DART_LIE_GROUP_USE_BASE_TYPES

  using Quaternion = math::Quaternion<Scalar, Options>;
  using QuaternionMap = Eigen::Map<Quaternion>;
  using ConstQuaternionMap = Eigen::Map<const Quaternion>;

  using Rotation = Eigen::Matrix<Scalar, 3, 3>;
  using RotationMap = Eigen::Map<Rotation>;
  using ConstRotationMap = Eigen::Map<const Rotation>;

  using Translation = Eigen::Matrix<Scalar, 3, 1>;
  using TranslationMap = Eigen::Map<Translation>;
  using ConstTranslationMap = Eigen::Map<const Translation>;

  using Transformation
      = Eigen::Transform<Scalar, SpaceDim, Eigen::Isometry, Options>;
  using TransformationMap = Eigen::Map<Transformation>;
  using ConstTransformationMap = Eigen::Map<const Transformation>;

  //  using EvalReturnType = typename Eval<Derived>::Type;

  //  template <typename OtherDerived>
  //  bool operator==(const SE3Base<OtherDerived>& other) const
  //  {
  //    return (position() == other.position())
  //           && (orientation() == other.orientation());
  //  }

  //  EvalReturnType eval() const
  //  {
  //    return derived().eval();
  //  }

  //  LieGroupNoAlias<Derived, SE3Base> noalias()
  //  {
  //    return LieGroupNoAlias<Derived, SE3Base>(derived());
  //  }

  Eigen::Map<const SO3<Scalar>> orientation() const;

  Eigen::Map<SO3<Scalar>> orientation();

  //  auto position() const;

  //  auto position();

  TranslationMap translation();

  ConstTranslationMap translation() const;

  /// Returns the x component of the orientation part in quaternion.
  Scalar quat_x() const;

  /// Returns the y component of the orientation part in quaternion.
  Scalar quat_y() const;

  /// Returns the z component of the orientation part in quaternion.
  Scalar quat_z() const;

  /// Returns the w component of the orientation part in quaternion.
  Scalar quat_w() const;

  /// Returns the x component of the orientation part in quaternion.
  Scalar& quat_x();

  /// Returns the y component of the orientation part in quaternion.
  Scalar& quat_y();

  /// Returns the z component of the orientation part in quaternion.
  Scalar& quat_z();

  /// Returns the w component of the orientation part in quaternion.
  Scalar& quat_w();

  /// Returns the x component of the position part.
  Scalar x() const;

  /// Returns the y component of the position part.
  Scalar y() const;

  /// Returns the z component of the position part.
  Scalar z() const;

  /// Returns the x component of the position part.
  Scalar& x();

  /// Returns the y component of the position part.
  Scalar& y();

  /// Returns the z component of the position part.
  Scalar& z();

  using Base::coeffs;
  using Base::data;

protected:
  using Base::derived;
};

//==============================================================================
template <typename Derived>
Eigen::Map<const SO3<typename SE3Base<Derived>::Scalar>>
SE3Base<Derived>::orientation() const
{
  return Eigen::Map<const SO3<Scalar>>(data());
}

//==============================================================================
template <typename Derived>
Eigen::Map<SO3<typename SE3Base<Derived>::Scalar>>
SE3Base<Derived>::orientation()
{
  return Eigen::Map<SO3<Scalar>>(data());
}

////==============================================================================
// template <typename Derived>
// auto SE3Base<Derived>::position() const
//{
//  return derived().position();
//}

////==============================================================================
// template <typename Derived>
// auto SE3Base<Derived>::position()
//{
//  return derived().position();
//}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::TranslationMap SE3Base<Derived>::translation()
{
  return TranslationMap(data() + 4);
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::ConstTranslationMap SE3Base<Derived>::translation()
    const
{
  return ConstTranslationMap(data() + 4);
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar SE3Base<Derived>::quat_x() const
{
  return coeffs().x();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar SE3Base<Derived>::quat_y() const
{
  return coeffs().y();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar SE3Base<Derived>::quat_z() const
{
  return coeffs().z();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar SE3Base<Derived>::quat_w() const
{
  return coeffs().w();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar& SE3Base<Derived>::quat_x()
{
  return coeffs().x();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar& SE3Base<Derived>::quat_y()
{
  return coeffs().y();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar& SE3Base<Derived>::quat_z()
{
  return coeffs().z();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar& SE3Base<Derived>::quat_w()
{
  return coeffs().w();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar SE3Base<Derived>::x() const
{
  return coeffs()[4];
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar SE3Base<Derived>::y() const
{
  return coeffs()[5];
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar SE3Base<Derived>::z() const
{
  return coeffs()[6];
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar& SE3Base<Derived>::x()
{
  return coeffs()[4];
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar& SE3Base<Derived>::y()
{
  return coeffs()[5];
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Scalar& SE3Base<Derived>::z()
{
  return coeffs()[6];
}

////==============================================================================
// template <typename Derived>
// class SE3AlgebraBase : public LieAlgebra<Derived>
//{
// public:
//  using Base = LieAlgebra<Derived>;
//  using TangentData = typename Base::TangentData;
//  using LieGroup = typename Base::LieGroup;
//  using LieAlgebraData = typename Base::LieAlgebraData;
//  using Jacobian = typename Base::Jacobian;

// protected:
//  using Base::derived;
//};

////==============================================================================
// template <typename Derived>
// class SE3TangentBase : public LieGroupTangent<Derived>
//{
// public:
//  using Base = LieGroupTangent<Derived>;
//  using Scalar = typename Base::Scalar;
//  using Matrix = typename Base::Matrix;
//  using TangentData = typename Base::TangentData;
//  using LieGroup = typename Base::LieGroup;
//  using LieAlgebra = typename Base::LieAlgebra;
//  using LieAlgebraData = typename Base::LieAlgebraData;
//  using Jacobian = typename Base::Jacobian;

//  /// Assignment operator
//  template <typename OtherDerived>
//  Derived& operator=(const SE3TangentBase<OtherDerived>& other)
//  {
//    derived().vector() = other.vector();
//    return derived();
//  }

//  /// Move operator
//  template <typename OtherDerived>
//  Derived& operator=(SE3TangentBase<OtherDerived>&& other)
//  {
//    derived().vector() = std::move(other.vector());
//    return derived();
//  }

//  /// Compound assignment operator
//  template <typename OtherDerived>
//  Derived& operator+=(const SE3TangentBase<OtherDerived>& other)
//  {
//    derived().vector() += other.vector();
//    return derived();
//  }

//  /// Compound assignment operator
//  template <typename OtherDerived>
//  Derived& operator-=(const SE3TangentBase<OtherDerived>& other)
//  {
//    derived().vector() -= other.vector();
//    return derived();
//  }

//  [[nodiscard]] bool is_zero() const
//  {
//    return derived().vector().isZero();
//  }

//  template <typename OtherDerived>
//  [[nodiscard]] Derived ad(const SE3TangentBase<OtherDerived>& other) const;

//  [[nodiscard]] const TangentData& vector() const;

//  [[nodiscard]] TangentData& vector();

//  template <typename SO3TangentDerived>
//  void set_angular(const SO3TangentBase<SO3TangentDerived>& other)
//  {
//    derived().set_angular(other);
//  }

//  [[nodiscard]] auto angular() const
//  {
//    return derived().angular();
//  }

//  template <typename RTangentDerived>
//  void set_linear(const RTangentBase<RTangentDerived>& other)
//  {
//    derived().set_linear(other);
//  }

//  [[nodiscard]] auto linear() const
//  {
//    return derived().linear();
//  }

// protected:
//  using Base::derived;
//};

////==============================================================================
// template <typename Derived>
// class SE3CotangentBase : public LieGroupCotangent<Derived>
//{
// public:
//  using Base = LieGroupCotangent<Derived>;
//  using TangentData = typename Base::TangentData;
//  using LieGroup = typename Base::LieGroup;
//  using LieAlgebraData = typename Base::LieAlgebraData;
//  using Jacobian = typename Base::Jacobian;

//  bool is_zero() const
//  {
//    return derived().vector().isZero();
//  }

// protected:
//  using Base::derived;
//};
namespace detail {

//==============================================================================
template <typename Derived>
struct NormalizationOperator<SE3Base<Derived>>
{
  template <typename T>
  static void run(T& x)
  {
    (void)x;
    // x.normalize();
  }
};

//==============================================================================
template <typename Derived>
struct RandomSetter<SE3Base<Derived>>
{
  template <typename T>
  static void run(T& x)
  {
    (void)x;
    //    using Scalar = typename SE3Base<Derived>::Scalar;
    //    using LieGroup = typename SE3Base<Derived>::LieGroup;

    //    x = LieGroup(Eigen::Quaternion<Scalar>::UnitRandom());
  }
};

} // namespace detail

} // namespace dart::math
