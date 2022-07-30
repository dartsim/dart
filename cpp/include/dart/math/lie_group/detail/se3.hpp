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

#include "dart/math/constant.hpp"
#include "dart/math/lie_group/detail/se3_base.hpp"
#include "dart/math/lie_group/r.hpp"
#include "dart/math/lie_group/so3.hpp"
#include "dart/math/lie_group/type.hpp"

namespace Eigen::internal {

//==============================================================================
template <typename Scalar_, int Options_>
struct traits<dart::math::SE3<Scalar_, Options_>>
{
  static constexpr int Options = Options_;
  static constexpr int SpaceDim = 3;
  static constexpr int GroupDim = 6;
  static constexpr int MatrixDim = 4;
  static constexpr int DataDim = traits<dart::math::SO3<Scalar_>>::DataDim
                                 + traits<dart::math::R3<Scalar_>>::DataDim;

  DART_LIE_GROUP_TRAITS_TYPES(SE3);

  using DataType = Eigen::Matrix<Scalar, DataDim, 1>;
};

} // namespace Eigen::internal

namespace dart::math {

//==============================================================================
template <typename Scalar_, int Options_>
class SE3 : public SE3Base<SE3<Scalar_, Options_>>
{
public:
  using Base = SE3Base<SE3<Scalar_, Options_>>;
  using This = SE3<Scalar_, Options_>;

  DART_LIE_GROUP_USE_BASE_TYPES;

  using Quaternion = typename Base::Quaternion;
  using QuaternionMap = typename Base::QuaternionMap;
  using ConstQuaternionMap = typename Base::ConstQuaternionMap;

  using Rotation = typename Base::Rotation;

  using Translation = typename Base::Translation;
  using TranslationMap = typename Base::TranslationMap;
  using ConstTranslationMap = typename Base::ConstTranslationMap;

  using Transformation = typename Base::Transformation;

  //  using EvalReturnType = typename Base::EvalReturnType;

  template <typename DerivedA, typename DerivedB>
  static Tangent Ad(
      const SE3Base<DerivedA>& T, const SE3TangentBase<DerivedB>& V);

  template <typename DerivedA, typename DerivedB>
  static Tangent Ad_R(
      const SE3Base<DerivedA>& T, const SE3TangentBase<DerivedB>& V);

  /// Default constructor
  SE3();

  DART_LIE_GROUP_CONSTRUCTORS(SE3);

  /** Copy constructor from LieGroupBase*/
  template <typename OtherDerived>
  SE3(const LieGroupBase<OtherDerived>& other) : m_data(other.coeffs())
  {
    /* Do nothing */
  }

  /** Move constructor from LieGroupBase */
  template <typename OtherDerived>
  SE3(LieGroupBase<OtherDerived>&& other) : m_data(std::move(other.coeffs()))
  {
    /* Do nothing */
  }

  /// Constructs from orientation and position from SO3Derived and RDerived.
  template <typename SO3Derived, typename RDerived>
  SE3(const SO3Base<SO3Derived>& orientation, const RBase<RDerived>& position);

  /// Constructs from orientation and position from SO3Derived and RDerived
  template <typename SO3Derived, typename RDerived>
  SE3(SO3Base<SO3Derived>&& orientation, RBase<RDerived>&& position);

  /// Constructs from Transform
  SE3(const Eigen::Transform<Scalar, 3, Eigen::Isometry, Options>& tf);

  /// Constructs from Transform
  SE3(Eigen::Transform<Scalar, 3, Eigen::Isometry, Options>&& tf);

  /// Constructs from AngleAxis and Translation
  template <typename TransDerived>
  SE3(const Eigen::AngleAxis<Scalar>& aa,
      const Eigen::MatrixBase<TransDerived>& trans);

  /// Constructs from Quaternion and Translation
  template <typename TransDerived>
  SE3(const Quaternion& quat, const Eigen::MatrixBase<TransDerived>& trans);

  /// Constructs from Quaternion and Translation
  template <typename TransDerived>
  SE3(Quaternion&& quat, Eigen::MatrixBase<TransDerived>&& trans);

  /// Constructs from Quaternion and Translation
  template <typename RotDerived, typename TransDerived>
  SE3(const Eigen::MatrixBase<RotDerived>& rot,
      const Eigen::MatrixBase<TransDerived>& trans);

  /// Constructs from Quaternion and Translation
  template <typename RotDerived, typename TransDerived>
  SE3(Eigen::MatrixBase<RotDerived>&& rot,
      Eigen::MatrixBase<TransDerived>&& trans);

  /// Constructs from position scalars
  SE3(Scalar x, Scalar y, Scalar z);

  /// Destructor
  ~SE3() = default;

  DART_LIE_GROUP_ASSIGN_OPERATORS(SE3);

  /// @{ @name Group operation

  DART_LIE_GROUP_USE_BASE_GROUP_OPERATIONS;

  /// @}

  using Base::to_quaternion;
  using Base::to_transformation;
  using Base::to_transformation_matrix;
  using Base::to_translation;

  const DataType& coeffs() const;

  DataType& coeffs();

  using Base::data;

protected:
  using Base::derived;
  using Base::quaternion;
  using Base::translation;

  DataType m_data;
};

DART_TEMPLATE_CLASS_SCALAR(SE3)

//==============================================================================
template <typename Scalar, int Options>
template <typename DerivedA, typename DerivedB>
typename SE3<Scalar, Options>::Tangent SE3<Scalar, Options>::Ad(
    const SE3Base<DerivedA>& T, const SE3TangentBase<DerivedB>& V)
{
  const auto& orientation = T.orientation();
  const auto& position = T.position();

  // Using rotation matrix is more efficient when multiplying 3d vector more
  // than once
  const Eigen::Matrix<Scalar, 3, 3> rotation
      = orientation.to_quaternion().toRotationMatrix();
  const auto& translation = position.coeffs();

  typename Tangent::DataType data;
  data.template head<3>().noalias() = rotation * V.angular_coeffs();
  data.template tail<3>().noalias()
      = rotation * V.linear_coeffs()
        + translation.cross(data.template head<3>());

  return Tangent(std::move(data));
}

//==============================================================================
template <typename Scalar, int Options>
template <typename DerivedA, typename DerivedB>
typename SE3<Scalar, Options>::Tangent SE3<Scalar, Options>::Ad_R(
    const SE3Base<DerivedA>& T, const SE3TangentBase<DerivedB>& V)
{
  const auto& orientation = T.orientation();

  // Using rotation matrix is more efficient when multiplying 3d vector more
  // than once
  const Eigen::Matrix<Scalar, 3, 3> rotation
      = orientation.to_quaternion().toRotationMatrix();

  typename Tangent::DataType data;
  data.template head<3>().noalias() = rotation * V.angular_coeffs();
  data.template tail<3>().noalias() = rotation * V.linear_coeffs();

  return Tangent(std::move(data));
}

//==============================================================================
template <typename Scalar, int Options>
SE3<Scalar, Options>::SE3() : m_data(DataType::Zero())
{
  this->quat_w() = 1;
}

//==============================================================================
template <typename Scalar, int Options>
template <typename SO3Derived, typename RDerived>
SE3<Scalar, Options>::SE3(
    const SO3Base<SO3Derived>& orientation, const RBase<RDerived>& position)
  : SE3(orientation.coeffs(), position.coeffs())
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
template <typename SO3Derived, typename RDerived>
SE3<Scalar, Options>::SE3(
    SO3Base<SO3Derived>&& orientation, RBase<RDerived>&& position)
  : SE3(std::move(orientation.coeffs()), std::move(position.coeffs()))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
SE3<Scalar, Options>::SE3(
    const Eigen::Transform<Scalar, 3, Eigen::Isometry, Options>& tf)
  : SE3(tf.linear(), tf.translation())
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
SE3<Scalar, Options>::SE3(
    Eigen::Transform<Scalar, 3, Eigen::Isometry, Options>&& tf)
  : SE3(std::move(tf.linear()), std::move(tf.translation()))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
template <typename TransDerived>
SE3<Scalar, Options>::SE3(
    const Eigen::AngleAxis<Scalar>& aa,
    const Eigen::MatrixBase<TransDerived>& trans)
  : SE3(Quaternion(aa), trans)
{
  // Do nothing
}

//==============================================================================
#if EIGEN_VERSION_AT_LEAST(3, 4, 0)
template <typename Scalar, int Options>
template <typename TransDerived>
SE3<Scalar, Options>::SE3(
    const Quaternion& quat, const Eigen::MatrixBase<TransDerived>& trans)
  : m_data{
      quat.coeffs()[0],
      quat.coeffs()[1],
      quat.coeffs()[2],
      quat.coeffs()[3],
      trans[0],
      trans[1],
      trans[2]}
{
  // Do nothing
}
#else
template <typename Scalar, int Options>
template <typename TransDerived>
SE3<Scalar, Options>::SE3(
    const Quaternion& quat, const Eigen::MatrixBase<TransDerived>& trans)
{
  m_data.template head<4>() = quat.coeffs();
  m_data.template tail<3>() = trans;
}
#endif

//==============================================================================
#if EIGEN_VERSION_AT_LEAST(3, 4, 0)
template <typename Scalar, int Options>
template <typename TransDerived>
SE3<Scalar, Options>::SE3(
    Quaternion&& quat, Eigen::MatrixBase<TransDerived>&& trans)
  : m_data{
      quat.coeffs()[0],
      quat.coeffs()[1],
      quat.coeffs()[2],
      quat.coeffs()[3],
      trans[0],
      trans[1],
      trans[2]}
{
  // Do nothing
}
#else
template <typename Scalar, int Options>
template <typename TransDerived>
SE3<Scalar, Options>::SE3(
    Quaternion&& quat, Eigen::MatrixBase<TransDerived>&& trans)
{
  m_data.template head<4>() = std::move(quat.coeffs());
  m_data.template tail<3>() = std::move(trans);
}
#endif

//==============================================================================
template <typename Scalar, int Options>
template <typename RotDerived, typename TransDerived>
SE3<Scalar, Options>::SE3(
    const Eigen::MatrixBase<RotDerived>& rot,
    const Eigen::MatrixBase<TransDerived>& trans)
  : SE3(Quaternion(rot), trans)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
template <typename RotDerived, typename TransDerived>
SE3<Scalar, Options>::SE3(
    Eigen::MatrixBase<RotDerived>&& rot,
    Eigen::MatrixBase<TransDerived>&& trans)
  : SE3(Quaternion(std::move(rot)), std::move(trans))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
SE3<Scalar, Options>::SE3(Scalar x, Scalar y, Scalar z)
  : SE3(Quaternion::Identity(), Translation(x, y, z))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
const typename SE3<Scalar, Options>::DataType& SE3<Scalar, Options>::coeffs()
    const
{
  return m_data;
}

//==============================================================================
template <typename Scalar, int Options>
typename SE3<Scalar, Options>::DataType& SE3<Scalar, Options>::coeffs()
{
  return m_data;
}

} // namespace dart::math
