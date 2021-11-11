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

#include <Eigen/Geometry>

#include "dart/math/lie_group/detail/lie_group_base.hpp"
#include "dart/math/lie_group/detail/macro.hpp"
#include "dart/math/lie_group/type.hpp"
#include "dart/math/type.hpp"

namespace dart::math {

//==============================================================================
template <typename Derived>
class SO3Base : public LieGroupBase<Derived>
{
public:
  using This = SO3Base<Derived>;
  using Base = LieGroupBase<Derived>;

  DART_LIE_GROUP_USE_BASE_TYPES;

  using Quaternion = math::Quaternion<Scalar, Options>;
  using QuaternionMap = Eigen::Map<Quaternion>;
  using ConstQuaternionMap = Eigen::Map<const Quaternion>;

  using Rotation = Eigen::Matrix<Scalar, 3, 3>;

  using Translation = Eigen::Matrix<Scalar, 3, 1>;
  using TranslationMap = Eigen::Map<Translation>;
  using ConstTranslationMap = Eigen::Map<const Translation>;

  using Transformation
      = Eigen::Transform<Scalar, SpaceDim, Eigen::Isometry, Options>;

  DART_LIE_GROUP_BASE_ASSIGN_OPERATORS(SO3Base);

  /// @{ @name Group operations

  template <typename OtherDerived>
  [[nodiscard]] LieGroup operator*(const SO3Base<OtherDerived>& other) const;

  template <typename RDerived>
  [[nodiscard]] typename RDerived::LieGroup operator*(
      const RBase<RDerived>& vector) const;

  [[nodiscard]] LieGroup inverse() const;

  Derived& inverse_in_place();

  [[nodiscard]] Tangent log(
      Jacobian* jacobian = nullptr, Scalar tolerance = eps<Scalar>()) const;

  /// @}

  void set_from_quaternion(Scalar w, Scalar x, Scalar y, Scalar z);

  void set_from_quaternion(const Eigen::Quaternion<Scalar>& q);

  /// @{ @name Euler angles

  /// @return Euler angles in the ranges [0,pi]x[-pi,pi]x[-pi,pi]
  [[nodiscard]] Eigen::Matrix<Scalar, 3, 1> rpy() const;

  void set_from_euler_angles_intrinsic(
      Scalar angle0,
      Scalar angle1,
      Scalar angle2,
      int axis0 = 0,
      int axis1 = 1,
      int axis2 = 2);

  void set_from_euler_angles_extrinsic(
      Scalar angle0,
      Scalar angle1,
      Scalar angle2,
      int axis0 = 0,
      int axis1 = 1,
      int axis2 = 2);

  /// @return Euler angles in the ranges [0,pi]x[-pi,pi]x[-pi,pi]
  [[nodiscard]] Eigen::Matrix<Scalar, 3, 1> euler_angles(
      int axis1 = 0, int axis2 = 1, int axis3 = 2) const;

  /// @return Euler angles in the ranges [0,pi]x[-pi,pi]x[-pi,pi]
  [[nodiscard]] Eigen::Matrix<Scalar, 3, 1> euler_angles_intrinsic(
      int axis1 = 0, int axis2 = 1, int axis3 = 2) const;

  /// @return Euler angles in the ranges [0,pi]x[-pi,pi]x[-pi,pi]
  [[nodiscard]] Eigen::Matrix<Scalar, 3, 1> euler_angles_extrinsic(
      int axis1 = 0, int axis2 = 1, int axis3 = 2) const;

  template <typename MatrixDrived>
  void set_from_rpy(const Eigen::MatrixBase<MatrixDrived>& angles);

  /// @}

  void normalize();

  /// Returns the w component of the in quaternion.
  [[nodiscard]] Scalar w() const;

  /// Returns the x component of the in quaternion.
  [[nodiscard]] Scalar x() const;

  /// Returns the y component of the in quaternion.
  [[nodiscard]] Scalar y() const;

  /// Returns the z component of the in quaternion.
  [[nodiscard]] Scalar z() const;

  /// Returns the w component of the in quaternion.
  [[nodiscard]] Scalar& w();

  /// Returns the x component of the in quaternion.
  [[nodiscard]] Scalar& x();

  /// Returns the y component of the in quaternion.
  [[nodiscard]] Scalar& y();

  /// Returns the z component of the in quaternion.
  [[nodiscard]] Scalar& z();

  using Base::coeffs;
  using Base::data;

  Quaternion to_quaternion() const;

  Rotation to_rotation_matrix() const;

  const ConstQuaternionMap quaternion() const;

  QuaternionMap quaternion();

protected:
  using Base::derived;
};

//==============================================================================
template <typename Derived>
template <typename OtherDerived>
typename SO3Base<Derived>::LieGroup SO3Base<Derived>::operator*(
    const SO3Base<OtherDerived>& other) const
{
  return LieGroup(quaternion() * other.quaternion());
}

//==============================================================================
template <typename Derived>
template <typename RDerived>
typename RDerived::LieGroup SO3Base<Derived>::operator*(
    const RBase<RDerived>& vector) const
{
  // TODO(JS): Check if the dimension of vector is (3x1)
  return typename RDerived::LieGroup(quaternion() * vector.coeffs());
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::LieGroup SO3Base<Derived>::inverse() const
{
  return LieGroup(quaternion().conjugate());
}

//==============================================================================
template <typename Derived>
Derived& SO3Base<Derived>::inverse_in_place()
{
  quaternion() = quaternion().conjugate();
  return derived();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Tangent SO3Base<Derived>::log(
    Jacobian* jacobian, Scalar tolerance) const
{
  const Scalar theta = 2 * std::acos(w());
  DART_ASSERT(!std::isnan(theta));

  typename Tangent::DataType data;
  if (theta < tolerance) {
    data.noalias() = 2 * quaternion().vec();
  } else {
    DART_ASSERT(std::sin(0.5 * theta));
    data.noalias() = theta * quaternion().vec() / std::sin(0.5 * theta);
  }

  const Tangent out = Tangent(std::move(data));

  if (jacobian) {
    (*jacobian) = out.left_jacobian(tolerance);
  }

  return out;
}

//==============================================================================
template <typename Derived>
void SO3Base<Derived>::set_from_quaternion(
    Scalar w, Scalar x, Scalar y, Scalar z)
{
  w() = w;
  x() = x;
  y() = y;
  z() = z;
  normalize();
}

//==============================================================================
template <typename Derived>
void SO3Base<Derived>::set_from_quaternion(const Eigen::Quaternion<Scalar>& q)
{
  coeffs() = q.coeffs();
  normalize();
}

//==============================================================================
template <typename Derived>
Eigen::Matrix<typename SO3Base<Derived>::Scalar, 3, 1>
SO3Base<Derived>::euler_angles(int axis1, int axis2, int axis3) const
{
  return euler_angles_intrinsic(axis1, axis2, axis3);
}

//==============================================================================
template <typename Derived>
Eigen::Matrix<typename SO3Base<Derived>::Scalar, 3, 1>
SO3Base<Derived>::euler_angles_intrinsic(int axis1, int axis2, int axis3) const
{
  return quaternion().toRotationMatrix().eulerAngles(axis1, axis2, axis3);
}

//==============================================================================
template <typename Derived>
Eigen::Matrix<typename SO3Base<Derived>::Scalar, 3, 1>
SO3Base<Derived>::euler_angles_extrinsic(int axis1, int axis2, int axis3) const
{
  return quaternion()
      .toRotationMatrix()
      .eulerAngles(axis3, axis2, axis1)
      .reverse();
}

//==============================================================================
template <typename Derived>
template <typename MatrixDrived>
void SO3Base<Derived>::set_from_rpy(
    const Eigen::MatrixBase<MatrixDrived>& angles)
{
  quaternion() = Quaternion(
      Eigen::AngleAxis<Scalar>(angles[2], Eigen::Matrix<Scalar, 3, 1>::UnitX())
      * Eigen::AngleAxis<Scalar>(
          angles[1], Eigen::Matrix<Scalar, 3, 1>::UnitY())
      * Eigen::AngleAxis<Scalar>(
          angles[0], Eigen::Matrix<Scalar, 3, 1>::UnitZ()));
  normalize();
}

//==============================================================================
template <typename Derived>
Eigen::Matrix<typename SO3Base<Derived>::Scalar, 3, 1> SO3Base<Derived>::rpy()
    const
{
  return euler_angles_intrinsic(2, 1, 0);
}

//==============================================================================
template <typename Derived>
void SO3Base<Derived>::set_from_euler_angles_intrinsic(
    Scalar angle0,
    Scalar angle1,
    Scalar angle2,
    int axis0,
    int axis1,
    int axis2)
{
  Eigen::Matrix<Scalar, 3, 1> a0 = Eigen::Matrix<Scalar, 3, 1>::Zero();
  Eigen::Matrix<Scalar, 3, 1> a1 = Eigen::Matrix<Scalar, 3, 1>::Zero();
  Eigen::Matrix<Scalar, 3, 1> a2 = Eigen::Matrix<Scalar, 3, 1>::Zero();
  a0(axis0) = 1;
  a1(axis1) = 1;
  a2(axis2) = 1;
  set_from_quaternion(
      Eigen::AngleAxis<Scalar>(angle0, a0)
      * Eigen::AngleAxis<Scalar>(angle1, a1)
      * Eigen::AngleAxis<Scalar>(angle2, a2));
}

//==============================================================================
template <typename Derived>
void SO3Base<Derived>::set_from_euler_angles_extrinsic(
    Scalar angle0,
    Scalar angle1,
    Scalar angle2,
    int axis0,
    int axis1,
    int axis2)
{
  set_from_euler_angles_intrinsic(angle2, angle1, angle0, axis2, axis1, axis0);
}

//==============================================================================
template <typename Derived>
void SO3Base<Derived>::normalize()
{
  if (coeffs().w() < 0) {
    coeffs() *= -1;
  }
  coeffs().normalize();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Scalar SO3Base<Derived>::w() const
{
  return coeffs().w();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Scalar SO3Base<Derived>::x() const
{
  return coeffs().x();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Scalar SO3Base<Derived>::y() const
{
  return coeffs().y();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Scalar SO3Base<Derived>::z() const
{
  return coeffs().z();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Scalar& SO3Base<Derived>::w()
{
  return coeffs().w();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Scalar& SO3Base<Derived>::x()
{
  return coeffs().x();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Scalar& SO3Base<Derived>::y()
{
  return coeffs().y();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Scalar& SO3Base<Derived>::z()
{
  return coeffs().z();
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Quaternion SO3Base<Derived>::to_quaternion() const
{
  return Quaternion(coeffs());
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::Rotation SO3Base<Derived>::to_rotation_matrix() const
{
  return to_quaternion().toRotationMatrix();
}

//==============================================================================
template <typename Derived>
const typename SO3Base<Derived>::ConstQuaternionMap
SO3Base<Derived>::quaternion() const
{
  return ConstQuaternionMap(data());
}

//==============================================================================
template <typename Derived>
typename SO3Base<Derived>::QuaternionMap SO3Base<Derived>::quaternion()
{
  return QuaternionMap(data());
}

namespace detail {

//==============================================================================
template <typename Derived>
struct NormalizationOperator<SO3Base<Derived>>
{
  template <typename T>
  static void run(T& x)
  {
    x.normalize();
  }
};

//==============================================================================
template <typename Derived>
struct RandomSetter<SO3Base<Derived>>
{
  template <typename T>
  static void run(T& x)
  {
    using Scalar = typename SO3Base<Derived>::Scalar;
    using LieGroup = typename SO3Base<Derived>::LieGroup;

    x = LieGroup(Eigen::Quaternion<Scalar>::UnitRandom());
  }
};

} // namespace detail

} // namespace dart::math
