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

namespace dart::math {

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

  using Translation = Eigen::Matrix<Scalar, 3, 1>;
  using TranslationMap = Eigen::Map<Translation>;
  using ConstTranslationMap = Eigen::Map<const Translation>;

  using Transformation
      = Eigen::Transform<Scalar, SpaceDim, Eigen::Isometry, Options>;

  //  using EvalReturnType = typename Eval<Derived>::Type;

  //  template <typename OtherDerived>
  //  bool operator==(const SE3Base<OtherDerived>& other) const
  //  {
  //    return (position() == other.position())
  //           && (orientation() == other.orientation());
  //  }

  /// @{ @name Group operations

  template <typename OtherDerived>
  [[nodiscard]] LieGroup operator*(const SE3Base<OtherDerived>& other) const
  {
    const ConstQuaternionMap& q = quaternion();
    return LieGroup(q * other.quaternion(), q * other.position() + position());
  }

  template <typename RDerived>
  [[nodiscard]] typename RDerived::LieGroup operator*(
      const RBase<RDerived>& vector) const
  {
    // TODO(JS): Check if the dimension of vector is (3x1)
    return typename RDerived::LieGroup(orientation() * vector + position());
  }

  [[nodiscard]] LieGroup inverse() const
  {
    const SO3<Scalar> r_inv = orientation().inverse();
    return LieGroup(r_inv, -(r_inv * position()));
  }

  LieGroup& inverse_in_place()
  {
    derived() = inverse();
    return derived();
  }

  [[nodiscard]] Tangent log(
      Jacobian* jacobian, Scalar tolerance = eps<Scalar>()) const
  {
    DART_UNUSED(jacobian, tolerance);
    DART_NOT_IMPLEMENTED;
    return Tangent();
  }

  /// @}

  //  EvalReturnType eval() const
  //  {
  //    return derived().eval();
  //  }

  //  LieGroupNoAlias<Derived, SE3Base> noalias()
  //  {
  //    return LieGroupNoAlias<Derived, SE3Base>(derived());
  //  }

  [[nodiscard]] Eigen::Map<const SO3<Scalar>> orientation() const;

  [[nodiscard]] Eigen::Map<SO3<Scalar>> orientation();

  [[nodiscard]] Eigen::Map<const R3<Scalar>> position() const;

  [[nodiscard]] Eigen::Map<R3<Scalar>> position();

  [[nodiscard]] Transformation to_transformation() const;

  [[nodiscard]] Eigen::Matrix<Scalar, 4, 4> to_transformation_matrix() const;

  [[nodiscard]] Quaternion to_quaternion() const;

  [[nodiscard]] Rotation to_rotation() const;

  [[nodiscard]] Translation to_translation() const;

  /// Returns the x component of the orientation part in quaternion.
  [[nodiscard]] Scalar quat_x() const;

  /// Returns the y component of the orientation part in quaternion.
  [[nodiscard]] Scalar quat_y() const;

  /// Returns the z component of the orientation part in quaternion.
  [[nodiscard]] Scalar quat_z() const;

  /// Returns the w component of the orientation part in quaternion.
  [[nodiscard]] Scalar quat_w() const;

  /// Returns the x component of the orientation part in quaternion.
  [[nodiscard]] Scalar& quat_x();

  /// Returns the y component of the orientation part in quaternion.
  [[nodiscard]] Scalar& quat_y();

  /// Returns the z component of the orientation part in quaternion.
  [[nodiscard]] Scalar& quat_z();

  /// Returns the w component of the orientation part in quaternion.
  [[nodiscard]] Scalar& quat_w();

  /// Returns the x component of the position part.
  [[nodiscard]] Scalar x() const;

  /// Returns the y component of the position part.
  [[nodiscard]] Scalar y() const;

  /// Returns the z component of the position part.
  [[nodiscard]] Scalar z() const;

  /// Returns the x component of the position part.
  [[nodiscard]] Scalar& x();

  /// Returns the y component of the position part.
  [[nodiscard]] Scalar& y();

  /// Returns the z component of the position part.
  [[nodiscard]] Scalar& z();

  using Base::coeffs;
  using Base::data;

protected:
  [[nodiscard]] const ConstQuaternionMap quaternion() const;

  [[nodiscard]] QuaternionMap quaternion();

  [[nodiscard]] const ConstTranslationMap translation() const;

  [[nodiscard]] TranslationMap translation();

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

//==============================================================================
template <typename Derived>
Eigen::Map<const R3<typename SE3Base<Derived>::Scalar>>
SE3Base<Derived>::position() const
{
  return Eigen::Map<const R3<Scalar>>(data() + 4);
}

//==============================================================================
template <typename Derived>
Eigen::Map<R3<typename SE3Base<Derived>::Scalar>> SE3Base<Derived>::position()
{
  return Eigen::Map<R3<Scalar>>(data() + 4);
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Transformation SE3Base<Derived>::to_transformation()
    const
{
  Transformation out = Transformation::Identity();
  out.linear() = to_quaternion().toRotationMatrix();
  out.translation() = to_translation();
  return out;
}

//==============================================================================
template <typename Derived>
Eigen::Matrix<typename SE3Base<Derived>::Scalar, 4, 4>
SE3Base<Derived>::to_transformation_matrix() const
{
  return to_transformation().matrix();
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Quaternion SE3Base<Derived>::to_quaternion() const
{
  return Quaternion(coeffs().template head<4>());
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Rotation SE3Base<Derived>::to_rotation() const
{
  return Rotation(quaternion());
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::Translation SE3Base<Derived>::to_translation() const
{
  return Translation(coeffs());
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

//==============================================================================
template <typename Derived>
const typename SE3Base<Derived>::ConstQuaternionMap
SE3Base<Derived>::quaternion() const
{
  return ConstQuaternionMap(data());
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::QuaternionMap SE3Base<Derived>::quaternion()
{
  return QuaternionMap(data());
}

//==============================================================================
template <typename Derived>
const typename SE3Base<Derived>::ConstTranslationMap
SE3Base<Derived>::translation() const
{
  return ConstTranslationMap(data() + 4);
}

//==============================================================================
template <typename Derived>
typename SE3Base<Derived>::TranslationMap SE3Base<Derived>::translation()
{
  return TranslationMap(data() + 4);
}

namespace detail {

//==============================================================================
template <typename Derived>
struct NormalizationOperator<SE3Base<Derived>>
{
  template <typename T>
  static void run(T& x)
  {
    x.orientation().normalize();
    // TODO(JS): Normalize the position as well?
  }
};

//==============================================================================
template <typename Derived>
struct RandomSetter<SE3Base<Derived>>
{
  template <typename T>
  static void run(T& x)
  {
    using Scalar = typename Derived::Scalar;
    using LieGroup = typename Derived::LieGroup;
    x = LieGroup(SO3<Scalar>::Random(), R3<Scalar>::Random());
  }
};

} // namespace detail

} // namespace dart::math
