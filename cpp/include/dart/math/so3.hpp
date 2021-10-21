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

#include "dart/common/macro.hpp"
#include "dart/math/constant.hpp"
#include "dart/math/detail/so3_base.hpp"
#include "dart/math/export.hpp"
#include "dart/math/r.hpp"
#include "dart/math/type.hpp"

namespace dart::math {

//==============================================================================
template <typename Scalar, int Options>
class SO3 : public SO3Base<SO3<Scalar, Options>>
{
public:
  using Base = SO3Base<SO3<Scalar, Options>>;

  using Matrix = typename Base::Matrix;
  using Quaternion = math::Quaternion<Scalar, Options>;

  using Tangent = typename Base::Tangent;
  using Cotangent = typename Base::Cotangent;
  using Jacobian = typename Base::Jacobian;
  using QuaternionType = Eigen::Quaternion<Scalar, Options>;

  // Eigen data
  using Rotation = typename Base::Rotation;
  using Translation = typename Base::Translation;
  using Transformation = typename Base::Transformation;

  static constexpr int GroupDim = Base::GroupDim;

  /// Creates an identity element
  static SO3<Scalar, Options> Identity();

  /// Creates an random element where it's sampled from an uniform distribution
  /// on SO(3).
  static SO3<Scalar, Options> Random();

  /// Default constructor
  SO3();

  /// Copy constructor
  SO3(const SO3& other) = default;

  /// Move constructor
  SO3(SO3&& other) = default;

  /// Copy constructor from OtherDerived
  template <typename OtherDerived>
  SO3(const SO3Base<OtherDerived>& other);

  /// Move constructor from OtherDerived
  template <typename OtherDerived>
  SO3(SO3Base<OtherDerived>&& other);

  /// Constructs from quaternion
  SO3(const math::Quaternion<Scalar, Options>& quat);

  /// Constructs from quaternion
  SO3(math::Quaternion<Scalar, Options>&& quat);

  /// Destructor
  ~SO3() = default;

  /// Assign operator
  SO3& operator=(const SO3& other);

  /// Move operator
  SO3& operator=(SO3&& other);

  template <typename Derived>
  SO3& operator=(const Eigen::MatrixBase<Derived>& matrix);

  /// Group operation
  SO3 operator*(const SO3& other) const;

  R3<Scalar> operator*(const R3<Scalar>& other) const;

  SO3Algebra<Scalar, Options> operator*(
      const SO3Algebra<Scalar, Options>& dx) const;

  /// @{ @name Euler angles

  Eigen::Matrix<Scalar, 3, 1> euler_angles(
      int axis1, int axis2, int axis3) const;

  Eigen::Matrix<Scalar, 3, 1> euler_angles_intrinsic(
      int axis1, int axis2, int axis3) const;

  Eigen::Matrix<Scalar, 3, 1> euler_angles_extrinsic(
      int axis1, int axis2, int axis3) const;

  Eigen::Matrix<Scalar, 3, 1> rpy() const;

  /// @}

  void set_identity();

  void set_random();

  SO3 inverse() const;

  SO3& inverse_in_place();

  bool is_identity() const
  {
    return (m_data.coeffs() == Quaternion::Identity().coeffs());
  }

  Tangent log(
      Jacobian* jacobian = nullptr, Scalar tolerance = eps<Scalar>()) const;

  Tangent ad(const Tangent& V) const;

  Jacobian ad_matrix() const;

  Transformation transformation() const;

  Matrix matrix() const;

  Rotation rotation() const;

  Translation translation() const;

  const Quaternion& quaternion() const;

protected:
  using Base::derived;

  Quaternion m_data;

private:
  void normalize();
};

DART_TEMPLATE_CLASS_SCALAR(SO3)

//==============================================================================
template <typename Derived>
class SO3AlgebraBase
{
public:
  using TangentData = typename Eigen::internal::traits<Derived>::TangentData;
  using LieGroup = typename Eigen::internal::traits<Derived>::LieGroup;
  using LieAlgebraData =
      typename Eigen::internal::traits<Derived>::LieAlgebraData;
  using Jacobian = typename Eigen::internal::traits<Derived>::Jacobian;

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
template <typename Scalar, int Options>
class SO3Algebra : public SO3AlgebraBase<SO3Algebra<Scalar, Options>>
{
public:
  using Base = SO3AlgebraBase<SO3Algebra<Scalar, Options>>;
  using TangentData = typename Base::TangentData;
  using LieGroup = typename Base::LieGroup;
  using LieAlgebraData = typename Base::LieAlgebraData;
  using Jacobian = typename Base::Jacobian;

  static constexpr int GroupDim = Base::GroupDim;
  static constexpr int MatrixDim = Base::MatrixDim;

  /// Default constructor
  SO3Algebra();

  /// Copy constructor
  SO3Algebra(const SO3Algebra& other);

  /// Move constructor
  SO3Algebra(SO3Algebra&& other);

  /// Constructs from 3x3 matrix
  template <typename Derived>
  explicit SO3Algebra(const Eigen::MatrixBase<Derived>& matrix);

  /// Constructs from 3x3 matrix
  template <typename Derived>
  explicit SO3Algebra(Eigen::MatrixBase<Derived>&& matrix);

  SO3Algebra operator/(Scalar scalar) const;

  SO3Tangent<Scalar, Options> vee() const;

  LieAlgebraData& mutable_matrix();

  const LieAlgebraData& matrix() const;

protected:
  using Base::derived;

  LieAlgebraData m_matrix;
};

DART_TEMPLATE_CLASS_SCALAR(SO3Algebra)

//==============================================================================
template <typename Derived>
class SO3TangentBase
{
public:
  using Tangent = typename Eigen::internal::traits<Derived>::Tangent;
  using TangentData = typename Eigen::internal::traits<Derived>::TangentData;
  using LieGroup = typename Eigen::internal::traits<Derived>::LieGroup;
  using LieAlgebra = typename Eigen::internal::traits<Derived>::LieAlgebra;
  using LieAlgebraData =
      typename Eigen::internal::traits<Derived>::LieAlgebraData;
  using Jacobian = typename Eigen::internal::traits<Derived>::Jacobian;

  template <typename OtherDerived>
  Derived& operator=(const SO3TangentBase<OtherDerived>& other)
  {
    derived() = other;
    return derived();
  }

  template <typename OtherDerived>
  Derived& operator=(SO3TangentBase<OtherDerived>&& other)
  {
    derived() = std::move(other);
    return derived();
  }

  template <typename OtherDerived>
  Derived& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    derived().vector() = other;
    return derived();
  }

  template <typename OtherDerived>
  Derived& operator=(Eigen::MatrixBase<OtherDerived>&& other)
  {
    derived().vector() = std::move(other);
    return derived();
  }

  bool is_zero() const
  {
    return derived().vector().isZero();
  }

  template <typename OtherDerived>
  Tangent cross(const RBase<OtherDerived>& other) const
  {
    return derived().vector().cross(other.vector());
  }

  const auto& vector() const
  {
    return derived().vector();
  }

  auto& vector()
  {
    return derived().vector();
  }

  friend std::ostream& operator<<(
      std::ostream& os, const SO3TangentBase<Derived>& x)
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
template <typename Scalar, int Options>
class SO3Tangent : public SO3TangentBase<SO3Tangent<Scalar, Options>>
{
public:
  using This = SO3Tangent;
  using Base = SO3TangentBase<SO3Tangent<Scalar, Options>>;
  using QuaternionType = typename Eigen::internal::traits<This>::QuaternionType;
  using TangentData = typename Base::TangentData;
  using LieGroup = typename Base::LieGroup;
  using LieAlgebra = typename Base::LieAlgebra;
  using Jacobian = typename Base::Jacobian;

  static constexpr int GroupDim = Base::GroupDim;
  static constexpr int MatrixDim = Base::MatrixDim;

  /// Default constructor
  SO3Tangent();

  /// Copy constructor
  template <typename Derived>
  SO3Tangent(const SO3TangentBase<Derived>& other);

  /// Move constructor
  template <typename Derived>
  SO3Tangent(SO3TangentBase<Derived>&& other);

  /// Copy constructor for coeffs
  template <typename Derived>
  SO3Tangent(const Eigen::MatrixBase<Derived>& coeffs);

  /// Move constructor for coeffs
  template <typename Derived>
  SO3Tangent(Eigen::MatrixBase<Derived>&& coeffs);

  template <typename Derived>
  SO3Tangent& operator=(const SO3TangentBase<Derived>& other);

  template <typename Derived>
  SO3Tangent& operator=(SO3TangentBase<Derived>&& other);

  template <typename Derived>
  SO3Tangent& operator=(const Eigen::MatrixBase<Derived>& other)
  {
    m_data = other;
  }

  template <typename Derived>
  SO3Tangent& operator=(Eigen::MatrixBase<Derived>&& other)
  {
    m_data = std::move(other);
  }

  Scalar operator[](int index) const;

  Scalar& operator[](int index);

  SO3Tangent operator-() const;

  SO3Tangent operator+(const SO3Tangent& other) const;

  Scalar operator*(const SO3Cotangent<Scalar, Options>& torque) const;

  void set_zero();

  void set_random();

  SO3Algebra<Scalar, Options> hat() const;

  LieGroup exp(
      Jacobian* jacobian = nullptr, Scalar tolerance = eps<Scalar>()) const;

  Jacobian left_jacobian(Scalar tolerance = eps<Scalar>()) const;

  Jacobian space_jacobian(Scalar tolerance = eps<Scalar>()) const;

  Jacobian right_jacobian(Scalar tolerance = eps<Scalar>()) const;

  Jacobian body_jacobian(Scalar tolerance = eps<Scalar>()) const;

  Jacobian left_jacobian_inverse(Scalar tolerance = eps<Scalar>()) const;

  Jacobian right_jacobian_inverse(Scalar tolerance = eps<Scalar>()) const;

  template <typename Derived>
  Jacobian left_jacobian_time_derivative(
      const math::MatrixBase<Derived>& dq,
      Scalar tolerance = eps<Scalar>()) const;

  Jacobian left_jacobian_time_derivative(
      int index, Scalar tolerance = eps<Scalar>()) const;

  std::array<Jacobian, 3> left_jacobian_time_derivative(
      Scalar tolerance = eps<Scalar>()) const;

  template <typename Derived>
  Jacobian right_jacobian_time_derivative(
      const math::MatrixBase<Derived>& dq,
      Scalar tolerance = eps<Scalar>()) const;

  Jacobian right_jacobian_time_derivative(
      int index, Scalar tolerance = eps<Scalar>()) const;

  std::array<Jacobian, 3> right_jacobian_time_derivative(
      Scalar tolerance = eps<Scalar>()) const;

  SO3Tangent ad(const SO3Tangent& other) const;

  Jacobian ad_matrix() const;

  TangentData& vector();

  const TangentData& vector() const;

  static SO3Tangent<Scalar, Options> Random();

protected:
  TangentData m_data;
};

DART_TEMPLATE_CLASS_SCALAR(SO3Tangent)

//==============================================================================
template <typename Derived>
class SO3CotangentBase
{
public:
  using TangentData = typename Eigen::internal::traits<Derived>::TangentData;
  using CotangentData =
      typename Eigen::internal::traits<Derived>::CotangentData;
  using LieGroup = typename Eigen::internal::traits<Derived>::LieGroup;
  using LieAlgebra = typename Eigen::internal::traits<Derived>::LieAlgebra;
  using LieAlgebraData =
      typename Eigen::internal::traits<Derived>::LieAlgebraData;
  using Jacobian = typename Eigen::internal::traits<Derived>::Jacobian;

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
template <typename Scalar, int Options>
class SO3Cotangent : public SO3CotangentBase<SO3Cotangent<Scalar, Options>>
{
public:
  using Base = SO3CotangentBase<SO3Cotangent<Scalar, Options>>;
  using LieGroup = typename Base::LieGroup;
  using CotangentData = typename Base::CotangentData;
  using LieAlgebra = typename Base::LieAlgebra;
  using Jacobian = typename Base::Jacobian;

  static constexpr int GroupDim = Base::GroupDim;
  static constexpr int MatrixDim = Base::MatrixDim;

  CotangentData& vector();
  const CotangentData& vector() const;

protected:
  CotangentData m_data;
};

DART_TEMPLATE_CLASS_SCALAR(SO3Cotangent)

} // namespace dart::math

namespace Eigen {

//==============================================================================
template <typename Scalar_, int Options>
class Map<dart::math::SO3<Scalar_, Options>, Options>
  : public dart::math::SO3Base<Map<dart::math::SO3<Scalar_, Options>, Options>>
{
public:
  using Scalar = Scalar_;
  using Base
      = dart::math::SO3Base<Map<dart::math::SO3<Scalar_, Options>, Options>>;

  /// Constructor
  ///
  /// @param[in] data: Pointer to an array of scalar values that this map to use
  /// as its underlying data. The size should be at least 4.
  Map(Scalar* data);

  using Base::operator=;

  const Map<Eigen::Quaternion<Scalar, Options>, Options>& quaternion() const;

  Map<Eigen::Quaternion<Scalar, Options>, Options>& quaternion();

private:
  Map<Eigen::Quaternion<Scalar, Options>> m_data;
};

//==============================================================================
template <typename Scalar_, int Options>
class Map<const dart::math::SO3<Scalar_, Options>, Options>
  : public dart::math::SO3Base<
        Map<const dart::math::SO3<Scalar_, Options>, Options>>
{
public:
  using Scalar = Scalar_;
  using Base = dart::math::SO3Base<
      Map<const dart::math::SO3<Scalar_, Options>, Options>>;

  /// Constructor
  ///
  /// @param[in] data: Pointer to an array of scalar values that this map to use
  /// as its underlying data. The size should be at least 4.
  Map(const Scalar* data);

  using Base::operator=;

  /// Returns
  const Map<Eigen::Quaternion<Scalar, Options>, Options>& quaternion() const;

private:
  Map<const Eigen::Quaternion<Scalar, Options>> m_data;
};

//==============================================================================
template <typename Scalar_, int Options>
class Map<dart::math::SO3Tangent<Scalar_, Options>, Options>
  : public dart::math::SO3TangentBase<
        Map<dart::math::SO3Tangent<Scalar_, Options>, Options>>
{
public:
  using Scalar = Scalar_;
  using Base = dart::math::SO3TangentBase<
      Map<dart::math::SO3Tangent<Scalar_, Options>, Options>>;
  using This = Map<dart::math::SO3Tangent<Scalar_, Options>, Options>;

  /// Constructor
  ///
  /// @param[in] data: Pointer to an array of scalar values that this map to use
  /// as its underlying data. The size should be at least 4.
  Map(Scalar* data);

  using Base::operator=;
  using Base::cross;

  template <typename Derived>
  This& operator=(const Eigen::MatrixBase<Derived>& other)
  {
    m_data = other;
    return *this;
  }

  template <typename Derived>
  This& operator=(Eigen::MatrixBase<Derived>&& other)
  {
    m_data = std::move(other);
    return *this;
  }

  const Map<typename Base::TangentData>& vector() const
  {
    return m_data;
  }

  Map<typename Base::TangentData>& vector()
  {
    return m_data;
  }

private:
  Map<typename Base::TangentData> m_data;
};

//==============================================================================
template <typename Scalar_, int Options>
class Map<const dart::math::SO3Tangent<Scalar_, Options>, Options>
  : public dart::math::SO3TangentBase<
        Map<const dart::math::SO3Tangent<Scalar_, Options>, Options>>
{
public:
  using Scalar = Scalar_;
  using Base = dart::math::SO3TangentBase<
      Map<const dart::math::SO3Tangent<Scalar_, Options>, Options>>;
  using This = Map<const dart::math::SO3Tangent<Scalar_, Options>, Options>;

  /// Constructor
  ///
  /// @param[in] data: Pointer to an array of scalar values that this map to use
  /// as its underlying data. The size should be at least 4.
  Map(const Scalar* data);

  using Base::cross;

  const Map<const typename Base::TangentData>& vector() const
  {
    return m_data;
  }

private:
  Map<const typename Base::TangentData> m_data;
};

} // namespace Eigen

#include "dart/math/detail/so3_impl.hpp"
