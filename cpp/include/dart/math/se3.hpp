///*
// * Copyright (c) 2011-2021, The DART development contributors:
// * https://github.com/dartsim/dart/blob/main/LICENSE
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions are met:
// *
// * 1. Redistributions of source code must retain the above copyright notice,
// * this list of conditions and the following disclaimer.
// *
// * 2. Redistributions in binary form must reproduce the above copyright
// notice,
// * this list of conditions and the following disclaimer in the documentation
// * and/or other materials provided with the distribution.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// * POSSIBILITY OF SUCH DAMAGE.
// */

//#pragma once

//#include "dart/math/constant.hpp"
//#include "dart/math/r.hpp"
//#include "dart/math/so3.hpp"
//#include "dart/math/detail/se3_base.hpp"
//#include "dart/math/type.hpp"

////==============================================================================
// template <typename Scalar, int Options>
// class SE3 : public SE3Base<SE3<Scalar, Options>>
//{
// public:
//  using Base = SE3Base<SE3<Scalar, Options>>;
//  using Matrix = typename Base::Matrix;
//  using LieAlgebra = typename Base::LieAlgebra;
//  using Tangent = typename Base::Tangent;
//  using TangentData = typename Base::TangentData;
//  using Cotangent = typename Base::Cotangent;
//  using Jacobian = typename Base::Jacobian;

//  // Eigen data
//  using Rotation = typename Base::Rotation;
//  using Translation = typename Base::Translation;
//  using Transformation = typename Base::Transformation;

//  using EvalReturnType = typename Base::EvalReturnType;

//  static SE3 Identity();

//  static SE3 Random();

//  template <typename DerivedA, typename DerivedB>
//  static Tangent Ad(
//      const SE3Base<DerivedA>& T, const SE3TangentBase<DerivedB>& V);

//  template <typename DerivedA, typename DerivedB>
//  static Tangent Ad_R(
//      const SE3Base<DerivedA>& T, const SE3TangentBase<DerivedB>& V);

//  /// Default constructor
//  SE3();

//  /// Copy constructor
//  SE3(const SE3& other) = default;

//  /// Move constructor
//  SE3(SE3&& other) = default;

//  /// Copy constructor
//  template <typename OtherDerived>
//  SE3(const SE3Base<OtherDerived>& other);

//  /// Move constructor
//  template <typename OtherDerived>
//  SE3(SE3Base<OtherDerived>&& other);

//  /// Constructs from orientation and position
//  SE3(const SO3<Scalar>& orientation, const R3<Scalar>& position);

//  /// Constructs from orientation and position
//  SE3(SO3<Scalar>&& orientation, R3<Scalar>&& position);

//  /// Constructs from orientation and position from SO3Derived and RDerived.
//  template <typename SO3Derived, typename RDerived>
//  SE3(const SO3Base<SO3Derived>& orientation, const RBase<RDerived>&
//  position);

//  /// Constructs from orientation and position from SO3Derived and RDerived
//  template <typename SO3Derived, typename RDerived>
//  SE3(SO3Base<SO3Derived>&& orientation, RBase<RDerived>&& position);

//  /// Assignment operator
//  SE3& operator=(const SE3& other);

//  /// Move operator
//  SE3& operator=(SE3&& other);

//  template <typename OtherDerived>
//  SE3& operator=(const SE3Base<OtherDerived>& other);

//  /// Assign operator for Eigen::Isometry3
//  SE3& operator=(
//      const Eigen::Transform<Scalar, 3, Eigen::Isometry, Options>& tf);

//  /// Group operation
//  template <typename OtherDerived>
//  SE3 operator*(const SE3Base<OtherDerived>& other) const;

//  /// Transforms a 3D point
//  R3<Scalar> operator*(const R3<Scalar>& position) const;

//  SE3Algebra<Scalar, Options> operator*(
//      const SE3Algebra<Scalar, Options>& dx) const;

//  void set_identity();

//  void set_random();

//  SE3Inverse<Scalar, Options> inverse() const;

//  SE3& inverse_in_place();

//  SE3& rotate(const SO3<Scalar, Options>& orientation);

//  bool is_identity() const
//  {
//    if (!m_position.is_zero()) {
//      return false;
//    }

//    if (!m_orientation.is_identity()) {
//      return false;
//    }

//    return true;
//  }

//  Tangent log(
//      Jacobian* jacobian = nullptr, Scalar tolerance = eps<Scalar>()) const;

//  Tangent ad(const Tangent& V) const;

//  Jacobian ad_matrix() const;

//  const EvalReturnType& eval() const
//  {
//    return *this;
//  }

//  SO3<Scalar>& orientation();

//  const SO3<Scalar>& orientation() const;

//  R3<Scalar>& position();

//  const R3<Scalar>& position() const;

//  Transformation transformation() const;

//  Matrix matrix() const;

//  Rotation rotation() const;

//  Translation& translation();

//  const Translation& translation() const;

//  using Base::derived;

// protected:
//  /// Orientation component
//  SO3<Scalar, Options> m_orientation;

//  /// Position component
//  R3<Scalar> m_position;
//};

// DART_TEMPLATE_CLASS_SCALAR(SE3)

////==============================================================================
// template <typename Scalar, int Options>
// class SE3Inverse : public SE3Base<SE3Inverse<Scalar, Options>>
//{
// public:
//  using Base = SE3Base<SE3Inverse<Scalar, Options>>;
//  using Matrix = typename Base::Matrix;
//  using LieAlgebra = typename Base::LieAlgebra;
//  using Tangent = typename Base::Tangent;
//  using TangentData = typename Base::TangentData;
//  using Cotangent = typename Base::Cotangent;
//  using Jacobian = typename Base::Jacobian;

//  // Eigen data
//  using Rotation = typename Base::Rotation;
//  using Translation = typename Base::Translation;
//  using Transformation = typename Base::Transformation;

//  using EvalReturnType = typename Base::EvalReturnType;

//  SE3Inverse(const SE3<Scalar, Options>& original);

//  /// Group operation
//  SE3<Scalar, Options> operator*(const SE3<Scalar, Options>& other) const;

//  SO3<Scalar> orientation() const;

//  R3<Scalar> position() const;

//  EvalReturnType eval() const;

// private:
//  const SE3<Scalar, Options>& m_original;
//};

////==============================================================================
// template <typename Scalar, int Options>
// class SE3Algebra : public SE3AlgebraBase<SE3Algebra<Scalar>>
//{
// public:
//  using This = SE3Algebra;
//  using Base = SE3AlgebraBase<SE3Algebra<Scalar>>;
//  using TangentData = typename Base::TangentData;
//  using LieGroup = typename Base::LieGroup;
//  using LieAlgebraData = typename Base::LieAlgebraData;
//  using Jacobian = typename Base::Jacobian;

//  static constexpr int GroupDim = Base::GroupDim;
//  static constexpr int MatrixDim = Base::MatrixDim;

//  static SE3Algebra Zero();

//  /// Default constructor
//  SE3Algebra();

//  /// Copy constructor
//  SE3Algebra(const SE3Algebra& other);

//  /// Move constructor
//  SE3Algebra(SE3Algebra&& other);

//  /// Constructs from the vector form of se(3)
//  explicit SE3Algebra(const SE3Tangent<Scalar, Options>& tangent);

//  /// Constructs from the vector form of se(3)
//  explicit SE3Algebra(SE3Tangent<Scalar, Options>&& tangent);

//  explicit SE3Algebra(const LieAlgebraData& data);

//  explicit SE3Algebra(LieAlgebraData&& data);

//  template <typename DerivedA, typename DerivedB>
//  explicit SE3Algebra(
//      const Eigen::MatrixBase<DerivedA>& angular,
//      const Eigen::MatrixBase<DerivedB>& linear);

//  void set_zero();

//  SE3Algebra operator/(Scalar scalar) const;

//  SE3Tangent<Scalar, Options> vee() const;

//  LieAlgebraData& mutable_matrix();

//  const LieAlgebraData& matrix() const;

// protected:
//  using Base::derived;

//  LieAlgebraData m_data;
//};

// DART_TEMPLATE_CLASS_SCALAR(SE3Algebra)

////==============================================================================
// template <typename Scalar_, int Options>
// class SE3Tangent : public SE3TangentBase<SE3Tangent<Scalar_>>
//{
// public:
//  using Scalar = Scalar_;
//  using Base = SE3TangentBase<SE3Tangent<Scalar>>;
//  using Matrix = typename Base::Matrix;
//  using TangentData = typename Base::TangentData;
//  using LieGroup = typename Base::LieGroup;
//  using LieAlgebra = typename Base::LieAlgebra;
//  using Jacobian = typename Base::Jacobian;

//  /// Default constructor
//  SE3Tangent();

//  /// Copy constructor
//  SE3Tangent(const SE3Tangent& other);

//  /// Move constructor
//  SE3Tangent(SE3Tangent&& other);

//  /// Copy constructor for coeffs
//  template <typename Derived>
//  explicit SE3Tangent(const Eigen::MatrixBase<Derived>& vector);

//  /// Move constructor for coeffs
//  template <typename Derived>
//  explicit SE3Tangent(Eigen::MatrixBase<Derived>&& vector);

//  template <typename DerivedA, typename DerivedB>
//  explicit SE3Tangent(
//      const Eigen::MatrixBase<DerivedA>& angular,
//      const Eigen::MatrixBase<DerivedB>& linear);

//  template <typename OtherDerived>
//  explicit SE3Tangent(const LieGroupTangent<OtherDerived>& other);

//  using Base::operator=;

//  SE3Tangent& operator=(const SE3Tangent& other);

//  SE3Tangent& operator=(SE3Tangent&& other);

//  Scalar operator[](int index) const;

//  Scalar& operator[](int index);

//  SE3Tangent operator-() const;

//  SE3Tangent operator+(const SE3Tangent& other) const;

//  SE3Tangent operator-(const SE3Tangent& other) const;

//  Scalar operator*(const SE3Cotangent<Scalar, Options>& wrench) const;

//  void set_zero();

//  void set_random();

//  template <typename SO3TangentDerived>
//  void set_angular(const SO3TangentBase<SO3TangentDerived>& other)
//  {
//    m_data.template head<3>() = other.vector();
//  }

//  template <typename SO3TangentDerived>
//  void set_angular(SO3TangentBase<SO3TangentDerived>&& other)
//  {
//    m_data.template head<3>() = std::move(other.vector());
//  }

//  template <typename RTangentDerived>
//  void set_linear(const RTangentBase<RTangentDerived>& other)
//  {
//    m_data.template tail<3>() = other.vector();
//  }

//  template <typename RTangentDerived>
//  void set_linear(RTangentBase<RTangentDerived>&& other)
//  {
//    m_data.template tail<3>() = std::move(other.vector());
//  }

//  template <typename MatrixDerived>
//  void set_linear(const Eigen::MatrixBase<MatrixDerived>& other)
//  {
//    m_data.template tail<3>() = other;
//  }

//  template <typename MatrixDerived>
//  void set_linear(Eigen::MatrixBase<MatrixDerived>&& other)
//  {
//    m_data.template tail<3>() = std::move(other);
//  }

//  /// Hat operation
//  SE3Algebra<Scalar, Options> hat() const;

//  /// Vee operation, reverse of hat().
//  template <typename Derived>
//  void vee(const Eigen::MatrixBase<Derived>& mat);

//  SE3<Scalar, Options> exp(
//      Jacobian* jacobian = nullptr, Scalar tolerance = eps<Scalar>()) const;

//  template <typename Derived>
//  SE3Tangent ad(const SE3TangentBase<Derived>& other) const;

//  Matrix ad_matrix() const;

//  Jacobian left_jacobian(Scalar tolerance = eps<Scalar>()) const;

//  Jacobian space_jacobian(Scalar tolerance = eps<Scalar>()) const;

//  Jacobian right_jacobian(Scalar tolerance = eps<Scalar>()) const;

//  Jacobian body_jacobian(Scalar tolerance = eps<Scalar>()) const;

//  Jacobian left_jacobian_inverse(Scalar tolerance = eps<Scalar>()) const;

//  Jacobian right_jacobian_inverse(Scalar tolerance = eps<Scalar>()) const;

//  SO3Tangent<Scalar, Options> angular() const;

//  Eigen::Map<SO3Tangent<Scalar, Options>> angular();

//  RTangent<Scalar, 3, Options> linear() const;

//  Eigen::Map<R3Tangent<Scalar, Options>> linear();

//  TangentData& vector();

//  const TangentData& vector() const;

//  static SE3Tangent Random();

// protected:
//  using Base::derived;

//  TangentData m_data;
//};

// DART_TEMPLATE_CLASS_SCALAR(SE3Tangent)

////==============================================================================
// template <typename Scalar, int Options_>
// class SE3Cotangent : public SE3CotangentBase<SE3Cotangent<Scalar>>
//{
// public:
//  using Base = SE3CotangentBase<SE3Cotangent<Scalar>>;
//  using CotangentData = typename Base::CotangentData;
//  using LieGroup = typename Base::LieGroup;
//  using LieAlgebra = typename Base::LieAlgebra;
//  using Jacobian = typename Base::Jacobian;

//  /// Default constructor
//  SE3Cotangent();

//  /// Copy constructor
//  template <typename Derived>
//  SE3Cotangent(const SE3CotangentBase<Derived>& other) :
//  m_data(other.vector())
//  {
//    // Do nothing
//  }

//  /// Move constructor
//  template <typename Derived>
//  SE3Cotangent(SE3Cotangent<Derived>&& other)
//    : m_data(std::move(other.vector()))
//  {
//    // Do nothing
//  }

//  /// Constructs from Eigen::MatrixBase
//  template <typename MatrixDerived>
//  SE3Cotangent(const Eigen::MatrixBase<MatrixDerived>& other) : m_data(other)
//  {
//    // Do nothing
//  }

//  /// Constructs from Eigen::MatrixBase
//  template <typename MatrixDerived>
//  SE3Cotangent(Eigen::MatrixBase<MatrixDerived>&& other)
//    : m_data(std::move(other))
//  {
//    // Do nothing
//  }

//  auto torque() const;

//  auto mutable_torque();

//  auto force() const;

//  auto mutable_force();

//  auto angular() const;

//  auto mutable_angular();

//  auto linear() const;

//  auto mutable_linear();

//  CotangentData& vector();

//  const CotangentData& vector() const;

// protected:
//  using Base::derived;

//  CotangentData m_data;
//};

// DART_TEMPLATE_CLASS_SCALAR(SE3Cotangent)

//} // namespace dart::math

// namespace Eigen {

////==============================================================================
// template <typename Scalar_, int Options>
// class Map<dart::math::SE3<Scalar_, Options>, Options>
//  : public dart::math::SE3Base<Map<dart::math::SE3<Scalar_, Options>,
//  Options>>
//{
// public:
//  using Scalar = Scalar_;
//  using Base
//      = dart::math::SE3Base<Map<dart::math::SE3<Scalar_, Options>, Options>>;

//  using Base::operator=;

//  Map(Scalar* data);

// private:
//  Map<dart::math::SO3<Scalar, Options>> m_orientation;
//  Map<dart::math::R3<Scalar, Options>> m_position;
//};

////==============================================================================
// template <typename Scalar_, int Options>
// class Map<const dart::math::SE3<Scalar_, Options>, Options>
//  : public dart::math::SE3Base<
//        Map<const dart::math::SE3<Scalar_, Options>, Options>>
//{
// public:
//  using Scalar = Scalar_;
//  using Base = dart::math::SE3Base<
//      Map<const dart::math::SE3<Scalar_, Options>, Options>>;

//  using Base::operator=;

//  Map(const Scalar* data);

// private:
//  Map<const dart::math::SO3<Scalar, Options>> m_orientation;
//  Map<const dart::math::R3<Scalar, Options>> m_position;
//};

//} // namespace Eigen

//#include "dart/math/detail/se3_impl.hpp"
