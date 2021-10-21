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

#include "dart/math/r.hpp"
#include "dart/math/se3.hpp"

namespace dart::math {

//==============================================================================
template <typename Scalar, int Options>
SE3<Scalar, Options> SE3<Scalar, Options>::Identity()
{
  return SE3(SO3<Scalar, Options>::Identity(), R3<Scalar, Options>::Identity());
}

//==============================================================================
template <typename Scalar, int Options>
SE3<Scalar, Options> SE3<Scalar, Options>::Random()
{
  return SE3(SO3<Scalar, Options>::Random(), R3<Scalar, Options>::Random());
}

//==============================================================================
template <typename Scalar, int Options>
SE3<Scalar, Options>::SE3()
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
template <typename OtherDerived>
SE3<Scalar, Options>::SE3(const SE3Base<OtherDerived>& other)
  : m_orientation(other.orientation()), m_position(other.position())
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
template <typename OtherDerived>
SE3<Scalar, Options>::SE3(SE3Base<OtherDerived>&& other)
  : m_orientation(std::move(other.orientation())),
    m_position(std::move(other.position()))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
SE3<Scalar, Options>::SE3(
    const SO3<Scalar>& orientation, const R3<Scalar>& position)
  : m_orientation(orientation), m_position(position)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
SE3<Scalar, Options>::SE3(SO3<Scalar>&& orientation, R3<Scalar>&& position)
  : m_orientation(std::move(orientation)), m_position(std::move(position))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
template <typename SO3Derived, typename RDerived>
SE3<Scalar, Options>::SE3(
    const SO3Base<SO3Derived>& orientation, const RBase<RDerived>& position)
  : m_orientation(orientation), m_position(position)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
template <typename SO3Derived, typename RDerived>
SE3<Scalar, Options>::SE3(
    SO3Base<SO3Derived>&& orientation, RBase<RDerived>&& position)
  : m_orientation(std::move(orientation)), m_position(std::move(position))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
SE3<Scalar, Options>& SE3<Scalar, Options>::operator=(
    const SE3<Scalar, Options>& other)
{
  m_orientation = other.m_orientation;
  m_position = other.m_position;
  return *this;
}

//==============================================================================
template <typename Scalar, int Options>
SE3<Scalar, Options>& SE3<Scalar, Options>::operator=(
    SE3<Scalar, Options>&& other)
{
  m_orientation = std::move(other.m_orientation);
  m_position = std::move(other.m_position);
  return *this;
}

//==============================================================================
template <typename Scalar, int Options>
template <typename OtherDerived>
SE3<Scalar, Options>& SE3<Scalar, Options>::operator=(
    const SE3Base<OtherDerived>& other)
{
  *this = other.eval();
  return *this;
}

//==============================================================================
template <typename Scalar, int Options>
SE3<Scalar, Options>& SE3<Scalar, Options>::operator=(
    const Eigen::Transform<Scalar, 3, Eigen::Isometry, Options>& tf)
{
  m_orientation = tf.rotation();
  m_position = tf.translation();
  return *this;
}

//==============================================================================
template <typename Scalar, int Options>
template <typename OtherDerived>
SE3<Scalar, Options> SE3<Scalar, Options>::operator*(
    const SE3Base<OtherDerived>& other) const
{
  // TODO(JS): Fix this not to use .eval()
  const auto other_evaluated = other.eval();
  return SE3<Scalar, Options>(
      m_orientation * other_evaluated.orientation(),
      m_position + m_orientation * other_evaluated.position());
}

//==============================================================================
template <typename Scalar, int Options>
R3<Scalar> SE3<Scalar, Options>::operator*(const R3<Scalar>& position) const
{
  return m_orientation * position + m_position;
}

//==============================================================================
template <typename Scalar, int Options>
SE3Algebra<Scalar, Options> SE3<Scalar, Options>::operator*(
    const SE3Algebra<Scalar, Options>& dx) const
{
  return SE3Algebra<Scalar, Options>(matrix() * dx.matrix());
}

//==============================================================================
template <typename Scalar, int Options>
void SE3<Scalar, Options>::set_identity()
{
  m_orientation.set_identity();
  m_position.set_identity();
}

//==============================================================================
template <typename Scalar, int Options>
void SE3<Scalar, Options>::set_random()
{
  m_orientation.set_random();
  m_position.set_random();
}

//==============================================================================
template <typename Scalar, int Options>
SE3Inverse<Scalar, Options> SE3<Scalar, Options>::inverse() const
{
  return SE3Inverse<Scalar, Options>(*this);
}

//==============================================================================
template <typename Scalar, int Options>
SE3<Scalar, Options>& SE3<Scalar, Options>::inverse_in_place()
{
  *this = inverse();
  return *this;
}

//==============================================================================
template <typename Scalar, int Options>
SE3<Scalar, Options>& SE3<Scalar, Options>::rotate(
    const SO3<Scalar, Options>& orientation)
{
  m_orientation = orientation * m_orientation;
  m_position = orientation * m_position;
  return *this;
}

//==============================================================================
template <typename Scalar, int Options>
typename SE3<Scalar, Options>::Tangent SE3<Scalar, Options>::log(
    Jacobian* jacobian, Scalar tolerance) const
{
  Tangent out;
  out.set_angular(m_orientation.log(nullptr, tolerance));
  out.set_linear(
      SO3Tangent<Scalar>(out.angular()).left_jacobian_inverse(tolerance)
      * m_position.vector());
  // TODO(JS): Use map

  if (jacobian) {
    (*jacobian) = out.right_jacobian_inverse(tolerance);
  }

  return out;
}

//==============================================================================
template <typename Scalar, int Options>
typename SE3<Scalar, Options>::Tangent SE3<Scalar, Options>::ad(
    const SE3<Scalar, Options>::Tangent& V) const
{
  return Ad(*this, V);
}

//==============================================================================
template <typename Scalar, int Options>
typename SE3<Scalar, Options>::Jacobian SE3<Scalar, Options>::ad_matrix() const
{
  Jacobian out;
  const auto rot_mat = m_orientation.quaternion().toRotationMatrix();
  out.template topLeftCorner<3, 3>() = rot_mat;
  out.template topRightCorner<3, 3>().setZero();
  out.template bottomLeftCorner<3, 3>().noalias()
      = skew(m_position.vector()) * rot_mat;
  out.template bottomRightCorner<3, 3>() = rot_mat;
  return out;
}

//==============================================================================
template <typename Scalar, int Options>
SO3<Scalar>& SE3<Scalar, Options>::orientation()
{
  return m_orientation;
}

//==============================================================================
template <typename Scalar, int Options>
const SO3<Scalar>& SE3<Scalar, Options>::orientation() const
{
  return m_orientation;
}

//==============================================================================
template <typename Scalar, int Options>
R3<Scalar>& SE3<Scalar, Options>::position()
{
  return m_position;
}

//==============================================================================
template <typename Scalar, int Options>
const R3<Scalar>& SE3<Scalar, Options>::position() const
{
  return m_position;
}

//==============================================================================
template <typename Scalar, int Options>
typename SE3<Scalar, Options>::Transformation
SE3<Scalar, Options>::transformation() const
{
  Transformation out = Transformation::Identity();
  out.linear() = std::move(rotation());
  out.translation() = std::move(translation());
  return out;
}

//==============================================================================
template <typename Scalar, int Options>
typename SE3<Scalar, Options>::Matrix SE3<Scalar, Options>::matrix() const
{
  return transformation().matrix();
}

//==============================================================================
template <typename Scalar, int Options>
typename SE3<Scalar, Options>::Rotation SE3<Scalar, Options>::rotation() const
{
  return m_orientation.rotation();
}

//==============================================================================
template <typename Scalar, int Options>
typename SE3<Scalar, Options>::Translation& SE3<Scalar, Options>::translation()
{
  return m_position.vector();
}

//==============================================================================
template <typename Scalar, int Options>
const typename SE3<Scalar, Options>::Translation&
SE3<Scalar, Options>::translation() const
{
  return m_position.vector();
}

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
  const auto rotation = orientation.rotation();
  const auto& vector = position.vector();

  TangentData data;
  data.template head<3>().noalias() = rotation * V.vector().template head<3>();
  data.template tail<3>().noalias() = rotation * V.vector().template tail<3>()
                                      + vector.cross(data.template head<3>());

  return SE3<Scalar, Options>::Tangent(std::move(data));
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
  const auto rotation = orientation.rotation();

  TangentData data;
  data.template head<3>().noalias() = rotation * V.vector().template head<3>();
  data.template tail<3>().noalias() = rotation * V.vector().template tail<3>();

  return SE3<Scalar, Options>::Tangent(std::move(data));
}

//==============================================================================
template <typename Scalar, int Options>
SE3Inverse<Scalar, Options>::SE3Inverse(const SE3<Scalar, Options>& original)
  : m_original(original)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
SE3<Scalar, Options> SE3Inverse<Scalar, Options>::operator*(
    const SE3<Scalar, Options>& other) const
{
  const SO3<Scalar, Options> Rt = m_original.orientation().inverse();
  return SE3<Scalar, Options>(
      Rt * other.orientation(),
      Rt * (other.position() - m_original.position()));
}

//==============================================================================
template <typename Scalar, int Options>
SO3<Scalar> SE3Inverse<Scalar, Options>::orientation() const
{
  return m_original.orientation().inverse();
}

//==============================================================================
template <typename Scalar, int Options>
R3<Scalar> SE3Inverse<Scalar, Options>::position() const
{
  return m_original.orientation().inverse() * -m_original.position();
}

//==============================================================================
template <typename Scalar, int Options>
typename SE3Inverse<Scalar, Options>::EvalReturnType
SE3Inverse<Scalar, Options>::eval() const
{
  const SO3<Scalar, Options> Rt = m_original.orientation().inverse();
  return SE3<Scalar, Options>(Rt, -(Rt * m_original.position()));
}

//==============================================================================
template <typename Scalar, int Options>
SE3Algebra<Scalar, Options> SE3Algebra<Scalar, Options>::Zero()
{
  SE3Algebra out;
  out.set_zero();
  return out;
}

//==============================================================================
template <typename Scalar, int Options>
SE3Algebra<Scalar, Options>::SE3Algebra() : m_data(LieAlgebraData::Zero())
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
SE3Algebra<Scalar, Options>::SE3Algebra(const SE3Algebra& other)
  : m_data(other.m_data)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
SE3Algebra<Scalar, Options>::SE3Algebra(SE3Algebra&& other)
  : m_data(std::move(other.m_data))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
SE3Algebra<Scalar, Options>::SE3Algebra(
    const SE3Tangent<Scalar, Options>& tangent)
{
  const auto& vector = tangent.vector();
  m_data.setZero();
  m_data.template topLeftCorner<3, 3>() = skew(vector.template head<3>());
  m_data.template topRightCorner<3, 1>() = vector.template tail<3>();
}

//==============================================================================
template <typename Scalar, int Options>
SE3Algebra<Scalar, Options>::SE3Algebra(SE3Tangent<Scalar, Options>&& tangent)
{
  const auto& vector = tangent.vector();
  m_data.setZero();
  m_data.template topLeftCorner<3, 3>() = skew(vector.template head<3>());
  m_data.template topRightCorner<3, 1>() = vector.template tail<3>();
}

//==============================================================================
template <typename Scalar, int Options>
SE3Algebra<Scalar, Options>::SE3Algebra(const LieAlgebraData& data)
  : m_data(data)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
SE3Algebra<Scalar, Options>::SE3Algebra(LieAlgebraData&& data)
  : m_data(std::move(data))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
template <typename DerivedA, typename DerivedB>
SE3Algebra<Scalar, Options>::SE3Algebra(
    const Eigen::MatrixBase<DerivedA>& angular,
    const Eigen::MatrixBase<DerivedB>& linear)
{
  m_data.template topLeftCorner<3, 3>() = angular;
  m_data.template topRightCorner<3, 1>() = linear;
  m_data.template bottomRows<1>().setZero();
}

//==============================================================================
template <typename Scalar, int Options>
void SE3Algebra<Scalar, Options>::set_zero()
{
  m_data.setZero();
}

//==============================================================================
template <typename Scalar, int Options>
SE3Algebra<Scalar, Options> SE3Algebra<Scalar, Options>::operator/(
    Scalar scalar) const
{
  return SE3Algebra<Scalar, Options>(m_data / scalar);
}

//==============================================================================
template <typename Scalar, int Options>
SE3Tangent<Scalar, Options> SE3Algebra<Scalar, Options>::vee() const
{
  return SE3Tangent<Scalar, Options>(
      Eigen::Matrix<Scalar, 3, 1>(m_data(2, 1), m_data(0, 2), m_data(1, 0)),
      m_data.template topRightCorner<3, 1>());
}

//==============================================================================
template <typename Scalar, int Options>
typename SE3Algebra<Scalar, Options>::LieAlgebraData&
SE3Algebra<Scalar, Options>::mutable_matrix()
{
  return m_data;
}

//==============================================================================
template <typename Scalar, int Options>
const typename SE3Algebra<Scalar, Options>::LieAlgebraData&
SE3Algebra<Scalar, Options>::matrix() const
{
  return m_data;
}

//==============================================================================
template <typename Derived>
template <typename OtherDerived>
Derived SE3TangentBase<Derived>::ad(
    const SE3TangentBase<OtherDerived>& other) const
{
  return derived().ad(other);
}

//==============================================================================
template <typename Derived>
const typename SE3TangentBase<Derived>::TangentData&
SE3TangentBase<Derived>::vector() const
{
  return derived().vector();
}

//==============================================================================
template <typename Derived>
typename SE3TangentBase<Derived>::TangentData& SE3TangentBase<Derived>::vector()
{
  return derived().vector();
}

//==============================================================================
template <typename Scalar, int Options>
SE3Tangent<Scalar, Options>::SE3Tangent() : m_data(TangentData::Zero())
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
SE3Tangent<Scalar, Options>::SE3Tangent(const SE3Tangent& other)
  : m_data(other.m_data)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
SE3Tangent<Scalar, Options>::SE3Tangent(SE3Tangent&& other)
  : m_data(std::move(other.m_data))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
template <typename Derived>
SE3Tangent<Scalar, Options>::SE3Tangent(
    const Eigen::MatrixBase<Derived>& coeffs)
  : m_data(coeffs)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
template <typename Derived>
SE3Tangent<Scalar, Options>::SE3Tangent(Eigen::MatrixBase<Derived>&& coeffs)
  : m_data(std::move(coeffs))
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
template <typename DerivedA, typename DerivedB>
SE3Tangent<Scalar, Options>::SE3Tangent(
    const Eigen::MatrixBase<DerivedA>& angular,
    const Eigen::MatrixBase<DerivedB>& linear)
{
  m_data.template head<3>() = angular;
  m_data.template tail<3>() = linear;
}

//==============================================================================
template <typename Scalar, int Options>
SE3Tangent<Scalar, Options>& SE3Tangent<Scalar, Options>::operator=(
    const SE3Tangent<Scalar, Options>& other)
{
  m_data = other.m_data;
  return *this;
}

//==============================================================================
template <typename Scalar, int Options>
SE3Tangent<Scalar, Options>& SE3Tangent<Scalar, Options>::operator=(
    SE3Tangent<Scalar, Options>&& other)
{
  m_data = std::move(other.m_data);
  return *this;
}

//==============================================================================
template <typename Scalar, int Options>
Scalar SE3Tangent<Scalar, Options>::operator[](int index) const
{
  return m_data[index];
}

//==============================================================================
template <typename Scalar, int Options>
Scalar& SE3Tangent<Scalar, Options>::operator[](int index)
{
  return m_data[index];
}

//==============================================================================
template <typename Scalar, int Options>
SE3Tangent<Scalar, Options> SE3Tangent<Scalar, Options>::operator-() const
{
  return SE3Tangent<Scalar, Options>(-m_data);
}

//==============================================================================
template <typename Scalar, int Options>
SE3Tangent<Scalar, Options> SE3Tangent<Scalar, Options>::operator+(
    const SE3Tangent& other) const
{
  return SE3Tangent(m_data + other.m_data);
}

//==============================================================================
template <typename Scalar, int Options>
SE3Tangent<Scalar, Options> SE3Tangent<Scalar, Options>::operator-(
    const SE3Tangent& other) const
{
  return SE3Tangent(m_data - other.m_data);
}

//==============================================================================
template <typename Scalar, int Options>
Scalar SE3Tangent<Scalar, Options>::operator*(
    const SE3Cotangent<Scalar, Options>& wrench) const
{
  return m_data.dot(wrench.m_data);
}

//==============================================================================
template <typename Scalar, int Options>
void SE3Tangent<Scalar, Options>::set_zero()
{
  m_data.setZero();
}

//==============================================================================
template <typename Scalar, int Options>
void SE3Tangent<Scalar, Options>::set_random()
{
  const SE3<Scalar, Options> tf = SE3<Scalar, Options>::Random();
  *this = tf.log();
}

//==============================================================================
template <typename Scalar, int Options>
SE3Algebra<Scalar, Options> SE3Tangent<Scalar, Options>::hat() const
{
  return SE3Algebra<Scalar, Options>(*this);
}

//==============================================================================
template <typename Scalar, int Options>
template <typename Derived>
void SE3Tangent<Scalar, Options>::vee(const math::MatrixBase<Derived>& mat)
{
  if constexpr (Derived::RowsAtCompileTime == Eigen::Dynamic) {
    DART_ASSERT(mat.rows() == 3);
  } else {
    static_assert(Derived::RowsAtCompileTime == 3, "Invalid size of rows.");
  }

  if constexpr (Derived::ColsAtCompileTime == Eigen::Dynamic) {
    DART_ASSERT(mat.cols() == 3);
  } else {
    static_assert(Derived::ColsAtCompileTime == 3, "Invalid size of cols.");
  }

  m_data.template head<3>() << mat(2, 1), mat(0, 2), mat(1, 0);
  m_data.template tail<3>();
  DART_NOT_IMPLEMENTED;
}

//==============================================================================
template <typename Scalar, int Options>
SE3<Scalar, Options> SE3Tangent<Scalar, Options>::exp(
    Jacobian* jacobian, Scalar tolerance) const
{
  // TODO(JS): Make this a map
  const auto so3_tangent = SO3Tangent<Scalar>(m_data.template head<3>());
  const auto so3_exp = so3_tangent.exp(nullptr, tolerance);
  const auto so3_left_jacobian = so3_tangent.left_jacobian(tolerance);

  const SE3<Scalar, Options> out(
      so3_exp, so3_left_jacobian * m_data.template tail<3>());

  if (jacobian) {
    (*jacobian) = right_jacobian();
  }

  return out;
}

//==============================================================================
template <typename Scalar, int Options>
template <typename Derived>
SE3Tangent<Scalar, Options> SE3Tangent<Scalar, Options>::ad(
    const SE3TangentBase<Derived>& other) const
{
  //--------------------------------------------------------------------------
  // ad(s1, s2) = | [w1]    0 | | w2 |
  //              | [v1] [w1] | | v2 |
  //
  //            = |          [w1]w2 |
  //              | [v1]w2 + [w1]v2 |
  //--------------------------------------------------------------------------

  TangentData data;
  const TangentData& vec1 = m_data;
  const TangentData& vec2 = other.vector();
  data.template head<3>()
      = vec1.template head<3>().cross(vec2.template head<3>());
  data.template tail<3>()
      = vec1.template head<3>().cross(vec2.template tail<3>())
        + vec1.template tail<3>().cross(vec2.template head<3>());
  return SE3Tangent(std::move(data));
}

//==============================================================================
template <typename Scalar, int Options>
typename SE3Tangent<Scalar, Options>::Matrix
SE3Tangent<Scalar, Options>::ad_matrix() const
{
  // TODO(JS): Create matrix type instead of using Jacobian

  Matrix out;

  out.template topLeftCorner<3>() = skew(m_data.template head<3>());
  out.template topRightCorner<3>().setZero();
  out.template topLeftCorner<3>() = skew(m_data.template tail<3>());
  out.template topLeftCorner<3>() = out.template topLeftCorner<3>();

  return out;
}

//==============================================================================
template <typename Derived, typename Scalar>
Eigen::Matrix<Scalar, 3, 3> compute_q(
    const Eigen::MatrixBase<Derived>& data, Scalar tolerance)
{
  const auto& a = data.template head<3>().eval();
  const auto& b = data.template tail<3>().eval();

  const auto A = skew(a);
  const auto B = skew(b);

  const Scalar t = a.norm();

  Eigen::Matrix<Scalar, 3, 3> out;

  if (t < tolerance) {
    out.noalias() = 0.5 * B;
  } else {
    const Scalar t2 = t * t;
    const Scalar t3 = t2 * t;
    const Scalar t4 = t3 * t;
    const Scalar t5 = t4 * t;
    const auto AB = (A * B).eval();
    const auto BA = (B * A).eval();
    const auto ABA = (AB * A).eval();
    const auto AAB = (A * AB).eval();
    const auto BAA = (BA * A).eval();
    const auto ABAA = (ABA * A).eval();
    const auto AABA = (A * ABA).eval();
    const Scalar st = std::sin(t);
    const Scalar ct = std::cos(t);
    // clang-format off
    out.noalias() =
      0.5 * B
      + ((t - st) / t3) * (AB + BA  + ABA)
      - ((1 - 0.5 * t2 - ct) / t4) * (AAB + BAA - 3 * ABA)
      - 0.5*((1 - 0.5*t2 - ct)/t4 - 3*(t - st - t3 / 6) / t5) * (ABAA + AABA);
    // clang-format on
  }

  return out;
}

//==============================================================================
template <typename Scalar, int Options>
typename SE3Tangent<Scalar, Options>::Jacobian
SE3Tangent<Scalar, Options>::left_jacobian(Scalar tolerance) const
{
  const auto& a = m_data.template head<3>();

  Jacobian jac;
  jac.template topLeftCorner<3, 3>()
      = SO3Tangent<Scalar>(a).left_jacobian(tolerance);
  jac.template topRightCorner<3, 3>().setZero();
  jac.template bottomLeftCorner<3, 3>() = compute_q(m_data, tolerance);
  jac.template bottomRightCorner<3, 3>() = jac.template topLeftCorner<3, 3>();
  return jac;
}

//==============================================================================
template <typename Scalar, int Options>
typename SE3Tangent<Scalar, Options>::Jacobian
SE3Tangent<Scalar, Options>::space_jacobian(Scalar tolerance) const
{
  return left_jacobian(tolerance);
}

//==============================================================================
template <typename Scalar, int Options>
typename SE3Tangent<Scalar, Options>::Jacobian
SE3Tangent<Scalar, Options>::right_jacobian(Scalar tolerance) const
{
  return SE3Tangent<Scalar, Options>(-m_data).left_jacobian(tolerance);
}

//==============================================================================
template <typename Scalar, int Options>
typename SE3Tangent<Scalar, Options>::Jacobian
SE3Tangent<Scalar, Options>::body_jacobian(Scalar tolerance) const
{
  return right_jacobian(tolerance);
}

//==============================================================================
template <typename Scalar, int Options>
typename SE3Tangent<Scalar, Options>::Jacobian
SE3Tangent<Scalar, Options>::left_jacobian_inverse(Scalar tolerance) const
{
  const auto& a = m_data.template head<3>();
  const Eigen::Matrix<Scalar, 3, 3> jac_inv
      = SO3Tangent<Scalar>(a).left_jacobian_inverse(tolerance);
  const auto& Q = compute_q(m_data, tolerance);

  Jacobian jac;
  jac.template topLeftCorner<3, 3>() = jac_inv;
  jac.template topRightCorner<3, 3>().setZero();
  jac.template bottomLeftCorner<3, 3>().noalias() = -jac_inv * Q * jac_inv;
  jac.template bottomRightCorner<3, 3>() = jac_inv;
  return jac;
}

//==============================================================================
template <typename Scalar, int Options>
typename SE3Tangent<Scalar, Options>::Jacobian
SE3Tangent<Scalar, Options>::right_jacobian_inverse(Scalar tolerance) const
{
  return SE3Tangent<Scalar, Options>(-m_data).left_jacobian_inverse(tolerance);
}

//==============================================================================
template <typename Scalar, int Options>
SO3Tangent<Scalar, Options> SE3Tangent<Scalar, Options>::angular() const
{
  return SO3Tangent<Scalar, Options>(m_data.template head<3>());
}

//==============================================================================
template <typename Scalar, int Options>
Eigen::Map<SO3Tangent<Scalar, Options>> SE3Tangent<Scalar, Options>::angular()
{
  return Eigen::Map<SO3Tangent<Scalar, Options>>(m_data.data());
}

//==============================================================================
template <typename Scalar, int Options>
RTangent<Scalar, 3, Options> SE3Tangent<Scalar, Options>::linear() const
{
  return RTangent<Scalar, 3, Options>(m_data.template tail<3>());
}

//==============================================================================
template <typename Scalar, int Options>
Eigen::Map<R3Tangent<Scalar, Options>> SE3Tangent<Scalar, Options>::linear()
{
  return Eigen::Map<R3Tangent<Scalar, Options>>(m_data.data() + 3);
}

//==============================================================================
template <typename Scalar, int Options>
typename SE3Tangent<Scalar, Options>::TangentData&
SE3Tangent<Scalar, Options>::vector()
{
  return m_data;
}

//==============================================================================
template <typename Scalar, int Options>
const typename SE3Tangent<Scalar, Options>::TangentData&
SE3Tangent<Scalar, Options>::vector() const
{
  return m_data;
}

//==============================================================================
template <typename Scalar, int Options>
SE3Tangent<Scalar, Options> SE3Tangent<Scalar, Options>::Random()
{
  SE3Tangent<Scalar, Options> out;
  out.set_random();
  return out;
}

//==============================================================================
template <typename Scalar, int Options>
SE3Cotangent<Scalar, Options>::SE3Cotangent() : m_data(CotangentData::Zero())
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
auto SE3Cotangent<Scalar, Options>::torque() const
{
  return m_data.template head<3>();
}

//==============================================================================
template <typename Scalar, int Options>
auto SE3Cotangent<Scalar, Options>::mutable_torque()
{
  return m_data.template head<3>();
}

//==============================================================================
template <typename Scalar, int Options>
auto SE3Cotangent<Scalar, Options>::force() const
{
  return m_data.template tail<3>();
}

//==============================================================================
template <typename Scalar, int Options>
auto SE3Cotangent<Scalar, Options>::mutable_force()
{
  return m_data.template tail<3>();
}

//==============================================================================
template <typename Scalar, int Options>
auto SE3Cotangent<Scalar, Options>::angular() const
{
  return m_data.template head<3>();
}

//==============================================================================
template <typename Scalar, int Options>
auto SE3Cotangent<Scalar, Options>::mutable_angular()
{
  return m_data.template head<3>();
}

//==============================================================================
template <typename Scalar, int Options>
auto SE3Cotangent<Scalar, Options>::linear() const
{
  return m_data.template tail<3>();
}

//==============================================================================
template <typename Scalar, int Options>
auto SE3Cotangent<Scalar, Options>::mutable_linear()
{
  return m_data.template tail<3>();
}

//==============================================================================
template <typename Scalar, int Options>
typename SE3Cotangent<Scalar, Options>::CotangentData&
SE3Cotangent<Scalar, Options>::vector()
{
  return m_data;
}

//==============================================================================
template <typename Scalar, int Options>
const typename SE3Cotangent<Scalar, Options>::CotangentData&
SE3Cotangent<Scalar, Options>::vector() const
{
  return m_data;
}

} // namespace dart::math

namespace Eigen {

//==============================================================================
template <typename Scalar, int Options>
Map<dart::math::SE3<Scalar, Options>, Options>::Map(Scalar* data)
  : m_orientation(data), m_position(data + dart::math::SO3<Scalar>::RepDim)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar, int Options>
Map<const dart::math::SE3<Scalar, Options>, Options>::Map(const Scalar* data)
  : m_orientation(data), m_position(data + dart::math::SO3<Scalar>::RepDim)
{
  // Do nothing
}

} // namespace Eigen
