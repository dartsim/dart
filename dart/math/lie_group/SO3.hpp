/*
 * Copyright (c) 2011-2023, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <dart/math/lie_group/SO3Base.hpp>

namespace Eigen::internal {

// TODO(JS): Move to a dedicated header file
/// @brief Specialization of Eigen::internal::traits for SO3
template <typename S, int Options_>
struct traits<::dart::math::SO3<S, Options_>>
{
  static constexpr int Options = Options_;
  static constexpr int CoeffsDim = 4;

  using Scalar = S;
  using Coeffs = ::Eigen::Matrix<S, CoeffsDim, 1, Options>;
  using PlainObject = ::dart::math::SO3<S, Options_>;
  using MatrixType = ::Eigen::Matrix<S, 3, 3>;
};

} // namespace Eigen::internal

namespace dart::math {

/// @brief SO3 is a specialization of LieGroupBase for SO3
/// @tparam S The scalar type
/// @tparam Options_ The options for the underlying Eigen::Matrix
template <typename S, int Options_>
class SO3 : public SO3Base<SO3<S, Options_>>
{
public:
  using Base = SO3Base<SO3<S, Options_>>;

  // LieGroupBase types
  using Scalar = typename Base::Scalar;
  using Coeffs = typename Base::Coeffs;
  using PlainObject = typename Base::PlainObject;
  using MatrixType = typename Base::MatrixType;

  // SO3Base specific types
  using QuaternionType = typename Base::QuaternionType;
  using ConstQuaternionType = typename Base::ConstQuaternionType;

  /// @brief Tag for the constructor that does not normalize the quaternion
  enum NoNormalizeTag
  {
    NoNormalize = 0,
  };

  using Base::Tolerance;

  /// Returns the identity SO3
  [[nodiscard]] static PlainObject Identity();

  /// Returns a random SO3
  [[nodiscard]] static PlainObject Random();

  /// Returns the exponential map of the given vector
  ///
  /// The exponential map of a vector @f$ \xi @f$ is an SO3 @f$ x @f$ such
  /// that @f$ \log(x) = \xi @f$.
  ///
  /// @param[in] dx The vector to be converted to an SO3.
  /// @param[in] tol The tolerance for the norm of the vector.
  /// @return The SO3.
  /// @tparam MatrixDrived The type of the vector
  /// @see Log()
  template <typename MatrixDrived>
  [[nodiscard]] static SO3 Exp(
      const Eigen::MatrixBase<MatrixDrived>& dx, S tol = Tolerance());

  /// Returns the exponential map of the given vector
  ///
  /// The exponential map of a vector @f$ \xi @f$ is an SO3 @f$ x @f$ such
  /// that @f$ \log(x) = \xi @f$.
  ///
  /// This function also returns the Jacobian of the exponential map.
  ///
  /// @param[in] dx The vector to be converted to an SO3.
  /// @param[out] jacobian The Jacobian of the exponential map.
  /// @param[in] tol The tolerance for the norm of the vector.
  /// @return The SO3.
  /// @tparam MatrixDrivedA The type of the vector
  /// @tparam MatrixDerivedB The type of the Jacobian
  /// @see Log()
  template <typename MatrixDrivedA, typename MatrixDerivedB>
  [[nodiscard]] static SO3 Exp(
      const Eigen::MatrixBase<MatrixDrivedA>& dx,
      Eigen::MatrixBase<MatrixDerivedB>* jacobian,
      S tol = Tolerance());

  /// Returns the logarithm map of the given SO3
  ///
  /// The logarithm map of an SO3 @f$ x @f$ is a vector @f$ \xi @f$ such
  /// that @f$ \log(x) = \xi @f$.
  ///
  /// @param[in] x The SO3 to be converted to a vector.
  /// @param[in] tol The tolerance for the norm of the vector.
  /// @return The vector.
  /// @tparam MatrixDrived The type of the SO3
  /// @see Exp()
  template <typename OtherDrived>
  [[nodiscard]] static Vector3<S> Log(
      const SO3Base<OtherDrived>& x, S tol = Tolerance());

  /// Returns the logarithm map of the given SO3
  ///
  /// The logarithm map of an SO3 @f$ x @f$ is a vector @f$ \xi @f$ such
  /// that @f$ \log(x) = \xi @f$.
  ///
  /// This function also returns the Jacobian of the logarithm map.
  ///
  /// @param[in] x The SO3 to be converted to a vector.
  /// @param[out] jacobian The Jacobian of the logarithm map.
  /// @param[in] tol The tolerance for the norm of the vector.
  /// @return The vector.
  /// @tparam MatrixDrivedA The type of the SO3
  /// @tparam MatrixDerivedB The type of the Jacobian
  /// @see Exp()
  template <typename OtherDrived, typename MatrixDerived>
  [[nodiscard]] static Vector3<S> Log(
      const SO3Base<OtherDrived>& x,
      Eigen::MatrixBase<MatrixDerived>* jacobian,
      S tol = Tolerance());

  /// Returns the hat operator of the given vector
  ///
  /// The hat operator of a vector is a skew-symmetric matrix @f$ \hat{v} @f$
  /// such that @f$ \hat{v} w = v \times w @f$ for any vector
  /// @f$ w @f$.
  ///
  /// @param[in] xi The vector to be converted to a skew-symmetric matrix.
  /// @return The skew-symmetric matrix.
  /// @tparam MatrixDrived The type of the vector
  /// @see Vee()
  template <typename MatrixDrived>
  [[nodiscard]] static Matrix3<S> Hat(
      const Eigen::MatrixBase<MatrixDrived>& xi);

  /// Returns the vee operator of the given skew-symmetric matrix
  ///
  /// The vee operator of a skew-symmetric matrix @f$ \hat{v} @f$ is a vector
  /// @f$ v @f$ such that @f$ \hat{v} w = v \times w @f$ for any vector
  /// @f$ w @f$.
  ///
  /// @param[in] matrix The skew-symmetric matrix to be converted to a vector.
  /// @return The vector.
  /// @tparam MatrixDrived The type of the skew-symmetric matrix
  /// @see Hat()
  template <typename MatrixDerived>
  [[nodiscard]] static Vector3<S> Vee(
      const Eigen::MatrixBase<MatrixDerived>& matrix);

  /// Returns the adjoint transformation of the given SO3
  ///
  /// The adjoint transformation of SO3 @f$ x @f$ is a matrix @f$ A @f$ such
  /// that @f$ A v = x v x^{-1} @f$ for any vector @f$ v @f$.
  ///
  /// @param[in] x The SO3 to be converted to a matrix.
  template <typename OtherDerived>
  [[nodiscard]] static Matrix3<S> Ad(const SO3Base<OtherDerived>& x);

  /// Returns the left Jacobian of the exponential map
  ///
  /// The left Jacobian of the exponential map is a matrix @f$ L @f$ such that
  /// @f$ Ad_{exp(L \xi)} = L Ad_{exp(\xi)} @f$ for any vector @f$ \xi @f$ and
  /// SO3. It expresses the change of the exponential map at the identity
  /// element to the exponential map at the group element.
  ///
  /// @param[in] xi The vector to be converted to a matrix.
  /// @param[in] tol The tolerance for the norm of the vector.
  /// @return The matrix.
  /// @tparam MatrixDrived The type of the vector
  /// @see RightJacobian()
  template <typename MatrixDrived>
  [[nodiscard]] static Matrix3<S> LeftJacobian(
      const Eigen::MatrixBase<MatrixDrived>& xi, S tol = Tolerance());

  /// Returns the right Jacobian of the exponential map
  ///
  /// The right Jacobian of the exponential map is a matrix @f$ R @f$ such that
  /// @f$ Ad_{exp(R \xi)} = Ad_{exp(\xi)} R @f$ for any vector @f$ \xi @f$ and
  /// SO3. It expresses the change of the exponential map at the group element
  /// to the exponential map at the identity element.
  ///
  /// @param[in] xi The vector to be converted to a matrix.
  /// @param[in] tol The tolerance for the norm of the vector.
  /// @return The matrix.
  /// @tparam MatrixDrived The type of the vector
  /// @see LeftJacobian()
  template <typename MatrixDrived>
  [[nodiscard]] static Matrix3<S> RightJacobian(
      const Eigen::MatrixBase<MatrixDrived>& xi, S tol = Tolerance());

  /// Returns the left Jacobian inverse of the exponential map
  ///
  /// @param[in] dx The vector to be converted to a matrix.
  /// @param[in] tol The tolerance for the norm of the vector.
  /// @return The matrix.
  /// @tparam MatrixDrived The type of the vector
  /// @see RightJacobianInverse()
  template <typename MatrixDrived>
  [[nodiscard]] static Matrix3<S> LeftJacobianInverse(
      const Eigen::MatrixBase<MatrixDrived>& dx, S tol = Tolerance());

  /// Returns the right Jacobian inverse of the exponential map
  ///
  /// @param[in] dx The vector to be converted to a matrix.
  /// @param[in] tol The tolerance for the norm of the vector.
  /// @return The matrix.
  /// @tparam MatrixDrived The type of the vector
  /// @see LeftJacobianInverse()
  template <typename MatrixDerived>
  [[nodiscard]] static Matrix3<S> RightJacobianInverse(
      const Eigen::MatrixBase<MatrixDerived>& dx, S tol = Tolerance());

  /// Default constructor that initializes the quaternion to identity
  SO3();

  /// Copy constructor
  /// @param[in] other The other SO3 to be copied
  SO3(const SO3& other);

  /// Move constructor
  /// @param[in] other The other SO3 to be moved
  SO3(SO3&& other);

  /// Constructs an SO3 from a quaternion
  template <typename QuaternionDrived>
  SO3(const ::Eigen::QuaternionBase<QuaternionDrived>& quat);

  /// Constructs an SO3 from a quaternion
  ///
  /// This constructor does not normalize the quaternion. It is useful when
  /// constructing SO3 from a quaternion that is already normalized.
  template <typename QuaternionDrived>
  SO3(const ::Eigen::QuaternionBase<QuaternionDrived>& quat, NoNormalizeTag);

  /// Constructs an SO3 from a quaternion
  template <typename QuaternionDrived>
  SO3(::Eigen::QuaternionBase<QuaternionDrived>&& quat);

  /// Constructs an SO3 from a quaternion
  ///
  /// This constructor does not normalize the quaternion. It is useful when
  /// constructing SO3 from a quaternion that is already normalized.
  template <typename QuaternionDrived>
  SO3(::Eigen::QuaternionBase<QuaternionDrived>&& quat, NoNormalizeTag);

  /// Copy assignment operator
  /// @param[in] other The other SO3 to be copied
  /// @return Reference to this SO3
  SO3& operator=(const SO3& other);

  /// Move assignment operator
  /// @param[in] other The other SO3 to be moved
  /// @return Reference to this SO3
  SO3& operator=(SO3&& other);

  using Base::normalize;

  /// Returns the matrix representation of this SO3
  ///
  /// The matrix representation of SO3 is a 3x3 orthogonal matrix
  [[nodiscard]] MatrixType toMatrix() const;

  /// Returns the quaternion representation of this SO3
  [[nodiscard]] const ConstQuaternionType quaternion() const;

  /// Returns the quaternion representation of this SO3
  [[nodiscard]] QuaternionType quaternion();

  /// Returns the coefficients of the underlying quaternion
  [[nodiscard]] const Coeffs& coeffs() const;

  /// Returns the coefficients of the underlying quaternion
  [[nodiscard]] Coeffs& coeffs();

private:
  /// The underlying quaternion coefficients
  Coeffs m_coeffs;
};

DART_TEMPLATE_CLASS_HEADER(MATH, SO3);

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

namespace dart::math {

//==============================================================================
template <typename S, int Options>
typename SO3<S, Options>::PlainObject SO3<S, Options>::Identity()
{
  return SO3(::Eigen::Quaternion<S>::Identity(), NoNormalize);
}

//==============================================================================
template <typename S, int Options>
typename SO3<S, Options>::PlainObject SO3<S, Options>::Random()
{
  return SO3(::Eigen::Quaternion<S>::UnitRandom());
}

//==============================================================================
template <typename S, int Options>
template <typename MatrixDerived>
SO3<S, Options> SO3<S, Options>::Exp(
    const Eigen::MatrixBase<MatrixDerived>& dx, S tol)
{
  const S theta = dx.norm();
  if (theta < tol) {
    const Vector3<S> vec = 0.5 * dx;
    return SO3<S, Options>{Eigen::Quaternion<S>(1.0, vec[0], vec[1], vec[2])};
  }

  const S half_theta = 0.5 * theta;
  const S sin_half_theta = std::sin(half_theta);
  const S cos_half_theta = std::cos(half_theta);
  const Vector3<S> vec = (sin_half_theta / theta) * dx;
  return SO3<S, Options>{
      Eigen::Quaternion<S>(cos_half_theta, vec[0], vec[1], vec[2])};
}

//==============================================================================
template <typename S, int Options>
template <typename MatrixDrivedA, typename MatrixDerivedB>
SO3<S, Options> SO3<S, Options>::Exp(
    const Eigen::MatrixBase<MatrixDrivedA>& dx,
    Eigen::MatrixBase<MatrixDerivedB>* jacobian,
    S tol)
{
  if (jacobian) {
    *jacobian = RightJacobian(dx, tol);
  }
  return Exp(dx, tol);
}

//==============================================================================
template <typename S, int Options>
template <typename OtherDrived>
Vector3<S> SO3<S, Options>::Log(const SO3Base<OtherDrived>& x, S tol)
{
  const S cos_theta = x.quaternion().w();
  const S theta = 2 * std::acos(cos_theta);
  DART_ASSERT(!std::isnan(theta));

  if (theta < tol) {
    return 2 * x.quaternion().vec();
  }

  const S theta_over_sin_half_theta = theta / std::sin(0.5 * theta);
  return theta_over_sin_half_theta * x.quaternion().vec();
}

//==============================================================================
template <typename S, int Options>
template <typename OtherDrived, typename MatrixDerived>
Vector3<S> SO3<S, Options>::Log(
    const SO3Base<OtherDrived>& x,
    Eigen::MatrixBase<MatrixDerived>* jacobian,
    S tol)
{
  const Vector3<S> xi = Log(x, tol);
  if (jacobian) {
    (*jacobian) = RightJacobianInverse(xi);
  }
  return xi;
}

//==============================================================================
template <typename S, int Options>
template <typename MatrixDrived>
Matrix3<S> SO3<S, Options>::Hat(const Eigen::MatrixBase<MatrixDrived>& xi)
{
  // clang-format off
  return Matrix3<S>{
    {      0, -xi[2], +xi[1]},
    { +xi[2],      0, -xi[0]},
    { -xi[1], +xi[0],      0}
  };
  // clang-format on
}

//==============================================================================
template <typename S, int Options>
template <typename MatrixDerived>
Vector3<S> SO3<S, Options>::Vee(const Eigen::MatrixBase<MatrixDerived>& matrix)
{
  return Vector3<S>{matrix(2, 1), matrix(0, 2), matrix(1, 0)};
}

//==============================================================================
template <typename S, int Options>
template <typename OtherDerived>
Matrix3<S> SO3<S, Options>::Ad(const SO3Base<OtherDerived>& x)
{
  return x.quaternion().toRotationMatrix();
}

//==============================================================================
template <typename S, int Options>
template <typename OtherDerived>
Matrix3<S> SO3<S, Options>::LeftJacobian(
    const Eigen::MatrixBase<OtherDerived>& xi, S tol)
{
  Matrix3<S> J = Matrix3<S>::Identity();

  const S t = xi.norm();

  if (t < tol) {
    J.noalias() += Hat(0.5 * xi);
    return J;
  }

  const S t2 = t * t;
  const S t3 = t2 * t;
  const S st = std::sin(t);
  const S ct = std::cos(t);
  const Matrix3<S> A = Hat(xi);
  J.noalias() += ((1 - ct) / t2) * A;
  J.noalias() += ((t - st) / t3) * A * A;

  return J;
}

//==============================================================================
template <typename S, int Options>
template <typename OtherDerived>
Matrix3<S> SO3<S, Options>::RightJacobian(
    const Eigen::MatrixBase<OtherDerived>& xi, S tol)
{
  return LeftJacobian(-xi, tol);
}

//==============================================================================
template <typename S, int Options>
template <typename OtherDerived>
Matrix3<S> SO3<S, Options>::LeftJacobianInverse(
    const Eigen::MatrixBase<OtherDerived>& dx, S tol)
{
  Matrix3<S> J = Matrix3<S>::Identity();

  const S theta = dx.norm();
  if (theta < tol) {
    J.noalias() += Hat(0.5 * dx);
    return J;
  }

  const S t2 = theta * theta;
  const S st = std::sin(theta);
  const S ct = std::cos(theta);
  const Matrix3<S> A = Hat(dx);
  J.noalias() -= 0.5 * A;
  J.noalias() += (1 / t2 - (1 + ct) / (2 * theta * st)) * A * A;

  return J;
}

//==============================================================================
template <typename S, int Options>
template <typename OtherDerived>
Matrix3<S> SO3<S, Options>::RightJacobianInverse(
    const Eigen::MatrixBase<OtherDerived>& dx, S tol)
{
  return LeftJacobianInverse(-dx, tol);
}

//==============================================================================
template <typename S, int Options>
SO3<S, Options>::SO3() : m_coeffs(::Eigen::Quaternion<S>::Identity().coeffs())
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SO3<S, Options>::SO3(const SO3& other) : m_coeffs(other.m_coeffs)
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SO3<S, Options>::SO3(SO3&& other) : m_coeffs(std::move(other.m_coeffs))
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
template <typename QuaternionDrived>
SO3<S, Options>::SO3(const ::Eigen::QuaternionBase<QuaternionDrived>& quat)
  : m_coeffs(quat.coeffs())
{
  normalize();
}

//==============================================================================
template <typename S, int Options>
template <typename QuaternionDrived>
SO3<S, Options>::SO3(
    const ::Eigen::QuaternionBase<QuaternionDrived>& quat, NoNormalizeTag)
  : m_coeffs(quat.coeffs())
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
template <typename QuaternionDrived>
SO3<S, Options>::SO3(::Eigen::QuaternionBase<QuaternionDrived>&& quat)
  : m_coeffs(std::move(quat.coeffs()))
{
  normalize();
}

//==============================================================================
template <typename S, int Options>
template <typename QuaternionDrived>
SO3<S, Options>::SO3(
    ::Eigen::QuaternionBase<QuaternionDrived>&& quat, NoNormalizeTag)
  : m_coeffs(std::move(quat.coeffs()))
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SO3<S, Options>& SO3<S, Options>::operator=(const SO3& other)
{
  m_coeffs = other.m_coeffs;
  return *this;
}

//==============================================================================
template <typename S, int Options>
SO3<S, Options>& SO3<S, Options>::operator=(SO3&& other)
{
  m_coeffs = std::move(other.m_coeffs);
  return *this;
}

//==============================================================================
template <typename S, int Options>
typename SO3<S, Options>::MatrixType SO3<S, Options>::toMatrix() const
{
  return ::Eigen::Quaternion<S>(m_coeffs).toRotationMatrix();
}

//==============================================================================
template <typename S, int Options>
const typename SO3<S, Options>::ConstQuaternionType
SO3<S, Options>::quaternion() const
{
  return ConstQuaternionType(m_coeffs.data());
}

//==============================================================================
template <typename S, int Options>
typename SO3<S, Options>::QuaternionType SO3<S, Options>::quaternion()
{
  return QuaternionType(m_coeffs.data());
}

//==============================================================================
template <typename S, int Options>
const typename SO3<S, Options>::Coeffs& SO3<S, Options>::coeffs() const
{
  return m_coeffs;
}

//==============================================================================
template <typename S, int Options>
typename SO3<S, Options>::Coeffs& SO3<S, Options>::coeffs()
{
  return m_coeffs;
}

} // namespace dart::math
