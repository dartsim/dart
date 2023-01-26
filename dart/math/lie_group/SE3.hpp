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

#include <dart/math/lie_group/SE3Base.hpp>
#include <dart/math/lie_group/SE3Tangent.hpp>
#include <dart/math/lie_group/SO3.hpp>
#include <dart/math/lie_group/SO3Map.hpp>

namespace Eigen::internal {

// TODO(JS): Move to a dedicated header file
/// @brief Specialization of Eigen::internal::traits for SE3
template <typename S>
struct traits<::dart::math::SE3<S>>
{
  using Scalar = S;

  // LieGroup common
  static constexpr int Dim = 3;
  static constexpr int DoF = 6;
  static constexpr int MatrixRepDim = 4;
  static constexpr int ParamSize = 7;
  using PlainObject = ::dart::math::SE3<S>;
  using MatrixType = ::Eigen::Matrix<S, MatrixRepDim, MatrixRepDim>;
  using Params = ::Eigen::Matrix<S, ParamSize, 1>;
  using Tangent = ::dart::math::SE3Tangent<S>;
};

} // namespace Eigen::internal

namespace dart::math {

/// @brief SE3 is a specialization of LieGroupBase for SE3
/// @tparam S The scalar type
template <typename S>
class SE3 : public SE3Base<SE3<S>>
{
public:
  using Base = SE3Base<SE3<S>>;
  using Scalar = typename Base::Scalar;

  // LieGroup common
  using Params = typename Base::Params;
  using PlainObject = typename Base::PlainObject;
  using MatrixType = typename Base::MatrixType;
  using Tangent = typename Base::Tangent;

  using Base::Tolerance;

  /// Returns the exponential map of the given vector
  ///
  /// @param dx The vector to be converted to SE3
  /// @param tol The tolerance for the norm of the vector
  template <typename MatrixDerived>
  [[nodiscard]] static SE3 Exp(
      const Eigen::MatrixBase<MatrixDerived>& dx, const S tol = Tolerance());
  // TODO(JS): Update this to take Tangent

  /// Returns the exponential map of the given vector and the Jacobian of the
  /// exponential map
  ///
  /// @param dx The vector to be converted to SE3
  /// @param jacobian The Jacobian of the exponential map
  /// @param tol The tolerance for the norm of the vector
  template <typename MatrixDerivedA, typename MatrixDerivedB>
  [[nodiscard]] static SE3 Exp(
      const Eigen::MatrixBase<MatrixDerivedA>& dx,
      Eigen::MatrixBase<MatrixDerivedB>* jacobian,
      S tol = Tolerance());

  /// Returns the logarithm map of the given SE3
  ///
  /// @param x The SE3 to be converted to vector
  /// @param tol The tolerance for the norm of the vector
  template <typename OtherDerived>
  [[nodiscard]] static Tangent Log(
      const SE3Base<OtherDerived>& x, S tol = Tolerance());

  /// Returns the logarithm map of the given SE3 and the Jacobian of the
  /// logarithm map
  ///
  /// @param x The SE3 to be converted to vector
  /// @param jacobian The Jacobian of the logarithm map
  /// @param tol The tolerance for the norm of the vector
  template <typename OtherDerived, typename MatrixDerived>
  [[nodiscard]] static Tangent Log(
      const SE3Base<OtherDerived>& x,
      Eigen::MatrixBase<MatrixDerived>* jacobian,
      S tol = Tolerance());

  /**
   * Returns the hat operator of the given vector
   *
   * The hat operator is defined as follows:
   * @f[
   *   \hat{\xi} = \begin{bmatrix}
   *     \hat{w} & 0 \\
   *     \hat{v} & \hat{w}
   *   \end{bmatrix}
   * @f] where @f$ \xi = (w, v) @f$ and @f$ \hat{w} @f$ and @f$ \hat{v} @f$ are
   * the hat operators of @f$ w @f$ and @f$ v @f$ respectively.
   *
   * @param xi The vector to be converted to matrix
   * @see Vee()
   */
  template <typename MatrixDrived>
  [[nodiscard]] static Matrix4<S> Hat(
      const Eigen::MatrixBase<MatrixDrived>& xi);

  /// Returns the vee operator of the given matrix
  template <typename MatrixDerived>
  [[nodiscard]] static Vector6<S> Vee(
      const Eigen::MatrixBase<MatrixDerived>& matrix);

  /// Returns the adjoint transformation matrix (6x6) of the given SE(3) point
  template <typename OtherDerived>
  [[nodiscard]] static Matrix6<S> Ad(const SE3Base<OtherDerived>& x);

  /// Performs the adjoint transformation on the given se(3) by the given SE(3)
  template <typename OtherDerived, typename MatrixDerived>
  [[nodiscard]] static Tangent Ad(
      const SE3Base<OtherDerived>& x,
      const Eigen::MatrixBase<MatrixDerived>& xi);

  template <typename MatrixDerived>
  [[nodiscard]] static Matrix6<S> LieBracket(
      const Eigen::MatrixBase<MatrixDerived>& dx);

  template <typename DerivedA, typename DerivedB>
  [[nodiscard]] static Tangent LieBracket(
      const Eigen::MatrixBase<DerivedA>& dx1,
      const Eigen::MatrixBase<DerivedB>& dx2);

  template <typename MatrixDerived>
  [[nodiscard]] static Matrix6<S> Cross(
      const Eigen::MatrixBase<MatrixDerived>& dx);

  template <typename DerivedA, typename DerivedB>
  [[nodiscard]] static Tangent Cross(
      const Eigen::MatrixBase<DerivedA>& dx1,
      const Eigen::MatrixBase<DerivedB>& dx2);

  /// Returns the left Jacobian of the exponential map
  ///
  /// @param[in] xi The vector to be converted to a matrix.
  /// @param[in] tol The tolerance for the norm of the vector.
  /// @return The matrix.
  /// @tparam MatrixDrived The type of the vector
  /// @see RightJacobian()
  template <typename MatrixDrived>
  [[nodiscard]] static Matrix6<S> LeftJacobian(
      const Eigen::MatrixBase<MatrixDrived>& xi, S tol = Tolerance());

  /// Returns the right Jacobian of the exponential map
  ///
  /// @param[in] xi The vector to be converted to a matrix.
  /// @param[in] tol The tolerance for the norm of the vector.
  /// @return The matrix.
  /// @tparam MatrixDrived The type of the vector
  /// @see LeftJacobian()
  template <typename MatrixDrived>
  [[nodiscard]] static Matrix6<S> RightJacobian(
      const Eigen::MatrixBase<MatrixDrived>& xi, S tol = Tolerance());

  /// Returns the left Jacobian inverse of the exponential map
  ///
  /// @param[in] dx The vector to be converted to a matrix.
  /// @param[in] tol The tolerance for the norm of the vector.
  /// @return The matrix.
  /// @tparam MatrixDrived The type of the vector
  /// @see RightJacobianInverse()
  template <typename MatrixDrived>
  [[nodiscard]] static Matrix6<S> LeftJacobianInverse(
      const Eigen::MatrixBase<MatrixDrived>& dx, S tol = Tolerance());

  /// Returns the right Jacobian inverse of the exponential map
  ///
  /// @param[in] dx The vector to be converted to a matrix.
  /// @param[in] tol The tolerance for the norm of the vector.
  /// @return The matrix.
  /// @tparam MatrixDrived The type of the vector
  /// @see LeftJacobianInverse()
  template <typename MatrixDerived>
  [[nodiscard]] static Matrix6<S> RightJacobianInverse(
      const Eigen::MatrixBase<MatrixDerived>& dx, S tol = Tolerance());

  /// Default constructor that initializes the quaternion to identity and the
  /// translation to zero
  SE3();

  /// Default constructor that does not initialize the quaternion and the
  /// translation
  ///
  /// This constructor is useful for performance reasons when the quaternion and
  /// translation are going to be initialized later.
  explicit SE3(NoInitializeTag);

  DART_DEFINE_CONSTRUCTORS_FOR_CONCRETE(SE3);

  /// Constructs an SE3 from a rotation (or SE(3)) and a translation (or R(3))
  ///
  /// @param rotation The rotation
  /// @param translation The translation
  template <typename SO3Derived, typename MatrixDerived>
  explicit SE3(
      const SO3Base<SO3Derived>& rotation,
      const Eigen::MatrixBase<MatrixDerived>& translation);

  template <typename SO3Derived, typename MatrixDerived>
  explicit SE3(
      SO3Base<SO3Derived>&& rotation,
      Eigen::MatrixBase<MatrixDerived>&& translation);

  /// Copy assignment operator
  SE3& operator=(const SE3& other);

  /// Move assignment operator
  SE3& operator=(SE3&& other) noexcept;

  /// Returns the underlying parameters
  [[nodiscard]] const Params& params() const;

  /// Returns the underlying parameters
  [[nodiscard]] Params& params();

private:
  /// The underlying parameters
  Params m_params;
};

DART_TEMPLATE_CLASS_HEADER(MATH, SE3);

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

namespace dart::math {

namespace detail {

//==============================================================================
template <typename S, typename DerivedA, typename DerivedB>
Matrix3<S> computeQ(
    S theta,
    const Eigen::MatrixBase<DerivedA>& phiX,
    const Eigen::MatrixBase<DerivedB>& rhoX)
{
  Matrix3<S> Q;

  const S theta2 = theta * theta;
  const S theta3 = theta2 * theta;
  const S theta4 = theta3 * theta;
  const S theta5 = theta4 * theta;
  const S sin_theta = std::sin(theta);
  const S cos_theta = std::cos(theta);
  const S tmp = (1 - 0.5 * theta2 - cos_theta) / theta4;

  const Matrix3<S> phiX_rhoX = phiX * rhoX;
  const Matrix3<S> rhoX_phiX = rhoX * phiX;
  const Matrix3<S> phiX_rhoX_phiX = phiX_rhoX * phiX;

  Q = 0.5 * rhoX;
  Q.noalias() += ((theta - sin_theta) / theta3)
                 * (phiX_rhoX + rhoX_phiX + phiX_rhoX_phiX);
  Q.noalias()
      -= tmp * (phiX * phiX_rhoX + rhoX_phiX * phiX - 3 * phiX_rhoX_phiX);
  Q.noalias() -= 0.5 * (tmp - 3 * (theta - sin_theta - theta3 / 6) / theta5)
                 * (phiX_rhoX_phiX * phiX + phiX * phiX_rhoX_phiX);

  return Q;
}

} // namespace detail

//==============================================================================
template <typename S>
template <typename MatrixDerived>
SE3<S> SE3<S>::Exp(const Eigen::MatrixBase<MatrixDerived>& dx, const S tol)
{
  const SO3<S> rotation = SO3<S>::Exp(dx.template head<3>(), tol);
  const Eigen::Vector3<S> translation
      = SO3<S>::LeftJacobian(dx.template head<3>(), tol)
        * dx.template tail<3>();
  // TODO(JS): Check if this version is faster than expMap() and expMapRot()
  return SE3(std::move(rotation), std::move(translation));
}

//==============================================================================
template <typename S>
template <typename MatrixDerivedA, typename MatrixDerivedB>
SE3<S> SE3<S>::Exp(
    const Eigen::MatrixBase<MatrixDerivedA>& dx,
    Eigen::MatrixBase<MatrixDerivedB>* jacobian,
    S tol)
{
  if (jacobian) {
    (*jacobian) = RightJacobian(dx, tol);
  }
  return Exp(dx, tol);
}

//==============================================================================
template <typename S>
template <typename OtherDerived>
typename SE3<S>::Tangent SE3<S>::Log(const SE3Base<OtherDerived>& x, S tol)
{
  const Eigen::Vector3<S> angular = SO3<S>::Log(x.rotation(), tol).params();
  const Eigen::Vector3<S> linear
      = SO3<S>::LeftJacobianInverse(angular, tol) * x.translation();
  return Tangent(std::move(angular), std::move(linear));
}

//==============================================================================
template <typename S>
template <typename OtherDerived, typename MatrixDerived>
typename SE3<S>::Tangent SE3<S>::Log(
    const SE3Base<OtherDerived>& x,
    Eigen::MatrixBase<MatrixDerived>* jacobian,
    S tol)
{
  const Tangent xi = Log(x, tol);
  if (jacobian) {
    (*jacobian) = RightJacobianInverse(xi, tol);
  }
  return xi;
}

//==============================================================================
template <typename S>
template <typename MatrixDerived>
Matrix4<S> SE3<S>::Hat(const Eigen::MatrixBase<MatrixDerived>& xi)
{
  Matrix4<S> out = Matrix4<S>::Zero();
  out.template topLeftCorner<3, 3>() = SO3<S>::Hat(xi.template head<3>());
  out.template topRightCorner<3, 1>() = xi.template tail<3>();
  out.template bottomRows<1>().setZero();
  return out;
}

//==============================================================================
template <typename S>
template <typename MatrixDerived>
Vector6<S> SE3<S>::Vee(const Eigen::MatrixBase<MatrixDerived>& matrix)
{
  Vector6<S> out;
  out.template head<3>() = SO3<S>::Vee(matrix.template topLeftCorner<3, 3>());
  out.template tail<3>() = matrix.template topRightCorner<3, 1>();
  return out;
}

//==============================================================================
template <typename S>
template <typename OtherDerived>
Matrix6<S> SE3<S>::Ad(const SE3Base<OtherDerived>& x)
{
  Matrix6<S> out;
  out.template topLeftCorner<3, 3>() = x.rotation().matrix();
  out.template topRightCorner<3, 3>().setZero();
  out.template bottomLeftCorner<3, 3>().noalias()
      = SO3<S>::Hat(x.translation()) * out.template topLeftCorner<3, 3>();
  out.template bottomRightCorner<3, 3>() = out.template topLeftCorner<3, 3>();
  return out;
}

//==============================================================================
template <typename S>
template <typename OtherDerived, typename MatrixDerived>
typename SE3<S>::Tangent SE3<S>::Ad(
    const SE3Base<OtherDerived>& x, const Eigen::MatrixBase<MatrixDerived>& xi)
{
  // Cache the rotation matrix for efficiency when multiplying 3d vector more
  // than once
  const Eigen::Matrix<S, 3, 3> rotation = x.rotation().toMatrix();

  const Vector3<S> angular = rotation * xi.template head<3>();
  Vector3<S> linear = rotation * xi.template tail<3>();
  linear.noalias() += x.translation().cross(angular);
  return Tangent(std::move(angular), std::move(linear));
}

//==============================================================================
template <typename S>
template <typename MatrixDerived>
Matrix6<S> SE3<S>::LieBracket(const Eigen::MatrixBase<MatrixDerived>& dx)
{
  Matrix6<S> out;
  out.template topLeftCorner<3, 3>() = SO3<S>::Hat(dx.template head<3>());
  out.template bottomLeftCorner<3, 3>() = SO3<S>::Hat(dx.template tail<3>());
  out.template topRightCorner<3, 3>().setZero();
  out.template bottomRightCorner<3, 3>() = out.template topLeftCorner<3, 3>();
  return out;
}

//==============================================================================
template <typename S>
template <typename DerivedA, typename DerivedB>
typename SE3<S>::Tangent SE3<S>::LieBracket(
    const Eigen::MatrixBase<DerivedA>& dx1,
    const Eigen::MatrixBase<DerivedB>& dx2)
{
  //--------------------------------------------------------------------------
  // ad(s1, s2) = | [w1]    0 | | w2 |
  //              | [v1] [w1] | | v2 |
  //
  //            = |          [w1]w2 |
  //              | [v1]w2 + [w1]v2 |
  //--------------------------------------------------------------------------
  Tangent out;
  as_angular(out).noalias() = as_angular(dx1).cross(as_angular(dx2));
  as_linear(out).noalias() = as_angular(dx1).cross(as_linear(dx2));
  return out;
}

//==============================================================================
template <typename S>
template <typename MatrixDerived>
Matrix6<S> SE3<S>::Cross(const Eigen::MatrixBase<MatrixDerived>& dx)
{
  return LieBracket(dx);
}

//==============================================================================
template <typename S>
template <typename DerivedA, typename DerivedB>
typename SE3<S>::Tangent SE3<S>::Cross(
    const Eigen::MatrixBase<DerivedA>& dx1,
    const Eigen::MatrixBase<DerivedB>& dx2)
{
  return LieBracket(dx1, dx2);
}

//==============================================================================
template <typename S>
template <typename MatrixDerived>
Matrix6<S> SE3<S>::LeftJacobian(
    const Eigen::MatrixBase<MatrixDerived>& xi, S tol)
{
  // Equation (100) in "Associating Uncertainty With Three-Dimensional Poses for
  // Use in Estimation Problems" by Timothy D. Barfoot and Paul Furgale
  //
  // The Jacobian is defined as:
  //   A = [ J 0 ]
  //       [ Q J ]
  // where J is the left Jacobian of SO(3) and Q is defined in Equation (102).

  const Vector3<S> phi = xi.template head<3>();
  const Vector3<S> rho = xi.template tail<3>();
  const Matrix3<S> phiX = SO3<S>::Hat(phi);
  const Matrix3<S> rhoX = SO3<S>::Hat(rho);
  const S theta = phi.norm();

  Matrix6<S> J;

  J.template topRightCorner<3, 3>().setZero();
  J.template topLeftCorner<3, 3>() = SO3<S>::LeftJacobian(phi);
  J.template bottomRightCorner<3, 3>() = J.template topLeftCorner<3, 3>();

  if (theta < tol) {
    J.template bottomLeftCorner<3, 3>().noalias() = 0.5 * rhoX;
  } else {
    J.template bottomLeftCorner<3, 3>() = detail::computeQ(theta, phiX, rhoX);
  }

  return J;
}

//==============================================================================
template <typename S>
template <typename MatrixDerived>
Matrix6<S> SE3<S>::RightJacobian(
    const Eigen::MatrixBase<MatrixDerived>& xi, S tol)
{
  return LeftJacobian(-xi, tol);
}

//==============================================================================
template <typename S>
template <typename MatrixDerived>
Matrix6<S> SE3<S>::LeftJacobianInverse(
    const Eigen::MatrixBase<MatrixDerived>& xi, S tol)
{
  // Equation (103) in "Associating Uncertainty With Three-Dimensional Poses for
  // Use in Estimation Problems" by Timothy D. Barfoot and Paul Furgale
  //
  // The Jacobian is defined as:
  //   A = [  J^(-1)      0      ]
  //       [ -J^(-1) * Q  J^(-1) ]
  // where J is the left Jacobian of SO(3) and Q is defined in Equation (102).

  const Vector3<S> phi = xi.template head<3>();
  const Vector3<S> rho = xi.template tail<3>();
  const Matrix3<S> phiX = SO3<S>::Hat(phi);
  const Matrix3<S> rhoX = SO3<S>::Hat(rho);
  const S theta = phi.norm();

  Matrix6<S> J;

  const Matrix3<S> Jinv = SO3<S>::LeftJacobian(phi).inverse();

  J.template topRightCorner<3, 3>().setZero();
  J.template topLeftCorner<3, 3>() = Jinv;
  J.template bottomRightCorner<3, 3>() = Jinv;

  if (theta < tol) {
    J.template bottomLeftCorner<3, 3>().noalias() = -Jinv * (0.5 * rhoX) * Jinv;
  } else {
    J.template bottomLeftCorner<3, 3>().noalias()
        = -Jinv * detail::computeQ(theta, phiX, rhoX) * Jinv;
  }

  return J;
}

//==============================================================================
template <typename S>
template <typename MatrixDerived>
Matrix6<S> SE3<S>::RightJacobianInverse(
    const Eigen::MatrixBase<MatrixDerived>& xi, S tol)
{
  return LeftJacobianInverse(-xi, tol);
}

//==============================================================================
template <typename S>
SE3<S>::SE3() : m_params(Params::Zero())
{
  params().w() = 1;
}

//==============================================================================
template <typename S>
SE3<S>::SE3(NoInitializeTag)
{
  // Do nothing
}

//==============================================================================
template <typename S>
template <typename SO3Derived, typename MatrixDerived>
SE3<S>::SE3(
    const SO3Base<SO3Derived>& rotation,
    const Eigen::MatrixBase<MatrixDerived>& translation)
{
  params().template head<4>() = rotation.params();
  params().template tail<3>() = translation;
}

//==============================================================================
template <typename S>
template <typename SO3Derived, typename MatrixDerived>
SE3<S>::SE3(
    SO3Base<SO3Derived>&& rotation,
    Eigen::MatrixBase<MatrixDerived>&& translation)
{
  params().template head<4>() = std::move(rotation.params());
  params().template tail<3>() = std::move(translation);
}

//==============================================================================
template <typename S>
SE3<S>& SE3<S>::operator=(const SE3& other)
{
  m_params = other.params();
  return *this;
}

//==============================================================================
template <typename S>
SE3<S>& SE3<S>::operator=(SE3&& other) noexcept
{
  m_params = std::move(other.params());
  return *this;
}

//==============================================================================
template <typename S>
const typename SE3<S>::Params& SE3<S>::params() const
{
  return m_params;
}

//==============================================================================
template <typename S>
typename SE3<S>::Params& SE3<S>::params()
{
  return m_params;
}

} // namespace dart::math
