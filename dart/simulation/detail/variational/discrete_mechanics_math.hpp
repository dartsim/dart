/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License
 */

#pragma once

#include <dart/math/lie_group/se3.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

/// SE(3) discrete-mechanics calculus kernels for the linear-time variational
/// integrator (PLAN-084). All spatial quantities use the `[angular; linear]`
/// twist convention that matches `compute/multibody_dynamics.cpp`
/// (`adjoint`/`motionCross`/`spatialInertia`).
///
/// Ported from the author's reference implementation (`dart/math/Geometry.cpp`
/// in github.com/jslee02/wafr2016) and re-expressed as standalone pure
/// functions so they can be unit-tested in isolation. The Bernoulli-series
/// truncation for `dexpInv`/`dexpInvTranspose` is the WAFR-2016 4th-order form
/// `I - ad/2 + ad^2/12 - ad^4/720`.
namespace dart::simulation::detail::variational {

using Vector6 = Eigen::Matrix<double, 6, 1>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;

inline constexpr double kSmallAngle = 1e-6;
inline constexpr double kPi = 3.14159265358979323846;

//==============================================================================
/// Spatial-motion Lie bracket ad_X(Y) = [X, Y] for twists in [angular; linear].
[[nodiscard]] inline Vector6 ad(const Vector6& x, const Vector6& y)
{
  Vector6 result;
  result.head<3>() = x.head<3>().cross(y.head<3>());
  result.tail<3>()
      = x.head<3>().cross(y.tail<3>()) + x.tail<3>().cross(y.head<3>());
  return result;
}

//==============================================================================
/// Dual (co-)adjoint bracket dad_X(Y) = ad_X^*(Y), the transpose action of `ad`
/// used to transport spatial momentum/force.
[[nodiscard]] inline Vector6 dad(const Vector6& x, const Vector6& y)
{
  Vector6 result;
  result.head<3>()
      = y.head<3>().cross(x.head<3>()) + y.tail<3>().cross(x.tail<3>());
  result.tail<3>() = y.tail<3>().cross(x.head<3>());
  return result;
}

//==============================================================================
/// Inverse right-trivialized tangent of exp applied to W: `dexp^{-1}_V(W)` (the
/// paper's `dlog_V`). This is exactly the SE(3) left-Jacobian inverse, so it
/// delegates to the shared `dart::math::lie_group` library, which provides the
/// exact Barfoot-Furgale closed form (strictly more accurate than the paper's
/// 4th-order Bernoulli truncation `I - ad/2 + ad^2/12 - ad^4/720`, whose
/// truncation error is ~machine-epsilon at millisecond steps). The
/// `[angular; linear]` twist convention matches. See the
/// `test_discrete_mechanics_lie_group_parity` cross-check.
[[nodiscard]] inline Vector6 dexpInv(const Vector6& v, const Vector6& w)
{
  return ::dart::math::SE3<double>::LeftJacobianInverse(v) * w;
}

//==============================================================================
/// Transpose of `dexpInv`: `dexp^{-T}_V(W)`, the workhorse for the discrete
/// momentum `mu = dexpInvTranspose(dt * V, G * V)`. Delegates to the transpose
/// of the shared SE(3) left-Jacobian inverse.
[[nodiscard]] inline Vector6 dexpInvTranspose(
    const Vector6& v, const Vector6& w)
{
  return ::dart::math::SE3<double>::LeftJacobianInverse(v).transpose() * w;
}

//==============================================================================
/// Right-trivialized inverse `dexp^{-1}_{V,right}` (6x6) of the exponential
/// map: the matrix `L` with `d/ds log(T exp(s xi))|_0 = L xi` for `T = exp(V)`.
/// By the Barfoot-Furgale identity this is the left-Jacobian inverse evaluated
/// at
/// `-V`. The exact recursive-Jacobian Newton step uses it to linearize the
/// average velocity `Vbar = log(dT)/dt` with respect to a body-frame
/// perturbation of the relative transform `dT`.
[[nodiscard]] inline Matrix6 dexpInvMatrixRight(const Vector6& v)
{
  return ::dart::math::SE3<double>::RightJacobianInverse(v);
}

//==============================================================================
/// SE(3) exponential map exp(xi) for a twist xi = [angular; linear].
[[nodiscard]] inline Eigen::Isometry3d se3Exp(const Vector6& xi)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  const double s2[] = {xi[0] * xi[0], xi[1] * xi[1], xi[2] * xi[2]};
  const double s3[] = {xi[0] * xi[1], xi[1] * xi[2], xi[2] * xi[0]};
  const double theta = std::sqrt(s2[0] + s2[1] + s2[2]);
  const double cosT = std::cos(theta);
  double alpha = 0.0;
  double beta = 0.0;
  double gamma = 0.0;
  const double dotWV = xi[0] * xi[3] + xi[1] * xi[4] + xi[2] * xi[5];

  if (theta > kSmallAngle) {
    const double sinT = std::sin(theta);
    alpha = sinT / theta;
    beta = (1.0 - cosT) / theta / theta;
    gamma = dotWV * (theta - sinT) / theta / theta / theta;
  } else {
    alpha = 1.0 - theta * theta / 6.0;
    beta = 0.5 - theta * theta / 24.0;
    gamma = dotWV / 6.0 - theta * theta / 120.0;
  }

  Eigen::Matrix3d r;
  r(0, 0) = beta * s2[0] + cosT;
  r(1, 0) = beta * s3[0] + alpha * xi[2];
  r(2, 0) = beta * s3[2] - alpha * xi[1];
  r(0, 1) = beta * s3[0] - alpha * xi[2];
  r(1, 1) = beta * s2[1] + cosT;
  r(2, 1) = beta * s3[1] + alpha * xi[0];
  r(0, 2) = beta * s3[2] + alpha * xi[1];
  r(1, 2) = beta * s3[1] - alpha * xi[0];
  r(2, 2) = beta * s2[2] + cosT;
  result.linear() = r;

  result.translation()[0]
      = alpha * xi[3] + beta * (xi[1] * xi[5] - xi[2] * xi[4]) + gamma * xi[0];
  result.translation()[1]
      = alpha * xi[4] + beta * (xi[2] * xi[3] - xi[0] * xi[5]) + gamma * xi[1];
  result.translation()[2]
      = alpha * xi[5] + beta * (xi[0] * xi[4] - xi[1] * xi[3]) + gamma * xi[2];

  return result;
}

//==============================================================================
/// SE(3) logarithm map log(T) returning a twist [angular; linear]. Handles the
/// theta -> pi singularity with a dedicated branch.
[[nodiscard]] inline Vector6 se3Log(const Eigen::Isometry3d& t)
{
  const Eigen::Matrix3d& rot = t.matrix().topLeftCorner<3, 3>();
  const double trace = rot(0, 0) + rot(1, 1) + rot(2, 2);
  const double theta
      = std::acos(std::max(std::min(0.5 * (trace - 1.0), 1.0), -1.0));
  Vector6 result;
  const auto& p = t.translation();

  if (theta > kPi - kSmallAngle) {
    const double c1 = 0.10132118364234; // 1 / pi^2
    const double c2 = 0.01507440267955; // 1 / 4 / pi - 2 / pi^3
    const double c3 = 0.00546765085347; // 3 / pi^4 - 1 / 4 / pi^2
    const double phi = kPi - theta;
    const double delta = 0.5 + 0.125 * phi * phi;

    const double w[]
        = {rot(2, 1) > rot(1, 2)
               ? theta * std::sqrt(1.0 + (rot(0, 0) - 1.0) * delta)
               : -theta * std::sqrt(1.0 + (rot(0, 0) - 1.0) * delta),
           rot(0, 2) > rot(2, 0)
               ? theta * std::sqrt(1.0 + (rot(1, 1) - 1.0) * delta)
               : -theta * std::sqrt(1.0 + (rot(1, 1) - 1.0) * delta),
           rot(1, 0) > rot(0, 1)
               ? theta * std::sqrt(1.0 + (rot(2, 2) - 1.0) * delta)
               : -theta * std::sqrt(1.0 + (rot(2, 2) - 1.0) * delta)};

    const double beta = 0.25 * theta * (kPi - theta);
    const double gamma = (w[0] * p[0] + w[1] * p[1] + w[2] * p[2])
                         * (c1 - c2 * phi + c3 * phi * phi);

    result << w[0], w[1], w[2],
        beta * p[0] - 0.5 * (w[1] * p[2] - w[2] * p[1]) + gamma * w[0],
        beta * p[1] - 0.5 * (w[2] * p[0] - w[0] * p[2]) + gamma * w[1],
        beta * p[2] - 0.5 * (w[0] * p[1] - w[1] * p[0]) + gamma * w[2];
    return result;
  }

  double alpha = 0.0;
  double beta = 0.0;
  double gamma = 0.0;
  if (theta > kSmallAngle) {
    alpha = 0.5 * theta / std::sin(theta);
    beta = (1.0 + std::cos(theta)) * alpha;
    gamma = (1.0 - beta) / theta / theta;
  } else {
    alpha = 0.5 + theta * theta / 12.0;
    beta = 1.0 - theta * theta / 12.0;
    gamma = 1.0 / 12.0 + theta * theta / 720.0;
  }

  const double w[]
      = {alpha * (rot(2, 1) - rot(1, 2)),
         alpha * (rot(0, 2) - rot(2, 0)),
         alpha * (rot(1, 0) - rot(0, 1))};
  gamma *= w[0] * p[0] + w[1] * p[1] + w[2] * p[2];

  result << w[0], w[1], w[2],
      beta * p[0] + 0.5 * (w[2] * p[1] - w[1] * p[2]) + gamma * w[0],
      beta * p[1] + 0.5 * (w[0] * p[2] - w[2] * p[0]) + gamma * w[1],
      beta * p[2] + 0.5 * (w[1] * p[0] - w[0] * p[1]) + gamma * w[2];
  return result;
}

//==============================================================================
/// Dual adjoint dAd_T(F) = Ad_T^{*}(F) = adjoint(T)^T * F, transporting a
/// spatial force/momentum F through the transform T (T expresses the child in
/// the parent).
[[nodiscard]] inline Vector6 dAdT(const Eigen::Isometry3d& t, const Vector6& f)
{
  Vector6 result;
  result.head<3>().noalias()
      = t.linear().transpose()
        * (f.head<3>() + f.tail<3>().cross(t.translation()));
  result.tail<3>().noalias() = t.linear().transpose() * f.tail<3>();
  return result;
}

//==============================================================================
/// Dual of the inverse adjoint dAd_{T^{-1}}(F) = adjoint(T^{-1})^T * F, used to
/// transmit a child's spatial impulse to its parent.
[[nodiscard]] inline Vector6 dAdInvT(
    const Eigen::Isometry3d& t, const Vector6& f)
{
  Vector6 result;
  result.tail<3>().noalias() = t.linear() * f.tail<3>();
  result.head<3>().noalias() = t.linear() * f.head<3>();
  result.head<3>() += t.translation().cross(result.tail<3>());
  return result;
}

//==============================================================================
/// Maps a world-frame linear acceleration (e.g. gravity) into a body-frame
/// spatial vector [0; R^T a], for a transform T expressing the body in the
/// world.
[[nodiscard]] inline Vector6 adInvRLinear(
    const Eigen::Isometry3d& t, const Eigen::Vector3d& a)
{
  Vector6 result = Vector6::Zero();
  result.tail<3>().noalias() = t.linear().transpose() * a;
  return result;
}

} // namespace dart::simulation::detail::variational
