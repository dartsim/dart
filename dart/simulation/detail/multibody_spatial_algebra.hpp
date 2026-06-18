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

#include "dart/simulation/comps/dynamics.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

/// Shared 6D spatial-algebra primitives for the rigid-body multibody compute
/// stages. The semi-implicit joint-space path (`compute/multibody_dynamics`)
/// and the linear-time variational integrator
/// (`compute/variational_integration`) both build their per-link
/// inertia/subspace data from the same kernels, so the byte-identical helpers
/// live here to avoid divergence. All spatial quantities use the `[angular;
/// linear]` twist convention.
namespace dart::simulation::detail {

using Vector6 = Eigen::Matrix<double, 6, 1>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;
using Subspace = Eigen::Matrix<double, 6, Eigen::Dynamic>;

//==============================================================================
/// Skew-symmetric (cross-product) matrix of a 3-vector.
[[nodiscard]] inline Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

//==============================================================================
/// Spatial motion adjoint for the [angular; linear] convention. For a transform
/// T = (R, p) that expresses frame B in frame A (x_A = R x_B + p), this maps a
/// spatial motion vector from B to A.
[[nodiscard]] inline Matrix6 adjoint(const Eigen::Isometry3d& transform)
{
  const Eigen::Matrix3d rotation = transform.linear();
  const Eigen::Vector3d translation = transform.translation();

  Matrix6 result = Matrix6::Zero();
  result.topLeftCorner<3, 3>() = rotation;
  result.bottomLeftCorner<3, 3>() = skew(translation) * rotation;
  result.bottomRightCorner<3, 3>() = rotation;
  return result;
}

//==============================================================================
/// Spatial inertia about the link frame origin in the [angular; linear]
/// convention, for a body with rotational inertia `inertia` about its center of
/// mass `c`:
///   [[ I_C - m c x c x , m c x ],
///    [ -m c x          , m 1   ]].
/// With the center of mass at the origin (c = 0) this reduces to the
/// block-diagonal form diag(I_C, m 1).
[[nodiscard]] inline Matrix6 spatialInertia(const comps::MassProperties& mass)
{
  const Eigen::Matrix3d comCross = skew(mass.localCenterOfMass);
  Matrix6 result = Matrix6::Zero();
  result.topLeftCorner<3, 3>() = mass.inertia - mass.mass * comCross * comCross;
  result.topRightCorner<3, 3>() = mass.mass * comCross;
  result.bottomLeftCorner<3, 3>() = -mass.mass * comCross;
  result.bottomRightCorner<3, 3>() = mass.mass * Eigen::Matrix3d::Identity();
  return result;
}

} // namespace dart::simulation::detail
