/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include <dart/simulation/experimental/export.hpp>

#include <dart/math/Geometry.hpp>
#include <dart/math/MathTypes.hpp>

#include <Eigen/Dense>

namespace dart::simulation::experimental::dynamics {

using SpatialVelocity = Eigen::Vector6d;
using SpatialAcceleration = Eigen::Vector6d;
using SpatialForce = Eigen::Vector6d;
using SpatialInertia = Eigen::Matrix6d;
using MotionSubspace = Eigen::Matrix<double, 6, Eigen::Dynamic>;
using MotionSubspace1 = Eigen::Matrix<double, 6, 1>;
using MotionSubspace2 = Eigen::Matrix<double, 6, 2>;
using MotionSubspace3 = Eigen::Matrix<double, 6, 3>;
using MotionSubspace6 = Eigen::Matrix<double, 6, 6>;

[[nodiscard]] DART_EXPERIMENTAL_API SpatialInertia makeSpatialInertia(
    double mass, const Eigen::Vector3d& com, const Eigen::Matrix3d& inertia);

[[nodiscard]] DART_EXPERIMENTAL_API SpatialInertia
makePointMassInertia(double mass, const Eigen::Vector3d& position);

[[nodiscard]] inline SpatialInertia transformSpatialInertia(
    const Eigen::Isometry3d& T, const SpatialInertia& I)
{
  return math::transformInertia(T, I);
}

[[nodiscard]] inline SpatialVelocity makeSpatialVelocity(
    const Eigen::Vector3d& angular, const Eigen::Vector3d& linear)
{
  return Eigen::compose(angular, linear);
}

[[nodiscard]] inline SpatialForce makeSpatialForce(
    const Eigen::Vector3d& torque, const Eigen::Vector3d& force)
{
  return Eigen::compose(torque, force);
}

[[nodiscard]] inline Eigen::Vector3d angular(const Eigen::Vector6d& V)
{
  return V.head<3>();
}

[[nodiscard]] inline Eigen::Vector3d linear(const Eigen::Vector6d& V)
{
  return V.tail<3>();
}

[[nodiscard]] inline SpatialVelocity spatialCross(
    const SpatialVelocity& V1, const SpatialVelocity& V2)
{
  return math::ad(V1, V2);
}

[[nodiscard]] inline SpatialForce spatialCrossStar(
    const SpatialVelocity& V, const SpatialForce& F)
{
  return math::dad(V, F);
}

[[nodiscard]] inline SpatialVelocity transformSpatialVelocity(
    const Eigen::Isometry3d& T, const SpatialVelocity& V)
{
  return math::AdT(T, V);
}

[[nodiscard]] inline SpatialVelocity transformSpatialVelocityInverse(
    const Eigen::Isometry3d& T, const SpatialVelocity& V)
{
  return math::AdInvT(T, V);
}

[[nodiscard]] inline SpatialForce transformSpatialForce(
    const Eigen::Isometry3d& T, const SpatialForce& F)
{
  return math::dAdT(T, F);
}

[[nodiscard]] inline SpatialForce transformSpatialForceInverse(
    const Eigen::Isometry3d& T, const SpatialForce& F)
{
  return math::dAdInvT(T, F);
}

} // namespace dart::simulation::experimental::dynamics
