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

#include <dart/simulation/export.hpp>

#include <dart/collision/native/types.hpp>

#include <Eigen/Core>

namespace dart::simulation::detail {

/// Rigid pose used by the internal rigid IPC CCD experiments.
///
/// Rotation is stored as a 3D rotation vector and interpolated linearly across
/// the unit step, matching the reference rigid-contact fixtures.
struct RigidIpcPose
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d rotation = Eigen::Vector3d::Zero();
};

[[nodiscard]] DART_SIMULATION_API Eigen::Matrix3d
rigidIpcRotationVectorToMatrix(const Eigen::Vector3d& rotation);

[[nodiscard]] DART_SIMULATION_API RigidIpcPose interpolateRigidIpcPose(
    const RigidIpcPose& start, const RigidIpcPose& end, double time);

[[nodiscard]] DART_SIMULATION_API Eigen::Vector3d transformRigidIpcPoint(
    const Eigen::Vector3d& localPoint, const RigidIpcPose& pose);

[[nodiscard]] DART_SIMULATION_API Eigen::Vector3d transformRigidIpcPoint(
    const Eigen::Vector3d& localPoint,
    const RigidIpcPose& start,
    const RigidIpcPose& end,
    double time);

/// Conservative bound on how far a body-local point travels over the unit step
/// under linear translation and rotation-vector interpolation.
///
/// Returns `||end.position - start.position|| + ||end.rotation -
/// start.rotation|| * ||localPoint||`. This bounds the displacement of the
/// transformed world point from its start position for every time in [0, 1]; it
/// is the same motion bound the curved ACCD queries advance against, so reusing
/// it for swept broad-phase culling stays consistent with the per-primitive
/// CCD.
[[nodiscard]] DART_SIMULATION_API double rigidIpcPointTrajectorySpeedBound(
    const Eigen::Vector3d& localPoint,
    const RigidIpcPose& start,
    const RigidIpcPose& end);

/// Residual for the rigid IPC point-edge contact equation at a parameterized
/// time and edge coordinate.
///
/// The edge coordinate is intentionally not clamped. Reference root finding
/// handles the domain separately from the equation value.
[[nodiscard]] DART_SIMULATION_API Eigen::Vector3d rigidIpcPointEdgeResidual(
    const Eigen::Vector3d& point,
    const RigidIpcPose& pointPoseStart,
    const RigidIpcPose& pointPoseEnd,
    const Eigen::Vector3d& edgeA,
    const Eigen::Vector3d& edgeB,
    const RigidIpcPose& edgePoseStart,
    const RigidIpcPose& edgePoseEnd,
    double time,
    double alpha);

/// Residual for the rigid IPC edge-edge contact equation at a parameterized
/// time and edge coordinates.
[[nodiscard]] DART_SIMULATION_API Eigen::Vector3d rigidIpcEdgeEdgeResidual(
    const Eigen::Vector3d& edgeA0,
    const Eigen::Vector3d& edgeA1,
    const RigidIpcPose& edgeAPoseStart,
    const RigidIpcPose& edgeAPoseEnd,
    const Eigen::Vector3d& edgeB0,
    const Eigen::Vector3d& edgeB1,
    const RigidIpcPose& edgeBPoseStart,
    const RigidIpcPose& edgeBPoseEnd,
    double time,
    double alpha,
    double beta);

/// Residual for the rigid IPC point-triangle contact equation at a
/// parameterized time and triangle coordinates.
[[nodiscard]] DART_SIMULATION_API Eigen::Vector3d rigidIpcPointTriangleResidual(
    const Eigen::Vector3d& point,
    const RigidIpcPose& pointPoseStart,
    const RigidIpcPose& pointPoseEnd,
    const Eigen::Vector3d& triangleA,
    const Eigen::Vector3d& triangleB,
    const Eigen::Vector3d& triangleC,
    const RigidIpcPose& trianglePoseStart,
    const RigidIpcPose& trianglePoseEnd,
    double time,
    double u,
    double v);

/// Returns whether triangle coordinates lie in the face-vertex domain.
[[nodiscard]] DART_SIMULATION_API bool rigidIpcFaceVertexDomainContains(
    double u, double v, double tolerance = 0.0);

/// Parameter-space subdivision query for rigid point-edge time of impact.
///
/// This mirrors the reference rigid IPC search domain `(t, alpha)` and reports
/// a conservative lower bound on the first zero-contact time found by
/// recursively subdividing the parameter box.
[[nodiscard]] DART_SIMULATION_API bool rigidIpcPointEdgeIntervalCcd(
    const Eigen::Vector3d& point,
    const RigidIpcPose& pointPoseStart,
    const RigidIpcPose& pointPoseEnd,
    const Eigen::Vector3d& edgeA,
    const Eigen::Vector3d& edgeB,
    const RigidIpcPose& edgePoseStart,
    const RigidIpcPose& edgePoseEnd,
    const collision::native::CcdOption& option,
    collision::native::CcdPrimitiveResult& result);

/// Parameter-space subdivision query for rigid edge-edge time of impact.
[[nodiscard]] DART_SIMULATION_API bool rigidIpcEdgeEdgeIntervalCcd(
    const Eigen::Vector3d& edgeA0,
    const Eigen::Vector3d& edgeA1,
    const RigidIpcPose& edgeAPoseStart,
    const RigidIpcPose& edgeAPoseEnd,
    const Eigen::Vector3d& edgeB0,
    const Eigen::Vector3d& edgeB1,
    const RigidIpcPose& edgeBPoseStart,
    const RigidIpcPose& edgeBPoseEnd,
    const collision::native::CcdOption& option,
    collision::native::CcdPrimitiveResult& result);

/// Parameter-space subdivision query for rigid point-triangle time of impact.
[[nodiscard]] DART_SIMULATION_API bool rigidIpcPointTriangleIntervalCcd(
    const Eigen::Vector3d& point,
    const RigidIpcPose& pointPoseStart,
    const RigidIpcPose& pointPoseEnd,
    const Eigen::Vector3d& triangleA,
    const Eigen::Vector3d& triangleB,
    const Eigen::Vector3d& triangleC,
    const RigidIpcPose& trianglePoseStart,
    const RigidIpcPose& trianglePoseEnd,
    const collision::native::CcdOption& option,
    collision::native::CcdPrimitiveResult& result);

/// Continuous collision between a rigid-body point and a rigid-body triangle.
///
/// Unlike primitive endpoint-linear CCD, each local vertex follows the curved
/// trajectory induced by linearly interpolating its body's rotation vector.
[[nodiscard]] DART_SIMULATION_API bool rigidIpcPointTriangleCcd(
    const Eigen::Vector3d& point,
    const RigidIpcPose& pointPoseStart,
    const RigidIpcPose& pointPoseEnd,
    const Eigen::Vector3d& triangleA,
    const Eigen::Vector3d& triangleB,
    const Eigen::Vector3d& triangleC,
    const RigidIpcPose& trianglePoseStart,
    const RigidIpcPose& trianglePoseEnd,
    const collision::native::CcdOption& option,
    collision::native::CcdPrimitiveResult& result);

/// Continuous collision between two rigid-body points following curved rigid
/// trajectories over the unit step.
[[nodiscard]] DART_SIMULATION_API bool rigidIpcPointPointCcd(
    const Eigen::Vector3d& pointA,
    const RigidIpcPose& pointAPoseStart,
    const RigidIpcPose& pointAPoseEnd,
    const Eigen::Vector3d& pointB,
    const RigidIpcPose& pointBPoseStart,
    const RigidIpcPose& pointBPoseEnd,
    const collision::native::CcdOption& option,
    collision::native::CcdPrimitiveResult& result);

/// Continuous collision between a rigid-body point and a rigid-body edge
/// following curved rigid trajectories over the unit step.
[[nodiscard]] DART_SIMULATION_API bool rigidIpcPointEdgeCcd(
    const Eigen::Vector3d& point,
    const RigidIpcPose& pointPoseStart,
    const RigidIpcPose& pointPoseEnd,
    const Eigen::Vector3d& edgeA,
    const Eigen::Vector3d& edgeB,
    const RigidIpcPose& edgePoseStart,
    const RigidIpcPose& edgePoseEnd,
    const collision::native::CcdOption& option,
    collision::native::CcdPrimitiveResult& result);

/// Continuous collision between two rigid-body edges following curved rigid
/// trajectories over the unit step.
[[nodiscard]] DART_SIMULATION_API bool rigidIpcEdgeEdgeCcd(
    const Eigen::Vector3d& edgeA0,
    const Eigen::Vector3d& edgeA1,
    const RigidIpcPose& edgeAPoseStart,
    const RigidIpcPose& edgeAPoseEnd,
    const Eigen::Vector3d& edgeB0,
    const Eigen::Vector3d& edgeB1,
    const RigidIpcPose& edgeBPoseStart,
    const RigidIpcPose& edgeBPoseEnd,
    const collision::native::CcdOption& option,
    collision::native::CcdPrimitiveResult& result);

} // namespace dart::simulation::detail
