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

#include <dart/collision/native/Export.hpp>
#include <dart/collision/native/Types.hpp>
#include <dart/collision/native/shapes/Shape.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>

namespace dart::collision::native {

[[nodiscard]] DART_COLLISION_NATIVE_API bool sphereCastSphere(
    const Eigen::Vector3d& sphereStart,
    const Eigen::Vector3d& sphereEnd,
    double sphereRadius,
    const SphereShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_NATIVE_API bool sphereCastBox(
    const Eigen::Vector3d& sphereStart,
    const Eigen::Vector3d& sphereEnd,
    double sphereRadius,
    const BoxShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_NATIVE_API bool sphereCastCapsule(
    const Eigen::Vector3d& sphereStart,
    const Eigen::Vector3d& sphereEnd,
    double sphereRadius,
    const CapsuleShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_NATIVE_API bool sphereCastPlane(
    const Eigen::Vector3d& sphereStart,
    const Eigen::Vector3d& sphereEnd,
    double sphereRadius,
    const PlaneShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_NATIVE_API bool sphereCastCylinder(
    const Eigen::Vector3d& sphereStart,
    const Eigen::Vector3d& sphereEnd,
    double sphereRadius,
    const CylinderShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_NATIVE_API bool sphereCastConvex(
    const Eigen::Vector3d& sphereStart,
    const Eigen::Vector3d& sphereEnd,
    double sphereRadius,
    const ConvexShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_NATIVE_API bool sphereCastMesh(
    const Eigen::Vector3d& sphereStart,
    const Eigen::Vector3d& sphereEnd,
    double sphereRadius,
    const MeshShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_NATIVE_API bool capsuleCastSphere(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const SphereShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_NATIVE_API bool capsuleCastBox(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const BoxShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_NATIVE_API bool capsuleCastCapsule(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const CapsuleShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_NATIVE_API bool capsuleCastPlane(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const PlaneShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_NATIVE_API bool capsuleCastCylinder(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const CylinderShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_NATIVE_API bool capsuleCastConvex(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const ConvexShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_NATIVE_API bool capsuleCastMesh(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const MeshShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_NATIVE_API bool conservativeAdvancement(
    const ConvexShape& shapeA,
    const Eigen::Isometry3d& transformAStart,
    const Eigen::Isometry3d& transformAEnd,
    const ConvexShape& shapeB,
    const Eigen::Isometry3d& transformB,
    const CcdOption& option,
    CcdResult& result);

/// Conservative advancement when BOTH convex shapes move over the step.
///
/// Generalizes conservativeAdvancement (which holds shape B fixed) to the case
/// where shape B also translates/rotates, matching reference-engine continuous
/// collision where each body carries its own motion. The closest distance can
/// change no faster than the sum of the two per-shape motion bounds, so
/// advancing by distance / (boundA + boundB) is conservative.
[[nodiscard]] DART_COLLISION_NATIVE_API bool convexCast(
    const ConvexShape& shapeA,
    const Eigen::Isometry3d& transformAStart,
    const Eigen::Isometry3d& transformAEnd,
    const ConvexShape& shapeB,
    const Eigen::Isometry3d& transformBStart,
    const Eigen::Isometry3d& transformBEnd,
    const CcdOption& option,
    CcdResult& result);

/// Conservative advancement for shape A following a cubic polynomial (spline)
/// trajectory against a static convex shape B.
///
/// The trajectory is a cubic Bezier in both translation and rotation vector,
/// matching the spline-motion parameterization of the reference engine that
/// supports it: translation `T(t) = sum_i B_i(t) translationControlPoints[i]`
/// and orientation `R(t) = exp([sum_i B_i(t) rotationControlPoints[i]])` with
/// `B_i` the cubic Bernstein basis. A spline path is the one motion model the
/// linear/screw casts cannot represent: it can curve away from the chord
/// between its endpoints, so a straight-line or screw cast over the same
/// endpoints can miss a collision the spline actually makes. Pass four zero
/// vectors for `rotationControlPoints` to sweep a curved translational path
/// with no spin.
///
/// With `CcdAdvancement::Conservative` (the default) the step is
/// acceleration-bounded -- the gap closes by at most `(v0 + angular)*dt +
/// accel*dt^2/2`, whose root gives the largest provably safe step. It never
/// overshoots a true contact (the SO(3) exponential's right Jacobian is a
/// contraction, so `|omega| <= |W'|`), which is the no-tunnelling guarantee
/// barrier/IPC solvers need. With `CcdAdvancement::Fast` the step instead
/// divides the gap by the maximum forward displacement over the rest of the
/// motion: far fewer iterations, but it may stride past a sharply curved first
/// contact and report a later one -- for rigid-body use where speed matters
/// more than first-contact precision.
[[nodiscard]] DART_COLLISION_NATIVE_API bool splineCast(
    const ConvexShape& shapeA,
    const std::array<Eigen::Vector3d, 4>& translationControlPoints,
    const std::array<Eigen::Vector3d, 4>& rotationControlPoints,
    const ConvexShape& shapeB,
    const Eigen::Isometry3d& transformB,
    const CcdOption& option,
    CcdResult& result);

} // namespace dart::collision::native
