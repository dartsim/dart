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

#include <dart/collision/experimental/export.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>
#include <dart/collision/experimental/types.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace dart::collision::experimental {

[[nodiscard]] DART_COLLISION_EXPERIMENTAL_API bool sphereCastSphere(
    const Eigen::Vector3d& sphereStart,
    const Eigen::Vector3d& sphereEnd,
    double sphereRadius,
    const SphereShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_EXPERIMENTAL_API bool sphereCastBox(
    const Eigen::Vector3d& sphereStart,
    const Eigen::Vector3d& sphereEnd,
    double sphereRadius,
    const BoxShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_EXPERIMENTAL_API bool sphereCastCapsule(
    const Eigen::Vector3d& sphereStart,
    const Eigen::Vector3d& sphereEnd,
    double sphereRadius,
    const CapsuleShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_EXPERIMENTAL_API bool sphereCastPlane(
    const Eigen::Vector3d& sphereStart,
    const Eigen::Vector3d& sphereEnd,
    double sphereRadius,
    const PlaneShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_EXPERIMENTAL_API bool sphereCastCylinder(
    const Eigen::Vector3d& sphereStart,
    const Eigen::Vector3d& sphereEnd,
    double sphereRadius,
    const CylinderShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_EXPERIMENTAL_API bool sphereCastConvex(
    const Eigen::Vector3d& sphereStart,
    const Eigen::Vector3d& sphereEnd,
    double sphereRadius,
    const ConvexShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_EXPERIMENTAL_API bool sphereCastMesh(
    const Eigen::Vector3d& sphereStart,
    const Eigen::Vector3d& sphereEnd,
    double sphereRadius,
    const MeshShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_EXPERIMENTAL_API bool capsuleCastSphere(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const SphereShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_EXPERIMENTAL_API bool capsuleCastBox(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const BoxShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_EXPERIMENTAL_API bool capsuleCastCapsule(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const CapsuleShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_EXPERIMENTAL_API bool capsuleCastPlane(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const PlaneShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_EXPERIMENTAL_API bool capsuleCastCylinder(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const CylinderShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_EXPERIMENTAL_API bool capsuleCastConvex(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const ConvexShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_EXPERIMENTAL_API bool capsuleCastMesh(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const MeshShape& target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result);

[[nodiscard]] DART_COLLISION_EXPERIMENTAL_API bool conservativeAdvancement(
    const ConvexShape& shapeA,
    const Eigen::Isometry3d& transformAStart,
    const Eigen::Isometry3d& transformAEnd,
    const ConvexShape& shapeB,
    const Eigen::Isometry3d& transformB,
    const CcdOption& option,
    CcdResult& result);

} // namespace dart::collision::experimental
