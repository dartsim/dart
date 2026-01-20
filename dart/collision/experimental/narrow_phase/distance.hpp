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
#include <dart/collision/experimental/types.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace dart::collision::experimental {

class BoxShape;
class CapsuleShape;
class CylinderShape;
class PlaneShape;
class SphereShape;
class SdfShape;

[[nodiscard]] DART_COLLISION_EXPERIMENTAL_API double distanceSphereSphere(
    const SphereShape& sphere1,
    const Eigen::Isometry3d& transform1,
    const SphereShape& sphere2,
    const Eigen::Isometry3d& transform2,
    DistanceResult& result,
    const DistanceOption& option = DistanceOption());

[[nodiscard]] DART_COLLISION_EXPERIMENTAL_API double distanceSphereBox(
    const SphereShape& sphere,
    const Eigen::Isometry3d& sphereTransform,
    const BoxShape& box,
    const Eigen::Isometry3d& boxTransform,
    DistanceResult& result,
    const DistanceOption& option = DistanceOption());

[[nodiscard]] DART_COLLISION_EXPERIMENTAL_API double distanceBoxBox(
    const BoxShape& box1,
    const Eigen::Isometry3d& transform1,
    const BoxShape& box2,
    const Eigen::Isometry3d& transform2,
    DistanceResult& result,
    const DistanceOption& option = DistanceOption());

[[nodiscard]] DART_COLLISION_EXPERIMENTAL_API double distanceCapsuleCapsule(
    const CapsuleShape& capsule1,
    const Eigen::Isometry3d& transform1,
    const CapsuleShape& capsule2,
    const Eigen::Isometry3d& transform2,
    DistanceResult& result,
    const DistanceOption& option = DistanceOption());

[[nodiscard]] DART_COLLISION_EXPERIMENTAL_API double distanceCapsuleSphere(
    const CapsuleShape& capsule,
    const Eigen::Isometry3d& capsuleTransform,
    const SphereShape& sphere,
    const Eigen::Isometry3d& sphereTransform,
    DistanceResult& result,
    const DistanceOption& option = DistanceOption());

[[nodiscard]] DART_COLLISION_EXPERIMENTAL_API double distanceCapsuleBox(
    const CapsuleShape& capsule,
    const Eigen::Isometry3d& capsuleTransform,
    const BoxShape& box,
    const Eigen::Isometry3d& boxTransform,
    DistanceResult& result,
    const DistanceOption& option = DistanceOption());

[[nodiscard]] DART_COLLISION_EXPERIMENTAL_API double distanceSphereSdf(
    const SphereShape& sphere,
    const Eigen::Isometry3d& sphereTransform,
    const SdfShape& sdf,
    const Eigen::Isometry3d& sdfTransform,
    DistanceResult& result,
    const DistanceOption& option = DistanceOption());

[[nodiscard]] DART_COLLISION_EXPERIMENTAL_API double distanceCapsuleSdf(
    const CapsuleShape& capsule,
    const Eigen::Isometry3d& capsuleTransform,
    const SdfShape& sdf,
    const Eigen::Isometry3d& sdfTransform,
    DistanceResult& result,
    const DistanceOption& option = DistanceOption());

}
