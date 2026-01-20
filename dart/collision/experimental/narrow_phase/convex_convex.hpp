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
#include <dart/collision/experimental/narrow_phase/gjk.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>
#include <dart/collision/experimental/types.hpp>

#include <Eigen/Geometry>

namespace dart::collision::experimental {

DART_COLLISION_EXPERIMENTAL_API SupportFunction makeConvexSupportFunction(
    const ConvexShape& shape, const Eigen::Isometry3d& transform);

DART_COLLISION_EXPERIMENTAL_API SupportFunction makeMeshSupportFunction(
    const MeshShape& shape, const Eigen::Isometry3d& transform);

DART_COLLISION_EXPERIMENTAL_API SupportFunction makeSphereSupportFunction(
    const SphereShape& shape, const Eigen::Isometry3d& transform);

DART_COLLISION_EXPERIMENTAL_API SupportFunction makeBoxSupportFunction(
    const BoxShape& shape, const Eigen::Isometry3d& transform);

DART_COLLISION_EXPERIMENTAL_API SupportFunction makeCapsuleSupportFunction(
    const CapsuleShape& shape, const Eigen::Isometry3d& transform);

DART_COLLISION_EXPERIMENTAL_API SupportFunction makeCylinderSupportFunction(
    const CylinderShape& shape, const Eigen::Isometry3d& transform);

DART_COLLISION_EXPERIMENTAL_API bool collideConvexConvex(
    const Shape& shape1,
    const Eigen::Isometry3d& tf1,
    const Shape& shape2,
    const Eigen::Isometry3d& tf2,
    CollisionResult& result,
    const CollisionOption& option);

DART_COLLISION_EXPERIMENTAL_API double distanceConvexConvex(
    const Shape& shape1,
    const Eigen::Isometry3d& tf1,
    const Shape& shape2,
    const Eigen::Isometry3d& tf2,
    DistanceResult& result,
    const DistanceOption& option);

} // namespace dart::collision::experimental
