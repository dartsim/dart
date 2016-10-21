/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "dart/collision/dart/CollideSphereSphere.hpp"

namespace dart {
namespace collision {

//==============================================================================
template <>
void collideSphereSphere<SphereSpherePolicy::PointOfInternalDivision>(
    const dynamics::SphereShape& sphereA,
    const Eigen::Isometry3d& tfA,
    const dynamics::SphereShape& sphereB,
    const Eigen::Isometry3d& tfB,
    NarrowPhaseCallback* callback)
{
  const Eigen::Vector3d& centerA = tfA.translation();
  const Eigen::Vector3d& centerB = tfB.translation();

  const auto& radiusA = sphereA.getRadius();
  const auto& radiusB = sphereB.getRadius();

  const auto radiusSum = radiusA + radiusB;
  const Eigen::Vector3d centerDiff = centerB - centerA;
  auto centerDiffNorm = centerDiff.norm();

  if (centerDiffNorm > radiusSum)
    return;

  if (callback)
  {
    // If the centers of two sphere are at the same position, the normal is
    // (0, 0, 0). Otherwise, normal is pointing from center of object 1 to
    // center of object 2.
    const Eigen::Vector3d normal = centerDiffNorm > 0.0
                               ? (centerDiff / centerDiffNorm).eval()
                               : centerDiff;
    const Eigen::Vector3d point = centerA + centerDiff * (radiusA / radiusSum);
    const auto depth = radiusSum - centerDiffNorm;

    callback->notifyContact(&sphereA, &sphereB, point, normal, depth);
  }
}

//==============================================================================
template <>
void collideSphereSphere<SphereSpherePolicy::MiddleOfIntersect>(
    const dynamics::SphereShape& sphereA,
    const Eigen::Isometry3d& tfA,
    const dynamics::SphereShape& sphereB,
    const Eigen::Isometry3d& tfB,
    NarrowPhaseCallback* callback)
{
  const Eigen::Vector3d& centerA = tfA.translation();
  const Eigen::Vector3d& centerB = tfB.translation();

  const auto& radiusA = sphereA.getRadius();
  const auto& radiusB = sphereB.getRadius();

  const auto radiusSum = radiusA + radiusB;
  const Eigen::Vector3d centerDiff = centerB - centerA;
  auto centerDiffNorm = centerDiff.norm();

  if (centerDiffNorm > radiusSum)
    return;

  if (callback)
  {
    // If the centers of two sphere are at the same position, the normal is
    // (0, 0, 0). Otherwise, normal is pointing from center of object 1 to
    // center of object 2.
    const Eigen::Vector3d normal = centerDiffNorm > 0.0
                               ? (centerDiff / centerDiffNorm).eval()
                               : centerDiff;
    const auto depth = radiusSum - centerDiffNorm;
    const Eigen::Vector3d point = centerA + normal * (radiusA - 0.5 * depth);

    callback->notifyContact(&sphereA, &sphereB, point, normal, depth);
  }
}

} // namespace collision
} // namespace dart
