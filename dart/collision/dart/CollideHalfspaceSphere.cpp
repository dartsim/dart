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

#include "dart/collision/dart/CollideHalfspaceSphere.hpp"

namespace dart {
namespace collision {

//==============================================================================
//template <>
//inline int collideHalfspaceSphere<HalfspaceSpherePolicy::OnPlane>(
//    const Halfspace& halfspaceA,
//    const Eigen::Isometry3d& tfA,
//    const dynamics::SphereShape& sphereB,
//    const Eigen::Isometry3d& tfB,
//    NarrowPhaseCallback* callback)
//{
//  const Eigen::Vector3d& halfspaceNormal = tfA.linear() * halfspaceA.getNormal();
//  const auto halfspaceOffset =
//      halfspaceNormal.dot(tfA.translation()) + halfspaceA.getOffset();

//  const auto sphereRadius = sphereB.getRadius();
//  const Eigen::Vector3d& sphereCenter = tfB.translation();

//  const auto d = halfspaceOffset - halfspaceNormal.dot(sphereCenter);
//  const auto depth = sphereRadius + d;

//  if (depth < 0.0)
//    return 0;

//  if (callback)
//  {
//    // Contact point on the plane
//    const Eigen::Vector3d point = sphereCenter - d * halfspaceNormal;

//    callback->notifyContact(
//        &halfspaceA, &sphereB, point, halfspaceNormal, depth);
//  }

//  return 1;
//}

//==============================================================================
//template <>
//inline int collideHalfspaceSphere<HalfspaceSpherePolicy::OnSphere>(
//    const Halfspace& halfspaceA,
//    const Eigen::Isometry3d& tfA,
//    const dynamics::SphereShape& sphereB,
//    const Eigen::Isometry3d& tfB,
//    NarrowPhaseCallback* callback)
//{
//  const Eigen::Vector3d& halfspaceNormal = tfA.linear() * halfspaceA.getNormal();
//  const auto halfspaceOffset =
//      halfspaceNormal.dot(tfA.translation()) + halfspaceA.getOffset();

//  const auto sphereRadius = sphereB.getRadius();
//  const Eigen::Vector3d& sphereCenter = tfB.translation();

//  const auto depth =
//      sphereRadius + halfspaceOffset - halfspaceNormal.dot(sphereCenter);

//  if (depth < 0.0)
//    return 0;

//  if (callback)
//  {
//    // Contact point on the sphere
//    const Eigen::Vector3d point = sphereCenter - sphereRadius * halfspaceNormal;

//    callback->notifyContact(
//        &halfspaceA, &sphereB, point, halfspaceNormal, depth);
//  }

//  return 1;
//}

//==============================================================================
//template <>
//inline int collideHalfspaceSphere<HalfspaceSpherePolicy::MiddleOfIntersect>(
//    const Halfspace& halfspaceA,
//    const Eigen::Isometry3d& tfA,
//    const dynamics::SphereShape& sphereB,
//    const Eigen::Isometry3d& tfB,
//    NarrowPhaseCallback* callback)
//{
//  const Eigen::Vector3d& halfspaceNormal = tfA.linear() * halfspaceA.getNormal();
//  const auto halfspaceOffset =
//      halfspaceNormal.dot(tfA.translation()) + halfspaceA.getOffset();

//  const auto sphereRadius = sphereB.getRadius();
//  const Eigen::Vector3d& sphereCenter = tfB.translation();

//  const auto depth =
//      sphereRadius + halfspaceOffset - halfspaceNormal.dot(sphereCenter);

//  if (depth < 0.0)
//    return 0;

//  if (callback)
//  {
//    // Contact point on the sphere
//    const Eigen::Vector3d point =
//        sphereCenter - (sphereRadius - 0.5 * depth) * halfspaceNormal;

//    callback->notifyContact(
//        &halfspaceA, &sphereB, point, halfspaceNormal, depth);
//  }

//  return 1;
//}

} // namespace collision
} // namespace dart
