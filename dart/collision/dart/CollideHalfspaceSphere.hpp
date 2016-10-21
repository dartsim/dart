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

#ifndef DART_COLLISION_DART_COLLIDEHALFSPACESPHERE_HPP_
#define DART_COLLISION_DART_COLLIDEHALFSPACESPHERE_HPP_

#include "dart/dynamics/PlaneShape.hpp"
#include "dart/collision/dart/NarrowPhaseAlgorithms.hpp"

namespace dart {
namespace collision {

/// Halfspace-sphere contact point policy
enum class HalfspaceSpherePolicy
{
  OnPlane,          ///< The contact point is on the plane.
  OnSphere,         ///< The contact point is on the sphere.
  MiddleOfIntersect ///< The contact point is at the middle of plane and sphere
};

using SphereHalfspacePolicy = HalfspaceSpherePolicy;

/// Collide sphere and Halfspace. The normal points from Halfspace to sphere.
//template <
//    HalfspaceSpherePolicy ContactPointPolicy = HalfspaceSpherePolicy::OnPlane>
//int collideHalfspaceSphere(const Halfspace& halfspaceA,
//                           const Eigen::Isometry3d& tfA,
//                           const dynamics::SphereShape& sphereB,
//                           const Eigen::Isometry3d& tfB,
//                           NarrowPhaseCallback* callback);

//template <
//    HalfspaceSpherePolicy ContactPointPolicy = HalfspaceSpherePolicy::OnPlane>
//int collideSphereHalfspace(const dynamics::SphereShape& sphereA,
//                           const Eigen::Isometry3d& tfA,
//                           const Halfspace& halfspaceB,
//                           const Eigen::Isometry3d& tfB,
//                           NarrowPhaseCallback* callback)
//{
//  ReverseReporter reverseCallback(*callback);

//  return collideHalfspaceSphere<ContactPointPolicy>(
//      halfspaceB, tfB, sphereA, tfA, &reverseCallback); // order switched
//}
// TODO(JS): move to detail namespace

/// Collide sphere and Halfspace.
//template <>
//int collideHalfspaceSphere<HalfspaceSpherePolicy::OnPlane>(
//    const Halfspace& halfspaceA,
//    const Eigen::Isometry3d& tfA,
//    const dynamics::SphereShape& sphereB,
//    const Eigen::Isometry3d& tfB,
//    NarrowPhaseCallback* callback);

//template <>
//int collideHalfspaceSphere<HalfspaceSpherePolicy::OnSphere>(
//    const Halfspace& halfspaceA,
//    const Eigen::Isometry3d& tfA,
//    const dynamics::SphereShape& sphereB,
//    const Eigen::Isometry3d& tfB,
//    NarrowPhaseCallback* callback);

//template <>
//int collideHalfspaceSphere<HalfspaceSpherePolicy::MiddleOfIntersect>(
//    const Halfspace& halfspaceA,
//    const Eigen::Isometry3d& tfA,
//    const dynamics::SphereShape& sphereB,
//    const Eigen::Isometry3d& tfB,
//    NarrowPhaseCallback* callback);

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_DART_COLLIDEHALFSPACESPHERE_HPP_
