/*
 * Copyright (c) 2011-2023, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#ifndef DART_COLLISION_FCL_BACKWARDCOMPATIBILITY_HPP_
#define DART_COLLISION_FCL_BACKWARDCOMPATIBILITY_HPP_

#include <dart/config.hpp>

// clang-format off
#define FCL_VERSION_AT_LEAST(x,y,z)                                            \
  (FCL_MAJOR_VERSION > x || (FCL_MAJOR_VERSION >= x &&                         \
  (FCL_MINOR_VERSION > y || (FCL_MINOR_VERSION >= y &&                         \
  FCL_PATCH_VERSION >= z))))

#define FCL_MAJOR_MINOR_VERSION_AT_MOST(x,y)                                   \
  (FCL_MAJOR_VERSION < x || (FCL_MAJOR_VERSION <= x &&                         \
  (FCL_MINOR_VERSION < y || (FCL_MINOR_VERSION <= y))))
// clang-format on

#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/config.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/geometric_shape_to_BVH_model.h>
#include <fcl/math/bv/OBBRSS.h>
#include <fcl/math/bv/utility.h>
#include <fcl/math/geometry.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/distance.h>

#if DART_HAVE_OCTOMAP && FCL_DART_HAVE_OCTOMAP
  #include <fcl/geometry/octree/octree.h>
#endif // DART_HAVE_OCTOMAP && FCL_DART_HAVE_OCTOMAP

#include <memory>

namespace dart {
namespace collision {
namespace fcl {

// Geometric fundamentals
using Vector3 = ::fcl::Vector3<double>;
using Matrix3 = ::fcl::Matrix3<double>;
using Transform3 = ::fcl::Transform3<double>;
// Geometric primitives
using Box = ::fcl::Box<double>;
using Cylinder = ::fcl::Cylinder<double>;
using Cone = ::fcl::Cone<double>;
using Ellipsoid = ::fcl::Ellipsoid<double>;
using Halfspace = ::fcl::Halfspace<double>;
using Sphere = ::fcl::Sphere<double>;
#if DART_HAVE_OCTOMAP && FCL_DART_HAVE_OCTOMAP
using OcTree = ::fcl::OcTree<double>;
#endif // DART_HAVE_OCTOMAP && FCL_DART_HAVE_OCTOMAP
// Collision objects
using CollisionObject = ::fcl::CollisionObject<double>;
using CollisionGeometry = ::fcl::CollisionGeometry<double>;
using DynamicAABBTreeCollisionManager
    = ::fcl::DynamicAABBTreeCollisionManager<double>;
using OBBRSS = ::fcl::OBBRSS<double>;
using CollisionRequest = ::fcl::CollisionRequest<double>;
using CollisionResult = ::fcl::CollisionResult<double>;
using DistanceRequest = ::fcl::DistanceRequest<double>;
using DistanceResult = ::fcl::DistanceResult<double>;
using Contact = ::fcl::Contact<double>;

/// Sets a rotation matrix given Euler-XYZ angles
void setEulerZYX(
    dart::collision::fcl::Matrix3& rot,
    double eulerX,
    double eulerY,
    double eulerZ);

} // namespace fcl
} // namespace collision
} // namespace dart

#endif // DART_COLLISION_FCL_BACKWARDCOMPATIBILITY_HPP_
