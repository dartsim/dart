/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#pragma once

#include "dart/config.hpp"

#include <Eigen/Dense>

// clang-format off
#define FCL_VERSION_AT_LEAST(x,y,z)                                            \
  (FCL_MAJOR_VERSION > x || (FCL_MAJOR_VERSION >= x &&                         \
  (FCL_MINOR_VERSION > y || (FCL_MINOR_VERSION >= y &&                         \
  FCL_PATCH_VERSION >= z))))

#define FCL_MAJOR_MINOR_VERSION_AT_MOST(x,y)                                   \
  (FCL_MAJOR_VERSION < x || (FCL_MAJOR_VERSION <= x &&                         \
  (FCL_MINOR_VERSION < y || (FCL_MINOR_VERSION <= y))))
// clang-format on

#include <fcl/config.h>

#if FCL_VERSION_AT_LEAST(0, 6, 0)

#  include <fcl/math/geometry.h>

#  include <fcl/geometry/bvh/BVH_model.h>
#  include <fcl/geometry/geometric_shape_to_BVH_model.h>
#  include <fcl/math/bv/OBBRSS.h>
#  include <fcl/math/bv/utility.h>
#  include <fcl/narrowphase/collision.h>
#  include <fcl/narrowphase/collision_object.h>
#  include <fcl/narrowphase/distance.h>

#else

#  include <fcl/math/matrix_3f.h>
#  include <fcl/math/transform.h>
#  include <fcl/math/vec_3f.h>

#  include <fcl/BV/OBBRSS.h>
#  include <fcl/BVH/BVH_model.h>
#  include <fcl/shape/geometric_shape_to_BVH_model.h>
#  include <fcl/collision.h>
#  include <fcl/collision_data.h>
#  include <fcl/collision_object.h>
#  include <fcl/distance.h>

#endif // FCL_VERSION_AT_LEAST(0,6,0)

#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>

#if DART_HAVE_OCTOMAP && FCL_HAVE_OCTOMAP
#  if FCL_VERSION_AT_LEAST(0, 6, 0)
#    include <fcl/geometry/octree/octree.h>
#  else
#    include <fcl/octree.h>
#  endif // FCL_VERSION_AT_LEAST(0,6,0)
#endif   // DART_HAVE_OCTOMAP && FCL_HAVE_OCTOMAP

namespace dart {
namespace collision2 {

#if FCL_VERSION_AT_LEAST(0, 6, 0)
// Geometric fundamentals
template <typename S>
using FclVector3 = ::fcl::Vector3<S>;
template <typename S>
using FclMatrix3 = ::fcl::Matrix3<S>;
template <typename S>
using FclTransform3 = ::fcl::Transform3<S>;
// Geometric primitives
template <typename S>
using FclBox = ::fcl::Box<S>;
template <typename S>
using FclCylinder = ::fcl::Cylinder<S>;
template <typename S>
using FclCone = ::fcl::Cone<S>;
template <typename S>
using FclEllipsoid = ::fcl::Ellipsoid<S>;
template <typename S>
using FclHalfspace = ::fcl::Halfspace<S>;
template <typename S>
using FclSphere = ::fcl::Sphere<S>;
#  if DART_HAVE_OCTOMAP && FCL_HAVE_OCTOMAP
template <typename S>
using FclOcTree = ::fcl::OcTree<S>;
#  endif // DART_HAVE_OCTOMAP && FCL_HAVE_OCTOMAP
// Collision objects
template <typename S>
using FclCollisionObject = ::fcl::CollisionObject<S>;
template <typename S>
using FclCollisionGeometry = ::fcl::CollisionGeometry<S>;
template <typename S>
using FclDynamicAABBTreeCollisionManager
    = ::fcl::DynamicAABBTreeCollisionManager<S>;
template <typename S>
using FclOBBRSS = ::fcl::OBBRSS<S>;
template <typename S>
using FclCollisionRequest = ::fcl::CollisionRequest<S>;
template <typename S>
using FclCollisionResult = ::fcl::CollisionResult<S>;
template <typename S>
using FclDistanceRequest = ::fcl::DistanceRequest<S>;
template <typename S>
using FclDistanceResult = ::fcl::DistanceResult<S>;
template <typename S>
using FclContact = ::fcl::Contact<S>;
#else
// Geometric fundamentals
template <typename S>
using FclVector3 = ::fcl::Vec3f;
template <typename S>
using FclMatrix3 = ::fcl::Matrix3f;
template <typename S>
using FclTransform3 = ::fcl::Transform3f;
// Geometric primitives
template <typename S>
using FclBox = ::fcl::Box;
template <typename S>
using FclCylinder = ::fcl::Cylinder;
template <typename S>
using FclCone = ::fcl::Cone;
template <typename S>
using FclEllipsoid = ::fcl::Ellipsoid;
template <typename S>
using FclHalfspace = ::fcl::Halfspace;
template <typename S>
using FclSphere = ::fcl::Sphere;
#  if DART_HAVE_OCTOMAP && FCL_HAVE_OCTOMAP
template <typename S>
using FclOcTree = ::fcl::OcTree;
#  endif // DART_HAVE_OCTOMAP && FCL_HAVE_OCTOMAP
// Collision objects
template <typename S>
using FclCollisionObject = ::fcl::CollisionObject;
template <typename S>
using FclCollisionGeometry = ::fcl::CollisionGeometry;
template <typename S>
using FclDynamicAABBTreeCollisionManager
    = ::fcl::DynamicAABBTreeCollisionManager;
template <typename S>
using FclOBBRSS = ::fcl::OBBRSS;
template <typename S>
using FclCollisionRequest = ::fcl::CollisionRequest;
template <typename S>
using FclCollisionResult = ::fcl::CollisionResult;
template <typename S>
using FclDistanceRequest = ::fcl::DistanceRequest;
template <typename S>
using FclDistanceResult = ::fcl::DistanceResult;
template <typename S>
using FclContact = ::fcl::Contact;
#endif

/// Returns norm of a 3-dim vector
template <typename S>
S length(const FclVector3<S>& t);

/// Returns squared norm of a 3-dim vector
template <typename S>
S length2(const FclVector3<S>& t);

/// Returns translation component of a transform
template <typename S>
FclVector3<S> getTranslation(const FclTransform3<S>& T);

/// Sets translation component of a transform
template <typename S>
void setTranslation(FclTransform3<S>& T, const FclVector3<S>& t);

/// Returns rotation component of a transform
template <typename S>
FclMatrix3<S> getRotation(const FclTransform3<S>& T);

/// Sets rotation component of a transform
template <typename S>
void setRotation(FclTransform3<S>& T, const FclMatrix3<S>& R);

/// Sets a rotation matrix given Euler-XYZ angles
template <typename S>
void setEulerZYX(FclMatrix3<S>& rot, S eulerX, S eulerY, S eulerZ);

/// Transforms a 3-dim vector by a transform and returns the result
template <typename S>
FclVector3<S> transform(const FclTransform3<S>& t, const FclVector3<S>& v);

} // namespace collision2
} // namespace dart

#include "dart/collision/fcl/detail/backward_compatibility_impl.hpp"
