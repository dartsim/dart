/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include "dart/common/eigen_include.hpp"

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

  #include <fcl/math/geometry.h>

  #include <fcl/geometry/bvh/BVH_model.h>
  #include <fcl/geometry/geometric_shape_to_BVH_model.h>
  #include <fcl/math/bv/OBBRSS.h>
  #include <fcl/math/bv/utility.h>
  #include <fcl/narrowphase/collision.h>
  #include <fcl/narrowphase/collision_object.h>
  #include <fcl/narrowphase/distance.h>

#else

  #include <fcl/math/matrix_3f.h>
  #include <fcl/math/transform.h>
  #include <fcl/math/vec_3f.h>

  #include <fcl/BV/OBBRSS.h>
  #include <fcl/BVH/BVH_model.h>
  #include <fcl/shape/geometric_shape_to_BVH_model.h>
  #include <fcl/collision.h>
  #include <fcl/collision_data.h>
  #include <fcl/collision_object.h>
  #include <fcl/distance.h>

#endif // FCL_VERSION_AT_LEAST(0,6,0)

#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>

#if DART_HAVE_OCTOMAP && FCL_HAVE_OCTOMAP
  #if FCL_VERSION_AT_LEAST(0, 6, 0)
    #include <fcl/geometry/octree/octree.h>
  #else
    #include <fcl/octree.h>
  #endif // FCL_VERSION_AT_LEAST(0,6,0)
#endif   // DART_HAVE_OCTOMAP && FCL_HAVE_OCTOMAP

namespace dart {
namespace collision {

#if FCL_VERSION_AT_LEAST(0, 6, 0)
// Geometric fundamentals
template <typename Scalar>
using FclVector3 = ::fcl::Vector3<Scalar>;
template <typename Scalar>
using FclMatrix3 = ::fcl::Matrix3<Scalar>;
template <typename Scalar>
using FclTransform3 = ::fcl::Transform3<Scalar>;
// Geometric primitives
template <typename Scalar>
using FclBox = ::fcl::Box<Scalar>;
template <typename Scalar>
using FclCylinder = ::fcl::Cylinder<Scalar>;
template <typename Scalar>
using FclCone = ::fcl::Cone<Scalar>;
template <typename Scalar>
using FclEllipsoid = ::fcl::Ellipsoid<Scalar>;
template <typename Scalar>
using FclHalfspace = ::fcl::Halfspace<Scalar>;
template <typename Scalar>
using FclSphere = ::fcl::Sphere<Scalar>;
  #if DART_HAVE_OCTOMAP && FCL_HAVE_OCTOMAP
template <typename Scalar>
using FclOcTree = ::fcl::OcTree<Scalar>;
  #endif // DART_HAVE_OCTOMAP && FCL_HAVE_OCTOMAP
// Collision objects
template <typename Scalar>
using FclCollisionObject = ::fcl::CollisionObject<Scalar>;
template <typename Scalar>
using FclCollisionGeometry = ::fcl::CollisionGeometry<Scalar>;
template <typename Scalar>
using FclDynamicAABBTreeCollisionManager
    = ::fcl::DynamicAABBTreeCollisionManager<Scalar>;
template <typename Scalar>
using FclOBBRSS = ::fcl::OBBRSS<Scalar>;
template <typename Scalar>
using FclCollisionRequest = ::fcl::CollisionRequest<Scalar>;
template <typename Scalar>
using FclCollisionResult = ::fcl::CollisionResult<Scalar>;
template <typename Scalar>
using FclDistanceRequest = ::fcl::DistanceRequest<Scalar>;
template <typename Scalar>
using FclDistanceResult = ::fcl::DistanceResult<Scalar>;
template <typename Scalar>
using FclContact = ::fcl::Contact<Scalar>;
#else
// Geometric fundamentals
template <typename Scalar>
using FclVector3 = ::fcl::Vec3f;
template <typename Scalar>
using FclMatrix3 = ::fcl::Matrix3f;
template <typename Scalar>
using FclTransform3 = ::fcl::Transform3f;
// Geometric primitives
template <typename Scalar>
using FclBox = ::fcl::Box;
template <typename Scalar>
using FclCylinder = ::fcl::Cylinder;
template <typename Scalar>
using FclCone = ::fcl::Cone;
template <typename Scalar>
using FclEllipsoid = ::fcl::Ellipsoid;
template <typename Scalar>
using FclHalfspace = ::fcl::Halfspace;
template <typename Scalar>
using FclSphere = ::fcl::Sphere;
  #if DART_HAVE_OCTOMAP && FCL_HAVE_OCTOMAP
template <typename Scalar>
using FclOcTree = ::fcl::OcTree;
  #endif // DART_HAVE_OCTOMAP && FCL_HAVE_OCTOMAP
// Collision objects
template <typename Scalar>
using FclCollisionObject = ::fcl::CollisionObject;
template <typename Scalar>
using FclCollisionGeometry = ::fcl::CollisionGeometry;
template <typename Scalar>
using FclDynamicAABBTreeCollisionManager
    = ::fcl::DynamicAABBTreeCollisionManager;
template <typename Scalar>
using FclOBBRSS = ::fcl::OBBRSS;
template <typename Scalar>
using FclCollisionRequest = ::fcl::CollisionRequest;
template <typename Scalar>
using FclCollisionResult = ::fcl::CollisionResult;
template <typename Scalar>
using FclDistanceRequest = ::fcl::DistanceRequest;
template <typename Scalar>
using FclDistanceResult = ::fcl::DistanceResult;
template <typename Scalar>
using FclContact = ::fcl::Contact;
#endif

/// Returns norm of a 3-dim vector
template <typename Scalar>
Scalar length(const FclVector3<Scalar>& t);

/// Returns squared norm of a 3-dim vector
template <typename Scalar>
Scalar length2(const FclVector3<Scalar>& t);

/// Returns translation component of a transform
template <typename Scalar>
FclVector3<Scalar> getTranslation(const FclTransform3<Scalar>& T);

/// Sets translation component of a transform
template <typename Scalar>
void setTranslation(FclTransform3<Scalar>& T, const FclVector3<Scalar>& t);

/// Returns rotation component of a transform
template <typename Scalar>
FclMatrix3<Scalar> getRotation(const FclTransform3<Scalar>& T);

/// Sets rotation component of a transform
template <typename Scalar>
void setRotation(FclTransform3<Scalar>& T, const FclMatrix3<Scalar>& R);

/// Sets a rotation matrix given Euler-XYZ angles
template <typename Scalar>
void setEulerZYX(
    FclMatrix3<Scalar>& rot, Scalar eulerX, Scalar eulerY, Scalar eulerZ);

/// Transforms a 3-dim vector by a transform and returns the result
template <typename Scalar>
FclVector3<Scalar> transform(
    const FclTransform3<Scalar>& t, const FclVector3<Scalar>& v);

/// Returns an identity transform
template <typename Scalar>
FclTransform3<Scalar> get_identity_transform();

} // namespace collision
} // namespace dart

#include "dart/collision/fcl/detail/backward_compatibility_impl.hpp"
