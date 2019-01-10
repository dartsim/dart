/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#include "dart/config.hpp"

#include <Eigen/Dense>

#define FCL_VERSION_AT_LEAST(x,y,z) \
  (FCL_MAJOR_VERSION > x || (FCL_MAJOR_VERSION >= x && \
  (FCL_MINOR_VERSION > y || (FCL_MINOR_VERSION >= y && \
  FCL_PATCH_VERSION >= z))))

#define FCL_MAJOR_MINOR_VERSION_AT_MOST(x,y) \
  (FCL_MAJOR_VERSION < x || (FCL_MAJOR_VERSION <= x && \
  (FCL_MINOR_VERSION < y || (FCL_MINOR_VERSION <= y))))

#include <fcl/config.h>

#if FCL_VERSION_AT_LEAST(0,6,0)

#include <fcl/math/geometry.h>

#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/geometric_shape_to_BVH_model.h>
#include <fcl/math/bv/OBBRSS.h>
#include <fcl/math/bv/utility.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/distance.h>

#else

#include <fcl/math/vec_3f.h>
#include <fcl/math/matrix_3f.h>
#include <fcl/math/transform.h>

#include <fcl/BV/OBBRSS.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/collision.h>
#include <fcl/collision_data.h>
#include <fcl/collision_object.h>
#include <fcl/distance.h>

#endif // FCL_VERSION_AT_LEAST(0,6,0)

#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>

#if HAVE_OCTOMAP && defined(FCL_HAVE_OCTOMAP)
#if FCL_VERSION_AT_LEAST(0,6,0)
#include <fcl/geometry/octree/octree.h>
#else
#include <fcl/octree.h>
#endif // FCL_VERSION_AT_LEAST(0,6,0)
#endif // HAVE_OCTOMAP && defined(FCL_HAVE_OCTOMAP)

#if FCL_VERSION_AT_LEAST(0,5,0)
#include <memory>
template <class T> using fcl_shared_ptr = std::shared_ptr<T>;
template <class T> using fcl_weak_ptr = std::weak_ptr<T>;
template <class T, class... Args>
fcl_shared_ptr<T> fcl_make_shared(Args&&... args)
{
  return std::make_shared<T>(std::forward<Args>(args)...);
}
#else
#include <boost/weak_ptr.hpp>
#include <boost/make_shared.hpp>
template <class T> using fcl_shared_ptr = boost::shared_ptr<T>;
template <class T> using fcl_weak_ptr = boost::weak_ptr<T>;
template <class T, class... Args>
fcl_shared_ptr<T> fcl_make_shared(Args&&... args)
{
  return boost::make_shared<T>(std::forward<Args>(args)...);
}
#endif // FCL_VERSION_AT_LEAST(0,5,0)

namespace dart {
namespace collision {
namespace fcl {

#if FCL_VERSION_AT_LEAST(0,6,0)
// Geometric fundamentals
using Vector3 = ::fcl::Vector3<double>;
using Matrix3 = ::fcl::Matrix3<double>;
using Transform3 = ::fcl::Transform3<double>;
// Geometric primitives
using Box = ::fcl::Box<double>;
using Cylinder = ::fcl::Cylinder<double>;
using Ellipsoid = ::fcl::Ellipsoid<double>;
using Halfspace = ::fcl::Halfspace<double>;
using Sphere = ::fcl::Sphere<double>;
#if HAVE_OCTOMAP && defined(FCL_HAVE_OCTOMAP)
using OcTree = ::fcl::OcTree<double>;
#endif // HAVE_OCTOMAP && defined(FCL_HAVE_OCTOMAP)
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
#else
// Geometric fundamentals
using Vector3 = ::fcl::Vec3f;
using Matrix3 = ::fcl::Matrix3f;
using Transform3 = ::fcl::Transform3f;
// Geometric primitives
using Box = ::fcl::Box;
using Cylinder = ::fcl::Cylinder;
using Halfspace = ::fcl::Halfspace;
using Sphere = ::fcl::Sphere;
#if HAVE_OCTOMAP && defined(FCL_HAVE_OCTOMAP)
using OcTree = ::fcl::OcTree;
#endif // HAVE_OCTOMAP && defined(FCL_HAVE_OCTOMAP)
// Collision objects
using CollisionObject = ::fcl::CollisionObject;
using CollisionGeometry = ::fcl::CollisionGeometry;
using DynamicAABBTreeCollisionManager = ::fcl::DynamicAABBTreeCollisionManager;
using OBBRSS = ::fcl::OBBRSS;
using CollisionRequest = ::fcl::CollisionRequest;
using CollisionResult = ::fcl::CollisionResult;
using DistanceRequest = ::fcl::DistanceRequest;
using DistanceResult = ::fcl::DistanceResult;
using Contact = ::fcl::Contact;
#endif

#if FCL_VERSION_AT_LEAST(0,4,0) && !FCL_VERSION_AT_LEAST(0,6,0)
using Ellipsoid = ::fcl::Ellipsoid;
#endif

/// Returns norm of a 3-dim vector
double length(const dart::collision::fcl::Vector3& t);

/// Returns squared norm of a 3-dim vector
double length2(const dart::collision::fcl::Vector3& t);

/// Returns translation component of a transform
dart::collision::fcl::Vector3 getTranslation(
    const dart::collision::fcl::Transform3& T);

/// Sets translation component of a transform
void setTranslation(
    dart::collision::fcl::Transform3& T,
    const dart::collision::fcl::Vector3& t);

/// Returns rotation component of a transform
dart::collision::fcl::Matrix3 getRotation(
    const dart::collision::fcl::Transform3& T);

/// Sets rotation component of a transform
void setRotation(
    dart::collision::fcl::Transform3& T,
    const dart::collision::fcl::Matrix3& R);

/// Sets a rotation matrix given Euler-XYZ angles
void setEulerZYX(
    dart::collision::fcl::Matrix3& rot,
    double eulerX,
    double eulerY,
    double eulerZ);

/// Transforms a 3-dim vector by a transform and returns the result
dart::collision::fcl::Vector3 transform(
    const dart::collision::fcl::Transform3& t,
    const dart::collision::fcl::Vector3& v);

} // namespace fcl
} // namespace collision
} // namespace dart

#endif // DART_COLLISION_FCL_BACKWARDCOMPATIBILITY_HPP_
