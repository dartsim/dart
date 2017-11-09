/*
 * Copyright (c) 2011-2017, The DART development contributors
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

#ifndef DART_COLLISION_FCL_FCLTTYPES_HPP_
#define DART_COLLISION_FCL_FCLTTYPES_HPP_

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
#endif
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>


#if FCL_VERSION_AT_LEAST(0,5,0)
#include <memory>
template <class T> using fcl_shared_ptr = std::shared_ptr<T>;
template <class T> using fcl_weak_ptr = std::weak_ptr<T>;
#else
#include <boost/weak_ptr.hpp>
template <class T> using fcl_shared_ptr = boost::shared_ptr<T>;
template <class T> using fcl_weak_ptr = boost::weak_ptr<T>;
#endif

namespace dart {
namespace fcl {
#if FCL_VERSION_AT_LEAST(0,6,0)
// Geometric fundamentals
typedef ::fcl::Vector3<double> Vector3;
typedef ::fcl::Matrix3<double> Matrix3;
typedef ::fcl::Transform3<double> Transform3;
// Geometric primitives
typedef ::fcl::Box<double> Box;
typedef ::fcl::Cylinder<double> Cylinder;
typedef ::fcl::Ellipsoid<double> Ellipsoid;
typedef ::fcl::Halfspace<double> Halfspace;
typedef ::fcl::Sphere<double> Sphere;
// Collision objects
typedef ::fcl::CollisionObject<double> CollisionObject;
typedef ::fcl::CollisionGeometry<double> CollisionGeometry;
typedef ::fcl::DynamicAABBTreeCollisionManager<double> DynamicAABBTreeCollisionManager;
typedef ::fcl::OBBRSS<double> OBBRSS;
typedef ::fcl::CollisionRequest<double> CollisionRequest;
typedef ::fcl::CollisionResult<double> CollisionResult;
typedef ::fcl::DistanceRequest<double> DistanceRequest;
typedef ::fcl::DistanceResult<double> DistanceResult;
typedef ::fcl::Contact<double> Contact;
#else
// Geometric fundamentals
typedef ::fcl::Vec3f Vector3;
typedef ::fcl::Matrix3f Matrix3;
typedef ::fcl::Transform3f Transform3;
// Geometric primitives
typedef ::fcl::Box Box;
typedef ::fcl::Cylinder Cylinder;
typedef ::fcl::Halfspace Halfspace;
typedef ::fcl::Sphere Sphere;
// Collision objects
typedef ::fcl::CollisionObject CollisionObject;
typedef ::fcl::CollisionGeometry CollisionGeometry;
typedef ::fcl::DynamicAABBTreeCollisionManager DynamicAABBTreeCollisionManager;
typedef ::fcl::OBBRSS OBBRSS;
typedef ::fcl::CollisionRequest CollisionRequest;
typedef ::fcl::CollisionResult CollisionResult;
typedef ::fcl::DistanceRequest DistanceRequest;
typedef ::fcl::DistanceResult DistanceResult;
typedef ::fcl::Contact Contact;
#endif

#if FCL_VERSION_AT_LEAST(0,4,0) && !FCL_VERSION_AT_LEAST(0,6,0)
typedef ::fcl::Ellipsoid Ellipsoid;
#endif

template <typename BV>
using BVHModel = ::fcl::BVHModel<BV>;

} // namespace fcl

namespace collision {

class FCLTypes
{
public:
#if !FCL_VERSION_AT_LEAST(0,6,0)
  /// Convert Eigen vector3 type to FCL vector3 type
  static dart::fcl::Vector3 convertVector3(const Eigen::Vector3d& _vec);
#endif
  /// Convert FCL vector3 type to Eigen vector3 type
  static Eigen::Vector3d convertVector3(const dart::fcl::Vector3& _vec);

  /// Convert FCL matrix3x3 type to Eigen matrix3x3 type
  static dart::fcl::Matrix3 convertMatrix3x3(const Eigen::Matrix3d& _R);

  /// Convert FCL transformation type to Eigen transformation type
  static dart::fcl::Transform3 convertTransform(const Eigen::Isometry3d& _T);
};

double length(const dart::fcl::Vector3& t);
dart::fcl::Vector3 getTranslation(const dart::fcl::Transform3& T);
void setTranslation(dart::fcl::Transform3& T, const dart::fcl::Vector3& t);
dart::fcl::Matrix3 getRotation(const dart::fcl::Transform3& T);
void setRotation(dart::fcl::Transform3& T, const dart::fcl::Matrix3& R);
void setEulerZYX(dart::fcl::Matrix3& rot, const double eulerX, const double eulerY, const double eulerZ);

#if FCL_VERSION_AT_LEAST(0,6,0)
#define FCL_TRANSFORM(t,v) (t) * (v)
#else
#define FCL_TRANSFORM(t,v) (t).transform((v))
#endif

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_FCL_FCLTTYPES_HPP_
