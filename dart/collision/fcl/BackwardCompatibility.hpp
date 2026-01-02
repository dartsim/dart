/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#ifndef DART_COLLISION_FCL_BACKWARDCOMPATIBILITY_HPP_
#define DART_COLLISION_FCL_BACKWARDCOMPATIBILITY_HPP_

#include <dart/config.hpp>

#include <dart/Export.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <memory>

#ifndef FCL_MAJOR_VERSION
  #define FCL_MAJOR_VERSION 0
#endif
#ifndef FCL_MINOR_VERSION
  #define FCL_MINOR_VERSION 0
#endif
#ifndef FCL_PATCH_VERSION
  #define FCL_PATCH_VERSION 0
#endif

// clang-format off
#define FCL_VERSION_AT_LEAST(x,y,z)                                            \
  (FCL_MAJOR_VERSION > x || (FCL_MAJOR_VERSION >= x &&                         \
  (FCL_MINOR_VERSION > y || (FCL_MINOR_VERSION >= y &&                         \
  FCL_PATCH_VERSION >= z))))

#define FCL_MAJOR_MINOR_VERSION_AT_MOST(x,y)                                   \
  (FCL_MAJOR_VERSION < x || (FCL_MAJOR_VERSION <= x &&                         \
  (FCL_MINOR_VERSION < y || (FCL_MINOR_VERSION <= y))))
// clang-format on

namespace fcl {

enum NodeType
{
  GEOM_UNKNOWN = 0,
  GEOM_CONVEX = 1
};

template <typename S>
using Vector3 = Eigen::Matrix<S, 3, 1>;

template <typename S>
using Matrix3 = Eigen::Matrix<S, 3, 3>;

template <typename S>
using Transform3 = Eigen::Transform<S, 3, Eigen::Isometry>;

template <typename S>
class CollisionGeometry
{
public:
  NodeType getNodeType() const { return mNodeType; }
  void setNodeType(NodeType nodeType) { mNodeType = nodeType; }

private:
  NodeType mNodeType{GEOM_UNKNOWN};
};

template <typename S>
class CollisionObject
{
public:
  CollisionObject() = default;

  explicit CollisionObject(std::shared_ptr<CollisionGeometry<S>> geometry)
    : mGeometry(std::move(geometry))
  {
  }

  CollisionGeometry<S>* collisionGeometry() const { return mGeometry.get(); }

  void* getUserData() const { return mUserData; }
  void setUserData(void* data) { mUserData = data; }

private:
  std::shared_ptr<CollisionGeometry<S>> mGeometry;
  void* mUserData{nullptr};
};

template <typename S>
class DynamicAABBTreeCollisionManager
{
};

template <typename S>
class OBBRSS
{
};

template <typename S>
struct CollisionRequest
{
};

template <typename S>
struct CollisionResult
{
};

template <typename S>
struct DistanceRequest
{
};

template <typename S>
struct DistanceResult
{
};

template <typename S>
struct Contact
{
};

template <typename S>
class Box : public CollisionGeometry<S>
{
public:
  Box(S x, S y, S z) : mSize(x, y, z) {}

private:
  Vector3<S> mSize;
};

template <typename S>
class Cylinder : public CollisionGeometry<S>
{
public:
  Cylinder(S radius, S height) : mRadius(radius), mHeight(height) {}

private:
  S mRadius;
  S mHeight;
};

template <typename S>
class Cone : public CollisionGeometry<S>
{
public:
  Cone(S radius, S height) : mRadius(radius), mHeight(height) {}

private:
  S mRadius;
  S mHeight;
};

template <typename S>
class Ellipsoid : public CollisionGeometry<S>
{
public:
  Ellipsoid(S a, S b, S c) : mRadii(a, b, c) {}

private:
  Vector3<S> mRadii;
};

template <typename S>
class Halfspace : public CollisionGeometry<S>
{
public:
  Halfspace(const Vector3<S>& normal, S offset)
    : mNormal(normal), mOffset(offset)
  {
  }

private:
  Vector3<S> mNormal;
  S mOffset;
};

template <typename S>
class Sphere : public CollisionGeometry<S>
{
public:
  explicit Sphere(S radius) : mRadius(radius) {}

private:
  S mRadius;
};

template <typename S>
class OcTree : public CollisionGeometry<S>
{
};

template <class BV>
class BVHModel
{
public:
  void beginModel() {}

  template <typename Vec>
  void addTriangle(const Vec&, const Vec&, const Vec&)
  {
  }

  void endModel() {}
};

} // namespace fcl

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
using OcTree = ::fcl::OcTree<double>;
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

/// Returns norm of a 3-dim vector
double DART_API length(const dart::collision::fcl::Vector3& t);

/// Returns squared norm of a 3-dim vector
double DART_API length2(const dart::collision::fcl::Vector3& t);

[[nodiscard]] inline dart::collision::fcl::Transform3 getTransform3Identity()
{
  return dart::collision::fcl::Transform3::Identity();
}

/// Returns translation component of a transform
dart::collision::fcl::Vector3 DART_API
getTranslation(const dart::collision::fcl::Transform3& T);

/// Sets translation component of a transform
void DART_API setTranslation(
    dart::collision::fcl::Transform3& T,
    const dart::collision::fcl::Vector3& t);

/// Returns rotation component of a transform
dart::collision::fcl::Matrix3 DART_API
getRotation(const dart::collision::fcl::Transform3& T);

/// Sets rotation component of a transform
void DART_API setRotation(
    dart::collision::fcl::Transform3& T,
    const dart::collision::fcl::Matrix3& R);

/// Sets a rotation matrix given Euler-XYZ angles
void DART_API setEulerZYX(
    dart::collision::fcl::Matrix3& rot,
    double eulerX,
    double eulerY,
    double eulerZ);

/// Transforms a 3-dim vector by a transform and returns the result
dart::collision::fcl::Vector3 DART_API transform(
    const dart::collision::fcl::Transform3& t,
    const dart::collision::fcl::Vector3& v);

} // namespace fcl
} // namespace collision
} // namespace dart

#endif // DART_COLLISION_FCL_BACKWARDCOMPATIBILITY_HPP_
