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

#include <dart/collision/experimental/narrow_phase/box_box.hpp>
#include <dart/collision/experimental/narrow_phase/capsule_box.hpp>
#include <dart/collision/experimental/narrow_phase/capsule_capsule.hpp>
#include <dart/collision/experimental/narrow_phase/capsule_sphere.hpp>
#include <dart/collision/experimental/narrow_phase/ccd.hpp>
#include <dart/collision/experimental/narrow_phase/convex_convex.hpp>
#include <dart/collision/experimental/narrow_phase/cylinder_collision.hpp>
#include <dart/collision/experimental/narrow_phase/distance.hpp>
#include <dart/collision/experimental/narrow_phase/narrow_phase.hpp>
#include <dart/collision/experimental/narrow_phase/plane_sphere.hpp>
#include <dart/collision/experimental/narrow_phase/raycast.hpp>
#include <dart/collision/experimental/narrow_phase/sphere_box.hpp>
#include <dart/collision/experimental/narrow_phase/sphere_sphere.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <limits>

namespace dart::collision::experimental {

bool NarrowPhase::collide(
    const Shape* shape1,
    const Eigen::Isometry3d& tf1,
    const Shape* shape2,
    const Eigen::Isometry3d& tf2,
    const CollisionOption& option,
    CollisionResult& result)
{
  if (!shape1 || !shape2) {
    return false;
  }

  ShapeType type1 = shape1->getType();
  ShapeType type2 = shape2->getType();

  if (type1 == ShapeType::Sphere && type2 == ShapeType::Sphere) {
    const auto* s1 = static_cast<const SphereShape*>(shape1);
    const auto* s2 = static_cast<const SphereShape*>(shape2);
    return collideSpheres(*s1, tf1, *s2, tf2, result, option);
  }

  if (type1 == ShapeType::Box && type2 == ShapeType::Box) {
    const auto* b1 = static_cast<const BoxShape*>(shape1);
    const auto* b2 = static_cast<const BoxShape*>(shape2);
    return collideBoxes(*b1, tf1, *b2, tf2, result, option);
  }

  if (type1 == ShapeType::Sphere && type2 == ShapeType::Box) {
    const auto* s = static_cast<const SphereShape*>(shape1);
    const auto* b = static_cast<const BoxShape*>(shape2);
    return collideSphereBox(*s, tf1, *b, tf2, result, option);
  }

  if (type1 == ShapeType::Box && type2 == ShapeType::Sphere) {
    const auto* b = static_cast<const BoxShape*>(shape1);
    const auto* s = static_cast<const SphereShape*>(shape2);
    return collideSphereBox(*s, tf2, *b, tf1, result, option);
  }

  if (type1 == ShapeType::Capsule && type2 == ShapeType::Capsule) {
    const auto* c1 = static_cast<const CapsuleShape*>(shape1);
    const auto* c2 = static_cast<const CapsuleShape*>(shape2);
    return collideCapsules(*c1, tf1, *c2, tf2, result, option);
  }

  if (type1 == ShapeType::Capsule && type2 == ShapeType::Sphere) {
    const auto* c = static_cast<const CapsuleShape*>(shape1);
    const auto* s = static_cast<const SphereShape*>(shape2);
    return collideCapsuleSphere(*c, tf1, *s, tf2, result, option);
  }

  if (type1 == ShapeType::Sphere && type2 == ShapeType::Capsule) {
    const auto* s = static_cast<const SphereShape*>(shape1);
    const auto* c = static_cast<const CapsuleShape*>(shape2);
    return collideCapsuleSphere(*c, tf2, *s, tf1, result, option);
  }

  if (type1 == ShapeType::Capsule && type2 == ShapeType::Box) {
    const auto* c = static_cast<const CapsuleShape*>(shape1);
    const auto* b = static_cast<const BoxShape*>(shape2);
    return collideCapsuleBox(*c, tf1, *b, tf2, result, option);
  }

  if (type1 == ShapeType::Box && type2 == ShapeType::Capsule) {
    const auto* b = static_cast<const BoxShape*>(shape1);
    const auto* c = static_cast<const CapsuleShape*>(shape2);
    return collideCapsuleBox(*c, tf2, *b, tf1, result, option);
  }

  if (type1 == ShapeType::Plane && type2 == ShapeType::Sphere) {
    const auto* p = static_cast<const PlaneShape*>(shape1);
    const auto* s = static_cast<const SphereShape*>(shape2);
    return collidePlaneSphere(*p, tf1, *s, tf2, result, option);
  }

  if (type1 == ShapeType::Sphere && type2 == ShapeType::Plane) {
    const auto* s = static_cast<const SphereShape*>(shape1);
    const auto* p = static_cast<const PlaneShape*>(shape2);
    return collidePlaneSphere(*p, tf2, *s, tf1, result, option);
  }

  if (type1 == ShapeType::Plane && type2 == ShapeType::Box) {
    const auto* p = static_cast<const PlaneShape*>(shape1);
    const auto* b = static_cast<const BoxShape*>(shape2);
    return collidePlaneBox(*p, tf1, *b, tf2, result, option);
  }

  if (type1 == ShapeType::Box && type2 == ShapeType::Plane) {
    const auto* b = static_cast<const BoxShape*>(shape1);
    const auto* p = static_cast<const PlaneShape*>(shape2);
    return collidePlaneBox(*p, tf2, *b, tf1, result, option);
  }

  if (type1 == ShapeType::Plane && type2 == ShapeType::Capsule) {
    const auto* p = static_cast<const PlaneShape*>(shape1);
    const auto* c = static_cast<const CapsuleShape*>(shape2);
    return collidePlaneCapsule(*p, tf1, *c, tf2, result, option);
  }

  if (type1 == ShapeType::Capsule && type2 == ShapeType::Plane) {
    const auto* c = static_cast<const CapsuleShape*>(shape1);
    const auto* p = static_cast<const PlaneShape*>(shape2);
    return collidePlaneCapsule(*p, tf2, *c, tf1, result, option);
  }

  if (type1 == ShapeType::Cylinder && type2 == ShapeType::Cylinder) {
    const auto* c1 = static_cast<const CylinderShape*>(shape1);
    const auto* c2 = static_cast<const CylinderShape*>(shape2);
    return collideCylinders(*c1, tf1, *c2, tf2, result, option);
  }

  if (type1 == ShapeType::Cylinder && type2 == ShapeType::Sphere) {
    const auto* c = static_cast<const CylinderShape*>(shape1);
    const auto* s = static_cast<const SphereShape*>(shape2);
    return collideCylinderSphere(*c, tf1, *s, tf2, result, option);
  }

  if (type1 == ShapeType::Sphere && type2 == ShapeType::Cylinder) {
    const auto* s = static_cast<const SphereShape*>(shape1);
    const auto* c = static_cast<const CylinderShape*>(shape2);
    return collideCylinderSphere(*c, tf2, *s, tf1, result, option);
  }

  if (type1 == ShapeType::Cylinder && type2 == ShapeType::Box) {
    const auto* c = static_cast<const CylinderShape*>(shape1);
    const auto* b = static_cast<const BoxShape*>(shape2);
    return collideCylinderBox(*c, tf1, *b, tf2, result, option);
  }

  if (type1 == ShapeType::Box && type2 == ShapeType::Cylinder) {
    const auto* b = static_cast<const BoxShape*>(shape1);
    const auto* c = static_cast<const CylinderShape*>(shape2);
    return collideCylinderBox(*c, tf2, *b, tf1, result, option);
  }

  if (type1 == ShapeType::Cylinder && type2 == ShapeType::Capsule) {
    const auto* cyl = static_cast<const CylinderShape*>(shape1);
    const auto* cap = static_cast<const CapsuleShape*>(shape2);
    return collideCylinderCapsule(*cyl, tf1, *cap, tf2, result, option);
  }

  if (type1 == ShapeType::Capsule && type2 == ShapeType::Cylinder) {
    const auto* cap = static_cast<const CapsuleShape*>(shape1);
    const auto* cyl = static_cast<const CylinderShape*>(shape2);
    return collideCylinderCapsule(*cyl, tf2, *cap, tf1, result, option);
  }

  if (type1 == ShapeType::Cylinder && type2 == ShapeType::Plane) {
    const auto* c = static_cast<const CylinderShape*>(shape1);
    const auto* p = static_cast<const PlaneShape*>(shape2);
    return collideCylinderPlane(*c, tf1, *p, tf2, result, option);
  }

  if (type1 == ShapeType::Plane && type2 == ShapeType::Cylinder) {
    const auto* p = static_cast<const PlaneShape*>(shape1);
    const auto* c = static_cast<const CylinderShape*>(shape2);
    return collideCylinderPlane(*c, tf2, *p, tf1, result, option);
  }

  if (type1 == ShapeType::Convex || type1 == ShapeType::Mesh
      || type2 == ShapeType::Convex || type2 == ShapeType::Mesh) {
    return collideConvexConvex(*shape1, tf1, *shape2, tf2, result, option);
  }

  return false;
}

bool NarrowPhase::collide(
    const CollisionObject& obj1,
    const CollisionObject& obj2,
    const CollisionOption& option,
    CollisionResult& result)
{
  return collide(
      obj1.getShape(),
      obj1.getTransform(),
      obj2.getShape(),
      obj2.getTransform(),
      option,
      result);
}

bool NarrowPhase::isSupported(ShapeType type1, ShapeType type2)
{
  if (type1 == ShapeType::Sphere && type2 == ShapeType::Sphere) {
    return true;
  }
  if (type1 == ShapeType::Box && type2 == ShapeType::Box) {
    return true;
  }
  if ((type1 == ShapeType::Sphere && type2 == ShapeType::Box)
      || (type1 == ShapeType::Box && type2 == ShapeType::Sphere)) {
    return true;
  }
  if (type1 == ShapeType::Capsule && type2 == ShapeType::Capsule) {
    return true;
  }
  if ((type1 == ShapeType::Capsule && type2 == ShapeType::Sphere)
      || (type1 == ShapeType::Sphere && type2 == ShapeType::Capsule)) {
    return true;
  }
  if ((type1 == ShapeType::Capsule && type2 == ShapeType::Box)
      || (type1 == ShapeType::Box && type2 == ShapeType::Capsule)) {
    return true;
  }
  if ((type1 == ShapeType::Plane && type2 == ShapeType::Sphere)
      || (type1 == ShapeType::Sphere && type2 == ShapeType::Plane)) {
    return true;
  }
  if ((type1 == ShapeType::Plane && type2 == ShapeType::Box)
      || (type1 == ShapeType::Box && type2 == ShapeType::Plane)) {
    return true;
  }
  if ((type1 == ShapeType::Plane && type2 == ShapeType::Capsule)
      || (type1 == ShapeType::Capsule && type2 == ShapeType::Plane)) {
    return true;
  }
  if (type1 == ShapeType::Cylinder && type2 == ShapeType::Cylinder) {
    return true;
  }
  if ((type1 == ShapeType::Cylinder && type2 == ShapeType::Sphere)
      || (type1 == ShapeType::Sphere && type2 == ShapeType::Cylinder)) {
    return true;
  }
  if ((type1 == ShapeType::Cylinder && type2 == ShapeType::Box)
      || (type1 == ShapeType::Box && type2 == ShapeType::Cylinder)) {
    return true;
  }
  if ((type1 == ShapeType::Cylinder && type2 == ShapeType::Capsule)
      || (type1 == ShapeType::Capsule && type2 == ShapeType::Cylinder)) {
    return true;
  }
  if ((type1 == ShapeType::Cylinder && type2 == ShapeType::Plane)
      || (type1 == ShapeType::Plane && type2 == ShapeType::Cylinder)) {
    return true;
  }
  if (type1 == ShapeType::Convex || type1 == ShapeType::Mesh
      || type2 == ShapeType::Convex || type2 == ShapeType::Mesh) {
    return true;
  }
  return false;
}

double NarrowPhase::distance(
    const CollisionObject& obj1,
    const CollisionObject& obj2,
    const DistanceOption& option,
    DistanceResult& result)
{
  const Shape* shape1 = obj1.getShape();
  const Shape* shape2 = obj2.getShape();

  if (!shape1 || !shape2) {
    return std::numeric_limits<double>::max();
  }

  ShapeType type1 = shape1->getType();
  ShapeType type2 = shape2->getType();

  const Eigen::Isometry3d& tf1 = obj1.getTransform();
  const Eigen::Isometry3d& tf2 = obj2.getTransform();

  result.object1 = &obj1;
  result.object2 = &obj2;

  if (type1 == ShapeType::Sphere && type2 == ShapeType::Sphere) {
    const auto* s1 = static_cast<const SphereShape*>(shape1);
    const auto* s2 = static_cast<const SphereShape*>(shape2);
    return distanceSphereSphere(*s1, tf1, *s2, tf2, result, option);
  }

  if (type1 == ShapeType::Sphere && type2 == ShapeType::Sdf) {
    const auto* s = static_cast<const SphereShape*>(shape1);
    const auto* sdf = static_cast<const SdfShape*>(shape2);
    return distanceSphereSdf(*s, tf1, *sdf, tf2, result, option);
  }

  if (type1 == ShapeType::Sdf && type2 == ShapeType::Sphere) {
    const auto* sdf = static_cast<const SdfShape*>(shape1);
    const auto* s = static_cast<const SphereShape*>(shape2);
    double d = distanceSphereSdf(*s, tf2, *sdf, tf1, result, option);
    std::swap(result.pointOnObject1, result.pointOnObject2);
    result.normal = -result.normal;
    return d;
  }

  if (type1 == ShapeType::Sphere && type2 == ShapeType::Box) {
    const auto* s = static_cast<const SphereShape*>(shape1);
    const auto* b = static_cast<const BoxShape*>(shape2);
    return distanceSphereBox(*s, tf1, *b, tf2, result, option);
  }

  if (type1 == ShapeType::Box && type2 == ShapeType::Sphere) {
    const auto* b = static_cast<const BoxShape*>(shape1);
    const auto* s = static_cast<const SphereShape*>(shape2);
    double d = distanceSphereBox(*s, tf2, *b, tf1, result, option);
    std::swap(result.pointOnObject1, result.pointOnObject2);
    result.normal = -result.normal;
    return d;
  }

  if (type1 == ShapeType::Box && type2 == ShapeType::Box) {
    const auto* b1 = static_cast<const BoxShape*>(shape1);
    const auto* b2 = static_cast<const BoxShape*>(shape2);
    return distanceBoxBox(*b1, tf1, *b2, tf2, result, option);
  }

  if (type1 == ShapeType::Capsule && type2 == ShapeType::Capsule) {
    const auto* c1 = static_cast<const CapsuleShape*>(shape1);
    const auto* c2 = static_cast<const CapsuleShape*>(shape2);
    return distanceCapsuleCapsule(*c1, tf1, *c2, tf2, result, option);
  }

  if (type1 == ShapeType::Capsule && type2 == ShapeType::Sdf) {
    const auto* c = static_cast<const CapsuleShape*>(shape1);
    const auto* sdf = static_cast<const SdfShape*>(shape2);
    return distanceCapsuleSdf(*c, tf1, *sdf, tf2, result, option);
  }

  if (type1 == ShapeType::Sdf && type2 == ShapeType::Capsule) {
    const auto* sdf = static_cast<const SdfShape*>(shape1);
    const auto* c = static_cast<const CapsuleShape*>(shape2);
    double d = distanceCapsuleSdf(*c, tf2, *sdf, tf1, result, option);
    std::swap(result.pointOnObject1, result.pointOnObject2);
    result.normal = -result.normal;
    return d;
  }

  if (type1 == ShapeType::Capsule && type2 == ShapeType::Sphere) {
    const auto* c = static_cast<const CapsuleShape*>(shape1);
    const auto* s = static_cast<const SphereShape*>(shape2);
    return distanceCapsuleSphere(*c, tf1, *s, tf2, result, option);
  }

  if (type1 == ShapeType::Sphere && type2 == ShapeType::Capsule) {
    const auto* s = static_cast<const SphereShape*>(shape1);
    const auto* c = static_cast<const CapsuleShape*>(shape2);
    double d = distanceCapsuleSphere(*c, tf2, *s, tf1, result, option);
    std::swap(result.pointOnObject1, result.pointOnObject2);
    result.normal = -result.normal;
    return d;
  }

  if (type1 == ShapeType::Capsule && type2 == ShapeType::Box) {
    const auto* c = static_cast<const CapsuleShape*>(shape1);
    const auto* b = static_cast<const BoxShape*>(shape2);
    return distanceCapsuleBox(*c, tf1, *b, tf2, result, option);
  }

  if (type1 == ShapeType::Box && type2 == ShapeType::Capsule) {
    const auto* b = static_cast<const BoxShape*>(shape1);
    const auto* c = static_cast<const CapsuleShape*>(shape2);
    double d = distanceCapsuleBox(*c, tf2, *b, tf1, result, option);
    std::swap(result.pointOnObject1, result.pointOnObject2);
    result.normal = -result.normal;
    return d;
  }

  if (type1 == ShapeType::Convex || type1 == ShapeType::Mesh
      || type2 == ShapeType::Convex || type2 == ShapeType::Mesh) {
    return distanceConvexConvex(*shape1, tf1, *shape2, tf2, result, option);
  }

  return std::numeric_limits<double>::max();
}

bool NarrowPhase::isDistanceSupported(ShapeType type1, ShapeType type2)
{
  if (type1 == ShapeType::Sphere && type2 == ShapeType::Sphere) {
    return true;
  }
  if ((type1 == ShapeType::Sphere && type2 == ShapeType::Sdf)
      || (type1 == ShapeType::Sdf && type2 == ShapeType::Sphere)) {
    return true;
  }
  if (type1 == ShapeType::Box && type2 == ShapeType::Box) {
    return true;
  }
  if ((type1 == ShapeType::Sphere && type2 == ShapeType::Box)
      || (type1 == ShapeType::Box && type2 == ShapeType::Sphere)) {
    return true;
  }
  if (type1 == ShapeType::Capsule && type2 == ShapeType::Capsule) {
    return true;
  }
  if ((type1 == ShapeType::Capsule && type2 == ShapeType::Sdf)
      || (type1 == ShapeType::Sdf && type2 == ShapeType::Capsule)) {
    return true;
  }
  if ((type1 == ShapeType::Capsule && type2 == ShapeType::Sphere)
      || (type1 == ShapeType::Sphere && type2 == ShapeType::Capsule)) {
    return true;
  }
  if ((type1 == ShapeType::Capsule && type2 == ShapeType::Box)
      || (type1 == ShapeType::Box && type2 == ShapeType::Capsule)) {
    return true;
  }
  if (type1 == ShapeType::Convex || type1 == ShapeType::Mesh
      || type2 == ShapeType::Convex || type2 == ShapeType::Mesh) {
    return true;
  }
  return false;
}

bool NarrowPhase::raycast(
    const Ray& ray,
    const CollisionObject& object,
    const RaycastOption& option,
    RaycastResult& result)
{
  const Shape* shape = object.getShape();
  if (!shape) {
    return false;
  }

  ShapeType type = shape->getType();
  const Eigen::Isometry3d& transform = object.getTransform();

  switch (type) {
    case ShapeType::Sphere: {
      const auto* s = static_cast<const SphereShape*>(shape);
      bool hit = raycastSphere(ray, *s, transform, option, result);
      if (hit) {
        result.object = &object;
      }
      return hit;
    }
    case ShapeType::Box: {
      const auto* b = static_cast<const BoxShape*>(shape);
      bool hit = raycastBox(ray, *b, transform, option, result);
      if (hit) {
        result.object = &object;
      }
      return hit;
    }
    case ShapeType::Capsule: {
      const auto* c = static_cast<const CapsuleShape*>(shape);
      bool hit = raycastCapsule(ray, *c, transform, option, result);
      if (hit) {
        result.object = &object;
      }
      return hit;
    }
    case ShapeType::Cylinder: {
      const auto* c = static_cast<const CylinderShape*>(shape);
      bool hit = raycastCylinder(ray, *c, transform, option, result);
      if (hit) {
        result.object = &object;
      }
      return hit;
    }
    case ShapeType::Plane: {
      const auto* p = static_cast<const PlaneShape*>(shape);
      bool hit = raycastPlane(ray, *p, transform, option, result);
      if (hit) {
        result.object = &object;
      }
      return hit;
    }
    case ShapeType::Mesh: {
      const auto* m = static_cast<const MeshShape*>(shape);
      bool hit = raycastMesh(ray, *m, transform, option, result);
      if (hit) {
        result.object = &object;
      }
      return hit;
    }
    case ShapeType::Convex: {
      const auto* c = static_cast<const ConvexShape*>(shape);
      bool hit = raycastConvex(ray, *c, transform, option, result);
      if (hit) {
        result.object = &object;
      }
      return hit;
    }
    default:
      return false;
  }
}

bool NarrowPhase::isRaycastSupported(ShapeType type)
{
  switch (type) {
    case ShapeType::Sphere:
    case ShapeType::Box:
    case ShapeType::Capsule:
    case ShapeType::Cylinder:
    case ShapeType::Plane:
    case ShapeType::Mesh:
    case ShapeType::Convex:
      return true;
    default:
      return false;
  }
}

bool NarrowPhase::sphereCast(
    const Eigen::Vector3d& sphereStart,
    const Eigen::Vector3d& sphereEnd,
    double sphereRadius,
    const CollisionObject& target,
    const CcdOption& option,
    CcdResult& result)
{
  const Shape* shape = target.getShape();
  if (!shape) {
    return false;
  }

  ShapeType type = shape->getType();
  const Eigen::Isometry3d& transform = target.getTransform();

  switch (type) {
    case ShapeType::Sphere: {
      const auto* s = static_cast<const SphereShape*>(shape);
      bool hit = sphereCastSphere(
          sphereStart, sphereEnd, sphereRadius, *s, transform, option, result);
      if (hit) {
        result.object = &target;
      }
      return hit;
    }
    case ShapeType::Box: {
      const auto* b = static_cast<const BoxShape*>(shape);
      bool hit = sphereCastBox(
          sphereStart, sphereEnd, sphereRadius, *b, transform, option, result);
      if (hit) {
        result.object = &target;
      }
      return hit;
    }
    case ShapeType::Capsule: {
      const auto* c = static_cast<const CapsuleShape*>(shape);
      bool hit = sphereCastCapsule(
          sphereStart, sphereEnd, sphereRadius, *c, transform, option, result);
      if (hit) {
        result.object = &target;
      }
      return hit;
    }
    case ShapeType::Cylinder: {
      const auto* c = static_cast<const CylinderShape*>(shape);
      bool hit = sphereCastCylinder(
          sphereStart, sphereEnd, sphereRadius, *c, transform, option, result);
      if (hit) {
        result.object = &target;
      }
      return hit;
    }
    case ShapeType::Plane: {
      const auto* p = static_cast<const PlaneShape*>(shape);
      bool hit = sphereCastPlane(
          sphereStart, sphereEnd, sphereRadius, *p, transform, option, result);
      if (hit) {
        result.object = &target;
      }
      return hit;
    }
    case ShapeType::Convex: {
      const auto* c = static_cast<const ConvexShape*>(shape);
      bool hit = sphereCastConvex(
          sphereStart, sphereEnd, sphereRadius, *c, transform, option, result);
      if (hit) {
        result.object = &target;
      }
      return hit;
    }
    case ShapeType::Mesh: {
      const auto* m = static_cast<const MeshShape*>(shape);
      bool hit = sphereCastMesh(
          sphereStart, sphereEnd, sphereRadius, *m, transform, option, result);
      if (hit) {
        result.object = &target;
      }
      return hit;
    }
    default:
      return false;
  }
}

bool NarrowPhase::isSphereCastSupported(ShapeType type)
{
  switch (type) {
    case ShapeType::Sphere:
    case ShapeType::Box:
    case ShapeType::Capsule:
    case ShapeType::Cylinder:
    case ShapeType::Plane:
    case ShapeType::Convex:
    case ShapeType::Mesh:
      return true;
    default:
      return false;
  }
}

bool NarrowPhase::capsuleCast(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const CollisionObject& target,
    const CcdOption& option,
    CcdResult& result)
{
  const Shape* shape = target.getShape();
  if (!shape) {
    return false;
  }

  ShapeType type = shape->getType();
  const Eigen::Isometry3d& transform = target.getTransform();

  switch (type) {
    case ShapeType::Sphere: {
      const auto* s = static_cast<const SphereShape*>(shape);
      bool hit = capsuleCastSphere(
          capsuleStart, capsuleEnd, capsule, *s, transform, option, result);
      if (hit) {
        result.object = &target;
      }
      return hit;
    }
    case ShapeType::Box: {
      const auto* b = static_cast<const BoxShape*>(shape);
      bool hit = capsuleCastBox(
          capsuleStart, capsuleEnd, capsule, *b, transform, option, result);
      if (hit) {
        result.object = &target;
      }
      return hit;
    }
    case ShapeType::Capsule: {
      const auto* c = static_cast<const CapsuleShape*>(shape);
      bool hit = capsuleCastCapsule(
          capsuleStart, capsuleEnd, capsule, *c, transform, option, result);
      if (hit) {
        result.object = &target;
      }
      return hit;
    }
    case ShapeType::Plane: {
      const auto* p = static_cast<const PlaneShape*>(shape);
      bool hit = capsuleCastPlane(
          capsuleStart, capsuleEnd, capsule, *p, transform, option, result);
      if (hit) {
        result.object = &target;
      }
      return hit;
    }
    case ShapeType::Cylinder: {
      const auto* c = static_cast<const CylinderShape*>(shape);
      bool hit = capsuleCastCylinder(
          capsuleStart, capsuleEnd, capsule, *c, transform, option, result);
      if (hit) {
        result.object = &target;
      }
      return hit;
    }
    case ShapeType::Convex: {
      const auto* c = static_cast<const ConvexShape*>(shape);
      bool hit = capsuleCastConvex(
          capsuleStart, capsuleEnd, capsule, *c, transform, option, result);
      if (hit) {
        result.object = &target;
      }
      return hit;
    }
    case ShapeType::Mesh: {
      const auto* m = static_cast<const MeshShape*>(shape);
      bool hit = capsuleCastMesh(
          capsuleStart, capsuleEnd, capsule, *m, transform, option, result);
      if (hit) {
        result.object = &target;
      }
      return hit;
    }
    default:
      return false;
  }
}

bool NarrowPhase::isCapsuleCastSupported(ShapeType type)
{
  switch (type) {
    case ShapeType::Sphere:
    case ShapeType::Box:
    case ShapeType::Capsule:
    case ShapeType::Plane:
    case ShapeType::Cylinder:
    case ShapeType::Convex:
    case ShapeType::Mesh:
      return true;
    default:
      return false;
  }
}

} // namespace dart::collision::experimental
