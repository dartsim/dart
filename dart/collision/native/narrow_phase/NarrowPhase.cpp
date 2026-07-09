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

#include <dart/collision/native/narrow_phase/BoxBox.hpp>
#include <dart/collision/native/narrow_phase/CapsuleBox.hpp>
#include <dart/collision/native/narrow_phase/CapsuleCapsule.hpp>
#include <dart/collision/native/narrow_phase/CapsuleSphere.hpp>
#include <dart/collision/native/narrow_phase/Ccd.hpp>
#include <dart/collision/native/narrow_phase/ConvexConvex.hpp>
#include <dart/collision/native/narrow_phase/CylinderCollision.hpp>
#include <dart/collision/native/narrow_phase/Distance.hpp>
#include <dart/collision/native/narrow_phase/MeshMesh.hpp>
#include <dart/collision/native/narrow_phase/NarrowPhase.hpp>
#include <dart/collision/native/narrow_phase/PlaneSphere.hpp>
#include <dart/collision/native/narrow_phase/Raycast.hpp>
#include <dart/collision/native/narrow_phase/SphereBox.hpp>
#include <dart/collision/native/narrow_phase/SphereSphere.hpp>
#include <dart/collision/native/shapes/Shape.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

namespace dart::collision::native {

namespace {

template <typename CollideFn>
bool collideWithFlippedNormals(
    CollisionResult& result,
    const CollisionOption& option,
    CollideFn&& collideFn)
{
  const auto existingContacts = result.numContacts();
  if (option.enableContact && existingContacts >= option.maxNumContacts) {
    return false;
  }

  CollisionOption localOption = option;
  if (option.enableContact) {
    localOption.maxNumContacts = option.maxNumContacts - existingContacts;
  }

  CollisionResult localResult;
  const bool hit = collideFn(localResult, localOption);
  if (!hit) {
    return false;
  }

  for (const auto& manifold : localResult.getManifolds()) {
    ContactManifold flipped;
    flipped.setType(manifold.getType());
    flipped.setObjects(manifold.getObject2(), manifold.getObject1());
    for (ContactPoint contact : manifold.getContacts()) {
      contact.normal = -contact.normal;
      std::swap(contact.object1, contact.object2);
      std::swap(contact.featureIndex1, contact.featureIndex2);
      flipped.addContact(contact);
    }
    result.addManifold(std::move(flipped));
  }

  return true;
}

bool isConvexFallbackType(ShapeType type)
{
  return type == ShapeType::Sphere || type == ShapeType::Box
         || type == ShapeType::Capsule || type == ShapeType::Cylinder
         || type == ShapeType::Convex;
}

bool isMeshPrimitiveType(ShapeType type)
{
  return type == ShapeType::Sphere || type == ShapeType::Box
         || type == ShapeType::Capsule || type == ShapeType::Cylinder
         || type == ShapeType::Convex;
}

bool isPlaneDistanceShapeType(ShapeType type)
{
  return type == ShapeType::Sphere || type == ShapeType::Box
         || type == ShapeType::Capsule || type == ShapeType::Cylinder
         || type == ShapeType::Convex || type == ShapeType::Mesh;
}

bool isCompoundCollisionPeer(ShapeType type)
{
  return type != ShapeType::Sdf;
}

bool collideShapes(
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

  if (option.maxNumContacts == 0) {
    return false;
  }

  ShapeType type1 = shape1->getType();
  ShapeType type2 = shape2->getType();

  if (type1 == ShapeType::Compound || type2 == ShapeType::Compound) {
    if (option.enableContact && result.numContacts() >= option.maxNumContacts)
      return false;

    bool hit = false;
    auto collideChild = [&](const Shape* childShape1,
                            const Eigen::Isometry3d& childTf1,
                            const Shape* childShape2,
                            const Eigen::Isometry3d& childTf2) {
      CollisionOption childOption = option;
      if (option.enableContact) {
        const std::size_t existingContacts = result.numContacts();
        if (existingContacts >= option.maxNumContacts)
          return true;

        childOption.maxNumContacts = option.maxNumContacts - existingContacts;
      }

      CollisionResult childResult;
      if (!collideShapes(
              childShape1,
              childTf1,
              childShape2,
              childTf2,
              childOption,
              childResult)) {
        return false;
      }

      hit = true;
      if (!option.enableContact)
        return true;

      for (const auto& manifold : childResult.getManifolds())
        result.addManifold(manifold);

      return result.numContacts() >= option.maxNumContacts;
    };

    if (type1 == ShapeType::Compound && type2 == ShapeType::Compound) {
      const auto* compound1 = static_cast<const CompoundShape*>(shape1);
      const auto* compound2 = static_cast<const CompoundShape*>(shape2);
      for (const auto& child1 : compound1->children()) {
        if (!child1.shape)
          continue;

        const Eigen::Isometry3d childTf1 = tf1 * child1.localTransform;
        for (const auto& child2 : compound2->children()) {
          if (!child2.shape)
            continue;

          const Eigen::Isometry3d childTf2 = tf2 * child2.localTransform;
          if (collideChild(
                  child1.shape.get(), childTf1, child2.shape.get(), childTf2))
            return hit;
        }
      }
    } else if (type1 == ShapeType::Compound) {
      const auto* compound = static_cast<const CompoundShape*>(shape1);
      for (const auto& child : compound->children()) {
        if (!child.shape)
          continue;

        const Eigen::Isometry3d childTf = tf1 * child.localTransform;
        if (collideChild(child.shape.get(), childTf, shape2, tf2))
          return hit;
      }
    } else {
      const auto* compound = static_cast<const CompoundShape*>(shape2);
      for (const auto& child : compound->children()) {
        if (!child.shape)
          continue;

        const Eigen::Isometry3d childTf = tf2 * child.localTransform;
        if (collideChild(shape1, tf1, child.shape.get(), childTf))
          return hit;
      }
    }

    return hit;
  }

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

  if (type1 == ShapeType::Plane && type2 == ShapeType::Sphere) {
    const auto* p = static_cast<const PlaneShape*>(shape1);
    const auto* s = static_cast<const SphereShape*>(shape2);
    return collideWithFlippedNormals(
        result,
        option,
        [&](CollisionResult& local, const CollisionOption& opt) {
          return collidePlaneSphere(*p, tf1, *s, tf2, local, opt);
        });
  }

  if (type1 == ShapeType::Sphere && type2 == ShapeType::Plane) {
    const auto* s = static_cast<const SphereShape*>(shape1);
    const auto* p = static_cast<const PlaneShape*>(shape2);
    return collidePlaneSphere(*p, tf2, *s, tf1, result, option);
  }

  if (type1 == ShapeType::Plane && type2 == ShapeType::Box) {
    const auto* p = static_cast<const PlaneShape*>(shape1);
    const auto* b = static_cast<const BoxShape*>(shape2);
    return collideWithFlippedNormals(
        result,
        option,
        [&](CollisionResult& local, const CollisionOption& opt) {
          return collidePlaneBox(*p, tf1, *b, tf2, local, opt);
        });
  }

  if (type1 == ShapeType::Box && type2 == ShapeType::Plane) {
    const auto* b = static_cast<const BoxShape*>(shape1);
    const auto* p = static_cast<const PlaneShape*>(shape2);
    return collidePlaneBox(*p, tf2, *b, tf1, result, option);
  }

  if (type1 == ShapeType::Plane && type2 == ShapeType::Capsule) {
    const auto* p = static_cast<const PlaneShape*>(shape1);
    const auto* c = static_cast<const CapsuleShape*>(shape2);
    return collideWithFlippedNormals(
        result,
        option,
        [&](CollisionResult& local, const CollisionOption& opt) {
          return collidePlaneCapsule(*p, tf1, *c, tf2, local, opt);
        });
  }

  if (type1 == ShapeType::Capsule && type2 == ShapeType::Plane) {
    const auto* c = static_cast<const CapsuleShape*>(shape1);
    const auto* p = static_cast<const PlaneShape*>(shape2);
    return collidePlaneCapsule(*p, tf2, *c, tf1, result, option);
  }

  if (type1 == ShapeType::Plane && type2 == ShapeType::Convex) {
    const auto* p = static_cast<const PlaneShape*>(shape1);
    const auto* c = static_cast<const ConvexShape*>(shape2);
    return collideWithFlippedNormals(
        result,
        option,
        [&](CollisionResult& local, const CollisionOption& opt) {
          return collidePlaneConvex(*p, tf1, *c, tf2, local, opt);
        });
  }

  if (type1 == ShapeType::Convex && type2 == ShapeType::Plane) {
    const auto* c = static_cast<const ConvexShape*>(shape1);
    const auto* p = static_cast<const PlaneShape*>(shape2);
    return collidePlaneConvex(*p, tf2, *c, tf1, result, option);
  }

  if (type1 == ShapeType::Sphere && type2 == ShapeType::Box) {
    const auto* s = static_cast<const SphereShape*>(shape1);
    const auto* b = static_cast<const BoxShape*>(shape2);
    return collideWithFlippedNormals(
        result,
        option,
        [&](CollisionResult& local, const CollisionOption& opt) {
          return collideSphereBox(*s, tf1, *b, tf2, local, opt);
        });
  }

  if (type1 == ShapeType::Box && type2 == ShapeType::Sphere) {
    const auto* b = static_cast<const BoxShape*>(shape1);
    const auto* s = static_cast<const SphereShape*>(shape2);
    return collideSphereBox(*s, tf2, *b, tf1, result, option);
  }

  if (type1 == ShapeType::Capsule && type2 == ShapeType::Sphere) {
    const auto* c = static_cast<const CapsuleShape*>(shape1);
    const auto* s = static_cast<const SphereShape*>(shape2);
    return collideCapsuleSphere(*c, tf1, *s, tf2, result, option);
  }

  if (type1 == ShapeType::Sphere && type2 == ShapeType::Capsule) {
    const auto* s = static_cast<const SphereShape*>(shape1);
    const auto* c = static_cast<const CapsuleShape*>(shape2);
    return collideWithFlippedNormals(
        result,
        option,
        [&](CollisionResult& local, const CollisionOption& opt) {
          return collideCapsuleSphere(*c, tf2, *s, tf1, local, opt);
        });
  }

  if (type1 == ShapeType::Capsule && type2 == ShapeType::Box) {
    const auto* c = static_cast<const CapsuleShape*>(shape1);
    const auto* b = static_cast<const BoxShape*>(shape2);
    return collideCapsuleBox(*c, tf1, *b, tf2, result, option);
  }

  if (type1 == ShapeType::Box && type2 == ShapeType::Capsule) {
    const auto* b = static_cast<const BoxShape*>(shape1);
    const auto* c = static_cast<const CapsuleShape*>(shape2);
    return collideWithFlippedNormals(
        result,
        option,
        [&](CollisionResult& local, const CollisionOption& opt) {
          return collideCapsuleBox(*c, tf2, *b, tf1, local, opt);
        });
  }

  if (type1 == ShapeType::Capsule && type2 == ShapeType::Capsule) {
    const auto* c1 = static_cast<const CapsuleShape*>(shape1);
    const auto* c2 = static_cast<const CapsuleShape*>(shape2);
    return collideCapsules(*c1, tf1, *c2, tf2, result, option);
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
    return collideWithFlippedNormals(
        result,
        option,
        [&](CollisionResult& local, const CollisionOption& opt) {
          return collideCylinderSphere(*c, tf2, *s, tf1, local, opt);
        });
  }

  if (type1 == ShapeType::Cylinder && type2 == ShapeType::Box) {
    const auto* c = static_cast<const CylinderShape*>(shape1);
    const auto* b = static_cast<const BoxShape*>(shape2);
    return collideCylinderBox(*c, tf1, *b, tf2, result, option);
  }

  if (type1 == ShapeType::Box && type2 == ShapeType::Cylinder) {
    const auto* b = static_cast<const BoxShape*>(shape1);
    const auto* c = static_cast<const CylinderShape*>(shape2);
    return collideWithFlippedNormals(
        result,
        option,
        [&](CollisionResult& local, const CollisionOption& opt) {
          return collideCylinderBox(*c, tf2, *b, tf1, local, opt);
        });
  }

  if (type1 == ShapeType::Cylinder && type2 == ShapeType::Capsule) {
    const auto* cyl = static_cast<const CylinderShape*>(shape1);
    const auto* cap = static_cast<const CapsuleShape*>(shape2);
    return collideCylinderCapsule(*cyl, tf1, *cap, tf2, result, option);
  }

  if (type1 == ShapeType::Capsule && type2 == ShapeType::Cylinder) {
    const auto* cap = static_cast<const CapsuleShape*>(shape1);
    const auto* cyl = static_cast<const CylinderShape*>(shape2);
    return collideWithFlippedNormals(
        result,
        option,
        [&](CollisionResult& local, const CollisionOption& opt) {
          return collideCylinderCapsule(*cyl, tf2, *cap, tf1, local, opt);
        });
  }

  if (type1 == ShapeType::Cylinder && type2 == ShapeType::Plane) {
    const auto* c = static_cast<const CylinderShape*>(shape1);
    const auto* p = static_cast<const PlaneShape*>(shape2);
    return collideCylinderPlane(*c, tf1, *p, tf2, result, option);
  }

  if (type1 == ShapeType::Plane && type2 == ShapeType::Cylinder) {
    const auto* p = static_cast<const PlaneShape*>(shape1);
    const auto* c = static_cast<const CylinderShape*>(shape2);
    return collideWithFlippedNormals(
        result,
        option,
        [&](CollisionResult& local, const CollisionOption& opt) {
          return collideCylinderPlane(*c, tf2, *p, tf1, local, opt);
        });
  }

  if (type1 == ShapeType::Mesh && type2 == ShapeType::Mesh) {
    const auto* m1 = static_cast<const MeshShape*>(shape1);
    const auto* m2 = static_cast<const MeshShape*>(shape2);
    return collideMeshMesh(*m1, tf1, *m2, tf2, result, option);
  }

  if (type1 == ShapeType::Mesh && type2 == ShapeType::Plane) {
    const auto* mesh = static_cast<const MeshShape*>(shape1);
    const auto* plane = static_cast<const PlaneShape*>(shape2);
    return collideWithFlippedNormals(
        result,
        option,
        [&](CollisionResult& local, const CollisionOption& opt) {
          return collidePlaneMesh(*plane, tf2, *mesh, tf1, local, opt);
        });
  }

  if (type1 == ShapeType::Plane && type2 == ShapeType::Mesh) {
    const auto* plane = static_cast<const PlaneShape*>(shape1);
    const auto* mesh = static_cast<const MeshShape*>(shape2);
    return collidePlaneMesh(*plane, tf1, *mesh, tf2, result, option);
  }

  if (type1 == ShapeType::Mesh && isMeshPrimitiveType(type2)) {
    const auto* mesh = static_cast<const MeshShape*>(shape1);
    return collidePrimitiveMesh(*shape2, tf2, *mesh, tf1, result, option);
  }

  if (type2 == ShapeType::Mesh && isMeshPrimitiveType(type1)) {
    const auto* mesh = static_cast<const MeshShape*>(shape2);
    return collideWithFlippedNormals(
        result,
        option,
        [&](CollisionResult& local, const CollisionOption& opt) {
          return collidePrimitiveMesh(*shape1, tf1, *mesh, tf2, local, opt);
        });
  }

  if ((type1 == ShapeType::Convex || type2 == ShapeType::Convex)
      && isConvexFallbackType(type1) && isConvexFallbackType(type2)) {
    return collideConvexConvex(*shape1, tf1, *shape2, tf2, result, option);
  }

  return false;
}

double distanceShapes(
    const Shape* shape1,
    const Eigen::Isometry3d& tf1,
    const Shape* shape2,
    const Eigen::Isometry3d& tf2,
    const DistanceOption& option,
    DistanceResult& result)
{
  if (!shape1 || !shape2) {
    result.distance = std::numeric_limits<double>::max();
    return result.distance;
  }

  const ShapeType type1 = shape1->getType();
  const ShapeType type2 = shape2->getType();

  if (type1 == ShapeType::Compound || type2 == ShapeType::Compound) {
    double bestDistance = std::numeric_limits<double>::max();
    DistanceResult bestResult;
    bool found = false;

    auto updateBest = [&](double distance, const DistanceResult& candidate) {
      if (distance < bestDistance) {
        bestDistance = distance;
        bestResult = candidate;
        found = true;
      }
    };

    if (type1 == ShapeType::Compound && type2 == ShapeType::Compound) {
      const auto* compound1 = static_cast<const CompoundShape*>(shape1);
      const auto* compound2 = static_cast<const CompoundShape*>(shape2);
      for (const auto& child1 : compound1->children()) {
        if (!child1.shape)
          continue;

        const Eigen::Isometry3d childTf1 = tf1 * child1.localTransform;
        for (const auto& child2 : compound2->children()) {
          if (!child2.shape)
            continue;

          const Eigen::Isometry3d childTf2 = tf2 * child2.localTransform;
          DistanceResult childResult;
          const double distance = distanceShapes(
              child1.shape.get(),
              childTf1,
              child2.shape.get(),
              childTf2,
              option,
              childResult);
          updateBest(distance, childResult);
        }
      }
    } else if (type1 == ShapeType::Compound) {
      const auto* compound = static_cast<const CompoundShape*>(shape1);
      for (const auto& child : compound->children()) {
        if (!child.shape)
          continue;

        const Eigen::Isometry3d childTf = tf1 * child.localTransform;
        DistanceResult childResult;
        const double distance = distanceShapes(
            child.shape.get(), childTf, shape2, tf2, option, childResult);
        updateBest(distance, childResult);
      }
    } else {
      const auto* compound = static_cast<const CompoundShape*>(shape2);
      for (const auto& child : compound->children()) {
        if (!child.shape)
          continue;

        const Eigen::Isometry3d childTf = tf2 * child.localTransform;
        DistanceResult childResult;
        const double distance = distanceShapes(
            shape1, tf1, child.shape.get(), childTf, option, childResult);
        updateBest(distance, childResult);
      }
    }

    if (!found) {
      result.distance = std::numeric_limits<double>::max();
      return result.distance;
    }

    result = bestResult;
    result.distance = bestDistance;
    return bestDistance;
  }

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
    const double distance
        = distanceSphereSdf(*s, tf2, *sdf, tf1, result, option);
    std::swap(result.pointOnObject1, result.pointOnObject2);
    result.normal = -result.normal;
    return distance;
  }

  if (type1 == ShapeType::Sdf && type2 == ShapeType::Sdf) {
    const auto* sdf1 = static_cast<const SdfShape*>(shape1);
    const auto* sdf2 = static_cast<const SdfShape*>(shape2);
    return distanceSdfSdf(*sdf1, tf1, *sdf2, tf2, result, option);
  }

  if (type1 == ShapeType::Sphere && type2 == ShapeType::Box) {
    const auto* s = static_cast<const SphereShape*>(shape1);
    const auto* b = static_cast<const BoxShape*>(shape2);
    return distanceSphereBox(*s, tf1, *b, tf2, result, option);
  }

  if (type1 == ShapeType::Box && type2 == ShapeType::Sphere) {
    const auto* b = static_cast<const BoxShape*>(shape1);
    const auto* s = static_cast<const SphereShape*>(shape2);
    const double distance = distanceSphereBox(*s, tf2, *b, tf1, result, option);
    std::swap(result.pointOnObject1, result.pointOnObject2);
    result.normal = -result.normal;
    return distance;
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
    const double distance
        = distanceCapsuleSdf(*c, tf2, *sdf, tf1, result, option);
    std::swap(result.pointOnObject1, result.pointOnObject2);
    result.normal = -result.normal;
    return distance;
  }

  if (type1 == ShapeType::Cylinder && type2 == ShapeType::Sdf) {
    const auto* c = static_cast<const CylinderShape*>(shape1);
    const auto* sdf = static_cast<const SdfShape*>(shape2);
    return distanceCylinderSdf(*c, tf1, *sdf, tf2, result, option);
  }

  if (type1 == ShapeType::Sdf && type2 == ShapeType::Cylinder) {
    const auto* sdf = static_cast<const SdfShape*>(shape1);
    const auto* c = static_cast<const CylinderShape*>(shape2);
    const double distance
        = distanceCylinderSdf(*c, tf2, *sdf, tf1, result, option);
    std::swap(result.pointOnObject1, result.pointOnObject2);
    result.normal = -result.normal;
    return distance;
  }

  if (type1 == ShapeType::Convex && type2 == ShapeType::Sdf) {
    const auto* c = static_cast<const ConvexShape*>(shape1);
    const auto* sdf = static_cast<const SdfShape*>(shape2);
    return distanceConvexSdf(*c, tf1, *sdf, tf2, result, option);
  }

  if (type1 == ShapeType::Sdf && type2 == ShapeType::Convex) {
    const auto* sdf = static_cast<const SdfShape*>(shape1);
    const auto* c = static_cast<const ConvexShape*>(shape2);
    const double distance
        = distanceConvexSdf(*c, tf2, *sdf, tf1, result, option);
    std::swap(result.pointOnObject1, result.pointOnObject2);
    result.normal = -result.normal;
    return distance;
  }

  if (type1 == ShapeType::Mesh && type2 == ShapeType::Sdf) {
    const auto* m = static_cast<const MeshShape*>(shape1);
    const auto* sdf = static_cast<const SdfShape*>(shape2);
    return distanceMeshSdf(*m, tf1, *sdf, tf2, result, option);
  }

  if (type1 == ShapeType::Sdf && type2 == ShapeType::Mesh) {
    const auto* sdf = static_cast<const SdfShape*>(shape1);
    const auto* m = static_cast<const MeshShape*>(shape2);
    const double distance = distanceMeshSdf(*m, tf2, *sdf, tf1, result, option);
    std::swap(result.pointOnObject1, result.pointOnObject2);
    result.normal = -result.normal;
    return distance;
  }

  if (type1 == ShapeType::Capsule && type2 == ShapeType::Sphere) {
    const auto* c = static_cast<const CapsuleShape*>(shape1);
    const auto* s = static_cast<const SphereShape*>(shape2);
    return distanceCapsuleSphere(*c, tf1, *s, tf2, result, option);
  }

  if (type1 == ShapeType::Sphere && type2 == ShapeType::Capsule) {
    const auto* s = static_cast<const SphereShape*>(shape1);
    const auto* c = static_cast<const CapsuleShape*>(shape2);
    const double distance
        = distanceCapsuleSphere(*c, tf2, *s, tf1, result, option);
    std::swap(result.pointOnObject1, result.pointOnObject2);
    result.normal = -result.normal;
    return distance;
  }

  if (type1 == ShapeType::Capsule && type2 == ShapeType::Box) {
    const auto* c = static_cast<const CapsuleShape*>(shape1);
    const auto* b = static_cast<const BoxShape*>(shape2);
    return distanceCapsuleBox(*c, tf1, *b, tf2, result, option);
  }

  if (type1 == ShapeType::Box && type2 == ShapeType::Capsule) {
    const auto* b = static_cast<const BoxShape*>(shape1);
    const auto* c = static_cast<const CapsuleShape*>(shape2);
    const double distance
        = distanceCapsuleBox(*c, tf2, *b, tf1, result, option);
    std::swap(result.pointOnObject1, result.pointOnObject2);
    result.normal = -result.normal;
    return distance;
  }

  if (type1 == ShapeType::Plane && isPlaneDistanceShapeType(type2)) {
    const auto* p = static_cast<const PlaneShape*>(shape1);
    return distancePlaneShape(*p, tf1, *shape2, tf2, result, option);
  }

  if (type2 == ShapeType::Plane && isPlaneDistanceShapeType(type1)) {
    const auto* p = static_cast<const PlaneShape*>(shape2);
    const double distance
        = distancePlaneShape(*p, tf2, *shape1, tf1, result, option);
    std::swap(result.pointOnObject1, result.pointOnObject2);
    result.normal = -result.normal;
    return distance;
  }

  if ((type1 == ShapeType::Cylinder || type2 == ShapeType::Cylinder)
      && isConvexFallbackType(type1) && isConvexFallbackType(type2)) {
    return distanceConvexConvex(*shape1, tf1, *shape2, tf2, result, option);
  }

  if (type1 == ShapeType::Mesh && type2 == ShapeType::Mesh) {
    const auto* m1 = static_cast<const MeshShape*>(shape1);
    const auto* m2 = static_cast<const MeshShape*>(shape2);
    return distanceMeshMesh(*m1, tf1, *m2, tf2, result, option);
  }

  if ((type1 == ShapeType::Convex || type2 == ShapeType::Convex)
      && isConvexFallbackType(type1) && isConvexFallbackType(type2)) {
    return distanceConvexConvex(*shape1, tf1, *shape2, tf2, result, option);
  }

  result.distance = std::numeric_limits<double>::max();
  return result.distance;
}

bool raycastShape(
    const Ray& ray,
    const Shape* shape,
    const Eigen::Isometry3d& transform,
    const RaycastOption& option,
    RaycastResult& result)
{
  result.clear();

  if (!shape) {
    return false;
  }

  switch (shape->getType()) {
    case ShapeType::Sphere: {
      const auto* s = static_cast<const SphereShape*>(shape);
      return raycastSphere(ray, *s, transform, option, result);
    }
    case ShapeType::Box: {
      const auto* b = static_cast<const BoxShape*>(shape);
      return raycastBox(ray, *b, transform, option, result);
    }
    case ShapeType::Capsule: {
      const auto* c = static_cast<const CapsuleShape*>(shape);
      return raycastCapsule(ray, *c, transform, option, result);
    }
    case ShapeType::Cylinder: {
      const auto* c = static_cast<const CylinderShape*>(shape);
      return raycastCylinder(ray, *c, transform, option, result);
    }
    case ShapeType::Plane: {
      const auto* p = static_cast<const PlaneShape*>(shape);
      return raycastPlane(ray, *p, transform, option, result);
    }
    case ShapeType::Mesh: {
      const auto* m = static_cast<const MeshShape*>(shape);
      return raycastMesh(ray, *m, transform, option, result);
    }
    case ShapeType::Convex: {
      const auto* c = static_cast<const ConvexShape*>(shape);
      return raycastConvex(ray, *c, transform, option, result);
    }
    case ShapeType::Compound: {
      const auto* compound = static_cast<const CompoundShape*>(shape);
      bool hit = false;
      double bestDistance = std::min(ray.maxDistance, option.maxDistance);
      RaycastResult bestResult;

      for (const auto& child : compound->children()) {
        if (!child.shape) {
          continue;
        }

        RaycastOption childOption = option;
        childOption.maxDistance = bestDistance;
        RaycastResult childResult;
        if (raycastShape(
                ray,
                child.shape.get(),
                transform * child.localTransform,
                childOption,
                childResult)) {
          if (!hit || childResult.distance <= bestDistance) {
            bestDistance = childResult.distance;
            bestResult = childResult;
          }
          hit = true;
        }
      }

      if (hit) {
        result = bestResult;
      }
      return hit;
    }
    default:
      return false;
  }
}

bool sphereCastShape(
    const Eigen::Vector3d& sphereStart,
    const Eigen::Vector3d& sphereEnd,
    double sphereRadius,
    const Shape* shape,
    const Eigen::Isometry3d& transform,
    const CcdOption& option,
    CcdResult& result)
{
  result.clear();

  if (!shape) {
    return false;
  }

  switch (shape->getType()) {
    case ShapeType::Sphere: {
      const auto* s = static_cast<const SphereShape*>(shape);
      return sphereCastSphere(
          sphereStart, sphereEnd, sphereRadius, *s, transform, option, result);
    }
    case ShapeType::Box: {
      const auto* b = static_cast<const BoxShape*>(shape);
      return sphereCastBox(
          sphereStart, sphereEnd, sphereRadius, *b, transform, option, result);
    }
    case ShapeType::Capsule: {
      const auto* c = static_cast<const CapsuleShape*>(shape);
      return sphereCastCapsule(
          sphereStart, sphereEnd, sphereRadius, *c, transform, option, result);
    }
    case ShapeType::Cylinder: {
      const auto* c = static_cast<const CylinderShape*>(shape);
      return sphereCastCylinder(
          sphereStart, sphereEnd, sphereRadius, *c, transform, option, result);
    }
    case ShapeType::Plane: {
      const auto* p = static_cast<const PlaneShape*>(shape);
      return sphereCastPlane(
          sphereStart, sphereEnd, sphereRadius, *p, transform, option, result);
    }
    case ShapeType::Convex: {
      const auto* c = static_cast<const ConvexShape*>(shape);
      return sphereCastConvex(
          sphereStart, sphereEnd, sphereRadius, *c, transform, option, result);
    }
    case ShapeType::Mesh: {
      const auto* m = static_cast<const MeshShape*>(shape);
      return sphereCastMesh(
          sphereStart, sphereEnd, sphereRadius, *m, transform, option, result);
    }
    case ShapeType::Compound: {
      const auto* compound = static_cast<const CompoundShape*>(shape);
      bool hit = false;
      double bestTime = 2.0;
      CcdResult bestResult;

      for (const auto& child : compound->children()) {
        if (!child.shape) {
          continue;
        }

        CcdResult childResult;
        if (sphereCastShape(
                sphereStart,
                sphereEnd,
                sphereRadius,
                child.shape.get(),
                transform * child.localTransform,
                option,
                childResult)) {
          if (childResult.timeOfImpact < bestTime) {
            bestTime = childResult.timeOfImpact;
            bestResult = childResult;
          }
          hit = true;
        }
      }

      if (hit) {
        result = bestResult;
      }
      return hit;
    }
    default:
      return false;
  }
}

bool capsuleCastShape(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const Shape* shape,
    const Eigen::Isometry3d& transform,
    const CcdOption& option,
    CcdResult& result)
{
  result.clear();

  if (!shape) {
    return false;
  }

  switch (shape->getType()) {
    case ShapeType::Sphere: {
      const auto* s = static_cast<const SphereShape*>(shape);
      return capsuleCastSphere(
          capsuleStart, capsuleEnd, capsule, *s, transform, option, result);
    }
    case ShapeType::Box: {
      const auto* b = static_cast<const BoxShape*>(shape);
      return capsuleCastBox(
          capsuleStart, capsuleEnd, capsule, *b, transform, option, result);
    }
    case ShapeType::Capsule: {
      const auto* c = static_cast<const CapsuleShape*>(shape);
      return capsuleCastCapsule(
          capsuleStart, capsuleEnd, capsule, *c, transform, option, result);
    }
    case ShapeType::Plane: {
      const auto* p = static_cast<const PlaneShape*>(shape);
      return capsuleCastPlane(
          capsuleStart, capsuleEnd, capsule, *p, transform, option, result);
    }
    case ShapeType::Cylinder: {
      const auto* c = static_cast<const CylinderShape*>(shape);
      return capsuleCastCylinder(
          capsuleStart, capsuleEnd, capsule, *c, transform, option, result);
    }
    case ShapeType::Convex: {
      const auto* c = static_cast<const ConvexShape*>(shape);
      return capsuleCastConvex(
          capsuleStart, capsuleEnd, capsule, *c, transform, option, result);
    }
    case ShapeType::Mesh: {
      const auto* m = static_cast<const MeshShape*>(shape);
      return capsuleCastMesh(
          capsuleStart, capsuleEnd, capsule, *m, transform, option, result);
    }
    case ShapeType::Compound: {
      const auto* compound = static_cast<const CompoundShape*>(shape);
      bool hit = false;
      double bestTime = 2.0;
      CcdResult bestResult;

      for (const auto& child : compound->children()) {
        if (!child.shape) {
          continue;
        }

        CcdResult childResult;
        if (capsuleCastShape(
                capsuleStart,
                capsuleEnd,
                capsule,
                child.shape.get(),
                transform * child.localTransform,
                option,
                childResult)) {
          if (childResult.timeOfImpact < bestTime) {
            bestTime = childResult.timeOfImpact;
            bestResult = childResult;
          }
          hit = true;
        }
      }

      if (hit) {
        result = bestResult;
      }
      return hit;
    }
    default:
      return false;
  }
}

bool collideBatchShapes(
    span<const NarrowPhasePair> pairs,
    span<CollisionResult> results,
    span<bool> hits,
    bool recordHits,
    const CollisionOption& option)
{
  if (results.size() < pairs.size()) {
    throw std::invalid_argument(
        "NarrowPhase::collideBatch requires one result for each pair");
  }
  if (recordHits && hits.size() < pairs.size()) {
    throw std::invalid_argument(
        "NarrowPhase::collideBatch requires one hit flag for each pair");
  }

  bool anyHit = false;
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    const auto& pair = pairs[i];
    if (pair.shapeA == nullptr || pair.shapeB == nullptr) {
      throw std::invalid_argument(
          "NarrowPhase::collideBatch received a null shape");
    }

    const bool hit = collideShapes(
        pair.shapeA, pair.tfA, pair.shapeB, pair.tfB, option, results[i]);
    if (recordHits) {
      hits[i] = hit;
    }
    anyHit |= hit;
  }

  return anyHit;
}

} // namespace

bool NarrowPhase::collide(
    const Shape* shape1,
    const Eigen::Isometry3d& tf1,
    const Shape* shape2,
    const Eigen::Isometry3d& tf2,
    const CollisionOption& option,
    CollisionResult& result)
{
  return collideShapes(shape1, tf1, shape2, tf2, option, result);
}

bool NarrowPhase::collideBatch(
    span<const NarrowPhasePair> pairs,
    span<CollisionResult> results,
    const CollisionOption& option)
{
  return collideBatchShapes(pairs, results, span<bool>(), false, option);
}

bool NarrowPhase::collideBatch(
    span<const NarrowPhasePair> pairs,
    span<CollisionResult> results,
    span<bool> hits,
    const CollisionOption& option)
{
  return collideBatchShapes(pairs, results, hits, true, option);
}

double NarrowPhase::distance(
    const Shape* shape1,
    const Eigen::Isometry3d& tf1,
    const Shape* shape2,
    const Eigen::Isometry3d& tf2,
    const DistanceOption& option,
    DistanceResult& result)
{
  return distanceShapes(shape1, tf1, shape2, tf2, option, result);
}

bool NarrowPhase::raycast(
    const Ray& ray,
    const Shape* shape,
    const Eigen::Isometry3d& transform,
    const RaycastOption& option,
    RaycastResult& result)
{
  return raycastShape(ray, shape, transform, option, result);
}

bool NarrowPhase::sphereCast(
    const Eigen::Vector3d& sphereStart,
    const Eigen::Vector3d& sphereEnd,
    double sphereRadius,
    const Shape* target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result)
{
  return sphereCastShape(
      sphereStart,
      sphereEnd,
      sphereRadius,
      target,
      targetTransform,
      option,
      result);
}

bool NarrowPhase::capsuleCast(
    const Eigen::Isometry3d& capsuleStart,
    const Eigen::Isometry3d& capsuleEnd,
    const CapsuleShape& capsule,
    const Shape* target,
    const Eigen::Isometry3d& targetTransform,
    const CcdOption& option,
    CcdResult& result)
{
  return capsuleCastShape(
      capsuleStart,
      capsuleEnd,
      capsule,
      target,
      targetTransform,
      option,
      result);
}

bool NarrowPhase::isSupported(ShapeType type1, ShapeType type2)
{
  if (type1 == ShapeType::Compound && type2 == ShapeType::Compound) {
    return true;
  }
  if (type1 == ShapeType::Compound) {
    return isCompoundCollisionPeer(type2);
  }
  if (type2 == ShapeType::Compound) {
    return isCompoundCollisionPeer(type1);
  }
  if (type1 == ShapeType::Sphere && type2 == ShapeType::Sphere) {
    return true;
  }
  if (type1 == ShapeType::Box && type2 == ShapeType::Box) {
    return true;
  }
  if ((type1 == ShapeType::Plane && type2 == ShapeType::Sphere)
      || (type1 == ShapeType::Sphere && type2 == ShapeType::Plane)
      || (type1 == ShapeType::Plane && type2 == ShapeType::Box)
      || (type1 == ShapeType::Box && type2 == ShapeType::Plane)
      || (type1 == ShapeType::Plane && type2 == ShapeType::Capsule)
      || (type1 == ShapeType::Capsule && type2 == ShapeType::Plane)
      || (type1 == ShapeType::Plane && type2 == ShapeType::Convex)
      || (type1 == ShapeType::Convex && type2 == ShapeType::Plane)) {
    return true;
  }
  if ((type1 == ShapeType::Sphere && type2 == ShapeType::Box)
      || (type1 == ShapeType::Box && type2 == ShapeType::Sphere)) {
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
  if (type1 == ShapeType::Capsule && type2 == ShapeType::Capsule) {
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
  if (type1 == ShapeType::Mesh
      && (type2 == ShapeType::Mesh || type2 == ShapeType::Plane
          || isMeshPrimitiveType(type2))) {
    return true;
  }
  if (type2 == ShapeType::Mesh
      && (type1 == ShapeType::Plane || isMeshPrimitiveType(type1))) {
    return true;
  }
  if ((type1 == ShapeType::Convex || type2 == ShapeType::Convex)
      && isConvexFallbackType(type1) && isConvexFallbackType(type2)) {
    return true;
  }
  return false;
}

bool NarrowPhase::isDistanceSupported(ShapeType type1, ShapeType type2)
{
  if (type1 == ShapeType::Compound || type2 == ShapeType::Compound) {
    return true;
  }
  if (type1 == ShapeType::Sphere && type2 == ShapeType::Sphere) {
    return true;
  }
  if ((type1 == ShapeType::Sphere && type2 == ShapeType::Sdf)
      || (type1 == ShapeType::Sdf && type2 == ShapeType::Sphere)
      || (type1 == ShapeType::Sdf && type2 == ShapeType::Sdf)) {
    return true;
  }
  if ((type1 == ShapeType::Sphere && type2 == ShapeType::Box)
      || (type1 == ShapeType::Box && type2 == ShapeType::Sphere)
      || (type1 == ShapeType::Box && type2 == ShapeType::Box)) {
    return true;
  }
  if (type1 == ShapeType::Capsule && type2 == ShapeType::Capsule) {
    return true;
  }
  if ((type1 == ShapeType::Capsule && type2 == ShapeType::Sdf)
      || (type1 == ShapeType::Sdf && type2 == ShapeType::Capsule)
      || (type1 == ShapeType::Cylinder && type2 == ShapeType::Sdf)
      || (type1 == ShapeType::Sdf && type2 == ShapeType::Cylinder)
      || (type1 == ShapeType::Convex && type2 == ShapeType::Sdf)
      || (type1 == ShapeType::Sdf && type2 == ShapeType::Convex)
      || (type1 == ShapeType::Mesh && type2 == ShapeType::Sdf)
      || (type1 == ShapeType::Sdf && type2 == ShapeType::Mesh)) {
    return true;
  }
  if ((type1 == ShapeType::Capsule && type2 == ShapeType::Sphere)
      || (type1 == ShapeType::Sphere && type2 == ShapeType::Capsule)
      || (type1 == ShapeType::Capsule && type2 == ShapeType::Box)
      || (type1 == ShapeType::Box && type2 == ShapeType::Capsule)) {
    return true;
  }
  if ((type1 == ShapeType::Plane && isPlaneDistanceShapeType(type2))
      || (type2 == ShapeType::Plane && isPlaneDistanceShapeType(type1))) {
    return true;
  }
  if ((type1 == ShapeType::Cylinder || type2 == ShapeType::Cylinder)
      && isConvexFallbackType(type1) && isConvexFallbackType(type2)) {
    return true;
  }
  if (type1 == ShapeType::Mesh && type2 == ShapeType::Mesh) {
    return true;
  }
  if ((type1 == ShapeType::Convex || type2 == ShapeType::Convex)
      && isConvexFallbackType(type1) && isConvexFallbackType(type2)) {
    return true;
  }
  return false;
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
    case ShapeType::Compound:
      return true;
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
    case ShapeType::Compound:
      return true;
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
    case ShapeType::Compound:
      return true;
    default:
      return false;
  }
}

} // namespace dart::collision::native
