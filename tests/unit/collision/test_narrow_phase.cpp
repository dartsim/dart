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

#include <dart/collision/native/collision_object.hpp>
#include <dart/collision/native/collision_world.hpp>
#include <dart/collision/native/narrow_phase/narrow_phase.hpp>
#include <dart/collision/native/sdf/dense_sdf_field.hpp>
#include <dart/collision/native/shapes/shape.hpp>

#include <gtest/gtest.h>

#include <array>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <vector>

#include <cmath>

using namespace dart::collision::native;

namespace {

std::vector<Eigen::Vector3d> makeOctahedronVertices(double scale = 1.0)
{
  return {
      {scale, 0.0, 0.0},
      {-scale, 0.0, 0.0},
      {0.0, scale, 0.0},
      {0.0, -scale, 0.0},
      {0.0, 0.0, scale},
      {0.0, 0.0, -scale}};
}

std::vector<Eigen::Vector3d> makeCubeMeshVertices(double halfExtent = 1.0)
{
  const double h = halfExtent;
  return {
      {-h, -h, -h},
      {h, -h, -h},
      {h, h, -h},
      {-h, h, -h},
      {-h, -h, h},
      {h, -h, h},
      {h, h, h},
      {-h, h, h}};
}

std::vector<MeshShape::Triangle> makeCubeMeshTriangles()
{
  return {
      {0, 2, 1},
      {0, 3, 2},
      {4, 5, 6},
      {4, 6, 7},
      {0, 1, 5},
      {0, 5, 4},
      {2, 3, 7},
      {2, 7, 6},
      {0, 4, 7},
      {0, 7, 3},
      {1, 2, 6},
      {1, 6, 5}};
}

Eigen::Isometry3d translated(double x, double y = 0.0, double z = 0.0)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(x, y, z);
  return tf;
}

void expectVectorExactlyEqual(
    const Eigen::Vector3d& expected, const Eigen::Vector3d& actual)
{
  EXPECT_EQ(expected.x(), actual.x());
  EXPECT_EQ(expected.y(), actual.y());
  EXPECT_EQ(expected.z(), actual.z());
}

void expectVectorNear(
    const Eigen::Vector3d& actual,
    const Eigen::Vector3d& expected,
    double tolerance)
{
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

void expectContactExactlyEqual(
    const ContactPoint& expected, const ContactPoint& actual)
{
  expectVectorExactlyEqual(expected.position, actual.position);
  expectVectorExactlyEqual(expected.normal, actual.normal);
  EXPECT_EQ(expected.depth, actual.depth);
  EXPECT_EQ(expected.object1, actual.object1);
  EXPECT_EQ(expected.object2, actual.object2);
  EXPECT_EQ(expected.featureIndex1, actual.featureIndex1);
  EXPECT_EQ(expected.featureIndex2, actual.featureIndex2);
}

void expectCollisionResultExactlyEqual(
    const CollisionResult& expected, const CollisionResult& actual)
{
  ASSERT_EQ(expected.numManifolds(), actual.numManifolds());
  ASSERT_EQ(expected.numContacts(), actual.numContacts());

  for (std::size_t i = 0; i < expected.numManifolds(); ++i) {
    const auto& expectedManifold = expected.getManifold(i);
    const auto& actualManifold = actual.getManifold(i);
    EXPECT_EQ(expectedManifold.getType(), actualManifold.getType());
    EXPECT_EQ(expectedManifold.getObject1(), actualManifold.getObject1());
    EXPECT_EQ(expectedManifold.getObject2(), actualManifold.getObject2());
    ASSERT_EQ(expectedManifold.numContacts(), actualManifold.numContacts());
    for (std::size_t j = 0; j < expectedManifold.numContacts(); ++j) {
      expectContactExactlyEqual(
          expectedManifold.getContact(j), actualManifold.getContact(j));
    }
  }
}

void expectSingleContactPairOrder(
    const CollisionResult& forward,
    const CollisionResult& swapped,
    double tolerance)
{
  ASSERT_EQ(forward.numContacts(), 1u);
  ASSERT_EQ(swapped.numContacts(), 1u);

  const ContactPoint& forwardContact = forward.getContact(0);
  const ContactPoint& swappedContact = swapped.getContact(0);

  EXPECT_NEAR(forwardContact.depth, swappedContact.depth, tolerance);
  expectVectorNear(forwardContact.position, swappedContact.position, tolerance);
  expectVectorNear(forwardContact.normal, -swappedContact.normal, tolerance);
  EXPECT_TRUE(forwardContact.position.allFinite());
  EXPECT_TRUE(swappedContact.position.allFinite());
  EXPECT_TRUE(forwardContact.normal.allFinite());
  EXPECT_TRUE(swappedContact.normal.allFinite());
}

void expectFiniteCollisionResult(const CollisionResult& result)
{
  for (std::size_t i = 0; i < result.numContacts(); ++i) {
    const ContactPoint& contact = result.getContact(i);
    EXPECT_TRUE(contact.position.allFinite());
    EXPECT_TRUE(contact.normal.allFinite());
    EXPECT_TRUE(std::isfinite(contact.depth));
  }
}

} // namespace

TEST(NarrowPhase, IsSupported)
{
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Sphere, ShapeType::Sphere));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Box, ShapeType::Box));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Sphere, ShapeType::Box));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Box, ShapeType::Sphere));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Capsule, ShapeType::Capsule));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Capsule, ShapeType::Sphere));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Sphere, ShapeType::Capsule));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Capsule, ShapeType::Box));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Box, ShapeType::Capsule));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Plane, ShapeType::Sphere));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Sphere, ShapeType::Plane));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Plane, ShapeType::Box));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Box, ShapeType::Plane));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Plane, ShapeType::Capsule));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Capsule, ShapeType::Plane));
  EXPECT_TRUE(
      NarrowPhase::isSupported(ShapeType::Cylinder, ShapeType::Cylinder));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Cylinder, ShapeType::Sphere));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Sphere, ShapeType::Cylinder));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Cylinder, ShapeType::Box));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Box, ShapeType::Cylinder));
  EXPECT_TRUE(
      NarrowPhase::isSupported(ShapeType::Cylinder, ShapeType::Capsule));
  EXPECT_TRUE(
      NarrowPhase::isSupported(ShapeType::Capsule, ShapeType::Cylinder));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Cylinder, ShapeType::Plane));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Plane, ShapeType::Cylinder));

  // Convex and Mesh types are supported via GJK
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Convex, ShapeType::Convex));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Convex, ShapeType::Sphere));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Mesh, ShapeType::Mesh));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Mesh, ShapeType::Sphere));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Mesh, ShapeType::Convex));
}

TEST(NarrowPhase, IsDistanceSupported)
{
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Sphere, ShapeType::Sphere));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Sphere, ShapeType::Sdf));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Sdf, ShapeType::Sphere));
  EXPECT_TRUE(NarrowPhase::isDistanceSupported(ShapeType::Sdf, ShapeType::Sdf));
  EXPECT_TRUE(NarrowPhase::isDistanceSupported(ShapeType::Box, ShapeType::Box));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Sphere, ShapeType::Box));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Capsule, ShapeType::Capsule));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Capsule, ShapeType::Sdf));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Cylinder, ShapeType::Sdf));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Convex, ShapeType::Sdf));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Mesh, ShapeType::Sdf));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Capsule, ShapeType::Sphere));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Capsule, ShapeType::Box));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Plane, ShapeType::Sphere));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Plane, ShapeType::Mesh));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Cylinder, ShapeType::Box));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Convex, ShapeType::Box));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Mesh, ShapeType::Sphere));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Compound, ShapeType::Plane));

  EXPECT_FALSE(
      NarrowPhase::isDistanceSupported(ShapeType::Plane, ShapeType::Sdf));
}

TEST(NarrowPhase, DispatchesSupportedCollisionPairs)
{
  SphereShape sphere(0.75);
  BoxShape box(Eigen::Vector3d::Constant(1.0));
  CapsuleShape capsule(0.45, 1.0);
  CylinderShape cylinder(0.5, 1.0);
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  ConvexShape convex(makeOctahedronVertices(0.75));
  MeshShape mesh(makeCubeMeshVertices(0.75), makeCubeMeshTriangles());

  CompoundShape compoundA;
  compoundA.addChild(std::make_unique<SphereShape>(0.5));
  CompoundShape compoundB;
  compoundB.addChild(std::make_unique<BoxShape>(Eigen::Vector3d::Ones()));

  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  const std::array<std::pair<const Shape*, const Shape*>, 36> supportedPairs{{
      {&sphere, &sphere},       {&box, &box},
      {&sphere, &box},          {&box, &sphere},
      {&capsule, &capsule},     {&capsule, &sphere},
      {&sphere, &capsule},      {&capsule, &box},
      {&box, &capsule},         {&plane, &sphere},
      {&sphere, &plane},        {&plane, &box},
      {&box, &plane},           {&plane, &capsule},
      {&capsule, &plane},       {&cylinder, &cylinder},
      {&cylinder, &sphere},     {&sphere, &cylinder},
      {&cylinder, &box},        {&box, &cylinder},
      {&cylinder, &capsule},    {&capsule, &cylinder},
      {&cylinder, &plane},      {&plane, &cylinder},
      {&plane, &convex},        {&convex, &plane},
      {&mesh, &mesh},           {&mesh, &plane},
      {&plane, &mesh},          {&mesh, &sphere},
      {&sphere, &mesh},         {&convex, &convex},
      {&convex, &sphere},       {&sphere, &convex},
      {&compoundA, &compoundB}, {&compoundA, &sphere},
  }};

  CollisionOption option;
  option.maxNumContacts = 8;
  for (const auto& pair : supportedPairs) {
    SCOPED_TRACE(
        static_cast<int>(pair.first->getType()) * 100
        + static_cast<int>(pair.second->getType()));
    EXPECT_TRUE(
        NarrowPhase::isSupported(
            pair.first->getType(), pair.second->getType()));

    CollisionResult result;
    EXPECT_TRUE(
        NarrowPhase::collide(
            pair.first, identity, pair.second, identity, option, result));
    EXPECT_GT(result.numContacts(), 0u);
  }

  CollisionResult result;
  EXPECT_FALSE(
      NarrowPhase::collide(
          nullptr, identity, &sphere, identity, option, result));
  EXPECT_FALSE(
      NarrowPhase::collide(&plane, identity, &plane, identity, option, result));
}

TEST(NarrowPhase, CompoundCollisionTraversesChildrenAndEarlyStops)
{
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  SphereShape sphere(0.5);
  BoxShape box(Eigen::Vector3d::Ones());

  CompoundShape compoundA;
  compoundA.addChild(nullptr);
  compoundA.addChild(std::make_unique<SphereShape>(0.5));

  CompoundShape compoundB;
  compoundB.addChild(nullptr);
  compoundB.addChild(std::make_unique<SphereShape>(0.5), translated(0.75));

  CollisionResult compoundPair;
  EXPECT_TRUE(
      NarrowPhase::collide(
          &compoundA,
          identity,
          &compoundB,
          identity,
          CollisionOption::binaryCheck(),
          compoundPair));
  EXPECT_EQ(compoundPair.numContacts(), 0u);

  CollisionOption option;
  option.maxNumContacts = 1;

  CollisionResult compoundShape;
  EXPECT_TRUE(
      NarrowPhase::collide(
          &compoundA,
          identity,
          &sphere,
          translated(0.75),
          option,
          compoundShape));
  EXPECT_EQ(compoundShape.numContacts(), 1u);

  CollisionResult shapeCompound;
  EXPECT_TRUE(
      NarrowPhase::collide(
          &sphere,
          translated(0.75),
          &compoundA,
          identity,
          option,
          shapeCompound));
  EXPECT_EQ(shapeCompound.numContacts(), 1u);

  CompoundShape emptyCompound;
  emptyCompound.addChild(nullptr);
  CollisionResult noChildHit;
  EXPECT_FALSE(
      NarrowPhase::collide(
          &sphere, identity, &emptyCompound, identity, option, noChildHit));

  CollisionResult prefilled;
  prefilled.addContact(ContactPoint{});
  EXPECT_FALSE(
      NarrowPhase::collide(
          &sphere, identity, &box, identity, option, prefilled));
}

TEST(NarrowPhase, DispatchesSupportedDistancePairs)
{
  auto makeDenseField = []() {
    const Eigen::Vector3d origin = Eigen::Vector3d::Constant(-0.5);
    const Eigen::Vector3i dims(4, 4, 4);
    auto field = std::make_shared<DenseSdfField>(origin, dims, 0.25, 10.0);
    for (int z = 0; z < dims.z(); ++z) {
      for (int y = 0; y < dims.y(); ++y) {
        for (int x = 0; x < dims.x(); ++x) {
          const Eigen::Vector3i index(x, y, z);
          const Eigen::Vector3d point
              = origin + Eigen::Vector3d(x, y, z) * 0.25;
          field->setDistance(index, point.norm() - 0.25);
          field->setObserved(index, true);
        }
      }
    }
    return field;
  };

  auto expectFiniteDistance = [](auto makeShapeA, auto makeShapeB) {
    CollisionWorld world;
    auto objA = world.createObject(makeShapeA());
    auto objB = world.createObject(makeShapeB());
    ASSERT_TRUE(objA.isValid());
    ASSERT_TRUE(objB.isValid());

    DistanceResult result;
    const double distance = NarrowPhase::distance(
        objA, objB, DistanceOption::unlimited(), result);
    EXPECT_TRUE(std::isfinite(distance));
    EXPECT_TRUE(result.pointOnObject1.allFinite());
    EXPECT_TRUE(result.pointOnObject2.allFinite());
    EXPECT_TRUE(result.normal.allFinite());
  };

  auto sphere = []() {
    return std::make_unique<SphereShape>(0.5);
  };
  auto box = []() {
    return std::make_unique<BoxShape>(Eigen::Vector3d::Constant(1.0));
  };
  auto capsule = []() {
    return std::make_unique<CapsuleShape>(0.35, 0.8);
  };
  auto cylinder = []() {
    return std::make_unique<CylinderShape>(0.4, 0.9);
  };
  auto plane = []() {
    return std::make_unique<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0);
  };
  auto convex = []() {
    return std::make_unique<ConvexShape>(makeOctahedronVertices(0.5));
  };
  auto mesh = []() {
    return std::make_unique<MeshShape>(
        makeCubeMeshVertices(0.5), makeCubeMeshTriangles());
  };
  auto sdf = [&]() {
    return std::make_unique<SdfShape>(makeDenseField());
  };
  auto compound = []() {
    auto shape = std::make_unique<CompoundShape>();
    shape->addChild(std::make_unique<SphereShape>(0.4));
    return shape;
  };

  expectFiniteDistance(sphere, sphere);
  expectFiniteDistance(sphere, box);
  expectFiniteDistance(box, sphere);
  expectFiniteDistance(box, box);
  expectFiniteDistance(capsule, capsule);
  expectFiniteDistance(capsule, sphere);
  expectFiniteDistance(sphere, capsule);
  expectFiniteDistance(capsule, box);
  expectFiniteDistance(box, capsule);
  expectFiniteDistance(plane, sphere);
  expectFiniteDistance(sphere, plane);
  expectFiniteDistance(plane, box);
  expectFiniteDistance(box, plane);
  expectFiniteDistance(plane, capsule);
  expectFiniteDistance(capsule, plane);
  expectFiniteDistance(plane, cylinder);
  expectFiniteDistance(cylinder, plane);
  expectFiniteDistance(plane, mesh);
  expectFiniteDistance(mesh, plane);
  expectFiniteDistance(cylinder, cylinder);
  expectFiniteDistance(cylinder, sphere);
  expectFiniteDistance(sphere, cylinder);
  expectFiniteDistance(cylinder, box);
  expectFiniteDistance(box, cylinder);
  expectFiniteDistance(cylinder, capsule);
  expectFiniteDistance(capsule, cylinder);
  expectFiniteDistance(mesh, mesh);
  expectFiniteDistance(convex, sphere);
  expectFiniteDistance(sphere, convex);
  expectFiniteDistance(mesh, convex);
  expectFiniteDistance(convex, mesh);
  expectFiniteDistance(sphere, sdf);
  expectFiniteDistance(sdf, sphere);
  expectFiniteDistance(capsule, sdf);
  expectFiniteDistance(sdf, capsule);
  expectFiniteDistance(cylinder, sdf);
  expectFiniteDistance(sdf, cylinder);
  expectFiniteDistance(convex, sdf);
  expectFiniteDistance(sdf, convex);
  expectFiniteDistance(mesh, sdf);
  expectFiniteDistance(sdf, mesh);
  expectFiniteDistance(sdf, sdf);
  expectFiniteDistance(compound, sphere);
  expectFiniteDistance(sphere, compound);
  expectFiniteDistance(compound, compound);

  CollisionWorld world;
  auto objA = world.createObject(
      std::make_unique<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));
  auto objB = world.createObject(
      std::make_unique<PlaneShape>(Eigen::Vector3d::UnitZ(), 1.0));
  DistanceResult unsupported;
  EXPECT_EQ(
      NarrowPhase::distance(
          objA, objB, DistanceOption::unlimited(), unsupported),
      std::numeric_limits<double>::max());
}

TEST(NarrowPhase, CompoundDistanceHandlesEmptyAndNullChildren)
{
  CollisionWorld world;
  auto sphere = world.createObject(std::make_unique<SphereShape>(0.5));

  auto emptyCompoundShape = std::make_unique<CompoundShape>();
  emptyCompoundShape->addChild(nullptr);
  auto emptyCompound = world.createObject(std::move(emptyCompoundShape));

  DistanceResult missing;
  EXPECT_EQ(
      NarrowPhase::distance(
          emptyCompound, sphere, DistanceOption::unlimited(), missing),
      std::numeric_limits<double>::max());

  auto compoundShape = std::make_unique<CompoundShape>();
  compoundShape->addChild(nullptr);
  compoundShape->addChild(std::make_unique<SphereShape>(0.5), translated(0.75));
  compoundShape->addChild(std::make_unique<SphereShape>(0.5), translated(2.0));
  auto compound = world.createObject(std::move(compoundShape));

  DistanceResult forward;
  const double distanceForward = NarrowPhase::distance(
      compound, sphere, DistanceOption::unlimited(), forward);
  EXPECT_LT(distanceForward, 0.0);
  EXPECT_EQ(forward.object1, &compound);
  EXPECT_EQ(forward.object2, &sphere);

  DistanceResult reverse;
  const double distanceReverse = NarrowPhase::distance(
      sphere, compound, DistanceOption::unlimited(), reverse);
  EXPECT_NEAR(distanceReverse, distanceForward, 1e-12);
  EXPECT_EQ(reverse.object1, &sphere);
  EXPECT_EQ(reverse.object2, &compound);
}

TEST(NarrowPhase, CompoundRayAndCastQueriesPickClosestChild)
{
  auto compoundShape = std::make_unique<CompoundShape>();
  compoundShape->addChild(nullptr);
  compoundShape->addChild(
      std::make_unique<SphereShape>(0.5), translated(0, 0, 5));
  compoundShape->addChild(
      std::make_unique<SphereShape>(0.5), translated(0, 0, 2));

  CollisionWorld world;
  auto target = world.createObject(std::move(compoundShape));

  RaycastResult raycast;
  ASSERT_TRUE(
      NarrowPhase::raycast(
          Ray(Eigen::Vector3d(0.0, 0.0, -2.0), Eigen::Vector3d::UnitZ()),
          target,
          RaycastOption::unlimited(),
          raycast));
  EXPECT_EQ(raycast.object, &target);
  EXPECT_LT(raycast.distance, 4.0);

  CcdResult sphereCast;
  ASSERT_TRUE(
      NarrowPhase::sphereCast(
          Eigen::Vector3d(0.0, 0.0, -2.0),
          Eigen::Vector3d(0.0, 0.0, 6.0),
          0.25,
          target,
          CcdOption::standard(),
          sphereCast));
  EXPECT_EQ(sphereCast.object, &target);
  EXPECT_LT(sphereCast.timeOfImpact, 0.5);

  CapsuleShape capsule(0.2, 0.5);
  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  capsuleStart.translation() = Eigen::Vector3d(0.0, 0.0, -2.0);
  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(0.0, 0.0, 6.0);

  CcdResult capsuleCast;
  ASSERT_TRUE(
      NarrowPhase::capsuleCast(
          capsuleStart,
          capsuleEnd,
          capsule,
          target,
          CcdOption::standard(),
          capsuleCast));
  EXPECT_EQ(capsuleCast.object, &target);
  EXPECT_LT(capsuleCast.timeOfImpact, 0.5);

  EXPECT_TRUE(NarrowPhase::isRaycastSupported(ShapeType::Compound));
  EXPECT_FALSE(NarrowPhase::isRaycastSupported(ShapeType::Sdf));
  EXPECT_TRUE(NarrowPhase::isSphereCastSupported(ShapeType::Compound));
  EXPECT_FALSE(NarrowPhase::isSphereCastSupported(ShapeType::Sdf));
  EXPECT_TRUE(NarrowPhase::isCapsuleCastSupported(ShapeType::Compound));
  EXPECT_FALSE(NarrowPhase::isCapsuleCastSupported(ShapeType::Sdf));
}

TEST(NarrowPhase, RayAndCastDispatchReachPrimitiveTargets)
{
  auto expectRayHit = [](const char* name, std::unique_ptr<Shape> shape) {
    SCOPED_TRACE(name);
    CollisionWorld world;
    auto target = world.createObject(std::move(shape));
    RaycastResult result;
    EXPECT_TRUE(
        NarrowPhase::raycast(
            Ray(Eigen::Vector3d(0.0, 0.0, -3.0), Eigen::Vector3d::UnitZ(), 8.0),
            target,
            RaycastOption::unlimited(),
            result));
    EXPECT_EQ(result.object, &target);
  };

  expectRayHit("capsule ray", std::make_unique<CapsuleShape>(0.4, 1.0));
  expectRayHit("cylinder ray", std::make_unique<CylinderShape>(0.4, 1.0));
  expectRayHit(
      "mesh ray",
      std::make_unique<MeshShape>(
          makeCubeMeshVertices(0.5), makeCubeMeshTriangles()));
  expectRayHit(
      "convex ray", std::make_unique<ConvexShape>(makeOctahedronVertices(0.5)));

  {
    SCOPED_TRACE("plane ray");
    CollisionWorld world;
    auto target = world.createObject(
        std::make_unique<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));
    RaycastResult result;
    EXPECT_TRUE(
        NarrowPhase::raycast(
            Ray(Eigen::Vector3d(0.0, 0.0, 3.0), -Eigen::Vector3d::UnitZ(), 8.0),
            target,
            RaycastOption::unlimited(),
            result));
    EXPECT_EQ(result.object, &target);
  }

  auto expectSphereCastHit = [](std::unique_ptr<Shape> shape) {
    CollisionWorld world;
    auto target = world.createObject(std::move(shape));
    CcdResult result;
    EXPECT_TRUE(
        NarrowPhase::sphereCast(
            Eigen::Vector3d(0.0, 0.0, -3.0),
            Eigen::Vector3d(0.0, 0.0, 3.0),
            0.1,
            target,
            CcdOption::standard(),
            result));
    EXPECT_EQ(result.object, &target);
  };

  expectSphereCastHit(std::make_unique<CapsuleShape>(0.4, 1.0));
  expectSphereCastHit(std::make_unique<CylinderShape>(0.4, 1.0));
  expectSphereCastHit(
      std::make_unique<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));
  expectSphereCastHit(
      std::make_unique<MeshShape>(
          makeCubeMeshVertices(0.5), makeCubeMeshTriangles()));
  expectSphereCastHit(
      std::make_unique<ConvexShape>(makeOctahedronVertices(0.5)));

  auto expectCapsuleCastHit = [](std::unique_ptr<Shape> shape) {
    CollisionWorld world;
    auto target = world.createObject(std::move(shape));
    CapsuleShape capsule(0.1, 0.4);
    Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
    start.translation() = Eigen::Vector3d(0.0, 0.0, -3.0);
    Eigen::Isometry3d end = Eigen::Isometry3d::Identity();
    end.translation() = Eigen::Vector3d(0.0, 0.0, 3.0);

    CcdResult result;
    EXPECT_TRUE(
        NarrowPhase::capsuleCast(
            start, end, capsule, target, CcdOption::standard(), result));
    EXPECT_EQ(result.object, &target);
  };

  expectCapsuleCastHit(std::make_unique<SphereShape>(0.5));
  expectCapsuleCastHit(
      std::make_unique<BoxShape>(Eigen::Vector3d::Ones() * 0.5));
  expectCapsuleCastHit(std::make_unique<CapsuleShape>(0.4, 1.0));
  expectCapsuleCastHit(
      std::make_unique<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));
  expectCapsuleCastHit(std::make_unique<CylinderShape>(0.4, 1.0));
  expectCapsuleCastHit(
      std::make_unique<ConvexShape>(makeOctahedronVertices(0.5)));
  expectCapsuleCastHit(
      std::make_unique<MeshShape>(
          makeCubeMeshVertices(0.5), makeCubeMeshTriangles()));

  CollisionObject invalid;
  RaycastResult raycast;
  EXPECT_FALSE(
      NarrowPhase::raycast(
          Ray(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ()),
          invalid,
          RaycastOption::unlimited(),
          raycast));

  CcdResult sphereCast;
  EXPECT_FALSE(
      NarrowPhase::sphereCast(
          Eigen::Vector3d::Zero(),
          Eigen::Vector3d::UnitZ(),
          0.1,
          invalid,
          CcdOption::standard(),
          sphereCast));

  CcdResult capsuleCast;
  const CapsuleShape capsule(0.1, 0.4);
  EXPECT_FALSE(
      NarrowPhase::capsuleCast(
          Eigen::Isometry3d::Identity(),
          Eigen::Isometry3d::Identity(),
          capsule,
          invalid,
          CcdOption::standard(),
          capsuleCast));

  CollisionWorld sdfWorld;
  auto sdfTarget = sdfWorld.createObject(
      std::make_unique<SdfShape>(std::make_shared<DenseSdfField>(
          Eigen::Vector3d::Constant(-0.5), Eigen::Vector3i(2, 2, 2), 1.0)));
  ASSERT_TRUE(sdfTarget.isValid());

  EXPECT_FALSE(
      NarrowPhase::raycast(
          Ray(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ()),
          sdfTarget,
          RaycastOption::unlimited(),
          raycast));
  EXPECT_FALSE(
      NarrowPhase::sphereCast(
          Eigen::Vector3d::Zero(),
          Eigen::Vector3d::UnitZ(),
          0.1,
          sdfTarget,
          CcdOption::standard(),
          sphereCast));
  EXPECT_FALSE(
      NarrowPhase::capsuleCast(
          Eigen::Isometry3d::Identity(),
          Eigen::Isometry3d::Identity(),
          capsule,
          sdfTarget,
          CcdOption::standard(),
          capsuleCast));
}

TEST(NarrowPhase, PrimitiveDispatcherPreservesScaleContactsUnderPairOrder)
{
  CollisionOption option;
  option.maxNumContacts = 1;

  {
    const SphereShape sphere(0.01);
    const BoxShape box(Eigen::Vector3d(15.0, 1.0, 1.0));
    const Eigen::Isometry3d boxTransform = Eigen::Isometry3d::Identity();
    const Eigen::Isometry3d sphereTransform
        = translated(15.0 * 0.95, 0.0, 1.0 + 0.01 * 0.5);

    CollisionResult sphereBox;
    EXPECT_TRUE(
        NarrowPhase::collide(
            &sphere, sphereTransform, &box, boxTransform, option, sphereBox));

    CollisionResult boxSphere;
    EXPECT_TRUE(
        NarrowPhase::collide(
            &box, boxTransform, &sphere, sphereTransform, option, boxSphere));

    expectSingleContactPairOrder(sphereBox, boxSphere, 1e-9);
    EXPECT_NEAR(sphereBox.getContact(0).depth, 0.005, 1e-9);
  }

  {
    const CylinderShape cylinder(9.0, 0.1);
    const SphereShape sphere(0.025);
    const Eigen::Vector3d obliqueRadial
        = Eigen::Vector3d(1.0, 2.0, 0.0).normalized();
    const Eigen::Vector3d sphereCenter
        = obliqueRadial * (cylinder.getRadius() - sphere.getRadius())
          + Eigen::Vector3d::UnitZ()
                * (0.5 * cylinder.getHeight() + 0.5 * sphere.getRadius());

    const Eigen::Isometry3d cylinderTransform = translated(1.0, -2.0, 0.5);
    const Eigen::Isometry3d sphereTransform
        = cylinderTransform
          * translated(sphereCenter.x(), sphereCenter.y(), sphereCenter.z());

    CollisionResult cylinderSphere;
    EXPECT_TRUE(
        NarrowPhase::collide(
            &cylinder,
            cylinderTransform,
            &sphere,
            sphereTransform,
            option,
            cylinderSphere));

    CollisionResult sphereCylinder;
    EXPECT_TRUE(
        NarrowPhase::collide(
            &sphere,
            sphereTransform,
            &cylinder,
            cylinderTransform,
            option,
            sphereCylinder));

    expectSingleContactPairOrder(cylinderSphere, sphereCylinder, 1e-9);
    EXPECT_NEAR(
        cylinderSphere.getContact(0).depth, 0.5 * sphere.getRadius(), 1e-9);
  }

  {
    const CapsuleShape capsule(50.0, 200.0);
    const SphereShape sphere(50.0);

    Eigen::Isometry3d capsuleTransform = Eigen::Isometry3d::Identity();
    capsuleTransform.linear()
        = Eigen::AngleAxisd(
              std::numbers::pi_v<double> * 0.5, Eigen::Vector3d::UnitX())
              .toRotationMatrix();
    capsuleTransform.translation() = Eigen::Vector3d(0.0, 50.0, 75.0);

    const Eigen::Isometry3d sphereTransform = Eigen::Isometry3d::Identity();

    CollisionResult capsuleSphere;
    EXPECT_TRUE(
        NarrowPhase::collide(
            &capsule,
            capsuleTransform,
            &sphere,
            sphereTransform,
            option,
            capsuleSphere));

    CollisionResult sphereCapsule;
    EXPECT_TRUE(
        NarrowPhase::collide(
            &sphere,
            sphereTransform,
            &capsule,
            capsuleTransform,
            option,
            sphereCapsule));

    expectSingleContactPairOrder(capsuleSphere, sphereCapsule, 1e-9);
    EXPECT_NEAR(capsuleSphere.getContact(0).depth, 25.0, 1e-9);
  }
}

TEST(NarrowPhase, collide_batch_dispatcher_matches_scalar_loop)
{
  SphereShape sphereA(0.75);
  SphereShape sphereB(0.60);
  BoxShape boxA(Eigen::Vector3d(0.5, 0.4, 0.3));
  BoxShape boxB(Eigen::Vector3d(0.4, 0.5, 0.3));
  ConvexShape convexA(makeOctahedronVertices(0.60));
  ConvexShape convexB(makeOctahedronVertices(0.55));
  MeshShape meshA(makeCubeMeshVertices(1.0), makeCubeMeshTriangles());
  MeshShape meshB(makeCubeMeshVertices(1.0), makeCubeMeshTriangles());

  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  const std::vector<NarrowPhasePair> pairs{
      {&sphereA, &sphereB, identity, translated(0.85)},
      {&boxA, &boxB, translated(0.0, 0.1), translated(0.65, -0.1)},
      {&convexA, &convexB, identity, translated(0.65)},
      {&meshA, &meshB, identity, translated(1.20)},
      {&sphereA, &sphereB, identity, translated(3.0)}};

  CollisionOption option;
  option.maxNumContacts = 8;

  std::vector<CollisionResult> batchResults(pairs.size());
  const bool batchHit = NarrowPhase::collideBatch(pairs, batchResults, option);

  bool scalarAnyHit = false;
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    CollisionResult scalarResult;
    const bool scalarHit = NarrowPhase::collide(
        pairs[i].shapeA,
        pairs[i].tfA,
        pairs[i].shapeB,
        pairs[i].tfB,
        option,
        scalarResult);
    scalarAnyHit |= scalarHit;
    expectCollisionResultExactlyEqual(scalarResult, batchResults[i]);
  }

  EXPECT_EQ(scalarAnyHit, batchHit);
}

TEST(NarrowPhase, collide_batch_dispatcher_reports_hit_flags)
{
  SphereShape sphereA(0.75);
  SphereShape sphereB(0.60);

  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  const std::array<NarrowPhasePair, 3> pairs{
      NarrowPhasePair{&sphereA, &sphereB, identity, translated(0.85)},
      NarrowPhasePair{&sphereA, &sphereB, identity, translated(3.0)},
      NarrowPhasePair{&sphereA, &sphereB, identity, translated(0.50)}};

  CollisionOption option;
  std::array<CollisionResult, pairs.size()> results;
  std::array<bool, pairs.size()> hits{};

  const bool anyHit = NarrowPhase::collideBatch(pairs, results, hits, option);

  EXPECT_TRUE(anyHit);
  EXPECT_TRUE(hits[0]);
  EXPECT_FALSE(hits[1]);
  EXPECT_TRUE(hits[2]);
  EXPECT_GE(results[0].numContacts(), 1u);
  EXPECT_EQ(results[1].numContacts(), 0u);
  EXPECT_GE(results[2].numContacts(), 1u);
}

TEST(NarrowPhase, ZeroExtentShapesNoCrashAcrossBatch)
{
  SphereShape zeroSphere(0.0);
  SphereShape unitSphere(1.0);
  BoxShape zeroBox(Eigen::Vector3d::Zero());
  BoxShape unitBox(Eigen::Vector3d::Ones());
  CapsuleShape zeroCapsule(0.0, 0.0);
  CylinderShape zeroCylinder(0.0, 0.0);
  ConvexShape pointConvex({Eigen::Vector3d::Zero()});
  ConvexShape unitConvex(makeOctahedronVertices(1.0));

  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  const std::array<NarrowPhasePair, 10> cases{{
      {&zeroSphere, &unitSphere, identity, identity},
      {&zeroSphere, &unitBox, identity, identity},
      {&zeroBox, &unitSphere, identity, identity},
      {&zeroBox, &unitBox, identity, identity},
      {&zeroCapsule, &unitSphere, identity, identity},
      {&zeroCapsule, &unitBox, identity, identity},
      {&zeroCylinder, &unitSphere, identity, identity},
      {&zeroCylinder, &unitBox, identity, identity},
      {&pointConvex, &unitSphere, identity, identity},
      {&pointConvex, &unitConvex, identity, identity},
  }};

  std::vector<NarrowPhasePair> pairs;
  pairs.reserve(120);
  for (int i = 0; i < 120; ++i) {
    pairs.push_back(cases[static_cast<std::size_t>(i) % cases.size()]);
    pairs.back().tfB = translated(0.01 * (i % 3));
  }

  CollisionOption option;
  option.maxNumContacts = 8;
  std::vector<CollisionResult> results(pairs.size());

  EXPECT_NO_THROW(NarrowPhase::collideBatch(pairs, results, option));
  for (const CollisionResult& result : results) {
    expectFiniteCollisionResult(result);
  }
}

TEST(NarrowPhase, CoincidentVertexMeshNoCrashAcrossBatch)
{
  std::vector<Eigen::Vector3d> vertices(5, Eigen::Vector3d::Zero());
  std::vector<MeshShape::Triangle> triangles
      = {{0, 0, 0}, {0, 1, 2}, {2, 3, 4}, {4, 4, 4}};
  MeshShape coincidentMesh(vertices, triangles);
  MeshShape cubeMesh(makeCubeMeshVertices(1.0), makeCubeMeshTriangles());
  SphereShape unitSphere(1.0);
  BoxShape unitBox(Eigen::Vector3d::Ones());

  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  const std::array<NarrowPhasePair, 4> cases{{
      {&coincidentMesh, &unitSphere, identity, identity},
      {&unitBox, &coincidentMesh, identity, identity},
      {&coincidentMesh, &cubeMesh, identity, identity},
      {&coincidentMesh, &coincidentMesh, identity, identity},
  }};

  std::vector<NarrowPhasePair> pairs;
  pairs.reserve(120);
  for (int i = 0; i < 120; ++i) {
    pairs.push_back(cases[static_cast<std::size_t>(i) % cases.size()]);
    pairs.back().tfB = translated(0.001 * (i % 5));
  }

  CollisionOption option;
  option.maxNumContacts = 8;
  std::vector<CollisionResult> results(pairs.size());

  EXPECT_NO_THROW(NarrowPhase::collideBatch(pairs, results, option));
  for (const CollisionResult& result : results) {
    expectFiniteCollisionResult(result);
  }
}

TEST(NarrowPhase, HugeMagnitudeBoxesRemainFiniteAcrossBatch)
{
  BoxShape boxA(Eigen::Vector3d::Constant(5.0e5));
  BoxShape boxB(Eigen::Vector3d::Constant(5.0e5));

  const Eigen::Isometry3d tfA = translated(1.0e6, -1.0e6, 1.0e6);
  std::vector<NarrowPhasePair> pairs;
  pairs.reserve(120);
  for (int i = 0; i < 120; ++i) {
    const double offset = (i % 2 == 0) ? 9.5e5 : 1.1e6;
    pairs.push_back(
        {&boxA, &boxB, tfA, translated(1.0e6 + offset, -1.0e6, 1.0e6)});
  }

  CollisionOption option;
  option.maxNumContacts = 8;
  std::vector<CollisionResult> results(pairs.size());

  EXPECT_NO_THROW(NarrowPhase::collideBatch(pairs, results, option));
  for (std::size_t i = 0; i < results.size(); ++i) {
    expectFiniteCollisionResult(results[i]);
    if (i % 2 == 0) {
      EXPECT_GE(results[i].numContacts(), 1u);
    } else {
      EXPECT_EQ(results[i].numContacts(), 0u);
    }
  }
}

TEST(NarrowPhase, TinyMagnitudeSpheresRemainFiniteAcrossBatch)
{
  SphereShape sphereA(1.0e-6);
  SphereShape sphereB(1.0e-6);

  const Eigen::Isometry3d tfA = Eigen::Isometry3d::Identity();
  std::vector<NarrowPhasePair> pairs;
  pairs.reserve(120);
  for (int i = 0; i < 120; ++i) {
    const double offset = (i % 2 == 0) ? 1.5e-6 : 3.0e-6;
    pairs.push_back({&sphereA, &sphereB, tfA, translated(offset)});
  }

  CollisionOption option;
  option.maxNumContacts = 8;
  std::vector<CollisionResult> results(pairs.size());

  EXPECT_NO_THROW(NarrowPhase::collideBatch(pairs, results, option));
  for (std::size_t i = 0; i < results.size(); ++i) {
    expectFiniteCollisionResult(results[i]);
    if (i % 2 == 0) {
      EXPECT_GE(results[i].numContacts(), 1u);
    } else {
      EXPECT_EQ(results[i].numContacts(), 0u);
    }
  }
}

TEST(NarrowPhase, collide_batch_dispatcher_rejects_malformed_inputs)
{
  SphereShape sphereA(0.75);
  SphereShape sphereB(0.60);

  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  const std::array<NarrowPhasePair, 2> validPairs{
      NarrowPhasePair{&sphereA, &sphereB, identity, translated(0.85)},
      NarrowPhasePair{&sphereA, &sphereB, identity, translated(3.0)}};

  std::array<CollisionResult, 1> shortResults;
  EXPECT_THROW(
      NarrowPhase::collideBatch(validPairs, shortResults),
      std::invalid_argument);

  std::array<CollisionResult, validPairs.size()> results;
  std::array<bool, 1> shortHits{};
  EXPECT_THROW(
      NarrowPhase::collideBatch(validPairs, results, shortHits),
      std::invalid_argument);

  const std::array<NarrowPhasePair, 1> nullShapePair{
      NarrowPhasePair{nullptr, &sphereB, identity, translated(0.85)}};
  std::array<CollisionResult, 1> nullShapeResults;
  EXPECT_THROW(
      NarrowPhase::collideBatch(nullShapePair, nullShapeResults),
      std::invalid_argument);
}

TEST(NarrowPhase, SphereSphere_Colliding)
{
  CollisionWorld world;
  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0), tf2);

  CollisionOption option;
  CollisionResult result;

  bool hit = NarrowPhase::collide(obj1, obj2, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(NarrowPhase, SphereSphere_Separated)
{
  CollisionWorld world;
  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(3.0, 0, 0);
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0), tf2);

  CollisionOption option;
  CollisionResult result;

  bool hit = NarrowPhase::collide(obj1, obj2, option, result);

  EXPECT_FALSE(hit);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(NarrowPhase, BoxBox_Colliding)
{
  CollisionWorld world;
  auto obj1 = world.createObject(
      std::make_unique<BoxShape>(Eigen::Vector3d(1, 1, 1)));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);
  auto obj2 = world.createObject(
      std::make_unique<BoxShape>(Eigen::Vector3d(1, 1, 1)), tf2);

  CollisionOption option;
  CollisionResult result;

  bool hit = NarrowPhase::collide(obj1, obj2, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(NarrowPhase, SphereBox_Colliding)
{
  CollisionWorld world;
  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);
  auto obj2 = world.createObject(
      std::make_unique<BoxShape>(Eigen::Vector3d(1, 1, 1)), tf2);

  CollisionOption option;
  CollisionResult result;

  bool hit = NarrowPhase::collide(obj1, obj2, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(NarrowPhase, SphereBox_NormalFollowsObjectOrder)
{
  CollisionWorld world;

  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(1.25, 0.0, 0.0);
  auto sphereObj
      = world.createObject(std::make_unique<SphereShape>(1.0), tfSphere);

  auto boxObj
      = world.createObject(std::make_unique<BoxShape>(Eigen::Vector3d::Ones()));

  CollisionOption option;
  CollisionResult result;

  const bool hit = NarrowPhase::collide(sphereObj, boxObj, option, result);

  EXPECT_TRUE(hit);
  ASSERT_GT(result.numContacts(), 0u);
  EXPECT_GT(result.getContact(0).normal.x(), 0.9);
}

TEST(NarrowPhase, BoxSphere_Colliding)
{
  CollisionWorld world;
  auto obj1 = world.createObject(
      std::make_unique<BoxShape>(Eigen::Vector3d(1, 1, 1)));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0), tf2);

  CollisionOption option;
  CollisionResult result;

  bool hit = NarrowPhase::collide(obj1, obj2, option, result);

  EXPECT_TRUE(hit);
  EXPECT_GE(result.numContacts(), 1u);

  EXPECT_NEAR(result.getContact(0).normal.x(), -1.0, 0.01);
}

TEST(NarrowPhase, PlaneMesh_NormalFollowsObjectOrder)
{
  CollisionWorld world;
  auto planeObj = world.createObject(
      std::make_unique<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));

  auto mesh = std::make_unique<MeshShape>(
      std::vector<Eigen::Vector3d>{
          Eigen::Vector3d(-0.5, -0.5, 0.0),
          Eigen::Vector3d(0.5, -0.5, 0.0),
          Eigen::Vector3d(0.0, 0.5, 0.0)},
      std::vector<Eigen::Vector3i>{Eigen::Vector3i(0, 1, 2)});
  Eigen::Isometry3d meshTf = Eigen::Isometry3d::Identity();
  meshTf.translation() = Eigen::Vector3d(0.0, 0.0, -0.25);
  auto meshObj = world.createObject(std::move(mesh), meshTf);

  CollisionOption option;
  CollisionResult result;

  const bool hit = NarrowPhase::collide(planeObj, meshObj, option, result);

  EXPECT_TRUE(hit);
  ASSERT_GT(result.numContacts(), 0u);
  EXPECT_LT(result.getContact(0).normal.z(), -0.9);
}

TEST(NarrowPhase, InvalidHandle_NoCollision)
{
  CollisionWorld world;
  CollisionObject obj1;
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0));

  CollisionOption option;
  CollisionResult result;

  bool hit = NarrowPhase::collide(obj1, obj2, option, result);

  EXPECT_FALSE(hit);
}
