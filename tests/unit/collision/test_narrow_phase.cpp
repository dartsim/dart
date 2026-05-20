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
#include <dart/collision/native/shapes/shape.hpp>

#include <gtest/gtest.h>

#include <array>
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
