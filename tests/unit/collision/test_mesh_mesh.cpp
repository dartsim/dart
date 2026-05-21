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
#include <dart/collision/native/narrow_phase/mesh_mesh.hpp>
#include <dart/collision/native/narrow_phase/narrow_phase.hpp>
#include <dart/collision/native/narrow_phase/raycast.hpp>
#include <dart/collision/native/shapes/shape.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

#include <cmath>

using namespace dart::collision::native;

namespace {

std::vector<Eigen::Vector3d> makeCubeVertices(double halfExtent)
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

std::vector<MeshShape::Triangle> makeCubeTriangles()
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

MeshShape makeGridMesh(int resolution, double scale)
{
  std::vector<Eigen::Vector3d> vertices;
  std::vector<MeshShape::Triangle> triangles;

  vertices.reserve(
      static_cast<std::size_t>((resolution + 1) * (resolution + 1)));
  triangles.reserve(static_cast<std::size_t>(resolution * resolution * 2));

  for (int y = 0; y <= resolution; ++y) {
    for (int x = 0; x <= resolution; ++x) {
      const double px = (static_cast<double>(x) / resolution - 0.5) * scale;
      const double py = (static_cast<double>(y) / resolution - 0.5) * scale;
      vertices.emplace_back(px, py, 0.0);
    }
  }

  const auto idx = [resolution](int x, int y) {
    return y * (resolution + 1) + x;
  };

  for (int y = 0; y < resolution; ++y) {
    for (int x = 0; x < resolution; ++x) {
      triangles.emplace_back(idx(x, y), idx(x + 1, y), idx(x + 1, y + 1));
      triangles.emplace_back(idx(x, y), idx(x + 1, y + 1), idx(x, y + 1));
    }
  }

  return MeshShape(std::move(vertices), std::move(triangles));
}

Eigen::Isometry3d makeMeshBatchTransform(
    double x, double y, double z, double angle)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear() = (Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX())
                 * Eigen::AngleAxisd(0.5 * angle, Eigen::Vector3d::UnitY()))
                    .toRotationMatrix();
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

void expectContactHasMirroredMeshFeatureId(
    const ContactPoint& contact, std::size_t triangleCount)
{
  ASSERT_GE(contact.featureIndex1, 0);
  ASSERT_GE(contact.featureIndex2, 0);
  EXPECT_EQ(contact.featureIndex1, contact.featureIndex2);
  EXPECT_LT(static_cast<std::size_t>(contact.featureIndex1), triangleCount);
  EXPECT_LT(static_cast<std::size_t>(contact.featureIndex2), triangleCount);
}

} // namespace

TEST(MeshMesh, BoxMeshesColliding)
{
  CollisionWorld world;
  auto obj1 = world.createObject(
      std::make_unique<MeshShape>(makeCubeVertices(1.0), makeCubeTriangles()));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0.0, 0.0);
  auto obj2 = world.createObject(
      std::make_unique<MeshShape>(makeCubeVertices(1.0), makeCubeTriangles()),
      tf2);

  CollisionOption option;
  option.maxNumContacts = 8;
  CollisionResult result;

  const bool hit = NarrowPhase::collide(obj1, obj2, option, result);
  ASSERT_TRUE(hit);
  ASSERT_GE(result.numContacts(), 1u);
  EXPECT_GT(result.getContact(0).normal.norm(), 0.9);
  EXPECT_GE(result.getContact(0).depth, 0.0);
}

TEST(MeshMesh, BvhTraversalUsesCurrentTransform)
{
  MeshShape meshA(makeCubeVertices(1.0), makeCubeTriangles());
  MeshShape meshB(makeCubeVertices(1.0), makeCubeTriangles());

  const Eigen::Isometry3d tfA = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();

  CollisionOption option;
  option.maxNumContacts = 8;

  tfB.translation() = Eigen::Vector3d(3.1, 0.0, 0.0);
  CollisionResult separatedBefore;
  EXPECT_FALSE(
      collideMeshMesh(meshA, tfA, meshB, tfB, separatedBefore, option));

  tfB.translation() = Eigen::Vector3d(1.5, 0.0, 0.0);
  CollisionResult overlapping;
  ASSERT_TRUE(collideMeshMesh(meshA, tfA, meshB, tfB, overlapping, option));
  ASSERT_GE(overlapping.numContacts(), 1u);

  tfB.translation() = Eigen::Vector3d(-3.1, 0.0, 0.0);
  CollisionResult separatedAfter;
  EXPECT_FALSE(collideMeshMesh(meshA, tfA, meshB, tfB, separatedAfter, option));
}

TEST(MeshMeshBatch, mesh_mesh_batch_determinism_vs_single)
{
  MeshShape meshA(makeCubeVertices(1.0), makeCubeTriangles());
  MeshShape meshB(makeCubeVertices(1.0), makeCubeTriangles());

  std::mt19937 rng(141421u);
  std::uniform_real_distribution<double> offset(-0.03, 0.03);
  std::uniform_real_distribution<double> angle(-0.05, 0.05);

  std::vector<MeshPair> pairs;
  pairs.reserve(100);
  for (int i = 0; i < 100; ++i) {
    const Eigen::Isometry3d tfA = makeMeshBatchTransform(
        offset(rng), offset(rng), offset(rng), angle(rng));
    const Eigen::Isometry3d tfB = makeMeshBatchTransform(
        1.20 + offset(rng), offset(rng), offset(rng), angle(rng));

    pairs.push_back(MeshPair{&meshA, &meshB, tfA, tfB});
  }

  CollisionOption option;
  option.maxNumContacts = 8;
  std::vector<CollisionResult> batchResults(pairs.size());
  collideMeshMeshBatch(pairs, batchResults, option);

  for (std::size_t i = 0; i < pairs.size(); ++i) {
    CollisionResult singleResult;
    ASSERT_TRUE(collideMeshMesh(
        *pairs[i].shapeA,
        pairs[i].tfA,
        *pairs[i].shapeB,
        pairs[i].tfB,
        singleResult,
        option));
    expectCollisionResultExactlyEqual(singleResult, batchResults[i]);
  }
}

TEST(MeshMeshBatch, RejectsMalformedInputs)
{
  MeshShape meshA(makeCubeVertices(1.0), makeCubeTriangles());
  MeshShape meshB(makeCubeVertices(1.0), makeCubeTriangles());

  const Eigen::Isometry3d tfA = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();
  tfB.translation() = Eigen::Vector3d(1.20, 0.0, 0.0);

  const std::vector<MeshPair> validPairs{{&meshA, &meshB, tfA, tfB}};
  std::vector<CollisionResult> emptyResults;
  EXPECT_THROW(
      collideMeshMeshBatch(validPairs, emptyResults), std::invalid_argument);

  const std::vector<MeshPair> nullShapePairs{{nullptr, &meshB, tfA, tfB}};
  std::vector<CollisionResult> results(1);
  EXPECT_THROW(
      collideMeshMeshBatch(nullShapePairs, results), std::invalid_argument);
}

TEST(MeshMesh, BoxMeshesSeparated)
{
  CollisionWorld world;
  auto obj1 = world.createObject(
      std::make_unique<MeshShape>(makeCubeVertices(1.0), makeCubeTriangles()));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(3.0, 0.0, 0.0);
  auto obj2 = world.createObject(
      std::make_unique<MeshShape>(makeCubeVertices(1.0), makeCubeTriangles()),
      tf2);

  CollisionResult result;
  EXPECT_FALSE(NarrowPhase::collide(obj1, obj2, CollisionOption(), result));
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(PrimitiveMesh, SphereVsMesh)
{
  CollisionWorld world;
  auto mesh
      = std::make_unique<MeshShape>(makeCubeVertices(1.0), makeCubeTriangles());
  const std::size_t triangleCount = mesh->getTriangles().size();
  auto meshObj = world.createObject(std::move(mesh));

  Eigen::Isometry3d sphereTf = Eigen::Isometry3d::Identity();
  sphereTf.translation() = Eigen::Vector3d(0.0, 0.0, 1.3);
  auto sphereObj
      = world.createObject(std::make_unique<SphereShape>(0.5), sphereTf);

  CollisionResult result;
  EXPECT_TRUE(
      NarrowPhase::collide(meshObj, sphereObj, CollisionOption(), result));
  EXPECT_GE(result.numContacts(), 1u);
  expectContactHasMirroredMeshFeatureId(result.getContact(0), triangleCount);
}

TEST(PrimitiveMesh, SphereTouchesSharedTriangleEdgeAtZeroDepth)
{
  std::vector<Eigen::Vector3d> vertices{
      {-1.0, -1.0, 0.0}, {1.0, -1.0, 0.0}, {1.0, 1.0, 0.0}, {-1.0, 1.0, 0.0}};
  std::vector<MeshShape::Triangle> triangles{{0, 1, 2}, {0, 2, 3}};

  const SphereShape sphere(4.0);
  CollisionOption option;
  option.maxNumContacts = 4u;

  auto expectTouchingContacts = [&](const Eigen::Isometry3d& meshTransform,
                                    const Eigen::Vector3d& sphereCenter,
                                    const Eigen::Vector3d& expectedNormal) {
    CollisionWorld world;
    auto meshObj = world.createObject(
        std::make_unique<MeshShape>(vertices, triangles), meshTransform);

    Eigen::Isometry3d sphereTransform = Eigen::Isometry3d::Identity();
    sphereTransform.translation() = sphereCenter;
    auto sphereObj = world.createObject(
        std::make_unique<SphereShape>(sphere), sphereTransform);

    CollisionResult result;
    ASSERT_TRUE(NarrowPhase::collide(meshObj, sphereObj, option, result));
    ASSERT_EQ(result.numContacts(), 2u);
    for (std::size_t i = 0; i < result.numContacts(); ++i) {
      const auto& contact = result.getContact(i);
      EXPECT_NEAR(contact.depth, 0.0, 1e-12);
      EXPECT_NEAR(contact.position.x(), meshTransform.translation().x(), 1e-12);
      EXPECT_NEAR(contact.position.y(), meshTransform.translation().y(), 1e-12);
      EXPECT_NEAR(contact.position.z(), meshTransform.translation().z(), 1e-12);
      EXPECT_NEAR(contact.normal.x(), expectedNormal.x(), 1e-12);
      EXPECT_NEAR(contact.normal.y(), expectedNormal.y(), 1e-12);
      EXPECT_NEAR(contact.normal.z(), expectedNormal.z(), 1e-12);
    }
  };

  Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  expectTouchingContacts(
      identity, Eigen::Vector3d(0.0, 0.0, 4.0), -Eigen::Vector3d::UnitZ());

  Eigen::Isometry3d translated = Eigen::Isometry3d::Identity();
  translated.translation() = Eigen::Vector3d(10.0, 30.0, 40.0);
  expectTouchingContacts(
      translated, Eigen::Vector3d(10.0, 30.0, 44.0), -Eigen::Vector3d::UnitZ());

  Eigen::Isometry3d rotated = Eigen::Isometry3d::Identity();
  rotated.linear() << 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0;
  rotated.translation() = Eigen::Vector3d(10.0, 30.0, 40.0);
  expectTouchingContacts(
      rotated, Eigen::Vector3d(10.0, 26.0, 40.0), Eigen::Vector3d::UnitY());
}

TEST(PrimitiveMesh, CapsuleVsMeshPairOrder)
{
  MeshShape mesh = makeGridMesh(1, 2.0);
  CapsuleShape capsule(0.25, 0.4);

  Eigen::Isometry3d meshTf = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d capsuleTf = Eigen::Isometry3d::Identity();
  capsuleTf.translation() = Eigen::Vector3d(0.0, 0.0, 0.4);

  CollisionOption option;
  option.maxNumContacts = 1;

  CollisionResult meshCapsule;
  const bool meshCapsuleCollides = NarrowPhase::collide(
      &mesh, meshTf, &capsule, capsuleTf, option, meshCapsule);

  ASSERT_TRUE(meshCapsuleCollides);
  ASSERT_EQ(meshCapsule.numContacts(), 1u);
  EXPECT_GT(meshCapsule.getContact(0).depth, 0.0);
  EXPECT_TRUE(meshCapsule.getContact(0).position.allFinite());
  EXPECT_TRUE(meshCapsule.getContact(0).normal.allFinite());
  expectContactHasMirroredMeshFeatureId(
      meshCapsule.getContact(0), mesh.getTriangles().size());

  CollisionResult capsuleMesh;
  const bool capsuleMeshCollides = NarrowPhase::collide(
      &capsule, capsuleTf, &mesh, meshTf, option, capsuleMesh);

  ASSERT_TRUE(capsuleMeshCollides);
  ASSERT_EQ(capsuleMesh.numContacts(), 1u);
  EXPECT_GT(capsuleMesh.getContact(0).depth, 0.0);
  EXPECT_TRUE(capsuleMesh.getContact(0).position.allFinite());
  EXPECT_TRUE(capsuleMesh.getContact(0).normal.allFinite());
  expectContactHasMirroredMeshFeatureId(
      capsuleMesh.getContact(0), mesh.getTriangles().size());
  EXPECT_NEAR(
      meshCapsule.getContact(0).depth, capsuleMesh.getContact(0).depth, 1e-10);
  EXPECT_NEAR(
      (meshCapsule.getContact(0).position - capsuleMesh.getContact(0).position)
          .norm(),
      0.0,
      1e-10);
  EXPECT_NEAR(
      (meshCapsule.getContact(0).normal + capsuleMesh.getContact(0).normal)
          .norm(),
      0.0,
      1e-10);
}

TEST(PrimitiveMesh, CylinderVsMeshPairOrder)
{
  MeshShape mesh = makeGridMesh(1, 2.0);
  CylinderShape cylinder(0.25, 0.4);

  Eigen::Isometry3d meshTf = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d cylinderTf = Eigen::Isometry3d::Identity();
  cylinderTf.translation() = Eigen::Vector3d(0.0, 0.0, 0.15);

  CollisionOption option;
  option.maxNumContacts = 1;

  CollisionResult meshCylinder;
  const bool meshCylinderCollides = NarrowPhase::collide(
      &mesh, meshTf, &cylinder, cylinderTf, option, meshCylinder);

  ASSERT_TRUE(meshCylinderCollides);
  ASSERT_EQ(meshCylinder.numContacts(), 1u);
  EXPECT_GT(meshCylinder.getContact(0).depth, 0.0);
  EXPECT_TRUE(meshCylinder.getContact(0).position.allFinite());
  EXPECT_TRUE(meshCylinder.getContact(0).normal.allFinite());
  expectContactHasMirroredMeshFeatureId(
      meshCylinder.getContact(0), mesh.getTriangles().size());

  CollisionResult cylinderMesh;
  const bool cylinderMeshCollides = NarrowPhase::collide(
      &cylinder, cylinderTf, &mesh, meshTf, option, cylinderMesh);

  ASSERT_TRUE(cylinderMeshCollides);
  ASSERT_EQ(cylinderMesh.numContacts(), 1u);
  EXPECT_GT(cylinderMesh.getContact(0).depth, 0.0);
  EXPECT_TRUE(cylinderMesh.getContact(0).position.allFinite());
  EXPECT_TRUE(cylinderMesh.getContact(0).normal.allFinite());
  expectContactHasMirroredMeshFeatureId(
      cylinderMesh.getContact(0), mesh.getTriangles().size());
  EXPECT_NEAR(
      meshCylinder.getContact(0).depth,
      cylinderMesh.getContact(0).depth,
      1e-10);
  EXPECT_NEAR(
      (meshCylinder.getContact(0).position
       - cylinderMesh.getContact(0).position)
          .norm(),
      0.0,
      1e-10);
  EXPECT_NEAR(
      (meshCylinder.getContact(0).normal + cylinderMesh.getContact(0).normal)
          .norm(),
      0.0,
      1e-10);
}

TEST(PrimitiveMesh, LargeFlatBoxMeshContactPatchIsCapped)
{
  CollisionWorld world;
  auto boxObj = world.createObject(
      std::make_unique<BoxShape>(Eigen::Vector3d(50.0, 50.0, 0.5)));

  Eigen::Isometry3d meshTf = Eigen::Isometry3d::Identity();
  meshTf.translation() = Eigen::Vector3d(0.0, 0.0, 0.25);
  auto mesh = std::make_unique<MeshShape>(makeGridMesh(40, 1.0));
  const std::size_t triangleCount = mesh->getTriangles().size();
  auto meshObj = world.createObject(std::move(mesh), meshTf);

  CollisionOption option;
  option.maxNumContacts = 1000;
  CollisionResult result;

  ASSERT_TRUE(NarrowPhase::collide(boxObj, meshObj, option, result));
  EXPECT_LE(result.numContacts(), 32u);
  ASSERT_GT(result.numContacts(), 30u);
  for (std::size_t i = 0; i < result.numContacts(); ++i) {
    const auto& contact = result.getContact(i);
    EXPECT_LT(contact.normal.z(), -0.99);
    EXPECT_NEAR(contact.depth, 0.25, 1e-12);
    expectContactHasMirroredMeshFeatureId(contact, triangleCount);
  }
}

TEST(PlaneMesh, ContactsCarryMeshFeatureIds)
{
  MeshShape mesh = makeGridMesh(1, 2.0);
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);

  Eigen::Isometry3d meshTf = Eigen::Isometry3d::Identity();
  meshTf.translation() = Eigen::Vector3d(0.0, 0.0, -0.1);
  const Eigen::Isometry3d planeTf = Eigen::Isometry3d::Identity();

  CollisionOption option;
  option.maxNumContacts = 1;

  CollisionResult meshPlane;
  ASSERT_TRUE(
      NarrowPhase::collide(&mesh, meshTf, &plane, planeTf, option, meshPlane));
  ASSERT_EQ(meshPlane.numContacts(), 1u);
  expectContactHasMirroredMeshFeatureId(
      meshPlane.getContact(0), mesh.getTriangles().size());

  CollisionResult planeMesh;
  ASSERT_TRUE(
      NarrowPhase::collide(&plane, planeTf, &mesh, meshTf, option, planeMesh));
  ASSERT_EQ(planeMesh.numContacts(), 1u);
  expectContactHasMirroredMeshFeatureId(
      planeMesh.getContact(0), mesh.getTriangles().size());
}

TEST(RaycastMesh, BvhTraversalHit)
{
  MeshShape mesh(makeCubeVertices(1.0), makeCubeTriangles());
  Ray ray(Eigen::Vector3d(-5.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0));
  RaycastResult result;
  EXPECT_TRUE(raycastMesh(
      ray,
      mesh,
      Eigen::Isometry3d::Identity(),
      RaycastOption::unlimited(),
      result));
  EXPECT_NEAR(result.distance, 4.0, 1e-10);
}

TEST(MeshMesh, SingleTriangleAndLargeMesh)
{
  std::vector<Eigen::Vector3d> triVertices
      = {{-0.5, -0.5, 0.0}, {0.5, -0.5, 0.0}, {0.0, 0.5, 0.0}};
  std::vector<MeshShape::Triangle> triFaces = {{0, 1, 2}};

  CollisionWorld world;
  auto singleTriangle
      = world.createObject(std::make_unique<MeshShape>(triVertices, triFaces));

  Eigen::Isometry3d gridTf = Eigen::Isometry3d::Identity();
  gridTf.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
  auto largeGrid = world.createObject(
      std::make_unique<MeshShape>(makeGridMesh(40, 4.0)), gridTf);

  CollisionResult result;
  EXPECT_TRUE(
      NarrowPhase::collide(
          singleTriangle, largeGrid, CollisionOption(), result));
  EXPECT_GE(result.numContacts(), 1u);

  DistanceResult distance;
  const double d = NarrowPhase::distance(
      singleTriangle, largeGrid, DistanceOption(), distance);
  EXPECT_LE(d, 0.0);
}

TEST(MeshMesh, SeparatedDistanceWitnessesAcrossPairOrder)
{
  CollisionWorld world;
  auto meshA = world.createObject(
      std::make_unique<MeshShape>(makeCubeVertices(1.0), makeCubeTriangles()));

  Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();
  tfB.translation() = Eigen::Vector3d(3.0, 0.0, 0.0);
  auto meshB = world.createObject(
      std::make_unique<MeshShape>(makeCubeVertices(1.0), makeCubeTriangles()),
      tfB);

  DistanceResult ab;
  const double distanceAB
      = NarrowPhase::distance(meshA, meshB, DistanceOption(), ab);
  EXPECT_NEAR(distanceAB, 1.0, 1e-9);
  EXPECT_NEAR(ab.distance, distanceAB, 1e-12);
  EXPECT_TRUE(ab.pointOnObject1.allFinite());
  EXPECT_TRUE(ab.pointOnObject2.allFinite());
  EXPECT_TRUE(ab.normal.allFinite());
  EXPECT_NEAR((ab.pointOnObject2 - ab.pointOnObject1).norm(), 1.0, 1e-9);

  DistanceResult ba;
  const double distanceBA
      = NarrowPhase::distance(meshB, meshA, DistanceOption(), ba);
  EXPECT_NEAR(distanceBA, distanceAB, 1e-9);
  EXPECT_TRUE(ba.pointOnObject1.allFinite());
  EXPECT_TRUE(ba.pointOnObject2.allFinite());
  EXPECT_TRUE(ba.normal.allFinite());
  EXPECT_NEAR((ba.pointOnObject2 - ba.pointOnObject1).norm(), 1.0, 1e-9);
  EXPECT_NEAR((ab.pointOnObject1 - ba.pointOnObject2).norm(), 0.0, 1e-9);
  EXPECT_NEAR((ab.pointOnObject2 - ba.pointOnObject1).norm(), 0.0, 1e-9);
}

TEST(MeshMesh, EmptyMeshesReturnNoContactsAndMaxDistance)
{
  MeshShape emptyMesh({}, {});
  MeshShape cubeMesh(makeCubeVertices(1.0), makeCubeTriangles());
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();

  CollisionOption collisionOption;
  CollisionResult collision;
  EXPECT_FALSE(collideMeshMesh(
      emptyMesh, identity, cubeMesh, identity, collision, collisionOption));

  DistanceResult distance;
  EXPECT_EQ(
      distanceMeshMesh(
          emptyMesh, identity, cubeMesh, identity, distance, DistanceOption()),
      std::numeric_limits<double>::max());

  SphereShape sphere(0.5);
  CollisionResult primitiveMesh;
  EXPECT_FALSE(collidePrimitiveMesh(
      sphere, identity, emptyMesh, identity, primitiveMesh, collisionOption));

  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  CollisionResult planeMesh;
  EXPECT_FALSE(
      collidePlaneMesh(plane, identity, emptyMesh, identity, planeMesh));
}

TEST(MeshMesh, BinaryContactTraversalStopsAfterFirstTriangleHit)
{
  MeshShape meshA(makeCubeVertices(1.0), makeCubeTriangles());
  MeshShape meshB(makeCubeVertices(1.0), makeCubeTriangles());

  CollisionOption option = CollisionOption::binaryCheck();
  CollisionResult result;
  EXPECT_TRUE(collideMeshMesh(
      meshA,
      Eigen::Isometry3d::Identity(),
      meshB,
      Eigen::Isometry3d::Identity(),
      result,
      option));
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(PrimitiveMesh, DegenerateTriangleContactsStayFinite)
{
  MeshShape degenerateMesh(
      {Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero()},
      {{0, 1, 2}});
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();

  CollisionOption option;
  option.maxNumContacts = 4;

  {
    SphereShape sphere(0.2);
    CollisionResult result;
    ASSERT_TRUE(collidePrimitiveMesh(
        sphere, identity, degenerateMesh, identity, result, option));
    ASSERT_EQ(result.numContacts(), 1u);
    EXPECT_TRUE(result.getContact(0).position.allFinite());
    EXPECT_TRUE(result.getContact(0).normal.allFinite());
    EXPECT_NEAR(result.getContact(0).normal.z(), 1.0, 1e-12);
    expectContactHasMirroredMeshFeatureId(
        result.getContact(0), degenerateMesh.getTriangles().size());
  }

  {
    CapsuleShape capsule(0.2, 1.0);
    CollisionResult result;
    ASSERT_TRUE(collidePrimitiveMesh(
        capsule, identity, degenerateMesh, identity, result, option));
    ASSERT_EQ(result.numContacts(), 1u);
    EXPECT_TRUE(result.getContact(0).position.allFinite());
    EXPECT_TRUE(result.getContact(0).normal.allFinite());
    EXPECT_NEAR(result.getContact(0).normal.z(), 1.0, 1e-12);
    expectContactHasMirroredMeshFeatureId(
        result.getContact(0), degenerateMesh.getTriangles().size());
  }
}

TEST(PrimitiveMesh, BinaryPrimitiveTriangleQueriesStopAfterHit)
{
  MeshShape mesh = makeGridMesh(1, 2.0);
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  CollisionOption option = CollisionOption::binaryCheck();

  {
    SphereShape sphere(0.5);
    Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
    tfSphere.translation() = Eigen::Vector3d(0.0, 0.0, 0.25);

    CollisionResult result;
    EXPECT_TRUE(
        collidePrimitiveMesh(sphere, tfSphere, mesh, identity, result, option));
    EXPECT_LE(result.numContacts(), 1u);
  }

  {
    CapsuleShape capsule(0.25, 0.4);
    Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
    tfCapsule.translation() = Eigen::Vector3d(0.0, 0.0, 0.25);

    CollisionResult result;
    EXPECT_TRUE(collidePrimitiveMesh(
        capsule, tfCapsule, mesh, identity, result, option));
    EXPECT_LE(result.numContacts(), 1u);
  }

  {
    BoxShape box(Eigen::Vector3d(0.25, 0.25, 0.25));
    Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
    tfBox.translation() = Eigen::Vector3d(0.0, 0.0, 0.1);

    CollisionResult result;
    EXPECT_TRUE(
        collidePrimitiveMesh(box, tfBox, mesh, identity, result, option));
    EXPECT_LE(result.numContacts(), 1u);
  }
}

TEST(PrimitiveMesh, CapsuleSegmentPiercesTriangleUsesFaceNormal)
{
  MeshShape triangleMesh(
      {Eigen::Vector3d(-1.0, -1.0, 0.0),
       Eigen::Vector3d(1.0, -1.0, 0.0),
       Eigen::Vector3d(0.0, 1.0, 0.0)},
      {{0, 1, 2}});
  CapsuleShape capsule(0.25, 1.0);

  Eigen::Isometry3d capsuleTf = Eigen::Isometry3d::Identity();
  capsuleTf.translation() = Eigen::Vector3d(0.0, 0.0, 0.1);

  CollisionResult result;
  ASSERT_TRUE(collidePrimitiveMesh(
      capsule,
      capsuleTf,
      triangleMesh,
      Eigen::Isometry3d::Identity(),
      result,
      CollisionOption()));
  ASSERT_EQ(result.numContacts(), 1u);
  EXPECT_NEAR(result.getContact(0).depth, capsule.getRadius(), 1e-12);
  EXPECT_NEAR(result.getContact(0).normal.z(), -1.0, 1e-12);
  expectContactHasMirroredMeshFeatureId(
      result.getContact(0), triangleMesh.getTriangles().size());
}

TEST(PrimitiveMesh, PrimitiveAabbAndContactLimitsSkipTriangles)
{
  MeshShape mesh(makeCubeVertices(1.0), makeCubeTriangles());
  SphereShape farSphere(0.25);
  SphereShape nearSphere(0.75);
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);

  Eigen::Isometry3d farTf = Eigen::Isometry3d::Identity();
  farTf.translation() = Eigen::Vector3d(10.0, 0.0, 0.0);

  CollisionOption option;
  CollisionResult farResult;
  EXPECT_FALSE(collidePrimitiveMesh(
      farSphere,
      farTf,
      mesh,
      Eigen::Isometry3d::Identity(),
      farResult,
      option));

  CollisionOption limitedOption;
  limitedOption.maxNumContacts = 0;
  CollisionResult limitedPrimitive;
  EXPECT_FALSE(collidePrimitiveMesh(
      nearSphere,
      Eigen::Isometry3d::Identity(),
      mesh,
      Eigen::Isometry3d::Identity(),
      limitedPrimitive,
      limitedOption));

  CollisionResult limitedPlane;
  EXPECT_FALSE(collidePlaneMesh(
      plane,
      Eigen::Isometry3d::Identity(),
      mesh,
      Eigen::Isometry3d::Identity(),
      limitedPlane,
      limitedOption));
}

TEST(PrimitiveMesh, DirectHelperRejectsUnsupportedAndLargeFaceLimits)
{
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  MeshShape mesh(makeCubeVertices(1.0), makeCubeTriangles());
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);

  CollisionResult unsupported;
  EXPECT_FALSE(collidePrimitiveMesh(
      plane, identity, mesh, identity, unsupported, CollisionOption()));

  BoxShape box(Eigen::Vector3d(50.0, 50.0, 0.5));
  CollisionOption noContacts;
  noContacts.maxNumContacts = 0;
  CollisionResult limited;
  EXPECT_FALSE(
      collidePrimitiveMesh(box, identity, mesh, identity, limited, noContacts));

  BoxShape tooSmallFace(Eigen::Vector3d(0.1, 0.1, 0.5));
  CollisionResult smallFace;
  EXPECT_FALSE(collidePrimitiveMesh(
      tooSmallFace, identity, mesh, identity, smallFace, CollisionOption()));
}

TEST(PrimitiveMesh, SphereTriangleInteriorUsesBarycentricClosestPoint)
{
  MeshShape triangleMesh(
      {Eigen::Vector3d(-1.0, -1.0, 0.0),
       Eigen::Vector3d(1.0, -1.0, 0.0),
       Eigen::Vector3d(0.0, 1.0, 0.0)},
      {{0, 1, 2}});
  SphereShape sphere(0.5);

  Eigen::Isometry3d sphereTf = Eigen::Isometry3d::Identity();
  sphereTf.translation() = Eigen::Vector3d(0.0, 0.0, 0.25);

  CollisionResult result;
  ASSERT_TRUE(collidePrimitiveMesh(
      sphere,
      sphereTf,
      triangleMesh,
      Eigen::Isometry3d::Identity(),
      result,
      CollisionOption()));
  ASSERT_EQ(result.numContacts(), 1u);
  EXPECT_NEAR(result.getContact(0).position.x(), 0.0, 1e-12);
  EXPECT_NEAR(result.getContact(0).position.y(), 0.0, 1e-12);
  EXPECT_NEAR(result.getContact(0).position.z(), 0.0, 1e-12);
  EXPECT_NEAR(result.getContact(0).depth, 0.25, 1e-12);
}

TEST(PrimitiveMesh, SphereOnTrianglePlaneUsesFaceNormal)
{
  MeshShape triangleMesh(
      {Eigen::Vector3d(-1.0, -1.0, 0.0),
       Eigen::Vector3d(1.0, -1.0, 0.0),
       Eigen::Vector3d(0.0, 1.0, 0.0)},
      {{0, 1, 2}});
  SphereShape sphere(0.25);

  Eigen::Isometry3d sphereTf = Eigen::Isometry3d::Identity();
  sphereTf.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);

  CollisionResult result;
  ASSERT_TRUE(collidePrimitiveMesh(
      sphere,
      sphereTf,
      triangleMesh,
      Eigen::Isometry3d::Identity(),
      result,
      CollisionOption()));
  ASSERT_EQ(result.numContacts(), 1u);
  EXPECT_NEAR(result.getContact(0).depth, sphere.getRadius(), 1e-12);
  EXPECT_TRUE(result.getContact(0).normal.allFinite());
  EXPECT_NEAR(result.getContact(0).normal.z(), 1.0, 1e-12);
}

TEST(PrimitiveMesh, ConvexPrimitiveUsesTriangleSupportPath)
{
  MeshShape triangleMesh(
      {Eigen::Vector3d(-1.0, -1.0, 0.0),
       Eigen::Vector3d(1.0, -1.0, 0.0),
       Eigen::Vector3d(0.0, 1.0, 0.0)},
      {{0, 1, 2}});
  ConvexShape convex(
      {Eigen::Vector3d(0.0, 0.0, -0.25),
       Eigen::Vector3d(0.25, 0.0, 0.25),
       Eigen::Vector3d(-0.25, 0.0, 0.25),
       Eigen::Vector3d(0.0, 0.25, 0.25)});

  CollisionResult result;
  ASSERT_TRUE(collidePrimitiveMesh(
      convex,
      Eigen::Isometry3d::Identity(),
      triangleMesh,
      Eigen::Isometry3d::Identity(),
      result,
      CollisionOption()));
  ASSERT_GT(result.numContacts(), 0u);
  EXPECT_TRUE(result.getContact(0).position.allFinite());
  EXPECT_TRUE(result.getContact(0).normal.allFinite());
}

TEST(PrimitiveMesh, CapsuleTriangleContactLimitAndSeparatedTriangle)
{
  MeshShape triangleMesh(
      {Eigen::Vector3d(-1.0, -1.0, 0.0),
       Eigen::Vector3d(1.0, -1.0, 0.0),
       Eigen::Vector3d(0.0, 1.0, 0.0)},
      {{0, 1, 2}});
  CapsuleShape capsule(0.25, 1.0);
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();

  CollisionOption noContacts;
  noContacts.maxNumContacts = 0u;
  CollisionResult limited;
  EXPECT_FALSE(collidePrimitiveMesh(
      capsule, identity, triangleMesh, identity, limited, noContacts));

  Eigen::Isometry3d separatedTf = Eigen::Isometry3d::Identity();
  separatedTf.translation() = Eigen::Vector3d(0.0, 1.2, 0.0);
  CapsuleShape thinCapsule(0.05, 0.25);
  CollisionResult separated;
  EXPECT_FALSE(collidePrimitiveMesh(
      thinCapsule,
      separatedTf,
      triangleMesh,
      identity,
      separated,
      CollisionOption()));
}
