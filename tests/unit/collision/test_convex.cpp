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

#include <dart/collision/native/collision_world.hpp>
#include <dart/collision/native/narrow_phase/convex_convex.hpp>
#include <dart/collision/native/narrow_phase/narrow_phase.hpp>
#include <dart/collision/native/shapes/shape.hpp>

#include <gtest/gtest.h>

#include <numbers>
#include <random>
#include <stdexcept>
#include <vector>

using namespace dart::collision::native;

std::vector<Eigen::Vector3d> makeOctahedronVertices(double scale = 1.0)
{
  return {
      {scale, 0, 0},
      {-scale, 0, 0},
      {0, scale, 0},
      {0, -scale, 0},
      {0, 0, scale},
      {0, 0, -scale}};
}

std::vector<Eigen::Vector3d> makeCubeVertices(double halfExtent = 1.0)
{
  double h = halfExtent;
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
      {0, 1, 2},
      {0, 2, 3},
      {4, 6, 5},
      {4, 7, 6},
      {0, 5, 1},
      {0, 4, 5},
      {2, 6, 7},
      {2, 7, 3},
      {0, 7, 4},
      {0, 3, 7},
      {1, 5, 6},
      {1, 6, 2}};
}

Eigen::Isometry3d makeConvexBatchTransform(
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

TEST(ConvexCollision, ConvexConvexIntersecting)
{
  auto octahedron1 = std::make_unique<ConvexShape>(makeOctahedronVertices());
  auto octahedron2 = std::make_unique<ConvexShape>(makeOctahedronVertices());

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);

  CollisionResult result;
  CollisionOption option;
  option.maxNumContacts = 1;

  bool collides = collideConvexConvex(
      *octahedron1, tf1, *octahedron2, tf2, result, option);

  EXPECT_TRUE(collides);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(ConvexCollisionBatch, convex_convex_batch_determinism_vs_single)
{
  ConvexShape convexA(makeOctahedronVertices(0.60));
  ConvexShape convexB(makeOctahedronVertices(0.55));

  std::mt19937 rng(577215u);
  std::uniform_real_distribution<double> offset(-0.04, 0.04);
  std::uniform_real_distribution<double> angle(-0.12, 0.12);

  std::vector<ConvexPair> pairs;
  pairs.reserve(100);
  for (int i = 0; i < 100; ++i) {
    const Eigen::Isometry3d tfA = makeConvexBatchTransform(
        offset(rng), offset(rng), offset(rng), angle(rng));
    const Eigen::Isometry3d tfB = makeConvexBatchTransform(
        0.65 + offset(rng), offset(rng), offset(rng), angle(rng));

    pairs.push_back(ConvexPair{&convexA, &convexB, tfA, tfB});
  }

  CollisionOption option;
  option.maxNumContacts = 1;
  std::vector<CollisionResult> batchResults(pairs.size());
  collideConvexConvexBatch(pairs, batchResults, option);

  for (std::size_t i = 0; i < pairs.size(); ++i) {
    CollisionResult singleResult;
    ASSERT_TRUE(collideConvexConvex(
        *pairs[i].shapeA,
        pairs[i].tfA,
        *pairs[i].shapeB,
        pairs[i].tfB,
        singleResult,
        option));
    expectCollisionResultExactlyEqual(singleResult, batchResults[i]);
  }
}

TEST(ConvexCollisionBatch, RejectsMalformedInputs)
{
  ConvexShape convexA(makeOctahedronVertices(0.60));
  ConvexShape convexB(makeOctahedronVertices(0.55));

  const Eigen::Isometry3d tfA = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();
  tfB.translation() = Eigen::Vector3d(0.65, 0.0, 0.0);

  const std::vector<ConvexPair> validPairs{{&convexA, &convexB, tfA, tfB}};
  std::vector<CollisionResult> emptyResults;
  EXPECT_THROW(
      collideConvexConvexBatch(validPairs, emptyResults),
      std::invalid_argument);

  const std::vector<ConvexPair> nullShapePairs{{nullptr, &convexB, tfA, tfB}};
  std::vector<CollisionResult> results(1);
  EXPECT_THROW(
      collideConvexConvexBatch(nullShapePairs, results), std::invalid_argument);
}

TEST(ConvexCollision, ConvexConvexSeparated)
{
  auto octahedron1 = std::make_unique<ConvexShape>(makeOctahedronVertices());
  auto octahedron2 = std::make_unique<ConvexShape>(makeOctahedronVertices());

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(3.0, 0, 0);

  CollisionResult result;
  CollisionOption option;

  bool collides = collideConvexConvex(
      *octahedron1, tf1, *octahedron2, tf2, result, option);

  EXPECT_FALSE(collides);
}

TEST(ConvexCollision, ConvexSphereIntersecting)
{
  auto octahedron = std::make_unique<ConvexShape>(makeOctahedronVertices());
  auto sphere = std::make_unique<SphereShape>(1.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);

  CollisionResult result;
  CollisionOption option;
  option.maxNumContacts = 1;

  bool collides
      = collideConvexConvex(*octahedron, tf1, *sphere, tf2, result, option);

  EXPECT_TRUE(collides);
}

TEST(ConvexCollision, ConvexBoxIntersecting)
{
  auto octahedron = std::make_unique<ConvexShape>(makeOctahedronVertices());
  auto box = std::make_unique<BoxShape>(Eigen::Vector3d(0.5, 0.5, 0.5));

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.0, 0, 0);

  CollisionResult result;
  CollisionOption option;
  option.maxNumContacts = 1;

  bool collides
      = collideConvexConvex(*octahedron, tf1, *box, tf2, result, option);

  EXPECT_TRUE(collides);
}

TEST(ConvexCollision, PlaneConvexPairOrder)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  ConvexShape octahedron(makeOctahedronVertices(0.5));

  Eigen::Isometry3d planeTf = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d convexTf = Eigen::Isometry3d::Identity();
  convexTf.translation() = Eigen::Vector3d(0, 0, 0.25);

  CollisionOption option;
  option.maxNumContacts = 1;

  CollisionResult planeConvex;
  const bool planeConvexCollides = NarrowPhase::collide(
      &plane, planeTf, &octahedron, convexTf, option, planeConvex);

  ASSERT_TRUE(planeConvexCollides);
  ASSERT_EQ(planeConvex.numContacts(), 1u);
  EXPECT_GT(planeConvex.getContact(0).depth, 0.0);
  EXPECT_LT(planeConvex.getContact(0).normal.z(), -0.9);
  EXPECT_TRUE(planeConvex.getContact(0).position.allFinite());

  CollisionResult convexPlane;
  const bool convexPlaneCollides = NarrowPhase::collide(
      &octahedron, convexTf, &plane, planeTf, option, convexPlane);

  ASSERT_TRUE(convexPlaneCollides);
  ASSERT_EQ(convexPlane.numContacts(), 1u);
  EXPECT_GT(convexPlane.getContact(0).depth, 0.0);
  EXPECT_GT(convexPlane.getContact(0).normal.z(), 0.9);
  EXPECT_TRUE(convexPlane.getContact(0).position.allFinite());
  EXPECT_NEAR(
      planeConvex.getContact(0).depth, convexPlane.getContact(0).depth, 1e-10);
  EXPECT_NEAR(
      (planeConvex.getContact(0).position - convexPlane.getContact(0).position)
          .norm(),
      0.0,
      1e-10);
}

TEST(ConvexCollision, CapsuleConvexPairOrder)
{
  CapsuleShape capsule(0.25, 1.0);
  ConvexShape octahedron(makeOctahedronVertices(0.5));

  Eigen::Isometry3d capsuleTf = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d convexTf = Eigen::Isometry3d::Identity();
  convexTf.translation() = Eigen::Vector3d(0.5, 0, 0);

  CollisionOption option;
  option.maxNumContacts = 1;

  CollisionResult capsuleConvex;
  const bool capsuleConvexCollides = NarrowPhase::collide(
      &capsule, capsuleTf, &octahedron, convexTf, option, capsuleConvex);

  ASSERT_TRUE(capsuleConvexCollides);
  ASSERT_EQ(capsuleConvex.numContacts(), 1u);
  EXPECT_GT(capsuleConvex.getContact(0).depth, 0.0);
  EXPECT_TRUE(capsuleConvex.getContact(0).position.allFinite());
  EXPECT_TRUE(capsuleConvex.getContact(0).normal.allFinite());
  EXPECT_LT(capsuleConvex.getContact(0).normal.x(), -0.5);

  CollisionResult convexCapsule;
  const bool convexCapsuleCollides = NarrowPhase::collide(
      &octahedron, convexTf, &capsule, capsuleTf, option, convexCapsule);

  ASSERT_TRUE(convexCapsuleCollides);
  ASSERT_EQ(convexCapsule.numContacts(), 1u);
  EXPECT_GT(convexCapsule.getContact(0).depth, 0.0);
  EXPECT_TRUE(convexCapsule.getContact(0).position.allFinite());
  EXPECT_TRUE(convexCapsule.getContact(0).normal.allFinite());
  EXPECT_GT(convexCapsule.getContact(0).normal.x(), 0.5);
  EXPECT_NEAR(
      capsuleConvex.getContact(0).depth,
      convexCapsule.getContact(0).depth,
      1e-10);
  EXPECT_NEAR(
      (capsuleConvex.getContact(0).position
       - convexCapsule.getContact(0).position)
          .norm(),
      0.0,
      1e-10);
}

TEST(ConvexCollision, CylinderConvexPairOrder)
{
  CylinderShape cylinder(0.25, 1.0);
  ConvexShape octahedron(makeOctahedronVertices(0.5));

  Eigen::Isometry3d cylinderTf = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d convexTf = Eigen::Isometry3d::Identity();
  convexTf.translation() = Eigen::Vector3d(0.5, 0, 0);

  CollisionOption option;
  option.maxNumContacts = 1;

  CollisionResult cylinderConvex;
  const bool cylinderConvexCollides = NarrowPhase::collide(
      &cylinder, cylinderTf, &octahedron, convexTf, option, cylinderConvex);

  ASSERT_TRUE(cylinderConvexCollides);
  ASSERT_EQ(cylinderConvex.numContacts(), 1u);
  EXPECT_GT(cylinderConvex.getContact(0).depth, 0.0);
  EXPECT_TRUE(cylinderConvex.getContact(0).position.allFinite());
  EXPECT_TRUE(cylinderConvex.getContact(0).normal.allFinite());
  EXPECT_LT(cylinderConvex.getContact(0).normal.x(), -0.5);

  CollisionResult convexCylinder;
  const bool convexCylinderCollides = NarrowPhase::collide(
      &octahedron, convexTf, &cylinder, cylinderTf, option, convexCylinder);

  ASSERT_TRUE(convexCylinderCollides);
  ASSERT_EQ(convexCylinder.numContacts(), 1u);
  EXPECT_GT(convexCylinder.getContact(0).depth, 0.0);
  EXPECT_TRUE(convexCylinder.getContact(0).position.allFinite());
  EXPECT_TRUE(convexCylinder.getContact(0).normal.allFinite());
  EXPECT_GT(convexCylinder.getContact(0).normal.x(), 0.5);
  EXPECT_NEAR(
      cylinderConvex.getContact(0).depth,
      convexCylinder.getContact(0).depth,
      1e-10);
  EXPECT_NEAR(
      (cylinderConvex.getContact(0).position
       - convexCylinder.getContact(0).position)
          .norm(),
      0.0,
      1e-10);
}

TEST(ConvexCollision, MeshConvexPairOrder)
{
  MeshShape mesh(makeCubeVertices(0.5), makeCubeTriangles());
  ConvexShape octahedron(makeOctahedronVertices(0.5));

  Eigen::Isometry3d meshTf = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d convexTf = Eigen::Isometry3d::Identity();
  convexTf.translation() = Eigen::Vector3d(0.75, 0, 0);

  CollisionOption option;
  option.maxNumContacts = 1;

  CollisionResult meshConvex;
  const bool meshConvexCollides = NarrowPhase::collide(
      &mesh, meshTf, &octahedron, convexTf, option, meshConvex);

  ASSERT_TRUE(meshConvexCollides);
  ASSERT_EQ(meshConvex.numContacts(), 1u);
  EXPECT_GT(meshConvex.getContact(0).depth, 0.0);
  EXPECT_TRUE(meshConvex.getContact(0).position.allFinite());
  EXPECT_TRUE(meshConvex.getContact(0).normal.allFinite());
  EXPECT_LT(meshConvex.getContact(0).normal.x(), -0.5);

  CollisionResult convexMesh;
  const bool convexMeshCollides = NarrowPhase::collide(
      &octahedron, convexTf, &mesh, meshTf, option, convexMesh);

  ASSERT_TRUE(convexMeshCollides);
  ASSERT_EQ(convexMesh.numContacts(), 1u);
  EXPECT_GT(convexMesh.getContact(0).depth, 0.0);
  EXPECT_TRUE(convexMesh.getContact(0).position.allFinite());
  EXPECT_TRUE(convexMesh.getContact(0).normal.allFinite());
  EXPECT_GT(convexMesh.getContact(0).normal.x(), 0.5);
  EXPECT_NEAR(
      meshConvex.getContact(0).depth, convexMesh.getContact(0).depth, 1e-10);
  EXPECT_NEAR(
      (meshConvex.getContact(0).position - convexMesh.getContact(0).position)
          .norm(),
      0.0,
      1e-10);
}

TEST(ConvexCollision, MeshMeshIntersecting)
{
  std::vector<Eigen::Vector3d> vertices1 = makeCubeVertices();
  std::vector<MeshShape::Triangle> triangles1
      = {{0, 1, 2},
         {0, 2, 3},
         {4, 6, 5},
         {4, 7, 6},
         {0, 5, 1},
         {0, 4, 5},
         {2, 6, 7},
         {2, 7, 3},
         {0, 7, 4},
         {0, 3, 7},
         {1, 5, 6},
         {1, 6, 2}};

  auto mesh1 = std::make_unique<MeshShape>(vertices1, triangles1);
  auto mesh2 = std::make_unique<MeshShape>(vertices1, triangles1);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);

  CollisionResult result;
  CollisionOption option;
  option.maxNumContacts = 1;

  bool collides = collideConvexConvex(*mesh1, tf1, *mesh2, tf2, result, option);

  EXPECT_TRUE(collides);
}

TEST(ConvexCollision, MeshMeshSeparated)
{
  std::vector<Eigen::Vector3d> vertices1 = makeCubeVertices();
  std::vector<MeshShape::Triangle> triangles1
      = {{0, 1, 2}, {0, 2, 3}, {4, 6, 5}, {4, 7, 6}};

  auto mesh1 = std::make_unique<MeshShape>(vertices1, triangles1);
  auto mesh2 = std::make_unique<MeshShape>(vertices1, triangles1);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(5.0, 0, 0);

  CollisionResult result;
  CollisionOption option;

  bool collides = collideConvexConvex(*mesh1, tf1, *mesh2, tf2, result, option);

  EXPECT_FALSE(collides);
}

TEST(ConvexCollision, NarrowPhaseSupportsConvex)
{
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Convex, ShapeType::Convex));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Convex, ShapeType::Sphere));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Sphere, ShapeType::Convex));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Convex, ShapeType::Box));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Mesh, ShapeType::Mesh));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Mesh, ShapeType::Convex));
}

TEST(ConvexCollision, CollisionWorldWithConvex)
{
  CollisionWorld world;

  auto obj1 = world.createObject(
      std::make_unique<ConvexShape>(makeOctahedronVertices()));
  auto obj2 = world.createObject(
      std::make_unique<ConvexShape>(makeOctahedronVertices()));

  obj1.setTransform(Eigen::Isometry3d::Identity());
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);
  obj2.setTransform(tf2);

  CollisionOption option;
  option.maxNumContacts = 10;
  CollisionResult result;

  bool collides = world.collide(option, result);

  EXPECT_TRUE(collides);
}

TEST(ConvexCollision, RotatedConvex)
{
  auto cube1 = std::make_unique<ConvexShape>(makeCubeVertices(0.5));
  auto cube2 = std::make_unique<ConvexShape>(makeCubeVertices(0.5));

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.linear() = Eigen::AngleAxisd(
                     std::numbers::pi_v<double> / 4, Eigen::Vector3d::UnitZ())
                     .toRotationMatrix();
  tf2.translation() = Eigen::Vector3d(1.0, 0, 0);

  CollisionResult result;
  CollisionOption option;
  option.maxNumContacts = 1;

  bool collides = collideConvexConvex(*cube1, tf1, *cube2, tf2, result, option);

  EXPECT_TRUE(collides);
}

TEST(ConvexCollision, RotatedConvexNoCollision)
{
  auto cube1 = std::make_unique<ConvexShape>(makeCubeVertices(0.5));
  auto cube2 = std::make_unique<ConvexShape>(makeCubeVertices(0.5));

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(2.0, 0, 0);

  CollisionResult result;
  CollisionOption option;

  bool collides = collideConvexConvex(*cube1, tf1, *cube2, tf2, result, option);

  EXPECT_FALSE(collides);
}
