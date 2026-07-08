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

#include <dart/collision/native/Types.hpp>
#include <dart/collision/native/narrow_phase/MeshMesh.hpp>
#include <dart/collision/native/shapes/Shape.hpp>

#include <gtest/gtest.h>

#include <array>

using namespace dart::collision::native;

namespace {

Eigen::Isometry3d translated(double x, double y, double z)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(x, y, z);
  return tf;
}

MeshShape makeUnitCubeMesh()
{
  std::vector<Eigen::Vector3d> vertices
      = {{-0.5, -0.5, -0.5},
         {0.5, -0.5, -0.5},
         {0.5, 0.5, -0.5},
         {-0.5, 0.5, -0.5},
         {-0.5, -0.5, 0.5},
         {0.5, -0.5, 0.5},
         {0.5, 0.5, 0.5},
         {-0.5, 0.5, 0.5}};
  std::vector<MeshShape::Triangle> triangles
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

  return MeshShape(vertices, triangles);
}

MeshShape makePlaneMesh(double z = 0.0)
{
  std::vector<Eigen::Vector3d> vertices
      = {{-1.0, -1.0, z}, {1.0, -1.0, z}, {1.0, 1.0, z}, {-1.0, 1.0, z}};
  std::vector<MeshShape::Triangle> triangles = {{0, 1, 2}, {0, 2, 3}};
  return MeshShape(vertices, triangles);
}

MeshShape makeSingleTriangleMesh(
    const Eigen::Vector3d& v0,
    const Eigen::Vector3d& v1,
    const Eigen::Vector3d& v2)
{
  std::vector<Eigen::Vector3d> vertices = {v0, v1, v2};
  std::vector<MeshShape::Triangle> triangles = {{0, 1, 2}};
  return MeshShape(vertices, triangles);
}

} // namespace

TEST(MeshMeshCollision, OverlappingCubeMeshesCollide)
{
  MeshShape mesh1 = makeUnitCubeMesh();
  MeshShape mesh2 = makeUnitCubeMesh();
  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.75, 0.0, 0.0);

  CollisionResult result;
  EXPECT_TRUE(collideMeshMesh(
      mesh1, tf1, mesh2, tf2, result, CollisionOption::fullContacts(16)));
  EXPECT_GT(result.numContacts(), 0u);

  const auto& contact = result.getContact(0);
  EXPECT_GE(contact.featureIndex1, 0);
  EXPECT_GE(contact.featureIndex2, 0);

  bool hasSecondToFirstXNormal = false;
  for (std::size_t i = 0; i < result.numContacts(); ++i) {
    hasSecondToFirstXNormal |= result.getContact(i).normal.x() < -0.99;
  }
  EXPECT_TRUE(hasSecondToFirstXNormal);
}

TEST(MeshMeshCollision, SeparatedMeshesDoNotCollide)
{
  MeshShape mesh1 = makeUnitCubeMesh();
  MeshShape mesh2 = makeUnitCubeMesh();
  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(3.0, 0.0, 0.0);

  CollisionResult result;
  EXPECT_FALSE(collideMeshMesh(
      mesh1, tf1, mesh2, tf2, result, CollisionOption::fullContacts(16)));
  EXPECT_EQ(0u, result.numContacts());
}

TEST(MeshMeshCollision, SeparatedCoplanarTrianglesDoNotCollide)
{
  MeshShape mesh1 = makeSingleTriangleMesh(
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0));
  MeshShape mesh2 = makeSingleTriangleMesh(
      Eigen::Vector3d(0.75, 0.75, 0.0),
      Eigen::Vector3d(1.75, 0.75, 0.0),
      Eigen::Vector3d(0.75, 1.75, 0.0));

  CollisionResult result;
  EXPECT_FALSE(collideMeshMesh(
      mesh1,
      Eigen::Isometry3d::Identity(),
      mesh2,
      Eigen::Isometry3d::Identity(),
      result,
      CollisionOption::fullContacts(16)));
  EXPECT_EQ(0u, result.numContacts());
}

TEST(MeshMeshCollision, EdgeThroughFaceTrianglesCollide)
{
  MeshShape horizontal = makeSingleTriangleMesh(
      Eigen::Vector3d(-1.0, -1.0, 0.0),
      Eigen::Vector3d(1.0, -1.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0));
  MeshShape vertical = makeSingleTriangleMesh(
      Eigen::Vector3d(0.0, -0.25, -1.0),
      Eigen::Vector3d(0.0, -0.25, 1.0),
      Eigen::Vector3d(0.0, 0.75, 0.25));

  CollisionResult result;
  EXPECT_TRUE(collideMeshMesh(
      horizontal,
      Eigen::Isometry3d::Identity(),
      vertical,
      Eigen::Isometry3d::Identity(),
      result,
      CollisionOption::fullContacts(4)));
  ASSERT_GT(result.numContacts(), 0u);

  bool hasFacePiercingContact = false;
  for (std::size_t i = 0; i < result.numContacts(); ++i) {
    const auto& contact = result.getContact(i);
    hasFacePiercingContact |= std::abs(contact.position.x()) < 1e-12
                              && std::abs(contact.position.z()) < 1e-12;
  }
  EXPECT_TRUE(hasFacePiercingContact);
}

TEST(MeshMeshCollision, ZeroContactLimitReturnsFalse)
{
  MeshShape mesh1 = makeUnitCubeMesh();
  MeshShape mesh2 = makeUnitCubeMesh();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.75, 0.0, 0.0);

  CollisionResult result;
  EXPECT_FALSE(collideMeshMesh(
      mesh1,
      Eigen::Isometry3d::Identity(),
      mesh2,
      tf2,
      result,
      CollisionOption::fullContacts(0)));
  EXPECT_EQ(0u, result.numContacts());
}

TEST(MeshMeshCollision, PrimitiveMeshSphereContact)
{
  SphereShape sphere(0.5);
  MeshShape mesh = makePlaneMesh();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(0.25, -0.25, 0.25);

  CollisionResult result;
  EXPECT_TRUE(collidePrimitiveMesh(
      sphere,
      tfSphere,
      mesh,
      Eigen::Isometry3d::Identity(),
      result,
      CollisionOption::fullContacts(4)));
  ASSERT_GT(result.numContacts(), 0u);

  const auto& contact = result.getContact(0);
  EXPECT_LT(contact.normal.z(), -0.99);
  EXPECT_NEAR(0.25, contact.depth, 1e-12);
  EXPECT_GE(contact.featureIndex1, 0);
  EXPECT_EQ(-1, contact.featureIndex2);
}

TEST(MeshMeshCollision, PrimitiveMeshCapsuleContact)
{
  CapsuleShape capsule(0.25, 0.5);
  MeshShape mesh = makePlaneMesh();
  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  tfCapsule.translation() = Eigen::Vector3d(0.25, -0.25, 0.25);

  CollisionResult result;
  EXPECT_TRUE(collidePrimitiveMesh(
      capsule,
      tfCapsule,
      mesh,
      Eigen::Isometry3d::Identity(),
      result,
      CollisionOption::fullContacts(4)));
  ASSERT_GT(result.numContacts(), 0u);

  const auto& contact = result.getContact(0);
  EXPECT_LT(contact.normal.z(), -0.1);
  EXPECT_GE(contact.featureIndex1, 0);
  EXPECT_EQ(-1, contact.featureIndex2);
}

TEST(MeshMeshCollision, PrimitiveMeshSupportContactsUsePrimitiveToMeshNormal)
{
  MeshShape mesh = makePlaneMesh();
  BoxShape box(Eigen::Vector3d::Constant(0.25));
  CylinderShape cylinder(0.25, 0.5);
  ConvexShape convex(
      {{0.25, 0.0, 0.0},
       {-0.25, 0.0, 0.0},
       {0.0, 0.25, 0.0},
       {0.0, -0.25, 0.0},
       {0.0, 0.0, 0.25},
       {0.0, 0.0, -0.25}});

  for (const Shape* primitive : {
           static_cast<const Shape*>(&box),
           static_cast<const Shape*>(&cylinder),
           static_cast<const Shape*>(&convex),
       }) {
    CollisionResult result;
    EXPECT_TRUE(collidePrimitiveMesh(
        *primitive,
        translated(0.0, 0.0, 0.2),
        mesh,
        Eigen::Isometry3d::Identity(),
        result,
        CollisionOption::fullContacts(4)));
    ASSERT_GT(result.numContacts(), 0u);
    EXPECT_LT(result.getContact(0).normal.z(), -0.1);
    EXPECT_GE(result.getContact(0).featureIndex1, 0);
    EXPECT_EQ(-1, result.getContact(0).featureIndex2);
  }
}

TEST(MeshMeshCollision, MeshMeshBatchCollidesPairs)
{
  MeshShape mesh1 = makeUnitCubeMesh();
  MeshShape mesh2 = makeUnitCubeMesh();
  const std::array<MeshPair, 2> pairs{{
      {&mesh1,
       &mesh2,
       Eigen::Isometry3d::Identity(),
       [] {
         Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
         tf.translation() = Eigen::Vector3d(0.75, 0.0, 0.0);
         return tf;
       }()},
      {&mesh1,
       &mesh2,
       Eigen::Isometry3d::Identity(),
       [] {
         Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
         tf.translation() = Eigen::Vector3d(3.0, 0.0, 0.0);
         return tf;
       }()},
  }};
  std::array<CollisionResult, 2> results;

  collideMeshMeshBatch(pairs, results, CollisionOption::fullContacts(4));

  EXPECT_GT(results[0].numContacts(), 0u);
  EXPECT_EQ(0u, results[1].numContacts());
}

TEST(MeshMeshCollision, MeshMeshBatchRejectsInvalidInputs)
{
  MeshShape mesh = makeUnitCubeMesh();
  const std::array<MeshPair, 1> pairs{{
      {&mesh,
       &mesh,
       Eigen::Isometry3d::Identity(),
       Eigen::Isometry3d::Identity()},
  }};

  EXPECT_THROW(
      collideMeshMeshBatch(pairs, span<CollisionResult>()),
      std::invalid_argument);

  std::array<CollisionResult, 1> results;
  const std::array<MeshPair, 1> nullPairs{{
      {nullptr,
       &mesh,
       Eigen::Isometry3d::Identity(),
       Eigen::Isometry3d::Identity()},
  }};
  EXPECT_THROW(collideMeshMeshBatch(nullPairs, results), std::invalid_argument);
}

TEST(MeshMeshCollision, DistanceMeshMeshReportsSeparatedDistance)
{
  MeshShape mesh1 = makeUnitCubeMesh();
  MeshShape mesh2 = makeUnitCubeMesh();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(3.0, 0.0, 0.0);

  DistanceOption option;
  option.enableNearestPoints = true;
  DistanceResult result;
  const double distance = distanceMeshMesh(
      mesh1, Eigen::Isometry3d::Identity(), mesh2, tf2, result, option);

  EXPECT_NEAR(2.0, distance, 1e-12);
  EXPECT_NEAR(2.0, result.distance, 1e-12);
  EXPECT_NEAR(0.5, result.pointOnObject1.x(), 1e-12);
  EXPECT_NEAR(2.5, result.pointOnObject2.x(), 1e-12);
}

TEST(MeshMeshCollision, PlaneMeshContact)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  MeshShape mesh = makePlaneMesh(-0.1);

  CollisionResult result;
  EXPECT_TRUE(collidePlaneMesh(
      plane,
      Eigen::Isometry3d::Identity(),
      mesh,
      Eigen::Isometry3d::Identity(),
      result,
      CollisionOption::fullContacts(4)));
  ASSERT_GT(result.numContacts(), 0u);

  const auto& contact = result.getContact(0);
  EXPECT_GT(contact.normal.z(), 0.99);
  EXPECT_NEAR(0.1, contact.depth, 1e-12);
  EXPECT_EQ(-1, contact.featureIndex1);
  EXPECT_GE(contact.featureIndex2, 0);
}

TEST(MeshMeshCollision, BinaryChecksDoNotAddContacts)
{
  MeshShape cube1 = makeUnitCubeMesh();
  MeshShape cube2 = makeUnitCubeMesh();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.75, 0.0, 0.0);

  CollisionResult meshMesh;
  EXPECT_TRUE(collideMeshMesh(
      cube1,
      Eigen::Isometry3d::Identity(),
      cube2,
      tf2,
      meshMesh,
      CollisionOption::binaryCheck()));
  EXPECT_EQ(0u, meshMesh.numContacts());

  SphereShape sphere(0.5);
  MeshShape planeMesh = makePlaneMesh();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(0.25, -0.25, 0.25);

  CollisionResult primitiveMesh;
  EXPECT_TRUE(collidePrimitiveMesh(
      sphere,
      tfSphere,
      planeMesh,
      Eigen::Isometry3d::Identity(),
      primitiveMesh,
      CollisionOption::binaryCheck()));
  EXPECT_EQ(0u, primitiveMesh.numContacts());

  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  CollisionResult planeMeshResult;
  EXPECT_TRUE(collidePlaneMesh(
      plane,
      Eigen::Isometry3d::Identity(),
      makePlaneMesh(-0.1),
      Eigen::Isometry3d::Identity(),
      planeMeshResult,
      CollisionOption::binaryCheck()));
  EXPECT_EQ(0u, planeMeshResult.numContacts());
}
