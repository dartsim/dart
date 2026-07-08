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
#include <dart/collision/native/narrow_phase/NarrowPhase.hpp>
#include <dart/collision/native/shapes/Shape.hpp>

#include <gtest/gtest.h>

#include <array>
#include <stdexcept>
#include <vector>

using namespace dart::collision::native;

namespace {

Eigen::Isometry3d translated(double x, double y, double z)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(x, y, z);
  return tf;
}

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

MeshShape makeSquareRingMesh()
{
  std::vector<Eigen::Vector3d> vertices
      = {{-2.0, -2.0, 0.0},
         {2.0, -2.0, 0.0},
         {2.0, 2.0, 0.0},
         {-2.0, 2.0, 0.0},
         {-0.5, -0.5, 0.0},
         {0.5, -0.5, 0.0},
         {0.5, 0.5, 0.0},
         {-0.5, 0.5, 0.0}};
  std::vector<MeshShape::Triangle> triangles
      = {{0, 1, 5},
         {0, 5, 4},
         {1, 2, 6},
         {1, 6, 5},
         {2, 3, 7},
         {2, 7, 6},
         {3, 0, 4},
         {3, 4, 7}};
  return MeshShape(vertices, triangles);
}

void expectSingleNormal(
    const Shape& shape1,
    const Eigen::Isometry3d& tf1,
    const Shape& shape2,
    const Eigen::Isometry3d& tf2,
    const Eigen::Vector3d& expectedNormal)
{
  CollisionResult result;
  const bool hit = NarrowPhase::collide(
      &shape1, tf1, &shape2, tf2, CollisionOption(), result);

  ASSERT_TRUE(hit);
  ASSERT_GE(result.numContacts(), 1u);
  EXPECT_TRUE(result.getContact(0).normal.isApprox(expectedNormal, 1e-12));
}

} // namespace

TEST(NarrowPhaseDispatch, RoutesOverlappingSphereSphere)
{
  SphereShape sphere1(1.0);
  SphereShape sphere2(1.0);

  CollisionResult result;
  const bool hit = NarrowPhase::collide(
      &sphere1,
      Eigen::Isometry3d::Identity(),
      &sphere2,
      translated(1.5, 0.0, 0.0),
      CollisionOption(),
      result);

  EXPECT_TRUE(hit);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(NarrowPhaseDispatch, ReturnsFalseForSeparatedSphereSphere)
{
  SphereShape sphere1(1.0);
  SphereShape sphere2(1.0);

  CollisionResult result;
  const bool hit = NarrowPhase::collide(
      &sphere1,
      Eigen::Isometry3d::Identity(),
      &sphere2,
      translated(3.0, 0.0, 0.0),
      CollisionOption(),
      result);

  EXPECT_FALSE(hit);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(NarrowPhaseDispatch, RoutesOverlappingBoxBox)
{
  BoxShape box1(Eigen::Vector3d(1.0, 1.0, 1.0));
  BoxShape box2(Eigen::Vector3d(1.0, 1.0, 1.0));

  CollisionResult result;
  const bool hit = NarrowPhase::collide(
      &box1,
      Eigen::Isometry3d::Identity(),
      &box2,
      translated(0.5, 0.0, 0.0),
      CollisionOption(),
      result);

  EXPECT_TRUE(hit);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(NarrowPhaseDispatch, RoutesSphereBoxInBothOrders)
{
  SphereShape sphere(1.0);
  BoxShape box(Eigen::Vector3d(1.0, 1.0, 1.0));

  CollisionResult sphereFirstResult;
  const bool sphereFirstHit = NarrowPhase::collide(
      &sphere,
      translated(0.0, 0.0, 1.5),
      &box,
      Eigen::Isometry3d::Identity(),
      CollisionOption(),
      sphereFirstResult);

  ASSERT_TRUE(sphereFirstHit);
  ASSERT_EQ(1u, sphereFirstResult.numContacts());
  EXPECT_TRUE(sphereFirstResult.getContact(0).normal.isApprox(
      Eigen::Vector3d::UnitZ(), 1e-12));

  CollisionResult boxFirstResult;
  const bool boxFirstHit = NarrowPhase::collide(
      &box,
      Eigen::Isometry3d::Identity(),
      &sphere,
      translated(0.0, 0.0, 1.5),
      CollisionOption(),
      boxFirstResult);

  ASSERT_TRUE(boxFirstHit);
  ASSERT_EQ(1u, boxFirstResult.numContacts());
  EXPECT_TRUE(boxFirstResult.getContact(0).normal.isApprox(
      -Eigen::Vector3d::UnitZ(), 1e-12));
}

TEST(NarrowPhaseDispatch, SphereBoxBinaryCheckDoesNotAddContacts)
{
  SphereShape sphere(1.0);
  BoxShape box(Eigen::Vector3d(1.0, 1.0, 1.0));
  CollisionOption option = CollisionOption::binaryCheck();

  CollisionResult sphereFirstResult;
  const bool sphereFirstHit = NarrowPhase::collide(
      &sphere,
      translated(0.0, 0.0, 1.5),
      &box,
      Eigen::Isometry3d::Identity(),
      option,
      sphereFirstResult);

  EXPECT_TRUE(sphereFirstHit);
  EXPECT_EQ(sphereFirstResult.numContacts(), 0u);

  CollisionResult boxFirstResult;
  const bool boxFirstHit = NarrowPhase::collide(
      &box,
      Eigen::Isometry3d::Identity(),
      &sphere,
      translated(0.0, 0.0, 1.5),
      option,
      boxFirstResult);

  EXPECT_TRUE(boxFirstHit);
  EXPECT_EQ(boxFirstResult.numContacts(), 0u);
}

TEST(NarrowPhaseDispatch, ReturnsFalseForUnsupportedPairs)
{
  SphereShape sphere(1.0);
  SdfShape sdf(nullptr);

  CollisionResult result;
  const bool hit = NarrowPhase::collide(
      &sphere,
      Eigen::Isometry3d::Identity(),
      &sdf,
      Eigen::Isometry3d::Identity(),
      CollisionOption(),
      result);

  EXPECT_FALSE(hit);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(NarrowPhaseDispatch, ReturnsFalseForNullShape)
{
  SphereShape sphere(1.0);

  CollisionResult result;
  EXPECT_FALSE(NarrowPhase::collide(
      nullptr,
      Eigen::Isometry3d::Identity(),
      &sphere,
      Eigen::Isometry3d::Identity(),
      CollisionOption(),
      result));
  EXPECT_FALSE(NarrowPhase::collide(
      &sphere,
      Eigen::Isometry3d::Identity(),
      nullptr,
      Eigen::Isometry3d::Identity(),
      CollisionOption(),
      result));
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(NarrowPhaseDispatch, BatchWithoutHitVectorReturnsAnyHit)
{
  SphereShape sphere1(1.0);
  SphereShape sphere2(1.0);
  BoxShape box1(Eigen::Vector3d(1.0, 1.0, 1.0));
  BoxShape box2(Eigen::Vector3d(1.0, 1.0, 1.0));

  const std::array<NarrowPhasePair, 2> pairs{{
      {&sphere1,
       &sphere2,
       Eigen::Isometry3d::Identity(),
       translated(3.0, 0.0, 0.0)},
      {&box1, &box2, Eigen::Isometry3d::Identity(), translated(0.5, 0.0, 0.0)},
  }};
  std::array<CollisionResult, 2> results;

  const bool hit = NarrowPhase::collideBatch(pairs, results, CollisionOption());

  EXPECT_TRUE(hit);
  EXPECT_EQ(results[0].numContacts(), 0u);
  EXPECT_GE(results[1].numContacts(), 1u);
}

TEST(NarrowPhaseDispatch, BatchRecordsPerPairHits)
{
  SphereShape sphere1(1.0);
  SphereShape sphere2(1.0);
  SdfShape sdf(nullptr);

  const std::array<NarrowPhasePair, 3> pairs{{
      {&sphere1,
       &sphere2,
       Eigen::Isometry3d::Identity(),
       translated(1.5, 0.0, 0.0)},
      {&sphere1,
       &sphere2,
       Eigen::Isometry3d::Identity(),
       translated(3.0, 0.0, 0.0)},
      {&sphere1,
       &sdf,
       Eigen::Isometry3d::Identity(),
       Eigen::Isometry3d::Identity()},
  }};
  std::array<CollisionResult, 3> results;
  std::array<bool, 3> hits{{false, true, true}};

  const bool anyHit
      = NarrowPhase::collideBatch(pairs, results, hits, CollisionOption());

  EXPECT_TRUE(anyHit);
  EXPECT_TRUE(hits[0]);
  EXPECT_FALSE(hits[1]);
  EXPECT_FALSE(hits[2]);
  EXPECT_GE(results[0].numContacts(), 1u);
  EXPECT_EQ(results[1].numContacts(), 0u);
  EXPECT_EQ(results[2].numContacts(), 0u);
}

TEST(NarrowPhaseDispatch, BatchRejectsMismatchedOutputSpans)
{
  SphereShape sphere1(1.0);
  SphereShape sphere2(1.0);

  const std::array<NarrowPhasePair, 1> pairs{{
      {&sphere1,
       &sphere2,
       Eigen::Isometry3d::Identity(),
       translated(1.5, 0.0, 0.0)},
  }};

  EXPECT_THROW(
      NarrowPhase::collideBatch(
          pairs, span<CollisionResult>(), CollisionOption()),
      std::invalid_argument);

  std::array<CollisionResult, 1> results;
  EXPECT_THROW(
      NarrowPhase::collideBatch(
          pairs, results, span<bool>(), CollisionOption()),
      std::invalid_argument);
}

TEST(NarrowPhaseDispatch, BatchRejectsNullShapes)
{
  SphereShape sphere(1.0);

  const std::array<NarrowPhasePair, 1> pairs{{
      {nullptr,
       &sphere,
       Eigen::Isometry3d::Identity(),
       Eigen::Isometry3d::Identity()},
  }};
  std::array<CollisionResult, 1> results;

  EXPECT_THROW(
      NarrowPhase::collideBatch(pairs, results, CollisionOption()),
      std::invalid_argument);
}

TEST(NarrowPhaseDispatch, ReportsP9SupportedPairs)
{
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Sphere, ShapeType::Sphere));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Box, ShapeType::Box));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Plane, ShapeType::Sphere));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Sphere, ShapeType::Plane));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Plane, ShapeType::Box));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Box, ShapeType::Plane));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Plane, ShapeType::Capsule));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Capsule, ShapeType::Plane));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Plane, ShapeType::Convex));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Convex, ShapeType::Plane));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Sphere, ShapeType::Box));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Box, ShapeType::Sphere));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Capsule, ShapeType::Sphere));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Sphere, ShapeType::Capsule));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Capsule, ShapeType::Box));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Box, ShapeType::Capsule));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Capsule, ShapeType::Capsule));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Convex, ShapeType::Convex));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Convex, ShapeType::Sphere));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Sphere, ShapeType::Convex));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Convex, ShapeType::Box));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Capsule, ShapeType::Convex));
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
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Convex, ShapeType::Cylinder));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Cylinder, ShapeType::Convex));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Mesh, ShapeType::Mesh));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Mesh, ShapeType::Sphere));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Sphere, ShapeType::Mesh));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Mesh, ShapeType::Box));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Capsule, ShapeType::Mesh));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Cylinder, ShapeType::Mesh));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Mesh, ShapeType::Plane));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Plane, ShapeType::Mesh));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Convex, ShapeType::Mesh));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Mesh, ShapeType::Convex));

  EXPECT_FALSE(NarrowPhase::isSupported(ShapeType::Mesh, ShapeType::Sdf));
  EXPECT_FALSE(NarrowPhase::isSupported(ShapeType::Compound, ShapeType::Mesh));
}

TEST(NarrowPhaseDispatch, ReportsOnlySupportedPlaneDistanceRows)
{
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Plane, ShapeType::Sphere));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Sphere, ShapeType::Plane));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Plane, ShapeType::Box));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Capsule, ShapeType::Plane));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Plane, ShapeType::Cylinder));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Convex, ShapeType::Plane));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Mesh, ShapeType::Plane));

  EXPECT_FALSE(
      NarrowPhase::isDistanceSupported(ShapeType::Plane, ShapeType::Plane));
  EXPECT_FALSE(
      NarrowPhase::isDistanceSupported(ShapeType::Plane, ShapeType::Sdf));
  EXPECT_FALSE(
      NarrowPhase::isDistanceSupported(ShapeType::Sdf, ShapeType::Plane));

  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  SdfShape sdf(nullptr);
  DistanceResult result;

  EXPECT_EQ(
      std::numeric_limits<double>::max(),
      NarrowPhase::distance(
          &plane,
          Eigen::Isometry3d::Identity(),
          &plane,
          Eigen::Isometry3d::Identity(),
          DistanceOption(),
          result));
  EXPECT_FALSE(result.isValid());

  result.clear();
  EXPECT_EQ(
      std::numeric_limits<double>::max(),
      NarrowPhase::distance(
          &plane,
          Eigen::Isometry3d::Identity(),
          &sdf,
          Eigen::Isometry3d::Identity(),
          DistanceOption(),
          result));
  EXPECT_FALSE(result.isValid());
}

TEST(NarrowPhaseDispatch, RoutesCapsuleSphereInBothOrders)
{
  CapsuleShape capsule(0.5, 2.0);
  SphereShape sphere(0.5);

  CollisionResult capsuleFirstResult;
  const bool capsuleFirstHit = NarrowPhase::collide(
      &capsule,
      Eigen::Isometry3d::Identity(),
      &sphere,
      translated(0.75, 0.0, 0.0),
      CollisionOption(),
      capsuleFirstResult);

  ASSERT_TRUE(capsuleFirstHit);
  ASSERT_EQ(1u, capsuleFirstResult.numContacts());
  EXPECT_TRUE(capsuleFirstResult.getContact(0).normal.isApprox(
      -Eigen::Vector3d::UnitX(), 1e-12));

  CollisionResult sphereFirstResult;
  const bool sphereFirstHit = NarrowPhase::collide(
      &sphere,
      translated(0.75, 0.0, 0.0),
      &capsule,
      Eigen::Isometry3d::Identity(),
      CollisionOption(),
      sphereFirstResult);

  ASSERT_TRUE(sphereFirstHit);
  ASSERT_EQ(1u, sphereFirstResult.numContacts());
  EXPECT_TRUE(sphereFirstResult.getContact(0).normal.isApprox(
      Eigen::Vector3d::UnitX(), 1e-12));
}

TEST(NarrowPhaseDispatch, CapsuleSphereBinaryCheckDoesNotAddContacts)
{
  CapsuleShape capsule(0.5, 2.0);
  SphereShape sphere(0.5);
  CollisionOption option = CollisionOption::binaryCheck();

  CollisionResult capsuleFirstResult;
  const bool capsuleFirstHit = NarrowPhase::collide(
      &capsule,
      Eigen::Isometry3d::Identity(),
      &sphere,
      translated(0.75, 0.0, 0.0),
      option,
      capsuleFirstResult);

  EXPECT_TRUE(capsuleFirstHit);
  EXPECT_EQ(0u, capsuleFirstResult.numContacts());

  CollisionResult sphereFirstResult;
  const bool sphereFirstHit = NarrowPhase::collide(
      &sphere,
      translated(0.75, 0.0, 0.0),
      &capsule,
      Eigen::Isometry3d::Identity(),
      option,
      sphereFirstResult);

  EXPECT_TRUE(sphereFirstHit);
  EXPECT_EQ(0u, sphereFirstResult.numContacts());
}

TEST(NarrowPhaseDispatch, RoutesCapsuleBoxInBothOrders)
{
  CapsuleShape capsule(0.5, 2.0);
  BoxShape box(Eigen::Vector3d(1.0, 1.0, 1.0));

  CollisionResult capsuleFirstResult;
  const bool capsuleFirstHit = NarrowPhase::collide(
      &capsule,
      translated(1.25, 0.0, 0.0),
      &box,
      Eigen::Isometry3d::Identity(),
      CollisionOption(),
      capsuleFirstResult);

  ASSERT_TRUE(capsuleFirstHit);
  ASSERT_GE(capsuleFirstResult.numContacts(), 1u);
  EXPECT_TRUE(capsuleFirstResult.getContact(0).normal.isApprox(
      Eigen::Vector3d::UnitX(), 1e-12));

  CollisionResult boxFirstResult;
  const bool boxFirstHit = NarrowPhase::collide(
      &box,
      Eigen::Isometry3d::Identity(),
      &capsule,
      translated(1.25, 0.0, 0.0),
      CollisionOption(),
      boxFirstResult);

  ASSERT_TRUE(boxFirstHit);
  ASSERT_GE(boxFirstResult.numContacts(), 1u);
  EXPECT_TRUE(boxFirstResult.getContact(0).normal.isApprox(
      -Eigen::Vector3d::UnitX(), 1e-12));
}

TEST(NarrowPhaseDispatch, CapsuleBoxBinaryCheckDoesNotAddContacts)
{
  CapsuleShape capsule(0.5, 2.0);
  BoxShape box(Eigen::Vector3d(1.0, 1.0, 1.0));
  CollisionOption option = CollisionOption::binaryCheck();

  CollisionResult capsuleFirstResult;
  const bool capsuleFirstHit = NarrowPhase::collide(
      &capsule,
      translated(1.25, 0.0, 0.0),
      &box,
      Eigen::Isometry3d::Identity(),
      option,
      capsuleFirstResult);

  EXPECT_TRUE(capsuleFirstHit);
  EXPECT_EQ(0u, capsuleFirstResult.numContacts());

  CollisionResult boxFirstResult;
  const bool boxFirstHit = NarrowPhase::collide(
      &box,
      Eigen::Isometry3d::Identity(),
      &capsule,
      translated(1.25, 0.0, 0.0),
      option,
      boxFirstResult);

  EXPECT_TRUE(boxFirstHit);
  EXPECT_EQ(0u, boxFirstResult.numContacts());
}

TEST(NarrowPhaseDispatch, RoutesCapsuleCapsule)
{
  CapsuleShape capsule1(0.5, 2.0);
  CapsuleShape capsule2(0.5, 2.0);

  CollisionResult result;
  const bool hit = NarrowPhase::collide(
      &capsule1,
      Eigen::Isometry3d::Identity(),
      &capsule2,
      translated(0.75, 0.0, 0.0),
      CollisionOption(),
      result);

  ASSERT_TRUE(hit);
  ASSERT_EQ(1u, result.numContacts());
  EXPECT_TRUE(
      result.getContact(0).normal.isApprox(-Eigen::Vector3d::UnitX(), 1e-12));
}

TEST(NarrowPhaseDispatch, CapsuleCapsuleBinaryCheckDoesNotAddContacts)
{
  CapsuleShape capsule1(0.5, 2.0);
  CapsuleShape capsule2(0.5, 2.0);

  CollisionResult result;
  const bool hit = NarrowPhase::collide(
      &capsule1,
      Eigen::Isometry3d::Identity(),
      &capsule2,
      translated(0.75, 0.0, 0.0),
      CollisionOption::binaryCheck(),
      result);

  EXPECT_TRUE(hit);
  EXPECT_EQ(0u, result.numContacts());
}

TEST(NarrowPhaseDispatch, RoutesCylinderPairsInBothOrders)
{
  CylinderShape cylinder(0.5, 2.0);
  SphereShape sphere(0.5);
  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));
  CapsuleShape capsule(0.5, 2.0);
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);

  expectSingleNormal(
      cylinder,
      Eigen::Isometry3d::Identity(),
      cylinder,
      translated(0.75, 0.0, 0.0),
      -Eigen::Vector3d::UnitX());
  expectSingleNormal(
      cylinder,
      Eigen::Isometry3d::Identity(),
      sphere,
      translated(0.75, 0.0, 0.0),
      -Eigen::Vector3d::UnitX());
  expectSingleNormal(
      sphere,
      translated(0.75, 0.0, 0.0),
      cylinder,
      Eigen::Isometry3d::Identity(),
      Eigen::Vector3d::UnitX());
  expectSingleNormal(
      cylinder,
      Eigen::Isometry3d::Identity(),
      box,
      translated(0.75, 0.0, 0.0),
      -Eigen::Vector3d::UnitX());
  expectSingleNormal(
      box,
      translated(0.75, 0.0, 0.0),
      cylinder,
      Eigen::Isometry3d::Identity(),
      Eigen::Vector3d::UnitX());
  expectSingleNormal(
      cylinder,
      Eigen::Isometry3d::Identity(),
      capsule,
      translated(0.75, 0.0, 0.0),
      -Eigen::Vector3d::UnitX());
  expectSingleNormal(
      capsule,
      translated(0.75, 0.0, 0.0),
      cylinder,
      Eigen::Isometry3d::Identity(),
      Eigen::Vector3d::UnitX());
  expectSingleNormal(
      cylinder,
      translated(0.0, 0.0, 0.75),
      plane,
      Eigen::Isometry3d::Identity(),
      Eigen::Vector3d::UnitZ());
  expectSingleNormal(
      plane,
      Eigen::Isometry3d::Identity(),
      cylinder,
      translated(0.0, 0.0, 0.75),
      -Eigen::Vector3d::UnitZ());
}

TEST(NarrowPhaseDispatch, FlippedCylinderBoxPreservesCapPatchManifold)
{
  CylinderShape cylinder(0.5, 2.0);
  BoxShape box(Eigen::Vector3d(1.0, 1.0, 0.5));
  const Eigen::Isometry3d cylinderTransform = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d boxTransform = translated(0.0, 0.0, 1.3);

  CollisionResult cylinderFirstResult;
  const bool cylinderFirstHit = NarrowPhase::collide(
      &cylinder,
      cylinderTransform,
      &box,
      boxTransform,
      CollisionOption(),
      cylinderFirstResult);

  ASSERT_TRUE(cylinderFirstHit);
  ASSERT_EQ(1u, cylinderFirstResult.numManifolds());
  ASSERT_GT(cylinderFirstResult.numContacts(), 1u);
  EXPECT_EQ(ContactType::Patch, cylinderFirstResult.getManifold(0).getType());

  CollisionResult boxFirstResult;
  const bool boxFirstHit = NarrowPhase::collide(
      &box,
      boxTransform,
      &cylinder,
      cylinderTransform,
      CollisionOption(),
      boxFirstResult);

  ASSERT_TRUE(boxFirstHit);
  ASSERT_EQ(1u, boxFirstResult.numManifolds());
  ASSERT_EQ(cylinderFirstResult.numContacts(), boxFirstResult.numContacts());
  EXPECT_EQ(ContactType::Patch, boxFirstResult.getManifold(0).getType());
  EXPECT_TRUE(boxFirstResult.getManifold(0).getSharedNormal().isApprox(
      Eigen::Vector3d::UnitZ(), 1e-12));
}

TEST(NarrowPhaseDispatch, RoutesContainedCylinderSphereNormalsInBothOrders)
{
  CylinderShape cylinder(1.0, 4.0);
  SphereShape sphere(0.5);

  expectSingleNormal(
      cylinder,
      Eigen::Isometry3d::Identity(),
      sphere,
      translated(0.75, 0.0, 0.0),
      -Eigen::Vector3d::UnitX());
  expectSingleNormal(
      sphere,
      translated(0.75, 0.0, 0.0),
      cylinder,
      Eigen::Isometry3d::Identity(),
      Eigen::Vector3d::UnitX());
}

TEST(NarrowPhaseDispatch, RoutesContainedCylinderCapsuleNormalsInBothOrders)
{
  CylinderShape cylinder(1.0, 4.0);
  CapsuleShape capsule(0.2, 1.0);

  expectSingleNormal(
      cylinder,
      Eigen::Isometry3d::Identity(),
      capsule,
      translated(0.9, 0.0, 0.0),
      -Eigen::Vector3d::UnitX());
  expectSingleNormal(
      capsule,
      translated(0.9, 0.0, 0.0),
      cylinder,
      Eigen::Isometry3d::Identity(),
      Eigen::Vector3d::UnitX());
}

TEST(NarrowPhaseDispatch, CylinderPairsBinaryCheckDoesNotAddContacts)
{
  CylinderShape cylinder(0.5, 2.0);
  SphereShape sphere(0.5);
  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));
  CollisionOption option = CollisionOption::binaryCheck();

  CollisionResult sphereFirstResult;
  const bool sphereFirstHit = NarrowPhase::collide(
      &sphere,
      translated(0.75, 0.0, 0.0),
      &cylinder,
      Eigen::Isometry3d::Identity(),
      option,
      sphereFirstResult);

  EXPECT_TRUE(sphereFirstHit);
  EXPECT_EQ(0u, sphereFirstResult.numContacts());

  CollisionResult boxFirstResult;
  const bool boxFirstHit = NarrowPhase::collide(
      &box,
      translated(0.75, 0.0, 0.0),
      &cylinder,
      Eigen::Isometry3d::Identity(),
      option,
      boxFirstResult);

  EXPECT_TRUE(boxFirstHit);
  EXPECT_EQ(0u, boxFirstResult.numContacts());
}

TEST(NarrowPhaseDispatch, RoutesMeshPairsInBothOrders)
{
  MeshShape cube1 = makeUnitCubeMesh();
  MeshShape cube2 = makeUnitCubeMesh();
  MeshShape planeMesh = makePlaneMesh();
  MeshShape belowPlaneMesh1 = makePlaneMesh(-0.1);
  MeshShape belowPlaneMesh2 = makePlaneMesh(-0.1);
  SphereShape sphere(0.5);
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  ConvexShape convex(makeOctahedronVertices(0.75));

  CollisionResult meshMesh;
  EXPECT_TRUE(NarrowPhase::collide(
      &cube1,
      Eigen::Isometry3d::Identity(),
      &cube2,
      translated(0.25, 0.0, 0.0),
      CollisionOption(),
      meshMesh));
  EXPECT_GT(meshMesh.numContacts(), 0u);

  CollisionResult meshSphere;
  EXPECT_TRUE(NarrowPhase::collide(
      &planeMesh,
      Eigen::Isometry3d::Identity(),
      &sphere,
      translated(0.25, -0.25, 0.25),
      CollisionOption(),
      meshSphere));
  ASSERT_GT(meshSphere.numContacts(), 0u);
  EXPECT_LT(meshSphere.getContact(0).normal.z(), -0.99);
  EXPECT_GE(meshSphere.getContact(0).featureIndex1, 0);
  EXPECT_EQ(-1, meshSphere.getContact(0).featureIndex2);

  CollisionResult sphereMesh;
  EXPECT_TRUE(NarrowPhase::collide(
      &sphere,
      translated(0.25, -0.25, 0.25),
      &planeMesh,
      Eigen::Isometry3d::Identity(),
      CollisionOption(),
      sphereMesh));
  ASSERT_GT(sphereMesh.numContacts(), 0u);
  EXPECT_GT(sphereMesh.getContact(0).normal.z(), 0.99);
  EXPECT_EQ(-1, sphereMesh.getContact(0).featureIndex1);
  EXPECT_GE(sphereMesh.getContact(0).featureIndex2, 0);

  CollisionResult meshPlane;
  EXPECT_TRUE(NarrowPhase::collide(
      &belowPlaneMesh1,
      Eigen::Isometry3d::Identity(),
      &plane,
      Eigen::Isometry3d::Identity(),
      CollisionOption(),
      meshPlane));
  EXPECT_GT(meshPlane.numContacts(), 0u);
  EXPECT_LT(meshPlane.getContact(0).normal.z(), -0.99);
  EXPECT_GE(meshPlane.getContact(0).featureIndex1, 0);
  EXPECT_EQ(-1, meshPlane.getContact(0).featureIndex2);

  CollisionResult planeMeshResult;
  EXPECT_TRUE(NarrowPhase::collide(
      &plane,
      Eigen::Isometry3d::Identity(),
      &belowPlaneMesh2,
      Eigen::Isometry3d::Identity(),
      CollisionOption(),
      planeMeshResult));
  EXPECT_GT(planeMeshResult.numContacts(), 0u);
  EXPECT_GT(planeMeshResult.getContact(0).normal.z(), 0.99);
  EXPECT_EQ(-1, planeMeshResult.getContact(0).featureIndex1);
  EXPECT_GE(planeMeshResult.getContact(0).featureIndex2, 0);

  CollisionResult convexMesh;
  EXPECT_TRUE(NarrowPhase::collide(
      &convex,
      Eigen::Isometry3d::Identity(),
      &cube1,
      translated(0.25, 0.0, 0.0),
      CollisionOption(),
      convexMesh));
  EXPECT_GT(convexMesh.numContacts(), 0u);
  EXPECT_EQ(-1, convexMesh.getContact(0).featureIndex1);
  EXPECT_GE(convexMesh.getContact(0).featureIndex2, 0);
}

TEST(NarrowPhaseDispatch, SupportMeshPrimitiveNormalsFollowObjectOrder)
{
  MeshShape planeMesh = makePlaneMesh();
  BoxShape box(Eigen::Vector3d::Constant(0.25));
  CylinderShape cylinder(0.25, 0.5);
  ConvexShape convex(makeOctahedronVertices(0.25));

  for (const Shape* primitive : {
           static_cast<const Shape*>(&box),
           static_cast<const Shape*>(&cylinder),
           static_cast<const Shape*>(&convex),
       }) {
    CollisionResult meshPrimitive;
    EXPECT_TRUE(NarrowPhase::collide(
        &planeMesh,
        Eigen::Isometry3d::Identity(),
        primitive,
        translated(0.0, 0.0, 0.2),
        CollisionOption(),
        meshPrimitive));
    ASSERT_GT(meshPrimitive.numContacts(), 0u);
    EXPECT_LT(meshPrimitive.getContact(0).normal.z(), -0.1);
    EXPECT_GE(meshPrimitive.getContact(0).featureIndex1, 0);
    EXPECT_EQ(-1, meshPrimitive.getContact(0).featureIndex2);

    CollisionResult primitiveMesh;
    EXPECT_TRUE(NarrowPhase::collide(
        primitive,
        translated(0.0, 0.0, 0.2),
        &planeMesh,
        Eigen::Isometry3d::Identity(),
        CollisionOption(),
        primitiveMesh));
    ASSERT_GT(primitiveMesh.numContacts(), 0u);
    EXPECT_GT(primitiveMesh.getContact(0).normal.z(), 0.1);
    EXPECT_EQ(-1, primitiveMesh.getContact(0).featureIndex1);
    EXPECT_GE(primitiveMesh.getContact(0).featureIndex2, 0);
  }
}

TEST(NarrowPhaseDispatch, RoutesMeshConvexThroughTriangleTests)
{
  MeshShape ringMesh = makeSquareRingMesh();
  ConvexShape convex(makeOctahedronVertices(0.2));

  CollisionResult convexMesh;
  EXPECT_FALSE(NarrowPhase::collide(
      &convex,
      Eigen::Isometry3d::Identity(),
      &ringMesh,
      Eigen::Isometry3d::Identity(),
      CollisionOption(),
      convexMesh));
  EXPECT_EQ(0u, convexMesh.numContacts());

  CollisionResult meshConvex;
  EXPECT_FALSE(NarrowPhase::collide(
      &ringMesh,
      Eigen::Isometry3d::Identity(),
      &convex,
      Eigen::Isometry3d::Identity(),
      CollisionOption(),
      meshConvex));
  EXPECT_EQ(0u, meshConvex.numContacts());
}

TEST(NarrowPhaseDispatch, MeshPairsBinaryCheckDoesNotAddContacts)
{
  SphereShape sphere(0.5);
  MeshShape mesh = makePlaneMesh();

  CollisionResult sphereMesh;
  EXPECT_TRUE(NarrowPhase::collide(
      &sphere,
      translated(0.25, -0.25, 0.25),
      &mesh,
      Eigen::Isometry3d::Identity(),
      CollisionOption::binaryCheck(),
      sphereMesh));
  EXPECT_EQ(0u, sphereMesh.numContacts());

  CollisionResult meshSphere;
  EXPECT_TRUE(NarrowPhase::collide(
      &mesh,
      Eigen::Isometry3d::Identity(),
      &sphere,
      translated(0.25, -0.25, 0.25),
      CollisionOption::binaryCheck(),
      meshSphere));
  EXPECT_EQ(0u, meshSphere.numContacts());
}

TEST(NarrowPhaseDispatch, RoutesConvexFallbackPairs)
{
  ConvexShape convex(makeOctahedronVertices());
  SphereShape sphere(0.75);
  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));
  CapsuleShape capsule(0.5, 2.0);
  CylinderShape cylinder(0.5, 2.0);

  CollisionResult convexSphere;
  EXPECT_TRUE(NarrowPhase::collide(
      &convex,
      Eigen::Isometry3d::Identity(),
      &sphere,
      translated(1.25, 0.0, 0.0),
      CollisionOption(),
      convexSphere));
  EXPECT_EQ(1u, convexSphere.numContacts());

  CollisionResult sphereConvex;
  EXPECT_TRUE(NarrowPhase::collide(
      &sphere,
      translated(1.25, 0.0, 0.0),
      &convex,
      Eigen::Isometry3d::Identity(),
      CollisionOption(),
      sphereConvex));
  EXPECT_EQ(1u, sphereConvex.numContacts());

  CollisionResult convexBox;
  EXPECT_TRUE(NarrowPhase::collide(
      &convex,
      Eigen::Isometry3d::Identity(),
      &box,
      translated(1.25, 0.0, 0.0),
      CollisionOption(),
      convexBox));
  EXPECT_EQ(1u, convexBox.numContacts());

  CollisionResult convexCapsule;
  EXPECT_TRUE(NarrowPhase::collide(
      &convex,
      Eigen::Isometry3d::Identity(),
      &capsule,
      translated(1.25, 0.0, 0.0),
      CollisionOption(),
      convexCapsule));
  EXPECT_EQ(1u, convexCapsule.numContacts());

  CollisionResult convexCylinder;
  EXPECT_TRUE(NarrowPhase::collide(
      &convex,
      Eigen::Isometry3d::Identity(),
      &cylinder,
      translated(1.25, 0.0, 0.0),
      CollisionOption(),
      convexCylinder));
  EXPECT_EQ(1u, convexCylinder.numContacts());
}

TEST(NarrowPhaseDispatch, ConvexFallbackBinaryCheckDoesNotAddContacts)
{
  ConvexShape convex(makeOctahedronVertices());
  SphereShape sphere(0.75);

  CollisionResult result;
  const bool hit = NarrowPhase::collide(
      &convex,
      Eigen::Isometry3d::Identity(),
      &sphere,
      translated(1.25, 0.0, 0.0),
      CollisionOption::binaryCheck(),
      result);

  EXPECT_TRUE(hit);
  EXPECT_EQ(0u, result.numContacts());
}

TEST(NarrowPhaseDispatch, RoutesPlanePairsInBothOrders)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  SphereShape sphere(1.0);
  BoxShape box(Eigen::Vector3d(1.0, 1.0, 1.0));
  CapsuleShape capsule(0.5, 2.0);
  ConvexShape convex(
      {Eigen::Vector3d(-0.5, -0.5, -0.25),
       Eigen::Vector3d(0.5, -0.5, -0.25),
       Eigen::Vector3d(0.0, 0.5, -0.25),
       Eigen::Vector3d(0.0, 0.0, 0.75)});

  CollisionResult planeSphere;
  EXPECT_TRUE(NarrowPhase::collide(
      &plane,
      Eigen::Isometry3d::Identity(),
      &sphere,
      translated(0.0, 0.0, 0.5),
      CollisionOption(),
      planeSphere));
  ASSERT_EQ(1u, planeSphere.numContacts());
  EXPECT_LT(planeSphere.getContact(0).normal.z(), -0.99);

  CollisionResult spherePlane;
  EXPECT_TRUE(NarrowPhase::collide(
      &sphere,
      translated(0.0, 0.0, 0.5),
      &plane,
      Eigen::Isometry3d::Identity(),
      CollisionOption(),
      spherePlane));
  ASSERT_EQ(1u, spherePlane.numContacts());
  EXPECT_GT(spherePlane.getContact(0).normal.z(), 0.99);

  CollisionResult planeBox;
  EXPECT_TRUE(NarrowPhase::collide(
      &plane,
      Eigen::Isometry3d::Identity(),
      &box,
      translated(0.0, 0.0, 0.5),
      CollisionOption(),
      planeBox));
  EXPECT_EQ(1u, planeBox.numContacts());

  CollisionResult boxPlane;
  EXPECT_TRUE(NarrowPhase::collide(
      &box,
      translated(0.0, 0.0, 0.5),
      &plane,
      Eigen::Isometry3d::Identity(),
      CollisionOption(),
      boxPlane));
  EXPECT_EQ(1u, boxPlane.numContacts());

  CollisionResult capsulePlane;
  EXPECT_TRUE(NarrowPhase::collide(
      &capsule,
      translated(0.0, 0.0, 1.3),
      &plane,
      Eigen::Isometry3d::Identity(),
      CollisionOption(),
      capsulePlane));
  EXPECT_EQ(1u, capsulePlane.numContacts());

  CollisionResult planeCapsule;
  EXPECT_TRUE(NarrowPhase::collide(
      &plane,
      Eigen::Isometry3d::Identity(),
      &capsule,
      translated(0.0, 0.0, 1.3),
      CollisionOption(),
      planeCapsule));
  EXPECT_EQ(1u, planeCapsule.numContacts());

  CollisionResult convexPlane;
  EXPECT_TRUE(NarrowPhase::collide(
      &convex,
      Eigen::Isometry3d::Identity(),
      &plane,
      Eigen::Isometry3d::Identity(),
      CollisionOption(),
      convexPlane));
  EXPECT_EQ(1u, convexPlane.numContacts());

  CollisionResult planeConvex;
  EXPECT_TRUE(NarrowPhase::collide(
      &plane,
      Eigen::Isometry3d::Identity(),
      &convex,
      Eigen::Isometry3d::Identity(),
      CollisionOption(),
      planeConvex));
  EXPECT_EQ(1u, planeConvex.numContacts());
}

TEST(NarrowPhaseDispatch, PlanePairsBinaryCheckDoesNotAddContacts)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  SphereShape sphere(1.0);

  CollisionResult result;
  EXPECT_TRUE(NarrowPhase::collide(
      &plane,
      Eigen::Isometry3d::Identity(),
      &sphere,
      translated(0.0, 0.0, 0.5),
      CollisionOption::binaryCheck(),
      result));
  EXPECT_EQ(0u, result.numContacts());

  CollisionResult reusedResult;
  reusedResult.addContact(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), 0.0);
  EXPECT_TRUE(NarrowPhase::collide(
      &sphere,
      translated(0.0, 0.0, 0.5),
      &plane,
      Eigen::Isometry3d::Identity(),
      CollisionOption::binaryCheck(),
      reusedResult));
  EXPECT_EQ(1u, reusedResult.numContacts());
}

TEST(NarrowPhaseDispatch, RespectsExhaustedContactBudget)
{
  SphereShape sphere1(1.0);
  SphereShape sphere2(1.0);
  BoxShape box1(Eigen::Vector3d(1.0, 1.0, 1.0));
  BoxShape box2(Eigen::Vector3d(1.0, 1.0, 1.0));

  CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 0;

  CollisionResult sphereResult;
  const bool sphereHit = NarrowPhase::collide(
      &sphere1,
      Eigen::Isometry3d::Identity(),
      &sphere2,
      translated(1.5, 0.0, 0.0),
      option,
      sphereResult);

  EXPECT_FALSE(sphereHit);
  EXPECT_EQ(sphereResult.numContacts(), 0u);

  option.enableContact = false;

  CollisionResult boxResult;
  const bool boxHit = NarrowPhase::collide(
      &box1,
      Eigen::Isometry3d::Identity(),
      &box2,
      translated(0.5, 0.0, 0.0),
      option,
      boxResult);

  EXPECT_FALSE(boxHit);
  EXPECT_EQ(boxResult.numContacts(), 0u);
}
