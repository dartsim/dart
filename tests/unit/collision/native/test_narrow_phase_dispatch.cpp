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

#include <dart/collision/native/narrow_phase/narrow_phase.hpp>
#include <dart/collision/native/shapes/shape.hpp>
#include <dart/collision/native/types.hpp>

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

TEST(NarrowPhaseDispatch, ReturnsFalseForUnroutedPairs)
{
  SphereShape sphere(1.0);
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);

  CollisionResult result;
  const bool hit = NarrowPhase::collide(
      &plane,
      Eigen::Isometry3d::Identity(),
      &sphere,
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
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);

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
       &plane,
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

TEST(NarrowPhaseDispatch, ReportsP6SupportedPairs)
{
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Sphere, ShapeType::Sphere));
  EXPECT_TRUE(NarrowPhase::isSupported(ShapeType::Box, ShapeType::Box));
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

  EXPECT_FALSE(NarrowPhase::isSupported(ShapeType::Sphere, ShapeType::Plane));
  EXPECT_FALSE(NarrowPhase::isSupported(ShapeType::Plane, ShapeType::Sphere));
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
