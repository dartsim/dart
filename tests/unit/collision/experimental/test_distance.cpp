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

#include <dart/collision/experimental/collision_object.hpp>
#include <dart/collision/experimental/collision_world.hpp>
#include <dart/collision/experimental/narrow_phase/distance.hpp>
#include <dart/collision/experimental/narrow_phase/narrow_phase.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <gtest/gtest.h>

using namespace dart::collision::experimental;

TEST(DistanceSphereSphere, Separated)
{
  SphereShape s1(1.0);
  SphereShape s2(1.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(5.0, 0, 0);

  DistanceResult result;
  double dist = distanceSphereSphere(s1, tf1, s2, tf2, result);

  EXPECT_NEAR(dist, 3.0, 1e-6);
  EXPECT_NEAR(result.pointOnObject1.x(), 1.0, 1e-6);
  EXPECT_NEAR(result.pointOnObject2.x(), 4.0, 1e-6);
}

TEST(DistanceSphereSphere, Touching)
{
  SphereShape s1(1.0);
  SphereShape s2(1.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(2.0, 0, 0);

  DistanceResult result;
  double dist = distanceSphereSphere(s1, tf1, s2, tf2, result);

  EXPECT_NEAR(dist, 0.0, 1e-6);
}

TEST(DistanceSphereSphere, Penetrating)
{
  SphereShape s1(1.0);
  SphereShape s2(1.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.0, 0, 0);

  DistanceResult result;
  double dist = distanceSphereSphere(s1, tf1, s2, tf2, result);

  EXPECT_NEAR(dist, -1.0, 1e-6);
}

TEST(DistanceSphereSphere, ScaleRegimes)
{
  const double scales[] = {1e-3, 1.0, 1e3};

  for (double scale : scales) {
    SphereShape s1(scale);
    SphereShape s2(scale);

    Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
    tf2.translation() = Eigen::Vector3d(3.0 * scale, 0, 0);

    DistanceResult result;
    double dist = distanceSphereSphere(s1, tf1, s2, tf2, result);

    double tol = 1e-6;
    if (scale > 1.0) {
      tol = 1e-6 * scale;
    }

    EXPECT_NEAR(dist, scale, tol);
    EXPECT_NEAR(result.pointOnObject1.x(), scale, tol);
    EXPECT_NEAR(result.pointOnObject2.x(), 2.0 * scale, tol);
  }
}

TEST(DistanceSphereBox, Separated)
{
  SphereShape sphere(1.0);
  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));

  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(3.0, 0, 0);

  DistanceResult result;
  double dist = distanceSphereBox(sphere, tfSphere, box, tfBox, result);

  EXPECT_NEAR(dist, 1.5, 1e-6);
}

TEST(DistanceSphereBox, Touching)
{
  SphereShape sphere(1.0);
  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));

  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(1.5, 0, 0);

  DistanceResult result;
  double dist = distanceSphereBox(sphere, tfSphere, box, tfBox, result);

  EXPECT_NEAR(dist, 0.0, 1e-6);
}

TEST(DistanceSphereBox, GrazingGap)
{
  SphereShape sphere(1.0);
  BoxShape box(Eigen::Vector3d(1.0, 1.0, 1.0));

  const double gap = 1e-4;

  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(2.0 + gap, 0, 0);

  DistanceResult result;
  double dist = distanceSphereBox(sphere, tfSphere, box, tfBox, result);

  EXPECT_NEAR(dist, gap, 1e-7);
}

TEST(DistanceSphereBox, Penetrating)
{
  SphereShape sphere(1.0);
  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));

  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(1.0, 0, 0);

  DistanceResult result;
  double dist = distanceSphereBox(sphere, tfSphere, box, tfBox, result);

  EXPECT_LT(dist, 0.0);
}

TEST(DistanceBoxBox, Separated)
{
  BoxShape b1(Eigen::Vector3d(0.5, 0.5, 0.5));
  BoxShape b2(Eigen::Vector3d(0.5, 0.5, 0.5));

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(3.0, 0, 0);

  DistanceResult result;
  double dist = distanceBoxBox(b1, tf1, b2, tf2, result);

  EXPECT_NEAR(dist, 2.0, 1e-6);
}

TEST(DistanceBoxBox, ThinFeatureSeparated)
{
  BoxShape b1(Eigen::Vector3d(1.0, 1.0, 1e-3));
  BoxShape b2(Eigen::Vector3d(1.0, 1.0, 1e-3));

  const double gap = 5e-3;

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0, 0, 2e-3 + gap);

  DistanceResult result;
  double dist = distanceBoxBox(b1, tf1, b2, tf2, result);

  EXPECT_NEAR(dist, gap, 1e-6);
}

TEST(DistanceCapsuleCapsule, Separated)
{
  CapsuleShape c1(0.5, 2.0);
  CapsuleShape c2(0.5, 2.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(3.0, 0, 0);

  DistanceResult result;
  double dist = distanceCapsuleCapsule(c1, tf1, c2, tf2, result);

  EXPECT_NEAR(dist, 2.0, 1e-6);
}

TEST(DistanceCapsuleCapsule, Parallel)
{
  CapsuleShape c1(0.5, 2.0);
  CapsuleShape c2(0.5, 2.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(2.0, 0, 0);

  DistanceResult result;
  double dist = distanceCapsuleCapsule(c1, tf1, c2, tf2, result);

  EXPECT_NEAR(dist, 1.0, 1e-6);
}

TEST(DistanceCapsuleCapsule, ZeroHeight)
{
  CapsuleShape c1(0.5, 0.0);
  CapsuleShape c2(0.75, 0.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(3.0, 0, 0);

  DistanceResult result;
  double dist = distanceCapsuleCapsule(c1, tf1, c2, tf2, result);

  EXPECT_NEAR(dist, 1.75, 1e-6);
}

TEST(DistanceCapsuleSphere, Separated)
{
  CapsuleShape capsule(0.5, 2.0);
  SphereShape sphere(0.5);

  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(3.0, 0, 0);

  DistanceResult result;
  double dist
      = distanceCapsuleSphere(capsule, tfCapsule, sphere, tfSphere, result);

  EXPECT_NEAR(dist, 2.0, 1e-6);
}

TEST(DistanceCapsuleSphere, ZeroHeight)
{
  CapsuleShape capsule(0.5, 0.0);
  SphereShape sphere(0.25);

  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(2.0, 0, 0);

  DistanceResult result;
  double dist
      = distanceCapsuleSphere(capsule, tfCapsule, sphere, tfSphere, result);

  EXPECT_NEAR(dist, 1.25, 1e-6);
}

TEST(DistanceCapsuleBox, Separated)
{
  CapsuleShape capsule(0.5, 2.0);
  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));

  Eigen::Isometry3d tfCapsule = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(3.0, 0, 0);

  DistanceResult result;
  double dist = distanceCapsuleBox(capsule, tfCapsule, box, tfBox, result);

  EXPECT_NEAR(dist, 2.0, 1e-6);
}

TEST(NarrowPhaseDistance, IsDistanceSupported)
{
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Sphere, ShapeType::Sphere));
  EXPECT_TRUE(NarrowPhase::isDistanceSupported(ShapeType::Box, ShapeType::Box));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Sphere, ShapeType::Box));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Box, ShapeType::Sphere));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Capsule, ShapeType::Capsule));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Capsule, ShapeType::Sphere));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Sphere, ShapeType::Capsule));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Capsule, ShapeType::Box));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Box, ShapeType::Capsule));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Plane, ShapeType::Sphere));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Cylinder, ShapeType::Box));
}

TEST(NarrowPhaseDistance, SphereSphere)
{
  CollisionWorld world;
  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(5.0, 0, 0);
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0), tf2);

  DistanceOption option;
  DistanceResult result;

  double dist = NarrowPhase::distance(obj1, obj2, option, result);

  EXPECT_NEAR(dist, 3.0, 1e-6);
}

TEST(NarrowPhaseDistance, BoxSphere)
{
  CollisionWorld world;
  auto obj1 = world.createObject(
      std::make_unique<BoxShape>(Eigen::Vector3d(0.5, 0.5, 0.5)));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(3.0, 0, 0);
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0), tf2);

  DistanceOption option;
  DistanceResult result;

  double dist = NarrowPhase::distance(obj1, obj2, option, result);

  EXPECT_NEAR(dist, 1.5, 1e-6);
}

TEST(NarrowPhaseDistance, CylinderBoxThinHeight)
{
  CollisionWorld world;

  Eigen::Isometry3d tfCylinder = Eigen::Isometry3d::Identity();
  tfCylinder.translation() = Eigen::Vector3d(2.0, 0, 0);
  auto obj1 = world.createObject(
      std::make_unique<CylinderShape>(0.5, 1e-3), tfCylinder);

  auto obj2 = world.createObject(
      std::make_unique<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0)));

  DistanceOption option;
  DistanceResult result;

  double dist = NarrowPhase::distance(obj1, obj2, option, result);

  EXPECT_NEAR(dist, 0.5, 1e-5);
}

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

TEST(NarrowPhaseDistance, ConvexConvexSeparated)
{
  CollisionWorld world;

  auto obj1 = world.createObject(
      std::make_unique<ConvexShape>(makeOctahedronVertices(1.0)));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(5.0, 0, 0);
  auto obj2 = world.createObject(
      std::make_unique<ConvexShape>(makeOctahedronVertices(1.0)), tf2);

  DistanceOption option;
  DistanceResult result;

  double dist = NarrowPhase::distance(obj1, obj2, option, result);

  EXPECT_GT(dist, 2.9);
  EXPECT_LT(dist, 3.1);
}

TEST(NarrowPhaseDistance, ConvexSphereSeparated)
{
  CollisionWorld world;

  auto obj1 = world.createObject(
      std::make_unique<ConvexShape>(makeOctahedronVertices(1.0)));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(5.0, 0, 0);
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0), tf2);

  DistanceOption option;
  DistanceResult result;

  double dist = NarrowPhase::distance(obj1, obj2, option, result);

  EXPECT_GT(dist, 2.9);
  EXPECT_LT(dist, 3.1);
}

TEST(NarrowPhaseDistance, ConvexIntersecting)
{
  CollisionWorld world;

  auto obj1 = world.createObject(
      std::make_unique<ConvexShape>(makeOctahedronVertices(1.0)));

  // Place octahedron 2 at (1,0,0). Its leftmost vertex will be at origin,
  // which is inside octahedron 1. The shapes clearly intersect.
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.0, 0, 0);
  auto obj2 = world.createObject(
      std::make_unique<ConvexShape>(makeOctahedronVertices(1.0)), tf2);

  DistanceOption option;
  DistanceResult result;

  double dist = NarrowPhase::distance(obj1, obj2, option, result);

  // For intersecting shapes, distance is negative (penetration depth).
  // The vertex (0,0,0) of octahedron 2 is inside octahedron 1.
  // Octahedron 1's faces have equation |x|+|y|+|z|=1, so the penetration
  // depth is approximately 1.0.
  EXPECT_LT(dist, 0.0);  // Negative distance = penetration
  EXPECT_GT(dist, -2.0); // Reasonable bound on penetration depth
}

TEST(NarrowPhaseDistance, ConvexTouching)
{
  CollisionWorld world;

  auto obj1 = world.createObject(
      std::make_unique<ConvexShape>(makeOctahedronVertices(1.0)));

  // Place octahedron 2 at (2,0,0) so its leftmost vertex at (1,0,0)
  // exactly touches octahedron 1's rightmost vertex at (1,0,0).
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(2.0, 0, 0);
  auto obj2 = world.createObject(
      std::make_unique<ConvexShape>(makeOctahedronVertices(1.0)), tf2);

  DistanceOption option;
  DistanceResult result;

  double dist = NarrowPhase::distance(obj1, obj2, option, result);

  // Shapes just touch at a single point - distance should be ~0
  EXPECT_NEAR(dist, 0.0, 1e-6);
}

TEST(NarrowPhaseDistance, IsDistanceSupportedConvex)
{
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Convex, ShapeType::Convex));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Convex, ShapeType::Sphere));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Mesh, ShapeType::Mesh));
  EXPECT_TRUE(
      NarrowPhase::isDistanceSupported(ShapeType::Mesh, ShapeType::Convex));
}
