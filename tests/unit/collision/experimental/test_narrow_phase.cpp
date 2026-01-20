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
#include <dart/collision/experimental/narrow_phase/narrow_phase.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <gtest/gtest.h>

using namespace dart::collision::experimental;

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
