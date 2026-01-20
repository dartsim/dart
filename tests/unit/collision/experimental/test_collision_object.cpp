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
#include <dart/collision/experimental/shapes/shape.hpp>

#include <gtest/gtest.h>

using namespace dart::collision::experimental;

TEST(CollisionObject, Construction)
{
  CollisionWorld world;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(1, 2, 3);

  auto obj = world.createObject(std::make_unique<SphereShape>(1.0), transform);

  EXPECT_TRUE(obj.isValid());
  EXPECT_NE(obj.getShape(), nullptr);
  EXPECT_EQ(obj.getShapeType(), ShapeType::Sphere);
  EXPECT_TRUE(obj.getTransform().isApprox(transform));
}

TEST(CollisionObject, UniqueEntities)
{
  CollisionWorld world;

  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));
  auto obj2 = world.createObject(std::make_unique<SphereShape>(2.0));

  EXPECT_NE(obj1.getEntity(), obj2.getEntity());
  EXPECT_NE(obj1, obj2);
}

TEST(CollisionObject, SetTransform)
{
  CollisionWorld world;
  auto obj = world.createObject(std::make_unique<SphereShape>(1.0));

  Eigen::Isometry3d newTransform = Eigen::Isometry3d::Identity();
  newTransform.translation() = Eigen::Vector3d(5, 6, 7);

  obj.setTransform(newTransform);

  EXPECT_TRUE(obj.getTransform().isApprox(newTransform));
}

TEST(CollisionObject, ComputeAabb_Sphere)
{
  CollisionWorld world;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(10, 0, 0);

  auto obj = world.createObject(std::make_unique<SphereShape>(1.0), transform);

  Aabb aabb = obj.computeAabb();

  EXPECT_NEAR(aabb.min.x(), 9.0, 1e-10);
  EXPECT_NEAR(aabb.max.x(), 11.0, 1e-10);
  EXPECT_NEAR(aabb.min.y(), -1.0, 1e-10);
  EXPECT_NEAR(aabb.max.y(), 1.0, 1e-10);
  EXPECT_NEAR(aabb.min.z(), -1.0, 1e-10);
  EXPECT_NEAR(aabb.max.z(), 1.0, 1e-10);
}

TEST(CollisionObject, ComputeAabb_Box)
{
  CollisionWorld world;
  auto obj = world.createObject(
      std::make_unique<BoxShape>(Eigen::Vector3d(1, 2, 3)));

  Aabb aabb = obj.computeAabb();

  EXPECT_NEAR(aabb.min.x(), -1.0, 1e-10);
  EXPECT_NEAR(aabb.max.x(), 1.0, 1e-10);
  EXPECT_NEAR(aabb.min.y(), -2.0, 1e-10);
  EXPECT_NEAR(aabb.max.y(), 2.0, 1e-10);
  EXPECT_NEAR(aabb.min.z(), -3.0, 1e-10);
  EXPECT_NEAR(aabb.max.z(), 3.0, 1e-10);
}

TEST(CollisionObject, ComputeAabb_RotatedBox)
{
  CollisionWorld world;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));

  auto obj = world.createObject(
      std::make_unique<BoxShape>(Eigen::Vector3d(1, 0.5, 0.5)), transform);

  Aabb aabb = obj.computeAabb();

  EXPECT_NEAR(aabb.min.x(), -0.5, 1e-10);
  EXPECT_NEAR(aabb.max.x(), 0.5, 1e-10);
  EXPECT_NEAR(aabb.min.y(), -1.0, 1e-10);
  EXPECT_NEAR(aabb.max.y(), 1.0, 1e-10);
}

TEST(CollisionObject, UserData)
{
  CollisionWorld world;
  auto obj = world.createObject(std::make_unique<SphereShape>(1.0));

  EXPECT_EQ(obj.getUserData(), nullptr);

  int testData = 42;
  obj.setUserData(&testData);

  EXPECT_EQ(obj.getUserData(), &testData);
  EXPECT_EQ(*static_cast<int*>(obj.getUserData()), 42);
}

TEST(CollisionObject, InvalidHandle)
{
  CollisionObject obj;

  EXPECT_FALSE(obj.isValid());
  EXPECT_EQ(obj.getShape(), nullptr);
  EXPECT_EQ(obj.getWorld(), nullptr);

  Aabb aabb = obj.computeAabb();
  EXPECT_TRUE(aabb.min.isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(aabb.max.isApprox(Eigen::Vector3d::Zero()));
}

TEST(CollisionObject, AabbUpdatesWithTransform)
{
  CollisionWorld world;
  auto obj = world.createObject(std::make_unique<SphereShape>(1.0));

  Aabb aabb1 = obj.computeAabb();
  EXPECT_NEAR(aabb1.center().x(), 0.0, 1e-10);

  Eigen::Isometry3d newTransform = Eigen::Isometry3d::Identity();
  newTransform.translation() = Eigen::Vector3d(100, 0, 0);
  obj.setTransform(newTransform);

  Aabb aabb2 = obj.computeAabb();
  EXPECT_NEAR(aabb2.center().x(), 100.0, 1e-10);
}

TEST(CollisionObject, HandleCopyable)
{
  CollisionWorld world;
  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));

  CollisionObject obj2 = obj1;

  EXPECT_EQ(obj1.getEntity(), obj2.getEntity());
  EXPECT_EQ(obj1.getWorld(), obj2.getWorld());
  EXPECT_EQ(obj1, obj2);
}

TEST(CollisionObject, HandleInvalidatedAfterDestroy)
{
  CollisionWorld world;
  auto obj = world.createObject(std::make_unique<SphereShape>(1.0));

  EXPECT_TRUE(obj.isValid());
  EXPECT_EQ(world.numObjects(), 1u);

  world.destroyObject(obj);

  EXPECT_FALSE(obj.isValid());
  EXPECT_EQ(world.numObjects(), 0u);
}

TEST(CollisionObject, Comparison)
{
  CollisionWorld world;
  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0));

  EXPECT_EQ(obj1, obj1);
  EXPECT_NE(obj1, obj2);
  EXPECT_TRUE(obj1 < obj2 || obj2 < obj1);
}
