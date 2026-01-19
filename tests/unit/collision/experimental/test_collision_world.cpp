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

#include <dart/collision/experimental/collision_world.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <gtest/gtest.h>

using namespace dart::collision::experimental;

TEST(CollisionWorld, EmptyWorld)
{
  CollisionWorld world;
  EXPECT_EQ(world.numObjects(), 0u);

  CollisionOption option;
  CollisionResult result;
  EXPECT_FALSE(world.collide(option, result));
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(CollisionWorld, AddRemoveObjects)
{
  CollisionWorld world;

  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));
  EXPECT_EQ(world.numObjects(), 1u);
  EXPECT_TRUE(obj1.isValid());

  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0));
  EXPECT_EQ(world.numObjects(), 2u);

  world.destroyObject(obj1);
  EXPECT_EQ(world.numObjects(), 1u);

  world.clear();
  EXPECT_EQ(world.numObjects(), 0u);
}

TEST(CollisionWorld, TwoSpheres_Colliding)
{
  CollisionWorld world;

  auto obj1 = world.createObject(
      std::make_unique<SphereShape>(1.0), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0), tf2);

  CollisionOption option;
  CollisionResult result;

  bool hasCollision = world.collide(option, result);

  EXPECT_TRUE(hasCollision);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(CollisionWorld, TwoSpheres_Separated)
{
  CollisionWorld world;

  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(5.0, 0, 0);
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0), tf2);

  CollisionOption option;
  CollisionResult result;

  bool hasCollision = world.collide(option, result);

  EXPECT_FALSE(hasCollision);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(CollisionWorld, MultipleObjects)
{
  CollisionWorld world;

  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0), tf2);

  Eigen::Isometry3d tf3 = Eigen::Isometry3d::Identity();
  tf3.translation() = Eigen::Vector3d(10.0, 0, 0);
  auto obj3 = world.createObject(std::make_unique<SphereShape>(1.0), tf3);

  CollisionOption option;
  CollisionResult result;

  bool hasCollision = world.collide(option, result);

  EXPECT_TRUE(hasCollision);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(CollisionWorld, DirectCollide)
{
  CollisionWorld world;

  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0), tf2);

  CollisionOption option;
  CollisionResult result;

  bool hasCollision = world.collide(obj1, obj2, option, result);

  EXPECT_TRUE(hasCollision);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(CollisionWorld, MixedShapes)
{
  CollisionWorld world;

  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);
  auto obj2 = world.createObject(
      std::make_unique<BoxShape>(Eigen::Vector3d(1, 1, 1)), tf2);

  CollisionOption option;
  CollisionResult result;

  bool hasCollision = world.collide(option, result);

  EXPECT_TRUE(hasCollision);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(CollisionWorld, UpdateObject)
{
  CollisionWorld world;

  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(5.0, 0, 0);
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0), tf2);

  CollisionOption option;
  CollisionResult result;

  EXPECT_FALSE(world.collide(option, result));

  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);
  obj2.setTransform(tf2);
  world.updateObject(obj2);

  result.clear();
  EXPECT_TRUE(world.collide(option, result));
}
