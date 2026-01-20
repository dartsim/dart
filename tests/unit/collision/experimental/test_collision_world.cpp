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

//==============================================================================
// CollisionWorld Raycast tests
//==============================================================================

TEST(CollisionWorldRaycast, RaycastAllOrderingAndRepeatability)
{
  CollisionWorld world;

  Eigen::Isometry3d tfFar = Eigen::Isometry3d::Identity();
  tfFar.translation() = Eigen::Vector3d(6, 0, 0);
  world.createObject(std::make_unique<SphereShape>(1.0), tfFar);

  Eigen::Isometry3d tfNear = Eigen::Isometry3d::Identity();
  tfNear.translation() = Eigen::Vector3d(3, 0, 0);
  world.createObject(std::make_unique<SphereShape>(1.0), tfNear);

  Eigen::Isometry3d tfMiss = Eigen::Isometry3d::Identity();
  tfMiss.translation() = Eigen::Vector3d(0, 5, 0);
  world.createObject(std::make_unique<SphereShape>(1.0), tfMiss);

  Ray ray(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0));
  RaycastOption option;

  std::vector<RaycastResult> results;
  ASSERT_TRUE(world.raycastAll(ray, option, results));
  ASSERT_EQ(results.size(), 2u);

  EXPECT_LT(results[0].distance, results[1].distance);
  EXPECT_NEAR(results[0].distance, 2.0, 1e-10);
  EXPECT_NEAR(results[1].distance, 5.0, 1e-10);

  for (int i = 0; i < 5; ++i) {
    std::vector<RaycastResult> repeat;
    ASSERT_TRUE(world.raycastAll(ray, option, repeat));
    ASSERT_EQ(repeat.size(), results.size());
    for (std::size_t j = 0; j < repeat.size(); ++j) {
      EXPECT_EQ(repeat[j].hit, results[j].hit);
      EXPECT_NEAR(repeat[j].distance, results[j].distance, 1e-10);
      EXPECT_NEAR(repeat[j].point.x(), results[j].point.x(), 1e-10);
      EXPECT_NEAR(repeat[j].point.y(), results[j].point.y(), 1e-10);
      EXPECT_NEAR(repeat[j].point.z(), results[j].point.z(), 1e-10);
    }
  }
}

//==============================================================================
// CollisionWorld SphereCast tests
//==============================================================================

TEST(CollisionWorldSphereCast, NoHit)
{
  CollisionWorld world;

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(10, 0, 0);
  world.createObject(std::make_unique<SphereShape>(1.0), tf);

  Eigen::Vector3d start(0, 0, 0);
  Eigen::Vector3d end(0, 0, 10);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  EXPECT_FALSE(world.sphereCast(start, end, radius, option, result));
  EXPECT_FALSE(result.hit);
}

TEST(CollisionWorldSphereCast, SingleHit)
{
  CollisionWorld world;

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0, 0, 5);
  auto target = world.createObject(std::make_unique<SphereShape>(1.0), tf);

  Eigen::Vector3d start(0, 0, 0);
  Eigen::Vector3d end(0, 0, 10);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  EXPECT_TRUE(world.sphereCast(start, end, radius, option, result));
  EXPECT_TRUE(result.hit);
  EXPECT_NE(result.object, nullptr);
  EXPECT_EQ(*result.object, target);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 1.0);
}

TEST(CollisionWorldSphereCast, ClosestHit)
{
  CollisionWorld world;

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  tf1.translation() = Eigen::Vector3d(0, 0, 5);
  auto near = world.createObject(std::make_unique<SphereShape>(1.0), tf1);

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0, 0, 8);
  world.createObject(std::make_unique<SphereShape>(1.0), tf2);

  Eigen::Vector3d start(0, 0, 0);
  Eigen::Vector3d end(0, 0, 15);
  double radius = 0.5;

  CcdOption option;
  CcdResult result;

  EXPECT_TRUE(world.sphereCast(start, end, radius, option, result));
  EXPECT_NE(result.object, nullptr);
  EXPECT_EQ(*result.object, near);
}

TEST(CollisionWorldSphereCast, SphereCastAll)
{
  CollisionWorld world;

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  tf1.translation() = Eigen::Vector3d(0, 0, 5);
  world.createObject(std::make_unique<SphereShape>(1.0), tf1);

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0, 0, 8);
  world.createObject(std::make_unique<SphereShape>(1.0), tf2);

  Eigen::Isometry3d tf3 = Eigen::Isometry3d::Identity();
  tf3.translation() = Eigen::Vector3d(10, 0, 0);
  world.createObject(std::make_unique<SphereShape>(1.0), tf3);

  Eigen::Vector3d start(0, 0, 0);
  Eigen::Vector3d end(0, 0, 15);
  double radius = 0.5;

  CcdOption option;
  std::vector<CcdResult> results;

  EXPECT_TRUE(world.sphereCastAll(start, end, radius, option, results));
  EXPECT_EQ(results.size(), 2u);

  EXPECT_LT(results[0].timeOfImpact, results[1].timeOfImpact);
}

TEST(CollisionWorldSphereCast, MixedShapes)
{
  CollisionWorld world;

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  tf1.translation() = Eigen::Vector3d(0, 0, 3);
  world.createObject(std::make_unique<SphereShape>(1.0), tf1);

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0, 0, 6);
  world.createObject(std::make_unique<BoxShape>(Eigen::Vector3d(2, 2, 2)), tf2);

  Eigen::Isometry3d tf3 = Eigen::Isometry3d::Identity();
  tf3.translation() = Eigen::Vector3d(0, 0, 10);
  world.createObject(std::make_unique<CapsuleShape>(0.5, 2.0), tf3);

  Eigen::Vector3d start(0, 0, 0);
  Eigen::Vector3d end(0, 0, 15);
  double radius = 0.25;

  CcdOption option;
  std::vector<CcdResult> results;

  EXPECT_TRUE(world.sphereCastAll(start, end, radius, option, results));
  EXPECT_EQ(results.size(), 3u);

  for (std::size_t i = 1; i < results.size(); ++i) {
    EXPECT_LE(results[i - 1].timeOfImpact, results[i].timeOfImpact);
  }
}

//==============================================================================
// CollisionWorld CapsuleCast tests
//==============================================================================

TEST(CollisionWorldCapsuleCast, NoHit)
{
  CollisionWorld world;

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(10, 0, 0);
  world.createObject(std::make_unique<SphereShape>(1.0), tf);

  CapsuleShape capsule(0.5, 2.0);
  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(0, 0, 10);

  CcdOption option;
  CcdResult result;

  EXPECT_FALSE(
      world.capsuleCast(capsuleStart, capsuleEnd, capsule, option, result));
  EXPECT_FALSE(result.hit);
}

TEST(CollisionWorldCapsuleCast, SingleHit)
{
  CollisionWorld world;

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0, 0, 5);
  auto target = world.createObject(std::make_unique<SphereShape>(1.0), tf);

  CapsuleShape capsule(0.5, 2.0);
  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(0, 0, 10);

  CcdOption option;
  CcdResult result;

  EXPECT_TRUE(
      world.capsuleCast(capsuleStart, capsuleEnd, capsule, option, result));
  EXPECT_TRUE(result.hit);
  EXPECT_NE(result.object, nullptr);
  EXPECT_EQ(*result.object, target);
  EXPECT_GT(result.timeOfImpact, 0.0);
  EXPECT_LT(result.timeOfImpact, 1.0);
}

TEST(CollisionWorldCapsuleCast, ClosestHit)
{
  CollisionWorld world;

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  tf1.translation() = Eigen::Vector3d(0, 0, 4);
  auto near = world.createObject(std::make_unique<SphereShape>(1.0), tf1);

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0, 0, 8);
  world.createObject(std::make_unique<SphereShape>(1.0), tf2);

  CapsuleShape capsule(0.5, 2.0);
  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(0, 0, 15);

  CcdOption option;
  CcdResult result;

  EXPECT_TRUE(
      world.capsuleCast(capsuleStart, capsuleEnd, capsule, option, result));
  EXPECT_NE(result.object, nullptr);
  EXPECT_EQ(*result.object, near);
}

TEST(CollisionWorldCapsuleCast, CapsuleCastAll)
{
  CollisionWorld world;

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  tf1.translation() = Eigen::Vector3d(0, 0, 4);
  world.createObject(std::make_unique<SphereShape>(1.0), tf1);

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0, 0, 8);
  world.createObject(std::make_unique<BoxShape>(Eigen::Vector3d(2, 2, 2)), tf2);

  Eigen::Isometry3d tf3 = Eigen::Isometry3d::Identity();
  tf3.translation() = Eigen::Vector3d(10, 0, 0);
  world.createObject(std::make_unique<SphereShape>(1.0), tf3);

  CapsuleShape capsule(0.3, 1.5);
  Eigen::Isometry3d capsuleStart = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d capsuleEnd = Eigen::Isometry3d::Identity();
  capsuleEnd.translation() = Eigen::Vector3d(0, 0, 15);

  CcdOption option;
  std::vector<CcdResult> results;

  EXPECT_TRUE(
      world.capsuleCastAll(capsuleStart, capsuleEnd, capsule, option, results));
  EXPECT_EQ(results.size(), 2u);

  EXPECT_LT(results[0].timeOfImpact, results[1].timeOfImpact);
}
