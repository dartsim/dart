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

#include <algorithm>

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
// CollisionWorld CollideAll tests
//==============================================================================

TEST(CollisionWorld, CollideAllOrderingAndRepeatability)
{
  CollisionWorld world;
  const double radius = 1.0;

  Eigen::Isometry3d tf0 = Eigen::Isometry3d::Identity();
  tf0.translation() = Eigen::Vector3d(3.0, 0, 0);
  auto obj0 = world.createObject(std::make_unique<SphereShape>(radius), tf0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  tf1.translation() = Eigen::Vector3d(0.0, 0, 0);
  auto obj1 = world.createObject(std::make_unique<SphereShape>(radius), tf1);

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);
  auto obj2 = world.createObject(std::make_unique<SphereShape>(radius), tf2);

  const std::size_t id0 = obj0.getId();
  const std::size_t id1 = obj1.getId();
  const std::size_t id2 = obj2.getId();

  std::vector<BroadPhasePair> expectedPairs;
  expectedPairs.emplace_back(std::min(id0, id2), std::max(id0, id2));
  expectedPairs.emplace_back(std::min(id1, id2), std::max(id1, id2));
  std::sort(expectedPairs.begin(), expectedPairs.end());

  CollisionOption option;

  auto snapshot = world.buildBroadPhaseSnapshot();
  ASSERT_EQ(snapshot.pairs.size(), expectedPairs.size());
  EXPECT_EQ(snapshot.pairs, expectedPairs);

  CollisionResult baseline;
  ASSERT_TRUE(world.collideAll(snapshot, option, baseline));
  ASSERT_EQ(baseline.numContacts(), snapshot.pairs.size());
  ASSERT_EQ(baseline.numManifolds(), snapshot.pairs.size());

  auto getCenter = [&](std::size_t id) -> Eigen::Vector3d {
    if (id == id0) {
      return tf0.translation();
    }
    if (id == id1) {
      return tf1.translation();
    }
    return tf2.translation();
  };

  struct ExpectedContact
  {
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    double depth = 0.0;
  };

  std::vector<ExpectedContact> expectedContacts;
  expectedContacts.reserve(snapshot.pairs.size());
  for (const auto& pair : snapshot.pairs) {
    const Eigen::Vector3d c1 = getCenter(pair.first);
    const Eigen::Vector3d c2 = getCenter(pair.second);
    const Eigen::Vector3d diff = c2 - c1;
    const double dist = diff.norm();
    const double penetration = 2.0 * radius - dist;

    ExpectedContact expected;
    if (dist < 1e-10) {
      expected.normal = Eigen::Vector3d::UnitZ();
      expected.point = c1;
    } else {
      expected.normal = -diff / dist;
      expected.point = c1 + expected.normal * (-radius + penetration * 0.5);
    }
    expected.depth = penetration;
    expectedContacts.push_back(expected);
  }

  for (std::size_t i = 0; i < expectedContacts.size(); ++i) {
    const auto& contact = baseline.getContact(i);
    const auto& expected = expectedContacts[i];

    EXPECT_NEAR(contact.position.x(), expected.point.x(), 1e-10);
    EXPECT_NEAR(contact.position.y(), expected.point.y(), 1e-10);
    EXPECT_NEAR(contact.position.z(), expected.point.z(), 1e-10);
    EXPECT_NEAR(contact.normal.x(), expected.normal.x(), 1e-10);
    EXPECT_NEAR(contact.normal.y(), expected.normal.y(), 1e-10);
    EXPECT_NEAR(contact.normal.z(), expected.normal.z(), 1e-10);
    EXPECT_NEAR(contact.depth, expected.depth, 1e-10);
  }

  for (int iteration = 0; iteration < 5; ++iteration) {
    auto repeatSnapshot = world.buildBroadPhaseSnapshot();
    EXPECT_EQ(repeatSnapshot.pairs, snapshot.pairs);

    CollisionResult repeat;
    ASSERT_TRUE(world.collideAll(repeatSnapshot, option, repeat));
    ASSERT_EQ(repeat.numContacts(), baseline.numContacts());
    ASSERT_EQ(repeat.numManifolds(), baseline.numManifolds());

    for (std::size_t i = 0; i < repeat.numContacts(); ++i) {
      const auto& baseContact = baseline.getContact(i);
      const auto& repeatContact = repeat.getContact(i);
      EXPECT_NEAR(repeatContact.position.x(), baseContact.position.x(), 1e-10);
      EXPECT_NEAR(repeatContact.position.y(), baseContact.position.y(), 1e-10);
      EXPECT_NEAR(repeatContact.position.z(), baseContact.position.z(), 1e-10);
      EXPECT_NEAR(repeatContact.normal.x(), baseContact.normal.x(), 1e-10);
      EXPECT_NEAR(repeatContact.normal.y(), baseContact.normal.y(), 1e-10);
      EXPECT_NEAR(repeatContact.normal.z(), baseContact.normal.z(), 1e-10);
      EXPECT_NEAR(repeatContact.depth, baseContact.depth, 1e-10);
    }
  }
}

TEST(CollisionWorld, ObjectIdRoundTrip)
{
  CollisionWorld world;

  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0));

  const std::size_t id1 = obj1.getId();
  const std::size_t id2 = obj2.getId();

  ASSERT_NE(id1, id2);

  auto lookup1 = world.getObjectById(id1);
  auto lookup2 = world.getObjectById(id2);

  EXPECT_TRUE(lookup1.isValid());
  EXPECT_TRUE(lookup2.isValid());
  EXPECT_EQ(lookup1.getEntity(), obj1.getEntity());
  EXPECT_EQ(lookup2.getEntity(), obj2.getEntity());

  world.destroyObject(obj1);
  auto missing = world.getObjectById(id1);
  EXPECT_FALSE(missing.isValid());

  auto obj3 = world.createObject(std::make_unique<SphereShape>(1.0));
  const std::size_t id3 = obj3.getId();
  EXPECT_NE(id3, id1);
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

//==============================================================================
// Reusable Buffer API tests
//==============================================================================

TEST(CollisionWorldBroadPhase, BuildSnapshotReusableBuffer)
{
  CollisionWorld world;

  world.createObject(
      std::make_unique<SphereShape>(1.0), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);
  world.createObject(std::make_unique<SphereShape>(1.0), tf2);

  Eigen::Isometry3d tf3 = Eigen::Isometry3d::Identity();
  tf3.translation() = Eigen::Vector3d(10, 0, 0);
  world.createObject(std::make_unique<SphereShape>(1.0), tf3);

  BroadPhaseSnapshot snapshot;
  world.buildBroadPhaseSnapshot(snapshot);

  EXPECT_EQ(snapshot.numObjects, 3u);
  EXPECT_EQ(snapshot.pairs.size(), 1u);

  std::size_t oldCapacity = snapshot.pairs.capacity();

  world.buildBroadPhaseSnapshot(snapshot);

  EXPECT_EQ(snapshot.pairs.capacity(), oldCapacity);
  EXPECT_EQ(snapshot.pairs.size(), 1u);
}

TEST(CollisionWorldBroadPhase, BuildSnapshotWithSettingsReusableBuffer)
{
  CollisionWorld world;

  world.createObject(
      std::make_unique<SphereShape>(1.0), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);
  world.createObject(std::make_unique<SphereShape>(1.0), tf2);

  BatchSettings settings;
  settings.deterministic = true;

  BroadPhaseSnapshot snapshot;
  world.buildBroadPhaseSnapshot(snapshot, settings);

  EXPECT_EQ(snapshot.numObjects, 2u);
  EXPECT_EQ(snapshot.pairs.size(), 1u);
  EXPECT_EQ(snapshot.pairs[0].first, 0u);
  EXPECT_LT(snapshot.pairs[0].first, snapshot.pairs[0].second);
}
