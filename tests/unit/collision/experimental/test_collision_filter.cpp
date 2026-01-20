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

#include <dart/collision/experimental/collision_filter.hpp>
#include <dart/collision/experimental/collision_world.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <gtest/gtest.h>

using namespace dart::collision::experimental;

//==============================================================================
// CollisionFilterData bitmask tests
//==============================================================================

TEST(CollisionFilterData, DefaultCanCollideWithEverything)
{
  CollisionFilterData data1;
  CollisionFilterData data2;

  EXPECT_TRUE(data1.canCollideWith(data2));
  EXPECT_TRUE(data2.canCollideWith(data1));
}

TEST(CollisionFilterData, SameGroupsCollide)
{
  CollisionFilterData data1{FilterGroup::Dynamic, kCollisionMaskAll};
  CollisionFilterData data2{FilterGroup::Dynamic, kCollisionMaskAll};

  EXPECT_TRUE(data1.canCollideWith(data2));
}

TEST(CollisionFilterData, DifferentGroupsWithMatchingMasksCollide)
{
  CollisionFilterData data1{FilterGroup::Static, kCollisionMaskAll};
  CollisionFilterData data2{FilterGroup::Dynamic, kCollisionMaskAll};

  EXPECT_TRUE(data1.canCollideWith(data2));
  EXPECT_TRUE(data2.canCollideWith(data1));
}

TEST(CollisionFilterData, MaskExcludesGroup)
{
  CollisionFilterData staticObj{FilterGroup::Static, FilterGroup::Static};
  CollisionFilterData dynamicObj{FilterGroup::Dynamic, FilterGroup::Dynamic};

  EXPECT_FALSE(staticObj.canCollideWith(dynamicObj));
  EXPECT_FALSE(dynamicObj.canCollideWith(staticObj));
}

TEST(CollisionFilterData, AsymmetricMaskFiltering)
{
  CollisionFilterData sensor{FilterGroup::Sensor, kCollisionMaskAll};
  CollisionFilterData debris{
      FilterGroup::Debris, kCollisionMaskAll & ~FilterGroup::Sensor};

  // canCollideWith requires BOTH directions to pass
  // debris excludes Sensor from its mask, so neither direction works
  EXPECT_FALSE(sensor.canCollideWith(debris));
  EXPECT_FALSE(debris.canCollideWith(sensor));
}

TEST(CollisionFilterData, NoneCollidesWithNothing)
{
  CollisionFilterData none = CollisionFilterData::none();
  CollisionFilterData all = CollisionFilterData::all();

  EXPECT_FALSE(none.canCollideWith(all));
  EXPECT_FALSE(all.canCollideWith(none));
  EXPECT_FALSE(none.canCollideWith(none));
}

TEST(CollisionFilterData, FactoryMethods)
{
  auto all = CollisionFilterData::all();
  EXPECT_EQ(all.collisionGroup, kCollisionGroupAll);
  EXPECT_EQ(all.collisionMask, kCollisionMaskAll);

  auto none = CollisionFilterData::none();
  EXPECT_EQ(none.collisionGroup, kCollisionGroupNone);
  EXPECT_EQ(none.collisionMask, kCollisionMaskNone);

  auto group = CollisionFilterData::group(FilterGroup::Character);
  EXPECT_EQ(group.collisionGroup, FilterGroup::Character);
  EXPECT_EQ(group.collisionMask, kCollisionMaskAll);

  auto both = CollisionFilterData::groupAndMask(
      FilterGroup::Static, FilterGroup::Dynamic | FilterGroup::Kinematic);
  EXPECT_EQ(both.collisionGroup, FilterGroup::Static);
  EXPECT_EQ(both.collisionMask, FilterGroup::Dynamic | FilterGroup::Kinematic);
}

TEST(CollisionFilterData, MultipleGroupsInMask)
{
  CollisionFilterData player{
      FilterGroup::Character,
      FilterGroup::Static | FilterGroup::Dynamic | FilterGroup::Trigger};
  CollisionFilterData wall{FilterGroup::Static, kCollisionMaskAll};
  CollisionFilterData bullet{FilterGroup::Dynamic, kCollisionMaskAll};
  CollisionFilterData sensor{FilterGroup::Sensor, kCollisionMaskAll};

  EXPECT_TRUE(player.canCollideWith(wall));
  EXPECT_TRUE(player.canCollideWith(bullet));
  EXPECT_FALSE(player.canCollideWith(sensor));
}

//==============================================================================
// CallbackCollisionFilter tests
//==============================================================================

TEST(CallbackCollisionFilter, IgnoresWhenCallbackReturnsTrue)
{
  CollisionWorld world;

  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0));

  bool filterCalled = false;
  CallbackCollisionFilter filter(
      [&](const CollisionObject&, const CollisionObject&) {
        filterCalled = true;
        return true;
      });

  EXPECT_TRUE(filter.ignoresCollision(obj1, obj2));
  EXPECT_TRUE(filterCalled);
}

TEST(CallbackCollisionFilter, AllowsWhenCallbackReturnsFalse)
{
  CollisionWorld world;

  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0));

  CallbackCollisionFilter filter(
      [](const CollisionObject&, const CollisionObject&) { return false; });

  EXPECT_FALSE(filter.ignoresCollision(obj1, obj2));
}

TEST(CallbackCollisionFilter, NullCallbackAllowsCollision)
{
  CollisionWorld world;

  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0));

  CallbackCollisionFilter filter(nullptr);

  EXPECT_FALSE(filter.ignoresCollision(obj1, obj2));
}

//==============================================================================
// CompositeCollisionFilter tests
//==============================================================================

TEST(CompositeCollisionFilter, EmptyAllowsAll)
{
  CollisionWorld world;

  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0));

  CompositeCollisionFilter composite;
  EXPECT_EQ(composite.numFilters(), 0u);
  EXPECT_FALSE(composite.ignoresCollision(obj1, obj2));
}

TEST(CompositeCollisionFilter, SingleFilterIgnores)
{
  CollisionWorld world;

  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0));

  CallbackCollisionFilter blockAll(
      [](const CollisionObject&, const CollisionObject&) { return true; });

  CompositeCollisionFilter composite;
  composite.addFilter(&blockAll);
  EXPECT_EQ(composite.numFilters(), 1u);
  EXPECT_TRUE(composite.ignoresCollision(obj1, obj2));
}

TEST(CompositeCollisionFilter, AnyFilterIgnores)
{
  CollisionWorld world;

  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0));

  CallbackCollisionFilter allowAll(
      [](const CollisionObject&, const CollisionObject&) { return false; });
  CallbackCollisionFilter blockAll(
      [](const CollisionObject&, const CollisionObject&) { return true; });

  CompositeCollisionFilter composite;
  composite.addFilter(&allowAll);
  composite.addFilter(&blockAll);

  EXPECT_EQ(composite.numFilters(), 2u);
  EXPECT_TRUE(composite.ignoresCollision(obj1, obj2));
}

TEST(CompositeCollisionFilter, RemoveFilter)
{
  CollisionWorld world;

  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0));

  CallbackCollisionFilter blockAll(
      [](const CollisionObject&, const CollisionObject&) { return true; });

  CompositeCollisionFilter composite;
  composite.addFilter(&blockAll);
  EXPECT_TRUE(composite.ignoresCollision(obj1, obj2));

  composite.removeFilter(&blockAll);
  EXPECT_EQ(composite.numFilters(), 0u);
  EXPECT_FALSE(composite.ignoresCollision(obj1, obj2));
}

TEST(CompositeCollisionFilter, ClearFilters)
{
  CollisionWorld world;

  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0));

  CallbackCollisionFilter filter1(
      [](const CollisionObject&, const CollisionObject&) { return true; });
  CallbackCollisionFilter filter2(
      [](const CollisionObject&, const CollisionObject&) { return true; });

  CompositeCollisionFilter composite;
  composite.addFilter(&filter1);
  composite.addFilter(&filter2);
  EXPECT_EQ(composite.numFilters(), 2u);

  composite.clearFilters();
  EXPECT_EQ(composite.numFilters(), 0u);
  EXPECT_FALSE(composite.ignoresCollision(obj1, obj2));
}

//==============================================================================
// CollisionObject filter API tests
//==============================================================================

TEST(CollisionObjectFilter, DefaultFilterData)
{
  CollisionWorld world;

  auto obj = world.createObject(std::make_unique<SphereShape>(1.0));

  EXPECT_EQ(obj.getCollisionGroup(), kCollisionGroupAll);
  EXPECT_EQ(obj.getCollisionMask(), kCollisionMaskAll);
}

TEST(CollisionObjectFilter, SetCollisionGroup)
{
  CollisionWorld world;

  auto obj = world.createObject(std::make_unique<SphereShape>(1.0));
  obj.setCollisionGroup(FilterGroup::Dynamic);

  EXPECT_EQ(obj.getCollisionGroup(), FilterGroup::Dynamic);
  EXPECT_EQ(obj.getCollisionMask(), kCollisionMaskAll);
}

TEST(CollisionObjectFilter, SetCollisionMask)
{
  CollisionWorld world;

  auto obj = world.createObject(std::make_unique<SphereShape>(1.0));
  obj.setCollisionMask(FilterGroup::Static | FilterGroup::Dynamic);

  EXPECT_EQ(obj.getCollisionGroup(), kCollisionGroupAll);
  EXPECT_EQ(obj.getCollisionMask(), FilterGroup::Static | FilterGroup::Dynamic);
}

TEST(CollisionObjectFilter, SetCollisionFilter)
{
  CollisionWorld world;

  auto obj = world.createObject(std::make_unique<SphereShape>(1.0));
  obj.setCollisionFilter(FilterGroup::Character, FilterGroup::Static);

  EXPECT_EQ(obj.getCollisionGroup(), FilterGroup::Character);
  EXPECT_EQ(obj.getCollisionMask(), FilterGroup::Static);
}

TEST(CollisionObjectFilter, SetCollisionFilterData)
{
  CollisionWorld world;

  auto obj = world.createObject(std::make_unique<SphereShape>(1.0));

  CollisionFilterData data{FilterGroup::Sensor, FilterGroup::Character};
  obj.setCollisionFilterData(data);

  const auto& retrieved = obj.getCollisionFilterData();
  EXPECT_EQ(retrieved.collisionGroup, FilterGroup::Sensor);
  EXPECT_EQ(retrieved.collisionMask, FilterGroup::Character);
}

//==============================================================================
// CollisionWorld filtering integration tests
//==============================================================================

TEST(CollisionWorldFilter, BitmaskFilteringPreventsCollision)
{
  CollisionWorld world;

  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));
  obj1.setCollisionFilter(FilterGroup::Static, FilterGroup::Static);

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0), tf2);
  obj2.setCollisionFilter(FilterGroup::Dynamic, FilterGroup::Dynamic);

  CollisionOption option;
  CollisionResult result;

  bool hasCollision = world.collide(option, result);

  EXPECT_FALSE(hasCollision);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(CollisionWorldFilter, BitmaskFilteringAllowsCollision)
{
  CollisionWorld world;

  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));
  obj1.setCollisionFilter(FilterGroup::Static, kCollisionMaskAll);

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0), tf2);
  obj2.setCollisionFilter(FilterGroup::Dynamic, kCollisionMaskAll);

  CollisionOption option;
  CollisionResult result;

  bool hasCollision = world.collide(option, result);

  EXPECT_TRUE(hasCollision);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(CollisionWorldFilter, CallbackFilterPreventsCollision)
{
  CollisionWorld world;

  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0), tf2);

  CallbackCollisionFilter blockAll(
      [](const CollisionObject&, const CollisionObject&) { return true; });

  CollisionOption option;
  option.collisionFilter = &blockAll;
  CollisionResult result;

  bool hasCollision = world.collide(option, result);

  EXPECT_FALSE(hasCollision);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(CollisionWorldFilter, CallbackFilterAllowsCollision)
{
  CollisionWorld world;

  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0), tf2);

  CallbackCollisionFilter allowAll(
      [](const CollisionObject&, const CollisionObject&) { return false; });

  CollisionOption option;
  option.collisionFilter = &allowAll;
  CollisionResult result;

  bool hasCollision = world.collide(option, result);

  EXPECT_TRUE(hasCollision);
  EXPECT_GE(result.numContacts(), 1u);
}

TEST(CollisionWorldFilter, DirectCollideRespectsFilter)
{
  CollisionWorld world;

  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));
  obj1.setCollisionFilter(FilterGroup::Static, FilterGroup::Static);

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0), tf2);
  obj2.setCollisionFilter(FilterGroup::Dynamic, FilterGroup::Dynamic);

  CollisionOption option;
  CollisionResult result;

  bool hasCollision = world.collide(obj1, obj2, option, result);

  EXPECT_FALSE(hasCollision);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(CollisionWorldFilter, CollideAllRespectsFilter)
{
  CollisionWorld world;

  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));
  obj1.setCollisionFilter(FilterGroup::Static, FilterGroup::Static);

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0), tf2);
  obj2.setCollisionFilter(FilterGroup::Dynamic, FilterGroup::Dynamic);

  Eigen::Isometry3d tf3 = Eigen::Isometry3d::Identity();
  tf3.translation() = Eigen::Vector3d(3.0, 0, 0);
  auto obj3 = world.createObject(std::make_unique<SphereShape>(1.0), tf3);
  obj3.setCollisionFilter(FilterGroup::Dynamic, FilterGroup::Dynamic);

  auto snapshot = world.buildBroadPhaseSnapshot();

  CollisionOption option;
  CollisionResult result;

  bool hasCollision = world.collideAll(snapshot, option, result);

  EXPECT_TRUE(hasCollision);
  EXPECT_EQ(result.numContacts(), 1u);
}

TEST(CollisionWorldFilter, MixedBitmaskAndCallback)
{
  CollisionWorld world;

  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));
  obj1.setCollisionFilter(FilterGroup::Dynamic, kCollisionMaskAll);

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0), tf2);
  obj2.setCollisionFilter(FilterGroup::Dynamic, kCollisionMaskAll);

  CallbackCollisionFilter blockAll(
      [](const CollisionObject&, const CollisionObject&) { return true; });

  CollisionOption option;
  option.collisionFilter = &blockAll;
  CollisionResult result;

  bool hasCollision = world.collide(option, result);

  EXPECT_FALSE(hasCollision);
  EXPECT_EQ(result.numContacts(), 0u);
}

TEST(CollisionWorldFilter, ThreeObjectsSelectiveFiltering)
{
  CollisionWorld world;

  // Three overlapping spheres in a line: static(0) - dynamic(1.5) - sensor(3.0)
  // All spheres have radius 1.0, so adjacent pairs overlap
  auto staticObj = world.createObject(std::make_unique<SphereShape>(1.0));
  staticObj.setCollisionFilter(FilterGroup::Static, kCollisionMaskAll);

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);
  auto dynamicObj = world.createObject(std::make_unique<SphereShape>(1.0), tf2);
  // Dynamic excludes Sensor from its mask
  dynamicObj.setCollisionFilter(
      FilterGroup::Dynamic, kCollisionMaskAll & ~FilterGroup::Sensor);

  Eigen::Isometry3d tf3 = Eigen::Isometry3d::Identity();
  tf3.translation() = Eigen::Vector3d(3.0, 0, 0);
  auto sensorObj = world.createObject(std::make_unique<SphereShape>(1.0), tf3);
  sensorObj.setCollisionFilter(FilterGroup::Sensor, kCollisionMaskAll);

  auto snapshot = world.buildBroadPhaseSnapshot();

  CollisionOption option;
  CollisionResult result;
  world.collideAll(snapshot, option, result);

  // Expected collisions:
  // - static <-> dynamic: YES (both allow each other)
  // - dynamic <-> sensor: NO (dynamic excludes sensor)
  // - static <-> sensor: NO (not overlapping, distance = 3.0 > 2.0)
  EXPECT_EQ(result.numContacts(), 1u);
}

//==============================================================================
// Helper function tests
//==============================================================================

TEST(ShouldCollide, BitmaskOnly)
{
  CollisionFilterData data1{FilterGroup::Static, kCollisionMaskAll};
  CollisionFilterData data2{FilterGroup::Dynamic, kCollisionMaskAll};

  EXPECT_TRUE(shouldCollide(data1, data2));

  CollisionFilterData data3{FilterGroup::Static, FilterGroup::Static};
  EXPECT_FALSE(shouldCollide(data2, data3));
}

TEST(ShouldCollide, WithCallback)
{
  CollisionWorld world;

  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));
  auto obj2 = world.createObject(std::make_unique<SphereShape>(1.0));

  CollisionFilterData data1;
  CollisionFilterData data2;

  CallbackCollisionFilter blockAll(
      [](const CollisionObject&, const CollisionObject&) { return true; });

  EXPECT_FALSE(shouldCollide(data1, data2, obj1, obj2, &blockAll));
  EXPECT_TRUE(shouldCollide(data1, data2, obj1, obj2, nullptr));
}
