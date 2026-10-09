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

#include "../../integration/AllocationCounting.hpp"
#include "dart/collision/CollisionGroup.hpp"
#include "dart/collision/dart/DARTCollisionDetector.hpp"
#include "dart/constraint/BoxedLcpConstraintSolver.hpp"
#include "dart/constraint/detail/ContactWarmStartCache.hpp"
#include "dart/dynamics/SimpleFrame.hpp"

#include <gtest/gtest.h>

#include <array>
#include <limits>

using dart::constraint::detail::ContactWarmStartCache;

namespace {

class ContactWarmStartCacheTest : public testing::Test
{
protected:
  ContactWarmStartCache::Key key(double point = 0.0)
  {
    ContactWarmStartCache::Key result;
    result.frames = {{&frames[0], &frames[1]}};
    result.points
        = {{Eigen::Vector3d(point, 0, 0), Eigen::Vector3d(0, point, 0)}};
    result.normals = {{Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX()}};
    return result;
  }

  void publish(const ContactWarmStartCache::Key& value, double impulse = 1.0)
  {
    cache.begin(0.001, 8);
    cache.add(&contacts[0], value);
    cache.update(
        &contacts[0],
        {{Eigen::Vector3d(impulse, 2, 3), Eigen::Vector3d(4, impulse, 6)}});
    cache.finish();
  }

  std::array<dart::dynamics::SimpleFrame, 3> frames;
  std::array<int, 8> contacts{};
  ContactWarmStartCache cache;
};

} // namespace

TEST_F(
    ContactWarmStartCacheTest, RequiresBothLocalPointsAndNormalsWithinTolerance)
{
  const auto original = key();
  for (int side = 0; side < 2; ++side) {
    for (int change = 0; change < 4; ++change) {
      SCOPED_TRACE(side);
      SCOPED_TRACE(change);
      publish(original);
      cache.begin(0.001, 1);
      auto candidate = original;
      if (change < 2)
        candidate.points[side][0] += change == 0 ? 0.0009 : 0.0011;
      else
        candidate.normals[side]
            = original.normals[side]
              + Eigen::Vector3d::UnitY() * (change == 2 ? 0.04 : 0.05);
      ASSERT_NE(
          ContactWarmStartCache::npos, cache.add(&contacts[1], candidate));
      ASSERT_NE(nullptr, cache.seed(&contacts[1]));
      EXPECT_EQ(change % 2 == 0, cache.seed(&contacts[1])->matched);
    }
  }
  publish(original);
  cache.begin(0.001, 1);
  auto scaledNormals = original;
  scaledNormals.normals[0] *= 2;
  scaledNormals.normals[1] *= 0.5;
  cache.add(&contacts[1], scaledNormals);
  EXPECT_TRUE(cache.seed(&contacts[1])->matched);
}

TEST_F(ContactWarmStartCacheTest, NearestOneToOneMatchHasStableTieBreak)
{
  cache.begin(0.001, 3);
  cache.add(&contacts[0], key(-0.0004));
  cache.add(&contacts[1], key(0.0004));
  cache.add(&contacts[2], key(0.0004));
  for (int i = 0; i < 3; ++i)
    cache.update(
        &contacts[i],
        {{Eigen::Vector3d(i + 1, 0, 0), Eigen::Vector3d(i + 1, 0, 0)}});
  cache.finish();

  cache.begin(0.001, 5);
  cache.add(&contacts[3], key(0.0003));
  cache.add(&contacts[4], key());
  cache.add(&contacts[5], key(0.0004));
  cache.add(&contacts[6], key());
  EXPECT_EQ(2.0, cache.seed(&contacts[3])->localImpulse[0]);
  EXPECT_EQ(1.0, cache.seed(&contacts[4])->localImpulse[0]);
  EXPECT_EQ(3.0, cache.seed(&contacts[5])->localImpulse[0]);
  EXPECT_FALSE(cache.seed(&contacts[6])->matched);
  auto otherPair = key();
  otherPair.frames[1] = &frames[2];
  cache.add(&contacts[7], otherPair);
  EXPECT_FALSE(cache.seed(&contacts[7])->matched);
}

TEST_F(
    ContactWarmStartCacheTest, ReversedPairUsesOppositeImpulseInOtherBodyFrame)
{
  const auto original = key(0.03);
  publish(original);
  auto reversed = original;
  for (int side = 0; side < 2; ++side) {
    reversed.frames[side] = original.frames[1 - side];
    reversed.points[side] = original.points[1 - side];
    reversed.normals[side] = -original.normals[1 - side];
  }
  cache.begin(0.001, 1);
  cache.add(&contacts[1], reversed);
  ASSERT_TRUE(cache.seed(&contacts[1])->matched);
  EXPECT_TRUE(cache.seed(&contacts[1])
                  ->localImpulse.isApprox(Eigen::Vector3d(-4, -1, -6), 0.0));
}

TEST_F(
    ContactWarmStartCacheTest, MissingContactsAndTimestepChangesExpireHistory)
{
  publish(key());
  EXPECT_EQ(1u, cache.size());
  cache.begin(0.002, 1);
  cache.add(&contacts[1], key());
  EXPECT_FALSE(cache.seed(&contacts[1])->matched);
  cache.update(
      &contacts[1], {{Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones()}});
  cache.finish();
  cache.begin(0.002, 0);
  cache.finish();
  EXPECT_EQ(0u, cache.size());
  cache.begin(0.002, 1);
  cache.add(&contacts[1], key());
  EXPECT_FALSE(cache.seed(&contacts[1])->matched);
  EXPECT_EQ(nullptr, cache.seed(&contacts[0]));
  cache.finish();
  EXPECT_EQ(0u, cache.size());
  cache.clear();
  EXPECT_EQ(0u, cache.size());
}

TEST_F(
    ContactWarmStartCacheTest,
    CollisionGroupIdentityAndContentInvalidateHistory)
{
  auto detector = dart::collision::DARTCollisionDetector::create();
  auto first = detector->createCollisionGroup();
  auto second = detector->createCollisionGroup();
  const auto value = key();
  for (int change = 0; change < 2; ++change) {
    cache.begin(0.001, 1, first.get(), 10);
    cache.add(&contacts[0], value);
    cache.update(
        &contacts[0], {{Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones()}});
    cache.finish();
    cache.begin(
        0.001,
        1,
        change == 0 ? second.get() : first.get(),
        change == 0 ? 10 : 11);
    cache.add(&contacts[1], value);
    EXPECT_FALSE(cache.seed(&contacts[1])->matched);
    cache.finish();
  }
}

TEST_F(ContactWarmStartCacheTest, InvalidGeometryAndImpulsesAreNeverRetained)
{
  cache.begin(0.001, 8);
  auto value = key();
  EXPECT_EQ(ContactWarmStartCache::npos, cache.add(nullptr, value));
  value.frames[0] = nullptr;
  EXPECT_EQ(ContactWarmStartCache::npos, cache.add(&contacts[0], value));
  value = key();
  value.frames[1] = value.frames[0];
  EXPECT_EQ(ContactWarmStartCache::npos, cache.add(&contacts[0], value));
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (int side = 0; side < 2; ++side) {
    value = key();
    value.points[side][0] = nan;
    EXPECT_EQ(ContactWarmStartCache::npos, cache.add(&contacts[0], value));
    value = key();
    value.normals[side].setZero();
    EXPECT_EQ(ContactWarmStartCache::npos, cache.add(&contacts[0], value));
    value.normals[side][0] = nan;
    EXPECT_EQ(ContactWarmStartCache::npos, cache.add(&contacts[0], value));
  }
  value = key();
  EXPECT_NE(ContactWarmStartCache::npos, cache.add(&contacts[0], value));
  EXPECT_EQ(ContactWarmStartCache::npos, cache.add(&contacts[0], value));
  cache.update(
      &contacts[0], {{Eigen::Vector3d::Ones(), Eigen::Vector3d(nan, 0, 0)}});
  cache.finish();
  EXPECT_EQ(0u, cache.size());
  for (double dt : {0.0, -0.001, nan}) {
    cache.begin(dt, 1);
    EXPECT_EQ(ContactWarmStartCache::npos, cache.add(&contacts[0], value));
    cache.finish();
    EXPECT_EQ(0u, cache.size());
  }
}

TEST_F(ContactWarmStartCacheTest, ReservedSnapshotAndMergeDoNotAllocate)
{
  constexpr std::size_t count = 64;
  std::array<int, count> identifiers{};
  std::array<ContactWarmStartCache::Key, count> keys;
  for (std::size_t i = 0; i < count; ++i)
    keys[i] = key(0.01 * i);
  cache.reserve(count);
  const std::array<Eigen::Vector3d, 2> impulses{
      {Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones()}};
  dart::test::ScopedHeapAllocationCounter heap;
  dart::test::ScopedRawHeapAllocationCounter raw;
  bool allMatched = true;
  for (int step = 0; step < 20; ++step) {
    cache.begin(0.001, count);
    for (std::size_t i = 0; i < count; ++i) {
      cache.add(&identifiers[i], keys[i]);
      if (step > 0)
        allMatched &= cache.seed(&identifiers[i])->matched;
      cache.update(&identifiers[i], impulses);
    }
    cache.finish();
  }
  raw.stop();
  heap.stop();
  EXPECT_TRUE(allMatched);
  EXPECT_EQ(count, cache.size());
  EXPECT_EQ(0u, heap.allocationCount());
  if (!raw.skipped()) {
    EXPECT_EQ(0u, raw.allocationCount());
  }
}

TEST(ContactWarmStartCache, SideStorageIsIsolatedAndErased)
{
  using namespace dart::constraint::detail;
  dart::constraint::BoxedLcpConstraintSolver first;
  dart::constraint::BoxedLcpConstraintSolver second;
  EXPECT_EQ(nullptr, findContactWarmStartCache(&first));
  auto& firstCache = getOrCreateContactWarmStartCache(&first);
  auto& secondCache = getOrCreateContactWarmStartCache(&second);
  EXPECT_EQ(&firstCache, findContactWarmStartCache(&first));
  EXPECT_NE(&firstCache, &secondCache);
  eraseContactWarmStartCache(&first);
  EXPECT_EQ(nullptr, findContactWarmStartCache(&first));
  EXPECT_EQ(&secondCache, findContactWarmStartCache(&second));
  eraseContactWarmStartCache(&second);
}
