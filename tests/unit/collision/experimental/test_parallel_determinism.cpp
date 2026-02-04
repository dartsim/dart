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

#include <memory>
#include <random>
#include <vector>

using namespace dart::collision::experimental;

namespace {

void PopulateRandomWorld(
    CollisionWorld& world, std::size_t count, unsigned seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);

  for (std::size_t i = 0; i < count; ++i) {
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = Eigen::Vector3d(dist(rng), dist(rng), dist(rng));
    world.createObject(std::make_unique<SphereShape>(0.5), tf);
  }
}

void ExpectSameContacts(
    const CollisionResult& baseline,
    const CollisionResult& candidate,
    double eps)
{
  ASSERT_EQ(candidate.numManifolds(), baseline.numManifolds());
  ASSERT_EQ(candidate.numContacts(), baseline.numContacts());
  for (std::size_t i = 0; i < baseline.numContacts(); ++i) {
    const auto& baseContact = baseline.getContact(i);
    const auto& candContact = candidate.getContact(i);
    EXPECT_NEAR(candContact.position.x(), baseContact.position.x(), eps);
    EXPECT_NEAR(candContact.position.y(), baseContact.position.y(), eps);
    EXPECT_NEAR(candContact.position.z(), baseContact.position.z(), eps);
    EXPECT_NEAR(candContact.normal.x(), baseContact.normal.x(), eps);
    EXPECT_NEAR(candContact.normal.y(), baseContact.normal.y(), eps);
    EXPECT_NEAR(candContact.normal.z(), baseContact.normal.z(), eps);
    EXPECT_NEAR(candContact.depth, baseContact.depth, eps);
  }
}

CollisionResult RunCollideAll(
    CollisionWorld& world,
    const BroadPhaseSnapshot& snapshot,
    const CollisionOption& option,
    int maxThreads,
    std::size_t grainSize)
{
  BatchSettings settings;
  settings.deterministic = true;
  settings.maxThreads = maxThreads;
  settings.grainSize = grainSize;

  CollisionResult result;
  world.collideAll(snapshot, option, result, settings);
  return result;
}

} // namespace

TEST(ParallelNarrowPhase, DeterministicResultsAcrossThreads)
{
  CollisionWorld world;
  PopulateRandomWorld(world, 240, 42u);

  BatchSettings snapshotSettings;
  snapshotSettings.deterministic = true;
  auto snapshot = world.buildBroadPhaseSnapshot(snapshotSettings);

  CollisionOption option = CollisionOption::fullContacts(200000);
  const std::vector<std::size_t> grainSizes{1, 64, 256};
  const std::vector<int> threadCounts{2, 4, 8};

  for (std::size_t grain : grainSizes) {
    auto baseline = RunCollideAll(world, snapshot, option, 1, grain);
    for (int threads : threadCounts) {
      auto parallel = RunCollideAll(world, snapshot, option, threads, grain);
      ExpectSameContacts(baseline, parallel, 1e-9);
    }
  }
}

TEST(ParallelNarrowPhase, FallsBackWhenPairsBelowGrain)
{
  CollisionWorld world;
  auto obj1 = world.createObject(std::make_unique<SphereShape>(1.0));
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(1.5, 0, 0);
  world.createObject(std::make_unique<SphereShape>(1.0), tf);

  BatchSettings snapshotSettings;
  snapshotSettings.deterministic = true;
  auto snapshot = world.buildBroadPhaseSnapshot(snapshotSettings);

  CollisionOption option = CollisionOption::fullContacts(1000);
  const std::size_t grainSize = snapshot.pairs.size() + 10;

  auto baseline = RunCollideAll(world, snapshot, option, 1, grainSize);
  auto parallel = RunCollideAll(world, snapshot, option, 4, grainSize);
  ExpectSameContacts(baseline, parallel, 1e-9);
}

TEST(ParallelNarrowPhase, EmptyPairsRemainEmpty)
{
  CollisionWorld world;
  BatchSettings snapshotSettings;
  snapshotSettings.deterministic = true;
  auto snapshot = world.buildBroadPhaseSnapshot(snapshotSettings);

  CollisionOption option = CollisionOption::fullContacts(1000);
  auto result = RunCollideAll(world, snapshot, option, 4, 1);
  EXPECT_EQ(result.numContacts(), 0u);
  EXPECT_EQ(result.numManifolds(), 0u);
}
