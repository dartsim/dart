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

#include "pair_registry.hpp"

#include <dart/collision/native/collision_world.hpp>
#include <dart/collision/native/narrow_phase/narrow_phase.hpp>

#include <gtest/gtest.h>

#include <set>

namespace sandbox = dart::examples::collision_sandbox;
namespace collision = dart::collision::native;

namespace {

std::size_t shapeRank(collision::ShapeType type)
{
  const auto fixtures = sandbox::shapeFixtures();
  for (std::size_t i = 0; i < fixtures.size(); ++i) {
    if (fixtures[i].type == type) {
      return i;
    }
  }
  return fixtures.size();
}

} // namespace

TEST(CollisionSandboxPairRegistry, CoversEveryUnorderedShapePair)
{
  const auto fixtures = sandbox::shapeFixtures();
  const auto cases = sandbox::pairCases();
  const std::size_t expectedPairCount
      = fixtures.size() * (fixtures.size() + 1) / 2;

  ASSERT_EQ(cases.size(), expectedPairCount);

  std::set<std::pair<collision::ShapeType, collision::ShapeType>> seen;
  for (std::size_t i = 0; i < cases.size(); ++i) {
    const sandbox::PairCase& pair = cases[i];
    EXPECT_EQ(pair.index, i);
    EXPECT_FALSE(pair.id.empty());
    EXPECT_FALSE(pair.label.empty());
    EXPECT_FALSE(pair.note.empty());
    EXPECT_LE(shapeRank(pair.shapeA), shapeRank(pair.shapeB));

    const auto [_, inserted] = seen.emplace(pair.shapeA, pair.shapeB);
    EXPECT_TRUE(inserted) << pair.id;

    const sandbox::PairCase* byId = sandbox::findPairCase(pair.id);
    ASSERT_NE(byId, nullptr);
    EXPECT_EQ(byId->index, pair.index);
  }
}

TEST(CollisionSandboxPairRegistry, MatchesNarrowPhaseSupport)
{
  for (const sandbox::PairCase& pair : sandbox::pairCases()) {
    const bool contactSupported
        = collision::NarrowPhase::isSupported(pair.shapeA, pair.shapeB);
    const bool distanceSupported
        = collision::NarrowPhase::isDistanceSupported(pair.shapeA, pair.shapeB);

    EXPECT_EQ(pair.supportsContact(), contactSupported) << pair.id;
    EXPECT_EQ(pair.supportsDistance(), contactSupported || distanceSupported)
        << pair.id;

    if (!contactSupported && distanceSupported) {
      EXPECT_EQ(pair.status, sandbox::PairStatus::DistanceOnly) << pair.id;
    } else if (!contactSupported && !distanceSupported) {
      EXPECT_EQ(pair.status, sandbox::PairStatus::Unsupported) << pair.id;
    } else if (sandbox::isAdaptedFallbackPair(pair.shapeA, pair.shapeB)) {
      EXPECT_EQ(pair.status, sandbox::PairStatus::AdaptedFallback) << pair.id;
    } else {
      EXPECT_EQ(pair.status, sandbox::PairStatus::Contact) << pair.id;
    }
  }
}

TEST(CollisionSandboxPairRegistry, ShapeFactoryCreatesEveryFixture)
{
  for (const sandbox::ShapeFixture& fixture : sandbox::shapeFixtures()) {
    auto shape = sandbox::makeShape(fixture.type);
    ASSERT_NE(shape, nullptr) << fixture.id;
    EXPECT_EQ(shape->getType(), fixture.type) << fixture.id;
    EXPECT_TRUE(
        shape->computeLocalAabb().min.allFinite()
        || fixture.type == collision::ShapeType::Plane)
        << fixture.id;
  }
}

TEST(CollisionSandboxPairRegistry, LiveCasesCanRunNativeQueries)
{
  for (const sandbox::PairCase& pair : sandbox::pairCases()) {
    if (!pair.supportsDistance()) {
      continue;
    }

    auto shapeA = sandbox::makeShape(pair.shapeA);
    auto shapeB = sandbox::makeShape(pair.shapeB);
    const sandbox::PairPose pose = sandbox::defaultPairPose(pair);

    if (pair.supportsContact()) {
      collision::CollisionResult result;
      collision::CollisionOption option;
      option.maxNumContacts = 16;
      const bool hit = collision::NarrowPhase::collide(
          shapeA.get(),
          pose.transformA,
          shapeB.get(),
          pose.transformB,
          option,
          result);
      EXPECT_EQ(hit, result.isCollision()) << pair.id;
      EXPECT_LE(result.numContacts(), option.maxNumContacts) << pair.id;
    } else {
      collision::CollisionWorld world;
      auto objA = world.createObject(std::move(shapeA), pose.transformA);
      auto objB = world.createObject(std::move(shapeB), pose.transformB);
      collision::DistanceResult result;
      collision::DistanceOption option;
      collision::NarrowPhase::distance(objA, objB, option, result);
      EXPECT_TRUE(result.isValid()) << pair.id;
    }
  }
}
