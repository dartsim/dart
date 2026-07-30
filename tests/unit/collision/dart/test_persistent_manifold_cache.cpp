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

#include <dart/collision/dart/PersistentManifoldCache.hpp>

#include <gtest/gtest.h>

#include <array>
#include <optional>

#include <cmath>

using namespace dart::collision::native;

//==============================================================================
TEST(PersistentManifoldCache, CreateAndRetrieve)
{
  PersistentManifoldCache cache;
  auto& manifold = cache.getOrCreate(11u, 29u);

  CachedContact contact;
  contact.localPointA = Eigen::Vector3d(0.1, 0.0, 0.0);
  contact.localPointB = Eigen::Vector3d(0.1, 0.0, 0.0);
  contact.penetrationDepth = 0.02;
  manifold.addOrReplace(contact);

  EXPECT_EQ(1u, cache.size());
  auto& same = cache.getOrCreate(11u, 29u);
  EXPECT_EQ(1, same.numContacts);
  EXPECT_NEAR(0.02, same.contacts[0].penetrationDepth, 1e-12);
}

//==============================================================================
TEST(PersistentManifoldCache, PairKeySymmetry)
{
  PersistentManifoldCache cache;

  auto& manifoldAB = cache.getOrCreate(3u, 9u);
  auto& manifoldBA = cache.getOrCreate(9u, 3u);

  EXPECT_EQ(&manifoldAB, &manifoldBA);
  EXPECT_EQ(1u, cache.size());
}

//==============================================================================
TEST(PersistentManifoldCache, ContactMatchingPreservesCachedImpulses)
{
  PersistentManifold manifold;

  CachedContact first;
  first.localPointA = Eigen::Vector3d(0.1, 0.2, 0.3);
  first.localPointB = Eigen::Vector3d(0.1, 0.2, 0.3);
  first.cachedNormalImpulse = 1.2;
  first.cachedFrictionImpulse1 = -0.3;
  first.cachedFrictionImpulse2 = 0.7;
  first.cachedFrictionBasis1 = Eigen::Vector3d::UnitX();
  first.cachedFrictionBasis2 = Eigen::Vector3d::UnitY();
  first.hasCachedFrictionBasis = true;
  manifold.addOrReplace(first);

  CachedContact second;
  second.localPointA = Eigen::Vector3d(0.1005, 0.2, 0.3005);
  second.localPointB = Eigen::Vector3d(0.1005, 0.2, 0.3005);
  second.penetrationDepth = 0.04;
  manifold.addOrReplace(second);

  ASSERT_EQ(1, manifold.numContacts);
  EXPECT_NEAR(1.2, manifold.contacts[0].cachedNormalImpulse, 1e-12);
  EXPECT_NEAR(-0.3, manifold.contacts[0].cachedFrictionImpulse1, 1e-12);
  EXPECT_NEAR(0.7, manifold.contacts[0].cachedFrictionImpulse2, 1e-12);
  EXPECT_TRUE(manifold.contacts[0].hasCachedFrictionBasis);
  EXPECT_TRUE(manifold.contacts[0].cachedFrictionBasis1.isApprox(
      Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(manifold.contacts[0].cachedFrictionBasis2.isApprox(
      Eigen::Vector3d::UnitY()));
  EXPECT_EQ(0, manifold.contacts[0].lifetime);
}

//==============================================================================
TEST(PersistentManifoldCache, ContactMatchingClearsFrictionForChangedNormal)
{
  PersistentManifold manifold;

  CachedContact first;
  first.localPointA = Eigen::Vector3d(0.1, 0.2, 0.3);
  first.localPointB = Eigen::Vector3d(0.1, 0.2, 0.3);
  first.normal = Eigen::Vector3d::UnitX();
  first.cachedNormalImpulse = 1.2;
  first.cachedFrictionImpulse1 = -0.3;
  first.cachedFrictionImpulse2 = 0.7;
  first.cachedFrictionBasis1 = Eigen::Vector3d::UnitY();
  first.cachedFrictionBasis2 = Eigen::Vector3d::UnitZ();
  first.hasCachedFrictionBasis = true;
  manifold.addOrReplace(first);

  CachedContact second;
  second.localPointA = Eigen::Vector3d(0.1005, 0.2, 0.3005);
  second.localPointB = Eigen::Vector3d(0.1005, 0.2, 0.3005);
  second.normal = -Eigen::Vector3d::UnitX();
  second.penetrationDepth = 0.04;
  manifold.addOrReplace(second);

  ASSERT_EQ(1, manifold.numContacts);
  EXPECT_NEAR(1.2, manifold.contacts[0].cachedNormalImpulse, 1e-12);
  EXPECT_NEAR(0.0, manifold.contacts[0].cachedFrictionImpulse1, 1e-12);
  EXPECT_NEAR(0.0, manifold.contacts[0].cachedFrictionImpulse2, 1e-12);
  EXPECT_FALSE(manifold.contacts[0].hasCachedFrictionBasis);
  EXPECT_TRUE(manifold.contacts[0].cachedFrictionBasis1.isZero());
  EXPECT_TRUE(manifold.contacts[0].cachedFrictionBasis2.isZero());
  EXPECT_EQ(0, manifold.contacts[0].lifetime);
}

//==============================================================================
TEST(PersistentManifoldCache, ReductionKeepsDeepAndWideContacts)
{
  PersistentManifold manifold;

  for (int i = 0; i < 6; ++i) {
    CachedContact contact;
    contact.localPointA = Eigen::Vector3d(0.04 * i, 0.03 * (i % 3), 0.0);
    contact.localPointB = contact.localPointA;
    contact.penetrationDepth = (i == 4) ? 0.25 : 0.01 * i;
    manifold.addOrReplace(contact);
  }

  EXPECT_EQ(4, manifold.numContacts);
  bool foundDeepest = false;
  for (int i = 0; i < manifold.numContacts; ++i) {
    if (std::abs(
            manifold.contacts[static_cast<std::size_t>(i)].penetrationDepth
            - 0.25)
        < 1e-12) {
      foundDeepest = true;
      break;
    }
  }
  EXPECT_TRUE(foundDeepest);
}

//==============================================================================
TEST(PersistentManifoldCache, ReductionCanDropInsertedContact)
{
  PersistentManifold manifold;

  std::array<Eigen::Vector3d, PersistentManifold::kMaxContacts> points{
      Eigen::Vector3d(-1.0, -1.0, 0.0),
      Eigen::Vector3d(1.0, -1.0, 0.0),
      Eigen::Vector3d(-1.0, 1.0, 0.0),
      Eigen::Vector3d(1.0, 1.0, 0.0)};

  for (std::size_t i = 0u; i < points.size(); ++i) {
    CachedContact contact;
    contact.localPointA = points[i];
    contact.localPointB = points[i];
    contact.penetrationDepth = 0.2 - 0.01 * static_cast<double>(i);
    manifold.addOrReplace(contact);
  }

  CachedContact inserted;
  inserted.localPointA = Eigen::Vector3d::Zero();
  inserted.localPointB = inserted.localPointA;
  inserted.penetrationDepth = 0.01;
  manifold.addOrReplace(inserted);

  EXPECT_EQ(PersistentManifold::kMaxContacts, manifold.numContacts);
  EXPECT_EQ(-1, manifold.findMatch(inserted.localPointA));
}

//==============================================================================
TEST(PersistentManifoldCache, RefreshRemovesDriftedContact)
{
  PersistentManifold manifold;
  CachedContact contact;
  contact.localPointA = Eigen::Vector3d::Zero();
  contact.localPointB = Eigen::Vector3d::Zero();
  contact.normal = Eigen::Vector3d::UnitX();
  manifold.addOrReplace(contact);

  Eigen::Isometry3d tfA = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();
  tfB.translation() = Eigen::Vector3d(0.1, 0.0, 0.0);

  manifold.refresh(tfA, tfB, 0.04);
  EXPECT_EQ(0, manifold.numContacts);
}

//==============================================================================
TEST(PersistentManifoldCache, RefreshAllKeepsUnknownPairsAndDropsDriftedPairs)
{
  PersistentManifoldCache cache;

  CachedContact stable;
  stable.localPointA = Eigen::Vector3d::Zero();
  stable.localPointB = Eigen::Vector3d::Zero();
  stable.normal = Eigen::Vector3d::Zero();
  cache.getOrCreate(1u, 2u).addOrReplace(stable);

  CachedContact missing;
  missing.localPointA = Eigen::Vector3d::Zero();
  missing.localPointB = Eigen::Vector3d::Zero();
  cache.getOrCreate(3u, 4u).addOrReplace(missing);

  CachedContact drifting;
  drifting.localPointA = Eigen::Vector3d::Zero();
  drifting.localPointB = Eigen::Vector3d::Zero();
  drifting.normal = Eigen::Vector3d::UnitX();
  cache.getOrCreate(5u, 6u).addOrReplace(drifting);

  cache.refreshAll(
      [](std::size_t idA, std::size_t idB) {
        if (idA == 3u || idB == 3u) {
          return std::optional<
              std::pair<Eigen::Isometry3d, Eigen::Isometry3d>>();
        }

        Eigen::Isometry3d tfA = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();
        if (idA == 5u || idB == 5u)
          tfB.translation() = Eigen::Vector3d(0.2, 0.0, 0.0);

        return std::optional<std::pair<Eigen::Isometry3d, Eigen::Isometry3d>>(
            std::make_pair(tfA, tfB));
      },
      0.04);

  EXPECT_EQ(2u, cache.size());
  auto& kept = cache.getOrCreate(2u, 1u);
  ASSERT_EQ(1, kept.numContacts);
  EXPECT_EQ(1, kept.contacts[0].lifetime);

  auto& unknown = cache.getOrCreate(4u, 3u);
  ASSERT_EQ(1, unknown.numContacts);
  EXPECT_EQ(0, unknown.contacts[0].lifetime);

  cache.remove(1u, 2u);
  EXPECT_EQ(1u, cache.size());
  cache.remove(3u, 4u);
  EXPECT_EQ(0u, cache.size());
  cache.getOrCreate(7u, 8u);
  EXPECT_EQ(1u, cache.size());
  cache.clear();
  EXPECT_EQ(0u, cache.size());
}

//==============================================================================
TEST(PersistentManifoldCache, RemoveObjectDropsEveryPairContainingObject)
{
  PersistentManifoldCache cache;

  CachedContact contact;
  contact.localPointA = Eigen::Vector3d::Zero();
  contact.localPointB = Eigen::Vector3d::Zero();

  cache.getOrCreate(1u, 2u).addOrReplace(contact);
  cache.getOrCreate(2u, 3u).addOrReplace(contact);
  cache.getOrCreate(4u, 5u).addOrReplace(contact);

  cache.removeObject(2u);

  EXPECT_EQ(1u, cache.size());
  auto& kept = cache.getOrCreate(5u, 4u);
  EXPECT_EQ(1, kept.numContacts);
  EXPECT_EQ(1u, cache.size());
}
