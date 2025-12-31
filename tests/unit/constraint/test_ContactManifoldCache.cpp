/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the BSD-style License.
 */

#include <dart/constraint/ContactManifoldCache.hpp>

#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/Contact.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <vector>

using namespace dart;

namespace {

class DummyCollisionObject : public collision::CollisionObject
{
public:
  DummyCollisionObject() : CollisionObject(nullptr, nullptr) {}

private:
  void updateEngineData() override {}
};

class TestCollisionResult : public collision::CollisionResult
{
public:
  void addContactRaw(const collision::Contact& contact)
  {
    mContacts.push_back(contact);
  }

  void clearRaw()
  {
    mContacts.clear();
  }
};

collision::Contact makeContact(
    collision::CollisionObject* objA,
    collision::CollisionObject* objB,
    const Eigen::Vector3d& point,
    double depth)
{
  collision::Contact contact;
  contact.collisionObject1 = objA;
  contact.collisionObject2 = objB;
  contact.point = point;
  contact.normal = Eigen::Vector3d::UnitZ();
  contact.penetrationDepth = depth;
  return contact;
}

bool hasPoint(
    const std::vector<collision::Contact>& contacts,
    const Eigen::Vector3d& point,
    double tol = 1e-12)
{
  for (const auto& contact : contacts) {
    if (contact.point.isApprox(point, tol))
      return true;
  }
  return false;
}

std::size_t churnCount(
    const std::vector<collision::Contact>& prev,
    const std::vector<collision::Contact>& curr,
    double tol = 1e-12)
{
  std::size_t churn = 0u;
  for (const auto& contact : prev) {
    if (!hasPoint(curr, contact.point, tol))
      ++churn;
  }
  for (const auto& contact : curr) {
    if (!hasPoint(prev, contact.point, tol))
      ++churn;
  }
  return churn;
}

} // namespace

TEST(ContactManifoldCache, DisabledClearsCache)
{
  constraint::ContactManifoldCache cache;
  constraint::ContactManifoldCacheOptions options;
  options.enabled = false;

  DummyCollisionObject objA;
  DummyCollisionObject objB;

  TestCollisionResult raw;
  raw.addContactRaw(makeContact(&objA, &objB, Eigen::Vector3d::Zero(), 0.1));

  std::vector<collision::Contact> output;
  cache.update(raw, options, output);

  EXPECT_TRUE(output.empty());
  EXPECT_EQ(cache.getNumManifolds(), 0u);
}

TEST(ContactManifoldCache, RetainsPointsAcrossFrames)
{
  constraint::ContactManifoldCache cache;
  constraint::ContactManifoldCacheOptions options;
  options.enabled = true;
  options.maxSeparationFrames = 2u;

  DummyCollisionObject objA;
  DummyCollisionObject objB;

  TestCollisionResult raw;
  raw.addContactRaw(
      makeContact(&objA, &objB, Eigen::Vector3d(0.0, 0.0, 0.0), 0.1));
  raw.addContactRaw(
      makeContact(&objA, &objB, Eigen::Vector3d(0.2, 0.0, 0.0), 0.2));

  std::vector<collision::Contact> output;
  cache.update(raw, options, output);

  EXPECT_EQ(output.size(), 2u);
  EXPECT_TRUE(hasPoint(output, Eigen::Vector3d(0.0, 0.0, 0.0)));
  EXPECT_TRUE(hasPoint(output, Eigen::Vector3d(0.2, 0.0, 0.0)));

  const auto firstOutput = output;

  raw.clearRaw();
  raw.addContactRaw(
      makeContact(&objA, &objB, Eigen::Vector3d(0.0, 0.0, 0.0), 0.1));

  cache.update(raw, options, output);

  EXPECT_EQ(output.size(), 2u);
  EXPECT_TRUE(hasPoint(output, Eigen::Vector3d(0.0, 0.0, 0.0)));
  EXPECT_TRUE(hasPoint(output, Eigen::Vector3d(0.2, 0.0, 0.0)));
  EXPECT_LT(churnCount(firstOutput, output), churnCount(firstOutput, raw.getContacts()));
}

TEST(ContactManifoldCache, CapsContactsAndKeepsDeepest)
{
  constraint::ContactManifoldCache cache;
  constraint::ContactManifoldCacheOptions options;
  options.enabled = true;
  options.maxPointsPerPair = 4u;

  DummyCollisionObject objA;
  DummyCollisionObject objB;

  TestCollisionResult raw;
  for (int i = 0; i < 6; ++i) {
    const double depth = 0.1 * (i + 1);
    raw.addContactRaw(
        makeContact(&objA, &objB, Eigen::Vector3d(i, 0.0, 0.0), depth));
  }

  std::vector<collision::Contact> output;
  cache.update(raw, options, output);

  EXPECT_EQ(output.size(), 4u);
  EXPECT_TRUE(hasPoint(output, Eigen::Vector3d(5.0, 0.0, 0.0)));
}

TEST(ContactManifoldCache, PrunesStalePairs)
{
  constraint::ContactManifoldCache cache;
  constraint::ContactManifoldCacheOptions options;
  options.enabled = true;
  options.maxSeparationFrames = 1u;

  DummyCollisionObject objA;
  DummyCollisionObject objB;

  TestCollisionResult raw;
  raw.addContactRaw(makeContact(&objA, &objB, Eigen::Vector3d::Zero(), 0.1));

  std::vector<collision::Contact> output;
  cache.update(raw, options, output);
  EXPECT_EQ(cache.getNumManifolds(), 1u);
  ASSERT_EQ(output.size(), 1u);
  EXPECT_TRUE(hasPoint(output, Eigen::Vector3d::Zero()));

  raw.clearRaw();
  cache.update(raw, options, output);
  EXPECT_EQ(cache.getNumManifolds(), 1u);
  ASSERT_EQ(output.size(), 1u);
  EXPECT_TRUE(hasPoint(output, Eigen::Vector3d::Zero()));

  cache.update(raw, options, output);
  EXPECT_EQ(cache.getNumManifolds(), 0u);
  EXPECT_TRUE(output.empty());
}
