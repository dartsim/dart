/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the BSD-style License.
 */

#include <dart/constraint/ContactManifoldCache.hpp>

#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/Contact.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/SimpleFrame.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

using namespace dart;

namespace {

class DummyCollisionDetector final : public collision::CollisionDetector
{
public:
  std::shared_ptr<CollisionDetector> cloneWithoutCollisionObjects()
      const override
  {
    return std::make_shared<DummyCollisionDetector>();
  }

  const std::string& getType() const override
  {
    static const std::string kType = "dummy";
    return kType;
  }

  std::unique_ptr<collision::CollisionGroup> createCollisionGroup() override
  {
    return nullptr;
  }

  bool collide(
      collision::CollisionGroup* /*group*/,
      const collision::CollisionOption& /*option*/,
      collision::CollisionResult* /*result*/) override
  {
    return false;
  }

  bool collide(
      collision::CollisionGroup* /*group1*/,
      collision::CollisionGroup* /*group2*/,
      const collision::CollisionOption& /*option*/,
      collision::CollisionResult* /*result*/) override
  {
    return false;
  }

  double distance(
      collision::CollisionGroup* /*group*/,
      const collision::DistanceOption& /*option*/,
      collision::DistanceResult* /*result*/) override
  {
    return 0.0;
  }

  double distance(
      collision::CollisionGroup* /*group1*/,
      collision::CollisionGroup* /*group2*/,
      const collision::DistanceOption& /*option*/,
      collision::DistanceResult* /*result*/) override
  {
    return 0.0;
  }

protected:
  std::unique_ptr<collision::CollisionObject> createCollisionObject(
      const dynamics::ShapeFrame* /*shapeFrame*/) override
  {
    return nullptr;
  }

  void refreshCollisionObject(collision::CollisionObject* /*object*/) override
  {
  }
};

class DummyCollisionObject : public collision::CollisionObject
{
public:
  DummyCollisionObject(
      collision::CollisionDetector* detector,
      const dynamics::ShapeFrame* shapeFrame)
    : CollisionObject(detector, shapeFrame)
  {
  }

private:
  void updateEngineData() override {}
};

struct TestCollisionObjects
{
  DummyCollisionDetector detector;
  dynamics::SimpleFrame frameA;
  dynamics::SimpleFrame frameB;
  DummyCollisionObject objA;
  DummyCollisionObject objB;

  TestCollisionObjects()
    : frameA(), frameB(), objA(&detector, &frameA), objB(&detector, &frameB)
  {
    const auto shape
        = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones());
    frameA.setShape(shape);
    frameB.setShape(shape);
  }
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

  TestCollisionObjects objects;

  TestCollisionResult raw;
  raw.addContactRaw(
      makeContact(&objects.objA, &objects.objB, Eigen::Vector3d::Zero(), 0.1));

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

  TestCollisionObjects objects;

  TestCollisionResult raw;
  raw.addContactRaw(makeContact(
      &objects.objA, &objects.objB, Eigen::Vector3d(0.0, 0.0, 0.0), 0.1));
  raw.addContactRaw(makeContact(
      &objects.objA, &objects.objB, Eigen::Vector3d(0.2, 0.0, 0.0), 0.2));

  std::vector<collision::Contact> output;
  cache.update(raw, options, output);

  EXPECT_EQ(output.size(), 2u);
  EXPECT_TRUE(hasPoint(output, Eigen::Vector3d(0.0, 0.0, 0.0)));
  EXPECT_TRUE(hasPoint(output, Eigen::Vector3d(0.2, 0.0, 0.0)));

  const auto firstOutput = output;

  raw.clearRaw();
  raw.addContactRaw(makeContact(
      &objects.objA, &objects.objB, Eigen::Vector3d(0.0, 0.0, 0.0), 0.1));

  cache.update(raw, options, output);

  EXPECT_EQ(output.size(), 2u);
  EXPECT_TRUE(hasPoint(output, Eigen::Vector3d(0.0, 0.0, 0.0)));
  EXPECT_TRUE(hasPoint(output, Eigen::Vector3d(0.2, 0.0, 0.0)));
  const auto rawContacts = std::vector<collision::Contact>(
      raw.getContacts().begin(), raw.getContacts().end());
  EXPECT_LT(
      churnCount(firstOutput, output), churnCount(firstOutput, rawContacts));
}

TEST(ContactManifoldCache, CapsContactsAndKeepsDeepest)
{
  constraint::ContactManifoldCache cache;
  constraint::ContactManifoldCacheOptions options;
  options.enabled = true;
  options.maxPointsPerPair = 4u;

  TestCollisionObjects objects;

  TestCollisionResult raw;
  for (int i = 0; i < 6; ++i) {
    const double depth = 0.1 * (i + 1);
    raw.addContactRaw(makeContact(
        &objects.objA, &objects.objB, Eigen::Vector3d(i, 0.0, 0.0), depth));
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

  TestCollisionObjects objects;

  TestCollisionResult raw;
  raw.addContactRaw(
      makeContact(&objects.objA, &objects.objB, Eigen::Vector3d::Zero(), 0.1));

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
