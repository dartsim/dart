/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the BSD-style License.
 */

#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/simulation/World.hpp>

#include <dart/collision/CollisionObject.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/WeldJoint.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

using namespace dart;
using namespace dart::collision;
using namespace dart::dynamics;
using namespace dart::simulation;

namespace {

SkeletonPtr createGround()
{
  auto ground = Skeleton::create("ground");
  auto pair = ground->createJointAndBodyNodePair<WeldJoint>(nullptr);
  auto shape
      = std::make_shared<BoxShape>(Eigen::Vector3d(10.0, 10.0, 0.2));
  pair.second->createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -0.1);
  pair.first->setTransformFromParentBodyNode(tf);

  return ground;
}

SkeletonPtr createBox()
{
  auto box = Skeleton::create("box");
  auto pair = box->createJointAndBodyNodePair<WeldJoint>(nullptr);
  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(1.0));
  pair.second->createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.0, 0.0, 0.45);
  pair.first->setTransformFromParentBodyNode(tf);

  return box;
}

bool contactLess(const Contact& a, const Contact& b)
{
  if (a.penetrationDepth != b.penetrationDepth)
    return a.penetrationDepth > b.penetrationDepth;

  if (a.point.x() != b.point.x())
    return a.point.x() < b.point.x();
  if (a.point.y() != b.point.y())
    return a.point.y() < b.point.y();
  if (a.point.z() != b.point.z())
    return a.point.z() < b.point.z();

  if (a.normal.x() != b.normal.x())
    return a.normal.x() < b.normal.x();
  if (a.normal.y() != b.normal.y())
    return a.normal.y() < b.normal.y();
  if (a.normal.z() != b.normal.z())
    return a.normal.z() < b.normal.z();

  return false;
}

bool isContactApproxEqual(
    const Contact& lhs, const Contact& rhs, double tol = 1e-12)
{
  if (!lhs.point.isApprox(rhs.point, tol))
    return false;
  if (!lhs.normal.isApprox(rhs.normal, tol))
    return false;
  if (std::abs(lhs.penetrationDepth - rhs.penetrationDepth) > tol)
    return false;

  const auto* lhsShape1
      = lhs.collisionObject1 ? lhs.collisionObject1->getShapeFrame() : nullptr;
  const auto* lhsShape2
      = lhs.collisionObject2 ? lhs.collisionObject2->getShapeFrame() : nullptr;
  const auto* rhsShape1
      = rhs.collisionObject1 ? rhs.collisionObject1->getShapeFrame() : nullptr;
  const auto* rhsShape2
      = rhs.collisionObject2 ? rhs.collisionObject2->getShapeFrame() : nullptr;

  const auto lhsPair = std::make_pair(
      lhsShape1 ? lhsShape1->getName() : std::string(),
      lhsShape2 ? lhsShape2->getName() : std::string());
  const auto rhsPair = std::make_pair(
      rhsShape1 ? rhsShape1->getName() : std::string(),
      rhsShape2 ? rhsShape2->getName() : std::string());

  return lhsPair == rhsPair;
}

std::vector<Contact> sortedContacts(const std::vector<Contact>& contacts)
{
  std::vector<Contact> sorted = contacts;
  std::sort(sorted.begin(), sorted.end(), contactLess);
  return sorted;
}

} // namespace

TEST(Simulation, ContactPatchCacheKeepsCollisionResultStable)
{
  auto world = World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  world->addSkeleton(createGround());
  world->addSkeleton(createBox());

  auto* solver = world->getConstraintSolver();
  ASSERT_NE(solver, nullptr);

  world->step();
  auto contactsOff = sortedContacts(world->getLastCollisionResult().getContacts());
  ASSERT_FALSE(contactsOff.empty());
  std::vector<Contact> constraintContactsOff;
  solver->getContactsUsedForConstraints(constraintContactsOff);
  EXPECT_EQ(constraintContactsOff.size(), contactsOff.size());

  solver->setContactPatchCacheEnabled(true);

  world->step();
  auto contactsOn = sortedContacts(world->getLastCollisionResult().getContacts());
  std::vector<Contact> constraintContactsOn;
  solver->getContactsUsedForConstraints(constraintContactsOn);
  EXPECT_EQ(constraintContactsOn.size(), solver->getNumPersistentContacts());

  ASSERT_EQ(contactsOff.size(), contactsOn.size());
  for (std::size_t i = 0; i < contactsOff.size(); ++i) {
    EXPECT_TRUE(isContactApproxEqual(contactsOff[i], contactsOn[i]))
        << "Contact mismatch at index " << i;
  }
}
