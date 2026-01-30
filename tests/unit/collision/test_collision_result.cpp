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

#include "../../helpers/gtest_utils.hpp"
#include "dart/collision/collision_group.hpp"
#include "dart/collision/collision_option.hpp"
#include "dart/collision/collision_result.hpp"
#include "dart/collision/dart/dart_collision_detector.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/box_shape.hpp"
#include "dart/dynamics/free_joint.hpp"
#include "dart/dynamics/shape.hpp"
#include "dart/dynamics/shape_node.hpp"
#include "dart/dynamics/skeleton.hpp"

#include <gtest/gtest.h>

#include <memory>

using namespace dart;
using namespace dart::collision;
using namespace dart::dynamics;

namespace {

std::shared_ptr<Skeleton> makeSkeleton(const std::string& name)
{
  auto skeleton = Skeleton::create(name);
  skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto* body = skeleton->getBodyNode(0);
  body->createShapeNodeWith<CollisionAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.5)));
  return skeleton;
}

std::unique_ptr<CollisionGroup> makeGroupWithOverlap(
    const std::shared_ptr<Skeleton>& a,
    const std::shared_ptr<Skeleton>& b,
    const std::shared_ptr<CollisionDetector>& detector)
{
  auto group = detector->createCollisionGroup();
  group->addShapeFrame(a->getBodyNode(0)->getShapeNode(0));
  group->addShapeFrame(b->getBodyNode(0)->getShapeNode(0));
  return group;
}

} // namespace

//==============================================================================
TEST(CollisionResultTests, TracksCollidingFramesAndBodies)
{
  auto detector = DARTCollisionDetector::create();
  auto skeletonA = makeSkeleton("a");
  auto skeletonB = makeSkeleton("b");
  auto group = makeGroupWithOverlap(skeletonA, skeletonB, detector);

  CollisionOption option;
  option.maxNumContacts = 1u;
  CollisionResult result;
  ASSERT_TRUE(group->collide(option, &result));

  EXPECT_TRUE(result.isCollision());
  EXPECT_TRUE(result);
  EXPECT_EQ(result.getNumContacts(), 1u);

  auto* bodyA = skeletonA->getBodyNode(0);
  auto* bodyB = skeletonB->getBodyNode(0);
  auto* shapeA = bodyA->getShapeNode(0);
  auto* shapeB = bodyB->getShapeNode(0);

  EXPECT_TRUE(result.inCollision(bodyA));
  EXPECT_TRUE(result.inCollision(bodyB));
  EXPECT_TRUE(result.inCollision(shapeA));
  EXPECT_TRUE(result.inCollision(shapeB));

  EXPECT_EQ(result.getCollidingBodyNodes().size(), 2u);
  EXPECT_EQ(result.getCollidingShapeFrames().size(), 2u);
}

//==============================================================================
TEST(CollisionResultTests, ContactProvidesBodyNodeAccessors)
{
  auto detector = DARTCollisionDetector::create();
  auto skeletonA = makeSkeleton("bodynode_a");
  auto skeletonB = makeSkeleton("bodynode_b");
  auto group = makeGroupWithOverlap(skeletonA, skeletonB, detector);

  CollisionOption option;
  option.maxNumContacts = 2u;

  CollisionResult result;
  ASSERT_TRUE(group->collide(option, &result));
  ASSERT_LT(0u, result.getNumContacts());

  const auto& contact = result.getContact(0);

  ASSERT_NE(nullptr, contact.getShapeFrame1());
  ASSERT_NE(nullptr, contact.getShapeFrame2());
  ASSERT_NE(nullptr, contact.getShapeNode1());
  ASSERT_NE(nullptr, contact.getShapeNode2());

  const auto bodyNode1 = contact.getBodyNodePtr1();
  const auto bodyNode2 = contact.getBodyNodePtr2();
  ASSERT_TRUE(bodyNode1);
  ASSERT_TRUE(bodyNode2);
  EXPECT_NE(bodyNode1.get(), bodyNode2.get());

  EXPECT_TRUE(
      bodyNode1.get() == skeletonA->getBodyNode(0)
      || bodyNode1.get() == skeletonB->getBodyNode(0));
  EXPECT_TRUE(
      bodyNode2.get() == skeletonA->getBodyNode(0)
      || bodyNode2.get() == skeletonB->getBodyNode(0));
}

//==============================================================================
TEST(CollisionResultTests, ClearRemovesContactsAndCollisionSets)
{
  auto detector = DARTCollisionDetector::create();
  auto skeletonA = makeSkeleton("solo_a");
  auto skeletonB = makeSkeleton("solo_b");
  auto group = makeGroupWithOverlap(skeletonA, skeletonB, detector);

  CollisionOption option;
  option.maxNumContacts = 1u;
  CollisionResult result;
  ASSERT_TRUE(group->collide(option, &result));
  ASSERT_TRUE(result.isCollision());
  ASSERT_EQ(result.getNumContacts(), 1u);

  auto* shapeA = skeletonA->getBodyNode(0)->getShapeNode(0);
  auto* shapeB = skeletonB->getBodyNode(0)->getShapeNode(0);
  ASSERT_TRUE(result.inCollision(shapeA));
  ASSERT_TRUE(result.inCollision(shapeB));

  result.clear();
  EXPECT_FALSE(result.isCollision());
  EXPECT_EQ(result.getNumContacts(), 0u);
  EXPECT_FALSE(result.inCollision(shapeA));
  EXPECT_FALSE(result.inCollision(shapeB));
  EXPECT_TRUE(result.getCollidingBodyNodes().empty());
  EXPECT_TRUE(result.getCollidingShapeFrames().empty());
}

//==============================================================================
TEST(CollisionResultTests, GetContactReturnsValidContact)
{
  auto detector = DARTCollisionDetector::create();
  auto skeletonA = makeSkeleton("contact_a");
  auto skeletonB = makeSkeleton("contact_b");
  auto group = makeGroupWithOverlap(skeletonA, skeletonB, detector);

  CollisionOption option;
  option.maxNumContacts = 5u;
  CollisionResult result;
  ASSERT_TRUE(group->collide(option, &result));

  const std::size_t numContacts = result.getNumContacts();
  ASSERT_GT(numContacts, 0u);

  const auto& constResult = result;
  const Contact& contact = constResult.getContact(0);
  EXPECT_TRUE(contact.point.allFinite());
  EXPECT_TRUE(contact.normal.allFinite());

  Contact& mutableContact = result.getContact(0);
  Eigen::Vector3d originalPoint = mutableContact.point;
  mutableContact.point = Eigen::Vector3d(1.0, 2.0, 3.0);
  EXPECT_EQ(result.getContact(0).point, Eigen::Vector3d(1.0, 2.0, 3.0));
  mutableContact.point = originalPoint;
}

//==============================================================================
TEST(CollisionResultTests, GetContactsReturnsSpan)
{
  auto detector = DARTCollisionDetector::create();
  auto skeletonA = makeSkeleton("span_a");
  auto skeletonB = makeSkeleton("span_b");
  auto group = makeGroupWithOverlap(skeletonA, skeletonB, detector);

  CollisionOption option;
  option.maxNumContacts = 10u;
  CollisionResult result;
  ASSERT_TRUE(group->collide(option, &result));

  auto contacts = result.getContacts();
  EXPECT_EQ(contacts.size(), result.getNumContacts());

  for (const auto& contact : contacts) {
    EXPECT_TRUE(contact.point.allFinite());
    EXPECT_NE(contact.collisionObject1, nullptr);
    EXPECT_NE(contact.collisionObject2, nullptr);
  }
}

//==============================================================================
TEST(CollisionResultTests, EmptyResultHasNoContacts)
{
  CollisionResult result;

  EXPECT_FALSE(result.isCollision());
  EXPECT_FALSE(result);
  EXPECT_EQ(result.getNumContacts(), 0u);
  EXPECT_TRUE(result.getContacts().empty());
  EXPECT_TRUE(result.getCollidingBodyNodes().empty());
  EXPECT_TRUE(result.getCollidingShapeFrames().empty());
}

//==============================================================================
TEST(CollisionResultTests, MaxNumContactsLimitsResults)
{
  auto detector = DARTCollisionDetector::create();
  auto skeletonA = makeSkeleton("limit_a");
  auto skeletonB = makeSkeleton("limit_b");
  auto group = makeGroupWithOverlap(skeletonA, skeletonB, detector);

  CollisionOption option1;
  option1.maxNumContacts = 1u;
  CollisionResult result1;
  ASSERT_TRUE(group->collide(option1, &result1));
  EXPECT_EQ(result1.getNumContacts(), 1u);

  CollisionOption option2;
  option2.maxNumContacts = 2u;
  CollisionResult result2;
  ASSERT_TRUE(group->collide(option2, &result2));
  EXPECT_LE(result2.getNumContacts(), 2u);
}
