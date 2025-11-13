/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "../../helpers/GTestUtils.hpp"
#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/CollisionResult.hpp"
#include "dart/collision/Contact.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Shape.hpp"
#include "dart/dynamics/ShapeNode.hpp"
#include "dart/dynamics/Skeleton.hpp"

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

class DummyCollisionObject : public CollisionObject
{
public:
  explicit DummyCollisionObject(const dynamics::ShapeFrame* frame)
    : CollisionObject(nullptr, frame)
  {
  }

  void updateEngineData() override {}
};

Contact makeContact(CollisionObject* a, CollisionObject* b)
{
  Contact contact;
  contact.collisionObject1 = a;
  contact.collisionObject2 = b;
  contact.point = Eigen::Vector3d::Zero();
  contact.normal = Eigen::Vector3d::UnitZ();
  contact.penetrationDepth = 0.1;
  return contact;
}

} // namespace

//==============================================================================
TEST(CollisionResultTests, TracksCollidingFramesAndBodies)
{
  auto skeletonA = makeSkeleton("a");
  auto skeletonB = makeSkeleton("b");

  auto shapeNodeA = skeletonA->getBodyNode(0)->getShapeNode(0);
  auto shapeNodeB = skeletonB->getBodyNode(0)->getShapeNode(0);

  auto objectA = std::make_unique<DummyCollisionObject>(shapeNodeA);
  auto objectB = std::make_unique<DummyCollisionObject>(shapeNodeB);

  CollisionResult result;
  result.addContact(makeContact(objectA.get(), objectB.get()));

  EXPECT_TRUE(result.isCollision());
  EXPECT_TRUE(result);
  EXPECT_EQ(result.getNumContacts(), 1u);

  EXPECT_TRUE(result.inCollision(skeletonA->getBodyNode(0)));
  EXPECT_TRUE(result.inCollision(skeletonB->getBodyNode(0)));
  EXPECT_TRUE(result.inCollision(shapeNodeA));
  EXPECT_TRUE(result.inCollision(shapeNodeB));

  const auto& contacts = result.getContacts();
  ASSERT_EQ(contacts.size(), 1u);
  EXPECT_EQ(contacts.front().collisionObject1, objectA.get());
  EXPECT_EQ(contacts.front().collisionObject2, objectB.get());

  EXPECT_EQ(result.getCollidingBodyNodes().size(), 2u);
  EXPECT_EQ(result.getCollidingShapeFrames().size(), 2u);
}

//==============================================================================
TEST(CollisionResultTests, ClearRemovesContactsAndCollisionSets)
{
  auto skeleton = makeSkeleton("solo");
  auto shapeNode = skeleton->getBodyNode(0)->getShapeNode(0);
  auto object = std::make_unique<DummyCollisionObject>(shapeNode);

  CollisionResult result;
  result.addContact(makeContact(object.get(), object.get()));
  ASSERT_TRUE(result.isCollision());
  ASSERT_EQ(result.getNumContacts(), 1u);
  ASSERT_TRUE(result.inCollision(shapeNode));

  result.clear();
  EXPECT_FALSE(result.isCollision());
  EXPECT_EQ(result.getNumContacts(), 0u);
  EXPECT_FALSE(result.inCollision(shapeNode));
  EXPECT_TRUE(result.getCollidingBodyNodes().empty());
  EXPECT_TRUE(result.getCollidingShapeFrames().empty());
}
