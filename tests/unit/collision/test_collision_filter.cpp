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

#include <dart/collision/collision_filter.hpp>
#include <dart/collision/collision_group.hpp>
#include <dart/collision/collision_object.hpp>
#include <dart/collision/collision_option.hpp>
#include <dart/collision/collision_result.hpp>
#include <dart/collision/dart/dart_collision_detector.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::collision;
using namespace dart::dynamics;

namespace {

class AlwaysIgnoreFilter : public CollisionFilter
{
public:
  bool ignoresCollision(
      const CollisionObject* /*object1*/,
      const CollisionObject* /*object2*/) const override
  {
    return true;
  }
};

class NeverIgnoreFilter : public CollisionFilter
{
public:
  bool ignoresCollision(
      const CollisionObject* /*object1*/,
      const CollisionObject* /*object2*/) const override
  {
    return false;
  }
};

class StubCollisionObject : public CollisionObject
{
public:
  StubCollisionObject(
      CollisionDetector* detector, const dynamics::ShapeFrame* shapeFrame)
    : CollisionObject(detector, shapeFrame)
  {
  }

  void updateEngineData() override {}

  void setShapeFrame(const dynamics::ShapeFrame* shapeFrame)
  {
    mShapeFrame = shapeFrame;
  }
};

SkeletonPtr createSkeleton(const std::string& name)
{
  auto skel = Skeleton::create(name);
  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  pair.second->createShapeNodeWith<CollisionAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d::Constant(1.0)));
  return skel;
}

} // namespace

TEST(CompositeCollisionFilter, EmptyFilterAllowsAll)
{
  CompositeCollisionFilter composite;
  EXPECT_FALSE(composite.ignoresCollision(nullptr, nullptr));
}

TEST(CompositeCollisionFilter, AddAndRemoveFilter)
{
  CompositeCollisionFilter composite;
  AlwaysIgnoreFilter ignoreFilter;

  composite.addCollisionFilter(&ignoreFilter);
  EXPECT_TRUE(composite.ignoresCollision(nullptr, nullptr));

  composite.removeCollisionFilter(&ignoreFilter);
  EXPECT_FALSE(composite.ignoresCollision(nullptr, nullptr));
}

TEST(CompositeCollisionFilter, MultipleFilters)
{
  CompositeCollisionFilter composite;
  NeverIgnoreFilter neverIgnore;
  AlwaysIgnoreFilter alwaysIgnore;

  composite.addCollisionFilter(&neverIgnore);
  EXPECT_FALSE(composite.ignoresCollision(nullptr, nullptr));

  composite.addCollisionFilter(&alwaysIgnore);
  EXPECT_TRUE(composite.ignoresCollision(nullptr, nullptr));
}

TEST(CompositeCollisionFilter, RemoveAllFilters)
{
  CompositeCollisionFilter composite;
  AlwaysIgnoreFilter filter1;
  AlwaysIgnoreFilter filter2;

  composite.addCollisionFilter(&filter1);
  composite.addCollisionFilter(&filter2);
  EXPECT_TRUE(composite.ignoresCollision(nullptr, nullptr));

  composite.removeAllCollisionFilters();
  EXPECT_FALSE(composite.ignoresCollision(nullptr, nullptr));
}

TEST(CompositeCollisionFilter, AddNullptrDoesNothing)
{
  CompositeCollisionFilter composite;

  // Adding nullptr should not crash or change behavior
  composite.addCollisionFilter(nullptr);
  EXPECT_FALSE(composite.ignoresCollision(nullptr, nullptr));
}

// Test BodyNodeCollisionFilter through actual collision detection
// since we cannot directly access CollisionObject pointers
TEST(BodyNodeCollisionFilter, BlacklistPairThroughCollision)
{
  auto skel1 = createSkeleton("skel1");
  auto skel2 = createSkeleton("skel2");
  auto body1 = skel1->getBodyNode(0);
  auto body2 = skel2->getBodyNode(0);

  // Position skeletons to overlap
  skel1->getJoint(0)->setPosition(0, 0.0);
  skel2->getJoint(0)->setPosition(0, 0.5); // Overlap with offset

  auto detector = DARTCollisionDetector::create();
  auto group = detector->createCollisionGroup();
  group->addShapeFrame(body1->getShapeNode(0));
  group->addShapeFrame(body2->getShapeNode(0));

  CollisionOption option;
  CollisionResult result;

  // Initially should collide (overlapping boxes)
  EXPECT_TRUE(group->collide(option, &result));

  // Add filter with blacklist
  auto filter = std::make_shared<BodyNodeCollisionFilter>();
  filter->addBodyNodePairToBlackList(body1, body2);
  option.collisionFilter = filter;

  result.clear();
  EXPECT_FALSE(group->collide(option, &result));

  // Remove from blacklist
  filter->removeBodyNodePairFromBlackList(body1, body2);

  result.clear();
  EXPECT_TRUE(group->collide(option, &result));
}

TEST(BodyNodeCollisionFilter, RemoveAllFromBlacklistThroughCollision)
{
  auto skel1 = createSkeleton("skel1");
  auto skel2 = createSkeleton("skel2");
  auto body1 = skel1->getBodyNode(0);
  auto body2 = skel2->getBodyNode(0);

  // Position skeletons to overlap
  skel1->getJoint(0)->setPosition(0, 0.0);
  skel2->getJoint(0)->setPosition(0, 0.5);

  auto detector = DARTCollisionDetector::create();
  auto group = detector->createCollisionGroup();
  group->addShapeFrame(body1->getShapeNode(0));
  group->addShapeFrame(body2->getShapeNode(0));

  auto filter = std::make_shared<BodyNodeCollisionFilter>();
  filter->addBodyNodePairToBlackList(body1, body2);

  CollisionOption option;
  option.collisionFilter = filter;
  CollisionResult result;

  EXPECT_FALSE(group->collide(option, &result));

  filter->removeAllBodyNodePairsFromBlackList();

  result.clear();
  EXPECT_TRUE(group->collide(option, &result));
}

TEST(BodyNodeCollisionFilter, AdjacentBodiesIgnored)
{
  auto skel = Skeleton::create("chain");

  // Create parent body
  auto pair1 = skel->createJointAndBodyNodePair<FreeJoint>(nullptr);
  pair1.second->createShapeNodeWith<CollisionAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d::Constant(1.0)));

  // Create child body connected by revolute joint
  BodyNode::Properties props;
  props.mName = "child";
  RevoluteJoint::Properties jointProps;
  jointProps.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0.5, 0, 0);
  auto pair2 = skel->createJointAndBodyNodePair<RevoluteJoint>(
      pair1.second, jointProps, props);
  pair2.second->createShapeNodeWith<CollisionAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d::Constant(1.0)));

  // By default, self-collision is disabled, so no collision expected
  auto detector = DARTCollisionDetector::create();
  auto group = detector->createCollisionGroup();
  group->addShapeFrame(pair1.second->getShapeNode(0));
  group->addShapeFrame(pair2.second->getShapeNode(0));

  auto filter = std::make_shared<BodyNodeCollisionFilter>();
  CollisionOption option;
  option.collisionFilter = filter;
  CollisionResult result;

  // Adjacent bodies should be ignored by default (self-collision disabled)
  EXPECT_FALSE(group->collide(option, &result));

  // Enable self-collision but not adjacent check
  skel->enableSelfCollisionCheck();
  skel->disableAdjacentBodyCheck();

  result.clear();
  EXPECT_FALSE(group->collide(option, &result));

  // Enable adjacent body check - now they should collide
  skel->enableAdjacentBodyCheck();

  result.clear();
  EXPECT_TRUE(group->collide(option, &result));
}

TEST(BodyNodeCollisionFilter, NonCollidableBodyIgnored)
{
  auto skel1 = createSkeleton("skel1");
  auto skel2 = createSkeleton("skel2");
  auto body1 = skel1->getBodyNode(0);
  auto body2 = skel2->getBodyNode(0);

  // Position skeletons to overlap
  skel1->getJoint(0)->setPosition(0, 0.0);
  skel2->getJoint(0)->setPosition(0, 0.5);

  auto detector = DARTCollisionDetector::create();
  auto group = detector->createCollisionGroup();
  group->addShapeFrame(body1->getShapeNode(0));
  group->addShapeFrame(body2->getShapeNode(0));

  auto filter = std::make_shared<BodyNodeCollisionFilter>();
  CollisionOption option;
  option.collisionFilter = filter;
  CollisionResult result;

  // Initially should collide
  EXPECT_TRUE(group->collide(option, &result));

  // Make one body non-collidable
  body1->setCollidable(false);

  result.clear();
  EXPECT_FALSE(group->collide(option, &result));

  // Restore collidability
  body1->setCollidable(true);

  result.clear();
  EXPECT_TRUE(group->collide(option, &result));
}

TEST(BodyNodeCollisionFilter, ImmobileSkeletonsIgnored)
{
  auto skel1 = createSkeleton("skel1");
  auto skel2 = createSkeleton("skel2");
  auto body1 = skel1->getBodyNode(0);
  auto body2 = skel2->getBodyNode(0);

  // Position skeletons to overlap
  skel1->getJoint(0)->setPosition(0, 0.0);
  skel2->getJoint(0)->setPosition(0, 0.5);

  auto detector = DARTCollisionDetector::create();
  auto group = detector->createCollisionGroup();
  group->addShapeFrame(body1->getShapeNode(0));
  group->addShapeFrame(body2->getShapeNode(0));

  auto filter = std::make_shared<BodyNodeCollisionFilter>();
  CollisionOption option;
  option.collisionFilter = filter;
  CollisionResult result;

  // Initially should collide
  EXPECT_TRUE(group->collide(option, &result));

  // Make both skeletons immobile
  skel1->setMobile(false);
  skel2->setMobile(false);

  result.clear();
  EXPECT_FALSE(group->collide(option, &result));

  // Make one mobile again
  skel1->setMobile(true);

  result.clear();
  EXPECT_TRUE(group->collide(option, &result));
}

//==============================================================================
// Edge case tests for CompositeCollisionFilter
//==============================================================================

TEST(CompositeCollisionFilter, RemoveNonExistentFilter)
{
  CompositeCollisionFilter composite;
  AlwaysIgnoreFilter alwaysIgnore;
  NeverIgnoreFilter neverIgnore;

  composite.addCollisionFilter(&alwaysIgnore);
  EXPECT_TRUE(composite.ignoresCollision(nullptr, nullptr));

  composite.removeCollisionFilter(&neverIgnore);

  EXPECT_TRUE(composite.ignoresCollision(nullptr, nullptr));
}

TEST(CompositeCollisionFilter, AddSameFilterTwice)
{
  CompositeCollisionFilter composite;
  AlwaysIgnoreFilter alwaysIgnore;

  composite.addCollisionFilter(&alwaysIgnore);
  composite.addCollisionFilter(&alwaysIgnore);

  EXPECT_TRUE(composite.ignoresCollision(nullptr, nullptr));

  composite.removeCollisionFilter(&alwaysIgnore);

  EXPECT_FALSE(composite.ignoresCollision(nullptr, nullptr));
}

//==============================================================================
// Edge case tests for BodyNodeCollisionFilter
//==============================================================================

TEST(BodyNodeCollisionFilter, SameBodyNodeCollision)
{
  auto skel = Skeleton::create("skel");
  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();

  pair.second->createShapeNodeWith<CollisionAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d::Constant(1.0)));
  pair.second->createShapeNodeWith<CollisionAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.5)));

  auto detector = DARTCollisionDetector::create();
  auto group = detector->createCollisionGroup();
  group->addShapeFrame(pair.second->getShapeNode(0));
  group->addShapeFrame(pair.second->getShapeNode(1));

  auto filter = std::make_shared<BodyNodeCollisionFilter>();
  CollisionOption option;
  option.collisionFilter = filter;
  CollisionResult result;

  EXPECT_FALSE(group->collide(option, &result));
}

TEST(BodyNodeCollisionFilter, AddDuplicatePairToBlacklist)
{
  auto skel1 = createSkeleton("skel1");
  auto skel2 = createSkeleton("skel2");
  auto body1 = skel1->getBodyNode(0);
  auto body2 = skel2->getBodyNode(0);

  skel1->getJoint(0)->setPosition(0, 0.0);
  skel2->getJoint(0)->setPosition(0, 0.5);

  auto detector = DARTCollisionDetector::create();
  auto group = detector->createCollisionGroup();
  group->addShapeFrame(body1->getShapeNode(0));
  group->addShapeFrame(body2->getShapeNode(0));

  auto filter = std::make_shared<BodyNodeCollisionFilter>();

  filter->addBodyNodePairToBlackList(body1, body2);
  filter->addBodyNodePairToBlackList(body1, body2);
  filter->addBodyNodePairToBlackList(body2, body1);

  CollisionOption option;
  option.collisionFilter = filter;
  CollisionResult result;

  EXPECT_FALSE(group->collide(option, &result));

  filter->removeBodyNodePairFromBlackList(body1, body2);

  result.clear();
  EXPECT_TRUE(group->collide(option, &result));
}

TEST(BodyNodeCollisionFilter, RemoveNonExistentPairFromBlacklist)
{
  auto skel1 = createSkeleton("skel1");
  auto skel2 = createSkeleton("skel2");
  auto skel3 = createSkeleton("skel3");
  auto body1 = skel1->getBodyNode(0);
  auto body2 = skel2->getBodyNode(0);
  auto body3 = skel3->getBodyNode(0);

  skel1->getJoint(0)->setPosition(0, 0.0);
  skel2->getJoint(0)->setPosition(0, 0.5);

  auto detector = DARTCollisionDetector::create();
  auto group = detector->createCollisionGroup();
  group->addShapeFrame(body1->getShapeNode(0));
  group->addShapeFrame(body2->getShapeNode(0));

  auto filter = std::make_shared<BodyNodeCollisionFilter>();

  filter->addBodyNodePairToBlackList(body1, body2);

  CollisionOption option;
  option.collisionFilter = filter;
  CollisionResult result;

  EXPECT_FALSE(group->collide(option, &result));

  filter->removeBodyNodePairFromBlackList(body1, body3);
  filter->removeBodyNodePairFromBlackList(body2, body3);

  result.clear();
  EXPECT_FALSE(group->collide(option, &result));

  filter->removeBodyNodePairFromBlackList(body1, body2);
  result.clear();
  EXPECT_TRUE(group->collide(option, &result));
}

TEST(BodyNodeCollisionFilter, ObjectSelfCollision)
{
  auto skel = createSkeleton("skel");
  auto body = skel->getBodyNode(0);

  auto detector = DARTCollisionDetector::create();
  auto group = detector->createCollisionGroup();
  group->addShapeFrame(body->getShapeNode(0));

  auto filter = std::make_shared<BodyNodeCollisionFilter>();
  CollisionOption option;
  option.collisionFilter = filter;
  CollisionResult result;

  EXPECT_FALSE(group->collide(option, &result));
}

TEST(BodyNodeCollisionFilter, SameCollisionObjectPointerIsIgnored)
{
  auto detector = DARTCollisionDetector::create();
  auto frame = std::make_shared<SimpleFrame>(Frame::World(), "frame");
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));

  StubCollisionObject object(detector.get(), frame.get());
  BodyNodeCollisionFilter filter;

  EXPECT_TRUE(filter.ignoresCollision(&object, &object));
}

TEST(BodyNodeCollisionFilter, NullShapeFrameReturnsFalse)
{
  auto detector = DARTCollisionDetector::create();
  auto frameA = std::make_shared<SimpleFrame>(Frame::World(), "frameA");
  auto frameB = std::make_shared<SimpleFrame>(Frame::World(), "frameB");
  frameA->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  frameB->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));

  StubCollisionObject objectA(detector.get(), frameA.get());
  StubCollisionObject objectB(detector.get(), frameB.get());
  objectA.setShapeFrame(nullptr);

  BodyNodeCollisionFilter filter;
  EXPECT_FALSE(filter.ignoresCollision(&objectA, &objectB));
}

TEST(BodyNodeCollisionFilter, NonShapeNodeFramesReturnFalse)
{
  auto detector = DARTCollisionDetector::create();
  auto frameA = std::make_shared<SimpleFrame>(Frame::World(), "frameA");
  auto frameB = std::make_shared<SimpleFrame>(Frame::World(), "frameB");
  frameA->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  frameB->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));

  StubCollisionObject objectA(detector.get(), frameA.get());
  StubCollisionObject objectB(detector.get(), frameB.get());

  BodyNodeCollisionFilter filter;
  EXPECT_FALSE(filter.ignoresCollision(&objectA, &objectB));
}
