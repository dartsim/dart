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
#include "dart/collision/collision_filter.hpp"
#include "dart/collision/collision_group.hpp"
#include "dart/collision/collision_option.hpp"
#include "dart/collision/collision_result.hpp"
#include "dart/collision/dart/dart_collision_detector.hpp"
#include "dart/collision/distance_option.hpp"
#include "dart/collision/distance_result.hpp"
#include "dart/collision/fcl/fcl_collision_detector.hpp"
#if DART_HAVE_BULLET
  #include "dart/collision/bullet/bullet_collision_detector.hpp"
#endif
#include "dart/collision/raycast_option.hpp"
#include "dart/collision/raycast_result.hpp"
#include "dart/config.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/box_shape.hpp"
#include "dart/dynamics/free_joint.hpp"
#include "dart/dynamics/shape.hpp"
#include "dart/dynamics/simple_frame.hpp"
#include "dart/dynamics/skeleton.hpp"

using namespace dart;
using namespace dart::collision;
using namespace dart::dynamics;

namespace {

std::shared_ptr<SimpleFrame> createBoxFrame(
    const std::string& name,
    const Eigen::Vector3d& translation = Eigen::Vector3d::Zero())
{
  auto frame = std::make_shared<SimpleFrame>(Frame::World(), name);
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  frame->setTranslation(translation);
  return frame;
}

class CountingFilter : public CollisionFilter
{
public:
  bool ignoresCollision(
      const CollisionObject*, const CollisionObject*) const override
  {
    ++mCallCount;
    return mReject;
  }

  mutable int mCallCount = 0;
  bool mReject = false;
};

std::unique_ptr<CollisionGroup> makeOverlappingGroup(
    std::shared_ptr<SimpleFrame>& outA, std::shared_ptr<SimpleFrame>& outB)
{
  auto detector = dart::collision::DARTCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  outA = createBoxFrame("boxA");
  outB = createBoxFrame("boxB", Eigen::Vector3d(0.25, 0, 0));

  group->addShapeFrame(outA.get());
  group->addShapeFrame(outB.get());

  return group;
}

} // namespace

//==============================================================================
TEST(CollisionGroupTests, CollidablesCanBeToggled)
{
  std::shared_ptr<SimpleFrame> frameA;
  std::shared_ptr<SimpleFrame> frameB;
  auto group = makeOverlappingGroup(frameA, frameB);

  CollisionResult result;
  CollisionOption option;

  ASSERT_TRUE(group->collide(option, &result));
  EXPECT_LT(0u, result.getNumContacts());

  group->removeShapeFrame(frameA.get());
  result.clear();
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(0u, result.getNumContacts());

  group->addShapeFrame(frameA.get());
  result.clear();
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_LT(0u, result.getNumContacts());
}

//==============================================================================
TEST(CollisionGroupTests, CollisionFilterAndContactLimitAreApplied)
{
  std::shared_ptr<SimpleFrame> frameA;
  std::shared_ptr<SimpleFrame> frameB;
  auto group = makeOverlappingGroup(frameA, frameB);

  auto filter = std::make_shared<CountingFilter>();
  CollisionOption option;
  option.collisionFilter = filter;

  CollisionResult result;
  filter->mReject = true;
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(1, filter->mCallCount);
  EXPECT_EQ(0u, result.getNumContacts());

  filter->mReject = false;
  option.maxNumContacts = 1u;
  result.clear();
  filter->mCallCount = 0;

  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_EQ(1u, result.getNumContacts());
  EXPECT_EQ(1, filter->mCallCount);

  // When we raise the limit we should collect more than one contact for the
  // overlapping cubes.
  option.maxNumContacts = 8u;
  result.clear();
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_LT(1u, result.getNumContacts());
}

//==============================================================================
TEST(CollisionGroupTests, GetCollisionDetector)
{
  auto detector = dart::collision::DARTCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  // Non-const version
  EXPECT_EQ(group->getCollisionDetector(), detector);

  // Const version
  const auto* constGroup = group.get();
  EXPECT_EQ(constGroup->getCollisionDetector(), detector);
}

//==============================================================================
TEST(CollisionGroupTests, ShapeFrameManagement)
{
  auto detector = dart::collision::DARTCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  auto frame1 = createBoxFrame("box1");
  auto frame2 = createBoxFrame("box2", Eigen::Vector3d(2, 0, 0));
  auto frame3 = createBoxFrame("box3", Eigen::Vector3d(4, 0, 0));

  // Initially empty
  EXPECT_EQ(group->getNumShapeFrames(), 0u);
  EXPECT_FALSE(group->hasShapeFrame(frame1.get()));

  // Add single frame
  group->addShapeFrame(frame1.get());
  EXPECT_EQ(group->getNumShapeFrames(), 1u);
  EXPECT_TRUE(group->hasShapeFrame(frame1.get()));
  EXPECT_EQ(group->getShapeFrame(0), frame1.get());

  // Add multiple frames
  std::vector<const dynamics::ShapeFrame*> frames
      = {frame2.get(), frame3.get()};
  group->addShapeFrames(frames);
  EXPECT_EQ(group->getNumShapeFrames(), 3u);
  EXPECT_TRUE(group->hasShapeFrame(frame2.get()));
  EXPECT_TRUE(group->hasShapeFrame(frame3.get()));

  // Remove single frame
  group->removeShapeFrame(frame1.get());
  EXPECT_EQ(group->getNumShapeFrames(), 2u);
  EXPECT_FALSE(group->hasShapeFrame(frame1.get()));

  // Remove multiple frames
  group->removeShapeFrames(frames);
  EXPECT_EQ(group->getNumShapeFrames(), 0u);

  // Add all back and remove all
  group->addShapeFrame(frame1.get());
  group->addShapeFrames(frames);
  EXPECT_EQ(group->getNumShapeFrames(), 3u);

  group->removeAllShapeFrames();
  EXPECT_EQ(group->getNumShapeFrames(), 0u);
}

//==============================================================================
TEST(CollisionGroupTests, AddShapeFramesOf)
{
  auto detector = dart::collision::DARTCollisionDetector::create();
  auto group1 = detector->createCollisionGroup();
  auto group2 = detector->createCollisionGroup();

  auto frame1 = createBoxFrame("box1");
  auto frame2 = createBoxFrame("box2", Eigen::Vector3d(2, 0, 0));

  group1->addShapeFrame(frame1.get());
  group1->addShapeFrame(frame2.get());

  // Add frames from another group
  group2->addShapeFramesOf(group1.get());
  EXPECT_EQ(group2->getNumShapeFrames(), 2u);
  EXPECT_TRUE(group2->hasShapeFrame(frame1.get()));
  EXPECT_TRUE(group2->hasShapeFrame(frame2.get()));

  // Add frames of nothing (terminating case)
  group2->addShapeFramesOf(); // Should not crash
}

//==============================================================================
TEST(CollisionGroupTests, RemoveShapeFramesOf)
{
  auto detector = dart::collision::DARTCollisionDetector::create();
  auto group1 = detector->createCollisionGroup();
  auto group2 = detector->createCollisionGroup();

  auto frame1 = createBoxFrame("box1");
  auto frame2 = createBoxFrame("box2", Eigen::Vector3d(2, 0, 0));
  auto frame3 = createBoxFrame("box3", Eigen::Vector3d(4, 0, 0));

  // Set up group1 with frame1 and frame2
  group1->addShapeFrame(frame1.get());
  group1->addShapeFrame(frame2.get());

  // Set up group2 with all three frames
  group2->addShapeFrame(frame1.get());
  group2->addShapeFrame(frame2.get());
  group2->addShapeFrame(frame3.get());
  EXPECT_EQ(group2->getNumShapeFrames(), 3u);

  // Remove frames of group1 from group2
  group2->removeShapeFramesOf(group1.get());
  EXPECT_EQ(group2->getNumShapeFrames(), 1u);
  EXPECT_FALSE(group2->hasShapeFrame(frame1.get()));
  EXPECT_FALSE(group2->hasShapeFrame(frame2.get()));
  EXPECT_TRUE(group2->hasShapeFrame(frame3.get()));

  // Remove frames of nothing (terminating case)
  group2->removeShapeFramesOf(); // Should not crash
}

//==============================================================================
TEST(CollisionGroupTests, AutomaticUpdate)
{
  auto detector = dart::collision::DARTCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  // Check default is automatic
  EXPECT_TRUE(group->getAutomaticUpdate());

  // Turn off automatic update
  group->setAutomaticUpdate(false);
  EXPECT_FALSE(group->getAutomaticUpdate());

  // Turn on automatic update
  group->setAutomaticUpdate(true);
  EXPECT_TRUE(group->getAutomaticUpdate());

  // Manual update should not crash
  group->update();
}

//==============================================================================
TEST(CollisionGroupTests, CollisionBetweenGroups)
{
  auto detector = dart::collision::DARTCollisionDetector::create();
  auto group1 = detector->createCollisionGroup();
  auto group2 = detector->createCollisionGroup();

  // Non-overlapping boxes
  auto frame1 = createBoxFrame("box1", Eigen::Vector3d(0, 0, 0));
  auto frame2 = createBoxFrame("box2", Eigen::Vector3d(5, 0, 0));

  group1->addShapeFrame(frame1.get());
  group2->addShapeFrame(frame2.get());

  CollisionResult result;
  CollisionOption option;

  // No collision between separated boxes
  EXPECT_FALSE(group1->collide(group2.get(), option, &result));
  EXPECT_EQ(0u, result.getNumContacts());

  // Move box2 to overlap with box1
  frame2->setTranslation(Eigen::Vector3d(0.5, 0, 0));

  result.clear();
  EXPECT_TRUE(group1->collide(group2.get(), option, &result));
  EXPECT_GT(result.getNumContacts(), 0u);
}

//==============================================================================
TEST(CollisionGroupTests, DistanceQuery)
{
  auto detector = dart::collision::FCLCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  auto frame1 = createBoxFrame("box1", Eigen::Vector3d(0, 0, 0));
  auto frame2 = createBoxFrame("box2", Eigen::Vector3d(3, 0, 0));

  group->addShapeFrame(frame1.get());
  group->addShapeFrame(frame2.get());

  DistanceOption distOption;
  DistanceResult distResult;

  double distance = group->distance(distOption, &distResult);

  EXPECT_GT(distance, 0.0);
  EXPECT_NEAR(distance, 2.0, 0.1);
}

//==============================================================================
TEST(CollisionGroupTests, DistanceBetweenGroups)
{
  auto detector = dart::collision::FCLCollisionDetector::create();
  auto group1 = detector->createCollisionGroup();
  auto group2 = detector->createCollisionGroup();

  auto frame1 = createBoxFrame("box1", Eigen::Vector3d(0, 0, 0));
  auto frame2 = createBoxFrame("box2", Eigen::Vector3d(4, 0, 0));

  group1->addShapeFrame(frame1.get());
  group2->addShapeFrame(frame2.get());

  DistanceOption distOption;
  DistanceResult distResult;

  double distance = group1->distance(group2.get(), distOption, &distResult);

  EXPECT_GT(distance, 0.0);
  EXPECT_NEAR(distance, 3.0, 0.1);
}

//==============================================================================
#if DART_HAVE_BULLET
TEST(CollisionGroupTests, RaycastBasic)
{
  auto detector = dart::collision::BulletCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  auto frame = createBoxFrame("box", Eigen::Vector3d(0, 0, 0));
  group->addShapeFrame(frame.get());

  RaycastOption rayOption;
  RaycastResult rayResult;

  Eigen::Vector3d from(-5, 0, 0);
  Eigen::Vector3d to(5, 0, 0);

  bool hit = group->raycast(from, to, rayOption, &rayResult);
  EXPECT_TRUE(hit);

  from = Eigen::Vector3d(-5, 10, 0);
  to = Eigen::Vector3d(5, 10, 0);

  rayResult = RaycastResult();
  hit = group->raycast(from, to, rayOption, &rayResult);
  EXPECT_FALSE(hit);
}
#endif

//==============================================================================
TEST(CollisionGroupTests, RemoveDeletedShapeFrames)
{
  auto detector = dart::collision::DARTCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  {
    auto frame = createBoxFrame("temporary_box");
    group->addShapeFrame(frame.get());
    EXPECT_EQ(group->getNumShapeFrames(), 1u);
    // frame goes out of scope here
  }

  // After the frame is deleted, removeDeletedShapeFrames should clean up
  group->removeDeletedShapeFrames();
  EXPECT_EQ(group->getNumShapeFrames(), 0u);
}

//==============================================================================
TEST(CollisionGroupTests, EmptyGroupOperations)
{
  auto detector = dart::collision::DARTCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  CollisionResult collResult;
  CollisionOption collOption;
  EXPECT_FALSE(group->collide(collOption, &collResult));

  group->update();
  group->removeDeletedShapeFrames();
  group->removeAllShapeFrames();
}

//==============================================================================
TEST(CollisionGroupTests, EmptyGroupWithFCL)
{
  auto detector = dart::collision::FCLCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  DistanceOption distOption;
  DistanceResult distResult;
  double distance = group->distance(distOption, &distResult);
  (void)distance;

  RaycastOption rayOption;
  RaycastResult rayResult;
  Eigen::Vector3d from(-5, 0, 0);
  Eigen::Vector3d to(5, 0, 0);
  bool hit = group->raycast(from, to, rayOption, &rayResult);
  EXPECT_FALSE(hit);
}

//==============================================================================
TEST(CollisionGroupTests, SubscribeToSkeleton)
{
  auto detector = dart::collision::DARTCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  // Create a skeleton with a body that has a collision shape
  auto skel = Skeleton::create("test_skel");
  auto pair = skel->createJointAndBodyNodePair<dynamics::FreeJoint>(
      nullptr,
      dynamics::FreeJoint::Properties(),
      dynamics::BodyNode::AspectProperties("body"));
  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d::Ones());
  pair.second->createShapeNodeWith<dynamics::CollisionAspect>(shape);

  // Subscribe to the skeleton
  group->subscribeTo(dynamics::ConstMetaSkeletonPtr(skel));

  EXPECT_TRUE(group->isSubscribedTo(skel.get()));
  // After update, the shape frames from the skeleton should be in the group
  group->update();
  EXPECT_GE(group->getNumShapeFrames(), 1u);

  // Unsubscribe
  group->unsubscribeFrom(skel.get());
  EXPECT_FALSE(group->isSubscribedTo(skel.get()));
}

//==============================================================================
TEST(CollisionGroupTests, SubscribeToBodyNode)
{
  auto detector = dart::collision::DARTCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  auto skel = Skeleton::create("bn_skel");
  auto pair = skel->createJointAndBodyNodePair<dynamics::FreeJoint>(
      nullptr,
      dynamics::FreeJoint::Properties(),
      dynamics::BodyNode::AspectProperties("body"));
  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d::Ones());
  pair.second->createShapeNodeWith<dynamics::CollisionAspect>(shape);

  auto bodyNode = pair.second;

  // Subscribe to the body node
  group->subscribeTo(dynamics::ConstBodyNodePtr(bodyNode));

  EXPECT_TRUE(group->isSubscribedTo(bodyNode));
  group->update();
  EXPECT_GE(group->getNumShapeFrames(), 1u);

  // Unsubscribe
  group->unsubscribeFrom(bodyNode);
  EXPECT_FALSE(group->isSubscribedTo(bodyNode));
}

//==============================================================================
TEST(CollisionGroupTests, AddShapeFrameDuplicateIsNoOp)
{
  auto detector = dart::collision::DARTCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  auto frame = createBoxFrame("dup_box");
  group->addShapeFrame(frame.get());
  EXPECT_EQ(group->getNumShapeFrames(), 1u);

  // Adding the same frame again should not increase the count
  group->addShapeFrame(frame.get());
  EXPECT_EQ(group->getNumShapeFrames(), 1u);
}

//==============================================================================
TEST(CollisionGroupTests, RemoveShapeFrameNullptrNoCrash)
{
  auto detector = dart::collision::DARTCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  // Removing nullptr should not crash
  group->removeShapeFrame(nullptr);
  EXPECT_EQ(group->getNumShapeFrames(), 0u);
}

//==============================================================================
TEST(CollisionGroupTests, RemoveShapeFrameNotInGroup)
{
  auto detector = dart::collision::DARTCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  auto frame = createBoxFrame("not_in_group");

  // Removing a frame that was never added should not crash
  group->removeShapeFrame(frame.get());
  EXPECT_EQ(group->getNumShapeFrames(), 0u);
}

//==============================================================================
TEST(CollisionGroupTests, ManualUpdateWithAutomaticOff)
{
  auto detector = dart::collision::DARTCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  auto frameA = createBoxFrame("manA");
  auto frameB = createBoxFrame("manB", Eigen::Vector3d(0.25, 0, 0));

  group->addShapeFrame(frameA.get());
  group->addShapeFrame(frameB.get());

  // Turn off automatic update
  group->setAutomaticUpdate(false);
  EXPECT_FALSE(group->getAutomaticUpdate());

  // Manually call update, then collide
  group->update();

  CollisionResult result;
  CollisionOption option;
  // Even with automatic off, collide should still work (it just won't
  // auto-update)
  bool collides = group->collide(option, &result);
  EXPECT_TRUE(collides);
}

//==============================================================================
TEST(CollisionGroupTests, AddShapeFramesOfSkeleton)
{
  auto detector = dart::collision::DARTCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  auto skel = Skeleton::create("skel_frames");
  auto pair = skel->createJointAndBodyNodePair<dynamics::FreeJoint>(
      nullptr,
      dynamics::FreeJoint::Properties(),
      dynamics::BodyNode::AspectProperties("body"));
  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d::Ones());
  pair.second->createShapeNodeWith<dynamics::CollisionAspect>(shape);

  // addShapeFramesOf with a MetaSkeleton should add its shape frames
  group->addShapeFramesOf(
      static_cast<const dynamics::MetaSkeleton*>(skel.get()));
  EXPECT_GE(group->getNumShapeFrames(), 1u);
}

//==============================================================================
TEST(CollisionGroupTests, AddShapeFramesOfBodyNode)
{
  auto detector = dart::collision::DARTCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  auto skel = Skeleton::create("bn_frames");
  auto pair = skel->createJointAndBodyNodePair<dynamics::FreeJoint>(
      nullptr,
      dynamics::FreeJoint::Properties(),
      dynamics::BodyNode::AspectProperties("body"));
  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d::Ones());
  pair.second->createShapeNodeWith<dynamics::CollisionAspect>(shape);

  group->addShapeFramesOf(static_cast<const dynamics::BodyNode*>(pair.second));
  EXPECT_GE(group->getNumShapeFrames(), 1u);
}

//==============================================================================
TEST(CollisionGroupTests, IsSubscribedToTerminator)
{
  auto detector = dart::collision::DARTCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  // The no-arg terminator should return true
  EXPECT_TRUE(group->isSubscribedTo());
}

//==============================================================================
TEST(CollisionGroupTests, AddShapeFrameNullptrIsNoOp)
{
  auto detector = dart::collision::DARTCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  group->addShapeFrame(nullptr);
  EXPECT_EQ(group->getNumShapeFrames(), 0u);
}
