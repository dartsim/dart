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

#include "../../helpers/GTestUtils.hpp"
#include "dart/collision/CollisionGroup.hpp"
#include "dart/collision/CollisionOption.hpp"
#include "dart/collision/CollisionResult.hpp"
#include "dart/collision/dart/DARTCollisionDetector.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Shape.hpp"
#include "dart/dynamics/ShapeNode.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SphereShape.hpp"

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
// Regression test for issue #2470: DARTCollisionDetector sphere-sphere
// collision contact memory corruption. Accessing Contact.point with isApprox()
// should not cause a segfault.
// https://github.com/dartsim/dart/issues/2470
TEST(CollisionResultTests, SphereSphereContactPointAccessible)
{
  auto detector = DARTCollisionDetector::create();

  // Create two sphere skeletons
  auto skeleton1 = Skeleton::create("sphere1");
  skeleton1->createJointAndBodyNodePair<FreeJoint>();
  auto* body1 = skeleton1->getBodyNode(0);
  body1->createShapeNodeWith<CollisionAspect>(
      std::make_shared<SphereShape>(1.0));

  auto skeleton2 = Skeleton::create("sphere2");
  skeleton2->createJointAndBodyNodePair<FreeJoint>();
  auto* body2 = skeleton2->getBodyNode(0);
  body2->createShapeNodeWith<CollisionAspect>(
      std::make_shared<SphereShape>(0.5));

  // Create collision group
  auto group = detector->createCollisionGroup();
  group->addShapeFrame(body1->getShapeNode(0));
  group->addShapeFrame(body2->getShapeNode(0));

  CollisionOption option;
  option.enableContact = true;
  CollisionResult result;

  // Test point contact scenario (spheres nearly touching)
  constexpr double tol = 1e-12;
  // Sphere 1 at origin (radius 1.0)
  // Sphere 2 at x = 1.5 - tol (radius 0.5)
  // Sum of radii = 1.5, so spheres are just barely overlapping
  skeleton1->getJoint(0)->setPositions(Eigen::Vector6d::Zero());
  Eigen::Vector6d pos2 = Eigen::Vector6d::Zero();
  pos2[3] = 1.5 - tol; // translation x
  skeleton2->getJoint(0)->setPositions(pos2);

  result.clear();
  bool collided = group->collide(option, &result);

  if (result.getNumContacts() > 0) {
    const auto& contact = result.getContact(0);

    // This operation previously crashed with SIGSEGV due to memory corruption
    // Accessing contact.point should not crash
    EXPECT_TRUE(contact.point.allFinite());

    // Verify contact.point values are accessible via isApprox
    // The expected contact point is approximately at x=1.0 (on sphere1's
    // surface)
    Eigen::Vector3d expectedPoint = Eigen::Vector3d::UnitX();
    bool pointAccessible = contact.point.isApprox(expectedPoint, 0.1);
    // We don't assert exact position, just that accessing it doesn't crash
    (void)pointAccessible;

    // Also verify normal is accessible
    EXPECT_TRUE(contact.normal.allFinite());
  }

  // Test overlapping scenario (spheres interpenetrating)
  pos2[3] = 1.0; // Spheres overlap by 0.5 units
  skeleton2->getJoint(0)->setPositions(pos2);

  result.clear();
  collided = group->collide(option, &result);
  EXPECT_TRUE(collided);
  ASSERT_GT(result.getNumContacts(), 0u);

  {
    const auto& contact = result.getContact(0);
    EXPECT_TRUE(contact.point.allFinite());
    EXPECT_TRUE(contact.normal.allFinite());
    EXPECT_GT(contact.penetrationDepth, 0.0);
  }
}
