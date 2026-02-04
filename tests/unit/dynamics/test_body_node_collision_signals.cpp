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

#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/box_shape.hpp"
#include "dart/dynamics/free_joint.hpp"
#include "dart/dynamics/shape_frame.hpp"
#include "dart/dynamics/shape_node.hpp"
#include "dart/dynamics/simple_frame.hpp"
#include "dart/dynamics/skeleton.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <vector>

namespace dart::dynamics {
namespace {

std::pair<SkeletonPtr, BodyNode*> makeBodyNode()
{
  auto skel = Skeleton::create("s");
  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  return {std::move(skel), pair.second};
}

class TestBodyNode final : public BodyNode
{
public:
  TestBodyNode(
      BodyNode* parent, Joint* parentJoint, const Properties& properties)
    : Entity(ConstructFrame),
      Frame(Frame::World()),
      BodyNode(parent, parentJoint, properties)
  {
    // Do nothing
  }

  void callProcessNewEntity(Entity* entity)
  {
    processNewEntity(entity);
  }

  friend class Skeleton;
};

std::pair<SkeletonPtr, TestBodyNode*> makeTestBodyNode()
{
  auto skel = Skeleton::create("s");
  auto pair = skel->createJointAndBodyNodePair<FreeJoint, TestBodyNode>();
  return {std::move(skel), static_cast<TestBodyNode*>(pair.second)};
}

TEST(BodyNodeCollisionSignals, FiresOnCreateAndRemove)
{
  auto [skel, bn] = makeBodyNode();

  int added = 0;
  int removed = 0;
  ConstShapePtr lastAdded;
  ConstShapePtr lastRemoved;

  auto addConnection = bn->onColShapeAdded.connect(
      [&](const BodyNode* body, ConstShapePtr shape) {
        ++added;
        EXPECT_EQ(body, bn);
        lastAdded = shape;
      });

  auto removeConnection = bn->onColShapeRemoved.connect(
      [&](const BodyNode* body, ConstShapePtr shape) {
        ++removed;
        EXPECT_EQ(body, bn);
        lastRemoved = shape;
      });
  (void)addConnection;
  (void)removeConnection;

  auto* shapeNode = bn->createShapeNodeWith<CollisionAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.1)));

  EXPECT_EQ(added, 1);
  EXPECT_EQ(removed, 0);
  EXPECT_EQ(lastAdded, shapeNode->getShape());

  shapeNode->remove();

  EXPECT_EQ(added, 1);
  EXPECT_EQ(removed, 1);
  EXPECT_EQ(lastRemoved, lastAdded);
}

TEST(BodyNodeCollisionSignals, TracksNonBodyNodeEntities)
{
  auto [skel, bn] = makeTestBodyNode();

  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.1));
  auto* shapeNode = bn->createShapeNodeWith<VisualAspect>(shape);
  ASSERT_NE(shapeNode, nullptr);

  auto frame = std::make_shared<SimpleFrame>();
  frame->setParentFrame(bn);

  bn->callProcessNewEntity(frame.get());

  frame->setParentFrame(Frame::World());
}

TEST(BodyNodeCollisionSignals, FiresOnCollidableToggle)
{
  auto [skel, bn] = makeBodyNode();

  int added = 0;
  int removed = 0;

  auto addConnection = bn->onColShapeAdded.connect(
      [&](const BodyNode* body, ConstShapePtr shape) {
        ++added;
        EXPECT_EQ(body, bn);
        EXPECT_TRUE(shape);
      });

  auto removeConnection = bn->onColShapeRemoved.connect(
      [&](const BodyNode* body, ConstShapePtr shape) {
        ++removed;
        EXPECT_EQ(body, bn);
        EXPECT_TRUE(shape);
      });
  (void)addConnection;
  (void)removeConnection;

  auto* shapeNode = bn->createShapeNodeWith<CollisionAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.2)));

  ASSERT_NE(shapeNode->get<CollisionAspect>(), nullptr);

  EXPECT_EQ(added, 1);
  EXPECT_EQ(removed, 0);

  shapeNode->get<CollisionAspect>()->setCollidable(false);
  EXPECT_EQ(added, 1);
  EXPECT_EQ(removed, 1);

  shapeNode->get<CollisionAspect>()->setCollidable(true);
  EXPECT_EQ(added, 2);
  EXPECT_EQ(removed, 1);

  // No-op when the value does not change.
  shapeNode->get<CollisionAspect>()->setCollidable(true);
  EXPECT_EQ(added, 2);
  EXPECT_EQ(removed, 1);
}

TEST(BodyNodeCollisionSignals, FiresOnAspectLifecycle)
{
  auto [skel, bn] = makeBodyNode();

  int added = 0;
  int removed = 0;

  auto addConnection = bn->onColShapeAdded.connect(
      [&](const BodyNode* body, ConstShapePtr shape) {
        ++added;
        EXPECT_EQ(body, bn);
        EXPECT_TRUE(shape);
      });

  auto removeConnection = bn->onColShapeRemoved.connect(
      [&](const BodyNode* body, ConstShapePtr shape) {
        ++removed;
        EXPECT_EQ(body, bn);
        EXPECT_TRUE(shape);
      });
  (void)addConnection;
  (void)removeConnection;

  auto* shapeNode = bn->createShapeNode(
      std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.3)));

  EXPECT_EQ(added, 0);
  EXPECT_EQ(removed, 0);

  shapeNode->createCollisionAspect();
  EXPECT_EQ(added, 1);
  EXPECT_EQ(removed, 0);

  shapeNode->removeAspect<CollisionAspect>();
  EXPECT_EQ(added, 1);
  EXPECT_EQ(removed, 1);
}

TEST(BodyNodeCollisionSignals, FiresOnShapeReplacement)
{
  auto [skel, bn] = makeBodyNode();

  int added = 0;
  int removed = 0;
  std::vector<ConstShapePtr> addedShapes;
  std::vector<ConstShapePtr> removedShapes;

  auto addConnection = bn->onColShapeAdded.connect(
      [&](const BodyNode* body, ConstShapePtr shape) {
        ++added;
        EXPECT_EQ(body, bn);
        addedShapes.push_back(shape);
      });

  auto removeConnection = bn->onColShapeRemoved.connect(
      [&](const BodyNode* body, ConstShapePtr shape) {
        ++removed;
        EXPECT_EQ(body, bn);
        removedShapes.push_back(shape);
      });
  (void)addConnection;
  (void)removeConnection;

  auto firstShape = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.4));
  auto secondShape = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.5));

  auto* shapeNode = bn->createShapeNodeWith<CollisionAspect>(firstShape);

  EXPECT_EQ(added, 1);
  EXPECT_EQ(removed, 0);
  ASSERT_FALSE(addedShapes.empty());
  EXPECT_EQ(addedShapes.back(), firstShape);

  shapeNode->setShape(secondShape);
  EXPECT_EQ(added, 2);
  EXPECT_EQ(removed, 1);
  ASSERT_FALSE(removedShapes.empty());
  EXPECT_EQ(removedShapes.back(), firstShape);
  EXPECT_EQ(addedShapes.back(), secondShape);

  // No-op for redundant set.
  shapeNode->setShape(secondShape);
  EXPECT_EQ(added, 2);
  EXPECT_EQ(removed, 1);

  // Removing the shape should fire only a removal.
  shapeNode->setShape(nullptr);
  EXPECT_EQ(added, 2);
  EXPECT_EQ(removed, 2);
  ASSERT_FALSE(removedShapes.empty());
  EXPECT_EQ(removedShapes.back(), secondShape);

  ASSERT_NE(shapeNode->get<CollisionAspect>(), nullptr);
  shapeNode->get<CollisionAspect>()->setCollidable(false);

  // Reassign while non-collidable should not fire.
  shapeNode->setShape(firstShape);
  EXPECT_EQ(added, 2);
  EXPECT_EQ(removed, 2);

  // Re-enabling collision on an already-assigned shape should add once.
  shapeNode->get<CollisionAspect>()->setCollidable(true);
  EXPECT_EQ(added, 3);
  EXPECT_EQ(removed, 2);
  EXPECT_EQ(addedShapes.back(), firstShape);
}

} // namespace
} // namespace dart::dynamics
