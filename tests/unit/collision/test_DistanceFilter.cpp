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
#include "dart/collision/collision_object.hpp"
#include "dart/collision/dart/dart_collision_detector.hpp"
#include "dart/collision/distance_filter.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/box_shape.hpp"
#include "dart/dynamics/free_joint.hpp"
#include "dart/dynamics/revolute_joint.hpp"
#include "dart/dynamics/shape_node.hpp"
#include "dart/dynamics/skeleton.hpp"

#include <gtest/gtest.h>

#include <array>
#include <memory>

using namespace dart;
using namespace dart::collision;
using namespace dart::dynamics;

namespace {

class DummyCollisionObject : public CollisionObject
{
public:
  DummyCollisionObject(CollisionDetector* detector, const ShapeFrame* frame)
    : CollisionObject(detector, frame)
  {
  }

  void updateEngineData() override
  {
    // Nothing to synchronize for the dummy collision object.
  }
};

struct ChainBodies
{
  std::shared_ptr<Skeleton> skeleton;
  BodyNode* root{nullptr};
  BodyNode* child{nullptr};
  BodyNode* grandchild{nullptr};
  ShapeNode* rootShape{nullptr};
  ShapeNode* childShape{nullptr};
  ShapeNode* grandchildShape{nullptr};
};

ChainBodies makeThreeBodyChain(const std::string& name)
{
  ChainBodies chain;
  chain.skeleton = Skeleton::create(name);

  auto [rootJoint, rootBody]
      = chain.skeleton->createJointAndBodyNodePair<FreeJoint>();
  chain.root = rootBody;
  chain.rootShape = chain.root->createShapeNodeWith<CollisionAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));

  RevoluteJoint::Properties revoluteProperties;
  revoluteProperties.mAxis = Eigen::Vector3d::UnitZ();
  auto [childJoint, childBody]
      = chain.skeleton->createJointAndBodyNodePair<RevoluteJoint>(
          chain.root, revoluteProperties, BodyNode::Properties());
  chain.child = childBody;
  chain.childShape = chain.child->createShapeNodeWith<CollisionAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.5)));

  auto [grandJoint, grandBody]
      = chain.skeleton->createJointAndBodyNodePair<RevoluteJoint>(
          chain.child, revoluteProperties, BodyNode::Properties());
  chain.grandchild = grandBody;
  chain.grandchildShape
      = chain.grandchild->createShapeNodeWith<CollisionAspect>(
          std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.25)));

  chain.skeleton->enableSelfCollisionCheck();
  chain.skeleton->enableAdjacentBodyCheck();

  for (auto* body : {chain.root, chain.child, chain.grandchild}) {
    body->setCollidable(true);
  }

  return chain;
}

DummyCollisionObject makeObject(
    const ShapeNode* shapeNode, CollisionDetector* detector)
{
  return DummyCollisionObject(detector, shapeNode);
}

} // namespace

//==============================================================================
TEST(DistanceFilterTests, RejectsIdenticalObjects)
{
  auto chain = makeThreeBodyChain("identical");
  auto detector = DARTCollisionDetector::create();
  auto object = makeObject(chain.rootShape, detector.get());

  BodyNodeDistanceFilter filter;
  EXPECT_FALSE(filter.needDistance(&object, &object));
}

//==============================================================================
TEST(DistanceFilterTests, SkipsNonCollidableBodies)
{
  auto chain = makeThreeBodyChain("non_collidable");
  auto detector = DARTCollisionDetector::create();
  chain.child->setCollidable(false);

  auto rootObj = makeObject(chain.rootShape, detector.get());
  auto childObj = makeObject(chain.childShape, detector.get());

  BodyNodeDistanceFilter filter;
  EXPECT_FALSE(filter.needDistance(&rootObj, &childObj));
}

//==============================================================================
TEST(DistanceFilterTests, RespectsSelfCollisionToggle)
{
  auto chain = makeThreeBodyChain("self_collision");
  auto detector = DARTCollisionDetector::create();
  chain.skeleton->disableSelfCollisionCheck();

  auto rootObj = makeObject(chain.rootShape, detector.get());
  auto childObj = makeObject(chain.childShape, detector.get());

  BodyNodeDistanceFilter filter;
  EXPECT_FALSE(filter.needDistance(&rootObj, &childObj));
}

//==============================================================================
TEST(DistanceFilterTests, AdjacentBodiesSkippedWhenDisabled)
{
  auto chain = makeThreeBodyChain("adjacent_only");
  auto detector = DARTCollisionDetector::create();
  chain.skeleton->enableSelfCollisionCheck();
  chain.skeleton->disableAdjacentBodyCheck();

  auto rootObj = makeObject(chain.rootShape, detector.get());
  auto childObj = makeObject(chain.childShape, detector.get());

  BodyNodeDistanceFilter filter;
  EXPECT_FALSE(filter.needDistance(&rootObj, &childObj));
}

//==============================================================================
TEST(DistanceFilterTests, NonAdjacentBodiesStillMeasured)
{
  auto chain = makeThreeBodyChain("non_adjacent");
  auto detector = DARTCollisionDetector::create();
  chain.skeleton->enableSelfCollisionCheck();
  chain.skeleton->disableAdjacentBodyCheck();

  auto rootObj = makeObject(chain.rootShape, detector.get());
  auto grandchildObj = makeObject(chain.grandchildShape, detector.get());

  BodyNodeDistanceFilter filter;
  EXPECT_TRUE(filter.needDistance(&rootObj, &grandchildObj));
}

//==============================================================================
TEST(DistanceFilterTests, SeparateSkeletonsAlwaysChecked)
{
  auto firstChain = makeThreeBodyChain("first");
  auto secondChain = makeThreeBodyChain("second");

  auto detector = DARTCollisionDetector::create();
  auto firstObj = makeObject(firstChain.rootShape, detector.get());
  auto secondObj = makeObject(secondChain.rootShape, detector.get());

  BodyNodeDistanceFilter filter;
  EXPECT_TRUE(filter.needDistance(&firstObj, &secondObj));
}
