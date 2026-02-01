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

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/end_effector.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/marker.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/math/constants.hpp>

#include <dart/common/diagnostics.hpp>

#include <dart/all.hpp>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <optional>

using namespace dart::dynamics;
using namespace dart::simulation;

static SkeletonPtr createBodyNodeSkeleton()
{
  auto skeleton = Skeleton::create("body_props");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setName("body");
  return skeleton;
}

TEST(BodyNodeProperties, SetCompositeProperties)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  const auto properties = body->getCompositeProperties();
  body->setProperties(properties);

  EXPECT_EQ(body->getSkeleton().get(), skeleton.get());
}

TEST(BodyNodeProperties, TransformDerivativesCache)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  skeleton->setPositions(
      Eigen::VectorXd::Constant(skeleton->getNumDofs(), 0.1));

  const Eigen::Matrix4d first = body->getWorldTransformDerivative(0);
  const Eigen::Matrix4d second = body->getWorldTransformDerivative(0);
  EXPECT_TRUE(first.isApprox(second));
}

TEST(BodyNodeProperties, MassAndInertia)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  body->setMass(2.5);
  EXPECT_DOUBLE_EQ(body->getMass(), 2.5);
  EXPECT_DOUBLE_EQ(body->getInertia().getMass(), 2.5);

  body->setMomentOfInertia(1.1, 2.2, 3.3, 0.1, 0.2, 0.3);
  double ixx = 0.0;
  double iyy = 0.0;
  double izz = 0.0;
  double ixy = 0.0;
  double ixz = 0.0;
  double iyz = 0.0;
  body->getMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
  EXPECT_DOUBLE_EQ(ixx, 1.1);
  EXPECT_DOUBLE_EQ(iyy, 2.2);
  EXPECT_DOUBLE_EQ(izz, 3.3);
  EXPECT_DOUBLE_EQ(ixy, 0.1);
  EXPECT_DOUBLE_EQ(ixz, 0.2);
  EXPECT_DOUBLE_EQ(iyz, 0.3);

  const Eigen::Vector3d localCom(0.1, -0.2, 0.3);
  body->setLocalCOM(localCom);
  EXPECT_TRUE(body->getLocalCOM().isApprox(localCom));
}

TEST(BodyNodeProperties, CollisionAndGravityModes)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  EXPECT_TRUE(body->getGravityMode());
  EXPECT_TRUE(body->isCollidable());

  body->setGravityMode(false);
  body->setCollidable(false);
  EXPECT_FALSE(body->getGravityMode());
  EXPECT_FALSE(body->isCollidable());

  body->setGravityMode(true);
  body->setCollidable(true);
  EXPECT_TRUE(body->getGravityMode());
  EXPECT_TRUE(body->isCollidable());
}

TEST(BodyNodeProperties, ComputeInertiaFromShapeNodes)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 2.0, 3.0));
  auto sphere = std::make_shared<SphereShape>(0.5);
  body->createShapeNodeWith<VisualAspect>(box);
  body->createShapeNodeWith<VisualAspect>(sphere);

  const auto unused = body->computeInertiaFromShapeNodes(
      [](const ShapeNode*) -> std::optional<double> { return std::nullopt; });
  EXPECT_FALSE(unused.has_value());

  const auto inertia = body->computeInertiaFromShapeNodes(
      [&](const ShapeNode* shapeNode) -> std::optional<double> {
        if (shapeNode->getShape().get() == box.get()) {
          return 2.0;
        }
        if (shapeNode->getShape().get() == sphere.get()) {
          return 1.0;
        }
        return std::nullopt;
      });
  ASSERT_TRUE(inertia.has_value());
  EXPECT_DOUBLE_EQ(inertia->getMass(), 3.0);
}

TEST(BodyNodeProperties, CreateShapeNodeWithVisualAspect)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  EXPECT_EQ(body->getNumShapeNodes(), 0u);

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 2.0, 3.0));
  ShapeNode* shapeNode = body->createShapeNodeWith<VisualAspect>(box);

  ASSERT_NE(shapeNode, nullptr);
  EXPECT_EQ(body->getNumShapeNodes(), 1u);
  EXPECT_TRUE(shapeNode->has<VisualAspect>());
  EXPECT_FALSE(shapeNode->has<CollisionAspect>());
  EXPECT_EQ(shapeNode->getShape().get(), box.get());
}

TEST(BodyNodeProperties, CreateShapeNodeWithCollisionAspect)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  auto sphere = std::make_shared<SphereShape>(0.5);
  ShapeNode* shapeNode = body->createShapeNodeWith<CollisionAspect>(sphere);

  ASSERT_NE(shapeNode, nullptr);
  EXPECT_EQ(body->getNumShapeNodes(), 1u);
  EXPECT_TRUE(shapeNode->has<CollisionAspect>());
  EXPECT_FALSE(shapeNode->has<VisualAspect>());
  EXPECT_EQ(shapeNode->getShape().get(), sphere.get());
}

TEST(BodyNodeProperties, CreateShapeNodeWithMultipleAspects)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  auto cylinder = std::make_shared<CylinderShape>(0.5, 1.0);
  ShapeNode* shapeNode
      = body->createShapeNodeWith<VisualAspect, CollisionAspect>(cylinder);

  ASSERT_NE(shapeNode, nullptr);
  EXPECT_EQ(body->getNumShapeNodes(), 1u);
  EXPECT_TRUE(shapeNode->has<VisualAspect>());
  EXPECT_TRUE(shapeNode->has<CollisionAspect>());
}

TEST(BodyNodeProperties, ShapeNodeAccessors)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  auto sphere = std::make_shared<SphereShape>(0.5);
  auto cylinder = std::make_shared<CylinderShape>(0.3, 0.8);

  body->createShapeNodeWith<VisualAspect>(box);
  body->createShapeNodeWith<CollisionAspect>(sphere);
  body->createShapeNodeWith<VisualAspect, CollisionAspect>(cylinder);

  EXPECT_EQ(body->getNumShapeNodes(), 3u);

  // Test getShapeNode by index
  EXPECT_EQ(body->getShapeNode(0)->getShape().get(), box.get());
  EXPECT_EQ(body->getShapeNode(1)->getShape().get(), sphere.get());
  EXPECT_EQ(body->getShapeNode(2)->getShape().get(), cylinder.get());

  // Test getNumShapeNodesWith
  EXPECT_EQ(body->getNumShapeNodesWith<VisualAspect>(), 2u);
  EXPECT_EQ(body->getNumShapeNodesWith<CollisionAspect>(), 2u);
}

TEST(BodyNodeProperties, ConstNodeAndShapeAccessors)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(0.3, 0.4, 0.5));
  body->createShapeNodeWith<VisualAspect>(box);

  const BodyNode& constBody = *body;

  DART_SUPPRESS_DEPRECATED_BEGIN
  const auto shapeNodes = constBody.getShapeNodes();
  DART_SUPPRESS_DEPRECATED_END
  EXPECT_EQ(shapeNodes.size(), 1u);
  EXPECT_EQ(shapeNodes[0]->getShape().get(), box.get());

  const auto nodes = constBody.getNodes();
  EXPECT_GE(nodes.size(), 1u);

  const auto mutableNodes = body->getNodes();
  EXPECT_GE(mutableNodes.size(), 1u);
}

TEST(BodyNodeProperties, ConstSpatialAccessors)
{
  auto skeleton = Skeleton::create("const_body_accessors");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  rootPair.second->setName("root");

  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  childPair.first->setAxis(Eigen::Vector3d::UnitZ());
  childPair.second->setName("child");

  skeleton->setVelocities(Eigen::VectorXd::Ones(skeleton->getNumDofs()) * 0.1);
  skeleton->setAccelerations(
      Eigen::VectorXd::Constant(skeleton->getNumDofs(), 0.2));

  const BodyNode* constRoot = rootPair.second;
  const BodyNode* constChild = childPair.second;

  EXPECT_EQ(constRoot->getSkeleton().get(), skeleton.get());
  EXPECT_EQ(constChild->getParentBodyNode(), rootPair.second);
  EXPECT_EQ(constChild->getParentJoint(), childPair.first);
  EXPECT_EQ(constRoot->getNumChildBodyNodes(), 1u);
  EXPECT_EQ(constRoot->getChildBodyNode(0), childPair.second);

  const auto& relVel = constChild->getRelativeSpatialVelocity();
  EXPECT_TRUE(relVel.array().isFinite().all());
  const auto& relAcc = constChild->getRelativeSpatialAcceleration();
  EXPECT_TRUE(relAcc.array().isFinite().all());
  const auto& primaryAcc = constChild->getPrimaryRelativeAcceleration();
  EXPECT_TRUE(primaryAcc.array().isFinite().all());
  const auto& partialAcc = constChild->getPartialAcceleration();
  EXPECT_TRUE(partialAcc.array().isFinite().all());

  const auto& jacobian = constChild->getJacobian();
  EXPECT_EQ(jacobian.rows(), 6);
  EXPECT_EQ(jacobian.cols(), static_cast<int>(skeleton->getNumDofs()));

  const auto& worldJacobian = constChild->getWorldJacobian();
  EXPECT_EQ(worldJacobian.rows(), 6);
  EXPECT_EQ(worldJacobian.cols(), static_cast<int>(skeleton->getNumDofs()));

  const auto& jacobianSpatialDeriv = constChild->getJacobianSpatialDeriv();
  EXPECT_EQ(jacobianSpatialDeriv.rows(), 6);
  EXPECT_EQ(
      jacobianSpatialDeriv.cols(), static_cast<int>(skeleton->getNumDofs()));

  const auto& jacobianClassicDeriv = constChild->getJacobianClassicDeriv();
  EXPECT_EQ(jacobianClassicDeriv.rows(), 6);
  EXPECT_EQ(
      jacobianClassicDeriv.cols(), static_cast<int>(skeleton->getNumDofs()));

  const auto& bodyVelocityChange = constChild->getBodyVelocityChange();
  EXPECT_TRUE(bodyVelocityChange.array().isFinite().all());
}

TEST(BodyNodeProperties, EachShapeNodeIteration)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  auto sphere = std::make_shared<SphereShape>(0.5);

  body->createShapeNodeWith<VisualAspect>(box);
  body->createShapeNodeWith<VisualAspect>(sphere);

  // Test eachShapeNode with void callback
  std::size_t count = 0;
  body->eachShapeNode([&count](const ShapeNode*) { ++count; });
  EXPECT_EQ(count, 2u);

  // Test eachShapeNodeWith for specific aspect
  count = 0;
  body->eachShapeNodeWith<VisualAspect>(
      [&count](const ShapeNode*) { ++count; });
  EXPECT_EQ(count, 2u);

  // Test early termination with bool callback
  count = 0;
  body->eachShapeNode([&count](const ShapeNode*) -> bool {
    ++count;
    return false; // Stop after first
  });
  EXPECT_EQ(count, 1u);
}

TEST(BodyNodeProperties, RemoveAllShapeNodesWith)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  auto sphere = std::make_shared<SphereShape>(0.5);
  auto cylinder = std::make_shared<CylinderShape>(0.3, 0.8);

  body->createShapeNodeWith<VisualAspect>(box);
  body->createShapeNodeWith<CollisionAspect>(sphere);
  body->createShapeNodeWith<VisualAspect, CollisionAspect>(cylinder);

  EXPECT_EQ(body->getNumShapeNodes(), 3u);
  EXPECT_EQ(body->getNumShapeNodesWith<VisualAspect>(), 2u);
  EXPECT_EQ(body->getNumShapeNodesWith<CollisionAspect>(), 2u);

  body->removeAllShapeNodesWith<VisualAspect>();

  EXPECT_EQ(body->getNumShapeNodesWith<VisualAspect>(), 0u);
  EXPECT_EQ(body->getNumShapeNodes(), 1u);
  EXPECT_EQ(body->getNumShapeNodesWith<CollisionAspect>(), 1u);
}

TEST(BodyNodeProperties, EndEffectorManagement)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  EXPECT_EQ(body->getNumEndEffectors(), 0u);

  EndEffector* ee1 = body->createEndEffector("ee1");
  ASSERT_NE(ee1, nullptr);
  EXPECT_EQ(body->getNumEndEffectors(), 1u);
  EXPECT_EQ(ee1->getName(), "ee1");

  EndEffector* ee2 = body->createEndEffector("ee2");
  ASSERT_NE(ee2, nullptr);
  EXPECT_EQ(body->getNumEndEffectors(), 2u);

  // Test getEndEffector by index
  EXPECT_EQ(body->getEndEffector(0), ee1);
  EXPECT_EQ(body->getEndEffector(1), ee2);

  // Test iteration
  std::size_t count = 0;
  body->eachEndEffector([&count](const EndEffector*) { ++count; });
  EXPECT_EQ(count, 2u);
}

TEST(BodyNodeProperties, MarkerManagement)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  EXPECT_EQ(body->getNumMarkers(), 0u);

  Marker* m1 = body->createMarker(std::string("marker1"));
  ASSERT_NE(m1, nullptr);
  EXPECT_EQ(body->getNumMarkers(), 1u);
  EXPECT_EQ(m1->getName(), "marker1");

  Marker* m2 = body->createMarker(
      "marker2", Eigen::Vector3d(0.1, 0.2, 0.3), Eigen::Vector4d(1, 0, 0, 1));
  ASSERT_NE(m2, nullptr);
  EXPECT_EQ(body->getNumMarkers(), 2u);

  // Test getMarker by index
  EXPECT_EQ(body->getMarker(0), m1);
  EXPECT_EQ(body->getMarker(1), m2);

  // Test iteration
  std::size_t count = 0;
  body->eachMarker([&count](const Marker*) { ++count; });
  EXPECT_EQ(count, 2u);
}

// ============================================================================
// Structural Operations Tests
// ============================================================================

// Helper function to create a multi-body skeleton for structural ops testing
static SkeletonPtr createChainSkeleton(const std::string& name, int numBodies)
{
  auto skeleton = Skeleton::create(name);
  BodyNode* parent = nullptr;

  for (int i = 0; i < numBodies; ++i) {
    BodyNode::Properties bodyProps;
    bodyProps.mName = "body" + std::to_string(i);
    bodyProps.mInertia.setMass(1.0);

    if (i == 0) {
      // First body uses FreeJoint to world
      auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>(
          nullptr, FreeJoint::Properties(), bodyProps);
      parent = pair.second;
    } else {
      // Subsequent bodies use RevoluteJoint
      RevoluteJoint::Properties jointProps;
      jointProps.mName = "joint" + std::to_string(i);
      jointProps.mAxis = Eigen::Vector3d::UnitZ();
      jointProps.mT_ParentBodyToJoint.translation()
          = Eigen::Vector3d(0, 0, 0.5);

      auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>(
          parent, jointProps, bodyProps);
      parent = pair.second;
    }
  }

  return skeleton;
}

TEST(BodyNodeProperties, MoveTransformJacobianAndExternalForce)
{
  auto source = createChainSkeleton("move_chain", 2);
  BodyNode* moving = source->getBodyNode("body1");

  auto target = Skeleton::create("move_target");
  auto targetRoot = target->createJointAndBodyNodePair<FreeJoint>();

  EXPECT_TRUE(moving->moveTo(target, targetRoot.second));
  EXPECT_EQ(moving->getSkeleton(), target);

  auto refFrame = SimpleFrame::createShared(Frame::World(), "move_ref");
  refFrame->setTranslation(Eigen::Vector3d(0.05, 0.0, -0.05));

  const auto worldTf = moving->getTransform(Frame::World());
  const auto refTf = moving->getTransform(refFrame.get());
  EXPECT_TRUE(worldTf.matrix().allFinite());
  EXPECT_TRUE(refTf.matrix().allFinite());

  const auto linearJac = moving->getLinearJacobian();
  const auto angularJac = moving->getAngularJacobian();
  EXPECT_EQ(linearJac.rows(), 3);
  EXPECT_EQ(angularJac.rows(), 3);

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(0.2, 0.2, 0.2));
  ShapeNode* shapeNode = moving->createShapeNodeWith<VisualAspect>(box);
  ASSERT_NE(shapeNode, nullptr);
  EXPECT_EQ(moving->getNumShapeNodes(), 1u);
  shapeNode->remove();
  EXPECT_EQ(moving->getNumShapeNodes(), 0u);

  moving->setExtForce(
      Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d::Zero(), true, true);
  moving->addExtForce(
      Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d::Zero(), true, true);
  target->computeForwardDynamics();

  const auto& bodyForce = moving->getBodyForce();
  EXPECT_TRUE(bodyForce.array().isFinite().all());

  SkeletonPtr removed = moving->remove("removed_move");
  ASSERT_NE(removed, nullptr);
  EXPECT_EQ(removed->getNumBodyNodes(), 1u);
}

TEST(BodyNodeStructural, SplitCreatesNewSkeleton)
{
  auto skeleton = createChainSkeleton("original", 4);
  ASSERT_EQ(skeleton->getNumBodyNodes(), 4u);

  // Split at body2 - should create a new skeleton with body2 and body3
  BodyNode* body2 = skeleton->getBodyNode("body2");
  ASSERT_NE(body2, nullptr);

  SkeletonPtr newSkel = body2->split("split_skeleton");

  // Original skeleton now has only 2 bodies (body0, body1)
  EXPECT_EQ(skeleton->getNumBodyNodes(), 2u);
  EXPECT_NE(skeleton->getBodyNode("body0"), nullptr);
  EXPECT_NE(skeleton->getBodyNode("body1"), nullptr);
  EXPECT_EQ(skeleton->getBodyNode("body2"), nullptr);
  EXPECT_EQ(skeleton->getBodyNode("body3"), nullptr);

  // New skeleton has 2 bodies (body2, body3)
  ASSERT_NE(newSkel, nullptr);
  EXPECT_EQ(newSkel->getName(), "split_skeleton");
  EXPECT_EQ(newSkel->getNumBodyNodes(), 2u);
  EXPECT_NE(newSkel->getBodyNode("body2"), nullptr);
  EXPECT_NE(newSkel->getBodyNode("body3"), nullptr);
}

TEST(BodyNodeStructural, SplitAtRootMovesEverything)
{
  auto skeleton = createChainSkeleton("original", 3);
  ASSERT_EQ(skeleton->getNumBodyNodes(), 3u);

  // Split at root body - should move entire tree
  BodyNode* root = skeleton->getBodyNode(0);
  SkeletonPtr newSkel = root->split("new_root");

  EXPECT_EQ(skeleton->getNumBodyNodes(), 0u);
  ASSERT_NE(newSkel, nullptr);
  EXPECT_EQ(newSkel->getNumBodyNodes(), 3u);
}

TEST(BodyNodeStructural, MoveToWithinSameSkeleton)
{
  auto skeleton = createChainSkeleton("skel", 3);
  // Initial structure: body0 -> body1 -> body2

  BodyNode* body1 = skeleton->getBodyNode("body1");
  BodyNode* body2 = skeleton->getBodyNode("body2");

  // Move body2 to be under body0 (instead of body1)
  BodyNode* body0 = skeleton->getBodyNode("body0");
  bool success = body2->moveTo(body0);

  EXPECT_TRUE(success);
  EXPECT_EQ(skeleton->getNumBodyNodes(), 3u);
  // body2 should now be child of body0
  EXPECT_EQ(body2->getParentBodyNode(), body0);
  // body1 should have no children now
  EXPECT_EQ(body1->getNumChildBodyNodes(), 0u);
}

TEST(BodyNodeStructural, MoveToDifferentSkeleton)
{
  auto skel1 = createChainSkeleton("skel1", 3);
  auto skel2 = Skeleton::create("skel2");

  // Create a root in skel2
  BodyNode::Properties bodyProps;
  bodyProps.mName = "skel2_root";
  auto pair = skel2->createJointAndBodyNodePair<FreeJoint>(
      nullptr, FreeJoint::Properties(), bodyProps);
  BodyNode* skel2Root = pair.second;

  EXPECT_EQ(skel1->getNumBodyNodes(), 3u);
  EXPECT_EQ(skel2->getNumBodyNodes(), 1u);

  // Move body2 (and body3 subtree if any) from skel1 to skel2
  BodyNode* body2 = skel1->getBodyNode("body2");
  bool success = body2->moveTo(skel2, skel2Root);

  EXPECT_TRUE(success);
  EXPECT_EQ(skel1->getNumBodyNodes(), 2u);
  EXPECT_EQ(skel2->getNumBodyNodes(), 2u);

  // body2 should now belong to skel2
  EXPECT_EQ(body2->getSkeleton(), skel2);
  EXPECT_EQ(body2->getParentBodyNode(), skel2Root);
}

TEST(BodyNodeStructural, MoveToAsNewRoot)
{
  auto skel1 = createChainSkeleton("skel1", 2);
  auto skel2 = Skeleton::create("skel2");

  BodyNode* body1 = skel1->getBodyNode("body1");

  // Move body1 to skel2 as root (nullptr parent)
  bool success = body1->moveTo(skel2, nullptr);

  EXPECT_TRUE(success);
  EXPECT_EQ(skel1->getNumBodyNodes(), 1u);
  EXPECT_EQ(skel2->getNumBodyNodes(), 1u);
  EXPECT_EQ(body1->getSkeleton(), skel2);
  EXPECT_EQ(body1->getParentBodyNode(), nullptr);
}

TEST(BodyNodeStructural, CopyToSameSkeleton)
{
  auto skeleton = createChainSkeleton("skel", 2);
  BodyNode* body1 = skeleton->getBodyNode("body1");
  BodyNode* body0 = skeleton->getBodyNode("body0");

  EXPECT_EQ(skeleton->getNumBodyNodes(), 2u);

  // Copy body1 to be another child of body0
  auto [joint, copiedBody] = body1->copyTo(body0, false);

  EXPECT_NE(copiedBody, nullptr);
  EXPECT_NE(copiedBody, body1); // It's a copy, not the same
  EXPECT_EQ(skeleton->getNumBodyNodes(), 3u);
  EXPECT_EQ(copiedBody->getParentBodyNode(), body0);
}

TEST(BodyNodeStructural, CopyToDifferentSkeleton)
{
  auto skel1 = createChainSkeleton("skel1", 2);
  auto skel2 = Skeleton::create("skel2");

  BodyNode::Properties bodyProps;
  bodyProps.mName = "skel2_root";
  auto pair = skel2->createJointAndBodyNodePair<FreeJoint>(
      nullptr, FreeJoint::Properties(), bodyProps);
  BodyNode* skel2Root = pair.second;

  BodyNode* body1 = skel1->getBodyNode("body1");

  // Copy body1 from skel1 to skel2
  auto [joint, copiedBody] = body1->copyTo(skel2, skel2Root, false);

  EXPECT_NE(copiedBody, nullptr);
  EXPECT_EQ(skel1->getNumBodyNodes(), 2u); // Original unchanged
  EXPECT_EQ(skel2->getNumBodyNodes(), 2u); // Has new copy
  EXPECT_EQ(copiedBody->getSkeleton(), skel2);
}

TEST(BodyNodeStructural, CopyToRecursive)
{
  auto skel1 = createChainSkeleton("skel1", 3);
  auto skel2 = Skeleton::create("skel2");

  BodyNode::Properties bodyProps;
  bodyProps.mName = "skel2_root";
  auto pair = skel2->createJointAndBodyNodePair<FreeJoint>(
      nullptr, FreeJoint::Properties(), bodyProps);
  BodyNode* skel2Root = pair.second;

  // Copy body1 recursively (should also copy body2)
  BodyNode* body1 = skel1->getBodyNode("body1");
  auto [joint, copiedBody] = body1->copyTo(skel2, skel2Root, true);

  EXPECT_NE(copiedBody, nullptr);
  EXPECT_EQ(skel1->getNumBodyNodes(), 3u); // Original unchanged
  EXPECT_EQ(skel2->getNumBodyNodes(), 3u); // Has copies of body1 and body2
}

TEST(BodyNodeStructural, CopyAsCreatesNewSkeleton)
{
  auto skeleton = createChainSkeleton("original", 3);
  BodyNode* body1 = skeleton->getBodyNode("body1");

  // Copy body1 to a new skeleton (non-recursive)
  SkeletonPtr newSkel = body1->copyAs("copied_skel", false);

  ASSERT_NE(newSkel, nullptr);
  EXPECT_EQ(newSkel->getName(), "copied_skel");
  EXPECT_EQ(newSkel->getNumBodyNodes(), 1u);
  EXPECT_EQ(skeleton->getNumBodyNodes(), 3u); // Original unchanged
}

TEST(BodyNodeStructural, CopyAsRecursive)
{
  auto skeleton = createChainSkeleton("original", 4);
  BodyNode* body1 = skeleton->getBodyNode("body1");

  // Copy body1 and its subtree to a new skeleton
  SkeletonPtr newSkel = body1->copyAs("copied_skel", true);

  ASSERT_NE(newSkel, nullptr);
  EXPECT_EQ(newSkel->getName(), "copied_skel");
  EXPECT_EQ(newSkel->getNumBodyNodes(), 3u);  // body1, body2, body3
  EXPECT_EQ(skeleton->getNumBodyNodes(), 4u); // Original unchanged
}

TEST(BodyNodeStructural, RemoveIsSameAsSplit)
{
  auto skeleton = createChainSkeleton("original", 3);
  BodyNode* body1 = skeleton->getBodyNode("body1");

  SkeletonPtr removed = body1->remove("removed_skel");

  ASSERT_NE(removed, nullptr);
  EXPECT_EQ(removed->getName(), "removed_skel");
  EXPECT_EQ(removed->getNumBodyNodes(), 2u);
  EXPECT_EQ(skeleton->getNumBodyNodes(), 1u);
}

TEST(BodyNodeStructural, CopiedBodyPreservesProperties)
{
  auto skeleton = createChainSkeleton("original", 2);
  BodyNode* body = skeleton->getBodyNode("body1");

  body->setMass(5.0);
  body->setGravityMode(false);

  SkeletonPtr newSkel = body->copyAs("copy", false);
  BodyNode* copied = newSkel->getBodyNode("body1");

  ASSERT_NE(copied, nullptr);
  EXPECT_DOUBLE_EQ(copied->getMass(), 5.0);
  EXPECT_FALSE(copied->getGravityMode());
}

// ============================================================================
// External Force Tests
// ============================================================================

TEST(BodyNodeExternalForce, AddExtForceLocalForceLocalOffset)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  // Clear any external forces
  body->clearExternalForces();
  Eigen::Vector6d initialExt = body->getExternalForceLocal();
  EXPECT_TRUE(initialExt.isZero());

  // Add force in local frame at local offset
  Eigen::Vector3d force(1.0, 0.0, 0.0);
  Eigen::Vector3d offset(0.0, 0.0, 0.5);
  body->addExtForce(force, offset, true, true);

  Eigen::Vector6d ext = body->getExternalForceLocal();
  // Force is applied at offset, creating a torque
  // torque = offset x force = (0,0,0.5) x (1,0,0) = (0, 0.5, 0)
  EXPECT_NEAR(ext[0], 0.0, 1e-10);
  EXPECT_NEAR(ext[1], 0.5, 1e-10);
  EXPECT_NEAR(ext[2], 0.0, 1e-10);
  EXPECT_NEAR(ext[3], 1.0, 1e-10);
  EXPECT_NEAR(ext[4], 0.0, 1e-10);
  EXPECT_NEAR(ext[5], 0.0, 1e-10);
}

TEST(BodyNodeExternalForce, AddExtForceGlobalForceLocalOffset)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);
  FreeJoint* joint = static_cast<FreeJoint*>(body->getParentJoint());

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear() = Eigen::AngleAxisd(dart::math::half_pi, Eigen::Vector3d::UnitZ())
                    .matrix();
  joint->setTransform(tf);

  body->clearExternalForces();

  Eigen::Vector3d globalForce(1.0, 0.0, 0.0);
  Eigen::Vector3d localOffset(0.0, 0.0, 0.0);
  body->addExtForce(globalForce, localOffset, false, true);

  Eigen::Vector6d ext = body->getExternalForceLocal();
  // Global X transformed to local frame via R^T where R rotates +90deg about Z
  // R^T * (1,0,0) = (0,-1,0) since R^T rotates -90deg about Z
  EXPECT_NEAR(ext[3], 0.0, 1e-10);
  EXPECT_NEAR(ext[4], -1.0, 1e-10);
  EXPECT_NEAR(ext[5], 0.0, 1e-10);
}

TEST(BodyNodeExternalForce, AddExtForceLocalForceGlobalOffset)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);
  FreeJoint* joint = static_cast<FreeJoint*>(body->getParentJoint());

  // Set body translation
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  joint->setTransform(tf);

  body->clearExternalForces();

  // Apply local force at global offset
  Eigen::Vector3d localForce(0.0, 1.0, 0.0);
  Eigen::Vector3d globalOffset(
      1.0, 0.0, 0.5); // Body is at (1,0,0), so local offset is (0,0,0.5)
  body->addExtForce(localForce, globalOffset, true, false);

  Eigen::Vector6d ext = body->getExternalForceLocal();
  // Local offset = globalOffset - bodyPosition = (0,0,0.5)
  // torque = (0,0,0.5) x (0,1,0) = (-0.5,0,0)
  EXPECT_NEAR(ext[0], -0.5, 1e-10);
  EXPECT_NEAR(ext[1], 0.0, 1e-10);
  EXPECT_NEAR(ext[2], 0.0, 1e-10);
  EXPECT_NEAR(ext[3], 0.0, 1e-10);
  EXPECT_NEAR(ext[4], 1.0, 1e-10);
  EXPECT_NEAR(ext[5], 0.0, 1e-10);
}

TEST(BodyNodeExternalForce, AddExtForceGlobalForceGlobalOffset)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);
  FreeJoint* joint = static_cast<FreeJoint*>(body->getParentJoint());

  // Set body transform: translate and rotate
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
  joint->setTransform(tf);

  body->clearExternalForces();

  // Apply global force at global offset (both at origin, simple case)
  Eigen::Vector3d globalForce(0.0, 0.0, 1.0);
  Eigen::Vector3d globalOffset(0.0, 0.0, 0.0);
  body->addExtForce(globalForce, globalOffset, false, false);

  Eigen::Vector6d ext = body->getExternalForceLocal();
  EXPECT_NEAR(ext[3], 0.0, 1e-10);
  EXPECT_NEAR(ext[4], 0.0, 1e-10);
  EXPECT_NEAR(ext[5], 1.0, 1e-10);
}

TEST(BodyNodeExternalForce, SetExtForceReplacesForces)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  body->clearExternalForces();

  // Add a force
  body->addExtForce(
      Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d::Zero(), true, true);

  // Now set a different force (should replace, not add)
  body->setExtForce(
      Eigen::Vector3d(0.0, 2.0, 0.0), Eigen::Vector3d::Zero(), true, true);

  Eigen::Vector6d ext = body->getExternalForceLocal();
  EXPECT_NEAR(ext[3], 0.0, 1e-10); // Old X force should be gone
  EXPECT_NEAR(ext[4], 2.0, 1e-10); // Only new Y force
  EXPECT_NEAR(ext[5], 0.0, 1e-10);
}

TEST(BodyNodeExternalForce, NaNForceIsIgnored)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  body->clearExternalForces();

  // Try to add a NaN force - should be ignored
  Eigen::Vector3d nanForce(std::nan(""), 0.0, 0.0);
  body->addExtForce(nanForce, Eigen::Vector3d::Zero(), true, true);

  Eigen::Vector6d ext = body->getExternalForceLocal();
  EXPECT_TRUE(ext.isZero());
}

TEST(BodyNodeExternalForce, InfOffsetIsIgnored)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  body->clearExternalForces();

  // Try to add force with infinite offset - should be ignored
  Eigen::Vector3d infOffset(std::numeric_limits<double>::infinity(), 0.0, 0.0);
  body->addExtForce(Eigen::Vector3d(1.0, 0.0, 0.0), infOffset, true, true);

  Eigen::Vector6d ext = body->getExternalForceLocal();
  EXPECT_TRUE(ext.isZero());
}

// ============================================================================
// External Torque Tests
// ============================================================================

TEST(BodyNodeExternalForce, AddExtTorqueLocal)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  body->clearExternalForces();

  Eigen::Vector3d torque(1.0, 2.0, 3.0);
  body->addExtTorque(torque, true);

  Eigen::Vector6d ext = body->getExternalForceLocal();
  EXPECT_NEAR(ext[0], 1.0, 1e-10);
  EXPECT_NEAR(ext[1], 2.0, 1e-10);
  EXPECT_NEAR(ext[2], 3.0, 1e-10);
  EXPECT_NEAR(ext[3], 0.0, 1e-10);
  EXPECT_NEAR(ext[4], 0.0, 1e-10);
  EXPECT_NEAR(ext[5], 0.0, 1e-10);
}

TEST(BodyNodeExternalForce, AddExtTorqueGlobal)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);
  FreeJoint* joint = static_cast<FreeJoint*>(body->getParentJoint());

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear() = Eigen::AngleAxisd(dart::math::half_pi, Eigen::Vector3d::UnitZ())
                    .matrix();
  joint->setTransform(tf);

  body->clearExternalForces();

  Eigen::Vector3d globalTorque(1.0, 0.0, 0.0);
  body->addExtTorque(globalTorque, false);

  Eigen::Vector6d ext = body->getExternalForceLocal();
  EXPECT_NEAR(ext[0], 0.0, 1e-10);
  EXPECT_NEAR(ext[1], -1.0, 1e-10);
  EXPECT_NEAR(ext[2], 0.0, 1e-10);
}

TEST(BodyNodeExternalForce, SetExtTorqueLocal)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  body->clearExternalForces();

  body->addExtTorque(Eigen::Vector3d(1.0, 0.0, 0.0), true);
  body->setExtTorque(Eigen::Vector3d(0.0, 5.0, 0.0), true);

  Eigen::Vector6d ext = body->getExternalForceLocal();
  EXPECT_NEAR(ext[0], 0.0, 1e-10);
  EXPECT_NEAR(ext[1], 5.0, 1e-10);
  EXPECT_NEAR(ext[2], 0.0, 1e-10);
}

TEST(BodyNodeExternalForce, SetExtTorqueGlobal)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  body->clearExternalForces();

  Eigen::Vector3d globalTorque(0.0, 0.0, 3.0);
  body->setExtTorque(globalTorque, false);

  Eigen::Vector6d ext = body->getExternalForceLocal();
  EXPECT_NEAR(ext[2], 3.0, 1e-10);
}

TEST(BodyNodeExternalForce, NaNTorqueIsIgnored)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  body->clearExternalForces();

  Eigen::Vector3d nanTorque(std::nan(""), 0.0, 0.0);
  body->addExtTorque(nanTorque, true);

  Eigen::Vector6d ext = body->getExternalForceLocal();
  EXPECT_TRUE(ext.isZero());
}

TEST(BodyNodeExternalForce, InfTorqueIsIgnored)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  body->clearExternalForces();

  Eigen::Vector3d infTorque(std::numeric_limits<double>::infinity(), 0.0, 0.0);
  body->addExtTorque(infTorque, true);

  Eigen::Vector6d ext = body->getExternalForceLocal();
  EXPECT_TRUE(ext.isZero());
}

TEST(BodyNodeExternalForce, SetExtForceNaNIsIgnored)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  body->clearExternalForces();
  body->addExtForce(
      Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d::Zero(), true, true);

  Eigen::Vector3d nanForce(std::nan(""), 0.0, 0.0);
  body->setExtForce(nanForce, Eigen::Vector3d::Zero(), true, true);

  Eigen::Vector6d ext = body->getExternalForceLocal();
  EXPECT_NEAR(ext[3], 1.0, 1e-10);
}

TEST(BodyNodeExternalForce, SetExtForceInfOffsetIsIgnored)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  body->clearExternalForces();
  body->addExtForce(
      Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d::Zero(), true, true);

  Eigen::Vector3d infOffset(std::numeric_limits<double>::infinity(), 0.0, 0.0);
  body->setExtForce(Eigen::Vector3d(2.0, 0.0, 0.0), infOffset, true, true);

  Eigen::Vector6d ext = body->getExternalForceLocal();
  EXPECT_NEAR(ext[3], 1.0, 1e-10);
}

TEST(BodyNodeExternalForce, SetExtTorqueNaNIsIgnored)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  body->clearExternalForces();
  body->addExtTorque(Eigen::Vector3d(1.0, 0.0, 0.0), true);

  Eigen::Vector3d nanTorque(std::nan(""), 0.0, 0.0);
  body->setExtTorque(nanTorque, true);

  Eigen::Vector6d ext = body->getExternalForceLocal();
  EXPECT_NEAR(ext[0], 1.0, 1e-10);
}

TEST(BodyNodeExternalForce, GetExternalForceGlobal)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  body->clearExternalForces();
  body->addExtForce(
      Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d::Zero(), true, true);

  Eigen::Vector6d globalExt = body->getExternalForceGlobal();
  EXPECT_TRUE(globalExt.array().isFinite().all());
  EXPECT_GT(globalExt.norm(), 0.0);
}

// ============================================================================
// Constraint Impulse Tests
// ============================================================================

TEST(BodyNodeConstraintImpulse, SetConstraintImpulse)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  Eigen::Vector6d impulse;
  impulse << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

  body->setConstraintImpulse(impulse);
  EXPECT_TRUE(body->getConstraintImpulse().isApprox(impulse));
}

TEST(BodyNodeConstraintImpulse, AddConstraintImpulse6d)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  body->clearConstraintImpulse();

  Eigen::Vector6d impulse1;
  impulse1 << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  body->addConstraintImpulse(impulse1);

  Eigen::Vector6d impulse2;
  impulse2 << 0.0, 2.0, 0.0, 0.0, 0.0, 0.0;
  body->addConstraintImpulse(impulse2);

  Eigen::Vector6d expected;
  expected << 1.0, 2.0, 0.0, 0.0, 0.0, 0.0;
  EXPECT_TRUE(body->getConstraintImpulse().isApprox(expected));
}

TEST(BodyNodeConstraintImpulse, AddConstraintImpulseWithOffset)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  body->clearConstraintImpulse();

  Eigen::Vector3d impulse(1.0, 0.0, 0.0);
  Eigen::Vector3d offset(0.0, 0.0, 0.0);
  body->addConstraintImpulse(impulse, offset, true, true);

  Eigen::Vector6d result = body->getConstraintImpulse();
  EXPECT_NEAR(result[3], 1.0, 1e-10);
}

TEST(BodyNodeConstraintImpulse, AddConstraintImpulseGlobalImpulseLocalOffset)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  body->clearConstraintImpulse();

  Eigen::Vector3d globalImpulse(1.0, 0.0, 0.0);
  Eigen::Vector3d localOffset(0.0, 0.0, 0.5);
  body->addConstraintImpulse(globalImpulse, localOffset, false, true);

  Eigen::Vector6d result = body->getConstraintImpulse();
  EXPECT_TRUE(result.array().isFinite().all());
  EXPECT_GT(result.norm(), 0.0);
}

TEST(BodyNodeConstraintImpulse, AddConstraintImpulseGlobalOffset)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  body->clearConstraintImpulse();

  Eigen::Vector3d impulse(0.0, 1.0, 0.0);
  Eigen::Vector3d globalOffset(0.0, 0.0, 0.0);
  body->addConstraintImpulse(impulse, globalOffset, true, false);

  Eigen::Vector6d result = body->getConstraintImpulse();
  EXPECT_TRUE(result.array().isFinite().all());
}

TEST(BodyNodeConstraintImpulse, ClearConstraintImpulse)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  body->setConstraintImpulse(Eigen::Vector6d::Ones());
  EXPECT_FALSE(body->getConstraintImpulse().isZero());

  body->clearConstraintImpulse();
  EXPECT_TRUE(body->getConstraintImpulse().isZero());
}

TEST(BodyNodeConstraintImpulse, PositionConstraintImpulse)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  body->clearPositionConstraintImpulse();
  EXPECT_TRUE(body->getPositionConstraintImpulse().isZero());

  Eigen::Vector6d impulse = Eigen::Vector6d::Ones() * 3.0;
  body->addPositionConstraintImpulse(impulse);
  EXPECT_TRUE(body->getPositionConstraintImpulse().isApprox(impulse));

  body->clearPositionConstraintImpulse();
  EXPECT_TRUE(body->getPositionConstraintImpulse().isZero());
}

// ============================================================================
// Body Force and Energy Tests
// ============================================================================

TEST(BodyNodeForce, GetBodyForce)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  const Eigen::Vector6d& bodyForce = body->getBodyForce();
  EXPECT_EQ(bodyForce.size(), 6);
  EXPECT_TRUE(bodyForce.array().isFinite().all());
}

TEST(BodyNodeEnergy, ComputeKineticEnergy)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);
  body->setMass(2.0);

  double ke = body->computeKineticEnergy();
  EXPECT_GE(ke, 0.0);
  EXPECT_TRUE(std::isfinite(ke));
}

TEST(BodyNodeEnergy, ComputePotentialEnergy)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);
  body->setMass(2.0);

  Eigen::Vector3d gravity(0, 0, -9.81);
  double pe = body->computePotentialEnergy(gravity);
  EXPECT_TRUE(std::isfinite(pe));
}

TEST(BodyNodeEnergy, ComputeLagrangian)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);
  body->setMass(2.0);

  Eigen::Vector3d gravity(0, 0, -9.81);
  double lagrangian = body->computeLagrangian(gravity);
  EXPECT_TRUE(std::isfinite(lagrangian));

  double ke = body->computeKineticEnergy();
  double pe = body->computePotentialEnergy(gravity);
  EXPECT_NEAR(lagrangian, ke - pe, 1e-10);
}

TEST(BodyNodeEnergy, LinearMomentum)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);
  body->setMass(2.0);

  Eigen::Vector3d momentum = body->getLinearMomentum();
  EXPECT_TRUE(momentum.array().isFinite().all());
}

TEST(BodyNodeEnergy, AngularMomentum)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);
  body->setMass(2.0);

  Eigen::Vector3d pivot = Eigen::Vector3d::Zero();
  Eigen::Vector3d angMomentum = body->getAngularMomentum(pivot);
  EXPECT_TRUE(angMomentum.array().isFinite().all());
}

// ============================================================================
// Gravity Mode Tests
// ============================================================================

TEST(BodyNodeGravity, SetGravityModeIdempotent)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  EXPECT_TRUE(body->getGravityMode());

  body->setGravityMode(true);
  EXPECT_TRUE(body->getGravityMode());

  body->setGravityMode(false);
  EXPECT_FALSE(body->getGravityMode());

  body->setGravityMode(false);
  EXPECT_FALSE(body->getGravityMode());
}

// ============================================================================
// Collidable Tests
// ============================================================================

TEST(BodyNodeCollidable, SetAndGetCollidable)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  EXPECT_TRUE(body->isCollidable());

  body->setCollidable(false);
  EXPECT_FALSE(body->isCollidable());

  body->setCollidable(true);
  EXPECT_TRUE(body->isCollidable());
}

// ============================================================================
// BodyNode Indexing Tests
// ============================================================================

TEST(BodyNodeIndexing, IndexInSkeleton)
{
  auto skeleton = Skeleton::create("idx_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setName("body0");

  auto child = pair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child.second->setName("body1");

  EXPECT_EQ(pair.second->getIndexInSkeleton(), 0u);
  EXPECT_EQ(child.second->getIndexInSkeleton(), 1u);
}

TEST(BodyNodeIndexing, IndexInTree)
{
  auto skeleton = Skeleton::create("tree_idx_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();

  auto child = pair.second->createChildJointAndBodyNodePair<RevoluteJoint>();

  EXPECT_EQ(pair.second->getIndexInTree(), 0u);
  EXPECT_EQ(child.second->getIndexInTree(), 1u);
}

TEST(BodyNodeIndexing, TreeIndex)
{
  auto skeleton = Skeleton::create("tree_index_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();

  auto child = pair.second->createChildJointAndBodyNodePair<RevoluteJoint>();

  EXPECT_EQ(pair.second->getTreeIndex(), 0u);
  EXPECT_EQ(child.second->getTreeIndex(), 0u);
}

// ============================================================================
// BodyNode Dependent DOF Tests
// ============================================================================

TEST(BodyNodeDependentDofs, DependsOn)
{
  auto skeleton = Skeleton::create("depends_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());

  auto child = pair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child.first->setAxis(Eigen::Vector3d::UnitY());

  EXPECT_TRUE(pair.second->dependsOn(0));
  EXPECT_FALSE(pair.second->dependsOn(1));

  EXPECT_TRUE(child.second->dependsOn(0));
  EXPECT_TRUE(child.second->dependsOn(1));
}

TEST(BodyNodeDependentDofs, GetNumDependentGenCoords)
{
  auto skeleton = Skeleton::create("dep_coords_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();

  auto child = pair.second->createChildJointAndBodyNodePair<RevoluteJoint>();

  EXPECT_EQ(pair.second->getNumDependentGenCoords(), 6u);
  EXPECT_EQ(child.second->getNumDependentGenCoords(), 7u);
}

TEST(BodyNodeDependentDofs, GetDependentGenCoordIndices)
{
  auto skeleton = Skeleton::create("dep_indices_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());

  auto child = pair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child.first->setAxis(Eigen::Vector3d::UnitY());

  auto indices = child.second->getDependentGenCoordIndices();
  EXPECT_EQ(indices.size(), 2u);
  EXPECT_EQ(indices[0], 0u);
  EXPECT_EQ(indices[1], 1u);
}

TEST(BodyNodeDependentDofs, GetDependentDofs)
{
  auto skeleton = Skeleton::create("dep_dofs_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());

  auto child = pair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child.first->setAxis(Eigen::Vector3d::UnitY());

  auto dofs = child.second->getDependentDofs();
  EXPECT_EQ(dofs.size(), 2u);

  const BodyNode* constChild = child.second;
  auto constDofs = constChild->getDependentDofs();
  EXPECT_EQ(constDofs.size(), 2u);
}

TEST(BodyNodeDependentDofs, GetChainDofs)
{
  auto skeleton = Skeleton::create("chain_dofs_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());

  auto child = pair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child.first->setAxis(Eigen::Vector3d::UnitY());

  auto chainDofs = child.second->getChainDofs();
  EXPECT_EQ(chainDofs.size(), 2u);
}

// ============================================================================
// BodyNode Jacobian Tests
// ============================================================================

TEST(BodyNodeJacobian, GetJacobian)
{
  auto skeleton = Skeleton::create("jac_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());
  pair.second->setMass(1.0);

  const auto& jac = pair.second->getJacobian();
  EXPECT_EQ(jac.rows(), 6);
  EXPECT_EQ(jac.cols(), 1);
  EXPECT_TRUE(jac.array().isFinite().all());
}

TEST(BodyNodeJacobian, GetWorldJacobian)
{
  auto skeleton = Skeleton::create("world_jac_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());
  pair.second->setMass(1.0);

  const auto& worldJac = pair.second->getWorldJacobian();
  EXPECT_EQ(worldJac.rows(), 6);
  EXPECT_EQ(worldJac.cols(), 1);
  EXPECT_TRUE(worldJac.array().isFinite().all());
}

TEST(BodyNodeJacobian, GetJacobianSpatialDeriv)
{
  auto skeleton = Skeleton::create("jac_deriv_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());
  pair.second->setMass(1.0);

  skeleton->setVelocity(0, 1.0);

  const auto& jacDeriv = pair.second->getJacobianSpatialDeriv();
  EXPECT_EQ(jacDeriv.rows(), 6);
  EXPECT_EQ(jacDeriv.cols(), 1);
  EXPECT_TRUE(jacDeriv.array().isFinite().all());
}

TEST(BodyNodeJacobian, GetJacobianClassicDeriv)
{
  auto skeleton = Skeleton::create("jac_classic_deriv_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());
  pair.second->setMass(1.0);

  skeleton->setVelocity(0, 1.0);

  const auto& jacClassicDeriv = pair.second->getJacobianClassicDeriv();
  EXPECT_EQ(jacClassicDeriv.rows(), 6);
  EXPECT_EQ(jacClassicDeriv.cols(), 1);
  EXPECT_TRUE(jacClassicDeriv.array().isFinite().all());
}

// ============================================================================
// BodyNode Velocity/Acceleration Tests
// ============================================================================

TEST(BodyNodeVelocity, GetBodyVelocityChange)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  const Eigen::Vector6d& delV = body->getBodyVelocityChange();
  EXPECT_EQ(delV.size(), 6);
  EXPECT_TRUE(delV.array().isFinite().all());
}

TEST(BodyNodeVelocity, GetPartialAcceleration)
{
  auto skeleton = Skeleton::create("partial_acc_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());
  pair.second->setMass(1.0);

  skeleton->setVelocity(0, 1.0);

  const auto& partialAcc = pair.second->getPartialAcceleration();
  EXPECT_EQ(partialAcc.size(), 6);
  EXPECT_TRUE(partialAcc.array().isFinite().all());
}

// ============================================================================
// BodyNode COM Tests
// ============================================================================

TEST(BodyNodeCOM, GetCOM)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);
  body->setMass(1.0);

  auto com = body->getCOM();
  EXPECT_TRUE(com.array().isFinite().all());
}

TEST(BodyNodeCOM, GetCOMLinearVelocity)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);
  body->setMass(1.0);

  auto comVel = body->getCOMLinearVelocity();
  EXPECT_TRUE(comVel.array().isFinite().all());
}

TEST(BodyNodeCOM, GetCOMSpatialVelocity)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);
  body->setMass(1.0);

  auto comSpatialVel = body->getCOMSpatialVelocity();
  EXPECT_EQ(comSpatialVel.size(), 6);
  EXPECT_TRUE(comSpatialVel.array().isFinite().all());
}

TEST(BodyNodeCOM, GetCOMLinearAcceleration)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);
  body->setMass(1.0);

  auto comLinAcc = body->getCOMLinearAcceleration();
  EXPECT_TRUE(comLinAcc.array().isFinite().all());
}

TEST(BodyNodeCOM, GetCOMSpatialAcceleration)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);
  body->setMass(1.0);

  auto comSpatialAcc = body->getCOMSpatialAcceleration();
  EXPECT_EQ(comSpatialAcc.size(), 6);
  EXPECT_TRUE(comSpatialAcc.array().isFinite().all());
}

// ============================================================================
// BodyNode Reactive Test
// ============================================================================

TEST(BodyNodeReactive, IsReactive)
{
  auto skeleton = Skeleton::create("reactive_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());
  pair.second->setMass(1.0);

  skeleton->setMobile(true);
  EXPECT_TRUE(pair.second->isReactive());

  skeleton->setMobile(false);
  EXPECT_FALSE(pair.second->isReactive());
}

// ============================================================================
// BodyNode Color/Alpha Tests
// ============================================================================

TEST(BodyNodeVisual, SetColor3d)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  body->createShapeNodeWith<VisualAspect>(box);

  Eigen::Vector3d color(1.0, 0.0, 0.0);
  body->setColor(color);

  auto* visual = body->getShapeNode(0)->getVisualAspect();
  ASSERT_NE(visual, nullptr);
  EXPECT_TRUE(visual->getColor().head<3>().isApprox(color));
}

TEST(BodyNodeVisual, SetColor4d)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  body->createShapeNodeWith<VisualAspect>(box);

  Eigen::Vector4d color(0.0, 1.0, 0.0, 0.5);
  body->setColor(color);

  auto* visual = body->getShapeNode(0)->getVisualAspect();
  ASSERT_NE(visual, nullptr);
  EXPECT_TRUE(visual->getColor().isApprox(color.head<3>()));
  EXPECT_NEAR(visual->getAlpha(), color[3], 1e-10);
}

TEST(BodyNodeVisual, SetAlpha)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  body->createShapeNodeWith<VisualAspect>(box);

  body->setAlpha(0.3);

  auto* visual = body->getShapeNode(0)->getVisualAspect();
  ASSERT_NE(visual, nullptr);
  EXPECT_NEAR(visual->getAlpha(), 0.3, 1e-10);
}

// ============================================================================
// BodyNode Inertia Tests
// ============================================================================

TEST(BodyNodeInertia, SetInertia)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  Inertia inertia;
  inertia.setMass(3.0);
  inertia.setLocalCOM(Eigen::Vector3d(0.1, 0.2, 0.3));

  body->setInertia(inertia);

  EXPECT_DOUBLE_EQ(body->getMass(), 3.0);
  EXPECT_TRUE(body->getLocalCOM().isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
  EXPECT_EQ(&body->getInertia(), &body->getInertia());
}

TEST(BodyNodeInertia, SetInertiaIdempotent)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  Inertia inertia;
  inertia.setMass(2.0);
  body->setInertia(inertia);

  auto version1 = skeleton->getVersion();
  body->setInertia(inertia);
  auto version2 = skeleton->getVersion();

  EXPECT_EQ(version1, version2);
}

TEST(BodyNodeInertia, GetSpatialInertia)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);
  body->setMass(2.0);

  const auto& spatialInertia = body->getSpatialInertia();
  EXPECT_EQ(spatialInertia.rows(), 6);
  EXPECT_EQ(spatialInertia.cols(), 6);
  EXPECT_TRUE(spatialInertia.array().isFinite().all());
}

TEST(BodyNodeInertia, GetArticulatedInertia)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);
  body->setMass(2.0);

  const auto& artInertia = body->getArticulatedInertia();
  EXPECT_TRUE(artInertia.matrix().array().isFinite().all());

  const auto& artInertiaImplicit = body->getArticulatedInertiaImplicit();
  EXPECT_TRUE(artInertiaImplicit.matrix().array().isFinite().all());
}

// ============================================================================
// BodyNode Child Accessors
// ============================================================================

TEST(BodyNodeChildren, GetChildJoint)
{
  auto skeleton = Skeleton::create("child_joint_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();

  auto child = pair.second->createChildJointAndBodyNodePair<RevoluteJoint>();

  EXPECT_EQ(pair.second->getNumChildJoints(), 1u);
  EXPECT_EQ(pair.second->getChildJoint(0), child.first);

  const BodyNode* constBody = pair.second;
  EXPECT_EQ(constBody->getChildJoint(0), child.first);
}

TEST(BodyNodeChildren, GetChildBodyNode)
{
  auto skeleton = Skeleton::create("child_body_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();

  auto child = pair.second->createChildJointAndBodyNodePair<RevoluteJoint>();

  EXPECT_EQ(pair.second->getNumChildBodyNodes(), 1u);
  EXPECT_EQ(pair.second->getChildBodyNode(0), child.second);

  const BodyNode* constBody = pair.second;
  EXPECT_EQ(constBody->getChildBodyNode(0), child.second);
}

//==============================================================================
TEST(BodyNodeConstAccessors, ParentChildAndSpatialFrames)
{
  auto skeleton = Skeleton::create("const_body_access");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();

  const BodyNode* constChild = childPair.second;
  EXPECT_EQ(constChild->getParentJoint(), childPair.first);
  EXPECT_EQ(constChild->getParentBodyNode(), rootPair.second);

  const BodyNode* constRoot = rootPair.second;
  EXPECT_EQ(constRoot->getChildJoint(0), childPair.first);
  EXPECT_EQ(constRoot->getChildBodyNode(0), childPair.second);

  rootPair.first->setVelocities(Eigen::Vector6d::Constant(0.1));
  rootPair.first->setAccelerations(Eigen::Vector6d::Constant(0.2));

  auto frame = SimpleFrame::createShared(Frame::World(), "const_frame");
  frame->setTranslation(Eigen::Vector3d(0.1, -0.2, 0.3));

  const auto velWorld
      = constRoot->getSpatialVelocity(Frame::World(), Frame::World());
  const auto velFrame = constRoot->getSpatialVelocity(frame.get(), frame.get());
  const auto accWorld
      = constRoot->getSpatialAcceleration(Frame::World(), Frame::World());
  const auto accFrame
      = constRoot->getSpatialAcceleration(frame.get(), frame.get());

  EXPECT_TRUE(velWorld.allFinite());
  EXPECT_TRUE(velFrame.allFinite());
  EXPECT_TRUE(accWorld.allFinite());
  EXPECT_TRUE(accFrame.allFinite());
}

//==============================================================================
TEST(NodeConstAccessors, BodyNodeAndSkeletonPtrs)
{
  auto skeleton = Skeleton::create("node_const_access");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto* marker = pair.second->createMarker(std::string("marker"));

  const Node* constNode = marker;
  const auto bodyPtr = constNode->getBodyNodePtr();
  const auto skelPtr = constNode->getSkeleton();

  EXPECT_EQ(bodyPtr.get(), pair.second);
  EXPECT_EQ(skelPtr.get(), skeleton.get());
}

// ============================================================================
// BodyNode Copy/Assignment Tests
// ============================================================================

TEST(BodyNodeCopy, CopyOperator)
{
  auto skeleton = Skeleton::create("copy_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setName("body0");
  pair.second->setMass(3.0);
  pair.second->setGravityMode(false);

  auto child = pair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child.second->setName("body1");

  child.second->copy(pair.second);
  EXPECT_DOUBLE_EQ(child.second->getMass(), 3.0);
  EXPECT_FALSE(child.second->getGravityMode());
}

TEST(BodyNodeCopy, CopyFromPointer)
{
  auto skeleton = Skeleton::create("copy_ptr_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(4.0);

  auto child = pair.second->createChildJointAndBodyNodePair<RevoluteJoint>();

  child.second->copy(pair.second);
  EXPECT_DOUBLE_EQ(child.second->getMass(), 4.0);

  child.second->copy(static_cast<BodyNode*>(nullptr));
  EXPECT_DOUBLE_EQ(child.second->getMass(), 4.0);
}

TEST(BodyNodeCopy, AssignmentOperator)
{
  auto skeleton = Skeleton::create("assign_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(5.0);

  auto child = pair.second->createChildJointAndBodyNodePair<RevoluteJoint>();

  *child.second = *pair.second;
  EXPECT_DOUBLE_EQ(child.second->getMass(), 5.0);
}

// ============================================================================
// BodyNode Nodes Accessor
// ============================================================================

TEST(BodyNodeNodes, GetNodes)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  body->createEndEffector("ee1");
  Marker::BasicProperties markerProps;
  markerProps.mName = "marker1";
  body->createMarker(markerProps);

  auto nodes = body->getNodes();
  EXPECT_GE(nodes.size(), 2u);

  const BodyNode* constBody = body;
  auto constNodes = constBody->getNodes();
  EXPECT_GE(constNodes.size(), 2u);
}

// ============================================================================
// BodyNode SoftBodyNode Cast
// ============================================================================

TEST(BodyNodeCast, AsSoftBodyNode)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  EXPECT_EQ(body->asSoftBodyNode(), nullptr);

  const BodyNode* constBody = body;
  EXPECT_EQ(constBody->asSoftBodyNode(), nullptr);
}

// ============================================================================
// BodyNode ClearInternalForces
// ============================================================================

TEST(BodyNodeForces, ClearInternalForces)
{
  auto skeleton = Skeleton::create("clear_int_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());
  pair.second->setMass(1.0);

  pair.first->setForce(0, 10.0);
  EXPECT_NEAR(pair.first->getForce(0), 10.0, 1e-10);

  pair.second->clearInternalForces();
  EXPECT_NEAR(pair.first->getForce(0), 0.0, 1e-10);
}

// ============================================================================
// BodyNode SetAspectState
// ============================================================================

TEST(BodyNodeAspect, SetAspectState)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  body->clearExternalForces();

  BodyNode::AspectState state;
  state.mFext = Eigen::Vector6d::Ones() * 2.0;
  body->setAspectState(state);

  EXPECT_TRUE(body->getExternalForceLocal().isApprox(state.mFext));

  body->setAspectState(state);
  EXPECT_TRUE(body->getExternalForceLocal().isApprox(state.mFext));
}

TEST(BodyNodeAspect, SetAspectProperties)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  BodyNode::AspectProperties props;
  props.mName = "new_body_name";
  props.mGravityMode = false;

  body->setAspectProperties(props);

  EXPECT_EQ(body->getName(), "new_body_name");
  EXPECT_FALSE(body->getGravityMode());
}

TEST(BodyNodeAspect, GetBodyNodeProperties)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);
  body->setMass(3.0);

  auto props = body->getBodyNodeProperties();
  (void)props;
}

// ============================================================================
// BodyNode RemoveAllShapeNodes
// ============================================================================

TEST(BodyNodeShapeNodes, RemoveAllShapeNodes)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  body->createShapeNodeWith<VisualAspect>(box);

  EXPECT_EQ(body->getNumShapeNodes(), 1u);

  body->removeAllShapeNodes();
  EXPECT_EQ(body->getNumShapeNodes(), 0u);
}

// ============================================================================
// BodyNode GetShapeNodes (vector)
// ============================================================================

TEST(BodyNodeShapeNodes, GetShapeNodesVector)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  auto sphere = std::make_shared<SphereShape>(0.5);

  body->createShapeNodeWith<VisualAspect>(box);
  body->createShapeNodeWith<CollisionAspect>(sphere);

  EXPECT_EQ(body->getNumShapeNodes(), 2u);

  std::vector<ShapeNode*> shapeNodes;
  shapeNodes.reserve(body->getNumShapeNodes());
  for (std::size_t i = 0; i < body->getNumShapeNodes(); ++i) {
    shapeNodes.push_back(body->getShapeNode(i));
  }
  EXPECT_EQ(shapeNodes.size(), 2u);
  EXPECT_EQ(shapeNodes[0], body->getShapeNode(0));
  EXPECT_EQ(shapeNodes[1], body->getShapeNode(1));

  const BodyNode* constBody = body;
  std::vector<const ShapeNode*> constShapeNodes;
  constShapeNodes.reserve(constBody->getNumShapeNodes());
  for (std::size_t i = 0; i < constBody->getNumShapeNodes(); ++i) {
    constShapeNodes.push_back(constBody->getShapeNode(i));
  }
  EXPECT_EQ(constShapeNodes.size(), 2u);
  EXPECT_EQ(constShapeNodes[0], constBody->getShapeNode(0));
  EXPECT_EQ(constShapeNodes[1], constBody->getShapeNode(1));
}

// ============================================================================
// BodyNode Skeleton Accessor
// ============================================================================

TEST(BodyNodeSkeleton, GetSkeleton)
{
  auto skeleton = Skeleton::create("skel_accessor_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();

  EXPECT_EQ(pair.second->getSkeleton(), skeleton);

  const BodyNode* constBody = pair.second;
  EXPECT_NE(constBody->getSkeleton(), nullptr);
}

// ============================================================================
// BodyNode Parent Accessors
// ============================================================================

TEST(BodyNodeParent, GetParentJoint)
{
  auto skeleton = Skeleton::create("parent_joint_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();

  EXPECT_NE(pair.second->getParentJoint(), nullptr);
  EXPECT_EQ(pair.second->getParentJoint(), pair.first);

  const BodyNode* constBody = pair.second;
  EXPECT_NE(constBody->getParentJoint(), nullptr);
}

TEST(BodyNodeParent, GetParentBodyNode)
{
  auto skeleton = Skeleton::create("parent_body_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();

  EXPECT_EQ(pair.second->getParentBodyNode(), nullptr);

  auto child = pair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  EXPECT_EQ(child.second->getParentBodyNode(), pair.second);

  const BodyNode* constChild = child.second;
  EXPECT_EQ(constChild->getParentBodyNode(), pair.second);
}

// ============================================================================
// BodyNode Transform Derivatives
// ============================================================================

TEST(BodyNodeTransform, GetWorldTransformDerivative)
{
  auto skeleton = Skeleton::create("transform_deriv_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());
  pair.second->setMass(1.0);

  skeleton->setPosition(0, 0.5);

  const auto& deriv = pair.second->getWorldTransformDerivative(0);
  EXPECT_EQ(deriv.rows(), 4);
  EXPECT_EQ(deriv.cols(), 4);
  EXPECT_TRUE(deriv.array().isFinite().all());
}

TEST(BodyNodeTransform, GetWorldTransformSecondDerivative)
{
  auto skeleton = Skeleton::create("transform_second_deriv_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());
  pair.second->setMass(1.0);

  skeleton->setPosition(0, 0.5);

  const auto& secondDeriv
      = pair.second->getWorldTransformSecondDerivative(0, 0);
  EXPECT_EQ(secondDeriv.rows(), 4);
  EXPECT_EQ(secondDeriv.cols(), 4);
  EXPECT_TRUE(secondDeriv.array().isFinite().all());
}

TEST(BodyNodeCoverage, GravityModeOff)
{
  auto skeleton = Skeleton::create("gravity_mode_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);
  pair.second->setGravityMode(false);
  EXPECT_FALSE(pair.second->getGravityMode());

  skeleton->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  skeleton->computeForwardDynamics();

  const Eigen::Vector6d& acc = pair.second->getSpatialAcceleration();
  EXPECT_TRUE(acc.isApprox(Eigen::Vector6d::Zero(), 1e-10));
}

TEST(BodyNodeCoverage, AddExtForceGlobalOffset)
{
  auto skeleton = Skeleton::create("ext_force_global");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  Eigen::Vector3d globalForce(0.0, 10.0, 0.0);
  Eigen::Vector3d globalOffset(1.0, 0.0, 0.0);

  pair.second->addExtForce(globalForce, globalOffset, false, false);
  const Eigen::Vector6d& extForce = pair.second->getExternalForceLocal();
  EXPECT_GT(extForce.norm(), 0.0);

  pair.second->clearExternalForces();
  EXPECT_NEAR(pair.second->getExternalForceLocal().norm(), 0.0, 1e-10);
}

TEST(BodyNodeCoverage, GetNodesConstAndNonConst)
{
  auto skeleton = Skeleton::create("get_nodes_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->createShapeNodeWith<VisualAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0)));

  BodyNode* body = pair.second;
  std::vector<Node*> nodes = body->getNodes();
  EXPECT_GE(nodes.size(), 1u);

  const BodyNode* constBody = body;
  std::vector<const Node*> constNodes = constBody->getNodes();
  EXPECT_EQ(constNodes.size(), nodes.size());
}

#include <dart/simulation/world.hpp>

#include <dart/dynamics/arrow_shape.hpp>
#include <dart/dynamics/soft_body_node.hpp>

TEST(NodeLifecycle, ShapeNodeAddAndCount)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(0.3, 0.4, 0.5));
  ShapeNode* shapeNode
      = body->createShapeNodeWith<VisualAspect, CollisionAspect>(box);
  ASSERT_NE(shapeNode, nullptr);
  EXPECT_EQ(body->getNumShapeNodes(), 1u);
  EXPECT_EQ(body->getShapeNode(0), shapeNode);
}

TEST(ArrowShape, SetPropertiesAndPositions)
{
  ArrowShape::Properties props;
  props.mRadius = 0.05;
  props.mHeadRadiusScale = 0.5;
  props.mHeadLengthScale = 1.5;
  props.mMinHeadLength = -0.1;
  props.mMaxHeadLength = 1.0;
  props.mDoubleArrow = true;

  ArrowShape arrow(
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::UnitZ(),
      props,
      Eigen::Vector4d::Ones(),
      6);

  arrow.setPositions(
      Eigen::Vector3d(0.1, 0.0, 0.0), Eigen::Vector3d(0.1, 0.2, 0.3));
  EXPECT_TRUE(arrow.getTail().isApprox(Eigen::Vector3d(0.1, 0.0, 0.0)));
  EXPECT_TRUE(arrow.getHead().isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));

  arrow.setProperties(props);
  const auto& updated = arrow.getProperties();
  EXPECT_NEAR(updated.mRadius, 0.05, 1e-12);
  EXPECT_DOUBLE_EQ(updated.mHeadRadiusScale, 1.0);
  EXPECT_DOUBLE_EQ(updated.mHeadLengthScale, 1.0);
  EXPECT_DOUBLE_EQ(updated.mMinHeadLength, 0.0);
  EXPECT_DOUBLE_EQ(updated.mMaxHeadLength, 1.0);
  EXPECT_TRUE(updated.mDoubleArrow);
}

TEST(FreeJoint, SpatialVelocityAndConversions)
{
  auto skeleton = Skeleton::create("free_joint_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto* joint = static_cast<FreeJoint*>(pair.first);
  ASSERT_NE(joint, nullptr);

  Eigen::Vector6d spatialVel = Eigen::Vector6d::Zero();
  spatialVel[0] = 0.2;
  spatialVel[4] = -0.1;
  joint->setSpatialVelocity(spatialVel, Frame::World(), Frame::World());
  const Eigen::Vector6d currentVel = pair.second->getSpatialVelocity();
  EXPECT_TRUE(currentVel.array().isFinite().all());
  EXPECT_GT(currentVel.norm(), 0.0);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear()
      = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()).toRotationMatrix();
  tf.translation() = Eigen::Vector3d(0.2, -0.1, 0.3);

  Eigen::Vector6d positions
      = joint->convertToPositions(tf, FreeJoint::CoordinateChart::EULER_ZYX);
  Eigen::Isometry3d tfRoundTrip = joint->convertToTransform(
      positions, FreeJoint::CoordinateChart::EULER_ZYX);
  EXPECT_TRUE(tfRoundTrip.translation().isApprox(tf.translation(), 1e-12));
}

TEST(ZeroDofJoint, WeldJointConstraintImpulsePaths)
{
  auto skeleton = Skeleton::create("weld_joint_test");
  auto pair = skeleton->createJointAndBodyNodePair<WeldJoint>();
  auto* weldJoint = pair.first;
  ASSERT_NE(weldJoint, nullptr);

  weldJoint->setConstraintImpulse(0, 1.0);
  EXPECT_DOUBLE_EQ(weldJoint->getConstraintImpulse(0), 0.0);
  weldJoint->resetConstraintImpulses();

  Eigen::VectorXd q0;
  Eigen::VectorXd v;
  Eigen::VectorXd result;
  weldJoint->integratePositions(q0, v, 0.01, result);
  EXPECT_EQ(result.size(), 0);

  const Eigen::Vector6d wrench = weldJoint->getBodyConstraintWrench();
  EXPECT_TRUE(wrench.array().isFinite().all());
}

TEST(SoftBodyNode, AggregateAugmentedMassMatrixViaWorldStep)
{
  auto world = dart::simulation::World::create();
  world->setTimeStep(0.001);
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  auto skeleton = Skeleton::create("soft_body_world");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;
  ASSERT_NE(softBody, nullptr);

  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d(0.2, 0.2, 0.2),
      Eigen::Isometry3d::Identity(),
      1.0,
      10.0,
      10.0,
      0.1);

  world->addSkeleton(skeleton);

  for (int i = 0; i < 5; ++i) {
    world->step();
  }

  const Eigen::MatrixXd augMass = skeleton->getAugMassMatrix();
  EXPECT_EQ(augMass.rows(), static_cast<int>(skeleton->getNumDofs()));
  EXPECT_TRUE(augMass.array().isFinite().all());
}

TEST(BodyNodeShapeNodes, DeprecatedGetShapeNodes)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  auto sphere = std::make_shared<SphereShape>(0.5);

  body->createShapeNodeWith<VisualAspect>(box);
  body->createShapeNodeWith<CollisionAspect>(sphere);

  DART_SUPPRESS_DEPRECATED_BEGIN
  auto shapeNodes = body->getShapeNodes();
  auto visualNodes = body->getShapeNodesWith<VisualAspect>();
  const BodyNode* constBody = body;
  auto constShapeNodes = constBody->getShapeNodes();
  auto constCollisionNodes = constBody->getShapeNodesWith<CollisionAspect>();
  DART_SUPPRESS_DEPRECATED_END

  EXPECT_EQ(shapeNodes.size(), 2u);
  EXPECT_EQ(visualNodes.size(), 1u);
  EXPECT_EQ(constShapeNodes.size(), 2u);
  EXPECT_EQ(constCollisionNodes.size(), 1u);
}

TEST(BodyNodeShapeNodes, RemoveShapeNodeReorders)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(0.2, 0.3, 0.4));
  auto sphere = std::make_shared<SphereShape>(0.15);
  auto cylinder = std::make_shared<CylinderShape>(0.1, 0.4);

  ShapeNode* node0 = body->createShapeNodeWith<VisualAspect>(box);
  ShapeNode* node1 = body->createShapeNodeWith<CollisionAspect>(sphere);
  ShapeNode* node2
      = body->createShapeNodeWith<VisualAspect, CollisionAspect>(cylinder);

  ASSERT_EQ(body->getNumShapeNodes(), 3u);

  node1->remove();
  EXPECT_EQ(body->getNumShapeNodes(), 2u);

  bool found0 = false;
  bool found2 = false;
  for (std::size_t i = 0; i < body->getNumShapeNodes(); ++i) {
    ShapeNode* current = body->getShapeNode(i);
    EXPECT_NE(current, node1);
    found0 = found0 || (current == node0);
    found2 = found2 || (current == node2);
  }
  EXPECT_TRUE(found0);
  EXPECT_TRUE(found2);
}

TEST(BodyNodeForces, ClearExternalForcesResets)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  body->addExtForce(
      Eigen::Vector3d(0.5, -0.2, 0.1), Eigen::Vector3d::Zero(), true, true);
  EXPECT_GT(body->getExternalForceLocal().norm(), 0.0);

  body->clearExternalForces();
  EXPECT_TRUE(body->getExternalForceLocal().isZero());
}

TEST(BodyNodeConstraintImpulse, ClearConstraintImpulseResetsState)
{
  auto skeleton = createBodyNodeSkeleton();
  BodyNode* body = skeleton->getBodyNode(0);

  body->setConstraintImpulse(Eigen::Vector6d::Ones());
  EXPECT_FALSE(body->getConstraintImpulse().isZero());

  body->clearConstraintImpulse();
  EXPECT_TRUE(body->getConstraintImpulse().isZero());
  EXPECT_TRUE(body->getBodyVelocityChange().isZero());
}

TEST(BodyNodeImpulse, BiasImpulseViaImpulseForwardDynamics)
{
  auto skeleton = Skeleton::create("impulse_bias_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());
  pair.second->setMass(1.0);

  pair.second->setConstraintImpulse(Eigen::Vector6d::Ones());
  skeleton->computeImpulseForwardDynamics();

  const Eigen::Vector6d& delV = pair.second->getBodyVelocityChange();
  EXPECT_TRUE(delV.array().isFinite().all());
}

TEST(BodyNodeTransform, WorldTransformDerivativeOutOfBounds)
{
  auto skeleton = Skeleton::create("transform_deriv_oob");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());
  pair.second->setMass(1.0);

  const std::size_t oobIndex = pair.second->getNumDependentGenCoords();
#ifndef NDEBUG
  EXPECT_DEATH(
      { pair.second->getWorldTransformDerivative(oobIndex); }, "Assertion");
#else
  const auto& deriv = pair.second->getWorldTransformDerivative(oobIndex);
  EXPECT_TRUE(deriv.isZero());
#endif
}
