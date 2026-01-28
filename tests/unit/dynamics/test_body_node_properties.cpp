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

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <optional>

using namespace dart::dynamics;

static SkeletonPtr createBodyNodeSkeleton()
{
  auto skeleton = Skeleton::create("body_props");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setName("body");
  return skeleton;
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
