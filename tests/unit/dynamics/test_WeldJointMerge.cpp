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

#include "helpers/GTestUtils.hpp"

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::dynamics;

//==============================================================================
TEST(WeldJointMerge, FlattensChildIntoParent)
{
  // Diagram (topology before merge):
  //   root/free -> parent --[weld]--> child --[rev]--> grandchild
  //   (shape lives on child; grandchild keeps its pose)
  auto skel = Skeleton::create("merge");

  auto rootPair = skel->createJointAndBodyNodePair<FreeJoint>();
  auto* rootJoint = rootPair.first;
  auto* parent = rootPair.second;
  rootJoint->setName("root");
  parent->setName("parent");
  parent->setInertia(
      Inertia(1.0, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity()));

  WeldJoint::Properties weldProps;
  Eigen::Isometry3d parentToChild = Eigen::Isometry3d::Identity();
  parentToChild.translation() = Eigen::Vector3d(0.2, -0.1, 0.3);
  weldProps.mT_ParentBodyToJoint = parentToChild;
  auto weldPair
      = skel->createJointAndBodyNodePair<WeldJoint>(parent, weldProps);
  auto* weld = weldPair.first;
  auto* child = weldPair.second;
  weld->setName("weld");
  child->setName("child");
  weld->setTransformFromParentBodyNode(parentToChild);
  child->setInertia(Inertia(
      2.0,
      Eigen::Vector3d(0.05, 0.0, 0.02),
      Eigen::Matrix3d::Identity() * 0.5));

  const auto shape = std::make_shared<BoxShape>(Eigen::Vector3d(0.1, 0.2, 0.3));
  auto* childShape = child->createShapeNodeWith<VisualAspect>(shape);
  Eigen::Isometry3d shapeRel = Eigen::Isometry3d::Identity();
  shapeRel.translation() = Eigen::Vector3d(0.05, 0.02, -0.03);
  childShape->setRelativeTransform(shapeRel);

  RevoluteJoint::Properties revProps;
  revProps.mAxis = Eigen::Vector3d::UnitZ();
  Eigen::Isometry3d childToGrandJoint = Eigen::Isometry3d::Identity();
  childToGrandJoint.translation() = Eigen::Vector3d(-0.15, 0.04, 0.0);
  revProps.mT_ParentBodyToJoint = childToGrandJoint;
  auto grandPair
      = child->createChildJointAndBodyNodePair<RevoluteJoint>(revProps);
  auto* grandJoint = grandPair.first;
  auto* grandchild = grandPair.second;
  grandchild->setName("grandchild");
  grandJoint->setPosition(0, 0.4);

  const auto preMergeShapeWorld = childShape->getWorldTransform();
  const auto preMergeGrandWorld = grandchild->getWorldTransform();

  ASSERT_EQ(3u, skel->getNumBodyNodes());
  ASSERT_EQ(3u, skel->getNumJoints());

  BodyNode* merged = weld->merge();
  ASSERT_EQ(parent, merged);

  ASSERT_EQ(2u, skel->getNumBodyNodes());
  ASSERT_EQ(2u, skel->getNumJoints());
  EXPECT_DOUBLE_EQ(3.0, parent->getMass());

  ASSERT_EQ(1u, parent->getNumShapeNodes());
  auto* mergedShape = parent->getShapeNode(0);
  ASSERT_NE(nullptr, mergedShape);
  EXPECT_MATRIX_DOUBLE_EQ(
      preMergeShapeWorld.matrix(), mergedShape->getWorldTransform().matrix());

  EXPECT_MATRIX_DOUBLE_EQ(
      preMergeGrandWorld.matrix(), grandchild->getWorldTransform().matrix());
  EXPECT_EQ(parent, grandchild->getParentBodyNode());
}
