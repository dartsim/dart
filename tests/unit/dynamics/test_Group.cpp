/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the BSD-style License.
 */

#include "helpers/dynamics_helpers.hpp"

#include <dart/dynamics/Group.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::dynamics;

//==============================================================================
TEST(GroupTest, CreateEmptyGroup)
{
  auto group = Group::create("empty_group");

  EXPECT_EQ(group->getName(), "empty_group");
  EXPECT_EQ(group->getNumBodyNodes(), 0u);
  EXPECT_EQ(group->getNumJoints(), 0u);
  EXPECT_EQ(group->getNumDofs(), 0u);
}

//==============================================================================
TEST(GroupTest, CreateGroupFromBodyNodes)
{
  auto skel = createNLinkRobot(3, Eigen::Vector3d(0.1, 0.1, 0.5), DOF_PITCH);

  std::vector<BodyNode*> bodyNodes;
  for (std::size_t i = 0; i < skel->getNumBodyNodes(); ++i) {
    bodyNodes.push_back(skel->getBodyNode(i));
  }

  auto group = Group::create("test_group", bodyNodes);

  EXPECT_EQ(group->getNumBodyNodes(), skel->getNumBodyNodes());
  EXPECT_EQ(group->getNumJoints(), skel->getNumJoints());
  EXPECT_EQ(group->getNumDofs(), skel->getNumDofs());
}

//==============================================================================
TEST(GroupTest, CreateGroupFromMetaSkeleton)
{
  auto skel = createNLinkRobot(3, Eigen::Vector3d(0.1, 0.1, 0.5), DOF_PITCH);

  auto group = Group::create("clone_group", skel);

  EXPECT_EQ(group->getNumBodyNodes(), skel->getNumBodyNodes());
  EXPECT_EQ(group->getNumJoints(), skel->getNumJoints());
  EXPECT_EQ(group->getNumDofs(), skel->getNumDofs());
}

//==============================================================================
TEST(GroupTest, AddBodyNode)
{
  auto skel = createNLinkRobot(3, Eigen::Vector3d(0.1, 0.1, 0.5), DOF_PITCH);
  auto group = Group::create("test_group");

  EXPECT_EQ(group->getNumBodyNodes(), 0u);

  bool result = group->addBodyNode(skel->getBodyNode(0), false);
  EXPECT_TRUE(result);
  EXPECT_EQ(group->getNumBodyNodes(), 1u);

  result = group->addBodyNode(skel->getBodyNode(0), false);
  EXPECT_FALSE(result);
  EXPECT_EQ(group->getNumBodyNodes(), 1u);
}

//==============================================================================
TEST(GroupTest, RemoveBodyNode)
{
  auto skel = createNLinkRobot(3, Eigen::Vector3d(0.1, 0.1, 0.5), DOF_PITCH);
  auto group = Group::create("test_group");

  group->addBodyNode(skel->getBodyNode(0), false);
  EXPECT_EQ(group->getNumBodyNodes(), 1u);

  bool result = group->removeBodyNode(skel->getBodyNode(0), false);
  EXPECT_TRUE(result);
  EXPECT_EQ(group->getNumBodyNodes(), 0u);

  result = group->removeBodyNode(skel->getBodyNode(0), false);
  EXPECT_FALSE(result);
}

//==============================================================================
TEST(GroupTest, AddJoint)
{
  auto skel = createNLinkRobot(3, Eigen::Vector3d(0.1, 0.1, 0.5), DOF_PITCH);
  auto group = Group::create("test_group");

  bool result = group->addJoint(skel->getJoint(0), false, false);
  EXPECT_TRUE(result);
  EXPECT_EQ(group->getNumJoints(), 1u);

  result = group->addJoint(skel->getJoint(0), false, false);
  EXPECT_FALSE(result);
  EXPECT_EQ(group->getNumJoints(), 1u);
}

//==============================================================================
TEST(GroupTest, AddJointWithDofs)
{
  auto skel = createNLinkRobot(3, Eigen::Vector3d(0.1, 0.1, 0.5), DOF_PITCH);
  auto group = Group::create("test_group");

  bool result = group->addJoint(skel->getJoint(0), true, false);
  EXPECT_TRUE(result);
  EXPECT_EQ(group->getNumJoints(), 1u);
  EXPECT_EQ(group->getNumDofs(), skel->getJoint(0)->getNumDofs());
}

//==============================================================================
TEST(GroupTest, RemoveJoint)
{
  auto skel = createNLinkRobot(3, Eigen::Vector3d(0.1, 0.1, 0.5), DOF_PITCH);
  auto group = Group::create("test_group");

  group->addJoint(skel->getJoint(0), true, false);
  EXPECT_EQ(group->getNumJoints(), 1u);

  bool result = group->removeJoint(skel->getJoint(0), true, false);
  EXPECT_TRUE(result);
  EXPECT_EQ(group->getNumJoints(), 0u);
}

//==============================================================================
TEST(GroupTest, AddDof)
{
  auto skel = createNLinkRobot(3, Eigen::Vector3d(0.1, 0.1, 0.5), DOF_PITCH);
  auto group = Group::create("test_group");

  bool result = group->addDof(skel->getDof(0), false, false);
  EXPECT_TRUE(result);
  EXPECT_EQ(group->getNumDofs(), 1u);

  result = group->addDof(skel->getDof(0), false, false);
  EXPECT_FALSE(result);
  EXPECT_EQ(group->getNumDofs(), 1u);
}

//==============================================================================
TEST(GroupTest, RemoveDof)
{
  auto skel = createNLinkRobot(3, Eigen::Vector3d(0.1, 0.1, 0.5), DOF_PITCH);
  auto group = Group::create("test_group");

  group->addDof(skel->getDof(0), false, false);
  EXPECT_EQ(group->getNumDofs(), 1u);

  bool result = group->removeDof(skel->getDof(0), false, false);
  EXPECT_TRUE(result);
  EXPECT_EQ(group->getNumDofs(), 0u);
}

//==============================================================================
TEST(GroupTest, AddComponent)
{
  auto skel = createNLinkRobot(3, Eigen::Vector3d(0.1, 0.1, 0.5), DOF_PITCH);
  auto group = Group::create("test_group");

  bool result = group->addComponent(skel->getBodyNode(0), false);
  EXPECT_TRUE(result);
  EXPECT_EQ(group->getNumBodyNodes(), 1u);
  EXPECT_GE(group->getNumDofs(), 0u);
}

//==============================================================================
TEST(GroupTest, RemoveComponent)
{
  auto skel = createNLinkRobot(3, Eigen::Vector3d(0.1, 0.1, 0.5), DOF_PITCH);

  std::vector<BodyNode*> bodyNodes = {skel->getBodyNode(0)};
  auto group = Group::create("test_group", bodyNodes);

  std::size_t initialBodies = group->getNumBodyNodes();
  bool result = group->removeComponent(skel->getBodyNode(0), false);
  EXPECT_TRUE(result);
  EXPECT_LT(group->getNumBodyNodes(), initialBodies);
}

//==============================================================================
TEST(GroupTest, CloneGroup)
{
  auto skel = createNLinkRobot(3, Eigen::Vector3d(0.1, 0.1, 0.5), DOF_PITCH);
  auto group = Group::create("original", skel);

  auto cloned = group->cloneGroup();

  EXPECT_NE(cloned.get(), group.get());
  EXPECT_EQ(cloned->getNumBodyNodes(), group->getNumBodyNodes());
  EXPECT_EQ(cloned->getNumJoints(), group->getNumJoints());
  EXPECT_EQ(cloned->getNumDofs(), group->getNumDofs());
}

//==============================================================================
TEST(GroupTest, CloneGroupWithName)
{
  auto skel = createNLinkRobot(3, Eigen::Vector3d(0.1, 0.1, 0.5), DOF_PITCH);
  auto group = Group::create("original", skel);

  auto cloned = group->cloneGroup("cloned_name");

  EXPECT_EQ(cloned->getName(), "cloned_name");
}

//==============================================================================
TEST(GroupTest, CloneMetaSkeleton)
{
  auto skel = createNLinkRobot(3, Eigen::Vector3d(0.1, 0.1, 0.5), DOF_PITCH);
  auto group = Group::create("original", skel);

  auto cloned = group->cloneMetaSkeleton("meta_clone");

  EXPECT_NE(cloned, nullptr);
  EXPECT_EQ(cloned->getName(), "meta_clone");
}

//==============================================================================
TEST(GroupTest, SwapBodyNodeIndices)
{
  auto skel = createNLinkRobot(3, Eigen::Vector3d(0.1, 0.1, 0.5), DOF_PITCH);
  auto group = Group::create("test_group", skel);

  ASSERT_GE(group->getNumBodyNodes(), 2u);

  auto body0 = group->getBodyNode(0);
  auto body1 = group->getBodyNode(1);

  group->swapBodyNodeIndices(0, 1);

  EXPECT_EQ(group->getBodyNode(0), body1);
  EXPECT_EQ(group->getBodyNode(1), body0);
}

//==============================================================================
TEST(GroupTest, SwapDofIndices)
{
  auto skel = createNLinkRobot(3, Eigen::Vector3d(0.1, 0.1, 0.5), DOF_PITCH);
  auto group = Group::create("test_group", skel);

  ASSERT_GE(group->getNumDofs(), 2u);

  auto dof0 = group->getDof(0);
  auto dof1 = group->getDof(1);

  group->swapDofIndices(0, 1);

  EXPECT_EQ(group->getDof(0), dof1);
  EXPECT_EQ(group->getDof(1), dof0);
}

//==============================================================================
TEST(GroupTest, AddBodyNodesSpan)
{
  auto skel = createNLinkRobot(3, Eigen::Vector3d(0.1, 0.1, 0.5), DOF_PITCH);
  auto group = Group::create("test_group");

  std::vector<BodyNode*> bodyNodes;
  for (std::size_t i = 0; i < skel->getNumBodyNodes(); ++i) {
    bodyNodes.push_back(skel->getBodyNode(i));
  }

  bool result = group->addBodyNodes(bodyNodes, false);
  EXPECT_TRUE(result);
  EXPECT_EQ(group->getNumBodyNodes(), skel->getNumBodyNodes());
}

//==============================================================================
TEST(GroupTest, RemoveBodyNodesSpan)
{
  auto skel = createNLinkRobot(3, Eigen::Vector3d(0.1, 0.1, 0.5), DOF_PITCH);
  auto group = Group::create("test_group", skel);

  std::vector<BodyNode*> bodyNodes;
  for (std::size_t i = 0; i < skel->getNumBodyNodes(); ++i) {
    bodyNodes.push_back(skel->getBodyNode(i));
  }

  bool result = group->removeBodyNodes(bodyNodes, false);
  EXPECT_TRUE(result);
  EXPECT_EQ(group->getNumBodyNodes(), 0u);
}

//==============================================================================
TEST(GroupTest, AddJointsSpan)
{
  auto skel = createNLinkRobot(3, Eigen::Vector3d(0.1, 0.1, 0.5), DOF_PITCH);
  auto group = Group::create("test_group");

  std::vector<Joint*> joints;
  for (std::size_t i = 0; i < skel->getNumJoints(); ++i) {
    joints.push_back(skel->getJoint(i));
  }

  bool result = group->addJoints(joints, false, false);
  EXPECT_TRUE(result);
  EXPECT_EQ(group->getNumJoints(), skel->getNumJoints());
}

//==============================================================================
TEST(GroupTest, AddDofsSpan)
{
  auto skel = createNLinkRobot(3, Eigen::Vector3d(0.1, 0.1, 0.5), DOF_PITCH);
  auto group = Group::create("test_group");

  std::vector<DegreeOfFreedom*> dofs;
  for (std::size_t i = 0; i < skel->getNumDofs(); ++i) {
    dofs.push_back(skel->getDof(i));
  }

  bool result = group->addDofs(dofs, false, false);
  EXPECT_TRUE(result);
  EXPECT_EQ(group->getNumDofs(), skel->getNumDofs());
}

//==============================================================================
TEST(GroupTest, CreateFromDofsSpan)
{
  auto skel = createNLinkRobot(3, Eigen::Vector3d(0.1, 0.1, 0.5), DOF_PITCH);

  std::vector<DegreeOfFreedom*> dofs;
  for (std::size_t i = 0; i < skel->getNumDofs(); ++i) {
    dofs.push_back(skel->getDof(i));
  }

  auto group = Group::create("dof_group", dofs);

  EXPECT_EQ(group->getNumDofs(), skel->getNumDofs());
}
