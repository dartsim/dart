/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the BSD-style License.
 */

#include "helpers/dynamics_helpers.hpp"

#include <dart/dynamics/chain.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/group.hpp>
#include <dart/dynamics/linkage.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <dart/all.hpp>

#include <gtest/gtest.h>

#include <vector>

#include <cmath>

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
TEST(GroupTest, ConstAccessors)
{
  auto skel = createNLinkRobot(2, Eigen::Vector3d(0.1, 0.1, 0.5), DOF_PITCH);
  auto group = Group::create("const_group", skel);

  const Group* constGroup = group.get();
  EXPECT_EQ(constGroup->getName(), "const_group");
  EXPECT_EQ(constGroup->getNumBodyNodes(), group->getNumBodyNodes());
  EXPECT_EQ(constGroup->getNumJoints(), group->getNumJoints());
  EXPECT_EQ(constGroup->getNumDofs(), group->getNumDofs());

  EXPECT_EQ(constGroup->getNumSkeletons(), 1u);
  EXPECT_TRUE(constGroup->hasSkeleton(skel.get()));

  const auto bodyNodes = constGroup->getBodyNodes();
  EXPECT_EQ(bodyNodes.size(), group->getNumBodyNodes());
  const auto joints = constGroup->getJoints();
  EXPECT_EQ(joints.size(), group->getNumJoints());
  const auto dofs = constGroup->getDofs();
  EXPECT_EQ(dofs.size(), group->getNumDofs());

  const auto* bodyNode = skel->getBodyNode(0);
  const auto* joint = skel->getJoint(0);
  const auto* dof = skel->getDof(0);
  const auto bodyName = bodyNode->getName();
  const auto jointName = joint->getName();

  EXPECT_EQ(constGroup->getBodyNode(0), bodyNode);
  EXPECT_EQ(constGroup->getBodyNode(bodyName), bodyNode);
  EXPECT_EQ(constGroup->getJoint(0), joint);
  EXPECT_EQ(constGroup->getJoint(jointName), joint);
  EXPECT_EQ(constGroup->getDof(0), dof);

  EXPECT_EQ(constGroup->getBodyNodes(bodyName).size(), 1u);
  EXPECT_EQ(constGroup->getJoints(jointName).size(), 1u);

  EXPECT_EQ(constGroup->getIndexOf(bodyNode, false), 0u);
  EXPECT_EQ(constGroup->getIndexOf(joint, false), 0u);
  EXPECT_EQ(constGroup->getIndexOf(dof, false), 0u);
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

namespace {

void setUniformInertia(const SkeletonPtr& skeleton, double mass)
{
  for (std::size_t i = 0; i < skeleton->getNumBodyNodes(); ++i) {
    dart::dynamics::Inertia inertia;
    inertia.setMass(mass);
    inertia.setMoment(0.1, 0.1, 0.1, 0.0, 0.0, 0.0);
    skeleton->getBodyNode(i)->setInertia(inertia);
  }
}

} // namespace

//==============================================================================
TEST(GroupTest, MetaSkeletonInterface)
{
  auto skel = createNLinkRobot(4, Eigen::Vector3d(0.2, 0.1, 0.3), DOF_PITCH);
  setUniformInertia(skel, 1.0);

  std::vector<BodyNode*> bodyNodes
      = {skel->getBodyNode(1), skel->getBodyNode(2), skel->getBodyNode(3)};
  auto group = Group::create("meta_group", bodyNodes, true, true);

  EXPECT_EQ(group->getNumBodyNodes(), bodyNodes.size());
  EXPECT_EQ(group->getBodyNode(0), bodyNodes[0]);
  EXPECT_EQ(group->getBodyNode(bodyNodes[0]->getName()), bodyNodes[0]);
  EXPECT_EQ(group->getBodyNodes(bodyNodes[0]->getName()).size(), 1u);
  EXPECT_TRUE(group->hasBodyNode(bodyNodes[1]));
  EXPECT_NE(group->getIndexOf(bodyNodes[1], false), INVALID_INDEX);

  EXPECT_EQ(group->getNumJoints(), group->getJoints().size());
  auto* joint0 = group->getJoint(0);
  ASSERT_NE(joint0, nullptr);
  EXPECT_EQ(group->getJoint(joint0->getName()), joint0);
  EXPECT_EQ(group->getJoints(joint0->getName()).size(), 1u);
  EXPECT_TRUE(group->hasJoint(joint0));
  EXPECT_NE(group->getIndexOf(joint0, false), INVALID_INDEX);

  auto* dof0 = group->getDof(0);
  ASSERT_NE(dof0, nullptr);
  EXPECT_NE(group->getIndexOf(dof0, false), INVALID_INDEX);
  EXPECT_EQ(group->getDofs().size(), group->getNumDofs());
  EXPECT_EQ(
      static_cast<const Group&>(*group).getDofs().size(), group->getNumDofs());

  group->setName("meta_group_updated");
  EXPECT_EQ(group->getName(), "meta_group_updated");

  Eigen::VectorXd positions
      = Eigen::VectorXd::LinSpaced(group->getNumDofs(), 0.1, 0.4);
  group->setPositions(positions);
  EXPECT_TRUE(group->getPositions().isApprox(positions));

  Eigen::VectorXd velocities
      = Eigen::VectorXd::Constant(group->getNumDofs(), -0.2);
  group->setVelocities(velocities);
  EXPECT_TRUE(group->getVelocities().isApprox(velocities));

  group->setPositionLowerLimit(0, -1.0);
  group->setPositionUpperLimit(0, 1.0);
  EXPECT_DOUBLE_EQ(group->getPositionLowerLimit(0), -1.0);
  EXPECT_DOUBLE_EQ(group->getPositionUpperLimit(0), 1.0);

  Eigen::VectorXd posLower
      = Eigen::VectorXd::Constant(group->getNumDofs(), -0.5);
  Eigen::VectorXd posUpper
      = Eigen::VectorXd::Constant(group->getNumDofs(), 0.5);
  group->setPositionLowerLimits(posLower);
  group->setPositionUpperLimits(posUpper);
  EXPECT_TRUE(group->getPositionLowerLimits().isApprox(posLower));
  EXPECT_TRUE(group->getPositionUpperLimits().isApprox(posUpper));

  group->setVelocityLowerLimit(0, -2.0);
  group->setVelocityUpperLimit(0, 2.0);
  EXPECT_DOUBLE_EQ(group->getVelocityLowerLimit(0), -2.0);
  EXPECT_DOUBLE_EQ(group->getVelocityUpperLimit(0), 2.0);

  Eigen::VectorXd velLower
      = Eigen::VectorXd::Constant(group->getNumDofs(), -3.0);
  Eigen::VectorXd velUpper
      = Eigen::VectorXd::Constant(group->getNumDofs(), 3.0);
  group->setVelocityLowerLimits(velLower);
  group->setVelocityUpperLimits(velUpper);
  EXPECT_TRUE(group->getVelocityLowerLimits().isApprox(velLower));
  EXPECT_TRUE(group->getVelocityUpperLimits().isApprox(velUpper));

  group->setAccelerationLowerLimit(0, -4.0);
  group->setAccelerationUpperLimit(0, 4.0);
  EXPECT_DOUBLE_EQ(group->getAccelerationLowerLimit(0), -4.0);
  EXPECT_DOUBLE_EQ(group->getAccelerationUpperLimit(0), 4.0);

  Eigen::VectorXd accelLower
      = Eigen::VectorXd::Constant(group->getNumDofs(), -5.0);
  Eigen::VectorXd accelUpper
      = Eigen::VectorXd::Constant(group->getNumDofs(), 5.0);
  group->setAccelerationLowerLimits(accelLower);
  group->setAccelerationUpperLimits(accelUpper);
  EXPECT_TRUE(group->getAccelerationLowerLimits().isApprox(accelLower));
  EXPECT_TRUE(group->getAccelerationUpperLimits().isApprox(accelUpper));

  group->setForceLowerLimit(0, -6.0);
  group->setForceUpperLimit(0, 6.0);
  EXPECT_DOUBLE_EQ(group->getForceLowerLimit(0), -6.0);
  EXPECT_DOUBLE_EQ(group->getForceUpperLimit(0), 6.0);

  Eigen::VectorXd forceLower
      = Eigen::VectorXd::Constant(group->getNumDofs(), -7.0);
  Eigen::VectorXd forceUpper
      = Eigen::VectorXd::Constant(group->getNumDofs(), 7.0);
  group->setForceLowerLimits(forceLower);
  group->setForceUpperLimits(forceUpper);
  EXPECT_TRUE(group->getForceLowerLimits().isApprox(forceLower));
  EXPECT_TRUE(group->getForceUpperLimits().isApprox(forceUpper));

  const auto& massMatrix = group->getMassMatrix();
  EXPECT_EQ(massMatrix.rows(), static_cast<int>(group->getNumDofs()));
  EXPECT_EQ(massMatrix.cols(), static_cast<int>(group->getNumDofs()));
  EXPECT_EQ(group->getAugMassMatrix().rows(), massMatrix.rows());
  EXPECT_EQ(group->getInvMassMatrix().rows(), massMatrix.rows());
  EXPECT_EQ(group->getInvAugMassMatrix().rows(), massMatrix.rows());

  EXPECT_EQ(group->getCoriolisForces().size(), group->getNumDofs());
  EXPECT_EQ(group->getGravityForces().size(), group->getNumDofs());
  EXPECT_EQ(group->getCoriolisAndGravityForces().size(), group->getNumDofs());
  EXPECT_EQ(group->getExternalForces().size(), group->getNumDofs());
  EXPECT_EQ(group->getConstraintForces().size(), group->getNumDofs());

  const BodyNode* target = bodyNodes[1];
  const auto jac = group->getJacobian(target);
  EXPECT_EQ(jac.cols(), static_cast<int>(group->getNumDofs()));
  EXPECT_EQ(group->getJacobian(target, Frame::World()).cols(), jac.cols());
  EXPECT_EQ(
      group->getJacobian(target, Eigen::Vector3d::UnitX()).cols(), jac.cols());
  EXPECT_EQ(
      group->getJacobian(target, Eigen::Vector3d::UnitY(), Frame::World())
          .cols(),
      jac.cols());

  EXPECT_EQ(group->getWorldJacobian(target).cols(), jac.cols());
  EXPECT_EQ(
      group->getWorldJacobian(target, Eigen::Vector3d::UnitZ()).cols(),
      jac.cols());

  EXPECT_EQ(group->getLinearJacobian(target).cols(), jac.cols());
  EXPECT_EQ(
      group->getLinearJacobian(target, Eigen::Vector3d::UnitX(), Frame::World())
          .cols(),
      jac.cols());

  EXPECT_EQ(group->getAngularJacobian(target).cols(), jac.cols());

  EXPECT_EQ(group->getJacobianSpatialDeriv(target).cols(), jac.cols());
  EXPECT_EQ(
      group->getJacobianSpatialDeriv(target, Frame::World()).cols(),
      jac.cols());
  EXPECT_EQ(
      group->getJacobianSpatialDeriv(target, Eigen::Vector3d::UnitX()).cols(),
      jac.cols());
  EXPECT_EQ(
      group
          ->getJacobianSpatialDeriv(
              target, Eigen::Vector3d::UnitY(), Frame::World())
          .cols(),
      jac.cols());

  EXPECT_EQ(group->getJacobianClassicDeriv(target).cols(), jac.cols());
  EXPECT_EQ(
      group->getJacobianClassicDeriv(target, Frame::World()).cols(),
      jac.cols());
  EXPECT_EQ(
      group
          ->getJacobianClassicDeriv(
              target, Eigen::Vector3d::UnitZ(), Frame::World())
          .cols(),
      jac.cols());

  EXPECT_EQ(group->getLinearJacobianDeriv(target).cols(), jac.cols());
  EXPECT_EQ(
      group
          ->getLinearJacobianDeriv(
              target, Eigen::Vector3d::UnitY(), Frame::World())
          .cols(),
      jac.cols());
  EXPECT_EQ(group->getAngularJacobianDeriv(target).cols(), jac.cols());

  const auto com = group->getCOM();
  EXPECT_TRUE(com.array().isFinite().all());
  const auto comSpatialVel = group->getCOMSpatialVelocity();
  EXPECT_TRUE(comSpatialVel.array().isFinite().all());
  const auto comLinearVel = group->getCOMLinearVelocity();
  EXPECT_TRUE(comLinearVel.array().isFinite().all());
  const auto comSpatialAcc = group->getCOMSpatialAcceleration();
  EXPECT_TRUE(comSpatialAcc.array().isFinite().all());
  const auto comLinearAcc = group->getCOMLinearAcceleration();
  EXPECT_TRUE(comLinearAcc.array().isFinite().all());

  const auto comJac = group->getCOMJacobian();
  EXPECT_EQ(comJac.cols(), static_cast<int>(group->getNumDofs()));
  const auto comLinearJac = group->getCOMLinearJacobian();
  EXPECT_EQ(comLinearJac.cols(), static_cast<int>(group->getNumDofs()));
  const auto comJacDeriv = group->getCOMJacobianSpatialDeriv();
  EXPECT_EQ(comJacDeriv.cols(), static_cast<int>(group->getNumDofs()));
  const auto comLinearJacDeriv = group->getCOMLinearJacobianDeriv();
  EXPECT_EQ(comLinearJacDeriv.cols(), static_cast<int>(group->getNumDofs()));

  EXPECT_DOUBLE_EQ(group->getMass(), 3.0);
  EXPECT_GE(group->computeKineticEnergy(), 0.0);
  EXPECT_TRUE(std::isfinite(group->computePotentialEnergy()));

  group->clearExternalForces();
  group->clearInternalForces();
}

//==============================================================================
TEST(GroupTest, ChainAndLinkageCreation)
{
  auto skel = createNLinkRobot(4, Eigen::Vector3d(0.2, 0.1, 0.3), DOF_PITCH);
  setUniformInertia(skel, 1.0);

  BodyNode* start = skel->getBodyNode(0);
  BodyNode* end = skel->getBodyNode(3);
  auto chain = Chain::create(start, end, "coverage_chain");
  ASSERT_NE(chain, nullptr);
  EXPECT_EQ(chain->getName(), "coverage_chain");

  Linkage::Criteria criteria(start, end);
  auto linkage = Linkage::create(criteria, "coverage_linkage");
  ASSERT_NE(linkage, nullptr);
  EXPECT_EQ(linkage->getName(), "coverage_linkage");
}

//==============================================================================
TEST(GroupTest, AddNullComponentWithoutWarning)
{
  auto group = Group::create("null_component_group");

  EXPECT_FALSE(group->addComponent(nullptr, false));
}

//==============================================================================
TEST(GroupTest, ConstAccessorsAndMembershipQueries)
{
  auto skel = createNLinkRobot(3, Eigen::Vector3d(0.1, 0.1, 0.5), DOF_PITCH);
  std::vector<BodyNode*> bodyNodes;
  bodyNodes.reserve(skel->getNumBodyNodes());
  for (std::size_t i = 0; i < skel->getNumBodyNodes(); ++i) {
    bodyNodes.push_back(skel->getBodyNode(i));
  }

  auto group = Group::create("const_group", bodyNodes);
  const Group& constGroup = *group;

  EXPECT_EQ(constGroup.getNumBodyNodes(), skel->getNumBodyNodes());
  EXPECT_EQ(constGroup.getNumJoints(), skel->getNumJoints());
  EXPECT_EQ(constGroup.getNumDofs(), skel->getNumDofs());

  auto* body0 = skel->getBodyNode(0);
  auto* joint0 = skel->getJoint(0);
  auto* dof0 = skel->getDof(0);

  EXPECT_TRUE(constGroup.hasBodyNode(body0));
  EXPECT_TRUE(constGroup.hasJoint(joint0));
  EXPECT_NE(constGroup.getIndexOf(body0, false), INVALID_INDEX);
  EXPECT_NE(constGroup.getIndexOf(joint0, false), INVALID_INDEX);
  EXPECT_NE(constGroup.getIndexOf(dof0, false), INVALID_INDEX);
}

//==============================================================================
TEST(GroupTest, ConstOverloads)
{
  auto skel = Skeleton::create("group_const_overloads");
  auto rootPair = skel->createJointAndBodyNodePair<RevoluteJoint>();
  rootPair.first->setName("root_joint");
  rootPair.second->setName("root_body");

  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  childPair.first->setName("child_joint");
  childPair.second->setName("child_body");

  std::vector<BodyNode*> bodyNodes = {rootPair.second, childPair.second};
  auto group = Group::create("const_overloads", bodyNodes);

  const Group& constGroup = *group;

  EXPECT_EQ(constGroup.getNumSkeletons(), 1u);
  EXPECT_TRUE(constGroup.hasSkeleton(skel.get()));

  EXPECT_EQ(constGroup.getNumBodyNodes(), bodyNodes.size());
  EXPECT_EQ(constGroup.getBodyNode(0), rootPair.second);
  EXPECT_EQ(constGroup.getBodyNode("child_body"), childPair.second);
  EXPECT_EQ(constGroup.getBodyNodes("child_body").size(), 1u);
  EXPECT_EQ(constGroup.getBodyNodes().size(), bodyNodes.size());

  EXPECT_EQ(constGroup.getNumJoints(), 2u);
  EXPECT_EQ(constGroup.getJoint(0), rootPair.first);
  EXPECT_EQ(constGroup.getJoint("child_joint"), childPair.first);
  EXPECT_EQ(constGroup.getJoints("child_joint").size(), 1u);
  EXPECT_EQ(constGroup.getJoints().size(), constGroup.getNumJoints());

  const auto* dof0 = constGroup.getDof(0);
  ASSERT_NE(dof0, nullptr);
  EXPECT_NE(constGroup.getIndexOf(dof0, false), INVALID_INDEX);

  DART_SUPPRESS_DEPRECATED_BEGIN
  EXPECT_EQ(constGroup.getDofs().size(), constGroup.getNumDofs());
  DART_SUPPRESS_DEPRECATED_END
}
