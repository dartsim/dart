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

#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/group.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <dart/common/deprecated.hpp>

#include <gtest/gtest.h>

using namespace dart::dynamics;

//==============================================================================
TEST(SkeletonAccessors, ReturnsMutableBodyNodeVector)
{
  auto skeleton = Skeleton::create("skeleton");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto* body = pair.second;

  DART_SUPPRESS_DEPRECATED_BEGIN
  auto& nodes = skeleton->getBodyNodes();
  DART_SUPPRESS_DEPRECATED_END
  ASSERT_EQ(nodes.size(), 1u);
  EXPECT_EQ(nodes.front(), body);
}

//==============================================================================
TEST(ReferentialSkeletonAccessors, ReturnsMutableBodyNodeVector)
{
  auto skeleton = Skeleton::create("skeleton");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();

  auto group = Group::create("group");
  ASSERT_TRUE(group->addComponent(pair.second));

  DART_SUPPRESS_DEPRECATED_BEGIN
  auto& nodes = group->getBodyNodes();
  DART_SUPPRESS_DEPRECATED_END
  ASSERT_EQ(nodes.size(), 1u);
  EXPECT_EQ(nodes.front(), pair.second);
}

//==============================================================================
TEST(SkeletonAccessors, DofsLimitsAndMassMatrix)
{
  auto skeleton = Skeleton::create("skeleton_ops");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  rootPair.first->setName("root_joint");
  rootPair.second->setName("root_body");

  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  childPair.first->setName("child_joint");
  childPair.first->setAxis(Eigen::Vector3d::UnitY());
  childPair.second->setName("child_body");

  EXPECT_EQ(skeleton->getNumBodyNodes(), 2u);
  EXPECT_EQ(skeleton->getNumJoints(), 2u);
  EXPECT_EQ(
      skeleton->getNumDofs(),
      rootPair.first->getNumDofs() + childPair.first->getNumDofs());

  EXPECT_EQ(skeleton->getBodyNode("root_body"), rootPair.second);
  EXPECT_EQ(skeleton->getJoint("child_joint"), childPair.first);

  const auto dofs = skeleton->getNumDofs();
  Eigen::VectorXd positions = Eigen::VectorXd::LinSpaced(
      static_cast<int>(dofs), 0.1, 0.1 + static_cast<double>(dofs - 1));
  skeleton->setPositions(positions);
  EXPECT_TRUE(skeleton->getPositions().isApprox(positions));

  Eigen::VectorXd limits = Eigen::VectorXd::Constant(dofs, 1.0);
  skeleton->setPositionLowerLimits(-limits);
  skeleton->setPositionUpperLimits(limits);
  EXPECT_TRUE(skeleton->getPositionLowerLimits().isApprox(-limits));
  EXPECT_TRUE(skeleton->getPositionUpperLimits().isApprox(limits));

  skeleton->setVelocityLowerLimits(-limits);
  skeleton->setVelocityUpperLimits(limits);
  EXPECT_TRUE(skeleton->getVelocityLowerLimits().isApprox(-limits));
  EXPECT_TRUE(skeleton->getVelocityUpperLimits().isApprox(limits));

  skeleton->computeForwardDynamics();
  skeleton->computeInverseDynamics(true, true, true);

  const auto massMatrix = skeleton->getMassMatrix();
  EXPECT_EQ(massMatrix.rows(), static_cast<int>(dofs));
  EXPECT_EQ(massMatrix.cols(), static_cast<int>(dofs));

  const auto com = skeleton->getCOM();
  EXPECT_TRUE(com.array().isFinite().all());
}

//==============================================================================
TEST(SkeletonAccessors, TreeAccessors)
{
  // Create skeleton with single tree
  auto skeleton = Skeleton::create("tree_test");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  rootPair.second->setName("root");

  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  childPair.second->setName("child");

  // Single tree skeleton
  EXPECT_EQ(skeleton->getNumTrees(), 1u);
  EXPECT_EQ(skeleton->getRootBodyNode(0), rootPair.second);

  // Get tree body nodes
  auto treeNodes = skeleton->getTreeBodyNodes(0);
  EXPECT_EQ(treeNodes.size(), 2u);

  // Get tree DOFs
  auto treeDofs = skeleton->getTreeDofs(0);
  EXPECT_EQ(treeDofs.size(), skeleton->getNumDofs());
}

//==============================================================================
TEST(SkeletonAccessors, ComputeForwardDynamicsNoArgs)
{
  auto skeleton = Skeleton::create("fwd_dynamics");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  // Set some velocities
  Eigen::VectorXd velocities = Eigen::VectorXd::Ones(skeleton->getNumDofs());
  skeleton->setVelocities(velocities);

  // Compute forward dynamics (no arguments version)
  skeleton->computeForwardDynamics();

  // Accelerations should be computed and finite
  const auto accelerations = skeleton->getAccelerations();
  EXPECT_EQ(accelerations.size(), static_cast<int>(skeleton->getNumDofs()));
  EXPECT_TRUE(accelerations.array().isFinite().all());
}

//==============================================================================
TEST(SkeletonAccessors, IntegratePositions)
{
  auto skeleton = Skeleton::create("integrate_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());

  // Set initial position and velocity
  skeleton->setPosition(0, 0.0);
  skeleton->setVelocity(0, 1.0); // 1 rad/s

  const double dt = 0.1;
  const double initialPos = skeleton->getPosition(0);

  // Integrate positions
  skeleton->integratePositions(dt);

  // Position should have changed by velocity * dt
  const double expectedPos = initialPos + 1.0 * dt;
  EXPECT_NEAR(skeleton->getPosition(0), expectedPos, 1e-10);
}

//==============================================================================
TEST(SkeletonAccessors, ClearConstraintImpulses)
{
  auto skeleton = Skeleton::create("impulse_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());

  skeleton->setJointConstraintImpulses(
      Eigen::VectorXd::Ones(skeleton->getNumDofs()));

  skeleton->clearConstraintImpulses();

  const auto impulses = skeleton->getJointConstraintImpulses();
  EXPECT_TRUE(impulses.isZero());
}

//==============================================================================
TEST(SkeletonAccessors, CenterOfMassAccessors)
{
  auto skeleton = Skeleton::create("com_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(2.0);

  // Set some velocity
  Eigen::VectorXd velocities = Eigen::VectorXd::Zero(skeleton->getNumDofs());
  velocities[3] = 1.0; // Linear velocity in x
  skeleton->setVelocities(velocities);

  // Get COM
  const auto com = skeleton->getCOM();
  EXPECT_TRUE(com.array().isFinite().all());

  // Get COM linear velocity
  const auto comVel = skeleton->getCOMLinearVelocity();
  EXPECT_TRUE(comVel.array().isFinite().all());

  // COM velocity should be non-zero since we set linear velocity
  EXPECT_GT(comVel.norm(), 0.0);
}

//==============================================================================
TEST(SkeletonAccessors, SupportPolygon)
{
  auto skeleton = Skeleton::create("support_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  // Get support polygon (may be empty for floating base without end effectors)
  const auto& polygon = skeleton->getSupportPolygon();

  // Just verify it doesn't crash and returns valid data structure
  // Empty polygon is valid for a skeleton without ground contact
  EXPECT_TRUE(polygon.empty() || polygon.size() >= 3u);

  // Also test tree-specific version
  const auto& treePolygon = skeleton->getSupportPolygon(0);
  EXPECT_TRUE(treePolygon.empty() || treePolygon.size() >= 3u);
}

//==============================================================================
TEST(SkeletonAccessors, IntegratePositionsWithVelocityChanges)
{
  auto skeleton = Skeleton::create("integrate_vel_change");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());

  // Set initial position and velocity
  skeleton->setPosition(0, 0.0);
  skeleton->setVelocity(0, 1.0);

  const double dt = 0.1;
  Eigen::VectorXd velocityChanges
      = Eigen::VectorXd::Ones(skeleton->getNumDofs());
  velocityChanges[0] = 0.5; // Additional velocity change

  const double initialPos = skeleton->getPosition(0);

  // Integrate with velocity changes
  skeleton->integratePositions(dt, velocityChanges);

  // Position should reflect both original velocity and velocity change
  // Expected: initialPos + (velocity + velocityChange) * dt
  const double expectedPos = initialPos + (1.0 + 0.5) * dt;
  EXPECT_NEAR(skeleton->getPosition(0), expectedPos, 1e-10);
}
