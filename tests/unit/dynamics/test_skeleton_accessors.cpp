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

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/end_effector.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/group.hpp>
#include <dart/dynamics/prismatic_joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/common/deprecated.hpp>

#include <gtest/gtest.h>

#include <cmath>

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

//==============================================================================
// Mass Matrix Numerical Correctness Tests
//==============================================================================

static SkeletonPtr createSimplePendulum(double length, double mass)
{
  auto skeleton = Skeleton::create("pendulum");

  RevoluteJoint::Properties jointProps;
  jointProps.mName = "joint";
  jointProps.mAxis = Eigen::Vector3d::UnitZ();

  BodyNode::Properties bodyProps;
  bodyProps.mName = "body";
  bodyProps.mInertia.setMass(mass);
  bodyProps.mInertia.setLocalCOM(Eigen::Vector3d(length / 2.0, 0.0, 0.0));
  // Moment of inertia for point mass at distance length/2: I = m * (l/2)^2
  double I = mass * (length / 2.0) * (length / 2.0);
  bodyProps.mInertia.setMoment(I, I, I, 0, 0, 0);

  skeleton->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, jointProps, bodyProps);

  return skeleton;
}

TEST(SkeletonMassMatrix, SimplePendulumMassMatrixValue)
{
  const double length = 1.0;
  const double mass = 2.0;
  auto skeleton = createSimplePendulum(length, mass);

  skeleton->setPosition(0, 0.0);
  const auto massMatrix = skeleton->getMassMatrix();

  ASSERT_EQ(massMatrix.rows(), 1);
  ASSERT_EQ(massMatrix.cols(), 1);

  // For a point mass at distance l/2 from rotation axis:
  // M = m * (l/2)^2 + I_body
  // where I_body = m * (l/2)^2 (parallel axis theorem from COM)
  // Total = m * (l/2)^2 + m * (l/2)^2 = 2 * m * (l/2)^2 = m * l^2 / 2
  const double expectedMass = mass * length * length / 2.0;
  EXPECT_NEAR(massMatrix(0, 0), expectedMass, 1e-8);
}

TEST(SkeletonMassMatrix, MassMatrixSymmetry)
{
  auto skeleton = Skeleton::create("multi_dof");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  rootPair.second->setMass(1.0);

  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  childPair.first->setAxis(Eigen::Vector3d::UnitZ());
  childPair.second->setMass(0.5);

  skeleton->setPositions(Eigen::VectorXd::Random(skeleton->getNumDofs()));

  const auto massMatrix = skeleton->getMassMatrix();

  // Mass matrix must be symmetric
  EXPECT_TRUE(massMatrix.isApprox(massMatrix.transpose(), 1e-10));
}

TEST(SkeletonMassMatrix, MassMatrixPositiveDefinite)
{
  auto skeleton = Skeleton::create("pd_test");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  rootPair.second->setMass(1.0);

  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  childPair.first->setAxis(Eigen::Vector3d::UnitY());
  childPair.second->setMass(0.5);

  skeleton->setPositions(Eigen::VectorXd::Random(skeleton->getNumDofs()));

  const auto massMatrix = skeleton->getMassMatrix();

  // All eigenvalues should be positive for a positive definite matrix
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(massMatrix);
  const auto eigenvalues = solver.eigenvalues();
  for (int i = 0; i < eigenvalues.size(); ++i) {
    EXPECT_GT(eigenvalues(i), 0.0);
  }
}

TEST(SkeletonMassMatrix, InverseMassMatrix)
{
  auto skeleton = Skeleton::create("inv_mass_test");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  rootPair.second->setMass(1.0);

  skeleton->setPositions(Eigen::VectorXd::Random(skeleton->getNumDofs()));

  const auto M = skeleton->getMassMatrix();
  const auto Minv = skeleton->getInvMassMatrix();

  // M * M^-1 should be identity
  Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(M.rows(), M.cols());
  EXPECT_TRUE((M * Minv).isApprox(identity, 1e-8));
}

TEST(SkeletonMassMatrix, AugmentedMassMatrix)
{
  auto skeleton = Skeleton::create("aug_mass_test");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  rootPair.second->setMass(1.0);

  const auto M = skeleton->getMassMatrix();
  const auto Maug = skeleton->getAugMassMatrix();

  // Augmented mass matrix should be at least as large as mass matrix
  EXPECT_EQ(Maug.rows(), M.rows());
  EXPECT_EQ(Maug.cols(), M.cols());

  // Diagonal elements should be >= corresponding mass matrix elements
  for (int i = 0; i < M.rows(); ++i) {
    EXPECT_GE(Maug(i, i), M(i, i) - 1e-10);
  }
}

//==============================================================================
// Support Polygon Tests
//==============================================================================

static SkeletonPtr createSkeletonWithEndEffectors()
{
  auto skeleton = Skeleton::create("ee_test");

  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  rootPair.second->setName("base");
  rootPair.second->setMass(10.0);

  RevoluteJoint::Properties leftLegProps;
  leftLegProps.mName = "left_leg_joint";
  leftLegProps.mAxis = Eigen::Vector3d::UnitY();
  leftLegProps.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(-0.2, 0, 0);

  BodyNode::Properties leftFootProps;
  leftFootProps.mName = "left_foot";
  leftFootProps.mInertia.setMass(1.0);

  auto leftPair = skeleton->createJointAndBodyNodePair<RevoluteJoint>(
      rootPair.second, leftLegProps, leftFootProps);

  auto leftEE = leftPair.second->createEndEffector("left_ee");
  leftEE->setDefaultRelativeTransform(Eigen::Isometry3d::Identity());

  RevoluteJoint::Properties rightLegProps;
  rightLegProps.mName = "right_leg_joint";
  rightLegProps.mAxis = Eigen::Vector3d::UnitY();
  rightLegProps.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0.2, 0, 0);

  BodyNode::Properties rightFootProps;
  rightFootProps.mName = "right_foot";
  rightFootProps.mInertia.setMass(1.0);

  auto rightPair = skeleton->createJointAndBodyNodePair<RevoluteJoint>(
      rootPair.second, rightLegProps, rightFootProps);

  auto rightEE = rightPair.second->createEndEffector("right_ee");
  rightEE->setDefaultRelativeTransform(Eigen::Isometry3d::Identity());

  return skeleton;
}

TEST(SkeletonSupportPolygon, EmptyWithoutEndEffectorsOnGround)
{
  auto skeleton = Skeleton::create("no_ee");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  const auto& polygon = skeleton->getSupportPolygon();
  EXPECT_TRUE(polygon.empty());
}

TEST(SkeletonSupportPolygon, ComputesWithEndEffectors)
{
  auto skeleton = createSkeletonWithEndEffectors();

  // Access support polygon - shouldn't crash
  const auto& polygon = skeleton->getSupportPolygon();
  (void)
      polygon; // May or may not have vertices depending on end effector config
}

TEST(SkeletonSupportPolygon, SupportIndicesTracking)
{
  auto skeleton = createSkeletonWithEndEffectors();

  const auto& indices = skeleton->getSupportIndices();
  // Returns indices of end effectors contributing to support
  EXPECT_TRUE(
      indices.empty() || indices.size() <= skeleton->getNumEndEffectors());
}

TEST(SkeletonSupportPolygon, SupportCentroid)
{
  auto skeleton = createSkeletonWithEndEffectors();

  const auto centroid = skeleton->getSupportCentroid();
  // Centroid should be finite (may be NaN if no support)
  if (!std::isnan(centroid.x()) && !std::isnan(centroid.y())) {
    EXPECT_TRUE(std::isfinite(centroid.x()));
    EXPECT_TRUE(std::isfinite(centroid.y()));
  }
}

TEST(SkeletonSupportPolygon, SupportVersion)
{
  auto skeleton = createSkeletonWithEndEffectors();

  const auto v1 = skeleton->getSupportVersion();

  // Change position - version should update
  skeleton->setPosition(0, 0.1);
  skeleton->getSupportPolygon(); // Force update

  const auto v2 = skeleton->getSupportVersion();
  EXPECT_GE(v2, v1);
}

TEST(SkeletonSupportPolygon, TreeSpecificSupportPolygon)
{
  auto skeleton = createSkeletonWithEndEffectors();

  // Test tree-specific support polygon
  const auto& treePolygon = skeleton->getSupportPolygon(0);
  const auto& treeIndices = skeleton->getSupportIndices(0);
  const auto treeCentroid = skeleton->getSupportCentroid(0);

  (void)treePolygon;
  (void)treeIndices;
  (void)treeCentroid;
  // Just verify they don't crash
}

//==============================================================================
// Skeleton Cloning Tests
//==============================================================================

TEST(SkeletonClone, BasicClone)
{
  auto original = Skeleton::create("original");
  auto pair = original->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setName("body");
  pair.second->setMass(2.5);

  original->setPositions(Eigen::VectorXd::Ones(original->getNumDofs()) * 0.5);

  auto clone = original->cloneSkeleton("clone");

  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->getName(), "clone");
  EXPECT_EQ(clone->getNumBodyNodes(), original->getNumBodyNodes());
  EXPECT_EQ(clone->getNumJoints(), original->getNumJoints());
  EXPECT_DOUBLE_EQ(clone->getBodyNode("body")->getMass(), 2.5);
  EXPECT_TRUE(clone->getPositions().isApprox(original->getPositions()));
}

TEST(SkeletonClone, CloneIndependence)
{
  auto original = Skeleton::create("original");
  auto pair = original->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  original->setPositions(Eigen::VectorXd::Zero(original->getNumDofs()));

  auto clone = original->cloneSkeleton("clone");

  // Modify original
  original->setPositions(Eigen::VectorXd::Ones(original->getNumDofs()) * 2.0);
  original->getBodyNode(0)->setMass(5.0);

  // Clone should be unaffected
  EXPECT_TRUE(clone->getPositions().isZero());
  EXPECT_DOUBLE_EQ(clone->getBodyNode(0)->getMass(), 1.0);
}

TEST(SkeletonClone, ClonePreservesJointProperties)
{
  auto original = Skeleton::create("original");

  RevoluteJoint::Properties jointProps;
  jointProps.mName = "revolute";
  jointProps.mAxis = Eigen::Vector3d(1.0, 1.0, 0.0).normalized();
  jointProps.mPositionLowerLimits << -1.5;
  jointProps.mPositionUpperLimits << 1.5;
  jointProps.mDampingCoefficients << 0.1;
  jointProps.mSpringStiffnesses << 10.0;

  BodyNode::Properties bodyProps;
  bodyProps.mName = "body";

  original->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, jointProps, bodyProps);

  auto clone = original->cloneSkeleton("clone");

  auto* clonedJoint = dynamic_cast<RevoluteJoint*>(clone->getJoint("revolute"));
  ASSERT_NE(clonedJoint, nullptr);

  EXPECT_TRUE(clonedJoint->getAxis().isApprox(jointProps.mAxis));
  EXPECT_DOUBLE_EQ(clonedJoint->getPositionLowerLimit(0), -1.5);
  EXPECT_DOUBLE_EQ(clonedJoint->getPositionUpperLimit(0), 1.5);
  EXPECT_DOUBLE_EQ(clonedJoint->getDampingCoefficient(0), 0.1);
  EXPECT_DOUBLE_EQ(clonedJoint->getSpringStiffness(0), 10.0);
}

TEST(SkeletonClone, ClonePreservesTreeStructure)
{
  auto original = Skeleton::create("original");

  auto rootPair = original->createJointAndBodyNodePair<FreeJoint>();
  rootPair.second->setName("root");

  auto child1
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child1.second->setName("child1");

  auto child2
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child2.second->setName("child2");

  auto grandchild
      = child1.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  grandchild.second->setName("grandchild");

  auto clone = original->cloneSkeleton("clone");

  EXPECT_EQ(clone->getNumBodyNodes(), 4u);

  auto* clonedRoot = clone->getBodyNode("root");
  ASSERT_NE(clonedRoot, nullptr);
  EXPECT_EQ(clonedRoot->getNumChildBodyNodes(), 2u);

  auto* clonedChild1 = clone->getBodyNode("child1");
  ASSERT_NE(clonedChild1, nullptr);
  EXPECT_EQ(clonedChild1->getParentBodyNode()->getName(), "root");
  EXPECT_EQ(clonedChild1->getNumChildBodyNodes(), 1u);

  auto* clonedGrandchild = clone->getBodyNode("grandchild");
  ASSERT_NE(clonedGrandchild, nullptr);
  EXPECT_EQ(clonedGrandchild->getParentBodyNode()->getName(), "child1");
}

TEST(SkeletonClone, CloneWithMimicJoint)
{
  auto original = Skeleton::create("mimic_test");

  RevoluteJoint::Properties masterProps;
  masterProps.mName = "master_joint";
  masterProps.mAxis = Eigen::Vector3d::UnitZ();
  BodyNode::Properties masterBodyProps;
  masterBodyProps.mName = "master_body";

  auto masterPair = original->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, masterProps, masterBodyProps);
  auto* masterJoint = masterPair.first;

  RevoluteJoint::Properties slaveProps;
  slaveProps.mName = "slave_joint";
  slaveProps.mAxis = Eigen::Vector3d::UnitZ();
  BodyNode::Properties slaveBodyProps;
  slaveBodyProps.mName = "slave_body";

  auto slavePair = original->createJointAndBodyNodePair<RevoluteJoint>(
      masterPair.second, slaveProps, slaveBodyProps);
  auto* slaveJoint = slavePair.first;

  // Set up mimic relationship: slave = 2.0 * master + 0.5
  slaveJoint->setMimicJoint(masterJoint, 2.0, 0.5);

  auto clone = original->cloneSkeleton("clone");

  auto* clonedSlave
      = dynamic_cast<RevoluteJoint*>(clone->getJoint("slave_joint"));
  ASSERT_NE(clonedSlave, nullptr);

  // Mimic multiplier and offset should be preserved
  EXPECT_DOUBLE_EQ(clonedSlave->getMimicMultiplier(0), 2.0);
  EXPECT_DOUBLE_EQ(clonedSlave->getMimicOffset(0), 0.5);

  // Verify clone has its own master joint
  auto* clonedMaster
      = dynamic_cast<RevoluteJoint*>(clone->getJoint("master_joint"));
  ASSERT_NE(clonedMaster, nullptr);
  EXPECT_NE(clonedMaster, masterJoint);
}

//==============================================================================
// Tree Manipulation Error Handling Tests
//==============================================================================

TEST(SkeletonTreeManipulation, GetBodyNodeByIndex)
{
  auto skeleton = Skeleton::create("test");
  skeleton->createJointAndBodyNodePair<FreeJoint>();

  EXPECT_EQ(skeleton->getBodyNode(0), skeleton->getRootBodyNode());
  EXPECT_NE(skeleton->getBodyNode(0), nullptr);
}

TEST(SkeletonTreeManipulation, GetJointByIndex)
{
  auto skeleton = Skeleton::create("test");
  skeleton->createJointAndBodyNodePair<FreeJoint>();

  EXPECT_NE(skeleton->getJoint(0), nullptr);
}

TEST(SkeletonTreeManipulation, GetNonexistentBodyNodeByName)
{
  auto skeleton = Skeleton::create("test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setName("actual_body");

  EXPECT_NE(skeleton->getBodyNode("actual_body"), nullptr);
  EXPECT_EQ(skeleton->getBodyNode("nonexistent"), nullptr);
  EXPECT_EQ(skeleton->getBodyNode(""), nullptr);
}

TEST(SkeletonTreeManipulation, GetNonexistentJointByName)
{
  auto skeleton = Skeleton::create("test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.first->setName("actual_joint");

  EXPECT_NE(skeleton->getJoint("actual_joint"), nullptr);
  EXPECT_EQ(skeleton->getJoint("nonexistent"), nullptr);
}

TEST(SkeletonTreeManipulation, GetTreeOutOfRange)
{
  auto skeleton = Skeleton::create("test");
  skeleton->createJointAndBodyNodePair<FreeJoint>();

  EXPECT_EQ(skeleton->getNumTrees(), 1u);
  EXPECT_NE(skeleton->getRootBodyNode(0), nullptr);
}

TEST(SkeletonTreeManipulation, EmptySkeleton)
{
  auto skeleton = Skeleton::create("empty");

  EXPECT_EQ(skeleton->getNumBodyNodes(), 0u);
  EXPECT_EQ(skeleton->getNumJoints(), 0u);
  EXPECT_EQ(skeleton->getNumDofs(), 0u);
  EXPECT_EQ(skeleton->getNumTrees(), 0u);

  // getMassMatrix on empty skeleton should return empty matrix
  const auto& massMatrix = skeleton->getMassMatrix();
  EXPECT_EQ(massMatrix.rows(), 0);
  EXPECT_EQ(massMatrix.cols(), 0);
}

// ============================================================================
// Skeleton Configuration Tests
// ============================================================================

TEST(SkeletonConfiguration, ConstructFromPositionsOnly)
{
  auto skeleton = Skeleton::create("config_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());

  auto child = pair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child.first->setAxis(Eigen::Vector3d::UnitY());

  const std::size_t dofs = skeleton->getNumDofs();
  ASSERT_EQ(dofs, 2u);

  Eigen::VectorXd positions(2);
  positions << 0.5, 1.0;

  Skeleton::Configuration config(positions);
  EXPECT_EQ(config.mPositions.size(), 2);
  EXPECT_EQ(config.mIndices.size(), 2u);
  EXPECT_EQ(config.mIndices[0], 0u);
  EXPECT_EQ(config.mIndices[1], 1u);
}

TEST(SkeletonConfiguration, ConstructWithAllVectors)
{
  Eigen::VectorXd pos(2);
  pos << 0.1, 0.2;
  Eigen::VectorXd vel(2);
  vel << 0.3, 0.4;
  Eigen::VectorXd acc(2);
  acc << 0.5, 0.6;
  Eigen::VectorXd forces(2);
  forces << 0.7, 0.8;
  Eigen::VectorXd commands(2);
  commands << 0.9, 1.0;

  Skeleton::Configuration config(pos, vel, acc, forces, commands);
  EXPECT_EQ(config.mPositions.size(), 2);
  EXPECT_EQ(config.mVelocities.size(), 2);
  EXPECT_EQ(config.mAccelerations.size(), 2);
  EXPECT_EQ(config.mForces.size(), 2);
  EXPECT_EQ(config.mCommands.size(), 2);
  EXPECT_EQ(config.mIndices.size(), 2u);
}

TEST(SkeletonConfiguration, EqualityOperator)
{
  Eigen::VectorXd pos(2);
  pos << 0.1, 0.2;

  Skeleton::Configuration config1(pos);
  Skeleton::Configuration config2(pos);

  EXPECT_TRUE(config1 == config2);
  EXPECT_FALSE(config1 != config2);

  Eigen::VectorXd pos2(2);
  pos2 << 0.3, 0.4;
  Skeleton::Configuration config3(pos2);

  EXPECT_FALSE(config1 == config3);
  EXPECT_TRUE(config1 != config3);
}

TEST(SkeletonConfiguration, SetAndGetConfiguration)
{
  auto skeleton = Skeleton::create("config_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());

  auto child = pair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child.first->setAxis(Eigen::Vector3d::UnitY());

  const std::size_t dofs = skeleton->getNumDofs();

  Eigen::VectorXd positions(dofs);
  positions << 0.5, 1.0;
  Eigen::VectorXd velocities(dofs);
  velocities << 0.1, 0.2;

  Skeleton::Configuration config(positions, velocities);
  skeleton->setConfiguration(config);

  EXPECT_TRUE(skeleton->getPositions().isApprox(positions));
  EXPECT_TRUE(skeleton->getVelocities().isApprox(velocities));

  // Get configuration with specific flags
  auto retrieved = skeleton->getConfiguration(
      Skeleton::CONFIG_POSITIONS | Skeleton::CONFIG_VELOCITIES);
  EXPECT_TRUE(retrieved.mPositions.isApprox(positions));
  EXPECT_TRUE(retrieved.mVelocities.isApprox(velocities));
}

TEST(SkeletonConfiguration, GetConfigurationWithIndices)
{
  auto skeleton = Skeleton::create("config_idx_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());

  auto child = pair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child.first->setAxis(Eigen::Vector3d::UnitY());

  Eigen::VectorXd positions(2);
  positions << 0.5, 1.0;
  skeleton->setPositions(positions);

  // Get configuration for specific indices
  std::vector<std::size_t> indices = {0};
  auto config = skeleton->getConfiguration(indices, Skeleton::CONFIG_POSITIONS);
  EXPECT_EQ(config.mPositions.size(), 1);
  EXPECT_NEAR(config.mPositions[0], 0.5, 1e-10);
}

TEST(SkeletonConfiguration, GetConfigurationNothing)
{
  auto skeleton = Skeleton::create("config_nothing");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());

  auto config = skeleton->getConfiguration(Skeleton::CONFIG_NOTHING);
  EXPECT_EQ(config.mPositions.size(), 0);
  EXPECT_EQ(config.mVelocities.size(), 0);
}

TEST(SkeletonConfiguration, GetConfigurationCommands)
{
  auto skeleton = Skeleton::create("config_cmd");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());

  skeleton->setCommand(0, 1.5);

  auto config = skeleton->getConfiguration(Skeleton::CONFIG_COMMANDS);
  EXPECT_EQ(config.mCommands.size(), 1);
  EXPECT_NEAR(config.mCommands[0], 1.5, 1e-10);
}

// ============================================================================
// Skeleton Mobile/Collision/Adjacent Tests
// ============================================================================

TEST(SkeletonProperties, MobileProperty)
{
  auto skeleton = Skeleton::create("mobile_test");
  skeleton->createJointAndBodyNodePair<FreeJoint>();

  // Default should be mobile
  EXPECT_TRUE(skeleton->isMobile());

  skeleton->setMobile(false);
  EXPECT_FALSE(skeleton->isMobile());

  skeleton->setMobile(true);
  EXPECT_TRUE(skeleton->isMobile());
}

TEST(SkeletonProperties, SelfCollisionCheck)
{
  auto skeleton = Skeleton::create("self_collision_test");
  skeleton->createJointAndBodyNodePair<FreeJoint>();

  // Test enable/disable
  skeleton->setSelfCollisionCheck(true);
  EXPECT_TRUE(skeleton->getSelfCollisionCheck());
  EXPECT_TRUE(skeleton->isEnabledSelfCollisionCheck());

  skeleton->setSelfCollisionCheck(false);
  EXPECT_FALSE(skeleton->getSelfCollisionCheck());
  EXPECT_FALSE(skeleton->isEnabledSelfCollisionCheck());

  // Test convenience methods
  skeleton->enableSelfCollisionCheck();
  EXPECT_TRUE(skeleton->getSelfCollisionCheck());

  skeleton->disableSelfCollisionCheck();
  EXPECT_FALSE(skeleton->getSelfCollisionCheck());
}

TEST(SkeletonProperties, AdjacentBodyCheck)
{
  auto skeleton = Skeleton::create("adjacent_body_test");
  skeleton->createJointAndBodyNodePair<FreeJoint>();

  skeleton->setAdjacentBodyCheck(true);
  EXPECT_TRUE(skeleton->getAdjacentBodyCheck());
  EXPECT_TRUE(skeleton->isEnabledAdjacentBodyCheck());

  skeleton->setAdjacentBodyCheck(false);
  EXPECT_FALSE(skeleton->getAdjacentBodyCheck());
  EXPECT_FALSE(skeleton->isEnabledAdjacentBodyCheck());

  // Test convenience methods
  skeleton->enableAdjacentBodyCheck();
  EXPECT_TRUE(skeleton->getAdjacentBodyCheck());

  skeleton->disableAdjacentBodyCheck();
  EXPECT_FALSE(skeleton->getAdjacentBodyCheck());
}

TEST(SkeletonProperties, TimeStep)
{
  auto skeleton = Skeleton::create("timestep_test");
  skeleton->createJointAndBodyNodePair<FreeJoint>();

  skeleton->setTimeStep(0.002);
  EXPECT_DOUBLE_EQ(skeleton->getTimeStep(), 0.002);

  skeleton->setTimeStep(0.01);
  EXPECT_DOUBLE_EQ(skeleton->getTimeStep(), 0.01);
}

TEST(SkeletonProperties, Gravity)
{
  auto skeleton = Skeleton::create("gravity_test");
  skeleton->createJointAndBodyNodePair<FreeJoint>();

  Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  skeleton->setGravity(gravity);
  EXPECT_TRUE(skeleton->getGravity().isApprox(gravity));

  Eigen::Vector3d newGravity(0.0, -9.81, 0.0);
  skeleton->setGravity(newGravity);
  EXPECT_TRUE(skeleton->getGravity().isApprox(newGravity));
}

TEST(SkeletonProperties, AspectProperties)
{
  auto skeleton = Skeleton::create("aspect_test");
  skeleton->createJointAndBodyNodePair<FreeJoint>();

  Skeleton::AspectProperties props;
  props.mName = "new_name";
  props.mIsMobile = false;
  props.mGravity = Eigen::Vector3d(0, 0, -10.0);
  props.mTimeStep = 0.005;
  props.mEnabledSelfCollisionCheck = true;
  props.mEnabledAdjacentBodyCheck = true;

  skeleton->setAspectProperties(props);

  EXPECT_EQ(skeleton->getName(), "new_name");
  EXPECT_FALSE(skeleton->isMobile());
  EXPECT_TRUE(skeleton->getGravity().isApprox(Eigen::Vector3d(0, 0, -10.0)));
  EXPECT_DOUBLE_EQ(skeleton->getTimeStep(), 0.005);
  EXPECT_TRUE(skeleton->getSelfCollisionCheck());
  EXPECT_TRUE(skeleton->getAdjacentBodyCheck());
}

// ============================================================================
// Skeleton IK Tests
// ============================================================================

TEST(SkeletonIK, CreateAndClearIK)
{
  auto skeleton = Skeleton::create("ik_test");
  skeleton->createJointAndBodyNodePair<FreeJoint>();

  // Initially no IK
  EXPECT_EQ(skeleton->getIK(false), nullptr);

  // Create IK
  const auto& ik = skeleton->createIK();
  EXPECT_NE(ik, nullptr);

  // getIK should return the same IK
  EXPECT_EQ(skeleton->getIK(false), ik);

  // getOrCreateIK should return the same IK
  EXPECT_EQ(skeleton->getOrCreateIK(), ik);

  // Clear IK
  skeleton->clearIK();
  EXPECT_EQ(skeleton->getIK(false), nullptr);

  // getIK with createIfNull=true should create a new one
  const auto& newIk = skeleton->getIK(true);
  EXPECT_NE(newIk, nullptr);
}

TEST(SkeletonIK, ConstGetIK)
{
  auto skeleton = Skeleton::create("ik_const_test");
  skeleton->createJointAndBodyNodePair<FreeJoint>();

  // Create IK
  skeleton->createIK();

  // Const access
  const Skeleton* constSkel = skeleton.get();
  auto constIk = constSkel->getIK();
  EXPECT_NE(constIk, nullptr);
}

// ============================================================================
// Skeleton Force/Dynamics Accessors
// ============================================================================

TEST(SkeletonForces, ClearExternalForces)
{
  auto skeleton = Skeleton::create("ext_force_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  // Add external force to body
  pair.second->addExtForce(
      Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d::Zero(), true, true);

  // Verify force was added
  EXPECT_FALSE(pair.second->getExternalForceLocal().isZero());

  // Clear all external forces
  skeleton->clearExternalForces();

  // Verify forces are cleared
  EXPECT_TRUE(pair.second->getExternalForceLocal().isZero());
}

TEST(SkeletonForces, ClearInternalForces)
{
  auto skeleton = Skeleton::create("int_force_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());

  // Set a force on the joint
  skeleton->setForce(0, 5.0);
  EXPECT_NEAR(skeleton->getForce(0), 5.0, 1e-10);

  // Clear internal forces
  skeleton->clearInternalForces();

  // Verify forces are cleared
  EXPECT_NEAR(skeleton->getForce(0), 0.0, 1e-10);
}

TEST(SkeletonForces, CoriolisForces)
{
  auto skeleton = Skeleton::create("coriolis_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  auto child = pair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child.first->setAxis(Eigen::Vector3d::UnitZ());
  child.second->setMass(0.5);

  // Set some velocities to generate Coriolis forces
  Eigen::VectorXd vel = Eigen::VectorXd::Zero(skeleton->getNumDofs());
  vel[0] = 1.0; // angular velocity
  vel[3] = 0.5; // linear velocity
  vel[6] = 2.0; // revolute joint velocity
  skeleton->setVelocities(vel);

  const auto& coriolisForces = skeleton->getCoriolisForces();
  EXPECT_EQ(coriolisForces.size(), static_cast<int>(skeleton->getNumDofs()));
  EXPECT_TRUE(coriolisForces.array().isFinite().all());
}

TEST(SkeletonForces, CoriolisAndGravityForces)
{
  auto skeleton = Skeleton::create("cg_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  skeleton->setGravity(Eigen::Vector3d(0, 0, -9.81));

  Eigen::VectorXd vel = Eigen::VectorXd::Zero(skeleton->getNumDofs());
  vel[0] = 1.0;
  skeleton->setVelocities(vel);

  const auto& cgForces = skeleton->getCoriolisAndGravityForces();
  EXPECT_EQ(cgForces.size(), static_cast<int>(skeleton->getNumDofs()));
  EXPECT_TRUE(cgForces.array().isFinite().all());

  // With gravity, the combined forces should be non-zero
  EXPECT_GT(cgForces.norm(), 0.0);
}

TEST(SkeletonForces, ExternalForces)
{
  auto skeleton = Skeleton::create("ext_forces_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  // Add external force
  pair.second->addExtForce(
      Eigen::Vector3d(10.0, 0.0, 0.0), Eigen::Vector3d::Zero(), true, true);

  const auto& extForces = skeleton->getExternalForces();
  EXPECT_EQ(extForces.size(), static_cast<int>(skeleton->getNumDofs()));
  EXPECT_TRUE(extForces.array().isFinite().all());
  EXPECT_GT(extForces.norm(), 0.0);
}

TEST(SkeletonForces, ConstraintForces)
{
  auto skeleton = Skeleton::create("constraint_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());
  pair.second->setMass(1.0);

  const auto& constraintForces = skeleton->getConstraintForces();
  EXPECT_EQ(constraintForces.size(), static_cast<int>(skeleton->getNumDofs()));
  EXPECT_TRUE(constraintForces.array().isFinite().all());
}

TEST(SkeletonForces, GravityForces)
{
  auto skeleton = Skeleton::create("gravity_forces_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  skeleton->setGravity(Eigen::Vector3d(0, 0, -9.81));

  const auto& gravityForces = skeleton->getGravityForces();
  EXPECT_EQ(gravityForces.size(), static_cast<int>(skeleton->getNumDofs()));
  EXPECT_TRUE(gravityForces.array().isFinite().all());
  EXPECT_GT(gravityForces.norm(), 0.0);
}

// ============================================================================
// Skeleton Forward Kinematics
// ============================================================================

TEST(SkeletonKinematics, ComputeForwardKinematics)
{
  auto skeleton = Skeleton::create("fk_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  auto child = pair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child.first->setAxis(Eigen::Vector3d::UnitZ());
  child.second->setMass(0.5);

  // Set some positions and velocities
  Eigen::VectorXd pos = Eigen::VectorXd::Zero(skeleton->getNumDofs());
  pos[6] = 0.5; // revolute joint position
  skeleton->setPositions(pos);

  Eigen::VectorXd vel = Eigen::VectorXd::Zero(skeleton->getNumDofs());
  vel[6] = 1.0;
  skeleton->setVelocities(vel);

  // Compute forward kinematics with all flags
  skeleton->computeForwardKinematics(true, true, true);

  // Verify transforms are finite
  auto transform = child.second->getWorldTransform();
  EXPECT_TRUE(transform.matrix().array().isFinite().all());
}

TEST(SkeletonKinematics, ComputeForwardKinematicsTransformOnly)
{
  auto skeleton = Skeleton::create("fk_transform_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());
  pair.second->setMass(1.0);

  skeleton->setPosition(0, 1.0);

  // Compute only transforms
  skeleton->computeForwardKinematics(true, false, false);

  auto transform = pair.second->getWorldTransform();
  EXPECT_TRUE(transform.matrix().array().isFinite().all());
}

// ============================================================================
// Skeleton Ptr and Mutex
// ============================================================================

TEST(SkeletonPtr, GetPtrAndSkeleton)
{
  auto skeleton = Skeleton::create("ptr_test");

  auto ptr = skeleton->getPtr();
  EXPECT_EQ(ptr, skeleton);

  auto skelPtr = skeleton->getSkeleton();
  EXPECT_EQ(skelPtr, skeleton);

  // Const versions
  const Skeleton* constSkel = skeleton.get();
  auto constPtr = constSkel->getPtr();
  EXPECT_NE(constPtr, nullptr);

  auto constSkelPtr = constSkel->getSkeleton();
  EXPECT_NE(constSkelPtr, nullptr);
}

TEST(SkeletonPtr, GetMutex)
{
  auto skeleton = Skeleton::create("mutex_test");

  // Just verify we can get the mutex without crashing
  auto& mutex = skeleton->getMutex();
  (void)mutex;

  // Verify lockable reference
  auto lockable = skeleton->getLockableReference();
  EXPECT_NE(lockable, nullptr);
}

// ============================================================================
// Skeleton State and Properties
// ============================================================================

TEST(SkeletonState, SetAndGetState)
{
  auto skeleton = Skeleton::create("state_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());
  pair.second->setMass(1.0);

  skeleton->setPosition(0, 1.5);

  auto state = skeleton->getState();

  // Create a new skeleton with same structure
  auto skeleton2 = skeleton->cloneSkeleton("clone");
  skeleton2->setPosition(0, 0.0);
  EXPECT_NEAR(skeleton2->getPosition(0), 0.0, 1e-10);

  // Restore state
  skeleton2->setState(state);
  EXPECT_NEAR(skeleton2->getPosition(0), 1.5, 1e-10);
}

TEST(SkeletonState, SetAndGetProperties)
{
  auto skeleton = Skeleton::create("props_test");
  skeleton->createJointAndBodyNodePair<FreeJoint>();

  skeleton->setMobile(false);
  skeleton->setGravity(Eigen::Vector3d(0, 0, -5.0));

  auto props = skeleton->getProperties();

  auto skeleton2 = Skeleton::create("props_test2");
  skeleton2->createJointAndBodyNodePair<FreeJoint>();
  skeleton2->setProperties(props);

  // Properties should be transferred
  EXPECT_FALSE(skeleton2->isMobile());
}

// ============================================================================
// Skeleton Naming
// ============================================================================

TEST(SkeletonNaming, SetName)
{
  auto skeleton = Skeleton::create("original_name");
  EXPECT_EQ(skeleton->getName(), "original_name");

  skeleton->setName("new_name");
  EXPECT_EQ(skeleton->getName(), "new_name");

  // Setting same name should be a no-op
  const auto& result = skeleton->setName("new_name");
  EXPECT_EQ(result, "new_name");
}

// ============================================================================
// Skeleton Body/Joint Lookup
// ============================================================================

TEST(SkeletonLookup, HasBodyNode)
{
  auto skeleton = Skeleton::create("has_bn_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();

  EXPECT_TRUE(skeleton->hasBodyNode(pair.second));

  auto skeleton2 = Skeleton::create("other");
  auto pair2 = skeleton2->createJointAndBodyNodePair<FreeJoint>();

  EXPECT_FALSE(skeleton->hasBodyNode(pair2.second));
  EXPECT_FALSE(skeleton->hasBodyNode(nullptr));
}

TEST(SkeletonLookup, HasJoint)
{
  auto skeleton = Skeleton::create("has_joint_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();

  EXPECT_TRUE(skeleton->hasJoint(pair.first));

  auto skeleton2 = Skeleton::create("other");
  auto pair2 = skeleton2->createJointAndBodyNodePair<FreeJoint>();

  EXPECT_FALSE(skeleton->hasJoint(pair2.first));
}

TEST(SkeletonLookup, GetBodyNodesByName)
{
  auto skeleton = Skeleton::create("bn_name_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setName("my_body");

  auto nodes = skeleton->getBodyNodes("my_body");
  EXPECT_EQ(nodes.size(), 1u);
  EXPECT_EQ(nodes[0], pair.second);

  auto empty = skeleton->getBodyNodes("nonexistent");
  EXPECT_TRUE(empty.empty());

  // Const version
  const Skeleton* constSkel = skeleton.get();
  auto constNodes = constSkel->getBodyNodes("my_body");
  EXPECT_EQ(constNodes.size(), 1u);

  auto constEmpty = constSkel->getBodyNodes("nonexistent");
  EXPECT_TRUE(constEmpty.empty());
}

TEST(SkeletonLookup, GetJointsByName)
{
  auto skeleton = Skeleton::create("joint_name_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.first->setName("my_joint");

  auto joints = skeleton->getJoints("my_joint");
  EXPECT_EQ(joints.size(), 1u);
  EXPECT_EQ(joints[0], pair.first);

  auto empty = skeleton->getJoints("nonexistent");
  EXPECT_TRUE(empty.empty());

  // Const version
  const Skeleton* constSkel = skeleton.get();
  auto constJoints = constSkel->getJoints("my_joint");
  EXPECT_EQ(constJoints.size(), 1u);

  auto constEmpty = constSkel->getJoints("nonexistent");
  EXPECT_TRUE(constEmpty.empty());
}

TEST(SkeletonLookup, GetJointsVector)
{
  auto skeleton = Skeleton::create("joints_vec_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  [[maybe_unused]] auto child
      = pair.second->createChildJointAndBodyNodePair<RevoluteJoint>();

  EXPECT_EQ(skeleton->getNumJoints(), 2u);

  const Skeleton* constSkel = skeleton.get();
  EXPECT_EQ(constSkel->getNumJoints(), 2u);
}

TEST(SkeletonLookup, GetIndexOf)
{
  auto skeleton = Skeleton::create("index_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto child = pair.second->createChildJointAndBodyNodePair<RevoluteJoint>();

  EXPECT_EQ(skeleton->getIndexOf(pair.second), 0u);
  EXPECT_EQ(skeleton->getIndexOf(child.second), 1u);

  EXPECT_EQ(skeleton->getIndexOf(pair.first), 0u);
  EXPECT_EQ(skeleton->getIndexOf(child.first), 1u);

  // DegreeOfFreedom index
  auto* dof = skeleton->getDof(0);
  EXPECT_EQ(skeleton->getIndexOf(dof), 0u);

  // Nullptr should return INVALID_INDEX
  EXPECT_EQ(
      skeleton->getIndexOf(static_cast<const BodyNode*>(nullptr), false),
      dart::dynamics::INVALID_INDEX);
  EXPECT_EQ(
      skeleton->getIndexOf(static_cast<const Joint*>(nullptr), false),
      dart::dynamics::INVALID_INDEX);
  EXPECT_EQ(
      skeleton->getIndexOf(static_cast<const DegreeOfFreedom*>(nullptr), false),
      dart::dynamics::INVALID_INDEX);
}

// ============================================================================
// Skeleton DOF Accessors
// ============================================================================

TEST(SkeletonDofs, GetDofByName)
{
  auto skeleton = Skeleton::create("dof_name_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setName("my_joint");

  auto* dof = skeleton->getDof("my_joint");
  EXPECT_NE(dof, nullptr);

  auto* noDof = skeleton->getDof("nonexistent");
  EXPECT_EQ(noDof, nullptr);

  // Const version
  const Skeleton* constSkel = skeleton.get();
  const auto* constDof = constSkel->getDof("my_joint");
  EXPECT_NE(constDof, nullptr);
}

TEST(SkeletonDofs, GetDofs)
{
  auto skeleton = Skeleton::create("dofs_test");
  [[maybe_unused]] auto pair
      = skeleton->createJointAndBodyNodePair<FreeJoint>();

  EXPECT_EQ(skeleton->getNumDofs(), 6u);

  const Skeleton* constSkel = skeleton.get();
  EXPECT_EQ(constSkel->getNumDofs(), 6u);
}

TEST(SkeletonDofs, GetTreeDofs)
{
  auto skeleton = Skeleton::create("tree_dofs_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  [[maybe_unused]] auto child
      = pair.second->createChildJointAndBodyNodePair<RevoluteJoint>();

  auto treeDofs = skeleton->getTreeDofs(0);
  EXPECT_EQ(treeDofs.size(), skeleton->getNumDofs());

  const Skeleton* constSkel = skeleton.get();
  auto constTreeDofs = constSkel->getTreeDofs(0);
  EXPECT_EQ(constTreeDofs.size(), skeleton->getNumDofs());
}

// ============================================================================
// Skeleton Energy
// ============================================================================

TEST(SkeletonEnergy, KineticEnergy)
{
  auto skeleton = Skeleton::create("ke_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  // Zero velocity -> zero KE
  EXPECT_NEAR(skeleton->computeKineticEnergy(), 0.0, 1e-10);

  // Set velocity
  Eigen::VectorXd vel = Eigen::VectorXd::Zero(skeleton->getNumDofs());
  vel[3] = 1.0; // linear velocity
  skeleton->setVelocities(vel);

  double ke = skeleton->computeKineticEnergy();
  EXPECT_GT(ke, 0.0);
  EXPECT_TRUE(std::isfinite(ke));
}

TEST(SkeletonEnergy, PotentialEnergy)
{
  auto skeleton = Skeleton::create("pe_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  skeleton->setGravity(Eigen::Vector3d(0, 0, -9.81));

  double pe = skeleton->computePotentialEnergy();
  EXPECT_TRUE(std::isfinite(pe));
}

// ============================================================================
// Skeleton Impulse Methods
// ============================================================================

TEST(SkeletonImpulse, ImpulseAppliedFlags)
{
  auto skeleton = Skeleton::create("impulse_flag_test");
  skeleton->createJointAndBodyNodePair<FreeJoint>();

  EXPECT_FALSE(skeleton->isImpulseApplied());
  skeleton->setImpulseApplied(true);
  EXPECT_TRUE(skeleton->isImpulseApplied());
  skeleton->setImpulseApplied(false);
  EXPECT_FALSE(skeleton->isImpulseApplied());
}

TEST(SkeletonImpulse, PositionImpulseAppliedFlags)
{
  auto skeleton = Skeleton::create("pos_impulse_flag_test");
  skeleton->createJointAndBodyNodePair<FreeJoint>();

  EXPECT_FALSE(skeleton->isPositionImpulseApplied());
  skeleton->setPositionImpulseApplied(true);
  EXPECT_TRUE(skeleton->isPositionImpulseApplied());
  skeleton->setPositionImpulseApplied(false);
  EXPECT_FALSE(skeleton->isPositionImpulseApplied());
}

TEST(SkeletonImpulse, ClearPositionConstraintImpulses)
{
  auto skeleton = Skeleton::create("clear_pos_impulse_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());
  pair.second->setMass(1.0);

  // Add position constraint impulse
  pair.second->addPositionConstraintImpulse(Eigen::Vector6d::Ones());
  EXPECT_FALSE(pair.second->getPositionConstraintImpulse().isZero());

  skeleton->clearPositionConstraintImpulses();
  EXPECT_TRUE(pair.second->getPositionConstraintImpulse().isZero());
}

TEST(SkeletonImpulse, PositionVelocityChanges)
{
  auto skeleton = Skeleton::create("pos_vel_changes_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());
  pair.second->setMass(1.0);

  const auto& changes = skeleton->getPositionVelocityChanges();
  EXPECT_EQ(changes.size(), static_cast<int>(skeleton->getNumDofs()));

  skeleton->clearPositionVelocityChanges();
  EXPECT_TRUE(skeleton->getPositionVelocityChanges().isZero());
}

// ============================================================================
// Skeleton Indexing Consistency
// ============================================================================

TEST(SkeletonIndexing, CheckIndexingConsistency)
{
  auto skeleton = Skeleton::create("indexing_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setName("root");

  auto child = pair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child.second->setName("child");

  EXPECT_TRUE(skeleton->checkIndexingConsistency());
}

// ============================================================================
// Skeleton Rigid/Soft Body Count
// ============================================================================

TEST(SkeletonCounts, RigidAndSoftBodyCounts)
{
  auto skeleton = Skeleton::create("count_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  auto child = pair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child.second->setMass(0.5);

  EXPECT_EQ(skeleton->getNumBodyNodes(), 2u);
  EXPECT_EQ(skeleton->getNumRigidBodyNodes(), 2u);
  EXPECT_EQ(skeleton->getNumSoftBodyNodes(), 0u);
}

// ============================================================================
// Skeleton Root Joint
// ============================================================================

TEST(SkeletonRoot, GetRootJoint)
{
  auto skeleton = Skeleton::create("root_joint_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();

  auto* rootJoint = skeleton->getRootJoint(0);
  EXPECT_NE(rootJoint, nullptr);
  EXPECT_EQ(rootJoint, pair.first);

  // Const version
  const Skeleton* constSkel = skeleton.get();
  const auto* constRootJoint = constSkel->getRootJoint(0);
  EXPECT_NE(constRootJoint, nullptr);
}

// ============================================================================
// Skeleton Velocity Differences
// ============================================================================

TEST(SkeletonDifferences, PositionDifferences)
{
  auto skeleton = Skeleton::create("pos_diff_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());

  Eigen::VectorXd q1(1);
  q1 << 0.5;
  Eigen::VectorXd q2(1);
  q2 << 1.5;

  auto diff = skeleton->getPositionDifferences(q2, q1);
  EXPECT_EQ(diff.size(), 1);
  EXPECT_NEAR(diff[0], 1.0, 1e-10);
}

TEST(SkeletonDifferences, VelocityDifferences)
{
  auto skeleton = Skeleton::create("vel_diff_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());

  Eigen::VectorXd dq1(1);
  dq1 << 0.5;
  Eigen::VectorXd dq2(1);
  dq2 << 1.5;

  auto diff = skeleton->getVelocityDifferences(dq2, dq1);
  EXPECT_EQ(diff.size(), 1);
  EXPECT_NEAR(diff[0], 1.0, 1e-10);
}

// ============================================================================
// Skeleton Inverse Dynamics
// ============================================================================

TEST(SkeletonDynamics, InverseDynamicsEmptySkeleton)
{
  auto skeleton = Skeleton::create("empty_id");

  // Should not crash on empty skeleton
  skeleton->computeInverseDynamics(true, true, true);
}

TEST(SkeletonDynamics, ComputeImpulseForwardDynamics)
{
  auto skeleton = Skeleton::create("impulse_fd_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());
  pair.second->setMass(1.0);

  // Should not crash
  skeleton->computeImpulseForwardDynamics();
}

// ============================================================================
// Skeleton Tree-specific Mass Matrix
// ============================================================================

TEST(SkeletonTreeMatrix, TreeMassMatrix)
{
  auto skeleton = Skeleton::create("tree_mass_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  auto child = pair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child.first->setAxis(Eigen::Vector3d::UnitZ());
  child.second->setMass(0.5);

  const auto& treeM = skeleton->getMassMatrix(0);
  EXPECT_EQ(treeM.rows(), static_cast<int>(skeleton->getNumDofs()));
  EXPECT_EQ(treeM.cols(), static_cast<int>(skeleton->getNumDofs()));
  EXPECT_TRUE(treeM.isApprox(treeM.transpose(), 1e-10));
}

TEST(SkeletonTreeMatrix, TreeInvMassMatrix)
{
  auto skeleton = Skeleton::create("tree_inv_mass_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  const auto& treeInvM = skeleton->getInvMassMatrix(0);
  EXPECT_EQ(treeInvM.rows(), static_cast<int>(skeleton->getNumDofs()));
}

TEST(SkeletonTreeMatrix, TreeAugMassMatrix)
{
  auto skeleton = Skeleton::create("tree_aug_mass_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  const auto& treeAugM = skeleton->getAugMassMatrix(0);
  EXPECT_EQ(treeAugM.rows(), static_cast<int>(skeleton->getNumDofs()));
}

TEST(SkeletonTreeMatrix, TreeInvAugMassMatrix)
{
  auto skeleton = Skeleton::create("tree_inv_aug_mass_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  const auto& treeInvAugM = skeleton->getInvAugMassMatrix(0);
  EXPECT_EQ(treeInvAugM.rows(), static_cast<int>(skeleton->getNumDofs()));
}

TEST(SkeletonTreeMatrix, TreeCoriolisForces)
{
  auto skeleton = Skeleton::create("tree_coriolis_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  const auto& treeCoriolis = skeleton->getCoriolisForces(0);
  EXPECT_EQ(treeCoriolis.size(), static_cast<int>(skeleton->getNumDofs()));
}

TEST(SkeletonTreeMatrix, TreeGravityForces)
{
  auto skeleton = Skeleton::create("tree_gravity_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  const auto& treeGravity = skeleton->getGravityForces(0);
  EXPECT_EQ(treeGravity.size(), static_cast<int>(skeleton->getNumDofs()));
}

TEST(SkeletonTreeMatrix, TreeCoriolisAndGravityForces)
{
  auto skeleton = Skeleton::create("tree_cg_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  const auto& treeCg = skeleton->getCoriolisAndGravityForces(0);
  EXPECT_EQ(treeCg.size(), static_cast<int>(skeleton->getNumDofs()));
}

TEST(SkeletonTreeMatrix, TreeExternalForces)
{
  auto skeleton = Skeleton::create("tree_ext_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  const auto& treeFext = skeleton->getExternalForces(0);
  EXPECT_EQ(treeFext.size(), static_cast<int>(skeleton->getNumDofs()));
}

TEST(SkeletonTreeMatrix, TreeConstraintForces)
{
  auto skeleton = Skeleton::create("tree_constraint_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());
  pair.second->setMass(1.0);

  const auto& treeFc = skeleton->getConstraintForces(0);
  EXPECT_EQ(treeFc.size(), static_cast<int>(skeleton->getNumDofs()));
}

// ============================================================================
// Skeleton Integrate Velocities
// ============================================================================

TEST(SkeletonIntegration, IntegrateVelocities)
{
  auto skeleton = Skeleton::create("integrate_vel_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());
  pair.second->setMass(1.0);

  skeleton->setVelocity(0, 1.0);
  skeleton->setAcceleration(0, 2.0);

  double dt = 0.1;
  double initialVel = skeleton->getVelocity(0);

  skeleton->integrateVelocities(dt);

  double expectedVel = initialVel + 2.0 * dt;
  EXPECT_NEAR(skeleton->getVelocity(0), expectedVel, 1e-10);
}

// ============================================================================
// Skeleton COM Jacobians
// ============================================================================

TEST(SkeletonCOM, COMJacobian)
{
  auto skeleton = Skeleton::create("com_jac_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  auto comJac = skeleton->getCOMJacobian();
  EXPECT_EQ(comJac.rows(), 6);
  EXPECT_EQ(comJac.cols(), static_cast<int>(skeleton->getNumDofs()));
  EXPECT_TRUE(comJac.array().isFinite().all());
}

TEST(SkeletonCOM, COMLinearJacobian)
{
  auto skeleton = Skeleton::create("com_lin_jac_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  auto comLinJac = skeleton->getCOMLinearJacobian();
  EXPECT_EQ(comLinJac.rows(), 3);
  EXPECT_EQ(comLinJac.cols(), static_cast<int>(skeleton->getNumDofs()));
}

TEST(SkeletonCOM, COMSpatialVelocity)
{
  auto skeleton = Skeleton::create("com_spatial_vel_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  Eigen::VectorXd vel = Eigen::VectorXd::Zero(skeleton->getNumDofs());
  vel[3] = 1.0;
  skeleton->setVelocities(vel);

  auto comSpatialVel = skeleton->getCOMSpatialVelocity();
  EXPECT_EQ(comSpatialVel.size(), 6);
  EXPECT_TRUE(comSpatialVel.array().isFinite().all());
}

TEST(SkeletonCOM, COMSpatialAcceleration)
{
  auto skeleton = Skeleton::create("com_spatial_acc_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  auto comSpatialAcc = skeleton->getCOMSpatialAcceleration();
  EXPECT_EQ(comSpatialAcc.size(), 6);
  EXPECT_TRUE(comSpatialAcc.array().isFinite().all());
}

TEST(SkeletonCOM, COMLinearAcceleration)
{
  auto skeleton = Skeleton::create("com_lin_acc_test");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  auto comLinAcc = skeleton->getCOMLinearAcceleration();
  EXPECT_EQ(comLinAcc.size(), 3);
  EXPECT_TRUE(comLinAcc.array().isFinite().all());
}
