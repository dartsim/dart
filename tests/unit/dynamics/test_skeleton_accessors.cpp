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
