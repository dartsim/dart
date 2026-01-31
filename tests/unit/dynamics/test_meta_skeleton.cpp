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

#include <dart/dynamics/ball_joint.hpp>
#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/meta_skeleton.hpp>
#include <dart/dynamics/prismatic_joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <gtest/gtest.h>

#include <vector>

#include <cmath>

using namespace dart;
using namespace dart::dynamics;

//==============================================================================
// Helper: Create a simple chain skeleton with N revolute joints
SkeletonPtr createChainSkeleton(
    std::size_t numBodies, const std::string& name = "chain")
{
  auto skel = Skeleton::create(name);

  BodyNode* parent = nullptr;
  for (std::size_t i = 0; i < numBodies; ++i) {
    auto pair = skel->createJointAndBodyNodePair<RevoluteJoint>(
        parent,
        RevoluteJoint::Properties(),
        BodyNode::AspectProperties("body_" + std::to_string(i)));
    pair.first->setName("joint_" + std::to_string(i));

    // Set mass properties
    Inertia inertia;
    inertia.setMass(1.0);
    pair.second->setInertia(inertia);

    parent = pair.second;
  }

  return skel;
}

// Helper: Create a skeleton with mixed joint types
SkeletonPtr createMixedSkeleton(const std::string& name = "mixed")
{
  auto skel = Skeleton::create(name);

  // Root: FreeJoint (6 DOFs)
  auto pair1 = skel->createJointAndBodyNodePair<FreeJoint>(
      nullptr,
      FreeJoint::Properties(),
      BodyNode::AspectProperties("root_body"));
  pair1.first->setName("free_joint");
  Inertia inertia1;
  inertia1.setMass(2.0);
  pair1.second->setInertia(inertia1);

  // Child 1: RevoluteJoint (1 DOF)
  auto pair2 = skel->createJointAndBodyNodePair<RevoluteJoint>(
      pair1.second,
      RevoluteJoint::Properties(),
      BodyNode::AspectProperties("child1"));
  pair2.first->setName("revolute_joint");
  Inertia inertia2;
  inertia2.setMass(1.0);
  pair2.second->setInertia(inertia2);

  // Child 2: BallJoint (3 DOFs)
  auto pair3 = skel->createJointAndBodyNodePair<BallJoint>(
      pair2.second,
      BallJoint::Properties(),
      BodyNode::AspectProperties("child2"));
  pair3.first->setName("ball_joint");
  Inertia inertia3;
  inertia3.setMass(0.5);
  pair3.second->setInertia(inertia3);

  return skel;
}

//==============================================================================
// Test name operations
TEST(MetaSkeletonTests, NameOperations)
{
  auto skel = createChainSkeleton(3, "test_skel");

  EXPECT_EQ(skel->getName(), "test_skel");

  skel->setName("renamed_skel");
  EXPECT_EQ(skel->getName(), "renamed_skel");
}

//==============================================================================
// Test structural properties
TEST(MetaSkeletonTests, StructuralProperties)
{
  auto skel = createChainSkeleton(5);

  // Number of body nodes
  EXPECT_EQ(skel->getNumBodyNodes(), 5u);

  // Number of joints
  EXPECT_EQ(skel->getNumJoints(), 5u);

  // Number of DOFs (5 revolute joints = 5 DOFs)
  EXPECT_EQ(skel->getNumDofs(), 5u);

  // Get BodyNode by index
  for (std::size_t i = 0; i < 5; ++i) {
    auto* body = skel->getBodyNode(i);
    ASSERT_NE(body, nullptr);
    EXPECT_EQ(body->getName(), "body_" + std::to_string(i));
  }

  // Get BodyNode by name
  auto* body2 = skel->getBodyNode("body_2");
  ASSERT_NE(body2, nullptr);
  EXPECT_EQ(body2->getName(), "body_2");

  // Get Joint by index
  for (std::size_t i = 0; i < 5; ++i) {
    auto* joint = skel->getJoint(i);
    ASSERT_NE(joint, nullptr);
    EXPECT_EQ(joint->getName(), "joint_" + std::to_string(i));
  }

  // Get Joint by name
  auto* joint3 = skel->getJoint("joint_3");
  ASSERT_NE(joint3, nullptr);
  EXPECT_EQ(joint3->getName(), "joint_3");

  // Get DOF by index
  for (std::size_t i = 0; i < 5; ++i) {
    auto* dof = skel->getDof(i);
    ASSERT_NE(dof, nullptr);
    EXPECT_EQ(dof->getIndexInSkeleton(), i);
  }
}

//==============================================================================
// Test hasBodyNode and hasJoint
TEST(MetaSkeletonTests, HasMethods)
{
  auto skel = createChainSkeleton(3);

  auto* body0 = skel->getBodyNode(0);
  auto* body1 = skel->getBodyNode(1);

  EXPECT_TRUE(skel->hasBodyNode(body0));
  EXPECT_TRUE(skel->hasBodyNode(body1));

  auto* joint0 = skel->getJoint(0);
  auto* joint1 = skel->getJoint(1);

  EXPECT_TRUE(skel->hasJoint(joint0));
  EXPECT_TRUE(skel->hasJoint(joint1));

  // Create another skeleton and check its bodies aren't in this one
  auto skel2 = createChainSkeleton(2, "other");
  auto* otherBody = skel2->getBodyNode(0);
  EXPECT_FALSE(skel->hasBodyNode(otherBody));

  auto* otherJoint = skel2->getJoint(0);
  EXPECT_FALSE(skel->hasJoint(otherJoint));
}

//==============================================================================
// Test getIndexOf methods
TEST(MetaSkeletonTests, GetIndexOf)
{
  auto skel = createChainSkeleton(4);

  for (std::size_t i = 0; i < 4; ++i) {
    auto* body = skel->getBodyNode(i);
    EXPECT_EQ(skel->getIndexOf(body), i);

    auto* joint = skel->getJoint(i);
    EXPECT_EQ(skel->getIndexOf(joint), i);

    auto* dof = skel->getDof(i);
    EXPECT_EQ(skel->getIndexOf(dof), i);
  }
}

//==============================================================================
// Test command operations
TEST(MetaSkeletonTests, CommandOperations)
{
  auto skel = createChainSkeleton(3);

  // Set individual command
  skel->setCommand(0, 1.0);
  EXPECT_DOUBLE_EQ(skel->getCommand(0), 1.0);

  skel->setCommand(1, 2.0);
  EXPECT_DOUBLE_EQ(skel->getCommand(1), 2.0);

  // Set all commands
  Eigen::VectorXd commands(3);
  commands << 0.5, 1.5, 2.5;
  skel->setCommands(commands);

  Eigen::VectorXd result = skel->getCommands();
  EXPECT_EQ(result.size(), 3);
  EXPECT_DOUBLE_EQ(result[0], 0.5);
  EXPECT_DOUBLE_EQ(result[1], 1.5);
  EXPECT_DOUBLE_EQ(result[2], 2.5);

  // Reset commands
  skel->resetCommands();
  result = skel->getCommands();
  EXPECT_DOUBLE_EQ(result[0], 0.0);
  EXPECT_DOUBLE_EQ(result[1], 0.0);
  EXPECT_DOUBLE_EQ(result[2], 0.0);
}

//==============================================================================
TEST(MetaSkeletonTests, ComQueriesWithFrame)
{
  auto skel = createMixedSkeleton("com_frame");

  skel->setVelocities(Eigen::VectorXd::Constant(skel->getNumDofs(), 0.2));
  skel->setAccelerations(Eigen::VectorXd::Constant(skel->getNumDofs(), -0.1));

  const auto com = skel->getCOM(Frame::World());
  EXPECT_TRUE(com.array().isFinite().all());

  const auto comLinearVelocity = skel->getCOMLinearVelocity(Frame::World());
  EXPECT_TRUE(comLinearVelocity.array().isFinite().all());

  const auto comLinearAcceleration
      = skel->getCOMLinearAcceleration(Frame::World());
  EXPECT_TRUE(comLinearAcceleration.array().isFinite().all());
}

//==============================================================================
// Test position operations
TEST(MetaSkeletonTests, PositionOperations)
{
  auto skel = createChainSkeleton(3);

  // Set individual position
  skel->setPosition(0, 0.5);
  EXPECT_DOUBLE_EQ(skel->getPosition(0), 0.5);

  // Set all positions
  Eigen::VectorXd positions(3);
  positions << 0.1, 0.2, 0.3;
  skel->setPositions(positions);

  Eigen::VectorXd result = skel->getPositions();
  EXPECT_EQ(result.size(), 3);
  EXPECT_DOUBLE_EQ(result[0], 0.1);
  EXPECT_DOUBLE_EQ(result[1], 0.2);
  EXPECT_DOUBLE_EQ(result[2], 0.3);

  // Reset positions
  skel->resetPositions();
  result = skel->getPositions();
  EXPECT_DOUBLE_EQ(result[0], 0.0);
  EXPECT_DOUBLE_EQ(result[1], 0.0);
  EXPECT_DOUBLE_EQ(result[2], 0.0);

  // Set positions for a subset
  std::vector<std::size_t> indices = {0, 2};
  Eigen::VectorXd subPositions(2);
  subPositions << 1.0, 3.0;
  skel->setPositions(indices, subPositions);

  result = skel->getPositions();
  EXPECT_DOUBLE_EQ(result[0], 1.0);
  EXPECT_DOUBLE_EQ(result[1], 0.0); // Unchanged
  EXPECT_DOUBLE_EQ(result[2], 3.0);

  // Get positions for a subset
  Eigen::VectorXd subResult = skel->getPositions(indices);
  EXPECT_EQ(subResult.size(), 2);
  EXPECT_DOUBLE_EQ(subResult[0], 1.0);
  EXPECT_DOUBLE_EQ(subResult[1], 3.0);
}

//==============================================================================
// Test position limits
TEST(MetaSkeletonTests, PositionLimits)
{
  auto skel = createChainSkeleton(3);

  // Set individual limits
  skel->setPositionLowerLimit(0, -1.0);
  EXPECT_DOUBLE_EQ(skel->getPositionLowerLimit(0), -1.0);

  skel->setPositionUpperLimit(0, 1.0);
  EXPECT_DOUBLE_EQ(skel->getPositionUpperLimit(0), 1.0);

  // Set all lower limits
  Eigen::VectorXd lowerLimits(3);
  lowerLimits << -0.5, -1.0, -1.5;
  skel->setPositionLowerLimits(lowerLimits);

  Eigen::VectorXd result = skel->getPositionLowerLimits();
  EXPECT_DOUBLE_EQ(result[0], -0.5);
  EXPECT_DOUBLE_EQ(result[1], -1.0);
  EXPECT_DOUBLE_EQ(result[2], -1.5);

  // Set all upper limits
  Eigen::VectorXd upperLimits(3);
  upperLimits << 0.5, 1.0, 1.5;
  skel->setPositionUpperLimits(upperLimits);

  result = skel->getPositionUpperLimits();
  EXPECT_DOUBLE_EQ(result[0], 0.5);
  EXPECT_DOUBLE_EQ(result[1], 1.0);
  EXPECT_DOUBLE_EQ(result[2], 1.5);

  // Set limits for a subset
  std::vector<std::size_t> indices = {0, 2};
  Eigen::VectorXd subLower(2);
  subLower << -2.0, -3.0;
  skel->setPositionLowerLimits(indices, subLower);

  Eigen::VectorXd subUpper(2);
  subUpper << 2.0, 3.0;
  skel->setPositionUpperLimits(indices, subUpper);

  result = skel->getPositionLowerLimits(indices);
  EXPECT_DOUBLE_EQ(result[0], -2.0);
  EXPECT_DOUBLE_EQ(result[1], -3.0);

  result = skel->getPositionUpperLimits(indices);
  EXPECT_DOUBLE_EQ(result[0], 2.0);
  EXPECT_DOUBLE_EQ(result[1], 3.0);
}

//==============================================================================
// Test velocity operations
TEST(MetaSkeletonTests, VelocityOperations)
{
  auto skel = createChainSkeleton(3);

  // Set individual velocity
  skel->setVelocity(0, 1.5);
  EXPECT_DOUBLE_EQ(skel->getVelocity(0), 1.5);

  // Set all velocities
  Eigen::VectorXd velocities(3);
  velocities << 0.1, 0.2, 0.3;
  skel->setVelocities(velocities);

  Eigen::VectorXd result = skel->getVelocities();
  EXPECT_DOUBLE_EQ(result[0], 0.1);
  EXPECT_DOUBLE_EQ(result[1], 0.2);
  EXPECT_DOUBLE_EQ(result[2], 0.3);

  // Reset velocities
  skel->resetVelocities();
  result = skel->getVelocities();
  EXPECT_DOUBLE_EQ(result[0], 0.0);
  EXPECT_DOUBLE_EQ(result[1], 0.0);
  EXPECT_DOUBLE_EQ(result[2], 0.0);

  // Set velocities for a subset
  std::vector<std::size_t> indices = {1};
  Eigen::VectorXd subVel(1);
  subVel << 5.0;
  skel->setVelocities(indices, subVel);

  result = skel->getVelocities();
  EXPECT_DOUBLE_EQ(result[1], 5.0);
}

//==============================================================================
// Test velocity limits
TEST(MetaSkeletonTests, VelocityLimits)
{
  auto skel = createChainSkeleton(3);

  // Set individual limits
  skel->setVelocityLowerLimit(0, -10.0);
  EXPECT_DOUBLE_EQ(skel->getVelocityLowerLimit(0), -10.0);

  skel->setVelocityUpperLimit(0, 10.0);
  EXPECT_DOUBLE_EQ(skel->getVelocityUpperLimit(0), 10.0);

  // Set all limits
  Eigen::VectorXd lowerLimits(3);
  lowerLimits << -5.0, -10.0, -15.0;
  skel->setVelocityLowerLimits(lowerLimits);

  Eigen::VectorXd result = skel->getVelocityLowerLimits();
  EXPECT_DOUBLE_EQ(result[0], -5.0);
  EXPECT_DOUBLE_EQ(result[1], -10.0);
  EXPECT_DOUBLE_EQ(result[2], -15.0);
}

//==============================================================================
// Test acceleration operations
TEST(MetaSkeletonTests, AccelerationOperations)
{
  auto skel = createChainSkeleton(3);

  // Set individual acceleration
  skel->setAcceleration(0, 2.5);
  EXPECT_DOUBLE_EQ(skel->getAcceleration(0), 2.5);

  // Set all accelerations
  Eigen::VectorXd accelerations(3);
  accelerations << 1.0, 2.0, 3.0;
  skel->setAccelerations(accelerations);

  Eigen::VectorXd result = skel->getAccelerations();
  EXPECT_DOUBLE_EQ(result[0], 1.0);
  EXPECT_DOUBLE_EQ(result[1], 2.0);
  EXPECT_DOUBLE_EQ(result[2], 3.0);

  // Reset accelerations
  skel->resetAccelerations();
  result = skel->getAccelerations();
  EXPECT_DOUBLE_EQ(result[0], 0.0);
  EXPECT_DOUBLE_EQ(result[1], 0.0);
  EXPECT_DOUBLE_EQ(result[2], 0.0);
}

//==============================================================================
// Test acceleration limits
TEST(MetaSkeletonTests, AccelerationLimits)
{
  auto skel = createChainSkeleton(3);

  // Set individual limits
  skel->setAccelerationLowerLimit(0, -100.0);
  EXPECT_DOUBLE_EQ(skel->getAccelerationLowerLimit(0), -100.0);

  skel->setAccelerationUpperLimit(0, 100.0);
  EXPECT_DOUBLE_EQ(skel->getAccelerationUpperLimit(0), 100.0);

  // Set all limits
  Eigen::VectorXd limits(3);
  limits << -50.0, -100.0, -150.0;
  skel->setAccelerationLowerLimits(limits);

  Eigen::VectorXd result = skel->getAccelerationLowerLimits();
  EXPECT_DOUBLE_EQ(result[0], -50.0);
  EXPECT_DOUBLE_EQ(result[1], -100.0);
  EXPECT_DOUBLE_EQ(result[2], -150.0);
}

//==============================================================================
// Test force operations
TEST(MetaSkeletonTests, ForceOperations)
{
  auto skel = createChainSkeleton(3);

  // Set individual force
  skel->setForce(0, 10.0);
  EXPECT_DOUBLE_EQ(skel->getForce(0), 10.0);

  // Set all forces
  Eigen::VectorXd forces(3);
  forces << 1.0, 2.0, 3.0;
  skel->setForces(forces);

  Eigen::VectorXd result = skel->getForces();
  EXPECT_DOUBLE_EQ(result[0], 1.0);
  EXPECT_DOUBLE_EQ(result[1], 2.0);
  EXPECT_DOUBLE_EQ(result[2], 3.0);

  // Reset forces
  skel->resetGeneralizedForces();
  result = skel->getForces();
  EXPECT_DOUBLE_EQ(result[0], 0.0);
  EXPECT_DOUBLE_EQ(result[1], 0.0);
  EXPECT_DOUBLE_EQ(result[2], 0.0);
}

//==============================================================================
// Test force limits
TEST(MetaSkeletonTests, ForceLimits)
{
  auto skel = createChainSkeleton(3);

  // Set individual limits
  skel->setForceLowerLimit(0, -50.0);
  EXPECT_DOUBLE_EQ(skel->getForceLowerLimit(0), -50.0);

  skel->setForceUpperLimit(0, 50.0);
  EXPECT_DOUBLE_EQ(skel->getForceUpperLimit(0), 50.0);

  // Set all limits
  Eigen::VectorXd lowerLimits(3);
  lowerLimits << -20.0, -30.0, -40.0;
  skel->setForceLowerLimits(lowerLimits);

  Eigen::VectorXd result = skel->getForceLowerLimits();
  EXPECT_DOUBLE_EQ(result[0], -20.0);
  EXPECT_DOUBLE_EQ(result[1], -30.0);
  EXPECT_DOUBLE_EQ(result[2], -40.0);
}

//==============================================================================
// Test velocity changes and constraint impulses
TEST(MetaSkeletonTests, VelocityChangesAndImpulses)
{
  auto skel = createChainSkeleton(3);

  // Get velocity changes (should be zero initially)
  Eigen::VectorXd velChanges = skel->getVelocityChanges();
  EXPECT_EQ(velChanges.size(), 3);
  EXPECT_DOUBLE_EQ(velChanges[0], 0.0);
  EXPECT_DOUBLE_EQ(velChanges[1], 0.0);
  EXPECT_DOUBLE_EQ(velChanges[2], 0.0);

  // Set and get constraint impulses
  Eigen::VectorXd impulses(3);
  impulses << 0.1, 0.2, 0.3;
  skel->setJointConstraintImpulses(impulses);

  Eigen::VectorXd result = skel->getJointConstraintImpulses();
  EXPECT_DOUBLE_EQ(result[0], 0.1);
  EXPECT_DOUBLE_EQ(result[1], 0.2);
  EXPECT_DOUBLE_EQ(result[2], 0.3);
}

//==============================================================================
// Test mass computation
TEST(MetaSkeletonTests, MassComputation)
{
  auto skel = createChainSkeleton(3);

  // Each body has mass 1.0, so total should be 3.0
  double totalMass = skel->getMass();
  EXPECT_DOUBLE_EQ(totalMass, 3.0);
}

//==============================================================================
// Test mass matrix (basic check)
TEST(MetaSkeletonTests, MassMatrix)
{
  auto skel = createChainSkeleton(3);

  const Eigen::MatrixXd& M = skel->getMassMatrix();

  // Mass matrix should be NxN where N is number of DOFs
  EXPECT_EQ(M.rows(), 3);
  EXPECT_EQ(M.cols(), 3);

  // Mass matrix should be symmetric
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(M(i, j), M(j, i), 1e-10);
    }
  }

  // Mass matrix should be positive semi-definite (all eigenvalues >= 0)
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(M);
  Eigen::VectorXd eigenvalues = eigensolver.eigenvalues();
  for (int i = 0; i < eigenvalues.size(); ++i) {
    EXPECT_GE(eigenvalues[i], -1e-10); // Allow small numerical error
  }
}

//==============================================================================
// Test inverse mass matrix
TEST(MetaSkeletonTests, InverseMassMatrix)
{
  auto skel = createChainSkeleton(3);

  const Eigen::MatrixXd& M = skel->getMassMatrix();
  const Eigen::MatrixXd& Minv = skel->getInvMassMatrix();

  // M * Minv should be identity
  Eigen::MatrixXd product = M * Minv;
  Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(3, 3);

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(product(i, j), identity(i, j), 1e-8);
    }
  }
}

//==============================================================================
// Test augmented mass matrix
TEST(MetaSkeletonTests, AugmentedMassMatrix)
{
  auto skel = createChainSkeleton(3);

  const Eigen::MatrixXd& augM = skel->getAugMassMatrix();

  // Augmented mass matrix should also be NxN
  EXPECT_EQ(augM.rows(), 3);
  EXPECT_EQ(augM.cols(), 3);

  // Should be symmetric
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(augM(i, j), augM(j, i), 1e-10);
    }
  }
}

//==============================================================================
// Test Coriolis and gravity forces
TEST(MetaSkeletonTests, CoriolisAndGravityForces)
{
  auto skel = createChainSkeleton(3);

  // Set non-zero velocities to generate Coriolis forces
  Eigen::VectorXd velocities(3);
  velocities << 1.0, 2.0, 3.0;
  skel->setVelocities(velocities);

  // Coriolis forces
  const Eigen::VectorXd& coriolis = skel->getCoriolisForces();
  EXPECT_EQ(coriolis.size(), 3);

  // Gravity forces
  const Eigen::VectorXd& gravity = skel->getGravityForces();
  EXPECT_EQ(gravity.size(), 3);

  // Combined
  const Eigen::VectorXd& combined = skel->getCoriolisAndGravityForces();
  EXPECT_EQ(combined.size(), 3);
}

//==============================================================================
// Test external and constraint forces
TEST(MetaSkeletonTests, ExternalAndConstraintForces)
{
  auto skel = createChainSkeleton(3);

  // External forces should be zero initially
  const Eigen::VectorXd& external = skel->getExternalForces();
  EXPECT_EQ(external.size(), 3);

  // Constraint forces should be zero initially
  const Eigen::VectorXd& constraint = skel->getConstraintForces();
  EXPECT_EQ(constraint.size(), 3);

  // Clear external forces (should not crash)
  skel->clearExternalForces();

  // Clear internal forces (should not crash)
  skel->clearInternalForces();
}

//==============================================================================
// Test energy computation
TEST(MetaSkeletonTests, EnergyComputation)
{
  auto skel = createChainSkeleton(3);

  // Set positions and velocities
  Eigen::VectorXd positions(3);
  positions << 0.1, 0.2, 0.3;
  skel->setPositions(positions);

  Eigen::VectorXd velocities(3);
  velocities << 1.0, 2.0, 3.0;
  skel->setVelocities(velocities);

  // Kinetic energy (should be positive with non-zero velocities)
  double KE = skel->computeKineticEnergy();
  EXPECT_GT(KE, 0.0);

  // Potential energy
  double PE = skel->computePotentialEnergy();
  // PE can be positive, negative, or zero depending on configuration

  // Lagrangian = KE - PE
  double L = skel->computeLagrangian();
  EXPECT_NEAR(L, KE - PE, 1e-10);
}

//==============================================================================
// Test COM (Center of Mass) computation
TEST(MetaSkeletonTests, COMComputation)
{
  auto skel = createChainSkeleton(3);

  // Get COM position
  Eigen::Vector3d com = skel->getCOM();
  // COM should be a valid 3D point
  EXPECT_FALSE(std::isnan(com[0]));
  EXPECT_FALSE(std::isnan(com[1]));
  EXPECT_FALSE(std::isnan(com[2]));

  // Get COM velocity (should be zero with zero velocities)
  Eigen::Vector3d comVel = skel->getCOMLinearVelocity();
  EXPECT_NEAR(comVel.norm(), 0.0, 1e-10);

  // Set velocities and check again
  Eigen::VectorXd velocities(3);
  velocities << 1.0, 0.0, 0.0;
  skel->setVelocities(velocities);

  comVel = skel->getCOMLinearVelocity();
  // Now COM velocity should be non-zero (in general)

  // Get COM acceleration
  Eigen::Vector3d comAcc = skel->getCOMLinearAcceleration();
  EXPECT_FALSE(std::isnan(comAcc[0]));
  EXPECT_FALSE(std::isnan(comAcc[1]));
  EXPECT_FALSE(std::isnan(comAcc[2]));

  // Get COM spatial velocity
  Eigen::Vector6d comSpatialVel = skel->getCOMSpatialVelocity();
  EXPECT_EQ(comSpatialVel.size(), 6);

  // Get COM spatial acceleration
  Eigen::Vector6d comSpatialAcc = skel->getCOMSpatialAcceleration();
  EXPECT_EQ(comSpatialAcc.size(), 6);
}

//==============================================================================
// Test Jacobian computation
TEST(MetaSkeletonTests, JacobianComputation)
{
  auto skel = createChainSkeleton(3);

  auto* endBody = skel->getBodyNode(2);
  ASSERT_NE(endBody, nullptr);

  // Get Jacobian for the end effector
  math::Jacobian J = skel->getJacobian(endBody);

  // Jacobian should be 6xN (6 = spatial dimension, N = number of DOFs)
  EXPECT_EQ(J.rows(), 6);
  EXPECT_EQ(J.cols(), 3);

  // Get world Jacobian
  math::Jacobian Jworld = skel->getWorldJacobian(endBody);
  EXPECT_EQ(Jworld.rows(), 6);
  EXPECT_EQ(Jworld.cols(), 3);

  // Get linear Jacobian
  math::LinearJacobian Jlin = skel->getLinearJacobian(endBody);
  EXPECT_EQ(Jlin.rows(), 3);
  EXPECT_EQ(Jlin.cols(), 3);

  // Get angular Jacobian
  math::AngularJacobian Jang = skel->getAngularJacobian(endBody);
  EXPECT_EQ(Jang.rows(), 3);
  EXPECT_EQ(Jang.cols(), 3);
}

//==============================================================================
// Test Jacobian with offset
TEST(MetaSkeletonTests, JacobianWithOffset)
{
  auto skel = createChainSkeleton(3);

  auto* endBody = skel->getBodyNode(2);
  ASSERT_NE(endBody, nullptr);

  Eigen::Vector3d offset(0.1, 0.2, 0.3);

  // Get Jacobian with offset
  math::Jacobian Joffset = skel->getJacobian(endBody, offset);
  EXPECT_EQ(Joffset.rows(), 6);
  EXPECT_EQ(Joffset.cols(), 3);

  // Get world Jacobian with offset
  math::Jacobian JworldOffset = skel->getWorldJacobian(endBody, offset);
  EXPECT_EQ(JworldOffset.rows(), 6);
  EXPECT_EQ(JworldOffset.cols(), 3);

  // Get linear Jacobian with offset
  math::LinearJacobian JlinOffset = skel->getLinearJacobian(endBody, offset);
  EXPECT_EQ(JlinOffset.rows(), 3);
  EXPECT_EQ(JlinOffset.cols(), 3);
}

//==============================================================================
// Test Jacobian time derivatives
TEST(MetaSkeletonTests, JacobianDerivatives)
{
  auto skel = createChainSkeleton(3);

  // Set velocities to make derivatives non-trivial
  Eigen::VectorXd velocities(3);
  velocities << 1.0, 2.0, 1.5;
  skel->setVelocities(velocities);

  auto* endBody = skel->getBodyNode(2);
  ASSERT_NE(endBody, nullptr);

  // Get spatial Jacobian derivative
  math::Jacobian Jdot = skel->getJacobianSpatialDeriv(endBody);
  EXPECT_EQ(Jdot.rows(), 6);
  EXPECT_EQ(Jdot.cols(), 3);

  // Get classical Jacobian derivative
  math::Jacobian JdotClassic = skel->getJacobianClassicDeriv(endBody);
  EXPECT_EQ(JdotClassic.rows(), 6);
  EXPECT_EQ(JdotClassic.cols(), 3);

  // Get linear Jacobian derivative
  math::LinearJacobian JlinDot = skel->getLinearJacobianDeriv(endBody);
  EXPECT_EQ(JlinDot.rows(), 3);
  EXPECT_EQ(JlinDot.cols(), 3);

  // Get angular Jacobian derivative
  math::AngularJacobian JangDot = skel->getAngularJacobianDeriv(endBody);
  EXPECT_EQ(JangDot.rows(), 3);
  EXPECT_EQ(JangDot.cols(), 3);
}

//==============================================================================
TEST(MetaSkeletonTests, FrameBasedComAndJacobianDerivs)
{
  auto skel = createMixedSkeleton("frame_derivs");
  skel->setVelocities(Eigen::VectorXd::Constant(skel->getNumDofs(), 0.2));

  auto* root = skel->getBodyNode(0);
  auto* endBody = skel->getBodyNode(skel->getNumBodyNodes() - 1u);
  ASSERT_NE(root, nullptr);
  ASSERT_NE(endBody, nullptr);

  const auto com = skel->getCOM(root);
  const auto comVel = skel->getCOMLinearVelocity(root);
  const auto comAcc = skel->getCOMLinearAcceleration(root);
  EXPECT_TRUE(com.array().isFinite().all());
  EXPECT_TRUE(comVel.array().isFinite().all());
  EXPECT_TRUE(comAcc.array().isFinite().all());

  const Eigen::Vector3d offset(0.01, -0.02, 0.03);
  const auto Jclassic = skel->getJacobianClassicDeriv(endBody, offset, root);
  EXPECT_EQ(Jclassic.cols(), static_cast<int>(skel->getNumDofs()));
  const auto Jlinear = skel->getLinearJacobianDeriv(endBody, root);
  EXPECT_EQ(Jlinear.cols(), static_cast<int>(skel->getNumDofs()));
  const auto Jangular = skel->getAngularJacobianDeriv(endBody, root);
  EXPECT_EQ(Jangular.cols(), static_cast<int>(skel->getNumDofs()));
}

//==============================================================================
// Test COM Jacobians
TEST(MetaSkeletonTests, COMJacobians)
{
  auto skel = createChainSkeleton(3);

  // Get COM Jacobian
  math::Jacobian Jcom = skel->getCOMJacobian();
  EXPECT_EQ(Jcom.rows(), 6);
  EXPECT_EQ(Jcom.cols(), 3);

  // Get COM linear Jacobian
  math::LinearJacobian JcomLin = skel->getCOMLinearJacobian();
  EXPECT_EQ(JcomLin.rows(), 3);
  EXPECT_EQ(JcomLin.cols(), 3);

  // Get COM Jacobian spatial derivative
  math::Jacobian JcomDot = skel->getCOMJacobianSpatialDeriv();
  EXPECT_EQ(JcomDot.rows(), 6);
  EXPECT_EQ(JcomDot.cols(), 3);

  // Get COM linear Jacobian derivative
  math::LinearJacobian JcomLinDot = skel->getCOMLinearJacobianDeriv();
  EXPECT_EQ(JcomLinDot.rows(), 3);
  EXPECT_EQ(JcomLinDot.cols(), 3);
}

//==============================================================================
// Test iteration methods
TEST(MetaSkeletonTests, IterationMethods)
{
  auto skel = createChainSkeleton(3);

  // Count body nodes using eachBodyNode
  int bodyCount = 0;
  skel->eachBodyNode([&bodyCount](const BodyNode*) { ++bodyCount; });
  EXPECT_EQ(bodyCount, 3);

  // Early exit from iteration
  int visitedBefore = 0;
  skel->eachBodyNode([&visitedBefore](const BodyNode*) -> bool {
    ++visitedBefore;
    return visitedBefore < 2; // Stop after visiting 2 bodies
  });
  EXPECT_EQ(visitedBefore, 2);

  // Count joints using eachJoint
  int jointCount = 0;
  skel->eachJoint([&jointCount](const Joint*) { ++jointCount; });
  EXPECT_EQ(jointCount, 3);

  // Count DOFs using eachDof
  int dofCount = 0;
  skel->eachDof([&dofCount](const DegreeOfFreedom*) { ++dofCount; });
  EXPECT_EQ(dofCount, 3);
}

//==============================================================================
// Test color setting
TEST(MetaSkeletonTests, ColorSetting)
{
  auto skel = createChainSkeleton(3);

  // Add shapes to body nodes for color testing
  for (std::size_t i = 0; i < skel->getNumBodyNodes(); ++i) {
    auto* body = skel->getBodyNode(i);
    body->createShapeNodeWith<VisualAspect>(
        std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  }

  // Set RGB color
  Eigen::Vector3d rgb(1.0, 0.0, 0.0); // Red
  skel->setColor(rgb);

  // Set RGBA color
  Eigen::Vector4d rgba(0.0, 1.0, 0.0, 0.5); // Green with 50% alpha
  skel->setColor(rgba);

  // Set alpha only
  skel->setAlpha(0.8);
}

//==============================================================================
// Test mixed joint skeleton
TEST(MetaSkeletonTests, MixedJointSkeleton)
{
  auto skel = createMixedSkeleton();

  // FreeJoint (6) + Revolute (1) + Ball (3) = 10 DOFs
  EXPECT_EQ(skel->getNumDofs(), 10u);
  EXPECT_EQ(skel->getNumBodyNodes(), 3u);
  EXPECT_EQ(skel->getNumJoints(), 3u);

  // Total mass: 2.0 + 1.0 + 0.5 = 3.5
  EXPECT_DOUBLE_EQ(skel->getMass(), 3.5);

  // Set all positions
  Eigen::VectorXd positions = Eigen::VectorXd::Zero(10);
  positions << 0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 0.5, 0.1, 0.2, 0.3;
  skel->setPositions(positions);

  Eigen::VectorXd result = skel->getPositions();
  EXPECT_EQ(result.size(), 10);
  for (int i = 0; i < 10; ++i) {
    EXPECT_DOUBLE_EQ(result[i], positions[i]);
  }
}

//==============================================================================
// Test getLockableReference
TEST(MetaSkeletonTests, LockableReference)
{
  auto skel = createChainSkeleton(3);

  auto lockable = skel->getLockableReference();
  ASSERT_NE(lockable, nullptr);

  // Just check that we can lock and unlock
  lockable->lock();
  lockable->unlock();
}

//==============================================================================
// Test cloneMetaSkeleton
TEST(MetaSkeletonTests, CloneMetaSkeleton)
{
  auto skel = createChainSkeleton(3, "original");

  // Set some state
  Eigen::VectorXd positions(3);
  positions << 0.1, 0.2, 0.3;
  skel->setPositions(positions);

  // Clone
  MetaSkeletonPtr clone = skel->cloneMetaSkeleton("cloned");

  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->getName(), "cloned");
  EXPECT_EQ(clone->getNumDofs(), skel->getNumDofs());
  EXPECT_EQ(clone->getNumBodyNodes(), skel->getNumBodyNodes());

  // State should be copied
  Eigen::VectorXd clonePositions = clone->getPositions();
  for (int i = 0; i < 3; ++i) {
    EXPECT_DOUBLE_EQ(clonePositions[i], positions[i]);
  }

  // Clone with same name
  MetaSkeletonPtr clone2 = skel->cloneMetaSkeleton("original");
  EXPECT_EQ(clone2->getName(), "original");
}

TEST(MetaSkeletonTests, CommandSubsetAccessors)
{
  auto skel = createChainSkeleton(4);
  std::vector<std::size_t> indices = {0u, 2u};

  Eigen::VectorXd commands(2);
  commands << 1.2, -0.7;
  skel->setCommands(indices, commands);

  Eigen::VectorXd result = skel->getCommands(indices);
  EXPECT_EQ(result.size(), 2);
  EXPECT_NEAR(result[0], 1.2, 1e-12);
  EXPECT_NEAR(result[1], -0.7, 1e-12);
}

TEST(MetaSkeletonTests, JacobianRelativeToSameNode)
{
  auto skel = createChainSkeleton(2);
  auto* body = skel->getBodyNode(1);
  ASSERT_NE(body, nullptr);

  const auto J = skel->getJacobian(body, body, Frame::World());
  EXPECT_TRUE(J.isZero());

  const Eigen::Vector3d offset(0.05, -0.02, 0.03);
  const auto Joffset = skel->getJacobian(body, offset, body, Frame::World());
  EXPECT_TRUE(Joffset.isZero());
}

TEST(MetaSkeletonTests, SpatialDerivativeRelativeToSameNode)
{
  auto skel = createChainSkeleton(2);
  auto* body = skel->getBodyNode(1);
  ASSERT_NE(body, nullptr);

  const auto dJ = skel->getJacobianSpatialDeriv(body, body, Frame::World());
  EXPECT_TRUE(dJ.isZero());

  const Eigen::Vector3d offset(0.02, 0.01, -0.04);
  const auto dJoffset
      = skel->getJacobianSpatialDeriv(body, offset, body, Frame::World());
  EXPECT_TRUE(dJoffset.isZero());
}

//=============================================================================
TEST(MetaSkeletonTests, JacobianInNodeCoordinates)
{
  auto skel = createChainSkeleton(2);
  auto* root = skel->getBodyNode(0);
  auto* endBody = skel->getBodyNode(1);
  ASSERT_NE(root, nullptr);
  ASSERT_NE(endBody, nullptr);

  const auto J = skel->getJacobian(endBody, root, endBody);
  EXPECT_EQ(J.rows(), 6);
  EXPECT_EQ(J.cols(), static_cast<int>(skel->getNumDofs()));

  const Eigen::Vector3d offset(0.02, -0.01, 0.03);
  const auto Joffset = skel->getJacobian(endBody, offset, root, endBody);
  EXPECT_EQ(Joffset.rows(), 6);
  EXPECT_EQ(Joffset.cols(), static_cast<int>(skel->getNumDofs()));
}

//=============================================================================
TEST(MetaSkeletonTests, JacobianInWorldCoordinates)
{
  auto skel = createChainSkeleton(2);
  auto* root = skel->getBodyNode(0);
  auto* endBody = skel->getBodyNode(1);
  ASSERT_NE(root, nullptr);
  ASSERT_NE(endBody, nullptr);

  const auto J = skel->getJacobian(endBody, root, Frame::World());
  EXPECT_EQ(J.rows(), 6);
  EXPECT_EQ(J.cols(), static_cast<int>(skel->getNumDofs()));

  const Eigen::Vector3d offset(0.01, 0.02, -0.02);
  const auto Joffset = skel->getJacobian(endBody, offset, root, Frame::World());
  EXPECT_EQ(Joffset.rows(), 6);
  EXPECT_EQ(Joffset.cols(), static_cast<int>(skel->getNumDofs()));
}

TEST(MetaSkeletonTests, CenterOfMassVelocityAndAcceleration)
{
  auto skel = createChainSkeleton(3, "com_vel_acc");

  skel->setPositions(Eigen::VectorXd::Constant(3, 0.2));
  skel->setVelocities(Eigen::VectorXd::Constant(3, -0.1));
  skel->setAccelerations(Eigen::VectorXd::Constant(3, 0.05));

  const auto com = skel->getCOM();
  EXPECT_TRUE(com.allFinite());

  const auto comVel = skel->getCOMLinearVelocity();
  EXPECT_TRUE(comVel.allFinite());

  const auto comAcc = skel->getCOMLinearAcceleration();
  EXPECT_TRUE(comAcc.allFinite());
}

TEST(MetaSkeletonTests, JacobianDifferentFrames)
{
  auto skel = createChainSkeleton(2, "jac_frames");
  auto* endBody = skel->getBodyNode(1);
  ASSERT_NE(endBody, nullptr);

  const auto Jworld = skel->getJacobian(endBody, Frame::World());
  EXPECT_EQ(Jworld.cols(), static_cast<int>(skel->getNumDofs()));

  const auto Jlocal = skel->getJacobian(endBody, endBody);
  EXPECT_EQ(Jlocal.cols(), static_cast<int>(skel->getNumDofs()));
}

TEST(MetaSkeletonTests, EnergyAndLagrangianRelationship)
{
  auto skel = createChainSkeleton(2, "energy_lagrangian");

  skel->setVelocities(Eigen::VectorXd::Constant(2, 0.25));

  const double kinetic = skel->computeKineticEnergy();
  const double potential = skel->computePotentialEnergy();
  const double lagrangian = skel->computeLagrangian();

  EXPECT_TRUE(std::isfinite(kinetic));
  EXPECT_TRUE(std::isfinite(potential));
  EXPECT_NEAR(lagrangian, kinetic - potential, 1e-10);
}

TEST(MetaSkeletonTests, VelocityAndAccelerationLimitVectors)
{
  auto skel = createChainSkeleton(3, "limits_vectors");

  Eigen::VectorXd velocityUpper(3);
  velocityUpper << 2.0, 3.0, 4.0;
  skel->setVelocityUpperLimits(velocityUpper);
  const auto velocityUpperOut = skel->getVelocityUpperLimits();
  EXPECT_DOUBLE_EQ(velocityUpperOut[0], 2.0);
  EXPECT_DOUBLE_EQ(velocityUpperOut[1], 3.0);
  EXPECT_DOUBLE_EQ(velocityUpperOut[2], 4.0);

  Eigen::VectorXd accelLower(3);
  accelLower << -1.0, -2.0, -3.0;
  skel->setAccelerationLowerLimits(accelLower);
  const auto accelLowerOut = skel->getAccelerationLowerLimits();
  EXPECT_DOUBLE_EQ(accelLowerOut[0], -1.0);
  EXPECT_DOUBLE_EQ(accelLowerOut[1], -2.0);
  EXPECT_DOUBLE_EQ(accelLowerOut[2], -3.0);

  Eigen::VectorXd accelUpper(3);
  accelUpper << 1.5, 2.5, 3.5;
  skel->setAccelerationUpperLimits(accelUpper);
  const auto accelUpperOut = skel->getAccelerationUpperLimits();
  EXPECT_DOUBLE_EQ(accelUpperOut[0], 1.5);
  EXPECT_DOUBLE_EQ(accelUpperOut[1], 2.5);
  EXPECT_DOUBLE_EQ(accelUpperOut[2], 3.5);
}
