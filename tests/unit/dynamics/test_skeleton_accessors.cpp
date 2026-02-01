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

#include "../../helpers/dynamics_helpers.hpp"

#include <dart/dynamics/ball_joint.hpp>
#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/detail/skeleton_aspect.hpp>
#include <dart/dynamics/end_effector.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/group.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/mimic_dof_properties.hpp>
#include <dart/dynamics/prismatic_joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/common/deprecated.hpp>

#include <dart/all.hpp>

#if defined(__clang__) && !defined(DART_COMPILER_CLANG)
  #define DART_COMPILER_CLANG
#endif
#if defined(__GNUC__) && !defined(DART_COMPILER_GCC) && !defined(__clang__)
  #define DART_COMPILER_GCC
#endif

#include <dart/common/diagnostics.hpp>
#include <dart/common/exception.hpp>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <array>
#include <memory>
#include <vector>

#include <cmath>

using namespace dart::dynamics;

namespace {

class TestSkeleton : public Skeleton
{
public:
  static std::shared_ptr<TestSkeleton> create(const std::string& name)
  {
    auto skeleton = std::shared_ptr<TestSkeleton>(
        new TestSkeleton(AspectPropertiesData(name)));
    skeleton->setPtr(skeleton);
    return skeleton;
  }

  using Skeleton::dirtyArticulatedInertia;
  using Skeleton::updateArticulatedInertia;

protected:
  explicit TestSkeleton(const AspectPropertiesData& properties)
    : Skeleton(properties)
  {
  }
};

} // namespace

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
TEST(SkeletonAccessors, IntegratePositionsWithZeroDofs)
{
  auto skeleton = Skeleton::create("integrate_zero_dof");
  skeleton->createJointAndBodyNodePair<WeldJoint>();

  Eigen::VectorXd velocityChanges;
  skeleton->integratePositions(0.01, velocityChanges);
  EXPECT_EQ(skeleton->getNumDofs(), 0u);
}

//==============================================================================
TEST(SkeletonAccessors, MassMatrixVariants)
{
  auto skel = Skeleton::create("mass_variants");
  auto pair1 = skel->createJointAndBodyNodePair<RevoluteJoint>();
  auto pair2 = skel->createJointAndBodyNodePair<PrismaticJoint>(pair1.second);
  pair1.second->setMass(2.0);
  pair2.second->setMass(1.5);

  skel->setPositions(Eigen::Vector2d(0.3, 0.1));
  skel->setVelocities(Eigen::Vector2d(0.5, -0.2));

  const auto& massMatrix = skel->getMassMatrix();
  EXPECT_EQ(massMatrix.rows(), 2);
  EXPECT_EQ(massMatrix.cols(), 2);
  EXPECT_TRUE(massMatrix.allFinite());

  const auto& augMassMatrix = skel->getAugMassMatrix();
  EXPECT_TRUE(augMassMatrix.allFinite());

  const auto& invMassMatrix = skel->getInvMassMatrix();
  EXPECT_TRUE(invMassMatrix.allFinite());

  const auto& invAugMassMatrix = skel->getInvAugMassMatrix();
  EXPECT_TRUE(invAugMassMatrix.allFinite());

  const auto& gravForces = skel->getGravityForces();
  EXPECT_TRUE(gravForces.allFinite());

  const auto& coriolisForces = skel->getCoriolisForces();
  EXPECT_TRUE(coriolisForces.allFinite());

  const auto& coriolisGravForces = skel->getCoriolisAndGravityForces();
  EXPECT_TRUE(coriolisGravForces.allFinite());

  const auto& extForces = skel->getExternalForces();
  EXPECT_TRUE(extForces.allFinite());
}

//=============================================================================
TEST(SkeletonAccessors, MassAndForceAccessorsWithInertia)
{
  auto skel = Skeleton::create("mass_force_inertia");

  RevoluteJoint::Properties rootJointProps;
  rootJointProps.mAxis = Eigen::Vector3d::UnitZ();
  BodyNode::Properties rootBodyProps;
  rootBodyProps.mInertia.setMass(2.0);
  rootBodyProps.mInertia.setMoment(0.2, 0.3, 0.4, 0.0, 0.0, 0.0);

  auto rootPair = skel->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, rootJointProps, rootBodyProps);

  PrismaticJoint::Properties childJointProps;
  childJointProps.mAxis = Eigen::Vector3d::UnitX();
  BodyNode::Properties childBodyProps;
  childBodyProps.mInertia.setMass(1.5);
  childBodyProps.mInertia.setMoment(0.1, 0.15, 0.2, 0.0, 0.0, 0.0);

  auto childPair = skel->createJointAndBodyNodePair<PrismaticJoint>(
      rootPair.second, childJointProps, childBodyProps);

  skel->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  skel->setPositions(Eigen::Vector2d(0.1, -0.2));
  skel->setVelocities(Eigen::Vector2d(0.3, 0.4));
  skel->setAccelerations(Eigen::Vector2d(-0.1, 0.2));
  skel->setForces(Eigen::Vector2d(0.5, -0.6));
  skel->setCommands(Eigen::Vector2d(0.2, 0.1));

  childPair.second->addExtForce(
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.1, 0.0, 0.0),
      false,
      false);

  const auto& massMatrix = skel->getMassMatrix();
  EXPECT_EQ(massMatrix.rows(), 2);
  EXPECT_EQ(massMatrix.cols(), 2);
  EXPECT_TRUE(massMatrix.allFinite());

  EXPECT_TRUE(skel->getAugMassMatrix().allFinite());
  EXPECT_TRUE(skel->getInvMassMatrix().allFinite());
  EXPECT_TRUE(skel->getInvAugMassMatrix().allFinite());

  EXPECT_TRUE(skel->getCoriolisForces().allFinite());
  EXPECT_TRUE(skel->getGravityForces().allFinite());
  EXPECT_TRUE(skel->getCoriolisAndGravityForces().allFinite());
  EXPECT_TRUE(skel->getExternalForces().allFinite());
  EXPECT_TRUE(skel->getConstraintForces().allFinite());
}

//==============================================================================
TEST(SkeletonAccessors, ConstAccessorOverloads)
{
  auto skel = Skeleton::create("const_accessors");

  RevoluteJoint::Properties rootJointProps;
  rootJointProps.mName = "root_joint";
  rootJointProps.mAxis = Eigen::Vector3d::UnitZ();
  BodyNode::Properties rootBodyProps;
  rootBodyProps.mName = "root_body";
  rootBodyProps.mInertia.setMass(2.0);
  rootBodyProps.mInertia.setMoment(0.2, 0.3, 0.4, 0.0, 0.0, 0.0);

  auto rootPair = skel->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, rootJointProps, rootBodyProps);

  PrismaticJoint::Properties childJointProps;
  childJointProps.mName = "child_joint";
  childJointProps.mAxis = Eigen::Vector3d::UnitX();
  BodyNode::Properties childBodyProps;
  childBodyProps.mName = "child_body";
  childBodyProps.mInertia.setMass(1.5);
  childBodyProps.mInertia.setMoment(0.1, 0.15, 0.2, 0.0, 0.0, 0.0);

  auto childPair = skel->createJointAndBodyNodePair<PrismaticJoint>(
      rootPair.second, childJointProps, childBodyProps);

  skel->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  skel->setPositions(Eigen::Vector2d(0.1, -0.2));
  skel->setVelocities(Eigen::Vector2d(0.3, 0.4));
  skel->setAccelerations(Eigen::Vector2d(-0.1, 0.2));
  skel->setForces(Eigen::Vector2d(0.5, -0.6));
  skel->setCommands(Eigen::Vector2d(0.2, 0.1));

  const Skeleton* constSkel = skel.get();

  const auto state = constSkel->getState();
  const auto config = constSkel->getConfiguration();
  (void)state;
  (void)config;

  EXPECT_EQ(constSkel->getPtr().get(), skel.get());
  EXPECT_EQ(constSkel->getSkeleton().get(), skel.get());

  EXPECT_EQ(constSkel->getRootBodyNode(0), rootPair.second);
  EXPECT_EQ(constSkel->getRootJoint(0), rootPair.first);

  EXPECT_EQ(constSkel->getBodyNode(0), rootPair.second);
  EXPECT_EQ(constSkel->getBodyNode("child_body"), childPair.second);
  EXPECT_EQ(constSkel->getJoint(0), rootPair.first);
  EXPECT_EQ(constSkel->getJoint("child_joint"), childPair.first);
  EXPECT_EQ(constSkel->getDof(0), rootPair.first->getDof(0));
  EXPECT_EQ(constSkel->getNumSoftBodyNodes(), 0u);

  DART_SUPPRESS_DEPRECATED_BEGIN
  const auto bodyNodes = constSkel->getBodyNodes();
  DART_SUPPRESS_DEPRECATED_END
  EXPECT_EQ(bodyNodes.size(), 2u);
  EXPECT_EQ(constSkel->getBodyNodes("child_body").size(), 1u);

  DART_SUPPRESS_DEPRECATED_BEGIN
  const auto joints = constSkel->getJoints();
  DART_SUPPRESS_DEPRECATED_END
  EXPECT_EQ(joints.size(), 2u);
  EXPECT_EQ(constSkel->getJoints("child_joint").size(), 1u);

  const auto treeBodyNodes = constSkel->getTreeBodyNodes(0);
  EXPECT_EQ(treeBodyNodes.size(), 2u);
  const auto treeDofs = constSkel->getTreeDofs(0);
  EXPECT_EQ(treeDofs.size(), 2u);

  const auto com = constSkel->getCOM();
  EXPECT_TRUE(com.array().isFinite().all());
  const auto comVel = constSkel->getCOMLinearVelocity();
  EXPECT_TRUE(comVel.array().isFinite().all());
  const auto comAcc = constSkel->getCOMLinearAcceleration();
  EXPECT_TRUE(comAcc.array().isFinite().all());

  const auto& massMatrix = constSkel->getMassMatrix();
  EXPECT_EQ(massMatrix.rows(), 2);
  EXPECT_EQ(massMatrix.cols(), 2);
  EXPECT_TRUE(massMatrix.allFinite());

  EXPECT_TRUE(constSkel->getAugMassMatrix().allFinite());
  EXPECT_TRUE(constSkel->getInvMassMatrix().allFinite());
  EXPECT_TRUE(constSkel->getInvAugMassMatrix().allFinite());
  EXPECT_TRUE(constSkel->getCoriolisForces().allFinite());
  EXPECT_TRUE(constSkel->getGravityForces().allFinite());
  EXPECT_TRUE(constSkel->getCoriolisAndGravityForces().allFinite());
  EXPECT_TRUE(constSkel->getExternalForces().allFinite());
  EXPECT_TRUE(constSkel->getConstraintForces().allFinite());
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

  // Set up mimic relationship: slave mirrors master
  slaveJoint->setActuatorType(0, Joint::MIMIC);
  ::dart::dynamics::MimicDofProperties mimicProps;
  mimicProps.mReferenceJoint = masterJoint;
  mimicProps.mReferenceDofIndex = 0;
  mimicProps.mMultiplier = 1.0;
  mimicProps.mOffset = 0.0;
  mimicProps.mConstraintType = ::dart::dynamics::MimicConstraintType::Motor;
  slaveJoint->setMimicJointDof(0, mimicProps);

  auto clone = original->cloneSkeleton("clone");

  auto* clonedSlave
      = dynamic_cast<RevoluteJoint*>(clone->getJoint("slave_joint"));
  ASSERT_NE(clonedSlave, nullptr);

  const auto clonedProps = clonedSlave->getMimicDofProperties();
  ASSERT_FALSE(clonedProps.empty());
  EXPECT_DOUBLE_EQ(clonedProps[0].mMultiplier, 1.0);
  EXPECT_DOUBLE_EQ(clonedProps[0].mOffset, 0.0);

  // Verify clone has its own master joint
  auto* clonedMaster
      = dynamic_cast<RevoluteJoint*>(clone->getJoint("master_joint"));
  ASSERT_NE(clonedMaster, nullptr);
  EXPECT_NE(clonedMaster, masterJoint);
}

//==============================================================================
TEST(SkeletonAccessors, JointAndDofVectorLookups)
{
  auto skeleton = Skeleton::create("joint_vector_access");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  rootPair.first->setName("root_joint");

  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  childPair.first->setName("child_joint");

  DART_SUPPRESS_DEPRECATED_BEGIN
  const auto joints = skeleton->getJoints();
  DART_SUPPRESS_DEPRECATED_END
  EXPECT_EQ(joints.size(), skeleton->getNumJoints());
  EXPECT_EQ(joints[0], skeleton->getJoint(0));

  const Skeleton* constSkeleton = skeleton.get();
  DART_SUPPRESS_DEPRECATED_BEGIN
  const auto constJoints = constSkeleton->getJoints();
  DART_SUPPRESS_DEPRECATED_END
  EXPECT_EQ(constJoints.size(), skeleton->getNumJoints());
  EXPECT_EQ(constJoints[1], skeleton->getJoint(1));

  const Skeleton* constSkeletonPtr = skeleton.get();
  DART_SUPPRESS_DEPRECATED_BEGIN
  const auto constJointsPtr = constSkeletonPtr->getJoints();
  DART_SUPPRESS_DEPRECATED_END
  EXPECT_EQ(constJointsPtr.size(), skeleton->getNumJoints());
  EXPECT_EQ(constSkeletonPtr->getJoint(0), skeleton->getJoint(0));

  const Skeleton& constRef = *skeleton;
  DART_SUPPRESS_DEPRECATED_BEGIN
  const auto constDofs = constRef.getDofs();
  DART_SUPPRESS_DEPRECATED_END
  EXPECT_EQ(constDofs.size(), skeleton->getNumDofs());

  EXPECT_EQ(skeleton->getBodyNode("missing_body"), nullptr);
  EXPECT_EQ(skeleton->getJoint("missing_joint"), nullptr);
}

//==============================================================================
TEST(SkeletonAccessors, ConstAccessorCoverage)
{
  auto skeleton = Skeleton::create("const_accessor");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  rootPair.first->setName("root_joint");
  rootPair.second->setName("root_body");

  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  childPair.first->setName("child_joint");
  childPair.second->setName("child_body");

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(0.2, 0.3, 0.4));
  rootPair.second->createShapeNodeWith<VisualAspect>(box);

  auto* marker = rootPair.second->createMarker(std::string("marker"));
  auto* endEffector = childPair.second->createEndEffector("ee");
  endEffector->setDefaultRelativeTransform(Eigen::Isometry3d::Identity());

  const Skeleton* constSkel = skeleton.get();

  EXPECT_EQ(constSkel->getRootBodyNode(0), rootPair.second);
  EXPECT_EQ(constSkel->getRootJoint(0), rootPair.first);

  EXPECT_EQ(constSkel->getBodyNode(0), rootPair.second);
  EXPECT_EQ(constSkel->getBodyNode("child_body"), childPair.second);
  EXPECT_EQ(constSkel->getBodyNode("missing_body"), nullptr);

  EXPECT_EQ(constSkel->getJoint(0), rootPair.first);
  EXPECT_EQ(constSkel->getJoint("child_joint"), childPair.first);
  EXPECT_EQ(constSkel->getJoint("missing_joint"), nullptr);

  EXPECT_EQ(constSkel->getNumSoftBodyNodes(), 0u);
  EXPECT_EQ(constSkel->getSoftBodyNode("missing_soft"), nullptr);

  DART_SUPPRESS_DEPRECATED_BEGIN
  const auto bodyNodes = constSkel->getBodyNodes();
  DART_SUPPRESS_DEPRECATED_END
  EXPECT_EQ(bodyNodes.size(), skeleton->getNumBodyNodes());

  const auto nodesByName = constSkel->getBodyNodes("root_body");
  ASSERT_EQ(nodesByName.size(), 1u);
  EXPECT_EQ(nodesByName[0], rootPair.second);

  const auto jointsByName = constSkel->getJoints("child_joint");
  ASSERT_EQ(jointsByName.size(), 1u);
  EXPECT_EQ(jointsByName[0], childPair.first);

  DART_SUPPRESS_DEPRECATED_BEGIN
  const auto constJoints = constSkel->getJoints();
  DART_SUPPRESS_DEPRECATED_END
  EXPECT_EQ(constJoints.size(), skeleton->getNumJoints());

  DART_SUPPRESS_DEPRECATED_BEGIN
  const auto constDofs = constSkel->getDofs();
  DART_SUPPRESS_DEPRECATED_END
  EXPECT_EQ(constDofs.size(), skeleton->getNumDofs());

  const auto* dof0 = constSkel->getDof(0);
  ASSERT_NE(dof0, nullptr);
  EXPECT_EQ(constSkel->getDof(dof0->getName()), dof0);

  const auto treeDofs = constSkel->getTreeDofs(0);
  EXPECT_EQ(treeDofs.size(), skeleton->getNumDofs());

  const auto treeBodyNodes = constSkel->getTreeBodyNodes(0);
  EXPECT_EQ(treeBodyNodes.size(), skeleton->getNumBodyNodes());

  EXPECT_EQ(constSkel->getMarker(0), marker);
  EXPECT_EQ(constSkel->getMarker("marker"), marker);
  EXPECT_EQ(constSkel->getEndEffector(0), endEffector);
  EXPECT_NE(constSkel->getShapeNode(0), nullptr);

  const auto config = constSkel->getConfiguration(
      Skeleton::CONFIG_POSITIONS | Skeleton::CONFIG_VELOCITIES);
  EXPECT_EQ(config.mPositions.size(), static_cast<int>(skeleton->getNumDofs()));
  EXPECT_EQ(
      config.mVelocities.size(), static_cast<int>(skeleton->getNumDofs()));

  const auto state = constSkel->getState();
  const auto properties = constSkel->getProperties();
  (void)state;
  (void)properties;
}

//==============================================================================
TEST(SkeletonAccessors, MetaSkeletonConstOverloads)
{
  auto skeleton = Skeleton::create("meta_const");

  RevoluteJoint::Properties rootJointProps;
  rootJointProps.mName = "joint0";
  rootJointProps.mAxis = Eigen::Vector3d::UnitZ();
  BodyNode::Properties rootBodyProps;
  rootBodyProps.mName = "body0";
  rootBodyProps.mInertia.setMass(1.0);
  auto rootPair = skeleton->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, rootJointProps, rootBodyProps);

  RevoluteJoint::Properties childJointProps;
  childJointProps.mName = "joint1";
  childJointProps.mAxis = Eigen::Vector3d::UnitY();
  BodyNode::Properties childBodyProps;
  childBodyProps.mName = "body1";
  childBodyProps.mInertia.setMass(1.0);
  auto childPair = skeleton->createJointAndBodyNodePair<RevoluteJoint>(
      rootPair.second, childJointProps, childBodyProps);

  const std::size_t dofs = skeleton->getNumDofs();
  Eigen::VectorXd positions(dofs);
  positions << 0.2, -0.4;
  Eigen::VectorXd velocities(dofs);
  velocities << 0.3, -0.1;
  Eigen::VectorXd accelerations(dofs);
  accelerations << -0.2, 0.6;
  Eigen::VectorXd forces(dofs);
  forces << 1.2, -0.8;
  Eigen::VectorXd commands(dofs);
  commands << 0.05, -0.1;

  skeleton->setPositions(positions);
  skeleton->setVelocities(velocities);
  skeleton->setAccelerations(accelerations);
  skeleton->setForces(forces);
  skeleton->setCommands(commands);

  Eigen::VectorXd lower = Eigen::VectorXd::Constant(dofs, -1.0);
  Eigen::VectorXd upper = Eigen::VectorXd::Constant(dofs, 1.0);
  skeleton->setPositionLowerLimits(lower);
  skeleton->setPositionUpperLimits(upper);
  skeleton->setVelocityLowerLimits(lower * 2.0);
  skeleton->setVelocityUpperLimits(upper * 2.0);
  skeleton->setAccelerationLowerLimits(lower * 3.0);
  skeleton->setAccelerationUpperLimits(upper * 3.0);
  skeleton->setForceLowerLimits(lower * 4.0);
  skeleton->setForceUpperLimits(upper * 4.0);

  const Skeleton* constSkel = skeleton.get();
  const MetaSkeleton* meta = static_cast<const MetaSkeleton*>(constSkel);

  EXPECT_EQ(meta->getNumBodyNodes(), 2u);
  EXPECT_EQ(meta->getBodyNode(0), rootPair.second);
  EXPECT_EQ(meta->getBodyNode("body1"), childPair.second);

  EXPECT_EQ(meta->getNumJoints(), 2u);
  EXPECT_EQ(meta->getJoint(0), rootPair.first);
  EXPECT_EQ(meta->getJoint("joint1"), childPair.first);

  EXPECT_EQ(meta->getNumDofs(), dofs);
  const auto* dof0 = meta->getDof(0);
  ASSERT_NE(dof0, nullptr);

  EXPECT_EQ(meta->getIndexOf(rootPair.second), 0u);
  EXPECT_EQ(meta->getIndexOf(childPair.first), 1u);
  EXPECT_EQ(meta->getIndexOf(dof0), 0u);

  EXPECT_TRUE(meta->getPositions().isApprox(positions));
  EXPECT_TRUE(meta->getVelocities().isApprox(velocities));
  EXPECT_TRUE(meta->getAccelerations().isApprox(accelerations));
  EXPECT_TRUE(meta->getForces().isApprox(forces));
  EXPECT_TRUE(meta->getCommands().isApprox(commands));

  EXPECT_TRUE(meta->getPositionLowerLimits().isApprox(lower));
  EXPECT_TRUE(meta->getPositionUpperLimits().isApprox(upper));
  EXPECT_TRUE(meta->getVelocityLowerLimits().isApprox(lower * 2.0));
  EXPECT_TRUE(meta->getVelocityUpperLimits().isApprox(upper * 2.0));
  EXPECT_TRUE(meta->getAccelerationLowerLimits().isApprox(lower * 3.0));
  EXPECT_TRUE(meta->getAccelerationUpperLimits().isApprox(upper * 3.0));
  EXPECT_TRUE(meta->getForceLowerLimits().isApprox(lower * 4.0));
  EXPECT_TRUE(meta->getForceUpperLimits().isApprox(upper * 4.0));
}

//==============================================================================
TEST(SkeletonAccessors, ZeroDofMassMatrixAndForceAccessors)
{
  auto skeleton = Skeleton::create("zero_dof_skeleton");
  auto pair = skeleton->createJointAndBodyNodePair<WeldJoint>();
  pair.second->setMass(1.0);

  const auto& treeMass = skeleton->getMassMatrix(0);
  EXPECT_EQ(treeMass.rows(), 0);
  EXPECT_EQ(treeMass.cols(), 0);

  const auto& skelMass = skeleton->getMassMatrix();
  EXPECT_EQ(skelMass.rows(), 0);
  EXPECT_EQ(skelMass.cols(), 0);

  const auto& treeAug = skeleton->getAugMassMatrix(0);
  EXPECT_EQ(treeAug.rows(), 0);
  EXPECT_EQ(treeAug.cols(), 0);
  const auto& skelAug = skeleton->getAugMassMatrix();
  EXPECT_EQ(skelAug.rows(), 0);
  EXPECT_EQ(skelAug.cols(), 0);

  const auto& treeInv = skeleton->getInvMassMatrix(0);
  EXPECT_EQ(treeInv.rows(), 0);
  EXPECT_EQ(treeInv.cols(), 0);
  const auto& skelInv = skeleton->getInvMassMatrix();
  EXPECT_EQ(skelInv.rows(), 0);
  EXPECT_EQ(skelInv.cols(), 0);

  const auto& treeInvAug = skeleton->getInvAugMassMatrix(0);
  EXPECT_EQ(treeInvAug.rows(), 0);
  EXPECT_EQ(treeInvAug.cols(), 0);
  const auto& skelInvAug = skeleton->getInvAugMassMatrix();
  EXPECT_EQ(skelInvAug.rows(), 0);
  EXPECT_EQ(skelInvAug.cols(), 0);

  const auto& treeC = skeleton->getCoriolisForces(0);
  EXPECT_EQ(treeC.size(), 0);
  const auto& skelC = skeleton->getCoriolisForces();
  EXPECT_EQ(skelC.size(), 0);

  const auto& treeG = skeleton->getGravityForces(0);
  EXPECT_EQ(treeG.size(), 0);
  const auto& skelG = skeleton->getGravityForces();
  EXPECT_EQ(skelG.size(), 0);

  const auto& treeCg = skeleton->getCoriolisAndGravityForces(0);
  EXPECT_EQ(treeCg.size(), 0);
  const auto& skelCg = skeleton->getCoriolisAndGravityForces();
  EXPECT_EQ(skelCg.size(), 0);

  const auto& treeFext = skeleton->getExternalForces(0);
  EXPECT_EQ(treeFext.size(), 0);
  const auto& skelFext = skeleton->getExternalForces();
  EXPECT_EQ(skelFext.size(), 0);
}

//==============================================================================
TEST(SkeletonAccessors, ArticulatedInertiaUpdates)
{
  auto skeleton = TestSkeleton::create("articulated_inertia");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  skeleton->dirtyArticulatedInertia(0);
  skeleton->updateArticulatedInertia();

  const auto& inertia = pair.second->getArticulatedInertia();
  (void)inertia;
}

//==============================================================================
TEST(BodyNodeOperations, MoveSplitChangeParentJoint)
{
  auto skeletonA = Skeleton::create("move_a");
  auto rootPairA = skeletonA->createJointAndBodyNodePair<FreeJoint>();
  auto childPairA
      = rootPairA.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  auto* child = childPairA.second;

  auto skeletonB = Skeleton::create("move_b");
  auto rootPairB = skeletonB->createJointAndBodyNodePair<FreeJoint>();

  EXPECT_TRUE(child->moveTo(skeletonB, rootPairB.second));
  EXPECT_EQ(child->getSkeleton(), skeletonB);

  EXPECT_TRUE(child->moveTo(nullptr));
  EXPECT_EQ(child->getSkeleton(), skeletonB);
  EXPECT_EQ(child->getParentBodyNode(), nullptr);

  auto splitSkeleton = child->split("split_skeleton");
  EXPECT_EQ(splitSkeleton->getName(), "split_skeleton");
  EXPECT_EQ(splitSkeleton->getNumBodyNodes(), 1u);

  auto changeSkeleton = Skeleton::create("change_joint");
  auto rootPairC = changeSkeleton->createJointAndBodyNodePair<FreeJoint>();
  auto childPairC
      = rootPairC.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  auto* changed = childPairC.second;

  auto* newJoint = changed->changeParentJointType<BallJoint>();
  ASSERT_NE(newJoint, nullptr);
  EXPECT_EQ(newJoint->getNumDofs(), 3u);
}

//==============================================================================
TEST(BodyNodeOperations, DependentDofsAndMomentum)
{
  auto skeleton = Skeleton::create("body_momentum");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto* joint = pair.first;
  auto* body = pair.second;

  EXPECT_EQ(body->getDependentDofs().size(), skeleton->getNumDofs());
  EXPECT_EQ(body->getDependentDof(0), joint->getDof(0));

  const BodyNode* constBody = body;
  EXPECT_NE(constBody->getDependentDof(0), nullptr);

  Eigen::Vector6d spatialVelocity = Eigen::Vector6d::Zero();
  spatialVelocity.tail<3>() = Eigen::Vector3d(0.2, -0.1, 0.3);
  joint->setSpatialVelocity(spatialVelocity, Frame::World(), Frame::World());

  const auto linearMomentum = body->getLinearMomentum();
  EXPECT_TRUE(linearMomentum.array().isFinite().all());

  const auto angularMomentum
      = body->getAngularMomentum(Eigen::Vector3d::Zero());
  EXPECT_TRUE(angularMomentum.array().isFinite().all());
}

//==============================================================================
TEST(ZeroDofJointAccessors, WeldJointJacobianAndSetters)
{
  auto skeleton = Skeleton::create("weld_joint");
  auto pair = skeleton->createJointAndBodyNodePair<WeldJoint>();
  auto* joint = static_cast<Joint*>(pair.first);

  const auto jac = joint->getRelativeJacobian();
  EXPECT_EQ(jac.rows(), 6);
  EXPECT_EQ(jac.cols(), 0);

  const auto jacDeriv = joint->getRelativeJacobianTimeDeriv();
  EXPECT_EQ(jacDeriv.rows(), 6);
  EXPECT_EQ(jacDeriv.cols(), 0);

  joint->setSpringStiffness(0, 10.0);
  joint->setRestPosition(0, 0.2);
  joint->setDampingCoefficient(0, 0.3);
  joint->setCoulombFriction(0, 0.1);

  EXPECT_DOUBLE_EQ(joint->getSpringStiffness(0), 0.0);
  EXPECT_DOUBLE_EQ(joint->getRestPosition(0), 0.0);
  EXPECT_DOUBLE_EQ(joint->getDampingCoefficient(0), 0.0);
  EXPECT_DOUBLE_EQ(joint->getCoulombFriction(0), 0.0);
}

//==============================================================================
TEST(JointAccessors, CopyPropertiesAndWrenchQueries)
{
  auto skeletonA = Skeleton::create("joint_copy_a");
  auto pairA = skeletonA->createJointAndBodyNodePair<RevoluteJoint>();
  auto* jointA = pairA.first;
  jointA->setName("joint_a");
  jointA->setLimitEnforcement(true);
  EXPECT_TRUE(jointA->areLimitsEnforced());

  auto skeletonB = Skeleton::create("joint_copy_b");
  auto pairB = skeletonB->createJointAndBodyNodePair<RevoluteJoint>();
  auto* jointB = pairB.first;

  jointB->copy(jointA);
  *jointB = *jointA;

  Joint::Properties jointProps;
  jointProps.mName = "extended_props_joint";
  Joint::ExtendedProperties extendedProps(
      jointProps, Joint::CompositeProperties());
  Joint::ExtendedProperties movedProps{
      Joint::Properties(jointProps), Joint::CompositeProperties()};
  (void)extendedProps;
  (void)movedProps;

  std::array<Joint::ActuatorType, 3> types
      = {Joint::FORCE, Joint::MIMIC, Joint::FORCE};
  auto skeletonC = Skeleton::create("joint_actuator_types");
  auto pairC = skeletonC->createJointAndBodyNodePair<BallJoint>();
  pairC.first->setActuatorTypes(types);
  EXPECT_EQ(pairC.first->getActuatorType(1), Joint::MIMIC);

  auto mimicSkeleton = Skeleton::create("joint_mimic_ops");
  auto refPair = mimicSkeleton->createJointAndBodyNodePair<RevoluteJoint>();
  auto mimicPair
      = refPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  mimicPair.first->setActuatorType(0, Joint::MIMIC);
  mimicPair.first->setMimicJoint(refPair.first, 1.0, 0.0);

  auto wrenchPair
      = pairA.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  auto* wrenchJoint = wrenchPair.first;
  auto* wrenchParentBody = pairA.second;
  auto* wrenchChildBody = wrenchPair.second;

  const auto wrenchDefault = wrenchJoint->getWrenchToChildBodyNode();
  const auto wrenchChild
      = wrenchJoint->getWrenchToChildBodyNode(wrenchChildBody);
  const auto wrenchParent
      = wrenchJoint->getWrenchToChildBodyNode(wrenchParentBody);
  const auto wrenchWorld
      = wrenchJoint->getWrenchToChildBodyNode(Frame::World());

  EXPECT_TRUE(wrenchDefault.array().isFinite().all());
  EXPECT_TRUE(wrenchChild.array().isFinite().all());
  EXPECT_TRUE(wrenchParent.array().isFinite().all());
  EXPECT_TRUE(wrenchWorld.array().isFinite().all());

  const auto wrenchToParent
      = wrenchJoint->getWrenchToParentBodyNode(Frame::World());
  EXPECT_TRUE(wrenchToParent.array().isFinite().all());

  wrenchJoint->notifyPositionUpdated();
  wrenchJoint->notifyVelocityUpdated();
  wrenchJoint->notifyAccelerationUpdated();
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

namespace {

SkeletonPtr createCoverageSkeleton()
{
  return createNLinkRobot(4, Eigen::Vector3d(0.2, 0.1, 0.3), DOF_YAW, false);
}

Eigen::VectorXd makeSequence(std::size_t size, double start, double step)
{
  Eigen::VectorXd values(static_cast<Eigen::Index>(size));
  for (std::size_t i = 0; i < size; ++i) {
    values[static_cast<Eigen::Index>(i)]
        = start + step * static_cast<double>(i);
  }
  return values;
}

} // namespace

//==============================================================================
TEST(SkeletonConfiguration, ConfigurationFlagsAndSegments)
{
  auto skeleton = createCoverageSkeleton();
  const std::size_t dofs = skeleton->getNumDofs();
  ASSERT_GE(dofs, 4u);

  const Eigen::VectorXd positions = makeSequence(dofs, 0.2, 0.1);
  const Eigen::VectorXd velocities = makeSequence(dofs, -0.3, 0.05);
  const Eigen::VectorXd accelerations = makeSequence(dofs, 0.4, 0.02);
  const Eigen::VectorXd forces = makeSequence(dofs, 1.0, -0.1);
  const Eigen::VectorXd commands = makeSequence(dofs, -0.5, 0.15);

  skeleton->setPositions(positions);
  skeleton->setVelocities(velocities);
  skeleton->setAccelerations(accelerations);
  skeleton->setForces(forces);
  skeleton->setCommands(commands);

  const std::vector<std::size_t> indices = {0u, 2u, 3u};
  const Eigen::VectorXd posSubset = makeSequence(indices.size(), 1.1, 0.2);
  const Eigen::VectorXd velSubset = makeSequence(indices.size(), -1.0, 0.3);
  const Eigen::VectorXd accSubset = makeSequence(indices.size(), 0.7, -0.05);
  const Eigen::VectorXd forceSubset = makeSequence(indices.size(), 2.0, -0.25);
  const Eigen::VectorXd cmdSubset = makeSequence(indices.size(), -1.5, 0.4);

  skeleton->setPositions(indices, posSubset);
  skeleton->setVelocities(indices, velSubset);
  skeleton->setAccelerations(indices, accSubset);
  skeleton->setForces(indices, forceSubset);
  skeleton->setCommands(indices, cmdSubset);

  EXPECT_TRUE(skeleton->getPositions(indices).isApprox(posSubset));
  EXPECT_TRUE(skeleton->getVelocities(indices).isApprox(velSubset));
  EXPECT_TRUE(skeleton->getAccelerations(indices).isApprox(accSubset));
  EXPECT_TRUE(skeleton->getForces(indices).isApprox(forceSubset));
  EXPECT_TRUE(skeleton->getCommands(indices).isApprox(cmdSubset));

  const auto config = skeleton->getConfiguration(
      indices, Skeleton::CONFIG_POSITIONS | Skeleton::CONFIG_FORCES);
  EXPECT_EQ(
      config.mPositions.size(), static_cast<Eigen::Index>(indices.size()));
  EXPECT_EQ(config.mVelocities.size(), 0);
  EXPECT_EQ(config.mAccelerations.size(), 0);
  EXPECT_EQ(config.mForces.size(), static_cast<Eigen::Index>(indices.size()));
  EXPECT_EQ(config.mCommands.size(), 0);
  EXPECT_TRUE(config.mPositions.isApprox(posSubset));
  EXPECT_TRUE(config.mForces.isApprox(forceSubset));

  Skeleton::Configuration updateConfig(
      indices, posSubset, velSubset, accSubset, forceSubset, cmdSubset);
  skeleton->setConfiguration(updateConfig);
  EXPECT_TRUE(skeleton->getCommands(indices).isApprox(cmdSubset));

  skeleton->resetPositions();
  skeleton->resetVelocities();
  skeleton->resetAccelerations();
  skeleton->resetCommands();
  skeleton->resetGeneralizedForces();

  EXPECT_TRUE(skeleton->getPositions().isZero());
  EXPECT_TRUE(skeleton->getVelocities().isZero());
  EXPECT_TRUE(skeleton->getAccelerations().isZero());
  EXPECT_TRUE(skeleton->getCommands().isZero());
  EXPECT_TRUE(skeleton->getForces().isZero());
}

//==============================================================================
TEST(SkeletonForces, LimitsAndForces)
{
  auto skeleton = createCoverageSkeleton();
  const std::size_t dofs = skeleton->getNumDofs();
  ASSERT_GE(dofs, 4u);

  const Eigen::VectorXd accLower = makeSequence(dofs, -2.0, -0.5);
  const Eigen::VectorXd accUpper = makeSequence(dofs, 2.0, 0.5);
  skeleton->setAccelerationLowerLimits(accLower);
  skeleton->setAccelerationUpperLimits(accUpper);
  EXPECT_TRUE(skeleton->getAccelerationLowerLimits().isApprox(accLower));
  EXPECT_TRUE(skeleton->getAccelerationUpperLimits().isApprox(accUpper));

  const std::vector<std::size_t> indices = {1u, 3u};
  Eigen::VectorXd forceLower(indices.size());
  forceLower << -4.0, -6.0;
  Eigen::VectorXd forceUpper(indices.size());
  forceUpper << 4.0, 6.0;
  skeleton->setForceLowerLimits(indices, forceLower);
  skeleton->setForceUpperLimits(indices, forceUpper);
  EXPECT_TRUE(skeleton->getForceLowerLimits(indices).isApprox(forceLower));
  EXPECT_TRUE(skeleton->getForceUpperLimits(indices).isApprox(forceUpper));

  skeleton->setForce(0, 3.25);
  EXPECT_NEAR(skeleton->getForce(0), 3.25, 1e-12);

  auto* body = skeleton->getBodyNode(0);
  ASSERT_NE(body, nullptr);
  body->addExtForce(
      Eigen::Vector3d(1.0, -0.5, 0.25), Eigen::Vector3d::Zero(), true, true);

  skeleton->setJointConstraintImpulses(
      Eigen::VectorXd::Ones(static_cast<Eigen::Index>(dofs)));
  skeleton->setForces(Eigen::VectorXd::Ones(static_cast<Eigen::Index>(dofs)));

  skeleton->clearExternalForces();
  skeleton->clearInternalForces();
  skeleton->clearConstraintImpulses();

  EXPECT_TRUE(skeleton->getExternalForces().isZero());
  EXPECT_TRUE(skeleton->getForces().isZero());
  EXPECT_TRUE(skeleton->getJointConstraintImpulses().isZero());
}

//==============================================================================
TEST(SkeletonDynamics, DynamicsAndJacobians)
{
  auto skeleton = createCoverageSkeleton();
  const std::size_t dofs = skeleton->getNumDofs();
  ASSERT_GE(dofs, 3u);

  skeleton->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  skeleton->setPositions(makeSequence(dofs, 0.1, 0.05));
  skeleton->setVelocities(makeSequence(dofs, 0.2, -0.04));
  skeleton->setAccelerations(makeSequence(dofs, -0.1, 0.03));
  skeleton->setForces(makeSequence(dofs, 1.0, 0.25));

  skeleton->computeForwardKinematics(true, true, true);
  skeleton->computeForwardDynamics();
  skeleton->computeInverseDynamics(true, true, true);

  const auto& M = skeleton->getMassMatrix();
  const auto& Mtree = skeleton->getMassMatrix(0);
  EXPECT_EQ(M.rows(), static_cast<Eigen::Index>(dofs));
  EXPECT_EQ(M.cols(), static_cast<Eigen::Index>(dofs));
  EXPECT_TRUE(M.isApprox(Mtree));

  const auto& Maug = skeleton->getAugMassMatrix();
  const auto& MaugTree = skeleton->getAugMassMatrix(0);
  EXPECT_TRUE(Maug.isApprox(MaugTree));

  const auto& Minv = skeleton->getInvMassMatrix();
  const auto& MinvTree = skeleton->getInvMassMatrix(0);
  EXPECT_TRUE(Minv.isApprox(MinvTree));

  const auto& MinvAug = skeleton->getInvAugMassMatrix();
  const auto& MinvAugTree = skeleton->getInvAugMassMatrix(0);
  EXPECT_TRUE(MinvAug.isApprox(MinvAugTree));

  const auto& coriolis = skeleton->getCoriolisForces();
  const auto& coriolisTree = skeleton->getCoriolisForces(0);
  EXPECT_EQ(coriolis.size(), static_cast<Eigen::Index>(dofs));
  EXPECT_TRUE(coriolis.isApprox(coriolisTree));

  const auto& gravity = skeleton->getGravityForces();
  const auto& gravityTree = skeleton->getGravityForces(0);
  EXPECT_EQ(gravity.size(), static_cast<Eigen::Index>(dofs));
  EXPECT_TRUE(gravity.isApprox(gravityTree));

  const auto& combined = skeleton->getCoriolisAndGravityForces();
  const auto& combinedTree = skeleton->getCoriolisAndGravityForces(0);
  EXPECT_EQ(combined.size(), static_cast<Eigen::Index>(dofs));
  EXPECT_TRUE(combined.isApprox(combinedTree));

  const auto& external = skeleton->getExternalForces();
  const auto& externalTree = skeleton->getExternalForces(0);
  EXPECT_EQ(external.size(), static_cast<Eigen::Index>(dofs));
  EXPECT_TRUE(external.isApprox(externalTree));

  const auto& constraints = skeleton->getConstraintForces();
  const auto& constraintsTree = skeleton->getConstraintForces(0);
  EXPECT_EQ(constraints.size(), static_cast<Eigen::Index>(dofs));
  EXPECT_TRUE(constraints.isApprox(constraintsTree));

  const double kinetic = skeleton->computeKineticEnergy();
  const double potential = skeleton->computePotentialEnergy();
  const double lagrangian = skeleton->computeLagrangian();
  EXPECT_TRUE(std::isfinite(kinetic));
  EXPECT_TRUE(std::isfinite(potential));
  EXPECT_NEAR(lagrangian, kinetic - potential, 1e-10);

  const auto Jcom = skeleton->getCOMJacobian();
  const auto JcomLinear = skeleton->getCOMLinearJacobian();
  const auto JcomDot = skeleton->getCOMJacobianSpatialDeriv();
  const auto JcomLinearDot = skeleton->getCOMLinearJacobianDeriv();
  EXPECT_EQ(Jcom.rows(), 6);
  EXPECT_EQ(Jcom.cols(), static_cast<Eigen::Index>(dofs));
  EXPECT_EQ(JcomLinear.rows(), 3);
  EXPECT_EQ(JcomLinear.cols(), static_cast<Eigen::Index>(dofs));
  EXPECT_EQ(JcomDot.rows(), 6);
  EXPECT_EQ(JcomDot.cols(), static_cast<Eigen::Index>(dofs));
  EXPECT_EQ(JcomLinearDot.rows(), 3);
  EXPECT_EQ(JcomLinearDot.cols(), static_cast<Eigen::Index>(dofs));
  EXPECT_TRUE(Jcom.array().isFinite().all());
  EXPECT_TRUE(JcomLinear.array().isFinite().all());
  EXPECT_TRUE(JcomDot.array().isFinite().all());
  EXPECT_TRUE(JcomLinearDot.array().isFinite().all());

  const auto* bodyA = skeleton->getBodyNode(0);
  const auto* bodyB = skeleton->getBodyNode(1);
  ASSERT_NE(bodyA, nullptr);
  ASSERT_NE(bodyB, nullptr);

  const Eigen::Vector3d offset(0.05, -0.02, 0.08);
  const auto Jrel = skeleton->getJacobian(bodyA, bodyB, Frame::World());
  const auto JrelOffset
      = skeleton->getJacobian(bodyA, offset, bodyB, Frame::World());
  const auto JlinRel
      = skeleton->getLinearJacobian(bodyA, bodyB, Frame::World());
  const auto JlinRelOffset
      = skeleton->getLinearJacobian(bodyA, offset, bodyB, Frame::World());
  const auto JangRel
      = skeleton->getAngularJacobian(bodyA, bodyB, Frame::World());
  const auto JspatialRel
      = skeleton->getJacobianSpatialDeriv(bodyA, bodyB, Frame::World());
  const auto JspatialRelOffset
      = skeleton->getJacobianSpatialDeriv(bodyA, offset, bodyB, Frame::World());
  const auto JclassicRel
      = skeleton->getJacobianClassicDeriv(bodyA, Frame::World());
  const auto JlinRelDot
      = skeleton->getLinearJacobianDeriv(bodyA, offset, Frame::World());

  EXPECT_EQ(Jrel.rows(), 6);
  EXPECT_EQ(Jrel.cols(), static_cast<Eigen::Index>(dofs));
  EXPECT_EQ(JrelOffset.rows(), 6);
  EXPECT_EQ(JlinRel.rows(), 3);
  EXPECT_EQ(JlinRelOffset.rows(), 3);
  EXPECT_EQ(JangRel.rows(), 3);
  EXPECT_EQ(JspatialRel.rows(), 6);
  EXPECT_EQ(JspatialRelOffset.rows(), 6);
  EXPECT_EQ(JclassicRel.rows(), 6);
  EXPECT_EQ(JlinRelDot.rows(), 3);
}

//==============================================================================
TEST(SkeletonAspectHelpers, BodyNodeStateVectorMismatch)
{
  auto skeleton = Skeleton::create("state_vector_mismatch");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();

  auto states = dart::dynamics::detail::getAllBodyNodeStates(skeleton.get());
  ASSERT_EQ(states.size(), 2u);

  states.pop_back();
  dart::dynamics::detail::setAllBodyNodeStates(skeleton.get(), states);

  const auto roundTrip
      = dart::dynamics::detail::getAllBodyNodeStates(skeleton.get());
  EXPECT_EQ(roundTrip.size(), 2u);
}

//==============================================================================
TEST(SkeletonRootBodyNode, ThrowsOnInvalidTreeIndex)
{
  auto empty = Skeleton::create("root_empty");
  EXPECT_THROW(empty->getRootBodyNode(0), dart::common::OutOfRangeException);

  auto skeleton = Skeleton::create("root_invalid");
  skeleton->createJointAndBodyNodePair<FreeJoint>();

  EXPECT_THROW(skeleton->getRootBodyNode(1), dart::common::OutOfRangeException);

  const Skeleton* constSkel = skeleton.get();
  EXPECT_THROW(
      constSkel->getRootBodyNode(1), dart::common::OutOfRangeException);
}

TEST(SkeletonClone, CloneMetaSkeleton)
{
  auto skeleton = Skeleton::create("meta_clone");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());

  Eigen::VectorXd positions = Eigen::VectorXd::Constant(
      static_cast<Eigen::Index>(skeleton->getNumDofs()), 0.4);
  skeleton->setPositions(positions);

  MetaSkeletonPtr clone = skeleton->cloneMetaSkeleton("meta_clone_copy");
  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->getName(), "meta_clone_copy");
  EXPECT_EQ(clone->getNumDofs(), skeleton->getNumDofs());
  EXPECT_TRUE(clone->getPositions().isApprox(positions));
}

TEST(SkeletonAccessors, BodyNodePropertiesRoundTrip)
{
  auto skeleton = Skeleton::create("bn_props_roundtrip");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();

  auto props = dart::dynamics::detail::getAllBodyNodeProperties(skeleton.get());
  ASSERT_EQ(props.size(), 2u);

  dart::dynamics::detail::setAllBodyNodeProperties(skeleton.get(), props);

  auto roundTrip
      = dart::dynamics::detail::getAllBodyNodeProperties(skeleton.get());
  EXPECT_EQ(roundTrip.size(), 2u);
}

TEST(SkeletonAccessors, ConstBodyNodeAndJointAccessors)
{
  auto skeleton = Skeleton::create("const_vectors");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();

  EXPECT_EQ(skeleton->getNumBodyNodes(), 2u);
  EXPECT_EQ(skeleton->getNumJoints(), 2u);

  const Skeleton* constSkel = skeleton.get();
  EXPECT_NE(constSkel->getBodyNode(0), nullptr);
  EXPECT_NE(constSkel->getBodyNode(1), nullptr);
  EXPECT_NE(constSkel->getJoint(0), nullptr);
  EXPECT_NE(constSkel->getJoint(1), nullptr);
}

//==============================================================================
TEST(FreeJointAccessors, CoordinateChartPreservesTransform)
{
  auto skeleton = Skeleton::create("free_chart");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto* joint = pair.first;

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear() = (Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitX())
                 * Eigen::AngleAxisd(-0.15, Eigen::Vector3d::UnitY())
                 * Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitZ()))
                    .toRotationMatrix();
  tf.translation() = Eigen::Vector3d(0.3, -0.2, 0.4);

  joint->setPositions(FreeJoint::convertToPositions(tf));
  const Eigen::Isometry3d before = joint->getRelativeTransform();

  joint->setCoordinateChart(FreeJoint::CoordinateChart::EULER_ZYX);
  const Eigen::Isometry3d after = joint->getRelativeTransform();

  EXPECT_TRUE(before.translation().isApprox(after.translation(), 1e-10));
  EXPECT_TRUE(before.linear().isApprox(after.linear(), 1e-10));
}

//==============================================================================
TEST(FreeJointAccessors, RelativeSpatialVelocityInWorldCoordinates)
{
  auto skeleton = Skeleton::create("free_rel_vel");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto* joint = pair.first;

  Eigen::Vector6d velocity;
  velocity << 0.2, -0.1, 0.15, 0.05, -0.3, 0.4;
  joint->setRelativeSpatialVelocity(velocity, Frame::World());

  EXPECT_TRUE(joint->getVelocities().allFinite());
  EXPECT_TRUE(joint->getRelativeSpatialVelocity().allFinite());
}

//==============================================================================
TEST(FreeJointAccessors, RelativeSpatialAccelerationInWorldCoordinates)
{
  auto skeleton = Skeleton::create("free_rel_acc");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto* joint = pair.first;

  Eigen::Vector6d acceleration;
  acceleration << -0.15, 0.25, -0.05, 0.4, -0.2, 0.1;
  joint->setRelativeSpatialAcceleration(acceleration, Frame::World());

  EXPECT_TRUE(joint->getAccelerations().allFinite());
  EXPECT_TRUE(joint->getRelativeSpatialAcceleration().allFinite());
}

//==============================================================================
TEST(FreeJointAccessors, LinearAndAngularVelocitySetters)
{
  auto skeleton = Skeleton::create("free_lin_ang_vel");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto* joint = pair.first;

  joint->setLinearVelocity(
      Eigen::Vector3d(0.1, -0.2, 0.3), Frame::World(), Frame::World());
  joint->setAngularVelocity(
      Eigen::Vector3d(-0.4, 0.2, 0.1), Frame::World(), Frame::World());

  EXPECT_TRUE(joint->getVelocities().allFinite());
}

//==============================================================================
TEST(FreeJointAccessors, LinearAndAngularAccelerationSetters)
{
  auto skeleton = Skeleton::create("free_lin_ang_acc");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto* joint = pair.first;

  joint->setLinearAcceleration(
      Eigen::Vector3d(0.2, -0.1, 0.05), Frame::World(), Frame::World());
  joint->setAngularAcceleration(
      Eigen::Vector3d(-0.05, 0.15, 0.25), Frame::World(), Frame::World());

  EXPECT_TRUE(joint->getAccelerations().allFinite());
}

//=============================================================================
TEST(SkeletonConfiguration, SetConfigurationWithAllVectors)
{
  auto skeleton = Skeleton::create("config_all_vectors");
  auto root = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  root.first->setAxis(Eigen::Vector3d::UnitZ());
  auto child = root.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child.first->setAxis(Eigen::Vector3d::UnitY());

  const auto dofs = skeleton->getNumDofs();
  Eigen::VectorXd positions = Eigen::VectorXd::LinSpaced(
      static_cast<int>(dofs), 0.1, 0.1 + static_cast<double>(dofs - 1));
  Eigen::VectorXd velocities = Eigen::VectorXd::Constant(dofs, -0.2);
  Eigen::VectorXd accelerations = Eigen::VectorXd::Constant(dofs, 0.3);
  Eigen::VectorXd forces = Eigen::VectorXd::Constant(dofs, -0.4);
  Eigen::VectorXd commands = Eigen::VectorXd::Constant(dofs, 0.5);

  Skeleton::Configuration config(
      positions, velocities, accelerations, forces, commands);
  skeleton->setConfiguration(config);

  EXPECT_TRUE(skeleton->getPositions().isApprox(positions));
  EXPECT_TRUE(skeleton->getVelocities().isApprox(velocities));
  EXPECT_TRUE(skeleton->getAccelerations().isApprox(accelerations));
  EXPECT_TRUE(skeleton->getForces().isApprox(forces));
  EXPECT_TRUE(skeleton->getCommands().isApprox(commands));
}

//=============================================================================
TEST(SkeletonState, RoundTripStateWithDerivatives)
{
  auto skeleton = Skeleton::create("state_round_trip");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setMass(1.0);

  Eigen::VectorXd positions = Eigen::VectorXd::Zero(skeleton->getNumDofs());
  positions[3] = 0.4;
  positions[4] = -0.2;
  Eigen::VectorXd velocities
      = Eigen::VectorXd::Constant(skeleton->getNumDofs(), 0.1);
  Eigen::VectorXd accelerations
      = Eigen::VectorXd::Constant(skeleton->getNumDofs(), -0.05);

  skeleton->setPositions(positions);
  skeleton->setVelocities(velocities);
  skeleton->setAccelerations(accelerations);

  const auto state = skeleton->getState();

  auto clone = skeleton->cloneSkeleton("state_round_trip_clone");
  clone->setState(state);

  EXPECT_TRUE(clone->getPositions().isApprox(positions));
  EXPECT_TRUE(clone->getVelocities().isApprox(velocities));
  EXPECT_TRUE(clone->getAccelerations().isApprox(accelerations));
}

//=============================================================================
TEST(SkeletonVersion, IncrementVersionCounter)
{
  auto skeleton = Skeleton::create("version_counter");
  const auto version = skeleton->getVersion();
  skeleton->incrementVersion();
  EXPECT_EQ(skeleton->getVersion(), version + 1u);
}

//=============================================================================
TEST(BodyNodeTreeOperations, MoveToMismatchedSkeleton)
{
  auto skeletonA = Skeleton::create("move_source");
  auto rootA = skeletonA->createJointAndBodyNodePair<FreeJoint>();
  auto childA = rootA.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  childA.first->setAxis(Eigen::Vector3d::UnitZ());

  auto skeletonB = Skeleton::create("move_target_parent");
  auto rootB = skeletonB->createJointAndBodyNodePair<FreeJoint>();

  EXPECT_FALSE(childA.second->moveTo(skeletonA, rootB.second));
  EXPECT_EQ(childA.second->getSkeleton(), skeletonA);
}

//=============================================================================
TEST(BodyNodeTreeOperations, MoveToNewSkeleton)
{
  auto skeleton = Skeleton::create("move_root");
  auto root = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto child = root.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child.first->setAxis(Eigen::Vector3d::UnitZ());
  child.second->setName("moved_body");

  auto newSkeleton = Skeleton::create("move_destination");
  EXPECT_TRUE(child.second->moveTo(newSkeleton, nullptr));

  EXPECT_EQ(newSkeleton->getNumBodyNodes(), 1u);
  EXPECT_EQ(newSkeleton->getBodyNode("moved_body"), child.second);
  EXPECT_EQ(skeleton->getNumBodyNodes(), 1u);
  EXPECT_EQ(skeleton->getBodyNode("moved_body"), nullptr);
}

//=============================================================================
TEST(BodyNodeTreeOperations, SplitAndRemove)
{
  auto skeleton = Skeleton::create("split_source");
  auto root = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto child = root.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child.first->setAxis(Eigen::Vector3d::UnitY());
  child.second->setName("split_body");

  auto splitSkeleton = child.second->split("split_target");
  EXPECT_EQ(splitSkeleton->getNumBodyNodes(), 1u);
  EXPECT_EQ(splitSkeleton->getBodyNode("split_body"), child.second);
  EXPECT_EQ(skeleton->getNumBodyNodes(), 1u);

  auto removedSkeleton = child.second->remove("removed_target");
  EXPECT_EQ(removedSkeleton->getName(), "removed_target");
  EXPECT_EQ(removedSkeleton->getNumBodyNodes(), 1u);
}

//=============================================================================
TEST(BodyNodeKinematics, COMLinearVelocityAndAcceleration)
{
  auto skeleton = Skeleton::create("body_com_kinematics");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto* body = pair.second;
  body->setMass(2.0);

  Eigen::VectorXd velocities = Eigen::VectorXd::Zero(skeleton->getNumDofs());
  velocities[3] = 1.0;
  velocities[4] = -0.5;
  velocities[5] = 0.2;
  skeleton->setVelocities(velocities);

  Eigen::VectorXd accelerations = Eigen::VectorXd::Zero(skeleton->getNumDofs());
  accelerations[3] = -0.1;
  accelerations[4] = 0.3;
  accelerations[5] = -0.2;
  skeleton->setAccelerations(accelerations);

  const auto comVel = body->getCOMLinearVelocity();
  const auto comAcc = body->getCOMLinearAcceleration();
  EXPECT_TRUE(comVel.array().isFinite().all());
  EXPECT_TRUE(comAcc.array().isFinite().all());
}

//=============================================================================
TEST(BodyNodeDynamics, MomentumAccessors)
{
  auto skeleton = Skeleton::create("body_momentum");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto* body = pair.second;
  body->setMass(3.0);

  Eigen::VectorXd velocities
      = Eigen::VectorXd::Constant(skeleton->getNumDofs(), 0.25);
  skeleton->setVelocities(velocities);

  const auto linear = body->getLinearMomentum();
  const auto angular = body->getAngularMomentum(Eigen::Vector3d::Zero());

  EXPECT_TRUE(linear.array().isFinite().all());
  EXPECT_TRUE(angular.array().isFinite().all());
  EXPECT_GT(linear.norm(), 0.0);
}

//=============================================================================
TEST(BodyNodeKinematics, COMSpatialVelocityAndAcceleration)
{
  auto skeleton = Skeleton::create("body_com_spatial");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto* body = pair.second;

  Eigen::VectorXd velocities = Eigen::VectorXd::Zero(skeleton->getNumDofs());
  velocities[0] = 0.2;
  velocities[1] = -0.1;
  velocities[2] = 0.3;
  skeleton->setVelocities(velocities);

  Eigen::VectorXd accelerations
      = Eigen::VectorXd::Constant(skeleton->getNumDofs(), 0.05);
  skeleton->setAccelerations(accelerations);

  const auto spatialVel = body->getCOMSpatialVelocity();
  const auto spatialAcc = body->getCOMSpatialAcceleration();

  EXPECT_TRUE(spatialVel.array().isFinite().all());
  EXPECT_TRUE(spatialAcc.array().isFinite().all());
}

//=============================================================================
TEST(BodyNodeTreeOps, MoveToNullParentCreatesNewTree)
{
  auto skeleton = Skeleton::create("move_null_parent");
  auto root = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto child = root.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child.first->setAxis(Eigen::Vector3d::UnitZ());

  ASSERT_EQ(skeleton->getNumTrees(), 1u);
  EXPECT_TRUE(child.second->moveTo(nullptr));
  EXPECT_EQ(child.second->getParentBodyNode(), nullptr);
  EXPECT_EQ(skeleton->getNumTrees(), 2u);
}

//=============================================================================
TEST(BodyNodeTreeOps, MoveToOtherSkeletonParent)
{
  auto skeletonA = Skeleton::create("move_src");
  auto rootA = skeletonA->createJointAndBodyNodePair<FreeJoint>();
  auto childA = rootA.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  childA.first->setAxis(Eigen::Vector3d::UnitY());

  auto skeletonB = Skeleton::create("move_dest");
  auto rootB = skeletonB->createJointAndBodyNodePair<FreeJoint>();

  EXPECT_TRUE(childA.second->moveTo(skeletonB, rootB.second));
  EXPECT_EQ(childA.second->getSkeleton(), skeletonB);
  EXPECT_EQ(childA.second->getParentBodyNode(), rootB.second);
  EXPECT_EQ(skeletonA->getNumBodyNodes(), 1u);
}

//=============================================================================
TEST(BodyNodeTreeOps, SplitCreatesNewSkeleton)
{
  auto skeleton = Skeleton::create("split_source");
  auto root = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto child = root.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child.first->setAxis(Eigen::Vector3d::UnitX());

  auto split = child.second->split("split_dest");
  ASSERT_NE(split, nullptr);
  EXPECT_EQ(split->getName(), "split_dest");
  EXPECT_EQ(split->getNumBodyNodes(), 1u);
  EXPECT_EQ(child.second->getSkeleton(), split);
  EXPECT_EQ(skeleton->getNumBodyNodes(), 1u);
}

//=============================================================================
TEST(BodyNodeKinematics, COMWorldFrameAccessors)
{
  auto skeleton = Skeleton::create("com_world");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto* joint = pair.first;
  auto* body = pair.second;

  body->setLocalCOM(Eigen::Vector3d(0.1, 0.2, 0.3));

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(1.0, -0.5, 2.0);
  joint->setPositions(FreeJoint::convertToPositions(tf));

  const Eigen::Vector3d linearVel(0.4, -0.2, 0.1);
  const Eigen::Vector3d linearAcc(0.05, 0.02, -0.03);
  joint->setAngularVelocity(
      Eigen::Vector3d::Zero(), Frame::World(), Frame::World());
  joint->setAngularAcceleration(
      Eigen::Vector3d::Zero(), Frame::World(), Frame::World());
  joint->setLinearVelocity(linearVel, Frame::World(), Frame::World());
  joint->setLinearAcceleration(linearAcc, Frame::World(), Frame::World());

  const Eigen::Vector3d expectedCom = tf.translation() + body->getLocalCOM();
  EXPECT_TRUE(body->getCOM(Frame::World()).isApprox(expectedCom, 1e-10));
  EXPECT_TRUE(body->getCOMLinearVelocity(Frame::World(), Frame::World())
                  .isApprox(linearVel, 1e-10));
  EXPECT_TRUE(body->getCOMLinearAcceleration(Frame::World(), Frame::World())
                  .isApprox(linearAcc, 1e-10));
}

//=============================================================================
TEST(BodyNodeDynamics, MomentumMatchesVelocity)
{
  auto skeleton = Skeleton::create("momentum_expected");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto* joint = pair.first;
  auto* body = pair.second;

  dart::dynamics::Inertia inertia;
  inertia.setMass(2.0);
  inertia.setMoment(1.0, 1.0, 1.0, 0.0, 0.0, 0.0);
  inertia.setLocalCOM(Eigen::Vector3d::Zero());
  body->setInertia(inertia);

  const Eigen::Vector3d linearVel(1.0, -2.0, 0.5);
  joint->setAngularVelocity(
      Eigen::Vector3d::Zero(), Frame::World(), Frame::World());
  joint->setLinearVelocity(linearVel, Frame::World(), Frame::World());

  EXPECT_TRUE(body->getLinearMomentum().isApprox(2.0 * linearVel, 1e-10));

  const Eigen::Vector3d angularVel(0.0, 0.0, 2.0);
  joint->setLinearVelocity(
      Eigen::Vector3d::Zero(), Frame::World(), Frame::World());
  joint->setAngularVelocity(angularVel, Frame::World(), Frame::World());

  const Eigen::Vector3d expectedAngular(0.0, 0.0, 2.0);
  EXPECT_TRUE(body->getAngularMomentum(body->getCOM())
                  .isApprox(expectedAngular, 1e-10));
}

namespace {

SkeletonPtr createComplexCloneSkeleton()
{
  auto skeleton = Skeleton::create("complex_clone_source");

  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  rootPair.first->setName("root_joint");
  rootPair.second->setName("root_body");

  auto revolutePair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  revolutePair.first->setName("rev_joint");
  revolutePair.first->setAxis(Eigen::Vector3d::UnitZ());
  revolutePair.second->setName("rev_body");

  auto prismaticPair
      = revolutePair.second->createChildJointAndBodyNodePair<PrismaticJoint>();
  prismaticPair.first->setName("pris_joint");
  prismaticPair.first->setAxis(Eigen::Vector3d::UnitX());
  prismaticPair.second->setName("pris_body");

  auto freePair
      = prismaticPair.second->createChildJointAndBodyNodePair<FreeJoint>();
  freePair.first->setName("free_joint");
  freePair.second->setName("free_body");

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(0.2, 0.1, 0.3));
  rootPair.second->createShapeNodeWith<VisualAspect, CollisionAspect>(box);
  revolutePair.second->createShapeNodeWith<VisualAspect, CollisionAspect>(box);
  prismaticPair.second->createShapeNodeWith<VisualAspect, CollisionAspect>(box);
  freePair.second->createShapeNodeWith<VisualAspect, CollisionAspect>(box);

  auto* endEffector = freePair.second->createEndEffector("tip");
  endEffector->setDefaultRelativeTransform(Eigen::Isometry3d::Identity());

  (void)freePair.second->createIK();

  prismaticPair.first->setMimicJoint(revolutePair.first, 1.25, -0.2);

  return skeleton;
}

} // namespace

//==============================================================================
TEST(SkeletonConfiguration, GetConfigurationAllFlags)
{
  auto skeleton = Skeleton::create("config_all_flags");
  auto root = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  root.first->setAxis(Eigen::Vector3d::UnitZ());
  auto child = root.second->createChildJointAndBodyNodePair<PrismaticJoint>();
  child.first->setAxis(Eigen::Vector3d::UnitX());

  const std::size_t dofs = skeleton->getNumDofs();
  const Eigen::VectorXd positions = makeSequence(dofs, 0.1, 0.05);
  const Eigen::VectorXd velocities = makeSequence(dofs, -0.2, 0.03);
  const Eigen::VectorXd accelerations = makeSequence(dofs, 0.4, -0.02);
  const Eigen::VectorXd forces = makeSequence(dofs, 1.5, 0.1);
  const Eigen::VectorXd commands = makeSequence(dofs, -0.6, 0.07);

  skeleton->setPositions(positions);
  skeleton->setVelocities(velocities);
  skeleton->setAccelerations(accelerations);
  skeleton->setForces(forces);
  skeleton->setCommands(commands);

  const auto config = skeleton->getConfiguration(Skeleton::CONFIG_ALL);
  EXPECT_TRUE(config.mPositions.isApprox(positions));
  EXPECT_TRUE(config.mVelocities.isApprox(velocities));
  EXPECT_TRUE(config.mAccelerations.isApprox(accelerations));
  EXPECT_TRUE(config.mForces.isApprox(forces));
  EXPECT_TRUE(config.mCommands.isApprox(commands));
}

//==============================================================================
TEST(SkeletonConfiguration, SetConfigurationWithIndicesPartial)
{
  auto skeleton = Skeleton::create("config_partial_indices");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  childPair.first->setAxis(Eigen::Vector3d::UnitY());

  const std::size_t dofs = skeleton->getNumDofs();
  ASSERT_GE(dofs, 2u);

  const std::vector<std::size_t> indices = {0u, dofs - 1u};
  Eigen::VectorXd positions(2);
  positions << 0.25, -0.5;
  Eigen::VectorXd commands(2);
  commands << 1.1, -1.2;

  Skeleton::Configuration config(
      indices,
      positions,
      Eigen::VectorXd(),
      Eigen::VectorXd(),
      Eigen::VectorXd(),
      commands);
  skeleton->setConfiguration(config);

  EXPECT_TRUE(skeleton->getPositions(indices).isApprox(positions));
  EXPECT_TRUE(skeleton->getCommands(indices).isApprox(commands));
}

//==============================================================================
TEST(SkeletonAccessors, ConstOverloads)
{
  auto skeleton = Skeleton::create("const_overloads");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  rootPair.first->setName("root_joint");
  rootPair.second->setName("root_body");

  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  childPair.first->setName("child_joint");
  childPair.second->setName("child_body");

  const Skeleton& constSkel = *skeleton;

  EXPECT_EQ(constSkel.getNumTrees(), 1u);
  EXPECT_EQ(constSkel.getRootBodyNode(0), rootPair.second);
  EXPECT_EQ(constSkel.getRootJoint(0), rootPair.first);

  EXPECT_EQ(constSkel.getBodyNode(0), rootPair.second);
  EXPECT_EQ(constSkel.getBodyNode("child_body"), childPair.second);
  EXPECT_EQ(constSkel.getBodyNodes("child_body").size(), 1u);
  EXPECT_TRUE(constSkel.hasBodyNode(childPair.second));

  EXPECT_EQ(constSkel.getJoint(0), rootPair.first);
  EXPECT_EQ(constSkel.getJoint("child_joint"), childPair.first);
  EXPECT_EQ(constSkel.getJoints("child_joint").size(), 1u);
  EXPECT_TRUE(constSkel.hasJoint(childPair.first));

  const auto* dof0 = constSkel.getDof(0);
  ASSERT_NE(dof0, nullptr);
  EXPECT_EQ(constSkel.getDof(dof0->getName()), dof0);
  DART_SUPPRESS_DEPRECATED_BEGIN
  EXPECT_EQ(constSkel.getDofs().size(), constSkel.getNumDofs());
  DART_SUPPRESS_DEPRECATED_END
  EXPECT_EQ(constSkel.getTreeDofs(0).size(), constSkel.getNumDofs());
  EXPECT_EQ(constSkel.getTreeBodyNodes(0).size(), constSkel.getNumBodyNodes());

  EXPECT_EQ(constSkel.getIndexOf(rootPair.second), 0u);
  EXPECT_EQ(constSkel.getIndexOf(rootPair.first), 0u);
  EXPECT_EQ(constSkel.getIndexOf(dof0), 0u);

  EXPECT_TRUE(constSkel.checkIndexingConsistency());

  EXPECT_GE(constSkel.getMass(), 0.0);
}

//==============================================================================
TEST(SkeletonAccessors, ConstShapeNodeAccessors)
{
  auto skeleton = Skeleton::create("const_shape_nodes");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setName("shape_body");

  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d(0.2, 0.1, 0.3));
  auto* shapeNode = pair.second->createShapeNodeWith<VisualAspect>(shape);
  shapeNode->setName("shape_node");

  const Skeleton& constSkel = *skeleton;

  EXPECT_EQ(constSkel.getNumShapeNodes(), 1u);
  EXPECT_EQ(constSkel.getShapeNode(0), shapeNode);
  EXPECT_EQ(constSkel.getShapeNode("shape_node"), shapeNode);
}

//==============================================================================
TEST(SkeletonAccessors, ConstMarkerAccessors)
{
  auto skeleton = Skeleton::create("const_markers");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setName("marker_body");

  auto* marker = pair.second->createMarker(
      "marker_node", Eigen::Vector3d::Zero(), Eigen::Vector4d::Ones());
  ASSERT_NE(marker, nullptr);

  const Skeleton& constSkel = *skeleton;
  EXPECT_EQ(constSkel.getNumMarkers(), 1u);
  EXPECT_EQ(constSkel.getMarker(0), marker);
  EXPECT_EQ(constSkel.getMarker("marker_node"), marker);
}

//==============================================================================
TEST(SkeletonAccessors, ConstEndEffectorAccessors)
{
  auto skeleton = Skeleton::create("const_end_effectors");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  pair.second->setName("ee_body");

  auto* endEffector = pair.second->createEndEffector("ee_node");
  ASSERT_NE(endEffector, nullptr);

  const Skeleton& constSkel = *skeleton;
  EXPECT_EQ(constSkel.getNumEndEffectors(), 1u);
  EXPECT_EQ(constSkel.getEndEffector(0), endEffector);
  EXPECT_EQ(constSkel.getEndEffector("ee_node"), endEffector);
}

//==============================================================================
TEST(SkeletonAccessors, SoftBodyConstOverloads)
{
  auto skeleton = Skeleton::create("soft_body_const");

  SoftBodyNode::UniqueProperties uniqueProps;
  uniqueProps.addPointMass(PointMass::Properties(Eigen::Vector3d::Zero(), 0.1));
  uniqueProps.addPointMass(
      PointMass::Properties(Eigen::Vector3d(0.1, 0.0, 0.0), 0.1));
  uniqueProps.connectPointMasses(0, 1);

  BodyNode::Properties bodyProps;
  bodyProps.mName = "soft_body";
  bodyProps.mInertia.setMass(0.2);
  SoftBodyNode::Properties props(bodyProps, uniqueProps);

  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>(
      nullptr, FreeJoint::Properties(), props);
  auto* softBody = pair.second;
  ASSERT_NE(softBody, nullptr);
  EXPECT_EQ(softBody->getNumPointMasses(), 2u);

  const Skeleton& constSkel = *skeleton;
  EXPECT_EQ(constSkel.getNumSoftBodyNodes(), 1u);
  EXPECT_EQ(constSkel.getSoftBodyNode(0), softBody);
  EXPECT_EQ(constSkel.getSoftBodyNode("soft_body"), softBody);

  const Skeleton* constSkelPtr = skeleton.get();
  EXPECT_EQ(constSkelPtr->getSoftBodyNode(0), softBody);
  EXPECT_EQ(constSkelPtr->getSoftBodyNode("soft_body"), softBody);

  skeleton->integratePositions(0.01);
  skeleton->integrateVelocities(0.01);

  Eigen::VectorXd velocityChanges
      = Eigen::VectorXd::Zero(skeleton->getNumDofs());
  skeleton->integratePositions(0.01, velocityChanges);
}

//==============================================================================
TEST(SkeletonClone, ComplexCloneWithShapesAndIK)
{
  auto skeleton = createComplexCloneSkeleton();
  const std::size_t dofs = skeleton->getNumDofs();
  const Eigen::VectorXd positions = makeSequence(dofs, 0.1, 0.05);
  const Eigen::VectorXd velocities = makeSequence(dofs, -0.2, 0.04);
  const Eigen::VectorXd accelerations = makeSequence(dofs, 0.3, -0.02);

  skeleton->setPositions(positions);
  skeleton->setVelocities(velocities);
  skeleton->setAccelerations(accelerations);

  auto clone = skeleton->cloneSkeleton("complex_clone");
  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->getNumBodyNodes(), skeleton->getNumBodyNodes());
  EXPECT_EQ(clone->getNumJoints(), skeleton->getNumJoints());

  auto* clonedFree = clone->getBodyNode("free_body");
  ASSERT_NE(clonedFree, nullptr);
  EXPECT_GT(clonedFree->getNumShapeNodes(), 0u);
  EXPECT_NE(clonedFree->getShapeNode(0), nullptr);

  auto originalIk = skeleton->getBodyNode("free_body")->getIK();
  auto cloneIk = clonedFree->getIK();
  EXPECT_NE(cloneIk, nullptr);
  EXPECT_NE(cloneIk.get(), originalIk.get());

  EXPECT_TRUE(clone->getPositions().isApprox(positions));
  EXPECT_TRUE(clone->getVelocities().isApprox(velocities));
  EXPECT_TRUE(clone->getAccelerations().isApprox(accelerations));
}

//==============================================================================
TEST(SkeletonClone, MimicJointReferenceUpdated)
{
  auto skeleton = createComplexCloneSkeleton();
  auto* mimicJoint = skeleton->getJoint("pris_joint");
  auto* referenceJoint = skeleton->getJoint("rev_joint");
  ASSERT_NE(mimicJoint, nullptr);
  ASSERT_NE(referenceJoint, nullptr);
  EXPECT_EQ(mimicJoint->getMimicJoint(0), referenceJoint);

  auto clone = skeleton->cloneSkeleton("mimic_clone");
  auto* clonedMimic = clone->getJoint("pris_joint");
  auto* clonedReference = clone->getJoint("rev_joint");
  ASSERT_NE(clonedMimic, nullptr);
  ASSERT_NE(clonedReference, nullptr);
  auto* clonedMimicRef = clonedMimic->getMimicJoint(0);
  ASSERT_NE(clonedMimicRef, nullptr);
  EXPECT_EQ(clonedMimicRef->getName(), clonedReference->getName());
  EXPECT_NE(clonedReference, referenceJoint);
  EXPECT_DOUBLE_EQ(clonedMimic->getMimicMultiplier(0), 1.25);
  EXPECT_DOUBLE_EQ(clonedMimic->getMimicOffset(0), -0.2);
}

//==============================================================================
TEST(SkeletonSupportPolygon, NoGravityAxesAndCentroid)
{
  auto skeleton = createSkeletonWithEndEffectors();
  skeleton->setGravity(Eigen::Vector3d::Zero());

  const auto& polygon = skeleton->getSupportPolygon();
  EXPECT_TRUE(polygon.empty());

  const auto axes = skeleton->getSupportAxes();
  EXPECT_TRUE(axes.first.isZero());
  EXPECT_TRUE(axes.second.isZero());

  const auto centroid = skeleton->getSupportCentroid();
  EXPECT_TRUE(std::isnan(centroid.x()));
  EXPECT_TRUE(std::isnan(centroid.y()));
}

//==============================================================================
TEST(SkeletonSupportPolygon, SupportVersionDirtyForSkeleton)
{
  auto skeleton = createSkeletonWithEndEffectors();
  skeleton->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

  const auto dirtyVersion = skeleton->getSupportVersion();
  const auto& polygon = skeleton->getSupportPolygon();
  (void)polygon;
  const auto cleanVersion = skeleton->getSupportVersion();
  EXPECT_GE(cleanVersion, dirtyVersion);

  const auto indices = skeleton->getSupportIndices();
  EXPECT_LE(indices.size(), skeleton->getNumEndEffectors());

  const auto axes = skeleton->getSupportAxes();
  EXPECT_TRUE(axes.first.array().isFinite().all());
  EXPECT_TRUE(axes.second.array().isFinite().all());
}

//==============================================================================
TEST(SkeletonSupportPolygon, SupportAxesAndVersionPerTree)
{
  auto skeleton = createSkeletonWithEndEffectors();
  skeleton->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

  const auto dirtyVersion = skeleton->getSupportVersion(0);
  const auto& polygon = skeleton->getSupportPolygon(0);
  (void)polygon;
  const auto cleanVersion = skeleton->getSupportVersion(0);
  EXPECT_GE(cleanVersion, dirtyVersion);

  const auto axes = skeleton->getSupportAxes(0);
  EXPECT_TRUE(axes.first.array().isFinite().all());
  EXPECT_TRUE(axes.second.array().isFinite().all());

  const auto centroid = skeleton->getSupportCentroid(0);
  if (!std::isnan(centroid.x()) && !std::isnan(centroid.y())) {
    EXPECT_TRUE(std::isfinite(centroid.x()));
    EXPECT_TRUE(std::isfinite(centroid.y()));
  }
}

//==============================================================================
TEST(SkeletonImpulse, ComputePositionVelocityChangesImmobile)
{
  auto skeleton = Skeleton::create("pos_vel_changes_immobile");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());

  skeleton->setMobile(false);
  skeleton->computePositionVelocityChanges();
  EXPECT_TRUE(skeleton->getPositionVelocityChanges().isZero());
}

//==============================================================================
TEST(SkeletonImpulse, ComputePositionVelocityChangesMobile)
{
  auto skeleton = Skeleton::create("pos_vel_changes_mobile");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());

  skeleton->computePositionVelocityChanges();
  const auto& changes = skeleton->getPositionVelocityChanges();
  EXPECT_EQ(changes.size(), static_cast<Eigen::Index>(skeleton->getNumDofs()));
  EXPECT_TRUE(changes.array().isFinite().all());
}

//==============================================================================
TEST(SkeletonImpulse, ImpulseForwardDynamicsImmobile)
{
  auto skeleton = Skeleton::create("impulse_fd_immobile");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());

  skeleton->setMobile(false);
  skeleton->computeImpulseForwardDynamics();
}

//==============================================================================
TEST(SkeletonTreeManipulation, MoveRootBetweenSkeletons)
{
  auto source = Skeleton::create("move_root_source");
  auto rootPair = source->createJointAndBodyNodePair<FreeJoint>();
  rootPair.first->setName("root_joint");
  rootPair.second->setName("root_body");
  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  childPair.first->setAxis(Eigen::Vector3d::UnitX());
  childPair.second->setName("child_body");
  childPair.second->createEndEffector("child_ee");

  auto destination = Skeleton::create("move_root_dest");
  auto destRoot = destination->createJointAndBodyNodePair<FreeJoint>();
  destRoot.second->setName("dest_root");

  EXPECT_TRUE(rootPair.second->moveTo(destination, nullptr));
  EXPECT_EQ(source->getNumBodyNodes(), 0u);
  EXPECT_EQ(destination->getNumBodyNodes(), 3u);
  EXPECT_EQ(destination->getNumTrees(), 2u);
  EXPECT_EQ(rootPair.second->getSkeleton(), destination);
}

//==============================================================================
TEST(SkeletonAccessors, DynamicsCacheQueries)
{
  auto skeleton = Skeleton::create("cache_queries");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  rootPair.second->setMass(1.0);
  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  childPair.first->setAxis(Eigen::Vector3d::UnitZ());
  childPair.second->setMass(0.5);

  skeleton->setPositions(Eigen::VectorXd::Random(skeleton->getNumDofs()));
  skeleton->setVelocities(Eigen::VectorXd::Random(skeleton->getNumDofs()));

  const Skeleton& constSkel = *skeleton;
  const auto& M = constSkel.getMassMatrix();
  const auto& Maug = constSkel.getAugMassMatrix();
  const auto& Minv = constSkel.getInvMassMatrix();
  const auto& MinvAug = constSkel.getInvAugMassMatrix();
  const auto& C = constSkel.getCoriolisForces();
  const auto& G = constSkel.getGravityForces();
  const auto& CG = constSkel.getCoriolisAndGravityForces();
  const auto& Fext = constSkel.getExternalForces();

  EXPECT_EQ(M.rows(), static_cast<int>(constSkel.getNumDofs()));
  EXPECT_EQ(Maug.rows(), static_cast<int>(constSkel.getNumDofs()));
  EXPECT_EQ(Minv.cols(), static_cast<int>(constSkel.getNumDofs()));
  EXPECT_EQ(MinvAug.cols(), static_cast<int>(constSkel.getNumDofs()));
  EXPECT_EQ(C.size(), static_cast<int>(constSkel.getNumDofs()));
  EXPECT_EQ(G.size(), static_cast<int>(constSkel.getNumDofs()));
  EXPECT_EQ(CG.size(), static_cast<int>(constSkel.getNumDofs()));
  EXPECT_EQ(Fext.size(), static_cast<int>(constSkel.getNumDofs()));
}

//==============================================================================
TEST(SkeletonAccessors, MultiTreeMassMatricesAndCaches)
{
  auto skeleton = Skeleton::create("multi_tree_cache");
  auto rootA = skeleton->createJointAndBodyNodePair<FreeJoint>();
  rootA.second->setMass(1.0);
  auto childA = rootA.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  childA.first->setAxis(Eigen::Vector3d::UnitZ());
  childA.second->setMass(0.5);

  auto rootB = skeleton->createJointAndBodyNodePair<FreeJoint>();
  rootB.second->setMass(1.2);
  auto childB = rootB.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  childB.first->setAxis(Eigen::Vector3d::UnitY());
  childB.second->setMass(0.4);

  ASSERT_EQ(skeleton->getNumTrees(), 2u);

  Eigen::VectorXd positions = Eigen::VectorXd::LinSpaced(
      static_cast<int>(skeleton->getNumDofs()),
      0.1,
      0.1 + static_cast<double>(skeleton->getNumDofs() - 1));
  Eigen::VectorXd velocities = Eigen::VectorXd::LinSpaced(
      static_cast<int>(skeleton->getNumDofs()), -0.2, 0.2);
  skeleton->setPositions(positions);
  skeleton->setVelocities(velocities);
  skeleton->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

  childB.second->addExtForce(
      Eigen::Vector3d(2.0, -1.0, 0.5), Eigen::Vector3d::Zero(), true, true);

  const auto tree0Dofs = skeleton->getTreeDofs(0);
  const auto tree1Dofs = skeleton->getTreeDofs(1);

  const auto& M0 = skeleton->getMassMatrix(0);
  const auto& M1 = skeleton->getMassMatrix(1);
  EXPECT_EQ(M0.rows(), static_cast<int>(tree0Dofs.size()));
  EXPECT_EQ(M1.cols(), static_cast<int>(tree1Dofs.size()));
  EXPECT_TRUE(M0.allFinite());
  EXPECT_TRUE(M1.allFinite());

  const auto& M0_cached = skeleton->getMassMatrix(0);
  const auto& M1_cached = skeleton->getMassMatrix(1);
  EXPECT_EQ(M0_cached.rows(), M0.rows());
  EXPECT_EQ(M1_cached.cols(), M1.cols());

  EXPECT_TRUE(skeleton->getAugMassMatrix(0).allFinite());
  EXPECT_TRUE(skeleton->getAugMassMatrix(1).allFinite());
  EXPECT_TRUE(skeleton->getInvMassMatrix(0).allFinite());
  EXPECT_TRUE(skeleton->getInvMassMatrix(1).allFinite());
  EXPECT_TRUE(skeleton->getInvAugMassMatrix(0).allFinite());
  EXPECT_TRUE(skeleton->getInvAugMassMatrix(1).allFinite());

  EXPECT_TRUE(skeleton->getCoriolisForces(0).allFinite());
  EXPECT_TRUE(skeleton->getCoriolisForces(1).allFinite());
  EXPECT_TRUE(skeleton->getGravityForces(0).allFinite());
  EXPECT_TRUE(skeleton->getGravityForces(1).allFinite());
  EXPECT_TRUE(skeleton->getCoriolisAndGravityForces(0).allFinite());
  EXPECT_TRUE(skeleton->getCoriolisAndGravityForces(1).allFinite());
  EXPECT_TRUE(skeleton->getExternalForces(0).allFinite());
  EXPECT_TRUE(skeleton->getExternalForces(1).allFinite());

  const auto& M = skeleton->getMassMatrix();
  EXPECT_EQ(M.rows(), static_cast<int>(skeleton->getNumDofs()));
  EXPECT_EQ(M.cols(), static_cast<int>(skeleton->getNumDofs()));
  EXPECT_TRUE(M.allFinite());
}

//==============================================================================
TEST(SkeletonImpulse, BiasImpulseAndVelocityChange)
{
  auto skeleton = Skeleton::create("bias_impulse_test");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  rootPair.second->setMass(1.0);
  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  childPair.first->setAxis(Eigen::Vector3d::UnitZ());
  childPair.second->setMass(0.5);

  Eigen::Vector6d impulseA;
  impulseA << 0.1, -0.2, 0.3, 0.4, -0.5, 0.6;
  Eigen::Vector6d impulseB;
  impulseB << -0.3, 0.2, -0.1, 0.5, 0.1, -0.2;

  skeleton->updateBiasImpulse(rootPair.second);
  skeleton->updateBiasImpulse(rootPair.second, impulseA);
  skeleton->updateBiasImpulse(
      rootPair.second, impulseA, childPair.second, impulseB);

  skeleton->updateVelocityChange();
  const auto velocityChanges = skeleton->getVelocityChanges();
  EXPECT_EQ(velocityChanges.size(), static_cast<int>(skeleton->getNumDofs()));

  childPair.second->addPositionConstraintImpulse(Eigen::Vector6d::Ones());
  skeleton->computePositionVelocityChanges();
  EXPECT_EQ(
      skeleton->getPositionVelocityChanges().size(),
      static_cast<int>(skeleton->getNumDofs()));

  skeleton->computeImpulseForwardDynamics();
}

//==============================================================================
TEST(SkeletonClone, CloneSkeletonDefaultName)
{
  auto skeleton = Skeleton::create("clone_default");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());
  pair.second->setMass(1.0);

  skeleton->setPositions(Eigen::VectorXd::Constant(
      static_cast<Eigen::Index>(skeleton->getNumDofs()), 0.25));

  auto clone = skeleton->cloneSkeleton();
  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->getName(), skeleton->getName());
  EXPECT_EQ(clone->getNumBodyNodes(), skeleton->getNumBodyNodes());
  EXPECT_TRUE(clone->getPositions().isApprox(skeleton->getPositions()));
}
