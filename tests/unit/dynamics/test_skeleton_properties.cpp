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
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

using namespace dart::dynamics;

static SkeletonPtr createTestSkeleton()
{
  auto skeleton = Skeleton::create("skeleton_props");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  childPair.first->setAxis(Eigen::Vector3d::UnitY());
  return skeleton;
}

TEST(SkeletonProperties, FlagsAndTiming)
{
  auto skeleton = Skeleton::create("TestSkeleton");
  EXPECT_EQ(skeleton->getName(), "TestSkeleton");

  EXPECT_TRUE(skeleton->isMobile());
  skeleton->setMobile(false);
  EXPECT_FALSE(skeleton->isMobile());
  skeleton->setMobile(true);
  EXPECT_TRUE(skeleton->isMobile());

  const double timeStep = 0.002;
  skeleton->setTimeStep(timeStep);
  EXPECT_DOUBLE_EQ(skeleton->getTimeStep(), timeStep);

  const Eigen::Vector3d gravity(0.0, -9.81, 0.0);
  skeleton->setGravity(gravity);
  EXPECT_TRUE(skeleton->getGravity().isApprox(gravity));

  EXPECT_FALSE(skeleton->getSelfCollisionCheck());
  skeleton->setSelfCollisionCheck(true);
  EXPECT_TRUE(skeleton->getSelfCollisionCheck());
  EXPECT_TRUE(skeleton->isEnabledSelfCollisionCheck());
  skeleton->setSelfCollisionCheck(false);
  EXPECT_FALSE(skeleton->getSelfCollisionCheck());

  EXPECT_FALSE(skeleton->getAdjacentBodyCheck());
  skeleton->setAdjacentBodyCheck(true);
  skeleton->setSelfCollisionCheck(true);
  EXPECT_TRUE(skeleton->getAdjacentBodyCheck());
  EXPECT_TRUE(skeleton->isEnabledAdjacentBodyCheck());
  skeleton->setAdjacentBodyCheck(false);
  EXPECT_FALSE(skeleton->getAdjacentBodyCheck());
}

TEST(SkeletonProperties, EnableDisableWrappers)
{
  auto skeleton = Skeleton::create("WrapperTest");

  skeleton->enableSelfCollisionCheck();
  EXPECT_TRUE(skeleton->isEnabledSelfCollisionCheck());
  skeleton->disableSelfCollisionCheck();
  EXPECT_FALSE(skeleton->isEnabledSelfCollisionCheck());

  skeleton->enableAdjacentBodyCheck();
  EXPECT_TRUE(skeleton->isEnabledAdjacentBodyCheck());
  skeleton->disableAdjacentBodyCheck();
  EXPECT_FALSE(skeleton->isEnabledAdjacentBodyCheck());
}

TEST(SkeletonProperties, StateAndPropertiesRoundTrip)
{
  auto skeleton = createTestSkeleton();

  skeleton->setMobile(false);
  skeleton->setGravity(Eigen::Vector3d(0.0, -5.0, 0.0));
  skeleton->setTimeStep(0.005);
  skeleton->enableSelfCollisionCheck();
  skeleton->enableAdjacentBodyCheck();

  const auto state = skeleton->getState();
  const auto props = skeleton->getProperties();

  auto skeleton2 = Skeleton::create("clone");
  skeleton2->createJointAndBodyNodePair<FreeJoint>();
  skeleton2->getBodyNode(0)->createChildJointAndBodyNodePair<RevoluteJoint>();

  skeleton2->setState(state);
  skeleton2->setProperties(props);

  EXPECT_EQ(skeleton2->getName(), skeleton->getName());
  EXPECT_EQ(skeleton2->isMobile(), skeleton->isMobile());
  EXPECT_TRUE(skeleton2->getGravity().isApprox(skeleton->getGravity()));
  EXPECT_DOUBLE_EQ(skeleton2->getTimeStep(), skeleton->getTimeStep());
  EXPECT_EQ(
      skeleton2->isEnabledSelfCollisionCheck(),
      skeleton->isEnabledSelfCollisionCheck());
  EXPECT_EQ(
      skeleton2->isEnabledAdjacentBodyCheck(),
      skeleton->isEnabledAdjacentBodyCheck());
}

TEST(SkeletonProperties, AspectPropertiesSetAll)
{
  auto skeleton = createTestSkeleton();

  Skeleton::AspectProperties aprops;
  aprops.mName = "aspect_test";
  aprops.mIsMobile = false;
  aprops.mGravity = Eigen::Vector3d(1.0, 2.0, 3.0);
  aprops.mTimeStep = 0.01;
  aprops.mEnabledSelfCollisionCheck = true;
  aprops.mEnabledAdjacentBodyCheck = true;

  skeleton->setAspectProperties(aprops);

  EXPECT_EQ(skeleton->getName(), "aspect_test");
  EXPECT_FALSE(skeleton->isMobile());
  EXPECT_TRUE(skeleton->getGravity().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_DOUBLE_EQ(skeleton->getTimeStep(), 0.01);
  EXPECT_TRUE(skeleton->isEnabledSelfCollisionCheck());
  EXPECT_TRUE(skeleton->isEnabledAdjacentBodyCheck());
}

TEST(SkeletonConfiguration, GetConfigWithCommands)
{
  auto skeleton = createTestSkeleton();
  const std::size_t dofs = skeleton->getNumDofs();

  Eigen::VectorXd cmds = Eigen::VectorXd::Ones(dofs) * 0.5;
  skeleton->setCommands(cmds);

  const auto config = skeleton->getConfiguration(Skeleton::CONFIG_COMMANDS);
  EXPECT_EQ(config.mCommands.size(), static_cast<int>(dofs));
  EXPECT_TRUE(config.mCommands.isApprox(cmds));

  const auto configAll = skeleton->getConfiguration(
      Skeleton::CONFIG_POSITIONS | Skeleton::CONFIG_VELOCITIES
      | Skeleton::CONFIG_ACCELERATIONS | Skeleton::CONFIG_FORCES
      | Skeleton::CONFIG_COMMANDS);
  EXPECT_EQ(configAll.mPositions.size(), static_cast<int>(dofs));
  EXPECT_EQ(configAll.mVelocities.size(), static_cast<int>(dofs));
  EXPECT_EQ(configAll.mAccelerations.size(), static_cast<int>(dofs));
  EXPECT_EQ(configAll.mForces.size(), static_cast<int>(dofs));
  EXPECT_EQ(configAll.mCommands.size(), static_cast<int>(dofs));
}

TEST(SkeletonConfiguration, GetConfigWithIndicesAndCommands)
{
  auto skeleton = createTestSkeleton();
  const std::size_t dofs = skeleton->getNumDofs();
  ASSERT_GT(dofs, 2u);

  Eigen::VectorXd cmds = Eigen::VectorXd::Ones(dofs) * 0.3;
  skeleton->setCommands(cmds);

  std::vector<std::size_t> indices = {0, 1};
  const auto config = skeleton->getConfiguration(
      indices,
      Skeleton::CONFIG_COMMANDS | Skeleton::CONFIG_FORCES
          | Skeleton::CONFIG_ACCELERATIONS);
  EXPECT_EQ(config.mCommands.size(), static_cast<int>(indices.size()));
  EXPECT_EQ(config.mForces.size(), static_cast<int>(indices.size()));
  EXPECT_EQ(config.mAccelerations.size(), static_cast<int>(indices.size()));
  EXPECT_EQ(config.mPositions.size(), 0);
  EXPECT_EQ(config.mVelocities.size(), 0);
}

TEST(SkeletonConfiguration, GetConfigCommandsForcesAccelerations)
{
  auto skeleton = Skeleton::create("cmd_test");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.second->createChildJointAndBodyNodePair<RevoluteJoint>();

  const std::size_t dofs = skeleton->getNumDofs();
  ASSERT_EQ(dofs, 2u);

  Eigen::VectorXd forces(2);
  forces << 1.0, 2.0;
  Eigen::VectorXd accelerations(2);
  accelerations << -0.1, 0.4;

  skeleton->setForces(forces);
  skeleton->setAccelerations(accelerations);

  // For FORCE actuator type, setForce() also writes mCommands, so commands
  // should match forces after the call above.
  const auto config = skeleton->getConfiguration(
      Skeleton::CONFIG_COMMANDS | Skeleton::CONFIG_FORCES
      | Skeleton::CONFIG_ACCELERATIONS);
  EXPECT_EQ(config.mPositions.size(), 0);
  EXPECT_EQ(config.mVelocities.size(), 0);
  EXPECT_TRUE(config.mCommands.isApprox(forces));
  EXPECT_TRUE(config.mForces.isApprox(forces));
  EXPECT_TRUE(config.mAccelerations.isApprox(accelerations));
}

TEST(SkeletonConfiguration, UpdatesSelectedDofs)
{
  auto skeleton = createTestSkeleton();
  const std::size_t dofs = skeleton->getNumDofs();

  skeleton->setPositions(Eigen::VectorXd::Zero(dofs));
  skeleton->setVelocities(Eigen::VectorXd::Zero(dofs));

  std::vector<std::size_t> indices = {0, 3, dofs - 1};
  Eigen::VectorXd positions(indices.size());
  positions << 1.2, -0.4, 0.7;

  Eigen::VectorXd velocities(indices.size());
  velocities << 0.2, -0.1, 0.3;

  Skeleton::Configuration config(indices, positions, velocities);
  skeleton->setConfiguration(config);

  const Eigen::VectorXd updatedPositions = skeleton->getPositions();
  const Eigen::VectorXd updatedVelocities = skeleton->getVelocities();

  for (std::size_t i = 0; i < indices.size(); ++i) {
    EXPECT_DOUBLE_EQ(updatedPositions[indices[i]], positions[i]);
    EXPECT_DOUBLE_EQ(updatedVelocities[indices[i]], velocities[i]);
  }

  for (std::size_t i = 0; i < dofs; ++i) {
    if (std::find(indices.begin(), indices.end(), i) == indices.end()) {
      EXPECT_DOUBLE_EQ(updatedPositions[i], 0.0);
      EXPECT_DOUBLE_EQ(updatedVelocities[i], 0.0);
    }
  }

  const auto subsetConfig
      = skeleton->getConfiguration(indices, Skeleton::CONFIG_VELOCITIES);
  EXPECT_EQ(subsetConfig.mPositions.size(), 0);
  EXPECT_EQ(subsetConfig.mVelocities.size(), positions.size());
  EXPECT_TRUE(subsetConfig.mVelocities.isApprox(velocities));
}

TEST(SkeletonConfiguration, ConfigurationFlagsForForcesAndCommands)
{
  auto skeleton = Skeleton::create("config_flags_skel");
  auto root = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  root.first->setAxis(Eigen::Vector3d::UnitZ());
  auto child = root.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child.first->setAxis(Eigen::Vector3d::UnitY());

  ASSERT_EQ(skeleton->getNumDofs(), 2u);

  Eigen::VectorXd forces(2);
  forces << 1.0, -2.0;
  skeleton->setForces(forces);

  Eigen::VectorXd accelerations(2);
  accelerations << 0.4, -0.3;
  skeleton->setAccelerations(accelerations);

  Eigen::VectorXd commands(2);
  commands << -0.5, 0.25;
  skeleton->setCommands(commands);

  const auto config = skeleton->getConfiguration(
      Skeleton::CONFIG_FORCES | Skeleton::CONFIG_ACCELERATIONS
      | Skeleton::CONFIG_COMMANDS);
  EXPECT_EQ(config.mPositions.size(), 0);
  EXPECT_EQ(config.mVelocities.size(), 0);
  EXPECT_TRUE(config.mForces.isApprox(forces));
  EXPECT_TRUE(config.mAccelerations.isApprox(accelerations));
  EXPECT_TRUE(config.mCommands.isApprox(commands));
}

#include <dart/dart.hpp>

TEST(SkeletonProperties, TreeLevelMassMatrixGetters)
{
  auto skeleton = Skeleton::create("multi_tree_matrices");
  auto root1 = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto child1 = root1.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child1.first->setAxis(Eigen::Vector3d::UnitY());

  auto root2 = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto child2 = root2.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child2.first->setAxis(Eigen::Vector3d::UnitX());

  ASSERT_EQ(skeleton->getNumTrees(), 2u);

  Eigen::Vector6d root1Pos = Eigen::Vector6d::Zero();
  Eigen::Vector6d root2Pos = Eigen::Vector6d::Zero();
  root2Pos[0] = 1.0;
  root1.first->setPositions(root1Pos);
  root2.first->setPositions(root2Pos);

  Eigen::VectorXd childPos(1);
  childPos[0] = 0.25;
  child1.first->setPositions(childPos);
  childPos[0] = -0.15;
  child2.first->setPositions(childPos);

  Eigen::VectorXd velocities = Eigen::VectorXd::Zero(skeleton->getNumDofs());
  velocities[0] = 0.3;
  velocities[6] = -0.2;
  skeleton->setVelocities(velocities);

  root1.second->addExtForce(Eigen::Vector3d(1.0, -2.0, 0.5));
  root2.second->addExtForce(Eigen::Vector3d(-0.5, 1.5, 0.0));

  const auto tree0Dofs = skeleton->getTreeDofs(0);
  const auto tree1Dofs = skeleton->getTreeDofs(1);

  const auto& mass0 = skeleton->getMassMatrix(0);
  EXPECT_EQ(mass0.rows(), static_cast<int>(tree0Dofs.size()));
  EXPECT_EQ(mass0.cols(), static_cast<int>(tree0Dofs.size()));

  const auto& augMass0 = skeleton->getAugMassMatrix(0);
  EXPECT_EQ(augMass0.rows(), static_cast<int>(tree0Dofs.size()));
  EXPECT_EQ(augMass0.cols(), static_cast<int>(tree0Dofs.size()));

  const auto& invMass0 = skeleton->getInvMassMatrix(0);
  EXPECT_EQ(invMass0.rows(), static_cast<int>(tree0Dofs.size()));
  EXPECT_EQ(invMass0.cols(), static_cast<int>(tree0Dofs.size()));

  const auto& invAugMass0 = skeleton->getInvAugMassMatrix(0);
  EXPECT_EQ(invAugMass0.rows(), static_cast<int>(tree0Dofs.size()));
  EXPECT_EQ(invAugMass0.cols(), static_cast<int>(tree0Dofs.size()));

  const auto& coriolis0 = skeleton->getCoriolisForces(0);
  EXPECT_EQ(coriolis0.size(), static_cast<int>(tree0Dofs.size()));

  const auto& gravity0 = skeleton->getGravityForces(0);
  EXPECT_EQ(gravity0.size(), static_cast<int>(tree0Dofs.size()));

  const auto& cg0 = skeleton->getCoriolisAndGravityForces(0);
  EXPECT_EQ(cg0.size(), static_cast<int>(tree0Dofs.size()));

  const auto& ext0 = skeleton->getExternalForces(0);
  EXPECT_EQ(ext0.size(), static_cast<int>(tree0Dofs.size()));

  const auto& constraint0 = skeleton->getConstraintForces(0);
  EXPECT_EQ(constraint0.size(), static_cast<int>(tree0Dofs.size()));

  const auto& mass1 = skeleton->getMassMatrix(1);
  EXPECT_EQ(mass1.rows(), static_cast<int>(tree1Dofs.size()));
  EXPECT_EQ(mass1.cols(), static_cast<int>(tree1Dofs.size()));

  const auto& augMass1 = skeleton->getAugMassMatrix(1);
  EXPECT_EQ(augMass1.rows(), static_cast<int>(tree1Dofs.size()));
  EXPECT_EQ(augMass1.cols(), static_cast<int>(tree1Dofs.size()));

  const auto& invMass1 = skeleton->getInvMassMatrix(1);
  EXPECT_EQ(invMass1.rows(), static_cast<int>(tree1Dofs.size()));
  EXPECT_EQ(invMass1.cols(), static_cast<int>(tree1Dofs.size()));

  const auto& invAugMass1 = skeleton->getInvAugMassMatrix(1);
  EXPECT_EQ(invAugMass1.rows(), static_cast<int>(tree1Dofs.size()));
  EXPECT_EQ(invAugMass1.cols(), static_cast<int>(tree1Dofs.size()));

  const auto& coriolis1 = skeleton->getCoriolisForces(1);
  EXPECT_EQ(coriolis1.size(), static_cast<int>(tree1Dofs.size()));

  const auto& gravity1 = skeleton->getGravityForces(1);
  EXPECT_EQ(gravity1.size(), static_cast<int>(tree1Dofs.size()));

  const auto& cg1 = skeleton->getCoriolisAndGravityForces(1);
  EXPECT_EQ(cg1.size(), static_cast<int>(tree1Dofs.size()));

  const auto& ext1 = skeleton->getExternalForces(1);
  EXPECT_EQ(ext1.size(), static_cast<int>(tree1Dofs.size()));

  const auto& constraint1 = skeleton->getConstraintForces(1);
  EXPECT_EQ(constraint1.size(), static_cast<int>(tree1Dofs.size()));

  Eigen::VectorXd positions = skeleton->getPositions();
  positions[0] += 0.2;
  positions[6] -= 0.1;
  skeleton->setPositions(positions);

  velocities = skeleton->getVelocities();
  velocities[0] -= 0.15;
  velocities[6] += 0.05;
  skeleton->setVelocities(velocities);

  (void)skeleton->getMassMatrix(0);
  (void)skeleton->getAugMassMatrix(1);
  (void)skeleton->getInvMassMatrix(0);
  (void)skeleton->getInvAugMassMatrix(1);
  (void)skeleton->getCoriolisForces(1);
  (void)skeleton->getGravityForces(0);
  (void)skeleton->getCoriolisAndGravityForces(1);
  (void)skeleton->getExternalForces(0);
}

TEST(SkeletonProperties, ClearExternalAndInternalForces)
{
  auto skeleton = Skeleton::create("force_clear");
  auto root = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto child = root.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child.first->setAxis(Eigen::Vector3d::UnitZ());

  root.second->addExtForce(Eigen::Vector3d(1.0, 0.0, 0.0));
  child.second->addExtForce(Eigen::Vector3d(0.0, -2.0, 1.0));
  EXPECT_FALSE(root.second->getExternalForceLocal().isZero());
  EXPECT_FALSE(child.second->getExternalForceLocal().isZero());

  skeleton->clearExternalForces();
  EXPECT_TRUE(root.second->getExternalForceLocal().isZero(0.0));
  EXPECT_TRUE(child.second->getExternalForceLocal().isZero(0.0));

  Eigen::VectorXd rootForces
      = Eigen::VectorXd::Constant(root.first->getNumDofs(), 1.5);
  Eigen::VectorXd childForces
      = Eigen::VectorXd::Constant(child.first->getNumDofs(), -0.75);
  root.first->setForces(rootForces);
  child.first->setForces(childForces);
  EXPECT_FALSE(root.first->getForces().isZero(0.0));
  EXPECT_FALSE(child.first->getForces().isZero(0.0));

  skeleton->clearInternalForces();
  EXPECT_TRUE(root.first->getForces().isZero(0.0));
  EXPECT_TRUE(child.first->getForces().isZero(0.0));
}

TEST(SkeletonProperties, ClearConstraintAndPositionImpulses)
{
  auto skeleton = Skeleton::create("impulse_clear");
  auto root = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto child = root.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child.first->setAxis(Eigen::Vector3d::UnitX());

  const Eigen::Vector6d impulse = Eigen::Vector6d::Ones();
  root.second->addConstraintImpulse(impulse);
  child.second->addConstraintImpulse(-impulse);
  root.second->addPositionConstraintImpulse(0.5 * impulse);
  child.second->addPositionConstraintImpulse(-0.25 * impulse);

  EXPECT_FALSE(root.second->getConstraintImpulse().isZero(0.0));
  EXPECT_FALSE(child.second->getConstraintImpulse().isZero(0.0));
  EXPECT_FALSE(root.second->getPositionConstraintImpulse().isZero(0.0));
  EXPECT_FALSE(child.second->getPositionConstraintImpulse().isZero(0.0));

  skeleton->clearConstraintImpulses();
  EXPECT_TRUE(root.second->getConstraintImpulse().isZero(0.0));
  EXPECT_TRUE(child.second->getConstraintImpulse().isZero(0.0));

  skeleton->clearPositionConstraintImpulses();
  EXPECT_TRUE(root.second->getPositionConstraintImpulse().isZero(0.0));
  EXPECT_TRUE(child.second->getPositionConstraintImpulse().isZero(0.0));
}

TEST(SkeletonProperties, BiasImpulseAndVelocityChangeUpdates)
{
  auto skeleton = Skeleton::create("bias_impulse");
  auto root = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto child = root.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  child.first->setAxis(Eigen::Vector3d::UnitY());

  Eigen::VectorXd positions = Eigen::VectorXd::Zero(skeleton->getNumDofs());
  positions[0] = 0.2;
  positions[6] = -0.1;
  skeleton->setPositions(positions);

  skeleton->updateBiasImpulse(child.second);

  const Eigen::Vector6d impulse = Eigen::Vector6d::Ones() * 0.1;
  skeleton->updateBiasImpulse(child.second, impulse);
  EXPECT_TRUE(child.second->getConstraintImpulse().isZero(0.0));

  Eigen::Vector6d impulse1 = Eigen::Vector6d::Zero();
  Eigen::Vector6d impulse2 = Eigen::Vector6d::Zero();
  impulse1[0] = 1.0;
  impulse2[2] = 1.0;
  skeleton->updateBiasImpulse(root.second, impulse1, child.second, impulse2);
  EXPECT_TRUE(root.second->getConstraintImpulse().isZero(0.0));
  EXPECT_TRUE(child.second->getConstraintImpulse().isZero(0.0));

  skeleton->updateVelocityChange();

  root.second->addPositionConstraintImpulse(Eigen::Vector6d::UnitY() * 0.05);
  skeleton->computePositionVelocityChanges();
  EXPECT_EQ(
      skeleton->getPositionVelocityChanges().size(),
      static_cast<int>(skeleton->getNumDofs()));

  skeleton->setImpulseApplied(true);
  EXPECT_TRUE(skeleton->isImpulseApplied());
  skeleton->setPositionImpulseApplied(true);
  EXPECT_TRUE(skeleton->isPositionImpulseApplied());
}

TEST(SkeletonProperties, ConstraintImpulsePathsViaWorldStep)
{
  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  auto skel1 = Skeleton::create("contact_1");
  auto root1 = skel1->createJointAndBodyNodePair<FreeJoint>();
  auto shape1 = std::make_shared<BoxShape>(Eigen::Vector3d(0.2, 0.2, 0.2));
  root1.second->createShapeNodeWith<VisualAspect, CollisionAspect>(shape1);
  root1.second->setMass(1.0);

  auto skel2 = Skeleton::create("contact_2");
  auto root2 = skel2->createJointAndBodyNodePair<FreeJoint>();
  auto shape2 = std::make_shared<BoxShape>(Eigen::Vector3d(0.2, 0.2, 0.2));
  root2.second->createShapeNodeWith<VisualAspect, CollisionAspect>(shape2);
  root2.second->setMass(1.0);

  Eigen::Vector6d pos1 = Eigen::Vector6d::Zero();
  Eigen::Vector6d pos2 = Eigen::Vector6d::Zero();
  pos2[1] = 0.05;
  root1.first->setPositions(pos1);
  root2.first->setPositions(pos2);

  world->addSkeleton(skel1);
  world->addSkeleton(skel2);
  EXPECT_EQ(world->getNumSkeletons(), 2u);

  world->step();
  EXPECT_EQ(world->getNumSkeletons(), 2u);
}

//==============================================================================
// Test const overloads and span-returning variants of Skeleton/BodyNode/Joint
// These cover lines in skeleton.cpp, body_node.cpp, meta_skeleton.cpp that are
// only reached when calling through const references.
TEST(SkeletonProperties, ConstOverloadsAndSpanVariants)
{
  using namespace dart::dynamics;

  // Build a multi-tree skeleton with soft body node
  auto skel = Skeleton::create("const_test");

  // Tree 0: FreeJoint -> revolute -> weld
  auto [freeJ, freeBody] = skel->createJointAndBodyNodePair<FreeJoint>();
  freeBody->setName("freeBody");
  freeBody->setMass(1.0);
  auto [revJ, revBody]
      = freeBody->createChildJointAndBodyNodePair<RevoluteJoint>();
  revJ->setName("revJoint");
  revBody->setName("revBody");
  revBody->setMass(1.0);
  revBody->createShapeNodeWith<VisualAspect, CollisionAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d(0.1, 0.1, 0.1)));

  // Tree 1: Another FreeJoint root
  auto [freeJ2, freeBody2] = skel->createJointAndBodyNodePair<FreeJoint>();
  freeBody2->setName("freeBody2");
  freeBody2->setMass(1.0);

  ASSERT_EQ(skel->getNumTrees(), 2u);
  ASSERT_GE(skel->getNumBodyNodes(), 3u);

  // Get a const reference to the skeleton
  const Skeleton& constSkel = *skel;

// Suppress deprecated warnings for testing const overloads that are deprecated
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

  auto constBodyNodes = constSkel.getBodyNodes();
  EXPECT_EQ(constBodyNodes.size(), skel->getNumBodyNodes());

  const BodyNode* constBn = constSkel.getBodyNode("revBody");
  ASSERT_NE(constBn, nullptr);
  EXPECT_EQ(constBn->getName(), "revBody");

  auto constNamedNodes = constSkel.getBodyNodes("freeBody");
  EXPECT_EQ(constNamedNodes.size(), 1u);

  auto constNoNodes = constSkel.getBodyNodes("nonexistent");
  EXPECT_EQ(constNoNodes.size(), 0u);

  EXPECT_TRUE(constSkel.hasBodyNode(skel->getBodyNode(0)));
  EXPECT_FALSE(constSkel.hasBodyNode(nullptr));

  const Joint* constJoint = constSkel.getJoint(0);
  ASSERT_NE(constJoint, nullptr);

  auto constJoints = constSkel.getJoints();
  EXPECT_EQ(constJoints.size(), skel->getNumJoints());

  const Joint* namedJoint = constSkel.getJoint("revJoint");
  ASSERT_NE(namedJoint, nullptr);

  auto tree0Bodies = constSkel.getTreeBodyNodes(0);
  EXPECT_GE(tree0Bodies.size(), 1u);
  auto tree1Bodies = constSkel.getTreeBodyNodes(1);
  EXPECT_GE(tree1Bodies.size(), 1u);

  const BodyNode* root0 = constSkel.getRootBodyNode(0);
  ASSERT_NE(root0, nullptr);
  const BodyNode* root1 = constSkel.getRootBodyNode(1);
  ASSERT_NE(root1, nullptr);

  auto dofSpan = skel->getDofs();
  EXPECT_EQ(static_cast<std::size_t>(dofSpan.size()), skel->getNumDofs());

  const DegreeOfFreedom* constDof = constSkel.getDof(0);
  ASSERT_NE(constDof, nullptr);

  auto constDofs = constSkel.getDofs();
  EXPECT_EQ(static_cast<std::size_t>(constDofs.size()), skel->getNumDofs());

#pragma GCC diagnostic pop

  // skeleton.cpp: getTreeDofs(treeIdx) const
  auto treeDofs0 = constSkel.getTreeDofs(0);
  EXPECT_GE(treeDofs0.size(), 1u);

  // --- Body node const accessors ---
  const BodyNode& constBody = *constBn;
  EXPECT_GE(constBody.getNumChildBodyNodes(), 0u);
  EXPECT_NE(constBody.getParentJoint(), nullptr);
  EXPECT_NE(constBody.getSkeleton().get(), nullptr);
}

//==============================================================================
// Test skeleton with zero DOFs (WeldJoint-only skeleton)
TEST(SkeletonProperties, ZeroDofSkeletonEdgeCases)
{
  using namespace dart::dynamics;

  auto skel = Skeleton::create("zero_dof");
  auto [weldJ, body] = skel->createJointAndBodyNodePair<WeldJoint>();
  body->setMass(1.0);

  ASSERT_EQ(skel->getNumDofs(), 0u);

  // skeleton.cpp lines ~1656, 1688: return zero vector when numDofs == 0
  auto positions = skel->getPositions();
  EXPECT_EQ(positions.size(), 0);
  auto velocities = skel->getVelocities();
  EXPECT_EQ(velocities.size(), 0);
  auto accelerations = skel->getAccelerations();
  EXPECT_EQ(accelerations.size(), 0);

  // Mass matrix and related for zero-DOF skeleton
  auto& massMatrix = skel->getMassMatrix();
  EXPECT_EQ(massMatrix.rows(), 0);
  EXPECT_EQ(massMatrix.cols(), 0);

  auto coriolisForces = skel->getCoriolisForces();
  EXPECT_EQ(coriolisForces.size(), 0);

  auto gravityForces = skel->getGravityForces();
  EXPECT_EQ(gravityForces.size(), 0);

  // COM Jacobian on zero-DOF (skeleton.cpp lines ~1749, 1796)
  auto comJac = skel->getCOMLinearJacobian();
  EXPECT_EQ(comJac.cols(), 0);

  // Integration with zero DOFs
  skel->integratePositions(0.001);
  skel->integrateVelocities(0.001);

  // Configuration operations
  auto config = skel->getConfiguration();
  skel->setConfiguration(config);

  // computeForwardKinematics on zero-dof
  skel->computeForwardKinematics();
}
