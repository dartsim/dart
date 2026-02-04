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
#include <dart/dynamics/degree_of_freedom.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/prismatic_joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/translational_joint.hpp>

#include <gtest/gtest.h>

#include <limits>

#include <cmath>

using namespace dart;
using namespace dart::dynamics;

//==============================================================================
// Helper function to create a simple skeleton with a revolute joint
SkeletonPtr createRevoluteSkeleton(const std::string& name = "test_skeleton")
{
  auto skel = Skeleton::create(name);

  auto pair = skel->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setName("revolute");
  pair.second->setName("body");

  return skel;
}

// Helper function to create a skeleton with a prismatic joint
SkeletonPtr createPrismaticSkeleton(const std::string& name = "test_skeleton")
{
  auto skel = Skeleton::create(name);

  auto pair = skel->createJointAndBodyNodePair<PrismaticJoint>();
  pair.first->setName("prismatic");
  pair.second->setName("body");

  return skel;
}

// Helper function to create a skeleton with a ball joint
SkeletonPtr createBallSkeleton(const std::string& name = "test_skeleton")
{
  auto skel = Skeleton::create(name);

  auto pair = skel->createJointAndBodyNodePair<BallJoint>();
  pair.first->setName("ball");
  pair.second->setName("body");

  return skel;
}

// Helper function to create a skeleton with a free joint
SkeletonPtr createFreeSkeleton(const std::string& name = "test_skeleton")
{
  auto skel = Skeleton::create(name);

  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  pair.first->setName("free");
  pair.second->setName("body");

  return skel;
}

// Helper function to create a skeleton with a translational joint
SkeletonPtr createTranslationalSkeleton(
    const std::string& name = "test_skeleton")
{
  auto skel = Skeleton::create(name);

  auto pair = skel->createJointAndBodyNodePair<TranslationalJoint>();
  pair.first->setName("translational");
  pair.second->setName("body");

  return skel;
}

//==============================================================================
// Test DOF naming
TEST(DegreeOfFreedomTests, Naming)
{
  auto skel = createRevoluteSkeleton();
  auto* dof = skel->getDof(0);

  ASSERT_NE(dof, nullptr);

  // Default name should be joint name
  EXPECT_EQ(dof->getName(), "revolute");

  // Set a custom name
  dof->setName("custom_dof");
  EXPECT_EQ(dof->getName(), "custom_dof");

  // preserveName should be true after setName
  EXPECT_TRUE(dof->isNamePreserved());

  // Reset preserve
  dof->preserveName(false);
  EXPECT_FALSE(dof->isNamePreserved());

  // Set name without preserving
  dof->setName("another_name", false);
  EXPECT_EQ(dof->getName(), "another_name");
  EXPECT_FALSE(dof->isNamePreserved());
}

//==============================================================================
// Test DOF indexing
TEST(DegreeOfFreedomTests, Indexing)
{
  auto skel = createFreeSkeleton();

  // Free joint has 6 DOFs
  ASSERT_EQ(skel->getNumDofs(), 6u);

  for (std::size_t i = 0; i < 6; ++i) {
    auto* dof = skel->getDof(i);
    ASSERT_NE(dof, nullptr);

    EXPECT_EQ(dof->getIndexInSkeleton(), i);
    EXPECT_EQ(dof->getIndexInTree(), i);
    // All 6 DOFs belong to the single FreeJoint
    EXPECT_EQ(dof->getIndexInJoint(), i);
    EXPECT_EQ(dof->getTreeIndex(), 0u);
  }
}

//==============================================================================
// Test position setting and getting
TEST(DegreeOfFreedomTests, Position)
{
  auto skel = createRevoluteSkeleton();
  auto* dof = skel->getDof(0);

  // Initial position should be 0
  EXPECT_DOUBLE_EQ(dof->getPosition(), 0.0);

  // Set position
  dof->setPosition(1.5);
  EXPECT_DOUBLE_EQ(dof->getPosition(), 1.5);

  // Reset position
  dof->resetPosition();
  EXPECT_DOUBLE_EQ(dof->getPosition(), 0.0);

  // Set initial position and reset
  dof->setInitialPosition(0.5);
  EXPECT_DOUBLE_EQ(dof->getInitialPosition(), 0.5);
  dof->setPosition(2.0);
  dof->resetPosition();
  EXPECT_DOUBLE_EQ(dof->getPosition(), 0.5);
}

//==============================================================================
// Test position limits
TEST(DegreeOfFreedomTests, PositionLimits)
{
  auto skel = createRevoluteSkeleton();
  auto* dof = skel->getDof(0);

  // Set lower limit
  dof->setPositionLowerLimit(-1.0);
  EXPECT_DOUBLE_EQ(dof->getPositionLowerLimit(), -1.0);

  // Set upper limit
  dof->setPositionUpperLimit(2.0);
  EXPECT_DOUBLE_EQ(dof->getPositionUpperLimit(), 2.0);

  // Set both limits at once
  dof->setPositionLimits(-0.5, 0.5);
  auto limits = dof->getPositionLimits();
  EXPECT_DOUBLE_EQ(limits.first, -0.5);
  EXPECT_DOUBLE_EQ(limits.second, 0.5);

  // Set limits with pair
  dof->setPositionLimits(std::make_pair(-2.0, 2.0));
  limits = dof->getPositionLimits();
  EXPECT_DOUBLE_EQ(limits.first, -2.0);
  EXPECT_DOUBLE_EQ(limits.second, 2.0);

  // Check hasPositionLimit
  EXPECT_TRUE(dof->hasPositionLimit());
}

//==============================================================================
// Test velocity setting and getting
TEST(DegreeOfFreedomTests, Velocity)
{
  auto skel = createRevoluteSkeleton();
  auto* dof = skel->getDof(0);

  // Initial velocity should be 0
  EXPECT_DOUBLE_EQ(dof->getVelocity(), 0.0);

  // Set velocity
  dof->setVelocity(2.5);
  EXPECT_DOUBLE_EQ(dof->getVelocity(), 2.5);

  // Reset velocity
  dof->resetVelocity();
  EXPECT_DOUBLE_EQ(dof->getVelocity(), 0.0);

  // Set initial velocity and reset
  dof->setInitialVelocity(1.0);
  EXPECT_DOUBLE_EQ(dof->getInitialVelocity(), 1.0);
  dof->setVelocity(5.0);
  dof->resetVelocity();
  EXPECT_DOUBLE_EQ(dof->getVelocity(), 1.0);
}

//==============================================================================
// Test velocity limits
TEST(DegreeOfFreedomTests, VelocityLimits)
{
  auto skel = createRevoluteSkeleton();
  auto* dof = skel->getDof(0);

  // Set lower limit
  dof->setVelocityLowerLimit(-5.0);
  EXPECT_DOUBLE_EQ(dof->getVelocityLowerLimit(), -5.0);

  // Set upper limit
  dof->setVelocityUpperLimit(5.0);
  EXPECT_DOUBLE_EQ(dof->getVelocityUpperLimit(), 5.0);

  // Set both limits at once
  dof->setVelocityLimits(-10.0, 10.0);
  auto limits = dof->getVelocityLimits();
  EXPECT_DOUBLE_EQ(limits.first, -10.0);
  EXPECT_DOUBLE_EQ(limits.second, 10.0);

  // Set limits with pair
  dof->setVelocityLimits(std::make_pair(-3.0, 3.0));
  limits = dof->getVelocityLimits();
  EXPECT_DOUBLE_EQ(limits.first, -3.0);
  EXPECT_DOUBLE_EQ(limits.second, 3.0);
}

//==============================================================================
// Test acceleration setting and getting
TEST(DegreeOfFreedomTests, Acceleration)
{
  auto skel = createRevoluteSkeleton();
  auto* dof = skel->getDof(0);

  // Initial acceleration should be 0
  EXPECT_DOUBLE_EQ(dof->getAcceleration(), 0.0);

  // Set acceleration
  dof->setAcceleration(3.5);
  EXPECT_DOUBLE_EQ(dof->getAcceleration(), 3.5);

  // Reset acceleration
  dof->resetAcceleration();
  EXPECT_DOUBLE_EQ(dof->getAcceleration(), 0.0);
}

//==============================================================================
// Test acceleration limits
TEST(DegreeOfFreedomTests, AccelerationLimits)
{
  auto skel = createRevoluteSkeleton();
  auto* dof = skel->getDof(0);

  // Set lower limit
  dof->setAccelerationLowerLimit(-100.0);
  EXPECT_DOUBLE_EQ(dof->getAccelerationLowerLimit(), -100.0);

  // Set upper limit
  dof->setAccelerationUpperLimit(100.0);
  EXPECT_DOUBLE_EQ(dof->getAccelerationUpperLimit(), 100.0);

  // Set both limits at once
  dof->setAccelerationLimits(-50.0, 50.0);
  auto limits = dof->getAccelerationLimits();
  EXPECT_DOUBLE_EQ(limits.first, -50.0);
  EXPECT_DOUBLE_EQ(limits.second, 50.0);

  // Set limits with pair
  dof->setAccelerationLimits(std::make_pair(-25.0, 25.0));
  limits = dof->getAccelerationLimits();
  EXPECT_DOUBLE_EQ(limits.first, -25.0);
  EXPECT_DOUBLE_EQ(limits.second, 25.0);
}

//==============================================================================
// Test force (torque) setting and getting
TEST(DegreeOfFreedomTests, Force)
{
  auto skel = createRevoluteSkeleton();
  auto* dof = skel->getDof(0);

  // Initial force should be 0
  EXPECT_DOUBLE_EQ(dof->getForce(), 0.0);

  // Set force
  dof->setForce(10.0);
  EXPECT_DOUBLE_EQ(dof->getForce(), 10.0);

  // Reset force
  dof->resetForce();
  EXPECT_DOUBLE_EQ(dof->getForce(), 0.0);
}

//==============================================================================
// Test force limits
TEST(DegreeOfFreedomTests, ForceLimits)
{
  auto skel = createRevoluteSkeleton();
  auto* dof = skel->getDof(0);

  // Set lower limit
  dof->setForceLowerLimit(-50.0);
  EXPECT_DOUBLE_EQ(dof->getForceLowerLimit(), -50.0);

  // Set upper limit
  dof->setForceUpperLimit(50.0);
  EXPECT_DOUBLE_EQ(dof->getForceUpperLimit(), 50.0);

  // Set both limits at once
  dof->setForceLimits(-30.0, 30.0);
  auto limits = dof->getForceLimits();
  EXPECT_DOUBLE_EQ(limits.first, -30.0);
  EXPECT_DOUBLE_EQ(limits.second, 30.0);

  // Set limits with pair
  dof->setForceLimits(std::make_pair(-20.0, 20.0));
  limits = dof->getForceLimits();
  EXPECT_DOUBLE_EQ(limits.first, -20.0);
  EXPECT_DOUBLE_EQ(limits.second, 20.0);
}

//==============================================================================
// Test command setting and getting
TEST(DegreeOfFreedomTests, Command)
{
  auto skel = createRevoluteSkeleton();
  auto* dof = skel->getDof(0);

  // Initial command should be 0
  EXPECT_DOUBLE_EQ(dof->getCommand(), 0.0);

  // Set command
  dof->setCommand(5.0);
  EXPECT_DOUBLE_EQ(dof->getCommand(), 5.0);

  // Reset command
  dof->resetCommand();
  EXPECT_DOUBLE_EQ(dof->getCommand(), 0.0);
}

//==============================================================================
// Test velocity change
TEST(DegreeOfFreedomTests, VelocityChange)
{
  auto skel = createRevoluteSkeleton();
  auto* dof = skel->getDof(0);

  // Initial velocity change should be 0
  EXPECT_DOUBLE_EQ(dof->getVelocityChange(), 0.0);

  // Set velocity change
  dof->setVelocityChange(0.5);
  EXPECT_DOUBLE_EQ(dof->getVelocityChange(), 0.5);

  // Reset velocity change
  dof->resetVelocityChange();
  EXPECT_DOUBLE_EQ(dof->getVelocityChange(), 0.0);
}

//==============================================================================
// Test constraint impulse
TEST(DegreeOfFreedomTests, ConstraintImpulse)
{
  auto skel = createRevoluteSkeleton();
  auto* dof = skel->getDof(0);

  // Initial constraint impulse should be 0
  EXPECT_DOUBLE_EQ(dof->getConstraintImpulse(), 0.0);

  // Set constraint impulse
  dof->setConstraintImpulse(1.5);
  EXPECT_DOUBLE_EQ(dof->getConstraintImpulse(), 1.5);

  // Reset constraint impulse
  dof->resetConstraintImpulse();
  EXPECT_DOUBLE_EQ(dof->getConstraintImpulse(), 0.0);
}

//==============================================================================
// Test spring stiffness
TEST(DegreeOfFreedomTests, SpringStiffness)
{
  auto skel = createRevoluteSkeleton();
  auto* dof = skel->getDof(0);

  // Default spring stiffness should be 0
  EXPECT_DOUBLE_EQ(dof->getSpringStiffness(), 0.0);

  // Set spring stiffness
  dof->setSpringStiffness(100.0);
  EXPECT_DOUBLE_EQ(dof->getSpringStiffness(), 100.0);
}

//==============================================================================
// Test rest position
TEST(DegreeOfFreedomTests, RestPosition)
{
  auto skel = createRevoluteSkeleton();
  auto* dof = skel->getDof(0);

  // Default rest position should be 0
  EXPECT_DOUBLE_EQ(dof->getRestPosition(), 0.0);

  // Set rest position
  dof->setRestPosition(0.5);
  EXPECT_DOUBLE_EQ(dof->getRestPosition(), 0.5);
}

//==============================================================================
// Test damping coefficient
TEST(DegreeOfFreedomTests, DampingCoefficient)
{
  auto skel = createRevoluteSkeleton();
  auto* dof = skel->getDof(0);

  // Default damping coefficient should be 0
  EXPECT_DOUBLE_EQ(dof->getDampingCoefficient(), 0.0);

  // Set damping coefficient
  dof->setDampingCoefficient(10.0);
  EXPECT_DOUBLE_EQ(dof->getDampingCoefficient(), 10.0);
}

//==============================================================================
// Test Coulomb friction
TEST(DegreeOfFreedomTests, CoulombFriction)
{
  auto skel = createRevoluteSkeleton();
  auto* dof = skel->getDof(0);

  // Default Coulomb friction should be 0
  EXPECT_DOUBLE_EQ(dof->getCoulombFriction(), 0.0);

  // Set Coulomb friction
  dof->setCoulombFriction(0.5);
  EXPECT_DOUBLE_EQ(dof->getCoulombFriction(), 0.5);
}

//==============================================================================
// Test relationships (Joint, Skeleton, BodyNode)
TEST(DegreeOfFreedomTests, Relationships)
{
  auto skel = createRevoluteSkeleton("my_skeleton");
  auto* dof = skel->getDof(0);

  // Get Joint
  auto* joint = dof->getJoint();
  ASSERT_NE(joint, nullptr);
  EXPECT_EQ(joint->getName(), "revolute");

  // Get Skeleton
  auto skeleton = dof->getSkeleton();
  ASSERT_NE(skeleton, nullptr);
  EXPECT_EQ(skeleton->getName(), "my_skeleton");

  // Get child BodyNode
  auto* childBody = dof->getChildBodyNode();
  ASSERT_NE(childBody, nullptr);
  EXPECT_EQ(childBody->getName(), "body");

  // Get parent BodyNode (should be nullptr for root joint)
  auto* parentBody = dof->getParentBodyNode();
  EXPECT_EQ(parentBody, nullptr);

  // Const versions
  const auto* constDof = static_cast<const DegreeOfFreedom*>(dof);
  EXPECT_NE(constDof->getJoint(), nullptr);
  EXPECT_NE(constDof->getSkeleton(), nullptr);
  EXPECT_NE(constDof->getChildBodyNode(), nullptr);
  EXPECT_EQ(constDof->getParentBodyNode(), nullptr);
}

//==============================================================================
// Test cyclic DOF
TEST(DegreeOfFreedomTests, Cyclic)
{
  // Ball joint DOFs should be cyclic
  auto ballSkel = createBallSkeleton();
  for (std::size_t i = 0; i < ballSkel->getNumDofs(); ++i) {
    auto* dof = ballSkel->getDof(i);
    // Ball joints use exponential map, which can be cyclic
    // The exact behavior depends on implementation
    (void)dof->isCyclic(); // Just ensure it doesn't crash
  }

  // Revolute joint can be cyclic or not depending on limits
  auto revSkel = createRevoluteSkeleton();
  auto* revDof = revSkel->getDof(0);

  // By default, revolute joints have no limits and are cyclic
  // Check if it has position limits
  (void)revDof->hasPositionLimit();
  (void)revDof->isCyclic(); // Just ensure it doesn't crash
}

//==============================================================================
// Test multi-DOF joint indexing (Translational joint has 3 DOFs)
TEST(DegreeOfFreedomTests, MultiDofJointIndexing)
{
  auto skel = createTranslationalSkeleton();

  // Translational joint has 3 DOFs
  ASSERT_EQ(skel->getNumDofs(), 3u);

  auto* joint = skel->getJoint(0);
  ASSERT_NE(joint, nullptr);

  for (std::size_t i = 0; i < 3; ++i) {
    auto* dof = skel->getDof(i);
    ASSERT_NE(dof, nullptr);

    // Index in skeleton matches global index
    EXPECT_EQ(dof->getIndexInSkeleton(), i);

    // Index in joint is local
    EXPECT_EQ(dof->getIndexInJoint(), i);

    // All belong to the same joint
    EXPECT_EQ(dof->getJoint(), joint);
  }
}

//==============================================================================
// Test multi-body skeleton indexing
TEST(DegreeOfFreedomTests, MultiBodySkeletonIndexing)
{
  auto skel = Skeleton::create("multi_body");

  // Create first body with revolute joint
  auto pair1 = skel->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr,
      RevoluteJoint::Properties(),
      BodyNode::AspectProperties("body1"));
  pair1.first->setName("joint1");

  // Create second body with prismatic joint
  auto pair2 = skel->createJointAndBodyNodePair<PrismaticJoint>(
      pair1.second,
      PrismaticJoint::Properties(),
      BodyNode::AspectProperties("body2"));
  pair2.first->setName("joint2");

  // Create third body with ball joint (3 DOFs)
  auto pair3 = skel->createJointAndBodyNodePair<BallJoint>(
      pair2.second,
      BallJoint::Properties(),
      BodyNode::AspectProperties("body3"));
  pair3.first->setName("joint3");

  // Total DOFs: 1 + 1 + 3 = 5
  EXPECT_EQ(skel->getNumDofs(), 5u);

  // Check indexing
  EXPECT_EQ(skel->getDof(0)->getIndexInSkeleton(), 0u);
  EXPECT_EQ(skel->getDof(0)->getIndexInJoint(), 0u);
  EXPECT_EQ(skel->getDof(0)->getJoint()->getName(), "joint1");

  EXPECT_EQ(skel->getDof(1)->getIndexInSkeleton(), 1u);
  EXPECT_EQ(skel->getDof(1)->getIndexInJoint(), 0u);
  EXPECT_EQ(skel->getDof(1)->getJoint()->getName(), "joint2");

  // Ball joint DOFs
  EXPECT_EQ(skel->getDof(2)->getIndexInSkeleton(), 2u);
  EXPECT_EQ(skel->getDof(2)->getIndexInJoint(), 0u);
  EXPECT_EQ(skel->getDof(2)->getJoint()->getName(), "joint3");

  EXPECT_EQ(skel->getDof(3)->getIndexInSkeleton(), 3u);
  EXPECT_EQ(skel->getDof(3)->getIndexInJoint(), 1u);
  EXPECT_EQ(skel->getDof(3)->getJoint()->getName(), "joint3");

  EXPECT_EQ(skel->getDof(4)->getIndexInSkeleton(), 4u);
  EXPECT_EQ(skel->getDof(4)->getIndexInJoint(), 2u);
  EXPECT_EQ(skel->getDof(4)->getJoint()->getName(), "joint3");
}

//==============================================================================
// Test DOF passive forces interaction
TEST(DegreeOfFreedomTests, PassiveForcesInteraction)
{
  auto skel = createRevoluteSkeleton();
  auto* dof = skel->getDof(0);

  // Set spring parameters
  dof->setSpringStiffness(100.0);
  dof->setRestPosition(0.5);
  EXPECT_DOUBLE_EQ(dof->getSpringStiffness(), 100.0);
  EXPECT_DOUBLE_EQ(dof->getRestPosition(), 0.5);

  // Set damping
  dof->setDampingCoefficient(5.0);
  EXPECT_DOUBLE_EQ(dof->getDampingCoefficient(), 5.0);

  // Set Coulomb friction
  dof->setCoulombFriction(0.1);
  EXPECT_DOUBLE_EQ(dof->getCoulombFriction(), 0.1);

  // Move DOF away from rest position
  dof->setPosition(1.0);
  dof->setVelocity(2.0);

  // These passive forces should be considered in dynamics computation
  // We're just testing the getters/setters here
}

//==============================================================================
// Test edge cases with extreme values
TEST(DegreeOfFreedomTests, ExtremeValues)
{
  auto skel = createRevoluteSkeleton();
  auto* dof = skel->getDof(0);

  // Set very large position
  dof->setPosition(1e10);
  EXPECT_DOUBLE_EQ(dof->getPosition(), 1e10);

  // Set very small position
  dof->setPosition(1e-10);
  EXPECT_DOUBLE_EQ(dof->getPosition(), 1e-10);

  // Set negative position
  dof->setPosition(-100.0);
  EXPECT_DOUBLE_EQ(dof->getPosition(), -100.0);

  // Set infinite limits
  dof->setPositionLowerLimit(-std::numeric_limits<double>::infinity());
  dof->setPositionUpperLimit(std::numeric_limits<double>::infinity());
  EXPECT_TRUE(std::isinf(dof->getPositionLowerLimit()));
  EXPECT_TRUE(std::isinf(dof->getPositionUpperLimit()));
}

//==============================================================================
// Test DOF with FreeJoint (6 DOFs with specific naming)
TEST(DegreeOfFreedomTests, FreeJointDofs)
{
  auto skel = createFreeSkeleton();

  EXPECT_EQ(skel->getNumDofs(), 6u);

  // Free joint DOFs: 3 rotation + 3 translation
  for (std::size_t i = 0; i < 6; ++i) {
    auto* dof = skel->getDof(i);
    ASSERT_NE(dof, nullptr);

    // Test setting all state variables
    dof->setPosition(static_cast<double>(i) * 0.1);
    dof->setVelocity(static_cast<double>(i) * 0.2);
    dof->setAcceleration(static_cast<double>(i) * 0.3);
    dof->setForce(static_cast<double>(i) * 0.4);
    dof->setCommand(static_cast<double>(i) * 0.5);

    EXPECT_DOUBLE_EQ(dof->getPosition(), static_cast<double>(i) * 0.1);
    EXPECT_DOUBLE_EQ(dof->getVelocity(), static_cast<double>(i) * 0.2);
    EXPECT_DOUBLE_EQ(dof->getAcceleration(), static_cast<double>(i) * 0.3);
    EXPECT_DOUBLE_EQ(dof->getForce(), static_cast<double>(i) * 0.4);
    EXPECT_DOUBLE_EQ(dof->getCommand(), static_cast<double>(i) * 0.5);
  }
}

//==============================================================================
// Test hasPositionLimit with infinite limits (no limits)
TEST(DegreeOfFreedomTests, HasPositionLimitWithInfiniteLimits)
{
  auto skel = createRevoluteSkeleton();
  auto* dof = skel->getDof(0);

  // Set infinite limits (no limits)
  dof->setPositionLowerLimit(-std::numeric_limits<double>::infinity());
  dof->setPositionUpperLimit(std::numeric_limits<double>::infinity());

  // With infinite limits, hasPositionLimit should return false
  EXPECT_FALSE(dof->hasPositionLimit());

  // Set finite limits
  dof->setPositionLimits(-1.0, 1.0);
  EXPECT_TRUE(dof->hasPositionLimit());

  // Set only lower limit to finite
  dof->setPositionLowerLimit(-1.0);
  dof->setPositionUpperLimit(std::numeric_limits<double>::infinity());
  EXPECT_TRUE(dof->hasPositionLimit());

  // Set only upper limit to finite
  dof->setPositionLowerLimit(-std::numeric_limits<double>::infinity());
  dof->setPositionUpperLimit(1.0);
  EXPECT_TRUE(dof->hasPositionLimit());
}

//==============================================================================
// Test isCyclic behavior
TEST(DegreeOfFreedomTests, IsCyclicBehavior)
{
  // Revolute joint with no limits should be cyclic
  auto revSkel = createRevoluteSkeleton();
  auto* revDof = revSkel->getDof(0);

  // Set infinite limits (no limits) - should be cyclic
  revDof->setPositionLowerLimit(-std::numeric_limits<double>::infinity());
  revDof->setPositionUpperLimit(std::numeric_limits<double>::infinity());
  EXPECT_TRUE(revDof->isCyclic());

  // Set finite limits - should not be cyclic
  revDof->setPositionLimits(-1.0, 1.0);
  EXPECT_FALSE(revDof->isCyclic());

  // Prismatic joint should not be cyclic (linear motion)
  auto prisSkel = createPrismaticSkeleton();
  auto* prisDof = prisSkel->getDof(0);
  EXPECT_FALSE(prisDof->isCyclic());
}

//==============================================================================
// Test DOF with parent body node (non-root joint)
TEST(DegreeOfFreedomTests, NonRootJointParentBodyNode)
{
  auto skel = Skeleton::create("chain");

  // Create first body (root)
  auto pair1 = skel->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr,
      RevoluteJoint::Properties(),
      BodyNode::AspectProperties("body1"));
  pair1.first->setName("joint1");

  // Create second body attached to first
  auto pair2 = skel->createJointAndBodyNodePair<RevoluteJoint>(
      pair1.second,
      RevoluteJoint::Properties(),
      BodyNode::AspectProperties("body2"));
  pair2.first->setName("joint2");

  // Get DOF from second joint
  auto* dof = skel->getDof(1);
  ASSERT_NE(dof, nullptr);

  // Parent body node should be body1
  auto* parentBody = dof->getParentBodyNode();
  ASSERT_NE(parentBody, nullptr);
  EXPECT_EQ(parentBody->getName(), "body1");

  // Child body node should be body2
  auto* childBody = dof->getChildBodyNode();
  ASSERT_NE(childBody, nullptr);
  EXPECT_EQ(childBody->getName(), "body2");
}

//==============================================================================
// Test individual lower/upper limit accessors
TEST(DegreeOfFreedomTests, IndividualLimitAccessors)
{
  auto skel = createRevoluteSkeleton();
  auto* dof = skel->getDof(0);

  // Test position limits individually
  dof->setPositionLowerLimit(-2.5);
  EXPECT_DOUBLE_EQ(dof->getPositionLowerLimit(), -2.5);

  dof->setPositionUpperLimit(3.5);
  EXPECT_DOUBLE_EQ(dof->getPositionUpperLimit(), 3.5);

  // Test velocity limits individually
  dof->setVelocityLowerLimit(-15.0);
  EXPECT_DOUBLE_EQ(dof->getVelocityLowerLimit(), -15.0);

  dof->setVelocityUpperLimit(15.0);
  EXPECT_DOUBLE_EQ(dof->getVelocityUpperLimit(), 15.0);

  // Test acceleration limits individually
  dof->setAccelerationLowerLimit(-200.0);
  EXPECT_DOUBLE_EQ(dof->getAccelerationLowerLimit(), -200.0);

  dof->setAccelerationUpperLimit(200.0);
  EXPECT_DOUBLE_EQ(dof->getAccelerationUpperLimit(), 200.0);

  // Test force limits individually
  dof->setForceLowerLimit(-75.0);
  EXPECT_DOUBLE_EQ(dof->getForceLowerLimit(), -75.0);

  dof->setForceUpperLimit(75.0);
  EXPECT_DOUBLE_EQ(dof->getForceUpperLimit(), 75.0);
}

//==============================================================================
// Test getTreeIndex
TEST(DegreeOfFreedomTests, TreeIndex)
{
  auto skel = createRevoluteSkeleton();
  auto* dof = skel->getDof(0);

  // Single skeleton has one tree, index should be 0
  EXPECT_EQ(dof->getTreeIndex(), 0u);
}

//==============================================================================
// Test const skeleton accessor
TEST(DegreeOfFreedomTests, ConstSkeletonAccessor)
{
  auto skel = createRevoluteSkeleton("const_test_skeleton");
  const auto* constDof = static_cast<const DegreeOfFreedom*>(skel->getDof(0));

  // Get const skeleton
  auto constSkel = constDof->getSkeleton();
  ASSERT_NE(constSkel, nullptr);
  EXPECT_EQ(constSkel->getName(), "const_test_skeleton");
}
