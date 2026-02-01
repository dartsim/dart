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

#include "helpers/dynamics_helpers.hpp"

#include "helpers/gtest_utils.hpp" // IWYU pragma: keep

#include "dart/constraint/constraint_solver.hpp"
#include "dart/constraint/joint_constraint.hpp"
#include "dart/constraint/joint_coulomb_friction_constraint.hpp"
#include "dart/constraint/revolute_joint_constraint.hpp"
#include "dart/dynamics/revolute_joint.hpp"
#include "dart/dynamics/skeleton.hpp"
#include "dart/dynamics/weld_joint.hpp"
#include "dart/simulation/world.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <cmath>
using namespace dart;
using namespace dart::constraint;
using namespace dart::dynamics;
using namespace dart::simulation;

namespace {

struct LcpBuffers
{
  std::vector<double> x;
  std::vector<double> lo;
  std::vector<double> hi;
  std::vector<double> b;
  std::vector<double> w;
  std::vector<int> findex;
};

std::shared_ptr<Skeleton> makeSingleRevoluteSkeleton()
{
  auto skeleton = Skeleton::create("single_dof");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  auto* joint = pair.first;
  joint->setAxis(Eigen::Vector3d::UnitZ());
  joint->setPositionLowerLimit(0, -0.3);
  joint->setPositionUpperLimit(0, 0.3);
  joint->setVelocityLowerLimit(0, -1.0);
  joint->setVelocityUpperLimit(0, 1.0);
  return skeleton;
}

void prepareConstraintInfo(
    constraint::ConstraintInfo& info,
    LcpBuffers& buffers,
    std::size_t dim,
    double invTimeStep,
    constraint::ConstraintPhase phase,
    bool useSplitImpulse)
{
  buffers.x.assign(dim, 0.0);
  buffers.lo.assign(dim, 0.0);
  buffers.hi.assign(dim, 0.0);
  buffers.b.assign(dim, 0.0);
  buffers.w.assign(dim, 0.0);
  buffers.findex.assign(dim, -1);

  info.x = buffers.x.data();
  info.lo = buffers.lo.data();
  info.hi = buffers.hi.data();
  info.b = buffers.b.data();
  info.w = buffers.w.data();
  info.findex = buffers.findex.data();
  info.invTimeStep = invTimeStep;
  info.phase = phase;
  info.useSplitImpulse = useSplitImpulse;
}

} // namespace

class ExposedJointConstraint : public JointConstraint
{
public:
  using JointConstraint::isActive;
  using JointConstraint::JointConstraint;
  using JointConstraint::update;

  dynamics::SkeletonPtr exposedGetRootSkeleton() const
  {
    return getRootSkeleton();
  }
};

class ExposedRevoluteJointConstraint final : public RevoluteJointConstraint
{
public:
  using RevoluteJointConstraint::applyImpulse;
  using RevoluteJointConstraint::applyUnitImpulse;
  using RevoluteJointConstraint::excite;
  using RevoluteJointConstraint::getInformation;
  using RevoluteJointConstraint::getVelocityChange;
  using RevoluteJointConstraint::isActive;
  using RevoluteJointConstraint::RevoluteJointConstraint;
  using RevoluteJointConstraint::unexcite;
  using RevoluteJointConstraint::update;
};

TEST(JointConstraintTests, InvalidPositionLimitsDoNotCrash)
{
  auto skeleton = makeSingleRevoluteSkeleton();
  auto* joint = static_cast<RevoluteJoint*>(skeleton->getJoint(0));

  joint->setPositionLowerLimit(0, 1.0);
  joint->setPositionUpperLimit(0, 0.0);

  ExposedJointConstraint constraint(joint);

  EXPECT_NO_THROW(constraint.update());
  EXPECT_FALSE(constraint.isActive());
}

TEST(JointConstraintTests, InvalidVelocityLimitsDoNotCrash)
{
  auto skeleton = makeSingleRevoluteSkeleton();
  auto* joint = static_cast<RevoluteJoint*>(skeleton->getJoint(0));

  joint->setVelocityLowerLimit(0, 1.0);
  joint->setVelocityUpperLimit(0, -1.0);

  ExposedJointConstraint constraint(joint);

  EXPECT_NO_THROW(constraint.update());
  EXPECT_FALSE(constraint.isActive());
}

TEST(JointConstraintTests, WorldStepWithInvalidLimitsDoesNotCrash)
{
  auto world = World::create();
  auto skeleton = makeSingleRevoluteSkeleton();
  auto* joint = static_cast<RevoluteJoint*>(skeleton->getJoint(0));

  joint->setPositionLowerLimit(0, 1.0);
  joint->setPositionUpperLimit(0, 0.0);
  joint->setLimitEnforcement(true);

  world->addSkeleton(skeleton);

  EXPECT_NO_THROW({
    for (int i = 0; i < 10; ++i) {
      world->step();
    }
  });
}

TEST(JointConstraintTests, ServoMotorConstraintActivation)
{
  auto world = World::create();
  auto skeleton = makeSingleRevoluteSkeleton();
  auto* joint = static_cast<RevoluteJoint*>(skeleton->getJoint(0));

  joint->setActuatorType(dynamics::Joint::SERVO);
  joint->setCommand(0, 0.5);
  joint->setForceLowerLimit(0, -100.0);
  joint->setForceUpperLimit(0, 100.0);

  world->addSkeleton(skeleton);

  ExposedJointConstraint constraint(joint);
  constraint.update();

  EXPECT_TRUE(constraint.isActive());
}

TEST(JointConstraintTests, GetRootSkeleton)
{
  auto skeleton = makeSingleRevoluteSkeleton();
  auto* joint = static_cast<RevoluteJoint*>(skeleton->getJoint(0));
  joint->setLimitEnforcement(true);
  skeleton->setPosition(0, -0.5);

  ExposedJointConstraint constraint(joint);
  constraint.update();

  if (constraint.isActive()) {
    auto rootSkel = constraint.exposedGetRootSkeleton();
    EXPECT_NE(rootSkel, nullptr);
  }
}

TEST(JointConstraintTests, RevoluteJointConstraintPaths)
{
  auto skelA = createBox(
      Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d(0.0, 0.1, 0.0));
  auto skelB = createBox(
      Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d(0.0, 0.3, 0.0));
  auto* bodyA = skelA->getBodyNode(0);
  auto* bodyB = skelB->getBodyNode(0);
  ASSERT_NE(bodyA, nullptr);
  ASSERT_NE(bodyB, nullptr);

  const Eigen::Vector3d jointPos(0.0, 0.2, 0.0);
  const Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  ExposedRevoluteJointConstraint constraint(bodyA, bodyB, jointPos, axis, axis);
  constraint.update();
  EXPECT_EQ(constraint.getDimension(), 5u);

  constraint::ConstraintInfo info{};
  LcpBuffers buffers;
  prepareConstraintInfo(
      info,
      buffers,
      constraint.getDimension(),
      1000.0,
      constraint::ConstraintPhase::Velocity,
      false);
  constraint.getInformation(&info);

  constraint.applyUnitImpulse(0);
  skelA->computeForwardDynamics();
  skelB->computeForwardDynamics();
  std::vector<double> vel(constraint.getDimension(), 0.0);
  constraint.getVelocityChange(vel.data(), true);

  std::vector<double> lambda(constraint.getDimension(), 0.1);
  constraint.applyImpulse(lambda.data());
  constraint.excite();
  constraint.unexcite();
}

TEST(JointConstraintTests, JointConstraintParameterClampsAndActivation)
{
  auto skeleton = dynamics::Skeleton::create("joint_constraint");
  auto pair = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>();
  auto* joint = pair.first;
  ASSERT_NE(joint, nullptr);
  joint->setAxis(Eigen::Vector3d::UnitZ());
  joint->setPositionLowerLimit(0, -0.1);
  joint->setPositionUpperLimit(0, 0.1);
  joint->setVelocityLowerLimit(0, -1.0);
  joint->setVelocityUpperLimit(0, 1.0);
  joint->setPosition(0, 0.2);

  ExposedJointConstraint constraint(joint);
  constraint.update();

  const double prevAllowance = constraint::JointConstraint::getErrorAllowance();
  const double prevErp
      = constraint::JointConstraint::getErrorReductionParameter();
  const double prevErv
      = constraint::JointConstraint::getMaxErrorReductionVelocity();
  const double prevCfm
      = constraint::JointConstraint::getConstraintForceMixing();

  constraint::JointConstraint::setErrorAllowance(-0.5);
  EXPECT_DOUBLE_EQ(constraint::JointConstraint::getErrorAllowance(), -0.5);
  constraint::JointConstraint::setErrorReductionParameter(1.5);
  EXPECT_DOUBLE_EQ(
      constraint::JointConstraint::getErrorReductionParameter(), 1.5);
  constraint::JointConstraint::setMaxErrorReductionVelocity(-1.0);
  EXPECT_DOUBLE_EQ(
      constraint::JointConstraint::getMaxErrorReductionVelocity(), -1.0);
  constraint::JointConstraint::setConstraintForceMixing(1e-12);
  EXPECT_DOUBLE_EQ(
      constraint::JointConstraint::getConstraintForceMixing(), 1e-12);

  constraint::JointConstraint::setErrorAllowance(prevAllowance);
  constraint::JointConstraint::setErrorReductionParameter(prevErp);
  constraint::JointConstraint::setMaxErrorReductionVelocity(prevErv);
  constraint::JointConstraint::setConstraintForceMixing(prevCfm);
}

TEST(JointConstraintTests, RevoluteJointConstraintSingleBodyInformation)
{
  auto skel = createBox(
      Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d(0.0, 0.1, 0.0));
  auto* body = skel->getBodyNode(0);
  ASSERT_NE(body, nullptr);

  const Eigen::Vector3d jointPos = body->getTransform().translation();
  const Eigen::Vector3d axis(1.0, 0.0, 0.0);
  ExposedRevoluteJointConstraint constraint(body, jointPos, axis);
  constraint.update();

  constraint::ConstraintInfo info{};
  LcpBuffers buffers;
  prepareConstraintInfo(
      info,
      buffers,
      constraint.getDimension(),
      1000.0,
      constraint::ConstraintPhase::Velocity,
      false);
  constraint.getInformation(&info);

  for (double value : buffers.b) {
    EXPECT_FALSE(value != value);
  }

  auto kinematicSkel = createBox(
      Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d(0.0, 0.2, 0.0));
  auto* kinematicJoint = kinematicSkel->getJoint(0);
  ASSERT_NE(kinematicJoint, nullptr);
  kinematicJoint->setActuatorType(dynamics::Joint::VELOCITY);
  ExposedRevoluteJointConstraint inactiveConstraint(
      kinematicSkel->getBodyNode(0),
      kinematicSkel->getBodyNode(0)->getTransform().translation(),
      Eigen::Vector3d::UnitZ());
  EXPECT_FALSE(inactiveConstraint.isActive());
}

TEST(JointConstraintTests, RevoluteJointConstraintSingleBodyImpulseVelocity)
{
  auto skel = createBox(
      Eigen::Vector3d(0.3, 0.3, 0.3), Eigen::Vector3d(0.0, 0.2, 0.0));
  auto* body = skel->getBodyNode(0);
  ASSERT_NE(body, nullptr);

  const Eigen::Vector3d jointPos = body->getTransform().translation();
  ExposedRevoluteJointConstraint constraint(
      body, jointPos, Eigen::Vector3d::UnitY());
  constraint.update();

  constraint.applyUnitImpulse(0);
  skel->computeForwardDynamics();
  constraint.excite();

  std::vector<double> vel(constraint.getDimension(), 0.0);
  constraint.getVelocityChange(vel.data(), false);

  bool anyNonZero = false;
  for (double value : vel) {
    if (value != 0.0) {
      anyNonZero = true;
      break;
    }
  }
  EXPECT_TRUE(anyNonZero);
}

TEST(JointConstraintTests, RevoluteJointConstraintSingleBodyExciteUnexcite)
{
  auto skel = createBox(
      Eigen::Vector3d(0.1, 0.1, 0.1), Eigen::Vector3d(0.0, 0.3, 0.0));
  auto* body = skel->getBodyNode(0);
  ASSERT_NE(body, nullptr);

  ExposedRevoluteJointConstraint constraint(
      body, body->getTransform().translation(), Eigen::Vector3d::UnitZ());

  EXPECT_FALSE(skel->isImpulseApplied());
  constraint.excite();
  EXPECT_TRUE(skel->isImpulseApplied());
  constraint.unexcite();
  EXPECT_FALSE(skel->isImpulseApplied());
}

TEST(JointConstraintTests, RevoluteJointConstraintSameSkeletonBothReactive)
{
  auto skel = Skeleton::create("revolute_chain");

  auto pair1 = skel->createJointAndBodyNodePair<RevoluteJoint>();
  pair1.first->setAxis(Eigen::Vector3d::UnitZ());
  pair1.second->setMass(1.0);

  auto pair2 = skel->createJointAndBodyNodePair<RevoluteJoint>(pair1.second);
  pair2.first->setAxis(Eigen::Vector3d::UnitZ());
  pair2.second->setMass(1.0);

  ExposedRevoluteJointConstraint constraint(
      pair1.second,
      pair2.second,
      pair1.second->getTransform().translation(),
      Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d::UnitZ());
  constraint.update();

  constraint.applyUnitImpulse(0);
  skel->computeForwardDynamics();
  constraint.excite();

  std::vector<double> vel(constraint.getDimension(), 0.0);
  constraint.getVelocityChange(vel.data(), false);
  for (double value : vel) {
    EXPECT_FALSE(value != value);
  }
}

TEST(JointConstraintTests, RevoluteJointConstraintSameSkeletonBody2NonReactive)
{
  auto skel = Skeleton::create("weld_child");

  auto pair1 = skel->createJointAndBodyNodePair<RevoluteJoint>();
  pair1.first->setAxis(Eigen::Vector3d::UnitZ());
  pair1.second->setMass(1.0);

  auto pair2 = skel->createJointAndBodyNodePair<WeldJoint>(pair1.second);
  pair2.second->setMass(1.0);

  ExposedRevoluteJointConstraint constraint(
      pair1.second,
      pair2.second,
      pair1.second->getTransform().translation(),
      Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d::UnitZ());
  constraint.update();

  EXPECT_TRUE(constraint.isActive());
  constraint.applyUnitImpulse(1);
  skel->computeForwardDynamics();
}

TEST(JointConstraintTests, RevoluteJointConstraintSameSkeletonBody1NonReactive)
{
  auto skel = Skeleton::create("weld_root");

  auto pair1 = skel->createJointAndBodyNodePair<WeldJoint>();
  pair1.second->setMass(1.0);

  auto pair2 = skel->createJointAndBodyNodePair<RevoluteJoint>(pair1.second);
  pair2.first->setAxis(Eigen::Vector3d::UnitZ());
  pair2.second->setMass(1.0);

  ExposedRevoluteJointConstraint constraint(
      pair1.second,
      pair2.second,
      pair2.second->getTransform().translation(),
      Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d::UnitZ());
  constraint.update();

  EXPECT_TRUE(constraint.isActive());
  constraint.applyUnitImpulse(2);
  skel->computeForwardDynamics();
}

TEST(JointConstraintTests, RevoluteJointConstraintDifferentSkeletonsVelocity)
{
  auto skelA = createBox(
      Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d(0.0, 0.1, 0.0));
  auto skelB = createBox(
      Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d(0.0, 0.4, 0.0));
  auto* bodyA = skelA->getBodyNode(0);
  auto* bodyB = skelB->getBodyNode(0);
  ASSERT_NE(bodyA, nullptr);
  ASSERT_NE(bodyB, nullptr);

  ExposedRevoluteJointConstraint constraint(
      bodyA,
      bodyB,
      Eigen::Vector3d(0.0, 0.2, 0.0),
      Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d::UnitZ());
  constraint.update();

  constraint.applyUnitImpulse(0);
  skelA->computeForwardDynamics();
  skelB->computeForwardDynamics();
  constraint.excite();

  std::vector<double> vel(constraint.getDimension(), 0.0);
  constraint.getVelocityChange(vel.data(), true);
  bool anyNonZero = false;
  for (double value : vel) {
    if (value != 0.0) {
      anyNonZero = true;
      break;
    }
  }
  EXPECT_TRUE(anyNonZero);
}

TEST(
    JointConstraintTests,
    RevoluteJointConstraintDifferentSkeletonsBody2NonReactive)
{
  auto skelA = createBox(
      Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d(0.0, 0.1, 0.0));
  auto skelB = createBox(
      Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d(0.0, 0.5, 0.0));
  auto* bodyA = skelA->getBodyNode(0);
  auto* bodyB = skelB->getBodyNode(0);
  ASSERT_NE(bodyA, nullptr);
  ASSERT_NE(bodyB, nullptr);

  auto* jointB = skelB->getJoint(0);
  ASSERT_NE(jointB, nullptr);
  jointB->setActuatorType(dynamics::Joint::VELOCITY);

  ExposedRevoluteJointConstraint constraint(
      bodyA,
      bodyB,
      Eigen::Vector3d(0.0, 0.3, 0.0),
      Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d::UnitZ());
  constraint.update();

  EXPECT_TRUE(constraint.isActive());
  constraint.applyUnitImpulse(0);
  skelA->computeForwardDynamics();
}

TEST(JointConstraintTests, RevoluteJointConstraintApplyImpulseTwoBodies)
{
  auto skelA = createBox(
      Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d(0.0, 0.1, 0.0));
  auto skelB = createBox(
      Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d(0.0, 0.4, 0.0));
  auto* bodyA = skelA->getBodyNode(0);
  auto* bodyB = skelB->getBodyNode(0);
  ASSERT_NE(bodyA, nullptr);
  ASSERT_NE(bodyB, nullptr);

  ExposedRevoluteJointConstraint constraint(
      bodyA,
      bodyB,
      Eigen::Vector3d(0.0, 0.2, 0.0),
      Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d::UnitZ());
  constraint.update();

  skelA->clearConstraintImpulses();
  skelB->clearConstraintImpulses();

  std::vector<double> lambda(constraint.getDimension(), 0.2);
  constraint.applyImpulse(lambda.data());

  const auto& impulseA = bodyA->getConstraintImpulse();
  const auto& impulseB = bodyB->getConstraintImpulse();
  bool anyNonZeroA = false;
  bool anyNonZeroB = false;
  for (int i = 0; i < 6; ++i) {
    if (impulseA[i] != 0.0) {
      anyNonZeroA = true;
    }
    if (impulseB[i] != 0.0) {
      anyNonZeroB = true;
    }
  }
  EXPECT_TRUE(anyNonZeroA);
  EXPECT_TRUE(anyNonZeroB);
}

TEST(JointCoulombFrictionConstraint, WorldStepAppliesCoulombFriction)
{
  auto world = World::create();
  ASSERT_NE(world, nullptr);
  world->setTimeStep(1e-3);

  auto skel = Skeleton::create("coulomb_friction");
  auto pair = skel->createJointAndBodyNodePair<RevoluteJoint>();
  auto* joint = pair.first;
  auto* body = pair.second;
  joint->setAxis(Eigen::Vector3d::UnitZ());
  joint->setActuatorType(Joint::FORCE);
  joint->setCoulombFriction(0, 0.8);
  joint->setVelocity(0, 1.0);
  body->addExtTorque(Eigen::Vector3d(0.0, 0.0, 0.5), true);

  world->addSkeleton(skel);

  EXPECT_VECTOR_DOUBLE_EQ(
      world->getGravity(), Eigen::Vector3d(0.0, 0.0, -9.81));

  const double initialVelocity = joint->getVelocity(0);
  for (int i = 0; i < 20; ++i) {
    world->step();
  }

  EXPECT_LT(std::abs(joint->getVelocity(0)), std::abs(initialVelocity));
  const double impulse = joint->getConstraintImpulse(0);
  EXPECT_TRUE(std::isfinite(impulse));
  EXPECT_LE(
      std::abs(impulse),
      joint->getCoulombFriction(0) * world->getTimeStep() + 1e-12);
}

TEST(JointConstraintTests, RevoluteJointConstraintWorldStepConverges)
{
  auto world = World::create();
  world->setGravity(Eigen::Vector3d::Zero());

  auto skelA = createBox(
      Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d(0.0, 0.1, 0.0));
  auto skelB = createBox(
      Eigen::Vector3d(0.2, 0.2, 0.2), Eigen::Vector3d(0.0, 0.5, 0.0));
  world->addSkeleton(skelA);
  world->addSkeleton(skelB);

  auto* bodyA = skelA->getBodyNode(0);
  auto* bodyB = skelB->getBodyNode(0);
  ASSERT_NE(bodyA, nullptr);
  ASSERT_NE(bodyB, nullptr);

  const Eigen::Vector3d jointPos(0.0, 0.3, 0.0);
  const Eigen::Vector3d axis1 = Eigen::Vector3d::Zero();
  const Eigen::Vector3d axis2 = Eigen::Vector3d::Zero();
  auto constraint = std::make_shared<RevoluteJointConstraint>(
      bodyA, bodyB, jointPos, axis1, axis2);
  world->getConstraintSolver()->addConstraint(constraint);

  const Eigen::Vector3d offsetA = bodyA->getTransform().inverse() * jointPos;
  const Eigen::Vector3d offsetB = bodyB->getTransform().inverse() * jointPos;

  skelB->setPosition(4, skelB->getPosition(4) + 0.25);

  const Eigen::Vector3d anchorA = bodyA->getTransform() * offsetA;
  const Eigen::Vector3d anchorB = bodyB->getTransform() * offsetB;
  const double initialError = (anchorA - anchorB).norm();
  EXPECT_GT(initialError, 0.0);

  for (int i = 0; i < 200; ++i) {
    world->step();
  }

  const Eigen::Vector3d finalAnchorA = bodyA->getTransform() * offsetA;
  const Eigen::Vector3d finalAnchorB = bodyB->getTransform() * offsetB;
  const double finalError = (finalAnchorA - finalAnchorB).norm();
  EXPECT_LT(finalError, initialError);
}

TEST(JointConstraintTests, JointConstraintWorldStepEnforcesLimits)
{
  auto world = World::create();

  auto skeleton = Skeleton::create("limit_skel");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  auto* joint = pair.first;
  ASSERT_NE(joint, nullptr);
  joint->setAxis(Eigen::Vector3d::UnitZ());
  joint->setPositionLowerLimit(0, -0.1);
  joint->setPositionUpperLimit(0, 0.1);
  joint->setVelocityLowerLimit(0, -0.5);
  joint->setVelocityUpperLimit(0, 0.5);
  joint->setLimitEnforcement(true);

  skeleton->setPosition(0, 0.5);
  world->addSkeleton(skeleton);

  auto constraint = std::make_shared<JointConstraint>(joint);
  world->getConstraintSolver()->addConstraint(constraint);

  const double initialPosition = joint->getPosition(0);
  for (int i = 0; i < 200; ++i) {
    world->step();
  }

  const double finalPosition = joint->getPosition(0);
  EXPECT_LT(finalPosition, initialPosition);
  EXPECT_LT(std::abs(finalPosition), 0.3);
}
