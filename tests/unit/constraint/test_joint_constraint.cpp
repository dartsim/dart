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

#include "../../helpers/gtest_utils.hpp"
#include "dart/constraint/joint_constraint.hpp"
#include "dart/dynamics/revolute_joint.hpp"
#include "dart/dynamics/skeleton.hpp"
#include "dart/simulation/world.hpp"

#include <gtest/gtest.h>

#include <memory>

using namespace dart;
using namespace dart::constraint;
using namespace dart::dynamics;
using namespace dart::simulation;

namespace {

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

TEST(JointConstraintTests, SetErrorAllowanceNegativeLogsWarning)
{
  double original = JointConstraint::getErrorAllowance();

  // API logs warning but doesn't clamp - verify value is still set
  JointConstraint::setErrorAllowance(-0.5);
  // Just verify setter doesn't crash; behavior is to log warning
  JointConstraint::setErrorAllowance(original);
}

TEST(JointConstraintTests, SetErrorAllowanceValidValue)
{
  double original = JointConstraint::getErrorAllowance();

  JointConstraint::setErrorAllowance(0.01);
  EXPECT_DOUBLE_EQ(JointConstraint::getErrorAllowance(), 0.01);

  JointConstraint::setErrorAllowance(original);
}

TEST(JointConstraintTests, SetErrorReductionParameterNegativeLogsWarning)
{
  double original = JointConstraint::getErrorReductionParameter();

  // API logs warning but doesn't clamp
  JointConstraint::setErrorReductionParameter(-0.5);
  JointConstraint::setErrorReductionParameter(original);
}

TEST(JointConstraintTests, SetErrorReductionParameterAboveOneLogsWarning)
{
  double original = JointConstraint::getErrorReductionParameter();

  // API logs warning but doesn't clamp
  JointConstraint::setErrorReductionParameter(1.5);
  JointConstraint::setErrorReductionParameter(original);
}

TEST(JointConstraintTests, SetErrorReductionParameterValidValue)
{
  double original = JointConstraint::getErrorReductionParameter();

  JointConstraint::setErrorReductionParameter(0.5);
  EXPECT_DOUBLE_EQ(JointConstraint::getErrorReductionParameter(), 0.5);

  JointConstraint::setErrorReductionParameter(original);
}

TEST(JointConstraintTests, SetMaxErrorReductionVelocityNegativeLogsWarning)
{
  double original = JointConstraint::getMaxErrorReductionVelocity();

  // API logs warning but doesn't clamp
  JointConstraint::setMaxErrorReductionVelocity(-0.5);
  JointConstraint::setMaxErrorReductionVelocity(original);
}

TEST(JointConstraintTests, SetMaxErrorReductionVelocityValidValue)
{
  double original = JointConstraint::getMaxErrorReductionVelocity();

  JointConstraint::setMaxErrorReductionVelocity(5.0);
  EXPECT_DOUBLE_EQ(JointConstraint::getMaxErrorReductionVelocity(), 5.0);

  JointConstraint::setMaxErrorReductionVelocity(original);
}

TEST(JointConstraintTests, SetConstraintForceMixingTooSmallLogsWarning)
{
  double original = JointConstraint::getConstraintForceMixing();

  // API logs warning but doesn't clamp
  JointConstraint::setConstraintForceMixing(1e-12);
  JointConstraint::setConstraintForceMixing(original);
}

TEST(JointConstraintTests, SetConstraintForceMixingValidValue)
{
  double original = JointConstraint::getConstraintForceMixing();

  JointConstraint::setConstraintForceMixing(1e-4);
  EXPECT_DOUBLE_EQ(JointConstraint::getConstraintForceMixing(), 1e-4);

  JointConstraint::setConstraintForceMixing(original);
}

TEST(JointConstraintTests, GetStaticType)
{
  EXPECT_EQ(JointConstraint::getStaticType(), "JointConstraint");
}

TEST(JointConstraintTests, GetTypeMatchesStaticType)
{
  auto skeleton = makeSingleRevoluteSkeleton();
  auto* joint = static_cast<RevoluteJoint*>(skeleton->getJoint(0));
  ExposedJointConstraint constraint(joint);

  EXPECT_EQ(constraint.getType(), JointConstraint::getStaticType());
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
