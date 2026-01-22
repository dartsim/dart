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
#include "dart/constraint/joint_limit_constraint.hpp"
#include "dart/dynamics/revolute_joint.hpp"
#include "dart/dynamics/skeleton.hpp"

#include <gtest/gtest.h>

#include <memory>

using namespace dart;
using namespace dart::constraint;
using namespace dart::dynamics;

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

class ExposedJointLimitConstraint : public JointLimitConstraint
{
public:
  using JointLimitConstraint::applyImpulse;
  using JointLimitConstraint::applyUnitImpulse;
  using JointLimitConstraint::excite;
  using JointLimitConstraint::getInformation;
  using JointLimitConstraint::getVelocityChange;
  using JointLimitConstraint::isActive;
  using JointLimitConstraint::JointLimitConstraint;
  using JointLimitConstraint::unexcite;
  using JointLimitConstraint::update;
};

//==============================================================================
TEST(JointLimitConstraintTests, ActivatesForPositionAndVelocityViolations)
{
  auto skeleton = makeSingleRevoluteSkeleton();
  auto* joint = static_cast<RevoluteJoint*>(skeleton->getJoint(0));
  ExposedJointLimitConstraint constraint(joint);

  joint->setPosition(0, 0.0);
  joint->setVelocity(0, 0.0);
  constraint.update();
  EXPECT_FALSE(constraint.isActive());

  joint->setPosition(0, joint->getPositionUpperLimit(0) + 0.05);
  constraint.update();
  EXPECT_TRUE(constraint.isActive());

  joint->setPosition(0, 0.0);
  joint->setVelocity(0, joint->getVelocityUpperLimit(0) + 0.1);
  constraint.update();
  EXPECT_TRUE(constraint.isActive());

  joint->setVelocity(0, 0.0);
  constraint.update();
  EXPECT_FALSE(constraint.isActive());
}

//==============================================================================
TEST(JointLimitConstraintTests, GlobalParametersCanBeConfigured)
{
  const double originalAllowance = JointLimitConstraint::getErrorAllowance();
  const double originalErp = JointLimitConstraint::getErrorReductionParameter();
  const double originalErv
      = JointLimitConstraint::getMaxErrorReductionVelocity();
  const double originalCfm = JointLimitConstraint::getConstraintForceMixing();

  JointLimitConstraint::setErrorAllowance(1e-4);
  JointLimitConstraint::setErrorReductionParameter(0.25);
  JointLimitConstraint::setMaxErrorReductionVelocity(0.5);
  JointLimitConstraint::setConstraintForceMixing(1e-6);

  EXPECT_DOUBLE_EQ(JointLimitConstraint::getErrorAllowance(), 1e-4);
  EXPECT_DOUBLE_EQ(JointLimitConstraint::getErrorReductionParameter(), 0.25);
  EXPECT_DOUBLE_EQ(JointLimitConstraint::getMaxErrorReductionVelocity(), 0.5);
  EXPECT_DOUBLE_EQ(JointLimitConstraint::getConstraintForceMixing(), 1e-6);

  JointLimitConstraint::setErrorAllowance(originalAllowance);
  JointLimitConstraint::setErrorReductionParameter(originalErp);
  JointLimitConstraint::setMaxErrorReductionVelocity(originalErv);
  JointLimitConstraint::setConstraintForceMixing(originalCfm);
}

//==============================================================================
TEST(JointLimitConstraintTests, GetType)
{
  auto skeleton = makeSingleRevoluteSkeleton();
  auto* joint = skeleton->getJoint(0);
  JointLimitConstraint constraint(joint);

  EXPECT_EQ(constraint.getType(), JointLimitConstraint::getStaticType());
  EXPECT_FALSE(constraint.getType().empty());
  EXPECT_EQ(std::string(constraint.getType()), "JointLimitConstraint");
}

//==============================================================================
TEST(JointLimitConstraintTests, ActivatesForLowerPositionViolation)
{
  auto skeleton = makeSingleRevoluteSkeleton();
  auto* joint = static_cast<RevoluteJoint*>(skeleton->getJoint(0));
  ExposedJointLimitConstraint constraint(joint);

  joint->setPosition(0, 0.0);
  joint->setVelocity(0, 0.0);
  constraint.update();
  EXPECT_FALSE(constraint.isActive());

  joint->setPosition(0, joint->getPositionLowerLimit(0) - 0.05);
  constraint.update();
  EXPECT_TRUE(constraint.isActive());
}

//==============================================================================
TEST(JointLimitConstraintTests, ActivatesForLowerVelocityViolation)
{
  auto skeleton = makeSingleRevoluteSkeleton();
  auto* joint = static_cast<RevoluteJoint*>(skeleton->getJoint(0));
  ExposedJointLimitConstraint constraint(joint);

  joint->setPosition(0, 0.0);
  joint->setVelocity(0, 0.0);
  constraint.update();
  EXPECT_FALSE(constraint.isActive());

  joint->setVelocity(0, joint->getVelocityLowerLimit(0) - 0.1);
  constraint.update();
  EXPECT_TRUE(constraint.isActive());
}

//==============================================================================
TEST(JointLimitConstraintTests, NotActiveAtExactBoundary)
{
  auto skeleton = makeSingleRevoluteSkeleton();
  auto* joint = static_cast<RevoluteJoint*>(skeleton->getJoint(0));
  ExposedJointLimitConstraint constraint(joint);

  joint->setPosition(0, joint->getPositionUpperLimit(0));
  joint->setVelocity(0, 0.0);
  constraint.update();
  EXPECT_FALSE(constraint.isActive());

  joint->setPosition(0, joint->getPositionLowerLimit(0));
  constraint.update();
  EXPECT_FALSE(constraint.isActive());
}

//==============================================================================
TEST(JointLimitConstraintTests, DimensionMatchesViolations)
{
  auto skeleton = Skeleton::create("multi_dof");

  RevoluteJoint::Properties props1;
  props1.mAxis = Eigen::Vector3d::UnitX();
  auto pair1
      = skeleton->createJointAndBodyNodePair<RevoluteJoint>(nullptr, props1);
  auto* joint1 = pair1.first;
  auto* body1 = pair1.second;

  RevoluteJoint::Properties props2;
  props2.mAxis = Eigen::Vector3d::UnitY();
  auto pair2
      = skeleton->createJointAndBodyNodePair<RevoluteJoint>(body1, props2);
  auto* joint2 = pair2.first;

  joint1->setPositionLowerLimit(0, -0.5);
  joint1->setPositionUpperLimit(0, 0.5);
  joint2->setPositionLowerLimit(0, -0.5);
  joint2->setPositionUpperLimit(0, 0.5);

  ExposedJointLimitConstraint constraint1(joint1);
  ExposedJointLimitConstraint constraint2(joint2);

  joint1->setPosition(0, 0.0);
  joint2->setPosition(0, 0.0);
  constraint1.update();
  constraint2.update();
  EXPECT_EQ(constraint1.getDimension(), 0u);
  EXPECT_EQ(constraint2.getDimension(), 0u);

  joint1->setPosition(0, 0.6);
  constraint1.update();
  EXPECT_EQ(constraint1.getDimension(), 1u);
  EXPECT_EQ(constraint2.getDimension(), 0u);
}

//==============================================================================
TEST(JointLimitConstraintTests, GetInformation)
{
  auto skeleton = makeSingleRevoluteSkeleton();
  auto* joint = static_cast<RevoluteJoint*>(skeleton->getJoint(0));
  ExposedJointLimitConstraint constraint(joint);

  joint->setPosition(0, joint->getPositionUpperLimit(0) + 0.1);
  skeleton->computeForwardKinematics();
  constraint.update();
  ASSERT_TRUE(constraint.isActive());

  constraint::ConstraintInfo info;
  double lo[6], hi[6], b[6], w[6], x[6];
  int findex[6];
  for (int i = 0; i < 6; ++i) {
    lo[i] = hi[i] = b[i] = w[i] = x[i] = 0.0;
    findex[i] = -1;
  }
  info.lo = lo;
  info.hi = hi;
  info.b = b;
  info.w = w;
  info.x = x;
  info.findex = findex;
  info.invTimeStep = 1000.0;

  // Ensure non-zero error allowance
  JointLimitConstraint::setErrorAllowance(0.01);

  constraint.getInformation(&info);

  EXPECT_LT(info.lo[0], info.hi[0]);
  // Position violation should result in some 'b' value (desired velocity
  // change)
  EXPECT_NE(info.b[0], 0.0);
}

//==============================================================================
TEST(JointLimitConstraintTests, ApplyImpulseAndVelocityChange)
{
  auto skeleton = makeSingleRevoluteSkeleton();
  auto* joint = static_cast<RevoluteJoint*>(skeleton->getJoint(0));
  ExposedJointLimitConstraint constraint(joint);

  joint->setPosition(0, joint->getPositionUpperLimit(0) + 0.1);
  skeleton->computeForwardKinematics();
  // computeForwardDynamics() updates articulated inertia and invProjArtInertia
  skeleton->computeForwardDynamics();
  constraint.update();

  double delVel[6] = {0, 0, 0, 0, 0, 0};
  constraint.applyUnitImpulse(0);
  skeleton->computeImpulseForwardDynamics();
  constraint.getVelocityChange(delVel, true);

  double lambda[1] = {1.0};
  constraint.applyImpulse(lambda);

  constraint.excite();
  constraint.unexcite();
}
