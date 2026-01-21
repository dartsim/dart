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
  using JointLimitConstraint::isActive;
  using JointLimitConstraint::JointLimitConstraint;
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

  // Restore globals to avoid leaking configuration into other tests.
  JointLimitConstraint::setErrorAllowance(originalAllowance);
  JointLimitConstraint::setErrorReductionParameter(originalErp);
  JointLimitConstraint::setMaxErrorReductionVelocity(originalErv);
  JointLimitConstraint::setConstraintForceMixing(originalCfm);
}
