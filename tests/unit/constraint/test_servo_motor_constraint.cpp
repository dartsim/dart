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

#include <dart/simulation/world.hpp>

#include <dart/constraint/servo_motor_constraint.hpp>

#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <gtest/gtest.h>

#include <vector>

#include <cmath>
using namespace dart;
using namespace dart::dynamics;
using namespace dart::constraint;

namespace {

SkeletonPtr createServoSkeleton()
{
  auto skel = Skeleton::create("servo_skel");
  auto pair = skel->createJointAndBodyNodePair<RevoluteJoint>();
  auto* joint = pair.first;
  joint->setAxis(Eigen::Vector3d::UnitZ());
  joint->setActuatorType(Joint::SERVO);
  return skel;
}

class ExposedServoMotorConstraint : public ServoMotorConstraint
{
public:
  using ServoMotorConstraint::applyImpulse;
  using ServoMotorConstraint::applyUnitImpulse;
  using ServoMotorConstraint::excite;
  using ServoMotorConstraint::getInformation;
  using ServoMotorConstraint::getRootSkeleton;
  using ServoMotorConstraint::getVelocityChange;
  using ServoMotorConstraint::isActive;
  using ServoMotorConstraint::ServoMotorConstraint;
  using ServoMotorConstraint::unexcite;
  using ServoMotorConstraint::update;
};

} // namespace

TEST(ServoMotorConstraint, GetType)
{
  auto skel = createServoSkeleton();
  auto* joint = skel->getJoint(0);

  ServoMotorConstraint constraint(joint);

  EXPECT_EQ(constraint.getType(), ServoMotorConstraint::getStaticType());
  EXPECT_FALSE(constraint.getType().empty());
  EXPECT_EQ(std::string(constraint.getType()), "ServoMotorConstraint");
}

TEST(ServoMotorConstraint, StaticSettings)
{
  double originalCfm = ServoMotorConstraint::getConstraintForceMixing();

  ServoMotorConstraint::setConstraintForceMixing(1e-6);
  EXPECT_DOUBLE_EQ(ServoMotorConstraint::getConstraintForceMixing(), 1e-6);

  ServoMotorConstraint::setConstraintForceMixing(1e-3);
  EXPECT_DOUBLE_EQ(ServoMotorConstraint::getConstraintForceMixing(), 1e-3);

  ServoMotorConstraint::setConstraintForceMixing(originalCfm);
}

TEST(ServoMotorConstraint, CfmMinimumWarned)
{
  double originalCfm = ServoMotorConstraint::getConstraintForceMixing();

  ServoMotorConstraint::setConstraintForceMixing(1e-12);
  EXPECT_LT(ServoMotorConstraint::getConstraintForceMixing(), 1e-9);

  ServoMotorConstraint::setConstraintForceMixing(originalCfm);
}

TEST(ServoMotorConstraint, IsActiveWithServoActuator)
{
  auto skel = createServoSkeleton();
  auto* joint = skel->getJoint(0);

  ExposedServoMotorConstraint constraint(joint);

  EXPECT_TRUE(constraint.isActive());
}

TEST(ServoMotorConstraint, IsInactiveWithForceActuator)
{
  auto skel = createServoSkeleton();
  auto* joint = skel->getJoint(0);

  joint->setActuatorType(Joint::FORCE);

  ExposedServoMotorConstraint constraint(joint);

  EXPECT_FALSE(constraint.isActive());
}

TEST(ServoMotorConstraint, IsInactiveWithVelocityActuator)
{
  auto skel = createServoSkeleton();
  auto* joint = skel->getJoint(0);

  joint->setActuatorType(Joint::VELOCITY);

  ExposedServoMotorConstraint constraint(joint);

  EXPECT_FALSE(constraint.isActive());
}

TEST(ServoMotorConstraint, IsInactiveWithPassiveActuator)
{
  auto skel = createServoSkeleton();
  auto* joint = skel->getJoint(0);

  joint->setActuatorType(Joint::PASSIVE);

  ExposedServoMotorConstraint constraint(joint);

  EXPECT_FALSE(constraint.isActive());
}

TEST(ServoMotorConstraint, IsInactiveWithAccelerationActuator)
{
  auto skel = createServoSkeleton();
  auto* joint = skel->getJoint(0);

  joint->setActuatorType(Joint::ACCELERATION);

  ExposedServoMotorConstraint constraint(joint);

  EXPECT_FALSE(constraint.isActive());
}

TEST(ServoMotorConstraint, IsInactiveWithLockedActuator)
{
  auto skel = createServoSkeleton();
  auto* joint = skel->getJoint(0);

  joint->setActuatorType(Joint::LOCKED);

  ExposedServoMotorConstraint constraint(joint);

  EXPECT_FALSE(constraint.isActive());
}

TEST(ServoMotorConstraint, GetRootSkeleton)
{
  auto skel = createServoSkeleton();
  auto* joint = skel->getJoint(0);

  ExposedServoMotorConstraint constraint(joint);

  auto rootSkel = constraint.getRootSkeleton();
  EXPECT_EQ(rootSkel, skel);
}

TEST(ServoMotorConstraint, UpdateWithZeroCommand)
{
  auto skel = createServoSkeleton();
  skel->setTimeStep(0.001);
  auto* joint = skel->getJoint(0);

  joint->setCommand(0, 0.0);
  joint->setVelocity(0, 0.0);

  ExposedServoMotorConstraint constraint(joint);
  constraint.update();

  EXPECT_EQ(constraint.getDimension(), 0u);
}

TEST(ServoMotorConstraint, UpdateWithNonZeroVelocityError)
{
  auto skel = createServoSkeleton();
  skel->setTimeStep(0.001);
  auto* joint = skel->getJoint(0);

  joint->setCommand(0, 1.0);
  joint->setVelocity(0, 0.0);

  ExposedServoMotorConstraint constraint(joint);
  constraint.update();

  EXPECT_EQ(constraint.getDimension(), 1u);
}

TEST(ServoMotorConstraint, MultiDofJoint)
{
  auto skel = Skeleton::create("multi_dof");

  RevoluteJoint::Properties props1;
  props1.mAxis = Eigen::Vector3d::UnitX();
  auto pair1 = skel->createJointAndBodyNodePair<RevoluteJoint>(nullptr, props1);
  auto* joint1 = pair1.first;
  auto* body1 = pair1.second;

  RevoluteJoint::Properties props2;
  props2.mAxis = Eigen::Vector3d::UnitY();
  auto pair2 = skel->createJointAndBodyNodePair<RevoluteJoint>(body1, props2);
  auto* joint2 = pair2.first;

  joint1->setActuatorType(Joint::SERVO);
  joint2->setActuatorType(Joint::SERVO);

  ExposedServoMotorConstraint constraint1(joint1);
  ExposedServoMotorConstraint constraint2(joint2);

  EXPECT_TRUE(constraint1.isActive());
  EXPECT_TRUE(constraint2.isActive());

  EXPECT_EQ(constraint1.getRootSkeleton(), skel);
  EXPECT_EQ(constraint2.getRootSkeleton(), skel);
}

TEST(ServoMotorConstraint, InformationAndImpulseFlow)
{
  auto skel = createServoSkeleton();
  skel->setTimeStep(0.01);
  auto* joint = skel->getJoint(0);

  joint->setForceLowerLimit(0, -5.0);
  joint->setForceUpperLimit(0, 5.0);
  joint->setCommand(0, 1.0);
  joint->setVelocity(0, 0.0);

  ExposedServoMotorConstraint constraint(joint);
  constraint.update();

  ASSERT_EQ(constraint.getDimension(), 1u);

  std::vector<double> x(1, 0.0);
  std::vector<double> lo(1, 0.0);
  std::vector<double> hi(1, 0.0);
  std::vector<double> b(1, 0.0);
  std::vector<double> w(1, 0.0);
  std::vector<int> findex(1, -1);

  ConstraintInfo info{
      x.data(),
      lo.data(),
      hi.data(),
      b.data(),
      w.data(),
      findex.data(),
      1.0 / skel->getTimeStep()};

  constraint.getInformation(&info);

  EXPECT_NEAR(b[0], 1.0, 1e-12);
  EXPECT_NEAR(lo[0], -5.0 * skel->getTimeStep(), 1e-12);
  EXPECT_NEAR(hi[0], 5.0 * skel->getTimeStep(), 1e-12);

  constraint.excite();
  EXPECT_TRUE(skel->isImpulseApplied());
  constraint.applyUnitImpulse(0);

  std::vector<double> delVel(1, 0.0);
  constraint.getVelocityChange(delVel.data(), false);
  EXPECT_TRUE(std::isfinite(delVel[0]));

  std::vector<double> delVelCfm(1, 0.0);
  constraint.getVelocityChange(delVelCfm.data(), true);
  EXPECT_NEAR(
      delVelCfm[0],
      delVel[0] * (1.0 + ServoMotorConstraint::getConstraintForceMixing()),
      1e-12);

  const double lambdaValue = 0.3;
  std::vector<double> lambda(1, lambdaValue);
  constraint.applyImpulse(lambda.data());
  EXPECT_NEAR(joint->getConstraintImpulse(0), lambdaValue, 1e-12);

  constraint.unexcite();
  EXPECT_FALSE(skel->isImpulseApplied());
}

TEST(ServoMotorConstraint, WorldStepTargetsVelocityAndRespectsForceLimits)
{
  auto world = simulation::World::create();
  ASSERT_NE(world, nullptr);
  world->setTimeStep(1e-3);

  auto skel = Skeleton::create("servo_world");
  auto pair = skel->createJointAndBodyNodePair<RevoluteJoint>();
  auto* joint = pair.first;
  joint->setAxis(Eigen::Vector3d::UnitZ());
  joint->setActuatorType(Joint::SERVO);
  joint->setForceLowerLimit(0, -2.0);
  joint->setForceUpperLimit(0, 2.0);
  joint->setCommand(0, 5.0);

  world->addSkeleton(skel);

  const double initialVelocity = joint->getVelocity(0);
  for (int i = 0; i < 20; ++i) {
    world->step();
  }

  EXPECT_GT(joint->getVelocity(0), initialVelocity);
  const double impulse = joint->getConstraintImpulse(0);
  EXPECT_TRUE(std::isfinite(impulse));
  EXPECT_LE(
      std::abs(impulse),
      joint->getForceUpperLimit(0) * world->getTimeStep() + 1e-12);
}
