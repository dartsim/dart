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

#include <dart/All.hpp>

#include <gtest/gtest.h>

#include <limits>

using namespace dart;
using namespace dart::dynamics;

//==============================================================================
static SkeletonPtr createSkeletonWithRevoluteJoint()
{
  auto skel = Skeleton::create("test_skeleton");

  auto pair = skel->createJointAndBodyNodePair<RevoluteJoint>();
  auto* joint = pair.first;
  auto* body = pair.second;
  joint->setName("test_joint");
  body->setName("test_body");
  body->setMass(1.0);

  return skel;
}

//==============================================================================
TEST(JointCommand, NaNCommandIgnored)
{
  auto skel = createSkeletonWithRevoluteJoint();
  auto* joint = skel->getJoint(0);

  const double nan = std::numeric_limits<double>::quiet_NaN();

  joint->setActuatorType(Joint::ACCELERATION);
  joint->setCommand(0, 0.0);
  const double originalCommand = joint->getCommand(0);
  EXPECT_DOUBLE_EQ(originalCommand, 0.0);

  joint->setCommand(0, nan);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), originalCommand)
      << "NaN command should be ignored";

  joint->setCommand(0, std::numeric_limits<double>::signaling_NaN());
  EXPECT_DOUBLE_EQ(joint->getCommand(0), originalCommand)
      << "Signaling NaN command should be ignored";
}

//==============================================================================
TEST(JointCommand, InfinityCommandIgnored)
{
  auto skel = createSkeletonWithRevoluteJoint();
  auto* joint = skel->getJoint(0);

  const double inf = std::numeric_limits<double>::infinity();

  joint->setActuatorType(Joint::ACCELERATION);
  joint->setCommand(0, 1.5);
  const double originalCommand = joint->getCommand(0);
  EXPECT_DOUBLE_EQ(originalCommand, 1.5);

  joint->setCommand(0, inf);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), originalCommand)
      << "+Inf command should be ignored";

  joint->setCommand(0, -inf);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), originalCommand)
      << "-Inf command should be ignored";
}

//==============================================================================
TEST(JointCommand, ValidCommandAccepted)
{
  auto skel = createSkeletonWithRevoluteJoint();
  auto* joint = skel->getJoint(0);

  joint->setActuatorType(Joint::ACCELERATION);

  joint->setCommand(0, 5.0);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), 5.0);

  joint->setCommand(0, -3.0);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), -3.0);

  joint->setCommand(0, 0.0);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), 0.0);

  joint->setCommand(0, 1e-10);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), 1e-10);

  joint->setCommand(0, 1e10);
  EXPECT_DOUBLE_EQ(joint->getCommand(0), 1e10);
}

//==============================================================================
TEST(JointCommand, NaNCommandIgnoredForAllActuatorTypes)
{
  auto skel = createSkeletonWithRevoluteJoint();
  auto* joint = skel->getJoint(0);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::vector<Joint::ActuatorType> actuatorTypes
      = {Joint::FORCE, Joint::SERVO, Joint::VELOCITY, Joint::ACCELERATION};

  for (auto actuatorType : actuatorTypes) {
    joint->setActuatorType(actuatorType);
    joint->setCommand(0, 1.0);
    const double originalCommand = joint->getCommand(0);

    joint->setCommand(0, nan);
    EXPECT_DOUBLE_EQ(joint->getCommand(0), originalCommand)
        << "NaN command should be ignored for actuator type " << actuatorType;
  }
}

//==============================================================================
TEST(JointCommand, SimulationDoesNotCrashWithInfAccelerationCommand)
{
  auto skel = createSkeletonWithRevoluteJoint();
  auto world = simulation::World::create();
  world->addSkeleton(skel);

  auto* joint = skel->getJoint(0);
  const double inf = std::numeric_limits<double>::infinity();

  joint->setActuatorType(Joint::ACCELERATION);
  joint->setCommand(0, inf);

  EXPECT_NO_THROW({
    for (int i = 0; i < 100; ++i) {
      world->step();
    }
  });

  Eigen::VectorXd positions = skel->getPositions();
  for (Eigen::Index i = 0; i < positions.size(); ++i) {
    EXPECT_TRUE(std::isfinite(positions[i]))
        << "Position " << i << " should be finite after simulation";
  }

  Eigen::VectorXd velocities = skel->getVelocities();
  for (Eigen::Index i = 0; i < velocities.size(); ++i) {
    EXPECT_TRUE(std::isfinite(velocities[i]))
        << "Velocity " << i << " should be finite after simulation";
  }
}

//==============================================================================
TEST(JointCommand, GzPhysics845Scenario)
{
  auto skel = createSkeletonWithRevoluteJoint();
  auto world = simulation::World::create();
  world->addSkeleton(skel);

  auto* joint = skel->getJoint(0);
  const double inf = std::numeric_limits<double>::infinity();

  joint->setActuatorType(Joint::ACCELERATION);

  joint->setAccelerationLowerLimit(0, -inf);
  joint->setAccelerationUpperLimit(0, inf);

  joint->setCommand(0, inf);

  EXPECT_NO_THROW({
    for (int i = 0; i < 10; ++i) {
      world->step();
    }
  }) << "Simulation should not crash with Inf acceleration command "
        "(gz-physics#845)";
}
