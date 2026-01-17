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

#include <dart/utils/sdf/sdf.hpp>

#include <dart/dart.hpp>

#include <TestHelpers.hpp>
#include <gtest/gtest.h>

//========================================================================================
TEST(Issue1583, ServoJointWithPositionLimits)
{
  const double pos_lb = -0.1;
  const double pos_ub = +0.1;
  const double vel_desired = 1;

  auto skel = dart::utils::SdfParser::readSkeleton(
      "dart://sample/sdf/test/test_issue1583.model");
  ASSERT_NE(skel, nullptr);

  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  world->addSkeleton(skel);
  ASSERT_EQ(world->getNumSkeletons(), 1);

  auto* joint = skel->getJoint("j1");
  joint->setPositionLowerLimit(0, pos_lb);
  joint->setPositionUpperLimit(0, pos_ub);
  joint->setLimitEnforcement(true);
  joint->setActuatorType(dart::dynamics::Joint::SERVO);

  EXPECT_DOUBLE_EQ(0, joint->getPosition(0));
  EXPECT_DOUBLE_EQ(0, joint->getVelocity(0));
  EXPECT_DOUBLE_EQ(0, joint->getAcceleration(0));

  for (std::size_t i = 0; i < 1000; ++i) {
    joint->setCommand(0, vel_desired);
    world->step();

    const double pos = joint->getPosition(0);
    const double vel = joint->getVelocity(0);

    EXPECT_LE(pos, pos_ub + 1e-6);
    EXPECT_GE(pos, pos_lb - 1e-6);

    if (std::abs(vel - vel_desired) > 1e-6) {
      EXPECT_NEAR(pos, pos_ub, 1e-2);
    }
  }

  EXPECT_NEAR(pos_ub, joint->getPosition(0), 1e-2);
  EXPECT_NEAR(0, joint->getVelocity(0), 1e-6);
}

//========================================================================================
TEST(Issue1683, ServoJointRecoversFromPositionLimits)
{
  const double posLowerBound = -0.75;
  const double posUpperBound = +0.85;
  const double velDesiredToUb = 1.0;
  const double velDesiredToLb = -1.0;
  const double velRecoverFromUb = -0.05;
  const double velRecoverFromLb = 0.05;

  auto skel = dart::utils::SdfParser::readSkeleton(
      "dart://sample/sdf/test/test_issue1683.model");
  ASSERT_NE(skel, nullptr);

  auto world = dart::simulation::World::create();
  world->addSkeleton(skel);
  ASSERT_EQ(world->getNumSkeletons(), 1);

  auto* joint = skel->getJoint("j1");
  joint->setPositionLowerLimit(0, posLowerBound);
  joint->setPositionUpperLimit(0, posUpperBound);
  joint->setLimitEnforcement(true);
  joint->setActuatorType(dart::dynamics::Joint::SERVO);
  auto* bodyNode = skel->getBodyNode("bar");
  ASSERT_NE(bodyNode, nullptr);

  EXPECT_DOUBLE_EQ(0, joint->getPosition(0));
  EXPECT_DOUBLE_EQ(0, joint->getVelocity(0));

  {
    // Drive the joint to the upper position limit.
    for (std::size_t i = 0; i < 1000; ++i) {
      joint->setCommand(0, velDesiredToUb);
      world->step();
    }

    EXPECT_NEAR(posUpperBound, joint->getPosition(0), 1e-6);

    // Reverse the command and expect the joint to recover from the limit.
    std::size_t firstMovementStep = 0;
    const double holdTorque = 5.0;
    for (std::size_t i = 0; i < 100; ++i) {
      joint->setCommand(0, velRecoverFromUb);
      skel->clearExternalForces();
      bodyNode->addExtTorque(Eigen::Vector3d(0.0, 0.0, holdTorque));
      world->step();
      if (firstMovementStep == 0
          && joint->getPosition(0) < posUpperBound - 1e-6) {
        firstMovementStep = i + 1;
      }
    }

    EXPECT_GT(firstMovementStep, 0u);
    EXPECT_LT(joint->getPosition(0), posUpperBound - 1e-3);
  }

  {
    // Drive the joint to the lower position limit.
    for (std::size_t i = 0; i < 2000; ++i) {
      joint->setCommand(0, velDesiredToLb);
      world->step();
    }

    EXPECT_NEAR(posLowerBound, joint->getPosition(0), 1e-6);

    // Reverse the command and expect the joint to recover from the limit.
    std::size_t firstMovementStep = 0;
    const double holdTorque = -5;
    for (std::size_t i = 0; i < 100; ++i) {
      joint->setCommand(0, velRecoverFromLb);
      skel->clearExternalForces();
      bodyNode->addExtTorque(Eigen::Vector3d(0.0, 0.0, holdTorque));
      world->step();
      if (firstMovementStep == 0
          && joint->getPosition(0) > posLowerBound + 1e-6) {
        firstMovementStep = i + 1;
      }
    }

    EXPECT_GT(firstMovementStep, 0u);
    EXPECT_GT(joint->getPosition(0), posLowerBound + 1e-3);
  }
}

//========================================================================================
TEST(Issue1683, ServoJointRespectsVelocityLimitsAwayFromBounds)
{
  const double velLimit = 0.1;
  const double torque = 5.0;

  auto skel = dart::utils::SdfParser::readSkeleton(
      "dart://sample/sdf/test/test_issue1583.model");
  ASSERT_NE(skel, nullptr);

  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  world->addSkeleton(skel);
  ASSERT_EQ(world->getNumSkeletons(), 1);

  auto* joint = skel->getJoint("j1");
  joint->setActuatorType(dart::dynamics::Joint::SERVO);
  joint->setLimitEnforcement(true);
  joint->setVelocityLowerLimit(0, -velLimit);
  joint->setVelocityUpperLimit(0, +velLimit);
  joint->setPositionLowerLimit(0, -1.0);
  joint->setPositionUpperLimit(0, +1.0);

  auto* bodyNode = skel->getBodyNode("bar");
  ASSERT_NE(bodyNode, nullptr);

  for (std::size_t i = 0; i < 200; ++i) {
    joint->setCommand(0, 0.0);
    skel->clearExternalForces();
    bodyNode->addExtTorque(Eigen::Vector3d(0.0, 0.0, torque));
    world->step();
    EXPECT_LE(joint->getVelocity(0), velLimit + 1e-6);
  }

  joint->setPosition(0, 0.0);
  joint->setVelocity(0, 0.0);

  for (std::size_t i = 0; i < 200; ++i) {
    joint->setCommand(0, 0.0);
    skel->clearExternalForces();
    bodyNode->addExtTorque(Eigen::Vector3d(0.0, 0.0, -torque));
    world->step();
    EXPECT_GE(joint->getVelocity(0), -velLimit - 1e-6);
  }
}

//==============================================================================
TEST(Issue2422, ServoJointRecoversFromPositionLimitsWithVelocityLimits)
{
  const double posLowerBound = -0.75;
  const double posUpperBound = +0.85;
  const double velDesiredToUb = 1.0;
  const double velDesiredToLb = -1.0;
  const double velRecoverFromUb = -0.05;
  const double velRecoverFromLb = 0.05;

  auto skel = dart::utils::SdfParser::readSkeleton(
      "dart://sample/sdf/test/test_issue1683.model");
  ASSERT_NE(skel, nullptr);

  auto world = dart::simulation::World::create();
  world->addSkeleton(skel);
  ASSERT_EQ(world->getNumSkeletons(), 1);

  auto* joint = skel->getJoint("j1");
  joint->setPositionLowerLimit(0, posLowerBound);
  joint->setPositionUpperLimit(0, posUpperBound);
  joint->setLimitEnforcement(true);
  joint->setActuatorType(dart::dynamics::Joint::SERVO);
  joint->setVelocityUpperLimit(0, 10.0);
  auto* bodyNode = skel->getBodyNode("bar");
  ASSERT_NE(bodyNode, nullptr);

  EXPECT_DOUBLE_EQ(0, joint->getPosition(0));
  EXPECT_DOUBLE_EQ(0, joint->getVelocity(0));

  {
    for (std::size_t i = 0; i < 1000; ++i) {
      joint->setCommand(0, velDesiredToUb);
      world->step();
    }

    EXPECT_NEAR(posUpperBound, joint->getPosition(0), 1e-6);

    std::size_t firstMovementStep = 0;
    const double holdTorque = 5.0;
    for (std::size_t i = 0; i < 100; ++i) {
      joint->setCommand(0, velRecoverFromUb);
      skel->clearExternalForces();
      bodyNode->addExtTorque(Eigen::Vector3d(0.0, 0.0, holdTorque));
      world->step();
      if (firstMovementStep == 0
          && joint->getPosition(0) < posUpperBound - 1e-6) {
        firstMovementStep = i + 1;
      }
    }

    EXPECT_GT(firstMovementStep, 0u);
    EXPECT_LT(joint->getPosition(0), posUpperBound - 1e-3);
  }

  {
    for (std::size_t i = 0; i < 2000; ++i) {
      joint->setCommand(0, velDesiredToLb);
      world->step();
    }

    EXPECT_NEAR(posLowerBound, joint->getPosition(0), 1e-6);

    std::size_t firstMovementStep = 0;
    const double holdTorque = -5;
    for (std::size_t i = 0; i < 100; ++i) {
      joint->setCommand(0, velRecoverFromLb);
      skel->clearExternalForces();
      bodyNode->addExtTorque(Eigen::Vector3d(0.0, 0.0, holdTorque));
      world->step();
      if (firstMovementStep == 0
          && joint->getPosition(0) > posLowerBound + 1e-6) {
        firstMovementStep = i + 1;
      }
    }

    EXPECT_GT(firstMovementStep, 0u);
    EXPECT_GT(joint->getPosition(0), posLowerBound + 1e-3);
  }
}
