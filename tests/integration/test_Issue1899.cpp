/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "dart/dynamics/dynamics.hpp"
#include "dart/math/math.hpp"
#include "dart/simulation/simulation.hpp"
#include "dart/utils/utils.hpp"

#include <gtest/gtest.h>

using namespace dart;
using namespace dynamics;
using namespace math;
using namespace simulation;
using namespace utils;

// Regression test for https://github.com/dartsim/dart/issues/1899.
//
// This reproduces the scenario provided in the attachment
// "Test_code_and_Reproduction_File.tar.gz" where a non-zero command is
// applied to a PASSIVE joint while stepping the world without resetting joint
// commands. Prior to the fix, ODE would assert when solving the constraints.
TEST(Issue1899, CollisionOfPrescribedJointsReproduction)
{
  const double tol = 1e-9;
  const double timeStep = 1e-3;
  const std::size_t numFrames = 5;

  auto world = SkelParser::readWorld(
      "dart://sample/skel/test/collision_of_prescribed_joints_test.skel");
  ASSERT_NE(world, nullptr);
  world->setTimeStep(timeStep);
  ASSERT_NEAR(world->getTimeStep(), timeStep, tol);

  auto skel1 = world->getSkeleton("skeleton 1");
  auto skel2 = world->getSkeleton("skeleton 2");
  auto skel3 = world->getSkeleton("skeleton 3");
  auto skel4 = world->getSkeleton("skeleton 4");
  auto skel5 = world->getSkeleton("skeleton 5");
  auto skel6 = world->getSkeleton("skeleton 6");
  ASSERT_NE(skel1, nullptr);
  ASSERT_NE(skel2, nullptr);
  ASSERT_NE(skel3, nullptr);
  ASSERT_NE(skel4, nullptr);
  ASSERT_NE(skel5, nullptr);
  ASSERT_NE(skel6, nullptr);

  Joint* joint1 = skel1->getJoint(0);
  Joint* joint2 = skel2->getJoint(0);
  Joint* joint3 = skel3->getJoint(0);
  Joint* joint4 = skel4->getJoint(0);
  Joint* joint5 = skel5->getJoint(0);
  Joint* joint6 = skel6->getJoint(0);
  ASSERT_NE(joint1, nullptr);
  ASSERT_NE(joint2, nullptr);
  ASSERT_NE(joint3, nullptr);
  ASSERT_NE(joint4, nullptr);
  ASSERT_NE(joint5, nullptr);
  ASSERT_NE(joint6, nullptr);
  ASSERT_EQ(joint1->getActuatorType(), Joint::FORCE);
  ASSERT_EQ(joint2->getActuatorType(), Joint::PASSIVE);
  ASSERT_EQ(joint3->getActuatorType(), Joint::SERVO);
  ASSERT_EQ(joint4->getActuatorType(), Joint::ACCELERATION);
  ASSERT_EQ(joint5->getActuatorType(), Joint::VELOCITY);
  ASSERT_EQ(joint6->getActuatorType(), Joint::LOCKED);

  for (std::size_t i = 0; i < numFrames; ++i) {
    const double time = world->getTime();

    joint1->setCommand(0, -0.5 * constantsd::pi() * std::cos(time));
    joint2->setCommand(0, -0.5 * constantsd::pi() * std::cos(time));
    joint3->setCommand(0, -0.5 * constantsd::pi() * std::cos(time));
    joint4->setCommand(0, -0.5 * constantsd::pi() * std::cos(time));
    joint5->setCommand(0, -0.5 * constantsd::pi() * std::sin(time));
    joint6->setCommand(0, -0.5 * constantsd::pi() * std::sin(time));

    // Do not reset joint commands after stepping.
    world->step(false);

    EXPECT_TRUE(joint1->isDynamic());
    EXPECT_TRUE(joint2->isDynamic());
    EXPECT_TRUE(joint3->isDynamic());

    EXPECT_TRUE(joint4->isKinematic());
    EXPECT_NEAR(joint4->getAcceleration(0), joint4->getCommand(0), tol);
    EXPECT_TRUE(joint5->isKinematic());
    EXPECT_NEAR(joint5->getVelocity(0), joint5->getCommand(0), tol);

    // The PASSIVE joint's command should have been cleared.
    EXPECT_EQ(joint2->getCommand(0), 0.0);

    EXPECT_TRUE(joint6->isKinematic());
    EXPECT_NEAR(joint6->getVelocity(0), 0.0, tol);
    EXPECT_NEAR(joint6->getAcceleration(0), 0.0, tol);
  }
}
