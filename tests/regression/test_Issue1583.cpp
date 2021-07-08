/*
 * Copyright (c) 2011-2020, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <TestHelpers.hpp>
#include <gtest/gtest.h>

#include <dart/dart.hpp>
#include <dart/utils/sdf/sdf.hpp>

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

  for (std::size_t i = 0; i < 1000; ++i)
  {
    joint->setCommand(0, vel_desired);
    world->step();

    const double pos = joint->getPosition(0);
    const double vel = joint->getVelocity(0);

    EXPECT_LE(pos, pos_ub + 1e-6);
    EXPECT_GE(pos, pos_lb - 1e-6);

    if (std::abs(vel - vel_desired) > 1e-6)
    {
      EXPECT_NEAR(pos, pos_ub, 1e-2);
    }
  }

  EXPECT_NEAR(pos_ub, joint->getPosition(0), 1e-2);
  EXPECT_NEAR(0, joint->getVelocity(0), 1e-6);
}
