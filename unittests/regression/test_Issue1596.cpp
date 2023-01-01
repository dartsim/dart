/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include <dart/io/sdf/sdf.hpp>

#include <dart/dart.hpp>

#include <TestHelpers.hpp>
#include <gtest/gtest.h>

//========================================================================================
TEST(Issue1596, ServoJointWithPositionLimits)
{
#if NDEBUG // release
  const auto num_steps = 50000;
#else
  const auto num_steps = 1000;
#endif
  auto skel = dart::io::SdfParser::readSkeleton(
      "dart://sample/sdf/test/test_issue1596.model");
  ASSERT_NE(skel, nullptr);

  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d(9.81, 9.81, 0));
  world->addSkeleton(skel);
  ASSERT_EQ(world->getNumSkeletons(), 1);

  auto* joint0 = skel->getJoint("joint_00");
  auto* joint1 = skel->getJoint("joint_01");
  std::vector<dart::dynamics::Joint*> joints = {joint0, joint1};

  for (auto joint : joints)
  {
    ASSERT_NE(joint, nullptr);
    ASSERT_EQ(
        joint->getType(), dart::dynamics::UniversalJoint::getStaticType());
    EXPECT_EQ(joint->getNumDofs(), 2);
    joint->setLimitEnforcement(true);

    EXPECT_DOUBLE_EQ(0, joint->getPosition(0));
    EXPECT_DOUBLE_EQ(0, joint->getPosition(1));
    EXPECT_DOUBLE_EQ(0, joint->getVelocity(0));
    EXPECT_DOUBLE_EQ(0, joint->getVelocity(1));
    EXPECT_DOUBLE_EQ(0, joint->getAcceleration(0));
    EXPECT_DOUBLE_EQ(0, joint->getAcceleration(1));
  }

  for (std::size_t i = 0; i < num_steps; ++i)
  {
    world->step();

    for (const auto joint : joints)
    {
      const double pos0 = joint->getPosition(0);
      const double pos1 = joint->getPosition(1);

      EXPECT_LE(pos0, joint->getPositionUpperLimit(0) + 1e-6);
      EXPECT_LE(pos1, joint->getPositionUpperLimit(1) + 1e-6);
      EXPECT_GE(pos0, joint->getPositionLowerLimit(0) - 1e-6);
      EXPECT_GE(pos1, joint->getPositionLowerLimit(1) - 1e-6);
    }
  }
}
