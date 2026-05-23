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

#include <dart/simulation/experimental/common/exceptions.hpp>
#include <dart/simulation/experimental/multibody/multibody.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace sim = dart::simulation::experimental;

TEST(Joint, GeneralizedPositionAndVelocity)
{
  sim::World world;
  auto arm = world.addMultibody("arm");
  auto base = arm.addLink("base");
  arm.addLink(
      "forearm",
      {.parentLink = base,
       .jointName = "elbow",
       .jointType = sim::JointType::Revolute});

  auto joint = arm.getJoint("elbow");
  ASSERT_TRUE(joint.has_value());
  EXPECT_TRUE(joint->isValid());
  EXPECT_EQ(joint->getDOFCount(), 1u);
  EXPECT_TRUE(joint->getPosition().isApprox(Eigen::VectorXd::Zero(1)));
  EXPECT_TRUE(joint->getVelocity().isApprox(Eigen::VectorXd::Zero(1)));

  Eigen::VectorXd position(1);
  position << 0.25;
  joint->setPosition(position);
  position[0] = 1.0;
  EXPECT_EQ(joint->getPosition().size(), 1);
  EXPECT_DOUBLE_EQ(joint->getPosition()[0], 0.25);

  Eigen::VectorXd velocity(1);
  velocity << -0.5;
  joint->setVelocity(velocity);
  velocity[0] = 1.0;
  EXPECT_EQ(joint->getVelocity().size(), 1);
  EXPECT_DOUBLE_EQ(joint->getVelocity()[0], -0.5);
}

TEST(Joint, ReportsExpectedDOFCounts)
{
  sim::World world;
  auto mechanism = world.addMultibody("mechanism");
  auto base = mechanism.addLink("base");

  const std::vector<std::pair<sim::JointType, std::size_t>> cases = {
      {sim::JointType::Fixed, 0},
      {sim::JointType::Revolute, 1},
      {sim::JointType::Prismatic, 1},
      {sim::JointType::Screw, 1},
      {sim::JointType::Universal, 2},
      {sim::JointType::Spherical, 3},
      {sim::JointType::Planar, 3},
      {sim::JointType::Free, 6},
      {sim::JointType::Custom, 0},
  };

  for (std::size_t i = 0; i < cases.size(); ++i) {
    const auto [type, expectedDof] = cases[i];
    const auto jointName = "joint_" + std::to_string(i);
    mechanism.addLink(
        "link_" + std::to_string(i),
        {.parentLink = base, .jointName = jointName, .jointType = type});

    auto joint = mechanism.getJoint(jointName);
    ASSERT_TRUE(joint.has_value());
    EXPECT_EQ(joint->getDOFCount(), expectedDof);

    joint->setPosition(Eigen::VectorXd::Zero(expectedDof));
    joint->setVelocity(Eigen::VectorXd::Zero(expectedDof));
    EXPECT_EQ(joint->getPosition().size(), expectedDof);
    EXPECT_EQ(joint->getVelocity().size(), expectedDof);
  }
}

TEST(Joint, FixedJointAcceptsZeroLengthState)
{
  sim::World world;
  auto mechanism = world.addMultibody("mechanism");
  auto base = mechanism.addLink("base");
  mechanism.addLink(
      "fixture",
      {.parentLink = base,
       .jointName = "weld",
       .jointType = sim::JointType::Fixed});

  auto joint = mechanism.getJoint("weld");
  ASSERT_TRUE(joint.has_value());
  EXPECT_EQ(joint->getDOFCount(), 0u);

  joint->setPosition(Eigen::VectorXd::Zero(0));
  joint->setVelocity(Eigen::VectorXd::Zero(0));

  EXPECT_EQ(joint->getPosition().size(), 0);
  EXPECT_EQ(joint->getVelocity().size(), 0);
}

TEST(Joint, InvalidHandleThrows)
{
  sim::World world;
  auto arm = world.addMultibody("arm");
  auto base = arm.addLink("base");

  auto invalidJoint = base.getParentJoint();
  EXPECT_FALSE(invalidJoint.isValid());
  EXPECT_THROW(
      { (void)invalidJoint.getDOFCount(); }, sim::InvalidArgumentException);
  EXPECT_THROW(
      { (void)invalidJoint.getPosition(); }, sim::InvalidArgumentException);
  EXPECT_THROW(
      invalidJoint.setPosition(Eigen::VectorXd::Zero(0)),
      sim::InvalidArgumentException);
}

TEST(Joint, RejectsInvalidStateVectors)
{
  sim::World world;
  auto arm = world.addMultibody("arm");
  auto base = arm.addLink("base");
  arm.addLink(
      "forearm",
      {.parentLink = base,
       .jointName = "elbow",
       .jointType = sim::JointType::Revolute});

  auto joint = arm.getJoint("elbow");
  ASSERT_TRUE(joint.has_value());

  EXPECT_THROW(
      joint->setPosition(Eigen::VectorXd::Zero(2)),
      sim::InvalidArgumentException);

  Eigen::VectorXd nonFinitePosition(1);
  nonFinitePosition << std::numeric_limits<double>::infinity();
  EXPECT_THROW(
      joint->setPosition(nonFinitePosition), sim::InvalidArgumentException);

  EXPECT_THROW(
      joint->setVelocity(Eigen::VectorXd::Zero(2)),
      sim::InvalidArgumentException);

  Eigen::VectorXd nonFiniteVelocity(1);
  nonFiniteVelocity << std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
      joint->setVelocity(nonFiniteVelocity), sim::InvalidArgumentException);
}
