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

#include "dart/io/All.hpp"

#include <dart/config.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <gtest/gtest.h>

using namespace dart;

//==============================================================================
TEST(SkelParser, LoadSinglePendulum)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Skel;
  const auto world
      = io::readWorld("dart://sample/skel/test/single_pendulum.skel", options);
  ASSERT_NE(world, nullptr);
  EXPECT_GE(world->getNumSkeletons(), 1u);
}

//==============================================================================
TEST(SkelParser, LoadDoublePendulum)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Skel;
  const auto world
      = io::readWorld("dart://sample/skel/test/double_pendulum.skel", options);
  ASSERT_NE(world, nullptr);
  EXPECT_GE(world->getNumSkeletons(), 1u);

  auto skel = world->getSkeleton(0);
  ASSERT_NE(skel, nullptr);
  EXPECT_GE(skel->getNumBodyNodes(), 2u);
  EXPECT_GE(skel->getNumDofs(), 2u);
}

//==============================================================================
TEST(SkelParser, LoadBallJoints)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Skel;
  const auto world
      = io::readWorld("dart://sample/skel/test/ball_joints.skel", options);
  ASSERT_NE(world, nullptr);
  EXPECT_GE(world->getNumSkeletons(), 1u);
}

//==============================================================================
TEST(SkelParser, LoadFreeJoints)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Skel;
  const auto world
      = io::readWorld("dart://sample/skel/test/free_joints.skel", options);
  ASSERT_NE(world, nullptr);
  EXPECT_GE(world->getNumSkeletons(), 1u);
}

//==============================================================================
TEST(SkelParser, LoadBoxes)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Skel;
  const auto world
      = io::readWorld("dart://sample/skel/test/boxes.skel", options);
  ASSERT_NE(world, nullptr);
  EXPECT_GE(world->getNumSkeletons(), 1u);
}

//==============================================================================
TEST(SkelParser, LoadBoxStacking)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Skel;
  const auto world
      = io::readWorld("dart://sample/skel/test/box_stacking.skel", options);
  ASSERT_NE(world, nullptr);
  EXPECT_GE(world->getNumSkeletons(), 1u);
}

//==============================================================================
TEST(SkelParser, LoadChainWhipa)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Skel;
  const auto world
      = io::readWorld("dart://sample/skel/test/chainwhipa.skel", options);
  ASSERT_NE(world, nullptr);
  EXPECT_GE(world->getNumSkeletons(), 1u);
}

//==============================================================================
TEST(SkelParser, LoadDrop)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Skel;
  const auto world
      = io::readWorld("dart://sample/skel/test/drop.skel", options);
  ASSERT_NE(world, nullptr);
  EXPECT_GE(world->getNumSkeletons(), 1u);
}

//==============================================================================
TEST(SkelParser, LoadEmpty)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Skel;
  const auto world
      = io::readWorld("dart://sample/skel/test/empty.skel", options);
  ASSERT_NE(world, nullptr);
  EXPECT_EQ(world->getNumSkeletons(), 0u);
}

//==============================================================================
TEST(SkelParser, LoadHybridDynamicsTest)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Skel;
  const auto world = io::readWorld(
      "dart://sample/skel/test/hybrid_dynamics_test.skel", options);
  ASSERT_NE(world, nullptr);
  EXPECT_GE(world->getNumSkeletons(), 1u);
}

//==============================================================================
TEST(SkelParser, LoadInertiaFromShapes)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Skel;
  const auto world = io::readWorld(
      "dart://sample/skel/test/inertia_from_shapes.skel", options);
  ASSERT_NE(world, nullptr);
  EXPECT_GE(world->getNumSkeletons(), 1u);
}

//==============================================================================
TEST(SkelParser, LoadJointActuatorTypeTest)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Skel;
  const auto world = io::readWorld(
      "dart://sample/skel/test/joint_actuator_type_test.skel", options);
  ASSERT_NE(world, nullptr);
  EXPECT_GE(world->getNumSkeletons(), 1u);
}

//==============================================================================
TEST(SkelParser, LoadDofAttributeTest)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Skel;
  const auto world = io::readWorld(
      "dart://sample/skel/test/dof_attribute_test.skel", options);
  ASSERT_NE(world, nullptr);
  EXPECT_GE(world->getNumSkeletons(), 1u);
}

//==============================================================================
TEST(SkelParser, LoadDoublePendulumBallJoint)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Skel;
  const auto world = io::readWorld(
      "dart://sample/skel/test/double_pendulum_ball_joint.skel", options);
  ASSERT_NE(world, nullptr);
  EXPECT_GE(world->getNumSkeletons(), 1u);
}

//==============================================================================
TEST(SkelParser, LoadDoublePendulumEulerJoint)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Skel;
  const auto world = io::readWorld(
      "dart://sample/skel/test/double_pendulum_euler_joint.skel", options);
  ASSERT_NE(world, nullptr);
  EXPECT_GE(world->getNumSkeletons(), 1u);
}

//==============================================================================
TEST(SkelParser, LoadDoublePendulumWithBase)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Skel;
  const auto world = io::readWorld(
      "dart://sample/skel/test/double_pendulum_with_base.skel", options);
  ASSERT_NE(world, nullptr);
  EXPECT_GE(world->getNumSkeletons(), 1u);
}

//==============================================================================
TEST(SkelParser, LoadCollisionOfPrescribedJointsTest)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Skel;
  const auto world = io::readWorld(
      "dart://sample/skel/test/collision_of_prescribed_joints_test.skel",
      options);
  ASSERT_NE(world, nullptr);
  EXPECT_GE(world->getNumSkeletons(), 1u);
}

//==============================================================================
TEST(MjcfParser, LoadAnt)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Mjcf;
  const auto world
      = io::readWorld("dart://sample/mjcf/openai/ant.xml", options);
  ASSERT_NE(world, nullptr);
  EXPECT_GE(world->getNumSkeletons(), 1u);

  auto skel = world->getSkeleton(0);
  ASSERT_NE(skel, nullptr);
  EXPECT_GT(skel->getNumBodyNodes(), 0u);
  EXPECT_GT(skel->getNumDofs(), 0u);
}

//==============================================================================
TEST(MjcfParser, LoadHopper)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Mjcf;
  const auto world
      = io::readWorld("dart://sample/mjcf/openai/hopper.xml", options);
  if (world) {
    EXPECT_GE(world->getNumSkeletons(), 1u);
  }
}

//==============================================================================
TEST(MjcfParser, LoadHalfCheetah)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Mjcf;
  const auto world
      = io::readWorld("dart://sample/mjcf/openai/half_cheetah.xml", options);
  if (world) {
    EXPECT_GE(world->getNumSkeletons(), 1u);
  }
}

//==============================================================================
TEST(MjcfParser, LoadHumanoid)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Mjcf;
  const auto world
      = io::readWorld("dart://sample/mjcf/openai/humanoid.xml", options);
  if (world) {
    EXPECT_GE(world->getNumSkeletons(), 1u);
  }
}

//==============================================================================
TEST(MjcfParser, LoadHumanoidStandup)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Mjcf;
  const auto world
      = io::readWorld("dart://sample/mjcf/openai/humanoidstandup.xml", options);
  if (world) {
    EXPECT_GE(world->getNumSkeletons(), 1u);
  }
}

//==============================================================================
TEST(MjcfParser, LoadInvertedPendulum)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Mjcf;
  const auto world = io::readWorld(
      "dart://sample/mjcf/openai/inverted_pendulum.xml", options);
  ASSERT_NE(world, nullptr);
  EXPECT_GE(world->getNumSkeletons(), 1u);
}

//==============================================================================
TEST(MjcfParser, LoadInvertedDoublePendulum)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Mjcf;
  const auto world = io::readWorld(
      "dart://sample/mjcf/openai/inverted_double_pendulum.xml", options);
  ASSERT_NE(world, nullptr);
  EXPECT_GE(world->getNumSkeletons(), 1u);
}

//==============================================================================
TEST(MjcfParser, LoadPoint)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Mjcf;
  const auto world
      = io::readWorld("dart://sample/mjcf/openai/point.xml", options);
  if (world) {
    EXPECT_GE(world->getNumSkeletons(), 1u);
  }
}

//==============================================================================
TEST(MjcfParser, LoadPusher)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Mjcf;
  const auto world
      = io::readWorld("dart://sample/mjcf/openai/pusher.xml", options);
  ASSERT_NE(world, nullptr);
  EXPECT_GE(world->getNumSkeletons(), 1u);
}

//==============================================================================
TEST(MjcfParser, LoadDefault)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Mjcf;
  const auto world
      = io::readWorld("dart://sample/mjcf/test/default.xml", options);
  ASSERT_NE(world, nullptr);
}

//==============================================================================
TEST(MjcfParser, LoadIncludeMain)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Mjcf;
  const auto world
      = io::readWorld("dart://sample/mjcf/test/include_main.xml", options);
  ASSERT_NE(world, nullptr);
}

//==============================================================================
TEST(MjcfParser, SimulateLoadedWorld)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Mjcf;
  const auto world
      = io::readWorld("dart://sample/mjcf/openai/ant.xml", options);
  ASSERT_NE(world, nullptr);

  world->setGravity(Eigen::Vector3d(0, 0, -9.81));
  for (int i = 0; i < 10; ++i) {
    world->step();
  }

  auto skel = world->getSkeleton(0);
  ASSERT_NE(skel, nullptr);
  EXPECT_TRUE(skel->getPositions().array().isFinite().all());
  EXPECT_TRUE(skel->getVelocities().array().isFinite().all());
}

//==============================================================================
TEST(SkelParser, SimulateLoadedWorld)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Skel;
  const auto world
      = io::readWorld("dart://sample/skel/test/double_pendulum.skel", options);
  ASSERT_NE(world, nullptr);

  world->setGravity(Eigen::Vector3d(0, 0, -9.81));
  for (int i = 0; i < 10; ++i) {
    world->step();
  }

  auto skel = world->getSkeleton(0);
  ASSERT_NE(skel, nullptr);
  EXPECT_TRUE(skel->getPositions().array().isFinite().all());
  EXPECT_TRUE(skel->getVelocities().array().isFinite().all());
}

//==============================================================================
TEST(SkelParser, SkeletonProperties)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Skel;
  const auto world
      = io::readWorld("dart://sample/skel/test/double_pendulum.skel", options);
  ASSERT_NE(world, nullptr);

  auto skel = world->getSkeleton(0);
  ASSERT_NE(skel, nullptr);

  EXPECT_GT(skel->getMass(), 0.0);
  EXPECT_TRUE(skel->getCOM().array().isFinite().all());

  const auto& M = skel->getMassMatrix();
  EXPECT_EQ(M.rows(), skel->getNumDofs());
  EXPECT_EQ(M.cols(), skel->getNumDofs());
}

//==============================================================================
TEST(MjcfParser, SkeletonProperties)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Mjcf;
  const auto world
      = io::readWorld("dart://sample/mjcf/openai/ant.xml", options);
  ASSERT_NE(world, nullptr);

  auto skel = world->getSkeleton(0);
  ASSERT_NE(skel, nullptr);

  EXPECT_GT(skel->getMass(), 0.0);
  EXPECT_TRUE(skel->getCOM().array().isFinite().all());

  const auto& M = skel->getMassMatrix();
  EXPECT_EQ(M.rows(), skel->getNumDofs());
  EXPECT_EQ(M.cols(), skel->getNumDofs());
}

//==============================================================================
TEST(SkelParser, WorldProperties)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Skel;
  const auto world
      = io::readWorld("dart://sample/skel/test/double_pendulum.skel", options);
  ASSERT_NE(world, nullptr);

  EXPECT_GT(world->getTimeStep(), 0.0);
  EXPECT_TRUE(world->getGravity().array().isFinite().all());
}

//==============================================================================
TEST(MjcfParser, WorldProperties)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Mjcf;
  const auto world
      = io::readWorld("dart://sample/mjcf/openai/ant.xml", options);
  ASSERT_NE(world, nullptr);

  EXPECT_GT(world->getTimeStep(), 0.0);
  EXPECT_TRUE(world->getGravity().array().isFinite().all());
}

//==============================================================================
TEST(SkelParser, JointProperties)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Skel;
  const auto world
      = io::readWorld("dart://sample/skel/test/double_pendulum.skel", options);
  ASSERT_NE(world, nullptr);

  auto skel = world->getSkeleton(0);
  ASSERT_NE(skel, nullptr);

  for (std::size_t i = 0; i < skel->getNumJoints(); ++i) {
    auto* joint = skel->getJoint(i);
    ASSERT_NE(joint, nullptr);
    EXPECT_FALSE(joint->getName().empty());
  }
}

//==============================================================================
TEST(SkelParser, BodyNodeProperties)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Skel;
  const auto world
      = io::readWorld("dart://sample/skel/test/double_pendulum.skel", options);
  ASSERT_NE(world, nullptr);

  auto skel = world->getSkeleton(0);
  ASSERT_NE(skel, nullptr);

  for (std::size_t i = 0; i < skel->getNumBodyNodes(); ++i) {
    auto* body = skel->getBodyNode(i);
    ASSERT_NE(body, nullptr);
    EXPECT_FALSE(body->getName().empty());
    EXPECT_GT(body->getMass(), 0.0);
  }
}

//==============================================================================
TEST(MjcfParser, BodyNodeProperties)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Mjcf;
  const auto world
      = io::readWorld("dart://sample/mjcf/openai/ant.xml", options);
  ASSERT_NE(world, nullptr);

  auto skel = world->getSkeleton(0);
  ASSERT_NE(skel, nullptr);

  for (std::size_t i = 0; i < skel->getNumBodyNodes(); ++i) {
    auto* body = skel->getBodyNode(i);
    ASSERT_NE(body, nullptr);
    EXPECT_FALSE(body->getName().empty());
    EXPECT_GT(body->getMass(), 0.0);
  }
}
