/*
 * Copyright (c) 2011-2021, The DART development contributors
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

// The tests in this file are inspired by the tests in:
// https://github.com/osrf/gazebo/blob/01b395a5fa92eb054c72f9a2027cdcfd35f287f4/test/integration/joint_force_torque.cc

#include <gtest/gtest.h>

#include "dart/dart.hpp"
#include "dart/utils/sdf/sdf.hpp"

#include "TestHelpers.hpp"

using namespace dart;
using namespace dart::math;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;

//==============================================================================
WorldPtr readWorld(const common::Uri& uri)
{
  WorldPtr world = SdfParser::readWorld(uri);
  for (auto i = 0u; i < world->getNumSkeletons(); ++i)
  {
    auto skel = world->getSkeleton(i);
    for (auto j = 0u; j < skel->getNumJoints(); ++j)
    {
      auto joint = skel->getJoint(j);
      joint->setLimitEnforcement(true);
    }
  }
  return world;
}

//==============================================================================
TEST(JointForceTorqueTest, Static)
{
  // Load world
  WorldPtr world = readWorld("dart://sample/sdf/test/force_torque_test.world");
  ASSERT_NE(world, nullptr);

  // Check if the world is correct loaded
  ASSERT_EQ(world->getNumSkeletons(), 1);
  SkeletonPtr model_1 = world->getSkeleton(0);
  ASSERT_NE(model_1, nullptr);
  EXPECT_EQ(model_1->getName(), "model_1");
  EXPECT_EQ(model_1->getNumBodyNodes(), 2);
  EXPECT_EQ(model_1->getNumJoints(), 2);

  world->setGravity(Eigen::Vector3d(0, 0, -50));

  // Simulate 1 step
  world->step();
  double t = world->getTime();

  // Get time step size
  double dt = world->getTimeStep();
  EXPECT_GT(dt, 0);

  // Check that time moves forward
  EXPECT_DOUBLE_EQ(t, dt);

  // Get joint and get force torque
  auto link_1 = model_1->getBodyNode("link_1");
  ASSERT_NE(link_1, nullptr);
  auto link_2 = model_1->getBodyNode("link_2");
  ASSERT_NE(link_2, nullptr);
  auto joint_01 = model_1->getJoint("joint_01");
  ASSERT_NE(joint_01, nullptr);
  auto joint_12 = model_1->getJoint("joint_12");
  ASSERT_NE(joint_12, nullptr);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();

  // Run 10 steps
  for (auto i = 0u; i < 10; ++i)
  {
    world->step();

    //----------------------
    // Test joint_01 wrench
    //----------------------

    // Reference adjustment for different joint frame conventions between Gazebo and DART
    tf.setIdentity();
    tf.translation() = joint_01->getTransformFromParentBodyNode().translation();
    SimpleFramePtr parentFrame01
        = std::make_shared<SimpleFrame>(Frame::World(), "parentFrame01", tf);
    tf.setIdentity();
    tf.translation() = joint_01->getTransformFromChildBodyNode().translation();
    SimpleFramePtr childFrame01
        = std::make_shared<SimpleFrame>(link_1, "childFrame01", tf);

    const Eigen::Vector6d parentF01 = joint_01->getWrenchToParentBodyNode(parentFrame01.get());
    EXPECT_DOUBLE_EQ(parentF01[0], 0); // torque
    EXPECT_DOUBLE_EQ(parentF01[1], 0);
    EXPECT_DOUBLE_EQ(parentF01[2], 0);
    EXPECT_DOUBLE_EQ(parentF01[3], 0); // force
    EXPECT_DOUBLE_EQ(parentF01[4], 0);
    EXPECT_DOUBLE_EQ(parentF01[5], 1000);

    const Eigen::Vector6d childF01 = joint_01->getWrenchToChildBodyNode(childFrame01.get());
    EXPECT_EQ(childF01, -parentF01);

    //----------------------
    // Test joint_12 wrench
    //----------------------

    // Reference adjustment for different joint frame conventions between Gazebo and DART
    tf.setIdentity();
    tf.translation() = joint_12->getTransformFromParentBodyNode().translation();
    SimpleFramePtr parentFrame12
        = std::make_shared<SimpleFrame>(link_1, "parentFrame12", tf);
    tf.setIdentity();
    tf.translation() = joint_12->getTransformFromChildBodyNode().translation();
    SimpleFramePtr childFrame12
        = std::make_shared<SimpleFrame>(link_2, "childFrame12", tf);

    const Eigen::Vector6d parentF12 = joint_12->getWrenchToParentBodyNode(parentFrame12.get());
    EXPECT_DOUBLE_EQ(parentF12[0], 0); // torque
    EXPECT_DOUBLE_EQ(parentF12[1], 0);
    EXPECT_DOUBLE_EQ(parentF12[2], 0);
    EXPECT_DOUBLE_EQ(parentF12[3], 0); // force
    EXPECT_DOUBLE_EQ(parentF12[4], 0);
    EXPECT_DOUBLE_EQ(parentF12[5], 500);

    const Eigen::Vector6d childF12 = joint_12->getWrenchToChildBodyNode(childFrame01.get());
    EXPECT_EQ(childF12, -parentF12);
  }
}

//==============================================================================
TEST(JointForceTorqueTest, ForceTorqeAtJointLimits)
{
  // Load world
  WorldPtr world = readWorld("dart://sample/sdf/test/force_torque_test.world");
  ASSERT_NE(world, nullptr);

  // Check if the world is correct loaded
  ASSERT_EQ(world->getNumSkeletons(), 1);
  SkeletonPtr model_1 = world->getSkeleton(0);
  ASSERT_NE(model_1, nullptr);
  EXPECT_EQ(model_1->getName(), "model_1");
  EXPECT_EQ(model_1->getNumBodyNodes(), 2);
  EXPECT_EQ(model_1->getNumJoints(), 2);

  world->setGravity(Eigen::Vector3d(0, 0, -50));

  // Simulate 1 step
  world->step();
  double t = world->getTime();

  // Get time step size
  double dt = world->getTimeStep();
  EXPECT_GT(dt, 0);

  // Check that time moves forward
  EXPECT_DOUBLE_EQ(t, dt);

  // Get joint and get force torque
  auto link_1 = model_1->getBodyNode("link_1");
  ASSERT_NE(link_1, nullptr);
  auto link_2 = model_1->getBodyNode("link_2");
  ASSERT_NE(link_2, nullptr);
  auto joint_01 = model_1->getJoint("joint_01");
  ASSERT_NE(joint_01, nullptr);
  auto joint_12 = model_1->getJoint("joint_12");
  ASSERT_NE(joint_12, nullptr);

  // Change gravity so that the top link topples over, then remeasure
  world->setGravity(Eigen::Vector3d(-30, 10, -50));

  // Wait for dynamics to be stabilized
  for (auto i = 0; i < 2000; ++i)
  {
    world->step();
  }

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();

  // Run 5 steps
  for (auto i = 0u; i < 5; ++i)
  {
    world->step();

    //----------------------
    // Test joint_01 wrench
    //----------------------

    // Reference adjustment for different joint frame conventions between Gazebo and DART
    tf.setIdentity();
    tf.translation() = joint_01->getTransformFromParentBodyNode().translation();
    SimpleFramePtr parentFrame01
        = std::make_shared<SimpleFrame>(Frame::World(), "parentFrame01", tf);
    tf.setIdentity();
    tf.translation() = joint_01->getTransformFromChildBodyNode().translation();
    SimpleFramePtr childFrame01
        = std::make_shared<SimpleFrame>(link_1, "childFrame01", tf);

    const Eigen::Vector6d parentF01
        = joint_01->getWrenchToParentBodyNode(parentFrame01.get());
    EXPECT_NEAR(parentF01[0], 750, 7.5); // torque
    EXPECT_NEAR(parentF01[1], 0, 4.5);
    EXPECT_NEAR(parentF01[2], -450, 0.1);
    EXPECT_NEAR(parentF01[3], 600, 6); // force
    EXPECT_NEAR(parentF01[4], -200, 10);
    EXPECT_NEAR(parentF01[5], 1000, 2);

    const Eigen::Vector6d childF01
        = joint_01->getWrenchToChildBodyNode(childFrame01.get());
    EXPECT_NEAR(childF01[0], -750, 7.5); // torque
    EXPECT_NEAR(childF01[1], -450, 4.5);
    EXPECT_NEAR(childF01[2], 0, 0.1);
    EXPECT_NEAR(childF01[3], -600, 6); // force
    EXPECT_NEAR(childF01[4], 1000, 10);
    EXPECT_NEAR(childF01[5], 200, 2);

    //----------------------
    // Test joint_12 wrench
    //----------------------

    // Reference adjustment for different joint frame conventions between Gazebo and DART
    tf.setIdentity();
    tf.translation() = joint_12->getTransformFromParentBodyNode().translation();
    SimpleFramePtr parentFrame12
        = std::make_shared<SimpleFrame>(link_1, "parentFrame12", tf);
    tf.setIdentity();
    tf.translation() = joint_12->getTransformFromChildBodyNode().translation();
    SimpleFramePtr childFrame12
        = std::make_shared<SimpleFrame>(link_2, "childFrame12", tf);

    const Eigen::Vector6d parentF12
        = joint_12->getWrenchToParentBodyNode(parentFrame12.get());
    EXPECT_NEAR(parentF12[0], 250, 2.5); // torque
    EXPECT_NEAR(parentF12[1], 150, 1.5);
    EXPECT_NEAR(parentF12[2], 0, 0.1);
    EXPECT_NEAR(parentF12[3], 300, 3); // force
    EXPECT_NEAR(parentF12[4], -500, 5);
    EXPECT_NEAR(parentF12[5], -100, 1);

    const Eigen::Vector6d childF12
        = joint_12->getWrenchToChildBodyNode(childFrame12.get());
    EXPECT_TRUE(childF12.isApprox(-parentF12));
  }
}
