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
#include <dart/simulation/experimental/comps/joint.hpp>
#include <dart/simulation/experimental/multi_body/joint.hpp>
#include <dart/simulation/experimental/multi_body/link.hpp>
#include <dart/simulation/experimental/multi_body/multi_body.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <gtest/gtest.h>

namespace dse = dart::simulation::experimental;

//==============================================================================
// Joint Handle Basic Tests
//==============================================================================

// Test Joint handle creation and getName
TEST(Joint, GetName)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");
  auto link1 = robot.addLink(
      "link1", {.parentLink = base, .jointName = "shoulder_joint"});

  auto joint = link1.getParentJoint();
  EXPECT_EQ(joint.getName(), "shoulder_joint");
}

// Test Joint handle getType for revolute (default)
TEST(Joint, GetTypeRevolute)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");
  auto link1 = robot.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "joint1",
       .jointType = dse::comps::JointType::Revolute});

  auto joint = link1.getParentJoint();
  EXPECT_EQ(joint.getType(), dse::comps::JointType::Revolute);
}

// Test Joint handle getType for prismatic
TEST(Joint, GetTypePrismatic)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");
  auto link1 = robot.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "slider",
       .jointType = dse::comps::JointType::Prismatic});

  auto joint = link1.getParentJoint();
  EXPECT_EQ(joint.getType(), dse::comps::JointType::Prismatic);
}

// Test Joint handle getType for fixed (0-DOF weld joint)
TEST(Joint, GetTypeFixed)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");
  auto link1 = robot.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "weld",
       .jointType = dse::comps::JointType::Fixed});

  auto joint = link1.getParentJoint();
  EXPECT_EQ(joint.getType(), dse::comps::JointType::Fixed);
}

// Test Fixed joint has 0 DOF (doesn't contribute to robot's DOF count)
TEST(Joint, FixedJointZeroDOF)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");

  // Add a revolute joint (1 DOF)
  auto link1 = robot.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "revolute1",
       .jointType = dse::comps::JointType::Revolute});

  // Add a fixed joint (0 DOF)
  auto link2 = robot.addLink(
      "link2",
      {.parentLink = link1,
       .jointName = "fixed1",
       .jointType = dse::comps::JointType::Fixed});

  // Add another revolute joint (1 DOF)
  auto link3 = robot.addLink(
      "link3",
      {.parentLink = link2,
       .jointName = "revolute2",
       .jointType = dse::comps::JointType::Revolute});

  // Total DOF should be 2 (revolute + fixed + revolute = 1 + 0 + 1)
  EXPECT_EQ(robot.getDOFCount(), 2u);
}

//==============================================================================
// Joint Axis Tests
//==============================================================================

// Test getAxis for revolute joint with default axis (Z)
TEST(Joint, GetAxisDefaultZ)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");
  auto link1 = robot.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "joint1",
       .jointType = dse::comps::JointType::Revolute});

  auto joint = link1.getParentJoint();
  Eigen::Vector3d axis = joint.getAxis();
  EXPECT_DOUBLE_EQ(axis.x(), 0.0);
  EXPECT_DOUBLE_EQ(axis.y(), 0.0);
  EXPECT_DOUBLE_EQ(axis.z(), 1.0);
}

// Test getAxis for revolute joint with custom axis
TEST(Joint, GetAxisCustom)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");
  auto link1 = robot.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "joint1",
       .jointType = dse::comps::JointType::Revolute,
       .axis = Eigen::Vector3d::UnitX()});

  auto joint = link1.getParentJoint();
  Eigen::Vector3d axis = joint.getAxis();
  EXPECT_DOUBLE_EQ(axis.x(), 1.0);
  EXPECT_DOUBLE_EQ(axis.y(), 0.0);
  EXPECT_DOUBLE_EQ(axis.z(), 0.0);
}

// Test getAxis for prismatic joint
TEST(Joint, GetAxisPrismatic)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");
  auto link1 = robot.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "slider",
       .jointType = dse::comps::JointType::Prismatic,
       .axis = Eigen::Vector3d::UnitY()});

  auto joint = link1.getParentJoint();
  Eigen::Vector3d axis = joint.getAxis();
  EXPECT_DOUBLE_EQ(axis.x(), 0.0);
  EXPECT_DOUBLE_EQ(axis.y(), 1.0);
  EXPECT_DOUBLE_EQ(axis.z(), 0.0);
}

//==============================================================================
// Joint Parent/Child Link Tests
//==============================================================================

// Test getParentLink and getChildLink
TEST(Joint, GetParentAndChildLinks)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");
  auto link1
      = robot.addLink("link1", {.parentLink = base, .jointName = "joint1"});

  auto joint = link1.getParentJoint();

  auto parentLink = joint.getParentLink();
  auto childLink = joint.getChildLink();

  EXPECT_EQ(parentLink.getName(), "base");
  EXPECT_EQ(childLink.getName(), "link1");
}

// Test isValid for a valid joint
TEST(Joint, IsValidTrue)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");
  auto link1
      = robot.addLink("link1", {.parentLink = base, .jointName = "joint1"});

  auto joint = link1.getParentJoint();
  EXPECT_TRUE(joint.isValid());
}

// Test getEntity returns a valid entity
TEST(Joint, GetEntity)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");
  auto link1
      = robot.addLink("link1", {.parentLink = base, .jointName = "joint1"});

  auto joint = link1.getParentJoint();
  auto entity = joint.getEntity();
  EXPECT_TRUE(entity != entt::null);
}

//==============================================================================
// Joint Chain Tests
//==============================================================================

// Test multiple joints in a chain
TEST(Joint, MultipleJointsInChain)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");

  auto base = robot.addLink("base");
  auto link1 = robot.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "shoulder",
       .jointType = dse::comps::JointType::Revolute,
       .axis = Eigen::Vector3d::UnitZ()});
  auto link2 = robot.addLink(
      "link2",
      {.parentLink = link1,
       .jointName = "elbow",
       .jointType = dse::comps::JointType::Revolute,
       .axis = Eigen::Vector3d::UnitY()});
  auto link3 = robot.addLink(
      "link3",
      {.parentLink = link2,
       .jointName = "wrist",
       .jointType = dse::comps::JointType::Prismatic,
       .axis = Eigen::Vector3d::UnitX()});

  // Check shoulder joint
  auto shoulder = link1.getParentJoint();
  EXPECT_EQ(shoulder.getName(), "shoulder");
  EXPECT_EQ(shoulder.getType(), dse::comps::JointType::Revolute);
  EXPECT_EQ(shoulder.getAxis(), Eigen::Vector3d::UnitZ());

  // Check elbow joint
  auto elbow = link2.getParentJoint();
  EXPECT_EQ(elbow.getName(), "elbow");
  EXPECT_EQ(elbow.getType(), dse::comps::JointType::Revolute);
  EXPECT_EQ(elbow.getAxis(), Eigen::Vector3d::UnitY());

  // Check wrist joint
  auto wrist = link3.getParentJoint();
  EXPECT_EQ(wrist.getName(), "wrist");
  EXPECT_EQ(wrist.getType(), dse::comps::JointType::Prismatic);
  EXPECT_EQ(wrist.getAxis(), Eigen::Vector3d::UnitX());
}

//==============================================================================
// Joint Handle Copy Semantics
//==============================================================================

// Test that joint handles are copyable and point to the same entity
TEST(Joint, HandleCopySemantics)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");
  auto link1
      = robot.addLink("link1", {.parentLink = base, .jointName = "joint1"});

  auto joint1 = link1.getParentJoint();
  auto joint2 = joint1; // Copy

  EXPECT_EQ(joint1.getEntity(), joint2.getEntity());
  EXPECT_EQ(joint1.getName(), joint2.getName());
  EXPECT_EQ(joint1.getType(), joint2.getType());
}

// Test joint handles from different access paths point to same entity
TEST(Joint, HandleFromDifferentPaths)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");
  auto link1
      = robot.addLink("link1", {.parentLink = base, .jointName = "joint1"});

  // Get joint via link's getParentJoint
  auto joint1 = link1.getParentJoint();

  // Get joint via robot's getJoint
  auto joint2 = robot.getJoint("joint1");

  ASSERT_TRUE(joint2.has_value());
  EXPECT_EQ(joint1.getEntity(), joint2->getEntity());
}

//==============================================================================
// All Joint Types - Type and DOF Verification
//==============================================================================

TEST(Joint, GetTypeBall)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");
  auto link1 = robot.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "ball_joint",
       .jointType = dse::comps::JointType::Ball});

  auto joint = link1.getParentJoint();
  EXPECT_EQ(joint.getType(), dse::comps::JointType::Ball);
}

TEST(Joint, BallJointThreeDOF)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");

  auto link1 = robot.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "ball",
       .jointType = dse::comps::JointType::Ball});

  EXPECT_EQ(robot.getDOFCount(), 3u);
}

TEST(Joint, GetTypeFree)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");
  auto link1 = robot.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "free_joint",
       .jointType = dse::comps::JointType::Free});

  auto joint = link1.getParentJoint();
  EXPECT_EQ(joint.getType(), dse::comps::JointType::Free);
}

TEST(Joint, FreeJointSixDOF)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");

  auto link1 = robot.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "free",
       .jointType = dse::comps::JointType::Free});

  EXPECT_EQ(robot.getDOFCount(), 6u);
}

TEST(Joint, GetTypeUniversal)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");
  auto link1 = robot.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "universal_joint",
       .jointType = dse::comps::JointType::Universal});

  auto joint = link1.getParentJoint();
  EXPECT_EQ(joint.getType(), dse::comps::JointType::Universal);
}

TEST(Joint, UniversalJointTwoDOF)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");

  auto link1 = robot.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "universal",
       .jointType = dse::comps::JointType::Universal});

  EXPECT_EQ(robot.getDOFCount(), 2u);
}

TEST(Joint, GetTypePlanar)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");
  auto link1 = robot.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "planar_joint",
       .jointType = dse::comps::JointType::Planar});

  auto joint = link1.getParentJoint();
  EXPECT_EQ(joint.getType(), dse::comps::JointType::Planar);
}

TEST(Joint, PlanarJointThreeDOF)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");

  auto link1 = robot.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "planar",
       .jointType = dse::comps::JointType::Planar});

  EXPECT_EQ(robot.getDOFCount(), 3u);
}

TEST(Joint, GetTypeScrew)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");
  auto link1 = robot.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "screw_joint",
       .jointType = dse::comps::JointType::Screw});

  auto joint = link1.getParentJoint();
  EXPECT_EQ(joint.getType(), dse::comps::JointType::Screw);
}

TEST(Joint, ScrewJointOneDOF)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");

  auto link1 = robot.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "screw",
       .jointType = dse::comps::JointType::Screw});

  EXPECT_EQ(robot.getDOFCount(), 1u);
}

//==============================================================================
// Mixed Joint Type Chain - DOF Accumulation
//==============================================================================

TEST(Joint, MixedJointTypesDOFAccumulation)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");

  auto link1 = robot.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "free",
       .jointType = dse::comps::JointType::Free});

  auto link2 = robot.addLink(
      "link2",
      {.parentLink = link1,
       .jointName = "ball",
       .jointType = dse::comps::JointType::Ball});

  auto link3 = robot.addLink(
      "link3",
      {.parentLink = link2,
       .jointName = "revolute",
       .jointType = dse::comps::JointType::Revolute});

  auto link4 = robot.addLink(
      "link4",
      {.parentLink = link3,
       .jointName = "fixed",
       .jointType = dse::comps::JointType::Fixed});

  EXPECT_EQ(robot.getDOFCount(), 10u);
}

TEST(Joint, AllJointTypesInOneRobot)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");

  auto l1 = robot.addLink(
      "l1",
      {.parentLink = base,
       .jointName = "j_fixed",
       .jointType = dse::comps::JointType::Fixed});

  auto l2 = robot.addLink(
      "l2",
      {.parentLink = l1,
       .jointName = "j_revolute",
       .jointType = dse::comps::JointType::Revolute});

  auto l3 = robot.addLink(
      "l3",
      {.parentLink = l2,
       .jointName = "j_prismatic",
       .jointType = dse::comps::JointType::Prismatic});

  auto l4 = robot.addLink(
      "l4",
      {.parentLink = l3,
       .jointName = "j_screw",
       .jointType = dse::comps::JointType::Screw});

  auto l5 = robot.addLink(
      "l5",
      {.parentLink = l4,
       .jointName = "j_universal",
       .jointType = dse::comps::JointType::Universal});

  auto l6 = robot.addLink(
      "l6",
      {.parentLink = l5,
       .jointName = "j_ball",
       .jointType = dse::comps::JointType::Ball});

  auto l7 = robot.addLink(
      "l7",
      {.parentLink = l6,
       .jointName = "j_planar",
       .jointType = dse::comps::JointType::Planar});

  auto l8 = robot.addLink(
      "l8",
      {.parentLink = l7,
       .jointName = "j_free",
       .jointType = dse::comps::JointType::Free});

  EXPECT_EQ(robot.getJointCount(), 8u);
  EXPECT_EQ(robot.getDOFCount(), 17u);
}
