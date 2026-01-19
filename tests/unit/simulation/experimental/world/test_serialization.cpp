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

#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/common/constants.hpp>
#include <dart/simulation/experimental/comps/frame_types.hpp>
#include <dart/simulation/experimental/comps/joint.hpp>
#include <dart/simulation/experimental/comps/multi_body.hpp>
#include <dart/simulation/experimental/comps/name.hpp>
#include <dart/simulation/experimental/frame/fixed_frame.hpp>
#include <dart/simulation/experimental/frame/frame.hpp>
#include <dart/simulation/experimental/frame/free_frame.hpp>
#include <dart/simulation/experimental/multi_body/joint.hpp>
#include <dart/simulation/experimental/multi_body/link.hpp>
#include <dart/simulation/experimental/multi_body/multi_body.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <gtest/gtest.h>

#include <iterator>
#include <sstream>

//==============================================================================
// Serialization Tests - Comprehensive Coverage
//==============================================================================

// Test save/load empty world
TEST(Serialization, EmptyWorld)
{
  dart::simulation::experimental::World world1;

  // Save to stream
  std::stringstream ss;
  world1.saveBinary(ss);
  // Load into new world
  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  // Verify empty
  EXPECT_EQ(world2.getMultiBodyCount(), 0);
  EXPECT_EQ(world2.getRigidBodyCount(), 0);
  EXPECT_FALSE(world2.isSimulationMode());
}

// Test save/load world with single multibody (no links)
TEST(Serialization, SingleMultiBodyNoLinks)
{
  dart::simulation::experimental::World world1;
  [[maybe_unused]] auto mb1 = world1.addMultiBody("robot1");

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  EXPECT_EQ(world2.getMultiBodyCount(), 1);
  auto mb_restored = world2.getMultiBody("robot1");
  ASSERT_TRUE(mb_restored.has_value());
  EXPECT_EQ(mb_restored->getName(), "robot1");
  EXPECT_EQ(mb_restored->getLinkCount(), 0);
  EXPECT_EQ(mb_restored->getJointCount(), 0);
}

// Test save/load world with single link (root, no parent joint)
TEST(Serialization, SingleRootLink)
{
  dart::simulation::experimental::World world1;
  auto mb = world1.addMultiBody("robot");
  [[maybe_unused]] auto base = mb.addLink("base");

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  auto mb_restored = world2.getMultiBody("robot");
  ASSERT_TRUE(mb_restored.has_value());
  EXPECT_EQ(mb_restored->getName(), "robot"); // Verify name preserved
  EXPECT_EQ(mb_restored->getLinkCount(), 1);
  EXPECT_EQ(mb_restored->getJointCount(), 0);
}

// Test save/load simple 2-link chain with full property verification
TEST(Serialization, TwoLinkChain)
{
  dart::simulation::experimental::World world1;
  auto mb = world1.addMultiBody("robot");
  auto base = mb.addLink("base");
  [[maybe_unused]] auto link1 = mb.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "joint1",
       .jointType = dart::simulation::experimental::comps::JointType::Revolute,
       .axis = {0, 0, 1}});

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  // Verify MultiBody restored
  auto mb_restored = world2.getMultiBody("robot");
  ASSERT_TRUE(mb_restored.has_value());
  EXPECT_EQ(mb_restored->getName(), "robot");
  EXPECT_EQ(mb_restored->getLinkCount(), 2);
  EXPECT_EQ(mb_restored->getJointCount(), 1);

  // Verify links are accessible by name
  auto base_restored = mb_restored->getLink("base");
  ASSERT_TRUE(base_restored.has_value());
  EXPECT_EQ(base_restored->getName(), "base");

  auto link1_restored = mb_restored->getLink("link1");
  ASSERT_TRUE(link1_restored.has_value());
  EXPECT_EQ(link1_restored->getName(), "link1");

  // Verify joint properties
  auto joint1_restored = mb_restored->getJoint("joint1");
  ASSERT_TRUE(joint1_restored.has_value());
  EXPECT_EQ(joint1_restored->getName(), "joint1");
  EXPECT_EQ(
      joint1_restored->getType(),
      dart::simulation::experimental::comps::JointType::Revolute);

  // Verify joint axis
  auto axis = joint1_restored->getAxis();
  EXPECT_DOUBLE_EQ(axis[0], 0.0);
  EXPECT_DOUBLE_EQ(axis[1], 0.0);
  EXPECT_DOUBLE_EQ(axis[2], 1.0);
}

// Test save/load preserves names
TEST(Serialization, PreservesNames)
{
  dart::simulation::experimental::World world1;
  auto mb = world1.addMultiBody("test_robot");
  [[maybe_unused]] auto base = mb.addLink("base_link");
  [[maybe_unused]] auto link = mb.addLink(
      "arm_link", {.parentLink = base, .jointName = "shoulder_joint"});

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  auto mb_restored = world2.getMultiBody("test_robot");
  ASSERT_TRUE(mb_restored.has_value());
  EXPECT_EQ(mb_restored->getName(), "test_robot");
}

// Test save/load with rigid bodies
TEST(Serialization, WithRigidBodies)
{
  dart::simulation::experimental::World world1;
  [[maybe_unused]] auto rb1 = world1.addRigidBody("box1");
  [[maybe_unused]] auto rb2 = world1.addRigidBody("box2");

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  EXPECT_EQ(world2.getRigidBodyCount(), 2);
  EXPECT_TRUE(world2.hasRigidBody("box1"));
  EXPECT_TRUE(world2.hasRigidBody("box2"));
}

// Test save/load preserves simulation mode
TEST(Serialization, PreservesSimulationMode)
{
  dart::simulation::experimental::World world1;
  world1.addMultiBody("robot");
  world1.enterSimulationMode();
  ASSERT_TRUE(world1.isSimulationMode());

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  EXPECT_TRUE(world2.isSimulationMode());
}

// Test save/load preserves auto-generation counters
TEST(Serialization, PreservesCounters)
{
  dart::simulation::experimental::World world1;
  [[maybe_unused]] auto mb1 = world1.addMultiBody("");
  [[maybe_unused]] auto mb2 = world1.addMultiBody("");

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  auto mb3 = world2.addMultiBody("");
  EXPECT_EQ(mb3.getName(), "multibody_003");
}

// Test multiple save/load cycles
TEST(Serialization, MultipleCycles)
{
  dart::simulation::experimental::World world1;
  auto mb = world1.addMultiBody("robot");
  mb.addLink("base");

  for (int i = 0; i < 3; ++i) {
    std::stringstream ss;
    world1.saveBinary(ss);

    dart::simulation::experimental::World world2;
    world2.loadBinary(ss);

    std::stringstream ss2;
    world2.saveBinary(ss2);
    world1.loadBinary(ss2);

    EXPECT_EQ(world1.getMultiBodyCount(), 1);
    auto mb_restored = world1.getMultiBody("robot");
    ASSERT_TRUE(mb_restored.has_value());
    EXPECT_EQ(mb_restored->getLinkCount(), 1);
  }
}

// Test load clears existing state
TEST(Serialization, LoadClearsExisting)
{
  dart::simulation::experimental::World world1;
  world1.addMultiBody("robot1");

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.addMultiBody("robot2");
  world2.addMultiBody("robot3");
  EXPECT_EQ(world2.getMultiBodyCount(), 2);

  world2.loadBinary(ss);
  EXPECT_EQ(world2.getMultiBodyCount(), 1);
  EXPECT_TRUE(world2.getMultiBody("robot1").has_value());
  EXPECT_FALSE(world2.getMultiBody("robot2").has_value());
  EXPECT_FALSE(world2.getMultiBody("robot3").has_value());
}

// Test saves and restores joint type (REVOLUTE)
TEST(Serialization, JointTypeRevolute)
{
  dart::simulation::experimental::World world1;
  auto mb = world1.addMultiBody("robot");
  [[maybe_unused]] auto base = mb.addLink("base");
  [[maybe_unused]] auto link1 = mb.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "j1",
       .jointType = dart::simulation::experimental::comps::JointType::Revolute,
       .axis = {0, 0, 1}});

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  [[maybe_unused]] auto mb2 = world2.getMultiBody("robot");
  ASSERT_TRUE(mb2.has_value());
  EXPECT_EQ(mb2->getJointCount(), 1);
}

// Test saves and restores joint type (PRISMATIC)
TEST(Serialization, JointTypePrismatic)
{
  dart::simulation::experimental::World world1;
  auto mb = world1.addMultiBody("robot");
  [[maybe_unused]] auto base = mb.addLink("base");
  [[maybe_unused]] auto link1 = mb.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "j1",
       .jointType = dart::simulation::experimental::comps::JointType::Prismatic,
       .axis = {1, 0, 0}});

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  [[maybe_unused]] auto mb2 = world2.getMultiBody("robot");
  ASSERT_TRUE(mb2.has_value());
  EXPECT_EQ(mb2->getJointCount(), 1);
}

// Test complex hierarchy (6-DOF manipulator)
TEST(Serialization, ComplexHierarchy)
{
  dart::simulation::experimental::World world1;
  auto robot = world1.addMultiBody("ur5");
  [[maybe_unused]] auto base = robot.addLink("base");
  [[maybe_unused]] auto link1 = robot.addLink(
      "shoulder_link",
      {.parentLink = base,
       .jointName = "shoulder_pan",
       .jointType = dart::simulation::experimental::comps::JointType::Revolute,
       .axis = {0, 0, 1}});
  auto link2 = robot.addLink(
      "upper_arm_link",
      {.parentLink = link1,
       .jointName = "shoulder_lift",
       .jointType = dart::simulation::experimental::comps::JointType::Revolute,
       .axis = {0, 1, 0}});
  auto link3 = robot.addLink(
      "forearm_link",
      {.parentLink = link2,
       .jointName = "elbow",
       .jointType = dart::simulation::experimental::comps::JointType::Revolute,
       .axis = {0, 1, 0}});
  auto link4 = robot.addLink(
      "wrist_1_link",
      {.parentLink = link3,
       .jointName = "wrist_1",
       .jointType = dart::simulation::experimental::comps::JointType::Revolute,
       .axis = {0, 1, 0}});
  auto link5 = robot.addLink(
      "wrist_2_link",
      {.parentLink = link4,
       .jointName = "wrist_2",
       .jointType = dart::simulation::experimental::comps::JointType::Revolute,
       .axis = {0, 0, 1}});
  [[maybe_unused]] auto link6 = robot.addLink(
      "wrist_3_link",
      {.parentLink = link5,
       .jointName = "wrist_3",
       .jointType = dart::simulation::experimental::comps::JointType::Revolute,
       .axis = {0, 1, 0}});

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  auto robot_restored = world2.getMultiBody("ur5");
  ASSERT_TRUE(robot_restored.has_value());
  EXPECT_EQ(robot_restored->getLinkCount(), 7);
  EXPECT_EQ(robot_restored->getJointCount(), 6);

  // Verify all link names are preserved
  EXPECT_TRUE(robot_restored->getLink("base").has_value());
  EXPECT_TRUE(robot_restored->getLink("shoulder_link").has_value());
  EXPECT_TRUE(robot_restored->getLink("upper_arm_link").has_value());
  EXPECT_TRUE(robot_restored->getLink("forearm_link").has_value());
  EXPECT_TRUE(robot_restored->getLink("wrist_1_link").has_value());
  EXPECT_TRUE(robot_restored->getLink("wrist_2_link").has_value());
  EXPECT_TRUE(robot_restored->getLink("wrist_3_link").has_value());

  // Verify all joints and their properties
  auto shoulder_pan = robot_restored->getJoint("shoulder_pan");
  ASSERT_TRUE(shoulder_pan.has_value());
  EXPECT_EQ(
      shoulder_pan->getType(),
      dart::simulation::experimental::comps::JointType::Revolute);
  auto axis1 = shoulder_pan->getAxis();
  EXPECT_DOUBLE_EQ(axis1[0], 0.0);
  EXPECT_DOUBLE_EQ(axis1[1], 0.0);
  EXPECT_DOUBLE_EQ(axis1[2], 1.0);

  auto shoulder_lift = robot_restored->getJoint("shoulder_lift");
  ASSERT_TRUE(shoulder_lift.has_value());
  EXPECT_EQ(
      shoulder_lift->getType(),
      dart::simulation::experimental::comps::JointType::Revolute);
  auto axis2 = shoulder_lift->getAxis();
  EXPECT_DOUBLE_EQ(axis2[0], 0.0);
  EXPECT_DOUBLE_EQ(axis2[1], 1.0);
  EXPECT_DOUBLE_EQ(axis2[2], 0.0);

  auto elbow = robot_restored->getJoint("elbow");
  ASSERT_TRUE(elbow.has_value());
  EXPECT_EQ(
      elbow->getType(),
      dart::simulation::experimental::comps::JointType::Revolute);
  auto axis3 = elbow->getAxis();
  EXPECT_DOUBLE_EQ(axis3[0], 0.0);
  EXPECT_DOUBLE_EQ(axis3[1], 1.0);
  EXPECT_DOUBLE_EQ(axis3[2], 0.0);

  auto wrist_1 = robot_restored->getJoint("wrist_1");
  ASSERT_TRUE(wrist_1.has_value());
  EXPECT_EQ(
      wrist_1->getType(),
      dart::simulation::experimental::comps::JointType::Revolute);

  auto wrist_2 = robot_restored->getJoint("wrist_2");
  ASSERT_TRUE(wrist_2.has_value());
  EXPECT_EQ(
      wrist_2->getType(),
      dart::simulation::experimental::comps::JointType::Revolute);
  auto axis5 = wrist_2->getAxis();
  EXPECT_DOUBLE_EQ(axis5[0], 0.0);
  EXPECT_DOUBLE_EQ(axis5[1], 0.0);
  EXPECT_DOUBLE_EQ(axis5[2], 1.0);

  auto wrist_3 = robot_restored->getJoint("wrist_3");
  ASSERT_TRUE(wrist_3.has_value());
  EXPECT_EQ(
      wrist_3->getType(),
      dart::simulation::experimental::comps::JointType::Revolute);
  auto axis6 = wrist_3->getAxis();
  EXPECT_DOUBLE_EQ(axis6[0], 0.0);
  EXPECT_DOUBLE_EQ(axis6[1], 1.0);
  EXPECT_DOUBLE_EQ(axis6[2], 0.0);
}

// Test multiple multibodies
TEST(Serialization, MultipleMultiBodies)
{
  dart::simulation::experimental::World world1;
  [[maybe_unused]] auto mb1 = world1.addMultiBody("robot1");
  [[maybe_unused]] auto mb2 = world1.addMultiBody("robot2");
  auto mb3 = world1.addMultiBody("robot3");

  mb1.addLink("base1");
  mb2.addLink("base2");
  mb3.addLink("base3");

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  EXPECT_EQ(world2.getMultiBodyCount(), 3);
  EXPECT_TRUE(world2.getMultiBody("robot1").has_value());
  EXPECT_TRUE(world2.getMultiBody("robot2").has_value());
  EXPECT_TRUE(world2.getMultiBody("robot3").has_value());
}

// Test mixed multibodies and rigid bodies
TEST(Serialization, MixedContent)
{
  dart::simulation::experimental::World world1;

  auto mb = world1.addMultiBody("robot");
  mb.addLink("base");

  [[maybe_unused]] auto rb1 = world1.addRigidBody("box1");
  [[maybe_unused]] auto rb2 = world1.addRigidBody("box2");

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  EXPECT_EQ(world2.getMultiBodyCount(), 1);
  EXPECT_EQ(world2.getRigidBodyCount(), 2);
  EXPECT_TRUE(world2.getMultiBody("robot").has_value());
  EXPECT_TRUE(world2.hasRigidBody("box1"));
  EXPECT_TRUE(world2.hasRigidBody("box2"));
}

// Test auto-generated names are preserved
TEST(Serialization, EmptyNames)
{
  dart::simulation::experimental::World world1;
  auto mb = world1.addMultiBody("");
  auto base = mb.addLink("");
  dart::simulation::experimental::LinkOptions opts{
      .parentLink = base, .jointName = ""};
  [[maybe_unused]] auto child = mb.addLink("", opts);

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  auto restoredMb = world2.getMultiBody("multibody_001");
  ASSERT_TRUE(restoredMb.has_value());
  EXPECT_EQ(restoredMb->getName(), "multibody_001");

  auto restoredBase = restoredMb->getLink("link_001");
  ASSERT_TRUE(restoredBase.has_value());
  EXPECT_EQ(restoredBase->getName(), "link_001");

  auto restoredChild = restoredMb->getLink("link_002");
  ASSERT_TRUE(restoredChild.has_value());
  EXPECT_EQ(restoredChild->getName(), "link_002");

  auto restoredJoint = restoredMb->getJoint("joint_001");
  ASSERT_TRUE(restoredJoint.has_value());
  EXPECT_EQ(restoredJoint->getName(), "joint_001");
}

// Test large hierarchy (stress test)
TEST(Serialization, LargeHierarchy)
{
  dart::simulation::experimental::World world1;
  auto robot = world1.addMultiBody("large_robot");

  // Create chain of 20 links
  auto prev = robot.addLink("base");
  for (int i = 1; i < 20; ++i) {
    prev = robot.addLink(
        "link_" + std::to_string(i),
        {.parentLink = prev, .jointName = "joint_" + std::to_string(i)});
  }

  std::stringstream ss;
  world1.saveBinary(ss);

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  auto robot2 = world2.getMultiBody("large_robot");
  ASSERT_TRUE(robot2.has_value());
  EXPECT_EQ(robot2->getLinkCount(), 20);
  EXPECT_EQ(robot2->getJointCount(), 19);
}

//==============================================================================
// Properties/State/Cache Serialization Tests
//==============================================================================

// Test cache components are NOT serialized
TEST(Serialization, CacheNotSerialized)
{
  dart::simulation::experimental::World world;

  // Create a FreeFrame and set a transform
  auto frame = world.addFreeFrame("test");
  Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
  T1.translate(Eigen::Vector3d(10, 20, 30));
  frame.setLocalTransform(T1);

  // Trigger cache computation to populate worldTransform
  auto cachedTransform = frame.getTransform();
  EXPECT_TRUE(cachedTransform.isApprox(T1));

  // Verify cache is clean
  {
    auto& registry = world.getRegistry();
    auto entity = frame.getEntity();
    ASSERT_TRUE(registry.valid(entity)) << "Entity should be valid";
    ASSERT_TRUE(
        registry.all_of<dart::simulation::experimental::comps::FrameCache>(
            entity))
        << "Entity should have FrameCache component";

    const auto& cache
        = registry.get<dart::simulation::experimental::comps::FrameCache>(
            entity);
    EXPECT_FALSE(cache.needTransformUpdate)
        << "Cache should be clean after getTransform()";
  }

  // Serialize
  std::stringstream ss;
  world.saveBinary(ss);

  // Deserialize into new world
  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  // Check the registry directly for the restored FreeFrame
  auto& registry2 = world2.getRegistry();
  auto view
      = registry2.view<dart::simulation::experimental::comps::FreeFrameTag>();

  ASSERT_FALSE(view.empty()) << "Should have restored FreeFrame";

  auto entity2 = *view.begin();

  ASSERT_TRUE(registry2.valid(entity2)) << "Restored entity should be valid";
  ASSERT_TRUE(
      registry2.all_of<dart::simulation::experimental::comps::FrameCache>(
          entity2))
      << "Restored entity should have FrameCache component";

  const auto& cache2
      = registry2.get<dart::simulation::experimental::comps::FrameCache>(
          entity2);

  // CRITICAL: Cache should be dirty after load (not serialized)
  EXPECT_TRUE(cache2.needTransformUpdate)
      << "Cache should be dirty after load - proves it was not serialized";

  // Verify state WAS serialized (parent should be entt::null)
  ASSERT_TRUE(
      registry2.all_of<dart::simulation::experimental::comps::FrameState>(
          entity2))
      << "Restored entity should have FrameState component";

  const auto& state2
      = registry2.get<dart::simulation::experimental::comps::FrameState>(
          entity2);
  EXPECT_FALSE(registry2.valid(state2.parentFrame))
      << "State should be serialized correctly (parent should be null)";

  // Verify properties WAS serialized (local transform should match)
  ASSERT_TRUE(
      registry2
          .all_of<dart::simulation::experimental::comps::FreeFrameProperties>(
              entity2))
      << "Restored entity should have FreeFrameProperties component";

  const auto& props2
      = registry2
            .get<dart::simulation::experimental::comps::FreeFrameProperties>(
                entity2);
  EXPECT_TRUE(props2.localTransform.isApprox(T1))
      << "Properties should be serialized correctly";
}

// Test state components ARE serialized
// TODO(serialization): Re-enable parent relationship testing once name-based
// entity reference system is implemented. Currently disabled because entity IDs
// change during deserialization, making entity references invalid.
TEST(Serialization, StateSerializedCorrectly)
{
  dart::simulation::experimental::World world;

  // Create parent and child frames
  auto parent = world.addFreeFrame("parent");
  auto child = world.addFreeFrame("child", parent);

  // Set transforms
  Eigen::Isometry3d T_parent = Eigen::Isometry3d::Identity();
  T_parent.translate(Eigen::Vector3d(5, 0, 0));
  parent.setLocalTransform(T_parent);

  Eigen::Isometry3d T_child = Eigen::Isometry3d::Identity();
  T_child.translate(Eigen::Vector3d(0, 10, 0));
  child.setLocalTransform(T_child);

  // Serialize
  std::stringstream ss;
  world.saveBinary(ss);

  // Deserialize
  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  // Check that we have 2 FreeFrames
  auto& registry2 = world2.getRegistry();
  auto view
      = registry2.view<dart::simulation::experimental::comps::FreeFrameTag>();
  EXPECT_EQ(std::distance(view.begin(), view.end()), 2)
      << "Should have restored 2 FreeFrames";

  // Verify that FrameState components exist
  for (auto entity : view) {
    // All FreeFrames should have FrameState component
    EXPECT_TRUE(
        registry2.all_of<dart::simulation::experimental::comps::FrameState>(
            entity))
        << "All FreeFrames should have FrameState after deserialization";
  }

  // Verify parent relationships are preserved after serialization
  // Count how many have valid parent references
  int framesWithParent = 0;
  int framesWithoutParent = 0;

  for (auto entity : view) {
    const auto& state
        = registry2.get<dart::simulation::experimental::comps::FrameState>(
            entity);
    if (registry2.valid(state.parentFrame)) {
      framesWithParent++;
    } else {
      framesWithoutParent++;
    }
  }

  // We should have 1 frame with parent (child) and 1 without (parent attached
  // to world)
  EXPECT_EQ(framesWithParent, 1)
      << "Should have 1 frame with parent relationship";
  EXPECT_EQ(framesWithoutParent, 1)
      << "Should have 1 frame without parent (attached to world)";
}

// Test properties ARE serialized
TEST(Serialization, PropertiesSerializedCorrectly)
{
  dart::simulation::experimental::World world;

  // Create FixedFrame with specific offset
  auto parent = world.addFreeFrame("parent");
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translate(Eigen::Vector3d(3.14, 2.71, 1.41));
  offset.rotate(
      Eigen::AngleAxisd(
          dart::simulation::experimental::pi / 4, Eigen::Vector3d(1, 0, 0)));

  auto fixed = world.addFixedFrame("fixed", parent, offset);

  // Serialize
  std::stringstream ss;
  world.saveBinary(ss);

  // Deserialize
  dart::simulation::experimental::World world2;
  world2.loadBinary(ss);

  // Find the FixedFrame
  auto& registry2 = world2.getRegistry();
  auto view
      = registry2.view<dart::simulation::experimental::comps::FixedFrameTag>();
  EXPECT_FALSE(view.empty()) << "Should have restored FixedFrame";

  auto entity = *view.begin();
  const auto& props
      = registry2
            .get<dart::simulation::experimental::comps::FixedFrameProperties>(
                entity);

  // Properties should match exactly
  EXPECT_TRUE(props.localTransform.isApprox(offset, 1e-10))
      << "Properties (localTransform) should be serialized correctly";
}

// Test round-trip consistency (save twice, should be identical)
TEST(Serialization, RoundTripConsistency)
{
  dart::simulation::experimental::World world;

  auto parent = world.addFreeFrame("parent");
  Eigen::Isometry3d T_parent = Eigen::Isometry3d::Identity();
  T_parent.translate(Eigen::Vector3d(1, 2, 3));
  parent.setLocalTransform(T_parent);

  auto child = world.addFreeFrame("child", parent);
  Eigen::Isometry3d T_child = Eigen::Isometry3d::Identity();
  T_child.translate(Eigen::Vector3d(4, 5, 6));
  child.setLocalTransform(T_child);

  [[maybe_unused]] auto parentTransform = parent.getTransform();

  std::stringstream ss1;
  world.saveBinary(ss1);

  std::stringstream ss2;
  world.saveBinary(ss2);

  EXPECT_EQ(ss1.str(), ss2.str())
      << "Repeated saves should produce identical binary output";

  dart::simulation::experimental::World world2;
  world2.loadBinary(ss1);

  auto& registry2 = world2.getRegistry();
  auto view = registry2.view<
      dart::simulation::experimental::comps::Name,
      dart::simulation::experimental::comps::FreeFrameProperties,
      dart::simulation::experimental::comps::FrameState>();

  std::size_t frameCount = 0;
  for (auto entity : view) {
    const auto& name
        = view.get<dart::simulation::experimental::comps::Name>(entity).name;
    const auto& props
        = view.get<dart::simulation::experimental::comps::FreeFrameProperties>(
            entity);
    const auto& state
        = view.get<dart::simulation::experimental::comps::FrameState>(entity);

    if (name == "parent") {
      EXPECT_TRUE(props.localTransform.isApprox(T_parent));
      EXPECT_TRUE(state.parentFrame == entt::null);
      ++frameCount;
    } else if (name == "child") {
      EXPECT_TRUE(props.localTransform.isApprox(T_child));
      EXPECT_TRUE(state.parentFrame != entt::null);
      ++frameCount;
    }
  }

  EXPECT_EQ(frameCount, 2u);
}

//==============================================================================
// Cloning Tests (via serialization)
//==============================================================================

TEST(Serialization, CloneDeepCopy)
{
  dart::simulation::experimental::World world;

  auto parent = world.addFreeFrame("parent");
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translate(Eigen::Vector3d(1, 2, 3));
  parent.setLocalTransform(T);

  auto child = world.addFreeFrame("child", parent);
  Eigen::Isometry3d T_child = Eigen::Isometry3d::Identity();
  T_child.translate(Eigen::Vector3d(4, 5, 6));
  child.setLocalTransform(T_child);

  std::stringstream ss;
  world.saveBinary(ss);

  dart::simulation::experimental::World clone;
  clone.loadBinary(ss);

  auto& cloneReg = clone.getRegistry();
  auto cloneView = cloneReg.view<
      dart::simulation::experimental::comps::Name,
      dart::simulation::experimental::comps::FreeFrameProperties>();
  EXPECT_EQ(std::distance(cloneView.begin(), cloneView.end()), 2);

  for (auto entity : cloneView) {
    auto& props
        = cloneView
              .get<dart::simulation::experimental::comps::FreeFrameProperties>(
                  entity);
    const auto& name
        = cloneView.get<dart::simulation::experimental::comps::Name>(entity)
              .name;
    if (name == "parent") {
      EXPECT_TRUE(props.localTransform.isApprox(T));
    } else if (name == "child") {
      EXPECT_TRUE(props.localTransform.isApprox(T_child));
      props.localTransform.translation() = Eigen::Vector3d(100, 200, 300);
    }
  }

  const auto& originalReg = world.getRegistry();
  auto originalView = originalReg.view<
      dart::simulation::experimental::comps::Name,
      dart::simulation::experimental::comps::FreeFrameProperties>();
  for (auto entity : originalView) {
    const auto& props
        = originalView
              .get<dart::simulation::experimental::comps::FreeFrameProperties>(
                  entity);
    const auto& name
        = originalView.get<dart::simulation::experimental::comps::Name>(entity)
              .name;
    if (name == "child") {
      EXPECT_TRUE(props.localTransform.isApprox(T_child))
          << "Modifying clone should not affect original world";
    }
  }
}

TEST(Serialization, CloneResetCounters)
{
  dart::simulation::experimental::World world;
  auto frame1 = world.addFreeFrame(); // free_frame_001
  auto frame2 = world.addFreeFrame(); // free_frame_002
  (void)frame1;
  (void)frame2;
  [[maybe_unused]] auto mb1 = world.addMultiBody(""); // multibody_001

  std::stringstream ss;
  world.saveBinary(ss);

  dart::simulation::experimental::World clone;
  clone.loadBinary(ss);

  auto nextFrame = clone.addFreeFrame();
  auto& cloneReg = clone.getRegistry();
  const auto& nextFrameName
      = cloneReg
            .get<dart::simulation::experimental::comps::Name>(
                nextFrame.getEntity())
            .name;
  EXPECT_EQ(nextFrameName, "free_frame_003");

  auto nextMb = clone.addMultiBody("");
  EXPECT_EQ(nextMb.getName(), "multibody_002");
}

//==============================================================================
// Golden/Stability Tests - Format Regression Detection
//==============================================================================

TEST(SerializationGolden, DeterministicOutput)
{
  auto createWorld = []() {
    dart::simulation::experimental::World world;
    auto robot = world.addMultiBody("test_robot");
    auto base = robot.addLink("base");
    robot.addLink(
        "link1",
        {.parentLink = base,
         .jointName = "joint1",
         .jointType
         = dart::simulation::experimental::comps::JointType::Revolute,
         .axis = {0, 0, 1}});
    return world;
  };

  auto world1 = createWorld();
  auto world2 = createWorld();

  std::stringstream ss1, ss2;
  world1.saveBinary(ss1);
  world2.saveBinary(ss2);

  EXPECT_EQ(ss1.str(), ss2.str())
      << "Identical worlds must produce identical binary output";
}

TEST(SerializationGolden, FormatVersionPresent)
{
  dart::simulation::experimental::World world;
  world.addMultiBody("robot");

  std::stringstream ss;
  world.saveBinary(ss);

  std::string data = ss.str();
  ASSERT_GE(data.size(), sizeof(std::uint32_t))
      << "Binary output must contain at least the version header";

  std::uint32_t version;
  std::memcpy(&version, data.data(), sizeof(version));
  EXPECT_EQ(version, 1u) << "Current format version should be 1";
}

TEST(SerializationGolden, MinimumWorldSize)
{
  dart::simulation::experimental::World emptyWorld;

  std::stringstream ss;
  emptyWorld.saveBinary(ss);

  std::string data = ss.str();
  EXPECT_GE(data.size(), 4u) << "Even empty world must have version header";
  EXPECT_LE(data.size(), 1024u) << "Empty world should be compact (< 1KB)";
}

TEST(SerializationGolden, LargeWorldStressTest)
{
  dart::simulation::experimental::World world;

  for (int r = 0; r < 100; ++r) {
    auto robot = world.addMultiBody("robot_" + std::to_string(r));
    auto prev = robot.addLink("base");
    for (int l = 1; l < 7; ++l) {
      prev = robot.addLink(
          "link_" + std::to_string(l),
          {.parentLink = prev,
           .jointName = "joint_" + std::to_string(l),
           .jointType
           = dart::simulation::experimental::comps::JointType::Revolute,
           .axis = {0, 0, 1}});
    }
  }

  std::stringstream ss;
  world.saveBinary(ss);

  dart::simulation::experimental::World restored;
  restored.loadBinary(ss);

  EXPECT_EQ(restored.getMultiBodyCount(), 100u);
  auto robot50 = restored.getMultiBody("robot_50");
  ASSERT_TRUE(robot50.has_value());
  EXPECT_EQ(robot50->getLinkCount(), 7u);
  EXPECT_EQ(robot50->getJointCount(), 6u);
}

TEST(SerializationGolden, AllJointTypesRoundTrip)
{
  dart::simulation::experimental::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");

  using JT = dart::simulation::experimental::comps::JointType;

  auto l1 = robot.addLink(
      "l1",
      {.parentLink = base, .jointName = "j_fixed", .jointType = JT::Fixed});
  auto l2 = robot.addLink(
      "l2",
      {.parentLink = l1,
       .jointName = "j_revolute",
       .jointType = JT::Revolute,
       .axis = {1, 0, 0}});
  auto l3 = robot.addLink(
      "l3",
      {.parentLink = l2,
       .jointName = "j_prismatic",
       .jointType = JT::Prismatic,
       .axis = {0, 1, 0}});
  auto l4 = robot.addLink(
      "l4",
      {.parentLink = l3,
       .jointName = "j_screw",
       .jointType = JT::Screw,
       .axis = {0, 0, 1}});
  auto l5 = robot.addLink(
      "l5",
      {.parentLink = l4,
       .jointName = "j_universal",
       .jointType = JT::Universal,
       .axis = {1, 0, 0}});
  auto l6 = robot.addLink(
      "l6", {.parentLink = l5, .jointName = "j_ball", .jointType = JT::Ball});
  auto l7 = robot.addLink(
      "l7",
      {.parentLink = l6,
       .jointName = "j_planar",
       .jointType = JT::Planar,
       .axis = {0, 0, 1}});
  [[maybe_unused]] auto l8 = robot.addLink(
      "l8", {.parentLink = l7, .jointName = "j_free", .jointType = JT::Free});

  std::stringstream ss;
  world.saveBinary(ss);

  dart::simulation::experimental::World restored;
  restored.loadBinary(ss);

  auto restoredRobot = restored.getMultiBody("robot");
  ASSERT_TRUE(restoredRobot.has_value());
  EXPECT_EQ(restoredRobot->getJointCount(), 8u);

  auto checkJoint = [&](const std::string& name, JT expectedType) {
    auto joint = restoredRobot->getJoint(name);
    ASSERT_TRUE(joint.has_value()) << "Joint " << name << " not found";
    EXPECT_EQ(joint->getType(), expectedType)
        << "Joint " << name << " has wrong type";
  };

  checkJoint("j_fixed", JT::Fixed);
  checkJoint("j_revolute", JT::Revolute);
  checkJoint("j_prismatic", JT::Prismatic);
  checkJoint("j_screw", JT::Screw);
  checkJoint("j_universal", JT::Universal);
  checkJoint("j_ball", JT::Ball);
  checkJoint("j_planar", JT::Planar);
  checkJoint("j_free", JT::Free);
}
