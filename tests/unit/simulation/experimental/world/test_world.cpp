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
#include <dart/simulation/experimental/common/exceptions.hpp>
#include <dart/simulation/experimental/comps/joint.hpp>
#include <dart/simulation/experimental/multi_body/multi_body.hpp>
#include <dart/simulation/experimental/version.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <gtest/gtest.h>

namespace dse = dart::simulation::experimental;

// Test World construction
TEST(World, Construction)
{
  // Create a World instance
  dart::simulation::experimental::World world;
  (void)world;       // Suppress unused variable warning
  EXPECT_TRUE(true); // If we get here, construction succeeded
}

// Test version information
TEST(Version, VersionString)
{
  // Test that version() returns a valid string_view
  auto ver = dart::simulation::experimental::version();
  EXPECT_FALSE(ver.empty());
  EXPECT_GT(ver.size(), 0u);

  // Test individual version components
  EXPECT_GE(dart::simulation::experimental::versionMajor(), 7);
  EXPECT_GE(dart::simulation::experimental::versionMinor(), 0);
  EXPECT_GE(dart::simulation::experimental::versionPatch(), 0);
}

//==============================================================================
// Mode Control Tests (Design Mode vs Simulation Mode)
//==============================================================================

// Test default mode is design mode
TEST(World, DefaultModeIsDesign)
{
  dart::simulation::experimental::World world;
  EXPECT_FALSE(world.isSimulationMode());
}

// Test entering simulation mode
TEST(World, EnterSimulationMode)
{
  dart::simulation::experimental::World world;

  // Initially in design mode
  EXPECT_FALSE(world.isSimulationMode());

  // Enter simulation mode
  world.enterSimulationMode();

  // Now in simulation mode
  EXPECT_TRUE(world.isSimulationMode());
}

// Test cannot enter simulation mode twice
TEST(World, CannotEnterSimulationModeTwice)
{
  dart::simulation::experimental::World world;

  // First call succeeds
  world.enterSimulationMode();
  EXPECT_TRUE(world.isSimulationMode());

  // Second call should throw
  EXPECT_THROW(
      world.enterSimulationMode(),
      dart::simulation::experimental::InvalidArgumentException);
}

// Test baking with empty world
TEST(World, BakingEmptyWorld)
{
  dart::simulation::experimental::World world;

  // Baking should work even with empty world
  EXPECT_NO_THROW(world.enterSimulationMode());
  EXPECT_TRUE(world.isSimulationMode());
}

// Test baking with multibodies
TEST(World, BakingWithMultibodies)
{
  dart::simulation::experimental::World world;

  // Create several multibodies with joints and links
  auto robot1 = world.addMultiBody("robot1");
  auto base1 = robot1.addLink("base");
  (void)robot1.addLink("link1", {.parentLink = base1, .jointName = "joint1"});

  auto robot2 = world.addMultiBody("robot2");
  auto base2 = robot2.addLink("base");
  auto link2
      = robot2.addLink("link1", {.parentLink = base2, .jointName = "joint1"});
  (void)robot2.addLink(
      "link2",
      {.parentLink = link2,
       .jointName = "joint2",
       .jointType
       = dart::simulation::experimental::comps::JointType::Prismatic});

  // Baking should succeed
  EXPECT_NO_THROW(world.enterSimulationMode());
  EXPECT_TRUE(world.isSimulationMode());

  // Counts should remain the same
  EXPECT_EQ(world.getMultiBodyCount(), 2u);
  EXPECT_EQ(robot1.getLinkCount(), 2u);
  EXPECT_EQ(robot1.getJointCount(), 1u);
  EXPECT_EQ(robot2.getLinkCount(), 3u);
  EXPECT_EQ(robot2.getJointCount(), 2u);
}

// Test that simulation operations require simulation mode
TEST(World, UpdateKinematicsRequiresSimulationMode)
{
  dart::simulation::experimental::World world;

  // updateKinematics should throw in design mode
  EXPECT_THROW(
      world.updateKinematics(),
      dart::simulation::experimental::InvalidArgumentException);

  // After entering simulation mode, should work
  world.enterSimulationMode();
  EXPECT_NO_THROW(world.updateKinematics());
}

TEST(World, StepRequiresSimulationMode)
{
  dse::World world;

  EXPECT_THROW(world.step(), dse::InvalidArgumentException);

  world.enterSimulationMode();
  EXPECT_NO_THROW(world.step());
}

TEST(World, Clear_ResetsContainersAndMode)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  (void)robot.addLink("base");
  (void)world.addRigidBody("box");

  world.enterSimulationMode();
  EXPECT_TRUE(world.isSimulationMode());

  world.clear();

  EXPECT_FALSE(world.isSimulationMode());
  EXPECT_EQ(world.getMultiBodyCount(), 0u);
  EXPECT_EQ(world.getRigidBodyCount(), 0u);
  EXPECT_FALSE(world.getMultiBody("robot").has_value());

  auto autoNamedMb = world.addMultiBody("");
  auto autoNamedRb = world.addRigidBody("");
  EXPECT_EQ(autoNamedMb.getName(), "multibody_001");
  EXPECT_EQ(autoNamedRb.getName(), "rigid_body_001");

  EXPECT_THROW(world.step(), dse::InvalidArgumentException);
}

TEST(World, GetMultiBody_NonExistentName_ReturnsNullopt)
{
  dse::World world;
  (void)world.addMultiBody("existing");

  auto missing = world.getMultiBody("does_not_exist");
  EXPECT_FALSE(missing.has_value());
}

TEST(World, StepAdvancesTime)
{
  dart::simulation::experimental::World world;
  world.setTimeStep(0.01);
  world.enterSimulationMode();

  EXPECT_DOUBLE_EQ(world.getTime(), 0.0);
  EXPECT_EQ(world.getFrame(), 0u);

  world.step();
  EXPECT_DOUBLE_EQ(world.getTime(), 0.01);
  EXPECT_EQ(world.getFrame(), 1u);

  world.step();
  EXPECT_DOUBLE_EQ(world.getTime(), 0.02);
  EXPECT_EQ(world.getFrame(), 2u);
}

TEST(World, FreeFallDynamics)
{
  namespace dse = dart::simulation::experimental;

  dse::World world;
  world.setGravity(Eigen::Vector3d(0, 0, -10.0));
  world.setTimeStep(0.001);

  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");
  auto slider = robot.addLink(
      "slider",
      {.parentLink = base,
       .jointName = "joint",
       .jointType = dse::comps::JointType::Prismatic,
       .axis = Eigen::Vector3d::UnitZ()});

  slider.setMass(1.0);

  world.enterSimulationMode();

  constexpr int numSteps = 100;
  constexpr double dt = 0.001;
  constexpr double g = 10.0;

  for (int i = 0; i < numSteps; ++i) {
    world.step();
  }

  double t = numSteps * dt;
  double expectedVel = -g * t;
  double expectedPos = -0.5 * g * t * t;

  auto joint = slider.getParentJoint();
  double actualVel = joint.getVelocity()(0);
  double actualPos = joint.getPosition()(0);

  EXPECT_NEAR(actualVel, expectedVel, 0.01);
  EXPECT_NEAR(actualPos, expectedPos, 0.001);
}

TEST(World, ExternalForceIntegration)
{
  dse::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.001);

  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");
  auto slider = robot.addLink(
      "slider",
      {.parentLink = base,
       .jointName = "joint",
       .jointType = dse::comps::JointType::Prismatic,
       .axis = Eigen::Vector3d::UnitX()});

  constexpr double mass = 2.0;
  constexpr double force = 10.0;
  slider.setMass(mass);

  world.enterSimulationMode();

  for (int i = 0; i < 100; ++i) {
    slider.addExternalForce(Eigen::Vector3d(force, 0, 0));
    world.step();
  }

  double t = 100 * 0.001;
  double expectedAccel = force / mass;
  double expectedVel = expectedAccel * t;
  double expectedPos = 0.5 * expectedAccel * t * t;

  auto joint = slider.getParentJoint();
  EXPECT_NEAR(joint.getVelocity()(0), expectedVel, 0.01);
  EXPECT_NEAR(joint.getPosition()(0), expectedPos, 0.001);
}

TEST(World, Step_ZeroGravityEnergy_DoesNotGrow)
{
  dse::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.001);

  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");
  auto slider = robot.addLink(
      "slider",
      {.parentLink = base,
       .jointName = "joint",
       .jointType = dse::comps::JointType::Prismatic,
       .axis = Eigen::Vector3d::UnitX()});

  constexpr double mass = 2.0;
  constexpr double initialVelocity = 3.0;
  slider.setMass(mass);

  Eigen::VectorXd vel(1);
  vel << initialVelocity;
  slider.getParentJoint().setVelocity(vel);

  world.enterSimulationMode();

  const double initialEnergy = 0.5 * mass * initialVelocity * initialVelocity;
  for (int i = 0; i < 500; ++i) {
    world.step();
    const double currentVelocity = slider.getParentJoint().getVelocity()(0);
    const double currentEnergy = 0.5 * mass * currentVelocity * currentVelocity;
    EXPECT_LE(currentEnergy, initialEnergy + 1e-8);
  }

  const double finalVelocity = slider.getParentJoint().getVelocity()(0);
  const double finalEnergy = 0.5 * mass * finalVelocity * finalVelocity;
  EXPECT_NEAR(finalEnergy, initialEnergy, 1e-8);
}
