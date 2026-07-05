/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License
 */

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/multibody/multibody.hpp>
#include <dart/simulation/world.hpp>
#include <dart/simulation/world_options.hpp>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <string_view>

namespace {

namespace sx = dart::simulation;

sx::RigidBody addDynamicBox(
    sx::World& world,
    std::string_view name,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& linearVelocity = Eigen::Vector3d::Zero())
{
  sx::RigidBodyOptions options;
  options.position = position;
  options.linearVelocity = linearVelocity;
  auto body = world.addRigidBody(name, options);
  body.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.2, 0.2, 0.2)));
  return body;
}

sx::RigidBody addStaticGround(sx::World& world)
{
  sx::RigidBodyOptions options;
  options.isStatic = true;
  options.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", options);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));
  return ground;
}

sx::DeactivationOptions fastSleepOptions()
{
  sx::DeactivationOptions options;
  options.enabled = true;
  options.timeUntilSleep = 0.03;
  options.linearSpeedThreshold = 1e-3;
  options.angularSpeedThreshold = 1e-3;
  options.generalizedSpeedThreshold = 1e-3;
  return options;
}

TEST(Deactivation, DefaultOffKeepsRestingBodyAwake)
{
  sx::World world;
  world.setTimeStep(0.01);
  world.setGravity(Eigen::Vector3d::Zero());
  addStaticGround(world);
  auto box = addDynamicBox(world, "box", Eigen::Vector3d(0.0, 0.0, 0.19));

  EXPECT_FALSE(world.isDeactivationEnabled());
  for (int i = 0; i < 10; ++i) {
    world.step();
  }

  EXPECT_FALSE(box.isSleeping());
  EXPECT_EQ(box.getDeactivationGroupIndex(), -1);
}

TEST(Deactivation, RestingRigidBodySleepsAndFreezes)
{
  sx::World world;
  world.setTimeStep(0.01);
  world.setGravity(Eigen::Vector3d::Zero());
  world.setDeactivationOptions(fastSleepOptions());
  addStaticGround(world);
  auto box = addDynamicBox(world, "box", Eigen::Vector3d(0.0, 0.0, 0.19));

  for (int i = 0; i < 6; ++i) {
    world.step();
  }

  ASSERT_TRUE(box.isSleeping());
  EXPECT_GE(box.getDeactivationGroupIndex(), 0);
  EXPECT_TRUE(box.getLinearVelocity().isZero(1e-12));
  EXPECT_TRUE(box.getAngularVelocity().isZero(1e-12));

  const Eigen::Vector3d frozenPosition = box.getTransform().translation();
  for (int i = 0; i < 5; ++i) {
    world.step();
  }

  EXPECT_TRUE(box.isSleeping());
  EXPECT_TRUE(box.getTransform().translation().isApprox(frozenPosition, 1e-12));
}

TEST(Deactivation, ExternalForceWakesSleepingRigidBody)
{
  sx::World world;
  world.setTimeStep(0.01);
  world.setGravity(Eigen::Vector3d::Zero());
  world.setDeactivationOptions(fastSleepOptions());
  addStaticGround(world);
  auto box = addDynamicBox(world, "box", Eigen::Vector3d(0.0, 0.0, 0.19));

  for (int i = 0; i < 6; ++i) {
    world.step();
  }
  ASSERT_TRUE(box.isSleeping());

  box.applyForce(Eigen::Vector3d(1.0, 0.0, 0.0));
  world.step();

  EXPECT_FALSE(box.isSleeping());
  EXPECT_GT(box.getLinearVelocity().x(), 0.0);
}

TEST(Deactivation, ContactWithActiveBodyWakesSleepingRigidBody)
{
  sx::World world;
  world.setTimeStep(0.01);
  world.setGravity(Eigen::Vector3d::Zero());
  world.setDeactivationOptions(fastSleepOptions());
  auto target = addDynamicBox(world, "target", Eigen::Vector3d::Zero());
  auto striker = addDynamicBox(
      world,
      "striker",
      Eigen::Vector3d(-1.0, 0.0, 0.0),
      Eigen::Vector3d(2.0, 0.0, 0.0));

  for (int i = 0; i < 6; ++i) {
    world.step();
  }
  ASSERT_TRUE(target.isSleeping());
  ASSERT_FALSE(striker.isSleeping());

  bool woke = false;
  for (int i = 0; i < 100; ++i) {
    world.step();
    if (!target.isSleeping()) {
      woke = true;
      break;
    }
  }

  EXPECT_TRUE(woke);
  EXPECT_FALSE(target.isSleeping());
}

TEST(Deactivation, DisablingClearsSleepState)
{
  sx::World world;
  world.setTimeStep(0.01);
  world.setGravity(Eigen::Vector3d::Zero());
  sx::DeactivationOptions options = fastSleepOptions();
  world.setDeactivationOptions(options);
  auto box = addDynamicBox(world, "box", Eigen::Vector3d::Zero());

  for (int i = 0; i < 6; ++i) {
    world.step();
  }
  ASSERT_TRUE(box.isSleeping());

  options.enabled = false;
  world.setDeactivationOptions(options);

  EXPECT_FALSE(world.isDeactivationEnabled());
  EXPECT_FALSE(box.isSleeping());
  EXPECT_EQ(box.getDeactivationGroupIndex(), -1);
}

namespace {

// Builds a single-DOF prismatic robot ("base" -> "carriage") whose carriage
// slides along `axis`. The carriage is dynamic and starts at rest, so a
// semi-implicit multibody can be exercised by the deactivation path.
sx::Multibody addPrismaticRobot(
    sx::World& world,
    std::string_view name,
    const Eigen::Vector3d& axis,
    bool attachCollisionShape = false)
{
  auto robot = world.addMultibody(name);
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "rail";
  spec.type = sx::JointType::Prismatic;
  spec.axis = axis;
  auto carriage = robot.addLink("carriage", base, spec);
  carriage.setMass(1.0);
  if (attachCollisionShape) {
    carriage.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
  }
  return robot;
}

} // namespace

TEST(Deactivation, RestingRigidBodyOnGroundSleepsUnderGravity)
{
  // A gravity-loaded box settling into contact with the ground should sleep
  // once its contact-driven velocity jitter stays below the configured speed
  // band. Mirrors the DART 6 resting-scene use case from #2920.
  sx::World world;
  world.setTimeStep(0.01);
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  sx::DeactivationOptions options = fastSleepOptions();
  options.timeUntilSleep = 0.1;
  options.linearSpeedThreshold = 0.05;
  options.angularSpeedThreshold = 0.05;
  world.setDeactivationOptions(options);
  addStaticGround(world);
  auto box = addDynamicBox(world, "box", Eigen::Vector3d(0.0, 0.0, 0.2005));

  bool slept = false;
  for (int i = 0; i < 60; ++i) {
    world.step();
    if (box.isSleeping()) {
      slept = true;
      break;
    }
  }

  ASSERT_TRUE(slept);
  const Eigen::Vector3d frozen = box.getTransform().translation();
  for (int i = 0; i < 20; ++i) {
    world.step();
  }
  EXPECT_TRUE(box.isSleeping());
  EXPECT_TRUE(box.getTransform().translation().isApprox(frozen, 1e-9));
}

TEST(Deactivation, PenetratingContactDefersSleepUntilConverged)
{
  // A body that is already quiet but whose contact still penetrates beyond the
  // sleep tolerance must not be frozen mid-interpenetration; it may sleep only
  // once the penetration correction has converged. Mirrors the DART 6 (#2920)
  // contact penetration-convergence gate.
  sx::World world;
  world.setTimeStep(0.01);
  world.setGravity(Eigen::Vector3d::Zero());
  sx::DeactivationOptions options = fastSleepOptions();
  options.timeUntilSleep = 0.0; // a quiet body is a sleep candidate immediately
  world.setDeactivationOptions(options);
  addStaticGround(world);
  // Embed the box ~3 cm into the ground so the initial contact depth exceeds
  // the sleep tolerance. With zero gravity and zero velocity the body stays
  // quiet; split-impulse position correction shrinks the penetration without
  // imparting velocity.
  auto box = addDynamicBox(world, "box", Eigen::Vector3d(0.0, 0.0, 0.17));

  // First step: the body is quiet (a sleep candidate) but still penetrating
  // beyond tolerance, so the gate keeps it awake rather than freezing it
  // mid-interpenetration. Without the gate it would sleep on this step.
  world.step();
  ASSERT_TRUE(box.getLinearVelocity().isZero(1e-9));
  EXPECT_FALSE(box.isSleeping());

  // Once the penetration correction converges below tolerance, the still-quiet
  // body is allowed to sleep.
  bool slept = false;
  for (int i = 0; i < 20; ++i) {
    world.step();
    if (box.isSleeping()) {
      slept = true;
      break;
    }
  }
  EXPECT_TRUE(slept);
}

TEST(Deactivation, RestingMultibodySleepsAndFreezes)
{
  sx::World world;
  world.setTimeStep(0.01);
  world.setGravity(Eigen::Vector3d::Zero());
  world.setDeactivationOptions(fastSleepOptions());
  auto robot = addPrismaticRobot(world, "robot", Eigen::Vector3d::UnitZ());

  EXPECT_FALSE(robot.isSleeping());
  EXPECT_EQ(robot.getDeactivationGroupIndex(), -1);

  for (int i = 0; i < 6; ++i) {
    world.step();
  }

  EXPECT_TRUE(robot.isSleeping());
  EXPECT_GE(robot.getDeactivationGroupIndex(), 0);
}

TEST(Deactivation, LinkForceWakesSleepingMultibody)
{
  sx::World world;
  world.setTimeStep(0.01);
  world.setGravity(Eigen::Vector3d::Zero());
  world.setDeactivationOptions(fastSleepOptions());
  auto robot = addPrismaticRobot(world, "robot", Eigen::Vector3d::UnitZ());
  auto carriage = *robot.getLink("carriage");

  for (int i = 0; i < 6; ++i) {
    world.step();
  }
  ASSERT_TRUE(robot.isSleeping());

  carriage.applyForce(Eigen::Vector3d(0.0, 0.0, 5.0));
  world.step();

  EXPECT_FALSE(robot.isSleeping());
}

TEST(Deactivation, CommandAccelerationWakesSleepingMultibody)
{
  sx::World world;
  world.setTimeStep(0.01);
  world.setGravity(Eigen::Vector3d::Zero());
  world.setDeactivationOptions(fastSleepOptions());
  auto robot = addPrismaticRobot(world, "robot", Eigen::Vector3d::UnitZ());
  auto carriage = *robot.getLink("carriage");
  auto joint = carriage.getParentJoint();
  joint.setActuatorType(sx::ActuatorType::Acceleration);

  for (int i = 0; i < 6; ++i) {
    world.step();
  }
  ASSERT_TRUE(robot.isSleeping());

  constexpr double commandAcceleration = 3.0;
  joint.setCommandAcceleration(
      Eigen::VectorXd::Constant(1, commandAcceleration));
  world.step();

  EXPECT_FALSE(robot.isSleeping());
  EXPECT_NEAR(joint.getAcceleration()[0], commandAcceleration, 1e-12);
  EXPECT_GT(joint.getVelocity()[0], 0.0);
}

TEST(Deactivation, ContactWithActiveBodyWakesSleepingMultibody)
{
  sx::World world;
  world.setTimeStep(0.01);
  world.setGravity(Eigen::Vector3d::Zero());
  world.setDeactivationOptions(fastSleepOptions());
  auto robot = addPrismaticRobot(
      world, "robot", Eigen::Vector3d::UnitX(), /*attachCollisionShape=*/true);
  auto striker = addDynamicBox(
      world,
      "striker",
      Eigen::Vector3d(-1.0, 0.0, 0.0),
      Eigen::Vector3d(2.0, 0.0, 0.0));

  for (int i = 0; i < 6; ++i) {
    world.step();
  }
  ASSERT_TRUE(robot.isSleeping());
  ASSERT_FALSE(striker.isSleeping());

  bool woke = false;
  for (int i = 0; i < 100; ++i) {
    world.step();
    if (!robot.isSleeping()) {
      woke = true;
      break;
    }
  }

  EXPECT_TRUE(woke);
}

} // namespace
