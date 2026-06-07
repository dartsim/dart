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

} // namespace
