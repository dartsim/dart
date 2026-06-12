/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License
 */

#include <dart/simulation/common/exceptions.hpp>
#include <dart/simulation/comps/multibody.hpp>
#include <dart/simulation/compute/multibody_constraint.hpp>
#include <dart/simulation/detail/entity_conversion.hpp>
#include <dart/simulation/detail/world_registry_access.hpp>
#include <dart/simulation/multibody/joint.hpp>
#include <dart/simulation/multibody/multibody.hpp>
#include <dart/simulation/world.hpp>

#include <gtest/gtest.h>

#include <limits>

namespace {

//==============================================================================
TEST(MultibodyConstraint, IntegratesSphericalAndFloatingManifolds)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto robot = world.addMultibody("robot");
  auto base = robot.addLink("base");

  sx::JointSpec sphericalSpec;
  sphericalSpec.name = "spherical";
  sphericalSpec.type = sx::JointType::Spherical;
  auto sphericalLink = robot.addLink("spherical_link", base, sphericalSpec);
  auto spherical = sphericalLink.getParentJoint();

  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto floatingLink = robot.addLink("floating_link", base, floatingSpec);
  auto floating = floatingLink.getParentJoint();

  const double dt = 0.25;
  const Eigen::Vector3d angularVelocity(0.2, -0.4, 0.6);
  Eigen::VectorXd floatingVelocity(6);
  floatingVelocity << 1.0, -2.0, 3.0, -0.3, 0.5, -0.7;

  Eigen::VectorXd nextVelocity(9);
  nextVelocity << angularVelocity, floatingVelocity;

  auto& registry = dart::simulation::detail::registryOf(world);
  const auto& structure = registry.get<sx::comps::MultibodyStructure>(
      dart::simulation::detail::toRegistryEntity(robot.getEntity()));
  sx::compute::integrateMultibodyPositions(
      registry, structure, nextVelocity, dt);

  EXPECT_TRUE(spherical.getVelocity().isApprox(angularVelocity, 1e-12));
  EXPECT_TRUE(spherical.getPosition().isApprox(angularVelocity * dt, 1e-12));
  EXPECT_TRUE(
      spherical.getAcceleration().isApprox(angularVelocity / dt, 1e-12));

  EXPECT_TRUE(floating.getVelocity().isApprox(floatingVelocity, 1e-12));
  EXPECT_TRUE(floating.getPosition().head<3>().isApprox(
      floatingVelocity.head<3>() * dt, 1e-12));
  EXPECT_TRUE(floating.getPosition().tail<3>().isApprox(
      floatingVelocity.tail<3>() * dt, 1e-12));
  EXPECT_TRUE(
      floating.getAcceleration().isApprox(floatingVelocity / dt, 1e-12));
}

//==============================================================================
TEST(MultibodyConstraint, AppliesPositionLimitsAfterLinearIntegration)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto robot = world.addMultibody("robot");
  auto base = robot.addLink("base");

  sx::JointSpec spec;
  spec.name = "slider";
  spec.type = sx::JointType::Prismatic;
  auto link = robot.addLink("link", base, spec);
  auto joint = link.getParentJoint();

  Eigen::VectorXd position(1);
  position << 0.9;
  joint.setPosition(position);

  Eigen::VectorXd lower(1);
  Eigen::VectorXd upper(1);
  lower << -std::numeric_limits<double>::infinity();
  upper << 1.0;
  joint.setPositionLimits(lower, upper);

  Eigen::VectorXd nextVelocity(1);
  nextVelocity << 20.0;

  auto& registry = dart::simulation::detail::registryOf(world);
  const auto& structure = registry.get<sx::comps::MultibodyStructure>(
      dart::simulation::detail::toRegistryEntity(robot.getEntity()));
  sx::compute::integrateMultibodyPositions(
      registry, structure, nextVelocity, 0.1);

  EXPECT_NEAR(joint.getPosition()[0], 1.0, 1e-12);
  EXPECT_NEAR(joint.getVelocity()[0], 0.0, 1e-12);
  EXPECT_NEAR(joint.getAcceleration()[0], 0.0, 1e-12);
}

//==============================================================================
TEST(MultibodyConstraint, RejectsWrongVelocityDimensionBeforeMutation)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto robot = world.addMultibody("robot");
  auto base = robot.addLink("base");

  sx::JointSpec spec;
  spec.name = "hinge";
  spec.type = sx::JointType::Revolute;
  auto link = robot.addLink("link", base, spec);
  auto joint = link.getParentJoint();

  Eigen::VectorXd position(1);
  position << 0.4;
  joint.setPosition(position);

  Eigen::VectorXd velocity(1);
  velocity << -0.2;
  joint.setVelocity(velocity);

  auto& registry = dart::simulation::detail::registryOf(world);
  const auto& structure = registry.get<sx::comps::MultibodyStructure>(
      dart::simulation::detail::toRegistryEntity(robot.getEntity()));
  const Eigen::VectorXd wrongSize = Eigen::VectorXd::Zero(2);
  EXPECT_THROW(
      sx::compute::integrateMultibodyPositions(
          registry, structure, wrongSize, 0.1),
      sx::InvalidArgumentException);

  EXPECT_TRUE(joint.getPosition().isApprox(position, 1e-12));
  EXPECT_TRUE(joint.getVelocity().isApprox(velocity, 1e-12));
}

} // namespace
