/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License
 */

#include <dart/simulation/experimental/common/exceptions.hpp>
#include <dart/simulation/experimental/comps/multibody.hpp>
#include <dart/simulation/experimental/compute/variational_integration.hpp>
#include <dart/simulation/experimental/multibody/multibody.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <entt/entt.hpp>
#include <gtest/gtest.h>

#include <cmath>

namespace {

namespace sx = dart::simulation::experimental;
namespace sxc = dart::simulation::experimental::compute;

// Returns the (single) multibody structure component in the world.
const sx::comps::MultibodyStructure& structureOf(sx::World& world)
{
  auto& registry = world.getRegistry();
  auto view = registry.view<sx::comps::MultibodyStructure>();
  return registry.get<sx::comps::MultibodyStructure>(*view.begin());
}

} // namespace

// A vertical prismatic joint under gravity is a constant-force linear system,
// so the variational integrator must reproduce the exact free-fall acceleration
// in a single step (mirrors World.MultibodyPrismaticFreeFall).
TEST(VariationalIntegration, PrismaticFreeFallMatchesConstantAcceleration)
{
  sx::World world; // default gravity (0, 0, -9.81)
  auto robot = world.addMultibody("slider");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "rail";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto carriage = robot.addLink("carriage", base, spec);
  carriage.setMass(3.0);

  const double dt = 0.01;
  world.setTimeStep(dt);
  world.enterSimulationMode();

  sxc::MultibodyVariationalState state;
  const auto report = sxc::integrateMultibodyVariational(
      world.getRegistry(), structureOf(world), world.getGravity(), dt, state);

  EXPECT_TRUE(report.converged);
  auto joint = carriage.getParentJoint();
  EXPECT_NEAR(joint.getAcceleration()[0], -9.81, 1e-9);
  EXPECT_NEAR(joint.getVelocity()[0], -9.81 * dt, 1e-9);
  EXPECT_NEAR(joint.getPosition()[0], -9.81 * dt * dt, 1e-9);
}

// The first-step generalized acceleration of a pendulum released from rest must
// match the Newtonian value m g L / I_pivot to the integrator's order.
TEST(VariationalIntegration, RevolutePendulumFirstStepAcceleration)
{
  sx::World world;
  auto robot = world.addMultibody("pendulum");
  auto base = robot.addLink("base");

  const double length = 1.5;
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(length, 0.0, 0.0);
  sx::JointSpec spec;
  spec.name = "hinge";
  spec.type = sx::JointType::Revolute;
  spec.axis = Eigen::Vector3d::UnitY();
  spec.transformFromParent = offset;
  auto bob = robot.addLink("bob", base, spec);

  const double mass = 2.0;
  const double inertiaYy = 0.2;
  bob.setMass(mass);
  bob.setInertia(Eigen::Vector3d(0.1, inertiaYy, 0.3).asDiagonal());

  const double dt = 1e-3;
  world.setTimeStep(dt);
  world.enterSimulationMode();

  sxc::MultibodyVariationalState state;
  const auto report = sxc::integrateMultibodyVariational(
      world.getRegistry(), structureOf(world), world.getGravity(), dt, state);

  EXPECT_TRUE(report.converged);
  const double expected
      = 9.81 * mass * length / (inertiaYy + mass * length * length);
  // Consistency with the continuum equations of motion to O(dt) of the discrete
  // acceleration (this is not the exact discrete value, so the tolerance is
  // looser than the prismatic constant-force gate).
  EXPECT_NEAR(bob.getParentJoint().getAcceleration()[0], expected, 1e-2);
}

// A passive pendulum conserves total mechanical energy under the symplectic
// variational integrator: no secular drift over a long horizon.
TEST(VariationalIntegration, PendulumConservesEnergyOverLongHorizon)
{
  sx::World world;
  auto robot = world.addMultibody("pendulum");
  auto base = robot.addLink("base");

  const double length = 1.0;
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(length, 0.0, 0.0);
  sx::JointSpec spec;
  spec.name = "hinge";
  spec.type = sx::JointType::Revolute;
  spec.axis = Eigen::Vector3d::UnitY();
  spec.transformFromParent = offset;
  auto bob = robot.addLink("bob", base, spec);
  bob.setMass(1.0);
  bob.setInertia(Eigen::Vector3d(0.05, 0.05, 0.05).asDiagonal());

  const double dt = 1e-3;
  world.setTimeStep(dt);
  world.enterSimulationMode();

  // Release from a raised position at rest.
  bob.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, 1.0));
  world.updateKinematics();

  auto& registry = world.getRegistry();
  const auto& structure = structureOf(world);
  const Eigen::Vector3d gravity = world.getGravity();

  const double energy0
      = sxc::computeMultibodyMechanicalEnergy(registry, structure, gravity);
  ASSERT_GT(std::abs(energy0), 1e-6);

  sxc::MultibodyVariationalState state;
  double maxRelativeDrift = 0.0;
  const int steps = 100000;
  for (int k = 0; k < steps; ++k) {
    sxc::integrateMultibodyVariational(registry, structure, gravity, dt, state);
    if (k % 1000 == 0) {
      const double energy
          = sxc::computeMultibodyMechanicalEnergy(registry, structure, gravity);
      ASSERT_FALSE(std::isnan(energy));
      maxRelativeDrift = std::max(
          maxRelativeDrift, std::abs(energy - energy0) / std::abs(energy0));
    }
  }

  // Symplectic behavior: bounded energy oscillation, no secular drift.
  EXPECT_LT(maxRelativeDrift, 1e-2);
}

// The variational integrator is reachable through the public method-name
// selector on the default World::step() path; selecting it conserves energy,
// confirming pipeline substitution (not the double-integrating stage-append
// path) and a facade-safe selection by method name.
TEST(VariationalIntegration, SelectableThroughWorldStep)
{
  sx::World world;
  EXPECT_EQ(world.getMultibodyIntegrationMethod(), "semi-implicit");
  EXPECT_THROW(
      world.setMultibodyIntegrationMethod("nonsense"),
      sx::InvalidArgumentException);
  world.setMultibodyIntegrationMethod("variational integrator");
  EXPECT_EQ(world.getMultibodyIntegrationMethod(), "variational integrator");

  auto robot = world.addMultibody("pendulum");
  auto base = robot.addLink("base");
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  sx::JointSpec spec;
  spec.name = "hinge";
  spec.type = sx::JointType::Revolute;
  spec.axis = Eigen::Vector3d::UnitY();
  spec.transformFromParent = offset;
  auto bob = robot.addLink("bob", base, spec);
  bob.setMass(1.0);
  bob.setInertia(Eigen::Vector3d(0.05, 0.05, 0.05).asDiagonal());

  world.setTimeStep(1e-3);
  world.enterSimulationMode();
  bob.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, 1.0));
  world.updateKinematics();

  auto& registry = world.getRegistry();
  const auto& structure = structureOf(world);
  const Eigen::Vector3d gravity = world.getGravity();
  const double energy0
      = sxc::computeMultibodyMechanicalEnergy(registry, structure, gravity);

  for (int k = 0; k < 20000; ++k) {
    world.step();
  }

  const double energy
      = sxc::computeMultibodyMechanicalEnergy(registry, structure, gravity);
  EXPECT_FALSE(std::isnan(energy));
  EXPECT_LT(std::abs(energy - energy0) / std::abs(energy0), 1e-2);
}
