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
#include <dart/simulation/experimental/compute/multibody_dynamics.hpp>
#include <dart/simulation/experimental/compute/variational_integration.hpp>
#include <dart/simulation/experimental/constraint/loop_closure_spec.hpp>
#include <dart/simulation/experimental/frame/frame.hpp>
#include <dart/simulation/experimental/multibody/multibody.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <entt/entt.hpp>
#include <gtest/gtest.h>

#include <array>
#include <functional>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

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
  EXPECT_EQ(world.getMultibodyOptions().integrationFamily, "semi-implicit");
  EXPECT_THROW(
      world.setMultibodyOptions({.integrationFamily = "nonsense"}),
      sx::InvalidArgumentException);
  world.setMultibodyOptions({.integrationFamily = "variational integrator"});
  EXPECT_EQ(
      world.getMultibodyOptions().integrationFamily, "variational integrator");

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

// The O(n) articulated-body inverse-mass apply that powers the RIQN step must
// equal the dense mass-matrix solve M(q)^{-1} b to machine precision, on a
// branchy mixed-joint chain at a non-trivial configuration.
TEST(VariationalIntegration, ArticulatedInverseMassMatchesDenseSolve)
{
  sx::World world;
  auto robot = world.addMultibody("chain");
  auto base = robot.addLink("base");

  const auto offset = [](double x) {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.translation() = Eigen::Vector3d(x, 0.0, 0.0);
    return t;
  };
  sx::JointSpec j1;
  j1.name = "j1";
  j1.type = sx::JointType::Revolute;
  j1.axis = Eigen::Vector3d::UnitY();
  j1.transformFromParent = offset(0.5);
  auto l1 = robot.addLink("l1", base, j1);
  sx::JointSpec j2;
  j2.name = "j2";
  j2.type = sx::JointType::Revolute;
  j2.axis = Eigen::Vector3d::UnitZ();
  j2.transformFromParent = offset(0.4);
  auto l2 = robot.addLink("l2", l1, j2);
  sx::JointSpec j3;
  j3.name = "j3";
  j3.type = sx::JointType::Prismatic;
  j3.axis = Eigen::Vector3d::UnitX();
  j3.transformFromParent = offset(0.3);
  auto l3 = robot.addLink("l3", l2, j3);

  for (auto link : {l1, l2, l3}) {
    link.setMass(1.3);
    link.setInertia(Eigen::Vector3d(0.1, 0.2, 0.15).asDiagonal());
  }

  world.setTimeStep(1e-3);
  world.enterSimulationMode();
  l1.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, 0.4));
  l2.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, -0.7));
  l3.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, 0.2));
  world.updateKinematics();

  auto& registry = world.getRegistry();
  const auto& structure = structureOf(world);
  const Eigen::MatrixXd massMatrix
      = sxc::computeMultibodyDynamicsTerms(
            registry, structure, world.getGravity())
            .massMatrix;
  ASSERT_EQ(massMatrix.rows(), 3);

  const Eigen::LDLT<Eigen::MatrixXd> dense(massMatrix);
  const std::vector<Eigen::VectorXd> rhs
      = {(Eigen::VectorXd(3) << 1, 0, 0).finished(),
         (Eigen::VectorXd(3) << 0, 1, 0).finished(),
         (Eigen::VectorXd(3) << 0, 0, 1).finished(),
         (Eigen::VectorXd(3) << 0.7, -1.3, 0.9).finished()};
  for (const auto& b : rhs) {
    const Eigen::VectorXd abi
        = sxc::computeMultibodyInverseMassProduct(registry, structure, b);
    const Eigen::VectorXd denseSolve = dense.solve(b);
    EXPECT_TRUE(abi.isApprox(denseSolve, 1e-9))
        << "b=" << b.transpose() << " abi=" << abi.transpose()
        << " dense=" << denseSolve.transpose();
  }
}

// The variational integrator is deterministic: two identical rollouts through
// the public World::step() path produce bit-identical final state.
TEST(VariationalIntegration, DeterministicAcrossRuns)
{
  const auto rollout = []() {
    sx::World world;
    world.setMultibodyOptions({.integrationFamily = "variational integrator"});
    auto robot = world.addMultibody("chain");
    auto parent = robot.addLink("base");
    for (int i = 0; i < 3; ++i) {
      Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
      offset.translation() = Eigen::Vector3d(0.4, 0.0, 0.0);
      sx::JointSpec spec;
      spec.name = "j" + std::to_string(i);
      spec.type = sx::JointType::Revolute;
      spec.axis = Eigen::Vector3d::UnitY();
      spec.transformFromParent = offset;
      auto link = robot.addLink("l" + std::to_string(i), parent, spec);
      link.setMass(1.0);
      link.setInertia(Eigen::Vector3d(0.05, 0.05, 0.05).asDiagonal());
      parent = link;
    }
    world.setTimeStep(1e-3);
    world.enterSimulationMode();
    auto joints = robot.getJoints();
    for (std::size_t i = 0; i < joints.size(); ++i) {
      joints[i].setPosition(Eigen::VectorXd::Constant(1, 0.3));
    }
    world.updateKinematics();
    for (int k = 0; k < 2000; ++k) {
      world.step();
    }
    Eigen::VectorXd finalState(static_cast<Eigen::Index>(joints.size()));
    for (std::size_t i = 0; i < joints.size(); ++i) {
      finalState[static_cast<Eigen::Index>(i)] = joints[i].getPosition()[0];
    }
    return finalState;
  };

  const Eigen::VectorXd a = rollout();
  const Eigen::VectorXd b = rollout();
  EXPECT_GT(a.norm(), 0.0);       // the chain actually evolved
  EXPECT_EQ((a - b).norm(), 0.0); // and did so bit-identically
}

// Phase B1: a floating base (fixed virtual base + Floating joint + body) under
// gravity free-falls — the body's linear acceleration is -g with no spurious
// angular acceleration. A free body is a constant-force linear system, so the
// variational integrator is exact.
TEST(VariationalIntegration, FloatingBaseFreeFall)
{
  sx::World world; // gravity (0, 0, -9.81)
  auto robot = world.addMultibody("floater");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "float";
  spec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, spec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  const double dt = 0.01;
  world.setTimeStep(dt);
  world.enterSimulationMode();

  sxc::MultibodyVariationalState state;
  const auto report = sxc::integrateMultibodyVariational(
      world.getRegistry(), structureOf(world), world.getGravity(), dt, state);

  EXPECT_TRUE(report.converged);
  const Eigen::VectorXd accel = body.getParentJoint().getAcceleration();
  ASSERT_EQ(accel.size(), 6);
  // Generalized velocity is [linear; angular]; gravity drives -g in z only.
  EXPECT_NEAR(accel[0], 0.0, 1e-9);
  EXPECT_NEAR(accel[1], 0.0, 1e-9);
  EXPECT_NEAR(accel[2], -9.81, 1e-9);
  EXPECT_LT(accel.tail<3>().norm(), 1e-9);
}

// Phase B1: a torque-free floating body (no gravity) with initial linear and
// angular velocity conserves total mechanical energy under the symplectic
// variational integrator over a long horizon — a strong check of the
// manifold-correct SE(3) retraction (asymmetric inertia induces tumbling).
TEST(VariationalIntegration, FloatingBaseTorqueFreeConservesEnergy)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  auto robot = world.addMultibody("floater");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "float";
  spec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, spec);
  body.setMass(1.5);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  const double dt = 1e-3;
  world.setTimeStep(dt);
  world.enterSimulationMode();
  Eigen::VectorXd v0(6);
  v0 << 0.4, -0.3, 0.2, 1.5, 2.0, -1.0; // [linear; angular] (tumbling)
  body.getParentJoint().setVelocity(v0);
  world.updateKinematics();

  auto& registry = world.getRegistry();
  const auto& structure = structureOf(world);
  const Eigen::Vector3d gravity = world.getGravity();
  const double energy0
      = sxc::computeMultibodyMechanicalEnergy(registry, structure, gravity);
  ASSERT_GT(energy0, 1e-6); // pure kinetic energy

  sxc::MultibodyVariationalState state;
  double maxRelativeDrift = 0.0;
  for (int k = 0; k < 20000; ++k) {
    sxc::integrateMultibodyVariational(registry, structure, gravity, dt, state);
    if (k % 500 == 0) {
      const double energy
          = sxc::computeMultibodyMechanicalEnergy(registry, structure, gravity);
      ASSERT_FALSE(std::isnan(energy));
      maxRelativeDrift = std::max(
          maxRelativeDrift, std::abs(energy - energy0) / std::abs(energy0));
    }
  }
  EXPECT_LT(maxRelativeDrift, 1e-2);
}

// Phase B2: a holonomic loop closure (the tip of a 2-link arm held at a fixed
// distance from a world anchor) is maintained by the impulse-based projection
// while the 1-DOF system still swings under gravity.
TEST(VariationalIntegration, MaintainsDistanceLoopClosure)
{
  sx::World world; // gravity (0, 0, -9.81)
  auto robot = world.addMultibody("loop");
  auto base = robot.addLink("base");
  const auto offset = [](double x) {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.translation() = Eigen::Vector3d(x, 0.0, 0.0);
    return t;
  };
  sx::JointSpec s1;
  s1.name = "j1";
  s1.type = sx::JointType::Revolute;
  s1.axis = Eigen::Vector3d::UnitY();
  s1.transformFromParent = offset(0.5);
  auto link1 = robot.addLink("link1", base, s1);
  link1.setMass(1.0);
  link1.setInertia(Eigen::Vector3d(0.05, 0.05, 0.05).asDiagonal());
  sx::JointSpec s2;
  s2.name = "j2";
  s2.type = sx::JointType::Revolute;
  s2.axis = Eigen::Vector3d::UnitY();
  s2.transformFromParent = offset(0.5);
  auto link2 = robot.addLink("link2", link1, s2);
  link2.setMass(1.0);
  link2.setInertia(Eigen::Vector3d(0.05, 0.05, 0.05).asDiagonal());

  const double dt = 1e-3;
  world.setTimeStep(dt);
  world.enterSimulationMode();
  link1.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, 0.5));
  link2.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, -0.4));
  world.updateKinematics();

  auto& registry = world.getRegistry();
  const auto& structure = structureOf(world);
  const Eigen::Vector3d gravity = world.getGravity();

  // Anchor the link2 origin at its initial distance from a fixed world point.
  const Eigen::Vector3d anchor(0.3, 0.0, 0.2);
  const double length = (link2.getTransform().translation() - anchor).norm();
  ASSERT_GT(length, 1e-3);

  sxc::VariationalLoopConstraint constraint;
  constraint.linkA = structure.links[2]; // link2
  constraint.pointA = Eigen::Vector3d::Zero();
  constraint.linkB = entt::null; // fixed world anchor
  constraint.pointB = anchor;
  constraint.distance = true;
  constraint.length = length;
  const std::vector<sxc::VariationalLoopConstraint> constraints{constraint};

  sxc::MultibodyVariationalState state;
  double maxViolation = 0.0;
  double motion = 0.0;
  for (int k = 0; k < 5000; ++k) {
    sxc::integrateMultibodyVariational(
        registry, structure, gravity, dt, state, 50, 1e-10, constraints);
    world.updateKinematics();
    const double dist = (link2.getTransform().translation() - anchor).norm();
    maxViolation = std::max(maxViolation, std::abs(dist - length));
    motion = std::max(
        motion, std::abs(link1.getParentJoint().getPosition()[0] - 0.5));
  }
  EXPECT_LT(maxViolation, 1e-6); // closure maintained
  EXPECT_GT(motion, 1e-3);       // the constrained system actually moved
}

// PLAN-082 headline acceptance gate. On a passive 10-link revolute chain (whose
// inertia is strongly configuration-dependent), the variational integrator's
// total mechanical energy stays in a bounded band with *no secular drift*: the
// least-squares slope of energy-vs-time is ~0. Semi-implicit Euler on the very
// same scene -- the default integration family -- conserves measurably worse.
// The no-secular-drift slope is the falsifiable headline gate.
TEST(VariationalIntegration, PassiveChainEnergyHasNoSecularDrift)
{
  constexpr int kLinks = 10;
  const double dt = 1e-3;
  const int steps = 100000;
  const int sampleEvery = 1000;

  // Build the identical passive chain into a fresh world (released bent, so the
  // configuration -- and thus the mass matrix -- varies strongly as it swings).
  const auto buildChain = [](sx::World& world) {
    auto robot = world.addMultibody("chain");
    auto parent = robot.addLink("base");
    for (int i = 0; i < kLinks; ++i) {
      Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
      offset.translation() = Eigen::Vector3d(i == 0 ? 0.0 : 0.3, 0.0, 0.0);
      sx::JointSpec spec;
      spec.name = "j" + std::to_string(i);
      spec.type = sx::JointType::Revolute;
      spec.axis = Eigen::Vector3d::UnitY();
      spec.transformFromParent = offset;
      auto link = robot.addLink("l" + std::to_string(i), parent, spec);
      link.setMass(0.7);
      link.setInertia(Eigen::Vector3d(0.02, 0.02, 0.02).asDiagonal());
      parent = link;
    }
    return robot;
  };

  // Least-squares slope of y vs sample index (per-sample units).
  const auto slope = [](const std::vector<double>& y) {
    const double n = static_cast<double>(y.size());
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (std::size_t i = 0; i < y.size(); ++i) {
      const double x = static_cast<double>(i);
      sx += x;
      sy += y[i];
      sxx += x * x;
      sxy += x * y[i];
    }
    return (n * sxy - sx * sy) / (n * sxx - sx * sx);
  };

  // Variational-integrator rollout (direct, so we control the solve).
  std::vector<double> viDev;
  double viBand = 0.0;
  {
    sx::World world;
    auto robot = buildChain(world);
    world.setTimeStep(dt);
    world.enterSimulationMode();
    for (auto joint : robot.getJoints()) {
      joint.setPosition(Eigen::VectorXd::Constant(1, 0.4));
    }
    world.updateKinematics();
    auto& registry = world.getRegistry();
    const auto& structure = structureOf(world);
    const Eigen::Vector3d gravity = world.getGravity();
    const double e0
        = sxc::computeMultibodyMechanicalEnergy(registry, structure, gravity);
    ASSERT_GT(std::abs(e0), 1e-6);
    sxc::MultibodyVariationalState state;
    for (int k = 0; k < steps; ++k) {
      sxc::integrateMultibodyVariational(
          registry, structure, gravity, dt, state);
      if (k % sampleEvery == 0) {
        const double e = sxc::computeMultibodyMechanicalEnergy(
            registry, structure, gravity);
        ASSERT_FALSE(std::isnan(e));
        const double dev = (e - e0) / std::abs(e0);
        viDev.push_back(dev);
        viBand = std::max(viBand, std::abs(dev));
      }
    }
  }

  // Semi-implicit Euler rollout (default family) on the same scene.
  std::vector<double> seDev;
  double seBand = 0.0;
  {
    sx::World world; // default method == "semi-implicit"
    auto robot = buildChain(world);
    world.setTimeStep(dt);
    world.enterSimulationMode();
    for (auto joint : robot.getJoints()) {
      joint.setPosition(Eigen::VectorXd::Constant(1, 0.4));
    }
    world.updateKinematics();
    auto& registry = world.getRegistry();
    const auto& structure = structureOf(world);
    const Eigen::Vector3d gravity = world.getGravity();
    const double e0
        = sxc::computeMultibodyMechanicalEnergy(registry, structure, gravity);
    for (int k = 0; k < steps; ++k) {
      world.step();
      if (k % sampleEvery == 0) {
        const double e = sxc::computeMultibodyMechanicalEnergy(
            registry, structure, gravity);
        ASSERT_FALSE(std::isnan(e));
        const double dev = (e - e0) / std::abs(e0);
        seDev.push_back(dev);
        seBand = std::max(seBand, std::abs(dev));
      }
    }
  }

  const double viSlope = std::abs(slope(viDev));
  const double seSlope = std::abs(slope(seDev));
  // Net fitted energy trend over the whole run (per-sample slope x #samples).
  const double viTrend = viSlope * static_cast<double>(viDev.size());

  // Measured (deterministic) on this scene: viBand ~= 0.0109, seBand ~= 0.39,
  // viSlope ~= 6e-6, seSlope ~= 4e-3 -- the VI conserves orders of magnitude
  // better. The *band* at 1e5 steps is trajectory-sensitive (the chain is
  // chaotic, so the per-step root -- found to 1e-10 either way -- compounds
  // into a slightly different rollout under Anderson acceleration vs the plain
  // step); the no-secular-drift *slope* below is the falsifiable gate per the
  // plan, and thresholds are tuned to recorded measurements.

  // (1) Bounded band: VI energy oscillation stays small (here ~1.1%), in stark
  //     contrast to semi-implicit Euler's ~39% drift below.
  EXPECT_LT(viBand, 1.5e-2) << "VI energy band " << viBand << " too large";
  // (2) Contrast: semi-implicit Euler on the same scene drifts far beyond 1%.
  EXPECT_GT(seBand, 5e-2)
      << "semi-implicit Euler did not drift beyond the band as expected; band "
      << seBand;
  // (3) No secular drift: the VI's net fitted energy trend is a small fraction
  //     of its oscillation band (slope ~= 0 within the oscillation noise).
  EXPECT_LT(viTrend, 0.25 * viBand) << "VI net energy trend " << viTrend
                                    << " is not small vs band " << viBand;
  // (4) The VI's drift slope is at least 50x smaller than semi-implicit
  // Euler's.
  EXPECT_LT(viSlope * 50.0, seSlope)
      << "VI slope " << viSlope << " not >=50x better than semi-implicit slope "
      << seSlope;
}

// Phase B2 gate: the holonomic constraint Jacobian J = dg/dq used by the
// loop-closure projection must match a central finite difference of the
// residual g(q) w.r.t. the generalized coordinates. Exercises both row types
// (a distance row and three point rows) and both endpoint kinds (a
// link-vs-world anchor and a link-vs-link pair).
TEST(VariationalIntegration, ConstraintJacobianMatchesFiniteDifference)
{
  sx::World world;
  auto robot = world.addMultibody("arm");
  auto base = robot.addLink("base");
  const auto offset = [](double x) {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.translation() = Eigen::Vector3d(x, 0.0, 0.0);
    return t;
  };
  sx::JointSpec s1;
  s1.name = "j1";
  s1.type = sx::JointType::Revolute;
  s1.axis = Eigen::Vector3d::UnitY();
  s1.transformFromParent = offset(0.5);
  auto link1 = robot.addLink("link1", base, s1);
  link1.setMass(1.0);
  link1.setInertia(Eigen::Vector3d(0.05, 0.05, 0.05).asDiagonal());
  sx::JointSpec s2;
  s2.name = "j2";
  s2.type = sx::JointType::Revolute;
  s2.axis = Eigen::Vector3d::UnitY();
  s2.transformFromParent = offset(0.5);
  auto link2 = robot.addLink("link2", link1, s2);
  link2.setMass(1.0);
  link2.setInertia(Eigen::Vector3d(0.05, 0.05, 0.05).asDiagonal());

  world.setTimeStep(1e-3);
  world.enterSimulationMode();
  auto joints = robot.getJoints();
  ASSERT_EQ(joints.size(), 2u);
  joints[0].setPosition(Eigen::VectorXd::Constant(1, 0.3));
  joints[1].setPosition(Eigen::VectorXd::Constant(1, -0.5));
  world.updateKinematics();

  auto& registry = world.getRegistry();
  const auto& structure = structureOf(world);

  std::vector<sxc::VariationalLoopConstraint> constraints;
  // (1) distance: a point on link2 to a fixed world anchor (linkB == null).
  sxc::VariationalLoopConstraint dist;
  dist.linkA = structure.links[2];
  dist.pointA = Eigen::Vector3d(0.1, 0.0, 0.0);
  dist.linkB = entt::null;
  dist.pointB = Eigen::Vector3d(0.4, 0.0, 0.1);
  dist.distance = true;
  dist.length = 0.0; // shifts g by a constant only; dg/dq is independent of it
  constraints.push_back(dist);
  // (2) point: a point on link2 coincident with a point on link1
  // (link-vs-link).
  sxc::VariationalLoopConstraint point;
  point.linkA = structure.links[2];
  point.pointA = Eigen::Vector3d::Zero();
  point.linkB = structure.links[1];
  point.pointB = Eigen::Vector3d(0.3, 0.0, 0.0);
  point.distance = false;
  constraints.push_back(point);

  const auto lin = sxc::computeVariationalConstraintLinearization(
      registry, structure, constraints);
  const Eigen::Index rows = lin.residual.size();
  const Eigen::Index dof = lin.jacobian.cols();
  ASSERT_EQ(rows, 4); // 1 distance + 3 point
  ASSERT_EQ(dof, 2);

  const double eps = 1e-6;
  Eigen::MatrixXd fd(rows, dof);
  for (Eigen::Index j = 0; j < dof; ++j) {
    const double q0 = joints[static_cast<std::size_t>(j)].getPosition()[0];
    joints[static_cast<std::size_t>(j)].setPosition(
        Eigen::VectorXd::Constant(1, q0 + eps));
    const Eigen::VectorXd plus = sxc::computeVariationalConstraintLinearization(
                                     registry, structure, constraints)
                                     .residual;
    joints[static_cast<std::size_t>(j)].setPosition(
        Eigen::VectorXd::Constant(1, q0 - eps));
    const Eigen::VectorXd minus
        = sxc::computeVariationalConstraintLinearization(
              registry, structure, constraints)
              .residual;
    joints[static_cast<std::size_t>(j)].setPosition(
        Eigen::VectorXd::Constant(1, q0)); // restore
    fd.col(j) = (plus - minus) / (2.0 * eps);
  }

  EXPECT_TRUE(fd.isApprox(lin.jacobian, 1e-6))
      << "analytic J=\n"
      << lin.jacobian << "\nfinite-difference=\n"
      << fd;
}

// PLAN-082 convergence gate: RIQN converges in a small, bounded number of
// iterations on a representative multi-link scene (mean <= 8 per the plan), and
// every step reaches the tolerance (no silent non-convergence).
TEST(VariationalIntegration, RiqnMeanIterationsWithinBudget)
{
  sx::World world;
  auto robot = world.addMultibody("chain");
  auto parent = robot.addLink("base");
  for (int i = 0; i < 6; ++i) {
    Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
    offset.translation() = Eigen::Vector3d(i == 0 ? 0.0 : 0.3, 0.0, 0.0);
    sx::JointSpec spec;
    spec.name = "j" + std::to_string(i);
    spec.type = sx::JointType::Revolute;
    spec.axis = Eigen::Vector3d::UnitY();
    spec.transformFromParent = offset;
    auto link = robot.addLink("l" + std::to_string(i), parent, spec);
    link.setMass(1.0);
    link.setInertia(Eigen::Vector3d(0.03, 0.03, 0.03).asDiagonal());
    parent = link;
  }
  const double dt = 1e-3;
  world.setTimeStep(dt);
  world.enterSimulationMode();
  for (auto joint : robot.getJoints()) {
    joint.setPosition(Eigen::VectorXd::Constant(1, 0.3));
  }
  world.updateKinematics();

  auto& registry = world.getRegistry();
  const auto& structure = structureOf(world);
  const Eigen::Vector3d gravity = world.getGravity();

  sxc::MultibodyVariationalState state;
  const int steps = 2000;
  std::size_t totalIters = 0;
  std::size_t maxIters = 0;
  for (int k = 0; k < steps; ++k) {
    const auto report = sxc::integrateMultibodyVariational(
        registry, structure, gravity, dt, state);
    ASSERT_TRUE(report.converged)
        << "step " << k << " residual " << report.residualNorm;
    totalIters += report.iterations;
    maxIters = std::max(maxIters, report.iterations);
  }
  const double meanIters = static_cast<double>(totalIters) / steps;
  EXPECT_LE(meanIters, 8.0) << "mean RIQN iterations " << meanIters;
  EXPECT_LE(maxIters, 50u) << "max RIQN iterations " << maxIters;
}

// PLAN-082 convergence gate: non-convergence is a documented hard error, not a
// silent best-effort step. Forcing an impossible tolerance in a single
// iteration must throw rather than write back a non-converged configuration.
TEST(VariationalIntegration, NonConvergenceRaisesDocumentedError)
{
  sx::World world;
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

  const double dt = 1e-3;
  world.setTimeStep(dt);
  world.enterSimulationMode();
  bob.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, 1.0));
  world.updateKinematics();

  sxc::MultibodyVariationalState state;
  // A single RIQN iteration cannot drive the nonlinear residual below 1e-30.
  EXPECT_THROW(
      sxc::integrateMultibodyVariational(
          world.getRegistry(),
          structureOf(world),
          world.getGravity(),
          dt,
          state,
          /*maxIterations=*/1,
          /*tolerance=*/1e-30),
      sx::InvalidOperationException);
}

// PLAN-082 momentum gate: a force-free floating body (no gravity) conserves
// both linear momentum (its COM advances at constant velocity) and angular
// momentum, a Noether symmetry the variational integrator preserves by
// construction. The body is a symmetric top (I_xx == I_yy) spinning about its
// symmetry axis while translating, so the world angular momentum L = R (I .
// omega) is an exact relative equilibrium and must hold to solver precision (a
// spin-axis drift from a faulty SO(3) retraction would break it).
TEST(VariationalIntegration, FloatingBaseConservesMomentum)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  auto robot = world.addMultibody("floater");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "float";
  spec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, spec);
  body.setMass(1.5);
  const Eigen::Vector3d inertia(0.2, 0.2, 0.1); // symmetric about z
  body.setInertia(inertia.asDiagonal());

  const double dt = 1e-3;
  world.setTimeStep(dt);
  world.enterSimulationMode();
  Eigen::VectorXd v0(6);
  v0 << 0.5, -0.3, 0.2, 0.0, 0.0, 1.4; // [linear; angular]: translate + z-spin
  body.getParentJoint().setVelocity(v0);
  world.updateKinematics();

  auto& registry = world.getRegistry();
  const auto& structure = structureOf(world);
  const Eigen::Vector3d gravity = world.getGravity();

  const auto worldAngularMomentum = [&]() {
    const Eigen::Matrix3d r = body.getTransform().linear();
    const Eigen::Vector3d omega = body.getParentJoint().getVelocity().tail<3>();
    return Eigen::Vector3d(r * (inertia.asDiagonal() * omega));
  };

  const Eigen::Vector3d l0 = worldAngularMomentum();
  ASSERT_GT(l0.norm(), 1e-6);
  Eigen::Vector3d previousCom = body.getTransform().translation();
  Eigen::Vector3d previousStep = Eigen::Vector3d::Zero();
  double maxAngularDrift = 0.0;
  double maxLinearJerk = 0.0; // change in per-step COM displacement
  sxc::MultibodyVariationalState state;
  for (int k = 0; k < 5000; ++k) {
    sxc::integrateMultibodyVariational(registry, structure, gravity, dt, state);
    world.updateKinematics();
    const Eigen::Vector3d com = body.getTransform().translation();
    const Eigen::Vector3d stepDisp = com - previousCom;
    if (k > 0) {
      maxLinearJerk = std::max(maxLinearJerk, (stepDisp - previousStep).norm());
    }
    previousStep = stepDisp;
    previousCom = com;
    maxAngularDrift = std::max(
        maxAngularDrift, (worldAngularMomentum() - l0).norm() / l0.norm());
  }
  // Linear momentum: COM velocity is constant, so the per-step displacement
  // does not change (zero jerk) to solver precision.
  EXPECT_LT(maxLinearJerk, 1e-9) << "linear-momentum jerk " << maxLinearJerk;
  // Angular momentum: world L is conserved to solver precision.
  EXPECT_LT(maxAngularDrift, 1e-8)
      << "angular-momentum drift " << maxAngularDrift;
}

// Phase B2 deliverable: a Point loop closure added through the public World API
// is solved by the variational integrator on the default World::step() path.
// The tip of a spatial 5-DOF arm is pinned to its rest-pose world position;
// under gravity the arm flexes (internal DOF remain) while the pinned point
// holds. The arm has a bent ("zigzag") rest shape so the 3-row point Jacobian
// is full rank (a straight arm would be singular for a 3D point pin). Loop
// closures are topology, so the closure is added in design mode; the rest-pose
// tip position (all joints at zero) is the product of the link offsets,
// computed here.
TEST(VariationalIntegration, LoopClosureSolvedThroughWorldStep)
{
  sx::World world;
  world.setMultibodyOptions({.integrationFamily = "variational integrator"});
  auto robot = world.addMultibody("arm");
  auto parent = robot.addLink("base");
  const double bend[5] = {0.0, 0.5, -0.5, 0.5, -0.5};
  Eigen::Isometry3d tipFromBase = Eigen::Isometry3d::Identity();
  for (int i = 0; i < 5; ++i) {
    Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
    offset.linear() = Eigen::AngleAxisd(bend[i], Eigen::Vector3d::UnitY())
                          .toRotationMatrix();
    offset.translation() = Eigen::Vector3d(0.3, 0.0, 0.0);
    sx::JointSpec spec;
    spec.name = "j" + std::to_string(i);
    spec.type = sx::JointType::Revolute;
    spec.axis = (i == 0) ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitY();
    spec.transformFromParent = offset;
    auto link = robot.addLink("l" + std::to_string(i), parent, spec);
    link.setMass(0.8);
    link.setInertia(Eigen::Vector3d(0.02, 0.02, 0.02).asDiagonal());
    parent = link;
    tipFromBase = tipFromBase * offset; // joints at zero => relative == offset
  }

  auto tip = robot.getLinks().back();
  Eigen::Isometry3d anchorFrame = Eigen::Isometry3d::Identity();
  anchorFrame.translation() = tipFromBase.translation();
  auto closure = world.addLoopClosure(
      "pin",
      {.frameA = tip,
       .frameB = sx::Frame::world(),
       .family = sx::LoopClosureFamily::Point,
       .offsetB = anchorFrame});
  closure.setRuntimePolicy(
      {.enabled = true,
       .kinematics = sx::ClosureKinematicsPolicy::ResidualOnly,
       .dynamics = sx::ClosureDynamicsPolicy::Solve});

  world.setTimeStep(1e-3);
  world.enterSimulationMode();
  auto joints = robot.getJoints();
  ASSERT_EQ(joints.size(), 5u);

  double maxResidual = 0.0;
  double motion = 0.0;
  for (int k = 0; k < 2000; ++k) {
    world.step();
    maxResidual = std::max(maxResidual, closure.computeResidual().norm);
    for (auto joint : joints) {
      motion = std::max(motion, std::abs(joint.getPosition()[0]));
    }
  }
  EXPECT_LT(maxResidual, 1e-6); // tip stays pinned through world.step()
  EXPECT_GT(motion, 1e-3);      // the arm still flexes under gravity
}

// Phase B2 deliverable (rejection): a Point closure spanning two multibodies
// cannot be enforced by the per-multibody variational solver and must be
// rejected at step time (the architect-flagged correctness trap), not silently
// mishandled.
TEST(VariationalIntegration, LoopClosureCrossMultibodyRejectedUnderVariational)
{
  sx::World world;
  world.setMultibodyOptions({.integrationFamily = "variational integrator"});
  const auto makeArm = [&](const char* name) {
    auto robot = world.addMultibody(name);
    auto base = robot.addLink(std::string(name) + "_base");
    sx::JointSpec spec;
    spec.name = std::string(name) + "_j";
    spec.type = sx::JointType::Revolute;
    spec.axis = Eigen::Vector3d::UnitY();
    Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
    offset.translation() = Eigen::Vector3d(0.5, 0.0, 0.0);
    spec.transformFromParent = offset;
    auto link = robot.addLink(std::string(name) + "_link", base, spec);
    link.setMass(1.0);
    link.setInertia(Eigen::Vector3d(0.05, 0.05, 0.05).asDiagonal());
    return link;
  };
  auto linkA = makeArm("a");
  auto linkB = makeArm("b");

  auto closure = world.addLoopClosure(
      "cross",
      {.frameA = linkA,
       .frameB = linkB,
       .family = sx::LoopClosureFamily::Point});
  closure.setRuntimePolicy(
      {.enabled = true,
       .kinematics = sx::ClosureKinematicsPolicy::ResidualOnly,
       .dynamics = sx::ClosureDynamicsPolicy::Solve});

  world.setTimeStep(1e-3);
  world.enterSimulationMode();
  EXPECT_THROW(world.step(), sx::InvalidOperationException);
}

// Phase B2 deliverable (rejection): the Rigid family (which needs an
// orientation residual the solver does not implement) is rejected under the
// variational method rather than silently dropping the orientation constraint.
// Phase B2 (Rigid family): the 6-row rigid constraint Jacobian (3 position + 3
// orientation) matches a central finite difference of the residual. The tip of
// a 3-DOF spatial arm is welded to its current world pose so the residual is
// zero at the base configuration -- where the first-order orientation Jacobian
// is exact -- and the difference is taken around it.
TEST(VariationalIntegration, RigidConstraintJacobianMatchesFiniteDifference)
{
  sx::World world;
  auto robot = world.addMultibody("arm");
  auto parent = robot.addLink("base");
  const Eigen::Vector3d axes[3]
      = {Eigen::Vector3d::UnitZ(),
         Eigen::Vector3d::UnitY(),
         Eigen::Vector3d::UnitX()};
  for (int i = 0; i < 3; ++i) {
    Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
    offset.translation() = Eigen::Vector3d(0.4, 0.0, 0.0);
    sx::JointSpec spec;
    spec.name = "j" + std::to_string(i);
    spec.type = sx::JointType::Revolute;
    spec.axis = axes[i];
    spec.transformFromParent = offset;
    auto link = robot.addLink("l" + std::to_string(i), parent, spec);
    link.setMass(1.0);
    link.setInertia(Eigen::Vector3d(0.05, 0.05, 0.05).asDiagonal());
    parent = link;
  }
  world.setTimeStep(1e-3);
  world.enterSimulationMode();
  auto joints = robot.getJoints();
  ASSERT_EQ(joints.size(), 3u);
  const double q0[3] = {0.3, -0.4, 0.5};
  for (int i = 0; i < 3; ++i) {
    joints[static_cast<std::size_t>(i)].setPosition(
        Eigen::VectorXd::Constant(1, q0[i]));
  }
  world.updateKinematics();

  auto& registry = world.getRegistry();
  const auto& structure = structureOf(world);
  const Eigen::Isometry3d tipPose = robot.getLinks().back().getTransform();

  sxc::VariationalLoopConstraint c;
  c.linkA = structure.links[3]; // tip
  c.pointA = Eigen::Vector3d::Zero();
  c.rotationA = Eigen::Matrix3d::Identity();
  c.linkB = entt::null; // world weld target = the tip's current pose
  c.pointB = tipPose.translation();
  c.rotationB = tipPose.linear();
  c.rigid = true;
  const std::vector<sxc::VariationalLoopConstraint> constraints{c};

  const auto lin = sxc::computeVariationalConstraintLinearization(
      registry, structure, constraints);
  ASSERT_EQ(lin.residual.size(), 6); // 3 position + 3 orientation
  ASSERT_EQ(lin.jacobian.cols(), 3);
  EXPECT_LT(lin.residual.norm(), 1e-9); // welded at the current pose

  const double eps = 1e-6;
  Eigen::MatrixXd fd(6, 3);
  for (int j = 0; j < 3; ++j) {
    const double base = joints[static_cast<std::size_t>(j)].getPosition()[0];
    joints[static_cast<std::size_t>(j)].setPosition(
        Eigen::VectorXd::Constant(1, base + eps));
    const Eigen::VectorXd plus = sxc::computeVariationalConstraintLinearization(
                                     registry, structure, constraints)
                                     .residual;
    joints[static_cast<std::size_t>(j)].setPosition(
        Eigen::VectorXd::Constant(1, base - eps));
    const Eigen::VectorXd minus
        = sxc::computeVariationalConstraintLinearization(
              registry, structure, constraints)
              .residual;
    joints[static_cast<std::size_t>(j)].setPosition(
        Eigen::VectorXd::Constant(1, base));
    fd.col(j) = (plus - minus) / (2.0 * eps);
  }
  EXPECT_TRUE(fd.isApprox(lin.jacobian, 1e-6))
      << "analytic J=\n"
      << lin.jacobian << "\nfinite-difference=\n"
      << fd;
}

// Phase B2 (Rigid family through the public API): a Rigid closure weld added
// via addLoopClosure holds the full pose (position + orientation) of a spatial
// 7-DOF arm's tip through World::step() under the variational method, while the
// redundant internal degree of freedom still flexes under gravity.
TEST(VariationalIntegration, LoopClosureRigidSolvedThroughWorldStep)
{
  sx::World world;
  world.setMultibodyOptions({.integrationFamily = "variational integrator"});
  auto robot = world.addMultibody("arm");
  auto parent = robot.addLink("base");
  const Eigen::Vector3d axes[7]
      = {Eigen::Vector3d::UnitZ(),
         Eigen::Vector3d::UnitY(),
         Eigen::Vector3d::UnitX(),
         Eigen::Vector3d::UnitY(),
         Eigen::Vector3d::UnitZ(),
         Eigen::Vector3d::UnitY(),
         Eigen::Vector3d::UnitX()};
  const double bend[7] = {0.2, 0.3, -0.2, 0.4, -0.3, 0.2, -0.4};
  Eigen::Isometry3d tipFromBase = Eigen::Isometry3d::Identity();
  for (int i = 0; i < 7; ++i) {
    Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
    offset.linear() = Eigen::AngleAxisd(bend[i], Eigen::Vector3d::UnitZ())
                          .toRotationMatrix();
    offset.translation() = Eigen::Vector3d(0.25, 0.0, 0.0);
    sx::JointSpec spec;
    spec.name = "j" + std::to_string(i);
    spec.type = sx::JointType::Revolute;
    spec.axis = axes[i];
    spec.transformFromParent = offset;
    auto link = robot.addLink("l" + std::to_string(i), parent, spec);
    link.setMass(0.6);
    link.setInertia(Eigen::Vector3d(0.01, 0.01, 0.01).asDiagonal());
    parent = link;
    tipFromBase = tipFromBase * offset; // joints at zero => relative == offset
  }

  auto tip = robot.getLinks().back();
  auto closure = world.addLoopClosure(
      "weld",
      {.frameA = tip,
       .frameB = sx::Frame::world(),
       .family = sx::LoopClosureFamily::Rigid,
       .offsetB = tipFromBase}); // weld target = the tip's rest pose
  closure.setRuntimePolicy(
      {.enabled = true,
       .kinematics = sx::ClosureKinematicsPolicy::ResidualOnly,
       .dynamics = sx::ClosureDynamicsPolicy::Solve});

  world.setTimeStep(1e-3);
  world.enterSimulationMode();
  auto joints = robot.getJoints();
  ASSERT_EQ(joints.size(), 7u);

  double maxResidual = 0.0;
  double motion = 0.0;
  for (int k = 0; k < 2000; ++k) {
    world.step();
    maxResidual = std::max(maxResidual, closure.computeResidual().norm);
    for (auto joint : joints) {
      motion = std::max(motion, std::abs(joint.getPosition()[0]));
    }
  }
  EXPECT_LT(maxResidual, 1e-6); // the 6-DOF weld holds through world.step()
  EXPECT_GT(motion, 1e-3);      // the redundant internal DOF still flexes
}

// Phase B2 (Distance family through the public API): a Distance loop closure
// added via addLoopClosure -- holding the tip of a 2-link arm at a fixed
// distance from a world anchor -- is solved by the variational integrator on
// the World::step() path while the arm swings under gravity.
TEST(VariationalIntegration, LoopClosureDistanceSolvedThroughWorldStep)
{
  sx::World world;
  world.setMultibodyOptions({.integrationFamily = "variational integrator"});
  auto robot = world.addMultibody("arm");
  auto base = robot.addLink("base");
  const auto offset = [](double x) {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.translation() = Eigen::Vector3d(x, 0.0, 0.0);
    return t;
  };
  sx::JointSpec s1;
  s1.name = "j1";
  s1.type = sx::JointType::Revolute;
  s1.axis = Eigen::Vector3d::UnitY();
  s1.transformFromParent = offset(0.5);
  auto link1 = robot.addLink("link1", base, s1);
  link1.setMass(1.0);
  link1.setInertia(Eigen::Vector3d(0.05, 0.05, 0.05).asDiagonal());
  sx::JointSpec s2;
  s2.name = "j2";
  s2.type = sx::JointType::Revolute;
  s2.axis = Eigen::Vector3d::UnitY();
  s2.transformFromParent = offset(0.5);
  auto link2 = robot.addLink("link2", link1, s2);
  link2.setMass(1.0);
  link2.setInertia(Eigen::Vector3d(0.05, 0.05, 0.05).asDiagonal());

  // Rest-pose tip (joints at zero) is the product of the link offsets.
  const Eigen::Vector3d tip(1.0, 0.0, 0.0);
  const Eigen::Vector3d anchorPoint(0.3, 0.0, 0.2);
  const double length = (tip - anchorPoint).norm();
  ASSERT_GT(length, 1e-3);
  Eigen::Isometry3d anchorFrame = Eigen::Isometry3d::Identity();
  anchorFrame.translation() = anchorPoint;

  auto closure = world.addLoopClosure(
      "distance",
      {.frameA = link2,
       .frameB = sx::Frame::world(),
       .family = sx::LoopClosureFamily::Distance,
       .offsetB = anchorFrame,
       .distance = length});
  closure.setRuntimePolicy(
      {.enabled = true,
       .kinematics = sx::ClosureKinematicsPolicy::ResidualOnly,
       .dynamics = sx::ClosureDynamicsPolicy::Solve});

  world.setTimeStep(1e-3);
  world.enterSimulationMode();

  double maxResidual = 0.0;
  double motion = 0.0;
  for (int k = 0; k < 3000; ++k) {
    world.step();
    maxResidual
        = std::max(maxResidual, std::abs(closure.computeResidual().norm));
    motion
        = std::max(motion, std::abs(link1.getParentJoint().getPosition()[0]));
  }
  EXPECT_LT(maxResidual, 1e-6); // distance held through world.step()
  EXPECT_GT(motion, 1e-3);      // the arm still swings under gravity
}

// PLAN-082 serialization gate: binary save/load round-trips the trajectory and
// does NOT re-bootstrap the two-step discrete-mechanics history. A reference
// pendulum runs 50 steps (establishing a non-trivial history), is saved, then
// run 50 more; a fresh world loaded from the save and run 50 steps must reach
// the bit-identical state. If MultibodyVariationalState were lost on load, the
// integrator would re-bootstrap from the current velocity and diverge.
TEST(VariationalIntegration, StateSerializationRoundTripsTrajectory)
{
  const auto buildPendulum = [](sx::World& world) {
    world.setMultibodyOptions({.integrationFamily = "variational integrator"});
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
  };

  sx::World reference;
  buildPendulum(reference);
  reference.setTimeStep(1e-3);
  reference.enterSimulationMode();
  reference.getMultibody("pendulum")
      ->getJoints()[0]
      .setPosition(Eigen::VectorXd::Constant(1, 1.0));
  reference.updateKinematics();
  for (int k = 0; k < 50; ++k) {
    reference.step();
  }

  // Save mid-rollout: the two-step history is now bootstrapped and non-trivial.
  std::stringstream buffer(std::ios::in | std::ios::out | std::ios::binary);
  reference.saveBinary(buffer);

  for (int k = 0; k < 50; ++k) {
    reference.step();
  }
  const double referenceFinal
      = reference.getMultibody("pendulum")->getJoints()[0].getPosition()[0];

  // Load into a fresh world and continue. The integration family is a World
  // option (not state), so re-select it; the variational history must come back
  // from the save so the continuation is bit-identical.
  sx::World loaded;
  buffer.seekg(0);
  loaded.loadBinary(buffer);
  loaded.setMultibodyOptions({.integrationFamily = "variational integrator"});
  ASSERT_TRUE(loaded.getMultibody("pendulum").has_value());
  for (int k = 0; k < 50; ++k) {
    loaded.step();
  }
  const double loadedFinal
      = loaded.getMultibody("pendulum")->getJoints()[0].getPosition()[0];

  EXPECT_EQ(loadedFinal, referenceFinal); // bit-identical, no re-bootstrap
}

// PLAN-082 A2 long-chain convergence gate: a 64-link revolute chain converges
// within the default iteration budget at every step. The fixed dt*M^{-1}
// quasi-Newton rate makes such a chain peak near ~456 iterations (measured) --
// far past any reasonable budget, raising the non-convergence error -- because
// dt*M^{-1} is only an approximate inverse Jacobian. The exact recursive-
// Jacobian Newton preconditioner (`applyExactNewtonStep`) bounds the iteration
// count to a few regardless of length (see LongChainExactPreconditioner... for
// the >=100-link cases the quasi-Newton step cannot reach within the default
// budget).
TEST(VariationalIntegration, LongChainConvergesWithinDefaultBudget)
{
  sx::World world;
  auto robot = world.addMultibody("chain");
  auto parent = robot.addLink("base");
  for (int i = 0; i < 64; ++i) {
    Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
    offset.translation() = Eigen::Vector3d(i == 0 ? 0.0 : 0.3, 0.0, 0.0);
    sx::JointSpec spec;
    spec.name = "j" + std::to_string(i);
    spec.type = sx::JointType::Revolute;
    spec.axis = Eigen::Vector3d::UnitY();
    spec.transformFromParent = offset;
    auto link = robot.addLink("l" + std::to_string(i), parent, spec);
    link.setMass(1.0);
    link.setInertia(Eigen::Vector3d(0.03, 0.03, 0.03).asDiagonal());
    parent = link;
  }
  const double dt = 1e-3;
  world.setTimeStep(dt);
  world.enterSimulationMode();
  for (auto joint : robot.getJoints()) {
    joint.setPosition(Eigen::VectorXd::Constant(1, 0.3));
  }
  world.updateKinematics();

  auto& registry = world.getRegistry();
  const auto& structure = structureOf(world);
  const Eigen::Vector3d gravity = world.getGravity();
  sxc::MultibodyVariationalState state;
  std::size_t maxIters = 0;
  // Each call uses the default maxIterations (50); a non-converging step would
  // throw, so reaching the end proves every step stayed within budget.
  for (int k = 0; k < 300; ++k) {
    const auto report = sxc::integrateMultibodyVariational(
        registry, structure, gravity, dt, state);
    ASSERT_TRUE(report.converged) << "step " << k;
    maxIters = std::max(maxIters, report.iterations);
  }
  EXPECT_LE(maxIters, 100u) << "max RIQN iterations " << maxIters;
}

// Exact recursive-Jacobian (Newton) preconditioner headline gate: passive
// chains of >=100 links (the cases the fixed dt*M^{-1} quasi-Newton step peaks
// near ~205 iterations on -- beyond the default 100 budget) converge in only a
// handful of iterations *within the default budget*, and the iteration count is
// length-independent (it does not grow from 100 to 128 links). Each call uses
// the default maxIterations; a non-converging step throws, so completing the
// rollout proves every step stayed within budget. Energy stays finite and the
// chain actually evolves (behavioral sanity), confirming a real integration --
// not a degenerate "converged at the guess" path.
TEST(VariationalIntegration, LongChainExactPreconditionerConvergesWithinBudget)
{
  for (const int links : {100, 128}) {
    sx::World world;
    auto robot = world.addMultibody("chain");
    auto parent = robot.addLink("base");
    for (int i = 0; i < links; ++i) {
      Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
      offset.translation() = Eigen::Vector3d(i == 0 ? 0.0 : 0.3, 0.0, 0.0);
      sx::JointSpec spec;
      spec.name = "j" + std::to_string(i);
      spec.type = sx::JointType::Revolute;
      spec.axis = Eigen::Vector3d::UnitY();
      spec.transformFromParent = offset;
      auto link = robot.addLink("l" + std::to_string(i), parent, spec);
      link.setMass(1.0);
      link.setInertia(Eigen::Vector3d(0.03, 0.03, 0.03).asDiagonal());
      parent = link;
    }
    const double dt = 1e-3;
    world.setTimeStep(dt);
    world.enterSimulationMode();
    for (auto joint : robot.getJoints()) {
      joint.setPosition(Eigen::VectorXd::Constant(1, 0.3)); // released bent
    }
    world.updateKinematics();

    auto& registry = world.getRegistry();
    const auto& structure = structureOf(world);
    const Eigen::Vector3d gravity = world.getGravity();
    const double energy0
        = sxc::computeMultibodyMechanicalEnergy(registry, structure, gravity);
    ASSERT_GT(std::abs(energy0), 1e-6);

    sxc::MultibodyVariationalState state;
    std::size_t maxIters = 0;
    const double tip0 = parent.getTransform().translation().z();
    double maxTipDrop = 0.0;
    for (int k = 0; k < 300; ++k) {
      // Default budget (100). With the fixed dt*M^{-1} preconditioner this
      // would raise the non-convergence error on the hardest steps (~205 >
      // 100); the exact Newton step converges in only a few.
      const auto report = sxc::integrateMultibodyVariational(
          registry, structure, gravity, dt, state);
      ASSERT_TRUE(report.converged) << "links=" << links << " step=" << k
                                    << " residual=" << report.residualNorm;
      maxIters = std::max(maxIters, report.iterations);
      const double energy
          = sxc::computeMultibodyMechanicalEnergy(registry, structure, gravity);
      ASSERT_FALSE(std::isnan(energy)) << "links=" << links << " step=" << k;
      world.updateKinematics();
      maxTipDrop = std::max(
          maxTipDrop, tip0 - parent.getTransform().translation().z());
    }

    // Exact preconditioner: a handful of iterations, well within the default
    // budget, on a chain the fixed quasi-Newton step cannot solve in budget.
    EXPECT_LE(maxIters, 20u)
        << "links=" << links << " max Newton iterations " << maxIters;
    // The chain actually swings down under gravity (real integration).
    EXPECT_GT(maxTipDrop, 1e-3)
        << "links=" << links << " tip drop " << maxTipDrop;
  }
}

// Zero default-path overhead: with the default ("semi-implicit") integration
// family, stepping a multibody never engages the variational integrator. Its
// per-multibody state component is created only by the variational stage's
// execute(), so its absence after stepping proves none of the DRNEA/RIQN/ABI
// machinery ran -- conventional solvers pay nothing for the VI feature.
TEST(VariationalIntegration, DefaultPathDoesNotEngageVariationalIntegrator)
{
  sx::World world; // default family is "semi-implicit"
  ASSERT_EQ(world.getMultibodyOptions().integrationFamily, "semi-implicit");
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
  for (int k = 0; k < 50; ++k) {
    world.step();
  }
  // The variational stage (hence its state component) is never instantiated.
  EXPECT_EQ(
      world.getRegistry().view<sxc::MultibodyVariationalState>().size(), 0u);
}

// Measurement harness (disabled in CI): mean/max RIQN iterations vs chain
// length, quantifying the long-chain convergence behavior. Run explicitly with
//   --gtest_also_run_disabled_tests
//   --gtest_filter=*RiqnIterationsVsChainLength*
TEST(VariationalIntegration, DISABLED_RiqnIterationsVsChainLength)
{
  for (int n : {8, 16, 32, 64, 100}) {
    sx::World world;
    auto robot = world.addMultibody("chain");
    auto parent = robot.addLink("base");
    for (int i = 0; i < n; ++i) {
      Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
      offset.translation() = Eigen::Vector3d(i == 0 ? 0.0 : 0.3, 0.0, 0.0);
      sx::JointSpec spec;
      spec.name = "j" + std::to_string(i);
      spec.type = sx::JointType::Revolute;
      spec.axis = Eigen::Vector3d::UnitY();
      spec.transformFromParent = offset;
      auto link = robot.addLink("l" + std::to_string(i), parent, spec);
      link.setMass(1.0);
      link.setInertia(Eigen::Vector3d(0.03, 0.03, 0.03).asDiagonal());
      parent = link;
    }
    const double dt = 1e-3;
    world.setTimeStep(dt);
    world.enterSimulationMode();
    for (auto joint : robot.getJoints()) {
      joint.setPosition(Eigen::VectorXd::Constant(1, 0.3));
    }
    world.updateKinematics();
    auto& registry = world.getRegistry();
    const auto& structure = structureOf(world);
    const Eigen::Vector3d gravity = world.getGravity();
    sxc::MultibodyVariationalState state;
    std::size_t total = 0;
    std::size_t maxIters = 0;
    const int steps = 200;
    for (int k = 0; k < steps; ++k) {
      const auto report = sxc::integrateMultibodyVariational(
          registry, structure, gravity, dt, state, /*maxIterations=*/500);
      total += report.iterations;
      maxIters = std::max(maxIters, report.iterations);
    }
    std::cout << "N=" << n
              << " meanIters=" << (static_cast<double>(total) / steps)
              << " maxIters=" << maxIters << "\n";
  }
}

// Manifold-Anderson convergence gate (long spherical chain). A long chain of
// links each on a Spherical joint lives on the SO(3) manifold, so it takes the
// manifold-correct fixed dt*M^{-1} quasi-Newton step (NOT the Euclidean exact-
// Newton path), accelerated by the tangent-space Anderson mixing. The plain
// fixed point (Anderson disabled) on a long, heavy, fast-spinning chain hits a
// per-step accuracy plateau and fails to reach a tight tolerance on most steps;
// the tangent-space Anderson acceleration pushes through that plateau and
// converges every step within a small iteration budget. Run the identical
// scene with the acceleration disabled (depth 0, the plain control) vs enabled
// (depth 5) and compare: the accelerated path converges on far more steps
// (here every step) within the same budget, with sane (near-conserved) energy.
//
// (Why a convergence-fraction gate rather than a raw iteration-count delta:
// measured on this codebase the manifold quasi-Newton step is well conditioned
// and converges in ~4-5 iterations until it reaches the residual-evaluation
// accuracy floor of a multi-DOF spherical chain -- there is no long slow
// iteration tail to shave, unlike a long Euclidean chain. The acceleration's
// payoff is therefore reaching a *lower residual floor*, i.e. converging to a
// tolerance the plain step cannot reach at all; at that tolerance the plain
// step exhausts the budget while the accelerated step needs only a handful of
// iterations -- the strongest, most reproducible form of the iteration-count
// comparison.)
TEST(VariationalIntegration, ManifoldAndersonAcceleratesSphericalChain)
{
  constexpr int kLinks = 20;
  const double dt = 1e-3;
  const int steps = 200;
  const int budget = 50;
  // Per-coordinate tolerance the plain manifold step plateaus above on most
  // steps but the Anderson-accelerated step reaches every step.
  const double tol = 1e-6;

  // A long, heavy chain hanging straight down along -Z (a stable, extended
  // equilibrium that does not fold into a singular configuration as a
  // horizontal chain would), spun fast so the per-step DEL residual is large
  // enough to exercise genuine RIQN iteration (a near-equilibrium passive chain
  // converges from the initial guess in ~1 iteration and never exercises the
  // solve).
  const auto buildChain = [](sx::World& world) {
    auto robot = world.addMultibody("spherical_chain");
    auto parent = robot.addLink("base");
    for (int i = 0; i < kLinks; ++i) {
      Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
      offset.translation() = Eigen::Vector3d(0, 0, i == 0 ? 0.0 : -0.6);
      sx::JointSpec spec;
      spec.name = "j" + std::to_string(i);
      spec.type = sx::JointType::Spherical;
      spec.transformFromParent = offset;
      auto link = robot.addLink("l" + std::to_string(i), parent, spec);
      link.setMass(3.0);
      link.setInertia(Eigen::Vector3d(0.3, 0.3, 0.3).asDiagonal());
      parent = link;
    }
    return robot;
  };

  // Roll the identical scene out with a given Anderson depth, recording how
  // many steps converged within the budget (the iteration cap is a
  // non-convergence hard error, so a step that exhausts it throws), the
  // max/mean iterations of the converged steps, and the energy drift.
  struct Result
  {
    std::size_t convergedSteps = 0;
    std::size_t maxIters = 0;
    double meanIters = 0.0;
    double maxEnergyDrift = 0.0;
  };
  const auto rollout = [&](std::size_t andersonDepth) {
    sx::World world;
    auto robot = buildChain(world);
    world.setTimeStep(dt);
    world.enterSimulationMode();
    // Bend each joint slightly off-axis and spin neighbouring links in opposite
    // directions so they shear past each other (a large, configuration-varying
    // per-step residual).
    auto joints = robot.getJoints();
    for (std::size_t i = 0; i < joints.size(); ++i) {
      joints[i].setPosition((Eigen::VectorXd(3) << 0.0, 0.03, 0.0).finished());
      const double s = (i % 2 == 0) ? 1.0 : -1.0;
      joints[i].setVelocity(
          (Eigen::VectorXd(3) << 0.0, 8.0 * s, 3.0 * s).finished());
    }
    world.updateKinematics();

    auto& registry = world.getRegistry();
    const auto& structure = structureOf(world);
    const Eigen::Vector3d gravity = world.getGravity();
    const double energy0
        = sxc::computeMultibodyMechanicalEnergy(registry, structure, gravity);

    Result result;
    sxc::MultibodyVariationalState state;
    std::size_t totalIters = 0;
    for (int k = 0; k < steps; ++k) {
      try {
        const auto report = sxc::integrateMultibodyVariational(
            registry,
            structure,
            gravity,
            dt,
            state,
            budget,
            tol,
            /*constraints=*/{},
            /*andersonDepth=*/andersonDepth);
        ++result.convergedSteps;
        totalIters += report.iterations;
        result.maxIters = std::max(result.maxIters, report.iterations);
        const double energy = sxc::computeMultibodyMechanicalEnergy(
            registry, structure, gravity);
        EXPECT_FALSE(std::isnan(energy)) << "depth=" << andersonDepth;
        result.maxEnergyDrift = std::max(
            result.maxEnergyDrift,
            std::abs(energy - energy0) / std::abs(energy0));
      } catch (const sx::InvalidOperationException&) {
        // A step exhausted the iteration budget without reaching `tol`.
        break;
      }
    }
    if (result.convergedSteps > 0) {
      result.meanIters
          = static_cast<double>(totalIters) / result.convergedSteps;
    }
    return result;
  };

  const Result plain = rollout(/*andersonDepth=*/0);       // disabled control
  const Result accelerated = rollout(/*andersonDepth=*/5); // tangent Anderson

  // Headline win: with the acceleration disabled the plain manifold
  // quasi-Newton step plateaus above the tolerance and converges on only a
  // small fraction of steps before a step exhausts the budget; the
  // tangent-space Anderson acceleration converges on every step.
  EXPECT_EQ(accelerated.convergedSteps, static_cast<std::size_t>(steps))
      << "accelerated did not converge every step (converged "
      << accelerated.convergedSteps << "/" << steps << ")";
  // A clear margin (the plain control converges on far fewer steps), not a
  // narrow tie. Measured: plain ~18/200, accelerated 200/200.
  EXPECT_LT(plain.convergedSteps * 4u, accelerated.convergedSteps)
      << "plain converged " << plain.convergedSteps << "/" << steps
      << " steps; accelerated converged " << accelerated.convergedSteps << "/"
      << steps
      << " -- the acceleration did not meaningfully extend convergence";
  // The accelerated path stays well within the iteration budget every step (the
  // plateau the plain step cannot cross is reached cheaply with mixing).
  EXPECT_LT(accelerated.maxIters, static_cast<std::size_t>(budget))
      << "accelerated max iters " << accelerated.maxIters
      << " reached the budget " << budget;
  EXPECT_LE(accelerated.meanIters, 12.0)
      << "accelerated mean iters " << accelerated.meanIters << " too high";

  // Identical converged behavior: the accelerated rollout's energy stays in a
  // tight band (no drift introduced by the acceleration; the converged DEL root
  // is preconditioner-independent).
  EXPECT_LT(accelerated.maxEnergyDrift, 1e-2)
      << "accelerated energy drift " << accelerated.maxEnergyDrift
      << " too large";
}

// Manifold-Anderson no-regression gate (floating base + spherical chain). A
// floating-base body carrying a spherical chain takes the same manifold
// quasi-Newton + tangent-space Anderson path. A floating base is well
// conditioned (it converges in ~1 iteration), so this is a correctness gate
// rather than a speed win: enabling the acceleration must keep the mixed
// SE(3)+SO(3) system converging with sane, near-conserved energy and an
// unchanged trajectory (the accelerated and plain runs reach the same root).
TEST(VariationalIntegration, ManifoldAndersonFloatingSphericalChainStaysCorrect)
{
  constexpr int kSphericalLinks = 12;
  const double dt = 1e-3;
  const int steps = 150;

  const auto buildChain = [](sx::World& world) {
    auto robot = world.addMultibody("floating_spherical");
    auto base = robot.addLink("base");
    sx::JointSpec floatSpec;
    floatSpec.name = "float";
    floatSpec.type = sx::JointType::Floating;
    auto parent = robot.addLink("body", base, floatSpec);
    parent.setMass(1.0);
    parent.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());
    for (int i = 0; i < kSphericalLinks; ++i) {
      Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
      offset.translation() = Eigen::Vector3d(0.3, 0.0, 0.0);
      sx::JointSpec spec;
      spec.name = "s" + std::to_string(i);
      spec.type = sx::JointType::Spherical;
      spec.transformFromParent = offset;
      auto link = robot.addLink("l" + std::to_string(i), parent, spec);
      link.setMass(0.6);
      link.setInertia(Eigen::Vector3d(0.02, 0.02, 0.02).asDiagonal());
      parent = link;
    }
    return robot;
  };

  struct Result
  {
    std::size_t maxIters = 0;
    double maxEnergyDrift = 0.0;
    Eigen::VectorXd finalConfig;
  };
  const auto rollout = [&](std::size_t andersonDepth) {
    sx::World world;
    auto robot = buildChain(world);
    world.setTimeStep(dt);
    world.enterSimulationMode();
    auto joints = robot.getJoints();
    for (std::size_t i = 1; i < joints.size(); ++i) { // joint 0 is the float
      joints[i].setPosition((Eigen::VectorXd(3) << 0.0, 0.25, 0.0).finished());
    }
    world.updateKinematics();

    auto& registry = world.getRegistry();
    const auto& structure = structureOf(world);
    const Eigen::Vector3d gravity = world.getGravity();
    const double energy0
        = sxc::computeMultibodyMechanicalEnergy(registry, structure, gravity);

    Result result;
    sxc::MultibodyVariationalState state;
    for (int k = 0; k < steps; ++k) {
      const auto report = sxc::integrateMultibodyVariational(
          registry,
          structure,
          gravity,
          dt,
          state,
          /*maxIterations=*/100,
          /*tolerance=*/1e-10,
          /*constraints=*/{},
          /*andersonDepth=*/andersonDepth);
      EXPECT_TRUE(report.converged)
          << "depth=" << andersonDepth << " step=" << k
          << " residual=" << report.residualNorm;
      result.maxIters = std::max(result.maxIters, report.iterations);
      const double energy
          = sxc::computeMultibodyMechanicalEnergy(registry, structure, gravity);
      EXPECT_FALSE(std::isnan(energy)) << "depth=" << andersonDepth;
      result.maxEnergyDrift = std::max(
          result.maxEnergyDrift,
          std::abs(energy - energy0) / std::abs(energy0));
    }
    Eigen::Index dof = 0;
    for (auto joint : joints) {
      dof += joint.getPosition().size();
    }
    result.finalConfig.resize(dof);
    Eigen::Index offset = 0;
    for (auto joint : joints) {
      const Eigen::VectorXd q = joint.getPosition();
      result.finalConfig.segment(offset, q.size()) = q;
      offset += q.size();
    }
    return result;
  };

  const Result plain = rollout(/*andersonDepth=*/0);
  const Result accelerated = rollout(/*andersonDepth=*/5);

  // Both converge (a floating base is well conditioned, so a handful of
  // iterations suffices) with sane energy, and the acceleration does not change
  // the integrated trajectory (the DEL root is preconditioner-independent).
  EXPECT_LE(accelerated.maxIters, 20u)
      << "accelerated max iters " << accelerated.maxIters;
  EXPECT_LT(accelerated.maxEnergyDrift, 1e-2)
      << "accelerated energy drift too large";
  ASSERT_EQ(accelerated.finalConfig.size(), plain.finalConfig.size());
  EXPECT_LT((accelerated.finalConfig - plain.finalConfig).norm(), 1e-6)
      << "accelerated and plain floating+spherical trajectories diverged";
}

namespace {

// Build a single floating body (fixed virtual base + Floating joint + body) of
// the given mass at the world origin. Returns the body Link handle.
sx::Link addFloatingBody(sx::World& world, double mass)
{
  auto robot = world.addMultibody("floater");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "float";
  spec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, spec);
  body.setMass(mass);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());
  return body;
}

} // namespace

// External Cartesian force on a free (floating) body: a force F applied at the
// body origin in zero gravity produces linear acceleration F/m and no angular
// acceleration. The semi-implicit forward-dynamics path is exact for this
// constant-force linear system.
TEST(ExternalForce, SemiImplicitFreeBodyAccelerationEqualsForceOverMass)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  const double mass = 2.5;
  auto body = addFloatingBody(world, mass);

  const double dt = 0.01;
  world.setTimeStep(dt); // default family: semi-implicit
  world.enterSimulationMode();

  const Eigen::Vector3d force(3.0, -4.0, 5.0);
  body.applyForce(force);

  world.step();

  // Starting from rest the post-step generalized acceleration equals qddot.
  const Eigen::VectorXd accel = body.getParentJoint().getAcceleration();
  ASSERT_EQ(accel.size(), 6);
  EXPECT_TRUE(accel.head<3>().isApprox(force / mass, 1e-6))
      << "linear accel = " << accel.head<3>().transpose()
      << " expected = " << (force / mass).transpose();
  EXPECT_LT(accel.tail<3>().norm(), 1e-6);
}

// The variational-integrator path produces the same free-body acceleration F/m
// for a constant external force (a free body is a constant-force linear system,
// so the variational integrator is exact).
TEST(ExternalForce, VariationalFreeBodyAccelerationEqualsForceOverMass)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  const double mass = 2.5;
  auto body = addFloatingBody(world, mass);

  const double dt = 0.01;
  world.setTimeStep(dt);
  world.enterSimulationMode();

  const Eigen::Vector3d force(3.0, -4.0, 5.0);
  body.applyForce(force);

  sxc::MultibodyVariationalState state;
  const auto report = sxc::integrateMultibodyVariational(
      world.getRegistry(), structureOf(world), world.getGravity(), dt, state);

  EXPECT_TRUE(report.converged);
  const Eigen::VectorXd accel = body.getParentJoint().getAcceleration();
  ASSERT_EQ(accel.size(), 6);
  EXPECT_TRUE(accel.head<3>().isApprox(force / mass, 1e-9))
      << "linear accel = " << accel.head<3>().transpose()
      << " expected = " << (force / mass).transpose();
  EXPECT_LT(accel.tail<3>().norm(), 1e-9);
}

// Gravity cancellation (semi-implicit): an upward external force equal to m*g
// holds a free body stationary for a step (zero generalized velocity).
TEST(ExternalForce, SemiImplicitUpwardForceCancelsGravity)
{
  sx::World world; // gravity (0, 0, -9.81)
  const double mass = 1.7;
  auto body = addFloatingBody(world, mass);

  const double dt = 0.01;
  world.setTimeStep(dt); // default family: semi-implicit
  world.enterSimulationMode();

  const Eigen::Vector3d gravity = world.getGravity();
  body.applyForce(-mass * gravity); // upward force = m*g

  world.step();

  const Eigen::VectorXd velocity = body.getParentJoint().getVelocity();
  ASSERT_EQ(velocity.size(), 6);
  EXPECT_LT(velocity.norm(), 1e-9) << "velocity = " << velocity.transpose();
}

// Gravity cancellation (variational): an upward external force equal to m*g
// holds a free body stationary for a step under the variational integrator.
TEST(ExternalForce, VariationalUpwardForceCancelsGravity)
{
  sx::World world; // gravity (0, 0, -9.81)
  const double mass = 1.7;
  auto body = addFloatingBody(world, mass);

  const double dt = 0.01;
  world.setTimeStep(dt);
  world.enterSimulationMode();

  const Eigen::Vector3d gravity = world.getGravity();
  body.applyForce(-mass * gravity); // upward force = m*g

  sxc::MultibodyVariationalState state;
  const auto report = sxc::integrateMultibodyVariational(
      world.getRegistry(), structureOf(world), gravity, dt, state);

  EXPECT_TRUE(report.converged);
  const Eigen::VectorXd velocity = body.getParentJoint().getVelocity();
  ASSERT_EQ(velocity.size(), 6);
  EXPECT_LT(velocity.norm(), 1e-9) << "velocity = " << velocity.transpose();
}

// Chain deflection (variational): a fixed-base horizontal revolute chain with a
// lateral force applied at the tip each step deflects the tip in the force's
// direction (sign/direction sanity through the public World::step() path).
TEST(ExternalForce, VariationalChainTipDeflectsInForceDirection)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero()); // isolate the applied force
  world.setMultibodyOptions({.integrationFamily = "variational integrator"});

  auto robot = world.addMultibody("chain");
  auto parent = robot.addLink("base");
  const int links = 3;
  for (int i = 0; i < links; ++i) {
    Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
    offset.translation() = Eigen::Vector3d(0.4, 0.0, 0.0);
    sx::JointSpec spec;
    spec.name = "j" + std::to_string(i);
    spec.type = sx::JointType::Revolute;
    spec.axis = Eigen::Vector3d::UnitZ(); // rotate in the x-y plane
    spec.transformFromParent = offset;
    auto link = robot.addLink("l" + std::to_string(i), parent, spec);
    link.setMass(1.0);
    link.setInertia(Eigen::Vector3d(0.02, 0.02, 0.02).asDiagonal());
    parent = link;
  }

  const double dt = 1e-3;
  world.setTimeStep(dt);
  world.enterSimulationMode();
  world.updateKinematics();

  auto tip = parent;
  const double initialY = tip.getTransform().translation().y();
  const Eigen::Vector3d force(0.0, 5.0, 0.0); // +y lateral push at the tip
  for (int k = 0; k < 50; ++k) {
    tip.applyForce(force);
    world.step();
  }
  const double finalY = tip.getTransform().translation().y();
  EXPECT_GT(finalY - initialY, 1e-4)
      << "tip moved from y=" << initialY << " to y=" << finalY;
}

// One-shot clear (semi-implicit, public step path): a force applied once does
// not persist to the next step. A second step with no re-apply matches a
// reference world that never had a force applied.
TEST(ExternalForce, SemiImplicitForceIsClearedAfterStep)
{
  const double mass = 2.0;
  const double dt = 0.01;
  const Eigen::Vector3d force(1.0, 2.0, 3.0);

  // Reference world: no external force ever applied.
  sx::World reference;
  reference.setGravity(Eigen::Vector3d::Zero());
  auto referenceBody = addFloatingBody(reference, mass);
  reference.setTimeStep(dt);
  reference.enterSimulationMode();
  reference.step(); // step 1 (no force)
  reference.step(); // step 2 (no force)
  const Eigen::VectorXd referenceVelocity
      = referenceBody.getParentJoint().getVelocity();

  // Test world: apply the force only before the first step.
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  auto body = addFloatingBody(world, mass);
  world.setTimeStep(dt);
  world.enterSimulationMode();
  body.applyForce(force);
  world.step(); // step 1 (force consumed here)
  const Eigen::VectorXd afterFirst = body.getParentJoint().getVelocity();
  world.step(); // step 2 (force must already be cleared)
  const Eigen::VectorXd afterSecond = body.getParentJoint().getVelocity();

  // The force acted on the first step (velocity changed)...
  EXPECT_GT(afterFirst.head<3>().norm(), 1e-9);
  // ...and the velocity does not change on the second, force-free step.
  EXPECT_TRUE(afterSecond.isApprox(afterFirst, 1e-12))
      << "after first = " << afterFirst.transpose()
      << " after second = " << afterSecond.transpose();
  // The reference (never forced) stays at rest, confirming the only motion came
  // from the single one-shot force.
  EXPECT_LT(referenceVelocity.norm(), 1e-12);
}

// One-shot clear (variational, public step path): mirrors the semi-implicit
// one-shot test for the variational integration family.
TEST(ExternalForce, VariationalForceIsClearedAfterStep)
{
  const double mass = 2.0;
  const double dt = 0.01;
  const Eigen::Vector3d force(1.0, 2.0, 3.0);

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions({.integrationFamily = "variational integrator"});
  auto body = addFloatingBody(world, mass);
  world.setTimeStep(dt);
  world.enterSimulationMode();

  body.applyForce(force);
  world.step(); // force consumed here
  const Eigen::VectorXd afterFirst = body.getParentJoint().getVelocity();
  world.step(); // force must already be cleared
  const Eigen::VectorXd afterSecond = body.getParentJoint().getVelocity();

  // The force acted once, then the body coasts at constant velocity (no
  // gravity, no further force), so the velocity is unchanged on the second
  // step.
  EXPECT_GT(afterFirst.head<3>().norm(), 1e-9);
  EXPECT_TRUE(afterSecond.isApprox(afterFirst, 1e-9))
      << "after first = " << afterFirst.transpose()
      << " after second = " << afterSecond.transpose();
}

//==============================================================================
// EXPERIMENTAL SPIKE (PLAN-082 contact-roadmap gate 2): in-loop compliant
// contact on a single-contact articulated scene.
//
// Confirms the central Phase C premise: a SMOOTH bounded (compliant/penalty)
// contact force, evaluated at the trial q^{k+1} on every RIQN iteration and
// folded into the forced-DEL residual on the forcing side, keeps the
// variational integrator's root-find robust (the bounded curvature is
// tolerated). Scene: a vertical prismatic slider (Euclidean coordinate, so the
// exact-Newton recursive-Jacobian RIQN path -- the one carrying the O(n)
// articulated structure) carrying a point mass, dropped onto a hard-coded
// ground plane z = 0. The hook applies a one-sided linear spring F = k(-z)
// upward at the carriage origin when it penetrates, mapped to generalized force
// by the trial-config point Jacobian via `variationalContactPointForce`.
//
// Spike scope: a hard-coded ground plane (no collision query; that is gate 1).
namespace {

// Build a vertical prismatic slider (fixed base at the origin + a z-axis
// prismatic carriage of the given mass). Returns the carriage Link handle; its
// joint position[0] is the carriage world height z.
sx::Link addVerticalSlider(sx::World& world, double mass)
{
  auto robot = world.addMultibody("slider");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "rail";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto carriage = robot.addLink("carriage", base, spec);
  carriage.setMass(mass);
  carriage.setInertia(Eigen::Vector3d(0.01, 0.01, 0.01).asDiagonal());
  return carriage;
}

// Result of dropping the slider onto the plane with a given contact stiffness.
struct SpikeDrop
{
  std::size_t maxIters = 0;     ///< peak RIQN iterations over the drop.
  double meanIters = 0.0;       ///< mean RIQN iterations over the drop.
  double restPenetration = 0.0; ///< |z| at the final (settled) step.
  bool allConverged = true;     ///< every step converged within budget.
  bool finite = true;           ///< no NaN/Inf in the trajectory.
};

// Drop the slider from `z0` under gravity for `steps` steps with a compliant
// one-sided penalty contact of stiffness `k` against the plane z = 0, applied
// through the in-loop variational contact hook. The carriage is the second
// (last) link, so its trial world height is linkWorldTransforms.back().
SpikeDrop dropOntoPlane(double mass, double k, double z0, double dt, int steps)
{
  sx::World world; // gravity (0, 0, -9.81)
  auto carriage = addVerticalSlider(world, mass);
  world.setTimeStep(dt);
  world.enterSimulationMode();
  carriage.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, z0));
  world.updateKinematics();

  auto& registry = world.getRegistry();
  const auto& structure = structureOf(world);
  const Eigen::Vector3d gravity = world.getGravity();

  // One-sided linear-spring penalty at the carriage origin: F = k(-z) up when
  // z < 0, evaluated at the trial configuration each RIQN iteration. The
  // carriage is structure.links.back() (base, carriage), so its trial-config
  // world height is the last linkWorldTransforms entry.
  const sxc::VariationalContactHook hook
      = [k](const sxc::VariationalContactContext& context) -> Eigen::VectorXd {
    const std::size_t carriageIndex = context.linkWorldTransforms.size() - 1;
    const double z
        = context.linkWorldTransforms[carriageIndex].translation().z();
    Eigen::VectorXd contactForce
        = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(context.dofCount));
    if (z < 0.0) {
      const Eigen::Vector3d worldForce(0.0, 0.0, k * (-z)); // upward penalty
      contactForce += sxc::variationalContactPointForce(
          context, carriageIndex, Eigen::Vector3d::Zero(), worldForce);
    }
    return contactForce;
  };

  SpikeDrop result;
  sxc::MultibodyVariationalState state;
  std::size_t totalIters = 0;
  for (int step = 0; step < steps; ++step) {
    const auto report = sxc::integrateMultibodyVariational(
        registry,
        structure,
        gravity,
        dt,
        state,
        /*maxIterations=*/100,
        /*tolerance=*/1e-10,
        /*constraints=*/{},
        /*andersonDepth=*/5,
        hook);
    result.allConverged = result.allConverged && report.converged;
    if (std::isnan(report.residualNorm)
        || !std::isfinite(report.residualNorm)) {
      result.finite = false;
    }
    totalIters += report.iterations;
    result.maxIters = std::max(result.maxIters, report.iterations);
  }
  result.meanIters = static_cast<double>(totalIters) / steps;
  // Settled height: position[0] is the carriage world z (base at the origin).
  const double finalZ = carriage.getParentJoint().getPosition()[0];
  result.finite = result.finite && std::isfinite(finalZ);
  result.restPenetration = finalZ < 0.0 ? -finalZ : 0.0;
  return result;
}

} // namespace

TEST(VariationalContactSpike, CompliantContactRiqnStaysRobustAcrossStiffness)
{
  const double mass = 1.0;
  const double g = 9.81;
  const double dt = 1e-3;
  const int steps = 3000; // ~3 s: enough to fall and settle.
  const double z0 = 0.05; // start above the plane.

  // Sweep stiffness over mg-scaled values (the bounded-curvature envelope the
  // roadmap claims C1-C3 tolerate). mg-scale = mass*g, so k_scale = K * mg.
  const std::array<double, 3> scales = {1e2, 1e3, 1e4};
  std::array<SpikeDrop, 3> drops{};

  std::ostringstream table;
  table << "\n[C2 spike] compliant single-contact prismatic slider drop"
        << " (mass=" << mass << ", dt=" << dt << ", steps=" << steps << ")\n";
  table << "  k/(mg) |   k (N/m) | max iters | mean iters | rest pen (m) |"
        << " mg/k (m)\n";

  for (std::size_t i = 0; i < scales.size(); ++i) {
    const double k = scales[i] * mass * g;
    drops[i] = dropOntoPlane(mass, k, z0, dt, steps);
    const double analytic = mass * g / k; // expected steady-state penetration.

    table << "  " << std::setw(6) << scales[i] << " | " << std::setw(9)
          << std::fixed << std::setprecision(1) << k << " | " << std::setw(9)
          << drops[i].maxIters << " | " << std::setw(10) << std::setprecision(2)
          << drops[i].meanIters << " | " << std::setw(12) << std::scientific
          << std::setprecision(3) << drops[i].restPenetration << " | "
          << std::setw(9) << analytic << std::defaultfloat << "\n";

    // Gate-2 robustness: every step converges within budget, no NaN.
    EXPECT_TRUE(drops[i].allConverged)
        << "k/(mg)=" << scales[i] << " failed to converge every step";
    EXPECT_TRUE(drops[i].finite)
        << "k/(mg)=" << scales[i] << " produced NaN/Inf";
    EXPECT_LT(drops[i].maxIters, 100u)
        << "k/(mg)=" << scales[i] << " hit the iteration budget";

    // The body rests near the plane: steady-state penetration ~ mg/k, bounded
    // and small (a soft contact leaves a small residual penetration). Allow a
    // generous band around the analytic spring compression to absorb the
    // discrete-time settle.
    EXPECT_LT(drops[i].restPenetration, 5.0 * analytic + 1e-4)
        << "k/(mg)=" << scales[i] << " penetration " << drops[i].restPenetration
        << " exceeds the mg/k envelope " << analytic;
    EXPECT_GE(drops[i].restPenetration, 0.0);
  }

  // Penetration strictly decreases with stiffness (the mg/k trend).
  EXPECT_GT(drops[0].restPenetration, drops[1].restPenetration);
  EXPECT_GT(drops[1].restPenetration, drops[2].restPenetration);

  std::cout << table.str() << std::flush;
  RecordProperty("c2_spike_table", table.str());
}

// EXPERIMENTAL SPIKE: the default-off path is numerically identical. An empty
// contact hook must reproduce the no-hook trajectory byte-for-byte (the opt-in
// hook adds zero overhead and changes no numerics when unset).
TEST(VariationalContactSpike, EmptyHookReproducesNoContactTrajectoryExactly)
{
  const auto run = [](bool passEmptyHook) {
    sx::World world;
    auto robot = world.addMultibody("chain");
    auto parent = robot.addLink("base");
    for (int i = 0; i < 3; ++i) {
      Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
      offset.translation() = Eigen::Vector3d(i == 0 ? 0.0 : 0.3, 0.0, 0.0);
      sx::JointSpec spec;
      spec.name = "j" + std::to_string(i);
      spec.type = sx::JointType::Revolute;
      spec.axis = Eigen::Vector3d::UnitY();
      spec.transformFromParent = offset;
      auto link = robot.addLink("l" + std::to_string(i), parent, spec);
      link.setMass(1.0);
      link.setInertia(Eigen::Vector3d(0.03, 0.03, 0.03).asDiagonal());
      parent = link;
    }
    const double dt = 1e-3;
    world.setTimeStep(dt);
    world.enterSimulationMode();
    for (auto joint : robot.getJoints()) {
      joint.setPosition(Eigen::VectorXd::Constant(1, 0.3));
    }
    world.updateKinematics();

    auto& registry = world.getRegistry();
    const auto& structure = structureOf(world);
    const Eigen::Vector3d gravity = world.getGravity();

    sxc::MultibodyVariationalState state;
    std::vector<double> trajectory;
    const sxc::VariationalContactHook emptyHook; // default-constructed: unset.
    for (int step = 0; step < 200; ++step) {
      if (passEmptyHook) {
        sxc::integrateMultibodyVariational(
            registry,
            structure,
            gravity,
            dt,
            state,
            100,
            1e-10,
            {},
            5,
            emptyHook);
      } else {
        sxc::integrateMultibodyVariational(
            registry, structure, gravity, dt, state);
      }
      for (auto joint : robot.getJoints()) {
        trajectory.push_back(joint.getPosition()[0]);
      }
    }
    return trajectory;
  };

  const std::vector<double> noHook = run(false);
  const std::vector<double> emptyHook = run(true);
  ASSERT_EQ(noHook.size(), emptyHook.size());
  // Byte-for-byte identical: an unset hook touches no numerics.
  for (std::size_t i = 0; i < noHook.size(); ++i) {
    EXPECT_EQ(noHook[i], emptyHook[i]) << "trajectory diverged at sample " << i;
  }
}

// ===========================================================================
// PLAN-082 Phase C rung C2 -- compliant ground contact via a real, configurable
// query. makeVariationalGroundContactHook promotes the gate-2 spike's
// hard-coded z = 0 plane into an analytic half-space + body-fixed contact
// points (the real distance/gradient query for the link-point-vs-ground case),
// with the VBD/XPBD quadratic-penalty force law F = k max(0,-d) n.
// ===========================================================================

// A prismatic slider dropped onto a plane at a NON-zero offset settles at
// (offset - mg/k): proves the plane is a real configurable query (not the
// hard-coded z = 0) and the compliant penalty leaves the bounded mg/k residual.
TEST(VariationalGroundContact, CompliantSliderRestsBelowConfigurablePlane)
{
  const double mass = 1.0;
  const double g = 9.81;
  const double dt = 1.0e-3;
  const double planeZ = 0.1; // a plane well away from the hard-coded z = 0.
  const double k = 1.0e3 * mass * g; // inside the k <= 1e4 mg envelope.
  const int steps = 3000;

  sx::World world;
  auto carriage = addVerticalSlider(world, mass);
  world.setTimeStep(dt);
  world.enterSimulationMode();
  carriage.getParentJoint().setPosition(
      Eigen::VectorXd::Constant(1, planeZ + 0.05)); // start above the plane.
  world.updateKinematics();

  const auto& structure = structureOf(world);
  sxc::VariationalGroundContact contact;
  contact.planeNormal = Eigen::Vector3d::UnitZ();
  contact.planePoint = Eigen::Vector3d(0.0, 0.0, planeZ);
  contact.stiffness = k;
  contact.points.push_back(
      {structure.links.size() - 1,
       Eigen::Vector3d::Zero()}); // carriage origin.
  const auto hook = sxc::makeVariationalGroundContactHook(contact);

  auto& registry = world.getRegistry();
  const Eigen::Vector3d gravity = world.getGravity();
  sxc::MultibodyVariationalState state;
  for (int step = 0; step < steps; ++step) {
    const auto report = sxc::integrateMultibodyVariational(
        registry, structure, gravity, dt, state, 100, 1e-10, {}, 5, hook);
    ASSERT_TRUE(report.converged) << "step " << step;
    ASSERT_TRUE(std::isfinite(report.residualNorm)) << "step " << step;
  }

  const double finalZ = carriage.getParentJoint().getPosition()[0];
  const double analytic = mass * g / k; // expected mg/k penetration.
  // Settles just below the configurable plane (z ~ planeZ - mg/k); a still-
  // hard-coded z = 0 query would instead settle near -mg/k, far below.
  EXPECT_NEAR(finalZ, planeZ - analytic, 2.0 * analytic + 1.0e-4);
  EXPECT_GT(finalZ, planeZ - 0.02);
}

// A revolute chain whose last link carries a tip contact point: a ground plane
// below the chain stops the tip from swinging through it. Compared against the
// contact-free swing (which dips far below), this exercises the *rotational*
// contact point Jacobian R (J_linear - [p] J_angular) a slider cannot.
TEST(VariationalGroundContact, CompliantContactStopsRevoluteChainTunneling)
{
  const double dt = 1.0e-3;
  const int steps = 1500;
  const double linkLen = 0.3;
  const double planeZ = -0.05;
  const double k = 5.0e3;

  struct ChainRun
  {
    double minTipZ = 1.0e9;
    bool finite = true;
    bool converged = true;
  };

  const auto runChain = [&](bool withContact) {
    sx::World world;
    auto robot = world.addMultibody("chain");
    auto parent = robot.addLink("base");
    sx::Link tip = parent;
    for (int i = 0; i < 2; ++i) {
      Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
      offset.translation() = Eigen::Vector3d(i == 0 ? 0.0 : linkLen, 0.0, 0.0);
      sx::JointSpec spec;
      spec.name = "j" + std::to_string(i);
      spec.type = sx::JointType::Revolute;
      spec.axis = Eigen::Vector3d::UnitY();
      spec.transformFromParent = offset;
      tip = robot.addLink("l" + std::to_string(i), parent, spec);
      tip.setMass(1.0);
      tip.setInertia(Eigen::Vector3d(0.03, 0.03, 0.03).asDiagonal());
      parent = tip;
    }
    world.setTimeStep(dt);
    world.enterSimulationMode();
    world.updateKinematics(); // start horizontal (joints at 0); gravity swings.

    const auto& structure = structureOf(world);
    const std::size_t tipLink = structure.links.size() - 1;
    const Eigen::Vector3d tipLocal(linkLen, 0.0, 0.0); // far end of last link.

    sxc::VariationalContactHook hook;
    if (withContact) {
      sxc::VariationalGroundContact contact;
      contact.planeNormal = Eigen::Vector3d::UnitZ();
      contact.planePoint = Eigen::Vector3d(0.0, 0.0, planeZ);
      contact.stiffness = k;
      contact.points.push_back({tipLink, tipLocal});
      hook = sxc::makeVariationalGroundContactHook(contact);
    }

    auto& registry = world.getRegistry();
    const Eigen::Vector3d gravity = world.getGravity();
    sxc::MultibodyVariationalState state;
    ChainRun run;
    for (int step = 0; step < steps; ++step) {
      const auto report = sxc::integrateMultibodyVariational(
          registry, structure, gravity, dt, state, 100, 1e-10, {}, 5, hook);
      run.converged = run.converged && report.converged;
      run.finite = run.finite && std::isfinite(report.residualNorm);
      world.updateKinematics();
      const double tipZ = (tip.getWorldTransform() * tipLocal).z();
      run.minTipZ = std::min(run.minTipZ, tipZ);
    }
    return run;
  };

  const ChainRun freeSwing = runChain(false);
  const ChainRun held = runChain(true);

  EXPECT_TRUE(freeSwing.finite);
  EXPECT_TRUE(freeSwing.converged);
  EXPECT_TRUE(held.finite);
  EXPECT_TRUE(held.converged);
  // Contact-free, the tip swings well below the plane.
  EXPECT_LT(freeSwing.minTipZ, planeZ - 0.1);
  // With contact the tip is stopped near the plane (a bounded penalty
  // penetration), nowhere near the free swing.
  EXPECT_GT(held.minTipZ, planeZ - 0.05);
  EXPECT_GT(held.minTipZ, freeSwing.minTipZ + 0.1);
}

// The factory's no-op path (zero stiffness) leaves the trajectory identical to
// the no-contact integrator (the compliant force is exactly zero, folded as
// residual -= dt * 0).
TEST(VariationalGroundContact, ZeroStiffnessMatchesNoContact)
{
  const auto run = [](bool withZeroHook) {
    sx::World world;
    auto carriage = addVerticalSlider(world, 1.0);
    world.setTimeStep(1.0e-3);
    world.enterSimulationMode();
    carriage.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, 0.5));
    world.updateKinematics();

    const auto& structure = structureOf(world);
    sxc::VariationalContactHook hook;
    if (withZeroHook) {
      sxc::VariationalGroundContact contact; // stiffness defaults to 0.
      contact.points.push_back(
          {structure.links.size() - 1, Eigen::Vector3d::Zero()});
      hook = sxc::makeVariationalGroundContactHook(contact);
    }
    auto& registry = world.getRegistry();
    const Eigen::Vector3d gravity = world.getGravity();
    sxc::MultibodyVariationalState state;
    std::vector<double> trajectory;
    for (int step = 0; step < 100; ++step) {
      sxc::integrateMultibodyVariational(
          registry, structure, gravity, 1.0e-3, state, 100, 1e-10, {}, 5, hook);
      trajectory.push_back(carriage.getParentJoint().getPosition()[0]);
    }
    return trajectory;
  };
  const std::vector<double> noContact = run(false);
  const std::vector<double> zeroStiff = run(true);
  ASSERT_EQ(noContact.size(), zeroStiff.size());
  for (std::size_t i = 0; i < noContact.size(); ++i) {
    EXPECT_NEAR(noContact[i], zeroStiff[i], 1.0e-12) << "diverged at " << i;
  }
}

// Degenerate configs are rejected at hook construction (no silent bad contact).
TEST(VariationalGroundContact, RejectsDegenerateConfig)
{
  sxc::VariationalGroundContact zeroNormal;
  zeroNormal.planeNormal = Eigen::Vector3d::Zero();
  zeroNormal.stiffness = 1.0;
  EXPECT_ANY_THROW((void)sxc::makeVariationalGroundContactHook(zeroNormal));

  sxc::VariationalGroundContact negativeStiffness;
  negativeStiffness.stiffness = -1.0;
  EXPECT_ANY_THROW(
      (void)sxc::makeVariationalGroundContactHook(negativeStiffness));

  sxc::VariationalGroundContact frictionNoEps;
  frictionNoEps.stiffness = 1.0;
  frictionNoEps.frictionCoefficient = 0.5;
  frictionNoEps.frictionRegularization = 0.0; // invalid with friction enabled.
  EXPECT_ANY_THROW((void)sxc::makeVariationalGroundContactHook(frictionNoEps));
}

// PLAN-082 Phase C rung C1 -- lagged regularized-Coulomb friction. A block
// sliding on the ground decelerates to rest with friction (kinetic friction
// ~ mu*mg opposing motion) but slides freely without it, exercising the
// q^k-anchored tangential-sliding term and the mu*|Fn| saturation.
TEST(VariationalGroundContact, LaggedFrictionDeceleratesSlidingBlock)
{
  const double mass = 1.0;
  const double g = 9.81;
  const double dt = 1.0e-3;
  const double k = 1.0e4;
  const double v0 = 1.0; // initial horizontal slide speed.
  const int steps = 2000;

  struct SlideRun
  {
    double finalXSpeed = 0.0;
    double xDisplacement = 0.0;
    double restZ = 0.0;
    bool finite = true;
    bool converged = true;
  };

  // A planar block: base -> prismatic X -> carrier -> prismatic Z -> block, so
  // the block has horizontal (slide) and vertical (settle) freedom.
  const auto runSlide = [&](double mu) {
    sx::World world;
    auto robot = world.addMultibody("block");
    auto base = robot.addLink("base");
    sx::JointSpec xspec;
    xspec.name = "x";
    xspec.type = sx::JointType::Prismatic;
    xspec.axis = Eigen::Vector3d::UnitX();
    auto carrier = robot.addLink("carrier", base, xspec);
    carrier.setMass(0.01);
    carrier.setInertia(Eigen::Vector3d(1.0e-4, 1.0e-4, 1.0e-4).asDiagonal());
    sx::JointSpec zspec;
    zspec.name = "z";
    zspec.type = sx::JointType::Prismatic;
    zspec.axis = Eigen::Vector3d::UnitZ();
    auto block = robot.addLink("block", carrier, zspec);
    block.setMass(mass);
    block.setInertia(Eigen::Vector3d(0.01, 0.01, 0.01).asDiagonal());

    world.setTimeStep(dt);
    world.enterSimulationMode();
    block.getParentJoint().setPosition(
        Eigen::VectorXd::Constant(
            1,
            -mass * g
                / k)); // start at the resting penetration: steady contact,
                       // no elastic bounce (clean kinetic-friction test).
    carrier.getParentJoint().setVelocity(
        Eigen::VectorXd::Constant(1, v0)); // initial horizontal slide.
    world.updateKinematics();

    const auto& structure = structureOf(world);
    sxc::VariationalGroundContact contact;
    contact.planeNormal = Eigen::Vector3d::UnitZ();
    contact.planePoint = Eigen::Vector3d::Zero();
    contact.stiffness = k;
    contact.frictionCoefficient = mu;
    contact.frictionRegularization = 1.0e-4;
    contact.points.push_back(
        {structure.links.size() - 1, Eigen::Vector3d::Zero()}); // block origin.
    const auto hook = sxc::makeVariationalGroundContactHook(contact);

    auto& registry = world.getRegistry();
    const Eigen::Vector3d gravity = world.getGravity();
    sxc::MultibodyVariationalState state;
    SlideRun run;
    for (int step = 0; step < steps; ++step) {
      const auto report = sxc::integrateMultibodyVariational(
          registry, structure, gravity, dt, state, 100, 1e-10, {}, 5, hook);
      run.converged = run.converged && report.converged;
      run.finite = run.finite && std::isfinite(report.residualNorm);
    }
    run.finalXSpeed = std::abs(carrier.getParentJoint().getVelocity()[0]);
    run.xDisplacement = std::abs(carrier.getParentJoint().getPosition()[0]);
    run.restZ = block.getParentJoint().getPosition()[0];
    return run;
  };

  const SlideRun frictionless = runSlide(0.0);
  const SlideRun frictional = runSlide(0.5);

  EXPECT_TRUE(frictionless.finite);
  EXPECT_TRUE(frictionless.converged);
  EXPECT_TRUE(frictional.finite);
  EXPECT_TRUE(frictional.converged);
  // Both settle onto the plane with the bounded mg/k penetration.
  EXPECT_NEAR(frictional.restZ, -mass * g / k, 5.0e-3);
  // Frictionless keeps sliding at ~v0; friction decelerates it to near rest.
  EXPECT_GT(frictionless.finalXSpeed, 0.8 * v0);
  EXPECT_LT(frictional.finalXSpeed, 0.1 * v0);
  // And friction travels far less.
  EXPECT_LT(frictional.xDisplacement, 0.5 * frictionless.xDisplacement);
}

// PLAN-082 Phase C rung C3 -- augmented-Lagrangian contact. The per-contact
// dual accumulates the steady contact load, so the resting penetration is
// centered at ~0 (the dual carries the weight) instead of the pure-penalty
// -mg/k offset. The symplectic VI is undamped, so the AL block oscillates about
// d=0; we compare the time-averaged contact-point height over a trailing
// window.
TEST(
    VariationalGroundContact,
    AugmentedLagrangianCentersContactAtZeroPenetration)
{
  const double mass = 1.0;
  const double g = 9.81;
  const double dt = 1.0e-3;
  const double k = 1.0e3 * mass * g; // soft: pure-penalty rest is mg/k.
  const int steps = 1500;
  const int window = 500;      // trailing average over several oscillations.
  const int dualInterval = 20; // AL outer loop: update the dual after the
                               // damped inner dynamics settle (not every step).
  const double penaltyPenetration = mass * g / k;

  struct Result
  {
    double meanZ = 0.0;
    double meanDual = 0.0;
  };

  const auto run = [&](bool useAl) {
    sx::World world;
    auto carriage = addVerticalSlider(world, mass);
    world.setTimeStep(dt);
    world.enterSimulationMode();
    carriage.getParentJoint().setPosition(
        Eigen::VectorXd::Constant(
            1, -penaltyPenetration)); // start at the pure-penalty rest.
    world.updateKinematics();

    const auto& structure = structureOf(world);
    const std::size_t carriageIndex = structure.links.size() - 1;
    sxc::VariationalGroundContact contact;
    contact.planeNormal = Eigen::Vector3d::UnitZ();
    contact.planePoint = Eigen::Vector3d::Zero();
    contact.stiffness = k;
    contact.dampingCoefficient = 200.0; // ~critical (2*sqrt(k*m)) to settle.
    contact.points.push_back({carriageIndex, Eigen::Vector3d::Zero()});

    sxc::VariationalGroundContactSolver solver(contact);
    const sxc::VariationalContactHook penaltyHook
        = sxc::makeVariationalGroundContactHook(contact);
    const sxc::VariationalContactHook hook
        = useAl ? solver.hook() : penaltyHook;

    auto& registry = world.getRegistry();
    const Eigen::Vector3d gravity = world.getGravity();
    sxc::MultibodyVariationalState state;
    std::vector<Eigen::Isometry3d> transforms(
        structure.links.size(), Eigen::Isometry3d::Identity());
    Result result;
    int count = 0;
    for (int step = 0; step < steps; ++step) {
      const auto report = sxc::integrateMultibodyVariational(
          registry, structure, gravity, dt, state, 100, 1e-10, {}, 5, hook);
      EXPECT_TRUE(report.converged);
      world.updateKinematics();
      transforms[carriageIndex] = carriage.getWorldTransform();
      // AL outer loop: advance the dual only after the damped inner dynamics
      // settle toward force balance (updating every step would overshoot).
      if (useAl && (step + 1) % dualInterval == 0) {
        solver.updateDuals(transforms);
      }
      if (step >= steps - window) {
        result.meanZ += carriage.getParentJoint().getPosition()[0];
        result.meanDual += useAl ? solver.duals()[0] : 0.0;
        ++count;
      }
    }
    result.meanZ /= count;
    result.meanDual /= count;
    return result;
  };

  const Result penalty = run(false);
  const Result al = run(true);

  // Pure penalty rests in steady penetration (~ -mg/k).
  EXPECT_LT(penalty.meanZ, -0.5 * penaltyPenetration);
  // AL centers the contact at ~0: the dual carries the weight, removing the
  // steady penetration offset.
  EXPECT_LT(std::abs(al.meanZ), 0.3 * penaltyPenetration);
  // The dual converged near the steady contact force mg.
  EXPECT_GT(al.meanDual, 0.5 * mass * g);
  EXPECT_LT(al.meanDual, 2.0 * mass * g);
}

// PLAN-082 Phase C: compliant ground contact reaches the integrator through the
// World surface. A slider on the variational-integrator family, configured via
// Multibody::setGroundContact + addGroundContactPoint and stepped with
// world.step() (not the compute API), drops onto the plane and rests at the
// mg/k penetration -- exercising the comps::VariationalContact stage wiring.
TEST(VariationalGroundContact, WorldSurfaceCompliantContactRestsOnGround)
{
  const double mass = 1.0;
  const double g = 9.81;
  const double dt = 1.0e-3;
  const double k = 1.0e3 * mass * g;
  const double damping = 200.0; // ~critical (2*sqrt(k*m)) to settle the drop.
  const int steps = 3000;

  sx::World world;
  world.setMultibodyOptions({.integrationFamily = "variational integrator"});
  auto robot = world.addMultibody("slider");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "rail";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto carriage = robot.addLink("carriage", base, spec);
  carriage.setMass(mass);
  carriage.setInertia(Eigen::Vector3d(0.01, 0.01, 0.01).asDiagonal());
  world.setTimeStep(dt);
  world.enterSimulationMode();
  carriage.getParentJoint().setPosition(
      Eigen::VectorXd::Constant(1, 0.02)); // start above the plane.
  world.updateKinematics();

  // Configure compliant ground contact through the World surface (the new stage
  // wiring), not the compute API.
  robot.setGroundContact(
      Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d::Zero(),
      k,
      /*frictionCoefficient=*/0.0,
      /*frictionRegularization=*/1.0e-4,
      damping);
  robot.addGroundContactPoint(carriage, Eigen::Vector3d::Zero());

  for (int step = 0; step < steps; ++step) {
    world.step();
  }

  const double finalZ = carriage.getParentJoint().getPosition()[0];
  const double analytic = mass * g / k; // mg/k penetration.
  EXPECT_TRUE(std::isfinite(finalZ));
  EXPECT_LT(finalZ, 0.0);   // settled in contact, below the plane.
  EXPECT_GT(finalZ, -0.01); // held near the plane (not free-falling through).
  EXPECT_NEAR(finalZ, -analytic, 1.5 * analytic); // ~mg/k penetration.
}
