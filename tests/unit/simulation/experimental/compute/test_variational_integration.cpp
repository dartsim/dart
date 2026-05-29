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
#include <dart/simulation/experimental/multibody/multibody.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <entt/entt.hpp>
#include <gtest/gtest.h>

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
    world.setMultibodyIntegrationMethod("variational integrator");
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

  // Measured (deterministic) on this scene: viBand ~= 0.0078, seBand ~= 0.39,
  // viSlope ~= 5.9e-6, seSlope ~= 4.0e-3 -- the VI conserves ~690x better.

  // (1) Bounded band: VI energy stays within 1%.
  EXPECT_LT(viBand, 1e-2) << "VI energy band " << viBand << " exceeds 1%";
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
