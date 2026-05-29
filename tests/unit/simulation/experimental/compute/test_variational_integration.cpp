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
