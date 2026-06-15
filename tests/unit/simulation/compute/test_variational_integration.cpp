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
#include <dart/simulation/comps/name.hpp>
#include <dart/simulation/comps/variational_contact.hpp>
#include <dart/simulation/compute/multibody_dynamics.hpp>
#include <dart/simulation/compute/variational_integration.hpp>
#include <dart/simulation/constraint/loop_closure_spec.hpp>
#include <dart/simulation/detail/entity_conversion.hpp>
#include <dart/simulation/detail/rigid_avbd/rigid_world_contact.hpp>
#include <dart/simulation/detail/world_registry_access.hpp>
#include <dart/simulation/frame/frame.hpp>
#include <dart/simulation/multibody/multibody.hpp>
#include <dart/simulation/world.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <entt/entt.hpp>
#include <gtest/gtest.h>

#include <array>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <cmath>

namespace {

namespace sx = dart::simulation;
namespace sxc = dart::simulation::compute;
namespace dvbd = dart::simulation::detail::deformable_vbd;

// Returns the (single) multibody structure component in the world.
const sx::comps::MultibodyStructure& structureOf(sx::World& world)
{
  auto& registry = dart::simulation::detail::registryOf(world);
  auto view = registry.view<sx::comps::MultibodyStructure>();
  return registry.get<sx::comps::MultibodyStructure>(*view.begin());
}

double signedRotationAroundAxis(
    const Eigen::Matrix3d& rotation, const Eigen::Vector3d& axis)
{
  const Eigen::Vector3d unitAxis = axis.normalized();
  const Eigen::AngleAxisd angleAxis(rotation);
  return angleAxis.angle() * angleAxis.axis().dot(unitAxis);
}

void expectPassiveChainEnergyDrift(
    double dt,
    int steps,
    int sampleEvery,
    double maxViBand,
    double minSeBand,
    double slopeAdvantage)
{
  constexpr int kLinks = 10;

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
    auto& registry = dart::simulation::detail::registryOf(world);
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
    auto& registry = dart::simulation::detail::registryOf(world);
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

  // The VI conserves substantially better than semi-implicit Euler on this
  // scene. The *band* is trajectory-sensitive (the chain is chaotic, so the
  // per-step root -- found to 1e-10 either way -- compounds into a slightly
  // different rollout under Anderson acceleration vs the plain step); the
  // no-secular-drift *slope* below is the falsifiable gate per the plan, and
  // thresholds are tuned to recorded measurements.

  // (1) Bounded band: VI energy oscillation stays small.
  EXPECT_LT(viBand, maxViBand) << "VI energy band " << viBand << " too large";
  // (2) Contrast: semi-implicit Euler on the same scene drifts measurably.
  EXPECT_GT(seBand, minSeBand)
      << "semi-implicit Euler did not drift beyond the band as expected; band "
      << seBand;
  // (3) No secular drift: the VI's net fitted energy trend is a small fraction
  //     of its oscillation band (slope ~= 0 within the oscillation noise).
  EXPECT_LT(viTrend, 0.25 * viBand) << "VI net energy trend " << viTrend
                                    << " is not small vs band " << viBand;
  // (4) The VI's drift slope is smaller than semi-implicit Euler's.
  EXPECT_LT(viSlope * slopeAdvantage, seSlope)
      << "VI slope " << viSlope << " not >=" << slopeAdvantage
      << "x better than semi-implicit slope " << seSlope;
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
      dart::simulation::detail::registryOf(world),
      structureOf(world),
      world.getGravity(),
      dt,
      state);

  EXPECT_TRUE(report.converged);
  auto joint = carriage.getParentJoint();
  EXPECT_NEAR(joint.getAcceleration()[0], -9.81, 1e-9);
  EXPECT_NEAR(joint.getVelocity()[0], -9.81 * dt, 1e-9);
  EXPECT_NEAR(joint.getPosition()[0], -9.81 * dt * dt, 1e-9);
}

TEST(VariationalIntegration, VelocityActuatorSinglePrismaticCommandsVelocity)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  auto robot = world.addMultibody("slider");
  auto base = robot.addLink("base");

  sx::JointSpec spec;
  spec.name = "rail";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto carriage = robot.addLink("carriage", base, spec);
  carriage.setMass(2.0);

  auto joint = carriage.getParentJoint();
  joint.setActuatorType(sx::ActuatorType::Velocity);
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, 0.5));
  joint.setForce(Eigen::VectorXd::Constant(1, 100.0));

  const double dt = 0.01;
  world.setTimeStep(dt);
  world.enterSimulationMode();
  world.step();

  EXPECT_NEAR(joint.getVelocity()[0], 0.5, 1e-12);
  EXPECT_NEAR(joint.getPosition()[0], 0.5 * dt, 1e-12);
  EXPECT_NEAR(joint.getAcceleration()[0], 0.5 / dt, 1e-12);
}

TEST(VariationalIntegration, VelocityActuatorCoupledRevoluteCommandsVelocity)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto robot = world.addMultibody("double_pendulum");
  auto base = robot.addLink("base");
  const auto offset = [](double x) {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(x, 0.0, 0.0);
    return transform;
  };

  sx::JointSpec spec1;
  spec1.name = "j1";
  spec1.type = sx::JointType::Revolute;
  spec1.axis = Eigen::Vector3d::UnitY();
  spec1.transformFromParent = offset(0.7);
  auto link1 = robot.addLink("link1", base, spec1);
  link1.setMass(1.5);
  link1.setInertia(Eigen::Vector3d(0.05, 0.08, 0.05).asDiagonal());

  sx::JointSpec spec2;
  spec2.name = "j2";
  spec2.type = sx::JointType::Revolute;
  spec2.axis = Eigen::Vector3d::UnitY();
  spec2.transformFromParent = offset(0.6);
  auto link2 = robot.addLink("link2", link1, spec2);
  link2.setMass(1.0);
  link2.setInertia(Eigen::Vector3d(0.04, 0.06, 0.04).asDiagonal());

  auto joint1 = link1.getParentJoint();
  auto joint2 = link2.getParentJoint();
  joint1.setActuatorType(sx::ActuatorType::Velocity);
  joint2.setActuatorType(sx::ActuatorType::Velocity);
  joint1.setCommandVelocity(Eigen::VectorXd::Constant(1, 0.3));
  joint2.setCommandVelocity(Eigen::VectorXd::Constant(1, -0.4));

  world.setTimeStep(0.005);
  world.enterSimulationMode();
  world.step();

  EXPECT_NEAR(joint1.getVelocity()[0], 0.3, 1e-12);
  EXPECT_NEAR(joint2.getVelocity()[0], -0.4, 1e-12);
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
      dart::simulation::detail::registryOf(world),
      structureOf(world),
      world.getGravity(),
      dt,
      state);

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

  auto& registry = dart::simulation::detail::registryOf(world);
  const auto& structure = structureOf(world);
  const Eigen::Vector3d gravity = world.getGravity();

  // Energy is read through the public solver-agnostic metrics surface
  // (PLAN-091 WP-091.24). For this single-multibody scene
  // World::computeStepMetrics().totalEnergy equals
  // computeMultibodyMechanicalEnergy(registry, structure, gravity) exactly (the
  // facade reuses that same helper), so the conservation bound below is
  // unchanged -- this just proves the public metrics are usable in place of the
  // internal registry read.
  const double energy0 = world.computeStepMetrics().totalEnergy;
  ASSERT_GT(std::abs(energy0), 1e-6);

  sxc::MultibodyVariationalState state;
  double maxRelativeDrift = 0.0;
  const int steps = 100000;
  for (int k = 0; k < steps; ++k) {
    sxc::integrateMultibodyVariational(registry, structure, gravity, dt, state);
    if (k % 1000 == 0) {
      const double energy = world.computeStepMetrics().totalEnergy;
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
  EXPECT_EQ(
      world.getMultibodyOptions().integrationFamily,
      sx::MultibodyIntegrationFamily::SemiImplicit);
  EXPECT_THROW(
      world.setMultibodyOptions(
          {.integrationFamily
           = static_cast<sx::MultibodyIntegrationFamily>(99)}),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      world.setMultibodyOptions(
          {.integrationFamily = sx::MultibodyIntegrationFamily::Variational,
           .variationalMaxIterations = 0}),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      world.setMultibodyOptions(
          {.integrationFamily = sx::MultibodyIntegrationFamily::Variational,
           .variationalTolerance = 0.0}),
      sx::InvalidArgumentException);
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational,
       .variationalMaxIterations = 120,
       .variationalTolerance = 2e-9});
  EXPECT_EQ(
      world.getMultibodyOptions().integrationFamily,
      sx::MultibodyIntegrationFamily::Variational);
  EXPECT_EQ(world.getMultibodyOptions().variationalMaxIterations, 120u);
  EXPECT_DOUBLE_EQ(world.getMultibodyOptions().variationalTolerance, 2e-9);

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

  // Read total mechanical energy through the public metrics surface
  // (PLAN-091 WP-091.24). For this single-multibody scene it equals
  // computeMultibodyMechanicalEnergy(registry, structure, gravity) exactly, so
  // the conservation bound is unchanged.
  const double energy0 = world.computeStepMetrics().totalEnergy;

  for (int k = 0; k < 20000; ++k) {
    world.step();
  }

  const double energy = world.computeStepMetrics().totalEnergy;
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

  auto& registry = dart::simulation::detail::registryOf(world);
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
    world.setMultibodyOptions(
        {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
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
      dart::simulation::detail::registryOf(world),
      structureOf(world),
      world.getGravity(),
      dt,
      state);

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

  auto& registry = dart::simulation::detail::registryOf(world);
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

  auto& registry = dart::simulation::detail::registryOf(world);
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

// PLAN-084 headline acceptance gate. On a passive 10-link revolute chain (whose
// inertia is strongly configuration-dependent), the variational integrator's
// total mechanical energy stays in a bounded band with *no secular drift*: the
// least-squares slope of energy-vs-time is ~0. Semi-implicit Euler on the very
// same scene -- the default integration family -- conserves measurably worse.
// The no-secular-drift slope is the falsifiable headline gate.
TEST(VariationalIntegration, PassiveChainEnergyHasNoSecularDrift)
{
#ifdef DART_CODECOV
  GTEST_SKIP()
      << "The 100k-step paper-reproduction gate is too slow under coverage; "
         "PassiveChainEnergyCoverageSmoke covers the coverage CI path.";
#endif
  expectPassiveChainEnergyDrift(
      /*dt=*/1e-3,
      /*steps=*/100000,
      /*sampleEvery=*/1000,
      /*maxViBand=*/1.5e-2,
      /*minSeBand=*/5e-2,
      /*slopeAdvantage=*/50.0);
}

// Coverage smoke for the same passive-chain scene. This keeps the coverage job
// below its timeout while the paper-sized acceptance gate above remains active
// in non-coverage builds.
TEST(VariationalIntegration, PassiveChainEnergyCoverageSmoke)
{
  expectPassiveChainEnergyDrift(
      /*dt=*/5e-3,
      /*steps=*/2000,
      /*sampleEvery=*/100,
      /*maxViBand=*/3e-2,
      /*minSeBand=*/5e-2,
      /*slopeAdvantage=*/20.0);
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

  auto& registry = dart::simulation::detail::registryOf(world);
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

// PLAN-084 convergence gate: RIQN converges in a small, bounded number of
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

  auto& registry = dart::simulation::detail::registryOf(world);
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

// Paper-gap smoke: the AVBD articulated path must at least keep the
// 50-link/50,000:1 high mass-ratio pendulum envelope finite and resettable
// through the configured World::step() path. This is stability coverage only,
// not a benchmark or paper-number completeness claim.
TEST(VariationalIntegration, PaperScaleHighRatioChainStaysFiniteAndResets)
{
  constexpr int kLinks = 50;
  constexpr double kLinkLength = 0.45;
  constexpr double kLinkWidth = 0.08;
  constexpr double kLightMass = 1.0;
  constexpr double kTipMass = 50000.0;
  constexpr double kDt = 0.005;

  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(kDt);
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational,
       .variationalMaxIterations = 200,
       .variationalTolerance = 1e-9});

  auto robot = world.addMultibody("paper_scale_high_ratio_chain");
  auto parent = robot.addLink("base");

  std::vector<sx::Joint> joints;
  joints.reserve(kLinks);
  for (int i = 0; i < kLinks; ++i) {
    Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
    offset.translation() = i == 0 ? Eigen::Vector3d::Zero()
                                  : Eigen::Vector3d(kLinkLength, 0.0, 0.0);

    sx::JointSpec spec;
    spec.name = "j" + std::to_string(i);
    spec.type = sx::JointType::Revolute;
    spec.axis = Eigen::Vector3d::UnitY();
    spec.transformFromParent = offset;

    auto link = robot.addLink("l" + std::to_string(i), parent, spec);
    const double mass = i == kLinks - 1 ? kTipMass : kLightMass;
    const double transverseInertia = mass * kLinkLength * kLinkLength / 12.0;
    const double axisInertia = mass * kLinkWidth * kLinkWidth / 6.0;
    link.setMass(mass);
    link.setInertia(
        Eigen::Vector3d(axisInertia, transverseInertia, transverseInertia)
            .asDiagonal());

    joints.push_back(link.getParentJoint());
    parent = link;
  }

  world.enterSimulationMode();
  world.setReplayRecordingEnabled(true);
  ASSERT_EQ(world.getReplayFrameCount(), 1u);
  double maxAbsPosition = 0.0;
  for (int step = 0; step < 32; ++step) {
    ASSERT_NO_THROW(world.step()) << "step " << step;
    for (const auto& joint : joints) {
      ASSERT_TRUE(joint.getPosition().allFinite()) << "step " << step;
      ASSERT_TRUE(joint.getVelocity().allFinite()) << "step " << step;
      maxAbsPosition
          = std::max(maxAbsPosition, std::abs(joint.getPosition()[0]));
    }
  }
  EXPECT_GT(maxAbsPosition, 1e-6);

  world.restoreReplayFrame(0);
  for (const auto& joint : joints) {
    EXPECT_EQ(joint.getPosition().norm(), 0.0);
    EXPECT_EQ(joint.getVelocity().norm(), 0.0);
  }

  ASSERT_NO_THROW(world.step());
  for (const auto& joint : joints) {
    EXPECT_TRUE(joint.getPosition().allFinite());
    EXPECT_TRUE(joint.getVelocity().allFinite());
  }
}

// PLAN-084 convergence gate: non-convergence is a documented hard error, not a
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
          dart::simulation::detail::registryOf(world),
          structureOf(world),
          world.getGravity(),
          dt,
          state,
          /*maxIterations=*/1,
          /*tolerance=*/1e-30),
      sx::InvalidOperationException);
}

// PLAN-084 momentum gate: a force-free floating body (no gravity) conserves
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

  auto& registry = dart::simulation::detail::registryOf(world);
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
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
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

TEST(VariationalIntegration, LoopClosureConstraintScratchIsBaked)
{
  sx::World world;
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
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
    tipFromBase = tipFromBase * offset;
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

  auto& registry = dart::simulation::detail::registryOf(world);
  auto structures = registry.view<sx::comps::MultibodyStructure>();
  ASSERT_FALSE(structures.empty());
  auto& scratch
      = registry.get<sxc::MultibodyVariationalScratch>(*structures.begin());
  const auto constraintCapacity = scratch.constraints.capacity();
  EXPECT_GE(constraintCapacity, 1u);
  EXPECT_EQ(scratch.tree.linkCount(), 6u);
  EXPECT_EQ(scratch.tree.dofCount(), 5u);
  ASSERT_EQ(scratch.step.currentSpatialVelocities.size(), 6u);
  EXPECT_EQ(scratch.step.position.size(), 5);
  EXPECT_EQ(scratch.step.velocity.size(), 5);
  EXPECT_EQ(scratch.step.appliedForce.size(), 5);
  EXPECT_EQ(scratch.step.nextPosition.size(), 5);
  EXPECT_EQ(scratch.step.residual.size(), 5);
  EXPECT_EQ(scratch.step.zeroAcceleration.size(), 5);
  ASSERT_EQ(scratch.anderson.stepDeltas.size(), 5u);
  ASSERT_EQ(scratch.anderson.iterateDeltas.size(), 5u);
  EXPECT_EQ(scratch.anderson.stepDeltas.front().size(), 5);
  EXPECT_EQ(scratch.anderson.iterateDeltas.front().size(), 5);
  EXPECT_EQ(scratch.anderson.stepMatrix.rows(), 5);
  EXPECT_EQ(scratch.anderson.stepMatrix.cols(), 5);
  EXPECT_EQ(scratch.anderson.mixMatrix.rows(), 5);
  EXPECT_EQ(scratch.anderson.mixMatrix.cols(), 5);
  EXPECT_GE(scratch.anderson.jointDelta.size(), 1);
  ASSERT_EQ(scratch.linearSolve.articulated.size(), 6u);
  ASSERT_EQ(scratch.linearSolve.forceProjector.size(), 6u);
  ASSERT_EQ(scratch.linearSolve.motionProjector.size(), 6u);
  EXPECT_EQ(scratch.linearSolve.rhs.size(), 5);
  EXPECT_EQ(scratch.linearSolve.result.size(), 5);
  EXPECT_GE(scratch.linearSolve.jointWork.size(), 1);
  EXPECT_GE(scratch.linearSolve.jointSolveWork.size(), 1);
  EXPECT_EQ(scratch.linearSolve.forceProjector.back().rows(), 6);
  EXPECT_EQ(scratch.linearSolve.forceProjector.back().cols(), 1);
  EXPECT_EQ(scratch.linearSolve.jointMatrix.back().rows(), 1);
  EXPECT_EQ(scratch.linearSolve.jointMatrix.back().cols(), 1);
  ASSERT_EQ(scratch.projection.jacobians.size(), 6u);
  EXPECT_EQ(scratch.projection.jacobians.back().rows(), 6);
  EXPECT_EQ(scratch.projection.jacobians.back().cols(), 5);
  EXPECT_EQ(scratch.projection.residual.size(), 3);
  EXPECT_EQ(scratch.projection.jacobian.rows(), 3);
  EXPECT_EQ(scratch.projection.jacobian.cols(), 5);
  EXPECT_EQ(scratch.projection.inverseMassJt.rows(), 5);
  EXPECT_EQ(scratch.projection.inverseMassJt.cols(), 3);
  EXPECT_EQ(scratch.projection.constraintMass.rows(), 3);
  EXPECT_EQ(scratch.projection.constraintMass.cols(), 3);
  EXPECT_EQ(scratch.projection.correction.size(), 5);

  for (int i = 0; i < 4; ++i) {
    world.step();
    EXPECT_EQ(scratch.constraints.capacity(), constraintCapacity);
    EXPECT_EQ(scratch.tree.linkCount(), 6u);
    EXPECT_EQ(scratch.tree.dofCount(), 5u);
    EXPECT_EQ(scratch.anderson.stepDeltas.size(), 5u);
    EXPECT_EQ(scratch.anderson.stepMatrix.rows(), 5);
    EXPECT_EQ(scratch.anderson.stepMatrix.cols(), 5);
    EXPECT_EQ(scratch.step.position.size(), 5);
    EXPECT_EQ(scratch.step.residual.size(), 5);
    EXPECT_EQ(scratch.step.zeroAcceleration.size(), 5);
    EXPECT_EQ(scratch.linearSolve.result.size(), 5);
    EXPECT_EQ(scratch.linearSolve.rhs.size(), 5);
    EXPECT_EQ(scratch.projection.residual.size(), 3);
    EXPECT_EQ(scratch.projection.jacobian.rows(), 3);
    EXPECT_EQ(scratch.projection.inverseMassJt.cols(), 3);
  }
}

// Phase B2 deliverable (rejection): a Point closure spanning two multibodies
// cannot be enforced by the per-multibody variational solver and must be
// rejected at step time (the architect-flagged correctness trap), not silently
// mishandled.
TEST(VariationalIntegration, LoopClosureCrossMultibodyRejectedUnderVariational)
{
  sx::World world;
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
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

  auto& registry = dart::simulation::detail::registryOf(world);
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
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
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

// PLAN-104 AVBD articulated bridge: a private fixed point-joint config on a
// multibody link now maps to the same variational rigid-closure solve path used
// by public loop closures. This keeps articulated endpoints out of the
// free-rigid 6-DOF snapshot writeback while still giving fixed link constraints
// a real solve path.
TEST(VariationalIntegration, AvbdFixedPointJointConfigSolvesLinkEndpoint)
{
  sx::World world;
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
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
    tipFromBase = tipFromBase * offset;
  }
  auto tip = robot.getLinks().back();

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink = sx::detail::toRegistryEntity(tip.getEntity());
  joint.childLink = entt::null;
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = tipFromBase.translation();
  config.targetRelativeOrientation
      = Eigen::Quaterniond(tipFromBase.linear()).conjugate();
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.setTimeStep(1e-3);
  world.enterSimulationMode();
  auto joints = robot.getJoints();
  ASSERT_EQ(joints.size(), 7u);

  double maxPositionResidual = 0.0;
  double maxRotationResidual = 0.0;
  double motion = 0.0;
  for (int k = 0; k < 2000; ++k) {
    world.step();
    const Eigen::Isometry3d tipTransform = tip.getWorldTransform();
    maxPositionResidual = std::max(
        maxPositionResidual,
        (tipTransform.translation() - tipFromBase.translation()).norm());
    maxRotationResidual = std::max(
        maxRotationResidual,
        (tipTransform.linear() - tipFromBase.linear()).norm());
    for (auto robotJoint : joints) {
      motion = std::max(motion, std::abs(robotJoint.getPosition()[0]));
    }
  }

  EXPECT_LT(maxPositionResidual, 1e-6);
  EXPECT_LT(maxRotationResidual, 1e-6);
  EXPECT_GT(motion, 1e-3);
}

// Breakable hard AVBD point-joint configs bridge into the articulated
// variational projection, then mark their source joint broken once the
// projection load exceeds the configured threshold so future steps skip it.
TEST(VariationalIntegration, AvbdBreakablePointJointConfigMarksLinkEndpoint)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  auto robot = world.addMultibody("pendulum");
  auto base = robot.addLink("base");

  sx::JointSpec spec;
  spec.name = "hinge";
  spec.type = sx::JointType::Revolute;
  spec.axis = Eigen::Vector3d::UnitY();
  spec.transformFromParent.translation() = Eigen::Vector3d(0.5, 0.0, 0.0);
  auto link = robot.addLink("link", base, spec);
  link.setMass(1.0);
  link.setInertia(Eigen::Vector3d(0.03, 0.03, 0.03).asDiagonal());

  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation() = spec.transformFromParent.translation();
  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink = sx::detail::toRegistryEntity(link.getEntity());
  joint.childLink = entt::null;
  joint.breakForce = 1e-18;
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = target.translation();
  config.targetRelativeOrientation
      = Eigen::Quaterniond(target.linear()).conjugate();
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.setTimeStep(1e-3);
  world.enterSimulationMode();

  link.applyForce(100.0 * Eigen::Vector3d::UnitZ());
  world.step();

  EXPECT_TRUE(registry.get<sx::comps::JointState>(jointEntity).broken);

  double maxPositionResidual = 0.0;
  for (int k = 0; k < 100; ++k) {
    link.applyForce(100.0 * Eigen::Vector3d::UnitZ());
    world.step();
    maxPositionResidual = std::max(
        maxPositionResidual,
        (link.getWorldTransform().translation() - target.translation()).norm());
  }

  EXPECT_GT(maxPositionResidual, 1e-3);
}

// PLAN-104 AVBD articulated fracture bridge: private fixed point-joint configs
// should survive binary save/load while broken, then re-enter the variational
// projection path after a broken-state reset without relying on facade
// extraction.
TEST(
    VariationalIntegration,
    AvbdBreakablePointJointConfigResetReengagesFixedRows)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  auto robot = world.addMultibody("resettable_private_breakable_pendulum");
  auto base = robot.addLink("base");

  sx::JointSpec spec;
  spec.name = "hinge";
  spec.type = sx::JointType::Revolute;
  spec.axis = Eigen::Vector3d::UnitY();
  spec.transformFromParent.translation() = Eigen::Vector3d(0.5, 0.0, 0.0);
  auto link = robot.addLink("link", base, spec);
  link.setMass(1.0);
  link.setInertia(Eigen::Vector3d(0.03, 0.03, 0.03).asDiagonal());

  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation() = spec.transformFromParent.translation();
  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  registry.emplace<sx::comps::Name>(
      jointEntity, "serialized_resettable_private_fixed");
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.name = "serialized_resettable_private_fixed";
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink = sx::detail::toRegistryEntity(link.getEntity());
  joint.childLink = entt::null;
  joint.breakForce = 1e-18;
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = target.translation();
  config.targetRelativeOrientation
      = Eigen::Quaterniond(target.linear()).conjugate();
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.setTimeStep(1e-3);
  world.enterSimulationMode();

  link.applyForce(100.0 * Eigen::Vector3d::UnitZ());
  world.step();

  auto& liveJoint = registry.get<sx::comps::JointState>(jointEntity);
  ASSERT_TRUE(liveJoint.broken);

  double maxBrokenPositionResidual = 0.0;
  for (int k = 0; k < 100; ++k) {
    link.applyForce(100.0 * Eigen::Vector3d::UnitZ());
    world.step();
    maxBrokenPositionResidual = std::max(
        maxBrokenPositionResidual,
        (link.getWorldTransform().translation() - target.translation()).norm());
  }
  ASSERT_GT(maxBrokenPositionResidual, 1e-3);

  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot
      = restored.getMultibody("resettable_private_breakable_pendulum");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredLink = restoredRobot->getLink("link");
  ASSERT_TRUE(restoredLink.has_value());

  auto& restoredRegistry = dart::simulation::detail::registryOf(restored);
  entt::entity restoredJointEntity = entt::null;
  auto jointView = restoredRegistry.view<sx::comps::JointModel>();
  for (const entt::entity entity : jointView) {
    if (jointView.get<sx::comps::JointModel>(entity).name
        == "serialized_resettable_private_fixed") {
      restoredJointEntity = entity;
      break;
    }
  }
  ASSERT_TRUE(restoredJointEntity != entt::null);
  auto& restoredJointModel
      = restoredRegistry.get<sx::comps::JointModel>(restoredJointEntity);
  auto& restoredJointState
      = restoredRegistry.get<sx::comps::JointState>(restoredJointEntity);
  ASSERT_TRUE(restoredJointState.broken);
  EXPECT_EQ(restoredJointModel.type, sx::comps::JointType::Fixed);
  EXPECT_EQ(
      restoredJointModel.parentLink,
      sx::detail::toRegistryEntity(restoredLink->getEntity()));
  EXPECT_TRUE(restoredJointModel.childLink == entt::null);
  ASSERT_TRUE(restoredRegistry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(
      restoredJointEntity));
  const auto& restoredConfig
      = restoredRegistry.get<dvbd::AvbdRigidWorldPointJointConfig>(
          restoredJointEntity);
  EXPECT_LT(
      (restoredConfig.localAnchorA - Eigen::Vector3d::Zero()).norm(), 1e-12);
  EXPECT_LT((restoredConfig.localAnchorB - target.translation()).norm(), 1e-12);
  EXPECT_TRUE(std::isinf(restoredConfig.startStiffness));
  EXPECT_TRUE(std::isinf(restoredConfig.maxStiffness));

  restoredJointModel.breakForce = std::numeric_limits<double>::infinity();
  restoredJointState.broken = false;

  restored.step();

  const Eigen::Isometry3d resetTransform = restoredLink->getWorldTransform();
  EXPECT_FALSE(restoredJointState.broken);
  EXPECT_LT((resetTransform.translation() - target.translation()).norm(), 1e-6);
  EXPECT_LT((resetTransform.linear() - target.linear()).norm(), 1e-6);
}

// PLAN-104 AVBD articulated fracture bridge: direct private fixed point-joint
// configs also need binary save/load and reset coverage when a floating
// multibody link is the parent endpoint of a world-link joint. This covers the
// parent-side anchor and target-orientation convention for non-topology links.
TEST(
    VariationalIntegration,
    AvbdBreakableParentPointJointConfigResetReengagesFixedRows)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto robot = world.addMultibody("serialized_private_parent_fixed_body");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  Eigen::VectorXd pose(6);
  pose << 0.2, -0.1, 0.15, 0.0, 0.0, 0.25;
  body.getParentJoint().setPosition(pose);

  const Eigen::Vector3d bodyAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d worldAnchor = body.getWorldTransform() * bodyAnchor;
  const Eigen::Matrix3d capturedRotation = body.getWorldTransform().linear();

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  registry.emplace<sx::comps::Name>(
      jointEntity, "serialized_resettable_private_parent_fixed");
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.name = "serialized_resettable_private_parent_fixed";
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink = sx::detail::toRegistryEntity(body.getEntity());
  joint.childLink = entt::null;
  joint.breakForce = 1e-18;
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = bodyAnchor;
  config.localAnchorB = worldAnchor;
  config.targetRelativeOrientation
      = Eigen::Quaterniond(capturedRotation).conjugate();
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  const auto anchorResidual = [&]() {
    return (body.getWorldTransform() * bodyAnchor - worldAnchor).norm();
  };
  const auto rotationError = [&]() {
    return (body.getWorldTransform().linear() - capturedRotation).norm();
  };
  const auto applyOffCenterForce = [&]() {
    body.applyForce(
        5.0 * Eigen::Vector3d::UnitY(),
        bodyAnchor + 0.4 * Eigen::Vector3d::UnitX());
  };

  world.enterSimulationMode();

  applyOffCenterForce();
  world.step();

  auto& liveJoint = registry.get<sx::comps::JointState>(jointEntity);
  ASSERT_TRUE(liveJoint.broken);
  EXPECT_LT(anchorResidual(), 1e-6);
  EXPECT_LT(rotationError(), 1e-6);

  double maxBrokenAnchorResidual = 0.0;
  double maxBrokenRotationError = 0.0;
  for (int k = 0; k < 20; ++k) {
    applyOffCenterForce();
    world.step();
    maxBrokenAnchorResidual
        = std::max(maxBrokenAnchorResidual, anchorResidual());
    maxBrokenRotationError = std::max(maxBrokenRotationError, rotationError());
  }
  ASSERT_GT(maxBrokenAnchorResidual, 1e-4);
  ASSERT_GT(maxBrokenRotationError, 1e-4);

  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot
      = restored.getMultibody("serialized_private_parent_fixed_body");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredBody = restoredRobot->getLink("body");
  ASSERT_TRUE(restoredBody.has_value());

  auto& restoredRegistry = dart::simulation::detail::registryOf(restored);
  entt::entity restoredJointEntity = entt::null;
  auto jointView = restoredRegistry.view<sx::comps::JointModel>();
  for (const entt::entity entity : jointView) {
    if (jointView.get<sx::comps::JointModel>(entity).name
        == "serialized_resettable_private_parent_fixed") {
      restoredJointEntity = entity;
      break;
    }
  }
  ASSERT_TRUE(restoredJointEntity != entt::null);
  auto& restoredJointModel
      = restoredRegistry.get<sx::comps::JointModel>(restoredJointEntity);
  auto& restoredJointState
      = restoredRegistry.get<sx::comps::JointState>(restoredJointEntity);
  ASSERT_TRUE(restoredJointState.broken);
  EXPECT_EQ(restoredJointModel.type, sx::comps::JointType::Fixed);
  EXPECT_EQ(
      restoredJointModel.parentLink,
      sx::detail::toRegistryEntity(restoredBody->getEntity()));
  EXPECT_TRUE(restoredJointModel.childLink == entt::null);
  ASSERT_TRUE(restoredRegistry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(
      restoredJointEntity));
  const auto& restoredConfig
      = restoredRegistry.get<dvbd::AvbdRigidWorldPointJointConfig>(
          restoredJointEntity);
  EXPECT_LT((restoredConfig.localAnchorA - bodyAnchor).norm(), 1e-12);
  EXPECT_LT((restoredConfig.localAnchorB - worldAnchor).norm(), 1e-12);
  EXPECT_LT(
      (restoredConfig.targetRelativeOrientation.toRotationMatrix()
       - capturedRotation.transpose())
          .norm(),
      1e-12);
  EXPECT_TRUE(std::isinf(restoredConfig.startStiffness));
  EXPECT_TRUE(std::isinf(restoredConfig.maxStiffness));

  const auto restoredAnchorResidual = [&]() {
    return (restoredBody->getWorldTransform() * bodyAnchor - worldAnchor)
        .norm();
  };
  const auto restoredRotationError = [&]() {
    return (restoredBody->getWorldTransform().linear() - capturedRotation)
        .norm();
  };

  restoredBody->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredJointModel.breakForce = std::numeric_limits<double>::infinity();
  restoredJointState.broken = false;

  restored.step();

  ASSERT_TRUE(restoredRegistry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(
      restoredJointEntity));
  const auto& resetConfig
      = restoredRegistry.get<dvbd::AvbdRigidWorldPointJointConfig>(
          restoredJointEntity);
  EXPECT_EQ(resetConfig.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(resetConfig.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_FALSE(restoredJointState.broken);
  EXPECT_LT(restoredAnchorResidual(), 1e-6);
  EXPECT_LT(restoredRotationError(), 1e-6);
}

// PLAN-104 AVBD articulated fracture bridge: direct private fixed point-joint
// configs also need binary save/load and reset coverage when the multibody link
// is the child endpoint of a world-link joint. This exercises the opposite
// endpoint polarity and the child-side target orientation convention.
TEST(
    VariationalIntegration,
    AvbdBreakableChildPointJointConfigResetReengagesFixedRows)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto robot = world.addMultibody("floater");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  Eigen::VectorXd pose(6);
  pose << 0.2, -0.1, 0.15, 0.0, 0.0, 0.25;
  body.getParentJoint().setPosition(pose);

  const Eigen::Vector3d bodyAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d worldAnchor = body.getWorldTransform() * bodyAnchor;
  const Eigen::Matrix3d capturedRotation = body.getWorldTransform().linear();

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  registry.emplace<sx::comps::Name>(
      jointEntity, "serialized_resettable_private_child_fixed");
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.name = "serialized_resettable_private_child_fixed";
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink = entt::null;
  joint.childLink = sx::detail::toRegistryEntity(body.getEntity());
  joint.breakForce = 1e-18;
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = worldAnchor;
  config.localAnchorB = bodyAnchor;
  config.targetRelativeOrientation = Eigen::Quaterniond(capturedRotation);
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  const auto anchorResidual = [&]() {
    return (worldAnchor - body.getWorldTransform() * bodyAnchor).norm();
  };
  const auto rotationError = [&]() {
    return (body.getWorldTransform().linear() - capturedRotation).norm();
  };
  const auto applyOffCenterForce = [&]() {
    body.applyForce(
        5.0 * Eigen::Vector3d::UnitY(),
        bodyAnchor + 0.4 * Eigen::Vector3d::UnitX());
  };

  world.enterSimulationMode();

  applyOffCenterForce();
  world.step();

  auto& liveJoint = registry.get<sx::comps::JointState>(jointEntity);
  ASSERT_TRUE(liveJoint.broken);
  EXPECT_LT(anchorResidual(), 1e-6);
  EXPECT_LT(rotationError(), 1e-6);

  double maxBrokenAnchorResidual = 0.0;
  double maxBrokenRotationError = 0.0;
  for (int k = 0; k < 20; ++k) {
    applyOffCenterForce();
    world.step();
    maxBrokenAnchorResidual
        = std::max(maxBrokenAnchorResidual, anchorResidual());
    maxBrokenRotationError = std::max(maxBrokenRotationError, rotationError());
  }
  ASSERT_GT(maxBrokenAnchorResidual, 1e-4);
  ASSERT_GT(maxBrokenRotationError, 1e-4);

  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot = restored.getMultibody("floater");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredBody = restoredRobot->getLink("body");
  ASSERT_TRUE(restoredBody.has_value());

  auto& restoredRegistry = dart::simulation::detail::registryOf(restored);
  entt::entity restoredJointEntity = entt::null;
  auto jointView = restoredRegistry.view<sx::comps::JointModel>();
  for (const entt::entity entity : jointView) {
    if (jointView.get<sx::comps::JointModel>(entity).name
        == "serialized_resettable_private_child_fixed") {
      restoredJointEntity = entity;
      break;
    }
  }
  ASSERT_TRUE(restoredJointEntity != entt::null);
  auto& restoredJointModel
      = restoredRegistry.get<sx::comps::JointModel>(restoredJointEntity);
  auto& restoredJointState
      = restoredRegistry.get<sx::comps::JointState>(restoredJointEntity);
  ASSERT_TRUE(restoredJointState.broken);
  EXPECT_EQ(restoredJointModel.type, sx::comps::JointType::Fixed);
  EXPECT_TRUE(restoredJointModel.parentLink == entt::null);
  EXPECT_EQ(
      restoredJointModel.childLink,
      sx::detail::toRegistryEntity(restoredBody->getEntity()));
  ASSERT_TRUE(restoredRegistry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(
      restoredJointEntity));
  const auto& restoredConfig
      = restoredRegistry.get<dvbd::AvbdRigidWorldPointJointConfig>(
          restoredJointEntity);
  EXPECT_LT((restoredConfig.localAnchorA - worldAnchor).norm(), 1e-12);
  EXPECT_LT((restoredConfig.localAnchorB - bodyAnchor).norm(), 1e-12);
  EXPECT_LT(
      (restoredConfig.targetRelativeOrientation.toRotationMatrix()
       - capturedRotation)
          .norm(),
      1e-12);
  EXPECT_TRUE(std::isinf(restoredConfig.startStiffness));
  EXPECT_TRUE(std::isinf(restoredConfig.maxStiffness));

  const auto restoredAnchorResidual = [&]() {
    return (worldAnchor - restoredBody->getWorldTransform() * bodyAnchor)
        .norm();
  };
  const auto restoredRotationError = [&]() {
    return (restoredBody->getWorldTransform().linear() - capturedRotation)
        .norm();
  };

  restoredBody->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredJointModel.breakForce = std::numeric_limits<double>::infinity();
  restoredJointState.broken = false;

  restored.step();

  ASSERT_TRUE(restoredRegistry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(
      restoredJointEntity));
  const auto& resetConfig
      = restoredRegistry.get<dvbd::AvbdRigidWorldPointJointConfig>(
          restoredJointEntity);
  EXPECT_EQ(resetConfig.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(resetConfig.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_FALSE(restoredJointState.broken);
  EXPECT_LT(restoredAnchorResidual(), 1e-6);
  EXPECT_LT(restoredRotationError(), 1e-6);
}

// PLAN-104 AVBD articulated fracture bridge: direct private spherical
// point-joint configs should keep their linear-only row mask and broken state
// across binary save/load, then re-engage only the anchor rows after reset.
TEST(
    VariationalIntegration,
    AvbdBreakableSphericalPointJointConfigSurvivesSaveLoadAndReset)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto robot = world.addMultibody("serialized_private_spherical_socket_body");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  Eigen::VectorXd pose(6);
  pose << 0.2, -0.1, 0.15, 0.0, 0.0, 0.25;
  body.getParentJoint().setPosition(pose);

  const Eigen::Vector3d bodyAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d worldAnchor = body.getWorldTransform() * bodyAnchor;
  const Eigen::Matrix3d capturedRotation = body.getWorldTransform().linear();

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  registry.emplace<sx::comps::Name>(
      jointEntity, "serialized_private_breakable_socket");
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  auto& jointState = registry.emplace<sx::comps::JointState>(jointEntity);
  registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.name = "serialized_private_breakable_socket";
  joint.type = sx::comps::JointType::Spherical;
  joint.parentLink = sx::detail::toRegistryEntity(body.getEntity());
  joint.childLink = entt::null;
  joint.breakForce = 1e-18;
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = bodyAnchor;
  config.localAnchorB = worldAnchor;
  config.linearAxisMask = dvbd::kAvbdRigidJointAllAxesMask;
  config.angularAxisMask = 0u;
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  const auto anchorResidual = [&]() {
    return (body.getWorldTransform() * bodyAnchor - worldAnchor).norm();
  };
  const auto applyOffCenterForce = [&]() {
    body.applyForce(
        5.0 * Eigen::Vector3d::UnitY(),
        bodyAnchor + 0.4 * Eigen::Vector3d::UnitX());
  };

  world.enterSimulationMode();

  applyOffCenterForce();
  world.step();

  ASSERT_TRUE(jointState.broken);
  EXPECT_LT(anchorResidual(), 1e-6);
  const Eigen::Isometry3d savedBrokenTransform = body.getWorldTransform();
  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot
      = restored.getMultibody("serialized_private_spherical_socket_body");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredBody = restoredRobot->getLink("body");
  ASSERT_TRUE(restoredBody.has_value());

  auto& restoredRegistry = dart::simulation::detail::registryOf(restored);
  entt::entity restoredJointEntity = entt::null;
  auto jointView = restoredRegistry.view<sx::comps::JointModel>();
  for (const entt::entity entity : jointView) {
    if (jointView.get<sx::comps::JointModel>(entity).name
        == "serialized_private_breakable_socket") {
      restoredJointEntity = entity;
      break;
    }
  }
  ASSERT_TRUE(restoredJointEntity != entt::null);
  auto& restoredJointModel
      = restoredRegistry.get<sx::comps::JointModel>(restoredJointEntity);
  auto& restoredJointState
      = restoredRegistry.get<sx::comps::JointState>(restoredJointEntity);
  ASSERT_TRUE(restoredJointState.broken);
  EXPECT_EQ(restoredJointModel.type, sx::comps::JointType::Spherical);
  EXPECT_EQ(
      restoredJointModel.parentLink,
      sx::detail::toRegistryEntity(restoredBody->getEntity()));
  EXPECT_TRUE(restoredJointModel.childLink == entt::null);
  ASSERT_TRUE(restoredRegistry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(
      restoredJointEntity));
  const auto& restoredConfig
      = restoredRegistry.get<dvbd::AvbdRigidWorldPointJointConfig>(
          restoredJointEntity);
  EXPECT_LT((restoredConfig.localAnchorA - bodyAnchor).norm(), 1e-12);
  EXPECT_LT((restoredConfig.localAnchorB - worldAnchor).norm(), 1e-12);
  EXPECT_EQ(restoredConfig.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(restoredConfig.angularAxisMask, 0u);
  EXPECT_TRUE(std::isinf(restoredConfig.startStiffness));
  EXPECT_TRUE(std::isinf(restoredConfig.maxStiffness));
  EXPECT_LT(
      (restoredBody->getWorldTransform().matrix()
       - savedBrokenTransform.matrix())
          .norm(),
      1e-12);

  const auto restoredAnchorResidual = [&]() {
    return (restoredBody->getWorldTransform() * bodyAnchor - worldAnchor)
        .norm();
  };
  const auto restoredRotationChange = [&]() {
    return (restoredBody->getWorldTransform().linear() - capturedRotation)
        .norm();
  };
  const auto applyRestoredOffCenterForce = [&]() {
    restoredBody->applyForce(
        5.0 * Eigen::Vector3d::UnitY(),
        bodyAnchor + 0.4 * Eigen::Vector3d::UnitX());
  };

  double maxBrokenAnchorResidual = 0.0;
  double maxBrokenRotationChange = 0.0;
  for (int k = 0; k < 20; ++k) {
    applyRestoredOffCenterForce();
    restored.step();
    maxBrokenAnchorResidual
        = std::max(maxBrokenAnchorResidual, restoredAnchorResidual());
    maxBrokenRotationChange
        = std::max(maxBrokenRotationChange, restoredRotationChange());
  }
  ASSERT_GT(maxBrokenAnchorResidual, 1e-4);
  ASSERT_GT(maxBrokenRotationChange, 1e-4);

  restoredBody->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredJointModel.breakForce = std::numeric_limits<double>::infinity();
  restoredJointState.broken = false;

  restored.step();

  ASSERT_TRUE(restoredRegistry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(
      restoredJointEntity));
  const auto& resetConfig
      = restoredRegistry.get<dvbd::AvbdRigidWorldPointJointConfig>(
          restoredJointEntity);
  EXPECT_EQ(resetConfig.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(resetConfig.angularAxisMask, 0u);
  EXPECT_FALSE(restoredJointState.broken);
  EXPECT_LT(restoredAnchorResidual(), 1e-6);
  EXPECT_GT(restoredRotationChange(), 1e-4);
}

// PLAN-104 AVBD articulated fracture bridge: direct private spherical
// point-joint configs also need persistent break/reset coverage when the
// multibody link is the child endpoint of a world-link joint. The linear-only
// rows should keep the anchor pinned while orientation remains free.
TEST(
    VariationalIntegration,
    AvbdBreakableSphericalChildPointJointConfigSurvivesSaveLoadAndReset)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto robot
      = world.addMultibody("serialized_private_child_spherical_socket_body");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  Eigen::VectorXd pose(6);
  pose << 0.2, -0.1, 0.15, 0.0, 0.0, 0.25;
  body.getParentJoint().setPosition(pose);

  const Eigen::Vector3d bodyAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d worldAnchor = body.getWorldTransform() * bodyAnchor;
  const Eigen::Matrix3d capturedRotation = body.getWorldTransform().linear();

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  registry.emplace<sx::comps::Name>(
      jointEntity, "serialized_private_child_breakable_socket");
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  auto& jointState = registry.emplace<sx::comps::JointState>(jointEntity);
  registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.name = "serialized_private_child_breakable_socket";
  joint.type = sx::comps::JointType::Spherical;
  joint.parentLink = entt::null;
  joint.childLink = sx::detail::toRegistryEntity(body.getEntity());
  joint.breakForce = 1e-18;
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = worldAnchor;
  config.localAnchorB = bodyAnchor;
  config.linearAxisMask = dvbd::kAvbdRigidJointAllAxesMask;
  config.angularAxisMask = 0u;
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  const auto anchorResidual = [&]() {
    return (worldAnchor - body.getWorldTransform() * bodyAnchor).norm();
  };
  const auto applyOffCenterForce = [&]() {
    body.applyForce(
        5.0 * Eigen::Vector3d::UnitY(),
        bodyAnchor + 0.4 * Eigen::Vector3d::UnitX());
  };

  world.enterSimulationMode();

  applyOffCenterForce();
  world.step();

  ASSERT_TRUE(jointState.broken);
  EXPECT_LT(anchorResidual(), 1e-6);
  const Eigen::Isometry3d savedBrokenTransform = body.getWorldTransform();
  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot
      = restored.getMultibody("serialized_private_child_spherical_socket_body");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredBody = restoredRobot->getLink("body");
  ASSERT_TRUE(restoredBody.has_value());

  auto& restoredRegistry = dart::simulation::detail::registryOf(restored);
  entt::entity restoredJointEntity = entt::null;
  auto jointView = restoredRegistry.view<sx::comps::JointModel>();
  for (const entt::entity entity : jointView) {
    if (jointView.get<sx::comps::JointModel>(entity).name
        == "serialized_private_child_breakable_socket") {
      restoredJointEntity = entity;
      break;
    }
  }
  ASSERT_TRUE(restoredJointEntity != entt::null);
  auto& restoredJointModel
      = restoredRegistry.get<sx::comps::JointModel>(restoredJointEntity);
  auto& restoredJointState
      = restoredRegistry.get<sx::comps::JointState>(restoredJointEntity);
  ASSERT_TRUE(restoredJointState.broken);
  EXPECT_EQ(restoredJointModel.type, sx::comps::JointType::Spherical);
  EXPECT_TRUE(restoredJointModel.parentLink == entt::null);
  EXPECT_EQ(
      restoredJointModel.childLink,
      sx::detail::toRegistryEntity(restoredBody->getEntity()));
  ASSERT_TRUE(restoredRegistry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(
      restoredJointEntity));
  const auto& restoredConfig
      = restoredRegistry.get<dvbd::AvbdRigidWorldPointJointConfig>(
          restoredJointEntity);
  EXPECT_LT((restoredConfig.localAnchorA - worldAnchor).norm(), 1e-12);
  EXPECT_LT((restoredConfig.localAnchorB - bodyAnchor).norm(), 1e-12);
  EXPECT_EQ(restoredConfig.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(restoredConfig.angularAxisMask, 0u);
  EXPECT_TRUE(std::isinf(restoredConfig.startStiffness));
  EXPECT_TRUE(std::isinf(restoredConfig.maxStiffness));
  EXPECT_LT(
      (restoredBody->getWorldTransform().matrix()
       - savedBrokenTransform.matrix())
          .norm(),
      1e-12);

  const auto restoredAnchorResidual = [&]() {
    return (worldAnchor - restoredBody->getWorldTransform() * bodyAnchor)
        .norm();
  };
  const auto restoredRotationChange = [&]() {
    return (restoredBody->getWorldTransform().linear() - capturedRotation)
        .norm();
  };
  const auto applyRestoredOffCenterForce = [&]() {
    restoredBody->applyForce(
        5.0 * Eigen::Vector3d::UnitY(),
        bodyAnchor + 0.4 * Eigen::Vector3d::UnitX());
  };

  double maxBrokenAnchorResidual = 0.0;
  double maxBrokenRotationChange = 0.0;
  for (int k = 0; k < 20; ++k) {
    applyRestoredOffCenterForce();
    restored.step();
    maxBrokenAnchorResidual
        = std::max(maxBrokenAnchorResidual, restoredAnchorResidual());
    maxBrokenRotationChange
        = std::max(maxBrokenRotationChange, restoredRotationChange());
  }
  ASSERT_GT(maxBrokenAnchorResidual, 1e-4);
  ASSERT_GT(maxBrokenRotationChange, 1e-4);

  restoredBody->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredJointModel.breakForce = std::numeric_limits<double>::infinity();
  restoredJointState.broken = false;

  restored.step();

  ASSERT_TRUE(restoredRegistry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(
      restoredJointEntity));
  const auto& resetConfig
      = restoredRegistry.get<dvbd::AvbdRigidWorldPointJointConfig>(
          restoredJointEntity);
  EXPECT_EQ(resetConfig.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(resetConfig.angularAxisMask, 0u);
  EXPECT_FALSE(restoredJointState.broken);
  EXPECT_LT(restoredAnchorResidual(), 1e-6);
  EXPECT_GT(restoredRotationChange(), 1e-4);
}

// Finite-stiffness private AVBD fixed point-joint configs bridge into the
// articulated variational path as compliant forces. They resist endpoint error
// without the hard projection used by infinite-stiffness configs.
TEST(VariationalIntegration, AvbdCompliantPointJointConfigPullsLinkEndpoint)
{
  struct Result
  {
    double maxTranslation = 0.0;
    double maxYaw = 0.0;
  };

  const auto rollout = [](bool compliant) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setMultibodyOptions(
        {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
    world.setTimeStep(0.002);

    auto robot = world.addMultibody("floater");
    auto base = robot.addLink("base");
    sx::JointSpec spec;
    spec.name = "float";
    spec.type = sx::JointType::Floating;
    auto body = robot.addLink("body", base, spec);
    body.setMass(2.0);
    body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

    if (compliant) {
      auto& registry = dart::simulation::detail::registryOf(world);
      const entt::entity jointEntity = registry.create();
      auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
      registry.emplace<sx::comps::JointState>(jointEntity);
      registry.emplace<sx::comps::JointActuation>(jointEntity);
      joint.type = sx::comps::JointType::Fixed;
      joint.parentLink = sx::detail::toRegistryEntity(body.getEntity());
      joint.childLink = entt::null;

      auto& config
          = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
      config.localAnchorA = Eigen::Vector3d::Zero();
      config.localAnchorB = Eigen::Vector3d::Zero();
      config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
      config.startStiffness = 1000.0;
      config.maxStiffness = 1000.0;
    }

    world.enterSimulationMode();

    Result result;
    for (int k = 0; k < 80; ++k) {
      body.applyForce(10.0 * Eigen::Vector3d::UnitX());
      body.applyForce(
          2.0 * Eigen::Vector3d::UnitY(), 0.5 * Eigen::Vector3d::UnitX());
      world.step();
      const Eigen::Isometry3d transform = body.getWorldTransform();
      result.maxTranslation
          = std::max(result.maxTranslation, transform.translation().x());
      result.maxYaw = std::max(
          result.maxYaw,
          std::abs(
              std::atan2(transform.linear()(1, 0), transform.linear()(0, 0))));
    }
    return result;
  };

  const Result free = rollout(/*compliant=*/false);
  const Result compliant = rollout(/*compliant=*/true);

  EXPECT_GT(free.maxTranslation, 2e-2);
  EXPECT_GT(compliant.maxTranslation, 1e-4);
  EXPECT_LT(compliant.maxTranslation, 0.5 * free.maxTranslation);
  EXPECT_GT(free.maxYaw, 1e-2);
  EXPECT_LT(compliant.maxYaw, 0.5 * free.maxYaw);
}

TEST(
    VariationalIntegration,
    AvbdCompliantPointJointConfigRemovalRestoresFreeMotion)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.002);

  auto robot = world.addMultibody("floater");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "float";
  spec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, spec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink = sx::detail::toRegistryEntity(body.getEntity());
  joint.childLink = entt::null;

  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.startStiffness = 1000.0;
  config.maxStiffness = 1000.0;

  world.enterSimulationMode();
  world.step();

  registry.remove<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  ASSERT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  body.getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));

  double maxTranslation = 0.0;
  double maxYaw = 0.0;
  for (int k = 0; k < 80; ++k) {
    body.applyForce(10.0 * Eigen::Vector3d::UnitX());
    body.applyForce(
        2.0 * Eigen::Vector3d::UnitY(), 0.5 * Eigen::Vector3d::UnitX());
    world.step();
    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxTranslation = std::max(maxTranslation, transform.translation().x());
    maxYaw = std::max(
        maxYaw,
        std::abs(
            std::atan2(transform.linear()(1, 0), transform.linear()(0, 0))));
  }

  EXPECT_GT(maxTranslation, 2e-2);
  EXPECT_GT(maxYaw, 1e-2);
}

TEST(
    VariationalIntegration,
    AvbdCompliantPointJointConfigRampsStiffnessAcrossSteps)
{
  struct Result
  {
    double maxTranslation = 0.0;
    double finalTranslation = 0.0;
    double maxYaw = 0.0;
  };

  const auto rollout = [](double startStiffness, double maxStiffness) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setMultibodyOptions(
        {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
    world.setTimeStep(0.002);

    auto robot = world.addMultibody("floater");
    auto base = robot.addLink("base");
    sx::JointSpec spec;
    spec.name = "float";
    spec.type = sx::JointType::Floating;
    auto body = robot.addLink("body", base, spec);
    body.setMass(2.0);
    body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

    auto& registry = dart::simulation::detail::registryOf(world);
    const entt::entity jointEntity = registry.create();
    auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
    registry.emplace<sx::comps::JointState>(jointEntity);
    registry.emplace<sx::comps::JointActuation>(jointEntity);
    joint.type = sx::comps::JointType::Fixed;
    joint.parentLink = sx::detail::toRegistryEntity(body.getEntity());
    joint.childLink = entt::null;

    auto& config
        = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
    config.localAnchorA = Eigen::Vector3d::Zero();
    config.localAnchorB = Eigen::Vector3d::Zero();
    config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
    config.startStiffness = startStiffness;
    config.maxStiffness = maxStiffness;

    world.enterSimulationMode();

    Result result;
    for (int k = 0; k < 80; ++k) {
      body.applyForce(10.0 * Eigen::Vector3d::UnitX());
      body.applyForce(
          2.0 * Eigen::Vector3d::UnitY(), 0.5 * Eigen::Vector3d::UnitX());
      world.step();
      const Eigen::Isometry3d transform = body.getWorldTransform();
      result.finalTranslation = transform.translation().x();
      result.maxTranslation
          = std::max(result.maxTranslation, result.finalTranslation);
      result.maxYaw = std::max(
          result.maxYaw,
          std::abs(
              std::atan2(transform.linear()(1, 0), transform.linear()(0, 0))));
    }
    return result;
  };

  const Result capped = rollout(/*startStiffness=*/30.0, /*maxStiffness=*/30.0);
  const Result ramped
      = rollout(/*startStiffness=*/30.0, /*maxStiffness=*/1000.0);

  EXPECT_GT(capped.finalTranslation, 1e-2);
  EXPECT_LT(ramped.finalTranslation, 0.7 * capped.finalTranslation);
  EXPECT_LT(ramped.maxTranslation, 0.7 * capped.maxTranslation);
  EXPECT_LT(ramped.maxYaw, 0.7 * capped.maxYaw);
}

TEST(
    VariationalIntegration,
    AvbdCompliantPublicPointJointConfigUsesPerRoleMaterialStiffness)
{
  struct Result
  {
    double maxTranslation = 0.0;
    double maxYaw = 0.0;
  };

  const auto rollout = [](double linearMaterialStiffness,
                          double angularMaterialStiffness,
                          bool driveLinear,
                          bool driveAngular) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setMultibodyOptions(
        {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
    world.setTimeStep(0.002);

    auto robot = world.addMultibody("floater");
    auto base = robot.addLink("base");
    sx::JointSpec spec;
    spec.name = "float";
    spec.type = sx::JointType::Floating;
    auto body = robot.addLink("body", base, spec);
    body.setMass(2.0);
    body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

    sx::Joint joint = world.addArticulatedFixedJoint("hold", body);
    joint.setAvbdStartStiffness(10.0);
    joint.setAvbdLinearStiffness(linearMaterialStiffness);
    joint.setAvbdAngularStiffness(angularMaterialStiffness);

    auto& registry = dart::simulation::detail::registryOf(world);
    const entt::entity jointEntity
        = sx::detail::toRegistryEntity(joint.getEntity());

    world.enterSimulationMode();

    const auto* config
        = registry.try_get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
    EXPECT_NE(config, nullptr);
    if (config != nullptr) {
      EXPECT_DOUBLE_EQ(
          config->linearMaterialStiffness, linearMaterialStiffness);
      EXPECT_DOUBLE_EQ(
          config->angularMaterialStiffness, angularMaterialStiffness);
      EXPECT_TRUE(std::isinf(config->maxStiffness));
    }

    Result result;
    for (int k = 0; k < 80; ++k) {
      if (driveLinear) {
        body.applyForce(10.0 * Eigen::Vector3d::UnitX());
      }
      if (driveAngular) {
        body.applyForce(
            2.0 * Eigen::Vector3d::UnitY(), 0.5 * Eigen::Vector3d::UnitX());
        body.applyForce(
            -2.0 * Eigen::Vector3d::UnitY(), -0.5 * Eigen::Vector3d::UnitX());
      }

      world.step();
      const Eigen::Isometry3d transform = body.getWorldTransform();
      result.maxTranslation = std::max(
          result.maxTranslation, std::abs(transform.translation().x()));
      result.maxYaw = std::max(
          result.maxYaw,
          std::abs(
              std::atan2(transform.linear()(1, 0), transform.linear()(0, 0))));
    }
    return result;
  };

  const Result softLinear = rollout(
      /*linearMaterialStiffness=*/10.0,
      /*angularMaterialStiffness=*/1000.0,
      /*driveLinear=*/true,
      /*driveAngular=*/false);
  const Result stiffLinear = rollout(
      /*linearMaterialStiffness=*/1000.0,
      /*angularMaterialStiffness=*/1000.0,
      /*driveLinear=*/true,
      /*driveAngular=*/false);
  EXPECT_GT(softLinear.maxTranslation, 1e-2);
  EXPECT_LT(stiffLinear.maxTranslation, 0.7 * softLinear.maxTranslation);

  const Result softAngular = rollout(
      /*linearMaterialStiffness=*/1000.0,
      /*angularMaterialStiffness=*/10.0,
      /*driveLinear=*/false,
      /*driveAngular=*/true);
  const Result stiffAngular = rollout(
      /*linearMaterialStiffness=*/1000.0,
      /*angularMaterialStiffness=*/1000.0,
      /*driveLinear=*/false,
      /*driveAngular=*/true);
  EXPECT_GT(softAngular.maxYaw, 1e-3);
  EXPECT_LT(stiffAngular.maxYaw, 0.7 * softAngular.maxYaw);
}

// Topology joints are the multibody tree itself, not external AVBD point-joint
// rows. Even if a private config is attached accidentally, the articulated
// bridge must not duplicate the tree joint as a rigid closure.
TEST(VariationalIntegration, AvbdPointJointConfigSkipsTopologyJoint)
{
  sx::World world;
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  auto robot = world.addMultibody("pendulum");
  auto base = robot.addLink("base");

  sx::JointSpec spec;
  spec.name = "hinge";
  spec.type = sx::JointType::Revolute;
  spec.axis = Eigen::Vector3d::UnitY();
  spec.transformFromParent.translation() = Eigen::Vector3d(0.4, 0.0, 0.0);
  auto link = robot.addLink("link", base, spec);
  link.setMass(1.0);
  link.setInertia(Eigen::Vector3d(0.03, 0.03, 0.03).asDiagonal());

  auto hinge = robot.getJoints().front();
  auto& registry = dart::simulation::detail::registryOf(world);
  auto& config = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(
      sx::detail::toRegistryEntity(hinge.getEntity()));
  config.localAnchorA = Eigen::Vector3d(0.4, 0.0, 0.0);
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();

  world.setTimeStep(1e-3);
  world.enterSimulationMode();
  for (int k = 0; k < 500; ++k) {
    world.step();
  }

  EXPECT_GT(std::abs(hinge.getPosition()[0]), 1e-3);
}

// Phase B2 (Distance family through the public API): a Distance loop closure
// added via addLoopClosure -- holding the tip of a 2-link arm at a fixed
// distance from a world anchor -- is solved by the variational integrator on
// the World::step() path while the arm swings under gravity.
TEST(VariationalIntegration, LoopClosureDistanceSolvedThroughWorldStep)
{
  sx::World world;
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
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

// PLAN-084 serialization gate: binary save/load round-trips the trajectory and
// does NOT re-bootstrap the two-step discrete-mechanics history. A reference
// pendulum runs 50 steps (establishing a non-trivial history), is saved, then
// run 50 more; a fresh world loaded from the save and run 50 steps must reach
// the bit-identical state. If MultibodyVariationalState were lost on load, the
// integrator would re-bootstrap from the current velocity and diverge.
TEST(VariationalIntegration, StateSerializationRoundTripsTrajectory)
{
  const auto buildPendulum = [](sx::World& world) {
    world.setMultibodyOptions(
        {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
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
  loaded.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  ASSERT_TRUE(loaded.getMultibody("pendulum").has_value());
  for (int k = 0; k < 50; ++k) {
    loaded.step();
  }
  const double loadedFinal
      = loaded.getMultibody("pendulum")->getJoints()[0].getPosition()[0];

  EXPECT_EQ(loadedFinal, referenceFinal); // bit-identical, no re-bootstrap
}

// PLAN-084 A2 long-chain convergence gate: a 64-link revolute chain converges
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

  auto& registry = dart::simulation::detail::registryOf(world);
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

    auto& registry = dart::simulation::detail::registryOf(world);
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
  ASSERT_EQ(
      world.getMultibodyOptions().integrationFamily,
      sx::MultibodyIntegrationFamily::SemiImplicit);
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
      dart::simulation::detail::registryOf(world)
          .view<sxc::MultibodyVariationalState>()
          .size(),
      0u);
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
    auto& registry = dart::simulation::detail::registryOf(world);
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

    auto& registry = dart::simulation::detail::registryOf(world);
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

    auto& registry = dart::simulation::detail::registryOf(world);
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

struct FloatingLinkPair
{
  sx::Link parent;
  sx::Link child;
};

FloatingLinkPair addFloatingLinkPair(sx::World& world)
{
  auto robot = world.addMultibody("floating_pair");
  auto base = robot.addLink("base");

  sx::JointSpec parentSpec;
  parentSpec.name = "parent_float";
  parentSpec.type = sx::JointType::Floating;
  auto parent = robot.addLink("parent", base, parentSpec);
  parent.setMass(2.0);
  parent.setInertia(Eigen::Vector3d(0.2, 0.25, 0.3).asDiagonal());

  sx::JointSpec childSpec;
  childSpec.name = "child_float";
  childSpec.type = sx::JointType::Floating;
  auto child = robot.addLink("child", base, childSpec);
  child.setMass(1.5);
  child.setInertia(Eigen::Vector3d(0.15, 0.2, 0.25).asDiagonal());

  return {parent, child};
}

} // namespace

// PLAN-104 AVBD articulated bridge: simulation-entry current-pose extraction
// now covers non-topology private point-joint entities whose endpoint is a
// multibody link. The generated hard fixed config captures the link's design
// pose and then projects it through the variational articulated path.
TEST(
    VariationalIntegration,
    AvbdFixedPointJointCurrentPoseExtractorWiresFloatingLinkEndpoint)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto body = addFloatingBody(world, /*mass=*/2.0);
  Eigen::VectorXd pose(6);
  pose << 0.35, -0.2, 0.15, 0.0, 0.0, 0.25;
  body.getParentJoint().setPosition(pose);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink = entt::null;
  joint.childLink = sx::detail::toRegistryEntity(body.getEntity());

  EXPECT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  world.enterSimulationMode();
  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  const Eigen::Isometry3d captured = body.getWorldTransform();
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_NEAR(
      (config.localAnchorA - captured.translation()).norm(), 0.0, 1e-12);
  EXPECT_NEAR(config.localAnchorB.norm(), 0.0, 1e-12);
  EXPECT_TRUE(std::isinf(config.startStiffness));
  EXPECT_TRUE(std::isinf(config.maxStiffness));

  double maxTranslationError = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < 60; ++k) {
    body.applyForce(
        3.0 * Eigen::Vector3d::UnitY(), 0.4 * Eigen::Vector3d::UnitX());
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxTranslationError = std::max(
        maxTranslationError,
        (transform.translation() - captured.translation()).norm());
    maxRotationError = std::max(
        maxRotationError, (transform.linear() - captured.linear()).norm());
  }

  EXPECT_LT(maxTranslationError, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
}

// PLAN-104 AVBD articulated bridge: fixed current-pose extraction also covers
// the parent-link world endpoint polarity, not only the child-link case above.
TEST(
    VariationalIntegration,
    AvbdFixedPointJointCurrentPoseExtractorWiresParentWorldLinkEndpoint)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto body = addFloatingBody(world, /*mass=*/2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());
  Eigen::VectorXd pose(6);
  pose << 0.35, -0.2, 0.15, 0.0, 0.0, 0.25;
  body.getParentJoint().setPosition(pose);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink = sx::detail::toRegistryEntity(body.getEntity());
  joint.childLink = entt::null;

  EXPECT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  world.enterSimulationMode();
  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  const Eigen::Isometry3d captured = body.getWorldTransform();
  const Eigen::Vector3d worldAnchor = captured.translation();
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_LT(config.localAnchorA.norm(), 1e-12);
  EXPECT_LT((config.localAnchorB - worldAnchor).norm(), 1e-12);
  EXPECT_LT(
      (config.targetRelativeOrientation.toRotationMatrix()
       - captured.linear().transpose())
          .norm(),
      1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_TRUE(std::isinf(config.startStiffness));
  EXPECT_TRUE(std::isinf(config.maxStiffness));

  const auto anchorResidual = [&]() {
    return (body.getWorldTransform() * Eigen::Vector3d::Zero() - worldAnchor)
        .norm();
  };

  double maxAnchorResidual = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < 60; ++k) {
    body.applyForce(
        3.0 * Eigen::Vector3d::UnitY(), 0.4 * Eigen::Vector3d::UnitX());
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxAnchorResidual = std::max(maxAnchorResidual, anchorResidual());
    maxRotationError = std::max(
        maxRotationError, (transform.linear() - captured.linear()).norm());
  }

  EXPECT_LT(maxAnchorResidual, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
}

// PLAN-104 AVBD articulated fracture bridge: private spherical point-joint
// current-pose extraction should also cover world-link endpoints in both
// endpoint polarities. The generated rows pin only the anchor pair, leaving
// relative orientation free.
TEST(
    VariationalIntegration,
    AvbdSphericalPointJointCurrentPoseExtractorWiresWorldLinkEndpointPolarities)
{
  const auto runCase = [](bool linkIsParentEndpoint) {
    SCOPED_TRACE(linkIsParentEndpoint ? "link-parent" : "link-child");

    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setMultibodyOptions(
        {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
    world.setTimeStep(0.005);

    auto body = addFloatingBody(world, /*mass=*/2.0);
    body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());
    Eigen::VectorXd pose(6);
    pose << 0.3, -0.2, 0.1, 0.0, 0.0, 0.25;
    body.getParentJoint().setPosition(pose);

    auto& registry = dart::simulation::detail::registryOf(world);
    const entt::entity jointEntity = registry.create();
    auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
    registry.emplace<sx::comps::JointState>(jointEntity);
    registry.emplace<sx::comps::JointActuation>(jointEntity);
    joint.type = sx::comps::JointType::Spherical;
    if (linkIsParentEndpoint) {
      joint.parentLink = sx::detail::toRegistryEntity(body.getEntity());
      joint.childLink = entt::null;
    } else {
      joint.parentLink = entt::null;
      joint.childLink = sx::detail::toRegistryEntity(body.getEntity());
    }

    EXPECT_FALSE(
        registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
    world.enterSimulationMode();
    ASSERT_TRUE(
        registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

    const Eigen::Isometry3d captured = body.getWorldTransform();
    const Eigen::Vector3d worldAnchor = captured.translation();
    const auto& config
        = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
    if (linkIsParentEndpoint) {
      EXPECT_LT(config.localAnchorA.norm(), 1e-12);
      EXPECT_LT((config.localAnchorB - worldAnchor).norm(), 1e-12);
      EXPECT_LT(
          (config.targetRelativeOrientation.toRotationMatrix()
           - captured.linear().transpose())
              .norm(),
          1e-12);
    } else {
      EXPECT_LT((config.localAnchorA - worldAnchor).norm(), 1e-12);
      EXPECT_LT(config.localAnchorB.norm(), 1e-12);
      EXPECT_LT(
          (config.targetRelativeOrientation.toRotationMatrix()
           - captured.linear())
              .norm(),
          1e-12);
    }
    EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
    EXPECT_EQ(config.angularAxisMask, 0u);
    EXPECT_TRUE(std::isinf(config.startStiffness));
    EXPECT_TRUE(std::isinf(config.maxStiffness));

    const auto anchorResidual = [&]() {
      const Eigen::Vector3d bodyAnchorWorld
          = body.getWorldTransform() * Eigen::Vector3d::Zero();
      return (bodyAnchorWorld - worldAnchor).norm();
    };

    double maxAnchorResidual = 0.0;
    double maxRotationError = 0.0;
    for (int k = 0; k < 60; ++k) {
      body.applyForce(
          3.0 * Eigen::Vector3d::UnitY(), 0.4 * Eigen::Vector3d::UnitX());
      world.step();

      const Eigen::Isometry3d transform = body.getWorldTransform();
      maxAnchorResidual = std::max(maxAnchorResidual, anchorResidual());
      maxRotationError = std::max(
          maxRotationError, (transform.linear() - captured.linear()).norm());
    }

    EXPECT_LT(maxAnchorResidual, 1e-6);
    EXPECT_GT(maxRotationError, 1e-4);
  };

  runCase(/*linkIsParentEndpoint=*/false);
  runCase(/*linkIsParentEndpoint=*/true);
}

// PLAN-104 AVBD articulated bridge: current-pose extraction also derives the
// masked angular rows for private revolute point-joint entities whose endpoint
// is a multibody link. The extracted velocity motor then drives only the free
// non-cardinal hinge axis.
TEST(
    VariationalIntegration,
    AvbdRevolutePointJointCurrentPoseExtractorDrivesFloatingLinkEndpoint)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto body = addFloatingBody(world, /*mass=*/2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Revolute;
  joint.parentLink = entt::null;
  joint.childLink = sx::detail::toRegistryEntity(body.getEntity());
  const Eigen::Vector3d hingeAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  joint.axis = hingeAxis;
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.4;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);

  EXPECT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  world.enterSimulationMode();
  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  world.updateKinematics();

  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_NEAR(config.angularAxes.col(2).dot(hingeAxis), 1.0, 1e-12);
  EXPECT_TRUE(std::isinf(config.startStiffness));
  EXPECT_TRUE(std::isinf(config.maxStiffness));

  constexpr int steps = 20;
  double maxPositionResidual = 0.0;
  double maxHingeAxisTilt = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxPositionResidual
        = std::max(maxPositionResidual, transform.translation().norm());
    maxHingeAxisTilt = std::max(
        maxHingeAxisTilt, (transform.linear() * hingeAxis - hingeAxis).norm());
  }

  const Eigen::Matrix3d expectedRotation
      = Eigen::AngleAxisd(targetSpeed * dt * steps, hingeAxis)
            .toRotationMatrix();
  EXPECT_LT(maxPositionResidual, 1e-6);
  EXPECT_LT(maxHingeAxisTilt, 1e-6);
  EXPECT_LT(
      (body.getWorldTransform().linear() - expectedRotation).norm(), 1e-6);
}

// PLAN-104 AVBD articulated bridge: the same current-pose extractor derives
// masks for private prismatic point-joint entities on multibody endpoints, so a
// hard velocity motor with a non-cardinal slider axis can be generated without
// hand-authored AVBD config rows.
TEST(
    VariationalIntegration,
    AvbdPrismaticPointJointCurrentPoseExtractorDrivesFloatingLinkEndpoint)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto body = addFloatingBody(world, /*mass=*/2.0);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Prismatic;
  joint.parentLink = entt::null;
  joint.childLink = sx::detail::toRegistryEntity(body.getEntity());
  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(1.0, 2.0, 0.5).normalized();
  joint.axis = sliderAxis;
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.3;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);

  EXPECT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  world.enterSimulationMode();
  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  world.updateKinematics();

  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(config.linearAxes.col(2).dot(sliderAxis), 1.0, 1e-12);
  EXPECT_TRUE(std::isinf(config.startStiffness));
  EXPECT_TRUE(std::isinf(config.maxStiffness));

  constexpr int steps = 20;
  double maxOrthogonalDrift = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    const Eigen::Vector3d position = transform.translation();
    maxOrthogonalDrift = std::max(
        maxOrthogonalDrift,
        (position - position.dot(sliderAxis) * sliderAxis).norm());
    maxRotationError = std::max(
        maxRotationError,
        (transform.linear() - Eigen::Matrix3d::Identity()).norm());
  }

  const Eigen::Vector3d position = body.getWorldTransform().translation();
  EXPECT_LT(maxOrthogonalDrift, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
  EXPECT_NEAR(position.dot(sliderAxis), targetSpeed * dt * steps, 1e-6);
}

// PLAN-104 AVBD articulated bridge: the public World articulated fixed-joint
// facade now creates a non-topology link-link point joint that the
// simulation-entry AVBD current-pose extractor converts into hard articulated
// variational rows.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedFixedJointFacadeWiresFloatingLinkEndpoint)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto robot = world.addMultibody("public_fixed_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  Eigen::VectorXd pose(6);
  pose << 0.25, -0.15, 0.2, 0.0, 0.0, 0.3;
  body.getParentJoint().setPosition(pose);

  sx::Joint joint = world.addArticulatedFixedJoint("base_hold", base, body);
  EXPECT_EQ(joint.getType(), sx::JointType::Fixed);
  EXPECT_EQ(joint.getParentLink().getName(), "base");
  EXPECT_EQ(joint.getChildLink().getName(), "body");
  EXPECT_TRUE(world.hasArticulatedJoint("base_hold"));
  EXPECT_EQ(world.getArticulatedJointCount(), 1u);
  ASSERT_TRUE(world.getArticulatedJoint("base_hold").has_value());
  EXPECT_EQ(world.getArticulatedJoints().size(), 1u);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());
  EXPECT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  world.enterSimulationMode();
  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  const Eigen::Isometry3d captured = body.getWorldTransform();
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_TRUE(std::isinf(config.startStiffness));
  EXPECT_TRUE(std::isinf(config.maxStiffness));

  double maxTranslationError = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < 60; ++k) {
    body.applyForce(
        3.0 * Eigen::Vector3d::UnitY(), 0.4 * Eigen::Vector3d::UnitX());
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxTranslationError = std::max(
        maxTranslationError,
        (transform.translation() - captured.translation()).norm());
    maxRotationError = std::max(
        maxRotationError, (transform.linear() - captured.linear()).norm());
  }

  EXPECT_LT(maxTranslationError, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
}

// PLAN-104 AVBD articulated bridge: public fixed point joints can also bind
// explicit off-origin link-local anchors, so fixed rows are not limited to link
// origins while holding a same-multibody relative pose.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedFixedJointFacadeDrivesOffsetAnchorPair)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  const Eigen::Vector3d parentAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d childAnchor(-0.1, 0.1, 0.0);
  sx::Joint joint = world.addArticulatedFixedJoint(
      "offset_hold", pair.parent, pair.child, parentAnchor, childAnchor);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_LT((config.localAnchorA - parentAnchor).norm(), 1e-12);
  EXPECT_LT((config.localAnchorB - childAnchor).norm(), 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);

  double maxAnchorResidual = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < 60; ++k) {
    pair.parent.applyForce(
        -3.0 * Eigen::Vector3d::UnitY(), 0.3 * Eigen::Vector3d::UnitX());
    pair.child.applyForce(
        3.0 * Eigen::Vector3d::UnitY(), -0.3 * Eigen::Vector3d::UnitX());
    world.step();

    const Eigen::Isometry3d parentTransform = pair.parent.getWorldTransform();
    const Eigen::Isometry3d childTransform = pair.child.getWorldTransform();
    maxAnchorResidual = std::max(
        maxAnchorResidual,
        (parentTransform * parentAnchor - childTransform * childAnchor).norm());
    maxRotationError = std::max(
        maxRotationError,
        (parentTransform.linear().transpose() * childTransform.linear()
         - Eigen::Matrix3d::Identity())
            .norm());
  }

  EXPECT_LT(maxAnchorResidual, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
}

// PLAN-104 AVBD articulated bridge: same-multibody public fixed point joints
// must preserve explicit link-local anchors and break-force metadata through
// design-mode binary save/load. The restored facade then rebuilds all-axis
// fixed AVBD rows at simulation entry.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedFixedJointFacadeSurvivesSaveLoad)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  const Eigen::Vector3d parentAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d childAnchor(-0.1, 0.1, 0.0);
  sx::Joint joint = world.addArticulatedFixedJoint(
      "serialized_link_hold",
      pair.parent,
      pair.child,
      parentAnchor,
      childAnchor);
  joint.setBreakForce(17.0);

  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot = restored.getMultibody("floating_pair");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredParent = restoredRobot->getLink("parent");
  ASSERT_TRUE(restoredParent.has_value());
  auto restoredChild = restoredRobot->getLink("child");
  ASSERT_TRUE(restoredChild.has_value());
  auto restoredJoint = restored.getArticulatedJoint("serialized_link_hold");
  ASSERT_TRUE(restoredJoint.has_value());
  EXPECT_EQ(restoredJoint->getType(), sx::JointType::Fixed);
  EXPECT_EQ(restoredJoint->getDOFCount(), 0u);
  EXPECT_EQ(restoredJoint->getParentLink().getName(), "parent");
  EXPECT_EQ(restoredJoint->getChildLink().getName(), "child");
  EXPECT_DOUBLE_EQ(restoredJoint->getBreakForce(), 17.0);
  EXPECT_FALSE(restoredJoint->isBroken());

  auto& registry = dart::simulation::detail::registryOf(restored);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(restoredJoint->getEntity());
  EXPECT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  restored.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_LT((config.localAnchorA - parentAnchor).norm(), 1e-12);
  EXPECT_LT((config.localAnchorB - childAnchor).norm(), 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_TRUE(std::isinf(config.startStiffness));
  EXPECT_TRUE(std::isinf(config.maxStiffness));

  const Eigen::Matrix3d capturedRelativeRotation
      = restoredParent->getWorldTransform().linear().transpose()
        * restoredChild->getWorldTransform().linear();
  double maxAnchorResidual = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < 40; ++k) {
    restoredParent->applyForce(
        -0.3 * Eigen::Vector3d::UnitY(),
        parentAnchor + 0.2 * Eigen::Vector3d::UnitX());
    restoredChild->applyForce(
        0.3 * Eigen::Vector3d::UnitY(),
        childAnchor - 0.2 * Eigen::Vector3d::UnitX());
    restored.step();

    const Eigen::Isometry3d parentTransform
        = restoredParent->getWorldTransform();
    const Eigen::Isometry3d childTransform = restoredChild->getWorldTransform();
    maxAnchorResidual = std::max(
        maxAnchorResidual,
        (parentTransform * parentAnchor - childTransform * childAnchor).norm());
    maxRotationError = std::max(
        maxRotationError,
        (parentTransform.linear().transpose() * childTransform.linear()
         - capturedRelativeRotation)
            .norm());
  }

  EXPECT_LT(maxAnchorResidual, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
  EXPECT_FALSE(restoredJoint->isBroken());
}

// PLAN-104 AVBD articulated bridge: finite AVBD stiffness set through public
// articulated facades must survive design-mode binary save/load and feed the
// private point-joint row configs rebuilt at simulation entry.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedStiffnessSurvivesSaveLoadIntoPrivateConfigs)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  FloatingLinkPair pair = addFloatingLinkPair(world);

  struct ExpectedJoint
  {
    std::string name;
    sx::JointType type;
    std::size_t dofs;
    double startStiffness;
    double linearStiffness;
    double angularStiffness;
    std::uint8_t linearAxisMask;
    std::uint8_t angularAxisMask;
    Eigen::Vector3d freeAxis;
    bool hasFreeLinearAxis = false;
    bool hasFreeAngularAxis = false;
  };

  const Eigen::Vector3d sliderAxis = Eigen::Vector3d::UnitX();
  const Eigen::Vector3d hingeAxis = Eigen::Vector3d::UnitY();
  const std::array<ExpectedJoint, 8> expected{{
      {
          "serialized_stiff_fixed",
          sx::JointType::Fixed,
          0u,
          3.0,
          234.0,
          567.0,
          dvbd::kAvbdRigidJointAllAxesMask,
          dvbd::kAvbdRigidJointAllAxesMask,
          Eigen::Vector3d::Zero(),
      },
      {
          "serialized_stiff_hinge",
          sx::JointType::Revolute,
          1u,
          4.0,
          345.0,
          678.0,
          dvbd::kAvbdRigidJointAllAxesMask,
          dvbd::avbdRigidJointAllButAxisMask(2u),
          hingeAxis,
          false,
          true,
      },
      {
          "serialized_stiff_slider",
          sx::JointType::Prismatic,
          1u,
          5.0,
          456.0,
          789.0,
          dvbd::avbdRigidJointAllButAxisMask(2u),
          dvbd::kAvbdRigidJointAllAxesMask,
          sliderAxis,
          true,
      },
      {
          "serialized_stiff_socket",
          sx::JointType::Spherical,
          3u,
          6.0,
          567.0,
          890.0,
          dvbd::kAvbdRigidJointAllAxesMask,
          0u,
          Eigen::Vector3d::Zero(),
      },
      {
          "serialized_stiff_world_fixed",
          sx::JointType::Fixed,
          0u,
          7.0,
          678.0,
          901.0,
          dvbd::kAvbdRigidJointAllAxesMask,
          dvbd::kAvbdRigidJointAllAxesMask,
          Eigen::Vector3d::Zero(),
      },
      {
          "serialized_stiff_world_hinge",
          sx::JointType::Revolute,
          1u,
          8.0,
          789.0,
          1012.0,
          dvbd::kAvbdRigidJointAllAxesMask,
          dvbd::avbdRigidJointAllButAxisMask(2u),
          hingeAxis,
          false,
          true,
      },
      {
          "serialized_stiff_world_slider",
          sx::JointType::Prismatic,
          1u,
          9.0,
          890.0,
          1123.0,
          dvbd::avbdRigidJointAllButAxisMask(2u),
          dvbd::kAvbdRigidJointAllAxesMask,
          sliderAxis,
          true,
      },
      {
          "serialized_stiff_world_socket",
          sx::JointType::Spherical,
          3u,
          10.0,
          901.0,
          1234.0,
          dvbd::kAvbdRigidJointAllAxesMask,
          0u,
          Eigen::Vector3d::Zero(),
      },
  }};

  std::array<sx::Joint, expected.size()> joints{
      world.addArticulatedFixedJoint(expected[0].name, pair.parent, pair.child),
      world.addArticulatedRevoluteJoint(
          expected[1].name, pair.parent, pair.child, hingeAxis),
      world.addArticulatedPrismaticJoint(
          expected[2].name, pair.parent, pair.child, sliderAxis),
      world.addArticulatedSphericalJoint(
          expected[3].name, pair.parent, pair.child),
      world.addArticulatedFixedJoint(expected[4].name, pair.parent),
      world.addArticulatedRevoluteJoint(
          expected[5].name, pair.parent, hingeAxis),
      world.addArticulatedPrismaticJoint(
          expected[6].name, pair.child, sliderAxis),
      world.addArticulatedSphericalJoint(expected[7].name, pair.child),
  };

  for (std::size_t i = 0; i < expected.size(); ++i) {
    joints[i].setAvbdStartStiffness(expected[i].startStiffness);
    joints[i].setAvbdLinearStiffness(expected[i].linearStiffness);
    joints[i].setAvbdAngularStiffness(expected[i].angularStiffness);
  }

  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto& registry = dart::simulation::detail::registryOf(restored);
  for (const ExpectedJoint& joint : expected) {
    SCOPED_TRACE(joint.name);
    auto restoredJoint = restored.getArticulatedJoint(joint.name);
    ASSERT_TRUE(restoredJoint.has_value());
    EXPECT_EQ(restoredJoint->getType(), joint.type);
    EXPECT_EQ(restoredJoint->getDOFCount(), joint.dofs);
    EXPECT_FALSE(registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(
        sx::detail::toRegistryEntity(restoredJoint->getEntity())));
  }

  restored.enterSimulationMode();

  for (const ExpectedJoint& joint : expected) {
    SCOPED_TRACE(joint.name);
    auto restoredJoint = restored.getArticulatedJoint(joint.name);
    ASSERT_TRUE(restoredJoint.has_value());
    const entt::entity jointEntity
        = sx::detail::toRegistryEntity(restoredJoint->getEntity());
    ASSERT_TRUE(
        registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
    const auto& config
        = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
    EXPECT_EQ(config.linearAxisMask, joint.linearAxisMask);
    EXPECT_EQ(config.angularAxisMask, joint.angularAxisMask);
    EXPECT_DOUBLE_EQ(config.startStiffness, joint.startStiffness);
    EXPECT_DOUBLE_EQ(config.linearMaterialStiffness, joint.linearStiffness);
    EXPECT_DOUBLE_EQ(config.angularMaterialStiffness, joint.angularStiffness);
    EXPECT_TRUE(std::isinf(config.maxStiffness));
    if (joint.hasFreeLinearAxis) {
      EXPECT_NEAR(config.linearAxes.col(2).dot(joint.freeAxis), 1.0, 1e-12);
    }
    if (joint.hasFreeAngularAxis) {
      EXPECT_NEAR(config.angularAxes.col(2).dot(joint.freeAxis), 1.0, 1e-12);
    }
  }
}

// PLAN-104 AVBD articulated bridge: public spherical point joints create
// linear-only AVBD rows for multibody endpoints, preserving the captured anchor
// while leaving relative orientation free.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedSphericalJointFacadeLeavesOrientationFree)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto robot = world.addMultibody("public_spherical_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  sx::Joint joint
      = world.addArticulatedSphericalJoint("base_socket", base, body);
  EXPECT_EQ(joint.getType(), sx::JointType::Spherical);
  EXPECT_EQ(joint.getDOFCount(), 3u);
  EXPECT_TRUE(world.hasArticulatedJoint("base_socket"));
  EXPECT_EQ(world.getArticulatedJointCount(), 1u);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());
  EXPECT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  world.enterSimulationMode();
  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, 0u);
  EXPECT_TRUE(std::isinf(config.startStiffness));
  EXPECT_TRUE(std::isinf(config.maxStiffness));

  const Eigen::Vector3d capturedPosition
      = body.getWorldTransform().translation();
  double maxTranslationError = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < 60; ++k) {
    body.applyForce(
        3.0 * Eigen::Vector3d::UnitY(), 0.4 * Eigen::Vector3d::UnitX());
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxTranslationError = std::max(
        maxTranslationError,
        (transform.translation() - capturedPosition).norm());
    maxRotationError = std::max(
        maxRotationError,
        (transform.linear() - Eigen::Matrix3d::Identity()).norm());
  }

  EXPECT_LT(maxTranslationError, 1e-6);
  EXPECT_GT(maxRotationError, 1e-4);
}

// PLAN-104 AVBD articulated bridge: world-anchored public spherical point
// joints also accept explicit off-origin world/link anchors. The variational
// projection pins only that anchor pair, leaving relative orientation free.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedWorldSphericalJointFacadeDrivesOffsetAnchor)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto robot = world.addMultibody("public_world_spherical_offset_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
  pose.head<3>() = Eigen::Vector3d(0.3, -0.2, 0.1);
  body.getParentJoint().setPosition(pose);
  const Eigen::Vector3d childAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d worldAnchor = body.getWorldTransform() * childAnchor;

  sx::Joint joint = world.addArticulatedSphericalJoint(
      "world_offset_socket", body, worldAnchor, childAnchor);
  EXPECT_EQ(joint.getType(), sx::JointType::Spherical);
  EXPECT_EQ(joint.getDOFCount(), 3u);
  EXPECT_TRUE(world.hasArticulatedJoint("world_offset_socket"));

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();
  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_LT((config.localAnchorA - worldAnchor).norm(), 1e-12);
  EXPECT_LT((config.localAnchorB - childAnchor).norm(), 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, 0u);

  const Eigen::Matrix3d capturedRotation = body.getWorldTransform().linear();
  double maxAnchorResidual = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < 60; ++k) {
    body.applyForce(
        3.0 * Eigen::Vector3d::UnitY(),
        childAnchor + 0.4 * Eigen::Vector3d::UnitX());
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxAnchorResidual = std::max(
        maxAnchorResidual, (transform * childAnchor - worldAnchor).norm());
    maxRotationError = std::max(
        maxRotationError, (transform.linear() - capturedRotation).norm());
  }

  EXPECT_LT(maxAnchorResidual, 1e-6);
  EXPECT_GT(maxRotationError, 1e-4);
}

// PLAN-104 AVBD articulated bridge: saving a public world-link spherical point
// joint in design mode must preserve enough facade state for loadBinary() plus
// enterSimulationMode() to rebuild the private linear-only AVBD rows.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedWorldSphericalJointFacadeSurvivesSaveLoad)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto robot = world.addMultibody("serialized_world_spherical_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  Eigen::VectorXd pose(6);
  pose << 0.3, -0.2, 0.1, 0.0, 0.0, 0.25;
  body.getParentJoint().setPosition(pose);
  const Eigen::Vector3d childAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d worldAnchor = body.getWorldTransform() * childAnchor;

  sx::Joint joint = world.addArticulatedSphericalJoint(
      "serialized_socket", body, worldAnchor, childAnchor);
  joint.setBreakForce(7.5);

  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot
      = restored.getMultibody("serialized_world_spherical_facade");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredBody = restoredRobot->getLink("body");
  ASSERT_TRUE(restoredBody.has_value());
  auto restoredJoint = restored.getArticulatedJoint("serialized_socket");
  ASSERT_TRUE(restoredJoint.has_value());
  EXPECT_EQ(restored.getArticulatedJointCount(), 1u);
  EXPECT_EQ(restoredJoint->getType(), sx::JointType::Spherical);
  EXPECT_EQ(restoredJoint->getDOFCount(), 3u);
  EXPECT_EQ(restoredJoint->getChildLink().getName(), "body");
  EXPECT_THROW(
      { (void)restoredJoint->getParentLink(); }, sx::InvalidArgumentException);
  EXPECT_DOUBLE_EQ(restoredJoint->getBreakForce(), 7.5);
  EXPECT_FALSE(restoredJoint->isBroken());

  auto& registry = dart::simulation::detail::registryOf(restored);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(restoredJoint->getEntity());
  EXPECT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  restored.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_LT((config.localAnchorA - worldAnchor).norm(), 1e-12);
  EXPECT_LT((config.localAnchorB - childAnchor).norm(), 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, 0u);
  EXPECT_TRUE(std::isinf(config.startStiffness));
  EXPECT_TRUE(std::isinf(config.maxStiffness));

  const Eigen::Matrix3d capturedRotation
      = restoredBody->getWorldTransform().linear();
  double maxAnchorResidual = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < 60; ++k) {
    restoredBody->applyForce(
        3.0 * Eigen::Vector3d::UnitY(),
        childAnchor + 0.4 * Eigen::Vector3d::UnitX());
    restored.step();

    const Eigen::Isometry3d transform = restoredBody->getWorldTransform();
    maxAnchorResidual = std::max(
        maxAnchorResidual, (transform * childAnchor - worldAnchor).norm());
    maxRotationError = std::max(
        maxRotationError, (transform.linear() - capturedRotation).norm());
  }

  EXPECT_LT(maxAnchorResidual, 1e-6);
  EXPECT_GT(maxRotationError, 1e-4);
}

// PLAN-104 AVBD articulated bridge: same-multibody link-link spherical point
// joints must also preserve their explicit facade anchors through design-mode
// binary save/load, then rebuild linear-only AVBD rows at simulation entry.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedLinkSphericalJointFacadeSurvivesSaveLoad)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  const Eigen::Vector3d parentAnchor(0.15, 0.05, 0.0);
  const Eigen::Vector3d childAnchor(-0.15, 0.05, 0.0);
  sx::Joint joint = world.addArticulatedSphericalJoint(
      "serialized_link_socket",
      pair.parent,
      pair.child,
      parentAnchor,
      childAnchor);
  joint.setBreakForce(9.0);

  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot = restored.getMultibody("floating_pair");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredParent = restoredRobot->getLink("parent");
  ASSERT_TRUE(restoredParent.has_value());
  auto restoredChild = restoredRobot->getLink("child");
  ASSERT_TRUE(restoredChild.has_value());
  auto restoredJoint = restored.getArticulatedJoint("serialized_link_socket");
  ASSERT_TRUE(restoredJoint.has_value());
  EXPECT_EQ(restored.getArticulatedJointCount(), 1u);
  EXPECT_EQ(restoredJoint->getType(), sx::JointType::Spherical);
  EXPECT_EQ(restoredJoint->getDOFCount(), 3u);
  EXPECT_EQ(restoredJoint->getParentLink().getName(), "parent");
  EXPECT_EQ(restoredJoint->getChildLink().getName(), "child");
  EXPECT_DOUBLE_EQ(restoredJoint->getBreakForce(), 9.0);
  EXPECT_FALSE(restoredJoint->isBroken());

  auto& registry = dart::simulation::detail::registryOf(restored);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(restoredJoint->getEntity());
  EXPECT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  restored.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_LT((config.localAnchorA - parentAnchor).norm(), 1e-12);
  EXPECT_LT((config.localAnchorB - childAnchor).norm(), 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, 0u);
  EXPECT_TRUE(std::isinf(config.startStiffness));
  EXPECT_TRUE(std::isinf(config.maxStiffness));

  const Eigen::Matrix3d capturedRelativeRotation
      = restoredParent->getWorldTransform().linear().transpose()
        * restoredChild->getWorldTransform().linear();
  double maxAnchorResidual = 0.0;
  double maxRelativeRotationChange = 0.0;
  for (int k = 0; k < 60; ++k) {
    restoredParent->applyForce(
        -3.0 * Eigen::Vector3d::UnitY(),
        parentAnchor + 0.4 * Eigen::Vector3d::UnitX());
    restoredChild->applyForce(
        3.0 * Eigen::Vector3d::UnitY(),
        childAnchor - 0.4 * Eigen::Vector3d::UnitX());
    restored.step();

    const Eigen::Isometry3d parentTransform
        = restoredParent->getWorldTransform();
    const Eigen::Isometry3d childTransform = restoredChild->getWorldTransform();
    maxAnchorResidual = std::max(
        maxAnchorResidual,
        (parentTransform * parentAnchor - childTransform * childAnchor).norm());
    maxRelativeRotationChange = std::max(
        maxRelativeRotationChange,
        (parentTransform.linear().transpose() * childTransform.linear()
         - capturedRelativeRotation)
            .norm());
  }

  EXPECT_LT(maxAnchorResidual, 1e-6);
  EXPECT_GT(maxRelativeRotationChange, 1e-4);
}

// PLAN-104 AVBD articulated bridge: break-force bookkeeping also applies to
// public spherical point joints. Once the linear-only rows trip the threshold,
// later variational extraction skips the broken joint and the floating link can
// drift away from the captured anchor.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedSphericalBreakForceMarksLinearRowsAndSkipsLaterSteps)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto robot = world.addMultibody("public_spherical_breakable_skip");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  sx::Joint joint
      = world.addArticulatedSphericalJoint("breakable_socket", base, body);
  joint.setBreakForce(1e-18);

  world.enterSimulationMode();
  const Eigen::Vector3d capturedPosition
      = body.getWorldTransform().translation();

  body.applyForce(4.0 * Eigen::Vector3d::UnitY());
  world.step();
  ASSERT_TRUE(joint.isBroken());

  double maxBrokenTranslationError = 0.0;
  for (int k = 0; k < 20; ++k) {
    body.applyForce(4.0 * Eigen::Vector3d::UnitY());
    world.step();
    maxBrokenTranslationError = std::max(
        maxBrokenTranslationError,
        (body.getWorldTransform().translation() - capturedPosition).norm());
  }

  EXPECT_GT(maxBrokenTranslationError, 1e-4);
}

// PLAN-104 AVBD articulated bridge: resetting a broken world-anchored public
// spherical point joint re-enables its linear anchor rows without adding
// orientation rows. The reset projection restores the pinned anchor while
// preserving the spherical facade's free relative orientation.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedWorldSphericalBreakForceResetReengagesLinearRows)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto robot = world.addMultibody("public_world_spherical_breakable_reset");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  Eigen::VectorXd pose(6);
  pose << 0.3, -0.2, 0.1, 0.0, 0.0, 0.25;
  body.getParentJoint().setPosition(pose);
  const Eigen::Vector3d childAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d worldAnchor = body.getWorldTransform() * childAnchor;
  const Eigen::Matrix3d capturedRotation = body.getWorldTransform().linear();

  sx::Joint joint = world.addArticulatedSphericalJoint(
      "world_breakable_socket", body, worldAnchor, childAnchor);
  joint.setBreakForce(1e-18);

  world.enterSimulationMode();

  body.applyForce(
      4.0 * Eigen::Vector3d::UnitY(),
      childAnchor + 0.4 * Eigen::Vector3d::UnitX());
  world.step();
  ASSERT_TRUE(joint.isBroken());

  double maxBrokenAnchorResidual = 0.0;
  double maxBrokenRotationError = 0.0;
  for (int k = 0; k < 20; ++k) {
    body.applyForce(
        4.0 * Eigen::Vector3d::UnitY(),
        childAnchor + 0.4 * Eigen::Vector3d::UnitX());
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxBrokenAnchorResidual = std::max(
        maxBrokenAnchorResidual,
        (transform * childAnchor - worldAnchor).norm());
    maxBrokenRotationError = std::max(
        maxBrokenRotationError, (transform.linear() - capturedRotation).norm());
  }
  ASSERT_GT(maxBrokenAnchorResidual, 1e-4);
  ASSERT_GT(maxBrokenRotationError, 1e-4);

  body.getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  joint.setBreakForce(1.0e12);
  joint.resetBreakage();
  ASSERT_FALSE(joint.isBroken());

  world.step();

  const Eigen::Isometry3d resetTransform = body.getWorldTransform();
  EXPECT_FALSE(joint.isBroken());
  EXPECT_LT((resetTransform * childAnchor - worldAnchor).norm(), 1e-6);
  EXPECT_GT((resetTransform.linear() - capturedRotation).norm(), 1e-4);
}

// PLAN-104 AVBD articulated bridge: broken world-link public spherical joints
// must keep their broken state through simulation-mode binary save/load. The
// restored linear-only rows stay skipped until reset, then re-engage the loaded
// world anchor without adding orientation rows.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedWorldSphericalBreakageSurvivesSaveLoadAndReset)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto robot = world.addMultibody("public_world_spherical_save_load_breakage");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  Eigen::VectorXd pose(6);
  pose << 0.3, -0.2, 0.1, 0.0, 0.0, 0.25;
  body.getParentJoint().setPosition(pose);
  const Eigen::Vector3d childAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d worldAnchor = body.getWorldTransform() * childAnchor;
  const Eigen::Matrix3d capturedRotation = body.getWorldTransform().linear();

  sx::Joint joint = world.addArticulatedSphericalJoint(
      "serialized_world_broken_socket", body, worldAnchor, childAnchor);
  joint.setBreakForce(1e-18);

  world.enterSimulationMode();

  body.applyForce(
      4.0 * Eigen::Vector3d::UnitY(),
      childAnchor + 0.4 * Eigen::Vector3d::UnitX());
  world.step();
  ASSERT_TRUE(joint.isBroken());

  const Eigen::Isometry3d savedBrokenTransform = body.getWorldTransform();

  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot
      = restored.getMultibody("public_world_spherical_save_load_breakage");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredBody = restoredRobot->getLink("body");
  ASSERT_TRUE(restoredBody.has_value());
  auto restoredJoint
      = restored.getArticulatedJoint("serialized_world_broken_socket");
  ASSERT_TRUE(restoredJoint.has_value());
  ASSERT_TRUE(restoredJoint->isBroken());
  EXPECT_EQ(restoredJoint->getType(), sx::JointType::Spherical);
  EXPECT_EQ(restoredJoint->getDOFCount(), 3u);
  EXPECT_EQ(restoredJoint->getChildLink().getName(), "body");
  EXPECT_THROW(
      { (void)restoredJoint->getParentLink(); }, sx::InvalidArgumentException);
  EXPECT_DOUBLE_EQ(restoredJoint->getBreakForce(), 1e-18);
  EXPECT_LT(
      (restoredBody->getWorldTransform().translation()
       - savedBrokenTransform.translation())
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredBody->getWorldTransform().linear()
       - savedBrokenTransform.linear())
          .norm(),
      1e-12);

  const Eigen::Matrix3d restoredCapturedRotation
      = restoredBody->getWorldTransform().linear();

  double maxBrokenAnchorResidual = 0.0;
  double maxBrokenRotationError = 0.0;
  for (int k = 0; k < 20; ++k) {
    restoredBody->applyForce(
        4.0 * Eigen::Vector3d::UnitY(),
        childAnchor + 0.4 * Eigen::Vector3d::UnitX());
    restored.step();

    const Eigen::Isometry3d transform = restoredBody->getWorldTransform();
    maxBrokenAnchorResidual = std::max(
        maxBrokenAnchorResidual,
        (transform * childAnchor - worldAnchor).norm());
    maxBrokenRotationError = std::max(
        maxBrokenRotationError,
        (transform.linear() - restoredCapturedRotation).norm());
  }
  ASSERT_GT(maxBrokenAnchorResidual, 1e-4);
  ASSERT_GT(maxBrokenRotationError, 1e-4);

  restoredBody->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredJoint->setBreakForce(1.0e12);
  restoredJoint->resetBreakage();
  ASSERT_FALSE(restoredJoint->isBroken());

  restored.step();

  const Eigen::Isometry3d resetTransform = restoredBody->getWorldTransform();
  EXPECT_FALSE(restoredJoint->isBroken());
  EXPECT_LT((resetTransform * childAnchor - worldAnchor).norm(), 1e-6);
  EXPECT_GT((resetTransform.linear() - capturedRotation).norm(), 1e-4);
}

// PLAN-104 AVBD articulated bridge: same-multibody link-link spherical joints
// must also keep broken-state bookkeeping across simulation-mode binary
// save/load. The restored linear-only rows stay skipped until reset, then
// re-engage the loaded anchor pair while still leaving relative orientation
// free.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedLinkSphericalBreakageSurvivesSaveLoadAndReset)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  const Eigen::Vector3d parentAnchor(0.15, 0.05, 0.0);
  const Eigen::Vector3d childAnchor(-0.15, 0.05, 0.0);
  sx::Joint joint = world.addArticulatedSphericalJoint(
      "serialized_link_broken_socket",
      pair.parent,
      pair.child,
      parentAnchor,
      childAnchor);
  joint.setBreakForce(1e-18);

  world.enterSimulationMode();

  pair.parent.applyForce(
      -4.0 * Eigen::Vector3d::UnitY(),
      parentAnchor + 0.4 * Eigen::Vector3d::UnitX());
  pair.child.applyForce(
      4.0 * Eigen::Vector3d::UnitY(),
      childAnchor - 0.4 * Eigen::Vector3d::UnitX());
  world.step();
  ASSERT_TRUE(joint.isBroken());

  const Eigen::Isometry3d savedParentTransform
      = pair.parent.getWorldTransform();
  const Eigen::Isometry3d savedChildTransform = pair.child.getWorldTransform();
  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot = restored.getMultibody("floating_pair");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredParent = restoredRobot->getLink("parent");
  ASSERT_TRUE(restoredParent.has_value());
  auto restoredChild = restoredRobot->getLink("child");
  ASSERT_TRUE(restoredChild.has_value());
  auto restoredJoint
      = restored.getArticulatedJoint("serialized_link_broken_socket");
  ASSERT_TRUE(restoredJoint.has_value());
  ASSERT_TRUE(restoredJoint->isBroken());
  EXPECT_DOUBLE_EQ(restoredJoint->getBreakForce(), 1e-18);
  EXPECT_EQ(restoredJoint->getType(), sx::JointType::Spherical);
  EXPECT_EQ(restoredJoint->getDOFCount(), 3u);
  EXPECT_EQ(restoredJoint->getParentLink().getName(), "parent");
  EXPECT_EQ(restoredJoint->getChildLink().getName(), "child");
  EXPECT_LT(
      (restoredParent->getWorldTransform().translation()
       - savedParentTransform.translation())
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredParent->getWorldTransform().linear()
       - savedParentTransform.linear())
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredChild->getWorldTransform().translation()
       - savedChildTransform.translation())
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredChild->getWorldTransform().linear()
       - savedChildTransform.linear())
          .norm(),
      1e-12);

  const Eigen::Matrix3d restoredCapturedRelativeRotation
      = restoredParent->getWorldTransform().linear().transpose()
        * restoredChild->getWorldTransform().linear();
  double maxBrokenAnchorResidual = 0.0;
  double maxBrokenRelativeRotationError = 0.0;
  for (int k = 0; k < 20; ++k) {
    restoredParent->applyForce(
        -4.0 * Eigen::Vector3d::UnitY(),
        parentAnchor + 0.4 * Eigen::Vector3d::UnitX());
    restoredChild->applyForce(
        4.0 * Eigen::Vector3d::UnitY(),
        childAnchor - 0.4 * Eigen::Vector3d::UnitX());
    restored.step();

    const Eigen::Isometry3d parentTransform
        = restoredParent->getWorldTransform();
    const Eigen::Isometry3d childTransform = restoredChild->getWorldTransform();
    maxBrokenAnchorResidual = std::max(
        maxBrokenAnchorResidual,
        (parentTransform * parentAnchor - childTransform * childAnchor).norm());
    maxBrokenRelativeRotationError = std::max(
        maxBrokenRelativeRotationError,
        (parentTransform.linear().transpose() * childTransform.linear()
         - restoredCapturedRelativeRotation)
            .norm());
  }
  ASSERT_GT(maxBrokenAnchorResidual, 1e-4);
  ASSERT_GT(maxBrokenRelativeRotationError, 1e-4);

  restoredParent->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredChild->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredJoint->setBreakForce(1.0e12);
  restoredJoint->resetBreakage();
  ASSERT_FALSE(restoredJoint->isBroken());

  restored.step();

  const Eigen::Isometry3d resetParentTransform
      = restoredParent->getWorldTransform();
  const Eigen::Isometry3d resetChildTransform
      = restoredChild->getWorldTransform();
  EXPECT_FALSE(restoredJoint->isBroken());
  EXPECT_LT(
      (resetParentTransform * parentAnchor - resetChildTransform * childAnchor)
          .norm(),
      1e-6);
  EXPECT_GT(
      (resetParentTransform.linear().transpose() * resetChildTransform.linear()
       - restoredCapturedRelativeRotation)
          .norm(),
      1e-4);
}

// PLAN-104 AVBD articulated bridge: public articulated revolute joints now
// exercise the same free-axis angular motor row as private masked point-joint
// configs, while the facade-created hard rows keep the endpoint pinned and a
// non-cardinal hinge axis aligned.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedRevoluteJointFacadeDrivesFloatingLinkEndpoint)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto robot = world.addMultibody("public_revolute_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  const Eigen::Vector3d hingeAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  sx::Joint joint
      = world.addArticulatedRevoluteJoint("base_hinge", base, body, hingeAxis);
  ASSERT_EQ(joint.getType(), sx::JointType::Revolute);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.4;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_NEAR(config.angularAxes.col(2).dot(hingeAxis), 1.0, 1e-12);

  constexpr int steps = 20;
  double maxPositionResidual = 0.0;
  double maxHingeAxisTilt = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxPositionResidual
        = std::max(maxPositionResidual, transform.translation().norm());
    maxHingeAxisTilt = std::max(
        maxHingeAxisTilt, (transform.linear() * hingeAxis - hingeAxis).norm());
  }

  const Eigen::Matrix3d expectedRotation
      = Eigen::AngleAxisd(targetSpeed * dt * steps, hingeAxis)
            .toRotationMatrix();
  EXPECT_LT(maxPositionResidual, 1e-6);
  EXPECT_LT(maxHingeAxisTilt, 1e-6);
  EXPECT_LT(
      (body.getWorldTransform().linear() - expectedRotation).norm(), 1e-6);
}

// PLAN-104 AVBD articulated bridge: public articulated revolute facades also
// expose the finite effort bound on the private angular motor row. A tiny
// positive torque limit keeps the masked hard rows active but prevents the
// velocity actuator from behaving like an unbounded coordinate target.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedRevoluteJointFacadeRespectsTinyTorqueLimit)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto robot = world.addMultibody("public_revolute_tiny_limit_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  const Eigen::Vector3d hingeAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  const Eigen::Vector3d parentAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d childAnchor(-0.1, 0.1, 0.0);
  Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
  pose.head<3>() = parentAnchor - childAnchor;
  body.getParentJoint().setPosition(pose);
  sx::Joint joint = world.addArticulatedRevoluteJoint(
      "base_hinge", base, body, hingeAxis, parentAnchor, childAnchor);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.4;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1e-9), Eigen::VectorXd::Constant(1, 1e-9));

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_LT((config.localAnchorA - parentAnchor).norm(), 1e-12);
  EXPECT_LT((config.localAnchorB - childAnchor).norm(), 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_NEAR(config.angularAxes.col(2).dot(hingeAxis), 1.0, 1e-12);

  constexpr int steps = 20;
  double maxAnchorResidual = 0.0;
  double maxHingeAxisTilt = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxAnchorResidual = std::max(
        maxAnchorResidual, (transform * childAnchor - parentAnchor).norm());
    maxHingeAxisTilt = std::max(
        maxHingeAxisTilt, (transform.linear() * hingeAxis - hingeAxis).norm());
  }

  const Eigen::Matrix3d rotation = body.getWorldTransform().linear();
  const double hingeMotion = signedRotationAroundAxis(rotation, hingeAxis);
  EXPECT_LT(maxAnchorResidual, 1e-6);
  EXPECT_LT(maxHingeAxisTilt, 1e-3);
  EXPECT_LT(
      std::abs(hingeMotion), 0.01 * targetSpeed * world.getTimeStep() * steps);
}

// PLAN-104 AVBD articulated bridge: public articulated revolute velocity
// commands are read from the source joint every step, so changing the command
// updates the free-axis motor target without rebuilding the facade joint.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedRevoluteJointFacadeFollowsCommandUpdates)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto robot = world.addMultibody("public_revolute_command_update_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  sx::Joint joint = world.addArticulatedRevoluteJoint(
      "base_hinge", base, body, Eigen::Vector3d::UnitZ());
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.4;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));

  world.enterSimulationMode();

  double maxPositionResidual = 0.0;
  double maxHingeAxisTilt = 0.0;
  auto stepAndYaw = [&]() {
    world.step();
    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxPositionResidual
        = std::max(maxPositionResidual, transform.translation().norm());
    maxHingeAxisTilt = std::max(
        maxHingeAxisTilt,
        (transform.linear() * Eigen::Vector3d::UnitZ()
         - Eigen::Vector3d::UnitZ())
            .norm());
    const Eigen::Matrix3d rotation = transform.linear();
    return std::atan2(rotation(1, 0), rotation(0, 0));
  };

  constexpr int forwardSteps = 10;
  double yawAfterForward = 0.0;
  for (int k = 0; k < forwardSteps; ++k) {
    yawAfterForward = stepAndYaw();
  }

  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, -targetSpeed));

  constexpr int reverseSteps = 10;
  double yawAfterReverse = yawAfterForward;
  for (int k = 0; k < reverseSteps; ++k) {
    yawAfterReverse = stepAndYaw();
  }

  EXPECT_LT(maxPositionResidual, 1e-6);
  EXPECT_LT(maxHingeAxisTilt, 1e-6);
  EXPECT_NEAR(yawAfterForward, targetSpeed * dt * forwardSteps, 1e-6);
  EXPECT_NEAR(
      yawAfterReverse, targetSpeed * dt * (forwardSteps - reverseSteps), 1e-6);
}

// PLAN-104 AVBD articulated bridge: same-multibody public revolute point
// joints also work when both endpoints are movable floating links, not only
// when one endpoint is the fixed base or world frame.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedRevoluteJointFacadeDrivesMovableLinkPair)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  const Eigen::Vector3d hingeAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  sx::Joint joint = world.addArticulatedRevoluteJoint(
      "movable_hinge", pair.parent, pair.child, hingeAxis);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.4;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_NEAR(config.angularAxes.col(2).dot(hingeAxis), 1.0, 1e-12);

  constexpr int steps = 20;
  double maxAnchorResidual = 0.0;
  double maxHingeAxisTilt = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d parentTransform = pair.parent.getWorldTransform();
    const Eigen::Isometry3d childTransform = pair.child.getWorldTransform();
    maxAnchorResidual = std::max(
        maxAnchorResidual,
        (parentTransform.translation() - childTransform.translation()).norm());
    maxHingeAxisTilt = std::max(
        maxHingeAxisTilt,
        (parentTransform.linear() * hingeAxis
         - childTransform.linear() * hingeAxis)
            .norm());
  }

  const Eigen::Matrix3d relativeRotation
      = pair.parent.getWorldTransform().linear().transpose()
        * pair.child.getWorldTransform().linear();
  const Eigen::Matrix3d expectedRelativeRotation
      = Eigen::AngleAxisd(targetSpeed * dt * steps, hingeAxis)
            .toRotationMatrix();
  EXPECT_LT(maxAnchorResidual, 1e-6);
  EXPECT_LT(maxHingeAxisTilt, 1e-6);
  EXPECT_LT((relativeRotation - expectedRelativeRotation).norm(), 1e-6);
}

// PLAN-104 AVBD articulated bridge: the same-multibody movable-link revolute
// facade must also keep its free-axis motor bounded by finite effort limits.
// A tiny positive torque limit keeps the anchor and hinge rows active without
// turning the velocity actuator into an unbounded relative-angle target.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedRevoluteJointFacadeMovablePairRespectsTinyTorqueLimit)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  const Eigen::Vector3d hingeAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  const Eigen::Vector3d parentAnchor(0.2, 0.0, 0.0);
  const Eigen::Vector3d childAnchor(-0.1, 0.0, 0.0);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = parentAnchor - childAnchor;
  pair.child.getParentJoint().setPosition(childPose);
  sx::Joint joint = world.addArticulatedRevoluteJoint(
      "movable_tiny_hinge",
      pair.parent,
      pair.child,
      hingeAxis,
      parentAnchor,
      childAnchor);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.4;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1e-9), Eigen::VectorXd::Constant(1, 1e-9));

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_LT((config.localAnchorA - parentAnchor).norm(), 1e-12);
  EXPECT_LT((config.localAnchorB - childAnchor).norm(), 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_NEAR(config.angularAxes.col(2).dot(hingeAxis), 1.0, 1e-12);

  constexpr int steps = 20;
  double maxAnchorResidual = 0.0;
  double maxHingeAxisTilt = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d parentTransform = pair.parent.getWorldTransform();
    const Eigen::Isometry3d childTransform = pair.child.getWorldTransform();
    maxAnchorResidual = std::max(
        maxAnchorResidual,
        (parentTransform * parentAnchor - childTransform * childAnchor).norm());
    maxHingeAxisTilt = std::max(
        maxHingeAxisTilt,
        (parentTransform.linear() * hingeAxis
         - childTransform.linear() * hingeAxis)
            .norm());
  }

  const Eigen::Matrix3d relativeRotation
      = pair.parent.getWorldTransform().linear().transpose()
        * pair.child.getWorldTransform().linear();
  const double relativeHingeMotion
      = signedRotationAroundAxis(relativeRotation, hingeAxis);
  EXPECT_LT(maxAnchorResidual, 1e-6);
  EXPECT_LT(maxHingeAxisTilt, 1e-3);
  EXPECT_LT(
      std::abs(relativeHingeMotion),
      0.01 * targetSpeed * world.getTimeStep() * steps);
}

// PLAN-104 AVBD articulated bridge: public revolute point joints can now use
// explicit link-local anchors instead of being limited to link origins. This is
// needed by source-demo joint grids and articulated fracture scenes whose
// constraints attach at off-origin points.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedRevoluteJointFacadeDrivesOffsetAnchorPair)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  const Eigen::Vector3d hingeAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  const Eigen::Vector3d parentAnchor(0.2, 0.0, 0.0);
  const Eigen::Vector3d childAnchor(-0.1, 0.0, 0.0);
  sx::Joint joint = world.addArticulatedRevoluteJoint(
      "offset_hinge",
      pair.parent,
      pair.child,
      hingeAxis,
      parentAnchor,
      childAnchor);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.4;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_LT((config.localAnchorA - parentAnchor).norm(), 1e-12);
  EXPECT_LT((config.localAnchorB - childAnchor).norm(), 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_NEAR(config.angularAxes.col(2).dot(hingeAxis), 1.0, 1e-12);

  constexpr int steps = 20;
  double maxAnchorResidual = 0.0;
  double maxHingeAxisTilt = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d parentTransform = pair.parent.getWorldTransform();
    const Eigen::Isometry3d childTransform = pair.child.getWorldTransform();
    maxAnchorResidual = std::max(
        maxAnchorResidual,
        (parentTransform * parentAnchor - childTransform * childAnchor).norm());
    maxHingeAxisTilt = std::max(
        maxHingeAxisTilt,
        (parentTransform.linear() * hingeAxis
         - childTransform.linear() * hingeAxis)
            .norm());
  }

  const Eigen::Matrix3d relativeRotation
      = pair.parent.getWorldTransform().linear().transpose()
        * pair.child.getWorldTransform().linear();
  const Eigen::Matrix3d expectedRelativeRotation
      = Eigen::AngleAxisd(targetSpeed * dt * steps, hingeAxis)
            .toRotationMatrix();
  EXPECT_LT(maxAnchorResidual, 1e-6);
  EXPECT_LT(maxHingeAxisTilt, 1e-6);
  EXPECT_LT((relativeRotation - expectedRelativeRotation).norm(), 1e-6);
}

// PLAN-104 AVBD articulated bridge: public same-multibody revolute motor
// facades must preserve explicit anchors, velocity actuator state, effort
// limits, and break-force metadata through design-mode binary save/load. The
// simulation-entry extractor then rebuilds the private hard rows plus the
// free-axis motor row from that serialized public facade state.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedRevoluteJointFacadeSurvivesSaveLoad)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  const Eigen::Vector3d hingeAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  const Eigen::Vector3d parentAnchor(0.2, 0.0, 0.0);
  const Eigen::Vector3d childAnchor(-0.1, 0.0, 0.0);
  sx::Joint joint = world.addArticulatedRevoluteJoint(
      "serialized_link_hinge",
      pair.parent,
      pair.child,
      hingeAxis,
      parentAnchor,
      childAnchor);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.4;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -800.0),
      Eigen::VectorXd::Constant(1, 900.0));
  joint.setBreakForce(11.0);

  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot = restored.getMultibody("floating_pair");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredParent = restoredRobot->getLink("parent");
  ASSERT_TRUE(restoredParent.has_value());
  auto restoredChild = restoredRobot->getLink("child");
  ASSERT_TRUE(restoredChild.has_value());
  auto restoredJoint = restored.getArticulatedJoint("serialized_link_hinge");
  ASSERT_TRUE(restoredJoint.has_value());
  EXPECT_EQ(restoredJoint->getType(), sx::JointType::Revolute);
  EXPECT_EQ(restoredJoint->getDOFCount(), 1u);
  EXPECT_EQ(restoredJoint->getParentLink().getName(), "parent");
  EXPECT_EQ(restoredJoint->getChildLink().getName(), "child");
  EXPECT_LT((restoredJoint->getAxis() - hingeAxis).norm(), 1e-12);
  EXPECT_EQ(restoredJoint->getActuatorType(), sx::ActuatorType::Velocity);
  ASSERT_EQ(restoredJoint->getCommandVelocity().size(), 1);
  EXPECT_DOUBLE_EQ(restoredJoint->getCommandVelocity()[0], targetSpeed);
  ASSERT_EQ(restoredJoint->getEffortLowerLimits().size(), 1);
  ASSERT_EQ(restoredJoint->getEffortUpperLimits().size(), 1);
  EXPECT_DOUBLE_EQ(restoredJoint->getEffortLowerLimits()[0], -800.0);
  EXPECT_DOUBLE_EQ(restoredJoint->getEffortUpperLimits()[0], 900.0);
  EXPECT_DOUBLE_EQ(restoredJoint->getBreakForce(), 11.0);
  EXPECT_FALSE(restoredJoint->isBroken());

  auto& registry = dart::simulation::detail::registryOf(restored);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(restoredJoint->getEntity());
  EXPECT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  restored.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_LT((config.localAnchorA - parentAnchor).norm(), 1e-12);
  EXPECT_LT((config.localAnchorB - childAnchor).norm(), 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_NEAR(config.angularAxes.col(2).dot(hingeAxis), 1.0, 1e-12);
  EXPECT_TRUE(std::isinf(config.startStiffness));
  EXPECT_TRUE(std::isinf(config.maxStiffness));

  constexpr int steps = 20;
  double maxAnchorResidual = 0.0;
  double maxHingeAxisTilt = 0.0;
  for (int k = 0; k < steps; ++k) {
    restored.step();

    const Eigen::Isometry3d parentTransform
        = restoredParent->getWorldTransform();
    const Eigen::Isometry3d childTransform = restoredChild->getWorldTransform();
    maxAnchorResidual = std::max(
        maxAnchorResidual,
        (parentTransform * parentAnchor - childTransform * childAnchor).norm());
    maxHingeAxisTilt = std::max(
        maxHingeAxisTilt,
        (parentTransform.linear() * hingeAxis
         - childTransform.linear() * hingeAxis)
            .norm());
  }

  const Eigen::Matrix3d relativeRotation
      = restoredParent->getWorldTransform().linear().transpose()
        * restoredChild->getWorldTransform().linear();
  const Eigen::Matrix3d expectedRelativeRotation
      = Eigen::AngleAxisd(targetSpeed * dt * steps, hingeAxis)
            .toRotationMatrix();
  EXPECT_LT(maxAnchorResidual, 1e-6);
  EXPECT_LT(maxHingeAxisTilt, 1e-6);
  EXPECT_LT((relativeRotation - expectedRelativeRotation).norm(), 1e-6);
}

// PLAN-104 AVBD articulated bridge: public articulated prismatic joints also
// route through the same current-pose extractor and can use the existing
// one-DOF velocity actuator bridge along a non-cardinal free linear axis.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedPrismaticJointFacadeDrivesFloatingLinkEndpoint)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto robot = world.addMultibody("public_prismatic_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(1.0, 2.0, 0.5).normalized();
  sx::Joint joint = world.addArticulatedPrismaticJoint(
      "base_slider", base, body, sliderAxis);
  ASSERT_EQ(joint.getType(), sx::JointType::Prismatic);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.3;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(config.linearAxes.col(2).dot(sliderAxis), 1.0, 1e-12);

  constexpr int steps = 20;
  double maxOrthogonalDrift = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    const Eigen::Vector3d position = transform.translation();
    maxOrthogonalDrift = std::max(
        maxOrthogonalDrift,
        (position - position.dot(sliderAxis) * sliderAxis).norm());
    maxRotationError = std::max(
        maxRotationError,
        (transform.linear() - Eigen::Matrix3d::Identity()).norm());
  }

  const Eigen::Vector3d position = body.getWorldTransform().translation();
  EXPECT_LT(maxOrthogonalDrift, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
  EXPECT_NEAR(position.dot(sliderAxis), targetSpeed * dt * steps, 1e-6);
}

// PLAN-104 AVBD articulated bridge: public articulated prismatic facades expose
// the finite effort bound on the private free-axis linear motor row.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedPrismaticJointFacadeRespectsTinyForceLimit)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto robot = world.addMultibody("public_prismatic_tiny_limit_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(1.0, 2.0, 0.5).normalized();
  const Eigen::Vector3d parentAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d childAnchor(-0.1, 0.1, 0.0);
  Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
  pose.head<3>() = parentAnchor - childAnchor;
  body.getParentJoint().setPosition(pose);
  sx::Joint joint = world.addArticulatedPrismaticJoint(
      "base_slider", base, body, sliderAxis, parentAnchor, childAnchor);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.3;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1e-9), Eigen::VectorXd::Constant(1, 1e-9));

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_LT((config.localAnchorA - parentAnchor).norm(), 1e-12);
  EXPECT_LT((config.localAnchorB - childAnchor).norm(), 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(config.linearAxes.col(2).dot(sliderAxis), 1.0, 1e-12);

  constexpr int steps = 20;
  double maxOrthogonalDrift = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    const Eigen::Vector3d anchorOffset = transform * childAnchor - parentAnchor;
    maxOrthogonalDrift = std::max(
        maxOrthogonalDrift,
        (anchorOffset - anchorOffset.dot(sliderAxis) * sliderAxis).norm());
    maxRotationError = std::max(
        maxRotationError,
        (transform.linear() - Eigen::Matrix3d::Identity()).norm());
  }

  const Eigen::Vector3d anchorOffset
      = body.getWorldTransform() * childAnchor - parentAnchor;
  EXPECT_LT(maxOrthogonalDrift, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
  EXPECT_LT(
      std::abs(anchorOffset.dot(sliderAxis)),
      0.01 * targetSpeed * world.getTimeStep() * steps);
}

// PLAN-104 AVBD articulated bridge: public articulated prismatic velocity
// commands are also sampled from the source joint each step, matching the
// revolute facade and the private point-joint velocity actuator lifecycle.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedPrismaticJointFacadeFollowsCommandUpdates)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto robot = world.addMultibody("public_prismatic_command_update_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  const Eigen::Vector3d sliderAxis = Eigen::Vector3d::UnitX();
  sx::Joint joint = world.addArticulatedPrismaticJoint(
      "base_slider", base, body, sliderAxis);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.3;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));

  world.enterSimulationMode();

  double maxOrthogonalDrift = 0.0;
  double maxRotationError = 0.0;
  auto stepAndSliderPosition = [&]() {
    world.step();
    const Eigen::Isometry3d transform = body.getWorldTransform();
    const Eigen::Vector3d position = transform.translation();
    maxOrthogonalDrift = std::max(
        maxOrthogonalDrift,
        (position - position.dot(sliderAxis) * sliderAxis).norm());
    maxRotationError = std::max(
        maxRotationError,
        (transform.linear() - Eigen::Matrix3d::Identity()).norm());
    return position.dot(sliderAxis);
  };

  constexpr int forwardSteps = 10;
  double positionAfterForward = 0.0;
  for (int k = 0; k < forwardSteps; ++k) {
    positionAfterForward = stepAndSliderPosition();
  }

  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, -targetSpeed));

  constexpr int reverseSteps = 10;
  double positionAfterReverse = positionAfterForward;
  for (int k = 0; k < reverseSteps; ++k) {
    positionAfterReverse = stepAndSliderPosition();
  }

  EXPECT_LT(maxOrthogonalDrift, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
  EXPECT_NEAR(positionAfterForward, targetSpeed * dt * forwardSteps, 1e-6);
  EXPECT_NEAR(
      positionAfterReverse,
      targetSpeed * dt * (forwardSteps - reverseSteps),
      1e-6);
}

// PLAN-104 AVBD articulated bridge: same-multibody public prismatic point
// joints likewise project both movable endpoints through the variational
// bridge while preserving the two locked translation rows and all angular rows.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedPrismaticJointFacadeDrivesMovableLinkPair)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  const FloatingLinkPair pair = addFloatingLinkPair(world);
  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(1.0, 2.0, 0.5).normalized();
  sx::Joint joint = world.addArticulatedPrismaticJoint(
      "movable_slider", pair.parent, pair.child, sliderAxis);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.3;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(config.linearAxes.col(2).dot(sliderAxis), 1.0, 1e-12);

  constexpr int steps = 20;
  double maxOrthogonalResidual = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d parentTransform = pair.parent.getWorldTransform();
    const Eigen::Isometry3d childTransform = pair.child.getWorldTransform();
    const Eigen::Vector3d relativePosition
        = childTransform.translation() - parentTransform.translation();
    maxOrthogonalResidual = std::max(
        maxOrthogonalResidual,
        (relativePosition - relativePosition.dot(sliderAxis) * sliderAxis)
            .norm());
    maxRotationError = std::max(
        maxRotationError,
        (parentTransform.linear().transpose() * childTransform.linear()
         - Eigen::Matrix3d::Identity())
            .norm());
  }

  const Eigen::Vector3d relativePosition
      = pair.child.getWorldTransform().translation()
        - pair.parent.getWorldTransform().translation();
  EXPECT_LT(maxOrthogonalResidual, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
  EXPECT_NEAR(relativePosition.dot(sliderAxis), targetSpeed * dt * steps, 1e-6);
}

// PLAN-104 AVBD articulated bridge: same-multibody public prismatic motors on
// two movable links must preserve the locked axes while honoring a finite
// effort cap on the free linear motor row.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedPrismaticJointFacadeMovablePairRespectsTinyForceLimit)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(1.0, 2.0, 0.5).normalized();
  const Eigen::Vector3d parentAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d childAnchor(-0.1, 0.1, 0.0);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = parentAnchor - childAnchor;
  pair.child.getParentJoint().setPosition(childPose);
  sx::Joint joint = world.addArticulatedPrismaticJoint(
      "movable_tiny_slider",
      pair.parent,
      pair.child,
      sliderAxis,
      parentAnchor,
      childAnchor);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.3;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1e-9), Eigen::VectorXd::Constant(1, 1e-9));

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_LT((config.localAnchorA - parentAnchor).norm(), 1e-12);
  EXPECT_LT((config.localAnchorB - childAnchor).norm(), 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(config.linearAxes.col(2).dot(sliderAxis), 1.0, 1e-12);

  constexpr int steps = 20;
  double maxOrthogonalResidual = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d parentTransform = pair.parent.getWorldTransform();
    const Eigen::Isometry3d childTransform = pair.child.getWorldTransform();
    const Eigen::Vector3d relativeAnchor
        = childTransform * childAnchor - parentTransform * parentAnchor;
    maxOrthogonalResidual = std::max(
        maxOrthogonalResidual,
        (relativeAnchor - relativeAnchor.dot(sliderAxis) * sliderAxis).norm());
    maxRotationError = std::max(
        maxRotationError,
        (parentTransform.linear().transpose() * childTransform.linear()
         - Eigen::Matrix3d::Identity())
            .norm());
  }

  const Eigen::Vector3d relativeAnchor
      = pair.child.getWorldTransform() * childAnchor
        - pair.parent.getWorldTransform() * parentAnchor;
  EXPECT_LT(maxOrthogonalResidual, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
  EXPECT_LT(
      std::abs(relativeAnchor.dot(sliderAxis)),
      0.01 * targetSpeed * world.getTimeStep() * steps);
}

// PLAN-104 AVBD articulated bridge: public prismatic point joints also preserve
// explicit link-local off-origin anchors while leaving only the configured
// slider direction free between two movable links.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedPrismaticJointFacadeDrivesOffsetAnchorPair)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  const Eigen::Vector3d sliderAxis = Eigen::Vector3d::UnitX();
  const Eigen::Vector3d parentAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d childAnchor(-0.1, 0.1, 0.0);
  sx::Joint joint = world.addArticulatedPrismaticJoint(
      "offset_slider",
      pair.parent,
      pair.child,
      sliderAxis,
      parentAnchor,
      childAnchor);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.3;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_LT((config.localAnchorA - parentAnchor).norm(), 1e-12);
  EXPECT_LT((config.localAnchorB - childAnchor).norm(), 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(config.linearAxes.col(2).dot(sliderAxis), 1.0, 1e-12);

  constexpr int steps = 20;
  double maxOrthogonalResidual = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d parentTransform = pair.parent.getWorldTransform();
    const Eigen::Isometry3d childTransform = pair.child.getWorldTransform();
    const Eigen::Vector3d relativeAnchor
        = childTransform * childAnchor - parentTransform * parentAnchor;
    maxOrthogonalResidual = std::max(
        maxOrthogonalResidual,
        (relativeAnchor - relativeAnchor.dot(sliderAxis) * sliderAxis).norm());
    maxRotationError = std::max(
        maxRotationError,
        (parentTransform.linear().transpose() * childTransform.linear()
         - Eigen::Matrix3d::Identity())
            .norm());
  }

  const Eigen::Vector3d relativeAnchor
      = pair.child.getWorldTransform() * childAnchor
        - pair.parent.getWorldTransform() * parentAnchor;
  EXPECT_LT(maxOrthogonalResidual, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
  EXPECT_NEAR(relativeAnchor.dot(sliderAxis), targetSpeed * dt * steps, 1e-6);
}

// PLAN-104 AVBD articulated bridge: same-multibody public prismatic motor
// facades must preserve explicit anchors, velocity actuator state, effort
// limits, and break-force metadata through design-mode binary save/load. The
// restored facade then rebuilds the private locked rows plus the free-axis
// linear motor row at simulation entry.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedPrismaticJointFacadeSurvivesSaveLoad)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  const Eigen::Vector3d sliderAxis = Eigen::Vector3d::UnitX();
  const Eigen::Vector3d parentAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d childAnchor(-0.1, 0.1, 0.0);
  sx::Joint joint = world.addArticulatedPrismaticJoint(
      "serialized_link_slider",
      pair.parent,
      pair.child,
      sliderAxis,
      parentAnchor,
      childAnchor);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.3;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -700.0),
      Eigen::VectorXd::Constant(1, 800.0));
  joint.setBreakForce(12.0);

  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot = restored.getMultibody("floating_pair");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredParent = restoredRobot->getLink("parent");
  ASSERT_TRUE(restoredParent.has_value());
  auto restoredChild = restoredRobot->getLink("child");
  ASSERT_TRUE(restoredChild.has_value());
  auto restoredJoint = restored.getArticulatedJoint("serialized_link_slider");
  ASSERT_TRUE(restoredJoint.has_value());
  EXPECT_EQ(restoredJoint->getType(), sx::JointType::Prismatic);
  EXPECT_EQ(restoredJoint->getDOFCount(), 1u);
  EXPECT_EQ(restoredJoint->getParentLink().getName(), "parent");
  EXPECT_EQ(restoredJoint->getChildLink().getName(), "child");
  EXPECT_LT((restoredJoint->getAxis() - sliderAxis).norm(), 1e-12);
  EXPECT_EQ(restoredJoint->getActuatorType(), sx::ActuatorType::Velocity);
  ASSERT_EQ(restoredJoint->getCommandVelocity().size(), 1);
  EXPECT_DOUBLE_EQ(restoredJoint->getCommandVelocity()[0], targetSpeed);
  ASSERT_EQ(restoredJoint->getEffortLowerLimits().size(), 1);
  ASSERT_EQ(restoredJoint->getEffortUpperLimits().size(), 1);
  EXPECT_DOUBLE_EQ(restoredJoint->getEffortLowerLimits()[0], -700.0);
  EXPECT_DOUBLE_EQ(restoredJoint->getEffortUpperLimits()[0], 800.0);
  EXPECT_DOUBLE_EQ(restoredJoint->getBreakForce(), 12.0);
  EXPECT_FALSE(restoredJoint->isBroken());

  auto& registry = dart::simulation::detail::registryOf(restored);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(restoredJoint->getEntity());
  EXPECT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  restored.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_LT((config.localAnchorA - parentAnchor).norm(), 1e-12);
  EXPECT_LT((config.localAnchorB - childAnchor).norm(), 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(config.linearAxes.col(2).dot(sliderAxis), 1.0, 1e-12);
  EXPECT_TRUE(std::isinf(config.startStiffness));
  EXPECT_TRUE(std::isinf(config.maxStiffness));

  constexpr int steps = 20;
  double maxOrthogonalResidual = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < steps; ++k) {
    restored.step();

    const Eigen::Isometry3d parentTransform
        = restoredParent->getWorldTransform();
    const Eigen::Isometry3d childTransform = restoredChild->getWorldTransform();
    const Eigen::Vector3d relativeAnchor
        = childTransform * childAnchor - parentTransform * parentAnchor;
    maxOrthogonalResidual = std::max(
        maxOrthogonalResidual,
        (relativeAnchor - relativeAnchor.dot(sliderAxis) * sliderAxis).norm());
    maxRotationError = std::max(
        maxRotationError,
        (parentTransform.linear().transpose() * childTransform.linear()
         - Eigen::Matrix3d::Identity())
            .norm());
  }

  const Eigen::Vector3d relativeAnchor
      = restoredChild->getWorldTransform() * childAnchor
        - restoredParent->getWorldTransform() * parentAnchor;
  EXPECT_LT(maxOrthogonalResidual, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
  EXPECT_NEAR(relativeAnchor.dot(sliderAxis), targetSpeed * dt * steps, 1e-6);
}

// PLAN-104 AVBD articulated bridge: the public articulated facade also carries
// the break-force lifecycle through the variational projection path. Once the
// projection load exceeds the threshold, later steps skip the broken point
// joint and the floating link can move away from the captured pose.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedBreakForceMarksJointBrokenAndSkipsLaterSteps)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto robot = world.addMultibody("public_breakable_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  sx::Joint joint
      = world.addArticulatedFixedJoint("breakable_hold", base, body);
  joint.setBreakForce(1e-18);

  world.enterSimulationMode();

  body.applyForce(4.0 * Eigen::Vector3d::UnitY());
  world.step();
  EXPECT_TRUE(joint.isBroken());

  double maxTranslation = 0.0;
  for (int k = 0; k < 20; ++k) {
    body.applyForce(4.0 * Eigen::Vector3d::UnitY());
    world.step();
    maxTranslation = std::max(
        maxTranslation, body.getWorldTransform().translation().norm());
  }

  EXPECT_GT(maxTranslation, 1e-4);
}

// PLAN-104 AVBD articulated bridge: resetting a broken public articulated
// point joint re-enables its existing AVBD config, so the next variational
// projection pulls the floating link back to the captured pose instead of
// leaving the joint permanently skipped.
TEST(VariationalIntegration, AvbdPublicArticulatedBreakForceResetReengagesJoint)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto robot = world.addMultibody("public_breakable_reset_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  Eigen::VectorXd pose(6);
  pose << 0.1, -0.2, 0.15, 0.0, 0.0, 0.2;
  body.getParentJoint().setPosition(pose);

  sx::Joint joint
      = world.addArticulatedFixedJoint("resettable_hold", base, body);
  joint.setBreakForce(1e-18);

  world.enterSimulationMode();
  const Eigen::Isometry3d captured = body.getWorldTransform();

  body.applyForce(4.0 * Eigen::Vector3d::UnitY());
  world.step();
  ASSERT_TRUE(joint.isBroken());

  double maxBrokenTranslationError = 0.0;
  for (int k = 0; k < 20; ++k) {
    body.applyForce(4.0 * Eigen::Vector3d::UnitY());
    world.step();
    maxBrokenTranslationError = std::max(
        maxBrokenTranslationError,
        (body.getWorldTransform().translation() - captured.translation())
            .norm());
  }
  ASSERT_GT(maxBrokenTranslationError, 1e-4);

  joint.setBreakForce(1.0e12);
  joint.resetBreakage();
  ASSERT_FALSE(joint.isBroken());

  world.step();

  const Eigen::Isometry3d resetTransform = body.getWorldTransform();
  EXPECT_FALSE(joint.isBroken());
  EXPECT_LT(
      (resetTransform.translation() - captured.translation()).norm(), 1e-6);
  EXPECT_LT((resetTransform.linear() - captured.linear()).norm(), 1e-6);
}

// PLAN-104 AVBD articulated bridge: broken-state bookkeeping must also survive
// binary save/load while the world is already in simulation mode. The restored
// public fixed joint remains skipped until reset, then the existing explicit
// anchor rows re-engage against the pose captured during load.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedBreakageSurvivesSaveLoadAndReset)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto robot = world.addMultibody("public_breakable_save_load_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  Eigen::VectorXd pose(6);
  pose << 0.1, -0.2, 0.15, 0.0, 0.0, 0.2;
  body.getParentJoint().setPosition(pose);

  const Eigen::Vector3d childAnchor(-0.15, 0.05, 0.0);
  const Eigen::Vector3d parentAnchor = body.getWorldTransform() * childAnchor;
  sx::Joint joint = world.addArticulatedFixedJoint(
      "serialized_broken_hold", base, body, parentAnchor, childAnchor);
  joint.setBreakForce(1e-18);

  world.enterSimulationMode();

  body.applyForce(
      4.0 * Eigen::Vector3d::UnitY(),
      childAnchor + 0.4 * Eigen::Vector3d::UnitX());
  world.step();
  ASSERT_TRUE(joint.isBroken());

  const Eigen::Isometry3d savedBrokenTransform = body.getWorldTransform();
  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot
      = restored.getMultibody("public_breakable_save_load_facade");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredBody = restoredRobot->getLink("body");
  ASSERT_TRUE(restoredBody.has_value());
  auto restoredJoint = restored.getArticulatedJoint("serialized_broken_hold");
  ASSERT_TRUE(restoredJoint.has_value());
  ASSERT_TRUE(restoredJoint->isBroken());
  EXPECT_EQ(restoredJoint->getType(), sx::JointType::Fixed);
  EXPECT_EQ(restoredJoint->getDOFCount(), 0u);
  EXPECT_EQ(restoredJoint->getParentLink().getName(), "base");
  EXPECT_EQ(restoredJoint->getChildLink().getName(), "body");
  EXPECT_DOUBLE_EQ(restoredJoint->getBreakForce(), 1e-18);
  EXPECT_LT(
      (restoredBody->getWorldTransform().translation()
       - savedBrokenTransform.translation())
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredBody->getWorldTransform().linear()
       - savedBrokenTransform.linear())
          .norm(),
      1e-12);

  const Eigen::Isometry3d restoredCaptured = restoredBody->getWorldTransform();
  double maxBrokenAnchorResidual = 0.0;
  double maxBrokenRotationError = 0.0;
  for (int k = 0; k < 20; ++k) {
    restoredBody->applyForce(
        4.0 * Eigen::Vector3d::UnitY(),
        childAnchor + 0.4 * Eigen::Vector3d::UnitX());
    restored.step();

    const Eigen::Isometry3d transform = restoredBody->getWorldTransform();
    maxBrokenAnchorResidual = std::max(
        maxBrokenAnchorResidual,
        (transform * childAnchor - parentAnchor).norm());
    maxBrokenRotationError = std::max(
        maxBrokenRotationError,
        (transform.linear() - restoredCaptured.linear()).norm());
  }
  ASSERT_GT(maxBrokenAnchorResidual, 1e-4);
  ASSERT_GT(maxBrokenRotationError, 1e-4);

  restoredBody->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredJoint->setBreakForce(1.0e12);
  restoredJoint->resetBreakage();
  ASSERT_FALSE(restoredJoint->isBroken());

  restored.step();

  const Eigen::Isometry3d resetTransform = restoredBody->getWorldTransform();
  EXPECT_FALSE(restoredJoint->isBroken());
  EXPECT_LT((resetTransform * childAnchor - parentAnchor).norm(), 1e-6);
  EXPECT_LT((resetTransform.linear() - restoredCaptured.linear()).norm(), 1e-6);
}

// PLAN-104 AVBD articulated bridge: break/reset bookkeeping also works when a
// public fixed point joint connects two movable same-multibody links. Resetting
// the joint re-enables the two-sided variational projection and restores the
// captured relative pose rather than anchoring either endpoint to world.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedBreakForceResetReengagesMovableLinkPair)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  sx::Joint joint = world.addArticulatedFixedJoint(
      "movable_breakable_hold", pair.parent, pair.child);
  joint.setBreakForce(1e-18);

  world.enterSimulationMode();
  const auto relativeTransform = [&]() {
    return pair.parent.getWorldTransform().inverse()
           * pair.child.getWorldTransform();
  };
  const Eigen::Isometry3d capturedRelative = relativeTransform();

  pair.parent.applyForce(-4.0 * Eigen::Vector3d::UnitY());
  pair.child.applyForce(4.0 * Eigen::Vector3d::UnitY());
  world.step();
  ASSERT_TRUE(joint.isBroken());

  double maxBrokenRelativeTranslationError = 0.0;
  for (int k = 0; k < 20; ++k) {
    pair.parent.applyForce(-4.0 * Eigen::Vector3d::UnitY());
    pair.child.applyForce(4.0 * Eigen::Vector3d::UnitY());
    world.step();

    const Eigen::Isometry3d transform = relativeTransform();
    maxBrokenRelativeTranslationError = std::max(
        maxBrokenRelativeTranslationError,
        (transform.translation() - capturedRelative.translation()).norm());
  }
  ASSERT_GT(maxBrokenRelativeTranslationError, 1e-4);

  pair.parent.getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  pair.child.getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  joint.setBreakForce(1.0e12);
  joint.resetBreakage();
  ASSERT_FALSE(joint.isBroken());

  world.step();

  const Eigen::Isometry3d resetRelative = relativeTransform();
  EXPECT_FALSE(joint.isBroken());
  EXPECT_LT(
      (resetRelative.translation() - capturedRelative.translation()).norm(),
      1e-6);
  EXPECT_LT((resetRelative.linear() - capturedRelative.linear()).norm(), 1e-6);
}

// PLAN-104 AVBD articulated bridge: movable same-multibody revolute velocity
// motors also break, skip, and re-engage when their constrained rows are built
// from explicit off-origin link anchors.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedRevoluteBreakForceResetReengagesMovableOffsetPair)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  const Eigen::Vector3d hingeAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  const Eigen::Vector3d parentAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d childAnchor(-0.1, 0.1, 0.0);
  sx::Joint joint = world.addArticulatedRevoluteJoint(
      "movable_offset_breakable_hinge",
      pair.parent,
      pair.child,
      hingeAxis,
      parentAnchor,
      childAnchor);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.4;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));
  joint.setBreakForce(1e-18);

  const auto parentAnchorWorld = [&]() {
    return pair.parent.getWorldTransform() * parentAnchor;
  };
  const auto childAnchorWorld = [&]() {
    return pair.child.getWorldTransform() * childAnchor;
  };
  const auto anchorResidual = [&]() {
    return (childAnchorWorld() - parentAnchorWorld()).norm();
  };
  const auto axisTilt = [&]() {
    return (pair.parent.getWorldTransform().linear() * hingeAxis
            - pair.child.getWorldTransform().linear() * hingeAxis)
        .norm();
  };
  const auto relativeHingePosition = [&]() {
    const Eigen::Matrix3d relativeRotation
        = pair.parent.getWorldTransform().linear().transpose()
          * pair.child.getWorldTransform().linear();
    return signedRotationAroundAxis(relativeRotation, hingeAxis);
  };

  world.enterSimulationMode();
  world.step();

  ASSERT_TRUE(joint.isBroken());
  const double firstStepHingePosition = relativeHingePosition();
  EXPECT_NEAR(firstStepHingePosition, targetSpeed * dt, 1e-6);
  EXPECT_LT(anchorResidual(), 1e-6);
  EXPECT_LT(axisTilt(), 1e-6);

  double maxBrokenAnchorResidual = 0.0;
  for (int k = 0; k < 20; ++k) {
    pair.parent.applyForce(
        -4.0 * Eigen::Vector3d::UnitY(),
        parentAnchor + 0.3 * Eigen::Vector3d::UnitX());
    pair.child.applyForce(
        4.0 * Eigen::Vector3d::UnitY(),
        childAnchor - 0.3 * Eigen::Vector3d::UnitX());
    world.step();

    maxBrokenAnchorResidual
        = std::max(maxBrokenAnchorResidual, anchorResidual());
  }
  ASSERT_GT(maxBrokenAnchorResidual, 1e-4);

  pair.parent.getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  pair.child.getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  joint.setBreakForce(1.0e12);
  joint.resetBreakage();

  ASSERT_FALSE(joint.isBroken());

  world.step();

  EXPECT_FALSE(joint.isBroken());
  EXPECT_LT(anchorResidual(), 1e-6);
  EXPECT_LT(axisTilt(), 2e-3);
  EXPECT_GT(relativeHingePosition(), firstStepHingePosition);
}

// PLAN-104 AVBD articulated bridge: movable same-multibody prismatic velocity
// motors also break, skip, and re-engage when their locked rows are built from
// explicit off-origin link anchors.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedPrismaticBreakForceResetReengagesMovableOffsetPair)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  const Eigen::Vector3d sliderAxis = Eigen::Vector3d::UnitX();
  const Eigen::Vector3d parentAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d childAnchor(-0.1, 0.1, 0.0);
  sx::Joint joint = world.addArticulatedPrismaticJoint(
      "movable_offset_breakable_slider",
      pair.parent,
      pair.child,
      sliderAxis,
      parentAnchor,
      childAnchor);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.3;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));
  joint.setBreakForce(1e-18);

  const auto anchorDelta = [&]() -> Eigen::Vector3d {
    const Eigen::Vector3d parentAnchorWorld
        = pair.parent.getWorldTransform() * parentAnchor;
    const Eigen::Vector3d childAnchorWorld
        = pair.child.getWorldTransform() * childAnchor;
    return childAnchorWorld - parentAnchorWorld;
  };
  const auto orthogonalAnchorResidual = [&]() {
    const Eigen::Vector3d delta = anchorDelta();
    return (delta - delta.dot(sliderAxis) * sliderAxis).norm();
  };
  const auto relativeRotationError = [&]() {
    return (pair.parent.getWorldTransform().linear().transpose()
                * pair.child.getWorldTransform().linear()
            - Eigen::Matrix3d::Identity())
        .norm();
  };

  world.enterSimulationMode();
  world.step();

  ASSERT_TRUE(joint.isBroken());
  const Eigen::Vector3d firstStepDelta = anchorDelta();
  EXPECT_NEAR(firstStepDelta.dot(sliderAxis), targetSpeed * dt, 1e-6);
  EXPECT_LT(orthogonalAnchorResidual(), 1e-6);
  EXPECT_LT(relativeRotationError(), 1e-6);

  double maxBrokenOrthogonalResidual = 0.0;
  for (int k = 0; k < 20; ++k) {
    pair.parent.applyForce(
        -4.0 * Eigen::Vector3d::UnitY(),
        parentAnchor + 0.3 * Eigen::Vector3d::UnitX());
    pair.child.applyForce(
        4.0 * Eigen::Vector3d::UnitY(),
        childAnchor - 0.3 * Eigen::Vector3d::UnitX());
    world.step();

    maxBrokenOrthogonalResidual
        = std::max(maxBrokenOrthogonalResidual, orthogonalAnchorResidual());
  }
  ASSERT_GT(maxBrokenOrthogonalResidual, 1e-4);

  pair.parent.getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  pair.child.getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  joint.setBreakForce(1.0e12);
  joint.resetBreakage();

  ASSERT_FALSE(joint.isBroken());

  world.step();

  EXPECT_FALSE(joint.isBroken());
  const double resetOrthogonalResidual = orthogonalAnchorResidual();
  EXPECT_LT(resetOrthogonalResidual, maxBrokenOrthogonalResidual * 0.05);
  EXPECT_LT(resetOrthogonalResidual, 2e-3);
  EXPECT_LT(relativeRotationError(), 1e-6);
  EXPECT_GT(anchorDelta().dot(sliderAxis), firstStepDelta.dot(sliderAxis));
}

// PLAN-104 AVBD articulated bridge: a broken public fixed point joint between
// two movable same-multibody links must remain skipped across simulation-mode
// binary save/load, then rebuild its explicit anchor rows after reset.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedMovableFixedBreakageSurvivesSaveLoadAndReset)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  const Eigen::Vector3d parentAnchor(0.15, 0.05, 0.0);
  const Eigen::Vector3d childAnchor(-0.15, 0.05, 0.0);
  sx::Joint joint = world.addArticulatedFixedJoint(
      "serialized_movable_broken_hold",
      pair.parent,
      pair.child,
      parentAnchor,
      childAnchor);
  joint.setBreakForce(1e-18);

  world.enterSimulationMode();

  pair.parent.applyForce(
      -4.0 * Eigen::Vector3d::UnitY(),
      parentAnchor + 0.4 * Eigen::Vector3d::UnitX());
  pair.child.applyForce(
      4.0 * Eigen::Vector3d::UnitY(),
      childAnchor - 0.4 * Eigen::Vector3d::UnitX());
  world.step();
  ASSERT_TRUE(joint.isBroken());

  const Eigen::Isometry3d savedParentTransform
      = pair.parent.getWorldTransform();
  const Eigen::Isometry3d savedChildTransform = pair.child.getWorldTransform();
  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot = restored.getMultibody("floating_pair");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredParent = restoredRobot->getLink("parent");
  ASSERT_TRUE(restoredParent.has_value());
  auto restoredChild = restoredRobot->getLink("child");
  ASSERT_TRUE(restoredChild.has_value());
  auto restoredJoint
      = restored.getArticulatedJoint("serialized_movable_broken_hold");
  ASSERT_TRUE(restoredJoint.has_value());
  ASSERT_TRUE(restoredJoint->isBroken());
  EXPECT_DOUBLE_EQ(restoredJoint->getBreakForce(), 1e-18);
  EXPECT_EQ(restoredJoint->getType(), sx::JointType::Fixed);
  EXPECT_EQ(restoredJoint->getDOFCount(), 0u);
  EXPECT_EQ(restoredJoint->getParentLink().getName(), "parent");
  EXPECT_EQ(restoredJoint->getChildLink().getName(), "child");
  EXPECT_LT(
      (restoredParent->getWorldTransform().translation()
       - savedParentTransform.translation())
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredParent->getWorldTransform().linear()
       - savedParentTransform.linear())
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredChild->getWorldTransform().translation()
       - savedChildTransform.translation())
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredChild->getWorldTransform().linear()
       - savedChildTransform.linear())
          .norm(),
      1e-12);

  const auto anchorResidual = [&]() {
    return (restoredParent->getWorldTransform() * parentAnchor
            - restoredChild->getWorldTransform() * childAnchor)
        .norm();
  };
  const auto relativeRotation = [&]() {
    return restoredParent->getWorldTransform().linear().transpose()
           * restoredChild->getWorldTransform().linear();
  };
  const Eigen::Matrix3d restoredRelativeRotation = relativeRotation();

  double maxBrokenAnchorResidual = 0.0;
  double maxBrokenRelativeRotationError = 0.0;
  for (int k = 0; k < 20; ++k) {
    restoredParent->applyForce(
        -4.0 * Eigen::Vector3d::UnitY(),
        parentAnchor + 0.4 * Eigen::Vector3d::UnitX());
    restoredChild->applyForce(
        4.0 * Eigen::Vector3d::UnitY(),
        childAnchor - 0.4 * Eigen::Vector3d::UnitX());
    restored.step();

    maxBrokenAnchorResidual
        = std::max(maxBrokenAnchorResidual, anchorResidual());
    maxBrokenRelativeRotationError = std::max(
        maxBrokenRelativeRotationError,
        (relativeRotation() - restoredRelativeRotation).norm());
  }
  ASSERT_GT(maxBrokenAnchorResidual, 1e-4);
  ASSERT_GT(maxBrokenRelativeRotationError, 1e-4);

  restoredParent->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredChild->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredJoint->setBreakForce(1.0e12);
  restoredJoint->resetBreakage();
  ASSERT_FALSE(restoredJoint->isBroken());

  restored.step();

  EXPECT_FALSE(restoredJoint->isBroken());
  EXPECT_LT(anchorResidual(), 1e-6);
  EXPECT_LT((relativeRotation() - restoredRelativeRotation).norm(), 1e-6);
}

// PLAN-104 AVBD articulated bridge: broken public revolute velocity motors
// between two movable same-multibody links must also survive simulation-mode
// binary save/load, then rebuild both masked hard rows and the free-axis motor
// through the public reset path.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedMovableRevoluteBreakageSurvivesSaveLoadAndReset)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  const Eigen::Vector3d hingeAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  const Eigen::Vector3d parentAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d childAnchor(-0.1, 0.1, 0.0);
  sx::Joint joint = world.addArticulatedRevoluteJoint(
      "serialized_movable_broken_hinge",
      pair.parent,
      pair.child,
      hingeAxis,
      parentAnchor,
      childAnchor);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.4;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));
  joint.setBreakForce(1e-18);

  world.enterSimulationMode();
  world.step();
  ASSERT_TRUE(joint.isBroken());

  const Eigen::Isometry3d savedParentTransform
      = pair.parent.getWorldTransform();
  const Eigen::Isometry3d savedChildTransform = pair.child.getWorldTransform();
  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot = restored.getMultibody("floating_pair");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredParent = restoredRobot->getLink("parent");
  ASSERT_TRUE(restoredParent.has_value());
  auto restoredChild = restoredRobot->getLink("child");
  ASSERT_TRUE(restoredChild.has_value());
  auto restoredJoint
      = restored.getArticulatedJoint("serialized_movable_broken_hinge");
  ASSERT_TRUE(restoredJoint.has_value());
  ASSERT_TRUE(restoredJoint->isBroken());
  EXPECT_DOUBLE_EQ(restoredJoint->getBreakForce(), 1e-18);
  EXPECT_EQ(restoredJoint->getType(), sx::JointType::Revolute);
  EXPECT_EQ(restoredJoint->getDOFCount(), 1u);
  EXPECT_EQ(restoredJoint->getParentLink().getName(), "parent");
  EXPECT_EQ(restoredJoint->getChildLink().getName(), "child");
  EXPECT_LT((restoredJoint->getAxis() - hingeAxis).norm(), 1e-12);
  EXPECT_EQ(restoredJoint->getActuatorType(), sx::ActuatorType::Velocity);
  ASSERT_EQ(restoredJoint->getCommandVelocity().size(), 1);
  EXPECT_DOUBLE_EQ(restoredJoint->getCommandVelocity()[0], targetSpeed);
  ASSERT_EQ(restoredJoint->getEffortLowerLimits().size(), 1);
  ASSERT_EQ(restoredJoint->getEffortUpperLimits().size(), 1);
  EXPECT_DOUBLE_EQ(restoredJoint->getEffortLowerLimits()[0], -1000.0);
  EXPECT_DOUBLE_EQ(restoredJoint->getEffortUpperLimits()[0], 1000.0);
  EXPECT_LT(
      (restoredParent->getWorldTransform().translation()
       - savedParentTransform.translation())
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredChild->getWorldTransform().translation()
       - savedChildTransform.translation())
          .norm(),
      1e-12);

  const auto parentAnchorWorld = [&]() {
    return restoredParent->getWorldTransform() * parentAnchor;
  };
  const auto childAnchorWorld = [&]() {
    return restoredChild->getWorldTransform() * childAnchor;
  };
  const auto anchorResidual = [&]() {
    return (childAnchorWorld() - parentAnchorWorld()).norm();
  };
  const auto axisTilt = [&]() {
    return (restoredParent->getWorldTransform().linear() * hingeAxis
            - restoredChild->getWorldTransform().linear() * hingeAxis)
        .norm();
  };
  const auto relativeHingePosition = [&]() {
    const Eigen::Matrix3d relativeRotation
        = restoredParent->getWorldTransform().linear().transpose()
          * restoredChild->getWorldTransform().linear();
    return signedRotationAroundAxis(relativeRotation, hingeAxis);
  };

  double maxBrokenAnchorResidual = 0.0;
  for (int k = 0; k < 20; ++k) {
    restoredParent->applyForce(
        -4.0 * Eigen::Vector3d::UnitY(),
        parentAnchor + 0.3 * Eigen::Vector3d::UnitX());
    restoredChild->applyForce(
        4.0 * Eigen::Vector3d::UnitY(),
        childAnchor - 0.3 * Eigen::Vector3d::UnitX());
    restored.step();
    maxBrokenAnchorResidual
        = std::max(maxBrokenAnchorResidual, anchorResidual());
  }
  ASSERT_GT(maxBrokenAnchorResidual, 1e-4);

  const double hingeBeforeReset = relativeHingePosition();
  restoredParent->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredChild->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredJoint->setCommandVelocity(Eigen::VectorXd::Constant(1, -targetSpeed));
  restoredJoint->setBreakForce(1.0e12);
  restoredJoint->resetBreakage();
  ASSERT_FALSE(restoredJoint->isBroken());

  constexpr int resetSteps = 10;
  for (int k = 0; k < resetSteps; ++k) {
    restored.step();
  }

  EXPECT_FALSE(restoredJoint->isBroken());
  EXPECT_LT(anchorResidual(), 2e-3);
  EXPECT_LT(axisTilt(), 2e-3);
  EXPECT_NEAR(
      relativeHingePosition() - hingeBeforeReset,
      -targetSpeed * dt * static_cast<double>(resetSteps),
      2e-3);
}

// PLAN-104 AVBD articulated bridge: broken public prismatic velocity motors
// between two movable same-multibody links need the same save/load/reset
// lifecycle as fixed and revolute movable-pair point joints.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedMovablePrismaticBreakageSurvivesSaveLoadAndReset)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(1.0, 2.0, 0.5).normalized();
  const Eigen::Vector3d parentAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d childAnchor(-0.1, 0.1, 0.0);
  sx::Joint joint = world.addArticulatedPrismaticJoint(
      "serialized_movable_broken_slider",
      pair.parent,
      pair.child,
      sliderAxis,
      parentAnchor,
      childAnchor);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.3;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));
  joint.setBreakForce(1e-18);

  world.enterSimulationMode();
  world.step();
  ASSERT_TRUE(joint.isBroken());

  const Eigen::Isometry3d savedParentTransform
      = pair.parent.getWorldTransform();
  const Eigen::Isometry3d savedChildTransform = pair.child.getWorldTransform();
  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot = restored.getMultibody("floating_pair");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredParent = restoredRobot->getLink("parent");
  ASSERT_TRUE(restoredParent.has_value());
  auto restoredChild = restoredRobot->getLink("child");
  ASSERT_TRUE(restoredChild.has_value());
  auto restoredJoint
      = restored.getArticulatedJoint("serialized_movable_broken_slider");
  ASSERT_TRUE(restoredJoint.has_value());
  ASSERT_TRUE(restoredJoint->isBroken());
  EXPECT_DOUBLE_EQ(restoredJoint->getBreakForce(), 1e-18);
  EXPECT_EQ(restoredJoint->getType(), sx::JointType::Prismatic);
  EXPECT_EQ(restoredJoint->getDOFCount(), 1u);
  EXPECT_EQ(restoredJoint->getParentLink().getName(), "parent");
  EXPECT_EQ(restoredJoint->getChildLink().getName(), "child");
  EXPECT_LT((restoredJoint->getAxis() - sliderAxis).norm(), 1e-12);
  EXPECT_EQ(restoredJoint->getActuatorType(), sx::ActuatorType::Velocity);
  ASSERT_EQ(restoredJoint->getCommandVelocity().size(), 1);
  EXPECT_DOUBLE_EQ(restoredJoint->getCommandVelocity()[0], targetSpeed);
  ASSERT_EQ(restoredJoint->getEffortLowerLimits().size(), 1);
  ASSERT_EQ(restoredJoint->getEffortUpperLimits().size(), 1);
  EXPECT_DOUBLE_EQ(restoredJoint->getEffortLowerLimits()[0], -1000.0);
  EXPECT_DOUBLE_EQ(restoredJoint->getEffortUpperLimits()[0], 1000.0);
  EXPECT_LT(
      (restoredParent->getWorldTransform().translation()
       - savedParentTransform.translation())
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredChild->getWorldTransform().translation()
       - savedChildTransform.translation())
          .norm(),
      1e-12);

  const auto anchorDelta = [&]() -> Eigen::Vector3d {
    const Eigen::Vector3d parentAnchorWorld
        = restoredParent->getWorldTransform() * parentAnchor;
    const Eigen::Vector3d childAnchorWorld
        = restoredChild->getWorldTransform() * childAnchor;
    return childAnchorWorld - parentAnchorWorld;
  };
  const auto sliderAxisWorld = [&]() {
    return restoredParent->getWorldTransform().linear() * sliderAxis;
  };
  const auto orthogonalAnchorResidual = [&]() {
    const Eigen::Vector3d delta = anchorDelta();
    const Eigen::Vector3d axis = sliderAxisWorld();
    return (delta - delta.dot(axis) * axis).norm();
  };
  const auto sliderPosition = [&]() {
    return anchorDelta().dot(sliderAxisWorld());
  };
  const auto relativeRotationError = [&]() {
    return (restoredParent->getWorldTransform().linear().transpose()
                * restoredChild->getWorldTransform().linear()
            - Eigen::Matrix3d::Identity())
        .norm();
  };

  double maxBrokenOrthogonalResidual = 0.0;
  for (int k = 0; k < 20; ++k) {
    restoredParent->applyForce(
        -4.0 * Eigen::Vector3d::UnitY(),
        parentAnchor + 0.3 * Eigen::Vector3d::UnitX());
    restoredChild->applyForce(
        4.0 * Eigen::Vector3d::UnitY(),
        childAnchor - 0.3 * Eigen::Vector3d::UnitX());
    restored.step();
    maxBrokenOrthogonalResidual
        = std::max(maxBrokenOrthogonalResidual, orthogonalAnchorResidual());
  }
  ASSERT_GT(maxBrokenOrthogonalResidual, 1e-4);

  const double sliderBeforeReset = sliderPosition();
  restoredParent->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredChild->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredJoint->setCommandVelocity(Eigen::VectorXd::Constant(1, -targetSpeed));
  restoredJoint->setBreakForce(1.0e12);
  restoredJoint->resetBreakage();
  ASSERT_FALSE(restoredJoint->isBroken());

  constexpr int resetSteps = 10;
  for (int k = 0; k < resetSteps; ++k) {
    restored.step();
  }

  EXPECT_FALSE(restoredJoint->isBroken());
  const double resetOrthogonalResidual = orthogonalAnchorResidual();
  EXPECT_LT(resetOrthogonalResidual, maxBrokenOrthogonalResidual * 0.05);
  EXPECT_LT(resetOrthogonalResidual, 2e-3);
  EXPECT_LT(relativeRotationError(), 1e-6);
  EXPECT_NEAR(
      sliderPosition() - sliderBeforeReset,
      -targetSpeed * dt * static_cast<double>(resetSteps),
      2e-3);
}

// PLAN-104 AVBD articulated bridge: public revolute velocity-motor rows can
// trip break-force bookkeeping, then the broken joint is skipped by later
// variational extraction instead of continuing to pin the floating link.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedRevoluteBreakForceMarksMotorRowsAndSkipsLaterSteps)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto robot = world.addMultibody("public_revolute_breakable_skip_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  const Eigen::Vector3d hingeAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  sx::Joint joint = world.addArticulatedRevoluteJoint(
      "breakable_hinge", base, body, hingeAxis);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.4;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));
  joint.setBreakForce(1e-18);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_NEAR(config.angularAxes.col(2).dot(hingeAxis), 1.0, 1e-12);

  world.step();

  ASSERT_TRUE(joint.isBroken());
  const double hingeProgress
      = signedRotationAroundAxis(body.getWorldTransform().linear(), hingeAxis);
  EXPECT_NEAR(hingeProgress, targetSpeed * dt, 1e-6);

  double maxPositionResidual = 0.0;
  for (int k = 0; k < 20; ++k) {
    body.applyForce(4.0 * Eigen::Vector3d::UnitY());
    world.step();
    maxPositionResidual = std::max(
        maxPositionResidual, body.getWorldTransform().translation().norm());
  }

  EXPECT_GT(maxPositionResidual, 1e-4);
}

// PLAN-104 AVBD articulated bridge: public revolute velocity-motor rows
// participate in break-force bookkeeping and can be re-enabled through the
// public broken-state reset path.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedRevoluteBreakForceResetReengagesMotorRows)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto robot = world.addMultibody("public_revolute_breakable_reset_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  const Eigen::Vector3d hingeAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  sx::Joint joint = world.addArticulatedRevoluteJoint(
      "breakable_hinge", base, body, hingeAxis);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.4;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));
  joint.setBreakForce(1e-18);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();
  world.step();

  ASSERT_TRUE(joint.isBroken());
  auto hingeProgress = [&]() {
    return signedRotationAroundAxis(
        body.getWorldTransform().linear(), hingeAxis);
  };
  EXPECT_NEAR(hingeProgress(), targetSpeed * dt, 1e-6);

  joint.setBreakForce(1.0e12);
  joint.resetBreakage();
  ASSERT_FALSE(joint.isBroken());

  world.step();

  const Eigen::Isometry3d transform = body.getWorldTransform();
  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_NEAR(config.angularAxes.col(2).dot(hingeAxis), 1.0, 1e-12);
  EXPECT_FALSE(joint.isBroken());
  EXPECT_LT(transform.translation().norm(), 1e-6);
  EXPECT_LT((transform.linear() * hingeAxis - hingeAxis).norm(), 1e-6);
  EXPECT_NEAR(hingeProgress(), targetSpeed * dt * 2.0, 1e-6);
}

// PLAN-104 AVBD articulated bridge: broken same-multibody public revolute
// motors must keep their broken state through simulation-mode binary save/load.
// The restored hard rows and free-axis motor stay skipped until reset, then
// re-engage from the loaded pose.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedRevoluteBreakageSurvivesSaveLoadAndReset)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto robot = world.addMultibody("public_revolute_save_load_breakage");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
  pose.head<3>() = Eigen::Vector3d(0.1, -0.2, 0.15);
  body.getParentJoint().setPosition(pose);
  const Eigen::Vector3d hingeAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  sx::Joint joint = world.addArticulatedRevoluteJoint(
      "serialized_broken_hinge",
      base,
      body,
      hingeAxis,
      body.getWorldTransform() * Eigen::Vector3d(-0.15, 0.05, 0.0),
      Eigen::Vector3d(-0.15, 0.05, 0.0));
  const Eigen::Vector3d parentAnchor(0.1 - 0.15, -0.2 + 0.05, 0.15);
  const Eigen::Vector3d childAnchor(-0.15, 0.05, 0.0);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.4;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));
  joint.setBreakForce(1e-18);

  world.enterSimulationMode();
  world.step();
  ASSERT_TRUE(joint.isBroken());

  const Eigen::Isometry3d savedBrokenTransform = body.getWorldTransform();
  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot
      = restored.getMultibody("public_revolute_save_load_breakage");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredBody = restoredRobot->getLink("body");
  ASSERT_TRUE(restoredBody.has_value());
  auto restoredJoint = restored.getArticulatedJoint("serialized_broken_hinge");
  ASSERT_TRUE(restoredJoint.has_value());
  ASSERT_TRUE(restoredJoint->isBroken());
  EXPECT_EQ(restoredJoint->getType(), sx::JointType::Revolute);
  EXPECT_EQ(restoredJoint->getDOFCount(), 1u);
  EXPECT_EQ(restoredJoint->getParentLink().getName(), "base");
  EXPECT_EQ(restoredJoint->getChildLink().getName(), "body");
  EXPECT_LT((restoredJoint->getAxis() - hingeAxis).norm(), 1e-12);
  EXPECT_DOUBLE_EQ(restoredJoint->getBreakForce(), 1e-18);
  EXPECT_EQ(restoredJoint->getActuatorType(), sx::ActuatorType::Velocity);
  ASSERT_EQ(restoredJoint->getCommandVelocity().size(), 1);
  EXPECT_DOUBLE_EQ(restoredJoint->getCommandVelocity()[0], targetSpeed);
  ASSERT_EQ(restoredJoint->getEffortLowerLimits().size(), 1);
  ASSERT_EQ(restoredJoint->getEffortUpperLimits().size(), 1);
  EXPECT_DOUBLE_EQ(restoredJoint->getEffortLowerLimits()[0], -1000.0);
  EXPECT_DOUBLE_EQ(restoredJoint->getEffortUpperLimits()[0], 1000.0);
  EXPECT_LT(
      (restoredBody->getWorldTransform().translation()
       - savedBrokenTransform.translation())
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredBody->getWorldTransform().linear()
       - savedBrokenTransform.linear())
          .norm(),
      1e-12);

  auto& registry = dart::simulation::detail::registryOf(restored);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(restoredJoint->getEntity());

  double maxBrokenAnchorResidual = 0.0;
  for (int k = 0; k < 20; ++k) {
    restoredBody->applyForce(4.0 * Eigen::Vector3d::UnitY());
    restored.step();
    maxBrokenAnchorResidual = std::max(
        maxBrokenAnchorResidual,
        (restoredBody->getWorldTransform() * childAnchor - parentAnchor)
            .norm());
  }
  ASSERT_GT(maxBrokenAnchorResidual, 1e-4);

  const Eigen::Matrix3d rotationBeforeReset
      = restoredBody->getWorldTransform().linear();
  restoredBody->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredJoint->setBreakForce(1.0e12);
  restoredJoint->resetBreakage();
  ASSERT_FALSE(restoredJoint->isBroken());

  restored.step();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_LT((config.localAnchorA - parentAnchor).norm(), 1e-12);
  EXPECT_LT((config.localAnchorB - childAnchor).norm(), 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_NEAR(config.angularAxes.col(2).dot(hingeAxis), 1.0, 1e-12);

  const Eigen::Isometry3d resetTransform = restoredBody->getWorldTransform();
  const Eigen::AngleAxisd incrementalRotation(
      resetTransform.linear() * rotationBeforeReset.transpose());
  const double signedHingeIncrement
      = incrementalRotation.angle() * incrementalRotation.axis().dot(hingeAxis);
  EXPECT_FALSE(restoredJoint->isBroken());
  EXPECT_LT((resetTransform * childAnchor - parentAnchor).norm(), 1e-6);
  EXPECT_LT((resetTransform.linear() * hingeAxis - hingeAxis).norm(), 1e-6);
  EXPECT_NEAR(signedHingeIncrement, targetSpeed * dt, 5e-4);
}

// PLAN-104 AVBD articulated bridge: public prismatic velocity-motor rows also
// mark their source joint broken and stop contributing masked hard rows on
// later steps, allowing force-driven drift outside the slider axis.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedPrismaticBreakForceMarksMotorRowsAndSkipsLaterSteps)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto robot = world.addMultibody("public_prismatic_breakable_skip_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(1.0, 2.0, 0.5).normalized();
  const Eigen::Vector3d lateralForce
      = (Eigen::Vector3d::UnitY()
         - Eigen::Vector3d::UnitY().dot(sliderAxis) * sliderAxis)
            .normalized();
  sx::Joint joint = world.addArticulatedPrismaticJoint(
      "breakable_slider", base, body, sliderAxis);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.3;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));
  joint.setBreakForce(1e-18);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(config.linearAxes.col(2).dot(sliderAxis), 1.0, 1e-12);

  world.step();

  ASSERT_TRUE(joint.isBroken());
  EXPECT_NEAR(
      body.getWorldTransform().translation().dot(sliderAxis),
      targetSpeed * dt,
      1e-6);

  double maxOrthogonalDrift = 0.0;
  for (int k = 0; k < 20; ++k) {
    body.applyForce(4.0 * lateralForce);
    world.step();
    const Eigen::Vector3d position = body.getWorldTransform().translation();
    maxOrthogonalDrift = std::max(
        maxOrthogonalDrift,
        (position - position.dot(sliderAxis) * sliderAxis).norm());
  }

  EXPECT_GT(maxOrthogonalDrift, 1e-4);
}

// PLAN-104 AVBD articulated bridge: public prismatic velocity-motor rows also
// break and re-engage through the public lifecycle while preserving the masked
// hard rows around the free slider axis.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedPrismaticBreakForceResetReengagesMotorRows)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto robot = world.addMultibody("public_prismatic_breakable_reset_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(1.0, 2.0, 0.5).normalized();
  sx::Joint joint = world.addArticulatedPrismaticJoint(
      "breakable_slider", base, body, sliderAxis);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.3;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));
  joint.setBreakForce(1e-18);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();
  world.step();

  ASSERT_TRUE(joint.isBroken());
  EXPECT_NEAR(
      body.getWorldTransform().translation().dot(sliderAxis),
      targetSpeed * dt,
      1e-6);

  joint.setBreakForce(1.0e12);
  joint.resetBreakage();
  ASSERT_FALSE(joint.isBroken());

  world.step();

  const Eigen::Isometry3d transform = body.getWorldTransform();
  const Eigen::Vector3d position = transform.translation();
  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(config.linearAxes.col(2).dot(sliderAxis), 1.0, 1e-12);
  EXPECT_FALSE(joint.isBroken());
  EXPECT_LT((position - position.dot(sliderAxis) * sliderAxis).norm(), 1e-6);
  EXPECT_LT((transform.linear() - Eigen::Matrix3d::Identity()).norm(), 1e-6);
  EXPECT_NEAR(position.dot(sliderAxis), targetSpeed * dt * 2.0, 1e-6);
}

// PLAN-104 AVBD articulated bridge: broken same-multibody public prismatic
// motors must keep their broken state through simulation-mode binary save/load.
// The restored masked hard rows and free-axis motor stay skipped until reset.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedPrismaticBreakageSurvivesSaveLoadAndReset)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto robot = world.addMultibody("public_prismatic_save_load_breakage");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
  pose.head<3>() = Eigen::Vector3d(0.05, -0.1, 0.0);
  body.getParentJoint().setPosition(pose);
  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(1.0, 2.0, 0.5).normalized();
  const Eigen::Vector3d childAnchor(0.0, 0.2, 0.1);
  const Eigen::Vector3d parentAnchor = body.getWorldTransform() * childAnchor;
  sx::Joint joint = world.addArticulatedPrismaticJoint(
      "serialized_broken_slider",
      base,
      body,
      sliderAxis,
      parentAnchor,
      childAnchor);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.3;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));
  joint.setBreakForce(1e-18);

  world.enterSimulationMode();
  world.step();
  ASSERT_TRUE(joint.isBroken());

  const Eigen::Isometry3d savedBrokenTransform = body.getWorldTransform();
  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot
      = restored.getMultibody("public_prismatic_save_load_breakage");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredBody = restoredRobot->getLink("body");
  ASSERT_TRUE(restoredBody.has_value());
  auto restoredJoint = restored.getArticulatedJoint("serialized_broken_slider");
  ASSERT_TRUE(restoredJoint.has_value());
  ASSERT_TRUE(restoredJoint->isBroken());
  EXPECT_EQ(restoredJoint->getType(), sx::JointType::Prismatic);
  EXPECT_EQ(restoredJoint->getDOFCount(), 1u);
  EXPECT_EQ(restoredJoint->getParentLink().getName(), "base");
  EXPECT_EQ(restoredJoint->getChildLink().getName(), "body");
  EXPECT_LT((restoredJoint->getAxis() - sliderAxis).norm(), 1e-12);
  EXPECT_DOUBLE_EQ(restoredJoint->getBreakForce(), 1e-18);
  EXPECT_EQ(restoredJoint->getActuatorType(), sx::ActuatorType::Velocity);
  ASSERT_EQ(restoredJoint->getCommandVelocity().size(), 1);
  EXPECT_DOUBLE_EQ(restoredJoint->getCommandVelocity()[0], targetSpeed);
  ASSERT_EQ(restoredJoint->getEffortLowerLimits().size(), 1);
  ASSERT_EQ(restoredJoint->getEffortUpperLimits().size(), 1);
  EXPECT_DOUBLE_EQ(restoredJoint->getEffortLowerLimits()[0], -1000.0);
  EXPECT_DOUBLE_EQ(restoredJoint->getEffortUpperLimits()[0], 1000.0);
  EXPECT_LT(
      (restoredBody->getWorldTransform().translation()
       - savedBrokenTransform.translation())
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredBody->getWorldTransform().linear()
       - savedBrokenTransform.linear())
          .norm(),
      1e-12);

  auto& registry = dart::simulation::detail::registryOf(restored);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(restoredJoint->getEntity());

  double maxOrthogonalDrift = 0.0;
  for (int k = 0; k < 20; ++k) {
    restoredBody->applyForce(4.0 * Eigen::Vector3d::UnitY());
    restored.step();
    const Eigen::Vector3d anchorDelta
        = restoredBody->getWorldTransform() * childAnchor - parentAnchor;
    maxOrthogonalDrift = std::max(
        maxOrthogonalDrift,
        (anchorDelta - anchorDelta.dot(sliderAxis) * sliderAxis).norm());
  }
  ASSERT_GT(maxOrthogonalDrift, 1e-4);

  const double sliderPositionBeforeReset
      = (restoredBody->getWorldTransform() * childAnchor - parentAnchor)
            .dot(sliderAxis);
  restoredBody->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredJoint->setBreakForce(1.0e12);
  restoredJoint->resetBreakage();
  ASSERT_FALSE(restoredJoint->isBroken());

  restored.step();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_LT((config.localAnchorA - parentAnchor).norm(), 1e-12);
  EXPECT_LT((config.localAnchorB - childAnchor).norm(), 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(config.linearAxes.col(2).dot(sliderAxis), 1.0, 1e-12);

  const Eigen::Isometry3d resetTransform = restoredBody->getWorldTransform();
  const Eigen::Vector3d resetPosition
      = resetTransform * childAnchor - parentAnchor;
  EXPECT_FALSE(restoredJoint->isBroken());
  EXPECT_LT(
      (resetPosition - resetPosition.dot(sliderAxis) * sliderAxis).norm(),
      1e-6);
  EXPECT_LT(
      (resetTransform.linear() - Eigen::Matrix3d::Identity()).norm(), 1e-6);
  EXPECT_NEAR(
      resetPosition.dot(sliderAxis),
      sliderPositionBeforeReset + targetSpeed * dt,
      1e-6);
}

// PLAN-104 AVBD articulated bridge: public articulated fixed joints can now use
// the world frame as one endpoint, expanding the facade beyond same-multibody
// link-link closures while still using the simulation-entry current-pose
// extractor.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedWorldFixedJointFacadeWiresFloatingLinkEndpoint)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto robot = world.addMultibody("public_world_fixed_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  Eigen::VectorXd pose(6);
  pose << 0.3, -0.2, 0.1, 0.0, 0.0, 0.25;
  body.getParentJoint().setPosition(pose);

  sx::Joint joint = world.addArticulatedFixedJoint("world_hold", body);
  EXPECT_EQ(joint.getType(), sx::JointType::Fixed);
  EXPECT_EQ(joint.getChildLink().getName(), "body");
  EXPECT_THROW({ (void)joint.getParentLink(); }, sx::InvalidArgumentException);
  EXPECT_TRUE(world.hasArticulatedJoint("world_hold"));
  EXPECT_EQ(world.getArticulatedJointCount(), 1u);

  world.enterSimulationMode();

  const Eigen::Isometry3d captured = body.getWorldTransform();
  double maxTranslationError = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < 60; ++k) {
    body.applyForce(
        3.0 * Eigen::Vector3d::UnitY(), 0.4 * Eigen::Vector3d::UnitX());
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxTranslationError = std::max(
        maxTranslationError,
        (transform.translation() - captured.translation()).norm());
    maxRotationError = std::max(
        maxRotationError, (transform.linear() - captured.linear()).norm());
  }

  EXPECT_LT(maxTranslationError, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
}

// PLAN-104 AVBD articulated bridge: world-link fixed point joints also preserve
// explicit off-origin anchors, holding the selected child-local point and the
// captured orientation rather than only the link origin.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedWorldFixedJointFacadeDrivesOffsetAnchor)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto robot = world.addMultibody("public_world_fixed_offset_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  Eigen::VectorXd pose(6);
  pose << 0.3, -0.2, 0.1, 0.0, 0.0, 0.25;
  body.getParentJoint().setPosition(pose);

  const Eigen::Vector3d childAnchor(0.2, 0.1, 0.0);
  const Eigen::Isometry3d captured = body.getWorldTransform();
  const Eigen::Vector3d worldAnchor = captured * childAnchor;
  sx::Joint joint = world.addArticulatedFixedJoint(
      "world_offset_hold", body, worldAnchor, childAnchor);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_LT((config.localAnchorA - worldAnchor).norm(), 1e-12);
  EXPECT_LT((config.localAnchorB - childAnchor).norm(), 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);

  double maxAnchorResidual = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < 60; ++k) {
    body.applyForce(
        3.0 * Eigen::Vector3d::UnitY(), 0.4 * Eigen::Vector3d::UnitX());
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxAnchorResidual = std::max(
        maxAnchorResidual, (transform * childAnchor - worldAnchor).norm());
    maxRotationError = std::max(
        maxRotationError, (transform.linear() - captured.linear()).norm());
  }

  EXPECT_LT(maxAnchorResidual, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
}

// PLAN-104 AVBD articulated bridge: world-link public fixed point joints must
// preserve explicit world/link anchors and break-force metadata across
// design-mode binary save/load before simulation entry rebuilds all-axis fixed
// AVBD rows.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedWorldFixedJointFacadeSurvivesSaveLoad)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto robot = world.addMultibody("serialized_world_fixed_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  Eigen::VectorXd pose(6);
  pose << 0.3, -0.2, 0.1, 0.0, 0.0, 0.25;
  body.getParentJoint().setPosition(pose);
  const Eigen::Vector3d childAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d worldAnchor = body.getWorldTransform() * childAnchor;
  sx::Joint joint = world.addArticulatedFixedJoint(
      "serialized_world_hold", body, worldAnchor, childAnchor);
  joint.setBreakForce(19.0);

  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot = restored.getMultibody("serialized_world_fixed_facade");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredBody = restoredRobot->getLink("body");
  ASSERT_TRUE(restoredBody.has_value());
  auto restoredJoint = restored.getArticulatedJoint("serialized_world_hold");
  ASSERT_TRUE(restoredJoint.has_value());
  EXPECT_EQ(restored.getArticulatedJointCount(), 1u);
  EXPECT_EQ(restoredJoint->getType(), sx::JointType::Fixed);
  EXPECT_EQ(restoredJoint->getDOFCount(), 0u);
  EXPECT_EQ(restoredJoint->getChildLink().getName(), "body");
  EXPECT_THROW(
      { (void)restoredJoint->getParentLink(); }, sx::InvalidArgumentException);
  EXPECT_DOUBLE_EQ(restoredJoint->getBreakForce(), 19.0);
  EXPECT_FALSE(restoredJoint->isBroken());

  auto& registry = dart::simulation::detail::registryOf(restored);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(restoredJoint->getEntity());
  EXPECT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  restored.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_LT((config.localAnchorA - worldAnchor).norm(), 1e-12);
  EXPECT_LT((config.localAnchorB - childAnchor).norm(), 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_TRUE(std::isinf(config.startStiffness));
  EXPECT_TRUE(std::isinf(config.maxStiffness));

  const Eigen::Matrix3d capturedRotation
      = restoredBody->getWorldTransform().linear();
  double maxAnchorResidual = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < 40; ++k) {
    restoredBody->applyForce(
        0.3 * Eigen::Vector3d::UnitY(),
        childAnchor + 0.2 * Eigen::Vector3d::UnitX());
    restored.step();

    const Eigen::Isometry3d transform = restoredBody->getWorldTransform();
    maxAnchorResidual = std::max(
        maxAnchorResidual, (transform * childAnchor - worldAnchor).norm());
    maxRotationError = std::max(
        maxRotationError, (transform.linear() - capturedRotation).norm());
  }

  EXPECT_LT(maxAnchorResidual, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
  EXPECT_FALSE(restoredJoint->isBroken());
}

// PLAN-104 AVBD articulated bridge: the world-link fixed facade also carries
// the public break/reset lifecycle. Resetting a broken world-anchored point
// joint reuses the simulation-entry captured pose instead of leaving the link
// permanently detached from the world endpoint.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedWorldFixedBreakForceResetReengagesJoint)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto robot = world.addMultibody("public_world_fixed_breakable_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  Eigen::VectorXd pose(6);
  pose << 0.3, -0.2, 0.1, 0.0, 0.0, 0.25;
  body.getParentJoint().setPosition(pose);

  sx::Joint joint
      = world.addArticulatedFixedJoint("world_breakable_hold", body);
  joint.setBreakForce(1e-18);

  world.enterSimulationMode();
  const Eigen::Isometry3d captured = body.getWorldTransform();

  body.applyForce(
      4.0 * Eigen::Vector3d::UnitY(), 0.4 * Eigen::Vector3d::UnitX());
  world.step();
  ASSERT_TRUE(joint.isBroken());

  double maxBrokenTranslationError = 0.0;
  double maxBrokenRotationError = 0.0;
  for (int k = 0; k < 20; ++k) {
    body.applyForce(
        4.0 * Eigen::Vector3d::UnitY(), 0.4 * Eigen::Vector3d::UnitX());
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxBrokenTranslationError = std::max(
        maxBrokenTranslationError,
        (transform.translation() - captured.translation()).norm());
    maxBrokenRotationError = std::max(
        maxBrokenRotationError,
        (transform.linear() - captured.linear()).norm());
  }
  ASSERT_GT(maxBrokenTranslationError, 1e-4);
  ASSERT_GT(maxBrokenRotationError, 1e-4);

  body.getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  joint.setBreakForce(1.0e12);
  joint.resetBreakage();
  ASSERT_FALSE(joint.isBroken());

  world.step();

  const Eigen::Isometry3d resetTransform = body.getWorldTransform();
  EXPECT_FALSE(joint.isBroken());
  EXPECT_LT(
      (resetTransform.translation() - captured.translation()).norm(), 1e-6);
  EXPECT_LT((resetTransform.linear() - captured.linear()).norm(), 1e-6);
}

// PLAN-104 AVBD articulated bridge: broken world-link public fixed joints also
// persist through simulation-mode binary save/load. The restored all-axis rows
// remain skipped while the joint is broken, then reset re-enables their
// explicit world/link anchors against the loaded pose.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedWorldFixedBreakageSurvivesSaveLoadAndReset)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto robot = world.addMultibody("public_world_fixed_save_load_breakage");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  Eigen::VectorXd pose(6);
  pose << 0.3, -0.2, 0.1, 0.0, 0.0, 0.25;
  body.getParentJoint().setPosition(pose);

  const Eigen::Vector3d childAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d worldAnchor = body.getWorldTransform() * childAnchor;
  sx::Joint joint = world.addArticulatedFixedJoint(
      "serialized_world_broken_hold", body, worldAnchor, childAnchor);
  joint.setBreakForce(1e-18);

  world.enterSimulationMode();

  body.applyForce(
      4.0 * Eigen::Vector3d::UnitY(),
      childAnchor + 0.4 * Eigen::Vector3d::UnitX());
  world.step();
  ASSERT_TRUE(joint.isBroken());

  const Eigen::Isometry3d savedBrokenTransform = body.getWorldTransform();
  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot
      = restored.getMultibody("public_world_fixed_save_load_breakage");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredBody = restoredRobot->getLink("body");
  ASSERT_TRUE(restoredBody.has_value());
  auto restoredJoint
      = restored.getArticulatedJoint("serialized_world_broken_hold");
  ASSERT_TRUE(restoredJoint.has_value());
  ASSERT_TRUE(restoredJoint->isBroken());
  EXPECT_DOUBLE_EQ(restoredJoint->getBreakForce(), 1e-18);
  EXPECT_EQ(restoredJoint->getType(), sx::JointType::Fixed);
  EXPECT_EQ(restoredJoint->getDOFCount(), 0u);
  EXPECT_EQ(restoredJoint->getChildLink().getName(), "body");
  EXPECT_THROW(
      { (void)restoredJoint->getParentLink(); }, sx::InvalidArgumentException);
  EXPECT_LT(
      (restoredBody->getWorldTransform().translation()
       - savedBrokenTransform.translation())
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredBody->getWorldTransform().linear()
       - savedBrokenTransform.linear())
          .norm(),
      1e-12);

  const Eigen::Isometry3d restoredCaptured = restoredBody->getWorldTransform();
  double maxBrokenTranslationError = 0.0;
  double maxBrokenRotationError = 0.0;
  double maxBrokenAnchorResidual = 0.0;
  for (int k = 0; k < 20; ++k) {
    restoredBody->applyForce(
        4.0 * Eigen::Vector3d::UnitY(),
        childAnchor + 0.4 * Eigen::Vector3d::UnitX());
    restored.step();

    const Eigen::Isometry3d transform = restoredBody->getWorldTransform();
    maxBrokenTranslationError = std::max(
        maxBrokenTranslationError,
        (transform.translation() - restoredCaptured.translation()).norm());
    maxBrokenRotationError = std::max(
        maxBrokenRotationError,
        (transform.linear() - restoredCaptured.linear()).norm());
    maxBrokenAnchorResidual = std::max(
        maxBrokenAnchorResidual,
        (transform * childAnchor - worldAnchor).norm());
  }
  ASSERT_GT(maxBrokenTranslationError, 1e-4);
  ASSERT_GT(maxBrokenRotationError, 1e-4);
  ASSERT_GT(maxBrokenAnchorResidual, 1e-4);

  restoredBody->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredJoint->setBreakForce(1.0e12);
  restoredJoint->resetBreakage();
  ASSERT_FALSE(restoredJoint->isBroken());

  restored.step();

  const Eigen::Isometry3d resetTransform = restoredBody->getWorldTransform();
  EXPECT_FALSE(restoredJoint->isBroken());
  EXPECT_LT((resetTransform * childAnchor - worldAnchor).norm(), 1e-6);
  EXPECT_LT((resetTransform.linear() - restoredCaptured.linear()).norm(), 1e-6);
}

// PLAN-104 AVBD articulated bridge: world-anchored revolute facades preserve
// the captured world anchor and hinge axis while the velocity actuator drives
// the free angular coordinate.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedWorldRevoluteJointFacadeDrivesFloatingLinkEndpoint)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto robot = world.addMultibody("public_world_revolute_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  const Eigen::Vector3d hingeAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  sx::Joint joint
      = world.addArticulatedRevoluteJoint("world_hinge", body, hingeAxis);
  ASSERT_EQ(joint.getType(), sx::JointType::Revolute);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.4;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_NEAR(config.angularAxes.col(2).dot(hingeAxis), 1.0, 1e-12);

  constexpr int steps = 20;
  double maxPositionResidual = 0.0;
  double maxHingeAxisTilt = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxPositionResidual
        = std::max(maxPositionResidual, transform.translation().norm());
    maxHingeAxisTilt = std::max(
        maxHingeAxisTilt, (transform.linear() * hingeAxis - hingeAxis).norm());
  }

  const Eigen::Matrix3d expectedRotation
      = Eigen::AngleAxisd(targetSpeed * dt * steps, hingeAxis)
            .toRotationMatrix();
  EXPECT_LT(maxPositionResidual, 1e-6);
  EXPECT_LT(maxHingeAxisTilt, 1e-6);
  EXPECT_LT(
      (body.getWorldTransform().linear() - expectedRotation).norm(), 1e-6);
}

// PLAN-104 AVBD articulated bridge: world-anchored revolute facades also accept
// explicit off-origin world/link anchors. The variational projection keeps the
// link-local anchor on the world anchor while the free hinge axis motor rotates
// the body around that point.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedWorldRevoluteJointFacadeDrivesOffsetAnchor)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto robot = world.addMultibody("public_world_revolute_offset_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
  pose.head<3>() = Eigen::Vector3d(0.3, -0.2, 0.1);
  body.getParentJoint().setPosition(pose);
  const Eigen::Vector3d childAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d worldAnchor = body.getWorldTransform() * childAnchor;

  const Eigen::Vector3d hingeAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  sx::Joint joint = world.addArticulatedRevoluteJoint(
      "world_offset_hinge", body, hingeAxis, worldAnchor, childAnchor);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.4;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_LT((config.localAnchorA - worldAnchor).norm(), 1e-12);
  EXPECT_LT((config.localAnchorB - childAnchor).norm(), 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_NEAR(config.angularAxes.col(2).dot(hingeAxis), 1.0, 1e-12);

  constexpr int steps = 20;
  double maxAnchorResidual = 0.0;
  double maxHingeAxisTilt = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxAnchorResidual = std::max(
        maxAnchorResidual, (transform * childAnchor - worldAnchor).norm());
    maxHingeAxisTilt = std::max(
        maxHingeAxisTilt, (transform.linear() * hingeAxis - hingeAxis).norm());
  }

  const Eigen::Matrix3d expectedRotation
      = Eigen::AngleAxisd(targetSpeed * dt * steps, hingeAxis)
            .toRotationMatrix();
  EXPECT_LT(maxAnchorResidual, 1e-6);
  EXPECT_LT(maxHingeAxisTilt, 1e-6);
  EXPECT_LT(
      (body.getWorldTransform().linear() - expectedRotation).norm(), 1e-6);
}

// PLAN-104 AVBD articulated bridge: world-link public revolute motor facades
// must also preserve explicit anchors, velocity actuator state, effort limits,
// and break-force metadata through design-mode binary save/load. Simulation
// entry then rebuilds the private hard rows plus the free-axis motor row.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedWorldRevoluteJointFacadeSurvivesSaveLoad)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto robot = world.addMultibody("serialized_world_revolute_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
  pose.head<3>() = Eigen::Vector3d(0.3, -0.2, 0.1);
  body.getParentJoint().setPosition(pose);
  const Eigen::Vector3d hingeAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  const Eigen::Vector3d childAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d worldAnchor = body.getWorldTransform() * childAnchor;

  sx::Joint joint = world.addArticulatedRevoluteJoint(
      "serialized_world_hinge", body, hingeAxis, worldAnchor, childAnchor);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.4;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -850.0),
      Eigen::VectorXd::Constant(1, 950.0));
  joint.setBreakForce(13.0);

  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot
      = restored.getMultibody("serialized_world_revolute_facade");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredBody = restoredRobot->getLink("body");
  ASSERT_TRUE(restoredBody.has_value());
  auto restoredJoint = restored.getArticulatedJoint("serialized_world_hinge");
  ASSERT_TRUE(restoredJoint.has_value());
  EXPECT_EQ(restored.getArticulatedJointCount(), 1u);
  EXPECT_EQ(restoredJoint->getType(), sx::JointType::Revolute);
  EXPECT_EQ(restoredJoint->getDOFCount(), 1u);
  EXPECT_EQ(restoredJoint->getChildLink().getName(), "body");
  EXPECT_THROW(
      { (void)restoredJoint->getParentLink(); }, sx::InvalidArgumentException);
  EXPECT_LT((restoredJoint->getAxis() - hingeAxis).norm(), 1e-12);
  EXPECT_EQ(restoredJoint->getActuatorType(), sx::ActuatorType::Velocity);
  ASSERT_EQ(restoredJoint->getCommandVelocity().size(), 1);
  EXPECT_DOUBLE_EQ(restoredJoint->getCommandVelocity()[0], targetSpeed);
  ASSERT_EQ(restoredJoint->getEffortLowerLimits().size(), 1);
  ASSERT_EQ(restoredJoint->getEffortUpperLimits().size(), 1);
  EXPECT_DOUBLE_EQ(restoredJoint->getEffortLowerLimits()[0], -850.0);
  EXPECT_DOUBLE_EQ(restoredJoint->getEffortUpperLimits()[0], 950.0);
  EXPECT_DOUBLE_EQ(restoredJoint->getBreakForce(), 13.0);
  EXPECT_FALSE(restoredJoint->isBroken());

  auto& registry = dart::simulation::detail::registryOf(restored);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(restoredJoint->getEntity());
  EXPECT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  restored.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_LT((config.localAnchorA - worldAnchor).norm(), 1e-12);
  EXPECT_LT((config.localAnchorB - childAnchor).norm(), 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_NEAR(config.angularAxes.col(2).dot(hingeAxis), 1.0, 1e-12);
  EXPECT_TRUE(std::isinf(config.startStiffness));
  EXPECT_TRUE(std::isinf(config.maxStiffness));

  constexpr int steps = 20;
  double maxAnchorResidual = 0.0;
  double maxHingeAxisTilt = 0.0;
  for (int k = 0; k < steps; ++k) {
    restored.step();

    const Eigen::Isometry3d transform = restoredBody->getWorldTransform();
    maxAnchorResidual = std::max(
        maxAnchorResidual, (transform * childAnchor - worldAnchor).norm());
    maxHingeAxisTilt = std::max(
        maxHingeAxisTilt, (transform.linear() * hingeAxis - hingeAxis).norm());
  }

  const Eigen::Matrix3d rotation = restoredBody->getWorldTransform().linear();
  const Eigen::Matrix3d expectedRotation
      = Eigen::AngleAxisd(targetSpeed * dt * steps, hingeAxis)
            .toRotationMatrix();
  EXPECT_LT(maxAnchorResidual, 1e-6);
  EXPECT_LT(maxHingeAxisTilt, 1e-6);
  EXPECT_LT((rotation - expectedRotation).norm(), 1e-6);
}

// PLAN-104 AVBD articulated bridge: world-anchored revolute facades expose the
// same finite effort bound as same-multibody facades, so a tiny positive torque
// limit keeps the world anchor and hinge axis hard while preventing the
// velocity actuator from acting like an unbounded motor.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedWorldRevoluteJointFacadeRespectsTinyTorqueLimit)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto robot = world.addMultibody("public_world_revolute_tiny_limit_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  const Eigen::Vector3d hingeAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  const Eigen::Vector3d childAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d worldAnchor = body.getWorldTransform() * childAnchor;
  sx::Joint joint = world.addArticulatedRevoluteJoint(
      "world_hinge", body, hingeAxis, worldAnchor, childAnchor);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.4;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1e-9), Eigen::VectorXd::Constant(1, 1e-9));

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_LT((config.localAnchorA - worldAnchor).norm(), 1e-12);
  EXPECT_LT((config.localAnchorB - childAnchor).norm(), 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_NEAR(config.angularAxes.col(2).dot(hingeAxis), 1.0, 1e-12);

  constexpr int steps = 20;
  double maxAnchorResidual = 0.0;
  double maxHingeAxisTilt = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxAnchorResidual = std::max(
        maxAnchorResidual, (transform * childAnchor - worldAnchor).norm());
    maxHingeAxisTilt = std::max(
        maxHingeAxisTilt, (transform.linear() * hingeAxis - hingeAxis).norm());
  }

  const Eigen::Matrix3d rotation = body.getWorldTransform().linear();
  const double hingeMotion = signedRotationAroundAxis(rotation, hingeAxis);
  EXPECT_LT(maxAnchorResidual, 1e-6);
  EXPECT_LT(maxHingeAxisTilt, 1e-3);
  EXPECT_LT(
      std::abs(hingeMotion), 0.01 * targetSpeed * world.getTimeStep() * steps);
}

// PLAN-104 AVBD articulated bridge: world-anchored public revolute facades
// also read velocity commands from the source joint each step, so command
// updates rebuild the world-anchor free-axis motor target.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedWorldRevoluteJointFacadeFollowsCommandUpdates)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto robot = world.addMultibody("public_world_revolute_command_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  sx::Joint joint = world.addArticulatedRevoluteJoint(
      "world_hinge", body, Eigen::Vector3d::UnitZ());
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.4;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));

  world.enterSimulationMode();

  double maxPositionResidual = 0.0;
  double maxHingeAxisTilt = 0.0;
  auto stepAndYaw = [&]() {
    world.step();
    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxPositionResidual
        = std::max(maxPositionResidual, transform.translation().norm());
    maxHingeAxisTilt = std::max(
        maxHingeAxisTilt,
        (transform.linear() * Eigen::Vector3d::UnitZ()
         - Eigen::Vector3d::UnitZ())
            .norm());
    const Eigen::Matrix3d rotation = transform.linear();
    return std::atan2(rotation(1, 0), rotation(0, 0));
  };

  constexpr int forwardSteps = 10;
  double yawAfterForward = 0.0;
  for (int k = 0; k < forwardSteps; ++k) {
    yawAfterForward = stepAndYaw();
  }

  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, -targetSpeed));

  constexpr int reverseSteps = 10;
  double yawAfterReverse = yawAfterForward;
  for (int k = 0; k < reverseSteps; ++k) {
    yawAfterReverse = stepAndYaw();
  }

  EXPECT_LT(maxPositionResidual, 1e-6);
  EXPECT_LT(maxHingeAxisTilt, 1e-6);
  EXPECT_NEAR(yawAfterForward, targetSpeed * dt * forwardSteps, 1e-6);
  EXPECT_NEAR(
      yawAfterReverse, targetSpeed * dt * (forwardSteps - reverseSteps), 1e-6);
}

// PLAN-104 AVBD articulated bridge: world-anchored public revolute motor rows
// can also trip break-force bookkeeping, after which later extraction skips the
// broken joint and no longer pins the floating link to the world anchor.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedWorldRevoluteBreakForceMarksMotorRowsAndSkipsLaterSteps)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto robot = world.addMultibody("public_world_revolute_breakable_skip");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  const Eigen::Vector3d hingeAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  sx::Joint joint = world.addArticulatedRevoluteJoint(
      "world_breakable_hinge", body, hingeAxis);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.4;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));
  joint.setBreakForce(1e-18);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_NEAR(config.angularAxes.col(2).dot(hingeAxis), 1.0, 1e-12);

  world.step();

  ASSERT_TRUE(joint.isBroken());
  const double hingeProgress
      = signedRotationAroundAxis(body.getWorldTransform().linear(), hingeAxis);
  EXPECT_NEAR(hingeProgress, targetSpeed * dt, 1e-6);

  double maxPositionResidual = 0.0;
  for (int k = 0; k < 20; ++k) {
    body.applyForce(4.0 * Eigen::Vector3d::UnitY());
    world.step();
    maxPositionResidual = std::max(
        maxPositionResidual, body.getWorldTransform().translation().norm());
  }

  EXPECT_GT(maxPositionResidual, 1e-4);
}

// PLAN-104 AVBD articulated bridge: world-anchored public revolute motor rows
// also participate in the public break/reset lifecycle, then re-enter the
// variational projection after reset.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedWorldRevoluteBreakForceResetReengagesMotorRows)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto robot = world.addMultibody("public_world_revolute_breakable_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  const Eigen::Vector3d hingeAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  sx::Joint joint = world.addArticulatedRevoluteJoint(
      "world_breakable_hinge", body, hingeAxis);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.4;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));
  joint.setBreakForce(1e-18);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();
  world.step();

  ASSERT_TRUE(joint.isBroken());
  auto hingeProgress = [&]() {
    return signedRotationAroundAxis(
        body.getWorldTransform().linear(), hingeAxis);
  };
  EXPECT_NEAR(hingeProgress(), targetSpeed * dt, 1e-6);

  joint.setBreakForce(1.0e12);
  joint.resetBreakage();
  ASSERT_FALSE(joint.isBroken());

  world.step();

  const Eigen::Isometry3d transform = body.getWorldTransform();
  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_NEAR(config.angularAxes.col(2).dot(hingeAxis), 1.0, 1e-12);
  EXPECT_FALSE(joint.isBroken());
  EXPECT_LT(transform.translation().norm(), 1e-6);
  EXPECT_LT((transform.linear() * hingeAxis - hingeAxis).norm(), 1e-6);
  EXPECT_NEAR(hingeProgress(), targetSpeed * dt * 2.0, 1e-6);
}

// PLAN-104 AVBD articulated bridge: a broken world-anchored public revolute
// motor must persist through simulation-mode binary save/load as skipped hard
// rows plus a restorable free-axis motor row.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedWorldRevoluteBreakageSurvivesSaveLoadAndReset)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto robot = world.addMultibody("public_world_revolute_save_load_breakage");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
  pose.head<3>() = Eigen::Vector3d(0.15, -0.05, 0.0);
  body.getParentJoint().setPosition(pose);
  const Eigen::Vector3d hingeAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  const Eigen::Vector3d childAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d worldAnchor = body.getWorldTransform() * childAnchor;
  sx::Joint joint = world.addArticulatedRevoluteJoint(
      "serialized_world_broken_hinge",
      body,
      hingeAxis,
      worldAnchor,
      childAnchor);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.4;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));
  joint.setBreakForce(1e-18);

  world.enterSimulationMode();
  world.step();
  ASSERT_TRUE(joint.isBroken());

  const Eigen::Isometry3d savedBrokenTransform = body.getWorldTransform();
  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot
      = restored.getMultibody("public_world_revolute_save_load_breakage");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredBody = restoredRobot->getLink("body");
  ASSERT_TRUE(restoredBody.has_value());
  auto restoredJoint
      = restored.getArticulatedJoint("serialized_world_broken_hinge");
  ASSERT_TRUE(restoredJoint.has_value());
  ASSERT_TRUE(restoredJoint->isBroken());
  EXPECT_EQ(restoredJoint->getType(), sx::JointType::Revolute);
  EXPECT_EQ(restoredJoint->getDOFCount(), 1u);
  EXPECT_EQ(restoredJoint->getChildLink().getName(), "body");
  EXPECT_THROW(
      { (void)restoredJoint->getParentLink(); }, sx::InvalidArgumentException);
  EXPECT_LT((restoredJoint->getAxis() - hingeAxis).norm(), 1e-12);
  EXPECT_DOUBLE_EQ(restoredJoint->getBreakForce(), 1e-18);
  EXPECT_EQ(restoredJoint->getActuatorType(), sx::ActuatorType::Velocity);
  ASSERT_EQ(restoredJoint->getCommandVelocity().size(), 1);
  EXPECT_DOUBLE_EQ(restoredJoint->getCommandVelocity()[0], targetSpeed);
  ASSERT_EQ(restoredJoint->getEffortLowerLimits().size(), 1);
  ASSERT_EQ(restoredJoint->getEffortUpperLimits().size(), 1);
  EXPECT_DOUBLE_EQ(restoredJoint->getEffortLowerLimits()[0], -1000.0);
  EXPECT_DOUBLE_EQ(restoredJoint->getEffortUpperLimits()[0], 1000.0);
  EXPECT_LT(
      (restoredBody->getWorldTransform().translation()
       - savedBrokenTransform.translation())
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredBody->getWorldTransform().linear()
       - savedBrokenTransform.linear())
          .norm(),
      1e-12);

  auto& registry = dart::simulation::detail::registryOf(restored);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(restoredJoint->getEntity());

  double maxBrokenAnchorResidual = 0.0;
  for (int k = 0; k < 20; ++k) {
    restoredBody->applyForce(4.0 * Eigen::Vector3d::UnitY());
    restored.step();
    maxBrokenAnchorResidual = std::max(
        maxBrokenAnchorResidual,
        (restoredBody->getWorldTransform() * childAnchor - worldAnchor).norm());
  }
  ASSERT_GT(maxBrokenAnchorResidual, 1e-4);

  const Eigen::Matrix3d rotationBeforeReset
      = restoredBody->getWorldTransform().linear();
  restoredBody->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredJoint->setBreakForce(1.0e12);
  restoredJoint->resetBreakage();
  ASSERT_FALSE(restoredJoint->isBroken());

  restored.step();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_LT((config.localAnchorA - worldAnchor).norm(), 1e-12);
  EXPECT_LT((config.localAnchorB - childAnchor).norm(), 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_NEAR(config.angularAxes.col(2).dot(hingeAxis), 1.0, 1e-12);

  const Eigen::Isometry3d resetTransform = restoredBody->getWorldTransform();
  const Eigen::AngleAxisd incrementalRotation(
      resetTransform.linear() * rotationBeforeReset.transpose());
  const double signedHingeIncrement
      = incrementalRotation.angle() * incrementalRotation.axis().dot(hingeAxis);
  EXPECT_FALSE(restoredJoint->isBroken());
  EXPECT_LT((resetTransform * childAnchor - worldAnchor).norm(), 1e-6);
  EXPECT_LT((resetTransform.linear() * hingeAxis - hingeAxis).norm(), 1e-6);
  EXPECT_NEAR(signedHingeIncrement, targetSpeed * dt, 5e-4);
}

// PLAN-104 AVBD articulated bridge: clearing a world after extracting
// world-link articulated rows must invalidate the old public facade and leave a
// fresh storage graph that can extract new AVBD world-link rows.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedWorldFacadeClearDropsExtractedRowsAndRebuilds)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  constexpr double dt = 0.005;
  world.setTimeStep(dt);

  auto robot = world.addMultibody("clear_revolute_world_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  const Eigen::Vector3d hingeAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  sx::Joint hinge
      = world.addArticulatedRevoluteJoint("clear_hinge", body, hingeAxis);
  ASSERT_EQ(hinge.getType(), sx::JointType::Revolute);
  hinge.setActuatorType(sx::ActuatorType::Velocity);
  const double hingeSpeed = 0.4;
  hinge.setCommandVelocity(Eigen::VectorXd::Constant(1, hingeSpeed));
  hinge.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));

  auto& firstRegistry = dart::simulation::detail::registryOf(world);
  const entt::entity hingeEntity
      = sx::detail::toRegistryEntity(hinge.getEntity());

  world.enterSimulationMode();

  ASSERT_TRUE(
      firstRegistry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(hingeEntity));
  const auto& hingeConfig
      = firstRegistry.get<dvbd::AvbdRigidWorldPointJointConfig>(hingeEntity);
  EXPECT_EQ(hingeConfig.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(
      hingeConfig.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_NEAR(hingeConfig.angularAxes.col(2).dot(hingeAxis), 1.0, 1e-12);

  constexpr int hingeSteps = 4;
  for (int k = 0; k < hingeSteps; ++k) {
    world.step();
  }
  EXPECT_GT(
      std::abs(signedRotationAroundAxis(
          body.getWorldTransform().linear(), hingeAxis)),
      1e-4);
  EXPECT_GT(world.getFrame(), 0u);

  world.clear();

  EXPECT_FALSE(world.isSimulationMode());
  EXPECT_FALSE(robot.isValid());
  EXPECT_FALSE(base.isValid());
  EXPECT_FALSE(body.isValid());
  EXPECT_FALSE(hinge.isValid());
  EXPECT_DOUBLE_EQ(world.getTimeStep(), 0.001);
  EXPECT_DOUBLE_EQ(world.getTime(), 0.0);
  EXPECT_EQ(world.getFrame(), 0u);
  EXPECT_EQ(world.getMultibodyCount(), 0u);
  EXPECT_EQ(world.getArticulatedJointCount(), 0u);
  EXPECT_EQ(
      world.getMultibodyOptions().integrationFamily,
      sx::MultibodyIntegrationFamily::SemiImplicit);
  EXPECT_FALSE(world.hasMultibody("clear_revolute_world_facade"));
  EXPECT_FALSE(world.hasArticulatedJoint("clear_hinge"));
  EXPECT_FALSE(world.getArticulatedJoint("clear_hinge").has_value());

  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(dt);

  auto rebuiltRobot = world.addMultibody("clear_prismatic_world_facade");
  auto rebuiltBase = rebuiltRobot.addLink("base");
  sx::JointSpec rebuiltFloatingSpec;
  rebuiltFloatingSpec.name = "floating";
  rebuiltFloatingSpec.type = sx::JointType::Floating;
  auto sliderBody
      = rebuiltRobot.addLink("body", rebuiltBase, rebuiltFloatingSpec);
  sliderBody.setMass(2.0);
  sliderBody.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(1.0, 2.0, 0.5).normalized();
  sx::Joint slider = world.addArticulatedPrismaticJoint(
      "clear_slider", sliderBody, sliderAxis);
  ASSERT_EQ(slider.getType(), sx::JointType::Prismatic);
  slider.setActuatorType(sx::ActuatorType::Velocity);
  const double sliderSpeed = 0.3;
  slider.setCommandVelocity(Eigen::VectorXd::Constant(1, sliderSpeed));
  slider.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));

  auto& secondRegistry = dart::simulation::detail::registryOf(world);
  const entt::entity sliderEntity
      = sx::detail::toRegistryEntity(slider.getEntity());
  EXPECT_TRUE(slider.isValid());
  EXPECT_TRUE(world.hasArticulatedJoint("clear_slider"));
  ASSERT_EQ(world.getArticulatedJointCount(), 1u);

  world.enterSimulationMode();

  ASSERT_TRUE(secondRegistry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(
      sliderEntity));
  const auto& sliderConfig
      = secondRegistry.get<dvbd::AvbdRigidWorldPointJointConfig>(sliderEntity);
  EXPECT_EQ(
      sliderConfig.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(sliderConfig.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(sliderConfig.linearAxes.col(2).dot(sliderAxis), 1.0, 1e-12);

  constexpr int sliderSteps = 4;
  for (int k = 0; k < sliderSteps; ++k) {
    world.step();
  }

  const Eigen::Isometry3d sliderTransform = sliderBody.getWorldTransform();
  const Eigen::Vector3d sliderPosition = sliderTransform.translation();
  EXPECT_LT(
      (sliderPosition - sliderPosition.dot(sliderAxis) * sliderAxis).norm(),
      1e-6);
  EXPECT_LT(
      (sliderTransform.linear() - Eigen::Matrix3d::Identity()).norm(), 1e-6);
  EXPECT_NEAR(
      sliderPosition.dot(sliderAxis), sliderSpeed * dt * sliderSteps, 1e-6);
}

// PLAN-104 AVBD articulated bridge: world-anchored prismatic facades preserve
// the captured world anchor and orientation while the velocity actuator drives
// the free translation axis.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedWorldPrismaticJointFacadeDrivesFloatingLinkEndpoint)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto robot = world.addMultibody("public_world_prismatic_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(1.0, 2.0, 0.5).normalized();
  sx::Joint joint
      = world.addArticulatedPrismaticJoint("world_slider", body, sliderAxis);
  ASSERT_EQ(joint.getType(), sx::JointType::Prismatic);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.3;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(config.linearAxes.col(2).dot(sliderAxis), 1.0, 1e-12);

  constexpr int steps = 20;
  double maxOrthogonalDrift = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    const Eigen::Vector3d position = transform.translation();
    maxOrthogonalDrift = std::max(
        maxOrthogonalDrift,
        (position - position.dot(sliderAxis) * sliderAxis).norm());
    maxRotationError = std::max(
        maxRotationError,
        (transform.linear() - Eigen::Matrix3d::Identity()).norm());
  }

  const Eigen::Vector3d position = body.getWorldTransform().translation();
  EXPECT_LT(maxOrthogonalDrift, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
  EXPECT_NEAR(position.dot(sliderAxis), targetSpeed * dt * steps, 1e-6);
}

// PLAN-104 AVBD articulated bridge: world-anchored prismatic facades preserve
// explicit off-origin anchors while freeing only translation along the
// configured world-frame slider axis.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedWorldPrismaticJointFacadeDrivesOffsetAnchor)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto robot = world.addMultibody("public_world_prismatic_offset_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
  pose.head<3>() = Eigen::Vector3d(0.2, -0.1, 0.3);
  body.getParentJoint().setPosition(pose);
  const Eigen::Vector3d childAnchor(0.1, 0.2, -0.1);
  const Eigen::Vector3d worldAnchor = body.getWorldTransform() * childAnchor;

  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(1.0, 2.0, 0.5).normalized();
  sx::Joint joint = world.addArticulatedPrismaticJoint(
      "world_offset_slider", body, sliderAxis, worldAnchor, childAnchor);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.3;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_LT((config.localAnchorA - worldAnchor).norm(), 1e-12);
  EXPECT_LT((config.localAnchorB - childAnchor).norm(), 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(config.linearAxes.col(2).dot(sliderAxis), 1.0, 1e-12);

  constexpr int steps = 20;
  double maxOrthogonalResidual = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    const Eigen::Vector3d anchorOffset = transform * childAnchor - worldAnchor;
    maxOrthogonalResidual = std::max(
        maxOrthogonalResidual,
        (anchorOffset - anchorOffset.dot(sliderAxis) * sliderAxis).norm());
    maxRotationError = std::max(
        maxRotationError,
        (transform.linear() - Eigen::Matrix3d::Identity()).norm());
  }

  const Eigen::Vector3d anchorOffset
      = body.getWorldTransform() * childAnchor - worldAnchor;
  EXPECT_LT(maxOrthogonalResidual, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
  EXPECT_NEAR(anchorOffset.dot(sliderAxis), targetSpeed * dt * steps, 1e-6);
}

// PLAN-104 AVBD articulated bridge: world-link public prismatic motor facades
// preserve explicit world/link anchors, velocity actuator state, effort limits,
// and break-force metadata across design-mode binary save/load before
// simulation entry rebuilds the private locked rows and free-axis motor row.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedWorldPrismaticJointFacadeSurvivesSaveLoad)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto robot = world.addMultibody("serialized_world_prismatic_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
  pose.head<3>() = Eigen::Vector3d(0.2, -0.1, 0.3);
  body.getParentJoint().setPosition(pose);
  const Eigen::Vector3d childAnchor(0.1, 0.2, -0.1);
  const Eigen::Vector3d worldAnchor = body.getWorldTransform() * childAnchor;

  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(1.0, 2.0, 0.5).normalized();
  sx::Joint joint = world.addArticulatedPrismaticJoint(
      "serialized_world_slider", body, sliderAxis, worldAnchor, childAnchor);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.3;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -750.0),
      Eigen::VectorXd::Constant(1, 850.0));
  joint.setBreakForce(14.0);

  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot
      = restored.getMultibody("serialized_world_prismatic_facade");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredBody = restoredRobot->getLink("body");
  ASSERT_TRUE(restoredBody.has_value());
  auto restoredJoint = restored.getArticulatedJoint("serialized_world_slider");
  ASSERT_TRUE(restoredJoint.has_value());
  EXPECT_EQ(restored.getArticulatedJointCount(), 1u);
  EXPECT_EQ(restoredJoint->getType(), sx::JointType::Prismatic);
  EXPECT_EQ(restoredJoint->getDOFCount(), 1u);
  EXPECT_EQ(restoredJoint->getChildLink().getName(), "body");
  EXPECT_THROW(
      { (void)restoredJoint->getParentLink(); }, sx::InvalidArgumentException);
  EXPECT_LT((restoredJoint->getAxis() - sliderAxis).norm(), 1e-12);
  EXPECT_EQ(restoredJoint->getActuatorType(), sx::ActuatorType::Velocity);
  ASSERT_EQ(restoredJoint->getCommandVelocity().size(), 1);
  EXPECT_DOUBLE_EQ(restoredJoint->getCommandVelocity()[0], targetSpeed);
  ASSERT_EQ(restoredJoint->getEffortLowerLimits().size(), 1);
  ASSERT_EQ(restoredJoint->getEffortUpperLimits().size(), 1);
  EXPECT_DOUBLE_EQ(restoredJoint->getEffortLowerLimits()[0], -750.0);
  EXPECT_DOUBLE_EQ(restoredJoint->getEffortUpperLimits()[0], 850.0);
  EXPECT_DOUBLE_EQ(restoredJoint->getBreakForce(), 14.0);
  EXPECT_FALSE(restoredJoint->isBroken());

  auto& registry = dart::simulation::detail::registryOf(restored);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(restoredJoint->getEntity());
  EXPECT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  restored.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_LT((config.localAnchorA - worldAnchor).norm(), 1e-12);
  EXPECT_LT((config.localAnchorB - childAnchor).norm(), 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(config.linearAxes.col(2).dot(sliderAxis), 1.0, 1e-12);
  EXPECT_TRUE(std::isinf(config.startStiffness));
  EXPECT_TRUE(std::isinf(config.maxStiffness));

  constexpr int steps = 20;
  double maxOrthogonalResidual = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < steps; ++k) {
    restored.step();

    const Eigen::Isometry3d transform = restoredBody->getWorldTransform();
    const Eigen::Vector3d anchorOffset = transform * childAnchor - worldAnchor;
    maxOrthogonalResidual = std::max(
        maxOrthogonalResidual,
        (anchorOffset - anchorOffset.dot(sliderAxis) * sliderAxis).norm());
    maxRotationError = std::max(
        maxRotationError,
        (transform.linear() - Eigen::Matrix3d::Identity()).norm());
  }

  const Eigen::Vector3d anchorOffset
      = restoredBody->getWorldTransform() * childAnchor - worldAnchor;
  EXPECT_LT(maxOrthogonalResidual, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
  EXPECT_NEAR(anchorOffset.dot(sliderAxis), targetSpeed * dt * steps, 1e-6);
}

// PLAN-104 AVBD articulated bridge: world-anchored prismatic facades also
// expose the private free-axis linear motor effort bound. A tiny positive force
// limit preserves the masked hard rows while preventing visible slider motion.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedWorldPrismaticJointFacadeRespectsTinyForceLimit)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto robot = world.addMultibody("public_world_prismatic_tiny_limit_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(1.0, 2.0, 0.5).normalized();
  const Eigen::Vector3d childAnchor(0.1, 0.2, -0.1);
  const Eigen::Vector3d worldAnchor = body.getWorldTransform() * childAnchor;
  sx::Joint joint = world.addArticulatedPrismaticJoint(
      "world_slider", body, sliderAxis, worldAnchor, childAnchor);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.3;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1e-9), Eigen::VectorXd::Constant(1, 1e-9));

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_LT((config.localAnchorA - worldAnchor).norm(), 1e-12);
  EXPECT_LT((config.localAnchorB - childAnchor).norm(), 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(config.linearAxes.col(2).dot(sliderAxis), 1.0, 1e-12);

  constexpr int steps = 20;
  double maxOrthogonalDrift = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    const Eigen::Vector3d anchorOffset = transform * childAnchor - worldAnchor;
    maxOrthogonalDrift = std::max(
        maxOrthogonalDrift,
        (anchorOffset - anchorOffset.dot(sliderAxis) * sliderAxis).norm());
    maxRotationError = std::max(
        maxRotationError,
        (transform.linear() - Eigen::Matrix3d::Identity()).norm());
  }

  const Eigen::Vector3d anchorOffset
      = body.getWorldTransform() * childAnchor - worldAnchor;
  EXPECT_LT(maxOrthogonalDrift, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
  EXPECT_LT(
      std::abs(anchorOffset.dot(sliderAxis)),
      0.01 * targetSpeed * world.getTimeStep() * steps);
}

// PLAN-104 AVBD articulated bridge: world-anchored public prismatic facades
// also rebuild the free-axis linear motor target when the command changes,
// while the masked rows keep orthogonal drift and rotation fixed.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedWorldPrismaticJointFacadeFollowsCommandUpdates)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto robot = world.addMultibody("public_world_prismatic_command_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  const Eigen::Vector3d sliderAxis = Eigen::Vector3d::UnitX();
  sx::Joint joint
      = world.addArticulatedPrismaticJoint("world_slider", body, sliderAxis);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.3;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));

  world.enterSimulationMode();

  double maxOrthogonalDrift = 0.0;
  double maxRotationError = 0.0;
  auto stepAndSliderPosition = [&]() {
    world.step();
    const Eigen::Isometry3d transform = body.getWorldTransform();
    const Eigen::Vector3d position = transform.translation();
    maxOrthogonalDrift = std::max(
        maxOrthogonalDrift,
        (position - position.dot(sliderAxis) * sliderAxis).norm());
    maxRotationError = std::max(
        maxRotationError,
        (transform.linear() - Eigen::Matrix3d::Identity()).norm());
    return position.dot(sliderAxis);
  };

  constexpr int forwardSteps = 10;
  double positionAfterForward = 0.0;
  for (int k = 0; k < forwardSteps; ++k) {
    positionAfterForward = stepAndSliderPosition();
  }

  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, -targetSpeed));

  constexpr int reverseSteps = 10;
  double positionAfterReverse = positionAfterForward;
  for (int k = 0; k < reverseSteps; ++k) {
    positionAfterReverse = stepAndSliderPosition();
  }

  EXPECT_LT(maxOrthogonalDrift, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
  EXPECT_NEAR(positionAfterForward, targetSpeed * dt * forwardSteps, 1e-6);
  EXPECT_NEAR(
      positionAfterReverse,
      targetSpeed * dt * (forwardSteps - reverseSteps),
      1e-6);
}

// PLAN-104 AVBD articulated bridge: world-anchored public prismatic motor rows
// also mark their source joint broken and stop contributing masked hard rows on
// later steps, allowing orthogonal drift away from the captured world slider.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedWorldPrismaticBreakForceMarksMotorRowsAndSkipsLaterSteps)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto robot = world.addMultibody("public_world_prismatic_breakable_skip");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(1.0, 2.0, 0.5).normalized();
  const Eigen::Vector3d lateralForce
      = (Eigen::Vector3d::UnitY()
         - Eigen::Vector3d::UnitY().dot(sliderAxis) * sliderAxis)
            .normalized();
  sx::Joint joint = world.addArticulatedPrismaticJoint(
      "world_breakable_slider", body, sliderAxis);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.3;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));
  joint.setBreakForce(1e-18);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(config.linearAxes.col(2).dot(sliderAxis), 1.0, 1e-12);

  world.step();

  ASSERT_TRUE(joint.isBroken());
  EXPECT_NEAR(
      body.getWorldTransform().translation().dot(sliderAxis),
      targetSpeed * dt,
      1e-6);

  double maxOrthogonalDrift = 0.0;
  for (int k = 0; k < 20; ++k) {
    body.applyForce(4.0 * lateralForce);
    world.step();
    const Eigen::Vector3d position = body.getWorldTransform().translation();
    maxOrthogonalDrift = std::max(
        maxOrthogonalDrift,
        (position - position.dot(sliderAxis) * sliderAxis).norm());
  }

  EXPECT_GT(maxOrthogonalDrift, 1e-4);
}

// PLAN-104 AVBD articulated bridge: world-anchored public prismatic motor rows
// likewise break and re-engage through the public lifecycle while the masked
// rows keep the slider's orthogonal coordinates fixed.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedWorldPrismaticBreakForceResetReengagesMotorRows)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto robot = world.addMultibody("public_world_prismatic_breakable_facade");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(1.0, 2.0, 0.5).normalized();
  sx::Joint joint = world.addArticulatedPrismaticJoint(
      "world_breakable_slider", body, sliderAxis);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.3;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));
  joint.setBreakForce(1e-18);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());

  world.enterSimulationMode();
  world.step();

  ASSERT_TRUE(joint.isBroken());
  EXPECT_NEAR(
      body.getWorldTransform().translation().dot(sliderAxis),
      targetSpeed * dt,
      1e-6);

  joint.setBreakForce(1.0e12);
  joint.resetBreakage();
  ASSERT_FALSE(joint.isBroken());

  world.step();

  const Eigen::Isometry3d transform = body.getWorldTransform();
  const Eigen::Vector3d position = transform.translation();
  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(config.linearAxes.col(2).dot(sliderAxis), 1.0, 1e-12);
  EXPECT_FALSE(joint.isBroken());
  EXPECT_LT((position - position.dot(sliderAxis) * sliderAxis).norm(), 1e-6);
  EXPECT_LT((transform.linear() - Eigen::Matrix3d::Identity()).norm(), 1e-6);
  EXPECT_NEAR(position.dot(sliderAxis), targetSpeed * dt * 2.0, 1e-6);
}

// PLAN-104 AVBD articulated bridge: a broken world-anchored public prismatic
// motor must persist through binary save/load as skipped masked rows. Resetting
// the restored joint then re-enables the slider constraint and motor row.
TEST(
    VariationalIntegration,
    AvbdPublicArticulatedWorldPrismaticBreakageSurvivesSaveLoadAndReset)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto robot = world.addMultibody("public_world_prismatic_save_load_breakage");
  auto base = robot.addLink("base");
  sx::JointSpec floatingSpec;
  floatingSpec.name = "floating";
  floatingSpec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, floatingSpec);
  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal());

  Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
  pose.head<3>() = Eigen::Vector3d(0.2, -0.15, 0.05);
  body.getParentJoint().setPosition(pose);
  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(1.0, 2.0, 0.5).normalized();
  const Eigen::Vector3d childAnchor(0.0, 0.2, 0.1);
  const Eigen::Vector3d worldAnchor = body.getWorldTransform() * childAnchor;
  sx::Joint joint = world.addArticulatedPrismaticJoint(
      "serialized_world_broken_slider",
      body,
      sliderAxis,
      worldAnchor,
      childAnchor);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  const double targetSpeed = 0.3;
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, targetSpeed));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -1000.0),
      Eigen::VectorXd::Constant(1, 1000.0));
  joint.setBreakForce(1e-18);

  world.enterSimulationMode();
  world.step();
  ASSERT_TRUE(joint.isBroken());

  const Eigen::Isometry3d savedBrokenTransform = body.getWorldTransform();
  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot
      = restored.getMultibody("public_world_prismatic_save_load_breakage");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredBody = restoredRobot->getLink("body");
  ASSERT_TRUE(restoredBody.has_value());
  auto restoredJoint
      = restored.getArticulatedJoint("serialized_world_broken_slider");
  ASSERT_TRUE(restoredJoint.has_value());
  ASSERT_TRUE(restoredJoint->isBroken());
  EXPECT_EQ(restoredJoint->getType(), sx::JointType::Prismatic);
  EXPECT_EQ(restoredJoint->getDOFCount(), 1u);
  EXPECT_EQ(restoredJoint->getChildLink().getName(), "body");
  EXPECT_THROW(
      { (void)restoredJoint->getParentLink(); }, sx::InvalidArgumentException);
  EXPECT_LT((restoredJoint->getAxis() - sliderAxis).norm(), 1e-12);
  EXPECT_DOUBLE_EQ(restoredJoint->getBreakForce(), 1e-18);
  EXPECT_EQ(restoredJoint->getActuatorType(), sx::ActuatorType::Velocity);
  ASSERT_EQ(restoredJoint->getCommandVelocity().size(), 1);
  EXPECT_DOUBLE_EQ(restoredJoint->getCommandVelocity()[0], targetSpeed);
  ASSERT_EQ(restoredJoint->getEffortLowerLimits().size(), 1);
  ASSERT_EQ(restoredJoint->getEffortUpperLimits().size(), 1);
  EXPECT_DOUBLE_EQ(restoredJoint->getEffortLowerLimits()[0], -1000.0);
  EXPECT_DOUBLE_EQ(restoredJoint->getEffortUpperLimits()[0], 1000.0);
  EXPECT_LT(
      (restoredBody->getWorldTransform().translation()
       - savedBrokenTransform.translation())
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredBody->getWorldTransform().linear()
       - savedBrokenTransform.linear())
          .norm(),
      1e-12);

  auto& registry = dart::simulation::detail::registryOf(restored);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(restoredJoint->getEntity());

  double maxOrthogonalDrift = 0.0;
  for (int k = 0; k < 20; ++k) {
    restoredBody->applyForce(4.0 * Eigen::Vector3d::UnitY());
    restored.step();
    const Eigen::Vector3d anchorDelta
        = restoredBody->getWorldTransform() * childAnchor - worldAnchor;
    maxOrthogonalDrift = std::max(
        maxOrthogonalDrift,
        (anchorDelta - anchorDelta.dot(sliderAxis) * sliderAxis).norm());
  }
  ASSERT_GT(maxOrthogonalDrift, 1e-4);

  const double sliderPositionBeforeReset
      = (restoredBody->getWorldTransform() * childAnchor - worldAnchor)
            .dot(sliderAxis);
  restoredBody->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredJoint->setBreakForce(1.0e12);
  restoredJoint->resetBreakage();
  ASSERT_FALSE(restoredJoint->isBroken());

  restored.step();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_LT((config.localAnchorA - worldAnchor).norm(), 1e-12);
  EXPECT_LT((config.localAnchorB - childAnchor).norm(), 1e-12);
  EXPECT_EQ(config.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(config.linearAxes.col(2).dot(sliderAxis), 1.0, 1e-12);

  const Eigen::Isometry3d resetTransform = restoredBody->getWorldTransform();
  const Eigen::Vector3d resetPosition
      = resetTransform * childAnchor - worldAnchor;
  EXPECT_FALSE(restoredJoint->isBroken());
  EXPECT_LT(
      (resetPosition - resetPosition.dot(sliderAxis) * sliderAxis).norm(),
      1e-6);
  EXPECT_LT(
      (resetTransform.linear() - Eigen::Matrix3d::Identity()).norm(), 1e-6);
  EXPECT_NEAR(
      resetPosition.dot(sliderAxis),
      sliderPositionBeforeReset + targetSpeed * dt,
      1e-6);
}

// PLAN-104 AVBD articulated bridge: private masked revolute point-joint configs
// on multibody links now become masked variational rigid closures instead of
// being skipped. The body origin stays pinned while rotation about the free
// hinge axis remains available.
TEST(
    VariationalIntegration,
    AvbdRevolutePointJointConfigSolvesMaskedFloatingLinkEndpoint)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto body = addFloatingBody(world, /*mass=*/2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Revolute;
  joint.parentLink = sx::detail::toRegistryEntity(body.getEntity());
  joint.childLink = entt::null;

  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.angularAxes
      = dvbd::avbdRigidJointAxesFromFreeAxis(Eigen::Vector3d::UnitZ());
  config.angularAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();

  double maxPositionResidual = 0.0;
  double maxHingeAxisTilt = 0.0;
  double maxYaw = 0.0;
  for (int k = 0; k < 80; ++k) {
    body.applyForce(
        2.0 * Eigen::Vector3d::UnitY(), 0.5 * Eigen::Vector3d::UnitX());
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxPositionResidual
        = std::max(maxPositionResidual, transform.translation().norm());
    maxHingeAxisTilt = std::max(
        maxHingeAxisTilt,
        (transform.linear() * Eigen::Vector3d::UnitZ()
         - Eigen::Vector3d::UnitZ())
            .norm());
    maxYaw = std::max(
        maxYaw,
        std::abs(
            std::atan2(transform.linear()(1, 0), transform.linear()(0, 0))));
  }

  EXPECT_LT(maxPositionResidual, 1e-6);
  EXPECT_LT(maxHingeAxisTilt, 1e-6);
  EXPECT_GT(maxYaw, 1e-3);
}

// PLAN-104 AVBD articulated bridge: a private hard revolute point-joint
// velocity actuator contributes the free-axis angular motor row while the
// masked holonomic rows keep the endpoint pinned and hinge axis aligned.
TEST(
    VariationalIntegration,
    AvbdRevolutePointJointVelocityActuatorDrivesMaskedFloatingLinkEndpoint)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto body = addFloatingBody(world, /*mass=*/2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Revolute;
  joint.parentLink = entt::null;
  joint.childLink = sx::detail::toRegistryEntity(body.getEntity());
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.4;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);

  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.angularAxes
      = dvbd::avbdRigidJointAxesFromFreeAxis(Eigen::Vector3d::UnitZ());
  config.angularAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();

  constexpr int steps = 20;
  double maxPositionResidual = 0.0;
  double maxHingeAxisTilt = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxPositionResidual
        = std::max(maxPositionResidual, transform.translation().norm());
    maxHingeAxisTilt = std::max(
        maxHingeAxisTilt,
        (transform.linear() * Eigen::Vector3d::UnitZ()
         - Eigen::Vector3d::UnitZ())
            .norm());
  }

  const Eigen::Matrix3d rotation = body.getWorldTransform().linear();
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  EXPECT_LT(maxPositionResidual, 1e-6);
  EXPECT_LT(maxHingeAxisTilt, 1e-6);
  EXPECT_NEAR(yaw, targetSpeed * dt * steps, 1e-6);
}

// PLAN-104 AVBD articulated bridge: the free-axis angular motor row is effort
// bounded. A tiny but positive torque limit still creates the motor row, but it
// no longer acts as an unbounded hard coordinate target.
TEST(
    VariationalIntegration,
    AvbdRevolutePointJointVelocityActuatorRespectsTinyTorqueLimit)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto body = addFloatingBody(world, /*mass=*/2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Revolute;
  joint.parentLink = entt::null;
  joint.childLink = sx::detail::toRegistryEntity(body.getEntity());
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.4;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1e-9);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1e-9);

  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.angularAxes
      = dvbd::avbdRigidJointAxesFromFreeAxis(Eigen::Vector3d::UnitZ());
  config.angularAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();

  constexpr int steps = 20;
  double maxPositionResidual = 0.0;
  double maxHingeAxisTilt = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxPositionResidual
        = std::max(maxPositionResidual, transform.translation().norm());
    maxHingeAxisTilt = std::max(
        maxHingeAxisTilt,
        (transform.linear() * Eigen::Vector3d::UnitZ()
         - Eigen::Vector3d::UnitZ())
            .norm());
  }

  const Eigen::Matrix3d rotation = body.getWorldTransform().linear();
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  EXPECT_LT(maxPositionResidual, 1e-6);
  EXPECT_LT(maxHingeAxisTilt, 1e-6);
  EXPECT_LT(std::abs(yaw), 1e-6);
}

// PLAN-104 AVBD articulated bridge: direct private finite-effort revolute rows
// should stay bounded when the child endpoint uses a generated non-cardinal
// free-axis basis, not only the cardinal Z basis.
TEST(
    VariationalIntegration,
    AvbdRevolutePointJointVelocityActuatorRespectsTinyTorqueLimitOnNonCardinalAxis)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto body = addFloatingBody(world, /*mass=*/2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Revolute;
  joint.parentLink = entt::null;
  joint.childLink = sx::detail::toRegistryEntity(body.getEntity());
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.4;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1e-9);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1e-9);

  const Eigen::Vector3d hingeAxis
      = Eigen::Vector3d(0.1, 0.6, -0.8).normalized();
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.angularAxes = dvbd::avbdRigidJointAxesFromFreeAxis(hingeAxis);
  config.angularAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();

  constexpr int steps = 20;
  double maxPositionResidual = 0.0;
  double maxHingeAxisTilt = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxPositionResidual
        = std::max(maxPositionResidual, transform.translation().norm());
    maxHingeAxisTilt = std::max(
        maxHingeAxisTilt, (transform.linear() * hingeAxis - hingeAxis).norm());
  }

  const double yaw
      = signedRotationAroundAxis(body.getWorldTransform().linear(), hingeAxis);
  EXPECT_LT(maxPositionResidual, 1e-6);
  EXPECT_LT(maxHingeAxisTilt, 1e-3);
  EXPECT_LT(std::abs(yaw), 0.01 * targetSpeed * world.getTimeStep() * steps);
}

// PLAN-104 AVBD articulated bridge: the bounded free-axis angular motor row
// must also honor tiny effort limits when the multibody link is the parent
// endpoint of a world-link private point-joint.
TEST(
    VariationalIntegration,
    AvbdRevolutePointJointVelocityActuatorParentEndpointRespectsTinyTorqueLimit)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto body = addFloatingBody(world, /*mass=*/2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Revolute;
  joint.parentLink = sx::detail::toRegistryEntity(body.getEntity());
  joint.childLink = entt::null;
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.4;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1e-9);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1e-9);

  const Eigen::Vector3d hingeAxis = Eigen::Vector3d::UnitZ();
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.angularAxes = dvbd::avbdRigidJointAxesFromFreeAxis(hingeAxis);
  config.angularAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();

  constexpr int steps = 20;
  double maxPositionResidual = 0.0;
  double maxHingeAxisTilt = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxPositionResidual
        = std::max(maxPositionResidual, transform.translation().norm());
    maxHingeAxisTilt = std::max(
        maxHingeAxisTilt, (transform.linear() * hingeAxis - hingeAxis).norm());
  }

  const double yaw
      = signedRotationAroundAxis(body.getWorldTransform().linear(), hingeAxis);
  EXPECT_LT(maxPositionResidual, 1e-6);
  EXPECT_LT(maxHingeAxisTilt, 1e-6);
  EXPECT_LT(std::abs(yaw), 1e-6);
}

// PLAN-104 AVBD articulated bridge: parent-endpoint finite-effort revolute rows
// should also stay bounded when the free angular axis is represented by a
// generated non-cardinal basis.
TEST(
    VariationalIntegration,
    AvbdRevolutePointJointVelocityActuatorParentEndpointRespectsTinyTorqueLimitOnNonCardinalAxis)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto body = addFloatingBody(world, /*mass=*/2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Revolute;
  joint.parentLink = sx::detail::toRegistryEntity(body.getEntity());
  joint.childLink = entt::null;
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.4;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1e-9);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1e-9);

  const Eigen::Vector3d hingeAxis
      = Eigen::Vector3d(0.2, -0.5, 0.8).normalized();
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.angularAxes = dvbd::avbdRigidJointAxesFromFreeAxis(hingeAxis);
  config.angularAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();

  constexpr int steps = 20;
  double maxPositionResidual = 0.0;
  double maxHingeAxisTilt = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxPositionResidual
        = std::max(maxPositionResidual, transform.translation().norm());
    maxHingeAxisTilt = std::max(
        maxHingeAxisTilt, (transform.linear() * hingeAxis - hingeAxis).norm());
  }

  const double yaw
      = signedRotationAroundAxis(body.getWorldTransform().linear(), hingeAxis);
  EXPECT_LT(maxPositionResidual, 1e-6);
  EXPECT_LT(maxHingeAxisTilt, 1e-3);
  EXPECT_LT(std::abs(yaw), 0.01 * targetSpeed * world.getTimeStep() * steps);
}

// PLAN-104 AVBD articulated bridge: private point-joint motor rows are rebuilt
// from the source joint each step, so command changes should not reuse stale
// targets while the masked holonomic rows keep the hinge anchored.
TEST(
    VariationalIntegration,
    AvbdRevolutePointJointVelocityActuatorFollowsCommandUpdates)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto body = addFloatingBody(world, /*mass=*/2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Revolute;
  joint.parentLink = entt::null;
  joint.childLink = sx::detail::toRegistryEntity(body.getEntity());
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.4;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);

  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.angularAxes
      = dvbd::avbdRigidJointAxesFromFreeAxis(Eigen::Vector3d::UnitZ());
  config.angularAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();

  double maxPositionResidual = 0.0;
  double maxHingeAxisTilt = 0.0;
  auto stepAndYaw = [&]() {
    world.step();
    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxPositionResidual
        = std::max(maxPositionResidual, transform.translation().norm());
    maxHingeAxisTilt = std::max(
        maxHingeAxisTilt,
        (transform.linear() * Eigen::Vector3d::UnitZ()
         - Eigen::Vector3d::UnitZ())
            .norm());
    const Eigen::Matrix3d rotation = transform.linear();
    return std::atan2(rotation(1, 0), rotation(0, 0));
  };

  constexpr int forwardSteps = 10;
  double yawAfterForward = 0.0;
  for (int k = 0; k < forwardSteps; ++k) {
    yawAfterForward = stepAndYaw();
  }

  registry.get<sx::comps::JointActuation>(jointEntity).commandVelocity
      = Eigen::VectorXd::Constant(1, -targetSpeed);

  constexpr int reverseSteps = 10;
  double yawAfterReverse = yawAfterForward;
  for (int k = 0; k < reverseSteps; ++k) {
    yawAfterReverse = stepAndYaw();
  }

  EXPECT_LT(maxPositionResidual, 1e-6);
  EXPECT_LT(maxHingeAxisTilt, 1e-6);
  EXPECT_NEAR(yawAfterForward, targetSpeed * dt * forwardSteps, 1e-6);
  EXPECT_NEAR(
      yawAfterReverse, targetSpeed * dt * (forwardSteps - reverseSteps), 1e-6);
}

// PLAN-104 AVBD articulated bridge: direct private revolute command updates
// should also rebuild velocity motor targets when the child endpoint uses a
// generated non-cardinal free-axis basis.
TEST(
    VariationalIntegration,
    AvbdRevolutePointJointVelocityActuatorFollowsNonCardinalCommandUpdates)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto body = addFloatingBody(world, /*mass=*/2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Revolute;
  joint.parentLink = entt::null;
  joint.childLink = sx::detail::toRegistryEntity(body.getEntity());
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.4;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);

  const Eigen::Vector3d hingeAxis
      = Eigen::Vector3d(-0.3, 0.5, 0.8).normalized();
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.angularAxes = dvbd::avbdRigidJointAxesFromFreeAxis(hingeAxis);
  config.angularAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();

  double maxPositionResidual = 0.0;
  double maxHingeAxisTilt = 0.0;
  auto stepAndYaw = [&]() {
    world.step();
    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxPositionResidual
        = std::max(maxPositionResidual, transform.translation().norm());
    maxHingeAxisTilt = std::max(
        maxHingeAxisTilt, (transform.linear() * hingeAxis - hingeAxis).norm());
    return signedRotationAroundAxis(transform.linear(), hingeAxis);
  };

  constexpr int forwardSteps = 10;
  double yawAfterForward = 0.0;
  for (int k = 0; k < forwardSteps; ++k) {
    yawAfterForward = stepAndYaw();
  }

  registry.get<sx::comps::JointActuation>(jointEntity).commandVelocity
      = Eigen::VectorXd::Constant(1, -targetSpeed);

  constexpr int reverseSteps = 10;
  double yawAfterReverse = yawAfterForward;
  for (int k = 0; k < reverseSteps; ++k) {
    yawAfterReverse = stepAndYaw();
  }

  EXPECT_LT(maxPositionResidual, 1e-6);
  EXPECT_LT(maxHingeAxisTilt, 1e-6);
  EXPECT_NEAR(yawAfterForward, targetSpeed * dt * forwardSteps, 1e-6);
  EXPECT_NEAR(
      yawAfterReverse, targetSpeed * dt * (forwardSteps - reverseSteps), 1e-6);
}

// PLAN-104 AVBD articulated bridge: command updates must also rebuild private
// parent-endpoint revolute motor targets, whose free-axis sign is opposite the
// child-endpoint world-link case.
TEST(
    VariationalIntegration,
    AvbdRevolutePointJointVelocityActuatorParentEndpointFollowsCommandUpdates)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto body = addFloatingBody(world, /*mass=*/2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Revolute;
  joint.parentLink = sx::detail::toRegistryEntity(body.getEntity());
  joint.childLink = entt::null;
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.4;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);

  const Eigen::Vector3d hingeAxis = Eigen::Vector3d::UnitZ();
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.angularAxes = dvbd::avbdRigidJointAxesFromFreeAxis(hingeAxis);
  config.angularAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();

  double maxPositionResidual = 0.0;
  double maxHingeAxisTilt = 0.0;
  auto stepAndYaw = [&]() {
    world.step();
    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxPositionResidual
        = std::max(maxPositionResidual, transform.translation().norm());
    maxHingeAxisTilt = std::max(
        maxHingeAxisTilt, (transform.linear() * hingeAxis - hingeAxis).norm());
    return signedRotationAroundAxis(transform.linear(), hingeAxis);
  };

  constexpr int forwardSteps = 10;
  double yawAfterForward = 0.0;
  for (int k = 0; k < forwardSteps; ++k) {
    yawAfterForward = stepAndYaw();
  }

  registry.get<sx::comps::JointActuation>(jointEntity).commandVelocity
      = Eigen::VectorXd::Constant(1, -targetSpeed);

  constexpr int reverseSteps = 10;
  double yawAfterReverse = yawAfterForward;
  for (int k = 0; k < reverseSteps; ++k) {
    yawAfterReverse = stepAndYaw();
  }

  EXPECT_LT(maxPositionResidual, 1e-6);
  EXPECT_LT(maxHingeAxisTilt, 1e-6);
  EXPECT_NEAR(yawAfterForward, -targetSpeed * dt * forwardSteps, 1e-6);
  EXPECT_NEAR(
      yawAfterReverse, -targetSpeed * dt * (forwardSteps - reverseSteps), 1e-6);
}

// PLAN-104 AVBD articulated bridge: parent-endpoint command updates should also
// rebuild velocity motor targets when the private revolute config uses a
// generated non-cardinal free-axis basis.
TEST(
    VariationalIntegration,
    AvbdRevolutePointJointVelocityActuatorParentEndpointFollowsNonCardinalCommandUpdates)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto body = addFloatingBody(world, /*mass=*/2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Revolute;
  joint.parentLink = sx::detail::toRegistryEntity(body.getEntity());
  joint.childLink = entt::null;
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.4;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);

  const Eigen::Vector3d hingeAxis
      = Eigen::Vector3d(0.2, -0.4, 0.9).normalized();
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.angularAxes = dvbd::avbdRigidJointAxesFromFreeAxis(hingeAxis);
  config.angularAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();

  double maxPositionResidual = 0.0;
  double maxHingeAxisTilt = 0.0;
  auto stepAndYaw = [&]() {
    world.step();
    const Eigen::Isometry3d transform = body.getWorldTransform();
    maxPositionResidual
        = std::max(maxPositionResidual, transform.translation().norm());
    maxHingeAxisTilt = std::max(
        maxHingeAxisTilt, (transform.linear() * hingeAxis - hingeAxis).norm());
    return signedRotationAroundAxis(transform.linear(), hingeAxis);
  };

  constexpr int forwardSteps = 10;
  double yawAfterForward = 0.0;
  for (int k = 0; k < forwardSteps; ++k) {
    yawAfterForward = stepAndYaw();
  }

  registry.get<sx::comps::JointActuation>(jointEntity).commandVelocity
      = Eigen::VectorXd::Constant(1, -targetSpeed);

  constexpr int reverseSteps = 10;
  double yawAfterReverse = yawAfterForward;
  for (int k = 0; k < reverseSteps; ++k) {
    yawAfterReverse = stepAndYaw();
  }

  EXPECT_LT(maxPositionResidual, 1e-6);
  EXPECT_LT(maxHingeAxisTilt, 1e-6);
  EXPECT_NEAR(yawAfterForward, -targetSpeed * dt * forwardSteps, 1e-6);
  EXPECT_NEAR(
      yawAfterReverse, -targetSpeed * dt * (forwardSteps - reverseSteps), 1e-6);
}

// PLAN-104 AVBD articulated bridge: a private hard prismatic point-joint
// velocity actuator contributes the free-axis linear motor row while the
// masked holonomic rows keep orthogonal drift and rotation fixed.
TEST(
    VariationalIntegration,
    AvbdPrismaticPointJointVelocityActuatorDrivesMaskedFloatingLinkEndpoint)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto body = addFloatingBody(world, /*mass=*/2.0);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Prismatic;
  joint.parentLink = entt::null;
  joint.childLink = sx::detail::toRegistryEntity(body.getEntity());
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.3;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);

  const Eigen::Vector3d sliderAxis = Eigen::Vector3d::UnitX();
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.linearAxes = dvbd::avbdRigidJointAxesFromFreeAxis(sliderAxis);
  config.angularAxes = config.linearAxes;
  config.linearAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.angularAxisMask = dvbd::kAvbdRigidJointAllAxesMask;
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();

  constexpr int steps = 20;
  double maxOrthogonalDrift = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    const Eigen::Vector3d position = transform.translation();
    maxOrthogonalDrift = std::max(
        maxOrthogonalDrift,
        (position - position.dot(sliderAxis) * sliderAxis).norm());
    maxRotationError = std::max(
        maxRotationError,
        (transform.linear() - Eigen::Matrix3d::Identity()).norm());
  }

  const Eigen::Vector3d position = body.getWorldTransform().translation();
  EXPECT_LT(maxOrthogonalDrift, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
  EXPECT_NEAR(position.dot(sliderAxis), targetSpeed * dt * steps, 1e-6);
}

// PLAN-104 AVBD articulated bridge: private prismatic point-joint motor rows
// are also rebuilt from the source joint each step, so command changes should
// not reuse stale slider targets while the masked rows keep the rail aligned.
TEST(
    VariationalIntegration,
    AvbdPrismaticPointJointVelocityActuatorFollowsCommandUpdates)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto body = addFloatingBody(world, /*mass=*/2.0);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Prismatic;
  joint.parentLink = entt::null;
  joint.childLink = sx::detail::toRegistryEntity(body.getEntity());
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.3;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);

  const Eigen::Vector3d sliderAxis = Eigen::Vector3d::UnitX();
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.linearAxes = dvbd::avbdRigidJointAxesFromFreeAxis(sliderAxis);
  config.angularAxes = config.linearAxes;
  config.linearAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.angularAxisMask = dvbd::kAvbdRigidJointAllAxesMask;
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();

  double maxOrthogonalDrift = 0.0;
  double maxRotationError = 0.0;
  auto stepAndSliderPosition = [&]() {
    world.step();
    const Eigen::Isometry3d transform = body.getWorldTransform();
    const Eigen::Vector3d position = transform.translation();
    maxOrthogonalDrift = std::max(
        maxOrthogonalDrift,
        (position - position.dot(sliderAxis) * sliderAxis).norm());
    maxRotationError = std::max(
        maxRotationError,
        (transform.linear() - Eigen::Matrix3d::Identity()).norm());
    return position.dot(sliderAxis);
  };

  constexpr int forwardSteps = 10;
  double positionAfterForward = 0.0;
  for (int k = 0; k < forwardSteps; ++k) {
    positionAfterForward = stepAndSliderPosition();
  }

  registry.get<sx::comps::JointActuation>(jointEntity).commandVelocity
      = Eigen::VectorXd::Constant(1, -targetSpeed);

  constexpr int reverseSteps = 10;
  double positionAfterReverse = positionAfterForward;
  for (int k = 0; k < reverseSteps; ++k) {
    positionAfterReverse = stepAndSliderPosition();
  }

  EXPECT_LT(maxOrthogonalDrift, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
  EXPECT_NEAR(positionAfterForward, targetSpeed * dt * forwardSteps, 1e-6);
  EXPECT_NEAR(
      positionAfterReverse,
      targetSpeed * dt * (forwardSteps - reverseSteps),
      1e-6);
}

// PLAN-104 AVBD articulated bridge: direct private prismatic command updates
// should also rebuild velocity motor targets when the child endpoint uses a
// generated non-cardinal slider basis.
TEST(
    VariationalIntegration,
    AvbdPrismaticPointJointVelocityActuatorFollowsNonCardinalCommandUpdates)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto body = addFloatingBody(world, /*mass=*/2.0);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Prismatic;
  joint.parentLink = entt::null;
  joint.childLink = sx::detail::toRegistryEntity(body.getEntity());
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.3;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);

  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(0.4, 0.8, -0.3).normalized();
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.linearAxes = dvbd::avbdRigidJointAxesFromFreeAxis(sliderAxis);
  config.angularAxes = config.linearAxes;
  config.linearAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.angularAxisMask = dvbd::kAvbdRigidJointAllAxesMask;
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();

  double maxOrthogonalDrift = 0.0;
  double maxRotationError = 0.0;
  auto stepAndSliderPosition = [&]() {
    world.step();
    const Eigen::Isometry3d transform = body.getWorldTransform();
    const Eigen::Vector3d position = transform.translation();
    maxOrthogonalDrift = std::max(
        maxOrthogonalDrift,
        (position - position.dot(sliderAxis) * sliderAxis).norm());
    maxRotationError = std::max(
        maxRotationError,
        (transform.linear() - Eigen::Matrix3d::Identity()).norm());
    return position.dot(sliderAxis);
  };

  constexpr int forwardSteps = 10;
  double positionAfterForward = 0.0;
  for (int k = 0; k < forwardSteps; ++k) {
    positionAfterForward = stepAndSliderPosition();
  }

  registry.get<sx::comps::JointActuation>(jointEntity).commandVelocity
      = Eigen::VectorXd::Constant(1, -targetSpeed);

  constexpr int reverseSteps = 10;
  double positionAfterReverse = positionAfterForward;
  for (int k = 0; k < reverseSteps; ++k) {
    positionAfterReverse = stepAndSliderPosition();
  }

  EXPECT_LT(maxOrthogonalDrift, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
  EXPECT_NEAR(positionAfterForward, targetSpeed * dt * forwardSteps, 1e-6);
  EXPECT_NEAR(
      positionAfterReverse,
      targetSpeed * dt * (forwardSteps - reverseSteps),
      1e-6);
}

// PLAN-104 AVBD articulated bridge: command updates must also rebuild private
// parent-endpoint prismatic motor targets, whose free-axis sign is opposite the
// child-endpoint world-link case.
TEST(
    VariationalIntegration,
    AvbdPrismaticPointJointVelocityActuatorParentEndpointFollowsCommandUpdates)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto body = addFloatingBody(world, /*mass=*/2.0);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Prismatic;
  joint.parentLink = sx::detail::toRegistryEntity(body.getEntity());
  joint.childLink = entt::null;
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.3;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);

  const Eigen::Vector3d sliderAxis = Eigen::Vector3d::UnitX();
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.linearAxes = dvbd::avbdRigidJointAxesFromFreeAxis(sliderAxis);
  config.angularAxes = config.linearAxes;
  config.linearAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.angularAxisMask = dvbd::kAvbdRigidJointAllAxesMask;
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();

  double maxOrthogonalDrift = 0.0;
  double maxRotationError = 0.0;
  auto stepAndSliderPosition = [&]() {
    world.step();
    const Eigen::Isometry3d transform = body.getWorldTransform();
    const Eigen::Vector3d position = transform.translation();
    maxOrthogonalDrift = std::max(
        maxOrthogonalDrift,
        (position - position.dot(sliderAxis) * sliderAxis).norm());
    maxRotationError = std::max(
        maxRotationError,
        (transform.linear() - Eigen::Matrix3d::Identity()).norm());
    return position.dot(sliderAxis);
  };

  constexpr int forwardSteps = 10;
  double positionAfterForward = 0.0;
  for (int k = 0; k < forwardSteps; ++k) {
    positionAfterForward = stepAndSliderPosition();
  }

  registry.get<sx::comps::JointActuation>(jointEntity).commandVelocity
      = Eigen::VectorXd::Constant(1, -targetSpeed);

  constexpr int reverseSteps = 10;
  double positionAfterReverse = positionAfterForward;
  for (int k = 0; k < reverseSteps; ++k) {
    positionAfterReverse = stepAndSliderPosition();
  }

  EXPECT_LT(maxOrthogonalDrift, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
  EXPECT_NEAR(positionAfterForward, -targetSpeed * dt * forwardSteps, 1e-6);
  EXPECT_NEAR(
      positionAfterReverse,
      -targetSpeed * dt * (forwardSteps - reverseSteps),
      1e-6);
}

// PLAN-104 AVBD articulated bridge: parent-endpoint command updates should also
// rebuild private prismatic velocity motor targets when the free slider axis is
// represented by a generated non-cardinal basis.
TEST(
    VariationalIntegration,
    AvbdPrismaticPointJointVelocityActuatorParentEndpointFollowsNonCardinalCommandUpdates)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto body = addFloatingBody(world, /*mass=*/2.0);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Prismatic;
  joint.parentLink = sx::detail::toRegistryEntity(body.getEntity());
  joint.childLink = entt::null;
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.3;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);

  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(0.7, -0.2, 0.5).normalized();
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.linearAxes = dvbd::avbdRigidJointAxesFromFreeAxis(sliderAxis);
  config.angularAxes = config.linearAxes;
  config.linearAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.angularAxisMask = dvbd::kAvbdRigidJointAllAxesMask;
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();

  double maxOrthogonalDrift = 0.0;
  double maxRotationError = 0.0;
  auto stepAndSliderPosition = [&]() {
    world.step();
    const Eigen::Isometry3d transform = body.getWorldTransform();
    const Eigen::Vector3d position = transform.translation();
    maxOrthogonalDrift = std::max(
        maxOrthogonalDrift,
        (position - position.dot(sliderAxis) * sliderAxis).norm());
    maxRotationError = std::max(
        maxRotationError,
        (transform.linear() - Eigen::Matrix3d::Identity()).norm());
    return position.dot(sliderAxis);
  };

  constexpr int forwardSteps = 10;
  double positionAfterForward = 0.0;
  for (int k = 0; k < forwardSteps; ++k) {
    positionAfterForward = stepAndSliderPosition();
  }

  registry.get<sx::comps::JointActuation>(jointEntity).commandVelocity
      = Eigen::VectorXd::Constant(1, -targetSpeed);

  constexpr int reverseSteps = 10;
  double positionAfterReverse = positionAfterForward;
  for (int k = 0; k < reverseSteps; ++k) {
    positionAfterReverse = stepAndSliderPosition();
  }

  EXPECT_LT(maxOrthogonalDrift, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
  EXPECT_NEAR(positionAfterForward, -targetSpeed * dt * forwardSteps, 1e-6);
  EXPECT_NEAR(
      positionAfterReverse,
      -targetSpeed * dt * (forwardSteps - reverseSteps),
      1e-6);
}

// PLAN-104 AVBD articulated bridge: the free-axis linear motor row also honors
// its effort bound instead of becoming an unbounded hard slider target whenever
// the limit is merely positive.
TEST(
    VariationalIntegration,
    AvbdPrismaticPointJointVelocityActuatorRespectsTinyForceLimit)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto body = addFloatingBody(world, /*mass=*/2.0);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Prismatic;
  joint.parentLink = entt::null;
  joint.childLink = sx::detail::toRegistryEntity(body.getEntity());
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, 0.3);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1e-9);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1e-9);

  const Eigen::Vector3d sliderAxis = Eigen::Vector3d::UnitX();
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.linearAxes = dvbd::avbdRigidJointAxesFromFreeAxis(sliderAxis);
  config.angularAxes = config.linearAxes;
  config.linearAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.angularAxisMask = dvbd::kAvbdRigidJointAllAxesMask;
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();

  constexpr int steps = 20;
  double maxOrthogonalDrift = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    const Eigen::Vector3d position = transform.translation();
    maxOrthogonalDrift = std::max(
        maxOrthogonalDrift,
        (position - position.dot(sliderAxis) * sliderAxis).norm());
    maxRotationError = std::max(
        maxRotationError,
        (transform.linear() - Eigen::Matrix3d::Identity()).norm());
  }

  const Eigen::Vector3d position = body.getWorldTransform().translation();
  EXPECT_LT(maxOrthogonalDrift, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
  EXPECT_LT(std::abs(position.dot(sliderAxis)), 1e-6);
}

// PLAN-104 AVBD articulated bridge: direct private finite-effort prismatic rows
// should stay bounded when the child endpoint uses a generated non-cardinal
// slider basis, not only the cardinal X basis.
TEST(
    VariationalIntegration,
    AvbdPrismaticPointJointVelocityActuatorRespectsTinyForceLimitOnNonCardinalAxis)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto body = addFloatingBody(world, /*mass=*/2.0);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Prismatic;
  joint.parentLink = entt::null;
  joint.childLink = sx::detail::toRegistryEntity(body.getEntity());
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, 0.3);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1e-9);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1e-9);

  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(0.5, -0.7, 0.4).normalized();
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.linearAxes = dvbd::avbdRigidJointAxesFromFreeAxis(sliderAxis);
  config.angularAxes = config.linearAxes;
  config.linearAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.angularAxisMask = dvbd::kAvbdRigidJointAllAxesMask;
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();

  constexpr int steps = 20;
  double maxOrthogonalDrift = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    const Eigen::Vector3d position = transform.translation();
    maxOrthogonalDrift = std::max(
        maxOrthogonalDrift,
        (position - position.dot(sliderAxis) * sliderAxis).norm());
    maxRotationError = std::max(
        maxRotationError,
        (transform.linear() - Eigen::Matrix3d::Identity()).norm());
  }

  const Eigen::Vector3d position = body.getWorldTransform().translation();
  EXPECT_LT(maxOrthogonalDrift, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
  EXPECT_LT(std::abs(position.dot(sliderAxis)), 1e-6);
}

// PLAN-104 AVBD articulated bridge: the bounded free-axis linear motor row must
// also honor tiny effort limits when the multibody link is the parent endpoint
// of a world-link private point-joint.
TEST(
    VariationalIntegration,
    AvbdPrismaticPointJointVelocityActuatorParentEndpointRespectsTinyForceLimit)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto body = addFloatingBody(world, /*mass=*/2.0);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Prismatic;
  joint.parentLink = sx::detail::toRegistryEntity(body.getEntity());
  joint.childLink = entt::null;
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, 0.3);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1e-9);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1e-9);

  const Eigen::Vector3d sliderAxis = Eigen::Vector3d::UnitX();
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.linearAxes = dvbd::avbdRigidJointAxesFromFreeAxis(sliderAxis);
  config.angularAxes = config.linearAxes;
  config.linearAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.angularAxisMask = dvbd::kAvbdRigidJointAllAxesMask;
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();

  constexpr int steps = 20;
  double maxOrthogonalDrift = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    const Eigen::Vector3d position = transform.translation();
    maxOrthogonalDrift = std::max(
        maxOrthogonalDrift,
        (position - position.dot(sliderAxis) * sliderAxis).norm());
    maxRotationError = std::max(
        maxRotationError,
        (transform.linear() - Eigen::Matrix3d::Identity()).norm());
  }

  const Eigen::Vector3d position = body.getWorldTransform().translation();
  EXPECT_LT(maxOrthogonalDrift, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
  EXPECT_LT(std::abs(position.dot(sliderAxis)), 1e-6);
}

// PLAN-104 AVBD articulated bridge: parent-endpoint finite-effort prismatic
// rows should also stay bounded when the free slider axis is represented by a
// generated non-cardinal basis.
TEST(
    VariationalIntegration,
    AvbdPrismaticPointJointVelocityActuatorParentEndpointRespectsTinyForceLimitOnNonCardinalAxis)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto body = addFloatingBody(world, /*mass=*/2.0);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Prismatic;
  joint.parentLink = sx::detail::toRegistryEntity(body.getEntity());
  joint.childLink = entt::null;
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, 0.3);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1e-9);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1e-9);

  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(0.3, 0.4, -0.9).normalized();
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.linearAxes = dvbd::avbdRigidJointAxesFromFreeAxis(sliderAxis);
  config.angularAxes = config.linearAxes;
  config.linearAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.angularAxisMask = dvbd::kAvbdRigidJointAllAxesMask;
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();

  constexpr int steps = 20;
  double maxOrthogonalDrift = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    const Eigen::Vector3d position = transform.translation();
    maxOrthogonalDrift = std::max(
        maxOrthogonalDrift,
        (position - position.dot(sliderAxis) * sliderAxis).norm());
    maxRotationError = std::max(
        maxRotationError,
        (transform.linear() - Eigen::Matrix3d::Identity()).norm());
  }

  const Eigen::Vector3d position = body.getWorldTransform().translation();
  EXPECT_LT(maxOrthogonalDrift, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
  EXPECT_LT(std::abs(position.dot(sliderAxis)), 1e-6);
}

// PLAN-104 AVBD articulated bridge: private prismatic configs should constrain
// all angular rows and only the two translation rows perpendicular to the free
// slider axis, leaving motion along that axis available.
TEST(
    VariationalIntegration,
    AvbdPrismaticPointJointConfigSolvesMaskedFloatingLinkEndpoint)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto body = addFloatingBody(world, /*mass=*/2.0);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Prismatic;
  joint.parentLink = sx::detail::toRegistryEntity(body.getEntity());
  joint.childLink = entt::null;

  const Eigen::Vector3d sliderAxis = Eigen::Vector3d::UnitX();
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.linearAxes = dvbd::avbdRigidJointAxesFromFreeAxis(sliderAxis);
  config.angularAxes = config.linearAxes;
  config.linearAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.angularAxisMask = dvbd::kAvbdRigidJointAllAxesMask;
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();

  double maxOrthogonalDrift = 0.0;
  double maxRotationError = 0.0;
  double maxSliderMotion = 0.0;
  for (int k = 0; k < 80; ++k) {
    body.applyForce(4.0 * sliderAxis);
    world.step();

    const Eigen::Isometry3d transform = body.getWorldTransform();
    const Eigen::Vector3d position = transform.translation();
    maxOrthogonalDrift = std::max(
        maxOrthogonalDrift,
        (position - position.dot(sliderAxis) * sliderAxis).norm());
    maxRotationError = std::max(
        maxRotationError,
        (transform.linear() - Eigen::Matrix3d::Identity()).norm());
    maxSliderMotion
        = std::max(maxSliderMotion, std::abs(position.dot(sliderAxis)));
  }

  EXPECT_LT(maxOrthogonalDrift, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
  EXPECT_GT(maxSliderMotion, 1e-3);
}

// Break-force row indexing also has to work for masked articulated AVBD rows:
// the revolute bridge contributes 3 linear rows plus 2 angular rows, not the
// fixed-joint 6-row shape.
TEST(
    VariationalIntegration,
    AvbdBreakableRevolutePointJointConfigMarksMaskedFloatingLinkEndpoint)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  auto body = addFloatingBody(world, /*mass=*/2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Revolute;
  joint.parentLink = sx::detail::toRegistryEntity(body.getEntity());
  joint.childLink = entt::null;
  joint.breakForce = 1e-18;

  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.angularAxes
      = dvbd::avbdRigidJointAxesFromFreeAxis(Eigen::Vector3d::UnitZ());
  config.angularAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();

  body.applyForce(4.0 * Eigen::Vector3d::UnitY());
  world.step();

  EXPECT_TRUE(registry.get<sx::comps::JointState>(jointEntity).broken);

  double maxPositionResidual = 0.0;
  for (int k = 0; k < 20; ++k) {
    body.applyForce(4.0 * Eigen::Vector3d::UnitY());
    world.step();
    maxPositionResidual = std::max(
        maxPositionResidual, body.getWorldTransform().translation().norm());
  }

  EXPECT_GT(maxPositionResidual, 1e-4);
}

// PLAN-104 AVBD articulated bridge: private revolute velocity point-joints
// should rebuild both the masked hard rows and the free-axis angular motor row
// after a break-force reset, mirroring the prismatic lifecycle coverage.
TEST(
    VariationalIntegration,
    AvbdBreakableRevoluteVelocityPointJointConfigResetReengagesMotorRows)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto body = addFloatingBody(world, /*mass=*/2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Revolute;
  joint.parentLink = entt::null;
  joint.childLink = sx::detail::toRegistryEntity(body.getEntity());
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.4;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);
  joint.breakForce = 1e-18;

  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.angularAxes
      = dvbd::avbdRigidJointAxesFromFreeAxis(Eigen::Vector3d::UnitZ());
  config.angularAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();
  world.step();

  auto& liveJointModel = registry.get<sx::comps::JointModel>(jointEntity);
  auto& liveJointState = registry.get<sx::comps::JointState>(jointEntity);
  ASSERT_TRUE(liveJointState.broken);
  auto yaw = [&]() {
    const Eigen::Matrix3d rotation = body.getWorldTransform().linear();
    return std::atan2(rotation(1, 0), rotation(0, 0));
  };
  EXPECT_NEAR(yaw(), targetSpeed * dt, 1e-6);

  liveJointState.broken = false;
  liveJointModel.breakForce = 1e6;

  world.step();

  const Eigen::Isometry3d transform = body.getWorldTransform();
  EXPECT_FALSE(liveJointState.broken);
  EXPECT_LT(transform.translation().norm(), 1e-6);
  EXPECT_LT(
      (transform.linear() * Eigen::Vector3d::UnitZ() - Eigen::Vector3d::UnitZ())
          .norm(),
      1e-6);
  EXPECT_NEAR(yaw(), targetSpeed * dt * 2.0, 1e-6);
}

// PLAN-104 AVBD articulated bridge: the in-memory reset path must also rebuild
// parent-endpoint private revolute velocity point-joints, whose free-axis motor
// has the opposite sign from the child-endpoint world-link case.
TEST(
    VariationalIntegration,
    AvbdBreakableRevoluteVelocityParentPointJointConfigResetReengagesMotorRows)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto body = addFloatingBody(world, /*mass=*/2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Revolute;
  joint.parentLink = sx::detail::toRegistryEntity(body.getEntity());
  joint.childLink = entt::null;
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.4;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);
  joint.breakForce = 1e-18;

  const Eigen::Vector3d hingeAxis = Eigen::Vector3d::UnitZ();
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.angularAxes = dvbd::avbdRigidJointAxesFromFreeAxis(hingeAxis);
  config.angularAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();
  world.step();

  auto& liveJointModel = registry.get<sx::comps::JointModel>(jointEntity);
  auto& liveJointState = registry.get<sx::comps::JointState>(jointEntity);
  ASSERT_TRUE(liveJointState.broken);
  auto yaw = [&]() {
    return signedRotationAroundAxis(
        body.getWorldTransform().linear(), hingeAxis);
  };
  EXPECT_NEAR(yaw(), -targetSpeed * dt, 1e-6);

  liveJointState.broken = false;
  liveJointModel.breakForce = 1e6;

  world.step();

  const Eigen::Isometry3d transform = body.getWorldTransform();
  EXPECT_FALSE(liveJointState.broken);
  EXPECT_LT(transform.translation().norm(), 1e-6);
  EXPECT_LT((transform.linear() * hingeAxis - hingeAxis).norm(), 1e-6);
  EXPECT_NEAR(yaw(), -targetSpeed * dt * 2.0, 1e-6);
}

// Break-force row indexing also covers the prismatic velocity-motor row: this
// bridge contributes 2 constrained linear rows, 1 free-axis motor row, and 3
// angular rows before marking the source joint broken.
TEST(
    VariationalIntegration,
    AvbdBreakablePrismaticVelocityPointJointConfigMarksMaskedFloatingLinkEndpoint)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto body = addFloatingBody(world, /*mass=*/2.0);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Prismatic;
  joint.parentLink = entt::null;
  joint.childLink = sx::detail::toRegistryEntity(body.getEntity());
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, 0.3);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);
  joint.breakForce = 1e-18;

  const Eigen::Vector3d sliderAxis = Eigen::Vector3d::UnitX();
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.linearAxes = dvbd::avbdRigidJointAxesFromFreeAxis(sliderAxis);
  config.angularAxes = config.linearAxes;
  config.linearAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.angularAxisMask = dvbd::kAvbdRigidJointAllAxesMask;
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();
  world.step();

  EXPECT_TRUE(registry.get<sx::comps::JointState>(jointEntity).broken);

  double maxOrthogonalDrift = 0.0;
  for (int k = 0; k < 20; ++k) {
    body.applyForce(4.0 * Eigen::Vector3d::UnitY());
    world.step();
    const Eigen::Vector3d position = body.getWorldTransform().translation();
    maxOrthogonalDrift = std::max(
        maxOrthogonalDrift,
        (position - position.dot(sliderAxis) * sliderAxis).norm());
  }

  EXPECT_GT(maxOrthogonalDrift, 1e-4);
}

// PLAN-104 AVBD articulated bridge: clearing a broken private prismatic
// velocity point-joint should let the next extraction rebuild both the masked
// hard rows and the free-axis motor row from the source joint.
TEST(
    VariationalIntegration,
    AvbdBreakablePrismaticVelocityPointJointConfigResetReengagesMotorRows)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto body = addFloatingBody(world, /*mass=*/2.0);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Prismatic;
  joint.parentLink = entt::null;
  joint.childLink = sx::detail::toRegistryEntity(body.getEntity());
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, 0.3);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);
  joint.breakForce = 1e-18;

  const Eigen::Vector3d sliderAxis = Eigen::Vector3d::UnitX();
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.linearAxes = dvbd::avbdRigidJointAxesFromFreeAxis(sliderAxis);
  config.angularAxes = config.linearAxes;
  config.linearAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.angularAxisMask = dvbd::kAvbdRigidJointAllAxesMask;
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();
  world.step();

  auto& liveJointModel = registry.get<sx::comps::JointModel>(jointEntity);
  auto& liveJointState = registry.get<sx::comps::JointState>(jointEntity);
  auto& liveJointActuation
      = registry.get<sx::comps::JointActuation>(jointEntity);
  EXPECT_TRUE(liveJointState.broken);

  double maxBrokenOrthogonalDrift = 0.0;
  for (int k = 0; k < 20; ++k) {
    body.applyForce(4.0 * Eigen::Vector3d::UnitY());
    world.step();
    const Eigen::Vector3d position = body.getWorldTransform().translation();
    maxBrokenOrthogonalDrift = std::max(
        maxBrokenOrthogonalDrift,
        (position - position.dot(sliderAxis) * sliderAxis).norm());
  }
  EXPECT_GT(maxBrokenOrthogonalDrift, 1e-4);

  const double sliderPositionBeforeReset
      = body.getWorldTransform().translation().dot(sliderAxis);
  liveJointState.broken = false;
  liveJointModel.breakForce = 1e6;

  double maxResetOrthogonalDrift = 0.0;
  double maxResetRotationError = 0.0;
  constexpr int resetSteps = 10;
  for (int k = 0; k < resetSteps; ++k) {
    world.step();
    const Eigen::Isometry3d transform = body.getWorldTransform();
    const Eigen::Vector3d position = transform.translation();
    maxResetOrthogonalDrift = std::max(
        maxResetOrthogonalDrift,
        (position - position.dot(sliderAxis) * sliderAxis).norm());
    maxResetRotationError = std::max(
        maxResetRotationError,
        (transform.linear() - Eigen::Matrix3d::Identity()).norm());
  }

  const double sliderPositionAfterReset
      = body.getWorldTransform().translation().dot(sliderAxis);
  EXPECT_FALSE(liveJointState.broken);
  EXPECT_LT(maxResetOrthogonalDrift, 1e-6);
  EXPECT_LT(maxResetRotationError, 1e-6);
  EXPECT_NEAR(
      sliderPositionAfterReset - sliderPositionBeforeReset,
      liveJointActuation.commandVelocity[0] * dt * resetSteps,
      1e-6);
}

// PLAN-104 AVBD articulated bridge: the in-memory reset path must also rebuild
// parent-endpoint private prismatic velocity point-joints, whose free-axis
// motor has the opposite sign from the child-endpoint world-link case.
TEST(
    VariationalIntegration,
    AvbdBreakablePrismaticVelocityParentPointJointConfigResetReengagesMotorRows)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto body = addFloatingBody(world, /*mass=*/2.0);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Prismatic;
  joint.parentLink = sx::detail::toRegistryEntity(body.getEntity());
  joint.childLink = entt::null;
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, 0.3);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);
  joint.breakForce = 1e-18;

  const Eigen::Vector3d sliderAxis = Eigen::Vector3d::UnitX();
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.linearAxes = dvbd::avbdRigidJointAxesFromFreeAxis(sliderAxis);
  config.angularAxes = config.linearAxes;
  config.linearAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.angularAxisMask = dvbd::kAvbdRigidJointAllAxesMask;
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();
  world.step();

  auto& liveJointModel = registry.get<sx::comps::JointModel>(jointEntity);
  auto& liveJointState = registry.get<sx::comps::JointState>(jointEntity);
  auto& liveJointActuation
      = registry.get<sx::comps::JointActuation>(jointEntity);
  EXPECT_TRUE(liveJointState.broken);

  double maxBrokenOrthogonalDrift = 0.0;
  for (int k = 0; k < 20; ++k) {
    body.applyForce(4.0 * Eigen::Vector3d::UnitY());
    world.step();
    const Eigen::Vector3d position = body.getWorldTransform().translation();
    maxBrokenOrthogonalDrift = std::max(
        maxBrokenOrthogonalDrift,
        (position - position.dot(sliderAxis) * sliderAxis).norm());
  }
  EXPECT_GT(maxBrokenOrthogonalDrift, 1e-4);

  const double sliderPositionBeforeReset
      = body.getWorldTransform().translation().dot(sliderAxis);
  liveJointState.broken = false;
  liveJointModel.breakForce = 1e6;

  double maxResetOrthogonalDrift = 0.0;
  double maxResetRotationError = 0.0;
  constexpr int resetSteps = 10;
  for (int k = 0; k < resetSteps; ++k) {
    world.step();
    const Eigen::Isometry3d transform = body.getWorldTransform();
    const Eigen::Vector3d position = transform.translation();
    maxResetOrthogonalDrift = std::max(
        maxResetOrthogonalDrift,
        (position - position.dot(sliderAxis) * sliderAxis).norm());
    maxResetRotationError = std::max(
        maxResetRotationError,
        (transform.linear() - Eigen::Matrix3d::Identity()).norm());
  }

  const double sliderPositionAfterReset
      = body.getWorldTransform().translation().dot(sliderAxis);
  EXPECT_FALSE(liveJointState.broken);
  EXPECT_LT(maxResetOrthogonalDrift, 1e-6);
  EXPECT_LT(maxResetRotationError, 1e-6);
  EXPECT_NEAR(
      sliderPositionAfterReset - sliderPositionBeforeReset,
      -liveJointActuation.commandVelocity[0] * dt * resetSteps,
      1e-6);
}

// PLAN-104 AVBD articulated bridge: direct private revolute velocity
// point-joint configs must persist their non-cardinal masked hard-row basis and
// free-axis motor state across simulation-mode binary save/load while broken,
// then re-engage on reset.
TEST(
    VariationalIntegration,
    AvbdBreakableRevoluteVelocityPointJointConfigSurvivesSaveLoadAndReset)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto body = addFloatingBody(world, /*mass=*/2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  registry.emplace<sx::comps::Name>(
      jointEntity, "serialized_private_breakable_hinge");
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.name = "serialized_private_breakable_hinge";
  joint.type = sx::comps::JointType::Revolute;
  joint.parentLink = entt::null;
  joint.childLink = sx::detail::toRegistryEntity(body.getEntity());
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.4;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);
  joint.breakForce = 1e-18;

  const Eigen::Vector3d hingeAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.angularAxes = dvbd::avbdRigidJointAxesFromFreeAxis(hingeAxis);
  config.angularAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();
  world.step();

  ASSERT_TRUE(registry.get<sx::comps::JointState>(jointEntity).broken);
  EXPECT_NEAR(
      signedRotationAroundAxis(body.getWorldTransform().linear(), hingeAxis),
      targetSpeed * dt,
      1e-6);
  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  auto restoredRobot = restored.getMultibody("floater");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredBody = restoredRobot->getLink("body");
  ASSERT_TRUE(restoredBody.has_value());

  auto& restoredRegistry = dart::simulation::detail::registryOf(restored);
  entt::entity restoredJointEntity = entt::null;
  auto jointView = restoredRegistry.view<sx::comps::JointModel>();
  for (const entt::entity entity : jointView) {
    if (jointView.get<sx::comps::JointModel>(entity).name
        == "serialized_private_breakable_hinge") {
      restoredJointEntity = entity;
      break;
    }
  }
  ASSERT_TRUE(restoredJointEntity != entt::null);
  auto& restoredJointModel
      = restoredRegistry.get<sx::comps::JointModel>(restoredJointEntity);
  auto& restoredJointState
      = restoredRegistry.get<sx::comps::JointState>(restoredJointEntity);
  auto& restoredJointActuation
      = restoredRegistry.get<sx::comps::JointActuation>(restoredJointEntity);
  ASSERT_TRUE(restoredJointState.broken);
  EXPECT_EQ(restoredJointModel.type, sx::comps::JointType::Revolute);
  EXPECT_TRUE(restoredJointModel.parentLink == entt::null);
  EXPECT_EQ(
      restoredJointModel.childLink,
      sx::detail::toRegistryEntity(restoredBody->getEntity()));
  EXPECT_EQ(
      restoredJointActuation.actuatorType, sx::comps::ActuatorType::Velocity);
  ASSERT_EQ(restoredJointActuation.commandVelocity.size(), 1);
  EXPECT_DOUBLE_EQ(restoredJointActuation.commandVelocity[0], targetSpeed);
  ASSERT_EQ(restoredJointModel.limits.effortLower.size(), 1);
  ASSERT_EQ(restoredJointModel.limits.effortUpper.size(), 1);
  EXPECT_DOUBLE_EQ(restoredJointModel.limits.effortLower[0], -1000.0);
  EXPECT_DOUBLE_EQ(restoredJointModel.limits.effortUpper[0], 1000.0);
  ASSERT_TRUE(restoredRegistry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(
      restoredJointEntity));
  const auto& restoredConfig
      = restoredRegistry.get<dvbd::AvbdRigidWorldPointJointConfig>(
          restoredJointEntity);
  EXPECT_LT(
      (restoredConfig.localAnchorA - Eigen::Vector3d::Zero()).norm(), 1e-12);
  EXPECT_LT(
      (restoredConfig.localAnchorB - Eigen::Vector3d::Zero()).norm(), 1e-12);
  EXPECT_EQ(restoredConfig.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(
      restoredConfig.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_NEAR(restoredConfig.angularAxes.col(2).dot(hingeAxis), 1.0, 1e-12);

  double maxBrokenPositionResidual = 0.0;
  for (int k = 0; k < 20; ++k) {
    restoredBody->applyForce(4.0 * Eigen::Vector3d::UnitY());
    restored.step();
    maxBrokenPositionResidual = std::max(
        maxBrokenPositionResidual,
        restoredBody->getWorldTransform().translation().norm());
  }
  ASSERT_GT(maxBrokenPositionResidual, 1e-4);

  const double hingeBeforeReset = signedRotationAroundAxis(
      restoredBody->getWorldTransform().linear(), hingeAxis);
  restoredBody->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredJointActuation.commandVelocity
      = Eigen::VectorXd::Constant(1, -targetSpeed);
  restoredJointModel.breakForce = 1e6;
  restoredJointState.broken = false;

  constexpr int resetSteps = 10;
  for (int k = 0; k < resetSteps; ++k) {
    restored.step();
  }

  ASSERT_TRUE(restoredRegistry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(
      restoredJointEntity));
  const Eigen::Isometry3d resetTransform = restoredBody->getWorldTransform();
  EXPECT_FALSE(restoredJointState.broken);
  EXPECT_LT(resetTransform.translation().norm(), 2e-3);
  EXPECT_LT((resetTransform.linear() * hingeAxis - hingeAxis).norm(), 2e-3);
  EXPECT_NEAR(
      signedRotationAroundAxis(resetTransform.linear(), hingeAxis)
          - hingeBeforeReset,
      -targetSpeed * dt * static_cast<double>(resetSteps),
      2e-3);
}

// PLAN-104 AVBD articulated bridge: direct private revolute velocity
// point-joint configs also need persistent non-cardinal break/reset coverage
// when the multibody link is the parent endpoint of a world-link joint. This
// exercises the opposite endpoint polarity from the child-link case above.
TEST(
    VariationalIntegration,
    AvbdBreakableRevoluteVelocityParentPointJointConfigSurvivesSaveLoadAndReset)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto body = addFloatingBody(world, /*mass=*/2.0);
  body.setInertia(Eigen::Vector3d(0.2, 0.2, 0.3).asDiagonal());

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  registry.emplace<sx::comps::Name>(
      jointEntity, "serialized_private_parent_breakable_hinge");
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.name = "serialized_private_parent_breakable_hinge";
  joint.type = sx::comps::JointType::Revolute;
  joint.parentLink = sx::detail::toRegistryEntity(body.getEntity());
  joint.childLink = entt::null;
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.4;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);
  joint.breakForce = 1e-18;

  const Eigen::Vector3d hingeAxis
      = Eigen::Vector3d(-2.0, 1.0, 3.0).normalized();
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.angularAxes = dvbd::avbdRigidJointAxesFromFreeAxis(hingeAxis);
  config.angularAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();
  world.step();

  ASSERT_TRUE(registry.get<sx::comps::JointState>(jointEntity).broken);
  EXPECT_NEAR(
      signedRotationAroundAxis(body.getWorldTransform().linear(), hingeAxis),
      -targetSpeed * dt,
      1e-6);
  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  auto restoredRobot = restored.getMultibody("floater");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredBody = restoredRobot->getLink("body");
  ASSERT_TRUE(restoredBody.has_value());

  auto& restoredRegistry = dart::simulation::detail::registryOf(restored);
  entt::entity restoredJointEntity = entt::null;
  auto jointView = restoredRegistry.view<sx::comps::JointModel>();
  for (const entt::entity entity : jointView) {
    if (jointView.get<sx::comps::JointModel>(entity).name
        == "serialized_private_parent_breakable_hinge") {
      restoredJointEntity = entity;
      break;
    }
  }
  ASSERT_TRUE(restoredJointEntity != entt::null);
  auto& restoredJointModel
      = restoredRegistry.get<sx::comps::JointModel>(restoredJointEntity);
  auto& restoredJointState
      = restoredRegistry.get<sx::comps::JointState>(restoredJointEntity);
  auto& restoredJointActuation
      = restoredRegistry.get<sx::comps::JointActuation>(restoredJointEntity);
  ASSERT_TRUE(restoredJointState.broken);
  EXPECT_EQ(restoredJointModel.type, sx::comps::JointType::Revolute);
  EXPECT_EQ(
      restoredJointModel.parentLink,
      sx::detail::toRegistryEntity(restoredBody->getEntity()));
  EXPECT_TRUE(restoredJointModel.childLink == entt::null);
  EXPECT_EQ(
      restoredJointActuation.actuatorType, sx::comps::ActuatorType::Velocity);
  ASSERT_EQ(restoredJointActuation.commandVelocity.size(), 1);
  EXPECT_DOUBLE_EQ(restoredJointActuation.commandVelocity[0], targetSpeed);
  ASSERT_EQ(restoredJointModel.limits.effortLower.size(), 1);
  ASSERT_EQ(restoredJointModel.limits.effortUpper.size(), 1);
  EXPECT_DOUBLE_EQ(restoredJointModel.limits.effortLower[0], -1000.0);
  EXPECT_DOUBLE_EQ(restoredJointModel.limits.effortUpper[0], 1000.0);
  ASSERT_TRUE(restoredRegistry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(
      restoredJointEntity));
  const auto& restoredConfig
      = restoredRegistry.get<dvbd::AvbdRigidWorldPointJointConfig>(
          restoredJointEntity);
  EXPECT_LT(
      (restoredConfig.localAnchorA - Eigen::Vector3d::Zero()).norm(), 1e-12);
  EXPECT_LT(
      (restoredConfig.localAnchorB - Eigen::Vector3d::Zero()).norm(), 1e-12);
  EXPECT_EQ(restoredConfig.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(
      restoredConfig.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_NEAR(restoredConfig.angularAxes.col(2).dot(hingeAxis), 1.0, 1e-12);

  double maxBrokenPositionResidual = 0.0;
  for (int k = 0; k < 20; ++k) {
    restoredBody->applyForce(4.0 * Eigen::Vector3d::UnitY());
    restored.step();
    maxBrokenPositionResidual = std::max(
        maxBrokenPositionResidual,
        restoredBody->getWorldTransform().translation().norm());
  }
  ASSERT_GT(maxBrokenPositionResidual, 1e-4);

  const double hingeBeforeReset = signedRotationAroundAxis(
      restoredBody->getWorldTransform().linear(), hingeAxis);
  restoredBody->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredJointActuation.commandVelocity
      = Eigen::VectorXd::Constant(1, -targetSpeed);
  restoredJointModel.breakForce = 1e6;
  restoredJointState.broken = false;

  constexpr int resetSteps = 10;
  for (int k = 0; k < resetSteps; ++k) {
    restored.step();
  }

  ASSERT_TRUE(restoredRegistry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(
      restoredJointEntity));
  const Eigen::Isometry3d resetTransform = restoredBody->getWorldTransform();
  EXPECT_FALSE(restoredJointState.broken);
  EXPECT_LT(resetTransform.translation().norm(), 2e-3);
  EXPECT_LT((resetTransform.linear() * hingeAxis - hingeAxis).norm(), 2e-3);
  EXPECT_NEAR(
      signedRotationAroundAxis(resetTransform.linear(), hingeAxis)
          - hingeBeforeReset,
      targetSpeed * dt * static_cast<double>(resetSteps),
      2e-3);
}

// PLAN-104 AVBD articulated bridge: direct private prismatic velocity
// point-joint configs likewise need binary persistence for their non-cardinal
// masked hard rows and free-axis linear motor before a broken-state reset.
TEST(
    VariationalIntegration,
    AvbdBreakablePrismaticVelocityPointJointConfigSurvivesSaveLoadAndReset)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto body = addFloatingBody(world, /*mass=*/2.0);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  registry.emplace<sx::comps::Name>(
      jointEntity, "serialized_private_breakable_slider");
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.name = "serialized_private_breakable_slider";
  joint.type = sx::comps::JointType::Prismatic;
  joint.parentLink = entt::null;
  joint.childLink = sx::detail::toRegistryEntity(body.getEntity());
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.3;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);
  joint.breakForce = 1e-18;

  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(1.0, 2.0, 0.5).normalized();
  const Eigen::Vector3d lateralForce
      = (Eigen::Vector3d::UnitY()
         - Eigen::Vector3d::UnitY().dot(sliderAxis) * sliderAxis)
            .normalized();
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.linearAxes = dvbd::avbdRigidJointAxesFromFreeAxis(sliderAxis);
  config.angularAxes = config.linearAxes;
  config.linearAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.angularAxisMask = dvbd::kAvbdRigidJointAllAxesMask;
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();
  world.step();

  ASSERT_TRUE(registry.get<sx::comps::JointState>(jointEntity).broken);
  EXPECT_NEAR(
      body.getWorldTransform().translation().dot(sliderAxis),
      targetSpeed * dt,
      1e-6);
  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  auto restoredRobot = restored.getMultibody("floater");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredBody = restoredRobot->getLink("body");
  ASSERT_TRUE(restoredBody.has_value());

  auto& restoredRegistry = dart::simulation::detail::registryOf(restored);
  entt::entity restoredJointEntity = entt::null;
  auto jointView = restoredRegistry.view<sx::comps::JointModel>();
  for (const entt::entity entity : jointView) {
    if (jointView.get<sx::comps::JointModel>(entity).name
        == "serialized_private_breakable_slider") {
      restoredJointEntity = entity;
      break;
    }
  }
  ASSERT_TRUE(restoredJointEntity != entt::null);
  auto& restoredJointModel
      = restoredRegistry.get<sx::comps::JointModel>(restoredJointEntity);
  auto& restoredJointState
      = restoredRegistry.get<sx::comps::JointState>(restoredJointEntity);
  auto& restoredJointActuation
      = restoredRegistry.get<sx::comps::JointActuation>(restoredJointEntity);
  ASSERT_TRUE(restoredJointState.broken);
  EXPECT_EQ(restoredJointModel.type, sx::comps::JointType::Prismatic);
  EXPECT_TRUE(restoredJointModel.parentLink == entt::null);
  EXPECT_EQ(
      restoredJointModel.childLink,
      sx::detail::toRegistryEntity(restoredBody->getEntity()));
  EXPECT_EQ(
      restoredJointActuation.actuatorType, sx::comps::ActuatorType::Velocity);
  ASSERT_EQ(restoredJointActuation.commandVelocity.size(), 1);
  EXPECT_DOUBLE_EQ(restoredJointActuation.commandVelocity[0], targetSpeed);
  ASSERT_EQ(restoredJointModel.limits.effortLower.size(), 1);
  ASSERT_EQ(restoredJointModel.limits.effortUpper.size(), 1);
  EXPECT_DOUBLE_EQ(restoredJointModel.limits.effortLower[0], -1000.0);
  EXPECT_DOUBLE_EQ(restoredJointModel.limits.effortUpper[0], 1000.0);
  ASSERT_TRUE(restoredRegistry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(
      restoredJointEntity));
  const auto& restoredConfig
      = restoredRegistry.get<dvbd::AvbdRigidWorldPointJointConfig>(
          restoredJointEntity);
  EXPECT_LT(
      (restoredConfig.localAnchorA - Eigen::Vector3d::Zero()).norm(), 1e-12);
  EXPECT_LT(
      (restoredConfig.localAnchorB - Eigen::Vector3d::Zero()).norm(), 1e-12);
  EXPECT_EQ(
      restoredConfig.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(restoredConfig.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(restoredConfig.linearAxes.col(2).dot(sliderAxis), 1.0, 1e-12);

  const auto sliderPosition = [&]() {
    return restoredBody->getWorldTransform().translation().dot(sliderAxis);
  };
  const auto orthogonalResidual = [&]() {
    const Eigen::Vector3d position
        = restoredBody->getWorldTransform().translation();
    return (position - position.dot(sliderAxis) * sliderAxis).norm();
  };
  double maxBrokenOrthogonalDrift = 0.0;
  for (int k = 0; k < 20; ++k) {
    restoredBody->applyForce(4.0 * lateralForce);
    restored.step();
    maxBrokenOrthogonalDrift
        = std::max(maxBrokenOrthogonalDrift, orthogonalResidual());
  }
  ASSERT_GT(maxBrokenOrthogonalDrift, 1e-4);

  const double sliderBeforeReset = sliderPosition();
  restoredBody->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredJointActuation.commandVelocity
      = Eigen::VectorXd::Constant(1, -targetSpeed);
  restoredJointModel.breakForce = 1e6;
  restoredJointState.broken = false;

  constexpr int resetSteps = 10;
  double maxResetOrthogonalDrift = 0.0;
  double maxResetRotationError = 0.0;
  for (int k = 0; k < resetSteps; ++k) {
    restored.step();
    maxResetOrthogonalDrift
        = std::max(maxResetOrthogonalDrift, orthogonalResidual());
    maxResetRotationError = std::max(
        maxResetRotationError,
        (restoredBody->getWorldTransform().linear()
         - Eigen::Matrix3d::Identity())
            .norm());
  }

  ASSERT_TRUE(restoredRegistry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(
      restoredJointEntity));
  EXPECT_FALSE(restoredJointState.broken);
  EXPECT_LT(maxResetOrthogonalDrift, 2e-3);
  EXPECT_LT(maxResetRotationError, 1e-6);
  EXPECT_NEAR(
      sliderPosition() - sliderBeforeReset,
      -targetSpeed * dt * static_cast<double>(resetSteps),
      2e-3);
}

// PLAN-104 AVBD articulated bridge: direct private prismatic velocity
// point-joint configs also need persistent non-cardinal break/reset coverage
// when the multibody link is the parent endpoint of a world-link joint. This
// exercises the opposite endpoint polarity from the child-link case above.
TEST(
    VariationalIntegration,
    AvbdBreakablePrismaticVelocityParentPointJointConfigSurvivesSaveLoadAndReset)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  auto body = addFloatingBody(world, /*mass=*/2.0);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  registry.emplace<sx::comps::Name>(
      jointEntity, "serialized_private_parent_breakable_slider");
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.name = "serialized_private_parent_breakable_slider";
  joint.type = sx::comps::JointType::Prismatic;
  joint.parentLink = sx::detail::toRegistryEntity(body.getEntity());
  joint.childLink = entt::null;
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.3;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);
  joint.breakForce = 1e-18;

  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(0.5, -1.0, 2.0).normalized();
  const Eigen::Vector3d lateralForce
      = (Eigen::Vector3d::UnitY()
         - Eigen::Vector3d::UnitY().dot(sliderAxis) * sliderAxis)
            .normalized();
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = Eigen::Vector3d::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.linearAxes = dvbd::avbdRigidJointAxesFromFreeAxis(sliderAxis);
  config.angularAxes = config.linearAxes;
  config.linearAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.angularAxisMask = dvbd::kAvbdRigidJointAllAxesMask;
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  world.enterSimulationMode();
  world.step();

  ASSERT_TRUE(registry.get<sx::comps::JointState>(jointEntity).broken);
  EXPECT_NEAR(
      body.getWorldTransform().translation().dot(sliderAxis),
      -targetSpeed * dt,
      1e-6);
  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  auto restoredRobot = restored.getMultibody("floater");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredBody = restoredRobot->getLink("body");
  ASSERT_TRUE(restoredBody.has_value());

  auto& restoredRegistry = dart::simulation::detail::registryOf(restored);
  entt::entity restoredJointEntity = entt::null;
  auto jointView = restoredRegistry.view<sx::comps::JointModel>();
  for (const entt::entity entity : jointView) {
    if (jointView.get<sx::comps::JointModel>(entity).name
        == "serialized_private_parent_breakable_slider") {
      restoredJointEntity = entity;
      break;
    }
  }
  ASSERT_TRUE(restoredJointEntity != entt::null);
  auto& restoredJointModel
      = restoredRegistry.get<sx::comps::JointModel>(restoredJointEntity);
  auto& restoredJointState
      = restoredRegistry.get<sx::comps::JointState>(restoredJointEntity);
  auto& restoredJointActuation
      = restoredRegistry.get<sx::comps::JointActuation>(restoredJointEntity);
  ASSERT_TRUE(restoredJointState.broken);
  EXPECT_EQ(restoredJointModel.type, sx::comps::JointType::Prismatic);
  EXPECT_EQ(
      restoredJointModel.parentLink,
      sx::detail::toRegistryEntity(restoredBody->getEntity()));
  EXPECT_TRUE(restoredJointModel.childLink == entt::null);
  EXPECT_EQ(
      restoredJointActuation.actuatorType, sx::comps::ActuatorType::Velocity);
  ASSERT_EQ(restoredJointActuation.commandVelocity.size(), 1);
  EXPECT_DOUBLE_EQ(restoredJointActuation.commandVelocity[0], targetSpeed);
  ASSERT_EQ(restoredJointModel.limits.effortLower.size(), 1);
  ASSERT_EQ(restoredJointModel.limits.effortUpper.size(), 1);
  EXPECT_DOUBLE_EQ(restoredJointModel.limits.effortLower[0], -1000.0);
  EXPECT_DOUBLE_EQ(restoredJointModel.limits.effortUpper[0], 1000.0);
  ASSERT_TRUE(restoredRegistry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(
      restoredJointEntity));
  const auto& restoredConfig
      = restoredRegistry.get<dvbd::AvbdRigidWorldPointJointConfig>(
          restoredJointEntity);
  EXPECT_LT(
      (restoredConfig.localAnchorA - Eigen::Vector3d::Zero()).norm(), 1e-12);
  EXPECT_LT(
      (restoredConfig.localAnchorB - Eigen::Vector3d::Zero()).norm(), 1e-12);
  EXPECT_EQ(
      restoredConfig.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(restoredConfig.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(restoredConfig.linearAxes.col(2).dot(sliderAxis), 1.0, 1e-12);

  const auto sliderPosition = [&]() {
    return restoredBody->getWorldTransform().translation().dot(sliderAxis);
  };
  const auto orthogonalResidual = [&]() {
    const Eigen::Vector3d position
        = restoredBody->getWorldTransform().translation();
    return (position - position.dot(sliderAxis) * sliderAxis).norm();
  };
  double maxBrokenOrthogonalDrift = 0.0;
  for (int k = 0; k < 20; ++k) {
    restoredBody->applyForce(4.0 * lateralForce);
    restored.step();
    maxBrokenOrthogonalDrift
        = std::max(maxBrokenOrthogonalDrift, orthogonalResidual());
  }
  ASSERT_GT(maxBrokenOrthogonalDrift, 1e-4);

  const double sliderBeforeReset = sliderPosition();
  restoredBody->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredJointActuation.commandVelocity
      = Eigen::VectorXd::Constant(1, -targetSpeed);
  restoredJointModel.breakForce = 1e6;
  restoredJointState.broken = false;

  constexpr int resetSteps = 10;
  double maxResetOrthogonalDrift = 0.0;
  double maxResetRotationError = 0.0;
  for (int k = 0; k < resetSteps; ++k) {
    restored.step();
    maxResetOrthogonalDrift
        = std::max(maxResetOrthogonalDrift, orthogonalResidual());
    maxResetRotationError = std::max(
        maxResetRotationError,
        (restoredBody->getWorldTransform().linear()
         - Eigen::Matrix3d::Identity())
            .norm());
  }

  ASSERT_TRUE(restoredRegistry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(
      restoredJointEntity));
  EXPECT_FALSE(restoredJointState.broken);
  EXPECT_LT(maxResetOrthogonalDrift, 2e-3);
  EXPECT_LT(maxResetRotationError, 1e-6);
  EXPECT_NEAR(
      sliderPosition() - sliderBeforeReset,
      targetSpeed * dt * static_cast<double>(resetSteps),
      2e-3);
}

// PLAN-104 AVBD articulated bridge: direct private fixed point-joint configs
// also need reset coverage between two movable same-multibody endpoints.
// Resetting the broken joint should re-enable both the linear anchor rows and
// the relative-orientation rows from the persistent private config.
TEST(
    VariationalIntegration,
    AvbdBreakableFixedPointJointConfigResetReengagesMovablePair)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink = sx::detail::toRegistryEntity(pair.parent.getEntity());
  joint.childLink = sx::detail::toRegistryEntity(pair.child.getEntity());
  joint.breakForce = 1e-18;

  const Eigen::Vector3d parentAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d childAnchor(-0.1, 0.1, 0.0);
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = parentAnchor;
  config.localAnchorB = childAnchor;
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.linearAxisMask = dvbd::kAvbdRigidJointAllAxesMask;
  config.angularAxisMask = dvbd::kAvbdRigidJointAllAxesMask;
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  const auto anchorResidual = [&]() {
    const Eigen::Vector3d parentAnchorWorld
        = pair.parent.getWorldTransform() * parentAnchor;
    const Eigen::Vector3d childAnchorWorld
        = pair.child.getWorldTransform() * childAnchor;
    return (childAnchorWorld - parentAnchorWorld).norm();
  };
  const auto relativeRotationError = [&]() {
    return (pair.parent.getWorldTransform().linear().transpose()
                * pair.child.getWorldTransform().linear()
            - Eigen::Matrix3d::Identity())
        .norm();
  };
  const auto applyOpposingOffsetForces = [&]() {
    pair.parent.applyForce(
        -4.0 * Eigen::Vector3d::UnitY(),
        parentAnchor + 0.4 * Eigen::Vector3d::UnitX());
    pair.child.applyForce(
        4.0 * Eigen::Vector3d::UnitY(),
        childAnchor - 0.4 * Eigen::Vector3d::UnitX());
  };

  world.enterSimulationMode();
  applyOpposingOffsetForces();
  world.step();

  auto& liveJointModel = registry.get<sx::comps::JointModel>(jointEntity);
  auto& liveJointState = registry.get<sx::comps::JointState>(jointEntity);
  ASSERT_TRUE(liveJointState.broken);
  EXPECT_LT(anchorResidual(), 1e-6);
  EXPECT_LT(relativeRotationError(), 1e-6);

  double maxBrokenAnchorResidual = 0.0;
  double maxBrokenRotationError = 0.0;
  for (int k = 0; k < 20; ++k) {
    applyOpposingOffsetForces();
    world.step();
    maxBrokenAnchorResidual
        = std::max(maxBrokenAnchorResidual, anchorResidual());
    maxBrokenRotationError
        = std::max(maxBrokenRotationError, relativeRotationError());
  }
  ASSERT_GT(maxBrokenAnchorResidual, 1e-4);
  ASSERT_GT(maxBrokenRotationError, 1e-4);

  pair.parent.getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  pair.child.getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  liveJointModel.breakForce = 1e6;
  liveJointState.broken = false;

  world.step();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& resetConfig
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(resetConfig.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(resetConfig.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_FALSE(liveJointState.broken);
  EXPECT_LT(anchorResidual(), 1e-6);
  EXPECT_LT(relativeRotationError(), 1e-6);
}

// PLAN-104 AVBD articulated bridge: private revolute velocity point-joint
// configs also cover two movable same-multibody endpoints, not only world-link
// endpoints. After a break/reset round trip, the next extraction samples the
// updated command and rebuilds both the masked hard rows and free-axis motor.
TEST(
    VariationalIntegration,
    AvbdBreakableRevoluteVelocityPointJointConfigResetReengagesMovablePair)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Revolute;
  joint.parentLink = sx::detail::toRegistryEntity(pair.parent.getEntity());
  joint.childLink = sx::detail::toRegistryEntity(pair.child.getEntity());
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.4;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);
  joint.breakForce = 1e-18;

  const Eigen::Vector3d hingeAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  const Eigen::Vector3d parentAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d childAnchor(-0.1, 0.1, 0.0);
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = parentAnchor;
  config.localAnchorB = childAnchor;
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.angularAxes = dvbd::avbdRigidJointAxesFromFreeAxis(hingeAxis);
  config.angularAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  const auto parentAnchorWorld = [&]() {
    return pair.parent.getWorldTransform() * parentAnchor;
  };
  const auto childAnchorWorld = [&]() {
    return pair.child.getWorldTransform() * childAnchor;
  };
  const auto anchorResidual = [&]() {
    return (childAnchorWorld() - parentAnchorWorld()).norm();
  };
  const auto axisTilt = [&]() {
    return (pair.parent.getWorldTransform().linear() * hingeAxis
            - pair.child.getWorldTransform().linear() * hingeAxis)
        .norm();
  };
  const auto relativeHingePosition = [&]() {
    const Eigen::Matrix3d relativeRotation
        = pair.parent.getWorldTransform().linear().transpose()
          * pair.child.getWorldTransform().linear();
    return signedRotationAroundAxis(relativeRotation, hingeAxis);
  };

  world.enterSimulationMode();
  world.step();

  auto& liveJointModel = registry.get<sx::comps::JointModel>(jointEntity);
  auto& liveJointState = registry.get<sx::comps::JointState>(jointEntity);
  ASSERT_TRUE(liveJointState.broken);
  const double firstStepHingePosition = relativeHingePosition();
  EXPECT_NEAR(firstStepHingePosition, targetSpeed * dt, 1e-6);
  EXPECT_LT(anchorResidual(), 1e-6);
  EXPECT_LT(axisTilt(), 1e-6);

  double maxBrokenAnchorResidual = 0.0;
  for (int k = 0; k < 20; ++k) {
    pair.parent.applyForce(
        -4.0 * Eigen::Vector3d::UnitY(),
        parentAnchor + 0.3 * Eigen::Vector3d::UnitX());
    pair.child.applyForce(
        4.0 * Eigen::Vector3d::UnitY(),
        childAnchor - 0.3 * Eigen::Vector3d::UnitX());
    world.step();
    maxBrokenAnchorResidual
        = std::max(maxBrokenAnchorResidual, anchorResidual());
  }
  ASSERT_GT(maxBrokenAnchorResidual, 1e-4);

  const double hingeBeforeReset = relativeHingePosition();
  pair.parent.getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  pair.child.getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  registry.get<sx::comps::JointActuation>(jointEntity).commandVelocity
      = Eigen::VectorXd::Constant(1, -targetSpeed);
  liveJointModel.breakForce = 1e6;
  liveJointState.broken = false;

  constexpr int resetSteps = 10;
  for (int k = 0; k < resetSteps; ++k) {
    world.step();
  }

  EXPECT_FALSE(liveJointState.broken);
  EXPECT_LT(anchorResidual(), 1e-6);
  EXPECT_LT(axisTilt(), 2e-3);
  EXPECT_NEAR(
      relativeHingePosition() - hingeBeforeReset,
      -targetSpeed * dt * static_cast<double>(resetSteps),
      2e-3);
}

// PLAN-104 AVBD articulated bridge: direct private prismatic velocity
// point-joint configs also cover two movable same-multibody endpoints. After a
// break/reset round trip, the next extraction samples the updated command and
// rebuilds both the masked hard rows and free-axis motor for a non-cardinal
// slider basis.
TEST(
    VariationalIntegration,
    AvbdBreakablePrismaticVelocityPointJointConfigResetReengagesMovablePair)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Prismatic;
  joint.parentLink = sx::detail::toRegistryEntity(pair.parent.getEntity());
  joint.childLink = sx::detail::toRegistryEntity(pair.child.getEntity());
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.3;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);
  joint.breakForce = 1e-18;

  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(0.5, -1.0, 2.0).normalized();
  const Eigen::Vector3d parentAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d childAnchor(-0.1, 0.1, 0.0);
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = parentAnchor;
  config.localAnchorB = childAnchor;
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.linearAxes = dvbd::avbdRigidJointAxesFromFreeAxis(sliderAxis);
  config.angularAxes = config.linearAxes;
  config.linearAxisMask = dvbd::avbdRigidJointAllButAxisMask(2u);
  config.angularAxisMask = dvbd::kAvbdRigidJointAllAxesMask;
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  const auto anchorDelta = [&]() -> Eigen::Vector3d {
    const Eigen::Vector3d parentAnchorWorld
        = pair.parent.getWorldTransform() * parentAnchor;
    const Eigen::Vector3d childAnchorWorld
        = pair.child.getWorldTransform() * childAnchor;
    return childAnchorWorld - parentAnchorWorld;
  };
  const auto sliderAxisWorld = [&]() {
    return pair.parent.getWorldTransform().linear() * sliderAxis;
  };
  const auto orthogonalAnchorResidual = [&]() {
    const Eigen::Vector3d delta = anchorDelta();
    const Eigen::Vector3d axis = sliderAxisWorld();
    return (delta - delta.dot(axis) * axis).norm();
  };
  const auto sliderPosition = [&]() {
    return anchorDelta().dot(sliderAxisWorld());
  };
  const auto relativeRotationError = [&]() {
    return (pair.parent.getWorldTransform().linear().transpose()
                * pair.child.getWorldTransform().linear()
            - Eigen::Matrix3d::Identity())
        .norm();
  };

  world.enterSimulationMode();
  world.step();

  auto& liveJointModel = registry.get<sx::comps::JointModel>(jointEntity);
  auto& liveJointState = registry.get<sx::comps::JointState>(jointEntity);
  ASSERT_TRUE(liveJointState.broken);
  EXPECT_NEAR(sliderPosition(), targetSpeed * dt, 1e-6);
  EXPECT_LT(orthogonalAnchorResidual(), 2e-6);
  EXPECT_LT(relativeRotationError(), 1e-6);

  double maxBrokenOrthogonalResidual = 0.0;
  for (int k = 0; k < 20; ++k) {
    pair.parent.applyForce(
        -4.0 * Eigen::Vector3d::UnitY(),
        parentAnchor + 0.3 * Eigen::Vector3d::UnitX());
    pair.child.applyForce(
        4.0 * Eigen::Vector3d::UnitY(),
        childAnchor - 0.3 * Eigen::Vector3d::UnitX());
    world.step();

    maxBrokenOrthogonalResidual
        = std::max(maxBrokenOrthogonalResidual, orthogonalAnchorResidual());
  }
  ASSERT_GT(maxBrokenOrthogonalResidual, 1e-4);

  const double sliderBeforeReset = sliderPosition();
  pair.parent.getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  pair.child.getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  registry.get<sx::comps::JointActuation>(jointEntity).commandVelocity
      = Eigen::VectorXd::Constant(1, -targetSpeed);
  liveJointModel.breakForce = 1e6;
  liveJointState.broken = false;

  constexpr int resetSteps = 10;
  for (int k = 0; k < resetSteps; ++k) {
    world.step();
  }

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& resetConfig
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(resetConfig.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(resetConfig.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(resetConfig.linearAxes.col(2).dot(sliderAxis), 1.0, 1e-12);
  EXPECT_FALSE(liveJointState.broken);
  const double resetOrthogonalResidual = orthogonalAnchorResidual();
  EXPECT_LT(resetOrthogonalResidual, maxBrokenOrthogonalResidual * 0.05);
  EXPECT_LT(resetOrthogonalResidual, 2e-3);
  EXPECT_LT(relativeRotationError(), 1e-6);
  EXPECT_NEAR(
      sliderPosition() - sliderBeforeReset,
      -targetSpeed * dt * static_cast<double>(resetSteps),
      2e-3);
}

// PLAN-104 AVBD articulated fracture bridge: direct private spherical
// point-joint configs also need reset coverage between two movable
// same-multibody endpoints. Resetting the broken joint should re-enable only
// the linear anchor rows while leaving relative orientation free.
TEST(
    VariationalIntegration,
    AvbdBreakableSphericalPointJointConfigResetReengagesMovablePair)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Spherical;
  joint.parentLink = sx::detail::toRegistryEntity(pair.parent.getEntity());
  joint.childLink = sx::detail::toRegistryEntity(pair.child.getEntity());
  joint.breakForce = 1e-18;

  const Eigen::Vector3d parentAnchor(0.2, 0.1, 0.0);
  const Eigen::Vector3d childAnchor(-0.1, 0.1, 0.0);
  auto& config
      = registry.emplace<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = parentAnchor;
  config.localAnchorB = childAnchor;
  config.linearAxisMask = dvbd::kAvbdRigidJointAllAxesMask;
  config.angularAxisMask = 0u;
  config.startStiffness = std::numeric_limits<double>::infinity();
  config.maxStiffness = std::numeric_limits<double>::infinity();

  const Eigen::Matrix3d capturedRelativeRotation
      = pair.parent.getWorldTransform().linear().transpose()
        * pair.child.getWorldTransform().linear();
  const auto anchorResidual = [&]() {
    const Eigen::Vector3d parentAnchorWorld
        = pair.parent.getWorldTransform() * parentAnchor;
    const Eigen::Vector3d childAnchorWorld
        = pair.child.getWorldTransform() * childAnchor;
    return (childAnchorWorld - parentAnchorWorld).norm();
  };
  const auto relativeRotationChange = [&]() {
    const Eigen::Matrix3d relativeRotation
        = pair.parent.getWorldTransform().linear().transpose()
          * pair.child.getWorldTransform().linear();
    return (relativeRotation - capturedRelativeRotation).norm();
  };
  const auto applyOpposingOffsetForces = [&]() {
    pair.parent.applyForce(
        -4.0 * Eigen::Vector3d::UnitY(),
        parentAnchor + 0.4 * Eigen::Vector3d::UnitX());
    pair.child.applyForce(
        4.0 * Eigen::Vector3d::UnitY(),
        childAnchor - 0.4 * Eigen::Vector3d::UnitX());
  };

  world.enterSimulationMode();
  applyOpposingOffsetForces();
  world.step();

  auto& liveJointModel = registry.get<sx::comps::JointModel>(jointEntity);
  auto& liveJointState = registry.get<sx::comps::JointState>(jointEntity);
  ASSERT_TRUE(liveJointState.broken);
  EXPECT_LT(anchorResidual(), 1e-6);

  double maxBrokenAnchorResidual = 0.0;
  double maxBrokenRotationChange = 0.0;
  for (int k = 0; k < 20; ++k) {
    applyOpposingOffsetForces();
    world.step();
    maxBrokenAnchorResidual
        = std::max(maxBrokenAnchorResidual, anchorResidual());
    maxBrokenRotationChange
        = std::max(maxBrokenRotationChange, relativeRotationChange());
  }
  ASSERT_GT(maxBrokenAnchorResidual, 1e-4);
  ASSERT_GT(maxBrokenRotationChange, 1e-4);

  pair.parent.getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  pair.child.getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  liveJointModel.breakForce = 1e6;
  liveJointState.broken = false;

  world.step();

  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& resetConfig
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(resetConfig.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(resetConfig.angularAxisMask, 0u);
  EXPECT_FALSE(liveJointState.broken);
  EXPECT_LT(anchorResidual(), 1e-6);
  EXPECT_GT(relativeRotationChange(), 1e-4);
}

// PLAN-104 AVBD articulated bridge: current-pose extracted private revolute
// velocity motors between two movable links must honor finite effort bounds,
// not become unbounded hard velocity targets after config generation.
TEST(
    VariationalIntegration,
    AvbdRevolutePointJointCurrentPoseExtractorRespectsTinyTorqueLimitOnMovablePair)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Revolute;
  joint.parentLink = sx::detail::toRegistryEntity(pair.parent.getEntity());
  joint.childLink = sx::detail::toRegistryEntity(pair.child.getEntity());
  const Eigen::Vector3d hingeAxis = Eigen::Vector3d::UnitZ();
  joint.axis = hingeAxis;
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, 0.4);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1e-9);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1e-9);

  EXPECT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  world.enterSimulationMode();
  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));

  const Eigen::Vector3d parentAnchor = config.localAnchorA;
  const Eigen::Vector3d childAnchor = config.localAnchorB;
  const auto anchorResidual = [&]() {
    const Eigen::Vector3d parentAnchorWorld
        = pair.parent.getWorldTransform() * parentAnchor;
    const Eigen::Vector3d childAnchorWorld
        = pair.child.getWorldTransform() * childAnchor;
    return (childAnchorWorld - parentAnchorWorld).norm();
  };
  const auto axisTilt = [&]() {
    return (pair.parent.getWorldTransform().linear() * hingeAxis
            - pair.child.getWorldTransform().linear() * hingeAxis)
        .norm();
  };
  const auto relativeYaw = [&]() {
    const Eigen::Matrix3d relativeRotation
        = pair.parent.getWorldTransform().linear().transpose()
          * pair.child.getWorldTransform().linear();
    return std::atan2(relativeRotation(1, 0), relativeRotation(0, 0));
  };

  constexpr int steps = 20;
  double maxAnchorResidual = 0.0;
  double maxAxisTilt = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();
    maxAnchorResidual = std::max(maxAnchorResidual, anchorResidual());
    maxAxisTilt = std::max(maxAxisTilt, axisTilt());
  }

  EXPECT_LT(maxAnchorResidual, 1e-3);
  EXPECT_LT(maxAxisTilt, 1e-6);
  EXPECT_LT(std::abs(relativeYaw()), 1e-3);
}

// PLAN-104 AVBD articulated bridge: current-pose extracted private prismatic
// velocity motors between two movable links must also honor finite effort
// bounds while the generated masked hard rows preserve the captured pose.
TEST(
    VariationalIntegration,
    AvbdPrismaticPointJointCurrentPoseExtractorRespectsTinyForceLimitOnMovablePair)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Prismatic;
  joint.parentLink = sx::detail::toRegistryEntity(pair.parent.getEntity());
  joint.childLink = sx::detail::toRegistryEntity(pair.child.getEntity());
  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(1.0, 2.0, 0.5).normalized();
  joint.axis = sliderAxis;
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, 0.3);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1e-9);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1e-9);

  EXPECT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  world.enterSimulationMode();
  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);

  const Eigen::Vector3d parentAnchor = config.localAnchorA;
  const Eigen::Vector3d childAnchor = config.localAnchorB;
  const auto anchorDelta = [&]() -> Eigen::Vector3d {
    const Eigen::Vector3d parentAnchorWorld
        = pair.parent.getWorldTransform() * parentAnchor;
    const Eigen::Vector3d childAnchorWorld
        = pair.child.getWorldTransform() * childAnchor;
    return childAnchorWorld - parentAnchorWorld;
  };
  const auto orthogonalAnchorResidual = [&]() {
    const Eigen::Vector3d delta = anchorDelta();
    return (delta - delta.dot(sliderAxis) * sliderAxis).norm();
  };
  const auto relativeRotationError = [&]() {
    return (pair.parent.getWorldTransform().linear().transpose()
                * pair.child.getWorldTransform().linear()
            - Eigen::Matrix3d::Identity())
        .norm();
  };

  const double capturedSliderPosition = anchorDelta().dot(sliderAxis);
  constexpr int steps = 20;
  double maxSliderDrift = 0.0;
  double maxOrthogonalResidual = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < steps; ++k) {
    world.step();
    maxSliderDrift = std::max(
        maxSliderDrift,
        std::abs(anchorDelta().dot(sliderAxis) - capturedSliderPosition));
    maxOrthogonalResidual
        = std::max(maxOrthogonalResidual, orthogonalAnchorResidual());
    maxRotationError = std::max(maxRotationError, relativeRotationError());
  }

  EXPECT_LT(maxSliderDrift, 1e-3);
  EXPECT_LT(maxOrthogonalResidual, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
}

// PLAN-104 AVBD articulated bridge: generated current-pose revolute motor rows
// must keep finite effort bounds across simulation-mode binary save/load. A
// restored tiny torque limit should still prevent the free hinge axis from
// becoming an unbounded hard velocity target.
TEST(
    VariationalIntegration,
    AvbdRevolutePointJointCurrentPoseTinyTorqueLimitSurvivesSaveLoad)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  registry.emplace<sx::comps::Name>(
      jointEntity, "serialized_generated_tiny_hinge");
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.name = "serialized_generated_tiny_hinge";
  joint.type = sx::comps::JointType::Revolute;
  joint.parentLink = sx::detail::toRegistryEntity(pair.parent.getEntity());
  joint.childLink = sx::detail::toRegistryEntity(pair.child.getEntity());
  const Eigen::Vector3d hingeAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  joint.axis = hingeAxis;
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, 0.4);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1e-9);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1e-9);

  world.enterSimulationMode();
  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot = restored.getMultibody("floating_pair");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredParent = restoredRobot->getLink("parent");
  ASSERT_TRUE(restoredParent.has_value());
  auto restoredChild = restoredRobot->getLink("child");
  ASSERT_TRUE(restoredChild.has_value());

  auto& restoredRegistry = dart::simulation::detail::registryOf(restored);
  entt::entity restoredJointEntity = entt::null;
  auto jointView = restoredRegistry.view<sx::comps::JointModel>();
  for (const entt::entity entity : jointView) {
    if (jointView.get<sx::comps::JointModel>(entity).name
        == "serialized_generated_tiny_hinge") {
      restoredJointEntity = entity;
      break;
    }
  }
  ASSERT_TRUE(restoredJointEntity != entt::null);
  const auto& restoredJointModel
      = restoredRegistry.get<sx::comps::JointModel>(restoredJointEntity);
  const auto& restoredJointActuation
      = restoredRegistry.get<sx::comps::JointActuation>(restoredJointEntity);
  EXPECT_EQ(restoredJointModel.type, sx::comps::JointType::Revolute);
  EXPECT_EQ(
      restoredJointActuation.actuatorType, sx::comps::ActuatorType::Velocity);
  ASSERT_EQ(restoredJointActuation.commandVelocity.size(), 1);
  EXPECT_DOUBLE_EQ(restoredJointActuation.commandVelocity[0], 0.4);
  ASSERT_EQ(restoredJointModel.limits.effortLower.size(), 1);
  ASSERT_EQ(restoredJointModel.limits.effortUpper.size(), 1);
  EXPECT_DOUBLE_EQ(restoredJointModel.limits.effortLower[0], -1e-9);
  EXPECT_DOUBLE_EQ(restoredJointModel.limits.effortUpper[0], 1e-9);
  ASSERT_TRUE(restoredRegistry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(
      restoredJointEntity));
  const auto& restoredConfig
      = restoredRegistry.get<dvbd::AvbdRigidWorldPointJointConfig>(
          restoredJointEntity);
  EXPECT_EQ(restoredConfig.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(
      restoredConfig.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_NEAR(restoredConfig.angularAxes.col(2).dot(hingeAxis), 1.0, 1e-12);

  const Eigen::Vector3d parentAnchor = restoredConfig.localAnchorA;
  const Eigen::Vector3d childAnchor = restoredConfig.localAnchorB;
  const auto anchorResidual = [&]() {
    const Eigen::Vector3d parentAnchorWorld
        = restoredParent->getWorldTransform() * parentAnchor;
    const Eigen::Vector3d childAnchorWorld
        = restoredChild->getWorldTransform() * childAnchor;
    return (childAnchorWorld - parentAnchorWorld).norm();
  };
  const auto axisTilt = [&]() {
    return (restoredParent->getWorldTransform().linear() * hingeAxis
            - restoredChild->getWorldTransform().linear() * hingeAxis)
        .norm();
  };
  const auto relativeHingePosition = [&]() {
    const Eigen::Matrix3d relativeRotation
        = restoredParent->getWorldTransform().linear().transpose()
          * restoredChild->getWorldTransform().linear();
    return signedRotationAroundAxis(relativeRotation, hingeAxis);
  };

  double maxAnchorResidual = 0.0;
  double maxAxisTilt = 0.0;
  for (int k = 0; k < 20; ++k) {
    restored.step();
    maxAnchorResidual = std::max(maxAnchorResidual, anchorResidual());
    maxAxisTilt = std::max(maxAxisTilt, axisTilt());
  }

  EXPECT_LT(maxAnchorResidual, 1e-3);
  EXPECT_LT(maxAxisTilt, 1e-6);
  EXPECT_LT(std::abs(relativeHingePosition()), 1e-3);
}

// PLAN-104 AVBD articulated bridge: generated current-pose prismatic motor rows
// must also keep finite effort bounds across simulation-mode binary save/load,
// so a restored tiny force limit preserves the hard rows without sliding the
// free axis like an unbounded velocity target.
TEST(
    VariationalIntegration,
    AvbdPrismaticPointJointCurrentPoseTinyForceLimitSurvivesSaveLoad)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  registry.emplace<sx::comps::Name>(
      jointEntity, "serialized_generated_tiny_slider");
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.name = "serialized_generated_tiny_slider";
  joint.type = sx::comps::JointType::Prismatic;
  joint.parentLink = sx::detail::toRegistryEntity(pair.parent.getEntity());
  joint.childLink = sx::detail::toRegistryEntity(pair.child.getEntity());
  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(1.0, 2.0, 0.5).normalized();
  joint.axis = sliderAxis;
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, 0.3);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1e-9);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1e-9);

  world.enterSimulationMode();
  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot = restored.getMultibody("floating_pair");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredParent = restoredRobot->getLink("parent");
  ASSERT_TRUE(restoredParent.has_value());
  auto restoredChild = restoredRobot->getLink("child");
  ASSERT_TRUE(restoredChild.has_value());

  auto& restoredRegistry = dart::simulation::detail::registryOf(restored);
  entt::entity restoredJointEntity = entt::null;
  auto jointView = restoredRegistry.view<sx::comps::JointModel>();
  for (const entt::entity entity : jointView) {
    if (jointView.get<sx::comps::JointModel>(entity).name
        == "serialized_generated_tiny_slider") {
      restoredJointEntity = entity;
      break;
    }
  }
  ASSERT_TRUE(restoredJointEntity != entt::null);
  const auto& restoredJointModel
      = restoredRegistry.get<sx::comps::JointModel>(restoredJointEntity);
  const auto& restoredJointActuation
      = restoredRegistry.get<sx::comps::JointActuation>(restoredJointEntity);
  EXPECT_EQ(restoredJointModel.type, sx::comps::JointType::Prismatic);
  EXPECT_EQ(
      restoredJointActuation.actuatorType, sx::comps::ActuatorType::Velocity);
  ASSERT_EQ(restoredJointActuation.commandVelocity.size(), 1);
  EXPECT_DOUBLE_EQ(restoredJointActuation.commandVelocity[0], 0.3);
  ASSERT_EQ(restoredJointModel.limits.effortLower.size(), 1);
  ASSERT_EQ(restoredJointModel.limits.effortUpper.size(), 1);
  EXPECT_DOUBLE_EQ(restoredJointModel.limits.effortLower[0], -1e-9);
  EXPECT_DOUBLE_EQ(restoredJointModel.limits.effortUpper[0], 1e-9);
  ASSERT_TRUE(restoredRegistry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(
      restoredJointEntity));
  const auto& restoredConfig
      = restoredRegistry.get<dvbd::AvbdRigidWorldPointJointConfig>(
          restoredJointEntity);
  EXPECT_EQ(
      restoredConfig.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(restoredConfig.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(restoredConfig.linearAxes.col(2).dot(sliderAxis), 1.0, 1e-12);

  const Eigen::Vector3d parentAnchor = restoredConfig.localAnchorA;
  const Eigen::Vector3d childAnchor = restoredConfig.localAnchorB;
  const auto anchorDelta = [&]() -> Eigen::Vector3d {
    const Eigen::Vector3d parentAnchorWorld
        = restoredParent->getWorldTransform() * parentAnchor;
    const Eigen::Vector3d childAnchorWorld
        = restoredChild->getWorldTransform() * childAnchor;
    return childAnchorWorld - parentAnchorWorld;
  };
  const auto sliderAxisWorld = [&]() {
    return restoredParent->getWorldTransform().linear() * sliderAxis;
  };
  const auto sliderPosition = [&]() {
    const Eigen::Vector3d axis = sliderAxisWorld();
    return anchorDelta().dot(axis);
  };
  const auto orthogonalAnchorResidual = [&]() {
    const Eigen::Vector3d delta = anchorDelta();
    const Eigen::Vector3d axis = sliderAxisWorld();
    return (delta - delta.dot(axis) * axis).norm();
  };
  const auto relativeRotationError = [&]() {
    return (restoredParent->getWorldTransform().linear().transpose()
                * restoredChild->getWorldTransform().linear()
            - Eigen::Matrix3d::Identity())
        .norm();
  };

  const double capturedSliderPosition = sliderPosition();
  double maxSliderDrift = 0.0;
  double maxOrthogonalResidual = 0.0;
  double maxRotationError = 0.0;
  for (int k = 0; k < 20; ++k) {
    restored.step();
    maxSliderDrift = std::max(
        maxSliderDrift, std::abs(sliderPosition() - capturedSliderPosition));
    maxOrthogonalResidual
        = std::max(maxOrthogonalResidual, orthogonalAnchorResidual());
    maxRotationError = std::max(maxRotationError, relativeRotationError());
  }

  EXPECT_LT(maxSliderDrift, 1e-3);
  EXPECT_LT(maxOrthogonalResidual, 1e-6);
  EXPECT_LT(maxRotationError, 1e-6);
}

// PLAN-104 AVBD articulated bridge: current-pose extraction should also rebuild
// a private fixed point-joint between two movable same-multibody links after a
// break/reset round trip. Unlike spherical rows, fixed rows re-engage both the
// captured anchor and relative orientation.
TEST(
    VariationalIntegration,
    AvbdFixedPointJointCurrentPoseExtractorResetReengagesMovablePair)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink = sx::detail::toRegistryEntity(pair.parent.getEntity());
  joint.childLink = sx::detail::toRegistryEntity(pair.child.getEntity());
  joint.breakForce = 1e-18;

  EXPECT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  world.enterSimulationMode();
  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_TRUE(std::isinf(config.startStiffness));
  EXPECT_TRUE(std::isinf(config.maxStiffness));

  const Eigen::Vector3d parentAnchor = config.localAnchorA;
  const Eigen::Vector3d childAnchor = config.localAnchorB;
  const Eigen::Matrix3d capturedRelativeRotation
      = pair.parent.getWorldTransform().linear().transpose()
        * pair.child.getWorldTransform().linear();
  const auto anchorResidual = [&]() {
    const Eigen::Vector3d parentAnchorWorld
        = pair.parent.getWorldTransform() * parentAnchor;
    const Eigen::Vector3d childAnchorWorld
        = pair.child.getWorldTransform() * childAnchor;
    return (childAnchorWorld - parentAnchorWorld).norm();
  };
  const auto relativeRotationError = [&]() {
    const Eigen::Matrix3d relativeRotation
        = pair.parent.getWorldTransform().linear().transpose()
          * pair.child.getWorldTransform().linear();
    return (relativeRotation - capturedRelativeRotation).norm();
  };
  const auto applyOpposingOffsetForces = [&]() {
    pair.parent.applyForce(
        -4.0 * Eigen::Vector3d::UnitY(),
        parentAnchor + 0.4 * Eigen::Vector3d::UnitX());
    pair.child.applyForce(
        4.0 * Eigen::Vector3d::UnitY(),
        childAnchor - 0.4 * Eigen::Vector3d::UnitX());
  };

  applyOpposingOffsetForces();
  world.step();

  auto& liveJointModel = registry.get<sx::comps::JointModel>(jointEntity);
  auto& liveJointState = registry.get<sx::comps::JointState>(jointEntity);
  ASSERT_TRUE(liveJointState.broken);
  EXPECT_LT(anchorResidual(), 1e-6);
  EXPECT_LT(relativeRotationError(), 1e-6);

  double maxBrokenAnchorResidual = 0.0;
  double maxBrokenRotationError = 0.0;
  for (int k = 0; k < 20; ++k) {
    applyOpposingOffsetForces();
    world.step();
    maxBrokenAnchorResidual
        = std::max(maxBrokenAnchorResidual, anchorResidual());
    maxBrokenRotationError
        = std::max(maxBrokenRotationError, relativeRotationError());
  }
  ASSERT_GT(maxBrokenAnchorResidual, 1e-4);
  ASSERT_GT(maxBrokenRotationError, 1e-4);

  pair.parent.getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  pair.child.getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  liveJointModel.breakForce = 1e6;
  liveJointState.broken = false;

  world.step();

  EXPECT_FALSE(liveJointState.broken);
  EXPECT_LT(anchorResidual(), 1e-6);
  EXPECT_LT(relativeRotationError(), 1e-6);
}

// PLAN-104 AVBD articulated bridge: generated private fixed current-pose rows
// between two movable links must keep captured anchors and relative orientation
// across simulation-mode binary save/load before reset re-enables all hard
// rows.
TEST(
    VariationalIntegration,
    AvbdFixedPointJointCurrentPoseExtractorBreakageSurvivesSaveLoad)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  registry.emplace<sx::comps::Name>(
      jointEntity, "serialized_generated_movable_fixed");
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.name = "serialized_generated_movable_fixed";
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink = sx::detail::toRegistryEntity(pair.parent.getEntity());
  joint.childLink = sx::detail::toRegistryEntity(pair.child.getEntity());
  joint.breakForce = 1e-18;

  EXPECT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  world.enterSimulationMode();
  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  const Eigen::Vector3d parentAnchor = config.localAnchorA;
  const Eigen::Vector3d childAnchor = config.localAnchorB;
  const Eigen::Matrix3d capturedRelativeRotation
      = pair.parent.getWorldTransform().linear().transpose()
        * pair.child.getWorldTransform().linear();

  const auto anchorResidual = [&]() {
    const Eigen::Vector3d parentAnchorWorld
        = pair.parent.getWorldTransform() * parentAnchor;
    const Eigen::Vector3d childAnchorWorld
        = pair.child.getWorldTransform() * childAnchor;
    return (childAnchorWorld - parentAnchorWorld).norm();
  };
  const auto relativeRotationError = [&]() {
    const Eigen::Matrix3d relativeRotation
        = pair.parent.getWorldTransform().linear().transpose()
          * pair.child.getWorldTransform().linear();
    return (relativeRotation - capturedRelativeRotation).norm();
  };
  const auto applyOpposingOffsetForces = [&]() {
    pair.parent.applyForce(
        -4.0 * Eigen::Vector3d::UnitY(),
        parentAnchor + 0.4 * Eigen::Vector3d::UnitX());
    pair.child.applyForce(
        4.0 * Eigen::Vector3d::UnitY(),
        childAnchor - 0.4 * Eigen::Vector3d::UnitX());
  };

  applyOpposingOffsetForces();
  world.step();

  ASSERT_TRUE(registry.get<sx::comps::JointState>(jointEntity).broken);
  EXPECT_LT(anchorResidual(), 1e-6);
  EXPECT_LT(relativeRotationError(), 1e-6);
  const Eigen::Isometry3d savedParentTransform
      = pair.parent.getWorldTransform();
  const Eigen::Isometry3d savedChildTransform = pair.child.getWorldTransform();
  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot = restored.getMultibody("floating_pair");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredParent = restoredRobot->getLink("parent");
  ASSERT_TRUE(restoredParent.has_value());
  auto restoredChild = restoredRobot->getLink("child");
  ASSERT_TRUE(restoredChild.has_value());

  auto& restoredRegistry = dart::simulation::detail::registryOf(restored);
  entt::entity restoredJointEntity = entt::null;
  auto jointView = restoredRegistry.view<sx::comps::JointModel>();
  for (const entt::entity entity : jointView) {
    if (jointView.get<sx::comps::JointModel>(entity).name
        == "serialized_generated_movable_fixed") {
      restoredJointEntity = entity;
      break;
    }
  }
  ASSERT_TRUE(restoredJointEntity != entt::null);
  auto& restoredJointModel
      = restoredRegistry.get<sx::comps::JointModel>(restoredJointEntity);
  auto& restoredJointState
      = restoredRegistry.get<sx::comps::JointState>(restoredJointEntity);
  ASSERT_TRUE(restoredJointState.broken);
  EXPECT_EQ(restoredJointModel.type, sx::comps::JointType::Fixed);
  EXPECT_EQ(
      restoredJointModel.parentLink,
      sx::detail::toRegistryEntity(restoredParent->getEntity()));
  EXPECT_EQ(
      restoredJointModel.childLink,
      sx::detail::toRegistryEntity(restoredChild->getEntity()));
  EXPECT_TRUE(restoredJointModel.hasRigidBodyFixedJointAnchors);
  EXPECT_LT(
      (restoredJointModel.rigidBodyFixedJointLocalAnchorParent - parentAnchor)
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredJointModel.rigidBodyFixedJointLocalAnchorChild - childAnchor)
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredParent->getWorldTransform().matrix()
       - savedParentTransform.matrix())
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredChild->getWorldTransform().matrix()
       - savedChildTransform.matrix())
          .norm(),
      1e-12);

  const auto restoredAnchorResidual = [&]() {
    const Eigen::Vector3d parentAnchorWorld
        = restoredParent->getWorldTransform() * parentAnchor;
    const Eigen::Vector3d childAnchorWorld
        = restoredChild->getWorldTransform() * childAnchor;
    return (childAnchorWorld - parentAnchorWorld).norm();
  };
  const auto restoredRelativeRotationError = [&]() {
    const Eigen::Matrix3d relativeRotation
        = restoredParent->getWorldTransform().linear().transpose()
          * restoredChild->getWorldTransform().linear();
    return (relativeRotation - capturedRelativeRotation).norm();
  };
  const auto applyRestoredOpposingOffsetForces = [&]() {
    restoredParent->applyForce(
        -4.0 * Eigen::Vector3d::UnitY(),
        parentAnchor + 0.4 * Eigen::Vector3d::UnitX());
    restoredChild->applyForce(
        4.0 * Eigen::Vector3d::UnitY(),
        childAnchor - 0.4 * Eigen::Vector3d::UnitX());
  };

  double maxBrokenAnchorResidual = 0.0;
  double maxBrokenRotationError = 0.0;
  for (int k = 0; k < 20; ++k) {
    applyRestoredOpposingOffsetForces();
    restored.step();
    maxBrokenAnchorResidual
        = std::max(maxBrokenAnchorResidual, restoredAnchorResidual());
    maxBrokenRotationError
        = std::max(maxBrokenRotationError, restoredRelativeRotationError());
  }
  ASSERT_GT(maxBrokenAnchorResidual, 1e-4);
  ASSERT_GT(maxBrokenRotationError, 1e-4);

  restoredParent->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredChild->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredJointModel.breakForce = 1e6;
  restoredJointState.broken = false;

  restored.step();

  ASSERT_TRUE(restoredRegistry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(
      restoredJointEntity));
  const auto& restoredConfig
      = restoredRegistry.get<dvbd::AvbdRigidWorldPointJointConfig>(
          restoredJointEntity);
  EXPECT_EQ(restoredConfig.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(restoredConfig.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_FALSE(restoredJointState.broken);
  EXPECT_LT(restoredAnchorResidual(), 1e-6);
  EXPECT_LT(restoredRelativeRotationError(), 1e-6);
}

// PLAN-104 AVBD articulated bridge: current-pose extraction must also cover
// private revolute velocity point-joints between two movable same-multibody
// links. After a break/reset round trip, the generated config should re-enable
// the masked hard rows while the free-axis motor samples the updated command.
TEST(
    VariationalIntegration,
    AvbdRevolutePointJointCurrentPoseExtractorResetReengagesMovablePair)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Revolute;
  joint.parentLink = sx::detail::toRegistryEntity(pair.parent.getEntity());
  joint.childLink = sx::detail::toRegistryEntity(pair.child.getEntity());
  const Eigen::Vector3d hingeAxis
      = Eigen::Vector3d(-2.0, 1.0, 3.0).normalized();
  joint.axis = hingeAxis;
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.4;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);
  joint.breakForce = 1e-18;

  EXPECT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  world.enterSimulationMode();
  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_TRUE(std::isinf(config.startStiffness));
  EXPECT_TRUE(std::isinf(config.maxStiffness));

  const Eigen::Vector3d parentAnchor = config.localAnchorA;
  const Eigen::Vector3d childAnchor = config.localAnchorB;
  const auto parentAnchorWorld = [&]() {
    return pair.parent.getWorldTransform() * parentAnchor;
  };
  const auto childAnchorWorld = [&]() {
    return pair.child.getWorldTransform() * childAnchor;
  };
  const auto anchorResidual = [&]() {
    return (childAnchorWorld() - parentAnchorWorld()).norm();
  };
  const auto axisTilt = [&]() {
    return (pair.parent.getWorldTransform().linear() * hingeAxis
            - pair.child.getWorldTransform().linear() * hingeAxis)
        .norm();
  };
  const auto relativeHingePosition = [&]() {
    const Eigen::Matrix3d relativeRotation
        = pair.parent.getWorldTransform().linear().transpose()
          * pair.child.getWorldTransform().linear();
    return signedRotationAroundAxis(relativeRotation, hingeAxis);
  };

  world.step();

  auto& liveJointModel = registry.get<sx::comps::JointModel>(jointEntity);
  auto& liveJointState = registry.get<sx::comps::JointState>(jointEntity);
  ASSERT_TRUE(liveJointState.broken);
  const double firstStepHingePosition = relativeHingePosition();
  EXPECT_NEAR(firstStepHingePosition, targetSpeed * dt, 1e-6);
  EXPECT_LT(anchorResidual(), 1e-6);
  EXPECT_LT(axisTilt(), 1e-6);

  double maxBrokenAnchorResidual = 0.0;
  for (int k = 0; k < 20; ++k) {
    pair.parent.applyForce(
        -4.0 * Eigen::Vector3d::UnitY(),
        parentAnchor + 0.3 * Eigen::Vector3d::UnitX());
    pair.child.applyForce(
        4.0 * Eigen::Vector3d::UnitY(),
        childAnchor - 0.3 * Eigen::Vector3d::UnitX());
    world.step();
    maxBrokenAnchorResidual
        = std::max(maxBrokenAnchorResidual, anchorResidual());
  }
  ASSERT_GT(maxBrokenAnchorResidual, 1e-4);

  const double hingeBeforeReset = relativeHingePosition();
  pair.parent.getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  pair.child.getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  registry.get<sx::comps::JointActuation>(jointEntity).commandVelocity
      = Eigen::VectorXd::Constant(1, -targetSpeed);
  liveJointModel.breakForce = 1e6;
  liveJointState.broken = false;

  constexpr int resetSteps = 10;
  for (int k = 0; k < resetSteps; ++k) {
    world.step();
  }

  EXPECT_FALSE(liveJointState.broken);
  EXPECT_LT(anchorResidual(), 2e-3);
  EXPECT_LT(axisTilt(), 2e-3);
  EXPECT_NEAR(
      relativeHingePosition() - hingeBeforeReset,
      -targetSpeed * dt * static_cast<double>(resetSteps),
      2e-3);
}

// PLAN-104 AVBD articulated bridge: current-pose extraction must also cover
// private prismatic velocity point-joints between two movable same-multibody
// links. After a break/reset round trip, the existing extracted config should
// re-enable its masked hard rows while the free-axis motor samples the updated
// command.
TEST(
    VariationalIntegration,
    AvbdPrismaticPointJointCurrentPoseExtractorResetReengagesMovablePair)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Prismatic;
  joint.parentLink = sx::detail::toRegistryEntity(pair.parent.getEntity());
  joint.childLink = sx::detail::toRegistryEntity(pair.child.getEntity());
  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(0.5, -1.0, 2.0).normalized();
  joint.axis = sliderAxis;
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.3;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);
  joint.breakForce = 1e-18;

  EXPECT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  world.enterSimulationMode();
  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_TRUE(std::isinf(config.startStiffness));
  EXPECT_TRUE(std::isinf(config.maxStiffness));

  const Eigen::Vector3d parentAnchor = config.localAnchorA;
  const Eigen::Vector3d childAnchor = config.localAnchorB;
  const auto anchorDelta = [&]() -> Eigen::Vector3d {
    const Eigen::Vector3d parentAnchorWorld
        = pair.parent.getWorldTransform() * parentAnchor;
    const Eigen::Vector3d childAnchorWorld
        = pair.child.getWorldTransform() * childAnchor;
    return childAnchorWorld - parentAnchorWorld;
  };
  const auto orthogonalAnchorResidual = [&]() {
    const Eigen::Vector3d delta = anchorDelta();
    const Eigen::Vector3d axis
        = pair.parent.getWorldTransform().linear() * sliderAxis;
    return (delta - delta.dot(axis) * axis).norm();
  };
  const auto sliderPosition = [&]() {
    const Eigen::Vector3d axis
        = pair.parent.getWorldTransform().linear() * sliderAxis;
    return anchorDelta().dot(axis);
  };
  const auto relativeRotationError = [&]() {
    return (pair.parent.getWorldTransform().linear().transpose()
                * pair.child.getWorldTransform().linear()
            - Eigen::Matrix3d::Identity())
        .norm();
  };

  world.step();

  auto& liveJointModel = registry.get<sx::comps::JointModel>(jointEntity);
  auto& liveJointState = registry.get<sx::comps::JointState>(jointEntity);
  ASSERT_TRUE(liveJointState.broken);
  EXPECT_NEAR(sliderPosition(), targetSpeed * dt, 1e-6);
  EXPECT_LT(orthogonalAnchorResidual(), 2e-6);
  EXPECT_LT(relativeRotationError(), 1e-6);

  double maxBrokenOrthogonalResidual = 0.0;
  for (int k = 0; k < 20; ++k) {
    pair.parent.applyForce(
        -4.0 * Eigen::Vector3d::UnitY(),
        parentAnchor + 0.3 * Eigen::Vector3d::UnitX());
    pair.child.applyForce(
        4.0 * Eigen::Vector3d::UnitY(),
        childAnchor - 0.3 * Eigen::Vector3d::UnitX());
    world.step();

    maxBrokenOrthogonalResidual
        = std::max(maxBrokenOrthogonalResidual, orthogonalAnchorResidual());
  }
  ASSERT_GT(maxBrokenOrthogonalResidual, 1e-4);

  const double sliderBeforeReset = sliderPosition();
  pair.parent.getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  pair.child.getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  registry.get<sx::comps::JointActuation>(jointEntity).commandVelocity
      = Eigen::VectorXd::Constant(1, -targetSpeed);
  liveJointModel.breakForce = 1e6;
  liveJointState.broken = false;

  constexpr int resetSteps = 10;
  for (int k = 0; k < resetSteps; ++k) {
    world.step();
  }

  EXPECT_FALSE(liveJointState.broken);
  const double resetOrthogonalResidual = orthogonalAnchorResidual();
  EXPECT_LT(resetOrthogonalResidual, maxBrokenOrthogonalResidual * 0.05);
  EXPECT_LT(resetOrthogonalResidual, 2e-3);
  EXPECT_LT(relativeRotationError(), 1e-6);
  EXPECT_NEAR(
      sliderPosition() - sliderBeforeReset,
      -targetSpeed * dt * static_cast<double>(resetSteps),
      2e-3);
}

// PLAN-104 AVBD articulated bridge: generated private revolute velocity rows
// between two movable links must keep their captured current-pose anchors and
// motor state across simulation-mode binary save/load before breakage reset.
TEST(
    VariationalIntegration,
    AvbdRevolutePointJointCurrentPoseExtractorBreakageSurvivesSaveLoad)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  registry.emplace<sx::comps::Name>(
      jointEntity, "serialized_generated_movable_hinge");
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.name = "serialized_generated_movable_hinge";
  joint.type = sx::comps::JointType::Revolute;
  joint.parentLink = sx::detail::toRegistryEntity(pair.parent.getEntity());
  joint.childLink = sx::detail::toRegistryEntity(pair.child.getEntity());
  const Eigen::Vector3d hingeAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  joint.axis = hingeAxis;
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.4;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);
  joint.breakForce = 1e-18;

  EXPECT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  world.enterSimulationMode();
  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_NEAR(config.angularAxes.col(2).dot(hingeAxis), 1.0, 1e-12);
  const Eigen::Vector3d parentAnchor = config.localAnchorA;
  const Eigen::Vector3d childAnchor = config.localAnchorB;

  const auto relativeHingePosition = [&]() {
    const Eigen::Matrix3d relativeRotation
        = pair.parent.getWorldTransform().linear().transpose()
          * pair.child.getWorldTransform().linear();
    return signedRotationAroundAxis(relativeRotation, hingeAxis);
  };

  world.step();

  ASSERT_TRUE(registry.get<sx::comps::JointState>(jointEntity).broken);
  EXPECT_NEAR(relativeHingePosition(), targetSpeed * dt, 1e-6);
  const Eigen::Isometry3d savedParentTransform
      = pair.parent.getWorldTransform();
  const Eigen::Isometry3d savedChildTransform = pair.child.getWorldTransform();
  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot = restored.getMultibody("floating_pair");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredParent = restoredRobot->getLink("parent");
  ASSERT_TRUE(restoredParent.has_value());
  auto restoredChild = restoredRobot->getLink("child");
  ASSERT_TRUE(restoredChild.has_value());

  auto& restoredRegistry = dart::simulation::detail::registryOf(restored);
  entt::entity restoredJointEntity = entt::null;
  auto jointView = restoredRegistry.view<sx::comps::JointModel>();
  for (const entt::entity entity : jointView) {
    if (jointView.get<sx::comps::JointModel>(entity).name
        == "serialized_generated_movable_hinge") {
      restoredJointEntity = entity;
      break;
    }
  }
  ASSERT_TRUE(restoredJointEntity != entt::null);
  auto& restoredJointModel
      = restoredRegistry.get<sx::comps::JointModel>(restoredJointEntity);
  auto& restoredJointState
      = restoredRegistry.get<sx::comps::JointState>(restoredJointEntity);
  auto& restoredJointActuation
      = restoredRegistry.get<sx::comps::JointActuation>(restoredJointEntity);
  ASSERT_TRUE(restoredJointState.broken);
  EXPECT_EQ(restoredJointModel.type, sx::comps::JointType::Revolute);
  EXPECT_EQ(
      restoredJointModel.parentLink,
      sx::detail::toRegistryEntity(restoredParent->getEntity()));
  EXPECT_EQ(
      restoredJointModel.childLink,
      sx::detail::toRegistryEntity(restoredChild->getEntity()));
  EXPECT_LT((restoredJointModel.axis - hingeAxis).norm(), 1e-12);
  EXPECT_EQ(
      restoredJointActuation.actuatorType, sx::comps::ActuatorType::Velocity);
  ASSERT_EQ(restoredJointActuation.commandVelocity.size(), 1);
  EXPECT_DOUBLE_EQ(restoredJointActuation.commandVelocity[0], targetSpeed);
  ASSERT_EQ(restoredJointModel.limits.effortLower.size(), 1);
  ASSERT_EQ(restoredJointModel.limits.effortUpper.size(), 1);
  EXPECT_DOUBLE_EQ(restoredJointModel.limits.effortLower[0], -1000.0);
  EXPECT_DOUBLE_EQ(restoredJointModel.limits.effortUpper[0], 1000.0);
  EXPECT_TRUE(restoredJointModel.hasRigidBodyFixedJointAnchors);
  EXPECT_LT(
      (restoredJointModel.rigidBodyFixedJointLocalAnchorParent - parentAnchor)
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredJointModel.rigidBodyFixedJointLocalAnchorChild - childAnchor)
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredParent->getWorldTransform().translation()
       - savedParentTransform.translation())
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredChild->getWorldTransform().translation()
       - savedChildTransform.translation())
          .norm(),
      1e-12);

  const auto parentAnchorWorld = [&]() {
    return restoredParent->getWorldTransform() * parentAnchor;
  };
  const auto childAnchorWorld = [&]() {
    return restoredChild->getWorldTransform() * childAnchor;
  };
  const auto anchorResidual = [&]() {
    return (childAnchorWorld() - parentAnchorWorld()).norm();
  };
  const auto axisTilt = [&]() {
    return (restoredParent->getWorldTransform().linear() * hingeAxis
            - restoredChild->getWorldTransform().linear() * hingeAxis)
        .norm();
  };
  const auto restoredRelativeHingePosition = [&]() {
    const Eigen::Matrix3d relativeRotation
        = restoredParent->getWorldTransform().linear().transpose()
          * restoredChild->getWorldTransform().linear();
    return signedRotationAroundAxis(relativeRotation, hingeAxis);
  };

  double maxBrokenAnchorResidual = 0.0;
  for (int k = 0; k < 20; ++k) {
    restoredParent->applyForce(
        -4.0 * Eigen::Vector3d::UnitY(),
        parentAnchor + 0.3 * Eigen::Vector3d::UnitX());
    restoredChild->applyForce(
        4.0 * Eigen::Vector3d::UnitY(),
        childAnchor - 0.3 * Eigen::Vector3d::UnitX());
    restored.step();
    maxBrokenAnchorResidual
        = std::max(maxBrokenAnchorResidual, anchorResidual());
  }
  ASSERT_GT(maxBrokenAnchorResidual, 1e-4);

  const double hingeBeforeReset = restoredRelativeHingePosition();
  restoredParent->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredChild->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredJointActuation.commandVelocity
      = Eigen::VectorXd::Constant(1, -targetSpeed);
  restoredJointModel.breakForce = 1e6;
  restoredJointState.broken = false;

  constexpr int resetSteps = 10;
  for (int k = 0; k < resetSteps; ++k) {
    restored.step();
  }

  ASSERT_TRUE(restoredRegistry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(
      restoredJointEntity));
  const auto& restoredConfig
      = restoredRegistry.get<dvbd::AvbdRigidWorldPointJointConfig>(
          restoredJointEntity);
  EXPECT_EQ(restoredConfig.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(
      restoredConfig.angularAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_NEAR(restoredConfig.angularAxes.col(2).dot(hingeAxis), 1.0, 1e-12);
  EXPECT_FALSE(restoredJointState.broken);
  EXPECT_LT(anchorResidual(), 2e-3);
  EXPECT_LT(axisTilt(), 2e-3);
  EXPECT_NEAR(
      restoredRelativeHingePosition() - hingeBeforeReset,
      -targetSpeed * dt * static_cast<double>(resetSteps),
      2e-3);
}

// PLAN-104 AVBD articulated bridge: generated private prismatic velocity rows
// between two movable links must also keep their captured current-pose anchors
// and motor state across simulation-mode binary save/load before reset.
TEST(
    VariationalIntegration,
    AvbdPrismaticPointJointCurrentPoseExtractorBreakageSurvivesSaveLoad)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  const double dt = 0.005;
  world.setTimeStep(dt);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  registry.emplace<sx::comps::Name>(
      jointEntity, "serialized_generated_movable_slider");
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  auto& jointActuation
      = registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.name = "serialized_generated_movable_slider";
  joint.type = sx::comps::JointType::Prismatic;
  joint.parentLink = sx::detail::toRegistryEntity(pair.parent.getEntity());
  joint.childLink = sx::detail::toRegistryEntity(pair.child.getEntity());
  const Eigen::Vector3d sliderAxis
      = Eigen::Vector3d(1.0, 2.0, 0.5).normalized();
  joint.axis = sliderAxis;
  jointActuation.actuatorType = sx::comps::ActuatorType::Velocity;
  const double targetSpeed = 0.3;
  jointActuation.commandVelocity = Eigen::VectorXd::Constant(1, targetSpeed);
  joint.limits.effortLower = Eigen::VectorXd::Constant(1, -1000.0);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(1, 1000.0);
  joint.breakForce = 1e-18;

  EXPECT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  world.enterSimulationMode();
  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(config.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(config.linearAxes.col(2).dot(sliderAxis), 1.0, 1e-12);
  const Eigen::Vector3d parentAnchor = config.localAnchorA;
  const Eigen::Vector3d childAnchor = config.localAnchorB;

  const auto anchorDelta = [&]() -> Eigen::Vector3d {
    const Eigen::Vector3d parentAnchorWorld
        = pair.parent.getWorldTransform() * parentAnchor;
    const Eigen::Vector3d childAnchorWorld
        = pair.child.getWorldTransform() * childAnchor;
    return childAnchorWorld - parentAnchorWorld;
  };

  world.step();

  ASSERT_TRUE(registry.get<sx::comps::JointState>(jointEntity).broken);
  EXPECT_NEAR(anchorDelta().dot(sliderAxis), targetSpeed * dt, 1e-6);
  const Eigen::Isometry3d savedParentTransform
      = pair.parent.getWorldTransform();
  const Eigen::Isometry3d savedChildTransform = pair.child.getWorldTransform();
  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot = restored.getMultibody("floating_pair");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredParent = restoredRobot->getLink("parent");
  ASSERT_TRUE(restoredParent.has_value());
  auto restoredChild = restoredRobot->getLink("child");
  ASSERT_TRUE(restoredChild.has_value());

  auto& restoredRegistry = dart::simulation::detail::registryOf(restored);
  entt::entity restoredJointEntity = entt::null;
  auto jointView = restoredRegistry.view<sx::comps::JointModel>();
  for (const entt::entity entity : jointView) {
    if (jointView.get<sx::comps::JointModel>(entity).name
        == "serialized_generated_movable_slider") {
      restoredJointEntity = entity;
      break;
    }
  }
  ASSERT_TRUE(restoredJointEntity != entt::null);
  auto& restoredJointModel
      = restoredRegistry.get<sx::comps::JointModel>(restoredJointEntity);
  auto& restoredJointState
      = restoredRegistry.get<sx::comps::JointState>(restoredJointEntity);
  auto& restoredJointActuation
      = restoredRegistry.get<sx::comps::JointActuation>(restoredJointEntity);
  ASSERT_TRUE(restoredJointState.broken);
  EXPECT_EQ(restoredJointModel.type, sx::comps::JointType::Prismatic);
  EXPECT_EQ(
      restoredJointModel.parentLink,
      sx::detail::toRegistryEntity(restoredParent->getEntity()));
  EXPECT_EQ(
      restoredJointModel.childLink,
      sx::detail::toRegistryEntity(restoredChild->getEntity()));
  EXPECT_LT((restoredJointModel.axis - sliderAxis).norm(), 1e-12);
  EXPECT_EQ(
      restoredJointActuation.actuatorType, sx::comps::ActuatorType::Velocity);
  ASSERT_EQ(restoredJointActuation.commandVelocity.size(), 1);
  EXPECT_DOUBLE_EQ(restoredJointActuation.commandVelocity[0], targetSpeed);
  ASSERT_EQ(restoredJointModel.limits.effortLower.size(), 1);
  ASSERT_EQ(restoredJointModel.limits.effortUpper.size(), 1);
  EXPECT_DOUBLE_EQ(restoredJointModel.limits.effortLower[0], -1000.0);
  EXPECT_DOUBLE_EQ(restoredJointModel.limits.effortUpper[0], 1000.0);
  EXPECT_TRUE(restoredJointModel.hasRigidBodyFixedJointAnchors);
  EXPECT_LT(
      (restoredJointModel.rigidBodyFixedJointLocalAnchorParent - parentAnchor)
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredJointModel.rigidBodyFixedJointLocalAnchorChild - childAnchor)
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredParent->getWorldTransform().translation()
       - savedParentTransform.translation())
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredChild->getWorldTransform().translation()
       - savedChildTransform.translation())
          .norm(),
      1e-12);

  const auto restoredAnchorDelta = [&]() -> Eigen::Vector3d {
    const Eigen::Vector3d parentAnchorWorld
        = restoredParent->getWorldTransform() * parentAnchor;
    const Eigen::Vector3d childAnchorWorld
        = restoredChild->getWorldTransform() * childAnchor;
    return childAnchorWorld - parentAnchorWorld;
  };
  const auto sliderAxisWorld = [&]() {
    return restoredParent->getWorldTransform().linear() * sliderAxis;
  };
  const auto orthogonalAnchorResidual = [&]() {
    const Eigen::Vector3d delta = restoredAnchorDelta();
    const Eigen::Vector3d axis = sliderAxisWorld();
    return (delta - delta.dot(axis) * axis).norm();
  };
  const auto sliderPosition = [&]() {
    return restoredAnchorDelta().dot(sliderAxisWorld());
  };
  const auto relativeRotationError = [&]() {
    return (restoredParent->getWorldTransform().linear().transpose()
                * restoredChild->getWorldTransform().linear()
            - Eigen::Matrix3d::Identity())
        .norm();
  };

  double maxBrokenOrthogonalResidual = 0.0;
  for (int k = 0; k < 20; ++k) {
    restoredParent->applyForce(
        -4.0 * Eigen::Vector3d::UnitY(),
        parentAnchor + 0.3 * Eigen::Vector3d::UnitX());
    restoredChild->applyForce(
        4.0 * Eigen::Vector3d::UnitY(),
        childAnchor - 0.3 * Eigen::Vector3d::UnitX());
    restored.step();
    maxBrokenOrthogonalResidual
        = std::max(maxBrokenOrthogonalResidual, orthogonalAnchorResidual());
  }
  ASSERT_GT(maxBrokenOrthogonalResidual, 1e-4);

  const double sliderBeforeReset = sliderPosition();
  restoredParent->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredChild->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredJointActuation.commandVelocity
      = Eigen::VectorXd::Constant(1, -targetSpeed);
  restoredJointModel.breakForce = 1e6;
  restoredJointState.broken = false;

  constexpr int resetSteps = 10;
  for (int k = 0; k < resetSteps; ++k) {
    restored.step();
  }

  ASSERT_TRUE(restoredRegistry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(
      restoredJointEntity));
  const auto& restoredConfig
      = restoredRegistry.get<dvbd::AvbdRigidWorldPointJointConfig>(
          restoredJointEntity);
  EXPECT_EQ(
      restoredConfig.linearAxisMask, dvbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(restoredConfig.angularAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(restoredConfig.linearAxes.col(2).dot(sliderAxis), 1.0, 1e-12);
  EXPECT_FALSE(restoredJointState.broken);
  const double resetOrthogonalResidual = orthogonalAnchorResidual();
  EXPECT_LT(resetOrthogonalResidual, maxBrokenOrthogonalResidual * 0.05);
  EXPECT_LT(resetOrthogonalResidual, 2e-3);
  EXPECT_LT(relativeRotationError(), 1e-6);
  EXPECT_NEAR(
      sliderPosition() - sliderBeforeReset,
      -targetSpeed * dt * static_cast<double>(resetSteps),
      2e-3);
}

// PLAN-104 AVBD articulated fracture bridge: generated private spherical
// current-pose rows between two movable links must keep their captured anchors
// across simulation-mode binary save/load before breakage reset re-enables only
// the linear rows.
TEST(
    VariationalIntegration,
    AvbdSphericalPointJointCurrentPoseExtractorBreakageSurvivesSaveLoad)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  registry.emplace<sx::comps::Name>(
      jointEntity, "serialized_generated_movable_spherical");
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.name = "serialized_generated_movable_spherical";
  joint.type = sx::comps::JointType::Spherical;
  joint.parentLink = sx::detail::toRegistryEntity(pair.parent.getEntity());
  joint.childLink = sx::detail::toRegistryEntity(pair.child.getEntity());
  joint.breakForce = 1e-18;

  EXPECT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  world.enterSimulationMode();
  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, 0u);
  const Eigen::Vector3d parentAnchor = config.localAnchorA;
  const Eigen::Vector3d childAnchor = config.localAnchorB;
  const Eigen::Matrix3d capturedRelativeRotation
      = pair.parent.getWorldTransform().linear().transpose()
        * pair.child.getWorldTransform().linear();

  const auto anchorResidual = [&]() {
    const Eigen::Vector3d parentAnchorWorld
        = pair.parent.getWorldTransform() * parentAnchor;
    const Eigen::Vector3d childAnchorWorld
        = pair.child.getWorldTransform() * childAnchor;
    return (childAnchorWorld - parentAnchorWorld).norm();
  };
  const auto applyOpposingOffsetForces = [&]() {
    pair.parent.applyForce(
        -4.0 * Eigen::Vector3d::UnitY(),
        parentAnchor + 0.4 * Eigen::Vector3d::UnitX());
    pair.child.applyForce(
        4.0 * Eigen::Vector3d::UnitY(),
        childAnchor - 0.4 * Eigen::Vector3d::UnitX());
  };

  applyOpposingOffsetForces();
  world.step();

  ASSERT_TRUE(registry.get<sx::comps::JointState>(jointEntity).broken);
  EXPECT_LT(anchorResidual(), 1e-6);
  const Eigen::Isometry3d savedParentTransform
      = pair.parent.getWorldTransform();
  const Eigen::Isometry3d savedChildTransform = pair.child.getWorldTransform();
  std::stringstream data;
  world.saveBinary(data);

  sx::World restored;
  restored.loadBinary(data);
  restored.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

  auto restoredRobot = restored.getMultibody("floating_pair");
  ASSERT_TRUE(restoredRobot.has_value());
  auto restoredParent = restoredRobot->getLink("parent");
  ASSERT_TRUE(restoredParent.has_value());
  auto restoredChild = restoredRobot->getLink("child");
  ASSERT_TRUE(restoredChild.has_value());

  auto& restoredRegistry = dart::simulation::detail::registryOf(restored);
  entt::entity restoredJointEntity = entt::null;
  auto jointView = restoredRegistry.view<sx::comps::JointModel>();
  for (const entt::entity entity : jointView) {
    if (jointView.get<sx::comps::JointModel>(entity).name
        == "serialized_generated_movable_spherical") {
      restoredJointEntity = entity;
      break;
    }
  }
  ASSERT_TRUE(restoredJointEntity != entt::null);
  auto& restoredJointModel
      = restoredRegistry.get<sx::comps::JointModel>(restoredJointEntity);
  auto& restoredJointState
      = restoredRegistry.get<sx::comps::JointState>(restoredJointEntity);
  ASSERT_TRUE(restoredJointState.broken);
  EXPECT_EQ(restoredJointModel.type, sx::comps::JointType::Spherical);
  EXPECT_EQ(
      restoredJointModel.parentLink,
      sx::detail::toRegistryEntity(restoredParent->getEntity()));
  EXPECT_EQ(
      restoredJointModel.childLink,
      sx::detail::toRegistryEntity(restoredChild->getEntity()));
  EXPECT_TRUE(restoredJointModel.hasRigidBodyFixedJointAnchors);
  EXPECT_LT(
      (restoredJointModel.rigidBodyFixedJointLocalAnchorParent - parentAnchor)
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredJointModel.rigidBodyFixedJointLocalAnchorChild - childAnchor)
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredParent->getWorldTransform().matrix()
       - savedParentTransform.matrix())
          .norm(),
      1e-12);
  EXPECT_LT(
      (restoredChild->getWorldTransform().matrix()
       - savedChildTransform.matrix())
          .norm(),
      1e-12);

  const auto restoredAnchorResidual = [&]() {
    const Eigen::Vector3d parentAnchorWorld
        = restoredParent->getWorldTransform() * parentAnchor;
    const Eigen::Vector3d childAnchorWorld
        = restoredChild->getWorldTransform() * childAnchor;
    return (childAnchorWorld - parentAnchorWorld).norm();
  };
  const auto restoredRelativeRotationChange = [&]() {
    const Eigen::Matrix3d relativeRotation
        = restoredParent->getWorldTransform().linear().transpose()
          * restoredChild->getWorldTransform().linear();
    return (relativeRotation - capturedRelativeRotation).norm();
  };
  const auto applyRestoredOpposingOffsetForces = [&]() {
    restoredParent->applyForce(
        -4.0 * Eigen::Vector3d::UnitY(),
        parentAnchor + 0.4 * Eigen::Vector3d::UnitX());
    restoredChild->applyForce(
        4.0 * Eigen::Vector3d::UnitY(),
        childAnchor - 0.4 * Eigen::Vector3d::UnitX());
  };

  double maxBrokenAnchorResidual = 0.0;
  double maxBrokenRotationChange = 0.0;
  for (int k = 0; k < 20; ++k) {
    applyRestoredOpposingOffsetForces();
    restored.step();
    maxBrokenAnchorResidual
        = std::max(maxBrokenAnchorResidual, restoredAnchorResidual());
    maxBrokenRotationChange
        = std::max(maxBrokenRotationChange, restoredRelativeRotationChange());
  }
  ASSERT_GT(maxBrokenAnchorResidual, 1e-4);
  ASSERT_GT(maxBrokenRotationChange, 1e-4);

  restoredParent->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredChild->getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  restoredJointModel.breakForce = 1e6;
  restoredJointState.broken = false;

  restored.step();

  ASSERT_TRUE(restoredRegistry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(
      restoredJointEntity));
  const auto& restoredConfig
      = restoredRegistry.get<dvbd::AvbdRigidWorldPointJointConfig>(
          restoredJointEntity);
  EXPECT_EQ(restoredConfig.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(restoredConfig.angularAxisMask, 0u);
  EXPECT_FALSE(restoredJointState.broken);
  EXPECT_LT(restoredAnchorResidual(), 1e-6);
  EXPECT_GT(restoredRelativeRotationChange(), 1e-4);
}

// PLAN-104 AVBD articulated fracture bridge: private spherical point-joint
// current-pose extraction also needs break/reset coverage between two movable
// same-multibody links. Resetting the broken joint should re-enable only the
// linear anchor rows while preserving the free relative orientation.
TEST(
    VariationalIntegration,
    AvbdSphericalPointJointCurrentPoseExtractorResetReengagesMovablePair)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  world.setTimeStep(0.005);

  FloatingLinkPair pair = addFloatingLinkPair(world);
  Eigen::VectorXd childPose = Eigen::VectorXd::Zero(6);
  childPose.head<3>() = Eigen::Vector3d(0.3, 0.0, 0.0);
  pair.child.getParentJoint().setPosition(childPose);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::JointModel>(jointEntity);
  registry.emplace<sx::comps::JointState>(jointEntity);
  registry.emplace<sx::comps::JointActuation>(jointEntity);
  joint.type = sx::comps::JointType::Spherical;
  joint.parentLink = sx::detail::toRegistryEntity(pair.parent.getEntity());
  joint.childLink = sx::detail::toRegistryEntity(pair.child.getEntity());
  joint.breakForce = 1e-18;

  EXPECT_FALSE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  world.enterSimulationMode();
  ASSERT_TRUE(
      registry.all_of<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity));

  const auto& config
      = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, dvbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, 0u);
  EXPECT_TRUE(std::isinf(config.startStiffness));
  EXPECT_TRUE(std::isinf(config.maxStiffness));

  const Eigen::Vector3d parentAnchor = config.localAnchorA;
  const Eigen::Vector3d childAnchor = config.localAnchorB;
  const Eigen::Matrix3d capturedRelativeRotation
      = pair.parent.getWorldTransform().linear().transpose()
        * pair.child.getWorldTransform().linear();
  const auto anchorResidual = [&]() {
    const Eigen::Vector3d parentAnchorWorld
        = pair.parent.getWorldTransform() * parentAnchor;
    const Eigen::Vector3d childAnchorWorld
        = pair.child.getWorldTransform() * childAnchor;
    return (childAnchorWorld - parentAnchorWorld).norm();
  };
  const auto relativeRotationChange = [&]() {
    const Eigen::Matrix3d relativeRotation
        = pair.parent.getWorldTransform().linear().transpose()
          * pair.child.getWorldTransform().linear();
    return (relativeRotation - capturedRelativeRotation).norm();
  };
  const auto applyOpposingOffsetForces = [&]() {
    pair.parent.applyForce(
        -4.0 * Eigen::Vector3d::UnitY(),
        parentAnchor + 0.4 * Eigen::Vector3d::UnitX());
    pair.child.applyForce(
        4.0 * Eigen::Vector3d::UnitY(),
        childAnchor - 0.4 * Eigen::Vector3d::UnitX());
  };

  applyOpposingOffsetForces();
  world.step();

  auto& liveJointModel = registry.get<sx::comps::JointModel>(jointEntity);
  auto& liveJointState = registry.get<sx::comps::JointState>(jointEntity);
  ASSERT_TRUE(liveJointState.broken);
  EXPECT_LT(anchorResidual(), 1e-6);

  double maxBrokenAnchorResidual = 0.0;
  double maxBrokenRotationChange = 0.0;
  for (int k = 0; k < 20; ++k) {
    applyOpposingOffsetForces();
    world.step();
    maxBrokenAnchorResidual
        = std::max(maxBrokenAnchorResidual, anchorResidual());
    maxBrokenRotationChange
        = std::max(maxBrokenRotationChange, relativeRotationChange());
  }
  ASSERT_GT(maxBrokenAnchorResidual, 1e-4);
  ASSERT_GT(maxBrokenRotationChange, 1e-4);

  pair.parent.getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  pair.child.getParentJoint().setVelocity(Eigen::VectorXd::Zero(6));
  liveJointModel.breakForce = 1e6;
  liveJointState.broken = false;

  world.step();

  EXPECT_FALSE(liveJointState.broken);
  EXPECT_LT(anchorResidual(), 1e-6);
  EXPECT_GT(relativeRotationChange(), 1e-4);
}

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
      dart::simulation::detail::registryOf(world),
      structureOf(world),
      world.getGravity(),
      dt,
      state);

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
      dart::simulation::detail::registryOf(world),
      structureOf(world),
      gravity,
      dt,
      state);

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
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});

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
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
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
// EXPERIMENTAL SPIKE (PLAN-084 contact-roadmap gate 2): in-loop compliant
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

  auto& registry = dart::simulation::detail::registryOf(world);
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

    auto& registry = dart::simulation::detail::registryOf(world);
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
// PLAN-084 Phase C rung C2 -- compliant ground contact via a real, configurable
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

  auto& registry = dart::simulation::detail::registryOf(world);
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

    auto& registry = dart::simulation::detail::registryOf(world);
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
    auto& registry = dart::simulation::detail::registryOf(world);
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

  // Negative damping would inject energy on the cadence-0 path; reject it the
  // same way the AL solver's normalizeGroundContact does.
  sxc::VariationalGroundContact negativeDamping;
  negativeDamping.stiffness = 1.0;
  negativeDamping.dampingCoefficient = -1.0;
  EXPECT_ANY_THROW(
      (void)sxc::makeVariationalGroundContactHook(negativeDamping));
}

// PLAN-084 Phase C rung C1 -- lagged regularized-Coulomb friction. A block
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

    auto& registry = dart::simulation::detail::registryOf(world);
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

// PLAN-084 Phase C rung C3 -- augmented-Lagrangian contact. The per-contact
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

    auto& registry = dart::simulation::detail::registryOf(world);
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

// PLAN-084 Phase C: compliant ground contact reaches the integrator through the
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
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
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

// PLAN-084 Phase C: the World-surface ground-contact config persists across
// binary save/load. comps::VariationalContact is a serialized Property
// component -- including its link-index parallel array via the generic
// POD-vector serialization path -- so a saved contact scene reloads with its
// contact intact. The reloaded slider rests on the plane instead of tunnelling
// through it (which it would under the prior runtime-only Cache config).
TEST(VariationalGroundContact, ConfigRoundTripsThroughBinarySaveLoad)
{
  const double mass = 1.0;
  const double g = 9.81;
  const double dt = 1.0e-3;
  const double k = 1.0e3 * mass * g;
  const double damping = 200.0;
  const double mu = 0.25;
  const Eigen::Vector3d localPoint(
      0.05, 0.0, 0.0); // lateral; same Z as origin.
  const int steps = 3000;

  sx::World reference;
  reference.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  auto robot = reference.addMultibody("slider");
  auto base = robot.addLink("base"); // link index 0.
  sx::JointSpec spec;
  spec.name = "rail";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto carriage = robot.addLink("carriage", base, spec); // link index 1.
  carriage.setMass(mass);
  carriage.setInertia(Eigen::Vector3d(0.01, 0.01, 0.01).asDiagonal());
  reference.setTimeStep(dt);
  reference.enterSimulationMode();
  carriage.getParentJoint().setPosition(
      Eigen::VectorXd::Constant(1, 0.02)); // start above the plane.
  reference.updateKinematics();
  robot.setGroundContact(
      Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d::Zero(),
      k,
      mu,
      /*frictionRegularization=*/1.0e-4,
      damping);
  robot.addGroundContactPoint(carriage, localPoint);

  // Save with the contact configured (before stepping): the round-trip must
  // restore the configuration, not the resulting rest state.
  std::stringstream buffer(std::ios::in | std::ios::out | std::ios::binary);
  reference.saveBinary(buffer);

  sx::World loaded;
  buffer.seekg(0);
  loaded.loadBinary(buffer);
  loaded.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  ASSERT_TRUE(loaded.getMultibody("slider").has_value());

  // The contact config came back field-for-field, including the std::vector
  // link-index / local-position parallel arrays.
  auto& registry = dart::simulation::detail::registryOf(loaded);
  auto view = registry.view<sx::comps::VariationalContact>();
  ASSERT_EQ(view.size(), 1u);
  const auto& cfg = registry.get<sx::comps::VariationalContact>(*view.begin());
  EXPECT_TRUE(cfg.planeNormal.isApprox(Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(cfg.planePoint.isApprox(Eigen::Vector3d::Zero()));
  EXPECT_EQ(cfg.stiffness, k);
  EXPECT_EQ(cfg.frictionCoefficient, mu);
  EXPECT_EQ(cfg.dampingCoefficient, damping);
  ASSERT_EQ(cfg.pointLinkIndices.size(), 1u);
  EXPECT_EQ(cfg.pointLinkIndices[0], 1u); // the carriage link.
  ASSERT_EQ(cfg.pointLocalPositions.size(), 1u);
  EXPECT_TRUE(cfg.pointLocalPositions[0].isApprox(localPoint));

  // And behaviorally: the reloaded slider rests on the plane (does not tunnel).
  for (int step = 0; step < steps; ++step) {
    loaded.step();
  }
  const double finalZ
      = loaded.getMultibody("slider")->getJoints()[0].getPosition()[0];
  EXPECT_TRUE(std::isfinite(finalZ));
  EXPECT_LT(finalZ, 0.0);   // settled in contact, below the plane.
  EXPECT_GT(finalZ, -0.05); // held near the plane, not free-fallen through.
}

// PLAN-084 Phase C rung C3 through the World surface: a nonzero dual-update
// cadence on Multibody::setGroundContact enables the augmented-Lagrangian rung
// on the world.step() path (duals persisted in VariationalContactDualState,
// advanced every `cadence` steps by the stage). The AL run centers the contact
// at ~0 penetration (the dual carries the weight), versus the C2 compliant
// run's steady -mg/k offset -- proving C3 is reachable from World::step(), not
// only the compute-API solver.
TEST(VariationalGroundContact, WorldSurfaceAugmentedLagrangianCentersContact)
{
  const double mass = 1.0;
  const double g = 9.81;
  const double dt = 1.0e-3;
  const double k = 1.0e3 * mass * g; // soft: pure-penalty rest is mg/k.
  const int steps = 1500;
  const int window = 500;
  const double penaltyPenetration = mass * g / k;

  const auto run = [&](std::size_t dualUpdateCadence) {
    sx::World world;
    world.setMultibodyOptions(
        {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
    auto carriage = addVerticalSlider(world, mass);
    world.setTimeStep(dt);
    world.enterSimulationMode();
    carriage.getParentJoint().setPosition(
        Eigen::VectorXd::Constant(
            1, -penaltyPenetration)); // pure-penalty rest.
    world.updateKinematics();
    auto robot = world.getMultibody("slider");
    robot->setGroundContact(
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::Zero(),
        k,
        /*frictionCoefficient=*/0.0,
        /*frictionRegularization=*/1.0e-4,
        /*dampingCoefficient=*/200.0, // ~critical (2*sqrt(k*m)) to settle.
        dualUpdateCadence);
    robot->addGroundContactPoint(carriage, Eigen::Vector3d::Zero());
    double meanZ = 0.0;
    int count = 0;
    for (int step = 0; step < steps; ++step) {
      world.step();
      if (step >= steps - window) {
        meanZ += carriage.getParentJoint().getPosition()[0];
        ++count;
      }
    }
    return meanZ / count;
  };

  const double penaltyMeanZ = run(/*dualUpdateCadence=*/0);
  const double alMeanZ = run(/*dualUpdateCadence=*/20);

  // C2 (cadence 0) rests in steady penetration (~ -mg/k).
  EXPECT_LT(penaltyMeanZ, -0.5 * penaltyPenetration);
  // C3 (cadence 20) centers the contact at ~0: the dual carries the weight.
  EXPECT_LT(std::abs(alMeanZ), 0.3 * penaltyPenetration);
}

// PLAN-084 Phase C: the C3 augmented-Lagrangian dual state round-trips through
// binary save/load so an AL contact scene resumes bit-identically. The duals
// (and the cadence counter) are warm-started across steps; persisting them in
// VariationalContactDualState means a mid-rollout save/load continues the exact
// same trajectory rather than restarting the dual ascent from zero.
TEST(VariationalGroundContact, AugmentedLagrangianDualStateRoundTrips)
{
  const double mass = 1.0;
  const double g = 9.81;
  const double dt = 1.0e-3;
  const double k = 1.0e3 * mass * g;
  const double penaltyPenetration = mass * g / k;

  sx::World reference;
  reference.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  auto carriage = addVerticalSlider(reference, mass);
  reference.setTimeStep(dt);
  reference.enterSimulationMode();
  carriage.getParentJoint().setPosition(
      Eigen::VectorXd::Constant(1, -penaltyPenetration));
  reference.updateKinematics();
  reference.getMultibody("slider")->setGroundContact(
      Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d::Zero(),
      k,
      /*frictionCoefficient=*/0.0,
      /*frictionRegularization=*/1.0e-4,
      /*dampingCoefficient=*/200.0,
      /*dualUpdateCadence=*/20);
  reference.getMultibody("slider")->addGroundContactPoint(
      carriage, Eigen::Vector3d::Zero());

  // Step past several dual updates so the saved dual accumulator is non-trivial
  // and the cadence counter is mid-interval (55 % 20 = 15).
  for (int step = 0; step < 55; ++step) {
    reference.step();
  }

  std::stringstream buffer(std::ios::in | std::ios::out | std::ios::binary);
  reference.saveBinary(buffer);

  for (int step = 0; step < 100; ++step) {
    reference.step();
  }
  const double referenceFinal
      = reference.getMultibody("slider")->getJoints()[0].getPosition()[0];

  sx::World loaded;
  buffer.seekg(0);
  loaded.loadBinary(buffer);
  loaded.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  ASSERT_TRUE(loaded.getMultibody("slider").has_value());
  for (int step = 0; step < 100; ++step) {
    loaded.step();
  }
  const double loadedFinal
      = loaded.getMultibody("slider")->getJoints()[0].getPosition()[0];

  // Bit-identical: the duals + cadence counter + config + VI history all came
  // back, so the continuation is the same trajectory (no dual-ascent restart).
  EXPECT_EQ(loadedFinal, referenceFinal);
}

// PLAN-084 Phase C: the C2 default ground-contact hook (dualUpdateCadence == 0)
// applies Kelvin-Voigt normal damping, not just the penalty --
// setGroundContact's dampingCoefficient takes effect on the auto World::step()
// path, not only the AL solver. A slider dropped onto a damped plane absorbs
// the impact and barely rebounds, while the undamped penalty contact bounces
// back up. Without the damping term the two runs would be byte-identical.
TEST(VariationalGroundContact, DampingSettlesContactOnDefaultPath)
{
  const double mass = 1.0;
  const double g = 9.81;
  const double dt = 1.0e-3;
  // Soft enough that the undamped bounce is not numerically dissipated away.
  const double k = 1.0e2 * mass * g;
  const int steps = 1500;

  // Highest point the slider reaches after first crossing into the half-space.
  const auto reboundHeight = [&](double damping) {
    sx::World world;
    world.setMultibodyOptions(
        {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
    auto carriage = addVerticalSlider(world, mass);
    world.setTimeStep(dt);
    world.enterSimulationMode();
    carriage.getParentJoint().setPosition(
        Eigen::VectorXd::Constant(1, 0.05)); // start above the plane.
    world.updateKinematics();
    world.getMultibody("slider")->setGroundContact(
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::Zero(),
        k,
        /*frictionCoefficient=*/0.0,
        /*frictionRegularization=*/1.0e-4,
        damping,
        /*dualUpdateCadence=*/0);
    world.getMultibody("slider")->addGroundContactPoint(
        carriage, Eigen::Vector3d::Zero());
    bool contacted = false;
    double maxRebound = -std::numeric_limits<double>::infinity();
    for (int step = 0; step < steps; ++step) {
      world.step();
      const double z = carriage.getParentJoint().getPosition()[0];
      if (z < 0.0) {
        contacted = true;
      }
      if (contacted && z > maxRebound) {
        maxRebound = z;
      }
    }
    return maxRebound;
  };

  const double undampedRebound = reboundHeight(0.0);
  const double dampedRebound
      = reboundHeight(200.0); // > critical (2*sqrt(k*m)).
  // The undamped penalty contact rebounds back up substantially.
  EXPECT_GT(undampedRebound, 5.0e-3);
  // Damping (now applied on the cadence-0 path) absorbs the impact, so the
  // slider barely rebounds. (Without the damping term `dampedRebound` would
  // equal `undampedRebound`.)
  EXPECT_LT(dampedRebound, 0.25 * undampedRebound);
}

// PLAN-084 Phase C: addGroundContactPoint rejects a Link from a different
// World. Separate registries can reuse raw entity ids, so without the
// getWorld() guard a foreign link could alias a numerically-equal link and
// register contact on the wrong body.
TEST(VariationalGroundContact, RejectsContactPointFromForeignWorld)
{
  sx::World world;
  world.setMultibodyOptions(
      {.integrationFamily = sx::MultibodyIntegrationFamily::Variational});
  auto robot = world.addMultibody("slider");
  auto base = robot.addLink("base");
  robot.setGroundContact(
      Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero(), 1.0e3);

  // A link that belongs to a different World.
  sx::World other;
  auto otherRobot = other.addMultibody("other");
  auto foreignLink = otherRobot.addLink("foreign");

  EXPECT_THROW(
      robot.addGroundContactPoint(foreignLink, Eigen::Vector3d::Zero()),
      sx::InvalidArgumentException);
  // A link from this World still registers fine.
  EXPECT_NO_THROW(robot.addGroundContactPoint(base, Eigen::Vector3d::Zero()));
}

// PLAN-084 Phase C link-vs-link contact (first slice): sphere-sphere
// self-contact. A link sliding toward a fixed sphere on the base is stopped by
// compliant sphere-sphere contact -- the moving sphere does not pass through
// the fixed one. Exercises makeVariationalLinkSphereContactHook (the simplest
// link-vs-link query, the first slice of the gate-1 contact-query workstream).
TEST(VariationalLinkContact, SphereContactStopsSlidingLink)
{
  const double dt = 1.0e-3;
  const int steps = 1500;
  const double k = 1.0e3;
  const double radius = 0.1;
  const double fixedX = 0.5; // base-sphere center x.
  const double v0 = 1.0;     // initial slide speed toward the fixed sphere.
  const double touchX = fixedX - 2.0 * radius; // 0.3: where the spheres touch.

  sx::World world;
  auto robot = world.addMultibody("slider");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "rail";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitX();
  auto carriage = robot.addLink("carriage", base, spec);
  carriage.setMass(1.0);
  carriage.setInertia(Eigen::Vector3d(0.01, 0.01, 0.01).asDiagonal());
  world.setTimeStep(dt);
  world.enterSimulationMode();
  carriage.getParentJoint().setVelocity(
      Eigen::VectorXd::Constant(1, v0)); // slide toward the fixed sphere.
  world.updateKinematics();

  const auto& structure = structureOf(world);
  sxc::VariationalSphereContactPair pair;
  pair.linkA = 0; // base (root), fixed.
  pair.centerA = Eigen::Vector3d(fixedX, 0.0, 0.0);
  pair.radiusA = radius;
  pair.linkB = structure.links.size() - 1; // carriage (sphere at its origin).
  pair.centerB = Eigen::Vector3d::Zero();
  pair.radiusB = radius;
  const auto hook = sxc::makeVariationalLinkSphereContactHook(k, 20.0, {pair});

  auto& registry = dart::simulation::detail::registryOf(world);
  const Eigen::Vector3d gravity
      = world.getGravity(); // -Z; irrelevant to the x-only carriage.
  sxc::MultibodyVariationalState state;
  double maxX = 0.0;
  bool finite = true;
  bool converged = true;
  for (int step = 0; step < steps; ++step) {
    const auto report = sxc::integrateMultibodyVariational(
        registry, structure, gravity, dt, state, 100, 1e-10, {}, 5, hook);
    converged = converged && report.converged;
    finite = finite && std::isfinite(report.residualNorm);
    maxX = std::max(maxX, carriage.getParentJoint().getPosition()[0]);
  }

  EXPECT_TRUE(converged);
  EXPECT_TRUE(finite);
  // The carriage slid up to the contact (~touchX)...
  EXPECT_GT(maxX, touchX - 0.05);
  // ...and sphere-sphere contact stopped it: it did not pass through the fixed
  // sphere (the overshoot is a bounded penalty penetration).
  EXPECT_LT(maxX, touchX + 0.08);
}
