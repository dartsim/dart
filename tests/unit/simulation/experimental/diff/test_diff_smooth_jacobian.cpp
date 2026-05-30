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

// FD-vs-analytic correctness gate for the contact-free smooth single-step
// Jacobian (PLAN-110 WS1). This test exercises the detail-only differentiable
// code path and is registered only when DART_BUILD_DIFF is ON.

#include <dart/simulation/experimental/comps/multibody.hpp>
#include <dart/simulation/experimental/detail/smooth_jacobians.hpp>
#include <dart/simulation/experimental/diff/step_derivatives.hpp>
#include <dart/simulation/experimental/multibody/multibody.hpp>
#include <dart/simulation/experimental/world.hpp>
#include <dart/simulation/experimental/world_options.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <entt/entt.hpp>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <limits>
#include <memory>
#include <vector>

#include <cmath>

namespace sim = dart::simulation::experimental;

namespace {

//==============================================================================
// Read the full state [q; q̇] of a multibody (joints in construction order).
Eigen::VectorXd readState(const sim::Multibody& mb)
{
  std::vector<double> q;
  std::vector<double> qdot;
  for (const auto& joint : mb.getJoints()) {
    const Eigen::VectorXd p = joint.getPosition();
    const Eigen::VectorXd v = joint.getVelocity();
    for (Eigen::Index i = 0; i < p.size(); ++i) {
      q.push_back(p[i]);
    }
    for (Eigen::Index i = 0; i < v.size(); ++i) {
      qdot.push_back(v[i]);
    }
  }
  Eigen::VectorXd state(q.size() + qdot.size());
  for (std::size_t i = 0; i < q.size(); ++i) {
    state[static_cast<Eigen::Index>(i)] = q[i];
  }
  for (std::size_t i = 0; i < qdot.size(); ++i) {
    state[static_cast<Eigen::Index>(q.size() + i)] = qdot[i];
  }
  return state;
}

//==============================================================================
struct PendulumState
{
  Eigen::VectorXd q;
  Eigen::VectorXd qdot;
  Eigen::VectorXd tau;
};

//==============================================================================
// Apply a state to all joints and the held effort.
void applyConfiguration(sim::Multibody& mb, const PendulumState& config)
{
  Eigen::Index offset = 0;
  for (auto joint : mb.getJoints()) {
    const auto dof = static_cast<Eigen::Index>(joint.getDOFCount());
    if (dof == 0) {
      continue;
    }
    joint.setPosition(config.q.segment(offset, dof));
    joint.setVelocity(config.qdot.segment(offset, dof));
    joint.setForce(config.tau.segment(offset, dof));
    offset += dof;
  }
}

//==============================================================================
// Finite-difference-of-STEP Jacobian: perturb each state component by h, run a
// single world.step() from the restored configuration, and central-difference
// the resulting next state. Returns ∂x'/∂x (2*ndof x 2*ndof).
//
// The build closure constructs a fresh world at the supplied configuration so
// each perturbed rollout is independent.
template <typename BuildFn>
Eigen::MatrixXd finiteDifferenceStepJacobian(
    const BuildFn& build, const PendulumState& nominal, double h)
{
  const Eigen::Index ndof = nominal.q.size();
  const Eigen::Index stateSize = 2 * ndof;

  Eigen::VectorXd nominalState(stateSize);
  nominalState.head(ndof) = nominal.q;
  nominalState.tail(ndof) = nominal.qdot;

  Eigen::MatrixXd jacobian(stateSize, stateSize);

  for (Eigen::Index k = 0; k < stateSize; ++k) {
    PendulumState plusConfig = nominal;
    PendulumState minusConfig = nominal;
    Eigen::VectorXd plusState = nominalState;
    Eigen::VectorXd minusState = nominalState;
    plusState[k] += h;
    minusState[k] -= h;
    plusConfig.q = plusState.head(ndof);
    plusConfig.qdot = plusState.tail(ndof);
    minusConfig.q = minusState.head(ndof);
    minusConfig.qdot = minusState.tail(ndof);

    auto plusWorld = build();
    auto plusMb = plusWorld->getMultibody("pendulum");
    applyConfiguration(*plusMb, plusConfig);
    plusWorld->step();
    const Eigen::VectorXd nextPlus = readState(*plusMb);

    auto minusWorld = build();
    auto minusMb = minusWorld->getMultibody("pendulum");
    applyConfiguration(*minusMb, minusConfig);
    minusWorld->step();
    const Eigen::VectorXd nextMinus = readState(*minusMb);

    jacobian.col(k) = (nextPlus - nextMinus) / (2.0 * h);
  }

  return jacobian;
}

//==============================================================================
double relativeError(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b)
{
  const double denom = std::max(
      1.0, std::max(a.cwiseAbs().maxCoeff(), b.cwiseAbs().maxCoeff()));
  return (a - b).cwiseAbs().maxCoeff() / denom;
}

//==============================================================================
// Build a single revolute pendulum that swings about the world Y axis with its
// link center of mass offset along +X, so gravity (along -Z) produces a
// nonzero, configuration-dependent torque. The base is the fixed root.
std::unique_ptr<sim::World> buildSinglePendulum()
{
  sim::WorldOptions options;
  options.differentiable = true;
  options.timeStep = 1e-3;
  auto world = std::make_unique<sim::World>(options);

  auto pendulum = world->addMultibody("pendulum");
  auto base = pendulum.addLink("base");

  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(1.0, 0.0, 0.0); // link length L = 1
  auto link = pendulum.addLink(
      "link",
      base,
      sim::JointSpec{
          .name = "hinge",
          .type = sim::JointType::Revolute,
          .axis = Eigen::Vector3d::UnitY(),
          .transformFromParent = offset});
  link.setMass(2.0);
  link.setInertia(0.05 * Eigen::Matrix3d::Identity());

  return world;
}

//==============================================================================
// Build a double revolute pendulum (two links, both hinging about Y), so the
// mass matrix is configuration dependent (nonzero dM/dq) and the velocity terms
// couple the two coordinates.
std::unique_ptr<sim::World> buildDoublePendulum()
{
  sim::WorldOptions options;
  options.differentiable = true;
  options.timeStep = 1e-3;
  auto world = std::make_unique<sim::World>(options);

  auto pendulum = world->addMultibody("pendulum");
  auto base = pendulum.addLink("base");

  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);

  auto link1 = pendulum.addLink(
      "link1",
      base,
      sim::JointSpec{
          .name = "hinge1",
          .type = sim::JointType::Revolute,
          .axis = Eigen::Vector3d::UnitY(),
          .transformFromParent = offset});
  link1.setMass(1.5);
  link1.setInertia(0.04 * Eigen::Matrix3d::Identity());

  auto link2 = pendulum.addLink(
      "link2",
      link1,
      sim::JointSpec{
          .name = "hinge2",
          .type = sim::JointType::Revolute,
          .axis = Eigen::Vector3d::UnitY(),
          .transformFromParent = offset});
  link2.setMass(1.0);
  link2.setInertia(0.03 * Eigen::Matrix3d::Identity());

  return world;
}

//==============================================================================
// Build a single spherical (ball) pendulum: one 3-DOF Spherical joint whose
// child link center of mass is offset along +X, so gravity (along -Z) produces
// a nonzero, orientation-dependent torque on the SO(3) coordinates. The base is
// the fixed root. The rotation vector is integrated on SO(3) by the forward
// step, so the position Jacobian must use the dexp/dlog blocks.
std::unique_ptr<sim::World> buildBallPendulum()
{
  sim::WorldOptions options;
  options.differentiable = true;
  options.timeStep = 1e-3;
  auto world = std::make_unique<sim::World>(options);

  auto pendulum = world->addMultibody("pendulum");
  auto base = pendulum.addLink("base");

  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(1.0, 0.0, 0.0); // link length L = 1
  auto link = pendulum.addLink(
      "link",
      base,
      sim::JointSpec{
          .name = "ball",
          .type = sim::JointType::Spherical,
          .transformFromParent = offset});
  link.setMass(2.0);
  link.setInertia(0.05 * Eigen::Matrix3d::Identity());

  return world;
}

//==============================================================================
// Build a single free-floating body: one 6-DOF Floating joint connecting a
// link to the fixed root, under gravity. The body has a diagonal inertia and a
// center of mass at the link origin. Translation integrates in the parent
// frame (R v) and orientation on SO(3), exercising both the SE(3) dexp/dlog
// blocks and the translation-vs-orientation coupling block.
std::unique_ptr<sim::World> buildFreeBody()
{
  sim::WorldOptions options;
  options.differentiable = true;
  options.timeStep = 1e-3;
  auto world = std::make_unique<sim::World>(options);

  auto body = world->addMultibody("pendulum");
  auto base = body.addLink("base");

  auto link = body.addLink(
      "link",
      base,
      sim::JointSpec{.name = "free", .type = sim::JointType::Floating});
  link.setMass(1.5);
  link.setInertia(
      (Eigen::Matrix3d() << 0.06, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.05)
          .finished());

  return world;
}

//==============================================================================
// Compute the analytic Jacobian at the multibody's current state via the
// detail entry point.
sim::StepDerivatives analyticDerivatives(
    sim::World& world, const Eigen::VectorXd& tau)
{
  auto mb = world.getMultibody("pendulum");
  // The detail function reads the multibody structure component from the
  // registry; the World exposes the registry through its internal accessor.
  return sim::detail::contactFreeStepDerivatives(
      world.getRegistry(),
      world.getRegistry()
          .get<dart::simulation::experimental::comps::MultibodyStructure>(
              mb->getEntity()),
      world.getGravity(),
      world.getTimeStep(),
      tau);
}

} // namespace

//==============================================================================
TEST(DiffSmoothJacobian, SingleRevolutePendulumMatchesFiniteDifference)
{
  // Pre-step state with q != 0, q̇ != 0, tau != 0.
  PendulumState config;
  config.q = Eigen::VectorXd::Constant(1, 0.3);
  config.qdot = Eigen::VectorXd::Constant(1, -0.7);
  config.tau = Eigen::VectorXd::Constant(1, 0.5);

  auto world = buildSinglePendulum();
  auto mb = world->getMultibody("pendulum");
  applyConfiguration(*mb, config);
  world->step(); // populate getStepDerivatives() and verify wiring.

  const sim::StepDerivatives analytic = world->getStepDerivatives();
  ASSERT_EQ(analytic.stateJacobian.rows(), 2);
  ASSERT_EQ(analytic.stateJacobian.cols(), 2);
  ASSERT_EQ(analytic.controlJacobian.rows(), 2);
  ASSERT_EQ(analytic.controlJacobian.cols(), 1);

  // The cached step derivatives must equal a direct evaluation at the same
  // pre-step state.
  auto freshWorld = buildSinglePendulum();
  auto freshMb = freshWorld->getMultibody("pendulum");
  applyConfiguration(*freshMb, config);
  const sim::StepDerivatives direct
      = analyticDerivatives(*freshWorld, config.tau);
  EXPECT_LT(relativeError(analytic.stateJacobian, direct.stateJacobian), 1e-12);

  // FD-of-step over the perturbation sweep; keep the best relative error.
  const std::array<double, 3> sweep{1e-5, 1e-6, 1e-7};
  double bestError = std::numeric_limits<double>::infinity();
  for (const double h : sweep) {
    const Eigen::MatrixXd fd
        = finiteDifferenceStepJacobian(buildSinglePendulum, config, h);
    bestError = std::min(bestError, relativeError(analytic.stateJacobian, fd));
  }
  EXPECT_LT(bestError, 1e-4) << "best FD-of-step relative error: " << bestError;

  // Closed-form oracle: for a single revolute pendulum about a horizontal axis,
  // the mass matrix is configuration independent (dM/dq = 0) and there is no
  // Coriolis coupling (dc/dq̇ = 0). The gravity generalized force is sinusoidal:
  // g(q) = A sin(q) + B cos(q), so dg/dq = A cos(q) - B sin(q) is closed form.
  // Then ∂q̇'/∂q = -dt * Minv * dg/dq. This uses only the dynamics' gravity term
  // sampled at two configs, independent of the FD-of-terms and FD-of-step
  // paths.
  const double Minv = 1.0 / mb->getMassMatrix()(0, 0);

  const auto gravityAt = [&](double angle) {
    auto sampleWorld = buildSinglePendulum();
    auto sampleMb = sampleWorld->getMultibody("pendulum");
    PendulumState sample;
    sample.q = Eigen::VectorXd::Constant(1, angle);
    sample.qdot = Eigen::VectorXd::Zero(1);
    sample.tau = Eigen::VectorXd::Zero(1);
    applyConfiguration(*sampleMb, sample);
    return sampleMb->getGravityForces()(0);
  };
  const double gAt0 = gravityAt(0.0);
  const double gAtHalfPi = gravityAt(M_PI / 2.0);
  const double bCoefficient = gAt0;      // g(0) = B
  const double aCoefficient = gAtHalfPi; // g(pi/2) = A
  const double q = config.q[0];
  const double dgdq = aCoefficient * std::cos(q) - bCoefficient * std::sin(q);
  const double oracleVelPos = -world->getTimeStep() * Minv * dgdq;

  EXPECT_NEAR(
      analytic.stateJacobian(1, 0),
      oracleVelPos,
      1e-6 + 1e-4 * std::abs(oracleVelPos));
}

//==============================================================================
TEST(DiffSmoothJacobian, DoubleRevolutePendulumMatchesFiniteDifference)
{
  PendulumState config;
  config.q = Eigen::VectorXd(2);
  config.q << 0.4, -0.6;
  config.qdot = Eigen::VectorXd(2);
  config.qdot << 0.9, -0.5;
  config.tau = Eigen::VectorXd(2);
  config.tau << 0.7, -0.3;

  auto world = buildDoublePendulum();
  auto mb = world->getMultibody("pendulum");
  applyConfiguration(*mb, config);
  world->step();

  const sim::StepDerivatives analytic = world->getStepDerivatives();
  ASSERT_EQ(analytic.stateJacobian.rows(), 4);
  ASSERT_EQ(analytic.stateJacobian.cols(), 4);
  ASSERT_EQ(analytic.controlJacobian.rows(), 4);
  ASSERT_EQ(analytic.controlJacobian.cols(), 2);

  const std::array<double, 3> sweep{1e-5, 1e-6, 1e-7};
  double bestStateError = std::numeric_limits<double>::infinity();
  double bestControlError = std::numeric_limits<double>::infinity();
  for (const double h : sweep) {
    const Eigen::MatrixXd fdState
        = finiteDifferenceStepJacobian(buildDoublePendulum, config, h);
    bestStateError = std::min(
        bestStateError, relativeError(analytic.stateJacobian, fdState));
  }
  EXPECT_LT(bestStateError, 1e-4)
      << "best FD-of-step state relative error: " << bestStateError;

  // Control Jacobian FD: perturb tau, central-difference the next state.
  for (const double h : sweep) {
    Eigen::MatrixXd fdControl(4, 2);
    for (Eigen::Index k = 0; k < 2; ++k) {
      PendulumState plus = config;
      PendulumState minus = config;
      plus.tau[k] += h;
      minus.tau[k] -= h;

      auto plusWorld = buildDoublePendulum();
      auto plusMb = plusWorld->getMultibody("pendulum");
      applyConfiguration(*plusMb, plus);
      plusWorld->step();
      const Eigen::VectorXd nextPlus = readState(*plusMb);

      auto minusWorld = buildDoublePendulum();
      auto minusMb = minusWorld->getMultibody("pendulum");
      applyConfiguration(*minusMb, minus);
      minusWorld->step();
      const Eigen::VectorXd nextMinus = readState(*minusMb);

      fdControl.col(k) = (nextPlus - nextMinus) / (2.0 * h);
    }
    bestControlError = std::min(
        bestControlError, relativeError(analytic.controlJacobian, fdControl));
  }
  EXPECT_LT(bestControlError, 1e-4)
      << "best FD-of-step control relative error: " << bestControlError;
}

//==============================================================================
//==============================================================================
TEST(DiffSmoothJacobian, BallPendulumMatchesFiniteDifference)
{
  // Pre-step state with a nontrivial rotation (q != 0) and angular velocity
  // (q̇ != 0), so the SO(3) position Jacobian is genuinely non-identity.
  PendulumState config;
  config.q = Eigen::VectorXd(3);
  config.q << 0.3, -0.5, 0.2;
  config.qdot = Eigen::VectorXd(3);
  config.qdot << 0.9, -0.4, 0.6;
  config.tau = Eigen::VectorXd(3);
  config.tau << 0.2, 0.1, -0.15;

  auto world = buildBallPendulum();
  auto mb = world->getMultibody("pendulum");
  applyConfiguration(*mb, config);
  world->step();

  const sim::StepDerivatives analytic = world->getStepDerivatives();
  ASSERT_EQ(analytic.stateJacobian.rows(), 6);
  ASSERT_EQ(analytic.stateJacobian.cols(), 6);
  ASSERT_EQ(analytic.controlJacobian.rows(), 6);
  ASSERT_EQ(analytic.controlJacobian.cols(), 3);

  // The cached step derivatives must equal a direct evaluation at the same
  // pre-step state.
  auto freshWorld = buildBallPendulum();
  auto freshMb = freshWorld->getMultibody("pendulum");
  applyConfiguration(*freshMb, config);
  const sim::StepDerivatives direct
      = analyticDerivatives(*freshWorld, config.tau);
  EXPECT_LT(relativeError(analytic.stateJacobian, direct.stateJacobian), 1e-12);

  // The SO(3) position block ∂q'/∂q is the top-left 3x3 of the state Jacobian.
  // A naive Euclidean integration would leave it equal to identity at this
  // nonzero rotation; assert it is genuinely non-trivial.
  const Eigen::Matrix3d posBlock = analytic.stateJacobian.topLeftCorner(3, 3);
  EXPECT_GT(
      (posBlock - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff(), 1e-6)
      << "SO(3) position block collapsed to identity:\n"
      << posBlock;

  // FD-of-step over the perturbation sweep for both Jacobians.
  const std::array<double, 3> sweep{1e-5, 1e-6, 1e-7};
  double bestStateError = std::numeric_limits<double>::infinity();
  for (const double h : sweep) {
    const Eigen::MatrixXd fd
        = finiteDifferenceStepJacobian(buildBallPendulum, config, h);
    bestStateError
        = std::min(bestStateError, relativeError(analytic.stateJacobian, fd));
  }
  EXPECT_LT(bestStateError, 1e-4)
      << "best FD-of-step state relative error: " << bestStateError;

  double bestControlError = std::numeric_limits<double>::infinity();
  for (const double h : sweep) {
    Eigen::MatrixXd fdControl(6, 3);
    for (Eigen::Index k = 0; k < 3; ++k) {
      PendulumState plus = config;
      PendulumState minus = config;
      plus.tau[k] += h;
      minus.tau[k] -= h;

      auto plusWorld = buildBallPendulum();
      auto plusMb = plusWorld->getMultibody("pendulum");
      applyConfiguration(*plusMb, plus);
      plusWorld->step();
      const Eigen::VectorXd nextPlus = readState(*plusMb);

      auto minusWorld = buildBallPendulum();
      auto minusMb = minusWorld->getMultibody("pendulum");
      applyConfiguration(*minusMb, minus);
      minusWorld->step();
      const Eigen::VectorXd nextMinus = readState(*minusMb);

      fdControl.col(k) = (nextPlus - nextMinus) / (2.0 * h);
    }
    bestControlError = std::min(
        bestControlError, relativeError(analytic.controlJacobian, fdControl));
  }
  EXPECT_LT(bestControlError, 1e-4)
      << "best FD-of-step control relative error: " << bestControlError;
}

//==============================================================================
TEST(DiffSmoothJacobian, FreeBodyMatchesFiniteDifference)
{
  // 6-DOF state: [translation; rotation vector] positions and [linear; angular]
  // body-twist velocities, with a nonzero orientation and angular velocity so
  // the SE(3) blocks and the translation-orientation coupling are exercised.
  PendulumState config;
  config.q = Eigen::VectorXd(6);
  config.q << 0.1, -0.2, 0.3, 0.4, -0.6, 0.25;
  config.qdot = Eigen::VectorXd(6);
  config.qdot << 0.5, -0.3, 0.2, 0.9, -0.4, 0.6;
  config.tau = Eigen::VectorXd(6);
  config.tau << 0.05, -0.1, 0.15, 0.2, 0.1, -0.15;

  auto world = buildFreeBody();
  auto mb = world->getMultibody("pendulum");
  applyConfiguration(*mb, config);
  world->step();

  const sim::StepDerivatives analytic = world->getStepDerivatives();
  ASSERT_EQ(analytic.stateJacobian.rows(), 12);
  ASSERT_EQ(analytic.stateJacobian.cols(), 12);
  ASSERT_EQ(analytic.controlJacobian.rows(), 12);
  ASSERT_EQ(analytic.controlJacobian.cols(), 6);

  auto freshWorld = buildFreeBody();
  auto freshMb = freshWorld->getMultibody("pendulum");
  applyConfiguration(*freshMb, config);
  const sim::StepDerivatives direct
      = analyticDerivatives(*freshWorld, config.tau);
  EXPECT_LT(relativeError(analytic.stateJacobian, direct.stateJacobian), 1e-12);

  // The SE(3) orientation position block (rows/cols 3..5 of the position-q
  // quadrant) must differ from identity at this nonzero orientation, and the
  // translation-vs-orientation coupling block (∂p'/∂θ, rows 0..2 vs cols 3..5)
  // must be non-trivial.
  const Eigen::Matrix3d orientationBlock
      = analytic.stateJacobian.block(3, 3, 3, 3);
  EXPECT_GT(
      (orientationBlock - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff(),
      1e-6)
      << "SE(3) orientation position block collapsed to identity:\n"
      << orientationBlock;
  const Eigen::Matrix3d couplingBlock
      = analytic.stateJacobian.block(0, 3, 3, 3);
  EXPECT_GT(couplingBlock.cwiseAbs().maxCoeff(), 1e-7)
      << "SE(3) translation-orientation coupling block is zero:\n"
      << couplingBlock;

  const std::array<double, 3> sweep{1e-5, 1e-6, 1e-7};
  double bestStateError = std::numeric_limits<double>::infinity();
  for (const double h : sweep) {
    const Eigen::MatrixXd fd
        = finiteDifferenceStepJacobian(buildFreeBody, config, h);
    bestStateError
        = std::min(bestStateError, relativeError(analytic.stateJacobian, fd));
  }
  EXPECT_LT(bestStateError, 1e-4)
      << "best FD-of-step state relative error: " << bestStateError;

  double bestControlError = std::numeric_limits<double>::infinity();
  for (const double h : sweep) {
    Eigen::MatrixXd fdControl(12, 6);
    for (Eigen::Index k = 0; k < 6; ++k) {
      PendulumState plus = config;
      PendulumState minus = config;
      plus.tau[k] += h;
      minus.tau[k] -= h;

      auto plusWorld = buildFreeBody();
      auto plusMb = plusWorld->getMultibody("pendulum");
      applyConfiguration(*plusMb, plus);
      plusWorld->step();
      const Eigen::VectorXd nextPlus = readState(*plusMb);

      auto minusWorld = buildFreeBody();
      auto minusMb = minusWorld->getMultibody("pendulum");
      applyConfiguration(*minusMb, minus);
      minusWorld->step();
      const Eigen::VectorXd nextMinus = readState(*minusMb);

      fdControl.col(k) = (nextPlus - nextMinus) / (2.0 * h);
    }
    bestControlError = std::min(
        bestControlError, relativeError(analytic.controlJacobian, fdControl));
  }
  EXPECT_LT(bestControlError, 1e-4)
      << "best FD-of-step control relative error: " << bestControlError;
}
