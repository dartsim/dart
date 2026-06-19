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

// FRAMEWORK-NEUTRAL multi-step differentiable rollout correctness gate
// (PLAN-110 rollout item). This test drives the PUBLIC diff::rollout entry: it
// rolls a stable (no-impact) free-fall scene forward over several steps,
// composing the per-step Jacobians into a whole-rollout VJP, then asserts that
// {initialStateGrad, controlGrads} matches a CENTRAL finite-difference of the
// WHOLE rollout for a scalar loss = sum of the final state. The scene stays in
// a single smooth regime (no contacts / mode switches), so finite differencing
// is well conditioned. Registered only when DART_BUILD_DIFF is ON.

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/contact.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/common/exceptions.hpp>
#include <dart/simulation/diff/rollout.hpp>
#include <dart/simulation/multibody/multibody.hpp>
#include <dart/simulation/world.hpp>
#include <dart/simulation/world_options.hpp>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <cmath>

namespace sx = dart::simulation;

namespace {

constexpr double kTimeStep = 1e-3;
constexpr std::size_t kSteps = 8;

//==============================================================================
// Two free-falling spheres, well above any ground, BoxedLcp + differentiable.
// They never touch the ground over the rollout horizon, so every step stays in
// the smooth (contact-free) regime and the whole rollout is a well-conditioned
// smooth function of the initial state and controls.
std::unique_ptr<sx::World> buildFreeFallScene()
{
  sx::WorldOptions options;
  options.timeStep = kTimeStep;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.differentiable = true;
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  auto world = std::make_unique<sx::World>(options);

  // A static ground far below: it never participates in a contact over the
  // horizon, so the rollout is purely smooth free-fall dynamics.
  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -50.0);
  auto ground = world->addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));
  ground.setFriction(0.0);

  sx::RigidBodyOptions sphereA;
  sphereA.mass = 2.0;
  sphereA.position = Eigen::Vector3d(0.0, 0.0, 5.0);
  sphereA.linearVelocity = Eigen::Vector3d(0.1, -0.2, 0.3);
  auto bodyA = world->addRigidBody("sphereA", sphereA);
  bodyA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  bodyA.setFriction(0.0);

  sx::RigidBodyOptions sphereB;
  sphereB.mass = 1.5;
  sphereB.position = Eigen::Vector3d(2.0, 1.0, 6.0);
  sphereB.linearVelocity = Eigen::Vector3d(-0.05, 0.15, -0.1);
  auto bodyB = world->addRigidBody("sphereB", sphereB);
  bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  bodyB.setFriction(0.0);

  return world;
}

//==============================================================================
// Mixed-family differentiable state vectors are full dense World vectors, while
// the current analytic Jacobian implementation is family-specific. Until
// full-world Jacobians land, public rollout rejects the mixed scene instead of
// returning gradients with mismatched shapes.
std::unique_ptr<sx::World> buildMixedRigidMultibodyScene()
{
  sx::WorldOptions options;
  options.timeStep = kTimeStep;
  options.gravity = Eigen::Vector3d::Zero();
  options.differentiable = true;
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  auto world = std::make_unique<sx::World>(options);

  sx::RigidBodyOptions bodyOptions;
  bodyOptions.mass = 1.0;
  bodyOptions.position = Eigen::Vector3d(0.0, 0.0, 1.0);
  bodyOptions.linearVelocity = Eigen::Vector3d(0.1, 0.2, 0.3);
  auto body = world->addRigidBody("body", bodyOptions);
  body.setForce(Eigen::Vector3d::Zero());

  auto robot = world->addMultibody("robot");
  auto base = robot.addLink("base");
  sx::JointSpec sliderSpec;
  sliderSpec.name = "slider";
  sliderSpec.type = sx::JointType::Prismatic;
  sliderSpec.axis = Eigen::Vector3d::UnitX();
  auto link = robot.addLink("link", base, sliderSpec);
  auto slider = link.getParentJoint();
  slider.setPosition(Eigen::VectorXd::Constant(1, 0.4));
  slider.setVelocity(Eigen::VectorXd::Constant(1, -0.2));
  slider.setForce(Eigen::VectorXd::Constant(1, 0.0));

  return world;
}

// Deterministic, modest control sequence (steps x num_efforts).
Eigen::MatrixXd buildControlSequence(std::size_t steps, Eigen::Index efforts)
{
  Eigen::MatrixXd controls(static_cast<Eigen::Index>(steps), efforts);
  for (Eigen::Index t = 0; t < controls.rows(); ++t) {
    for (Eigen::Index j = 0; j < efforts; ++j) {
      controls(t, j) = 0.05 * std::sin(0.7 * (t + 1) + 0.3 * j) + 0.01 * j;
    }
  }
  return controls;
}

// Scalar loss = sum of the final state. Re-runs the WHOLE rollout from a fresh
// world (so every finite-difference probe starts from an identical state) and
// returns the loss.
double rolloutFinalStateSum(
    const Eigen::VectorXd& initialState, const Eigen::MatrixXd& controls)
{
  auto world = buildFreeFallScene();
  const sx::diff::RolloutTrajectory trajectory
      = sx::diff::rollout(*world, initialState, controls, kSteps);
  return trajectory.states.back().sum();
}

} // namespace

//==============================================================================
// The analytic whole-rollout VJP {initialStateGrad, controlGrads} for a
// loss = sum(final state) must match a central finite-difference of the whole
// rollout, over a range of step sizes.
TEST(DiffRollout, WholeRolloutVjpMatchesFiniteDifference)
{
  auto world = buildFreeFallScene();
  const Eigen::VectorXd initialState = world->getStateVector();
  const auto efforts = static_cast<Eigen::Index>(world->getNumEfforts());
  const Eigen::MatrixXd controls = buildControlSequence(kSteps, efforts);

  // No contacts at the start (free fall well above the ground).
  ASSERT_TRUE(world->collide().empty())
      << "expected a contact-free free-fall scene";

  const sx::diff::RolloutTrajectory trajectory
      = sx::diff::rollout(*world, initialState, controls, kSteps);

  ASSERT_EQ(trajectory.states.size(), kSteps + 1);
  ASSERT_EQ(trajectory.stepDerivatives.size(), kSteps);
  ASSERT_EQ(trajectory.states.front().size(), initialState.size());

  // Loss = sum(final state) => dL/dx_final = ones.
  const Eigen::VectorXd finalStateGrad
      = Eigen::VectorXd::Ones(trajectory.states.back().size());
  const sx::diff::RolloutGradient gradient
      = trajectory.rolloutVjp(finalStateGrad);

  ASSERT_EQ(gradient.initialStateGrad.size(), initialState.size());
  ASSERT_EQ(gradient.controlGrads.size(), kSteps);

  const double base = rolloutFinalStateSum(initialState, controls);
  // Sanity: the trajectory's recorded final state matches a fresh re-run.
  ASSERT_NEAR(trajectory.states.back().sum(), base, 1e-12);

  const std::vector<double> stepSizes = {1e-5, 1e-6, 1e-7};

  double worstStateRel = 0.0;
  double worstControlRel = 0.0;

  for (const double h : stepSizes) {
    // ---- Gradient w.r.t. each initial-state component. ----
    for (Eigen::Index i = 0; i < initialState.size(); ++i) {
      Eigen::VectorXd plus = initialState;
      Eigen::VectorXd minus = initialState;
      plus[i] += h;
      minus[i] -= h;
      const double fdPlus = rolloutFinalStateSum(plus, controls);
      const double fdMinus = rolloutFinalStateSum(minus, controls);
      const double fd = (fdPlus - fdMinus) / (2.0 * h);
      const double analytic = gradient.initialStateGrad[i];
      const double denom = std::max(std::abs(analytic), 1e-9);
      const double rel = std::abs(fd - analytic) / denom;
      worstStateRel = std::max(worstStateRel, rel);
      EXPECT_LT(rel, 1e-4) << "initialStateGrad[" << i
                           << "] mismatch at h=" << h << " fd=" << fd
                           << " analytic=" << analytic;
    }

    // ---- Gradient w.r.t. each control[t][j] component. ----
    for (std::size_t t = 0; t < kSteps; ++t) {
      for (Eigen::Index j = 0; j < efforts; ++j) {
        Eigen::MatrixXd plus = controls;
        Eigen::MatrixXd minus = controls;
        plus(static_cast<Eigen::Index>(t), j) += h;
        minus(static_cast<Eigen::Index>(t), j) -= h;
        const double fdPlus = rolloutFinalStateSum(initialState, plus);
        const double fdMinus = rolloutFinalStateSum(initialState, minus);
        const double fd = (fdPlus - fdMinus) / (2.0 * h);
        const double analytic = gradient.controlGrads[t][j];
        const double denom = std::max(std::abs(analytic), 1e-9);
        const double rel = std::abs(fd - analytic) / denom;
        worstControlRel = std::max(worstControlRel, rel);
        EXPECT_LT(rel, 1e-4)
            << "controlGrads[" << t << "][" << j << "] mismatch at h=" << h
            << " fd=" << fd << " analytic=" << analytic;
      }
    }
  }

  // Surface the worst observed relative error for log inspection.
  RecordProperty("worst_state_rel", std::to_string(worstStateRel));
  RecordProperty("worst_control_rel", std::to_string(worstControlRel));
  std::cout << "[rollout-FD] worst initialStateGrad rel = " << worstStateRel
            << ", worst controlGrads rel = " << worstControlRel << std::endl;
}

//==============================================================================
// The per-step-state-gradient overload reduces to the final-state VJP when only
// the final-state gradient is nonzero.
TEST(DiffRollout, PerStepVjpReducesToFinalStateVjp)
{
  auto world = buildFreeFallScene();
  const Eigen::VectorXd initialState = world->getStateVector();
  const auto efforts = static_cast<Eigen::Index>(world->getNumEfforts());
  const Eigen::MatrixXd controls = buildControlSequence(kSteps, efforts);

  const sx::diff::RolloutTrajectory trajectory
      = sx::diff::rollout(*world, initialState, controls, kSteps);

  const Eigen::Index stateSize = trajectory.states.back().size();
  const Eigen::VectorXd finalStateGrad = Eigen::VectorXd::Ones(stateSize);
  const sx::diff::RolloutGradient viaFinal
      = trajectory.rolloutVjp(finalStateGrad);

  std::vector<Eigen::VectorXd> stateGrads(
      kSteps + 1, Eigen::VectorXd::Zero(stateSize));
  stateGrads.back() = finalStateGrad;
  const sx::diff::RolloutGradient viaPerStep
      = trajectory.rolloutVjpPerStep(stateGrads);

  EXPECT_LT(
      (viaFinal.initialStateGrad - viaPerStep.initialStateGrad)
          .cwiseAbs()
          .maxCoeff(),
      1e-12);
  ASSERT_EQ(viaFinal.controlGrads.size(), viaPerStep.controlGrads.size());
  for (std::size_t t = 0; t < viaFinal.controlGrads.size(); ++t) {
    EXPECT_LT(
        (viaFinal.controlGrads[t] - viaPerStep.controlGrads[t])
            .cwiseAbs()
            .maxCoeff(),
        1e-12);
  }
}

//==============================================================================
TEST(DiffRollout, RejectsMixedRigidBodyAndMultibodyWorld)
{
  auto world = buildMixedRigidMultibodyScene();
  const Eigen::VectorXd initialState = world->getStateVector();
  const auto efforts = static_cast<Eigen::Index>(world->getNumEfforts());
  const Eigen::MatrixXd controls = Eigen::MatrixXd::Zero(kSteps, efforts);

  ASSERT_GT(world->getNumRigidBodyDofs(), 0u);
  ASSERT_GT(world->getNumDofs(), world->getNumRigidBodyDofs());

  EXPECT_THROW(
      static_cast<void>(
          sx::diff::rollout(*world, initialState, controls, kSteps)),
      sx::NotImplementedException);
  EXPECT_THROW(world->step(), sx::NotImplementedException);
}

//==============================================================================
// Input-shape guards: wrong-sized control sequence / state, and steps == 0.
TEST(DiffRollout, RejectsBadShapes)
{
  auto world = buildFreeFallScene();
  const Eigen::VectorXd initialState = world->getStateVector();
  const auto efforts = static_cast<Eigen::Index>(world->getNumEfforts());
  const Eigen::MatrixXd controls = buildControlSequence(kSteps, efforts);

  EXPECT_THROW(
      static_cast<void>(sx::diff::rollout(*world, initialState, controls, 0)),
      sx::InvalidArgumentException);

  Eigen::VectorXd wrongState(initialState.size() + 1);
  wrongState.setZero();
  EXPECT_THROW(
      static_cast<void>(
          sx::diff::rollout(*world, wrongState, controls, kSteps)),
      sx::InvalidArgumentException);

  Eigen::MatrixXd wrongControls(static_cast<Eigen::Index>(kSteps), efforts + 1);
  wrongControls.setZero();
  EXPECT_THROW(
      static_cast<void>(
          sx::diff::rollout(*world, initialState, wrongControls, kSteps)),
      sx::InvalidArgumentException);
}
