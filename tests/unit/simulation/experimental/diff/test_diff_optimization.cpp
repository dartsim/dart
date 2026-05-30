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

// The Nimble paper's ACTUAL contribution: the analytic gradients ENABLE
// gradient-based optimization through the simulator, not merely matching finite
// differences. These two correctness/convergence tests stand in for the paper's
// headline experiments:
//
//   1. TRAJECTORY OPTIMIZATION (throw / catapult): a translational body is
//      thrown under gravity; we optimize its INITIAL VELOCITY by plain gradient
//      descent so its final position hits a target. The gradient comes from
//      diff::rollout + RolloutTrajectory::rolloutVjp (the whole-rollout
//      reverse-mode VJP). The scene is a contact-free ballistic arc (a single
//      smooth regime), so the rollout is well conditioned and gradient descent
//      converges cleanly. We assert the loss decreases (near-)monotonically and
//      drops below a small threshold within a bounded iteration count.
//
//   2. SYSTEM IDENTIFICATION (mass recovery): we set a TRUE mass, generate a
//      reference trajectory, then from a WRONG initial mass guess recover the
//      true value by gradient descent on a trajectory-matching loss. The
//      gradient with respect to mass comes from
//      StepDerivatives::parameterJacobian (MASS registered via
//      addDifferentiableParameter), backpropagated through the rollout. A
//      constant applied force makes the trajectory mass-dependent even in free
//      fall (a = F/m + g), so the loss is a sharp, well-conditioned function of
//      the mass. We assert the recovered mass converges to the true mass within
//      tolerance.
//
// Both optimizers are plain gradient descent, self-contained here. Registered
// only when DART_BUILD_DIFF is ON.

#include <dart/simulation/experimental/body/collision_shape.hpp>
#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/body/rigid_body_options.hpp>
#include <dart/simulation/experimental/diff/physical_parameter.hpp>
#include <dart/simulation/experimental/diff/rollout.hpp>
#include <dart/simulation/experimental/diff/step_derivatives.hpp>
#include <dart/simulation/experimental/world.hpp>
#include <dart/simulation/experimental/world_options.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include <cmath>

namespace sx = dart::simulation::experimental;

namespace {

constexpr double kTimeStep = 1e-2;

//==============================================================================
// TRAJECTORY-OPTIMIZATION scene: a single translational body thrown under
// gravity, well above any ground so the rollout stays contact-free (one smooth
// regime). State layout for one dynamic body is [px, py, pz, vx, vy, vz]
// (size 6); control is the applied force (size 3).
std::unique_ptr<sx::World> buildThrowScene()
{
  sx::WorldOptions options;
  options.timeStep = kTimeStep;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.differentiable = true;
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  auto world = std::make_unique<sx::World>(options);

  // Static ground far below: never contacted over the horizon, so the rollout
  // is a smooth ballistic arc.
  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -100.0);
  auto ground = world->addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(50.0, 50.0, 0.5)));
  ground.setFriction(0.0);

  sx::RigidBodyOptions projectile;
  projectile.mass = 1.0;
  projectile.position = Eigen::Vector3d(0.0, 0.0, 5.0);
  auto body = world->addRigidBody("projectile", projectile);
  body.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
  body.setFriction(0.0);

  return world;
}

//==============================================================================
// SYSTEM-IDENTIFICATION scene: a single translational body with the given mass,
// free-falling (no contact). A constant applied force is threaded through the
// rollout CONTROL sequence (u = τ is the applied force), so the post-step
// acceleration is F/m + g and the trajectory depends on the mass; recovering m
// from the trajectory is the system-ID task. (The force is supplied as the
// control matrix rather than via setForce, because diff::rollout overwrites the
// control vector each step.)
std::unique_ptr<sx::World> buildMassScene(double mass)
{
  sx::WorldOptions options;
  options.timeStep = kTimeStep;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.differentiable = true;
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  auto world = std::make_unique<sx::World>(options);

  sx::RigidBodyOptions bodyOptions;
  bodyOptions.mass = mass;
  bodyOptions.position = Eigen::Vector3d(0.0, 0.0, 10.0);
  auto body = world->addRigidBody("body", bodyOptions);
  body.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
  body.setFriction(0.0);

  return world;
}

} // namespace

//==============================================================================
// TRAJECTORY OPTIMIZATION: optimize the initial velocity so the final position
// reaches a target. Loss = 0.5 * ||p_final - p_target||². The gradient with
// respect to the initial velocity is read from the whole-rollout VJP
// (rolloutVjp restricted to the position rows of the final state, then the
// velocity rows of initialStateGrad).
TEST(DiffOptimization, ThrowReachesTargetByGradientDescent)
{
  constexpr std::size_t kSteps = 60;
  constexpr int kMaxIters = 400;
  constexpr double kLearningRate = 0.5;
  constexpr double kConvergedLoss = 1e-4;

  auto world = buildThrowScene();
  const Eigen::VectorXd initialState = world->getStateVector();
  const Eigen::Index stateSize = initialState.size();
  ASSERT_EQ(stateSize, 6) << "expected one dynamic body (state = [p; v])";
  const auto efforts = static_cast<Eigen::Index>(world->getNumEfforts());

  // No control input: this is a pure ballistic throw driven by the initial
  // velocity alone.
  const Eigen::MatrixXd controls
      = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(kSteps), efforts);

  // Target landing position: a fixed point the projectile must reach after
  // kSteps. Chosen to be reachable by some initial velocity under gravity.
  const Eigen::Vector3d target(3.0, -2.0, 4.0);

  // Decision variable: the initial velocity (state rows 3..5). Start from rest.
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();

  const auto runRollout = [&](const Eigen::Vector3d& v) {
    Eigen::VectorXd x0 = initialState;
    x0.tail<3>() = v;
    return sx::diff::rollout(*world, x0, controls, kSteps);
  };

  double previousLoss = std::numeric_limits<double>::infinity();
  double loss = previousLoss;
  bool monotonic = true;
  int iters = 0;

  for (iters = 0; iters < kMaxIters; ++iters) {
    const sx::diff::RolloutTrajectory trajectory = runRollout(velocity);
    const Eigen::Vector3d finalPos = trajectory.states.back().head<3>();
    const Eigen::Vector3d residual = finalPos - target;
    loss = 0.5 * residual.squaredNorm();

    if (loss < kConvergedLoss) {
      break;
    }

    // dL/dx_final: only the position rows carry the residual.
    Eigen::VectorXd finalStateGrad = Eigen::VectorXd::Zero(stateSize);
    finalStateGrad.head<3>() = residual;
    const sx::diff::RolloutGradient gradient
        = trajectory.rolloutVjp(finalStateGrad);

    // The decision variable is the initial velocity = initialStateGrad
    // rows 3..5.
    const Eigen::Vector3d velocityGrad = gradient.initialStateGrad.tail<3>();
    velocity -= kLearningRate * velocityGrad;

    // Loss must not increase from one iteration to the next (allow a tiny
    // numerical slack). A clean ballistic regime descends monotonically.
    if (iters > 0 && loss > previousLoss + 1e-9) {
      monotonic = false;
    }
    previousLoss = loss;
  }

  EXPECT_TRUE(monotonic) << "loss increased during gradient descent";
  EXPECT_LT(loss, kConvergedLoss) << "did not converge within " << kMaxIters
                                  << " iterations (loss=" << loss << ")";

  // Confirm the final position is actually at the target.
  const sx::diff::RolloutTrajectory finalTrajectory = runRollout(velocity);
  const Eigen::Vector3d finalPos = finalTrajectory.states.back().head<3>();
  EXPECT_LT((finalPos - target).norm(), 2e-2)
      << "final position " << finalPos.transpose() << " vs target "
      << target.transpose();

  GTEST_LOG_(INFO) << "throw-opt converged in " << iters
                   << " iters, loss=" << loss
                   << ", recovered v0=" << velocity.transpose();
  std::cout << "[throw-opt] iters=" << iters << " final loss=" << loss
            << " v0=" << velocity.transpose()
            << " final pos=" << finalPos.transpose()
            << " target=" << target.transpose() << std::endl;
}

//==============================================================================
// SYSTEM IDENTIFICATION: recover a body's true mass from a reference trajectory
// by gradient descent on a trajectory-matching loss. The gradient with respect
// to mass uses StepDerivatives::parameterJacobian (MASS registered), composed
// across the rollout: dL/dm = Σ_t (dL/dx_{t+1})ᵀ · (∂x_{t+1}/∂m), where the
// per-step parameter Jacobian is taken at the pre-step configuration the
// reference trajectory passes through.
TEST(DiffOptimization, MassRecoveredByGradientDescent)
{
  constexpr std::size_t kSteps = 40;
  constexpr int kMaxIters = 1200;
  constexpr double kLearningRate = 0.2;
  constexpr double kTrueMass = 2.5;
  constexpr double kInitialGuess = 1.0;
  constexpr double kMassTolerance = 1e-2;
  const Eigen::Vector3d kForce(1.5, -0.8, 0.6);

  // ---- Reference trajectory from the TRUE mass. ----
  auto referenceWorld = buildMassScene(kTrueMass);
  const Eigen::VectorXd initialState = referenceWorld->getStateVector();
  const Eigen::Index stateSize = initialState.size();
  ASSERT_EQ(stateSize, 6);
  const auto efforts
      = static_cast<Eigen::Index>(referenceWorld->getNumEfforts());
  ASSERT_EQ(efforts, 3) << "expected one dynamic body (u = applied force)";
  // Constant applied force, threaded through the control sequence so the
  // acceleration F/m + g is mass-dependent and the trajectory identifies m.
  Eigen::MatrixXd controls(static_cast<Eigen::Index>(kSteps), efforts);
  for (Eigen::Index t = 0; t < controls.rows(); ++t) {
    controls.row(t) = kForce.transpose();
  }

  const auto referenceTrajectory
      = sx::diff::rollout(*referenceWorld, initialState, controls, kSteps);
  std::vector<Eigen::VectorXd> referenceStates = referenceTrajectory.states;

  // ---- Gradient descent on the mass. ----
  // Loss = 0.5 * Σ_t ||x_t(guess) - x_t(true)||² over the recorded states.
  double mass = kInitialGuess;
  double loss = std::numeric_limits<double>::infinity();
  double previousLoss = loss;
  bool monotonic = true;
  int iters = 0;

  for (iters = 0; iters < kMaxIters; ++iters) {
    // Build a fresh world at the current mass guess and register MASS for
    // differentiation, so each step caches ∂x'/∂m alongside the state Jacobian.
    auto world = buildMassScene(mass);
    auto body = world->getRigidBody("body");
    world->addDifferentiableParameter(*body, sx::PhysicalParameter::MASS);
    ASSERT_EQ(world->getNumDifferentiableParameters(), 1u);

    const auto trajectory
        = sx::diff::rollout(*world, initialState, controls, kSteps);
    ASSERT_EQ(trajectory.states.size(), kSteps + 1);
    ASSERT_EQ(trajectory.stepDerivatives.size(), kSteps);

    // Per-state residuals (skip x_0, which is identical for any mass).
    loss = 0.0;
    std::vector<Eigen::VectorXd> stateResiduals(
        kSteps + 1, Eigen::VectorXd::Zero(stateSize));
    for (std::size_t t = 1; t <= kSteps; ++t) {
      const Eigen::VectorXd residual
          = trajectory.states[t] - referenceStates[t];
      stateResiduals[t] = residual; // dL/dx_t
      loss += 0.5 * residual.squaredNorm();
    }

    if (loss < 1e-12) {
      break;
    }

    // Backpropagate the per-state residuals through the chained step Jacobians,
    // accumulating the mass gradient from each step's parameterJacobian.
    // Adjoint g carries dL/dx_t backwards; for t = steps-1 ... 0:
    //   dL/dm += parameterJacobianᵀ_t · g_{t+1}
    //   g_t = stateResiduals[t] + stateJacobianᵀ_t · g_{t+1}
    double massGrad = 0.0;
    Eigen::VectorXd g = stateResiduals[kSteps];
    for (std::size_t t = kSteps; t-- > 0;) {
      const sx::StepDerivatives& d = trajectory.stepDerivatives[t];
      ASSERT_EQ(d.parameterJacobian.rows(), stateSize);
      ASSERT_EQ(d.parameterJacobian.cols(), 1);
      massGrad += d.parameterJacobian.col(0).dot(g);
      g = d.stateJacobian.transpose() * g;
      if (t > 0) {
        g += stateResiduals[t];
      }
    }

    mass -= kLearningRate * massGrad;

    if (iters > 0 && loss > previousLoss + 1e-9) {
      monotonic = false;
    }
    previousLoss = loss;

    if (std::abs(mass - kTrueMass) < 0.5 * kMassTolerance) {
      break;
    }
  }

  EXPECT_TRUE(monotonic) << "loss increased during gradient descent";
  EXPECT_NEAR(mass, kTrueMass, kMassTolerance)
      << "did not recover the true mass within " << kMaxIters
      << " iterations (mass=" << mass << ", true=" << kTrueMass << ")";

  GTEST_LOG_(INFO) << "mass-id converged in " << iters
                   << " iters, recovered mass=" << mass
                   << " (true=" << kTrueMass << "), final loss=" << loss;
  std::cout << "[mass-id] iters=" << iters << " recovered mass=" << mass
            << " true mass=" << kTrueMass << " final loss=" << loss
            << std::endl;
}
