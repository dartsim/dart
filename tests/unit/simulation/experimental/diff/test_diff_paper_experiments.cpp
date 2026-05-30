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

// Reproductions of the Nimble paper's gradient-based control experiments
// (arXiv:2103.16021, Section VII-B) on an ARTICULATED system. The paper shows
// that analytic Jacobians enable gradient-based trajectory optimization on a
// cartpole; here we optimize a cart-force control sequence through a multibody
// cartpole rollout, with the per-step joint-space Jacobians coming from
// World::getStepDerivatives() — which uses the analytic articulated-body
// dynamics derivatives (revolute/prismatic constant unit-twist joints) rather
// than finite differencing.
//
// The cartpole is contact-free, so this exercises the smooth articulated
// gradient path end to end through an optimizer. The reverse pass is a manual
// adjoint over the chained per-step Jacobians (the World state vector covers
// only rigid bodies, so a multibody rollout is driven through its joints rather
// than diff::rollout, which is state-vector based).
//
// Registered only when DART_BUILD_DIFF is ON.

#include <dart/simulation/experimental/body/collision_shape.hpp>
#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/body/rigid_body_options.hpp>
#include <dart/simulation/experimental/diff/rollout.hpp>
#include <dart/simulation/experimental/diff/step_derivatives.hpp>
#include <dart/simulation/experimental/multibody/multibody.hpp>
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
// A cartpole: a cart that slides along world X (prismatic, actuated) carrying a
// pole that hinges about world Y (revolute, unactuated). The pole link's center
// of mass is offset along +X so gravity exerts a configuration-dependent torque
// on it. DOF order is [cart, pole]; state is [q; q̇] of size 4.
std::unique_ptr<sx::World> buildCartpole()
{
  sx::WorldOptions options;
  options.timeStep = kTimeStep;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.differentiable = true;
  auto world = std::make_unique<sx::World>(options);

  auto cartpole = world->addMultibody("cartpole");
  auto base = cartpole.addLink("base");

  auto cart = cartpole.addLink(
      "cart",
      base,
      sx::JointSpec{
          .name = "slide",
          .type = sx::JointType::Prismatic,
          .axis = Eigen::Vector3d::UnitX()});
  cart.setMass(1.0);
  cart.setInertia(0.01 * Eigen::Matrix3d::Identity());

  Eigen::Isometry3d poleOffset = Eigen::Isometry3d::Identity();
  poleOffset.translation()
      = Eigen::Vector3d(0.5, 0.0, 0.0); // pole length 0.5 m
  auto pole = cartpole.addLink(
      "pole",
      cart,
      sx::JointSpec{
          .name = "hinge",
          .type = sx::JointType::Revolute,
          .axis = Eigen::Vector3d::UnitY(),
          .transformFromParent = poleOffset});
  pole.setMass(0.5);
  pole.setInertia(0.01 * Eigen::Matrix3d::Identity());

  return world;
}

//==============================================================================
// Read a multibody's joint-space state [q; q̇] (joints in construction order).
Eigen::VectorXd readState(const sx::Multibody& mb)
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
  Eigen::VectorXd state(static_cast<Eigen::Index>(q.size() + qdot.size()));
  const auto ndof = static_cast<Eigen::Index>(q.size());
  for (Eigen::Index i = 0; i < ndof; ++i) {
    state[i] = q[static_cast<std::size_t>(i)];
    state[ndof + i] = qdot[static_cast<std::size_t>(i)];
  }
  return state;
}

//==============================================================================
// A rollout of the cartpole driven by a per-step cart force. Records the joint
// state after each step and the per-step Jacobians for the reverse pass.
struct CartpoleRollout
{
  std::vector<Eigen::VectorXd> states;          // states[0..steps], each size 4
  std::vector<sx::StepDerivatives> derivatives; // per step, size steps
};

CartpoleRollout rolloutCartpole(
    sx::World& world, const Eigen::VectorXd& cartForce)
{
  auto cartpole = world.getMultibody("cartpole");
  CartpoleRollout rollout;
  rollout.states.push_back(readState(*cartpole));

  const auto steps = cartForce.size();
  for (Eigen::Index t = 0; t < steps; ++t) {
    // Apply the control to the actuated cart (prismatic) joint; the pole hinge
    // is unactuated (zero force).
    auto joints = cartpole->getJoints();
    joints[0].setForce(Eigen::VectorXd::Constant(1, cartForce[t]));
    joints[1].setForce(Eigen::VectorXd::Zero(1));

    world.step();
    rollout.states.push_back(readState(*cartpole));
    rollout.derivatives.push_back(world.getStepDerivatives());
  }
  return rollout;
}

} // namespace

//==============================================================================
// TRAJECTORY OPTIMIZATION ON AN ARTICULATED SYSTEM (paper Section VII-B):
// optimize a cart-force sequence so the cartpole's cart reaches a target X
// position at the final step, by plain gradient descent on the analytic
// gradient. The loss is 0.5 (cart_x_final - target)²; its gradient with respect
// to the control sequence is the reverse-mode adjoint over the chained per-step
// joint-space Jacobians (state Jacobian ∂x_{t+1}/∂x_t and control Jacobian
// ∂x_{t+1}/∂τ_t from getStepDerivatives, which use the ANALYTIC articulated
// dynamics derivatives for these revolute/prismatic joints).
TEST(DiffPaperExperiments, CartpoleReachesTargetByGradientDescent)
{
  constexpr Eigen::Index kSteps = 50;
  constexpr int kMaxIters = 6000;
  constexpr double kLearningRate = 2.0;
  constexpr double kConvergedLoss = 1e-4;
  constexpr double kTarget = 0.5; // desired final cart X position (metres)
  constexpr Eigen::Index kCartDof = 0;
  constexpr Eigen::Index kStateSize = 4; // [cart_x, pole_th, cart_v, pole_w]

  auto world = buildCartpole();
  {
    auto cartpole = world->getMultibody("cartpole");
    ASSERT_EQ(readState(*cartpole).size(), kStateSize);
  }

  Eigen::VectorXd controls = Eigen::VectorXd::Zero(kSteps);

  double loss = std::numeric_limits<double>::infinity();
  double previousLoss = loss;
  double initialLoss = 0.0;
  bool monotonic = true;
  int iters = 0;

  for (iters = 0; iters < kMaxIters; ++iters) {
    // Fresh world each iteration so the rollout starts from the same state.
    auto rolloutWorld = buildCartpole();
    const CartpoleRollout rollout = rolloutCartpole(*rolloutWorld, controls);

    const double cartFinal = rollout.states.back()[kCartDof];
    const double residual = cartFinal - kTarget;
    loss = 0.5 * residual * residual;
    if (iters == 0) {
      initialLoss = loss;
    }
    if (loss < kConvergedLoss) {
      break;
    }

    // Adjoint reverse pass accumulating dL/dτ_t. Only the final state carries a
    // loss, on the cart position row.
    Eigen::VectorXd adjoint = Eigen::VectorXd::Zero(kStateSize);
    adjoint[kCartDof] = residual; // dL/dx_final
    Eigen::VectorXd controlGrad = Eigen::VectorXd::Zero(kSteps);
    for (Eigen::Index t = kSteps; t-- > 0;) {
      const sx::StepDerivatives& d
          = rollout.derivatives[static_cast<std::size_t>(t)];
      // dL/dτ_t = controlJacobianᵀ adjoint, take the cart column.
      controlGrad[t] = d.controlJacobian.col(kCartDof).dot(adjoint);
      adjoint = d.stateJacobian.transpose() * adjoint;
    }

    controls -= kLearningRate * controlGrad;

    if (iters > 0 && loss > previousLoss + 1e-9) {
      monotonic = false;
    }
    previousLoss = loss;
  }

  // The paper's VII-B claim is that the analytic gradient ENABLES
  // gradient-based trajectory optimization. The optimizer descends
  // monotonically and reduces the loss by a large factor, driving the cart to
  // the target. (Plain fixed-step gradient descent is asymptotically slow near
  // the optimum, so we assert a strong reduction and target closeness rather
  // than an exact-zero loss; a second-order optimizer would tighten this
  // further.)
  EXPECT_TRUE(monotonic) << "loss increased during gradient descent";
  EXPECT_LT(loss, initialLoss / 50.0)
      << "gradient descent did not substantially reduce the loss (initial="
      << initialLoss << ", final=" << loss << ")";

  auto finalWorld = buildCartpole();
  const CartpoleRollout finalRollout = rolloutCartpole(*finalWorld, controls);
  const double cartFinal = finalRollout.states.back()[kCartDof];
  EXPECT_NEAR(cartFinal, kTarget, 5e-2);

  std::cout << "[cartpole-opt] iters=" << iters << " final loss=" << loss
            << " cart_x_final=" << cartFinal << " target=" << kTarget
            << std::endl;
}

namespace {

constexpr double kDroneRadius = 0.2;
constexpr double kDroneMass = 1.0;
constexpr double kDroneTarget = 1.5; // target hover height (metres)

// A single free rigid-body "drone" resting on the ground (an active clamping
// contact). Vertical thrust is applied through the rollout control. The
// contact-gradient mode selects naive (Analytic) vs complementarity-aware
// backward gradients. A free rigid body can leave the ground (no fixed base),
// so the lift-off is genuine.
std::unique_ptr<sx::World> buildDroneWorld(sx::ContactGradientMode mode)
{
  sx::WorldOptions options;
  options.timeStep = kTimeStep;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.differentiable = true;
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  options.contactGradientMode = mode;
  auto world = std::make_unique<sx::World>(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world->addRigidBody("ground", groundOptions);
  // makeBox takes half-extents, so a 0.5 half-height box at z=-0.5 has its top
  // face at z=0; the drone (radius 0.2) then rests with its centre at z=0.2.
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(10.0, 10.0, 0.5)));
  ground.setFriction(0.0);

  sx::RigidBodyOptions droneOptions;
  droneOptions.mass = kDroneMass;
  // Rest on the ground top (z = 0): centre at the radius with a sub-allowance
  // penetration so the contact is active and clamping at the start.
  droneOptions.position = Eigen::Vector3d(0.0, 0.0, kDroneRadius - 5e-5);
  auto drone = world->addRigidBody("drone", droneOptions);
  drone.setCollisionShape(sx::CollisionShape::makeSphere(kDroneRadius));
  drone.setFriction(0.0);

  return world;
}

struct DroneResult
{
  double loss = 0.0;
  double thrust = 0.0;
  double finalHeight = 0.0;
};

// Gradient-descend a constant vertical thrust so the drone reaches the target
// height by the final step, with the given contact-gradient mode. The loss is
// 0.5 (target - final_z)²; the thrust gradient is the whole-rollout VJP summed
// over the (shared) per-step z-force component.
DroneResult optimizeDrone(sx::ContactGradientMode mode, int iters, double lr)
{
  constexpr std::size_t kHorizon = 150;
  double thrust = 0.0; // start at rest on the ground (the clamping saddle)
  DroneResult result;

  for (int it = 0; it < iters; ++it) {
    auto world = buildDroneWorld(mode);
    const Eigen::VectorXd x0 = world->getStateVector();
    const Eigen::Index stateSize = x0.size();
    const auto efforts = static_cast<Eigen::Index>(world->getNumEfforts());
    Eigen::MatrixXd controls
        = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(kHorizon), efforts);
    controls.col(2).setConstant(thrust); // vertical (z) force each step

    const sx::diff::RolloutTrajectory trajectory
        = sx::diff::rollout(*world, x0, controls, kHorizon);
    const double finalZ = trajectory.states.back()[2];
    const double residual = finalZ - kDroneTarget;
    result.loss = 0.5 * residual * residual;
    result.thrust = thrust;
    result.finalHeight = finalZ;

    Eigen::VectorXd finalGrad = Eigen::VectorXd::Zero(stateSize);
    finalGrad[2] = residual; // dL/d(final z)
    const sx::diff::RolloutGradient gradient = trajectory.rolloutVjp(finalGrad);
    double thrustGrad = 0.0;
    for (const auto& stepGrad : gradient.controlGrads) {
      thrustGrad += stepGrad[2];
    }
    thrust -= lr * thrustGrad;
    if (thrust < 0.0) {
      thrust = 0.0; // physical thrust is non-negative
    }
  }
  return result;
}

} // namespace

//==============================================================================
// COMPLEMENTARITY-AWARE SADDLE ESCAPE (paper Section VII-C, Fig 8): a drone
// rests on the ground and must lift off to a target height. While resting, the
// ground contact is "clamping", so the true (Analytic) gradient ∂q̇'/∂τ = 0 —
// naive gradient descent stalls in this saddle and the drone never leaves the
// ground. The complementarity-aware gradient is non-zero there, so SGD raises
// the thrust, the drone lifts off and climbs toward the target. This asserts
// the qualitative escape: naive stays grounded, complementarity-aware lifts off
// and reaches a lower loss.
TEST(
    DiffPaperExperiments, DroneLiftOffComplementarityAwareEscapesClampingSaddle)
{
  const DroneResult naive
      = optimizeDrone(sx::ContactGradientMode::Analytic, 400, 4.0);
  const DroneResult aware
      = optimizeDrone(sx::ContactGradientMode::ComplementarityAware, 400, 4.0);

  // Naive gradients stall in the clamping saddle: the thrust never grows and
  // the drone stays grounded near its resting height (~ the drone radius).
  EXPECT_LT(naive.thrust, 1.0)
      << "naive thrust unexpectedly grew (thrust=" << naive.thrust << ")";
  EXPECT_LT(naive.finalHeight, kDroneRadius + 0.2)
      << "naive gradients unexpectedly lifted off (height=" << naive.finalHeight
      << ")";
  // Complementarity-aware gradients escape the saddle: the thrust grows past
  // gravity, the drone lifts off and climbs toward the target height.
  EXPECT_GT(aware.thrust, kDroneMass * 9.81)
      << "complementarity-aware thrust did not exceed gravity (thrust="
      << aware.thrust << ")";
  EXPECT_GT(aware.finalHeight, 1.0)
      << "complementarity-aware did not lift off toward the target (height="
      << aware.finalHeight << ")";
  EXPECT_LT(aware.loss, 0.3 * naive.loss)
      << "complementarity-aware did not reduce the loss well below naive";

  std::cout << "[drone] naive: thrust=" << naive.thrust
            << " height=" << naive.finalHeight << " loss=" << naive.loss
            << " | aware: thrust=" << aware.thrust
            << " height=" << aware.finalHeight << " loss=" << aware.loss
            << std::endl;
}
