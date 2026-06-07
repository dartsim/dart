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

// Differentiable-simulation GUI demo (PLAN-110). Before any frame is drawn the
// example runs a gradient-based TRAJECTORY OPTIMIZATION: a ballistic body is
// "thrown" under gravity and we optimize its INITIAL VELOCITY by plain gradient
// descent so the converged rollout lands on a fixed target marker. The gradient
// comes from `diff::rollout` + `RolloutTrajectory::rolloutVjp` (the
// whole-rollout reverse-mode VJP), exactly as in the throw-to-target test in
// `tests/unit/simulation/diff/test_diff_optimization.cpp`.
//
// The Filament viewer then visualizes the result: a ground plane, the target
// marker, the faint PRE-optimization trajectory (rest throw, which misses), and
// the animated POST-optimization rollout (the projectile sphere replaying the
// converged states frame by frame).
//
// Headless capture works like the other experimental GUI examples: the
// `--headless`, `--frames`, `--width`, `--height`, `--screenshot`, and `--out`
// flags are parsed by `gui::runApplication`, so the CTest headless smoke can
// render a non-blank frame without a display.

#include <dart/gui/application.hpp>
#include <dart/gui/renderable.hpp>

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/diff/rollout.hpp>
#include <dart/simulation/world.hpp>
#include <dart/simulation/world_options.hpp>

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/sphere_shape.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <cstddef>

namespace dynamics = dart::dynamics;
namespace gui = dart::gui;
namespace sx = dart::simulation;

namespace {

//==============================================================================
constexpr double kTimeStep = 1e-2;
constexpr std::size_t kSteps = 60;
constexpr int kMaxIters = 400;
constexpr double kLearningRate = 0.5;
constexpr double kConvergedLoss = 1e-4;
const Eigen::Vector3d kTarget(3.0, -2.0, 4.0);
constexpr double kProjectileRadius = 0.2;

//==============================================================================
Eigen::Vector4d rgba(double r, double g, double b, double a = 1.0)
{
  return {r, g, b, a};
}

//==============================================================================
Eigen::Isometry3d makeTransform(const Eigen::Vector3d& translation)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = translation;
  return transform;
}

//==============================================================================
// Throw-to-target scene: a single translational projectile thrown under gravity
// well above the ground, so the rollout stays a smooth contact-free ballistic
// arc (one differentiable regime). State layout is [px, py, pz, vx, vy, vz].
// Mirrors `buildThrowScene` in test_diff_optimization.cpp.
std::unique_ptr<sx::World> buildThrowScene()
{
  sx::WorldOptions options;
  options.timeStep = kTimeStep;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.differentiable = true;
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  auto world = std::make_unique<sx::World>(options);

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
  body.setCollisionShape(sx::CollisionShape::makeSphere(kProjectileRadius));
  body.setFriction(0.0);

  return world;
}

//==============================================================================
struct OptimizationResult
{
  std::vector<Eigen::Vector3d> initialTrajectory;   ///< rest throw (pre-opt)
  std::vector<Eigen::Vector3d> optimizedTrajectory; ///< converged (post-opt)
  Eigen::Vector3d recoveredVelocity = Eigen::Vector3d::Zero();
  double finalLoss = std::numeric_limits<double>::infinity();
  double finalDistance = std::numeric_limits<double>::infinity();
  int iterations = 0;
};

//==============================================================================
std::vector<Eigen::Vector3d> positionsOf(
    const sx::diff::RolloutTrajectory& trajectory)
{
  std::vector<Eigen::Vector3d> positions;
  positions.reserve(trajectory.states.size());
  for (const auto& state : trajectory.states) {
    positions.push_back(state.head<3>());
  }
  return positions;
}

//==============================================================================
// Gradient-based trajectory optimization: optimize the initial velocity so the
// final position reaches kTarget. Loss = 0.5 * ||p_final - p_target||²; the
// gradient w.r.t. the initial velocity is read from the whole-rollout VJP
// (rolloutVjp restricted to the position rows of the final state, then the
// velocity rows of initialStateGrad). Reuses the throw-to-target loop from
// test_diff_optimization.cpp.
OptimizationResult optimizeThrow()
{
  OptimizationResult result;

  auto world = buildThrowScene();
  const Eigen::VectorXd initialState = world->getStateVector();
  const Eigen::Index stateSize = initialState.size();
  const auto efforts = static_cast<Eigen::Index>(world->getNumEfforts());

  // No control input: a pure ballistic throw driven by the initial velocity.
  const Eigen::MatrixXd controls
      = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(kSteps), efforts);

  const auto runRollout = [&](const Eigen::Vector3d& v) {
    Eigen::VectorXd x0 = initialState;
    x0.tail<3>() = v;
    return sx::diff::rollout(*world, x0, controls, kSteps);
  };

  // Pre-optimization reference: throw from rest, which simply falls short.
  result.initialTrajectory = positionsOf(runRollout(Eigen::Vector3d::Zero()));

  // Decision variable: the initial velocity (state rows 3..5). Start from rest.
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  double loss = std::numeric_limits<double>::infinity();
  int iters = 0;

  for (iters = 0; iters < kMaxIters; ++iters) {
    const sx::diff::RolloutTrajectory trajectory = runRollout(velocity);
    const Eigen::Vector3d finalPos = trajectory.states.back().head<3>();
    const Eigen::Vector3d residual = finalPos - kTarget;
    loss = 0.5 * residual.squaredNorm();

    if (loss < kConvergedLoss) {
      break;
    }

    // dL/dx_final: only the position rows carry the residual.
    Eigen::VectorXd finalStateGrad = Eigen::VectorXd::Zero(stateSize);
    finalStateGrad.head<3>() = residual;
    const sx::diff::RolloutGradient gradient
        = trajectory.rolloutVjp(finalStateGrad);

    // The decision variable is the initial velocity = initialStateGrad rows
    // 3..5.
    const Eigen::Vector3d velocityGrad = gradient.initialStateGrad.tail<3>();
    velocity -= kLearningRate * velocityGrad;
  }

  const sx::diff::RolloutTrajectory finalTrajectory = runRollout(velocity);
  const Eigen::Vector3d finalPos = finalTrajectory.states.back().head<3>();

  result.optimizedTrajectory = positionsOf(finalTrajectory);
  result.recoveredVelocity = velocity;
  result.finalLoss = loss;
  result.finalDistance = (finalPos - kTarget).norm();
  result.iterations = iters;

  std::cout << "[throw-opt] iters=" << result.iterations
            << " final loss=" << result.finalLoss
            << " v0=" << result.recoveredVelocity.transpose()
            << " final pos=" << finalPos.transpose()
            << " target=" << kTarget.transpose()
            << " distance=" << result.finalDistance << std::endl;

  return result;
}

//==============================================================================
struct ExampleState
{
  OptimizationResult optimization;
  std::vector<dynamics::SimpleFramePtr> renderFrames;
  dynamics::SimpleFramePtr projectileFrame;
  std::size_t playhead = 0;
  double time = 0.0;

  void addFrame(dynamics::SimpleFramePtr frame)
  {
    renderFrames.push_back(std::move(frame));
  }

  std::vector<gui::RenderableDescriptor> renderables() const
  {
    std::vector<gui::RenderableDescriptor> descriptors;
    descriptors.reserve(renderFrames.size());
    for (const auto& frame : renderFrames) {
      if (auto descriptor = gui::describeShapeFrame(*frame)) {
        descriptors.push_back(*descriptor);
      }
    }
    return descriptors;
  }

  void advance()
  {
    if (optimization.optimizedTrajectory.empty()) {
      return;
    }
    const auto& trajectory = optimization.optimizedTrajectory;
    projectileFrame->setTransform(makeTransform(trajectory[playhead]));
    time = static_cast<double>(playhead) * kTimeStep;
    playhead = (playhead + 1) % trajectory.size();
  }

  void reset()
  {
    playhead = 0;
    time = 0.0;
    if (!optimization.optimizedTrajectory.empty()) {
      projectileFrame->setTransform(
          makeTransform(optimization.optimizedTrajectory.front()));
    }
  }
};

//==============================================================================
void addGround(ExampleState& state)
{
  // A visual ground plane near the bottom of the throw arc (the physics ground
  // sits far below to keep the rollout contact-free; this is purely visual).
  auto frame = dynamics::SimpleFrame::createShared(
      dynamics::Frame::World(),
      "ground_visual",
      makeTransform({1.5, -1.0, 0.0}));
  frame->setShape(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(8.0, 8.0, 0.1)));
  frame->getVisualAspect(true)->setRGBA(rgba(0.40, 0.43, 0.46));
  state.addFrame(frame);
}

//==============================================================================
void addTargetMarker(ExampleState& state)
{
  auto frame = dynamics::SimpleFrame::createShared(
      dynamics::Frame::World(), "target_marker", makeTransform(kTarget));
  frame->setShape(std::make_shared<dynamics::SphereShape>(0.18));
  frame->getVisualAspect(true)->setRGBA(rgba(0.95, 0.78, 0.16, 0.9));
  state.addFrame(frame);
}

//==============================================================================
// Faint marker spheres tracing the pre-optimization (rest throw) trajectory, so
// the converged result can be compared against the un-optimized one.
void addPreOptimizationTrail(ExampleState& state)
{
  const auto& trajectory = state.optimization.initialTrajectory;
  for (std::size_t i = 0; i < trajectory.size(); i += 5) {
    auto frame = dynamics::SimpleFrame::createShared(
        dynamics::Frame::World(),
        "pre_opt_" + std::to_string(i),
        makeTransform(trajectory[i]));
    frame->setShape(std::make_shared<dynamics::SphereShape>(0.05));
    frame->getVisualAspect(true)->setRGBA(rgba(0.55, 0.58, 0.62, 0.45));
    state.addFrame(frame);
  }
}

//==============================================================================
// Marker spheres tracing the optimized trajectory so the converged arc is
// visible even in a single screenshot, alongside the animated projectile.
void addOptimizedTrail(ExampleState& state)
{
  const auto& trajectory = state.optimization.optimizedTrajectory;
  for (std::size_t i = 0; i < trajectory.size(); i += 4) {
    auto frame = dynamics::SimpleFrame::createShared(
        dynamics::Frame::World(),
        "opt_trail_" + std::to_string(i),
        makeTransform(trajectory[i]));
    frame->setShape(std::make_shared<dynamics::SphereShape>(0.07));
    frame->getVisualAspect(true)->setRGBA(rgba(0.18, 0.55, 0.95, 0.8));
    state.addFrame(frame);
  }
}

//==============================================================================
void addProjectile(ExampleState& state)
{
  Eigen::Vector3d start = Eigen::Vector3d(0.0, 0.0, 5.0);
  if (!state.optimization.optimizedTrajectory.empty()) {
    start = state.optimization.optimizedTrajectory.front();
  }
  state.projectileFrame = dynamics::SimpleFrame::createShared(
      dynamics::Frame::World(), "projectile_visual", makeTransform(start));
  state.projectileFrame->setShape(
      std::make_shared<dynamics::SphereShape>(kProjectileRadius));
  state.projectileFrame->getVisualAspect(true)->setRGBA(rgba(0.93, 0.30, 0.22));
  state.addFrame(state.projectileFrame);
}

//==============================================================================
std::shared_ptr<ExampleState> makeExampleState()
{
  auto state = std::make_shared<ExampleState>();
  state->optimization = optimizeThrow();

  addGround(*state);
  addTargetMarker(*state);
  addPreOptimizationTrail(*state);
  addOptimizedTrail(*state);
  addProjectile(*state);
  state->reset();
  return state;
}

//==============================================================================
gui::RunOptions makeRunDefaults()
{
  gui::RunOptions options;
  options.windowTitle = "DART Differentiable Simulation";
  options.width = 1280;
  options.height = 720;
  return options;
}

//==============================================================================
gui::OrbitCamera makeCamera()
{
  gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(1.5, -1.0, 2.5);
  camera.yaw = -0.72;
  camera.pitch = 0.30;
  camera.distance = 9.0;
  return camera;
}

//==============================================================================
std::vector<gui::KeyboardAction> createKeyboardActions(
    const std::shared_ptr<ExampleState>& state)
{
  gui::KeyboardAction reset;
  reset.label = "reset playback";
  reset.shortcut = gui::KeyboardShortcut::characterKey('r');
  reset.callback = [state](gui::KeyboardActionContext&) {
    state->reset();
  };
  return {std::move(reset)};
}

//==============================================================================
gui::Panel createControlsPanel(const std::shared_ptr<ExampleState>& state)
{
  gui::Panel panel;
  panel.title = "Differentiable Throw";
  panel.build = [state](gui::PanelBuilder& builder) {
    const OptimizationResult& opt = state->optimization;
    builder.text("Gradient descent through diff::rollout + rolloutVjp");
    builder.text("Iterations: " + std::to_string(opt.iterations));
    builder.text("Final loss: " + std::to_string(opt.finalLoss));
    builder.text("Distance to target: " + std::to_string(opt.finalDistance));
    builder.separator();
    if (builder.button("Reset Playback")) {
      state->reset();
    }
  };
  return panel;
}

} // namespace

int main(int argc, char* argv[])
{
  const auto state = makeExampleState();

  gui::ApplicationOptions options;
  options.runDefaults = makeRunDefaults();
  options.camera = makeCamera();
  options.timeStep = kTimeStep;
  options.advanceSimulation = false;
  options.preStep = [state]() {
    state->advance();
  };
  options.renderableProvider = [state]() {
    return state->renderables();
  };
  options.panels.push_back(createControlsPanel(state));
  options.keyboardActions = createKeyboardActions(state);

  return gui::runApplication(argc, argv, options);
}
