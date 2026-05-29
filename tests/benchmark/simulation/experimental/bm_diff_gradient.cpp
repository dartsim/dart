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

// The Nimble paper's HEADLINE benchmark: an ANALYTIC full step Jacobian
// (World::getStepDerivatives()) versus the SAME state Jacobian recovered by
// FINITE DIFFERENCING the forward step. On a representative contact scene
// (a sphere resting/sliding on a static box ground, BoxedLcp + differentiable),
// the analytic path returns ∂x'/∂x in a single backward assembly, whereas the
// finite-difference path must run 2 * (2*ndof) forward steps (a central
// difference of step() for each of the 2*ndof state components). Reporting both
// times side by side makes the speedup ratio (the paper claims ~10-100x)
// visible in the benchmark output — this file MEASURES and reports it honestly;
// it does not assert a specific ratio.
//
// A secondary pair benchmarks diff-ON vs diff-OFF step() on the same scene to
// show the opt-in differentiable overhead of a single forward step.
//
// Gated behind DART_BUILD_DIFF; registered in this directory's CMakeLists only
// when DART_BUILD_DIFF is ON.

#include <dart/simulation/experimental/body/collision_shape.hpp>
#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/body/rigid_body_options.hpp>
#include <dart/simulation/experimental/diff/step_derivatives.hpp>
#include <dart/simulation/experimental/world.hpp>
#include <dart/simulation/experimental/world_options.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <benchmark/benchmark.h>

#include <memory>

namespace sx = dart::simulation::experimental;

namespace {

constexpr double kTimeStep = 1e-3;
constexpr double kSphereRadius = 0.5;
constexpr double kSphereMass = 2.0;

//==============================================================================
// Representative contact scene: a sphere resting on (and sliding along) a
// static box ground, BoxedLcp, with the differentiable opt-in controllable so
// the same scene serves the diff-ON / diff-OFF comparison. The sphere is placed
// with a sub-allowance penetration and a horizontal velocity so the contact is
// active and firmly clamping during the step (the regime the analytic
// contact-aware Jacobian targets).
std::unique_ptr<sx::World> buildContactScene(bool differentiable)
{
  sx::WorldOptions options;
  options.timeStep = kTimeStep;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.differentiable = differentiable;
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  auto world = std::make_unique<sx::World>(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world->addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));
  ground.setFriction(0.0);

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.mass = kSphereMass;
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, kSphereRadius - 5e-5);
  sphereOptions.linearVelocity = Eigen::Vector3d(0.3, 0.0, -0.2);
  auto sphere = world->addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(kSphereRadius));
  sphere.setFriction(0.0);

  return world;
}

} // namespace

//==============================================================================
// ANALYTIC: one getStepDerivatives() returns the full ∂x'/∂x (and the control /
// parameter blocks) for the most recent step. We step once per iteration to
// stay at the same configuration the FD path differentiates.
static void BM_StateJacobian_Analytic(benchmark::State& state)
{
  auto world = buildContactScene(/*differentiable=*/true);
  const Eigen::VectorXd x0 = world->getStateVector();
  const Eigen::VectorXd u0 = world->getControlVector();

  for (auto _ : state) {
    world->setStateVector(x0);
    world->setControlVector(u0);
    world->step();
    const sx::StepDerivatives derivatives = world->getStepDerivatives();
    benchmark::DoNotOptimize(derivatives.stateJacobian.data());
  }
}
BENCHMARK(BM_StateJacobian_Analytic);

//==============================================================================
// FINITE DIFFERENCES: recover the SAME ∂x'/∂x by central-differencing step()
// over each of the 2*ndof state components — 2 * (2*ndof) forward steps per
// Jacobian. This is the baseline the analytic path is meant to replace; the
// ratio of this time to BM_StateJacobian_Analytic is the headline speedup.
static void BM_StateJacobian_FiniteDifference(benchmark::State& state)
{
  auto world = buildContactScene(/*differentiable=*/false);
  const Eigen::VectorXd x0 = world->getStateVector();
  const Eigen::VectorXd u0 = world->getControlVector();
  const Eigen::Index stateSize = x0.size();
  const double h = 1e-6;

  Eigen::MatrixXd jacobian(stateSize, stateSize);

  for (auto _ : state) {
    for (Eigen::Index i = 0; i < stateSize; ++i) {
      Eigen::VectorXd plus = x0;
      Eigen::VectorXd minus = x0;
      plus[i] += h;
      minus[i] -= h;

      world->setStateVector(plus);
      world->setControlVector(u0);
      world->step();
      const Eigen::VectorXd nextPlus = world->getStateVector();

      world->setStateVector(minus);
      world->setControlVector(u0);
      world->step();
      const Eigen::VectorXd nextMinus = world->getStateVector();

      jacobian.col(i) = (nextPlus - nextMinus) / (2.0 * h);
    }
    benchmark::DoNotOptimize(jacobian.data());
  }
}
BENCHMARK(BM_StateJacobian_FiniteDifference);

//==============================================================================
// Opt-in overhead: a single forward step() with differentiable ON. The
// difference from BM_Step_DiffOff is the cost of caching the per-step
// Jacobians.
static void BM_Step_DiffOn(benchmark::State& state)
{
  auto world = buildContactScene(/*differentiable=*/true);
  const Eigen::VectorXd x0 = world->getStateVector();
  const Eigen::VectorXd u0 = world->getControlVector();

  for (auto _ : state) {
    world->setStateVector(x0);
    world->setControlVector(u0);
    world->step();
    benchmark::DoNotOptimize(world->getStateVector().data());
  }
}
BENCHMARK(BM_Step_DiffOn);

//==============================================================================
// Baseline forward step() with differentiable OFF (no gradient bookkeeping).
static void BM_Step_DiffOff(benchmark::State& state)
{
  auto world = buildContactScene(/*differentiable=*/false);
  const Eigen::VectorXd x0 = world->getStateVector();
  const Eigen::VectorXd u0 = world->getControlVector();

  for (auto _ : state) {
    world->setStateVector(x0);
    world->setControlVector(u0);
    world->step();
    benchmark::DoNotOptimize(world->getStateVector().data());
  }
}
BENCHMARK(BM_Step_DiffOff);

BENCHMARK_MAIN();
