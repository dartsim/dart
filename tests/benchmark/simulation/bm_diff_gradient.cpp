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

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/diff/step_derivatives.hpp>
#include <dart/simulation/multibody/multibody.hpp>
#include <dart/simulation/world.hpp>
#include <dart/simulation/world_options.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <benchmark/benchmark.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <cstdint>

namespace sx = dart::simulation;

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

//==============================================================================
// A scalable free-body contact scene: N spheres resting on (and sliding along)
// a static ground, spaced apart so each has its own active clamping ground
// contact (no body-body overlap). State dim = 6*N, contacts = N. This is the
// regime where DART's analytic contact gradient (implicit differentiation of
// the boxed-LCP solve) pays off: the finite-difference baseline must re-run
// collision detection and the LCP solve for each of 2*(6*N) perturbed steps,
// while the analytic path resolves the contacts once and differentiates the
// solved system in a single backward assembly — so the speedup ratio grows with
// the number of bodies/contacts, the paper's "scales with system size" result.
std::unique_ptr<sx::World> buildContactPile(int nBodies, bool differentiable)
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
      sx::CollisionShape::makeBox(
          Eigen::Vector3d(4.0 + 1.5 * nBodies, 5.0, 0.5)));
  ground.setFriction(0.0);

  for (int i = 0; i < nBodies; ++i) {
    sx::RigidBodyOptions sphereOptions;
    sphereOptions.mass = kSphereMass;
    sphereOptions.position = Eigen::Vector3d(
        1.2 * static_cast<double>(i), 0.0, kSphereRadius - 5e-5);
    sphereOptions.linearVelocity = Eigen::Vector3d(0.3, 0.0, -0.2);
    auto sphere
        = world->addRigidBody("sphere" + std::to_string(i), sphereOptions);
    sphere.setCollisionShape(sx::CollisionShape::makeSphere(kSphereRadius));
    sphere.setFriction(0.0);
  }

  return world;
}

//==============================================================================
// A contact-free N-link revolute chain (a pendulum chain), the smooth
// articulated analogue of the paper's systems for the DOF-scaling study. Each
// link hinges about the world Y axis with a +X offset so gravity produces a
// configuration-dependent torque; mass/inertia are uniform. ndof == nLinks.
//
// This scene exercises the contact-free smooth single-step Jacobian, which is
// where the analytic-vs-finite-difference speedup is governed purely by DOF
// (the finite-difference baseline runs 2*(2*ndof) forward steps; the analytic
// path is a single backward assembly). It lets the benchmark reproduce the
// paper's claim that the speedup ratio grows with the size of the dynamic
// system.
std::unique_ptr<sx::World> buildRevoluteChain(int nLinks, bool differentiable)
{
  sx::WorldOptions options;
  options.timeStep = kTimeStep;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.differentiable = differentiable;
  auto world = std::make_unique<sx::World>(options);

  auto chain = world->addMultibody("chain");
  auto parent = chain.addLink("base");

  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(0.5, 0.0, 0.0); // link length 0.5 m
  for (int i = 0; i < nLinks; ++i) {
    auto link = chain.addLink(
        "link" + std::to_string(i),
        parent,
        sx::JointSpec{
            .name = "hinge" + std::to_string(i),
            .type = sx::JointType::Revolute,
            .axis = Eigen::Vector3d::UnitY(),
            .transformFromParent = offset});
    link.setMass(1.0);
    link.setInertia(0.01 * Eigen::Matrix3d::Identity());
    parent = link;
  }

  return world;
}

//==============================================================================
// Read/apply a multibody's joint-space state [q; q̇] (the World-level state
// vector covers only rigid bodies, so the chain is addressed through its
// joints, in construction order).
Eigen::VectorXd readChainState(const sx::Multibody& mb)
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

void applyChainState(sx::Multibody& mb, const Eigen::VectorXd& state)
{
  const Eigen::Index ndof = state.size() / 2;
  Eigen::Index offset = 0;
  for (auto joint : mb.getJoints()) {
    const auto dof = static_cast<Eigen::Index>(joint.getDOFCount());
    if (dof == 0) {
      continue;
    }
    joint.setPosition(state.segment(offset, dof));
    joint.setVelocity(state.segment(ndof + offset, dof));
    offset += dof;
  }
}

// A small, well-conditioned nonzero state so the smooth Jacobian is exercised
// away from the trivial configuration.
Eigen::VectorXd nominalChainState(Eigen::Index ndof)
{
  Eigen::VectorXd x(2 * ndof);
  for (Eigen::Index i = 0; i < ndof; ++i) {
    x[i] = 0.1 + 0.01 * static_cast<double>(i);           // q
    x[ndof + i] = -0.05 + 0.005 * static_cast<double>(i); // q̇
  }
  return x;
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

//==============================================================================
// DOF-scaling speedup study (the paper's headline metric). For an N-link
// revolute chain, time the full single-step state Jacobian computed
// ANALYTICALLY (one getStepDerivatives()) against the SAME Jacobian recovered
// by central finite differencing (2 * (2*ndof) forward steps), and report the
// speedup ratio. The ratio is the hardware-normalized metric the paper's Table
// II uses (the finite-difference baseline normalizes out the machine), and it
// grows with DOF — reproducing the paper's result that the analytic speedup
// scales with the size of the dynamic system. This MEASURES and reports
// (counters dof, analytic_ms, fd_ms, speedup); it asserts no specific ratio.
static void BM_ChainJacobianSpeedup(benchmark::State& state)
{
  using clock = std::chrono::steady_clock;
  const int nLinks = static_cast<int>(state.range(0));

  auto analyticWorld = buildRevoluteChain(nLinks, /*differentiable=*/true);
  auto analyticMb = analyticWorld->getMultibody("chain");
  auto fdWorld = buildRevoluteChain(nLinks, /*differentiable=*/false);
  auto fdMb = fdWorld->getMultibody("chain");

  const Eigen::Index ndof = static_cast<Eigen::Index>(nLinks);
  const Eigen::VectorXd x0 = nominalChainState(ndof);
  const Eigen::Index stateSize = 2 * ndof;
  const double h = 1e-6;

  // Warm up the analytic path so first-call allocation is not timed.
  applyChainState(*analyticMb, x0);
  analyticWorld->step();
  benchmark::DoNotOptimize(
      analyticWorld->getStepDerivatives().stateJacobian.data());

  double analyticSeconds = 0.0;
  double fdSeconds = 0.0;
  // Per-call times are collected so the MEDIAN can be reported: the median is
  // robust to the sample COUNT (unlike the minimum, whose order statistic
  // drifts downward as more samples are drawn) and to occasional load/turbo
  // spikes, so it is the fair statistic for the head-to-head reference
  // comparison (compared median-vs-median at matched methodology).
  std::vector<double> analyticTimes;
  std::vector<double> fdTimes;
  std::int64_t reps = 0;

  for (auto _ : state) {
    // Analytic: a single backward assembly returns the full ∂x'/∂x.
    applyChainState(*analyticMb, x0);
    const auto a0 = clock::now();
    analyticWorld->step();
    const sx::StepDerivatives derivatives = analyticWorld->getStepDerivatives();
    const auto a1 = clock::now();
    benchmark::DoNotOptimize(derivatives.stateJacobian.data());
    const double analyticCall = std::chrono::duration<double>(a1 - a0).count();
    analyticSeconds += analyticCall;
    analyticTimes.push_back(analyticCall);

    // Finite difference: central-difference step() over each state component.
    Eigen::MatrixXd jacobian(stateSize, stateSize);
    const auto f0 = clock::now();
    for (Eigen::Index k = 0; k < stateSize; ++k) {
      Eigen::VectorXd plus = x0;
      Eigen::VectorXd minus = x0;
      plus[k] += h;
      minus[k] -= h;

      applyChainState(*fdMb, plus);
      fdWorld->step();
      const Eigen::VectorXd nextPlus = readChainState(*fdMb);

      applyChainState(*fdMb, minus);
      fdWorld->step();
      const Eigen::VectorXd nextMinus = readChainState(*fdMb);

      jacobian.col(k) = (nextPlus - nextMinus) / (2.0 * h);
    }
    const auto f1 = clock::now();
    benchmark::DoNotOptimize(jacobian.data());
    const double fdCall = std::chrono::duration<double>(f1 - f0).count();
    fdSeconds += fdCall;
    fdTimes.push_back(fdCall);

    ++reps;
  }

  const auto median = [](std::vector<double>& xs) {
    if (xs.empty()) {
      return 0.0;
    }
    std::sort(xs.begin(), xs.end());
    const std::size_t mid = xs.size() / 2;
    return (xs.size() % 2 == 0) ? 0.5 * (xs[mid - 1] + xs[mid]) : xs[mid];
  };
  const double analyticMedian = median(analyticTimes);
  const double fdMedian = median(fdTimes);

  const auto denom = static_cast<double>(std::max<std::int64_t>(reps, 1));
  state.counters["dof"] = static_cast<double>(ndof);
  state.counters["analytic_ms"] = 1e3 * analyticSeconds / denom;
  state.counters["fd_ms"] = 1e3 * fdSeconds / denom;
  state.counters["speedup"]
      = analyticSeconds > 0.0 ? fdSeconds / analyticSeconds : 0.0;
  // Median per-call times (sample-count-robust) for the reference comparison.
  state.counters["analytic_ms_median"] = 1e3 * analyticMedian;
  state.counters["fd_ms_median"] = 1e3 * fdMedian;
  state.counters["speedup_median"]
      = analyticMedian > 0.0 ? fdMedian / analyticMedian : 0.0;
}
BENCHMARK(BM_ChainJacobianSpeedup)
    ->Arg(1)
    ->Arg(2)
    ->Arg(5)
    ->Arg(9)
    ->Arg(18)
    ->Arg(32)
    ->Unit(benchmark::kMillisecond);

//==============================================================================
// Contact-path DOF/contact-scaling speedup study (the regime where DART's
// analytic gradient is a true implicit differentiation, not finite-difference).
// For N free bodies each in an active ground contact (state dim 6*N, N
// contacts), time the analytic full state Jacobian (one getStepDerivatives())
// against central finite differencing of step() over the 6*N rigid-body state
// components (the World state vector covers rigid bodies, so it is used here).
// The finite-difference baseline re-runs collision + LCP for every perturbed
// step, so its cost grows with the number of contacts while the analytic
// backward assembly does not — the speedup ratio climbs with N. Reported as
// counters (dof, contacts, analytic_ms, fd_ms, speedup); no ratio is asserted.
static void BM_ContactScaleSpeedup(benchmark::State& state)
{
  using clock = std::chrono::steady_clock;
  const int nBodies = static_cast<int>(state.range(0));

  auto analyticWorld = buildContactPile(nBodies, /*differentiable=*/true);
  auto fdWorld = buildContactPile(nBodies, /*differentiable=*/false);

  const Eigen::VectorXd x0 = analyticWorld->getStateVector();
  const Eigen::VectorXd u0 = analyticWorld->getControlVector();
  const Eigen::Index stateSize = x0.size();
  const double h = 1e-6;

  // Warm up the analytic path.
  analyticWorld->setStateVector(x0);
  analyticWorld->setControlVector(u0);
  analyticWorld->step();
  benchmark::DoNotOptimize(
      analyticWorld->getStepDerivatives().stateJacobian.data());

  double analyticSeconds = 0.0;
  double fdSeconds = 0.0;
  std::int64_t reps = 0;

  for (auto _ : state) {
    analyticWorld->setStateVector(x0);
    analyticWorld->setControlVector(u0);
    const auto a0 = clock::now();
    analyticWorld->step();
    const sx::StepDerivatives derivatives = analyticWorld->getStepDerivatives();
    const auto a1 = clock::now();
    benchmark::DoNotOptimize(derivatives.stateJacobian.data());
    analyticSeconds += std::chrono::duration<double>(a1 - a0).count();

    Eigen::MatrixXd jacobian(stateSize, stateSize);
    const auto f0 = clock::now();
    for (Eigen::Index k = 0; k < stateSize; ++k) {
      Eigen::VectorXd plus = x0;
      Eigen::VectorXd minus = x0;
      plus[k] += h;
      minus[k] -= h;

      fdWorld->setStateVector(plus);
      fdWorld->setControlVector(u0);
      fdWorld->step();
      const Eigen::VectorXd nextPlus = fdWorld->getStateVector();

      fdWorld->setStateVector(minus);
      fdWorld->setControlVector(u0);
      fdWorld->step();
      const Eigen::VectorXd nextMinus = fdWorld->getStateVector();

      jacobian.col(k) = (nextPlus - nextMinus) / (2.0 * h);
    }
    const auto f1 = clock::now();
    benchmark::DoNotOptimize(jacobian.data());
    fdSeconds += std::chrono::duration<double>(f1 - f0).count();

    ++reps;
  }

  const auto denom = static_cast<double>(std::max<std::int64_t>(reps, 1));
  state.counters["dof"] = static_cast<double>(stateSize);
  state.counters["contacts"] = static_cast<double>(nBodies);
  state.counters["analytic_ms"] = 1e3 * analyticSeconds / denom;
  state.counters["fd_ms"] = 1e3 * fdSeconds / denom;
  state.counters["speedup"]
      = analyticSeconds > 0.0 ? fdSeconds / analyticSeconds : 0.0;
}
BENCHMARK(BM_ContactScaleSpeedup)
    ->Arg(1)
    ->Arg(2)
    ->Arg(4)
    ->Arg(8)
    ->Arg(16)
    ->Arg(32)
    ->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
