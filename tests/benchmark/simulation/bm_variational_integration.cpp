/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License
 */

// Scaling evidence for the linear-time variational integrator (PLAN-084). The
// RIQN root update uses an O(n) articulated-body-inertia inverse-mass solve and
// the DRNEA residual is O(n), so the per-step cost is O(iterations * n). To
// isolate intrinsic scaling from chaotic chain dynamics, the full-step
// benchmark resets to a fixed, well-conditioned configuration each iteration
// (the passive chain would otherwise collapse into stiff states over the
// benchmark's many steps, inflating the RIQN iteration count). Google
// Benchmark's Complexity(oN) fits the measured times to a complexity class.

#include <dart/simulation/comps/joint.hpp>
#include <dart/simulation/comps/multibody.hpp>
#include <dart/simulation/compute/variational_integration.hpp>
#include <dart/simulation/detail/world_registry_access.hpp>
#include <dart/simulation/multibody/multibody.hpp>
#include <dart/simulation/world.hpp>

#include <Eigen/Geometry>
#include <benchmark/benchmark.h>
#include <entt/entt.hpp>

#include <memory>
#include <string>
#include <vector>

namespace sx = dart::simulation;
namespace sxc = dart::simulation::compute;

namespace {

// A fixed-base n-link revolute chain at a mild configuration, with the initial
// joint positions captured so a benchmark can reset to them each iteration.
struct Chain
{
  std::unique_ptr<sx::World> world;
  const sx::comps::MultibodyStructure* structure = nullptr;
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
  std::vector<entt::entity> joints;
  std::vector<Eigen::VectorXd> initialPositions;
};

Chain makeChain(int n)
{
  Chain chain;
  chain.world = std::make_unique<sx::World>();
  sx::World& world = *chain.world;
  auto robot = world.addMultibody("chain");
  auto parent = robot.addLink("base");
  for (int i = 0; i < n; ++i) {
    Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
    offset.translation() = Eigen::Vector3d(0.3, 0.0, 0.0);
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
  const auto joints = robot.getJoints();
  for (std::size_t i = 0; i < joints.size(); ++i) {
    auto joint = joints[i];
    joint.setPosition(
        Eigen::VectorXd::Constant(1, 0.1 * static_cast<double>(i % 5) - 0.2));
  }
  world.updateKinematics();

  auto& registry = dart::simulation::detail::registryOf(world);
  auto view = registry.view<sx::comps::MultibodyStructure>();
  chain.structure = &registry.get<sx::comps::MultibodyStructure>(*view.begin());
  chain.gravity = world.getGravity();
  for (const auto jointEntity : chain.structure->joints) {
    chain.joints.push_back(jointEntity);
    chain.initialPositions.push_back(
        registry.get<sx::comps::Joint>(jointEntity).position);
  }
  return chain;
}

void resetToInitial(sx::detail::WorldRegistry& registry, const Chain& chain)
{
  for (std::size_t j = 0; j < chain.joints.size(); ++j) {
    auto& joint = registry.get<sx::comps::Joint>(chain.joints[j]);
    joint.position = chain.initialPositions[j];
    joint.velocity.setZero();
    joint.acceleration.setZero();
  }
}

} // namespace

// One full variational step (RIQN root-find with the O(n) ABI inverse-mass
// solve) from a fixed configuration, versus chain length n. The per-step
// configuration is reset (untimed) each iteration so the measurement reflects
// intrinsic scaling rather than the evolving chain's convergence difficulty.
void BM_VariationalStep(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  Chain chain = makeChain(n);
  auto& registry = dart::simulation::detail::registryOf(*chain.world);
  for (auto _ : state) {
    state.PauseTiming();
    resetToInitial(registry, chain);
    sxc::MultibodyVariationalState vstate;
    state.ResumeTiming();
    // Generous iteration budget so the scaling measurement is never cut short
    // by the non-convergence guard at the largest chain lengths.
    sxc::integrateMultibodyVariational(
        registry, *chain.structure, chain.gravity, 1e-3, vstate, 400);
  }
  state.SetComplexityN(n);
}
BENCHMARK(BM_VariationalStep)
    ->RangeMultiplier(2)
    // The exact recursive-Jacobian Newton preconditioner keeps the iteration
    // count small and length-independent (~3 iters at every n), so the per-step
    // cost stays linear across the long-chain regime the integrator targets.
    ->Range(4, 128)
    ->Complexity(benchmark::oN);

// The O(n) articulated-body inverse-mass apply alone (the RIQN quasi-Newton
// step kernel), isolating the linear-time solve from residual evaluation.
void BM_ArticulatedInverseMass(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  Chain chain = makeChain(n);
  auto& registry = dart::simulation::detail::registryOf(*chain.world);
  const Eigen::VectorXd impulse = Eigen::VectorXd::Ones(n);
  for (auto _ : state) {
    benchmark::DoNotOptimize(
        sxc::computeMultibodyInverseMassProduct(
            registry, *chain.structure, impulse));
  }
  state.SetComplexityN(n);
}
BENCHMARK(BM_ArticulatedInverseMass)
    ->RangeMultiplier(2)
    ->Range(4, 128)
    ->Complexity(benchmark::oN);

BENCHMARK_MAIN();
