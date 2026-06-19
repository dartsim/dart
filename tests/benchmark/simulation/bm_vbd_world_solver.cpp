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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

// Compares the experimental Vertex Block Descent inner solver against the
// default gradient-descent deformable solver, stepping the same contact-free
// mass-spring grid through the real World pipeline. Both minimize the same
// variational objective, so this measures per-step CPU cost on matched scenes.

#include <dart/simulation/body/deformable_body.hpp>
#include <dart/simulation/comps/deformable_body.hpp>
#include <dart/simulation/compute/detail/world_step_stages.hpp>
#include <dart/simulation/compute/sequential_executor.hpp>
#include <dart/simulation/compute/world_step_stage.hpp>
#include <dart/simulation/detail/world_registry_access.hpp>
#include <dart/simulation/world.hpp>

#include <Eigen/Core>
#include <benchmark/benchmark.h>
#include <entt/entt.hpp>

#include <memory>

namespace sx = dart::simulation;
namespace compute = dart::simulation::compute;

namespace {

// A pinned square spring grid (structural + shear springs), top row fixed.
sx::DeformableBodyOptions makeGridOptions(int side)
{
  sx::DeformableBodyOptions options;
  const double spacing = 1.0;
  const auto index = [side](int r, int c) {
    return static_cast<std::size_t>(r * side + c);
  };
  for (int r = 0; r < side; ++r) {
    for (int c = 0; c < side; ++c) {
      options.positions.emplace_back(spacing * c, -spacing * r, 0.0);
      options.masses.push_back(1.0);
    }
  }
  for (int c = 0; c < side; ++c) {
    options.fixedNodes.push_back(index(0, c));
  }
  for (int r = 0; r < side; ++r) {
    for (int c = 0; c < side; ++c) {
      if (c + 1 < side) {
        options.edges.push_back({index(r, c), index(r, c + 1), -1.0});
      }
      if (r + 1 < side) {
        options.edges.push_back({index(r, c), index(r + 1, c), -1.0});
      }
      if (r + 1 < side && c + 1 < side) {
        options.edges.push_back({index(r, c), index(r + 1, c + 1), -1.0});
        options.edges.push_back({index(r + 1, c), index(r, c + 1), -1.0});
      }
    }
  }
  options.edgeStiffness = 500.0;
  return options;
}

std::unique_ptr<sx::World> makeGridWorld(int side, bool enableVbd)
{
  auto world = std::make_unique<sx::World>();
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  world->setTimeStep(0.005);
  world->addDeformableBody("grid", makeGridOptions(side));
  if (enableVbd) {
    auto& registry = dart::simulation::detail::registryOf(*world);
    for (const auto entity : registry.view<sx::comps::DeformableBodyTag>()) {
      // Cap at 50 sweeps but stop early once converged, matching how the
      // default solver terminates on residual.
      registry.emplace_or_replace<sx::comps::DeformableVbdConfig>(
          entity, sx::comps::DeformableVbdConfig{true, 50, 1e-6});
    }
  }
  return world;
}

void runWorldStepBenchmark(benchmark::State& state, bool enableVbd)
{
  const int side = static_cast<int>(state.range(0));
  auto world = makeGridWorld(side, enableVbd);
  compute::SequentialExecutor executor;
  compute::DeformableDynamicsStage stage;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage);

  for (auto _ : state) {
    world->step(executor, pipeline);
    benchmark::ClobberMemory();
  }
  state.counters["vertices"] = static_cast<double>(side * side);
}

} // namespace

//==============================================================================
static void BM_VbdWorldStepDefault(benchmark::State& state)
{
  runWorldStepBenchmark(state, /*enableVbd=*/false);
}
BENCHMARK(BM_VbdWorldStepDefault)->Arg(8)->Arg(16)->Arg(24);

//==============================================================================
static void BM_VbdWorldStepVbd(benchmark::State& state)
{
  runWorldStepBenchmark(state, /*enableVbd=*/true);
}
BENCHMARK(BM_VbdWorldStepVbd)->Arg(8)->Arg(16)->Arg(24);

BENCHMARK_MAIN();
