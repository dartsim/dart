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
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
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

#include <dart/simulation/experimental/body/collision_shape.hpp>
#include <dart/simulation/experimental/body/deformable_body.hpp>
#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/body/rigid_body_options.hpp>
#include <dart/simulation/experimental/compute/sequential_executor.hpp>
#include <dart/simulation/experimental/compute/world_step_stage.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <Eigen/Core>
#include <benchmark/benchmark.h>

#include <algorithm>
#include <string>
#include <vector>

#include <cmath>
#include <cstddef>

namespace sx = dart::simulation::experimental;

namespace {

//==============================================================================
struct DeformableGridWorld
{
  DeformableGridWorld(int columns, int rows, bool withGround)
  {
    columns = std::max(columns, 2);
    rows = std::max(rows, 2);

    sx::DeformableBodyOptions options;
    options.edgeStiffness = 80.0;
    options.damping = 1.0;

    const auto index = [columns](int col, int row) {
      return static_cast<std::size_t>(row * columns + col);
    };

    constexpr double spacing = 0.08;
    const double halfWidth = 0.5 * spacing * static_cast<double>(columns - 1);
    for (int row = 0; row < rows; ++row) {
      for (int col = 0; col < columns; ++col) {
        options.positions.push_back(
            Eigen::Vector3d(
                spacing * static_cast<double>(col) - halfWidth,
                0.01 * std::sin(0.31 * static_cast<double>(col + row)),
                1.0 - 0.05 * static_cast<double>(row)));
        options.velocities.push_back(Eigen::Vector3d::Zero());
        options.masses.push_back(0.05);
      }
    }

    options.fixedNodes = {index(0, 0), index(columns - 1, 0)};

    for (int row = 0; row < rows; ++row) {
      for (int col = 0; col < columns; ++col) {
        if (col + 1 < columns) {
          options.edges.push_back({index(col, row), index(col + 1, row), -1.0});
        }
        if (row + 1 < rows) {
          options.edges.push_back({index(col, row), index(col, row + 1), -1.0});
        }
        if (col + 1 < columns && row + 1 < rows) {
          options.edges.push_back(
              {index(col, row), index(col + 1, row + 1), -1.0});
          options.edges.push_back(
              {index(col + 1, row), index(col, row + 1), -1.0});
        }
      }
    }

    if (withGround) {
      sx::RigidBodyOptions groundOptions;
      groundOptions.isStatic = true;
      groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.05);
      auto ground = world.addRigidBody("ground", groundOptions);
      ground.setCollisionShape(
          sx::CollisionShape::makeBox(Eigen::Vector3d(4.0, 4.0, 0.05)));
      ground.setDeformableGroundBarrier(true);
    }

    body = world.addDeformableBody("grid", options);
    nodeCount = body.getNodeCount();
    edgeCount = body.getEdgeCount();

    world.setTimeStep(1.0 / 240.0);
    world.enterSimulationMode();
  }

  sx::World world;
  sx::DeformableBody body;
  std::size_t nodeCount = 0;
  std::size_t edgeCount = 0;
};

//==============================================================================
struct RigidOnlyWorld
{
  explicit RigidOnlyWorld(int staticBodyCount)
  {
    for (int i = 0; i < staticBodyCount; ++i) {
      sx::RigidBodyOptions options;
      options.isStatic = true;
      options.position
          = Eigen::Vector3d(2.0 * static_cast<double>(i), 0.0, -0.05);
      auto body
          = world.addRigidBody("static_box_" + std::to_string(i), options);
      body.setCollisionShape(
          sx::CollisionShape::makeBox(Eigen::Vector3d(1.0, 1.0, 0.05)));
    }

    world.setTimeStep(1.0 / 240.0);
    world.enterSimulationMode();
  }

  sx::World world;
};

//==============================================================================
void BM_WorldStepWithoutDeformables(benchmark::State& state)
{
  const auto staticBodyCount = static_cast<int>(state.range(0));
  RigidOnlyWorld fixture(staticBodyCount);

  for (auto _ : state) {
    fixture.world.step();
    benchmark::DoNotOptimize(fixture.world.getFrame());
  }

  state.counters["static_bodies"] = static_cast<double>(staticBodyCount);
}

//==============================================================================
void BM_DeformableGridStep(benchmark::State& state)
{
  const auto columns = static_cast<int>(state.range(0));
  const auto rows = static_cast<int>(state.range(1));
  const bool withGround = state.range(2) != 0;
  DeformableGridWorld fixture(columns, rows, withGround);

  for (auto _ : state) {
    fixture.world.step();
    benchmark::DoNotOptimize(
        fixture.body.getPosition(fixture.nodeCount - 1).z());
  }

  state.counters["nodes"] = static_cast<double>(fixture.nodeCount);
  state.counters["edges"] = static_cast<double>(fixture.edgeCount);
  state.counters["ground"] = withGround ? 1.0 : 0.0;
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * fixture.nodeCount));
}

//==============================================================================
void BM_DeformableGridStage(benchmark::State& state)
{
  const auto columns = static_cast<int>(state.range(0));
  const auto rows = static_cast<int>(state.range(1));
  const bool withGround = state.range(2) != 0;
  DeformableGridWorld fixture(columns, rows, withGround);
  sx::compute::SequentialExecutor executor;
  sx::compute::DeformableDynamicsStage deformableStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(deformableStage);

  for (auto _ : state) {
    fixture.world.step(executor, pipeline);
    benchmark::DoNotOptimize(
        fixture.body.getPosition(fixture.nodeCount - 1).z());
  }

  const auto& stats = deformableStage.getLastStats();
  state.counters["nodes"] = static_cast<double>(fixture.nodeCount);
  state.counters["edges"] = static_cast<double>(fixture.edgeCount);
  state.counters["ground"] = withGround ? 1.0 : 0.0;
  state.counters["objective_evals"]
      = static_cast<double>(stats.objectiveEvaluations);
  state.counters["solver_iterations"]
      = static_cast<double>(stats.solverIterations);
  state.counters["line_search_trials"]
      = static_cast<double>(stats.lineSearchTrials);
  state.counters["line_search_rejects"]
      = static_cast<double>(stats.rejectedLineSearchCandidates);
  state.counters["accepted_steps"]
      = static_cast<double>(stats.acceptedLineSearchSteps);
  state.counters["initial_projections"]
      = static_cast<double>(stats.initialProjectionCount);
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * fixture.nodeCount));
}

} // namespace

BENCHMARK(BM_WorldStepWithoutDeformables)->Arg(0)->Arg(32);

BENCHMARK(BM_DeformableGridStep)
    ->Args({8, 4, 0})
    ->Args({16, 8, 0})
    ->Args({32, 16, 0})
    ->Args({16, 8, 1})
    ->Args({32, 16, 1});

BENCHMARK(BM_DeformableGridStage)
    ->Args({8, 4, 0})
    ->Args({16, 8, 0})
    ->Args({32, 16, 0})
    ->Args({16, 8, 1})
    ->Args({32, 16, 1});

BENCHMARK_MAIN();
