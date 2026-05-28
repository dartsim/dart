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

#include <dart/simulation/experimental/detail/deformable_vbd/block_descent.hpp>

#include <Eigen/Core>
#include <benchmark/benchmark.h>

#include <vector>

namespace vbd = dart::simulation::experimental::detail::deformable_vbd;

namespace {

using Vec3 = Eigen::Vector3d;

//==============================================================================
// A square cloth-like grid of `side` x `side` vertices connected by structural
// springs (axis-aligned) plus shear springs (diagonals), with the top row
// pinned. This produces a multi-color vertex graph exercising the colored
// Gauss-Seidel sweep.
struct GridScene
{
  std::vector<Vec3> positions;
  std::vector<double> masses;
  std::vector<std::uint8_t> fixed;
  std::vector<Vec3> inertialTargets;
  std::vector<vbd::SpringElement> springs;
  vbd::VertexColoring coloring;
  vbd::SpringAdjacency adjacency;
  double stiffness = 500.0;
  double timeStep = 0.02;
};

GridScene makeGrid(int side)
{
  GridScene scene;
  const double spacing = 1.0;
  const auto index = [side](int r, int c) {
    return static_cast<std::uint32_t>(r * side + c);
  };
  for (int r = 0; r < side; ++r) {
    for (int c = 0; c < side; ++c) {
      scene.positions.emplace_back(spacing * c, -spacing * r, 0.0);
      scene.masses.push_back(1.0);
      scene.fixed.push_back(r == 0 ? 1u : 0u);
    }
  }
  scene.inertialTargets = scene.positions;
  for (std::size_t i = 0; i < scene.positions.size(); ++i) {
    if (scene.fixed[i] == 0u) {
      scene.inertialTargets[i] += Vec3(0.0, -0.05, 0.01);
    }
  }
  const double rest = 0.9 * spacing;
  const double diagRest = 0.9 * spacing * 1.41421356;
  for (int r = 0; r < side; ++r) {
    for (int c = 0; c < side; ++c) {
      if (c + 1 < side) {
        scene.springs.push_back({index(r, c), index(r, c + 1), rest});
      }
      if (r + 1 < side) {
        scene.springs.push_back({index(r, c), index(r + 1, c), rest});
      }
      if (r + 1 < side && c + 1 < side) {
        scene.springs.push_back({index(r, c), index(r + 1, c + 1), diagRest});
        scene.springs.push_back({index(r + 1, c), index(r, c + 1), diagRest});
      }
    }
  }
  scene.coloring = vbd::colorSprings(scene.positions.size(), scene.springs);
  scene.adjacency
      = vbd::SpringAdjacency::build(scene.positions.size(), scene.springs);
  return scene;
}

} // namespace

//==============================================================================
// One full implicit-Euler step (20 VBD sweeps) on a square spring grid.
static void BM_VbdBlockDescentGridStep(benchmark::State& state)
{
  const int side = static_cast<int>(state.range(0));
  const GridScene base = makeGrid(side);
  vbd::BlockDescentOptions options;
  options.iterations = 20;

  for (auto _ : state) {
    state.PauseTiming();
    std::vector<Vec3> positions = base.positions;
    state.ResumeTiming();
    vbd::BlockDescentStats stats = vbd::blockDescentMassSpring(
        positions,
        base.masses,
        base.fixed,
        base.inertialTargets,
        base.springs,
        base.stiffness,
        base.timeStep,
        base.coloring,
        base.adjacency,
        options);
    benchmark::DoNotOptimize(stats.finalResidualNormSquared);
    benchmark::DoNotOptimize(positions);
  }
  state.counters["vertices"] = static_cast<double>(base.positions.size());
  state.counters["springs"] = static_cast<double>(base.springs.size());
  state.counters["colors"] = static_cast<double>(base.coloring.colorCount());
}
BENCHMARK(BM_VbdBlockDescentGridStep)->Arg(8)->Arg(16)->Arg(32);

//==============================================================================
// Cost of building the vertex coloring for a grid (precomputed once per mesh).
static void BM_VbdColorGrid(benchmark::State& state)
{
  const int side = static_cast<int>(state.range(0));
  const GridScene base = makeGrid(side);
  for (auto _ : state) {
    vbd::VertexColoring coloring
        = vbd::colorSprings(base.positions.size(), base.springs);
    benchmark::DoNotOptimize(coloring.colorOfVertex);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_VbdColorGrid)->Arg(8)->Arg(16)->Arg(32);

BENCHMARK_MAIN();
