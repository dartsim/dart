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

#include <dart/simulation/compute/parallel_executor.hpp>
#include <dart/simulation/detail/deformable_vbd/block_descent.hpp>
#include <dart/simulation/detail/deformable_vbd/parallel_block_descent.hpp>

#include <Eigen/Core>
#include <benchmark/benchmark.h>

#include <vector>

namespace vbd = dart::simulation::detail::deformable_vbd;
namespace compute = dart::simulation::compute;

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

namespace {

// A tetrahedral bar of `cubes` unit cubes stacked along z, each split into six
// Kuhn tetrahedra sharing the main diagonal. The k=0 ring is pinned.
struct TetBarScene
{
  std::vector<Vec3> positions;
  std::vector<double> masses;
  std::vector<std::uint8_t> fixed;
  std::vector<Vec3> inertialTargets;
  std::vector<vbd::TetMeshElement> tets;
  vbd::VertexColoring coloring;
  vbd::TetAdjacency adjacency;
  double mu = 3000.0;
  double lambda = 6000.0;
  double timeStep = 0.01;
};

TetBarScene makeTetBar(int cubes)
{
  TetBarScene scene;
  const auto index = [](int i, int j, int k) {
    return static_cast<std::uint32_t>(((k * 2) + j) * 2 + i);
  };
  for (int k = 0; k <= cubes; ++k) {
    for (int j = 0; j < 2; ++j) {
      for (int i = 0; i < 2; ++i) {
        scene.positions.emplace_back(i, j, k);
        scene.masses.push_back(1.0);
        scene.fixed.push_back(k == 0 ? 1u : 0u);
      }
    }
  }
  scene.inertialTargets = scene.positions;
  for (std::size_t v = 0; v < scene.positions.size(); ++v) {
    if (scene.fixed[v] == 0u) {
      scene.inertialTargets[v] += Vec3(0.02, -0.03, 0.0);
    }
  }

  static const int kuhn[6][4]
      = {{0, 1, 3, 7},
         {0, 3, 2, 7},
         {0, 2, 6, 7},
         {0, 6, 4, 7},
         {0, 4, 5, 7},
         {0, 5, 1, 7}};
  for (int c = 0; c < cubes; ++c) {
    std::array<std::uint32_t, 8> corner{};
    for (int n = 0; n < 8; ++n) {
      const int i = n & 1;
      const int j = (n >> 1) & 1;
      const int k = c + ((n >> 2) & 1);
      corner[static_cast<std::size_t>(n)] = index(i, j, k);
    }
    for (const auto& t : kuhn) {
      const std::array<std::uint32_t, 4> v
          = {corner[t[0]], corner[t[1]], corner[t[2]], corner[t[3]]};
      vbd::TetRestShape rest = vbd::makeTetRestShape(
          {scene.positions[v[0]],
           scene.positions[v[1]],
           scene.positions[v[2]],
           scene.positions[v[3]]});
      scene.tets.push_back({v, rest});
    }
  }
  scene.coloring = vbd::colorTetMesh(scene.positions.size(), scene.tets);
  scene.adjacency
      = vbd::TetAdjacency::build(scene.positions.size(), scene.tets);
  return scene;
}

} // namespace

//==============================================================================
// One implicit-Euler step (20 VBD sweeps) on a tetrahedral Neo-Hookean bar.
static void BM_VbdTetMeshStep(benchmark::State& state)
{
  const int cubes = static_cast<int>(state.range(0));
  const TetBarScene base = makeTetBar(cubes);
  vbd::BlockDescentOptions options;
  options.iterations = 20;

  for (auto _ : state) {
    state.PauseTiming();
    std::vector<Vec3> positions = base.positions;
    state.ResumeTiming();
    vbd::BlockDescentStats stats = vbd::blockDescentTetMesh(
        positions,
        base.masses,
        base.fixed,
        base.inertialTargets,
        base.tets,
        base.mu,
        base.lambda,
        base.timeStep,
        base.coloring,
        base.adjacency,
        options);
    benchmark::DoNotOptimize(stats.finalResidualNormSquared);
    benchmark::DoNotOptimize(positions);
  }
  state.counters["vertices"] = static_cast<double>(base.positions.size());
  state.counters["tets"] = static_cast<double>(base.tets.size());
  state.counters["colors"] = static_cast<double>(base.coloring.colorCount());
}
BENCHMARK(BM_VbdTetMeshStep)->Arg(4)->Arg(16)->Arg(64);

namespace {

// The TinyVBD reference default scene: a 20-vertex strand tilted 30 degrees,
// pinned at vertex 0, with structural springs (i, i+1) and skip springs
// (i, i+2), a 1:1000 tip mass ratio, and 100 iterations per step. Matched here
// for a per-step CPU comparison against the TinyVBD reference implementation
// (same vertex/spring/iteration counts and the same per-vertex VBD update).
struct StrandScene
{
  std::vector<Vec3> positions;
  std::vector<double> masses;
  std::vector<std::uint8_t> fixed;
  std::vector<Vec3> inertialTargets;
  std::vector<vbd::SpringElement> springs;
  vbd::VertexColoring coloring;
  vbd::SpringAdjacency adjacency;
  double stiffness = 1.0e8;
  double timeStep = 1.0 / 60.0;
};

StrandScene makeTiltedStrand(int numVerts)
{
  StrandScene scene;
  const double spacing = 0.05;
  const double tanAngle = 0.57735; // 30 degrees
  for (int i = 0; i < numVerts; ++i) {
    scene.positions.emplace_back(
        i * spacing, 2.0 + i * spacing * tanAngle, 0.0);
    scene.masses.push_back(i == numVerts - 1 ? 1000.0 : 1.0);
    scene.fixed.push_back(i == 0 ? 1u : 0u);
  }
  for (int i = 0; i + 1 < numVerts; ++i) {
    const double rest = (scene.positions[i + 1] - scene.positions[i]).norm();
    scene.springs.push_back(
        {static_cast<std::uint32_t>(i),
         static_cast<std::uint32_t>(i + 1),
         rest});
  }
  for (int i = 0; i + 2 < numVerts; ++i) { // skip springs (i, i+2)
    const double rest = (scene.positions[i + 2] - scene.positions[i]).norm();
    scene.springs.push_back(
        {static_cast<std::uint32_t>(i),
         static_cast<std::uint32_t>(i + 2),
         rest});
  }
  scene.inertialTargets = scene.positions;
  const Vec3 gravityStep(0.0, -10.0 * scene.timeStep * scene.timeStep, 0.0);
  for (std::size_t i = 0; i < scene.positions.size(); ++i) {
    if (scene.fixed[i] == 0u) {
      scene.inertialTargets[i] += gravityStep;
    }
  }
  scene.coloring = vbd::colorSprings(scene.positions.size(), scene.springs);
  scene.adjacency
      = vbd::SpringAdjacency::build(scene.positions.size(), scene.springs);
  return scene;
}

} // namespace

//==============================================================================
// One step (100 sweeps) on the TinyVBD reference tilted-strand scene, for a
// per-step CPU comparison against the TinyVBD reference implementation.
static void BM_VbdTinyStrandStep(benchmark::State& state)
{
  const StrandScene base = makeTiltedStrand(static_cast<int>(state.range(0)));
  vbd::BlockDescentOptions options;
  options.iterations = 100;
  for (auto _ : state) {
    state.PauseTiming();
    std::vector<Vec3> positions = base.inertialTargets;
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
}
BENCHMARK(BM_VbdTinyStrandStep)->Arg(20)->Arg(100)->Arg(400);

//==============================================================================
// Thread-scaling of the colored Gauss-Seidel sweep on a large spring grid: one
// step (20 sweeps) at increasing worker-thread counts. Same-color vertices are
// independent, so this is VBD's CPU parallelism in action.
static void BM_VbdParallelGridStep(benchmark::State& state)
{
  const int side = 96;
  const unsigned int threads = static_cast<unsigned int>(state.range(0));
  const GridScene base = makeGrid(side);
  compute::ParallelExecutor executor(threads);
  vbd::BlockDescentOptions options;
  options.iterations = 20;
  for (auto _ : state) {
    state.PauseTiming();
    std::vector<Vec3> positions = base.positions;
    state.ResumeTiming();
    vbd::BlockDescentStats stats = vbd::parallelBlockDescentMassSpring(
        positions,
        base.masses,
        base.fixed,
        base.inertialTargets,
        base.springs,
        base.stiffness,
        base.timeStep,
        base.coloring,
        base.adjacency,
        options,
        executor);
    benchmark::DoNotOptimize(stats.finalResidualNormSquared);
    benchmark::DoNotOptimize(positions);
  }
  state.counters["threads"] = threads;
  state.counters["vertices"] = static_cast<double>(side * side);
}
BENCHMARK(BM_VbdParallelGridStep)->Arg(1)->Arg(2)->Arg(4)->Arg(8);

BENCHMARK_MAIN();
