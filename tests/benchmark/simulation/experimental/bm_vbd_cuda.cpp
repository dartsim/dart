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

// Compares per-step CPU and GPU VBD block descent on a square spring grid with
// a fixed sweep budget, so the GPU's graph-colored parallelism can be measured
// against the single-threaded CPU path on matched scenes.

#include <dart/simulation/experimental/detail/deformable_vbd/block_descent.hpp>

#include <Eigen/Core>
#include <benchmark/benchmark.h>
#include <dart/simulation/experimental/compute/cuda/vbd_block_descent_cuda.cuh>

#include <array>
#include <vector>

namespace vbd = dart::simulation::experimental::detail::deformable_vbd;
namespace cuda = dart::simulation::experimental::compute::cuda;

namespace {

using Vec3 = Eigen::Vector3d;

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
  double timeStep = 0.01;
};

GridScene makeGrid(int side)
{
  GridScene scene;
  const auto index = [side](int r, int c) {
    return static_cast<std::uint32_t>(r * side + c);
  };
  for (int r = 0; r < side; ++r) {
    for (int c = 0; c < side; ++c) {
      scene.positions.emplace_back(c, -r, 0.0);
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
  const double rest = 0.9;
  for (int r = 0; r < side; ++r) {
    for (int c = 0; c < side; ++c) {
      if (c + 1 < side) {
        scene.springs.push_back({index(r, c), index(r, c + 1), rest});
      }
      if (r + 1 < side) {
        scene.springs.push_back({index(r, c), index(r + 1, c), rest});
      }
      if (r + 1 < side && c + 1 < side) {
        scene.springs.push_back({index(r, c), index(r + 1, c + 1), rest * 1.4});
      }
    }
  }
  scene.coloring = vbd::colorSprings(scene.positions.size(), scene.springs);
  scene.adjacency
      = vbd::SpringAdjacency::build(scene.positions.size(), scene.springs);
  return scene;
}

cuda::VbdCudaMassSpringProblem makeProblem(
    const GridScene& scene, std::size_t iters)
{
  cuda::VbdCudaMassSpringProblem problem;
  problem.nodeCount = scene.positions.size();
  problem.stiffness = scene.stiffness;
  problem.timeStep = scene.timeStep;
  problem.iterations = iters;
  for (std::size_t i = 0; i < scene.positions.size(); ++i) {
    const Vec3& y = scene.inertialTargets[i];
    problem.positions.push_back(y.x());
    problem.positions.push_back(y.y());
    problem.positions.push_back(y.z());
    problem.inertialTargets.push_back(y.x());
    problem.inertialTargets.push_back(y.y());
    problem.inertialTargets.push_back(y.z());
  }
  problem.masses = scene.masses;
  problem.fixed = scene.fixed;
  for (const auto& s : scene.springs) {
    problem.springA.push_back(s.a);
    problem.springB.push_back(s.b);
    problem.springRest.push_back(s.restLength);
  }
  problem.incidentOffsets.push_back(0);
  for (std::size_t v = 0; v < scene.positions.size(); ++v) {
    for (const std::uint32_t s : scene.adjacency.incidentSprings[v]) {
      problem.incidentSprings.push_back(s);
    }
    problem.incidentOffsets.push_back(
        static_cast<std::uint32_t>(problem.incidentSprings.size()));
  }
  problem.colorOffsets.push_back(0);
  for (const auto& group : scene.coloring.groups) {
    for (const std::uint32_t v : group) {
      problem.colorVertices.push_back(v);
    }
    problem.colorOffsets.push_back(
        static_cast<std::uint32_t>(problem.colorVertices.size()));
  }
  return problem;
}

cuda::VbdCudaRolloutProblem makeRolloutProblem(
    const GridScene& scene, std::size_t iters, std::size_t steps)
{
  const cuda::VbdCudaMassSpringProblem single = makeProblem(scene, iters);
  cuda::VbdCudaRolloutProblem problem;
  problem.nodeCount = single.nodeCount;
  for (const auto& p : scene.positions) {
    problem.positions.push_back(p.x());
    problem.positions.push_back(p.y());
    problem.positions.push_back(p.z());
    problem.velocities.push_back(0.0);
    problem.velocities.push_back(0.0);
    problem.velocities.push_back(0.0);
  }
  problem.masses = single.masses;
  problem.fixed = single.fixed;
  problem.springA = single.springA;
  problem.springB = single.springB;
  problem.springRest = single.springRest;
  problem.incidentOffsets = single.incidentOffsets;
  problem.incidentSprings = single.incidentSprings;
  problem.colorOffsets = single.colorOffsets;
  problem.colorVertices = single.colorVertices;
  problem.gravity[0] = 0.0;
  problem.gravity[1] = -9.81;
  problem.gravity[2] = 0.0;
  problem.stiffness = scene.stiffness;
  problem.timeStep = scene.timeStep;
  problem.iterations = iters;
  problem.stepCount = steps;
  return problem;
}

} // namespace

//==============================================================================
static void BM_VbdCpuStep(benchmark::State& state)
{
  const int side = static_cast<int>(state.range(0));
  const GridScene scene = makeGrid(side);
  vbd::BlockDescentOptions options;
  options.iterations = 20;
  for (auto _ : state) {
    state.PauseTiming();
    std::vector<Vec3> positions = scene.inertialTargets;
    state.ResumeTiming();
    vbd::blockDescentMassSpring(
        positions,
        scene.masses,
        scene.fixed,
        scene.inertialTargets,
        scene.springs,
        scene.stiffness,
        scene.timeStep,
        scene.coloring,
        scene.adjacency,
        options);
    benchmark::DoNotOptimize(positions);
  }
  state.counters["vertices"] = static_cast<double>(side * side);
}
BENCHMARK(BM_VbdCpuStep)->Arg(32)->Arg(64)->Arg(128);

//==============================================================================
static void BM_VbdCudaStep(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("no CUDA device available");
    return;
  }
  const int side = static_cast<int>(state.range(0));
  const GridScene scene = makeGrid(side);
  const cuda::VbdCudaMassSpringProblem base = makeProblem(scene, 20);
  for (auto _ : state) {
    state.PauseTiming();
    cuda::VbdCudaMassSpringProblem problem = base;
    state.ResumeTiming();
    cuda::vbdStepMassSpringCuda(problem);
    benchmark::DoNotOptimize(problem.positions);
  }
  state.counters["vertices"] = static_cast<double>(side * side);
}
BENCHMARK(BM_VbdCudaStep)->Arg(32)->Arg(64)->Arg(128);

//==============================================================================
// Device-resident rollout: one upload, `steps` full steps on the GPU, one
// download. This is the steady-state GPU per-step cost without per-step
// transfer or first-call warmup. Reports per-step time via a counter.
static void BM_VbdCudaRollout(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("no CUDA device available");
    return;
  }
  const int side = static_cast<int>(state.range(0));
  const std::size_t steps = 50;
  const GridScene scene = makeGrid(side);
  const cuda::VbdCudaRolloutProblem base = makeRolloutProblem(scene, 20, steps);
  for (auto _ : state) {
    state.PauseTiming();
    cuda::VbdCudaRolloutProblem problem = base;
    state.ResumeTiming();
    cuda::vbdRolloutMassSpringCuda(problem);
    benchmark::DoNotOptimize(problem.positions);
  }
  state.counters["vertices"] = static_cast<double>(side * side);
  state.counters["steps"] = static_cast<double>(steps);
}
BENCHMARK(BM_VbdCudaRollout)->Arg(32)->Arg(64)->Arg(128);

namespace {

// A tetrahedral bar of `cubes` stacked unit cubes (six Kuhn tets each), bottom
// ring pinned, used for the tet GPU-vs-CPU comparison (the paper's domain).
struct TetBar
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

TetBar makeTetBar(int cubes)
{
  TetBar scene;
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
      scene.inertialTargets[v] += Vec3(0.02, -0.03, 0.01);
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
      corner[static_cast<std::size_t>(n)]
          = index(n & 1, (n >> 1) & 1, c + ((n >> 2) & 1));
    }
    for (const auto& t : kuhn) {
      const std::array<std::uint32_t, 4> v
          = {corner[t[0]], corner[t[1]], corner[t[2]], corner[t[3]]};
      scene.tets.push_back(
          {v,
           vbd::makeTetRestShape(
               {scene.positions[v[0]],
                scene.positions[v[1]],
                scene.positions[v[2]],
                scene.positions[v[3]]})});
    }
  }
  scene.coloring = vbd::colorTetMesh(scene.positions.size(), scene.tets);
  scene.adjacency
      = vbd::TetAdjacency::build(scene.positions.size(), scene.tets);
  return scene;
}

cuda::VbdCudaTetProblem makeTetProblem(const TetBar& scene, std::size_t iters)
{
  cuda::VbdCudaTetProblem problem;
  const std::size_t n = scene.positions.size();
  problem.nodeCount = n;
  problem.mu = scene.mu;
  problem.lambda = scene.lambda;
  problem.timeStep = scene.timeStep;
  problem.iterations = iters;
  for (std::size_t i = 0; i < n; ++i) {
    const Vec3 warm
        = scene.fixed[i] != 0u ? scene.positions[i] : scene.inertialTargets[i];
    problem.positions.push_back(warm.x());
    problem.positions.push_back(warm.y());
    problem.positions.push_back(warm.z());
    problem.inertialTargets.push_back(scene.inertialTargets[i].x());
    problem.inertialTargets.push_back(scene.inertialTargets[i].y());
    problem.inertialTargets.push_back(scene.inertialTargets[i].z());
  }
  problem.masses = scene.masses;
  problem.fixed = scene.fixed;
  for (const auto& tet : scene.tets) {
    for (int k = 0; k < 4; ++k) {
      problem.tetVertices.push_back(tet.vertices[static_cast<std::size_t>(k)]);
    }
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        problem.tetRestShapeInverse.push_back(tet.rest.restShapeInverse(r, c));
      }
    }
    problem.tetRestVolume.push_back(tet.rest.restVolume);
  }
  problem.incidentTetOffsets.push_back(0);
  for (std::size_t v = 0; v < n; ++v) {
    for (const auto& [tetIndex, local] : scene.adjacency.incidentTets[v]) {
      problem.incidentTetIndex.push_back(tetIndex);
      problem.incidentLocalVertex.push_back(local);
    }
    problem.incidentTetOffsets.push_back(
        static_cast<std::uint32_t>(problem.incidentTetIndex.size()));
  }
  problem.colorOffsets.push_back(0);
  for (const auto& group : scene.coloring.groups) {
    for (const std::uint32_t v : group) {
      problem.colorVertices.push_back(v);
    }
    problem.colorOffsets.push_back(
        static_cast<std::uint32_t>(problem.colorVertices.size()));
  }
  return problem;
}

} // namespace

//==============================================================================
static void BM_VbdCpuTetStep(benchmark::State& state)
{
  const int cubes = static_cast<int>(state.range(0));
  const TetBar scene = makeTetBar(cubes);
  vbd::BlockDescentOptions options;
  options.iterations = 20;
  for (auto _ : state) {
    state.PauseTiming();
    std::vector<Vec3> positions = scene.inertialTargets;
    state.ResumeTiming();
    vbd::blockDescentTetMesh(
        positions,
        scene.masses,
        scene.fixed,
        scene.inertialTargets,
        scene.tets,
        scene.mu,
        scene.lambda,
        scene.timeStep,
        scene.coloring,
        scene.adjacency,
        options);
    benchmark::DoNotOptimize(positions);
  }
  state.counters["vertices"] = static_cast<double>(scene.positions.size());
  state.counters["tets"] = static_cast<double>(scene.tets.size());
}
BENCHMARK(BM_VbdCpuTetStep)->Arg(32)->Arg(128)->Arg(512);

//==============================================================================
static void BM_VbdCudaTetStep(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("no CUDA device available");
    return;
  }
  const int cubes = static_cast<int>(state.range(0));
  const TetBar scene = makeTetBar(cubes);
  const cuda::VbdCudaTetProblem base = makeTetProblem(scene, 20);
  for (auto _ : state) {
    state.PauseTiming();
    cuda::VbdCudaTetProblem problem = base;
    state.ResumeTiming();
    cuda::vbdStepTetMeshCuda(problem);
    benchmark::DoNotOptimize(problem.positions);
  }
  state.counters["vertices"] = static_cast<double>(scene.positions.size());
  state.counters["tets"] = static_cast<double>(scene.tets.size());
}
BENCHMARK(BM_VbdCudaTetStep)->Arg(32)->Arg(128)->Arg(512);

namespace {

cuda::VbdCudaTetRolloutProblem makeTetRolloutProblem(
    const TetBar& scene, std::size_t iterations, std::size_t steps, bool graph)
{
  const std::size_t n = scene.positions.size();
  cuda::VbdCudaTetRolloutProblem problem;
  problem.nodeCount = n;
  problem.mu = scene.mu;
  problem.lambda = scene.lambda;
  problem.timeStep = scene.timeStep;
  problem.iterations = iterations;
  problem.stepCount = steps;
  problem.useCudaGraph = graph;
  problem.gravity[1] = -9.81;
  for (std::size_t i = 0; i < n; ++i) {
    problem.positions.push_back(scene.positions[i].x());
    problem.positions.push_back(scene.positions[i].y());
    problem.positions.push_back(scene.positions[i].z());
    problem.velocities.push_back(0.0);
    problem.velocities.push_back(0.0);
    problem.velocities.push_back(0.0);
  }
  problem.masses = scene.masses;
  problem.fixed = scene.fixed;
  for (const auto& tet : scene.tets) {
    for (int k = 0; k < 4; ++k) {
      problem.tetVertices.push_back(tet.vertices[static_cast<std::size_t>(k)]);
    }
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        problem.tetRestShapeInverse.push_back(tet.rest.restShapeInverse(r, c));
      }
    }
    problem.tetRestVolume.push_back(tet.rest.restVolume);
  }
  problem.incidentTetOffsets.push_back(0);
  for (std::size_t v = 0; v < n; ++v) {
    for (const auto& [tetIndex, local] : scene.adjacency.incidentTets[v]) {
      problem.incidentTetIndex.push_back(tetIndex);
      problem.incidentLocalVertex.push_back(local);
    }
    problem.incidentTetOffsets.push_back(
        static_cast<std::uint32_t>(problem.incidentTetIndex.size()));
  }
  problem.colorOffsets.push_back(0);
  for (const auto& group : scene.coloring.groups) {
    for (const std::uint32_t v : group) {
      problem.colorVertices.push_back(v);
    }
    problem.colorOffsets.push_back(
        static_cast<std::uint32_t>(problem.colorVertices.size()));
  }
  return problem;
}

} // namespace

//==============================================================================
// Device-resident tetrahedral rollout, with and without CUDA-graph capture, to
// show the per-launch overhead the graph removes for the many small per-color
// kernel launches.
static void BM_VbdCudaTetRollout(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("no CUDA device available");
    return;
  }
  const int cubes = static_cast<int>(state.range(0));
  const bool graph = state.range(1) != 0;
  const std::size_t steps = 50;
  const TetBar scene = makeTetBar(cubes);
  const cuda::VbdCudaTetRolloutProblem seed
      = makeTetRolloutProblem(scene, 20, steps, graph);
  for (auto _ : state) {
    state.PauseTiming();
    cuda::VbdCudaTetRolloutProblem problem = seed;
    state.ResumeTiming();
    cuda::vbdRolloutTetMeshCuda(problem);
    benchmark::DoNotOptimize(problem.positions);
  }
  state.counters["vertices"] = static_cast<double>(scene.positions.size());
  state.counters["tets"] = static_cast<double>(scene.tets.size());
  state.counters["steps"] = static_cast<double>(steps);
  state.counters["graph"] = graph ? 1.0 : 0.0;
}
BENCHMARK(BM_VbdCudaTetRollout)
    ->Args({128, 0})
    ->Args({128, 1})
    ->Args({512, 0})
    ->Args({512, 1});

BENCHMARK_MAIN();
