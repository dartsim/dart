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

BENCHMARK_MAIN();
