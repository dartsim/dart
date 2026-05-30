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

#include <dart/simulation/experimental/detail/deformable_vbd/vertex_block_kernel.hpp>

#include <Eigen/Core>
#include <benchmark/benchmark.h>

#include <vector>

namespace vbd = dart::simulation::experimental::detail::deformable_vbd;

namespace {

//==============================================================================
void consume(vbd::VertexBlock& block)
{
  benchmark::DoNotOptimize(block.force);
  benchmark::DoNotOptimize(block.hessian);
  benchmark::ClobberMemory();
}

//==============================================================================
// Deterministic ring of neighbor positions around a central vertex, used to
// model a vertex with `count` incident springs.
std::vector<Eigen::Vector3d> makeNeighbors(int count)
{
  std::vector<Eigen::Vector3d> neighbors;
  neighbors.reserve(static_cast<std::size_t>(count));
  for (int k = 0; k < count; ++k) {
    const double angle = 0.37 * static_cast<double>(k);
    neighbors.emplace_back(
        1.3 * std::cos(angle),
        1.3 * std::sin(angle),
        0.05 * static_cast<double>(k));
  }
  return neighbors;
}

} // namespace

//==============================================================================
static void BM_VbdInertiaTerm(benchmark::State& state)
{
  const Eigen::Vector3d position(0.3, -0.7, 1.2);
  const Eigen::Vector3d target(0.1, -0.5, 1.0);
  for (auto _ : state) {
    vbd::VertexBlock block;
    vbd::addInertiaTerm(block, 1.5, 0.01, position, target);
    consume(block);
  }
}
BENCHMARK(BM_VbdInertiaTerm);

//==============================================================================
static void BM_VbdSpringTerm(benchmark::State& state)
{
  const Eigen::Vector3d self(0.0, 0.0, 0.0);
  const Eigen::Vector3d other(1.6, 0.2, -0.1);
  for (auto _ : state) {
    vbd::VertexBlock block;
    vbd::addSpringTerm(block, 100.0, 1.0, self, other, /*clampToPsd=*/true);
    consume(block);
  }
}
BENCHMARK(BM_VbdSpringTerm);

//==============================================================================
static void BM_VbdSolveVertexBlock(benchmark::State& state)
{
  vbd::VertexBlock block;
  block.hessian << 4.0, 1.0, 0.5, 1.0, 3.0, 0.2, 0.5, 0.2, 5.0;
  block.force = Eigen::Vector3d(1.0, -2.0, 0.5);
  for (auto _ : state) {
    benchmark::DoNotOptimize(block.force);
    Eigen::Vector3d delta = vbd::solveVertexBlock(block);
    benchmark::DoNotOptimize(delta);
  }
}
BENCHMARK(BM_VbdSolveVertexBlock);

//==============================================================================
// Full per-vertex block: assemble inertia + `count` incident springs, then take
// one block Newton step. This is the cost of one vertex update in a sweep.
static void BM_VbdAssembleAndSolve(benchmark::State& state)
{
  const int count = static_cast<int>(state.range(0));
  const std::vector<Eigen::Vector3d> neighbors = makeNeighbors(count);
  const Eigen::Vector3d self(0.2, 0.1, -0.05);
  const Eigen::Vector3d target(0.0, 0.0, 0.0);
  for (auto _ : state) {
    vbd::VertexBlock block;
    vbd::addInertiaTerm(block, 1.0, 0.01, self, target);
    for (const auto& neighbor : neighbors) {
      vbd::addSpringTerm(
          block, 1.0e4, 1.0, self, neighbor, /*clampToPsd=*/true);
    }
    Eigen::Vector3d delta = vbd::solveVertexBlock(block);
    benchmark::DoNotOptimize(delta);
    consume(block);
  }
}
BENCHMARK(BM_VbdAssembleAndSolve)->Arg(4)->Arg(8)->Arg(16);

BENCHMARK_MAIN();
