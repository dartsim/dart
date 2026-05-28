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

// First performance scaffold for the experimental rigid IPC solver
// (PLAN-082 Workstream 7). These micro-benchmarks measure the cost of the
// core hot path: per-primitive reduced-coordinate barrier derivatives, the
// scene-level sparse assembly, the projected-Newton barrier solve, and the
// conservative CCD line-search bound.
//
// They are a DART-internal baseline harness, not a parity claim. Comparison
// against the current DART rigid contact path, the audited reference
// implementation, and the paper scene families is gated on completing the
// algorithm's correctness (rigorous interval CCD, corpus parity, production
// convergence). See docs/dev_tasks/rigid_ipc_solver/benchmarks.md.

#include <dart/simulation/experimental/detail/rigid_ipc_barrier.hpp>

#include <Eigen/Core>
#include <benchmark/benchmark.h>

#include <array>
#include <vector>

namespace sxdetail = dart::simulation::experimental::detail;

namespace {

void consumeReduced(sxdetail::RigidIpcReducedBarrierResult& result)
{
  benchmark::DoNotOptimize(result.value);
  benchmark::DoNotOptimize(result.gradient);
  benchmark::DoNotOptimize(result.hessian);
  benchmark::ClobberMemory();
}

sxdetail::RigidIpcBarrierOptions activeBarrierOptions()
{
  sxdetail::RigidIpcBarrierOptions options;
  options.squaredActivationDistance = 1.0;
  options.stiffness = 1.0;
  options.projectReducedHessianToPsd = true;
  return options;
}

// A unit right-triangle surface placed at height z, matching the rigid IPC
// barrier unit tests.
sxdetail::RigidIpcBarrierSurface makeTriangleSurface(double z, bool dynamic)
{
  sxdetail::RigidIpcBarrierSurface surface;
  surface.dynamic = dynamic;
  surface.pose.position = Eigen::Vector3d(0.0, 0.0, z);
  surface.vertices
      = {Eigen::Vector3d(0.0, 0.0, 0.0),
         Eigen::Vector3d(1.0, 0.0, 0.0),
         Eigen::Vector3d(0.0, 1.0, 0.0)};
  surface.triangles = {Eigen::Vector3i(0, 1, 2)};
  return surface;
}

} // namespace

//==============================================================================
static void BM_RigidIpcReducedBarrier_PointTriangle(benchmark::State& state)
{
  const Eigen::Vector3d point(0.25, 0.25, 0.5);
  const sxdetail::RigidIpcPose pointPose;
  const Eigen::Vector3d a(0.0, 0.0, 0.0);
  const Eigen::Vector3d b(1.0, 0.0, 0.0);
  const Eigen::Vector3d c(0.0, 1.0, 0.0);
  const sxdetail::RigidIpcPose trianglePose;
  const auto options = activeBarrierOptions();

  for (auto _ : state) {
    auto result = sxdetail::rigidIpcPointTriangleReducedBarrier(
        point, pointPose, a, b, c, trianglePose, options);
    consumeReduced(result);
  }
}
BENCHMARK(BM_RigidIpcReducedBarrier_PointTriangle);

//==============================================================================
static void BM_RigidIpcReducedBarrier_PointEdge(benchmark::State& state)
{
  const Eigen::Vector3d point(0.5, 0.0, 0.5);
  const sxdetail::RigidIpcPose pointPose;
  const Eigen::Vector3d edgeA(0.0, 0.0, 0.0);
  const Eigen::Vector3d edgeB(1.0, 0.0, 0.0);
  const sxdetail::RigidIpcPose edgePose;
  const auto options = activeBarrierOptions();

  for (auto _ : state) {
    auto result = sxdetail::rigidIpcPointEdgeReducedBarrier(
        point, pointPose, edgeA, edgeB, edgePose, options);
    consumeReduced(result);
  }
}
BENCHMARK(BM_RigidIpcReducedBarrier_PointEdge);

//==============================================================================
static void BM_RigidIpcReducedBarrier_EdgeEdge(benchmark::State& state)
{
  const Eigen::Vector3d a0(0.0, 0.0, 0.0);
  const Eigen::Vector3d a1(1.0, 0.0, 0.0);
  const sxdetail::RigidIpcPose poseA;
  const Eigen::Vector3d b0(0.5, -0.5, 0.5);
  const Eigen::Vector3d b1(0.5, 0.5, 0.5);
  const sxdetail::RigidIpcPose poseB;
  const auto options = activeBarrierOptions();

  for (auto _ : state) {
    auto result = sxdetail::rigidIpcEdgeEdgeReducedBarrier(
        a0, a1, poseA, b0, b1, poseB, options);
    consumeReduced(result);
  }
}
BENCHMARK(BM_RigidIpcReducedBarrier_EdgeEdge);

//==============================================================================
static void BM_RigidIpcReducedBarrier_PointPoint(benchmark::State& state)
{
  const Eigen::Vector3d pointA(0.0, 0.0, 0.0);
  const sxdetail::RigidIpcPose poseA;
  const Eigen::Vector3d pointB(0.0, 0.0, 0.5);
  const sxdetail::RigidIpcPose poseB;
  const auto options = activeBarrierOptions();

  for (auto _ : state) {
    auto result = sxdetail::rigidIpcPointPointReducedBarrier(
        pointA, poseA, pointB, poseB, options);
    consumeReduced(result);
  }
}
BENCHMARK(BM_RigidIpcReducedBarrier_PointPoint);

//==============================================================================
// Scene-level sparse assembly over a stack of dynamic triangles above one
// static triangle. The body count is the benchmark argument so assembly cost
// can be tracked as the active contact set grows.
static void BM_RigidIpcAssembleBarrierSystem(benchmark::State& state)
{
  const auto bodyCount = static_cast<std::size_t>(state.range(0));
  std::vector<sxdetail::RigidIpcBarrierSurface> surfaces;
  surfaces.reserve(bodyCount + 1);
  surfaces.push_back(makeTriangleSurface(0.0, /*dynamic=*/false));
  for (std::size_t i = 0; i < bodyCount; ++i) {
    surfaces.push_back(makeTriangleSurface(
        0.4 * static_cast<double>(i + 1), /*dynamic=*/true));
  }
  const auto options = activeBarrierOptions();

  for (auto _ : state) {
    auto assembly = sxdetail::assembleRigidIpcBarrierSystem(surfaces, options);
    benchmark::DoNotOptimize(assembly.value);
    benchmark::DoNotOptimize(assembly.gradient);
    benchmark::DoNotOptimize(assembly.hessian);
    benchmark::ClobberMemory();
  }
  state.SetComplexityN(state.range(0));
}
BENCHMARK(BM_RigidIpcAssembleBarrierSystem)
    ->RangeMultiplier(2)
    ->Range(1, 32)
    ->Complexity();

//==============================================================================
// Full projected-Newton barrier solve on a two-body point contact, the core
// runtime hot path of the opt-in rigid IPC stage.
static void BM_RigidIpcProjectedNewtonSolve_TwoBody(benchmark::State& state)
{
  sxdetail::RigidIpcBarrierSurface dynamicBody;
  dynamicBody.dynamic = true;
  dynamicBody.pose.position = Eigen::Vector3d(0.0, 0.0, 0.25);
  dynamicBody.vertices.push_back(Eigen::Vector3d::Zero());

  sxdetail::RigidIpcBarrierSurface staticBody;
  staticBody.dynamic = false;
  staticBody.vertices.push_back(Eigen::Vector3d::Zero());

  const std::array<sxdetail::RigidIpcBarrierSurface, 2> surfaces{
      dynamicBody, staticBody};

  sxdetail::RigidIpcProjectedNewtonSolveOptions options;
  options.barrier.squaredActivationDistance = 1.0;
  options.newton.maxStepNorm = 0.05;
  options.maxIterations = static_cast<std::size_t>(state.range(0));

  for (auto _ : state) {
    auto result = sxdetail::solveRigidIpcProjectedNewtonBarrierSystem(
        surfaces, options);
    benchmark::DoNotOptimize(result.surfaces);
    benchmark::DoNotOptimize(result.stats.finalValue);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_RigidIpcProjectedNewtonSolve_TwoBody)->Arg(4)->Arg(8)->Arg(16);

//==============================================================================
// Conservative curved-CCD line-search bound between a static and a dynamic
// triangle crossing it. CCD typically dominates IPC step cost.
static void BM_RigidIpcLineSearchStepBound(benchmark::State& state)
{
  const std::array<sxdetail::RigidIpcBarrierSurface, 2> startSurfaces{
      makeTriangleSurface(0.5, /*dynamic=*/true),
      makeTriangleSurface(0.0, /*dynamic=*/false)};
  std::array<sxdetail::RigidIpcBarrierSurface, 2> endSurfaces = startSurfaces;
  endSurfaces[0].pose.position.z() = -0.1;

  sxdetail::RigidIpcLineSearchOptions options;
  options.minSeparation = 0.0;

  for (auto _ : state) {
    auto result = sxdetail::computeRigidIpcLineSearchStepBound(
        startSurfaces, endSurfaces, options);
    benchmark::DoNotOptimize(result.stepBound);
    benchmark::DoNotOptimize(result.limited);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_RigidIpcLineSearchStepBound);
