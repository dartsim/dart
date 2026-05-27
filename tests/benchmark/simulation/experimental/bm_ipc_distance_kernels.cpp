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
 *   INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 *   EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/simulation/experimental/detail/deformable_contact/primitive_distance.hpp>

#include <benchmark/benchmark.h>

#include <array>

namespace dc = dart::simulation::experimental::detail::deformable_contact;

namespace {

//==============================================================================
std::array<Eigen::Vector3d, 4> pointTriangleCase()
{
  return {
      Eigen::Vector3d(0.23, 0.31, 1.7),
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.2, 0.1, 0.0),
      Eigen::Vector3d(-0.1, 1.1, 0.2)};
}

//==============================================================================
std::array<Eigen::Vector3d, 4> edgeEdgeCase()
{
  return {
      Eigen::Vector3d(-0.8, -0.1, 0.0),
      Eigen::Vector3d(1.1, 0.2, 0.3),
      Eigen::Vector3d(0.2, -1.2, 1.4),
      Eigen::Vector3d(-0.1, 1.3, 1.8)};
}

//==============================================================================
std::array<Eigen::Vector3d, 4> nearlyParallelEdgeEdgeCase()
{
  return {
      Eigen::Vector3d(-1.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(-1.0, 1e-4, 0.0),
      Eigen::Vector3d(1.0, -1e-4, 0.0)};
}

} // namespace

//==============================================================================
static void BM_IpcPointTriangleDistanceValue(benchmark::State& state)
{
  const auto [p, a, b, c] = pointTriangleCase();
  for (auto _ : state) {
    benchmark::DoNotOptimize(
        dc::pointTriangleSquaredDistance(p, a, b, c).squaredDistance);
  }
}
BENCHMARK(BM_IpcPointTriangleDistanceValue);

//==============================================================================
static void BM_IpcPointTriangleDistanceGradient(benchmark::State& state)
{
  const auto [p, a, b, c] = pointTriangleCase();
  for (auto _ : state) {
    benchmark::DoNotOptimize(
        dc::pointTriangleSquaredDistanceGradient(p, a, b, c));
  }
}
BENCHMARK(BM_IpcPointTriangleDistanceGradient);

//==============================================================================
static void BM_IpcPointTriangleDistanceHessian(benchmark::State& state)
{
  const auto [p, a, b, c] = pointTriangleCase();
  for (auto _ : state) {
    benchmark::DoNotOptimize(
        dc::pointTriangleSquaredDistanceHessian(p, a, b, c));
  }
}
BENCHMARK(BM_IpcPointTriangleDistanceHessian);

//==============================================================================
static void BM_IpcEdgeEdgeDistanceValue(benchmark::State& state)
{
  const auto [a, b, c, d] = edgeEdgeCase();
  for (auto _ : state) {
    benchmark::DoNotOptimize(
        dc::edgeEdgeSquaredDistance(a, b, c, d).squaredDistance);
  }
}
BENCHMARK(BM_IpcEdgeEdgeDistanceValue);

//==============================================================================
static void BM_IpcEdgeEdgeDistanceGradient(benchmark::State& state)
{
  const auto [a, b, c, d] = edgeEdgeCase();
  for (auto _ : state) {
    benchmark::DoNotOptimize(dc::edgeEdgeSquaredDistanceGradient(a, b, c, d));
  }
}
BENCHMARK(BM_IpcEdgeEdgeDistanceGradient);

//==============================================================================
static void BM_IpcEdgeEdgeDistanceHessian(benchmark::State& state)
{
  const auto [a, b, c, d] = edgeEdgeCase();
  for (auto _ : state) {
    benchmark::DoNotOptimize(dc::edgeEdgeSquaredDistanceHessian(a, b, c, d));
  }
}
BENCHMARK(BM_IpcEdgeEdgeDistanceHessian);

//==============================================================================
static void BM_IpcEdgeEdgeMollifier(benchmark::State& state)
{
  const auto [a, b, c, d] = nearlyParallelEdgeEdgeCase();
  constexpr double threshold = 0.016;
  for (auto _ : state) {
    benchmark::DoNotOptimize(dc::edgeEdgeMollifier(a, b, c, d, threshold));
  }
}
BENCHMARK(BM_IpcEdgeEdgeMollifier);

//==============================================================================
static void BM_IpcEdgeEdgeMollifierGradient(benchmark::State& state)
{
  const auto [a, b, c, d] = nearlyParallelEdgeEdgeCase();
  constexpr double threshold = 0.016;
  for (auto _ : state) {
    benchmark::DoNotOptimize(
        dc::edgeEdgeMollifierGradient(a, b, c, d, threshold));
  }
}
BENCHMARK(BM_IpcEdgeEdgeMollifierGradient);

//==============================================================================
static void BM_IpcEdgeEdgeMollifierHessian(benchmark::State& state)
{
  const auto [a, b, c, d] = nearlyParallelEdgeEdgeCase();
  constexpr double threshold = 0.016;
  for (auto _ : state) {
    benchmark::DoNotOptimize(
        dc::edgeEdgeMollifierHessian(a, b, c, d, threshold));
  }
}
BENCHMARK(BM_IpcEdgeEdgeMollifierHessian);
