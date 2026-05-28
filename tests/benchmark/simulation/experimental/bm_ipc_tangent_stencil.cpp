/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary form, with or
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

#include <dart/simulation/experimental/detail/deformable_contact/tangent_stencil.hpp>

#include <benchmark/benchmark.h>

#include <array>
#include <vector>

#include <cstddef>

namespace dc = dart::simulation::experimental::detail::deformable_contact;

namespace {

//==============================================================================
template <typename Stencil>
void consumeStencil(Stencil& stencil)
{
  benchmark::DoNotOptimize(stencil.basis);
  benchmark::DoNotOptimize(stencil.projection);
  benchmark::DoNotOptimize(stencil.metric);
  benchmark::ClobberMemory();
}

//==============================================================================
std::vector<std::array<Eigen::Vector3d, 4>> makeStencilWorkload(
    const std::size_t count)
{
  std::vector<std::array<Eigen::Vector3d, 4>> workload;
  workload.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const double offset = static_cast<double>(i % 31) * 0.001;
    workload.push_back(
        {Eigen::Vector3d(0.5 + offset, 1.0, 0.25),
         Eigen::Vector3d(0.0, 0.0, 0.0),
         Eigen::Vector3d(2.0, 0.0, 0.0),
         Eigen::Vector3d(0.0, 4.0, 0.0)});
  }
  return workload;
}

} // namespace

//==============================================================================
static void BM_IpcPointTriangleTangentStencil(benchmark::State& state)
{
  Eigen::Vector3d p(0.5, 1.0, 0.25);
  Eigen::Vector3d a(0.0, 0.0, 0.0);
  Eigen::Vector3d b(2.0, 0.0, 0.0);
  Eigen::Vector3d c(0.0, 4.0, 0.0);

  dc::PointTriangleTangentStencil stencil;
  for (auto _ : state) {
    benchmark::DoNotOptimize(p);
    benchmark::DoNotOptimize(a);
    benchmark::DoNotOptimize(b);
    benchmark::DoNotOptimize(c);
    stencil = dc::pointTriangleTangentStencil(p, a, b, c);
    consumeStencil(stencil);
  }
}
BENCHMARK(BM_IpcPointTriangleTangentStencil);

//==============================================================================
static void BM_IpcEdgeEdgeTangentStencil(benchmark::State& state)
{
  Eigen::Vector3d a(0.0, 0.0, 0.0);
  Eigen::Vector3d b(2.0, 0.0, 0.0);
  Eigen::Vector3d c(0.5, -1.0, 1.0);
  Eigen::Vector3d d(0.5, 3.0, 1.0);

  dc::EdgeEdgeTangentStencil stencil;
  for (auto _ : state) {
    benchmark::DoNotOptimize(a);
    benchmark::DoNotOptimize(b);
    benchmark::DoNotOptimize(c);
    benchmark::DoNotOptimize(d);
    stencil = dc::edgeEdgeTangentStencil(a, b, c, d);
    consumeStencil(stencil);
  }
}
BENCHMARK(BM_IpcEdgeEdgeTangentStencil);

//==============================================================================
static void BM_IpcPointEdgeTangentStencil(benchmark::State& state)
{
  Eigen::Vector3d p(0.5, 1.0, 0.0);
  Eigen::Vector3d a(0.0, 0.0, 0.0);
  Eigen::Vector3d b(2.0, 0.0, 0.0);

  dc::PointEdgeTangentStencil stencil;
  for (auto _ : state) {
    benchmark::DoNotOptimize(p);
    benchmark::DoNotOptimize(a);
    benchmark::DoNotOptimize(b);
    stencil = dc::pointEdgeTangentStencil(p, a, b);
    consumeStencil(stencil);
  }
}
BENCHMARK(BM_IpcPointEdgeTangentStencil);

//==============================================================================
static void BM_IpcPointPointTangentStencil(benchmark::State& state)
{
  Eigen::Vector3d a(0.0, 0.0, 0.0);
  Eigen::Vector3d b(0.0, 0.0, 2.0);

  dc::PointPointTangentStencil stencil;
  for (auto _ : state) {
    benchmark::DoNotOptimize(a);
    benchmark::DoNotOptimize(b);
    stencil = dc::pointPointTangentStencil(a, b);
    consumeStencil(stencil);
  }
}
BENCHMARK(BM_IpcPointPointTangentStencil);

//==============================================================================
static void BM_IpcPointTriangleTangentStencilBatch(benchmark::State& state)
{
  const std::size_t count = static_cast<std::size_t>(state.range(0));
  const auto workload = makeStencilWorkload(count);

  double checksum = 0.0;
  for (auto _ : state) {
    for (const auto& points : workload) {
      auto stencil = dc::pointTriangleTangentStencil(
          points[0], points[1], points[2], points[3]);
      checksum += stencil.projection(0, 0);
      benchmark::DoNotOptimize(stencil);
    }
  }

  benchmark::DoNotOptimize(checksum);
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(count));
}
BENCHMARK(BM_IpcPointTriangleTangentStencilBatch)->Arg(64)->Arg(256);
