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

#include <dart/simulation/detail/deformable_contact/barrier_kernel.hpp>

#include <benchmark/benchmark.h>

namespace dc = dart::simulation::detail::deformable_contact;

namespace {

//==============================================================================
void consumeScalar(dc::BarrierScalarDerivatives& result)
{
  benchmark::DoNotOptimize(result.value);
  benchmark::DoNotOptimize(result.firstDerivative);
  benchmark::DoNotOptimize(result.secondDerivative);
  benchmark::ClobberMemory();
}

//==============================================================================
void consumePrimitive(dc::PrimitiveBarrierResult& result)
{
  benchmark::DoNotOptimize(result.value);
  benchmark::DoNotOptimize(result.gradient);
  benchmark::DoNotOptimize(result.hessian);
  benchmark::ClobberMemory();
}

} // namespace

//==============================================================================
static void BM_IpcBarrierScalarActive(benchmark::State& state)
{
  double squaredDistance = 0.25;
  double squaredActivationDistance = 1.0;
  dc::BarrierScalarDerivatives result;
  for (auto _ : state) {
    benchmark::DoNotOptimize(squaredDistance);
    benchmark::DoNotOptimize(squaredActivationDistance);
    result
        = dc::c2ClampedLogBarrier(squaredDistance, squaredActivationDistance);
    consumeScalar(result);
  }
}
BENCHMARK(BM_IpcBarrierScalarActive);

//==============================================================================
static void BM_IpcBarrierScalarInactive(benchmark::State& state)
{
  double squaredDistance = 1.25;
  double squaredActivationDistance = 1.0;
  dc::BarrierScalarDerivatives result;
  for (auto _ : state) {
    benchmark::DoNotOptimize(squaredDistance);
    benchmark::DoNotOptimize(squaredActivationDistance);
    result
        = dc::c2ClampedLogBarrier(squaredDistance, squaredActivationDistance);
    consumeScalar(result);
  }
}
BENCHMARK(BM_IpcBarrierScalarInactive);

//==============================================================================
static void BM_IpcPointTriangleBarrier(benchmark::State& state)
{
  Eigen::Vector3d p(0.2, 0.3, 0.5);
  Eigen::Vector3d a(0.0, 0.0, 0.0);
  Eigen::Vector3d b(1.0, 0.0, 0.0);
  Eigen::Vector3d c(0.0, 1.0, 0.0);
  double squaredActivationDistance = 1.0;

  dc::PrimitiveBarrierResult result;
  for (auto _ : state) {
    benchmark::DoNotOptimize(p);
    benchmark::DoNotOptimize(a);
    benchmark::DoNotOptimize(b);
    benchmark::DoNotOptimize(c);
    benchmark::DoNotOptimize(squaredActivationDistance);
    result = dc::pointTriangleBarrier(p, a, b, c, squaredActivationDistance);
    consumePrimitive(result);
  }
}
BENCHMARK(BM_IpcPointTriangleBarrier);

//==============================================================================
static void BM_IpcEdgeEdgeBarrier(benchmark::State& state)
{
  Eigen::Vector3d a(-1.0, 0.0, 0.5);
  Eigen::Vector3d b(1.0, 0.0, 0.5);
  Eigen::Vector3d c(0.0, -1.0, 0.0);
  Eigen::Vector3d d(0.0, 1.0, 0.0);
  double squaredActivationDistance = 1.0;

  dc::PrimitiveBarrierResult result;
  for (auto _ : state) {
    benchmark::DoNotOptimize(a);
    benchmark::DoNotOptimize(b);
    benchmark::DoNotOptimize(c);
    benchmark::DoNotOptimize(d);
    benchmark::DoNotOptimize(squaredActivationDistance);
    result = dc::edgeEdgeBarrier(a, b, c, d, squaredActivationDistance);
    consumePrimitive(result);
  }
}
BENCHMARK(BM_IpcEdgeEdgeBarrier);

//==============================================================================
static void BM_IpcMollifiedEdgeEdgeBarrier(benchmark::State& state)
{
  Eigen::Vector3d a(-1.0, 0.0, 0.5);
  Eigen::Vector3d b(1.0, 0.0, 0.5);
  Eigen::Vector3d c(0.0, -1.0, 0.0);
  Eigen::Vector3d d(0.0, 1.0, 0.0);
  double squaredActivationDistance = 1.0;
  double mollifierThreshold = 64.0;

  dc::PrimitiveBarrierResult result;
  for (auto _ : state) {
    benchmark::DoNotOptimize(a);
    benchmark::DoNotOptimize(b);
    benchmark::DoNotOptimize(c);
    benchmark::DoNotOptimize(d);
    benchmark::DoNotOptimize(squaredActivationDistance);
    benchmark::DoNotOptimize(mollifierThreshold);
    result = dc::mollifiedEdgeEdgeBarrier(
        a, b, c, d, squaredActivationDistance, mollifierThreshold);
    consumePrimitive(result);
  }
}
BENCHMARK(BM_IpcMollifiedEdgeEdgeBarrier);
