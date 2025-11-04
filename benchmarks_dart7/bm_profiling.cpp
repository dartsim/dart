/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <benchmark/benchmark.h>
#include <dart7/common/profiling.hpp>

#include <cmath>

using namespace dart7::common;

// Benchmark ScopedTimer overhead
static void BM_ScopedTimerOverhead(benchmark::State& state)
{
  ProfileStats::reset();

  for (auto _ : state) {
    DART7_PROFILE_SCOPE("test");
  }
}
BENCHMARK(BM_ScopedTimerOverhead);

// Benchmark profiling with actual work
static void BM_ProfilingWithWork(benchmark::State& state)
{
  ProfileStats::reset();

  for (auto _ : state) {
    DART7_PROFILE_SCOPE("computation");
    volatile double result = 0.0;
    for (int i = 0; i < state.range(0); ++i) {
      result += std::sin(i * 0.001);
    }
  }
}
BENCHMARK(BM_ProfilingWithWork)->Range(8, 8 << 10);

// Benchmark manual profiling
static void BM_ManualProfiling(benchmark::State& state)
{
  ProfileStats::reset();

  for (auto _ : state) {
    DART7_PROFILE_BEGIN(manual);
    volatile double result = 0.0;
    for (int i = 0; i < 100; ++i) {
      result += i;
    }
    DART7_PROFILE_END(manual);
  }
}
BENCHMARK(BM_ManualProfiling);

BENCHMARK_MAIN();
