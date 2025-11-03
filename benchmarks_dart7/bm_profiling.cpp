/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
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
