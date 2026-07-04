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

#include <dart/simd/simd.hpp>

#include <benchmark/benchmark.h>

#include <random>
#include <vector>

using namespace dart::simd;

namespace {

template <typename T>
aligned_vector<T> generateRandomData(
    std::size_t size, T lo, T hi, int seed = 42)
{
  std::mt19937 gen(seed);
  std::uniform_real_distribution<T> dist(lo, hi);
  aligned_vector<T> data(size);
  for (auto& val : data) {
    val = dist(gen);
  }
  return data;
}

constexpr int kMinSize = 1 << 10;
constexpr int kMaxSize = 1 << 16;

} // namespace

static void BM_Sin_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data = generateRandomData<float>(n, -10.0f, 10.0f);
  aligned_vector<float> result(n);

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto v = VecT::load(&data[i]);
      auto r = sin(v);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float));
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
BENCHMARK(BM_Sin_DART_f32)
    ->Repetitions(5)
    ->ReportAggregatesOnly(true)
    ->RangeMultiplier(4)
    ->Range(kMinSize, kMaxSize);

static void BM_Cos_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data = generateRandomData<float>(n, -10.0f, 10.0f);
  aligned_vector<float> result(n);

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto v = VecT::load(&data[i]);
      auto r = cos(v);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float));
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
BENCHMARK(BM_Cos_DART_f32)
    ->Repetitions(5)
    ->ReportAggregatesOnly(true)
    ->RangeMultiplier(4)
    ->Range(kMinSize, kMaxSize);

static void BM_SinCos_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data = generateRandomData<float>(n, -10.0f, 10.0f);
  aligned_vector<float> resultSin(n), resultCos(n);

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto v = VecT::load(&data[i]);
      auto [s, c] = sincos(v);
      s.store(&resultSin[i]);
      c.store(&resultCos[i]);
    }
    benchmark::DoNotOptimize(resultSin.data());
    benchmark::DoNotOptimize(resultCos.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float));
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
BENCHMARK(BM_SinCos_DART_f32)
    ->Repetitions(5)
    ->ReportAggregatesOnly(true)
    ->RangeMultiplier(4)
    ->Range(kMinSize, kMaxSize);

static void BM_Tan_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data = generateRandomData<float>(n, -1.5f, 1.5f);
  aligned_vector<float> result(n);

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto v = VecT::load(&data[i]);
      auto r = tan(v);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float));
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
BENCHMARK(BM_Tan_DART_f32)
    ->Repetitions(5)
    ->ReportAggregatesOnly(true)
    ->RangeMultiplier(4)
    ->Range(kMinSize, kMaxSize);

static void BM_Exp_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data = generateRandomData<float>(n, -5.0f, 5.0f);
  aligned_vector<float> result(n);

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto v = VecT::load(&data[i]);
      auto r = exp(v);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float));
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
BENCHMARK(BM_Exp_DART_f32)
    ->Repetitions(5)
    ->ReportAggregatesOnly(true)
    ->RangeMultiplier(4)
    ->Range(kMinSize, kMaxSize);

static void BM_Log_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data = generateRandomData<float>(n, 0.01f, 100.0f);
  aligned_vector<float> result(n);

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto v = VecT::load(&data[i]);
      auto r = log(v);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float));
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
BENCHMARK(BM_Log_DART_f32)
    ->Repetitions(5)
    ->ReportAggregatesOnly(true)
    ->RangeMultiplier(4)
    ->Range(kMinSize, kMaxSize);

static void BM_Pow_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto bases = generateRandomData<float>(n, 0.5f, 5.0f, 1);
  auto exponents = generateRandomData<float>(n, -2.0f, 2.0f, 2);
  aligned_vector<float> result(n);

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto b = VecT::load(&bases[i]);
      auto e = VecT::load(&exponents[i]);
      auto r = pow(b, e);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 2);
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
BENCHMARK(BM_Pow_DART_f32)
    ->Repetitions(5)
    ->ReportAggregatesOnly(true)
    ->RangeMultiplier(4)
    ->Range(kMinSize, kMaxSize);

static void BM_Asin_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data = generateRandomData<float>(n, -0.99f, 0.99f);
  aligned_vector<float> result(n);

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto v = VecT::load(&data[i]);
      auto r = asin(v);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float));
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
BENCHMARK(BM_Asin_DART_f32)
    ->Repetitions(5)
    ->ReportAggregatesOnly(true)
    ->RangeMultiplier(4)
    ->Range(kMinSize, kMaxSize);

static void BM_Atan2_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto yData = generateRandomData<float>(n, -10.0f, 10.0f, 1);
  auto xData = generateRandomData<float>(n, -10.0f, 10.0f, 2);
  aligned_vector<float> result(n);

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto y = VecT::load(&yData[i]);
      auto x = VecT::load(&xData[i]);
      auto r = atan2(y, x);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 2);
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
BENCHMARK(BM_Atan2_DART_f32)
    ->Repetitions(5)
    ->ReportAggregatesOnly(true)
    ->RangeMultiplier(4)
    ->Range(kMinSize, kMaxSize);

static void BM_Sinh_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data = generateRandomData<float>(n, -5.0f, 5.0f);
  aligned_vector<float> result(n);

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto v = VecT::load(&data[i]);
      auto r = sinh(v);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float));
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
BENCHMARK(BM_Sinh_DART_f32)
    ->Repetitions(5)
    ->ReportAggregatesOnly(true)
    ->RangeMultiplier(4)
    ->Range(kMinSize, kMaxSize);

static void BM_Tanh_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data = generateRandomData<float>(n, -5.0f, 5.0f);
  aligned_vector<float> result(n);

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto v = VecT::load(&data[i]);
      auto r = tanh(v);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float));
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
BENCHMARK(BM_Tanh_DART_f32)
    ->Repetitions(5)
    ->ReportAggregatesOnly(true)
    ->RangeMultiplier(4)
    ->Range(kMinSize, kMaxSize);

static void BM_Erf_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data = generateRandomData<float>(n, -3.0f, 3.0f);
  aligned_vector<float> result(n);

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto v = VecT::load(&data[i]);
      auto r = erf(v);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float));
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
BENCHMARK(BM_Erf_DART_f32)
    ->Repetitions(5)
    ->ReportAggregatesOnly(true)
    ->RangeMultiplier(4)
    ->Range(kMinSize, kMaxSize);

static void BM_Cbrt_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data = generateRandomData<float>(n, -100.0f, 100.0f);
  aligned_vector<float> result(n);

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto v = VecT::load(&data[i]);
      auto r = cbrt(v);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float));
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
BENCHMARK(BM_Cbrt_DART_f32)
    ->Repetitions(5)
    ->ReportAggregatesOnly(true)
    ->RangeMultiplier(4)
    ->Range(kMinSize, kMaxSize);

static void BM_Copysign_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto magnitudes = generateRandomData<float>(n, 0.0f, 100.0f, 1);
  auto signs = generateRandomData<float>(n, -1.0f, 1.0f, 2);
  aligned_vector<float> result(n);

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto m = VecT::load(&magnitudes[i]);
      auto s = VecT::load(&signs[i]);
      auto r = copysign(m, s);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 2);
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
BENCHMARK(BM_Copysign_DART_f32)
    ->Repetitions(5)
    ->ReportAggregatesOnly(true)
    ->RangeMultiplier(4)
    ->Range(kMinSize, kMaxSize);

static void BM_Sin_DART_f64(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data = generateRandomData<double>(n, -10.0, 10.0);
  aligned_vector<double> result(n);

  constexpr std::size_t W = preferred_width_v<double>;
  using VecT = Vec<double, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto v = VecT::load(&data[i]);
      auto r = sin(v);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(double));
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
BENCHMARK(BM_Sin_DART_f64)
    ->Repetitions(5)
    ->ReportAggregatesOnly(true)
    ->RangeMultiplier(4)
    ->Range(kMinSize, kMaxSize);

static void BM_Exp_DART_f64(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data = generateRandomData<double>(n, -5.0, 5.0);
  aligned_vector<double> result(n);

  constexpr std::size_t W = preferred_width_v<double>;
  using VecT = Vec<double, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto v = VecT::load(&data[i]);
      auto r = exp(v);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(double));
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
BENCHMARK(BM_Exp_DART_f64)
    ->Repetitions(5)
    ->ReportAggregatesOnly(true)
    ->RangeMultiplier(4)
    ->Range(kMinSize, kMaxSize);

static void BM_Log_DART_f64(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data = generateRandomData<double>(n, 0.01, 100.0);
  aligned_vector<double> result(n);

  constexpr std::size_t W = preferred_width_v<double>;
  using VecT = Vec<double, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto v = VecT::load(&data[i]);
      auto r = log(v);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(double));
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
BENCHMARK(BM_Log_DART_f64)
    ->Repetitions(5)
    ->ReportAggregatesOnly(true)
    ->RangeMultiplier(4)
    ->Range(kMinSize, kMaxSize);

BENCHMARK_MAIN();
