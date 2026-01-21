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

constexpr std::size_t kArraySize = 4096;

template <typename T>
aligned_vector<T> generateRandomData(std::size_t size, int seed = 42)
{
  std::mt19937 gen(seed);
  std::uniform_real_distribution<T> dist(0.1, 100.0);
  aligned_vector<T> data(size);
  for (auto& val : data) {
    val = dist(gen);
  }
  return data;
}

} // namespace

static void BM_Vec4f_Add(benchmark::State& state)
{
  auto data_a = generateRandomData<float>(kArraySize, 1);
  auto data_b = generateRandomData<float>(kArraySize, 2);
  aligned_vector<float> result(kArraySize);

  for (auto _ : state) {
    for (std::size_t i = 0; i < kArraySize; i += 4) {
      auto a = Vec<float, 4>::load(&data_a[i]);
      auto b = Vec<float, 4>::load(&data_b[i]);
      auto c = a + b;
      c.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * kArraySize * sizeof(float));
}
BENCHMARK(BM_Vec4f_Add);

static void BM_Vec4f_Mul(benchmark::State& state)
{
  auto data_a = generateRandomData<float>(kArraySize, 1);
  auto data_b = generateRandomData<float>(kArraySize, 2);
  aligned_vector<float> result(kArraySize);

  for (auto _ : state) {
    for (std::size_t i = 0; i < kArraySize; i += 4) {
      auto a = Vec<float, 4>::load(&data_a[i]);
      auto b = Vec<float, 4>::load(&data_b[i]);
      auto c = a * b;
      c.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * kArraySize * sizeof(float));
}
BENCHMARK(BM_Vec4f_Mul);

static void BM_Vec4f_Div(benchmark::State& state)
{
  auto data_a = generateRandomData<float>(kArraySize, 1);
  auto data_b = generateRandomData<float>(kArraySize, 2);
  aligned_vector<float> result(kArraySize);

  for (auto _ : state) {
    for (std::size_t i = 0; i < kArraySize; i += 4) {
      auto a = Vec<float, 4>::load(&data_a[i]);
      auto b = Vec<float, 4>::load(&data_b[i]);
      auto c = a / b;
      c.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * kArraySize * sizeof(float));
}
BENCHMARK(BM_Vec4f_Div);

static void BM_Vec4f_Sqrt(benchmark::State& state)
{
  auto data = generateRandomData<float>(kArraySize, 1);
  aligned_vector<float> result(kArraySize);

  for (auto _ : state) {
    for (std::size_t i = 0; i < kArraySize; i += 4) {
      auto v = Vec<float, 4>::load(&data[i]);
      auto r = sqrt(v);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * kArraySize * sizeof(float));
}
BENCHMARK(BM_Vec4f_Sqrt);

static void BM_Vec4f_Rsqrt(benchmark::State& state)
{
  auto data = generateRandomData<float>(kArraySize, 1);
  aligned_vector<float> result(kArraySize);

  for (auto _ : state) {
    for (std::size_t i = 0; i < kArraySize; i += 4) {
      auto v = Vec<float, 4>::load(&data[i]);
      auto r = rsqrt(v);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * kArraySize * sizeof(float));
}
BENCHMARK(BM_Vec4f_Rsqrt);

static void BM_Vec4f_Fmadd(benchmark::State& state)
{
  auto data_a = generateRandomData<float>(kArraySize, 1);
  auto data_b = generateRandomData<float>(kArraySize, 2);
  auto data_c = generateRandomData<float>(kArraySize, 3);
  aligned_vector<float> result(kArraySize);

  for (auto _ : state) {
    for (std::size_t i = 0; i < kArraySize; i += 4) {
      auto a = Vec<float, 4>::load(&data_a[i]);
      auto b = Vec<float, 4>::load(&data_b[i]);
      auto c = Vec<float, 4>::load(&data_c[i]);
      auto r = fmadd(a, b, c);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * kArraySize * sizeof(float));
}
BENCHMARK(BM_Vec4f_Fmadd);

static void BM_Vec4f_Min(benchmark::State& state)
{
  auto data_a = generateRandomData<float>(kArraySize, 1);
  auto data_b = generateRandomData<float>(kArraySize, 2);
  aligned_vector<float> result(kArraySize);

  for (auto _ : state) {
    for (std::size_t i = 0; i < kArraySize; i += 4) {
      auto a = Vec<float, 4>::load(&data_a[i]);
      auto b = Vec<float, 4>::load(&data_b[i]);
      auto r = min(a, b);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * kArraySize * sizeof(float));
}
BENCHMARK(BM_Vec4f_Min);

static void BM_Vec4f_Hsum(benchmark::State& state)
{
  auto data = generateRandomData<float>(kArraySize, 1);
  float result = 0;

  for (auto _ : state) {
    for (std::size_t i = 0; i < kArraySize; i += 4) {
      auto v = Vec<float, 4>::load(&data[i]);
      result += hsum(v);
    }
    benchmark::DoNotOptimize(result);
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * kArraySize * sizeof(float));
}
BENCHMARK(BM_Vec4f_Hsum);

static void BM_Vec4f_Select(benchmark::State& state)
{
  auto data_a = generateRandomData<float>(kArraySize, 1);
  auto data_b = generateRandomData<float>(kArraySize, 2);
  aligned_vector<float> result(kArraySize);

  for (auto _ : state) {
    for (std::size_t i = 0; i < kArraySize; i += 4) {
      auto a = Vec<float, 4>::load(&data_a[i]);
      auto b = Vec<float, 4>::load(&data_b[i]);
      auto mask = a < b;
      auto r = select(mask, a, b);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * kArraySize * sizeof(float));
}
BENCHMARK(BM_Vec4f_Select);

static void BM_Vec2d_Add(benchmark::State& state)
{
  auto data_a = generateRandomData<double>(kArraySize, 1);
  auto data_b = generateRandomData<double>(kArraySize, 2);
  aligned_vector<double> result(kArraySize);

  for (auto _ : state) {
    for (std::size_t i = 0; i < kArraySize; i += 2) {
      auto a = Vec<double, 2>::load(&data_a[i]);
      auto b = Vec<double, 2>::load(&data_b[i]);
      auto c = a + b;
      c.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * kArraySize * sizeof(double));
}
BENCHMARK(BM_Vec2d_Add);

static void BM_Vec2d_Sqrt(benchmark::State& state)
{
  auto data = generateRandomData<double>(kArraySize, 1);
  aligned_vector<double> result(kArraySize);

  for (auto _ : state) {
    for (std::size_t i = 0; i < kArraySize; i += 2) {
      auto v = Vec<double, 2>::load(&data[i]);
      auto r = sqrt(v);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * kArraySize * sizeof(double));
}
BENCHMARK(BM_Vec2d_Sqrt);

static void BM_Vec4i_Add(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_int_distribution<std::int32_t> dist(1, 1000);
  aligned_vector<std::int32_t> data_a(kArraySize);
  aligned_vector<std::int32_t> data_b(kArraySize);
  aligned_vector<std::int32_t> result(kArraySize);

  for (auto& val : data_a)
    val = dist(gen);
  for (auto& val : data_b)
    val = dist(gen);

  for (auto _ : state) {
    for (std::size_t i = 0; i < kArraySize; i += 4) {
      auto a = Vec<std::int32_t, 4>::load(&data_a[i]);
      auto b = Vec<std::int32_t, 4>::load(&data_b[i]);
      auto c = a + b;
      c.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * kArraySize
      * sizeof(std::int32_t));
}
BENCHMARK(BM_Vec4i_Add);

static void BM_Vec4i_Mul(benchmark::State& state)
{
  std::mt19937 gen(42);
  std::uniform_int_distribution<std::int32_t> dist(1, 100);
  aligned_vector<std::int32_t> data_a(kArraySize);
  aligned_vector<std::int32_t> data_b(kArraySize);
  aligned_vector<std::int32_t> result(kArraySize);

  for (auto& val : data_a)
    val = dist(gen);
  for (auto& val : data_b)
    val = dist(gen);

  for (auto _ : state) {
    for (std::size_t i = 0; i < kArraySize; i += 4) {
      auto a = Vec<std::int32_t, 4>::load(&data_a[i]);
      auto b = Vec<std::int32_t, 4>::load(&data_b[i]);
      auto c = a * b;
      c.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * kArraySize
      * sizeof(std::int32_t));
}
BENCHMARK(BM_Vec4i_Mul);

static void BM_DotProduct_Vec4f(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data_a = generateRandomData<float>(n, 1);
  auto data_b = generateRandomData<float>(n, 2);

  for (auto _ : state) {
    float result = 0;
    for (std::size_t i = 0; i < n; i += 4) {
      auto a = Vec<float, 4>::load(&data_a[i]);
      auto b = Vec<float, 4>::load(&data_b[i]);
      result += hsum(a * b);
    }
    benchmark::DoNotOptimize(result);
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 2);
}
BENCHMARK(BM_DotProduct_Vec4f)->RangeMultiplier(4)->Range(64, 16384);

static void BM_DotProduct_Scalar(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data_a = generateRandomData<float>(n, 1);
  auto data_b = generateRandomData<float>(n, 2);

  for (auto _ : state) {
    float result = 0;
    for (std::size_t i = 0; i < n; ++i) {
      result += data_a[i] * data_b[i];
    }
    benchmark::DoNotOptimize(result);
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 2);
}
BENCHMARK(BM_DotProduct_Scalar)->RangeMultiplier(4)->Range(64, 16384);

static void BM_VectorNormalize_Vec4f(benchmark::State& state)
{
  auto data = generateRandomData<float>(kArraySize, 1);
  aligned_vector<float> result(kArraySize);

  for (auto _ : state) {
    for (std::size_t i = 0; i < kArraySize; i += 4) {
      auto v = Vec<float, 4>::load(&data[i]);
      auto len_sq = hsum(v * v);
      auto inv_len = 1.0f / std::sqrt(len_sq);
      auto normalized = v * Vec<float, 4>::broadcast(inv_len);
      normalized.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * kArraySize * sizeof(float));
}
BENCHMARK(BM_VectorNormalize_Vec4f);

static void BM_VectorNormalize_Rsqrt(benchmark::State& state)
{
  auto data = generateRandomData<float>(kArraySize, 1);
  aligned_vector<float> result(kArraySize);

  for (auto _ : state) {
    for (std::size_t i = 0; i < kArraySize; i += 4) {
      auto v = Vec<float, 4>::load(&data[i]);
      auto len_sq = hsum(v * v);
      auto inv_len_vec = rsqrt(Vec<float, 4>::broadcast(len_sq));
      auto normalized = v * Vec<float, 4>::broadcast(inv_len_vec[0]);
      normalized.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * kArraySize * sizeof(float));
}
BENCHMARK(BM_VectorNormalize_Rsqrt);

BENCHMARK_MAIN();
