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

#include <drjit/packet.h>

#include <benchmark/benchmark.h>

#include <random>
#include <vector>

namespace ds = dart::simd;

namespace {

template <typename T, std::size_t Alignment = 32>
std::vector<T> generateAlignedData(std::size_t size, int seed = 42)
{
  std::mt19937 gen(seed);
  std::uniform_real_distribution<T> dist(0.1, 100.0);
  std::vector<T> data(size);
  for (auto& val : data) {
    val = dist(gen);
  }
  return data;
}

} // namespace

static void BM_DartSimd_Add_Float4(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto a = generateAlignedData<float>(n, 1);
  auto b = generateAlignedData<float>(n, 2);
  std::vector<float> result(n);

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += 4) {
      auto va = ds::Vec<float, 4>::load(&a[i]);
      auto vb = ds::Vec<float, 4>::load(&b[i]);
      auto vc = va + vb;
      vc.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 2);
}
BENCHMARK(BM_DartSimd_Add_Float4)->RangeMultiplier(4)->Range(64, 16384);

static void BM_DrJit_Add_Float4(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto a = generateAlignedData<float>(n, 1);
  auto b = generateAlignedData<float>(n, 2);
  std::vector<float> result(n);

  using Packet = drjit::Packet<float, 4>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += 4) {
      auto va = drjit::load<Packet>(&a[i]);
      auto vb = drjit::load<Packet>(&b[i]);
      auto vc = va + vb;
      drjit::store(&result[i], vc);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 2);
}
BENCHMARK(BM_DrJit_Add_Float4)->RangeMultiplier(4)->Range(64, 16384);

static void BM_DartSimd_Mul_Float4(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto a = generateAlignedData<float>(n, 1);
  auto b = generateAlignedData<float>(n, 2);
  std::vector<float> result(n);

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += 4) {
      auto va = ds::Vec<float, 4>::load(&a[i]);
      auto vb = ds::Vec<float, 4>::load(&b[i]);
      auto vc = va * vb;
      vc.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 2);
}
BENCHMARK(BM_DartSimd_Mul_Float4)->RangeMultiplier(4)->Range(64, 16384);

static void BM_DrJit_Mul_Float4(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto a = generateAlignedData<float>(n, 1);
  auto b = generateAlignedData<float>(n, 2);
  std::vector<float> result(n);

  using Packet = drjit::Packet<float, 4>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += 4) {
      auto va = drjit::load<Packet>(&a[i]);
      auto vb = drjit::load<Packet>(&b[i]);
      auto vc = va * vb;
      drjit::store(&result[i], vc);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 2);
}
BENCHMARK(BM_DrJit_Mul_Float4)->RangeMultiplier(4)->Range(64, 16384);

static void BM_DartSimd_Sqrt_Float4(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data = generateAlignedData<float>(n, 1);
  std::vector<float> result(n);

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += 4) {
      auto v = ds::Vec<float, 4>::load(&data[i]);
      auto r = ds::sqrt(v);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float));
}
BENCHMARK(BM_DartSimd_Sqrt_Float4)->RangeMultiplier(4)->Range(64, 16384);

static void BM_DrJit_Sqrt_Float4(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data = generateAlignedData<float>(n, 1);
  std::vector<float> result(n);

  using Packet = drjit::Packet<float, 4>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += 4) {
      auto v = drjit::load<Packet>(&data[i]);
      auto r = drjit::sqrt(v);
      drjit::store(&result[i], r);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float));
}
BENCHMARK(BM_DrJit_Sqrt_Float4)->RangeMultiplier(4)->Range(64, 16384);

static void BM_DartSimd_Fmadd_Float4(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto a = generateAlignedData<float>(n, 1);
  auto b = generateAlignedData<float>(n, 2);
  auto c = generateAlignedData<float>(n, 3);
  std::vector<float> result(n);

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += 4) {
      auto va = ds::Vec<float, 4>::load(&a[i]);
      auto vb = ds::Vec<float, 4>::load(&b[i]);
      auto vc = ds::Vec<float, 4>::load(&c[i]);
      auto vr = ds::fmadd(va, vb, vc);
      vr.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 3);
}
BENCHMARK(BM_DartSimd_Fmadd_Float4)->RangeMultiplier(4)->Range(64, 16384);

static void BM_DrJit_Fmadd_Float4(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto a = generateAlignedData<float>(n, 1);
  auto b = generateAlignedData<float>(n, 2);
  auto c = generateAlignedData<float>(n, 3);
  std::vector<float> result(n);

  using Packet = drjit::Packet<float, 4>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += 4) {
      auto va = drjit::load<Packet>(&a[i]);
      auto vb = drjit::load<Packet>(&b[i]);
      auto vc = drjit::load<Packet>(&c[i]);
      auto vr = drjit::fmadd(va, vb, vc);
      drjit::store(&result[i], vr);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 3);
}
BENCHMARK(BM_DrJit_Fmadd_Float4)->RangeMultiplier(4)->Range(64, 16384);

static void BM_DartSimd_DotProduct(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto a = generateAlignedData<float>(n, 1);
  auto b = generateAlignedData<float>(n, 2);

  for (auto _ : state) {
    ds::Vec<float, 4> sum = ds::Vec<float, 4>::zero();
    for (std::size_t i = 0; i < n; i += 4) {
      auto va = ds::Vec<float, 4>::load(&a[i]);
      auto vb = ds::Vec<float, 4>::load(&b[i]);
      sum = ds::fmadd(va, vb, sum);
    }
    float result = ds::hsum(sum);
    benchmark::DoNotOptimize(result);
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
BENCHMARK(BM_DartSimd_DotProduct)->RangeMultiplier(4)->Range(64, 16384);

static void BM_DrJit_DotProduct(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto a = generateAlignedData<float>(n, 1);
  auto b = generateAlignedData<float>(n, 2);

  using Packet = drjit::Packet<float, 4>;

  for (auto _ : state) {
    Packet sum = drjit::zeros<Packet>();
    for (std::size_t i = 0; i < n; i += 4) {
      auto va = drjit::load<Packet>(&a[i]);
      auto vb = drjit::load<Packet>(&b[i]);
      sum = drjit::fmadd(va, vb, sum);
    }
    float result = drjit::sum(sum);
    benchmark::DoNotOptimize(result);
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
BENCHMARK(BM_DrJit_DotProduct)->RangeMultiplier(4)->Range(64, 16384);

static void BM_DartSimd_Add_Double4(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto a = generateAlignedData<double>(n, 1);
  auto b = generateAlignedData<double>(n, 2);
  std::vector<double> result(n);

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += 4) {
      auto va = ds::Vec<double, 4>::load(&a[i]);
      auto vb = ds::Vec<double, 4>::load(&b[i]);
      auto vc = va + vb;
      vc.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(double) * 2);
}
BENCHMARK(BM_DartSimd_Add_Double4)->RangeMultiplier(4)->Range(64, 16384);

static void BM_DrJit_Add_Double4(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto a = generateAlignedData<double>(n, 1);
  auto b = generateAlignedData<double>(n, 2);
  std::vector<double> result(n);

  using Packet = drjit::Packet<double, 4>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += 4) {
      auto va = drjit::load<Packet>(&a[i]);
      auto vb = drjit::load<Packet>(&b[i]);
      auto vc = va + vb;
      drjit::store(&result[i], vc);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(double) * 2);
}
BENCHMARK(BM_DrJit_Add_Double4)->RangeMultiplier(4)->Range(64, 16384);

BENCHMARK_MAIN();
