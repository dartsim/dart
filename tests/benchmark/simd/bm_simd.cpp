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

#include <iostream>
#include <random>
#include <vector>

// Optional drjit comparison (enabled via DART_HAS_DRJIT compile definition)
#if defined(DART_HAS_DRJIT)
  #include <drjit/packet.h>
#endif

using namespace dart::simd;

namespace {

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

// =============================================================================
// Size ranges for different benchmark categories
// =============================================================================

// Toy/synthetic benchmarks (current range)
constexpr int kMinSize = 1 << 10; // 1024
constexpr int kMaxSize = 1 << 16; // 65536

// Real-world robotics/physics scales
// These represent actual workload sizes in DART simulations

// Collision detection: AABB pairs to test in broadphase
// - Simple scene: ~100 objects → 100 pairs
// - Complex scene: ~1000 objects → ~10K-50K pairs
constexpr int kCollisionMin = 64;
constexpr int kCollisionMax = 1 << 14; // 16384

// Mesh processing: vertices per mesh
// - Simple robot link: ~1K vertices
// - Complex mesh: ~100K vertices
// - Point cloud (LiDAR frame): ~100K-1M points
constexpr int kMeshMin = 1 << 10;     // 1K
constexpr int kMeshMax = 1 << 20;     // 1M (extreme case)
constexpr int kMeshTypical = 1 << 16; // 64K (typical mesh)

// Dynamics: Jacobian/mass matrix sizes
// - Simple arm: 6 DOF → 36 elements
// - Humanoid: ~30-50 DOF → 900-2500 elements
// - Multi-robot: ~100+ DOF → 10K+ elements
constexpr int kDynamicsMin = 36;
constexpr int kDynamicsMax = 1 << 14; // 16384

// Contact handling: contacts per frame
// - Light contact: 10-50
// - Heavy contact (pile): 100-1000
// - Extreme (granular): 10K+
constexpr int kContactMin = 16;
constexpr int kContactMax = 1 << 12; // 4096

} // namespace

// =============================================================================
// Float Add Benchmarks (DART vs drjit)
// =============================================================================

static void BM_Add_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data_a = generateRandomData<float>(n, 1);
  auto data_b = generateRandomData<float>(n, 2);
  aligned_vector<float> result(n);

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto a = VecT::load(&data_a[i]);
      auto b = VecT::load(&data_b[i]);
      auto c = a + b;
      c.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 2);
}
BENCHMARK(BM_Add_DART_f32)->RangeMultiplier(4)->Range(kMinSize, kMaxSize);

static void BM_Add_DART_f32_Unrolled(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data_a = generateRandomData<float>(n, 1);
  auto data_b = generateRandomData<float>(n, 2);
  aligned_vector<float> result(n);

  constexpr std::size_t W = preferred_width_v<float>;
  constexpr std::size_t STRIDE = W * 2;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    std::size_t i = 0;
    for (; i + STRIDE <= n; i += STRIDE) {
      auto a0 = VecT::load(&data_a[i]);
      auto a1 = VecT::load(&data_a[i + W]);
      auto b0 = VecT::load(&data_b[i]);
      auto b1 = VecT::load(&data_b[i + W]);
      auto c0 = a0 + b0;
      auto c1 = a1 + b1;
      c0.store(&result[i]);
      c1.store(&result[i + W]);
    }
    for (; i < n; i += W) {
      auto a = VecT::load(&data_a[i]);
      auto b = VecT::load(&data_b[i]);
      (a + b).store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 2);
}
BENCHMARK(BM_Add_DART_f32_Unrolled)
    ->RangeMultiplier(4)
    ->Range(kMinSize, kMaxSize);

#if defined(DART_HAS_DRJIT)
static void BM_Add_DrJit_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto a = generateRandomData<float>(n, 1);
  auto b = generateRandomData<float>(n, 2);
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
BENCHMARK(BM_Add_DrJit_f32)->RangeMultiplier(4)->Range(kMinSize, kMaxSize);
#endif

// =============================================================================
// Float Multiply Benchmarks
// =============================================================================

static void BM_Mul_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data_a = generateRandomData<float>(n, 1);
  auto data_b = generateRandomData<float>(n, 2);
  aligned_vector<float> result(n);

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto a = VecT::load(&data_a[i]);
      auto b = VecT::load(&data_b[i]);
      auto c = a * b;
      c.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 2);
}
BENCHMARK(BM_Mul_DART_f32)->RangeMultiplier(4)->Range(kMinSize, kMaxSize);

#if defined(DART_HAS_DRJIT)
static void BM_Mul_DrJit_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto a = generateRandomData<float>(n, 1);
  auto b = generateRandomData<float>(n, 2);
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
BENCHMARK(BM_Mul_DrJit_f32)->RangeMultiplier(4)->Range(kMinSize, kMaxSize);
#endif

// =============================================================================
// Float Division Benchmarks
// =============================================================================

static void BM_Div_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data_a = generateRandomData<float>(n, 1);
  auto data_b = generateRandomData<float>(n, 2);
  aligned_vector<float> result(n);

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto a = VecT::load(&data_a[i]);
      auto b = VecT::load(&data_b[i]);
      auto c = a / b;
      c.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 2);
}
BENCHMARK(BM_Div_DART_f32)->RangeMultiplier(4)->Range(kMinSize, kMaxSize);

// =============================================================================
// Float Sqrt Benchmarks
// =============================================================================

static void BM_Sqrt_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data = generateRandomData<float>(n, 1);
  aligned_vector<float> result(n);

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto v = VecT::load(&data[i]);
      auto r = sqrt(v);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float));
}
BENCHMARK(BM_Sqrt_DART_f32)->RangeMultiplier(4)->Range(kMinSize, kMaxSize);

#if defined(DART_HAS_DRJIT)
static void BM_Sqrt_DrJit_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data = generateRandomData<float>(n, 1);
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
BENCHMARK(BM_Sqrt_DrJit_f32)->RangeMultiplier(4)->Range(kMinSize, kMaxSize);
#endif

// =============================================================================
// Float FMA Benchmarks (Fused Multiply-Add)
// =============================================================================

static void BM_FMA_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data_a = generateRandomData<float>(n, 1);
  auto data_b = generateRandomData<float>(n, 2);
  auto data_c = generateRandomData<float>(n, 3);
  aligned_vector<float> result(n);

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto a = VecT::load(&data_a[i]);
      auto b = VecT::load(&data_b[i]);
      auto c = VecT::load(&data_c[i]);
      auto r = fmadd(a, b, c);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 3);
}
BENCHMARK(BM_FMA_DART_f32)->RangeMultiplier(4)->Range(kMinSize, kMaxSize);

static void BM_FMA_DART_f32_Unrolled(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data_a = generateRandomData<float>(n, 1);
  auto data_b = generateRandomData<float>(n, 2);
  auto data_c = generateRandomData<float>(n, 3);
  aligned_vector<float> result(n);

  constexpr std::size_t W = preferred_width_v<float>;
  constexpr std::size_t STRIDE = W * 2;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    std::size_t i = 0;
    for (; i + STRIDE <= n; i += STRIDE) {
      auto a0 = VecT::load(&data_a[i]);
      auto a1 = VecT::load(&data_a[i + W]);
      auto b0 = VecT::load(&data_b[i]);
      auto b1 = VecT::load(&data_b[i + W]);
      auto c0 = VecT::load(&data_c[i]);
      auto c1 = VecT::load(&data_c[i + W]);
      auto r0 = fmadd(a0, b0, c0);
      auto r1 = fmadd(a1, b1, c1);
      r0.store(&result[i]);
      r1.store(&result[i + W]);
    }
    for (; i < n; i += W) {
      auto a = VecT::load(&data_a[i]);
      auto b = VecT::load(&data_b[i]);
      auto c = VecT::load(&data_c[i]);
      fmadd(a, b, c).store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 3);
}
BENCHMARK(BM_FMA_DART_f32_Unrolled)
    ->RangeMultiplier(4)
    ->Range(kMinSize, kMaxSize);

#if defined(DART_HAS_DRJIT)
static void BM_FMA_DrJit_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto a = generateRandomData<float>(n, 1);
  auto b = generateRandomData<float>(n, 2);
  auto c = generateRandomData<float>(n, 3);
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
BENCHMARK(BM_FMA_DrJit_f32)->RangeMultiplier(4)->Range(kMinSize, kMaxSize);
#endif

// =============================================================================
// Dot Product Benchmarks
// =============================================================================

static void BM_Dot_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data_a = generateRandomData<float>(n, 1);
  auto data_b = generateRandomData<float>(n, 2);

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    VecT acc = VecT::zero();
    for (std::size_t i = 0; i < n; i += W) {
      auto a = VecT::load(&data_a[i]);
      auto b = VecT::load(&data_b[i]);
      acc = fmadd(a, b, acc);
    }
    float result = hsum(acc);
    benchmark::DoNotOptimize(result);
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 2);
}
BENCHMARK(BM_Dot_DART_f32)->RangeMultiplier(4)->Range(kMinSize, kMaxSize);

static void BM_Dot_DART_f32_Unrolled(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data_a = generateRandomData<float>(n, 1);
  auto data_b = generateRandomData<float>(n, 2);

  constexpr std::size_t W = preferred_width_v<float>;
  constexpr std::size_t STRIDE = W * 4;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    VecT acc0 = VecT::zero();
    VecT acc1 = VecT::zero();
    VecT acc2 = VecT::zero();
    VecT acc3 = VecT::zero();
    std::size_t i = 0;
    for (; i + STRIDE <= n; i += STRIDE) {
      auto a0 = VecT::load(&data_a[i]);
      auto a1 = VecT::load(&data_a[i + W]);
      auto a2 = VecT::load(&data_a[i + W * 2]);
      auto a3 = VecT::load(&data_a[i + W * 3]);
      auto b0 = VecT::load(&data_b[i]);
      auto b1 = VecT::load(&data_b[i + W]);
      auto b2 = VecT::load(&data_b[i + W * 2]);
      auto b3 = VecT::load(&data_b[i + W * 3]);
      acc0 = fmadd(a0, b0, acc0);
      acc1 = fmadd(a1, b1, acc1);
      acc2 = fmadd(a2, b2, acc2);
      acc3 = fmadd(a3, b3, acc3);
    }
    for (; i < n; i += W) {
      auto a = VecT::load(&data_a[i]);
      auto b = VecT::load(&data_b[i]);
      acc0 = fmadd(a, b, acc0);
    }
    float result = hsum(acc0 + acc1 + acc2 + acc3);
    benchmark::DoNotOptimize(result);
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 2);
}
BENCHMARK(BM_Dot_DART_f32_Unrolled)
    ->RangeMultiplier(4)
    ->Range(kMinSize, kMaxSize);

#if defined(DART_HAS_DRJIT)
static void BM_Dot_DrJit_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto a = generateRandomData<float>(n, 1);
  auto b = generateRandomData<float>(n, 2);

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
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 2);
}
BENCHMARK(BM_Dot_DrJit_f32)->RangeMultiplier(4)->Range(kMinSize, kMaxSize);
#endif

// =============================================================================
// Double Add Benchmarks
// =============================================================================

static void BM_Add_DART_f64(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data_a = generateRandomData<double>(n, 1);
  auto data_b = generateRandomData<double>(n, 2);
  aligned_vector<double> result(n);

  constexpr std::size_t W = preferred_width_v<double>;
  using VecT = Vec<double, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto a = VecT::load(&data_a[i]);
      auto b = VecT::load(&data_b[i]);
      auto c = a + b;
      c.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(double) * 2);
}
BENCHMARK(BM_Add_DART_f64)->RangeMultiplier(4)->Range(kMinSize, kMaxSize);

#if defined(DART_HAS_DRJIT)
static void BM_Add_DrJit_f64(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto a = generateRandomData<double>(n, 1);
  auto b = generateRandomData<double>(n, 2);
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
BENCHMARK(BM_Add_DrJit_f64)->RangeMultiplier(4)->Range(kMinSize, kMaxSize);
#endif

// =============================================================================
// DART-only benchmarks (operations without drjit equivalent)
// =============================================================================

static void BM_Rsqrt_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data = generateRandomData<float>(n, 1);
  aligned_vector<float> result(n);

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto v = VecT::load(&data[i]);
      auto r = rsqrt(v);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float));
}
BENCHMARK(BM_Rsqrt_DART_f32)->RangeMultiplier(4)->Range(kMinSize, kMaxSize);

static void BM_Min_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data_a = generateRandomData<float>(n, 1);
  auto data_b = generateRandomData<float>(n, 2);
  aligned_vector<float> result(n);

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto a = VecT::load(&data_a[i]);
      auto b = VecT::load(&data_b[i]);
      auto r = min(a, b);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 2);
}
BENCHMARK(BM_Min_DART_f32)->RangeMultiplier(4)->Range(kMinSize, kMaxSize);

static void BM_Select_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto data_a = generateRandomData<float>(n, 1);
  auto data_b = generateRandomData<float>(n, 2);
  aligned_vector<float> result(n);

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto a = VecT::load(&data_a[i]);
      auto b = VecT::load(&data_b[i]);
      auto mask = a < b;
      auto r = select(mask, a, b);
      r.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 2);
}
BENCHMARK(BM_Select_DART_f32)->RangeMultiplier(4)->Range(kMinSize, kMaxSize);

// =============================================================================
// Real-World Scale Benchmarks
// =============================================================================
// These benchmarks use sizes typical in robotics/physics simulation

// -----------------------------------------------------------------------------
// Batch Point Transform (Mesh/Point Cloud Processing)
// Real-world: Transform vertices of a mesh or point cloud by a 4x4 matrix
// Typical sizes: 1K-100K vertices for meshes, up to 1M for point clouds
// -----------------------------------------------------------------------------

static void BM_TransformPoints_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  // Generate n 3D points (padded to 4 floats each for SIMD alignment)
  aligned_vector<float> points_x(n);
  aligned_vector<float> points_y(n);
  aligned_vector<float> points_z(n);
  aligned_vector<float> result_x(n);
  aligned_vector<float> result_y(n);
  aligned_vector<float> result_z(n);

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> dist(-10.0f, 10.0f);
  for (std::size_t i = 0; i < n; ++i) {
    points_x[i] = dist(gen);
    points_y[i] = dist(gen);
    points_z[i] = dist(gen);
  }

  // 4x4 transform matrix (rotation + translation)
  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  // Simplified: just apply rotation and translation
  VecT m00 = VecT::broadcast(0.866f); // cos(30°)
  VecT m01 = VecT::broadcast(-0.5f);  // -sin(30°)
  VecT m10 = VecT::broadcast(0.5f);   // sin(30°)
  VecT m11 = VecT::broadcast(0.866f);
  VecT tx = VecT::broadcast(1.0f);
  VecT ty = VecT::broadcast(2.0f);
  VecT tz = VecT::broadcast(3.0f);

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto x = VecT::load(&points_x[i]);
      auto y = VecT::load(&points_y[i]);
      auto z = VecT::load(&points_z[i]);

      // Apply rotation (Z-axis rotation for simplicity) + translation
      auto rx = fmadd(m00, x, fmadd(m01, y, tx));
      auto ry = fmadd(m10, x, fmadd(m11, y, ty));
      auto rz = z + tz;

      rx.store(&result_x[i]);
      ry.store(&result_y[i]);
      rz.store(&result_z[i]);
    }
    benchmark::DoNotOptimize(result_x.data());
    benchmark::DoNotOptimize(result_y.data());
    benchmark::DoNotOptimize(result_z.data());
  }
  // 3 floats read + 3 floats written per point
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 6);
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
// Mesh scales: 1K, 4K, 16K, 64K, 256K vertices
BENCHMARK(BM_TransformPoints_DART_f32)
    ->RangeMultiplier(4)
    ->Range(kMeshMin, kMeshTypical);

// Point cloud scale: up to 1M points (extreme but realistic for LiDAR)
BENCHMARK(BM_TransformPoints_DART_f32)
    ->Name("BM_TransformPoints_DART_f32_PointCloud")
    ->Arg(1 << 18)
    ->Arg(1 << 20);

#if defined(DART_HAS_DRJIT)
static void BM_TransformPoints_DrJit_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  std::vector<float> points_x(n), points_y(n), points_z(n);
  std::vector<float> result_x(n), result_y(n), result_z(n);

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> dist(-10.0f, 10.0f);
  for (std::size_t i = 0; i < n; ++i) {
    points_x[i] = dist(gen);
    points_y[i] = dist(gen);
    points_z[i] = dist(gen);
  }

  using Packet = drjit::Packet<float, 4>;
  Packet m00 = drjit::full<Packet>(0.866f);
  Packet m01 = drjit::full<Packet>(-0.5f);
  Packet m10 = drjit::full<Packet>(0.5f);
  Packet m11 = drjit::full<Packet>(0.866f);
  Packet tx = drjit::full<Packet>(1.0f);
  Packet ty = drjit::full<Packet>(2.0f);
  Packet tz = drjit::full<Packet>(3.0f);

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += 4) {
      auto x = drjit::load<Packet>(&points_x[i]);
      auto y = drjit::load<Packet>(&points_y[i]);
      auto z = drjit::load<Packet>(&points_z[i]);

      auto rx = drjit::fmadd(m00, x, drjit::fmadd(m01, y, tx));
      auto ry = drjit::fmadd(m10, x, drjit::fmadd(m11, y, ty));
      auto rz = z + tz;

      drjit::store(&result_x[i], rx);
      drjit::store(&result_y[i], ry);
      drjit::store(&result_z[i], rz);
    }
    benchmark::DoNotOptimize(result_x.data());
    benchmark::DoNotOptimize(result_y.data());
    benchmark::DoNotOptimize(result_z.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 6);
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
BENCHMARK(BM_TransformPoints_DrJit_f32)
    ->RangeMultiplier(4)
    ->Range(kMeshMin, kMeshTypical);
#endif

// -----------------------------------------------------------------------------
// Batch Dot Product (Contact Normal Processing)
// Real-world: Compute contact penetration depths, force projections
// Typical sizes: 10-1000 contacts per frame
// -----------------------------------------------------------------------------

static void BM_BatchDot3_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  // Two arrays of 3D vectors
  aligned_vector<float> a_x(n), a_y(n), a_z(n);
  aligned_vector<float> b_x(n), b_y(n), b_z(n);
  aligned_vector<float> result(n);

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  for (std::size_t i = 0; i < n; ++i) {
    a_x[i] = dist(gen);
    a_y[i] = dist(gen);
    a_z[i] = dist(gen);
    b_x[i] = dist(gen);
    b_y[i] = dist(gen);
    b_z[i] = dist(gen);
  }

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto ax = VecT::load(&a_x[i]);
      auto ay = VecT::load(&a_y[i]);
      auto az = VecT::load(&a_z[i]);
      auto bx = VecT::load(&b_x[i]);
      auto by = VecT::load(&b_y[i]);
      auto bz = VecT::load(&b_z[i]);

      // dot(a, b) = ax*bx + ay*by + az*bz
      auto dot = fmadd(ax, bx, fmadd(ay, by, az * bz));
      dot.store(&result[i]);
    }
    benchmark::DoNotOptimize(result.data());
  }
  // 6 floats read, 1 float written per pair
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 7);
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
// Contact scales: 16, 64, 256, 1024, 4096 contacts
BENCHMARK(BM_BatchDot3_DART_f32)
    ->RangeMultiplier(4)
    ->Range(kContactMin, kContactMax);

#if defined(DART_HAS_DRJIT)
static void BM_BatchDot3_DrJit_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  std::vector<float> a_x(n), a_y(n), a_z(n);
  std::vector<float> b_x(n), b_y(n), b_z(n);
  std::vector<float> result(n);

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  for (std::size_t i = 0; i < n; ++i) {
    a_x[i] = dist(gen);
    a_y[i] = dist(gen);
    a_z[i] = dist(gen);
    b_x[i] = dist(gen);
    b_y[i] = dist(gen);
    b_z[i] = dist(gen);
  }

  using Packet = drjit::Packet<float, 4>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += 4) {
      auto ax = drjit::load<Packet>(&a_x[i]);
      auto ay = drjit::load<Packet>(&a_y[i]);
      auto az = drjit::load<Packet>(&a_z[i]);
      auto bx = drjit::load<Packet>(&b_x[i]);
      auto by = drjit::load<Packet>(&b_y[i]);
      auto bz = drjit::load<Packet>(&b_z[i]);

      // dot(a, b) = ax*bx + ay*by + az*bz
      auto dot = drjit::fmadd(ax, bx, drjit::fmadd(ay, by, az * bz));
      drjit::store(&result[i], dot);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 7);
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
BENCHMARK(BM_BatchDot3_DrJit_f32)
    ->RangeMultiplier(4)
    ->Range(kContactMin, kContactMax);
#endif

// -----------------------------------------------------------------------------
// Batch Cross Product (Torque Computation)
// Real-world: Compute torques τ = r × F for many contact points
// Typical sizes: 10-1000 contacts
// -----------------------------------------------------------------------------

static void BM_BatchCross3_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  aligned_vector<float> a_x(n), a_y(n), a_z(n);
  aligned_vector<float> b_x(n), b_y(n), b_z(n);
  aligned_vector<float> r_x(n), r_y(n), r_z(n);

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  for (std::size_t i = 0; i < n; ++i) {
    a_x[i] = dist(gen);
    a_y[i] = dist(gen);
    a_z[i] = dist(gen);
    b_x[i] = dist(gen);
    b_y[i] = dist(gen);
    b_z[i] = dist(gen);
  }

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto ax = VecT::load(&a_x[i]);
      auto ay = VecT::load(&a_y[i]);
      auto az = VecT::load(&a_z[i]);
      auto bx = VecT::load(&b_x[i]);
      auto by = VecT::load(&b_y[i]);
      auto bz = VecT::load(&b_z[i]);

      // cross(a, b) = (ay*bz - az*by, az*bx - ax*bz, ax*by - ay*bx)
      auto cx = fmsub(ay, bz, az * by);
      auto cy = fmsub(az, bx, ax * bz);
      auto cz = fmsub(ax, by, ay * bx);

      cx.store(&r_x[i]);
      cy.store(&r_y[i]);
      cz.store(&r_z[i]);
    }
    benchmark::DoNotOptimize(r_x.data());
    benchmark::DoNotOptimize(r_y.data());
    benchmark::DoNotOptimize(r_z.data());
  }
  // 6 floats read, 3 floats written per pair
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 9);
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
BENCHMARK(BM_BatchCross3_DART_f32)
    ->RangeMultiplier(4)
    ->Range(kContactMin, kContactMax);

#if defined(DART_HAS_DRJIT)
static void BM_BatchCross3_DrJit_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  std::vector<float> a_x(n), a_y(n), a_z(n);
  std::vector<float> b_x(n), b_y(n), b_z(n);
  std::vector<float> r_x(n), r_y(n), r_z(n);

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  for (std::size_t i = 0; i < n; ++i) {
    a_x[i] = dist(gen);
    a_y[i] = dist(gen);
    a_z[i] = dist(gen);
    b_x[i] = dist(gen);
    b_y[i] = dist(gen);
    b_z[i] = dist(gen);
  }

  using Packet = drjit::Packet<float, 4>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += 4) {
      auto ax = drjit::load<Packet>(&a_x[i]);
      auto ay = drjit::load<Packet>(&a_y[i]);
      auto az = drjit::load<Packet>(&a_z[i]);
      auto bx = drjit::load<Packet>(&b_x[i]);
      auto by = drjit::load<Packet>(&b_y[i]);
      auto bz = drjit::load<Packet>(&b_z[i]);

      // cross(a, b) = (ay*bz - az*by, az*bx - ax*bz, ax*by - ay*bx)
      auto cx = drjit::fmsub(ay, bz, az * by);
      auto cy = drjit::fmsub(az, bx, ax * bz);
      auto cz = drjit::fmsub(ax, by, ay * bx);

      drjit::store(&r_x[i], cx);
      drjit::store(&r_y[i], cy);
      drjit::store(&r_z[i], cz);
    }
    benchmark::DoNotOptimize(r_x.data());
    benchmark::DoNotOptimize(r_y.data());
    benchmark::DoNotOptimize(r_z.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 9);
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
BENCHMARK(BM_BatchCross3_DrJit_f32)
    ->RangeMultiplier(4)
    ->Range(kContactMin, kContactMax);
#endif

// -----------------------------------------------------------------------------
// Batch Normalize (Unit Vector Computation)
// Real-world: Normalize contact normals, compute unit directions
// Typical sizes: Similar to contact handling
// -----------------------------------------------------------------------------

static void BM_BatchNormalize3_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  aligned_vector<float> v_x(n), v_y(n), v_z(n);

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> dist(0.1f, 10.0f);
  for (std::size_t i = 0; i < n; ++i) {
    v_x[i] = dist(gen);
    v_y[i] = dist(gen);
    v_z[i] = dist(gen);
  }

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      auto x = VecT::load(&v_x[i]);
      auto y = VecT::load(&v_y[i]);
      auto z = VecT::load(&v_z[i]);

      // norm = sqrt(x² + y² + z²)
      auto norm_sq = fmadd(x, x, fmadd(y, y, z * z));
      auto inv_norm = rsqrt(norm_sq);

      // Normalize
      (x * inv_norm).store(&v_x[i]);
      (y * inv_norm).store(&v_y[i]);
      (z * inv_norm).store(&v_z[i]);
    }
    benchmark::DoNotOptimize(v_x.data());
    benchmark::DoNotOptimize(v_y.data());
    benchmark::DoNotOptimize(v_z.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 6);
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
BENCHMARK(BM_BatchNormalize3_DART_f32)
    ->RangeMultiplier(4)
    ->Range(kContactMin, kContactMax);

#if defined(DART_HAS_DRJIT)
static void BM_BatchNormalize3_DrJit_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  std::vector<float> v_x(n), v_y(n), v_z(n);

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> dist(0.1f, 10.0f);
  for (std::size_t i = 0; i < n; ++i) {
    v_x[i] = dist(gen);
    v_y[i] = dist(gen);
    v_z[i] = dist(gen);
  }

  using Packet = drjit::Packet<float, 4>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += 4) {
      auto x = drjit::load<Packet>(&v_x[i]);
      auto y = drjit::load<Packet>(&v_y[i]);
      auto z = drjit::load<Packet>(&v_z[i]);

      // norm = sqrt(x² + y² + z²)
      auto norm_sq = drjit::fmadd(x, x, drjit::fmadd(y, y, z * z));
      auto inv_norm = drjit::rsqrt(norm_sq);

      // Normalize
      drjit::store(&v_x[i], x * inv_norm);
      drjit::store(&v_y[i], y * inv_norm);
      drjit::store(&v_z[i], z * inv_norm);
    }
    benchmark::DoNotOptimize(v_x.data());
    benchmark::DoNotOptimize(v_y.data());
    benchmark::DoNotOptimize(v_z.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 6);
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
BENCHMARK(BM_BatchNormalize3_DrJit_f32)
    ->RangeMultiplier(4)
    ->Range(kContactMin, kContactMax);
#endif

// -----------------------------------------------------------------------------
// Matrix-Vector Multiply (Jacobian Application)
// Real-world: Apply Jacobian to joint velocities, compute end-effector velocity
// Typical sizes: 6×N where N=6-100 DOFs → 36-600 elements per row
// -----------------------------------------------------------------------------

static void BM_MatVec_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0); // Vector size (DOFs)
  const std::size_t m = 6;              // Output size (6D twist)

  // Row-major matrix (m × n)
  aligned_vector<float> matrix(m * n);
  aligned_vector<float> vec(n);
  aligned_vector<float> result(m);

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  for (auto& v : matrix)
    v = dist(gen);
  for (auto& v : vec)
    v = dist(gen);

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t row = 0; row < m; ++row) {
      VecT acc = VecT::zero();
      const float* row_ptr = &matrix[row * n];
      for (std::size_t j = 0; j < n; j += W) {
        auto mat_val = VecT::load(&row_ptr[j]);
        auto vec_val = VecT::load(&vec[j]);
        acc = fmadd(mat_val, vec_val, acc);
      }
      result[row] = hsum(acc);
    }
    benchmark::DoNotOptimize(result.data());
  }
  // m rows × n elements read from matrix + n elements from vec
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * (m * n + n) * sizeof(float));
}
// Jacobian sizes: 36 (6 DOF), 180 (30 DOF humanoid), 600 (100 DOF)
BENCHMARK(BM_MatVec_DART_f32)
    ->Arg(kDynamicsMin)
    ->Arg(180)
    ->Arg(600)
    ->Arg(kDynamicsMax);

#if defined(DART_HAS_DRJIT)
static void BM_MatVec_DrJit_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  const std::size_t m = 6;

  std::vector<float> matrix(m * n);
  std::vector<float> vec(n);
  std::vector<float> result(m);

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  for (auto& v : matrix)
    v = dist(gen);
  for (auto& v : vec)
    v = dist(gen);

  using Packet = drjit::Packet<float, 4>;

  for (auto _ : state) {
    for (std::size_t row = 0; row < m; ++row) {
      Packet acc = drjit::zeros<Packet>();
      const float* row_ptr = &matrix[row * n];
      for (std::size_t j = 0; j < n; j += 4) {
        auto mat_val = drjit::load<Packet>(&row_ptr[j]);
        auto vec_val = drjit::load<Packet>(&vec[j]);
        acc = drjit::fmadd(mat_val, vec_val, acc);
      }
      result[row] = drjit::sum(acc);
    }
    benchmark::DoNotOptimize(result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * (m * n + n) * sizeof(float));
}
BENCHMARK(BM_MatVec_DrJit_f32)
    ->Arg(kDynamicsMin)
    ->Arg(180)
    ->Arg(600)
    ->Arg(kDynamicsMax);
#endif

// -----------------------------------------------------------------------------
// AABB Overlap Test (Collision Broadphase)
// Real-world: Test N AABB pairs for overlap in broadphase collision detection
// Typical sizes: 100-10000 pairs per frame
// -----------------------------------------------------------------------------

static void BM_AABBOverlap_DART_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0); // Number of AABB pairs

  // AABB = (min_x, min_y, min_z, max_x, max_y, max_z)
  // Store as SoA for SIMD efficiency
  aligned_vector<float> a_min_x(n), a_min_y(n), a_min_z(n);
  aligned_vector<float> a_max_x(n), a_max_y(n), a_max_z(n);
  aligned_vector<float> b_min_x(n), b_min_y(n), b_min_z(n);
  aligned_vector<float> b_max_x(n), b_max_y(n), b_max_z(n);
  aligned_vector<int32_t> overlap_result(n);

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> pos(-10.0f, 10.0f);
  std::uniform_real_distribution<float> size(0.1f, 2.0f);

  for (std::size_t i = 0; i < n; ++i) {
    float ax = pos(gen), ay = pos(gen), az = pos(gen);
    float as = size(gen);
    a_min_x[i] = ax;
    a_min_y[i] = ay;
    a_min_z[i] = az;
    a_max_x[i] = ax + as;
    a_max_y[i] = ay + as;
    a_max_z[i] = az + as;

    float bx = pos(gen), by = pos(gen), bz = pos(gen);
    float bs = size(gen);
    b_min_x[i] = bx;
    b_min_y[i] = by;
    b_min_z[i] = bz;
    b_max_x[i] = bx + bs;
    b_max_y[i] = by + bs;
    b_max_z[i] = bz + bs;
  }

  constexpr std::size_t W = preferred_width_v<float>;
  using VecT = Vec<float, W>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += W) {
      // Load AABBs
      auto a_minx = VecT::load(&a_min_x[i]);
      auto a_miny = VecT::load(&a_min_y[i]);
      auto a_minz = VecT::load(&a_min_z[i]);
      auto a_maxx = VecT::load(&a_max_x[i]);
      auto a_maxy = VecT::load(&a_max_y[i]);
      auto a_maxz = VecT::load(&a_max_z[i]);

      auto b_minx = VecT::load(&b_min_x[i]);
      auto b_miny = VecT::load(&b_min_y[i]);
      auto b_minz = VecT::load(&b_min_z[i]);
      auto b_maxx = VecT::load(&b_max_x[i]);
      auto b_maxy = VecT::load(&b_max_y[i]);
      auto b_maxz = VecT::load(&b_max_z[i]);

      // AABB overlap test: overlap iff (a.max >= b.min) && (a.min <= b.max) for
      // all axes
      auto ox = (a_maxx >= b_minx) & (a_minx <= b_maxx);
      auto oy = (a_maxy >= b_miny) & (a_miny <= b_maxy);
      auto oz = (a_maxz >= b_minz) & (a_minz <= b_maxz);
      auto overlap = ox & oy & oz;

      // Store result as int mask (1 = overlap, 0 = no overlap)
      auto result = select(overlap, VecT::broadcast(1.0f), VecT::zero());
      // Convert to int and store
      for (std::size_t j = 0; j < W && i + j < n; ++j) {
        overlap_result[i + j] = static_cast<int32_t>(result[j]);
      }
    }
    benchmark::DoNotOptimize(overlap_result.data());
  }
  // 12 floats read per pair
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 12);
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
// Collision broadphase scales
BENCHMARK(BM_AABBOverlap_DART_f32)
    ->RangeMultiplier(4)
    ->Range(kCollisionMin, kCollisionMax);

#if defined(DART_HAS_DRJIT)
static void BM_AABBOverlap_DrJit_f32(benchmark::State& state)
{
  const std::size_t n = state.range(0);

  std::vector<float> a_min_x(n), a_min_y(n), a_min_z(n);
  std::vector<float> a_max_x(n), a_max_y(n), a_max_z(n);
  std::vector<float> b_min_x(n), b_min_y(n), b_min_z(n);
  std::vector<float> b_max_x(n), b_max_y(n), b_max_z(n);
  std::vector<int32_t> overlap_result(n);

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> pos(-10.0f, 10.0f);
  std::uniform_real_distribution<float> size(0.1f, 2.0f);

  for (std::size_t i = 0; i < n; ++i) {
    float ax = pos(gen), ay = pos(gen), az = pos(gen);
    float as = size(gen);
    a_min_x[i] = ax;
    a_min_y[i] = ay;
    a_min_z[i] = az;
    a_max_x[i] = ax + as;
    a_max_y[i] = ay + as;
    a_max_z[i] = az + as;

    float bx = pos(gen), by = pos(gen), bz = pos(gen);
    float bs = size(gen);
    b_min_x[i] = bx;
    b_min_y[i] = by;
    b_min_z[i] = bz;
    b_max_x[i] = bx + bs;
    b_max_y[i] = by + bs;
    b_max_z[i] = bz + bs;
  }

  using Packet = drjit::Packet<float, 4>;

  for (auto _ : state) {
    for (std::size_t i = 0; i < n; i += 4) {
      auto a_minx = drjit::load<Packet>(&a_min_x[i]);
      auto a_miny = drjit::load<Packet>(&a_min_y[i]);
      auto a_minz = drjit::load<Packet>(&a_min_z[i]);
      auto a_maxx = drjit::load<Packet>(&a_max_x[i]);
      auto a_maxy = drjit::load<Packet>(&a_max_y[i]);
      auto a_maxz = drjit::load<Packet>(&a_max_z[i]);

      auto b_minx = drjit::load<Packet>(&b_min_x[i]);
      auto b_miny = drjit::load<Packet>(&b_min_y[i]);
      auto b_minz = drjit::load<Packet>(&b_min_z[i]);
      auto b_maxx = drjit::load<Packet>(&b_max_x[i]);
      auto b_maxy = drjit::load<Packet>(&b_max_y[i]);
      auto b_maxz = drjit::load<Packet>(&b_max_z[i]);

      // AABB overlap test
      auto ox = (a_maxx >= b_minx) & (a_minx <= b_maxx);
      auto oy = (a_maxy >= b_miny) & (a_miny <= b_maxy);
      auto oz = (a_maxz >= b_minz) & (a_minz <= b_maxz);
      auto overlap = ox & oy & oz;

      auto result = drjit::select(
          overlap, drjit::full<Packet>(1.0f), drjit::zeros<Packet>());
      for (std::size_t j = 0; j < 4 && i + j < n; ++j) {
        overlap_result[i + j] = static_cast<int32_t>(result[j]);
      }
    }
    benchmark::DoNotOptimize(overlap_result.data());
  }
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * n * sizeof(float) * 12);
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()) * n);
}
BENCHMARK(BM_AABBOverlap_DrJit_f32)
    ->RangeMultiplier(4)
    ->Range(kCollisionMin, kCollisionMax);
#endif

// =============================================================================
// Main with info output
// =============================================================================

int main(int argc, char** argv)
{
  std::cout << "dart::simd Benchmarks\n";
  std::cout << "=====================\n";
  std::cout << "Backend: " << dart::simd::backend_name << "\n";
#if defined(DART_HAS_DRJIT)
  std::cout << "drjit comparison: ENABLED\n";
#else
  std::cout
      << "drjit comparison: DISABLED (build with DART_SIMD_FETCH_DRJIT=ON "
         "to enable)\n";
#endif
  std::cout << "\n";
  std::cout << "Naming: BM_<Op>_<Library>_<Type>/<Size>\n";
  std::cout << "  - DART vs DrJit benchmarks are grouped by operation\n";
  std::cout << "  - Same sizes allow direct comparison\n";
  std::cout << "\n";
  std::cout << "Real-world scale benchmarks:\n";
  std::cout << "  - TransformPoints: Mesh/point cloud vertex transforms\n";
  std::cout << "  - BatchDot3/Cross3: Contact force computations\n";
  std::cout << "  - MatVec: Jacobian application\n";
  std::cout << "  - AABBOverlap: Collision broadphase\n";
  std::cout << "\n";

  ::benchmark::Initialize(&argc, argv);
  ::benchmark::RunSpecifiedBenchmarks();
  ::benchmark::Shutdown();
  return 0;
}
