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

/// @file bm_spatial_algebra.cpp
/// @brief Benchmarks for batched spatial algebra transforms (AdT, AdInvT,
///        dAdT, dAdInvT) using SIMD SoA layout.
///
/// Compares per-element Eigen loop against the batch::*_batch() functions
/// that use dart::simd SoA cross products for groups of 4 transforms.
///
/// Run:
///   ./bm_spatial_algebra --benchmark_counters_tabular=true
///
/// Expected result: 1.15×–1.25× speedup at batch sizes ≥ 8.

#include <dart/math/geometry.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <benchmark/benchmark.h>

#include <random>

namespace {

std::mt19937& rng()
{
  static std::mt19937 gen(42);
  return gen;
}

Eigen::Isometry3d randomTransform()
{
  std::uniform_real_distribution<double> dist(-10.0, 10.0);
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  Eigen::Vector3d axis
      = Eigen::Vector3d(dist(rng()), dist(rng()), dist(rng())).normalized();
  T.linear() = Eigen::AngleAxisd(dist(rng()), axis).toRotationMatrix();
  T.translation() = Eigen::Vector3d(dist(rng()), dist(rng()), dist(rng()));
  return T;
}

Eigen::Vector6d randomVector6()
{
  std::uniform_real_distribution<double> dist(-10.0, 10.0);
  Eigen::Vector6d v;
  for (int i = 0; i < 6; ++i) {
    v[i] = dist(rng());
  }
  return v;
}

// =============================================================================
// Helper: generate N random transforms + vectors
// =============================================================================

struct BatchData
{
  std::vector<Eigen::Isometry3d> transforms;
  std::vector<Eigen::Vector6d> inputs;
  std::vector<Eigen::Vector6d> outputs;
};

BatchData makeBatchData(int n)
{
  BatchData d;
  d.transforms.resize(n);
  d.inputs.resize(n);
  d.outputs.resize(n);
  for (int i = 0; i < n; ++i) {
    d.transforms[i] = randomTransform();
    d.inputs[i] = randomVector6();
  }
  return d;
}

// =============================================================================
// AdT Batch: Eigen per-call loop vs SIMD SoA batch
// =============================================================================

static void BM_AdT_Batch_Eigen(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  auto data = makeBatchData(n);
  for (auto _ : state) {
    for (int i = 0; i < n; ++i) {
      data.outputs[i] = dart::math::AdT(data.transforms[i], data.inputs[i]);
    }
    benchmark::DoNotOptimize(data.outputs.data());
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(n));
}
BENCHMARK(BM_AdT_Batch_Eigen)
    ->Repetitions(10)
    ->ReportAggregatesOnly(true)
    ->MinTime(1.0)
    ->Arg(4)   // Minimum SIMD batch (4 lanes)
    ->Arg(8)   // 2 SIMD batches
    ->Arg(16)  // 4 SIMD batches
    ->Arg(32)  // 8 SIMD batches
    ->Arg(64)  // 16 SIMD batches (multi-body tree)
    ->Arg(128) // Large articulated system
    ->Unit(benchmark::kNanosecond);

static void BM_AdT_Batch_SIMD(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  auto data = makeBatchData(n);
  for (auto _ : state) {
    dart::math::AdT_batch(
        data.transforms.data(), data.inputs.data(), data.outputs.data(), n);
    benchmark::DoNotOptimize(data.outputs.data());
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(n));
}
BENCHMARK(BM_AdT_Batch_SIMD)
    ->Repetitions(10)
    ->ReportAggregatesOnly(true)
    ->MinTime(1.0)
    ->Arg(4)
    ->Arg(8)
    ->Arg(16)
    ->Arg(32)
    ->Arg(64)
    ->Arg(128)
    ->Unit(benchmark::kNanosecond);

// =============================================================================
// AdInvT Batch: Eigen per-call loop vs SIMD SoA batch
// =============================================================================

static void BM_AdInvT_Batch_Eigen(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  auto data = makeBatchData(n);
  for (auto _ : state) {
    for (int i = 0; i < n; ++i) {
      data.outputs[i] = dart::math::AdInvT(data.transforms[i], data.inputs[i]);
    }
    benchmark::DoNotOptimize(data.outputs.data());
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(n));
}
BENCHMARK(BM_AdInvT_Batch_Eigen)
    ->Repetitions(10)
    ->ReportAggregatesOnly(true)
    ->MinTime(1.0)
    ->Arg(4)
    ->Arg(8)
    ->Arg(16)
    ->Arg(32)
    ->Arg(64)
    ->Arg(128)
    ->Unit(benchmark::kNanosecond);

static void BM_AdInvT_Batch_SIMD(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  auto data = makeBatchData(n);
  for (auto _ : state) {
    dart::math::AdInvT_batch(
        data.transforms.data(), data.inputs.data(), data.outputs.data(), n);
    benchmark::DoNotOptimize(data.outputs.data());
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(n));
}
BENCHMARK(BM_AdInvT_Batch_SIMD)
    ->Repetitions(10)
    ->ReportAggregatesOnly(true)
    ->MinTime(1.0)
    ->Arg(4)
    ->Arg(8)
    ->Arg(16)
    ->Arg(32)
    ->Arg(64)
    ->Arg(128)
    ->Unit(benchmark::kNanosecond);

// =============================================================================
// dAdT Batch: Eigen per-call loop vs SIMD SoA batch
// =============================================================================

static void BM_dAdT_Batch_Eigen(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  auto data = makeBatchData(n);
  for (auto _ : state) {
    for (int i = 0; i < n; ++i) {
      data.outputs[i] = dart::math::dAdT(data.transforms[i], data.inputs[i]);
    }
    benchmark::DoNotOptimize(data.outputs.data());
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(n));
}
BENCHMARK(BM_dAdT_Batch_Eigen)
    ->Repetitions(10)
    ->ReportAggregatesOnly(true)
    ->MinTime(1.0)
    ->Arg(4)
    ->Arg(8)
    ->Arg(16)
    ->Arg(32)
    ->Arg(64)
    ->Arg(128)
    ->Unit(benchmark::kNanosecond);

static void BM_dAdT_Batch_SIMD(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  auto data = makeBatchData(n);
  for (auto _ : state) {
    dart::math::dAdT_batch(
        data.transforms.data(), data.inputs.data(), data.outputs.data(), n);
    benchmark::DoNotOptimize(data.outputs.data());
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(n));
}
BENCHMARK(BM_dAdT_Batch_SIMD)
    ->Repetitions(10)
    ->ReportAggregatesOnly(true)
    ->MinTime(1.0)
    ->Arg(4)
    ->Arg(8)
    ->Arg(16)
    ->Arg(32)
    ->Arg(64)
    ->Arg(128)
    ->Unit(benchmark::kNanosecond);

// =============================================================================
// dAdInvT Batch: Eigen per-call loop vs SIMD SoA batch
// =============================================================================

static void BM_dAdInvT_Batch_Eigen(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  auto data = makeBatchData(n);
  for (auto _ : state) {
    for (int i = 0; i < n; ++i) {
      data.outputs[i] = dart::math::dAdInvT(data.transforms[i], data.inputs[i]);
    }
    benchmark::DoNotOptimize(data.outputs.data());
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(n));
}
BENCHMARK(BM_dAdInvT_Batch_Eigen)
    ->Repetitions(10)
    ->ReportAggregatesOnly(true)
    ->MinTime(1.0)
    ->Arg(4)
    ->Arg(8)
    ->Arg(16)
    ->Arg(32)
    ->Arg(64)
    ->Arg(128)
    ->Unit(benchmark::kNanosecond);

static void BM_dAdInvT_Batch_SIMD(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  auto data = makeBatchData(n);
  for (auto _ : state) {
    dart::math::dAdInvT_batch(
        data.transforms.data(), data.inputs.data(), data.outputs.data(), n);
    benchmark::DoNotOptimize(data.outputs.data());
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(n));
}
BENCHMARK(BM_dAdInvT_Batch_SIMD)
    ->Repetitions(10)
    ->ReportAggregatesOnly(true)
    ->MinTime(1.0)
    ->Arg(4)
    ->Arg(8)
    ->Arg(16)
    ->Arg(32)
    ->Arg(64)
    ->Arg(128)
    ->Unit(benchmark::kNanosecond);

} // namespace
