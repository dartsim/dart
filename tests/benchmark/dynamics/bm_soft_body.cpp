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

/// @file bm_soft_body.cpp
/// @brief Benchmarks for soft-body cross-product accumulation using SIMD.
///
/// Simulates the SoftBodyNode torque computation pattern:
///   torque += (position_i - center) × force_i
/// This is a sum of N cross products, batched 4 at a time via SoA layout.
///
/// Run:
///   ./bm_soft_body --benchmark_counters_tabular=true
///
/// Expected result: 1.2×–1.3× speedup at N ≥ 64.

#include <dart/simd/simd.hpp>

#include <Eigen/Core>
#include <benchmark/benchmark.h>

#include <random>
#include <vector>

namespace {

std::mt19937& rng()
{
  static std::mt19937 gen(42);
  return gen;
}

std::vector<Eigen::Vector3d> generatePoints(std::size_t count)
{
  std::uniform_real_distribution<double> dist(-10.0, 10.0);
  std::vector<Eigen::Vector3d> pts(count);
  for (auto& p : pts) {
    p = Eigen::Vector3d(dist(rng()), dist(rng()), dist(rng()));
  }
  return pts;
}

// =============================================================================
// Eigen: Scalar cross-product accumulation (baseline)
// Pattern: torque += (pos_i - center) × force_i
// =============================================================================

static void BM_CrossAccum_Eigen(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto positions = generatePoints(n);
  auto forces = generatePoints(n);
  Eigen::Vector3d center(0.0, 0.0, 0.0);

  for (auto _ : state) {
    Eigen::Vector3d torque = Eigen::Vector3d::Zero();
    for (std::size_t i = 0; i < n; ++i) {
      torque += (positions[i] - center).cross(forces[i]);
    }
    benchmark::DoNotOptimize(torque.data());
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(n));
}
BENCHMARK(BM_CrossAccum_Eigen)
    ->Repetitions(10)
    ->ReportAggregatesOnly(true)
    ->MinTime(1.0)
    ->Arg(16)   // Small soft body
    ->Arg(64)   // Typical soft body node
    ->Arg(128)  // Medium soft body
    ->Arg(256)  // Large soft body
    ->Arg(512)  // Very large soft body
    ->Arg(1024) // Cloth / deformable
    ->Unit(benchmark::kNanosecond);

// =============================================================================
// SIMD: Batched cross-product accumulation via SoA EigenSoA3
// Processes 4 point masses at a time
// =============================================================================

static void BM_CrossAccum_SIMD(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto positions = generatePoints(n);
  auto forces = generatePoints(n);
  Eigen::Vector3d center(0.0, 0.0, 0.0);

  for (auto _ : state) {
    using V4 = dart::simd::Vec<double, 4>;
    V4 torque_x = V4::zero();
    V4 torque_y = V4::zero();
    V4 torque_z = V4::zero();

    std::size_t i = 0;
    for (; i + 4 <= n; i += 4) {
      std::array<Eigen::Vector3d, 4> dp, fv;
      for (int k = 0; k < 4; ++k) {
        dp[k] = positions[i + k] - center;
        fv[k] = forces[i + k];
      }

      auto soa_dp = dart::simd::transposeAosToSoa(dp);
      auto soa_f = dart::simd::transposeAosToSoa(fv);
      auto cross = dart::simd::cross3(soa_dp, soa_f);

      torque_x = torque_x + cross.x;
      torque_y = torque_y + cross.y;
      torque_z = torque_z + cross.z;
    }

    // Horizontal sum
    alignas(32) double tx[4], ty[4], tz[4];
    torque_x.store(tx);
    torque_y.store(ty);
    torque_z.store(tz);
    Eigen::Vector3d torque(
        tx[0] + tx[1] + tx[2] + tx[3],
        ty[0] + ty[1] + ty[2] + ty[3],
        tz[0] + tz[1] + tz[2] + tz[3]);

    // Handle remainder
    for (; i < n; ++i) {
      torque += (positions[i] - center).cross(forces[i]);
    }

    benchmark::DoNotOptimize(torque.data());
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(n));
}
BENCHMARK(BM_CrossAccum_SIMD)
    ->Repetitions(10)
    ->ReportAggregatesOnly(true)
    ->MinTime(1.0)
    ->Arg(16)
    ->Arg(64)
    ->Arg(128)
    ->Arg(256)
    ->Arg(512)
    ->Arg(1024)
    ->Unit(benchmark::kNanosecond);

} // namespace
