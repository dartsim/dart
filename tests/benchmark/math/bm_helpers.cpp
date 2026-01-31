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

/// @file bm_helpers.cpp
/// @brief Benchmarks for isNan() vectorized optimization.
///
/// Compares the old per-element loop against the new Eigen .array().isNaN()
/// vectorized implementation. The optimization targets the typical robotics
/// matrix sizes (3×3 joint rotations, 6×6 spatial inertias, etc.).
///
/// Run:
///   ./bm_helpers --benchmark_counters_tabular=true
///
/// Expected result: 1.2×–1.3× speedup at small sizes (3×3, 6×6).

#include <dart/math/helpers.hpp>

#include <Eigen/Core>
#include <benchmark/benchmark.h>

#include <random>

#include <cmath>

namespace {

Eigen::MatrixXd generateRandomMatrix(int rows, int cols, int seed = 42)
{
  std::mt19937 gen(seed);
  std::uniform_real_distribution<double> dist(-100.0, 100.0);
  Eigen::MatrixXd m(rows, cols);
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      m(i, j) = dist(gen);
    }
  }
  return m;
}

// =============================================================================
// isNan Baseline: old per-element loop
// =============================================================================

static void BM_isNan_Baseline(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  Eigen::MatrixXd m = generateRandomMatrix(n, n);

  for (auto _ : state) {
    bool result = false;
    for (int i = 0; i < m.rows(); ++i) {
      for (int j = 0; j < m.cols(); ++j) {
        if (std::isnan(m(i, j))) {
          result = true;
          break;
        }
      }
    }
    benchmark::DoNotOptimize(result);
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(n * n));
}
BENCHMARK(BM_isNan_Baseline)
    ->Repetitions(10)
    ->ReportAggregatesOnly(true)
    ->MinTime(1.0)
    ->Arg(3)  // 3×3: joint rotation matrix
    ->Arg(4)  // 4×4: homogeneous transform
    ->Arg(6)  // 6×6: spatial inertia
    ->Arg(12) // 12×12: bimanual spatial
    ->Arg(50) // 50×50: large system matrix
    ->Unit(benchmark::kNanosecond);

// =============================================================================
// isNan Optimized: Eigen .array().isNaN().any() vectorized
// =============================================================================

static void BM_isNan_Optimized(benchmark::State& state)
{
  const int n = static_cast<int>(state.range(0));
  Eigen::MatrixXd m = generateRandomMatrix(n, n);

  for (auto _ : state) {
    bool result = dart::math::isNan(m);
    benchmark::DoNotOptimize(result);
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(n * n));
}
BENCHMARK(BM_isNan_Optimized)
    ->Repetitions(10)
    ->ReportAggregatesOnly(true)
    ->MinTime(1.0)
    ->Arg(3)
    ->Arg(4)
    ->Arg(6)
    ->Arg(12)
    ->Arg(50)
    ->Unit(benchmark::kNanosecond);

} // namespace
