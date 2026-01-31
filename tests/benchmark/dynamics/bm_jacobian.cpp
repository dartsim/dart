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

/// @file bm_jacobian.cpp
/// @brief Benchmarks for AdInvTJac block-matrix optimization.
///
/// Compares the old per-column AdInvT loop against the new block-matrix
/// implementation that replaces N separate (3×3 × 3×1) multiplies with
/// two (3×3 × 3×N) matrix multiplies.
///
/// Run:
///   ./bm_jacobian --benchmark_counters_tabular=true
///
/// Expected result: 1.5×–2.0× speedup across all DOF counts.

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

Eigen::Matrix<double, 6, Eigen::Dynamic> randomJacobian(int cols)
{
  std::uniform_real_distribution<double> dist(-1.0, 1.0);
  Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, cols);
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < cols; ++j) {
      J(i, j) = dist(rng());
    }
  }
  return J;
}

// =============================================================================
// AdInvTJac: Per-column baseline (old implementation)
// Applies AdInvT to each column individually: O(N) separate 6×1 transforms
// =============================================================================

static void BM_AdInvTJac_PerColumn(benchmark::State& state)
{
  const int cols = state.range(0);
  auto T = randomTransform();
  auto J = randomJacobian(cols);

  for (auto _ : state) {
    Eigen::Matrix<double, 6, Eigen::Dynamic> ret(6, cols);
    for (int i = 0; i < cols; ++i) {
      ret.col(i) = dart::math::AdInvT(T, J.col(i));
    }
    benchmark::DoNotOptimize(ret.data());
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(cols));
}
BENCHMARK(BM_AdInvTJac_PerColumn)
    ->Repetitions(10)
    ->ReportAggregatesOnly(true)
    ->MinTime(1.0)
    ->Arg(6)  // 6-DOF: typical robot arm
    ->Arg(7)  // 7-DOF: redundant arm (Franka, Kuka iiwa)
    ->Arg(12) // 12-DOF: bimanual system
    ->Arg(18) // 18-DOF: humanoid upper body
    ->Arg(30) // 30-DOF: full humanoid
    ->Arg(50) // 50-DOF: high-DOF system (snake robot, soft robot)
    ->Unit(benchmark::kNanosecond);

// =============================================================================
// AdInvTJac: Block-matrix optimized (new implementation)
// Two (3×3 × 3×N) matrix multiplies instead of N separate (3×3 × 3×1) calls
// =============================================================================

static void BM_AdInvTJac_Block(benchmark::State& state)
{
  const int cols = state.range(0);
  auto T = randomTransform();
  auto J = randomJacobian(cols);

  for (auto _ : state) {
    auto ret = dart::math::AdInvTJac(T, J);
    benchmark::DoNotOptimize(ret.data());
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(cols));
}
BENCHMARK(BM_AdInvTJac_Block)
    ->Repetitions(10)
    ->ReportAggregatesOnly(true)
    ->MinTime(1.0)
    ->Arg(6)
    ->Arg(7)
    ->Arg(12)
    ->Arg(18)
    ->Arg(30)
    ->Arg(50)
    ->Unit(benchmark::kNanosecond);

} // namespace
