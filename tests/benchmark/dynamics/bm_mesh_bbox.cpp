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

/// @file bm_mesh_bbox.cpp
/// @brief Benchmarks for mesh bounding-box computation using SIMD.
///
/// Compares three approaches for computing the axis-aligned bounding box
/// of a point cloud (typical mesh vertex processing):
///   1. Eigen cwiseMin/cwiseMax loop (baseline)
///   2. dart::simd::Vector3d per-element min/max
///   3. SoA batch with Vec<double,4> min/max + horizontal reduction
///
/// Run:
///   ./bm_mesh_bbox --benchmark_counters_tabular=true
///
/// Expected result: 2.0×–2.7× speedup for both SIMD variants.

#include <dart/simd/simd.hpp>

#include <Eigen/Core>
#include <benchmark/benchmark.h>

#include <limits>
#include <random>
#include <vector>

namespace {

std::vector<Eigen::Vector3d> generateVertices(std::size_t count, int seed = 42)
{
  std::mt19937 gen(seed);
  std::uniform_real_distribution<double> dist(-100.0, 100.0);
  std::vector<Eigen::Vector3d> verts(count);
  for (auto& v : verts) {
    v = Eigen::Vector3d(dist(gen), dist(gen), dist(gen));
  }
  return verts;
}

// =============================================================================
// Eigen cwiseMin/cwiseMax loop (baseline)
// =============================================================================

static void BM_MeshBBox_Eigen(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto verts = generateVertices(n);

  for (auto _ : state) {
    Eigen::Vector3d bmin
        = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
    Eigen::Vector3d bmax
        = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());
    for (std::size_t i = 0; i < n; ++i) {
      bmin = bmin.cwiseMin(verts[i]);
      bmax = bmax.cwiseMax(verts[i]);
    }
    benchmark::DoNotOptimize(bmin.data());
    benchmark::DoNotOptimize(bmax.data());
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(n));
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(n)
      * static_cast<int64_t>(3 * sizeof(double)));
}
BENCHMARK(BM_MeshBBox_Eigen)
    ->Repetitions(10)
    ->ReportAggregatesOnly(true)
    ->MinTime(1.0)
    ->Arg(256)   // Small mesh (simple collision shape)
    ->Arg(1024)  // Medium mesh
    ->Arg(4096)  // Typical robot visual mesh
    ->Arg(16384) // Detailed mesh
    ->Arg(65536) // High-poly mesh
    ->Unit(benchmark::kNanosecond);

// =============================================================================
// SoA batch: Vec<double,4> min/max + horizontal reduction
// =============================================================================

static void BM_MeshBBox_SoA(benchmark::State& state)
{
  const std::size_t n = state.range(0);
  auto verts = generateVertices(n);

  for (auto _ : state) {
    using V4 = dart::simd::Vec<double, 4>;
    V4 min_x = V4::broadcast(std::numeric_limits<double>::max());
    V4 min_y = V4::broadcast(std::numeric_limits<double>::max());
    V4 min_z = V4::broadcast(std::numeric_limits<double>::max());
    V4 max_x = V4::broadcast(std::numeric_limits<double>::lowest());
    V4 max_y = V4::broadcast(std::numeric_limits<double>::lowest());
    V4 max_z = V4::broadcast(std::numeric_limits<double>::lowest());

    std::size_t i = 0;
    for (; i + 4 <= n; i += 4) {
      std::array<Eigen::Vector3d, 4> batch
          = {verts[i], verts[i + 1], verts[i + 2], verts[i + 3]};
      auto soa = dart::simd::transposeAosToSoa(batch);
      min_x = dart::simd::min(min_x, soa.x);
      min_y = dart::simd::min(min_y, soa.y);
      min_z = dart::simd::min(min_z, soa.z);
      max_x = dart::simd::max(max_x, soa.x);
      max_y = dart::simd::max(max_y, soa.y);
      max_z = dart::simd::max(max_z, soa.z);
    }

    // Horizontal reduction of the 4 SIMD lanes
    alignas(32) double mx[4], my[4], mz[4], Mx[4], My[4], Mz[4];
    min_x.store(mx);
    min_y.store(my);
    min_z.store(mz);
    max_x.store(Mx);
    max_y.store(My);
    max_z.store(Mz);

    double final_min_x = mx[0], final_min_y = my[0], final_min_z = mz[0];
    double final_max_x = Mx[0], final_max_y = My[0], final_max_z = Mz[0];
    for (int k = 1; k < 4; ++k) {
      final_min_x = std::min(final_min_x, mx[k]);
      final_min_y = std::min(final_min_y, my[k]);
      final_min_z = std::min(final_min_z, mz[k]);
      final_max_x = std::max(final_max_x, Mx[k]);
      final_max_y = std::max(final_max_y, My[k]);
      final_max_z = std::max(final_max_z, Mz[k]);
    }

    // Handle remaining vertices
    for (; i < n; ++i) {
      final_min_x = std::min(final_min_x, verts[i][0]);
      final_min_y = std::min(final_min_y, verts[i][1]);
      final_min_z = std::min(final_min_z, verts[i][2]);
      final_max_x = std::max(final_max_x, verts[i][0]);
      final_max_y = std::max(final_max_y, verts[i][1]);
      final_max_z = std::max(final_max_z, verts[i][2]);
    }

    Eigen::Vector3d bmin(final_min_x, final_min_y, final_min_z);
    Eigen::Vector3d bmax(final_max_x, final_max_y, final_max_z);
    benchmark::DoNotOptimize(bmin.data());
    benchmark::DoNotOptimize(bmax.data());
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(n));
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations()) * static_cast<int64_t>(n)
      * static_cast<int64_t>(3 * sizeof(double)));
}
BENCHMARK(BM_MeshBBox_SoA)
    ->Repetitions(10)
    ->ReportAggregatesOnly(true)
    ->MinTime(1.0)
    ->Arg(256)
    ->Arg(1024)
    ->Arg(4096)
    ->Arg(16384)
    ->Arg(65536)
    ->Unit(benchmark::kNanosecond);

} // namespace
