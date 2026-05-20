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

/// @file bm_sparse_occupancy_grid.cpp
/// @brief Native sparse occupancy-grid microbenchmarks.
///
/// Run:
///   ./bm_sparse_occupancy_grid --benchmark_counters_tabular=true

#include <dart/config.hpp>

#include <dart/dynamics/sparse_occupancy_grid.hpp>

#include <benchmark/benchmark.h>

#include <span>
#include <vector>

#include <cstdint>

#ifndef DART_TESTS_HAVE_OCTOMAP
  #define DART_TESTS_HAVE_OCTOMAP 0
#endif

#if DART_TESTS_HAVE_OCTOMAP
  #include <octomap/octomap.h>
#endif

namespace {

//==============================================================================
std::vector<Eigen::Vector3d> makePointCloud(std::size_t count)
{
  std::vector<Eigen::Vector3d> points;
  points.reserve(count);

  for (std::size_t i = 0; i < count; ++i) {
    const double x = 0.05 * static_cast<double>(i % 64u);
    const double y = 0.05 * static_cast<double>((i / 64u) % 64u);
    const double z = 0.05 * static_cast<double>((i / 4096u) % 16u);
    points.emplace_back(x, y, z);
  }

  return points;
}

//==============================================================================
std::span<const Eigen::Vector3d> asSpan(
    const std::vector<Eigen::Vector3d>& points)
{
  return std::span<const Eigen::Vector3d>(points.data(), points.size());
}

//==============================================================================
static void BM_NativePointUpdates(benchmark::State& state)
{
  const auto points = makePointCloud(static_cast<std::size_t>(state.range(0)));
  dart::dynamics::SparseOccupancyGrid grid(0.05);

  for (auto _ : state) {
    grid.clear();
    for (const auto& point : points) {
      grid.updateOccupancy(point, true);
    }
    benchmark::DoNotOptimize(grid.getNumOccupiedCells());
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<std::int64_t>(points.size()));
}
BENCHMARK(BM_NativePointUpdates)
    ->Arg(128)
    ->Arg(1024)
    ->Arg(8192)
    ->Unit(benchmark::kMicrosecond);

//==============================================================================
static void BM_NativeRayInsertions(benchmark::State& state)
{
  const auto points = makePointCloud(static_cast<std::size_t>(state.range(0)));
  dart::dynamics::SparseOccupancyGrid grid(0.05);
  const Eigen::Vector3d sensorOrigin = Eigen::Vector3d::Zero();

  for (auto _ : state) {
    grid.clear();
    for (const auto& point : points) {
      grid.insertRay(sensorOrigin, point);
    }
    benchmark::DoNotOptimize(grid.getNumCells());
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<std::int64_t>(points.size()));
}
BENCHMARK(BM_NativeRayInsertions)
    ->Arg(128)
    ->Arg(1024)
    ->Arg(8192)
    ->Unit(benchmark::kMicrosecond);

//==============================================================================
static void BM_NativePointCloudInsert(benchmark::State& state)
{
  const auto points = makePointCloud(static_cast<std::size_t>(state.range(0)));
  dart::dynamics::SparseOccupancyGrid grid(0.05);

  for (auto _ : state) {
    grid.clear();
    grid.insertPointCloud(asSpan(points), Eigen::Vector3d::Zero());
    benchmark::DoNotOptimize(grid.getNumCells());
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<std::int64_t>(points.size()));
}
BENCHMARK(BM_NativePointCloudInsert)
    ->Arg(128)
    ->Arg(1024)
    ->Arg(8192)
    ->Unit(benchmark::kMicrosecond);

//==============================================================================
static void BM_NativePointCloudInsertThreaded(benchmark::State& state)
{
  const auto points = makePointCloud(static_cast<std::size_t>(state.range(0)));
  const auto numThreads = static_cast<std::size_t>(state.range(1));
  dart::dynamics::SparseOccupancyGrid grid(0.05);

  for (auto _ : state) {
    grid.clear();
    grid.insertPointCloud(
        asSpan(points),
        Eigen::Vector3d::Zero(),
        Eigen::Isometry3d::Identity(),
        numThreads);
    benchmark::DoNotOptimize(grid.getNumCells());
  }

  state.counters["threads"]
      = benchmark::Counter(static_cast<double>(numThreads));
  state.SetItemsProcessed(
      state.iterations() * static_cast<std::int64_t>(points.size()));
}
BENCHMARK(BM_NativePointCloudInsertThreaded)
    ->Args({1024, 2})
    ->Args({8192, 2})
    ->Args({8192, 4})
    ->Args({8192, 8})
    ->Unit(benchmark::kMicrosecond);

//==============================================================================
static void BM_NativeOccupancyQueries(benchmark::State& state)
{
  const auto points = makePointCloud(static_cast<std::size_t>(state.range(0)));
  dart::dynamics::SparseOccupancyGrid grid(0.05);
  for (const auto& point : points) {
    grid.updateOccupancy(point, true);
  }

  for (auto _ : state) {
    double occupancy = 0.0;
    for (const auto& point : points) {
      occupancy += grid.getOccupancy(point);
    }
    benchmark::DoNotOptimize(occupancy);
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<std::int64_t>(points.size()));
}
BENCHMARK(BM_NativeOccupancyQueries)
    ->Arg(128)
    ->Arg(1024)
    ->Arg(8192)
    ->Unit(benchmark::kMicrosecond);

//==============================================================================
static void BM_NativeOccupiedCellExtraction(benchmark::State& state)
{
  const auto points = makePointCloud(static_cast<std::size_t>(state.range(0)));
  dart::dynamics::SparseOccupancyGrid grid(0.05);
  for (const auto& point : points) {
    grid.updateOccupancy(point, true);
  }

  for (auto _ : state) {
    const auto cells = grid.getOccupiedCells();
    benchmark::DoNotOptimize(cells.data());
    benchmark::DoNotOptimize(cells.size());
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<std::int64_t>(points.size()));
}
BENCHMARK(BM_NativeOccupiedCellExtraction)
    ->Arg(128)
    ->Arg(1024)
    ->Arg(8192)
    ->Unit(benchmark::kMicrosecond);

#if DART_TESTS_HAVE_OCTOMAP
//==============================================================================
octomap::point3d toPoint3d(const Eigen::Vector3d& point)
{
  return octomap::point3d(
      static_cast<float>(point.x()),
      static_cast<float>(point.y()),
      static_cast<float>(point.z()));
}

//==============================================================================
static void BM_ReferencePointUpdates(benchmark::State& state)
{
  const auto points = makePointCloud(static_cast<std::size_t>(state.range(0)));

  for (auto _ : state) {
    octomap::OcTree tree(0.05);
    for (const auto& point : points) {
      tree.updateNode(toPoint3d(point), true);
    }
    benchmark::DoNotOptimize(tree.size());
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<std::int64_t>(points.size()));
}
BENCHMARK(BM_ReferencePointUpdates)
    ->Arg(128)
    ->Arg(1024)
    ->Arg(8192)
    ->Unit(benchmark::kMicrosecond);

//==============================================================================
static void BM_ReferenceRayInsertions(benchmark::State& state)
{
  const auto points = makePointCloud(static_cast<std::size_t>(state.range(0)));
  const octomap::point3d sensorOrigin(0.0, 0.0, 0.0);

  for (auto _ : state) {
    octomap::OcTree tree(0.05);
    for (const auto& point : points) {
      tree.insertRay(sensorOrigin, toPoint3d(point));
    }
    benchmark::DoNotOptimize(tree.size());
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<std::int64_t>(points.size()));
}
BENCHMARK(BM_ReferenceRayInsertions)
    ->Arg(128)
    ->Arg(1024)
    ->Arg(8192)
    ->Unit(benchmark::kMicrosecond);

//==============================================================================
static void BM_ReferencePointCloudInsert(benchmark::State& state)
{
  const auto points = makePointCloud(static_cast<std::size_t>(state.range(0)));
  octomap::Pointcloud pointCloud;
  for (const auto& point : points) {
    pointCloud.push_back(toPoint3d(point));
  }

  for (auto _ : state) {
    octomap::OcTree tree(0.05);
    tree.insertPointCloud(pointCloud, octomap::point3d(0.0, 0.0, 0.0));
    benchmark::DoNotOptimize(tree.size());
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<std::int64_t>(points.size()));
}
BENCHMARK(BM_ReferencePointCloudInsert)
    ->Arg(128)
    ->Arg(1024)
    ->Arg(8192)
    ->Unit(benchmark::kMicrosecond);

//==============================================================================
static void BM_ReferenceOccupancyQueries(benchmark::State& state)
{
  const auto points = makePointCloud(static_cast<std::size_t>(state.range(0)));
  octomap::OcTree tree(0.05);
  for (const auto& point : points) {
    tree.updateNode(toPoint3d(point), true);
  }

  for (auto _ : state) {
    double occupancy = 0.0;
    for (const auto& point : points) {
      const auto* node = tree.search(toPoint3d(point));
      occupancy += node ? node->getOccupancy() : 0.0;
    }
    benchmark::DoNotOptimize(occupancy);
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<std::int64_t>(points.size()));
}
BENCHMARK(BM_ReferenceOccupancyQueries)
    ->Arg(128)
    ->Arg(1024)
    ->Arg(8192)
    ->Unit(benchmark::kMicrosecond);

//==============================================================================
static void BM_ReferenceOccupiedCellExtraction(benchmark::State& state)
{
  const auto points = makePointCloud(static_cast<std::size_t>(state.range(0)));
  octomap::OcTree tree(0.05);
  for (const auto& point : points) {
    tree.updateNode(toPoint3d(point), true);
  }

  for (auto _ : state) {
    std::vector<dart::dynamics::SparseOccupancyGrid::OccupiedCell> cells;
    cells.reserve(tree.getNumLeafNodes());
    for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end;
         ++it) {
      if (tree.isNodeOccupied(*it)) {
        cells.push_back(
            dart::dynamics::SparseOccupancyGrid::OccupiedCell{
                dart::dynamics::SparseOccupancyGrid::CellKey{},
                Eigen::Vector3d(it.getX(), it.getY(), it.getZ()),
                tree.getResolution(),
                it->getOccupancy()});
      }
    }
    benchmark::DoNotOptimize(cells.data());
    benchmark::DoNotOptimize(cells.size());
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<std::int64_t>(points.size()));
}
BENCHMARK(BM_ReferenceOccupiedCellExtraction)
    ->Arg(128)
    ->Arg(1024)
    ->Arg(8192)
    ->Unit(benchmark::kMicrosecond);
#endif

} // namespace
