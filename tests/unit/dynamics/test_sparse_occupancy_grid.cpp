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

#include <dart/dynamics/sparse_occupancy_grid.hpp>

#include <dart/common/exception.hpp>

#include <gtest/gtest.h>

#include <array>
#include <vector>

#ifndef DART_TESTS_HAVE_OCTOMAP
  #define DART_TESTS_HAVE_OCTOMAP 0
#endif

#if DART_TESTS_HAVE_OCTOMAP
  #include <octomap/octomap.h>
#endif

using namespace dart;
using namespace dynamics;

//==============================================================================
TEST(SparseOccupancyGrid, ConstructionAndCoordinates)
{
  EXPECT_THROW(SparseOccupancyGrid(0.0), common::InvalidArgumentException);
  EXPECT_THROW(SparseOccupancyGrid(-0.1), common::InvalidArgumentException);

  SparseOccupancyGrid grid(0.25);
  EXPECT_DOUBLE_EQ(grid.getResolution(), 0.25);

  const auto key = grid.worldToCell(Eigen::Vector3d(0.0, 0.24, -0.01));
  EXPECT_TRUE((key == SparseOccupancyGrid::CellKey{0, 0, -1}));

  const auto center
      = grid.getCellCenter(SparseOccupancyGrid::CellKey{2, -1, 0});
  EXPECT_TRUE(center.isApprox(Eigen::Vector3d(0.625, -0.125, 0.125)));
}

//==============================================================================
TEST(SparseOccupancyGrid, ProbabilityUpdates)
{
  SparseOccupancyGrid grid(0.1);
  const Eigen::Vector3d point(0.12, 0.0, 0.0);
  const auto key = grid.worldToCell(point);

  EXPECT_DOUBLE_EQ(grid.getOccupancy(point), 0.0);
  EXPECT_FALSE(grid.isOccupied(key));

  grid.updateOccupancy(point, true);
  const double occupied = grid.getOccupancy(point);
  EXPECT_GT(occupied, 0.5);
  EXPECT_TRUE(grid.isOccupied(key));

  grid.updateOccupancy(point, false);
  EXPECT_LT(grid.getOccupancy(point), occupied);

  grid.setOccupancy(key, 0.2);
  EXPECT_DOUBLE_EQ(grid.getOccupancy(key), 0.2);
  EXPECT_FALSE(grid.isOccupied(key));

  EXPECT_THROW(grid.setOccupancy(key, 1.0), common::InvalidArgumentException);
}

//==============================================================================
TEST(SparseOccupancyGrid, RayUpdatesAndTracing)
{
  SparseOccupancyGrid grid(0.1);
  const Eigen::Vector3d from(0.05, 0.05, 0.05);
  const Eigen::Vector3d to(0.95, 0.05, 0.05);

  const auto cells = grid.traceRay(from, to);
  ASSERT_EQ(cells.size(), 10u);
  EXPECT_TRUE((cells.front() == SparseOccupancyGrid::CellKey{0, 0, 0}));
  EXPECT_TRUE((cells.back() == SparseOccupancyGrid::CellKey{9, 0, 0}));

  grid.insertRay(from, to);
  EXPECT_LT(grid.getOccupancy(from), 0.5);
  EXPECT_LT(grid.getOccupancy(Eigen::Vector3d(0.55, 0.05, 0.05)), 0.5);
  EXPECT_GT(grid.getOccupancy(to), 0.5);
}

//==============================================================================
TEST(SparseOccupancyGrid, PointCloudUpdates)
{
  SparseOccupancyGrid grid(0.1);
  const std::array<Eigen::Vector3d, 2> points{
      Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(0.0, 1.0, 0.0)};

  grid.insertPointCloud(points);

  EXPECT_GT(grid.getOccupancy(Eigen::Vector3d(1.0, 0.0, 0.0)), 0.5);
  EXPECT_GT(grid.getOccupancy(Eigen::Vector3d(0.0, 1.0, 0.0)), 0.5);

  Eigen::Isometry3d relativeTo = Eigen::Isometry3d::Identity();
  relativeTo.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

  SparseOccupancyGrid transformedGrid(0.1);
  transformedGrid.insertPointCloud(
      std::span<const Eigen::Vector3d>(points.data(), 1),
      Eigen::Vector3d::Zero(),
      relativeTo);

  EXPECT_GT(transformedGrid.getOccupancy(Eigen::Vector3d(2.0, 2.0, 3.0)), 0.5);
  EXPECT_LT(transformedGrid.getOccupancy(Eigen::Vector3d(1.5, 2.0, 3.0)), 0.5);
}

//==============================================================================
TEST(SparseOccupancyGrid, OccupiedCellExtractionAndCopy)
{
  SparseOccupancyGrid grid(0.1);
  const SparseOccupancyGrid::CellKey occupiedKey{0, 0, 0};
  const SparseOccupancyGrid::CellKey freeKey{1, 0, 0};
  grid.setOccupancy(occupiedKey, 0.9);
  grid.setOccupancy(freeKey, 0.2);

  const auto cells = grid.getOccupiedCells();
  ASSERT_EQ(cells.size(), 1u);
  EXPECT_TRUE((cells.front().key == occupiedKey));
  EXPECT_TRUE(cells.front().center.isApprox(Eigen::Vector3d(0.05, 0.05, 0.05)));
  EXPECT_DOUBLE_EQ(cells.front().size, 0.1);
  EXPECT_DOUBLE_EQ(cells.front().occupancy, 0.9);

  const SparseOccupancyGrid copied = grid;
  grid.clear();
  EXPECT_EQ(grid.getNumCells(), 0u);
  EXPECT_EQ(copied.getNumCells(), 2u);
  EXPECT_EQ(copied.getNumOccupiedCells(), 1u);
}

#if DART_TESTS_HAVE_OCTOMAP
//==============================================================================
TEST(SparseOccupancyGrid, PointCloudMatchesOctomapOccupiedCells)
{
  constexpr double resolution = 0.1;
  const std::vector<Eigen::Vector3d> points{
      Eigen::Vector3d(0.5, 0.0, 0.0),
      Eigen::Vector3d(0.0, 0.5, 0.0),
      Eigen::Vector3d(0.5, 0.5, 0.0)};

  SparseOccupancyGrid grid(resolution);
  grid.insertPointCloud(points, Eigen::Vector3d::Zero());

  octomap::Pointcloud pointCloud;
  for (const auto& point : points) {
    pointCloud.push_back(
        static_cast<float>(point.x()),
        static_cast<float>(point.y()),
        static_cast<float>(point.z()));
  }

  octomap::OcTree octree(resolution);
  octree.insertPointCloud(pointCloud, octomap::point3d(0.0, 0.0, 0.0));

  std::size_t occupiedCells = 0;
  for (auto it = octree.begin_leafs(), end = octree.end_leafs(); it != end;
       ++it) {
    if (!octree.isNodeOccupied(*it)) {
      continue;
    }

    ++occupiedCells;
    EXPECT_TRUE(grid.isOccupied(
        grid.worldToCell(Eigen::Vector3d(it.getX(), it.getY(), it.getZ()))));
  }

  EXPECT_EQ(grid.getNumOccupiedCells(), occupiedCells);
}
#endif
