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
#include <limits>
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
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(SparseOccupancyGrid{inf}, common::InvalidArgumentException);
  EXPECT_THROW(SparseOccupancyGrid{nan}, common::InvalidArgumentException);

  SparseOccupancyGrid grid(0.25);
  EXPECT_DOUBLE_EQ(grid.getResolution(), 0.25);

  const auto key = grid.worldToCell(Eigen::Vector3d(0.0, 0.24, -0.01));
  EXPECT_TRUE((key == SparseOccupancyGrid::CellKey{0, 0, -1}));

  const auto center
      = grid.getCellCenter(SparseOccupancyGrid::CellKey{2, -1, 0});
  EXPECT_TRUE(center.isApprox(Eigen::Vector3d(0.625, -0.125, 0.125)));
}

//==============================================================================
TEST(SparseOccupancyGrid, RejectsNonFiniteCoordinates)
{
  SparseOccupancyGrid grid(0.1);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const Eigen::Vector3d finite(0.05, 0.05, 0.05);

  EXPECT_THROW(
      grid.worldToCell(Eigen::Vector3d(nan, 0.0, 0.0)),
      common::InvalidArgumentException);
  EXPECT_THROW(
      grid.getOccupancy(Eigen::Vector3d(0.0, inf, 0.0)),
      common::InvalidArgumentException);
  EXPECT_THROW(
      grid.updateOccupancy(Eigen::Vector3d(0.0, 0.0, -inf)),
      common::InvalidArgumentException);
  EXPECT_THROW(
      grid.insertRay(finite, Eigen::Vector3d(inf, 0.0, 0.0)),
      common::InvalidArgumentException);
  EXPECT_THROW(
      grid.traceRay(Eigen::Vector3d(0.0, nan, 0.0), finite),
      common::InvalidArgumentException);

  const std::array<Eigen::Vector3d, 3> points{
      Eigen::Vector3d(0.25, 0.05, 0.05),
      Eigen::Vector3d(nan, 0.05, 0.05),
      Eigen::Vector3d(0.05, inf, 0.05)};

  grid.insertPointCloud(points, finite);
  EXPECT_EQ(grid.getNumOccupiedCells(), 1u);
  EXPECT_GT(grid.getOccupancy(Eigen::Vector3d(0.25, 0.05, 0.05)), 0.5);

  SparseOccupancyGrid invalidOnlyGrid(0.1);
  const std::array<Eigen::Vector3d, 1> invalidPoints{
      Eigen::Vector3d(nan, 0.0, 0.0)};
  invalidOnlyGrid.insertPointCloud(invalidPoints, finite);
  EXPECT_EQ(invalidOnlyGrid.getNumCells(), 0u);

  EXPECT_THROW(
      grid.insertPointCloud(points, Eigen::Vector3d(0.0, 0.0, inf)),
      common::InvalidArgumentException);
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
  EXPECT_THROW(
      grid.setOccupancy(key, std::numeric_limits<double>::quiet_NaN()),
      common::InvalidArgumentException);

  grid.setOccupancyThreshold(0.6);
  grid.setHitProbability(0.8);
  grid.setMissProbability(0.3);
  EXPECT_DOUBLE_EQ(grid.getOccupancyThreshold(), 0.6);
  EXPECT_DOUBLE_EQ(grid.getHitProbability(), 0.8);
  EXPECT_DOUBLE_EQ(grid.getMissProbability(), 0.3);
  EXPECT_THROW(
      grid.setOccupancyThreshold(0.0), common::InvalidArgumentException);
  EXPECT_THROW(grid.setHitProbability(1.0), common::InvalidArgumentException);
  EXPECT_THROW(
      grid.setMissProbability(std::numeric_limits<double>::infinity()),
      common::InvalidArgumentException);
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

  SparseOccupancyGrid sameCellGrid(0.1);
  const auto sameCellTrace = sameCellGrid.traceRay(from, from);
  ASSERT_EQ(sameCellTrace.size(), 1u);
  EXPECT_TRUE((sameCellTrace.front() == SparseOccupancyGrid::CellKey{0, 0, 0}));

  sameCellGrid.insertRay(from, from);
  EXPECT_EQ(sameCellGrid.getNumCells(), 1u);
  EXPECT_GT(sameCellGrid.getOccupancy(from), 0.5);
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

  SparseOccupancyGrid threadedGrid(0.1);
  threadedGrid.insertPointCloud(
      points, Eigen::Vector3d::Zero(), Eigen::Isometry3d::Identity(), 2);

  EXPECT_EQ(threadedGrid.getNumOccupiedCells(), grid.getNumOccupiedCells());
  EXPECT_GT(threadedGrid.getOccupancy(Eigen::Vector3d(1.0, 0.0, 0.0)), 0.5);
  EXPECT_GT(threadedGrid.getOccupancy(Eigen::Vector3d(0.0, 1.0, 0.0)), 0.5);

  SparseOccupancyGrid autoThreadGrid(0.1);
  autoThreadGrid.insertPointCloud(
      points, Eigen::Vector3d::Zero(), Eigen::Isometry3d::Identity(), 0);
  EXPECT_EQ(autoThreadGrid.getNumOccupiedCells(), grid.getNumOccupiedCells());
}

//==============================================================================
TEST(SparseOccupancyGrid, PointCloudAccumulatesRepeatedObservations)
{
  const Eigen::Vector3d origin(0.05, 0.05, 0.05);
  const Eigen::Vector3d endpoint(0.95, 0.05, 0.05);
  const Eigen::Vector3d freePoint(0.35, 0.05, 0.05);

  SparseOccupancyGrid singleGrid(0.1);
  const std::array<Eigen::Vector3d, 1> singlePoint{endpoint};
  singleGrid.insertPointCloud(singlePoint, origin);

  SparseOccupancyGrid repeatedGrid(0.1);
  const std::array<Eigen::Vector3d, 2> repeatedPoints{endpoint, endpoint};
  repeatedGrid.insertPointCloud(repeatedPoints, origin);

  EXPECT_GT(
      repeatedGrid.getOccupancy(endpoint), singleGrid.getOccupancy(endpoint));
  EXPECT_LT(
      repeatedGrid.getOccupancy(freePoint), singleGrid.getOccupancy(freePoint));

  std::vector<Eigen::Vector3d> densePoints(600, endpoint);
  SparseOccupancyGrid serialGrid(0.1);
  serialGrid.insertPointCloud(
      densePoints, origin, Eigen::Isometry3d::Identity(), 1);

  SparseOccupancyGrid threadedGrid(0.1);
  threadedGrid.insertPointCloud(
      densePoints, origin, Eigen::Isometry3d::Identity(), 4);

  EXPECT_DOUBLE_EQ(
      threadedGrid.getOccupancy(endpoint), serialGrid.getOccupancy(endpoint));
  EXPECT_DOUBLE_EQ(
      threadedGrid.getOccupancy(freePoint), serialGrid.getOccupancy(freePoint));
}

//==============================================================================
TEST(SparseOccupancyGrid, PointCloudEndpointCellsWinOverFreeUpdates)
{
  const Eigen::Vector3d origin(0.05, 0.05, 0.05);
  const Eigen::Vector3d nearEndpoint(0.35, 0.05, 0.05);
  const Eigen::Vector3d farEndpoint(0.75, 0.05, 0.05);

  SparseOccupancyGrid hitOnlyGrid(0.1);
  const std::array<Eigen::Vector3d, 1> nearOnly{nearEndpoint};
  hitOnlyGrid.insertPointCloud(nearOnly, origin);

  SparseOccupancyGrid mixedGrid(0.1);
  const std::array<Eigen::Vector3d, 2> mixedPoints{nearEndpoint, farEndpoint};
  mixedGrid.insertPointCloud(mixedPoints, origin);

  EXPECT_DOUBLE_EQ(
      mixedGrid.getOccupancy(nearEndpoint),
      hitOnlyGrid.getOccupancy(nearEndpoint));
  EXPECT_GT(mixedGrid.getOccupancy(farEndpoint), 0.5);
  EXPECT_LT(mixedGrid.getOccupancy(Eigen::Vector3d(0.25, 0.05, 0.05)), 0.5);
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

  const SparseOccupancyGrid::CellKey secondOccupiedKey{2, 0, 0};
  grid.setOccupancy(secondOccupiedKey, 0.9);
  const SparseOccupancyGrid::CellKey thirdOccupiedKey{0, 1, 0};
  const SparseOccupancyGrid::CellKey fourthOccupiedKey{0, 1, 1};
  grid.setOccupancy(thirdOccupiedKey, 0.9);
  grid.setOccupancy(fourthOccupiedKey, 0.9);
  const auto sortedCells = grid.getOccupiedCells();
  ASSERT_EQ(sortedCells.size(), 4u);
  EXPECT_TRUE((sortedCells[0].key == occupiedKey));
  EXPECT_TRUE((sortedCells[1].key == thirdOccupiedKey));
  EXPECT_TRUE((sortedCells[2].key == fourthOccupiedKey));
  EXPECT_TRUE((sortedCells[3].key == secondOccupiedKey));

  const auto cachedCells = grid.getOccupiedCells();
  ASSERT_EQ(cachedCells.size(), sortedCells.size());
  EXPECT_TRUE((cachedCells[2].key == fourthOccupiedKey));

  grid.setOccupancyThreshold(0.95);
  EXPECT_TRUE(grid.getOccupiedCells().empty());

  const SparseOccupancyGrid copied = grid;
  grid.clear();
  EXPECT_EQ(grid.getNumCells(), 0u);
  EXPECT_EQ(copied.getNumCells(), 5u);
  EXPECT_EQ(copied.getNumOccupiedCells(), 0u);
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
