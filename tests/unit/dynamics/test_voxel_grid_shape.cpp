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

#include <dart/config.hpp>

#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/voxel_grid_shape.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

using namespace dart;
using namespace dynamics;

//==============================================================================
TEST(VoxelGridShape, BasicProperties)
{
  VoxelGridShape shape(0.05);
  EXPECT_EQ(shape.getType(), VoxelGridShape::getStaticType());
  ASSERT_NE(shape.getOccupancyGrid(), nullptr);
  EXPECT_DOUBLE_EQ(shape.getOccupancyGrid()->getResolution(), 0.05);

  auto grid = std::make_shared<SparseOccupancyGrid>(0.1);
  shape.setOccupancyGrid(grid);
  EXPECT_EQ(shape.getOccupancyGrid(), grid);
  EXPECT_EQ(
      static_cast<const VoxelGridShape*>(&shape)->getOccupancyGrid(), grid);

  shape.setOccupancyGrid(nullptr);
  EXPECT_EQ(shape.getOccupancyGrid(), grid);

#if DART_HAVE_OCTOMAP
  EXPECT_DOUBLE_EQ(shape.getOctree()->getResolution(), 0.1);

  auto octree = std::make_shared<octomap::OcTree>(0.2);
  shape.setOctree(octree);
  EXPECT_EQ(shape.getOctree(), octree);
  EXPECT_EQ(static_cast<const VoxelGridShape*>(&shape)->getOctree(), octree);

  shape.setOctree(nullptr);
  EXPECT_EQ(shape.getOctree(), octree);
#endif
}

//==============================================================================
TEST(VoxelGridShape, ConstructorWithNullNativeStorage)
{
  VoxelGridShape shape(std::shared_ptr<SparseOccupancyGrid>{});
  ASSERT_NE(shape.getOccupancyGrid(), nullptr);
  EXPECT_DOUBLE_EQ(shape.getOccupancyGrid()->getResolution(), 0.01);
}

//==============================================================================
TEST(VoxelGridShape, NotifyOccupancyGridUpdatedRefreshesShapeCaches)
{
  VoxelGridShape shape(0.1);
  (void)shape.getBoundingBox();
  (void)shape.getVolume();

  auto grid = shape.getOccupancyGrid();
  ASSERT_NE(grid, nullptr);
  grid->setOccupancy(SparseOccupancyGrid::CellKey{2, 0, 0}, 0.9);
  shape.notifyOccupancyGridUpdated();

  const auto& boundingBox = shape.getBoundingBox();
  EXPECT_TRUE(boundingBox.getMin().isApprox(Eigen::Vector3d(0.2, 0.0, 0.0)));
  EXPECT_TRUE(boundingBox.getMax().isApprox(Eigen::Vector3d(0.3, 0.1, 0.1)));
  EXPECT_NEAR(shape.getVolume(), 0.001, 1e-15);
}

#if DART_HAVE_OCTOMAP
//==============================================================================
TEST(VoxelGridShape, ConstructorWithNullOctree)
{
  VoxelGridShape shape(std::shared_ptr<octomap::OcTree>{});
  ASSERT_NE(shape.getOctree(), nullptr);
  ASSERT_NE(shape.getOccupancyGrid(), nullptr);
  EXPECT_DOUBLE_EQ(shape.getOctree()->getResolution(), 0.01);
  EXPECT_DOUBLE_EQ(shape.getOccupancyGrid()->getResolution(), 0.01);
}

//==============================================================================
TEST(VoxelGridShape, SetOctreeImportsNativeStorage)
{
  VoxelGridShape shape(0.05);
  auto octree = std::make_shared<octomap::OcTree>(0.2);
  octree->updateNode(0.4, 0.0, 0.0, true);

  shape.setOctree(octree);
  shape.setOctree(octree);

  EXPECT_EQ(shape.getOctree(), octree);
  EXPECT_DOUBLE_EQ(shape.getOccupancyGrid()->getResolution(), 0.2);
  EXPECT_GT(shape.getOccupancy(Eigen::Vector3d(0.4, 0.0, 0.0)), 0.5);
}
#endif

//==============================================================================
TEST(VoxelGridShape, Occupancy)
{
  VoxelGridShape shape(0.1);
  const Eigen::Vector3d point(0.5, 0.5, 0.5);

  const double initial = shape.getOccupancy(point);

  shape.updateOccupancy(point, true);
  const double occupied = shape.getOccupancy(point);
  EXPECT_GT(occupied, initial);

  shape.updateOccupancy(point, false);
  EXPECT_LT(shape.getOccupancy(point), occupied);
}

//==============================================================================
TEST(VoxelGridShape, RayUpdate)
{
  VoxelGridShape shape(0.1);
  const Eigen::Vector3d from(0.05, 0.05, 0.05);
  const Eigen::Vector3d to(0.95, 0.05, 0.05);

  shape.updateOccupancy(from, to);

  EXPECT_GT(shape.getOccupancy(to), 0.5);
  EXPECT_LT(shape.getOccupancy(Eigen::Vector3d(0.55, 0.05, 0.05)), 0.5);
}

//==============================================================================
TEST(VoxelGridShape, PointCloudUpdate)
{
  VoxelGridShape shape(0.1);
  const std::vector<Eigen::Vector3d> points{
      Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(0.0, 1.0, 0.0)};

  shape.updateOccupancy(
      std::span<const Eigen::Vector3d>(points.data(), points.size()),
      Eigen::Vector3d::Zero());

  EXPECT_GT(shape.getOccupancy(Eigen::Vector3d(1.0, 0.0, 0.0)), 0.5);
  EXPECT_GT(shape.getOccupancy(Eigen::Vector3d(0.0, 1.0, 0.0)), 0.5);
}

//==============================================================================
TEST(VoxelGridShape, PointCloudUpdateWithFrame)
{
  VoxelGridShape shape(0.1);
  const std::vector<Eigen::Vector3d> points{Eigen::Vector3d(0.2, 0.0, 0.0)};

  SimpleFrame frame(Frame::World(), "sensor_frame");
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  frame.setTransform(transform);

  shape.updateOccupancy(
      std::span<const Eigen::Vector3d>(points.data(), points.size()),
      Eigen::Vector3d::Zero(),
      &frame);

  EXPECT_GT(shape.getOccupancy(Eigen::Vector3d(1.2, 2.0, 3.0)), 0.5);
  EXPECT_DOUBLE_EQ(shape.getOccupancy(Eigen::Vector3d(2.0, 2.0, 2.0)), 0.0);
}

#if DART_HAVE_OCTOMAP
//==============================================================================
TEST(VoxelGridShape, OctomapPointCloudUpdate)
{
  VoxelGridShape shape(0.1);
  octomap::Pointcloud pointCloud;
  pointCloud.push_back(1.0, 0.0, 0.0);
  pointCloud.push_back(0.0, 1.0, 0.0);

  shape.updateOccupancy(pointCloud, Eigen::Vector3d::Zero());

  EXPECT_GT(shape.getOccupancy(Eigen::Vector3d(1.0, 0.0, 0.0)), 0.5);
  EXPECT_GT(shape.getOccupancy(Eigen::Vector3d(0.0, 1.0, 0.0)), 0.5);
}
#endif

//==============================================================================
TEST(VoxelGridShape, Clone)
{
  VoxelGridShape shape(0.1);
  shape.updateOccupancy(Eigen::Vector3d(1.0, 1.0, 1.0), true);

  auto cloned = std::dynamic_pointer_cast<VoxelGridShape>(shape.clone());
  ASSERT_TRUE(cloned != nullptr);
  EXPECT_DOUBLE_EQ(cloned->getOccupancyGrid()->getResolution(), 0.1);
  EXPECT_DOUBLE_EQ(
      cloned->getOccupancy(Eigen::Vector3d(1.0, 1.0, 1.0)),
      shape.getOccupancy(Eigen::Vector3d(1.0, 1.0, 1.0)));

#if DART_HAVE_OCTOMAP
  EXPECT_DOUBLE_EQ(cloned->getOctree()->getResolution(), 0.1);
#endif
}

//==============================================================================
TEST(VoxelGridShape, InertiaBoundingBoxAndVolume)
{
  VoxelGridShape shape(0.1);
  EXPECT_TRUE(shape.computeInertia(1.0).isApprox(Eigen::Matrix3d::Identity()));

  shape.updateOccupancy(Eigen::Vector3d(1.0, 1.0, 1.0), true);

  const auto& boundingBox = shape.getBoundingBox();
  EXPECT_TRUE(boundingBox.getMin().isApprox(Eigen::Vector3d(1.0, 1.0, 1.0)));
  EXPECT_TRUE(boundingBox.getMax().isApprox(Eigen::Vector3d(1.1, 1.1, 1.1)));
  EXPECT_NEAR(shape.getVolume(), 0.001, 1e-15);
}
