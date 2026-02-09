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

#if DART_HAVE_OCTOMAP

using namespace dart;
using namespace dynamics;

//==============================================================================
TEST(VoxelGridShape, BasicProperties)
{
  VoxelGridShape shape(0.05);
  EXPECT_EQ(shape.getType(), VoxelGridShape::getStaticType());
  EXPECT_DOUBLE_EQ(shape.getOctree()->getResolution(), 0.05);

  auto octree = std::make_shared<octomap::OcTree>(0.1);
  shape.setOctree(octree);
  EXPECT_EQ(shape.getOctree(), octree);
  EXPECT_EQ(static_cast<const VoxelGridShape*>(&shape)->getOctree(), octree);

  // Test null octree assignment (should be ignored as per .cpp)
  shape.setOctree(nullptr);
  EXPECT_EQ(shape.getOctree(), octree);
}

//==============================================================================
TEST(VoxelGridShape, ConstructorWithNullOctree)
{
  VoxelGridShape shape(std::shared_ptr<octomap::OcTree>{});
  ASSERT_NE(shape.getOctree(), nullptr);
  EXPECT_DOUBLE_EQ(shape.getOctree()->getResolution(), 0.01);
}

//==============================================================================
TEST(VoxelGridShape, SetOctreeSamePointer)
{
  VoxelGridShape shape(0.05);
  auto octree = std::make_shared<octomap::OcTree>(0.2);
  shape.setOctree(octree);
  shape.setOctree(octree);
  EXPECT_EQ(shape.getOctree(), octree);
}

//==============================================================================
TEST(VoxelGridShape, Occupancy)
{
  VoxelGridShape shape(0.1);
  Eigen::Vector3d p(0.5, 0.5, 0.5);

  // Initial occupancy
  double initial = shape.getOccupancy(p);

  shape.updateOccupancy(p, true);
  double occupied = shape.getOccupancy(p);
  EXPECT_GT(occupied, initial);

  shape.updateOccupancy(p, false);
  EXPECT_LT(shape.getOccupancy(p), occupied);
}

//==============================================================================
TEST(VoxelGridShape, RayUpdate)
{
  VoxelGridShape shape(0.1);
  Eigen::Vector3d from(0, 0, 0);
  Eigen::Vector3d to(1, 0, 0);

  shape.updateOccupancy(from, to);

  // Endpoint should be more likely occupied
  EXPECT_GT(shape.getOccupancy(to), 0.5);
  // Midpoint should be more likely free
  EXPECT_LT(shape.getOccupancy(Eigen::Vector3d(0.5, 0, 0)), 0.5);
}

//==============================================================================
TEST(VoxelGridShape, PointCloudUpdate)
{
  VoxelGridShape shape(0.1);
  octomap::Pointcloud pc;
  pc.push_back(1.0, 0.0, 0.0);
  pc.push_back(0.0, 1.0, 0.0);

  shape.updateOccupancy(pc, Eigen::Vector3d::Zero());

  EXPECT_GT(shape.getOccupancy(Eigen::Vector3d(1, 0, 0)), 0.5);
  EXPECT_GT(shape.getOccupancy(Eigen::Vector3d(0, 1, 0)), 0.5);
}

//==============================================================================
TEST(VoxelGridShape, PointCloudUpdateWithFrame)
{
  VoxelGridShape shape(0.1);
  octomap::Pointcloud pc;
  pc.push_back(0.2, 0.0, 0.0);

  SimpleFrame frame(Frame::World(), "sensor_frame");
  frame.setTransform(Eigen::Isometry3d::Identity());

  shape.updateOccupancy(pc, Eigen::Vector3d::Zero(), &frame);

  EXPECT_GT(shape.getOccupancy(Eigen::Vector3d(0.2, 0.0, 0.0)), 0.0);
  EXPECT_DOUBLE_EQ(shape.getOccupancy(Eigen::Vector3d(2.0, 2.0, 2.0)), 0.0);
}

//==============================================================================
TEST(VoxelGridShape, Clone)
{
  VoxelGridShape shape(0.1);
  shape.updateOccupancy(Eigen::Vector3d(1, 1, 1), true);

  auto cloned = std::dynamic_pointer_cast<VoxelGridShape>(shape.clone());
  ASSERT_TRUE(cloned != nullptr);
  EXPECT_DOUBLE_EQ(cloned->getOctree()->getResolution(), 0.1);
  EXPECT_DOUBLE_EQ(
      cloned->getOccupancy(Eigen::Vector3d(1, 1, 1)),
      shape.getOccupancy(Eigen::Vector3d(1, 1, 1)));
}

//==============================================================================
TEST(VoxelGridShape, InertiaAndVolume)
{
  VoxelGridShape shape(0.1);
  // Current implementation returns Identity
  EXPECT_TRUE(shape.computeInertia(1.0).isApprox(Eigen::Matrix3d::Identity()));

  // Trigger updateBoundingBox and updateVolume coverage
  // These are protected and called by Shape when needed
  shape.getBoundingBox();
}

#endif
