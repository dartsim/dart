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

#include <dart/dynamics/LineSegmentShape.hpp>
#include <dart/dynamics/PointCloudShape.hpp>

#include <gtest/gtest.h>

#include <vector>

using namespace dart::dynamics;

//==============================================================================
TEST(ShapeEditBounds, LineSegmentVertexEditsRefreshBoundingBox)
{
  LineSegmentShape shape(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX());
  EXPECT_EQ(Eigen::Vector3d::UnitX(), shape.getBoundingBox().getMax());

  shape.addVertex(Eigen::Vector3d(0, 2, 0));
  EXPECT_EQ(2.0, shape.getBoundingBox().getMax().y());

  shape.setVertex(2, Eigen::Vector3d(0, 0, 3));
  EXPECT_EQ(Eigen::Vector3d(1, 0, 3), shape.getBoundingBox().getMax());

  shape.removeVertex(2);
  EXPECT_EQ(Eigen::Vector3d::UnitX(), shape.getBoundingBox().getMax());

  shape.addVertex(Eigen::Vector3d(-4, 0, 0), 0u);
  EXPECT_EQ(Eigen::Vector3d(-4, 0, 0), shape.getBoundingBox().getMin());

  // This constructor goes through Shape(LINE_SEGMENT), which initializes the
  // lazy flags, so querying before the first vertex is well defined.
  LineSegmentShape empty;
  EXPECT_EQ(Eigen::Vector3d::Zero(), empty.getBoundingBox().getMax());
  empty.addVertex(Eigen::Vector3d(5, 0, 0));
  EXPECT_EQ(Eigen::Vector3d(5, 0, 0), empty.getBoundingBox().getMax());
}

//==============================================================================
TEST(ShapeEditBounds, PointCloudEditsRefreshBoundingBoxAndVersion)
{
  PointCloudShape shape;
  shape.addPoint(Eigen::Vector3d(1, 0, 0));
  EXPECT_EQ(Eigen::Vector3d(1, 0, 0), shape.getBoundingBox().getMax());

  shape.addPoint(Eigen::Vector3d(2, 0, 0));
  EXPECT_EQ(Eigen::Vector3d(2, 0, 0), shape.getBoundingBox().getMax());

  shape.addPoint(std::vector<Eigen::Vector3d>{Eigen::Vector3d(0, 2, 0)});
  EXPECT_EQ(Eigen::Vector3d(2, 2, 0), shape.getBoundingBox().getMax());

  shape.setPoint({Eigen::Vector3d(0, 0, 3)});
  EXPECT_EQ(Eigen::Vector3d(0, 0, 3), shape.getBoundingBox().getMax());

  const auto version = shape.getVersion();
  shape.removeAllPoints();
  EXPECT_NE(version, shape.getVersion());
  EXPECT_EQ(Eigen::Vector3d::Zero(), shape.getBoundingBox().getMax());

#if HAVE_OCTOMAP
  octomap::Pointcloud cloud;
  cloud.push_back(4.0f, 0.0f, 0.0f);
  shape.setPoints(cloud);
  EXPECT_EQ(Eigen::Vector3d(4, 0, 0), shape.getBoundingBox().getMax());

  cloud.clear();
  cloud.push_back(0.0f, 5.0f, 0.0f);
  shape.addPoints(cloud);
  EXPECT_EQ(Eigen::Vector3d(4, 5, 0), shape.getBoundingBox().getMax());
#endif
}
