/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <dart/dynamics/HeightmapShape.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <vector>

using namespace dart::dynamics;

// HeightmapShape::setScale and setHeightField guard against non-finite
// (NaN/Inf) and non-positive inputs so they never reach the collision backend
// (e.g. ODE) where they would trip an internal assertion or undefined
// behavior. See https://github.com/gazebosim/gz-physics/issues/847

//==============================================================================
TEST(HeightmapShapeValidation, AcceptsValidPositiveFiniteScale)
{
  auto heightmap = std::make_shared<HeightmapShape<double>>();

  // Default scale is (1, 1, 1).
  EXPECT_EQ(heightmap->getScale(), Eigen::Vector3d(1.0, 1.0, 1.0));

  heightmap->setScale(Eigen::Vector3d(2.0, 3.0, 0.5));
  EXPECT_EQ(heightmap->getScale(), Eigen::Vector3d(2.0, 3.0, 0.5));
}

//==============================================================================
TEST(HeightmapShapeValidation, RejectsNonFiniteScaleAndPreservesOriginal)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  auto heightmap = std::make_shared<HeightmapShape<double>>();
  heightmap->setScale(Eigen::Vector3d(2.0, 2.0, 2.0));
  const Eigen::Vector3d original = heightmap->getScale();

  heightmap->setScale(Eigen::Vector3d(inf, 1.0, 1.0));
  EXPECT_EQ(heightmap->getScale(), original);

  heightmap->setScale(Eigen::Vector3d(1.0, -inf, 1.0));
  EXPECT_EQ(heightmap->getScale(), original);

  heightmap->setScale(Eigen::Vector3d(1.0, 1.0, nan));
  EXPECT_EQ(heightmap->getScale(), original);
}

//==============================================================================
TEST(HeightmapShapeValidation, RejectsNonPositiveScaleAndPreservesOriginal)
{
  auto heightmap = std::make_shared<HeightmapShape<double>>();
  heightmap->setScale(Eigen::Vector3d(2.0, 2.0, 2.0));
  const Eigen::Vector3d original = heightmap->getScale();

  heightmap->setScale(Eigen::Vector3d(0.0, 1.0, 1.0));
  EXPECT_EQ(heightmap->getScale(), original);

  heightmap->setScale(Eigen::Vector3d(1.0, -1.0, 1.0));
  EXPECT_EQ(heightmap->getScale(), original);
}

//==============================================================================
TEST(HeightmapShapeValidation, RejectsNonFiniteHeightFieldAndPreservesOriginal)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  auto heightmap = std::make_shared<HeightmapShape<double>>();
  heightmap->setHeightField(2u, 2u, std::vector<double>{0.0, 1.0, 2.0, 3.0});
  EXPECT_DOUBLE_EQ(heightmap->getMaxHeight(), 3.0);
  EXPECT_DOUBLE_EQ(heightmap->getMinHeight(), 0.0);

  // A height field carrying a non-finite value must be rejected, leaving the
  // previously set (valid) field intact.
  heightmap->setHeightField(2u, 2u, std::vector<double>{0.0, nan, 2.0, 3.0});
  EXPECT_DOUBLE_EQ(heightmap->getMaxHeight(), 3.0);
  EXPECT_DOUBLE_EQ(heightmap->getMinHeight(), 0.0);

  heightmap->setHeightField(2u, 2u, std::vector<double>{0.0, 1.0, inf, 3.0});
  EXPECT_DOUBLE_EQ(heightmap->getMaxHeight(), 3.0);
  EXPECT_DOUBLE_EQ(heightmap->getMinHeight(), 0.0);
}

//==============================================================================
TEST(HeightmapShapeValidation, InvalidInputsDoNotThrow)
{
  const float inf = std::numeric_limits<float>::infinity();

  auto heightmap = std::make_shared<HeightmapShape<float>>();

  EXPECT_NO_THROW(heightmap->setScale(Eigen::Vector3f(inf, inf, inf)));
  EXPECT_NO_THROW(heightmap->setHeightField(
      2u, 2u, std::vector<float>{0.0f, 1.0f, 2.0f, 3.0f}));
  EXPECT_NO_THROW(heightmap->setHeightField(
      2u, 2u, std::vector<float>{0.0f, inf, 2.0f, 3.0f}));
}
