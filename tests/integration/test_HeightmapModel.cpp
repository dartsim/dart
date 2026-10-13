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

#include "demo_models/HeightmapModel.hpp"

#include <dart/math/Random.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

TEST(HeightmapModelTest, PreservesFloatRandomFieldAndScale)
{
  dart::math::Random::setSeed(1234);
  dart::dynamics::HeightmapShapef::HeightField expected(3, 4);
  for (Eigen::Index row = 0; row < expected.rows(); ++row) {
    for (Eigen::Index column = 0; column < expected.cols(); ++column)
      expected(row, column) = dart::math::Random::uniform(-0.2f, 0.5f);
  }
  const float next = dart::math::Random::uniform(0.f, 1.f);

  dart::math::Random::setSeed(1234);
  const auto shape
      = dart_demos::createHeightmapShape(4, 3, 2.f, 3.f, -0.2f, 0.5f);
  EXPECT_TRUE((shape->getHeightField().array() == expected.array()).all());
  EXPECT_TRUE(shape->getScale().isApprox(Eigen::Vector3f(0.5f, 1.f, 1.f)));
  EXPECT_FLOAT_EQ(dart::math::Random::uniform(0.f, 1.f), next);
}

TEST(HeightmapModelTest, BuildsDefaultTerrain)
{
  const auto shape
      = dart_demos::createHeightmapShape(100, 100, 2.f, 2.f, 0.f, 0.1f);
  EXPECT_EQ(shape->getHeightField().rows(), 100);
  EXPECT_EQ(shape->getHeightField().cols(), 100);
  EXPECT_TRUE(shape->getScale().isApprox(Eigen::Vector3f(0.02f, 0.02f, 1.f)));
  EXPECT_GE(shape->getMinHeight(), 0.f);
  EXPECT_LE(shape->getMaxHeight(), 0.1f);
}

TEST(HeightmapModelTest, RejectsInvalidInputsBeforeDrawingRandomNumbers)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float infinity = std::numeric_limits<float>::infinity();
  const auto maximum = std::numeric_limits<std::size_t>::max();
  dart::math::Random::setSeed(1234);
  const float next = dart::math::Random::uniform(0.f, 1.f);
  dart::math::Random::setSeed(1234);

  EXPECT_THROW(
      dart_demos::createHeightmapShape(0, 5, 2.f, 2.f, 0.f, 0.1f),
      std::invalid_argument);
  EXPECT_THROW(
      dart_demos::createHeightmapShape(5, 0, 2.f, 2.f, 0.f, 0.1f),
      std::invalid_argument);
  EXPECT_THROW(
      dart_demos::createHeightmapShape(maximum, 2, 2.f, 2.f, 0.f, 0.1f),
      std::invalid_argument);
  EXPECT_THROW(
      dart_demos::createHeightmapShape(
          maximum / sizeof(float) + 1, 1, 2.f, 2.f, 0.f, 0.1f),
      std::invalid_argument);
  EXPECT_THROW(
      dart_demos::createHeightmapShape(5, 5, 0.f, 2.f, 0.f, 0.1f),
      std::invalid_argument);
  EXPECT_THROW(
      dart_demos::createHeightmapShape(5, 5, 2.f, -1.f, 0.f, 0.1f),
      std::invalid_argument);
  EXPECT_THROW(
      dart_demos::createHeightmapShape(5, 5, infinity, 2.f, 0.f, 0.1f),
      std::invalid_argument);
  EXPECT_THROW(
      dart_demos::createHeightmapShape(5, 5, 2.f, nan, 0.f, 0.1f),
      std::invalid_argument);
  EXPECT_THROW(
      dart_demos::createHeightmapShape(5, 5, 2.f, 2.f, nan, 0.1f),
      std::invalid_argument);
  EXPECT_THROW(
      dart_demos::createHeightmapShape(5, 5, 2.f, 2.f, 0.f, infinity),
      std::invalid_argument);
  EXPECT_THROW(
      dart_demos::createHeightmapShape(5, 5, 2.f, 2.f, 1.f, 0.f),
      std::invalid_argument);
  EXPECT_THROW(
      dart_demos::createHeightmapShape(
          5,
          5,
          2.f,
          2.f,
          -std::numeric_limits<float>::max(),
          std::numeric_limits<float>::max()),
      std::invalid_argument);
  EXPECT_THROW(
      dart_demos::createHeightmapShape(
          5, 5, std::numeric_limits<float>::denorm_min(), 2.f, 0.f, 0.1f),
      std::invalid_argument);
  EXPECT_FLOAT_EQ(dart::math::Random::uniform(0.f, 1.f), next);
}
