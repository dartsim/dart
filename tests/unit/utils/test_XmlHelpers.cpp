/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
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

// Unit tests for XmlHelpers parsing functions
// See: https://github.com/dartsim/dart/issues/2423

#include <dart/utils/XmlHelpers.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <string>

using namespace dart::utils;

//==============================================================================
// Tests for valid input (round-trip parsing)
//==============================================================================

TEST(XmlHelpers, ValidVector2dParsing)
{
  const std::string input = "1.5 2.5";
  const Eigen::Vector2d result = toVector2d(input);
  EXPECT_DOUBLE_EQ(result[0], 1.5);
  EXPECT_DOUBLE_EQ(result[1], 2.5);
}

TEST(XmlHelpers, ValidVector3dParsing)
{
  const std::string input = "1.0 2.0 3.0";
  const Eigen::Vector3d result = toVector3d(input);
  EXPECT_DOUBLE_EQ(result[0], 1.0);
  EXPECT_DOUBLE_EQ(result[1], 2.0);
  EXPECT_DOUBLE_EQ(result[2], 3.0);
}

TEST(XmlHelpers, ValidVector6dParsing)
{
  const std::string input = "1.0 2.0 3.0 4.0 5.0 6.0";
  const Eigen::Vector6d result = toVector6d(input);
  EXPECT_DOUBLE_EQ(result[0], 1.0);
  EXPECT_DOUBLE_EQ(result[1], 2.0);
  EXPECT_DOUBLE_EQ(result[2], 3.0);
  EXPECT_DOUBLE_EQ(result[3], 4.0);
  EXPECT_DOUBLE_EQ(result[4], 5.0);
  EXPECT_DOUBLE_EQ(result[5], 6.0);
}

//==============================================================================
// Tests for invalid input - Issue #2423
// These tests verify that exceptions are properly thrown when parsing fails
//==============================================================================

TEST(XmlHelpers, InvalidVector2dThrowsException)
{
  // Test with invalid second component
  const std::string input = "1.0 invalid";
  EXPECT_THROW(toVector2d(input), std::exception);
}

TEST(XmlHelpers, InvalidVector2dFirstComponentThrowsException)
{
  // Test with invalid first component
  const std::string input = "not_a_number 2.0";
  EXPECT_THROW(toVector2d(input), std::exception);
}

TEST(XmlHelpers, InvalidVector2iThrowsException)
{
  const std::string input = "1 invalid";
  EXPECT_THROW(toVector2i(input), std::exception);
}

TEST(XmlHelpers, InvalidVector3dThrowsException)
{
  const std::string input = "1.0 2.0 invalid";
  EXPECT_THROW(toVector3d(input), std::exception);
}

TEST(XmlHelpers, InvalidVector3iThrowsException)
{
  const std::string input = "1 2 invalid";
  EXPECT_THROW(toVector3i(input), std::exception);
}

TEST(XmlHelpers, InvalidVector4dThrowsException)
{
  const std::string input = "1.0 2.0 3.0 invalid";
  EXPECT_THROW(toVector4d(input), std::exception);
}

TEST(XmlHelpers, InvalidVector6dThrowsException)
{
  const std::string input = "1.0 2.0 3.0 4.0 5.0 invalid";
  EXPECT_THROW(toVector6d(input), std::exception);
}

TEST(XmlHelpers, InvalidVectorXdThrowsException)
{
  const std::string input = "1.0 2.0 invalid 4.0";
  EXPECT_THROW(toVectorXd(input), std::exception);
}

TEST(XmlHelpers, InvalidIsometry3dThrowsException)
{
  // Isometry3d expects 6 values: x, y, z, roll, pitch, yaw
  const std::string input = "1.0 2.0 3.0 0.0 0.0 invalid";
  EXPECT_THROW(toIsometry3d(input), std::exception);
}

TEST(XmlHelpers, InvalidIsometry3dWithExtrinsicRotationThrowsException)
{
  const std::string input = "1.0 2.0 3.0 invalid 0.0 0.0";
  EXPECT_THROW(toIsometry3dWithExtrinsicRotation(input), std::exception);
}

//==============================================================================
// Test that valid Isometry3d parsing still works
//==============================================================================

TEST(XmlHelpers, ValidIsometry3dParsing)
{
  const std::string input = "1.0 2.0 3.0 0.0 0.0 0.0";
  const Eigen::Isometry3d result = toIsometry3d(input);
  EXPECT_DOUBLE_EQ(result.translation()[0], 1.0);
  EXPECT_DOUBLE_EQ(result.translation()[1], 2.0);
  EXPECT_DOUBLE_EQ(result.translation()[2], 3.0);
}

//==============================================================================
// Test scalar parsing functions
//==============================================================================

TEST(XmlHelpers, ValidDoubleParsing)
{
  EXPECT_DOUBLE_EQ(toDouble("3.14159"), 3.14159);
  EXPECT_DOUBLE_EQ(toDouble("-2.5"), -2.5);
  EXPECT_DOUBLE_EQ(toDouble("0"), 0.0);
}

TEST(XmlHelpers, InvalidDoubleThrowsException)
{
  EXPECT_THROW(toDouble("not_a_number"), std::exception);
}

TEST(XmlHelpers, ValidIntParsing)
{
  EXPECT_EQ(toInt("42"), 42);
  EXPECT_EQ(toInt("-10"), -10);
  EXPECT_EQ(toInt("0"), 0);
}

TEST(XmlHelpers, InvalidIntThrowsException)
{
  EXPECT_THROW(toInt("not_a_number"), std::exception);
}

TEST(XmlHelpers, ValidBoolParsing)
{
  EXPECT_TRUE(toBool("true"));
  EXPECT_TRUE(toBool("TRUE"));
  EXPECT_TRUE(toBool("1"));
  EXPECT_FALSE(toBool("false"));
  EXPECT_FALSE(toBool("FALSE"));
  EXPECT_FALSE(toBool("0"));
}
