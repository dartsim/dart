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

#include <dart/dynamics/LineSegmentShape.hpp>

#include <dart/common/Deprecated.hpp>

#include <gtest/gtest.h>

#include <memory>

using namespace dart::dynamics;

//==============================================================================
// Both constructors set up a line segment the same way: the two-point one used
// to report UNSUPPORTED from the deprecated getShapeType().
TEST(LineSegmentShape, BothConstructorsReportLineSegmentType)
{
  const auto empty = std::make_shared<LineSegmentShape>(0.1f);
  const auto segment = std::make_shared<LineSegmentShape>(
      Eigen::Vector3d(-1.0, -2.0, -3.0), Eigen::Vector3d(1.0, 2.0, 3.0), 0.1f);

  DART_SUPPRESS_DEPRECATED_BEGIN
  EXPECT_EQ(Shape::LINE_SEGMENT, empty->getShapeType());
  EXPECT_EQ(Shape::LINE_SEGMENT, segment->getShapeType());
  DART_SUPPRESS_DEPRECATED_END

  EXPECT_EQ(LineSegmentShape::getStaticType(), segment->getType());
  EXPECT_TRUE(segment->getBoundingBox().getMin().isApprox(
      Eigen::Vector3d(-1.0, -2.0, -3.0)));
  EXPECT_TRUE(segment->getBoundingBox().getMax().isApprox(
      Eigen::Vector3d(1.0, 2.0, 3.0)));
}
