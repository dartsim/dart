/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary form, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/simulation/experimental/common/exceptions.hpp>
#include <dart/simulation/experimental/io/codim_mesh.hpp>

#include <gtest/gtest.h>

#include <sstream>
#include <string>

namespace sx = dart::simulation::experimental;
namespace io = dart::simulation::experimental::io;

namespace {

io::SegmentMesh parseSeg(const std::string& text)
{
  std::istringstream input(text);
  return io::loadSegLineMesh(input);
}

io::PointSet parsePt(const std::string& text)
{
  std::istringstream input(text);
  return io::loadPointSet(input);
}

} // namespace

//==============================================================================
TEST(CodimMesh, SegPolylineExpandsToConsecutiveSegments)
{
  // A 3-vertex polyline becomes two segments; a comment and a vn are ignored.
  const io::SegmentMesh mesh
      = parseSeg("# a strand\nv 0 0 0\nv 1 0 0\nv 2 0 0\nvn 0 0 1\nl 1 2 3\n");

  ASSERT_EQ(mesh.positions.size(), 3u);
  ASSERT_EQ(mesh.segments.size(), 2u);
  EXPECT_TRUE(mesh.positions[2].isApprox(Eigen::Vector3d(2.0, 0.0, 0.0)));
  EXPECT_EQ(mesh.segments[0], (std::array<std::size_t, 2>{0, 1}));
  EXPECT_EQ(mesh.segments[1], (std::array<std::size_t, 2>{1, 2}));
}

//==============================================================================
TEST(CodimMesh, SegResolvesNegativeIndicesAndRejectsOutOfRange)
{
  const io::SegmentMesh mesh = parseSeg("v 0 0 0\nv 1 0 0\nl -2 -1\n");
  ASSERT_EQ(mesh.segments.size(), 1u);
  EXPECT_EQ(mesh.segments[0], (std::array<std::size_t, 2>{0, 1}));

  EXPECT_THROW(
      parseSeg("v 0 0 0\nv 1 0 0\nl 1 5\n"), sx::InvalidArgumentException);
  EXPECT_THROW(parseSeg("v 0 0 0\nv 1 0 0\n"), sx::InvalidArgumentException);
}

//==============================================================================
TEST(CodimMesh, PointSetReadsBareAndWavefrontCoordinates)
{
  // Mixed bare "x y z" and Wavefront "v x y z" lines, plus a comment.
  const io::PointSet points = parsePt("# cloud\n0 0 0\n1 2 3\nv 4 5 6\n");
  ASSERT_EQ(points.positions.size(), 3u);
  EXPECT_TRUE(points.positions[0].isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
  EXPECT_TRUE(points.positions[1].isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_TRUE(points.positions[2].isApprox(Eigen::Vector3d(4.0, 5.0, 6.0)));
}

//==============================================================================
TEST(CodimMesh, PointSetRejectsMalformedAndEmpty)
{
  EXPECT_THROW(parsePt("0 0\n"), sx::InvalidArgumentException);
  EXPECT_THROW(parsePt("# only a comment\n"), sx::InvalidArgumentException);
}
