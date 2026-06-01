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
#include <dart/simulation/experimental/io/obj_triangle_mesh.hpp>

#include <gtest/gtest.h>

#include <sstream>
#include <string>

namespace sx = dart::simulation::experimental;
namespace io = dart::simulation::experimental::io;

namespace {

// A unit square (two triangles) with vertex/texcoord/normal face tokens, a
// comment, and ignored vn/vt directives.
const char* kSquareObj = R"(
# a unit square in the z = 0 plane
v 0 0 0
v 1 0 0
v 1 1 0
v 0 1 0
vt 0 0
vn 0 0 1
f 1/1/1 2/1/1 3/1/1
f 1//1 3//1 4//1
)";

io::TriangleMesh parse(const std::string& text)
{
  std::istringstream input(text);
  return io::loadObjTriangleMesh(input);
}

} // namespace

//==============================================================================
TEST(ObjTriangleMesh, ParsesVerticesAndFacesIgnoringOtherDirectives)
{
  const io::TriangleMesh mesh = parse(kSquareObj);

  ASSERT_EQ(mesh.positions.size(), 4u);
  ASSERT_EQ(mesh.triangles.size(), 2u);

  EXPECT_TRUE(mesh.positions[0].isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
  EXPECT_TRUE(mesh.positions[2].isApprox(Eigen::Vector3d(1.0, 1.0, 0.0)));

  // 1-based face indices remap to 0-based, keeping only the position index.
  EXPECT_EQ(mesh.triangles[0], (std::array<std::size_t, 3>{0, 1, 2}));
  EXPECT_EQ(mesh.triangles[1], (std::array<std::size_t, 3>{0, 2, 3}));
}

//==============================================================================
TEST(ObjTriangleMesh, FanTriangulatesPolygonFaces)
{
  // A single quad face fan-triangulates into two triangles.
  const io::TriangleMesh mesh
      = parse("v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf 1 2 3 4\n");
  ASSERT_EQ(mesh.triangles.size(), 2u);
  EXPECT_EQ(mesh.triangles[0], (std::array<std::size_t, 3>{0, 1, 2}));
  EXPECT_EQ(mesh.triangles[1], (std::array<std::size_t, 3>{0, 2, 3}));
}

//==============================================================================
TEST(ObjTriangleMesh, ResolvesNegativeRelativeIndices)
{
  // -1 refers to the most recent vertex, -3 to three back.
  const io::TriangleMesh mesh
      = parse("v 0 0 0\nv 1 0 0\nv 0 1 0\nf -3 -2 -1\n");
  ASSERT_EQ(mesh.triangles.size(), 1u);
  EXPECT_EQ(mesh.triangles[0], (std::array<std::size_t, 3>{0, 1, 2}));
}

//==============================================================================
TEST(ObjTriangleMesh, RejectsOutOfRangeFaceIndex)
{
  EXPECT_THROW(
      parse("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 9\n"),
      sx::InvalidArgumentException);
}

//==============================================================================
TEST(ObjTriangleMesh, RejectsMeshWithoutFaces)
{
  EXPECT_THROW(
      parse("v 0 0 0\nv 1 0 0\nv 0 1 0\n"), sx::InvalidArgumentException);
}
