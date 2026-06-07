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

#include <dart/simulation/body/deformable_body.hpp>
#include <dart/simulation/common/exceptions.hpp>
#include <dart/simulation/io/gmsh_tet_mesh.hpp>
#include <dart/simulation/world.hpp>

#include <gtest/gtest.h>

#include <sstream>
#include <string>

namespace sx = dart::simulation;
namespace io = dart::simulation::io;

namespace {

// A GMSH 2.2 ASCII mesh: a unit tetrahedron split into two tets sharing the
// (1,2,3) face, with non-contiguous node ids and a stray triangle element that
// the loader must ignore.
const char* kTwoTetMsh = R"(
$MeshFormat
2.2 0 8
$EndMeshFormat
$Nodes
5
10 0 0 0
11 1 0 0
12 0 1 0
13 0 0 1
14 1 1 1
$EndNodes
$Elements
3
1 2 2 0 1 10 11 12
2 4 2 1 1 10 11 12 13
3 4 2 1 1 11 12 13 14
$EndElements
)";

io::TetMesh parse(const std::string& text)
{
  std::istringstream input(text);
  return io::loadGmshTetMesh(input);
}

} // namespace

//==============================================================================
TEST(GmshTetMesh, ParsesNodesAndTetrahedraIgnoringOtherElements)
{
  const io::TetMesh mesh = parse(kTwoTetMsh);

  ASSERT_EQ(mesh.positions.size(), 5u);
  // The stray triangle (element type 2) is ignored; both tetrahedra are kept.
  ASSERT_EQ(mesh.tetrahedra.size(), 2u);

  // Node ids 10..14 remap to 0-based indices 0..4 in first-seen order.
  EXPECT_TRUE(mesh.positions[0].isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
  EXPECT_TRUE(mesh.positions[4].isApprox(Eigen::Vector3d(1.0, 1.0, 1.0)));

  // Tet 1 references ids 10,11,12,13 -> indices 0,1,2,3.
  EXPECT_EQ(mesh.tetrahedra[0], (std::array<std::size_t, 4>{0, 1, 2, 3}));
  // Tet 2 references ids 11,12,13,14 -> indices 1,2,3,4.
  EXPECT_EQ(mesh.tetrahedra[1], (std::array<std::size_t, 4>{1, 2, 3, 4}));
}

//==============================================================================
TEST(GmshTetMesh, ParsesFormat41EntityBlocks)
{
  // GMSH 4.1 entity-block layout: node tags then coordinates per block; a
  // per-block element type. One tetrahedron over four nodes.
  const io::TetMesh mesh = parse(
      "$MeshFormat\n4.1 0 8\n$EndMeshFormat\n"
      "$Nodes\n1 4 1 4\n2 1 0 4\n1\n2\n3\n4\n"
      "0 0 0\n1 0 0\n0 1 0\n0 0 1\n$EndNodes\n"
      "$Elements\n1 1 1 1\n3 1 4 1\n1 1 2 3 4\n$EndElements\n");

  ASSERT_EQ(mesh.positions.size(), 4u);
  ASSERT_EQ(mesh.tetrahedra.size(), 1u);
  EXPECT_TRUE(mesh.positions[3].isApprox(Eigen::Vector3d(0.0, 0.0, 1.0)));
  EXPECT_EQ(mesh.tetrahedra[0], (std::array<std::size_t, 4>{0, 1, 2, 3}));
}

//==============================================================================
TEST(GmshTetMesh, RejectsMalformedAndUnsupportedMeshes)
{
  // Unsupported (binary) format.
  EXPECT_THROW(
      parse(
          "$MeshFormat\n2.2 1 8\n$EndMeshFormat\n$Nodes\n0\n$EndNodes\n"
          "$Elements\n0\n$EndElements\n"),
      sx::InvalidArgumentException);

  // Unsupported version (5.x is beyond the supported 2.x/4.x range).
  EXPECT_THROW(
      parse("$MeshFormat\n5.0 0 8\n$EndMeshFormat\n"),
      sx::InvalidArgumentException);

  // Tetra references an unknown node id.
  EXPECT_THROW(
      parse(
          "$MeshFormat\n2.2 0 8\n$EndMeshFormat\n$Nodes\n1\n1 0 0 0\n"
          "$EndNodes\n$Elements\n1\n1 4 0 1 2 3 4\n$EndElements\n"),
      sx::InvalidArgumentException);

  // No tetrahedra at all.
  EXPECT_THROW(
      parse(
          "$MeshFormat\n2.2 0 8\n$EndMeshFormat\n$Nodes\n1\n1 0 0 0\n"
          "$EndNodes\n$Elements\n0\n$EndElements\n"),
      sx::InvalidArgumentException);
}

//==============================================================================
// A FEM body built from a loaded GMSH tetrahedron, pinned at one node, hangs
// from it under gravity and stays finite/bounded -- the importer feeds the FEM
// solver end to end.
TEST(GmshTetMesh, LoadedTetSimulatesAsFemBody)
{
  const io::TetMesh mesh = parse(
      "$MeshFormat\n2.2 0 8\n$EndMeshFormat\n$Nodes\n4\n"
      "1 0 0 0\n2 1 0 0\n3 0 1 0\n4 0 0 1\n$EndNodes\n"
      "$Elements\n1\n1 4 0 1 2 3 4\n$EndElements\n");
  ASSERT_EQ(mesh.tetrahedra.size(), 1u);

  sx::DeformableBodyOptions options;
  options.positions = mesh.positions;
  for (const auto& tet : mesh.tetrahedra) {
    options.tetrahedra.push_back(
        sx::DeformableTetrahedron{tet[0], tet[1], tet[2], tet[3]});
  }
  options.fixedNodes = {0};
  options.material.useFiniteElementElasticity = true;

  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.005);
  auto body = world.addDeformableBody("gmsh_tet", options);

  world.step(200);

  EXPECT_TRUE(body.getPosition(0).isApprox(Eigen::Vector3d::Zero(), 1e-9));
  for (std::size_t i = 0; i < body.getNodeCount(); ++i) {
    EXPECT_TRUE(body.getPosition(i).allFinite());
    EXPECT_LT(body.getPosition(i).norm(), 5.0); // bounded (no blow-up)
  }
}
