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

#include <dart/simulation/detail/deformable_vbd/vertex_coloring.hpp>

#include <dart/common/memory_manager.hpp>
#include <dart/common/stl_allocator.hpp>

#include <gtest/gtest.h>

#include <set>
#include <vector>

namespace vbd = dart::simulation::detail::deformable_vbd;
namespace common = dart::common;

namespace {

//==============================================================================
// Verify the groups partition all vertices exactly once and match
// colorOfVertex.
void expectGroupsPartitionVertices(const vbd::VertexColoring& coloring)
{
  std::set<std::uint32_t> seen;
  for (std::uint32_t color = 0; color < coloring.groups.size(); ++color) {
    for (const std::uint32_t vertex : coloring.groups[color]) {
      EXPECT_TRUE(seen.insert(vertex).second)
          << "vertex " << vertex << " in multiple groups";
      EXPECT_EQ(coloring.colorOfVertex[vertex], color);
    }
  }
  EXPECT_EQ(seen.size(), coloring.colorOfVertex.size());
}

} // namespace

//==============================================================================
TEST(VbdVertexColoring, IsolatedVerticesUseOneColor)
{
  vbd::VertexAdjacency adjacency(5);
  const vbd::VertexColoring coloring = vbd::greedyColorVertices(adjacency);
  EXPECT_EQ(coloring.colorCount(), 1u);
  EXPECT_TRUE(vbd::isProperColoring(adjacency, coloring));
  expectGroupsPartitionVertices(coloring);
}

//==============================================================================
TEST(VbdVertexColoring, SingleEdgeUsesTwoColors)
{
  vbd::VertexAdjacency adjacency(2);
  adjacency.addEdge(0, 1);
  const vbd::VertexColoring coloring = vbd::greedyColorVertices(adjacency);
  EXPECT_EQ(coloring.colorCount(), 2u);
  EXPECT_NE(coloring.colorOfVertex[0], coloring.colorOfVertex[1]);
  EXPECT_TRUE(vbd::isProperColoring(adjacency, coloring));
}

//==============================================================================
TEST(VbdVertexColoring, ChainIsBipartite)
{
  // A spring chain 0-1-2-3-4-5 is bipartite -> 2 colors.
  vbd::VertexAdjacency adjacency(6);
  for (std::uint32_t v = 0; v + 1 < 6; ++v) {
    adjacency.addEdge(v, v + 1);
  }
  const vbd::VertexColoring coloring = vbd::greedyColorVertices(adjacency);
  EXPECT_EQ(coloring.colorCount(), 2u);
  EXPECT_TRUE(vbd::isProperColoring(adjacency, coloring));
  expectGroupsPartitionVertices(coloring);
}

//==============================================================================
TEST(VbdVertexColoring, TriangleCliqueUsesThreeColors)
{
  vbd::VertexAdjacency adjacency(3);
  adjacency.addTriangle(0, 1, 2);
  const vbd::VertexColoring coloring = vbd::greedyColorVertices(adjacency);
  EXPECT_EQ(coloring.colorCount(), 3u);
  EXPECT_TRUE(vbd::isProperColoring(adjacency, coloring));
}

//==============================================================================
TEST(VbdVertexColoring, TetrahedronCliqueUsesFourColors)
{
  vbd::VertexAdjacency adjacency(4);
  adjacency.addTetrahedron(0, 1, 2, 3);
  const vbd::VertexColoring coloring = vbd::greedyColorVertices(adjacency);
  EXPECT_EQ(coloring.colorCount(), 4u);
  EXPECT_TRUE(vbd::isProperColoring(adjacency, coloring));
  expectGroupsPartitionVertices(coloring);
}

//==============================================================================
TEST(VbdVertexColoring, AddElementMatchesPairwiseEdges)
{
  vbd::VertexAdjacency viaElement(4);
  viaElement.addElement({0, 1, 2, 3});
  viaElement.finalize();

  vbd::VertexAdjacency viaTet(4);
  viaTet.addTetrahedron(0, 1, 2, 3);
  viaTet.finalize();

  for (std::size_t v = 0; v < 4; ++v) {
    EXPECT_EQ(viaElement.neighbors(v), viaTet.neighbors(v));
  }
}

//==============================================================================
TEST(VbdVertexColoring, IgnoresSelfLoopsAndOutOfRange)
{
  vbd::VertexAdjacency adjacency(3);
  adjacency.addEdge(0, 0);  // self loop ignored
  adjacency.addEdge(0, 99); // out of range ignored
  adjacency.addEdge(1, 2);
  adjacency.finalize();
  EXPECT_TRUE(adjacency.neighbors(0).empty());
  EXPECT_EQ(adjacency.neighbors(1).size(), 1u);
}

//==============================================================================
TEST(VbdVertexColoring, DuplicateEdgesCollapse)
{
  vbd::VertexAdjacency adjacency(2);
  adjacency.addEdge(0, 1);
  adjacency.addEdge(0, 1);
  adjacency.addEdge(1, 0);
  adjacency.finalize();
  EXPECT_EQ(adjacency.neighbors(0).size(), 1u);
  EXPECT_EQ(adjacency.neighbors(1).size(), 1u);
}

//==============================================================================
TEST(VbdVertexColoring, StructuredTetMeshIsProperAndBounded)
{
  // A strip of tetrahedra sharing faces: vertices 0..N-1, tets (i,i+1,i+2,i+3).
  constexpr std::uint32_t vertexCount = 12;
  vbd::VertexAdjacency adjacency(vertexCount);
  for (std::uint32_t i = 0; i + 3 < vertexCount; ++i) {
    adjacency.addTetrahedron(i, i + 1, i + 2, i + 3);
  }
  const vbd::VertexColoring coloring = vbd::greedyColorVertices(adjacency);

  EXPECT_TRUE(vbd::isProperColoring(adjacency, coloring));
  expectGroupsPartitionVertices(coloring);
  // Greedy coloring uses at most maxDegree + 1 colors.
  EXPECT_LE(coloring.colorCount(), adjacency.maxDegree() + 1);
}

//==============================================================================
TEST(VbdVertexColoring, ColoringIsDeterministic)
{
  vbd::VertexAdjacency a(8);
  vbd::VertexAdjacency b(8);
  for (std::uint32_t v = 0; v + 1 < 8; ++v) {
    a.addEdge(v, v + 1);
    b.addEdge(v, v + 1);
  }
  a.addTriangle(0, 2, 4);
  b.addTriangle(0, 2, 4);
  const vbd::VertexColoring first = vbd::greedyColorVertices(a);
  const vbd::VertexColoring second = vbd::greedyColorVertices(b);
  EXPECT_EQ(first.colorOfVertex, second.colorOfVertex);
}

//==============================================================================
TEST(VbdVertexColoring, ColoringUsesProvidedAllocatorForNestedStorage)
{
  common::MemoryManager memoryManager;
  auto& allocator = memoryManager.getFreeListAllocator();
  const auto allocationsBefore = allocator.getAllocationCount();

  vbd::VertexAdjacency adjacency(4, allocator);
  adjacency.addTetrahedron(0, 1, 2, 3);
  const vbd::VertexColoring coloring = vbd::greedyColorVertices(adjacency);

  EXPECT_TRUE(vbd::isProperColoring(adjacency, coloring));
  EXPECT_GT(allocator.getAllocationCount(), allocationsBefore);
  EXPECT_EQ(
      adjacency.neighbors(0).get_allocator(),
      common::StlAllocator<std::uint32_t>{allocator});
  EXPECT_EQ(
      coloring.colorOfVertex.get_allocator(),
      common::StlAllocator<std::uint32_t>{allocator});
  ASSERT_FALSE(coloring.groups.empty());
  EXPECT_EQ(
      coloring.groups[0].get_allocator(),
      common::StlAllocator<std::uint32_t>{allocator});
}
