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

#pragma once

#include <dart/common/stl_allocator.hpp>

#include <algorithm>
#include <initializer_list>
#include <span>
#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::simulation::detail::deformable_vbd {

/// Sentinel color marking an unprocessed vertex.
inline constexpr std::uint32_t kUncolored = 0xffffffffu;

/// Sparse vertex-vertex adjacency induced by force elements.
///
/// Two vertices are adjacent iff they share at least one force element (a
/// spring edge, a surface triangle, or a tetrahedron). VBD colors this graph so
/// that vertices of the same color never share an element: their per-vertex
/// block updates are then mutually independent and can run in parallel, while
/// colors are swept sequentially (Gauss-Seidel). The reference uses the
/// *vertex* graph (3-8 colors) rather than the element/dual graph (7-76 colors)
/// precisely because fewer colors means fewer sequential sweeps.
class VertexAdjacency
{
public:
  using NeighborVector
      = std::vector<std::uint32_t, ::dart::common::StlAllocator<std::uint32_t>>;
  using NeighborVectorAllocator = ::dart::common::StlAllocator<NeighborVector>;
  using NeighborRows = std::vector<NeighborVector, NeighborVectorAllocator>;

  explicit VertexAdjacency(
      std::size_t vertexCount,
      ::dart::common::MemoryAllocator& allocator
      = ::dart::common::MemoryAllocator::GetDefault())
    : m_allocator(&allocator), m_neighbors(NeighborVectorAllocator{allocator})
  {
    resizeNeighborRows(vertexCount);
  }

  [[nodiscard]] std::size_t vertexCount() const noexcept
  {
    return m_neighbors.size();
  }

  /// Connect two vertices (a single edge). Out-of-range indices and self-loops
  /// are ignored. Duplicate edges are collapsed in finalize().
  void addEdge(std::uint32_t a, std::uint32_t b)
  {
    if (a == b) {
      return;
    }
    if (a >= m_neighbors.size() || b >= m_neighbors.size()) {
      return;
    }
    m_neighbors[a].push_back(b);
    m_neighbors[b].push_back(a);
    m_finalized = false;
  }

  /// Connect every pair of vertices in an element (the element forms a clique
  /// in the vertex graph). Used for triangles (3 vertices) and tetrahedra (4
  /// vertices).
  void addElement(std::span<const std::uint32_t> vertices)
  {
    for (std::size_t i = 0; i < vertices.size(); ++i) {
      for (std::size_t j = i + 1; j < vertices.size(); ++j) {
        addEdge(vertices[i], vertices[j]);
      }
    }
  }

  void addElement(std::initializer_list<std::uint32_t> vertices)
  {
    addElement(
        std::span<const std::uint32_t>{vertices.begin(), vertices.size()});
  }

  void addTriangle(std::uint32_t a, std::uint32_t b, std::uint32_t c)
  {
    addEdge(a, b);
    addEdge(b, c);
    addEdge(c, a);
  }

  void addTetrahedron(
      std::uint32_t a, std::uint32_t b, std::uint32_t c, std::uint32_t d)
  {
    addEdge(a, b);
    addEdge(a, c);
    addEdge(a, d);
    addEdge(b, c);
    addEdge(b, d);
    addEdge(c, d);
  }

  /// Sort and de-duplicate each vertex's neighbor list. Idempotent.
  void finalize()
  {
    if (m_finalized) {
      return;
    }
    for (auto& list : m_neighbors) {
      std::sort(list.begin(), list.end());
      list.erase(std::unique(list.begin(), list.end()), list.end());
    }
    m_finalized = true;
  }

  [[nodiscard]] const NeighborVector& neighbors(std::size_t vertex) const
  {
    return m_neighbors[vertex];
  }

  [[nodiscard]] ::dart::common::MemoryAllocator& allocator() const noexcept
  {
    return *m_allocator;
  }

  /// Largest neighbor-list size across all vertices (the graph's maximum
  /// degree). An upper bound on the color count returned by greedy coloring is
  /// `maxDegree() + 1`.
  [[nodiscard]] std::size_t maxDegree() const
  {
    std::size_t maximum = 0;
    for (const auto& list : m_neighbors) {
      maximum = std::max(maximum, list.size());
    }
    return maximum;
  }

private:
  void resizeNeighborRows(std::size_t vertexCount)
  {
    if (vertexCount < m_neighbors.size()) {
      m_neighbors.resize(vertexCount);
      return;
    }
    m_neighbors.reserve(vertexCount);
    while (m_neighbors.size() < vertexCount) {
      m_neighbors.emplace_back(
          ::dart::common::StlAllocator<std::uint32_t>{*m_allocator});
    }
  }

  ::dart::common::MemoryAllocator* m_allocator;
  NeighborRows m_neighbors;
  bool m_finalized = true;
};

/// A proper coloring of the vertex graph.
struct VertexColoring
{
  using VertexVector
      = std::vector<std::uint32_t, ::dart::common::StlAllocator<std::uint32_t>>;
  using VertexGroup = VertexVector;
  using VertexGroupAllocator = ::dart::common::StlAllocator<VertexGroup>;
  using VertexGroups = std::vector<VertexGroup, VertexGroupAllocator>;

  VertexColoring()
    : VertexColoring(::dart::common::MemoryAllocator::GetDefault())
  {
    // Intentionally empty.
  }

  explicit VertexColoring(::dart::common::MemoryAllocator& allocator)
    : colorOfVertex(::dart::common::StlAllocator<std::uint32_t>{allocator}),
      groups(VertexGroupAllocator{allocator}),
      m_allocator(&allocator)
  {
    // Intentionally empty.
  }

  /// `colorOfVertex[v]` is the color index assigned to vertex `v`.
  VertexVector colorOfVertex;
  /// `groups[c]` lists, in ascending index order, the vertices of color `c`.
  VertexGroups groups;

  [[nodiscard]] std::size_t colorCount() const noexcept
  {
    return groups.size();
  }

  void ensureColorCount(std::size_t colorCount)
  {
    if (colorCount < groups.size()) {
      groups.resize(colorCount);
      return;
    }
    groups.reserve(colorCount);
    while (groups.size() < colorCount) {
      groups.emplace_back(
          ::dart::common::StlAllocator<std::uint32_t>{*m_allocator});
    }
  }

private:
  ::dart::common::MemoryAllocator* m_allocator;
};

//==============================================================================
/// Greedily color the vertex graph so adjacent vertices receive distinct
/// colors.
///
/// Vertices are processed largest-degree-first (ties broken by ascending index)
/// — the Welsh-Powell ordering — which tends to use few colors, and the result
/// is deterministic. Each vertex takes the smallest color not used by its
/// already-colored neighbors. The color count is at most `maxDegree() + 1`.
inline VertexColoring greedyColorVertices(VertexAdjacency& adjacency)
{
  adjacency.finalize();
  const std::size_t vertexCount = adjacency.vertexCount();
  auto& allocator = adjacency.allocator();

  VertexColoring coloring(allocator);
  coloring.colorOfVertex.assign(vertexCount, kUncolored);

  // Welsh-Powell vertex order: descending degree, ascending index on ties.
  std::vector<std::uint32_t, ::dart::common::StlAllocator<std::uint32_t>> order(
      vertexCount, ::dart::common::StlAllocator<std::uint32_t>{allocator});
  for (std::size_t v = 0; v < vertexCount; ++v) {
    order[v] = static_cast<std::uint32_t>(v);
  }
  std::sort(
      order.begin(), order.end(), [&](std::uint32_t lhs, std::uint32_t rhs) {
        const std::size_t lhsDegree = adjacency.neighbors(lhs).size();
        const std::size_t rhsDegree = adjacency.neighbors(rhs).size();
        if (lhsDegree != rhsDegree) {
          return lhsDegree > rhsDegree;
        }
        return lhs < rhs;
      });

  std::vector<std::uint8_t, ::dart::common::StlAllocator<std::uint8_t>>
      usedColor(::dart::common::StlAllocator<std::uint8_t>{allocator});
  for (const std::uint32_t vertex : order) {
    const auto& neighbors = adjacency.neighbors(vertex);
    usedColor.assign(neighbors.size() + 1, 0u);
    for (const std::uint32_t neighbor : neighbors) {
      const std::uint32_t neighborColor = coloring.colorOfVertex[neighbor];
      if (neighborColor != kUncolored && neighborColor < usedColor.size()) {
        usedColor[neighborColor] = 1u;
      }
    }

    std::uint32_t color = 0;
    while (color < usedColor.size() && usedColor[color] != 0u) {
      ++color;
    }
    coloring.colorOfVertex[vertex] = color;
    if (color >= coloring.groups.size()) {
      coloring.ensureColorCount(color + 1);
    }
  }

  // Build the per-color groups in ascending vertex-index order so sweeps are
  // deterministic regardless of the processing order above.
  for (std::uint32_t v = 0; v < vertexCount; ++v) {
    const std::uint32_t color = coloring.colorOfVertex[v];
    coloring.groups[color].push_back(v);
  }
  return coloring;
}

//==============================================================================
/// True iff `coloring` is a proper coloring of `adjacency`: every vertex has a
/// color and no edge joins two equally-colored vertices. Used to verify the
/// conflict-free guarantee that makes within-color parallelism sound.
inline bool isProperColoring(
    const VertexAdjacency& adjacency, const VertexColoring& coloring)
{
  const std::size_t vertexCount = adjacency.vertexCount();
  if (coloring.colorOfVertex.size() != vertexCount) {
    return false;
  }
  for (std::uint32_t v = 0; v < vertexCount; ++v) {
    if (coloring.colorOfVertex[v] == kUncolored) {
      return false;
    }
    for (const std::uint32_t neighbor : adjacency.neighbors(v)) {
      if (coloring.colorOfVertex[neighbor] == coloring.colorOfVertex[v]) {
        return false;
      }
    }
  }
  return true;
}

} // namespace dart::simulation::detail::deformable_vbd
