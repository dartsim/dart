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

#pragma once

#include <dart/collision/dart/narrow_phase/Gjk-impl.hpp>

#include <memory>
#include <vector>

namespace dart::collision::native::detail {

struct EpaFace
{
  std::array<int, 3> vertices;
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  double distance = 0.0;
  bool valid = true;
};

struct EpaEdge
{
  int a = 0;
  int b = 0;
};

struct EpaScratch
{
  std::vector<SupportPoint> vertices;
  std::vector<EpaFace> faces;
  std::vector<EpaEdge> edges;
  bool inUse = false;
  std::unique_ptr<EpaScratch> nested;
};

struct EpaScratchScope
{
  EpaScratch& scratch;

  explicit EpaScratchScope(EpaScratch& storage) : scratch(storage)
  {
    scratch.inUse = true;
  }

  ~EpaScratchScope()
  {
    scratch.inUse = false;
  }
};

// Shared across callable types on this thread, with a slot per nesting level.
EpaScratch& epaScratch();
void addIfUniqueEdge(std::vector<EpaEdge>& edges, int a, int b);
bool addFace(
    std::vector<EpaFace>& faces,
    const std::vector<SupportPoint>& vertices,
    int a,
    int b,
    int c);
void fillPenetrationResult(
    const EpaFace& closestFace,
    const std::vector<SupportPoint>& vertices,
    double closestDist,
    EpaResult& result);

template <typename SupportA, typename SupportB>
EpaResult penetrationT(
    const SupportA& supportA,
    const SupportB& supportB,
    const GjkSimplex& initialSimplex)
{
  EpaResult result;
  auto& scratch = epaScratch();
  const EpaScratchScope scope(scratch);
  auto& vertices = scratch.vertices;
  auto& faces = scratch.faces;
  auto& edges = scratch.edges;
  vertices.clear();
  faces.clear();
  edges.clear();

  if (initialSimplex.size < 4) {
    return result;
  }

  vertices.reserve(64);
  for (int i = 0; i < initialSimplex.size; ++i) {
    vertices.push_back(initialSimplex.points[i]);
  }

  faces.reserve(64);

  addFace(faces, vertices, 0, 1, 2);
  addFace(faces, vertices, 0, 3, 1);
  addFace(faces, vertices, 0, 2, 3);
  addFace(faces, vertices, 1, 3, 2);

  for (int iteration = 0; iteration < Epa::kMaxIterations; ++iteration) {
    int closestFaceIdx = -1;
    double closestDist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < faces.size(); ++i) {
      if (faces[i].valid && faces[i].distance < closestDist) {
        closestDist = faces[i].distance;
        closestFaceIdx = static_cast<int>(i);
      }
    }

    if (closestFaceIdx < 0) {
      break;
    }

    const EpaFace& closestFace = faces[closestFaceIdx];
    const Eigen::Vector3d& faceNormal = closestFace.normal;

    SupportPoint newPoint = computeSupportT(supportA, supportB, faceNormal);
    const double newDist = newPoint.v.dot(faceNormal);

    if (newDist - closestDist < Epa::kTolerance) {
      fillPenetrationResult(closestFace, vertices, closestDist, result);
      return result;
    }

    const int newVertexIdx = static_cast<int>(vertices.size());
    vertices.push_back(newPoint);

    edges.clear();
    edges.reserve(32);

    for (size_t i = 0; i < faces.size(); ++i) {
      if (!faces[i].valid) {
        continue;
      }

      const int v0 = faces[i].vertices[0];
      if (faces[i].normal.dot(newPoint.v - vertices[v0].v) > 0.0) {
        faces[i].valid = false;
        addIfUniqueEdge(edges, faces[i].vertices[0], faces[i].vertices[1]);
        addIfUniqueEdge(edges, faces[i].vertices[1], faces[i].vertices[2]);
        addIfUniqueEdge(edges, faces[i].vertices[2], faces[i].vertices[0]);
      }
    }

    for (const auto& edge : edges) {
      addFace(faces, vertices, edge.a, edge.b, newVertexIdx);
    }
  }

  return result;
}

} // namespace dart::collision::native::detail
