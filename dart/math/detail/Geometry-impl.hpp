/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#ifndef DART_MATH_DETAIL_GEOMETRY_IMPL_HPP_
#define DART_MATH_DETAIL_GEOMETRY_IMPL_HPP_

#include "dart/math/Geometry.hpp"

#include <unordered_map>

#include "dart/external/convhull_3d/convhull_3d.h"

namespace dart {
namespace math {

//==============================================================================
template <typename S, typename Index>
std::tuple<
    std::vector<Eigen::Matrix<S, 3, 1>>,
    std::vector<Eigen::Matrix<Index, 3, 1>>>
discardUnusedVertices(
    const std::vector<Eigen::Matrix<S, 3, 1>>& vertices,
    const std::vector<Eigen::Matrix<Index, 3, 1>>& triangles)
{
  auto newVertices = std::vector<Eigen::Matrix<S, 3, 1>>();
  auto newTriangles = std::vector<Eigen::Matrix<Index, 3, 1>>();
  newTriangles.resize(triangles.size());
  auto indexMap = std::unordered_map<Index, Index>();
  auto newIndex = 0;

  for (auto i = 0u; i < triangles.size(); ++i)
  {
    const auto& triangle = triangles[i];
    auto& newTriangle = newTriangles[i];

    for (auto j = 0u; j < 3; ++j)
    {
      const auto result
          = indexMap.insert(std::make_pair(triangle[j], newIndex));
      const bool& inserted = result.second;
      if (inserted)
      {
        newVertices.push_back(vertices[triangle[j]]);
        newIndex++;
      }

      newTriangle[j] = indexMap[triangle[j]];
    }
  }

  return std::make_tuple(newVertices, newTriangles);
}

//==============================================================================
template <typename S, typename Index>
std::tuple<
    std::vector<Eigen::Matrix<S, 3, 1>>,
    std::vector<Eigen::Matrix<Index, 3, 1>>>
computeConvexHull3D(
    const std::vector<Eigen::Matrix<S, 3, 1>>& inputVertices, bool optimize)
{
  ch_vertex* vertices = new ch_vertex[inputVertices.size()];

  for (auto i = 0u; i < inputVertices.size(); ++i)
  {
    const Eigen::Matrix<S, 3, 1>& inputV = inputVertices[i];
    ch_vertex& v = vertices[i];
    v.x = inputV[0];
    v.y = inputV[1];
    v.z = inputV[2];
  }

  int* faces = nullptr;
  int numFaces = 0;

  convhull_3d_build(vertices, inputVertices.size(), &faces, &numFaces);

  std::vector<Eigen::Matrix<Index, 3, 1>> eigenFaces;
  eigenFaces.reserve(numFaces);

  for (auto i = 0; i < numFaces; ++i)
  {
    const auto index1 = faces[3 * i];
    const auto index2 = faces[3 * i + 1];
    const auto index3 = faces[3 * i + 2];

    eigenFaces.emplace_back(index1, index2, index3);
  }

  free(faces);
  delete[] vertices;

  if (optimize)
    return discardUnusedVertices<S, Index>(inputVertices, eigenFaces);
  else
    return std::make_pair(inputVertices, eigenFaces);
}

} // namespace math
} // namespace dart

#endif // DART_MATH_DETAIL_GEOMETRY_IMPL_HPP_
