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

#ifndef CONVHULL_3D_INCLUDED
#define CONVHULL_3D_INCLUDED

#include <dart/math/detail/convhull.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <cstdlib>
#include <vector>

#ifdef CONVHULL_3D_USE_SINGLE_PRECISION
typedef float CH_FLOAT;
#else
typedef double CH_FLOAT;
#endif

typedef struct _ch_vertex
{
  union
  {
    CH_FLOAT v[3];
    struct
    {
      CH_FLOAT x, y, z;
    };
  };
} ch_vertex;

typedef ch_vertex ch_vec3;

inline void convhull_3d_build(
    ch_vertex* const in_vertices, const int nVert, int** out_faces, int* nOut_faces)
{
  if (out_faces != nullptr) {
    *out_faces = nullptr;
  }
  if (nOut_faces != nullptr) {
    *nOut_faces = 0;
  }
  if (in_vertices == nullptr || nVert <= 0 || out_faces == nullptr
      || nOut_faces == nullptr) {
    return;
  }

  std::vector<Eigen::Matrix<CH_FLOAT, 3, 1>> vertices;
  vertices.reserve(static_cast<std::size_t>(nVert));
  for (int i = 0; i < nVert; ++i) {
    vertices.emplace_back(in_vertices[i].x, in_vertices[i].y, in_vertices[i].z);
  }

  std::vector<int> faces;
  int numFaces = 0;
  dart::math::detail::convexHull3dBuild(vertices, faces, numFaces);
  if (faces.empty()) {
    return;
  }

  const auto bytes = faces.size() * sizeof(int);
  *out_faces = static_cast<int*>(std::malloc(bytes));
  if (*out_faces == nullptr) {
    *nOut_faces = 0;
    return;
  }

  std::copy(faces.begin(), faces.end(), *out_faces);
  *nOut_faces = numFaces;
}

inline void convhull_3d_build_alloc(
    ch_vertex* const in_vertices,
    const int nVert,
    int** out_faces,
    int* nOut_faces,
    void* allocator)
{
  (void)allocator;
  convhull_3d_build(in_vertices, nVert, out_faces, nOut_faces);
}

#endif /* CONVHULL_3D_INCLUDED */
