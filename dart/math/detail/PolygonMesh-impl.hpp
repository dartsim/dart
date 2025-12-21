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

#ifndef DART_MATH_DETAIL_POLYGONMESH_IMPL_HPP_
#define DART_MATH_DETAIL_POLYGONMESH_IMPL_HPP_

#include <dart/math/PolygonMesh.hpp>

#include <utility>

namespace dart {
namespace math {

//==============================================================================
template <typename S>
PolygonMesh<S>::PolygonMesh()
{
  // Do nothing
}

//==============================================================================
template <typename S>
void PolygonMesh<S>::reserveFaces(std::size_t n)
{
  mFaces.reserve(n);
}

//==============================================================================
template <typename S>
void PolygonMesh<S>::reserveVertices(std::size_t n)
{
  this->mVertices.reserve(n);
}

//==============================================================================
template <typename S>
void PolygonMesh<S>::addVertex(S x, S y, S z)
{
  this->mVertices.emplace_back(x, y, z);
}

//==============================================================================
template <typename S>
void PolygonMesh<S>::addVertex(const Vector3& vertex)
{
  this->mVertices.push_back(vertex);
}

//==============================================================================
template <typename S>
void PolygonMesh<S>::reserveVertexNormals(std::size_t n)
{
  this->mVertexNormals.reserve(n);
}

//==============================================================================
template <typename S>
void PolygonMesh<S>::addVertexNormal(S x, S y, S z)
{
  this->mVertexNormals.emplace_back(x, y, z);
}

//==============================================================================
template <typename S>
void PolygonMesh<S>::addVertexNormal(const Vector3& normal)
{
  this->mVertexNormals.push_back(normal);
}

//==============================================================================
template <typename S>
void PolygonMesh<S>::addFace(const Face& face)
{
  mFaces.push_back(face);
}

//==============================================================================
template <typename S>
void PolygonMesh<S>::addFace(Face&& face)
{
  mFaces.push_back(std::move(face));
}

//==============================================================================
template <typename S>
void PolygonMesh<S>::addFace(std::initializer_list<Index> indices)
{
  mFaces.emplace_back(indices);
}

//==============================================================================
template <typename S>
bool PolygonMesh<S>::hasFaces() const
{
  return !mFaces.empty();
}

//==============================================================================
template <typename S>
std::size_t PolygonMesh<S>::getNumFaces() const
{
  return mFaces.size();
}

//==============================================================================
template <typename S>
const typename PolygonMesh<S>::Faces& PolygonMesh<S>::getFaces() const
{
  return mFaces;
}

//==============================================================================
template <typename S>
typename PolygonMesh<S>::Faces& PolygonMesh<S>::getFaces()
{
  return mFaces;
}

//==============================================================================
template <typename S>
void PolygonMesh<S>::clear()
{
  mFaces.clear();
  Base::clear();
}

//==============================================================================
template <typename S>
typename PolygonMesh<S>::TriMeshType PolygonMesh<S>::triangulate() const
{
  TriMeshType triMesh;

  triMesh.getVertices() = this->mVertices;
  if (this->hasVertexNormals()) {
    triMesh.getVertexNormals() = this->mVertexNormals;
  }

  std::size_t triangleCount = 0;
  for (const auto& face : mFaces) {
    if (face.size() >= 3) {
      triangleCount += face.size() - 2;
    }
  }
  triMesh.reserveTriangles(triangleCount);

  for (const auto& face : mFaces) {
    if (face.size() < 3) {
      continue;
    }
    const Index v0 = face[0];
    for (std::size_t i = 1; i + 1 < face.size(); ++i) {
      triMesh.addTriangle(v0, face[i], face[i + 1]);
    }
  }

  if (triMesh.hasTriangles()
      && triMesh.getVertexNormals().size() != triMesh.getVertices().size()) {
    triMesh.computeVertexNormals();
  }

  return triMesh;
}

} // namespace math
} // namespace dart

#endif // DART_MATH_DETAIL_POLYGONMESH_IMPL_HPP_
