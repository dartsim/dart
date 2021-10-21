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

#pragma once

#include "dart/math/geometry/tetra_mesh.hpp"
#include "dart/math/geometry/util.hpp"

namespace dart {
namespace math {

//==============================================================================
template <typename S>
TetraMesh<S>::TetraMesh()
{
  // Do nothing
}

//==============================================================================
template <typename S>
const std::string& TetraMesh<S>::getStaticType()
{
  static const std::string type("TetraMesh");
  return type;
}

//==============================================================================
template <typename S>
const std::string& TetraMesh<S>::get_type() const
{
  return getStaticType();
}

//==============================================================================
template <typename S>
void TetraMesh<S>::setTriangles(
    const Vertices& vertices, const Triangles& triangles)
{
  clear();

  this->m_vertices = vertices;
  mTriangles = triangles;
}

//==============================================================================
template <typename S>
void TetraMesh<S>::computeVertexNormals()
{
  computeTriangleNormals();

  this->m_vertex_normals.clear();
  this->m_vertex_normals.resize(this->m_vertices.size(), Vector3::Zero());

  for (auto i = 0u; i < mTriangles.size(); ++i) {
    auto& triangle = mTriangles[i];
    this->m_vertex_normals[triangle[0]] += mTetraNormals[i];
    this->m_vertex_normals[triangle[1]] += mTetraNormals[i];
    this->m_vertex_normals[triangle[2]] += mTetraNormals[i];
  }

  this->normalize_vertex_vormals();
}

//==============================================================================
template <typename S>
bool TetraMesh<S>::hasTriangles() const
{
  return !mTriangles.empty();
}

//==============================================================================
template <typename S>
bool TetraMesh<S>::hasTriangleNormals() const
{
  return hasTriangles() && mTriangles.size() == mTetraNormals.size();
}

//==============================================================================
template <typename S>
const typename TetraMesh<S>::Triangles& TetraMesh<S>::get_triangles() const
{
  return mTriangles;
}

//==============================================================================
template <typename S>
const typename TetraMesh<S>::Normals& TetraMesh<S>::getTriangleNormals() const
{
  return mTetraNormals;
}

//==============================================================================
template <typename S>
void TetraMesh<S>::clear()
{
  mTriangles.clear();
  mTetraNormals.clear();
  Base::clear();
}

//==============================================================================
template <typename S>
TetraMesh<S> TetraMesh<S>::operator+(const TetraMesh& other) const
{
  return (TetraMesh(*this) += other);
}

//==============================================================================
template <typename S>
TetraMesh<S>& TetraMesh<S>::operator+=(const TetraMesh& other)
{
  if (other.is_empty())
    return *this;

  const auto oldNumVertices = this->m_vertices.size();
  const auto oldNumTriangles = mTriangles.size();

  Base::operator+=(other);

  // Insert triangle normals if both meshes have normals. Otherwise, clean the
  // triangle normals.
  if ((!hasTriangles() || hasTriangleNormals()) && other.hasTriangleNormals()) {
    mTetraNormals.insert(
        mTetraNormals.end(),
        other.mTetraNormals.begin(),
        other.mTetraNormals.end());
  } else {
    mTetraNormals.clear();
  }

  const Triangle offset = Triangle::Constant(oldNumVertices);
  mTriangles.resize(mTriangles.size() + other.mTriangles.size());
  for (auto i = 0u; i < other.mTriangles.size(); ++i) {
    mTriangles[i + oldNumTriangles] = other.mTriangles[i] + offset;
  }

  return *this;
}

//==============================================================================
template <typename S>
std::shared_ptr<TetraMesh<S>> TetraMesh<S>::generateConvexHull(
    bool optimize) const
{
  auto triangles = Triangles();
  auto vertices = Vertices();
  std::tie(vertices, triangles)
      = compute_convex_hull_3d<S, Index>(this->m_vertices, optimize);

  auto mesh = std::make_shared<TetraMesh<S>>();
  mesh->setTriangles(vertices, triangles);

  return mesh;
}

//==============================================================================
template <typename S>
void TetraMesh<S>::computeTriangleNormals()
{
  mTetraNormals.resize(mTriangles.size());

  for (auto i = 0u; i < mTriangles.size(); ++i) {
    auto& triangle = mTriangles[i];
    const Vector3 v01
        = this->m_vertices[triangle[1]] - this->m_vertices[triangle[0]];
    const Vector3 v02
        = this->m_vertices[triangle[2]] - this->m_vertices[triangle[0]];
    mTetraNormals[i] = v01.cross(v02);
  }

  normalizeTriangleNormals();
}

//==============================================================================
template <typename S>
void TetraMesh<S>::normalizeTriangleNormals()
{
  for (auto& normal : mTetraNormals) {
    normal.normalize();
  }
}

} // namespace math
} // namespace dart
