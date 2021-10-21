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

#include "dart/math/geometry/mesh.hpp"

namespace dart {
namespace math {

//==============================================================================
template <typename S>
Mesh<S>::~Mesh()
{
  // Do nothing
}

//==============================================================================
template <typename S>
void Mesh<S>::reserve_vertices(int size)
{
  m_vertices.reserve(size);
}

//==============================================================================
template <typename S>
void Mesh<S>::add_vertex(const Vector3& vertex)
{
  m_vertices.push_back(vertex);
}

//==============================================================================
template <typename S>
int Mesh<S>::get_num_vertices() const
{
  return m_vertices.size();
}

//==============================================================================
template <typename S>
bool Mesh<S>::has_vertices() const
{
  return !m_vertices.empty();
}

//==============================================================================
template <typename S>
void Mesh<S>::reserve_vertex_normals(int size)
{
  m_vertex_normals.reserve(size);
}

//==============================================================================
template <typename S>
void Mesh<S>::add_vertex_normal(const Vector3& vertex)
{
  m_vertex_normals.push_back(vertex);
}

//==============================================================================
template <typename S>
int Mesh<S>::get_num_vertex_normals() const
{
  return m_vertex_normals.size();
}

//==============================================================================
template <typename S>
bool Mesh<S>::has_vertex_normals() const
{
  return has_vertices() && m_vertices.size() == m_vertex_normals.size();
}

//==============================================================================
template <typename S>
const typename Mesh<S>::Vertices& Mesh<S>::get_vertices() const
{
  return this->m_vertices;
}

//==============================================================================
template <typename S>
const typename Mesh<S>::Normals& Mesh<S>::get_vertex_normals() const
{
  return this->m_vertex_normals;
}

//==============================================================================
template <typename S>
void Mesh<S>::clear()
{
  m_vertices.clear();
  m_vertex_normals.clear();
}

//==============================================================================
template <typename S>
bool Mesh<S>::is_empty() const
{
  return !(this->has_vertices());
}

//==============================================================================
template <typename S>
void Mesh<S>::translate(const Vector3& translation)
{
  for (auto& vertex : m_vertices) {
    vertex += translation;
  }
}

//==============================================================================
template <typename S>
Mesh<S> Mesh<S>::operator+(const Mesh& other) const
{
  return (Mesh(*this) += other);
}

//==============================================================================
template <typename S>
Mesh<S>& Mesh<S>::operator+=(const Mesh& other)
{
  if (other.is_empty())
    return *this;

  // Insert vertex normals if both meshes have normals. Otherwise, clean the
  // vertex normals.
  if ((is_empty() || has_vertex_normals()) && other.has_vertex_normals()) {
    m_vertex_normals.insert(
        m_vertex_normals.end(),
        other.m_vertex_normals.begin(),
        other.m_vertex_normals.end());
  } else {
    m_vertex_normals.clear();
  }

  // Insert vertices
  m_vertices.insert(
      m_vertices.end(), other.m_vertices.begin(), other.m_vertices.end());

  return *this;
}

//==============================================================================
template <typename S>
Mesh<S>::Mesh()
{
  // Do nothing
}

//==============================================================================
template <typename S>
void Mesh<S>::normalize_vertex_vormals()
{
  for (auto& normal : m_vertex_normals) {
    normal.normalize();
  }
}

} // namespace math
} // namespace dart
