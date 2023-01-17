/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#include <dart/math/Fwd.hpp>

#include <vector>

namespace dart::math {

/// Base class for meshes.
template <typename S>
class Mesh
{
public:
  // Type aliases
  using Scalar = S;
  using Index = std::size_t;
  using Vector3 = Matrix<S, 3, 1>;
  using Vertices = std::vector<Vector3>;
  using Normals = std::vector<Vector3>;
  using Indices = std::vector<Index>;

  /// Destructor.
  virtual ~Mesh();

  /// Returns true if the mesh contains vertices.
  bool hasVertices() const;

  /// Returns true if the mesh contains vertex normals.
  bool hasVertexNormals() const;

  /// Returns the vertices of the mesh.
  const Vertices& getVertices() const;

  /// Returns the vertex normals of the mesh.
  const Normals& getVertexNormals() const;

  /// Clears all the vertices and vertex normals.
  virtual void clear();

  /// Returns true if the mesh has no vertices.
  bool isEmpty() const;

  /// Translates the mesh vertices by adding \c translation to the vertices.
  void translate(const Vector3& translation);

  /// Addition operator.
  Mesh operator+(const Mesh& other) const;

  /// Addition assignment operator.
  Mesh& operator+=(const Mesh& other);

protected:
  /// Default constructor.
  Mesh();

  /// Normalizes the vertex normals.
  void normalizeVertexNormals();

  /// Vertices of the mesh.
  Vertices mVertices;

  /// Vertex normals of the mesh.
  Normals mVertexNormals;
};

DART_TEMPLATE_CLASS_HEADER(MATH, Mesh);

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

namespace dart::math {

//==============================================================================
template <typename S>
Mesh<S>::~Mesh()
{
  // Do nothing
}

//==============================================================================
template <typename S>
bool Mesh<S>::hasVertices() const
{
  return !mVertices.empty();
}

//==============================================================================
template <typename S>
bool Mesh<S>::hasVertexNormals() const
{
  return hasVertices() && mVertices.size() == mVertexNormals.size();
}

//==============================================================================
template <typename S>
const typename Mesh<S>::Vertices& Mesh<S>::getVertices() const
{
  return this->mVertices;
}

//==============================================================================
template <typename S>
const typename Mesh<S>::Normals& Mesh<S>::getVertexNormals() const
{
  return this->mVertexNormals;
}

//==============================================================================
template <typename S>
void Mesh<S>::clear()
{
  mVertices.clear();
  mVertexNormals.clear();
}

//==============================================================================
template <typename S>
bool Mesh<S>::isEmpty() const
{
  return !(this->hasVertices());
}

//==============================================================================
template <typename S>
void Mesh<S>::translate(const Vector3& translation)
{
  for (auto& vertex : mVertices) {
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
  if (other.isEmpty())
    return *this;

  // Insert vertex normals if both meshes have normals. Otherwise, clean the
  // vertex normals.
  if ((isEmpty() || hasVertexNormals()) && other.hasVertexNormals()) {
    mVertexNormals.insert(
        mVertexNormals.end(),
        other.mVertexNormals.begin(),
        other.mVertexNormals.end());
  } else {
    mVertexNormals.clear();
  }

  // Insert vertices
  mVertices.insert(
      mVertices.end(), other.mVertices.begin(), other.mVertices.end());

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
void Mesh<S>::normalizeVertexNormals()
{
  for (auto& normal : mVertexNormals) {
    normal.normalize();
  }
}

} // namespace dart::math
