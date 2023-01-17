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
#include <dart/math/geometry/Mesh.hpp>

#include <memory>

namespace dart::math {

/// A triangular mesh.
/// @tparam S The scalar type.
template <typename S>
class TriMesh : public Mesh<S>
{
public:
  // Type aliases
  using Scalar = S;
  using Base = Mesh<S>;
  using Index = typename Base::Index;
  using Vector3 = typename Base::Vector3;
  using Triangle = Matrix<Index, 3, 1>;
  using Vertices = typename Base::Vertices;
  using Normals = typename Base::Normals;
  using Triangles = std::vector<Triangle>;

  /// Default constructor.
  TriMesh();

  /// Destructor
  ~TriMesh() override = default;

  /// Sets vertices and triangles.
  void setTriangles(const Vertices& vertices, const Triangles& triangles);

  /// Reserves space for vertices.
  /// @param size The number of vertices to reserve space for.
  void reserveVertices(std::size_t size);

  /// Adds a vertex to the mesh.
  /// @param vertex The vertex to add.
  void addVertex(const Vector3& vertex);

  /// Adds a vertex to the mesh.
  /// @param x The X component of the vertex to add.
  /// @param y The Y component of the vertex to add.
  /// @param z The Z component of the vertex to add.
  void addVertex(S x, S y, S z);

  /// Reserves space for vertex normals.
  /// @param size The number of vertex normals to reserve space for.
  void reserveVertexNormals(std::size_t size);

  /// Adds a vertex normal to the mesh.
  /// @param normal The vertex normal to add.
  void addVertexNormal(const Vector3& normal);

  /// Adds a vertex normal to the mesh.
  /// @param x The X component of the vertex normal to add.
  /// @param y The Y component of the vertex normal to add.
  /// @param z The Z component of the vertex normal to add.
  void addVertexNormal(S x, S y, S z);

  /// Reserves space for triangles.
  /// @param size The number of triangles to reserve space for.
  void reserveTriangles(std::size_t size);

  /// Adds a triangle to the mesh.
  /// @param triangle The triangle to add.
  void addTriangle(const Triangle& triangle);

  /// Adds a triangle to the mesh.
  /// @param a The index of the first vertex of the triangle.
  /// @param b The index of the second vertex of the triangle.
  /// @param c The index of the third vertex of the triangle.
  void addTriangle(Index a, Index b, Index c);

  /// Computes vertex normals.
  void computeVertexNormals();

  /// Returns true if the mesh contains triangles.
  [[nodiscard]] bool hasTriangles() const;

  /// Returns true if the mesh contains triangle normals.
  [[nodiscard]] bool hasTriangleNormals() const;

  /// Returns the triangles of the mesh.
  [[nodiscard]] const Triangles& getTriangles() const;

  /// Returns the triangle normals of the mesh.
  [[nodiscard]] const Normals& getTriangleNormals() const;

  /// Clears all the data in the trimesh.
  void clear() override;

  /// Addition operator.
  TriMesh operator+(const TriMesh& other) const;

  /// Addition assignment operator.
  TriMesh& operator+=(const TriMesh& other);

  /// Generates a convex hull that encloses the trimesh.
  ///
  /// \param[in] optimize: (Optional) Whether to discard vertices that are not
  /// used in the convex hull.
  [[nodiscard]] std::shared_ptr<TriMesh<S>> generateConvexHull(
      bool optimize = true) const;

protected:
  /// Computes triangle normals.
  void computeTriangleNormals();

  /// Normalizes triangle normals.
  void normalizeTriangleNormals();

  /// Triangle indices of the mesh.
  Triangles mTriangles;

  /// Triangle normals of the mesh.
  Normals mTriangleNormals;
};

DART_TEMPLATE_CLASS_HEADER(MATH, TriMesh);

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

#include <dart/math/Geometry.hpp>

#include <Eigen/Geometry>

namespace dart::math {

//==============================================================================
template <typename S>
TriMesh<S>::TriMesh()
{
  // Do nothing
}

//==============================================================================
template <typename S>
void TriMesh<S>::setTriangles(
    const Vertices& vertices, const Triangles& triangles)
{
  clear();

  this->mVertices = vertices;
  mTriangles = triangles;
}

//==============================================================================
template <typename S>
void TriMesh<S>::reserveVertices(std::size_t size)
{
  this->mVertices.reserve(size);
}

//==============================================================================
template <typename S>
void TriMesh<S>::addVertex(const Vector3& vertex)
{
  this->mVertices.push_back(vertex);
}

//==============================================================================
template <typename S>
void TriMesh<S>::addVertex(S x, S y, S z)
{
  addVertex(Vector3(x, y, z));
}

//==============================================================================
template <typename S>
void TriMesh<S>::reserveVertexNormals(std::size_t size)
{
  this->mVertexNormals.reserve(size);
}

//==============================================================================
template <typename S>
void TriMesh<S>::addVertexNormal(const Vector3& vertex)
{
  this->mVertexNormals.push_back(vertex.normalized());
}

//==============================================================================
template <typename S>
void TriMesh<S>::addVertexNormal(S x, S y, S z)
{
  addVertexNormal(Vector3(x, y, z));
}

//==============================================================================
template <typename S>
void TriMesh<S>::reserveTriangles(std::size_t size)
{
  this->mTriangles.reserve(size);
}

//==============================================================================
template <typename S>
void TriMesh<S>::addTriangle(const Triangle& triangle)
{
  this->mTriangles.push_back(triangle);
}

//==============================================================================
template <typename S>
void TriMesh<S>::addTriangle(Index a, Index b, Index c)
{
  addTriangle(Triangle(a, b, c));
}

//==============================================================================
template <typename S>
void TriMesh<S>::computeVertexNormals()
{
  computeTriangleNormals();

  this->mVertexNormals.clear();
  this->mVertexNormals.resize(this->mVertices.size(), Vector3::Zero());

  for (auto i = 0u; i < mTriangles.size(); ++i) {
    auto& triangle = mTriangles[i];
    this->mVertexNormals[triangle[0]] += mTriangleNormals[i];
    this->mVertexNormals[triangle[1]] += mTriangleNormals[i];
    this->mVertexNormals[triangle[2]] += mTriangleNormals[i];
  }

  this->normalizeVertexNormals();
}

//==============================================================================
template <typename S>
bool TriMesh<S>::hasTriangles() const
{
  return !mTriangles.empty();
}

//==============================================================================
template <typename S>
bool TriMesh<S>::hasTriangleNormals() const
{
  return hasTriangles() && mTriangles.size() == mTriangleNormals.size();
}

//==============================================================================
template <typename S>
const typename TriMesh<S>::Triangles& TriMesh<S>::getTriangles() const
{
  return mTriangles;
}

//==============================================================================
template <typename S>
const typename TriMesh<S>::Normals& TriMesh<S>::getTriangleNormals() const
{
  return mTriangleNormals;
}

//==============================================================================
template <typename S>
void TriMesh<S>::clear()
{
  mTriangles.clear();
  mTriangleNormals.clear();
  Base::clear();
}

//==============================================================================
template <typename S>
TriMesh<S> TriMesh<S>::operator+(const TriMesh& other) const
{
  return (TriMesh(*this) += other);
}

//==============================================================================
template <typename S>
TriMesh<S>& TriMesh<S>::operator+=(const TriMesh& other)
{
  if (other.isEmpty())
    return *this;

  const auto oldNumVertices = this->mVertices.size();
  const auto oldNumTriangles = mTriangles.size();

  Base::operator+=(other);

  // Insert triangle normals if both meshes have normals. Otherwise, clean the
  // triangle normals.
  if ((!hasTriangles() || hasTriangleNormals()) && other.hasTriangleNormals()) {
    mTriangleNormals.insert(
        mTriangleNormals.end(),
        other.mTriangleNormals.begin(),
        other.mTriangleNormals.end());
  } else {
    mTriangleNormals.clear();
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
std::shared_ptr<TriMesh<S>> TriMesh<S>::generateConvexHull(bool optimize) const
{
  auto triangles = Triangles();
  auto vertices = Vertices();
  std::tie(vertices, triangles)
      = computeConvexHull3D<S, Index>(this->mVertices, optimize);

  auto mesh = std::make_shared<TriMesh<S>>();
  mesh->setTriangles(vertices, triangles);

  return mesh;
}

//==============================================================================
template <typename S>
void TriMesh<S>::computeTriangleNormals()
{
  mTriangleNormals.resize(mTriangles.size());

  for (auto i = 0u; i < mTriangles.size(); ++i) {
    auto& triangle = mTriangles[i];
    const Vector3 v01
        = this->mVertices[triangle[1]] - this->mVertices[triangle[0]];
    const Vector3 v02
        = this->mVertices[triangle[2]] - this->mVertices[triangle[0]];
    mTriangleNormals[i] = v01.cross(v02);
  }

  normalizeTriangleNormals();
}

//==============================================================================
template <typename S>
void TriMesh<S>::normalizeTriangleNormals()
{
  for (auto& normal : mTriangleNormals) {
    normal.normalize();
  }
}

} // namespace dart::math
