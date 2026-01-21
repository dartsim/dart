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

#ifndef DART_MATH_POLYGONMESH_HPP_
#define DART_MATH_POLYGONMESH_HPP_

#include <dart/math/mesh.hpp>
#include <dart/math/tri_mesh.hpp>

#include <dart/export.hpp>

#include <initializer_list>
#include <vector>

namespace dart {
namespace math {

/// This class represents polygon meshes with variable-length faces.
template <typename S_>
class PolygonMesh : public Mesh<S_>
{
public:
  // Type aliases
  using S = S_;
  using Base = Mesh<S>;
  using Index = typename Base::Index;
  using Vector3 = typename Base::Vector3;
  using Vertices = typename Base::Vertices;
  using Normals = typename Base::Normals;
  using Face = std::vector<Index>;
  using Faces = std::vector<Face>;
  using TriMeshType = TriMesh<S>;

  /// Default constructor.
  PolygonMesh();

  /// Destructor.
  ~PolygonMesh() override = default;

  /// Reserves space for faces.
  void reserveFaces(std::size_t n);

  /// Reserves space for vertices.
  void reserveVertices(std::size_t n);

  /// Adds a vertex to the mesh.
  void addVertex(S x, S y, S z);

  /// Adds a vertex to the mesh.
  void addVertex(const Vector3& vertex);

  /// Reserves space for vertex normals.
  void reserveVertexNormals(std::size_t n);

  /// Adds a vertex normal to the mesh.
  void addVertexNormal(S x, S y, S z);

  /// Adds a vertex normal to the mesh.
  void addVertexNormal(const Vector3& normal);

  /// Adds a face to the mesh.
  void addFace(const Face& face);

  /// Adds a face to the mesh.
  void addFace(Face&& face);

  /// Adds a face to the mesh.
  void addFace(std::initializer_list<Index> indices);

  /// Returns true if the mesh contains faces.
  bool hasFaces() const;

  /// Returns the number of faces.
  std::size_t getNumFaces() const;

  /// Returns the faces of the mesh.
  const Faces& getFaces() const;

  /// Returns the faces of the mesh.
  Faces& getFaces();

  /// Clears all data in the mesh.
  void clear() override;

  /// Triangulates the polygon mesh into a TriMesh using ear clipping on a
  /// projected plane; faces are assumed simple and planar.
  TriMeshType triangulate() const;

protected:
  /// Faces of the mesh.
  Faces mFaces;
};

#if DART_OS_WINDOWS
extern template class PolygonMesh<double>;
#endif

using PolygonMeshf = PolygonMesh<float>;
using PolygonMeshd = PolygonMesh<double>;

} // namespace math
} // namespace dart

#include <dart/math/detail/polygon_mesh-impl.hpp>

#endif // DART_MATH_POLYGONMESH_HPP_
