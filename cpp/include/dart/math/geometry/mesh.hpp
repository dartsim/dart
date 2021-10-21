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

#include <vector>

#include "dart/common/eigen_include.hpp"
#include "dart/math/geometry/geometry3.hpp"

namespace dart {
namespace math {

/// Base class for meshes.
template <typename S_>
class Mesh : public Geometry3<S_>
{
public:
  // Type aliases
  using S = S_;
  using Index = std::size_t;
  using Vector3 = Eigen::Matrix<S, 3, 1>;
  using Vertices = std::vector<Vector3>;
  using Normals = std::vector<Vector3>;
  using Indices = std::vector<Index>;

  /// Destructor.
  virtual ~Mesh();

  void reserve_vertices(int size);

  void add_vertex(const Vector3& vertex);

  int get_num_vertices() const;

  /// Returns true if the mesh contains vertices.
  bool has_vertices() const;

  /// Returns the vertices of the mesh.
  const Vertices& get_vertices() const;

  void reserve_vertex_normals(int size);

  void add_vertex_normal(const Vector3& normal);

  int get_num_vertex_normals() const;

  /// Returns true if the mesh contains vertex normals.
  bool has_vertex_normals() const;

  /// Returns the vertex normals of the mesh.
  const Normals& get_vertex_normals() const;

  /// Clears all the vertices and vertex normals.
  virtual void clear();

  /// Returns true if the mesh has no vertices.
  bool is_empty() const;

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
  void normalize_vertex_vormals();

  /// Vertices of the mesh.
  Vertices m_vertices;

  /// Vertex normals of the mesh.
  Normals m_vertex_normals;
};

using Meshf = Mesh<float>;
using Meshd = Mesh<double>;

} // namespace math
} // namespace dart

#include "dart/math/geometry/detail/mesh_impl.hpp"
