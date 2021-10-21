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

#include "dart/math/export.hpp"
#include "dart/math/geometry/mesh.hpp"

namespace dart {
namespace math {

/// This class represents triangle meshes.
template <typename S_>
class TriMesh : public Mesh<S_>
{
public:
  // Type aliases
  using S = S_;
  using Base = Mesh<S>;
  using Index = typename Base::Index;
  using Vector3 = typename Base::Vector3;
  using Triangle = Eigen::Matrix<Index, 3, 1>;
  using Vertices = typename Base::Vertices;
  using Normals = typename Base::Normals;
  using Triangles = std::vector<Triangle>;

  /// Default constructor.
  TriMesh();

  /// Destructor
  ~TriMesh() override = default;

  static const std::string& GetType();

  const std::string& get_type() const override;

  void reserve_triangles(int size);

  /// Sets vertices and triangles.
  void set_triangles(const Vertices& vertices, const Triangles& triangles);

  void add_triangle(const Triangle& triangle);

  int get_num_triangles() const;

  /// Computes vertex normals.
  void compute_vertex_normals();

  /// Returns true if the mesh contains triangles.
  bool has_triangles() const;

  /// Returns true if the mesh contains triangle normals.
  bool has_triangle_normals() const;

  /// Returns the triangles of the mesh.
  const Triangles& get_triangles() const;

  /// Returns the triangle normals of the mesh.
  const Normals& get_triangle_normals() const;

  /// Clears all the data in the trimesh.
  void clear() override;

  /// Addition operator.
  TriMesh operator+(const TriMesh& other) const;

  /// Addition assignment operator.
  TriMesh& operator+=(const TriMesh& other);

  /// Computes the volume of this mesh, assuming that the mesh is watertight and
  /// orientable.
  S get_volume() const override;
  // TODO(JS): Make this to lazy-evaluate

  /// Generates a convex hull that encloses the trimesh.
  ///
  /// \param[in] optimize: (Optional) Whether to discard vertices that are not
  /// used in the convex hull.
  std::shared_ptr<TriMesh<S>> generate_convex_hull(bool optimize = true) const;

protected:
  /// Computes triangle normals.
  void compute_triangle_normals();

  /// Normalizes triangle normals.
  void normalize_triangle_normals();

  /// Triangle indices of the mesh.
  Triangles m_triangles;

  /// Triangle normals of the mesh.
  Normals m_triangle_normals;
};

extern template class DART_MATH_API TriMesh<double>;

using TriMeshf = TriMesh<float>;
using TriMeshd = TriMesh<double>;

} // namespace math
} // namespace dart

#include "dart/math/geometry/detail/tri_mesh_impl.hpp"
