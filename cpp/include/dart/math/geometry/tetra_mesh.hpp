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

#include <memory>

#include "dart/math/geometry/mesh.hpp"

namespace dart {
namespace math {

/// This class represents triangle meshes.
template <typename S_>
class TetraMesh : public Mesh<S_>
{
public:
  // Type aliases
  using S = S_;
  using Base = Mesh<S>;
  using Index = typename Base::Index;
  using Vector3 = typename Base::Vector3;
  using Vector4 = Eigen::Matrix<S, 4, 1>;
  using Triangle = Eigen::Matrix<Index, 3, 1>;
  using Tetra = Eigen::Matrix<Index, 3, 1>;
  using Vertices = typename Base::Vertices;
  using Normals = typename Base::Normals;
  using Triangles = std::vector<Triangle>;
  using Tetras = std::vector<Tetra>;

  /// Default constructor.
  TetraMesh();

  /// Destructor
  ~TetraMesh() override = default;

  static const std::string& GetType();

  const std::string& get_type() const override;

  /// Sets vertices and triangles.
  void setTriangles(const Vertices& vertices, const Triangles& triangles);

  /// Computes vertex normals.
  void computeVertexNormals();

  /// Returns true if the mesh contains triangles.
  bool hasTriangles() const;

  /// Returns true if the mesh contains triangle normals.
  bool hasTriangleNormals() const;

  /// Returns the triangles of the mesh.
  const Triangles& get_triangles() const;

  /// Returns the triangle normals of the mesh.
  const Normals& getTriangleNormals() const;

  /// Clears all the data in the trimesh.
  void clear() override;

  /// Addition operator.
  TetraMesh operator+(const TetraMesh& other) const;

  /// Addition assignment operator.
  TetraMesh& operator+=(const TetraMesh& other);

  /// Generates a convex hull that encloses the trimesh.
  ///
  /// \param[in] optimize: (Optional) Whether to discard vertices that are not
  /// used in the convex hull.
  std::shared_ptr<TetraMesh<S>> generate_convex_hull(
      bool optimize = true) const;

protected:
  /// Computes triangle normals.
  void computeTriangleNormals();

  /// Normalizes triangle normals.
  void normalizeTriangleNormals();

  /// Tetra indices of the mesh.
  Triangles mTriangles;

  /// Tetra indices of the mesh.
  Tetras mTetras;

  /// Tetra normals of the mesh.
  Normals mTetraNormals;
};

extern template class DART_MATH_API TetraMesh<double>;

using TetraMeshf = TetraMesh<float>;
using TetraMeshd = TetraMesh<double>;

} // namespace math
} // namespace dart

#include "dart/math/geometry/detail/tetra_mesh_impl.hpp"
