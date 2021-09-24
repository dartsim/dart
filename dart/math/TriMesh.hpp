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

#ifndef DART_MATH_TRIMESH_HPP_
#define DART_MATH_TRIMESH_HPP_

#include <memory>

#include "dart/math/Mesh.hpp"

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

  /// Sets vertices and triangles.
  void setTriangles(const Vertices& vertices, const Triangles& triangles);

  /// Computes vertex normals.
  void computeVertexNormals();

  /// Returns true if the mesh contains triangles.
  bool hasTriangles() const;

  /// Returns true if the mesh contains triangle normals.
  bool hasTriangleNormals() const;

  /// Returns the triangles of the mesh.
  const Triangles& getTriangles() const;

  /// Returns the triangle normals of the mesh.
  const Normals& getTriangleNormals() const;

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
  std::shared_ptr<TriMesh<S>> generateConvexHull(bool optimize = true) const;

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

extern template class TriMesh<double>;

using TriMeshf = TriMesh<float>;
using TriMeshd = TriMesh<double>;

} // namespace math
} // namespace dart

#include "dart/math/detail/TriMesh-impl.hpp"

#endif // DART_MATH_TRIMESH_HPP_
