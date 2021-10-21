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

#include <map>
#include <vector>

#include "dart/common/eigen_include.hpp"
#include "dart/math/geometry/tri_mesh.hpp"

namespace dart {
namespace math {

/// The class Icosphere represents an icosphere where the subdivision and radius
/// are configurable.
template <typename S_>
class Icosphere : public TriMesh<S_>
{
public:
  // Type aliases
  using S = S_;
  using Base = TriMesh<S>;
  using Index = typename Base::Index;
  using Vector3 = typename Base::Vector3;
  using Triangle = typename Base::Triangle;
  using Vertices = std::vector<Vector3>;
  using Normals = typename Base::Normals;
  using Triangles = std::vector<Triangle>;

  /// Returns the number of vertices of icosphere given subdivisions.
  static std::size_t getNumVertices(std::size_t subdivisions);

  /// Returns the number of edges of icosphere given subdivisions.
  static std::size_t getNumEdges(std::size_t subdivisions);

  /// Returns the number of triangles of icosphere given subdivisions.
  static std::size_t getNumTriangles(std::size_t subdivisions);

  /// Returns vertices and faces of icosahedron given radius.
  static std::pair<Vertices, Triangles> computeIcosahedron(S radius);

  /// Construct an icosphere given radius and subdivisions.
  ///
  /// \param[in] radius: The radius of the icosphere.
  /// \param[in] subdivisions: The number of subdividing an icosahedron. Passing
  /// 1 generates icosahedron without subdividing.
  Icosphere(S radius, std::size_t subdivisions);

  /// Returns the radius of the icosphere.
  S getRadius() const;

  /// Returns the number of subdivisions of the icosphere.
  std::size_t getNumSubdivisions() const;

private:
  /// Internal function to build icosphere given radius and subdivisions.
  void build();

  /// Radius of icosphere.
  S mRadius;

  /// Number of subdividing an icosahedron.
  std::size_t mSubdivisions;
};

using Icospheref = Icosphere<float>;
using Icosphered = Icosphere<double>;

} // namespace math
} // namespace dart

#include "dart/math/geometry/detail/icosphere_impl.hpp"
