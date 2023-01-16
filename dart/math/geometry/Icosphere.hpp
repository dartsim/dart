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

#include <dart/math/geometry/TriMesh.hpp>

#include <Eigen/Core>

#include <map>
#include <vector>

namespace dart::math {

/// The class Icosphere represents an icosphere where the subdivision and radius
/// are configurable.
template <typename S>
class Icosphere : public TriMesh<S>
{
public:
  // Type aliases
  using Scalar = S;
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

DART_TEMPLATE_CLASS_HEADER(MATH, Icosphere)

} // namespace dart::math

#include <dart/math/Constants.hpp>

#include <array>

namespace dart::math {

//==============================================================================
template <typename S>
std::size_t Icosphere<S>::getNumVertices(std::size_t subdivisions)
{
  std::size_t numVertices = 12;
  for (auto i = 0u; i < subdivisions; ++i)
    numVertices += getNumEdges(i);
  return numVertices;
}

//==============================================================================
template <typename S>
std::size_t Icosphere<S>::getNumEdges(std::size_t subdivisions)
{
  return getNumTriangles(subdivisions) / 2 * 3;
}

//==============================================================================
template <typename S>
std::size_t Icosphere<S>::getNumTriangles(std::size_t subdivisions)
{
  return 20 * std::pow(4, subdivisions);
}

//==============================================================================
template <typename S>
std::pair<typename Icosphere<S>::Vertices, typename Icosphere<S>::Triangles>
Icosphere<S>::computeIcosahedron(S radius)
{
  const S unitX = 1 / std::sqrt(1 + phi<S>() * phi<S>());
  const S unitZ = unitX * phi<S>();

  const S x = radius * unitX;
  const S z = radius * unitZ;

  std::vector<Vector3> vertices
      = {{-x, 0, z},
         {x, 0, z},
         {-x, 0, -z},
         {x, 0, -z},
         {0, z, x},
         {0, z, -x},
         {0, -z, x},
         {0, -z, -x},
         {z, x, 0},
         {-z, x, 0},
         {z, -x, 0},
         {-z, -x, 0}};

  static std::vector<Triangle> triangles
      = {{0, 4, 1},  {0, 9, 4},  {9, 5, 4},  {4, 5, 8},  {4, 8, 1},
         {8, 10, 1}, {8, 3, 10}, {5, 3, 8},  {5, 2, 3},  {2, 7, 3},
         {7, 10, 3}, {7, 6, 10}, {7, 11, 6}, {11, 0, 6}, {0, 1, 6},
         {6, 1, 10}, {9, 0, 11}, {9, 11, 2}, {9, 2, 5},  {7, 2, 11}};

  return std::make_pair(vertices, triangles);
}

//==============================================================================
template <typename S>
Icosphere<S>::Icosphere(S radius, std::size_t subdivisions)
  : mRadius(radius), mSubdivisions(subdivisions)
{
  static_assert(
      std::is_floating_point<S>::value,
      "Scalar must be a floating point type.");
  assert(radius > 0);

  build();
}

//==============================================================================
template <typename S>
S Icosphere<S>::getRadius() const
{
  return mRadius;
}

//==============================================================================
template <typename S>
std::size_t Icosphere<S>::getNumSubdivisions() const
{
  return mSubdivisions;
}

//==============================================================================
template <typename S>
void Icosphere<S>::build()
{
  // Reference: https://schneide.blog/2016/07/15/generating-an-icosphere-in-c/

  // Create icosahedron
  std::tie(this->mVertices, this->mTriangles) = computeIcosahedron(mRadius);

  // Return if no need to subdivide
  if (mSubdivisions == 0)
    return;

  // Create index map that is used for subdivision
  using IndexMap = std::map<std::pair<std::size_t, std::size_t>, std::size_t>;
  IndexMap midVertexIndices;

  // Create a temporary array of faces that is used for subdivision
  std::vector<Triangle> tmpFaces;
  if (mSubdivisions % 2) {
    this->mTriangles.reserve(getNumTriangles(mSubdivisions - 1));
    tmpFaces.reserve(getNumTriangles(mSubdivisions));
  } else {
    this->mTriangles.reserve(getNumTriangles(mSubdivisions));
    tmpFaces.reserve(getNumTriangles(mSubdivisions - 1));
  }

  // Create more intermediate variables that are used for subdivision
  std::vector<Triangle>* currFaces = &(this->mTriangles);
  std::vector<Triangle>* newFaces = &tmpFaces;
  std::array<std::size_t, 3> mid;

  // Subdivide icosahedron/icosphere iteratively. The key is to not duplicate
  // the newly created vertices and faces during each subdivision.
  for (std::size_t i = 0; i < mSubdivisions; ++i) {
    // Clear the array of faces that will store the faces of the subdivided
    // isosphere in this iteration. This is because the faces of the previous
    // isosphere are not reused.
    (*newFaces).clear();
    midVertexIndices.clear();

    // Iterate each face of the previous icosphere and divide the face into
    // four new faces.
    for (std::size_t j = 0; j < (*currFaces).size(); ++j) {
      const auto& outter = (*currFaces)[j];

      // Create vertices on the middle of edges if not already created.
      for (std::size_t k = 0; k < 3; ++k) {
        auto indexA = outter[k];
        auto indexB = outter[(k + 1) % 3];

        // Sort indices to guarantee that the key is unique for the same pairs
        // of indices.
        if (indexA > indexB)
          std::swap(indexA, indexB);

        // Check whether the mid vertex given index pair is already created.
        const auto result = midVertexIndices.insert(
            {{indexA, indexB}, this->mVertices.size()});
        const auto& inserted = result.second;
        if (inserted) {
          // Create a vertex on the middle of the edge where the length of the
          // vertex is equal to the radius of the icosphere.
          const auto& v1 = this->mVertices[indexA];
          const auto& v2 = this->mVertices[indexB];
          this->mVertices.emplace_back(mRadius * (v1 + v2).normalized());
        }

        mid[k] = result.first->second;
      }

      // Add four new faces.
      (*newFaces).emplace_back(Triangle(outter[0], mid[0], mid[2]));
      (*newFaces).emplace_back(Triangle(mid[0], outter[1], mid[1]));
      (*newFaces).emplace_back(Triangle(mid[0], mid[1], mid[2]));
      (*newFaces).emplace_back(Triangle(mid[2], mid[1], outter[2]));
    }

    // Swap the arrays of faces.
    std::swap(currFaces, newFaces);
  }

  // Assign faces if needed.
  this->mTriangles = *currFaces;
}

} // namespace dart::math
