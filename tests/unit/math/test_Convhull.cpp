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

#include <dart/math/detail/convhull.hpp>

#include <gtest/gtest.h>

#include <set>
#include <span>
#include <unordered_set>
#include <vector>

#include <cmath>

using dart::math::detail::convexHull3dBuild;

//==============================================================================
TEST(ConvhullInternal, SortFloatAndInt)
{
  float values[] = {3.0f, 1.0f, 2.0f};
  float* outValues = nullptr;
  int indices[] = {0, 0, 0};
  dart::math::detail::convhull_internal::sortFloat(
      values, outValues, indices, 3, true);

  EXPECT_FLOAT_EQ(values[0], 3.0f);
  EXPECT_FLOAT_EQ(values[1], 2.0f);
  EXPECT_FLOAT_EQ(values[2], 1.0f);

  int ints[] = {3, 1, 2};
  dart::math::detail::convhull_internal::sortInt(ints, 3);

  EXPECT_EQ(ints[0], 1);
  EXPECT_EQ(ints[1], 2);
  EXPECT_EQ(ints[2], 3);
}

//==============================================================================
// Typed Test Setup - Test both float and double precision
//==============================================================================
template <typename T>
class ConvexHullTest : public ::testing::Test
{
protected:
  using Scalar = T;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
};

using ScalarTypes = ::testing::Types<float, double>;
TYPED_TEST_SUITE(ConvexHullTest, ScalarTypes);

//==============================================================================
// Basic Tests
//==============================================================================
TYPED_TEST(ConvexHullTest, EmptyInput)
{
  using Vector3 = typename TestFixture::Vector3;

  std::vector<Vector3> vertices;
  std::vector<int> faces;
  int numFaces = 0;

  convexHull3dBuild(std::span<const Vector3>(vertices), faces, numFaces);

  EXPECT_EQ(numFaces, 0);
  EXPECT_TRUE(faces.empty());
}

//==============================================================================
TYPED_TEST(ConvexHullTest, TooFewVertices)
{
  using Scalar = typename TestFixture::Scalar;
  using Vector3 = typename TestFixture::Vector3;

  std::vector<Vector3> vertices(3);
  vertices[0] = Vector3(Scalar(0), Scalar(0), Scalar(0));
  vertices[1] = Vector3(Scalar(1), Scalar(0), Scalar(0));
  vertices[2] = Vector3(Scalar(0), Scalar(1), Scalar(0));

  std::vector<int> faces;
  int numFaces = 0;

  convexHull3dBuild(std::span<const Vector3>(vertices), faces, numFaces);

  EXPECT_EQ(numFaces, 0);
  EXPECT_TRUE(faces.empty());
}

//==============================================================================
TYPED_TEST(ConvexHullTest, Tetrahedron)
{
  using Scalar = typename TestFixture::Scalar;
  using Vector3 = typename TestFixture::Vector3;

  std::vector<Vector3> vertices(4);
  vertices[0] = Vector3(Scalar(0), Scalar(0), Scalar(0));
  vertices[1] = Vector3(Scalar(1), Scalar(0), Scalar(0));
  vertices[2] = Vector3(Scalar(0), Scalar(1), Scalar(0));
  vertices[3] = Vector3(Scalar(0), Scalar(0), Scalar(1));

  std::vector<int> faces;
  int numFaces = 0;
  convexHull3dBuild(std::span<const Vector3>(vertices), faces, numFaces);

  // A tetrahedron should have exactly 4 triangular faces
  ASSERT_EQ(numFaces, 4) << "Tetrahedron must have exactly 4 faces";
  ASSERT_EQ(faces.size(), 12u) << "4 faces * 3 vertices = 12 indices";

  // All vertices should be used (each vertex appears in the hull)
  std::set<int> usedVertices;
  for (const auto& idx : faces) {
    ASSERT_GE(idx, 0) << "Vertex index must be non-negative";
    ASSERT_LT(idx, 4) << "Vertex index must be < number of vertices";
    usedVertices.insert(idx);
  }
  EXPECT_EQ(usedVertices.size(), 4u) << "All 4 vertices should be used";
}

//==============================================================================
TYPED_TEST(ConvexHullTest, Cube)
{
  using Scalar = typename TestFixture::Scalar;
  using Vector3 = typename TestFixture::Vector3;

  std::vector<Vector3> vertices(8);
  vertices[0] = Vector3(Scalar(0), Scalar(0), Scalar(0));
  vertices[1] = Vector3(Scalar(1), Scalar(0), Scalar(0));
  vertices[2] = Vector3(Scalar(0), Scalar(1), Scalar(0));
  vertices[3] = Vector3(Scalar(1), Scalar(1), Scalar(0));
  vertices[4] = Vector3(Scalar(0), Scalar(0), Scalar(1));
  vertices[5] = Vector3(Scalar(1), Scalar(0), Scalar(1));
  vertices[6] = Vector3(Scalar(0), Scalar(1), Scalar(1));
  vertices[7] = Vector3(Scalar(1), Scalar(1), Scalar(1));

  std::vector<int> faces;
  int numFaces = 0;
  convexHull3dBuild(std::span<const Vector3>(vertices), faces, numFaces);

  // A cube has 6 square faces, each split into 2 triangles = 12 faces
  ASSERT_EQ(numFaces, 12) << "Cube must have exactly 12 triangular faces";
  ASSERT_EQ(faces.size(), 36u) << "12 faces * 3 vertices = 36 indices";

  // All vertices should be used (all 8 cube vertices are on the hull)
  std::set<int> usedVertices;
  for (const auto& idx : faces) {
    ASSERT_GE(idx, 0) << "Vertex index must be non-negative";
    ASSERT_LT(idx, static_cast<int>(vertices.size()))
        << "Vertex index must be < number of vertices";
    usedVertices.insert(idx);
  }
  EXPECT_EQ(usedVertices.size(), 8u) << "All 8 cube vertices should be used";
}

//==============================================================================
TYPED_TEST(ConvexHullTest, CubeWithInternalPoint)
{
  using Scalar = typename TestFixture::Scalar;
  using Vector3 = typename TestFixture::Vector3;

  std::vector<Vector3> vertices(9);
  // Cube vertices
  vertices[0] = Vector3(Scalar(0), Scalar(0), Scalar(0));
  vertices[1] = Vector3(Scalar(1), Scalar(0), Scalar(0));
  vertices[2] = Vector3(Scalar(0), Scalar(1), Scalar(0));
  vertices[3] = Vector3(Scalar(1), Scalar(1), Scalar(0));
  vertices[4] = Vector3(Scalar(0), Scalar(0), Scalar(1));
  vertices[5] = Vector3(Scalar(1), Scalar(0), Scalar(1));
  vertices[6] = Vector3(Scalar(0), Scalar(1), Scalar(1));
  vertices[7] = Vector3(Scalar(1), Scalar(1), Scalar(1));
  // Internal point (well inside to avoid numerical precision issues)
  vertices[8] = Vector3(Scalar(0.5), Scalar(0.5), Scalar(0.5));

  std::vector<int> faces;
  int numFaces = 0;
  convexHull3dBuild(std::span<const Vector3>(vertices), faces, numFaces);

  // Should produce cube hull (12 faces) or slightly more if noise perturbs
  // internal point For double precision: exactly 12 faces For float precision:
  // noise perturbation can cause internal point to appear on hull
  EXPECT_GE(numFaces, 12) << "Should have at least 12 faces (cube)";
  EXPECT_LE(numFaces, 16)
      << "Should have at most 16 faces (slightly perturbed)";
  EXPECT_EQ(faces.size(), static_cast<size_t>(numFaces * 3));

  // All 8 cube vertices should be used
  std::set<int> usedVertices(faces.begin(), faces.end());
  for (int i = 0; i < 8; ++i) {
    EXPECT_GT(usedVertices.count(i), 0u)
        << "Cube vertex " << i << " should be on the convex hull";
  }
}

//==============================================================================
TYPED_TEST(ConvexHullTest, Octahedron)
{
  using Scalar = typename TestFixture::Scalar;
  using Vector3 = typename TestFixture::Vector3;

  std::vector<Vector3> vertices(6);
  vertices[0] = Vector3(Scalar(1), Scalar(0), Scalar(0));
  vertices[1] = Vector3(Scalar(-1), Scalar(0), Scalar(0));
  vertices[2] = Vector3(Scalar(0), Scalar(1), Scalar(0));
  vertices[3] = Vector3(Scalar(0), Scalar(-1), Scalar(0));
  vertices[4] = Vector3(Scalar(0), Scalar(0), Scalar(1));
  vertices[5] = Vector3(Scalar(0), Scalar(0), Scalar(-1));

  std::vector<int> faces;
  int numFaces = 0;
  convexHull3dBuild(std::span<const Vector3>(vertices), faces, numFaces);

  // An octahedron has exactly 8 triangular faces
  ASSERT_EQ(numFaces, 8) << "Octahedron must have exactly 8 faces";
  ASSERT_EQ(faces.size(), 24u) << "8 faces * 3 vertices = 24 indices";

  // All 6 vertices should be used
  std::set<int> usedVertices;
  for (const auto& idx : faces) {
    ASSERT_GE(idx, 0) << "Vertex index must be non-negative";
    ASSERT_LT(idx, static_cast<int>(vertices.size()))
        << "Vertex index must be < number of vertices";
    usedVertices.insert(idx);
  }
  EXPECT_EQ(usedVertices.size(), 6u) << "All 6 vertices should be used";
}

//==============================================================================
// Edge Case Tests
//==============================================================================
TYPED_TEST(ConvexHullTest, CoplanarPoints)
{
  using Scalar = typename TestFixture::Scalar;
  using Vector3 = typename TestFixture::Vector3;

  // All points on the same plane (XY plane)
  std::vector<Vector3> vertices(5);
  vertices[0] = Vector3(Scalar(0), Scalar(0), Scalar(0));
  vertices[1] = Vector3(Scalar(1), Scalar(0), Scalar(0));
  vertices[2] = Vector3(Scalar(0), Scalar(1), Scalar(0));
  vertices[3] = Vector3(Scalar(1), Scalar(1), Scalar(0));
  vertices[4] = Vector3(Scalar(0.5), Scalar(0.5), Scalar(0));

  std::vector<int> faces;
  int numFaces = 0;

  convexHull3dBuild(std::span<const Vector3>(vertices), faces, numFaces);

  // Should handle degenerate case gracefully (may succeed or fail)
  // At minimum, should not crash or produce invalid results
  if (numFaces > 0) {
    EXPECT_TRUE(faces.size() == static_cast<size_t>(numFaces * 3));
  }
}

//==============================================================================
TYPED_TEST(ConvexHullTest, DuplicateVertices)
{
  using Scalar = typename TestFixture::Scalar;
  using Vector3 = typename TestFixture::Vector3;

  std::vector<Vector3> vertices(6);
  vertices[0] = Vector3(Scalar(0), Scalar(0), Scalar(0));
  vertices[1] = Vector3(Scalar(1), Scalar(0), Scalar(0));
  vertices[2] = Vector3(Scalar(0), Scalar(1), Scalar(0));
  vertices[3] = Vector3(Scalar(0), Scalar(0), Scalar(1));
  vertices[4]
      = Vector3(Scalar(0), Scalar(0), Scalar(0)); // Duplicate of vertex 0
  vertices[5]
      = Vector3(Scalar(1), Scalar(0), Scalar(0)); // Duplicate of vertex 1

  std::vector<int> faces;
  int numFaces = 0;

  convexHull3dBuild(std::span<const Vector3>(vertices), faces, numFaces);

  // Implementation adds noise to avoid degeneracy, so duplicates become
  // slightly different points. Should produce valid hull (4-6 faces)
  EXPECT_GE(numFaces, 4);
  EXPECT_LE(numFaces, 6);
  EXPECT_EQ(faces.size(), static_cast<size_t>(numFaces * 3));
}

//==============================================================================
// Property-Based Tests
//==============================================================================
TYPED_TEST(ConvexHullTest, AllFaceIndicesValid)
{
  using Scalar = typename TestFixture::Scalar;
  using Vector3 = typename TestFixture::Vector3;

  std::vector<Vector3> vertices(8);
  vertices[0] = Vector3(Scalar(0), Scalar(0), Scalar(0));
  vertices[1] = Vector3(Scalar(1), Scalar(0), Scalar(0));
  vertices[2] = Vector3(Scalar(0), Scalar(1), Scalar(0));
  vertices[3] = Vector3(Scalar(1), Scalar(1), Scalar(0));
  vertices[4] = Vector3(Scalar(0), Scalar(0), Scalar(1));
  vertices[5] = Vector3(Scalar(1), Scalar(0), Scalar(1));
  vertices[6] = Vector3(Scalar(0), Scalar(1), Scalar(1));
  vertices[7] = Vector3(Scalar(1), Scalar(1), Scalar(1));

  std::vector<int> faces;
  int numFaces = 0;

  convexHull3dBuild(std::span<const Vector3>(vertices), faces, numFaces);

  // All face indices must be valid
  for (const auto& idx : faces) {
    EXPECT_GE(idx, 0) << "Face index must be non-negative";
    EXPECT_LT(idx, static_cast<int>(vertices.size()))
        << "Face index must be < number of vertices";
  }
}

//==============================================================================
TYPED_TEST(ConvexHullTest, AllFacesAreTriangles)
{
  using Scalar = typename TestFixture::Scalar;
  using Vector3 = typename TestFixture::Vector3;

  std::vector<Vector3> vertices(8);
  vertices[0] = Vector3(Scalar(0), Scalar(0), Scalar(0));
  vertices[1] = Vector3(Scalar(1), Scalar(0), Scalar(0));
  vertices[2] = Vector3(Scalar(0), Scalar(1), Scalar(0));
  vertices[3] = Vector3(Scalar(1), Scalar(1), Scalar(0));
  vertices[4] = Vector3(Scalar(0), Scalar(0), Scalar(1));
  vertices[5] = Vector3(Scalar(1), Scalar(0), Scalar(1));
  vertices[6] = Vector3(Scalar(0), Scalar(1), Scalar(1));
  vertices[7] = Vector3(Scalar(1), Scalar(1), Scalar(1));

  std::vector<int> faces;
  int numFaces = 0;

  convexHull3dBuild(std::span<const Vector3>(vertices), faces, numFaces);

  // Number of indices must be exactly 3 * numFaces
  EXPECT_EQ(faces.size(), static_cast<size_t>(numFaces * 3))
      << "Each face must have exactly 3 vertices";
}

//==============================================================================
TYPED_TEST(ConvexHullTest, NoInternalVertices)
{
  using Scalar = typename TestFixture::Scalar;
  using Vector3 = typename TestFixture::Vector3;

  // Create a cube with a clearly internal point (far from boundaries)
  std::vector<Vector3> vertices(9);
  vertices[0] = Vector3(Scalar(0), Scalar(0), Scalar(0));
  vertices[1] = Vector3(Scalar(1), Scalar(0), Scalar(0));
  vertices[2] = Vector3(Scalar(0), Scalar(1), Scalar(0));
  vertices[3] = Vector3(Scalar(1), Scalar(1), Scalar(0));
  vertices[4] = Vector3(Scalar(0), Scalar(0), Scalar(1));
  vertices[5] = Vector3(Scalar(1), Scalar(0), Scalar(1));
  vertices[6] = Vector3(Scalar(0), Scalar(1), Scalar(1));
  vertices[7] = Vector3(Scalar(1), Scalar(1), Scalar(1));
  // Use a clearly internal point far from cube boundaries
  vertices[8] = Vector3(Scalar(0.25), Scalar(0.25), Scalar(0.25));

  std::vector<int> faces;
  int numFaces = 0;

  convexHull3dBuild(std::span<const Vector3>(vertices), faces, numFaces);

  // The internal point (vertex 8) should NOT appear in any face
  // (unless noise perturbation pushes it outside, which is less likely with
  // 0.25)
  std::unordered_set<int> usedVertices(faces.begin(), faces.end());

  // All 8 cube vertices must always be on the hull
  for (int i = 0; i < 8; ++i) {
    EXPECT_GT(usedVertices.count(i), 0u)
        << "Cube vertex " << i << " should be on the convex hull";
  }

  // For double precision, internal point should definitely not be on hull
  // For float precision with noise, it might rarely appear on hull due to
  // perturbation
  if (std::is_same<Scalar, double>::value) {
    EXPECT_EQ(usedVertices.count(8), 0u)
        << "Internal point should not be on the convex hull (double precision)";
  }
}

//==============================================================================
TYPED_TEST(ConvexHullTest, ConsistentWindingOrder)
{
  using Scalar = typename TestFixture::Scalar;
  using Vector3 = typename TestFixture::Vector3;

  std::vector<Vector3> vertices(4);
  vertices[0] = Vector3(Scalar(0), Scalar(0), Scalar(0));
  vertices[1] = Vector3(Scalar(1), Scalar(0), Scalar(0));
  vertices[2] = Vector3(Scalar(0), Scalar(1), Scalar(0));
  vertices[3] = Vector3(Scalar(0), Scalar(0), Scalar(1));

  std::vector<int> faces;
  int numFaces = 0;

  convexHull3dBuild(std::span<const Vector3>(vertices), faces, numFaces);

  // Compute centroid
  Vector3 centroid = Vector3::Zero();
  for (const auto& v : vertices) {
    centroid += v;
  }
  centroid /= static_cast<Scalar>(vertices.size());

  // For each face, verify that the normal points outward
  for (int i = 0; i < numFaces; ++i) {
    int i0 = faces[i * 3 + 0];
    int i1 = faces[i * 3 + 1];
    int i2 = faces[i * 3 + 2];

    Vector3 v0 = vertices[i0];
    Vector3 v1 = vertices[i1];
    Vector3 v2 = vertices[i2];

    // Compute face normal (cross product manually)
    Vector3 edge1 = v1 - v0;
    Vector3 edge2 = v2 - v0;
    Vector3 normal;
    normal[0] = edge1[1] * edge2[2] - edge1[2] * edge2[1];
    normal[1] = edge1[2] * edge2[0] - edge1[0] * edge2[2];
    normal[2] = edge1[0] * edge2[1] - edge1[1] * edge2[0];

    // Vector from centroid to face
    Vector3 toFace = v0 - centroid;

    // Normal should point outward (same direction as toFace)
    Scalar dot = normal.dot(toFace);
    EXPECT_GT(dot, Scalar(-1e-10))
        << "Face " << i << " normal should point outward from centroid";
  }
}
