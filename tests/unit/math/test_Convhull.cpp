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

#include "dart/math/detail/convhull.hpp"

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <array>
#include <set>
#include <type_traits>
#include <unordered_set>
#include <vector>

using dart::math::detail::convexHull3dBuild;

//==============================================================================
TEST(ConvhullInternal, SortFloatAndInt)
{
  std::array<float, 3> values{{3.0f, 1.0f, 2.0f}};
  float* outValues = nullptr;
  std::array<int, 3> indices{{0, 0, 0}};
  dart::math::detail::convhull_internal::sortFloat(
      values.data(),
      outValues,
      indices.data(),
      static_cast<int>(values.size()),
      true);

  EXPECT_FLOAT_EQ(values[0], 3.0f);
  EXPECT_FLOAT_EQ(values[1], 2.0f);
  EXPECT_FLOAT_EQ(values[2], 1.0f);

  std::array<int, 3> ints{{3, 1, 2}};
  dart::math::detail::convhull_internal::sortInt(
      ints.data(), static_cast<int>(ints.size()));

  EXPECT_EQ(ints[0], 1);
  EXPECT_EQ(ints[1], 2);
  EXPECT_EQ(ints[2], 3);
}

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
TYPED_TEST(ConvexHullTest, EmptyInput)
{
  using Vector3 = typename TestFixture::Vector3;

  std::vector<Vector3> vertices;
  std::vector<int> faces;
  int numFaces = 0;

  convexHull3dBuild(vertices, faces, numFaces);

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
  convexHull3dBuild(vertices, faces, numFaces);

  ASSERT_EQ(numFaces, 4);
  ASSERT_EQ(faces.size(), 12u);

  std::set<int> usedVertices;
  for (const auto& index : faces) {
    ASSERT_GE(index, 0);
    ASSERT_LT(index, static_cast<int>(vertices.size()));
    usedVertices.insert(index);
  }
  EXPECT_EQ(usedVertices.size(), 4u);
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
  convexHull3dBuild(vertices, faces, numFaces);

  ASSERT_EQ(numFaces, 12);
  ASSERT_EQ(faces.size(), 36u);

  std::set<int> usedVertices;
  for (const auto& index : faces) {
    ASSERT_GE(index, 0);
    ASSERT_LT(index, static_cast<int>(vertices.size()));
    usedVertices.insert(index);
  }
  EXPECT_EQ(usedVertices.size(), 8u);
}

//==============================================================================
TYPED_TEST(ConvexHullTest, NoInternalVertices)
{
  using Scalar = typename TestFixture::Scalar;
  using Vector3 = typename TestFixture::Vector3;

  std::vector<Vector3> vertices(9);
  vertices[0] = Vector3(Scalar(0), Scalar(0), Scalar(0));
  vertices[1] = Vector3(Scalar(1), Scalar(0), Scalar(0));
  vertices[2] = Vector3(Scalar(0), Scalar(1), Scalar(0));
  vertices[3] = Vector3(Scalar(1), Scalar(1), Scalar(0));
  vertices[4] = Vector3(Scalar(0), Scalar(0), Scalar(1));
  vertices[5] = Vector3(Scalar(1), Scalar(0), Scalar(1));
  vertices[6] = Vector3(Scalar(0), Scalar(1), Scalar(1));
  vertices[7] = Vector3(Scalar(1), Scalar(1), Scalar(1));
  vertices[8] = Vector3(Scalar(0.25), Scalar(0.25), Scalar(0.25));

  std::vector<int> faces;
  int numFaces = 0;
  convexHull3dBuild(vertices, faces, numFaces);

  std::unordered_set<int> usedVertices(faces.begin(), faces.end());

  for (int i = 0; i < 8; ++i) {
    EXPECT_TRUE(usedVertices.count(i) > 0)
        << "Cube vertex " << i << " should be on the convex hull";
  }

  if (std::is_same<Scalar, double>::value) {
    EXPECT_TRUE(usedVertices.count(8) == 0)
        << "Internal point should not be on the convex hull";
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
  convexHull3dBuild(vertices, faces, numFaces);

  Vector3 centroid = Vector3::Zero();
  for (const auto& vertex : vertices) {
    centroid += vertex;
  }
  centroid /= static_cast<Scalar>(vertices.size());

  for (int i = 0; i < numFaces; ++i) {
    const int i0 = faces[i * 3 + 0];
    const int i1 = faces[i * 3 + 1];
    const int i2 = faces[i * 3 + 2];

    const Vector3 edge1 = vertices[i1] - vertices[i0];
    const Vector3 edge2 = vertices[i2] - vertices[i0];
    const Vector3 normal = edge1.cross(edge2);
    const Vector3 toFace = vertices[i0] - centroid;

    EXPECT_GT(normal.dot(toFace), Scalar(-1e-10))
        << "Face " << i << " normal should point outward";
  }
}
