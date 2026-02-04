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

#include "../../helpers/gtest_utils.hpp"
#include "dart/math/icosphere.hpp"

#include <gtest/gtest.h>

using namespace dart;
using namespace math;

//==============================================================================
TEST(IcosphereTests, NumOfVerticesAndTriangles)
{
  const double radius = 5.0;

  for (auto i = 0; i < 8; ++i) {
    const auto subdivisions = i;
    const auto icosphere = Icosphered(radius, subdivisions);
    const auto& vertices = icosphere.getVertices();
    const auto& triangles = icosphere.getTriangles();

    EXPECT_EQ(vertices.size(), Icosphered::getNumVertices(subdivisions));
    EXPECT_EQ(triangles.size(), Icosphered::getNumTriangles(subdivisions));

    for (const auto& v : vertices) {
      EXPECT_DOUBLE_EQ(v.norm(), radius);
    }
  }
}

//==============================================================================
TEST(IcosphereTests, Constructor)
{
  auto s1 = Icosphered(1, 0);
  EXPECT_FALSE(s1.isEmpty());
  EXPECT_DOUBLE_EQ(s1.getRadius(), 1);
  EXPECT_EQ(s1.getNumSubdivisions(), 0);

  auto s2 = Icosphered(2, 3);
  EXPECT_FALSE(s2.isEmpty());
  EXPECT_DOUBLE_EQ(s2.getRadius(), 2);
  EXPECT_EQ(s2.getNumSubdivisions(), 3);
}

//==============================================================================
TEST(IcosphereTests, IcosahedronVerticesAndTriangles)
{
  const double radius = 2.5;
  const auto result = Icosphered::computeIcosahedron(radius);
  const auto& vertices = result.first;
  const auto& triangles = result.second;

  EXPECT_EQ(vertices.size(), 12u);
  EXPECT_EQ(triangles.size(), 20u);

  for (const auto& vertex : vertices) {
    EXPECT_NEAR(vertex.norm(), radius, 1e-12);
  }

  for (const auto& triangle : triangles) {
    EXPECT_LT(triangle[0], vertices.size());
    EXPECT_LT(triangle[1], vertices.size());
    EXPECT_LT(triangle[2], vertices.size());
  }
}

//==============================================================================
TEST(IcosphereTests, EdgeCountMatchesTriangleCount)
{
  for (std::size_t subdivisions = 0; subdivisions < 4; ++subdivisions) {
    const auto triangles = Icosphered::getNumTriangles(subdivisions);
    const auto edges = Icosphered::getNumEdges(subdivisions);
    EXPECT_EQ(edges, triangles / 2 * 3);
  }
}

//==============================================================================
TEST(IcosphereTests, FloatAndDoubleInstantiation)
{
  const float radiusF = 1.5f;
  const std::size_t subdivF = 1;
  const Icospheref sphereF(radiusF, subdivF);
  EXPECT_FALSE(sphereF.isEmpty());
  EXPECT_EQ(sphereF.getNumSubdivisions(), subdivF);
  EXPECT_EQ(sphereF.getVertices().size(), Icospheref::getNumVertices(subdivF));
  EXPECT_EQ(
      sphereF.getTriangles().size(), Icospheref::getNumTriangles(subdivF));

  const double radiusD = 2.25;
  const std::size_t subdivD = 2;
  const Icosphered sphereD(radiusD, subdivD);
  EXPECT_FALSE(sphereD.isEmpty());
  EXPECT_EQ(sphereD.getNumSubdivisions(), subdivD);
  EXPECT_EQ(sphereD.getVertices().size(), Icosphered::getNumVertices(subdivD));
  EXPECT_EQ(
      sphereD.getTriangles().size(), Icosphered::getNumTriangles(subdivD));

  const auto icosa = Icospheref::computeIcosahedron(radiusF);
  EXPECT_EQ(icosa.first.size(), 12u);
  EXPECT_EQ(icosa.second.size(), 20u);
}

//==============================================================================
TEST(IcosphereTests, ExplicitTemplateCoverage)
{
  using IcosphereDouble = dart::math::Icosphere<double>;
  using IcosphereFloat = dart::math::Icosphere<float>;

  EXPECT_EQ(IcosphereDouble::getNumVertices(0), 12u);
  EXPECT_EQ(IcosphereDouble::getNumVertices(1), 42u);
  EXPECT_EQ(IcosphereDouble::getNumEdges(0), 30u);
  EXPECT_EQ(IcosphereDouble::getNumTriangles(0), 20u);
  EXPECT_EQ(IcosphereDouble::getNumTriangles(1), 80u);

  const auto icosa = IcosphereDouble::computeIcosahedron(1.0);
  EXPECT_EQ(icosa.first.size(), 12u);
  EXPECT_EQ(icosa.second.size(), 20u);

  const IcosphereDouble ico0(1.0, 0);
  EXPECT_DOUBLE_EQ(ico0.getRadius(), 1.0);
  EXPECT_EQ(ico0.getNumSubdivisions(), 0u);

  const IcosphereDouble ico1(2.0, 1);
  EXPECT_DOUBLE_EQ(ico1.getRadius(), 2.0);
  EXPECT_EQ(ico1.getNumSubdivisions(), 1u);

  const IcosphereDouble ico2(1.0, 2);
  EXPECT_EQ(ico2.getTriangles().size(), IcosphereDouble::getNumTriangles(2));

  const IcosphereFloat icof(1.5f, 1);
  EXPECT_FLOAT_EQ(icof.getRadius(), 1.5f);
}

//==============================================================================
TEST(IcosphereTests, SubdivisionBuildUsesImplHeader)
{
  const double radius = 1.25;
  const std::size_t subdivisions = 2;

  const dart::math::Icosphere<double> icosphere(radius, subdivisions);

  EXPECT_EQ(
      icosphere.getVertices().size(),
      dart::math::Icosphere<double>::getNumVertices(subdivisions));
  EXPECT_EQ(
      icosphere.getTriangles().size(),
      dart::math::Icosphere<double>::getNumTriangles(subdivisions));

  for (const auto& vertex : icosphere.getVertices()) {
    EXPECT_NEAR(vertex.norm(), radius, 1e-12);
  }
}
