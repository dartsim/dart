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

#include <dart/math/detail/MasonryArchGeometry.hpp>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include <array>
#include <limits>

namespace {

using dart::math::detail::MasonryArchBarrierGapPolicy;
using dart::math::detail::MasonryArchStoneWedgeGeometry;

constexpr double kPinnedObjVertexTolerance = 2e-8;

void expectVerticesNear(
    const MasonryArchStoneWedgeGeometry& actual,
    const std::array<Eigen::Vector3d, 8>& expected)
{
  for (std::size_t i = 0u; i < expected.size(); ++i) {
    EXPECT_TRUE(
        actual.vertices[i].isApprox(expected[i], kPinnedObjVertexTolerance))
        << "vertex " << i << " actual=" << actual.vertices[i].transpose()
        << " expected=" << expected[i].transpose();
  }
}

void expectFiniteConvexWedge(const MasonryArchStoneWedgeGeometry& stone)
{
  EXPECT_TRUE(stone.centroid.allFinite());
  EXPECT_TRUE(stone.min.allFinite());
  EXPECT_TRUE(stone.max.allFinite());
  EXPECT_TRUE(std::isfinite(stone.volume));
  EXPECT_GT(stone.volume, 0.0);
  EXPECT_TRUE(stone.momentPerUnitMass.allFinite());
  EXPECT_TRUE(stone.momentPerUnitMass.isApprox(
      stone.momentPerUnitMass.transpose(), 1e-14));
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(
      stone.momentPerUnitMass);
  ASSERT_EQ(eigenSolver.info(), Eigen::Success);
  EXPECT_GT(eigenSolver.eigenvalues().minCoeff(), 0.0);
  EXPECT_TRUE((stone.max - stone.min).array().isFinite().all());
  EXPECT_TRUE(((stone.max - stone.min).array() > 0.0).all());

  for (const auto& vertex : stone.vertices) {
    EXPECT_TRUE(vertex.allFinite());
    EXPECT_TRUE(((vertex - stone.min).array() >= -1e-12).all());
    EXPECT_TRUE(((stone.max - vertex).array() >= -1e-12).all());
  }

  for (const auto& triangle :
       dart::math::detail::getMasonryArchStoneWedgeTriangles()) {
    const auto& a = stone.vertices[triangle[0]];
    const auto& b = stone.vertices[triangle[1]];
    const auto& c = stone.vertices[triangle[2]];
    const Eigen::Vector3d normal = (b - a).cross(c - a);
    ASSERT_GT(normal.norm(), 1e-12);
    EXPECT_LT(normal.dot(stone.centroid - a), 1e-12);
    for (const auto& vertex : stone.vertices)
      EXPECT_LE(normal.dot(vertex - a), 1e-12);
  }
}

Eigen::Matrix3d computeTetrahedralMomentPerUnitMass(
    const MasonryArchStoneWedgeGeometry& stone)
{
  double volume = 0.0;
  Eigen::Vector3d firstMoment = Eigen::Vector3d::Zero();
  Eigen::Matrix3d secondMoment = Eigen::Matrix3d::Zero();

  for (const auto& triangle :
       dart::math::detail::getMasonryArchStoneWedgeTriangles()) {
    const Eigen::Vector3d a = stone.vertices[triangle[0]] - stone.centroid;
    const Eigen::Vector3d b = stone.vertices[triangle[1]] - stone.centroid;
    const Eigen::Vector3d c = stone.vertices[triangle[2]] - stone.centroid;
    const double tetraVolume = std::abs(a.dot(b.cross(c))) / 6.0;
    const Eigen::Vector3d sum = a + b + c;
    volume += tetraVolume;
    firstMoment += 0.25 * tetraVolume * sum;
    secondMoment += tetraVolume
                    * (sum * sum.transpose() + a * a.transpose()
                       + b * b.transpose() + c * c.transpose())
                    / 20.0;
  }

  EXPECT_NEAR(volume, stone.volume, 1e-13);
  EXPECT_LE(firstMoment.norm(), 1e-13) << firstMoment.transpose();
  const Eigen::Matrix3d covariance = secondMoment / volume;
  return covariance.trace() * Eigen::Matrix3d::Identity() - covariance;
}

} // namespace

//==============================================================================
TEST(MasonryArchGeometry, SourceOffsetWedgesMatchPinnedRigidIpcObjSamples)
{
  // Values below are the six-decimal OBJ vertices at Rigid-IPC commit
  // 23b6ba6fbf8434056444ae106356fd2209136988, converted from source
  // (x, height, depth) centimeters to DART (x, depth, height) meters.
  const auto arch25 = dart::math::detail::generateMasonryArchStoneWedges(25u);
  ASSERT_EQ(arch25.size(), 25u);

  const std::array<Eigen::Vector3d, 8> expected25Springer = {{
      {-0.26315009, -0.05, 0.001},
      {-0.26315009, 0.05, 0.001},
      {-0.25097166, -0.05, 0.05506144},
      {-0.25097166, 0.05, 0.05506144},
      {-0.36565382, -0.05, 0.001},
      {-0.36565382, 0.05, 0.001},
      {-0.34837115, -0.05, 0.07771845},
      {-0.34837115, 0.05, 0.07771845},
  }};
  expectVerticesNear(arch25.front(), expected25Springer);
  EXPECT_TRUE(arch25.front().centroid.isApprox(
      Eigen::Vector3d(-0.309922711831, 0.0, 0.0340220619104), 2e-8));
  const Eigen::Vector3d springerExtents
      = arch25.front().max - arch25.front().min;
  EXPECT_NEAR(springerExtents.x(), 0.11468216, 1e-8);
  EXPECT_NEAR(springerExtents.y(), 0.1, 1e-8);
  EXPECT_NEAR(springerExtents.z(), 0.07671845, 1e-8);
  EXPECT_NEAR(arch25.front().volume, 0.000670270538989, 2e-10);
  EXPECT_TRUE(arch25.front().momentPerUnitMass.isApprox(
      computeTetrahedralMomentPerUnitMass(arch25.front()), 1e-12));

  const std::array<Eigen::Vector3d, 8> expected25Crown = {{
      {-0.01694454, -0.03510962, 0.58582120},
      {-0.01694454, 0.03510962, 0.58582120},
      {0.01694454, -0.03510962, 0.58582120},
      {0.01694454, 0.03510962, 0.58582120},
      {-0.03823896, -0.03510962, 0.65273375},
      {-0.03823896, 0.03510962, 0.65273375},
      {0.03823896, -0.03510962, 0.65273375},
      {0.03823896, 0.03510962, 0.65273375},
  }};
  expectVerticesNear(arch25[12], expected25Crown);
  EXPECT_TRUE(arch25[12].centroid.isApprox(
      Eigen::Vector3d(0.0, 0.0, 0.623580886778), 2e-8));
  EXPECT_NEAR(arch25[12].volume, 0.000259282346043, 2e-10);
  EXPECT_TRUE(arch25[12].momentPerUnitMass.isApprox(
      computeTetrahedralMomentPerUnitMass(arch25[12]), 1e-12));

  const auto arch101 = dart::math::detail::generateMasonryArchStoneWedges(101u);
  ASSERT_EQ(arch101.size(), 101u);
  const std::array<Eigen::Vector3d, 8> expected101Crown = {{
      {-0.00415074, -0.03500692, 0.62648966},
      {-0.00415074, 0.03500692, 0.62648966},
      {0.00415074, -0.03500692, 0.62648966},
      {0.00415074, 0.03500692, 0.62648966},
      {-0.00972056, -0.03500692, 0.69628159},
      {-0.00972056, 0.03500692, 0.69628159},
      {0.00972056, -0.03500692, 0.69628159},
      {0.00972056, 0.03500692, 0.69628159},
  }};
  expectVerticesNear(arch101[50], expected101Crown);
  EXPECT_TRUE(arch101[50].centroid.isApprox(
      Eigen::Vector3d(0.0, 0.0, 0.666056281770), 2e-8));
  EXPECT_NEAR(arch101[50].volume, 0.0000677807344730, 2e-11);
  const Eigen::Matrix3d expected101CrownMoment
      = computeTetrahedralMomentPerUnitMass(arch101[50]);
  // The source-offset crown is almost exactly symmetric, so the independent
  // tetrahedral sum loses a few low bits through cancellation. Compare the
  // resulting SI-valued moments with a tight absolute tolerance.
  EXPECT_LE(
      (arch101[50].momentPerUnitMass - expected101CrownMoment)
          .cwiseAbs()
          .maxCoeff(),
      5e-15)
      << "actual:\n"
      << arch101[50].momentPerUnitMass << "\nexpected:\n"
      << expected101CrownMoment << "\ndifference:\n"
      << arch101[50].momentPerUnitMass - expected101CrownMoment;
}

//==============================================================================
TEST(MasonryArchGeometry, ContactingWedgesAreFiniteConvexAndPreserveBoxFit)
{
  const auto wedges25 = dart::math::detail::generateMasonryArchStoneWedges(
      25u, {}, MasonryArchBarrierGapPolicy::OmitSourceOffsets);
  const auto boxes25 = dart::math::detail::generateMasonryArchStoneBoxes(25u);
  ASSERT_EQ(wedges25.size(), boxes25.size());

  for (std::size_t i = 0u; i < wedges25.size(); ++i) {
    const auto& wedge = wedges25[i];
    expectFiniteConvexWedge(wedge);
    EXPECT_TRUE(wedge.momentPerUnitMass.isApprox(
        computeTetrahedralMomentPerUnitMass(wedge), 1e-12));

    const Eigen::Vector3d vertexMean
        = (wedge.vertices[0] + wedge.vertices[1] + wedge.vertices[2]
           + wedge.vertices[3] + wedge.vertices[4] + wedge.vertices[5]
           + wedge.vertices[6] + wedge.vertices[7])
          / 8.0;
    EXPECT_TRUE(boxes25[i].transform.translation().isApprox(vertexMean, 1e-12));
    EXPECT_NEAR(boxes25[i].size.y(), wedge.max.y() - wedge.min.y(), 1e-12);
    EXPECT_NEAR(boxes25[i].size.z(), wedge.max.y() - wedge.min.y(), 1e-12);
  }

  const auto wedges101 = dart::math::detail::generateMasonryArchStoneWedges(
      101u, {}, MasonryArchBarrierGapPolicy::OmitSourceOffsets);
  ASSERT_EQ(wedges101.size(), 101u);
  for (const auto& wedge : wedges101)
    expectFiniteConvexWedge(wedge);

  const auto source25 = dart::math::detail::generateMasonryArchStoneWedges(25u);
  EXPECT_TRUE((source25[12].centroid - wedges25[12].centroid)
                  .isApprox(Eigen::Vector3d(0.0, 0.0, 0.012), 1e-12));
  EXPECT_TRUE((source25.front().centroid - wedges25.front().centroid)
                  .isApprox(Eigen::Vector3d(-0.012, 0.0, 0.0), 1e-12));
}

//==============================================================================
TEST(MasonryArchGeometry, ShallowEndFaceExpansionIsExplicitAndMassExact)
{
  constexpr double kExpansion = 1e-4;
  const auto nominal = dart::math::detail::generateMasonryArchStoneWedges(
      25u, {}, MasonryArchBarrierGapPolicy::OmitSourceOffsets);
  const auto expanded = dart::math::detail::generateMasonryArchStoneWedges(
      25u, {}, MasonryArchBarrierGapPolicy::OmitSourceOffsets, kExpansion);
  ASSERT_EQ(nominal.size(), expanded.size());

  for (std::size_t i = 0u; i < nominal.size(); ++i) {
    expectFiniteConvexWedge(expanded[i]);
    EXPECT_GT(expanded[i].volume, nominal[i].volume);
    EXPECT_TRUE(expanded[i].momentPerUnitMass.isApprox(
        computeTetrahedralMomentPerUnitMass(expanded[i]), 1e-12));

    const Eigen::Vector3d nominalFace0
        = 0.25
          * (nominal[i].vertices[0] + nominal[i].vertices[1]
             + nominal[i].vertices[4] + nominal[i].vertices[5]);
    const Eigen::Vector3d nominalFace1
        = 0.25
          * (nominal[i].vertices[2] + nominal[i].vertices[3]
             + nominal[i].vertices[6] + nominal[i].vertices[7]);
    const Eigen::Vector3d expandedFace0
        = 0.25
          * (expanded[i].vertices[0] + expanded[i].vertices[1]
             + expanded[i].vertices[4] + expanded[i].vertices[5]);
    const Eigen::Vector3d expandedFace1
        = 0.25
          * (expanded[i].vertices[2] + expanded[i].vertices[3]
             + expanded[i].vertices[6] + expanded[i].vertices[7]);
    EXPECT_NEAR((expandedFace0 - nominalFace0).norm(), 0.5 * kExpansion, 1e-12);
    EXPECT_NEAR((expandedFace1 - nominalFace1).norm(), 0.5 * kExpansion, 1e-12);
    EXPECT_NEAR(
        (expandedFace1 - expandedFace0).norm()
            - (nominalFace1 - nominalFace0).norm(),
        kExpansion,
        1e-12);
  }
}
