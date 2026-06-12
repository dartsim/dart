/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary form, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/simulation/detail/deformable_contact/tangent_stencil.hpp>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include <cmath>

namespace dc = dart::simulation::detail::deformable_contact;

namespace {

//==============================================================================
dc::Vector12d stack12(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d)
{
  dc::Vector12d x;
  x << a, b, c, d;
  return x;
}

//==============================================================================
dc::Vector9d stack9(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
  dc::Vector9d x;
  x << a, b, c;
  return x;
}

//==============================================================================
dc::Vector6d stack6(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  dc::Vector6d x;
  x << a, b;
  return x;
}

//==============================================================================
template <typename Derived>
void expectFinite(const Eigen::MatrixBase<Derived>& value)
{
  for (int row = 0; row < value.rows(); ++row) {
    for (int col = 0; col < value.cols(); ++col) {
      EXPECT_TRUE(std::isfinite(value(row, col)))
          << "row=" << row << " col=" << col;
    }
  }
}

//==============================================================================
template <typename DerivedA, typename DerivedB>
void expectMatrixNear(
    const Eigen::MatrixBase<DerivedA>& actual,
    const Eigen::MatrixBase<DerivedB>& expected,
    const double tolerance)
{
  ASSERT_EQ(actual.rows(), expected.rows());
  ASSERT_EQ(actual.cols(), expected.cols());
  for (int row = 0; row < actual.rows(); ++row) {
    for (int col = 0; col < actual.cols(); ++col) {
      EXPECT_NEAR(actual(row, col), expected(row, col), tolerance)
          << "row=" << row << " col=" << col;
    }
  }
}

//==============================================================================
void expectBasisOrthonormal(const dc::Matrix3x2d& basis)
{
  expectFinite(basis);
  EXPECT_NEAR(basis.col(0).norm(), 1.0, 1e-14);
  EXPECT_NEAR(basis.col(1).norm(), 1.0, 1e-14);
  EXPECT_NEAR(basis.col(0).dot(basis.col(1)), 0.0, 1e-14);
}

//==============================================================================
template <typename Projection, typename Metric, typename Displacement>
void expectMetricMatchesProjection(
    const Projection& projection,
    const Metric& metric,
    const Displacement& displacement)
{
  expectMatrixNear(metric, projection.transpose() * projection, 1e-14);
  expectMatrixNear(
      dc::liftTangentialDisplacement(projection, projection * displacement),
      metric * displacement,
      1e-14);
  EXPECT_NEAR(
      displacement.dot(metric * displacement),
      (projection * displacement).squaredNorm(),
      1e-12);

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(metric);
  ASSERT_EQ(eigenSolver.info(), Eigen::Success);
  EXPECT_GE(eigenSolver.eigenvalues().minCoeff(), -1e-12);
}

//==============================================================================
template <typename Projection, typename Displacement>
void expectUniformTranslationNullspace(
    const Projection& projection, const Displacement& displacement)
{
  expectMatrixNear(projection * displacement, Eigen::Vector2d::Zero(), 1e-14);
}

} // namespace

//==============================================================================
TEST(IpcTangentStencil, PointTriangleMatchesUpstreamInteriorStencil)
{
  const Eigen::Vector3d p(0.5, 1.0, 0.25);
  const Eigen::Vector3d a(0.0, 0.0, 0.0);
  const Eigen::Vector3d b(2.0, 0.0, 0.0);
  const Eigen::Vector3d c(0.0, 4.0, 0.0);
  const auto stencil = dc::pointTriangleTangentStencil(p, a, b, c);

  EXPECT_FALSE(stencil.usedFallbackBasis);
  expectBasisOrthonormal(stencil.basis);
  EXPECT_NEAR(stencil.coordinates.x(), 0.25, 1e-14);
  EXPECT_NEAR(stencil.coordinates.y(), 0.25, 1e-14);
  expectMatrixNear(
      stencil.basis,
      (dc::Matrix3x2d() << 1.0, 0.0, 0.0, 1.0, 0.0, 0.0).finished(),
      1e-14);

  const dc::Vector12d dx = stack12(
      Eigen::Vector3d(1.0, 2.0, 3.0),
      Eigen::Vector3d(-1.0, 0.5, 0.0),
      Eigen::Vector3d(0.5, -0.25, 1.0),
      Eigen::Vector3d(-0.75, 1.5, -0.5));
  const Eigen::Vector3d relative
      = dx.segment<3>(0)
        - (dx.segment<3>(3)
           + stencil.coordinates.x() * (dx.segment<3>(6) - dx.segment<3>(3))
           + stencil.coordinates.y() * (dx.segment<3>(9) - dx.segment<3>(3)));
  expectMatrixNear(
      stencil.projection * dx, stencil.basis.transpose() * relative, 1e-14);
  expectMetricMatchesProjection(stencil.projection, stencil.metric, dx);
  expectUniformTranslationNullspace(
      stencil.projection,
      stack12(
          Eigen::Vector3d(1.0, -2.0, 0.5),
          Eigen::Vector3d(1.0, -2.0, 0.5),
          Eigen::Vector3d(1.0, -2.0, 0.5),
          Eigen::Vector3d(1.0, -2.0, 0.5)));
  expectMatrixNear(
      stencil.projection
          * stack12(
              Eigen::Vector3d::UnitZ(),
              Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Zero()),
      Eigen::Vector2d::Zero(),
      1e-14);

  const Eigen::Vector2d tangent(0.25, -0.5);
  const dc::Vector12d lifted
      = dc::liftTangentialDisplacement(stencil.projection, tangent);
  const Eigen::Vector3d tangent3 = stencil.basis * tangent;
  const dc::Vector12d expected
      = stack12(tangent3, -0.5 * tangent3, -0.25 * tangent3, -0.25 * tangent3);
  expectMatrixNear(lifted, expected, 1e-14);
}

//==============================================================================
TEST(IpcTangentStencil, EdgeEdgeMatchesUpstreamInteriorStencil)
{
  const Eigen::Vector3d a(0.0, 0.0, 0.0);
  const Eigen::Vector3d b(2.0, 0.0, 0.0);
  const Eigen::Vector3d c(0.5, -1.0, 1.0);
  const Eigen::Vector3d d(0.5, 3.0, 1.0);
  const auto stencil = dc::edgeEdgeTangentStencil(a, b, c, d);

  EXPECT_FALSE(stencil.usedFallbackBasis);
  expectBasisOrthonormal(stencil.basis);
  EXPECT_NEAR(stencil.coordinates.x(), 0.25, 1e-14);
  EXPECT_NEAR(stencil.coordinates.y(), 0.25, 1e-14);
  expectMatrixNear(
      stencil.basis,
      (dc::Matrix3x2d() << 1.0, 0.0, 0.0, 1.0, 0.0, 0.0).finished(),
      1e-14);

  const dc::Vector12d dx = stack12(
      Eigen::Vector3d(1.0, 2.0, 3.0),
      Eigen::Vector3d(-1.0, 0.5, 0.0),
      Eigen::Vector3d(0.5, -0.25, 1.0),
      Eigen::Vector3d(-0.75, 1.5, -0.5));
  const Eigen::Vector3d relative
      = dx.segment<3>(0)
        + stencil.coordinates.x() * (dx.segment<3>(3) - dx.segment<3>(0))
        - (dx.segment<3>(6)
           + stencil.coordinates.y() * (dx.segment<3>(9) - dx.segment<3>(6)));
  expectMatrixNear(
      stencil.projection * dx, stencil.basis.transpose() * relative, 1e-14);
  expectMetricMatchesProjection(stencil.projection, stencil.metric, dx);
  expectUniformTranslationNullspace(
      stencil.projection,
      stack12(
          Eigen::Vector3d(1.0, -2.0, 0.5),
          Eigen::Vector3d(1.0, -2.0, 0.5),
          Eigen::Vector3d(1.0, -2.0, 0.5),
          Eigen::Vector3d(1.0, -2.0, 0.5)));
  expectMatrixNear(
      stencil.projection
          * stack12(
              Eigen::Vector3d::UnitZ(),
              Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Zero()),
      Eigen::Vector2d::Zero(),
      1e-14);

  const Eigen::Vector2d tangent(-0.5, 0.75);
  const dc::Vector12d lifted
      = dc::liftTangentialDisplacement(stencil.projection, tangent);
  const Eigen::Vector3d tangent3 = stencil.basis * tangent;
  const dc::Vector12d expected = stack12(
      0.75 * tangent3, 0.25 * tangent3, -0.75 * tangent3, -0.25 * tangent3);
  expectMatrixNear(lifted, expected, 1e-14);
}

//==============================================================================
TEST(IpcTangentStencil, PointEdgeMatchesUpstreamStencil)
{
  const Eigen::Vector3d p(0.5, 1.0, 0.0);
  const Eigen::Vector3d a(0.0, 0.0, 0.0);
  const Eigen::Vector3d b(2.0, 0.0, 0.0);
  const auto stencil = dc::pointEdgeTangentStencil(p, a, b);

  EXPECT_FALSE(stencil.usedFallbackBasis);
  expectBasisOrthonormal(stencil.basis);
  EXPECT_NEAR(stencil.coordinate, 0.25, 1e-14);
  expectMatrixNear(
      stencil.basis,
      (dc::Matrix3x2d() << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0).finished(),
      1e-14);

  const dc::Vector9d dx = stack9(
      Eigen::Vector3d(1.0, 2.0, 3.0),
      Eigen::Vector3d(-1.0, 0.5, 0.0),
      Eigen::Vector3d(0.5, -0.25, 1.0));
  const Eigen::Vector3d relative
      = dx.segment<3>(0)
        - (dx.segment<3>(3)
           + stencil.coordinate * (dx.segment<3>(6) - dx.segment<3>(3)));
  expectMatrixNear(
      stencil.projection * dx, stencil.basis.transpose() * relative, 1e-14);
  expectMetricMatchesProjection(stencil.projection, stencil.metric, dx);
  expectUniformTranslationNullspace(
      stencil.projection,
      stack9(
          Eigen::Vector3d(1.0, -2.0, 0.5),
          Eigen::Vector3d(1.0, -2.0, 0.5),
          Eigen::Vector3d(1.0, -2.0, 0.5)));
  expectMatrixNear(
      stencil.projection
          * stack9(
              Eigen::Vector3d::UnitY(),
              Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Zero()),
      Eigen::Vector2d::Zero(),
      1e-14);
}

//==============================================================================
TEST(IpcTangentStencil, PointPointMatchesUpstreamStencil)
{
  const Eigen::Vector3d a(0.0, 0.0, 0.0);
  const Eigen::Vector3d b(0.0, 0.0, 2.0);
  const auto stencil = dc::pointPointTangentStencil(a, b);

  EXPECT_FALSE(stencil.usedFallbackBasis);
  expectBasisOrthonormal(stencil.basis);
  expectMatrixNear(
      stencil.basis,
      (dc::Matrix3x2d() << 1.0, 0.0, 0.0, 1.0, 0.0, 0.0).finished(),
      1e-14);

  const dc::Vector6d dx
      = stack6(Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d(-1.0, 0.5, 0.0));
  const Eigen::Vector3d relative = dx.segment<3>(0) - dx.segment<3>(3);
  expectMatrixNear(
      stencil.projection * dx, stencil.basis.transpose() * relative, 1e-14);
  expectMetricMatchesProjection(stencil.projection, stencil.metric, dx);
  expectUniformTranslationNullspace(
      stencil.projection,
      stack6(Eigen::Vector3d(1.0, -2.0, 0.5), Eigen::Vector3d(1.0, -2.0, 0.5)));
  expectMatrixNear(
      stencil.projection
          * stack6(Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero()),
      Eigen::Vector2d::Zero(),
      1e-14);
}

//==============================================================================
TEST(IpcTangentStencil, UnconstrainedCoordinatesMatchNonAxisAlignedCases)
{
  const Eigen::Vector3d a(1.0, -2.0, 0.5);
  const Eigen::Vector3d ab(2.0, 1.0, 0.25);
  const Eigen::Vector3d ac(-0.5, 1.5, 0.75);
  const Eigen::Vector3d normal = ab.cross(ac).normalized();
  const Eigen::Vector3d p = a + 0.5 * ab + 0.5 * ac + 0.2 * normal;
  const auto pointTriangle
      = dc::pointTriangleTangentStencil(p, a, a + ab, a + ac);
  EXPECT_NEAR(pointTriangle.coordinates.x(), 0.5, 1e-14);
  EXPECT_NEAR(pointTriangle.coordinates.y(), 0.5, 1e-14);

  const Eigen::Vector3d edgeA0(-0.25, 0.75, 1.0);
  const Eigen::Vector3d edgeA(2.0, 1.0, -0.5);
  const Eigen::Vector3d edgeB(-0.5, 1.0, 0.75);
  const Eigen::Vector3d edgeNormal = edgeA.cross(edgeB).normalized();
  const Eigen::Vector3d edgeB0
      = edgeA0 + 0.5 * edgeA + 0.3 * edgeNormal - 0.5 * edgeB;
  const auto edgeEdge = dc::edgeEdgeTangentStencil(
      edgeA0, edgeA0 + edgeA, edgeB0, edgeB0 + edgeB);
  EXPECT_NEAR(edgeEdge.coordinates.x(), 0.5, 1e-14);
  EXPECT_NEAR(edgeEdge.coordinates.y(), 0.5, 1e-14);

  const Eigen::Vector3d pointEdgeA(0.25, -0.5, 0.75);
  const Eigen::Vector3d pointEdgeDelta(1.5, -0.5, 0.25);
  const Eigen::Vector3d pointEdgeNormal = pointEdgeDelta.unitOrthogonal();
  const auto pointEdge = dc::pointEdgeTangentStencil(
      pointEdgeA + 0.5 * pointEdgeDelta + 0.25 * pointEdgeNormal,
      pointEdgeA,
      pointEdgeA + pointEdgeDelta);
  EXPECT_NEAR(pointEdge.coordinate, 0.5, 1e-14);
}

//==============================================================================
TEST(IpcTangentStencil, DegenerateInputsUseFiniteFallbacks)
{
  const Eigen::Vector3d p(0.0, 0.0, 0.0);
  const auto pointTriangle = dc::pointTriangleTangentStencil(
      p, p, p, Eigen::Vector3d(1.0, 0.0, 0.0));
  const auto edgeEdge = dc::edgeEdgeTangentStencil(p, p, p, p);
  const auto pointEdge = dc::pointEdgeTangentStencil(p, p, p);
  const auto pointPoint = dc::pointPointTangentStencil(p, p);

  EXPECT_TRUE(pointTriangle.usedFallbackBasis);
  EXPECT_TRUE(edgeEdge.usedFallbackBasis);
  EXPECT_TRUE(pointEdge.usedFallbackBasis);
  EXPECT_TRUE(pointPoint.usedFallbackBasis);
  expectBasisOrthonormal(pointTriangle.basis);
  expectBasisOrthonormal(edgeEdge.basis);
  expectBasisOrthonormal(pointEdge.basis);
  expectBasisOrthonormal(pointPoint.basis);
  expectFinite(pointTriangle.projection);
  expectFinite(edgeEdge.projection);
  expectFinite(pointEdge.projection);
  expectFinite(pointPoint.projection);
  expectFinite(pointTriangle.metric);
  expectFinite(edgeEdge.metric);
  expectFinite(pointEdge.metric);
  expectFinite(pointPoint.metric);
}
