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

#include <dart/simulation/detail/deformable_contact/primitive_distance.hpp>

#include <gtest/gtest.h>

#include <array>

#include <cmath>

namespace dc = dart::simulation::detail::deformable_contact;

namespace {

//==============================================================================
void expectVectorNear(
    const Eigen::Vector3d& actual,
    const Eigen::Vector3d& expected,
    const double tolerance = 1e-12)
{
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

//==============================================================================
dc::Vector12d stack(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d)
{
  dc::Vector12d x;
  x.segment<3>(0) = a;
  x.segment<3>(3) = b;
  x.segment<3>(6) = c;
  x.segment<3>(9) = d;
  return x;
}

//==============================================================================
Eigen::Vector3d segment(const dc::Vector12d& x, int block)
{
  return x.segment<3>(3 * block);
}

//==============================================================================
dc::Vector12d finiteGradient(
    const dc::Vector12d& x, const auto& valueFunction, const double step = 1e-6)
{
  dc::Vector12d gradient;
  for (int axis = 0; axis < 12; ++axis) {
    dc::Vector12d forward = x;
    dc::Vector12d backward = x;
    forward[axis] += step;
    backward[axis] -= step;
    gradient[axis]
        = (valueFunction(forward) - valueFunction(backward)) / (2.0 * step);
  }
  return gradient;
}

//==============================================================================
dc::Matrix12d finiteHessian(
    const dc::Vector12d& x, const auto& valueFunction, const double step = 1e-5)
{
  dc::Matrix12d hessian;
  const double f0 = valueFunction(x);
  for (int row = 0; row < 12; ++row) {
    for (int col = row; col < 12; ++col) {
      double value = 0.0;
      if (row == col) {
        dc::Vector12d forward = x;
        dc::Vector12d backward = x;
        forward[row] += step;
        backward[row] -= step;
        value = (valueFunction(forward) - 2.0 * f0 + valueFunction(backward))
                / (step * step);
      } else {
        dc::Vector12d pp = x;
        dc::Vector12d pm = x;
        dc::Vector12d mp = x;
        dc::Vector12d mm = x;
        pp[row] += step;
        pp[col] += step;
        pm[row] += step;
        pm[col] -= step;
        mp[row] -= step;
        mp[col] += step;
        mm[row] -= step;
        mm[col] -= step;
        value = (valueFunction(pp) - valueFunction(pm) - valueFunction(mp)
                 + valueFunction(mm))
                / (4.0 * step * step);
      }
      hessian(row, col) = value;
      hessian(col, row) = value;
    }
  }
  return hessian;
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
      ASSERT_NEAR(actual(row, col), expected(row, col), tolerance)
          << "row=" << row << " col=" << col;
    }
  }
}

//==============================================================================
template <typename Derived>
void expectFiniteSymmetric(
    const Eigen::MatrixBase<Derived>& matrix, const double tolerance = 1e-12)
{
  for (int row = 0; row < matrix.rows(); ++row) {
    for (int col = 0; col < matrix.cols(); ++col) {
      ASSERT_TRUE(std::isfinite(matrix(row, col)))
          << "row=" << row << " col=" << col;
      ASSERT_NEAR(matrix(row, col), matrix(col, row), tolerance)
          << "row=" << row << " col=" << col;
    }
  }
}

//==============================================================================
template <typename Derived>
void expectTranslationNullspace(
    const Eigen::MatrixBase<Derived>& hessian,
    const int blockCount,
    const double tolerance = 1e-10)
{
  for (int axis = 0; axis < 3; ++axis) {
    Eigen::VectorXd translation = Eigen::VectorXd::Zero(3 * blockCount);
    for (int block = 0; block < blockCount; ++block) {
      translation[3 * block + axis] = 1.0;
    }
    EXPECT_NEAR((hessian * translation).norm(), 0.0, tolerance);
    EXPECT_NEAR((translation.transpose() * hessian).norm(), 0.0, tolerance);
  }
}

//==============================================================================
void expectPointEdgeDerivativesMatch(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const dc::PointEdgeFeature expectedFeature)
{
  const auto distance = dc::pointEdgeSquaredDistance(p, a, b);
  EXPECT_EQ(distance.feature, expectedFeature);

  const dc::Vector12d x = stack(p, a, b, Eigen::Vector3d::Zero());
  const auto value = [](const dc::Vector12d& q) {
    return dc::pointEdgeSquaredDistance(
               segment(q, 0), segment(q, 1), segment(q, 2))
        .squaredDistance;
  };

  const dc::Vector9d analyticGradient
      = dc::pointEdgeSquaredDistanceGradient(p, a, b);
  const dc::Vector9d numericGradient = finiteGradient(x, value).head<9>();
  ASSERT_TRUE(analyticGradient.isApprox(numericGradient, 1e-6))
      << "analytic=" << analyticGradient.transpose()
      << "\nnumeric=" << numericGradient.transpose();

  const dc::Matrix9d analyticHessian
      = dc::pointEdgeSquaredDistanceHessian(p, a, b);
  const dc::Matrix9d numericHessian
      = finiteHessian(x, value).topLeftCorner<9, 9>();
  expectFiniteSymmetric(analyticHessian);
  expectTranslationNullspace(analyticHessian, 3);
  expectMatrixNear(analyticHessian, numericHessian, 2e-4);
}

//==============================================================================
void expectPointTriangleDerivativesMatch(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const dc::PointTriangleFeature expectedFeature)
{
  const auto distance = dc::pointTriangleSquaredDistance(p, a, b, c);
  EXPECT_EQ(distance.feature, expectedFeature);

  const dc::Vector12d x = stack(p, a, b, c);
  const auto value = [](const dc::Vector12d& q) {
    return dc::pointTriangleSquaredDistance(
               segment(q, 0), segment(q, 1), segment(q, 2), segment(q, 3))
        .squaredDistance;
  };

  const dc::Vector12d analyticGradient
      = dc::pointTriangleSquaredDistanceGradient(p, a, b, c);
  const dc::Vector12d numericGradient = finiteGradient(x, value);
  ASSERT_TRUE(analyticGradient.isApprox(numericGradient, 1e-6))
      << "analytic=" << analyticGradient.transpose()
      << "\nnumeric=" << numericGradient.transpose();

  const dc::Matrix12d analyticHessian
      = dc::pointTriangleSquaredDistanceHessian(p, a, b, c);
  const dc::Matrix12d numericHessian = finiteHessian(x, value);
  expectFiniteSymmetric(analyticHessian);
  expectTranslationNullspace(analyticHessian, 4);
  expectMatrixNear(analyticHessian, numericHessian, 2e-4);
}

//==============================================================================
void expectEdgeEdgeDerivativesMatch(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d,
    const dc::EdgeEdgeFeature expectedFeature)
{
  const auto distance = dc::edgeEdgeSquaredDistance(a, b, c, d);
  EXPECT_EQ(distance.feature, expectedFeature);

  const dc::Vector12d x = stack(a, b, c, d);
  const auto value = [](const dc::Vector12d& q) {
    return dc::edgeEdgeSquaredDistance(
               segment(q, 0), segment(q, 1), segment(q, 2), segment(q, 3))
        .squaredDistance;
  };

  const dc::Vector12d analyticGradient
      = dc::edgeEdgeSquaredDistanceGradient(a, b, c, d);
  const dc::Vector12d numericGradient = finiteGradient(x, value);
  ASSERT_TRUE(analyticGradient.isApprox(numericGradient, 1e-6))
      << "analytic=" << analyticGradient.transpose()
      << "\nnumeric=" << numericGradient.transpose();

  const dc::Matrix12d analyticHessian
      = dc::edgeEdgeSquaredDistanceHessian(a, b, c, d);
  const dc::Matrix12d numericHessian = finiteHessian(x, value);
  expectFiniteSymmetric(analyticHessian);
  expectTranslationNullspace(analyticHessian, 4);
  expectMatrixNear(analyticHessian, numericHessian, 2e-4);
}

} // namespace

//==============================================================================
TEST(IpcPrimitiveDistance, ComputesPointTriangleClosestFeatures)
{
  const Eigen::Vector3d a(0.0, 0.0, 0.0);
  const Eigen::Vector3d b(1.0, 0.0, 0.0);
  const Eigen::Vector3d c(0.0, 1.0, 0.0);

  const auto face = dc::pointTriangleSquaredDistance(
      Eigen::Vector3d(0.25, 0.25, 2.0), a, b, c);
  EXPECT_EQ(face.feature, dc::PointTriangleFeature::Face);
  EXPECT_NEAR(face.squaredDistance, 4.0, 1e-12);
  expectVectorNear(face.closestPoint, Eigen::Vector3d(0.25, 0.25, 0.0));
  expectVectorNear(face.barycentric, Eigen::Vector3d(0.5, 0.25, 0.25));

  const auto edge = dc::pointTriangleSquaredDistance(
      Eigen::Vector3d(0.50, -2.0, 0.0), a, b, c);
  EXPECT_EQ(edge.feature, dc::PointTriangleFeature::EdgeAB);
  EXPECT_NEAR(edge.squaredDistance, 4.0, 1e-12);
  expectVectorNear(edge.closestPoint, Eigen::Vector3d(0.5, 0.0, 0.0));
  expectVectorNear(edge.barycentric, Eigen::Vector3d(0.5, 0.5, 0.0));

  const auto vertex = dc::pointTriangleSquaredDistance(
      Eigen::Vector3d(-1.0, -1.0, 2.0), a, b, c);
  EXPECT_EQ(vertex.feature, dc::PointTriangleFeature::VertexA);
  EXPECT_NEAR(vertex.squaredDistance, 6.0, 1e-12);
  expectVectorNear(vertex.closestPoint, a);
  expectVectorNear(vertex.barycentric, Eigen::Vector3d(1.0, 0.0, 0.0));
}

//==============================================================================
TEST(IpcPrimitiveDistance, HandlesDegeneratePointTriangleAsEdges)
{
  const Eigen::Vector3d a(0.0, 0.0, 0.0);
  const Eigen::Vector3d b(1.0, 0.0, 0.0);
  const Eigen::Vector3d c(2.0, 0.0, 0.0);
  const auto distance = dc::pointTriangleSquaredDistance(
      Eigen::Vector3d(0.75, 2.0, 0.0), a, b, c);

  EXPECT_EQ(distance.feature, dc::PointTriangleFeature::Degenerate);
  EXPECT_NEAR(distance.squaredDistance, 4.0, 1e-12);
  expectVectorNear(distance.closestPoint, Eigen::Vector3d(0.75, 0.0, 0.0));
  EXPECT_NEAR(distance.barycentric.sum(), 1.0, 1e-12);
}

//==============================================================================
TEST(IpcPrimitiveDistance, KeepsSmallValidGeometryNondegenerate)
{
  const Eigen::Vector3d a(0.0, 0.0, 0.0);
  const Eigen::Vector3d b(1e-4, 0.0, 0.0);
  const Eigen::Vector3d c(0.0, 1e-4, 0.0);
  const Eigen::Vector3d p(2.5e-5, 2.5e-5, 1e-4);

  const auto triangle = dc::pointTriangleSquaredDistance(p, a, b, c);
  EXPECT_EQ(triangle.feature, dc::PointTriangleFeature::Face);
  EXPECT_NEAR(triangle.squaredDistance, 1e-8, 1e-20);
  expectVectorNear(
      triangle.barycentric, Eigen::Vector3d(0.5, 0.25, 0.25), 1e-12);

  const auto edge
      = dc::pointEdgeSquaredDistance(Eigen::Vector3d(5e-5, 2e-4, 0.0), a, b);
  EXPECT_EQ(edge.feature, dc::PointEdgeFeature::PointEdgeInterior);
  EXPECT_NEAR(edge.squaredDistance, 4e-8, 1e-20);

  const auto edgeEdge = dc::edgeEdgeSquaredDistance(
      Eigen::Vector3d(-5e-5, 0.0, 0.0),
      Eigen::Vector3d(5e-5, 0.0, 0.0),
      Eigen::Vector3d(0.0, -5e-5, 1e-4),
      Eigen::Vector3d(0.0, 5e-5, 1e-4));
  EXPECT_EQ(edgeEdge.feature, dc::EdgeEdgeFeature::EdgeAInteriorEdgeBInterior);
  EXPECT_NEAR(edgeEdge.squaredDistance, 1e-8, 1e-20);
}

//==============================================================================
TEST(IpcPrimitiveDistance, PointPointDerivativesAreAnalytic)
{
  const Eigen::Vector3d a(0.2, -0.4, 1.3);
  const Eigen::Vector3d b(-0.7, 0.5, 0.1);

  const auto gradient = dc::pointPointSquaredDistanceGradient(a, b);
  EXPECT_TRUE(gradient.isApprox(
      (dc::Vector6d() << 1.8, -1.8, 2.4, -1.8, 1.8, -2.4).finished()));
  expectMatrixNear(
      dc::pointPointSquaredDistanceHessian(),
      finiteHessian(
          (dc::Vector12d() << a,
           b,
           Eigen::Vector3d::Zero(),
           Eigen::Vector3d::Zero())
              .finished(),
          [](const dc::Vector12d& q) {
            return dc::pointPointSquaredDistance(segment(q, 0), segment(q, 1));
          })
          .topLeftCorner<6, 6>(),
      2e-4);
}

//==============================================================================
TEST(IpcPrimitiveDistance, PointEdgeFeatureDerivativesMatchFiniteDifferences)
{
  expectPointEdgeDerivativesMatch(
      Eigen::Vector3d(-1.0, 0.5, 0.25),
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      dc::PointEdgeFeature::PointEdgeStart);
  expectPointEdgeDerivativesMatch(
      Eigen::Vector3d(2.0, 0.5, 0.25),
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      dc::PointEdgeFeature::PointEdgeEnd);
  expectPointEdgeDerivativesMatch(
      Eigen::Vector3d(0.4, 0.5, 0.25),
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      dc::PointEdgeFeature::PointEdgeInterior);
}

//==============================================================================
TEST(IpcPrimitiveDistance, PointTriangleDerivativesMatchFiniteDifferences)
{
  const Eigen::Vector3d a(0.0, 0.0, 0.0);
  const Eigen::Vector3d b(1.0, 0.0, 0.0);
  const Eigen::Vector3d c(0.0, 1.0, 0.0);

  expectPointTriangleDerivativesMatch(
      Eigen::Vector3d(-1.0, -1.0, 1.5),
      a,
      b,
      c,
      dc::PointTriangleFeature::VertexA);
  expectPointTriangleDerivativesMatch(
      Eigen::Vector3d(2.0, -0.5, 1.5),
      a,
      b,
      c,
      dc::PointTriangleFeature::VertexB);
  expectPointTriangleDerivativesMatch(
      Eigen::Vector3d(-0.5, 2.0, 1.5),
      a,
      b,
      c,
      dc::PointTriangleFeature::VertexC);
  expectPointTriangleDerivativesMatch(
      Eigen::Vector3d(0.5, -1.0, 1.5),
      a,
      b,
      c,
      dc::PointTriangleFeature::EdgeAB);
  expectPointTriangleDerivativesMatch(
      Eigen::Vector3d(0.8, 0.8, 1.5),
      a,
      b,
      c,
      dc::PointTriangleFeature::EdgeBC);
  expectPointTriangleDerivativesMatch(
      Eigen::Vector3d(-1.0, 0.5, 1.5),
      a,
      b,
      c,
      dc::PointTriangleFeature::EdgeCA);
  expectPointTriangleDerivativesMatch(
      Eigen::Vector3d(0.25, 0.25, 1.5),
      a,
      b,
      c,
      dc::PointTriangleFeature::Face);
}

//==============================================================================
TEST(IpcPrimitiveDistance, ComputesEdgeEdgeClosestFeatures)
{
  const auto interior = dc::edgeEdgeSquaredDistance(
      Eigen::Vector3d(-1.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, -1.0, 2.0),
      Eigen::Vector3d(0.0, 1.0, 2.0));
  EXPECT_EQ(interior.feature, dc::EdgeEdgeFeature::EdgeAInteriorEdgeBInterior);
  EXPECT_NEAR(interior.squaredDistance, 4.0, 1e-12);
  EXPECT_NEAR(interior.edgeACoordinate, 0.5, 1e-12);
  EXPECT_NEAR(interior.edgeBCoordinate, 0.5, 1e-12);
  expectVectorNear(interior.closestPointOnA, Eigen::Vector3d::Zero());
  expectVectorNear(interior.closestPointOnB, Eigen::Vector3d(0.0, 0.0, 2.0));

  const auto vertexEdge = dc::edgeEdgeSquaredDistance(
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(-1.0, 1.0, 0.0),
      Eigen::Vector3d(-1.0, -1.0, 0.0));
  EXPECT_EQ(vertexEdge.feature, dc::EdgeEdgeFeature::EdgeAStartEdgeBInterior);
  EXPECT_NEAR(vertexEdge.squaredDistance, 1.0, 1e-12);
}

//==============================================================================
TEST(IpcPrimitiveDistance, EdgeEdgeDerivativesMatchFiniteDifferences)
{
  expectEdgeEdgeDerivativesMatch(
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(-1.0, 1.0, 0.4),
      Eigen::Vector3d(-1.0, 2.0, 0.4),
      dc::EdgeEdgeFeature::EdgeAStartEdgeBStart);
  expectEdgeEdgeDerivativesMatch(
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(-1.0, -2.0, 0.4),
      Eigen::Vector3d(-1.0, -1.0, 0.4),
      dc::EdgeEdgeFeature::EdgeAStartEdgeBEnd);
  expectEdgeEdgeDerivativesMatch(
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(2.0, 1.0, 0.4),
      Eigen::Vector3d(2.0, 2.0, 0.4),
      dc::EdgeEdgeFeature::EdgeAEndEdgeBStart);
  expectEdgeEdgeDerivativesMatch(
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(2.0, -2.0, 0.4),
      Eigen::Vector3d(2.0, -1.0, 0.4),
      dc::EdgeEdgeFeature::EdgeAEndEdgeBEnd);
  expectEdgeEdgeDerivativesMatch(
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.5, 1.0, 0.4),
      Eigen::Vector3d(0.5, 2.0, 0.4),
      dc::EdgeEdgeFeature::EdgeAInteriorEdgeBStart);
  expectEdgeEdgeDerivativesMatch(
      Eigen::Vector3d(-1.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, -3.0, 0.4),
      Eigen::Vector3d(0.0, -2.0, 0.4),
      dc::EdgeEdgeFeature::EdgeAInteriorEdgeBEnd);
  expectEdgeEdgeDerivativesMatch(
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(-1.0, -1.0, 0.4),
      Eigen::Vector3d(-1.0, 1.0, 0.4),
      dc::EdgeEdgeFeature::EdgeAStartEdgeBInterior);
  expectEdgeEdgeDerivativesMatch(
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(2.0, -1.0, 0.4),
      Eigen::Vector3d(2.0, 1.0, 0.4),
      dc::EdgeEdgeFeature::EdgeAEndEdgeBInterior);
  expectEdgeEdgeDerivativesMatch(
      Eigen::Vector3d(-0.8, -0.1, 0.0),
      Eigen::Vector3d(1.1, 0.2, 0.3),
      Eigen::Vector3d(0.2, -1.2, 1.4),
      Eigen::Vector3d(-0.1, 1.3, 1.8),
      dc::EdgeEdgeFeature::EdgeAInteriorEdgeBInterior);
}

//==============================================================================
TEST(IpcPrimitiveDistance, HandlesParallelAndNearParallelEdgeEdgeDistances)
{
  const auto overlapping = dc::edgeEdgeSquaredDistance(
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.25, 0.0, 0.0),
      Eigen::Vector3d(0.75, 0.0, 0.0));
  EXPECT_NEAR(overlapping.squaredDistance, 0.0, 1e-15);

  const auto disjoint = dc::edgeEdgeSquaredDistance(
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(2.0, 0.1, 0.0),
      Eigen::Vector3d(3.0, 0.1, 0.0));
  EXPECT_NEAR(disjoint.squaredDistance, 1.01, 1e-12);
  EXPECT_EQ(disjoint.feature, dc::EdgeEdgeFeature::EdgeAEndEdgeBStart);

  const auto nearParallelSkew = dc::edgeEdgeSquaredDistance(
      Eigen::Vector3d(-1.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(-1.0, 1e-8, 1e-4),
      Eigen::Vector3d(1.0, 2e-8, 1e-4));
  EXPECT_TRUE(std::isfinite(nearParallelSkew.squaredDistance));
  EXPECT_GE(nearParallelSkew.squaredDistance, 0.0);
  EXPECT_LT(std::abs(nearParallelSkew.squaredDistance - 1e-8), 1e-12);
  expectFiniteSymmetric(
      dc::edgeEdgeSquaredDistanceHessian(
          Eigen::Vector3d(-1.0, 0.0, 0.0),
          Eigen::Vector3d(1.0, 0.0, 0.0),
          Eigen::Vector3d(-1.0, 1e-8, 1e-4),
          Eigen::Vector3d(1.0, 2e-8, 1e-4)));

  const auto smallParallel = dc::edgeEdgeSquaredDistance(
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1e-4, 0.0, 0.0),
      Eigen::Vector3d(2e-5, 1e-8, 0.0),
      Eigen::Vector3d(8e-5, 1e-8, 0.0));
  EXPECT_NEAR(smallParallel.squaredDistance, 1e-16, 1e-28);
  EXPECT_EQ(
      smallParallel.feature, dc::EdgeEdgeFeature::EdgeAInteriorEdgeBStart);
  expectFiniteSymmetric(
      dc::edgeEdgeSquaredDistanceHessian(
          Eigen::Vector3d(0.0, 0.0, 0.0),
          Eigen::Vector3d(1e-4, 0.0, 0.0),
          Eigen::Vector3d(2e-5, 1e-8, 0.0),
          Eigen::Vector3d(8e-5, 1e-8, 0.0)));
}

//==============================================================================
TEST(IpcPrimitiveDistance, DistancesAreTranslationInvariant)
{
  const Eigen::Vector3d offset(3.0, -2.0, 0.75);
  const Eigen::Vector3d p(0.23, 0.31, 1.7);
  const Eigen::Vector3d a(0.0, 0.0, 0.0);
  const Eigen::Vector3d b(1.2, 0.1, 0.0);
  const Eigen::Vector3d c(-0.1, 1.1, 0.2);

  const auto base = dc::pointTriangleSquaredDistance(p, a, b, c);
  const auto translated = dc::pointTriangleSquaredDistance(
      p + offset, a + offset, b + offset, c + offset);
  EXPECT_NEAR(base.squaredDistance, translated.squaredDistance, 1e-12);

  const auto gradient = dc::pointTriangleSquaredDistanceGradient(p, a, b, c);
  EXPECT_NEAR(
      (gradient.segment<3>(0) + gradient.segment<3>(3) + gradient.segment<3>(6)
       + gradient.segment<3>(9))
          .norm(),
      0.0,
      1e-12);
}

//==============================================================================
TEST(IpcPrimitiveDistance, EdgeEdgeMollifierMatchesReferenceShape)
{
  const Eigen::Vector3d restA(0.0, 0.0, 0.0);
  const Eigen::Vector3d restB(2.0, 0.0, 0.0);
  const Eigen::Vector3d restC(0.0, 0.0, 0.0);
  const Eigen::Vector3d restD(0.0, 2.0, 0.0);
  const double threshold
      = dc::edgeEdgeMollifierThreshold(restA, restB, restC, restD);
  EXPECT_NEAR(threshold, 0.016, 1e-15);

  const double ratio = 0.25;
  const double x = ratio * threshold;
  EXPECT_NEAR(dc::edgeEdgeMollifier(x, threshold), 0.4375, 1e-15);
  EXPECT_NEAR(
      dc::edgeEdgeMollifierGradient(x, threshold),
      2.0 * (1.0 - ratio) / threshold,
      1e-12);
  EXPECT_NEAR(
      dc::edgeEdgeMollifierHessian(x, threshold),
      -2.0 / (threshold * threshold),
      1e-12);
  EXPECT_DOUBLE_EQ(dc::edgeEdgeMollifier(2.0 * threshold, threshold), 1.0);
  EXPECT_DOUBLE_EQ(
      dc::edgeEdgeMollifierGradient(2.0 * threshold, threshold), 0.0);
}

//==============================================================================
TEST(IpcPrimitiveDistance, EdgeEdgeCrossNormDerivativesMatchFiniteDifferences)
{
  const Eigen::Vector3d a(-0.9, 0.2, 0.1);
  const Eigen::Vector3d b(1.3, -0.3, 0.4);
  const Eigen::Vector3d c(0.1, -1.0, 0.7);
  const Eigen::Vector3d d(-0.2, 1.4, 1.1);
  const dc::Vector12d x = stack(a, b, c, d);

  const auto value = [](const dc::Vector12d& q) {
    return dc::edgeEdgeCrossSquaredNorm(
        segment(q, 0), segment(q, 1), segment(q, 2), segment(q, 3));
  };

  const dc::Vector12d analyticGradient
      = dc::edgeEdgeCrossSquaredNormGradient(a, b, c, d);
  const dc::Vector12d numericGradient = finiteGradient(x, value);
  ASSERT_TRUE(analyticGradient.isApprox(numericGradient, 1e-6))
      << "analytic=" << analyticGradient.transpose()
      << "\nnumeric=" << numericGradient.transpose();

  const dc::Matrix12d analyticHessian
      = dc::edgeEdgeCrossSquaredNormHessian(a, b, c, d);
  const dc::Matrix12d numericHessian = finiteHessian(x, value);
  expectMatrixNear(analyticHessian, numericHessian, 2e-4);
}

//==============================================================================
TEST(IpcPrimitiveDistance, EdgeEdgeMollifierDerivativesMatchFiniteDifferences)
{
  const Eigen::Vector3d a(-1.0, 0.0, 0.0);
  const Eigen::Vector3d b(1.0, 0.0, 0.0);
  const Eigen::Vector3d c(-1.0, 1e-4, 0.0);
  const Eigen::Vector3d d(1.0, -1e-4, 0.0);
  const double threshold = 0.016;
  ASSERT_LT(dc::edgeEdgeCrossSquaredNorm(a, b, c, d), threshold);
  const dc::Vector12d x = stack(a, b, c, d);

  const auto value = [threshold](const dc::Vector12d& q) {
    return dc::edgeEdgeMollifier(
        segment(q, 0), segment(q, 1), segment(q, 2), segment(q, 3), threshold);
  };

  const dc::Vector12d analyticGradient
      = dc::edgeEdgeMollifierGradient(a, b, c, d, threshold);
  const dc::Vector12d numericGradient = finiteGradient(x, value, 1e-7);
  ASSERT_TRUE(analyticGradient.isApprox(numericGradient, 1e-7))
      << "analytic=" << analyticGradient.transpose()
      << "\nnumeric=" << numericGradient.transpose();

  const dc::Matrix12d analyticHessian
      = dc::edgeEdgeMollifierHessian(a, b, c, d, threshold);
  const dc::Matrix12d numericHessian = finiteHessian(x, value, 1e-5);
  expectMatrixNear(analyticHessian, numericHessian, 2e-3);
}
