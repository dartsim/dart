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

#include <dart/simulation/experimental/detail/deformable_contact/barrier_kernel.hpp>
#include <dart/simulation/experimental/detail/deformable_contact/primitive_distance.hpp>

#include <gtest/gtest.h>

#include <functional>

#include <cmath>

namespace dc = dart::simulation::experimental::detail::deformable_contact;

namespace {

using ValueFunction = std::function<double(const dc::Vector12d&)>;

//==============================================================================
dc::Vector12d stack(
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
dc::Vector12d finiteGradient(
    const dc::Vector12d& x,
    const ValueFunction& value,
    const double step = 1e-6)
{
  dc::Vector12d gradient = dc::Vector12d::Zero();
  for (int i = 0; i < x.size(); ++i) {
    dc::Vector12d plus = x;
    dc::Vector12d minus = x;
    plus[i] += step;
    minus[i] -= step;
    gradient[i] = (value(plus) - value(minus)) / (2.0 * step);
  }
  return gradient;
}

//==============================================================================
dc::Matrix12d finiteHessian(
    const dc::Vector12d& x,
    const ValueFunction& value,
    const double step = 1e-4)
{
  dc::Matrix12d hessian = dc::Matrix12d::Zero();
  for (int row = 0; row < x.size(); ++row) {
    for (int col = 0; col < x.size(); ++col) {
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
      hessian(row, col) = (value(pp) - value(pm) - value(mp) + value(mm))
                          / (4.0 * step * step);
    }
  }
  return hessian;
}

//==============================================================================
template <typename DerivedA, typename DerivedB>
void expectVectorNear(
    const Eigen::MatrixBase<DerivedA>& actual,
    const Eigen::MatrixBase<DerivedB>& expected,
    const double tolerance)
{
  ASSERT_EQ(actual.size(), expected.size());
  for (int i = 0; i < actual.size(); ++i) {
    EXPECT_NEAR(actual[i], expected[i], tolerance) << "index=" << i;
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
      EXPECT_NEAR(matrix(row, col), matrix(col, row), tolerance)
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
dc::PrimitiveBarrierResult pointTriangleBarrierFromVector(
    const dc::Vector12d& x, const double squaredActivationDistance)
{
  return dc::pointTriangleBarrier(
      x.segment<3>(0),
      x.segment<3>(3),
      x.segment<3>(6),
      x.segment<3>(9),
      squaredActivationDistance);
}

//==============================================================================
dc::PrimitiveBarrierResult edgeEdgeBarrierFromVector(
    const dc::Vector12d& x, const double squaredActivationDistance)
{
  return dc::edgeEdgeBarrier(
      x.segment<3>(0),
      x.segment<3>(3),
      x.segment<3>(6),
      x.segment<3>(9),
      squaredActivationDistance);
}

//==============================================================================
dc::PrimitiveBarrierResult mollifiedEdgeEdgeBarrierFromVector(
    const dc::Vector12d& x,
    const double squaredActivationDistance,
    const double mollifierThreshold)
{
  return dc::mollifiedEdgeEdgeBarrier(
      x.segment<3>(0),
      x.segment<3>(3),
      x.segment<3>(6),
      x.segment<3>(9),
      squaredActivationDistance,
      mollifierThreshold);
}

} // namespace

//==============================================================================
TEST(IpcBarrierKernel, ScalarMatchesUpstreamC2ClampedLogFormula)
{
  const auto interior = dc::c2ClampedLogBarrier(0.25, 1.0);
  EXPECT_TRUE(interior.active);
  EXPECT_DOUBLE_EQ(interior.safeSquaredDistance, 0.25);
  EXPECT_NEAR(interior.value, 0.7797905781299385, 1e-15);
  EXPECT_NEAR(interior.firstDerivative, -4.329441541679836, 1e-15);
  EXPECT_NEAR(interior.secondDerivative, 23.77258872223978, 1e-14);

  const auto nearThreshold = dc::c2ClampedLogBarrier(0.99, 1.0);
  EXPECT_TRUE(nearThreshold.active);
  EXPECT_NEAR(nearThreshold.value, 1.0050335853501468e-6, 1e-18);
  EXPECT_NEAR(nearThreshold.firstDerivative, -3.020168180801304e-4, 1e-16);
  EXPECT_NEAR(nearThreshold.secondDerivative, 0.060606742516104053, 1e-14);

  const auto threshold = dc::c2ClampedLogBarrier(1.0, 1.0);
  EXPECT_FALSE(threshold.active);
  EXPECT_EQ(threshold.value, 0.0);
  EXPECT_EQ(threshold.firstDerivative, 0.0);
  EXPECT_EQ(threshold.secondDerivative, 0.0);

  const auto outside = dc::c2ClampedLogBarrier(1.25, 1.0);
  EXPECT_FALSE(outside.active);
  EXPECT_EQ(outside.value, 0.0);
  EXPECT_EQ(outside.firstDerivative, 0.0);
  EXPECT_EQ(outside.secondDerivative, 0.0);
}

//==============================================================================
TEST(IpcBarrierKernel, ScalarGuardsNonPositiveDistanceAsActiveRepulsion)
{
  const auto result = dc::c2ClampedLogBarrier(0.0, 1.0);
  EXPECT_TRUE(result.active);
  EXPECT_EQ(result.squaredDistance, 0.0);
  EXPECT_GT(result.safeSquaredDistance, 0.0);
  EXPECT_TRUE(std::isfinite(result.value));
  EXPECT_TRUE(std::isfinite(result.firstDerivative));
  EXPECT_TRUE(std::isfinite(result.secondDerivative));
  EXPECT_GT(result.value, 0.0);
  EXPECT_LT(result.firstDerivative, 0.0);
  EXPECT_GT(result.secondDerivative, 0.0);
}

//==============================================================================
TEST(IpcBarrierKernel, PointTriangleDerivativesMatchFiniteDifferences)
{
  const dc::Vector12d x = stack(
      Eigen::Vector3d(0.2, 0.3, 0.5),
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0));
  constexpr double kSquaredActivationDistance = 1.0;

  const auto result
      = pointTriangleBarrierFromVector(x, kSquaredActivationDistance);
  ASSERT_TRUE(result.active);
  expectFiniteSymmetric(result.hessian);

  const auto value = [](const dc::Vector12d& y) {
    return pointTriangleBarrierFromVector(y, kSquaredActivationDistance).value;
  };

  expectVectorNear(result.gradient, finiteGradient(x, value), 2e-6);
  expectMatrixNear(result.hessian, finiteHessian(x, value), 3e-3);
}

//==============================================================================
TEST(IpcBarrierKernel, EdgeEdgeDerivativesMatchFiniteDifferences)
{
  const dc::Vector12d x = stack(
      Eigen::Vector3d(-1.0, 0.0, 0.5),
      Eigen::Vector3d(1.0, 0.0, 0.5),
      Eigen::Vector3d(0.0, -1.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0));
  constexpr double kSquaredActivationDistance = 1.0;

  const auto result = edgeEdgeBarrierFromVector(x, kSquaredActivationDistance);
  ASSERT_TRUE(result.active);
  expectFiniteSymmetric(result.hessian);

  const auto value = [](const dc::Vector12d& y) {
    return edgeEdgeBarrierFromVector(y, kSquaredActivationDistance).value;
  };

  expectVectorNear(result.gradient, finiteGradient(x, value), 2e-6);
  expectMatrixNear(result.hessian, finiteHessian(x, value), 4e-3);
}

//==============================================================================
TEST(IpcBarrierKernel, MollifiedEdgeEdgeMatchesFiniteDifferences)
{
  const dc::Vector12d x = stack(
      Eigen::Vector3d(-1.0, 0.0, 0.5),
      Eigen::Vector3d(1.0, 0.0, 0.5),
      Eigen::Vector3d(0.0, -1.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0));
  constexpr double kSquaredActivationDistance = 1.0;
  constexpr double kMollifierThreshold = 64.0;

  const auto result = mollifiedEdgeEdgeBarrierFromVector(
      x, kSquaredActivationDistance, kMollifierThreshold);
  ASSERT_TRUE(result.active);
  EXPECT_GT(result.mollifier, 0.0);
  EXPECT_LT(result.mollifier, 1.0);
  expectFiniteSymmetric(result.hessian);

  const auto value = [](const dc::Vector12d& y) {
    return mollifiedEdgeEdgeBarrierFromVector(
               y, kSquaredActivationDistance, kMollifierThreshold)
        .value;
  };

  expectVectorNear(result.gradient, finiteGradient(x, value), 3e-6);
  expectMatrixNear(result.hessian, finiteHessian(x, value), 5e-3);
}

//==============================================================================
TEST(IpcBarrierKernel, InactivePrimitiveBarriersReturnZeroDerivatives)
{
  constexpr double kSquaredActivationDistance = 1.0;

  const auto pointTriangle = dc::pointTriangleBarrier(
      Eigen::Vector3d(0.2, 0.3, 2.0),
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0),
      kSquaredActivationDistance);
  EXPECT_FALSE(pointTriangle.active);
  EXPECT_EQ(pointTriangle.value, 0.0);
  EXPECT_EQ(pointTriangle.gradient.norm(), 0.0);
  EXPECT_EQ(pointTriangle.hessian.norm(), 0.0);

  const auto edgeEdge = dc::edgeEdgeBarrier(
      Eigen::Vector3d(-1.0, 0.0, 2.0),
      Eigen::Vector3d(1.0, 0.0, 2.0),
      Eigen::Vector3d(0.0, -1.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0),
      kSquaredActivationDistance);
  EXPECT_FALSE(edgeEdge.active);
  EXPECT_EQ(edgeEdge.value, 0.0);
  EXPECT_EQ(edgeEdge.gradient.norm(), 0.0);
  EXPECT_EQ(edgeEdge.hessian.norm(), 0.0);
}

//==============================================================================
TEST(IpcBarrierKernel, NegativeGradientIncreasesPointTriangleDistance)
{
  dc::Vector12d x = stack(
      Eigen::Vector3d(0.2, 0.3, 0.5),
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0));
  constexpr double kSquaredActivationDistance = 1.0;

  const auto result
      = pointTriangleBarrierFromVector(x, kSquaredActivationDistance);
  ASSERT_TRUE(result.active);
  ASSERT_GT(result.gradient.norm(), 0.0);

  const double before
      = dc::pointTriangleSquaredDistance(
            x.segment<3>(0), x.segment<3>(3), x.segment<3>(6), x.segment<3>(9))
            .squaredDistance;
  x -= 1e-4 * result.gradient.normalized();
  const double after
      = dc::pointTriangleSquaredDistance(
            x.segment<3>(0), x.segment<3>(3), x.segment<3>(6), x.segment<3>(9))
            .squaredDistance;
  EXPECT_GT(after, before);
}
