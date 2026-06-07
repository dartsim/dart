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

#include <dart/simulation/detail/deformable_contact/barrier_kernel.hpp>
#include <dart/simulation/detail/deformable_contact/primitive_distance.hpp>
#include <dart/simulation/detail/deformable_contact/tangent_stencil.hpp>
#include <dart/simulation/detail/newton_barrier/barrier_kernel.hpp>
#include <dart/simulation/detail/newton_barrier/friction_kernel.hpp>
#include <dart/simulation/detail/newton_barrier/primitive_distance.hpp>
#include <dart/simulation/detail/newton_barrier/psd_projection.hpp>
#include <dart/simulation/detail/newton_barrier/tangent_stencil.hpp>
#include <dart/simulation/detail/rigid_ipc_barrier.hpp>

#include <gtest/gtest.h>

#include <type_traits>

namespace dc = dart::simulation::detail::deformable_contact;
namespace nb = dart::simulation::detail::newton_barrier;
namespace rigid = dart::simulation::detail;

namespace {

//==============================================================================
template <typename DerivedA, typename DerivedB>
void expectMatrixNear(
    const Eigen::MatrixBase<DerivedA>& actual,
    const Eigen::MatrixBase<DerivedB>& expected,
    const double tolerance = 1e-14)
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
void expectBarrierNear(
    const nb::PrimitiveBarrierResult& actual,
    const nb::PrimitiveBarrierResult& expected)
{
  EXPECT_EQ(actual.active, expected.active);
  EXPECT_NEAR(actual.value, expected.value, 1e-14);
  EXPECT_NEAR(actual.squaredDistance, expected.squaredDistance, 1e-14);
  EXPECT_NEAR(actual.safeSquaredDistance, expected.safeSquaredDistance, 1e-14);
  EXPECT_NEAR(
      actual.squaredActivationDistance,
      expected.squaredActivationDistance,
      1e-14);
  EXPECT_NEAR(actual.mollifier, expected.mollifier, 1e-14);
  EXPECT_NEAR(actual.mollifierThreshold, expected.mollifierThreshold, 1e-14);
  expectMatrixNear(actual.gradient, expected.gradient);
  expectMatrixNear(actual.hessian, expected.hessian);
}

//==============================================================================
void expectScalarBarrierNear(
    const nb::BarrierScalarDerivatives& actual,
    const nb::BarrierScalarDerivatives& expected)
{
  EXPECT_EQ(actual.active, expected.active);
  EXPECT_NEAR(actual.value, expected.value, 1e-14);
  EXPECT_NEAR(actual.firstDerivative, expected.firstDerivative, 1e-14);
  EXPECT_NEAR(actual.secondDerivative, expected.secondDerivative, 1e-14);
  EXPECT_NEAR(actual.squaredDistance, expected.squaredDistance, 1e-14);
  EXPECT_NEAR(actual.safeSquaredDistance, expected.safeSquaredDistance, 1e-14);
  EXPECT_NEAR(
      actual.squaredActivationDistance,
      expected.squaredActivationDistance,
      1e-14);
}

} // namespace

//==============================================================================
TEST(NewtonBarrierPrimitives, DeformableContactHeadersForwardSharedTypes)
{
  static_assert(std::is_same_v<dc::Vector6d, nb::Vector6d>);
  static_assert(std::is_same_v<dc::Vector9d, nb::Vector9d>);
  static_assert(std::is_same_v<dc::Vector12d, nb::Vector12d>);
  static_assert(std::is_same_v<dc::Matrix6d, nb::Matrix6d>);
  static_assert(std::is_same_v<dc::Matrix9d, nb::Matrix9d>);
  static_assert(std::is_same_v<dc::Matrix12d, nb::Matrix12d>);
  static_assert(
      std::is_same_v<dc::PointTriangleDistance, nb::PointTriangleDistance>);
  static_assert(std::is_same_v<dc::EdgeEdgeDistance, nb::EdgeEdgeDistance>);
  static_assert(
      std::is_same_v<dc::PrimitiveBarrierResult, nb::PrimitiveBarrierResult>);
  static_assert(std::is_same_v<
                dc::PointTriangleTangentStencil,
                nb::PointTriangleTangentStencil>);

  EXPECT_DOUBLE_EQ(dc::detail::kRelativeEpsilon, nb::detail::kRelativeEpsilon);
  EXPECT_DOUBLE_EQ(
      dc::detail::kBarrierDistanceFloorScale,
      nb::detail::kBarrierDistanceFloorScale);
  EXPECT_DOUBLE_EQ(
      dc::detail::kTangentBasisEpsilon, nb::detail::kTangentBasisEpsilon);
}

//==============================================================================
TEST(NewtonBarrierPrimitives, DistanceForwardingMatchesSharedOwner)
{
  const Eigen::Vector3d p(0.25, 0.35, 0.4);
  const Eigen::Vector3d a(0.0, 0.0, 0.0);
  const Eigen::Vector3d b(1.25, 0.1, 0.0);
  const Eigen::Vector3d c(0.1, 1.5, 0.25);
  const Eigen::Vector3d d(0.25, 0.4, 1.2);

  const auto oldPointTriangle = dc::pointTriangleSquaredDistance(p, a, b, c);
  const auto newPointTriangle = nb::pointTriangleSquaredDistance(p, a, b, c);
  EXPECT_EQ(oldPointTriangle.feature, newPointTriangle.feature);
  EXPECT_NEAR(
      oldPointTriangle.squaredDistance,
      newPointTriangle.squaredDistance,
      1e-14);
  expectMatrixNear(oldPointTriangle.barycentric, newPointTriangle.barycentric);
  expectMatrixNear(
      oldPointTriangle.closestPoint, newPointTriangle.closestPoint);
  expectMatrixNear(
      dc::pointTriangleSquaredDistanceGradient(p, a, b, c),
      nb::pointTriangleSquaredDistanceGradient(p, a, b, c));
  expectMatrixNear(
      dc::pointTriangleSquaredDistanceHessian(p, a, b, c),
      nb::pointTriangleSquaredDistanceHessian(p, a, b, c));

  const auto oldPointEdge = dc::pointEdgeSquaredDistance(p, a, b);
  const auto newPointEdge = nb::pointEdgeSquaredDistance(p, a, b);
  EXPECT_EQ(oldPointEdge.feature, newPointEdge.feature);
  EXPECT_NEAR(
      oldPointEdge.squaredDistance, newPointEdge.squaredDistance, 1e-14);
  EXPECT_NEAR(oldPointEdge.edgeCoordinate, newPointEdge.edgeCoordinate, 1e-14);
  expectMatrixNear(oldPointEdge.closestPoint, newPointEdge.closestPoint);
  expectMatrixNear(
      dc::pointEdgeSquaredDistanceGradient(p, a, b),
      nb::pointEdgeSquaredDistanceGradient(p, a, b));
  expectMatrixNear(
      dc::pointEdgeSquaredDistanceHessian(p, a, b),
      nb::pointEdgeSquaredDistanceHessian(p, a, b));

  const auto oldEdgeEdge = dc::edgeEdgeSquaredDistance(a, b, c, d);
  const auto newEdgeEdge = nb::edgeEdgeSquaredDistance(a, b, c, d);
  EXPECT_EQ(oldEdgeEdge.feature, newEdgeEdge.feature);
  EXPECT_NEAR(oldEdgeEdge.squaredDistance, newEdgeEdge.squaredDistance, 1e-14);
  EXPECT_NEAR(oldEdgeEdge.edgeACoordinate, newEdgeEdge.edgeACoordinate, 1e-14);
  EXPECT_NEAR(oldEdgeEdge.edgeBCoordinate, newEdgeEdge.edgeBCoordinate, 1e-14);
  expectMatrixNear(oldEdgeEdge.closestPointOnA, newEdgeEdge.closestPointOnA);
  expectMatrixNear(oldEdgeEdge.closestPointOnB, newEdgeEdge.closestPointOnB);
  expectMatrixNear(
      dc::edgeEdgeSquaredDistanceGradient(a, b, c, d),
      nb::edgeEdgeSquaredDistanceGradient(a, b, c, d));
  expectMatrixNear(
      dc::edgeEdgeSquaredDistanceHessian(a, b, c, d),
      nb::edgeEdgeSquaredDistanceHessian(a, b, c, d));

  EXPECT_NEAR(
      dc::pointPointSquaredDistance(a, d),
      nb::pointPointSquaredDistance(a, d),
      1e-14);
  expectMatrixNear(
      dc::pointPointSquaredDistanceGradient(a, d),
      nb::pointPointSquaredDistanceGradient(a, d));
  expectMatrixNear(
      dc::pointPointSquaredDistanceHessian(),
      nb::pointPointSquaredDistanceHessian());
}

//==============================================================================
TEST(NewtonBarrierPrimitives, BarrierForwardingMatchesSharedOwner)
{
  const Eigen::Vector3d p(0.25, 0.35, 0.4);
  const Eigen::Vector3d a(0.0, 0.0, 0.0);
  const Eigen::Vector3d b(1.25, 0.1, 0.0);
  const Eigen::Vector3d c(0.1, 1.5, 0.25);
  const Eigen::Vector3d d(0.25, 0.4, 1.2);
  constexpr double kDhat2 = 2.0;
  constexpr double kKappa = 3.5;

  expectScalarBarrierNear(
      dc::c2ClampedLogBarrier(0.25, kDhat2),
      nb::c2ClampedLogBarrier(0.25, kDhat2));
  expectScalarBarrierNear(
      dc::c2ClampedLogBarrier(4.0, kDhat2),
      nb::c2ClampedLogBarrier(4.0, kDhat2));
  expectScalarBarrierNear(
      dc::c2ClampedLogBarrier(0.0, kDhat2),
      nb::c2ClampedLogBarrier(0.0, kDhat2));

  expectBarrierNear(
      dc::pointTriangleBarrier(p, a, b, c, kDhat2, kKappa),
      nb::pointTriangleBarrier(p, a, b, c, kDhat2, kKappa));
  expectBarrierNear(
      dc::pointEdgeBarrier(p, a, b, kDhat2, kKappa),
      nb::pointEdgeBarrier(p, a, b, kDhat2, kKappa));
  expectBarrierNear(
      dc::edgeEdgeBarrier(a, b, c, d, kDhat2, kKappa),
      nb::edgeEdgeBarrier(a, b, c, d, kDhat2, kKappa));
  expectBarrierNear(
      dc::pointPointBarrier(a, d, kDhat2, kKappa),
      nb::pointPointBarrier(a, d, kDhat2, kKappa));
  expectBarrierNear(
      dc::mollifiedEdgeEdgeBarrier(a, b, c, d, kDhat2, 1e-3, kKappa),
      nb::mollifiedEdgeEdgeBarrier(a, b, c, d, kDhat2, 1e-3, kKappa));
}

//==============================================================================
TEST(NewtonBarrierPrimitives, TangentForwardingMatchesSharedOwner)
{
  const Eigen::Vector3d p(0.25, 0.35, 0.4);
  const Eigen::Vector3d a(0.0, 0.0, 0.0);
  const Eigen::Vector3d b(1.25, 0.1, 0.0);
  const Eigen::Vector3d c(0.1, 1.5, 0.25);
  const Eigen::Vector3d d(0.25, 0.4, 1.2);

  const auto oldPointTriangle = dc::pointTriangleTangentStencil(p, a, b, c);
  const auto newPointTriangle = nb::pointTriangleTangentStencil(p, a, b, c);
  EXPECT_EQ(
      oldPointTriangle.usedFallbackBasis, newPointTriangle.usedFallbackBasis);
  expectMatrixNear(oldPointTriangle.basis, newPointTriangle.basis);
  expectMatrixNear(oldPointTriangle.coordinates, newPointTriangle.coordinates);
  expectMatrixNear(oldPointTriangle.projection, newPointTriangle.projection);
  expectMatrixNear(oldPointTriangle.metric, newPointTriangle.metric);

  const auto oldPointEdge = dc::pointEdgeTangentStencil(p, a, b);
  const auto newPointEdge = nb::pointEdgeTangentStencil(p, a, b);
  EXPECT_EQ(oldPointEdge.usedFallbackBasis, newPointEdge.usedFallbackBasis);
  EXPECT_NEAR(oldPointEdge.coordinate, newPointEdge.coordinate, 1e-14);
  expectMatrixNear(oldPointEdge.basis, newPointEdge.basis);
  expectMatrixNear(oldPointEdge.projection, newPointEdge.projection);
  expectMatrixNear(oldPointEdge.metric, newPointEdge.metric);

  const auto oldEdgeEdge = dc::edgeEdgeTangentStencil(a, b, c, d);
  const auto newEdgeEdge = nb::edgeEdgeTangentStencil(a, b, c, d);
  EXPECT_EQ(oldEdgeEdge.usedFallbackBasis, newEdgeEdge.usedFallbackBasis);
  expectMatrixNear(oldEdgeEdge.basis, newEdgeEdge.basis);
  expectMatrixNear(oldEdgeEdge.coordinates, newEdgeEdge.coordinates);
  expectMatrixNear(oldEdgeEdge.projection, newEdgeEdge.projection);
  expectMatrixNear(oldEdgeEdge.metric, newEdgeEdge.metric);

  const auto oldPointPoint = dc::pointPointTangentStencil(a, d);
  const auto newPointPoint = nb::pointPointTangentStencil(a, d);
  EXPECT_EQ(oldPointPoint.usedFallbackBasis, newPointPoint.usedFallbackBasis);
  expectMatrixNear(oldPointPoint.basis, newPointPoint.basis);
  expectMatrixNear(oldPointPoint.projection, newPointPoint.projection);
  expectMatrixNear(oldPointPoint.metric, newPointPoint.metric);
}

//==============================================================================
TEST(NewtonBarrierPrimitives, PsdProjectionClampsNegativeEigenvalues)
{
  Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero();
  matrix.diagonal() << 4.0, -2.0, 1.0;

  Eigen::Matrix3d expected = Eigen::Matrix3d::Zero();
  expected.diagonal() << 4.0, 0.0, 1.0;

  expectMatrixNear(nb::projectSymmetricMatrixToPsd<3>(matrix), expected);
}

//==============================================================================
TEST(NewtonBarrierPrimitives, PsdProjectionSymmetrizesBeforeProjection)
{
  Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero();
  matrix << 1.0, 2.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -3.0;

  Eigen::Matrix3d expected = Eigen::Matrix3d::Zero();
  expected << 1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0;

  expectMatrixNear(nb::projectSymmetricMatrixToPsd<3>(matrix), expected);
}

//==============================================================================
TEST(NewtonBarrierPrimitives, RigidIpcConsumesSharedPrimitiveOwner)
{
  const Eigen::Vector3d point(0.25, 0.35, 0.4);
  const Eigen::Vector3d a(0.0, 0.0, 0.0);
  const Eigen::Vector3d b(1.25, 0.1, 0.0);
  const Eigen::Vector3d c(0.1, 1.5, 0.25);
  const rigid::RigidIpcPose identity;
  const rigid::RigidIpcBarrierOptions barrierOptions{
      .squaredActivationDistance = 2.0,
      .stiffness = 3.5,
      .projectReducedHessianToPsd = false};

  const auto reduced = rigid::rigidIpcPointTriangleReducedBarrier(
      point, identity, a, b, c, identity, barrierOptions);
  const auto expectedPrimitive
      = nb::pointTriangleBarrier(point, a, b, c, 2.0, 3.5);
  expectBarrierNear(reduced.primitive, expectedPrimitive);

  const Eigen::Vector3d laggedA(0.0, 0.0, 0.0);
  const Eigen::Vector3d laggedB(0.0, 0.0, 1.0);
  const Eigen::Vector3d pointA(0.2, 0.0, 0.0);
  const Eigen::Vector3d pointB(0.0, 0.3, 1.0);
  const rigid::RigidIpcFrictionOptions frictionOptions{
      .coefficient = 0.5,
      .laggedNormalForce = 4.0,
      .staticFrictionDisplacement = 0.1,
      .projectReducedHessianToPsd = false};

  const auto potential = rigid::rigidIpcPointPointFrictionPotential(
      laggedA, laggedB, pointA, pointB, frictionOptions);
  const auto stencil = nb::pointPointTangentStencil(laggedA, laggedB);
  nb::Vector6d displacement;
  displacement.head<3>() = pointA - laggedA;
  displacement.tail<3>() = pointB - laggedB;
  const auto expectedPotential = nb::projectedFrictionPotential<6>(
      stencil.projection,
      displacement,
      frictionOptions.coefficient * frictionOptions.laggedNormalForce,
      frictionOptions.staticFrictionDisplacement);
  expectMatrixNear(
      potential.tangentialDisplacement, stencil.projection * displacement);
  EXPECT_NEAR(potential.value, expectedPotential.value, 1e-14);
  expectMatrixNear(potential.gradient.head<6>(), expectedPotential.gradient);
  expectMatrixNear(
      potential.hessian.topLeftCorner<6, 6>(), expectedPotential.hessian);
  EXPECT_TRUE(potential.active);
  EXPECT_GT(potential.value, 0.0);
}
