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
#include <dart/simulation/detail/deformable_contact/continuous_collision_step.hpp>
#include <dart/simulation/detail/deformable_contact/primitive_distance.hpp>
#include <dart/simulation/detail/deformable_contact/tangent_stencil.hpp>
#include <dart/simulation/detail/newton_barrier/articulation_constraint.hpp>
#include <dart/simulation/detail/newton_barrier/barrier_kernel.hpp>
#include <dart/simulation/detail/newton_barrier/friction_kernel.hpp>
#include <dart/simulation/detail/newton_barrier/line_search.hpp>
#include <dart/simulation/detail/newton_barrier/primitive_distance.hpp>
#include <dart/simulation/detail/newton_barrier/projected_newton.hpp>
#include <dart/simulation/detail/newton_barrier/psd_projection.hpp>
#include <dart/simulation/detail/newton_barrier/tangent_stencil.hpp>
#include <dart/simulation/detail/rigid_ipc_barrier.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <type_traits>

#include <cmath>

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

//==============================================================================
template <int Outputs, int Inputs, typename Function>
Eigen::Matrix<double, Outputs, Inputs> finiteDifferenceJacobian(
    const Eigen::Matrix<double, Inputs, 1>& x,
    Function&& function,
    const double epsilon = 1e-6)
{
  Eigen::Matrix<double, Outputs, Inputs> jacobian;
  for (int col = 0; col < Inputs; ++col) {
    Eigen::Matrix<double, Inputs, 1> xPlus = x;
    Eigen::Matrix<double, Inputs, 1> xMinus = x;
    xPlus[col] += epsilon;
    xMinus[col] -= epsilon;
    jacobian.col(col) = (function(xPlus) - function(xMinus)) / (2.0 * epsilon);
  }
  return jacobian;
}

} // namespace

//==============================================================================
TEST(NewtonBarrierPrimitives, ArticulationPointConnectionAndFixedPoint)
{
  Eigen::Vector3d pointA(0.25, -0.1, 0.4);
  Eigen::Vector3d pointB(-0.2, 0.3, 0.1);
  const auto connection = nb::pointConnectionConstraint(pointA, pointB);
  EXPECT_TRUE(connection.residual.isApprox(pointA - pointB, 1e-14));
  EXPECT_NEAR(connection.squaredNorm, (pointA - pointB).squaredNorm(), 1e-14);

  Eigen::Matrix<double, 6, 1> pairState;
  pairState << pointA, pointB;
  const auto numericalConnectionJacobian
      = finiteDifferenceJacobian<3, 6>(pairState, [](const auto& x) {
          return nb::pointConnectionConstraint(
                     x.template head<3>(), x.template tail<3>())
              .residual;
        });
  expectMatrixNear(connection.jacobian, numericalConnectionJacobian, 1e-10);

  const Eigen::Vector3d target(0.0, -0.2, 0.5);
  const auto fixed = nb::fixedPointConstraint(pointA, target);
  EXPECT_TRUE(fixed.residual.isApprox(pointA - target, 1e-14));
  EXPECT_NEAR(fixed.squaredNorm, (pointA - target).squaredNorm(), 1e-14);

  const auto numericalFixedJacobian
      = finiteDifferenceJacobian<3, 3>(pointA, [&](const auto& point) {
          return nb::fixedPointConstraint(point, target).residual;
        });
  expectMatrixNear(fixed.jacobian, numericalFixedJacobian, 1e-10);
}

//==============================================================================
TEST(NewtonBarrierPrimitives, ArticulationHingeAndConeTwist)
{
  const Eigen::Vector3d axisA = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d axisB
      = Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitY())
        * Eigen::Vector3d::UnitZ();

  const auto hinge = nb::hingeAxisConstraint(axisA, axisB);
  EXPECT_TRUE(hinge.residual.isApprox(axisA.cross(axisB), 1e-14));
  EXPECT_NEAR(hinge.squaredNorm, std::pow(std::sin(0.25), 2), 1e-14);

  const auto aligned = nb::hingeAxisConstraint(axisA, 2.0 * axisA);
  EXPECT_TRUE(aligned.residual.isZero(1e-14));
  EXPECT_NEAR(aligned.squaredNorm, 0.0, 1e-14);

  const double bend = 0.35;
  const double twist = -0.45;
  const Eigen::AngleAxisd bendRotation(bend, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd twistRotation(twist, axisA);
  const Eigen::Vector3d referenceA = Eigen::Vector3d::UnitX();
  const Eigen::Vector3d coneAxisB = bendRotation * axisA;
  const Eigen::Vector3d referenceB = bendRotation * twistRotation * referenceA;

  const auto cone
      = nb::coneTwistCoordinates(axisA, referenceA, coneAxisB, referenceB);
  EXPECT_NEAR(cone.bendAngle, bend, 1e-14);
  EXPECT_NEAR(cone.twistAngle, twist, 1e-14);
}

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
  using DcLineSearchOptions = dc::ContinuousCollisionStepOptions;
  using DcLineSearchStats = dc::ContinuousCollisionStepStats;
  using RigidLineSearchOptions = rigid::RigidIpcLineSearchOptions;
  using RigidLineSearchStats = rigid::RigidIpcLineSearchStats;
  static_assert(std::is_same_v<DcLineSearchOptions, nb::LineSearchOptions>);
  static_assert(std::is_same_v<DcLineSearchStats, nb::LineSearchStats>);
  static_assert(std::is_same_v<RigidLineSearchOptions, nb::LineSearchOptions>);
  static_assert(std::is_same_v<RigidLineSearchStats, nb::LineSearchStats>);

  EXPECT_DOUBLE_EQ(dc::detail::kRelativeEpsilon, nb::detail::kRelativeEpsilon);
  EXPECT_DOUBLE_EQ(
      dc::detail::kBarrierDistanceFloorScale,
      nb::detail::kBarrierDistanceFloorScale);
  EXPECT_DOUBLE_EQ(
      dc::detail::kTangentBasisEpsilon, nb::detail::kTangentBasisEpsilon);
}

//==============================================================================
TEST(NewtonBarrierPrimitives, LineSearchStatsAccumulateSharedCounters)
{
  const nb::LineSearchOptions options;
  EXPECT_DOUBLE_EQ(options.minSeparation, 0.0);
  EXPECT_DOUBLE_EQ(options.tolerance, 1e-6);
  EXPECT_EQ(options.maxIterations, 64);

  nb::LineSearchStats total;
  nb::LineSearchStats addend;
  addend.pointPointChecks = 1;
  addend.pointEdgeChecks = 2;
  addend.edgeEdgeChecks = 3;
  addend.pointTriangleChecks = 4;
  addend.hits = 5;
  addend.misses = 6;
  addend.indeterminate = 7;
  addend.zeroStepCount = 8;

  nb::accumulateLineSearchStats(total, addend);
  nb::accumulateLineSearchStats(total, addend);

  EXPECT_EQ(total.pointPointChecks, 2u);
  EXPECT_EQ(total.pointEdgeChecks, 4u);
  EXPECT_EQ(total.edgeEdgeChecks, 6u);
  EXPECT_EQ(total.pointTriangleChecks, 8u);
  EXPECT_EQ(total.hits, 10u);
  EXPECT_EQ(total.misses, 12u);
  EXPECT_EQ(total.indeterminate, 14u);
  EXPECT_EQ(total.zeroStepCount, 16u);
}

//==============================================================================
TEST(NewtonBarrierPrimitives, LineSearchCcdOptionUsesSharedConservativePolicy)
{
  nb::LineSearchOptions options;
  options.minSeparation = -0.25;
  options.tolerance = -1.0;
  options.maxIterations = 0;

  const auto ccdOption = nb::makeLineSearchCcdOption(options);

  EXPECT_DOUBLE_EQ(ccdOption.minSeparation, 0.0);
  EXPECT_DOUBLE_EQ(ccdOption.tolerance, 0.0);
  EXPECT_EQ(ccdOption.maxIterations, 1);
  EXPECT_EQ(
      ccdOption.advancement,
      dart::collision::native::CcdAdvancement::Conservative);

  options.minSeparation = 0.125;
  options.tolerance = 1e-4;
  options.maxIterations = 17;

  const auto positiveOption = nb::makeLineSearchCcdOption(options);

  EXPECT_DOUBLE_EQ(positiveOption.minSeparation, 0.125);
  EXPECT_DOUBLE_EQ(positiveOption.tolerance, 1e-4);
  EXPECT_EQ(positiveOption.maxIterations, 17);
  EXPECT_EQ(
      positiveOption.advancement,
      dart::collision::native::CcdAdvancement::Conservative);
}

//==============================================================================
TEST(NewtonBarrierPrimitives, LineSearchPositiveStepPredicateIsShared)
{
  EXPECT_TRUE(nb::allowsPositiveLineSearchStep(1.0, false));
  EXPECT_FALSE(nb::allowsPositiveLineSearchStep(0.0, false));
  EXPECT_FALSE(nb::allowsPositiveLineSearchStep(-1.0, false));
  EXPECT_FALSE(nb::allowsPositiveLineSearchStep(0.5, true));
}

//==============================================================================
TEST(NewtonBarrierPrimitives, LineSearchFullStepPredicateIsShared)
{
  EXPECT_TRUE(nb::allowsFullLineSearchStep(1.0, true, false));
  EXPECT_TRUE(nb::allowsFullLineSearchStep(0.25, false, false));
  EXPECT_FALSE(nb::allowsFullLineSearchStep(0.25, true, false));
  EXPECT_FALSE(nb::allowsFullLineSearchStep(1.0, true, true));

  dc::ContinuousCollisionStepResult deformable;
  rigid::RigidIpcLineSearchResult rigidResult;
  EXPECT_TRUE(deformable.allowsFullStep());
  EXPECT_TRUE(rigidResult.allowsFullStep());

  deformable.hit = true;
  deformable.stepBound = 0.25;
  rigidResult.limited = true;
  rigidResult.stepBound = 0.25;
  EXPECT_FALSE(deformable.allowsFullStep());
  EXPECT_FALSE(rigidResult.allowsFullStep());
}

//==============================================================================
TEST(NewtonBarrierPrimitives, LineSearchStepScaleUsesSharedClampingPolicy)
{
  EXPECT_NEAR(nb::makeLineSearchStepScale(0.25, 0.8), 0.2, 1e-15);
  EXPECT_DOUBLE_EQ(nb::makeLineSearchStepScale(2.0, 2.0), 1.0);
  EXPECT_DOUBLE_EQ(nb::makeLineSearchStepScale(-0.5, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(nb::makeLineSearchStepScale(0.5, -0.5), 0.0);
  EXPECT_DOUBLE_EQ(
      nb::makeLineSearchStepScale(
          std::numeric_limits<double>::quiet_NaN(), 1.0),
      0.0);
  EXPECT_DOUBLE_EQ(
      nb::makeLineSearchStepScale(
          0.5, std::numeric_limits<double>::quiet_NaN()),
      0.0);
}

//==============================================================================
TEST(NewtonBarrierPrimitives, InteriorLineSearchStepScaleStaysInsideBound)
{
  const double fullStep = nb::makeInteriorLineSearchStepScale(1.0);
  EXPECT_GT(fullStep, 0.0);
  EXPECT_LT(fullStep, 1.0);

  const double partialStep = nb::makeInteriorLineSearchStepScale(0.25);
  EXPECT_GT(partialStep, 0.0);
  EXPECT_LT(partialStep, 0.25);

  EXPECT_DOUBLE_EQ(nb::makeInteriorLineSearchStepScale(0.0), 0.0);
  EXPECT_DOUBLE_EQ(
      nb::makeInteriorLineSearchStepScale(
          std::numeric_limits<double>::denorm_min()),
      0.0);
  EXPECT_DOUBLE_EQ(
      nb::makeInteriorLineSearchStepScale(
          std::numeric_limits<double>::quiet_NaN()),
      0.0);
}

//==============================================================================
TEST(NewtonBarrierPrimitives, SufficientDecreasePolicySanitizesSharedScalars)
{
  EXPECT_DOUBLE_EQ(
      nb::sanitizeSufficientDecreaseFactor(
          std::numeric_limits<double>::quiet_NaN()),
      nb::kDefaultSufficientDecreaseFactor);
  EXPECT_DOUBLE_EQ(nb::sanitizeSufficientDecreaseFactor(-1.0), 0.0);
  EXPECT_DOUBLE_EQ(
      nb::sanitizeSufficientDecreaseFactor(2.0), std::nextafter(1.0, 0.0));
  EXPECT_DOUBLE_EQ(nb::sanitizeSufficientDecreaseFactor(1e-3), 1e-3);

  EXPECT_DOUBLE_EQ(
      nb::sanitizeBacktrackingScale(std::numeric_limits<double>::quiet_NaN()),
      nb::kDefaultBacktrackingScale);
  EXPECT_DOUBLE_EQ(
      nb::sanitizeBacktrackingScale(0.0), nb::kDefaultBacktrackingScale);
  EXPECT_DOUBLE_EQ(
      nb::sanitizeBacktrackingScale(1.0), nb::kDefaultBacktrackingScale);
  EXPECT_DOUBLE_EQ(nb::sanitizeBacktrackingScale(0.25), 0.25);
}

//==============================================================================
TEST(NewtonBarrierPrimitives, SufficientDecreasePolicyChecksArmijoThreshold)
{
  EXPECT_DOUBLE_EQ(nb::sufficientDecreaseThreshold(10.0, -20.0), 9.998);
  EXPECT_TRUE(nb::satisfiesSufficientDecrease(10.0, 9.998, -20.0));
  EXPECT_FALSE(nb::satisfiesSufficientDecrease(10.0, 9.9981, -20.0));

  EXPECT_TRUE(nb::satisfiesSufficientDecrease(10.0, 9.0, -2.0, 0.5));
  EXPECT_FALSE(nb::satisfiesSufficientDecrease(10.0, 9.1, -2.0, 0.5));
  EXPECT_FALSE(
      nb::satisfiesSufficientDecrease(
          10.0, std::numeric_limits<double>::quiet_NaN(), -2.0));
  EXPECT_FALSE(
      nb::satisfiesSufficientDecrease(
          10.0, 9.0, std::numeric_limits<double>::quiet_NaN()));
}

//==============================================================================
TEST(NewtonBarrierPrimitives, LineSearchHitAccountingClampsSharedStepBound)
{
  nb::LineSearchStats stats;

  EXPECT_DOUBLE_EQ(nb::recordLineSearchHit(stats, 0.25), 0.25);
  EXPECT_EQ(stats.hits, 1u);
  EXPECT_EQ(stats.zeroStepCount, 0u);

  EXPECT_DOUBLE_EQ(nb::recordLineSearchHit(stats, 2.0), 1.0);
  EXPECT_EQ(stats.hits, 2u);
  EXPECT_EQ(stats.zeroStepCount, 0u);

  EXPECT_DOUBLE_EQ(nb::recordLineSearchHit(stats, -0.5), 0.0);
  EXPECT_EQ(stats.hits, 3u);
  EXPECT_EQ(stats.zeroStepCount, 1u);
}

//==============================================================================
TEST(NewtonBarrierPrimitives, LineSearchCcdOutcomeAccountsNativeStatus)
{
  nb::LineSearchStats stats;

  dart::collision::native::CcdPrimitiveResult hit;
  hit.hit = true;
  hit.status = dart::collision::native::CcdPrimitiveStatus::Hit;
  hit.timeOfImpact = 1.25;
  const auto hitOutcome = nb::recordLineSearchCcdOutcome(stats, hit);
  EXPECT_TRUE(hitOutcome.hit);
  EXPECT_FALSE(hitOutcome.indeterminate);
  EXPECT_DOUBLE_EQ(hitOutcome.stepBound, 1.0);
  EXPECT_EQ(stats.hits, 1u);
  EXPECT_EQ(stats.misses, 0u);
  EXPECT_EQ(stats.indeterminate, 0u);

  dart::collision::native::CcdPrimitiveResult miss;
  miss.status = dart::collision::native::CcdPrimitiveStatus::Miss;
  const auto missOutcome = nb::recordLineSearchCcdOutcome(stats, miss);
  EXPECT_FALSE(missOutcome.hit);
  EXPECT_FALSE(missOutcome.indeterminate);
  EXPECT_DOUBLE_EQ(missOutcome.stepBound, 1.0);
  EXPECT_EQ(stats.hits, 1u);
  EXPECT_EQ(stats.misses, 1u);
  EXPECT_EQ(stats.indeterminate, 0u);

  dart::collision::native::CcdPrimitiveResult indeterminate;
  indeterminate.status
      = dart::collision::native::CcdPrimitiveStatus::Indeterminate;
  indeterminate.timeOfImpact = 0.25;
  const auto indeterminateOutcome
      = nb::recordLineSearchCcdOutcome(stats, indeterminate);
  EXPECT_FALSE(indeterminateOutcome.hit);
  EXPECT_TRUE(indeterminateOutcome.indeterminate);
  EXPECT_DOUBLE_EQ(indeterminateOutcome.stepBound, 0.25);
  EXPECT_EQ(stats.hits, 1u);
  EXPECT_EQ(stats.misses, 1u);
  EXPECT_EQ(stats.indeterminate, 1u);
  EXPECT_EQ(stats.zeroStepCount, 0u);

  indeterminate.timeOfImpact = -0.25;
  const auto zeroIndeterminateOutcome
      = nb::recordLineSearchCcdOutcome(stats, indeterminate);
  EXPECT_FALSE(zeroIndeterminateOutcome.hit);
  EXPECT_TRUE(zeroIndeterminateOutcome.indeterminate);
  EXPECT_DOUBLE_EQ(zeroIndeterminateOutcome.stepBound, 0.0);
  EXPECT_EQ(stats.hits, 1u);
  EXPECT_EQ(stats.misses, 1u);
  EXPECT_EQ(stats.indeterminate, 2u);
  EXPECT_EQ(stats.zeroStepCount, 1u);
}

//==============================================================================
TEST(NewtonBarrierPrimitives, LineSearchResultsUseSharedPositiveStepPredicate)
{
  dc::ContinuousCollisionStepResult deformable;
  rigid::RigidIpcLineSearchResult rigidResult;

  deformable.stepBound = 0.25;
  rigidResult.limited = true;
  rigidResult.stepBound = 0.25;
  EXPECT_TRUE(deformable.allowsPositiveStep());
  EXPECT_TRUE(rigidResult.allowsPositiveStep());

  deformable.stepBound = 0.0;
  rigidResult.stepBound = 0.0;
  EXPECT_FALSE(deformable.allowsPositiveStep());
  EXPECT_FALSE(rigidResult.allowsPositiveStep());

  deformable.stepBound = 0.25;
  rigidResult.stepBound = 0.25;
  deformable.indeterminate = true;
  rigidResult.indeterminate = true;
  EXPECT_FALSE(deformable.allowsPositiveStep());
  EXPECT_FALSE(rigidResult.allowsPositiveStep());
}

//==============================================================================
TEST(NewtonBarrierPrimitives, ProjectedNewtonDiagnosticsUseSharedPolicy)
{
  EXPECT_DOUBLE_EQ(nb::sanitizeProjectedNewtonTolerance(-1.0), 0.0);
  EXPECT_DOUBLE_EQ(
      nb::sanitizeProjectedNewtonTolerance(
          std::numeric_limits<double>::quiet_NaN()),
      0.0);
  EXPECT_DOUBLE_EQ(nb::sanitizeProjectedNewtonTolerance(1e-8), 1e-8);

  EXPECT_DOUBLE_EQ(nb::projectedNewtonResidualNormFromSquared(-1.0), 0.0);
  EXPECT_DOUBLE_EQ(nb::projectedNewtonResidualNormFromSquared(4.0), 2.0);

  EXPECT_TRUE(nb::projectedNewtonResidualConverged(1e-9, 1e-8));
  EXPECT_FALSE(nb::projectedNewtonResidualConverged(1e-7, 1e-8));
  EXPECT_TRUE(nb::projectedNewtonSquaredResidualConverged(1e-18, 1e-9));
  EXPECT_FALSE(nb::projectedNewtonSquaredResidualConverged(1e-16, 1e-9));

  EXPECT_DOUBLE_EQ(
      nb::projectedNewtonEffectiveGradientTolerance(1e-8, 1e-3, 2.0), 2e-3);
  EXPECT_DOUBLE_EQ(
      nb::projectedNewtonEffectiveGradientTolerance(1e-8, -1.0, 2.0), 1e-8);
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
  EXPECT_NEAR(potential.work, expectedPotential.work, 1e-14);
  expectMatrixNear(potential.gradient.head<6>(), expectedPotential.gradient);
  expectMatrixNear(
      potential.hessian.topLeftCorner<6, 6>(), expectedPotential.hessian);
  EXPECT_TRUE(potential.active);
  EXPECT_GT(potential.value, 0.0);
}

//==============================================================================
TEST(NewtonBarrierPrimitives, FrictionWorkUsesSharedMollifier)
{
  const auto staticContribution = nb::frictionWorkContribution(0.05, 4.0, 0.2);
  EXPECT_TRUE(staticContribution.active);
  EXPECT_FALSE(staticContribution.dynamicBranch);
  EXPECT_NEAR(staticContribution.work, 4.0 * (0.5 - 0.0625) * 0.05, 1e-15);

  const auto dynamicContribution = nb::frictionWorkContribution(0.4, 4.0, 0.2);
  EXPECT_TRUE(dynamicContribution.active);
  EXPECT_TRUE(dynamicContribution.dynamicBranch);
  EXPECT_NEAR(dynamicContribution.work, 4.0 * 0.4, 1e-15);

  const double tinyStaticDisplacement
      = std::numeric_limits<double>::denorm_min();
  const auto nonFiniteSmoothContribution = nb::frictionWorkContribution(
      tinyStaticDisplacement, 4.0, tinyStaticDisplacement);
  EXPECT_FALSE(nonFiniteSmoothContribution.active);
  EXPECT_DOUBLE_EQ(nonFiniteSmoothContribution.work, 0.0);

  EXPECT_FALSE(nb::frictionWorkContribution(0.1, 0.0, 0.2).active);
  EXPECT_FALSE(nb::frictionWorkContribution(0.1, 4.0, 0.0).active);
}
