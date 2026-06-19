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
#include <dart/simulation/detail/newton_barrier/change_of_variable.hpp>
#include <dart/simulation/detail/newton_barrier/friction_kernel.hpp>
#include <dart/simulation/detail/newton_barrier/line_search.hpp>
#include <dart/simulation/detail/newton_barrier/mixed_domain_coupling.hpp>
#include <dart/simulation/detail/newton_barrier/primitive_distance.hpp>
#include <dart/simulation/detail/newton_barrier/projected_newton.hpp>
#include <dart/simulation/detail/newton_barrier/psd_projection.hpp>
#include <dart/simulation/detail/newton_barrier/restitution_damping.hpp>
#include <dart/simulation/detail/newton_barrier/tangent_stencil.hpp>
#include <dart/simulation/detail/rigid_ipc/rigid_ipc_barrier.hpp>

#include <dart/common/free_list_allocator.hpp>
#include <dart/common/memory_allocator_debugger.hpp>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include <limits>
#include <type_traits>
#include <vector>

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
template <typename Derived>
void expectSelfAdjointPsd(
    const Eigen::MatrixBase<Derived>& matrix, const double tolerance = 1e-12)
{
  EXPECT_TRUE(matrix.isApprox(matrix.transpose(), tolerance));
  const Eigen::SelfAdjointEigenSolver<typename Derived::PlainObject> solver(
      matrix);
  ASSERT_EQ(solver.info(), Eigen::Success);
  EXPECT_GE(solver.eigenvalues().minCoeff(), -tolerance);
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
TEST(NewtonBarrierPrimitives, EqualityChangeOfVariableDetectsRankAndResidual)
{
  Eigen::SparseMatrix<double> jacobian(3, 4);
  jacobian.insert(0, 0) = 2.0;
  jacobian.insert(0, 2) = 0.25;
  jacobian.insert(1, 1) = 3.0;
  jacobian.insert(1, 3) = -0.5;
  jacobian.insert(2, 0) = 4.0;
  jacobian.insert(2, 1) = -3.0;
  jacobian.insert(2, 2) = 0.5;
  jacobian.insert(2, 3) = 0.5;
  jacobian.makeCompressed();

  Eigen::Vector3d residual;
  residual << -4.0, 6.0, -14.0;

  const auto change = nb::makeEqualityChangeOfVariable(jacobian, residual);
  ASSERT_TRUE(change.valid);
  EXPECT_EQ(change.rank, 2);
  EXPECT_FALSE(change.fullRowRank);
  EXPECT_EQ(change.independentRows.size(), 2u);
  EXPECT_EQ(change.constrainedColumns.size(), 2u);
  EXPECT_EQ(change.freeColumns.size(), 2u);

  const Eigen::MatrixXd denseJacobian(jacobian);
  EXPECT_NEAR(
      (denseJacobian * change.particularStep + residual).norm(), 0.0, 1e-12);
  expectMatrixNear(
      denseJacobian * change.nullspaceBasis,
      Eigen::MatrixXd::Zero(3, 2),
      1e-12);
  EXPECT_NEAR(change.residualNorm, 0.0, 1e-12);
}

//==============================================================================
TEST(NewtonBarrierPrimitives, EqualityChangeOfVariableUsesProvidedAllocator)
{
  dart::common::MemoryAllocatorDebugger<dart::common::FreeListAllocator>
      allocator;
  ASSERT_TRUE(allocator.isEmpty());

  {
    Eigen::SparseMatrix<double> jacobian(3, 4);
    jacobian.insert(0, 0) = 2.0;
    jacobian.insert(0, 2) = 0.25;
    jacobian.insert(1, 1) = 3.0;
    jacobian.insert(1, 3) = -0.5;
    jacobian.insert(2, 0) = 4.0;
    jacobian.insert(2, 1) = -3.0;
    jacobian.insert(2, 2) = 0.5;
    jacobian.insert(2, 3) = 0.5;
    jacobian.makeCompressed();

    Eigen::Vector3d residual;
    residual << -4.0, 6.0, -14.0;

    const auto change
        = nb::makeEqualityChangeOfVariable(jacobian, residual, allocator);
    ASSERT_TRUE(change.valid);
    EXPECT_EQ(change.independentRows.size(), 2u);
    EXPECT_EQ(change.constrainedColumns.size(), 2u);
    EXPECT_EQ(change.freeColumns.size(), 2u);
    EXPECT_GT(allocator.getAllocationCount(), 0u);
    EXPECT_GT(allocator.getPeakAllocatedSize(), 0u);
  }

  EXPECT_TRUE(allocator.isEmpty());
}

//==============================================================================
TEST(NewtonBarrierPrimitives, EqualityChangeOfVariableKeepsFreeSparseBasis)
{
  Eigen::SparseMatrix<double> jacobian(2, 5);
  jacobian.insert(0, 0) = 4.0;
  jacobian.insert(0, 2) = 0.25;
  jacobian.insert(1, 1) = 5.0;
  jacobian.insert(1, 3) = -0.5;
  jacobian.insert(1, 4) = 0.125;
  jacobian.makeCompressed();

  Eigen::Vector2d residual;
  residual << -2.0, 3.0;

  const auto change = nb::makeEqualityChangeOfVariable(jacobian, residual);
  ASSERT_TRUE(change.valid);
  ASSERT_EQ(change.rank, 2);
  ASSERT_EQ(change.freeColumns.size(), 3u);
  ASSERT_EQ(change.nullspaceBasis.cols(), 3);

  for (int col = 0; col < static_cast<int>(change.freeColumns.size()); ++col) {
    const int freeDof = change.freeColumns[col];
    EXPECT_NEAR(change.nullspaceBasis(freeDof, col), 1.0, 1e-14);
    for (int other = 0; other < static_cast<int>(change.freeColumns.size());
         ++other) {
      if (other == col) {
        continue;
      }
      EXPECT_NEAR(
          change.nullspaceBasis(change.freeColumns[other], col), 0.0, 1e-14);
    }
  }

  int nonzeros = 0;
  for (int row = 0; row < change.nullspaceBasis.rows(); ++row) {
    for (int col = 0; col < change.nullspaceBasis.cols(); ++col) {
      if (std::abs(change.nullspaceBasis(row, col)) > 1e-14) {
        ++nonzeros;
      }
    }
  }
  EXPECT_LE(
      nonzeros,
      static_cast<int>(
          change.freeColumns.size() * (change.constrainedColumns.size() + 1)));

  const Eigen::MatrixXd denseJacobian(jacobian);
  expectMatrixNear(
      denseJacobian * change.nullspaceBasis,
      Eigen::MatrixXd::Zero(2, 3),
      1e-12);
}

//==============================================================================
TEST(NewtonBarrierPrimitives, EqualityChangeOfVariableMatchesKktSolve)
{
  Eigen::SparseMatrix<double> jacobian(2, 4);
  jacobian.insert(0, 0) = 2.0;
  jacobian.insert(0, 2) = 0.25;
  jacobian.insert(1, 1) = 3.0;
  jacobian.insert(1, 3) = -0.5;
  jacobian.makeCompressed();

  Eigen::Vector2d residual;
  residual << -1.0, 0.75;

  Eigen::Matrix4d hessian;
  hessian << 5.0, 0.2, 0.1, 0.0, 0.2, 4.0, 0.0, -0.1, 0.1, 0.0, 3.0, 0.25, 0.0,
      -0.1, 0.25, 2.5;
  Eigen::Vector4d gradient;
  gradient << -0.7, 0.4, 0.2, -0.3;

  const auto change = nb::makeEqualityChangeOfVariable(jacobian, residual);
  ASSERT_TRUE(change.valid);
  const Eigen::VectorXd reducedStep
      = nb::solveEqualityConstrainedQuadraticReduced(hessian, gradient, change);

  Eigen::Matrix<double, 6, 6> kkt = Eigen::Matrix<double, 6, 6>::Zero();
  kkt.topLeftCorner<4, 4>() = hessian;
  kkt.topRightCorner<4, 2>() = Eigen::MatrixXd(jacobian).transpose();
  kkt.bottomLeftCorner<2, 4>() = Eigen::MatrixXd(jacobian);

  Eigen::Matrix<double, 6, 1> rhs;
  rhs.head<4>() = -gradient;
  rhs.tail<2>() = -residual;
  const Eigen::Matrix<double, 6, 1> kktSolution = kkt.fullPivLu().solve(rhs);
  const Eigen::Vector4d kktStep = kktSolution.head<4>();

  expectMatrixNear(reducedStep, kktStep, 1e-12);
  EXPECT_NEAR(
      (Eigen::MatrixXd(jacobian) * reducedStep + residual).norm(), 0.0, 1e-12);
}

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
TEST(NewtonBarrierPrimitives, ArticulationSlidingAndRelativeSliding)
{
  const Eigen::Vector3d point(0.6, 0.25, -0.1);
  const Eigen::Vector3d origin(0.1, -0.05, 0.2);
  const Eigen::Vector3d axis = Eigen::Vector3d(1.0, 2.0, -1.0).normalized();
  const Eigen::Matrix3d projector
      = Eigen::Matrix3d::Identity() - axis * axis.transpose();

  const auto sliding = nb::slidingConstraint(point, origin, axis);
  EXPECT_TRUE(sliding.residual.isApprox(projector * (point - origin), 1e-14));
  EXPECT_NEAR(sliding.coordinate, axis.dot(point - origin), 1e-14);
  EXPECT_NEAR(sliding.squaredNorm, sliding.residual.squaredNorm(), 1e-14);

  const auto numericalSlidingJacobian
      = finiteDifferenceJacobian<3, 3>(point, [&](const auto& p) {
          return nb::slidingConstraint(p, origin, axis).residual;
        });
  expectMatrixNear(sliding.jacobian, numericalSlidingJacobian, 1e-10);

  const Eigen::Vector3d pointB(-0.2, 0.3, 0.5);
  Eigen::Matrix<double, 6, 1> pairState;
  pairState << point, pointB;
  const auto relative = nb::relativeSlidingConstraint(point, pointB, axis);
  EXPECT_TRUE(relative.residual.isApprox(projector * (point - pointB), 1e-14));
  EXPECT_NEAR(relative.coordinate, axis.dot(point - pointB), 1e-14);
  EXPECT_NEAR(relative.squaredNorm, relative.residual.squaredNorm(), 1e-14);

  const auto numericalRelativeJacobian
      = finiteDifferenceJacobian<3, 6>(pairState, [&](const auto& x) {
          return nb::relativeSlidingConstraint(
                     x.template head<3>(), x.template tail<3>(), axis)
              .residual;
        });
  expectMatrixNear(relative.jacobian, numericalRelativeJacobian, 1e-10);
}

//==============================================================================
TEST(NewtonBarrierPrimitives, ArticulationDistanceAndBoundedDistance)
{
  const Eigen::Vector3d pointA(0.35, -0.2, 0.45);
  const Eigen::Vector3d pointB(-0.15, 0.1, 0.05);
  const double targetDistance = 0.8;
  const auto distance = nb::distanceConstraint(pointA, pointB, targetDistance);
  EXPECT_NEAR(distance.distance, (pointA - pointB).norm(), 1e-14);
  EXPECT_NEAR(distance.residual, distance.distance - targetDistance, 1e-14);

  Eigen::Matrix<double, 6, 1> pairState;
  pairState << pointA, pointB;
  const auto numericalDistanceJacobian
      = finiteDifferenceJacobian<1, 6>(pairState, [&](const auto& x) {
          Eigen::Matrix<double, 1, 1> residual;
          residual[0]
              = nb::distanceConstraint(
                    x.template head<3>(), x.template tail<3>(), targetDistance)
                    .residual;
          return residual;
        });
  expectMatrixNear(distance.jacobian, numericalDistanceJacobian, 1e-10);

  const double lower = distance.distance - 0.05;
  const double upper = distance.distance + 0.5;
  const double activationDistance = 0.1;
  const auto bounded = nb::boundedDistanceBarrier(
      pointA, pointB, lower, upper, activationDistance, /*stiffness=*/2.0);
  EXPECT_TRUE(nb::rangeBarrierFeasible(bounded.margins));
  EXPECT_TRUE(bounded.active);
  EXPECT_TRUE(bounded.value > 0.0);
  EXPECT_NEAR(bounded.margins.lower, 0.05, 1e-14);
  EXPECT_NEAR(bounded.margins.upper, 0.5, 1e-14);
  expectSelfAdjointPsd(bounded.hessian);

  const auto numericalBarrierGradient
      = finiteDifferenceJacobian<1, 6>(pairState, [&](const auto& x) {
          Eigen::Matrix<double, 1, 1> value;
          value[0] = nb::boundedDistanceBarrier(
                         x.template head<3>(),
                         x.template tail<3>(),
                         lower,
                         upper,
                         activationDistance,
                         /*stiffness=*/2.0)
                         .value;
          return value;
        });
  expectMatrixNear(
      bounded.gradient.transpose(), numericalBarrierGradient, 1e-8);

  const auto infeasible = nb::boundedDistanceBarrier(
      pointA,
      pointB,
      distance.distance + 0.1,
      distance.distance + 0.5,
      activationDistance);
  EXPECT_FALSE(nb::rangeBarrierFeasible(infeasible.margins));
  EXPECT_TRUE(infeasible.active);
  EXPECT_TRUE(std::isfinite(infeasible.value));
}

//==============================================================================
TEST(NewtonBarrierPrimitives, ArticulationSlidingRangeAndRotationRange)
{
  const Eigen::Vector3d point(0.4, -0.2, 0.15);
  const Eigen::Vector3d origin(-0.1, 0.3, -0.05);
  const Eigen::Vector3d axis = Eigen::Vector3d(2.0, -1.0, 1.0).normalized();
  const double coordinate = axis.dot(point - origin);
  const double lower = coordinate - 0.04;
  const double upper = coordinate + 0.5;
  const double activationDistance = 0.1;

  const auto sliding = nb::slidingRangeBarrier(
      point, origin, axis, lower, upper, activationDistance, /*stiffness=*/3.0);
  EXPECT_TRUE(nb::rangeBarrierFeasible(sliding.margins));
  EXPECT_TRUE(sliding.active);
  EXPECT_NEAR(sliding.coordinate, coordinate, 1e-14);
  EXPECT_NEAR(sliding.margins.lower, 0.04, 1e-14);
  EXPECT_NEAR(sliding.margins.upper, 0.5, 1e-14);
  expectSelfAdjointPsd(sliding.hessian);

  const auto numericalSlidingRangeGradient
      = finiteDifferenceJacobian<1, 3>(point, [&](const auto& p) {
          Eigen::Matrix<double, 1, 1> value;
          value[0] = nb::slidingRangeBarrier(
                         p,
                         origin,
                         axis,
                         lower,
                         upper,
                         activationDistance,
                         /*stiffness=*/3.0)
                         .value;
          return value;
        });
  expectMatrixNear(
      sliding.gradient.transpose(), numericalSlidingRangeGradient, 1e-8);

  const double angle = 0.08;
  const auto rotation = nb::rotationRangeBarrier(
      angle, /*lower=*/0.0, /*upper=*/1.0, activationDistance, 2.0);
  EXPECT_TRUE(nb::rangeBarrierFeasible(rotation.margins));
  EXPECT_TRUE(rotation.active);
  EXPECT_GE(rotation.secondDerivative, 0.0);

  const auto numericalRotationDerivative = finiteDifferenceJacobian<1, 1>(
      Eigen::Matrix<double, 1, 1>::Constant(angle), [&](const auto& x) {
        Eigen::Matrix<double, 1, 1> value;
        value[0] = nb::rotationRangeBarrier(
                       x[0],
                       /*lower=*/0.0,
                       /*upper=*/1.0,
                       activationDistance,
                       2.0)
                       .value;
        return value;
      });
  EXPECT_NEAR(
      rotation.firstDerivative, numericalRotationDerivative(0, 0), 1e-8);

  const Eigen::Vector3d twistAxis = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d referenceA = Eigen::Vector3d::UnitX();
  const Eigen::Vector3d axisB = twistAxis;
  const Eigen::Vector3d referenceB
      = Eigen::AngleAxisd(angle, twistAxis) * referenceA;
  const auto coneTwistRange = nb::coneTwistRotationRangeBarrier(
      twistAxis,
      referenceA,
      axisB,
      referenceB,
      /*lower=*/0.0,
      /*upper=*/1.0,
      activationDistance,
      2.0);
  EXPECT_NEAR(coneTwistRange.angle, angle, 1e-14);
  EXPECT_NEAR(coneTwistRange.value, rotation.value, 1e-14);
}

//==============================================================================
TEST(NewtonBarrierPrimitives, ArticulationRangeBarrierHandlesTinyActivation)
{
  const double tiny = std::numeric_limits<double>::denorm_min();
  const auto barrier = nb::scalarRangeBarrier(
      /*coordinate=*/0.0,
      /*lower=*/0.0,
      /*upper=*/1.0,
      /*activationDistance=*/tiny,
      /*stiffness=*/1.0);

  EXPECT_TRUE(barrier.activeLower);
  EXPECT_TRUE(std::isfinite(barrier.value));
  EXPECT_TRUE(std::isfinite(barrier.firstDerivative));
  EXPECT_TRUE(std::isfinite(barrier.secondDerivative));
}

//==============================================================================
TEST(NewtonBarrierPrimitives, RestitutionVelocityTargetActivatesOnApproach)
{
  const auto target
      = nb::makeRestitutionVelocityTarget(/*approachVelocity=*/-2.0, 0.75);
  EXPECT_TRUE(target.active);
  EXPECT_DOUBLE_EQ(target.coefficient, 0.75);
  EXPECT_DOUBLE_EQ(target.targetSeparatingVelocity, 1.5);

  const auto clamped
      = nb::makeRestitutionVelocityTarget(/*approachVelocity=*/-2.0, 3.0);
  EXPECT_TRUE(clamped.active);
  EXPECT_DOUBLE_EQ(clamped.coefficient, 1.0);
  EXPECT_DOUBLE_EQ(clamped.targetSeparatingVelocity, 2.0);

  EXPECT_FALSE(
      nb::makeRestitutionVelocityTarget(/*approachVelocity=*/0.1, 0.75).active);
  EXPECT_FALSE(
      nb::makeRestitutionVelocityTarget(/*approachVelocity=*/-1e-4, 0.75)
          .active);
  EXPECT_FALSE(
      nb::makeRestitutionVelocityTarget(
          std::numeric_limits<double>::quiet_NaN(), 0.75)
          .active);
}

//==============================================================================
TEST(NewtonBarrierPrimitives, Bdf2HistoryRestartsThenSerializes)
{
  constexpr double timeStep = 0.1;
  nb::Bdf2StepHistory<1> history;
  history.currentPosition << 0.0;
  history.currentVelocity << 2.0;

  const auto restartTerm
      = nb::makeBdf2InertialTerm(history, /*mass=*/3.0, timeStep);
  ASSERT_TRUE(restartTerm.active);
  EXPECT_TRUE(restartTerm.restarted);
  EXPECT_EQ(restartTerm.order, 1);
  EXPECT_NEAR(restartTerm.scalarWeight, 300.0, 1e-12);
  EXPECT_NEAR(restartTerm.targetPosition[0], 0.2, 1e-14);

  Eigen::Matrix<double, 1, 1> acceptedPosition;
  acceptedPosition << 0.25;
  const auto restartVelocity
      = nb::makeBdf2VelocityUpdate(history, acceptedPosition, timeStep);
  ASSERT_TRUE(restartVelocity.active);
  EXPECT_TRUE(restartVelocity.restarted);
  EXPECT_EQ(restartVelocity.order, 1);
  EXPECT_NEAR(restartVelocity.velocity[0], 2.5, 1e-14);

  const auto advanced = nb::advanceBdf2History(
      history, acceptedPosition, restartVelocity.velocity);
  EXPECT_TRUE(advanced.hasPreviousPosition);
  EXPECT_NEAR(advanced.previousPosition[0], 0.0, 1e-14);
  EXPECT_NEAR(advanced.currentPosition[0], 0.25, 1e-14);

  const auto serialized = advanced;
  const auto bdf2Term
      = nb::makeBdf2InertialTerm(serialized, /*mass=*/3.0, timeStep);
  ASSERT_TRUE(bdf2Term.active);
  EXPECT_FALSE(bdf2Term.restarted);
  EXPECT_EQ(bdf2Term.order, 2);
  EXPECT_NEAR(bdf2Term.scalarWeight, 675.0, 1e-12);
  EXPECT_NEAR(bdf2Term.targetPosition[0], 1.0 / 3.0, 1e-14);

  Eigen::Matrix<double, 1, 1> offsetCandidate;
  offsetCandidate << bdf2Term.targetPosition[0] + 0.2;
  EXPECT_NEAR(
      nb::evaluateInertialEnergy(bdf2Term, offsetCandidate),
      0.5 * 675.0 * 0.04,
      1e-12);
}

//==============================================================================
TEST(NewtonBarrierPrimitives, Bdf2VelocityImprovesQuadraticSample)
{
  constexpr double timeStep = 0.01;
  nb::Bdf2StepHistory<1> history;
  history.hasPreviousPosition = true;
  history.previousPosition << 0.0;
  history.currentPosition << timeStep * timeStep;
  history.currentVelocity << 2.0 * timeStep;

  Eigen::Matrix<double, 1, 1> acceptedPosition;
  acceptedPosition << 4.0 * timeStep * timeStep;

  const auto bdf2
      = nb::makeBdf2VelocityUpdate(history, acceptedPosition, timeStep);
  ASSERT_TRUE(bdf2.active);
  EXPECT_EQ(bdf2.order, 2);

  history.hasPreviousPosition = false;
  const auto backwardEuler
      = nb::makeBdf2VelocityUpdate(history, acceptedPosition, timeStep);
  ASSERT_TRUE(backwardEuler.active);
  EXPECT_EQ(backwardEuler.order, 1);

  const double exactVelocityAtAcceptedTime = 4.0 * timeStep;
  EXPECT_NEAR(bdf2.velocity[0], exactVelocityAtAcceptedTime, 1e-14);
  EXPECT_LT(
      std::abs(bdf2.velocity[0] - exactVelocityAtAcceptedTime),
      std::abs(backwardEuler.velocity[0] - exactVelocityAtAcceptedTime));
}

//==============================================================================
TEST(NewtonBarrierPrimitives, FallingBoxEnergyDiagnosticSweepsAreMonotone)
{
  nb::FallingBoxEnergyDiagnosticOptions base;
  base.height = 0.25;
  base.velocity = -1.5;
  base.clearance = 0.004;
  base.activationDistance = 0.01;
  base.barrierStiffness = 2.0;
  base.compression = 0.002;

  for (const double timeStep : {0.1, 0.01, 0.001, 0.0001}) {
    auto options = base;
    options.timeStep = timeStep;
    const auto sample = nb::makeFallingBoxEnergyDiagnostic(options);
    EXPECT_TRUE(sample.active);
    EXPECT_TRUE(sample.barrierActive);
    EXPECT_TRUE(std::isfinite(sample.totalEnergy));
  }

  auto soft = base;
  soft.youngModulus = 1e5;
  auto stiff = base;
  stiff.youngModulus = 1e8;
  EXPECT_LT(
      nb::makeFallingBoxEnergyDiagnostic(soft).elasticEnergy,
      nb::makeFallingBoxEnergyDiagnostic(stiff).elasticEnergy);

  auto lowKappa = base;
  lowKappa.barrierStiffness = 1.0;
  auto highKappa = base;
  highKappa.barrierStiffness = 10.0;
  EXPECT_LT(
      nb::makeFallingBoxEnergyDiagnostic(lowKappa).barrierEnergy,
      nb::makeFallingBoxEnergyDiagnostic(highKappa).barrierEnergy);

  auto narrowActivation = base;
  narrowActivation.activationDistance = 0.006;
  auto wideActivation = base;
  wideActivation.activationDistance = 0.02;
  EXPECT_LT(
      nb::makeFallingBoxEnergyDiagnostic(narrowActivation).barrierEnergy,
      nb::makeFallingBoxEnergyDiagnostic(wideActivation).barrierEnergy);

  auto lowGravity = base;
  lowGravity.gravity = 1.0;
  auto highGravity = base;
  highGravity.gravity = 8.0;
  EXPECT_LT(
      nb::makeFallingBoxEnergyDiagnostic(lowGravity).gravitationalEnergy,
      nb::makeFallingBoxEnergyDiagnostic(highGravity).gravitationalEnergy);

  base.timeStep = -0.01;
  EXPECT_FALSE(nb::makeFallingBoxEnergyDiagnostic(base).active);
}

//==============================================================================
TEST(NewtonBarrierPrimitives, BarrierForceCurveCapturesKappaSensitivity)
{
  constexpr double activationDistance = 0.01;
  constexpr double squaredActivationDistance
      = activationDistance * activationDistance;

  const auto normalForceMagnitude = [](const double clearance,
                                       const double stiffness) {
    const auto barrier = nb::c2ClampedLogBarrier(
        clearance * clearance, squaredActivationDistance);
    if (!barrier.active) {
      return 0.0;
    }
    return std::abs(stiffness * barrier.firstDerivative * 2.0 * clearance);
  };

  EXPECT_FALSE(
      nb::c2ClampedLogBarrier(
          squaredActivationDistance, squaredActivationDistance)
          .active);
  EXPECT_NEAR(
      normalForceMagnitude(activationDistance, /*stiffness=*/1.0), 0.0, 1e-14);

  const double farForce
      = normalForceMagnitude(/*clearance=*/0.008, /*stiffness=*/1.0);
  const double middleForce
      = normalForceMagnitude(/*clearance=*/0.004, /*stiffness=*/1.0);
  const double nearForce
      = normalForceMagnitude(/*clearance=*/0.002, /*stiffness=*/1.0);
  EXPECT_GT(farForce, 0.0);
  EXPECT_LT(farForce, middleForce);
  EXPECT_LT(middleForce, nearForce);

  const double lowKappaForce
      = normalForceMagnitude(/*clearance=*/0.004, /*stiffness=*/0.1);
  const double highKappaForce
      = normalForceMagnitude(/*clearance=*/0.004, /*stiffness=*/10.0);
  EXPECT_NEAR(lowKappaForce, 0.1 * middleForce, 1e-12);
  EXPECT_NEAR(highKappaForce, 10.0 * middleForce, 1e-12);

  const double nearSlope
      = (normalForceMagnitude(/*clearance=*/0.0015, /*stiffness=*/1.0)
         - normalForceMagnitude(/*clearance=*/0.0025, /*stiffness=*/1.0))
        / 0.001;
  const double farSlope
      = (normalForceMagnitude(/*clearance=*/0.006, /*stiffness=*/1.0)
         - normalForceMagnitude(/*clearance=*/0.007, /*stiffness=*/1.0))
        / 0.001;
  EXPECT_TRUE(std::isfinite(nearSlope));
  EXPECT_TRUE(std::isfinite(farSlope));
  EXPECT_GT(nearSlope, 2.0 * farSlope);
}

//==============================================================================
TEST(NewtonBarrierPrimitives, RayleighDampingProjectsHessianAndDissipates)
{
  Eigen::Vector2d displacement;
  displacement << 0.2, -0.4;
  Eigen::Matrix2d indefinite;
  indefinite << 2.0, 0.5, 0.5, -1.0;

  const auto contactDamping = nb::makeSemiImplicitRayleighDampingTerm<2>(
      displacement, indefinite, /*coefficient=*/0.25, /*timeStep=*/0.01);
  ASSERT_TRUE(contactDamping.active);
  EXPECT_NEAR(contactDamping.scale, 25.0, 1e-14);
  expectSelfAdjointPsd(contactDamping.hessian);
  EXPECT_GE(contactDamping.energy, 0.0);
  EXPECT_LE(contactDamping.generalizedForce.dot(displacement), 1e-14);
  expectMatrixNear(
      contactDamping.gradient, -contactDamping.generalizedForce, 1e-14);

  Eigen::Matrix<double, 1, 1> hingeDisplacement;
  hingeDisplacement << 0.15;
  Eigen::Matrix<double, 1, 1> hingeHessian;
  hingeHessian << 12.0;

  const auto lowHingeDamping = nb::makeSemiImplicitRayleighDampingTerm<1>(
      hingeDisplacement,
      hingeHessian,
      /*coefficient=*/0.05,
      /*timeStep=*/0.01);
  const auto highHingeDamping = nb::makeSemiImplicitRayleighDampingTerm<1>(
      hingeDisplacement,
      hingeHessian,
      /*coefficient=*/0.5,
      /*timeStep=*/0.01);
  ASSERT_TRUE(lowHingeDamping.active);
  ASSERT_TRUE(highHingeDamping.active);
  EXPECT_LT(lowHingeDamping.energy, highHingeDamping.energy);
  EXPECT_LT(highHingeDamping.generalizedForce[0], 0.0);

  EXPECT_FALSE(
      nb::makeSemiImplicitRayleighDampingTerm<2>(
          displacement, indefinite, /*coefficient=*/0.0, /*timeStep=*/0.01)
          .active);
}

//==============================================================================
TEST(NewtonBarrierPrimitives, MixedDomainOracleOwnersPreserveVariantRows)
{
  EXPECT_EQ(
      nb::mixedDomainOracleOwner(
          nb::MixedDomainType::Rigid, nb::MixedDomainType::Rigid),
      nb::MixedDomainOracleOwner::RigidIpc);
  EXPECT_EQ(
      nb::mixedDomainOracleOwner(
          nb::MixedDomainType::Deformable, nb::MixedDomainType::Deformable),
      nb::MixedDomainOracleOwner::DeformableIpc);
  EXPECT_EQ(
      nb::mixedDomainOracleOwner(
          nb::MixedDomainType::Affine, nb::MixedDomainType::Affine),
      nb::MixedDomainOracleOwner::AffineBodyDynamics);
  EXPECT_EQ(
      nb::mixedDomainOracleOwner(
          nb::MixedDomainType::Rod, nb::MixedDomainType::Rod),
      nb::MixedDomainOracleOwner::DeformableIpc);
  EXPECT_EQ(
      nb::mixedDomainOracleOwner(
          nb::MixedDomainType::Shell, nb::MixedDomainType::Codimensional),
      nb::MixedDomainOracleOwner::MixedNewtonBarrier);
  EXPECT_EQ(
      nb::mixedDomainOracleOwner(
          nb::MixedDomainType::Rigid, nb::MixedDomainType::Affine),
      nb::MixedDomainOracleOwner::MixedNewtonBarrier);
}

//==============================================================================
TEST(NewtonBarrierPrimitives, MixedDomainSurfacesCoverAllAdapters)
{
  std::vector<nb::MixedDomainSurface> surfaces;
  const std::vector<nb::MixedDomainType> domains{
      nb::MixedDomainType::Rigid,
      nb::MixedDomainType::Deformable,
      nb::MixedDomainType::Affine,
      nb::MixedDomainType::Particle,
      nb::MixedDomainType::Rod,
      nb::MixedDomainType::Shell,
      nb::MixedDomainType::Codimensional};
  for (std::size_t i = 0; i < domains.size(); ++i) {
    surfaces.push_back(
        nb::makeMixedDomainSurface(
            domains[i],
            i,
            {Eigen::Vector3d(0.001 * static_cast<double>(i), 0.0, 0.0)}));
  }

  nb::MixedDomainCandidateOptions options;
  options.activationDistance = 0.1;
  options.exactDistanceFilter = false;
  options.includePointEdge = false;
  options.includeEdgeEdge = false;
  options.includePointTriangle = false;
  const auto candidates
      = nb::buildMixedDomainContactCandidates(surfaces, options);

  EXPECT_EQ(candidates.stats.surfaceCount, domains.size());
  EXPECT_EQ(candidates.stats.activeSurfaceCount, domains.size());
  EXPECT_EQ(candidates.stats.dynamicSurfaceCount, domains.size());
  for (std::size_t i = 0; i < domains.size(); ++i) {
    EXPECT_EQ(candidates.stats.domainCounts[i], 1u);
  }
  EXPECT_EQ(candidates.candidates.size(), 21u);

  const auto restartSurfaces = surfaces;
  const auto restartCandidates
      = nb::buildMixedDomainContactCandidates(restartSurfaces, options);
  EXPECT_EQ(
      nb::mixedDomainCandidateKeys(candidates),
      nb::mixedDomainCandidateKeys(restartCandidates));
}

//==============================================================================
TEST(NewtonBarrierPrimitives, MixedDomainCandidatesCoverPrimitiveFamilies)
{
  auto rigid = nb::makeMixedDomainSurface(
      nb::MixedDomainType::Rigid,
      0,
      {Eigen::Vector3d(0.0, 0.0, 0.0),
       Eigen::Vector3d(1.0, 0.0, 0.0),
       Eigen::Vector3d(0.0, 1.0, 0.0)},
      {Eigen::Vector3i(0, 1, 2)});
  rigid.frictionCoefficient = 0.4;

  auto deformable = nb::makeMixedDomainSurface(
      nb::MixedDomainType::Deformable,
      0,
      {Eigen::Vector3d(0.05, 0.05, 0.02),
       Eigen::Vector3d(0.75, 0.05, 0.02),
       Eigen::Vector3d(0.05, 0.75, 0.02)},
      {Eigen::Vector3i(0, 1, 2)});
  deformable.frictionCoefficient = 0.7;

  const std::vector<nb::MixedDomainSurface> surfaces{rigid, deformable};
  nb::MixedDomainCandidateOptions options;
  options.activationDistance = 1.0;
  const auto candidates
      = nb::buildMixedDomainContactCandidates(surfaces, options);

  EXPECT_EQ(candidates.stats.domainCounts[0], 1u);
  EXPECT_EQ(candidates.stats.domainCounts[1], 1u);
  EXPECT_EQ(candidates.stats.broadPhasePairCount, 1u);
  EXPECT_GT(candidates.stats.pointPointCandidateCount, 0u);
  EXPECT_GT(candidates.stats.pointEdgeCandidateCount, 0u);
  EXPECT_GT(candidates.stats.edgeEdgeCandidateCount, 0u);
  EXPECT_GT(candidates.stats.pointTriangleCandidateCount, 0u);

  const auto diagnostics = nb::evaluateMixedDomainBarrierDiagnostics(
      surfaces, candidates, 1.0, 2.0);
  EXPECT_TRUE(diagnostics.finite);
  EXPECT_EQ(diagnostics.candidateCount, candidates.candidates.size());
  EXPECT_GT(diagnostics.activeBarrierCount, 0u);
  EXPECT_GT(diagnostics.value, 0.0);
  EXPECT_NEAR(diagnostics.maxFrictionCoefficient, 0.7, 1e-14);
  EXPECT_TRUE(std::isfinite(diagnostics.minSquaredDistance));
}

//==============================================================================
TEST(NewtonBarrierPrimitives, MixedDomainPointCcdProducesDeterministicRestart)
{
  auto particleA = nb::makeMixedDomainSurface(
      nb::MixedDomainType::Particle, 0, {Eigen::Vector3d(-0.5, 0.0, 0.0)});
  nb::setMixedDomainEndVertices(particleA, {Eigen::Vector3d(0.5, 0.0, 0.0)});

  auto particleB = nb::makeMixedDomainSurface(
      nb::MixedDomainType::Particle, 1, {Eigen::Vector3d(0.5, 0.0, 0.0)});
  nb::setMixedDomainEndVertices(particleB, {Eigen::Vector3d(-0.5, 0.0, 0.0)});

  const std::vector<nb::MixedDomainSurface> surfaces{particleA, particleB};
  nb::MixedDomainCandidateOptions candidateOptions;
  candidateOptions.activationDistance = 2.0;
  candidateOptions.includePointEdge = false;
  candidateOptions.includeEdgeEdge = false;
  candidateOptions.includePointTriangle = false;
  const auto candidates
      = nb::buildMixedDomainContactCandidates(surfaces, candidateOptions);
  ASSERT_EQ(candidates.candidates.size(), 1u);

  nb::LineSearchOptions lineSearch;
  lineSearch.minSeparation = 0.1;
  const auto ccd
      = nb::mixedDomainPointPointLinearCcd(surfaces, candidates, lineSearch);
  EXPECT_TRUE(ccd.limited);
  EXPECT_TRUE(ccd.allowsPositiveStep());
  EXPECT_NEAR(ccd.stepBound, 0.45, 1e-14);
  EXPECT_EQ(ccd.stats.pointPointChecks, 1u);
  EXPECT_EQ(ccd.stats.hits, 1u);

  const auto restartCandidates
      = nb::buildMixedDomainContactCandidates(surfaces, candidateOptions);
  const auto restartCcd = nb::mixedDomainPointPointLinearCcd(
      surfaces, restartCandidates, lineSearch);
  EXPECT_EQ(
      nb::mixedDomainCandidateKeys(candidates),
      nb::mixedDomainCandidateKeys(restartCandidates));
  EXPECT_DOUBLE_EQ(ccd.stepBound, restartCcd.stepBound);
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
