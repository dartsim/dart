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

#include <dart/simulation/detail/affine_body_dynamics.hpp>
#include <dart/simulation/detail/rigid_ipc/rigid_ipc_barrier.hpp>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <functional>

namespace detail = dart::simulation::detail;

namespace {

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
template <int Size>
Eigen::Matrix<double, Size, 1> finiteGradient(
    const Eigen::Matrix<double, Size, 1>& x,
    const std::function<double(const Eigen::Matrix<double, Size, 1>&)>& value,
    const double step = 1e-6)
{
  Eigen::Matrix<double, Size, 1> gradient
      = Eigen::Matrix<double, Size, 1>::Zero();
  for (int i = 0; i < x.size(); ++i) {
    Eigen::Matrix<double, Size, 1> plus = x;
    Eigen::Matrix<double, Size, 1> minus = x;
    plus[i] += step;
    minus[i] -= step;
    gradient[i] = (value(plus) - value(minus)) / (2.0 * step);
  }
  return gradient;
}

//==============================================================================
template <int Size>
Eigen::Matrix<double, Size, Size> finiteHessian(
    const Eigen::Matrix<double, Size, 1>& x,
    const std::function<double(const Eigen::Matrix<double, Size, 1>&)>& value,
    const double step = 1e-4)
{
  Eigen::Matrix<double, Size, Size> hessian
      = Eigen::Matrix<double, Size, Size>::Zero();
  const double center = value(x);
  for (int row = 0; row < x.size(); ++row) {
    for (int col = row; col < x.size(); ++col) {
      double entry = 0.0;
      if (row == col) {
        Eigen::Matrix<double, Size, 1> plus = x;
        Eigen::Matrix<double, Size, 1> minus = x;
        plus[row] += step;
        minus[row] -= step;
        entry = (value(plus) - 2.0 * center + value(minus)) / (step * step);
      } else {
        Eigen::Matrix<double, Size, 1> pp = x;
        Eigen::Matrix<double, Size, 1> pm = x;
        Eigen::Matrix<double, Size, 1> mp = x;
        Eigen::Matrix<double, Size, 1> mm = x;
        pp[row] += step;
        pp[col] += step;
        pm[row] += step;
        pm[col] -= step;
        mp[row] -= step;
        mp[col] += step;
        mm[row] -= step;
        mm[col] -= step;
        entry = (value(pp) - value(pm) - value(mp) + value(mm))
                / (4.0 * step * step);
      }
      hessian(row, col) = entry;
      hessian(col, row) = entry;
    }
  }
  return hessian;
}

//==============================================================================
Eigen::Matrix<double, 3, 12> finitePointJacobian(
    const detail::AffineVector12d& x,
    const Eigen::Vector3d& localPoint,
    const double step = 1e-6)
{
  Eigen::Matrix<double, 3, 12> jacobian = Eigen::Matrix<double, 3, 12>::Zero();
  for (int col = 0; col < x.size(); ++col) {
    detail::AffineVector12d plus = x;
    detail::AffineVector12d minus = x;
    plus[col] += step;
    minus[col] -= step;
    jacobian.col(col)
        = (detail::affineWorldPoint(
               detail::affineBodyStateFromVector(plus), localPoint)
           - detail::affineWorldPoint(
               detail::affineBodyStateFromVector(minus), localPoint))
          / (2.0 * step);
  }
  return jacobian;
}

//==============================================================================
detail::AffineVector24d packStates(
    const detail::AffineBodyState& bodyA, const detail::AffineBodyState& bodyB)
{
  detail::AffineVector24d vector;
  vector.head<12>() = detail::affineBodyStateToVector(bodyA);
  vector.tail<12>() = detail::affineBodyStateToVector(bodyB);
  return vector;
}

//==============================================================================
detail::AffineBodyState bodyAFromVector(const detail::AffineVector24d& vector)
{
  return detail::affineBodyStateFromVector(vector.head<12>());
}

//==============================================================================
detail::AffineBodyState bodyBFromVector(const detail::AffineVector24d& vector)
{
  return detail::affineBodyStateFromVector(vector.tail<12>());
}

//==============================================================================
detail::AffineBodyState makeDerivativeBody(const Eigen::Vector3d& translation)
{
  detail::AffineBodyState state;
  state.translation = translation;
  state.linearMap << 1.02, 0.03, -0.01, -0.02, 0.97, 0.04, 0.01, -0.03, 1.05;
  return state;
}

//==============================================================================
Eigen::Matrix3d skew(const Eigen::Vector3d& value)
{
  Eigen::Matrix3d matrix;
  matrix << 0.0, -value.z(), value.y(), value.z(), 0.0, -value.x(), -value.y(),
      value.x(), 0.0;
  return matrix;
}

//==============================================================================
Eigen::Matrix<double, 24, 12> affineRigidTangentBasis()
{
  Eigen::Matrix<double, 24, 12> basis = Eigen::Matrix<double, 24, 12>::Zero();
  for (int body = 0; body < 2; ++body) {
    const int affineOffset = 12 * body;
    const int rigidOffset = 6 * body;
    basis.block<3, 3>(affineOffset, rigidOffset) = Eigen::Matrix3d::Identity();
    for (int axis = 0; axis < 3; ++axis) {
      const Eigen::Matrix3d variation = skew(Eigen::Vector3d::Unit(axis));
      for (int col = 0; col < 3; ++col) {
        basis.block<3, 1>(affineOffset + 3 + 3 * col, rigidOffset + 3 + axis)
            = variation.col(col);
      }
    }
  }
  return basis;
}

//==============================================================================
Eigen::Matrix3d rigidRotationSecondDerivativeAtIdentity(
    const int axisA, const int axisB)
{
  const Eigen::Matrix3d generatorA = skew(Eigen::Vector3d::Unit(axisA));
  const Eigen::Matrix3d generatorB = skew(Eigen::Vector3d::Unit(axisB));
  if (axisA == axisB) {
    return generatorA * generatorA;
  }

  return 0.5 * (generatorA * generatorB + generatorB * generatorA);
}

//==============================================================================
Eigen::Matrix<double, 12, 12> rigidRotationCurvatureCorrectionAtIdentity(
    const detail::AffineVector24d& affineGradient)
{
  Eigen::Matrix<double, 12, 12> correction
      = Eigen::Matrix<double, 12, 12>::Zero();

  for (int body = 0; body < 2; ++body) {
    Eigen::Matrix3d linearMapGradient = Eigen::Matrix3d::Zero();
    for (int col = 0; col < 3; ++col) {
      linearMapGradient.col(col)
          = affineGradient.segment<3>(12 * body + 3 + 3 * col);
    }

    for (int axisA = 0; axisA < 3; ++axisA) {
      for (int axisB = 0; axisB < 3; ++axisB) {
        correction(6 * body + 3 + axisA, 6 * body + 3 + axisB)
            = linearMapGradient
                  .cwiseProduct(
                      rigidRotationSecondDerivativeAtIdentity(axisA, axisB))
                  .sum();
      }
    }
  }

  return 0.5
         * (correction + Eigen::Matrix<double, 12, 12>(correction.transpose()));
}

//==============================================================================
detail::AffineFrictionOptions makeAffineFrictionOptions()
{
  detail::AffineFrictionOptions options;
  options.coefficient = 0.7;
  options.laggedNormalForce = 2.5;
  options.staticFrictionDisplacement = 0.2;
  options.projectHessianToPsd = false;
  return options;
}

//==============================================================================
detail::RigidIpcFrictionOptions makeRigidFrictionOptions(
    const detail::AffineFrictionOptions& affineOptions)
{
  detail::RigidIpcFrictionOptions rigidOptions;
  rigidOptions.coefficient = affineOptions.coefficient;
  rigidOptions.laggedNormalForce = affineOptions.laggedNormalForce;
  rigidOptions.staticFrictionDisplacement
      = affineOptions.staticFrictionDisplacement;
  rigidOptions.projectReducedHessianToPsd = false;
  return rigidOptions;
}

//==============================================================================
detail::AffinePointTriangleMicroSolveOptions makeAffineMicroSolveOptions()
{
  detail::AffinePointTriangleMicroSolveOptions options;
  options.barrier.squaredActivationDistance = 0.25;
  options.barrier.stiffness = 0.04;
  options.barrier.projectHessianToPsd = true;
  options.inertialWeight = 1.0;
  options.orthogonalityStiffness = 0.5;
  options.gradientTolerance = 1e-8;
  options.maxIterations = 32;
  options.maxLineSearchIterations = 24;
  options.maxStepNorm = 0.2;
  return options;
}

//==============================================================================
void expectRigidFrictionOracleMatches(
    const detail::AffinePrimitiveFrictionResult& affine,
    const detail::RigidIpcReducedFrictionResult& rigid)
{
  ASSERT_TRUE(affine.active);
  ASSERT_TRUE(rigid.active);
  EXPECT_NEAR(affine.value, rigid.value, 1e-14);
  EXPECT_NEAR(affine.work, rigid.potential.work, 1e-14);
  expectVectorNear(
      affine.tangentialDisplacement,
      rigid.potential.tangentialDisplacement,
      1e-14);

  const Eigen::Matrix<double, 24, 12> tangentBasis = affineRigidTangentBasis();
  expectVectorNear(
      tangentBasis.transpose() * affine.gradient, rigid.gradient, 2e-5);

  const Eigen::Matrix<double, 12, 12> correctedAffineHessian
      = tangentBasis.transpose() * affine.hessian * tangentBasis
        + rigidRotationCurvatureCorrectionAtIdentity(affine.gradient);
  expectMatrixNear(correctedAffineHessian, rigid.hessian, 5e-3);
}

//==============================================================================
double pointTriangleSquaredDistance(
    const detail::AffineBodyState& pointBody,
    const Eigen::Vector3d& point,
    const detail::AffineBodyState& triangleBody,
    const Eigen::Vector3d& triangleA,
    const Eigen::Vector3d& triangleB,
    const Eigen::Vector3d& triangleC,
    const detail::AffineBarrierOptions& options)
{
  return detail::affinePointTriangleBarrier(
             pointBody,
             point,
             triangleBody,
             triangleA,
             triangleB,
             triangleC,
             options)
      .primitive.squaredDistance;
}

} // namespace

//==============================================================================
TEST(AffineBodyDynamics, SurfaceAdapterMapsVerticesAndJacobians)
{
  detail::AffineBodyState state = makeDerivativeBody({0.2, -0.1, 0.35});
  const Eigen::Vector3d localPoint(0.25, -0.2, 0.4);
  const detail::AffineVector12d x = detail::affineBodyStateToVector(state);

  EXPECT_EQ(
      detail::affineBodyStateFromVector(x).translation, state.translation);
  expectMatrixNear(
      detail::affineWorldPoint(state, localPoint),
      state.translation + state.linearMap * localPoint,
      1e-15);
  expectMatrixNear(
      detail::affinePointJacobian(localPoint),
      finitePointJacobian(x, localPoint),
      1e-10);

  detail::AffineSurfaceAdapter surface;
  surface.restVertices
      = {Eigen::Vector3d::Zero(), localPoint, Eigen::Vector3d::UnitX()};
  surface.triangles = {Eigen::Vector3i(0, 1, 2)};

  expectMatrixNear(
      detail::affineSurfaceVertexWorld(surface, state, 1),
      detail::affineWorldPoint(state, localPoint),
      1e-15);
  expectMatrixNear(
      detail::affineSurfaceVertexJacobian(surface, 1),
      detail::affinePointJacobian(localPoint),
      1e-15);
}

//==============================================================================
TEST(AffineBodyDynamics, PointTriangleBarrierDerivativesMatchFiniteDifferences)
{
  detail::AffineBarrierOptions options;
  options.squaredActivationDistance = 1.0;
  options.stiffness = 1.7;
  options.projectHessianToPsd = false;

  const detail::AffineBodyState pointBody
      = makeDerivativeBody({0.02, -0.01, 0.45});
  const detail::AffineBodyState triangleBody
      = makeDerivativeBody({0.0, 0.0, 0.0});
  const Eigen::Vector3d point(0.24, 0.19, 0.0);
  const Eigen::Vector3d triangleA(0.0, 0.0, 0.0);
  const Eigen::Vector3d triangleB(1.0, 0.0, 0.0);
  const Eigen::Vector3d triangleC(0.0, 1.0, 0.0);
  const detail::AffineVector24d x = packStates(pointBody, triangleBody);

  const auto value = [&](const detail::AffineVector24d& y) {
    return detail::affinePointTriangleBarrier(
               bodyAFromVector(y),
               point,
               bodyBFromVector(y),
               triangleA,
               triangleB,
               triangleC,
               options)
        .value;
  };
  const auto actual = detail::affinePointTriangleBarrier(
      pointBody, point, triangleBody, triangleA, triangleB, triangleC, options);

  EXPECT_TRUE(actual.active);
  EXPECT_EQ(
      actual.primitiveType, detail::AffineBarrierPrimitive::PointTriangle);
  EXPECT_NEAR(actual.value, actual.primitive.value, 1e-14);
  expectVectorNear(actual.gradient, finiteGradient<24>(x, value), 3e-5);
  expectMatrixNear(actual.hessian, finiteHessian<24>(x, value), 6e-3);
}

//==============================================================================
TEST(AffineBodyDynamics, PointEdgeBarrierDerivativesMatchFiniteDifferences)
{
  detail::AffineBarrierOptions options;
  options.squaredActivationDistance = 1.0;
  options.stiffness = 1.9;

  const detail::AffineBodyState pointBody
      = makeDerivativeBody({0.0, 0.0, 0.38});
  const detail::AffineBodyState edgeBody = makeDerivativeBody({0.0, 0.0, 0.0});
  const Eigen::Vector3d point(0.2, 0.2, 0.0);
  const Eigen::Vector3d edgeA(-0.5, 0.0, 0.0);
  const Eigen::Vector3d edgeB(0.6, 0.0, 0.0);
  const detail::AffineVector24d x = packStates(pointBody, edgeBody);

  const auto value = [&](const detail::AffineVector24d& y) {
    return detail::affinePointEdgeBarrier(
               bodyAFromVector(y),
               point,
               bodyBFromVector(y),
               edgeA,
               edgeB,
               options)
        .value;
  };
  const auto actual = detail::affinePointEdgeBarrier(
      pointBody, point, edgeBody, edgeA, edgeB, options);

  EXPECT_TRUE(actual.active);
  EXPECT_EQ(actual.primitiveType, detail::AffineBarrierPrimitive::PointEdge);
  EXPECT_NEAR(actual.value, actual.primitive.value, 1e-14);
  expectVectorNear(actual.gradient, finiteGradient<24>(x, value), 3e-5);
  expectMatrixNear(actual.hessian, finiteHessian<24>(x, value), 6e-3);
}

//==============================================================================
TEST(AffineBodyDynamics, EdgeEdgeBarrierDerivativesMatchFiniteDifferences)
{
  detail::AffineBarrierOptions options;
  options.squaredActivationDistance = 1.0;
  options.stiffness = 2.1;

  const detail::AffineBodyState edgeABody
      = makeDerivativeBody({0.0, 0.0, 0.32});
  const detail::AffineBodyState edgeBBody = makeDerivativeBody({0.0, 0.0, 0.0});
  const Eigen::Vector3d edgeA0(-0.5, 0.0, 0.0);
  const Eigen::Vector3d edgeA1(0.5, 0.0, 0.0);
  const Eigen::Vector3d edgeB0(0.0, -0.5, 0.0);
  const Eigen::Vector3d edgeB1(0.0, 0.5, 0.0);
  const detail::AffineVector24d x = packStates(edgeABody, edgeBBody);

  const auto value = [&](const detail::AffineVector24d& y) {
    return detail::affineEdgeEdgeBarrier(
               bodyAFromVector(y),
               edgeA0,
               edgeA1,
               bodyBFromVector(y),
               edgeB0,
               edgeB1,
               options)
        .value;
  };
  const auto actual = detail::affineEdgeEdgeBarrier(
      edgeABody, edgeA0, edgeA1, edgeBBody, edgeB0, edgeB1, options);

  EXPECT_TRUE(actual.active);
  EXPECT_EQ(actual.primitiveType, detail::AffineBarrierPrimitive::EdgeEdge);
  EXPECT_NEAR(actual.value, actual.primitive.value, 1e-14);
  expectVectorNear(actual.gradient, finiteGradient<24>(x, value), 3e-5);
  expectMatrixNear(actual.hessian, finiteHessian<24>(x, value), 8e-3);
}

//==============================================================================
TEST(AffineBodyDynamics, PointPointBarrierDerivativesMatchFiniteDifferences)
{
  detail::AffineBarrierOptions options;
  options.squaredActivationDistance = 1.0;
  options.stiffness = 1.3;

  const detail::AffineBodyState pointABody
      = makeDerivativeBody({0.0, 0.0, 0.3});
  const detail::AffineBodyState pointBBody
      = makeDerivativeBody({0.0, 0.0, 0.0});
  const Eigen::Vector3d pointA(0.2, 0.0, 0.0);
  const Eigen::Vector3d pointB(-0.1, 0.05, 0.0);
  const detail::AffineVector24d x = packStates(pointABody, pointBBody);

  const auto value = [&](const detail::AffineVector24d& y) {
    return detail::affinePointPointBarrier(
               bodyAFromVector(y), pointA, bodyBFromVector(y), pointB, options)
        .value;
  };
  const auto actual = detail::affinePointPointBarrier(
      pointABody, pointA, pointBBody, pointB, options);

  EXPECT_TRUE(actual.active);
  EXPECT_EQ(actual.primitiveType, detail::AffineBarrierPrimitive::PointPoint);
  EXPECT_NEAR(actual.value, actual.primitive.value, 1e-14);
  expectVectorNear(actual.gradient, finiteGradient<24>(x, value), 3e-5);
  expectMatrixNear(actual.hessian, finiteHessian<24>(x, value), 6e-3);
}

//==============================================================================
TEST(AffineBodyDynamics, PointPointFrictionDerivativesMatchFiniteDifferences)
{
  detail::AffineFrictionOptions options = makeAffineFrictionOptions();

  const detail::AffineBodyState laggedPointABody
      = makeDerivativeBody({0.0, 0.0, 0.38});
  detail::AffineBodyState pointABody = laggedPointABody;
  pointABody.translation += Eigen::Vector3d(0.03, -0.02, 0.01);
  pointABody.linearMap(0, 1) += 0.02;

  const detail::AffineBodyState laggedPointBBody
      = makeDerivativeBody({0.02, -0.01, 0.0});
  detail::AffineBodyState pointBBody = laggedPointBBody;
  pointBBody.translation += Eigen::Vector3d(-0.01, 0.04, 0.0);
  pointBBody.linearMap(1, 0) -= 0.015;

  const Eigen::Vector3d pointA(0.2, -0.1, 0.0);
  const Eigen::Vector3d pointB(-0.05, 0.12, 0.0);
  const detail::AffineVector24d x = packStates(pointABody, pointBBody);

  const auto value = [&](const detail::AffineVector24d& y) {
    return detail::affinePointPointFrictionPotential(
               pointA,
               laggedPointABody,
               bodyAFromVector(y),
               pointB,
               laggedPointBBody,
               bodyBFromVector(y),
               options)
        .value;
  };
  const auto actual = detail::affinePointPointFrictionPotential(
      pointA,
      laggedPointABody,
      pointABody,
      pointB,
      laggedPointBBody,
      pointBBody,
      options);

  EXPECT_TRUE(actual.active);
  EXPECT_EQ(actual.primitiveType, detail::AffineBarrierPrimitive::PointPoint);
  EXPECT_GT(actual.tangentialDisplacementNorm, 0.0);
  EXPECT_NEAR(
      actual.weight, options.coefficient * options.laggedNormalForce, 1e-14);
  expectVectorNear(actual.gradient, finiteGradient<24>(x, value), 3e-5);
  expectMatrixNear(actual.hessian, finiteHessian<24>(x, value), 2e-4);
}

//==============================================================================
TEST(AffineBodyDynamics, PointEdgeFrictionDerivativesMatchFiniteDifferences)
{
  detail::AffineFrictionOptions options = makeAffineFrictionOptions();

  const detail::AffineBodyState laggedPointBody
      = makeDerivativeBody({0.0, 0.0, 0.34});
  detail::AffineBodyState pointBody = laggedPointBody;
  pointBody.translation += Eigen::Vector3d(0.025, -0.02, 0.01);
  pointBody.linearMap(0, 2) += 0.018;

  const detail::AffineBodyState laggedEdgeBody
      = makeDerivativeBody({0.02, -0.01, 0.0});
  detail::AffineBodyState edgeBody = laggedEdgeBody;
  edgeBody.translation += Eigen::Vector3d(-0.02, 0.035, 0.0);
  edgeBody.linearMap(1, 0) -= 0.012;

  const Eigen::Vector3d point(0.2, 0.18, 0.0);
  const Eigen::Vector3d edgeA(-0.45, 0.0, 0.0);
  const Eigen::Vector3d edgeB(0.55, 0.05, 0.0);
  const detail::AffineVector24d x = packStates(pointBody, edgeBody);

  const auto value = [&](const detail::AffineVector24d& y) {
    return detail::affinePointEdgeFrictionPotential(
               point,
               laggedPointBody,
               bodyAFromVector(y),
               edgeA,
               edgeB,
               laggedEdgeBody,
               bodyBFromVector(y),
               options)
        .value;
  };
  const auto actual = detail::affinePointEdgeFrictionPotential(
      point,
      laggedPointBody,
      pointBody,
      edgeA,
      edgeB,
      laggedEdgeBody,
      edgeBody,
      options);

  EXPECT_TRUE(actual.active);
  EXPECT_EQ(actual.primitiveType, detail::AffineBarrierPrimitive::PointEdge);
  EXPECT_GT(actual.tangentialDisplacementNorm, 0.0);
  expectVectorNear(actual.gradient, finiteGradient<24>(x, value), 3e-5);
  expectMatrixNear(actual.hessian, finiteHessian<24>(x, value), 3e-4);
}

//==============================================================================
TEST(AffineBodyDynamics, EdgeEdgeFrictionDerivativesMatchFiniteDifferences)
{
  detail::AffineFrictionOptions options = makeAffineFrictionOptions();

  const detail::AffineBodyState laggedEdgeABody
      = makeDerivativeBody({0.0, 0.0, 0.32});
  detail::AffineBodyState edgeABody = laggedEdgeABody;
  edgeABody.translation += Eigen::Vector3d(0.025, -0.015, 0.01);
  edgeABody.linearMap(0, 1) += 0.014;

  const detail::AffineBodyState laggedEdgeBBody
      = makeDerivativeBody({0.02, -0.01, 0.0});
  detail::AffineBodyState edgeBBody = laggedEdgeBBody;
  edgeBBody.translation += Eigen::Vector3d(-0.015, 0.03, 0.0);
  edgeBBody.linearMap(1, 2) -= 0.013;

  const Eigen::Vector3d edgeA0(-0.5, 0.0, 0.0);
  const Eigen::Vector3d edgeA1(0.5, 0.02, 0.0);
  const Eigen::Vector3d edgeB0(0.0, -0.45, 0.0);
  const Eigen::Vector3d edgeB1(0.04, 0.55, 0.0);
  const detail::AffineVector24d x = packStates(edgeABody, edgeBBody);

  const auto value = [&](const detail::AffineVector24d& y) {
    return detail::affineEdgeEdgeFrictionPotential(
               edgeA0,
               edgeA1,
               laggedEdgeABody,
               bodyAFromVector(y),
               edgeB0,
               edgeB1,
               laggedEdgeBBody,
               bodyBFromVector(y),
               options)
        .value;
  };
  const auto actual = detail::affineEdgeEdgeFrictionPotential(
      edgeA0,
      edgeA1,
      laggedEdgeABody,
      edgeABody,
      edgeB0,
      edgeB1,
      laggedEdgeBBody,
      edgeBBody,
      options);

  EXPECT_TRUE(actual.active);
  EXPECT_EQ(actual.primitiveType, detail::AffineBarrierPrimitive::EdgeEdge);
  EXPECT_GT(actual.tangentialDisplacementNorm, 0.0);
  expectVectorNear(actual.gradient, finiteGradient<24>(x, value), 3e-5);
  expectMatrixNear(actual.hessian, finiteHessian<24>(x, value), 3e-4);
}

//==============================================================================
TEST(AffineBodyDynamics, PointTriangleFrictionDerivativesMatchFiniteDifferences)
{
  detail::AffineFrictionOptions options = makeAffineFrictionOptions();

  const detail::AffineBodyState laggedPointBody
      = makeDerivativeBody({0.0, 0.0, 0.43});
  detail::AffineBodyState pointBody = laggedPointBody;
  pointBody.translation += Eigen::Vector3d(0.03, -0.02, 0.01);
  pointBody.linearMap(2, 0) += 0.016;

  const detail::AffineBodyState laggedTriangleBody
      = makeDerivativeBody({0.02, -0.01, 0.0});
  detail::AffineBodyState triangleBody = laggedTriangleBody;
  triangleBody.translation += Eigen::Vector3d(-0.015, 0.035, 0.0);
  triangleBody.linearMap(0, 1) -= 0.011;

  const Eigen::Vector3d point(0.22, 0.18, 0.0);
  const Eigen::Vector3d triangleA(0.0, 0.0, 0.0);
  const Eigen::Vector3d triangleB(1.0, 0.0, 0.0);
  const Eigen::Vector3d triangleC(0.0, 1.0, 0.0);
  const detail::AffineVector24d x = packStates(pointBody, triangleBody);

  const auto value = [&](const detail::AffineVector24d& y) {
    return detail::affinePointTriangleFrictionPotential(
               point,
               laggedPointBody,
               bodyAFromVector(y),
               triangleA,
               triangleB,
               triangleC,
               laggedTriangleBody,
               bodyBFromVector(y),
               options)
        .value;
  };
  const auto actual = detail::affinePointTriangleFrictionPotential(
      point,
      laggedPointBody,
      pointBody,
      triangleA,
      triangleB,
      triangleC,
      laggedTriangleBody,
      triangleBody,
      options);

  EXPECT_TRUE(actual.active);
  EXPECT_EQ(
      actual.primitiveType, detail::AffineBarrierPrimitive::PointTriangle);
  EXPECT_GT(actual.tangentialDisplacementNorm, 0.0);
  expectVectorNear(actual.gradient, finiteGradient<24>(x, value), 3e-5);
  expectMatrixNear(actual.hessian, finiteHessian<24>(x, value), 3e-4);
}

//==============================================================================
TEST(AffineBodyDynamics, PointPointFrictionRigidTangentMatchesRigidIpcOracle)
{
  const detail::AffineFrictionOptions affineOptions
      = makeAffineFrictionOptions();
  const detail::RigidIpcFrictionOptions rigidOptions
      = makeRigidFrictionOptions(affineOptions);

  detail::AffineBodyState laggedPointABody;
  laggedPointABody.translation = {0.0, 0.0, 0.42};
  detail::AffineBodyState pointABody = laggedPointABody;
  pointABody.translation += Eigen::Vector3d(0.03, -0.02, 0.01);

  detail::AffineBodyState laggedPointBBody;
  laggedPointBBody.translation = {0.02, -0.01, 0.0};
  detail::AffineBodyState pointBBody = laggedPointBBody;
  pointBBody.translation += Eigen::Vector3d(-0.01, 0.035, 0.0);

  const Eigen::Vector3d pointA(0.18, -0.08, 0.0);
  const Eigen::Vector3d pointB(-0.04, 0.11, 0.0);

  const auto affine = detail::affinePointPointFrictionPotential(
      pointA,
      laggedPointABody,
      pointABody,
      pointB,
      laggedPointBBody,
      pointBBody,
      affineOptions);
  const auto rigid = detail::rigidIpcPointPointReducedFrictionPotential(
      pointA,
      {laggedPointABody.translation, Eigen::Vector3d::Zero()},
      {pointABody.translation, Eigen::Vector3d::Zero()},
      pointB,
      {laggedPointBBody.translation, Eigen::Vector3d::Zero()},
      {pointBBody.translation, Eigen::Vector3d::Zero()},
      rigidOptions);

  expectRigidFrictionOracleMatches(affine, rigid);
}

//==============================================================================
TEST(AffineBodyDynamics, PointEdgeFrictionRigidTangentMatchesRigidIpcOracle)
{
  const detail::AffineFrictionOptions affineOptions
      = makeAffineFrictionOptions();
  const detail::RigidIpcFrictionOptions rigidOptions
      = makeRigidFrictionOptions(affineOptions);

  detail::AffineBodyState laggedPointBody;
  laggedPointBody.translation = {0.0, 0.0, 0.34};
  detail::AffineBodyState pointBody = laggedPointBody;
  pointBody.translation += Eigen::Vector3d(0.025, -0.02, 0.01);

  detail::AffineBodyState laggedEdgeBody;
  laggedEdgeBody.translation = {0.02, -0.01, 0.0};
  detail::AffineBodyState edgeBody = laggedEdgeBody;
  edgeBody.translation += Eigen::Vector3d(-0.02, 0.035, 0.0);

  const Eigen::Vector3d point(0.2, 0.18, 0.0);
  const Eigen::Vector3d edgeA(-0.45, 0.0, 0.0);
  const Eigen::Vector3d edgeB(0.55, 0.05, 0.0);

  const auto affine = detail::affinePointEdgeFrictionPotential(
      point,
      laggedPointBody,
      pointBody,
      edgeA,
      edgeB,
      laggedEdgeBody,
      edgeBody,
      affineOptions);
  const auto rigid = detail::rigidIpcPointEdgeReducedFrictionPotential(
      point,
      {laggedPointBody.translation, Eigen::Vector3d::Zero()},
      {pointBody.translation, Eigen::Vector3d::Zero()},
      edgeA,
      edgeB,
      {laggedEdgeBody.translation, Eigen::Vector3d::Zero()},
      {edgeBody.translation, Eigen::Vector3d::Zero()},
      rigidOptions);

  expectRigidFrictionOracleMatches(affine, rigid);
}

//==============================================================================
TEST(AffineBodyDynamics, EdgeEdgeFrictionRigidTangentMatchesRigidIpcOracle)
{
  const detail::AffineFrictionOptions affineOptions
      = makeAffineFrictionOptions();
  const detail::RigidIpcFrictionOptions rigidOptions
      = makeRigidFrictionOptions(affineOptions);

  detail::AffineBodyState laggedEdgeABody;
  laggedEdgeABody.translation = {0.0, 0.0, 0.32};
  detail::AffineBodyState edgeABody = laggedEdgeABody;
  edgeABody.translation += Eigen::Vector3d(0.025, -0.015, 0.01);

  detail::AffineBodyState laggedEdgeBBody;
  laggedEdgeBBody.translation = {0.02, -0.01, 0.0};
  detail::AffineBodyState edgeBBody = laggedEdgeBBody;
  edgeBBody.translation += Eigen::Vector3d(-0.015, 0.03, 0.0);

  const Eigen::Vector3d edgeA0(-0.5, 0.0, 0.0);
  const Eigen::Vector3d edgeA1(0.5, 0.02, 0.0);
  const Eigen::Vector3d edgeB0(0.0, -0.45, 0.0);
  const Eigen::Vector3d edgeB1(0.04, 0.55, 0.0);

  const auto affine = detail::affineEdgeEdgeFrictionPotential(
      edgeA0,
      edgeA1,
      laggedEdgeABody,
      edgeABody,
      edgeB0,
      edgeB1,
      laggedEdgeBBody,
      edgeBBody,
      affineOptions);
  const auto rigid = detail::rigidIpcEdgeEdgeReducedFrictionPotential(
      edgeA0,
      edgeA1,
      {laggedEdgeABody.translation, Eigen::Vector3d::Zero()},
      {edgeABody.translation, Eigen::Vector3d::Zero()},
      edgeB0,
      edgeB1,
      {laggedEdgeBBody.translation, Eigen::Vector3d::Zero()},
      {edgeBBody.translation, Eigen::Vector3d::Zero()},
      rigidOptions);

  expectRigidFrictionOracleMatches(affine, rigid);
}

//==============================================================================
TEST(AffineBodyDynamics, PointTriangleFrictionRigidTangentMatchesRigidIpcOracle)
{
  const detail::AffineFrictionOptions affineOptions
      = makeAffineFrictionOptions();
  const detail::RigidIpcFrictionOptions rigidOptions
      = makeRigidFrictionOptions(affineOptions);

  detail::AffineBodyState laggedPointBody;
  laggedPointBody.translation = {0.0, 0.0, 0.43};
  detail::AffineBodyState pointBody = laggedPointBody;
  pointBody.translation += Eigen::Vector3d(0.03, -0.02, 0.01);

  detail::AffineBodyState laggedTriangleBody;
  laggedTriangleBody.translation = {0.02, -0.01, 0.0};
  detail::AffineBodyState triangleBody = laggedTriangleBody;
  triangleBody.translation += Eigen::Vector3d(-0.015, 0.035, 0.0);

  const Eigen::Vector3d point(0.22, 0.18, 0.0);
  const Eigen::Vector3d triangleA(0.0, 0.0, 0.0);
  const Eigen::Vector3d triangleB(1.0, 0.0, 0.0);
  const Eigen::Vector3d triangleC(0.0, 1.0, 0.0);

  const auto affine = detail::affinePointTriangleFrictionPotential(
      point,
      laggedPointBody,
      pointBody,
      triangleA,
      triangleB,
      triangleC,
      laggedTriangleBody,
      triangleBody,
      affineOptions);
  const auto rigid = detail::rigidIpcPointTriangleReducedFrictionPotential(
      point,
      {laggedPointBody.translation, Eigen::Vector3d::Zero()},
      {pointBody.translation, Eigen::Vector3d::Zero()},
      triangleA,
      triangleB,
      triangleC,
      {laggedTriangleBody.translation, Eigen::Vector3d::Zero()},
      {triangleBody.translation, Eigen::Vector3d::Zero()},
      rigidOptions);

  expectRigidFrictionOracleMatches(affine, rigid);
}

//==============================================================================
TEST(AffineBodyDynamics, OrthogonalityEnergyDerivativesMatchFiniteDifferences)
{
  constexpr double kStiffness = 2.4;
  detail::AffineBodyState rigidState;
  rigidState.linearMap
      = Eigen::AngleAxisd(0.4, Eigen::Vector3d(0.2, 0.5, 0.8).normalized())
            .toRotationMatrix();

  const auto rigidEnergy
      = detail::affineOrthogonalityEnergy(rigidState, kStiffness);
  EXPECT_TRUE(rigidEnergy.active);
  EXPECT_NEAR(rigidEnergy.value, 0.0, 1e-14);
  EXPECT_LE(rigidEnergy.gradient.norm(), 1e-12);

  detail::AffineBodyState state = rigidState;
  state.linearMap(0, 1) += 0.08;
  state.linearMap(1, 2) -= 0.05;
  state.linearMap(2, 0) += 0.04;
  const detail::AffineVector12d x = detail::affineBodyStateToVector(state);
  const auto value = [&](const detail::AffineVector12d& y) {
    return detail::affineOrthogonalityEnergy(
               detail::affineBodyStateFromVector(y), kStiffness)
        .value;
  };
  const auto actual = detail::affineOrthogonalityEnergy(state, kStiffness);

  EXPECT_TRUE(actual.active);
  EXPECT_GT(actual.value, 0.0);
  expectVectorNear(actual.gradient, finiteGradient<12>(x, value), 2e-6);
  expectMatrixNear(actual.hessian, finiteHessian<12>(x, value), 2e-5);
}

//==============================================================================
TEST(AffineBodyDynamics, RigidTangentDerivativesMatchRigidIpcOracle)
{
  detail::AffineBarrierOptions affineOptions;
  affineOptions.squaredActivationDistance = 1.0;
  affineOptions.stiffness = 1.7;

  detail::RigidIpcBarrierOptions rigidOptions;
  rigidOptions.squaredActivationDistance
      = affineOptions.squaredActivationDistance;
  rigidOptions.stiffness = affineOptions.stiffness;
  rigidOptions.projectReducedHessianToPsd = false;

  detail::AffineBodyState pointBody;
  pointBody.translation = {0.0, 0.0, 0.48};
  detail::AffineBodyState triangleBody;
  triangleBody.translation = {0.02, -0.01, 0.0};

  const Eigen::Vector3d point(0.22, 0.18, 0.0);
  const Eigen::Vector3d triangleA(0.0, 0.0, 0.0);
  const Eigen::Vector3d triangleB(1.0, 0.0, 0.0);
  const Eigen::Vector3d triangleC(0.0, 1.0, 0.0);

  const auto affine = detail::affinePointTriangleBarrier(
      pointBody,
      point,
      triangleBody,
      triangleA,
      triangleB,
      triangleC,
      affineOptions);
  const auto rigid = detail::rigidIpcPointTriangleReducedBarrier(
      point,
      {pointBody.translation, Eigen::Vector3d::Zero()},
      triangleA,
      triangleB,
      triangleC,
      {triangleBody.translation, Eigen::Vector3d::Zero()},
      rigidOptions);

  ASSERT_TRUE(affine.active);
  ASSERT_TRUE(rigid.active);
  EXPECT_NEAR(affine.value, rigid.value, 1e-14);
  const Eigen::Matrix<double, 24, 12> tangentBasis = affineRigidTangentBasis();
  expectVectorNear(
      tangentBasis.transpose() * affine.gradient, rigid.gradient, 2e-5);

  const Eigen::Matrix<double, 12, 12> correctedAffineHessian
      = tangentBasis.transpose() * affine.hessian * tangentBasis
        + rigidRotationCurvatureCorrectionAtIdentity(affine.gradient);
  expectMatrixNear(correctedAffineHessian, rigid.hessian, 5e-3);
}

//==============================================================================
TEST(AffineBodyDynamics, PointTriangleMicroSolveReducesImplicitObjective)
{
  detail::AffineBodyState initialPointBody;
  initialPointBody.translation = {0.0, 0.0, 0.08};
  initialPointBody.linearMap
      = Eigen::AngleAxisd(0.08, Eigen::Vector3d::UnitX()).toRotationMatrix();
  initialPointBody.mass = 3.0;
  initialPointBody.linearVelocity = {0.1, -0.2, 0.3};

  detail::AffineBodyState inertialTarget = initialPointBody;
  inertialTarget.translation = {0.0, 0.0, 0.02};
  inertialTarget.linearMap(0, 1) += 0.04;
  inertialTarget.linearMap(1, 2) -= 0.03;

  detail::AffineBodyState triangleBody;
  triangleBody.dynamic = false;

  const Eigen::Vector3d point(0.2, 0.15, 0.0);
  const Eigen::Vector3d triangleA(0.0, 0.0, 0.0);
  const Eigen::Vector3d triangleB(1.0, 0.0, 0.0);
  const Eigen::Vector3d triangleC(0.0, 1.0, 0.0);
  const auto options = makeAffineMicroSolveOptions();

  const double targetSquaredDistance = pointTriangleSquaredDistance(
      inertialTarget,
      point,
      triangleBody,
      triangleA,
      triangleB,
      triangleC,
      options.barrier);
  const double targetOrthogonality
      = detail::affineOrthogonalityEnergy(
            inertialTarget, options.orthogonalityStiffness)
            .value;

  const auto result = detail::affinePointTriangleMicroSolve(
      initialPointBody,
      inertialTarget,
      point,
      triangleBody,
      triangleA,
      triangleB,
      triangleC,
      options);

  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.converged);
  EXPECT_GT(result.iterations, 0);
  EXPECT_TRUE(result.barrierActive);
  EXPECT_LT(result.finalValue, result.initialValue);
  EXPECT_LT(result.finalGradientNorm, result.initialGradientNorm);
  EXPECT_GT(result.finalSquaredDistance, targetSquaredDistance);
  EXPECT_DOUBLE_EQ(result.state.mass, initialPointBody.mass);
  EXPECT_EQ(result.state.linearVelocity, initialPointBody.linearVelocity);

  const double finalOrthogonality
      = detail::affineOrthogonalityEnergy(
            result.state, options.orthogonalityStiffness)
            .value;
  EXPECT_LT(finalOrthogonality, targetOrthogonality);
}

//==============================================================================
TEST(AffineBodyDynamics, PointTriangleRuntimeStepUpdatesVelocityAndState)
{
  detail::AffineBodyState initialPointBody;
  initialPointBody.translation = {0.0, 0.0, 0.08};
  initialPointBody.linearMap
      = Eigen::AngleAxisd(0.08, Eigen::Vector3d::UnitX()).toRotationMatrix();
  initialPointBody.linearVelocity = {0.05, -0.02, -2.0};
  initialPointBody.affineVelocity(0, 1) = 0.7;
  initialPointBody.affineVelocity(1, 2) = -0.5;
  initialPointBody.mass = 2.0;

  detail::AffineBodyState triangleBody;
  triangleBody.dynamic = false;

  const Eigen::Vector3d point(0.2, 0.15, 0.0);
  const Eigen::Vector3d triangleA(0.0, 0.0, 0.0);
  const Eigen::Vector3d triangleB(1.0, 0.0, 0.0);
  const Eigen::Vector3d triangleC(0.0, 1.0, 0.0);

  detail::AffinePointTriangleRuntimeStepOptions options;
  options.solve = makeAffineMicroSolveOptions();
  options.timeStep = 0.03;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);

  const auto result = detail::affinePointTriangleRuntimeStep(
      initialPointBody,
      point,
      triangleBody,
      triangleA,
      triangleB,
      triangleC,
      options);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.converged);
  EXPECT_TRUE(result.solve.valid);
  EXPECT_TRUE(result.solve.barrierActive);
  EXPECT_LT(result.solve.finalValue, result.solve.initialValue);

  const double targetSquaredDistance = pointTriangleSquaredDistance(
      result.inertialTarget,
      point,
      triangleBody,
      triangleA,
      triangleB,
      triangleC,
      options.solve.barrier);
  EXPECT_GT(result.solve.finalSquaredDistance, targetSquaredDistance);

  const Eigen::Vector3d targetVelocity
      = (result.inertialTarget.translation - initialPointBody.translation)
        / options.timeStep;
  EXPECT_GT(result.solve.state.linearVelocity.z(), targetVelocity.z());
  EXPECT_GT(result.displacementNorm, 0.0);
  EXPECT_GT(result.linearSpeed, 0.0);
  EXPECT_GT(result.affineVelocityNorm, 0.0);
  EXPECT_DOUBLE_EQ(result.solve.state.mass, initialPointBody.mass);
}

//==============================================================================
TEST(AffineBodyDynamics, PointTrianglePairRuntimeStepMovesBothBodies)
{
  detail::AffineBodyState initialPointBody;
  initialPointBody.translation = {0.02, 0.02, 0.09};
  initialPointBody.linearMap
      = Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitY()).toRotationMatrix();
  initialPointBody.linearVelocity = {0.04, 0.0, -1.8};
  initialPointBody.affineVelocity(0, 1) = 0.35;
  initialPointBody.affineVelocity(1, 2) = -0.25;

  detail::AffineBodyState initialTriangleBody;
  initialTriangleBody.translation = {0.0, 0.0, 0.0};
  initialTriangleBody.linearMap
      = Eigen::AngleAxisd(-0.04, Eigen::Vector3d::UnitX()).toRotationMatrix();
  initialTriangleBody.linearVelocity = {-0.02, 0.0, 0.25};
  initialTriangleBody.affineVelocity(0, 2) = -0.2;
  initialTriangleBody.affineVelocity(2, 1) = 0.15;

  const Eigen::Vector3d point(0.0, 0.0, 0.0);
  const Eigen::Vector3d triangleA(-0.55, -0.45, 0.0);
  const Eigen::Vector3d triangleB(0.55, -0.45, 0.0);
  const Eigen::Vector3d triangleC(-0.55, 0.55, 0.0);

  detail::AffinePointTriangleRuntimeStepOptions options;
  options.solve = makeAffineMicroSolveOptions();
  options.solve.gradientTolerance = 1e-6;
  options.solve.maxIterations = 48;
  options.solve.maxLineSearchIterations = 24;
  options.solve.maxStepNorm = 0.2;
  options.timeStep = 0.03;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);

  const auto result = detail::affinePointTrianglePairRuntimeStep(
      initialPointBody,
      point,
      initialTriangleBody,
      triangleA,
      triangleB,
      triangleC,
      options);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.converged);
  EXPECT_TRUE(result.solve.valid);
  EXPECT_TRUE(result.solve.barrierActive);
  EXPECT_LT(result.solve.finalValue, result.solve.initialValue);

  const double targetSquaredDistance = pointTriangleSquaredDistance(
      result.pointInertialTarget,
      point,
      result.triangleInertialTarget,
      triangleA,
      triangleB,
      triangleC,
      options.solve.barrier);
  EXPECT_GT(result.solve.finalSquaredDistance, targetSquaredDistance);

  EXPECT_GT(result.pointDisplacementNorm, 0.0);
  EXPECT_GT(result.triangleDisplacementNorm, 0.0);
  EXPECT_GT(result.maxLinearSpeed, 0.0);
  EXPECT_GT(result.maxAffineVelocityNorm, 0.0);
  EXPECT_NE(
      result.solve.pointState.linearVelocity.z(),
      result.pointInertialTarget.linearVelocity.z());
  EXPECT_NE(
      result.solve.triangleState.linearVelocity.z(),
      result.triangleInertialTarget.linearVelocity.z());
}

//==============================================================================
TEST(AffineBodyDynamics, PointTrianglePairRuntimeStepKeepsStaticTriangleFixed)
{
  detail::AffineBodyState initialPointBody;
  initialPointBody.translation = {0.02, 0.02, 0.09};
  initialPointBody.linearVelocity = {0.04, 0.0, -1.8};
  initialPointBody.affineVelocity(0, 1) = 0.35;
  initialPointBody.affineVelocity(1, 2) = -0.25;

  detail::AffineBodyState initialTriangleBody;
  initialTriangleBody.translation = {0.0, 0.0, 0.0};
  initialTriangleBody.linearMap
      = Eigen::AngleAxisd(-0.04, Eigen::Vector3d::UnitX()).toRotationMatrix();
  initialTriangleBody.linearVelocity = {-0.02, 0.0, 0.25};
  initialTriangleBody.affineVelocity(0, 2) = -0.2;
  initialTriangleBody.affineVelocity(2, 1) = 0.15;
  initialTriangleBody.dynamic = false;

  const Eigen::Vector3d point(0.0, 0.0, 0.0);
  const Eigen::Vector3d triangleA(-0.55, -0.45, 0.0);
  const Eigen::Vector3d triangleB(0.55, -0.45, 0.0);
  const Eigen::Vector3d triangleC(-0.55, 0.55, 0.0);

  detail::AffinePointTriangleRuntimeStepOptions options;
  options.solve = makeAffineMicroSolveOptions();
  options.solve.gradientTolerance = 1e-6;
  options.solve.maxIterations = 48;
  options.solve.maxLineSearchIterations = 24;
  options.solve.maxStepNorm = 0.2;
  options.timeStep = 0.03;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);

  const auto result = detail::affinePointTrianglePairRuntimeStep(
      initialPointBody,
      point,
      initialTriangleBody,
      triangleA,
      triangleB,
      triangleC,
      options);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.converged);
  EXPECT_TRUE(result.solve.barrierActive);
  EXPECT_GT(result.pointDisplacementNorm, 0.0);
  EXPECT_DOUBLE_EQ(result.triangleDisplacementNorm, 0.0);
  EXPECT_TRUE(result.solve.triangleState.translation.isApprox(
      initialTriangleBody.translation));
  EXPECT_TRUE(result.solve.triangleState.linearMap.isApprox(
      initialTriangleBody.linearMap));
  EXPECT_TRUE(result.solve.triangleState.linearVelocity.isApprox(
      initialTriangleBody.linearVelocity));
  EXPECT_TRUE(result.solve.triangleState.affineVelocity.isApprox(
      initialTriangleBody.affineVelocity));
}
