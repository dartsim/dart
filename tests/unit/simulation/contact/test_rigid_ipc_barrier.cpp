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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS AND
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
#include <dart/simulation/detail/rigid_ipc_barrier.hpp>

#include <dart/common/free_list_allocator.hpp>
#include <dart/common/memory_allocator_debugger.hpp>
#include <dart/common/memory_manager.hpp>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <numbers>
#include <span>
#include <vector>

#include <cmath>

namespace expdetail = dart::simulation::detail;
namespace dc = dart::simulation::detail::deformable_contact;

namespace {

//==============================================================================
void expectPrimitiveBarrierNear(
    const expdetail::RigidIpcPrimitiveBarrierResult& actual,
    const expdetail::RigidIpcPrimitiveBarrierResult& expected,
    const double tolerance)
{
  EXPECT_EQ(actual.active, expected.active);
  EXPECT_NEAR(actual.value, expected.value, tolerance);
  EXPECT_NEAR(actual.squaredDistance, expected.squaredDistance, tolerance);
  EXPECT_NEAR(
      actual.safeSquaredDistance, expected.safeSquaredDistance, tolerance);
  EXPECT_NEAR(
      actual.squaredActivationDistance,
      expected.squaredActivationDistance,
      tolerance);
  EXPECT_LE((actual.gradient - expected.gradient).norm(), tolerance);
  EXPECT_LE((actual.hessian - expected.hessian).norm(), tolerance);
}

//==============================================================================
expdetail::RigidIpcPose poseFromVector(
    const expdetail::RigidIpcVector12d& x, const int body)
{
  return {x.segment<3>(6 * body), x.segment<3>(6 * body + 3)};
}

//==============================================================================
expdetail::RigidIpcVector12d finiteGradient(
    const expdetail::RigidIpcVector12d& x,
    const std::function<double(const expdetail::RigidIpcVector12d&)>& value,
    const double step = 1e-6)
{
  expdetail::RigidIpcVector12d gradient = expdetail::RigidIpcVector12d::Zero();
  for (int i = 0; i < x.size(); ++i) {
    expdetail::RigidIpcVector12d plus = x;
    expdetail::RigidIpcVector12d minus = x;
    plus[i] += step;
    minus[i] -= step;
    gradient[i] = (value(plus) - value(minus)) / (2.0 * step);
  }
  return gradient;
}

//==============================================================================
expdetail::RigidIpcMatrix12d finiteHessian(
    const expdetail::RigidIpcVector12d& x,
    const std::function<double(const expdetail::RigidIpcVector12d&)>& value,
    const double step = 1e-4)
{
  expdetail::RigidIpcMatrix12d hessian = expdetail::RigidIpcMatrix12d::Zero();
  const double center = value(x);
  for (int row = 0; row < x.size(); ++row) {
    for (int col = row; col < x.size(); ++col) {
      double entry = 0.0;
      if (row == col) {
        expdetail::RigidIpcVector12d plus = x;
        expdetail::RigidIpcVector12d minus = x;
        plus[row] += step;
        minus[row] -= step;
        entry = (value(plus) - 2.0 * center + value(minus)) / (step * step);
      } else {
        expdetail::RigidIpcVector12d pp = x;
        expdetail::RigidIpcVector12d pm = x;
        expdetail::RigidIpcVector12d mp = x;
        expdetail::RigidIpcVector12d mm = x;
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
Eigen::Matrix<double, 6, 1> finiteGradient6(
    const Eigen::Matrix<double, 6, 1>& x,
    const std::function<double(const Eigen::Matrix<double, 6, 1>&)>& value,
    const double step = 1e-6)
{
  Eigen::Matrix<double, 6, 1> gradient = Eigen::Matrix<double, 6, 1>::Zero();
  for (int i = 0; i < x.size(); ++i) {
    Eigen::Matrix<double, 6, 1> plus = x;
    Eigen::Matrix<double, 6, 1> minus = x;
    plus[i] += step;
    minus[i] -= step;
    gradient[i] = (value(plus) - value(minus)) / (2.0 * step);
  }
  return gradient;
}

//==============================================================================
Eigen::Matrix<double, 6, 6> finiteHessian6(
    const Eigen::Matrix<double, 6, 1>& x,
    const std::function<double(const Eigen::Matrix<double, 6, 1>&)>& value,
    const double step = 1e-4)
{
  Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Zero();
  const double center = value(x);
  for (int row = 0; row < x.size(); ++row) {
    for (int col = row; col < x.size(); ++col) {
      double entry = 0.0;
      if (row == col) {
        Eigen::Matrix<double, 6, 1> plus = x;
        Eigen::Matrix<double, 6, 1> minus = x;
        plus[row] += step;
        minus[row] -= step;
        entry = (value(plus) - 2.0 * center + value(minus)) / (step * step);
      } else {
        Eigen::Matrix<double, 6, 1> pp = x;
        Eigen::Matrix<double, 6, 1> pm = x;
        Eigen::Matrix<double, 6, 1> mp = x;
        Eigen::Matrix<double, 6, 1> mm = x;
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
template <int Size>
Eigen::Matrix<double, Size, 1> finiteGradientSized(
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
Eigen::Matrix<double, Size, Size> finiteHessianSized(
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
bool containsPrimitive(
    const expdetail::RigidIpcBarrierAssembly& assembly,
    const expdetail::RigidIpcBarrierPrimitive primitive)
{
  return std::any_of(
      assembly.activeConstraints.begin(),
      assembly.activeConstraints.end(),
      [primitive](const expdetail::RigidIpcBarrierConstraint& constraint) {
        return constraint.primitive == primitive;
      });
}

//==============================================================================
expdetail::RigidIpcBarrierSurface makeTriangleSurface(const double z)
{
  expdetail::RigidIpcBarrierSurface surface;
  surface.pose.position = Eigen::Vector3d(0.0, 0.0, z);
  surface.vertices
      = {Eigen::Vector3d(0.0, 0.0, 0.0),
         Eigen::Vector3d(1.0, 0.0, 0.0),
         Eigen::Vector3d(0.0, 1.0, 0.0)};
  surface.triangles = {Eigen::Vector3i(0, 1, 2)};
  return surface;
}

//==============================================================================
expdetail::RigidIpcBarrierAssembly makeAssembly(
    const Eigen::VectorXd& gradient, const Eigen::MatrixXd& hessian)
{
  expdetail::RigidIpcBarrierAssembly assembly;
  assembly.gradient = gradient;
  assembly.hessian = hessian.sparseView();
  assembly.equalityResidual.resize(0);
  assembly.equalityJacobian.resize(0, gradient.size());
  return assembly;
}

} // namespace

//==============================================================================
TEST(RigidIpcBarrier, PointTriangleMatchesWorldSpaceKernelAtInterpolatedPose)
{
  expdetail::RigidIpcBarrierOptions options;
  options.squaredActivationDistance = 1.0;
  options.stiffness = 2.0;

  const Eigen::Vector3d point(0.25, 0.25, 0.0);
  const Eigen::Vector3d triangleA(0.0, 0.0, 0.0);
  const Eigen::Vector3d triangleB(1.0, 0.0, 0.0);
  const Eigen::Vector3d triangleC(0.0, 1.0, 0.0);
  const expdetail::RigidIpcPose pointPoseStart{
      Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose pointPoseEnd{
      Eigen::Vector3d(0.0, 0.0, 0.5), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose trianglePose;
  constexpr double kTime = 0.5;

  const auto actual = expdetail::rigidIpcPointTriangleBarrierAtTime(
      point,
      pointPoseStart,
      pointPoseEnd,
      triangleA,
      triangleB,
      triangleC,
      trianglePose,
      trianglePose,
      kTime,
      options);

  const auto expected = dc::pointTriangleBarrier(
      expdetail::transformRigidIpcPoint(
          point, pointPoseStart, pointPoseEnd, kTime),
      triangleA,
      triangleB,
      triangleC,
      options.squaredActivationDistance,
      options.stiffness);

  EXPECT_TRUE(actual.active);
  EXPECT_GT(actual.value, 0.0);
  EXPECT_NEAR(actual.squaredDistance, 0.75 * 0.75, 1e-15);
  expectPrimitiveBarrierNear(actual, expected, 1e-14);
}

//==============================================================================
TEST(RigidIpcBarrier, PointTriangleIsInactiveOutsideActivationDistance)
{
  expdetail::RigidIpcBarrierOptions options;
  options.squaredActivationDistance = 0.25;
  options.stiffness = 4.0;

  const Eigen::Vector3d point(0.25, 0.25, 0.0);
  const Eigen::Vector3d triangleA(0.0, 0.0, 0.0);
  const Eigen::Vector3d triangleB(1.0, 0.0, 0.0);
  const Eigen::Vector3d triangleC(0.0, 1.0, 0.0);
  const expdetail::RigidIpcPose pointPose{
      Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose trianglePose;

  const auto actual = expdetail::rigidIpcPointTriangleBarrierAtTime(
      point,
      pointPose,
      pointPose,
      triangleA,
      triangleB,
      triangleC,
      trianglePose,
      trianglePose,
      0.0,
      options);

  EXPECT_FALSE(actual.active);
  EXPECT_EQ(actual.value, 0.0);
  EXPECT_NEAR(actual.squaredDistance, 1.0, 1e-15);
}

//==============================================================================
TEST(RigidIpcBarrier, PointEdgeMatchesWorldSpaceKernelAtInterpolatedPose)
{
  expdetail::RigidIpcBarrierOptions options;
  options.squaredActivationDistance = 1.0;
  options.stiffness = 2.5;

  const Eigen::Vector3d point(0.2, 0.15, 0.0);
  const Eigen::Vector3d edgeA(-0.6, 0.0, 0.0);
  const Eigen::Vector3d edgeB(0.6, 0.0, 0.0);
  const expdetail::RigidIpcPose pointPoseStart{
      Eigen::Vector3d(0.0, 0.0, 0.75), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose pointPoseEnd{
      Eigen::Vector3d(0.0, 0.0, 0.25), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose edgePose;
  constexpr double kTime = 0.5;

  const auto actual = expdetail::rigidIpcPointEdgeBarrierAtTime(
      point,
      pointPoseStart,
      pointPoseEnd,
      edgeA,
      edgeB,
      edgePose,
      edgePose,
      kTime,
      options);

  const auto expected = dc::pointEdgeBarrier(
      expdetail::transformRigidIpcPoint(
          point, pointPoseStart, pointPoseEnd, kTime),
      edgeA,
      edgeB,
      options.squaredActivationDistance,
      options.stiffness);

  EXPECT_TRUE(actual.active);
  EXPECT_GT(actual.value, 0.0);
  EXPECT_NEAR(actual.squaredDistance, 0.15 * 0.15 + 0.5 * 0.5, 1e-15);
  expectPrimitiveBarrierNear(actual, expected, 1e-14);
}

//==============================================================================
TEST(RigidIpcBarrier, EdgeEdgeMatchesWorldSpaceKernelAfterRigidRotation)
{
  expdetail::RigidIpcBarrierOptions options;
  options.squaredActivationDistance = 1.0;
  options.stiffness = 3.0;

  const Eigen::Vector3d edgeA0(-1.0, 0.0, 0.0);
  const Eigen::Vector3d edgeA1(1.0, 0.0, 0.0);
  const Eigen::Vector3d edgeB0(-1.0, 0.0, 0.0);
  const Eigen::Vector3d edgeB1(1.0, 0.0, 0.0);
  const expdetail::RigidIpcPose edgeAPose;
  const expdetail::RigidIpcPose edgeBPoseStart{
      Eigen::Vector3d(0.0, 0.0, 0.25), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose edgeBPoseEnd{
      Eigen::Vector3d(0.0, 0.0, 0.25),
      Eigen::Vector3d(0.0, 0.0, 1.57079632679489661923)};
  constexpr double kTime = 1.0;

  const auto actual = expdetail::rigidIpcEdgeEdgeBarrierAtTime(
      edgeA0,
      edgeA1,
      edgeAPose,
      edgeAPose,
      edgeB0,
      edgeB1,
      edgeBPoseStart,
      edgeBPoseEnd,
      kTime,
      options);

  const auto expected = dc::edgeEdgeBarrier(
      edgeA0,
      edgeA1,
      expdetail::transformRigidIpcPoint(
          edgeB0, edgeBPoseStart, edgeBPoseEnd, kTime),
      expdetail::transformRigidIpcPoint(
          edgeB1, edgeBPoseStart, edgeBPoseEnd, kTime),
      options.squaredActivationDistance,
      options.stiffness);

  EXPECT_TRUE(actual.active);
  EXPECT_GT(actual.value, 0.0);
  EXPECT_NEAR(actual.squaredDistance, 0.25 * 0.25, 1e-15);
  expectPrimitiveBarrierNear(actual, expected, 1e-14);
}

//==============================================================================
TEST(RigidIpcBarrier, PointPointMatchesWorldSpaceKernelAtInterpolatedPose)
{
  expdetail::RigidIpcBarrierOptions options;
  options.squaredActivationDistance = 1.0;
  options.stiffness = 1.5;

  const Eigen::Vector3d pointA(0.1, 0.0, 0.0);
  const Eigen::Vector3d pointB(-0.1, 0.0, 0.0);
  const expdetail::RigidIpcPose pointAPoseStart{
      Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose pointAPoseEnd{
      Eigen::Vector3d(0.0, 0.0, 0.2), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose pointBPoseStart{
      Eigen::Vector3d(0.0, 0.0, 0.5), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose pointBPoseEnd{
      Eigen::Vector3d(0.0, 0.0, 0.3), Eigen::Vector3d::Zero()};
  constexpr double kTime = 0.25;

  const auto actual = expdetail::rigidIpcPointPointBarrierAtTime(
      pointA,
      pointAPoseStart,
      pointAPoseEnd,
      pointB,
      pointBPoseStart,
      pointBPoseEnd,
      kTime,
      options);

  const auto expected = dc::pointPointBarrier(
      expdetail::transformRigidIpcPoint(
          pointA, pointAPoseStart, pointAPoseEnd, kTime),
      expdetail::transformRigidIpcPoint(
          pointB, pointBPoseStart, pointBPoseEnd, kTime),
      options.squaredActivationDistance,
      options.stiffness);

  EXPECT_TRUE(actual.active);
  EXPECT_GT(actual.value, 0.0);
  expectPrimitiveBarrierNear(actual, expected, 1e-14);
}

//==============================================================================
TEST(RigidIpcBarrier, PointTriangleReducedDerivativesMatchFiniteDifferences)
{
  expdetail::RigidIpcBarrierOptions options;
  options.squaredActivationDistance = 1.0;
  options.stiffness = 1.7;
  options.projectReducedHessianToPsd = false;

  const Eigen::Vector3d point(0.22, 0.18, 0.0);
  const Eigen::Vector3d triangleA(0.0, 0.0, 0.0);
  const Eigen::Vector3d triangleB(1.0, 0.0, 0.0);
  const Eigen::Vector3d triangleC(0.0, 1.0, 0.0);

  expdetail::RigidIpcVector12d x;
  x << 0.0, 0.0, 0.48, 0.08, -0.04, 0.03, 0.02, -0.01, 0.0, -0.03, 0.06, -0.02;

  const auto value = [&](const expdetail::RigidIpcVector12d& y) {
    return expdetail::rigidIpcPointTriangleReducedBarrier(
               point,
               poseFromVector(y, 0),
               triangleA,
               triangleB,
               triangleC,
               poseFromVector(y, 1),
               options)
        .value;
  };

  const auto actual = expdetail::rigidIpcPointTriangleReducedBarrier(
      point,
      poseFromVector(x, 0),
      triangleA,
      triangleB,
      triangleC,
      poseFromVector(x, 1),
      options);

  EXPECT_TRUE(actual.active);
  EXPECT_GT(actual.value, 0.0);
  expectVectorNear(actual.gradient, finiteGradient(x, value), 2e-5);
  expectMatrixNear(actual.hessian, finiteHessian(x, value), 4e-3);
}

//==============================================================================
TEST(RigidIpcBarrier, PointEdgeReducedDerivativesMatchFiniteDifferences)
{
  expdetail::RigidIpcBarrierOptions options;
  options.squaredActivationDistance = 1.0;
  options.stiffness = 1.9;
  options.projectReducedHessianToPsd = false;

  const Eigen::Vector3d point(0.1, 0.15, 0.0);
  const Eigen::Vector3d edgeA(-0.6, 0.0, 0.0);
  const Eigen::Vector3d edgeB(0.6, 0.0, 0.0);

  expdetail::RigidIpcVector12d x;
  x << 0.0, 0.0, 0.45, 0.05, -0.03, 0.02, 0.01, -0.02, 0.0, -0.02, 0.04, 0.03;

  const auto value = [&](const expdetail::RigidIpcVector12d& y) {
    return expdetail::rigidIpcPointEdgeReducedBarrier(
               point,
               poseFromVector(y, 0),
               edgeA,
               edgeB,
               poseFromVector(y, 1),
               options)
        .value;
  };

  const auto actual = expdetail::rigidIpcPointEdgeReducedBarrier(
      point, poseFromVector(x, 0), edgeA, edgeB, poseFromVector(x, 1), options);

  EXPECT_TRUE(actual.active);
  EXPECT_GT(actual.value, 0.0);
  expectVectorNear(actual.gradient, finiteGradient(x, value), 3e-5);
  expectMatrixNear(actual.hessian, finiteHessian(x, value), 5e-3);
}

//==============================================================================
TEST(RigidIpcBarrier, EdgeEdgeReducedGradientMatchesFiniteDifference)
{
  expdetail::RigidIpcBarrierOptions options;
  options.squaredActivationDistance = 1.0;
  options.stiffness = 2.25;

  const Eigen::Vector3d edgeA0(-0.6, 0.0, 0.0);
  const Eigen::Vector3d edgeA1(0.6, 0.0, 0.0);
  const Eigen::Vector3d edgeB0(0.0, -0.6, 0.0);
  const Eigen::Vector3d edgeB1(0.0, 0.6, 0.0);

  expdetail::RigidIpcVector12d x;
  x << 0.0, 0.0, 0.0, 0.04, -0.03, 0.02, 0.0, 0.0, 0.32, -0.02, 0.05, 0.07;

  const auto value = [&](const expdetail::RigidIpcVector12d& y) {
    return expdetail::rigidIpcEdgeEdgeReducedBarrier(
               edgeA0,
               edgeA1,
               poseFromVector(y, 0),
               edgeB0,
               edgeB1,
               poseFromVector(y, 1),
               options)
        .value;
  };

  const auto actual = expdetail::rigidIpcEdgeEdgeReducedBarrier(
      edgeA0,
      edgeA1,
      poseFromVector(x, 0),
      edgeB0,
      edgeB1,
      poseFromVector(x, 1),
      options);

  EXPECT_TRUE(actual.active);
  EXPECT_GT(actual.value, 0.0);
  expectVectorNear(actual.gradient, finiteGradient(x, value), 3e-5);

  const expdetail::RigidIpcMatrix12d symmetricHessian
      = 0.5
        * (actual.hessian
           + expdetail::RigidIpcMatrix12d(actual.hessian.transpose()));
  EXPECT_LE((actual.hessian - symmetricHessian).norm(), 1e-10);

  Eigen::SelfAdjointEigenSolver<expdetail::RigidIpcMatrix12d> solver(
      symmetricHessian);
  ASSERT_EQ(solver.info(), Eigen::Success);
  EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-10);
}

//==============================================================================
TEST(RigidIpcBarrier, PointPointReducedDerivativesMatchFiniteDifferences)
{
  expdetail::RigidIpcBarrierOptions options;
  options.squaredActivationDistance = 1.0;
  options.stiffness = 1.3;
  options.projectReducedHessianToPsd = false;

  const Eigen::Vector3d pointA(0.2, 0.1, 0.0);
  const Eigen::Vector3d pointB(-0.1, -0.05, 0.0);

  expdetail::RigidIpcVector12d x;
  x << 0.0, 0.0, 0.1, 0.04, -0.03, 0.02, 0.0, 0.0, 0.45, -0.02, 0.05, 0.03;

  const auto value = [&](const expdetail::RigidIpcVector12d& y) {
    return expdetail::rigidIpcPointPointReducedBarrier(
               pointA,
               poseFromVector(y, 0),
               pointB,
               poseFromVector(y, 1),
               options)
        .value;
  };

  const auto actual = expdetail::rigidIpcPointPointReducedBarrier(
      pointA, poseFromVector(x, 0), pointB, poseFromVector(x, 1), options);

  EXPECT_TRUE(actual.active);
  EXPECT_GT(actual.value, 0.0);
  expectVectorNear(actual.gradient, finiteGradient(x, value), 3e-5);
  expectMatrixNear(actual.hessian, finiteHessian(x, value), 5e-3);
}

//==============================================================================
TEST(RigidIpcBarrier, PointPointFrictionPotentialMatchesFiniteDifferences)
{
  expdetail::RigidIpcFrictionOptions options;
  options.coefficient = 0.5;
  options.laggedNormalForce = 4.0;
  options.staticFrictionDisplacement = 0.1;

  const Eigen::Vector3d laggedA = Eigen::Vector3d::Zero();
  const Eigen::Vector3d laggedB = Eigen::Vector3d::UnitZ();

  Eigen::Matrix<double, 6, 1> x;
  x << 0.12, 0.03, 0.0, -0.01, 0.02, 1.0;

  const auto value = [&](const Eigen::Matrix<double, 6, 1>& y) {
    return expdetail::rigidIpcPointPointFrictionPotential(
               laggedA, laggedB, y.head<3>(), y.tail<3>(), options)
        .value;
  };

  const auto actual = expdetail::rigidIpcPointPointFrictionPotential(
      laggedA, laggedB, x.head<3>(), x.tail<3>(), options);

  ASSERT_TRUE(actual.active);
  EXPECT_TRUE(actual.dynamicBranch);
  EXPECT_GT(actual.value, 0.0);
  EXPECT_NEAR(actual.weight, 2.0, 1e-15);
  EXPECT_GT(
      actual.tangentialDisplacementNorm, options.staticFrictionDisplacement);
  expectVectorNear(actual.gradient.head<6>(), finiteGradient6(x, value), 1e-6);
  expectMatrixNear(
      actual.hessian.topLeftCorner<6, 6>(), finiteHessian6(x, value), 2e-5);
}

//==============================================================================
TEST(RigidIpcBarrier, PointPointFrictionPotentialUsesStaticBranchAtLowSlip)
{
  expdetail::RigidIpcFrictionOptions options;
  options.coefficient = 0.25;
  options.laggedNormalForce = 3.0;
  options.staticFrictionDisplacement = 0.2;

  const Eigen::Vector3d laggedA = Eigen::Vector3d::Zero();
  const Eigen::Vector3d laggedB = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d pointA(0.02, 0.01, 0.0);
  const Eigen::Vector3d pointB(0.0, 0.0, 1.0);

  const auto actual = expdetail::rigidIpcPointPointFrictionPotential(
      laggedA, laggedB, pointA, pointB, options);

  ASSERT_TRUE(actual.active);
  EXPECT_FALSE(actual.dynamicBranch);
  EXPECT_GT(actual.value, 0.0);
  EXPECT_LT(
      actual.tangentialDisplacementNorm, options.staticFrictionDisplacement);
  EXPECT_GT(actual.gradient.head<6>().norm(), 0.0);

  options.coefficient = 0.0;
  EXPECT_FALSE(
      expdetail::rigidIpcPointPointFrictionPotential(
          laggedA, laggedB, pointA, pointB, options)
          .active);
}

//==============================================================================
TEST(RigidIpcBarrier, PointEdgeFrictionPotentialMatchesFiniteDifferences)
{
  expdetail::RigidIpcFrictionOptions options;
  options.coefficient = 0.35;
  options.laggedNormalForce = 6.0;
  options.staticFrictionDisplacement = 0.12;

  const Eigen::Vector3d laggedPoint(0.2, 0.3, 0.4);
  const Eigen::Vector3d laggedEdgeA(-0.4, -0.1, 0.0);
  const Eigen::Vector3d laggedEdgeB(0.6, 0.2, 0.8);

  Eigen::Matrix<double, 9, 1> x;
  x << 0.36, 0.42, 0.43, -0.44, -0.04, 0.04, 0.66, 0.12, 0.83;

  const auto value = [&](const Eigen::Matrix<double, 9, 1>& y) {
    return expdetail::rigidIpcPointEdgeFrictionPotential(
               laggedPoint,
               laggedEdgeA,
               laggedEdgeB,
               y.segment<3>(0),
               y.segment<3>(3),
               y.segment<3>(6),
               options)
        .value;
  };

  const auto actual = expdetail::rigidIpcPointEdgeFrictionPotential(
      laggedPoint,
      laggedEdgeA,
      laggedEdgeB,
      x.segment<3>(0),
      x.segment<3>(3),
      x.segment<3>(6),
      options);

  ASSERT_TRUE(actual.active);
  EXPECT_GT(actual.value, 0.0);
  expectVectorNear(
      actual.gradient.head<9>(), finiteGradientSized<9>(x, value), 1e-6);
  expectMatrixNear(
      actual.hessian.topLeftCorner<9, 9>(),
      finiteHessianSized<9>(x, value),
      3e-5);
}

//==============================================================================
TEST(RigidIpcBarrier, EdgeEdgeFrictionPotentialMatchesFiniteDifferences)
{
  expdetail::RigidIpcFrictionOptions options;
  options.coefficient = 0.45;
  options.laggedNormalForce = 3.0;
  options.staticFrictionDisplacement = 0.09;

  const Eigen::Vector3d laggedEdgeA0(-0.5, 0.0, 0.0);
  const Eigen::Vector3d laggedEdgeA1(0.5, 0.1, 0.1);
  const Eigen::Vector3d laggedEdgeB0(0.0, -0.4, 0.7);
  const Eigen::Vector3d laggedEdgeB1(0.1, 0.6, 0.9);

  expdetail::RigidIpcVector12d x;
  x << -0.4, 0.04, 0.02, 0.56, 0.08, 0.16, 0.04, -0.3, 0.66, 0.17, 0.54, 0.95;

  const auto value = [&](const expdetail::RigidIpcVector12d& y) {
    return expdetail::rigidIpcEdgeEdgeFrictionPotential(
               laggedEdgeA0,
               laggedEdgeA1,
               laggedEdgeB0,
               laggedEdgeB1,
               y.segment<3>(0),
               y.segment<3>(3),
               y.segment<3>(6),
               y.segment<3>(9),
               options)
        .value;
  };

  const auto actual = expdetail::rigidIpcEdgeEdgeFrictionPotential(
      laggedEdgeA0,
      laggedEdgeA1,
      laggedEdgeB0,
      laggedEdgeB1,
      x.segment<3>(0),
      x.segment<3>(3),
      x.segment<3>(6),
      x.segment<3>(9),
      options);

  ASSERT_TRUE(actual.active);
  EXPECT_GT(actual.value, 0.0);
  expectVectorNear(actual.gradient, finiteGradient(x, value), 1e-6);
  expectMatrixNear(actual.hessian, finiteHessian(x, value), 3e-5);
}

//==============================================================================
TEST(RigidIpcBarrier, PointTriangleFrictionPotentialMatchesFiniteDifferences)
{
  expdetail::RigidIpcFrictionOptions options;
  options.coefficient = 0.3;
  options.laggedNormalForce = 4.0;
  options.staticFrictionDisplacement = 0.1;

  const Eigen::Vector3d laggedPoint(0.25, 0.2, 0.6);
  const Eigen::Vector3d laggedTriangleA(0.0, 0.0, 0.0);
  const Eigen::Vector3d laggedTriangleB(1.0, 0.1, 0.0);
  const Eigen::Vector3d laggedTriangleC(0.1, 0.9, 0.2);

  expdetail::RigidIpcVector12d x;
  x << 0.38, 0.3, 0.58, -0.02, 0.05, 0.01, 1.06, 0.04, 0.03, 0.13, 0.98, 0.22;

  const auto value = [&](const expdetail::RigidIpcVector12d& y) {
    return expdetail::rigidIpcPointTriangleFrictionPotential(
               laggedPoint,
               laggedTriangleA,
               laggedTriangleB,
               laggedTriangleC,
               y.segment<3>(0),
               y.segment<3>(3),
               y.segment<3>(6),
               y.segment<3>(9),
               options)
        .value;
  };

  const auto actual = expdetail::rigidIpcPointTriangleFrictionPotential(
      laggedPoint,
      laggedTriangleA,
      laggedTriangleB,
      laggedTriangleC,
      x.segment<3>(0),
      x.segment<3>(3),
      x.segment<3>(6),
      x.segment<3>(9),
      options);

  ASSERT_TRUE(actual.active);
  EXPECT_GT(actual.value, 0.0);
  expectVectorNear(actual.gradient, finiteGradient(x, value), 1e-6);
  expectMatrixNear(actual.hessian, finiteHessian(x, value), 3e-5);
}

//==============================================================================
TEST(
    RigidIpcBarrier, PointPointReducedFrictionDerivativesMatchFiniteDifferences)
{
  expdetail::RigidIpcFrictionOptions options;
  options.coefficient = 0.4;
  options.laggedNormalForce = 5.0;
  options.staticFrictionDisplacement = 0.2;
  options.projectReducedHessianToPsd = false;

  const Eigen::Vector3d pointA(0.2, 0.1, 0.0);
  const Eigen::Vector3d pointB(-0.1, 0.05, 0.0);
  const expdetail::RigidIpcPose laggedAPose{
      Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose laggedBPose{
      Eigen::Vector3d(0.0, 0.0, 0.7), Eigen::Vector3d::Zero()};

  expdetail::RigidIpcVector12d x;
  x << 0.03, 0.02, 0.0, 0.04, -0.03, 0.02, -0.01, 0.04, 0.68, -0.02, 0.05, 0.03;

  const auto value = [&](const expdetail::RigidIpcVector12d& y) {
    return expdetail::rigidIpcPointPointReducedFrictionPotential(
               pointA,
               laggedAPose,
               poseFromVector(y, 0),
               pointB,
               laggedBPose,
               poseFromVector(y, 1),
               options)
        .value;
  };

  const auto actual = expdetail::rigidIpcPointPointReducedFrictionPotential(
      pointA,
      laggedAPose,
      poseFromVector(x, 0),
      pointB,
      laggedBPose,
      poseFromVector(x, 1),
      options);

  ASSERT_TRUE(actual.active);
  EXPECT_GT(actual.value, 0.0);
  expectVectorNear(actual.gradient, finiteGradient(x, value), 5e-5);
  expectMatrixNear(actual.hessian, finiteHessian(x, value), 1e-2);
}

//==============================================================================
TEST(RigidIpcBarrier, PointEdgeReducedFrictionDerivativesMatchFiniteDifferences)
{
  expdetail::RigidIpcFrictionOptions options;
  options.coefficient = 0.5;
  options.laggedNormalForce = 4.0;
  options.staticFrictionDisplacement = 0.15;
  options.projectReducedHessianToPsd = false;

  const Eigen::Vector3d point(0.1, -0.2, 0.0);
  const Eigen::Vector3d edgeA(-0.3, 0.0, 0.1);
  const Eigen::Vector3d edgeB(0.4, 0.2, 0.0);
  const expdetail::RigidIpcPose laggedPointPose{
      Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d::Zero()};
  const expdetail::RigidIpcPose laggedEdgePose{
      Eigen::Vector3d(0.0, 0.1, 0.5), Eigen::Vector3d::Zero()};

  expdetail::RigidIpcVector12d x;
  x << 0.06, 0.03, 0.02, 0.03, -0.02, 0.04, -0.03, 0.17, 0.48, -0.04, 0.05,
      0.02;

  const auto value = [&](const expdetail::RigidIpcVector12d& y) {
    return expdetail::rigidIpcPointEdgeReducedFrictionPotential(
               point,
               laggedPointPose,
               poseFromVector(y, 0),
               edgeA,
               edgeB,
               laggedEdgePose,
               poseFromVector(y, 1),
               options)
        .value;
  };

  const auto actual = expdetail::rigidIpcPointEdgeReducedFrictionPotential(
      point,
      laggedPointPose,
      poseFromVector(x, 0),
      edgeA,
      edgeB,
      laggedEdgePose,
      poseFromVector(x, 1),
      options);

  ASSERT_TRUE(actual.active);
  EXPECT_GT(actual.value, 0.0);
  expectVectorNear(actual.gradient, finiteGradient(x, value), 6e-5);
  expectMatrixNear(actual.hessian, finiteHessian(x, value), 2e-2);
}

//==============================================================================
TEST(RigidIpcBarrier, ObjectiveAssemblyAddsLaggedFrictionRows)
{
  expdetail::RigidIpcBarrierOptions barrierOptions;
  barrierOptions.squaredActivationDistance = 1.0;
  barrierOptions.stiffness = 1.2;

  expdetail::RigidIpcFrictionOptions frictionOptions;
  frictionOptions.coefficient = 1.0;
  frictionOptions.staticFrictionDisplacement = 0.05;
  frictionOptions.projectReducedHessianToPsd = true;

  expdetail::RigidIpcBarrierSurface dynamicPoint;
  dynamicPoint.frictionCoefficient = 0.25;
  dynamicPoint.vertices.push_back(Eigen::Vector3d::Zero());

  expdetail::RigidIpcBarrierSurface staticPoint;
  staticPoint.dynamic = false;
  staticPoint.frictionCoefficient = 1.0;
  staticPoint.pose.position = Eigen::Vector3d(0.0, 0.0, 0.4);
  staticPoint.vertices.push_back(Eigen::Vector3d::Zero());

  const std::array<expdetail::RigidIpcBarrierSurface, 2> laggedSurfaces{
      dynamicPoint, staticPoint};
  dynamicPoint.pose.position = Eigen::Vector3d(0.1, 0.0, 0.0);
  const std::array<expdetail::RigidIpcBarrierSurface, 2> currentSurfaces{
      dynamicPoint, staticPoint};

  const auto frictionless = expdetail::assembleRigidIpcObjectiveSystem(
      currentSurfaces,
      std::span<const expdetail::RigidIpcBodyDynamicsTerm>{},
      barrierOptions);
  const auto withFriction = expdetail::assembleRigidIpcObjectiveSystem(
      currentSurfaces,
      laggedSurfaces,
      std::span<const expdetail::RigidIpcBodyDynamicsTerm>{},
      barrierOptions,
      frictionOptions);

  ASSERT_EQ(withFriction.activeFrictionConstraints.size(), 1u);
  EXPECT_EQ(
      withFriction.activeFrictionConstraints.front().primitive,
      expdetail::RigidIpcBarrierPrimitive::VertexVertex);
  EXPECT_NEAR(
      withFriction.activeFrictionConstraints.front().coefficient, 0.5, 1e-15);
  EXPECT_GT(
      withFriction.activeFrictionConstraints.front().laggedNormalForce, 0.0);
  EXPECT_GT(withFriction.value, frictionless.value);
  EXPECT_EQ(withFriction.gradient.size(), frictionless.gradient.size());
  EXPECT_EQ(withFriction.hessian.rows(), frictionless.hessian.rows());
  EXPECT_EQ(withFriction.hessian.cols(), frictionless.hessian.cols());
}

//==============================================================================
TEST(RigidIpcBarrier, SceneAssemblyScattersDynamicBodyRows)
{
  expdetail::RigidIpcBarrierOptions options;
  options.squaredActivationDistance = 1.0;
  options.stiffness = 1.4;

  expdetail::RigidIpcBarrierSurface dynamicBody;
  dynamicBody.body = 10;
  dynamicBody.dynamic = true;
  dynamicBody.vertices.push_back(Eigen::Vector3d::Zero());

  expdetail::RigidIpcBarrierSurface staticBody;
  staticBody.body = 20;
  staticBody.dynamic = false;
  staticBody.pose.position = Eigen::Vector3d(0.0, 0.0, 0.35);
  staticBody.vertices.push_back(Eigen::Vector3d::Zero());

  const std::array<expdetail::RigidIpcBarrierSurface, 2> surfaces{
      dynamicBody, staticBody};
  const auto assembly
      = expdetail::assembleRigidIpcBarrierSystem(surfaces, options);
  const auto reduced = expdetail::rigidIpcPointPointReducedBarrier(
      dynamicBody.vertices.front(),
      dynamicBody.pose,
      staticBody.vertices.front(),
      staticBody.pose,
      options);

  ASSERT_TRUE(reduced.active);
  ASSERT_EQ(assembly.bodyDofOffsets.size(), 2u);
  EXPECT_EQ(assembly.bodyDofOffsets[0], 0u);
  EXPECT_EQ(
      assembly.bodyDofOffsets[1], expdetail::RigidIpcBarrierAssembly::npos);
  ASSERT_EQ(assembly.gradient.size(), 6);
  EXPECT_EQ(assembly.hessian.rows(), 6);
  EXPECT_EQ(assembly.hessian.cols(), 6);
  ASSERT_EQ(assembly.activeConstraints.size(), 1u);
  EXPECT_EQ(
      assembly.activeConstraints.front().primitive,
      expdetail::RigidIpcBarrierPrimitive::VertexVertex);
  EXPECT_EQ(assembly.activeConstraints.front().bodyA, 0u);
  EXPECT_EQ(assembly.activeConstraints.front().bodyB, 1u);
  EXPECT_NEAR(assembly.value, reduced.value, 1e-14);
  expectVectorNear(assembly.gradient, reduced.gradient.head<6>(), 1e-12);

  const Eigen::MatrixXd denseHessian(assembly.hessian);
  expectMatrixNear(denseHessian, reduced.hessian.topLeftCorner<6, 6>(), 1e-12);
}

//==============================================================================
TEST(RigidIpcBarrier, SceneAssemblyOwnsAllCrossBodyPrimitiveFamilies)
{
  expdetail::RigidIpcBarrierOptions options;
  options.squaredActivationDistance = 1.0;
  options.stiffness = 0.8;

  expdetail::RigidIpcBarrierSurface bodyA;
  bodyA.vertices
      = {Eigen::Vector3d(0.0, 0.0, 0.0),
         Eigen::Vector3d(1.0, 0.0, 0.0),
         Eigen::Vector3d(0.0, 1.0, 0.0)};
  bodyA.triangles = {Eigen::Vector3i(0, 1, 2)};

  expdetail::RigidIpcBarrierSurface bodyB = bodyA;
  bodyB.pose.position = Eigen::Vector3d(0.0, 0.0, 0.25);

  const std::array<expdetail::RigidIpcBarrierSurface, 2> surfaces{bodyA, bodyB};
  const auto assembly
      = expdetail::assembleRigidIpcBarrierSystem(surfaces, options);

  EXPECT_EQ(assembly.bodyDofOffsets[0], 0u);
  EXPECT_EQ(assembly.bodyDofOffsets[1], 6u);
  EXPECT_EQ(assembly.gradient.size(), 12);
  EXPECT_EQ(assembly.hessian.rows(), 12);
  EXPECT_EQ(assembly.hessian.cols(), 12);
  EXPECT_GT(assembly.value, 0.0);
  EXPECT_GT(assembly.activeConstraints.size(), 0u);
  EXPECT_TRUE(containsPrimitive(
      assembly, expdetail::RigidIpcBarrierPrimitive::VertexVertex));
  EXPECT_TRUE(containsPrimitive(
      assembly, expdetail::RigidIpcBarrierPrimitive::EdgeVertex));
  EXPECT_TRUE(containsPrimitive(
      assembly, expdetail::RigidIpcBarrierPrimitive::EdgeEdge));
  EXPECT_TRUE(containsPrimitive(
      assembly, expdetail::RigidIpcBarrierPrimitive::FaceVertex));

  const Eigen::MatrixXd denseHessian(assembly.hessian);
  expectMatrixNear(denseHessian, denseHessian.transpose(), 1e-10);
}

//==============================================================================
TEST(RigidIpcBarrier, SceneAssemblySkipsStaticStaticPairs)
{
  expdetail::RigidIpcBarrierOptions options;
  options.squaredActivationDistance = 1.0;

  expdetail::RigidIpcBarrierSurface bodyA;
  bodyA.dynamic = false;
  bodyA.vertices.push_back(Eigen::Vector3d::Zero());

  expdetail::RigidIpcBarrierSurface bodyB;
  bodyB.dynamic = false;
  bodyB.pose.position = Eigen::Vector3d(0.0, 0.0, 0.25);
  bodyB.vertices.push_back(Eigen::Vector3d::Zero());

  const std::array<expdetail::RigidIpcBarrierSurface, 2> surfaces{bodyA, bodyB};
  const auto assembly
      = expdetail::assembleRigidIpcBarrierSystem(surfaces, options);

  EXPECT_EQ(assembly.gradient.size(), 0);
  EXPECT_EQ(assembly.hessian.rows(), 0);
  EXPECT_EQ(assembly.hessian.cols(), 0);
  EXPECT_EQ(assembly.value, 0.0);
  EXPECT_TRUE(assembly.activeConstraints.empty());
}

//==============================================================================
// The conservative broad-phase cull must be behavior-preserving: adding a body
// far outside the activation distance contributes nothing to the assembled
// system, and a within-activation pair is never wrongly culled.
TEST(RigidIpcBarrier, SceneAssemblyBroadPhaseIsBehaviorPreserving)
{
  expdetail::RigidIpcBarrierOptions options;
  options.squaredActivationDistance = 1.0;

  expdetail::RigidIpcBarrierSurface dynamicNear;
  dynamicNear.dynamic = true;
  dynamicNear.pose.position = Eigen::Vector3d(0.0, 0.0, 0.25);
  dynamicNear.vertices.push_back(Eigen::Vector3d::Zero());

  expdetail::RigidIpcBarrierSurface staticNear;
  staticNear.dynamic = false;
  staticNear.vertices.push_back(Eigen::Vector3d::Zero());

  // Far outside the activation distance (1.0) from both near bodies.
  expdetail::RigidIpcBarrierSurface farDynamic;
  farDynamic.dynamic = true;
  farDynamic.pose.position = Eigen::Vector3d(0.0, 0.0, 100.0);
  farDynamic.vertices.push_back(Eigen::Vector3d::Zero());

  const std::array<expdetail::RigidIpcBarrierSurface, 2> nearOnly{
      dynamicNear, staticNear};
  const std::array<expdetail::RigidIpcBarrierSurface, 3> nearPlusFar{
      dynamicNear, staticNear, farDynamic};

  const auto nearAssembly
      = expdetail::assembleRigidIpcBarrierSystem(nearOnly, options);
  const auto withFarAssembly
      = expdetail::assembleRigidIpcBarrierSystem(nearPlusFar, options);

  // The within-activation pair is genuinely active (not culled).
  EXPECT_GT(nearAssembly.activeConstraints.size(), 0u);

  // The far body adds no active constraints and does not perturb the near-pair
  // value or gradient; its own dynamic DOF block stays exactly zero.
  EXPECT_EQ(
      withFarAssembly.activeConstraints.size(),
      nearAssembly.activeConstraints.size());
  EXPECT_DOUBLE_EQ(withFarAssembly.value, nearAssembly.value);
  ASSERT_EQ(nearAssembly.gradient.size(), 6);
  ASSERT_EQ(withFarAssembly.gradient.size(), 12);
  const Eigen::VectorXd nearBlockDelta
      = withFarAssembly.gradient.head(6) - nearAssembly.gradient;
  EXPECT_TRUE(nearBlockDelta.isZero(0.0));
  EXPECT_TRUE(withFarAssembly.gradient.tail(6).isZero(0.0));
}

//==============================================================================
TEST(RigidIpcBarrier, LineSearchLimitsFaceVertexCrossing)
{
  expdetail::RigidIpcBarrierSurface pointStart;
  pointStart.vertices.push_back(Eigen::Vector3d(0.25, 0.25, 0.0));
  pointStart.pose.position = Eigen::Vector3d(0.0, 0.0, 0.5);

  expdetail::RigidIpcBarrierSurface pointEnd = pointStart;
  pointEnd.pose.position = Eigen::Vector3d(0.0, 0.0, -0.5);

  expdetail::RigidIpcBarrierSurface triangle = makeTriangleSurface(0.0);
  triangle.dynamic = false;

  const std::array<expdetail::RigidIpcBarrierSurface, 2> start{
      pointStart, triangle};
  const std::array<expdetail::RigidIpcBarrierSurface, 2> end{
      pointEnd, triangle};
  const auto result = expdetail::computeRigidIpcLineSearchStepBound(start, end);

  EXPECT_TRUE(result.limited);
  EXPECT_FALSE(result.indeterminate);
  EXPECT_TRUE(result.allowsPositiveStep());
  EXPECT_GT(result.stepBound, 0.0);
  EXPECT_LE(result.stepBound, 0.5 + 1e-5);
  EXPECT_EQ(
      result.limitingPrimitive,
      expdetail::RigidIpcBarrierPrimitive::FaceVertex);
  EXPECT_EQ(result.bodyA, 0u);
  EXPECT_EQ(result.bodyB, 1u);
  EXPECT_EQ(result.stats.pointTriangleChecks, 1u);
  EXPECT_EQ(result.stats.hits, 1u);
  EXPECT_EQ(result.stats.zeroStepCount, 0u);
}

//==============================================================================
TEST(RigidIpcBarrier, LineSearchLimitsVertexVertexCrossing)
{
  expdetail::RigidIpcBarrierSurface pointStart;
  pointStart.vertices.push_back(Eigen::Vector3d::Zero());
  pointStart.pose.position = Eigen::Vector3d(0.0, 0.0, 0.5);

  expdetail::RigidIpcBarrierSurface pointEnd = pointStart;
  pointEnd.pose.position = Eigen::Vector3d(0.0, 0.0, -0.5);

  expdetail::RigidIpcBarrierSurface staticPoint;
  staticPoint.dynamic = false;
  staticPoint.vertices.push_back(Eigen::Vector3d::Zero());

  const std::array<expdetail::RigidIpcBarrierSurface, 2> start{
      pointStart, staticPoint};
  const std::array<expdetail::RigidIpcBarrierSurface, 2> end{
      pointEnd, staticPoint};
  const auto result = expdetail::computeRigidIpcLineSearchStepBound(start, end);

  EXPECT_TRUE(result.limited);
  EXPECT_FALSE(result.indeterminate);
  EXPECT_TRUE(result.allowsPositiveStep());
  EXPECT_GT(result.stepBound, 0.0);
  EXPECT_LE(result.stepBound, 0.5 + 1e-5);
  EXPECT_EQ(
      result.limitingPrimitive,
      expdetail::RigidIpcBarrierPrimitive::VertexVertex);
  EXPECT_EQ(result.bodyA, 0u);
  EXPECT_EQ(result.bodyB, 1u);
  EXPECT_EQ(result.vertices[0], 0u);
  EXPECT_EQ(result.vertices[1], 0u);
  EXPECT_EQ(result.stats.pointPointChecks, 1u);
  EXPECT_EQ(result.stats.hits, 1u);
  EXPECT_EQ(result.stats.zeroStepCount, 0u);
}

//==============================================================================
// The line-search swept broad-phase cull must use the rotational motion bound,
// not endpoint AABBs: a body that rotates a far vertex through a static point
// mid-step keeps both endpoints away from contact, so a naive start-AABB cull
// would wrongly drop the pair (tunneling). The motion-bound cull must keep it,
// while still skipping genuinely far, slow pairs.
TEST(RigidIpcBarrier, LineSearchSweptCullKeepsRotatingContact)
{
  // Dynamic vertex one unit from the body origin, rotating 180 deg about +z:
  // start (1,0,0) -> mid (0,1,0) -> end (-1,0,0). Both endpoints are ~1.41 from
  // the static point at (0,1,0), but the mid-step arc passes through it.
  expdetail::RigidIpcBarrierSurface rotatingStart;
  rotatingStart.vertices.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
  expdetail::RigidIpcBarrierSurface rotatingEnd = rotatingStart;
  rotatingEnd.pose.rotation = Eigen::Vector3d(0.0, 0.0, std::numbers::pi);

  expdetail::RigidIpcBarrierSurface staticPoint;
  staticPoint.dynamic = false;
  staticPoint.vertices.push_back(Eigen::Vector3d(0.0, 1.0, 0.0));

  const std::array<expdetail::RigidIpcBarrierSurface, 2> start{
      rotatingStart, staticPoint};
  const std::array<expdetail::RigidIpcBarrierSurface, 2> end{
      rotatingEnd, staticPoint};
  const auto result = expdetail::computeRigidIpcLineSearchStepBound(start, end);

  // The rotating pair is not culled, and the curved CCD finds the mid-step hit.
  EXPECT_EQ(result.stats.pointPointChecks, 1u);
  EXPECT_TRUE(result.limited);
  EXPECT_GT(result.stats.hits, 0u);
  EXPECT_LT(result.stepBound, 1.0);
  EXPECT_GT(result.stepBound, 0.0);
}

//==============================================================================
TEST(RigidIpcBarrier, LineSearchSweptCullSkipsFarSlowPairs)
{
  // A nearly-stationary dynamic vertex far from a static point: the swept cull
  // must skip the pair entirely (no CCD check, full step allowed).
  expdetail::RigidIpcBarrierSurface dynamicStart;
  dynamicStart.vertices.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
  expdetail::RigidIpcBarrierSurface dynamicEnd = dynamicStart;
  dynamicEnd.pose.position = Eigen::Vector3d(0.01, 0.0, 0.0);

  expdetail::RigidIpcBarrierSurface staticPoint;
  staticPoint.dynamic = false;
  staticPoint.vertices.push_back(Eigen::Vector3d(0.0, 10.0, 0.0));

  const std::array<expdetail::RigidIpcBarrierSurface, 2> start{
      dynamicStart, staticPoint};
  const std::array<expdetail::RigidIpcBarrierSurface, 2> end{
      dynamicEnd, staticPoint};
  const auto result = expdetail::computeRigidIpcLineSearchStepBound(start, end);

  EXPECT_EQ(result.stats.pointPointChecks, 0u);
  EXPECT_FALSE(result.limited);
  EXPECT_EQ(result.stats.hits, 0u);
  EXPECT_DOUBLE_EQ(result.stepBound, 1.0);
}

//==============================================================================
// Regression guard: the swept cull reach must include the CCD convergence
// tolerance. curvedAccdAdvance reports a hit once the clearance reaches
// convergeAbs (= max(tolerance, 1e-12)), so a pair whose true minimum distance
// lands in (minSeparation, minSeparation + convergeAbs] must NOT be culled.
// Here body A slides exactly one unit to x=1 while body B sits at x=1.0000005,
// leaving a final gap of 5e-7 < the default tolerance 1e-6. Without the
// convergeAbs term in the reach the pair is wrongly skipped (start AABB gap
// 1.0000005 > reach 1.0); with it the pair is examined and the contact found.
TEST(RigidIpcBarrier, LineSearchSweptCullKeepsContactWithinToleranceBand)
{
  expdetail::RigidIpcBarrierSurface movingStart;
  movingStart.vertices.push_back(Eigen::Vector3d::Zero());
  expdetail::RigidIpcBarrierSurface movingEnd = movingStart;
  movingEnd.pose.position = Eigen::Vector3d(1.0, 0.0, 0.0);

  expdetail::RigidIpcBarrierSurface staticPoint;
  staticPoint.dynamic = false;
  staticPoint.pose.position = Eigen::Vector3d(1.0000005, 0.0, 0.0);
  staticPoint.vertices.push_back(Eigen::Vector3d::Zero());

  const std::array<expdetail::RigidIpcBarrierSurface, 2> start{
      movingStart, staticPoint};
  const std::array<expdetail::RigidIpcBarrierSurface, 2> end{
      movingEnd, staticPoint};
  const auto result = expdetail::computeRigidIpcLineSearchStepBound(start, end);

  EXPECT_EQ(result.stats.pointPointChecks, 1u);
  EXPECT_GT(result.stats.hits, 0u);
}

//==============================================================================
TEST(RigidIpcBarrier, LineSearchRejectsInitialSeparationViolation)
{
  expdetail::RigidIpcBarrierSurface point;
  point.vertices.push_back(Eigen::Vector3d(0.25, 0.25, 0.0));
  point.pose.position = Eigen::Vector3d(0.0, 0.0, 0.05);

  expdetail::RigidIpcBarrierSurface triangle = makeTriangleSurface(0.0);
  triangle.dynamic = false;

  expdetail::RigidIpcLineSearchOptions options;
  options.minSeparation = 0.1;

  const std::array<expdetail::RigidIpcBarrierSurface, 2> surfaces{
      point, triangle};
  const auto result = expdetail::computeRigidIpcLineSearchStepBound(
      surfaces, surfaces, options);

  EXPECT_TRUE(result.limited);
  EXPECT_FALSE(result.allowsPositiveStep());
  EXPECT_EQ(result.stepBound, 0.0);
  EXPECT_EQ(result.stats.hits, 1u);
  EXPECT_EQ(result.stats.zeroStepCount, 1u);
}

//==============================================================================
// When the conservative CCD exhausts its iteration budget on a pair (here
// forced with maxIterations = 1), it cannot prove a definitive hit/miss -- but
// every advance it took was a provably contact-free sub-step, so the time it
// reached is a valid lower bound on the true time of impact. The line search
// uses that as a conservative POSITIVE step bound (limiting, not blocking) so
// the solve can still advance, while staying strictly before the true crossing.
// Here the point sweeps from z = 0.5 through the triangle at z = 0 (true TOI =
// 0.5), so the reported bound must be positive and < 0.5 -- intersection-free,
// not frozen.
TEST(RigidIpcBarrier, LineSearchUsesProvenSafeTimeOnIterationExhaustion)
{
  expdetail::RigidIpcBarrierSurface pointStart;
  pointStart.vertices.push_back(Eigen::Vector3d(0.25, 0.25, 0.0));
  pointStart.pose.position = Eigen::Vector3d(0.0, 0.0, 0.5);

  expdetail::RigidIpcBarrierSurface pointEnd = pointStart;
  pointEnd.pose.position = Eigen::Vector3d(0.0, 0.0, -0.5);

  expdetail::RigidIpcBarrierSurface triangle = makeTriangleSurface(0.0);
  triangle.dynamic = false;

  expdetail::RigidIpcLineSearchOptions options;
  options.maxIterations = 1;

  const std::array<expdetail::RigidIpcBarrierSurface, 2> start{
      pointStart, triangle};
  const std::array<expdetail::RigidIpcBarrierSurface, 2> end{
      pointEnd, triangle};
  const auto result
      = expdetail::computeRigidIpcLineSearchStepBound(start, end, options);

  EXPECT_GT(result.stats.indeterminate, 0u); // the ACCD did exhaust its budget
  EXPECT_TRUE(result.limited);
  // A provably-safe positive bound, not a frozen zero step.
  EXPECT_TRUE(result.allowsPositiveStep());
  EXPECT_FALSE(result.indeterminate);
  EXPECT_GT(result.stepBound, 0.0);
  // Strictly before the true crossing at t = 0.5 (intersection-free guarantee).
  EXPECT_LT(result.stepBound, 0.5);
}

//==============================================================================
TEST(RigidIpcBarrier, LineSearchChecksSupportedSurfacePrimitiveFamilies)
{
  // bodyA slides in x while bodyB stays a fixed small offset away in z: the
  // pair is close enough that the swept broad phase examines it (exercising all
  // primitive families), but the parallel-plane slide never produces a contact.
  expdetail::RigidIpcBarrierSurface bodyAStart = makeTriangleSurface(0.0);
  expdetail::RigidIpcBarrierSurface bodyAEnd = bodyAStart;
  bodyAEnd.pose.position = Eigen::Vector3d(0.5, 0.0, 0.0);
  expdetail::RigidIpcBarrierSurface bodyB = makeTriangleSurface(0.3);

  const std::array<expdetail::RigidIpcBarrierSurface, 2> start{
      bodyAStart, bodyB};
  const std::array<expdetail::RigidIpcBarrierSurface, 2> end{bodyAEnd, bodyB};
  const auto result = expdetail::computeRigidIpcLineSearchStepBound(start, end);

  EXPECT_FALSE(result.limited);
  EXPECT_TRUE(result.allowsPositiveStep());
  EXPECT_EQ(result.stepBound, 1.0);
  EXPECT_GT(result.stats.pointPointChecks, 0u);
  EXPECT_GT(result.stats.pointEdgeChecks, 0u);
  EXPECT_GT(result.stats.edgeEdgeChecks, 0u);
  EXPECT_GT(result.stats.pointTriangleChecks, 0u);
  EXPECT_EQ(result.stats.hits, 0u);
  EXPECT_EQ(
      result.stats.misses,
      result.stats.pointPointChecks + result.stats.pointEdgeChecks
          + result.stats.edgeEdgeChecks + result.stats.pointTriangleChecks);
}

//==============================================================================
TEST(RigidIpcBarrier, ProjectedNewtonStepSolvesBarrierSystem)
{
  Eigen::VectorXd gradient(2);
  gradient << 2.0, -4.0;
  Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(2, 2);
  hessian.diagonal() << 2.0, 4.0;

  expdetail::RigidIpcProjectedNewtonOptions options;
  options.hessianRegularization = 0.0;
  options.gradientTolerance = 0.0;

  const auto step = expdetail::computeRigidIpcProjectedNewtonStep(
      makeAssembly(gradient, hessian), options);

  ASSERT_TRUE(step.success);
  EXPECT_FALSE(step.converged);
  EXPECT_EQ(step.status, expdetail::RigidIpcProjectedNewtonStatus::DescentStep);
  EXPECT_TRUE(step.hasDescentDirection());

  Eigen::VectorXd expected(2);
  expected << -1.0, 1.0;
  expectVectorNear(step.delta.asEigen(), expected, 1e-14);
  EXPECT_NEAR(step.stats.rawStepNorm, expected.norm(), 1e-14);
  EXPECT_NEAR(step.stats.stepNorm, expected.norm(), 1e-14);
  EXPECT_NEAR(step.stats.gradientDotStep, -6.0, 1e-14);
}

//==============================================================================
TEST(RigidIpcBarrier, ProjectedNewtonStepHonorsLineSearchBound)
{
  Eigen::VectorXd gradient(2);
  gradient << -2.0, 0.0;
  const Eigen::MatrixXd hessian = Eigen::MatrixXd::Identity(2, 2);

  expdetail::RigidIpcLineSearchResult lineSearch;
  lineSearch.limited = true;
  lineSearch.stepBound = 0.25;

  expdetail::RigidIpcProjectedNewtonOptions options;
  options.hessianRegularization = 0.0;
  options.gradientTolerance = 0.0;
  options.lineSearchSafetyScale = 0.8;

  const auto step = expdetail::computeRigidIpcProjectedNewtonStep(
      makeAssembly(gradient, hessian), lineSearch, options);

  ASSERT_TRUE(step.success);
  EXPECT_EQ(step.status, expdetail::RigidIpcProjectedNewtonStatus::DescentStep);
  EXPECT_TRUE(step.stats.usedLineSearch);
  EXPECT_TRUE(step.stats.lineSearchLimited);
  EXPECT_NEAR(step.stats.stepScale, 0.2, 1e-14);

  Eigen::VectorXd expected(2);
  expected << 0.4, 0.0;
  expectVectorNear(step.delta.asEigen(), expected, 1e-14);
  EXPECT_NEAR(step.stats.rawStepNorm, 2.0, 1e-14);
  EXPECT_NEAR(step.stats.stepNorm, 0.4, 1e-14);
}

//==============================================================================
TEST(RigidIpcBarrier, ProjectedNewtonStepBlocksUnsafeLineSearch)
{
  Eigen::VectorXd gradient(1);
  gradient << -1.0;
  const Eigen::MatrixXd hessian = Eigen::MatrixXd::Identity(1, 1);

  expdetail::RigidIpcLineSearchResult lineSearch;
  lineSearch.limited = true;
  lineSearch.indeterminate = true;
  lineSearch.stepBound = 0.0;

  expdetail::RigidIpcProjectedNewtonOptions options;
  options.hessianRegularization = 0.0;
  options.gradientTolerance = 0.0;

  const auto step = expdetail::computeRigidIpcProjectedNewtonStep(
      makeAssembly(gradient, hessian), lineSearch, options);

  EXPECT_FALSE(step.success);
  EXPECT_TRUE(step.lineSearchBlocked);
  EXPECT_EQ(
      step.status, expdetail::RigidIpcProjectedNewtonStatus::LineSearchBlocked);
  EXPECT_EQ(step.delta.size(), 1);
  EXPECT_EQ(step.delta[0], 0.0);
  EXPECT_TRUE(step.stats.usedLineSearch);
  EXPECT_TRUE(step.stats.lineSearchLimited);
  EXPECT_EQ(step.stats.stepScale, 0.0);
}

//==============================================================================
TEST(RigidIpcBarrier, ProjectedNewtonStepTreatsNoDofsAsConverged)
{
  expdetail::RigidIpcBarrierAssembly assembly;
  assembly.gradient.resize(0);
  assembly.hessian.resize(0, 0);

  const auto step = expdetail::computeRigidIpcProjectedNewtonStep(assembly);

  EXPECT_TRUE(step.success);
  EXPECT_TRUE(step.converged);
  EXPECT_EQ(step.status, expdetail::RigidIpcProjectedNewtonStatus::NoDofs);
  EXPECT_EQ(step.delta.size(), 0);
  EXPECT_EQ(step.stats.dofs, 0u);
}

//==============================================================================
TEST(RigidIpcBarrier, ObjectiveAssemblyAddsBodyDynamicsTerm)
{
  expdetail::RigidIpcBarrierSurface surface;
  surface.dynamic = true;
  surface.pose.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  surface.pose.rotation = Eigen::Vector3d(0.1, 0.2, 0.3);

  expdetail::RigidIpcBodyDynamicsTerm dynamics;
  dynamics.active = true;
  dynamics.targetPose.position = Eigen::Vector3d(0.5, 2.5, 4.0);
  dynamics.targetPose.rotation = Eigen::Vector3d(0.0, 0.25, 0.5);
  dynamics.diagonalWeights << 2.0, 3.0, 4.0, 5.0, 6.0, 7.0;
  dynamics.generalizedForce << 1.0, -2.0, 0.5, -1.5, 0.25, 2.0;

  const std::array<expdetail::RigidIpcBarrierSurface, 1> surfaces{surface};
  const std::array<expdetail::RigidIpcBodyDynamicsTerm, 1> dynamicsTerms{
      dynamics};
  const auto assembly = expdetail::assembleRigidIpcObjectiveSystem(
      surfaces, dynamicsTerms, expdetail::RigidIpcBarrierOptions{});

  expdetail::RigidIpcVector6d q;
  q << 1.0, 2.0, 3.0, 0.1, 0.2, 0.3;
  expdetail::RigidIpcVector6d target;
  target << 0.5, 2.5, 4.0, 0.0, 0.25, 0.5;
  const expdetail::RigidIpcVector6d residual = q - target;
  const expdetail::RigidIpcVector6d expectedGradient
      = dynamics.diagonalWeights.cwiseProduct(residual)
        - dynamics.generalizedForce;
  const double expectedValue
      = 0.5 * dynamics.diagonalWeights.dot(residual.cwiseProduct(residual))
        - dynamics.generalizedForce.dot(q);

  ASSERT_EQ(assembly.bodyDofOffsets.size(), 1u);
  EXPECT_EQ(assembly.bodyDofOffsets[0], 0u);
  EXPECT_EQ(assembly.activeDynamicsTerms, 1u);
  EXPECT_TRUE(assembly.activeConstraints.empty());
  EXPECT_NEAR(assembly.value, expectedValue, 1e-14);
  expectVectorNear(assembly.gradient, expectedGradient, 1e-14);

  const Eigen::MatrixXd denseHessian(assembly.hessian);
  expectMatrixNear(
      denseHessian,
      dynamics.diagonalWeights.asDiagonal().toDenseMatrix(),
      1e-14);
}

//==============================================================================
TEST(RigidIpcBarrier, BodyDynamicsTermUsesPhysicalMassInertiaAndStepState)
{
  expdetail::RigidIpcBodyDynamicsState state;
  state.pose.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  state.pose.rotation = Eigen::Vector3d(0.1, 0.2, 0.3);
  state.velocity << 0.5, -1.0, 2.0, 0.1, -0.2, 0.3;
  state.mass = 4.0;
  state.inertia = Eigen::Vector3d(2.0, 3.0, 5.0).asDiagonal();
  state.generalizedForce << 8.0, -4.0, 12.0, -1.0, 2.0, -3.0;

  constexpr double timeStep = 0.25;
  const auto term = expdetail::makeRigidIpcBodyDynamicsTerm(state, timeStep);

  ASSERT_TRUE(term.active);
  EXPECT_TRUE(term.targetPose.position.isApprox(
      Eigen::Vector3d(1.125, 1.75, 3.5), 1e-15));
  EXPECT_TRUE(term.targetPose.rotation.isApprox(
      Eigen::Vector3d(0.125, 0.15, 0.375), 1e-15));

  expdetail::RigidIpcVector6d expectedWeights;
  expectedWeights << 64.0, 64.0, 64.0, 32.0, 48.0, 80.0;
  expectVectorNear(term.diagonalWeights, expectedWeights, 1e-15);
  expectVectorNear(term.generalizedForce, state.generalizedForce, 1e-15);
}

//==============================================================================
TEST(RigidIpcBarrier, BodyDynamicsTermRejectsInvalidPhysicalState)
{
  expdetail::RigidIpcBodyDynamicsState state;
  state.mass = 1.0;

  EXPECT_FALSE(expdetail::makeRigidIpcBodyDynamicsTerm(state, 0.0).active);

  state.mass = -1.0;
  EXPECT_FALSE(expdetail::makeRigidIpcBodyDynamicsTerm(state, 0.01).active);

  state.mass = 1.0;
  state.velocity[0] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(expdetail::makeRigidIpcBodyDynamicsTerm(state, 0.01).active);
}

//==============================================================================
TEST(RigidIpcBarrier, ProjectedNewtonSolveFollowsBodyDynamicsTerm)
{
  expdetail::RigidIpcBarrierSurface surface;
  surface.dynamic = true;

  expdetail::RigidIpcBodyDynamicsTerm dynamics;
  dynamics.active = true;
  dynamics.diagonalWeights.setOnes();
  dynamics.diagonalWeights[0] = 2.0;
  dynamics.generalizedForce[0] = 1.0;

  expdetail::RigidIpcProjectedNewtonSolveOptions options;
  options.dynamicsTerms.push_back(dynamics);
  options.newton.hessianRegularization = 0.0;
  options.newton.gradientTolerance = 1e-12;
  options.stepTolerance = 1e-12;

  const std::array<expdetail::RigidIpcBarrierSurface, 1> surfaces{surface};
  const auto result
      = expdetail::solveRigidIpcProjectedNewtonBarrierSystem(surfaces, options);

  EXPECT_TRUE(result.converged);
  EXPECT_FALSE(result.failed);
  EXPECT_TRUE(result.madeProgress());
  EXPECT_EQ(result.stats.acceptedSteps, 1u);
  EXPECT_EQ(result.assembly.activeDynamicsTerms, 1u);
  EXPECT_NEAR(result.surfaces.front().pose.position.x(), 0.5, 1e-12);
  EXPECT_NEAR(result.stats.finalGradientNorm, 0.0, 1e-12);
}

//==============================================================================
TEST(RigidIpcBarrier, ProjectedNewtonSolveScratchUsesProvidedAllocator)
{
  namespace common = dart::common;

  common::MemoryAllocatorDebugger<common::FreeListAllocator> allocator;

  {
    expdetail::RigidIpcBarrierSurface surfaceA = makeTriangleSurface(0.0);
    expdetail::RigidIpcBarrierSurface surfaceB = makeTriangleSurface(0.25);
    const std::array<expdetail::RigidIpcBarrierSurface, 2> surfaces{
        surfaceA, surfaceB};

    expdetail::RigidIpcProjectedNewtonSolveOptions options(allocator);
    options.barrier.squaredActivationDistance = 1.0;
    options.maxIterations = 0;
    auto& articulation = options.articulationConstraints.emplace_back();
    articulation.active = true;
    articulation.bodyA = 0;
    articulation.bodyB = 1;
    articulation.localPointA = Eigen::Vector3d::Zero();
    articulation.localPointB = Eigen::Vector3d::Zero();
    expdetail::RigidIpcProjectedNewtonSolveResult result(allocator);
    expdetail::RigidIpcProjectedNewtonSolveScratch scratch(allocator);

    expdetail::solveRigidIpcProjectedNewtonBarrierSystem(
        surfaces, options, result, scratch);

    EXPECT_GT(allocator.getAllocationCount(), 0u)
        << "allocator-aware rigid IPC projected-Newton scratch should reserve "
           "surface work vectors, repeated assembly scratch, step delta, and "
           "result assembly vectors from the provided free allocator";
    EXPECT_GE(allocator.getPeakAllocatedSize(), allocator.getAllocatedSize())
        << "solve-internal scratch should stay owned by the provided allocator";
  }

  EXPECT_TRUE(allocator.isEmpty());
}

//==============================================================================
TEST(RigidIpcBarrier, ProjectedNewtonSolveBacktracksForSufficientDecrease)
{
  expdetail::RigidIpcBarrierSurface surface;
  surface.dynamic = true;

  expdetail::RigidIpcBodyDynamicsTerm dynamics;
  dynamics.active = true;
  dynamics.diagonalWeights.setOnes();
  dynamics.diagonalWeights[0] = 2.0;
  dynamics.generalizedForce[0] = 1.0;

  expdetail::RigidIpcProjectedNewtonSolveOptions options;
  options.dynamicsTerms.push_back(dynamics);
  options.newton.hessianRegularization = 0.0;
  options.newton.gradientTolerance = 0.0;
  options.newton.sufficientDecreaseFactor = 0.8;
  options.stepTolerance = 0.13;

  const std::array<expdetail::RigidIpcBarrierSurface, 1> surfaces{surface};
  const auto result
      = expdetail::solveRigidIpcProjectedNewtonBarrierSystem(surfaces, options);

  EXPECT_TRUE(result.converged);
  EXPECT_FALSE(result.failed);
  EXPECT_EQ(result.stats.acceptedSteps, 1u);
  EXPECT_EQ(result.stats.sufficientDecreaseChecks, 3u);
  EXPECT_EQ(result.stats.sufficientDecreaseBacktracks, 2u);
  EXPECT_NEAR(result.lastStep.stats.stepScale, 0.25, 1e-14);
  EXPECT_NEAR(result.lastStep.stats.stepNorm, 0.125, 1e-14);
  EXPECT_NEAR(result.surfaces.front().pose.position.x(), 0.125, 1e-12);
  EXPECT_LT(result.stats.finalValue, result.stats.initialValue);
}

//==============================================================================
TEST(RigidIpcBarrier, ProjectedNewtonBacktrackingRechecksKinematicSweep)
{
  expdetail::RigidIpcBarrierSurface dynamicPoint;
  dynamicPoint.body = 0u;
  dynamicPoint.dynamic = true;
  dynamicPoint.pose.position = Eigen::Vector3d(0.2, 0.0, 0.0);
  dynamicPoint.vertices.push_back(Eigen::Vector3d::Zero());

  expdetail::RigidIpcBarrierSurface kinematicPoint;
  kinematicPoint.body = 1u;
  kinematicPoint.dynamic = false;
  kinematicPoint.kinematic = true;
  kinematicPoint.kinematicStartPose.position = Eigen::Vector3d::Zero();
  kinematicPoint.pose.position = Eigen::Vector3d(1.0, 0.0, 0.0);
  kinematicPoint.vertices.push_back(Eigen::Vector3d::Zero());

  expdetail::RigidIpcBodyDynamicsTerm dynamics;
  dynamics.active = true;
  dynamics.targetPose = dynamicPoint.pose;
  dynamics.diagonalWeights.setOnes();
  dynamics.generalizedForce[0] = 1.0;

  expdetail::RigidIpcProjectedNewtonSolveOptions options;
  options.barrier.squaredActivationDistance = 1e-6;
  options.dynamicsTerms.resize(2);
  options.dynamicsTerms[0] = dynamics;
  options.newton.hessianRegularization = 0.0;
  options.newton.gradientTolerance = 0.0;
  options.newton.sufficientDecreaseFactor = 0.8;
  options.stepTolerance = 1e-12;

  const std::array<expdetail::RigidIpcBarrierSurface, 2> surfaces{
      dynamicPoint, kinematicPoint};
  const auto result
      = expdetail::solveRigidIpcProjectedNewtonBarrierSystem(surfaces, options);

  EXPECT_TRUE(result.converged);
  EXPECT_FALSE(result.failed);
  EXPECT_EQ(result.stats.acceptedSteps, 1u);
  EXPECT_GT(result.stats.sufficientDecreaseBacktracks, 0u);
  EXPECT_GT(result.stats.lineSearchHits, 0u);
  EXPECT_GT(result.surfaces.front().pose.position.x(), 1.0);
  EXPECT_NEAR(result.surfaces.front().pose.position.x(), 1.2, 1e-12);
}

//==============================================================================
TEST(RigidIpcBarrier, ProjectedNewtonSolveAcceptsDecreasingArmijoFallback)
{
  expdetail::RigidIpcBarrierSurface surface;
  surface.dynamic = true;

  expdetail::RigidIpcBodyDynamicsTerm dynamics;
  dynamics.active = true;
  dynamics.diagonalWeights.setOnes();
  dynamics.diagonalWeights[0] = 2.0;
  dynamics.generalizedForce[0] = 1.0;

  expdetail::RigidIpcProjectedNewtonSolveOptions options;
  options.dynamicsTerms.push_back(dynamics);
  options.newton.hessianRegularization = 0.0;
  options.newton.gradientTolerance = 0.0;
  options.newton.sufficientDecreaseFactor = 0.8;
  options.newton.maxBacktrackingIterations = 0;
  options.stepTolerance = 1.0;

  const std::array<expdetail::RigidIpcBarrierSurface, 1> surfaces{surface};
  const auto result
      = expdetail::solveRigidIpcProjectedNewtonBarrierSystem(surfaces, options);

  EXPECT_TRUE(result.converged);
  EXPECT_FALSE(result.failed);
  EXPECT_EQ(result.stats.acceptedSteps, 1u);
  EXPECT_EQ(result.stats.sufficientDecreaseChecks, 1u);
  EXPECT_EQ(result.stats.sufficientDecreaseBacktracks, 0u);
  EXPECT_NEAR(result.lastStep.stats.stepScale, 1.0, 1e-14);
  EXPECT_NEAR(result.surfaces.front().pose.position.x(), 0.5, 1e-12);
  EXPECT_LT(result.stats.finalValue, result.stats.initialValue);
}

//==============================================================================
TEST(RigidIpcBarrier, ProjectedNewtonSolveAggregatesLineSearchDiagnostics)
{
  expdetail::RigidIpcBarrierSurface dynamicBody = makeTriangleSurface(0.0);
  dynamicBody.dynamic = true;

  // Offset just in z so the body's dynamics-driven x slide stays within the
  // swept broad-phase reach (the line search examines the pair every iteration)
  // without ever producing a contact.
  expdetail::RigidIpcBarrierSurface staticBody = makeTriangleSurface(0.3);
  staticBody.dynamic = false;

  expdetail::RigidIpcBodyDynamicsTerm dynamics;
  dynamics.active = true;
  dynamics.diagonalWeights.setOnes();
  dynamics.generalizedForce[0] = 1.0;

  expdetail::RigidIpcProjectedNewtonSolveOptions options;
  options.dynamicsTerms.resize(2);
  options.dynamicsTerms[0] = dynamics;
  options.newton.hessianRegularization = 0.0;
  options.newton.gradientTolerance = 1e-12;
  options.stepTolerance = 1e-12;

  const std::array<expdetail::RigidIpcBarrierSurface, 2> surfaces{
      dynamicBody, staticBody};
  const auto result
      = expdetail::solveRigidIpcProjectedNewtonBarrierSystem(surfaces, options);

  EXPECT_TRUE(result.converged);
  EXPECT_FALSE(result.failed);
  EXPECT_TRUE(result.madeProgress());
  EXPECT_GT(result.stats.lineSearchPointPointChecks, 0u);
  EXPECT_GT(result.stats.lineSearchPointEdgeChecks, 0u);
  EXPECT_GT(result.stats.lineSearchEdgeEdgeChecks, 0u);
  EXPECT_GT(result.stats.lineSearchPointTriangleChecks, 0u);
  EXPECT_EQ(result.stats.lineSearchHits, 0u);
  EXPECT_EQ(
      result.stats.lineSearchMisses,
      result.stats.lineSearchPointPointChecks
          + result.stats.lineSearchPointEdgeChecks
          + result.stats.lineSearchEdgeEdgeChecks
          + result.stats.lineSearchPointTriangleChecks);
  EXPECT_EQ(result.stats.lineSearchIndeterminateCount, 0u);
  EXPECT_EQ(result.stats.lineSearchZeroStepCount, 0u);
  EXPECT_FALSE(result.lineSearch.limited);
  EXPECT_EQ(result.lineSearch.stepBound, 1.0);
}

//==============================================================================
TEST(RigidIpcBarrier, ProjectedNewtonSolveReducesBarrierValue)
{
  expdetail::RigidIpcBarrierSurface dynamicBody;
  dynamicBody.dynamic = true;
  dynamicBody.pose.position = Eigen::Vector3d(0.0, 0.0, 0.25);
  dynamicBody.vertices.push_back(Eigen::Vector3d::Zero());

  expdetail::RigidIpcBarrierSurface staticBody;
  staticBody.dynamic = false;
  staticBody.vertices.push_back(Eigen::Vector3d::Zero());

  std::array<expdetail::RigidIpcBarrierSurface, 2> surfaces{
      dynamicBody, staticBody};

  expdetail::RigidIpcProjectedNewtonSolveOptions options;
  options.barrier.squaredActivationDistance = 1.0;
  options.newton.maxStepNorm = 0.05;
  options.maxIterations = 6;

  const auto initialAssembly
      = expdetail::assembleRigidIpcBarrierSystem(surfaces, options.barrier);
  const auto result
      = expdetail::solveRigidIpcProjectedNewtonBarrierSystem(surfaces, options);

  EXPECT_FALSE(result.failed);
  EXPECT_TRUE(result.madeProgress());
  EXPECT_GT(result.stats.acceptedSteps, 0u);
  EXPECT_GT(
      result.surfaces[0].pose.position.z(), dynamicBody.pose.position.z());
  EXPECT_LT(result.stats.finalValue, initialAssembly.value);
  EXPECT_LT(result.stats.finalGradientNorm, initialAssembly.gradient.norm());
}

//==============================================================================
TEST(RigidIpcBarrier, ProjectedNewtonSolveRepeatsLaggedFrictionPasses)
{
  expdetail::RigidIpcBarrierSurface dynamicBody;
  dynamicBody.dynamic = true;
  dynamicBody.frictionCoefficient = 0.25;
  dynamicBody.pose.position = Eigen::Vector3d(0.0, 0.0, 0.25);
  dynamicBody.vertices.push_back(Eigen::Vector3d::Zero());

  expdetail::RigidIpcBarrierSurface staticBody;
  staticBody.dynamic = false;
  staticBody.frictionCoefficient = 1.0;
  staticBody.vertices.push_back(Eigen::Vector3d::Zero());

  expdetail::RigidIpcProjectedNewtonSolveOptions options;
  options.barrier.squaredActivationDistance = 1.0;
  options.barrier.stiffness = 0.1;
  options.friction.coefficient = 1.0;
  options.friction.staticFrictionDisplacement = 0.05;
  options.frictionIterations = 2;
  options.maxIterations = 4;
  options.newton.gradientTolerance = 1e100;
  options.stepTolerance = 1e-12;

  const std::array<expdetail::RigidIpcBarrierSurface, 2> surfaces{
      dynamicBody, staticBody};
  const auto result
      = expdetail::solveRigidIpcProjectedNewtonBarrierSystem(surfaces, options);

  EXPECT_TRUE(result.converged);
  EXPECT_FALSE(result.failed);
  EXPECT_EQ(result.stats.frictionIterations, 2u);
  EXPECT_GT(result.stats.activeFrictionConstraints, 0u);
  EXPECT_EQ(result.stats.acceptedSteps, 0u);
  EXPECT_TRUE(std::isfinite(result.stats.finalMomentumBalance));

  options.frictionIterations = 3;
  options.frictionConvergenceTolerance = 1e100;
  const auto earlyStopResult
      = expdetail::solveRigidIpcProjectedNewtonBarrierSystem(surfaces, options);

  EXPECT_TRUE(earlyStopResult.converged);
  EXPECT_EQ(earlyStopResult.stats.frictionIterations, 1u);
}

//==============================================================================
TEST(RigidIpcBarrier, ProjectedNewtonSolveCanDisableLaggedFrictionPasses)
{
  expdetail::RigidIpcBarrierSurface dynamicBody;
  dynamicBody.dynamic = true;
  dynamicBody.pose.position = Eigen::Vector3d(0.0, 0.0, 0.25);
  dynamicBody.vertices.push_back(Eigen::Vector3d::Zero());

  expdetail::RigidIpcBarrierSurface staticBody;
  staticBody.dynamic = false;
  staticBody.vertices.push_back(Eigen::Vector3d::Zero());

  expdetail::RigidIpcProjectedNewtonSolveOptions options;
  options.barrier.squaredActivationDistance = 1.0;
  options.friction.coefficient = 1.0;
  options.friction.staticFrictionDisplacement = 0.05;
  options.frictionIterations = 0;
  options.newton.maxStepNorm = 0.05;
  options.maxIterations = 2;

  const std::array<expdetail::RigidIpcBarrierSurface, 2> surfaces{
      dynamicBody, staticBody};
  const auto result
      = expdetail::solveRigidIpcProjectedNewtonBarrierSystem(surfaces, options);

  EXPECT_EQ(result.stats.frictionIterations, 0u);
  EXPECT_EQ(result.stats.activeFrictionConstraints, 0u);
  EXPECT_TRUE(result.assembly.activeFrictionConstraints.empty());
}

//==============================================================================
TEST(RigidIpcBarrier, ProjectedNewtonSolveTreatsNoDofsAsConverged)
{
  expdetail::RigidIpcBarrierSurface bodyA;
  bodyA.dynamic = false;
  bodyA.vertices.push_back(Eigen::Vector3d::Zero());

  expdetail::RigidIpcBarrierSurface bodyB = bodyA;
  bodyB.pose.position = Eigen::Vector3d(0.0, 0.0, 0.25);

  const std::array<expdetail::RigidIpcBarrierSurface, 2> surfaces{bodyA, bodyB};
  const auto result
      = expdetail::solveRigidIpcProjectedNewtonBarrierSystem(surfaces);

  EXPECT_TRUE(result.converged);
  EXPECT_FALSE(result.failed);
  EXPECT_EQ(
      result.status, expdetail::RigidIpcProjectedNewtonSolveStatus::NoDofs);
  EXPECT_EQ(result.stats.acceptedSteps, 0u);
  EXPECT_EQ(result.assembly.gradient.size(), 0);
}

//==============================================================================
// Isolated correctness tests for the rigid CCD pose geometry primitives that
// every rigid IPC barrier/CCD path depends on.
TEST(RigidIpcCcdGeometry, RotationVectorToMatrixMatchesAngleAxis)
{
  // Zero rotation is the identity.
  EXPECT_TRUE(
      expdetail::rigidIpcRotationVectorToMatrix(Eigen::Vector3d::Zero())
          .isApprox(Eigen::Matrix3d::Identity()));

  // Arbitrary rotation vectors must match Eigen's axis-angle construction and
  // stay orthonormal/right-handed.
  const std::array<Eigen::Vector3d, 4> rotations{
      Eigen::Vector3d(0.0, 0.0, std::numbers::pi / 2.0),
      Eigen::Vector3d(std::numbers::pi / 3.0, 0.0, 0.0),
      Eigen::Vector3d(0.3, -0.7, 1.1),
      Eigen::Vector3d(-1.5, 0.4, 0.2)};
  for (const Eigen::Vector3d& r : rotations) {
    const double angle = r.norm();
    const Eigen::Vector3d axis = r / angle;
    const Eigen::Matrix3d expected
        = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    const Eigen::Matrix3d actual = expdetail::rigidIpcRotationVectorToMatrix(r);
    EXPECT_TRUE(actual.isApprox(expected, 1e-12));
    EXPECT_TRUE((actual * actual.transpose())
                    .isApprox(Eigen::Matrix3d::Identity(), 1e-12));
    EXPECT_NEAR(actual.determinant(), 1.0, 1e-12);
  }

  // A quarter turn about +z maps +x to +y.
  const Eigen::Matrix3d rz = expdetail::rigidIpcRotationVectorToMatrix(
      Eigen::Vector3d(0.0, 0.0, std::numbers::pi / 2.0));
  EXPECT_TRUE((rz * Eigen::Vector3d::UnitX())
                  .isApprox(Eigen::Vector3d::UnitY(), 1e-12));
}

//==============================================================================
TEST(RigidIpcCcdGeometry, TransformPointAppliesRotationThenTranslation)
{
  expdetail::RigidIpcPose pose;
  pose.position = Eigen::Vector3d(1.0, -2.0, 0.5);
  pose.rotation = Eigen::Vector3d(0.2, 0.5, -0.3);

  const Eigen::Matrix3d r
      = expdetail::rigidIpcRotationVectorToMatrix(pose.rotation);
  const Eigen::Vector3d local(0.7, 0.1, -0.4);
  const Eigen::Vector3d expected = r * local + pose.position;

  EXPECT_TRUE(
      expdetail::transformRigidIpcPoint(local, pose).isApprox(expected, 1e-12));

  // Identity pose leaves the point unchanged.
  EXPECT_TRUE(
      expdetail::transformRigidIpcPoint(local, expdetail::RigidIpcPose{})
          .isApprox(local, 1e-12));
}

//==============================================================================
TEST(RigidIpcCcdGeometry, InterpolatePoseIsLinearInPositionAndRotationVector)
{
  expdetail::RigidIpcPose start;
  start.position = Eigen::Vector3d(0.0, 1.0, 2.0);
  start.rotation = Eigen::Vector3d(0.1, -0.2, 0.3);
  expdetail::RigidIpcPose end;
  end.position = Eigen::Vector3d(2.0, -1.0, 0.0);
  end.rotation = Eigen::Vector3d(-0.3, 0.4, 0.9);

  const auto at0 = expdetail::interpolateRigidIpcPose(start, end, 0.0);
  EXPECT_TRUE(at0.position.isApprox(start.position));
  EXPECT_TRUE(at0.rotation.isApprox(start.rotation));

  const auto at1 = expdetail::interpolateRigidIpcPose(start, end, 1.0);
  EXPECT_TRUE(at1.position.isApprox(end.position));
  EXPECT_TRUE(at1.rotation.isApprox(end.rotation));

  const auto mid = expdetail::interpolateRigidIpcPose(start, end, 0.5);
  EXPECT_TRUE(mid.position.isApprox(0.5 * (start.position + end.position)));
  EXPECT_TRUE(mid.rotation.isApprox(0.5 * (start.rotation + end.rotation)));

  // The swept transform overload equals transforming at the interpolated pose.
  const Eigen::Vector3d local(0.3, -0.6, 0.2);
  const auto interpPose = expdetail::interpolateRigidIpcPose(start, end, 0.37);
  EXPECT_TRUE(
      expdetail::transformRigidIpcPoint(local, start, end, 0.37)
          .isApprox(
              expdetail::transformRigidIpcPoint(local, interpPose), 1e-12));
}

// The adaptive-kappa helpers port `ipc::initial_barrier_stiffness` and
// `ipc::update_barrier_stiffness` (dmin = 0) onto DART's squared-distance
// clamped-log barrier. These verify the suggested-kappa gradient ratio, the
// kappa_min / kappa_max clamp bounds, invalid-input fallbacks, and the
// per-iteration doubling rule.
TEST(RigidIpcAdaptiveStiffness, InitialStiffnessUsesGradientRatioWithinBounds)
{
  const double bbox = 1.0;
  const double squaredActivationDistance = 1e-4; // dhat = 0.01
  const double averageMass = 1.0;
  const double scale = 1e11;

  // Reproduce the clamp bounds the helper computes.
  const double d0 = 1e-8 * bbox;
  const double d0Squared = d0 * d0;
  const double secondDerivative
      = dc::c2ClampedLogBarrier(d0Squared, squaredActivationDistance)
            .secondDerivative;
  const double kappaMin
      = scale * averageMass / (4.0 * d0Squared * secondDerivative);
  const double kappaMax = 100.0 * kappaMin;

  // gradEnergy = (kappaTarget, 0), gradBarrier = (-1, 0) => suggested kappa =
  // -gradBarrier.dot(gradEnergy) / |gradBarrier|^2 = kappaTarget.
  const double kappaTarget = 2.0 * kappaMin; // inside [kappaMin, kappaMax]
  Eigen::VectorXd gradEnergy(2);
  gradEnergy << kappaTarget, 0.0;
  Eigen::VectorXd gradBarrier(2);
  gradBarrier << -1.0, 0.0;

  double maxStiffness = 0.0;
  const double kappa = expdetail::computeInitialRigidIpcBarrierStiffness(
      bbox,
      squaredActivationDistance,
      averageMass,
      gradEnergy,
      gradBarrier,
      scale,
      maxStiffness);

  EXPECT_NEAR(kappa, kappaTarget, kappaTarget * 1e-9);
  EXPECT_NEAR(maxStiffness, kappaMax, kappaMax * 1e-9);
}

TEST(RigidIpcAdaptiveStiffness, InitialStiffnessClampsToMinWhenBarrierInactive)
{
  const double bbox = 2.0;
  const double squaredActivationDistance = 1e-4;
  const double averageMass = 3.0;
  const double scale = 1e11;

  const double d0 = 1e-8 * bbox;
  const double d0Squared = d0 * d0;
  const double secondDerivative
      = dc::c2ClampedLogBarrier(d0Squared, squaredActivationDistance)
            .secondDerivative;
  const double kappaMin
      = scale * averageMass / (4.0 * d0Squared * secondDerivative);

  // Inactive barrier => zero barrier gradient => suggested kappa defaults to 1,
  // then clamps up to kappa_min (far above the old fixed kappa = 1).
  Eigen::VectorXd gradEnergy(3);
  gradEnergy << 1.0, 2.0, 3.0;
  const Eigen::VectorXd gradBarrier = Eigen::VectorXd::Zero(3);

  double maxStiffness = 0.0;
  const double kappa = expdetail::computeInitialRigidIpcBarrierStiffness(
      bbox,
      squaredActivationDistance,
      averageMass,
      gradEnergy,
      gradBarrier,
      scale,
      maxStiffness);

  EXPECT_NEAR(kappa, kappaMin, kappaMin * 1e-9);
  EXPECT_GT(kappa, 1.0);
  EXPECT_NEAR(maxStiffness, 100.0 * kappaMin, kappaMin * 1e-7);
}

TEST(RigidIpcAdaptiveStiffness, InitialStiffnessClampsToMaxForLargeRatio)
{
  const double bbox = 1.0;
  const double squaredActivationDistance = 1e-4;
  const double averageMass = 1.0;
  const double scale = 1e11;

  const double d0 = 1e-8 * bbox;
  const double d0Squared = d0 * d0;
  const double secondDerivative
      = dc::c2ClampedLogBarrier(d0Squared, squaredActivationDistance)
            .secondDerivative;
  const double kappaMax
      = 100.0 * scale * averageMass / (4.0 * d0Squared * secondDerivative);

  Eigen::VectorXd gradEnergy(1);
  gradEnergy << 1e12 * kappaMax;
  Eigen::VectorXd gradBarrier(1);
  gradBarrier << -1.0;

  double maxStiffness = 0.0;
  const double kappa = expdetail::computeInitialRigidIpcBarrierStiffness(
      bbox,
      squaredActivationDistance,
      averageMass,
      gradEnergy,
      gradBarrier,
      scale,
      maxStiffness);

  EXPECT_NEAR(kappa, kappaMax, kappaMax * 1e-9);
}

TEST(RigidIpcAdaptiveStiffness, InitialStiffnessRejectsInvalidInputs)
{
  Eigen::VectorXd gradient(1);
  gradient << 1.0;
  double maxStiffness = 0.0;

  EXPECT_EQ(
      expdetail::computeInitialRigidIpcBarrierStiffness(
          0.0, 1e-4, 1.0, gradient, gradient, 1e11, maxStiffness),
      1.0);
  EXPECT_FALSE(std::isfinite(maxStiffness));

  EXPECT_EQ(
      expdetail::computeInitialRigidIpcBarrierStiffness(
          1.0, 0.0, 1.0, gradient, gradient, 1e11, maxStiffness),
      1.0);
  EXPECT_EQ(
      expdetail::computeInitialRigidIpcBarrierStiffness(
          1.0, 1e-4, 0.0, gradient, gradient, 1e11, maxStiffness),
      1.0);
}

TEST(RigidIpcAdaptiveStiffness, UpdateDoublesWhenStrugglingToSeparate)
{
  const double bbox = 1.0;
  const double dhatEpsilonScale = 1e-3; // band = (1e-3)^2 squared distance
  const double epsilon = dhatEpsilonScale * bbox;
  const double bandSquared = epsilon * epsilon;
  const double inside = 0.25 * bandSquared;
  const double closer = 0.10 * bandSquared;

  // Inside the band and still approaching => doubled.
  EXPECT_NEAR(
      expdetail::updateRigidIpcBarrierStiffness(
          inside, closer, 1e6, 100.0, bbox, dhatEpsilonScale),
      200.0,
      1e-9);
  // Doubling is capped at maxStiffness.
  EXPECT_NEAR(
      expdetail::updateRigidIpcBarrierStiffness(
          inside, closer, 150.0, 100.0, bbox, dhatEpsilonScale),
      150.0,
      1e-9);
  // Receding => unchanged.
  EXPECT_NEAR(
      expdetail::updateRigidIpcBarrierStiffness(
          closer, inside, 1e6, 100.0, bbox, dhatEpsilonScale),
      100.0,
      1e-9);
  // Outside the band => unchanged even while approaching.
  EXPECT_NEAR(
      expdetail::updateRigidIpcBarrierStiffness(
          2.0 * bandSquared,
          1.5 * bandSquared,
          1e6,
          100.0,
          bbox,
          dhatEpsilonScale),
      100.0,
      1e-9);
}

TEST(RigidIpcAdaptiveStiffness, InitialStiffnessClampsProbeForOversizedWorld)
{
  // For a grossly mis-scaled world (bbox diagonal >= 1e6 * dhat) the raw probe
  // distance d0 = 1e-8 * bbox would land outside the activation band, where the
  // barrier second derivative is 0. The reference d0 clamp pulls it to an
  // interior point so a real adaptive kappa is still returned instead of the
  // kappa = 1 fallback.
  const double bbox = 2.0e6; // d0 = 2e-2, d0^2 = 4e-4 >= dhat^2 = 1e-4
  const double squaredActivationDistance = 1e-4;
  const double averageMass = 1.0;
  const double scale = 1e11;

  // Expected kappa_min uses the clamped probe distance 0.5 * dhat^2.
  const double clampedD0Squared = 0.5 * squaredActivationDistance;
  const double secondDerivative
      = dc::c2ClampedLogBarrier(clampedD0Squared, squaredActivationDistance)
            .secondDerivative;
  ASSERT_GT(secondDerivative, 0.0);
  const double kappaMin
      = scale * averageMass / (4.0 * clampedD0Squared * secondDerivative);

  // Inactive barrier => suggested kappa defaults to 1, clamped up to kappa_min.
  Eigen::VectorXd gradEnergy(1);
  gradEnergy << 5.0;
  const Eigen::VectorXd gradBarrier = Eigen::VectorXd::Zero(1);

  double maxStiffness = 0.0;
  const double kappa = expdetail::computeInitialRigidIpcBarrierStiffness(
      bbox,
      squaredActivationDistance,
      averageMass,
      gradEnergy,
      gradBarrier,
      scale,
      maxStiffness);

  EXPECT_NEAR(kappa, kappaMin, kappaMin * 1e-9);
  EXPECT_GT(kappa, 1.0); // did not fall back to the kappa = 1 default
  EXPECT_TRUE(std::isfinite(maxStiffness));
}
