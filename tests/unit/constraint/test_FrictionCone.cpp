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

#include "../../integration/AllocationCounting.hpp"
#include "FrictionRegressionCases.hpp"
#include "dart/constraint/detail/FrictionCone.hpp"
#include "dart/constraint/detail/FrictionRows.hpp"

#include <Eigen/Cholesky>
#include <Eigen/QR>
#include <gtest/gtest.h>

#include <algorithm>
#include <future>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

#include <cfenv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

using namespace dart::constraint::detail;

namespace {

using WideVector = Eigen::Matrix<long double, 3, 1>;
// Case 1481 of the certificate gate: the negative boundary-angle interval was
// too narrow for the former fast scan.
const Eigen::Matrix3d angleRegressionMatrix = [] {
  Eigen::Matrix3d H;
  H << 9.2262669053731354, -0.85140655213626348, 2.0162788383275716,
      -0.85140655213626348, 1.1266781846934588, -0.50746117462211937,
      2.0162788383275716, -0.50746117462211926, 3.8093325701017848;
  return H;
}();
const Eigen::Vector3d angleRegressionLinear(
    0.33890073956668498, 0.26636504987113141, 0.21106576606743932);

// Case 14806 of the same gate: the secular candidate fails certification at
// cond(H)=1e8, exercising the counted dense scan and its allocation contract.
const Eigen::Matrix3d boundaryFallbackMatrix = [] {
  Eigen::Matrix3d H;
  H << 49624365.732912913, -40338883.018506698, -29540346.2012682,
      -40338883.018506698, 32793684.558535863, 24008382.652795985,
      -29540346.201268196, 24008382.652795985, 17591950.708551209;
  return H;
}();
const Eigen::Vector3d boundaryFallbackLinear(
    -0.077530063162165985, 0.28456983999152358, -0.29484743180755263);
const FrictionCone boundaryFallbackCone{
    Eigen::Vector2d(1.0, 11.030526431268397), FrictionConeLaw::Ellipse};

// Independent certificate: normalize the original problem in extended
// precision, then check K, K* and complementarity without production helpers.
// Data-relative scales permit cancellation in H*lambda+c at cond(H)=1e8;
// each normalized inequality still uses the specified 1e-10 tolerance.
struct Certificate
{
  long double primal;
  long double dual;
  long double complementarity;

  long double worst() const
  {
    return std::max({primal, dual, complementarity});
  }
};

long double support(const WideVector& v, const FrictionCone& cone)
{
  const long double a = cone.mu[0] * v[1];
  const long double b = cone.mu[1] * v[2];
  return cone.law == FrictionConeLaw::Box ? std::abs(a) + std::abs(b)
                                          : std::hypot(a, b);
}

Certificate independentlyCertify(
    const Eigen::Matrix3d& H,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& impulse,
    const FrictionCone& cone,
    bool exact = false)
{
  const WideVector x = impulse.cast<long double>();
  const long double matrixScale = H.cwiseAbs().maxCoeff();
  const long double linearScale = c.cwiseAbs().maxCoeff();
  const long double scale = std::max(matrixScale, linearScale);
  Eigen::Matrix<long double, 3, 3> normalizedH = H.cast<long double>();
  WideVector normalizedC = c.cast<long double>();
  if (scale > 0) {
    normalizedH /= scale;
    normalizedC /= scale;
  }
  const WideVector product = normalizedH * x;
  WideVector dual = product + normalizedC;
  if (exact)
    dual[0] += support(dual, cone);
  long double a = 0, b = 0, zeroAxisViolation = 0;
  for (int i = 0; i < 2; ++i) {
    if (cone.mu[i] == 0)
      zeroAxisViolation = std::max(zeroAxisViolation, std::abs(x[i + 1]));
    else if (i == 0)
      a = x[1] / cone.mu[0];
    else
      b = x[2] / cone.mu[1];
  }
  const long double tangent = cone.law == FrictionConeLaw::Box
                                  ? std::max(std::abs(a), std::abs(b))
                                  : std::hypot(a, b);
  const WideVector data
      = normalizedH.cwiseAbs() * x.cwiseAbs() + normalizedC.cwiseAbs();
  const long double impulseScale = std::max(
      x.cwiseAbs().maxCoeff(), matrixScale > 0 ? linearScale / matrixScale : 0);
  const long double dualScale = data[0] + support(data, cone);
  const long double dotScale
      = x.cwiseAbs().dot(data)
        + (exact ? std::abs(x[0]) * support(data, cone) : 0);
  const auto relative = [](long double violation, long double denominator) {
    return denominator > 0  ? violation / denominator
           : violation == 0 ? 0
                            : std::numeric_limits<long double>::infinity();
  };
  return {
      relative(
          std::max({0.0L, -x[0], tangent - x[0], zeroAxisViolation}),
          impulseScale),
      relative(std::max(0.0L, support(dual, cone) - dual[0]), dualScale),
      relative(std::abs(x.dot(dual)), dotScale)};
}

Eigen::Matrix3d effectiveMatrix(
    const Eigen::Matrix3d& H, const LocalSolveResult& result)
{
  return H + result.regularization * Eigen::Matrix3d::Identity();
}

class ProblemBank
{
public:
  explicit ProblemBank(std::uint64_t seed) : engine(seed) {}

  double uniform(double low, double high)
  {
    return std::uniform_real_distribution<double>(low, high)(engine);
  }

  Eigen::Vector3d vector()
  {
    Eigen::Vector3d result;
    for (int i = 0; i < 3; ++i)
      result[i] = normal(engine);
    return result;
  }

  Eigen::Matrix3d gram(double floor = 0.05, bool mixed = false)
  {
    Eigen::Matrix3d J;
    for (int i = 0; i < 3; ++i)
      J.col(i) = vector();
    if (mixed) {
      constexpr double scales[] = {1, 1, 10, 0.1};
      for (int i = 0; i < 3; ++i)
        J.col(i) *= scales[static_cast<int>(uniform(0, 4))];
    }
    return J * J.transpose() + floor * Eigen::Matrix3d::Identity();
  }

  Eigen::Matrix3d conditioned(double condition)
  {
    Eigen::Matrix3d J;
    for (int i = 0; i < 3; ++i)
      J.col(i) = vector();
    const Eigen::Matrix3d Q = J.householderQr().householderQ();
    return Q * Eigen::Vector3d(1, std::sqrt(condition), condition).asDiagonal()
           * Q.transpose();
  }

  FrictionCone ellipse(double low = 0.2, double high = 1.2)
  {
    return {
        Eigen::Vector2d(uniform(low, high), uniform(low, high)),
        FrictionConeLaw::Ellipse};
  }

private:
  std::mt19937_64 engine;
  std::normal_distribution<double> normal;
};

Eigen::Vector3d pgsBox(
    const Eigen::Matrix3d& H,
    const Eigen::Vector3d& c,
    const FrictionCone& cone)
{
  Eigen::Vector3d x = Eigen::Vector3d::Zero();
  for (int sweep = 0; sweep < 20000; ++sweep) {
    const Eigen::Vector3d previous = x;
    for (int i = 0; i < 3; ++i) {
      const double next = x[i] - (H.row(i).dot(x) + c[i]) / H(i, i);
      x[i] = i == 0 ? std::max(0.0, next)
                    : std::clamp(
                        next, -cone.mu[i - 1] * x[0], cone.mu[i - 1] * x[0]);
    }
    if ((x - previous).norm() < 1e-14 * (1 + x.norm()))
      break;
  }
  return x;
}

} // namespace

TEST(FrictionCone, ProjectionDualityAndDeSaxce)
{
  ProblemBank bank(7);
  for (const auto law : {FrictionConeLaw::Ellipse, FrictionConeLaw::Box}) {
    for (int i = 0; i < 2000; ++i) {
      FrictionCone cone = bank.ellipse(0.05, 2);
      cone.law = law;
      const Eigen::Vector3d v = bank.vector();
      const Eigen::Vector3d x = projectCone(v, cone);
      EXPECT_LE(coneViolation(x, cone), 1e-10 * (1 + x.norm()));
      EXPECT_LE((projectCone(x, cone) - x).norm(), 1e-10 * (1 + x.norm()));
      // Moreau's orthogonal decomposition independently verifies Euclidean
      // projection, including anisotropic ellipses and the box dual.
      const Eigen::Vector3d dual = x - v;
      EXPECT_GE(dual[0] - support(dual.cast<long double>(), cone), -1e-10);
      EXPECT_NEAR(x.dot(dual), 0, 1e-10 * (1 + x.norm() * dual.norm()));
      const Eigen::Vector3d shifted = deSaxce(v, cone);
      EXPECT_DOUBLE_EQ(shifted[1], v[1]);
      EXPECT_DOUBLE_EQ(shifted[2], v[2]);
      EXPECT_NEAR(
          shifted[0] - v[0], support(v.cast<long double>(), cone), 1e-12);
    }
  }
}

TEST(FrictionCone, ClosedFormsAndCodexProblems)
{
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  for (const auto law : {FrictionConeLaw::Ellipse, FrictionConeLaw::Box}) {
    const FrictionCone cone{Eigen::Vector2d::Constant(0.5), law};
    const auto interior = solveConeQp(I, Eigen::Vector3d(-2, 0.1, -0.2), cone);
    ASSERT_TRUE(interior.certified);
    EXPECT_NEAR(
        (interior.impulse - Eigen::Vector3d(2, -0.1, 0.2)).norm(), 0, 1e-12);
    const auto apex = solveConeQp(I, Eigen::Vector3d(2, 1, -1), cone);
    ASSERT_TRUE(apex.certified);
    EXPECT_DOUBLE_EQ(apex.impulse.norm(), 0);
    const Eigen::Vector3d c(-1, 2, 0);
    const auto associated = solveConeQp(I, c, cone);
    const auto exact = solveExactContact(I, c, cone);
    ASSERT_TRUE(associated.certified);
    ASSERT_TRUE(exact.certified);
    EXPECT_NEAR(
        (associated.impulse - Eigen::Vector3d(1.6, -0.8, 0)).norm(), 0, 1e-10);
    EXPECT_NEAR((exact.impulse - Eigen::Vector3d(1, -0.5, 0)).norm(), 0, 1e-10);
    EXPECT_LT(
        contactViolation(exact.impulse, I * exact.impulse + c, 1, cone), 1e-9);
    EXPECT_GT(
        contactViolation(
            associated.impulse, I * associated.impulse + c, 1, cone),
        0.1);
    EXPECT_LT(
        contactViolation(
            associated.impulse, I * associated.impulse + c, 1, cone, true),
        1e-9);
  }
  // Codex's cycling frozen QP: the underestimated step oscillated at 100/101
  // iterations; its certified exact minimizer is (1/102,-1/102,0).
  Eigen::Matrix3d H;
  H << 52.5, -49.5, 0, -49.5, 52.5, 0, 0, 0, 3;
  const FrictionCone cone;
  const Eigen::Vector3d c(-1, 1, 0);
  const auto result = solveConeQp(H, c, cone);
  ASSERT_TRUE(result.certified);
  EXPECT_NEAR(
      (result.impulse - Eigen::Vector3d(1.0 / 102, -1.0 / 102, 0)).norm(),
      0,
      1e-12);
  EXPECT_LE(independentlyCertify(H, c, result.impulse, cone).worst(), 1e-10L);
  for (double mu : {0.0, 1e-12, 1e-9, 1e-8, 1e-7, 2e-7, 1e-6, 1e-3, 0.5}) {
    const FrictionCone tiny{
        Eigen::Vector2d::Constant(mu), FrictionConeLaw::Ellipse};
    const auto local = solveExactContact(I, Eigen::Vector3d(-1, 2, 0), tiny);
    ASSERT_TRUE(local.certified) << mu;
    EXPECT_NEAR(local.impulse[0], 1, 1e-10);
    EXPECT_NEAR(local.impulse[1], -mu, 1e-10);
  }
}

TEST(FrictionCone, OneDimensionalWedgesAndVelocityUnits)
{
  const Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d c(-1, 2, 3);
  for (auto law : {FrictionConeLaw::Ellipse, FrictionConeLaw::Box}) {
    for (int axis = 0; axis < 2; ++axis) {
      FrictionCone cone{Eigen::Vector2d::Zero(), law};
      cone.mu[axis] = 0.5;
      const auto qp = solveConeQp(H, c, cone);
      const auto exact = solveExactContact(H, c, cone);
      ASSERT_TRUE(qp.certified);
      ASSERT_TRUE(exact.certified);
      Eigen::Vector3d expected = Eigen::Vector3d::Zero();
      expected[0] = (1 + 0.5 * c[axis + 1]) / 1.25;
      expected[axis + 1] = -0.5 * expected[0];
      EXPECT_LE((qp.impulse - expected).norm(), 1e-12);
      expected[0] = 1;
      expected[axis + 1] = -0.5;
      EXPECT_LE((exact.impulse - expected).norm(), 1e-10);
      EXPECT_LE(
          independentlyCertify(H, c, exact.impulse, cone, true).worst(),
          1e-10L);
      EXPECT_NEAR(
          contactViolation(
              Eigen::Vector3d::Zero(), Eigen::Vector3d(-1, 0, 0), 3, cone),
          1,
          1e-12);
    }
  }
}

TEST(FrictionCone, BoundaryAcrossPositiveSecularPole)
{
  const Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
  const FrictionCone cone;
  // The positive generalized eigenvalue is 1. These solutions have KKT
  // multipliers 1/3, 1 (the singular hard case), and 3, respectively.
  for (double normal : {-1.0, 0.0, 1.0}) {
    const Eigen::Vector3d c(normal, 2.0, 0.0);
    std::feclearexcept(FE_INVALID);
    const auto result = solveConeQp(H, c, cone);
    EXPECT_EQ(std::fetestexcept(FE_INVALID), 0);
    const double impulse = (2.0 - normal) / 2.0;
    ASSERT_TRUE(result.certified);
    EXPECT_LE(
        (result.impulse - Eigen::Vector3d(impulse, -impulse, 0.0)).norm(),
        1e-12);
    EXPECT_EQ(result.numLocalFallbacks, 0u);
    EXPECT_LE(independentlyCertify(H, c, result.impulse, cone).worst(), 1e-10L);
  }
}

TEST(FrictionCone, OpeningExactContactUsesNoQp)
{
  const Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
  for (const auto law : {FrictionConeLaw::Ellipse, FrictionConeLaw::Box}) {
    const FrictionCone cone{Eigen::Vector2d(0.5, 0.3), law};
    for (double normal : {0.0, 1.0}) {
      const Eigen::Vector3d c(normal, 2.0, -3.0);
      for (double warm : {0.0, 5.0}) {
        const auto result = solveExactContact(H, c, cone, warm);
        ASSERT_TRUE(result.certified);
        EXPECT_TRUE(result.impulse.isZero());
        EXPECT_EQ(result.numQpSolves, 0u);
        EXPECT_EQ(result.numLocalFallbacks, 0u);
        EXPECT_DOUBLE_EQ(
            result.normalShift,
            static_cast<double>(support(c.cast<long double>(), cone)));
        EXPECT_LE(
            independentlyCertify(H, c, result.impulse, cone, true).worst(),
            1e-10L);
      }
    }
  }
}

TEST(FrictionCone, RejectsRadialNewtonBias)
{
  Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
  H.bottomRightCorner<2, 2>() << 2, 1, 1, 8;
  const Eigen::Vector3d c(-1, 2, 3);
  const FrictionCone cone{
      Eigen::Vector2d::Constant(0.5), FrictionConeLaw::Ellipse};
  Eigen::Vector3d radial;
  radial[0] = 1;
  radial.tail<2>() = -H.bottomRightCorner<2, 2>().ldlt().solve(c.tail<2>());
  radial.tail<2>() *= 0.5 / radial.tail<2>().norm();
  EXPECT_GT(independentlyCertify(H, c, radial, cone, true).worst(), 1e-3L);
  EXPECT_GT(contactViolation(radial, H * radial + c, 8, cone), 0.01);
  const auto result = solveExactContact(H, c, cone);
  ASSERT_TRUE(result.certified);
  EXPECT_LE(
      independentlyCertify(H, c, result.impulse, cone, true).worst(), 1e-10L);
}

TEST(FrictionCone, OriginalBoundaryRegressionFixtures)
{
  for (const auto& fixture : frictionRegressionCases) {
    SCOPED_TRACE(fixture.name);
    const Eigen::Matrix3d H
        = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            fixture.matrix);
    const Eigen::Vector3d c = Eigen::Map<const Eigen::Vector3d>(fixture.linear);
    const Eigen::Vector3d expected
        = Eigen::Map<const Eigen::Vector3d>(fixture.expected);
    const FrictionCone cone{
        Eigen::Map<const Eigen::Vector2d>(fixture.mu),
        FrictionConeLaw::Ellipse};
    const auto result = solveConeQp(H, c, cone);
    ASSERT_TRUE(result.certified);
    EXPECT_LE(
        independentlyCertify(
            effectiveMatrix(H, result), c, result.impulse, cone)
            .worst(),
        1e-10L);
    const auto objective = [&](const Eigen::Vector3d& x) {
      return 0.5 * x.dot(H * x) + c.dot(x);
    };
    EXPECT_NEAR(
        objective(result.impulse),
        objective(expected),
        1e-9 * (1 + std::abs(objective(expected))));
  }
}

TEST(FrictionCone, OriginalSplitCyclesAndWorstRadialBias)
{
  for (const auto& fixture : frictionExactRegressionCases) {
    SCOPED_TRACE(fixture.name);
    const Eigen::Matrix3d H
        = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            fixture.matrix);
    const Eigen::Vector3d c = Eigen::Map<const Eigen::Vector3d>(fixture.linear);
    const Eigen::Vector3d expected
        = Eigen::Map<const Eigen::Vector3d>(fixture.expected);
    const FrictionCone cone{
        Eigen::Map<const Eigen::Vector2d>(fixture.mu),
        FrictionConeLaw::Ellipse};
    const auto result = solveExactContact(H, c, cone);
    ASSERT_TRUE(result.certified);
    EXPECT_LE(
        independentlyCertify(
            effectiveMatrix(H, result), c, result.impulse, cone, true)
            .worst(),
        1e-10L);
    EXPECT_LE((result.impulse - expected).norm(), 1e-9 * (1 + expected.norm()));
    EXPECT_LT(
        contactViolation(
            result.impulse,
            H * result.impulse + c,
            H.diagonal().maxCoeff(),
            cone),
        1e-8);
  }
}

TEST(FrictionCone, CertifiedFallbackIsCounted)
{
  const auto angle = solveConeQp(
      angleRegressionMatrix, angleRegressionLinear, FrictionCone{});
  ASSERT_TRUE(angle.certified);
  EXPECT_LE(
      independentlyCertify(
          angleRegressionMatrix,
          angleRegressionLinear,
          angle.impulse,
          FrictionCone{})
          .worst(),
      1e-10L);
  const auto& cone = boundaryFallbackCone;
  const auto result
      = solveConeQp(boundaryFallbackMatrix, boundaryFallbackLinear, cone);
  ASSERT_TRUE(result.certified);
  EXPECT_EQ(result.numQpSolves, 1u);
  EXPECT_GT(result.numLocalFallbacks, 0u);
  EXPECT_LE(
      independentlyCertify(
          boundaryFallbackMatrix, boundaryFallbackLinear, result.impulse, cone)
          .worst(),
      1e-10L);
}

TEST(FrictionCone, DesignCheckProblemBanks)
{
  // Recreate the distributions and sizes of check_design_math.py (7),
  // critique_checks.py (3), and final_checks.py (11) with mt19937_64.
  // Original NumPy cases exposing narrow minima are preserved above verbatim.
  ProblemBank bank(7);
  for (int i = 0; i < 360; ++i) {
    const Eigen::Matrix3d H = bank.gram();
    FrictionCone cone = bank.ellipse();
    if (i >= 300)
      cone.mu[1] = i % 3 == 0 ? 1e-7 : i % 3 == 1 ? 1e-8 : 1e-12;
    Eigen::Vector3d c = 3 * bank.vector();
    c[0] = -bank.uniform(0.5, 2);
    const auto result = solveExactContact(H, c, cone);
    ASSERT_TRUE(result.certified) << i;
    EXPECT_LE(
        independentlyCertify(
            effectiveMatrix(H, result), c, result.impulse, cone, true)
            .worst(),
        1e-10L)
        << i;
    const auto warm = solveExactContact(H, c, cone, result.normalShift);
    ASSERT_TRUE(warm.certified);
    EXPECT_LE(
        (warm.impulse - result.impulse).norm(),
        1e-8 * (1 + result.impulse.norm()));
  }
  ProblemBank boxBank(3);
  for (int i = 0; i < 200; ++i) {
    const Eigen::Matrix3d H = boxBank.gram(0.5);
    Eigen::Vector3d c = 2 * boxBank.vector();
    c[0] = -boxBank.uniform(0.05, 1);
    const FrictionCone cone{Eigen::Vector2d(0.5, 0.3), FrictionConeLaw::Box};
    const auto result = solveExactContact(H, c, cone);
    ASSERT_TRUE(result.certified) << i;
    EXPECT_LE(
        independentlyCertify(
            effectiveMatrix(H, result), c, result.impulse, cone, true)
            .worst(),
        1e-10L)
        << i;
    EXPECT_LE(
        (pgsBox(H, c, cone) - result.impulse).norm(),
        1e-8 * (1 + result.impulse.norm()))
        << i;
  }
  ProblemBank finalBank(11);
  for (int i = 0; i < 8000; ++i) {
    const bool mixed = i >= 2000;
    const Eigen::Matrix3d H = finalBank.gram(mixed ? 1e-3 : 0.05, mixed);
    const FrictionCone cone
        = mixed ? finalBank.ellipse(0.05, 2) : finalBank.ellipse();
    Eigen::Vector3d c = 3 * finalBank.vector();
    c[0] = finalBank.uniform(-2, 0.5);
    const auto result = solveConeQp(H, c, cone);
    ASSERT_TRUE(result.certified) << i;
    EXPECT_LE(
        independentlyCertify(
            effectiveMatrix(H, result), c, result.impulse, cone)
            .worst(),
        1e-10L)
        << i;
  }
  for (int i = 0; i < 600; ++i) {
    const Eigen::Matrix3d H
        = i >= 300 && i < 450 ? Eigen::Matrix3d(
              Eigen::Matrix3d::Identity() * finalBank.uniform(0.5, 2))
                              : finalBank.gram();
    const FrictionCone cone
        = i < 300 ? finalBank.ellipse()
                  : FrictionCone{
                      Eigen::Vector2d::Constant(0.5), FrictionConeLaw::Ellipse};
    Eigen::Vector3d c = 3 * finalBank.vector();
    c[0] = -finalBank.uniform(0.5, 2);
    const auto result = solveExactContact(H, c, cone);
    ASSERT_TRUE(result.certified) << "final E/F case=" << i;
    EXPECT_LE(
        independentlyCertify(
            effectiveMatrix(H, result), c, result.impulse, cone, true)
            .worst(),
        1e-10L)
        << i;
    if (i >= 300 && i < 450) {
      const double normal = -c[0] / H(0, 0);
      Eigen::Vector3d expected = -c / H(0, 0);
      if (expected.tail<2>().norm() > cone.mu[0] * normal)
        expected.tail<2>() *= cone.mu[0] * normal / expected.tail<2>().norm();
      EXPECT_LE(
          (result.impulse - expected).norm(), 1e-9 * (1 + expected.norm()));
    }
  }
}

TEST(FrictionCone, RandomizedCertificateGate)
{
  // Full acceptance gate:
  // DART_FRICTION_GATE_COUNT=100000
  // ./build/default/cpp/Release/tests/unit/constraint/test_FrictionCone
  //   --gtest_filter=FrictionCone.RandomizedCertificateGate
  // Default 2000 keeps CI inexpensive; both counts use the same seed/prefix.
  const char* requested = std::getenv("DART_FRICTION_GATE_COUNT");
  const std::uint64_t count
      = requested ? std::strtoull(requested, nullptr, 10) : 2000;
  ASSERT_GT(count, 0u);
  ProblemBank bank(0x6D61727466726963ULL);
  std::uint64_t fallbackProblems = 0, fallbacks = 0;
  long double worst = 0;
  for (std::uint64_t i = 0; i < count; ++i) {
    const double condition = std::pow(10.0, 0.5 * (i % 17));
    const Eigen::Matrix3d H = bank.conditioned(condition);
    FrictionCone cone;
    const double mu = std::pow(10.0, -12.0 + static_cast<double>(i % 13));
    const double ratio = i % 37 == 0 ? 1e3 : std::pow(10.0, bank.uniform(0, 3));
    cone.mu = Eigen::Vector2d(mu, mu * ratio);
    if (i % 4 == 0)
      cone.law = FrictionConeLaw::Box;
    else if (i % 4 == 1)
      cone.mu[1] = cone.mu[0];
    if (i % 19 == 0)
      cone.mu = Eigen::Vector2d(0.7, 150);
    if (i % 31 == 0)
      std::swap(cone.mu[0], cone.mu[1]);
    const Eigen::Vector3d c = bank.vector();
    const auto result = solveConeQp(H, c, cone);
    ASSERT_TRUE(result.certified) << "case=" << i << " cond=" << condition
                                  << " mu=" << cone.mu.transpose() << " H=\n"
                                  << H << " c=" << c.transpose();
    const auto certificate = independentlyCertify(
        effectiveMatrix(H, result), c, result.impulse, cone);
    ASSERT_LE(certificate.worst(), 1e-10L)
        << "case=" << i << " primal=" << certificate.primal
        << " dual=" << certificate.dual
        << " complementarity=" << certificate.complementarity;
    ASSERT_TRUE(
        coneQpCertificate(effectiveMatrix(H, result), c, result.impulse, cone));
    worst = std::max(worst, certificate.worst());
    fallbackProblems += result.numLocalFallbacks != 0;
    fallbacks += result.numLocalFallbacks;
  }
  std::cout << "Friction certificate gate: problems=" << count
            << " fast_path_failures=" << fallbackProblems
            << " fast_path_failure_rate="
            << static_cast<double>(fallbackProblems) / count
            << " local_fallbacks=" << fallbacks
            << " worst_independent_certificate=" << static_cast<double>(worst)
            << '\n';
}

TEST(FrictionCone, IdenticalResultsAcrossThreads)
{
  Eigen::Matrix3d H;
  H << 3, 0.2, -0.4, 0.2, 2, 0.1, -0.4, 0.1, 1;
  const Eigen::Vector3d c(-1, 2, -3);
  const FrictionCone cone{Eigen::Vector2d(0.7, 1.2), FrictionConeLaw::Ellipse};
  const auto expected = solveExactContact(H, c, cone);
  ASSERT_TRUE(expected.certified);
  std::vector<std::future<LocalSolveResult>> results;
  for (int i = 0; i < 8; ++i)
    results.push_back(std::async(
        std::launch::async, [=] { return solveExactContact(H, c, cone); }));
  for (auto& future : results) {
    const auto result = future.get();
    ASSERT_TRUE(result.certified);
    EXPECT_EQ(
        std::memcmp(
            result.impulse.data(), expected.impulse.data(), 3 * sizeof(double)),
        0);
    EXPECT_EQ(result.numLocalFallbacks, expected.numLocalFallbacks);
    EXPECT_EQ(result.numQpSolves, expected.numQpSolves);
    EXPECT_DOUBLE_EQ(result.normalShift, expected.normalShift);
    EXPECT_DOUBLE_EQ(result.regularization, expected.regularization);
  }
}

TEST(FrictionCone, CertificateRequiresSymmetricPositiveSemidefiniteBlock)
{
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  // Its determinant is only -6.15e-13, but its negative eigenvalue is
  // -7.01e-7: absolute tolerances on principal minors accept this block.
  Eigen::Matrix3d slightlyIndefinite;
  slightlyIndefinite << 4.74210068e-4, 2.17760294e-2, 2.66206375e-3,
      2.17760294e-2, 1, 1.22283510e-1, 2.66206375e-3, 1.22283510e-1,
      1.49534025e-2;
  for (const auto law : {FrictionConeLaw::Ellipse, FrictionConeLaw::Box}) {
    const FrictionCone cone{Eigen::Vector2d::Ones(), law};
    for (const double scale : {1e-20, 1.0, 1e20}) {
      SCOPED_TRACE(scale);
      const Eigen::Vector3d c(scale, 0, 0);
      // At the apex the first-order conditions hold, but -I is unbounded
      // below along the normal ray.
      EXPECT_FALSE(coneQpCertificate(
          -scale * Eigen::Matrix3d::Identity(), c, zero, cone));
      Eigen::Matrix3d indefinite
          = scale * Eigen::Vector3d(-1, 1, 1).asDiagonal();
      EXPECT_FALSE(coneQpCertificate(indefinite, c, zero, cone));
      indefinite.setZero();
      indefinite(0, 1) = indefinite(1, 0) = scale;
      EXPECT_FALSE(coneQpCertificate(indefinite, c, zero, cone));
      EXPECT_FALSE(
          coneQpCertificate(scale * slightlyIndefinite, c, zero, cone));
      Eigen::Matrix3d asymmetric = scale * Eigen::Matrix3d::Identity();
      asymmetric(0, 1) = scale;
      EXPECT_FALSE(coneQpCertificate(asymmetric, c, zero, cone));

      const Eigen::Matrix3d singular
          = scale * Eigen::Vector3d(1, 0, 0).asDiagonal();
      const Eigen::Vector3d optimum(1, 0.5, -0.5);
      // No regularization: the free tangential directions remain minimizers.
      EXPECT_TRUE(coneQpCertificate(singular, -c, optimum, cone));
      EXPECT_TRUE(coneQpCertificate(singular, c, zero, cone));
    }
    EXPECT_TRUE(coneQpCertificate(
        Eigen::Matrix3d::Zero(), zero, Eigen::Vector3d(1, 0.5, -0.5), cone));
    // Common H/c scaling must not erase an indefinite H when c is huge.
    const Eigen::Matrix3d negativeSubnormal
        = -std::numeric_limits<double>::denorm_min()
          * Eigen::Matrix3d::Identity();
    const Eigen::Vector3d hugeLinear(std::numeric_limits<double>::max(), 0, 0);
    EXPECT_FALSE(coneQpCertificate(negativeSubnormal, hugeLinear, zero, cone));
    EXPECT_FALSE(solveConeQp(negativeSubnormal, hugeLinear, cone).certified);
    EXPECT_FALSE(
        solveExactContact(negativeSubnormal, hugeLinear, cone).certified);
  }
}

TEST(FrictionCone, LocalSolvesPreserveMinimizersAtExtremeObjectiveScales)
{
  const Eigen::Vector3d expected(1, 0, 0);
  for (const auto law : {FrictionConeLaw::Ellipse, FrictionConeLaw::Box}) {
    const FrictionCone cone{Eigen::Vector2d::Ones(), law};
    for (const double scale :
         {6e307,
          1e-300,
          std::numeric_limits<double>::max(),
          std::numeric_limits<double>::min(),
          std::numeric_limits<double>::denorm_min()}) {
      SCOPED_TRACE(scale);
      const Eigen::Matrix3d H = scale * Eigen::Matrix3d::Identity();
      const Eigen::Vector3d c(-scale, 0, 0);
      // Even an unregularized trace overflows for 6e307 * I. At the other
      // endpoint, squared objectives and absolute pivot thresholds underflow.
      for (const auto& result :
           {solveConeQp(H, c, cone), solveExactContact(H, c, cone)}) {
        ASSERT_TRUE(result.certified);
        EXPECT_LE((result.impulse - expected).norm(), 1e-12);
        EXPECT_DOUBLE_EQ(result.regularization, 0);
        EXPECT_DOUBLE_EQ(result.normalShift, 0);
        EXPECT_TRUE(coneQpCertificate(H, c, result.impulse, cone));
        EXPECT_LE(
            independentlyCertify(H, c, result.impulse, cone).worst(), 1e-10L);
      }
    }
  }
}

TEST(FrictionCone, ExtremeObjectiveScalesPreserveNormalShiftUnitsAndWarmStarts)
{
  for (const auto law : {FrictionConeLaw::Ellipse, FrictionConeLaw::Box}) {
    const FrictionCone cone{Eigen::Vector2d::Constant(0.5), law};
    for (const double scale : {1.0, 6e307, 1e-300}) {
      SCOPED_TRACE(scale);
      const Eigen::Matrix3d H = scale * Eigen::Matrix3d::Identity();
      const Eigen::Vector3d c(-scale, 2 * scale, 0);
      const auto associated = solveConeQp(H, c, cone);
      ASSERT_TRUE(associated.certified);
      EXPECT_LE(
          (associated.impulse - Eigen::Vector3d(1.6, -0.8, 0)).norm(), 1e-10);
      const auto exact = solveExactContact(H, c, cone);
      ASSERT_TRUE(exact.certified);
      EXPECT_LE((exact.impulse - Eigen::Vector3d(1, -0.5, 0)).norm(), 1e-10);
      EXPECT_NEAR(exact.normalShift / scale, 0.75, 1e-10);
      EXPECT_LE(
          independentlyCertify(H, c, exact.impulse, cone, true).worst(),
          1e-10L);
      // The exact warm shift should certify immediately after bracketing;
      // this also detects an input shift left in the caller's units.
      const auto warm = solveExactContact(H, c, cone, 0.75 * scale);
      ASSERT_TRUE(warm.certified);
      EXPECT_EQ(warm.numQpSolves, 3u);
      EXPECT_NEAR(warm.normalShift / scale, 0.75, 1e-10);
      EXPECT_LE((warm.impulse - exact.impulse).norm(), 1e-10);
      if (scale != 1) {
        // An optional shift outside the scaled range must still permit the
        // cold solve, including overflow when scaling a huge warm shift up.
        const double mismatchedShift = scale < 1 ? 1e300 : 1e-300;
        const auto mismatched = solveExactContact(H, c, cone, mismatchedShift);
        ASSERT_TRUE(mismatched.certified);
        EXPECT_LE((mismatched.impulse - exact.impulse).norm(), 1e-10);
        EXPECT_NEAR(mismatched.normalShift / scale, 0.75, 1e-10);
      }
    }
  }
}

TEST(FrictionCone, ExtremeObjectiveScalesPreserveRegularizationUnits)
{
  const Eigen::Matrix3d singular = Eigen::Vector3d(1, 1, 0).asDiagonal();
  const Eigen::Vector3d c(-1, 2, 0);
  for (const auto law : {FrictionConeLaw::Ellipse, FrictionConeLaw::Box}) {
    // Adding the required regularization to these diagonal entries would
    // overflow the effective matrix exposed to the caller.
    const double largest = std::numeric_limits<double>::max();
    const FrictionCone unitCone{Eigen::Vector2d::Ones(), law};
    const Eigen::Matrix3d largestSingular = largest * singular;
    const Eigen::Vector3d largestLinear(-largest, 0, 0);
    EXPECT_FALSE(
        solveConeQp(largestSingular, largestLinear, unitCone).certified);
    EXPECT_FALSE(
        solveExactContact(largestSingular, largestLinear, unitCone).certified);
    const FrictionCone cone{Eigen::Vector2d::Constant(0.5), law};
    const auto ordinaryQp = solveConeQp(singular, c, cone);
    const auto ordinaryExact = solveExactContact(singular, c, cone);
    ASSERT_TRUE(ordinaryQp.certified);
    ASSERT_TRUE(ordinaryExact.certified);
    for (const double scale : {6e307, 1e-300}) {
      SCOPED_TRACE(scale);
      const Eigen::Matrix3d H = scale * singular;
      const Eigen::Vector3d linear = scale * c;
      const auto qp = solveConeQp(H, linear, cone);
      ASSERT_TRUE(qp.certified);
      EXPECT_NEAR(qp.regularization / scale, 2e-12, 1e-23);
      EXPECT_LE((qp.impulse - ordinaryQp.impulse).norm(), 1e-10);
      EXPECT_TRUE(
          coneQpCertificate(effectiveMatrix(H, qp), linear, qp.impulse, cone));
      EXPECT_LE(
          independentlyCertify(effectiveMatrix(H, qp), linear, qp.impulse, cone)
              .worst(),
          1e-10L);
      const auto exact = solveExactContact(H, linear, cone);
      ASSERT_TRUE(exact.certified);
      EXPECT_NEAR(exact.regularization / scale, 2e-12, 1e-23);
      EXPECT_LE((exact.impulse - ordinaryExact.impulse).norm(), 1e-10);
      EXPECT_NEAR(exact.normalShift / scale, ordinaryExact.normalShift, 1e-10);
      EXPECT_LE(
          independentlyCertify(
              effectiveMatrix(H, exact), linear, exact.impulse, cone, true)
              .worst(),
          1e-10L);
    }
    // The required diagonal shift itself is below the double range here;
    // round it up and solve with the same effective H exposed to the caller.
    const double smallest = std::numeric_limits<double>::denorm_min();
    const Eigen::Matrix3d H = smallest * singular;
    const Eigen::Vector3d linear(-smallest, 0, 0);
    for (const auto& result :
         {solveConeQp(H, linear, cone), solveExactContact(H, linear, cone)}) {
      ASSERT_TRUE(result.certified);
      EXPECT_DOUBLE_EQ(result.regularization, smallest);
      EXPECT_LE((result.impulse - Eigen::Vector3d(0.5, 0, 0)).norm(), 1e-12);
      EXPECT_TRUE(coneQpCertificate(
          effectiveMatrix(H, result), linear, result.impulse, cone));
      EXPECT_LE(
          independentlyCertify(
              effectiveMatrix(H, result), linear, result.impulse, cone)
              .worst(),
          1e-10L);
    }
  }
}

TEST(FrictionCone, AllZeroBlocksPreserveFixedRegularizationAtExtremeScales)
{
  const Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
  const FrictionCone cone{
      Eigen::Vector2d::Constant(0.5), FrictionConeLaw::Ellipse};
  const double smallest = std::numeric_limits<double>::denorm_min();
  const Eigen::Vector3d tinyLinear(-smallest, 0, 0);
  const double expected = smallest / 1e-12;
  for (const auto& result :
       {solveConeQp(H, tinyLinear, cone),
        solveExactContact(H, tinyLinear, cone)}) {
    ASSERT_TRUE(result.certified);
    EXPECT_DOUBLE_EQ(result.regularization, 1e-12);
    EXPECT_NEAR(result.impulse[0] / expected, 1, 1e-10);
    EXPECT_TRUE(result.impulse.tail<2>().isZero());
    EXPECT_TRUE(coneQpCertificate(
        effectiveMatrix(H, result), tinyLinear, result.impulse, cone));
  }
  // The fixed zero-block regularization also leaves an opening apex valid
  // when max|c| / max|H_effective| exceeds the double range.
  const Eigen::Vector3d hugeLinear(6e307, 6e307, 0);
  const auto qp = solveConeQp(H, hugeLinear, cone);
  ASSERT_TRUE(qp.certified);
  EXPECT_TRUE(qp.impulse.isZero());
  EXPECT_DOUBLE_EQ(qp.regularization, 1e-12);
  EXPECT_TRUE(
      coneQpCertificate(effectiveMatrix(H, qp), hugeLinear, qp.impulse, cone));
  const auto exact = solveExactContact(H, hugeLinear, cone);
  ASSERT_TRUE(exact.certified);
  EXPECT_TRUE(exact.impulse.isZero());
  EXPECT_DOUBLE_EQ(exact.regularization, 1e-12);
  EXPECT_DOUBLE_EQ(exact.normalShift, 3e307);
}

TEST(FrictionCone, UnrepresentableCommonScaleCannotProduceFalseCertificates)
{
  // Overflow of max|c| / max|H| must not hide positive complementarity.
  EXPECT_FALSE(coneQpCertificate(
      1e-320 * Eigen::Matrix3d::Identity(),
      Eigen::Vector3d(1, 0, 0),
      Eigen::Vector3d(1, 0, 0),
      FrictionCone{}));
  // The enormous linear term lies on a disabled tangent. Losing the tiny
  // normal H and c during common scaling would falsely certify the apex.
  const Eigen::Matrix3d H = 1e-300 * Eigen::Matrix3d::Identity();
  const Eigen::Vector3d c(-1e-300, 1e300, 0);
  const FrictionCone cone{Eigen::Vector2d(0, 1), FrictionConeLaw::Ellipse};
  EXPECT_FALSE(solveConeQp(H, c, cone).certified);
  EXPECT_FALSE(solveExactContact(H, c, cone).certified);
  EXPECT_FALSE(coneQpCertificate(H, c, Eigen::Vector3d::Zero(), cone));
  const Eigen::Matrix3d zeroH = Eigen::Matrix3d::Zero();
  const double smallest = std::numeric_limits<double>::denorm_min();
  const double largest = std::numeric_limits<double>::max();
  const Eigen::Vector3d mixedLinear(-smallest, largest, 0);
  EXPECT_FALSE(solveConeQp(zeroH, mixedLinear, cone).certified);
  EXPECT_FALSE(solveExactContact(zeroH, mixedLinear, cone).certified);
  EXPECT_FALSE(
      coneQpCertificate(zeroH, mixedLinear, Eigen::Vector3d::Zero(), cone));
  EXPECT_FALSE(coneQpCertificate(
      zeroH,
      Eigen::Vector3d(smallest, largest, 0),
      Eigen::Vector3d(1, 0, 0),
      cone));
}

TEST(FrictionCone, CertificateIsRelativeToProblemAndImpulseScales)
{
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  const Eigen::Vector3d unitNormal(1, 0, 0);
  for (const auto law : {FrictionConeLaw::Ellipse, FrictionConeLaw::Box}) {
    const FrictionCone cone{Eigen::Vector2d::Ones(), law};
    for (const double scale : {1e-20, 1.0, 1e20}) {
      SCOPED_TRACE(scale);
      const Eigen::Matrix3d H = scale * I;
      const Eigen::Vector3d c(-scale, 0, 0);
      EXPECT_FALSE(coneQpCertificate(H, c, zero, cone));
      EXPECT_TRUE(coneQpCertificate(H, c, unitNormal, cone));
      EXPECT_GT(independentlyCertify(H, c, zero, cone).worst(), 1e-10L);
      EXPECT_EQ(independentlyCertify(H, c, unitNormal, cone).worst(), 0);

      // The internal certificate also applies to both interior and boundary
      // solutions after the same uniform objective scaling.
      for (const Eigen::Vector3d& linear :
           {c, Eigen::Vector3d(-scale, 2 * scale, 0)}) {
        const auto result = solveConeQp(H, linear, cone);
        ASSERT_TRUE(result.certified);
        EXPECT_TRUE(coneQpCertificate(
            effectiveMatrix(H, result), linear, result.impulse, cone));
        EXPECT_LE(
            independentlyCertify(
                effectiveMatrix(H, result), linear, result.impulse, cone)
                .worst(),
            1e-10L);
        const Eigen::Vector3d expected
            = linear[1] == 0 ? unitNormal : Eigen::Vector3d(1.5, -1.5, 0);
        EXPECT_LE((result.impulse - expected).norm(), 1e-10);
      }
    }
    for (const double size : {1e-200, 1e-20, 1.0, 1e20, 1e200}) {
      SCOPED_TRACE(size);
      const Eigen::Vector3d optimum(size, 0, 0);
      const Eigen::Vector3d c = -optimum;
      EXPECT_TRUE(coneQpCertificate(I, c, optimum, cone));
      EXPECT_FALSE(coneQpCertificate(I, c, zero, cone));
      EXPECT_FALSE(coneQpCertificate(I, c, (1 - 1e-6) * optimum, cone));
      EXPECT_FALSE(coneQpCertificate(I, c, (1 + 1e-6) * optimum, cone));
      EXPECT_TRUE(coneQpCertificate(I, c, (1 - 1e-12) * optimum, cone));
      EXPECT_TRUE(coneQpCertificate(I, c, (1 + 1e-12) * optimum, cone));

      // Isolate primal infeasibility with zero gradient and complementarity.
      const Eigen::Vector3d infeasible(size, 2 * size, 0);
      EXPECT_FALSE(coneQpCertificate(I, -infeasible, infeasible, cone));
      // Isolate complementarity on a primal- and dual-feasible normal ray.
      EXPECT_FALSE(coneQpCertificate(I, zero, optimum, cone));
    }
  }
}

TEST(FrictionCone, SingularRegularizationInvalidInputAndAllocation)
{
  Eigen::Matrix3d singular = Eigen::Matrix3d::Identity();
  singular(2, 2) = 0;
  const Eigen::Vector3d c(-1, 2, 3);
  const FrictionCone cone{Eigen::Vector2d(0.5, 0.3), FrictionConeLaw::Ellipse};
  const auto regularized = solveConeQp(singular, c, cone);
  ASSERT_TRUE(regularized.certified);
  EXPECT_DOUBLE_EQ(regularized.regularization, 2e-12);
  EXPECT_LE(
      independentlyCertify(
          effectiveMatrix(singular, regularized), c, regularized.impulse, cone)
          .worst(),
      1e-10L);
  const auto zero = solveConeQp(Eigen::Matrix3d::Zero(), c, cone);
  ASSERT_TRUE(zero.certified);
  EXPECT_DOUBLE_EQ(zero.regularization, 1e-12);
  EXPECT_LE(
      independentlyCertify(
          effectiveMatrix(Eigen::Matrix3d::Zero(), zero), c, zero.impulse, cone)
          .worst(),
      1e-10L);
  Eigen::Matrix3d indefinite = Eigen::Matrix3d::Identity();
  indefinite(0, 0) = -1;
  EXPECT_FALSE(solveConeQp(indefinite, c, cone).certified);
  Eigen::Matrix3d nonsymmetric = Eigen::Matrix3d::Identity();
  nonsymmetric(0, 1) = 0.5;
  EXPECT_FALSE(solveConeQp(nonsymmetric, c, cone).certified);
  Eigen::Vector3d invalid = c;
  invalid[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(
      solveConeQp(Eigen::Matrix3d::Identity(), invalid, cone).certified);
  const FrictionCone negative{
      Eigen::Vector2d(-1, 0.5), FrictionConeLaw::Ellipse};
  EXPECT_FALSE(solveConeQp(Eigen::Matrix3d::Identity(), c, negative).certified);
  Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
  bool allCertified = true;
  double sum = 0;
  for (int i = 0; i < 4; ++i) {
    solveConeQp(H, c, cone);
    solveExactContact(H, c, cone);
    projectCone(c, cone);
  }
  const double lo[] = {0, -0.5, -0.5};
  const double hi[] = {std::numeric_limits<double>::infinity(), 0.5, 0.5};
  const int findex[] = {-1, 0, 0};
  FrictionRowClassification rows;
  ASSERT_TRUE(classifyFrictionRows(3, lo, hi, findex, rows));
  const auto fallbackWarmup = solveConeQp(
      boundaryFallbackMatrix, boundaryFallbackLinear, boundaryFallbackCone);
  ASSERT_TRUE(fallbackWarmup.certified);
  ASSERT_GT(fallbackWarmup.numLocalFallbacks, 0u);
  dart::test::ScopedHeapAllocationCounter heap;
  dart::test::ScopedRawHeapAllocationCounter raw;
  for (int i = 0; i < 100; ++i) {
    allCertified
        = allCertified && classifyFrictionRows(3, lo, hi, findex, rows);
    const auto qp = solveConeQp(H, c, cone);
    const auto exact = solveExactContact(H, c, cone);
    const auto fallback = solveConeQp(
        boundaryFallbackMatrix, boundaryFallbackLinear, boundaryFallbackCone);
    allCertified = allCertified && fallback.certified;
    sum += contactViolation(exact.impulse, H * exact.impulse + c, 1, cone);
    allCertified = allCertified && qp.certified && exact.certified;
  }
  raw.stop();
  heap.stop();
  EXPECT_TRUE(allCertified);
  EXPECT_LT(sum, 1e-6);
  EXPECT_EQ(heap.allocationCount(), 0u);
  if (!raw.skipped()) {
    EXPECT_EQ(raw.allocationCount(), 0u);
  }
}

//==============================================================================
TEST(FrictionCone, ExactContactRefinesUntilTheContactCertifies)
{
  // The root tolerance is met here before the contact certificate passes.
  const Eigen::Matrix3d H = Eigen::Vector3d(0.001, 1.0, 1.0).asDiagonal();
  const Eigen::Vector3d c(-0.01, 2.0, 1.0);
  FrictionCone cone;
  cone.mu = Eigen::Vector2d(0.001, 150.0);
  const auto result = solveExactContact(H, c, cone);
  EXPECT_TRUE(result.certified);
}

//==============================================================================
TEST(FrictionCone, RankDeficientBlocksAreNotRejectedAsIndefinite)
{
  // a * a^T is PSD with rank one; its LDLT pivots can round slightly negative.
  for (const Eigen::Vector3d& a :
       {Eigen::Vector3d(0.3, 0.7, 1.1),
        Eigen::Vector3d(1e-3, 2.0, -0.5),
        Eigen::Vector3d(0.1, 0.1, 0.1),
        Eigen::Vector3d(
            0.69266230874610279, 0.72181976494641975, -0.70243298661799547)}) {
    const Eigen::Matrix3d H = a * a.transpose();
    FrictionCone cone;
    cone.mu = Eigen::Vector2d(0.5, 0.5);
    EXPECT_TRUE(coneQpCertificate(
        H, Eigen::Vector3d(1, 0, 0), Eigen::Vector3d::Zero(), cone))
        << a.transpose();
    const auto result = solveConeQp(H, Eigen::Vector3d(-1.0, 0.2, -0.3), cone);
    EXPECT_TRUE(result.certified) << a.transpose();
  }
  // An indefinite block is still rejected.
  FrictionCone cone;
  EXPECT_FALSE(coneQpCertificate(
      -Eigen::Matrix3d::Identity(),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d::Zero(),
      cone));
}
