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

#ifndef DART_TESTS_LCPPROBLEMFACTORY_HPP_
#define DART_TESTS_LCPPROBLEMFACTORY_HPP_

#include <dart/math/lcp/lcp_types.hpp>

#include <Eigen/Core>
#include <Eigen/QR>

#include <limits>
#include <optional>
#include <random>
#include <string>
#include <vector>

namespace dart::test {

enum class ProblemCategory
{
  Standard,
  Boxed,
  BoxedFriction
};

enum class ProblemDifficulty
{
  WellConditioned,
  IllConditioned,
  NearSingular
};

struct FactoryProblem
{
  std::string name;
  dart::math::LcpProblem problem;
  std::optional<Eigen::VectorXd> expectedSolution;
  ProblemCategory category;
  ProblemDifficulty difficulty;
};

class LcpProblemFactory
{
public:
  static constexpr double kInf = std::numeric_limits<double>::infinity();

  static FactoryProblem empty()
  {
    return FactoryProblem{
        "empty_n0",
        dart::math::LcpProblem(
            Eigen::MatrixXd(0, 0),
            Eigen::VectorXd(0),
            Eigen::VectorXd(0),
            Eigen::VectorXd(0),
            Eigen::VectorXi(0)),
        Eigen::VectorXd(0),
        ProblemCategory::Standard,
        ProblemDifficulty::WellConditioned};
  }

  static FactoryProblem trivial1d()
  {
    Eigen::MatrixXd A(1, 1);
    A << 2.0;
    Eigen::VectorXd xStar(1);
    xStar << 0.5;
    Eigen::VectorXd b = A * xStar;

    return FactoryProblem{
        "trivial_1d",
        dart::math::LcpProblem(
            A,
            b,
            Eigen::VectorXd::Zero(1),
            Eigen::VectorXd::Constant(1, kInf),
            Eigen::VectorXi::Constant(1, -1)),
        xStar,
        ProblemCategory::Standard,
        ProblemDifficulty::WellConditioned};
  }

  static FactoryProblem trivial1dAtLowerBound()
  {
    Eigen::MatrixXd A(1, 1);
    A << 2.0;
    Eigen::VectorXd b(1);
    b << -1.0; // x=0 gives w=1 > 0, so x*=0

    return FactoryProblem{
        "trivial_1d_at_lower",
        dart::math::LcpProblem(
            A,
            b,
            Eigen::VectorXd::Zero(1),
            Eigen::VectorXd::Constant(1, kInf),
            Eigen::VectorXi::Constant(1, -1)),
        Eigen::VectorXd::Zero(1),
        ProblemCategory::Standard,
        ProblemDifficulty::WellConditioned};
  }

  static FactoryProblem standard2dSpd()
  {
    Eigen::Matrix2d A;
    A << 4.0, 1.0, 1.0, 3.0;
    Eigen::Vector2d xStar(0.5, 0.25);
    Eigen::Vector2d b = A * xStar;

    return FactoryProblem{
        "standard_2d_spd",
        dart::math::LcpProblem(
            A,
            b,
            Eigen::Vector2d::Zero(),
            Eigen::Vector2d::Constant(kInf),
            Eigen::Vector2i::Constant(-1)),
        xStar,
        ProblemCategory::Standard,
        ProblemDifficulty::WellConditioned};
  }

  static FactoryProblem standard4dSpd()
  {
    Eigen::Matrix4d A;
    A << 5.0, 0.5, 0.3, 0.2, 0.5, 5.0, 0.4, 0.3, 0.3, 0.4, 5.0, 0.5, 0.2, 0.3,
        0.5, 5.0;
    Eigen::Vector4d xStar(0.3, 0.2, 0.15, 0.1);
    Eigen::Vector4d b = A * xStar;

    return FactoryProblem{
        "standard_4d_spd",
        dart::math::LcpProblem(
            A,
            b,
            Eigen::Vector4d::Zero(),
            Eigen::Vector4d::Constant(kInf),
            Eigen::Vector4i::Constant(-1)),
        xStar,
        ProblemCategory::Standard,
        ProblemDifficulty::WellConditioned};
  }

  static FactoryProblem boxed2dActiveUpper()
  {
    Eigen::Matrix2d A;
    A << 2.0, 0.5, 0.5, 1.5;
    Eigen::Vector2d lo(-1.0, 0.0);
    Eigen::Vector2d hi(1.0, 0.2);
    Eigen::Vector2d xStar(0.1, 0.2); // At upper bound on second component
    Eigen::Vector2d w(0.0, -0.3);    // w[1] < 0 since at upper
    Eigen::Vector2d b = A * xStar - w;

    return FactoryProblem{
        "boxed_2d_active_upper",
        dart::math::LcpProblem(A, b, lo, hi, Eigen::Vector2i::Constant(-1)),
        xStar,
        ProblemCategory::Boxed,
        ProblemDifficulty::WellConditioned};
  }

  static FactoryProblem boxed3dMixedBounds()
  {
    Eigen::Matrix3d A;
    A << 3.0, 0.2, 0.0, 0.2, 2.5, 0.1, 0.0, 0.1, 2.0;
    Eigen::Vector3d lo(-0.2, -0.5, -0.1);
    Eigen::Vector3d hi(0.0, 0.5, 0.5);
    Eigen::Vector3d xStar(0.0, -0.5, 0.3);
    Eigen::Vector3d w(-0.1, 0.2, 0.0);
    Eigen::Vector3d b = A * xStar - w;

    return FactoryProblem{
        "boxed_3d_mixed_bounds",
        dart::math::LcpProblem(A, b, lo, hi, Eigen::Vector3i::Constant(-1)),
        xStar,
        ProblemCategory::Boxed,
        ProblemDifficulty::WellConditioned};
  }

  static FactoryProblem singleContactFriction()
  {
    Eigen::Matrix3d A;
    A << 4.0, 0.5, 0.0, 0.5, 3.0, 0.25, 0.0, 0.25, 2.5;
    Eigen::Vector3d xStar(1.0, 0.2, -0.1);
    Eigen::Vector3d b = A * xStar;

    const double mu = 0.5;
    Eigen::Vector3d lo(0.0, -mu, -mu);
    Eigen::Vector3d hi(kInf, mu, mu);
    Eigen::Vector3i findex(-1, 0, 0);

    return FactoryProblem{
        "single_contact_friction",
        dart::math::LcpProblem(A, b, lo, hi, findex),
        xStar,
        ProblemCategory::BoxedFriction,
        ProblemDifficulty::WellConditioned};
  }

  static FactoryProblem twoContactsFriction()
  {
    const int n = 6;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);

    Eigen::Matrix3d A1;
    A1 << 3.0, 0.1, 0.0, 0.1, 2.5, 0.2, 0.0, 0.2, 2.2;
    Eigen::Matrix3d A2;
    A2 << 4.0, 0.3, 0.0, 0.3, 3.5, 0.2, 0.0, 0.2, 3.0;
    A.block<3, 3>(0, 0) = A1;
    A.block<3, 3>(3, 3) = A2;

    Eigen::VectorXd xStar = Eigen::VectorXd::Zero(n);
    xStar[3] = 1.2;
    xStar[4] = 0.2;
    xStar[5] = -0.1;
    Eigen::VectorXd b = A * xStar;

    const double mu1 = 0.6;
    const double mu2 = 0.3;
    Eigen::VectorXd lo = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd hi = Eigen::VectorXd::Zero(n);
    Eigen::VectorXi findex = Eigen::VectorXi::Constant(n, -1);

    lo[0] = 0.0;
    hi[0] = kInf;
    lo[1] = -mu1;
    hi[1] = mu1;
    findex[1] = 0;
    lo[2] = -mu1;
    hi[2] = mu1;
    findex[2] = 0;

    lo[3] = 0.0;
    hi[3] = kInf;
    lo[4] = -mu2;
    hi[4] = mu2;
    findex[4] = 3;
    lo[5] = -mu2;
    hi[5] = mu2;
    findex[5] = 3;

    return FactoryProblem{
        "two_contacts_friction",
        dart::math::LcpProblem(A, b, lo, hi, findex),
        xStar,
        ProblemCategory::BoxedFriction,
        ProblemDifficulty::WellConditioned};
  }

  static FactoryProblem illConditioned8d(unsigned seed = 2001)
  {
    const int n = 8;
    Eigen::MatrixXd A = makeIllConditionedSpd(n, seed);
    Eigen::VectorXd xStar = makePositiveVector(n, seed + 1, 0.1, 1.0);
    Eigen::VectorXd b = A * xStar;

    return FactoryProblem{
        "ill_conditioned_8d",
        dart::math::LcpProblem(
            A,
            b,
            Eigen::VectorXd::Zero(n),
            Eigen::VectorXd::Constant(n, kInf),
            Eigen::VectorXi::Constant(n, -1)),
        xStar,
        ProblemCategory::Standard,
        ProblemDifficulty::IllConditioned};
  }

  static FactoryProblem massRatio12d(unsigned seed = 2002)
  {
    const int n = 12;
    Eigen::MatrixXd A = makeMassRatioSpd(n, seed);
    Eigen::VectorXd xStar = makePositiveVector(n, seed + 1, 0.05, 0.6);
    Eigen::VectorXd b = A * xStar;

    return FactoryProblem{
        "mass_ratio_12d",
        dart::math::LcpProblem(
            A,
            b,
            Eigen::VectorXd::Zero(n),
            Eigen::VectorXd::Constant(n, kInf),
            Eigen::VectorXi::Constant(n, -1)),
        xStar,
        ProblemCategory::Standard,
        ProblemDifficulty::IllConditioned};
  }

  static FactoryProblem randomStandard(int n, unsigned seed)
  {
    Eigen::MatrixXd A = makeSpdMatrix(n, seed, 1.0);
    Eigen::VectorXd xStar = makePositiveVector(n, seed + 1, 0.1, 0.5);
    Eigen::VectorXd b = A * xStar;

    return FactoryProblem{
        "random_standard_" + std::to_string(n) + "d",
        dart::math::LcpProblem(
            A,
            b,
            Eigen::VectorXd::Zero(n),
            Eigen::VectorXd::Constant(n, kInf),
            Eigen::VectorXi::Constant(n, -1)),
        xStar,
        ProblemCategory::Standard,
        ProblemDifficulty::WellConditioned};
  }

  static std::vector<FactoryProblem> getEdgeCases()
  {
    return {empty(), trivial1d(), trivial1dAtLowerBound()};
  }

  static std::vector<FactoryProblem> getStandardProblems()
  {
    return {standard2dSpd(), standard4dSpd()};
  }

  static std::vector<FactoryProblem> getBoxedProblems()
  {
    return {boxed2dActiveUpper(), boxed3dMixedBounds()};
  }

  static std::vector<FactoryProblem> getFrictionProblems()
  {
    return {singleContactFriction(), twoContactsFriction()};
  }

  static std::vector<FactoryProblem> getStressProblems()
  {
    return {illConditioned8d(), massRatio12d()};
  }

  static std::vector<FactoryProblem> getWellConditionedProblems()
  {
    std::vector<FactoryProblem> result;
    auto edges = getEdgeCases();
    auto standard = getStandardProblems();
    auto boxed = getBoxedProblems();
    auto friction = getFrictionProblems();

    result.insert(result.end(), edges.begin(), edges.end());
    result.insert(result.end(), standard.begin(), standard.end());
    result.insert(result.end(), boxed.begin(), boxed.end());
    result.insert(result.end(), friction.begin(), friction.end());
    return result;
  }

  static std::vector<FactoryProblem> getAllProblems()
  {
    auto result = getWellConditionedProblems();
    auto stress = getStressProblems();
    result.insert(result.end(), stress.begin(), stress.end());
    return result;
  }

private:
  static Eigen::MatrixXd makeSpdMatrix(int n, unsigned seed, double diagShift)
  {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    Eigen::MatrixXd M(n, n);
    for (int r = 0; r < n; ++r) {
      for (int c = 0; c < n; ++c) {
        M(r, c) = dist(rng);
      }
    }

    Eigen::MatrixXd A = M.transpose() * M;
    A += diagShift * Eigen::MatrixXd::Identity(n, n);
    return A;
  }

  static Eigen::MatrixXd makeIllConditionedSpd(int n, unsigned seed)
  {
    std::mt19937 rng(seed);
    std::normal_distribution<double> dist(0.0, 1.0);

    Eigen::MatrixXd M(n, n);
    for (int r = 0; r < n; ++r) {
      for (int c = 0; c < n; ++c) {
        M(r, c) = dist(rng);
      }
    }

    Eigen::HouseholderQR<Eigen::MatrixXd> qr(M);
    Eigen::MatrixXd Q = qr.householderQ() * Eigen::MatrixXd::Identity(n, n);

    Eigen::VectorXd eigs(n);
    const double minExp = -6.0;
    const double maxExp = 1.5;
    for (int i = 0; i < n; ++i) {
      const double t = (n > 1) ? static_cast<double>(i) / (n - 1) : 0.0;
      eigs[i] = std::pow(10.0, minExp + (maxExp - minExp) * t);
    }

    return Q * eigs.asDiagonal() * Q.transpose();
  }

  static Eigen::MatrixXd makeMassRatioSpd(int n, unsigned seed)
  {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dist(-0.1, 0.1);

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
    for (int i = 0; i < n; ++i) {
      const double t = (n > 1) ? static_cast<double>(i) / (n - 1) : 0.0;
      A(i, i) = std::pow(10.0, -3.0 + 6.0 * t);
    }

    for (int r = 0; r < n; ++r) {
      for (int c = r + 1; c < n; ++c) {
        const double v = 0.01 * dist(rng);
        A(r, c) += v;
        A(c, r) += v;
      }
    }

    A += 0.05 * Eigen::MatrixXd::Identity(n, n);
    return A;
  }

  static Eigen::VectorXd makePositiveVector(
      int n, unsigned seed, double minVal, double maxVal)
  {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dist(minVal, maxVal);
    Eigen::VectorXd x(n);
    for (int i = 0; i < n; ++i) {
      x[i] = dist(rng);
    }
    return x;
  }
};

} // namespace dart::test

#endif // DART_TESTS_LCPPROBLEMFACTORY_HPP_
