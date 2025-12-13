/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Stress and regression coverage for the math::LcpSolver implementations.
 */

#include <dart/math/lcp/LcpValidation.hpp>
#include <dart/math/lcp/pivoting/DantzigSolver.hpp>
#include <dart/math/lcp/projection/PgsSolver.hpp>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <limits>
#include <random>
#include <vector>

using namespace dart::math;

namespace {

Eigen::MatrixXd MakeRandomSpdMatrix(int n, std::mt19937& rng)
{
  std::uniform_real_distribution<double> dist(-1.0, 1.0);

  Eigen::MatrixXd M(n, n);
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c < n; ++c)
      M(r, c) = dist(rng);
  }

  return M.transpose() * M
         + static_cast<double>(n) * Eigen::MatrixXd::Identity(n, n);
}

void ExpectFeasibleSolution(
    const LcpProblem& problem, const Eigen::VectorXd& x, double tol)
{
  const Eigen::VectorXd w = problem.A * x - problem.b;
  Eigen::VectorXd loEff;
  Eigen::VectorXd hiEff;
  std::string message;
  ASSERT_TRUE(detail::computeEffectiveBounds(
      problem.lo, problem.hi, problem.findex, x, loEff, hiEff, &message))
      << message;
  EXPECT_TRUE(detail::validateSolution(x, w, loEff, hiEff, tol, &message))
      << message;
}

} // namespace

//==============================================================================
TEST(LcpSolversStress, RandomSpdStandardProblems)
{
  std::mt19937 rng(123u);
  const std::vector<int> sizes = {1, 2, 5, 10};

  for (const int n : sizes) {
    const Eigen::MatrixXd A = MakeRandomSpdMatrix(n, rng);

    std::uniform_real_distribution<double> dist(0.1, 1.0);
    Eigen::VectorXd xStar(n);
    for (int i = 0; i < n; ++i)
      xStar[i] = dist(rng);

    const Eigen::VectorXd b = A * xStar;

    const Eigen::VectorXd lo = Eigen::VectorXd::Zero(n);
    const Eigen::VectorXd hi
        = Eigen::VectorXd::Constant(n, std::numeric_limits<double>::infinity());
    const Eigen::VectorXi findex = Eigen::VectorXi::Constant(n, -1);
    const LcpProblem problem(A, b, lo, hi, findex);

    DantzigSolver dantzig;
    LcpOptions dantzigOptions = dantzig.getDefaultOptions();
    dantzigOptions.warmStart = false;
    dantzigOptions.validateSolution = true;
    dantzigOptions.absoluteTolerance = 1e-8;
    dantzigOptions.complementarityTolerance = 1e-6;

    Eigen::VectorXd xDantzig = Eigen::VectorXd::Zero(n);
    const auto dantzigResult = dantzig.solve(problem, xDantzig, dantzigOptions);
    ASSERT_TRUE(dantzigResult.succeeded()) << dantzigResult.message;
    ExpectFeasibleSolution(problem, xDantzig, 1e-6);
    EXPECT_NEAR((xDantzig - xStar).lpNorm<Eigen::Infinity>(), 0.0, 1e-6);

    PgsSolver pgs;
    LcpOptions pgsOptions = pgs.getDefaultOptions();
    pgsOptions.warmStart = false;
    pgsOptions.maxIterations = 10000;
    pgsOptions.absoluteTolerance = 1e-4;
    pgsOptions.relativeTolerance = 1e-2;
    pgsOptions.complementarityTolerance = 1e-2;
    pgsOptions.validateSolution = true;

    Eigen::VectorXd xPgs = Eigen::VectorXd::Zero(n);
    const auto pgsResult = pgs.solve(problem, xPgs, pgsOptions);
    ASSERT_TRUE(pgsResult.succeeded()) << pgsResult.message;
    ExpectFeasibleSolution(problem, xPgs, 1e-2);
    EXPECT_NEAR((xPgs - xDantzig).lpNorm<Eigen::Infinity>(), 0.0, 1e-2);
  }
}

//==============================================================================
TEST(LcpSolversStress, RandomSpdFrictionIndexProblems)
{
  std::mt19937 rng(456u);
  std::uniform_real_distribution<double> dist(0.1, 1.0);
  std::uniform_real_distribution<double> muDist(0.2, 1.0);
  const std::vector<int> contactsCases = {1, 4, 8};

  for (const int numContacts : contactsCases) {
    const int n = 3 * numContacts;
    const Eigen::MatrixXd A = MakeRandomSpdMatrix(n, rng);

    Eigen::VectorXd xStar = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd lo = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd hi = Eigen::VectorXd::Zero(n);
    Eigen::VectorXi findex = Eigen::VectorXi::Constant(n, -1);

    for (int c = 0; c < numContacts; ++c) {
      const int base = 3 * c;
      const double normal = (c == 0) ? 0.0 : dist(rng);
      const double mu = muDist(rng);

      xStar[base + 0] = normal;
      xStar[base + 1] = 0.25 * mu * normal;
      xStar[base + 2] = -0.15 * mu * normal;

      lo[base + 0] = 0.0;
      hi[base + 0] = std::numeric_limits<double>::infinity();

      lo[base + 1] = -mu;
      hi[base + 1] = mu;
      findex[base + 1] = base + 0;

      lo[base + 2] = -mu;
      hi[base + 2] = mu;
      findex[base + 2] = base + 0;
    }

    const Eigen::VectorXd b = A * xStar;
    const LcpProblem problem(A, b, lo, hi, findex);

    DantzigSolver dantzig;
    LcpOptions dantzigOptions = dantzig.getDefaultOptions();
    dantzigOptions.warmStart = false;
    dantzigOptions.validateSolution = true;

    Eigen::VectorXd xDantzig = Eigen::VectorXd::Zero(n);
    const auto dantzigResult = dantzig.solve(problem, xDantzig, dantzigOptions);
    ASSERT_TRUE(dantzigResult.succeeded()) << dantzigResult.message;
    ExpectFeasibleSolution(problem, xDantzig, 1e-6);

    PgsSolver pgs;
    LcpOptions pgsOptions = pgs.getDefaultOptions();
    pgsOptions.warmStart = false;
    pgsOptions.maxIterations = 20000;
    pgsOptions.absoluteTolerance = 1e-4;
    pgsOptions.relativeTolerance = 1e-2;
    pgsOptions.complementarityTolerance = 2e-2;
    pgsOptions.validateSolution = true;

    Eigen::VectorXd xPgs = Eigen::VectorXd::Zero(n);
    const auto pgsResult = pgs.solve(problem, xPgs, pgsOptions);
    ASSERT_TRUE(pgsResult.succeeded()) << pgsResult.message;
    ExpectFeasibleSolution(problem, xPgs, 2e-2);

    for (int c = 0; c < numContacts; ++c) {
      const int base = 3 * c;
      const double mu = hi[base + 1];
      EXPECT_LE(std::abs(xPgs[base + 1]), mu * xPgs[base + 0] + 1e-8);
      EXPECT_LE(std::abs(xPgs[base + 2]), mu * xPgs[base + 0] + 1e-8);
    }
  }
}
