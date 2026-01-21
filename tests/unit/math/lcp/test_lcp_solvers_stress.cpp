/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Stress and regression coverage for the math::LcpSolver implementations.
 */

#include <dart/math/lcp/lcp_validation.hpp>
#include <dart/math/lcp/pivoting/dantzig_solver.hpp>
#include <dart/math/lcp/projection/pgs_solver.hpp>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <limits>
#include <random>
#include <vector>

using namespace dart::math;

namespace {

// Deterministic scaling avoids platform-specific uniform_real_distribution
// output.
double Uniform01(std::mt19937& rng)
{
  const double denom = static_cast<double>(rng.max()) + 1.0;
  return static_cast<double>(rng()) / denom;
}

double Uniform(std::mt19937& rng, double lo, double hi)
{
  return lo + (hi - lo) * Uniform01(rng);
}

Eigen::MatrixXd MakeRandomSpdMatrix(int n, std::mt19937& rng)
{
  Eigen::MatrixXd M(n, n);
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c < n; ++c) {
      M(r, c) = Uniform(rng, -1.0, 1.0);
    }
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
  ASSERT_TRUE(
      detail::computeEffectiveBounds(
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

    Eigen::VectorXd xStar(n);
    for (int i = 0; i < n; ++i) {
      xStar[i] = Uniform(rng, 0.1, 1.0);
    }

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
      const double normal = (c == 0) ? 0.0 : Uniform(rng, 0.1, 1.0);
      const double mu = Uniform(rng, 0.2, 1.0);

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

//==============================================================================
TEST(LcpSolversStress, RandomSpdBoxedProblems)
{
  std::mt19937 rng(789u);
  const std::vector<int> sizes = {2, 5, 10};

  for (const int n : sizes) {
    const Eigen::MatrixXd A = MakeRandomSpdMatrix(n, rng);
    const Eigen::VectorXd lo = Eigen::VectorXd::Constant(n, -1.0);
    const Eigen::VectorXd hi = Eigen::VectorXd::Constant(n, 1.0);
    const Eigen::VectorXi findex = Eigen::VectorXi::Constant(n, -1);

    Eigen::VectorXd xStar(n);
    Eigen::VectorXd w = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n; ++i) {
      const int mode = i % 3;
      if (mode == 0) {
        xStar[i] = lo[i];
        w[i] = Uniform(rng, 0.1, 1.0);
      } else if (mode == 1) {
        xStar[i] = hi[i];
        w[i] = -Uniform(rng, 0.1, 1.0);
      } else {
        xStar[i] = 0.5 * Uniform(rng, -1.0, 1.0);
        w[i] = 0.0;
      }
    }

    const Eigen::VectorXd b = A * xStar - w;
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
    pgsOptions.maxIterations = 20000;
    pgsOptions.absoluteTolerance = 1e-4;
    pgsOptions.relativeTolerance = 1e-2;
    pgsOptions.complementarityTolerance = 1e-2;
    pgsOptions.validateSolution = true;

    Eigen::VectorXd xPgs = Eigen::VectorXd::Zero(n);
    const auto pgsResult = pgs.solve(problem, xPgs, pgsOptions);
    ASSERT_TRUE(pgsResult.succeeded()) << pgsResult.message;
    ExpectFeasibleSolution(problem, xPgs, 2e-2);
    EXPECT_NEAR((xPgs - xDantzig).lpNorm<Eigen::Infinity>(), 0.0, 2e-2);
  }
}
