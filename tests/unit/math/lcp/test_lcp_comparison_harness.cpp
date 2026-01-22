/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Solver-agnostic LCP comparison harness coverage.
 */

#include "tests/common/lcpsolver/lcp_test_fixtures.hpp"
#include "tests/common/lcpsolver/lcp_test_harness.hpp"

#include <dart/math/lcp/newton/fischer_burmeister_newton_solver.hpp>
#include <dart/math/lcp/newton/minimum_map_newton_solver.hpp>
#include <dart/math/lcp/newton/penalized_fischer_burmeister_newton_solver.hpp>
#include <dart/math/lcp/other/interior_point_solver.hpp>
#include <dart/math/lcp/other/mprgp_solver.hpp>
#include <dart/math/lcp/other/shock_propagation_solver.hpp>
#include <dart/math/lcp/other/staggering_solver.hpp>
#include <dart/math/lcp/pivoting/baraff_solver.hpp>
#include <dart/math/lcp/pivoting/dantzig_solver.hpp>
#include <dart/math/lcp/pivoting/direct_solver.hpp>
#include <dart/math/lcp/pivoting/lemke_solver.hpp>
#include <dart/math/lcp/projection/bgs_solver.hpp>
#include <dart/math/lcp/projection/blocked_jacobi_solver.hpp>
#include <dart/math/lcp/projection/jacobi_solver.hpp>
#include <dart/math/lcp/projection/nncg_solver.hpp>
#include <dart/math/lcp/projection/pgs_solver.hpp>
#include <dart/math/lcp/projection/red_black_gauss_seidel_solver.hpp>
#include <dart/math/lcp/projection/subspace_minimization_solver.hpp>
#include <dart/math/lcp/projection/symmetric_psor_solver.hpp>

#include <Eigen/Dense>
#include <gtest/gtest.h>

namespace {

using dart::math::LcpOptions;
using dart::math::LcpSolver;
using dart::math::LcpSolverStatus;
using dart::test::LcpFixture;

void ExpectSolverPassesFixture(
    LcpSolver& solver,
    const LcpFixture& fixture,
    const LcpOptions& options,
    double expectedTol,
    bool allowMaxIterations)
{
  Eigen::VectorXd x = Eigen::VectorXd::Zero(fixture.problem.b.size());
  const auto report
      = dart::test::SolveAndCheck(solver, fixture.problem, x, options);

  const bool statusOk
      = report.result.succeeded()
        || (allowMaxIterations
            && report.result.status == LcpSolverStatus::MaxIterations
            && report.check.ok);

  EXPECT_TRUE(statusOk) << fixture.name << ": "
                        << dart::test::DescribeReport(report);
  EXPECT_TRUE(report.check.ok)
      << fixture.name << ": " << dart::test::DescribeReport(report);

  ASSERT_EQ(x.size(), fixture.expected.size());
  const double error = (x - fixture.expected).lpNorm<Eigen::Infinity>();
  EXPECT_NEAR(error, 0.0, expectedTol)
      << fixture.name << ": " << dart::test::DescribeReport(report);
}

} // namespace

//==============================================================================
TEST(LcpComparisonHarness, DantzigOnStandardAndBoxedFixtures)
{
  dart::math::DantzigSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.absoluteTolerance = 1e-8;
  options.relativeTolerance = 1e-6;
  options.complementarityTolerance = 1e-6;

  for (const auto& fixture : dart::test::getStandardBoxedFixtures()) {
    ExpectSolverPassesFixture(solver, fixture, options, 1e-6, false);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, BaraffOnStandardFixtures)
{
  dart::math::BaraffSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.absoluteTolerance = 1e-8;
  options.relativeTolerance = 1e-6;
  options.complementarityTolerance = 1e-6;

  for (const auto& fixture : dart::test::getStandardBoxedFixtures()) {
    if (fixture.kind != dart::test::LcpFixtureKind::Standard) {
      continue;
    }
    ExpectSolverPassesFixture(solver, fixture, options, 1e-6, false);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, InteriorPointOnStandardFixtures)
{
  dart::math::InteriorPointSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = 100;
  options.absoluteTolerance = 1e-6;
  options.relativeTolerance = 1e-4;
  options.complementarityTolerance = 1e-6;

  for (const auto& fixture : dart::test::getStandardBoxedFixtures()) {
    if (fixture.kind != dart::test::LcpFixtureKind::Standard) {
      continue;
    }
    ExpectSolverPassesFixture(solver, fixture, options, 1e-4, true);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, MprgpOnStandardFixtures)
{
  dart::math::MprgpSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = 200;
  options.absoluteTolerance = 1e-8;
  options.relativeTolerance = 1e-6;
  options.complementarityTolerance = 1e-6;

  for (const auto& fixture : dart::test::getStandardBoxedFixtures()) {
    if (fixture.kind != dart::test::LcpFixtureKind::Standard) {
      continue;
    }
    ExpectSolverPassesFixture(solver, fixture, options, 1e-6, false);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, ShockPropagationOnStandardFixtures)
{
  dart::math::ShockPropagationSolver solver;

  for (const auto& fixture : dart::test::getStandardBoxedFixtures()) {
    if (fixture.kind != dart::test::LcpFixtureKind::Standard) {
      continue;
    }

    const int n = static_cast<int>(fixture.problem.b.size());

    dart::math::ShockPropagationSolver::Parameters params;
    params.blockSizes = {n};
    params.layers = {{0}};

    LcpOptions options = solver.getDefaultOptions();
    options.warmStart = false;
    options.validateSolution = false;
    options.maxIterations = 1;
    options.absoluteTolerance = 1e-8;
    options.relativeTolerance = 1e-6;
    options.complementarityTolerance = 1e-6;
    options.customOptions = &params;

    ExpectSolverPassesFixture(solver, fixture, options, 1e-6, false);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, ShockPropagationOnFrictionIndexFixtures)
{
  dart::math::ShockPropagationSolver solver;

  for (const auto& fixture : dart::test::getFrictionIndexFixtures()) {
    const int n = static_cast<int>(fixture.problem.b.size());
    if (n % 3 != 0) {
      continue;
    }

    const int numBlocks = n / 3;
    dart::math::ShockPropagationSolver::Parameters params;
    params.blockSizes.assign(numBlocks, 3);
    params.layers.clear();
    params.layers.reserve(numBlocks);
    for (int i = 0; i < numBlocks; ++i) {
      params.layers.push_back({i});
    }

    LcpOptions options = solver.getDefaultOptions();
    options.warmStart = false;
    options.validateSolution = false;
    options.maxIterations = 1;
    options.absoluteTolerance = 1e-6;
    options.relativeTolerance = 1e-4;
    options.complementarityTolerance = 1e-6;
    options.customOptions = &params;

    ExpectSolverPassesFixture(solver, fixture, options, 1e-6, false);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, PgsOnStandardAndBoxedFixtures)
{
  dart::math::PgsSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = 20000;
  options.absoluteTolerance = 1e-4;
  options.relativeTolerance = 1e-2;
  options.complementarityTolerance = 1e-2;

  for (const auto& fixture : dart::test::getStandardBoxedFixtures()) {
    ExpectSolverPassesFixture(solver, fixture, options, 1e-2, true);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, JacobiOnStandardAndBoxedFixtures)
{
  dart::math::JacobiSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = 20000;
  options.absoluteTolerance = 1e-4;
  options.relativeTolerance = 1e-2;
  options.complementarityTolerance = 1e-2;

  for (const auto& fixture : dart::test::getStandardBoxedFixtures()) {
    ExpectSolverPassesFixture(solver, fixture, options, 1e-2, true);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, SymmetricPsorOnStandardAndBoxedFixtures)
{
  dart::math::SymmetricPsorSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = 20000;
  options.absoluteTolerance = 1e-4;
  options.relativeTolerance = 1e-2;
  options.complementarityTolerance = 1e-2;

  for (const auto& fixture : dart::test::getStandardBoxedFixtures()) {
    ExpectSolverPassesFixture(solver, fixture, options, 1e-2, true);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, RedBlackGaussSeidelOnStandardAndBoxedFixtures)
{
  dart::math::RedBlackGaussSeidelSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = 20000;
  options.absoluteTolerance = 1e-4;
  options.relativeTolerance = 1e-2;
  options.complementarityTolerance = 1e-2;

  for (const auto& fixture : dart::test::getStandardBoxedFixtures()) {
    ExpectSolverPassesFixture(solver, fixture, options, 1e-2, true);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, StaggeringOnStandardAndBoxedFixtures)
{
  dart::math::StaggeringSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = 200;
  options.absoluteTolerance = 1e-4;
  options.relativeTolerance = 1e-2;
  options.complementarityTolerance = 1e-2;

  for (const auto& fixture : dart::test::getStandardBoxedFixtures()) {
    ExpectSolverPassesFixture(solver, fixture, options, 1e-2, true);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, BgsOnStandardAndBoxedFixtures)
{
  dart::math::BgsSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = 5000;
  options.absoluteTolerance = 1e-4;
  options.relativeTolerance = 1e-2;
  options.complementarityTolerance = 1e-2;

  for (const auto& fixture : dart::test::getStandardBoxedFixtures()) {
    ExpectSolverPassesFixture(solver, fixture, options, 1e-2, true);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, BgsOnStandardFixtureWithExplicitBlocks)
{
  dart::math::BgsSolver solver;

  for (const auto& fixture : dart::test::getStandardBoxedFixtures()) {
    if (fixture.kind != dart::test::LcpFixtureKind::Standard) {
      continue;
    }
    const int n = static_cast<int>(fixture.problem.b.size());
    if (n > 3) {
      continue;
    }

    dart::math::BgsSolver::Parameters params;
    params.blockSizes = {n};

    LcpOptions options = solver.getDefaultOptions();
    options.warmStart = false;
    options.validateSolution = false;
    options.maxIterations = 5;
    options.absoluteTolerance = 1e-8;
    options.relativeTolerance = 1e-6;
    options.complementarityTolerance = 1e-6;
    options.customOptions = &params;

    ExpectSolverPassesFixture(solver, fixture, options, 1e-6, false);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, BlockedJacobiOnStandardAndBoxedFixtures)
{
  dart::math::BlockedJacobiSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = 20000;
  options.absoluteTolerance = 1e-4;
  options.relativeTolerance = 1e-2;
  options.complementarityTolerance = 1e-2;

  for (const auto& fixture : dart::test::getStandardBoxedFixtures()) {
    ExpectSolverPassesFixture(solver, fixture, options, 1e-2, true);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, BlockedJacobiOnStandardFixtureWithExplicitBlocks)
{
  dart::math::BlockedJacobiSolver solver;

  for (const auto& fixture : dart::test::getStandardBoxedFixtures()) {
    if (fixture.kind != dart::test::LcpFixtureKind::Standard) {
      continue;
    }
    const int n = static_cast<int>(fixture.problem.b.size());
    if (n > 3) {
      continue;
    }

    dart::math::BlockedJacobiSolver::Parameters params;
    params.blockSizes = {n};

    LcpOptions options = solver.getDefaultOptions();
    options.warmStart = false;
    options.validateSolution = false;
    options.maxIterations = 5;
    options.absoluteTolerance = 1e-8;
    options.relativeTolerance = 1e-6;
    options.complementarityTolerance = 1e-6;
    options.customOptions = &params;

    ExpectSolverPassesFixture(solver, fixture, options, 1e-6, false);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, NncgOnStandardAndBoxedFixtures)
{
  dart::math::NncgSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = 1000;
  options.absoluteTolerance = 1e-4;
  options.relativeTolerance = 1e-2;
  options.complementarityTolerance = 1e-2;

  dart::math::NncgSolver::Parameters params;
  params.pgsIterations = 2;
  params.restartInterval = 10;
  params.restartThreshold = 1.0;
  options.customOptions = &params;

  for (const auto& fixture : dart::test::getStandardBoxedFixtures()) {
    ExpectSolverPassesFixture(solver, fixture, options, 1e-2, true);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, SubspaceMinimizationOnStandardAndBoxedFixtures)
{
  dart::math::SubspaceMinimizationSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = 1000;
  options.absoluteTolerance = 1e-4;
  options.relativeTolerance = 1e-2;
  options.complementarityTolerance = 1e-2;

  dart::math::SubspaceMinimizationSolver::Parameters params;
  params.pgsIterations = 5;
  options.customOptions = &params;

  for (const auto& fixture : dart::test::getStandardBoxedFixtures()) {
    ExpectSolverPassesFixture(solver, fixture, options, 1e-2, true);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, DantzigOnFrictionIndexFixtures)
{
  dart::math::DantzigSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.absoluteTolerance = 1e-8;
  options.relativeTolerance = 1e-6;
  options.complementarityTolerance = 1e-6;

  for (const auto& fixture : dart::test::getFrictionIndexFixtures()) {
    ExpectSolverPassesFixture(solver, fixture, options, 1e-6, false);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, PgsOnFrictionIndexFixtures)
{
  dart::math::PgsSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = 20000;
  options.absoluteTolerance = 1e-4;
  options.relativeTolerance = 1e-2;
  options.complementarityTolerance = 2e-2;

  for (const auto& fixture : dart::test::getFrictionIndexFixtures()) {
    ExpectSolverPassesFixture(solver, fixture, options, 2e-2, true);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, JacobiOnFrictionIndexFixtures)
{
  dart::math::JacobiSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = 20000;
  options.absoluteTolerance = 1e-4;
  options.relativeTolerance = 1e-2;
  options.complementarityTolerance = 2e-2;

  for (const auto& fixture : dart::test::getFrictionIndexFixtures()) {
    ExpectSolverPassesFixture(solver, fixture, options, 2e-2, true);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, SymmetricPsorOnFrictionIndexFixtures)
{
  dart::math::SymmetricPsorSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = 20000;
  options.absoluteTolerance = 1e-4;
  options.relativeTolerance = 1e-2;
  options.complementarityTolerance = 2e-2;

  for (const auto& fixture : dart::test::getFrictionIndexFixtures()) {
    ExpectSolverPassesFixture(solver, fixture, options, 2e-2, true);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, RedBlackGaussSeidelOnFrictionIndexFixtures)
{
  dart::math::RedBlackGaussSeidelSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = 20000;
  options.absoluteTolerance = 1e-4;
  options.relativeTolerance = 1e-2;
  options.complementarityTolerance = 2e-2;

  for (const auto& fixture : dart::test::getFrictionIndexFixtures()) {
    ExpectSolverPassesFixture(solver, fixture, options, 2e-2, true);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, StaggeringOnFrictionIndexFixtures)
{
  dart::math::StaggeringSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = 200;
  options.absoluteTolerance = 1e-4;
  options.relativeTolerance = 1e-2;
  options.complementarityTolerance = 2e-2;

  for (const auto& fixture : dart::test::getFrictionIndexFixtures()) {
    ExpectSolverPassesFixture(solver, fixture, options, 2e-2, true);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, BgsOnFrictionIndexFixtures)
{
  dart::math::BgsSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = 5000;
  options.absoluteTolerance = 1e-4;
  options.relativeTolerance = 1e-2;
  options.complementarityTolerance = 2e-2;

  for (const auto& fixture : dart::test::getFrictionIndexFixtures()) {
    ExpectSolverPassesFixture(solver, fixture, options, 2e-2, true);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, BlockedJacobiOnFrictionIndexFixtures)
{
  dart::math::BlockedJacobiSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = 20000;
  options.absoluteTolerance = 1e-4;
  options.relativeTolerance = 1e-2;
  options.complementarityTolerance = 2e-2;

  for (const auto& fixture : dart::test::getFrictionIndexFixtures()) {
    ExpectSolverPassesFixture(solver, fixture, options, 2e-2, true);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, NncgOnFrictionIndexFixtures)
{
  dart::math::NncgSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = 5000;
  options.absoluteTolerance = 1e-4;
  options.relativeTolerance = 1e-2;
  options.complementarityTolerance = 2e-2;

  dart::math::NncgSolver::Parameters params;
  params.pgsIterations = 2;
  params.restartInterval = 10;
  params.restartThreshold = 1.0;
  options.customOptions = &params;

  for (const auto& fixture : dart::test::getFrictionIndexFixtures()) {
    ExpectSolverPassesFixture(solver, fixture, options, 2e-2, true);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, SubspaceMinimizationOnFrictionIndexFixtures)
{
  dart::math::SubspaceMinimizationSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = 5000;
  options.absoluteTolerance = 1e-4;
  options.relativeTolerance = 1e-2;
  options.complementarityTolerance = 2e-2;

  dart::math::SubspaceMinimizationSolver::Parameters params;
  params.pgsIterations = 5;
  options.customOptions = &params;

  for (const auto& fixture : dart::test::getFrictionIndexFixtures()) {
    ExpectSolverPassesFixture(solver, fixture, options, 2e-2, true);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, LemkeOnStandardFixtures)
{
  dart::math::LemkeSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.absoluteTolerance = 1e-8;
  options.relativeTolerance = 1e-6;
  options.complementarityTolerance = 1e-6;

  for (const auto& fixture : dart::test::getStandardBoxedFixtures()) {
    if (fixture.kind != dart::test::LcpFixtureKind::Standard) {
      continue;
    }
    ExpectSolverPassesFixture(solver, fixture, options, 1e-6, false);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, DirectOnStandardFixtures)
{
  dart::math::DirectSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.absoluteTolerance = 1e-10;
  options.relativeTolerance = 1e-8;
  options.complementarityTolerance = 1e-8;

  for (const auto& fixture : dart::test::getStandardBoxedFixtures()) {
    if (fixture.kind != dart::test::LcpFixtureKind::Standard) {
      continue;
    }
    ExpectSolverPassesFixture(solver, fixture, options, 1e-8, false);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, MinimumMapNewtonOnStandardFixtures)
{
  dart::math::MinimumMapNewtonSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = 50;
  options.absoluteTolerance = 1e-8;
  options.relativeTolerance = 1e-6;
  options.complementarityTolerance = 1e-6;

  for (const auto& fixture : dart::test::getStandardBoxedFixtures()) {
    if (fixture.kind != dart::test::LcpFixtureKind::Standard) {
      continue;
    }
    ExpectSolverPassesFixture(solver, fixture, options, 1e-6, false);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, FischerBurmeisterNewtonOnStandardFixtures)
{
  dart::math::FischerBurmeisterNewtonSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = 50;
  options.absoluteTolerance = 1e-8;
  options.relativeTolerance = 1e-6;
  options.complementarityTolerance = 1e-6;

  for (const auto& fixture : dart::test::getStandardBoxedFixtures()) {
    if (fixture.kind != dart::test::LcpFixtureKind::Standard) {
      continue;
    }
    ExpectSolverPassesFixture(solver, fixture, options, 1e-6, false);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, PenalizedFischerBurmeisterNewtonOnStandardFixtures)
{
  dart::math::PenalizedFischerBurmeisterNewtonSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  options.validateSolution = false;
  options.maxIterations = 50;
  options.absoluteTolerance = 1e-8;
  options.relativeTolerance = 1e-6;
  options.complementarityTolerance = 1e-6;

  dart::math::PenalizedFischerBurmeisterNewtonSolver::Parameters params;
  params.lambda = 0.5;
  options.customOptions = &params;

  for (const auto& fixture : dart::test::getStandardBoxedFixtures()) {
    if (fixture.kind != dart::test::LcpFixtureKind::Standard) {
      continue;
    }
    ExpectSolverPassesFixture(solver, fixture, options, 1e-6, false);
  }
}

//==============================================================================
TEST(LcpComparisonHarness, StaggeringRunsNormalAndFrictionPasses)
{
  dart::math::StaggeringSolver solver;

  Eigen::MatrixXd A(2, 2);
  A << 2.0, 0.0, 0.0, 2.0;
  Eigen::VectorXd b(2);
  b << 1.0, 1.0;
  Eigen::VectorXd lo(2);
  lo << 0.0, -0.5;
  Eigen::VectorXd hi(2);
  hi << 1.0, 0.5;
  Eigen::VectorXi findex(2);
  findex << -1, 0;

  dart::math::LcpProblem problem(A, b, lo, hi, findex);
  Eigen::VectorXd x;

  auto options = solver.getDefaultOptions();
  options.validateSolution = false;
  options.warmStart = false;
  options.maxIterations = 1;
  options.absoluteTolerance = 1e-12;
  options.relativeTolerance = 1e-12;
  options.complementarityTolerance = 1e-12;

  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(x.size(), 2);
  EXPECT_NE(result.status, dart::math::LcpSolverStatus::InvalidProblem);
}
