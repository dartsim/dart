/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Solver-agnostic LCP comparison harness coverage.
 */

#include "tests/common/lcpsolver/LcpTestFixtures.hpp"
#include "tests/common/lcpsolver/LcpTestHarness.hpp"

#include <dart/math/lcp/newton/FischerBurmeisterNewtonSolver.hpp>
#include <dart/math/lcp/newton/MinimumMapNewtonSolver.hpp>
#include <dart/math/lcp/newton/PenalizedFischerBurmeisterNewtonSolver.hpp>
#include <dart/math/lcp/other/StaggeringSolver.hpp>
#include <dart/math/lcp/pivoting/BaraffSolver.hpp>
#include <dart/math/lcp/pivoting/DantzigSolver.hpp>
#include <dart/math/lcp/pivoting/LemkeSolver.hpp>
#include <dart/math/lcp/projection/BgsSolver.hpp>
#include <dart/math/lcp/projection/JacobiSolver.hpp>
#include <dart/math/lcp/projection/NncgSolver.hpp>
#include <dart/math/lcp/projection/PgsSolver.hpp>
#include <dart/math/lcp/projection/RedBlackGaussSeidelSolver.hpp>
#include <dart/math/lcp/projection/SubspaceMinimizationSolver.hpp>
#include <dart/math/lcp/projection/SymmetricPsorSolver.hpp>

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
    if (fixture.kind != dart::test::LcpFixtureKind::Standard)
      continue;
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
    if (fixture.kind != dart::test::LcpFixtureKind::Standard)
      continue;
    ExpectSolverPassesFixture(solver, fixture, options, 1e-6, false);
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
    if (fixture.kind != dart::test::LcpFixtureKind::Standard)
      continue;
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
    if (fixture.kind != dart::test::LcpFixtureKind::Standard)
      continue;
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
    if (fixture.kind != dart::test::LcpFixtureKind::Standard)
      continue;
    ExpectSolverPassesFixture(solver, fixture, options, 1e-6, false);
  }
}
