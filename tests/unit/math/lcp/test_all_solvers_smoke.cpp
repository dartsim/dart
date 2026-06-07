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

#include "tests/common/lcpsolver/lcp_problem_factory.hpp"
#include "tests/common/lcpsolver/lcp_solver_manifest.hpp"

#include <gtest/gtest.h>

#include <string>

#include <cmath>

using namespace dart::math;
using namespace dart::test;

namespace {

LcpProblemSupport supportFor(const ProblemCategory category)
{
  using enum ProblemCategory;

  switch (category) {
    case Standard:
      return LcpProblemSupport::Standard;
    case Boxed:
      return LcpProblemSupport::Boxed;
    case BoxedFriction:
      return LcpProblemSupport::FrictionIndex;
  }
  return LcpProblemSupport::Standard;
}

bool canSolve(
    const LcpSolverManifestEntry& solver, const FactoryProblem& problem)
{
  return supportsProblem(solver, supportFor(problem.category));
}

bool producedIterate(const LcpResult& result)
{
  return result.status == LcpSolverStatus::Success
         || result.status == LcpSolverStatus::MaxIterations;
}

void expectWithinEffectiveBounds(
    const std::string& solverName,
    const LcpProblem& problem,
    const Eigen::VectorXd& x)
{
  ASSERT_EQ(x.size(), problem.b.size()) << solverName;
  EXPECT_TRUE(x.allFinite()) << solverName << " produced non-finite values";

  for (Eigen::Index i = 0; i < x.size(); ++i) {
    double lo = problem.lo[i];
    double hi = problem.hi[i];
    if (problem.findex[i] >= 0) {
      const double limit = std::abs(hi * x[problem.findex[i]]);
      lo = -limit;
      hi = limit;
    }

    if (std::isfinite(lo)) {
      EXPECT_GE(x[i], lo - 1e-8) << solverName << " at index " << i;
    }
    if (std::isfinite(hi)) {
      EXPECT_LE(x[i], hi + 1e-8) << solverName << " at index " << i;
    }
  }
}

} // namespace

class AllSolversSmokeTest : public ::testing::Test
{
};

TEST_F(AllSolversSmokeTest, ManifestMatchesConstructedSolverMetadata)
{
  EXPECT_EQ(kLcpSolverManifest.size(), 24u);
  EXPECT_EQ(countSolversSupporting(LcpProblemSupport::Standard), 24u);
  EXPECT_EQ(countSolversSupporting(LcpProblemSupport::Boxed), 16u);
  EXPECT_EQ(countSolversSupporting(LcpProblemSupport::FrictionIndex), 16u);

  for (const auto& solverCase : kLcpSolverManifest) {
    const auto solver = solverCase.create();
    ASSERT_NE(solver, nullptr) << solverCase.name;
    EXPECT_EQ(solver->getName(), std::string(solverCase.name))
        << solverCase.name;
  }
}

TEST_F(AllSolversSmokeTest, EmptyProblemSucceeds)
{
  auto problem = LcpProblemFactory::empty();

  for (const auto& solverCase : kLcpSolverManifest) {
    const auto solver = solverCase.create();
    Eigen::VectorXd x;
    auto result
        = solver->solve(problem.problem, x, solver->getDefaultOptions());
    EXPECT_TRUE(result.succeeded())
        << solverCase.name << " failed on empty problem: " << result.message;
    EXPECT_EQ(x.size(), 0) << solverCase.name
                           << " should return empty solution";
  }
}

TEST_F(AllSolversSmokeTest, Trivial1dDoesNotCrash)
{
  auto problem = LcpProblemFactory::trivial1d();

  for (const auto& solverCase : kLcpSolverManifest) {
    if (!canSolve(solverCase, problem)) {
      continue;
    }

    const auto solver = solverCase.create();
    Eigen::VectorXd x;
    auto result
        = solver->solve(problem.problem, x, solver->getDefaultOptions());
    EXPECT_EQ(x.size(), 1) << solverCase.name
                           << " should return size-1 solution";
    EXPECT_TRUE(x.allFinite())
        << solverCase.name << " produced non-finite values";
  }
}

TEST_F(AllSolversSmokeTest, Standard2dDoesNotCrash)
{
  auto problem = LcpProblemFactory::standard2dSpd();

  for (const auto& solverCase : kLcpSolverManifest) {
    if (!canSolve(solverCase, problem)) {
      continue;
    }

    const auto solver = solverCase.create();
    Eigen::VectorXd x;
    LcpOptions options = solver->getDefaultOptions();
    options.maxIterations = 1000;
    auto result = solver->solve(problem.problem, x, options);

    EXPECT_EQ(x.size(), 2) << solverCase.name;
    EXPECT_TRUE(x.allFinite())
        << solverCase.name << " produced non-finite values";
  }
}

TEST_F(AllSolversSmokeTest, BoxedProblemHandledCorrectly)
{
  auto problem = LcpProblemFactory::boxed2dActiveUpper();

  for (const auto& solverCase : kLcpSolverManifest) {
    const auto solver = solverCase.create();
    if (!solverCase.supportsBoxed) {
      Eigen::VectorXd x;
      auto result
          = solver->solve(problem.problem, x, solver->getDefaultOptions());
      EXPECT_TRUE(
          result.status == LcpSolverStatus::Success
          || result.status == LcpSolverStatus::MaxIterations)
          << solverCase.name << " should handle boxed: " << result.message;
      continue;
    }

    Eigen::VectorXd x;
    LcpOptions options = solver->getDefaultOptions();
    options.maxIterations = 1000;
    auto result = solver->solve(problem.problem, x, options);

    EXPECT_TRUE(
        result.succeeded() || result.status == LcpSolverStatus::MaxIterations)
        << solverCase.name << " failed on boxed: " << result.message;
    EXPECT_TRUE(x.allFinite())
        << solverCase.name << " produced non-finite values";
  }
}

TEST_F(AllSolversSmokeTest, FrictionProblemDoesNotCrash)
{
  auto problem = LcpProblemFactory::singleContactFriction();

  for (const auto& solverCase : kLcpSolverManifest) {
    if (!solverCase.supportsFrictionIndex) {
      continue;
    }

    const auto solver = solverCase.create();
    Eigen::VectorXd x;
    LcpOptions options = solver->getDefaultOptions();
    options.maxIterations = 1000;
    auto result = solver->solve(problem.problem, x, options);

    // Smoke test: just verify no crash and finite output
    // Some iterative solvers may not converge on friction problems
    EXPECT_EQ(x.size(), problem.problem.b.size()) << solverCase.name;
    EXPECT_TRUE(x.allFinite())
        << solverCase.name << " produced non-finite values";
  }
}

TEST(AdvancedBoxedSolvers, MetadataAndParametersCanBeCustomized)
{
  AdmmSolver admm;
  AdmmSolver::Parameters admmParams;
  admmParams.rhoInit = 0.35;
  admmParams.muProx = 1e-8;
  admmParams.adaptiveRhoTolerance = 2.5;
  admmParams.adaptiveRho = false;
  admm.setParameters(admmParams);
  EXPECT_EQ(admm.getName(), "Admm");
  EXPECT_EQ(admm.getCategory(), "Other");
  EXPECT_DOUBLE_EQ(admm.getParameters().rhoInit, admmParams.rhoInit);
  EXPECT_DOUBLE_EQ(admm.getParameters().muProx, admmParams.muProx);
  EXPECT_DOUBLE_EQ(
      admm.getParameters().adaptiveRhoTolerance,
      admmParams.adaptiveRhoTolerance);
  EXPECT_EQ(admm.getParameters().adaptiveRho, admmParams.adaptiveRho);

  SapSolver sap;
  SapSolver::Parameters sapParams;
  sapParams.regularization = 1e-3;
  sapParams.armijosParameter = 5e-4;
  sapParams.backtrackingFactor = 0.25;
  sapParams.maxLineSearchIterations = 8;
  sap.setParameters(sapParams);
  EXPECT_EQ(sap.getName(), "Sap");
  EXPECT_EQ(sap.getCategory(), "Other");
  EXPECT_DOUBLE_EQ(
      sap.getParameters().regularization, sapParams.regularization);
  EXPECT_DOUBLE_EQ(
      sap.getParameters().armijosParameter, sapParams.armijosParameter);
  EXPECT_DOUBLE_EQ(
      sap.getParameters().backtrackingFactor, sapParams.backtrackingFactor);
  EXPECT_EQ(
      sap.getParameters().maxLineSearchIterations,
      sapParams.maxLineSearchIterations);

  BoxedSemiSmoothNewtonSolver boxedNewton;
  BoxedSemiSmoothNewtonSolver::Parameters newtonParams;
  newtonParams.maxLineSearchSteps = 12;
  newtonParams.stepReduction = 0.35;
  newtonParams.sufficientDecrease = 2e-4;
  newtonParams.minStep = 1e-10;
  newtonParams.jacobianRegularization = 1e-8;
  boxedNewton.setParameters(newtonParams);
  EXPECT_EQ(boxedNewton.getName(), "BoxedSemiSmoothNewton");
  EXPECT_EQ(boxedNewton.getCategory(), "Newton");
  EXPECT_EQ(
      boxedNewton.getParameters().maxLineSearchSteps,
      newtonParams.maxLineSearchSteps);
  EXPECT_DOUBLE_EQ(
      boxedNewton.getParameters().stepReduction, newtonParams.stepReduction);
  EXPECT_DOUBLE_EQ(
      boxedNewton.getParameters().sufficientDecrease,
      newtonParams.sufficientDecrease);
  EXPECT_DOUBLE_EQ(boxedNewton.getParameters().minStep, newtonParams.minStep);
  EXPECT_DOUBLE_EQ(
      boxedNewton.getParameters().jacobianRegularization,
      newtonParams.jacobianRegularization);
}

TEST(AdvancedBoxedSolvers, MixedBoundsWorkflowHonorsWarmStartsAndCustomOptions)
{
  const auto problem = LcpProblemFactory::boxed3dMixedBounds();

  AdmmSolver admm;
  AdmmSolver::Parameters admmParams;
  admmParams.rhoInit = 0.45;
  admmParams.muProx = 1e-8;
  admmParams.adaptiveRhoTolerance = 2.0;
  admmParams.adaptiveRho = true;
  LcpOptions admmOptions = admm.getDefaultOptions();
  admmOptions.maxIterations = 120;
  admmOptions.absoluteTolerance = 1e-7;
  admmOptions.validateSolution = false;
  admmOptions.warmStart = true;
  admmOptions.customOptions = &admmParams;
  Eigen::VectorXd admmX(3);
  admmX << -2.0, 2.0, 2.0;
  auto admmResult = admm.solve(problem.problem, admmX, admmOptions);
  EXPECT_TRUE(producedIterate(admmResult)) << admmResult.message;
  expectWithinEffectiveBounds(admm.getName(), problem.problem, admmX);

  SapSolver sap;
  SapSolver::Parameters sapParams;
  sapParams.regularization = 1e-3;
  sapParams.armijosParameter = 1e-4;
  sapParams.backtrackingFactor = 0.5;
  sapParams.maxLineSearchIterations = 12;
  LcpOptions sapOptions = sap.getDefaultOptions();
  sapOptions.maxIterations = 80;
  sapOptions.absoluteTolerance = 1e-7;
  sapOptions.validateSolution = false;
  sapOptions.warmStart = true;
  sapOptions.customOptions = &sapParams;
  Eigen::VectorXd sapX(3);
  sapX << -3.0, 3.0, 3.0;
  auto sapResult = sap.solve(problem.problem, sapX, sapOptions);
  EXPECT_TRUE(producedIterate(sapResult)) << sapResult.message;
  expectWithinEffectiveBounds(sap.getName(), problem.problem, sapX);

  BoxedSemiSmoothNewtonSolver boxedNewton;
  BoxedSemiSmoothNewtonSolver::Parameters newtonParams;
  newtonParams.maxLineSearchSteps = 16;
  newtonParams.stepReduction = 0.5;
  newtonParams.sufficientDecrease = 1e-4;
  newtonParams.minStep = 1e-10;
  newtonParams.jacobianRegularization = 1e-8;
  LcpOptions newtonOptions = boxedNewton.getDefaultOptions();
  newtonOptions.maxIterations = 80;
  newtonOptions.absoluteTolerance = 1e-8;
  newtonOptions.complementarityTolerance = 1e-8;
  newtonOptions.warmStart = true;
  newtonOptions.customOptions = &newtonParams;
  Eigen::VectorXd newtonX(3);
  newtonX << -3.0, 3.0, 3.0;
  auto newtonResult
      = boxedNewton.solve(problem.problem, newtonX, newtonOptions);
  EXPECT_TRUE(producedIterate(newtonResult)) << newtonResult.message;
  expectWithinEffectiveBounds(boxedNewton.getName(), problem.problem, newtonX);
}

TEST(AdvancedBoxedSolvers, InvalidMixedBoundsAreRejectedBeforeIteration)
{
  auto problem = LcpProblemFactory::boxed3dMixedBounds().problem;
  problem.lo[1] = 0.75;
  problem.hi[1] = -0.75;

  AdmmSolver admm;
  SapSolver sap;
  BoxedSemiSmoothNewtonSolver boxedNewton;

  Eigen::VectorXd x;
  auto result = admm.solve(problem, x, admm.getDefaultOptions());
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_FALSE(result.message.empty());

  result = sap.solve(problem, x, sap.getDefaultOptions());
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_FALSE(result.message.empty());

  result = boxedNewton.solve(problem, x, boxedNewton.getDefaultOptions());
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
  EXPECT_FALSE(result.message.empty());
}

TEST(AdvancedBoxedSolvers, BoxedNewtonReportsConfiguredLineSearchFailure)
{
  const auto problem = LcpProblemFactory::standard2dSpd();
  BoxedSemiSmoothNewtonSolver solver;
  BoxedSemiSmoothNewtonSolver::Parameters params;
  params.maxLineSearchSteps = 0;

  LcpOptions options = solver.getDefaultOptions();
  options.maxIterations = 4;
  options.validateSolution = false;
  options.customOptions = &params;

  Eigen::VectorXd x(2);
  x << 0.0, 0.0;
  const auto result = solver.solve(problem.problem, x, options);

  EXPECT_EQ(result.status, LcpSolverStatus::Failed);
  EXPECT_EQ(result.message, "Line search failed");
  EXPECT_TRUE(x.allFinite());
}
