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

#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>

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

std::filesystem::path dartSourcePath()
{
  return std::filesystem::path(DART_SOURCE_DIR);
}

std::vector<std::string> readDocumentedSolverManifestNames()
{
  const auto path
      = dartSourcePath() / "docs/background/lcp/07_selection-guide.md";
  std::ifstream input(path);
  if (!input) {
    return {};
  }

  constexpr std::string_view kBegin
      = "<!-- dart-lcp-solver-manifest: begin -->";
  constexpr std::string_view kEnd = "<!-- dart-lcp-solver-manifest: end -->";

  bool inManifestBlock = false;
  std::vector<std::string> names;
  std::string line;
  while (std::getline(input, line)) {
    if (line == kBegin) {
      inManifestBlock = true;
      continue;
    }
    if (line == kEnd) {
      break;
    }
    if (!inManifestBlock) {
      continue;
    }

    if (line.rfind("- ", 0) != 0) {
      continue;
    }

    const auto firstTick = line.find('`');
    if (firstTick == std::string::npos) {
      continue;
    }
    const auto secondTick = line.find('`', firstTick + 1);
    if (secondTick == std::string::npos) {
      continue;
    }
    names.push_back(line.substr(firstTick + 1, secondTick - firstTick - 1));
  }

  return names;
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
    EXPECT_TRUE(solver->supportsStandardLcp()) << solverCase.name;
    EXPECT_EQ(solver->supportsBoxedLcp(), solverCase.supportsBoxed)
        << solverCase.name;
    EXPECT_EQ(solver->supportsFrictionIndex(), solverCase.supportsFrictionIndex)
        << solverCase.name;
  }
}

TEST_F(AllSolversSmokeTest, SolverCapabilityPredicatesClassifyProblemForms)
{
  const auto standard = LcpProblemFactory::standard2dSpd();
  const auto boxed = LcpProblemFactory::boxed2dActiveUpper();
  const auto friction = LcpProblemFactory::singleContactFriction();
  const auto largeStandard = LcpProblemFactory::standard4dSpd();

  for (const auto& solverCase : kLcpSolverManifest) {
    const auto solver = solverCase.create();
    ASSERT_NE(solver, nullptr) << solverCase.name;
    EXPECT_EQ(solver->supportsProblem(standard.problem), true)
        << solverCase.name;
    EXPECT_EQ(solver->supportsProblem(boxed.problem), solverCase.supportsBoxed)
        << solverCase.name;
    EXPECT_EQ(
        solver->supportsProblem(friction.problem),
        solverCase.supportsFrictionIndex)
        << solverCase.name;
    EXPECT_EQ(
        solver->supportsProblem(largeStandard.problem),
        std::string_view(solverCase.name) != "Direct")
        << solverCase.name
        << " should report only actual native per-problem support";
  }
}

TEST_F(
    AllSolversSmokeTest,
    SolverCapabilityPredicatesUseDefaultToleranceForNearStandardForm)
{
  Eigen::Matrix2d A = Eigen::Matrix2d::Identity();
  Eigen::Vector2d b(1.0, 2.0);
  Eigen::Vector2d lo(1e-9, -1e-9);
  Eigen::Vector2d hi
      = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
  const LcpProblem problem(A, b, lo, hi);

  EXPECT_EQ(problem.getType(), LcpProblemType::Boxed);
  EXPECT_EQ(problem.getType(1e-8), LcpProblemType::Standard);

  for (const auto& solverCase : kLcpSolverManifest) {
    const auto solver = solverCase.create();
    ASSERT_NE(solver, nullptr) << solverCase.name;
    EXPECT_TRUE(solver->supportsProblem(problem))
        << solverCase.name
        << " should use its default tolerance for near-standard bounds";
    EXPECT_EQ(solver->supportsProblem(problem, 0.0), solverCase.supportsBoxed)
        << solverCase.name
        << " should still allow exact classification when requested";
    EXPECT_TRUE(solver->supportsProblem(problem, 1e-8)) << solverCase.name;
  }
}

TEST_F(AllSolversSmokeTest, DocumentedSolverAvailabilityMatchesManifest)
{
  const auto documentedNames = readDocumentedSolverManifestNames();
  ASSERT_EQ(documentedNames.size(), kLcpSolverManifest.size());

  for (std::size_t i = 0; i < kLcpSolverManifest.size(); ++i) {
    EXPECT_EQ(documentedNames[i], std::string(kLcpSolverManifest[i].name))
        << "docs/background/lcp/07_selection-guide.md entry " << i;
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

TEST_F(AllSolversSmokeTest, NearSingularStandardProblemProducesExpectedIterates)
{
  const auto problem = LcpProblemFactory::nearSingular4d();
  ASSERT_TRUE(problem.expectedSolution.has_value());
  EXPECT_EQ(problem.difficulty, ProblemDifficulty::NearSingular);
  EXPECT_EQ(problem.problem.getType(), LcpProblemType::Standard);

  for (const auto& solverCase : kLcpSolverManifest) {
    const auto solver = solverCase.create();
    ASSERT_NE(solver, nullptr) << solverCase.name;
    EXPECT_EQ(
        solver->supportsProblem(problem.problem),
        std::string_view(solverCase.name) != "Direct")
        << solverCase.name
        << " should report native support only for actual direct solves";

    Eigen::VectorXd x;
    LcpOptions options = LcpOptions::highAccuracy();
    options.validateSolution = false;
    auto result = solver->solve(problem.problem, x, options);

    EXPECT_TRUE(
        result.status == LcpSolverStatus::Success
        || result.status == LcpSolverStatus::MaxIterations)
        << solverCase.name
        << " failed on near-singular standard problem: " << result.message;
    EXPECT_TRUE(x.allFinite())
        << solverCase.name << " produced non-finite values";
    ASSERT_EQ(x.size(), problem.expectedSolution->size()) << solverCase.name;
    EXPECT_LE((x - *problem.expectedSolution).lpNorm<Eigen::Infinity>(), 2e-4)
        << solverCase.name << " should stay comparable on " << problem.name;
  }
}

TEST_F(AllSolversSmokeTest, BoxedProblemHandledCorrectly)
{
  auto problem = LcpProblemFactory::boxed2dActiveUpper();
  EXPECT_FALSE(problem.problem.isStandardLcp());
  EXPECT_TRUE(problem.problem.isBoxedLcp());
  EXPECT_FALSE(problem.problem.hasFrictionIndex());

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
  EXPECT_FALSE(problem.problem.isStandardLcp());
  EXPECT_FALSE(problem.problem.isBoxedLcp());
  EXPECT_TRUE(problem.problem.hasFrictionIndex());

  for (const auto& solverCase : kLcpSolverManifest) {
    const auto solver = solverCase.create();
    Eigen::VectorXd x;
    LcpOptions options = solver->getDefaultOptions();
    options.maxIterations = 1000;
    auto result = solver->solve(problem.problem, x, options);

    if (!solverCase.supportsFrictionIndex) {
      EXPECT_TRUE(result.succeeded())
          << solverCase.name
          << " should delegate friction-index problems through Dantzig: "
          << result.message;
      expectWithinEffectiveBounds(
          std::string(solverCase.name), problem.problem, x);
      continue;
    }

    // Smoke test: just verify no crash and finite output
    // Some iterative solvers may not converge on friction problems
    EXPECT_EQ(x.size(), problem.problem.b.size()) << solverCase.name;
    EXPECT_TRUE(x.allFinite())
        << solverCase.name << " produced non-finite values";
  }
}

TEST_F(AllSolversSmokeTest, ActiveFrictionProblemStaysComparable)
{
  auto problem = LcpProblemFactory::activeFrictionIndexContact();
  ASSERT_TRUE(problem.expectedSolution.has_value());
  EXPECT_FALSE(problem.problem.isStandardLcp());
  EXPECT_FALSE(problem.problem.isBoxedLcp());
  EXPECT_TRUE(problem.problem.hasFrictionIndex());

  for (const auto& solverCase : kLcpSolverManifest) {
    const auto solver = solverCase.create();
    Eigen::VectorXd x;
    LcpOptions options = solver->getDefaultOptions();
    options.maxIterations = 2000;
    options.absoluteTolerance = 1e-8;
    options.relativeTolerance = 1e-8;
    options.complementarityTolerance = 1e-8;
    options.validateSolution = false;
    auto result = solver->solve(problem.problem, x, options);

    EXPECT_TRUE(producedIterate(result))
        << solverCase.name << " failed on " << problem.name << ": "
        << result.message;
    EXPECT_TRUE(x.allFinite())
        << solverCase.name << " produced non-finite values";
    ASSERT_EQ(x.size(), problem.expectedSolution->size()) << solverCase.name;
    expectWithinEffectiveBounds(
        std::string(solverCase.name), problem.problem, x);
    EXPECT_LE((x - *problem.expectedSolution).lpNorm<Eigen::Infinity>(), 2e-4)
        << solverCase.name << " should stay comparable on " << problem.name;
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
