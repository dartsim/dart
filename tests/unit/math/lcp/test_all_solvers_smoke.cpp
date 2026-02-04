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

#include <dart/math/lcp/all.hpp>

#include <gtest/gtest.h>

#include <memory>

using namespace dart::math;
using namespace dart::test;

namespace {

struct SolverTestCase
{
  std::string name;
  std::unique_ptr<LcpSolver> solver;
  bool supportsStandard;
  bool supportsBoxed;
  bool supportsFindex;
};

std::vector<SolverTestCase> createAllSolvers()
{
  std::vector<SolverTestCase> solvers;

  solvers.push_back(
      {"Dantzig", std::make_unique<DantzigSolver>(), true, true, true});
  solvers.push_back(
      {"Lemke", std::make_unique<LemkeSolver>(), true, false, false});
  solvers.push_back(
      {"Baraff", std::make_unique<BaraffSolver>(), true, false, false});
  solvers.push_back(
      {"Direct", std::make_unique<DirectSolver>(), true, false, false});

  solvers.push_back({"Pgs", std::make_unique<PgsSolver>(), true, true, true});
  solvers.push_back(
      {"SymmetricPsor",
       std::make_unique<SymmetricPsorSolver>(),
       true,
       true,
       true});
  solvers.push_back(
      {"Jacobi", std::make_unique<JacobiSolver>(), true, true, true});
  solvers.push_back(
      {"RedBlackGaussSeidel",
       std::make_unique<RedBlackGaussSeidelSolver>(),
       true,
       true,
       true});
  solvers.push_back(
      {"BlockedJacobi",
       std::make_unique<BlockedJacobiSolver>(),
       true,
       true,
       true});
  solvers.push_back({"Bgs", std::make_unique<BgsSolver>(), true, true, true});
  solvers.push_back({"Nncg", std::make_unique<NncgSolver>(), true, true, true});
  solvers.push_back(
      {"SubspaceMinimization",
       std::make_unique<SubspaceMinimizationSolver>(),
       true,
       true,
       true});
  solvers.push_back({"Apgd", std::make_unique<ApgdSolver>(), true, true, true});
  solvers.push_back({"Tgs", std::make_unique<TgsSolver>(), true, true, true});

  solvers.push_back(
      {"MinimumMapNewton",
       std::make_unique<MinimumMapNewtonSolver>(),
       true,
       false,
       false});
  solvers.push_back(
      {"FischerBurmeisterNewton",
       std::make_unique<FischerBurmeisterNewtonSolver>(),
       true,
       false,
       false});
  solvers.push_back(
      {"PenalizedFischerBurmeisterNewton",
       std::make_unique<PenalizedFischerBurmeisterNewtonSolver>(),
       true,
       false,
       false});

  solvers.push_back(
      {"InteriorPoint",
       std::make_unique<InteriorPointSolver>(),
       true,
       false,
       false});
  solvers.push_back(
      {"Mprgp", std::make_unique<MprgpSolver>(), true, false, false});
  solvers.push_back(
      {"ShockPropagation",
       std::make_unique<ShockPropagationSolver>(),
       true,
       true,
       true});
  solvers.push_back(
      {"Staggering", std::make_unique<StaggeringSolver>(), true, true, true});
  solvers.push_back({"Admm", std::make_unique<AdmmSolver>(), true, true, true});
  solvers.push_back({"Sap", std::make_unique<SapSolver>(), true, true, true});
  solvers.push_back(
      {"BoxedSemiSmoothNewton",
       std::make_unique<BoxedSemiSmoothNewtonSolver>(),
       true,
       true,
       true});

  return solvers;
}

bool canSolve(const SolverTestCase& solver, const FactoryProblem& problem)
{
  switch (problem.category) {
    case ProblemCategory::Standard:
      return solver.supportsStandard;
    case ProblemCategory::Boxed:
      return solver.supportsBoxed;
    case ProblemCategory::BoxedFriction:
      return solver.supportsFindex;
  }
  return false;
}

} // namespace

class AllSolversSmokeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    mSolvers = createAllSolvers();
  }

  std::vector<SolverTestCase> mSolvers;
};

TEST_F(AllSolversSmokeTest, EmptyProblemSucceeds)
{
  auto problem = LcpProblemFactory::empty();

  for (auto& solver : mSolvers) {
    Eigen::VectorXd x;
    auto result = solver.solver->solve(
        problem.problem, x, solver.solver->getDefaultOptions());
    EXPECT_TRUE(result.succeeded())
        << solver.name << " failed on empty problem: " << result.message;
    EXPECT_EQ(x.size(), 0) << solver.name << " should return empty solution";
  }
}

TEST_F(AllSolversSmokeTest, Trivial1dDoesNotCrash)
{
  auto problem = LcpProblemFactory::trivial1d();

  for (auto& solver : mSolvers) {
    if (!canSolve(solver, problem)) {
      continue;
    }

    Eigen::VectorXd x;
    auto result = solver.solver->solve(
        problem.problem, x, solver.solver->getDefaultOptions());
    EXPECT_EQ(x.size(), 1) << solver.name << " should return size-1 solution";
    EXPECT_TRUE(x.allFinite()) << solver.name << " produced non-finite values";
  }
}

TEST_F(AllSolversSmokeTest, Standard2dDoesNotCrash)
{
  auto problem = LcpProblemFactory::standard2dSpd();

  for (auto& solver : mSolvers) {
    if (!canSolve(solver, problem)) {
      continue;
    }

    Eigen::VectorXd x;
    LcpOptions options = solver.solver->getDefaultOptions();
    options.maxIterations = 1000;
    auto result = solver.solver->solve(problem.problem, x, options);

    EXPECT_EQ(x.size(), 2) << solver.name;
    EXPECT_TRUE(x.allFinite()) << solver.name << " produced non-finite values";
  }
}

TEST_F(AllSolversSmokeTest, BoxedProblemHandledCorrectly)
{
  auto problem = LcpProblemFactory::boxed2dActiveUpper();

  for (auto& solver : mSolvers) {
    if (!solver.supportsBoxed) {
      Eigen::VectorXd x;
      auto result = solver.solver->solve(
          problem.problem, x, solver.solver->getDefaultOptions());
      EXPECT_TRUE(
          result.status == LcpSolverStatus::Success
          || result.status == LcpSolverStatus::MaxIterations)
          << solver.name << " should handle boxed: " << result.message;
      continue;
    }

    Eigen::VectorXd x;
    LcpOptions options = solver.solver->getDefaultOptions();
    options.maxIterations = 1000;
    auto result = solver.solver->solve(problem.problem, x, options);

    EXPECT_TRUE(
        result.succeeded() || result.status == LcpSolverStatus::MaxIterations)
        << solver.name << " failed on boxed: " << result.message;
    EXPECT_TRUE(x.allFinite()) << solver.name << " produced non-finite values";
  }
}

TEST_F(AllSolversSmokeTest, FrictionProblemDoesNotCrash)
{
  auto problem = LcpProblemFactory::singleContactFriction();

  for (auto& solver : mSolvers) {
    if (!solver.supportsFindex) {
      continue;
    }

    Eigen::VectorXd x;
    LcpOptions options = solver.solver->getDefaultOptions();
    options.maxIterations = 1000;
    auto result = solver.solver->solve(problem.problem, x, options);

    // Smoke test: just verify no crash and finite output
    // Some iterative solvers may not converge on friction problems
    EXPECT_EQ(x.size(), problem.problem.b.size()) << solver.name;
    EXPECT_TRUE(x.allFinite()) << solver.name << " produced non-finite values";
  }
}
