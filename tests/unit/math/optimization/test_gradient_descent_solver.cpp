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

#include "dart/math/optimization/function.hpp"
#include "dart/math/optimization/gradient_descent_solver.hpp"
#include "dart/math/optimization/problem.hpp"

#include <gtest/gtest.h>

using namespace dart::math;

namespace {

class QuadraticFunction : public Function
{
public:
  QuadraticFunction() : Function("Quadratic") {}

  double eval(const Eigen::VectorXd& x) override
  {
    return x.squaredNorm();
  }

  void evalGradient(
      const Eigen::VectorXd& x, Eigen::Map<Eigen::VectorXd> grad) override
  {
    grad = 2.0 * x;
  }
};

class RosenbrockFunction : public Function
{
public:
  RosenbrockFunction() : Function("Rosenbrock") {}

  double eval(const Eigen::VectorXd& x) override
  {
    double sum = 0.0;
    for (int i = 0; i < x.size() - 1; ++i) {
      double t1 = x[i + 1] - x[i] * x[i];
      double t2 = 1.0 - x[i];
      sum += 100.0 * t1 * t1 + t2 * t2;
    }
    return sum;
  }

  void evalGradient(
      const Eigen::VectorXd& x, Eigen::Map<Eigen::VectorXd> grad) override
  {
    grad.setZero();
    for (int i = 0; i < x.size() - 1; ++i) {
      double t1 = x[i + 1] - x[i] * x[i];
      grad[i] += -400.0 * x[i] * t1 - 2.0 * (1.0 - x[i]);
      grad[i + 1] += 200.0 * t1;
    }
  }
};

class LinearConstraint : public Function
{
public:
  LinearConstraint(double target) : Function("Linear"), mTarget(target) {}

  double eval(const Eigen::VectorXd& x) override
  {
    return x.sum() - mTarget;
  }

  void evalGradient(
      const Eigen::VectorXd& /*x*/, Eigen::Map<Eigen::VectorXd> grad) override
  {
    grad.setOnes();
  }

private:
  double mTarget;
};

} // namespace

//==============================================================================
TEST(GradientDescentSolver, DefaultConstruction)
{
  GradientDescentSolver solver;
  EXPECT_EQ(solver.getType(), GradientDescentSolver::Type);
  EXPECT_GT(solver.getStepSize(), 0.0);
  EXPECT_GE(solver.getMaxAttempts(), 1u);
}

//==============================================================================
TEST(GradientDescentSolver, ConstructWithProblem)
{
  auto problem = std::make_shared<Problem>(2);
  problem->setObjective(std::make_shared<QuadraticFunction>());

  GradientDescentSolver solver(problem);
  EXPECT_EQ(solver.getProblem(), problem);
}

//==============================================================================
TEST(GradientDescentSolver, ConstructWithProperties)
{
  GradientDescentSolver::Properties props;
  props.mTolerance = 1e-8;
  props.mNumMaxIterations = 500;

  GradientDescentSolver solver(props);
  EXPECT_DOUBLE_EQ(solver.getTolerance(), 1e-8);
  EXPECT_EQ(solver.getNumMaxIterations(), 500u);
}

//==============================================================================
TEST(GradientDescentSolver, SetStepSize)
{
  GradientDescentSolver solver;
  solver.setStepSize(0.05);
  EXPECT_DOUBLE_EQ(solver.getStepSize(), 0.05);

  solver.setStepSize(0.2);
  EXPECT_DOUBLE_EQ(solver.getStepSize(), 0.2);
}

//==============================================================================
TEST(GradientDescentSolver, SetMaxAttempts)
{
  GradientDescentSolver solver;
  solver.setMaxAttempts(5);
  EXPECT_EQ(solver.getMaxAttempts(), 5u);

  solver.setMaxAttempts(10);
  EXPECT_EQ(solver.getMaxAttempts(), 10u);
}

//==============================================================================
TEST(GradientDescentSolver, SetPerturbationStep)
{
  GradientDescentSolver solver;
  solver.setPerturbationStep(50);
  EXPECT_EQ(solver.getPerturbationStep(), 50u);

  solver.setPerturbationStep(0);
  EXPECT_EQ(solver.getPerturbationStep(), 0u);
}

//==============================================================================
TEST(GradientDescentSolver, SetMaxPerturbationFactor)
{
  GradientDescentSolver solver;
  solver.setMaxPerturbationFactor(0.5);
  EXPECT_DOUBLE_EQ(solver.getMaxPerturbationFactor(), 0.5);
}

//==============================================================================
TEST(GradientDescentSolver, SetDefaultConstraintWeight)
{
  GradientDescentSolver solver;
  solver.setDefaultConstraintWeight(2.0);
  EXPECT_DOUBLE_EQ(solver.getDefaultConstraintWeight(), 2.0);
}

//==============================================================================
TEST(GradientDescentSolver, ConstraintWeights)
{
  GradientDescentSolver solver;

  Eigen::VectorXd eqWeights(2);
  eqWeights << 1.0, 2.0;
  solver.getEqConstraintWeights() = eqWeights;
  EXPECT_TRUE(solver.getEqConstraintWeights().isApprox(eqWeights));

  Eigen::VectorXd ineqWeights(3);
  ineqWeights << 0.5, 1.0, 1.5;
  solver.getIneqConstraintWeights() = ineqWeights;
  EXPECT_TRUE(solver.getIneqConstraintWeights().isApprox(ineqWeights));

  const GradientDescentSolver& constSolver = solver;
  EXPECT_TRUE(constSolver.getEqConstraintWeights().isApprox(eqWeights));
  EXPECT_TRUE(constSolver.getIneqConstraintWeights().isApprox(ineqWeights));
}

//==============================================================================
TEST(GradientDescentSolver, SolveQuadratic)
{
  auto problem = std::make_shared<Problem>(2);
  problem->setObjective(std::make_shared<QuadraticFunction>());
  problem->setLowerBounds(Eigen::Vector2d(-10, -10));
  problem->setUpperBounds(Eigen::Vector2d(10, 10));
  problem->setInitialGuess(Eigen::Vector2d(5, 5));

  GradientDescentSolver solver(problem);
  solver.setStepSize(0.1);
  solver.setNumMaxIterations(1000);

  bool solved = solver.solve();
  EXPECT_TRUE(solved);

  Eigen::VectorXd solution = problem->getOptimalSolution();
  EXPECT_LT(solution.norm(), 1.0);
}

//==============================================================================
TEST(GradientDescentSolver, GetLastConfiguration)
{
  auto problem = std::make_shared<Problem>(2);
  problem->setObjective(std::make_shared<QuadraticFunction>());
  problem->setLowerBounds(Eigen::Vector2d(-10, -10));
  problem->setUpperBounds(Eigen::Vector2d(10, 10));
  problem->setInitialGuess(Eigen::Vector2d(1, 1));

  GradientDescentSolver solver(problem);
  solver.setNumMaxIterations(10);
  solver.solve();

  Eigen::VectorXd lastConfig = solver.getLastConfiguration();
  EXPECT_EQ(lastConfig.size(), 2);
  EXPECT_TRUE(lastConfig.array().isFinite().all());
}

//==============================================================================
TEST(GradientDescentSolver, GetLastNumIterations)
{
  auto problem = std::make_shared<Problem>(2);
  problem->setObjective(std::make_shared<QuadraticFunction>());
  problem->setLowerBounds(Eigen::Vector2d(-10, -10));
  problem->setUpperBounds(Eigen::Vector2d(10, 10));
  problem->setInitialGuess(Eigen::Vector2d(1, 1));

  GradientDescentSolver solver(problem);
  solver.setNumMaxIterations(50);
  solver.solve();

  std::size_t iters = solver.getLastNumIterations();
  EXPECT_GT(iters, 0u);
}

//==============================================================================
TEST(GradientDescentSolver, Clone)
{
  GradientDescentSolver solver;
  solver.setStepSize(0.15);
  solver.setMaxAttempts(3);
  solver.setPerturbationStep(25);

  auto cloned = solver.clone();
  ASSERT_NE(cloned, nullptr);

  auto* gdCloned = dynamic_cast<GradientDescentSolver*>(cloned.get());
  ASSERT_NE(gdCloned, nullptr);
  EXPECT_DOUBLE_EQ(gdCloned->getStepSize(), 0.15);
  EXPECT_EQ(gdCloned->getMaxAttempts(), 3u);
  EXPECT_EQ(gdCloned->getPerturbationStep(), 25u);
}

//==============================================================================
TEST(GradientDescentSolver, Copy)
{
  GradientDescentSolver solver1;
  solver1.setStepSize(0.2);
  solver1.setMaxAttempts(4);

  GradientDescentSolver solver2;
  solver2.copy(solver1);

  EXPECT_DOUBLE_EQ(solver2.getStepSize(), 0.2);
  EXPECT_EQ(solver2.getMaxAttempts(), 4u);
}

//==============================================================================
TEST(GradientDescentSolver, AssignmentOperator)
{
  GradientDescentSolver solver1;
  solver1.setStepSize(0.25);
  solver1.setMaxAttempts(5);

  GradientDescentSolver solver2;
  solver2 = solver1;

  EXPECT_DOUBLE_EQ(solver2.getStepSize(), 0.25);
  EXPECT_EQ(solver2.getMaxAttempts(), 5u);
}

//==============================================================================
TEST(GradientDescentSolver, SetProperties)
{
  GradientDescentSolver solver;

  GradientDescentSolver::Properties props;
  props.mNumMaxIterations = 200;

  solver.setProperties(props);
  EXPECT_EQ(solver.getNumMaxIterations(), 200u);
}

//==============================================================================
TEST(GradientDescentSolver, SetUniqueProperties)
{
  GradientDescentSolver solver;

  GradientDescentSolver::UniqueProperties uniqueProps;
  uniqueProps.mStepSize = 0.3;
  uniqueProps.mMaxAttempts = 7;
  uniqueProps.mPerturbationStep = 30;
  uniqueProps.mMaxPerturbationFactor = 0.8;
  uniqueProps.mDefaultConstraintWeight = 1.5;

  solver.setProperties(uniqueProps);
  EXPECT_DOUBLE_EQ(solver.getStepSize(), 0.3);
  EXPECT_EQ(solver.getMaxAttempts(), 7u);
  EXPECT_EQ(solver.getPerturbationStep(), 30u);
  EXPECT_DOUBLE_EQ(solver.getMaxPerturbationFactor(), 0.8);
  EXPECT_DOUBLE_EQ(solver.getDefaultConstraintWeight(), 1.5);
}

//==============================================================================
TEST(GradientDescentSolver, GetGradientDescentProperties)
{
  GradientDescentSolver solver;
  solver.setStepSize(0.12);
  solver.setMaxAttempts(6);

  auto props = solver.getGradientDescentProperties();
  EXPECT_DOUBLE_EQ(props.mStepSize, 0.12);
  EXPECT_EQ(props.mMaxAttempts, 6u);
}

//==============================================================================
TEST(GradientDescentSolver, RandomizeConfiguration)
{
  auto problem = std::make_shared<Problem>(3);
  problem->setLowerBounds(Eigen::Vector3d(-5, -5, -5));
  problem->setUpperBounds(Eigen::Vector3d(5, 5, 5));

  GradientDescentSolver solver(problem);

  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  solver.randomizeConfiguration(x);

  for (int i = 0; i < 3; ++i) {
    EXPECT_GE(x[i], -5.0);
    EXPECT_LE(x[i], 5.0);
  }
}

//==============================================================================
TEST(GradientDescentSolver, ClampToBoundary)
{
  auto problem = std::make_shared<Problem>(3);
  problem->setLowerBounds(Eigen::Vector3d(-1, -1, -1));
  problem->setUpperBounds(Eigen::Vector3d(1, 1, 1));

  GradientDescentSolver solver(problem);

  Eigen::VectorXd x(3);
  x << -5.0, 0.0, 5.0;
  solver.clampToBoundary(x);

  EXPECT_DOUBLE_EQ(x[0], -1.0);
  EXPECT_DOUBLE_EQ(x[1], 0.0);
  EXPECT_DOUBLE_EQ(x[2], 1.0);
}

//==============================================================================
TEST(GradientDescentSolver, SolveWithConstraint)
{
  auto problem = std::make_shared<Problem>(2);
  problem->setObjective(std::make_shared<QuadraticFunction>());
  problem->addEqConstraint(std::make_shared<LinearConstraint>(1.0));
  problem->setLowerBounds(Eigen::Vector2d(-10, -10));
  problem->setUpperBounds(Eigen::Vector2d(10, 10));
  problem->setInitialGuess(Eigen::Vector2d(0.5, 0.5));

  GradientDescentSolver solver(problem);
  solver.setStepSize(0.05);
  solver.setNumMaxIterations(500);
  solver.setDefaultConstraintWeight(10.0);

  solver.solve();

  Eigen::VectorXd solution = problem->getOptimalSolution();
  EXPECT_TRUE(solution.array().isFinite().all());
}

//==============================================================================
TEST(GradientDescentSolver, SolveWithPerturbation)
{
  auto problem = std::make_shared<Problem>(2);
  problem->setObjective(std::make_shared<QuadraticFunction>());
  problem->setLowerBounds(Eigen::Vector2d(-10, -10));
  problem->setUpperBounds(Eigen::Vector2d(10, 10));
  problem->setInitialGuess(Eigen::Vector2d(5, 5));

  GradientDescentSolver solver(problem);
  solver.setStepSize(0.1);
  solver.setNumMaxIterations(100);
  solver.setPerturbationStep(20);
  solver.setMaxPerturbationFactor(0.5);

  solver.solve();

  Eigen::VectorXd solution = problem->getOptimalSolution();
  EXPECT_TRUE(solution.array().isFinite().all());
}

//==============================================================================
TEST(GradientDescentSolver, MultipleAttempts)
{
  auto problem = std::make_shared<Problem>(2);
  problem->setObjective(std::make_shared<RosenbrockFunction>());
  problem->setLowerBounds(Eigen::Vector2d(-5, -5));
  problem->setUpperBounds(Eigen::Vector2d(5, 5));
  problem->setInitialGuess(Eigen::Vector2d(-2, 2));

  GradientDescentSolver solver(problem);
  solver.setStepSize(0.001);
  solver.setNumMaxIterations(100);
  solver.setMaxAttempts(3);

  solver.solve();

  Eigen::VectorXd solution = problem->getOptimalSolution();
  EXPECT_TRUE(solution.array().isFinite().all());
}
