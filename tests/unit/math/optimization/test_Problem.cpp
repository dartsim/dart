/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the BSD-style License.
 */

#include "dart/math/optimization/Problem.hpp"

#include <gtest/gtest.h>

using namespace dart::math;

namespace {

class SimpleFunction : public Function
{
public:
  explicit SimpleFunction(const std::string& name = "SimpleFunction")
    : Function(name)
  {
  }

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

} // namespace

TEST(ProblemTest, ConstructorInitializesDimension)
{
  Problem prob(3);
  EXPECT_EQ(prob.getDimension(), 3u);
  EXPECT_EQ(prob.getInitialGuess().size(), 3);
  EXPECT_EQ(prob.getLowerBounds().size(), 3);
  EXPECT_EQ(prob.getUpperBounds().size(), 3);
}

TEST(ProblemTest, SetDimensionClearsSeeds)
{
  Problem prob(2);
  prob.addSeed(Eigen::Vector2d(1.0, 2.0));
  EXPECT_EQ(prob.getSeeds().size(), 1u);

  prob.setDimension(3);
  EXPECT_EQ(prob.getDimension(), 3u);
  EXPECT_EQ(prob.getSeeds().size(), 0u);
}

TEST(ProblemTest, AddAndGetEqConstraints)
{
  Problem prob(2);
  EXPECT_EQ(prob.getNumEqConstraints(), 0u);

  auto constraint = std::make_shared<SimpleFunction>("eq1");
  prob.addEqConstraint(constraint);
  EXPECT_EQ(prob.getNumEqConstraints(), 1u);
  EXPECT_EQ(prob.getEqConstraint(0), constraint);
}

TEST(ProblemTest, AddAndGetIneqConstraints)
{
  Problem prob(2);
  EXPECT_EQ(prob.getNumIneqConstraints(), 0u);

  auto constraint = std::make_shared<SimpleFunction>("ineq1");
  prob.addIneqConstraint(constraint);
  EXPECT_EQ(prob.getNumIneqConstraints(), 1u);
  EXPECT_EQ(prob.getIneqConstraint(0), constraint);
}

TEST(ProblemTest, RemoveEqConstraint)
{
  Problem prob(2);
  auto c1 = std::make_shared<SimpleFunction>("c1");
  auto c2 = std::make_shared<SimpleFunction>("c2");
  prob.addEqConstraint(c1);
  prob.addEqConstraint(c2);
  EXPECT_EQ(prob.getNumEqConstraints(), 2u);

  prob.removeEqConstraint(c1);
  EXPECT_EQ(prob.getNumEqConstraints(), 1u);
  EXPECT_EQ(prob.getEqConstraint(0), c2);
}

TEST(ProblemTest, RemoveIneqConstraint)
{
  Problem prob(2);
  auto c1 = std::make_shared<SimpleFunction>("c1");
  auto c2 = std::make_shared<SimpleFunction>("c2");
  prob.addIneqConstraint(c1);
  prob.addIneqConstraint(c2);
  EXPECT_EQ(prob.getNumIneqConstraints(), 2u);

  prob.removeIneqConstraint(c1);
  EXPECT_EQ(prob.getNumIneqConstraints(), 1u);
  EXPECT_EQ(prob.getIneqConstraint(0), c2);
}

TEST(ProblemTest, RemoveAllConstraints)
{
  Problem prob(2);
  prob.addEqConstraint(std::make_shared<SimpleFunction>());
  prob.addEqConstraint(std::make_shared<SimpleFunction>());
  prob.addIneqConstraint(std::make_shared<SimpleFunction>());

  prob.removeAllEqConstraints();
  EXPECT_EQ(prob.getNumEqConstraints(), 0u);
  EXPECT_EQ(prob.getNumIneqConstraints(), 1u);

  prob.removeAllIneqConstraints();
  EXPECT_EQ(prob.getNumIneqConstraints(), 0u);
}

TEST(ProblemTest, SetAndGetObjective)
{
  Problem prob(2);
  EXPECT_EQ(prob.getObjective(), nullptr);

  auto obj = std::make_shared<SimpleFunction>("objective");
  prob.setObjective(obj);
  EXPECT_EQ(prob.getObjective(), obj);
}

TEST(ProblemTest, SetAndGetBounds)
{
  Problem prob(2);

  Eigen::Vector2d lb(-1.0, -2.0);
  Eigen::Vector2d ub(1.0, 2.0);

  prob.setLowerBounds(lb);
  prob.setUpperBounds(ub);

  EXPECT_EQ(prob.getLowerBounds(), lb);
  EXPECT_EQ(prob.getUpperBounds(), ub);
}

TEST(ProblemTest, SeedsManagement)
{
  Problem prob(2);
  EXPECT_EQ(prob.getSeeds().size(), 0u);

  Eigen::Vector2d seed1(0.5, 0.5);
  Eigen::Vector2d seed2(0.8, 0.2);

  prob.addSeed(seed1);
  prob.addSeed(seed2);
  EXPECT_EQ(prob.getSeeds().size(), 2u);
  EXPECT_EQ(prob.getSeed(0), seed1);
  EXPECT_EQ(prob.getSeed(1), seed2);

  prob.clearAllSeeds();
  EXPECT_EQ(prob.getSeeds().size(), 0u);
}

TEST(ProblemTest, OptimalSolutionTracking)
{
  Problem prob(2);

  Eigen::Vector2d solution(1.5, 2.5);
  prob.setOptimalSolution(solution);
  prob.setOptimumValue(3.14);

  EXPECT_EQ(prob.getOptimalSolution(), solution);
  EXPECT_DOUBLE_EQ(prob.getOptimumValue(), 3.14);
}
