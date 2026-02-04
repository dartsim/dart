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

#include "helpers/gtest_utils.hpp"

#include "dart/common/logging.hpp"
#include "dart/config.hpp"
#include "dart/dynamics/free_joint.hpp"
#include "dart/dynamics/inverse_kinematics.hpp"
#include "dart/dynamics/skeleton.hpp"
#include "dart/math/optimization/function.hpp"
#include "dart/math/optimization/gradient_descent_solver.hpp"
#include "dart/math/optimization/problem.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <fstream>
#include <iostream>
#include <sstream>

#include <cstdio>

using namespace std;
using namespace Eigen;
using namespace dart::math;
using namespace dart::dynamics;

//==============================================================================
/// @brief class SampleObjFunc
class SampleObjFunc : public Function
{
public:
  /// @brief Constructor
  SampleObjFunc() : Function() {}

  /// @brief Destructor
  virtual ~SampleObjFunc() {}

  /// @copydoc Function::eval
  double eval(const Eigen::VectorXd& _x) override
  {
    return std::sqrt(_x[1]);
  }

  /// @copydoc Function::evalGradient
  void evalGradient(
      const Eigen::VectorXd& _x, Eigen::Map<Eigen::VectorXd> _grad) override
  {
    _grad[0] = 0.0;
    _grad[1] = 0.5 / std::sqrt(_x[1]);
  }
};

//==============================================================================
class SampleConstFunc : public Function
{
public:
  /// @brief Constructor
  SampleConstFunc(double _a, double _b) : Function(), mA(_a), mB(_b) {}

  /// @brief Destructor
  virtual ~SampleConstFunc() {}

  /// @copydoc Function::eval
  double eval(const Eigen::VectorXd& _x) override
  {
    return ((mA * _x[0] + mB) * (mA * _x[0] + mB) * (mA * _x[0] + mB) - _x[1]);
  }

  /// @copydoc Function::evalGradient
  void evalGradient(
      const Eigen::VectorXd& _x, Eigen::Map<Eigen::VectorXd> _grad) override
  {
    _grad[0] = 3 * mA * (mA * _x[0] + mB) * (mA * _x[0] + mB);
    _grad[1] = -1.0;
  }

private:
  /// @brief Data
  double mA;

  /// @brief Data
  double mB;
};

//==============================================================================
TEST(Optimizer, GradientDescent)
{
  std::shared_ptr<Problem> prob = std::make_shared<Problem>(2);

  prob->setLowerBounds(Eigen::Vector2d(-HUGE_VAL, 0));
  prob->setInitialGuess(Eigen::Vector2d(1.234, 5.678));

  FunctionPtr obj = std::make_shared<SampleObjFunc>();
  prob->setObjective(obj);

  GradientDescentSolver solver(prob);
  EXPECT_TRUE(solver.solve());

  double minF = prob->getOptimumValue();
  Eigen::VectorXd optX = prob->getOptimalSolution();

  EXPECT_NEAR(minF, 0, 1e-6);
  EXPECT_EQ(optX.size(), static_cast<int>(prob->getDimension()));
  EXPECT_NEAR(optX[0], 1.234, 0.0);
  EXPECT_NEAR(optX[1], 0.0, solver.getTolerance());
}

//==============================================================================
bool compareStringAndFile(
    const std::string& content, const std::string& fileName)
{
  std::ifstream ifs(fileName, std::ifstream::in);
  EXPECT_TRUE(ifs.is_open());

  auto itr = content.begin();

  char c = ifs.get();
  while (ifs.good()) {
    if (*itr != c) {
      return false;
    }

    c = ifs.get();
    itr++;
  }

  ifs.close();

  return true;
}

//==============================================================================
TEST(Optimizer, OutStream)
{
  std::shared_ptr<Problem> prob = std::make_shared<Problem>(2);

  prob->setLowerBounds(Eigen::Vector2d(-HUGE_VAL, 0));
  prob->setInitialGuess(Eigen::Vector2d(1.234, 5.678));

  FunctionPtr obj = std::make_shared<SampleObjFunc>();
  prob->setObjective(obj);

  GradientDescentSolver solver(prob);
  solver.setIterationsPerPrint(50);

  // Print the progress to a std::string
  std::stringstream ss;
  solver.setOutStream(&ss);
  EXPECT_TRUE(solver.solve());
  std::string outputString = ss.str();

  // Print the progress to a file
  std::string outputFile = "test_optimizer_outstream.txt";
  std::ofstream ofs(outputFile);
  EXPECT_TRUE(ofs.is_open());
  solver.setOutStream(&ofs);
  EXPECT_TRUE(solver.solve());
  ofs.close();

  // Compare them
  EXPECT_TRUE(compareStringAndFile(outputString, outputFile));

  std::remove(outputFile.c_str());
}

//==============================================================================
TEST(Optimizer, ConstraintAccessorsReturnStoredConstraints)
{
  std::shared_ptr<Problem> prob = std::make_shared<Problem>(2);

  FunctionPtr eq = std::make_shared<SampleConstFunc>(1.0, 0.0);
  FunctionPtr ineq = std::make_shared<SampleConstFunc>(-1.0, 1.0);
  prob->addEqConstraint(eq);
  prob->addIneqConstraint(ineq);

  EXPECT_EQ(prob->getEqConstraint(0), eq);
  EXPECT_EQ(prob->getIneqConstraint(0), ineq);
}
