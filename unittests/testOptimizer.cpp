/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

// For problem
#include <iostream>
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "dart/config.h"
#include "dart/common/Console.h"
#include "dart/optimizer/Function.h"
#include "dart/optimizer/Problem.h"
#ifdef HAVE_NLOPT
  #include "dart/optimizer/nlopt/NloptSolver.h"
#endif
#ifdef HAVE_IPOPT
  #include "dart/optimizer/ipopt/IpoptSolver.h"
#endif
#ifdef HAVE_SNOPT
  #include "dart/optimizer/snopt/SnoptSolver.h"
#endif

using namespace std;
using namespace Eigen;
using namespace dart::optimizer;

//==============================================================================
/// \brief class SampleObjFunc
class SampleObjFunc : public Function
{
public:
  /// \brief Constructor
  SampleObjFunc() : Function() {}

  /// \brief Destructor
  virtual ~SampleObjFunc() {}

  /// \copydoc Function::eval
  virtual double eval(Eigen::Map<const Eigen::VectorXd>& _x)
  {
    return std::sqrt(_x[1]);
  }

  /// \copydoc Function::evalGradient
  virtual void evalGradient(Eigen::Map<const Eigen::VectorXd>& _x,
                            Eigen::Map<Eigen::VectorXd> _grad)
  {
    _grad[0] = 0.0;
    _grad[1] = 0.5 / std::sqrt(_x[1]);
  }
};

//==============================================================================
class SampleConstFunc : public Function
{
public:
  /// \brief Constructor
  SampleConstFunc(double _a, double _b) : Function(), mA(_a), mB(_b) {}

  /// \brief Destructor
  virtual ~SampleConstFunc() {}

  /// \copydoc Function::eval
  virtual double eval(Eigen::Map<const Eigen::VectorXd>& _x)
  {
    return ((mA*_x[0] + mB) * (mA*_x[0] + mB) * (mA*_x[0] + mB) - _x[1]);
  }

  /// \copydoc Function::evalGradient
  virtual void evalGradient(Eigen::Map<const Eigen::VectorXd>& _x,
                            Eigen::Map<Eigen::VectorXd> _grad)
  {
    _grad[0] = 3 * mA * (mA*_x[0] + mB) * (mA*_x[0] + mB);
    _grad[1] = -1.0;
  }

private:
  /// \brief Data
  double mA;

  /// \brief Data
  double mB;
};

//==============================================================================
#ifdef HAVE_NLOPT
TEST(Optimizer, BasicNlopt)
{
  // Problem reference: http://ab-initio.mit.edu/wiki/index.php/NLopt_Tutorial

  Problem prob(2);

  prob.setLowerBounds(Eigen::Vector2d(-HUGE_VAL, 0));
  prob.setInitialGuess(Eigen::Vector2d(1.234, 5.678));

  SampleObjFunc obj;
  prob.setObjective(&obj);

  SampleConstFunc const1( 2, 0);
  SampleConstFunc const2(-1, 1);
  prob.addIneqConstraint(&const1);
  prob.addIneqConstraint(&const2);

  NloptSolver solver(&prob, NLOPT_LD_MMA);
  solver.solve();

  double minF = prob.getOptimumValue();
  Eigen::VectorXd optX = prob.getOptimalSolution();

  EXPECT_NEAR(minF, 0.544330847, 1e-6);
  EXPECT_EQ(static_cast<size_t>(optX.size()), prob.getDimension());
  EXPECT_NEAR(optX[0], 0.333334, 1e-6);
  EXPECT_NEAR(optX[1], 0.296296, 1e-6);
}
#endif

//==============================================================================
#ifdef HAVE_IPOPT
TEST(Optimizer, BasicIpopt)
{
  dterr << "Ipopt does not pass this test yet. Please see #153.";
  return;

  Problem prob(2);

  prob.setLowerBounds(Eigen::Vector2d(-HUGE_VAL, 0));
  prob.setInitialGuess(Eigen::Vector2d(1.234, 5.678));

  SampleObjFunc obj;
  prob.setObjective(&obj);

  SampleConstFunc const1( 2, 0);
  SampleConstFunc const2(-1, 1);
  prob.addIneqConstraint(&const1);
  prob.addIneqConstraint(&const2);

  IpoptSolver solver(&prob);
  solver.solve();

  double minF = prob.getOptimumValue();
  Eigen::VectorXd optX = prob.getOptimalSolution();

  EXPECT_NEAR(minF, 0.544330847, 1e-6);
  EXPECT_EQ(static_cast<size_t>(optX.size()), prob.getDimension());
  EXPECT_NEAR(optX[0], 0.333334, 1e-6);
  EXPECT_NEAR(optX[1], 0.296296, 1e-6);
}
#endif

//==============================================================================
#ifdef HAVE_SNOPT
TEST(Optimizer, BasicSnopt)
{
  dterr << "SNOPT is not implemented yet.\n";
  return;
}
#endif

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

