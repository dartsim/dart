/*
 * Copyright (c) 2011-2014, Georgia Tech Research Corporation
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
#include "dart/optimizer/nlopt/NloptSolver.h"
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
class SampleObjFunc : public Function
{
public:
  SampleObjFunc()
    : Function()
  {
  }

  virtual double operator()(Eigen::Map<const Eigen::VectorXd>& _x,
                            Eigen::Map<Eigen::VectorXd>& _grad)
  {
    if (_grad.size() > 0)
    {
      _grad[0] = 0.0;
      _grad[1] = 0.5 / std::sqrt(_x[1]);
    }
    return std::sqrt(_x[1]);
  }
};

//==============================================================================
class SampleConstFunc : public Function
{
public:
  SampleConstFunc(double _a, double _b)
    : Function(),
      a(_a),
      b(_b)
  {
  }

  virtual double operator()(Eigen::Map<const Eigen::VectorXd>& _x,
                            Eigen::Map<Eigen::VectorXd>& _grad)
  {
    if (_grad.size() > 0)
    {
      _grad[0] = 3 * a * (a*_x[0] + b) * (a*_x[0] + b);
      _grad[1] = -1.0;
    }
    return ((a*_x[0] + b) * (a*_x[0] + b) * (a*_x[0] + b) - _x[1]);
  }

private:
  double a;
  double b;
};

//==============================================================================
TEST(Optimizer, BasicNlopt)
{
  Problem prob(2);

  prob.setLowerBounds(Eigen::Vector2d(-HUGE_VAL, 0));
  prob.setInitialGuess(Eigen::Vector2d(1.234, 5.678));

  SampleObjFunc* c = new SampleObjFunc();
  prob.setObjective(c);

  SampleConstFunc const1 = SampleConstFunc( 2, 0);
  SampleConstFunc const2 = SampleConstFunc(-1, 1);
  prob.addIneqConstraint(&const1);
  prob.addIneqConstraint(&const2);

  NloptSolver solver(&prob, NLOPT_LD_MMA);
  solver.solve();

  double minF = prob.getOptimalValue();
  Eigen::VectorXd optX = prob.getOptimumParameters();

  EXPECT_NEAR(minF, 0.544330847, 1e-6);
  EXPECT_EQ(optX.size(), prob.getDimension());
  EXPECT_NEAR(optX[0], 0.333334, 1e-6);
  EXPECT_NEAR(optX[1], 0.296296, 1e-6);
}

//==============================================================================
#ifdef HAVE_IPOPT
TEST(Optimizer, BasicIpopt)
{
  dterr << "IPOPT is not implemented yet.\n";
  return;
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
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

