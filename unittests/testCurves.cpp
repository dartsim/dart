/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
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

#include <iostream>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "TestHelpers.h"

#include "dart/common/common.h"
#include "dart/math/math.h"

using namespace Eigen;
using namespace dart;
using namespace common;
using namespace math;

//==============================================================================
TEST(Curves, UniformKnots)
{
  Eigen::VectorXd knots;
  Eigen::VectorXd expectedKnots;

  BSpline bs1(1, 5, 0.0, 4.0, true);
  knots = bs1.getKnots();
  expectedKnots.resize(7);
  expectedKnots << 0, 0, 1, 2, 3, 4, 4;
  EXPECT_EQ(knots, expectedKnots);

  BSpline bs2(2, 5, 0.0, 3.0, true);
  knots = bs2.getKnots();
  expectedKnots.resize(8);
  expectedKnots << 0, 0, 0, 1, 2, 3, 3, 3;
  EXPECT_EQ(knots, expectedKnots);

  BSpline bs3(3, 5, 0.0, 2.0, true);
  knots = bs3.getKnots();
  expectedKnots.resize(9);
  expectedKnots << 0, 0, 0, 0, 1, 2, 2, 2, 2;
  EXPECT_EQ(knots, expectedKnots);

  BSpline bs4(1, 5, 0.0, 1.0, true);
  knots = bs4.getKnots();
  expectedKnots.resize(7);
  expectedKnots << 0, 0, 1.0/4.0, 2.0/4.0, 3.0/4.0, 1, 1;
  EXPECT_EQ(knots, expectedKnots);

  BSpline bs5(2, 5, 0.0, 1.0, true);
  knots = bs5.getKnots();
  expectedKnots.resize(8);
  expectedKnots << 0, 0, 0, 1.0/3.0, 2.0/3.0, 1, 1, 1;
  EXPECT_EQ(knots, expectedKnots);

  BSpline bs6(3, 5, 0.0, 1.0, true);
  knots = bs6.getKnots();
  expectedKnots.resize(9);
  expectedKnots << 0, 0, 0, 0, 1.0/2.0, 1, 1, 1, 1;
  EXPECT_EQ(knots, expectedKnots);
}

//==============================================================================
TEST(Curves, BSplineBasic)
{
  BSpline bs(2, 3, 0.0, 6.0, false);

  Eigen::VectorXd ctrlPts1(3);
  Eigen::VectorXd ctrlPts2(3);
  ctrlPts1 << 0, 8, 8;
  ctrlPts2 << 0, 8, 0;

  bs.setControlPoints(ctrlPts1);
  EXPECT_NEAR(bs(3.0), 7.0, 1e-12);

  bs.setControlPoints(ctrlPts2);
  EXPECT_NEAR(bs(3.0), 6.0, 1e-12);
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

