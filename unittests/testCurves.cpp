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
TEST(Curves, Basic)
{
  BSpline1f bs1f;
  BSpline2f bs2f;
  BSpline3f bs3f;

  BSpline1d bs1d;
  BSpline2d bs2d;
  BSpline3d bs3d;

  BSpline2d bs(2, 3, 0.0, 6.0, false);

  Eigen::MatrixXd ctrlPts(2, 3);
  ctrlPts << 0, 8, 8, 0, 8, 0;

  bs.setControlPoints(ctrlPts);
  EXPECT_NEAR(bs(3.0)[0], 7.0, 1e-12);
  EXPECT_NEAR(bs(3.0)[1], 6.0, 1e-12);

  // Set control point
  bs.setControlPoint(0, 0, 10.0);
  EXPECT_EQ(bs.getControlPoint(0, 0), 10.0);

  // Set knot
  bs.setKnot(0, 10.0);
  EXPECT_EQ(bs.getKnot(0), 10.0);
}

//==============================================================================
TEST(Curves, UniformKnots)
{
  Eigen::VectorXd knots;
  Eigen::VectorXd expectedKnots;

  BSpline1d bs1(1, 5, 0.0, 4.0, true);
  knots = bs1.knots();
  expectedKnots.resize(7);
  expectedKnots << 0, 0, 1, 2, 3, 4, 4;
  EXPECT_EQ(knots, expectedKnots);

  BSpline1d bs2(2, 5, 0.0, 3.0, true);
  knots = bs2.knots();
  expectedKnots.resize(8);
  expectedKnots << 0, 0, 0, 1, 2, 3, 3, 3;
  EXPECT_EQ(knots, expectedKnots);

  BSpline1d bs3(3, 5, 0.0, 2.0, true);
  knots = bs3.knots();
  expectedKnots.resize(9);
  expectedKnots << 0, 0, 0, 0, 1, 2, 2, 2, 2;
  EXPECT_EQ(knots, expectedKnots);

  BSpline1d bs4(1, 5, 0.0, 1.0, true);
  knots = bs4.knots();
  expectedKnots.resize(7);
  expectedKnots << 0, 0, 1.0/4.0, 2.0/4.0, 3.0/4.0, 1, 1;
  EXPECT_EQ(knots, expectedKnots);

  BSpline1d bs5(2, 5, 0.0, 1.0, true);
  knots = bs5.knots();
  expectedKnots.resize(8);
  expectedKnots << 0, 0, 0, 1.0/3.0, 2.0/3.0, 1, 1, 1;
  EXPECT_EQ(knots, expectedKnots);

  BSpline1d bs6(3, 5, 0.0, 1.0, true);
  knots = bs6.knots();
  expectedKnots.resize(9);
  expectedKnots << 0, 0, 0, 0, 1.0/2.0, 1, 1, 1, 1;
  EXPECT_EQ(knots, expectedKnots);
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

