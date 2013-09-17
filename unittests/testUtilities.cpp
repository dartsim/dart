/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
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

#include "common/UtilsCode.h"
#include "math/Geometry.h"
#include "math/Helpers.h"
#include <iostream>
#include <gtest/gtest.h>
#include <Eigen/Dense>

/* ********************************************************************************************* */
TEST(UTILS, ROTATION) {
  using namespace dart;
  using namespace math;

  // Create Initial ExpMap
  Eigen::Vector3d axis(2.0, 1.0, 1.0);
  axis.normalize();
  double angle = 1.2;
  EXPECT_DOUBLE_EQ(axis.norm(), 1.0);
  Eigen::Vector3d expmap = axis * angle;


  // Test conversion between expmap and quaternion
  Eigen::Quaterniond q = expToQuat(expmap);
  Eigen::Vector3d expmap2 = quatToExp(q);

  EXPECT_NEAR((expmap - expmap2).norm(), 0.0, DART_EPSILON)
    << "Orig: " << expmap << " Reconstructed: " << expmap2;

  // Test conversion between matrix and euler
  Eigen::Matrix3d m = q.toRotationMatrix();
  Eigen::Vector3d e = matrixToEulerXYZ(m);
  Eigen::Matrix3d m2 = eulerXYZToMatrix(e);

  EXPECT_NEAR((m - m2).norm(), 0.0, DART_EPSILON)
    << "Orig: " << m << " Reconstructed: " << m2;
}

/* ********************************************************************************************* */
TEST(UTILS, UTILS) {
  // Test CR Matrix
  EXPECT_DOUBLE_EQ(dart::math::CR(0, 1), -1.0);

  // Test randomize function
  double x = dart::math::random(0.0, 2.0);
  EXPECT_LT(0.0, x);
  EXPECT_LT(x, 2.0);

  // Test transform
  Eigen::Isometry3d M = Eigen::Isometry3d::Identity();
  M.translation() = Eigen::Vector3d(3.0, 2.0, 1.0);
  Eigen::Vector3d pt(1.0, 0.5, 1.0);
  Eigen::Vector3d result = M * pt;
  Eigen::Vector3d expected(4.0, 2.5, 2.0);
  EXPECT_NEAR( (result - expected).norm(), 0.0, DART_EPSILON)
    << "result = " << result << " expected = " << expected;
}

/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */
