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

#include "dart/math/geometry.hpp"
#include "dart/math/helpers.hpp"

#include <gtest/gtest.h>

#include <iostream>
#include <span>
#include <vector>

#include <cmath>

using namespace dart;
using namespace dart::math;
using namespace dart::test;
using namespace Eigen;

#define LIE_GROUP_OPT_TOL 1e-12

//==============================================================================
Eigen::Matrix4d toMatrixForm(const Eigen::Vector6d& v)
{
  Eigen::Matrix4d result = Eigen::Matrix4d::Zero();

  result(0, 1) = -v(2);
  result(1, 0) = v(2);
  result(0, 2) = v(1);
  result(2, 0) = -v(1);
  result(1, 2) = -v(0);
  result(2, 1) = v(0);

  result(0, 3) = v(3);
  result(1, 3) = v(4);
  result(2, 3) = v(5);

  return result;
}

//==============================================================================
Eigen::Vector6d fromMatrixForm(const Eigen::Matrix4d& m)
{
  Eigen::Vector6d ret;
  ret << m(2, 1), m(0, 2), m(1, 0), m(0, 3), m(1, 3), m(2, 3);
  return ret;
}

//==============================================================================
void testEulerAngles(const Eigen::Vector3d& angle)
{
  Eigen::Matrix3d mat1;
  Eigen::Matrix3d mat2;

  // XYX
  mat1 = math::eulerXYXToMatrix(angle);
  mat2 = Eigen::AngleAxisd(angle(0), Eigen::Vector3d::UnitX())
         * Eigen::AngleAxisd(angle(1), Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(angle(2), Eigen::Vector3d::UnitX());

  EXPECT_TRUE(math::verifyRotation(mat1));
  EXPECT_TRUE(math::verifyRotation(mat2));
  EXPECT_TRUE(equals(mat1, mat2));
  EXPECT_TRUE(equals(mat1, eulerXYXToMatrix(matrixToEulerXYX(mat1))));

  // XYZ
  mat1 = math::eulerXYZToMatrix(angle);
  mat2 = Eigen::AngleAxisd(angle(0), Eigen::Vector3d::UnitX())
         * Eigen::AngleAxisd(angle(1), Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(angle(2), Eigen::Vector3d::UnitZ());

  EXPECT_TRUE(math::verifyRotation(mat1));
  EXPECT_TRUE(math::verifyRotation(mat2));
  EXPECT_TRUE(equals(mat1, mat2));
  EXPECT_TRUE(equals(mat1, eulerXYZToMatrix(matrixToEulerXYZ(mat1))));

  // XZX
  mat1 = math::eulerXZXToMatrix(angle);
  mat2 = Eigen::AngleAxisd(angle(0), Eigen::Vector3d::UnitX())
         * Eigen::AngleAxisd(angle(1), Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(angle(2), Eigen::Vector3d::UnitX());

  EXPECT_TRUE(math::verifyRotation(mat1));
  EXPECT_TRUE(math::verifyRotation(mat2));
  EXPECT_TRUE(equals(mat1, mat2));
  // EXPECT_TRUE(equals(mat1, eulerXZXToMatrix(matrixToEulerXZX(mat1))));

  // XZY
  mat1 = math::eulerXZYToMatrix(angle);
  mat2 = Eigen::AngleAxisd(angle(0), Eigen::Vector3d::UnitX())
         * Eigen::AngleAxisd(angle(1), Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(angle(2), Eigen::Vector3d::UnitY());

  EXPECT_TRUE(math::verifyRotation(mat1));
  EXPECT_TRUE(math::verifyRotation(mat2));
  EXPECT_TRUE(equals(mat1, mat2));
  EXPECT_TRUE(equals(mat1, eulerXZYToMatrix(matrixToEulerXZY(mat1))));

  // YXY
  mat1 = math::eulerYXYToMatrix(angle);
  mat2 = Eigen::AngleAxisd(angle(0), Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(angle(1), Eigen::Vector3d::UnitX())
         * Eigen::AngleAxisd(angle(2), Eigen::Vector3d::UnitY());

  EXPECT_TRUE(math::verifyRotation(mat1));
  EXPECT_TRUE(math::verifyRotation(mat2));
  EXPECT_TRUE(equals(mat1, mat2));
  // EXPECT_TRUE(equals(mat1, eulerYXYToMatrix(matrixToEulerYXY(mat1))));

  // YXZ
  mat1 = math::eulerYXZToMatrix(angle);
  mat2 = Eigen::AngleAxisd(angle(0), Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(angle(1), Eigen::Vector3d::UnitX())
         * Eigen::AngleAxisd(angle(2), Eigen::Vector3d::UnitZ());

  EXPECT_TRUE(math::verifyRotation(mat1));
  EXPECT_TRUE(math::verifyRotation(mat2));
  EXPECT_TRUE(equals(mat1, mat2));
  EXPECT_TRUE(equals(mat1, eulerYXZToMatrix(matrixToEulerYXZ(mat1))));

  // YZX
  mat1 = math::eulerYZXToMatrix(angle);
  mat2 = Eigen::AngleAxisd(angle(0), Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(angle(1), Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(angle(2), Eigen::Vector3d::UnitX());

  EXPECT_TRUE(math::verifyRotation(mat1));
  EXPECT_TRUE(math::verifyRotation(mat2));
  EXPECT_TRUE(equals(mat1, mat2));
  EXPECT_TRUE(equals(mat1, eulerYZXToMatrix(matrixToEulerYZX(mat1))));

  // YZY
  mat1 = math::eulerYZYToMatrix(angle);
  mat2 = Eigen::AngleAxisd(angle(0), Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(angle(1), Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(angle(2), Eigen::Vector3d::UnitY());

  EXPECT_TRUE(math::verifyRotation(mat1));
  EXPECT_TRUE(math::verifyRotation(mat2));
  EXPECT_TRUE(equals(mat1, mat2));
  // EXPECT_TRUE(equals(mat1, eulerYZYToMatrix(matrixToEulerYZY(mat1))));

  // ZXY
  mat1 = math::eulerZXYToMatrix(angle);
  mat2 = Eigen::AngleAxisd(angle(0), Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(angle(1), Eigen::Vector3d::UnitX())
         * Eigen::AngleAxisd(angle(2), Eigen::Vector3d::UnitY());

  EXPECT_TRUE(math::verifyRotation(mat1));
  EXPECT_TRUE(math::verifyRotation(mat2));
  EXPECT_TRUE(equals(mat1, mat2));
  EXPECT_TRUE(equals(mat1, eulerZXYToMatrix(matrixToEulerZXY(mat1))));

  // ZYX
  mat1 = math::eulerZYXToMatrix(angle);
  mat2 = Eigen::AngleAxisd(angle(0), Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(angle(1), Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(angle(2), Eigen::Vector3d::UnitX());

  EXPECT_TRUE(math::verifyRotation(mat1));
  EXPECT_TRUE(math::verifyRotation(mat2));
  EXPECT_TRUE(equals(mat1, mat2));
  EXPECT_TRUE(equals(mat1, eulerZYXToMatrix(matrixToEulerZYX(mat1))));

  // ZXZ
  mat1 = math::eulerZXZToMatrix(angle);
  mat2 = Eigen::AngleAxisd(angle(0), Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(angle(1), Eigen::Vector3d::UnitX())
         * Eigen::AngleAxisd(angle(2), Eigen::Vector3d::UnitZ());

  EXPECT_TRUE(math::verifyRotation(mat1));
  EXPECT_TRUE(math::verifyRotation(mat2));
  EXPECT_TRUE(equals(mat1, mat2));
  // EXPECT_TRUE(equals(mat1, eulerZXZToMatrix(matrixToEulerZXZ(mat1))));

  // ZYZ
  mat1 = math::eulerZYZToMatrix(angle);
  mat2 = Eigen::AngleAxisd(angle(0), Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(angle(1), Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(angle(2), Eigen::Vector3d::UnitZ());

  EXPECT_TRUE(math::verifyRotation(mat1));
  EXPECT_TRUE(math::verifyRotation(mat2));
  EXPECT_TRUE(equals(mat1, mat2));
  // EXPECT_TRUE(equals(mat1, eulerZYZToMatrix(matrixToEulerZYZ(mat1))));
}

//==============================================================================
TEST(Geometry, ComputeConvexHullUsesSortedAngles)
{
  SupportPolygon points;
  points.emplace_back(0.0, 0.0);
  points.emplace_back(1.0, 0.0);
  points.emplace_back(1.0, 1.0);
  points.emplace_back(0.0, 1.0);

  std::vector<std::size_t> indices;
  const auto hull
      = computeConvexHull(indices, std::span<const Eigen::Vector2d>(points));

  EXPECT_EQ(hull.size(), 4u);
  EXPECT_EQ(indices.size(), 4u);
}

//==============================================================================
TEST(Geometry, SkewSymmetricRoundTrip)
{
  const Eigen::Vector3d v(1.0, -2.0, 3.0);
  const Eigen::Matrix3d skew = makeSkewSymmetric(v);

  EXPECT_TRUE(skew.isApprox(-skew.transpose()));
  EXPECT_VECTOR_NEAR(fromSkewSymmetric(skew), v, 1e-12);
}

//==============================================================================
TEST(Geometry, QuaternionDerivatives)
{
  const Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

  const Eigen::Matrix3d dW = quatDeriv(q, 0);
  EXPECT_TRUE(dW.isApprox(2.0 * Eigen::Matrix3d::Identity()));

  const Eigen::Matrix3d dX = quatDeriv(q, 1);
  EXPECT_NEAR(dX(1, 2), -2.0, 1e-12);
  EXPECT_NEAR(dX(2, 1), 2.0, 1e-12);

  const Eigen::Matrix3d d2W = quatSecondDeriv(q, 0, 0);
  EXPECT_TRUE(d2W.isApprox(2.0 * Eigen::Matrix3d::Identity()));

  const Eigen::Matrix3d d2WX = quatSecondDeriv(q, 0, 1);
  EXPECT_NEAR(d2WX(1, 2), -2.0, 1e-12);
  EXPECT_NEAR(d2WX(2, 1), 2.0, 1e-12);
}

//==============================================================================
TEST(Geometry, QuaternionDerivativeBranches)
{
  const Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

  const Eigen::Matrix3d dY = quatDeriv(q, 2);
  EXPECT_NEAR(dY(0, 2), 2.0, 1e-12);
  EXPECT_NEAR(dY(2, 0), -2.0, 1e-12);

  const Eigen::Matrix3d dZ = quatDeriv(q, 3);
  EXPECT_NEAR(dZ(0, 1), -2.0, 1e-12);
  EXPECT_NEAR(dZ(1, 0), 2.0, 1e-12);

  const Eigen::Matrix3d d2YY = quatSecondDeriv(q, 2, 2);
  EXPECT_NEAR(d2YY(0, 0), -2.0, 1e-12);
  EXPECT_NEAR(d2YY(1, 1), 2.0, 1e-12);

  const Eigen::Matrix3d d2YZ = quatSecondDeriv(q, 2, 3);
  EXPECT_NEAR(d2YZ(1, 2), 2.0, 1e-12);
  EXPECT_NEAR(d2YZ(2, 1), 2.0, 1e-12);

  const Eigen::Matrix3d d2ZY = quatSecondDeriv(q, 3, 1);
  EXPECT_NEAR(d2ZY(0, 2), 2.0, 1e-12);
  EXPECT_NEAR(d2ZY(2, 0), 2.0, 1e-12);
}

//==============================================================================
TEST(Geometry, RotatePointAndExpMapJacobians)
{
  const Eigen::Quaterniond q(
      Eigen::AngleAxisd(0.5 * math::pi, Eigen::Vector3d::UnitZ()));

  const Eigen::Vector3d rotated
      = rotatePoint(q, Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_VECTOR_NEAR(rotated, Eigen::Vector3d(0.0, 1.0, 0.0), 1e-12);

  const Eigen::Vector3d rotated2 = rotatePoint(q, 0.0, 1.0, 0.0);
  EXPECT_VECTOR_NEAR(rotated2, Eigen::Vector3d(-1.0, 0.0, 0.0), 1e-12);

  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  const Eigen::Matrix3d jac = expMapJac(zero);
  EXPECT_TRUE(jac.isApprox(Eigen::Matrix3d::Identity()));

  const Eigen::Vector3d qdot(1.0, 2.0, -3.0);
  const Eigen::Matrix3d jacDot = expMapJacDot(zero, qdot);
  const Eigen::Matrix3d expected = 0.5 * makeSkewSymmetric(qdot);
  EXPECT_TRUE(jacDot.isApprox(expected));

  const Eigen::Matrix3d deriv = expMapJacDeriv(zero, 1);
  const Eigen::Matrix3d expectedDeriv
      = expMapJacDot(zero, Eigen::Vector3d::UnitY());
  EXPECT_TRUE(deriv.isApprox(expectedDeriv));
}

//==============================================================================
TEST(Geometry, ExpMapJacDotNonZero)
{
  const Eigen::Vector3d q(0.2, -0.1, 0.3);
  const Eigen::Vector3d qdot(0.1, 0.05, -0.2);

  const Eigen::Matrix3d jacDot = expMapJacDot(q, qdot);
  const double dt = 1e-6;
  const Eigen::Matrix3d numeric
      = (expMapJac(q + dt * qdot) - expMapJac(q)) / dt;
  EXPECT_TRUE(jacDot.isApprox(numeric, 1e-5));
}

//==============================================================================
TEST(Geometry, ExpAndLogMaps)
{
  const Eigen::Vector3d exp(0.1, -0.2, 0.3);
  const Eigen::Quaterniond q = expToQuat(exp);
  const Eigen::Vector3d expBack = quatToExp(q);
  EXPECT_VECTOR_NEAR(expBack, exp, 1e-9);

  const Eigen::Matrix3d rot = expMapRot(exp);
  const Eigen::Vector3d logRot = logMap(rot);
  EXPECT_VECTOR_NEAR(logRot, exp, 1e-9);

  const Eigen::Isometry3d angular = expAngular(exp);
  EXPECT_TRUE(angular.linear().isApprox(rot, 1e-12));

  Eigen::Vector6d twist = Eigen::Vector6d::Zero();
  twist.tail<3>() = Eigen::Vector3d(1.0, -2.0, 3.0);
  const Eigen::Isometry3d T = expMap(twist);
  EXPECT_VECTOR_NEAR(T.translation(), twist.tail<3>(), 1e-12);

  const Eigen::Vector6d logT = logMap(T);
  EXPECT_VECTOR_NEAR(logT.head<3>(), Eigen::Vector3d::Zero(), 1e-12);
  EXPECT_VECTOR_NEAR(logT.tail<3>(), twist.tail<3>(), 1e-12);
}

//==============================================================================
TEST(Geometry, EulerConversions)
{
  const Eigen::Vector3d angles(0.2, -0.3, 0.4);
  const Eigen::Matrix3d Rxyz = eulerXYZToMatrix(angles);
  const Eigen::Vector3d recovered = matrixToEulerXYZ(Rxyz);
  EXPECT_TRUE(eulerXYZToMatrix(recovered).isApprox(Rxyz, 1e-12));

  const Eigen::Vector3d anglesZyx(-0.1, 0.25, -0.2);
  const Eigen::Matrix3d Rzyx = eulerZYXToMatrix(anglesZyx);
  const Eigen::Vector3d recoveredZyx = matrixToEulerZYX(Rzyx);
  EXPECT_TRUE(eulerZYXToMatrix(recoveredZyx).isApprox(Rzyx, 1e-12));
}

//==============================================================================
TEST(Geometry, EulerSingularities)
{
  constexpr double tol = 1e-12;
  const Eigen::Matrix3d xyzPos
      = Eigen::AngleAxisd(math::half_pi, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
  const Eigen::Vector3d xyzPosAngles = matrixToEulerXYZ(xyzPos);
  EXPECT_NEAR(xyzPosAngles[0], 0.0, tol);
  EXPECT_NEAR(std::abs(xyzPosAngles[1]), math::half_pi, tol);
  EXPECT_TRUE(std::isfinite(xyzPosAngles[2]));

  const Eigen::Matrix3d xyzNeg
      = Eigen::AngleAxisd(-math::half_pi, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
  const Eigen::Vector3d xyzNegAngles = matrixToEulerXYZ(xyzNeg);
  EXPECT_NEAR(xyzNegAngles[0], 0.0, tol);
  EXPECT_NEAR(std::abs(xyzNegAngles[1]), math::half_pi, tol);
  EXPECT_TRUE(std::isfinite(xyzNegAngles[2]));

  const Eigen::Matrix3d zyxPos
      = Eigen::AngleAxisd(-math::half_pi, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
  const Eigen::Vector3d zyxPosAngles = matrixToEulerZYX(zyxPos);
  EXPECT_NEAR(zyxPosAngles[0], 0.0, tol);
  EXPECT_NEAR(std::abs(zyxPosAngles[1]), math::half_pi, tol);
  EXPECT_TRUE(std::isfinite(zyxPosAngles[2]));

  const Eigen::Matrix3d zyxNeg
      = Eigen::AngleAxisd(math::half_pi, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
  const Eigen::Vector3d zyxNegAngles = matrixToEulerZYX(zyxNeg);
  EXPECT_NEAR(zyxNegAngles[0], 0.0, tol);
  EXPECT_NEAR(std::abs(zyxNegAngles[1]), math::half_pi, tol);
  EXPECT_TRUE(std::isfinite(zyxNegAngles[2]));

  const Eigen::Matrix3d xzyPos
      = Eigen::AngleAxisd(-math::half_pi, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
  const Eigen::Vector3d xzyPosAngles = matrixToEulerXZY(xzyPos);
  EXPECT_NEAR(xzyPosAngles[0], 0.0, tol);
  EXPECT_NEAR(std::abs(xzyPosAngles[1]), math::half_pi, tol);
  EXPECT_TRUE(std::isfinite(xzyPosAngles[2]));

  const Eigen::Matrix3d xzyNeg
      = Eigen::AngleAxisd(math::half_pi, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
  const Eigen::Vector3d xzyNegAngles = matrixToEulerXZY(xzyNeg);
  EXPECT_NEAR(xzyNegAngles[0], 0.0, tol);
  EXPECT_NEAR(std::abs(xzyNegAngles[1]), math::half_pi, tol);
  EXPECT_TRUE(std::isfinite(xzyNegAngles[2]));

  const Eigen::Matrix3d yzxPos
      = Eigen::AngleAxisd(math::half_pi, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
  const Eigen::Vector3d yzxPosAngles = matrixToEulerYZX(yzxPos);
  EXPECT_NEAR(yzxPosAngles[0], 0.0, tol);
  EXPECT_NEAR(std::abs(yzxPosAngles[1]), math::half_pi, tol);
  EXPECT_TRUE(std::isfinite(yzxPosAngles[2]));

  const Eigen::Matrix3d yzxNeg
      = Eigen::AngleAxisd(-math::half_pi, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
  const Eigen::Vector3d yzxNegAngles = matrixToEulerYZX(yzxNeg);
  EXPECT_NEAR(yzxNegAngles[0], 0.0, tol);
  EXPECT_NEAR(std::abs(yzxNegAngles[1]), math::half_pi, tol);
  EXPECT_TRUE(std::isfinite(yzxNegAngles[2]));

  const Eigen::Matrix3d zxyPos
      = Eigen::AngleAxisd(math::half_pi, Eigen::Vector3d::UnitX())
            .toRotationMatrix();
  const Eigen::Vector3d zxyPosAngles = matrixToEulerZXY(zxyPos);
  EXPECT_NEAR(zxyPosAngles[0], 0.0, tol);
  EXPECT_NEAR(std::abs(zxyPosAngles[1]), math::half_pi, tol);
  EXPECT_TRUE(std::isfinite(zxyPosAngles[2]));

  const Eigen::Matrix3d zxyNeg
      = Eigen::AngleAxisd(-math::half_pi, Eigen::Vector3d::UnitX())
            .toRotationMatrix();
  const Eigen::Vector3d zxyNegAngles = matrixToEulerZXY(zxyNeg);
  EXPECT_NEAR(zxyNegAngles[0], 0.0, tol);
  EXPECT_NEAR(std::abs(zxyNegAngles[1]), math::half_pi, tol);
  EXPECT_TRUE(std::isfinite(zxyNegAngles[2]));

  const Eigen::Matrix3d yxzPos
      = Eigen::AngleAxisd(-math::half_pi, Eigen::Vector3d::UnitX())
            .toRotationMatrix();
  const Eigen::Vector3d yxzPosAngles = matrixToEulerYXZ(yxzPos);
  EXPECT_NEAR(yxzPosAngles[0], 0.0, tol);
  EXPECT_NEAR(std::abs(yxzPosAngles[1]), math::half_pi, tol);
  EXPECT_TRUE(std::isfinite(yxzPosAngles[2]));

  const Eigen::Matrix3d yxzNeg
      = Eigen::AngleAxisd(math::half_pi, Eigen::Vector3d::UnitX())
            .toRotationMatrix();
  const Eigen::Vector3d yxzNegAngles = matrixToEulerYXZ(yxzNeg);
  EXPECT_NEAR(yxzNegAngles[0], 0.0, tol);
  EXPECT_NEAR(std::abs(yxzNegAngles[1]), math::half_pi, tol);
  EXPECT_TRUE(std::isfinite(yxzNegAngles[2]));

  const Eigen::Matrix3d xyxPos
      = Eigen::AngleAxisd(math::pi, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
  const Eigen::Vector3d xyxPosAngles = matrixToEulerXYX(xyxPos);
  EXPECT_NEAR(xyxPosAngles[1], math::pi, tol);
  EXPECT_NEAR(xyxPosAngles[2], 0.0, tol);
  EXPECT_TRUE(std::isfinite(xyxPosAngles[0]));

  const Eigen::Matrix3d xyxNeg = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d xyxNegAngles = matrixToEulerXYX(xyxNeg);
  EXPECT_NEAR(xyxNegAngles[1], 0.0, tol);
  EXPECT_NEAR(xyxNegAngles[2], 0.0, tol);
  EXPECT_TRUE(std::isfinite(xyxNegAngles[0]));
}

//==============================================================================
TEST(LieGroupOperators, EulerAngles)
{
  // TODO: Special angles such as (PI, 0, 0)

  //
  int numTest = 1;
  for (int i = 0; i < numTest; ++i) {
    Eigen::Vector3d angle = Eigen::Vector3d::Random();
    testEulerAngles(angle);
  }
}

//==============================================================================
#define EPSILON_EXPMAP_THETA 1.0e-3
TEST(LieGroupOperators, ExponentialMappings)
{
  int numTest = 100;

  // Exp
  for (int i = 0; i < numTest; ++i) {
    Eigen::Vector6d s = Eigen::Vector6d::Random();
    Eigen::Isometry3d Exp_s = math::expMap(s);
    Eigen::Matrix4d Exp_s_2 = Eigen::Matrix4d::Identity();

    double theta = s.head<3>().norm();
    Eigen::Matrix3d R = Matrix3d::Zero();
    Eigen::Matrix3d qss = math::makeSkewSymmetric(s.head<3>());
    Eigen::Matrix3d qss2 = qss * qss;
    Eigen::Matrix3d P = Eigen::Matrix3d::Zero();

    if (theta < EPSILON_EXPMAP_THETA) {
      R = Matrix3d::Identity() + qss + 0.5 * qss2;
      P = Matrix3d::Identity() + 0.5 * qss + (1 / 6) * qss2;
    } else {
      R = Matrix3d::Identity() + (sin(theta) / theta) * qss
          + ((1 - cos(theta)) / (theta * theta)) * qss2;
      P = Matrix3d::Identity() + ((1 - cos(theta)) / (theta * theta)) * qss
          + ((theta - sin(theta)) / (theta * theta * theta)) * qss2;
    }

    Exp_s_2.topLeftCorner<3, 3>() = R;
    Exp_s_2.topRightCorner<3, 1>() = P * s.tail<3>();

    //
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        EXPECT_NEAR(Exp_s(i, j), Exp_s_2(i, j), LIE_GROUP_OPT_TOL);
      }
    }
  }

  // ExpAngular
  for (int i = 0; i < numTest; ++i) {
    Eigen::Vector6d s = Eigen::Vector6d::Random();
    s.tail<3>() = Eigen::Vector3d::Zero();
    Eigen::Isometry3d Exp_s = math::expAngular(s.head<3>());
    Eigen::Matrix4d Exp_s_2 = Eigen::Matrix4d::Identity();

    double theta = s.head<3>().norm();
    Eigen::Matrix3d R = Matrix3d::Zero();
    Eigen::Matrix3d qss = math::makeSkewSymmetric(s.head<3>());
    Eigen::Matrix3d qss2 = qss * qss;
    Eigen::Matrix3d P = Eigen::Matrix3d::Zero();

    if (theta < EPSILON_EXPMAP_THETA) {
      R = Matrix3d::Identity() + qss + 0.5 * qss2;
      P = Matrix3d::Identity() + 0.5 * qss + (1 / 6) * qss2;
    } else {
      R = Matrix3d::Identity() + (sin(theta) / theta) * qss
          + ((1 - cos(theta)) / (theta * theta)) * qss2;
      P = Matrix3d::Identity() + ((1 - cos(theta)) / (theta * theta)) * qss
          + ((theta - sin(theta)) / (theta * theta * theta)) * qss2;
    }

    Exp_s_2.topLeftCorner<3, 3>() = R;
    Exp_s_2.topRightCorner<3, 1>() = P * s.tail<3>();

    //
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        EXPECT_NEAR(Exp_s(i, j), Exp_s_2(i, j), LIE_GROUP_OPT_TOL);
      }
    }
  }

  // ExpLinear
  for (int i = 0; i < numTest; ++i) {
    Eigen::Vector6d s = Eigen::Vector6d::Random();
    s.head<3>() = Eigen::Vector3d::Zero();
    Eigen::Isometry3d Exp_s(Eigen::Translation3d(s.tail<3>()));
    Eigen::Matrix4d Exp_s_2 = Eigen::Matrix4d::Identity();

    double theta = s.head<3>().norm();
    Eigen::Matrix3d R = Matrix3d::Zero();
    Eigen::Matrix3d qss = math::makeSkewSymmetric(s.head<3>());
    Eigen::Matrix3d qss2 = qss * qss;
    Eigen::Matrix3d P = Eigen::Matrix3d::Zero();

    if (theta < EPSILON_EXPMAP_THETA) {
      R = Matrix3d::Identity() + qss + 0.5 * qss2;
      P = Matrix3d::Identity() + 0.5 * qss + (1 / 6) * qss2;
    } else {
      R = Matrix3d::Identity() + (sin(theta) / theta) * qss
          + ((1 - cos(theta)) / (theta * theta)) * qss2;
      P = Matrix3d::Identity() + ((1 - cos(theta)) / (theta * theta)) * qss
          + ((theta - sin(theta)) / (theta * theta * theta)) * qss2;
    }

    Exp_s_2.topLeftCorner<3, 3>() = R;
    Exp_s_2.topRightCorner<3, 1>() = P * s.tail<3>();

    //
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        EXPECT_NEAR(Exp_s(i, j), Exp_s_2(i, j), LIE_GROUP_OPT_TOL);
      }
    }
  }
  // Exponential mapping test with high values
  int numExpTests = 100;
  double min = -1e+128;
  double max = +1e+128;

  for (int idxTest = 0; idxTest < numExpTests; ++idxTest) {
    Eigen::Vector3d randomS = Eigen::Vector3d::Zero();

    for (int i = 0; i < 3; ++i) {
      randomS[i] = Random::uniform(min, max);
    }

    Eigen::Isometry3d T = math::expAngular(randomS);
    EXPECT_TRUE(math::verifyTransform(T));
  }

  for (int idxTest = 0; idxTest < numExpTests; ++idxTest) {
    Eigen::Vector6d randomS = Eigen::Vector6d::Zero();

    for (int i = 0; i < 6; ++i) {
      randomS[i] = Random::uniform(min, max);
    }

    Eigen::Isometry3d T = math::expMap(randomS);
    EXPECT_TRUE(math::verifyTransform(T));
  }
}

//==============================================================================
TEST(Geometry, SupportPolygonUtilities)
{
  SupportPolygon square;
  square.emplace_back(0.0, 0.0);
  square.emplace_back(1.0, 0.0);
  square.emplace_back(1.0, 1.0);
  square.emplace_back(0.0, 1.0);

  EXPECT_TRUE(isInsideSupportPolygon(Eigen::Vector2d(0.5, 0.5), square));
  EXPECT_FALSE(isInsideSupportPolygon(Eigen::Vector2d(1.5, 0.5), square));
  EXPECT_FALSE(
      isInsideSupportPolygon(Eigen::Vector2d(1.0, 0.5), square, false));

  Eigen::Vector2d intersection;
  auto result = computeIntersection(
      intersection,
      Eigen::Vector2d(0.0, 0.0),
      Eigen::Vector2d(1.0, 1.0),
      Eigen::Vector2d(0.0, 1.0),
      Eigen::Vector2d(1.0, 0.0));
  EXPECT_EQ(result, INTERSECTING);
  EXPECT_TRUE(intersection.isApprox(Eigen::Vector2d(0.5, 0.5)));

  result = computeIntersection(
      intersection,
      Eigen::Vector2d(0.0, 0.0),
      Eigen::Vector2d(1.0, 0.0),
      Eigen::Vector2d(0.0, 1.0),
      Eigen::Vector2d(1.0, 1.0));
  EXPECT_EQ(result, PARALLEL);

  result = computeIntersection(
      intersection,
      Eigen::Vector2d(0.0, 0.0),
      Eigen::Vector2d(1.0, 0.0),
      Eigen::Vector2d(2.0, -1.0),
      Eigen::Vector2d(2.0, 1.0));
  EXPECT_EQ(result, BEYOND_ENDPOINTS);

  const auto closest = computeClosestPointOnLineSegment(
      Eigen::Vector2d(2.0, 0.5),
      Eigen::Vector2d(0.0, 0.0),
      Eigen::Vector2d(1.0, 0.0));
  EXPECT_TRUE(closest.isApprox(Eigen::Vector2d(1.0, 0.0)));

  std::size_t idx1 = 0;
  std::size_t idx2 = 0;
  const auto closestPoly = computeClosestPointOnSupportPolygon(
      idx1, idx2, Eigen::Vector2d(1.5, 0.5), square);
  EXPECT_TRUE(closestPoly.isApprox(Eigen::Vector2d(1.0, 0.5)));
  EXPECT_NE(idx1, idx2);
}

//==============================================================================
TEST(LieGroupOperators, AdjointMappings)
{
  int numTest = 100;

  // AdT(V) == T * V * InvT
  for (int i = 0; i < numTest; ++i) {
    Eigen::Vector6d t = Eigen::Vector6d::Random();
    Eigen::Isometry3d T = math::expMap(t);
    Eigen::Vector6d V = Eigen::Vector6d::Random();

    Eigen::Vector6d AdTV = AdT(T, V);

    // Ad(T, V) = T * [V] * InvT
    Eigen::Matrix4d T_V_InvT
        = T.matrix() * toMatrixForm(V) * T.inverse().matrix();
    Eigen::Vector6d T_V_InvT_se3 = fromMatrixForm(T_V_InvT);

    for (int j = 0; j < 6; ++j) {
      EXPECT_NEAR(AdTV(j), T_V_InvT_se3(j), LIE_GROUP_OPT_TOL);
    }

    // Ad(T, V) = [R 0; [p]R R] * V
    Eigen::Matrix6d AdTMatrix = Eigen::Matrix6d::Zero();
    AdTMatrix.topLeftCorner<3, 3>() = T.linear();
    AdTMatrix.bottomRightCorner<3, 3>() = T.linear();
    AdTMatrix.bottomLeftCorner<3, 3>()
        = math::makeSkewSymmetric(T.translation()) * T.linear();
    Eigen::Vector6d AdTMatrix_V = AdTMatrix * V;
    for (int j = 0; j < 6; ++j) {
      EXPECT_NEAR(AdTV(j), AdTMatrix_V(j), LIE_GROUP_OPT_TOL);
    }
  }

  // AdR == AdT([R 0; 0 1], V)
  for (int i = 0; i < numTest; ++i) {
    Eigen::Vector6d t = Eigen::Vector6d::Random();
    Eigen::Isometry3d T = math::expMap(t);
    Eigen::Isometry3d R = Eigen::Isometry3d::Identity();
    R.linear() = T.linear();
    Eigen::Vector6d V = Eigen::Vector6d::Random();

    Eigen::Vector6d AdTV = AdT(R, V);
    Eigen::Vector6d AdRV = AdR(T, V);

    for (int j = 0; j < 6; ++j) {
      EXPECT_NEAR(AdTV(j), AdRV(j), LIE_GROUP_OPT_TOL);
    }
  }

  // AdTAngular == AdT(T, se3(w, 0))
  for (int i = 0; i < numTest; ++i) {
    Eigen::Vector6d t = Eigen::Vector6d::Random();
    Eigen::Isometry3d T = math::expMap(t);
    Eigen::Vector3d w = Eigen::Vector3d::Random();
    Eigen::Vector6d V = Eigen::Vector6d::Zero();
    V.head<3>() = w;

    Eigen::Vector6d AdTV = AdT(T, V);
    Eigen::Vector6d AdTAng = AdTAngular(T, w);

    for (int j = 0; j < 6; ++j) {
      EXPECT_NEAR(AdTV(j), AdTAng(j), LIE_GROUP_OPT_TOL);
    }
  }

  // AdTLinear == AdT(T, se3(w, 0))
  for (int i = 0; i < numTest; ++i) {
    Eigen::Vector6d t = Eigen::Vector6d::Random();
    Eigen::Isometry3d T = math::expMap(t);
    Eigen::Vector3d v = Eigen::Vector3d::Random();
    Eigen::Vector6d V = Eigen::Vector6d::Zero();
    V.tail<3>() = v;

    Eigen::Vector6d AdTV = AdT(T, V);
    Eigen::Vector6d AdTLin = AdTLinear(T, v);

    for (int j = 0; j < 6; ++j) {
      EXPECT_NEAR(AdTV(j), AdTLin(j), LIE_GROUP_OPT_TOL);
    }
  }

  // AdTJac
  for (int i = 0; i < numTest; ++i) {
    Eigen::Vector6d t = Eigen::Vector6d::Random();
    Eigen::Isometry3d T = math::expMap(t);
    Eigen::Vector3d v = Eigen::Vector3d::Random();
    Eigen::Vector6d V = Eigen::Vector6d::Zero();
    V.tail<3>() = v;

    Eigen::Vector6d AdTV = AdT(T, V);
    Eigen::Vector6d AdTLin = AdTLinear(T, v);

    for (int j = 0; j < 6; ++j) {
      EXPECT_NEAR(AdTV(j), AdTLin(j), LIE_GROUP_OPT_TOL);
    }
  }

  // AdInvT
  for (int i = 0; i < numTest; ++i) {
    Eigen::Vector6d t = Eigen::Vector6d::Random();
    Eigen::Isometry3d T = math::expMap(t);
    Eigen::Isometry3d InvT = T.inverse();
    Eigen::Vector6d V = Eigen::Vector6d::Random();

    Eigen::Vector6d Ad_InvT = AdT(InvT, V);
    Eigen::Vector6d AdInv_T = AdInvT(T, V);

    for (int j = 0; j < 6; ++j) {
      EXPECT_NEAR(Ad_InvT(j), AdInv_T(j), LIE_GROUP_OPT_TOL);
    }
  }

  // AdInvRLinear
  for (int i = 0; i < numTest; ++i) {
    Eigen::Vector6d t = Eigen::Vector6d::Random();
    Eigen::Isometry3d T = math::expMap(t);
    Eigen::Vector3d v = Eigen::Vector3d::Random();
    Eigen::Vector6d V = Eigen::Vector6d::Zero();
    V.tail<3>() = v;
    Eigen::Isometry3d R = Eigen::Isometry3d::Identity();
    R.linear() = T.linear();

    Eigen::Vector6d AdT_ = AdT(R.inverse(), V);
    Eigen::Vector6d AdInvRLinear_ = AdInvRLinear(T, v);

    for (int j = 0; j < 6; ++j) {
      EXPECT_NEAR(AdT_(j), AdInvRLinear_(j), LIE_GROUP_OPT_TOL);
    }
  }

  // dAdT
  for (int i = 0; i < numTest; ++i) {
    Eigen::Vector6d t = Eigen::Vector6d::Random();
    Eigen::Isometry3d T = math::expMap(t);
    Eigen::Vector6d F = Eigen::Vector6d::Random();

    Eigen::Vector6d dAdTF = dAdT(T, F);

    // dAd(T, F) = [R 0; [p]R R]^T * F
    Eigen::Matrix6d AdTMatrix = Eigen::Matrix6d::Zero();
    AdTMatrix.topLeftCorner<3, 3>() = T.linear();
    AdTMatrix.bottomRightCorner<3, 3>() = T.linear();
    AdTMatrix.bottomLeftCorner<3, 3>()
        = math::makeSkewSymmetric(T.translation()) * T.linear();
    Eigen::Vector6d AdTTransMatrix_V = AdTMatrix.transpose() * F;
    for (int j = 0; j < 6; ++j) {
      EXPECT_NEAR(dAdTF(j), AdTTransMatrix_V(j), LIE_GROUP_OPT_TOL);
    }
  }

  // dAdInvT
  for (int i = 0; i < numTest; ++i) {
    Eigen::Vector6d t = Eigen::Vector6d::Random();
    Eigen::Isometry3d T = math::expMap(t);
    Eigen::Isometry3d InvT = T.inverse();
    Eigen::Vector6d F = Eigen::Vector6d::Random();

    Eigen::Vector6d dAdInvT_F = dAdInvT(T, F);

    //
    Eigen::Vector6d dAd_InvTF = dAdT(InvT, F);

    for (int j = 0; j < 6; ++j) {
      EXPECT_NEAR(dAdInvT_F(j), dAd_InvTF(j), LIE_GROUP_OPT_TOL);
    }

    // dAd(T, F) = [R 0; [p]R R]^T * F
    Eigen::Matrix6d AdInvTMatrix = Eigen::Matrix6d::Zero();
    AdInvTMatrix.topLeftCorner<3, 3>() = InvT.linear();
    AdInvTMatrix.bottomRightCorner<3, 3>() = InvT.linear();
    AdInvTMatrix.bottomLeftCorner<3, 3>()
        = math::makeSkewSymmetric(InvT.translation()) * InvT.linear();
    Eigen::Vector6d AdInvTTransMatrix_V = AdInvTMatrix.transpose() * F;
    for (int j = 0; j < 6; ++j) {
      EXPECT_NEAR(dAdInvT_F(j), AdInvTTransMatrix_V(j), LIE_GROUP_OPT_TOL);
    }
  }

  // dAdInvR
  for (int i = 0; i < numTest; ++i) {
    Eigen::Vector6d t = Eigen::Vector6d::Random();
    Eigen::Isometry3d T = math::expMap(t);
    Eigen::Isometry3d InvT = T.inverse();
    Eigen::Isometry3d InvR = Eigen::Isometry3d::Identity();
    InvR.linear() = InvT.linear();
    Eigen::Vector6d F = Eigen::Vector6d::Random();

    Eigen::Vector6d dAdInvR_F = dAdInvR(T, F);

    //
    Eigen::Vector6d dAd_InvTF = dAdT(InvR, F);

    for (int j = 0; j < 6; ++j) {
      EXPECT_NEAR(dAdInvR_F(j), dAd_InvTF(j), LIE_GROUP_OPT_TOL);
    }
  }

  // ad
  for (int i = 0; i < numTest; ++i) {
    Eigen::Vector6d V = Eigen::Vector6d::Random();
    Eigen::Vector6d W = Eigen::Vector6d::Random();

    Eigen::Vector6d ad_V_W = ad(V, W);

    //
    Eigen::Matrix6d adV_Matrix = Eigen::Matrix6d::Zero();
    adV_Matrix.topLeftCorner<3, 3>() = math::makeSkewSymmetric(V.head<3>());
    adV_Matrix.bottomRightCorner<3, 3>() = math::makeSkewSymmetric(V.head<3>());
    adV_Matrix.bottomLeftCorner<3, 3>() = math::makeSkewSymmetric(V.tail<3>());
    Eigen::Vector6d adV_Matrix_W = adV_Matrix * W;

    for (int j = 0; j < 6; ++j) {
      EXPECT_NEAR(ad_V_W(j), adV_Matrix_W(j), LIE_GROUP_OPT_TOL);
    }
  }

  // dad
  for (int i = 0; i < numTest; ++i) {
    Eigen::Vector6d V = Eigen::Vector6d::Random();
    Eigen::Vector6d F = Eigen::Vector6d::Random();

    Eigen::Vector6d dad_V_F = dad(V, F);

    //
    Eigen::Matrix6d dadV_Matrix = Eigen::Matrix6d::Zero();
    dadV_Matrix.topLeftCorner<3, 3>() = math::makeSkewSymmetric(V.head<3>());
    dadV_Matrix.bottomRightCorner<3, 3>()
        = math::makeSkewSymmetric(V.head<3>());
    dadV_Matrix.bottomLeftCorner<3, 3>() = math::makeSkewSymmetric(V.tail<3>());
    Eigen::Vector6d dadV_Matrix_F = dadV_Matrix.transpose() * F;

    for (int j = 0; j < 6; ++j) {
      EXPECT_NEAR(dad_V_F(j), dadV_Matrix_F(j), LIE_GROUP_OPT_TOL);
    }
  }
}
