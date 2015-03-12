/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
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
#include <gtest/gtest.h>
#include "TestHelpers.h"

#include "dart/math/Geometry.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/BallJoint.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/PrismaticJoint.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/WeldJoint.h"
#include "dart/dynamics/TranslationalJoint.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;

#define LIE_GROUP_OPT_TOL 1e-12

/******************************************************************************/
Eigen::Matrix4d toMatrixForm(const Eigen::Vector6d& v) {
    Eigen::Matrix4d result = Eigen::Matrix4d::Zero();

    result(0, 1) = -v(2);
    result(1, 0) =  v(2);
    result(0, 2) =  v(1);
    result(2, 0) = -v(1);
    result(1, 2) = -v(0);
    result(2, 1) =  v(0);

    result(0, 3) = v(3);
    result(1, 3) = v(4);
    result(2, 3) = v(5);

    return result;
}

/******************************************************************************/
Eigen::Vector6d fromMatrixForm(const Eigen::Matrix4d& m) {
    Eigen::Vector6d ret;
    ret << m(2,1), m(0,2), m(1,0), m(0,3), m(1,3), m(2,3);
    return ret;
}

/******************************************************************************/
void testEulerAngles(const Eigen::Vector3d& angle)
{
    Eigen::Matrix3d mat1;
    Eigen::Matrix3d mat2;

    // XYX
    mat1 = math::eulerXYXToMatrix(angle);
    mat2 = Eigen::AngleAxisd(angle(0), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(angle(1), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(angle(2), Eigen::Vector3d::UnitX());

    EXPECT_TRUE(math::verifyRotation(mat1));
    EXPECT_TRUE(math::verifyRotation(mat2));
    EXPECT_TRUE(equals(mat1, mat2));
    EXPECT_TRUE(equals(mat1, eulerXYXToMatrix(matrixToEulerXYX(mat1))));

    // XYZ
    mat1 = math::eulerXYZToMatrix(angle);
    mat2 = Eigen::AngleAxisd(angle(0), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(angle(1), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(angle(2), Eigen::Vector3d::UnitZ());

    EXPECT_TRUE(math::verifyRotation(mat1));
    EXPECT_TRUE(math::verifyRotation(mat2));
    EXPECT_TRUE(equals(mat1, mat2));
    EXPECT_TRUE(equals(mat1, eulerXYZToMatrix(matrixToEulerXYZ(mat1))));

    // XZX
    mat1 = math::eulerXZXToMatrix(angle);
    mat2 = Eigen::AngleAxisd(angle(0), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(angle(1), Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(angle(2), Eigen::Vector3d::UnitX());

    EXPECT_TRUE(math::verifyRotation(mat1));
    EXPECT_TRUE(math::verifyRotation(mat2));
    EXPECT_TRUE(equals(mat1, mat2));
    //EXPECT_TRUE(equals(mat1, eulerXZXToMatrix(matrixToEulerXZX(mat1))));

    // XZY
    mat1 = math::eulerXZYToMatrix(angle);
    mat2 = Eigen::AngleAxisd(angle(0), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(angle(1), Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(angle(2), Eigen::Vector3d::UnitY());

    EXPECT_TRUE(math::verifyRotation(mat1));
    EXPECT_TRUE(math::verifyRotation(mat2));
    EXPECT_TRUE(equals(mat1, mat2));
    EXPECT_TRUE(equals(mat1, eulerXZYToMatrix(matrixToEulerXZY(mat1))));

    // YXY
    mat1 = math::eulerYXYToMatrix(angle);
    mat2 = Eigen::AngleAxisd(angle(0), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(angle(1), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(angle(2), Eigen::Vector3d::UnitY());

    EXPECT_TRUE(math::verifyRotation(mat1));
    EXPECT_TRUE(math::verifyRotation(mat2));
    EXPECT_TRUE(equals(mat1, mat2));
    //EXPECT_TRUE(equals(mat1, eulerYXYToMatrix(matrixToEulerYXY(mat1))));

    // YXZ
    mat1 = math::eulerYXZToMatrix(angle);
    mat2 = Eigen::AngleAxisd(angle(0), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(angle(1), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(angle(2), Eigen::Vector3d::UnitZ());

    EXPECT_TRUE(math::verifyRotation(mat1));
    EXPECT_TRUE(math::verifyRotation(mat2));
    EXPECT_TRUE(equals(mat1, mat2));
    EXPECT_TRUE(equals(mat1, eulerYXZToMatrix(matrixToEulerYXZ(mat1))));

    // YZX
    mat1 = math::eulerYZXToMatrix(angle);
    mat2 = Eigen::AngleAxisd(angle(0), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(angle(1), Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(angle(2), Eigen::Vector3d::UnitX());

    EXPECT_TRUE(math::verifyRotation(mat1));
    EXPECT_TRUE(math::verifyRotation(mat2));
    EXPECT_TRUE(equals(mat1, mat2));
    EXPECT_TRUE(equals(mat1, eulerYZXToMatrix(matrixToEulerYZX(mat1))));

    // YZY
    mat1 = math::eulerYZYToMatrix(angle);
    mat2 = Eigen::AngleAxisd(angle(0), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(angle(1), Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(angle(2), Eigen::Vector3d::UnitY());

    EXPECT_TRUE(math::verifyRotation(mat1));
    EXPECT_TRUE(math::verifyRotation(mat2));
    EXPECT_TRUE(equals(mat1, mat2));
    //EXPECT_TRUE(equals(mat1, eulerYZYToMatrix(matrixToEulerYZY(mat1))));

    // ZXY
    mat1 = math::eulerZXYToMatrix(angle);
    mat2 = Eigen::AngleAxisd(angle(0), Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(angle(1), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(angle(2), Eigen::Vector3d::UnitY());

    EXPECT_TRUE(math::verifyRotation(mat1));
    EXPECT_TRUE(math::verifyRotation(mat2));
    EXPECT_TRUE(equals(mat1, mat2));
    EXPECT_TRUE(equals(mat1, eulerZXYToMatrix(matrixToEulerZXY(mat1))));

    // ZYX
    mat1 = math::eulerZYXToMatrix(angle);
    mat2 = Eigen::AngleAxisd(angle(0), Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(angle(1), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(angle(2), Eigen::Vector3d::UnitX());

    EXPECT_TRUE(math::verifyRotation(mat1));
    EXPECT_TRUE(math::verifyRotation(mat2));
    EXPECT_TRUE(equals(mat1, mat2));
    EXPECT_TRUE(equals(mat1, eulerZYXToMatrix(matrixToEulerZYX(mat1))));

    // ZXZ
    mat1 = math::eulerZXZToMatrix(angle);
    mat2 = Eigen::AngleAxisd(angle(0), Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(angle(1), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(angle(2), Eigen::Vector3d::UnitZ());

    EXPECT_TRUE(math::verifyRotation(mat1));
    EXPECT_TRUE(math::verifyRotation(mat2));
    EXPECT_TRUE(equals(mat1, mat2));
    //EXPECT_TRUE(equals(mat1, eulerZXZToMatrix(matrixToEulerZXZ(mat1))));

    // ZYZ
    mat1 = math::eulerZYZToMatrix(angle);
    mat2 = Eigen::AngleAxisd(angle(0), Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(angle(1), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(angle(2), Eigen::Vector3d::UnitZ());

    EXPECT_TRUE(math::verifyRotation(mat1));
    EXPECT_TRUE(math::verifyRotation(mat2));
    EXPECT_TRUE(equals(mat1, mat2));
    //EXPECT_TRUE(equals(mat1, eulerZYZToMatrix(matrixToEulerZYZ(mat1))));
}

/******************************************************************************/
TEST(LIE_GROUP_OPERATORS, EULER_ANGLES)
{
    // TODO: Special angles such as (PI, 0, 0)

    //
    int numTest = 1;
    for (int i = 0; i < numTest; ++i)
    {
        Eigen::Vector3d angle = Eigen::Vector3d::Random();
        testEulerAngles(angle);
    }
}

/******************************************************************************/
#define EPSILON_EXPMAP_THETA 1.0e-3
TEST(LIE_GROUP_OPERATORS, EXPONENTIAL_MAPPINGS)
{
    int numTest = 100;

    // Exp
    for (int i = 0; i < numTest; ++i)
    {
        Eigen::Vector6d s = Eigen::Vector6d::Random();
        Eigen::Isometry3d Exp_s = math::expMap(s);
        Eigen::Matrix4d Exp_s_2 = Eigen::Matrix4d::Identity();

        double theta = s.head<3>().norm();
        Eigen::Matrix3d R = Matrix3d::Zero();
        Eigen::Matrix3d qss =  math::makeSkewSymmetric(s.head<3>());
        Eigen::Matrix3d qss2 =  qss*qss;
        Eigen::Matrix3d P = Eigen::Matrix3d::Zero();

        if (theta < EPSILON_EXPMAP_THETA)
        {
            R = Matrix3d::Identity() + qss + 0.5*qss2;
            P = Matrix3d::Identity() + 0.5*qss + (1/6)*qss2;
        }
        else
        {
            R = Matrix3d::Identity() + (sin(theta)/theta)*qss + ((1-cos(theta))/(theta*theta))*qss2;
            P = Matrix3d::Identity() + ((1-cos(theta))/(theta*theta))*qss + ((theta-sin(theta))/(theta*theta*theta))*qss2;
        }

        Exp_s_2.topLeftCorner<3,3>() = R;
        Exp_s_2.topRightCorner<3,1>() = P*s.tail<3>();

        //
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                EXPECT_NEAR(Exp_s(i,j), Exp_s_2(i,j), LIE_GROUP_OPT_TOL);
    }

    // ExpAngular
    for (int i = 0; i < numTest; ++i)
    {
        Eigen::Vector6d s = Eigen::Vector6d::Random();
        s.tail<3>() = Eigen::Vector3d::Zero();
        Eigen::Isometry3d Exp_s = math::expAngular(s.head<3>());
        Eigen::Matrix4d Exp_s_2 = Eigen::Matrix4d::Identity();

        double theta = s.head<3>().norm();
        Eigen::Matrix3d R = Matrix3d::Zero();
        Eigen::Matrix3d qss =  math::makeSkewSymmetric(s.head<3>());
        Eigen::Matrix3d qss2 =  qss*qss;
        Eigen::Matrix3d P = Eigen::Matrix3d::Zero();

        if (theta < EPSILON_EXPMAP_THETA)
        {
            R = Matrix3d::Identity() + qss + 0.5*qss2;
            P = Matrix3d::Identity() + 0.5*qss + (1/6)*qss2;
        }
        else
        {
            R = Matrix3d::Identity() + (sin(theta)/theta)*qss + ((1-cos(theta))/(theta*theta))*qss2;
            P = Matrix3d::Identity() + ((1-cos(theta))/(theta*theta))*qss + ((theta-sin(theta))/(theta*theta*theta))*qss2;
        }

        Exp_s_2.topLeftCorner<3,3>() = R;
        Exp_s_2.topRightCorner<3,1>() = P*s.tail<3>();

        //
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                EXPECT_NEAR(Exp_s(i,j), Exp_s_2(i,j), LIE_GROUP_OPT_TOL);
    }

    // ExpLinear
    for (int i = 0; i < numTest; ++i)
    {
        Eigen::Vector6d s = Eigen::Vector6d::Random();
        s.head<3>() = Eigen::Vector3d::Zero();
        Eigen::Isometry3d Exp_s(Eigen::Translation3d(s.tail<3>()));
        Eigen::Matrix4d Exp_s_2 = Eigen::Matrix4d::Identity();

        double theta = s.head<3>().norm();
        Eigen::Matrix3d R = Matrix3d::Zero();
        Eigen::Matrix3d qss =  math::makeSkewSymmetric(s.head<3>());
        Eigen::Matrix3d qss2 =  qss*qss;
        Eigen::Matrix3d P = Eigen::Matrix3d::Zero();

        if (theta < EPSILON_EXPMAP_THETA)
        {
            R = Matrix3d::Identity() + qss + 0.5*qss2;
            P = Matrix3d::Identity() + 0.5*qss + (1/6)*qss2;
        }
        else
        {
            R = Matrix3d::Identity() + (sin(theta)/theta)*qss + ((1-cos(theta))/(theta*theta))*qss2;
            P = Matrix3d::Identity() + ((1-cos(theta))/(theta*theta))*qss + ((theta-sin(theta))/(theta*theta*theta))*qss2;
        }

        Exp_s_2.topLeftCorner<3,3>() = R;
        Exp_s_2.topRightCorner<3,1>() = P*s.tail<3>();

        //
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                EXPECT_NEAR(Exp_s(i,j), Exp_s_2(i,j), LIE_GROUP_OPT_TOL);
    }
    // Exponential mapping test with high values
    int numExpTests = 100;
    double min = -1e+128;
    double max = +1e+128;

    for (int idxTest = 0; idxTest < numExpTests; ++idxTest)
    {
        Eigen::Vector3d randomS = Eigen::Vector3d::Zero();

        for (int i = 0; i < 3; ++i)
          randomS[i] = random(min, max);

        Eigen::Isometry3d T = math::expAngular(randomS);
        EXPECT_TRUE(math::verifyTransform(T));
    }

    for (int idxTest = 0; idxTest < numExpTests; ++idxTest)
    {
        Eigen::Vector6d randomS = Eigen::Vector6d::Zero();

        for (int i = 0; i < 6; ++i)
          randomS[i] = random(min, max);

        Eigen::Isometry3d T = math::expMap(randomS);
        EXPECT_TRUE(math::verifyTransform(T));
    }
}

/******************************************************************************/
TEST(LIE_GROUP_OPERATORS, ADJOINT_MAPPINGS)
{
    int numTest = 100;

    // AdT(V) == T * V * InvT
    for (int i = 0; i < numTest; ++i)
    {
        Eigen::Vector6d t = Eigen::Vector6d::Random();
        Eigen::Isometry3d T = math::expMap(t);
        Eigen::Vector6d V = Eigen::Vector6d::Random();

        Eigen::Vector6d AdTV = AdT(T, V);

        // Ad(T, V) = T * [V] * InvT
        Eigen::Matrix4d T_V_InvT = T.matrix() * toMatrixForm(V) * T.inverse().matrix();
        Eigen::Vector6d T_V_InvT_se3 = fromMatrixForm(T_V_InvT);

        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(AdTV(j), T_V_InvT_se3(j), LIE_GROUP_OPT_TOL);

        // Ad(T, V) = [R 0; [p]R R] * V
        Eigen::Matrix6d AdTMatrix = Eigen::Matrix6d::Zero();
        AdTMatrix.topLeftCorner<3,3>() = T.linear();
        AdTMatrix.bottomRightCorner<3,3>() = T.linear();
        AdTMatrix.bottomLeftCorner<3,3>() = math::makeSkewSymmetric(T.translation()) * T.linear();
        Eigen::Vector6d AdTMatrix_V = AdTMatrix * V;
        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(AdTV(j), AdTMatrix_V(j), LIE_GROUP_OPT_TOL);
    }

    // AdR == AdT([R 0; 0 1], V)
    for (int i = 0; i < numTest; ++i)
    {
        Eigen::Vector6d t = Eigen::Vector6d::Random();
        Eigen::Isometry3d T = math::expMap(t);
        Eigen::Isometry3d R = Eigen::Isometry3d::Identity();
        R = T.linear();
        Eigen::Vector6d V = Eigen::Vector6d::Random();

        Eigen::Vector6d AdTV = AdT(R, V);
        Eigen::Vector6d AdRV = AdR(T, V);

        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(AdTV(j), AdRV(j), LIE_GROUP_OPT_TOL);
    }

    // AdTAngular == AdT(T, se3(w, 0))
    for (int i = 0; i < numTest; ++i)
    {
        Eigen::Vector6d t = Eigen::Vector6d::Random();
        Eigen::Isometry3d T = math::expMap(t);
        Eigen::Vector3d w = Eigen::Vector3d::Random();
        Eigen::Vector6d V = Eigen::Vector6d::Zero();
        V.head<3>() = w;

        Eigen::Vector6d AdTV = AdT(T, V);
        Eigen::Vector6d AdTAng = AdTAngular(T, w);

        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(AdTV(j), AdTAng(j), LIE_GROUP_OPT_TOL);
    }

    // AdTLinear == AdT(T, se3(w, 0))
    for (int i = 0; i < numTest; ++i)
    {
        Eigen::Vector6d t = Eigen::Vector6d::Random();
        Eigen::Isometry3d T = math::expMap(t);
        Eigen::Vector3d v = Eigen::Vector3d::Random();
        Eigen::Vector6d V = Eigen::Vector6d::Zero();
        V.tail<3>() = v;

        Eigen::Vector6d AdTV = AdT(T, V);
        Eigen::Vector6d AdTLin = AdTLinear(T, v);

        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(AdTV(j), AdTLin(j), LIE_GROUP_OPT_TOL);
    }

    // AdTJac
    for (int i = 0; i < numTest; ++i)
    {
        Eigen::Vector6d t = Eigen::Vector6d::Random();
        Eigen::Isometry3d T = math::expMap(t);
        Eigen::Vector3d v = Eigen::Vector3d::Random();
        Eigen::Vector6d V = Eigen::Vector6d::Zero();
        V.tail<3>() = v;

        Eigen::Vector6d AdTV = AdT(T, V);
        Eigen::Vector6d AdTLin = AdTLinear(T, v);

        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(AdTV(j), AdTLin(j), LIE_GROUP_OPT_TOL);
    }

    // AdInvT
    for (int i = 0; i < numTest; ++i)
    {
        Eigen::Vector6d t = Eigen::Vector6d::Random();
        Eigen::Isometry3d T = math::expMap(t);
        Eigen::Isometry3d InvT = T.inverse();
        Eigen::Vector6d V = Eigen::Vector6d::Random();

        Eigen::Vector6d Ad_InvT = AdT(InvT, V);
        Eigen::Vector6d AdInv_T = AdInvT(T, V);

        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(Ad_InvT(j), AdInv_T(j), LIE_GROUP_OPT_TOL);
    }

    // AdInvRLinear
    for (int i = 0; i < numTest; ++i)
    {
        Eigen::Vector6d t = Eigen::Vector6d::Random();
        Eigen::Isometry3d T = math::expMap(t);
        Eigen::Vector3d v = Eigen::Vector3d::Random();
        Eigen::Vector6d V = Eigen::Vector6d::Zero();
        V.tail<3>() = v;
        Eigen::Isometry3d R = Eigen::Isometry3d::Identity();
        R = T.linear();

        Eigen::Vector6d AdT_ = AdT(R.inverse(), V);
        Eigen::Vector6d AdInvRLinear_ = AdInvRLinear(T, v);

        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(AdT_(j), AdInvRLinear_(j), LIE_GROUP_OPT_TOL);
    }

    // dAdT
    for (int i = 0; i < numTest; ++i)
    {
        Eigen::Vector6d t = Eigen::Vector6d::Random();
        Eigen::Isometry3d T = math::expMap(t);
        Eigen::Vector6d F = Eigen::Vector6d::Random();

        Eigen::Vector6d dAdTF = dAdT(T, F);

        // dAd(T, F) = [R 0; [p]R R]^T * F
        Eigen::Matrix6d AdTMatrix = Eigen::Matrix6d::Zero();
        AdTMatrix.topLeftCorner<3,3>() = T.linear();
        AdTMatrix.bottomRightCorner<3,3>() = T.linear();
        AdTMatrix.bottomLeftCorner<3,3>() = math::makeSkewSymmetric(T.translation()) * T.linear();
        Eigen::Vector6d AdTTransMatrix_V = AdTMatrix.transpose() * F;
        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(dAdTF(j), AdTTransMatrix_V(j), LIE_GROUP_OPT_TOL);
    }

    // dAdInvT
    for (int i = 0; i < numTest; ++i)
    {
        Eigen::Vector6d t = Eigen::Vector6d::Random();
        Eigen::Isometry3d T = math::expMap(t);
        Eigen::Isometry3d InvT = T.inverse();
        Eigen::Vector6d F = Eigen::Vector6d::Random();

        Eigen::Vector6d dAdInvT_F = dAdInvT(T, F);

        //
        Eigen::Vector6d dAd_InvTF = dAdT(InvT, F);

        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(dAdInvT_F(j), dAd_InvTF(j), LIE_GROUP_OPT_TOL);

        // dAd(T, F) = [R 0; [p]R R]^T * F
        Eigen::Matrix6d AdInvTMatrix = Eigen::Matrix6d::Zero();
        AdInvTMatrix.topLeftCorner<3,3>() = InvT.linear();
        AdInvTMatrix.bottomRightCorner<3,3>() = InvT.linear();
        AdInvTMatrix.bottomLeftCorner<3,3>() = math::makeSkewSymmetric(InvT.translation()) * InvT.linear();
        Eigen::Vector6d AdInvTTransMatrix_V = AdInvTMatrix.transpose() * F;
        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(dAdInvT_F(j), AdInvTTransMatrix_V(j), LIE_GROUP_OPT_TOL);
    }

    // dAdInvR
    for (int i = 0; i < numTest; ++i)
    {
        Eigen::Vector6d t = Eigen::Vector6d::Random();
        Eigen::Isometry3d T = math::expMap(t);
        Eigen::Isometry3d InvT = T.inverse();
        Eigen::Isometry3d InvR = Eigen::Isometry3d::Identity();
        InvR = InvT.linear();
        Eigen::Vector6d F = Eigen::Vector6d::Random();

        Eigen::Vector6d dAdInvR_F = dAdInvR(T, F);

        //
        Eigen::Vector6d dAd_InvTF = dAdT(InvR, F);

        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(dAdInvR_F(j), dAd_InvTF(j), LIE_GROUP_OPT_TOL);
    }

    // ad
    for (int i = 0; i < numTest; ++i)
    {
        Eigen::Vector6d V = Eigen::Vector6d::Random();
        Eigen::Vector6d W = Eigen::Vector6d::Random();

        Eigen::Vector6d ad_V_W = ad(V, W);

        //
        Eigen::Matrix6d adV_Matrix = Eigen::Matrix6d::Zero();
        adV_Matrix.topLeftCorner<3,3>() = math::makeSkewSymmetric(V.head<3>());
        adV_Matrix.bottomRightCorner<3,3>() = math::makeSkewSymmetric(V.head<3>());
        adV_Matrix.bottomLeftCorner<3,3>() = math::makeSkewSymmetric(V.tail<3>());
        Eigen::Vector6d adV_Matrix_W = adV_Matrix * W;

        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(ad_V_W(j), adV_Matrix_W(j), LIE_GROUP_OPT_TOL);
    }

    // dad
    for (int i = 0; i < numTest; ++i)
    {
        Eigen::Vector6d V = Eigen::Vector6d::Random();
        Eigen::Vector6d F = Eigen::Vector6d::Random();

        Eigen::Vector6d dad_V_F = dad(V, F);

        //
        Eigen::Matrix6d dadV_Matrix = Eigen::Matrix6d::Zero();
        dadV_Matrix.topLeftCorner<3,3>() = math::makeSkewSymmetric(V.head<3>());
        dadV_Matrix.bottomRightCorner<3,3>() = math::makeSkewSymmetric(V.head<3>());
        dadV_Matrix.bottomLeftCorner<3,3>() = math::makeSkewSymmetric(V.tail<3>());
        Eigen::Vector6d dadV_Matrix_F= dadV_Matrix.transpose() * F;

        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(dad_V_F(j), dadV_Matrix_F(j), LIE_GROUP_OPT_TOL);
    }
}

Eigen::Vector3d log(const Eigen::Matrix3d& _m)
{
  // T = (R, p) = exp([w, v]), t = ||w||
  // v = beta*p + gamma*w + 1 / 2*cross(p, w)
  // where
  // beta  = t*(1 + cos(t))/(2*sin(t))
  // gamma = <w, p>*(1 - beta)/t^2

  // Restricting theta to the interval [0, pi]
  const double cos_t = std::max(std::min(0.5 * (_m.trace() - 1.0), 1.0), -1.0);
  const double theta = std::acos(cos_t);

  // If theta is very close to pi (note that theta cannot be greater than pi)
  if (DART_PI - theta < DART_EPSILON)
  {
    const double pi_theta = DART_PI - theta;
    const double delta    = 0.5 + 0.125*pi_theta*pi_theta;

    return Eigen::Vector3d(
          _m(2, 1) > _m(1, 2) ?
             theta*std::sqrt(1.0 + (_m(0, 0) - 1.0)*delta) :
            -theta*std::sqrt(1.0 + (_m(0, 0) - 1.0)*delta),
          _m(0, 2) > _m(2, 0) ?
             theta*std::sqrt(1.0 + (_m(1, 1) - 1.0)*delta) :
            -theta*std::sqrt(1.0 + (_m(1, 1) - 1.0)*delta),
          _m(1, 0) > _m(0, 1) ?
             theta*std::sqrt(1.0 + (_m(2, 2) - 1.0)*delta) :
            -theta*std::sqrt(1.0 + (_m(2, 2) - 1.0)*delta));
  }
  else
  {
    double alpha;

    if (theta > DART_EPSILON)
      alpha = 0.5*theta/std::sin(theta);
    else
      alpha = 3.0/(6.0 - theta*theta);

    return Eigen::Vector3d(alpha*(_m(2, 1) - _m(1, 2)),
                           alpha*(_m(0, 2) - _m(2, 0)),
                           alpha*(_m(1, 0) - _m(0, 1)));
  }
}

Eigen::Vector3d log2(const Eigen::Matrix3d& _R)
{
  // T = (R, p) = exp([w, v]), t = ||w||
  // v = beta*p + gamma*w + 1 / 2*cross(p, w)
  // where
  // beta  = t*(1 + cos(t))/(2*sin(t))
  // gamma = <w, p>*(1 - beta)/t^2

  // cos_t
  double cos_t = 0.5*(_R.trace() - 1.0);

  // If cos_t is close to -1 (note that the range of cos_t is [+1, -1]), which
  // means the theta is close to k*pi (k = odd integer)
  constexpr double epsMinus1 = DART_EPSILON - 1.0;
  if (cos_t < epsMinus1)
  {
    constexpr double oneMinusEps = 1.0 - DART_EPSILON;
    if (_R(0, 0) > oneMinusEps)
    {
      return Eigen::Vector3d(DART_PI, 0.0, 0.0);
    }
    else if (_R(1, 1) > oneMinusEps)
    {
      return Eigen::Vector3d(0.0, DART_PI, 0.0);
    }
    else if ( _R(2, 2) > oneMinusEps)
    {
      return Eigen::Vector3d(0.0, 0.0, DART_PI);
    }
    else
    {
      constexpr double piOverSin2 = DART_PI/std::sqrt(2.0);

      return Eigen::Vector3d(
            piOverSin2*std::sqrt((_R(1,0)*_R(1,0) + _R(2,0)*_R(2,0))/(1.0 - _R(0,0))),
            piOverSin2*std::sqrt((_R(0,1)*_R(0,1) + _R(2,1)*_R(2,1))/(1.0 - _R(1,1))),
            piOverSin2*std::sqrt((_R(0,2)*_R(0,2) + _R(1,2)*_R(1,2))/(1.0 - _R(2,2))));
    }
  }
  else
  {
    if (cos_t > 1.0)
    {
      cos_t = 1.0;
    }
    else if(cos_t < -1.0)
    {
      cos_t = -1.0;
    }

    const double theta = std::acos(cos_t);

    double t_st;

    if (theta < DART_EPSILON)
      t_st = 3.0/(6.0 - theta*theta);
    else
      t_st = theta/(2.0 * std::sin(theta));

    return Eigen::Vector3d(t_st*(_R(2, 1) - _R(1, 2)),
                           t_st*(_R(0, 2) - _R(2, 0)),
                           t_st*(_R(1, 0) - _R(0, 1)));
  }
}

//==============================================================================
Eigen::Vector6d convertToPositions(const Eigen::Isometry3d& _tf)
{
  Eigen::Vector6d x;
  x.head<3>() = dart::math::logMap(_tf.linear());
//  x.head<3>() = log2(_tf.linear());
  x.tail<3>() = _tf.translation();
  return x;
}

//==============================================================================
Eigen::Isometry3d convertToTransform(
    const Eigen::Vector6d& _positions)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear() = dart::math::expMapRot(_positions.head<3>());
  tf.translation() = _positions.tail<3>();
  return tf;
}
//==============================================================================
TEST(Geometry, Issue333)
{
  Eigen::Isometry3d Tf;
  Eigen::VectorXd q;
  Eigen::Isometry3d Tf_back;

  Tf.setIdentity();
  Tf.translation() << 0.1, 0.2, 0.3;

  double dang = 10*M_PI/180.0;
  int n = (int) (2*M_PI /dang);

  Tf.matrix() << -0.815214,  -0.57916,        -0,  0.437796,
    -0.57916,  0.815214,         0,  0.488175,
        0,         0,        -1,      1.03,
        0,         0,         0,         1;

   double det = Tf.linear().determinant();
  std::cout << "Determinant of test TF: "<< det << std::endl;

/*
  for( size_t i = 0; i < n; ++i ) {
    Eigen::Matrix3d rot;
    rot = Eigen::AngleAxisd( i*dang, Eigen::Vector3d(0,0,1) );
    Tf.linear() = rot;
*/
    // Get position with helper function
    q = convertToPositions( Tf );

    // Get translation back with helper function
    Tf_back = convertToTransform( q );
    //std::cout << "i: "<< i << std::endl;
    std::cout << "\n ** Tf original: \n"<< Tf.matrix() << std::endl;
    std::cout << "\n ** Positions from Tf: \n"<< q.transpose() << std::endl;
    std::cout << "\n ** Tf from q: \n" << Tf_back.matrix() << std::endl;
    std::cout << "\n\n" << std::endl;

/*
  }
*/
}


/******************************************************************************/
int main(int argc, char* argv[])
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}


