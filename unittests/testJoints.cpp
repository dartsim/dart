/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/23/2013
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

#include <iostream>
#include <gtest/gtest.h>
#include "TestHelpers.h"

#include "common/UtilsCode.h"
#include "math/Geometry.h"
#include "math/Helpers.h"
#include "dynamics/BallJoint.h"
#include "dynamics/FreeJoint.h"
#include "dynamics/PrismaticJoint.h"
#include "dynamics/RevoluteJoint.h"
#include "dynamics/Skeleton.h"
#include "dynamics/TranslationalJoint.h"
#include "dynamics/UniversalJoint.h"
#include "dynamics/WeldJoint.h"
#include "dynamics/EulerJoint.h"
#include "dynamics/ScrewJoint.h"

using namespace dart;
using namespace math;
using namespace dynamics;

#define JOINT_TOL 0.01

/******************************************************************************/
class JOINTS : public testing::Test
{
public:
    void kinematicsTest(Joint* _joint);
};

/******************************************************************************/
void JOINTS::kinematicsTest(Joint* _joint)
{
    int dof = _joint->getNumGenCoords();

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    VectorXd q = VectorXd::Zero(dof);
    VectorXd dq = VectorXd::Zero(dof);
    VectorXd ddq = VectorXd::Zero(dof);

    for (int idxTest = 0; idxTest < 100; ++idxTest)
    {
        double dt = 0.000001;

        for (int i = 0; i < dof; ++i)
        {
            q(i) = random(-DART_PI, DART_PI);
            dq(i) = random(-DART_PI, DART_PI);
            ddq(i) = random(-DART_PI, DART_PI);
        }

        _joint->set_q(q);
        _joint->set_dq(dq);
        _joint->set_ddq(ddq);

        _joint->updateKinematics();

        if (_joint->getNumGenCoords() == 0)
            return;

        Eigen::Isometry3d T = _joint->getLocalTransform();
        Eigen::Vector6d V = _joint->getLocalVelocity();
        Jacobian J = _joint->getLocalJacobian();
        Eigen::Vector6d dV = _joint->getLocalAcceleration();
        Jacobian dJ = _joint->getLocalJacobianFirstDerivative();

        //--------------------------------------------------------------------------
        // Test T
        //--------------------------------------------------------------------------
        EXPECT_TRUE(math::verifyTransform(T));

        //--------------------------------------------------------------------------
        // Test V == J * dq
        //--------------------------------------------------------------------------
        Eigen::Vector6d Jdq = J * _joint->get_dq();
        for (int i = 0; i < 6; ++i)
            EXPECT_NEAR(V(i), Jdq(i), JOINT_TOL);

        //--------------------------------------------------------------------------
        // Test dV == dJ * dq + J * ddq
        //--------------------------------------------------------------------------
        Eigen::Vector6d dJdq = dJ * _joint->get_dq();
        Eigen::Vector6d Jddq = J * _joint->get_ddq();
        Eigen::Vector6d dJdq_Jddq = dJdq + Jddq;
        for (int i = 0; i < 6; ++i)
            EXPECT_NEAR(dV(i), dJdq_Jddq(i), JOINT_TOL);

        //--------------------------------------------------------------------------
        // Test analytic Jacobian and numerical Jacobian
        // J == numericalJ
        //--------------------------------------------------------------------------
        Jacobian numericJ = Jacobian::Zero(6,dof);
        for (int i = 0; i < dof; ++i)
        {
            // a
            Eigen::VectorXd q_a = q;
            _joint->set_q(q_a);
            _joint->updateKinematics();
            Eigen::Isometry3d T_a = _joint->getLocalTransform();

            // b
            Eigen::VectorXd q_b = q;
            q_b(i) += dt;
            _joint->set_q(q_b);
            _joint->updateKinematics();
            Eigen::Isometry3d T_b = _joint->getLocalTransform();

            //
            Eigen::Isometry3d Tinv_a = T_a.inverse();
            Eigen::Matrix4d Tinv_a_eigen = Tinv_a.matrix();

            // dTdq
            Eigen::Matrix4d T_a_eigen = T_a.matrix();
            Eigen::Matrix4d T_b_eigen = T_b.matrix();
            Eigen::Matrix4d dTdq_eigen = (T_b_eigen - T_a_eigen) / dt;
            //Matrix4d dTdq_eigen = (T_b_eigen * T_a_eigen.inverse()) / dt;

            // J(i)
            Eigen::Matrix4d Ji_4x4matrix_eigen = Tinv_a_eigen * dTdq_eigen;
            Eigen::Vector6d Ji;
            Ji[0] = Ji_4x4matrix_eigen(2,1);
            Ji[1] = Ji_4x4matrix_eigen(0,2);
            Ji[2] = Ji_4x4matrix_eigen(1,0);
            Ji[3] = Ji_4x4matrix_eigen(0,3);
            Ji[4] = Ji_4x4matrix_eigen(1,3);
            Ji[5] = Ji_4x4matrix_eigen(2,3);
            numericJ.col(i) = Ji;
        }

        for (int i = 0; i < dof; ++i)
            for (int j = 0; j < 6; ++j)
                EXPECT_NEAR(J.col(i)(j), numericJ.col(i)(j), JOINT_TOL);

        //--------------------------------------------------------------------------
        // Test first time derivative of analytic Jacobian and numerical Jacobian
        // dJ == numerical_dJ
        //--------------------------------------------------------------------------
        Jacobian numeric_dJ = Jacobian::Zero(6,dof);
        for (int i = 0; i < dof; ++i)
        {

            // a
            Eigen::VectorXd q_a = q;
            _joint->set_q(q_a);
            _joint->updateKinematics();
            Jacobian J_a = _joint->getLocalJacobian();

            // b
            Eigen::VectorXd q_b = q;
            q_b(i) += dt;
            _joint->set_q(q_b);
            _joint->updateKinematics();
            Jacobian J_b = _joint->getLocalJacobian();

            //
            Jacobian dJ_dq = (J_b - J_a) / dt;

            // J(i)
            numeric_dJ += dJ_dq * dq(i);
        }

        for (int i = 0; i < dof; ++i)
            for (int j = 0; j < 6; ++j)
                EXPECT_NEAR(dJ.col(i)(j), numeric_dJ.col(i)(j), JOINT_TOL);
    }
}

// 0-dof joint
TEST_F(JOINTS, WELD_JOINT)
{
    WeldJoint weldJoint;

    kinematicsTest(&weldJoint);
}

// 1-dof joint
TEST_F(JOINTS, REVOLUTE_JOINT)
{
    RevoluteJoint revJoint;

    kinematicsTest(&revJoint);
}

// 1-dof joint
TEST_F(JOINTS, PRISMATIC_JOINT)
{
    PrismaticJoint priJoint;

    kinematicsTest(&priJoint);
}

// 1-dof joint
TEST_F(JOINTS, SCREW_JOINT)
{
    ScrewJoint screwJoint;

    kinematicsTest(&screwJoint);
}

// 2-dof joint
TEST_F(JOINTS, UNIVERSAL_JOINT)
{
    UniversalJoint univJoint;

    kinematicsTest(&univJoint);
}

// 3-dof joint
TEST_F(JOINTS, BALL_JOINT)
{
    BallJoint ballJoint;

    kinematicsTest(&ballJoint);
}

// 3-dof joint
TEST_F(JOINTS, EULER_JOINT)
{
    EulerJoint eulerJoint;
    eulerJoint.setAxisOrder(EulerJoint::AO_XYZ);
    kinematicsTest(&eulerJoint);

    eulerJoint.setAxisOrder(EulerJoint::AO_ZYX);
    kinematicsTest(&eulerJoint);
}

// 3-dof joint
TEST_F(JOINTS, TRANSLATIONAL_JOINT)
{
    TranslationalJoint translationalJoint;

    kinematicsTest(&translationalJoint);
}

// 6-dof joint
TEST_F(JOINTS, FREE_JOINT)
{
    FreeJoint freeJoint;

    kinematicsTest(&freeJoint);
}

/******************************************************************************/
int main(int argc, char* argv[])
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}


