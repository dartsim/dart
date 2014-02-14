/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/PrismaticJoint.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/TranslationalJoint.h"
#include "dart/dynamics/UniversalJoint.h"
#include "dart/dynamics/WeldJoint.h"
#include "dart/dynamics/EulerJoint.h"
#include "dart/dynamics/ScrewJoint.h"
#include "dart/dynamics/PlanarJoint.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"
#include "dart/utils/Paths.h"
#include "dart/utils/SkelParser.h"

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
    int numTests = 100;

    BodyNode* bodyNode = new BodyNode();
    bodyNode->setParentJoint(_joint);

    Skeleton skeleton;
    skeleton.addBodyNode(bodyNode);
    skeleton.init();

    int dof = _joint->getNumGenCoords();

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    VectorXd q = VectorXd::Zero(dof);
    VectorXd dq = VectorXd::Zero(dof);

    for (int idxTest = 0; idxTest < 100; ++idxTest)
    {
        double q_delta = 0.000001;

        for (int i = 0; i < dof; ++i)
        {
            q(i) = random(-DART_PI*1.0, DART_PI*1.0);
            dq(i) = random(-DART_PI*1.0, DART_PI*1.0);
        }

        Eigen::VectorXd state = Eigen::VectorXd::Zero(2*dof);
        state.head(dof) = q;
        state.tail(dof) = dq;
        skeleton.setState(state);

        if (_joint->getNumGenCoords() == 0)
            return;

        Eigen::Isometry3d T = _joint->getLocalTransform();
        Jacobian J = _joint->getLocalJacobian();
        Jacobian dJ = _joint->getLocalJacobianTimeDeriv();

        //--------------------------------------------------------------------------
        // Test T
        //--------------------------------------------------------------------------
        EXPECT_TRUE(math::verifyTransform(T));

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
            skeleton.setConfig(q_a);
            Eigen::Isometry3d T_a = _joint->getLocalTransform();

            // b
            Eigen::VectorXd q_b = q;
            q_b(i) += q_delta;
            _joint->set_q(q_b);
            skeleton.setConfig(q_b);
            Eigen::Isometry3d T_b = _joint->getLocalTransform();

            //
            Eigen::Isometry3d Tinv_a = T_a.inverse();
            Eigen::Matrix4d Tinv_a_eigen = Tinv_a.matrix();

            // dTdq
            Eigen::Matrix4d T_a_eigen = T_a.matrix();
            Eigen::Matrix4d T_b_eigen = T_b.matrix();
            Eigen::Matrix4d dTdq_eigen = (T_b_eigen - T_a_eigen) / q_delta;
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
            skeleton.setConfig(q_a);
            Jacobian J_a = _joint->getLocalJacobian();

            // b
            Eigen::VectorXd q_b = q;
            q_b(i) += q_delta;
            _joint->set_q(q_b);
            skeleton.setConfig(q_b);
            Jacobian J_b = _joint->getLocalJacobian();

            //
            Jacobian dJ_dq = (J_b - J_a) / q_delta;

            // J(i)
            numeric_dJ += dJ_dq * dq(i);
        }

        for (int i = 0; i < dof; ++i)
            for (int j = 0; j < 6; ++j)
                EXPECT_NEAR(dJ.col(i)(j), numeric_dJ.col(i)(j), JOINT_TOL);
    }

    // Forward kinematics test with high joint position
    double posMin = -1e+64;
    double posMax = +1e+64;

    for (int idxTest = 0; idxTest < numTests; ++idxTest)
    {
        for (int i = 0; i < dof; ++i)
            q(i) = random(posMin, posMax);

        skeleton.setConfig(q);

        if (_joint->getNumGenCoords() == 0)
            return;

        Eigen::Isometry3d T = _joint->getLocalTransform();
        EXPECT_TRUE(math::verifyTransform(T));
    }
}

// 0-dof joint
TEST_F(JOINTS, WELD_JOINT)
{
    WeldJoint* weldJoint = new WeldJoint;

    kinematicsTest(weldJoint);
}

// 1-dof joint
TEST_F(JOINTS, REVOLUTE_JOINT)
{
    RevoluteJoint* revJoint = new RevoluteJoint;

    kinematicsTest(revJoint);
}

// 1-dof joint
TEST_F(JOINTS, PRISMATIC_JOINT)
{
    PrismaticJoint* priJoint = new PrismaticJoint;

    kinematicsTest(priJoint);
}

// 1-dof joint
TEST_F(JOINTS, SCREW_JOINT)
{
    ScrewJoint* screwJoint = new ScrewJoint;

    kinematicsTest(screwJoint);
}

// 2-dof joint
TEST_F(JOINTS, UNIVERSAL_JOINT)
{
    UniversalJoint* univJoint = new UniversalJoint;

    kinematicsTest(univJoint);
}

// 3-dof joint
//TEST_F(JOINTS, BALL_JOINT)
//{
//    BallJoint* ballJoint = new BallJoint;

//    kinematicsTest(ballJoint);
//}

// 3-dof joint
TEST_F(JOINTS, EULER_JOINT)
{
    EulerJoint* eulerJoint1 = new EulerJoint;

    eulerJoint1->setAxisOrder(EulerJoint::AO_XYZ);
    kinematicsTest(eulerJoint1);

    EulerJoint* eulerJoint2 = new EulerJoint;

    eulerJoint2->setAxisOrder(EulerJoint::AO_ZYX);
    kinematicsTest(eulerJoint2);
}

// 3-dof joint
TEST_F(JOINTS, TRANSLATIONAL_JOINT)
{
    TranslationalJoint* translationalJoint = new TranslationalJoint;

    kinematicsTest(translationalJoint);
}

// 3-dof joint
TEST_F(JOINTS, PLANAR_JOINT)
{
    PlanarJoint* planarJoint = new PlanarJoint;

    kinematicsTest(planarJoint);
}

// 6-dof joint
//TEST_F(JOINTS, FREE_JOINT)
//{
//    FreeJoint* freeJoint = new FreeJoint;

//    kinematicsTest(freeJoint);
//}

TEST_F(JOINTS, POSITION_LIMIT)
{
    double tol = 1e-4;

    simulation::World* myWorld = utils::SkelParser::readSkelFile(
            DART_DATA_PATH"/skel/test/joint_limit_test.skel");
    EXPECT_TRUE(myWorld != NULL);

    myWorld->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

    dynamics::Skeleton* pendulum = myWorld->getSkeleton("double_pendulum");
    EXPECT_TRUE(pendulum != NULL);

    dynamics::Joint* joint0 = pendulum->getJoint("joint0");
    dynamics::Joint* joint1 = pendulum->getJoint("joint1");

    EXPECT_TRUE(joint0 != NULL);
    EXPECT_TRUE(joint1 != NULL);

    double limit0 = DART_PI / 6.0;
    double limit1 = DART_PI / 6.0;

    joint0->setPositionLimited(true);
    joint0->getGenCoord(0)->set_qMin(-limit0);
    joint0->getGenCoord(0)->set_qMax(limit0);

    joint1->setPositionLimited(true);
    joint1->getGenCoord(0)->set_qMin(-limit1);
    joint1->getGenCoord(0)->set_qMax(limit1);

    double simTime = 2.0;
    double timeStep = myWorld->getTimeStep();
    int nSteps = simTime / timeStep;

    for (int i = 0; i < nSteps; i++)
    {
        myWorld->step();
    }

    double jointPos0 = joint0->getGenCoord(0)->get_q();
    double jointPos1 = joint1->getGenCoord(0)->get_q();

    double jointVel0 = joint0->getGenCoord(0)->get_dq();
    double jointVel1 = joint1->getGenCoord(0)->get_dq();

    // NOTE: The ideal result is that the joint position limit was obeyed with
    //       zero tolerance. To do so, DART should correct the joint limit
    //       violation in which is not implemented yet. This feature should be
    //       added in DART.
    EXPECT_NEAR(jointPos0, -limit0, 1e-4);
    EXPECT_NEAR(jointPos1, -limit1, 1e-3);

    EXPECT_NEAR(jointVel0, 0.0, tol);
    EXPECT_NEAR(jointVel1, 0.0, tol);
}

/******************************************************************************/
int main(int argc, char* argv[])
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}


