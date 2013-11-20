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

#include <Eigen/LU>
#include <gtest/gtest.h>

#include "TestHelpers.h"

#include "common/Console.h"
#include "common/Timer.h"
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
#include "simulation/World.h"
#include "utils/Paths.h"
#include "utils/SkelParser.h"

using namespace dart;
using namespace math;
using namespace dynamics;

#define EOM_TOL 0.01

/******************************************************************************/
class EOM : public testing::Test
{
public:
    void equationsOfMotionTest(const std::string& _fileName);
};

//#ifndef NDEBUG
TEST_F(EOM, EquationOfMotionPerformance)
{
    common::Timer timer;
    std::vector<int> dofs;

    // Set 1
//    int numItr = 2000;
//    dofs.push_back(1);
//    dofs.push_back(2);
//    dofs.push_back(5);
//    dofs.push_back(10);
//    dofs.push_back(15);
//    dofs.push_back(20);
//    dofs.push_back(25);

    int numItr = 1000;
    dofs.push_back(5);
    dofs.push_back(10);
    dofs.push_back(15);
    dofs.push_back(20);
    dofs.push_back(25);
    dofs.push_back(30);
    dofs.push_back(35);
    dofs.push_back(40);
    dofs.push_back(45);
    dofs.push_back(50);

    // Set 2
//    int numItr = 500;
//    dofs.push_back(10);
//    dofs.push_back(20);
//    dofs.push_back(30);
//    dofs.push_back(40);
//    dofs.push_back(50);
//    dofs.push_back(60);
//    dofs.push_back(70);
//    dofs.push_back(80);
//    dofs.push_back(90);
//    dofs.push_back(100);

    // Set 3
//    int numItr = 5;
//    dofs.push_back(100);
//    dofs.push_back(200);
//    dofs.push_back(300);
//    dofs.push_back(400);
//    dofs.push_back(500);
//    dofs.push_back(600);
//    dofs.push_back(700);

    std::vector<double> oldResult(dofs.size(), 0);
    std::vector<double> newResult(dofs.size(), 0);
    std::vector<double> oldResultWithEOM(dofs.size(), 0);
    std::vector<double> newResultWithEOM(dofs.size(), 0);

    for (int i = 0; i < dofs.size(); ++i)
    {
        simulation::World world;
        dynamics::Skeleton* skeleton =
                createNLinkRobot(dofs[i], Eigen::Vector3d::Ones(), DOF_X, true);
        world.addSkeleton(skeleton);

        // Random state
        Eigen::VectorXd state = skeleton->getState();
        for (int k = 0; k < state.size(); ++k)
        {
            // TODO: The range is [-0.4pi, 0.4pi] until we resolve
            //       singular Jacobian issue.
            state[k] = math::random(-DART_PI*0.4, DART_PI*0.4);
        }
        skeleton->setState(state);

        timer.start();
        for (int j = 0; j < numItr; j++)
        {
            world.step();
        }
        timer.stop();
        newResult[i] = timer.getLastElapsedTime();
    }

    for (int i = 0; i < dofs.size(); ++i)
    {
        simulation::World world;
        dynamics::Skeleton* skeleton =
                createNLinkRobot(dofs[i], Eigen::Vector3d::Ones(), DOF_X, true);
        world.addSkeleton(skeleton);

        // Random state
        Eigen::VectorXd state = skeleton->getState();
        for (int k = 0; k < state.size(); ++k)
        {
            // TODO: The range is [-0.4pi, 0.4pi] until we resolve
            //       singular Jacobian issue.
            state[k] = math::random(-DART_PI*0.4, DART_PI*0.4);
        }
        skeleton->setState(state);

        timer.start();
        for (int j = 0; j < numItr; j++)
        {
            world.step();
            Eigen::MatrixXd M    = skeleton->getMassMatrix();
            Eigen::MatrixXd MInv = skeleton->getInvMassMatrix();
//            Eigen::VectorXd Cg   = skeleton->getCombinedVector();
//            Eigen::VectorXd C    = skeleton->getCoriolisForceVector();
//            Eigen::VectorXd g    = skeleton->getGravityForceVector();
//            Eigen::VectorXd Fext = skeleton->getExternalForceVector();
//            Eigen::MatrixXd J = skeleton->getBodyNode(skeleton->getNumBodyNodes()-1)->getBodyJacobian();
        }
        timer.stop();
        newResultWithEOM[i] = timer.getLastElapsedTime();
    }

//    for (int i = 0; i < dofs.size(); ++i)
//    {
//        simulation::World world;
//        dynamics::Skeleton* skeleton =
//                createNLinkRobot(dofs[i], Eigen::Vector3d::Ones(), DOF_X, true);
//        world.addSkeleton(skeleton);

//        // Random state
//        Eigen::VectorXd state = skeleton->getState();
//        for (int k = 0; k < state.size(); ++k)
//        {
//            // TODO: The range is [-0.4pi, 0.4pi] until we resolve
//            //       singular Jacobian issue.
//            state[k] = math::random(-DART_PI*0.4, DART_PI*0.4);
//        }
//        skeleton->setState(state);

//        timer.start();
//        for (int j = 0; j < numItr; j++)
//        {
//            world.step();
//        }
//        timer.stop();
//        oldResult[i] = timer.getLastElapsedTime();
//    }

//    for (int i = 0; i < dofs.size(); ++i)
//    {
//        simulation::World world;
//        dynamics::Skeleton* skeleton =
//                createNLinkRobot(dofs[i], Eigen::Vector3d::Ones(), DOF_X, true);
//        world.addSkeleton(skeleton);

//        // Random state
//        Eigen::VectorXd state = skeleton->getState();
//        for (int k = 0; k < state.size(); ++k)
//        {
//            // TODO: The range is [-0.4pi, 0.4pi] until we resolve
//            //       singular Jacobian issue.
//            state[k] = math::random(-DART_PI*0.4, DART_PI*0.4);
//        }
//        skeleton->setState(state);

//        timer.start();
//        for (int j = 0; j < numItr; j++)
//        {
//            world.step();
//            Eigen::MatrixXd M_OLD    = skeleton->getMassMatrix_OLD();
//            Eigen::MatrixXd MInv_OLD = skeleton->getInvMassMatrix_OLD();
//            Eigen::VectorXd Cg_OLD   = skeleton->getCombinedVector_OLD();
//            Eigen::VectorXd C_OLD    = skeleton->getCoriolisForceVector_OLD();
//            Eigen::VectorXd g_OLD    = skeleton->getGravityForceVector_OLD();
//            Eigen::VectorXd Fext_OLD = skeleton->getExternalForceVector_OLD();
//            Eigen::MatrixXd J = skeleton->getBodyNode(skeleton->getNumBodyNodes()-1)->getBodyJacobian();
//        }
//        timer.stop();
//        oldResultWithEOM[i] = timer.getLastElapsedTime();
//    }

    std::cout << "--------------------------------------------------------------" << std::endl;
    std::cout << std::setw(12) << " dof";
    for (int i = 0; i < dofs.size(); ++i)
        std::cout << " | " << std::setw(4) << dofs[i] << "";
    std::cout << std::endl;
    std::cout << "--------------------------------------------------------------" << std::endl;
    std::cout << std::setw(12) << " old w/o EOM";
    for (int i = 0; i < dofs.size(); ++i)
        std::cout << " | " << std::setw(4) << oldResult[i] << "";
    std::cout << std::endl;
    std::cout << std::setw(12) << " new w/o EOM";
    for (int i = 0; i < dofs.size(); ++i)
        std::cout << " | " << std::setw(4) << newResult[i] << "";
    std::cout << std::endl;
    std::cout << std::setw(12) << " old w EOM";
    for (int i = 0; i < dofs.size(); ++i)
        std::cout << " | " << std::setw(4) << oldResultWithEOM[i] << "";
    std::cout << std::endl;
    std::cout << std::setw(12) << " new w EOM";
    for (int i = 0; i < dofs.size(); ++i)
        std::cout << " | " << std::setw(4) << newResultWithEOM[i] << "";
    std::cout << std::endl;
}
//#endif

/******************************************************************************/
int main(int argc, char* argv[])
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
