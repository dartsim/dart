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

#include <Eigen/LU>
#include <gtest/gtest.h>

#include "TestHelpers.h"

#include "dart/common/Console.h"
#include "dart/common/Timer.h"
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
#include "dart/simulation/World.h"
#include "dart/utils/Paths.h"
#include "dart/utils/SkelParser.h"

using namespace std;
using namespace Eigen;

using namespace dart;
using namespace math;
using namespace dynamics;

//==============================================================================
MatrixXd getMassMatrix(dynamics::Skeleton* _skel)
{
  int skelDof = _skel->getNumGenCoords();

  MatrixXd skelM = MatrixXd::Zero(skelDof, skelDof);  // Mass matrix of skeleton
  MatrixXd M;  // Body mass
  MatrixXd I;  // Body inertia
  MatrixXd J;  // Body Jacobian

  for (int i = 0; i < _skel->getNumBodyNodes(); ++i)
  {
    dynamics::BodyNode* body = _skel->getBodyNode(i);

    int dof = body->getNumDependentGenCoords();
    I = body->getInertia();
    J = body->getBodyJacobian();

    EXPECT_EQ(I.rows(), 6);
    EXPECT_EQ(I.cols(), 6);
    EXPECT_EQ(J.rows(), 6);
    EXPECT_EQ(J.cols(), dof);

    M = J.transpose() * I * J;  // (dof x dof) matrix

    for (int j = 0; j < dof; ++j)
    {
      int jIdx = body->getDependentGenCoord(j);

      for (int k = 0; k < dof; ++k)
      {
        int kIdx = body->getDependentGenCoord(k);

        skelM(jIdx, kIdx) += M(j, k);
      }
    }
  }

  return skelM;
}

//==============================================================================
void equationsOfMotionTest(const string& _fileName)
{
  //---------------------------- Settings --------------------------------------
  // Number of random state tests for each skeletons
  int nRandomItr = 1000;

  // Lower and upper bound of configuration for system
  double lb = -1.5 * DART_PI;
  double ub =  1.5 * DART_PI;

  simulation::World* myWorld = NULL;

  //----------------------------- Tests ----------------------------------------
  // Check whether multiplication of mass matrix and its inverse is identity
  // matrix.
  {
    myWorld = utils::SkelParser::readSkelFile(_fileName);
    EXPECT_TRUE(myWorld != NULL);

    for (int j = 0; j < myWorld->getNumSkeletons(); ++j)
    {
      dynamics::Skeleton* skel = myWorld->getSkeleton(j);

      int dof            = skel->getNumGenCoords();
      int nBodyNodes     = skel->getNumBodyNodes();

      if (dof == 0)
      {
        cout << "Skeleton [" << skel->getName() << "] is skipped since it has "
             << "0 DOF." << endl;
        continue;
      }

      for (int k = 0; k < nRandomItr; ++k)
      {
        // Set random states
        VectorXd x = skel->getState();
        for (int l = 0; l < x.size(); ++l)
          x[l] = math::random(lb, ub);
        skel->setState(x);

        //------------------------ Mass Matrix Test ----------------------------
        // Get matrices
        MatrixXd M      = skel->getMassMatrix();
        MatrixXd M2     = getMassMatrix(skel);
        MatrixXd InvM   = skel->getInvMassMatrix();
        MatrixXd InvM2  = M.inverse();
        MatrixXd M_InvM = M * InvM;
        MatrixXd InvM_M = InvM * M;
        MatrixXd I      = MatrixXd::Identity(dof, dof);

        // Check if the number of generalized coordinates and dimension of mass
        // matrix are same.
        EXPECT_EQ(M.rows(), dof);
        EXPECT_EQ(M.cols(), dof);

        // Check mass matrix
        EXPECT_TRUE(equals(M, M2, 1e-6));
        if (!equals(M, M2, 1e-6))
        {
          cout << "M :" << endl << M  << endl << endl;
          cout << "M2:" << endl << M2 << endl << endl;
        }

        // Check mass inverse matrix
        EXPECT_TRUE(equals(InvM, InvM2, 1e-6));
        if (!equals(InvM, InvM2, 1e-6))
        {
          cout << "InvM :" << endl << InvM  << endl << endl;
          cout << "InvM2:" << endl << InvM2 << endl << endl;
        }

        // Check if both of (M * InvM) and (InvM * M) are identity.
        EXPECT_TRUE(equals(M_InvM, I, 1e-6));
        if (!equals(M_InvM, I, 1e-6))
        {
          cout << "M_InvM:" << endl << M_InvM << endl << endl;
        }
        EXPECT_TRUE(equals(InvM_M, I, 1e-6));
        if (!equals(InvM_M, I, 1e-6))
        {
          cout << "InvM_M:" << endl << InvM_M << endl << endl;
        }

        //------- Coriolis Force Vector and Combined Force Vector Tests --------
        // Get C1, Coriolis force vector using recursive method
        VectorXd C = skel->getCoriolisForceVector();
        VectorXd Cg = skel->getCombinedVector();

        // Get C2, Coriolis force vector using inverse dynamics algorithm
        Vector3d oldGravity = skel->getGravity();
        VectorXd oldTau     = skel->getInternalForceVector();
        VectorXd oldDdq     = skel->get_ddq();
        // TODO(JS): Save external forces of body nodes

        skel->clearInternalForceVector();
        skel->clearExternalForceVector();
        skel->set_ddq(VectorXd::Zero(dof));

        EXPECT_TRUE(skel->getInternalForceVector() == VectorXd::Zero(dof));
        EXPECT_TRUE(skel->getExternalForceVector() == VectorXd::Zero(dof));
        EXPECT_TRUE(skel->get_ddq()                == VectorXd::Zero(dof));

        skel->setGravity(Vector3d::Zero());
        EXPECT_TRUE(skel->getGravity() == Vector3d::Zero());
        skel->computeInverseDynamicsLinear(false, false, false, false);
        VectorXd C2 = skel->get_tau();

        skel->setGravity(oldGravity);
        EXPECT_TRUE(skel->getGravity() == oldGravity);
        skel->computeInverseDynamicsLinear(false, false, false, false);
        VectorXd Cg2 = skel->get_tau();

        EXPECT_TRUE(equals(C, C2, 1e-6));
        if (!equals(C, C2, 1e-6))
        {
          cout << "C :" << C.transpose()  << endl;
          cout << "C2:" << C2.transpose() << endl;
        }

        EXPECT_TRUE(equals(Cg, Cg2, 1e-6));
        if (!equals(Cg, Cg2, 1e-6))
        {
          cout << "Cg :" << Cg.transpose()  << endl;
          cout << "Cg2:" << Cg2.transpose() << endl;
        }

        skel->set_tau(oldTau);
        skel->set_ddq(oldDdq);
        // TODO(JS): Restore external forces of body nodes

        //------------------- Combined Force Vector Test -----------------------
        // TODO(JS): Not implemented yet.

        //---------------------- Damping Force Test ----------------------------
        // TODO(JS): Not implemented yet.

        //--------------------- External Force Test ----------------------------
        // TODO(JS): Not implemented yet.
      }
    }

    delete myWorld;
  }
}

//==============================================================================
TEST(EOM, EquationOfMotion)
{
  dtdbg << "single_pendulum.skel" << std::endl;
  equationsOfMotionTest(DART_DATA_PATH"/skel/test/single_pendulum.skel");

  dtdbg << "single_pendulum_euler_joint.skel" << std::endl;
  equationsOfMotionTest(DART_DATA_PATH"/skel/test/single_pendulum_euler_joint.skel");

  dtdbg << "single_pendulum_ball_joint.skel" << std::endl;
  equationsOfMotionTest(DART_DATA_PATH"/skel/test/single_pendulum_ball_joint.skel");

  dtdbg << "double_pendulum.skel" << std::endl;
  equationsOfMotionTest(DART_DATA_PATH"/skel/test/double_pendulum.skel");

//  dtdbg << "double_pendulum_euler_joint.skel" << std::endl;
//  equationsOfMotionTest(DART_DATA_PATH"/skel/test/double_pendulum_euler_joint.skel");

//  dtdbg << "double_pendulum_ball_joint.skel" << std::endl;
//  equationsOfMotionTest(DART_DATA_PATH"/skel/test/double_pendulum_ball_joint.skel");

  dtdbg << "serial_chain_revolute_joint.skel" << std::endl;
  equationsOfMotionTest(DART_DATA_PATH"/skel/test/serial_chain_revolute_joint.skel");

//  dtdbg << "serial_chain_eulerxyz_joint.skel" << std::endl;
//  equationsOfMotionTest(DART_DATA_PATH"/skel/test/serial_chain_eulerxyz_joint.skel");

//  dtdbg << "serial_chain_ball_joint.skel" << std::endl;
//  equationsOfMotionTest(DART_DATA_PATH"/skel/test/serial_chain_ball_joint.skel");

//  dtdbg << "serial_chain_ball_joint_20.skel" << std::endl;
//  equationsOfMotionTest(DART_DATA_PATH"/skel/test/serial_chain_ball_joint_20.skel");

//  dtdbg << "serial_chain_ball_joint_40.skel" << std::endl;
//  equationsOfMotionTest(DART_DATA_PATH"/skel/test/serial_chain_ball_joint_40.skel");

  dtdbg << "simple_tree_structure.skel" << std::endl;
  equationsOfMotionTest(DART_DATA_PATH"/skel/test/simple_tree_structure.skel");

//  dtdbg << "simple_tree_structure_euler_joint.skel" << std::endl;
//  equationsOfMotionTest(DART_DATA_PATH"/skel/test/simple_tree_structure_euler_joint.skel");

//  dtdbg << "simple_tree_structure_ball_joint.skel" << std::endl;
//  equationsOfMotionTest(DART_DATA_PATH"/skel/test/simple_tree_structure_ball_joint.skel");

  dtdbg << "tree_structure.skel" << std::endl;
  equationsOfMotionTest(DART_DATA_PATH"/skel/test/tree_structure.skel");

//  dtdbg << "tree_structure_euler_joint.skel" << std::endl;
//  equationsOfMotionTest(DART_DATA_PATH"/skel/test/tree_structure_euler_joint.skel");

  dtdbg << "tree_structure_ball_joint.skel" << std::endl;
  equationsOfMotionTest(DART_DATA_PATH"/skel/test/tree_structure_ball_joint.skel");

//  dtdbg << "fullbody1.skel" << std::endl;
//  equationsOfMotionTest(DART_DATA_PATH"/skel/fullbody1.skel");
}

//==============================================================================
int main(int argc, char* argv[])
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
