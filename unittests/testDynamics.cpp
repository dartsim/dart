/*
 * Copyright (c) 2011-2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sumit Jain <sumit@cc.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
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

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "TestHelpers.h"

#include "dart/common/Console.h"
#include "dart/math/Geometry.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"
#include "dart/utils/SkelParser.h"
#include "dart/utils/Paths.h"

//==============================================================================
class DynamicsTest : public ::testing::Test
{
public:
  // Get Skel file list to test.
  const std::vector<std::string>& getList();

  // Get mass matrix of _skel using Jacobians and inertias of each body
  // in _skel.
  MatrixXd getMassMatrix(dynamics::Skeleton* _skel);

  // Get augmented mass matrix of _skel using Jacobians and inertias of
  // each body in _skel.
  MatrixXd getAugMassMatrix(dynamics::Skeleton* _skel);

  // Compare velocities computed by recursive method, Jacobian, and finite
  // difference.
  void compareVelocities(const std::string& _fileName);

  // Compare accelerations computed by recursive method, Jacobian, and finite
  // difference.
  void compareAccelerations(const std::string& _fileName);

  // Compare dynamics terms in equations of motion such as mass matrix, mass
  // inverse matrix, Coriolis force vector, gravity force vector, and external
  // force vector.
  void compareEquationsOfMotion(const std::string& _fileName);

  // Test skeleton's COM and its related quantities.
  void centerOfMass(const std::string& _fileName);

protected:
  // Sets up the test fixture.
  virtual void SetUp();

  // Skel file list.
  std::vector<std::string> list;
};

//==============================================================================
void DynamicsTest::SetUp()
{
  list.push_back(DART_DATA_PATH"skel/test/chainwhipa.skel");
  list.push_back(DART_DATA_PATH"skel/test/single_pendulum.skel");
  list.push_back(DART_DATA_PATH"skel/test/single_pendulum_euler_joint.skel");
  list.push_back(DART_DATA_PATH"skel/test/single_pendulum_ball_joint.skel");
  list.push_back(DART_DATA_PATH"skel/test/double_pendulum.skel");
  list.push_back(DART_DATA_PATH"skel/test/double_pendulum_euler_joint.skel");
  list.push_back(DART_DATA_PATH"skel/test/double_pendulum_ball_joint.skel");
  list.push_back(DART_DATA_PATH"skel/test/serial_chain_revolute_joint.skel");
  list.push_back(DART_DATA_PATH"skel/test/serial_chain_eulerxyz_joint.skel");
  list.push_back(DART_DATA_PATH"skel/test/serial_chain_ball_joint.skel");
  list.push_back(DART_DATA_PATH"skel/test/serial_chain_ball_joint_20.skel");
  list.push_back(DART_DATA_PATH"skel/test/serial_chain_ball_joint_40.skel");
  list.push_back(DART_DATA_PATH"skel/test/simple_tree_structure.skel");
  list.push_back(DART_DATA_PATH"skel/test/simple_tree_structure_euler_joint.skel");
  list.push_back(DART_DATA_PATH"skel/test/simple_tree_structure_ball_joint.skel");
  list.push_back(DART_DATA_PATH"skel/test/tree_structure.skel");
  list.push_back(DART_DATA_PATH"skel/test/tree_structure_euler_joint.skel");
  list.push_back(DART_DATA_PATH"skel/test/tree_structure_ball_joint.skel");
  list.push_back(DART_DATA_PATH"skel/fullbody1.skel");
}

//==============================================================================
const std::vector<std::string>& DynamicsTest::getList()
{
  return list;
}

//==============================================================================
MatrixXd DynamicsTest::getMassMatrix(dynamics::Skeleton* _skel)
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
      int jIdx = body->getDependentGenCoordIndex(j);

      for (int k = 0; k < dof; ++k)
      {
        int kIdx = body->getDependentGenCoordIndex(k);

        skelM(jIdx, kIdx) += M(j, k);
      }
    }
  }

  return skelM;
}

//==============================================================================
MatrixXd DynamicsTest::getAugMassMatrix(dynamics::Skeleton* _skel)
{
  int    dof = _skel->getNumGenCoords();
  double dt  = _skel->getTimeStep();

  MatrixXd M = getMassMatrix(_skel);
  MatrixXd D = MatrixXd::Zero(dof, dof);
  MatrixXd K = MatrixXd::Zero(dof, dof);
  MatrixXd AugM;

  // Compute diagonal matrices of joint damping and joint stiffness
  for (int i = 0; i < _skel->getNumBodyNodes(); ++i)
  {
    dynamics::BodyNode* body  = _skel->getBodyNode(i);
    dynamics::Joint*    joint = body->getParentJoint();

    EXPECT_TRUE(body  != NULL);
    EXPECT_TRUE(joint != NULL);

    int dof = joint->getNumGenCoords();

    for (int j = 0; j < dof; ++j)
    {
      int idx = joint->getGenCoord(j)->getSkeletonIndex();

      D(idx, idx) = joint->getDampingCoefficient(j);
      K(idx, idx) = joint->getSpringStiffness(j);
    }
  }

  AugM = M + (dt * D) + (dt * dt * K);

  return AugM;
}

//==============================================================================
void DynamicsTest::compareVelocities(const std::string& _fileName)
{
  using namespace std;
  using namespace Eigen;
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace utils;

  //----------------------------- Settings -------------------------------------
  const double TOLERANCE = 1.0e-6;
#ifndef NDEBUG  // Debug mode
  int nRandomItr = 10;
#else
  int nRandomItr = 1;
#endif
  double qLB  = -0.5 * DART_PI;
  double qUB  =  0.5 * DART_PI;
  double dqLB = -0.5 * DART_PI;
  double dqUB =  0.5 * DART_PI;
  double ddqLB = -0.5 * DART_PI;
  double ddqUB =  0.5 * DART_PI;
  Vector3d gravity(0.0, -9.81, 0.0);

  // load skeleton
  World* world = SkelParser::readSkelFile(_fileName);
  assert(world != NULL);
  world->setGravity(gravity);

  //------------------------------ Tests ---------------------------------------
  for (int i = 0; i < world->getNumSkeletons(); ++i)
  {
    Skeleton* skeleton = world->getSkeleton(i);
    assert(skeleton != NULL);
    int dof = skeleton->getNumGenCoords();

    for (int j = 0; j < nRandomItr; ++j)
    {
      // Generate a random state
      VectorXd q   = VectorXd(dof);
      VectorXd dq  = VectorXd(dof);
      VectorXd ddq = VectorXd(dof);
      for (int k = 0; k < dof; ++k)
      {
        q[k]   = math::random(qLB,   qUB);
        dq[k]  = math::random(dqLB,  dqUB);
        ddq[k] = math::random(ddqLB, ddqUB);
      }
      VectorXd state = VectorXd::Zero(dof * 2);
      state << q, dq;
      skeleton->setState(state);
      skeleton->set_ddq(ddq);
      skeleton->computeInverseDynamicsLinear(true, true, false, false);

      // For each body node
      for (int k = 0; k < skeleton->getNumBodyNodes(); ++k)
      {
        BodyNode* bn = skeleton->getBodyNode(k);

        // Calculation of velocities using recursive method
        Vector6d vBody  = bn->getBodyVelocity();
        Vector6d vWorld = bn->getWorldVelocity();
        Vector6d aBody  = bn->getBodyAcceleration();
        Vector6d aWorld = bn->getWorldAcceleration();

        // Calculation of velocities using Jacobian and dq
        MatrixXd JBody   = bn->getBodyJacobian();
        MatrixXd JWorld  = bn->getWorldJacobian();
        MatrixXd dJBody  = bn->getBodyJacobianTimeDeriv();
        MatrixXd dJWorld = bn->getWorldJacobianTimeDeriv();
        Vector6d vBody2  = Vector6d::Zero();
        Vector6d vWorld2 = Vector6d::Zero();
        Vector6d aBody2  = Vector6d::Zero();
        Vector6d aWorld2 = Vector6d::Zero();
        for (int l = 0; l < bn->getNumDependentGenCoords(); ++l)
        {
          int idx = bn->getDependentGenCoordIndex(l);
          vBody2  += JBody.col(l)  * dq[idx];
          vWorld2 += JWorld.col(l) * dq[idx];
          aBody2  += dJBody.col(l) * dq[idx] + JBody.col(l) * ddq[idx];
          aWorld2 += dJWorld.col(l) * dq[idx] + JWorld.col(l) * ddq[idx];
        }

        // Comparing two velocities
        EXPECT_TRUE(equals(vBody,  vBody2,  TOLERANCE));
        EXPECT_TRUE(equals(vWorld, vWorld2, TOLERANCE));
        EXPECT_TRUE(equals(aBody, aBody2, TOLERANCE));
        EXPECT_TRUE(equals(aWorld, aWorld2, TOLERANCE));

        // Debugging code
        if (!equals(vBody, vBody2, TOLERANCE))
        {
          cout << "vBody : " << vBody.transpose()  << endl;
          cout << "vBody2: " << vBody2.transpose() << endl;
        }
        if (!equals(vWorld, vWorld2, TOLERANCE))
        {
          cout << "vWorld : " << vWorld.transpose()  << endl;
          cout << "vWorld2: " << vWorld2.transpose() << endl;
        }
        if (!equals(aBody, aBody2, TOLERANCE))
        {
          cout << "aBody : "  << aBody.transpose()  << endl;
          cout << "aBody2: "  << aBody2.transpose() << endl;
        }
        if (!equals(aWorld, aWorld2, TOLERANCE))
        {
          cout << "aWorld : " << aWorld.transpose()  << endl;
          cout << "aWorld2: " << aWorld2.transpose() << endl;
        }
      }
    }
  }

  delete world;
}

//==============================================================================
void DynamicsTest::compareAccelerations(const std::string& _fileName)
{
  using namespace std;
  using namespace Eigen;
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace utils;

  //----------------------------- Settings -------------------------------------
  const double TOLERANCE = 1.0e-2;
#ifndef NDEBUG  // Debug mode
  int nRandomItr = 2;
#else
  int nRandomItr = 10;
#endif
  double qLB   = -0.5 * DART_PI;
  double qUB   =  0.5 * DART_PI;
  double dqLB  = -0.5 * DART_PI;
  double dqUB  =  0.5 * DART_PI;
  double ddqLB = -0.5 * DART_PI;
  double ddqUB =  0.5 * DART_PI;
  Vector3d gravity(0.0, -9.81, 0.0);
  double timeStep = 1.0e-6;

  // load skeleton
  World* world = SkelParser::readSkelFile(_fileName);
  assert(world != NULL);
  world->setGravity(gravity);
  world->setTimeStep(timeStep);

  //------------------------------ Tests ---------------------------------------
  for (int i = 0; i < world->getNumSkeletons(); ++i)
  {
    Skeleton* skeleton = world->getSkeleton(i);
    assert(skeleton != NULL);
    int dof = skeleton->getNumGenCoords();

    for (int j = 0; j < nRandomItr; ++j)
    {
      // Generate a random state and ddq
      VectorXd q   = VectorXd(dof);
      VectorXd dq  = VectorXd(dof);
      VectorXd ddq = VectorXd(dof);
      for (int k = 0; k < dof; ++k)
      {
        q[k]   = math::random(qLB,   qUB);
        dq[k]  = math::random(dqLB,  dqUB);
        ddq[k] = math::random(ddqLB, ddqUB);

//        q[k]   = 0.0;
//        dq[k]  = 0.0;
//        ddq[k] = 0.0;
      }
      VectorXd x = VectorXd::Zero(dof * 2);
      x << q, dq;
      skeleton->setState(x);
      skeleton->set_ddq(ddq);

      // Set x(k+1) = x(k) + dt * dx(k)
      VectorXd qNext  = q  + timeStep * dq;
      VectorXd dqNext = dq + timeStep * ddq;
      VectorXd xNext  = VectorXd::Zero(dof * 2);
      xNext << qNext, dqNext;

      // For each body node
      for (int k = 0; k < skeleton->getNumBodyNodes(); ++k)
      {
        BodyNode* bn = skeleton->getBodyNode(k);
        int nDepGenCoord = bn->getNumDependentGenCoords();

        // Calculation of velocities and Jacobian at k-th time step
        skeleton->setState(x);
        skeleton->set_ddq(ddq);
        skeleton->computeInverseDynamicsLinear(true, true, false, false);
        Vector6d vBody1  = bn->getBodyVelocity();
        Vector6d vWorld1 = bn->getWorldVelocity();
        MatrixXd JBody1  = bn->getBodyJacobian();
        MatrixXd JWorld1 = bn->getWorldJacobian();
        Isometry3d T1    = bn->getWorldTransform();

        // Get accelerations and time derivatives of Jacobians at k-th time step
        Vector6d aBody1   = bn->getBodyAcceleration();
        Vector6d aWorld1  = bn->getWorldAcceleration();
        MatrixXd dJBody1  = bn->getBodyJacobianTimeDeriv();
        MatrixXd dJWorld1 = bn->getWorldJacobianTimeDeriv();

        // Calculation of velocities and Jacobian at (k+1)-th time step
        skeleton->setState(xNext);
        skeleton->set_ddq(ddq);
        skeleton->computeInverseDynamicsLinear(true, true, false, false);
        Vector6d vBody2  = bn->getBodyVelocity();
        Vector6d vWorld2 = bn->getWorldVelocity();
        MatrixXd JBody2  = bn->getBodyJacobian();
        MatrixXd JWorld2 = bn->getWorldJacobian();
        Isometry3d T2    = bn->getWorldTransform();

        // Get accelerations and time derivatives of Jacobians at k-th time step
        Vector6d aBody2   = bn->getBodyAcceleration();
        Vector6d aWorld2  = bn->getWorldAcceleration();
        MatrixXd dJBody2  = bn->getBodyJacobianTimeDeriv();
        MatrixXd dJWorld2 = bn->getWorldJacobianTimeDeriv();

        // Calculation of approximated accelerations and time derivatives of
        // Jacobians
        Vector6d aBodyApprox   = (vBody2  - vBody1)  / timeStep;
        Vector6d aWorldApprox  = (vWorld2 - vWorld1) / timeStep;

        // TODO(JS): Finite difference of Jacobian test is not implemented yet.
//        MatrixXd dJBodyApprox  = (JBody2  - JBody1)  / timeStep;
//        MatrixXd dJWorldApprox = (JWorld2 - JWorld1) / timeStep;
//        MatrixXd dJBodyApprox  = MatrixXd::Zero(6, nDepGenCoord);
//        MatrixXd dJWorldApprox = MatrixXd::Zero(6, nDepGenCoord);

//        for (int l = 0; l < nDepGenCoord; ++l)
//        {
//          skeleton->setConfig(q);
//          Jacobian JBody_a = bn->getBodyJacobian();

//          int idx = bn->getDependentGenCoordIndex(l);
//          VectorXd qGrad = q;
//          qGrad[idx] = qNext[idx];
//          skeleton->setConfig(qGrad);
//          Jacobian JBody_b = bn->getBodyJacobian();

//          Jacobian dJBody_dq = (JBody_b - JBody_a) / (qNext[idx] - q[idx]);

//          dJBodyApprox += dJBody_dq * dq[idx];
//        }

        // Comparing two velocities
        EXPECT_TRUE(equals(aBody1,   aBodyApprox,   TOLERANCE));
        EXPECT_TRUE(equals(aBody2,   aBodyApprox,   TOLERANCE));
        EXPECT_TRUE(equals(aWorld1,  aWorldApprox,  TOLERANCE));
        EXPECT_TRUE(equals(aWorld2,  aWorldApprox,  TOLERANCE));
//        EXPECT_TRUE(equals(dJBody1,  dJBodyApprox,  TOLERANCE));
//        EXPECT_TRUE(equals(dJBody2,  dJBodyApprox,  TOLERANCE));
//        EXPECT_TRUE(equals(dJWorld1, dJWorldApprox, TOLERANCE));
//        EXPECT_TRUE(equals(dJWorld2, dJWorldApprox, TOLERANCE));

        // Debugging code
        if (!equals(aBody1, aBodyApprox, TOLERANCE))
        {
          cout << "aBody1     :" << aBody1.transpose()      << endl;
          cout << "aBodyApprox:" << aBodyApprox.transpose() << endl;
        }
        if (!equals(aBody2, aBodyApprox, TOLERANCE))
        {
          cout << "aBody2     :" << aBody2.transpose()      << endl;
          cout << "aBodyApprox:" << aBodyApprox.transpose() << endl;
        }
        if (!equals(aWorld1, aWorldApprox, TOLERANCE))
        {
          cout << "aWorld1     :" << aWorld1.transpose()      << endl;
          cout << "aWorldApprox:" << aWorldApprox.transpose() << endl;
        }
        if (!equals(aWorld2, aWorldApprox, TOLERANCE))
        {
          cout << "aWorld2     :" << aWorld2.transpose()      << endl;
          cout << "aWorldApprox:" << aWorldApprox.transpose() << endl;
        }
//        if (!equals(dJBody1, dJBodyApprox, TOLERANCE))
//        {
//          cout << "Name        :" << bn->getName()        << endl;
//          cout << "dJBody1     :" << endl << dJBody1      << endl;
//          cout << "dJBodyApprox:" << endl << dJBodyApprox << endl;
//        }
//        if (!equals(dJBody2, dJBodyApprox, TOLERANCE))
//        {
//          cout << "dJBody2:"      << endl << dJBody2.transpose()      << endl;
//          cout << "dJBodyApprox:" << endl << dJBodyApprox.transpose() << endl;
//        }
//        if (!equals(dJWorld1, dJWorldApprox, TOLERANCE))
//        {
//          cout << "dJWorld1     :" << endl << dJWorld1      << endl;
//          cout << "dJWorldApprox:" << endl << dJWorldApprox << endl;
//        }
//        if (!equals(dJWorld2, dJWorldApprox, TOLERANCE))
//        {
//          cout << "dJWorld2     :" << endl << dJWorld2      << endl;
//          cout << "dJWorldApprox:" << endl << dJWorldApprox << endl;
//        }
      }
    }
  }

  delete world;
}

//==============================================================================
void DynamicsTest::compareEquationsOfMotion(const std::string& _fileName)
{
  using namespace std;
  using namespace Eigen;
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace utils;

  //---------------------------- Settings --------------------------------------
  // Number of random state tests for each skeletons
#ifndef NDEBUG  // Debug mode
  int nRandomItr = 5;
#else
  int nRandomItr = 100;
#endif

  // Lower and upper bound of configuration for system
  double lb = -1.5 * DART_PI;
  double ub =  1.5 * DART_PI;

  // Lower and upper bound of joint damping and stiffness
  double lbD =  0.0;
  double ubD = 10.0;
  double lbK =  0.0;
  double ubK = 10.0;

  simulation::World* myWorld = NULL;

  //----------------------------- Tests ----------------------------------------
  // Check whether multiplication of mass matrix and its inverse is identity
  // matrix.
  myWorld = utils::SkelParser::readSkelFile(_fileName);
  EXPECT_TRUE(myWorld != NULL);

  for (int i = 0; i < myWorld->getNumSkeletons(); ++i)
  {
    dynamics::Skeleton* skel = myWorld->getSkeleton(i);

    int dof            = skel->getNumGenCoords();
//    int nBodyNodes     = skel->getNumBodyNodes();

    if (dof == 0)
    {
      dtmsg << "Skeleton [" << skel->getName() << "] is skipped since it has "
            << "0 DOF." << endl;
      continue;
    }

    for (int j = 0; j < nRandomItr; ++j)
    {
      // Random joint stiffness and damping coefficient
      for (int k = 0; k < skel->getNumBodyNodes(); ++k)
      {
        BodyNode* body     = skel->getBodyNode(k);
        Joint*    joint    = body->getParentJoint();
        int       localDof = joint->getNumGenCoords();

        for (int l = 0; l < localDof; ++l)
        {
          joint->setDampingCoefficient(l, random(lbD,  ubD));
          joint->setSpringStiffness   (l, random(lbK,  ubK));

          double lbRP = -1e+1;
          double ubRP = +1e+1;
          joint->setRestPosition      (l, random(lbRP, ubRP));
        }
      }

      // Set random states
      VectorXd x = skel->getState();
      for (int k = 0; k < x.size(); ++k)
        x[k] = random(lb, ub);
      skel->setState(x);

      //------------------------ Mass Matrix Test ----------------------------
      // Get matrices
      MatrixXd M      = skel->getMassMatrix();
      MatrixXd M2     = getMassMatrix(skel);
      MatrixXd InvM   = skel->getInvMassMatrix();
      MatrixXd M_InvM = M * InvM;
      MatrixXd InvM_M = InvM * M;

      MatrixXd AugM         = skel->getAugMassMatrix();
      MatrixXd AugM2        = getAugMassMatrix(skel);
      MatrixXd InvAugM      = skel->getInvAugMassMatrix();
      MatrixXd AugM_InvAugM = AugM * InvAugM;
      MatrixXd InvAugM_AugM = InvAugM * AugM;

      MatrixXd I        = MatrixXd::Identity(dof, dof);

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

      // Check augmented mass matrix
      EXPECT_TRUE(equals(AugM, AugM2, 1e-6));
      if (!equals(AugM, AugM2, 1e-6))
      {
        cout << "AugM :" << endl << AugM  << endl << endl;
        cout << "AugM2:" << endl << AugM2 << endl << endl;
      }

      // Check if both of (M * InvM) and (InvM * M) are identity.
      EXPECT_TRUE(equals(M_InvM, I, 1e-6));
      if (!equals(M_InvM, I, 1e-6))
      {
        cout << "InvM  :" << endl << InvM << endl << endl;
      }
      EXPECT_TRUE(equals(InvM_M, I, 1e-6));
      if (!equals(InvM_M, I, 1e-6))
      {
        cout << "InvM_M:" << endl << InvM_M << endl << endl;
      }

      // Check if both of (M * InvM) and (InvM * M) are identity.
      EXPECT_TRUE(equals(AugM_InvAugM, I, 1e-6));
      if (!equals(AugM_InvAugM, I, 1e-6))
      {
        cout << "AugM_InvAugM  :" << endl << AugM_InvAugM << endl << endl;
      }
      EXPECT_TRUE(equals(InvAugM_AugM, I, 1e-6));
      if (!equals(InvAugM_AugM, I, 1e-6))
      {
        cout << "InvAugM_AugM:" << endl << InvAugM_AugM << endl << endl;
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

//==============================================================================
void DynamicsTest::centerOfMass(const std::string& _fileName)
{
  using namespace std;
  using namespace Eigen;
  using namespace dart;
  using namespace math;
  using namespace dynamics;
  using namespace simulation;
  using namespace utils;

  //---------------------------- Settings --------------------------------------
  // Number of random state tests for each skeletons
#ifndef NDEBUG  // Debug mode
  int nRandomItr = 5;
#else
  int nRandomItr = 1;
#endif

  // Lower and upper bound of configuration for system
  double lb = -1.5 * DART_PI;
  double ub =  1.5 * DART_PI;

  // Lower and upper bound of joint damping and stiffness
  double lbD =  0.0;
  double ubD = 10.0;
  double lbK =  0.0;
  double ubK = 10.0;

  simulation::World* myWorld = NULL;

  //----------------------------- Tests ----------------------------------------
  // Check whether multiplication of mass matrix and its inverse is identity
  // matrix.
  myWorld = utils::SkelParser::readSkelFile(_fileName);
  EXPECT_TRUE(myWorld != NULL);

  for (int i = 0; i < myWorld->getNumSkeletons(); ++i)
  {
    dynamics::Skeleton* skel = myWorld->getSkeleton(i);

    int dof            = skel->getNumGenCoords();
//    int nBodyNodes     = skel->getNumBodyNodes();

    if (dof == 0)
    {
      dtmsg << "Skeleton [" << skel->getName() << "] is skipped since it has "
            << "0 DOF." << endl;
      continue;
    }

    for (int j = 0; j < nRandomItr; ++j)
    {
      // Random joint stiffness and damping coefficient
      for (int k = 0; k < skel->getNumBodyNodes(); ++k)
      {
        BodyNode* body     = skel->getBodyNode(k);
        Joint*    joint    = body->getParentJoint();
        int       localDof = joint->getNumGenCoords();

        for (int l = 0; l < localDof; ++l)
        {
          joint->setDampingCoefficient(l, random(lbD,  ubD));
          joint->setSpringStiffness   (l, random(lbK,  ubK));

          double lbRP = -1e+1;
          double ubRP = +1e+1;
          joint->setRestPosition      (l, random(lbRP, ubRP));
        }
      }

      // Set random states
      VectorXd x = skel->getState();
      for (int k = 0; k < x.size(); ++k)
        x[k] = random(lb, ub);
      skel->setState(x);

      VectorXd tau = skel->get_tau();
      for (int k = 0; k < tau.size(); ++k)
        tau[k] = random(lb, ub);
      skel->set_tau(tau);

      skel->computeForwardDynamics();

      VectorXd q  = skel->get_q();
      VectorXd dq = skel->get_dq();
      VectorXd ddq = skel->get_ddq();

      VectorXd com   = skel->getWorldCOM();
      VectorXd dcom  = skel->getWorldCOMVelocity();
      VectorXd ddcom = skel->getWorldCOMAcceleration();

      MatrixXd comJ  = skel->getWorldCOMJacobian();
      MatrixXd comdJ = skel->getWorldCOMJacobianTimeDeriv();

      VectorXd dcom2  = comJ * dq;
      VectorXd ddcom2 = comdJ * dq + comJ * ddq;

      EXPECT_TRUE(equals(dcom, dcom2, 1e-6));
      if (!equals(dcom, dcom2, 1e-6))
      {
        cout << "dcom :" << dcom.transpose()  << endl;
        cout << "dcom2:" << dcom2.transpose() << endl;
      }

      EXPECT_TRUE(equals(ddcom, ddcom2, 1e-6));
      if (!equals(ddcom, ddcom2, 1e-6))
      {
        cout << "ddcom :" << ddcom.transpose()  << endl;
        cout << "ddcom2:" << ddcom2.transpose() << endl;
      }
    }
  }

  delete myWorld;
}

////==============================================================================
//TEST_F(DynamicsTest, compareVelocities)
//{
//  for (int i = 0; i < getList().size(); ++i)
//  {
//#ifndef NDEBUG
//    dtdbg << getList()[i] << std::endl;
//#endif
//    compareVelocities(getList()[i]);
//  }
//}

////==============================================================================
//TEST_F(DynamicsTest, compareAccelerations)
//{
//  for (int i = 0; i < getList().size(); ++i)
//  {
//#ifndef NDEBUG
//    dtdbg << getList()[i] << std::endl;
//#endif
//    compareAccelerations(getList()[i]);
//  }
//}

////==============================================================================
//TEST_F(DynamicsTest, compareEquationsOfMotion)
//{
//  for (int i = 0; i < getList().size(); ++i)
//  {
//    ////////////////////////////////////////////////////////////////////////////
//    // TODO(JS): Following five skel files, which contain euler joints couldn't
//    //           pass EQUATIONS_OF_MOTION, are disabled.
//    std::string skelFileName = getList()[i];
//    if (skelFileName == DART_DATA_PATH"skel/test/chainwhipa.skel"
//        || skelFileName == DART_DATA_PATH"skel/test/serial_chain_eulerxyz_joint.skel"
//        || skelFileName == DART_DATA_PATH"skel/test/simple_tree_structure_euler_joint.skel"
//        || skelFileName == DART_DATA_PATH"skel/test/tree_structure_euler_joint.skel"
//        || skelFileName == DART_DATA_PATH"skel/fullbody1.skel")
//    {
//        continue;
//    }
//    ////////////////////////////////////////////////////////////////////////////

//#ifndef NDEBUG
//    dtdbg << getList()[i] << std::endl;
//#endif
//    compareEquationsOfMotion(getList()[i]);
//  }
//}

//==============================================================================
TEST_F(DynamicsTest, testCenterOfMass)
{
  for (int i = 0; i < getList().size(); ++i)
  {
#ifndef NDEBUG
    dtdbg << getList()[i] << std::endl;
#endif
    centerOfMass(getList()[i]);
  }
}

//==============================================================================
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

