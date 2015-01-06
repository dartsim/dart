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

using namespace Eigen;
using namespace dart;

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

  //
  void testConstraintImpulse(const std::string& _fileName);

  // Test impulse based dynamics
  void testImpulseBasedDynamics(const std::string& _fileName);

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
  int skelDof = _skel->getNumDofs();

  MatrixXd skelM = MatrixXd::Zero(skelDof, skelDof);  // Mass matrix of skeleton
  MatrixXd M;  // Body mass
  MatrixXd I;  // Body inertia
  MatrixXd J;  // Body Jacobian

  for (size_t i = 0; i < _skel->getNumBodyNodes(); ++i)
  {
    dynamics::BodyNode* body = _skel->getBodyNode(i);

    int dof = body->getNumDependentGenCoords();
    I = body->getSpatialInertia();
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
  int    dof = _skel->getNumDofs();
  double dt  = _skel->getTimeStep();

  MatrixXd M = getMassMatrix(_skel);
  MatrixXd D = MatrixXd::Zero(dof, dof);
  MatrixXd K = MatrixXd::Zero(dof, dof);
  MatrixXd AugM;

  // Compute diagonal matrices of joint damping and joint stiffness
  for (size_t i = 0; i < _skel->getNumBodyNodes(); ++i)
  {
    dynamics::BodyNode* body  = _skel->getBodyNode(i);
    dynamics::Joint*    joint = body->getParentJoint();

    EXPECT_TRUE(body  != NULL);
    EXPECT_TRUE(joint != NULL);

    int dof = joint->getNumDofs();

    for (int j = 0; j < dof; ++j)
    {
      int idx = joint->getIndexInSkeleton(j);

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
  World* world = SkelParser::readWorld(_fileName);
  assert(world != NULL);
  world->setGravity(gravity);

  //------------------------------ Tests ---------------------------------------
  for (size_t i = 0; i < world->getNumSkeletons(); ++i)
  {
    Skeleton* skeleton = world->getSkeleton(i);
    assert(skeleton != NULL);
    int dof = skeleton->getNumDofs();

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
      skeleton->setPositions(q);
      skeleton->setVelocities(dq);
      skeleton->setAccelerations(ddq);
      skeleton->computeForwardKinematics(true, true, true);
      skeleton->computeInverseDynamics(false, false);

      // For each body node
      for (size_t k = 0; k < skeleton->getNumBodyNodes(); ++k)
      {
        BodyNode* bn = skeleton->getBodyNode(k);

        // Calculation of velocities using recursive method
        Vector3d BodyLinVel = bn->getBodyLinearVelocity();
        Vector3d BodyAngVel = bn->getBodyAngularVelocity();
        Vector3d WorldLinVel = bn->getWorldLinearVelocity();
        Vector3d WorldAngVel = bn->getWorldAngularVelocity();
        Vector3d BodyLinAcc = bn->getBodyLinearAcceleration();
        Vector3d BodyAngAcc = bn->getBodyAngularAcceleration();
        Vector3d WorldLinAcc = bn->getWorldLinearAcceleration();
        Vector3d WorldAngAcc = bn->getWorldAngularAcceleration();

        // Calculation of velocities using Jacobian and dq
        MatrixXd BodyLinJac = bn->getBodyLinearJacobian();
        MatrixXd BodyAngJac = bn->getBodyAngularJacobian();
        MatrixXd WorldLinJac = bn->getWorldLinearJacobian();
        MatrixXd WorldAngJac = bn->getWorldAngularJacobian();
        MatrixXd BodyLinJacDeriv = bn->getBodyLinearJacobianDeriv();
        MatrixXd BodyAngJacDeriv = bn->getBodyAngularJacobianDeriv();
        MatrixXd WorldLinJacDeriv = bn->getWorldLinearJacobianDeriv();
        MatrixXd WorldAngJacDeriv = bn->getWorldAngularJacobianDeriv();
        Vector3d BodyLinVel2 = Vector3d::Zero();
        Vector3d BodyAngVel2 = Vector3d::Zero();
        Vector3d WorldLinVel2 = Vector3d::Zero();
        Vector3d WorldAngVel2 = Vector3d::Zero();
        Vector3d BodyLinAcc2 = Vector3d::Zero();
        Vector3d BodyAngAcc2 = Vector3d::Zero();
        Vector3d WorldLinAcc2 = Vector3d::Zero();
        Vector3d WorldAngAcc2 = Vector3d::Zero();

        for (size_t l = 0; l < bn->getNumDependentGenCoords(); ++l)
        {
          int idx = bn->getDependentGenCoordIndex(l);

          BodyLinVel2 += BodyLinJac.col(l) * dq[idx];
          BodyAngVel2 += BodyAngJac.col(l) * dq[idx];
          WorldLinVel2 += WorldLinJac.col(l) * dq[idx];
          WorldAngVel2 += WorldAngJac.col(l) * dq[idx];
          BodyLinAcc2 += BodyLinJacDeriv.col(l) * dq[idx]
                         + BodyLinJac.col(l) * ddq[idx];
          BodyAngAcc2 += BodyAngJacDeriv.col(l) * dq[idx]
                         + BodyAngJac.col(l) * ddq[idx];
          WorldLinAcc2 += WorldLinJacDeriv.col(l) * dq[idx]
                         + WorldLinJac.col(l) * ddq[idx];
          WorldAngAcc2 += WorldAngJacDeriv.col(l) * dq[idx]
                         + WorldAngJac.col(l) * ddq[idx];
        }

        // Comparing two velocities
        EXPECT_TRUE(equals(BodyLinVel,  BodyLinVel2,  TOLERANCE));
        EXPECT_TRUE(equals(BodyAngVel,  BodyAngVel2,  TOLERANCE));
        EXPECT_TRUE(equals(WorldLinVel, WorldLinVel2, TOLERANCE));
        EXPECT_TRUE(equals(WorldAngVel, WorldAngVel2, TOLERANCE));
        EXPECT_TRUE(equals(BodyLinAcc, BodyLinAcc2, TOLERANCE));
        EXPECT_TRUE(equals(BodyAngAcc, BodyAngAcc2, TOLERANCE));
        EXPECT_TRUE(equals(WorldLinAcc, WorldLinAcc2, TOLERANCE));
        EXPECT_TRUE(equals(WorldAngAcc, WorldAngAcc2, TOLERANCE));

        // Debugging code
        if (!equals(BodyLinVel, BodyLinVel2, TOLERANCE))
        {
          cout << "BodyLinVel : " << BodyLinVel.transpose()  << endl;
          cout << "BodyLinVel2: " << BodyLinVel2.transpose() << endl;
        }
        if (!equals(BodyAngVel, BodyAngVel2, TOLERANCE))
        {
          cout << "vBody : " << BodyAngVel.transpose()  << endl;
          cout << "BodyAngVel2: " << BodyAngVel2.transpose() << endl;
        }
        if (!equals(WorldLinVel, WorldLinVel2, TOLERANCE))
        {
          cout << "WorldLinVel : " << WorldLinVel.transpose()  << endl;
          cout << "WorldLinVel2: " << WorldLinVel2.transpose() << endl;
        }
        if (!equals(WorldAngVel, WorldAngVel2, TOLERANCE))
        {
          cout << "WorldAngVel : " << WorldAngVel.transpose()  << endl;
          cout << "WorldAngVel2: " << WorldAngVel2.transpose() << endl;
        }
        if (!equals(BodyLinAcc, BodyLinAcc2, TOLERANCE))
        {
          cout << "BodyLinAcc : "  << BodyLinAcc.transpose()  << endl;
          cout << "BodyLinAcc2: "  << BodyLinAcc2.transpose() << endl;
        }
        if (!equals(BodyAngAcc, BodyAngAcc, TOLERANCE))
        {
          cout << "BodyAngAcc : "  << BodyAngAcc.transpose()  << endl;
          cout << "BodyAngAcc2: "  << BodyAngAcc2.transpose() << endl;
        }
        if (!equals(WorldLinAcc, WorldLinAcc, TOLERANCE))
        {
          cout << "WorldLinAcc : " << WorldLinAcc.transpose()  << endl;
          cout << "WorldLinAcc2: " << WorldLinAcc2.transpose() << endl;
        }
        if (!equals(WorldAngAcc, WorldAngAcc2, TOLERANCE))
        {
          cout << "WorldAngAcc : " << WorldAngAcc.transpose()  << endl;
          cout << "WorldAngAcc2: " << WorldAngAcc2.transpose() << endl;
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
  World* world = SkelParser::readWorld(_fileName);
  assert(world != NULL);
  world->setGravity(gravity);
  world->setTimeStep(timeStep);

  //------------------------------ Tests ---------------------------------------
  for (size_t i = 0; i < world->getNumSkeletons(); ++i)
  {
    Skeleton* skeleton = world->getSkeleton(i);
    assert(skeleton != NULL);
    int dof = skeleton->getNumDofs();

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

      VectorXd qNext  =  q +  dq * timeStep;
      VectorXd dqNext = dq + ddq * timeStep;

      // For each body node
      for (size_t k = 0; k < skeleton->getNumBodyNodes(); ++k)
      {
        BodyNode* bn = skeleton->getBodyNode(k);
        // int nDepGenCoord = bn->getNumDependentGenCoords();

        // Calculation of velocities and Jacobian at k-th time step
        skeleton->setPositions(q);
        skeleton->setVelocities(dq);
        skeleton->setAccelerations(ddq);
        skeleton->computeForwardKinematics(true, true, true);

        Vector3d BodyLinVel1 = bn->getBodyLinearVelocity();
        Vector3d BodyAngVel1 = bn->getBodyAngularVelocity();
        Vector3d WorldLinVel1 = bn->getWorldLinearVelocity();
        Vector3d WorldAngVel1 = bn->getWorldAngularVelocity();
        MatrixXd BodyLinJac1 = bn->getBodyLinearJacobian();
        MatrixXd BodyAngJac1 = bn->getBodyAngularJacobian();
        MatrixXd WorldLinJac1 = bn->getWorldLinearJacobian();
        MatrixXd WorldAngJac1 = bn->getWorldAngularJacobian();
        // Isometry3d T1    = bn->getTransform();

        // Get accelerations and time derivatives of Jacobians at k-th time step
        Vector3d BodyLinAcc1 = bn->getBodyLinearAcceleration();
        Vector3d BodyAngAcc1 = bn->getBodyAngularAcceleration();
        Vector3d WorldLinAcc1 = bn->getWorldLinearAcceleration();
        Vector3d WorldAngAcc1 = bn->getWorldAngularAcceleration();
        MatrixXd BodyLinJacDeriv1 = bn->getBodyLinearJacobianDeriv();
        MatrixXd BodyAngJacDeriv1 = bn->getBodyAngularJacobianDeriv();
        MatrixXd WorldLinJacDeriv1 = bn->getWorldLinearJacobianDeriv();
        MatrixXd WorldAngJacDeriv1 = bn->getWorldAngularJacobianDeriv();

        // Calculation of velocities and Jacobian at (k+1)-th time step
        skeleton->setPositions(qNext);
        skeleton->setVelocities(dqNext);
        skeleton->setAccelerations(ddq);
        skeleton->computeForwardKinematics(true, true, true);

        Vector3d BodyLinVel2 = bn->getBodyLinearVelocity();
        Vector3d BodyAngVel2 = bn->getBodyAngularVelocity();
        Vector3d WorldLinVel2 = bn->getWorldLinearVelocity();
        Vector3d WorldAngVel2 = bn->getWorldAngularVelocity();
        MatrixXd BodyLinJac2 = bn->getBodyLinearJacobian();
        MatrixXd BodyAngJac2 = bn->getBodyAngularJacobian();
        MatrixXd WorldLinJac2 = bn->getWorldLinearJacobian();
        MatrixXd WorldAngJac2 = bn->getWorldAngularJacobian();
        // Isometry3d T2    = bn->getTransform();

        // Get accelerations and time derivatives of Jacobians at k-th time step
        Vector3d BodyLinAcc2 = bn->getBodyLinearAcceleration();
        Vector3d BodyAngAcc2 = bn->getBodyAngularAcceleration();
        Vector3d WorldLinAcc2 = bn->getWorldLinearAcceleration();
        Vector3d WorldAngAcc2 = bn->getWorldAngularAcceleration();
        MatrixXd BodyLinJacDeriv2 = bn->getBodyLinearJacobianDeriv();
        MatrixXd BodyAngJacDeriv2 = bn->getBodyAngularJacobianDeriv();
        MatrixXd WorldLinJacDeriv2 = bn->getWorldLinearJacobianDeriv();
        MatrixXd WorldAngJacDeriv2 = bn->getWorldAngularJacobianDeriv();

        // Calculation of approximated accelerations and time derivatives of
        // Jacobians
        Vector3d BodyLinAccApprox   = (BodyLinVel2  - BodyLinVel1)  / timeStep;
        Vector3d BodyAngAccApprox   = (BodyAngVel2  - BodyAngVel1)  / timeStep;
        Vector3d WorldLinAccApprox  = (WorldLinVel2 - WorldLinVel1) / timeStep;
        Vector3d WorldAngAccApprox  = (WorldAngVel2 - WorldAngVel1) / timeStep;

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
        EXPECT_TRUE(equals(BodyLinAcc1,   BodyLinAccApprox,   TOLERANCE));
        EXPECT_TRUE(equals(BodyAngAcc1,   BodyAngAccApprox,   TOLERANCE));
        EXPECT_TRUE(equals(BodyLinAcc2,   BodyLinAccApprox,   TOLERANCE));
        EXPECT_TRUE(equals(BodyAngAcc2,   BodyAngAccApprox,   TOLERANCE));
        EXPECT_TRUE(equals(WorldLinAcc1,  WorldLinAccApprox,  TOLERANCE));
        EXPECT_TRUE(equals(WorldAngAcc1,  WorldAngAccApprox,  TOLERANCE));
        EXPECT_TRUE(equals(WorldLinAcc2,  WorldLinAccApprox,  TOLERANCE));
        EXPECT_TRUE(equals(WorldAngAcc2,  WorldAngAccApprox,  TOLERANCE));
//        EXPECT_TRUE(equals(dJBody1,  dJBodyApprox,  TOLERANCE));
//        EXPECT_TRUE(equals(dJBody2,  dJBodyApprox,  TOLERANCE));
//        EXPECT_TRUE(equals(dJWorld1, dJWorldApprox, TOLERANCE));
//        EXPECT_TRUE(equals(dJWorld2, dJWorldApprox, TOLERANCE));

        // Debugging code
        if (!equals(BodyLinAcc1, BodyLinAccApprox, TOLERANCE))
        {
          cout << "BodyLinAcc1     :" << BodyLinAcc1.transpose()      << endl;
          cout << "BodyLinAccApprox:" << BodyLinAccApprox.transpose() << endl;
        }
        if (!equals(BodyAngAcc1, BodyAngAccApprox, TOLERANCE))
        {
          cout << "BodyAngAcc1     :" << BodyAngAcc1.transpose()      << endl;
          cout << "BodyAngAccApprox:" << BodyAngAccApprox.transpose() << endl;
        }
        if (!equals(BodyLinAcc2, BodyLinAccApprox, TOLERANCE))
        {
          cout << "BodyLinAcc2     :" << BodyLinAcc2.transpose()      << endl;
          cout << "BodyLinAccApprox:" << BodyLinAccApprox.transpose() << endl;
        }
        if (!equals(BodyAngAcc2, BodyAngAccApprox, TOLERANCE))
        {
          cout << "BodyAngAcc2     :" << BodyAngAcc2.transpose()      << endl;
          cout << "BodyAngAccApprox:" << BodyAngAccApprox.transpose() << endl;
        }
        if (!equals(WorldLinAcc1, WorldLinAccApprox, TOLERANCE))
        {
          cout << "WorldLinAcc1     :" << WorldLinAcc1.transpose()      << endl;
          cout << "WorldLinAccApprox:" << WorldLinAccApprox.transpose() << endl;
        }
        if (!equals(WorldAngAcc1, WorldAngAccApprox, TOLERANCE))
        {
          cout << "WorldAngAcc1     :" << WorldAngAcc1.transpose()      << endl;
          cout << "WorldAngAccApprox:" << WorldAngAccApprox.transpose() << endl;
        }
        if (!equals(WorldLinAcc2, WorldLinAccApprox, TOLERANCE))
        {
          cout << "WorldLinAcc2     :" << WorldLinAcc2.transpose()      << endl;
          cout << "WorldLinAccApprox:" << WorldLinAccApprox.transpose() << endl;
        }
        if (!equals(WorldAngAcc2, WorldAngAccApprox, TOLERANCE))
        {
          cout << "WorldAngAcc2     :" << WorldAngAcc2.transpose()      << endl;
          cout << "WorldAngAccApprox:" << WorldAngAccApprox.transpose() << endl;
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
  size_t nRandomItr = 5;
#else
  size_t nRandomItr = 100;
#endif

  // Lower and upper bound of configuration for system
  double lb = -1.0 * DART_PI;
  double ub =  1.0 * DART_PI;

  // Lower and upper bound of joint damping and stiffness
  double lbD =  0.0;
  double ubD = 10.0;
  double lbK =  0.0;
  double ubK = 10.0;

  simulation::World* myWorld = NULL;

  //----------------------------- Tests ----------------------------------------
  // Check whether multiplication of mass matrix and its inverse is identity
  // matrix.
  myWorld = utils::SkelParser::readWorld(_fileName);
  EXPECT_TRUE(myWorld != NULL);

  for (size_t i = 0; i < myWorld->getNumSkeletons(); ++i)
  {
    dynamics::Skeleton* skel = myWorld->getSkeleton(i);

    size_t dof = skel->getNumDofs();
//    int nBodyNodes = skel->getNumBodyNodes();

    if (dof == 0)
    {
      dtmsg << "Skeleton [" << skel->getName() << "] is skipped since it has "
            << "0 DOF." << endl;
      continue;
    }

    for (size_t j = 0; j < nRandomItr; ++j)
    {
      // Random joint stiffness and damping coefficient
      for (size_t k = 0; k < skel->getNumBodyNodes(); ++k)
      {
        BodyNode* body     = skel->getBodyNode(k);
        Joint*    joint    = body->getParentJoint();
        size_t    localDof = joint->getNumDofs();

        for (size_t l = 0; l < localDof; ++l)
        {
          joint->setDampingCoefficient(l, random(lbD,  ubD));
          joint->setSpringStiffness   (l, random(lbK,  ubK));

          double lbRP = joint->getPositionLowerLimit(l);
          double ubRP = joint->getPositionUpperLimit(l);
          if (lbRP < -DART_PI)
            lbRP = -DART_PI;
          if (ubRP > DART_PI)
            ubRP = DART_PI;
          joint->setRestPosition      (l, random(lbRP, ubRP));
        }
      }

      // Set random states
      VectorXd x = skel->getState();
      for (int k = 0; k < x.size(); ++k)
        x[k] = random(lb, ub);
      skel->setState(x);
      skel->computeForwardKinematics(true, true, true);

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
      VectorXd C = skel->getCoriolisForces();
      VectorXd Cg = skel->getCoriolisAndGravityForces();

      // Get C2, Coriolis force vector using inverse dynamics algorithm
      Vector3d oldGravity = skel->getGravity();
      VectorXd oldTau     = skel->getForces();
      VectorXd oldDdq     = skel->getAccelerations();
      // TODO(JS): Save external forces of body nodes

      skel->resetForces();
      skel->clearExternalForces();
      skel->setAccelerations(VectorXd::Zero(dof));

      EXPECT_TRUE(skel->getForces() == VectorXd::Zero(dof));
      EXPECT_TRUE(skel->getExternalForces() == VectorXd::Zero(dof));
      EXPECT_TRUE(skel->getAccelerations() == VectorXd::Zero(dof));

      skel->setGravity(Vector3d::Zero());
      EXPECT_TRUE(skel->getGravity() == Vector3d::Zero());
      skel->computeInverseDynamics(false, false);
      VectorXd C2 = skel->getForces();

      skel->setGravity(oldGravity);
      EXPECT_TRUE(skel->getGravity() == oldGravity);
      skel->computeInverseDynamics(false, false);
      VectorXd Cg2 = skel->getForces();

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

      skel->setForces(oldTau);
      skel->setAccelerations(oldDdq);
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
  size_t nRandomItr = 10;
#else
  size_t nRandomItr = 100;
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
  myWorld = utils::SkelParser::readWorld(_fileName);
  EXPECT_TRUE(myWorld != NULL);

  for (size_t i = 0; i < myWorld->getNumSkeletons(); ++i)
  {
    dynamics::Skeleton* skel = myWorld->getSkeleton(i);

    int dof            = skel->getNumDofs();
//    int nBodyNodes     = skel->getNumBodyNodes();

    if (dof == 0)
    {
      dtmsg << "Skeleton [" << skel->getName() << "] is skipped since it has "
            << "0 DOF." << endl;
      continue;
    }

    for (size_t j = 0; j < nRandomItr; ++j)
    {
      // Random joint stiffness and damping coefficient
      for (size_t k = 0; k < skel->getNumBodyNodes(); ++k)
      {
        BodyNode* body     = skel->getBodyNode(k);
        Joint*    joint    = body->getParentJoint();
        int       localDof = joint->getNumDofs();

        for (int l = 0; l < localDof; ++l)
        {
          joint->setDampingCoefficient(l, random(lbD,  ubD));
          joint->setSpringStiffness   (l, random(lbK,  ubK));

          double lbRP = joint->getPositionLowerLimit(l);
          double ubRP = joint->getPositionUpperLimit(l);
          if (lbRP < -DART_PI)
            lbRP = -DART_PI;
          if (ubRP > DART_PI)
            ubRP = DART_PI;
          joint->setRestPosition      (l, random(lbRP, ubRP));
        }
      }

      // Set random states
      VectorXd x = skel->getState();
      for (int k = 0; k < x.size(); ++k)
        x[k] = random(lb, ub);
      skel->setState(x);
      skel->computeForwardKinematics(true, true, true);

      VectorXd tau = skel->getForces();
      for (int k = 0; k < tau.size(); ++k)
        tau[k] = random(lb, ub);
      skel->setForces(tau);

      skel->computeForwardDynamics();

      VectorXd q  = skel->getPositions();
      VectorXd dq = skel->getVelocities();
      VectorXd ddq = skel->getAccelerations();

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

//==============================================================================
void DynamicsTest::testConstraintImpulse(const std::string& _fileName)
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
  size_t nRandomItr = 1;
#else
  size_t nRandomItr = 1;
#endif

  // Lower and upper bound of configuration for system
//  double lb = -1.5 * DART_PI;
//  double ub =  1.5 * DART_PI;

  simulation::World* myWorld = NULL;

  //----------------------------- Tests ----------------------------------------
  // Check whether multiplication of mass matrix and its inverse is identity
  // matrix.
  myWorld = utils::SkelParser::readWorld(_fileName);
  EXPECT_TRUE(myWorld != NULL);

  for (size_t i = 0; i < myWorld->getNumSkeletons(); ++i)
  {
    dynamics::Skeleton* skel = myWorld->getSkeleton(i);

    size_t dof            = skel->getNumDofs();
//    int nBodyNodes     = skel->getNumBodyNodes();

    if (dof == 0 || !skel->isMobile())
    {
      dtdbg << "Skeleton [" << skel->getName() << "] is skipped since it has "
            << "0 DOF or is immobile." << endl;
      continue;
    }

    for (size_t j = 0; j < nRandomItr; ++j)
    {
      // Set random configurations
      for (size_t k = 0; k < skel->getNumBodyNodes(); ++k)
      {
        BodyNode* body     = skel->getBodyNode(k);
        Joint*    joint    = body->getParentJoint();
        int       localDof = joint->getNumDofs();

        for (int l = 0; l < localDof; ++l)
        {
          double lbRP = joint->getPositionLowerLimit(l);
          double ubRP = joint->getPositionUpperLimit(l);
          if (lbRP < -DART_PI)
            lbRP = -DART_PI;
          if (ubRP > DART_PI)
            ubRP = DART_PI;
          joint->setPosition(l, random(lbRP, ubRP));
        }

        // Set constraint impulse on each body
        skel->clearConstraintImpulses();
        Eigen::Vector6d impulseOnBody = Eigen::Vector6d::Random();
        body->setConstraintImpulse(impulseOnBody);

        // Get constraint force vector
        Eigen::VectorXd constraintVector1 = skel->getConstraintForces();

        // Get constraint force vector by using Jacobian of skeleon
        Eigen::MatrixXd bodyJacobian = body->getBodyJacobian();
        Eigen::VectorXd constraintVector2 = bodyJacobian.transpose()
                                            * impulseOnBody
                                            / skel->getTimeStep();

        size_t index = 0;
        for (size_t l = 0; l < dof; ++l)
        {
          if (constraintVector1(l) == 0.0)
            continue;

          EXPECT_NEAR(constraintVector1(l), constraintVector2(index), 1e-6);
          index++;
        }
        assert(static_cast<size_t>(bodyJacobian.cols()) == index);
      }
    }
  }

  delete myWorld;
}

//==============================================================================
void DynamicsTest::testImpulseBasedDynamics(const std::string& _fileName)
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
  size_t nRandomItr = 1;
#else
  size_t nRandomItr = 100;
#endif

  // Lower and upper bound of configuration for system
  double lb = -1.5 * DART_PI;
  double ub =  1.5 * DART_PI;

  simulation::World* myWorld = NULL;

  //----------------------------- Tests ----------------------------------------
  // Check whether multiplication of mass matrix and its inverse is identity
  // matrix.
  myWorld = utils::SkelParser::readWorld(_fileName);
  EXPECT_TRUE(myWorld != NULL);

  for (size_t i = 0; i < myWorld->getNumSkeletons(); ++i)
  {
    dynamics::Skeleton* skel = myWorld->getSkeleton(i);

    int dof            = skel->getNumDofs();
//    int nBodyNodes     = skel->getNumBodyNodes();

    if (dof == 0 || !skel->isMobile())
    {
      dtdbg << "Skeleton [" << skel->getName() << "] is skipped since it has "
            << "0 DOF or is immobile." << endl;
      continue;
    }

    for (size_t j = 0; j < nRandomItr; ++j)
    {
      // Set random configurations
      for (size_t k = 0; k < skel->getNumBodyNodes(); ++k)
      {
        BodyNode* body     = skel->getBodyNode(k);
        Joint*    joint    = body->getParentJoint();
        int       localDof = joint->getNumDofs();

        for (int l = 0; l < localDof; ++l)
        {
          double lbRP = joint->getPositionLowerLimit(l);
          double ubRP = joint->getPositionUpperLimit(l);
          if (lbRP < -DART_PI)
            lbRP = -DART_PI;
          if (ubRP > DART_PI)
            ubRP = DART_PI;
          joint->setPosition(l, random(lbRP, ubRP));
        }
      }
//      skel->setPositions(VectorXd::Zero(dof));

      // TODO(JS): Just clear what should be
      skel->clearExternalForces();
      skel->clearConstraintImpulses();

      // Set random impulses
      VectorXd impulses = VectorXd::Zero(dof);
      for (int k = 0; k < impulses.size(); ++k)
        impulses[k] = random(lb, ub);
      skel->setJointConstraintImpulses(impulses);

      // Compute impulse-based forward dynamics
      skel->computeImpulseForwardDynamics();

      // Compare resultant velocity change and invM * impulses
      VectorXd deltaVel1 = skel->getVelocityChanges();
      MatrixXd invM = skel->getInvMassMatrix();
      VectorXd deltaVel2 = invM * impulses;

      EXPECT_TRUE(equals(deltaVel1, deltaVel2, 1e-6));
      if (!equals(deltaVel1, deltaVel2, 1e-6))
      {
        cout << "deltaVel1: " << deltaVel1.transpose()  << endl;
        cout << "deltaVel2: " << deltaVel2.transpose() << endl;
      }
    }
  }

  delete myWorld;
}

//==============================================================================
TEST_F(DynamicsTest, compareVelocities)
{
  for (size_t i = 0; i < getList().size(); ++i)
  {
#ifndef NDEBUG
    dtdbg << getList()[i] << std::endl;
#endif
    compareVelocities(getList()[i]);
  }
}

//==============================================================================
TEST_F(DynamicsTest, compareAccelerations)
{
  for (size_t i = 0; i < getList().size(); ++i)
  {
#ifndef NDEBUG
    dtdbg << getList()[i] << std::endl;
#endif
    compareAccelerations(getList()[i]);
  }
}

//==============================================================================
TEST_F(DynamicsTest, compareEquationsOfMotion)
{
  for (size_t i = 0; i < getList().size(); ++i)
  {
    ////////////////////////////////////////////////////////////////////////////
    // TODO(JS): Following skel files, which contain euler joints couldn't
    //           pass EQUATIONS_OF_MOTION, are disabled.
    std::string skelFileName = getList()[i];
    if (skelFileName == DART_DATA_PATH"skel/test/double_pendulum_euler_joint.skel"
        || skelFileName == DART_DATA_PATH"skel/test/chainwhipa.skel"
        || skelFileName == DART_DATA_PATH"skel/test/serial_chain_eulerxyz_joint.skel"
        || skelFileName == DART_DATA_PATH"skel/test/simple_tree_structure_euler_joint.skel"
        || skelFileName == DART_DATA_PATH"skel/test/tree_structure_euler_joint.skel"
        || skelFileName == DART_DATA_PATH"skel/fullbody1.skel")
    {
        continue;
    }
    ////////////////////////////////////////////////////////////////////////////

#ifndef NDEBUG
    dtdbg << getList()[i] << std::endl;
#endif
    compareEquationsOfMotion(getList()[i]);
  }
}

//==============================================================================
TEST_F(DynamicsTest, testCenterOfMass)
{
  for (size_t i = 0; i < getList().size(); ++i)
  {
#ifndef NDEBUG
    dtdbg << getList()[i] << std::endl;
#endif
    centerOfMass(getList()[i]);
  }
}

//==============================================================================
TEST_F(DynamicsTest, testConstraintImpulse)
{
  for (size_t i = 0; i < getList().size(); ++i)
  {
#ifndef NDEBUG
    dtdbg << getList()[i] << std::endl;
#endif
    testConstraintImpulse(getList()[i]);
  }
}

//==============================================================================
TEST_F(DynamicsTest, testImpulseBasedDynamics)
{
  for (size_t i = 0; i < getList().size(); ++i)
  {
#ifndef NDEBUG
    dtdbg << getList()[i] << std::endl;
#endif
    testImpulseBasedDynamics(getList()[i]);
  }
}

//==============================================================================
TEST_F(DynamicsTest, HybridDynamics)
{
  const double tol       = 1e-6;
  const size_t numFrames = 1e+3;
  const double timeStep  = 1e-3;  // 10 secs

  // Load world and skeleton
  World* world = utils::SkelParser::readWorld(
                   DART_DATA_PATH"/skel/test/hybrid_dynamics_test.skel");
  world->setTimeStep(timeStep);
  EXPECT_TRUE(world != NULL);
  EXPECT_NEAR(world->getTimeStep(), timeStep, tol);

  Skeleton* skel = world->getSkeleton("skeleton 1");
  EXPECT_TRUE(skel != NULL);
  EXPECT_NEAR(skel->getTimeStep(), timeStep, tol);

  const size_t numDofs = skel->getNumDofs();

  // Zero initial states
  Eigen::VectorXd q0  = Eigen::VectorXd::Zero(numDofs);
  Eigen::VectorXd dq0 = Eigen::VectorXd::Zero(numDofs);

  // Initialize the skeleton with the zero initial states
  skel->setPositions(q0);
  skel->setVelocities(dq0);
  skel->computeForwardKinematics(true, true, true);
  EXPECT_TRUE(equals(skel->getPositions(), q0));
  EXPECT_TRUE(equals(skel->getVelocities(), dq0));

  // Make sure all the joint actuator types
  EXPECT_EQ(skel->getJoint(0)->getActuatorType(), Joint::TORQUE);
  EXPECT_EQ(skel->getJoint(1)->getActuatorType(), Joint::ACCELERATION);
  EXPECT_EQ(skel->getJoint(2)->getActuatorType(), Joint::VELOCITY);
  EXPECT_EQ(skel->getJoint(3)->getActuatorType(), Joint::ACCELERATION);
  EXPECT_EQ(skel->getJoint(4)->getActuatorType(), Joint::VELOCITY);

  // Prepare command for each joint types per simulation steps
  Eigen::MatrixXd command = Eigen::MatrixXd::Zero(numFrames, numDofs);
  Eigen::VectorXd amp = Eigen::VectorXd::Zero(numDofs);
  for (size_t i = 0; i < numDofs; ++i)
    amp[i] = math::random(-2.0, 2.0);
  for (size_t i = 0; i < numFrames; ++i)
  {
    for (size_t j = 0; j < numDofs; ++j)
      command(i,j) = amp[j] * std::sin(i * timeStep);
  }

  // Record joint forces for joint[1~4]
  Eigen::MatrixXd forces  = Eigen::MatrixXd::Zero(numFrames, numDofs);
  for (size_t i = 0; i < numFrames; ++i)
  {
    skel->setCommands(command.row(i));

    world->step(false);

    forces.row(i) = skel->getForces();

    EXPECT_NEAR(command(i,0), skel->getForce(0), tol);
    EXPECT_NEAR(command(i,0), forces(i,0), tol);
    EXPECT_NEAR(command(i,1), skel->getAcceleration(1), tol);
    EXPECT_NEAR(command(i,2), skel->getVelocity(2), tol);
    EXPECT_NEAR(command(i,3), skel->getAcceleration(3), tol);
    EXPECT_NEAR(command(i,4), skel->getVelocity(4), tol);
  }

  // Restore the skeleton to the initial state
  skel->setPositions(q0);
  skel->setVelocities(dq0);
  skel->computeForwardKinematics(true, true, true);
  EXPECT_TRUE(equals(skel->getPositions(), q0));
  EXPECT_TRUE(equals(skel->getVelocities(), dq0));

  // Change all the actuator types to torque
  skel->getJoint(0)->setActuatorType(Joint::TORQUE);
  skel->getJoint(1)->setActuatorType(Joint::TORQUE);
  skel->getJoint(2)->setActuatorType(Joint::TORQUE);
  skel->getJoint(3)->setActuatorType(Joint::TORQUE);
  skel->getJoint(4)->setActuatorType(Joint::TORQUE);
  EXPECT_EQ(skel->getJoint(0)->getActuatorType(), Joint::TORQUE);
  EXPECT_EQ(skel->getJoint(1)->getActuatorType(), Joint::TORQUE);
  EXPECT_EQ(skel->getJoint(2)->getActuatorType(), Joint::TORQUE);
  EXPECT_EQ(skel->getJoint(3)->getActuatorType(), Joint::TORQUE);
  EXPECT_EQ(skel->getJoint(4)->getActuatorType(), Joint::TORQUE);

  // Test if the skeleton moves as the command with the joint forces
  Eigen::MatrixXd output = Eigen::MatrixXd::Zero(numFrames, numDofs);
  for (size_t i = 0; i < numFrames; ++i)
  {
    skel->setCommands(forces.row(i));

    world->step(false);

    output(i,0) = skel->getJoint(0)->getForce(0);
    output(i,1) = skel->getJoint(1)->getAcceleration(0);
    output(i,2) = skel->getJoint(2)->getVelocity(0);
    output(i,3) = skel->getJoint(3)->getAcceleration(0);
    output(i,4) = skel->getJoint(4)->getVelocity(0);

    EXPECT_NEAR(command(i,0), output(i,0), tol);
    EXPECT_NEAR(command(i,1), output(i,1), tol);
    EXPECT_NEAR(command(i,2), output(i,2), tol);
    EXPECT_NEAR(command(i,3), output(i,3), tol);
    EXPECT_NEAR(command(i,4), output(i,4), tol);
  }
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
