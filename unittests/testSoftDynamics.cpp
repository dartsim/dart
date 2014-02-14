/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
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

#include <vector>
#include <string>

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <boost/math/special_functions/fpclassify.hpp>

#include <dart/common/Console.h>
#include <dart/math/Helpers.h>
#include <dart/dynamics/Joint.h>

#include "dart/dynamics/SoftSkeleton.h"
#include "dart/dynamics/SoftBodyNode.h"
#include "dart/dynamics/PointMass.h"
#include "dart/simulation/SoftWorld.h"
#include "dart/utils/Paths.h"
#include "dart/utils/SoftParser.h"

using namespace std;
using namespace Eigen;
using namespace dart;

//==============================================================================
/// Returns true if the two matrices are equal within the given bound
template <class MATRIX>
bool equals(const DenseBase<MATRIX>& A, const DenseBase<MATRIX>& B,
            double tol = 1e-5)
{
  // Get the matrix sizes and sanity check the call
  const size_t n1 = A.cols(), m1 = A.rows();
  const size_t n2 = B.cols(), m2 = B.rows();
  if (m1 != m2 || n1 != n2)
    return false;

  // Check each index
  for (size_t i = 0; i < m1; i++)
  {
    for (size_t j = 0; j < n1; j++)
    {
      if (boost::math::isnan(A(i,j)) ^ boost::math::isnan(B(i,j)))
        return false;
      else if (fabs(A(i,j) - B(i,j)) > tol)
        return false;
    }
  }

  // If no problems, the two matrices are equal
  return true;
}

//==============================================================================
class SoftDynamicsTest : public ::testing::Test
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

  // TODO(JS): Not implemented yet.
  // Compare velocities computed by recursive method, Jacobian, and finite
  // difference.
  void compareVelocities(const std::string& _fileName) {}

  // TODO(JS): Not implemented yet.
  // Compare accelerations computed by recursive method, Jacobian, and finite
  // difference.
  void compareAccelerations(const std::string& _fileName) {}

  // Compare dynamics terms in equations of motion such as mass matrix, mass
  // inverse matrix, Coriolis force vector, gravity force vector, and external
  // force vector.
  void compareEquationsOfMotion(const std::string& _fileName);

protected:
  // Sets up the test fixture.
  virtual void SetUp();

  // Skel file list.
  std::vector<std::string> list;
};

//==============================================================================
void SoftDynamicsTest::SetUp()
{
  list.push_back(DART_DATA_PATH"skel/test/test_drop_box.skel");
}

//==============================================================================
const std::vector<std::string>& SoftDynamicsTest::getList()
{
  return list;
}

//==============================================================================
MatrixXd SoftDynamicsTest::getMassMatrix(dynamics::Skeleton* _skel)
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

  dynamics::SoftSkeleton* softSkel
      = dynamic_cast<dynamics::SoftSkeleton*>(_skel);

  if (softSkel == NULL)
    return skelM;

  for (int i = 0; i < softSkel->getNumSoftBodyNodes(); ++i)
  {
    dynamics::SoftBodyNode* softBody = softSkel->getSoftBodyNode(i);

    for (int j = 0; j < softBody->getNumPointMasses(); ++j)
    {
      dynamics::PointMass* pm = softBody->getPointMass(j);

      int dof = pm->getNumDependentGenCoords();

      double mass = pm->getMass();
      Matrix3d I  = mass * Matrix3d::Identity();
      J = pm->getBodyJacobian();

      EXPECT_EQ(J.rows(), 3);
      EXPECT_EQ(J.cols(), dof);

      M = J.transpose() * I * J;  // (dof x dof) matrix

      for (int k = 0; k < dof; ++k)
      {
        int kIdx = pm->getDependentGenCoord(k);

        for (int l = 0; l < dof; ++l)
        {
          int lIdx = pm->getDependentGenCoord(l);

          skelM(kIdx, lIdx) += M(k, l);
        }
      }
    }
  }

  return skelM;
}

//==============================================================================
MatrixXd SoftDynamicsTest::getAugMassMatrix(dynamics::Skeleton* _skel)
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

  dynamics::SoftSkeleton* softSkel
      = dynamic_cast<dynamics::SoftSkeleton*>(_skel);

  if (softSkel != NULL)
  {
    for (int i = 0; i < softSkel->getNumSoftBodyNodes(); ++i)
    {
      dynamics::SoftBodyNode* softBody = softSkel->getSoftBodyNode(i);

      for (int j = 0; j < softBody->getNumPointMasses(); ++j)
      {
        dynamics::PointMass* pm = softBody->getPointMass(j);

        int dof = 3;

        for (int k = 0; k < dof; ++k)
        {
          int idx = pm->getGenCoord(k)->getSkeletonIndex();

          D(idx, idx) = softBody->getDampingCoefficient();
          K(idx, idx) = softBody->getVertexSpringStiffness();
        }
      }
    }
  }

  AugM = M + (dt * D) + (dt * dt * K);

  return AugM;
}

//==============================================================================
void SoftDynamicsTest::compareEquationsOfMotion(const std::string& _fileName)
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
  int nRandomItr = 10;
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

  simulation::SoftWorld* myWorld = NULL;

  //----------------------------- Tests ----------------------------------------
  // Check whether multiplication of mass matrix and its inverse is identity
  // matrix.
  myWorld = utils::SoftSkelParser::readSoftFile(_fileName);
  EXPECT_TRUE(myWorld != NULL);

  for (int i = 0; i < myWorld->getNumSkeletons(); ++i)
  {
    dynamics::Skeleton* skel = myWorld->getSkeleton(i);
    dynamics::SoftSkeleton* softSkel
        = dynamic_cast<dynamics::SoftSkeleton*>(skel);

    int dof            = skel->getNumGenCoords();
//    int nBodyNodes     = skel->getNumBodyNodes();
    int nSoftBodyNodes = 0;
    if (softSkel != NULL)
      nSoftBodyNodes = softSkel->getNumSoftBodyNodes();

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

          double lbRP = joint->getGenCoord(l)->get_qMin();
          double ubRP = joint->getGenCoord(l)->get_qMax();
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
        cout << "InvAugM  :" << endl << InvAugM << endl << endl;
        cout << "InvAugM2  :" << endl << AugM.inverse() << endl << endl;
        cout << "AugM_InvAugM  :" << endl << AugM_InvAugM << endl << endl;
      }
      EXPECT_TRUE(equals(InvAugM_AugM, I, 1e-6));
      if (!equals(InvAugM_AugM, I, 1e-6))
      {
        cout << "InvAugM_AugM:" << endl << InvAugM_AugM << endl << endl;
      }

      //------- Coriolis Force Vector and Combined Force Vector Tests --------
      // Get C1, Coriolis force vector using recursive method
      VectorXd C  = skel->getCoriolisForceVector();
      VectorXd Cg = skel->getCombinedVector();

      // Get C2, Coriolis force vector using inverse dynamics algorithm
      Vector3d oldGravity = skel->getGravity();
      VectorXd oldTau     = skel->getInternalForceVector();
      VectorXd oldDdq     = skel->get_ddq();
      // TODO(JS): Save external forces of body nodes
      vector<double> oldKv(nSoftBodyNodes, 0.0);
      vector<double> oldKe(nSoftBodyNodes, 0.0);
      vector<double>  oldD(nSoftBodyNodes, 0.0);
      for (int k = 0; k < nSoftBodyNodes; ++k)
      {
        assert(softSkel != NULL);
        dynamics::SoftBodyNode* sbn = softSkel->getSoftBodyNode(k);
        oldKv[k] = sbn->getVertexSpringStiffness();
        oldKe[k] = sbn->getEdgeSpringStiffness();
        oldD[k]  = sbn->getDampingCoefficient();
      }

      skel->clearInternalForceVector();
      skel->clearExternalForceVector();
      skel->set_ddq(VectorXd::Zero(dof));
      for (int k = 0; k < nSoftBodyNodes; ++k)
      {
        assert(softSkel != NULL);
        dynamics::SoftBodyNode* sbn = softSkel->getSoftBodyNode(k);
        sbn->setVertexSpringStiffness(0.0);
        sbn->setEdgeSpringStiffness(0.0);
        sbn->setDampingCoefficient(0.0);
      }

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
    }
  }

  delete myWorld;
}

//==============================================================================
TEST_F(SoftDynamicsTest, compareEquationsOfMotion)
{
  for (int i = 0; i < getList().size(); ++i)
  {
#ifndef NDEBUG
    dtdbg << getList()[i] << std::endl;
#endif
    compareEquationsOfMotion(getList()[i]);
  }
}

//==============================================================================
int main(int argc, char* argv[])
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
