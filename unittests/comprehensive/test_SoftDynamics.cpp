/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "dart/common/Console.hpp"
#include "dart/math/Constants.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/PointMass.hpp"
#include "dart/simulation/World.hpp"
#include "dart/io/SkelParser.hpp"

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
  const std::size_t n1 = A.cols(), m1 = A.rows();
  const std::size_t n2 = B.cols(), m2 = B.rows();
  if (m1 != m2 || n1 != n2)
    return false;

  // Check each index
  for (std::size_t i = 0; i < m1; i++)
  {
    for (std::size_t j = 0; j < n1; j++)
    {
      if (std::isnan(A(i,j)) ^ std::isnan(B(i,j)))
        return false;
      else if (std::abs(A(i,j) - B(i,j)) > tol)
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
  MatrixXd getMassMatrix(dynamics::SkeletonPtr _skel);

  // Get augmented mass matrix of _skel using Jacobians and inertias of
  // each body in _skel.
  MatrixXd getAugMassMatrix(dynamics::SkeletonPtr _skel);

  // TODO(JS): Not implemented yet.
  // Compare velocities computed by recursive method, Jacobian, and finite
  // difference.
  //void compareVelocities(const std::string& _fileName) {}

  // TODO(JS): Not implemented yet.
  // Compare accelerations computed by recursive method, Jacobian, and finite
  // difference.
  //void compareAccelerations(const std::string& _fileName) {}

  // Compare dynamics terms in equations of motion such as mass matrix, mass
  // inverse matrix, Coriolis force vector, gravity force vector, and external
  // force vector.
  void compareEquationsOfMotion(const std::string& _fileName);

protected:
  // Sets up the test fixture.
  void SetUp() override;

  // Skel file list.
  std::vector<std::string> list;
};

//==============================================================================
void SoftDynamicsTest::SetUp()
{
  list.push_back("dart://sample/skel/test/test_drop_box.skel");
}

//==============================================================================
const std::vector<std::string>& SoftDynamicsTest::getList()
{
  return list;
}

//==============================================================================
MatrixXd SoftDynamicsTest::getMassMatrix(dynamics::SkeletonPtr _skel)
{
  int skelDof = _skel->getNumDofs();

  MatrixXd skelM = MatrixXd::Zero(skelDof, skelDof);  // Mass matrix of skeleton
  MatrixXd M;  // Body mass
  Eigen::Matrix6d I;  // Body inertia
  math::Jacobian J;  // Body Jacobian

  for (std::size_t i = 0; i < _skel->getNumBodyNodes(); ++i)
  {
    dynamics::BodyNode* body = _skel->getBodyNode(i);

    int dof = body->getNumDependentGenCoords();
    I = body->getSpatialInertia();
    J = body->getJacobian();

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
MatrixXd SoftDynamicsTest::getAugMassMatrix(dynamics::SkeletonPtr _skel)
{
  int    dof = _skel->getNumDofs();
  double dt  = _skel->getTimeStep();

  MatrixXd M = getMassMatrix(_skel);
  MatrixXd D = MatrixXd::Zero(dof, dof);
  MatrixXd K = MatrixXd::Zero(dof, dof);
  MatrixXd AugM;

  // Compute diagonal matrices of joint damping and joint stiffness
  for (std::size_t i = 0; i < _skel->getNumBodyNodes(); ++i)
  {
    dynamics::BodyNode* body  = _skel->getBodyNode(i);
    dynamics::Joint*    joint = body->getParentJoint();

    EXPECT_TRUE(body  != nullptr);
    EXPECT_TRUE(joint != nullptr);

    int dof = joint->getNumDofs();

    for (int j = 0; j < dof; ++j)
    {
      int idx = joint->getIndexInSkeleton(j);

      D(idx, idx) = joint->getDampingCoefficient(j);
      K(idx, idx) = joint->getSpringStiffness(j);
    }
  }

//  dynamics::SkeletonPtr softSkel
//      = dynamic_cast<dynamics::SkeletonPtr>(_skel);

//  if (softSkel != nullptr)
//  {
//    for (int i = 0; i < softSkel->getNumSoftBodyNodes(); ++i)
//    {
//      dynamics::SoftBodyNode* softBody = softSkel->getSoftBodyNode(i);

//      for (int j = 0; j < softBody->getNumPointMasses(); ++j)
//      {
//        dynamics::PointMass* pm = softBody->getPointMass(j);

//        int dof = 3;

//        for (int k = 0; k < dof; ++k)
//        {
//          int idx = pm->getIndexInSkeleton(k);

//          D(idx, idx) = softBody->getDampingCoefficient();
//          K(idx, idx) = softBody->getVertexSpringStiffness();
//        }
//      }
//    }
//  }

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
  using namespace io;

  //---------------------------- Settings --------------------------------------
  // Number of random state tests for each skeletons
#ifndef NDEBUG  // Debug mode
  std::size_t nRandomItr = 1;
#else
  std::size_t nRandomItr = 1;
#endif

  // Lower and upper bound of configuration for system
  double lb = -1.5 * constantsd::pi();
  double ub =  1.5 * constantsd::pi();

  // Lower and upper bound of joint damping and stiffness
  double lbD =  0.0;
  double ubD = 10.0;
  double lbK =  0.0;
  double ubK = 10.0;

  simulation::WorldPtr myWorld;

  //----------------------------- Tests ----------------------------------------
  // Check whether multiplication of mass matrix and its inverse is identity
  // matrix.
  myWorld = io::SkelParser::readWorld(_fileName);
  EXPECT_TRUE(myWorld != nullptr);

  for (std::size_t i = 0; i < myWorld->getNumSkeletons(); ++i)
  {
    dynamics::SkeletonPtr softSkel = myWorld->getSkeleton(i);

    int dof            = softSkel->getNumDofs();
//    int nBodyNodes     = skel->getNumBodyNodes();
    int nSoftBodyNodes = 0;
    if (softSkel != nullptr)
      nSoftBodyNodes = softSkel->getNumSoftBodyNodes();

    if (dof == 0)
    {
      dtmsg << "Skeleton [" << softSkel->getName() << "] is skipped since it has "
           << "0 DOF." << endl;
      continue;
    }

    for (std::size_t j = 0; j < nRandomItr; ++j)
    {
      // Random joint stiffness and damping coefficient
      for (std::size_t k = 0; k < softSkel->getNumBodyNodes(); ++k)
      {
        BodyNode* body     = softSkel->getBodyNode(k);
        Joint*    joint    = body->getParentJoint();
        int       localDof = joint->getNumDofs();

        for (int l = 0; l < localDof; ++l)
        {
          joint->setDampingCoefficient(l, Random::uniform(lbD,  ubD));
          joint->setSpringStiffness   (l, Random::uniform(lbK,  ubK));

          double lbRP = joint->getPositionLowerLimit(l);
          double ubRP = joint->getPositionUpperLimit(l);
          joint->setRestPosition      (l, Random::uniform(lbRP, ubRP));
        }
      }

      // Set random states
      Skeleton::Configuration x = softSkel->getConfiguration();
      for (auto k = 0u; k < softSkel->getNumDofs(); ++k)
      {
        x.mPositions[k] = Random::uniform(lb, ub);
        x.mVelocities[k] = Random::uniform(lb, ub);
      }
      softSkel->setConfiguration(x);

      //------------------------ Mass Matrix Test ----------------------------
      // Get matrices
      MatrixXd M      = softSkel->getMassMatrix();
      MatrixXd M2     = getMassMatrix(softSkel);
      MatrixXd InvM   = softSkel->getInvMassMatrix();
      MatrixXd M_InvM = M * InvM;
      MatrixXd InvM_M = InvM * M;

      MatrixXd AugM         = softSkel->getAugMassMatrix();
      MatrixXd AugM2        = getAugMassMatrix(softSkel);
      MatrixXd InvAugM      = softSkel->getInvAugMassMatrix();
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
      VectorXd C  = softSkel->getCoriolisForces();
      VectorXd Cg = softSkel->getCoriolisAndGravityForces();

      // Get C2, Coriolis force vector using inverse dynamics algorithm
      Vector3d oldGravity = softSkel->getGravity();
      VectorXd oldTau     = softSkel->getForces();
      VectorXd oldDdq     = softSkel->getAccelerations();
      // TODO(JS): Save external forces of body nodes
      vector<double> oldKv(nSoftBodyNodes, 0.0);
      vector<double> oldKe(nSoftBodyNodes, 0.0);
      vector<double>  oldD(nSoftBodyNodes, 0.0);
      for (int k = 0; k < nSoftBodyNodes; ++k)
      {
        assert(softSkel != nullptr);
        dynamics::SoftBodyNode* sbn = softSkel->getSoftBodyNode(k);
        oldKv[k] = sbn->getVertexSpringStiffness();
        oldKe[k] = sbn->getEdgeSpringStiffness();
        oldD[k]  = sbn->getDampingCoefficient();
      }

      softSkel->resetGeneralizedForces();
      softSkel->clearExternalForces();
      softSkel->setAccelerations(VectorXd::Zero(dof));
      for (int k = 0; k < nSoftBodyNodes; ++k)
      {
        assert(softSkel != nullptr);
        dynamics::SoftBodyNode* sbn = softSkel->getSoftBodyNode(k);
        sbn->setVertexSpringStiffness(0.0);
        sbn->setEdgeSpringStiffness(0.0);
        sbn->setDampingCoefficient(0.0);
      }

      EXPECT_TRUE(softSkel->getForces()         == VectorXd::Zero(dof));
      EXPECT_TRUE(softSkel->getExternalForces() == VectorXd::Zero(dof));
      EXPECT_TRUE(softSkel->getAccelerations()  == VectorXd::Zero(dof));

      softSkel->setGravity(Vector3d::Zero());
      EXPECT_TRUE(softSkel->getGravity() == Vector3d::Zero());
      softSkel->computeInverseDynamics(false, false);
      VectorXd C2 = softSkel->getForces();

      softSkel->setGravity(oldGravity);
      EXPECT_TRUE(softSkel->getGravity() == oldGravity);
      softSkel->computeInverseDynamics(false, false);
      VectorXd Cg2 = softSkel->getForces();

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

      softSkel->setForces(oldTau);
      softSkel->setAccelerations(oldDdq);
      // TODO(JS): Restore external forces of body nodes
    }
  }
}

//==============================================================================
TEST_F(SoftDynamicsTest, compareEquationsOfMotion)
{
  // TODO(JS): Equations of motion for softbody skeleton is not done yet

//  for (int i = 0; i < getList().size(); ++i)
//  {
//#ifndef NDEBUG
//    dtdbg << getList()[i] << std::endl;
//#endif
//    compareEquationsOfMotion(getList()[i]);
//  }
}
