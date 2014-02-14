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

#include "dart/constraint/SoftConstraintDynamics.h"

#include <vector>

#include <dart/dynamics/Skeleton.h>
#include <dart/dynamics/Joint.h>

#include "dart/dynamics/SoftBodyNode.h"
#include "dart/dynamics/PointMass.h"
#include "dart/collision/fcl_mesh/SoftFCLMeshCollisionDetector.h"

namespace dart {
namespace constraint {

SoftConstraintDynamics::SoftConstraintDynamics(
    const std::vector<dynamics::Skeleton*>& _skeletons,
    double _dt,
    double _mu,
    int _d,
    bool _useODE,
    collision::CollisionDetector* _collisionDetector)
  : ConstraintDynamics(_skeletons, _dt, _mu, _d, _useODE, _collisionDetector)
{
}

SoftConstraintDynamics::~SoftConstraintDynamics()
{
}

void SoftConstraintDynamics::applySolutionODE()
{
  Eigen::VectorXd contactForces(Eigen::VectorXd::Zero(getTotalNumDofs()));
  Eigen::VectorXd jointLimitForces(Eigen::VectorXd::Zero(getTotalNumDofs()));
  int nContacts = getNumContacts();

  if (nContacts > 0)
  {
    Eigen::VectorXd f_n = mX.head(nContacts);
    Eigen::VectorXd f_d = mX.segment(nContacts, nContacts * 2);
    contactForces.noalias() = mN * f_n;
    contactForces.noalias() += mB * f_d;
    for (int i = 0; i < nContacts; i++)
    {
      collision::Contact& cnt = mCollisionDetector->getContact(i);
      cnt.force.noalias()
          = getTangentBasisMatrixODE(cnt.point, cnt.normal)
            * f_d.segment(i * 2, 2);
      cnt.force.noalias() += cnt.normal * f_n[i];

      // Add contact force to body nodes or point masses
      Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
      Eigen::Vector6d F = Eigen::Vector6d::Zero();

      dynamics::BodyNode* bn1 = cnt.collisionNode1->getBodyNode();
      dynamics::SoftBodyNode* soft1
          = dynamic_cast<dynamics::SoftBodyNode*>(bn1);

      if (soft1 != NULL
          && static_cast<collision::SoftCollisionInfo*>(cnt.userData)->isSoft1)
      {
        collision::SoftCollisionInfo* sci
            = static_cast<collision::SoftCollisionInfo*>(cnt.userData);
        assert(sci != NULL);
        assert(sci->pm1 != NULL);
        sci->pm1->addExtForce(cnt.force);
      }
      else
      {
        T.translation() = bn1->getWorldTransform().inverse() * cnt.point;
        F.tail<3>()     = bn1->getWorldTransform().linear().transpose()
                          * cnt.force;
        bn1->addContactForce(math::dAdInvT(T, F));
      }

      dynamics::BodyNode* bn2 = cnt.collisionNode2->getBodyNode();
      dynamics::SoftBodyNode* soft2
          = dynamic_cast<dynamics::SoftBodyNode*>(bn2);

      if (soft2 != NULL
          && static_cast<collision::SoftCollisionInfo*>(cnt.userData)->isSoft2)
      {
        collision::SoftCollisionInfo* sci
            = static_cast<collision::SoftCollisionInfo*>(cnt.userData);
        assert(sci != NULL);
        assert(sci->pm2 != NULL);
        sci->pm2->addExtForce(-cnt.force);
      }
      else
      {
        T.translation() = bn2->getWorldTransform().inverse() * cnt.point;
        F.tail<3>()     = bn2->getWorldTransform().linear().transpose()
                          * (-cnt.force);
        bn2->addContactForce(math::dAdInvT(T, F));
      }
    }
  }
  for (int i = 0; i < mLimitingDofIndex.size(); i++)
  {
    // hitting upper bound
    if (mLimitingDofIndex[i] > 0)
      jointLimitForces[mLimitingDofIndex[i] - 1] = -mX[nContacts * 3 + i];
    else
      jointLimitForces[abs(mLimitingDofIndex[i]) - 1] = mX[nContacts * 3 + i];
  }

  Eigen::VectorXd lambda = Eigen::VectorXd::Zero(mGInv.rows());
  for (int i = 0; i < mSkeletons.size(); i++)
  {
    if (!mSkeletons[i]->isMobile() || mSkeletons[i]->getNumGenCoords() == 0)
      continue;
    mContactForces[i] = contactForces.segment(mIndices[i],
                                              mSkeletons[i]->getNumGenCoords());

    mTotalConstrForces[i] =
        mContactForces[i]
        + jointLimitForces.segment(mIndices[i],
                                   mSkeletons[i]->getNumGenCoords());

    if (mConstraints.size() > 0)
    {
      Eigen::VectorXd tempVec =
          mGInv * (
            mTauHat - mJMInv[i]
            * (contactForces.segment(mIndices[i],
                                     mSkeletons[i]->getNumGenCoords())
               + jointLimitForces.segment(mIndices[i],
                                          mSkeletons[i]->getNumGenCoords())));
      mTotalConstrForces[i] += mJ[i].transpose() * tempVec;
      lambda += tempVec;
    }
  }

  int count = 0;
  for (int i = 0; i < mConstraints.size(); i++)
  {
    mConstraints[i]->setLagrangeMultipliers(
          lambda.segment(count, mConstraints[i]->getNumRows()));
    count += mConstraints[i]->getNumRows();
  }
}

void SoftConstraintDynamics::updateNBMatricesODE()
{
  mN = Eigen::MatrixXd::Zero(getTotalNumDofs(), getNumContacts());
  mB = Eigen::MatrixXd::Zero(getTotalNumDofs(), getNumContacts() * 2);
  for (int i = 0; i < getNumContacts(); i++)
  {
    collision::Contact& cnt = mCollisionDetector->getContact(i);
    Eigen::Vector3d p = cnt.point;
    int skelID1 = mBodyIndexToSkelIndex[cnt.collisionNode1->getIndex()];
    int skelID2 = mBodyIndexToSkelIndex[cnt.collisionNode2->getIndex()];

    Eigen::Vector3d N21 = cnt.normal;
    Eigen::Vector3d N12 = -cnt.normal;
    Eigen::MatrixXd B21 = getTangentBasisMatrixODE(p, N21);
    Eigen::MatrixXd B12 = -B21;

    if (mSkeletons[skelID1]->isMobile()
        && mSkeletons[skelID1]->getNumGenCoords() > 0)
    {
      int index1 = mIndices[skelID1];
      int NDOF1 =
          cnt.collisionNode1->getBodyNode()->getSkeleton()->getNumGenCoords();

      dynamics::BodyNode* body1 = cnt.collisionNode1->getBodyNode();
      dynamics::SoftBodyNode* soft1
          = dynamic_cast<dynamics::SoftBodyNode*>(body1);
      Eigen::MatrixXd J21t;

      if (soft1 != NULL
          && static_cast<collision::SoftCollisionInfo*>(cnt.userData)->isSoft1)
      {
        collision::SoftCollisionInfo* sci
            = static_cast<collision::SoftCollisionInfo*>(cnt.userData);
        assert(sci != NULL);
        assert(sci->pm1 != NULL);
        J21t = getJacobian(sci->pm1);
      }
      else
      {
        J21t = ConstraintDynamics::getJacobian(
                 cnt.collisionNode1->getBodyNode(), p);
      }

      mN.block(index1, i, NDOF1, 1).noalias() = J21t * N21;
      mB.block(index1, i * 2, NDOF1, 2).noalias() = J21t * B21;
    }

    if (mSkeletons[skelID2]->isMobile()
        && mSkeletons[skelID2]->getNumGenCoords() > 0)
    {
      int index2 = mIndices[skelID2];
      int NDOF2 =
          cnt.collisionNode2->getBodyNode()->getSkeleton()->getNumGenCoords();

      dynamics::BodyNode* body2 = cnt.collisionNode2->getBodyNode();
      dynamics::SoftBodyNode* soft2
          = dynamic_cast<dynamics::SoftBodyNode*>(body2);
      Eigen::MatrixXd J12t;

      if (soft2 != NULL
          && static_cast<collision::SoftCollisionInfo*>(cnt.userData)->isSoft2)
      {
        collision::SoftCollisionInfo* sci
            = static_cast<collision::SoftCollisionInfo*>(cnt.userData);
        assert(sci != NULL);
        assert(sci->pm2 != NULL);
        J12t = getJacobian(sci->pm2);
      }
      else
      {
        J12t = ConstraintDynamics::getJacobian(
                 cnt.collisionNode2->getBodyNode(),  p);
      }

      mN.block(index2, i, NDOF2, 1).noalias() += J12t * N12;
      mB.block(index2, i * 2, NDOF2, 2).noalias() += J12t * B12;
    }
  }
}

Eigen::MatrixXd SoftConstraintDynamics::getJacobian(
    dynamics::PointMass* pointMass)
{
  dynamics::Skeleton* skel = pointMass->getParentSoftBodyNode()->getSkeleton();

  Eigen::MatrixXd Jt     = Eigen::MatrixXd::Zero(skel->getNumGenCoords(), 3);
  Eigen::MatrixXd JtBody = pointMass->getWorldJacobian().transpose();

  for (int i = 0; i < pointMass->getNumDependentGenCoords(); ++i)
  {
    int genCoordIdx = pointMass->getDependentGenCoord(i);
    Jt.row(genCoordIdx) = JtBody.row(i);
  }

  return Jt;
}

}  // namespace constraint
}  // namespace dart
