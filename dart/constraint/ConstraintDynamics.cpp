/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>
 *            Jeongseok Lee <jslee02@gmail.com>
 * Date:
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

#include "dart/constraint/ConstraintDynamics.h"

#include <vector>

#include "dart/common/Timer.h"
#include "dart/math/Helpers.h"
#include "dart/lcpsolver/LCPSolver.h"
#include "dart/collision/fcl/FCLCollisionDetector.h"
#include "dart/collision/dart/DARTCollisionDetector.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/GenCoord.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/Skeleton.h"

#define CONSTRAIN_DYNAMICS_EPSILON 1e-6

namespace dart {
namespace constraint {

ConstraintDynamics::ConstraintDynamics(
    const std::vector<dynamics::Skeleton*>& _skeletons,
    double _dt,
    double _mu,
    int _d,
    bool _useODE,
    collision::CollisionDetector* _collisionDetector)
  : mSkeletons(_skeletons),
    mDt(_dt),
    mMu(_mu),
    mNumDir(_d),
    mCollisionDetector(_collisionDetector),
    mUseODELCPSolver(_useODE)
{
  assert(_collisionDetector != NULL && "Invalid collision detector.");
  initialize();
}

ConstraintDynamics::~ConstraintDynamics()
{
  delete mCollisionDetector;
}

void ConstraintDynamics::computeConstraintForces()
{
  //            static Timer t1("t1");

  if (getTotalNumDofs() == 0)
    return;
  mCollisionDetector->clearAllContacts();
  mCollisionDetector->detectCollision(true, true);

  //            t1.startTimer();
  mLimitingDofIndex.clear();

  for (int i = 0; i < mSkeletons.size(); i++)
  {
    if (!mSkeletons[i]->isMobile() || mSkeletons[i]->getNumGenCoords() == 0)
      continue;

    for (int j = 0; j < mSkeletons[i]->getNumBodyNodes(); j++)
    {
      dynamics::Joint* joint = mSkeletons[i]->getJoint(j);
      if (!joint->isPositionLimited())
        continue;
      for (int k = 0; k < mSkeletons[i]->getJoint(j)->getNumGenCoords(); k++)
      {
        dynamics::GenCoord* genCoord = joint->getGenCoord(k);
        double val = genCoord->get_q();
        double ub  = genCoord->get_qMax();
        double lb  = genCoord->get_qMin();

        if (val >= ub)
        {
          mLimitingDofIndex.push_back(mIndices[i]
                                      + genCoord->getSkeletonIndex() + 1);
//          std::cout << "Skeleton " << i << " Dof " << j
//                    << " hits upper bound" << std::endl;
        }
        if (val <= lb)
        {
          mLimitingDofIndex.push_back(-(mIndices[i]
                                        + genCoord->getSkeletonIndex() + 1));
//          std::cout << "Skeleton " << i << " Dof " << j
//                    << " hits lower bound" << std::endl;
        }
      }
    }
  }

  if (mCollisionDetector->getNumContacts() == 0
      && mLimitingDofIndex.size() == 0)
  {
    for (int i = 0; i < mSkeletons.size(); i++)
      mContactForces[i].setZero();
    if (mConstraints.size() == 0)
    {
      for (int i = 0; i < mSkeletons.size(); i++)
        mTotalConstrForces[i].setZero();
    }
    else
    {
      computeConstraintWithoutContact();
    }
  }
  else
  {
    if (mUseODELCPSolver)
    {
      fillMatricesODE();
      solve();
      applySolutionODE();
    }
    else
    {
      fillMatrices();
      solve();
      applySolution();
    }
  }
  //            t1.stopTimer();
  //            t1.printScreen();
}

void ConstraintDynamics::addConstraint(Constraint *_constr)
{
  // create an entry in the mSkeletonIDMap
  Eigen::Vector2i id(-1, -1);
  dynamics::Skeleton *skel1 = _constr->getBodyNode1()->getSkeleton();
  for (int i = 0; i < mSkeletons.size(); i++) {
    if (skel1 == mSkeletons[i]) {
      id[0] = i;
      break;
    }
  }
  if (id[0] == -1) {
    std::cout << "Invalid constraint" << std::endl;
    return;
  }

  if (_constr->getBodyNode2()) {
    dynamics::Skeleton *skel2 = _constr->getBodyNode2()->getSkeleton();
    for (int i = 0; i < mSkeletons.size(); i++) {
      if (skel2 == mSkeletons[i]) {
        id[1] = i;
        break;
      }
    }
    if (id[1] == -1) {
      std::cout << "Invalid constraint" << std::endl;
      return;
    }
  }
  mSkeletonIDMap[_constr] = id;

  // extend global structures for the new constraint
  mConstraints.push_back(_constr);
  mTotalRows += _constr->getNumRows();
  for (int i = 0; i < mSkeletons.size(); i++) {
    mJ[i].conservativeResize(mTotalRows, mSkeletons[i]->getNumGenCoords());
    mJ[i].bottomRows(_constr->getNumRows()).setZero();
  }
  mC = Eigen::VectorXd(mTotalRows);
  mCDot = Eigen::VectorXd(mTotalRows);
  mGInv = Eigen::MatrixXd::Zero(mTotalRows, mTotalRows);
  mTauHat = Eigen::VectorXd(mTotalRows);
}

void ConstraintDynamics::deleteConstraint(Constraint* _constr)
{
  int index = -1;
  for (int i = 0; i < mConstraints.size(); i++) {
    if (_constr == mConstraints[i]) {
      index = i;
      break;
    }
  }
  if (index == -1)
    return;

  int count = 0;
  for (int i = 0; i < index; i++)
    count += mConstraints[i]->getNumRows();
  int shiftRows = mTotalRows - count - mConstraints[index]->getNumRows();
  mTotalRows -= mConstraints[index]->getNumRows();

  for (int i = 0; i < mSkeletons.size(); i++) {
    mJ[i].block(count, 0, shiftRows, mSkeletons[i]->getNumGenCoords()) = mJ[i].bottomRows(shiftRows);
    mJ[i].conservativeResize(mTotalRows, mSkeletons[i]->getNumGenCoords());
  }
  mC.resize(mTotalRows);
  mCDot.resize(mTotalRows);
  mGInv.resize(mTotalRows, mTotalRows);
  mTauHat.resize(mTotalRows);

  mConstraints.erase(mConstraints.begin() + index);
  mSkeletonIDMap.erase(_constr);
  delete _constr;
}

void dart::constraint::ConstraintDynamics::deleteConstraint()
{
  int nConstr = mConstraints.size();
  for (int i = nConstr - 1; i >= 0; i--) {
    deleteConstraint(mConstraints[i]);
  }
}

void ConstraintDynamics::addSkeleton(dynamics::Skeleton* _skeleton)
{
  assert(_skeleton != NULL && "Invalid skeleton.");

  // If mSkeletons already has _skeleton, then we do nothing.
  if (find(mSkeletons.begin(), mSkeletons.end(), _skeleton) !=
      mSkeletons.end())
  {
    std::cout << "Skeleton [" << _skeleton->getName()
              << "] is already in the constraint handler." << std::endl;
    return;
  }

  mSkeletons.push_back(_skeleton);

  int nSkeletons = mSkeletons.size();
  int nBodyNodes = _skeleton->getNumBodyNodes();
  int nGenCoords = _skeleton->getNumGenCoords();

  // Add all body nodes into mCollisionChecker
  for (int i = 0; i < nBodyNodes; i++)
  {
    mCollisionDetector->addCollisionSkeletonNode(_skeleton->getBodyNode(i));
    mBodyIndexToSkelIndex.push_back(nSkeletons-1);
  }

  Eigen::VectorXd newConstrForce;
  if (_skeleton->isMobile() && _skeleton->getNumGenCoords() > 0)
    newConstrForce = Eigen::VectorXd::Zero(nGenCoords);
  mContactForces.push_back(newConstrForce);
  mTotalConstrForces.push_back(newConstrForce);

  if (_skeleton->isMobile() && _skeleton->getNumGenCoords() > 0)
    mIndices.push_back(mIndices.back() + nGenCoords);
  else
    mIndices.push_back(mIndices.back());

  assert(mMInv.rows() == mMInv.rows() && "The mass matrix should be square.");
  int N = mMInv.rows();
  if (_skeleton->isMobile() && _skeleton->getNumGenCoords() > 0)
    N += nGenCoords;
  mMInv = Eigen::MatrixXd::Zero(N, N);
  mTauStar = Eigen::VectorXd::Zero(N);
  mJ.push_back(Eigen::MatrixXd());
  mPreJ.push_back(Eigen::MatrixXd());
  mJMInv.push_back(Eigen::MatrixXd());
  mZ = Eigen::MatrixXd(N, N);
}

void ConstraintDynamics::removeSkeleton(dynamics::Skeleton* _skeleton)
{
  assert(_skeleton != NULL && "Invalid skeleton.");

  // Find index of _skeleton in mSkeleton.
  int iSkeleton = 0;
  for (; iSkeleton < mSkeletons.size(); ++iSkeleton)
    if (mSkeletons[iSkeleton] == _skeleton)
      break;

  // If iSkeleton is equal to the number of skeletons, then _skeleton is not
  // in mSkeleton. We do nothing.
  if (iSkeleton == mSkeletons.size())
  {
    std::cout << "Skeleton [" << _skeleton->getName()
              << "] is not in constraint handler." << std::endl;
    return;
  }

  int nBodyNodes = _skeleton->getNumBodyNodes();
  int nGenCoords = _skeleton->getNumGenCoords();

  // Add all body nodes into mCollisionChecker
  for (int i = 0; i < nBodyNodes; i++)
    mCollisionDetector->removeCollisionSkeletonNode(_skeleton->getBodyNode(i));

  // Update mBodyIndexToSkelIndex
  mBodyIndexToSkelIndex.erase(
        remove(mBodyIndexToSkelIndex.begin(),
               mBodyIndexToSkelIndex.end(), iSkeleton),
        mBodyIndexToSkelIndex.end());
  int it = 0;
  for (; it < mBodyIndexToSkelIndex.size(); ++it)
    if (mBodyIndexToSkelIndex[it] == iSkeleton + 1)
      break;
  for (int i = it; i < mBodyIndexToSkelIndex.size(); ++i)
    mBodyIndexToSkelIndex[i]--;

  mContactForces.pop_back();
  mTotalConstrForces.pop_back();

  // Update mIndices.
  if (_skeleton->isMobile() && _skeleton->getNumGenCoords() > 0)
    for (int i = iSkeleton + 1; i < mIndices.size() - 1; ++i)
      mIndices[i] = mIndices[i+1] - nGenCoords;
  else
    for (int i = iSkeleton + 1; i < mIndices.size() - 1; ++i)
      mIndices[i] = mIndices[i+1];
  mIndices.pop_back();

  assert(mMInv.rows() == mMInv.rows() && "The mass matrix should be square.");
  int N = mMInv.rows();
  if (_skeleton->isMobile() && _skeleton->getNumGenCoords() > 0)
    N -= nGenCoords;
  mMInv = Eigen::MatrixXd::Zero(N, N);
  mTauStar = Eigen::VectorXd::Zero(N);
  mJ.pop_back();
  mPreJ.pop_back();
  mJMInv.pop_back();
  mZ = Eigen::MatrixXd(N, N);

  // Remove _skeleton in mSkeletons and delete it.
  mSkeletons.erase(remove(mSkeletons.begin(), mSkeletons.end(), _skeleton),
                   mSkeletons.end());
}

void ConstraintDynamics::setTimeStep(double _timeStep)
{
  mDt = _timeStep;
}

double ConstraintDynamics::getTimeStep() const
{
  return mDt;
}

void ConstraintDynamics::setCollisionDetector(
    collision::CollisionDetector* _collisionDetector)
{
  // TODO(JS): If there are collision objects in the old collision detector,
  //           then we should register the objects to the new collision
  //           detector. This is not implemented yet. See Issue #127.

  assert(_collisionDetector != NULL && "Invalid collision detector.");

  if (_collisionDetector == mCollisionDetector)
    return;

  delete mCollisionDetector;

  mCollisionDetector = _collisionDetector;

  initialize();
}

Eigen::VectorXd ConstraintDynamics::getTotalConstraintForce(
    int _skelIndex) const
{
  return mTotalConstrForces[_skelIndex];
}

Eigen::VectorXd ConstraintDynamics::getContactForce(int _skelIndex) const
{
  return mContactForces[_skelIndex];
}

collision::CollisionDetector* ConstraintDynamics::getCollisionDetector() const
{
  return mCollisionDetector;
}

int ConstraintDynamics::getNumContacts() const
{
  return mCollisionDetector->getNumContacts();
}

Constraint*ConstraintDynamics::getConstraint(int _index) const
{
  return mConstraints[_index];
}

void ConstraintDynamics::initialize()
{
  mBodyIndexToSkelIndex.clear();
  // Add all body nodes into mCollisionDetector
  int rows = 0;
  int cols = 0;
  for (int i = 0; i < mSkeletons.size(); i++)
  {
    dynamics::Skeleton* skel = mSkeletons[i];
    int nNodes = skel->getNumBodyNodes();

    for (int j = 0; j < nNodes; j++)
    {
      dynamics::BodyNode* node = skel->getBodyNode(j);
      if (node->isCollidable())
      {
        mCollisionDetector->addCollisionSkeletonNode(node);
        mBodyIndexToSkelIndex.push_back(i);
      }
    }

    if (mSkeletons[i]->isMobile() && mSkeletons[i]->getNumGenCoords() > 0)
    {
      // Immobile objets have mass of infinity
      rows += skel->getAugMassMatrix().rows();
      cols += skel->getAugMassMatrix().cols();
    }
  }

  mContactForces.resize(mSkeletons.size());
  mTotalConstrForces.resize(mSkeletons.size());
  for (int i = 0; i < mSkeletons.size(); i++)
  {
    int dof = mSkeletons[i]->getNumGenCoords();
    if (!mSkeletons[i]->isMobile() || dof == 0)
      continue;
    mContactForces[i]     = Eigen::VectorXd::Zero(dof);
    mTotalConstrForces[i] = Eigen::VectorXd::Zero(dof);
  }

  mMInv = Eigen::MatrixXd::Zero(rows, cols);
  mTauStar = Eigen::VectorXd::Zero(rows);

  // Initialize the index vector:
  // If we have 3 skeletons,
  // mIndices[0] = 0
  // mIndices[1] = nDof0
  // mIndices[2] = nDof0 + nDof1
  // mIndices[3] = nDof0 + nDof1 + nDof2

  mIndices.clear();
  int sumNDofs = 0;
  mIndices.push_back(sumNDofs);

  for (int i = 0; i < mSkeletons.size(); i++)
  {
    dynamics::Skeleton* skel = mSkeletons[i];
    int nDofs = skel->getNumGenCoords();
    if (!mSkeletons[i]->isMobile() || mSkeletons[i]->getNumGenCoords() == 0)
      nDofs = 0;
    sumNDofs += nDofs;
    mIndices.push_back(sumNDofs);
  }

  mTotalRows = 0;
  mJ.resize(mSkeletons.size());
  mPreJ.resize(mSkeletons.size());
  mJMInv.resize(mSkeletons.size());
  mZ = Eigen::MatrixXd(rows, cols);
}

void ConstraintDynamics::computeConstraintWithoutContact()
{
  updateMassMat();
  updateConstraintTerms();
  Eigen::VectorXd lambda = mGInv * mTauHat;

  for (int i = 0; i < mSkeletons.size(); i++)
  {
    if (!mSkeletons[i]->isMobile() || mSkeletons[i]->getNumGenCoords() == 0)
      continue;
    mTotalConstrForces[i] = mJ[i].transpose() * lambda;
  }
}

void ConstraintDynamics::fillMatrices()
{
  int nContacts = getNumContacts();
  int nJointLimits = mLimitingDofIndex.size();
  int nConstrs = mConstraints.size();
  int cd = nContacts * mNumDir;
  int dimA = nContacts * (2 + mNumDir) + nJointLimits;
  mA = Eigen::MatrixXd::Zero(dimA, dimA);
  mQBar = Eigen::VectorXd::Zero(dimA);
  updateMassMat();
  updateTauStar();

  Eigen::MatrixXd augMInv = mMInv;
  Eigen::VectorXd tauVec = Eigen::VectorXd::Zero(getTotalNumDofs());
  if (nConstrs > 0)
  {
    updateConstraintTerms();
    augMInv -= mZ.triangularView<Eigen::Lower>();

    Eigen::VectorXd tempVec = mDt * mGInv * mTauHat;
    for (int i = 0; i < mSkeletons.size(); i++)
    {
      if (!mSkeletons[i]->isMobile() || mSkeletons[i]->getNumGenCoords() == 0)
        continue;
      tauVec.segment(mIndices[i], mSkeletons[i]->getNumGenCoords()) =
          mJ[i].transpose() * tempVec;
    }
  }
  tauVec = mMInv * (tauVec + mTauStar);

  Eigen::MatrixXd NTerm(getTotalNumDofs(), nContacts);
  Eigen::MatrixXd BTerm(getTotalNumDofs(), cd);

  if (nContacts > 0)
  {
    updateNBMatrices();
    Eigen::MatrixXd E = getContactMatrix();
    // Compute NTerm and BTerm
    NTerm.noalias() = augMInv * mN;
    BTerm.noalias() = augMInv * mB;
    mA.block(0, 0, nContacts, nContacts).triangularView<Eigen::Lower>()
        = mN.transpose() * NTerm;
    mA.block(0, 0, nContacts, nContacts).triangularView<Eigen::StrictlyUpper>()
        = mA.block(0, 0, nContacts, nContacts).transpose();
    mA.block(0, nContacts, nContacts, cd).noalias() = mN.transpose() * BTerm;
    mA.block(nContacts, 0, cd, nContacts)
        = mA.block(0, nContacts, nContacts, cd).transpose();
    mA.block(nContacts, nContacts, cd, cd).triangularView<Eigen::Lower>()
        = mB.transpose() * BTerm;
    mA.block(nContacts, nContacts,
             cd, cd).triangularView<Eigen::StrictlyUpper>()
        = mA.block(nContacts, nContacts, cd, cd).transpose();
    mA.block(nContacts, nContacts + cd, cd, nContacts) = E;
    mA.block(nContacts + cd, 0, nContacts, nContacts) = getMuMatrix();
    mA.block(nContacts + cd, nContacts, nContacts, cd) = -E.transpose();

    mQBar.segment(0, nContacts).noalias() = mN.transpose() * tauVec;
    mQBar.segment(nContacts, cd).noalias() = mB.transpose() * tauVec;
  }

  if (nJointLimits > 0)
  {
    int jointStart = 2 * nContacts + cd;
    for (int i = 0; i < nJointLimits; i++)
    {
      for (int j = 0; j < nJointLimits; j++)
      {
        if (mLimitingDofIndex[i] * mLimitingDofIndex[j] < 0)
          mA(jointStart + i, jointStart + j) =
              -augMInv(abs(mLimitingDofIndex[i]) - 1,
                       abs(mLimitingDofIndex[j]) - 1);
        else
          mA(jointStart + i, jointStart + j) =
              augMInv(abs(mLimitingDofIndex[i]) - 1,
                      abs(mLimitingDofIndex[j]) - 1);
      }
    }

    for (int i = 0; i < nJointLimits; i++)
    {
      if (mLimitingDofIndex[i] > 0)  // hitting upper bound
        mQBar[jointStart + i] = -tauVec[abs(mLimitingDofIndex[i]) - 1];
      else  // hitting lower bound
        mQBar[jointStart + i] = tauVec[abs(mLimitingDofIndex[i]) - 1];
    }

    if (nContacts > 0)
    {
      Eigen::MatrixXd STerm(mMInv.rows(), nJointLimits);
      for (int i = 0; i < nJointLimits; i++)
      {
        if (mLimitingDofIndex[i] > 0)  // hitting upper bound
          STerm.col(i) = -augMInv.col(mLimitingDofIndex[i] - 1);
        else
          STerm.col(i) = augMInv.col(abs(mLimitingDofIndex[i]) - 1);
      }
      mA.block(0, jointStart, nContacts, nJointLimits) =
          mN.transpose() * STerm;

      mA.block(nContacts, jointStart, cd, nJointLimits) =
          mB.transpose() * STerm;

      for (int i = 0; i < nJointLimits; i++)
      {
        if (mLimitingDofIndex[i] > 0)
        {  // hitting uppder bound
          mA.block(jointStart + i, 0, 1, nContacts) =
              -NTerm.row(mLimitingDofIndex[i] - 1);
          mA.block(jointStart + i, nContacts, 1, cd) =
              -BTerm.row(mLimitingDofIndex[i] - 1);
        }
        else
        {
          mA.block(jointStart + i, 0, 1, nContacts) =
              NTerm.row(abs(mLimitingDofIndex[i]) - 1);
          mA.block(jointStart + i, nContacts, 1, cd) =
              BTerm.row(abs(mLimitingDofIndex[i]) - 1);
        }
      }
    }
  }
  mQBar /= mDt;

  int cfmSize = getNumContacts() * (1 + mNumDir);
  // Add small values to diagnal to keep it away from singular, similar to cfm
  // varaible in ODE
  for (int i = 0; i < cfmSize; ++i)
    mA(i, i) += 0.001 * mA(i, i);
}

void ConstraintDynamics::fillMatricesODE()
{
  int nContacts = getNumContacts();
  int nJointLimits = mLimitingDofIndex.size();
  int nConstrs = mConstraints.size();
  int cd = nContacts * 2;
  int dimA = nContacts * 3 + nJointLimits;
  mA = Eigen::MatrixXd(dimA, dimA);
  mQBar = Eigen::VectorXd(dimA);
  updateMassMat();
  updateTauStar();

  Eigen::MatrixXd augMInv = mMInv;
  Eigen::VectorXd tauVec = Eigen::VectorXd::Zero(getTotalNumDofs());
  if (nConstrs > 0)
  {
    updateConstraintTerms();
    augMInv -= mZ.triangularView<Eigen::Lower>();

    Eigen::VectorXd tempVec = mDt * mGInv * mTauHat;
    for (int i = 0; i < mSkeletons.size(); i++)
    {
      if (!mSkeletons[i]->isMobile() || mSkeletons[i]->getNumGenCoords() == 0)
        continue;
      tauVec.segment(mIndices[i], mSkeletons[i]->getNumGenCoords()) =
          mJ[i].transpose() * tempVec;
    }
  }
  tauVec = mMInv * (tauVec + mTauStar);

  Eigen::MatrixXd NTerm(getTotalNumDofs(), nContacts);
  Eigen::MatrixXd BTerm(getTotalNumDofs(), cd);

  if (nContacts > 0)
  {
    updateNBMatricesODE();
    // Compute NTerm and BTerm
    NTerm.noalias() = augMInv * mN;
    BTerm.noalias() = augMInv * mB;
    mA.block(0, 0, nContacts, nContacts).triangularView<Eigen::Lower>() =
        mN.transpose() * NTerm;
    mA.block(0, 0,
             nContacts, nContacts).triangularView<Eigen::StrictlyUpper>() =
        mA.block(0, 0, nContacts, nContacts).transpose();
    mA.block(0, nContacts, nContacts, cd).noalias() = mN.transpose() * BTerm;
    mA.block(nContacts, 0, cd, nContacts) =
        mA.block(0, nContacts, nContacts, cd).transpose();
    mA.block(nContacts, nContacts, cd, cd).triangularView<Eigen::Lower>() =
        mB.transpose() * BTerm;
    mA.block(nContacts, nContacts,
             cd, cd).triangularView<Eigen::StrictlyUpper>()
        = mA.block(nContacts, nContacts, cd, cd).transpose();

    mQBar.segment(0, nContacts).noalias() = mN.transpose() * tauVec;
    mQBar.segment(nContacts, cd).noalias() = mB.transpose() * tauVec;
  }

  if (nJointLimits > 0)
  {
    int jointStart = nContacts + cd;
    for (int i = 0; i < nJointLimits; i++)
    {
      for (int j = 0; j < nJointLimits; j++)
      {
        if (mLimitingDofIndex[i] * mLimitingDofIndex[j] < 0)
          mA(jointStart + i, jointStart + j) =
              -augMInv(abs(mLimitingDofIndex[i]) - 1,
                       abs(mLimitingDofIndex[j]) - 1);
        else
          mA(jointStart + i, jointStart + j) =
              augMInv(abs(mLimitingDofIndex[i]) - 1,
                      abs(mLimitingDofIndex[j]) - 1);
      }
    }

    for (int i = 0; i < nJointLimits; i++)
    {
      if (mLimitingDofIndex[i] > 0)  // hitting upper bound
        mQBar[jointStart + i] = -tauVec[abs(mLimitingDofIndex[i]) - 1];
      else  // hitting lower bound
        mQBar[jointStart + i] = tauVec[abs(mLimitingDofIndex[i]) - 1];
    }

    if (nContacts > 0)
    {
      Eigen::MatrixXd STerm(mMInv.rows(), nJointLimits);
      for (int i = 0; i < nJointLimits; i++)
      {
        if (mLimitingDofIndex[i] > 0)  // hitting upper bound
          STerm.col(i) = -augMInv.col(mLimitingDofIndex[i] - 1);
        else
          STerm.col(i) = augMInv.col(abs(mLimitingDofIndex[i]) - 1);
      }
      mA.block(0, jointStart, nContacts, nJointLimits) = mN.transpose() * STerm;

      mA.block(nContacts, jointStart, cd, nJointLimits) =
          mB.transpose() * STerm;

      for (int i = 0; i < nJointLimits; i++)
      {
        if (mLimitingDofIndex[i] > 0)
        {  // hitting uppder bound
          mA.block(jointStart + i, 0, 1, nContacts) =
              -NTerm.row(mLimitingDofIndex[i] - 1);
          mA.block(jointStart + i, nContacts, 1, cd) =
              -BTerm.row(mLimitingDofIndex[i] - 1);
        }
        else
        {
          mA.block(jointStart + i, 0, 1, nContacts) =
              NTerm.row(abs(mLimitingDofIndex[i]) - 1);
          mA.block(jointStart + i, nContacts, 1, cd) =
              BTerm.row(abs(mLimitingDofIndex[i]) - 1);
        }
      }
    }
  }
  mQBar /= mDt;

  int cfmSize = getNumContacts() * 3;
  // Add small values to diagnal to keep it away from singular, similar to cfm
  // varaible in ODE
  for (int i = 0; i < cfmSize; ++i)
    mA(i, i) += 0.001 * mA(i, i);
}

bool ConstraintDynamics::solve()
{
  lcpsolver::LCPSolver solver = lcpsolver::LCPSolver();
  bool b = solver.Solve(mA, mQBar, &mX, getNumContacts(), mMu, mNumDir,
                        mUseODELCPSolver);
  return b;
}

void ConstraintDynamics::applySolution()
{
  Eigen::VectorXd contactForces(Eigen::VectorXd::Zero(getTotalNumDofs()));
  Eigen::VectorXd jointLimitForces(Eigen::VectorXd::Zero(getTotalNumDofs()));
  int nContacts = getNumContacts();

  if (nContacts > 0)
  {
    Eigen::VectorXd f_n = mX.head(nContacts);
    Eigen::VectorXd f_d = mX.segment(nContacts, nContacts * mNumDir);
    contactForces.noalias() = mN * f_n;
    contactForces.noalias() += mB * f_d;
    for (int i = 0; i < nContacts; i++)
    {
      collision::Contact& contact = mCollisionDetector->getContact(i);
      contact.force.noalias() =
          getTangentBasisMatrix(contact.point, contact.normal)
          * f_d.segment(i * mNumDir, mNumDir);
      contact.force.noalias() += contact.normal * f_n[i];
    }
  }
  for (int i = 0; i < mLimitingDofIndex.size(); i++)
  {
    // hitting upper bound
    if (mLimitingDofIndex[i] > 0)
    {
      jointLimitForces[mLimitingDofIndex[i] - 1] =
          -mX[nContacts * (2 + mNumDir) + i];
    }
    else
    {
      jointLimitForces[abs(mLimitingDofIndex[i]) - 1] =
          mX[nContacts * (2 + mNumDir) + i];
    }
  }

  Eigen::VectorXd lambda = Eigen::VectorXd::Zero(mGInv.rows());
  for (int i = 0; i < mSkeletons.size(); i++)
  {
    if (!mSkeletons[i]->isMobile() || mSkeletons[i]->getNumGenCoords() == 0)
      continue;
    mContactForces[i] = contactForces.segment(mIndices[i],
                                              mSkeletons[i]->getNumGenCoords());

    mTotalConstrForces[i] = mContactForces[i]
                            + jointLimitForces.segment(
                              mIndices[i], mSkeletons[i]->getNumGenCoords());

    if (mConstraints.size() > 0)
    {
      Eigen::VectorXd tempVec =
          mGInv * (mTauHat - mJMInv[i]
                   * (contactForces.segment(
                        mIndices[i], mSkeletons[i]->getNumGenCoords())
                      + jointLimitForces.segment(
                        mIndices[i], mSkeletons[i]->getNumGenCoords())));
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

void ConstraintDynamics::applySolutionODE()
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
      collision::Contact& contact = mCollisionDetector->getContact(i);
      contact.force.noalias() =
          getTangentBasisMatrixODE(contact.point, contact.normal)
          * f_d.segment(i * 2, 2);
      contact.force.noalias() += contact.normal * f_n[i];

      // Add contact force to body nodes
      Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
      Eigen::Vector6d F = Eigen::Vector6d::Zero();

      dynamics::BodyNode* bn1 = contact.collisionNode1->getBodyNode();
      T.translation() = bn1->getWorldTransform().inverse() * contact.point;
      F.tail<3>()     = bn1->getWorldTransform().linear().transpose()
                        * contact.force;
      bn1->addContactForce(math::dAdInvT(T, F));

      dynamics::BodyNode* bn2 = contact.collisionNode2->getBodyNode();
      T.translation() = bn2->getWorldTransform().inverse() * contact.point;
      F.tail<3>()     = bn2->getWorldTransform().linear().transpose()
                        * (-contact.force);
      bn2->addContactForce(math::dAdInvT(T, F));
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

void ConstraintDynamics::updateMassMat()
{
  int start = 0;
  for (int i = 0; i < mSkeletons.size(); i++)
  {
    if (!mSkeletons[i]->isMobile() || mSkeletons[i]->getNumGenCoords() == 0)
      continue;
    mMInv.block(
          start, start,
          mSkeletons[i]->getNumGenCoords(),
          mSkeletons[i]->getNumGenCoords())
        = mSkeletons[i]->getInvAugMassMatrix();
    start += mSkeletons[i]->getNumGenCoords();
  }
}

void ConstraintDynamics::updateTauStar()
{
  int startRow = 0;
  for (int i = 0; i < mSkeletons.size(); i++)
  {
    if (!mSkeletons[i]->isMobile() || mSkeletons[i]->getNumGenCoords() == 0)
      continue;

    Eigen::VectorXd tau = mSkeletons[i]->getExternalForceVector()
                          + mSkeletons[i]->getInternalForceVector()
                          + mSkeletons[i]->getDampingForceVector();
    Eigen::VectorXd tauStar =
        (mSkeletons[i]->getAugMassMatrix() * mSkeletons[i]->get_dq())
        - (mDt * (mSkeletons[i]->getCombinedVector() - tau));
    mTauStar.block(startRow, 0, tauStar.rows(), 1) = tauStar;
    startRow += tauStar.rows();
  }
}

void ConstraintDynamics::updateNBMatrices()
{
  mN = Eigen::MatrixXd::Zero(getTotalNumDofs(), getNumContacts());
  mB = Eigen::MatrixXd::Zero(getTotalNumDofs(), getNumContacts() * mNumDir);
  for (int i = 0; i < getNumContacts(); i++)
  {
    collision::Contact& c = mCollisionDetector->getContact(i);
    Eigen::Vector3d p = c.point;
    int skelID1 = mBodyIndexToSkelIndex[c.collisionNode1->getIndex()];
    int skelID2 = mBodyIndexToSkelIndex[c.collisionNode2->getIndex()];

    Eigen::Vector3d N21 = c.normal;
    Eigen::Vector3d N12 = -c.normal;
    Eigen::MatrixXd B21 = getTangentBasisMatrix(p, N21);
    Eigen::MatrixXd B12 = -B21;

    if (mSkeletons[skelID1]->isMobile()
        && mSkeletons[skelID1]->getNumGenCoords() > 0)
    {
      int index1 = mIndices[skelID1];
      int NDOF1 =
          c.collisionNode1->getBodyNode()->getSkeleton()->getNumGenCoords();
      //    Vector3d N21 = c.normal;
      Eigen::MatrixXd J21t = getJacobian(c.collisionNode1->getBodyNode(), p);
      mN.block(index1, i, NDOF1, 1).noalias() = J21t * N21;
      // B21 = getTangentBasisMatrix(p, N21);
      mB.block(index1, i * mNumDir, NDOF1, mNumDir).noalias() = J21t * B21;
    }

    if (mSkeletons[skelID2]->isMobile()
        && mSkeletons[skelID2]->getNumGenCoords() > 0)
    {
      int index2 = mIndices[skelID2];
      int NDOF2 =
          c.collisionNode2->getBodyNode()->getSkeleton()->getNumGenCoords();
      // Vector3d N12 = -c.normal;
      // if (B21.rows() == 0)
      //   B12 = getTangentBasisMatrix(p, N12);
      // else
      //   B12 = -B21;
      Eigen::MatrixXd J12t = getJacobian(c.collisionNode2->getBodyNode(), p);
      mN.block(index2, i, NDOF2, 1).noalias() += J12t * N12;
      mB.block(index2, i * mNumDir, NDOF2, mNumDir).noalias() += J12t * B12;
    }
  }
}

void ConstraintDynamics::updateNBMatricesODE()
{
  mN = Eigen::MatrixXd::Zero(getTotalNumDofs(), getNumContacts());
  mB = Eigen::MatrixXd::Zero(getTotalNumDofs(), getNumContacts() * 2);
  for (int i = 0; i < getNumContacts(); i++)
  {
    collision::Contact& c = mCollisionDetector->getContact(i);
    Eigen::Vector3d p = c.point;
    int skelID1 = mBodyIndexToSkelIndex[c.collisionNode1->getIndex()];
    int skelID2 = mBodyIndexToSkelIndex[c.collisionNode2->getIndex()];

    Eigen::Vector3d N21 = c.normal;
    Eigen::Vector3d N12 = -c.normal;
    Eigen::MatrixXd B21 = getTangentBasisMatrixODE(p, N21);
    Eigen::MatrixXd B12 = -B21;

    if (mSkeletons[skelID1]->isMobile()
        && mSkeletons[skelID1]->getNumGenCoords() > 0)
    {
      int index1 = mIndices[skelID1];
      int NDOF1 =
          c.collisionNode1->getBodyNode()->getSkeleton()->getNumGenCoords();
      //    Vector3d N21 = c.normal;
      Eigen::MatrixXd J21t = getJacobian(c.collisionNode1->getBodyNode(), p);
      mN.block(index1, i, NDOF1, 1).noalias() = J21t * N21;
      // B21 = getTangentBasisMatrix(p, N21);
      mB.block(index1, i * 2, NDOF1, 2).noalias() = J21t * B21;
    }

    if (mSkeletons[skelID2]->isMobile()
        && mSkeletons[skelID2]->getNumGenCoords() > 0)
    {
      int index2 = mIndices[skelID2];
      int NDOF2 =
          c.collisionNode2->getBodyNode()->getSkeleton()->getNumGenCoords();
      // Vector3d N12 = -c.normal;
      // if (B21.rows() == 0)
      //   B12 = getTangentBasisMatrix(p, N12);
      // else
      //    B12 = -B21;
      Eigen::MatrixXd J12t = getJacobian(c.collisionNode2->getBodyNode(), p);
      mN.block(index2, i, NDOF2, 1).noalias() += J12t * N12;
      mB.block(index2, i * 2, NDOF2, 2).noalias() += J12t * B12;
    }
  }
}

Eigen::MatrixXd ConstraintDynamics::getJacobian(dynamics::BodyNode* node,
                                                const Eigen::Vector3d& p)
{
  int nDofs = node->getSkeleton()->getNumGenCoords();
  Eigen::MatrixXd Jt = Eigen::MatrixXd::Zero(nDofs, 3);
  const Eigen::Vector3d& pos = node->getWorldTransform().translation();
  Eigen::MatrixXd JtBody
      = node->getWorldJacobian(p - pos).bottomRows<3>().transpose();

  for (int dofIndex = 0; dofIndex < node->getNumDependentGenCoords();
       dofIndex++)
  {
    int i = node->getDependentGenCoordIndex(dofIndex);
    Jt.row(i) = JtBody.row(dofIndex);
  }

  return Jt;
}

Eigen::MatrixXd ConstraintDynamics::getTangentBasisMatrix(
    const Eigen::Vector3d& p, const Eigen::Vector3d& n)
{
  Eigen::MatrixXd T(Eigen::MatrixXd::Zero(3, mNumDir));

  // Pick an arbitrary vector to take the cross product of (in this case,
  // Z-axis)
  Eigen::Vector3d tangent = Eigen::Vector3d::UnitZ().cross(n);
  // If they're too close, pick another tangent (use X-axis as arbitrary vector)
  if (tangent.norm() < CONSTRAIN_DYNAMICS_EPSILON)
    tangent = Eigen::Vector3d::UnitX().cross(n);
  tangent.normalize();

  // Rotate the tangent around the normal to compute bases.
  // Note: a possible speedup is in place for mNumDir % 2 = 0
  // Each basis and its opposite belong in the matrix, so we iterate half as
  // many times.
  double angle = (2 * DART_PI) / mNumDir;
  int iter = (mNumDir % 2 == 0) ? mNumDir / 2 : mNumDir;
  for (int i = 0; i < iter; i++)
  {
    Eigen::Vector3d basis = Eigen::Quaterniond(Eigen::AngleAxisd(i * angle, n))
                            * tangent;
    T.col(i) = basis;

    if (mNumDir % 2 == 0)
      T.col(i + iter) = -basis;
  }
  return T;
}

Eigen::MatrixXd ConstraintDynamics::getTangentBasisMatrixODE(
    const Eigen::Vector3d& p, const Eigen::Vector3d& n)
{
  Eigen::MatrixXd T(Eigen::MatrixXd::Zero(3, 2));

  // Pick an arbitrary vector to take the cross product of (in this case,
  // Z-axis)
  Eigen::Vector3d tangent = Eigen::Vector3d::UnitZ().cross(n);
  // If they're too close, pick another tangent (use X-axis as arbitrary vector)
  if (tangent.norm() < CONSTRAIN_DYNAMICS_EPSILON)
    tangent = Eigen::Vector3d::UnitX().cross(n);
  tangent.normalize();

  // Rotate the tangent around the normal to compute bases.
  // Note: a possible speedup is in place for mNumDir % 2 = 0
  // Each basis and its opposite belong in the matrix, so we iterate half as
  // many times

  double angle = DART_PI / 2.0;
  T.col(0) = tangent;
  T.col(1) = Eigen::Quaterniond(Eigen::AngleAxisd(angle, n)) * tangent;
  return T;
}

Eigen::MatrixXd ConstraintDynamics::getContactMatrix() const
{
  Eigen::MatrixXd E = Eigen::MatrixXd::Zero(getNumContacts() * mNumDir,
                                            getNumContacts());
  Eigen::VectorXd column = Eigen::VectorXd::Ones(mNumDir);
  for (int i = 0; i < getNumContacts(); i++)
    E.block(i * mNumDir, i, mNumDir, 1) = column;
  return E;
}

Eigen::MatrixXd ConstraintDynamics::getMuMatrix() const
{
  int c = getNumContacts();
  return Eigen::MatrixXd::Identity(c, c) * mMu;
}

void ConstraintDynamics::updateConstraintTerms()
{
  // update preJ
  mPreJ = mJ;
  // compute J
  int count = 0;
  for (int i = 0; i < mConstraints.size(); i++)
  {
    if (mSkeletonIDMap[mConstraints[i]][1] == -1)
      mConstraints[i]->updateDynamics(mJ[mSkeletonIDMap[mConstraints[i]][0]], mC, mCDot, count);
    else
      mConstraints[i]->updateDynamics(mJ[mSkeletonIDMap[mConstraints[i]][0]], mJ[mSkeletonIDMap[mConstraints[i]][1]], mC, mCDot, count);

    count += mConstraints[i]->getNumRows();
  }
  // compute JMInv, GInv, Z
  mGInv.triangularView<Eigen::Lower>().setZero();
  for (int i = 0; i < mSkeletons.size(); i++)
  {
    if (!mSkeletons[i]->isMobile() || mSkeletons[i]->getNumGenCoords() == 0)
      continue;
    mJMInv[i] = mJ[i] * mSkeletons[i]->getInvAugMassMatrix();
    mGInv.triangularView<Eigen::Lower>() += (mJMInv[i] * mJ[i].transpose());
  }
  mGInv = mGInv.ldlt().solve(Eigen::MatrixXd::Identity(mTotalRows, mTotalRows));
  for (int i = 0; i < mSkeletons.size(); i++)
  {
    if (!mSkeletons[i]->isMobile() || mSkeletons[i]->getNumGenCoords() == 0)
      continue;
    mZ.block(mIndices[i], mIndices[i],
             mSkeletons[i]->getNumGenCoords(),
             mSkeletons[i]->getNumGenCoords()).triangularView<Eigen::Lower>() =
        mJMInv[i].transpose() * mGInv * mJMInv[i];
    for (int j = 0; j < i; j++)
    {
      if (!mSkeletons[j]->isMobile() || mSkeletons[j]->getNumGenCoords() == 0)
        continue;
      mZ.block(mIndices[i], mIndices[j],
               mSkeletons[i]->getNumGenCoords(),
               mSkeletons[j]->getNumGenCoords()).noalias() =
          mJMInv[i].transpose() * mGInv * mJMInv[j];
    }
  }

  // compute tauHat
  double ks = 500;
  double kd = 50;
  mTauHat.setZero();
  for (int i = 0; i < mSkeletons.size(); i++)
  {
    if (!mSkeletons[i]->isMobile() || mSkeletons[i]->getNumGenCoords() == 0)
      continue;
    Eigen::VectorXd qDot = mSkeletons[i]->get_dq();
    mTauHat.noalias() += -(mJ[i] - mPreJ[i]) / mDt * qDot;
    mTauHat.noalias() -= mJMInv[i] * (mSkeletons[i]->getInternalForceVector()
                                      + mSkeletons[i]->getExternalForceVector()
                                      + mSkeletons[i]->getDampingForceVector()
                                      - mSkeletons[i]->getCombinedVector());
  }
  mTauHat -= ks * mC + kd * mCDot;
}

int ConstraintDynamics::getTotalNumDofs() const
{
  return mIndices[mIndices.size() - 1];
}

}  // namespace constraint
}  // namespace dart
