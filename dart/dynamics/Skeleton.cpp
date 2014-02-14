/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
 *            Jeongseok Lee <jslee02@gmail.com>
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

#include "dart/dynamics/Skeleton.h"

#include <queue>
#include <string>
#include <vector>

#include "dart/math/Geometry.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/GenCoord.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/Marker.h"

namespace dart {
namespace dynamics {

Skeleton::Skeleton(const std::string& _name)
  : GenCoordSystem(),
    mName(_name),
    mIsSelfCollidable(false),
    mTimeStep(0.001),
    mGravity(Eigen::Vector3d(0.0, 0.0, -9.81)),
    mTotalMass(0.0),
    mIsMobile(true),
    mIsMassMatrixDirty(true),
    mIsAugMassMatrixDirty(true),
    mIsInvMassMatrixDirty(true),
    mIsInvAugMassMatrixDirty(true),
    mIsCoriolisVectorDirty(true),
    mIsGravityForceVectorDirty(true),
    mIsCombinedVectorDirty(true),
    mIsExternalForceVectorDirty(true),
    mIsDampingForceVectorDirty(true) {
}

Skeleton::~Skeleton() {
  for (std::vector<BodyNode*>::const_iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it)
    delete (*it);
}

void Skeleton::setName(const std::string& _name) {
  mName = _name;
}

const std::string& Skeleton::getName() const {
  return mName;
}

void Skeleton::setSelfCollidable(bool _isSelfCollidable) {
  mIsSelfCollidable = _isSelfCollidable;
}

bool Skeleton::isSelfCollidable() const {
  return mIsSelfCollidable;
}

void Skeleton::setMobile(bool _isMobile) {
  mIsMobile = _isMobile;
}

bool Skeleton::isMobile() const {
  return mIsMobile;
}

void Skeleton::setTimeStep(double _timeStep) {
  assert(_timeStep > 0.0);
  mTimeStep = _timeStep;
}

double Skeleton::getTimeStep() const {
  return mTimeStep;
}

void Skeleton::setGravity(const Eigen::Vector3d& _gravity) {
  mGravity = _gravity;
}

const Eigen::Vector3d& Skeleton::getGravity() const {
  return mGravity;
}

double Skeleton::getMass() const {
  return mTotalMass;
}

void Skeleton::init(double _timeStep, const Eigen::Vector3d& _gravity) {
  // Set timestep and gravity
  setTimeStep(_timeStep);
  setGravity(_gravity);

  // Rearrange the list of body nodes with BFS (Breadth First Search)
  std::queue<BodyNode*> queue;
  queue.push(mBodyNodes[0]);
  mBodyNodes.clear();
  while (!queue.empty())   {
    BodyNode* itBodyNode = queue.front();
    queue.pop();
    mBodyNodes.push_back(itBodyNode);
    for (int i = 0; i < itBodyNode->getNumChildBodyNodes(); ++i)
      queue.push(itBodyNode->getChildBodyNode(i));
  }

  // Initialize body nodes and generalized coordinates
  mGenCoords.clear();
  for (int i = 0; i < getNumBodyNodes(); ++i) {
    mBodyNodes[i]->aggregateGenCoords(&mGenCoords);
    mBodyNodes[i]->init(this, i);
    mBodyNodes[i]->updateTransform();
    mBodyNodes[i]->updateVelocity();
    mBodyNodes[i]->updateEta();
  }
  for (std::vector<BodyNode*>::reverse_iterator it = mBodyNodes.rbegin();
       it != mBodyNodes.rend(); ++it) {
    (*it)->updateArticulatedInertia(mTimeStep);
  }

  // Set dimension of dynamics quantities
  int dof = getNumGenCoords();
  mM    = Eigen::MatrixXd::Zero(dof, dof);
  mAugM = Eigen::MatrixXd::Zero(dof, dof);
  mInvM = Eigen::MatrixXd::Zero(dof, dof);
  mInvAugM = Eigen::MatrixXd::Zero(dof, dof);
  mCvec = Eigen::VectorXd::Zero(dof);
  mG    = Eigen::VectorXd::Zero(dof);
  mCg   = Eigen::VectorXd::Zero(dof);
  mFext = Eigen::VectorXd::Zero(dof);
  mFc   = Eigen::VectorXd::Zero(dof);
  mFd   = Eigen::VectorXd::Zero(dof);

  // Clear external/internal force
  clearExternalForceVector();
  clearInternalForceVector();

  // Calculate mass
  mTotalMass = 0.0;
  for (int i = 0; i < getNumBodyNodes(); i++)
    mTotalMass += getBodyNode(i)->getMass();
}

void Skeleton::addBodyNode(BodyNode* _body) {
  assert(_body && _body->getParentJoint());
  mBodyNodes.push_back(_body);
}

int Skeleton::getNumBodyNodes() const {
  return static_cast<int>(mBodyNodes.size());
}

BodyNode* Skeleton::getRootBodyNode() const {
  // We assume that the first element of body nodes is root.
  return mBodyNodes[0];
}

BodyNode* Skeleton::getBodyNode(int _idx) const {
  return mBodyNodes[_idx];
}

BodyNode* Skeleton::getBodyNode(const std::string& _name) const {
  assert(!_name.empty());

  for (std::vector<BodyNode*>::const_iterator itrBody = mBodyNodes.begin();
       itrBody != mBodyNodes.end(); ++itrBody) {
    if ((*itrBody)->getName() == _name)
      return *itrBody;
  }

  return NULL;
}

Joint* Skeleton::getJoint(int _idx) const {
  return mBodyNodes[_idx]->getParentJoint();
}

Joint* Skeleton::getJoint(const std::string& _name) const {
  assert(!_name.empty());

  for (std::vector<BodyNode*>::const_iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it) {
    if ((*it)->getParentJoint()->getName() == _name)
      return (*it)->getParentJoint();
  }

  return NULL;
}

Marker* Skeleton::getMarker(const std::string& _name) const {
  assert(!_name.empty());

  for (std::vector<BodyNode*>::const_iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it) {
    for (int i = 0; i < (*it)->getNumMarkers(); ++i) {
      if ((*it)->getMarker(i)->getName() == _name)
        return (*it)->getMarker(i);
    }
  }

  return NULL;
}

Eigen::VectorXd Skeleton::getConfig(const std::vector<int>& _id) const {
  Eigen::VectorXd q(_id.size());

  for (unsigned int i = 0; i < _id.size(); i++)
    q[i] = mGenCoords[_id[i]]->get_q();

  return q;
}

Eigen::VectorXd Skeleton::getConfig() const {
  return get_q();
}

void Skeleton::setConfig(const std::vector<int>& _id,
                         const Eigen::VectorXd& _config) {
  for ( unsigned int i = 0; i < _id.size(); i++ )
    mGenCoords[_id[i]]->set_q(_config(i));

  for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it) {
    (*it)->updateTransform();
    (*it)->updateVelocity();
    (*it)->updateEta();
  }

  for (std::vector<BodyNode*>::reverse_iterator it = mBodyNodes.rbegin();
       it != mBodyNodes.rend(); ++it) {
    (*it)->updateArticulatedInertia(mTimeStep);
  }

  mIsMassMatrixDirty = true;
  mIsAugMassMatrixDirty = true;
  mIsInvMassMatrixDirty = true;
  mIsInvAugMassMatrixDirty = true;
  mIsCoriolisVectorDirty = true;
  mIsGravityForceVectorDirty = true;
  mIsCombinedVectorDirty = true;
  mIsExternalForceVectorDirty = true;
  // mIsDampingForceVectorDirty = true;

  for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it) {
    (*it)->mIsBodyJacobianDirty = true;
    (*it)->mIsBodyJacobianTimeDerivDirty = true;
  }
}

void Skeleton::setConfig(const Eigen::VectorXd& _config) {
  set_q(_config);

  for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it) {
    (*it)->updateTransform();
    (*it)->updateVelocity();
    (*it)->updateEta();
  }

  for (std::vector<BodyNode*>::reverse_iterator it = mBodyNodes.rbegin();
       it != mBodyNodes.rend(); ++it) {
    (*it)->updateArticulatedInertia(mTimeStep);
  }

  mIsMassMatrixDirty = true;
  mIsAugMassMatrixDirty = true;
  mIsInvMassMatrixDirty = true;
  mIsInvAugMassMatrixDirty = true;
  mIsCoriolisVectorDirty = true;
  mIsGravityForceVectorDirty = true;
  mIsCombinedVectorDirty = true;
  mIsExternalForceVectorDirty = true;
  // mIsDampingForceVectorDirty = true;

  for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it) {
    (*it)->mIsBodyJacobianDirty = true;
    (*it)->mIsBodyJacobianTimeDerivDirty = true;
  }
}

void Skeleton::setState(const Eigen::VectorXd& _state) {
  set_q(_state.head(_state.size() / 2));
  set_dq(_state.tail(_state.size() / 2));

  for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it) {
    // TODO(JS): This is workaround for Issue #122.
    if ((*it)->getParentJoint()->getJointType() == Joint::BALL
        || (*it)->getParentJoint()->getJointType() == Joint::FREE) {
      (*it)->updateTransform_Issue122(mTimeStep);
      (*it)->updateVelocity();
      (*it)->updateEta_Issue122();
    } else {
      (*it)->updateTransform();
      (*it)->updateVelocity();
      (*it)->updateEta();
    }
  }

  for (std::vector<BodyNode*>::reverse_iterator it = mBodyNodes.rbegin();
       it != mBodyNodes.rend(); ++it) {
    (*it)->updateArticulatedInertia(mTimeStep);
  }

  mIsMassMatrixDirty = true;
  mIsAugMassMatrixDirty = true;
  mIsInvMassMatrixDirty = true;
  mIsInvAugMassMatrixDirty = true;
  mIsCoriolisVectorDirty = true;
  mIsGravityForceVectorDirty = true;
  mIsCombinedVectorDirty = true;
  mIsExternalForceVectorDirty = true;
  mIsDampingForceVectorDirty = true;

  for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it) {
    (*it)->mIsBodyJacobianDirty = true;
    (*it)->mIsBodyJacobianTimeDerivDirty = true;
  }
}

Eigen::VectorXd Skeleton::getState() {
  Eigen::VectorXd state(2 * mGenCoords.size());
  state << get_q(), get_dq();
  return state;
}

const Eigen::MatrixXd& Skeleton::getMassMatrix() {
  if (mIsMassMatrixDirty)
    updateMassMatrix();
  return mM;
}

const Eigen::MatrixXd& Skeleton::getAugMassMatrix() {
  if (mIsAugMassMatrixDirty)
    updateAugMassMatrix();
  return mAugM;
}

const Eigen::MatrixXd& Skeleton::getInvMassMatrix() {
  if (mIsInvMassMatrixDirty)
    updateInvMassMatrix();
  return mInvM;
}

const Eigen::MatrixXd& Skeleton::getInvAugMassMatrix() {
  if (mIsInvAugMassMatrixDirty)
    updateInvAugMassMatrix();
  return mInvAugM;
}

const Eigen::VectorXd& Skeleton::getCoriolisForceVector() {
  if (mIsCoriolisVectorDirty)
    updateCoriolisForceVector();
  return mCvec;
}

const Eigen::VectorXd& Skeleton::getGravityForceVector() {
  if (mIsGravityForceVectorDirty)
    updateGravityForceVector();
  return mG;
}

const Eigen::VectorXd& Skeleton::getCombinedVector() {
  if (mIsCombinedVectorDirty)
    updateCombinedVector();
  return mCg;
}

const Eigen::VectorXd& Skeleton::getExternalForceVector() {
  if (mIsExternalForceVectorDirty)
    updateExternalForceVector();
  return mFext;
}

Eigen::VectorXd Skeleton::getInternalForceVector() const {
  return get_tau();
}

const Eigen::VectorXd& Skeleton::getDampingForceVector() {
  if (mIsDampingForceVectorDirty)
    updateDampingForceVector();
  return mFd;
}

const Eigen::VectorXd& Skeleton::getConstraintForceVector() {
  return mFc;
}

void Skeleton::draw(renderer::RenderInterface* _ri,
                    const Eigen::Vector4d& _color,
                    bool _useDefaultColor) const {
  getRootBodyNode()->draw(_ri, _color, _useDefaultColor);
}

void Skeleton::drawMarkers(renderer::RenderInterface* _ri,
                           const Eigen::Vector4d& _color,
                           bool _useDefaultColor) const {
  getRootBodyNode()->drawMarkers(_ri, _color, _useDefaultColor);
}

void Skeleton::updateMassMatrix() {
  assert(mM.cols() == getNumGenCoords() && mM.rows() == getNumGenCoords());
  assert(getNumGenCoords() > 0);

  mM.setZero();

  // Backup the origianl internal force
  Eigen::VectorXd originalGenAcceleration = get_ddq();

  int dof = getNumGenCoords();
  Eigen::VectorXd e = Eigen::VectorXd::Zero(dof);
  for (int j = 0; j < dof; ++j) {
    e[j] = 1.0;
    set_ddq(e);

    // Prepare cache data
    for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
         it != mBodyNodes.end(); ++it) {
      (*it)->updateMassMatrix();
    }

    // Mass matrix
    //    for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
    //         it != mBodyNodes.end(); ++it)
    for (int i = mBodyNodes.size() - 1; i > -1 ; --i) {
      mBodyNodes[i]->aggregateMassMatrix(&mM, j);
      int localDof = mBodyNodes[i]->mParentJoint->getNumGenCoords();
      if (localDof > 0) {
        int iStart =
            mBodyNodes[i]->mParentJoint->getGenCoord(0)->getSkeletonIndex();
        if (iStart + localDof < j)
          break;
      }
    }

    e[j] = 0.0;
  }
  mM.triangularView<Eigen::StrictlyUpper>() = mM.transpose();

  // Restore the origianl internal force
  set_ddq(originalGenAcceleration);

  mIsMassMatrixDirty = false;
}

void Skeleton::updateAugMassMatrix() {
  assert(mAugM.cols() == getNumGenCoords() && mAugM.rows() == getNumGenCoords());
  assert(getNumGenCoords() > 0);

  mAugM.setZero();

  // Backup the origianl internal force
  Eigen::VectorXd originalGenAcceleration = get_ddq();

  int dof = getNumGenCoords();
  Eigen::VectorXd e = Eigen::VectorXd::Zero(dof);
  for (int j = 0; j < dof; ++j) {
    e[j] = 1.0;
    set_ddq(e);

    // Prepare cache data
    for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
         it != mBodyNodes.end(); ++it) {
      (*it)->updateMassMatrix();
    }

    // Mass matrix
    //    for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
    //         it != mBodyNodes.end(); ++it)
    for (int i = mBodyNodes.size() - 1; i > -1 ; --i) {
      mBodyNodes[i]->aggregateAugMassMatrix(&mAugM, j, mTimeStep);
      int localDof = mBodyNodes[i]->mParentJoint->getNumGenCoords();
      if (localDof > 0) {
        int iStart =
            mBodyNodes[i]->mParentJoint->getGenCoord(0)->getSkeletonIndex();
        if (iStart + localDof < j)
          break;
      }
    }

    e[j] = 0.0;
  }
  mAugM.triangularView<Eigen::StrictlyUpper>() = mAugM.transpose();

  // Restore the origianl internal force
  set_ddq(originalGenAcceleration);

  mIsAugMassMatrixDirty = false;
}

void Skeleton::updateInvMassMatrix() {
  assert(mInvM.cols() == getNumGenCoords() &&
         mInvM.rows() == getNumGenCoords());
  assert(getNumGenCoords() > 0);

  // We don't need to set mInvM as zero matrix as long as the below is correct
  // mInvM.setZero();

  // Backup the origianl internal force
  Eigen::VectorXd originalInternalForce = get_tau();

  int dof = getNumGenCoords();
  Eigen::VectorXd e = Eigen::VectorXd::Zero(dof);
  for (int j = 0; j < dof; ++j) {
    e[j] = 1.0;
    set_tau(e);

    // Prepare cache data
    for (std::vector<BodyNode*>::reverse_iterator it = mBodyNodes.rbegin();
         it != mBodyNodes.rend(); ++it) {
      (*it)->updateInvMassMatrix();
    }

    // Inverse of mass matrix
    //    for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
    //         it != mBodyNodes.end(); ++it)
    for (int i = 0; i < mBodyNodes.size(); ++i) {
      mBodyNodes[i]->aggregateInvMassMatrix(&mInvM, j);
      int localDof = mBodyNodes[i]->mParentJoint->getNumGenCoords();
      if (localDof > 0) {
        int iStart =
            mBodyNodes[i]->mParentJoint->getGenCoord(0)->getSkeletonIndex();
        if (iStart + localDof > j)
          break;
      }
    }

    e[j] = 0.0;
  }
  mInvM.triangularView<Eigen::StrictlyLower>() = mInvM.transpose();

  // Restore the origianl internal force
  set_tau(originalInternalForce);

  mIsInvMassMatrixDirty = false;
}

void Skeleton::updateInvAugMassMatrix() {
  assert(mInvAugM.cols() == getNumGenCoords() &&
         mInvAugM.rows() == getNumGenCoords());
  assert(getNumGenCoords() > 0);

  // We don't need to set mInvM as zero matrix as long as the below is correct
  // mInvM.setZero();

  // Backup the origianl internal force
  Eigen::VectorXd originalInternalForce = get_tau();

  int dof = getNumGenCoords();
  Eigen::VectorXd e = Eigen::VectorXd::Zero(dof);
  for (int j = 0; j < dof; ++j) {
    e[j] = 1.0;
    set_tau(e);

    // Prepare cache data
    for (std::vector<BodyNode*>::reverse_iterator it = mBodyNodes.rbegin();
         it != mBodyNodes.rend(); ++it) {
      (*it)->updateInvAugMassMatrix();
    }

    // Inverse of mass matrix
    //    for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
    //         it != mBodyNodes.end(); ++it)
    for (int i = 0; i < mBodyNodes.size(); ++i) {
      mBodyNodes[i]->aggregateInvAugMassMatrix(&mInvAugM, j, mTimeStep);
      int localDof = mBodyNodes[i]->mParentJoint->getNumGenCoords();
      if (localDof > 0) {
        int iStart =
            mBodyNodes[i]->mParentJoint->getGenCoord(0)->getSkeletonIndex();
        if (iStart + localDof > j)
          break;
      }
    }

    e[j] = 0.0;
  }
  mInvAugM.triangularView<Eigen::StrictlyLower>() = mInvAugM.transpose();

  // Restore the origianl internal force
  set_tau(originalInternalForce);

  mIsInvAugMassMatrixDirty = false;
}

void Skeleton::updateCoriolisForceVector() {
  assert(mCvec.size() == getNumGenCoords());
  assert(getNumGenCoords() > 0);

  mCvec.setZero();
  for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it) {
    (*it)->updateCombinedVector();
  }
  for (std::vector<BodyNode*>::reverse_iterator it = mBodyNodes.rbegin();
       it != mBodyNodes.rend(); ++it) {
    (*it)->aggregateCoriolisForceVector(&mCvec);
  }

  mIsCoriolisVectorDirty = false;
}

void Skeleton::updateGravityForceVector() {
  assert(mG.size() == getNumGenCoords());
  assert(getNumGenCoords() > 0);

  // Calcualtion mass matrix, M
  mG.setZero();
  for (std::vector<BodyNode*>::reverse_iterator it = mBodyNodes.rbegin();
       it != mBodyNodes.rend(); ++it) {
    (*it)->aggregateGravityForceVector(&mG, mGravity);
  }

  mIsGravityForceVectorDirty = false;
}

void Skeleton::updateCombinedVector() {
  assert(mCg.size() == getNumGenCoords());
  assert(getNumGenCoords() > 0);

  mCg.setZero();
  for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it) {
    (*it)->updateCombinedVector();
  }
  for (std::vector<BodyNode*>::reverse_iterator it = mBodyNodes.rbegin();
       it != mBodyNodes.rend(); ++it) {
    (*it)->aggregateCombinedVector(&mCg, mGravity);
  }

  mIsCombinedVectorDirty = false;
}

void Skeleton::updateExternalForceVector() {
  assert(mFext.size() == getNumGenCoords());
  assert(getNumGenCoords() > 0);

  // Clear external force.
  mFext.setZero();
  for (std::vector<BodyNode*>::reverse_iterator itr = mBodyNodes.rbegin();
       itr != mBodyNodes.rend(); ++itr)
    (*itr)->aggregateExternalForces(&mFext);

  mIsExternalForceVectorDirty = false;
}

void Skeleton::updateDampingForceVector() {
  assert(mFd.size() == getNumGenCoords());
  assert(getNumGenCoords() > 0);

  // Clear external force.
  mFd.setZero();

  for (std::vector<BodyNode*>::iterator itr = mBodyNodes.begin();
       itr != mBodyNodes.end(); ++itr) {
    Eigen::VectorXd jointDampingForce =
        (*itr)->getParentJoint()->getDampingForces();
    for (int i = 0; i < jointDampingForce.size(); i++) {
      mFd((*itr)->getParentJoint()->getGenCoord(i)->getSkeletonIndex()) =
          jointDampingForce(i);
    }
  }
}

void Skeleton::computeInverseDynamicsLinear(bool _computeJacobian,
                                            bool _computeJacobianDeriv,
                                            bool _withExternalForces,
                                            bool _withDampingForces) {
  // Skip immobile or 0-dof skeleton
  if (getNumGenCoords() == 0)
    return;

  // Forward recursion
  for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it) {
    (*it)->updateAcceleration();
  }

  // Backward recursion
  for (std::vector<BodyNode*>::reverse_iterator it = mBodyNodes.rbegin();
       it != mBodyNodes.rend(); ++it) {
    (*it)->updateBodyForce(mGravity, _withExternalForces);
    (*it)->updateGeneralizedForce(_withDampingForces);
  }
}

void Skeleton::clearExternalForceVector() {
  for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it) {
    (*it)->clearExternalForces();
  }
}

void Skeleton::computeForwardDynamics() {
  // Skip immobile or 0-dof skeleton
  if (!isMobile() || getNumGenCoords() == 0)
    return;

  // Backward recursion
  for (std::vector<BodyNode*>::reverse_iterator it = mBodyNodes.rbegin();
       it != mBodyNodes.rend(); ++it) {
    (*it)->updateBiasForce(mTimeStep, mGravity);
  }

  // Forward recursion
  for (std::vector<BodyNode*>::iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it) {
    (*it)->update_ddq();
    (*it)->update_F_fs();
  }
}

void Skeleton::setInternalForceVector(const Eigen::VectorXd& _forces) {
  set_tau(_forces);
}

void Skeleton::setMinInternalForceVector(const Eigen::VectorXd& _minForces) {
  set_tauMin(_minForces);
}

Eigen::VectorXd Skeleton::getMinInternalForces() const {
  return get_tauMin();
}

void Skeleton::setMaxInternalForceVector(const Eigen::VectorXd& _maxForces) {
  set_tauMax(_maxForces);
}

Eigen::VectorXd Skeleton::getMaxInternalForceVector() const {
  return get_tauMax();
}

void Skeleton::clearInternalForceVector() {
  set_tau(Eigen::VectorXd::Zero(getNumGenCoords()));
}

void Skeleton::setConstraintForceVector(const Eigen::VectorXd& _Fc) {
  mFc = _Fc;
}

Eigen::Vector3d Skeleton::getWorldCOM() {
  // COM
  Eigen::Vector3d com(0.0, 0.0, 0.0);

  // Compute sum of each body's COM multiplied by body's mass
  const int nNodes = getNumBodyNodes();
  for (int i = 0; i < nNodes; i++) {
    BodyNode* bodyNode = getBodyNode(i);
    com += bodyNode->getMass() * bodyNode->getWorldCOM();
  }

  // Divide the sum by the total mass
  assert(mTotalMass != 0.0);
  return com / mTotalMass;
}

Eigen::Vector3d Skeleton::getWorldCOMVelocity() {
  // Velocity of COM
  Eigen::Vector3d comVel(0.0, 0.0, 0.0);

  // Compute sum of each body's COM velocities multiplied by body's mass
  const int nNodes = getNumBodyNodes();
  for (int i = 0; i < nNodes; i++) {
    BodyNode* bodyNode = getBodyNode(i);
    comVel += bodyNode->getMass() * bodyNode->getWorldCOMVelocity();
  }

  // Divide the sum by the total mass
  assert(mTotalMass != 0.0);
  return comVel / mTotalMass;
}

Eigen::Vector3d Skeleton::getWorldCOMAcceleration() {
  // Acceleration of COM
  Eigen::Vector3d comAcc(0.0, 0.0, 0.0);

  // Compute sum of each body's COM accelerations multiplied by body's mass
  const int nNodes = getNumBodyNodes();
  for (int i = 0; i < nNodes; i++) {
    BodyNode* bodyNode = getBodyNode(i);
    comAcc += bodyNode->getMass() * bodyNode->getWorldCOMAcceleration();
  }

  // Divide the sum by the total mass
  assert(mTotalMass != 0.0);
  return comAcc / mTotalMass;
}

Eigen::MatrixXd Skeleton::getWorldCOMJacobian() {
  // Jacobian of COM
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, getNumGenCoords());

  // Compute sum of each body's Jacobian of COM accelerations multiplied by
  // body's mass
  const int nNodes = getNumBodyNodes();
  for (int i = 0; i < nNodes; i++) {
    // BodyNode iterator
    BodyNode* bodyNode = getBodyNode(i);

    // Compute weighted Jacobian
    Eigen::MatrixXd localJ
        = bodyNode->getMass()
          * bodyNode->getWorldJacobian(
              bodyNode->getLocalCOM(), true).bottomRows<3>();

    // Assign the weighted Jacobian to total Jacobian
    for (int j = 0; j < bodyNode->getNumDependentGenCoords(); ++j) {
      int idx = bodyNode->getDependentGenCoordIndex(j);
      J.col(idx) += localJ.col(j);
    }
  }

  // Divide the sum by the total mass
  assert(mTotalMass != 0.0);
  return J / mTotalMass;
}

Eigen::MatrixXd Skeleton::getWorldCOMJacobianTimeDeriv() {
  // Jacobian time derivative of COM
  Eigen::MatrixXd dJ = Eigen::MatrixXd::Zero(3, getNumGenCoords());

  // Compute sum of each body's Jacobian time derivative of COM accelerations
  // multiplied by body's mass
  const int nNodes = getNumBodyNodes();
  for (int i = 0; i < nNodes; i++) {
    // BodyNode iterator
    BodyNode* bodyNode = getBodyNode(i);

    // Compute weighted Jacobian time derivative
    Eigen::MatrixXd localJ
        = bodyNode->getMass()
          * bodyNode->getWorldJacobianTimeDeriv(bodyNode->getLocalCOM(), true).bottomRows<3>();

    // Assign the weighted Jacobian to total Jacobian time derivative
    for (int j = 0; j < bodyNode->getNumDependentGenCoords(); ++j) {
      int idx = bodyNode->getDependentGenCoordIndex(j);
      dJ.col(idx) += localJ.col(j);
    }
  }

  // Divide the sum by the total mass
  assert(mTotalMass != 0.0);
  return dJ / mTotalMass;
}

double Skeleton::getKineticEnergy() const {
  double KE = 0.0;

  for (std::vector<BodyNode*>::const_iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it)
  {
    KE += (*it)->getKineticEnergy();
  }

  assert(KE >= 0.0 && "Kinetic energy should be positive value.");
  return KE;
}

double Skeleton::getPotentialEnergy() const {
  double PE = 0.0;

  for (std::vector<BodyNode*>::const_iterator it = mBodyNodes.begin();
       it != mBodyNodes.end(); ++it)
  {
    PE += (*it)->getPotentialEnergy(mGravity);
    PE += (*it)->getParentJoint()->getPotentialEnergy();
  }

  return PE;
}

}  // namespace dynamics
}  // namespace dart
