/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
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

#include "dart/dynamics/BodyNode.h"

#include <algorithm>
#include <vector>
#include <string>

#include "dart/common/Console.h"
#include "dart/math/Helpers.h"
#include "dart/renderer/RenderInterface.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/Shape.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/Marker.h"

namespace dart {
namespace dynamics {

int BodyNode::msBodyNodeCount = 0;

BodyNode::BodyNode(const std::string& _name)
  : mSkelIndex(-1),
    mName(_name),
    mIsCollidable(true),
    mIsColliding(false),
    mSkeleton(NULL),
    mParentJoint(NULL),
    mParentBodyNode(NULL),
    mChildBodyNodes(std::vector<BodyNode*>(0)),
    mGravityMode(true),
    mCenterOfMass(Eigen::Vector3d::Zero()),
    mMass(1.0),
    mIxx(1.0),
    mIyy(1.0),
    mIzz(1.0),
    mIxy(0.0),
    mIxz(0.0),
    mIyz(0.0),
    mI(Eigen::Matrix6d::Identity()),
    mW(Eigen::Isometry3d::Identity()),
    mV(Eigen::Vector6d::Zero()),
    mEta(Eigen::Vector6d::Zero()),
    mdV(Eigen::Vector6d::Zero()),
    mF(Eigen::Vector6d::Zero()),
    mFext(Eigen::Vector6d::Zero()),
    mFgravity(Eigen::Vector6d::Zero()),
    mAI(Eigen::Matrix6d::Identity()),
    mB(Eigen::Vector6d::Zero()),
    mBeta(Eigen::Vector6d::Zero()),
    mID(BodyNode::msBodyNodeCount++),
    mIsBodyJacobianDirty(true),
    mIsBodyJacobianTimeDerivDirty(true) {
}

BodyNode::~BodyNode() {
  for (std::vector<Shape*>::const_iterator it = mVizShapes.begin();
       it != mVizShapes.end(); ++it)
    delete (*it);

  for (std::vector<Shape*>::const_iterator itColShape = mColShapes.begin();
       itColShape != mColShapes.end(); ++itColShape)
    if (mVizShapes.end() == std::find(mVizShapes.begin(), mVizShapes.end(),
                                      *itColShape))
      delete (*itColShape);

  for (std::vector<Marker*>::const_iterator it = mMarkers.begin();
       it != mMarkers.end(); ++it)
    delete (*it);

  delete mParentJoint;
}

void BodyNode::setName(const std::string& _name) {
  mName = _name;
}

const std::string& BodyNode::getName() const {
  return mName;
}

void BodyNode::setGravityMode(bool _gravityMode) {
  mGravityMode = _gravityMode;
}

bool BodyNode::getGravityMode() const {
  return mGravityMode;
}

bool BodyNode::isCollidable() const {
  return mIsCollidable;
}

void BodyNode::setCollidable(bool _isCollidable) {
  mIsCollidable = _isCollidable;
}

void BodyNode::setMass(double _mass) {
  assert(_mass >= 0.0 && "Negative mass is not allowable.");
  mMass = _mass;
  _updateGeralizedInertia();
}

double BodyNode::getMass() const {
  return mMass;
}

BodyNode* BodyNode::getParentBodyNode() const {
  return mParentBodyNode;
}

void BodyNode::addChildBodyNode(BodyNode* _body) {
  assert(_body != NULL);
  mChildBodyNodes.push_back(_body);
  _body->mParentBodyNode = this;
}

BodyNode* BodyNode::getChildBodyNode(int _idx) const {
  assert(0 <= _idx && _idx < mChildBodyNodes.size());
  return mChildBodyNodes[_idx];
}

int BodyNode::getNumChildBodyNodes() const {
  return mChildBodyNodes.size();
}

void BodyNode::addMarker(Marker* _marker) {
  mMarkers.push_back(_marker);
}

int BodyNode::getNumMarkers() const {
  return mMarkers.size();
}

Marker* BodyNode::getMarker(int _idx) const {
  return mMarkers[_idx];
}

bool BodyNode::dependsOn(int _genCoordIndex) const {
  return std::binary_search(mDependentGenCoordIndices.begin(),
                            mDependentGenCoordIndices.end(),
                            _genCoordIndex);
}

int BodyNode::getNumDependentGenCoords() const {
  return mDependentGenCoordIndices.size();
}

int BodyNode::getDependentGenCoordIndex(int _arrayIndex) const {
  assert(0 <= _arrayIndex && _arrayIndex < mDependentGenCoordIndices.size());
  return mDependentGenCoordIndices[_arrayIndex];
}

const Eigen::Isometry3d& BodyNode::getWorldTransform() const {
  return mW;
}

const Eigen::Vector6d& BodyNode::getBodyVelocity() const {
  return mV;
}

Eigen::Vector6d BodyNode::getWorldVelocity(
    const Eigen::Vector3d& _offset, bool _isLocal) const {
  Eigen::Isometry3d T = mW;
  if (_isLocal)
    T.translation() = mW.linear() * -_offset;
  else
    T.translation() = -_offset;
  return math::AdT(T, mV);
}

const Eigen::Vector6d& BodyNode::getBodyAcceleration() const {
  return mdV;
}

Eigen::Vector6d BodyNode::getWorldAcceleration(
    const Eigen::Vector3d& _offset, bool _isOffsetLocal) const {
  Eigen::Isometry3d T = mW;
  if (_isOffsetLocal)
    T.translation() = mW.linear() * -_offset;
  else
    T.translation() = -_offset;

  Eigen::Vector6d dV = mdV;
  dV.tail<3>() += mV.head<3>().cross(mV.tail<3>());

  return math::AdT(T, dV);
}

const math::Jacobian& BodyNode::getBodyJacobian() {
  if (mIsBodyJacobianDirty)
    _updateBodyJacobian();
  return mBodyJacobian;
}

math::Jacobian BodyNode::getWorldJacobian(
    const Eigen::Vector3d& _offset, bool _isOffsetLocal) {
  Eigen::Isometry3d T = mW;
  if (_isOffsetLocal)
    T.translation() = mW.linear() * -_offset;
  else
    T.translation() = -_offset;
  return math::AdTJac(T, getBodyJacobian());
}

const math::Jacobian& BodyNode::getBodyJacobianTimeDeriv() {
  if (mIsBodyJacobianTimeDerivDirty)
    _updateBodyJacobianTimeDeriv();
  return mBodyJacobianTimeDeriv;
}

math::Jacobian BodyNode::getWorldJacobianTimeDeriv(
    const Eigen::Vector3d& _offset, bool _isOffsetLocal) {
  Eigen::Isometry3d T = mW;
  if (_isOffsetLocal)
    T.translation() = mW.linear() * -_offset;
  else
    T.translation() = -_offset;

  math::Jacobian bodyJacobianTimeDeriv = getBodyJacobianTimeDeriv();
  for (int i = 0; i < mBodyJacobianTimeDeriv.cols(); ++i)
  {
    bodyJacobianTimeDeriv.col(i).tail<3>()
        += mV.head<3>().cross(mBodyJacobian.col(i).tail<3>());
  }

  return math::AdTJac(T, bodyJacobianTimeDeriv);
}

void BodyNode::setColliding(bool _isColliding) {
  mIsColliding = _isColliding;
}

bool BodyNode::isColliding() {
  return mIsColliding;
}

void BodyNode::init(Skeleton* _skeleton, int _skeletonIndex) {
  assert(_skeleton);

  mSkeleton = _skeleton;
  mSkelIndex = _skeletonIndex;
  mParentJoint->mSkelIndex = _skeletonIndex;

  //--------------------------------------------------------------------------
  // Fill the list of generalized coordinates this node depends on, and sort
  // it.
  //--------------------------------------------------------------------------
  if (mParentBodyNode)
    mDependentGenCoordIndices = mParentBodyNode->mDependentGenCoordIndices;
  else
    mDependentGenCoordIndices.clear();
  for (int i = 0; i < mParentJoint->getNumGenCoords(); i++)
    mDependentGenCoordIndices.push_back(
          mParentJoint->getGenCoord(i)->getSkeletonIndex());
  std::sort(mDependentGenCoordIndices.begin(), mDependentGenCoordIndices.end());

#ifndef NDEBUG
  // Check whether there is duplicated indices.
  int nDepGenCoordIndices = mDependentGenCoordIndices.size();
  for (int i = 0; i < nDepGenCoordIndices - 1; i++) {
    for (int j = i + 1; j < nDepGenCoordIndices; j++) {
      assert(mDependentGenCoordIndices[i] !=
          mDependentGenCoordIndices[j] &&
          "Duplicated index is found in mDependentGenCoordIndices.");
    }
  }
#endif

  //--------------------------------------------------------------------------
  // Set dimensions of dynamics matrices and vectors.
  //--------------------------------------------------------------------------
  int numDepGenCoords = getNumDependentGenCoords();
  mBodyJacobian.setZero(6, numDepGenCoords);
  mBodyJacobianTimeDeriv.setZero(6, numDepGenCoords);

  //--------------------------------------------------------------------------
  // Set dimensions of cache data for recursive algorithms
  //--------------------------------------------------------------------------
  int dof = mParentJoint->getNumGenCoords();
  mAI_S.setZero(6, dof);
  mPsi.setZero(dof, dof);
  mImplicitPsi.setZero(dof, dof);
  mAlpha.setZero(dof);
}

void BodyNode::aggregateGenCoords(std::vector<GenCoord*>* _genCoords) {
  assert(mParentJoint);
  for (int i = 0; i < mParentJoint->getNumGenCoords(); ++i) {
    mParentJoint->getGenCoord(i)->setSkeletonIndex(_genCoords->size());
    _genCoords->push_back(mParentJoint->getGenCoord(i));
  }
}

void BodyNode::draw(renderer::RenderInterface* _ri,
                    const Eigen::Vector4d& _color,
                    bool _useDefaultColor,
                    int _depth) const {
  if (_ri == NULL)
    return;

  _ri->pushMatrix();

  // render the self geometry
  mParentJoint->applyGLTransform(_ri);

  _ri->pushName((unsigned)mID);
  for (int i = 0; i < mVizShapes.size(); i++) {
    _ri->pushMatrix();
    mVizShapes[i]->draw(_ri, _color, _useDefaultColor);
    _ri->popMatrix();
  }
  _ri->popName();

  // render the subtree
  for (unsigned int i = 0; i < mChildBodyNodes.size(); i++) {
    mChildBodyNodes[i]->draw(_ri, _color, _useDefaultColor);
  }

  _ri->popMatrix();
}

void BodyNode::drawMarkers(renderer::RenderInterface* _ri,
                           const Eigen::Vector4d& _color,
                           bool _useDefaultColor) const {
  if (!_ri)
    return;

  _ri->pushMatrix();

  mParentJoint->applyGLTransform(_ri);

  // render the corresponding mMarkerss
  for (unsigned int i = 0; i < mMarkers.size(); i++)
    mMarkers[i]->draw(_ri, true, _color, _useDefaultColor);

  for (unsigned int i = 0; i < mChildBodyNodes.size(); i++)
    mChildBodyNodes[i]->drawMarkers(_ri, _color, _useDefaultColor);

  _ri->popMatrix();
}

void BodyNode::updateTransform() {
  mParentJoint->updateTransform();
  if (mParentBodyNode) {
    mW = mParentBodyNode->getWorldTransform()
         * mParentJoint->getLocalTransform();
  } else {
    mW = mParentJoint->getLocalTransform();
  }
  assert(math::verifyTransform(mW));

  mParentJoint->updateJacobian();
}

void BodyNode::updateTransform_Issue122(double _timeStep) {
  mParentJoint->updateTransform_Issue122(_timeStep);
  if (mParentBodyNode) {
    mW = mParentBodyNode->getWorldTransform()
         * mParentJoint->getLocalTransform();
  } else {
    mW = mParentJoint->getLocalTransform();
  }
  assert(math::verifyTransform(mW));

  mParentJoint->updateJacobian_Issue122();
}

void BodyNode::updateVelocity() {
  //--------------------------------------------------------------------------
  // Body velocity update
  //
  // V(i) = Ad(T(i, i-1), V(i-1)) + S * dq
  //--------------------------------------------------------------------------

  if (mParentJoint->getNumGenCoords() > 0) {
    mV.noalias() = mParentJoint->getLocalJacobian() * mParentJoint->get_dq();
    if (mParentBodyNode) {
      mV += math::AdInvT(mParentJoint->getLocalTransform(),
                         mParentBodyNode->getBodyVelocity());
    }
  }

  assert(!math::isNan(mV));
}

void BodyNode::updateEta() {
  mParentJoint->updateJacobianTimeDeriv();

  if (mParentJoint->getNumGenCoords() > 0) {
    mEta = math::ad(mV, mParentJoint->getLocalJacobian() *
                    mParentJoint->get_dq());
    mEta.noalias() += mParentJoint->getLocalJacobianTimeDeriv() *
                      mParentJoint->get_dq();
    assert(!math::isNan(mEta));
  }
}

void BodyNode::updateEta_Issue122() {
  mParentJoint->updateJacobianTimeDeriv_Issue122();

  if (mParentJoint->getNumGenCoords() > 0) {
    mEta = math::ad(mV, mParentJoint->getLocalJacobian() *
                    mParentJoint->get_dq());
    mEta.noalias() += mParentJoint->getLocalJacobianTimeDeriv() *
                      mParentJoint->get_dq();
    assert(!math::isNan(mEta));
  }
}

void BodyNode::updateAcceleration() {
  // dV(i) = Ad(T(i, i-1), dV(i-1))
  //         + ad(V(i), S * dq) + dS * dq
  //         + S * ddq
  //       = Ad(T(i, i-1), dV(i-1))
  //         + eta
  //         + S * ddq

  if (mParentJoint->getNumGenCoords() > 0) {
    mdV = mEta;
    mdV.noalias() += mParentJoint->getLocalJacobian() * mParentJoint->get_ddq();
    if (mParentBodyNode) {
      mdV += math::AdInvT(mParentJoint->getLocalTransform(),
                          mParentBodyNode->getBodyAcceleration());
    }
  }

  assert(!math::isNan(mdV));
}

void BodyNode::setInertia(double _Ixx, double _Iyy, double _Izz,
                          double _Ixy, double _Ixz, double _Iyz) {
  assert(_Ixx >= 0.0);
  assert(_Iyy >= 0.0);
  assert(_Izz >= 0.0);

  mIxx = _Ixx;
  mIyy = _Iyy;
  mIzz = _Izz;

  mIxy = _Ixy;
  mIxz = _Ixz;
  mIyz = _Iyz;

  _updateGeralizedInertia();
}

void BodyNode::setLocalCOM(const Eigen::Vector3d& _com) {
  mCenterOfMass = _com;
  _updateGeralizedInertia();
}

const Eigen::Vector3d& BodyNode::getLocalCOM() const {
  return mCenterOfMass;
}

Eigen::Vector3d BodyNode::getWorldCOM() const {
  return mW * mCenterOfMass;
}

Eigen::Vector3d BodyNode::getWorldCOMVelocity() const {
  return getWorldVelocity(mCenterOfMass, true).tail<3>();
}

Eigen::Vector3d BodyNode::getWorldCOMAcceleration() const {
  return getWorldAcceleration(mCenterOfMass, true).tail<3>();
}

Eigen::Matrix6d BodyNode::getInertia() const {
  return mI;
}

int BodyNode::getSkeletonIndex() const {
  return mSkelIndex;
}

void BodyNode::addVisualizationShape(Shape* _p) {
  mVizShapes.push_back(_p);
}

int BodyNode::getNumVisualizationShapes() const {
  return mVizShapes.size();
}

Shape*BodyNode::getVisualizationShape(int _idx) const {
  return mVizShapes[_idx];
}

void BodyNode::addCollisionShape(Shape* _p) {
  mColShapes.push_back(_p);
}

int BodyNode::getNumCollisionShapes() const {
  return mColShapes.size();
}

Shape*BodyNode::getCollisionShape(int _idx) const {
  return mColShapes[_idx];
}

Skeleton*BodyNode::getSkeleton() const {
  return mSkeleton;
}

void BodyNode::setParentJoint(Joint* _joint) {
  mParentJoint = _joint;
}

Joint*BodyNode::getParentJoint() const {
  return mParentJoint;
}

void BodyNode::addExtForce(const Eigen::Vector3d& _force,
                           const Eigen::Vector3d& _offset,
                           bool _isOffsetLocal, bool _isForceLocal) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  Eigen::Vector6d F = Eigen::Vector6d::Zero();

  if (_isOffsetLocal)
    T.translation() = _offset;
  else
    T.translation() = getWorldTransform().inverse() * _offset;

  if (_isForceLocal)
    F.tail<3>() = _force;
  else
    F.tail<3>() = mW.linear().transpose() * _force;

  mFext += math::dAdInvT(T, F);
}

void BodyNode::setExtForce(const Eigen::Vector3d& _force,
                           const Eigen::Vector3d& _offset,
                           bool _isOffsetLocal, bool _isForceLocal) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  Eigen::Vector6d F = Eigen::Vector6d::Zero();

  if (_isOffsetLocal)
    T.translation() = _offset;
  else
    T.translation() = getWorldTransform().inverse() * _offset;

  if (_isForceLocal)
    F.tail<3>() = _force;
  else
    F.tail<3>() = mW.linear().transpose() * _force;

  mFext = math::dAdInvT(T, F);
}

void BodyNode::addExtTorque(const Eigen::Vector3d& _torque, bool _isLocal) {
  if (_isLocal)
    mFext.head<3>() += _torque;
  else
    mFext.head<3>() += mW.linear() * _torque;
}

void BodyNode::setExtTorque(const Eigen::Vector3d& _torque, bool _isLocal) {
  if (_isLocal)
    mFext.head<3>() = _torque;
  else
    mFext.head<3>() = mW.linear() * _torque;
}

const Eigen::Vector6d& BodyNode::getExternalForceLocal() const {
  return mFext;
}

Eigen::Vector6d BodyNode::getExternalForceGlobal() const {
  return math::dAdInvT(mW, mFext);
}

void BodyNode::addContactForce(const Eigen::Vector6d& _contactForce) {
  mContactForces.push_back(_contactForce);
}

int BodyNode::getNumContactForces() const {
  return mContactForces.size();
}

const Eigen::Vector6d& BodyNode::getContactForce(int _idx) {
  assert(0 <= _idx && _idx < mContactForces.size());
  return mContactForces[_idx];
}

void BodyNode::clearContactForces() {
  mContactForces.clear();
}

const Eigen::Vector6d& BodyNode::getBodyForce() const {
  return mF;
}

double BodyNode::getKineticEnergy() const {
  return 0.5 * mV.dot(mI * mV);
}

double dart::dynamics::BodyNode::getPotentialEnergy(
    const Eigen::Vector3d& _gravity) const {
  return -mMass * mW.translation().dot(_gravity);
}

Eigen::Vector3d BodyNode::getLinearMomentum() const {
  return (mI * mV).tail<3>();
}

Eigen::Vector3d BodyNode::getAngularMomentum(const Eigen::Vector3d& _pivot) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = _pivot;
  return math::dAdT(T, mI * mV).head<3>();
}

void BodyNode::updateBodyForce(const Eigen::Vector3d& _gravity,
                               bool _withExternalForces) {
  if (mGravityMode == true)
    mFgravity.noalias() = mI * math::AdInvRLinear(mW, _gravity);
  else
    mFgravity.setZero();

  mF.noalias() = mI * mdV;       // Inertial force
  if (_withExternalForces)
    mF -= mFext;                 // External force
  mF -= mFgravity;               // Gravity force
  mF -= math::dad(mV, mI * mV);  // Coriolis force

  for (std::vector<BodyNode*>::iterator iChildBody = mChildBodyNodes.begin();
       iChildBody != mChildBodyNodes.end(); ++iChildBody) {
    Joint* childJoint = (*iChildBody)->getParentJoint();
    assert(childJoint != NULL);

    mF += math::dAdInvT(childJoint->getLocalTransform(),
                        (*iChildBody)->getBodyForce());
  }

  assert(!math::isNan(mF));
}

void BodyNode::updateGeneralizedForce(bool _withDampingForces) {
  assert(mParentJoint != NULL);

  const math::Jacobian& J = mParentJoint->getLocalJacobian();

  //    if (_withDampingForces)
  //        mF -= mFDamp;

  assert(!math::isNan(J.transpose()*mF));

  mParentJoint->set_tau(J.transpose()*mF);
}

void BodyNode::updateArticulatedInertia(double _timeStep) {
  assert(mParentJoint != NULL);

  // Articulated inertia
  mAI = mI;
  mImplicitAI = mI;
  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it) {
    mAI += math::transformInertia(
             (*it)->getParentJoint()->getLocalTransform().inverse(),
             (*it)->mPi);
    mImplicitAI += math::transformInertia(
                     (*it)->getParentJoint()->getLocalTransform().inverse(),
                     (*it)->mImplicitPi);
  }
  assert(!math::isNan(mAI));
  assert(!math::isNan(mImplicitAI));

  // Cache data: PsiK and Psi
  mAI_S.noalias() = mAI * mParentJoint->getLocalJacobian();
  mImplicitAI_S.noalias() = mImplicitAI * mParentJoint->getLocalJacobian();
  int dof = mParentJoint->getNumGenCoords();
  if (dof > 0) {
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(dof, dof);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(dof, dof);
    for (int i = 0; i < dof; ++i) {
      K(i, i) = mParentJoint->getSpringStiffness(i);
      D(i, i) = mParentJoint->getDampingCoefficient(i);
    }

    Eigen::MatrixXd omega =
        mParentJoint->getLocalJacobian().transpose() * mAI_S;
    Eigen::MatrixXd implicitOmega =
        mParentJoint->getLocalJacobian().transpose() * mImplicitAI_S;
#ifndef NDEBUG
    // Eigen::FullPivLU<Eigen::MatrixXd> omegaKLU(omega + _timeStep * K);
    // Eigen::FullPivLU<Eigen::MatrixXd> omegaLU(omega);
    // assert(omegaKLU.isInvertible());
    // assert(omegaLU.isInvertible());
#endif
    // mPsiK = (omega + _timeStep*_timeStep*K + _timeStep * K).inverse();
    mImplicitPsi
        = (implicitOmega
           + _timeStep * D
           + _timeStep * _timeStep * K
           ).ldlt().solve(Eigen::MatrixXd::Identity(dof, dof));
    // mPsi = (omega).inverse();
    mPsi = (omega).ldlt().solve(Eigen::MatrixXd::Identity(dof, dof));
  }
  assert(!math::isNan(mImplicitPsi));
  assert(!math::isNan(mPsi));

  // Cache data: AI_S_Psi
  mAI_S_Psi = mAI_S * mPsi;
  mImplicitAI_S_ImplicitPsi = mImplicitAI_S * mImplicitPsi;

  // Cache data: Pi
  mPi = mAI;
  mImplicitPi = mImplicitAI;
  if (dof > 0)
  {
    mPi.noalias() -= mAI_S * mPsi * mAI_S.transpose();
    mImplicitPi.noalias()
        -= mImplicitAI_S * mImplicitPsi * mImplicitAI_S.transpose();
  }
  assert(!math::isNan(mPi));
  assert(!math::isNan(mImplicitPi));
}

void BodyNode::updateBiasForce(double _timeStep,
                               const Eigen::Vector3d& _gravity) {
  // Bias force
  if (mGravityMode == true)
    mFgravity.noalias() = mI * math::AdInvRLinear(mW, _gravity);
  else
    mFgravity.setZero();
  mB = -math::dad(mV, mI * mV) - mFext - mFgravity;
  assert(!math::isNan(mB));
  for (int i = 0; i < mContactForces.size(); ++i)
    mB -= mContactForces[i];
  assert(!math::isNan(mB));
  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it) {
    mB += math::dAdInvT((*it)->getParentJoint()->getLocalTransform(),
                        (*it)->mBeta);
  }
  assert(!math::isNan(mB));

  // Cache data: alpha
  int dof = mParentJoint->getNumGenCoords();
  if (dof > 0) {
    mAlpha = mParentJoint->get_tau()
             + mParentJoint->getSpringForces(_timeStep)
             + mParentJoint->getDampingForces();
    for (int i = 0; i < dof; i++) {
      int idx = mParentJoint->getGenCoord(i)->getSkeletonIndex();
      mAlpha(i) += mSkeleton->getConstraintForceVector()[idx];
    }
    mAlpha.noalias() -= mImplicitAI_S.transpose() * mEta;
    mAlpha.noalias() -= mParentJoint->getLocalJacobian().transpose() * mB;
    assert(!math::isNan(mAlpha));
  }

  // Cache data: beta
  mBeta = mB;
  mBeta.noalias() += mImplicitAI * mEta;
  if (dof > 0) {
    mBeta.noalias() += mImplicitAI_S * mImplicitPsi * mAlpha;
  }
  assert(!math::isNan(mBeta));
}

void BodyNode::update_ddq() {
  if (mParentJoint->getNumGenCoords() == 0)
    return;

  Eigen::VectorXd ddq;
  if (mParentBodyNode) {
    ddq.noalias() =
        mImplicitPsi * (mAlpha - mImplicitAI_S.transpose() *
                        math::AdInvT(mParentJoint->getLocalTransform(),
                                     mParentBodyNode->getBodyAcceleration()));
  } else {
    ddq.noalias() = mImplicitPsi * mAlpha;
  }

  mParentJoint->set_ddq(ddq);
  assert(!math::isNan(ddq));

  updateAcceleration();
}

void BodyNode::update_F_fs() {
  mF = mB;
  mF.noalias() = mAI * mdV;
  assert(!math::isNan(mF));
}

void BodyNode::aggregateCoriolisForceVector(Eigen::VectorXd* _C) {
  aggregateCombinedVector(_C, Eigen::Vector3d::Zero());
}

void BodyNode::aggregateGravityForceVector(Eigen::VectorXd* _g,
                                           const Eigen::Vector3d& _gravity) {
  if (mGravityMode == true)
    mG_F = mI * math::AdInvRLinear(mW, _gravity);
  else
    mG_F.setZero();

  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it) {
    mG_F += math::dAdInvT((*it)->mParentJoint->getLocalTransform(),
                          (*it)->mG_F);
  }

  int nGenCoords = mParentJoint->getNumGenCoords();
  if (nGenCoords > 0) {
    Eigen::VectorXd g = -(mParentJoint->getLocalJacobian().transpose() * mG_F);
    int iStart = mParentJoint->getGenCoord(0)->getSkeletonIndex();
    _g->segment(iStart, nGenCoords) = g;
  }
}

void BodyNode::updateCombinedVector() {
  if (mParentJoint->getNumGenCoords() > 0) {
    if (mParentBodyNode) {
      mCg_dV = math::AdInvT(mParentJoint->getLocalTransform(),
                            mParentBodyNode->mCg_dV) + mEta;
    } else {
      mCg_dV = mEta;
    }
  }
}

void BodyNode::aggregateCombinedVector(Eigen::VectorXd* _Cg,
                                       const Eigen::Vector3d& _gravity) {
  // H(i) = I(i) * W(i) -
  //        dad{V}(I(i) * V(i)) + sum(k \in children) dAd_{T(i,j)^{-1}}(H(k))
  if (mGravityMode == true)
    mFgravity = mI * math::AdInvRLinear(mW, _gravity);
  else
    mFgravity.setZero();

  mCg_F = mI * mCg_dV;
  mCg_F -= mFgravity;
  mCg_F -= math::dad(mV, mI * mV);

  for (std::vector<BodyNode*>::iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it) {
    mCg_F += math::dAdInvT((*it)->getParentJoint()->getLocalTransform(),
                           (*it)->mCg_F);
  }

  int nGenCoords = mParentJoint->getNumGenCoords();
  if (nGenCoords > 0) {
    Eigen::VectorXd Cg =
        mParentJoint->getLocalJacobian().transpose() * mCg_F;
    int iStart = mParentJoint->getGenCoord(0)->getSkeletonIndex();
    _Cg->segment(iStart, nGenCoords) = Cg;
  }
}

void BodyNode::aggregateExternalForces(Eigen::VectorXd* _Fext) {
  mFext_F = mFext;

  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it) {
    mFext_F += math::dAdInvT((*it)->mParentJoint->getLocalTransform(),
                             (*it)->mFext_F);
  }

  int nGenCoords = mParentJoint->getNumGenCoords();
  if (nGenCoords > 0) {
    Eigen::VectorXd Fext = mParentJoint->getLocalJacobian().transpose()*mFext_F;
    int iStart = mParentJoint->getGenCoord(0)->getSkeletonIndex();
    _Fext->segment(iStart, nGenCoords) = Fext;
  }
}

void BodyNode::updateMassMatrix() {
  mM_dV.setZero();
  int dof = mParentJoint->getNumGenCoords();
  if (dof > 0) {
    mM_dV.noalias() += mParentJoint->getLocalJacobian() *
                       mParentJoint->get_ddq();
    assert(!math::isNan(mM_dV));
  }
  if (mParentBodyNode)
    mM_dV += math::AdInvT(mParentJoint->getLocalTransform(),
                          mParentBodyNode->mM_dV);
  assert(!math::isNan(mM_dV));
}

void BodyNode::aggregateMassMatrix(Eigen::MatrixXd* _MCol, int _col) {
  mM_F.noalias() = mI * mM_dV;
  assert(!math::isNan(mM_F));
  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it) {
    mM_F += math::dAdInvT((*it)->getParentJoint()->getLocalTransform(),
                          (*it)->mM_F);
  }
  assert(!math::isNan(mM_F));

  int dof = mParentJoint->getNumGenCoords();
  if (dof > 0) {
    int iStart = mParentJoint->getGenCoord(0)->getSkeletonIndex();
    _MCol->block(iStart, _col, dof, 1).noalias() =
        mParentJoint->getLocalJacobian().transpose() * mM_F;
  }
}

void BodyNode::aggregateAugMassMatrix(Eigen::MatrixXd* _MCol, int _col,
                                      double _timeStep) {
  mM_F.noalias() = mI * mM_dV;
  assert(!math::isNan(mM_F));
  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it) {
    mM_F += math::dAdInvT((*it)->getParentJoint()->getLocalTransform(),
                          (*it)->mM_F);
  }
  assert(!math::isNan(mM_F));

  int dof = mParentJoint->getNumGenCoords();
  if (dof > 0) {
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(dof, dof);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(dof, dof);
    for (int i = 0; i < dof; ++i) {
      K(i, i) = mParentJoint->getSpringStiffness(i);
      D(i, i) = mParentJoint->getDampingCoefficient(i);
    }
    int iStart = mParentJoint->getGenCoord(0)->getSkeletonIndex();
    _MCol->block(iStart, _col, dof, 1).noalias()
        = mParentJoint->getLocalJacobian().transpose() * mM_F
          + D * (_timeStep * mParentJoint->get_ddq())
          + K * (_timeStep * _timeStep * mParentJoint->get_ddq());
  }
}

void BodyNode::updateInvMassMatrix() {
  mInvM_c.setZero();
  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it) {
    mInvM_c += math::dAdInvT((*it)->getParentJoint()->getLocalTransform(),
                             (*it)->mInvM_b);
  }
  assert(!math::isNan(mInvM_c));

  // Cache data: mInvM2_a
  int dof = mParentJoint->getNumGenCoords();
  if (dof > 0) {
    mInvM_a = mParentJoint->get_tau();
    mInvM_a.noalias() -= mParentJoint->getLocalJacobian().transpose() * mInvM_c;
    assert(!math::isNan(mInvM_a));
  }

  // Cache data: mInvM2_b
  if (mParentBodyNode) {
    mInvM_b = mInvM_c;
    if (dof > 0)
      mInvM_b.noalias() += mAI_S_Psi * mInvM_a;
  }
  assert(!math::isNan(mInvM_b));
}

void BodyNode::updateInvAugMassMatrix() {
  mInvM_c.setZero();
  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it) {
    mInvM_c += math::dAdInvT((*it)->getParentJoint()->getLocalTransform(),
                             (*it)->mInvM_b);
  }
  assert(!math::isNan(mInvM_c));

  // Cache data: mInvM2_a
  int dof = mParentJoint->getNumGenCoords();
  if (dof > 0) {
    mInvM_a = mParentJoint->get_tau();
    mInvM_a.noalias() -= mParentJoint->getLocalJacobian().transpose() * mInvM_c;
    assert(!math::isNan(mInvM_a));
  }

  // Cache data: mInvM2_b
  if (mParentBodyNode) {
    mInvM_b = mInvM_c;
    if (dof > 0)
      mInvM_b.noalias() += mImplicitAI_S_ImplicitPsi * mInvM_a;
  }
  assert(!math::isNan(mInvM_b));
}

void BodyNode::aggregateInvMassMatrix(Eigen::MatrixXd* _InvMCol, int _col) {
  Eigen::VectorXd MInvCol;
  int dof = mParentJoint->getNumGenCoords();
  if (dof > 0) {
    if (mParentBodyNode) {
      MInvCol.noalias() = mPsi * mInvM_a;
      MInvCol.noalias() -= mAI_S_Psi.transpose()
                           * math::AdInvT(mParentJoint->getLocalTransform(),
                                          mParentBodyNode->mInvM_U);
    } else {
      MInvCol.noalias() = mPsi * mInvM_a;
    }
    assert(!math::isNan(MInvCol));

    // Assign
    int iStart = mParentJoint->getGenCoord(0)->getSkeletonIndex();
    _InvMCol->block(iStart, _col, dof, 1) = MInvCol;
  }

  if (mChildBodyNodes.size() > 0) {
    if (dof > 0)
      mInvM_U.noalias() = mParentJoint->getLocalJacobian() * MInvCol;
    else
      mInvM_U.setZero();

    if (mParentBodyNode) {
      mInvM_U += math::AdInvT(mParentJoint->getLocalTransform(),
                              mParentBodyNode->mInvM_U);
    }
    assert(!math::isNan(mInvM_U));
  }
}

void BodyNode::aggregateInvAugMassMatrix(Eigen::MatrixXd* _InvMCol, int _col,
                                         double /*_timeStep*/) {
  Eigen::VectorXd MInvCol;
  int dof = mParentJoint->getNumGenCoords();
  if (dof > 0) {
    if (mParentBodyNode) {
      MInvCol.noalias() = mImplicitPsi * mInvM_a;
      MInvCol.noalias() -= mImplicitAI_S_ImplicitPsi.transpose()
                           * math::AdInvT(mParentJoint->getLocalTransform(),
                                          mParentBodyNode->mInvM_U);
    } else {
      MInvCol.noalias() = mImplicitPsi * mInvM_a;
    }
    assert(!math::isNan(MInvCol));

    // Assign
    int iStart = mParentJoint->getGenCoord(0)->getSkeletonIndex();
    _InvMCol->block(iStart, _col, dof, 1) = MInvCol;
  }

  if (mChildBodyNodes.size() > 0) {
    if (dof > 0)
      mInvM_U.noalias() = mParentJoint->getLocalJacobian() * MInvCol;
    else
      mInvM_U.setZero();

    if (mParentBodyNode) {
      mInvM_U += math::AdInvT(mParentJoint->getLocalTransform(),
                              mParentBodyNode->mInvM_U);
    }
    assert(!math::isNan(mInvM_U));
  }
}

void BodyNode::_updateBodyJacobian() {
  //--------------------------------------------------------------------------
  // Jacobian update
  //
  // J = | J1 J2 ... Jn |
  //   = | Ad(T(i,i-1), J_parent) J_local |
  //
  //   J_parent: (6 x parentDOF)
  //    J_local: (6 x localDOF)
  //         Ji: (6 x 1) se3
  //          n: number of dependent coordinates
  //--------------------------------------------------------------------------

  const int localDof     = mParentJoint->getNumGenCoords();
  const int ascendantDof = getNumDependentGenCoords() - localDof;

  // Parent Jacobian
  if (mParentBodyNode) {
    assert(mParentBodyNode->getBodyJacobian().cols() +
           mParentJoint->getNumGenCoords() == mBodyJacobian.cols());

    assert(mParentJoint);
    mBodyJacobian.leftCols(ascendantDof) =
        math::AdInvTJac(mParentJoint->getLocalTransform(),
                        mParentBodyNode->getBodyJacobian());
  }

  // Local Jacobian
  mBodyJacobian.rightCols(localDof) = mParentJoint->getLocalJacobian();

  mIsBodyJacobianDirty = false;
}

void BodyNode::_updateBodyJacobianTimeDeriv()
{
  //--------------------------------------------------------------------------
  // Jacobian first derivative update
  //
  // dJ = | dJ1 dJ2 ... dJn |
  //   = | Ad(T(i,i-1), dJ_parent) dJ_local |
  //
  //   dJ_parent: (6 x parentDOF)
  //    dJ_local: (6 x localDOF)
  //         dJi: (6 x 1) se3
  //          n: number of dependent coordinates
  //--------------------------------------------------------------------------

  const int numLocalDOFs = mParentJoint->getNumGenCoords();
  const int numParentDOFs = getNumDependentGenCoords() - numLocalDOFs;
  math::Jacobian J = getBodyJacobian();

  // Parent Jacobian
  if (mParentBodyNode) {
    assert(mParentBodyNode->mBodyJacobianTimeDeriv.cols()
           + mParentJoint->getNumGenCoords() == mBodyJacobianTimeDeriv.cols());

    assert(mParentJoint);
    mBodyJacobianTimeDeriv.leftCols(numParentDOFs)
        = math::AdInvTJac(mParentJoint->getLocalTransform(),
                          mParentBodyNode->mBodyJacobianTimeDeriv);
    for (int i = 0; i < numParentDOFs; ++i)
      mBodyJacobianTimeDeriv.col(i) -= math::ad(mV, J.col(i));
  }

  // Local Jacobian
  mBodyJacobianTimeDeriv.rightCols(numLocalDOFs) =
      mParentJoint->getLocalJacobianTimeDeriv();

  mIsBodyJacobianTimeDerivDirty = false;
}

void BodyNode::_updateGeralizedInertia() {
  // G = | I - m*[r]*[r]   m*[r] |
  //     |        -m*[r]     m*I |

  // m*r
  double mr0 = mMass * mCenterOfMass[0];
  double mr1 = mMass * mCenterOfMass[1];
  double mr2 = mMass * mCenterOfMass[2];

  // m*[r]*[r]
  double mr0r0 = mr0 * mCenterOfMass[0];
  double mr1r1 = mr1 * mCenterOfMass[1];
  double mr2r2 = mr2 * mCenterOfMass[2];
  double mr0r1 = mr0 * mCenterOfMass[1];
  double mr1r2 = mr1 * mCenterOfMass[2];
  double mr2r0 = mr2 * mCenterOfMass[0];

  // Top left corner (3x3)
  mI(0, 0) =  mIxx + mr1r1 + mr2r2;
  mI(1, 1) =  mIyy + mr2r2 + mr0r0;
  mI(2, 2) =  mIzz + mr0r0 + mr1r1;
  mI(0, 1) =  mIxy - mr0r1;
  mI(0, 2) =  mIxz - mr2r0;
  mI(1, 2) =  mIyz - mr1r2;

  // Top right corner (3x3)
  mI(1, 5) = -mr0;
  mI(0, 5) =  mr1;
  mI(0, 4) = -mr2;
  mI(2, 4) =  mr0;
  mI(2, 3) = -mr1;
  mI(1, 3) =  mr2;
  assert(mI(0, 3) == 0.0);
  assert(mI(1, 4) == 0.0);
  assert(mI(2, 5) == 0.0);

  // Bottom right corner (3x3)
  mI(3, 3) =  mMass;
  mI(4, 4) =  mMass;
  mI(5, 5) =  mMass;
  assert(mI(3, 4) == 0.0);
  assert(mI(3, 5) == 0.0);
  assert(mI(4, 5) == 0.0);

  mI.triangularView<Eigen::StrictlyLower>() = mI.transpose();
}

void BodyNode::clearExternalForces() {
  mFext.setZero();
  mContactForces.clear();
}

}  // namespace dynamics
}  // namespace dart
