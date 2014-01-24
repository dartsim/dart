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

#include "dart/dynamics/Joint.h"

#include <string>

#include "dart/common/Console.h"
#include "dart/renderer/RenderInterface.h"
#include "dart/dynamics/BodyNode.h"

namespace dart {
namespace dynamics {

Joint::Joint(JointType _type, const std::string& _name)
  : mName(_name),
    mSkelIndex(-1),
    mJointType(_type),
    mIsPositionLimited(true),
    mT_ParentBodyToJoint(Eigen::Isometry3d::Identity()),
    mT_ChildBodyToJoint(Eigen::Isometry3d::Identity()),
    mT(Eigen::Isometry3d::Identity()) {
}

Joint::~Joint() {
}

void Joint::setName(const std::string& _name) {
  mName = _name;
}

const std::string& Joint::getName() const {
  return mName;
}

Joint::JointType Joint::getJointType() const {
  return mJointType;
}

const Eigen::Isometry3d&Joint::getLocalTransform() const {
  return mT;
}

const math::Jacobian&Joint::getLocalJacobian() const {
  return mS;
}

const math::Jacobian&Joint::getLocalJacobianTimeDeriv() const {
  return mdS;
}

bool Joint::contains(const GenCoord* _genCoord) const {
  return find(mGenCoords.begin(), mGenCoords.end(), _genCoord) !=
      mGenCoords.end() ? true : false;
}

int Joint::getGenCoordLocalIndex(int _dofSkelIndex) const {
  for (unsigned int i = 0; i < mGenCoords.size(); i++)
    if (mGenCoords[i]->getSkeletonIndex() == _dofSkelIndex)
      return i;
  return -1;
}

void Joint::setPositionLimited(bool _isPositionLimited) {
  mIsPositionLimited = _isPositionLimited;
}

bool Joint::isPositionLimited() const {
  return mIsPositionLimited;
}

int Joint::getSkeletonIndex() const {
  return mSkelIndex;
}

void Joint::setTransformFromParentBodyNode(const Eigen::Isometry3d& _T) {
  assert(math::verifyTransform(_T));
  mT_ParentBodyToJoint = _T;
}

void Joint::setTransformFromChildBodyNode(const Eigen::Isometry3d& _T) {
  assert(math::verifyTransform(_T));
  mT_ChildBodyToJoint = _T;
}

const Eigen::Isometry3d&Joint::getTransformFromParentBodyNode() const {
  return mT_ParentBodyToJoint;
}

const Eigen::Isometry3d&Joint::getTransformFromChildBodyNode() const {
  return mT_ChildBodyToJoint;
}

void Joint::applyGLTransform(renderer::RenderInterface* _ri) {
  _ri->transform(mT);
}

void Joint::setDampingCoefficient(int _idx, double _d) {
  assert(0 <= _idx && _idx < getNumGenCoords());
  assert(_d >= 0.0);
  mDampingCoefficient[_idx] = _d;
}

double Joint::getDampingCoefficient(int _idx) const {
  assert(0 <= _idx && _idx < getNumGenCoords());
  return mDampingCoefficient[_idx];
}

Eigen::VectorXd Joint::getDampingForces() const {
  int numDofs = getNumGenCoords();
  Eigen::VectorXd dampingForce(numDofs);

  for (int i = 0; i < numDofs; ++i)
    dampingForce(i) = -mDampingCoefficient[i] * getGenCoord(i)->get_dq();

  return dampingForce;
}

void Joint::setSpringStiffness(int _idx, double _k) {
  assert(0 <= _idx && _idx < getNumGenCoords());
  assert(_k >= 0.0);
  mSpringStiffness[_idx] = _k;
}

double Joint::getSpringStiffness(int _idx) const {
  assert(0 <= _idx && _idx < getNumGenCoords());
  return mSpringStiffness[_idx];
}

void Joint::setRestPosition(int _idx, double _q0) {
  assert(0 <= _idx && _idx < getNumGenCoords());

  if (getGenCoord(_idx)->get_qMin() > _q0
      || getGenCoord(_idx)->get_qMax() < _q0)
  {
    dtwarn << "Rest position of joint[" << getName() << "], " << _q0
           << ", is out of the limit range["
           << getGenCoord(_idx)->get_qMin() << ", "
           << getGenCoord(_idx)->get_qMax() << "] in index[" << _idx
           << "].\n";
  }

  mRestPosition[_idx] = _q0;
}

double Joint::getRestPosition(int _idx) const {
  assert(0 <= _idx && _idx < getNumGenCoords());
  return mRestPosition[_idx];
}

Eigen::VectorXd Joint::getSpringForces(double _timeStep) const {
  int dof = getNumGenCoords();
  Eigen::VectorXd springForce(dof);
  for (int i = 0; i < dof; ++i) {
    springForce(i) =
        -mSpringStiffness[i] * (getGenCoord(i)->get_q()
                                + getGenCoord(i)->get_dq() * _timeStep
                                - mRestPosition[i]);
  }
  return springForce;
}

double Joint::getPotentialEnergy() const {
  double PE = 0.0;
  int dof = getNumGenCoords();

  // Spring energy
  Eigen::VectorXd q = get_q();
  assert(q.size() == dof);
  for (int i = 0; i < dof; ++i) {
    PE += 0.5 * mSpringStiffness[i]
          * (q[i] - mRestPosition[i]) * (q[i] - mRestPosition[i]);
  }

  return PE;
}

}  // namespace dynamics
}  // namespace dart
