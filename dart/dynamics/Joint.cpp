/*
 * Copyright (c) 2011-2014, Georgia Tech Research Corporation
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
#include "dart/math/Helpers.h"
#include "dart/renderer/RenderInterface.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"

namespace dart {
namespace dynamics {

//==============================================================================
Joint::Joint(const std::string& _name)
  : mName(_name),
    mSkeleton(NULL),
    mT_ParentBodyToJoint(Eigen::Isometry3d::Identity()),
    mT_ChildBodyToJoint(Eigen::Isometry3d::Identity()),
    mT(Eigen::Isometry3d::Identity()),
    mSpatialVelocity(Eigen::Vector6d::Zero()),
    mWrench(Eigen::Vector6d::Zero()),
    mIsPositionLimited(true)
{
}

Joint::~Joint() {
}

void Joint::setName(const std::string& _name) {
  mName = _name;
}

const std::string& Joint::getName() const {
  return mName;
}

Skeleton* Joint::getSkeleton() const
{
  return mSkeleton;
}

const Eigen::Isometry3d& Joint::getLocalTransform() const {
  return mT;
}

//==============================================================================
//bool Joint::contains(const GenCoord* _genCoord) const {
//  return find(mGenCoords.begin(), mGenCoords.end(), _genCoord) !=
//      mGenCoords.end() ? true : false;
//}

//==============================================================================
//int Joint::getGenCoordLocalIndex(int _dofSkelIndex) const
//{
//  for (unsigned int i = 0; i < mGenCoords.size(); i++)
//    if (mGenCoords[i]->getIndexInSkeleton() == _dofSkelIndex)
//      return i;
//  return -1;
//}

void Joint::setPositionLimited(bool _isPositionLimited) {
  mIsPositionLimited = _isPositionLimited;
}

bool Joint::isPositionLimited() const {
  return mIsPositionLimited;
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

void Joint::init(Skeleton* _skel)
{
  mSkeleton = _skel;
}

//==============================================================================
//Eigen::VectorXd Joint::getDampingForces() const
//{
//  int numDofs = getNumDofs();
//  Eigen::VectorXd dampingForce(numDofs);

//  for (int i = 0; i < numDofs; ++i)
//    dampingForce(i) = -mDampingCoefficient[i] * getGenCoord(i)->getVel();

//  return dampingForce;
//}

//==============================================================================
//Eigen::VectorXd Joint::getSpringForces(double _timeStep) const
//{
//  int dof = getNumDofs();
//  Eigen::VectorXd springForce(dof);
//  for (int i = 0; i < dof; ++i)
//  {
//    springForce(i) =
//        -mSpringStiffness[i] * (getGenCoord(i)->getPos()
//                                + getGenCoord(i)->getVel() * _timeStep
//                                - mRestPosition[i]);
//  }
//  assert(!math::isNan(springForce));
//  return springForce;
//}

}  // namespace dynamics
}  // namespace dart
