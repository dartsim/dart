/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
 *            Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/21/2013
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

#include "renderer/RenderInterface.h"
#include "dynamics/BodyNode.h"
#include "dynamics/Joint.h"

namespace dart {
namespace dynamics {

Joint::Joint(BodyNode* _parent, BodyNode* _child, const std::string& _name)
    : mName(_name),
      mSkelIndex(-1),
      mJointType(UNKNOWN),
      mParentBodyNode(_parent),
      mChildBodyNode(_child),
      mIsPositionLimited(true),
      mT_ParentBodyToJoint(Eigen::Isometry3d::Identity()),
      mT_ChildBodyToJoint(Eigen::Isometry3d::Identity()),
      mT(Eigen::Isometry3d::Identity()),
      mV(Eigen::Vector6d::Zero()),
      mS(math::Jacobian::Zero(6,0)),
      mdV(Eigen::Vector6d::Zero()),
      mdS(math::Jacobian::Zero(6,0))
{
    setParentBodyNode(mParentBodyNode);
    setChildBodyNode(mChildBodyNode);
}

Joint::~Joint()
{
}

void Joint::setName(const std::string& _name)
{
    mName = _name;
}

const std::string& Joint::getName() const
{
    return mName;
}

Joint::JointType Joint::getJointType() const
{
    return mJointType;
}

const Eigen::Isometry3d&Joint::getLocalTransform() const
{
    return mT;
}

const math::Jacobian&Joint::getLocalJacobian() const
{
    return mS;
}

const Eigen::Vector6d&Joint::getLocalVelocity() const
{
    return mV;
}

const math::Jacobian&Joint::getLocalJacobianFirstDerivative() const
{
    return mdS;
}

const Eigen::Vector6d&Joint::getLocalAcceleration() const
{
    return mdV;
}

bool Joint::isPresent(const GenCoord* _q) const
{
    for (unsigned int i = 0; i < getNumGenCoords(); i++)
        if (_q == mGenCoords[i])
            return true;

    return false;
}

int Joint::getGenCoordLocalIndex(int _dofSkelIndex) const
{
    for (unsigned int i = 0; i < mGenCoords.size(); i++)
        if (mGenCoords[i]->getSkeletonIndex() == _dofSkelIndex)
            return i;

    return -1;
}

void Joint::setPositionLimited(bool _positionLimit)
{
    mIsPositionLimited = _positionLimit;
}

bool Joint::isPositionLimited() const
{
    return mIsPositionLimited;
}

void Joint::setSkeletonIndex(int _idx)
{
    mSkelIndex= _idx;
}

int Joint::getSkeletonIndex() const
{
    return mSkelIndex;
}

void Joint::setParentBodyNode(BodyNode* _body)
{
    mParentBodyNode = _body;

    if (mParentBodyNode != NULL)
    {
        mParentBodyNode->addChildJoint(this);

        if (mChildBodyNode != NULL)
        {
            mChildBodyNode->setParentBodyNode(mParentBodyNode);
            mParentBodyNode->addChildBodyNode(mChildBodyNode);
        }
    }
}

void Joint::setChildBodyNode(BodyNode* _body)
{
    mChildBodyNode = _body;

    if (mChildBodyNode != NULL)
    {
        mChildBodyNode->setParentJoint(this);

        if (mParentBodyNode != NULL)
        {
            mParentBodyNode->addChildBodyNode(mChildBodyNode);
            mChildBodyNode->setParentBodyNode(mParentBodyNode);
        }
    }
}

void Joint::setTransformFromParentBodyNode(const Eigen::Isometry3d& _T)
{
    assert(math::verifyTransform(_T));

    mT_ParentBodyToJoint = _T;
}

void Joint::setTransformFromChildBodyNode(const Eigen::Isometry3d& _T)
{
    assert(math::verifyTransform(_T));

    mT_ChildBodyToJoint = _T;
}

BodyNode* Joint::getParentBodyNode() const
{
    return mParentBodyNode;
}

BodyNode* Joint::getChildBodyNode() const
{
    return mChildBodyNode;
}

const Eigen::Isometry3d&Joint::getTransformFromParentBodyNode() const
{
    return mT_ParentBodyToJoint;
}

const Eigen::Isometry3d&Joint::getTransformFromChildBodyNode() const
{
    return mT_ChildBodyToJoint;
}

void Joint::updateKinematics(bool _firstDerivative,
                             bool _secondDerivative)
{
    _updateTransform();
    _updateVelocity();
    _updateAcceleration();
}

void Joint::applyGLTransform(renderer::RenderInterface* _ri)
{
    _ri->transform(mT);
}

void Joint::setDampingCoefficient(int _idx, double _d)
{
    assert(0 <= _idx && _idx < getNumGenCoords());
    assert(_d >= 0.0);

    mDampingCoefficient[_idx] = _d;
}

double Joint::getDampingCoefficient(int _idx) const
{
    assert(0 <= _idx && _idx < getNumGenCoords());

    return mDampingCoefficient[_idx];
}

Eigen::VectorXd Joint::getDampingForces() const
{
    int numDofs = getNumGenCoords();
    Eigen::VectorXd dampingForce(numDofs);

    for (int i = 0; i < numDofs; ++i)
        dampingForce(i) = -mDampingCoefficient[i] * getGenCoord(i)->get_dq();

    return dampingForce;
}

} // namespace dynamics
} // namespace dart


