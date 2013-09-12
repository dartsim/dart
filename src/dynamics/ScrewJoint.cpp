/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#include "math/Geometry.h"
#include "dynamics/BodyNode.h"
#include "dynamics/ScrewJoint.h"

namespace dart {
namespace dynamics {

ScrewJoint::ScrewJoint(BodyNode* _parent, BodyNode* _child,
                       const Eigen::Vector3d& axis,
                       double _pitch,
                       const std::string& _name)
    : Joint(_parent, _child, _name),
      mAxis(axis.normalized()),
      mPitch(_pitch)
{
    mJointType = SCREW;

    mGenCoords.push_back(&mCoordinate);

    mS = Eigen::Matrix<double,6,1>::Zero();
    mdS = Eigen::Matrix<double,6,1>::Zero();

    mDampingCoefficient.resize(1, 0);
}

ScrewJoint::~ScrewJoint()
{
}

void ScrewJoint::setAxis(const Eigen::Vector3d& _axis)
{
    assert(_axis.norm() == 1);
    mAxis = _axis;
}

const Eigen::Vector3d&ScrewJoint::getAxis() const
{
    return mAxis;
}

Eigen::Vector3d ScrewJoint::getAxisGlobal() const
{
    Eigen::Isometry3d parentTransf = Eigen::Isometry3d::Identity();

    if (this->mParentBodyNode != NULL)
        parentTransf = mParentBodyNode->getWorldTransform();

    return parentTransf.linear() * mT_ParentBodyToJoint.linear() * mAxis;
}

void ScrewJoint::setPitch(double _pitch)
{
    mPitch = _pitch;
}

double ScrewJoint::getPitch() const
{
    return mPitch;
}

void ScrewJoint::_updateTransform()
{
    // T
    Eigen::Vector6d S = Eigen::Vector6d::Zero();
    S.head<3>() = mAxis;
    S.tail<3>() = mAxis*mPitch/DART_2PI;
    mT = mT_ParentBodyToJoint
         * math::expMap(S*mCoordinate.get_q())
         * mT_ChildBodyToJoint.inverse();

    assert(math::verifyTransform(mT));
}

void ScrewJoint::_updateVelocity()
{
    // S
    Eigen::Vector6d S = Eigen::Vector6d::Zero();
    S.head<3>() = mAxis;
    S.tail<3>() = mAxis*mPitch/DART_2PI;
    mS = math::AdT(mT_ChildBodyToJoint, S);

    // V = S * dq
    mV = mS * get_dq();
}

void ScrewJoint::_updateAcceleration()
{
    // dS = 0
    mdS.setZero();

    // dV = dS * dq + S * ddq
    mdV = mS * get_ddq();
}

} // namespace dynamics
} // namespace dart
