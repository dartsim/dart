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
#include "dynamics/PrismaticJoint.h"

namespace dart {

using namespace math;

namespace dynamics {

PrismaticJoint::PrismaticJoint(BodyNode* _parent, BodyNode* _child,
                               const Eigen::Vector3d& axis,
                               const std::string& _name)
    : Joint(_parent, _child, _name),
      mAxis(axis.normalized())
{
    mJointType = PRISMATIC;

    mGenCoords.push_back(&mCoordinate);

    mS = Eigen::Matrix<double,6,1>::Zero();
    mdS = Eigen::Matrix<double,6,1>::Zero();

    mDampingCoefficient.resize(1, 0);
}

PrismaticJoint::~PrismaticJoint()
{
}

void PrismaticJoint::setAxis(const Eigen::Vector3d& _axis)
{
    assert(_axis.norm() == 1.0);
    mAxis = _axis;
}

const Eigen::Vector3d&PrismaticJoint::getAxis() const
{
    return mAxis;
}

Eigen::Vector3d PrismaticJoint::getAxisGlobal() const
{
    Eigen::Isometry3d parentTransf = Eigen::Isometry3d::Identity();

    if (this->mParentBodyNode != NULL)
        parentTransf = mParentBodyNode->getWorldTransform();

    return parentTransf.linear() * mT_ParentBodyToJoint.linear() * mAxis;
}

void PrismaticJoint::_updateTransform()
{
    // T
    mT = mT_ParentBodyToJoint
         * Eigen::Translation3d(mAxis * mCoordinate.get_q())
         * mT_ChildBodyToJoint.inverse();
}

void PrismaticJoint::_updateVelocity()
{
    // S
    mS = math::AdTLinear(mT_ChildBodyToJoint, mAxis);

    // V = S * dq
    mV.noalias() = mS * get_dq();
    //mV.setAngular(mAxis * mCoordinate.get_q());
}

void PrismaticJoint::_updateAcceleration()
{
    // dS = 0
    mdS.setZero();

    // dV = dS * dq + S * ddq
    mdV = mS * get_ddq();
}

} // namespace dynamics
} // namespace dart
