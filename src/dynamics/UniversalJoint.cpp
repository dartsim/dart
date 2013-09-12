/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 08/22/2013
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

#include "UniversalJoint.h"

#include "math/Helpers.h"
#include "math/Geometry.h"

namespace dart {
namespace dynamics {

UniversalJoint::UniversalJoint(BodyNode* _parent, BodyNode* _child,
                               const Eigen::Vector3d& _axis0,
                               const Eigen::Vector3d& _axis1,
                               const std::string& _name)
    : Joint(_parent, _child, _name)
{
    mJointType = UNIVERSAL;

    mGenCoords.push_back(&mCoordinate[0]);
    mGenCoords.push_back(&mCoordinate[1]);

    mS = Eigen::Matrix<double,6,2>::Zero();
    mdS = Eigen::Matrix<double,6,2>::Zero();

    mDampingCoefficient.resize(2, 0);

    mAxis[0] = _axis0.normalized();
    mAxis[1] = _axis1.normalized();
}

UniversalJoint::~UniversalJoint()
{
}

void UniversalJoint::setAxis1(const Eigen::Vector3d& _axis)
{
    assert(_axis.norm() == 1);
    mAxis[0] = _axis;
}

void UniversalJoint::setAxis2(const Eigen::Vector3d& _axis)
{
    assert(_axis.norm() == 1);
    mAxis[1] = _axis;
}

const Eigen::Vector3d& UniversalJoint::getAxis1() const
{
    return mAxis[0];
}

const Eigen::Vector3d& UniversalJoint::getAxis2() const
{
    return mAxis[1];
}

inline void UniversalJoint::_updateTransform()
{
    // T
    mT = mT_ParentBodyToJoint *
         math::expAngular(mAxis[0] * mCoordinate[0].get_q()) *
         math::expAngular(mAxis[1] * mCoordinate[1].get_q()) *
         mT_ChildBodyToJoint.inverse();
}

inline void UniversalJoint::_updateVelocity()
{
    // S
    mS.col(0) = math::AdTAngular(mT_ChildBodyToJoint*math::expAngular(-mAxis[1]*mCoordinate[1].get_q()), mAxis[0]);
    mS.col(1) = math::AdTAngular(mT_ChildBodyToJoint, mAxis[1]);

    // V = S * dq
    mV = mS * get_dq();
}

inline void UniversalJoint::_updateAcceleration()
{
    // dS
    mdS.col(0) = -math::ad(mS.col(1)*mCoordinate[1].get_dq(), math::AdTAngular(mT_ChildBodyToJoint * math::expAngular(-mAxis[1]*mCoordinate[1].get_q()), mAxis[0]));
    //mdS.col(1) = setZero();

    // dV = dS * dq + S * ddq
    mdV = mdS * get_dq() + mS * get_ddq();
}

} // namespace dynamics
} // namespace dart
