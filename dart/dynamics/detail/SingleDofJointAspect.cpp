/*
 * Copyright (c) 2015-2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
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

#include "dart/dynamics/SingleDofJoint.h"

namespace dart {
namespace dynamics {
namespace detail {

//==============================================================================
SingleDofJointState::SingleDofJointState(
    double position,
    double velocity,
    double acceleration,
    double force,
    double command)
  : mPosition(position),
    mVelocity(velocity),
    mAcceleration(acceleration),
    mForce(force),
    mCommand(command)
{
  // Do nothing
}

//==============================================================================
SingleDofJointUniqueProperties::SingleDofJointUniqueProperties(
    double _positionLowerLimit,
    double _positionUpperLimit,
    double _velocityLowerLimit,
    double _velocityUpperLimit,
    double _accelerationLowerLimit,
    double _accelerationUpperLimit,
    double _forceLowerLimit,
    double _forceUpperLimit,
    double _springStiffness,
    double _restPosition,
    double _dampingCoefficient,
    double _coulombFriction,
    bool _preserveDofName,
    std::string _dofName)
  : mPositionLowerLimit(_positionLowerLimit),
    mPositionUpperLimit(_positionUpperLimit),
    mInitialPosition(0.0),
    mVelocityLowerLimit(_velocityLowerLimit),
    mVelocityUpperLimit(_velocityUpperLimit),
    mInitialVelocity(0.0),
    mAccelerationLowerLimit(_accelerationLowerLimit),
    mAccelerationUpperLimit(_accelerationUpperLimit),
    mForceLowerLimit(_forceLowerLimit),
    mForceUpperLimit(_forceUpperLimit),
    mSpringStiffness(_springStiffness),
    mRestPosition(_restPosition),
    mDampingCoefficient(_dampingCoefficient),
    mFriction(_coulombFriction),
    mPreserveDofName(_preserveDofName),
    mDofName(_dofName)
{
  // Do nothing
}

//==============================================================================
SingleDofJointProperties::SingleDofJointProperties(
    const Joint::Properties& _jointProperties,
    const SingleDofJointUniqueProperties& _singleDofProperties)
  : Joint::Properties(_jointProperties),
    SingleDofJointUniqueProperties(_singleDofProperties)
{
  // Do nothing
}

} // namespace detail
} // namespace dynamics
} // namespace dart
