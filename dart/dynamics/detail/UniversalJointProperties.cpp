/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
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

#include "dart/dynamics/UniversalJoint.hpp"

namespace dart {
namespace dynamics {
namespace detail {

//==============================================================================
UniversalJointUniqueProperties::UniversalJointUniqueProperties(
    const Eigen::Vector3d& _axis1, const Eigen::Vector3d& _axis2)
  : mAxis({_axis1.normalized(), _axis2.normalized()})
{
  // Do nothing
}

//==============================================================================
UniversalJointProperties::UniversalJointProperties(
    const MultiDofJoint<2>::Properties& _multiDofProperties,
    const UniversalJointUniqueProperties& _universalProperties)
  : MultiDofJoint<2>::Properties(_multiDofProperties),
    UniversalJointUniqueProperties(_universalProperties)
{
  // Do nothing
}

//==============================================================================
void UniversalJointAddon::setAxis1(const Eigen::Vector3d& _axis)
{
  mProperties.mAxis[0] = _axis.normalized();
  notifyPropertiesUpdate();
}

//==============================================================================
const Eigen::Vector3d& UniversalJointAddon::getAxis1() const
{
  return mProperties.mAxis[0];
}

//==============================================================================
void UniversalJointAddon::setAxis2(const Eigen::Vector3d& _axis)
{
  mProperties.mAxis[1] = _axis.normalized();
  notifyPropertiesUpdate();
}

//==============================================================================
const Eigen::Vector3d& UniversalJointAddon::getAxis2() const
{
  return mProperties.mAxis[1];
}

} // namespace detail
} // namespace dynamics
} // namespace dart
