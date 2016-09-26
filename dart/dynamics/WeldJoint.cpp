/*
 * Copyright (c) 2013-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2013-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "dart/dynamics/WeldJoint.hpp"

#include <string>

#include "dart/math/Helpers.hpp"
#include "dart/math/Geometry.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
WeldJoint::Properties::Properties(const Joint::Properties& _properties)
  : ZeroDofJoint::Properties(_properties)
{
  // Do nothing
}

//==============================================================================
WeldJoint::~WeldJoint()
{
  // Do nothing
}

//==============================================================================
const std::string& WeldJoint::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& WeldJoint::getStaticType()
{
  static const std::string name = "WeldJoint";
  return name;
}

//==============================================================================
bool WeldJoint::isCyclic(std::size_t /*_index*/) const
{
  return false;
}

//==============================================================================
WeldJoint::Properties WeldJoint::getWeldJointProperties() const
{
  return getZeroDofJointProperties();
}

//==============================================================================
void WeldJoint::setTransformFromParentBodyNode(const Eigen::Isometry3d& _T)
{
  Joint::setTransformFromParentBodyNode(_T);

  mT = Joint::mAspectProperties.mT_ParentBodyToJoint * Joint::mAspectProperties.mT_ChildBodyToJoint.inverse();
}

//==============================================================================
void WeldJoint::setTransformFromChildBodyNode(const Eigen::Isometry3d& _T)
{
  Joint::setTransformFromChildBodyNode(_T);

  mT = Joint::mAspectProperties.mT_ParentBodyToJoint * Joint::mAspectProperties.mT_ChildBodyToJoint.inverse();
}

//==============================================================================
WeldJoint::WeldJoint(const Properties& properties)
{
  // Inherited Aspects must be created in the final joint class or else we
  // get pure virtual function calls
  createJointAspect(properties);
}

//==============================================================================
Joint* WeldJoint::clone() const
{
  return new WeldJoint(getWeldJointProperties());
}

//==============================================================================
void WeldJoint::updateRelativeTransform() const
{
  // Do nothing
}

//==============================================================================
void WeldJoint::updateRelativeSpatialVelocity() const
{
  // Do nothing
  // Should we have mSpatialVelocity.setZero() here instead?
}

//==============================================================================
void WeldJoint::updateRelativeSpatialAcceleration() const
{
  // Do nothing
  // Should we have mSpatialAcceleration.setZero() here instead?
}

//==============================================================================
void WeldJoint::updateRelativePrimaryAcceleration() const
{
  // Do nothing
}

//==============================================================================
void WeldJoint::updateRelativeJacobian(bool) const
{
  // Do nothing
}

//==============================================================================
void WeldJoint::updateRelativeJacobianTimeDeriv() const
{
  // Do nothing
}

}  // namespace dynamics
}  // namespace dart
