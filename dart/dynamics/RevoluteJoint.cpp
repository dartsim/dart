/*
 * Copyright (c) 2013-2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#include "dart/dynamics/RevoluteJoint.h"

#include <string>

#include "dart/common/Console.h"
#include "dart/math/Geometry.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/BodyNode.h"

namespace dart {
namespace dynamics {

//==============================================================================
RevoluteJoint::~RevoluteJoint()
{
  // Do nothing
}

//==============================================================================
void RevoluteJoint::setProperties(const Properties& _properties)
{
  GeometricJoint<math::RealSpace>::setProperties(
        static_cast<const GeometricJoint<math::RealSpace>::Properties&>(_properties));
  setProperties(static_cast<const UniqueProperties&>(_properties));
}

//==============================================================================
void RevoluteJoint::setProperties(const UniqueProperties& _properties)
{
  setAspectProperties(_properties);
}

//==============================================================================
void RevoluteJoint::setAspectProperties(const AspectProperties& properties)
{
  setAxis(properties.mAxis);
}

//==============================================================================
RevoluteJoint::Properties RevoluteJoint::getRevoluteJointProperties() const
{
  return Properties(getGeometricJointProperties(), mAspectProperties);
}

//==============================================================================
void RevoluteJoint::copy(const RevoluteJoint& _otherJoint)
{
  if(this == &_otherJoint)
    return;

  setProperties(_otherJoint.getRevoluteJointProperties());
}

//==============================================================================
void RevoluteJoint::copy(const RevoluteJoint* _otherJoint)
{
  if(nullptr == _otherJoint)
    return;

  copy(*_otherJoint);
}

//==============================================================================
RevoluteJoint& RevoluteJoint::operator=(const RevoluteJoint& _otherJoint)
{
  copy(_otherJoint);
  return *this;
}

//==============================================================================
const std::string& RevoluteJoint::getType() const
{
  return getStaticType();
}

//==============================================================================
bool RevoluteJoint::isCyclic(std::size_t _index) const
{
  return !hasPositionLimit(_index);
}

//==============================================================================
const std::string& RevoluteJoint::getStaticType()
{
  static const std::string name = "RevoluteJoint";
  return name;
}

//==============================================================================
void RevoluteJoint::setAxis(const Eigen::Vector3d& _axis)
{
  if(_axis == mAspectProperties.mAxis)
    return;

  mAspectProperties.mAxis = _axis.normalized();
  Joint::notifyPositionUpdate();
  updateLocalJacobian();
  Joint::incrementVersion();
}

//==============================================================================
const Eigen::Vector3d& RevoluteJoint::getAxis() const
{
  return mAspectProperties.mAxis;
}

//==============================================================================
const GeometricJoint<math::RealSpace>::JacobianMatrix
RevoluteJoint::getLocalJacobianStatic(
    const GeometricJoint<math::RealSpace>::Vector& /*positions*/) const
{
  GeometricJoint<math::RealSpace>::JacobianMatrix jacobian
      = math::AdTAngular(
        Joint::mAspectProperties.mT_ChildBodyToJoint, getAxis());

  // Verification
  assert(!math::isNan(jacobian));

  return jacobian;
}

//==============================================================================
RevoluteJoint::RevoluteJoint(const Properties& properties)
  : detail::RevoluteJointBase(properties)
{
  // Inherited Aspects must be created in the final joint class in reverse order
  // or else we get pure virtual function calls
  createRevoluteJointAspect(properties);
  createGeometricJointAspect(properties);
  createJointAspect(properties);
}

//==============================================================================
Joint* RevoluteJoint::clone() const
{
  return new RevoluteJoint(getRevoluteJointProperties());
}

//==============================================================================
void RevoluteJoint::updateDegreeOfFreedomNames()
{
  // Same name as the joint it belongs to.
  if (!mDofs[0]->isNamePreserved())
    mDofs[0]->setName(Joint::mAspectProperties.mName, false);
}

//==============================================================================
void RevoluteJoint::updateLocalTransform() const
{
  mT = Joint::mAspectProperties.mT_ParentBodyToJoint
       * math::expAngular(getAxis() * getPositionsStatic())
       * Joint::mAspectProperties.mT_ChildBodyToJoint.inverse();

  // Verification
  assert(math::verifyTransform(mT));
}

//==============================================================================
void RevoluteJoint::updateLocalJacobian(bool _mandatory) const
{
  if(_mandatory)
    mJacobian = getLocalJacobianStatic(getPositionsStatic());
}

//==============================================================================
void RevoluteJoint::updateLocalJacobianTimeDeriv() const
{
  // Time derivative of revolute joint is always zero
  assert(mJacobianDeriv == Eigen::Vector6d::Zero());
}

}  // namespace dynamics
}  // namespace dart
