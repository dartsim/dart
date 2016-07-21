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

#include "dart/dynamics/RevoluteJoint.hpp"

#include <string>

#include "dart/common/Console.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/dynamics/BodyNode.hpp"

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
  GenericJoint<math::R1Space>::setProperties(
        static_cast<const GenericJoint<math::R1Space>::Properties&>(_properties));
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
  return Properties(getGenericJointProperties(), mAspectProperties);
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
  updateRelativeJacobian();
  Joint::incrementVersion();
}

//==============================================================================
const Eigen::Vector3d& RevoluteJoint::getAxis() const
{
  return mAspectProperties.mAxis;
}

//==============================================================================
GenericJoint<math::R1Space>::JacobianMatrix
RevoluteJoint::getRelativeJacobianStatic(
    const GenericJoint<math::R1Space>::Vector& /*positions*/) const
{
  GenericJoint<math::R1Space>::JacobianMatrix jacobian
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
  createGenericJointAspect(properties);
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
void RevoluteJoint::updateRelativeTransform() const
{
  mT = Joint::mAspectProperties.mT_ParentBodyToJoint
       * math::expAngular(getAxis() * getPositionsStatic())
       * Joint::mAspectProperties.mT_ChildBodyToJoint.inverse();

  // Verification
  assert(math::verifyTransform(mT));
}

//==============================================================================
void RevoluteJoint::updateRelativeJacobian(bool _mandatory) const
{
  if(_mandatory)
    mJacobian = getRelativeJacobianStatic(getPositionsStatic());
}

//==============================================================================
void RevoluteJoint::updateRelativeJacobianTimeDeriv() const
{
  // Time derivative of revolute joint is always zero
  assert(mJacobianDeriv == Eigen::Vector6d::Zero());
}

}  // namespace dynamics
}  // namespace dart
