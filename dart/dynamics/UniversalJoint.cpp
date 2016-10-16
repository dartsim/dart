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

#include "dart/dynamics/UniversalJoint.hpp"

#include <string>

#include "dart/math/Helpers.hpp"
#include "dart/math/Geometry.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
UniversalJoint::~UniversalJoint()
{
  // Do nothing
}

//==============================================================================
void UniversalJoint::setProperties(const Properties& _properties)
{
  GenericJoint<math::R2Space>::setProperties(
        static_cast<const GenericJoint<math::R2Space>::Properties&>(
          _properties));
  setProperties(static_cast<const UniqueProperties&>(_properties));
}

//==============================================================================
void UniversalJoint::setProperties(const UniqueProperties& _properties)
{
  setAspectProperties(_properties);
}

//==============================================================================
void UniversalJoint::setAspectProperties(const AspectProperties& properties)
{
  setAxis1(properties.mAxis[0]);
  setAxis2(properties.mAxis[1]);
}

//==============================================================================
UniversalJoint::Properties UniversalJoint::getUniversalJointProperties() const
{
  return Properties(getGenericJointProperties(), mAspectProperties);
}

//==============================================================================
void UniversalJoint::copy(const UniversalJoint& _otherJoint)
{
  if(this == &_otherJoint)
    return;

  setProperties(_otherJoint.getUniversalJointProperties());
}

//==============================================================================
void UniversalJoint::copy(const UniversalJoint* _otherJoint)
{
  if(nullptr == _otherJoint)
    return;

  copy(*this);
}

//==============================================================================
UniversalJoint& UniversalJoint::operator=(const UniversalJoint& _otherJoint)
{
  copy(_otherJoint);
  return *this;
}

//==============================================================================
const std::string& UniversalJoint::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& UniversalJoint::getStaticType()
{
  static const std::string name = "UniversalJoint";
  return name;
}

//==============================================================================
bool UniversalJoint::isCyclic(std::size_t _index) const
{
  return !hasPositionLimit(_index);
}

//==============================================================================
void UniversalJoint::setAxis1(const Eigen::Vector3d& _axis)
{
  mAspectProperties.mAxis[0] = _axis;
  Joint::notifyPositionUpdate();
  Joint::incrementVersion();
}

//==============================================================================
void UniversalJoint::setAxis2(const Eigen::Vector3d& _axis)
{
  mAspectProperties.mAxis[1] = _axis;
  Joint::notifyPositionUpdate();
  Joint::incrementVersion();
}

//==============================================================================
const Eigen::Vector3d& UniversalJoint::getAxis1() const
{
  return mAspectProperties.mAxis[0];
}

//==============================================================================
const Eigen::Vector3d& UniversalJoint::getAxis2() const
{
  return mAspectProperties.mAxis[1];
}

//==============================================================================
Eigen::Matrix<double, 6, 2> UniversalJoint::getRelativeJacobianStatic(
    const Eigen::Vector2d& _positions) const
{
  Eigen::Matrix<double, 6, 2> J;
  J.col(0) = math::AdTAngular(
               Joint::mAspectProperties.mT_ChildBodyToJoint
               * math::expAngular(-getAxis2() * _positions[1]), getAxis1());
  J.col(1) = math::AdTAngular(Joint::mAspectProperties.mT_ChildBodyToJoint, getAxis2());
  assert(!math::isNan(J));
  return J;
}

//==============================================================================
UniversalJoint::UniversalJoint(const Properties& properties)
  : detail::UniversalJointBase(properties)
{
  // Inherited Aspects must be created in the final joint class in reverse order
  // or else we get pure virtual function calls
  createUniversalJointAspect(properties);
  createGenericJointAspect(properties);
  createJointAspect(properties);
}

//==============================================================================
Joint* UniversalJoint::clone() const
{
  return new UniversalJoint(getUniversalJointProperties());
}

//==============================================================================
void UniversalJoint::updateDegreeOfFreedomNames()
{
  if(!mDofs[0]->isNamePreserved())
    mDofs[0]->setName(Joint::mAspectProperties.mName + "_1", false);
  if(!mDofs[1]->isNamePreserved())
    mDofs[1]->setName(Joint::mAspectProperties.mName + "_2", false);
}

//==============================================================================
void UniversalJoint::updateRelativeTransform() const
{
  const Eigen::Vector2d& positions = getPositionsStatic();
  mT = Joint::mAspectProperties.mT_ParentBodyToJoint
       * Eigen::AngleAxisd(positions[0], getAxis1())
       * Eigen::AngleAxisd(positions[1], getAxis2())
       * Joint::mAspectProperties.mT_ChildBodyToJoint.inverse();
  assert(math::verifyTransform(mT));
}

//==============================================================================
void UniversalJoint::updateRelativeJacobian(bool) const
{
  mJacobian = getRelativeJacobianStatic(getPositionsStatic());
}

//==============================================================================
void UniversalJoint::updateRelativeJacobianTimeDeriv() const
{
  Eigen::Vector6d tmpV1 = getRelativeJacobianStatic().col(1)
                        * getVelocitiesStatic()[1];

  Eigen::Isometry3d tmpT = math::expAngular(
        -getAxis2() * getPositionsStatic()[1]);

  Eigen::Vector6d tmpV2
      = math::AdTAngular(Joint::mAspectProperties.mT_ChildBodyToJoint * tmpT, getAxis1());

  mJacobianDeriv.col(0) = -math::ad(tmpV1, tmpV2);

  assert(!math::isNan(mJacobianDeriv.col(0)));
  assert(mJacobianDeriv.col(1) == Eigen::Vector6d::Zero());
}

}  // namespace dynamics
}  // namespace dart
