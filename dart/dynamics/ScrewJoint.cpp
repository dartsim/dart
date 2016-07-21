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

#include "dart/dynamics/ScrewJoint.hpp"

#include <string>

#include "dart/math/Geometry.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/dynamics/BodyNode.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
ScrewJoint::~ScrewJoint()
{
  // Do nothing
}

//==============================================================================
void ScrewJoint::setProperties(const Properties& _properties)
{
  GenericJoint<math::R1Space>::setProperties(
        static_cast<const GenericJoint<math::R1Space>::Properties&>(_properties));
  setProperties(static_cast<const UniqueProperties&>(_properties));
}

//==============================================================================
void ScrewJoint::setProperties(const UniqueProperties& _properties)
{
  setAspectProperties(_properties);
}

//==============================================================================
void ScrewJoint::setAspectProperties(const AspectProperties& properties)
{
  setAxis(properties.mAxis);
  setPitch(properties.mPitch);
}

//==============================================================================
ScrewJoint::Properties ScrewJoint::getScrewJointProperties() const
{
  return Properties(getGenericJointProperties(), mAspectProperties);
}

//==============================================================================
void ScrewJoint::copy(const ScrewJoint& _otherJoint)
{
  if(this == &_otherJoint)
    return;

  setProperties(_otherJoint.getScrewJointProperties());
}

//==============================================================================
void ScrewJoint::copy(const ScrewJoint* _otherJoint)
{
  if(nullptr == _otherJoint)
    return;

  copy(*_otherJoint);
}

//==============================================================================
ScrewJoint& ScrewJoint::operator=(const ScrewJoint& _otherJoint)
{
  copy(_otherJoint);
  return *this;
}

//==============================================================================
const std::string& ScrewJoint::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& ScrewJoint::getStaticType()
{
  static const std::string name = "ScrewJoint";
  return name;
}

//==============================================================================
bool ScrewJoint::isCyclic(std::size_t /*_index*/) const
{
  return false;
}

//==============================================================================
void ScrewJoint::setAxis(const Eigen::Vector3d& _axis)
{
  if(_axis == mAspectProperties.mAxis)
    return;

  mAspectProperties.mAxis = _axis.normalized();
  Joint::notifyPositionUpdate();
  updateRelativeJacobian();
  Joint::incrementVersion();
}

//==============================================================================
const Eigen::Vector3d& ScrewJoint::getAxis() const
{
  return mAspectProperties.mAxis;
}

//==============================================================================
void ScrewJoint::setPitch(double _pitch)
{
  if(_pitch == mAspectProperties.mPitch)
    return;

  mAspectProperties.mPitch = _pitch;
  Joint::notifyPositionUpdate();
  updateRelativeJacobian();
  Joint::incrementVersion();
}

//==============================================================================
double ScrewJoint::getPitch() const
{
  return mAspectProperties.mPitch;
}

//==============================================================================
GenericJoint<math::R1Space>::JacobianMatrix
ScrewJoint::getRelativeJacobianStatic(
    const GenericJoint<math::R1Space>::Vector& /*positions*/) const
{
  using namespace dart::math::suffixes;

  Eigen::Vector6d S = Eigen::Vector6d::Zero();
  S.head<3>() = getAxis();
  S.tail<3>() = getAxis() * getPitch() * 0.5_pi;

  GenericJoint<math::R1Space>::JacobianMatrix jacobian
      = math::AdT(Joint::mAspectProperties.mT_ChildBodyToJoint, S);

  assert(!math::isNan(jacobian));

  return jacobian;
}

//==============================================================================
ScrewJoint::ScrewJoint(const Properties& properties)
  : detail::ScrewJointBase(properties)
{
  // Inherited Aspects must be created in the final joint class in reverse order
  // or else we get pure virtual function calls
  createScrewJointAspect(properties);
  createGenericJointAspect(properties);
  createJointAspect(properties);
}

//==============================================================================
Joint* ScrewJoint::clone() const
{
  return new ScrewJoint(getScrewJointProperties());
}

//==============================================================================
void ScrewJoint::updateDegreeOfFreedomNames()
{
  // Same name as the joint it belongs to.
  if (!mDofs[0]->isNamePreserved())
    mDofs[0]->setName(Joint::mAspectProperties.mName, false);
}

//==============================================================================
void ScrewJoint::updateRelativeTransform() const
{
  using namespace dart::math::suffixes;

  Eigen::Vector6d S = Eigen::Vector6d::Zero();
  S.head<3>() = getAxis();
  S.tail<3>() = getAxis()*getPitch()*0.5_pi;
  mT = Joint::mAspectProperties.mT_ParentBodyToJoint
       * math::expMap(S * getPositionsStatic())
       * Joint::mAspectProperties.mT_ChildBodyToJoint.inverse();
  assert(math::verifyTransform(mT));
}

//==============================================================================
void ScrewJoint::updateRelativeJacobian(bool _mandatory) const
{
  if(_mandatory)
    mJacobian = getRelativeJacobianStatic(getPositionsStatic());
}

//==============================================================================
void ScrewJoint::updateRelativeJacobianTimeDeriv() const
{
  // Time derivative of screw joint is always zero
  assert(mJacobianDeriv == Eigen::Vector6d::Zero());
}

}  // namespace dynamics
}  // namespace dart
