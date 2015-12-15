/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
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

#include "dart/dynamics/ScrewJoint.h"

#include <string>

#include "dart/math/Geometry.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/BodyNode.h"

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
  SingleDofJoint::setProperties(
        static_cast<const SingleDofJoint::Properties&>(_properties));
  setProperties(static_cast<const UniqueProperties&>(_properties));
}

//==============================================================================
void ScrewJoint::setProperties(const UniqueProperties& _properties)
{
  setAxis(_properties.mAxis);
  setPitch(_properties.mPitch);
}

//==============================================================================
ScrewJoint::Properties ScrewJoint::getScrewJointProperties() const
{
  return Properties(getSingleDofJointProperties(),
                    getScrewJointAddon()->getProperties());
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
bool ScrewJoint::isCyclic(size_t _index) const
{
  return false;
}

//==============================================================================
void ScrewJoint::setAxis(const Eigen::Vector3d& _axis)
{
  getScrewJointAddon()->setAxis(_axis);
}

//==============================================================================
const Eigen::Vector3d& ScrewJoint::getAxis() const
{
  return getScrewJointAddon()->getAxis();
}

//==============================================================================
void ScrewJoint::setPitch(double _pitch)
{
  getScrewJointAddon()->setPitch(_pitch);
}

//==============================================================================
double ScrewJoint::getPitch() const
{
  return getScrewJointAddon()->getPitch();
}

//==============================================================================
ScrewJoint::ScrewJoint(const Properties& _properties)
  : detail::ScrewJointBase(_properties, common::NoArg)
{
  createScrewJointAddon(_properties);

  // Inherited Joint Properties must be set in the final joint class or else we
  // get pure virtual function calls
  SingleDofJoint::setProperties(_properties);
}

//==============================================================================
Joint* ScrewJoint::clone() const
{
  return new ScrewJoint(getScrewJointProperties());
}

//==============================================================================
void ScrewJoint::updateLocalTransform() const
{
  Eigen::Vector6d S = Eigen::Vector6d::Zero();
  S.head<3>() = getAxis();
  S.tail<3>() = getAxis()*getPitch()/DART_2PI;
  mT = mJointP.mT_ParentBodyToJoint
       * math::expMap(S * getPositionStatic())
       * mJointP.mT_ChildBodyToJoint.inverse();
  assert(math::verifyTransform(mT));
}

//==============================================================================
void ScrewJoint::updateLocalJacobian(bool _mandatory) const
{
  if(_mandatory)
  {
    Eigen::Vector6d S = Eigen::Vector6d::Zero();
    S.head<3>() = getAxis();
    S.tail<3>() = getAxis()*getPitch()/DART_2PI;
    mJacobian = math::AdT(mJointP.mT_ChildBodyToJoint, S);
    assert(!math::isNan(mJacobian));
  }
}

//==============================================================================
void ScrewJoint::updateLocalJacobianTimeDeriv() const
{
  // Time derivative of screw joint is always zero
  assert(mJacobianDeriv == Eigen::Vector6d::Zero());
}

}  // namespace dynamics
}  // namespace dart
