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

#include "kido/dynamics/PrismaticJoint.h"

#include <string>

#include "kido/math/Geometry.h"
#include "kido/math/Helpers.h"
#include "kido/dynamics/BodyNode.h"

namespace kido {
namespace dynamics {

//==============================================================================
PrismaticJoint::UniqueProperties::UniqueProperties(const Eigen::Vector3d& _axis)
  : mAxis(_axis.normalized())
{
  // Do nothing
}

//==============================================================================
PrismaticJoint::Properties::Properties(
    const SingleDofJoint::Properties& _singleDofProperties,
    const PrismaticJoint::UniqueProperties& _prismaticProperties)
  : SingleDofJoint::Properties(_singleDofProperties),
    PrismaticJoint::UniqueProperties(_prismaticProperties)
{
  // Do nothing
}

//==============================================================================
PrismaticJoint::~PrismaticJoint()
{
  // Do nothing
}

//==============================================================================
void PrismaticJoint::setProperties(const Properties& _properties)
{
  SingleDofJoint::setProperties(
        static_cast<const SingleDofJoint::Properties&>(_properties));
  setProperties(static_cast<const UniqueProperties&>(_properties));
}

//==============================================================================
void PrismaticJoint::setProperties(const UniqueProperties& _properties)
{
  setAxis(_properties.mAxis);
}

//==============================================================================
PrismaticJoint::Properties PrismaticJoint::getPrismaticJointProperties() const
{
  return Properties(getSingleDofJointProperties(), mPrismaticP);
}

//==============================================================================
void PrismaticJoint::copy(const PrismaticJoint& _otherJoint)
{
  if(this == &_otherJoint)
    return;

  setProperties(_otherJoint.getPrismaticJointProperties());
}

//==============================================================================
void PrismaticJoint::copy(const PrismaticJoint* _otherJoint)
{
  if(nullptr == _otherJoint)
    return;

  copy(*_otherJoint);
}

//==============================================================================
PrismaticJoint& PrismaticJoint::operator=(const PrismaticJoint& _otherJoint)
{
  copy(_otherJoint);
  return *this;
}

//==============================================================================
const std::string& PrismaticJoint::getType() const
{
    return getStaticType();
}

//==============================================================================
const std::string& PrismaticJoint::getStaticType()
{
  static const std::string name = "PrismaticJoint";
  return name;
}

//==============================================================================
bool PrismaticJoint::isCyclic(size_t _index) const
{
  return false;
}

//==============================================================================
void PrismaticJoint::setAxis(const Eigen::Vector3d& _axis)
{
  mPrismaticP.mAxis = _axis.normalized();
  updateLocalJacobian();
  notifyPositionUpdate();
}

//==============================================================================
const Eigen::Vector3d& PrismaticJoint::getAxis() const
{
  return mPrismaticP.mAxis;
}

//==============================================================================
PrismaticJoint::PrismaticJoint(const Properties& _properties)
  : SingleDofJoint(_properties)
{
  setProperties(_properties);
  updateDegreeOfFreedomNames();
}

//==============================================================================
Joint* PrismaticJoint::clone() const
{
  return new PrismaticJoint(getPrismaticJointProperties());
}

//==============================================================================
void PrismaticJoint::updateLocalTransform() const
{
  mT = mJointP.mT_ParentBodyToJoint
       * Eigen::Translation3d(mPrismaticP.mAxis * getPositionStatic())
       * mJointP.mT_ChildBodyToJoint.inverse();

  // Verification
  assert(math::verifyTransform(mT));
}

//==============================================================================
void PrismaticJoint::updateLocalJacobian(bool _mandatory) const
{
  if(_mandatory)
  {
    mJacobian = math::AdTLinear(mJointP.mT_ChildBodyToJoint, mPrismaticP.mAxis);

    // Verification
    assert(!math::isNan(mJacobian));
  }
}

//==============================================================================
void PrismaticJoint::updateLocalJacobianTimeDeriv() const
{
  // Time derivative of prismatic joint is always zero
  assert(mJacobianDeriv == Eigen::Vector6d::Zero());
}

}  // namespace dynamics
}  // namespace kido
