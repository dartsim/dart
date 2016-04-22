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

#include "dart/dynamics/PrismaticJoint.h"

#include <string>

#include "dart/math/Geometry.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/BodyNode.h"

namespace dart {
namespace dynamics {

//==============================================================================
PrismaticJoint::~PrismaticJoint()
{
  // Do nothing
}

//==============================================================================
void PrismaticJoint::setProperties(const Properties& _properties)
{
  GeometricJoint<math::R1Space>::setProperties(
        static_cast<const GeometricJoint<math::R1Space>::Properties&>(_properties));
  setProperties(static_cast<const UniqueProperties&>(_properties));
}

//==============================================================================
void PrismaticJoint::setProperties(const UniqueProperties& _properties)
{
  setAspectProperties(_properties);
}

//==============================================================================
void PrismaticJoint::setAspectProperties(const AspectProperties& properties)
{
  setAxis(properties.mAxis);
}

//==============================================================================
PrismaticJoint::Properties PrismaticJoint::getPrismaticJointProperties() const
{
  return Properties(getGeometricJointProperties(), mAspectProperties);
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
bool PrismaticJoint::isCyclic(std::size_t /*_index*/) const
{
  return false;
}

//==============================================================================
void PrismaticJoint::setAxis(const Eigen::Vector3d& _axis)
{
  if(_axis == mAspectProperties.mAxis)
    return;

  mAspectProperties.mAxis = _axis.normalized();
  Joint::notifyPositionUpdate();
  updateLocalJacobian();
  Joint::incrementVersion();
}

//==============================================================================
const Eigen::Vector3d& PrismaticJoint::getAxis() const
{
  return mAspectProperties.mAxis;
}

//==============================================================================
const GeometricJoint<math::R1Space>::JacobianMatrix
PrismaticJoint::getLocalJacobianStatic(
    const GeometricJoint<math::R1Space>::Vector& /*positions*/) const
{
  GeometricJoint<math::R1Space>::JacobianMatrix jacobian
      = math::AdTLinear(Joint::mAspectProperties.mT_ChildBodyToJoint,
                        getAxis());

  // Verification
  assert(!math::isNan(jacobian));

  return jacobian;
}

//==============================================================================
PrismaticJoint::PrismaticJoint(const Properties& properties)
  : detail::PrismaticJointBase(properties)
{
  // Inherited Aspects must be created in the final joint class in reverse order
  // or else we get pure virtual function calls
  createPrismaticJointAspect(properties);
  createGeometricJointAspect(properties);
  createJointAspect(properties);
}

//==============================================================================
Joint* PrismaticJoint::clone() const
{
  return new PrismaticJoint(getPrismaticJointProperties());
}

//==============================================================================
void PrismaticJoint::updateDegreeOfFreedomNames()
{
  // Same name as the joint it belongs to.
  if (!mDofs[0]->isNamePreserved())
    mDofs[0]->setName(Joint::mAspectProperties.mName, false);
}

//==============================================================================
void PrismaticJoint::updateLocalTransform() const
{
  mT = Joint::mAspectProperties.mT_ParentBodyToJoint
       * Eigen::Translation3d(getAxis() * getPositionsStatic())
       * Joint::mAspectProperties.mT_ChildBodyToJoint.inverse();

  // Verification
  assert(math::verifyTransform(mT));
}

//==============================================================================
void PrismaticJoint::updateLocalJacobian(bool _mandatory) const
{
  if(_mandatory)
    mJacobian = getLocalJacobianStatic(getPositionsStatic());
}

//==============================================================================
void PrismaticJoint::updateLocalJacobianTimeDeriv() const
{
  // Time derivative of prismatic joint is always zero
  assert(mJacobianDeriv == Eigen::Vector6d::Zero());
}

}  // namespace dynamics
}  // namespace dart
